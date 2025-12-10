#!/usr/bin/env python3
"""
Export Isaac Sim stage to USDZ or USDA format compatible with Apple's USD viewer.

This module handles:
1. Flattening the stage (resolving all references)
2. Converting OmniPBR MDL materials to UsdPreviewSurface
3. Fixing color attribute types (float3 -> color3f)
4. Filtering environments and ground plane
5. Packaging to USDZ (or saving as USDA)

Usage:
    from export_usdz import export_stage_to_usdz
    
    # In Isaac Sim, after scene is set up:
    import omni
    stage = omni.usd.get_context().get_stage()
    
    # Export to USDZ (packaged)
    export_stage_to_usdz(stage, "output.usdz")
    
    # Export to USDA (text, for inspection)
    export_stage_to_usdz(stage, "output.usda")
    
    # Export specific environments without ground
    export_stage_to_usdz(stage, "output.usdz", include_ground=False, env_indices=[0, 1, 2])
"""

import os
import shutil
import tempfile
import urllib.request
import urllib.error
from typing import List, Optional, Set, Tuple, Dict

from pxr import Usd, UsdShade, Sdf, Gf, UsdUtils

# Try to import PIL for texture atlas creation
try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("[export_usdz] Warning: PIL not available, UDIM texture atlas creation disabled")


def _create_udim_atlas(tile_paths: List[str], output_path: str) -> bool:
    """Create a horizontal texture atlas from UDIM tiles.
    
    Stitches tiles 1001, 1002, ... horizontally into a single image.
    Returns True if successful.
    """
    if not HAS_PIL or not tile_paths:
        return False
    
    try:
        # Load all tile images
        images = []
        for path in sorted(tile_paths):
            if os.path.exists(path):
                img = Image.open(path)
                images.append(img)
        
        if not images:
            return False
        
        # Get dimensions (assume all tiles are same size)
        tile_width = images[0].width
        tile_height = images[0].height
        
        # Create atlas (horizontal strip)
        atlas_width = tile_width * len(images)
        atlas_height = tile_height
        atlas = Image.new('RGB', (atlas_width, atlas_height))
        
        # Paste tiles
        for i, img in enumerate(images):
            # Convert to RGB if needed
            if img.mode != 'RGB':
                img = img.convert('RGB')
            atlas.paste(img, (i * tile_width, 0))
        
        # Resize if too large (RealityKit/Metal limit is often 16k)
        max_width = 16384
        if atlas_width > max_width:
            new_width = max_width
            new_height = int(atlas_height * (new_width / atlas_width))
            print(f"[export_usdz] Downscaling atlas from {atlas_width}x{atlas_height} to {new_width}x{new_height}...")
            atlas = atlas.resize((new_width, new_height), Image.LANCZOS)
        
        # Save atlas
        atlas.save(output_path, 'PNG')
        print(f"[export_usdz] Created UDIM atlas: {len(images)} tiles -> {output_path} ({atlas.width}x{atlas.height})")
        return True
        
    except Exception as e:
        print(f"[export_usdz] Warning: Failed to create UDIM atlas: {e}")
        return False


def _remap_udim_uvs_in_layer(layer, num_tiles: int) -> int:
    """Remap UV coordinates in the layer from UDIM space to 0-1 range.
    
    UDIM UVs span 0-N (where N is number of tiles).
    We scale U by 1/N to fit everything in 0-1 range.
    
    Returns count of modified UV attributes.
    """
    if num_tiles <= 1:
        return 0
    
    modified_count = 0
    scale_factor = 1.0 / num_tiles
    
    def process_prim(prim_spec):
        nonlocal modified_count
        
        # Look for UV (st) primvar attributes
        for attr_name in list(prim_spec.attributes.keys()):
            if 'primvars:st' not in attr_name:
                continue
            
            attr_spec = prim_spec.attributes[attr_name]
            if attr_spec is None or attr_spec.default is None:
                continue
            
            # Check if it's an array of float2
            try:
                uv_array = list(attr_spec.default)
                if not uv_array or not hasattr(uv_array[0], '__len__'):
                    continue
                
                # Scale U coordinates
                new_uvs = []
                for uv in uv_array:
                    if len(uv) >= 2:
                        new_u = float(uv[0]) * scale_factor
                        new_v = float(uv[1])  # V stays the same
                        new_uvs.append(Gf.Vec2f(new_u, new_v))
                    else:
                        new_uvs.append(uv)
                
                if modified_count < 5:  # Print first 5 remapped arrays for debug
                    print(f"[export_usdz] Debug UV remap: {uv_array[0]} -> {new_uvs[0]}")
                
                attr_spec.default = new_uvs
                modified_count += 1
                
            except Exception:
                continue
        
        # Recurse into children
        for child_name in list(prim_spec.nameChildren.keys()):
            child_spec = prim_spec.nameChildren[child_name]
            process_prim(child_spec)
    
    # Process all root prims
    for root_name in list(layer.rootPrims.keys()):
        root_prim = layer.rootPrims[root_name]
        process_prim(root_prim)
    
    return modified_count


def _download_remote_texture(url: str, output_dir: str) -> Tuple[str, int]:
    """Download a remote texture to a local file, returning (local_path, num_tiles).
    
    Handles UDIM texture patterns where <UDIM> placeholder needs to be resolved
    to specific tile numbers (1001, 1002, etc.).
    """
    if not url.startswith(('http://', 'https://')):
        return url, 0  # Already local
    
    # Check for UDIM patterns - both raw and URL-encoded versions
    udim_patterns = ['<UDIM>', '%3CUDIM%3E', '%3cudim%3e']
    has_udim = any(pattern in url for pattern in udim_patterns)
    
    if has_udim:
        return _download_udim_texture(url, output_dir, udim_patterns)
    
    # Regular (non-UDIM) texture download
    filename = os.path.basename(url.split('?')[0])
    local_path = os.path.join(output_dir, filename)
    
    try:
        print(f"[export_usdz] Downloading texture: {filename}")
        urllib.request.urlretrieve(url, local_path)
        return local_path, 0
    except Exception as e:
        print(f"[export_usdz] Warning: Failed to download texture {url}: {e}")
        return url, 0  # Return original URL if download fails


def _download_udim_texture(url: str, output_dir: str, udim_patterns: List[str]) -> str:
    """Download UDIM texture tiles by resolving <UDIM> pattern to tile numbers.
    
    UDIM tiles are numbered 1001-1100+ where:
    - 1001 = first tile (U=0, V=0)
    - 1002 = second tile (U=1, V=0)
    - 1011 = (U=0, V=1)
    etc.
    
    Most models only use tile 1001, but we try a range just in case.
    """
    # Common UDIM tile range - most models use just 1001, some use a few more
    udim_tiles = list(range(1001, 1010)) + list(range(1011, 1020))  # 1001-1009, 1011-1019
    
    downloaded_paths = []
    
    for tile in udim_tiles:
        # Replace UDIM pattern with tile number
        tile_url = url
        for pattern in udim_patterns:
            tile_url = tile_url.replace(pattern, str(tile))
        
        # Extract filename for this tile
        filename = os.path.basename(tile_url.split('?')[0])
        local_path = os.path.join(output_dir, filename)
        
        try:
            # Use a HEAD request first to check if the file exists (faster)
            req = urllib.request.Request(tile_url, method='HEAD')
            urllib.request.urlopen(req, timeout=5)
            
            # File exists, download it
            print(f"[export_usdz] Downloading UDIM tile {tile}: {filename}")
            urllib.request.urlretrieve(tile_url, local_path)
            downloaded_paths.append(local_path)
                
        except urllib.error.HTTPError as e:
            if e.code == 404:
                # Tile doesn't exist, this is normal for UDIM - not all tiles are used
                continue
            else:
                print(f"[export_usdz] Warning: HTTP error {e.code} for UDIM tile {tile}")
        except Exception as e:
            # Timeout or other error, skip this tile
            continue
    
    if downloaded_paths:
        print(f"[export_usdz] Downloaded {len(downloaded_paths)} UDIM tile(s)")
        
        # If multiple tiles, create atlas
        if len(downloaded_paths) > 1 and HAS_PIL:
            # Generate atlas filename
            base_url = url
            for pattern in udim_patterns:
                base_url = base_url.replace(pattern, 'atlas')
            atlas_filename = os.path.basename(base_url.split('?')[0])
            atlas_path = os.path.join(output_dir, atlas_filename)
            
            if _create_udim_atlas(downloaded_paths, atlas_path):
                return atlas_path, len(downloaded_paths)
        
        # Return first tile if atlas creation failed or only one tile
        return downloaded_paths[0], len(downloaded_paths)
    else:
        # No tiles found - extract filename for error message
        filename = os.path.basename(url.split('?')[0])
        print(f"[export_usdz] Warning: No UDIM tiles found for {filename}")
        return url, 0  # Return original URL as fallback


def _resolve_all_udim_textures(layer, output_dir: str) -> Tuple[int, int]:
    """Find and resolve ALL UDIM texture references in the layer.
    
    Traverses the entire layer looking for Sdf.AssetPath attributes that contain
    UDIM patterns, downloads the textures (creating atlases if possible), and 
    rewrites the attribute values to point to the local files.
    
    Also remaps UV coordinates if UDIM atlases were created.
    
    Returns tuple of (resolved_texture_count, max_tile_count).
    """
    udim_patterns = ['<UDIM>', '%3CUDIM%3E', '%3cudim%3e']
    resolved_count = 0
    max_tiles = 0
    
    def has_udim_pattern(path_str: str) -> bool:
        return any(pattern in path_str for pattern in udim_patterns)
    
    def resolve_udim_path(url: str) -> Tuple[str, bool, int]:
        """Download UDIM texture and return (local_path, success, num_tiles)."""
        nonlocal max_tiles
        
        if not url.startswith(('http://', 'https://')):
            return url, False, 0
        
        local_path, num_tiles = _download_udim_texture(url, output_dir, udim_patterns)
        
        if num_tiles > max_tiles:
            max_tiles = num_tiles
        
        if local_path != url and not local_path.startswith(('http://', 'https://')):
            # Return just the filename since it's in the same directory
            return os.path.basename(local_path), True, num_tiles
        return url, False, 0
    
    def process_prim_attributes(prim_spec):
        """Process all attributes of a prim for UDIM textures."""
        nonlocal resolved_count
        
        for attr_name in list(prim_spec.attributes.keys()):
            attr_spec = prim_spec.attributes[attr_name]
            if attr_spec is None:
                continue
            
            # Check if this is an asset path attribute
            if attr_spec.typeName == Sdf.ValueTypeNames.Asset:
                if attr_spec.default and isinstance(attr_spec.default, Sdf.AssetPath):
                    original_path = attr_spec.default.path
                    if has_udim_pattern(original_path):
                        new_path, success, _ = resolve_udim_path(original_path)
                        if success:
                            attr_spec.default = Sdf.AssetPath(new_path)
                            resolved_count += 1
    
    def traverse_and_resolve(prim_spec):
        """Recursively traverse prims and resolve UDIM textures."""
        if prim_spec is None:
            return
        
        process_prim_attributes(prim_spec)
        
        for child_name in list(prim_spec.nameChildren.keys()):
            child_spec = prim_spec.nameChildren[child_name]
            traverse_and_resolve(child_spec)
    
    # Traverse all root prims
    for root_name in list(layer.rootPrims.keys()):
        root_prim = layer.rootPrims[root_name]
        traverse_and_resolve(root_prim)
    
    # Remap UV coordinates if we created an atlas
    if max_tiles > 1:
        uv_remapped = _remap_udim_uvs_in_layer(layer, max_tiles)
        if uv_remapped > 0:
            print(f"[export_usdz] Remapped {uv_remapped} UV attributes for {max_tiles}-tile atlas")
    
    return resolved_count, max_tiles


def export_stage_to_usdz(
    stage: Usd.Stage,
    output_path: str,
    include_ground: bool = True,
    env_indices: Optional[List[int]] = None,
) -> str:
    """
    Export an Isaac Sim stage to USDZ or USDA format.
    
    Args:
        stage: The USD stage from Isaac Sim (omni.usd.get_context().get_stage())
        output_path: Output path for the file (.usdz, .usda, or .usdc)
        include_ground: Whether to include ground plane prims (default: True)
        env_indices: List of environment indices to include (default: None = all environments)
                    e.g., [0, 1, 2] will include env_0, env_1, env_2
    
    Returns:
        Path to the created file
    
    Example:
        import omni
        from export_usdz import export_stage_to_usdz
        
        stage = omni.usd.get_context().get_stage()
        
        # Export to USDZ (packaged for Apple devices)
        export_stage_to_usdz(stage, "scene.usdz")
        
        # Export to USDA (text format for inspection)
        export_stage_to_usdz(stage, "scene.usda")
        
        # With filtering
        export_stage_to_usdz(stage, "scene.usdz", include_ground=False, env_indices=[0])
    """
    # Create temp directory for intermediate files
    tmp_dir = tempfile.mkdtemp(prefix="usdz_export_")
    
    # Step 1: Flatten the stage to resolve all references and instances
    # This creates a new stage with all references composed and instances expanded
    tmp_usda_path = os.path.join(tmp_dir, "flattened.usda")
    print(f"[export_usdz] Flattening stage to: {tmp_usda_path}")
    flattened_layer = stage.Flatten()
    flattened_layer.Export(tmp_usda_path)
    
    # Step 2: Process the flattened file
    print("[export_usdz] Processing materials and filtering prims...")
    cleaned_path = _process_usda(
        tmp_usda_path,
        include_ground=include_ground,
        env_indices=env_indices,
        output_dir=tmp_dir,
    )
    
    # Step 3: Output based on file extension
    output_path = os.path.abspath(output_path)
    _, ext = os.path.splitext(output_path.lower())
    
    if ext == ".usdz":
        # Package to USDZ
        print(f"[export_usdz] Creating USDZ package: {output_path}")
        success = UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(cleaned_path), output_path)
        if not success:
            raise RuntimeError(f"Failed to create USDZ package: {output_path}")
    elif ext == ".usda":
        # Copy the cleaned USDA directly
        print(f"[export_usdz] Copying USDA to: {output_path}")
        shutil.copy(cleaned_path, output_path)
    elif ext == ".usdc":
        # Export as binary USD
        print(f"[export_usdz] Exporting USDC to: {output_path}")
        cleaned_stage = Usd.Stage.Open(cleaned_path)
        cleaned_stage.Export(output_path)
    else:
        # Default to USDZ
        print(f"[export_usdz] Unknown extension '{ext}', defaulting to USDZ")
        success = UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(cleaned_path), output_path)
        if not success:
            raise RuntimeError(f"Failed to create USDZ package: {output_path}")
    
    print(f"[export_usdz] Successfully exported: {output_path}")
    return output_path


def _process_usda(
    usda_path: str,
    include_ground: bool = True,
    env_indices: Optional[List[int]] = None,
    output_dir: Optional[str] = None,
) -> str:
    """
    Process a USDA file:
    1. Filter prims based on include_ground and env_indices
    2. Convert MDL materials to UsdPreviewSurface
    3. Fix color attribute types (float3 -> color3f for Apple compatibility)
    4. Download remote textures to local files
    
    Returns path to processed USDA file.
    """
    layer = Sdf.Layer.FindOrOpen(usda_path)
    if layer is None:
        raise RuntimeError(f"Failed to open layer: {usda_path}")
    
    # Determine which env paths to keep
    env_paths_to_keep: Optional[Set[str]] = None
    if env_indices is not None:
        env_paths_to_keep = {f"/World/envs/env_{i}" for i in env_indices}
    
    # Collect prims to remove and shaders to process
    prims_to_remove = []
    mdl_shaders = []  # (shader_path, material_path, diffuse_color)
    preview_shaders_to_fix = []  # prim_specs with float3 diffuseColor
    instanceable_prims = []  # prims with instanceable = true
    over_prims = []  # 'over' prims that should be converted to 'def'
    
    def should_remove_prim(prim_path: str) -> bool:
        """Check if a prim should be removed based on filters."""
        path_str = str(prim_path)
        
        # Filter ground plane
        if not include_ground:
            if "GroundPlane" in path_str or "defaultGroundPlane" in path_str:
                return True
        
        # Filter collision geometry (physics-only, not visual)
        if "/collisions" in path_str.lower():
            return True
        
        # Filter OmniverseKit camera/viewport prims (not part of scene)
        if "OmniverseKit_" in path_str:
            return True
        
        # Filter environments
        if env_paths_to_keep is not None:
            if "/World/envs/env_" in path_str:
                # Check if this path starts with any of the env paths to keep
                for env_path in env_paths_to_keep:
                    if path_str.startswith(env_path + "/") or path_str == env_path:
                        return False  # Keep this prim
                
                # Check if this is a top-level env prim not in our keep list
                parts = path_str.split("/")
                for i, part in enumerate(parts):
                    if part.startswith("env_") and i > 0 and parts[i-1] == "envs":
                        env_full_path = "/".join(parts[:i+1])
                        if env_full_path not in env_paths_to_keep:
                            return True
        
        return False
    
    def extract_diffuse_info(prim_spec):
        """Extract diffuse color and texture from MDL shader inputs."""
        color = Gf.Vec3f(0.5, 0.5, 0.5)  # default gray
        texture_path = None
        found_color = False
        matched_attr = None
        
        # Collect all attributes
        all_attrs = {name: prim_spec.attributes[name] for name in prim_spec.attributes.keys()}
        
        # Look for diffuse texture first
        texture_patterns = ['diffuse_texture', 'albedo_texture', 'basecolor_texture', 'base_color_texture']
        for attr_name, attr_spec in all_attrs.items():
            if not attr_spec or not attr_spec.default:
                continue
            attr_lower = attr_name.lower()
            if any(pattern in attr_lower for pattern in texture_patterns):
                if isinstance(attr_spec.default, Sdf.AssetPath):
                    texture_path = attr_spec.default.path
                    break
        
        # Look for diffuse color - ITERATE PATTERNS FIRST (priority order)
        # This ensures we check diffuse_reflection_color before diffuse_color_constant
        color_patterns = [
            'diffuse_reflection_color',     # OmniPBR main displayed color (TOP PRIORITY)
            'albedo_add_color',             # Some NVIDIA MDL
            'diffuse_tint',                 # Tint applied to texture/color
            'base_color',                   # Standard PBR
            'albedo',                       # Alternative name
            'diffuse_color_constant',       # Usually a neutral base, lower priority
        ]
        
        # For each pattern in priority order, check if any attribute matches
        for pattern in color_patterns:
            if found_color:
                break
            for attr_name, attr_spec in all_attrs.items():
                if not attr_spec or not attr_spec.default:
                    continue
                attr_lower = attr_name.lower()
                # Check if pattern matches and it's not a texture
                if pattern in attr_lower and 'texture' not in attr_lower:
                    val = attr_spec.default
                    if hasattr(val, '__len__') and len(val) >= 3:
                        color = Gf.Vec3f(float(val[0]), float(val[1]), float(val[2]))
                        found_color = True
                        matched_attr = attr_name
                        break
        
        # Debug output showing which attribute matched
        prim_name = str(prim_spec.path).split('/')[-1]
        if found_color and matched_attr:
            print(f"[export_usdz] {prim_name}: matched '{matched_attr}' -> ({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")
        
        # Debug output if no color was found
        if not found_color and not texture_path:
            color_attrs = [a for a in all_attrs.keys() if 'color' in a.lower() or 'albedo' in a.lower() or 'diffuse' in a.lower()]
            if color_attrs:
                print(f"[export_usdz] DEBUG: {prim_name} - no color found. Attrs: {color_attrs[:8]}")
                        
        return color, texture_path
    
    def traverse_prims(prim_spec, parent_removed=False):
        """Recursively traverse and collect prims to process."""
        if prim_spec is None:
            return
        
        path = prim_spec.path
        
        # Check if this prim should be removed
        if should_remove_prim(path):
            prims_to_remove.append(path)
            return  # Don't process children of removed prims
        
        if parent_removed:
            return
        
        # Check for 'over' prims (prototype definitions) - convert to 'def'
        if prim_spec.specifier == Sdf.SpecifierOver:
            # Skip converting internal prototypes to avoid ghost geometry
            if not prim_spec.name.startswith("Flattened_Prototype"):
                over_prims.append(prim_spec)
        
        # Check for instanceable prims - need to remove instanceable flag for RealityKit
        if prim_spec.instanceable:
            instanceable_prims.append(prim_spec)
        
        prim_type = prim_spec.typeName
        
        # Check for MDL shaders and PreviewSurface shaders
        if prim_type == "Shader":
            has_mdl = False
            is_preview_surface = False
            
            for prop_name in list(prim_spec.attributes.keys()):
                if 'mdl' in prop_name.lower():
                    attr_spec = prim_spec.attributes[prop_name]
                    if attr_spec and attr_spec.default:
                        default_val = attr_spec.default
                        if isinstance(default_val, Sdf.AssetPath):
                            if 'OmniPBR.mdl' in default_val.path:
                                has_mdl = True
                                break
                
                if prop_name == "info:id":
                    attr_spec = prim_spec.attributes[prop_name]
                    if attr_spec and attr_spec.default == "UsdPreviewSurface":
                        is_preview_surface = True
            
            if has_mdl:
                parent_path = path.GetParentPath()
                color, texture = extract_diffuse_info(prim_spec)
                # Debug: show extracted color/texture
                prim_name = str(path).split('/')[-1]
                if texture:
                    print(f"[export_usdz] Extracted from {prim_name}: texture={os.path.basename(texture)}")
                else:
                    print(f"[export_usdz] Extracted from {prim_name}: color=({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")
                mdl_shaders.append((path, parent_path, color, texture))
            elif is_preview_surface:
                # Check if diffuseColor needs type fix (float3 -> color3f)
                if "inputs:diffuseColor" in prim_spec.attributes:
                    attr_spec = prim_spec.attributes["inputs:diffuseColor"]
                    if attr_spec and str(attr_spec.typeName) != str(Sdf.ValueTypeNames.Color3f):
                        preview_shaders_to_fix.append(prim_spec)
        
        # Recurse into children
        for child_name in list(prim_spec.nameChildren.keys()):
            child_spec = prim_spec.nameChildren[child_name]
            traverse_prims(child_spec, parent_removed=False)
    
    # Traverse all root prims
    for root_name in list(layer.rootPrims.keys()):
        root_prim = layer.rootPrims[root_name]
        traverse_prims(root_prim)
    
    print(f"[export_usdz] Found {len(prims_to_remove)} prims to remove")
    print(f"[export_usdz] Found {len(mdl_shaders)} MDL shaders to convert")
    print(f"[export_usdz] Found {len(preview_shaders_to_fix)} PreviewSurface shaders to fix")
    print(f"[export_usdz] Found {len(over_prims)} 'over' prims to convert to 'def'")
    print(f"[export_usdz] Found {len(instanceable_prims)} instanceable prims to de-instance")
    
    # Apply changes in a single change block for efficiency
    with Sdf.ChangeBlock():
        # Remove filtered prims (environments, ground)
        for prim_path in prims_to_remove:
            parent_path = prim_path.GetParentPath()
            prim_name = prim_path.name
            parent_spec = layer.GetPrimAtPath(parent_path)
            if parent_spec and prim_name in parent_spec.nameChildren:
                del parent_spec.nameChildren[prim_name]
        
        # Convert MDL shaders to PreviewSurface
        created_count = 0
        global_max_tiles = 0
        for shader_path, material_path, diffuse_color, diffuse_texture in mdl_shaders:
            material_spec = layer.GetPrimAtPath(material_path)
            if not material_spec:
                continue
            
            ps_name = "PreviewSurface"
            # Create UsdPreviewSurface shader
            ps_name = "preview_surface"
            ps_path = material_path.AppendChild(ps_name)
            
            created_count += 1
            print(f"[export_usdz] Creating shader {ps_name} for {material_path}")
            
            ps_spec = Sdf.PrimSpec(material_spec, ps_name, Sdf.SpecifierDef, "Shader")
            ps_spec.typeName = "Shader"
            
            id_attr = Sdf.AttributeSpec(ps_spec, "info:id", Sdf.ValueTypeNames.Token)
            id_attr.default = "UsdPreviewSurface"
            
            # Set diffuse color
            if diffuse_texture:
                # Download remote texture if needed
                texture_path = diffuse_texture
                if output_dir and diffuse_texture.startswith(('http://', 'https://')):
                    local_path, num_tiles = _download_remote_texture(diffuse_texture, output_dir)
                    if num_tiles > global_max_tiles:
                        global_max_tiles = num_tiles
                    
                    if local_path != diffuse_texture:
                        # Use relative path (just filename) since it's in same dir as USDA
                        texture_path = os.path.basename(local_path)
                
                # Create UV texture shader
                tex_name = "diffuseTexture"
                tex_path = material_path.AppendChild(tex_name)
                
                tex_spec = Sdf.PrimSpec(material_spec, tex_name, Sdf.SpecifierDef, "Shader")
                tex_spec.typeName = "Shader"
                
                tex_id = Sdf.AttributeSpec(tex_spec, "info:id", Sdf.ValueTypeNames.Token)
                tex_id.default = "UsdUVTexture"
                
                tex_file = Sdf.AttributeSpec(tex_spec, "inputs:file", Sdf.ValueTypeNames.Asset)
                tex_file.default = Sdf.AssetPath(texture_path)
                
                # Define output
                Sdf.AttributeSpec(tex_spec, "outputs:rgb", Sdf.ValueTypeNames.Float3)
                
                # Connect texture to PreviewSurface
                diffuse_attr = Sdf.AttributeSpec(ps_spec, "inputs:diffuseColor", Sdf.ValueTypeNames.Color3f)
                diffuse_attr.connectionPathList.explicitItems = [tex_path.AppendProperty("outputs:rgb")]
                
                # Setup UV reading
                reader_name = "stReader"
                reader_path = material_path.AppendChild(reader_name)
                
                # Check if stReader already exists (might have been created by another shader)
                if reader_name not in material_spec.nameChildren:
                    reader_spec = Sdf.PrimSpec(material_spec, reader_name, Sdf.SpecifierDef, "Shader")
                    reader_spec.typeName = "Shader"
                    
                    reader_id = Sdf.AttributeSpec(reader_spec, "info:id", Sdf.ValueTypeNames.Token)
                    reader_id.default = "UsdPrimvarReader_float2"
                    
                    reader_var = Sdf.AttributeSpec(reader_spec, "inputs:varname", Sdf.ValueTypeNames.Token)
                    reader_var.default = "st"
                    
                    # Define output
                    Sdf.AttributeSpec(reader_spec, "outputs:result", Sdf.ValueTypeNames.Float2)
                
                # Connect st to texture
                st_input = Sdf.AttributeSpec(tex_spec, "inputs:st", Sdf.ValueTypeNames.Float2)
                st_input.connectionPathList.explicitItems = [reader_path.AppendProperty("outputs:result")]
                
            else:
                # Use constant color
                diffuse_attr = Sdf.AttributeSpec(ps_spec, "inputs:diffuseColor", Sdf.ValueTypeNames.Color3f)
                diffuse_attr.default = diffuse_color
            
            # Set roughness/metallic (default values for now)
            roughness_attr = Sdf.AttributeSpec(ps_spec, "inputs:roughness", Sdf.ValueTypeNames.Float)
            roughness_attr.default = 0.5
            
            metallic_attr = Sdf.AttributeSpec(ps_spec, "inputs:metallic", Sdf.ValueTypeNames.Float)
            metallic_attr.default = 0.0
            
            # Define output
            Sdf.AttributeSpec(ps_spec, "outputs:surface", Sdf.ValueTypeNames.Token)
            
            # Connect PreviewSurface to material outputs
            if "outputs:surface" in material_spec.attributes:
                mat_surface_attr = material_spec.attributes["outputs:surface"]
                mat_surface_attr.connectionPathList.ClearEdits()
                mat_surface_attr.connectionPathList.explicitItems = [ps_path.AppendProperty("outputs:surface")]
            else:
                mat_surface_attr = Sdf.AttributeSpec(material_spec, "outputs:surface", Sdf.ValueTypeNames.Token)
                mat_surface_attr.connectionPathList.explicitItems = [ps_path.AppendProperty("outputs:surface")]
            
            # Delete MDL shader
            shader_name = shader_path.name
            if shader_name in material_spec.nameChildren and shader_name != ps_name:
                del material_spec.nameChildren[shader_name]
            
            # Also delete the 'preview' subfolder if it exists - it contains nested texture shaders
            # with UDIM URLs (diffuseColor_tex, roughness_tex, metallic_tex, normal_tex)
            if "preview" in material_spec.nameChildren:
                del material_spec.nameChildren["preview"]
        
        # Remap UV coordinates if we created an atlas (from MDL processing)
        if global_max_tiles > 1:
            uv_remapped = _remap_udim_uvs_in_layer(layer, global_max_tiles)
            if uv_remapped > 0:
                print(f"[export_usdz] Remapped {uv_remapped} UV attributes for {global_max_tiles}-tile atlas")
        
        # Fix PreviewSurface shaders with wrong diffuseColor type (float3 -> color3f)
        fixed_count = 0
        for prim_spec in preview_shaders_to_fix:
            if "inputs:diffuseColor" in prim_spec.attributes:
                attr_spec = prim_spec.attributes["inputs:diffuseColor"]
                old_value = attr_spec.default
                if old_value is not None and hasattr(old_value, '__len__') and len(old_value) >= 3:
                    color_value = Gf.Vec3f(float(old_value[0]), float(old_value[1]), float(old_value[2]))
                    prim_spec.RemoveProperty(attr_spec)
                    new_attr = Sdf.AttributeSpec(prim_spec, "inputs:diffuseColor", Sdf.ValueTypeNames.Color3f)
                    new_attr.default = color_value
                    fixed_count += 1
            
            # Also fix emissiveColor if present
            if "inputs:emissiveColor" in prim_spec.attributes:
                attr_spec = prim_spec.attributes["inputs:emissiveColor"]
                if str(attr_spec.typeName) != str(Sdf.ValueTypeNames.Color3f):
                    old_value = attr_spec.default
                    if old_value is not None and hasattr(old_value, '__len__') and len(old_value) >= 3:
                        color_value = Gf.Vec3f(float(old_value[0]), float(old_value[1]), float(old_value[2]))
                        prim_spec.RemoveProperty(attr_spec)
                        new_attr = Sdf.AttributeSpec(prim_spec, "inputs:emissiveColor", Sdf.ValueTypeNames.Color3f)
                        new_attr.default = color_value
        
        # Convert 'over' prims to 'def' for RealityKit compatibility
        # RealityKit needs 'def' (definitions) not 'over' (overrides) for prototypes
        over_converted = 0
        for prim_spec in over_prims:
            try:
                prim_spec.specifier = Sdf.SpecifierDef
                over_converted += 1
            except Exception:
                pass
        
        # Remove instanceable flag from prims - RealityKit doesn't handle USD instances well
        # This forces the geometry to be expanded inline
        deinstanced = 0
        for prim_spec in instanceable_prims:
            try:
                prim_spec.instanceable = False
                deinstanced += 1
            except Exception:
                pass
        
        # AFTER de-instancing: Clean up orphan prototype prims at root level
        # These are leftover prototypes (e.g., /visuals) that got de-instanced but left behind
        orphan_prototypes_removed = 0
        pseudo_root = layer.pseudoRoot
        if pseudo_root:
            children_to_remove = []
            for child_name in list(pseudo_root.nameChildren.keys()):
                child_path = "/" + child_name
                # Keep: /World (scene), /Light, etc.
                # Remove: /visuals, /__Prototype_*, or other prototype artifacts
        # AFTER de-instancing: Clean up orphan prototype prims at root level
        # Those are leftover prototypes that got de-instanced but left behind
        orphan_prototypes_removed = 0
        pseudo_root = layer.pseudoRoot
        if pseudo_root:
            children_to_remove = []
            for child_name in list(pseudo_root.nameChildren.keys()):
                child_path = "/" + child_name
                # Keep: /World (scene), /Light, etc.
                # Remove: /visuals, /__Prototype_*, /physicsScene
                # NOTE: Do NOT remove Flattened_Prototype_* as they are needed for geometry, just keep them as 'over'
                if (child_name == "visuals" or 
                    child_name.startswith("__Prototype") or 
                    child_name == "physicsScene"):
                    children_to_remove.append(child_name)
            
            for child_name in children_to_remove:
                try:
                    del pseudo_root.nameChildren[child_name]
                    orphan_prototypes_removed += 1
                except Exception:
                    pass
        
        if orphan_prototypes_removed > 0:
            print(f"[export_usdz] Removed {orphan_prototypes_removed} orphan prototype prims")
    
    print(f"[export_usdz] Created {created_count} PreviewSurface shaders")
    print(f"[export_usdz] Fixed {fixed_count} color attribute types")
    print(f"[export_usdz] Converted {over_converted} 'over' prims to 'def'")
    print(f"[export_usdz] De-instanced {deinstanced} instanceable prims")
    
    # Resolve all UDIM texture references in the layer
    # Resolve all UDIM texture references in the layer
    if output_dir:
        resolved_udim, max_tiles = _resolve_all_udim_textures(layer, output_dir)
        if resolved_udim > 0:
            print(f"[export_usdz] Resolved {resolved_udim} UDIM texture references using max {max_tiles} tiles")
    
    # Export cleaned file
    cleaned_path = usda_path.rsplit('.', 1)[0] + '_cleaned.usda'
    layer.Export(cleaned_path)
    
    return cleaned_path



# Command-line interface
if __name__ == "__main__":
    import sys
    
    def print_usage():
        print("Usage: python export_usdz.py <input.usda> <output.usdz> [options]")
        print("\nOptions:")
        print("  --no-ground          Exclude ground plane from export")
        print("  --envs N,M,...       Only include specified environment indices")
        print("\nExamples:")
        print("  python export_usdz.py scene.usda output.usdz")
        print("  python export_usdz.py scene.usda output.usdz --no-ground")
        print("  python export_usdz.py scene.usda output.usdz --envs 0,1,2")
        print("  python export_usdz.py scene.usda output.usdz --no-ground --envs 0")
    
    if len(sys.argv) < 3:
        print_usage()
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_path = sys.argv[2]
    
    include_ground = True
    env_indices = None
    
    i = 3
    while i < len(sys.argv):
        if sys.argv[i] == "--no-ground":
            include_ground = False
        elif sys.argv[i] == "--envs" and i + 1 < len(sys.argv):
            env_indices = [int(x.strip()) for x in sys.argv[i + 1].split(",")]
            i += 1
        elif sys.argv[i] in ["-h", "--help"]:
            print_usage()
            sys.exit(0)
        i += 1
    
    # Open stage and export
    stage = Usd.Stage.Open(input_path)
    if stage is None:
        print(f"Error: Failed to open: {input_path}")
        sys.exit(1)
    
    export_stage_to_usdz(
        stage,
        output_path,
        include_ground=include_ground,
        env_indices=env_indices,
    )
