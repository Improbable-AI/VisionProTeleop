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
from typing import List, Optional, Set

from pxr import Usd, UsdShade, Sdf, Gf, UsdUtils


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
    
    # Step 1: Export flattened stage
    tmp_usda_path = os.path.join(tmp_dir, "flattened.usda")
    print(f"[export_usdz] Exporting flattened stage to: {tmp_usda_path}")
    stage.Export(tmp_usda_path)
    
    # Step 2: Process the flattened file
    print("[export_usdz] Processing materials and filtering prims...")
    cleaned_path = _process_usda(
        tmp_usda_path,
        include_ground=include_ground,
        env_indices=env_indices,
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
) -> str:
    """
    Process a USDA file:
    1. Filter prims based on include_ground and env_indices
    2. Convert MDL materials to UsdPreviewSurface
    3. Fix color attribute types (float3 -> color3f for Apple compatibility)
    
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
    
    def should_remove_prim(prim_path: str) -> bool:
        """Check if a prim should be removed based on filters."""
        path_str = str(prim_path)
        
        # Filter ground plane
        if not include_ground:
            if "GroundPlane" in path_str or "defaultGroundPlane" in path_str:
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
    
    def extract_diffuse_color(prim_spec) -> Gf.Vec3f:
        """Extract diffuse color from MDL shader inputs."""
        color = Gf.Vec3f(0.5, 0.5, 0.5)  # default gray
        for prop_name in prim_spec.attributes.keys():
            if 'diffuse_color_constant' in prop_name or 'diffuse_color' in prop_name:
                attr_spec = prim_spec.attributes[prop_name]
                if attr_spec and attr_spec.default:
                    val = attr_spec.default
                    if hasattr(val, '__len__') and len(val) >= 3:
                        color = Gf.Vec3f(float(val[0]), float(val[1]), float(val[2]))
                        break
        return color
    
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
                color = extract_diffuse_color(prim_spec)
                mdl_shaders.append((path, parent_path, color))
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
        for shader_path, material_path, diffuse_color in mdl_shaders:
            material_spec = layer.GetPrimAtPath(material_path)
            if not material_spec:
                continue
            
            ps_name = "PreviewSurface"
            if ps_name not in material_spec.nameChildren:
                # Create PreviewSurface shader
                ps_path = material_path.AppendChild(ps_name)
                ps_spec = Sdf.CreatePrimInLayer(layer, ps_path)
                ps_spec.typeName = "Shader"
                ps_spec.specifier = Sdf.SpecifierDef
                
                # Set attributes with correct types for Apple compatibility
                id_attr = Sdf.AttributeSpec(ps_spec, "info:id", Sdf.ValueTypeNames.Token)
                id_attr.default = "UsdPreviewSurface"
                
                # IMPORTANT: Use Color3f, not Float3, for Apple's USD viewer
                diffuse_attr = Sdf.AttributeSpec(ps_spec, "inputs:diffuseColor", Sdf.ValueTypeNames.Color3f)
                diffuse_attr.default = diffuse_color
                
                metallic_attr = Sdf.AttributeSpec(ps_spec, "inputs:metallic", Sdf.ValueTypeNames.Float)
                metallic_attr.default = 0.0
                
                roughness_attr = Sdf.AttributeSpec(ps_spec, "inputs:roughness", Sdf.ValueTypeNames.Float)
                roughness_attr.default = 0.5
                
                Sdf.AttributeSpec(ps_spec, "outputs:surface", Sdf.ValueTypeNames.Token)
                
                created_count += 1
            
            # Connect material outputs:surface to PreviewSurface
            ps_path = material_path.AppendChild(ps_name)
            surface_attr_name = "outputs:surface"
            
            if surface_attr_name in material_spec.attributes:
                mat_surface_attr = material_spec.attributes[surface_attr_name]
                mat_surface_attr.connectionPathList.ClearEdits()
                mat_surface_attr.connectionPathList.explicitItems = [ps_path.AppendProperty("outputs:surface")]
            else:
                mat_surface_attr = Sdf.AttributeSpec(material_spec, surface_attr_name, Sdf.ValueTypeNames.Token)
                mat_surface_attr.connectionPathList.explicitItems = [ps_path.AppendProperty("outputs:surface")]
            
            # Delete MDL shader
            shader_name = shader_path.name
            if shader_name in material_spec.nameChildren and shader_name != ps_name:
                del material_spec.nameChildren[shader_name]
        
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
    
    print(f"[export_usdz] Created {created_count} PreviewSurface shaders")
    print(f"[export_usdz] Fixed {fixed_count} color attribute types")
    
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
