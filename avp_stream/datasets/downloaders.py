"""
Download helpers for cloud storage providers (Dropbox, Google Drive).

Handles converting shared links to direct download URLs and downloading
recording files to local directories.

Requirements:
    - gdown: For Google Drive folder downloads (pip install gdown)
"""

import os
import re
import json
from pathlib import Path
from typing import Optional, Callable
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

from .models import PublicRecording


def _convert_dropbox_url(url: str) -> str:
    """
    Convert a Dropbox shared link to a direct download URL.
    
    Dropbox shared links end with ?dl=0 for web preview.
    Changing to ?dl=1 forces direct download.
    For folders, we need to add ?dl=1 to download as zip.
    """
    # Remove any existing query parameters
    base_url = url.split("?")[0]
    
    # Add direct download parameter
    return f"{base_url}?dl=1"


def _extract_gdrive_folder_id(url: str) -> Optional[str]:
    """Extract folder ID from Google Drive URL."""
    patterns = [
        r"/folders/([a-zA-Z0-9_-]+)",
        r"[?&]id=([a-zA-Z0-9_-]+)",
    ]
    
    for pattern in patterns:
        match = re.search(pattern, url)
        if match:
            return match.group(1)
    return None


def _download_gdrive_folder(url: str, dest_path: Path, quiet: bool = False) -> Path:
    """
    Download a Google Drive folder using gdown.
    
    Args:
        url: Google Drive folder URL or folder ID
        dest_path: Destination directory
        quiet: Suppress gdown output
        
    Returns:
        Path to downloaded folder
    """
    try:
        import gdown
    except ImportError:
        raise RuntimeError(
            "gdown is required for Google Drive folder downloads.\n"
            "Install it with: pip install gdown"
        )
    
    folder_id = _extract_gdrive_folder_id(url)
    if not folder_id:
        raise RuntimeError(f"Could not extract folder ID from URL: {url}")
    
    # Ensure destination exists
    dest_path.mkdir(parents=True, exist_ok=True)
    
    # gdown.download_folder downloads to cwd, so we need to change to dest_path
    original_cwd = os.getcwd()
    
    try:
        os.chdir(dest_path)
        gdown.download_folder(id=folder_id, quiet=quiet, use_cookies=False)
        return dest_path
    finally:
        os.chdir(original_cwd)


def _download_file(
    url: str,
    dest_path: Path,
    progress_callback: Optional[Callable[[int, int], None]] = None
) -> None:
    """
    Download a file from URL to local path.
    
    Args:
        url: URL to download from
        dest_path: Local path to save to
        progress_callback: Optional callback(bytes_downloaded, total_bytes)
    """
    request = Request(url)
    request.add_header("User-Agent", "avp_stream/1.0")
    
    try:
        with urlopen(request, timeout=60) as response:
            total_size = int(response.headers.get("Content-Length", 0))
            downloaded = 0
            chunk_size = 8192
            
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(dest_path, "wb") as f:
                while True:
                    chunk = response.read(chunk_size)
                    if not chunk:
                        break
                    f.write(chunk)
                    downloaded += len(chunk)
                    if progress_callback:
                        progress_callback(downloaded, total_size)
                        
    except HTTPError as e:
        raise RuntimeError(f"Download failed ({e.code}): {e.reason}") from e
    except URLError as e:
        raise RuntimeError(f"Network error: {e.reason}") from e


def download_recording(
    recording: PublicRecording,
    dest_dir: str = "./downloads",
    show_progress: bool = True
) -> Path:
    """
    Download a public recording to local directory.
    
    This handles the conversion of Dropbox and Google Drive shared links
    to direct download URLs automatically.
    
    Args:
        recording: PublicRecording object to download
        dest_dir: Destination directory (default: "./downloads")
        show_progress: Whether to show download progress
        
    Returns:
        Path to the downloaded file/directory
        
    Raises:
        RuntimeError: If download fails
        
    Example:
        >>> from avp_stream.datasets import list_public_recordings, download_recording
        >>> recordings = list_public_recordings()
        >>> if recordings:
        ...     path = download_recording(recordings[0])
        ...     print(f"Downloaded to: {path}")
        
    Note:
        - Dropbox links download the shared folder as a zip file
        - Google Drive folders are downloaded using gdown (requires: pip install gdown)
    """
    dest_path = Path(dest_dir)
    dest_path.mkdir(parents=True, exist_ok=True)
    
    # Create a safe folder name from the title
    safe_title = re.sub(r'[^\w\s-]', '', recording.title).strip()
    safe_title = re.sub(r'[-\s]+', '_', safe_title)
    
    # Handle Google Drive folders with gdown
    if recording.provider in ("googleDrive", "google_drive") and "/folders/" in recording.cloud_url:
        print(f"📂 Downloading Google Drive folder: {recording.title}")
        target_dir = dest_path / safe_title
        downloaded_path = _download_gdrive_folder(
            recording.cloud_url, 
            target_dir,
            quiet=not show_progress
        )
        print(f"✅ Downloaded to: {downloaded_path}")
        return downloaded_path
    
    # Handle Dropbox
    if recording.provider == "dropbox":
        download_url = _convert_dropbox_url(recording.cloud_url)
        filename = f"{safe_title}.zip"
        file_path = dest_path / filename
        
        # Show progress if tqdm is available
        pbar = None
        if show_progress:
            try:
                from tqdm import tqdm
                pbar = tqdm(
                    total=0,
                    unit="B",
                    unit_scale=True,
                    desc=f"Downloading {recording.title}"
                )
                
                def update_progress(downloaded: int, total: int):
                    if pbar.total != total:
                        pbar.total = total
                        pbar.refresh()
                    pbar.n = downloaded
                    pbar.refresh()
            except ImportError:
                print(f"Downloading {recording.title}...")
                update_progress = None
        else:
            update_progress = None
        
        try:
            _download_file(download_url, file_path, update_progress)
            if pbar:
                pbar.close()
            print(f"✅ Downloaded to: {file_path}")
            return file_path
        except Exception as e:
            if pbar:
                pbar.close()
            raise RuntimeError(f"Failed to download {recording.title}: {e}") from e
    
    # For other providers, return the URL
    print(f"⚠️  Provider '{recording.provider}' not supported for direct download.")
    print(f"    URL: {recording.cloud_url}")
    return dest_path
