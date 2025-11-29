#!/usr/bin/env python3
"""
Simple client to convert a MuJoCo scene into USDZ using the local API server.

Notes:
- The repo contains 'scenes/franka_emika_panda/scene_mughang.xml' (no underscore).
- This script will zip the containing directory with assets and POST it to /convert.
- The server must be running (see mujoco_arviewer/converter_server.py).

Usage:
  python converter_client.py \
      --server http://localhost:8000 \
      --xml scenes/franka_emika_panda/scene_mughang.xml \
      --out downloads
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Optional

import requests


def zip_scene_dir(scene_xml: Path) -> tuple[Path, str]:
    """Create a temporary ZIP containing the XML and all assets in its directory.

    Returns (zip_path, xml_relpath_inside_zip)
    """
    if not scene_xml.exists():
        # Helper: if user typed scene_mug_hang.xml, suggest the known file.
        alt = scene_xml.with_name(scene_xml.name.replace("_", ""))
        msg = f"XML not found: {scene_xml}"
        if alt.exists():
            msg += f". Did you mean: {alt}?"
        raise FileNotFoundError(msg)

    base_dir = scene_xml.parent
    xml_relpath = scene_xml.name  # keep XML at zip root

    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".zip")
    tmp_path = Path(tmp.name)
    tmp.close()

    with zipfile.ZipFile(tmp_path, "w", compression=zipfile.ZIP_DEFLATED) as z:
        for p in base_dir.rglob("*"):
            if p.is_file():
                arcname = p.relative_to(base_dir)
                z.write(p, arcname)
    return tmp_path, xml_relpath


def convert_and_download(server: str, scene_xml: Path, out_dir: Path) -> Path:
    zip_path, xml_relpath = zip_scene_dir(scene_xml)

    files = {
        "file": ("bundle.zip", open(zip_path, "rb"), "application/zip"),
    }
    data = {"xml_relpath": xml_relpath}
    try:
        resp = requests.post(f"{server}/convert", files=files, data=data, timeout=300)
        resp.raise_for_status()
        payload = resp.json()
        download_url = payload["download_url"]
        file_name = payload["file_name"]

        # Download the resulting USDZ 
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / xml_relpath.replace(".xml", ".usdz")

        r = requests.get(f"{server}{download_url}", stream=True, timeout=300)
        r.raise_for_status()
        with open(out_path, "wb") as f:
            for chunk in r.iter_content(chunk_size=1024 * 1024):
                if chunk:
                    f.write(chunk)
        return out_path
    finally:
        try:
            os.remove(zip_path)
        except OSError:
            pass


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Client for MuJoCo→USDZ converter API")
    parser.add_argument("--server", default="http://mujoco-usd-convert.xyz", help="Base URL of the converter server")
    parser.add_argument("--xml", default="scenes/franka_emika_panda/scene_mughang.xml", help="Path to the scene XML")
    parser.add_argument("--out", default="downloads", help="Output directory to save USDZ")
    args = parser.parse_args(argv)

    server = args.server.rstrip("/")
    scene_xml = Path(args.xml)
    out_dir = Path(args.out)

    try:
        out_path = convert_and_download(server, scene_xml, out_dir)
    except Exception as e:
        print(f"Conversion failed: {e}")
        return 1

    print(f"✅ Converted and downloaded: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
