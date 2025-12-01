#!/usr/bin/env python3
"""
Generate a printable PDF with multiple ArUco markers for extrinsic calibration.

This script creates a page with large ArUco markers suitable for ARKit image tracking.
The markers need to be large enough for reliable detection by both:
1. ARKit's image tracking (which needs high-resolution reference images)
2. OpenCV's ArUco detection on the external camera

Usage:
    python generate_aruco_markers.py                    # Default: 4 markers, 100mm size
    python generate_aruco_markers.py --num-markers 6    # 6 markers
    python generate_aruco_markers.py --marker-size 150  # 150mm markers
    python generate_aruco_markers.py --output markers.pdf --dictionary DICT_4X4_50

Requirements:
    pip install opencv-python-headless reportlab numpy

Output:
    - aruco_markers.pdf: Printable PDF with markers
    - aruco_marker_N.png: Individual marker images for ARKit reference
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    from reportlab.lib.pagesizes import A4, LETTER
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    from reportlab.lib.utils import ImageReader
except ImportError:
    print("Error: reportlab is required for PDF generation.")
    print("Install with: pip install reportlab")
    sys.exit(1)


# ArUco dictionary mappings
ARUCO_DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def generate_aruco_marker(
    marker_id: int, 
    dictionary_name: str = "DICT_4X4_50", 
    marker_size_pixels: int = 1000
) -> np.ndarray:
    """
    Generate a single ArUco marker image.
    
    Args:
        marker_id: The ID of the marker to generate
        dictionary_name: Name of the ArUco dictionary
        marker_size_pixels: Size of the marker in pixels (without border)
    
    Returns:
        numpy array containing the marker image (grayscale)
    """
    if dictionary_name not in ARUCO_DICTIONARIES:
        raise ValueError(f"Unknown dictionary: {dictionary_name}")
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dictionary_name])
    
    # Generate the marker
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_pixels)
    
    return marker_image


def add_white_border(image: np.ndarray, border_ratio: float = 0.1) -> np.ndarray:
    """
    Add a white border around the marker image.
    The border helps with detection by providing contrast.
    
    Args:
        image: The marker image
        border_ratio: Border size as a ratio of marker size
    
    Returns:
        Image with white border
    """
    border_pixels = int(image.shape[0] * border_ratio)
    bordered = cv2.copyMakeBorder(
        image,
        border_pixels, border_pixels, border_pixels, border_pixels,
        cv2.BORDER_CONSTANT,
        value=255  # White border
    )
    return bordered


def save_marker_image(marker_image: np.ndarray, output_path: str, marker_id: int):
    """Save a marker image as PNG for ARKit reference."""
    cv2.imwrite(output_path, marker_image)
    print(f"  Saved marker {marker_id} image: {output_path}")


def create_marker_pdf(
    output_path: str,
    marker_ids: list,
    marker_size_mm: float,
    dictionary_name: str,
    page_size: tuple,
    output_dir: str,
    border_ratio: float = 0.1
):
    """
    Create a PDF with multiple ArUco markers.
    
    Args:
        output_path: Path for the output PDF
        marker_ids: List of marker IDs to include
        marker_size_mm: Physical size of each marker in millimeters
        dictionary_name: ArUco dictionary name
        page_size: Page size tuple (width, height) in points
        output_dir: Directory to save individual marker PNGs
        border_ratio: Border size ratio for markers
    """
    # High-resolution for printing (300 DPI)
    dpi = 300
    marker_size_pixels = int(marker_size_mm / 25.4 * dpi)  # Convert mm to pixels
    
    print(f"\n📄 Generating ArUco markers...")
    print(f"  Dictionary: {dictionary_name}")
    print(f"  Marker size: {marker_size_mm}mm ({marker_size_pixels} pixels at {dpi} DPI)")
    print(f"  Number of markers: {len(marker_ids)}")
    print(f"  Marker IDs: {marker_ids}")
    
    # Calculate page dimensions
    page_width, page_height = page_size
    page_width_mm = page_width / mm
    page_height_mm = page_height / mm
    
    # Calculate total marker size with border
    total_marker_size_mm = marker_size_mm * (1 + 2 * border_ratio)
    
    # Calculate grid layout
    margin_mm = 15  # Page margin in mm
    usable_width_mm = page_width_mm - 2 * margin_mm
    usable_height_mm = page_height_mm - 2 * margin_mm
    
    # Calculate how many markers fit per row/column with spacing
    spacing_mm = 10  # Space between markers
    cols = max(1, int((usable_width_mm + spacing_mm) / (total_marker_size_mm + spacing_mm)))
    rows = max(1, int((usable_height_mm + spacing_mm) / (total_marker_size_mm + spacing_mm)))
    
    markers_per_page = cols * rows
    num_pages = (len(marker_ids) + markers_per_page - 1) // markers_per_page
    
    print(f"  Layout: {cols}x{rows} markers per page, {num_pages} page(s)")
    
    # Create PDF
    c = canvas.Canvas(output_path, pagesize=page_size)
    
    # Generate and place markers
    marker_idx = 0
    for page in range(num_pages):
        if page > 0:
            c.showPage()
        
        # Add page header
        c.setFont("Helvetica-Bold", 14)
        c.drawString(margin_mm * mm, page_height - 10 * mm, 
                     f"ArUco Calibration Markers - {dictionary_name}")
        c.setFont("Helvetica", 10)
        c.drawString(margin_mm * mm, page_height - 18 * mm, 
                     f"Marker size: {marker_size_mm}mm | Page {page + 1}/{num_pages}")
        
        # Calculate starting position (centered on page)
        grid_width_mm = cols * total_marker_size_mm + (cols - 1) * spacing_mm
        grid_height_mm = rows * total_marker_size_mm + (rows - 1) * spacing_mm
        start_x_mm = (page_width_mm - grid_width_mm) / 2
        start_y_mm = page_height_mm - margin_mm - 25 - (page_height_mm - margin_mm * 2 - 25 - grid_height_mm) / 2
        
        for row in range(rows):
            for col in range(cols):
                if marker_idx >= len(marker_ids):
                    break
                
                marker_id = marker_ids[marker_idx]
                
                # Generate marker image
                marker_image = generate_aruco_marker(
                    marker_id, dictionary_name, marker_size_pixels
                )
                
                # Add white border
                bordered_image = add_white_border(marker_image, border_ratio)
                
                # Save individual marker PNG for ARKit
                marker_png_path = os.path.join(output_dir, f"aruco_marker_{marker_id}.png")
                save_marker_image(bordered_image, marker_png_path, marker_id)
                
                # Convert to PIL-compatible format for PDF
                from io import BytesIO
                from PIL import Image
                
                # Convert grayscale to RGB for PIL
                marker_rgb = cv2.cvtColor(bordered_image, cv2.COLOR_GRAY2RGB)
                pil_image = Image.fromarray(marker_rgb)
                
                # Calculate position on page
                x_mm = start_x_mm + col * (total_marker_size_mm + spacing_mm)
                y_mm = start_y_mm - row * (total_marker_size_mm + spacing_mm) - total_marker_size_mm
                
                # Draw marker on PDF
                img_buffer = BytesIO()
                pil_image.save(img_buffer, format='PNG')
                img_buffer.seek(0)
                img_reader = ImageReader(img_buffer)
                
                c.drawImage(
                    img_reader,
                    x_mm * mm, y_mm * mm,
                    total_marker_size_mm * mm, total_marker_size_mm * mm
                )
                
                # Add marker ID label below the marker
                c.setFont("Helvetica", 8)
                label_x = x_mm + total_marker_size_mm / 2
                label_y = y_mm - 4
                c.drawCentredString(label_x * mm, label_y * mm, f"ID: {marker_id}")
                
                marker_idx += 1
            
            if marker_idx >= len(marker_ids):
                break
    
    # Add calibration info page
    c.showPage()
    c.setFont("Helvetica-Bold", 16)
    c.drawString(margin_mm * mm, page_height - 30 * mm, "Extrinsic Calibration Information")
    
    c.setFont("Helvetica", 11)
    info_lines = [
        "",
        f"Dictionary: {dictionary_name}",
        f"Marker physical size: {marker_size_mm} mm",
        f"Marker IDs: {', '.join(map(str, marker_ids))}",
        "",
        "Usage Instructions:",
        "",
        "1. Print this PDF at 100% scale (no scaling/fit to page)",
        "2. Verify marker size with a ruler after printing",
        "3. Mount the markers on a rigid, flat surface",
        "4. Ensure good lighting without reflections",
        "",
        "For ARKit Image Tracking:",
        f"  - Individual marker images saved to: {output_dir}",
        "  - Add these images to ARKit as reference images",
        f"  - Set physical width to {marker_size_mm * (1 + 2 * border_ratio):.1f} mm",
        "",
        "For Extrinsic Calibration:",
        "  - Both ARKit and the external camera will detect the same markers",
        "  - ARKit provides: T_world^marker (marker pose in ARKit world frame)",
        "  - Camera provides: T_camera^marker (marker pose in camera frame)",
        "  - Using head pose from ARKit: T_world^head",
        "  - Solve for: T_head^camera (extrinsic calibration)",
    ]
    
    y = page_height - 50 * mm
    for line in info_lines:
        c.drawString(margin_mm * mm, y, line)
        y -= 14
    
    c.save()
    print(f"\n✅ PDF saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate printable ArUco markers for extrinsic calibration"
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="aruco_markers.pdf",
        help="Output PDF file path"
    )
    parser.add_argument(
        "--output-dir", "-d",
        type=str,
        default=None,
        help="Directory to save individual marker PNGs (default: same as PDF)"
    )
    parser.add_argument(
        "--num-markers", "-n",
        type=int,
        default=4,
        help="Number of markers to generate (default: 4)"
    )
    parser.add_argument(
        "--start-id",
        type=int,
        default=0,
        help="Starting marker ID (default: 0)"
    )
    parser.add_argument(
        "--marker-ids",
        type=str,
        default=None,
        help="Comma-separated list of specific marker IDs (overrides --num-markers)"
    )
    parser.add_argument(
        "--marker-size", "-s",
        type=float,
        default=100,
        help="Marker size in millimeters (default: 100mm)"
    )
    parser.add_argument(
        "--dictionary",
        type=str,
        default="DICT_4X4_50",
        choices=list(ARUCO_DICTIONARIES.keys()),
        help="ArUco dictionary to use (default: DICT_4X4_50)"
    )
    parser.add_argument(
        "--page-size",
        type=str,
        default="A4",
        choices=["A4", "LETTER"],
        help="Page size for PDF (default: A4)"
    )
    parser.add_argument(
        "--border-ratio",
        type=float,
        default=0.1,
        help="White border size as ratio of marker size (default: 0.1)"
    )
    
    args = parser.parse_args()
    
    # Determine output directory
    output_path = Path(args.output).resolve()
    if args.output_dir:
        output_dir = Path(args.output_dir).resolve()
    else:
        output_dir = output_path.parent
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Determine marker IDs
    if args.marker_ids:
        marker_ids = [int(x.strip()) for x in args.marker_ids.split(",")]
    else:
        marker_ids = list(range(args.start_id, args.start_id + args.num_markers))
    
    # Validate marker IDs for dictionary
    dict_enum = ARUCO_DICTIONARIES[args.dictionary]
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_enum)
    
    # Get max ID for the dictionary (approximate)
    max_dict_id = {
        "DICT_4X4_50": 50, "DICT_4X4_100": 100, "DICT_4X4_250": 250, "DICT_4X4_1000": 1000,
        "DICT_5X5_50": 50, "DICT_5X5_100": 100, "DICT_5X5_250": 250, "DICT_5X5_1000": 1000,
        "DICT_6X6_50": 50, "DICT_6X6_100": 100, "DICT_6X6_250": 250, "DICT_6X6_1000": 1000,
        "DICT_7X7_50": 50, "DICT_7X7_100": 100, "DICT_7X7_250": 250, "DICT_7X7_1000": 1000,
        "DICT_ARUCO_ORIGINAL": 1024,
        "DICT_APRILTAG_16h5": 30, "DICT_APRILTAG_25h9": 35,
        "DICT_APRILTAG_36h10": 2320, "DICT_APRILTAG_36h11": 587,
    }.get(args.dictionary, 50)
    
    invalid_ids = [mid for mid in marker_ids if mid < 0 or mid >= max_dict_id]
    if invalid_ids:
        print(f"Error: Invalid marker IDs for {args.dictionary}: {invalid_ids}")
        print(f"Valid range: 0 to {max_dict_id - 1}")
        sys.exit(1)
    
    # Page size
    page_size = A4 if args.page_size == "A4" else LETTER
    
    # Generate the PDF
    create_marker_pdf(
        output_path=str(output_path),
        marker_ids=marker_ids,
        marker_size_mm=args.marker_size,
        dictionary_name=args.dictionary,
        page_size=page_size,
        output_dir=str(output_dir),
        border_ratio=args.border_ratio
    )
    
    print(f"\n📁 Individual marker PNGs saved to: {output_dir}")
    print("\n💡 Next steps:")
    print("   1. Print the PDF at 100% scale")
    print("   2. Add marker PNGs to ARKit as reference images")
    print("   3. Run extrinsic calibration in the app")


if __name__ == "__main__":
    main()
