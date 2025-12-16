#!/usr/bin/env python3
"""
Prepare Calibration Sheets
==========================

Generates a printable PDF containing all calibration patterns needed for
the Python-centric camera calibration workflow.

Usage:
    python -m avp_stream.prepare_calibration
    python -m avp_stream.prepare_calibration --output my_calibration.pdf

The generated PDF contains:
- Page 1: ChArUco board for intrinsic calibration (7x5 squares, 30mm)
- Page 2: ArUco verification grid (2x2, 50mm tags with 30mm margins)
- Pages 3-5: Large ArUco markers for extrinsic calibration (IDs 11, 12, 13)

IMPORTANT: Print at 100% scale (no scaling/fit-to-page) for accurate calibration!
"""

import cv2
import numpy as np
import argparse
from pathlib import Path

# Default calibration parameters
DEFAULT_CHARUCO_ROWS = 7
DEFAULT_CHARUCO_COLS = 5
DEFAULT_CHARUCO_SQUARE_SIZE = 0.030  # 30mm
DEFAULT_CHARUCO_MARKER_SIZE = 0.023  # 23mm

DEFAULT_VERIFICATION_TAG_SIZE = 0.050  # 50mm
DEFAULT_VERIFICATION_MARGIN = 0.030    # 30mm between tags

DEFAULT_EXTRINSIC_MARKER_SIZE = 0.100  # 100mm (large for easy detection)
EXTRINSIC_MARKER_IDS = [0, 2, 3]    # ArUco IDs for extrinsic calibration


def generate_charuco_page(
    rows: int = DEFAULT_CHARUCO_ROWS,
    cols: int = DEFAULT_CHARUCO_COLS,
    square_size: float = DEFAULT_CHARUCO_SQUARE_SIZE,
    marker_size: float = DEFAULT_CHARUCO_MARKER_SIZE,
    dpi: int = 300,
) -> np.ndarray:
    """
    Generate a ChArUco board image for intrinsic calibration.
    
    Args:
        rows: Number of rows (squares)
        cols: Number of columns (squares)
        square_size: Size of each square in meters
        marker_size: Size of ArUco markers in meters
        dpi: Output resolution
    
    Returns:
        Image array suitable for PDF embedding
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((cols, rows), square_size, marker_size, dictionary)
    
    board_width_m = cols * square_size
    board_height_m = rows * square_size
    
    margin_m = 0.02  # 20mm margin
    img_width_m = board_width_m + 2 * margin_m
    img_height_m = board_height_m + 2 * margin_m
    
    pixels_per_meter = dpi / 0.0254
    img_width_px = int(img_width_m * pixels_per_meter)
    img_height_px = int(img_height_m * pixels_per_meter)
    margin_px = int(margin_m * pixels_per_meter)
    
    img = board.generateImage((img_width_px, img_height_px), marginSize=margin_px)
    
    return img


def generate_verification_page(
    tag_size: float = DEFAULT_VERIFICATION_TAG_SIZE,
    margin: float = DEFAULT_VERIFICATION_MARGIN,
    tag_ids: list = [0, 1, 2, 3],
    dpi: int = 300,
) -> np.ndarray:
    """
    Generate a 2x2 ArUco verification grid.
    
    Args:
        tag_size: Size of each tag in meters
        margin: Margin between tags in meters
        tag_ids: List of 4 ArUco IDs to use
        dpi: Output resolution
    
    Returns:
        Image array suitable for PDF embedding
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    pixels_per_meter = dpi / 0.0254
    tag_size_px = int(tag_size * pixels_per_meter)
    margin_px = int(margin * pixels_per_meter)
    
    # 2x2 grid with margins
    grid_width_px = 2 * tag_size_px + 3 * margin_px
    grid_height_px = 2 * tag_size_px + 3 * margin_px
    
    # Create white background
    img = np.ones((grid_height_px, grid_width_px), dtype=np.uint8) * 255
    
    # Generate and place each tag
    positions = [
        (margin_px, margin_px),  # Top-left
        (2 * margin_px + tag_size_px, margin_px),  # Top-right
        (margin_px, 2 * margin_px + tag_size_px),  # Bottom-left
        (2 * margin_px + tag_size_px, 2 * margin_px + tag_size_px),  # Bottom-right
    ]
    
    for i, (x, y) in enumerate(positions):
        if i < len(tag_ids):
            tag = cv2.aruco.generateImageMarker(dictionary, tag_ids[i], tag_size_px)
            img[y:y + tag_size_px, x:x + tag_size_px] = tag
    
    return img


def generate_extrinsic_marker_page(
    marker_id: int,
    marker_size: float = DEFAULT_EXTRINSIC_MARKER_SIZE,
    dpi: int = 300,
) -> np.ndarray:
    """
    Generate a single large ArUco marker for extrinsic calibration.
    
    Args:
        marker_id: ArUco marker ID
        marker_size: Size of marker in meters
        dpi: Output resolution
    
    Returns:
        Image array suitable for PDF embedding
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    pixels_per_meter = dpi / 0.0254
    marker_size_px = int(marker_size * pixels_per_meter)
    margin_px = int(0.03 * pixels_per_meter)  # 30mm margin
    
    # Generate marker
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size_px)
    
    # Add margin
    img_height = marker_size_px + 2 * margin_px
    img_width = marker_size_px + 2 * margin_px
    img = np.ones((img_height, img_width), dtype=np.uint8) * 255
    img[margin_px:margin_px + marker_size_px, margin_px:margin_px + marker_size_px] = marker
    
    return img


def add_text_to_image(
    img: np.ndarray,
    text: str,
    position: str = "bottom",
    font_scale: float = 0.8,
) -> np.ndarray:
    """Add text annotation to an image."""
    if len(img.shape) == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 2
    color = (0, 0, 0)  # Black text
    
    # Get text size
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    
    # Position text
    if position == "bottom":
        x = (img.shape[1] - text_width) // 2
        y = img.shape[0] - 20
    elif position == "top":
        x = (img.shape[1] - text_width) // 2
        y = text_height + 20
    else:
        x, y = 20, text_height + 20
    
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness)
    return img


def generate_calibration_pdf(
    output_path: str = "calibration_sheets.pdf",
    charuco_rows: int = DEFAULT_CHARUCO_ROWS,
    charuco_cols: int = DEFAULT_CHARUCO_COLS,
    charuco_square: float = DEFAULT_CHARUCO_SQUARE_SIZE,
    charuco_marker: float = DEFAULT_CHARUCO_MARKER_SIZE,
    verification_tag_size: float = DEFAULT_VERIFICATION_TAG_SIZE,
    verification_margin: float = DEFAULT_VERIFICATION_MARGIN,
    extrinsic_marker_size: float = DEFAULT_EXTRINSIC_MARKER_SIZE,
    dpi: int = 300,
):
    """
    Generate a complete calibration PDF with all required patterns.
    
    Args:
        output_path: Output PDF file path
        charuco_*: ChArUco board parameters
        verification_*: Verification grid parameters
        extrinsic_marker_size: Size of extrinsic calibration markers
        dpi: Output resolution
    """
    try:
        from matplotlib.backends.backend_pdf import PdfPages
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: matplotlib is required for PDF generation.")
        print("Install with: pip install matplotlib")
        return
    
    print(f"Generating calibration PDF: {output_path}")
    print(f"  DPI: {dpi}")
    print()
    
    with PdfPages(output_path) as pdf:
        # Page 1: ChArUco board
        print(f"Page 1: ChArUco Board ({charuco_cols}x{charuco_rows}, {charuco_square*1000:.0f}mm squares)")
        charuco_img = generate_charuco_page(
            charuco_rows, charuco_cols, charuco_square, charuco_marker, dpi
        )
        charuco_img = add_text_to_image(
            charuco_img,
            f"Intrinsic Calibration - ChArUco {charuco_cols}x{charuco_rows} - {charuco_square*1000:.0f}mm squares - PRINT AT 100%",
            position="top"
        )
        
        fig, ax = plt.subplots(figsize=(8.5, 11))  # Letter size
        ax.imshow(charuco_img, cmap='gray' if len(charuco_img.shape) == 2 else None, aspect='equal')
        ax.axis('off')
        plt.tight_layout(pad=0)
        pdf.savefig(fig, dpi=dpi)
        plt.close(fig)
        
        # Page 2: Verification grid
        print(f"Page 2: Verification Grid (2x2, {verification_tag_size*1000:.0f}mm tags)")
        verif_img = generate_verification_page(
            verification_tag_size, verification_margin, [0, 1, 2, 3], dpi
        )
        verif_img = add_text_to_image(
            verif_img,
            f"Verification Grid - {verification_tag_size*1000:.0f}mm tags, {verification_margin*1000:.0f}mm margin - PRINT AT 100%",
            position="top"
        )
        
        fig, ax = plt.subplots(figsize=(8.5, 11))
        ax.imshow(verif_img, cmap='gray' if len(verif_img.shape) == 2 else None, aspect='equal')
        ax.axis('off')
        plt.tight_layout(pad=0)
        pdf.savefig(fig, dpi=dpi)
        plt.close(fig)
        
        # Pages 3-5: Extrinsic calibration markers
        for i, marker_id in enumerate(EXTRINSIC_MARKER_IDS):
            print(f"Page {3+i}: Extrinsic Marker ID {marker_id} ({extrinsic_marker_size*1000:.0f}mm)")
            marker_img = generate_extrinsic_marker_page(marker_id, extrinsic_marker_size, dpi)
            marker_img = add_text_to_image(
                marker_img,
                f"Extrinsic Calibration Marker #{i+1} - ID {marker_id} - {extrinsic_marker_size*1000:.0f}mm - PRINT AT 100%",
                position="top"
            )
            
            fig, ax = plt.subplots(figsize=(8.5, 11))
            ax.imshow(marker_img, cmap='gray' if len(marker_img.shape) == 2 else None, aspect='equal')
            ax.axis('off')
            plt.tight_layout(pad=0)
            pdf.savefig(fig, dpi=dpi)
            plt.close(fig)
    
    print()
    print("=" * 60)
    print(f"SUCCESS: Generated {output_path}")
    print("=" * 60)
    print()
    print("PRINTING INSTRUCTIONS:")
    print("  1. Open the PDF and print ALL pages")
    print("  2. IMPORTANT: Set scaling to 100% (no fit-to-page!)")
    print("  3. Use high-quality paper for best results")
    print("  4. Verify sizes with a ruler after printing:")
    print(f"     - ChArUco squares should be {charuco_square*1000:.0f}mm")
    print(f"     - Verification tags should be {verification_tag_size*1000:.0f}mm")
    print(f"     - Extrinsic markers should be {extrinsic_marker_size*1000:.0f}mm")
    print()
    print("Next step: Run calibration with:")
    print("  python -m avp_stream.run_calibration")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Generate calibration sheets PDF for camera calibration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m avp_stream.prepare_calibration
  python -m avp_stream.prepare_calibration --output my_calibration.pdf
  python -m avp_stream.prepare_calibration --charuco-square 0.025  # 25mm squares
        """
    )
    
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="calibration_sheets.pdf",
        help="Output PDF file path (default: calibration_sheets.pdf)"
    )
    
    # ChArUco parameters
    parser.add_argument("--charuco-rows", type=int, default=DEFAULT_CHARUCO_ROWS,
                        help=f"ChArUco board rows (default: {DEFAULT_CHARUCO_ROWS})")
    parser.add_argument("--charuco-cols", type=int, default=DEFAULT_CHARUCO_COLS,
                        help=f"ChArUco board columns (default: {DEFAULT_CHARUCO_COLS})")
    parser.add_argument("--charuco-square", type=float, default=DEFAULT_CHARUCO_SQUARE_SIZE,
                        help=f"ChArUco square size in meters (default: {DEFAULT_CHARUCO_SQUARE_SIZE})")
    parser.add_argument("--charuco-marker", type=float, default=DEFAULT_CHARUCO_MARKER_SIZE,
                        help=f"ChArUco marker size in meters (default: {DEFAULT_CHARUCO_MARKER_SIZE})")
    
    # Verification parameters
    parser.add_argument("--verification-tag-size", type=float, default=DEFAULT_VERIFICATION_TAG_SIZE,
                        help=f"Verification tag size in meters (default: {DEFAULT_VERIFICATION_TAG_SIZE})")
    parser.add_argument("--verification-margin", type=float, default=DEFAULT_VERIFICATION_MARGIN,
                        help=f"Verification tag margin in meters (default: {DEFAULT_VERIFICATION_MARGIN})")
    
    # Extrinsic parameters
    parser.add_argument("--extrinsic-marker-size", type=float, default=DEFAULT_EXTRINSIC_MARKER_SIZE,
                        help=f"Extrinsic marker size in meters (default: {DEFAULT_EXTRINSIC_MARKER_SIZE})")
    
    parser.add_argument("--dpi", type=int, default=300,
                        help="Output resolution (default: 300)")
    
    args = parser.parse_args()
    
    generate_calibration_pdf(
        output_path=args.output,
        charuco_rows=args.charuco_rows,
        charuco_cols=args.charuco_cols,
        charuco_square=args.charuco_square,
        charuco_marker=args.charuco_marker,
        verification_tag_size=args.verification_tag_size,
        verification_margin=args.verification_margin,
        extrinsic_marker_size=args.extrinsic_marker_size,
        dpi=args.dpi,
    )


if __name__ == "__main__":
    main()
