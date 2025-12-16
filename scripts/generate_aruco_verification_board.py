#!/usr/bin/env python3
"""
Generate an ArUco tag verification board PDF for intrinsic calibration verification.

The board has ArUco tags arranged in a configurable grid (cols x rows) with known spacing.
This allows verification of calibration by measuring distances between detected tag centers.
"""

import cv2
import numpy as np
import argparse
from pathlib import Path

try:
    from reportlab.lib.pagesizes import A4, LETTER
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    HAS_REPORTLAB = True
except ImportError:
    HAS_REPORTLAB = False


def generate_aruco_board_pdf(
    output_path: str = "aruco_verification_board.pdf",
    tag_size_mm: float = 50.0,
    margin_mm: float = 30.0,
    tag_ids: list = None,
    dictionary_id: int = cv2.aruco.DICT_4X4_50,
    page_size: str = "A4",
    cols: int = 2,
    rows: int = 2
):
    """
    Generate a PDF with ArUco tags in a grid for calibration verification.
    
    Args:
        output_path: Output PDF file path
        tag_size_mm: Size of each ArUco tag in millimeters
        margin_mm: Margin/spacing between tags in millimeters
        tag_ids: List of tag IDs to use (row-major order). If None, uses 0, 1, 2, ...
        dictionary_id: ArUco dictionary to use
        page_size: Page size ("A4" or "LETTER")
        cols: Number of columns in the grid
        rows: Number of rows in the grid
    """
    if not HAS_REPORTLAB:
        print("Error: reportlab is required. Install with: pip install reportlab")
        return None
    
    # Default tag IDs
    if tag_ids is None:
        tag_ids = list(range(cols * rows))
    
    # Ensure we have enough tag IDs
    if len(tag_ids) < cols * rows:
        tag_ids = tag_ids + list(range(len(tag_ids), cols * rows))
    
    # Page setup
    if page_size.upper() == "A4":
        page = A4
    else:
        page = LETTER
    
    page_width, page_height = page
    
    # Calculate board dimensions
    board_width = cols * tag_size_mm * mm + (cols - 1) * margin_mm * mm
    board_height = rows * tag_size_mm * mm + (rows - 1) * margin_mm * mm
    
    # Center the board on the page
    start_x = (page_width - board_width) / 2
    start_y = (page_height - board_height) / 2 + board_height  # PDF origin is bottom-left
    
    # Generate ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    
    # Create PDF
    c = canvas.Canvas(output_path, pagesize=page)
    
    # Title
    c.setFont("Helvetica-Bold", 16)
    c.drawCentredString(page_width / 2, page_height - 30, "ArUco Calibration Verification Board")
    
    # Specifications text
    c.setFont("Helvetica", 10)
    center_dist = tag_size_mm + margin_mm
    specs = [
        f"Grid: {cols}x{rows} ({cols * rows} tags)",
        f"Tag Size: {tag_size_mm:.1f} mm",
        f"Margin Between Tags: {margin_mm:.1f} mm",
        f"Tag IDs: {tag_ids[:cols*rows]}",
        f"Dictionary: DICT_4X4_50",
        f"Center-to-Center Distance (adjacent): {center_dist:.1f} mm",
    ]
    
    y_text = page_height - 50
    for spec in specs:
        c.drawString(50, y_text, spec)
        y_text -= 14
    
    # Draw the ArUco tags in a grid (row-major order)
    tag_size_px = 200  # Resolution for tag generation
    
    for row_idx in range(rows):
        for col_idx in range(cols):
            idx = row_idx * cols + col_idx
            tag_id = tag_ids[idx]
        
        # Generate tag image
        tag_img = cv2.aruco.generateImageMarker(aruco_dict, tag_id, tag_size_px)
        
        # Save temporarily as PNG
        temp_path = f"/tmp/aruco_tag_{tag_id}.png"
        cv2.imwrite(temp_path, tag_img)
        
        # Calculate position
        x = start_x + col_idx * (tag_size_mm * mm + margin_mm * mm)
        y = start_y - (row_idx + 1) * tag_size_mm * mm - row_idx * margin_mm * mm
        
        # Draw tag
        c.drawImage(temp_path, x, y, width=tag_size_mm * mm, height=tag_size_mm * mm)
        
        # Draw border for clarity
        c.setStrokeColorRGB(0.5, 0.5, 0.5)
        c.rect(x, y, tag_size_mm * mm, tag_size_mm * mm, stroke=1, fill=0)
        
        # Label the tag ID
        c.setFont("Helvetica", 8)
        c.drawString(x + 2, y + tag_size_mm * mm + 2, f"ID: {tag_id}")
    
    # NOTE: No measurement lines drawn inside the board area to avoid
    # interfering with ArUco tag detection. Distance info is in the text above.
    
    # Bottom instructions
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica", 9)
    instructions = [
        "Instructions:",
        "1. Print this page at 100% scale (no scaling/fit to page)",
        "2. Verify tag size with a ruler before use",
        "3. Mount on a flat, rigid surface",
        "4. Use verify_intrinsic_calibration.py to test calibration accuracy",
    ]
    
    y_inst = 100
    for inst in instructions:
        c.drawString(50, y_inst, inst)
        y_inst -= 12
    
    c.save()
    print(f"Generated ArUco verification board: {output_path}")
    
    # Also generate a JSON config file for the verification script
    config = {
        "tag_size_m": tag_size_mm / 1000.0,
        "margin_m": margin_mm / 1000.0,
        "tag_ids": tag_ids[:cols*rows],
        "cols": cols,
        "rows": rows,
        "dictionary": "DICT_4X4_50",
        "layout": f"{cols}x{rows}",
        "center_distance_adjacent_m": (tag_size_mm + margin_mm) / 1000.0,
    }
    
    config_path = output_path.replace(".pdf", "_config.json")
    import json
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"Generated config file: {config_path}")
    
    return output_path, config


def generate_aruco_board_image(
    output_path: str = "aruco_verification_board.png",
    tag_size_px: int = 200,
    margin_px: int = 60,
    tag_ids: list = None,
    dictionary_id: int = cv2.aruco.DICT_4X4_50,
    dpi: int = 300,
    tag_size_mm: float = 50.0,
    margin_mm: float = 30.0,
    cols: int = 2,
    rows: int = 2
):
    """
    Generate a PNG image of the ArUco board (alternative to PDF).
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    
    # Default tag IDs
    if tag_ids is None:
        tag_ids = list(range(cols * rows))
    
    # Ensure we have enough tag IDs
    if len(tag_ids) < cols * rows:
        tag_ids = tag_ids + list(range(len(tag_ids), cols * rows))
    
    # Calculate pixel sizes based on DPI if mm values provided
    if tag_size_mm > 0 and dpi > 0:
        tag_size_px = int(tag_size_mm / 25.4 * dpi)
        margin_px = int(margin_mm / 25.4 * dpi)
    
    # Board dimensions
    board_width = cols * tag_size_px + (cols - 1) * margin_px + 2 * margin_px  # Extra border
    board_height = rows * tag_size_px + (rows - 1) * margin_px + 2 * margin_px
    
    # Create white board
    board = np.ones((board_height, board_width), dtype=np.uint8) * 255
    
    for row_idx in range(rows):
        for col_idx in range(cols):
            idx = row_idx * cols + col_idx
            tag_id = tag_ids[idx]
            
            # Generate tag
            tag_img = cv2.aruco.generateImageMarker(aruco_dict, tag_id, tag_size_px)
            
            # Calculate position with border margin
            x = margin_px + col_idx * (tag_size_px + margin_px)
            y = margin_px + row_idx * (tag_size_px + margin_px)
            
            # Place tag
            board[y:y+tag_size_px, x:x+tag_size_px] = tag_img
    
    cv2.imwrite(output_path, board)
    print(f"Generated ArUco verification board image: {output_path}")
    
    return output_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate ArUco verification board for calibration testing")
    parser.add_argument("--output", type=str, default="aruco_verification_board.pdf", help="Output file path")
    parser.add_argument("--tag-size", type=float, default=50.0, help="Tag size in mm")
    parser.add_argument("--margin", type=float, default=30.0, help="Margin between tags in mm")
    parser.add_argument("--tag-ids", type=int, nargs="+", default=None, help="Tag IDs to use (row-major order)")
    parser.add_argument("--cols", type=int, default=2, help="Number of columns")
    parser.add_argument("--rows", type=int, default=2, help="Number of rows")
    parser.add_argument("--page-size", choices=["A4", "LETTER"], default="A4", help="Page size for PDF")
    parser.add_argument("--format", choices=["pdf", "png", "both"], default="pdf", help="Output format")
    
    args = parser.parse_args()
    
    if args.format in ["pdf", "both"]:
        generate_aruco_board_pdf(
            output_path=args.output if args.output.endswith(".pdf") else args.output + ".pdf",
            tag_size_mm=args.tag_size,
            margin_mm=args.margin,
            tag_ids=args.tag_ids,
            page_size=args.page_size,
            cols=args.cols,
            rows=args.rows,
        )
    
    if args.format in ["png", "both"]:
        png_path = args.output.replace(".pdf", ".png") if args.output.endswith(".pdf") else args.output + ".png"
        generate_aruco_board_image(
            output_path=png_path,
            tag_size_mm=args.tag_size,
            margin_mm=args.margin,
            tag_ids=args.tag_ids,
            cols=args.cols,
            rows=args.rows,
        )
