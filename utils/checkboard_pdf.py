#!/usr/bin/env python3
"""
Generate a letter-size PDF checkerboard for camera calibration.

Usage:
    python make_checkerboard_pdf.py

Output:
    checkerboard_letter.pdf  (in current directory)

Notes:
- OpenCV's chessboard pattern is defined in terms of "inner corners".
- If you want a 9x6 inner-corner pattern (common), that corresponds to
  10 x 7 squares on the printed board.
"""

from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas

def draw_checkerboard_pdf(
    filename="checkerboard_letter.pdf",
    inner_corners_x=9,
    inner_corners_y=6,
    margin_inch=0.5,
):
    # Letter size in points: 1 inch = 72 points
    page_width, page_height = letter

    # Number of squares = inner_corners + 1
    squares_x = inner_corners_x + 1
    squares_y = inner_corners_y + 1

    margin = margin_inch * 72.0  # points

    # Maximum drawable width/height
    draw_width = page_width - 2 * margin
    draw_height = page_height - 2 * margin

    # Square size so that the pattern fits within the drawable area
    square_size = min(draw_width / squares_x, draw_height / squares_y)

    # Actual pattern width/height
    cb_width = square_size * squares_x
    cb_height = square_size * squares_y

    # Center the checkerboard on the page
    origin_x = (page_width - cb_width) / 2.0
    origin_y = (page_height - cb_height) / 2.0

    c = canvas.Canvas(filename, pagesize=letter)

    # Optional: draw border rectangle around pattern (for visual reference)
    c.setLineWidth(1)
    c.rect(origin_x, origin_y, cb_width, cb_height)

    # Draw squares: (0,0) in bottom-left of checkerboard
    for j in range(squares_y):
        for i in range(squares_x):
            # Checker pattern: alternate black/white
            if (i + j) % 2 == 0:
                # Black square
                x = origin_x + i * square_size
                y = origin_y + j * square_size
                c.setFillColorRGB(0, 0, 0)
                c.rect(x, y, square_size, square_size, stroke=0, fill=1)

    # Small text info at the bottom
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica", 10)
    info_text = f"{inner_corners_x}x{inner_corners_y} inner corners " \
                f"({squares_x}x{squares_y} squares), letter-size"
    c.drawString(margin, margin * 0.5, info_text)

    c.showPage()
    c.save()
    print(f"Saved checkerboard PDF to: {filename}")
    print(
        "Important: measure the edge length of one printed square (in meters) "
        "and use that as 'square_size' in calibration."
    )

if __name__ == "__main__":
    draw_checkerboard_pdf(
        filename="checkerboard_letter.pdf",
        inner_corners_x=9,
        inner_corners_y=6,
        margin_inch=0.5,
    )