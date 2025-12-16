import cv2
import numpy as np
import argparse
from pathlib import Path

def generate_checkerboard(output_file: str, rows=7, cols=5, square_size=0.030):
    """
    Generate a standard checkerboard pattern using numpy and matplotlib.
    """
    board_width_m = cols * square_size
    board_height_m = rows * square_size
    
    # Create grid
    # 0 = black, 1 = white (or vice versa, usually top-left is black)
    # We want squares.
    grid = np.zeros((rows, cols), dtype=np.uint8)
    grid[0::2, 0::2] = 255
    grid[1::2, 1::2] = 255
    
    # Scale up for high res image
    pixels_per_square = 300 # Arbitrary high res
    img = np.kron(grid, np.ones((pixels_per_square, pixels_per_square), dtype=np.uint8))
    
    # Margins
    margin_px = pixels_per_square // 2
    img_height, img_width = img.shape
    padded = np.full((img_height + 2*margin_px, img_width + 2*margin_px), 255, dtype=np.uint8)
    padded[margin_px:margin_px+img_height, margin_px:margin_px+img_width] = img
    
    # Physical size calculation for PDF
    # width_px = cols * pixels_per_square
    # 1 square = square_size meters
    # pixels_per_meter = pixels_per_square / square_size
    
    img_width_m = board_width_m + square_size # one square margin roughly (0.5 each side)
    img_height_m = board_height_m + square_size
    
    dpi = (pixels_per_square / square_size) * 0.0254
    
    print(f"Generating Checkerboard: {cols}x{rows} squares")
    print(f"Square size: {square_size*1000:.1f}mm")
    
    if output_file.lower().endswith('.pdf'):
        import matplotlib.pyplot as plt
        fig_width_in = img_width_m / 0.0254
        fig_height_in = img_height_m / 0.0254
        
        fig = plt.figure(figsize=(fig_width_in, fig_height_in))
        ax = plt.Axes(fig, [0., 0., 1., 1.])
        ax.set_axis_off()
        fig.add_axes(ax)
        
        ax.imshow(padded, cmap='gray', aspect='equal')
        fig.savefig(output_file, dpi=dpi)
        plt.close(fig)
    else:
        cv2.imwrite(output_file, padded)
    print(f"Saved to {output_file}")


def generate_charuco_board(output_file: str, rows=7, cols=5, square_size=0.030, marker_size=0.023):
    """
    Generate a ChArUco board image for printing.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((cols, rows), square_size, marker_size, dictionary)
    
    board_width_m = cols * square_size
    board_height_m = rows * square_size
    
    margin_m = 0.02
    img_width_m = board_width_m + 2 * margin_m
    img_height_m = board_height_m + 2 * margin_m
    
    dpi = 300
    pixels_per_meter = dpi / 0.0254
    
    img_width_px = int(img_width_m * pixels_per_meter)
    img_height_px = int(img_height_m * pixels_per_meter)
    
    print(f"Generating ChArUco: {cols}x{rows} squares")
    print(f"Square size: {square_size*1000:.1f}mm")
    
    img = board.generateImage((img_width_px, img_height_px), marginSize=int(margin_m * pixels_per_meter))
    
    if output_file.lower().endswith('.pdf'):
        import matplotlib.pyplot as plt
        fig_width_in = img_width_m / 0.0254
        fig_height_in = img_height_m / 0.0254
        
        fig = plt.figure(figsize=(fig_width_in, fig_height_in))
        ax = plt.Axes(fig, [0., 0., 1., 1.])
        ax.set_axis_off()
        fig.add_axes(ax)
        
        ax.imshow(img, cmap='gray', aspect='equal')
        fig.savefig(output_file, dpi=dpi)
        plt.close(fig)
    else:
        cv2.imwrite(output_file, img)
    
    print(f"Saved to {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate calibration board")
    parser.add_argument("--type", choices=['charuco', 'checkerboard'], default='charuco', help="Board type")
    parser.add_argument("--output", type=str, required=True, help="Output filename")
    parser.add_argument("--rows", type=int, default=7, help="Number of rows (squares)")
    parser.add_argument("--cols", type=int, default=5, help="Number of columns (squares)")
    parser.add_argument("--square", type=float, default=0.030, help="Square size in meters")
    parser.add_argument("--marker", type=float, default=0.023, help="Marker size in meters (for ChArUco)")
    
    args = parser.parse_args()
    
    if args.type == 'charuco':
        generate_charuco_board(
            args.output,
            rows=args.rows,
            cols=args.cols,
            square_size=args.square,
            marker_size=args.marker
        )
    else:
        generate_checkerboard(
            args.output,
            rows=args.rows,
            cols=args.cols,
            square_size=args.square
        )
