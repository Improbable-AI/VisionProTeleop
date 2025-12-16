import os

# Variations
types = ["charuco", "checkerboard"]
sizes_mm = [10, 20, 30]
grids = [(6, 5), (5, 4), (4, 3)] # (rows, cols)

# Helper to calculate marker size for ChArUco (approx 75% of square)
def get_marker_size(square_size_mm):
    return square_size_mm * 0.75

for t in types:
    for s_mm in sizes_mm:
        for rows, cols in grids:
            square_m = s_mm / 1000.0
            
            # Filename
            filename = f"board_{t}_{rows}x{cols}_{s_mm}mm.pdf"
            
            # Build command
            cmd = f"python scripts/generate_calibration_boards.py --type {t} --output {filename} --rows {rows} --cols {cols} --square {square_m}"
            
            if t == "charuco":
                marker_m = get_marker_size(s_mm) / 1000.0
                cmd += f" --marker {marker_m}"
            
            print(f"Generating {filename}...")
            os.system(cmd)
            
print("Done generating 18 variations.")
