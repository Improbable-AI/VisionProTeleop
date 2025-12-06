#!/usr/bin/env python3
"""
Compare ArUco marker generation between OpenCV and iPhone Swift implementation.
This helps verify that both systems generate identical markers.
"""

import cv2
import numpy as np

# iPhone Swift implementation's hardcoded marker patterns for DICT_4X4_50
# From CalibrationDisplayManager.swift - dict4x4_50 array
IPHONE_DICT_4X4_50 = {
    0: [1,0,0,0, 0,1,0,1, 1,0,1,1, 0,0,0,0],
    1: [0,1,1,1, 1,0,0,0, 0,1,0,0, 1,1,1,1],
    2: [1,0,1,0, 0,1,1,1, 1,0,0,0, 1,0,1,0],
    3: [0,1,0,1, 1,0,1,0, 0,1,1,1, 0,1,0,1],
}


def opencv_marker_to_bits(marker_id: int, dictionary=cv2.aruco.DICT_4X4_50):
    """Generate marker using OpenCV and extract bit pattern."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
    
    # Generate high-res marker
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 200)
    
    # The marker has a 1-cell border, so the data is in the inner 4x4 grid
    # For a 200px image of a 4x4 marker: border=1 cell, so 6 total cells
    cell_size = 200 // 6  # ~33 pixels per cell
    border_cells = 1
    
    bits = []
    for row in range(4):
        for col in range(4):
            # Sample center of each data cell (skip border)
            y = (border_cells + row) * cell_size + cell_size // 2
            x = (border_cells + col) * cell_size + cell_size // 2
            
            # Sample pixel value (0=black, 255=white)
            # In ArUco: 0=black=bit 0, 255=white=bit 1
            pixel_value = marker_img[y, x]
            bit = 1 if pixel_value > 127 else 0
            bits.append(bit)
    
    return bits, marker_img


def bits_to_visual(bits):
    """Convert bit array to visual ASCII representation."""
    lines = []
    for row in range(4):
        line = ""
        for col in range(4):
            bit = bits[row * 4 + col]
            line += "██" if bit == 0 else "  "  # 0=black, 1=white
        lines.append(line)
    return "\n".join(lines)


def compare_markers(marker_ids=[0, 1, 2, 3]):
    """Compare OpenCV and iPhone marker generation."""
    print("\n" + "="*70)
    print("ArUco Marker Comparison: OpenCV vs iPhone Swift Implementation")
    print("="*70 + "\n")
    
    all_match = True
    
    for marker_id in marker_ids:
        print(f"\n{'─'*70}")
        print(f"Marker ID: {marker_id}")
        print(f"{'─'*70}")
        
        # Get OpenCV bits
        opencv_bits, opencv_img = opencv_marker_to_bits(marker_id)
        
        # Get iPhone bits
        iphone_bits = IPHONE_DICT_4X4_50.get(marker_id)
        
        if iphone_bits is None:
            print(f"⚠️  Warning: No iPhone pattern available for marker {marker_id}")
            continue
        
        # Compare
        match = opencv_bits == iphone_bits
        all_match = all_match and match
        
        print(f"\nOpenCV Pattern:")
        print(bits_to_visual(opencv_bits))
        print(f"\nBits: {opencv_bits}")
        
        print(f"\niPhone Pattern:")
        print(bits_to_visual(iphone_bits))
        print(f"\nBits: {iphone_bits}")
        
        if match:
            print(f"\n✅ MATCH - Markers are identical")
        else:
            print(f"\n❌ MISMATCH - Markers are different!")
            print(f"\nDifferences:")
            for i, (o, p) in enumerate(zip(opencv_bits, iphone_bits)):
                if o != p:
                    row, col = i // 4, i % 4
                    print(f"  Position ({row},{col}): OpenCV={o}, iPhone={p}")
        
        # Save comparison image
        cv2.imwrite(f"/tmp/aruco_marker_{marker_id}_opencv.png", opencv_img)
    
    print(f"\n{'='*70}")
    if all_match:
        print("✅ ALL MARKERS MATCH - OpenCV and iPhone generate identical markers")
    else:
        print("❌ MISMATCH DETECTED - OpenCV and iPhone generate different markers!")
        print("\n💡 This explains why detection is failing!")
        print("   The iPhone displays one pattern, but OpenCV/ARKit expect another.")
    print("="*70 + "\n")
    
    return all_match


if __name__ == "__main__":
    compare_markers([0, 1, 2, 3])
