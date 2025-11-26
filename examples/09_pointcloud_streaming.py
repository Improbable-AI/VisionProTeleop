import argparse
import time
import numpy as np
import math
from avp_stream import VisionProStreamer

def generate_dynamic_sphere(t, radius=0.2, num_points=2000):
    """Generate random points on a sphere surface with dynamic radius and colors."""
    indices = np.arange(0, num_points, dtype=float) + 0.5
    phi = np.arccos(1 - 2*indices/num_points)
    theta = np.pi * (1 + 5**0.5) * indices

    # Dynamic radius: Pulse between 0.8x and 1.2x
    current_radius = radius * (1.0 + 0.2 * np.sin(t * 2.0))
    
    x = current_radius * np.sin(phi) * np.cos(theta)
    y = current_radius * np.sin(phi) * np.sin(theta)
    z = current_radius * np.cos(phi)

    points = np.stack((x, y, z), axis=-1).astype(np.float32)
    
    # Dynamic Colors: Rotating gradient
    # Map position to 0-1
    norm_x = (x / current_radius + 1) * 0.5
    norm_y = (y / current_radius + 1) * 0.5
    norm_z = (z / current_radius + 1) * 0.5
    
    # Add time shift to colors
    r = (np.sin(norm_x * np.pi + t) + 1) * 0.5 * 255
    g = (np.sin(norm_y * np.pi + t + 2.0) + 1) * 0.5 * 255
    b = (np.sin(norm_z * np.pi + t + 4.0) + 1) * 0.5 * 255
    
    colors = np.stack((r, g, b), axis=-1).astype(np.uint8)
    
    return points, colors

def main():
    parser = argparse.ArgumentParser(description="Stream a dynamic point cloud sphere to Vision Pro.")
    parser.add_argument("--ip", type=str, required=True, help="Vision Pro IP address")
    parser.add_argument("--radius", type=float, default=0.2, help="Base sphere radius in meters")
    args = parser.parse_args()

    print(f"Connecting to Vision Pro at {args.ip}...")
    streamer = VisionProStreamer(ip=args.ip, record=False)
    
    # # Configure video (optional, but good for reference)
    # streamer.configure_video(device=None, size="640x480", fps=30)
    
    # Enable point cloud streaming
    streamer.enable_point_cloud()
    
    # Start the server
    streamer.serve()

    print("Streaming started. Press Ctrl+C to stop.")
    
    # Fixed transform (1.5m up, 1m in front)
    transform = np.eye(4, dtype=np.float32)
    transform[0, 3] = 0.0
    transform[1, 3] = 1.5
    transform[2, 3] = -1.0
    
    try:
        start_time = time.time()
        while True:
            t = time.time() - start_time
            
            # Generate dynamic points and colors 
            points, colors = generate_dynamic_sphere(t, radius=args.radius)
            
            # Send data 
            streamer.update_pc(points, colors=colors, transform=transform)
            
            # time.sleep(0.033) # ~30 FPS
            
    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == "__main__":
    main()
