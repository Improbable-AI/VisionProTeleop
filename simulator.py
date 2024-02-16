import threading
from queue import Queue
import numpy as np
import open3d as o3d
import grpc
import handtracking_pb2
import handtracking_pb2_grpc
import argparse

def visualize_transformations(queue):
    # Initialize Open3D visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window("Hand and Head Tracking Visualization", width=800, height=600)

    # Create coordinate frames for visualization
    left_hand_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    right_hand_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    head_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    # Add the coordinate frames to the visualizer
    vis.add_geometry(left_hand_frame)
    vis.add_geometry(right_hand_frame)
    vis.add_geometry(head_frame)

    while True:
        try:
            # Get the latest transformations from the queue
            transformations = queue.get(timeout=1)  # Adjust timeout as needed
            if transformations is None:
                break  # Exit loop if None is received, signaling the end of the program

            # Update the coordinate frames based on the received matrices
            left_hand_frame.transform(transformations["left_hand"])
            right_hand_frame.transform(transformations["right_hand"])
            head_frame.transform(transformations["head"])

            # Update the visualization
            vis.update_geometry(left_hand_frame)
            vis.update_geometry(right_hand_frame)
            vis.update_geometry(head_frame)
            vis.poll_events()
            vis.update_renderer()
        except Queue.Empty:
            continue

    vis.destroy_window()

def run(args, queue):
    request = handtracking_pb2.HandUpdate()

    with grpc.insecure_channel(f'{args.vision_pro_ip}:12345') as channel:
        stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
        responses = stub.StreamHandUpdates(request)
        for response in responses:
            transformations = {
                "left_hand": process_matrix(response.left_hand.wristMatrix),
                "right_hand": process_matrix(response.right_hand.wristMatrix),
                "head": process_matrix(response.Head)
            }
            queue.put(transformations)

        queue.put(None)  # Signal the end of updates

def process_matrix(message):
    return np.array([[message.m00, message.m01, message.m02, message.m03],
                     [message.m10, message.m11, message.m12, message.m13],
                     [message.m20, message.m21, message.m22, message.m23],
                     [message.m30, message.m31, message.m32, message.m33]])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--vision-pro-ip', type=str, default='169.254.58.205')
    args = parser.parse_args()

    queue = Queue()
    visualization_thread = threading.Thread(target=visualize_transformations, args=(queue,))
    visualization_thread.start()

    print("Starting the gRPC client")
    run(args, queue)

    visualization_thread.join()
