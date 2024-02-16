# subscriber.py

import os
import logging

import grpc
import handtracking_pb2
import handtracking_pb2_grpc
import numpy as np 
import open3d 

import argparse

def run(args):

    request = handtracking_pb2.HandUpdate()

    with grpc.insecure_channel(f'{args.vision_pro_ip}:12345') as channel:
        stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
        # Correctly invoking a server-streaming RPC
        responses = stub.StreamHandUpdates(request)
        for response in responses:
            print(f"Received update for left hand: \n {process_matrix(response.left_hand.wristMatrix)}")
            print(f"Received update for right hand: \n {process_matrix(response.right_hand.wristMatrix)}")
            print(f"Received update for head: \n {process_matrix(response.Head)}")
            print('\n')


def process_matrix(message): 
    return np.array([[message.m00, message.m01, message.m02, message.m03],
                     [message.m10, message.m11, message.m12, message.m13],
                     [message.m20, message.m21, message.m22, message.m23],
                     [message.m30, message.m31, message.m32, message.m33]])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--vision-pro-ip', type=str, default = '169.254.58.205')
    args = parser.parse_args()
    print("Starting the gRPC client")
    run(args)
