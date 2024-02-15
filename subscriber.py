# subscriber.py

import os
import logging

# os.environ['GRPC_VERBOSITY'] = 'DEBUG'
# # os.environ['GRPC_TRACE'] = 'all'
# logging.basicConfig(level=logging.DEBUG)


import grpc
import handtracking_pb2
import handtracking_pb2_grpc

import argparse

def run(args):

    request = handtracking_pb2.HandUpdate()

    with grpc.insecure_channel(f'{args.vision_pro_ip}:12345') as channel:
        stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
        # Correctly invoking a server-streaming RPC
        responses = stub.StreamHandUpdates(request)
        for response in responses:
            print(f"Received update for left hand: {response.left_hand.wristMatrix}")
            print(f"Received update for right hand: {response.right_hand.wristMatrix}")
            print(f"Received update for head: {response.Head}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--vision-pro-ip', type=str, default = '169.254.58.205')
    args = parser.parse_args()
    print("Starting the gRPC client")
    run(args)
