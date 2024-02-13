# subscriber.py

import os
import logging

# os.environ['GRPC_VERBOSITY'] = 'DEBUG'
# # os.environ['GRPC_TRACE'] = 'all'
# logging.basicConfig(level=logging.DEBUG)


import grpc
import handtracking_pb2
import handtracking_pb2_grpc

def run():

    request = handtracking_pb2.HandUpdate()

    with grpc.insecure_channel('localhost:12345') as channel:
        stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
        # Correctly invoking a server-streaming RPC
        responses = stub.StreamHandUpdates(request)
        for response in responses:
            print(f"Received update for left hand: {response.left_hand}")
            print(f"Received update for right hand: {response.right_hand}")

if __name__ == '__main__':
    print("Starting the gRPC client")
    run()
