# publisher.py
from concurrent import futures
import time
import grpc
import handtracking_pb2
import handtracking_pb2_grpc

t0 = time.time() 


class HandTrackingService(handtracking_pb2_grpc.HandTrackingServiceServicer):
    def StreamHandUpdates(self, request_iterator, context):
        while True:
            print("Sending hand update")
            t = time.time() - t0
            hand_update = handtracking_pb2.HandUpdate()
            hand_update.left_hand.x = 0.1 + t
            hand_update.left_hand.y = 0.2 + t
            hand_update.left_hand.z = 0.3 + t
            hand_update.right_hand.x = 0.4 + t
            hand_update.right_hand.y = 0.5 + t
            hand_update.right_hand.z = 0.6 + t
            yield hand_update  # Correctly yield the update to the client
            time.sleep(0.01)  # Send an update every second


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    handtracking_pb2_grpc.add_HandTrackingServiceServicer_to_server(
        HandTrackingService(), server)
    server.add_insecure_port('localhost:12345')
    server.start()
    print("Server started, listening on 'localhost:50051'.")
    server.wait_for_termination()

if __name__ == '__main__':
    serve()
