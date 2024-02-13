from concurrent import futures
import grpc
import hand_pb2
import hand_pb2_grpc

class HandTrackingService(hand_pb2_grpc.HandTrackingServiceServicer):
    def StreamHandUpdates(self, request_iterator, context):
        for hand_update in request_iterator:
            print(f"Received update for left hand: {hand_update.left_hand}")
            print(f"Received update for right hand: {hand_update.right_hand}")
            yield handtracking_pb2.HandUpdateAck(message="Update Received")

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    hand_pb2_grpc.add_HandTrackingServiceServicer_to_server(HandTrackingService(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()

if __name__ == '__main__':
    serve()
