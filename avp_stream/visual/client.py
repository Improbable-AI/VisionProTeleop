# viewer_answer.py
import asyncio
import json
import logging

import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription

logging.basicConfig(level=logging.INFO)


async def main():
    print("DEBUG: Starting client...")
    reader, writer = await asyncio.open_connection("127.0.0.1", 9999)
    print("DEBUG: Connected to server")
    
    pc = RTCPeerConnection()
    print("DEBUG: Peer connection created")

    video_track_received = asyncio.Event()
    video_task = None

    @pc.on("track")
    async def on_track(track):
        nonlocal video_task
        print(f"DEBUG: Track callback triggered, kind={track.kind}")
        logging.info("Track received: kind=%s", track.kind)
        if track.kind != "video":
            print("DEBUG: Track is not video, ignoring")
            return

        print("DEBUG: Setting video_track_received event")
        video_track_received.set()

        print("DEBUG: Starting frame receiving loop")
        while True:
            try:
                frame = await track.recv()
                print("DEBUG: Frame received")
                img = frame.to_ndarray(format="bgr24")

                cv2.imshow("Remote camera", img)
                # break on 'q'
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("DEBUG: 'q' key pressed, exiting")
                    break
            except Exception as e:
                logging.error("Error receiving frame: %s", e)
                break

        logging.info("Stopping video display")
        cv2.destroyAllWindows()
        await pc.close()

    @pc.on("iceconnectionstatechange")
    async def on_ice_state_change():
        print(f"DEBUG: ICE state changed to: {pc.iceConnectionState}")
        logging.info("ICE state: %s", pc.iceConnectionState)
        if pc.iceConnectionState in ("failed", "closed"):
            await pc.close()
            writer.close()

    # Read offer from server
    print("DEBUG: Waiting for offer from server...")
    line = await reader.readline()
    print(f"DEBUG: Received line: {line[:100] if line else 'None'}...")
    if not line:
        logging.error("No offer received")
        return

    offer_payload = json.loads(line.decode("utf-8"))
    print("DEBUG: Parsed offer JSON")
    offer = RTCSessionDescription(
        sdp=offer_payload["sdp"],
        type=offer_payload["type"],
    )
    print("DEBUG: Setting remote description...")
    await pc.setRemoteDescription(offer)
    logging.info("Remote description set (offer)")

    # Create and send answer
    print("DEBUG: Creating answer...")
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    print("DEBUG: Local description set (answer)")

    # Wait until ICE gathering is complete (no trickle)
    print(f"DEBUG: ICE gathering state: {pc.iceGatheringState}")
    while pc.iceGatheringState != "complete":
        await asyncio.sleep(0.1)
    print("DEBUG: ICE gathering complete")

    answer_payload = {
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type,
    }
    writer.write((json.dumps(answer_payload) + "\n").encode("utf-8"))
    await writer.drain()
    logging.info("Sent answer to server")
    print("DEBUG: Answer sent to server")

    # Wait for video track to be received
    print("DEBUG: Waiting for video track...")
    await video_track_received.wait()
    logging.info("Video track established, receiving frames...")
    print("DEBUG: Video track received, now in main loop")

    # Keep loop alive while video displays
    try:
        while pc.connectionState != "closed":
            await asyncio.sleep(1)
            print(f"DEBUG: Connection state: {pc.connectionState}")
    except asyncio.CancelledError:
        print("DEBUG: Main loop cancelled")
        pass
    finally:
        print("DEBUG: Cleaning up...")
        cv2.destroyAllWindows()
        await pc.close()


if __name__ == "__main__":
    asyncio.run(main())