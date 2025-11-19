
# camera_offer.py
import asyncio
import json
import logging

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

logging.basicConfig(level=logging.INFO)


async def run_peer(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    print("DEBUG: Client connected, starting run_peer")
    pc = RTCPeerConnection()
    print("DEBUG: Peer connection created")

    @pc.on("iceconnectionstatechange")
    async def on_ice_state_change():
        print(f"DEBUG: ICE state changed to: {pc.iceConnectionState}")
        logging.info("ICE state: %s", pc.iceConnectionState)
        if pc.iceConnectionState in ("failed", "closed"):
            await pc.close()
            writer.close()
            await writer.wait_closed()

    # Open Mac webcam via AVFoundation
    print("DEBUG: Opening webcam...")
    try:
        # For macOS, use "0" for default camera or "FaceTime HD Camera" for built-in camera
        player = MediaPlayer(
            "0:none",             # device 0 for video, no audio
            format="avfoundation",
            options={"video_size": "640x480", "framerate": "30"},
        )
        print(f"DEBUG: MediaPlayer created, has video: {player.video is not None}")
    except Exception as e:
        print(f"DEBUG: Error creating MediaPlayer: {e}")
        raise

    if player.video:
        print("DEBUG: Adding video track to peer connection")
        pc.addTrack(player.video)

    # Create offer
    print("DEBUG: Creating offer...")
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    print("DEBUG: Local description set (offer)")

    # Wait until ICE gathering is complete (no trickle)
    print(f"DEBUG: ICE gathering state: {pc.iceGatheringState}")
    while pc.iceGatheringState != "complete":
        await asyncio.sleep(0.1)
    print("DEBUG: ICE gathering complete")

    # Send SDP offer as JSON (one line)
    offer_payload = {
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type,
    }
    print("DEBUG: Sending offer to client...")
    writer.write((json.dumps(offer_payload) + "\n").encode("utf-8"))
    await writer.drain()
    logging.info("Sent offer to client")
    print("DEBUG: Offer sent")

    # Wait for answer from client
    print("DEBUG: Waiting for answer from client...")
    line = await reader.readline()
    print(f"DEBUG: Received line: {line[:100] if line else 'None'}...")
    if not line:
        logging.info("Client disconnected before sending answer")
        await pc.close()
        return

    answer_payload = json.loads(line.decode("utf-8"))
    print("DEBUG: Parsed answer JSON")
    answer = RTCSessionDescription(
        sdp=answer_payload["sdp"],
        type=answer_payload["type"],
    )

    print("DEBUG: Setting remote description (answer)...")
    await pc.setRemoteDescription(answer)
    logging.info("Remote description set (answer)")
    print("DEBUG: Remote description set, connection should be establishing")

    # Keep connection alive until closed
    try:
        print("DEBUG: Entering keep-alive loop")
        while True:
            await asyncio.sleep(1)
            print(f"DEBUG: Server connection state: {pc.connectionState}")
    except asyncio.CancelledError:
        print("DEBUG: Keep-alive loop cancelled")
        pass
    finally:
        logging.info("Closing peer connection")
        print("DEBUG: Cleaning up server connection")
        await pc.close()
        writer.close()
        await writer.wait_closed()


async def main():
    server = await asyncio.start_server(run_peer, "0.0.0.0", 9999)
    addr = ", ".join(str(sock.getsockname()) for sock in server.sockets)
    logging.info("Camera offer server listening on %s", addr)

    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())