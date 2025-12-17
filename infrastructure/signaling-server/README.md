# VisionProTeleop Signaling Server

WebRTC signaling server for cross-network VisionProTeleop connections.
Built on Cloudflare Workers with Durable Objects for real-time room management.

## Setup

1. Install Wrangler CLI:
```bash
npm install -g wrangler
```

2. Login to Cloudflare:
```bash
wrangler login
```

3. Deploy:
```bash
cd infrastructure/signaling-server
npm install
wrangler deploy
```

## How It Works

1. VisionOS generates a room code and connects to this server
2. Python client connects with the same room code
3. Server relays WebRTC SDP offers/answers between them
4. Once WebRTC connection is established, signaling server is no longer needed

## Protocol

WebSocket messages are JSON with a `type` field:

```typescript
// Join a room
{ type: "join", room: "ABC-1234" }

// Leave current room
{ type: "leave" }

// Send SDP offer/answer (relayed to peer)
{ type: "sdp", sdp: "...", sdpType: "offer" | "answer" }

// Send ICE candidate (relayed to peer)
{ type: "ice", candidate: {...} }

// Server notifications
{ type: "peer-joined" }      // Another peer joined your room
{ type: "peer-left" }        // Peer left your room  
{ type: "error", message: "..." }
```
