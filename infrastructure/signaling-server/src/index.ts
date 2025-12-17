/**
 * VisionProTeleop Signaling Server
 * 
 * Cloudflare Worker with Durable Objects for WebRTC signaling.
 * Handles room-based peer pairing for cross-network connections.
 */

export interface Env {
    ROOMS: DurableObjectNamespace;
    CF_TURN_KEY_ID: string;
    CF_TURN_API_TOKEN: string;
}

// ============================================================================
// Main Worker Entry Point
// ============================================================================

export default {
    async fetch(request: Request, env: Env): Promise<Response> {
        const url = new URL(request.url);

        // Health check endpoint
        if (url.pathname === "/health") {
            return new Response(JSON.stringify({ status: "ok", timestamp: Date.now() }), {
                headers: { "Content-Type": "application/json" }
            });
        }

        // WebSocket upgrade for signaling
        if (url.pathname === "/ws") {
            const upgradeHeader = request.headers.get("Upgrade");
            if (upgradeHeader !== "websocket") {
                return new Response("Expected WebSocket", { status: 426 });
            }

            // Route to a singleton Durable Object that manages all rooms
            const id = env.ROOMS.idFromName("global");
            const room = env.ROOMS.get(id);
            return room.fetch(request);
        }

        // Default response
        return new Response("VisionProTeleop Signaling Server\n\nConnect via WebSocket at /ws", {
            headers: { "Content-Type": "text/plain" }
        });
    }
};

// ============================================================================
// Signaling Room Durable Object
// ============================================================================

interface Peer {
    ws: WebSocket;
    room: string | null;
    role: "visionos" | "python" | null;
}

interface SignalingMessage {
    type: string;
    room?: string;
    role?: "visionos" | "python";
    sdp?: string;
    sdpType?: "offer" | "answer";
    candidate?: { candidate: string; sdpMid?: string; sdpMLineIndex?: number };
    message?: string;
}

export class SignalingRoom {
    private state: DurableObjectState;
    private peers: Map<WebSocket, Peer> = new Map();
    private rooms: Map<string, Set<WebSocket>> = new Map();

    private env: Env | undefined;

    constructor(state: DurableObjectState, env: Env) {
        this.state = state;
        this.env = env;
    }

    async fetch(request: Request): Promise<Response> {
        const pair = new WebSocketPair();
        const [client, server] = Object.values(pair);

        this.handleWebSocket(server);

        return new Response(null, {
            status: 101,
            webSocket: client,
        });
    }

    private handleWebSocket(ws: WebSocket) {
        // Accept the WebSocket connection
        ws.accept();

        // Initialize peer state
        this.peers.set(ws, { ws, room: null, role: null });

        ws.addEventListener("message", (event) => {
            try {
                const data = JSON.parse(event.data as string) as SignalingMessage;
                this.handleMessage(ws, data);
            } catch (e) {
                this.send(ws, { type: "error", message: "Invalid JSON" });
            }
        });

        ws.addEventListener("close", () => {
            this.handleDisconnect(ws);
        });

        ws.addEventListener("error", () => {
            this.handleDisconnect(ws);
        });
    }

    private handleMessage(ws: WebSocket, msg: SignalingMessage) {
        const peer = this.peers.get(ws);
        if (!peer) return;

        switch (msg.type) {
            case "join":
                this.handleJoin(ws, peer, msg.room || "", msg.role);
                break;

            case "leave":
                this.handleLeave(ws, peer);
                break;

            case "sdp":
                this.relayToPeer(ws, peer, {
                    type: "sdp",
                    sdp: msg.sdp,
                    sdpType: msg.sdpType,
                });
                break;

            case "ice":
                this.relayToPeer(ws, peer, {
                    type: "ice",
                    candidate: msg.candidate,
                });
                break;

            case "ping":
                this.send(ws, { type: "pong" });
                break;

            case "request-offer":
                this.relayToPeer(ws, peer, {
                    type: "request-offer"
                });
                break;

            case "request-ice":
                this.handleIceRequest(ws);
                break;

            default:
                this.send(ws, { type: "error", message: `Unknown message type: ${msg.type}` });
        }
    }

    private async handleIceRequest(ws: WebSocket) {
        try {
            const iceServers = await this.generateIceServers();
            this.send(ws, {
                type: "ice-servers",
                servers: iceServers
            });
        } catch (e: any) {
            console.error("Error generating ICE servers:", e);
            // Fallback to Google STUN if generation fails
            this.send(ws, {
                type: "ice-servers",
                servers: [{ urls: ["stun:stun.l.google.com:19302"] }]
            });
        }
    }

    private async generateIceServers(): Promise<any[]> {
        if (!this.env?.CF_TURN_KEY_ID || !this.env?.CF_TURN_API_TOKEN) {
            console.error("Missing credentials for Cloudflare TURN");
            throw new Error("Missing credentials");
        }

        const turnKeyId = this.env.CF_TURN_KEY_ID;
        const turnApiToken = this.env.CF_TURN_API_TOKEN;

        // Correct endpoint: /credentials/generate-ice-servers
        const url = `https://rtc.live.cloudflare.com/v1/turn/keys/${turnKeyId}/credentials/generate-ice-servers`;

        try {
            const response = await fetch(url, {
                method: "POST",
                headers: {
                    "Authorization": `Bearer ${turnApiToken}`,
                    "Content-Type": "application/json"
                },
                body: JSON.stringify({ ttl: 86400 }) // 24 hours TTL
            });

            if (!response.ok) {
                const errorText = await response.text();
                console.error(`Cloudflare TURN API error: ${response.status} ${errorText}`);
                throw new Error(`Failed to generate ICE servers: ${response.status} ${errorText}`);
            }

            const data = await response.json() as any;
            console.log("Cloudflare TURN response:", JSON.stringify(data, null, 2));
            
            if (!data.iceServers) {
                throw new Error("Invalid response from Cloudflare: missing iceServers");
            }
            return data.iceServers;
        } catch (e) {
            console.error("Cloudflare API error:", e);
            throw e;
        }
    }

    private handleJoin(ws: WebSocket, peer: Peer, roomCode: string, role?: "visionos" | "python") {
        // Validate room code
        if (!roomCode || roomCode.length < 3) {
            this.send(ws, { type: "error", message: "Invalid room code" });
            return;
        }

        // Leave current room if in one
        if (peer.room) {
            this.handleLeave(ws, peer);
        }

        // Join new room
        peer.room = roomCode;
        peer.role = role || null;

        if (!this.rooms.has(roomCode)) {
            this.rooms.set(roomCode, new Set());
        }
        const room = this.rooms.get(roomCode)!;

        // Check if a peer with this role already exists in the room
        if (role) {
            for (const existingWs of room) {
                const existingPeer = this.peers.get(existingWs);
                // If found a peer with the same role, remove them (replace with new connection)
                if (existingPeer && existingPeer.role === role && existingWs !== ws) {
                    this.send(existingWs, { type: "error", message: "New connection replacing you" });
                    this.handleLeave(existingWs, existingPeer);
                    try { existingWs.close(1008, "Replaced by new connection"); } catch (e) { }
                    console.log(`Replaced existing peer with role ${role} in room ${roomCode}`);
                }
            }
        }

        // Check room capacity (max 2 peers per room)
        if (room.size >= 2) {
            this.send(ws, { type: "error", message: "Room is full" });
            peer.room = null;
            return;
        }

        room.add(ws);

        // Notify the joining peer
        this.send(ws, {
            type: "joined",
            room: roomCode,
            peersInRoom: room.size
        });

        // Notify existing peer that someone joined
        for (const otherWs of room) {
            if (otherWs !== ws) {
                const otherPeer = this.peers.get(otherWs);
                this.send(otherWs, {
                    type: "peer-joined",
                    peerRole: peer.role
                });
                // Also tell the new joiner about the existing peer
                this.send(ws, {
                    type: "peer-joined",
                    peerRole: otherPeer?.role
                });
            }
        }

        console.log(`Peer joined room ${roomCode} (${room.size} peers)`);
    }

    private handleLeave(ws: WebSocket, peer: Peer) {
        if (!peer.room) return;

        const room = this.rooms.get(peer.room);
        if (room) {
            room.delete(ws);

            // Notify remaining peer
            for (const otherWs of room) {
                this.send(otherWs, { type: "peer-left" });
            }

            // Clean up empty room
            if (room.size === 0) {
                this.rooms.delete(peer.room);
            }
        }

        console.log(`Peer left room ${peer.room}`);
        peer.room = null;
        peer.role = null;
    }

    private handleDisconnect(ws: WebSocket) {
        const peer = this.peers.get(ws);
        if (peer) {
            this.handleLeave(ws, peer);
            this.peers.delete(ws);
        }
    }

    private relayToPeer(senderWs: WebSocket, senderPeer: Peer, message: object) {
        if (!senderPeer.room) {
            this.send(senderWs, { type: "error", message: "Not in a room" });
            return;
        }

        const room = this.rooms.get(senderPeer.room);
        if (!room) return;

        // Send to the other peer in the room
        for (const ws of room) {
            if (ws !== senderWs) {
                this.send(ws, message);
            }
        }
    }

    private send(ws: WebSocket, message: object) {
        try {
            ws.send(JSON.stringify(message));
        } catch (e) {
            // WebSocket might be closed
        }
    }
}
