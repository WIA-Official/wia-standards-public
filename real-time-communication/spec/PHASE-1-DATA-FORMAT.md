# WIA-COMM-019 — Phase 1: Data Format

> Real-time communication canonical envelopes: WebRTC architecture, SIP session, RTP/RTCP media transport, and the runtime conventions that fix the wire format for every protocol below.

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive framework for real-time communication systems, enabling low-latency voice, video, and data transmission across diverse network environments with high quality and security.

### 1.2 Scope

The standard covers:
- WebRTC peer-to-peer communication architecture
- SIP for call signaling and session management
- RTP/RTCP for media transport and quality feedback
- SRTP for secure media encryption
- Adaptive jitter buffer management
- Video and audio codec selection (VP8/VP9, H.264, Opus, etc.)
- NAT traversal using STUN, TURN, and ICE
- Signaling server protocols (WebSocket, SIP, XMPP)
- Quality of Experience (QoE) measurement and optimization
- Low-latency streaming techniques
- Multi-party video conferencing (MCU/SFU architectures)
- Push-to-talk communication systems

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to connect people seamlessly and securely, regardless of geographical or technological barriers, fostering global communication and collaboration.

### 1.4 Terminology

- **WebRTC**: Web Real-Time Communication
- **SIP**: Session Initiation Protocol
- **RTP**: Real-time Transport Protocol
- **RTCP**: RTP Control Protocol
- **SRTP**: Secure RTP
- **ICE**: Interactive Connectivity Establishment
- **STUN**: Session Traversal Utilities for NAT
- **TURN**: Traversal Using Relays around NAT
- **SDP**: Session Description Protocol
- **NAT**: Network Address Translation
- **MCU**: Multipoint Control Unit
- **SFU**: Selective Forwarding Unit
- **QoE**: Quality of Experience
- **MOS**: Mean Opinion Score
- **PTT**: Push-to-Talk

---


## 2. WebRTC Architecture

### 2.1 Overview

WebRTC provides browser-based real-time communication without plugins, using open standards for audio, video, and data exchange.

### 2.2 Core Components

#### 2.2.1 PeerConnection API

The central interface for WebRTC peer-to-peer connections:

```javascript
const configuration = {
  iceServers: [
    { urls: 'stun:stun.l.google.com:19302' },
    {
      urls: 'turn:turn.example.com:3478',
      username: 'user',
      credential: 'pass'
    }
  ],
  iceCandidatePoolSize: 10,
  bundlePolicy: 'max-bundle',
  rtcpMuxPolicy: 'require'
};

const pc = new RTCPeerConnection(configuration);
```

**Key Methods:**
- `createOffer()`: Generate SDP offer
- `createAnswer()`: Generate SDP answer
- `setLocalDescription(desc)`: Set local SDP
- `setRemoteDescription(desc)`: Set remote SDP
- `addIceCandidate(candidate)`: Add ICE candidate
- `addTrack(track, stream)`: Add media track
- `addTransceiver(kind, init)`: Add transceiver

**Connection States:**
- `new`: Initial state
- `connecting`: ICE/DTLS in progress
- `connected`: Media flowing
- `disconnected`: Temporary disconnection
- `failed`: Connection failed
- `closed`: Connection closed

#### 2.2.2 MediaStream API

Access to camera and microphone:

```javascript
const constraints = {
  audio: {
    echoCancellation: true,
    noiseSuppression: true,
    autoGainControl: true,
    sampleRate: 48000,
    channelCount: 2
  },
  video: {
    width: { min: 640, ideal: 1280, max: 1920 },
    height: { min: 480, ideal: 720, max: 1080 },
    frameRate: { min: 15, ideal: 30, max: 60 },
    facingMode: 'user'
  }
};

const stream = await navigator.mediaDevices.getUserMedia(constraints);
```

**MediaStreamTrack Properties:**
- `kind`: 'audio' or 'video'
- `label`: Device label
- `enabled`: Track enabled state
- `muted`: Track muted state
- `readyState`: 'live' or 'ended'

#### 2.2.3 DataChannel API

Arbitrary data exchange:

```javascript
const dataChannel = pc.createDataChannel('chat', {
  ordered: true,
  maxRetransmits: 3,
  protocol: 'json'
});

dataChannel.onopen = () => {
  dataChannel.send(JSON.stringify({ msg: 'Hello' }));
};

dataChannel.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

**Properties:**
- `ordered`: In-order delivery guarantee
- `maxPacketLifeTime`: Maximum retransmission time
- `maxRetransmits`: Maximum retransmission count
- `protocol`: Subprotocol name
- `negotiated`: Pre-negotiated channel
- `id`: Channel identifier

### 2.3 Signaling Flow

WebRTC requires external signaling for peer discovery and session negotiation:

```
Alice                    Signaling Server                    Bob
  |                              |                              |
  |------ createOffer() -------->|                              |
  |                              |                              |
  |                              |-------- offer SDP --------->|
  |                              |                              |
  |                              |<------- answer SDP ---------|
  |                              |                              |
  |<----- answer SDP ------------|                              |
  |                              |                              |
  |------ ICE candidate -------->|                              |
  |                              |------ ICE candidate -------->|
  |                              |                              |
  |<----- ICE candidate ---------|                              |
  |                              |<----- ICE candidate ---------|
  |                              |                              |
  |<=========== Media Connection (P2P) =====================>|
```

### 2.4 SDP (Session Description Protocol)

SDP describes media capabilities and parameters:

```
v=0
o=- 1234567890 2 IN IP4 127.0.0.1
s=-
t=0 0
a=group:BUNDLE 0 1
a=msid-semantic: WMS stream1

m=audio 9 UDP/TLS/RTP/SAVPF 111 103 104
c=IN IP4 0.0.0.0
a=rtcp:9 IN IP4 0.0.0.0
a=ice-ufrag:F7gI
a=ice-pwd:x9cml/YzichV2+XlhiMu8g
a=fingerprint:sha-256 49:66:12:17:0D:1C:91:AE:57:4C:C6:36:DD:D5:5D:48
a=setup:actpass
a=mid:0
a=sendrecv
a=rtcp-mux
a=rtpmap:111 opus/48000/2
a=fmtp:111 minptime=10;useinbandfec=1

m=video 9 UDP/TLS/RTP/SAVPF 96 97 98
c=IN IP4 0.0.0.0
a=rtcp:9 IN IP4 0.0.0.0
a=ice-ufrag:F7gI
a=ice-pwd:x9cml/YzichV2+XlhiMu8g
a=fingerprint:sha-256 49:66:12:17:0D:1C:91:AE:57:4C:C6:36:DD:D5:5D:48
a=setup:actpass
a=mid:1
a=sendrecv
a=rtcp-mux
a=rtpmap:96 VP8/90000
a=rtpmap:97 VP9/90000
a=rtpmap:98 H264/90000
a=fmtp:98 profile-level-id=42e01f;packetization-mode=1
```

**SDP Sections:**
- **Session-level**: Global parameters
- **Media-level**: Per-media description (audio, video, data)
- **Attributes**: Codec parameters, encryption, ICE credentials

---


## 3. SIP (Session Initiation Protocol)

### 3.1 SIP Architecture

SIP is a signaling protocol for initiating, maintaining, and terminating real-time sessions:

**Components:**
- **User Agent (UA)**: Endpoint device
- **Proxy Server**: Routes requests
- **Registrar**: Registers user locations
- **Redirect Server**: Returns alternate addresses
- **Back-to-Back User Agent (B2BUA)**: Intermediary UA

### 3.2 SIP Messages

#### 3.2.1 Request Methods

| Method | Description | Usage |
|--------|-------------|-------|
| INVITE | Initiate session | Start call |
| ACK | Acknowledge INVITE | Confirm session |
| BYE | Terminate session | End call |
| CANCEL | Cancel pending request | Cancel ringing |
| REGISTER | Register location | User registration |
| OPTIONS | Query capabilities | Presence check |
| INFO | Mid-session info | DTMF, events |
| PRACK | Provisional acknowledgment | Reliable provisional |
| UPDATE | Session modification | Mid-call update |
| REFER | Transfer request | Call transfer |
| SUBSCRIBE | Subscribe to event | Presence subscription |
| NOTIFY | Event notification | Presence update |

#### 3.2.2 Response Codes

**1xx - Provisional:**
- 100 Trying
- 180 Ringing
- 181 Call Is Being Forwarded
- 183 Session Progress

**2xx - Success:**
- 200 OK

**3xx - Redirection:**
- 301 Moved Permanently
- 302 Moved Temporarily

**4xx - Client Error:**
- 400 Bad Request
- 401 Unauthorized
- 403 Forbidden
- 404 Not Found
- 486 Busy Here
- 487 Request Terminated

**5xx - Server Error:**
- 500 Server Internal Error
- 503 Service Unavailable

**6xx - Global Failure:**
- 600 Busy Everywhere
- 603 Decline

### 3.3 SIP Call Flow

Basic call establishment:

```
Alice (UA)                  Proxy                    Bob (UA)
    |                         |                         |
    |------ INVITE ---------->|                         |
    |<----- 100 Trying -------|                         |
    |                         |------ INVITE ---------->|
    |                         |<----- 100 Trying -------|
    |                         |<----- 180 Ringing ------|
    |<----- 180 Ringing ------|                         |
    |                         |<----- 200 OK ----------|
    |<----- 200 OK -----------|                         |
    |------ ACK ------------->|------ ACK ------------->|
    |                         |                         |
    |<=============== RTP Media Stream ===============>|
    |                         |                         |
    |------ BYE ------------->|------ BYE ------------->|
    |<----- 200 OK -----------|<----- 200 OK -----------|
```

### 3.4 SIP Message Format

Example INVITE:

```
INVITE sip:bob@example.com SIP/2.0
Via: SIP/2.0/UDP pc33.atlanta.com;branch=z9hG4bK776asdhds
Max-Forwards: 70
To: Bob <sip:bob@example.com>
From: Alice <sip:alice@atlanta.com>;tag=1928301774
Call-ID: a84b4c76e66710@pc33.atlanta.com
CSeq: 314159 INVITE
Contact: <sip:alice@pc33.atlanta.com>
Content-Type: application/sdp
Content-Length: 142

v=0
o=alice 2890844526 2890844526 IN IP4 pc33.atlanta.com
s=Session SDP
c=IN IP4 pc33.atlanta.com
t=0 0
m=audio 49170 RTP/AVP 0
a=rtpmap:0 PCMU/8000
```

---



## A.1 Canonical envelope conventions

Every Phase 1 RTC envelope follows the WIA family baseline: UTF-8
JSON for control messages with RFC 8785 canonicalisation, RTP
binary frames for media payloads. Signing applies to control
envelopes; media payloads are protected by SRTP at the transport
layer.

## A.2 Session descriptor envelope

```json
{
  "wia_rtc_version": "1.0.0",
  "type": "session_descriptor",
  "session_id": "sess_01HX...",
  "participants": ["did:wia:user:alice", "did:wia:user:bob"],
  "media_descriptions": [
    { "kind": "audio", "codec": "opus", "rate": 48000 },
    { "kind": "video", "codec": "vp9", "rate": 90000 }
  ],
  "ice_candidates": [...],
  "started_at": "RFC 3339"
}
```

## A.3 SDP compatibility

The session descriptor round-trips cleanly to RFC 8866 SDP (Session
Description Protocol). The bridge profile documents the mapping for
SIP-based deployments that need SDP compatibility for legacy peer
interop.

## A.4 RTP packet header conventions

RTP packets follow IETF RFC 3550 with the standard's WIA-RTC
profile (a documented payload type assignment for the supported
codec set). Packets carry the standard sequence number, timestamp,
and SSRC fields plus optional header extensions per RFC 8285.


## Z.1 Glossary

The companion glossary at `https://wiastandards.com/real-time-communication/glossary/`
expands every term used throughout this Phase. Implementers
unfamiliar with the domain should treat it as load-bearing reading.

## Z.2 Cross-standard composition

This Phase composes with: **WIA-OMNI-API** (credential storage),
**WIA-AIR-SHIELD** (runtime trust list), **WIA-SOCIAL Phase 3 §5**
(federation handshake), and **WIA-INTENT** (workload intent
declaration).

## Z.3 Conformance test suite + reference container

A black-box conformance test suite at
`https://github.com/WIA-Official/wia-real-time-communication-conformance` walks
every public endpoint and protocol exchange. The reference
container at `wia/real-time-communication-host:1.0.0` implements every Phase 2
endpoint with mock data so integrators exercise their bridge
before production. The companion CLI at `cli/real-time-communication.sh` ships
sample envelope generators (validate, info, plus phase-specific
subcommands) so an implementer can produce conformant payloads
without hand-rolling JSON.

## Z.4 Implementation runbook

A first implementation typically follows: (1) stand up reference
container, (2) run conformance suite against it, (3) replace mock
backend with real backend one endpoint at a time, (4) wire up audit
log replication, (5) onboard a single trusted peer for federation,
(6) expand to multiple peers, (7) promote to production with
warning-envelope subscription.

## Z.5 Backwards-compatibility promise + governance

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable.
Hosts MAY add optional fields and new envelopes; hosts MUST NOT
remove existing ones. Breaking changes ride a major version bump
with a 12-month deprecation window per IETF RFC 8594 / 9745, and
require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.
