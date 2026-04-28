# WIA-COMM-019 — Phase 3: Protocol

> Signalling-server, QoE, low-latency streaming, video-conferencing, and push-to-talk protocol layer. Every protocol exchange is wire-level with replay defence applied uniformly.

## 9. Signaling Servers

### 9.1 WebSocket Signaling

Real-time bidirectional signaling:

**Server (Node.js):**
```javascript
const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 8080 });

const clients = new Map();

wss.on('connection', (ws) => {
  ws.on('message', (message) => {
    const data = JSON.parse(message);

    switch (data.type) {
      case 'register':
        clients.set(data.userId, ws);
        break;

      case 'offer':
      case 'answer':
      case 'ice-candidate':
        const recipientWs = clients.get(data.to);
        if (recipientWs) {
          recipientWs.send(JSON.stringify(data));
        }
        break;
    }
  });

  ws.on('close', () => {
    // Remove client
  });
});
```

**Client:**
```javascript
const ws = new WebSocket('wss://signal.example.com');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'register',
    userId: 'alice'
  }));
};

ws.onmessage = async (event) => {
  const data = JSON.parse(event.data);

  switch (data.type) {
    case 'offer':
      await pc.setRemoteDescription(data.offer);
      const answer = await pc.createAnswer();
      await pc.setLocalDescription(answer);
      ws.send(JSON.stringify({
        type: 'answer',
        to: data.from,
        answer: answer
      }));
      break;

    case 'ice-candidate':
      await pc.addIceCandidate(data.candidate);
      break;
  }
};
```

### 9.2 SIP over WebSocket (RFC 7118)

SIP signaling in browsers:

**Configuration:**
```javascript
const sipConfig = {
  uri: 'sip:alice@example.com',
  wsServers: ['wss://sip.example.com:7443'],
  authorizationUser: 'alice',
  password: 'secret',
  registerExpires: 3600,
  sessionDescriptionHandlerFactoryOptions: {
    constraints: {
      audio: true,
      video: true
    }
  }
};
```

### 9.3 XMPP (Jingle)

XMPP-based signaling using Jingle extensions:

**Features:**
- Presence and roster management
- Security (TLS, SASL)
- Federation support

---


## 10. Quality of Experience (QoE)

### 10.1 Objective Metrics

#### 10.1.1 MOS (Mean Opinion Score)

5-point scale for perceived quality:

| MOS | Quality | Impairment |
|-----|---------|------------|
| 5.0 | Excellent | Imperceptible |
| 4.0 | Good | Perceptible but not annoying |
| 3.0 | Fair | Slightly annoying |
| 2.0 | Poor | Annoying |
| 1.0 | Bad | Very annoying |

**E-Model (G.107):**

Calculates MOS from network parameters:

```
R = R0 - Is - Id - Ie + A

MOS = 1 + 0.035*R + 7*10^-6*R*(R-60)*(100-R)
```

Where:
- R0: Basic signal-to-noise ratio
- Is: Simultaneous impairment
- Id: Delay impairment
- Ie: Equipment impairment
- A: Advantage factor

#### 10.1.2 PESQ/POLQA

Perceptual audio quality measurement:

**PESQ (ITU-T P.862):**
- Range: -0.5 to 4.5
- Maps to MOS
- Narrowband (8 kHz) and wideband (16 kHz)

**POLQA (ITU-T P.863):**
- Next-generation PESQ
- Super-wideband (48 kHz)
- Better correlation with subjective scores

#### 10.1.3 VMAF

Video quality metric from Netflix:

**Features:**
- Machine learning-based
- Scale: 0-100
- Correlates well with subjective quality
- Considers spatial and temporal artifacts

### 10.2 Network Metrics

#### 10.2.1 Latency

End-to-end delay:

**Components:**
- Encoding delay
- Packetization delay
- Network transmission delay
- Jitter buffer delay
- Decoding delay

**Targets:**
- VoIP: < 150ms (one-way)
- Video conferencing: < 150ms
- Interactive gaming: < 50ms

**Measurement:**
Via RTCP:

```
RTT = Current_Time - LSR - DLSR
One_Way_Delay = RTT / 2 (estimate)
```

#### 10.2.2 Jitter

Variation in packet arrival times:

**Calculation (RFC 3550):**
```
J(i) = J(i-1) + (|D(i-1,i)| - J(i-1)) / 16

D(i-1,i) = (R(i) - S(i)) - (R(i-1) - S(i-1))
```

Where:
- R(i): Receive timestamp
- S(i): Send timestamp (RTP)
- J(i): Smoothed jitter

**Targets:**
- VoIP: < 30ms
- Video: < 50ms

#### 10.2.3 Packet Loss

**Calculation:**
```
loss_rate = (packets_lost / packets_sent) * 100%
```

**Targets:**
- VoIP: < 1%
- Video: < 3%

**Mitigation:**
- Forward error correction (FEC)
- Packet loss concealment (PLC)
- Redundancy (RED)

### 10.3 WebRTC Statistics API

```javascript
const stats = await pc.getStats();

stats.forEach(report => {
  if (report.type === 'inbound-rtp' && report.kind === 'audio') {
    console.log('Packets received:', report.packetsReceived);
    console.log('Packets lost:', report.packetsLost);
    console.log('Jitter:', report.jitter);
  }

  if (report.type === 'candidate-pair' && report.state === 'succeeded') {
    console.log('RTT:', report.currentRoundTripTime);
    console.log('Available outgoing bitrate:', report.availableOutgoingBitrate);
  }
});
```

---


## 11. Low-Latency Streaming

### 11.1 Glass-to-Glass Latency

Total delay from camera to display:

**Target:** < 1 second (ideally < 500ms)

**Components:**
1. Camera capture: 16-33ms (30-60 fps)
2. Encoding: 20-100ms
3. Packetization: 10-20ms
4. Network transmission: 10-200ms
5. Jitter buffer: 20-60ms
6. Decoding: 20-50ms
7. Display: 16-33ms

**Total:** ~112-496ms (best case to typical)

### 11.2 Techniques

#### 11.2.1 Ultra-Low Latency Codecs

- **AV1**: Low Delay mode
- **H.264**: Baseline with low-delay HRD
- **VP9**: Real-time mode

#### 11.2.2 Small GOP Size

- **Intra-only**: Every frame is I-frame (highest latency, large bandwidth)
- **Small GOP**: 1-2 second GOP (good balance)
- **Keyframe frequency**: Every 1-3 seconds

#### 11.2.3 Fast Encoding Presets

Trade compression efficiency for speed:

**x264 presets:**
- `ultrafast`: Lowest latency, largest file
- `superfast`: Very low latency
- `veryfast`: Low latency (recommended)

#### 11.2.4 UDP vs. TCP

- **UDP**: Lower latency, no retransmission
- **TCP**: Higher latency, reliable delivery

**WebRTC uses UDP** with application-level retransmission (NACK).

### 11.3 Adaptive Bitrate Streaming

Adjust quality based on network:

**Simulcast:**
Send multiple resolutions simultaneously, receiver chooses:

```javascript
const sender = pc.addTrack(track, stream);

const params = sender.getParameters();
params.encodings = [
  { rid: 'high', maxBitrate: 1500000 },
  { rid: 'medium', maxBitrate: 600000, scaleResolutionDownBy: 2 },
  { rid: 'low', maxBitrate: 200000, scaleResolutionDownBy: 4 }
];

sender.setParameters(params);
```

**SVC (Scalable Video Coding):**
Single stream with multiple layers:
- Temporal scalability: Frame rate layers
- Spatial scalability: Resolution layers
- Quality scalability: SNR layers

---


## 12. Video Conferencing Systems

### 12.1 Architectures

#### 12.1.1 MCU (Multipoint Control Unit)

Centralized mixing:

```
Participant A ----\
                   \
Participant B -------> MCU (mixes all streams) --> Single mixed stream to all
                   /
Participant C ----/
```

**Pros:**
- Low client bandwidth
- Single decode per client
- Consistent layout

**Cons:**
- High server CPU
- Quality loss (transcoding)
- Latency from mixing

#### 12.1.2 SFU (Selective Forwarding Unit)

Routing without transcoding:

```
Participant A ----\
                   \
Participant B -------> SFU (forwards selectively) --> N-1 streams to each participant
                   /
Participant C ----/
```

**Pros:**
- Lower server CPU
- Better quality (no transcoding)
- Lower latency

**Cons:**
- Higher client bandwidth
- Multiple encodes/decodes
- Complexity in client

#### 12.1.3 Mesh (P2P Full Mesh)

Direct peer-to-peer between all participants:

```
Participant A <-------> Participant B
     ^                      ^
     |                      |
     v                      v
Participant C <-------> Participant D
```

**Pros:**
- No server infrastructure
- Lowest latency

**Cons:**
- Exponential bandwidth growth: N*(N-1)
- Scalability limit: ~4-6 participants
- NAT traversal complexity

### 12.2 Layout Management

**Common Layouts:**
- **Speaker-focused**: Large view of active speaker, thumbnails of others
- **Grid**: Equal-sized tiles
- **Picture-in-Picture**: Main content + small self-view
- **Gallery**: Multiple equal participants
- **Presentation**: Screen share + video thumbnails

**Active Speaker Detection:**
```javascript
conference.on('audio-level-changed', (participant, level) => {
  if (level > threshold) {
    switchLayout('speaker-focused', participant);
  }
});
```

### 12.3 Bandwidth Optimization

**Simulcast Reception:**
SFU selects appropriate quality per receiver:

```javascript
// Receiver subscribes to medium quality
receiver.setParameters({
  encodings: [
    { rid: 'medium', active: true }
  ]
});
```

**Video Pause for Off-Screen:**
Stop video for participants not visible:

```javascript
participants.forEach(p => {
  if (!isVisible(p)) {
    p.setVideoEnabled(false);
  }
});
```

---


## 13. Push-to-Talk Systems

### 13.1 Architecture

Half-duplex communication with floor control:

**Components:**
- **PTT Client**: User device
- **PTT Server**: Floor control logic
- **Media Server**: Audio distribution

### 13.2 Floor Control

**States:**
- **Idle**: No one speaking
- **Granted**: User has floor
- **Queued**: User requested, waiting
- **Denied**: Request rejected (another user has floor)

**Message Flow:**
```
Client A                PTT Server              Client B, C
   |                         |                      |
   |--- Floor Request ------>|                      |
   |<-- Floor Granted -------|                      |
   |                         |                      |
   |--- Audio Stream ------->|--- Audio Stream --->|
   |                         |                      |
   |--- Floor Release ------>|                      |
   |<-- Floor Idle ----------|                      |
```

### 13.3 Priority Levels

| Priority | Use Case | Preemption |
|----------|----------|------------|
| Emergency | Critical alerts | Always preempts |
| High | Supervisor, dispatch | Preempts normal |
| Normal | Standard users | No preemption |
| Low | Background info | Easily preempted |

### 13.4 Late Join

Users joining mid-transmission:

**Strategies:**
- **Immediate join**: Hear ongoing transmission
- **Buffer replay**: Replay last N seconds
- **Wait for next**: Wait until floor released

### 13.5 Implementation

```javascript
class PushToTalk {
  constructor(config) {
    this.channel = config.channel;
    this.priority = config.priority || 'normal';
    this.floorState = 'idle';
  }

  requestFloor() {
    this.send({
      type: 'floor-request',
      channel: this.channel,
      priority: this.priority
    });
  }

  releaseFloor() {
    this.send({
      type: 'floor-release',
      channel: this.channel
    });
    this.floorState = 'idle';
  }

  onFloorGranted() {
    this.floorState = 'granted';
    this.startAudioTransmission();
  }

  onFloorDenied(reason) {
    this.floorState = 'denied';
    console.log('Floor denied:', reason);
  }
}
```

---



## A.1 Signalling protocol (SIP / WebRTC)

The signalling protocol layer wraps SIP exchanges (RFC 3261) and
WebRTC signalling (driven by browser-side JavaScript) in standardised
envelopes. The standard does not invent a new signalling protocol;
it provides envelope shapes that wrap existing protocol exchanges
for audit and federation.

## A.2 QoE thresholds

The standard recommends QoE thresholds:

| Metric | Acceptable | Excellent |
|--------|------------|-----------|
| One-way mouth-to-ear latency | ≤ 150 ms | ≤ 100 ms |
| Packet loss | ≤ 1% | ≤ 0.1% |
| Jitter | ≤ 30 ms | ≤ 10 ms |
| MOS (Mean Opinion Score) | ≥ 3.5 | ≥ 4.0 |

QoE telemetry envelopes carry these metrics; hosts aggregate per-call
to detect degradation patterns.

## A.3 Low-latency streaming

Low-latency streaming (LL-HLS, DASH-CMAF-LL, WebRTC-based) targets
sub-second glass-to-glass latency. The protocol exchanges carry the
chunk identifier, the server timestamp, and the receive timestamp so
end-to-end latency is measurable from envelopes alone.

## A.4 Video conferencing

The video-conferencing protocol layer adds: simulcast (multiple
resolutions per sender), SVC (scalable video coding) layer
selection, and active-speaker signalling. Each protocol exchange
is signed and replay-protected per the standard's general discipline.

## A.5 Push-to-talk

Push-to-talk semantics require: floor-control (one speaker at a
time), low-latency squelch (the receiver hears the speaker within
500 ms of the speaker pressing PTT), and group membership management.
The protocol exchanges carry these primitives with the standard
replay defence.


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
