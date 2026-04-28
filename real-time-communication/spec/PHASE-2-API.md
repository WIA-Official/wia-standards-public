# WIA-COMM-019 — Phase 2: API

> Media-transport API surface: RTP/RTCP, SRTP encryption, jitter buffer management, codec negotiation, and NAT traversal — each presented as a worked endpoint specification with operational guidance.

## 4. RTP/RTCP Media Transport

### 4.1 RTP (Real-time Transport Protocol)

RTP provides end-to-end delivery for real-time data with sequencing and timestamping.

#### 4.1.1 RTP Header Format

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|V=2|P|X|  CC   |M|     PT      |       Sequence Number         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                           Timestamp                           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|           Synchronization Source (SSRC) Identifier            |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|            Contributing Source (CSRC) Identifiers             |
|                             ....                              |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

**Fields:**
- **V (Version)**: RTP version (2)
- **P (Padding)**: Padding flag
- **X (Extension)**: Header extension flag
- **CC (CSRC Count)**: Number of CSRC identifiers
- **M (Marker)**: Marker bit (frame boundary)
- **PT (Payload Type)**: Codec identifier
- **Sequence Number**: Packet sequence (16-bit)
- **Timestamp**: Media timestamp (32-bit)
- **SSRC**: Synchronization source identifier
- **CSRC**: Contributing source identifiers (for mixers)

#### 4.1.2 Payload Types

Common payload types:

| PT | Encoding | Media Type | Clock Rate |
|----|----------|------------|------------|
| 0 | PCMU | Audio | 8000 Hz |
| 3 | GSM | Audio | 8000 Hz |
| 4 | G723 | Audio | 8000 Hz |
| 8 | PCMA | Audio | 8000 Hz |
| 9 | G722 | Audio | 8000 Hz |
| 18 | G729 | Audio | 8000 Hz |
| 96-127 | Dynamic | Audio/Video | Varies |

**Dynamic Payload Types** (96-127) are negotiated via SDP for:
- Opus (typically 111)
- VP8 (typically 96)
- VP9 (typically 98)
- H.264 (typically 102)

### 4.2 RTCP (RTP Control Protocol)

RTCP provides feedback on media quality and participant information.

#### 4.2.1 RTCP Packet Types

| Type | Name | Description |
|------|------|-------------|
| 200 | SR | Sender Report |
| 201 | RR | Receiver Report |
| 202 | SDES | Source Description |
| 203 | BYE | Goodbye |
| 204 | APP | Application-defined |
| 205 | RTPFB | Transport Layer Feedback |
| 206 | PSFB | Payload-specific Feedback |

#### 4.2.2 Sender Report (SR)

Includes transmission statistics:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|V=2|P|    RC   |   PT=SR=200   |             length            |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                         SSRC of sender                        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|              NTP timestamp, most significant word             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|             NTP timestamp, least significant word             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                         RTP timestamp                         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     sender's packet count                     |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                      sender's octet count                     |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

#### 4.2.3 Receiver Report (RR)

Includes reception statistics:

- Fraction lost: Packet loss fraction since last report
- Cumulative packets lost: Total packets lost
- Extended highest sequence number: Last sequence + cycles
- Interarrival jitter: Statistical variance of RTP packet arrival
- Last SR timestamp (LSR): NTP timestamp from last SR
- Delay since last SR (DLSR): Delay for RTT calculation

#### 4.2.4 RTCP Extended Reports (XR)

Enhanced quality metrics:
- VoIP Metrics Block (RFC 3611)
- Statistics Summary Block
- Delay Metrics Block
- Discard Metrics Block

### 4.3 RTP Session Multiplexing

**BUNDLE (RFC 8843):**
- Multiplex audio/video on single 5-tuple
- Reduce ICE overhead
- Single DTLS handshake

**RTCP Multiplexing (RFC 5761):**
- Share port for RTP and RTCP
- Differentiate via packet type (RTP PT < 64, RTCP > 127)

---


## 5. SRTP Encryption

### 5.1 Overview

SRTP (Secure RTP) provides confidentiality, authentication, and replay protection for RTP/RTCP.

### 5.2 Encryption Algorithms

**Cipher Suites:**

| Suite | Cipher | Auth | Key Length |
|-------|--------|------|------------|
| AES_CM_128_HMAC_SHA1_80 | AES-128-CTR | HMAC-SHA1-80 | 128-bit |
| AES_CM_128_HMAC_SHA1_32 | AES-128-CTR | HMAC-SHA1-32 | 128-bit |
| AEAD_AES_128_GCM | AES-128-GCM | GCM | 128-bit |
| AEAD_AES_256_GCM | AES-256-GCM | GCM | 256-bit |

**Recommended:** AEAD_AES_128_GCM (authenticated encryption)

### 5.3 Key Management

#### 5.3.1 DTLS-SRTP (RFC 5764)

WebRTC uses DTLS for key exchange:

```
Client                                         Server
  |                                               |
  |<============ DTLS Handshake ================>|
  |                                               |
  | ClientHello (use_srtp extension)              |
  |---------------------------------------------->|
  |                                               |
  |                       ServerHello, Certificate|
  |<----------------------------------------------|
  |                                               |
  | Certificate, ClientKeyExchange, Finished      |
  |---------------------------------------------->|
  |                                               |
  |                                      Finished |
  |<----------------------------------------------|
  |                                               |
  |<=========== Derive SRTP Keys ===============>|
  |                                               |
  |<=========== SRTP Media Flow =================>|
```

**Key Derivation:**
- Master key and salt extracted from DTLS
- SRTP keys derived using PRF
- Separate keys for sending/receiving
- Perfect forward secrecy

#### 5.3.2 SDES (RFC 4568)

Legacy key exchange via SDP (not recommended for WebRTC):

```
a=crypto:1 AES_CM_128_HMAC_SHA1_80 \
  inline:PS1uQCVeeCFCanVmcjkpPywjNWhcYD0mXXtxaVBR|2^20|1:32
```

### 5.4 Security Features

**Replay Protection:**
- Replay list or window-based detection
- Sequence number tracking
- Timestamp validation

**Authentication:**
- HMAC-SHA1 or GCM auth tag
- Prevents tampering
- Source validation

**Encryption:**
- AES counter mode or GCM
- Per-packet IV
- Confidentiality of media

---


## 6. Jitter Buffer Management

### 6.1 Purpose

Jitter buffers compensate for network delay variations, ensuring smooth media playback.

### 6.2 Jitter Buffer Types

#### 6.2.1 Fixed Jitter Buffer

Constant delay:

```
Delay = Initial_Offset (e.g., 50ms)
```

**Pros:**
- Simple implementation
- Predictable latency

**Cons:**
- May underflow or overflow
- Not adaptive to network conditions

#### 6.2.2 Adaptive Jitter Buffer

Dynamic delay based on network conditions:

```
Delay = Base_Delay + Adaptive_Component

Adaptive_Component = f(jitter, packet_loss, buffer_occupancy)
```

**Algorithm:**
1. Measure inter-arrival jitter
2. Adjust buffer size dynamically
3. Balance latency vs. quality

**Parameters:**
- `min_delay`: Minimum buffer size (e.g., 20ms)
- `max_delay`: Maximum buffer size (e.g., 200ms)
- `target_delay`: Optimal delay (e.g., 60ms)
- `jitter_threshold`: Trigger for adjustment

### 6.3 Packet Loss Concealment (PLC)

When packets are lost or late:

**Audio PLC:**
- **Interpolation**: Generate audio from adjacent packets
- **Waveform substitution**: Repeat previous waveform
- **Model-based**: Use codec-specific models (Opus PLC)
- **Comfort noise**: Generate background noise

**Video PLC:**
- **Frame freeze**: Repeat last frame
- **Error concealment**: Spatial/temporal interpolation
- **Slice copying**: Copy undamaged slices
- **Motion compensation**: Estimate missing blocks

### 6.4 Time Stretching/Compression

Adjust playback speed to manage buffer:

**Techniques:**
- **WSOLA** (Waveform Similarity Overlap-Add)
- **SOLA** (Synchronous Overlap-Add)
- **Phase vocoder**: Frequency domain modification

**Parameters:**
- Stretch factor: 0.9-1.1 (±10%)
- Segment size: 10-20ms
- Overlap: 25-50%

### 6.5 Voice Activity Detection (VAD)

Detect speech vs. silence:

**Benefits:**
- Skip playout during silence
- Faster buffer adaptation
- Bandwidth savings (discontinuous transmission)

**Algorithms:**
- Energy-based threshold
- Zero-crossing rate
- Spectral entropy
- Machine learning models

---


## 7. Codec Selection and Negotiation

### 7.1 Video Codecs

#### 7.1.1 VP8

Open-source codec from Google (WebM project):

**Features:**
- Royalty-free
- Profile 0-3
- Resolution up to 1080p (practical limit)
- Bitrate range: 100-2000 kbps

**SDP Example:**
```
a=rtpmap:96 VP8/90000
a=rtcp-fb:96 nack
a=rtcp-fb:96 nack pli
a=rtcp-fb:96 ccm fir
```

#### 7.1.2 VP9

Next-generation VP codec:

**Features:**
- 30-50% better compression than VP8
- Resolution up to 8K
- Scalable Video Coding (SVC)
- Profile 0-3

**SDP Example:**
```
a=rtpmap:98 VP9/90000
a=fmtp:98 profile-id=0
a=rtcp-fb:98 nack
a=rtcp-fb:98 nack pli
```

#### 7.1.3 H.264/AVC

Industry standard codec:

**Profiles:**
- **Baseline** (BP): Low complexity, mobile
- **Main** (MP): Broadcast quality
- **High** (HiP): HD/UHD quality

**SDP Example:**
```
a=rtpmap:102 H264/90000
a=fmtp:102 profile-level-id=42e01f;packetization-mode=1
a=rtcp-fb:102 nack
a=rtcp-fb:102 nack pli
a=rtcp-fb:102 ccm fir
```

**profile-level-id:**
- `42e01f`: Baseline Profile, Level 3.1
- `4d001f`: Main Profile, Level 3.1
- `64001f`: High Profile, Level 3.1

#### 7.1.4 H.265/HEVC

High-efficiency codec:

**Features:**
- 50% better compression than H.264
- Resolution up to 8K
- Main, Main 10 profiles

**Licensing:** Patent-encumbered (use with caution)

#### 7.1.5 AV1

Next-generation open codec:

**Features:**
- 30% better than VP9 / H.265
- Royalty-free
- Alliance for Open Media (AOMedia)

**Status:** Emerging support in browsers

### 7.2 Audio Codecs

#### 7.2.1 Opus (Recommended)

Universal audio codec (RFC 6716):

**Features:**
- Bitrate: 6-510 kbps
- Sampling: 8-48 kHz
- Low latency: 5-66.5ms
- Combines SILK (speech) + CELT (music)
- Adaptive mode switching

**SDP Example:**
```
a=rtpmap:111 opus/48000/2
a=fmtp:111 minptime=10;useinbandfec=1;stereo=1
```

**Parameters:**
- `minptime`: Minimum packet time (10ms)
- `useinbandfec`: Forward error correction
- `stereo`: Stereo encoding
- `maxaveragebitrate`: Bitrate limit

#### 7.2.2 G.711 (PCMU/PCMA)

Classic telephony codec:

**Features:**
- 64 kbps fixed bitrate
- 8 kHz sampling
- µ-law (PCMU) in North America
- A-law (PCMA) in Europe

**SDP Example:**
```
a=rtpmap:0 PCMU/8000
a=rtpmap:8 PCMA/8000
```

#### 7.2.3 G.722

Wideband telephony codec:

**Features:**
- 64 kbps bitrate
- 16 kHz sampling (7 kHz bandwidth)
- Better quality than G.711

**SDP Example:**
```
a=rtpmap:9 G722/8000
```

Note: Clock rate is 8000 Hz for historical reasons, but actual sampling is 16 kHz.

### 7.3 Codec Negotiation

#### 7.3.1 SDP Offer/Answer

Codec capabilities exchanged via SDP:

**Offer (Alice):**
```
m=audio 9 UDP/TLS/RTP/SAVPF 111 103 9 0 8
a=rtpmap:111 opus/48000/2
a=rtpmap:103 ISAC/16000
a=rtpmap:9 G722/8000
a=rtpmap:0 PCMU/8000
a=rtpmap:8 PCMA/8000
```

**Answer (Bob):**
```
m=audio 9 UDP/TLS/RTP/SAVPF 111 9
a=rtpmap:111 opus/48000/2
a=rtpmap:9 G722/8000
```

Bob chooses Opus and G.722 from Alice's offer.

#### 7.3.2 Codec Preference

Set codec order in SDP (first = most preferred):

```javascript
const transceivers = pc.getTransceivers();
transceivers.forEach(transceiver => {
  if (transceiver.sender.track.kind === 'video') {
    const codecs = RTCRtpSender.getCapabilities('video').codecs;
    const vp9 = codecs.find(c => c.mimeType === 'video/VP9');
    const vp8 = codecs.find(c => c.mimeType === 'video/VP8');
    const h264 = codecs.find(c => c.mimeType === 'video/H264');
    transceiver.setCodecPreferences([vp9, vp8, h264]);
  }
});
```

### 7.4 Bandwidth Adaptation

Dynamic bitrate adjustment based on network conditions:

**Techniques:**
- **REMB** (Receiver Estimated Maximum Bitrate)
- **TWCC** (Transport-Wide Congestion Control)
- **GCC** (Google Congestion Control)

**SDP:**
```
a=rtcp-fb:96 goog-remb
a=rtcp-fb:96 transport-cc
```

---


## 8. NAT Traversal (STUN/TURN/ICE)

### 8.1 NAT Types

| NAT Type | P2P Connectivity | Description |
|----------|------------------|-------------|
| Full Cone | Excellent | Any external host can send to mapped port |
| Restricted Cone | Good | External host must be previously contacted |
| Port Restricted | Fair | External host+port must be previously contacted |
| Symmetric | Poor | Different mapping for each destination |

### 8.2 STUN (RFC 5389)

Session Traversal Utilities for NAT:

**Purpose:**
- Discover public IP address
- Determine NAT type
- Keep NAT bindings alive

**Message Flow:**
```
Client                              STUN Server
  |                                      |
  |------ Binding Request -------------->|
  |                                      |
  |<----- Binding Response --------------|
  |       (XOR-MAPPED-ADDRESS)           |
```

**Binding Response:**
```json
{
  "type": "binding-response",
  "xor_mapped_address": {
    "ip": "203.0.113.45",
    "port": 54321
  }
}
```

**STUN Servers (Public):**
- `stun:stun.l.google.com:19302`
- `stun:stun1.l.google.com:19302`
- `stun:stun.services.mozilla.com`

### 8.3 TURN (RFC 5766)

Traversal Using Relays around NAT:

**Purpose:**
- Relay media when direct P2P fails
- Support symmetric NATs
- Fallback mechanism

**Message Flow:**
```
Client                         TURN Server                   Peer
  |                                 |                          |
  |--- Allocate Request ----------->|                          |
  |<-- Allocate Success (relayed) --|                          |
  |                                 |                          |
  |--- CreatePermission (peer IP) ->|                          |
  |<-- Success ---------------------|                          |
  |                                 |                          |
  |--- ChannelBind (peer) --------->|                          |
  |<-- Success ---------------------|                          |
  |                                 |                          |
  |=== ChannelData ================>|=== ChannelData =========>|
  |                                 |                          |
  |<== ChannelData =================|<== ChannelData ==========|
```

**TURN Credentials:**
```javascript
{
  urls: 'turn:turn.example.com:3478',
  username: 'user1234',
  credential: 'pass5678',
  credentialType: 'password'
}
```

**Protocols:**
- UDP (default, lowest latency)
- TCP (firewall-friendly)
- TLS (encrypted signaling)

### 8.4 ICE (RFC 8445)

Interactive Connectivity Establishment:

**Process:**
1. **Gather candidates**: host, srflx, relay
2. **Exchange candidates**: Via signaling
3. **Pair candidates**: Create candidate pairs
4. **Check connectivity**: STUN binding checks
5. **Nominate pair**: Select best working pair

**Candidate Types:**

| Type | Description | Priority |
|------|-------------|----------|
| host | Local network interface | High |
| srflx | Server reflexive (STUN) | Medium |
| prflx | Peer reflexive (discovered) | Medium |
| relay | TURN relay | Low |

**Candidate Format:**
```
candidate:1 1 UDP 2130706431 192.168.1.100 54321 typ host
candidate:2 1 UDP 1694498815 203.0.113.45 54322 typ srflx raddr 192.168.1.100 rport 54321
candidate:3 1 UDP 16777215 198.51.100.10 60000 typ relay raddr 203.0.113.45 rport 54322
```

**Fields:**
- Foundation: Candidate group identifier
- Component: 1=RTP, 2=RTCP
- Transport: UDP/TCP
- Priority: Calculated priority
- IP/Port: Connection endpoint
- Type: host/srflx/prflx/relay
- raddr/rport: Related address/port

**Priority Calculation:**
```
priority = (2^24 * type_preference) +
           (2^8 * local_preference) +
           (256 - component_id)
```

**Type Preferences:**
- host: 126
- srflx: 100
- relay: 0

### 8.5 Trickle ICE

Progressive candidate exchange:

**Benefits:**
- Faster connection setup
- Start connectivity checks earlier
- Don't wait for all candidates

**Flow:**
```
Alice                                                       Bob
  |                                                          |
  |------ createOffer() + local candidates (host) --------->|
  |                                                          |
  |                 (continue gathering srflx, relay)        |
  |                                                          |
  |------ additional ICE candidates -------------------->   |
  |                                                          |
  |<----- createAnswer() + Bob's candidates -----------------|
  |                                                          |
  |<----- additional ICE candidates -------------------------|
  |                                                          |
  |<=========== Start connectivity checks ================>|
```

---



## A.1 Endpoint reference (control plane)

```http
POST /rtc/v1/session/start         # initiate a session
POST /rtc/v1/session/{id}/answer   # accept a session
POST /rtc/v1/session/{id}/hangup   # terminate a session
POST /rtc/v1/ice/candidate         # exchange ICE candidates
GET  /rtc/v1/turn/credentials      # request short-lived TURN credentials
```

Every endpoint follows the discovery convention at
`/.well-known/wia-real-time-communication`.

## A.2 NAT traversal endpoint

The TURN credential endpoint mints short-lived credentials for the
session's TURN allocation. Credentials expire in 1 hour; clients
refresh on a documented cadence.

## A.3 Codec negotiation

The session-start endpoint accepts a list of codec preferences
ordered by preference. The host returns the negotiated codec
selection per media direction. The negotiated set MUST be a subset
of both peers' offered codecs; if no overlap exists, the session
fails with a problem document of type `.../codec-negotiation-failed`.

## A.4 QoE telemetry

QoE telemetry envelopes flow from the client to the host on a
documented cadence (typically every 5 seconds). The envelopes carry
RTT, jitter, packet loss, and end-to-end latency estimates. Hosts
aggregate these for per-call quality dashboards.


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
