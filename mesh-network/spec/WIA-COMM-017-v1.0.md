# WIA-COMM-017: Mesh Network Specification v1.0

> **Standard ID:** WIA-COMM-017
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communications Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Mesh Network Fundamentals](#2-mesh-network-fundamentals)
3. [Routing Protocols](#3-routing-protocols)
4. [WiFi Mesh (802.11s)](#4-wifi-mesh-80211s)
5. [Bluetooth Mesh](#5-bluetooth-mesh)
6. [Thread/Zigbee Mesh](#6-threadzigbee-mesh)
7. [LoRa Mesh Networks](#7-lora-mesh-networks)
8. [Self-Healing Mechanisms](#8-self-healing-mechanisms)
9. [Node Discovery and Pairing](#9-node-discovery-and-pairing)
10. [Quality of Service (QoS)](#10-quality-of-service-qos)
11. [Security](#11-security)
12. [Implementation Guidelines](#12-implementation-guidelines)
13. [Performance Optimization](#13-performance-optimization)
14. [References](#14-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for mesh networking technologies, enabling decentralized, self-healing, and scalable network infrastructures across multiple protocols and use cases.

### 1.2 Scope

The standard covers:
- Mesh topology design and optimization
- Multi-hop routing protocols (AODV, OLSR, BATMAN)
- WiFi mesh networking (IEEE 802.11s)
- Bluetooth mesh architecture
- IoT mesh protocols (Thread, Zigbee)
- LoRa mesh networks
- Self-healing and redundancy mechanisms
- Node discovery and management
- Bandwidth and latency optimization
- Security and encryption

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize network access by enabling resilient, low-cost mesh networks that can bring connectivity to underserved communities and enhance network reliability globally.

### 1.4 Terminology

- **Mesh Node**: A network device that can route traffic for other nodes
- **Mesh Router**: A node that forwards packets between other nodes
- **Mesh Gateway**: A node that provides external network access
- **Mesh Client**: A device that connects to the mesh but doesn't route
- **Hop**: A single transmission between two adjacent nodes
- **Path**: A series of hops from source to destination
- **Neighbor**: An adjacent node within direct communication range

---

## 2. Mesh Network Fundamentals

### 2.1 Mesh Topology Types

#### 2.1.1 Full Mesh
Every node connects to every other node:

```
Number of connections = n × (n - 1) / 2
```

Where `n` is the number of nodes.

**Advantages**: Maximum redundancy, lowest latency
**Disadvantages**: High complexity, bandwidth overhead

#### 2.1.2 Partial Mesh
Nodes connect to a subset of other nodes:

```
Minimum neighbors per node = 3 (recommended)
```

**Advantages**: Balance of redundancy and efficiency
**Disadvantages**: Potential single points of failure

#### 2.1.3 Hybrid Mesh
Combination of wired backbone and wireless mesh:

```
Gateway nodes ←→ Wired backbone
     ↕
Mesh nodes ←→ Wireless mesh
```

### 2.2 Network Metrics

#### 2.2.1 Path Cost
Calculate the cost of a path based on multiple factors:

```
PathCost = Σ(LinkCost_i)

LinkCost = α × (1/Bandwidth) + β × Latency + γ × PacketLoss
```

Where:
- `α, β, γ` = Weighting factors (sum to 1.0)
- `Bandwidth` = Link bandwidth in Mbps
- `Latency` = Link latency in ms
- `PacketLoss` = Packet loss rate (0-1)

Recommended weights:
- `α = 0.4` (bandwidth weight)
- `β = 0.3` (latency weight)
- `γ = 0.3` (packet loss weight)

#### 2.2.2 Network Diameter
Maximum number of hops between any two nodes:

```
Diameter = max(hops(i, j)) for all node pairs i, j
```

**Recommended**: Diameter ≤ 10 hops

#### 2.2.3 Node Degree
Number of direct neighbors per node:

```
Degree(node) = |{neighbors of node}|
```

**Recommended**: 3 ≤ Degree ≤ 8

### 2.3 Network Capacity

#### 2.3.1 Theoretical Capacity
For a mesh network with `n` nodes:

```
C_total = n × C_node / (avg_hops + 1)
```

Where:
- `C_total` = Total network capacity
- `C_node` = Single node capacity
- `avg_hops` = Average number of hops

#### 2.3.2 Effective Throughput
Accounting for routing overhead:

```
T_effective = T_raw × (1 - overhead)

overhead = routing_traffic / total_traffic
```

Typical overhead: 10-30%

---

## 3. Routing Protocols

### 3.1 AODV (Ad hoc On-Demand Distance Vector)

#### 3.1.1 Route Discovery
When node S needs a route to destination D:

1. **Route Request (RREQ)**:
   - S broadcasts RREQ packet
   - Intermediate nodes forward until D is reached
   - Each node records reverse path

2. **Route Reply (RREP)**:
   - D sends RREP back to S via reverse path
   - Establishes forward route to D

3. **Route Maintenance**:
   - Active routes monitored via HELLO messages
   - Failed links trigger route error (RERR)
   - Broken routes removed from routing tables

#### 3.1.2 AODV Packet Format

```
RREQ: [Type | Hop Count | RREQ ID | Dest Addr | Dest Seq | Src Addr | Src Seq]
RREP: [Type | Hop Count | Dest Addr | Dest Seq | Src Addr | Lifetime]
RERR: [Type | Dest Count | Unreachable Dest List]
```

#### 3.1.3 Routing Table Entry

```
{
  destination: IP address,
  next_hop: IP address,
  hop_count: number,
  sequence_number: number,
  lifetime: timestamp,
  precursor_list: [IP addresses]
}
```

### 3.2 OLSR (Optimized Link State Routing)

#### 3.2.1 Multipoint Relays (MPRs)
Each node selects a subset of neighbors as MPRs:

```
MPR(node) = minimal set of neighbors covering all 2-hop neighbors
```

#### 3.2.2 HELLO Messages
Broadcast periodically to discover neighbors:

```
HELLO: [
  willingness: 0-7,
  neighbor_list: [
    {address, link_type, neighbor_type}
  ]
]
```

Interval: 2 seconds (default)

#### 3.2.3 Topology Control (TC) Messages
MPRs broadcast topology information:

```
TC: [
  advertised_neighbor_seq: number,
  neighbor_list: [addresses]
]
```

Interval: 5 seconds (default)

#### 3.2.4 Routing Table Computation
Use Dijkstra's algorithm on topology graph:

```
1. Build topology graph from TC messages
2. Compute shortest paths to all destinations
3. Update routing table with next hops
4. Recompute on topology changes
```

### 3.3 BATMAN (Better Approach To Mobile Adhoc Networking)

#### 3.3.1 Originator Messages (OGM)
Each node broadcasts OGMs periodically:

```
OGM: [
  originator: address,
  sequence_number: number,
  ttl: number,
  tq: transmit quality (0-255)
]
```

Interval: 1 second (default)

#### 3.3.2 Transmit Quality (TQ) Metric
Bidirectional link quality calculation:

```
TQ_new = TQ_received × (local_link_quality / 255)

local_link_quality = (received_ogms / total_ogms) × 255
```

#### 3.3.3 Best Path Selection
Choose path with highest TQ value:

```
best_path = argmax(TQ_path)

TQ_path = product of all link TQ values on path
```

#### 3.3.4 Route Switching
Switch to better route when TQ difference exceeds threshold:

```
if TQ_new > TQ_current × (1 + hysteresis):
    switch_route(new_path)
```

Recommended hysteresis: 0.20 (20%)

---

## 4. WiFi Mesh (802.11s)

### 4.1 IEEE 802.11s Overview

WiFi mesh operates on standard WiFi frequencies:
- 2.4 GHz: Channels 1-13
- 5 GHz: Channels 36-165

### 4.2 Mesh Peering Management

#### 4.2.1 Peer Link Establishment
Three-way handshake:

```
Node A → Node B: Peer Link Open
Node B → Node A: Peer Link Confirm
Node A → Node B: Peer Link Confirm
```

#### 4.2.2 Mesh Security Association (MSA)
When using SAE (Simultaneous Authentication of Equals):

```
1. SAE authentication frames exchanged
2. Pairwise Master Key (PMK) derived
3. 4-way handshake for temporal keys
4. Secure mesh link established
```

### 4.3 HWMP (Hybrid Wireless Mesh Protocol)

#### 4.3.1 Proactive Mode (Tree-Based)
Root node announces path:

```
Root → PREQ (Proactive Path Request)
Nodes → PREP (Path Reply) to root
```

#### 4.3.2 Reactive Mode (On-Demand)
Similar to AODV:

```
Source → PREQ broadcast
Destination → PREP unicast
```

#### 4.3.3 Airtime Link Metric
Standard metric for path selection:

```
C_a = [O + B_t / r] × 1 / (1 - e_f)
```

Where:
- `O` = Channel access overhead (fixed)
- `B_t` = Test frame length (8192 bits)
- `r` = Data rate in Mbps
- `e_f` = Frame error rate

### 4.4 Mesh Configuration

#### 4.4.1 Mesh Profile
```json
{
  "mesh_id": "MyMeshNetwork",
  "channel": 6,
  "channel_width": 20,
  "frequency": 2437,
  "beacon_interval": 100,
  "dtim_period": 2,
  "max_peer_links": 32,
  "path_refresh_time": 1000,
  "hwmp_active_path_timeout": 5000,
  "hwmp_preq_min_interval": 10,
  "hwmp_net_diameter_traversal_time": 50,
  "hwmp_rootmode": 4,
  "encryption": "sae"
}
```

#### 4.4.2 Recommended Settings

| Parameter | Home | Enterprise | Outdoor |
|-----------|------|------------|---------|
| Channel Width | 20 MHz | 20/40 MHz | 20 MHz |
| Beacon Interval | 100 ms | 100 ms | 100 ms |
| Max Peers | 8-16 | 16-32 | 8-16 |
| Path Refresh | 1000 ms | 500 ms | 2000 ms |

---

## 5. Bluetooth Mesh

### 5.1 Bluetooth Mesh Architecture

#### 5.1.1 Mesh Layers
```
Applications
    ↕
Models (Generic OnOff, Sensor, etc.)
    ↕
Foundation Models (Configuration, Health)
    ↕
Access Layer (Message encryption/decryption)
    ↕
Upper Transport Layer (Segmentation/reassembly)
    ↕
Lower Transport Layer (Encryption)
    ↕
Network Layer (Routing, relay)
    ↕
Bearer Layer (Advertising, GATT)
```

### 5.2 Provisioning

#### 5.2.1 Provisioning Process
```
1. Unprovisioned Device Beacon
2. Provisioner sends Provisioning Invite
3. Device capabilities exchange
4. Public key exchange (ECDH)
5. Authentication (OOB or input/output)
6. Distribution of network keys
7. Device added to network
```

#### 5.2.2 Network Keys
- **NetKey**: Encrypts network layer
- **AppKey**: Encrypts application data
- **DevKey**: Unique per device, for configuration

### 5.3 Message Routing

#### 5.3.1 Flooding Algorithm
Messages propagated via managed flooding:

```
1. Node receives message
2. Check message cache (prevent duplicates)
3. If new message:
   a. Process if destined for this node
   b. Relay if TTL > 0
   c. Add to cache
   d. Decrement TTL
   e. Rebroadcast
```

#### 5.3.2 TTL (Time To Live)
Controls message propagation:

```
Initial TTL: 0-127
0 = no relay
127 = maximum propagation
Default: 5-10 hops
```

### 5.4 Addressing

#### 5.4.1 Address Types
- **Unicast**: Single element (0x0001-0x7FFF)
- **Group**: Multiple elements (0xC000-0xFEFF)
- **Virtual**: Label UUID (0x8000-0xBFFF)
- **Unassigned**: 0x0000

#### 5.4.2 Subscription
Elements subscribe to group/virtual addresses:

```json
{
  "element": "0x0012",
  "subscriptions": [
    "0xC001",  // Living Room group
    "0xC002",  // All Lights group
    "virtual-uuid-1234"
  ]
}
```

### 5.5 Low Power Nodes (LPN)

#### 5.5.1 Friendship
LPN pairs with Friend node for power savings:

```
LPN → Friend Request
Friend → Friend Offer
LPN → Friend Poll (periodically)
Friend → Cached messages
```

#### 5.5.2 Poll Interval
```
Poll Interval = ReceiveDelay + PollTimeout

Typical values:
- ReceiveDelay: 10-100 ms
- PollTimeout: 1-60 seconds
```

---

## 6. Thread/Zigbee Mesh

### 6.1 Thread Network Architecture

#### 6.1.1 Device Roles
- **Leader**: Manages router ID assignment
- **Router**: Full Thread Device (FTD), routes packets
- **Router Eligible End Device (REED)**: Can become router
- **End Device**: Minimal Thread Device (MTD)
- **Border Router**: Connects to external networks

#### 6.1.2 Network Topology
```
Internet
   ↕
Border Router (Leader)
   ↕
Routers (mesh topology)
   ↕
End Devices (star topology)
```

### 6.2 Thread Routing

#### 6.2.1 MLE (Mesh Link Establishment)
Neighbor discovery and link quality assessment:

```
MLE Advertisement: [
  leader_data,
  route_data,
  network_data
]

Interval: 32 seconds (default)
```

#### 6.2.2 Router Selection
Devices become routers based on network needs:

```
if (network.routers < MAX_ROUTERS) AND (link_quality > THRESHOLD):
    upgrade_to_router()

MAX_ROUTERS = 32 (Thread spec)
```

#### 6.2.3 Path Cost Calculation
```
Cost = (1 + out_cost + in_cost) × link_margin_factor

link_margin_factor = 1.0 / (1.0 + link_margin/128)
```

### 6.3 Network Formation

#### 6.3.1 Network Parameters
```json
{
  "network_name": "ThreadNet",
  "extended_pan_id": "0x0123456789ABCDEF",
  "pan_id": "0x1234",
  "channel": 15,
  "network_key": "128-bit key",
  "mesh_local_prefix": "fd00::/64"
}
```

#### 6.3.2 Commissioning
```
1. Joiner discovers Commissioner
2. DTLS session established
3. Network credentials transferred
4. Joiner joins network
5. Joiner becomes End Device or Router
```

### 6.4 IPv6 Addressing

#### 6.4.1 Address Types
- **Link-Local**: fe80::/64
- **Mesh-Local**: fd00::/64 (ULA)
- **Global**: 2000::/3 (via Border Router)

#### 6.4.2 Address Assignment
```
Mesh-Local: fd<mesh-local-prefix>::<IID>

IID = Interface Identifier (64-bit)
Derived from: MAC address or random
```

---

## 7. LoRa Mesh Networks

### 7.1 LoRa Physical Layer

#### 7.1.1 Modulation Parameters
```
Spreading Factor (SF): 7-12
Bandwidth (BW): 125, 250, 500 kHz
Coding Rate (CR): 4/5, 4/6, 4/7, 4/8
```

#### 7.1.2 Data Rate Calculation
```
R_b = SF × (BW / 2^SF) × CR

Example (SF=7, BW=125kHz, CR=4/5):
R_b = 7 × (125000 / 128) × 0.8 = 5468 bps
```

#### 7.1.3 Range Estimation
```
Range = 10^((P_tx - P_rx - L_0) / (10 × n))

Where:
- P_tx = Transmit power (dBm)
- P_rx = Receiver sensitivity (dBm)
- L_0 = Path loss at 1m reference
- n = Path loss exponent (2-4)
```

Typical range:
- Urban: 2-5 km
- Suburban: 5-10 km
- Rural: 10-15 km

### 7.2 LoRa Mesh Routing

#### 7.2.1 Distance Vector Protocol
Modified AODV for LoRa constraints:

```
Route Discovery:
1. Source broadcasts RREQ with SF12 (max range)
2. Nodes forward RREQ, record reverse path
3. Destination replies with RREP
4. Data sent with optimal SF for each hop
```

#### 7.2.2 Adaptive SF Selection
```
SF = SF_min + floor(distance / hop_distance)

SF_min = 7
SF_max = 12
hop_distance = max_range / (SF_max - SF_min)
```

#### 7.2.3 Duty Cycle Management
Comply with regional regulations:

```
Europe (868 MHz): 1% duty cycle
US (915 MHz): No duty cycle limit (but FCC power limits)
Asia (433 MHz): Varies by country

Tx_time_max = 3600000 × duty_cycle  // ms per hour
```

### 7.3 Energy Optimization

#### 7.3.1 Sleep Scheduling
```
Wake_interval = Header_time + Rx_window + Processing_time

Recommended:
- Header_time: 100-500 ms (preamble detection)
- Rx_window: 1-5 seconds
- Sleep_duration: 60-300 seconds
```

#### 7.3.2 Power Consumption
```
Battery_life = Battery_capacity / Average_current

Average_current = (I_tx × D_tx + I_rx × D_rx + I_sleep × D_sleep)

Typical values:
- I_tx: 120 mA (at +14 dBm)
- I_rx: 12 mA
- I_sleep: 1-10 µA
```

### 7.4 LoRa Mesh Packet Format

```
[Preamble | Header | Payload | CRC]

Header:
- Destination ID (2 bytes)
- Source ID (2 bytes)
- Hop count (1 byte)
- Sequence number (2 bytes)
- Payload length (1 byte)

Payload: Up to 255 bytes
```

---

## 8. Self-Healing Mechanisms

### 8.1 Link Failure Detection

#### 8.1.1 Heartbeat Mechanism
```
Heartbeat_interval = 1-5 seconds
Failure_threshold = 3-5 missed heartbeats

if (missed_heartbeats >= threshold):
    declare_link_failed()
    trigger_route_repair()
```

#### 8.1.2 Proactive Monitoring
```
Monitor metrics:
- RSSI (Received Signal Strength Indicator)
- LQI (Link Quality Indicator)
- Packet loss rate
- Round-trip time (RTT)

if (metric < threshold):
    initiate_route_diversification()
```

### 8.2 Route Repair

#### 8.2.1 Local Repair (AODV)
```
1. Detect link failure
2. Upstream node initiates local RREQ
3. Search for alternative path within k hops
4. If found: Update route, send RREP to source
5. If not found: Send RERR to source
```

Recommended k = 2-3 hops

#### 8.2.2 Path Redundancy
Maintain multiple paths to critical destinations:

```
Primary path: Shortest hop count
Backup path 1: Minimal overlap with primary
Backup path 2: Maximally disjoint from primary and backup 1

Switch to backup if:
- Primary fails
- Primary quality degrades significantly
```

### 8.3 Network Partitioning

#### 8.3.1 Partition Detection
```
1. Each node maintains neighbor list
2. Nodes exchange reachability information
3. Build reachable node set
4. If (reachable_nodes < expected_nodes):
     partition_detected = true
```

#### 8.3.2 Partition Healing
```
1. Detect partition
2. Identify border nodes
3. Increase transmission power (if possible)
4. Use mobile nodes to bridge partitions
5. Deploy additional relay nodes
6. Merge partitions when contact restored
```

### 8.4 Congestion Management

#### 8.4.1 Congestion Detection
```
Congestion_metric = (Queue_length / Queue_capacity) + (Packet_loss_rate)

if (Congestion_metric > threshold):
    trigger_congestion_control()

Threshold: 0.7-0.8
```

#### 8.4.2 Congestion Mitigation
```
1. Rate limiting: Reduce source transmission rate
2. Load balancing: Distribute traffic across multiple paths
3. Priority queuing: Prioritize critical traffic
4. Backpressure: Signal upstream nodes to slow down
```

---

## 9. Node Discovery and Pairing

### 9.1 Discovery Mechanisms

#### 9.1.1 Passive Scanning
```
1. Listen on all channels
2. Collect beacon frames
3. Build neighbor table with RSSI
4. Select best channel and neighbors
```

Scan duration per channel: 50-200 ms

#### 9.1.2 Active Scanning
```
1. Send probe request on each channel
2. Wait for probe responses
3. Collect network information
4. Select network to join
```

#### 9.1.3 Neighbor Table Entry
```json
{
  "neighbor_id": "0xABCD",
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "rssi": -65,
  "lqi": 200,
  "last_seen": 1234567890,
  "capabilities": ["routing", "gateway"],
  "hop_count": 2
}
```

### 9.2 Pairing Protocols

#### 9.2.1 WiFi Mesh Peering
```
1. Passive scanning finds mesh networks
2. Select mesh network (by mesh ID)
3. Authenticate (SAE or open)
4. Peer link management (PLM) handshake
5. Establish secure connection
```

#### 9.2.2 Bluetooth Mesh Provisioning
```
1. Scan for unprovisioned beacons
2. Provisioner sends invite
3. Exchange capabilities
4. ECDH public key exchange
5. OOB authentication (optional)
6. Network key distribution
7. Node configured and joined
```

#### 9.2.3 Thread Commissioning
```
1. Joiner discovers Commissioner
2. DTLS session with PSKd
3. Network credentials transferred
4. MLE attach to Thread network
5. Address assignment
6. Routing table population
```

### 9.3 Security Pairing

#### 9.3.1 SAE (Simultaneous Authentication of Equals)
```
1. Commit phase: Exchange commit messages
2. Confirm phase: Exchange confirm messages
3. PMK derivation: Both derive shared PMK
4. 4-way handshake: Derive temporal keys
```

#### 9.3.2 Out-of-Band (OOB) Authentication
Options:
- **Static OOB**: Pre-shared PIN or passphrase
- **Numeric Comparison**: User compares displayed numbers
- **Passkey Entry**: User enters displayed passkey
- **NFC**: Near-field communication transfer
- **QR Code**: Scan authentication data

---

## 10. Quality of Service (QoS)

### 10.1 Traffic Classification

#### 10.1.1 Priority Levels
```
0: Background (bulk data)
1: Best Effort (default)
2: Excellent Effort (business traffic)
3: Critical Applications (VoIP, video)
4: Video (< 100 ms latency)
5: Voice (< 10 ms latency)
6: Internetwork Control
7: Network Control
```

#### 10.1.2 Traffic Marking
```
DSCP (Differentiated Services Code Point):
- EF (Expedited Forwarding): Voice
- AF4x: Video
- AF3x: Critical data
- BE (Best Effort): Default
```

### 10.2 Queue Management

#### 10.2.1 Priority Queuing
```
Queues (highest to lowest priority):
1. Voice/Control (strict priority)
2. Video (strict priority)
3. Critical data (WFQ)
4. Best effort (WFQ)
5. Background (WFQ if bandwidth available)

WFQ = Weighted Fair Queuing
```

#### 10.2.2 Active Queue Management
```
RED (Random Early Detection):

if (queue_length < min_threshold):
    accept_packet()
elif (queue_length > max_threshold):
    drop_packet()
else:
    probability = (queue_length - min_threshold) / (max_threshold - min_threshold)
    if (random() < probability):
        drop_packet()
    else:
        accept_packet()
```

### 10.3 Bandwidth Allocation

#### 10.3.1 Weighted Fair Sharing
```
Bandwidth_i = Total_bandwidth × (Weight_i / Σ Weights)

Recommended weights:
- Voice: 30%
- Video: 30%
- Critical data: 25%
- Best effort: 15%
```

#### 10.3.2 Admission Control
```
if (requested_bandwidth + current_usage > threshold × total_bandwidth):
    reject_request()
else:
    accept_request()
    allocate_bandwidth()

Threshold: 0.80-0.90 (leave headroom)
```

### 10.4 Latency Optimization

#### 10.4.1 Fast Path Forwarding
```
1. Classify high-priority packets
2. Bypass complex processing
3. Use hardware acceleration
4. Minimal queuing delay
```

#### 10.4.2 Multi-Path Load Balancing
```
1. Identify multiple paths to destination
2. Hash flow identifier to select path
3. Monitor path quality metrics
4. Rebalance flows on quality degradation
```

---

## 11. Security

### 11.1 Encryption

#### 11.1.1 WiFi Mesh Security (WPA3-SAE)
```
Authentication: SAE (Dragonfly handshake)
Encryption: AES-CCMP-128 or AES-GCMP-256
Key Management: PMK caching, opportunistic PMKSA caching
```

#### 11.1.2 Bluetooth Mesh Security
```
Network Encryption: AES-CCM with NetKey
Application Encryption: AES-CCM with AppKey
Provisioning: ECDH + AES-CCM
Replay Protection: Sequence numbers
```

#### 11.1.3 Thread Security
```
Commissioning: DTLS 1.2 with PSKd
Network: AES-128-CCM
Key Management: Network key rotation
```

### 11.2 Authentication

#### 11.2.1 Node Authentication
```
1. Device presents credentials
2. Network verifies identity
3. Challenge-response exchange
4. Session keys established
```

#### 11.2.2 Message Authentication
```
MAC = HMAC-SHA256(Key, Message)

Verify:
if (received_MAC == computed_MAC):
    accept_message()
else:
    drop_message()
```

### 11.3 Intrusion Detection

#### 11.3.1 Anomaly Detection
Monitor for suspicious behavior:
- Unusual traffic patterns
- Excessive route requests
- Malformed packets
- Replay attacks
- Flooding attacks

#### 11.3.2 Countermeasures
```
Rate limiting:
if (requests_per_second > threshold):
    blacklist_node(duration)

Packet validation:
if (not valid_packet_structure):
    drop_packet()
    log_incident()
```

### 11.4 Key Management

#### 11.4.1 Key Hierarchy
```
Master Key (permanent)
    ↓
Network Key (rotated periodically)
    ↓
Session Keys (per connection)
```

#### 11.4.2 Key Rotation
```
Rotation_interval = 30-90 days

Rotation process:
1. Generate new key
2. Distribute to all nodes
3. Activate new key at scheduled time
4. Deprecate old key
5. Remove old key after grace period
```

---

## 12. Implementation Guidelines

### 12.1 Required Components

Any WIA-COMM-017 compliant system must include:

1. **Node Management**: Create, configure, and manage mesh nodes
2. **Routing Engine**: Implement at least one routing protocol
3. **Discovery Service**: Node and neighbor discovery
4. **Security Module**: Encryption and authentication
5. **QoS Manager**: Traffic prioritization and bandwidth management
6. **Monitoring System**: Network health and performance tracking

### 12.2 API Interface

#### 12.2.1 Create Mesh Node
```typescript
interface MeshNodeConfig {
  id: string;
  protocol: 'wifi-mesh' | 'bluetooth-mesh' | 'thread' | 'lora-mesh';
  role: 'router' | 'gateway' | 'end-device';
  capabilities: string[];
  security: SecurityConfig;
}

interface MeshNode {
  id: string;
  config: MeshNodeConfig;
  neighbors: Neighbor[];
  routes: Route[];
  status: 'active' | 'inactive' | 'failed';
}
```

#### 12.2.2 Route Packet
```typescript
interface RoutingRequest {
  source: string;
  destination: string;
  data: Buffer;
  priority: 'low' | 'medium' | 'high' | 'critical';
  maxHops?: number;
  qos?: QoSConfig;
}

interface RoutingResult {
  success: boolean;
  path: string[];  // Node IDs in path
  latency: number;  // ms
  bandwidth: number;  // Mbps
  hopCount: number;
}
```

#### 12.2.3 Discover Peers
```typescript
interface DiscoveryConfig {
  protocol: string;
  timeout: number;  // ms
  channels?: number[];
  rssiThreshold?: number;  // dBm
}

interface Peer {
  id: string;
  address: string;
  rssi: number;
  lqi: number;
  capabilities: string[];
  lastSeen: Date;
}
```

### 12.3 Data Formats

#### 12.3.1 Node Information
```json
{
  "id": "node-001",
  "protocol": "wifi-mesh",
  "role": "gateway",
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "ip_address": "192.168.1.100",
  "mesh_address": "0x1234",
  "capabilities": ["routing", "gateway", "internet"],
  "neighbors": [
    {
      "id": "node-002",
      "rssi": -65,
      "lqi": 200,
      "hop_count": 1
    }
  ],
  "routes": [
    {
      "destination": "node-050",
      "next_hop": "node-002",
      "metric": 150,
      "hop_count": 3
    }
  ],
  "status": "active"
}
```

#### 12.3.2 Network Topology
```json
{
  "network_id": "MyMeshNet",
  "protocol": "wifi-mesh",
  "nodes": ["node-001", "node-002", "node-003"],
  "links": [
    {
      "from": "node-001",
      "to": "node-002",
      "quality": 0.95,
      "bandwidth": 50.0,
      "latency": 5.2
    }
  ],
  "gateways": ["node-001"],
  "diameter": 5,
  "avg_hop_count": 2.3
}
```

### 12.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| M001 | Node not found | Check node ID |
| M002 | Route not available | Initiate route discovery |
| M003 | Insufficient bandwidth | Use QoS or wait |
| M004 | Security failure | Re-authenticate |
| M005 | Network partition | Bridge partition |
| M006 | Hop limit exceeded | Adjust max hops |
| M007 | Invalid protocol | Check protocol support |

---

## 13. Performance Optimization

### 13.1 Topology Optimization

#### 13.1.1 Node Placement
```
Optimal spacing = 0.6 × max_range

Coverage overlap: 2-3 nodes per area
```

#### 13.1.2 Channel Selection
```
Auto-channel algorithm:
1. Scan all channels
2. Measure interference per channel
3. Select channel with minimum interference
4. Re-scan periodically (every 1-24 hours)
```

### 13.2 Routing Optimization

#### 13.2.1 Route Caching
```
Cache entries: 100-1000 routes
TTL: 60-300 seconds
Eviction: LRU (Least Recently Used)
```

#### 13.2.2 Multicast Optimization
```
1. Build multicast tree (Steiner tree)
2. Use MPRs (OLSR) for efficient forwarding
3. Prune inactive branches
4. Minimize tree cost
```

### 13.3 Power Optimization

#### 13.3.1 Transmission Power Control
```
P_tx = P_min + margin + (path_loss - P_rx_sensitivity)

Adjust P_tx based on:
- Distance to neighbor
- Required link quality
- Interference level
```

#### 13.3.2 Sleep Scheduling
```
Coordinated sleep:
1. Nodes announce sleep schedule
2. Neighbors adjust routing
3. Wake at scheduled intervals
4. Process queued messages
5. Return to sleep
```

### 13.4 Bandwidth Optimization

#### 13.4.1 Aggregation
```
Aggregate small packets:
if (packet_size < threshold) AND (queue_not_empty):
    aggregate_packets()
    transmit_aggregated_frame()
```

#### 13.4.2 Compression
```
Header compression:
- Compress IPv6 headers (6LoWPAN)
- Compress common protocol fields
- Use context-based compression

Payload compression:
- GZIP for bulk data
- Protocol-specific compression
```

---

## 14. References

### 14.1 Standards Documents

1. IEEE 802.11s-2011: "Mesh Networking"
2. Bluetooth Mesh Profile Specification v1.0
3. Thread Specification v1.3
4. ZigBee Specification r22
5. LoRaWAN Specification v1.0.4
6. RFC 3561: AODV
7. RFC 3626: OLSR
8. BATMAN-adv Protocol Documentation

### 14.2 Network Parameters

| Parameter | Symbol | Typical Value |
|-----------|--------|---------------|
| WiFi Range | R_wifi | 100-300 m |
| BLE Range | R_ble | 10-50 m |
| Thread Range | R_thread | 10-100 m |
| LoRa Range | R_lora | 2-15 km |
| Max Hops | H_max | 10 |
| Routing Overhead | O_r | 10-30% |

### 14.3 WIA Standards

- WIA-INTENT: Intent-based network configuration
- WIA-OMNI-API: Universal API gateway
- WIA-IOT: IoT device standards
- WIA-SECURITY: Security protocols
- WIA-5G-6G: Next-generation cellular

---

## Appendix A: Example Calculations

### A.1 Path Cost Calculation

```
Given path: A → B → C → D

Link AB: BW=100Mbps, Latency=5ms, Loss=0.01
Link BC: BW=50Mbps, Latency=10ms, Loss=0.02
Link CD: BW=75Mbps, Latency=7ms, Loss=0.01

Weights: α=0.4, β=0.3, γ=0.3

Cost_AB = 0.4×(1/100) + 0.3×5 + 0.3×0.01 = 1.507
Cost_BC = 0.4×(1/50) + 0.3×10 + 0.3×0.02 = 3.014
Cost_CD = 0.4×(1/75) + 0.3×7 + 0.3×0.01 = 2.108

Total Cost = 1.507 + 3.014 + 2.108 = 6.629
```

### A.2 LoRa Data Rate

```
Configuration:
- SF = 10
- BW = 125 kHz
- CR = 4/5

Calculation:
R_b = 10 × (125000 / 1024) × 0.8
R_b = 10 × 122.07 × 0.8
R_b = 976.56 bps ≈ 977 bps
```

### A.3 Battery Life Estimation

```
LoRa node:
- Battery: 2000 mAh @ 3.3V
- Tx current: 120 mA, 1s per minute
- Rx current: 12 mA, 5s per minute
- Sleep current: 5 µA, 54s per minute

Average current:
I_avg = (120×1 + 12×5 + 0.005×54) / 60
I_avg = (120 + 60 + 0.27) / 60
I_avg = 3.00 mA

Battery life:
T = 2000 mAh / 3.00 mA = 667 hours ≈ 28 days
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-COMM-017 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
