# WIA-COMM-016 — Phase 2: API Interface

> VPN-protocol canonical Phase 2: API surface (tunnels + key-rotation + peers + telemetry + audit).

# WIA-COMM-016: VPN Protocol Specification v1.0

> **Standard ID:** WIA-COMM-016
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communication Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [IPsec Protocol Suite](#2-ipsec-protocol-suite)
3. [OpenVPN](#3-openvpn)
4. [WireGuard](#4-wireguard)
5. [L2TP/IPsec](#5-l2tpipsec)
6. [SSL/TLS VPN](#6-ssltls-vpn)
7. [VPN Architecture](#7-vpn-architecture)
8. [Key Exchange Mechanisms](#8-key-exchange-mechanisms)
9. [Perfect Forward Secrecy](#9-perfect-forward-secrecy)
10. [Split Tunneling](#10-split-tunneling)
11. [VPN Concentrators](#11-vpn-concentrators)
12. [Performance Optimization](#12-performance-optimization)
13. [Security Considerations](#13-security-considerations)
14. [Implementation Guidelines](#14-implementation-guidelines)
15. [References](#15-references)

---


## 11. VPN Concentrators

### 11.1 Concentrator Overview

VPN concentrators are dedicated devices/servers for handling large-scale VPN deployments.

**Functions:**
- Terminate VPN tunnels
- Authentication and authorization
- Encryption/decryption
- Traffic routing
- Session management

### 11.2 Concentrator Architecture

**Hardware Accelerated:**
```
┌─────────────────────────────┐
│  Management Interface       │
├─────────────────────────────┤
│  VPN Termination Engine     │
│  - Session Management       │
│  - Authentication           │
├─────────────────────────────┤
│  Crypto Accelerator (HW)    │
│  - AES-NI                   │
│  - Dedicated Crypto Chip    │
├─────────────────────────────┤
│  Network Interfaces         │
│  - 10 GbE / 40 GbE          │
└─────────────────────────────┘
```

**Performance Metrics:**
```
Enterprise Concentrator:
- Concurrent Sessions: 10,000 - 100,000+
- Throughput: 10 - 100 Gbps
- New Sessions/sec: 1,000 - 10,000
- Latency: <1 ms
```

### 11.3 High Availability

**Active-Passive:**
```
[Primary Concentrator] <--> [Standby Concentrator]
         |                         |
         +-------[Heartbeat]-------+
         |
    [VPN Clients]

Failover:
- Heartbeat monitoring
- State synchronization
- Virtual IP failover
- Session preservation
```

**Active-Active:**
```
[Concentrator 1] <--> [Concentrator 2]
       |                    |
       +--[Load Balancer]---+
                |
          [VPN Clients]

Benefits:
- Load distribution
- Higher capacity
- Fault tolerance
- Geographic distribution
```

### 11.4 Session Management

**Session Lifecycle:**
```
1. Connection Request
   - Authentication
   - Authorization
   - Resource allocation

2. Session Establishment
   - Tunnel creation
   - Key exchange
   - IP assignment

3. Active Session
   - Data transfer
   - Keep-alive
   - Monitoring

4. Session Termination
   - Graceful disconnect
   - Timeout
   - Resource cleanup
```

**Session Limits:**
```
Per-User Limits:
- Maximum concurrent sessions: 5
- Session timeout: 8 hours
- Idle timeout: 30 minutes
- Reauthentication: 24 hours

Global Limits:
- Maximum total sessions
- Bandwidth per session
- Connection rate limiting
```

---




---

## A.1 Endpoint reference

```http
POST /vpn-protocol/v1/tunnels                    # provision tunnel
GET  /vpn-protocol/v1/tunnels/{id}               # fetch tunnel record
GET  /vpn-protocol/v1/tunnels/{id}/state         # current SA state
POST /vpn-protocol/v1/keys/rotate                # request key rotation
GET  /vpn-protocol/v1/peers                      # list authorised peers
POST /vpn-protocol/v1/peers                      # register a peer
WS   /vpn-protocol/v1/state/stream               # SA state stream
GET  /vpn-protocol/v1/audit/{tunnelId}           # audit trail
```

Every endpoint follows the discovery convention at `/.well-known/wia-vpn-protocol`. Tunnel-provisioning endpoints require a tenant-administrator credential plus the operator's per-domain authorization scope.

## A.2 Tunnel-provisioning API

`POST /tunnels` accepts the Phase 1 §A.1 envelope. The endpoint validates the cipher-suite envelope against the operator's policy (e.g., post-quantum required for tunnels carrying sensitive traffic; hybrid-PQ required for cross-jurisdiction tunnels), allocates the per-tunnel monitoring slot, and returns the configuration bundle (or a configuration URL for clients that pull rather than push). Tunnel state is one of `provisioning`, `ready`, `up`, `degraded`, `down`, `terminated`; state transitions emit audit events.

## A.3 Key-rotation API

`POST /keys/rotate` triggers a rekey on a live tunnel. The endpoint enforces the operator's rotation policy (default 8 hours for IKEv2 Phase 1 SA, 1 hour for Phase 2 SA, 2 minutes for WireGuard handshake re-derivation per the Noise specification). Forced rotations on incident detection or on certificate revocation events are scheduled within the documented response-time SLA. Rotation failures bound the tunnel's life: a tunnel that cannot rotate within the policy window is torn down and re-established.

## A.4 Peer-management API

`POST /peers` registers a peer with the peer-record envelope: peer identifier, peer authentication credential reference, allowed-tunnel scope, geographic-jurisdiction tag for compliance with cross-border encryption rules, and the per-peer rate-limit envelope. Peer revocation follows the symmetric protocol: revoked peers are added to the operator's peer-deny list with the revocation reason, and tunnels referencing the peer are torn down within the revocation-propagation SLA.

## A.5 Telemetry WebSocket

The state-stream WebSocket multiplexes per-tunnel events: SA establishment, rekey, dead-peer detection (DPD), MOBIKE address change, fragmentation activation, MTU discovery outcome, crypto-suite negotiation outcome, and tunnel teardown. Subscribers can filter by tunnel-class and by event-class. The broker emits push events on threshold-crossing conditions (rekey skew over policy threshold; DPD failures exceeding policy; cipher-suite downgrade alerts).

## A.6 Audit and rate-limit envelope

`GET /audit/{tunnelId}` returns the immutable audit trail: tunnel-creation event, every rekey event, peer-state transitions, certificate-validation outcomes, cipher-suite renegotiations, and tunnel-teardown event. The audit-trail integrity is anchored into a Merkle tree per-tenant. Rate limits: 1000 req/h authenticated, 5000 req/h trusted-partner. WebSocket subscriptions are bounded at 100 simultaneous per credential.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/vpn-protocol/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-vpn-protocol-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/vpn-protocol-host:1.0.0` ships every vpn-protocol envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/vpn-protocol.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Vpn-protocol deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
