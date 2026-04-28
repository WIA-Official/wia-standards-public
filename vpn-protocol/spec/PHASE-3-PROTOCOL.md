# WIA-COMM-016 — Phase 3: Protocol

> VPN-protocol canonical Phase 3: protocols (IKEv2 + OpenVPN + WireGuard + SSL-TLS + key-exchange).

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


## 7. VPN Architecture

### 7.1 Remote Access VPN

**Architecture:**
```
Remote Users --> Internet --> VPN Gateway --> Corporate Network

Components:
- VPN Client: User device software
- VPN Gateway: Corporate entry point
- Authentication Server: RADIUS, LDAP, AD
- Access Control: Firewall, NAC
```

**Authentication Methods:**
1. **Username/Password**: Basic authentication
2. **Certificates**: X.509 digital certificates
3. **Multi-Factor**: Password + OTP/Token
4. **Single Sign-On**: SAML, OAuth

**Access Control:**
```
Authorization Levels:
- Full Access: All internal resources
- Limited Access: Specific subnets/services
- Role-Based: Access based on user role
- Posture-Based: Device compliance check
```

### 7.2 Site-to-Site VPN

**Architecture:**
```
Office A <--> VPN Gateway A <--Internet--> VPN Gateway B <--> Office B

Characteristics:
- Always-on tunnel
- Gateway-to-gateway
- Transparent to users
- Static routing or dynamic (BGP)
```

**Redundancy:**
```
High Availability:
- Dual VPN gateways (active-passive)
- Multiple ISP links
- Dynamic failover
- Load balancing

Topology:
┌─────────┐     ┌─────────┐
│ Gateway │<--->│ Gateway │
│   A1    │     │   B1    │
└─────────┘     └─────────┘
     ∧               ∧
     |               |
     v               v
┌─────────┐     ┌─────────┐
│ Gateway │<--->│ Gateway │
│   A2    │     │   B2    │
└─────────┘     └─────────┘
```

### 7.3 Hub-and-Spoke VPN

**Architecture:**
```
        [Branch B]
             |
             v
[Branch A]-->[HQ Hub]-->[Branch C]
             ^
             |
        [Branch D]

Benefits:
- Centralized management
- Reduced complexity
- Easier troubleshooting

Drawbacks:
- HQ is single point of failure
- All traffic through hub (latency)
```

### 7.4 Full Mesh VPN

**Architecture:**
```
[Office A]<--->[Office B]
    ^  \         /  ^
    |   \       /   |
    |    \     /    |
    v     \   /     v
[Office D]<-->[Office C]

Benefits:
- Direct site-to-site communication
- Optimal routing
- No single point of failure

Drawbacks:
- Complex configuration (n*(n-1)/2 tunnels)
- Scalability challenges
```

---



## 8. Key Exchange Mechanisms

### 8.1 Diffie-Hellman Key Exchange

**DH Algorithm:**
```
Public Parameters: p (prime), g (generator)

Alice:
1. Choose secret a
2. Compute A = g^a mod p
3. Send A to Bob

Bob:
1. Choose secret b
2. Compute B = g^b mod p
3. Send B to Alice

Shared Secret:
Alice: K = B^a mod p
Bob:   K = A^b mod p

Result: K = g^(ab) mod p (same for both)
```

**DH Groups:**
```
Group 1:  768-bit MODP   (Deprecated)
Group 2:  1024-bit MODP  (Deprecated)
Group 5:  1536-bit MODP  (Weak)
Group 14: 2048-bit MODP  (Minimum recommended)
Group 15: 3072-bit MODP  (Recommended)
Group 16: 4096-bit MODP  (High security)
Group 19: 256-bit ECC    (Efficient, recommended)
Group 20: 384-bit ECC    (High security)
```

### 8.2 Elliptic Curve Diffie-Hellman (ECDH)

**Advantages:**
- Smaller key sizes
- Faster computation
- Same security as larger RSA/DH keys

**Curves:**
```
Curve25519:
- 256-bit security
- High performance
- Side-channel resistant
- Used in WireGuard

P-256 (secp256r1):
- NIST standard
- Widely supported
- 128-bit security

P-384 (secp384r1):
- NIST standard
- 192-bit security
- Government use
```

### 8.3 Pre-Shared Keys (PSK)

**PSK Characteristics:**
- Simple configuration
- No PKI required
- Shared secret between peers
- Must be securely distributed

**Security Considerations:**
```
PSK Best Practices:
- Minimum 32 characters (256 bits)
- Random generation (not passphrases)
- Secure distribution (out-of-band)
- Regular rotation
- Unique per tunnel

Weak PSK = Weak VPN Security!
```

---



## 9. Perfect Forward Secrecy

### 9.1 PFS Overview

Perfect Forward Secrecy ensures that:
- Session keys are not compromised if long-term keys are
- Each session has unique encryption keys
- Past sessions cannot be decrypted

**Without PFS:**
```
Master Key Compromise:
- All past sessions decryptable
- All future sessions compromised
- Catastrophic security failure
```

**With PFS:**
```
Master Key Compromise:
- Session keys remain secure
- Only current session at risk
- Limited damage
```

### 9.2 PFS Implementation

**IPsec PFS:**
```
Configuration:
- Enable PFS in Phase 2
- Specify DH group for PFS
- Separate DH exchange per SA

Example:
Phase 2 (IPsec):
- PFS: Enabled
- DH Group: Group 14
- Rekey: Every 3600s

Result: New DH exchange every rekey
```

**OpenVPN PFS:**
```
TLS 1.3 provides PFS by default:
- Ephemeral key exchange
- No static RSA key exchange
- Forward secrecy guaranteed

TLS 1.2 configuration:
- Disable RSA key exchange
- Use ECDHE/DHE cipher suites
```

**WireGuard PFS:**
```
Built-in PFS:
- New session keys every 120s
- Ephemeral keys discarded immediately
- No long-term compromise risk
```

---



## 10. Split Tunneling

### 10.1 Split Tunneling Overview

Split tunneling allows selective routing:
- VPN traffic: Through secure tunnel
- Internet traffic: Direct (bypass VPN)

**Full Tunnel:**
```
User Device --> VPN Gateway --> Internet
                           --> Corporate Network

All traffic through VPN
```

**Split Tunnel:**
```
User Device --> VPN Gateway --> Corporate Network
           \
            \--> Internet (Direct)

Only corporate traffic through VPN
```

### 10.2 Split Tunneling Configuration

**IPsec Policy:**
```
Encryption Domain:
- Local: 0.0.0.0/0 (Full) or 192.168.1.0/24 (Split)
- Remote: 10.0.0.0/8 (Corporate)

Routing:
- VPN routes: 10.0.0.0/8
- Default route: Direct (not through VPN)
```

**OpenVPN:**
```
# Full Tunnel
push "redirect-gateway def1"

# Split Tunnel
push "route 10.0.0.0 255.0.0.0"
push "route 192.168.1.0 255.255.255.0"
# Do NOT push redirect-gateway
```

**WireGuard:**
```
# Full Tunnel
AllowedIPs = 0.0.0.0/0, ::/0

# Split Tunnel
AllowedIPs = 10.0.0.0/8, 192.168.1.0/24
```

### 10.3 Split Tunneling Security

**Benefits:**
- Reduced VPN gateway load
- Better internet performance
- Lower latency for non-corporate traffic

**Risks:**
- Bypass corporate security controls
- Potential data leakage
- Malware from unprotected internet

**Mitigation:**
```
Security Controls:
- Endpoint protection (antivirus, EDR)
- DNS filtering (block malicious domains)
- Application control (whitelist/blacklist)
- Zero Trust Network Access (ZTNA)
```

---



## 13. Security Considerations

### 13.1 Encryption Standards

**Symmetric Encryption:**
```
Recommended (2025):
- AES-256-GCM (AEAD)
- AES-128-GCM (AEAD)
- ChaCha20-Poly1305 (AEAD)

Acceptable:
- AES-256-CBC + HMAC-SHA256
- AES-128-CBC + HMAC-SHA256

Deprecated:
- 3DES
- Blowfish
- RC4
```

**Asymmetric Encryption:**
```
RSA:
- Minimum: 2048-bit
- Recommended: 3072-bit or 4096-bit

ECC:
- Curve25519 (256-bit security)
- P-256, P-384, P-521 (NIST curves)

Avoid:
- RSA < 2048-bit
- DSA
- P-192 curve
```

### 13.2 Authentication

**Strong Authentication:**
```
Multi-Factor Authentication:
1. Something you know: Password/PIN
2. Something you have: Token/Certificate
3. Something you are: Biometric

Implementation:
- RADIUS with MFA
- Certificate + Password
- SAML/OAuth integration
- TOTP/HOTP tokens
```

**Certificate Management:**
```
Best Practices:
- Use internal CA for VPN certificates
- 2048-bit RSA minimum
- SHA-256 or SHA-384 signature
- Certificate expiry: 1-2 years
- Certificate revocation list (CRL)
- OCSP for real-time validation
```

### 13.3 Attack Mitigation

**Common VPN Attacks:**
```
1. Man-in-the-Middle (MITM)
   Mitigation: Certificate validation, PFS

2. Replay Attacks
   Mitigation: Sequence numbers, timestamps

3. DoS/DDoS
   Mitigation: Rate limiting, IKE cookies

4. Brute Force
   Mitigation: Account lockout, MFA

5. Traffic Analysis
   Mitigation: Padding, constant rate
```

**IKE Cookie Protection:**
```
IKEv2 DoS Protection:
1. Client sends initial message
2. Server responds with cookie
3. Client resends with cookie
4. Server processes (stateful)

Result: Stateless until cookie validated
```

---




---

## A.1 IKEv2 protocol

The IKEv2 protocol per RFC 7296 covers the initial exchange (IKE_SA_INIT establishes the IKE SA with cipher proposal + DH key share + nonces; IKE_AUTH authenticates the peers and creates the first CHILD_SA), optional CREATE_CHILD_SA exchanges for additional CHILD_SAs and rekey, INFORMATIONAL exchanges for keep-alive / DPD / config-payload negotiation, and the EAP authentication path per RFC 5247 for cases where an EAP server backs the responder. The protocol covers the canonical attack-mitigations: cookie-based DoS protection (RFC 7296 §2.6), redirect (RFC 5685), and downgrade-resistance via the AUTH payload covering the SA proposal.

## A.2 OpenVPN protocol

OpenVPN protocol covers the TLS-based control channel (TLS 1.2 or TLS 1.3 per RFC 8446) plus the data channel encrypted under symmetric keys derived from the TLS master key. The protocol supports both UDP and TCP transports; UDP is preferred for low-latency, TCP is required for environments where UDP is filtered. Configuration covers TLS-Auth and TLS-Crypt (HMAC firewall to defeat un-authenticated DoS), cert-based authentication, the cipher negotiation envelope (data-ciphers list with the agreed-upon AEAD cipher), and the compression-disabled stance per the VORACLE attack guidance.

## A.3 WireGuard protocol

WireGuard protocol per the Noise IKpsk2 framework covers the four handshake messages (Initiator → Responder → Initiator → Responder), the data-keepalive interval, the cookie-based DoS protection that rate-limits handshake attempts under load, and the cryptokey-routing envelope where each peer is associated with a fixed allowed-IPs list. The protocol bans cipher negotiation: WireGuard's fixed cipher suite eliminates downgrade attacks. Post-quantum hardening per the WireGuard MAINTAINERS' guidance uses an optional pre-shared key combined with the Noise key schedule.

## A.4 SSL/TLS-VPN protocol

SSL/TLS VPN protocol covers the application-layer TLS tunnel (RFC 8446 TLS 1.3 mandated for new deployments), the per-application proxy envelope, the SAML / OIDC integration for user authentication, the certificate-pinning envelope for client trust, and the client-certificate authentication envelope for mutual TLS where adopted. The protocol covers HTTP/3 over QUIC (RFC 9000 + RFC 9114) for environments where the underlying QUIC connection migration semantics are useful for roaming clients.

## A.5 Key-exchange and forward-secrecy protocol

Key exchange follows the protocol-specific catalogue: IKEv2 supports DH groups 2/5/14/15/16/18/19/20/21 with operator-policy gating to disallow groups 2 and 5 in production; OpenVPN derives keys via TLS handshake; WireGuard uses Curve25519 X25519. Perfect forward secrecy is enforced by ephemeral key exchange in every protocol's catalogue; long-term private keys never derive session keys directly. Post-quantum migration per IETF drafts on hybrid X25519+ML-KEM-768 + hybrid Ed25519+ML-DSA-65 captures the migration path.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the management-plane control traffic. Per-tunnel data-plane integrity uses the AEAD-mode authentication of every encrypted packet; replay-window enforcement per RFC 4303 §3.4 (ESP) / OpenVPN-replay-window (default 64) / WireGuard-replay-window (default 65536) defeats data-plane replay. MOBIKE address changes preserve the SA replay-counter so the tunnel can roam across networks without reset.


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
