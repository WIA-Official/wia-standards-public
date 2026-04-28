# WIA-COMM-016 — Phase 1: Data Format

> VPN-protocol canonical Phase 1: tunnel + cipher-suite + IKEv2 SA + WireGuard + identity envelopes.

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


## 2. IPsec Protocol Suite

### 2.1 IPsec Overview

IPsec provides network-layer security through:
- **Authentication**: Verify packet source
- **Integrity**: Detect packet tampering
- **Confidentiality**: Encrypt packet payload
- **Replay protection**: Prevent replay attacks

**Architecture:**
```
IPsec Stack:
┌─────────────────────────────────┐
│  Application Layer              │
├─────────────────────────────────┤
│  Transport Layer (TCP/UDP)      │
├─────────────────────────────────┤
│  IPsec Layer                    │
│  - IKE (Key Exchange)           │
│  - ESP (Encryption)             │
│  - AH (Authentication)          │
├─────────────────────────────────┤
│  IP Layer                       │
├─────────────────────────────────┤
│  Link Layer                     │
└─────────────────────────────────┘
```

### 2.2 IKEv2 (Internet Key Exchange v2)

IKEv2 establishes and maintains IPsec Security Associations (SAs).

**IKEv2 Exchange:**
```
Initiator                        Responder
   |                                |
   |------ IKE_SA_INIT (HDR, SA,  --|
   |       KEi, Ni)                 |
   |                                |
   |<----- IKE_SA_INIT (HDR, SA,  --|
   |       KEr, Nr)                 |
   |                                |
   |------ IKE_AUTH (HDR, SK {...})--|
   |                                |
   |<----- IKE_AUTH (HDR, SK {...})--|
   |                                |
   [IKE_SA and CHILD_SA established]

Messages:
1. IKE_SA_INIT: Exchange DH keys, negotiate algorithms
2. IKE_AUTH: Authenticate parties, create CHILD_SA
```

**IKEv2 Parameters:**
```
Encryption Algorithms:
- AES-256-GCM (Recommended)
- AES-256-CBC
- AES-128-GCM
- ChaCha20-Poly1305

Integrity Algorithms:
- SHA-256 (Recommended)
- SHA-384
- SHA-512

DH Groups:
- Group 14: 2048-bit MODP (Minimum)
- Group 15: 3072-bit MODP
- Group 16: 4096-bit MODP
- Group 19: 256-bit ECC
- Group 20: 384-bit ECC

PRF (Pseudorandom Function):
- PRF-HMAC-SHA-256
- PRF-HMAC-SHA-384
```

### 2.3 ESP (Encapsulating Security Payload)

ESP provides encryption and optional authentication.

**ESP Packet Format:**
```
Transport Mode:
┌────────────────┬─────┬──────────────┬─────┬─────┐
│ IP Header      │ ESP │ TCP/UDP Data │ ESP │ ESP │
│ (Original)     │ Hdr │ (Encrypted)  │Trail│ Auth│
└────────────────┴─────┴──────────────┴─────┴─────┘

Tunnel Mode:
┌────────────┬─────┬────────────┬─────────────┬─────┬─────┐
│ New IP Hdr │ ESP │ Original   │ TCP/UDP     │ ESP │ ESP │
│            │ Hdr │ IP Header  │ Data        │Trail│ Auth│
│            │     │ (Encrypted)│ (Encrypted) │     │     │
└────────────┴─────┴────────────┴─────────────┴─────┴─────┘

ESP Header:
- SPI (Security Parameters Index): 32 bits
- Sequence Number: 32 bits
- Payload Data: Variable
- Padding: 0-255 bytes
- Pad Length: 8 bits
- Next Header: 8 bits
- ICV (Integrity Check Value): Variable
```

**ESP Modes:**
1. **Transport Mode**:
   - Encrypts only payload
   - Preserves IP header
   - Used for end-to-end communication

2. **Tunnel Mode**:
   - Encrypts entire IP packet
   - Adds new IP header
   - Used for gateway-to-gateway VPN

### 2.4 AH (Authentication Header)

AH provides authentication and integrity (no encryption).

**AH Packet Format:**
```
┌────────────┬──────┬─────────────────┐
│ IP Header  │  AH  │ TCP/UDP + Data  │
└────────────┴──────┴─────────────────┘

AH Header:
- Next Header: 8 bits
- Payload Length: 8 bits
- Reserved: 16 bits
- SPI: 32 bits
- Sequence Number: 32 bits
- ICV: Variable (12-16 bytes typical)
```

**Use Cases:**
- Authentication without encryption
- Regulatory compliance (some countries)
- Header integrity verification

**Limitations:**
- No confidentiality
- NAT incompatible (modifies IP header)
- Largely superseded by ESP with authentication

### 2.5 IPsec Configuration Example

```
Configuration:
Phase 1 (IKE_SA):
- Protocol: IKEv2
- Encryption: AES-256-GCM
- Integrity: SHA-256
- DH Group: Group 14 (modp2048)
- Lifetime: 28800s (8 hours)
- Authentication: Pre-Shared Key or Certificates

Phase 2 (CHILD_SA/IPsec):
- Protocol: ESP
- Mode: Tunnel
- Encryption: AES-256-GCM
- PFS: Group 14
- Lifetime: 3600s (1 hour)
- DPD: 30s interval

Policy:
- Local Subnet: 192.168.1.0/24
- Remote Subnet: 10.0.0.0/24
- Action: Encrypt
```

---



## 3. OpenVPN

### 3.1 OpenVPN Overview

OpenVPN is an SSL/TLS-based VPN providing:
- Cross-platform compatibility
- Flexible configuration
- NAT-friendly operation
- Strong encryption

**Architecture:**
```
OpenVPN Stack:
┌──────────────────────────┐
│  Application Layer       │
├──────────────────────────┤
│  TLS Layer (Control)     │
│  - Authentication        │
│  - Key Exchange          │
├──────────────────────────┤
│  Data Channel            │
│  - Encryption (AES-GCM)  │
│  - Compression (LZ4)     │
├──────────────────────────┤
│  Transport (UDP/TCP)     │
├──────────────────────────┤
│  IP Layer                │
└──────────────────────────┘
```

### 3.2 OpenVPN Protocol

**Control Channel (TLS):**
```
Client                          Server
  |                               |
  |--- Client Hello ------------->|
  |<-- Server Hello, Cert --------|
  |--- Client Key Exchange ------>|
  |--- Change Cipher Spec ------->|
  |<-- Change Cipher Spec --------|
  |                               |
  [TLS Session Established]
  |                               |
  |--- Push Request ------------->|
  |<-- Push Reply (IP, Routes) ---|
  |                               |
  [VPN Tunnel Active]
```

**Data Channel:**
```
Packet Format:
┌─────┬──────┬─────────┬──────────┬─────┐
│ UDP │ OVPN │ Packet  │ Payload  │ HMAC│
│ Hdr │ Hdr  │ ID      │(Encrypted)│     │
└─────┴──────┴─────────┴──────────┴─────┘

OVPN Header:
- Opcode: 8 bits (P_DATA_V1, P_DATA_V2)
- Key ID: 3 bits
- Peer ID: 24 bits (optional)
```

### 3.3 OpenVPN Configuration

**Server Configuration:**
```
# Protocol
proto udp
port 1194

# Network
dev tun
topology subnet
server 10.8.0.0 255.255.255.0

# Certificates
ca ca.crt
cert server.crt
key server.key
dh dh2048.pem
tls-auth ta.key 0

# Encryption
cipher AES-256-GCM
auth SHA256
tls-version-min 1.3

# Security
user nobody
group nogroup
persist-key
persist-tun

# Routing
push "route 192.168.1.0 255.255.255.0"
push "redirect-gateway def1 bypass-dhcp"
push "dhcp-option DNS 8.8.8.8"

# Performance
fast-io
sndbuf 393216
rcvbuf 393216
push "sndbuf 393216"
push "rcvbuf 393216"

# Logging
verb 3
```

**Client Configuration:**
```
client
dev tun
proto udp

remote vpn.example.com 1194
resolv-retry infinite
nobind

# Certificates
ca ca.crt
cert client.crt
key client.key
tls-auth ta.key 1

# Encryption
cipher AES-256-GCM
auth SHA256

# Security
persist-key
persist-tun

# Logging
verb 3
```

### 3.4 OpenVPN Security

**TLS Options:**
- **TLS 1.3**: Latest TLS version
- **TLS Auth**: HMAC signature for DoS protection
- **TLS Crypt**: Encrypt control channel
- **Certificate Verification**: Validate server/client certs

**Cipher Suites:**
```
Recommended Ciphers:
- AES-256-GCM (AEAD)
- AES-128-GCM (AEAD)
- ChaCha20-Poly1305 (AEAD)

Legacy (Avoid):
- AES-256-CBC + SHA256
- AES-128-CBC + SHA1
```

---



## 4. WireGuard

### 4.1 WireGuard Overview

WireGuard is a modern, fast, and simple VPN protocol featuring:
- Minimal codebase (~4,000 lines)
- State-of-the-art cryptography
- Superior performance
- Simple configuration
- Built into Linux kernel (5.6+)

**Design Principles:**
- Simplicity over complexity
- Modern cryptography only
- Minimal attack surface
- Stealth (silent to port scans)

### 4.2 WireGuard Cryptography

**Cryptographic Primitives:**
```
Encryption: ChaCha20-Poly1305 (AEAD)
Key Exchange: Curve25519
Hashing: BLAKE2s
Keyed Hashing: BLAKE2s-MAC

Key Derivation: HKDF (HMAC-based KDF)
```

**Key Structure:**
```
Each peer has:
- Private Key: 32-byte Curve25519 secret
- Public Key: 32-byte Curve25519 public
- Pre-shared Key (optional): 32-byte symmetric key

No certificate infrastructure required!
```

### 4.3 WireGuard Protocol

**Handshake (Noise_IK):**
```
Initiator                        Responder
   |                                |
   |-- Handshake Init ----------->  |
   |   (pubkey_i, encrypted_data)   |
   |                                |
   |<- Handshake Response --------- |
   |   (pubkey_r, encrypted_data)   |
   |                                |
   [Secure tunnel established]
   |                                |
   |<-------- Data Packets -------->|
   |                                |
```

**Handshake Details:**
1. **Initiator Message**:
   - Ephemeral public key
   - Encrypted static public key
   - Encrypted timestamp
   - MAC authentication

2. **Responder Message**:
   - Ephemeral public key
   - Encrypted nothing (confirmation)
   - MAC authentication

3. **Data Transmission**:
   - Each packet encrypted with rotating keys
   - Counter-based nonce
   - 120-second rekey interval

### 4.4 WireGuard Configuration

**Interface Configuration:**
```ini
[Interface]
PrivateKey = <base64-private-key>
Address = 10.0.0.1/24
ListenPort = 51820
DNS = 1.1.1.1

# Optional
PostUp = iptables -A FORWARD -i wg0 -j ACCEPT
PostDown = iptables -D FORWARD -i wg0 -j ACCEPT

[Peer]
PublicKey = <base64-public-key>
AllowedIPs = 10.0.0.2/32
Endpoint = 203.0.113.1:51820
PersistentKeepalive = 25
```

**Key Generation:**
```bash
# Generate private key
wg genkey > private.key

# Derive public key
cat private.key | wg pubkey > public.key

# Generate pre-shared key (optional)
wg genpsk > preshared.key
```

### 4.5 WireGuard Routing

**AllowedIPs Routing Table:**
```
AllowedIPs acts as:
1. Cryptokey routing (incoming packets)
2. Routing table (outgoing packets)

Example Configurations:

Full Tunnel (all traffic):
AllowedIPs = 0.0.0.0/0, ::/0

Split Tunnel (specific networks):
AllowedIPs = 10.0.0.0/8, 192.168.0.0/16

Single Host:
AllowedIPs = 10.0.0.5/32
```

### 4.6 WireGuard Security Features

**Cryptographic Guarantees:**
- **Perfect Forward Secrecy**: Rotating session keys
- **Identity Hiding**: Silent on port scans
- **Replay Protection**: Counter-based nonces
- **DoS Protection**: Stateless cookie exchange

**Session Management:**
```
Timing:
- Handshake every 2 minutes (120s)
- Rekey on every handshake
- Packet counter never reused
- Old keys immediately discarded

State:
- Minimal state per peer
- Fast handshake (<1 RTT)
- Roaming support (IP mobility)
```

---



## 5. L2TP/IPsec

### 5.1 L2TP Overview

L2TP (Layer 2 Tunneling Protocol) + IPsec combines:
- L2TP: Tunneling protocol (no encryption)
- IPsec: Encryption and authentication

**Architecture:**
```
┌─────────────────────────┐
│  PPP Session            │
├─────────────────────────┤
│  L2TP Tunnel            │
│  (Port 1701)            │
├─────────────────────────┤
│  IPsec ESP              │
│  (Encryption)           │
├─────────────────────────┤
│  UDP/IP                 │
└─────────────────────────┘
```

### 5.2 L2TP Protocol

**L2TP Packet Format:**
```
┌─────┬─────┬───────┬──────────┬─────┐
│ UDP │ L2TP│  PPP  │ IP Data  │ ESP │
│ Hdr │ Hdr │  Hdr  │ (Payload)│ Auth│
└─────┴─────┴───────┴──────────┴─────┘

L2TP Header:
- Flags: 8 bits
- Version: 4 bits
- Length: 16 bits (optional)
- Tunnel ID: 16 bits
- Session ID: 16 bits
- Ns: 16 bits (optional)
- Nr: 16 bits (optional)
```

**L2TP Connection Establishment:**
```
LAC (Client)                 LNS (Server)
   |                            |
   |--- SCCRQ (Start-Ctrl-Conn-Req) -->|
   |<-- SCCRP (Reply) --------------|
   |--- SCCCN (Connected) ---------->|
   |                            |
   |--- ICRQ (Incoming-Call-Req) -->|
   |<-- ICRP (Reply) --------------|
   |--- ICCN (Connected) ---------->|
   |                            |
   [L2TP Tunnel Established]
   |                            |
   |<------- PPP Session ------->|
   |                            |
```

### 5.3 L2TP/IPsec Configuration

**IPsec Phase:**
```
IKEv1/IKEv2 Configuration:
- Encryption: AES-256
- Authentication: SHA-256
- DH Group: Group 14
- NAT-T: Enabled (UDP 4500)
```

**L2TP Phase:**
```
L2TP Configuration:
- Port: UDP 1701
- Authentication: PAP, CHAP, or MS-CHAPv2
- Compression: Optional (MPPC)
```

**Windows Built-in Support:**
```
L2TP/IPsec is natively supported in:
- Windows (All versions)
- macOS
- iOS/iPadOS
- Android

Configuration:
- VPN Type: L2TP/IPsec
- Pre-shared key or Certificate
- Username/Password authentication
```

---



## 6. SSL/TLS VPN

### 6.1 SSL VPN Overview

SSL/TLS VPN provides secure remote access using web browsers and SSL/TLS protocol.

**Types:**
1. **SSL Portal VPN**: Web-based access (HTTPS)
2. **SSL Tunnel VPN**: Full network layer tunnel
3. **Application Proxy**: Per-application access

### 6.2 SSL Portal VPN

**Architecture:**
```
User Browser <--HTTPS--> SSL VPN Gateway <--> Internal Resources
                           (Reverse Proxy)

Features:
- No client software required
- Web browser access
- Application translation (RDP, SSH, etc.)
- Limited to web-based applications
```

### 6.3 SSL Tunnel VPN

**Architecture:**
```
VPN Client <--TLS Tunnel--> VPN Gateway <--> Internal Network
            (Full IP Tunnel)

Features:
- Full network access
- Requires client software
- Layer 3 connectivity
- All protocols supported
```

**DTLS (Datagram TLS):**
```
DTLS for VPN:
- UDP-based TLS
- Lower latency than TCP-based TLS
- Handles packet loss better
- Ideal for real-time applications

Port: UDP 443 (typical)
```

### 6.4 SSTP (Secure Socket Tunneling Protocol)

Microsoft's SSL VPN protocol:
```
SSTP Characteristics:
- Protocol: SSL/TLS over TCP
- Port: TCP 443 (HTTPS)
- Firewall friendly
- Windows native support

SSTP Packet:
┌─────┬─────┬─────┬──────────┐
│ TCP │ TLS │SSTP │   PPP    │
│ Hdr │     │ Hdr │  Payload │
└─────┴─────┴─────┴──────────┘
```

---




---

## A.1 Tunnel-record envelope

The Phase 1 envelope groups VPN tunnels by protocol family (IPsec — IKEv2 negotiated, ESP/AH transport; OpenVPN — TLS-based; WireGuard — Noise IKpsk2-based; SSL/TLS VPN — application-level; L2TP/IPsec — combined L2 framing with IPsec encryption) with the canonical fields: tunnel identifier, peer endpoints (initiator + responder addresses with NAT-T detection), authentication method (PSK, X.509 certificate per RFC 5280 with PKIX validation, EAP per RFC 5247, certificateless WireGuard public key), cipher suite per protocol catalogue (IKEv2: ENCR_AES_GCM_16 + PRF_HMAC_SHA2_512 + DH 19/20/21/31; OpenVPN: AES-256-GCM + HMAC-SHA512; WireGuard: ChaCha20-Poly1305 + BLAKE2s + Curve25519), Phase 1 + Phase 2 SA lifetimes, NAT-traversal envelope, and the audit envelope tied to the operator's tenant.

## A.2 Cipher-suite descriptor

A cipher-suite descriptor MUST list: encryption algorithm with key length (AES-128-GCM / AES-256-GCM / ChaCha20-Poly1305), integrity protection (AES-GCM combined / HMAC-SHA-256 / HMAC-SHA-384 / HMAC-SHA-512 / Poly1305), key-derivation (HKDF per RFC 5869 / IKEv2 PRF / Noise key schedule), key-exchange (ECDHE on P-256 / P-384 / P-521 per FIPS 186-4; X25519 / X448 per RFC 7748; ML-KEM per NIST FIPS 203 for post-quantum), and the authentication-mechanism envelope (RSA-PSS per RFC 8017; ECDSA on the same curves; EdDSA on Ed25519 per RFC 8032; ML-DSA per NIST FIPS 204). Hybrid post-quantum suites combine classical KEX with ML-KEM per the IETF draft hybrid-design framework.

## A.3 IKEv2 SA descriptor

IKEv2 SA descriptors carry the IKE_SA_INIT exchange parameters (initiator/responder SPI, cipher proposal, DH group, nonces), IKE_AUTH parameters (identity payload, certificate payload, certificate-request payload, AUTH payload computation per RFC 7296 §2.15), CHILD_SA negotiation parameters, and the rekey envelope per RFC 7296 §1.3.2. IKEv2 fragmentation per RFC 7383 is captured for environments where UDP fragmentation is unreliable. MOBIKE per RFC 4555 is captured where the tunnel must survive the client's address change.

## A.4 WireGuard tunnel descriptor

WireGuard tunnel descriptors carry: per-peer Curve25519 static keypair, optional pre-shared key (per RFC 7693 BLAKE2s mixing for post-quantum hardening), per-peer allowed-IPs (cryptokey routing), persistent-keepalive interval (typically 25 seconds when traversing NAT), MTU envelope (typical 1420 bytes for IPv4, 1400 for IPv6), and the handshake-rotation envelope (rekey after 120 seconds or 2^60 messages whichever is sooner per the Noise IKpsk2 specification). The descriptor cross-references the cipher-suite descriptor at Phase 1 §A.2 with WireGuard's fixed suite (ChaCha20-Poly1305-AEAD + Curve25519 + BLAKE2s + HKDF).

## A.5 Identity and certificate descriptor

Identity and certificate descriptors follow RFC 5280 (PKIX) and RFC 8410 for Curve25519/Ed25519: subject DN, SAN entries (DNS / IP / URI / OtherName), key-usage envelope (digitalSignature, keyEncipherment for RSA; non-repudiation for signing-only certificates), extended-key-usage envelope (id-kp-clientAuth, id-kp-serverAuth, id-kp-ipsecIKE per RFC 4945), certificate-policy OIDs, CRL distribution points + OCSP responder URLs per RFC 6960, and the certificate-transparency envelope per RFC 9162 where the operator publishes to public CT logs.


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
