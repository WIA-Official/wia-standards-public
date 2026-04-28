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

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive VPN (Virtual Private Network) protocols and technologies for secure, private communication over public networks. It covers modern protocols (WireGuard, IKEv2), established standards (OpenVPN, IPsec), and enterprise deployment patterns.

### 1.2 Scope

The standard covers:
- IPsec protocol suite (IKEv2, ESP, AH)
- SSL/TLS-based VPN protocols (OpenVPN, SSTP)
- Modern VPN protocols (WireGuard)
- Legacy protocols (L2TP/IPsec, PPTP)
- VPN architectures (site-to-site, remote access)
- Key exchange and encryption mechanisms
- Performance optimization techniques

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - VPN technology protects privacy, enables secure remote work, and bridges digital divides by providing safe, encrypted communications accessible to individuals and organizations worldwide.

### 1.4 Terminology

- **VPN**: Virtual Private Network
- **IPsec**: Internet Protocol Security
- **IKE**: Internet Key Exchange
- **ESP**: Encapsulating Security Payload
- **AH**: Authentication Header
- **SA**: Security Association
- **SPD**: Security Policy Database
- **PFS**: Perfect Forward Secrecy
- **NAT-T**: NAT Traversal
- **MTU**: Maximum Transmission Unit

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

## 12. Performance Optimization

### 12.1 MTU Optimization

**MTU Basics:**
```
Standard Ethernet MTU: 1500 bytes

VPN Overhead:
- IPsec ESP: ~50-60 bytes
- OpenVPN: ~40-100 bytes
- WireGuard: ~60 bytes

Optimal VPN MTU:
- IPsec: 1400-1440 bytes
- OpenVPN: 1400-1450 bytes
- WireGuard: 1420-1440 bytes
```

**MTU Discovery:**
```bash
# Test MTU size (Windows)
ping -f -l 1400 destination

# Test MTU size (Linux)
ping -M do -s 1400 destination

# Optimal MTU = Largest size that doesn't fragment
```

**Configuration:**
```
OpenVPN:
tun-mtu 1400
mssfix 1360

IPsec:
ip link set dev ipsec0 mtu 1400

WireGuard:
[Interface]
MTU = 1420
```

### 12.2 Compression

**Compression Algorithms:**
```
LZO (Legacy):
- Fast compression
- Moderate ratio
- CPU overhead

LZ4:
- Very fast compression
- Lower CPU usage
- Recommended for VPN

Considerations:
- Compression helps: Text, code, logs
- Compression hurts: Encrypted data, video, images
- Security risk: VORACLE, CRIME attacks on TLS
```

**Recommendation:**
```
Modern Best Practice:
- Disable compression (security)
- Use hardware acceleration instead
- Let application layer handle compression if needed
```

### 12.3 Hardware Acceleration

**AES-NI (Intel/AMD):**
```
CPU Instructions for AES:
- Hardware AES encryption/decryption
- 5-10x faster than software
- Lower CPU usage

Check Support:
# Linux
grep aes /proc/cpuinfo

# Ensure OpenSSL uses AES-NI
openssl speed -evp aes-256-gcm
```

**Crypto Accelerators:**
```
Dedicated Hardware:
- Intel QuickAssist Technology (QAT)
- Cavium OCTEON
- Marvell NITROX

Benefits:
- Offload crypto operations
- Higher throughput
- Lower CPU usage
- Scalability
```

### 12.4 Protocol Selection

**Performance Comparison:**
```
Throughput (1 Gbps link, AES-256):

WireGuard:    950 Mbps (Best)
IPsec/IKEv2:  900 Mbps (with AES-NI)
OpenVPN UDP:  600 Mbps
OpenVPN TCP:  400 Mbps (Worst)

Recommendation:
- Maximum performance: WireGuard
- Enterprise features: IPsec
- Cross-platform: OpenVPN
```

**Latency Optimization:**
```
Protocol Selection:
- UDP-based: Lower latency
- TCP-based: Higher latency (avoid)

QoS Configuration:
- Priority queues for VPN traffic
- DSCP marking (EF, AF41)
- Traffic shaping

Example:
iptables -t mangle -A OUTPUT -p udp --dport 51820 -j DSCP --set-dscp-class EF
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

## 14. Implementation Guidelines

### 14.1 Protocol Selection Matrix

```
Scenario → Protocol Recommendation:

Modern Infrastructure:
- Recommendation: WireGuard
- Reason: Performance, simplicity, security

Enterprise Environment:
- Recommendation: IPsec/IKEv2
- Reason: Maturity, features, support

Cross-Platform:
- Recommendation: OpenVPN
- Reason: Universal support

Mobile Users:
- Recommendation: IKEv2 or WireGuard
- Reason: Roaming support, battery efficiency

Legacy Systems:
- Recommendation: L2TP/IPsec
- Reason: Native Windows support
```

### 14.2 Deployment Checklist

**Pre-Deployment:**
- [ ] Define security requirements
- [ ] Select VPN protocol
- [ ] Plan IP addressing (no overlap)
- [ ] Design routing (full/split tunnel)
- [ ] Certificate infrastructure (if needed)
- [ ] Authentication method (RADIUS, LDAP)
- [ ] High availability requirements

**Deployment:**
- [ ] Install and configure VPN gateway
- [ ] Configure firewall rules
- [ ] Set up authentication server
- [ ] Generate certificates/keys
- [ ] Configure client software
- [ ] Test connectivity
- [ ] Verify encryption
- [ ] Performance testing

**Post-Deployment:**
- [ ] Monitoring and logging
- [ ] User training
- [ ] Documentation
- [ ] Incident response plan
- [ ] Regular security audits
- [ ] Certificate renewal process
- [ ] Backup and disaster recovery

### 14.3 Troubleshooting

**Common Issues:**
```
1. Cannot Connect
   - Check firewall rules (UDP 500, 4500, 51820, etc.)
   - Verify NAT-T support
   - Check authentication credentials
   - Verify certificate validity

2. Slow Performance
   - Check MTU settings
   - Verify hardware acceleration
   - Monitor CPU usage
   - Test bandwidth

3. Intermittent Disconnects
   - Check DPD/keepalive settings
   - Verify idle timeout
   - Check network stability
   - Review logs

4. Routing Issues
   - Verify routing tables
   - Check split tunnel config
   - Verify firewall rules
   - Test DNS resolution
```

**Diagnostic Commands:**
```bash
# IPsec
ipsec status
ipsec statusall
ip xfrm state
ip xfrm policy

# OpenVPN
openvpn --show-ciphers
systemctl status openvpn@server

# WireGuard
wg show
wg show wg0 allowed-ips
ip link show wg0

# General
ip route show
ip addr show
tcpdump -i any -n esp
```

---

## 15. References

### Standards Bodies
- **IETF**: RFC 4301 (IPsec), RFC 7296 (IKEv2), RFC 5246 (TLS)
- **ISO/IEC**: 29192 (Lightweight cryptography)
- **NIST**: SP 800-77 (IPsec VPN), SP 800-52 (TLS)

### RFCs
- RFC 4301: Security Architecture for the Internet Protocol
- RFC 7296: Internet Key Exchange Protocol Version 2 (IKEv2)
- RFC 4303: IP Encapsulating Security Payload (ESP)
- RFC 4302: IP Authentication Header (AH)
- RFC 2661: Layer Two Tunneling Protocol "L2TP"
- RFC 8446: The Transport Layer Security (TLS) Protocol Version 1.3

### WIA Standards
- WIA-INTENT: Intent-based network configuration
- WIA-OMNI-API: Universal API gateway
- WIA-SECURITY: Security framework
- WIA-NETWORK: Network infrastructure

### Research Papers
1. "WireGuard: Next Generation Kernel Network Tunnel" (Donenfeld, 2017)
2. "Analysis of VPN Protocol Security" (IEEE, 2024)
3. "Performance Evaluation of VPN Protocols" (ACM, 2024)

---

**弘益人間 (Benefit All Humanity)**

*This specification is maintained by the WIA Communication Research Group and is continuously updated to reflect the latest advancements in VPN technology and security.*

*© 2025 SmileStory Inc. / WIA - MIT License*
