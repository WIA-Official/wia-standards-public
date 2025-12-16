# WIA-TLS-LITE v1.0 Specification

> Lightweight TLS for Resource-Constrained Devices
>
> 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

**Status:** Draft
**Version:** 1.0
**Date:** 2025-12-15
**Author:** World Certification Industry Association

---

## Abstract

WIA-TLS-LITE is a lightweight transport layer security protocol designed for IoT devices, embedded systems, and resource-constrained environments where standard TLS 1.3 is too heavy.

## Problem Statement

| Metric | TLS 1.3 | Target (WIA-TLS-LITE) |
|--------|---------|----------------------|
| Handshake Messages | 2-RTT | 1-RTT (0-RTT resumption) |
| Code Size | ~100KB | <20KB |
| RAM Usage | ~40KB | <8KB |
| Flash Usage | ~200KB | <50KB |
| Handshake Time | ~500ms | <100ms |
| Cipher Suites | 5+ | 2 (optimized) |

## Design Goals

1. **Minimal Footprint** - Run on 32KB RAM microcontrollers
2. **Fast Handshake** - Sub-100ms connection setup
3. **IoT-Optimized** - MQTT, CoAP, HTTP/2 friendly
4. **Interoperable** - Fallback to TLS 1.3 when possible
5. **Secure** - No compromise on cryptographic security

## Protocol Overview

### Cipher Suites (Mandatory)

```
WIA_TLS_LITE_CHACHA20_POLY1305_SHA256  (0x1301)
WIA_TLS_LITE_AES_128_GCM_SHA256        (0x1302)
```

### Key Exchange (Mandatory)

```
X25519 (Curve25519 ECDH)
```

### Signature (Mandatory)

```
Ed25519 (EdDSA)
```

## Handshake Protocol

### 1-RTT Full Handshake

```
Client                                    Server
  |                                         |
  |  ClientHello                           |
  |  + key_share (X25519)                  |
  |  + supported_versions (WIA-TLS-LITE)   |
  | --------------------------------------> |
  |                                         |
  |                        ServerHello     |
  |                        + key_share     |
  |               EncryptedExtensions      |
  |                         Certificate    |
  |                   CertificateVerify    |
  |                            Finished    |
  | <-------------------------------------- |
  |                                         |
  |  Finished                              |
  | --------------------------------------> |
  |                                         |
  |  [Application Data]  <--------------->  |
```

### 0-RTT Resumption (PSK)

```
Client                                    Server
  |                                         |
  |  ClientHello                           |
  |  + pre_shared_key                      |
  |  + early_data                          |
  |  [Application Data (0-RTT)]            |
  | --------------------------------------> |
  |                                         |
  |                        ServerHello     |
  |                        + pre_shared_key |
  |               EncryptedExtensions      |
  |                            Finished    |
  | <-------------------------------------- |
  |                                         |
  |  [Application Data]  <--------------->  |
```

## Record Layer

### Record Format (Simplified)

```
struct {
    ContentType type;           // 1 byte
    uint16 length;              // 2 bytes (max 16KB)
    opaque fragment[length];    // Encrypted payload
} WIATLSLiteRecord;
```

### Content Types

```
enum {
    change_cipher_spec(20),     // Legacy, ignored
    alert(21),
    handshake(22),
    application_data(23),
    heartbeat(24),              // Optional, for keep-alive
} ContentType;
```

## Extensions

### Mandatory Extensions

| Extension | Code | Description |
|-----------|------|-------------|
| supported_versions | 0x002B | Must include WIA-TLS-LITE |
| key_share | 0x0033 | X25519 key exchange |
| signature_algorithms | 0x000D | Ed25519 |

### Optional Extensions

| Extension | Code | Description |
|-----------|------|-------------|
| pre_shared_key | 0x0029 | Session resumption |
| early_data | 0x002A | 0-RTT data |
| heartbeat | 0x000F | Keep-alive for IoT |
| compress_certificate | 0x001B | Reduce cert size |

## Certificate Handling

### Compressed Certificates

WIA-TLS-LITE supports certificate compression to reduce bandwidth:

```
struct {
    CertificateCompressionAlgorithm algorithm;
    uint24 uncompressed_length;
    opaque compressed_certificate<1..2^24-1>;
} CompressedCertificate;

enum {
    zlib(1),
    brotli(2),
    zstd(3),
} CertificateCompressionAlgorithm;
```

### Raw Public Keys (RPK)

For constrained devices, raw public keys can replace full certificates:

```
struct {
    ASN1_subjectPublicKeyInfo public_key;  // ~44 bytes for Ed25519
} RawPublicKey;
```

## Session Management

### Session Tickets

```
struct {
    uint32 ticket_lifetime;        // Max 7 days
    uint32 ticket_age_add;         // Obfuscation
    opaque ticket_nonce<0..255>;
    opaque ticket<1..2^16-1>;
    Extension extensions<0..2^16-2>;
} NewSessionTicket;
```

### PSK Derivation

```
PSK = HKDF-Expand-Label(
    resumption_master_secret,
    "resumption",
    ticket_nonce,
    Hash.length
)
```

## Security Considerations

### Minimum Security Requirements

- Key exchange: 128-bit security (X25519)
- Symmetric encryption: 128-bit (AES-128-GCM or ChaCha20-Poly1305)
- Hash: 256-bit (SHA-256)
- Signature: 128-bit (Ed25519)

### Prohibited Features

- RSA key exchange (removed)
- Static DH (removed)
- CBC mode ciphers (removed)
- MD5, SHA-1 (removed)
- Export ciphers (removed)
- Compression (removed - CRIME attack)

### Downgrade Prevention

ClientHello must include:
```
supported_versions: [WIA-TLS-LITE, TLS 1.3]
```

Server must reject if client claims WIA-TLS-LITE but sends TLS 1.2 parameters.

## Implementation Requirements

### Minimum Hardware

| Resource | Requirement |
|----------|-------------|
| RAM | 8 KB |
| Flash | 50 KB |
| CPU | 32-bit ARM Cortex-M0+ or equivalent |

### Reference Implementation

```c
// Minimal API
wia_tls_lite_context* wia_tls_lite_init(void);
int wia_tls_lite_connect(wia_tls_lite_context* ctx, const char* host);
int wia_tls_lite_send(wia_tls_lite_context* ctx, const uint8_t* data, size_t len);
int wia_tls_lite_recv(wia_tls_lite_context* ctx, uint8_t* buf, size_t max_len);
void wia_tls_lite_close(wia_tls_lite_context* ctx);
```

### Memory Layout (8KB RAM)

```
+------------------+
| Handshake Buffer | 2 KB
+------------------+
| Record Buffer    | 2 KB
+------------------+
| Crypto State     | 1 KB
+------------------+
| Session Cache    | 1 KB
+------------------+
| Stack/Heap       | 2 KB
+------------------+
```

## Interoperability

### TLS 1.3 Fallback

If server doesn't support WIA-TLS-LITE:

1. Server responds with `supported_versions: TLS 1.3`
2. Client falls back to standard TLS 1.3
3. Connection proceeds with full TLS 1.3

### Protocol Identification

```
ALPN: wia-tls-lite/1
```

## Test Vectors

### X25519 Key Exchange

```
Client Private Key (32 bytes):
  77076d0a7318a57d3c16c17251b26645df4c2f87ebc0992ab177fba51db92c2a

Client Public Key (32 bytes):
  8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a

Server Private Key (32 bytes):
  5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2f8b27ff88e0eb

Server Public Key (32 bytes):
  de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f

Shared Secret (32 bytes):
  4a5d9d5ba4ce2de1728e3bf480350f25e07e21c947d19e3376f09b3c1e161742
```

## IANA Considerations

### Protocol Version

```
WIA-TLS-LITE 1.0: 0x0305
```

### Cipher Suite Registry

```
WIA_TLS_LITE_CHACHA20_POLY1305_SHA256: 0x1391
WIA_TLS_LITE_AES_128_GCM_SHA256:       0x1392
```

## References

- RFC 8446 - TLS 1.3
- RFC 7748 - X25519
- RFC 8032 - Ed25519
- RFC 7250 - Raw Public Keys in TLS
- RFC 8879 - Certificate Compression

---

## Appendix A: Comparison with Existing Protocols

| Feature | TLS 1.3 | DTLS 1.3 | WIA-TLS-LITE |
|---------|---------|----------|--------------|
| Stream Transport | Yes | No | Yes |
| Datagram Transport | No | Yes | Planned |
| 0-RTT | Yes | Yes | Yes |
| Code Size | ~100KB | ~120KB | <20KB |
| RAM | ~40KB | ~50KB | <8KB |
| IoT Optimized | No | Partial | Yes |
| Raw Public Keys | Optional | Optional | Mandatory |

---

**World Certification Industry Association**

https://wia.family

홍익인간 (弘益人間) - Benefit All Humanity
