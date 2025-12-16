# WIA-DPKI v1.0 Specification

> Decentralized Public Key Infrastructure
>
> 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

**Status:** Draft
**Version:** 1.0
**Date:** 2025-12-15
**Author:** World Certification Industry Association

---

## Abstract

WIA-DPKI is a decentralized public key infrastructure that eliminates dependence on centralized Certificate Authorities (CAs) by using Decentralized Identifiers (DIDs) and distributed ledger technology.

## Problem Statement

### Current PKI Problems

| Problem | Impact |
|---------|--------|
| CA Single Point of Failure | DigiNotar breach (2011) compromised all users |
| CA Trust Monopoly | ~5 CAs control 98% of market |
| Certificate Cost | $100-$1000/year for EV certificates |
| Revocation Delay | CRL/OCSP can take hours to propagate |
| Geographic Restrictions | Some CAs blocked in certain countries |
| Privacy | CA knows all your domains |

### WIA-DPKI Solution

| Feature | WIA-DPKI |
|---------|----------|
| Trust Model | Decentralized, no single CA |
| Cost | Free (gas fees only) |
| Revocation | Instant (on-chain) |
| Privacy | Self-sovereign identity |
| Availability | Global, censorship-resistant |

## Architecture

### Overview

```
+-------------------+     +-------------------+
|   DID Document    |     |   DID Document    |
|   (Identity A)    |     |   (Identity B)    |
+--------+----------+     +----------+--------+
         |                           |
         v                           v
+--------+---------------------------+--------+
|              WIA-DPKI Registry              |
|         (Distributed Ledger/DHT)           |
+--------------------------------------------+
         |                           |
         v                           v
+--------+----------+     +----------+--------+
|   Verifiable      |     |   Verifiable      |
|   Credential      |     |   Credential      |
+-------------------+     +-------------------+
```

### Components

1. **DID (Decentralized Identifier)** - Self-sovereign identity
2. **DID Document** - Public keys and service endpoints
3. **WIA Registry** - Decentralized storage for DID Documents
4. **Verifiable Credentials** - Attestations (replaces certificates)
5. **Resolver** - Looks up DID Documents

## DID Method: `did:wia`

### DID Syntax

```
did:wia:<network>:<identifier>

Examples:
did:wia:mainnet:0x1234567890abcdef1234567890abcdef12345678
did:wia:testnet:z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK
```

### Networks

| Network | Description | Ledger |
|---------|-------------|--------|
| mainnet | Production | Ethereum L2 (Optimism/Arbitrum) |
| testnet | Testing | Sepolia |
| local | Development | Local Hardhat |
| ipfs | Serverless | IPFS + IPNS |

## DID Document

### Schema

```json
{
  "@context": [
    "https://www.w3.org/ns/did/v1",
    "https://wiastandards.com/ns/dpki/v1"
  ],
  "id": "did:wia:mainnet:0x1234...5678",
  "controller": "did:wia:mainnet:0x1234...5678",

  "verificationMethod": [
    {
      "id": "did:wia:mainnet:0x1234...5678#key-1",
      "type": "Ed25519VerificationKey2020",
      "controller": "did:wia:mainnet:0x1234...5678",
      "publicKeyMultibase": "z6Mkf5rGMoatrSj1f4CyvuHBeXJELe9RPdzo2PKGNCKVtZxP"
    },
    {
      "id": "did:wia:mainnet:0x1234...5678#key-2",
      "type": "X25519KeyAgreementKey2020",
      "controller": "did:wia:mainnet:0x1234...5678",
      "publicKeyMultibase": "z6LSbysY2xFMRpGMhb7tFTLMpeuPRaqaWM1yECx2AtzE3KCc"
    }
  ],

  "authentication": ["#key-1"],
  "assertionMethod": ["#key-1"],
  "keyAgreement": ["#key-2"],

  "service": [
    {
      "id": "did:wia:mainnet:0x1234...5678#web",
      "type": "LinkedDomains",
      "serviceEndpoint": "https://example.com"
    },
    {
      "id": "did:wia:mainnet:0x1234...5678#messaging",
      "type": "DIDCommMessaging",
      "serviceEndpoint": "https://example.com/didcomm"
    }
  ],

  "created": "2025-12-15T00:00:00Z",
  "updated": "2025-12-15T00:00:00Z"
}
```

## Domain Binding

### Linking DID to Domain

To prove ownership of a domain:

#### Method 1: DNS TXT Record

```
_did.example.com  TXT  "did:wia:mainnet:0x1234...5678"
```

#### Method 2: Well-Known URI

```
GET https://example.com/.well-known/did.json

Response:
{
  "id": "did:wia:mainnet:0x1234...5678",
  "domain": "example.com",
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-15T00:00:00Z",
    "proofPurpose": "assertionMethod",
    "verificationMethod": "did:wia:mainnet:0x1234...5678#key-1",
    "proofValue": "z58DAdFfa9SkqZMVPxAQpic7ndTe..."
  }
}
```

### Browser Verification Flow

```
1. Browser connects to https://example.com
2. Server presents DID instead of certificate
3. Browser resolves did:wia:mainnet:0x1234...
4. Browser fetches DID Document from WIA Registry
5. Browser verifies domain binding (DNS or .well-known)
6. Browser verifies TLS handshake signature with DID key
7. Connection established (green lock)
```

## Smart Contract

### WIA Registry Contract

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

interface IWIARegistry {

    struct DIDDocument {
        bytes32 didHash;
        address controller;
        bytes publicKey;
        string serviceEndpoint;
        uint256 created;
        uint256 updated;
        bool revoked;
    }

    event DIDRegistered(bytes32 indexed didHash, address indexed controller);
    event DIDUpdated(bytes32 indexed didHash, address indexed controller);
    event DIDRevoked(bytes32 indexed didHash, address indexed controller);

    function register(
        bytes32 didHash,
        bytes calldata publicKey,
        string calldata serviceEndpoint
    ) external;

    function update(
        bytes32 didHash,
        bytes calldata newPublicKey,
        string calldata newServiceEndpoint
    ) external;

    function revoke(bytes32 didHash) external;

    function resolve(bytes32 didHash) external view returns (DIDDocument memory);

    function isRevoked(bytes32 didHash) external view returns (bool);

    function transferControl(bytes32 didHash, address newController) external;
}
```

### Gas Costs (Estimated)

| Operation | Gas | Cost (@ 10 gwei) |
|-----------|-----|------------------|
| Register | ~100,000 | ~$0.10 |
| Update | ~50,000 | ~$0.05 |
| Revoke | ~30,000 | ~$0.03 |
| Resolve | ~10,000 | Free (view) |

## Verifiable Credentials

### Replacing X.509 Certificates

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wiastandards.com/ns/dpki/v1"
  ],
  "type": ["VerifiableCredential", "DomainOwnershipCredential"],
  "issuer": "did:wia:mainnet:0xABCD...DCBA",
  "issuanceDate": "2025-12-15T00:00:00Z",
  "expirationDate": "2026-12-15T00:00:00Z",

  "credentialSubject": {
    "id": "did:wia:mainnet:0x1234...5678",
    "domain": "example.com",
    "organizationName": "Example Inc.",
    "country": "US"
  },

  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-15T00:00:00Z",
    "proofPurpose": "assertionMethod",
    "verificationMethod": "did:wia:mainnet:0xABCD...DCBA#key-1",
    "proofValue": "z58DAdFfa9SkqZMVPxAQpic7ndTe..."
  }
}
```

### Trust Anchors

Instead of root CAs, WIA-DPKI uses community-curated trust anchors:

| Trust Anchor | Type | Description |
|--------------|------|-------------|
| `did:wia:mainnet:wia-root` | Foundation | WIA Foundation root |
| `did:wia:mainnet:community` | Community | Community-elected |
| `did:wia:mainnet:gov-*` | Government | National registrars |
| User-defined | Custom | Personal trust list |

## Revocation

### Instant On-Chain Revocation

```solidity
function revoke(bytes32 didHash) external {
    require(msg.sender == documents[didHash].controller, "Not controller");
    documents[didHash].revoked = true;
    emit DIDRevoked(didHash, msg.sender);
}
```

### Checking Revocation

```javascript
const isRevoked = await wiaRegistry.isRevoked(didHash);
// Instant - no CRL download, no OCSP query
```

## TLS Integration

### WIA-DPKI TLS Extension

```
Extension: wia_dpki (0x0055)

struct {
    opaque did<1..2^16-1>;            // DID string
    opaque did_document<1..2^16-1>;   // Cached DID Document (optional)
    opaque domain_proof<1..2^16-1>;   // Domain binding proof
} WIADPKIExtension;
```

### Handshake Flow

```
Client                                    Server
  |                                         |
  |  ClientHello                           |
  |  + wia_dpki (request)                  |
  | --------------------------------------> |
  |                                         |
  |                        ServerHello     |
  |  + wia_dpki (DID + proof)              |
  |                                         |
  |  [Client resolves DID, verifies]       |
  |                                         |
  |  Finished                              |
  | --------------------------------------> |
```

## Migration Path

### Phase 1: Hybrid Mode

- Servers present both X.509 certificate AND DID
- Browsers that support WIA-DPKI use DID
- Legacy browsers use X.509

### Phase 2: DID-Preferred

- WIA-DPKI is default
- X.509 as fallback only

### Phase 3: DID-Only

- X.509 deprecated
- Full decentralization achieved

## Privacy Considerations

### Selective Disclosure

DID owners can choose what to reveal:

```json
{
  "credentialSubject": {
    "id": "did:wia:mainnet:0x1234...5678",
    "domain": "example.com"
    // Organization name NOT disclosed
  }
}
```

### Unlinkability

- Use different DIDs for different contexts
- Key rotation without changing DID
- Zero-knowledge proofs for verification

## Security Considerations

### Key Management

- Hardware security modules (HSM) recommended
- Multi-signature control for high-value DIDs
- Social recovery mechanisms

### Attack Vectors

| Attack | Mitigation |
|--------|------------|
| 51% Attack | Use established L2 with economic security |
| Key Compromise | Instant revocation + key rotation |
| DID Squatting | First-come-first-served + trademark dispute resolution |
| Replay Attack | Timestamp + nonce in proofs |

## Implementation

### JavaScript SDK

```javascript
import { WIAResolver, WIARegistry } from '@wia/dpki';

// Resolve DID
const resolver = new WIAResolver();
const didDocument = await resolver.resolve('did:wia:mainnet:0x1234...5678');

// Register new DID
const registry = new WIARegistry(provider);
const did = await registry.register({
  publicKey: myPublicKey,
  serviceEndpoint: 'https://example.com'
});

// Verify domain
const isValid = await resolver.verifyDomainBinding('did:wia:mainnet:0x1234...', 'example.com');
```

### CLI Tool

```bash
# Create new DID
wia-dpki create --network mainnet

# Resolve DID
wia-dpki resolve did:wia:mainnet:0x1234...5678

# Bind domain
wia-dpki bind-domain --did did:wia:mainnet:0x1234... --domain example.com

# Revoke DID
wia-dpki revoke did:wia:mainnet:0x1234...5678
```

## IANA Considerations

### DID Method Registration

```
Method Name: wia
Method Specification: https://wiastandards.com/specs/dpki
```

### TLS Extension

```
Extension Name: wia_dpki
Extension Code: 0x0055 (proposed)
```

## References

- W3C DID Core 1.0
- W3C Verifiable Credentials 1.0
- EIP-1056 - Ethereum Lightweight Identity
- RFC 8446 - TLS 1.3
- Certificate Transparency (RFC 6962)

---

**World Certification Industry Association**

https://wiastandards.com

홍익인간 (弘益人間) - Benefit All Humanity
