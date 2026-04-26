# WIA-SEC-014: Hardware Security Module - Phases 2, 3 & 4

**Version:** 1.0
**Status:** Official Standard
**Last Updated:** 2025-12-25
**Category:** Security (SEC)

---

## Table of Contents

### Phase 2: Advanced Features
1. [Cloud HSM Integration](#phase-2-advanced-features)
2. [Blockchain & Cryptocurrency Support](#22-blockchain--cryptocurrency-support)
3. [Post-Quantum Cryptography](#23-post-quantum-cryptography)
4. [Advanced Key Management](#24-advanced-key-management)

### Phase 3: Enterprise Integration
5. [Multi-Tenancy Architecture](#phase-3-enterprise-integration)
6. [Federated HSM Networks](#32-federated-hsm-networks)
7. [Certificate Authority Integration](#33-certificate-authority-integration)
8. [DevOps & Automation](#34-devops--automation)

### Phase 4: Next-Generation Features
9. [AI-Powered Security](#phase-4-next-generation-features)
10. [Quantum Key Distribution](#42-quantum-key-distribution)
11. [Homomorphic Encryption Support](#43-homomorphic-encryption-support)
12. [Zero-Knowledge Proofs](#44-zero-knowledge-proofs)

---

# Phase 2: Advanced Features

## 2.1 Cloud HSM Integration

### 2.1.1 Architecture

```
┌─────────────────────────────────────────────┐
│         Cloud Provider (AWS/Azure/GCP)       │
├─────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │   HSM    │  │   HSM    │  │   HSM    │  │
│  │ Cluster  │  │ Cluster  │  │ Cluster  │  │
│  │  (US-E)  │  │  (US-W)  │  │  (EU)    │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  │
│       │             │             │         │
│  ┌────┴─────────────┴─────────────┴─────┐  │
│  │     Key Synchronization Service      │  │
│  └────────────────┬─────────────────────┘  │
└───────────────────┼──────────────────────────┘
                    │
                    ▼
            ┌──────────────┐
            │ Application  │
            │   Servers    │
            └──────────────┘
```

### 2.1.2 Cloud-Native Features

#### Multi-Region Deployment
```json
{
  "deployment": {
    "type": "multi-region",
    "regions": [
      {
        "name": "us-east-1",
        "hsmCluster": "cluster-us-east",
        "replicas": 3,
        "role": "primary"
      },
      {
        "name": "eu-west-1",
        "hsmCluster": "cluster-eu-west",
        "replicas": 2,
        "role": "secondary"
      }
    ],
    "replication": {
      "mode": "async",
      "encryptionInTransit": true,
      "compressionEnabled": false
    }
  }
}
```

#### Auto-Scaling
- Dynamic HSM instance allocation
- Load-based scaling policies
- Automatic failover and recovery
- Health monitoring and alerting

### 2.1.3 Cloud Provider Integration

#### AWS CloudHSM
```python
import boto3

client = boto3.client('cloudhsmv2')

# Create cluster
cluster = client.create_cluster(
    SubnetIds=['subnet-123', 'subnet-456'],
    HsmType='hsm1.medium',
    SourceBackupId='backup-abc123'
)

# Initialize HSM
hsm = client.create_hsm(
    ClusterId=cluster['ClusterId'],
    AvailabilityZone='us-east-1a'
)
```

#### Azure Dedicated HSM
```bash
az keyvault create \
  --name myvault \
  --resource-group mygroup \
  --location eastus \
  --sku premium \
  --enable-hsm-key-protection true
```

#### Google Cloud HSM
```go
import "cloud.google.com/go/kms/apiv1"

client, _ := kms.NewKeyManagementClient(ctx)
req := &kmspb.CreateCryptoKeyRequest{
    Parent: "projects/my-project/locations/us/keyRings/my-ring",
    CryptoKeyId: "my-hsm-key",
    CryptoKey: &kmspb.CryptoKey{
        Purpose: kmspb.CryptoKey_ASYMMETRIC_SIGN,
        ProtectionLevel: kmspb.ProtectionLevel_HSM,
    },
}
```

---

## 2.2 Blockchain & Cryptocurrency Support

### 2.2.1 Supported Blockchains

| Blockchain | Curve | Signature Algorithm |
|------------|-------|---------------------|
| Bitcoin | secp256k1 | ECDSA |
| Ethereum | secp256k1 | ECDSA |
| Solana | Ed25519 | EdDSA |
| Cardano | Ed25519 | EdDSA |
| Polkadot | Sr25519 | Schnorr |
| Cosmos | secp256k1 | ECDSA |

### 2.2.2 Wallet Integration

#### Hierarchical Deterministic (HD) Wallets
```json
{
  "operation": "deriveKey",
  "masterKey": "master-key-handle",
  "derivationPath": "m/44'/60'/0'/0/0",
  "standard": "BIP-32/BIP-44",
  "curve": "secp256k1",
  "attributes": {
    "CKA_EXTRACTABLE": false,
    "CKA_SIGN": true,
    "label": "ethereum-wallet-0"
  }
}
```

#### Transaction Signing
```javascript
// Ethereum transaction signing
const tx = {
  nonce: '0x0',
  gasPrice: '0x4a817c800',
  gasLimit: '0x5208',
  to: '0x742d35Cc6634C0532925a3b844Bc454e4438f44e',
  value: '0xde0b6b3a7640000',
  data: '0x'
};

const serializedTx = serializeTransaction(tx);
const signature = await hsm.sign({
  keyHandle: 'eth-signing-key',
  data: keccak256(serializedTx),
  algorithm: 'ECDSA-secp256k1'
});

const signedTx = {
  ...tx,
  v: signature.v,
  r: signature.r,
  s: signature.s
};
```

### 2.2.3 Smart Contract Integration

#### Solidity Integration
```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract HSMVerifier {
    address public hsmPublicKey;

    constructor(address _hsmKey) {
        hsmPublicKey = _hsmKey;
    }

    function verifySignature(
        bytes32 messageHash,
        bytes memory signature
    ) public view returns (bool) {
        return recoverSigner(messageHash, signature) == hsmPublicKey;
    }

    function recoverSigner(
        bytes32 messageHash,
        bytes memory signature
    ) internal pure returns (address) {
        (bytes32 r, bytes32 s, uint8 v) = splitSignature(signature);
        return ecrecover(messageHash, v, r, s);
    }
}
```

---

## 2.3 Post-Quantum Cryptography

### 2.3.1 NIST PQC Standards

#### Supported Algorithms

**Key Encapsulation Mechanisms (KEM):**
- CRYSTALS-Kyber (FIPS 203)
  - Kyber512, Kyber768, Kyber1024

**Digital Signatures:**
- CRYSTALS-Dilithium (FIPS 204)
  - Dilithium2, Dilithium3, Dilithium5
- SPHINCS+ (FIPS 205)
  - SPHINCS+-128f, SPHINCS+-192f, SPHINCS+-256f

### 2.3.2 Hybrid Cryptography

#### Hybrid Key Exchange
```json
{
  "operation": "hybridKeyExchange",
  "classical": {
    "algorithm": "ECDH-P256",
    "keyHandle": "classical-key"
  },
  "postQuantum": {
    "algorithm": "Kyber768",
    "keyHandle": "pqc-key"
  },
  "derivation": {
    "kdf": "HKDF-SHA256",
    "info": "hybrid-key-exchange-v1"
  }
}
```

#### Migration Strategy
```
Phase 1: Classical only (current)
         RSA-2048 / ECDSA-P256
              ↓


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.
