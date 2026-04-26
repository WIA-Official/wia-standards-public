# WIA-EDU-011 Digital Credential Standard v1.2

## Phase 3: Protocol & Blockchain Integration

**Status:** ✅ Complete
**Version:** 1.2.0
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 extends Phase 2 by defining blockchain anchoring protocols, smart contract specifications, and decentralized verification mechanisms. This ensures credentials have immutable, publicly-verifiable proofs.

## 2. Scope

Phase 3 covers:
- Blockchain anchoring protocol
- Smart contract specifications
- Multi-chain support
- Revocation lists on-chain
- Gas optimization strategies
- Decentralized storage integration

## 3. Blockchain Requirements

### 3.1 Supported Networks

**Primary Networks:**
- Ethereum Mainnet
- Polygon (MATIC)
- Arbitrum

**Testnet Support:**
- Sepolia (Ethereum)
- Mumbai (Polygon)

### 3.2 Smart Contract Architecture

**Core Contracts:**
- `WIACredentialRegistry.sol` - Credential anchoring
- `WIAIssuerRegistry.sol` - Trusted issuers
- `WIARevocationList.sol` - Revocation management

## 4. Credential Anchoring Protocol

### 4.1 Hash Anchoring

**Process:**
1. Generate SHA-256 hash of credential JSON-LD
2. Create Merkle tree for batch credentials
3. Anchor Merkle root to blockchain
4. Store transaction hash in credential proof

**Hash Format:**
```
0x{sha256(canonicalized-json-ld)}
```

### 4.2 Smart Contract Integration

**Solidity Interface:**
```solidity
interface IWIACredentialRegistry {
    function anchorCredential(
        bytes32 credentialHash,
        string calldata did,
        uint256 issuanceDate
    ) external returns (uint256 credentialId);

    function verifyCredential(
        bytes32 credentialHash
    ) external view returns (bool exists, uint256 timestamp);

    function revokeCredential(
        bytes32 credentialHash,
        string calldata reason
    ) external;
}
```

## 5. Batch Issuance Optimization

### 5.1 Merkle Tree Batching

**Benefits:**
- Issue 100+ credentials in single transaction
- Cost: $0.01-$0.05 per credential
- Proof size: ~1 KB per credential

**Merkle Proof Structure:**
```json
{
  "merkleRoot": "0x...",
  "proof": [
    "0x...",
    "0x...",
    "0x..."
  ],
  "leafIndex": 42
}
```

### 5.2 Gas Optimization

**Strategies:**
- Batch transactions during low gas periods
- Use EIP-1559 gas pricing
- Implement on Polygon for <$0.10 per batch
- Store only hashes, not full data

## 6. Revocation Management

### 6.1 Bitmap-Based Revocation

**RevocationList2020 Implementation:**
```solidity
contract WIARevocationList {
    mapping(uint256 => uint256) private revocationBitmap;

    function revoke(uint256 credentialIndex) external onlyIssuer {
        uint256 bucket = credentialIndex / 256;
        uint256 position = credentialIndex % 256;
        revocationBitmap[bucket] |= (1 << position);
    }

    function isRevoked(uint256 credentialIndex)
        external view returns (bool) {
        uint256 bucket = credentialIndex / 256;
        uint256 position = credentialIndex % 256;
        return (revocationBitmap[bucket] & (1 << position)) != 0;
    }
}
```

### 6.2 Status List 2021

**Privacy-Preserving Revocation:**
- Herd privacy for revocation checks
- Bitstring compression
- Efficient storage: 131,072 credentials per list

## 7. Decentralized Storage

### 7.1 IPFS Integration

**Credential Storage:**
```
ipfs://{CID}
```

**Benefits:**
- Decentralized, immutable storage
- Content-addressed retrieval
- Redundancy through pinning services

### 7.2 Arweave for Permanent Storage

**Long-term Archival:**
- One-time payment for permanent storage
- Historical credential lookups
- Compliance with retention policies

## 8. Multi-Chain Support

### 8.1 Cross-Chain Verification

**Supported Bridges:**
- LayerZero for omnichain messaging
- Chainlink CCIP for cross-chain verification
- Wormhole for multi-chain state

### 8.2 Chain Selection Strategy

| Use Case | Recommended Chain | Cost | Speed |
|----------|------------------|------|-------|
| High-value degrees | Ethereum | $5-20 | 15s |
| Batch issuance | Polygon | $0.01 | 2s |
| Enterprise | Arbitrum | $0.50 | 1s |
| Testing | Sepolia | Free | 15s |

## 9. Blockchain Verification Protocol

### 9.1 On-Chain Verification

**Steps:**
1. Extract credential hash from VC
2. Query blockchain for hash existence
3. Verify timestamp matches issuance date
4. Check issuer DID is authorized
5. Verify revocation status

**Web3.js Example:**
```javascript
const registry = new web3.eth.Contract(ABI, ADDRESS);
const result = await registry.methods
  .verifyCredential(credentialHash)
  .call();

if (result.exists) {
  console.log('Credential anchored at:', result.timestamp);
}
```

## 10. Event Logging

### 10.1 Smart Contract Events

```solidity
event CredentialIssued(
    bytes32 indexed credentialHash,
    string did,
    uint256 timestamp
);

event CredentialRevoked(
    bytes32 indexed credentialHash,
    string reason,
    uint256 timestamp
);
```

### 10.2 Event Indexing

- The Graph for blockchain data indexing
- SubQuery for multi-chain support
- Custom indexers for institution-specific needs

## 11. Security Considerations

### 11.1 Smart Contract Security

**Best Practices:**
- OpenZeppelin security libraries
- Multi-signature wallets for admin functions
- Time-locked upgrades
- Circuit breakers for emergency stops
- Regular security audits

### 11.2 Key Management

**Hardware Security Modules (HSM):**
- Store institutional signing keys
- AWS KMS, Azure Key Vault support
- Multi-party computation (MPC) wallets

## 12. Cost Analysis

### 12.1 Transaction Costs

**Polygon (Recommended):**
- Single credential: $0.01-$0.05
- Batch (100 credentials): $0.50-$2.00
- Revocation: $0.01-$0.03

**Ethereum Mainnet:**
- Single credential: $5-$50 (varies)
- Batch (100 credentials): $20-$200
- Revocation: $5-$30

## 13. Compliance

Phase 3 MUST comply with:
- ✅ EIP-712 (Typed structured data hashing)
- ✅ EIP-1559 (Gas pricing)
- ✅ EIP-2535 (Diamond standard for upgrades)
- ✅ OpenZeppelin security standards
- ✅ Blockchain data privacy regulations

---

**Philosophy:** 弘益人間 (Benefit All Humanity)

*WIA-EDU-011 Digital Credential Standard*

© 2025 MIT License


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
