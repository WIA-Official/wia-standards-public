# WIA-FIN-004 Digital Currency Standard
## Phase 3: Protocol Specification

**Version:** 1.0  
**Status:** Production Ready  
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the communication protocols and interoperability standards for digital currency systems to exchange value and data across platforms.

---

## 2. Cross-Platform Payment Protocol

### 2.1 Payment Initiation

```
┌─────────┐   1. Initiate    ┌──────────┐   2. Route     ┌────────────┐
│ Sender  │ ──────────────> │ Gateway  │ ────────────> │ Recipient  │
│ System  │                 │          │               │ System     │
└─────────┘                 └──────────┘               └────────────┘
                                  │
                            3. Settle
                                  │
                                  ▼
                           ┌──────────────┐
                           │ Settlement   │
                           │ Network      │
                           └──────────────┘
```

### 2.2 Message Format

```json
{
  "protocol": "WIA-FIN-004-v1.0",
  "messageId": "UUID",
  "messageType": "PAYMENT_INITIATION",
  "timestamp": "ISO_8601",
  "sender": {
    "systemId": "string",
    "accountId": "string"
  },
  "recipient": {
    "systemId": "string",
    "accountId": "string"
  },
  "amount": {
    "value": "decimal",
    "currency": "string"
  },
  "settlement": {
    "method": "INSTANT | BATCH",
    "network": "string"
  },
  "signature": "digital_signature"
}
```

---

## 3. Atomic Swap Protocol

### 3.1 Hash Time-Locked Contract (HTLC)

```
1. Alice generates secret S, computes H = hash(S)
2. Alice → Bob: "I'll pay you 100 USDC if you reveal S within 24h"
3. Bob → Alice: "I'll pay you 1 ETH if you reveal S within 24h"
4. Bob reveals S to claim 100 USDC
5. Alice uses S to claim 1 ETH
6. If timeout expires, both get refunded
```

### 3.2 Smart Contract Interface

```solidity
interface IAtomicSwap {
    function initiate(
        address participant,
        bytes32 secretHash,
        uint256 lockTime
    ) external payable;
    
    function redeem(bytes32 secret) external;
    
    function refund() external;
}
```

---

## 4. Cross-Border Settlement

### 4.1 Multi-CBDC Bridge Protocol

```
┌──────┐   CBDC-A    ┌────────┐   Bridge    ┌────────┐   CBDC-B    ┌──────┐
│ Bank │ ─────────> │ Central │ ─────────> │ Central │ ─────────> │ Bank │
│  A   │            │ Bank A  │            │ Bank B  │            │  B   │
└──────┘            └────────┘            └────────┘            └──────┘
                         │                      │
                         └──────────────────────┘
                         Atomic Settlement Network
```

### 4.2 Settlement Message

```json
{
  "settlementId": "UUID",
  "fromCurrency": "CBDC-USD",
  "toCurrency": "CBDC-EUR",
  "amount": "decimal",
  "exchangeRate": "decimal",
  "participants": [
    {
      "role": "ORIGINATOR | BENEFICIARY",
      "centralBank": "FED | ECB",
      "institution": "string"
    }
  ],
  "atomicConditions": {
    "allOrNothing": true,
    "timelock": "ISO_8601"
  }
}
```

---

## 5. Interledger Protocol (ILP) Integration

### 5.1 ILP Payment Flow

```
Sender → Connector A → Connector B → Recipient
  │          │             │            │
  │    Ledger A       Ledger B      Ledger C
  │          │             │            │
  └──────────┴─────────────┴────────────┘
        Atomic Multi-Hop Transfer
```

### 5.2 ILP Packet Format

```json
{
  "ilp_version": "1.0",
  "source_address": "g.alice.usd",
  "destination_address": "g.bob.eur",
  "amount": "100.00",
  "data": {
    "wia_fin_004": {
      "transactionId": "UUID",
      "compliance": { /* ... */ }
    }
  },
  "execution_condition": "hash",
  "expiration": "ISO_8601"
}
```

---

## 6. Privacy-Preserving Protocols

### 6.1 Zero-Knowledge Compliance Proof

```
Prover (User)                    Verifier (Regulator/System)
      │                                     │
      │  1. Generate ZK Proof               │
      │     "I am KYC verified"             │
      │     "Transaction is compliant"       │
      │     (without revealing identity)     │
      │                                     │
      │  2. Send Proof ──────────────────> │
      │                                     │
      │  3. Verify Proof                    │
      │     ✓ KYC: Valid                    │
      │     ✓ Amount within limits          │
      │     ✓ Not sanctioned                │
      │                                     │
      │  4. Accept/Reject <──────────────  │
```

### 6.2 ZK Proof Structure

```json
{
  "proofType": "zk-SNARK | zk-STARK",
  "claim": "KYC_VERIFIED | AML_COMPLIANT",
  "proof": "base64_encoded_proof",
  "publicInputs": {
    "accountHash": "hash",
    "timestamp": "ISO_8601",
    "jurisdiction": "ISO_3166"
  },
  "verificationKey": "public_key"
}
```

---

## 7. Security Protocols

### 7.1 Transaction Signing

```javascript
// 1. Create transaction
const tx = {
  from: "ACC-123",
  to: "ACC-456",
  amount: "100.00",
  currency: "USDC",
  nonce: 42,
  timestamp: Date.now()
};

// 2. Serialize
const message = canonicalizeJSON(tx);

// 3. Hash
const hash = SHA256(message);

// 4. Sign
const signature = ECDSA.sign(hash, privateKey);

// 5. Broadcast
broadcastTransaction({
  transaction: tx,
  signature: signature,
  publicKey: publicKey
});
```

### 7.2 Multi-Signature Protocol

```
Required: 2-of-3 signatures

Signers: Alice, Bob, Carol

1. Transaction created
2. Alice signs → Partial signature 1
3. Bob signs → Partial signature 2
4. Combine signatures → Valid multi-sig
5. Broadcast transaction
```

---

## 8. Network Protocols

### 8.1 Peer-to-Peer Messaging

```
Node Discovery → Handshake → Message Exchange → Consensus

1. Bootstrap nodes
2. Peer discovery (DHT)
3. Version negotiation
4. Transaction propagation
5. Block/state sync
```

### 8.2 Consensus Protocol

**Proof of Stake (Recommended)**

```
1. Validators stake tokens
2. Leader selection (pseudorandom)
3. Block proposal
4. Attestations (2/3+ validators)
5. Finality reached
6. Rewards distributed
```

---

## 9. Compliance Protocol

### 9.1 Travel Rule Implementation

```json
{
  "travelRule": {
    "threshold": {
      "amount": "1000.00",
      "currency": "USD"
    },
    "originator": {
      "name": "Alice Smith",
      "accountNumber": "ACC-123",
      "address": "123 Main St, City, Country",
      "vasp": "Exchange A"
    },
    "beneficiary": {
      "name": "Bob Johnson",
      "accountNumber": "ACC-456",
      "vasp": "Exchange B"
    },
    "transmitted": {
      "method": "InterVASP | TRUST | Direct",
      "timestamp": "ISO_8601",
      "confirmation": "receipt_hash"
    }
  }
}
```

---

## 10. Disaster Recovery Protocol

### 10.1 System Failover

```
Primary System     Backup System      Recovery
     │                  │                  │
     │ Normal Ops       │ Standby          │
     │                  │                  │
     X Failure          │                  │
     │                  │                  │
     │ <─── Detect ─────┤                  │
     │                  │                  │
     │           Activate Backup           │
     │                  │                  │
     │                  ▼                  │
     │             Traffic Switch          │
     │                  │                  │
     │                  │  Continue Ops    │
```

### 10.2 State Recovery

```
1. Detect failure
2. Switch to backup
3. Replay transaction log
4. Verify state consistency
5. Resume operations
6. Investigate failure
```

---

**End of Phase 3 Specification**

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for digital-currency is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/digital-currency/api/` — TypeScript SDK skeleton
- `wia-standards/standards/digital-currency/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/digital-currency/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


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
