# WIA-LEG-006: Digital Asset Inheritance Standard
## Version 1.0 - Phase 1: Data Format Specification

**Status:** Final
**Published:** 2025-01-15
**Authors:** WIA Technical Committee on Digital Legacy
**License:** MIT License

---

## 1. Introduction

### 1.1 Purpose
This specification defines the data format for digital asset inheritance plans, enabling interoperable estate planning across wallets, platforms, and legal systems.

### 1.2 Scope
Phase 1 covers:
- JSON schema for inheritance plans
- Asset inventory structures
- Beneficiary definitions
- Distribution rules
- Trigger mechanisms
- Metadata and versioning

### 1.3 Philosophy
Embodies 弘益人間 (Hongik Ingan) - "Benefit All Humanity" through universal accessibility and legal compliance.

---

## 2. Data Model Overview

### 2.1 Core Structure
```json
{
  "wiaStandard": "LEG-006",
  "version": "1.0",
  "inheritancePlan": {
    "planId": "string (UUID v4)",
    "metadata": {},
    "owner": {},
    "assets": [],
    "beneficiaries": [],
    "distribution": {},
    "triggers": [],
    "legal": {},
    "execution": {}
  },
  "signatures": []
}
```

### 2.2 Field Definitions

#### 2.2.1 Metadata
```json
"metadata": {
  "created": "ISO 8601 timestamp",
  "lastModified": "ISO 8601 timestamp",
  "version": "Semantic version (e.g., 1.2.3)",
  "previousVersionHash": "SHA-256 hash of previous version",
  "title": "Human-readable plan name",
  "description": "Plan description",
  "jurisdiction": "ISO 3166-2 code",
  "language": "IETF BCP 47 language tag",
  "currency": "ISO 4217 currency code",
  "tags": ["array of strings"],
  "confidentiality": "public|private|confidential",
  "encryption": {
    "algorithm": "AES-256-GCM|RSA-4096",
    "keyDerivation": "PBKDF2-HMAC-SHA256|Argon2id",
    "encryptedFields": ["array of JSON paths"]
  }
}
```

---

## 3. Asset Specifications

### 3.1 Cryptocurrency Assets
```json
{
  "assetId": "unique identifier",
  "type": "cryptocurrency",
  "classification": "primary-holding|savings|investment|trading",
  "cryptocurrency": {
    "symbol": "BTC|ETH|etc",
    "fullName": "Bitcoin|Ethereum|etc",
    "network": "mainnet|testnet",
    "amount": "string (decimal)",
    "walletType": "hardware|software|paper|custodial",
    "walletProvider": "string",
    "addresses": [
      {
        "address": "blockchain address",
        "type": "P2PKH|P2WPKH|etc",
        "balance": "string (decimal)",
        "derivationPath": "BIP32/BIP44 path"
      }
    ]
  },
  "access": {
    "method": "single-sig|multi-sig|shamir-secret",
    "requiredSignatures": "number (for multi-sig)",
    "totalKeys": "number (for multi-sig)",
    "keyShares": []
  },
  "valuation": {
    "acquisitionCost": {"amount": number, "currency": "string", "date": "ISO 8601"},
    "currentValue": {"amount": number, "currency": "string", "asOf": "ISO 8601"},
    "taxBasis": "FIFO|LIFO|HIFO|specific-identification"
  }
}
```

### 3.2 NFT Assets
```json
{
  "assetId": "unique identifier",
  "type": "nft",
  "classification": "art|collectible|gaming|utility|domain",
  "nft": {
    "blockchain": "Ethereum|Polygon|Solana|etc",
    "standard": "ERC-721|ERC-1155|SPL|etc",
    "contractAddress": "smart contract address",
    "tokenId": "token identifier",
    "ownerAddress": "current owner address",
    "metadata": {
      "name": "NFT name",
      "description": "NFT description",
      "image": "IPFS or HTTP URL",
      "attributes": [],
      "externalUrl": "optional external URL"
    },
    "provenance": []
  },
  "rights": {
    "copyright": "retained|transferred|shared",
    "commercialUse": "permitted|restricted|prohibited",
    "transferable": boolean,
    "royalties": {"percentage": number, "recipient": "address"}
  }
}
```

### 3.3 Digital Real Estate
```json
{
  "assetId": "unique identifier",
  "type": "digital-real-estate",
  "classification": "investment|business|personal",
  "metaverseProperty": {
    "platform": "Decentraland|The Sandbox|etc",
    "parcels": [
      {"coordinates": {}, "size": "string", "landId": "token ID"}
    ],
    "blockchain": "Ethereum|Polygon|etc",
    "contractAddress": "smart contract address",
    "tokenIds": ["array of token IDs"],
    "developments": []
  },
  "access": {
    "platformAccount": {
      "username": "string",
      "email": "string",
      "wallet": "connected wallet address"
    }
  }
}
```

---

## 4. Beneficiary Framework

### 4.1 Individual Beneficiary
```json
{
  "beneficiaryId": "unique identifier",
  "identity": {
    "did": "W3C Decentralized Identifier",
    "legalName": "full legal name",
    "relationship": "spouse|child|sibling|parent|friend|charity",
    "dateOfBirth": "ISO 8601 date",
    "contact": {
      "email": "email address",
      "phone": "E.164 format",
      "address": {}
    }
  },
  "allocation": {
    "percentage": "number 0-100",
    "specificAssets": ["array of assetIds"],
    "residualShare": "percentage of residual estate"
  },
  "conditions": {
    "ageRequirement": "minimum age",
    "educationMilestone": "description",
    "trustStructure": {
      "type": "spendthrift|discretionary|special-needs",
      "trustee": "DID of trustee",
      "distributionSchedule": []
    }
  },
  "contingent": {
    "primary": boolean,
    "alternates": ["beneficiaryIds"],
    "failureConditions": ["predeceased|disclaimed|incapacitated"]
  }
}
```

### 4.2 Organizational Beneficiary
```json
{
  "beneficiaryId": "unique identifier",
  "identity": {
    "type": "charitable-organization|trust|foundation|corporation",
    "legalName": "official registered name",
    "taxId": "EIN or equivalent",
    "charityRegistration": "501(c)(3) or equivalent",
    "wallet": "receiving address"
  },
  "allocation": {
    "percentage": "number 0-100",
    "specificAssets": [],
    "residualShare": "percentage"
  },
  "purpose": "description of charitable purpose"
}
```

---

## 5. Distribution Logic

### 5.1 Distribution Methods
- `equal`: Equal distribution among beneficiaries
- `percentage`: Custom percentage allocation
- `specific-bequest`: Specific assets to specific beneficiaries
- `residual`: Remainder after specific bequests
- `conditional`: Based on conditions being met

### 5.2 Distribution Rules Schema
```json
"distribution": {
  "method": "equal|percentage|mixed",
  "rules": [
    {
      "ruleId": "unique identifier",
      "type": "specific-bequest|percentage-split|residual",
      "asset": "assetId or pattern",
      "beneficiary": "beneficiaryId",
      "condition": "optional condition expression"
    }
  ],
  "simultaneousDeath": {
    "presumption": "beneficiary-predeceased|uniform-simultaneous-death-act",
    "survivorshipPeriod": "duration (e.g., 30 days)"
  },
  "taxOptimization": {
    "strategy": "marital-deduction|charitable-remainder|generation-skipping",
    "estateTaxExemption": number,
    "jurisdiction": "tax jurisdiction"
  }
}
```

---

## 6. Trigger Mechanisms

### 6.1 Trigger Types

#### 6.1.1 Dead Man's Switch
```json
{
  "triggerId": "unique identifier",
  "type": "dead-mans-switch",
  "inactivityPeriod": "number (days)",
  "unit": "days|hours|months",
  "checkInMethod": "biometric-app|email|sms|multi-factor",
  "notificationSchedule": [
    {
      "beforeTrigger": "days before trigger",
      "recipients": ["owner|executor|beneficiary"],
      "method": "email|sms|push|certified-mail"
    }
  ],
  "gracePeriod": "days after initial trigger",
  "resetOnActivity": boolean
}
```

#### 6.1.2 Oracle-Verified Trigger
```json
{
  "triggerId": "unique identifier",
  "type": "oracle-verified",
  "oracle": {
    "type": "death-certificate|court-order|medical-incapacity",
    "provider": "oracle service name",
    "apiEndpoint": "HTTPS URL",
    "requiredConfirmations": "number of oracle confirmations needed",
    "verificationMethod": "multi-jurisdiction-consensus|single-authority"
  }
}
```

#### 6.1.3 Manual Executor Trigger
```json
{
  "triggerId": "unique identifier",
  "type": "manual-executor",
  "authorizedParties": ["array of DIDs"],
  "requiredSignatures": "number of signatures needed",
  "signatories": ["executor|attorney|beneficiary"],
  "documentationRequired": ["death-certificate|letters-testamentary|court-order"]
}
```

### 6.2 Trigger Logic
```json
"triggerLogic": {
  "operator": "OR|AND",
  "minimumConfidence": "0.0-1.0 (e.g., 0.95)",
  "disputeResolution": {
    "arbitrator": "arbitration service",
    "appealPeriod": "days"
  }
}
```

---

## 7. Legal Compliance

### 7.1 Jurisdiction Support
- ISO 3166-2 jurisdiction codes
- Support for multiple concurrent jurisdictions
- Conflict-of-law rules specification

### 7.2 Required Legal Fields
```json
"legal": {
  "governingLaw": "ISO 3166-2 code",
  "testamentaryCapacity": {
    "certifiedBy": "attorney|physician",
    "date": "ISO 8601",
    "documentHash": "SHA-256 hash"
  },
  "witnesses": [
    {
      "name": "witness name",
      "signature": "digital signature",
      "timestamp": "ISO 8601"
    }
  ],
  "noContest": boolean,
  "arbitration": {
    "required": boolean,
    "forum": "arbitration service",
    "rules": "AAA|JAMS|ICC"
  }
}
```

---

## 8. Validation Rules

### 8.1 Required Fields
- `wiaStandard`: Must be "LEG-006"
- `version`: Must be "1.0"
- `inheritancePlan.planId`: Must be UUID v4
- `inheritancePlan.metadata.created`: Must be valid ISO 8601
- `inheritancePlan.owner.identity.did`: Must be valid DID

### 8.2 Constraints
- Total beneficiary allocation must equal 100% or less
- Asset IDs must be unique within plan
- Beneficiary IDs must be unique within plan
- Dates must not be in the future (except for scheduled events)

### 8.3 Cross-Field Validation
- Distribution rules must reference existing assets and beneficiaries
- Trigger signatories must be defined parties
- Conditional beneficiaries must have valid conditions

---

## 9. Security Considerations

### 9.1 Encryption
- All private keys and seed phrases MUST be encrypted
- Recommended: AES-256-GCM or RSA-4096
- Key derivation: PBKDF2-HMAC-SHA256 with 100,000+ iterations

### 9.2 Access Control
- Separate encryption for different data sensitivity levels
- Owner-only access: Private keys, seed phrases
- Executor access: Asset locations, beneficiary contacts
- Beneficiary access: Allocation information only

### 9.3 Audit Trail
- Every plan modification MUST include previousVersionHash
- Timestamps MUST be immutable
- Digital signatures MUST be verifiable

---

## 10. Examples

### 10.1 Minimal Valid Plan
See repository: `examples/minimal-plan.json`

### 10.2 Comprehensive Plan
See repository: `examples/comprehensive-plan.json`

### 10.3 Multi-Beneficiary Plan
See repository: `examples/multi-beneficiary-plan.json`

---

## 11. Interoperability

### 11.1 JSON-LD Context
Available at: `https://wiastandards.com/contexts/leg-006/v1.0`

### 11.2 API Endpoints
See Phase 2 specification (v1.1) for API details.

### 11.3 Smart Contract Integration
See Phase 3 specification (v1.2) for on-chain integration.

---

## 12. References

- W3C Decentralized Identifiers: https://www.w3.org/TR/did-core/
- ISO 8601 Date/Time: https://www.iso.org/iso-8601-date-and-time-format.html
- ISO 3166-2 Jurisdiction Codes: https://www.iso.org/iso-3166-country-codes.html
- BIP32 Hierarchical Deterministic Wallets: https://github.com/bitcoin/bips/blob/master/bip-0032.mediawiki
- ERC-721 Non-Fungible Token Standard: https://eips.ethereum.org/EIPS/eip-721

---

## Appendix A: JSON Schema

Complete JSON Schema available at:
`https://wiastandards.com/schemas/leg-006/v1.0/inheritance-plan.json`

---

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association
Licensed under MIT License


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
