# WIA-LEG-006: Digital Asset Inheritance Standard
## Version 1.2 - Phase 3: Smart Contract Protocol

**Status:** Final
**Published:** 2025-03-01
**Authors:** WIA Technical Committee on Digital Legacy
**License:** MIT License
**Requires:** v1.0 (Phase 1), v1.1 (Phase 2)

---

## 1. Introduction

### 1.1 Purpose
Phase 3 specifies on-chain smart contract protocols for automated, trustless digital asset inheritance execution.

### 1.2 Scope
- Multi-signature wallet schemes
- Dead man's switch implementations
- Oracle integration protocols
- Cross-chain coordination
- Smart contract security requirements

---

## 2. Multi-Signature Inheritance Contracts

### 2.1 Standard 2-of-3 Multi-Sig
```solidity
interface IInheritanceMultiSig {
    function transferInheritance(
        address beneficiary,
        uint256 amount
    ) external requiresTwoSignatures;

    function checkIn() external onlyOwner;

    function getInactivityPeriod() external view returns (uint256);
}
```

### 2.2 Configuration Parameters
- `totalSigners`: 2-10 (recommended: 3)
- `requiredSignatures`: 1 to totalSigners (recommended: 2)
- `timeLockPeriod`: 0-30 days (recommended: 14)

### 2.3 Security Requirements
- Formal verification of multi-sig logic
- Third-party audit by certified auditor
- Time-delay between approval and execution
- Emergency pause mechanism

---

## 3. Dead Man's Switch Protocol

### 3.1 Heartbeat Contract Interface
```solidity
interface IDeadManSwitch {
    function checkIn(bytes memory biometricProof) external;
    function updateState() external;
    function executeInheritance() external;

    function lastCheckIn() external view returns (uint256);
    function currentState() external view returns (State);

    enum State { Active, Warning, Grace, Triggered }
}
```

### 3.2 State Transitions
- **Active → Warning:** `timeSinceCheckIn > (checkInInterval - warningPeriod)`
- **Warning → Grace:** `timeSinceCheckIn > checkInInterval`
- **Grace → Triggered:** `timeSinceCheckIn > (checkInInterval + gracePeriod)`
- **Any → Active:** Owner successful check-in

### 3.3 Check-In Requirements
- Biometric verification (optional but recommended)
- Location verification (geofencing)
- Device fingerprinting
- Multi-factor authentication

### 3.4 Notification System
- On-chain events for each state transition
- Off-chain notification service integration
- Progressive urgency levels
- Multiple notification channels

---

## 4. Oracle Integration

### 4.1 Death Certificate Oracle Interface
```solidity
interface IDeathCertificateOracle {
    function reportDeath(
        bytes32 ssnHash,
        uint256 timestamp,
        bytes memory oracleSignature,
        bytes32[] memory proof
    ) external;

    function verifyDeath(bytes32 ssnHash)
        external view returns (bool verified, uint256 timestamp);
}
```

### 4.2 Oracle Requirements
- Minimum 3 independent oracle confirmations
- Oracle identity verification via WIA-IDENTITY
- Merkle proof for data authenticity
- Time window for confirmations (max 30 days)

### 4.3 Supported Oracle Types
- Government vital statistics offices
- Medical examiner systems
- Probate court systems
- Licensed death certificate verification services

### 4.4 Oracle Security
- Oracle reputation system
- Slashing for false reports
- Dispute resolution mechanism
- Appeal period (30-90 days)

---

## 5. Conditional Distribution Logic

### 5.1 Age-Based Distribution
```solidity
interface IAgeBasedInheritance {
    struct DistributionSchedule {
        uint256 ageRequired;
        uint256 percentage;
    }

    function claimInheritance(address beneficiary) external;
    function getClaimableAmount(address beneficiary)
        external view returns (uint256);
}
```

### 5.2 Milestone-Based Distribution
- Educational achievement verification
- Professional certification confirmation
- Custom condition evaluation
- Verifiable credential integration

---

## 6. Cross-Chain Coordination

### 6.1 Unified Trigger Protocol
```
1. Oracle Network broadcasts trigger event
2. Each blockchain's contract receives trigger
3. Contracts enter "pending" state
4. Coordination period (24 hours)
5. Synchronized execution across chains
```

### 6.2 Cross-Chain Message Format
```json
{
  "planId": "UUID",
  "triggerType": "oracle|manual|deadman",
  "triggerTimestamp": "Unix timestamp",
  "triggerHash": "SHA-256 hash",
  "sourceChain": "chainId",
  "signatures": []
}
```

### 6.3 Supported Bridge Protocols
- LayerZero for cross-chain messaging
- Chainlink CCIP for secure cross-chain transfers
- Axelar for cross-chain communication
- Wormhole for multi-chain coordination

---

## 7. Smart Contract Security Standards

### 7.1 Mandatory Security Measures
- Reentrancy guards on all external calls
- Integer overflow/underflow protection
- Access control on sensitive functions
- Circuit breakers for emergency situations
- Time-locks on critical operations

### 7.2 Formal Verification Requirements
- Theorem proving for core logic
- Model checking for state transitions
- Symbolic execution for all paths
- Property-based testing

### 7.3 Audit Requirements
- Minimum 2 independent audits
- Auditors from WIA-approved list
- Public audit reports required
- Bug bounty program (min $50k pool)

### 7.4 Testing Requirements
- 100% code coverage
- Fuzzing tests (min 100k iterations)
- Mainnet fork testing
- Gas optimization analysis

---

## 8. Gas Optimization

### 8.1 Storage Optimization
- Pack related variables into single slots
- Use events for historical data
- Minimize SSTORE operations
- Use calldata instead of memory where possible

### 8.2 Computation Optimization
- Off-chain computation with on-chain verification
- Batch operations where feasible
- Early returns to save gas
- View functions for read operations

### 8.3 Layer 2 Deployment
- Primary deployment on L2 (Arbitrum, Optimism, Polygon)
- L1 deployment for high-value estates
- Cross-layer communication support

---

## 9. Emergency Procedures

### 9.1 Contract Pause
```solidity
function emergencyPause() external onlyEmergencyMultiSig {
    require(!paused, "Already paused");
    paused = true;
    emit EmergencyPause(block.timestamp);
}
```

### 9.2 Emergency Withdrawal
- Requires M-of-N emergency multisig
- 7-day time-lock after pause
- All beneficiaries notified
- Funds sent to secure backup address

### 9.3 Contract Upgrade
- Proxy pattern for upgradeability
- Time-locked upgrades (min 14 days)
- Beneficiary approval for major upgrades
- Immutable core inheritance logic

---

## 10. Blockchain-Specific Implementations

### 10.1 Ethereum (EVM)
- Solidity 0.8.20+
- ERC-20/ERC-721 integration
- ENS domain support
- Gas optimization for Layer 1

### 10.2 Bitcoin
- Multi-sig P2WSH addresses
- Time-locked transactions (nLockTime)
- Taproot-based complex scripts
- Lightning Network inheritance

### 10.3 Solana
- Rust-based programs
- SPL token support
- Program Derived Addresses (PDAs)
- Low-latency execution

### 10.4 Cardano
- Plutus smart contracts
- Native token support
- eUTXO model considerations
- Formal verification friendly

---

## 11. Integration with Phase 2 APIs

### 11.1 Contract Deployment
- `POST /contracts/deploy` endpoint
- Returns contract address and transaction hash
- Automatic verification on block explorers

### 11.2 Contract Interaction
- `POST /contracts/{address}/call` for function calls
- Gas estimation before execution
- Transaction status tracking

### 11.3 Event Monitoring
- Webhook notifications for contract events
- Real-time event streaming
- Historical event queries

---

## 12. Reference Implementations

### 12.1 GitHub Repository
`https://github.com/WIA-Official/inheritance-contracts`

### 12.2 Deployed Contracts
- Ethereum Mainnet: `0x...`
- Polygon: `0x...`
- Arbitrum: `0x...`
- Optimism: `0x...`

### 12.3 Contract Verification
All contracts verified on Etherscan, PolygonScan, Arbiscan

---

## 13. References

- Ethereum Smart Contracts: https://ethereum.org/en/developers/docs/smart-contracts/
- Solidity Documentation: https://docs.soliditylang.org/
- OpenZeppelin Contracts: https://docs.openzeppelin.com/contracts/
- ConsenSys Smart Contract Best Practices: https://consensys.github.io/smart-contract-best-practices/

---

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association
Licensed under MIT License


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
