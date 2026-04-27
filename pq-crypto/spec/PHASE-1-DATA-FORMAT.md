# WIA-pq-crypto PHASE 1 — Data Format Specification

**Standard:** WIA-pq-crypto
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for post-
quantum cryptography lifecycle operations: PQC algorithm
registry records, key-pair registry records, key-rotation
records, hybrid-mode binding records, migration-phase
declarations, conformance-evidence records, and the cross-
references binding cryptographic state to the systems that
depend on it. The shape interoperates with NIST PQC
standards (FIPS 203 ML-KEM, FIPS 204 ML-DSA, FIPS 205
SLH-DSA), CNSA 2.0 transition guidance, and IETF protocols
for hybrid handshakes.

References (CITATION-POLICY ALLOW only):
- NIST FIPS 203 — Module-Lattice-Based Key-Encapsulation Mechanism (ML-KEM)
- NIST FIPS 204 — Module-Lattice-Based Digital Signature Algorithm (ML-DSA)
- NIST FIPS 205 — Stateless Hash-Based Digital Signature Algorithm (SLH-DSA)
- IETF RFC 8446 (TLS 1.3), RFC 8410 (Ed25519/Ed448),
  RFC 8032 (EdDSA), RFC 9180 (HPKE)
- IETF draft-ietf-tls-hybrid-design — TLS 1.3 hybrid key exchange
- IETF RFC 9258 (PQ in CMS), RFC 9162 (Certificate Transparency 2.0)
- ISO/IEC 18033-2 — Encryption algorithms reference
- NSA Commercial National Security Algorithm Suite 2.0 (CNSA 2.0)
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 7517 (JWK)
- ISO/IEC 27001:2022 — information security management

---

## §1 Scope

This PHASE applies to the data shape used by deployments
operating cryptographic infrastructure undergoing or having
completed migration to post-quantum primitives. It addresses
the algorithm registry, key-pair lifecycle, hybrid-mode
binding, migration-phase declaration, and conformance
evidence; transport protocols and detailed key-exchange
mechanics are in PHASE 3 and integration with broader
identity, transport, and signing systems is in PHASE 4.

The standard is migration-aware: every operation carries
the migration phase under which it executed (Classical,
Hybrid, PQ-only) so cross-deployment interoperability and
audit replay across the migration are well-defined. A
deployment is permitted to advance phases per its own
roadmap; rollback is recorded as a first-class event.

In scope: algorithm registry, key-pair registry, key-rotation
record, hybrid-mode binding, migration-phase declaration,
algorithm-deprecation record, conformance-evidence record,
attestation record. Out of scope: cryptographic primitive
implementation details (carried as references to authoritative
specs), HSM-vendor proprietary control protocols (carried
opaquely as vendor extensions), and end-application protocol
choices (the deployment may elect any PQ-secure protocol it
declares in its capability document).

## §2 Algorithm registry record

The deployment's algorithm registry catalogues the
algorithms it accepts and uses:

| Field             | Source / Binding                                       |
|-------------------|--------------------------------------------------------|
| `algorithmRef`    | URN of form `urn:wia:pqc:alg:<id>`                     |
| `algorithmName`   | canonical name (e.g., `ML-KEM-768`, `ML-DSA-65`,       |
|                   | `SLH-DSA-SHA2-128s`, `Ed25519`, `RSA-3072`,            |
|                   | `ECDSA-P-256`)                                         |
| `algorithmClass`  | `kem`, `signature`, `hash`, `mac`, `kdf`, `aead`,      |
|                   | `block-cipher`                                         |
| `nistCategory`    | NIST PQC security category (1, 2, 3, 5)               |
| `referenceSpec`   | URN of authoritative spec (FIPS 203/204/205, RFC, etc.)|
| `vendorImpls[]`   | URNs of validated implementations                      |
| `deprecationDate` | RFC 3339 (or null if not deprecated)                  |
| `acceptedFor[]`   | role classes that may use this algorithm              |

Registry mutations are signed and audit-chained. An algorithm
moved to deprecated state still remains usable for verifying
historical artifacts but is refused for new operations.

## §3 Key-pair registry record

Every active key pair is tracked:

- `keypairRef` — URN of form `urn:wia:pqc:keypair:<authority>:<id>`
- `algorithmRef` — URN of the algorithm
- `holderRef` — URN of the holding entity (operator, system,
  identity)
- `purpose` — `tls-server`, `tls-client`, `code-signing`,
  `document-signing`, `key-encapsulation`, `audit-signing`,
  `attestation`, `iot-device-attestation`
- `publicKeyRef` — URI of the public key (JWK, X.509 cert, or
  HPKE public key)
- `keyManagementRef` — URN of the HSM or key-management
  system holding the private key
- `createdAt` — RFC 3339
- `validNotBefore`, `validNotAfter` — RFC 3339
- `migrationPhase` — `classical`, `hybrid`, `pq-only`
- `successorRef` — URN of the successor key pair after
  rotation (null until rotation)

Key pairs without `keyManagementRef` (i.e., keys held
outside an audited key-management system) are refused at
boundary intake.

## §4 Key-rotation record

Each rotation event is recorded:

- `rotationId` — URN
- `predecessorRef` — URN of the prior key pair
- `successorRef` — URN of the new key pair
- `rotatedAt` — RFC 3339
- `rotationReason` — `scheduled`, `compromise`, `algorithm-
  deprecation`, `migration-phase-advance`, `key-management-
  migration`
- `overlapWindowStart`, `overlapWindowEnd` — period during
  which both keys remain valid for verification
- `attestationRefs[]` — URNs of attestations from the key-
  management system confirming the rotation
- `signatures[]` — signing-authority signatures

Rotations whose `overlapWindow` is shorter than the
deployment-declared minimum are flagged as urgent (e.g.,
compromise rotations); the audit chain records the urgency
class.

## §5 Hybrid-mode binding record

For migration-phase `hybrid`, the binding between classical
and PQ algorithms is recorded:

- `bindingId` — URN
- `classicalAlgorithmRef` — URN
- `pqAlgorithmRef` — URN
- `bindingMode` — `concatenate` (concatenation of secrets),
  `kdf-combine` (KDF over both secrets), `dual-signature`
  (independent signatures verified together)
- `protocolRef` — URN of the protocol the binding applies to
  (e.g., TLS 1.3 hybrid handshake, hybrid CMS signature)
- `validFromPhase` — migration phase at which this binding
  becomes active
- `validToPhase` — migration phase at which this binding
  is superseded

Hybrid bindings are versioned; partner-facing bindings are
mirrored to partners' capability documents through the
exchange protocol in PHASE 3.

## §6 Migration-phase declaration record

The deployment's overall migration-phase posture:

- `phaseDeclarationId` — URN
- `currentPhase` — `classical`, `hybrid`, `pq-only`
- `phaseEntryAt` — RFC 3339
- `targetNextPhase` — closed enum or null
- `targetNextPhaseAt` — declared target date or null
- `algorithmRosterRef` — URN of the algorithm-registry
  snapshot active at this declaration
- `exceptionsRef[]` — URNs of role classes operating under
  documented exception (e.g., legacy IoT devices on classical)
- `responsibleOfficerRef` — URN of the deployment's
  cryptographic-officer

Phase advances and rollbacks are first-class events that
update this record and emit cross-references to all keys
operating under the new phase.

## §7 Algorithm-deprecation record

Per algorithm undergoing deprecation:

- `deprecationId` — URN
- `algorithmRef` — URN
- `deprecationAnnouncedAt` — RFC 3339
- `acceptanceCutoffAt` — RFC 3339 — boundary refuses new
  operations using this algorithm beyond this date
- `verificationCutoffAt` — RFC 3339 — boundary refuses
  verification of historical artifacts beyond this date
- `replacementAlgorithmRefs[]` — URNs of the recommended
  replacements
- `migrationGuidanceRef` — URI of deployment-internal
  migration guidance

A deprecation announcement is mandatory at least
deployment-declared notice ahead of the acceptance cutoff;
shorter notices are exception-classed and require
cryptographic-officer counter-signature.

## §8 Conformance-evidence record

Conformance evidence accumulates per the deployment's
declared cadence:

- `evidenceId` — URN
- `period` — declared evidence period (e.g., `2026-Q2`)
- `phaseDeclarationRef` — URN
- `keypairCensusRef` — URN of the keypair-census artifact
  for the period
- `rotationLogRef` — URN of rotation events in the period
- `deprecationLogRef` — URN of deprecation events in the
  period
- `attestationsRef[]` — HSM attestations in the period
- `auditSignatureRef` — auditor signature URN

Conformance-evidence records are versioned and referenced by
partners and regulators.

## §9 HSM attestation record

Attestations from the key-management system:

- `attestationId` — URN
- `kmRef` — URN of the key-management system
- `attestationKindRef` — declared attestation type (HSM
  health, key creation, key rotation, key destruction)
- `attestationPayloadRef` — URI of the signed attestation
  payload
- `attestationSignature` — signature over the payload
- `at` — RFC 3339

Attestations bind key-pair lifecycle events to evidence that
the private key was generated and held inside the audited
KM system.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                     | Use site                                                 |
|-------------------------------|----------------------------------------------------------|
| WIA-network-security          | TLS cipher-suite floor governance                        |
| WIA-supply-chain              | code-signing key registry                                 |
| WIA-military-satellite        | TT&C cipher-suite migration                               |

## Annex B — Conformance disclosure

Sections §2, §3, §4, §6, §7, §8 are mandatory. §5 (Hybrid)
is mandatory for any deployment in `hybrid` migration phase;
§9 (Attestation) is mandatory for any deployment using HSM-
based key management (effectively all production deployments).

## Annex C — Versioning and deprecation

Versioning follows SemVer 2.0.0. Algorithm-deprecation events
respect the cutoff dates declared in §7; emergency
deprecations (CVE-driven) follow the deployment's incident-
response cycle.

## Annex D — Worked algorithm registry entry (informative)

```json
{
  "algorithmRef": "urn:wia:pqc:alg:ml-kem-768",
  "algorithmName": "ML-KEM-768",
  "algorithmClass": "kem",
  "nistCategory": 3,
  "referenceSpec": "urn:wia:spec:nist-fips-203",
  "vendorImpls": ["urn:wia:vendor-impl:vendor-a-mlkem-1.2"],
  "deprecationDate": null,
  "acceptedFor": ["tls-server","tls-client","kem-hybrid"]
}
```

## Annex E — Vendor extensions

HSM and KM vendors may extend records with `x-vendor-*`
fields. Extensions MUST NOT contradict canonical fields and
MUST NOT be required for core conformance.

## Annex F — Time discipline cross-reference

Time fields use the discipline of PHASE 3 §6. Records that
fail clock discipline are tagged `provisional` until
backfilled.

## Annex G — Conformance level

Implementations declare conformance level (Surface / Verified
/ Anchored). Anchored requires a continuous evidence package
plus an annual cryptographic-audit covering the integration
contracts in PHASE 4.

## Annex H — Worked migration-phase declaration

```json
{
  "phaseDeclarationId": "urn:wia:pqc:phase:authority-x:p-2026-q2",
  "currentPhase": "hybrid",
  "phaseEntryAt": "2026-04-01T00:00:00+09:00",
  "targetNextPhase": "pq-only",
  "targetNextPhaseAt": "2027-04-01T00:00:00+09:00",
  "algorithmRosterRef": "urn:wia:pqc:roster:authority-x:r-2026-q2",
  "exceptionsRef": ["urn:wia:pqc:exception:authority-x:legacy-iot"],
  "responsibleOfficerRef": "urn:wia:auth:authority-x-crypto-officer"
}
```

## Annex I — Algorithm-acceptance matrix

The deployment publishes a matrix declaring which algorithms
are accepted for which roles in which migration phase. The
matrix is the canonical reference for partners verifying
cipher-suite negotiation; mismatches are surfaced as
capability-document violations during exchange.
