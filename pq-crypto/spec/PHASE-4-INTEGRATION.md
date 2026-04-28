# WIA-pq-crypto PHASE 4 — Integration Specification

**Standard:** WIA-pq-crypto
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how a post-quantum-cryptography
(PQC) deployment integrates the data, APIs, and protocols
from PHASEs 1–3 with broader operational systems: PKI / CA
roster, HSM and key-management infrastructure, TLS / IKE /
SSH endpoints, code-signing pipelines, document signature
infrastructure, hardware-token (PIV/CAC/FIDO2) ecosystems,
and the cross-domain consumers that depend on cryptographic
state. It also specifies the migration-phase declarations
that other WIA standards reference to gate hybrid-mode
adoption.

References (CITATION-POLICY ALLOW only):
- NIST FIPS 203 — Module-Lattice-Based Key-Encapsulation Mechanism (ML-KEM)
- NIST FIPS 204 — Module-Lattice-Based Digital Signature Algorithm (ML-DSA)
- NIST FIPS 205 — Stateless Hash-Based Digital Signature Algorithm (SLH-DSA)
- NIST SP 800-208 — Recommendation for Stateful Hash-Based Signature Schemes (LMS / HSS / XMSS)
- NIST SP 800-227 (draft) — Recommendations for Key-Encapsulation Mechanisms
- NSA CNSA 2.0 — Commercial National Security Algorithm Suite 2.0 transition guidance
- BSI TR-02102-1 — Cryptographic Mechanisms recommendations
- ANSSI Avis sur la migration vers la cryptographie post-quantique
- IETF RFC 8446 (TLS 1.3), RFC 9180 (HPKE), RFC 8391 (XMSS), RFC 8554 (LMS / HSS)
- IETF Hybrid PQ KEX drafts (X25519+ML-KEM-768, secp256r1+ML-KEM-768, X-Wing)
- ISO/IEC 18033-2 — Encryption algorithms (asymmetric reference)
- ISO/IEC 14888-2/-3 — Digital signatures with appendix
- ISO/IEC 19790:2025 — Security requirements for cryptographic modules
- FIPS 140-3 — Security Requirements for Cryptographic Modules

---

## §1 PKI / CA roster integration

PQC migration touches every PKI in the deployment:

- **Internal CA**: deployment-operated; rolls to ML-DSA
  (default ML-DSA-65 for end-entity, ML-DSA-87 for root /
  intermediate)
- **External commercial CA**: tracked per CA's published PQC
  roadmap; the deployment maintains the CA-roster manifest
  with each CA's PQC support status
- **National PKI**: KR 행정전자서명·NPKI, EU eIDAS QSCD,
  US FPKI — tracked separately because each migrates on a
  national timeline

The boundary surfaces a per-endpoint PQC capability matrix:
which TLS endpoint supports hybrid (Phase B), which is
PQC-only (Phase C), which is still classical-only (Phase A).
Migration phase declarations (per PHASE 1 §6) are queryable
by operating standards (e.g., `WIA-medical-data-privacy`,
`WIA-payment-system`) so they can refuse classical-only
endpoints once their own phase target is reached.

## §2 HSM / KMS integration

PQC parameter sets are large compared to classical (ML-KEM-768
public key 1184 B, ciphertext 1088 B, ML-DSA-65 signature 3293 B,
SLH-DSA-128f signature 17088 B). HSM integration:

- HSM firmware version + supported PQC algorithm list pinned
  per device in the HSM-fleet manifest
- KMS partitions tagged with the PQC algorithm allow-list
  per partition (e.g., a partition restricted to ML-DSA only)
- Hybrid keys (classical + PQC) carry both sub-key references;
  the KMS surfaces both so callers can verify the hybrid
  binding integrity
- Key wrapping uses ML-KEM-768 + AES-256-Wrap (RFC 5649 KW)
  or, where supported, ML-KEM-1024 for higher-tier wrapping

The deployment maintains a per-HSM-vendor compatibility matrix
because PQC adoption varies (FIPS 140-3 PQC validation is
phased; some vendors ship pre-validation FIPS-140-3 boundary
parameters first).

## §3 TLS / IKE / SSH endpoint integration

TLS 1.3 hybrid handshakes adopt named groups from the IETF
Hybrid PQ KEX drafts:

- `X25519MLKEM768` — production-recommended hybrid (Phase B)
- `SecP256r1MLKEM768` — alternative hybrid for FIPS-only
  environments
- `X-Wing` — combiner with explicit security proof (per
  the IETF specification)

The boundary's TLS surface declares supported groups in the
capability document; the migration-phase declaration controls
which groups are acceptable to peers per WIA standard. IKEv2
(RFC 7296) with PQ extensions tracks the IETF
`pqc-ikev2` work; SSH (RFC 4253) hybrid kex tracks the
`sntrup761x25519-sha512` (and successor ML-KEM-based) drafts.

## §4 Code-signing pipeline integration

Signed-software supply chains migrate to PQC signatures:

- Build-time signing: ML-DSA-65 default; SLH-DSA reserved
  for high-assurance (long-lived) artifacts where statefulness
  isn't acceptable
- Stateful hash-based signing (LMS / HSS / XMSS per NIST
  SP 800-208 + RFC 8391 / RFC 8554): firmware roots-of-trust
  where lifecycle keys + tree-state management are tractable
- Verifier-side: code-signing verifiers maintain hybrid
  signing trust during migration (both classical + PQC
  signatures verified; either passing is acceptable per the
  deployment's hybrid policy)

## §5 Document-signature integration

Long-lived documents (notarial, archival, regulated records)
adopt PQC signatures with extra discipline:

- `validityWindow` declarations per signature so verifiers
  know whether to trust a classical-only signature (per
  WIA-pq-crypto PHASE 1 §6 phase target valid at signing
  time)
- Signature renewal: a document signed under classical-only
  in 2026 may be re-signed under hybrid or PQC-only in 2030
  to extend its trust horizon
- Long-term archival: prefer SLH-DSA (stateless) for
  documents archived for decades because LMS / HSS state
  loss invalidates the key

The renewal trail itself is signed and audit-chained so
verifiers see the full signature lineage.

## §6 Hardware-token integration

PIV / CAC / FIDO2 tokens migrate to PQC algorithms as the
respective specifications publish PQC profiles:

- PIV (NIST SP 800-73) — PQC profile drafting
- FIDO2 / WebAuthn — CTAP2.2 PQ extensions tracked
- KR 공동인증·금융인증 — KISA PQC profile timeline
  per regulator publication

Token capability is part of the deployment's identity
infrastructure (cross-reference to WIA-identity-management
PHASE 4 §3); a token's declared PQC algorithm set gates
its acceptance for high-assurance authentication flows.

## §7 Cross-domain phase declaration

Other WIA standards reference WIA-pq-crypto PHASE 1 §6
phase declarations:

- `WIA-medical-data-privacy` PHASE 3 §10 — clinical TLS
  cipher-suite floor
- `WIA-payment-system` PHASE 3 §8 — payment-rail TLS floor
- `WIA-mobile-payment` PHASE 3 §7 — wallet-attestation
  signature suite
- `WIA-network-security` PHASE 3 — sensor / endpoint TLS
  surfaces
- `WIA-military-satellite` PHASE 3 §8 — TT&C link
  cryptography
- `WIA-nft` PHASE 4 — bridge / wallet attestation
- `WIA-medical-robot` PHASE 3 — surgical-robot tele-link
  encryption

Each consumer publishes its phase-target declaration in
its capability document; the migration-coordination service
(this PHASE §9) aggregates these declarations to surface
deployment-wide migration status.

## §8 Algorithm-rollout governance

Adopting a new PQC algorithm into production follows:

1. **Pilot lane**: enabled on non-critical endpoints with
   detailed telemetry (handshake success rate, latency,
   parameter-size impact on packet fragmentation)
2. **Expanded pilot**: progressively wider lanes as
   telemetry validates correctness; per-region rollouts
   to absorb edge-network MTU sensitivity
3. **General availability**: full migration target with
   classical fallback retained per phase target
4. **Deprecation**: classical-only acceptance retired per
   the deployment's published deprecation timeline; the
   audit chain captures every retired algorithm

Each rollout stage emits an audit-chain entry with the
algorithm identifier, scope (endpoints / regions affected),
and the operating authority's signature.

## §9 Migration-coordination service

The deployment runs a migration-coordination service that:

- Aggregates per-domain phase declarations
- Surfaces gap analysis (which domains lag the deployment-wide
  target)
- Schedules cross-domain handover windows (e.g., when a
  domain's downstream consumer can't yet handle PQC, the
  coordinator schedules a planned downgrade window)
- Publishes the deployment-wide migration dashboard

Cross-coalition coordination (where the deployment serves
partners) follows a documented exchange schedule so the
ecosystem migrates in coordinated waves rather than
fragmenting into incompatible islands.

## §10 Operational SLAs

| Concern                                | Default SLA                |
|----------------------------------------|----------------------------|
| Phase-declaration query                | ≤ 100 ms p95               |
| HSM PQC operation (ML-DSA-65 sign)     | per HSM vendor spec        |
| TLS hybrid handshake overhead          | < 30% over classical       |
| Algorithm-roster refresh               | daily                      |
| CA-roster PQC-status refresh           | hourly                     |
| Audit-chain entry availability         | ≤ 10 s                     |
| Migration-dashboard refresh            | hourly                     |

## §11 Quarterly compliance report

The boundary emits a quarterly compliance report:

- Endpoint inventory by phase (A / B / C)
- Algorithm distribution across endpoints
- Hybrid-handshake success vs. classical-only fallback rates
- HSM PQC validation status
- CA-roster PQC-readiness summary
- Pilot-lane telemetry summary
- Cross-domain gap analysis
- Audit-chain integrity check results

## §12 Acceptance criteria

A deployment claims conformance when:

1. Every cryptographic endpoint declares a phase per PHASE 1 §6
2. Every HSM in service is in the fleet manifest with current
   PQC algorithm support
3. Hybrid TLS is operational on at least the high-priority
   endpoints (per the deployment's prioritisation)
4. Migration-phase declarations are referenced and honoured by
   downstream WIA-domain consumers
5. Audit-chain entries match the deployment's algorithm-rollout
   timeline for the prior quarter
6. Quarterly compliance report has no integrity-check failures

## §13 Common pitfalls (informative)

- **Parameter-size MTU fragmentation** — PQC handshakes
  exceed 1500-byte MTU for many parameter combinations;
  deployments SHOULD test on real-world network paths,
  including legacy middleboxes that fragment poorly
- **HSM firmware lag** — vendors ship PQC support behind
  validation timelines; deployments SHOULD plan for
  vendor-staged rollout, not a single cutover
- **CA migration desync** — different CAs migrate on
  different schedules; mixed PQC / classical chains common
  during transition
- **Stateful HBS state-loss** — LMS / HSS / XMSS state must
  not be lost or replayed across HSM partitions; deployment
  policy SHOULD prefer SLH-DSA where state management is
  hard
- **Hybrid binding misuse** — naive concatenation of
  classical + PQC keys is insecure; combiners must follow
  the IETF Hybrid PQ KEX draft's binding rules (or X-Wing's
  explicit construction)
- **Algorithm-identifier confusion** — Kyber / Dilithium are
  pre-FIPS names; FIPS-published names are ML-KEM / ML-DSA
  with different parameter selections. Deployments MUST
  reference the FIPS identifiers in normative records

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table

| Reference                  | Use site                                                     |
|----------------------------|--------------------------------------------------------------|
| WIA-network-security       | sensor / endpoint TLS cipher-suite floor                     |
| WIA-identity-management    | hardware-token PQC profile attestation                       |
| WIA-payment-system         | payment-rail TLS migration phase                             |
| WIA-mobile-payment         | wallet-attestation signature suite                           |
| WIA-medical-data-privacy   | clinical-TLS hybrid floor                                    |
| WIA-military-satellite     | TT&C link cryptography                                       |
| WIA-supply-chain           | code-signing pipeline                                        |

## Annex B — Decommissioning checklist (informative)

When a deployment retires a classical-only endpoint:

- [ ] Phase target met for the endpoint's domain
- [ ] Downstream consumers acknowledge readiness
- [ ] Audit-chain entry recording retirement signed
- [ ] CA / KMS / HSM partitions reconfigured
- [ ] Capability document published with updated surface
- [ ] Migration dashboard reflects retirement

## Annex C — Conformance disclosure

Sections §1, §2, §3, §7, §10, §11, §12 are mandatory.
§4 (code-signing) is mandatory for deployments running
software-supply-chain pipelines. §5 (document-signature) is
mandatory for archival / notarial deployments. §6 (hardware
token) is mandatory for deployments accepting hardware-token
authentication.

## Annex D — Worked migration calendar (informative)

A worked migration calendar for a deployment with
moderate cryptographic surface:

- 2026 Q1–Q2: HSM firmware audit + hybrid TLS pilot on
  internal endpoints
- 2026 Q3: hybrid TLS general availability for internal
  east-west traffic
- 2026 Q4: hybrid TLS pilot for external customer-facing
  TLS (limit to allow-listed customers)
- 2027 Q1: code-signing pipeline migration (ML-DSA-65 default)
- 2027 Q2–Q3: external customer-facing hybrid TLS GA
- 2028: classical-only deprecation begins on internal
  endpoints
- 2030+: hybrid → PQC-only migration tracked by deployment
  policy

The actual calendar varies per deployment; the audit chain
captures the deployment's actual progression for compliance
purposes.

## Annex E — Algorithm allow-list anchoring

The deployment's allow-list of permitted PQC algorithms is
anchored to the deployment's signing key. Allow-list mutations
(adding / retiring algorithms) require a counter-signature
from the deployment's cryptographic-policy authority and emit
an audit-chain entry.

The allow-list itself references upstream guidance (NIST
FIPS 203/204/205, CNSA 2.0, BSI TR-02102-1, ANSSI guidance);
mutations document which upstream change motivated them.
