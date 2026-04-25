# WIA-AI-017 Specification: PHASE 4
## Future Extensions and Research

**Version:** 1.0
**Status:** Experimental
**Date:** 2025-12-25
**Philosophy:** 弘益人間 (홍익인간) · Benefit All Humanity

---

## 1. Post-Quantum Cryptography

### 1.1 Quantum-Resistant Signatures

Future support for:
- **CRYSTALS-Dilithium** (NIST PQC standard)
- FALCON (compact signatures)
- SPHINCS+ (stateless hash-based)

### 1.2 Hybrid Signatures

Combine classical and post-quantum:
```
HybridSignature ::= SEQUENCE {
    classicalSig Ed25519Signature,
    pqSig DilithiumSignature
}
```

---

## 2. Zero-Knowledge Proofs

### 2.1 Selective Disclosure

Prove content properties without revealing content:
- "Created by verified entity X"
- "Generated before time T"
- "Not modified since creation"

### 2.2 zk-SNARKs for Provenance

Prove valid provenance chain without revealing full history.

---

## 3. Federated Authentication

### 3.1 Cross-Platform Verification

Standardized protocol for verifying content across platforms without central authority.

### 3.2 Decentralized Identity (DID)

Integration with W3C Decentralized Identifiers for creator attribution.

---

## 4. Hardware-Backed Authentication

### 4.1 Trusted Execution Environments

Use TEE (e.g., Intel SGX, ARM TrustZone) for:
- Secure key storage
- Attestation of signing environment
- Tamper-proof authentication

### 4.2 Camera-Native Signing

Sign content at capture time in camera hardware:
```
CameraSignature ::= SEQUENCE {
    sensorId OCTET STRING,
    captureTime GeneralizedTime,
    rawHash OCTET STRING,
    signature OCTET STRING
}
```

---

## 5. AI-Assisted Authentication

### 5.1 Adaptive Detection Models

Models that:
- Continuously learn new deepfake techniques
- Adapt to emerging GAN architectures
- Self-update with federated learning

### 5.2 Automated Forensics

AI systems that automatically:
- Identify tampering
- Suggest verification strategies
- Generate explanation reports

---

## 6. Standardization Roadmap

### 6.1 ISO/IEC Submission

Path to international standardization:
- Submit to ISO/IEC JTC 1/SC 27 (IT Security)
- Coordinate with C2PA steering committee
- Engage with ITU-T Study Group 16

### 6.2 Industry Adoption

Target adoption by:
- Social media platforms (2026)
- News organizations (2026)
- Camera manufacturers (2027)
- Operating systems (2028)

---

## 7. Research Directions

### 7.1 Open Problems

- Real-time video stream authentication
- Preserving watermarks across format conversions
- Balancing privacy and transparency
- Detecting subtle AI enhancements

### 7.2 Call for Contributions

Community invited to contribute to:
- Detection model improvements
- New watermarking techniques
- Performance optimizations
- Use case studies

---

## Long-Horizon Engineering Considerations

### Threat Model Evolution

The Phase 4 work programme accepts that the dominant threats to AI-content authentication evolve faster than the standards process. The threat model is therefore revised on an annual cadence, with each revision documenting:

1. New synthesis modalities reported in the public technical literature.
2. Demonstrated attacks against the Phase 1–3 protocols.
3. Defensive measures incorporated in the next revision.

Revisions follow the editorial discipline of ISO/IEC JTC 1 directives, with public comment periods of at least 60 days for substantive changes.

### Cryptographic Agility

A core design assumption is that any single signature algorithm in §6.1 (Phase 1) may be deprecated within the standard's lifetime. Implementations therefore MUST support algorithm-agile manifests: the algorithm identifier travels with each signature, and verifiers MUST refuse to silently substitute algorithms.

When a new family is introduced (post-quantum or otherwise), the migration playbook prescribes:

1. **Issue dual-signed manifests** containing both legacy and new signatures.
2. **Verifier deployment phase** — verifiers learn to validate the new family but continue to accept legacy signatures.
3. **Producer transition phase** — producers progressively switch to single-signed new-family manifests.
4. **Legacy retirement phase** — legacy verification is removed after coordinated deprecation.

### Hardware Roots of Trust

Camera-native signing per §4.2 is governed by:

- **TCG TPM 2.0 Library Specification (Parts 1–4)** for trusted platform modules in supporting hardware.
- **GlobalPlatform TEE Internal Core API Specification v1.3** for trusted execution on mobile platforms.
- **IETF RATS Architecture (RFC 9334)** and Entity Attestation Token (EAT) for attestation flows.

Signed-at-capture content carries an attestation evidence blob that the verifier can present to a relying party for independent evaluation.

### Federated Operation

When a content asset moves across organisational boundaries (creator → distributor → platform → consumer), the chain of custody is preserved by reference rather than by re-signing. Each handoff appends a counter-signed assertion to the manifest envelope without modifying upstream assertions, preserving the cryptographic continuity from origin.

## 8. Reference Standards Alignment

### 8.1 Post-Quantum Cryptography

The post-quantum signature families referenced in §1 align with the NIST Post-Quantum Cryptography (PQC) Standardization Project outputs:

- **FIPS 204** — Module-Lattice-Based Digital Signature Standard (ML-DSA, derived from CRYSTALS-Dilithium).
- **FIPS 205** — Stateless Hash-Based Digital Signature Standard (SLH-DSA, derived from SPHINCS+).
- **NIST IR 8413** — Status report on the third round of the NIST PQC standardization process (FALCON track).

Hybrid construction guidance follows the IETF draft `hybrid-signature-spec` and the related ISO/IEC SC 27 work programme on post-quantum key management.

### 8.2 Trusted Execution and Attestation

Hardware-backed signing in §4 maps to:

- **TCG TPM 2.0 Library Specification** for trusted platform modules.
- **IETF Remote Attestation Procedures (RATS) Architecture (RFC 9334)** for attestation.
- **Confidential Computing Consortium** specifications for TEE-attested workloads.

### 8.3 Decentralised Identity

Federated and decentralised verification in §3 uses:

- **W3C Decentralized Identifiers (DIDs) v1.0** — Recommendation.
- **W3C Verifiable Credentials Data Model 2.0** — Recommendation.
- **W3C JSON-LD 1.1** — Recommendation, for credential payload encoding.

### 8.4 Standards Liaison

The §6 standardisation roadmap targets:

- **ISO/IEC JTC 1/SC 27** — IT Security techniques (publication track for content provenance).
- **ISO/IEC JTC 1/SC 29** — Coding of audio, picture, multimedia information (alignment with V-PCC and image coding).
- **ITU-T Study Group 16** — Multimedia, with focus on media authenticity standards.
- **C2PA Steering Committee** — open coordination for content credentials.

### 8.5 Research Conduct

Research contributions to Phase 4 follow the principles of the ACM Code of Ethics and Professional Conduct and the IEEE Code of Ethics, with reproducibility expectations aligned with the FAIR Guiding Principles for scientific data management as adopted by the Research Data Alliance.

## 9. Long-Horizon Appendix

### 9.1 Deprecation discipline

Phase 4 explicitly accepts that some §1–§5 mechanisms will be deprecated within the standard's lifetime. The deprecation discipline is:

1. **Announce** the candidate deprecation in the public registry with a deprecation rationale.
2. **Mark** the deprecated mechanism in subsequent manifests with the `wia.deprecated.v1` flag carrying the announced sunset date.
3. **Sunset** the deprecated mechanism after the announced window has fully elapsed and the migration playbook has been completed.
4. **Archive** the deprecated mechanism's specification, test vectors, and reference implementation in the historical registry.

This discipline gives implementers predictable migration windows and prevents silent removal of mechanisms still in field use.

### 9.2 Conformance forward compatibility

Conformance with Phase 1–3 is forward-compatible when:

- Manifests carry the `wia.spec_version.v1` claim with the major and minor version of the specification under which they were produced.
- Verifiers gracefully ignore unknown labels rather than rejecting the manifest, except where the unknown label is marked `must-understand`.
- Producers do not introduce `must-understand` labels without coordination through the WIA registry.

This three-rule contract preserves interoperability across version boundaries without requiring synchronous upgrade across producers and verifiers.

### 9.3 Public benchmarking

The Phase 4 work programme commits to publishing an annual benchmark report covering all conformant implementations that opt in. The report includes detection accuracy on the reference suite, robustness scores, throughput and latency at the published workload profiles, and post-quantum readiness status.

The benchmark suite itself is governed by an open editorial process, with new test vectors added on a quarterly cadence and proposed by any participant in the WIA standards community.

### 9.4 Stewardship

Stewardship of WIA-AI-017 is exercised by the WIA standards committee, which maintains the public specification, registry, and benchmark suite. The committee operates under transparency rules requiring public meeting minutes, public issue tracking, and public archives of all substantive technical correspondence. Decisions affecting backward compatibility require a 60-day public comment period and committee supermajority.

### 9.5 Closing

The Phase 4 horizon is explicitly long. The reason for that horizon — and for the agility, deprecation, and stewardship disciplines that come with it — is that authentic content matters far beyond any single technology cycle. The standard is engineered to outlast the specific cryptographic, machine-learning, and platform choices made today, while remaining useful and verifiable across the full duration of that horizon.

## 10. Liaison Map

### 10.1 ISO/IEC liaison

WIA-AI-017 is positioned for liaison with the following ISO/IEC committees:

- **ISO/IEC JTC 1/SC 27** (Information security, cybersecurity and privacy protection) — primary publication track for cryptographic and identity components.
- **ISO/IEC JTC 1/SC 29** (Coding of audio, picture, multimedia, and hypermedia information) — alignment with image/video coding standards used by the authenticated content payloads.
- **ISO/IEC JTC 1/SC 38** (Cloud computing and distributed platforms) — alignment for federated and cloud-deployed services.
- **ISO/IEC JTC 1/SC 42** (Artificial intelligence) — alignment for detection-model evaluation methodology and data-quality requirements.
- **ISO/TC 46** (Information and documentation) — alignment for persistent identifier and provenance vocabulary.

### 10.2 IETF liaison

The relevant IETF working groups are:

- **JOSE** — Javascript Object Signing and Encryption.
- **COSE** — CBOR Object Signing and Encryption.
- **CBOR** — Concise Binary Object Representation.
- **OAUTH** — for federated identity and token issuance.
- **RATS** — Remote Attestation Procedures, for hardware-backed signing.
- **ACME** — for automated certificate issuance in federated deployments.

### 10.3 W3C liaison

The relevant W3C activities are:

- **Verifiable Credentials Working Group**.
- **Decentralized Identifier Working Group**.
- **Web Authentication Working Group** (for client-side cryptographic operations).
- **Provenance Working Group** (PROV-O).
- **Web Application Security Working Group** (for browser-side verification UX).

### 10.4 Industry-led liaisons

Industry coordination paths include the C2PA, the AOMedia Software Alliance for codec coordination, the Khronos Group for glTF and WebGL alignment, and the Cloud Native Computing Foundation for OpenTelemetry alignment.

### 10.5 Public engagement

Public technical correspondence occurs on the WIA standards mailing list and in the public issue tracker. Substantive proposals require a written specification, a reference implementation, and a test plan. The public archive is preserved indefinitely as part of the stewardship discipline described in §9.4.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
