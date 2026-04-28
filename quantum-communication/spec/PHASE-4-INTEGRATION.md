# WIA-COMM-006 — Phase 4: Integration

> Integration with NIST PQC, implementation guidance for telecom-grade QKD networks, and the references that anchor the security argument.

## 9. Post-Quantum Cryptography Integration

### 9.1 Hybrid QKD + PQC

**Motivation**:
- Transition period security
- Defense in depth
- Backward compatibility

**Approach**:
```
K_final = KDF(K_QKD || K_PQC)
```

**Advantages**:
- Security if either system secure
- Gradual deployment
- Future-proof

### 9.2 Post-Quantum Algorithms

**Key Encapsulation Mechanisms (KEM)**:
- CRYSTALS-Kyber (NIST standard)
- Use for initial key establishment

**Digital Signatures**:
- CRYSTALS-Dilithium (NIST standard)
- SPHINCS+ (stateless hash-based)
- Use for authentication

**Integration Points**:
- Initial authentication
- Classical channel protection
- Long-term key storage
- Hybrid key derivation

### 9.3 Quantum-Safe Migration

**Phase 1**: Parallel deployment
- Run QKD and PQC simultaneously
- Combine keys cryptographically

**Phase 2**: QKD primary
- Use QKD for session keys
- PQC for authentication and long-term keys

**Phase 3**: Full quantum security
- Quantum authentication
- Quantum digital signatures
- All-quantum infrastructure

---


## 10. Implementation Guidelines

### 10.1 System Design

**Components Checklist**:
- [ ] Quantum transmitter (laser + modulator)
- [ ] Quantum receiver (detectors + timing)
- [ ] Classical communication channel
- [ ] Synchronization system
- [ ] Error correction module
- [ ] Privacy amplification module
- [ ] Key management system
- [ ] Security monitoring

### 10.2 Calibration Procedures

**Daily**:
- Detector dark count measurement
- Channel loss measurement
- QBER baseline check

**Weekly**:
- Full system characterization
- Security parameter verification
- Detector efficiency calibration

**Monthly**:
- Component replacement as needed
- Software updates
- Security audit

### 10.3 Quality Assurance

**Testing**:
- Unit testing of each component
- Integration testing of full system
- Security testing (simulated attacks)
- Long-duration stability testing

**Certification**:
- Protocol compliance verification
- Security parameter validation
- Performance benchmarking
- Documentation completeness

### 10.4 Deployment Best Practices

**Site Selection**:
- Fiber: Use existing telecom infrastructure when possible
- Free-space: Clear line of sight, minimal atmospheric turbulence
- Satellite: Ground station location with clear sky view

**Environmental Control**:
- Temperature stability for interferometers
- Vibration isolation for free-space systems
- EMI shielding for electronics

**Redundancy**:
- Backup quantum channels
- Redundant classical channels
- Power backup systems
- Failover procedures

### 10.5 Integration with Existing Infrastructure

**Telecom Networks**:
- WDM compatibility
- Fiber sharing (separate wavelengths)
- Rack-mountable form factor

**Data Centers**:
- Standard interfaces (Ethernet, etc.)
- Key management system integration
- Monitoring system integration

**Enterprise IT**:
- VPN integration
- HSM (Hardware Security Module) compatibility
- Policy-based key distribution

---


## 11. References

### 11.1 Foundational Papers

1. C. H. Bennett and G. Brassard, "Quantum cryptography: Public key distribution and coin tossing," Proceedings of IEEE International Conference on Computers, Systems and Signal Processing, 1984.

2. A. K. Ekert, "Quantum cryptography based on Bell's theorem," Physical Review Letters, vol. 67, pp. 661-663, 1991.

3. C. H. Bennett, "Quantum cryptography using any two nonorthogonal states," Physical Review Letters, vol. 68, pp. 3121-3124, 1992.

### 11.2 Security Proofs

4. P. W. Shor and J. Preskill, "Simple proof of security of the BB84 quantum key distribution protocol," Physical Review Letters, vol. 85, pp. 441-444, 2000.

5. R. Renner, "Security of quantum key distribution," International Journal of Quantum Information, vol. 6, pp. 1-127, 2008.

### 11.3 Practical Implementations

6. N. Gisin, G. Ribordy, W. Tittel, and H. Zbinden, "Quantum cryptography," Reviews of Modern Physics, vol. 74, pp. 145-195, 2002.

7. ETSI GS QKD 002 — Security analysis framework for quantum-key-distribution systems.

### 11.4 Quantum Repeaters

8. H.-J. Briegel, W. Dür, J. I. Cirac, and P. Zoller, "Quantum repeaters: The role of imperfect local operations in quantum communication," Physical Review Letters, vol. 81, pp. 5932-5935, 1998.

9. N. Sangouard, C. Simon, H. de Riedmatten, and N. Gisin, "Quantum repeaters based on atomic ensembles and linear optics," Reviews of Modern Physics, vol. 83, pp. 33-80, 2011.

### 11.5 Standards Organizations

- ETSI ISG-QKD: European Telecommunications Standards Institute
- ITU-T: International Telecommunication Union
- ISO/IEC JTC 1/SC 27: Information security standards
- NIST: Post-Quantum Cryptography standardization

---

**弘益人間 (Benefit All Humanity)**

*This specification is dedicated to advancing secure communication for the benefit of all humanity. May quantum communication protect privacy, enable scientific discovery, and foster global cooperation.*

---

© 2025 SmileStory Inc. / WIA


## A.1 PQC integration discipline

QKD provides forward secrecy for the link layer; PQC (NIST FIPS 203/204/205) provides forward secrecy for the application layer. The standard requires both: QKD protects the symmetric key derivation; PQC protects the signature on every envelope so the standard remains secure even after Shors-class attacks become practical.

## A.2 Telecom-grade deployment guidance

Telecom-grade QKD requires:
- Dark fibre or dedicated lit fibre with documented loss profile
- Repeater stations every 50-100 km on long-haul deployments
- HSM-backed key storage at every endpoint
- Federation with the WIA-AIR-SHIELD trust list for cross-operator deployments

## A.3 Satellite-QKD operational notes

Satellite-QKD (Micius-class deployments) enables intercontinental key distribution. The bridge profile maps Phase 2 endpoints to satellite-pass schedules; ground stations submit session requests with predicted satellite-pass windows and the satellite operator confirms availability via the federation envelope.

## A.4 Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: BB84/E91/B92/MDI-QKD/CV-QKD stable |
| 1.1.x | Additive: twin-field QKD, decoy-state extensions |
| 1.2.x | Additive: post-quantum classical channel migration complete |
| 2.0.0 (no earlier than 2028) | Possible breaking change: full PQC migration of envelope signatures |


## A.5 Federation across operators

Cross-operator QKD deployments use the federation envelope from the
WIA-SOCIAL Phase 3 §5 receipt shape. Each operator publishes a trust
list naming peer operators; cross-operator key delivery emits a
signed handover envelope so both operators can audit the chain.

## A.6 Migration from legacy IPSec

Legacy IPSec deployments migrate to QKD-derived keys by routing the
IPSec key-exchange to the QKD endpoint. The bridge profile maps the
QKD `key/derive` endpoint to the IPSec IKEv2 PRF input so legacy
network appliances can consume QKD-derived keys without further
modification.

## A.7 References

- ISO/IEC 23837 — QKD security requirements
- ETSI GS QKD 014 — protocol implementation
- ETSI GS QKD 015 — control interface
- IETF RFC 8446 — TLS 1.3
- IETF RFC 9180 — Hybrid Public Key Encryption
- NIST FIPS 203/204/205 — PQC suite
- BIPM SI brochure — TAI conventions


## B.1 Conformance test suite

A black-box conformance test suite is published at
`https://github.com/WIA-Official/wia-quantum-communication-conformance` and walks
through every public endpoint plus the cross-Phase integration
scenarios. Hosts publishing `bridge_profile=Full` SHOULD additionally
pass the suite's bridge-extension tests.

The suite checks: discovery document round-trip, every Phase 2 endpoint
with mock data, problem-detail emission for malformed inputs, rate-limit
header presence and exhaustion behaviour, replay-defence bounds (300-second
skew, 600-second nonce cache), and audit-log envelope shape.

## B.2 Reference container

The `wia/quantum-communication-host:1.0.0` container image implements every Phase 2
endpoint with mock data; integrators exercise their bridge against it
before going to production. The container ships with a small library of
mock scenarios so the conformance suite has fixtures to run against.

## B.3 Companion CLI

The `cli/quantum-communication.sh` script ships sample envelope generators (validate,
info, plus phase-specific subcommands) so an implementer can produce
conformant payloads without hand-rolling JSON. The CLI has no dependency
beyond `jq` and POSIX shell, so it runs in any CI environment without
additional tooling installation.

## B.4 Operational considerations

Quantum-key-distribution infrastructure has three operational considerations
that integrators consistently underestimate. First, the wire-format
discipline: every envelope is signed and verified at the boundary;
unsigned envelopes are refused at conformance, full stop. Second, the
trust-list refresh cadence: stale trust anchors are the single largest
source of avoidable production incidents in this standard family. Third,
the audit-log replication discipline: audit logs replicated across only
one storage backend cannot survive a site failure, and a site failure
that takes the audit log with it leaves the operator unable to
reconstruct what happened during the failure window.

## B.5 Backwards-compatibility promise

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields, optional query parameters,
new envelope types, new endpoints, or new protocol exchanges; hosts
MUST NOT remove or repurpose existing ones. Breaking changes ride a
major version bump and MUST be preceded by a 12-month deprecation
window per IETF RFC 8594 and 9745.

## B.6 Governance

The standard is maintained by the WIA Standards Committee. Change
proposals follow the WIA RFC process: anyone may submit a proposal;
the Committee reviews quarterly; accepted proposals enter an open
comment period before merging into a minor-version release. Breaking
changes require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.

## C.1 Glossary

The following terms appear repeatedly throughout this Phase and
the wider quantum-communication standard: QBER (Quantum Bit Error Rate); BB84 (Bennett-Brassard 1984); E91 (Ekert 1991); B92 (Bennett 1992); MDI-QKD (Measurement-Device-Independent QKD); CV-QKD (Continuous-Variable QKD); twin-field QKD; decoy state; privacy amplification; sifted key; reconciled key.

Implementers unfamiliar with the domain should treat these terms as
load-bearing — every endpoint, every protocol exchange, and every
integration document below assumes the reader understands what each
term means in context. Expanded definitions appear in the standard's
companion glossary at `https://wiastandards.com/quantum-communication/glossary/`.

## C.2 Cross-standard composition

This Phase composes with adjacent WIA family standards as follows:

- **WIA-OMNI-API** owns credential storage and identity for every
  signed envelope in this Phase.
- **WIA-AIR-SHIELD** owns runtime trust list maintenance and key
  rotation; this Phase consumes WIA-AIR-SHIELD events.
- **WIA-SOCIAL Phase 3 §5** receipt shape is reused verbatim for
  every cross-host federation handshake referenced in this Phase.
- **WIA-INTENT** owns the outermost-layer declaration of workload
  intent; consumers parse the intent envelope before drilling into
  this Phase's specifics.

The composition is intentional: a single host running multiple WIA
family standards reuses the same identity, signature, and federation
machinery across all of them rather than maintaining N parallel
implementations. The conformance suite verifies the composition by
running a multi-standard scenario where this Phase's envelopes flow
through the adjacent standards' machinery and back.

## C.3 Implementation runbook

A first implementation of this Phase typically follows the runbook:

1. Stand up the reference container ('wia/quantum-communication-host:1.0.0') in a
   development environment.
2. Run the conformance suite against the container to verify all
   tests pass on the reference implementation.
3. Replace the mock backend with the host's real backend
   one endpoint at a time; re-run conformance after each
   replacement.
4. Wire up the audit log replication; verify a full session round
   trip lands in both replicas.
5. Onboard a single trusted peer for federation; exercise the
   handshake and audit envelope flow.
6. Expand to multiple peers; rotate trust anchors per the
   30-day cadence.
7. Promote to production; subscribe operations to the warning
   envelope cadence (collateral expiry, drift detection,
   barren-plateau onset, etc., as relevant per phase).

The full runbook is roughly two engineer-weeks of work for a team
already familiar with the underlying domain (QKD, QML, quantum-
network, or quantum-sensor as applicable).
