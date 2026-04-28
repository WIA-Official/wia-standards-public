# WIA-QUA-003 — Phase 4: Integration

> Integration with classical-internet TCP/IP stack, implementation guidance for telecom-grade quantum repeater chains, and the references list that grounds the design.

## 10. Implementation Guidelines

### 10.1 Hardware Requirements

**Photon Sources**:
- Type: SPDC, quantum dots, or single-photon sources
- Wavelength: 1550 nm (telecom C-band preferred)
- Brightness: > 10⁶ pairs/s/mW

**Detectors**:
- Type: Superconducting nanowire (SNSPD) or InGaAs APD
- Efficiency: > 80% (SNSPD) or > 20% (APD)
- Dark count rate: < 100 Hz
- Timing jitter: < 100 ps

**Quantum Memories**:
- Storage time: > 1 ms (metro), > 1 s (long-distance)
- Efficiency: > 50%
- Fidelity: > 0.99

### 10.2 Software Requirements

**Control Software**:
- Real-time synchronization (< 1 μs precision)
- Timing control for pulsed systems
- Data acquisition and processing

**Network Management**:
- Topology discovery
- Route computation
- Resource allocation
- Monitoring and diagnostics

### 10.3 Calibration

**Polarization Alignment**:
- Align reference frames between nodes
- Compensate for fiber birefringence
- Periodic re-calibration (hourly)

**Timing Synchronization**:
- GPS or White Rabbit protocol
- Precision: < 100 ps
- Continuous monitoring

**Detector Calibration**:
- Efficiency measurement
- Dark count characterization
- Afterpulsing assessment

---


## 11. References

### 11.1 Foundational Papers

1. Bennett, C.H., & Brassard, G. (1984). "Quantum cryptography: Public key distribution and coin tossing." *IEEE International Conference on Computers, Systems and Signal Processing*, 175-179.

2. Ekert, A.K. (1991). "Quantum cryptography based on Bell's theorem." *Physical Review Letters*, 67(6), 661.

3. Bennett, C.H. (1992). "Quantum cryptography using any two nonorthogonal states." *Physical Review Letters*, 68(21), 3121.



### 11.2 Recent Advances




### 11.3 Standards and Protocols

9. ETSI GS QKD 002: "Quantum Key Distribution (QKD); Use Cases and Applicability"

10. ETSI GS QKD 014: "Quantum Key Distribution (QKD); Protocol and data format of REST-based key delivery API"

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*


## A.1 Classical-internet integration

Quantum networks coexist with the classical internet on a parallel infrastructure. The bridge profile maps quantum-network primitives to classical-internet flows: classical IP packets ride the standard internet; quantum-network protocol envelopes ride a dedicated TLS 1.3 channel to the network controller.

## A.2 Telecom-grade deployment

Telecom-grade quantum networks require: dark fibre (lit fibre's classical traffic creates Raman noise that swamps single-photon detectors), repeater stations every 50-100 km, HSM-backed identity for every node, and federated trust lists across operator boundaries.

## A.3 Quantum-internet roadmap

The full quantum internet (with entanglement at planetary scale and untrusted intermediate nodes) is a 2030+ deployment target. The standard's envelope schema is designed to absorb the transition without breaking existing trusted-node networks; the architecture-tier field tells consumers what they can expect.

## A.4 Implementation guidance

A first quantum-network deployment typically targets a metro-area trusted-node network with 3-5 nodes connected by dedicated dark fibre. The reference container at `wia/quantum-network-host:1.0.0` implements every Phase 2 endpoint with mock entanglement so integrators can test their bridge before deploying real quantum hardware.


## A.5 Bridge to TCP/IP

Classical TCP/IP for control-plane messaging; quantum-network protocol
envelopes for data-plane entanglement requests. The bridge maps the
two planes via a shared network-controller endpoint that owns the
mapping between classical IP addresses and quantum-network URNs.

## A.6 Federation across operators

Cross-operator quantum networks use the federation envelope from
WIA-SOCIAL Phase 3 §5 receipt shape. The trust-list refresh cadence
is faster than for other WIA family standards (typically 1 hour vs
30 days) because quantum-network fidelity drift is fast and trust
must track operational reality.

## A.7 Reference deployment

A first quantum-network deployment is typically a metro-area
trusted-node network with 3-5 nodes connected by dedicated dark
fibre. The reference deployment guide at
`https://wiastandards.com/quantum-network/reference-deploy/`
documents the per-node hardware requirements and the operational
runbook for the first 90 days of production.

## A.8 Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: trusted-node + curve-repeater tiers stable |
| 1.1.x | Additive: more measurement-device-independent protocols |
| 1.2.x | Additive: quantum-internet-tier reference specification |
| 2.0.0 (no earlier than 2030) | Possible breaking change: full quantum-internet migration |

## A.9 References

- IETF RFC 8446 — TLS 1.3
- IETF RFC 9180 — HPKE
- ETSI GS QKD 014 / 015 — QKD interoperability
- ITU-T Y.3800 series — quantum information networks
- BIPM SI brochure — TAI conventions


## B.1 Conformance test suite

A black-box conformance test suite is published at
`https://github.com/WIA-Official/wia-quantum-network-conformance` and walks
through every public endpoint plus the cross-Phase integration
scenarios. Hosts publishing `bridge_profile=Full` SHOULD additionally
pass the suite's bridge-extension tests.

The suite checks: discovery document round-trip, every Phase 2 endpoint
with mock data, problem-detail emission for malformed inputs, rate-limit
header presence and exhaustion behaviour, replay-defence bounds (300-second
skew, 600-second nonce cache), and audit-log envelope shape.

## B.2 Reference container

The `wia/quantum-network-host:1.0.0` container image implements every Phase 2
endpoint with mock data; integrators exercise their bridge against it
before going to production. The container ships with a small library of
mock scenarios so the conformance suite has fixtures to run against.

## B.3 Companion CLI

The `cli/quantum-network.sh` script ships sample envelope generators (validate,
info, plus phase-specific subcommands) so an implementer can produce
conformant payloads without hand-rolling JSON. The CLI has no dependency
beyond `jq` and POSIX shell, so it runs in any CI environment without
additional tooling installation.

## B.4 Operational considerations

Quantum-network infrastructure has three operational considerations
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
the wider quantum-network standard: entanglement swap; quantum repeater; quantum memory; quantum teleportation; trusted-node network; curve repeater network; quantum-internet; link-state announcement; fidelity-weighted routing; URN scheme; trust list.

Implementers unfamiliar with the domain should treat these terms as
load-bearing — every endpoint, every protocol exchange, and every
integration document below assumes the reader understands what each
term means in context. Expanded definitions appear in the standard's
companion glossary at `https://wiastandards.com/quantum-network/glossary/`.

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

1. Stand up the reference container ('wia/quantum-network-host:1.0.0') in a
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
