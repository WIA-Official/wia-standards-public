# WIA-MED-006 PHASE 4: Safety Protocols Specification

## Overview
This specification defines comprehensive safety requirements for surgical robotic systems.

## 1. Safety Standards Compliance

### 1.1 Mandatory Standards
- **IEC 60601-1**: Medical electrical equipment - General safety
- **IEC 60601-2-77**: Robotically-assisted surgical equipment
- **ISO 13482**: Safety requirements for personal care robots
- **ISO 14971**: Medical device risk management
- **IEC 62304**: Medical device software lifecycle (Class C)

## 2. Hardware Safety

### 2.1 Emergency Stop System
- **E-Stop Buttons**: Minimum 2 (console + bedside)
- **Response Time**: <50ms to power cutoff
- **Action**: Immediate motor power disconnect + brake engagement
- **Indicator**: Red LED + audible alarm
- **Recovery**: Manual reset required after E-stop

### 2.2 Safety PLC
- **Independence**: Separate from main control system
- **Redundancy**: Dual independent channels
- **Watchdog**: <100ms timeout
- **Self-Test**: Power-on + periodic (every 1 hour)

### 2.3 Mechanical Limits
- **Hard Stops**: Physical mechanical limiters at joint extremes
- **Limit Switches**: Redundant switches before hard stops
- **Breakaway Links**: Designed failure points (500N threshold)

## 3. Software Safety

### 3.1 Fail-Safe Behaviors
- **Communication Loss**: Stop within 1 second, hold position
- **Sensor Failure**: Switch to redundant sensor or safe stop
- **Software Crash**: Watchdog reset, safe mode boot
- **Power Loss**: Battery backup (5-10 min), controlled shutdown

### 3.2 Safety-Critical Functions
- **Workspace Limits**: Software boundaries, warn at 90%, stop at 100%
- **Force Limits**: Configurable max force (default 15N), hard limit 20N
- **Velocity Limits**: Max 100mm/s near patient, 500mm/s repositioning
- **Collision Detection**: Real-time arm-arm and arm-patient checks

### 3.3 Software Verification
- **Static Analysis**: MISRA C compliance
- **Unit Testing**: >90% code coverage
- **Integration Testing**: All safety scenarios
- **Formal Verification**: Safety-critical algorithms

## 4. Procedural Safety

### 4.1 Pre-Surgery Checklist
- [ ] Patient identity confirmed
- [ ] Surgical site marked
- [ ] System self-test passed
- [ ] Tool calibration verified
- [ ] Emergency stop tested
- [ ] Backup plan established
- [ ] Team roles assigned

### 4.2 Intra-operative Monitoring
- **Control Latency**: Continuous monitoring, alert if >20ms
- **Position Error**: Alert if >1.0mm deviation
- **Motor Current**: Alert if >90% rated
- **Temperature**: Alert if >40°C
- **Network (remote)**: Alert if latency >100ms or packet loss >0.1%

### 4.3 Event Logging
- **Log All Events**: System events, user actions, safety events, performance
- **Format**: [Timestamp] [Severity] [Component] [Message]
- **Storage**: Local (1 hour buffer) + central (7 years)
- **Encryption**: AES-256 at rest
- **Audit Trail**: Tamper-proof, legally compliant

## 5. Emergency Procedures

### 5.1 Emergency Classification
- **Level 1**: Information (monitor)
- **Level 2**: Caution (slow down)
- **Level 3**: Warning (modify action)
- **Level 4**: Emergency (stop, handover)
- **Level 5**: Crisis (open conversion, CPR)

### 5.2 Local Handover Protocol
1. **Recognition** (1-5s): System failure detected
2. **Assessment** (5-15s): Severity evaluation
3. **Decision** (5-10s): Continue vs. handover
4. **Transition** (1-3min): Robot to safe position, detach, bedside surgeon takes over
5. **Documentation**: Incident logging

## 6. Quality Assurance

### 6.1 Maintenance Schedule
- **Daily**: Visual inspection, self-test, cleaning
- **Weekly**: Deep clean, supplies check, log review
- **Monthly**: Precision calibration, preventive maintenance
- **Quarterly**: Comprehensive inspection, external audit
- **Annual**: Full system validation, regulatory compliance check

### 6.2 Performance Metrics (KPIs)
- **Uptime**: >98%
- **Error Rate**: <0.1% per surgery
- **Mean Time to Repair**: <2 hours
- **Complication Rate**: <5%
- **Conversion Rate**: <2%

## 7. Training Requirements

### 7.1 Mandatory Training
- **System Operation**: 40 hours minimum
- **Safety Protocols**: 8 hours
- **Emergency Procedures**: 4 hours + quarterly drills
- **Simulator**: GEARS score ≥20

### 7.2 Certification
- **Initial**: WIA-MED-006 Level 2 minimum
- **Renewal**: Every 5 years
- **Continuing Education**: 10 CME credits annually

---

**Status**: ✅ Complete  
**Version**: 1.0.0  
**Date**: 2025-01-01

© 2025 WIA · MIT License

## P.4 Integration Cross-References

This Phase describes how the data formats (Phase 1), API surface (Phase 2),
and protocol layer (Phase 3) compose with adjacent infrastructure to form a
production deployment.

### P.4.1 Deployment Topologies

| Topology | When to Use | Trade-off |
|----------|------------|-----------|
| Single-region active-passive | Predictable latency, single-region users | Cold standby cost |
| Multi-region active-active | Global users, regional sovereignty | Conflict resolution complexity |
| Edge fan-out | Low latency at the edge, central system of record | Cache coherence |
| Air-gapped enclave | Regulatory / national security domains | Manual reconciliation |

### P.4.2 Dependency Inventory

Every implementation MUST publish a Software Bill of Materials (SBOM) in
SPDX 2.3 or CycloneDX 1.5 format covering: (a) direct runtime dependencies,
(b) transitive dependencies pinned to specific versions, (c) base container
images, (d) cryptographic libraries.

### P.4.3 Operational Readiness Checklist

- [ ] Health check endpoint returns 200 within 1 s p99
- [ ] Metrics exposed in Prometheus or OTLP format
- [ ] Logs are structured JSON with correlation IDs
- [ ] Traces use W3C Trace Context headers end-to-end
- [ ] Backups verified by quarterly restore drill
- [ ] Runbook published and indexed
- [ ] Disaster recovery RTO / RPO documented
- [ ] On-call rotation defined and acknowledged

### P.4.4 Migration Pathways

Adopters migrating from legacy systems should follow the staged pattern:
(1) shadow read, (2) shadow write, (3) primary write with legacy fallback,
(4) primary read, (5) legacy decommission. Each stage runs for at least one
business cycle before the next.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of WIA-ROB-006: Surgical Robot Standard so that conformance claims at any
Phase remain unambiguous.*

