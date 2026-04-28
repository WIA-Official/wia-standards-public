# WIA-QUA-020 — Phase 2: API Interface

> Scientific-instrument canonical Phase 2: API surface (instruments + measurements + calibrations + methods + samples + telemetry + audit).

# WIA-QUA-020: Scientific Instrument Standard
# Version 1.0.0

**Standard ID:** WIA-QUA-020
**Title:** Scientific Instrument
**Category:** QUA (미래기술/양자/물리)
**Status:** Active
**Published:** 2025-01-01
**Updated:** 2025-01-01

---

## Abstract

This specification defines a comprehensive standard for advanced scientific instruments used in research, discovery, and innovation. The WIA-QUA-020 standard covers particle accelerators, mass spectrometers, electron microscopes, X-ray crystallography systems, NMR spectrometers, gravitational wave detectors, telescopes, spectrophotometers, chromatography systems, calorimeters, data acquisition systems, and calibration standards.

**弘益人間 (Benefit All Humanity)** - This standard facilitates global collaboration, data sharing, and reproducibility in scientific research.

---


## 3. Data Acquisition Systems

### 3.1 Analog-to-Digital Conversion (ADC)

**Specifications:**
- Resolution: 12-24 bits
- Sampling rate: 1 kHz to 10 GHz
- Input range: ±10 V
- Accuracy: 0.01%

### 3.2 Signal Processing

**Digital Filtering:**
- Low-pass filters
- Band-pass filters
- Notch filters (50/60 Hz)
- Savitzky-Golay smoothing

**Fast Fourier Transform (FFT):**
```
X(k) = Σ x(n)·e^(-i·2π·k·n/N)
```

---



## 9. API Specification

### 9.1 TypeScript SDK

See `api/typescript/` for full implementation.

### 9.2 REST API

**Base URL:** `https://api.wia-instrument.org/v1`

**Endpoints:**
- `GET /instruments` - List instruments
- `GET /instruments/{id}` - Get instrument details
- `POST /instruments/{id}/measure` - Start measurement
- `GET /instruments/{id}/status` - Get status
- `GET /instruments/{id}/data` - Download data

---




---

## A.1 Endpoint reference

```http
POST /scientific-instrument/v1/instruments         # register instrument
GET  /scientific-instrument/v1/instruments/{id}    # fetch instrument record
POST /scientific-instrument/v1/measurements        # submit measurement result
GET  /scientific-instrument/v1/measurements/{id}   # fetch measurement
GET  /scientific-instrument/v1/calibrations/{id}   # fetch calibration cert
POST /scientific-instrument/v1/calibrations        # record new calibration
GET  /scientific-instrument/v1/methods/{id}        # fetch method record
POST /scientific-instrument/v1/samples             # register sample
WS   /scientific-instrument/v1/state/stream        # instrument state stream
GET  /scientific-instrument/v1/audit/{id}          # audit trail
```

Every endpoint follows the discovery convention at
`/.well-known/wia-scientific-instrument`. Instrument-registration
endpoints require a laboratory-administrator credential plus the
ISO/IEC 17025 accreditation scope of the operating laboratory.

## A.2 Instrument-registration API

`POST /instruments` accepts the Phase 1 §A.1 envelope. The endpoint
validates the capability descriptor against the laboratory's
accredited scope (an instrument outside accreditation scope is
flagged with a non-fatal warning), allocates the per-instrument
calibration-tracking record, and returns the instrument's registration
URI. Instruments transition through states `provisioning`, `available`,
`in-use`, `out-of-calibration`, `under-maintenance`, `retired`; state
transitions emit audit events that survive instrument deregistration.

## A.3 Measurement-submission API

`POST /measurements` accepts the Phase 1 §A.2 envelope. The endpoint
enforces method-validity gating: a measurement submitted under a
method whose validation expired is accepted but flagged
`pending-revalidation`. The endpoint computes the measurement-
uncertainty budget on the server side as a defense-in-depth check
against operator transcription error and emits a warning if the
client-supplied uncertainty deviates from the server computation
beyond a documented tolerance. Storage of raw-data payloads is
content-addressed via SHA-256 (per FIPS 180-4) so duplicate
submissions are deduplicated transparently.

## A.4 Calibration-record API

`POST /calibrations` accepts the Phase 1 §A.3 envelope. The endpoint
verifies the calibrating-laboratory's ISO/IEC 17025 accreditation
status against the ILAC-MRA signatory database (cached daily), and
verifies that every reference-standard cited in the calibration chain
ultimately traces to an SI unit through a recognised national
metrology institute. Calibrations whose chain breaks (e.g., a vendor
"factory-traceable" claim with no NMI link) are accepted but flagged
`traceability-gap` and excluded from accreditation-scope reporting.

## A.5 Method-management API

`GET /methods/{id}` returns the method record: method identifier,
method-document URI (linking to the underlying ISO / ASTM / USP /
IUPAC / vendor / in-house method), method-validation envelope per
ICH Q2(R2) (specificity, accuracy, precision, linearity, range,
detection limit, quantitation limit, robustness — with the
acceptance criterion and pass/fail outcome per parameter),
revalidation-due date, and the change-control envelope per ICH Q9
where the method has been modified since validation.

## A.6 Telemetry WebSocket

The state-stream WebSocket multiplexes per-instrument events: power
state, run state (idle / running / aborted / errored), per-method
progress (e.g., chromatographic run-time progress), error events
with severity classification, environmental-sensor envelope
(ambient temperature, humidity, vibration, power voltage), and the
auto-tuning / re-zero / autosampler state envelope. Subscribers can
filter by instrument-class, by error-class, and by environmental-
threshold-crossing predicate. Rate limits: 1000 req/h authenticated,
5000 req/h trusted-partner. WebSocket subscriptions are bounded at
50 simultaneous per credential.

## A.7 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: instrument
registration, every calibration event, every measurement event with
the method reference, every state transition, every operator-
authentication event tied to instrument operation, every method-
modification event, and the retirement event. The audit-trail
integrity is anchored into a Merkle tree per-laboratory and the
root is committed to the operator's hardware-token-secured archive
per ISO/IEC 27037 §6.3.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/scientific-instrument/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-scientific-instrument-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/scientific-instrument-host:1.0.0` ships every scientific-instrument envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/scientific-instrument.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Scientific-instrument deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-scientific-instrument-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

弘益人間 — Benefit All Humanity.
