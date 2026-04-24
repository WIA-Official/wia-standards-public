# WIA-MED-BE-001 · Bionic Eye — Phase 2: API Interface

**Version:** 1.1.0
**Status:** Active
**Last Updated:** 2026-04-25
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 2 specifies the **clinician programming interface (CPI)**, the **patient-app interface (PAI)**, and the **research telemetry interface (RTI)** used to configure, run, and analyse a WIA-compliant visual prosthesis. All three surfaces speak the same JSON/Arrow wire format and reuse the Phase 1 data contract.

Three distinct consumers, one interface family:

| Surface | Consumer          | Scope                                                |
|---------|-------------------|------------------------------------------------------|
| CPI     | Ophthalmology / neuro-prosthetics clinician | fitting, thresholding, safety overrides |
| PAI     | Patient and caregivers     | start/pause/stop, scenes, mode, brightness  |
| RTI     | Research / regulators      | de-identified psychophysics and device logs |

### 1.1 Normative references

- OpenAPI Specification 3.1.0
- RFC 9457 — Problem Details for HTTP APIs
- HL7 FHIR R5 — `Device`, `DeviceMetric`, `Procedure`, `Observation`
- ISO 15223-1 — Symbols used with medical device labels
- IEC 62366-1 — Usability engineering
- ISO 14971 — Application of risk management to medical devices
- ISO/IEC 81001-5-1 — Health software security lifecycle

---

## 2. Base URL, devices, and discovery

```
https://cpi.bionic-eye.wia.org/v1        # clinician
https://pai.bionic-eye.wia.org/v1        # patient
https://rti.bionic-eye.wia.org/v1        # research
```

Each surface publishes `/.well-known/wia-bionic-eye`:

```json
{
  "standard": "WIA-MED-BE-001",
  "surface":  "CPI",
  "version":  "1.1",
  "supported_implants": ["epiretinal-60e", "subretinal-378e",
                         "subretinal-1600e", "cortical-60e"],
  "websocket": "wss://cpi.bionic-eye.wia.org/v1/stream",
  "openapi":   "https://cpi.bionic-eye.wia.org/v1/openapi.json"
}
```

### 2.1 Authentication

- **Clinician (CPI)**: SMART-on-FHIR launch with `launch/patient`, `launch/encounter`, and custom scopes `wia.be.fitting.read`, `wia.be.fitting.write`, `wia.be.safety.override`. Short-lived JWTs.
- **Patient (PAI)**: device-bound credential (FIDO2 passkey) + caregiver recovery; access tokens are rotated nightly.
- **Research (RTI)**: study-specific OAuth 2.1 client credentials; row-level filter by IRB protocol.

Every request MUST pass TLS 1.3 with OCSP stapling; device-side clients pin the server's intermediate CA.

---

## 3. Resource model

```
/devices/{device_id}                         implant unit (one per patient)
/devices/{device_id}/fittings                fitting sessions
/devices/{device_id}/fittings/{id}           full fitting document
/devices/{device_id}/fittings/{id}/threshold-walk   psychophysics protocol
/devices/{device_id}/phosphene-map           latest accepted map
/devices/{device_id}/phosphene-map/history   versioned history (R/O)
/devices/{device_id}/safety/overrides        override log (append-only)
/devices/{device_id}/telemetry               recent cycles (RTI only)
/devices/{device_id}/modes                   scene preset list
/devices/{device_id}/emergency-stop          POST = trigger e-stop
/patients/{patient_id}/qol                   quality-of-life survey stream
```

---

## 4. Fitting session lifecycle

A fitting session is the clinical appointment where the implant's electrode thresholds, waveforms, and phosphene map are reset.

### 4.1 State machine

```
  draft ── start ─► running ── commit ─► accepted ──┐
    │                 │                             │
    │                 └── abandon ─► rolled_back    │
    └───── cancel ───► cancelled                    │
                                                    ▼
                                              immutable archive
```

### 4.2 Endpoints (CPI)

```
POST   /devices/{id}/fittings                  # create draft
GET    /devices/{id}/fittings/{fid}            # read
PATCH  /devices/{id}/fittings/{fid}            # mutate draft
POST   /devices/{id}/fittings/{fid}:start      # start (safety check)
POST   /devices/{id}/fittings/{fid}:commit     # commit (irrevocable)
POST   /devices/{id}/fittings/{fid}:abandon    # roll back to previous accepted
GET    /devices/{id}/fittings/{fid}/log        # full audit trail
```

### 4.3 Commit gate (safety-critical)

A fitting commit MUST refuse when any of these conditions hold:

| Reject code                      | Reason                                        |
|----------------------------------|-----------------------------------------------|
| `shannon_violation`              | Any electrode exceeds `k ≤ 1.5` envelope      |
| `impedance_out_of_spec`          | Any active electrode Z outside configured band|
| `threshold_missing`              | Fewer than 90 % of electrodes have threshold  |
| `dc_leakage_high`                | ≥ 100 nA measured during probe sweep          |
| `psychophysics_incomplete`       | No 50 % threshold for ≥ 10 % of active set    |
| `patient_not_present`            | Patient presence beacon not detected          |
| `clinician_scope_missing`        | Token lacks `wia.be.fitting.write`            |

Every reject returns an RFC 9457 problem document and writes a non-repudiable entry to the audit log.

---

## 5. Psychophysics — threshold walk

### 5.1 Protocol

A two-down/one-up staircase (Levitt 1971), bracketing a 70.7 % correct response.

```http
POST /devices/dev-001/fittings/f-42/threshold-walk
Content-Type: application/json

{
  "electrodes": [1, 2, 3, 4, 5, 6, 7, 8],
  "start_uA":   100,
  "step_uA_coarse": 20,
  "step_uA_fine":    5,
  "reversals_coarse": 3,
  "reversals_fine":   6,
  "max_trial_count":  60,
  "catch_trial_rate": 0.15,
  "stimulus_ms":      200,
  "iti_ms":          1500
}
```

### 5.2 Trial stream (WebSocket)

```
wss://cpi.bionic-eye.wia.org/v1/stream?topic=threshold&device=dev-001
```

Server frames:

```json
{ "electrode": 3, "trial": 17, "amp_uA": 85, "response": "yes",
  "reaction_ms": 412, "staircase_state": "fine:r3" }
```

When all electrodes converge the server emits:

```json
{ "electrode": 3, "threshold_uA": 78, "slope": 0.024, "CI95": [71, 86] }
```

### 5.3 Psychometric fit

After the walk, the server MUST fit a Weibull function per electrode and store the parameters alongside the raw trials. The fitting document references the trial set by checksum so it is auditable post-hoc.

---

## 6. Phosphene-map editor

### 6.1 Get and put

```
GET  /devices/{id}/phosphene-map
PUT  /devices/{id}/phosphene-map?If-Match={revision}
```

Body is the array of `PhospheneLocus` objects (Phase 1 § 5.1). `If-Match` is mandatory to prevent lost updates.

### 6.2 Diff and dry run

```
POST /devices/{id}/phosphene-map:diff
  body = proposed map → returns added / removed / changed loci
POST /devices/{id}/phosphene-map:dry-run
  body = proposed map → simulates cycles against last 100 recorded scenes
```

The dry-run response includes a *percept quality index* (PQI, 0–100) based on Beyeler et al. (2019) axon-pathway model; clinicians are advised against committing maps with PQI delta < 0 without a written justification.

---

## 7. Modes and scene presets (PAI)

Patients switch between preset stimulation profiles:

```
GET  /devices/{id}/modes
POST /devices/{id}/modes/{mode_id}:activate
```

Default modes:

| Mode         | Description                                      |
|--------------|--------------------------------------------------|
| `navigation` | Emphasis on edges + obstacle priority            |
| `reading`    | High-contrast OCR, slow refresh                  |
| `face`       | Face-first salience, privacy guard on bystanders |
| `dim`        | Low ambient lux, thresholds scaled down          |
| `eco`        | Reduced duty cycle for battery extension         |

### 7.1 Intensity slider

```
POST /devices/{id}/intensity   body: {"scale": 0.6}   # 0–1
```

Hardware MUST clamp `scale × amplitude` to `max_safe_current_uA`.

### 7.2 Emergency stop (patient-facing)

```
POST /devices/{id}/emergency-stop  body: {"reason":"patient-discomfort"}
```

Triggers the Phase 4 three-phase stop sequence. Always returns `202` because the action cannot be reverted synchronously.

---

## 8. Telemetry and research (RTI)

### 8.1 Cycles batch

```
GET /devices/{id}/telemetry?since=2026-04-24T00:00:00Z&limit=10000
Accept: application/vnd.apache.arrow.stream
```

Returns an Arrow IPC stream of `VisualCycle` rows with fields sorted by `timestamp`. De-identification is enforced at the gateway; any request that would emit PHI returns `403 phi_redacted`.

### 8.2 Psychophysics export

```
GET /studies/{study_id}/trials
    ?device={id}&from=2026-04-01&to=2026-04-30
    &format=parquet
```

The IRB protocol number gates access; export logs include researcher ID, destination domain, and record count — a monthly digest is published to the study's Linear/Jira ticket.

---

## 9. Error model (RFC 9457)

```json
{
  "type":     "https://errors.bionic-eye.wia.org/shannon_violation",
  "title":    "Commit rejected: Shannon charge-density limit exceeded",
  "status":   422,
  "detail":   "Electrode 7 requested 240μA × 250μs = 0.060μC into 0.00196cm²",
  "instance": "/devices/dev-001/fittings/f-42:commit",
  "errors": [
    {"electrode": 7, "Q_uC": 0.060, "A_cm2": 0.00196,
     "log10_Q_over_A": 1.49, "k_allowed": 1.50 }
  ]
}
```

Other canonical `type` suffixes: `impedance_out_of_spec`, `threshold_missing`, `map_stale` (`If-Match` mismatch), `device_offline`, `patient_not_present`, `safety_override_required`.

---

## 10. Rate limits and safety quotas

| Class                         | Burst | Refill       | Note                               |
|-------------------------------|-------|--------------|------------------------------------|
| CPI: fitting mutations        |   10  |   1 / 60 s   | intentional friction for safety    |
| CPI: threshold walk (sessions)|    4  |   1 / 10 min | CPU-bound psychophysics            |
| PAI: mode switch              |   60  |   2 / s      | user-facing latency-sensitive      |
| PAI: emergency-stop           |  100  |  10 / s      | no-throttle policy ≥ 10 req/s      |
| RTI: telemetry bulk           |   10  |   1 / 30 s   | Arrow bulk export                  |

`RateLimit-Limit / Remaining / Reset` headers are set on every response per Phase 2 convention shared with other WIA standards.

---

## 11. Worked examples

### 11.1 Clinician — run a threshold walk and commit a fitting

```bash
TOKEN=$(smart-on-fhir-login --client clinic-cli \
        --scope "wia.be.fitting.write launch/patient")

curl -sS -H "Authorization: Bearer $TOKEN" \
  https://cpi.bionic-eye.wia.org/v1/devices/dev-001/fittings \
  -X POST -H 'Content-Type: application/json' \
  -d '{"notes":"Routine 3-month review"}'
# → 201, Location: /devices/dev-001/fittings/f-42

curl -sS -H "Authorization: Bearer $TOKEN" \
  https://cpi.bionic-eye.wia.org/v1/devices/dev-001/fittings/f-42/threshold-walk \
  -X POST -d @walk-protocol.json

# (stream trials on WebSocket…)

curl -sS -H "Authorization: Bearer $TOKEN" \
  https://cpi.bionic-eye.wia.org/v1/devices/dev-001/fittings/f-42:commit \
  -X POST
```

### 11.2 Patient — switch to reading mode

```swift
let req = URLRequest(url:"https://pai.bionic-eye.wia.org/v1/devices/dev-001/modes/reading:activate")
let (_, resp) = try await URLSession.shared.data(for: req)   // 202 Accepted
```

### 11.3 Researcher — export trial data

```python
import pyarrow.flight as fl
client = fl.FlightClient("grpc+tls://rti.bionic-eye.wia.org:443")
tkn = client.authenticate_basic_token(b"study-42", b"")
opts = fl.FlightCallOptions(headers=[tkn])
ticket = client.get_flight_info(fl.FlightDescriptor.for_path("trials","study-42","2026-04"),
                                options=opts).endpoints[0].ticket
table = client.do_get(ticket, opts).read_all()
```

---

## 12. Audit, non-repudiation, and FHIR integration

### 12.1 FHIR mapping

- `/devices/{id}` → `Device` resource (`identifier.system = "urn:wia:be:device"`).
- Each fitting commit writes a `Procedure` resource (`code = 91873-9 "Visual prosthesis fitting"`).
- Every trial set writes a `Observation` with `code = 91874-7 "Retinal-stimulation threshold"` and `valueQuantity` in μA.

### 12.2 Non-repudiation

Commits are signed with the clinician's device-held ES256 key; the signature MUST cover the full fitting JSON canonicalised per RFC 8785. The signed payload is appended to an immutable ledger (`/devices/{id}/fittings/{fid}/log`), periodically anchored on the WIA Chain so tampering is detectable across institutions.

### 12.3 Alarm routing

The Emergency-stop endpoint mirrors every activation to the hospital's alarm channel (FHIR `Communication` with `priority = asap`) and to the caregiver app; Phase 4 covers the three-phase hardware response.

---

## 13. Compliance checklist

- [ ] `openapi.json` reachable; `.well-known` discovery present on each surface
- [ ] SMART-on-FHIR launch works on CPI; FIDO2 passkey on PAI
- [ ] Fitting state machine enforced server-side
- [ ] Commit gate rejects on the eight safety codes (§ 4.3)
- [ ] Threshold-walk stream emits per-trial events and converged thresholds
- [ ] `If-Match` enforced on phosphene-map PUT
- [ ] Emergency-stop is always accepted, never throttled
- [ ] Telemetry returns Arrow streams; PHI is redacted
- [ ] FHIR Device/Procedure/Observation resources mirror CPI events
- [ ] Audit log is signed (ES256) and chain-anchored

---

## 14. References

1. Levitt H (1971). Transformed up-down methods in psychoacoustics. *J. Acoust. Soc. Am.* 49.
2. Beyeler M et al. (2019). A model of ganglion axon pathways accounts for percepts. *Scientific Reports* 9.
3. SMART-on-FHIR App Launch Framework 2.0.
4. RFC 9457; RFC 8785 (JSON Canonicalization).
5. HL7 FHIR R5 — Device, Procedure, Observation, Communication.
6. ISO 14971; ISO/IEC 81001-5-1; IEC 62366-1.

---

**Document Status:** ACTIVE
**Effective Date:** April 25, 2026
**Review Date:** April 25, 2028

© 2026 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
