# WIA-laser-weapon PHASE 2 — API Interface Specification

**Standard:** WIA-laser-weapon
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a laser-weapon deployment exposes
for engagement-command intake (from WIA-missile-defense), pre-fire
eye-safety zone validation, beam-emission lifecycle reporting,
thermal/coolant telemetry streaming, beam-quality post-shot
publication, and range-test scheduling. The shape is HTTP/JSON for
C2/range-control planes; tactical fire-control loops use the
constrained binary mapping described in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS)
- STANAG 5516 — for J11.x weapon-direction inbound
- IEC 60825-1 — eye-safety classification
- ANSI Z136.1 — laser hazard zones
- ISO 11146 — M² measurement methods
- WIA-missile-defense PHASE 2 — referenced for engagement decision delivery
- WIA-military-communication PHASE 2 — referenced for cross-domain transport

---

## §1 Engagement-command intake

```
POST /commands/engage HTTP/1.1
Host: lw.coalition-c2.example
Authorization: Bearer eyJhbGciOiJFUzI1NiJ9...
Content-Type: application/wia-lw+json
WIA-RoE-Authorisation: urn:wia:lw:roe:rok-army.adcc:authA-2026

{
  "decisionRef": "urn:wia:md:decision:d-91a7",
  "wsRef": "urn:wia:lw:ws:rok-army:hel-bty-1",
  "engagementCategory": "counter-uas",
  "targetRef": "urn:wia:md:track:fusion-A:91a7",
  "intendedEffect": "mission-kill",
  "expectedDwellTime": {"seconds": 3.5, "confidence": "medium"}
}
```

The boundary validates the RoE authorisation, runs the eye-safety
zone check (PHASE 2 §2) before issuing a fire instruction to the
weapon system, and records the engagement decision. On rejection
the response carries `urn:wia:lw:problem:roe-not-authorised`,
`urn:wia:lw:problem:eye-safety-zone-violation`, or
`urn:wia:lw:problem:weapon-not-armed`.

## §2 Eye-safety-zone check

```
POST /safety/zone-check HTTP/1.1
{
  "wsRef": "urn:wia:lw:ws:...",
  "beamGeometryAtFire": {"azimuth": 1.234, "elevation": 0.567},
  "beamGeometryAtEnd": {"azimuth": 1.235, "elevation": 0.568},
  "atmosphericChannel": {...}
}
```

The boundary returns `clear` or `violation` with the violating
zone (per IEC 60825-1 / ANSI Z136.1 nominal-ocular-hazard distance
NOHD plus operational buffer). The deployment policy enumerates
human-rated zones and friendly-asset zones that always trigger
violation.

## §3 Beam-emission lifecycle

The weapon system pushes lifecycle events:

```
POST /emissions HTTP/1.1
{
  "emissionId": "urn:wia:lw:emission:hel-bty-1:e-91a7",
  "wsRef": "urn:wia:lw:ws:hel-bty-1",
  "decisionRef": "urn:wia:md:decision:d-91a7",
  "startTimestamp": "2026-04-27T09:31:14.523000+09:00",
  "beamGeometry": {...},
  "pulseModel": "cw",
  "commandedAveragePower": {"kw": 50, "uncert": 1.5},
  "commandedDuration": 3.5
}
```

End-of-emission update:

```
PUT /emissions/<emissionId> HTTP/1.1
{
  "endTimestamp": "2026-04-27T09:31:18.045000+09:00",
  "actualEnergyDelivered": {"joules": 167500, "uncert": 4500},
  "thermalLimitsReached": false
}
```

Emissions are append-only at the lifecycle level; corrections
issue a new record with a `corrigenda` reference.

## §4 Thermal/coolant telemetry stream

Thermal and coolant samples flow as a stream:

```
GET /weapons/<wsRef>/thermal/$subscribe HTTP/1.1
Authorization: Bearer ...
Accept: text/event-stream
```

The boundary streams Server-Sent Events at the configured sample
rate (typically 10 Hz). Subscribers consume for safety monitoring
and for engineering trend analysis.

## §5 Beam-quality publication

```
POST /emissions/<emissionId>/beam-quality HTTP/1.1
{
  "m2": 1.32,
  "pointingStability": {"rmsMicroradians": 8.7},
  "intensityProfileRef": "https://lw-storage.example/profiles/...",
  "aperturePower": {"kw": 49.4, "uncert": 1.0}
}
```

Beam-quality records feed the deployment's mission-effectiveness
analysis. The boundary indexes M² and pointing-stability per
weapon system so trend reports surface degradation early.

## §6 Atmospheric-channel logging

```
POST /emissions/<emissionId>/atmospheric-channel HTTP/1.1
{
  "slantRangeMetres": 1240,
  "cnSquared": "moderate",
  "aerosolBand": "haze",
  "weatherSnapshot": {"tempC": 14.2, "pressureHpa": 1013, "rh": 0.72, "windMs": 4.5}
}
```

Atmospheric records pair with beam-quality records so post-event
analysis distinguishes channel effects from weapon-internal effects.

## §7 Range-test scheduling

For range-test deployments:

```
POST /range/schedule HTTP/1.1
{
  "wsRef": "...",
  "testPlan": "<reference to test plan document>",
  "scheduledStart": "...",
  "scheduledEnd": "...",
  "rangeControlAuthorisationRef": "..."
}
```

Range-test schedules are gated by the range-control authorisation;
schedules are visible to all range stakeholders so over-scheduling
is detected.

## §8 Status query

```
GET /weapons/<wsRef>/status HTTP/1.1
```

Returns the most recent weapon-status record (PHASE 1 §6),
including thermal state, coolant state, optical-axis state, and
state-transition timeline since the last cooldown.

## §9 Errors and warnings

| URI                                              | Status | Meaning                                       |
|--------------------------------------------------|-------:|-----------------------------------------------|
| `urn:wia:lw:problem:roe-not-authorised`          | 403    | engagement category outside RoE authorisation |
| `urn:wia:lw:problem:eye-safety-zone-violation`   | 422    | proposed beam path violates IEC 60825-1 zone  |
| `urn:wia:lw:problem:weapon-not-armed`            | 409    | weapon not in armed-full-power state          |
| `urn:wia:lw:problem:thermal-limit-pending`       | 503    | thermal state too high for new emission       |
| `urn:wia:lw:problem:m2-out-of-band`              | 422    | post-shot M² exceeds operational threshold    |
| `urn:wia:lw:problem:state-transition-invalid`    | 422    | state transition out of order                  |
| `urn:wia:lw:problem:audit-unavailable`           | 503    | audit chain write failed                      |

Warnings (200-OK with caveats) use `Warning:` headers per RFC 7234
§5.5 with codes namespaced under `wia-lw-`.

## Annex A — Worked engagement sequence (informative)

1. WIA-missile-defense engagement decision issues to laser-weapon
   boundary via PHASE 2 §1
2. Boundary runs eye-safety-zone check; clears
3. Boundary instructs weapon system to fire; weapon transitions
   `armed-full-power` → `firing`
4. Weapon emits PHASE 2 §3 startEmission record
5. Thermal stream publishes per-sample telemetry
6. Weapon transitions `firing` → `cooldown` at end of duration
7. Weapon emits endEmission update
8. Beam-quality and atmospheric records published post-shot
9. WIA-missile-defense outcome record references this emission
   for the kill-assessment chain

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex B — Capability advertisement (informative)

```
GET /.well-known/wia/laser-weapon/capabilities HTTP/1.1
```

Response advertises supported engagement categories, supported
weapon-system classes (HEL kW bands, IEC 60825-1 classes), the
deployment's RoE authorities, the eye-safety-zone planner reference,
the atmospheric-monitoring sensor inventory, and the federation
peers. Capability documents are signed.

## Annex C — Pagination + rate limiting (informative)

Emission and weapon-status queries paginate at ≤ 1000 entries per
page. Per-token rate-limit defaults: 100 engagement-commands per
minute per fire-control authority, 1000 thermal samples per second
per weapon system. Rate-limit refusals carry
`urn:wia:lw:problem:rate-limited`.

## Annex D — Idempotency

Engagement commands accept `Idempotency-Key`; retries within 24 h
with the same key return the original response. Different bodies
under the same key are rejected with
`urn:wia:lw:problem:idempotency-conflict`.

## Annex E — Worked thermal-stream sample (informative)

```
event: thermal-sample
data: {"timestamp":"2026-04-27T09:31:14.523+09:00","wsRef":"urn:wia:lw:ws:hel-bty-1","primaryElementC":42.3,"coolantFlowLpm":35.2,"coolantReturnC":31.7,"reservoirLevelPct":0.78}

event: thermal-limit-warning
data: {"timestamp":"2026-04-27T09:31:17.910+09:00","wsRef":"...","level":"approaching","field":"primaryElementC","value":78.4,"threshold":80.0}
```

Subscribers (safety operator console + engineering trend store)
consume the same stream; safety operator surfaces warnings, the
engineering store retains for trend analysis.

## Annex F — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. Coalition-shared
records carry the originating deployment's version in the audit
chain. Deprecation enters a 12-month sunset window with migration
notes recorded in the chain so coalition partners know which
version their counterpart honours.

## Annex G — Worked range-test schedule (informative)

```
POST /range/schedule HTTP/1.1
Host: lw.range-c.example
Authorization: Bearer ...
Content-Type: application/wia-lw+json

{
  "wsRef": "urn:wia:lw:ws:test-range-A:hel-1",
  "testPlan": "https://range-c.example/plans/2026-04-27-mocking-uas",
  "scheduledStart": "2026-04-27T13:00:00+09:00",
  "scheduledEnd": "2026-04-27T17:00:00+09:00",
  "rangeControlAuthorisationRef": "urn:wia:lw:rangeauth:rangeC-2026-04-27"
}
```

Response on accepted schedule:

```
201 Created
Location: /range/schedule/sched-91a7
WIA-Audit-Event-Id: urn:wia:lw:audit:2026-04-27T08:30:00+09:00:9c01
```

Conflicting schedules (overlapping range time-windows for the same
weapon system) are refused with `urn:wia:lw:problem:range-conflict`.

## Annex H — Long-running emission status

Long-dwell emissions (e.g., 30-second cooking against a hardened
target) update the lifecycle event with intermediate progress:

```
POST /emissions/<emissionId>/progress HTTP/1.1
{
  "elapsedSeconds": 15.0,
  "energyDeliveredJoulesSoFar": 720000,
  "thermalState": "approaching-limit"
}
```

This is informational; the canonical record is the start + end
records (PHASE 2 §3).

## Annex I — Worked atmospheric-channel logging (informative)

Atmospheric records flow per emission so that post-shot beam-quality
analysis can attribute degradation to atmospheric vs. weapon-internal
sources. A worked example for an engagement at slant-range 1240 m
through moderate-haze atmosphere:

```json
{
  "channelId": "urn:wia:lw:channel:c-91a7",
  "emissionRef": "urn:wia:lw:emission:hel-bty-1:e-91a7",
  "slantRangeMetres": 1240,
  "cnSquared": "moderate",
  "cnSquaredNumeric": 8.2e-15,
  "aerosolBand": "haze",
  "weatherSnapshot": {
    "tempC": 14.2,
    "pressureHpa": 1013,
    "rh": 0.72,
    "windMs": 4.5,
    "windDir": 230
  },
  "transmissionEstimate": {
    "atSourceWavelengthBand": "nir-1µm",
    "endToEndTransmittance": 0.62,
    "atmosphericLossDb": 2.07
  }
}
```

The transmissionEstimate is computed at the boundary from a vendor-
provided atmospheric-loss model; the deployment's mission-effectiveness
analysis joins this with the beam-quality record to attribute
on-target intensity correctly.

## Annex J — Idempotency policy

Engagement-command and emission-lifecycle POST/PUT calls accept Idempotency-Key. Boundary stores keys for 24 h; retries within that window with the same key return the original response. Different bodies under the same key return urn:wia:lw:problem:idempotency-conflict.

## Annex K — Capability versioning

Capability documents declare wia.standardVersion alongside wia.implementationVersion so coalition partners verify compatibility.
