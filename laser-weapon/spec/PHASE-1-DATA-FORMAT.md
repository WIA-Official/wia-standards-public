# WIA-laser-weapon PHASE 1 — Data Format Specification

**Standard:** WIA-laser-weapon
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for directed-energy
weapon (DEW) — laser-class — operational records: weapon-system
identifiers, beam-emission events, target-engagement records,
thermal/coolant telemetry, beam-control quality metrics, atmospheric-
propagation evidence, optical-payload calibration records, and the
cross-references that bind these to the broader fire-control picture.
The shape interoperates with allied tactical-data-link standards
(STANAG 5516 J-series via WIA-missile-defense) so a coalition
deployment does not require parallel data models.

References (CITATION-POLICY ALLOW only):
- STANAG 5516 — Tactical Data Link 16 (J-series messages, J11.x weapon-direction, J12.x weapon-status)
- STANAG 4607 — Ground Moving Target Indicator format (for ground-based sensor cueing)
- IEC 60825-1 — Safety of laser products (eye-safety classification)
- ANSI Z136.1 — Safe use of lasers (referenced for human-rated zones)
- ISO 11146 — Lasers and laser-related equipment: Test methods for laser beam widths, divergence, and propagation ratios (M²)
- ISO 13694 — Lasers and laser-related equipment: Test methods for laser beam power density distribution
- IEC 61040 — Power and energy measuring detectors, instruments and equipment for laser radiation
- WGS-84 — geodetic reference frame
- IETF RFC 7515 (JWS), RFC 8259 (JSON), RFC 9162 (Certificate Transparency 2.0 pattern)

---

## §1 Scope

This PHASE applies to systems that schedule, fire, or report on
laser-class directed-energy effects: high-energy laser (HEL) for
counter-UAS, counter-rocket-artillery-mortar (C-RAM), counter-
small-boat, anti-sensor (dazzling) within rules of engagement,
and ground-test-range instrumentation for HEL development. It
addresses the *shape* of operational records; protocols for
transport are in PHASE 3, integration with C2 in PHASE 4.

The standard is rules-of-engagement-aware: an implementation MUST
declare the controlling RoE under which its records originate
(coalition, national, range-test-only) so downstream consumers
apply the correct interpretation of engagement authority and
target-class authorisation.

In scope: weapon-system identification, beam-emission lifecycle,
target-engagement records, thermal and coolant telemetry, beam-
quality metrics, atmospheric-channel observations, optical-payload
calibration. Out of scope: chemical-laser fuel handling (covered by
HazMat standards), kinetic-warhead intercept (handled by
WIA-missile-defense), high-power-microwave (separate WIA standard).

## §2 Weapon-system identifier

Each laser weapon system carries a stable identifier:

| Field             | Source / Binding                                       |
|-------------------|--------------------------------------------------------|
| `wsRef`           | URN of form `urn:wia:lw:ws:<authority>:<id>`           |
| `vendor`          | manufacturer of record                                 |
| `model`           | model designation                                      |
| `serialNumber`    | manufacturer serial                                    |
| `wavelengthBand`  | one of `nir-1µm` (Yb fibre, Nd:YAG class), `mid-ir`,  |
|                   | `vis-green-532nm` (test-range), `uv` (research)        |
| `nominalAveragePower` | nominal kW band; precise output is per-fire reported   |
| `beamDirector`    | gimbal type, optical-aperture diameter, IEC 60825-1 class |
| `coolingArchitecture` | air, liquid (closed-loop or open), thermo-electric  |
| `eyeSafetyClass`  | IEC 60825-1 class (typically Class 4 for HEL)          |

A weapon system not in the deployment's registry is refused at
beam-emission time. Registry changes are auditable.

## §3 Beam-emission record

Every beam emission emits a record:

- `emissionId` — URN of form `urn:wia:lw:emission:<wsRef>:<seq>`
- `wsRef` — emitting weapon-system URN
- `decisionRef` — engagement decision (PHASE 1 §4) or test-range
  schedule reference
- `startTimestamp` — RFC 3339 with offset (microsecond precision
  where the underlying clock supports it)
- `endTimestamp` — RFC 3339 with offset
- `beamGeometry` — pointing vector at start (azimuth, elevation,
  WGS-84-aligned) plus angular-rate covariance
- `pulseModel` — closed enum: `cw` (continuous wave), `quasi-cw`,
  `pulsed-train` (PRF + duty cycle), `single-pulse`
- `commandedAveragePower` — kW + measurement uncertainty
- `commandedDuration` — seconds (or pulse-count for pulsed)
- `actualEnergyDelivered` — joules + uncertainty (post-shot
  calorimeter or telemetry-integrated)

Each record is signed by the weapon system's signing key. The
audit chain links the engagement decision to the emission to the
weapon-status (PHASE 1 §6).

## §4 Engagement decision link

Engagement decisions for laser weapons reuse the WIA-missile-defense
PHASE 1 §6 record shape extended with:

- `engagementCategory` — closed enum: `counter-uas`, `c-ram`,
  `counter-small-boat`, `anti-sensor-dazzle`, `optical-disable`,
  `range-test`
- `roeAuthorisationRef` — RoE evidence required for the category
- `eyeSafetyZoneCheck` — boolean indicating clearance from the
  IEC 60825-1 / ANSI Z136.1 hazard-zone planner before fire

A `range-test` decision does not require RoE; it requires a
range-control authorisation reference instead. An engagement
without an eye-safety-zone-check pass at the moment of fire is
recorded as a test-only event regardless of the engagement
category.

## §5 Target record

For combat engagements (excluding range-test):

- `targetId` — URN
- `kinematicCategory` — drawn from WIA-missile-defense PHASE 1 §3
  (cruise-missile, low-altitude UAS, surface boat, etc.)
- `trackRef` — fused-track URN from the missile-defence boundary
- `intendedEffect` — closed enum: `kinetic-kill`, `mission-kill`,
  `sensor-degrade`, `non-lethal-deter`
- `expectedDwellTime` — seconds at commanded power required to
  achieve the intended effect, with a confidence band (`low`,
  `medium`, `high`); precise prediction is per-vendor lethality
  tables held outside this PHASE

## §6 Weapon-status record (laser-specific)

Extending WIA-missile-defense PHASE 1 §7 with laser-specific states:

- `state` — one of `cold` (off), `warming`, `armed-low-power`
  (rangefinder/track only), `armed-full-power` (ready to fire),
  `firing`, `cooldown`, `fault`
- `thermalState` — primary-element temperature in °C with delta
  vs. cooldown-target
- `coolantState` — coolant flow rate, return temperature, reservoir
  level
- `opticalAxisState` — beam-director gimbal pointing health (within
  spec / tracking / saturated / lost)

State transitions are signed; state transitions out of order
(e.g., `firing` directly after `cold` without `warming` and
`armed-full-power`) are rejected.

## §7 Thermal / coolant telemetry

For each emission and warm-up cycle:

- `cycleId` — URN
- `wsRef` — weapon-system URN
- `samples[]` — per-sample: timestamp, primary-element temperature,
  coolant flow rate, coolant return temperature, reservoir level
- `thermalLimitsExceeded` — boolean; if true, the cycle terminated
  before commanded duration

Sample rate is per-vendor; typical 10 Hz to 100 Hz during firing.
Exceeded thermal limits trigger automatic cooldown and an audit
event tagged for engineering review.

## §8 Beam-quality metric

Per-emission beam quality is recorded post-shot:

- `bqId` — URN
- `emissionRef` — emission URN
- `m2` — beam quality factor (M²) per ISO 11146; ≥ 1 (1 = ideal
  diffraction-limited)
- `pointingStability` — RMS angular jitter in microradians over the
  emission duration
- `intensityProfileRef` — URI of the recorded power-density
  distribution (per ISO 13694) for the emission's representative
  sample
- `aperturePower` — measured kW at aperture per IEC 61040

Beam-quality records feed the deployment's mission-effectiveness
analysis. Trends are tracked per weapon system to surface
component degradation.

## §9 Atmospheric-channel observation

For each emission, the boundary records the propagation channel:

- `channelId` — URN
- `emissionRef` — emission URN
- `slantRangeMetres` — geometric distance to target at start of
  emission
- `cnSquared` — atmospheric refractive-index structure parameter
  (Cn²) along the path; reported as a band (low / moderate / high
  turbulence) when full profiling is unavailable
- `aerosolBand` — visible-IR transmittance band (clear / haze /
  fog / smoke / dust)
- `weatherSnapshot` — temperature, pressure, humidity, surface wind
  at engagement location

Atmospheric channel records help post-event analysis distinguish
beam-quality issues from atmospheric ones; they are also used for
mission-planning when forecasting next engagement windows.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                     | Use site                                                 |
|-------------------------------|----------------------------------------------------------|
| WIA-missile-defense           | engagement decisions intake (PHASE 4 §2)                 |
| WIA-military-communication    | cross-coalition disclosure of outcomes                   |
| WIA-nbc-defense               | casualty integration when intercept produces NBC effects |
| WIA-public-safety             | civil-emergency notification when test-range overlap     |

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` reference the
deployment policy explaining the gap; `excluded` carries a
justification citing the controlling RoE allowance. A deployment
that is `partial` or `excluded` on §3 (Beam emission), §4
(Engagement decision link), §6 (Weapon status), or §7 (Thermal
telemetry) is non-conformant overall.

## Annex C — Versioning and deprecation

Versioning follows Semantic Versioning 2.0.0. Coalition-shared
records carry the originating deployment's version so cross-coalition
audit replay works across version transitions. Deprecation enters
a 12-month sunset window with migration notes recorded in the audit
chain.

## Annex D — Worked beam-emission record (informative)

A typical counter-UAS engagement at 50 kW:

```json
{
  "emissionId": "urn:wia:lw:emission:rok-army.hel-bty-1:e-91a7-2026-04-27",
  "wsRef": "urn:wia:lw:ws:rok-army:hel-bty-1",
  "decisionRef": "urn:wia:md:decision:d-91a7",
  "startTimestamp": "2026-04-27T09:31:14.523000+09:00",
  "endTimestamp": "2026-04-27T09:31:18.045000+09:00",
  "beamGeometry": {
    "azimuthRad": 1.234567,
    "elevationRad": 0.567890,
    "azimuthRateRadS": 0.0123,
    "elevationRateRadS": -0.0045,
    "covariance": [[1e-7, 0], [0, 1e-7]]
  },
  "pulseModel": "cw",
  "commandedAveragePower": {"kw": 50, "uncert": 1.5},
  "commandedDuration": 3.5,
  "actualEnergyDelivered": {"joules": 167500, "uncert": 4500},
  "atmosphericChannelRef": "urn:wia:lw:channel:c-91a7",
  "beamQualityRef": "urn:wia:lw:bq:b-91a7"
}
```

The boundary chains this record to the WIA-missile-defense
decision and the post-shot weapon-status transitions so end-to-end
reconstruction (decision → emission → outcome) works across
both audit chains.

## Annex E — Range-test record schema (informative)

For range-test deployments a separate record schema applies:

- `rangeRef` — URN of the test-range
- `testPlanRef` — URN of the test-plan document
- `testCondition` — combinations of distance / target type / atmospheric
  mode being characterised
- `rangeAuthorisationRef` — URN of the range-control authorisation

Range-test emissions carry the same beam-emission shape as combat
emissions but with `decisionRef` replaced by `rangeAuthorisationRef`
and `engagementCategory: range-test`.

## Annex F — Optical-payload calibration record (informative)

Calibration records bind the weapon system to its measurement
chain back to a national metrology institute's traceable standards:

- `calibrationId` — URN
- `wsRef` — weapon system being calibrated
- `calibrationDate` — RFC 3339 with offset
- `m2Baseline` — measured M² at calibration time per ISO 11146
- `pointingStabilityBaseline` — RMS angular jitter at calibration
- `powerMeterCalibration` — primary power meter's calibration
  certificate URI (typically traceable to NIST or KRISS)
- `apertureBeamProfileSamplePath` — URI of the as-measured
  power-density distribution per ISO 13694
- `calibratingLaboratory` — URN of the calibrating laboratory (must
  hold ISO 17025 accreditation for the modality)
- `nextCalibrationDue` — date

A weapon system whose calibration is past `nextCalibrationDue` is
moved to `armed-low-power` only state until recalibrated; engagement
command intake refuses full-power emissions for an out-of-calibration
weapon system.

## Annex G — Pulse-mode notes

Pulsed-train pulseModel records carry an additional pulseRepetitionRateHz field plus per-pulse-energy uncertainty so that thermal stress on optical elements can be reconstructed post-event. Single-pulse and quasi-cw flows do not require these extensions.

## Annex H — RoE crosswalk

Engagement category to RoE-authority crosswalk is per-deployment and held in deployment policy.

## Annex I — Conformance level
Implementations declare conformance level (Surface/Verified/Anchored) per WIA-missile-defense PHASE 3 Annex E.
