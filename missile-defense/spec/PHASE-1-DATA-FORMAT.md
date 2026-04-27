# WIA-missile-defense PHASE 1 — Data Format Specification

**Standard:** WIA-missile-defense
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for missile-defence
records: track identifiers, sensor observations, fused tracks,
threat-assessment outputs, engagement decisions, weapon-status
reports, intercept telemetry, and the cross-references that bind
these to the operational picture. The shape interoperates with
allied tactical-data-link standards (Link 16 / J-series) so that
a multi-national air- and missile-defence network does not require
parallel data models.

References (CITATION-POLICY ALLOW only):
- STANAG 5516 — Tactical Data Link 16 (J-series messages)
- STANAG 5522 — Tactical Data Link 22
- STANAG 5511 — Link 11/11B
- MIL-STD-6016 — Tactical Data Link J-series messages (US realisation)
- MIL-STD-2525D — Common Warfighting Symbology
- ICAO Doc 4444 — air-traffic management procedures (for civil-airspace overlap)
- WGS-84 (NIMA TR8350.2) — geodetic reference frame
- ISO 19115 — geographic information metadata
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 9162 (Certificate Transparency 2.0 pattern)

---

## §1 Scope

This PHASE applies to systems that detect, track, classify, decide
upon, and engage ballistic, hypersonic, cruise, and tactical-missile
threats. It addresses the *shape* of records produced and consumed
by sensors, fusion engines, command-and-control systems, and weapon
systems. Protocols for transport are addressed in PHASE 3, integration
with the broader operational picture in PHASE 4.

The standard is allied-doctrine-aware: an implementation MUST declare
which national or coalition doctrine its records originate under so
that downstream consumers can apply the correct interpretation of
identification (friend / hostile / unknown), engagement authority,
and rules of engagement.

In scope: track and fused-track records, threat assessments, fire-
control solutions, engagement decisions, weapon-status, intercept
telemetry. Out of scope: launch-platform internals (covered by the
respective platform's WIA standard) and post-engagement battle-damage
assessment beyond the immediate outcome flag.

## §2 Track identifier model

Every observation and fused track carries a structured identifier:

- `trackId` — URN of form `urn:wia:md:track:<authority>:<seq>`
- `authority` — issuing fusion-engine identifier (one fusion engine
  per coalition partner; cross-coalition fusion uses a federated
  authority)
- `seq` — monotonically increasing sequence within the authority
- `sourceObservations[]` — IDs of contributing observations (PHASE 1 §3)

Identifiers are *globally unique and never reused*. Track
correlation (when two fusion engines determine they hold the same
physical object) creates a federation track that references both
authorities' tracks; neither original is retired.

## §3 Sensor observation record

A sensor (radar, IR/optical search-and-track, space-based asset)
emits an observation:

- `obsId` — URN of form `urn:wia:md:obs:<sensor>:<timestamp>:<seq>`
- `sensorRef` — sensor URN (PHASE 4 §3 fleet registry)
- `timestamp` — RFC 3339 with offset; sensor clock plus authoritative
  time-source skew (PHASE 3 §6)
- `position` — WGS-84 lat/lon/alt with covariance matrix (3×3); for
  range-rate sensors include `range`, `bearing`, `elevation` plus
  `rangeRate` with their respective uncertainties
- `velocity` — 3-vector m/s in WGS-84-aligned ENU frame at the
  position fix, with covariance
- `kinematicCategory` — one of `surface`, `low-altitude`, `aircraft`,
  `cruise-missile`, `tactical-ballistic`, `medium-range-ballistic`,
  `intermediate-range-ballistic`, `intercontinental-ballistic`,
  `hypersonic-glide`, `space-object`
- `signature` — sensor-specific signature features (RCS, IR
  band-specific intensities, optical-photometric magnitude); fields
  vary by sensor type but each is a structured numeric with units
  and uncertainty
- `metadata` — sensor model, firmware, calibration date

Permitted units (closed): position in metres, velocity in m/s,
range in metres, bearing/elevation in radians, RCS in m², IR
intensity in W/sr-µm. Observations outside the unit list are rejected.

## §4 Identification (IFF / NCTR)

Tracks carry an identification:

- `identification` — one of `pending`, `unknown`, `assumed-friend`,
  `friend`, `neutral`, `suspect`, `hostile` (coalition-aligned with
  STANAG 5516 J3.x identification fields)
- `identificationBasis[]` — list of evidence: `iff-mode-3a`,
  `iff-mode-5`, `iff-mode-s`, `nctr` (non-cooperative target
  recognition), `flight-plan-correlation`, `corridor-correlation`,
  `behaviour-pattern`
- `identifyingPrincipal` — URN of the operator or fusion engine
  that asserted the identification
- `identificationTimestamp` — RFC 3339 with offset

Identifications are append-only at the per-source level; conflicting
identifications from different sources surface as a federation
event for coalition resolution rather than auto-merge.

## §5 Threat-assessment record

A fused track that meets a doctrine threshold receives a threat
assessment:

- `assessmentId` — URN
- `trackRef` — fused track URN
- `threatLevel` — one of `none`, `low`, `medium`, `high`, `critical`
- `threatType` — coded threat type drawn from the deployment's
  threat-catalogue (e.g., `tbm-class-A`, `cm-low-altitude`,
  `hypersonic-glide-vehicle`)
- `predictedImpact` — predicted impact area (polygon) and time
  window with uncertainty
- `assessingPrincipal` — URN of the assessing fusion engine
- `confidenceBand` — `tentative`, `probable`, `confirmed`

Threat assessments are revisable as additional sensor data arrives;
each revision is a new record referencing the prior. The chain of
revisions is hash-chained so an after-the-fact rewrite is detectable.

## §6 Engagement decision record

When the rules of engagement and assessment justify, an engagement
decision is recorded:

- `decisionId` — URN
- `assessmentRef` — threat assessment that triggered the decision
- `engagementType` — one of `intercept`, `divert`, `monitor-only`,
  `non-engagement`
- `decidingAuthority` — URN of the engagement authority who decided
- `roeBasis[]` — list of rules-of-engagement codes invoked
- `weaponSystemRef` — URN of the weapon system tasked (if intercept)
- `expectedKillProbability` — band (`low`, `medium`, `high`) — *not*
  a fabricated numeric; numeric Pk is out of scope because no
  cross-vendor methodology is universally agreed
- `decisionTimestamp` — RFC 3339 with offset

Decisions are append-only; revising a decision (cease-fire, hand-off
to another weapon system, abort) issues a new decision record
referencing the prior. The audit chain links the assessment, the
decision, and the resulting weapon-status reports (§7).

## §7 Weapon-status record

When a weapon system is tasked, it emits status records:

- `statusId` — URN
- `weaponSystemRef` — weapon-system URN
- `decisionRef` — engagement decision being executed
- `state` — one of `slewed`, `armed`, `launched`, `boost`, `mid-course`,
  `terminal`, `intercept-attempt`, `success`, `miss`, `aborted`
- `stateTimestamp` — RFC 3339 with offset
- `interceptorRef` — URN of the specific interceptor (if launched)
- `telemetry` — interceptor telemetry summary (position, velocity,
  remaining propellant, guidance lock status); structure varies by
  weapon system

State transitions are signed by the weapon system's signing key so
that the weapon system's status cannot be spoofed by another node
on the network.

## §8 Intercept-outcome record

After an engagement, the outcome is recorded:

- `outcomeId` — URN
- `decisionRef` — engagement decision
- `outcome` — one of `success`, `partial`, `miss`, `aborted`,
  `unknown`
- `confidenceBasis[]` — evidence for the outcome (sensor
  re-acquisition, debris track, optical signature change)
- `assessingPrincipal` — URN of the assessing fusion engine
- `outcomeTimestamp` — RFC 3339 with offset

Outcomes are revisable as additional evidence (e.g., post-event
sensor re-acquisition) refines the assessment. Each revision is a
new record referencing the prior.

## §9 Allied report shapes (informative)

This PHASE preserves report shapes used in allied operations:

| Shape                  | Trigger                                        |
|------------------------|-----------------------------------------------|
| Track-state report     | aggregated fused-track snapshot                |
| Threat-assessment alert | new or escalated threat assessment             |
| Engagement-status      | per-engagement timeline summary                |
| Weapon-status summary   | aggregated weapon-system readiness              |
| Post-engagement summary | aggregated intercept outcomes for a period     |

Reports are derived projections rendered on demand from the
canonical records; the canonical record remains in the WIA shape,
which is more general and lossless.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked example: tactical-ballistic track (informative)

```json
{
  "trackId": "urn:wia:md:track:fusion-A:91a7",
  "authority": "fusion-A",
  "seq": 91,
  "kinematicCategory": "tactical-ballistic",
  "lastUpdate": "2026-04-27T09:31:14.523+09:00",
  "currentState": {
    "position": {"lat": 37.4516, "lon": 126.6531, "alt": 12500, "covarianceM2": [...]},
    "velocity": {"vx": 280, "vy": -50, "vz": 1500, "covarianceM2s2": [...]}
  },
  "identification": {
    "identification": "hostile",
    "identificationBasis": ["nctr", "behaviour-pattern"],
    "identifyingPrincipal": "urn:wia:md:fusion:fusion-A",
    "identificationTimestamp": "2026-04-27T09:31:13.910+09:00"
  },
  "sourceObservations": [
    "urn:wia:md:obs:radar-7e2:2026-04-27T09:31:14.523+09:00:0001",
    "urn:wia:md:obs:radar-7e3:2026-04-27T09:31:14.617+09:00:0001",
    "urn:wia:md:obs:eo-91a7:2026-04-27T09:31:14.610+09:00:0001"
  ]
}
```

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` reference the
deployment policy explaining the gap; sections marked `excluded`
carry a justification citing the controlling doctrine's allowance.
A deployment that is `partial` or `excluded` on §3 (Sensor observation),
§4 (Identification), §5 (Threat assessment), §6 (Engagement decision),
or §7 (Weapon status) is non-conformant overall.

## Annex C — Cross-domain references (informative)

| Reference                          | Use site                                                |
|------------------------------------|---------------------------------------------------------|
| WIA-military-communication         | engagement decision transport, federation messaging     |
| WIA-medical-data-privacy           | casualty triage from intercept-debris consequences      |
| WIA-medical-imaging                | imaging-driven medical-evacuation coordination          |
| WIA-public-safety                  | civil-emergency notification for predicted-impact       |
| WIA-nbc-defense                    | when intercept produces NBC consequences (e.g., chem warhead) |

The boundary verifies the cross-domain reference exists at the
referenced standard's boundary before delivery so downstream
readers do not see dangling references.

## Annex D — Versioning and deprecation (informative)

Versioning follows Semantic Versioning 2.0.0. Major bumps require
a coalition-wide compatibility window of at least 90 days where
both versions are honoured by every operationally-fielded reference
implementation. Deprecated versions enter a 12-month sunset window
during which the registry marks the version as Deprecated; migration
notes are recorded in the audit chain so coalition partners know
which version their counterpart honours.

Patch-level errata are issued without a deprecation window because
they do not change normative behaviour. Errata are tracked in a
public errata register whose entries are signed by the WIA Standards
working group chair.

## Annex E — Allied report worked rendering (informative)

A track-state report rendered from the canonical records:

```json
{
  "reportId": "urn:wia:md:report:trackstate:2026-04-27T09:31:30+09:00",
  "asOf": "2026-04-27T09:31:30+09:00",
  "tracks": [
    {
      "trackRef": "urn:wia:md:track:fusion-A:91a7",
      "kinematicCategory": "tactical-ballistic",
      "identification": "hostile",
      "currentState": {"position": {...}, "velocity": {...}},
      "associatedAssessments": ["urn:wia:md:assessment:9c0a"]
    }
  ],
  "issuedBy": "urn:wia:md:fusion:fusion-A",
  "signature": "<JWS detached>"
}
```

A receiving system can replay the rendering inputs against the
canonical records to confirm fidelity. A discrepancy between the
report and the canonical record is treated as an integrity incident.
