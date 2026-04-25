# WIA-ROB-SVC-001: Service Robot
## Phase 2 — Perception & Social Context

**Standard ID:** WIA-ROB-SVC-001
**Category:** Robotics — Service
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 2 specifies how a service robot's perception layer constructs the world model L3 reasons over and the *social context* annotations that distinguish a service robot from an industrial one. Where an industrial robot needs to know that something is in its way, a service robot needs to know whether the something is a person, who that person is to the task at hand, what they are likely about to do, and whether they have indicated they need or do not want the robot's attention.

## 2. World Model Required Channels

The world model delivered to L3 contains:

```
WorldModel := {
    metric_grid:     OccupancyGrid(local, ego-centered, ≤ N seconds old),
    person_tracks:   List<PersonTrack>,
    object_tracks:   List<ObjectTrack>,
    venue_layer:     Map<RegionID, VenueLabel>,
    self_pose:       Pose(stamped),
    sensor_health:   Map<SensorID, HealthStatus>,
    model_age:       Duration since last full update
}
```

`person_tracks` and `object_tracks` are separated because the policy authority over the two differs sharply. Approaching a person requires different stand-off, signaling, and yielding rules than approaching an object.

## 3. Person Track Schema

```
PersonTrack := {
    id:              UUID (stable across update steps within a session),
    pose:            Pose(stamped),
    velocity:        Velocity(stamped),
    intent:          Enum(passing, approaching, attending, waiting, unclear),
    attention:       float in [0, 1],   // estimated attention toward platform
    accessibility_cues: List<Cue>,      // mobility aid, service animal, white cane, child
    confidence:      float in [0, 1],
    provenance:      List<SensorID>
}
```

`intent` and `attention` are estimates. The estimates are normative — they must be present — but their accuracy is bounded by sensor availability and lighting. A platform must not present a low-confidence intent estimate as if it were certain. Where intent is `unclear`, L3 policies must default to non-intrusive behavior (continue at distance, wait for clearer signal).

`accessibility_cues` is a list because a person may present multiple cues simultaneously. The list is open: vendors may extend with additional cues, but the standard's defined set must be supported.

## 4. Defined Accessibility Cues

| Cue | Inferred from | L3 Implication |
|-----|--------------|----------------|
| `white_cane` | optical, length-and-color signature | yield with extra stand-off; do not approach silently |
| `service_animal` | optical, accompanying-animal pattern | maintain quiet operation; do not engage the animal |
| `wheelchair` | optical + range | adjust offered surfaces and voice height |
| `walker_or_aid` | optical + range, slow gait | extend stand-off and dwell tolerance |
| `child` | optical, size and gait estimation | tighter stand-off; conservative engagement |
| `companion_pair` | grouped tracks at consistent offset | treat as a unit; do not split between them |

Detection of these cues must operate without storing identifying biometric records. A person carrying a white cane is described as a track with the `white_cane` cue, not as an identified individual.

## 5. Venue Layer

The venue layer carries persistent annotations of the space: corridors, doorways, queue zones, no-entry zones, accessibility zones, quiet zones, exit routes. The layer is loaded from the venue's signed map and may be amended locally only with provenance and expiry.

The standard requires three normative venue labels:

- `accessibility_zone` — areas where extra stand-off and slower motion are required (priority parking for mobility aids, designated waiting areas for users with disabilities).
- `quiet_zone` — voice and audio output are suppressed below a declared threshold; the platform substitutes visual or screen-based output.
- `exit_route` — paths the platform must keep clear; the platform never positions itself blocking an exit route.

A platform that does not honor venue labels is non-conformant.

## 6. Sensor Fusion Discipline

### 6.1 Time Synchronization

All sensor inputs are timestamped at the source where possible. Drift beyond the declared envelope raises an alert; an out-of-sync sensor produces unreliable social context.

### 6.2 Spatial Calibration

Calibration is signed at commissioning. Field re-calibration requires a fresh signature and is itself an audit event. A camera that has been moved to look at a different area is not the same camera, for purposes of the manifest.

### 6.3 Lighting and Acoustic Envelopes

The platform declares the lighting and ambient-noise envelopes within which its perception is conformant. Operating outside the envelope is permitted only if the platform automatically derates: lower confidence on tracks, lower-tier interactions, more conservative stand-off.

## 7. Privacy in Perception

### 7.1 Local Inference Only

Person-track features (intent, attention, accessibility cues) are inferred locally and not transmitted to the cloud unless the platform's accountability model explicitly authorizes it. The default deployment model is local inference; cloud-augmented inference is opt-in and disclosed in the public-facing materials.

### 7.2 No Persistent Biometric Records

A service robot must not maintain persistent biometric records of individuals it has encountered. Track IDs are session-scoped and expire when the session ends. A platform that re-identifies a previously-seen individual across sessions is non-conformant unless the deployment is explicitly opted-in (a registered concierge service for hotel guests, with consent and disclosure).

### 7.3 Minor and Vulnerable Subject Protection

Where the perception layer detects a `child` cue or a vulnerability cue (e.g., apparent disorientation, distress), the platform must default to *less* engagement, not more, unless an authorized policy explicitly directs otherwise. The default protects vulnerable subjects from over-interaction by an autonomous agent.

## 8. Failure Modes

The perception layer transitions through declared modes:

| Mode | Trigger | L3 Restriction |
|------|---------|----------------|
| `nominal` | sensors healthy, model fresh | full interaction set |
| `degraded` | one or more sensors degraded | engagement actions require operator-style oversight or platform falls back to basic stand-off |
| `stale` | model_age exceeds budget | engagement blocked; platform proceeds to a known waypoint |
| `blind` | no metric range data | platform stops, signals, calls for help |

Mode transitions are audit events. Sustained `degraded` operation triggers a maintenance call; the deployment policy may not silently accept indefinite degradation.

## 9. Tracker Discipline

The tracker maintains tentative, confirmed, and dormant tracks with the same discipline as in WIA-ROB-SEC-001 Phase 2 §8: confirmation requires multi-frame agreement, IDs are opaque, identity re-binding across blanked zones is prohibited absent explicit policy.

A service robot's tracker is generally more conservative on confirmation than a security robot's. False-positive person tracks degrade the user experience (the platform addresses empty space); false-negative person tracks are safety hazards. The default favors slightly delayed confirmation over hasty action.

## 10. Conformance Tests

The platform's perception conformance is demonstrated by, at minimum:

- a calibration test producing the signed extrinsics in the manifest,
- a sensor-failure injection test producing each declared mode transition,
- a venue-label test demonstrating that the platform honors `quiet_zone`, `accessibility_zone`, and `exit_route`,
- an accessibility-cue test demonstrating detection of each defined cue at the declared minimum lighting,
- a privacy test demonstrating that inferred features are not persisted across sessions.

Test reports are signed by the commissioner and attached to the conformance manifest.

## 11. Worked Example — Lobby Greeter

A lobby greeter platform stands at a hotel lobby stand with a 0.6 m radius footprint. A guest approaches at 1.2 m/s. The tracker confirms a person track at 4 m, the intent classifier reports `approaching` with confidence 0.71, attention rises to 0.84 as the guest looks at the platform.

The platform queries the venue layer: the lobby is a public zone, no `quiet_zone` flag, no `accessibility_zone` flag. The accessibility-cue list is empty. The world-model freshness is `nominal`.

L3 receives this snapshot and consults its policy. The greeter policy fires I2 (acknowledge — soft welcome animation) at attention ≥ 0.7 with dwell ≥ 1 s, then I3 (offer — "Welcome, can I help you find your room or the elevators?") at attention ≥ 0.8 with dwell ≥ 2 s. The justification packet records the policy nodes that fired, the world-model snapshot at firing time, and the per-track confidences.

Replay: an auditor reading the audit log can produce the same I2 → I3 sequence from the snapshot and the policy file. If the replay does not match, the platform is non-conformant.

## 12. Cross-Modal Robustness

Service-robot perception is robust to single-sensor failure: a person track that depends on a single failing camera must not propagate as if confirmed. The L2 layer enforces a corroboration rule for engagement-relevant tracks: at least two healthy sensors must agree on a person track before L3 is permitted to fire I2 or above.

The corroboration rule is bypassed only under operator authority and only with audit. A platform that has degraded to a single working sensor may still operate in I0/I1 (passive presence and ambient signaling) but must not initiate engagement. The conservative rule defends both the user and the platform: a misidentified engagement is a service failure, and a service failure built on a single-sensor hallucination is unrecoverable in the audit log.

## 13. Sensor Health Telemetry

Each sensor exposes a health channel containing, at minimum, a freshness timestamp, a quality metric in [0, 1], a fault counter, and the originating channel identifier. The L2 layer aggregates these into the `sensor_health` map delivered to L3. The map is normative: L3 policies that depend on multi-sensor corroboration consult `sensor_health` to confirm that the corroborating sensor has been operating during the dwell window.

A sensor whose quality metric drops below the threshold for longer than the smoothing window transitions to `degraded`. A sensor with a fault counter above the cutoff transitions to `failed`. Both transitions are audit events. The map exposes transition timestamps so an auditor can correlate engagement decisions with sensor state at the moment of decision.

## 14. Acoustic Subsystem

Microphones are subject to additional discipline. The platform must:

- declare its microphone array geometry in the manifest,
- localize sound events into the metric grid where physically possible,
- distinguish ambient noise from directed speech,
- suppress recording in `quiet_zone` and `no-recording` regions at the platform layer, not by L3 policy alone.

A platform that streams raw audio to a cloud service for inference is non-conformant unless the deployment's accessibility-and-privacy disclosure explicitly authorizes it and the user is informed prior to engagement. The default is local inference; cloud inference is opt-in.
