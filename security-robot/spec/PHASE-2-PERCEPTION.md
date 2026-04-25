# WIA-ROB-SEC-001: Security Robot
## Phase 2 — Perception & World Model

**Standard ID:** WIA-ROB-SEC-001
**Category:** Robotics — Security
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2026-04-26

---

## 1. Purpose

Phase 2 specifies how raw sensor streams become the world model that Phase 3 reasons over. The standard does not prescribe a specific sensor suite, but it does prescribe the *shape* of what the perception layer must deliver, the freshness guarantees on that delivery, and the failure modes the platform must surface to higher layers.

## 2. Sensor Classes

A conformant platform declares its sensor suite in the conformance manifest. The standard recognizes five classes:

| Class | Examples | Primary Role |
|-------|---------|--------------|
| Optical | RGB cameras, IR cameras | object recognition, scene context |
| Range | LiDAR, structured-light, ToF | metric mapping, obstacle avoidance |
| Acoustic | microphones, ultrasonic | event detection, glass-break, voice presence |
| Inertial | IMU, wheel odometry | self-localization |
| Environmental | gas, temperature, smoke | facility-safety triggers |

A platform need not include every class, but its declared set determines the response policies it is allowed to participate in. A platform with no acoustic class cannot host a glass-break response policy; the deployment manager must reject such an assignment.

## 3. World Model Structure

### 3.1 Required Channels

The world model delivered to L3 must contain:

```
WorldModel := {
    metric_grid:     OccupancyGrid(local, ego-centered, ≤ N seconds old),
    tracks:          List<Track>,
    semantic_layer:  Map<RegionID, SemanticLabel>,
    self_pose:       Pose(stamped),
    sensor_health:   Map<SensorID, HealthStatus>,
    model_age:       Duration since last full update
}
```

Every field is timestamped against the platform clock. `model_age` is normative: if it exceeds the freshness budget declared in the manifest, the perception layer must mark the model `stale` and L3 must treat any escalation action as unauthorized while the flag is set.

### 3.2 Track Schema

```
Track := {
    id:           UUID (stable across update steps within a session),
    class:        Enum(person, vehicle, animal, unknown),
    pose:         Pose(stamped),
    velocity:     Velocity(stamped),
    confidence:   float in [0, 1],
    first_seen:   Timestamp,
    last_seen:    Timestamp,
    provenance:   List<SensorID>
}
```

Provenance is normative. A track produced by a single sensor cannot be presented as if it were corroborated. L3 policies for higher-stakes actions (siren, dispatch, supervisor handoff) typically require multi-sensor provenance.

### 3.3 Semantic Layer

The semantic layer carries persistent annotations of the environment: door, window, exit, restricted zone, public zone, no-recording zone. It is loaded from the site map and may be locally amended by the perception layer (for example, a door's open/closed state) but never silently. Any local amendment carries provenance and an expiry; an amendment that has not been re-corroborated by the expiry is dropped back to the loaded value.

## 4. Sensor Fusion Discipline

### 4.1 Time Synchronization

All sensor inputs must be timestamped at the source where physically possible. Where it is not (legacy USB cameras, network microphones), the fusion layer applies a one-time offset calibrated at startup and checks the drift continuously. Drift beyond the configured envelope raises a non-suppressible alert — a sensor whose clock has drifted is, for security purposes, a corrupted sensor.

### 4.2 Spatial Calibration

Calibration parameters (intrinsics, extrinsics) are signed at commissioning and stored in the conformance manifest. A sensor may not be re-calibrated in the field without a fresh signature and a fresh audit event. This deters an attacker from misaligning a camera to create blind spots.

### 4.3 Adversarial Inputs

The standard recognizes three families of adversarial input:

1. **Spoofing.** A laser projected at a camera, an ultrasonic playback at a microphone, a GPS spoofing source. Mitigation: cross-modal corroboration. A track that exists in optical alone, with no range-class confirmation, must not by itself trigger escalation. Manifest declares which modalities are "critical" for each escalation tier.
2. **Jamming.** Saturating a sensor with noise. Detection: each sensor exposes a per-frame noise/signal-quality metric; sustained degradation flags `sensor_health.degraded` to L3.
3. **Evasion.** Adversarial examples engineered to defeat a classifier. Mitigation: conservative thresholds, multi-frame agreement requirements, and the principle that an unrecognized agent in a restricted zone is not down-classified to "absent" — it remains "unknown agent present."

## 5. Privacy in Perception

The perception layer must support, at the layer level, a *no-recording zone* annotation in the semantic layer. While the platform is in such a zone, recording-class sensors (cameras, microphones) must operate in a blanked mode: detection signal still feeds the world model, but raw frames must not be persisted to the audit log.

Blanked mode is enforced by the platform, not by L3 policy alone. A conformant L1/L2 must physically truncate recording-class telemetry while in a no-record zone, and that truncation must be itself audited (a record exists that says "frames blanked from t1 to t2 in zone Z" without containing the frames themselves).

## 6. Failure Modes

The perception layer must transition through declared modes only. The four normative modes are:

| Mode | Trigger | L3 Restriction |
|------|---------|----------------|
| `nominal` | all sensors healthy, model fresh | full action set |
| `degraded` | one or more sensors `degraded`, model fresh | escalation actions require explicit operator confirmation |
| `stale` | `model_age` exceeds budget | escalation actions blocked; routine patrol blocked; platform proceeds to a known waypoint |
| `blind` | no metric range data available | platform stops, illuminates, calls operator |

Mode transitions are audit events. A platform that has been in `degraded` for longer than the configured threshold escalates to a maintenance call; the deployment policy may not silently accept indefinite degradation.

## 7. Conformance Tests

The platform's perception conformance is demonstrated by, at minimum:

- a calibration test producing the signed extrinsics in the manifest,
- a sensor-failure injection test producing each of `degraded`, `stale`, `blind`,
- a no-recording-zone test demonstrating frame truncation,
- a tracker continuity test under cross-modal handoff (a person walking from a camera-only zone into a camera+LiDAR zone retains track ID).

Test reports are signed by the commissioner and attached to the conformance manifest.

---

Phase 2 ends here. Phase 3 covers how the world model is consumed: patrol planning, anomaly response, and the policy graph.

## 8. Tracker Lifecycle

The tracker holds a population of tracks, each progressing through a small state machine:

```
            +---------+      +-----------+      +---------+
spawn ----> | tentative| ---> | confirmed | ---> | dormant |
            +---------+      +-----------+      +---------+
                |                  |                |
                +--- prune <-------+ <--------------+
```

A `tentative` track is one that has been observed below the confirmation cutoff and is held only briefly. Confirmation requires multi-frame agreement and, for higher-stakes downstream consumers, multi-modal corroboration. A `dormant` track is one whose evidence has aged past the freshness budget but which is held briefly to catch a re-entering subject. Pruned tracks are removed from the model and their IDs may be reused only after the configured cool-down.

Track IDs must be opaque to downstream consumers. The standard prohibits encoding identity into the ID.

## 9. Calibration Drift Detection

Calibration is checked online. The standard recognizes three indicators:

1. **Static-scene reprojection.** When the platform is stationary, scene features should reproject across modalities with sub-pixel agreement. Sustained disagreement above threshold indicates a calibration shift and triggers `degraded`.
2. **Self-consistency under motion.** A small motion that the IMU reports must be consistent with the apparent motion seen by range and optical sensors. Inconsistency triggers `degraded`.
3. **Light-level normalization.** Optical sensors must adjust to ambient light without clipping shadow regions; sustained clipping of large image fractions raises `degraded` and may, by policy, route the platform back toward better-lit waypoints.

Any of these triggers raise an audit event. A platform whose calibration drift is repaired by a re-calibration must, by the rules in Phase 1 §4, re-sign its calibration record.

## 10. Worked Example

Consider a platform patrolling an indoor corridor at 22:00 with the following declared suite: one RGB camera, one LiDAR (4-line, 25 m), one IMU, two microphones. A person enters the corridor. The tracker spawns a `tentative` track from the LiDAR cluster, corroborates with the camera within two frames, and confirms the track. The semantic layer reports the corridor as a `restricted_zone` between 22:00 and 06:00. The decision layer queries the world model with `track_present(person, restricted_zone, conf >= 0.7)`. The query is satisfied; the response policy fires its T1 `Observe` and increases the scan rate. The justification packet records the confidences, the corroborating sensors, and the policy node identifier. None of this requires escalation, but every step is auditable.

This example is normative for what a platform must be able to *demonstrate*: the chain of evidence from sensor frame to logged record, traceable end-to-end.

## 11. Cross-Modal Consistency Checks

A worked invariant: a person-class track at distance d in the metric grid must, given a calibrated optical sensor, project to a bounding box whose pixel area lies within a known envelope as a function of d. A track that violates the envelope — for example, a "person" at d = 8 m whose optical signature is two pixels wide — is rejected by the cross-modal consistency check and never delivered to L3 with `confirmed` status.

The envelope is part of the calibration record. A camera whose envelope drifts during operation is a calibration-drift indicator under §9 and triggers `degraded`.

## 12. Privacy-Preserving Inference

Where the deployment requires demographic disaggregation for the §3.3 audit (Phase 4), the inference is performed inside the L2 perception layer and the resulting category is not associated with the track ID in the persistent record. Only an aggregated count per response-tier reaches the audit log. This protects the privacy of any individual track while still permitting the deployment-level statistical audit.

A platform that exposes per-track demographic labels in its persisted records is non-conformant in privacy-regulated jurisdictions, even if the labels are correct. The point of the §3.3 audit is collective, not individual, and the persistence model must reflect that.

## 13. Sensor Health Telemetry

Each sensor exposes a health channel containing, at minimum, a freshness timestamp, a quality metric in [0, 1], a fault counter, and the originating channel identifier. The L2 layer aggregates these into the `sensor_health` map delivered to L3. The map is normative: L3 policies that depend on multi-sensor corroboration consult `sensor_health` to confirm that the corroborating sensor has been operating during the dwell window.

A sensor whose quality metric drops below the configured threshold for longer than the configured smoothing window transitions to `degraded`. A sensor with a fault counter above the configured cutoff transitions to `failed`. Both transitions are audit events. The `sensor_health` map exposes the transition timestamps so that an auditor can correlate response decisions with the sensor state at the moment of decision.

## 14. Tracker Identity Re-binding

When a track exits the world model and a new track enters within the configured re-binding window at a compatible kinematic state, the tracker may bind the new track to the prior track's identity. Re-binding is a probabilistic decision and must carry a confidence; the audit record reflects the re-bind decision and its evidence. A re-bind that turns out, in retrospect, to have stitched two distinct subjects into one identity is itself an incident worth surfacing — the tracker's re-bind log is part of the lifecycle audit.

The standard prohibits re-binding across `no-recording zone` boundaries when the inside-zone segment is blanked. A subject that disappears into a blanked zone re-emerges with a fresh identity unless the deployment policy explicitly permits cross-zone re-binding under a documented justification. The conservative default protects against an attacker exploiting blank zones to launder identity.
