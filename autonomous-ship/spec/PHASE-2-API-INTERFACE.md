# WIA-AUTO-015 PHASE 2 — API Interface Specification

**Standard:** WIA-AUTO-015
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Remote Operation Centers

### 5.1 ROC Architecture

#### 5.1.1 Components

```
ROC System Components:

1. Control Stations (redundant)
   - Primary operator station
   - Supervisor station
   - Emergency override station

2. Display Systems
   - Multi-screen navigation displays
   - Sensor feed monitors
   - System status dashboard
   - Communication interfaces

3. Communication Infrastructure
   - Satellite (primary): VSAT, Iridium
   - Cellular (coastal): 4G/5G
   - VHF radio (backup)
   - Emergency beacon

4. Decision Support
   - AI recommendation engine
   - Voyage optimization
   - Weather routing
   - Predictive maintenance
```

#### 5.1.2 Operator Workload

```
Max vessels per operator:

N_max = floor(
    operator_capacity / Σ[vessels] attention_required(vessel)
)

attention_required(vessel) = base_attention × (
    1 + complexity_factor +
    weather_factor +
    traffic_factor +
    system_health_factor
)

Typical values:
  - Open ocean, good conditions: 4-6 vessels per operator
  - Coastal waters: 2-3 vessels per operator
  - Port approach: 1 vessel per operator
```

### 5.2 Communication Protocols

#### 5.2.1 Data Transmission Priorities

```
Priority Queue:

Priority 0 (CRITICAL - immediate):
  - Collision warnings
  - System failures
  - Emergency situations

Priority 1 (HIGH - <1 second):
  - Navigation commands
  - Sensor data
  - Status updates

Priority 2 (MEDIUM - <5 seconds):
  - Video streams
  - Non-critical alerts
  - Performance data

Priority 3 (LOW - <30 seconds):
  - Logs
  - Analytics
  - Diagnostics
```

#### 5.2.2 Bandwidth Management

```
Minimum bandwidth requirements:

- Critical data: 10 kbps (always available)
- Navigation control: 100 kbps
- Video (compressed): 500 kbps per camera
- Full telemetry: 1 Mbps
- HD video: 5 Mbps (optional, good conditions)

Adaptive transmission:

IF bandwidth < required:
    REDUCE video_quality
    INCREASE compression_ratio
    SEND critical_data_only
    ALERT operator
```

### 5.3 Handover Procedures

#### 5.3.1 Shift Handover

```
Shift Handover Protocol:

1. Briefing Phase (15 minutes before)
   - Review vessel status
   - Discuss ongoing situations
   - Identify potential issues

2. Dual Operation (5 minutes)
   - Outgoing operator maintains control
   - Incoming operator observes
   - Questions and clarifications

3. Control Transfer
   - Incoming operator acknowledges readiness
   - Outgoing operator transfers control
   - System logs transfer
   - Confirmation to all vessels

4. Verification (5 minutes after)
   - New operator confirms all systems
   - Reviews pending actions
   - Outgoing operator available for questions
```

---

## 6. Sensor Systems

### 6.1 Radar

#### 6.1.1 X-Band Radar (9 GHz)

**Characteristics:**
- Range: 0.5 - 48 NM
- Resolution: High (better target discrimination)
- Weather: Affected by rain/fog
- Use: Navigation, collision avoidance

**ARPA Functions:**
```
Target Tracking:

1. Detection
   threshold = noise_floor + 3σ

2. Association
   track(t+1) = nearest_target_within_gate(track(t))

3. Filtering (Kalman Filter)
   x̂(t|t) = x̂(t|t-1) + K(t) × [z(t) - H × x̂(t|t-1)]

   Where:
     x̂ = state estimate [position, velocity]
     z = measurement
     K = Kalman gain
     H = observation matrix

4. Prediction
   CPA, TCPA = calculate_from_state(x̂)
```

#### 6.1.2 S-Band Radar (3 GHz)

**Characteristics:**
- Range: 1 - 96 NM
- Resolution: Lower than X-band
- Weather: Better performance in precipitation
- Use: Long-range detection, weather monitoring

### 6.2 LiDAR

#### 6.2.1 Applications

- Near-field obstacle detection (< 500m)
- Port approach and docking
- 3D environmental mapping
- Precision navigation in restricted waters

**Point Cloud Processing:**
```
1. Segmentation
   clusters = DBSCAN(point_cloud, eps=0.5m, min_points=10)

2. Object Classification
   FOR EACH cluster:
       features = extract_features(cluster)
       class = classifier.predict(features)
       # Classes: vessel, buoy, structure, debris, etc.

3. Tracking
   tracked_objects = associate_with_previous_scan(clusters)

4. Collision Prediction
   FOR EACH object IN tracked_objects:
       IF predicted_collision(object):
           ALERT operator
           RECOMMEND avoidance_maneuver(object)
```

### 6.3 Cameras

#### 6.3.1 Visual Spectrum Cameras

- 360° coverage with multiple cameras
- Minimum 4K resolution
- Low-light sensitivity
- Real-time object detection

**Computer Vision Pipeline:**
```
1. Image Acquisition
   images = capture_from_all_cameras(t)

2. Preprocessing
   images_enhanced = adaptive_histogram_equalization(images)

3. Object Detection (YOLO/SSD)
   detections = model.detect(images_enhanced)
   # Detect: vessels, buoys, landmarks, navigation marks

4. Tracking (SORT/DeepSORT)
   tracks = tracker.update(detections)

5. Scene Understanding
   situation = analyze_scene(tracks, ais_data, radar_data)
```

#### 6.3.2 Thermal (IR) Cameras

- Night operation
- Fog penetration
- Man-overboard detection
- Temperature monitoring (engine, cargo)

### 6.4 AIS (Automatic Identification System)

#### 6.4.1 Message Types

```
AIS Message Processing:

Message 1/2/3 (Position Report):
  - MMSI, position, course, speed, heading
  - Update rate: 2-10 seconds (underway)

Message 5 (Static Data):
  - Ship name, call sign, IMO number
  - Dimensions, ship type
  - Update rate: 6 minutes

Message 21 (Aid-to-Navigation):
  - Buoy, lighthouse positions

Message 14 (Safety Message):
  - Text broadcast to all vessels
```

#### 6.4.2 Sensor Fusion

Combine AIS with radar for enhanced tracking:

```
Fusion Algorithm:

1. Data Association
   FOR EACH radar_track:
       ais_match = find_nearest_ais(radar_track, max_distance=0.5NM)

2. State Estimation
   IF ais_match EXISTS:
       # Fused track with higher confidence
       position = weighted_average(radar_pos, ais_pos, weights)
       velocity = ais_velocity  # More accurate from AIS
       identity = ais_identity
       confidence = 0.95
   ELSE:
       # Radar-only track (may be vessel without AIS)
       use_radar_data()
       confidence = 0.6

3. Uncertainty Quantification
   position_uncertainty = √(σ²_radar + σ²_ais)
   velocity_uncertainty = σ_ais  # AIS velocity generally reliable
```

### 6.5 Environmental Sensors

#### 6.5.1 Weather Monitoring

```
Sensor Suite:
- Anemometer: Wind speed/direction (±0.1 m/s, ±1°)
- Barometer: Atmospheric pressure (±0.1 hPa)
- Hygrometer: Humidity (±2%)
- Thermometer: Air/water temperature (±0.1°C)
- Wave sensor: Wave height/period (±0.1m, ±0.5s)

Data Integration:
weather_state = {
    wind: {speed, direction, gusts},
    pressure: {value, trend},
    visibility: computed_from_cameras,
    sea_state: {wave_height, period, direction},
    precipitation: from_radar_returns
}

Route Adjustment:
IF weather_state.wind.speed > threshold OR
   weather_state.sea_state.wave_height > threshold:
    new_route = optimize_route_for_weather(current_route, weather_state)
    IF deviation > acceptable_limit:
        REQUEST shore_approval
```

---

## 7. Route Planning and Optimization

### 7.1 Multi-Objective Optimization

#### 7.1.1 Objective Function

```
Cost Function:

J = α₁ × fuel_cost +
    α₂ × time_cost +
    α₃ × risk_cost +
    α₄ × emission_cost +
    α₅ × maintenance_cost

Where:
  α₁, α₂, α₃, α₄, α₅ = weighting factors (Σα = 1)

Fuel Cost:
  fuel_cost = ∫[route] (fuel_rate(speed, weather) × distance)

Time Cost:
  time_cost = ∫[route] (1 / speed) dx × time_value

Risk Cost:
  risk_cost = Σ[segments] (
      collision_risk × severity +
      grounding_risk × severity +
      piracy_risk × severity
  )

Emission Cost:
  emission_cost = ∫[route] (
      CO₂_rate + NOₓ_rate + SOₓ_rate
  ) × carbon_price
```

#### 7.1.2 Constraints

```
Optimization Constraints:

1. Navigational Safety:
   depth(x, y) ≥ draft + UKC(speed)
   distance_to_hazards ≥ safety_margin

2. Weather Limits:
   wave_height ≤ max_operational_wave_height
   wind_speed ≤ max_operational_wind_speed

3. Traffic Separation Schemes:
   IF in_TSS:
       course = TSS_direction ± 5°

4. Emission Control Areas (ECA):
   IF in_ECA:
       fuel_sulfur_content ≤ 0.1%  # IMO 2020

5. Time Windows:
   arrival_time ∈ [ETA_min, ETA_max]

6. Ship Performance:
   speed ∈ [min_steerage_speed, max_service_speed]
```

### 7.2 Weather Routing

#### 7.2.1 Wave Impact on Speed

```
Speed loss due to waves:

V_actual = V_calm_water × (1 - loss_factor)

loss_factor = f(
    wave_height,
    wave_period,
    wave_direction_relative_to_heading,
    ship_characteristics
)

Simplified model:
loss_factor = k₁ × (H_significant / L_ship)² ×
              |cos(θ_wave - θ_ship)|

Where:
  H_significant = significant wave height
  L_ship = ship length
  θ_wave = wave direction
  θ_ship = ship heading
  k₁ = empirical constant (typically 10-20)
```

#### 7.2.2 Dynamic Route Optimization

```
Re-planning Triggers:

1. Weather Update:
   IF |weather_forecast_new - weather_forecast_old| > threshold:
       recompute_optimal_route()

2. Periodic Review:
   EVERY 6 hours:
       IF potential_improvement > 5%:
           recompute_optimal_route()

3. Unexpected Conditions:
   IF actual_conditions != forecast_conditions:
       immediate_route_adjustment()

Re-planning Algorithm:

new_route = A_star_search(
    start = current_position,
    goal = destination,
    heuristic = great_circle_distance,
    cost = total_voyage_cost(segment, weather_forecast),
    constraints = safety_constraints
)

IF new_route.cost < current_route.cost × 0.95:
    REQUEST approval_from_shore()
    IF approved:
        UPDATE route
        NOTIFY all_systems
```

### 7.3 Port Approach and Docking

#### 7.3.1 Precision Navigation


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
