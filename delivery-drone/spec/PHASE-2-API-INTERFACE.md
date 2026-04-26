# WIA-AUTO-017 PHASE 2 — API Interface Specification

**Standard:** WIA-AUTO-017
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 4. Navigation and Path Planning

### 4.1 Route Planning

#### 4.1.1 A* Path Planning Algorithm

```python
def astar_path(start, goal, obstacles):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while not open_set.empty():
        current = open_set.get()[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current):
            if is_obstacle(neighbor, obstacles):
                continue

            tentative_g = g_score[current] + distance(current, neighbor)

            if tentative_g < g_score.get(neighbor, infinity):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                open_set.put((f_score[neighbor], neighbor))

    return None  # No path found
```

#### 4.1.2 Heuristic Function

Euclidean distance with altitude penalty:

```
h(n) = √[(xg - xn)² + (yg - yn)² + α(zg - zn)²]
```

Where:
- `α` = Altitude weight factor (typically 1.5)
- `(xg, yg, zg)` = Goal coordinates
- `(xn, yn, zn)` = Current node coordinates

### 4.2 Obstacle Avoidance

#### 4.2.1 LiDAR-based Detection

Process point cloud data to detect obstacles:

```
1. Segment point cloud into voxels (0.5m³)
2. Identify occupied voxels
3. Cluster nearby voxels into obstacles
4. Calculate obstacle bounding boxes
5. Plan avoidance trajectory
```

#### 4.2.2 Dynamic Window Approach

Select optimal velocity considering:

```
G(v, ω) = α × heading(v, ω) + β × clearance(v, ω) + γ × velocity(v, ω)
```

Where:
- `v` = Linear velocity
- `ω` = Angular velocity
- `α, β, γ` = Weighting factors
- `heading()` = Progress toward goal
- `clearance()` = Distance to obstacles
- `velocity()` = Forward velocity

### 4.3 Precision Landing

#### 4.3.1 Visual Servoing

Use AprilTag markers for precision landing:

```
Landing Sequence:
1. Detect landing pad from 10m altitude
2. Center above pad using visual feedback
3. Descend at 0.3 m/s
4. Fine-tune position at 3m altitude
5. Final descent at 0.1 m/s
6. Ground contact detection via IMU
7. Motor shutdown
```

#### 4.3.2 Position Error Correction

```
Position Error: ep = √[(xp - x)² + (yp - y)²]
Acceptable Error: ep < 0.2m (before landing)
```

---

## 5. Payload Management

### 5.1 Weight and Balance

#### 5.1.1 Center of Gravity

CG must remain within acceptable limits:

```
CGx = Σ(mi × xi) / mtotal
CGy = Σ(mi × yi) / mtotal
CGz = Σ(mi × zi) / mtotal
```

Acceptable CG range:
```
|CGx| < 0.05m
|CGy| < 0.05m
```

#### 5.1.2 Payload Capacity

Maximum payload calculation:

```
Wpmax = (Tmax × SF - Wd × g) / g
```

Where:
- `Wpmax` = Maximum payload weight (kg)
- `Tmax` = Maximum total thrust (N)
- `SF` = Safety factor (0.6-0.7)
- `Wd` = Drone dry weight (kg)
- `g` = Gravitational acceleration (9.81 m/s²)

### 5.2 Secure Attachment

Requirements:
- Positive locking mechanism
- Load sensors for confirmation
- Anti-sway stabilization
- Quick-release for delivery
- Tamper detection

### 5.3 Delivery Mechanisms

#### 5.3.1 Landing Delivery
```
1. Land at delivery location
2. Unlock payload
3. Visual/audio confirmation
4. Wait for package removal (max 30s)
5. Takeoff and return
```

#### 5.3.2 Winch Delivery
```
1. Hover at safe altitude (3-5m)
2. Lower package via motorized winch
3. Detect ground contact
4. Release package
5. Retract winch
6. Depart
```

#### 5.3.3 Drop Delivery
```
1. Position over drop zone
2. Reduce altitude to 2m
3. Release package to cushioned receptacle
4. Confirm delivery via camera
5. Depart
```

---

## 6. UTM Integration

### 6.1 UTM Architecture

Drones must integrate with UTM systems following standards:

```
┌─────────────┐
│   Drone     │
│   Operator  │
└──────┬──────┘
       │
       ↓
┌──────────────┐      ┌─────────────┐
│ UTM Service  │←────→│    ANSP     │
│   Provider   │      │   (ATC)     │
└──────┬───────┘      └─────────────┘
       │
       ↓
┌──────────────┐
│  Airspace    │
│ Authorization│
└──────────────┘
```

### 6.2 Required UTM Messages

#### 6.2.1 Flight Plan Submission

```json
{
  "operation_id": "OP-20250101-1234",
  "operator_id": "WIA-OP-001",
  "drone_id": "WIA-DRN-X1-0042",
  "flight_plan": {
    "departure": {
      "location": {"lat": 37.7749, "lng": -122.4194, "alt": 10},
      "time": "2025-01-01T10:00:00Z"
    },
    "arrival": {
      "location": {"lat": 37.7849, "lng": -122.4094, "alt": 0},
      "time": "2025-01-01T10:15:00Z"
    },
    "waypoints": [...],
    "max_altitude": 120,
    "max_speed": 20
  }
}
```

#### 6.2.2 Position Reports

Send every 1 second during flight:

```json
{
  "drone_id": "WIA-DRN-X1-0042",
  "timestamp": "2025-01-01T10:05:30Z",
  "position": {
    "lat": 37.7799,
    "lng": -122.4144,
    "alt_msl": 125,
    "alt_agl": 35
  },
  "velocity": {
    "vx": 12.5,
    "vy": 3.2,
    "vz": -0.5
  },
  "heading": 045,
  "battery": 78,
  "status": "enroute"
}
```

### 6.3 Conflict Detection

UTM must detect and resolve conflicts:

```
Separation Requirements:
- Horizontal: 50m minimum
- Vertical: 30m minimum

Conflict Alert:
If predicted separation < requirements within 60s,
issue conflict alert and recommend avoidance maneuver.
```

---


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
