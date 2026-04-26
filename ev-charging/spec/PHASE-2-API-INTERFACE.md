# WIA-AUTO-005 PHASE 2 — API Interface Specification

**Standard:** WIA-AUTO-005
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 4. Communication Protocols

### 4.1 OCPP (Open Charge Point Protocol)

**Version Support:**
- OCPP 1.6: Current standard
- OCPP 2.0.1: Latest version with ISO 15118 support

**Architecture:**
```
[Charging Station] ←→ [OCPP] ←→ [Central Management System]
```

**Core Functionality:**
```json
{
  "messageType": "StatusNotification",
  "connectorId": 1,
  "status": "Charging",
  "errorCode": "NoError",
  "timestamp": "2025-12-26T10:00:00Z"
}
```

**OCPP Operations:**
1. **Remote Commands**:
   - Start/Stop charging
   - Unlock connector
   - Reset station
   - Update firmware

2. **Monitoring**:
   - Real-time power metrics
   - Energy consumption
   - Fault detection
   - Connector status

3. **Smart Charging**:
   - Set charging profiles
   - Power limits
   - Schedule management

### 4.2 ISO 15118 (Plug & Charge)

**Features:**
- Automatic authentication via digital certificate
- Encrypted communication (TLS)
- Bidirectional communication
- V2G support

**Communication Stack:**
```
Application Layer: ISO 15118-2 (Messages)
Transport Layer: TCP/IP
Network Layer: IPv6
Data Link: HomePlug Green PHY / MCS
Physical: Power Line Communication (PLC) via charging cable
```

**Authentication Flow:**
```
1. Vehicle connects to EVSE
2. Physical connection verified
3. Digital certificate exchange
4. Payment contract validation
5. Charging parameters negotiated
6. Charging authorized and starts
```

**Message Types:**
```xml
<SessionSetupReq>
  <Header>
    <SessionID>A1B2C3D4</SessionID>
  </Header>
  <EVCCID>1A:2B:3C:4D:5E:6F</EVCCID>
</SessionSetupReq>

<ChargeParameterDiscoveryReq>
  <MaxEntriesSAScheduleTuple>3</MaxEntriesSAScheduleTuple>
  <RequestedEnergyTransferMode>DC_extended</RequestedEnergyTransferMode>
  <DC_EVChargeParameter>
    <DepartureTime>18000</DepartureTime>
    <DC_EVStatus>
      <EVRESSSOC>25</EVRESSSOC>
    </DC_EVStatus>
    <EVMaximumCurrentLimit>
      <Value>400</Value>
      <Unit>A</Unit>
    </EVMaximumCurrentLimit>
  </DC_EVChargeParameter>
</ChargeParameterDiscoveryReq>
```

### 4.3 IEC 61851 (Control Pilot)

**PWM Signal Specification:**
```
Frequency: 1 kHz ± 0.1 kHz
Voltage: +12V / -12V
Duty Cycle: 10-96%
```

**Duty Cycle to Current Mapping:**
```
10% ≤ D < 85%: I = (D / 100) × 0.6 × 1000 A
85% ≤ D ≤ 96%: I = ((D - 64) / 100) × 2.5 × 1000 A

Examples:
D = 30%: I = 18A
D = 50%: I = 30A
D = 80%: I = 48A
```

**State Machine:**
```
State A: Vehicle not connected (12V)
State B: Vehicle connected, not ready (9V)
State C: Vehicle connected, ready to charge (6V)
State D: Vehicle connected, charging with ventilation (3V)
State E: Short circuit detected (0V)
State F: EVSE not available (-12V)
```

---

## 5. Smart Charging

### 5.1 Load Management

**Objectives:**
- Prevent grid overload
- Optimize energy costs
- Balance power across multiple chargers
- Integrate renewable energy

**Dynamic Load Balancing:**
```
Available Power = Grid Limit - Building Load
Charger Power = Available Power / Number of Active Chargers

Example:
Grid Limit: 100 kW
Building Load: 30 kW
Active Chargers: 3
Charger Power: (100 - 30) / 3 = 23.3 kW each
```

**Priority Levels:**
```
1. Critical (Emergency vehicles, fleet): 100% power
2. High (Departing soon): 80% power
3. Medium (Standard): 60% power
4. Low (Overnight): 40% power
```

### 5.2 Demand Response

**Time-of-Use (TOU) Optimization:**
```
Off-Peak (23:00-07:00): $0.08/kWh → Charge at 100%
Mid-Peak (07:00-17:00): $0.15/kWh → Charge at 50%
On-Peak (17:00-23:00): $0.35/kWh → Charge at 0% (if possible)
```

**Scheduling Algorithm:**
```python
def optimize_charging(departure_time, current_soc, target_soc, battery_capacity):
    energy_needed = battery_capacity * (target_soc - current_soc)

    # Get TOU schedule
    rate_schedule = get_rate_schedule(now, departure_time)

    # Sort periods by price (cheapest first)
    sorted_periods = sort_by_price(rate_schedule)

    # Allocate charging to cheapest periods
    charging_schedule = []
    remaining_energy = energy_needed

    for period in sorted_periods:
        if remaining_energy <= 0:
            break

        available_time = period.duration
        max_energy = period.power * available_time

        allocated_energy = min(remaining_energy, max_energy)
        charging_schedule.append({
            'start': period.start,
            'end': period.end,
            'power': allocated_energy / available_time,
            'cost': allocated_energy * period.price
        })

        remaining_energy -= allocated_energy

    return charging_schedule
```

### 5.3 Solar Integration

**Solar + Storage + EV Charging:**
```
Priority Order:
1. Critical loads (building essential systems)
2. EV charging (from solar if available)
3. Battery storage charging
4. Export to grid (if net metering available)
```

**Solar Charging Algorithm:**
```
Available Solar = Solar Generation - Building Load

If Available Solar > 0:
    EV Charging Power = min(Available Solar, EV Max Power)
Else:
    If Battery SOC > 80%:
        EV Charging Power = Battery Discharge (up to EV Max Power)
    Else:
        EV Charging Power = Grid Power (limited by TOU strategy)
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
