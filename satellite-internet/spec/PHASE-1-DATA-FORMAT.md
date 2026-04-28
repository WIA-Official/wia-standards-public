# WIA-COMM-005 — Phase 1: Data Format

> Satellite-internet canonical envelopes: LEO/MEO/GEO constellation descriptors, inter-satellite link records, and the ground-station / user-terminal data shapes.

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for satellite internet systems, providing global broadband connectivity through space-based networks. It covers LEO constellations (like Starlink, OneWeb, Kuiper), MEO systems, and traditional GEO satellites.

### 1.2 Scope

The standard covers:
- Orbital architecture and constellation design
- Inter-satellite communication protocols
- Ground station and gateway infrastructure
- User terminal specifications
- RF link budgets and frequency planning
- Latency optimization techniques
- Handover and mobility management
- Regulatory compliance and debris mitigation

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to bridge the digital divide by providing universal internet access to remote, rural, maritime, and aviation environments, enabling global connectivity for all.

### 1.4 Terminology

- **LEO**: Low Earth Orbit (500-2000 km altitude)
- **MEO**: Medium Earth Orbit (2000-35,786 km altitude)
- **GEO**: Geostationary Earth Orbit (35,786 km altitude)
- **ISL**: Inter-Satellite Link (optical or RF links between satellites)
- **EIRP**: Effective Isotropic Radiated Power
- **G/T**: Gain-to-noise-temperature ratio
- **Ka-band**: 26.5-40 GHz frequency range
- **Ku-band**: 12-18 GHz frequency range
- **V-band**: 40-75 GHz frequency range

---


## 2. LEO Constellation Architecture

### 2.1 Orbital Parameters

LEO constellations operate at altitudes between 500-2000 km, providing low-latency global coverage.

**Typical LEO Configuration:**
```
Altitude: 550 km (Starlink), 600 km (OneWeb), 590 km (Kuiper)
Inclination: 53°, 70°, 87° (for different coverage patterns)
Orbital Planes: 20-80 planes
Satellites per Plane: 20-80 satellites
Total Satellites: 300-40,000+ satellites
Orbital Period: ~90-100 minutes
```

### 2.2 Coverage Patterns

**Polar Coverage:**
```
Inclination: 87-90°
Coverage: Entire globe including poles
Use Case: Global scientific, military, emergency
```

**Mid-Latitude Coverage:**
```
Inclination: 53-70°
Coverage: Optimized for population centers
Use Case: Commercial broadband services
```

### 2.3 Constellation Phasing

Satellites are distributed uniformly across orbital planes to ensure continuous coverage:

```
Phase Angle = 360° × (plane_number / total_planes)
Satellite Spacing = 360° / satellites_per_plane
```

**Example (Starlink Phase 1):**
```
72 orbital planes
22 satellites per plane
1,584 total satellites
Phase angle: 5° between planes
```

### 2.4 Visibility and Elevation Angles

Minimum elevation angle determines satellite visibility:

```
cos(θ) = R_e / (R_e + h)
```

Where:
- `θ` = Minimum elevation angle (typically 25-40°)
- `R_e` = Earth radius (6,371 km)
- `h` = Satellite altitude

**Coverage radius at minimum elevation:**
```
r = R_e × arccos[R_e × cos(θ) / (R_e + h)]
```

---


## 3. MEO and GEO Systems

### 3.1 MEO Architecture

**Orbit Parameters:**
```
Altitude: 8,000-20,000 km
Period: 6-12 hours
Satellites: 10-50 satellites
Latency: 100-150 ms round-trip
```

**Use Cases:**
- Navigation (GPS, Galileo, BeiDou)
- Regional broadband coverage
- Backup for LEO systems

### 3.2 GEO Architecture

**Orbit Parameters:**
```
Altitude: 35,786 km
Period: 23 hours 56 minutes (geosynchronous)
Satellites: 3-5 for global coverage
Latency: 500-600 ms round-trip
```

**Use Cases:**
- TV broadcast and DTH
- Maritime and aviation (legacy)
- Weather monitoring
- Emergency backup

### 3.3 Hybrid Architectures

Modern systems combine multiple orbit types:

```
LEO: Low-latency user access
MEO: Regional coverage fill
GEO: Broadcast and backup
```

---


## 4. Inter-Satellite Links (ISL)

### 4.1 Optical ISL

**Technology:**
```
Wavelength: 1550 nm (C-band optical)
Data Rate: 1-100 Gbps per link
Range: Up to 5,000 km
Beam Divergence: <10 microradians
```

**Advantages:**
- High bandwidth (10-100x RF)
- No frequency licensing required
- Minimal interference
- Secure (difficult to intercept)

**Link Budget:**
```
P_rx = P_tx + G_tx + G_rx - L_space - L_point
```

Where:
- `P_rx` = Received power (dBm)
- `P_tx` = Transmitted power (dBm)
- `G_tx`, `G_rx` = Antenna gains (dBi)
- `L_space` = Free-space loss
- `L_point` = Pointing loss

### 4.2 RF ISL

**Frequency Bands:**
```
Ka-band: 23-27 GHz
V-band: 60 GHz
E-band: 71-76 GHz
```

**Data Rates:**
```
Ka-band: 100 Mbps - 1 Gbps
V-band: 1-10 Gbps
```

### 4.3 Mesh Networking

ISL enables mesh topology for:
- Reduced ground station dependency
- Lower latency (space routing)
- Increased resilience
- Global coverage without ground infrastructure

**Routing Protocols:**
```
- OSPF-based (Open Shortest Path First)
- BGP for inter-constellation routing
- DTN (Delay-Tolerant Networking) for deep space
```

---



## A.1 Canonical envelope conventions

Every Phase 1 satellite-internet envelope follows the WIA family
baseline: UTF-8 JSON, RFC 8785 canonicalisation, Ed25519
signatures, ULID identifiers. Orbital elements use the standard
Two-Line Element (TLE) format reference for legacy compatibility
plus a modern OEM (Orbital Ephemeris Message, CCSDS 502.0-B) for
high-precision propagation.

## A.2 Constellation descriptor envelope

```json
{
  "wia_satinternet_version": "1.0.0",
  "type": "constellation_descriptor",
  "constellation_id": "const_01HX...",
  "tier": "leo" | "meo" | "geo",
  "satellite_count": 4408,
  "altitude_km": 550,
  "inclination_deg": 53.0,
  "expected_minimum_elevation_deg": 25.0,
  "first_launch_at": "RFC 3339"
}
```

## A.3 Inter-satellite link record

ISL envelopes carry the link endpoints, the optical wavelength,
the link rate (typically 100 Gbps for laser ISLs), and the link
quality estimate. The envelope is signed by the satellite operator
so downstream consumers can verify which constellation produced
the link.

## A.4 Ground-station and user-terminal records

Ground-station envelopes carry station identity, geographic
location, supported frequency bands, and operational state.
User-terminal envelopes carry terminal identity, owner, supported
frequency bands, and the last-known link metrics for performance
monitoring.


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/satellite-internet/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-satellite-internet-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/satellite-internet-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/satellite-internet.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.


## A.5 Worked LEO-constellation envelope

```json
{
  "wia_satinternet_version": "1.0.0",
  "type": "constellation_descriptor",
  "constellation_id": "starlink-shell-1",
  "tier": "leo",
  "satellite_count": 1584,
  "altitude_km": 550,
  "inclination_deg": 53.0,
  "expected_minimum_elevation_deg": 25.0,
  "isl_capable": true,
  "first_launch_at": "2019-11-11T12:56:00Z",
  "operator_id": "did:wia:satinternet:operator-A",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

A real-world constellation publishes one descriptor per orbital
shell; the consumer aggregates across shells when computing
end-to-end coverage at a location.


## A.6 ISL link-budget envelope detail

```json
{
  "wia_satinternet_version": "1.0.0",
  "type": "isl_link_budget",
  "link_id": "isl_01HX...",
  "satellites": ["sat-A", "sat-B"],
  "carrier": "1550nm-laser",
  "transmit_power_dbm": 35,
  "receiver_sensitivity_dbm": -40,
  "free_space_loss_db": 75,
  "atmospheric_loss_db": 0,
  "fade_margin_db": 8,
  "link_quality_db": 12,
  "rate_gbps": 100,
  "valid_until": "RFC 3339"
}
```

ISL envelopes refresh on a 1-minute cadence for active links;
operations dashboards subscribe to detect fading events.

## A.7 Glossary expansion

LEO: Low Earth Orbit, typically 500-2000km altitude. MEO: Medium
Earth Orbit, 2000-35000km. GEO: Geostationary Earth Orbit, ~35786km.
ISL: Inter-Satellite Link. TLE: Two-Line Element set (legacy
orbital description). OEM: Orbital Ephemeris Message (CCSDS
502.0-B). RTT: Round-Trip Time. Fade margin: link-budget headroom
above receiver sensitivity. Phased array: electronically-steerable
antenna with no mechanical movement.


## A.10 Operational case studies

Three deployment patterns appear repeatedly in production
satellite-internet operators:

**Pattern A — High-density urban**: high-elevation user terminals
on rooftops, dense ground-station network, primary use case is
backup connectivity for terrestrial-fibre outages. Capacity per
satellite is rarely a constraint; latency optimisation dominates.
The standards latency-optimisation endpoint informs route
selection.

**Pattern B — Rural broadband**: low-density terminals at user
homes, sparse ground-station network, primary use case is primary
connectivity in unfibred areas. Capacity per satellite is the
binding constraint; the constellations sizing decisions translate
directly into per-subscriber bandwidth.

**Pattern C — Mobility (maritime, aviation, RV)**: terminals
moving through the constellation; handover frequency is the
dominant operational concern. The handover-management protocol
must keep up with the terminal motion; the standards handover
budget is documented per operator.

## A.11 Performance benchmarking

Conformant operators publish performance benchmarks on a quarterly
cadence: median round-trip time per region, 95th-percentile RTT,
sustained throughput per terminal class, handover success rate,
fade-event MTBF (Mean Time Between Failures). The benchmarks feed
into customer choice and into regulatory reports.

## A.12 Audit and lawful-intercept compatibility

Jurisdictions requiring lawful intercept (US CALEA, EU under ETSI
ES 201 671, KR 통신비밀보호법) declare the requirement in the
discovery document. Sessions in those jurisdictions emit a notice
envelope on session start; the actual intercept happens through a
separate signed channel that audit-logs every access.

## A.13 Closing implementer note

Satellite-internet is one of the highest-stakes infrastructures in
modern society: it crosses every sovereign boundary, carries every
class of digital traffic, and serves customers in every regulatory
regime simultaneously. The standards wire-format discipline is what
makes operating in this environment safe and auditable.

A first deployment that follows the runbook reaches production in
about 90 days; the depth of audit and compliance work concentrated
in those 90 days is what justifies the discipline. Subsequent
deployments to additional jurisdictions reuse the same machinery
with per-jurisdiction policy overlays.


## A.14 Standards references summary

References used in this Phase: ISO 19156 (Observations and
Measurements), ISO 24113 (space debris mitigation), CCSDS 502.0-B
(Orbital Ephemeris Message), CCSDS 132.0-B (TM Space Data Link
Protocol), ITU-R Radio Regulations, IEEE 802.3df (data centre
links), 3GPP NTN specifications, IETF RFC 8446 (TLS 1.3), IETF
RFC 8785 (JSON Canonicalization).
