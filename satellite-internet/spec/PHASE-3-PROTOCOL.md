# WIA-COMM-005 — Phase 3: Protocol

> Handover-management, frequency-coordination, orbital-mechanics, and debris-mitigation protocol layer. Every protocol exchange is wire-level with audit and replay defence.

## 9. Handover Management

### 9.1 Handover Prediction

Satellites are visible for 4-7 minutes in LEO. Handover must be predicted and executed seamlessly.

**Visibility Duration:**
```
t_visible = 2 × arccos(R_e / (R_e + h)) / ω
```

Where:
- `R_e` = Earth radius (6,371 km)
- `h` = Satellite altitude (550 km)
- `ω` = Satellite angular velocity (rad/s)

**Example (550 km LEO):**
```
t_visible ≈ 4-7 minutes (depending on elevation angle)
```

### 9.2 Handover Types

**1. Intra-Plane Handover:**
```
Frequency: Every 4-7 minutes
Trigger: Satellite elevation < threshold
Next Satellite: Same orbital plane
```

**2. Inter-Plane Handover:**
```
Frequency: Every 20-40 minutes
Trigger: Better SNR available
Next Satellite: Different orbital plane
```

**3. Gateway Handover:**
```
Frequency: Variable
Trigger: Traffic optimization, satellite switches gateways
Impact: Minimal (transparent to user)
```

### 9.3 Handover Protocol

```
1. Measurement Phase (30s before):
   - Monitor SNR of current satellite
   - Scan for next satellite
   - Measure Doppler shift

2. Decision Phase (10s before):
   - Select best next satellite
   - Reserve resources
   - Prepare beam steering

3. Execution Phase (seamless):
   - Switch antenna beam
   - Adjust frequency for Doppler
   - Maintain TCP connections (no packet loss)
```

---


## 10. Frequency Coordination

### 10.1 Frequency Bands

**Ka-band (Primary for LEO):**
```
Uplink (User → Satellite): 27.5-30.0 GHz
Downlink (Satellite → User): 17.7-20.2 GHz
Gateway Uplink: 27.5-30.0 GHz
Gateway Downlink: 17.7-20.2 GHz
```

**Ku-band (Legacy, GEO):**
```
Uplink: 12.75-13.25 GHz, 13.75-14.5 GHz
Downlink: 10.7-11.7 GHz, 11.7-12.75 GHz
```

**V-band (Future High-Throughput):**
```
Uplink: 47.2-50.2 GHz
Downlink: 37.5-42.5 GHz
Bandwidth: Up to 10 GHz total
```

### 10.2 ITU Coordination

**Filing Requirements:**
```
1. Advance Publication: 7 years before launch
2. Coordination Request: 5 years before launch
3. Notification: Within 30 days of satellite operation
4. Frequency Assignment: Record in ITU Master Register
```

### 10.3 Interference Mitigation

**Techniques:**
```
1. Frequency Reuse:
   - Orthogonal polarizations (V/H or LHCP/RHCP)
   - Spot beams with frequency reuse
   - Geographic isolation

2. Dynamic Spectrum Management:
   - Cognitive radio techniques
   - Real-time interference monitoring
   - Adaptive power control

3. Coordination with GEO:
   - Off-axis EIRP limits
   - Exclusion zones (±2° of GEO arc)
   - Time-division sharing
```

---


## 11. Orbital Mechanics

### 11.1 Orbital Velocity

```
v = √(GM / r)
```

Where:
- `G` = Gravitational constant (6.674 × 10⁻¹¹ m³/kg·s²)
- `M` = Earth mass (5.972 × 10²⁴ kg)
- `r` = Orbital radius (R_e + h)

**Examples:**
```
LEO (550 km): v = 7.59 km/s
MEO (8000 km): v = 4.84 km/s
GEO (35,786 km): v = 3.07 km/s
```

### 11.2 Orbital Period

```
T = 2π√(r³ / GM)
```

**Examples:**
```
LEO (550 km): T = 95.6 minutes
MEO (8000 km): T = 6.0 hours
GEO (35,786 km): T = 23.93 hours
```

### 11.3 Doppler Shift

```
Δf = (v / c) × f₀ × cos(θ)
```

Where:
- `v` = Satellite velocity (7.59 km/s for 550 km LEO)
- `c` = Speed of light (299,792 km/s)
- `f₀` = Carrier frequency (28 GHz)
- `θ` = Angle between velocity vector and user

**Maximum Doppler (28 GHz):**
```
Δf_max = (7590 / 299792) × 28 × 10⁹ = ±710 kHz
```

**Doppler Rate:**
```
df/dt ≈ 10-50 kHz/s (requires continuous tracking)
```

---


## 12. Space Debris Mitigation

### 12.1 End-of-Life Disposal

**25-Year Rule (LEO):**
```
Satellites must deorbit within 25 years after end-of-mission
```

**Deorbit Strategies:**
```
1. Active Deorbit (LEO):
   - Use remaining propellant
   - Controlled reentry
   - Timeline: 1-12 months

2. Passive Deorbit (LEO < 600 km):
   - Atmospheric drag
   - Timeline: 1-5 years

3. Graveyard Orbit (GEO):
   - Raise orbit by 300+ km
   - Remove from GEO belt
```

### 12.2 Collision Avoidance

**Conjunction Analysis:**
```
- Daily screening against catalog (TLE data)
- Alert threshold: Probability of collision > 10⁻⁴
- Maneuver threshold: Miss distance < 1 km
```

**Maneuver Strategies:**
```
ΔV = √(2μ/r) × Δh / (2h)
```

Where:
- `μ` = Earth gravitational parameter
- `r` = Orbital radius
- `Δh` = Altitude change (typically 500-1000 m)

### 12.3 Design for Demise

```
1. Material Selection:
   - Avoid high-melting-point materials (titanium, inconel)
   - Use aluminum, composites

2. Component Sizing:
   - Limit component mass to prevent ground impact
   - Target: < 5 kg surviving components

3. Propellant Passivation:
   - Deplete propellant tanks before deorbit
   - Prevent on-orbit explosions
```

---



## A.1 Handover-management protocol

LEO constellations require continuous handover as satellites pass
overhead. The handover protocol exchanges signed envelopes between
the user terminal, the source satellite, the destination satellite,
and the constellation controller. Handover events are logged to
the audit chain so an operations-team review can reconstruct any
performance anomaly.

## A.2 Frequency-coordination protocol

Frequency coordination across operators (Starlink, Kuiper,
OneWeb, etc.) follows ITU-R conventions. The protocol envelopes
carry the operator identity, the requested frequency band, the
geographic footprint, and the time window. ITU-R coordinates the
allocations; the envelope shape is the wire format the operators
exchange with each other and with ITU-R.

## A.3 Orbital-mechanics and debris-mitigation

Conjunction-warning envelopes carry the predicted close-approach
parameters between two satellites or between a satellite and a
debris object. Standard 96-bit nonce + 300-second skew window +
600-second seen-nonce cache applies; conjunction warnings are
the highest-priority traffic class in the standard.

## A.4 Audit log replication

Audit envelopes are replicated across at least two storage backends
with retention sized to the regulatory window: typically 5 years
for spectrum-coordination records, 10 years for safety-related
conjunction-warning records.


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


## A.5 Worked handover trace

```
T+0    Terminal sees Sat-A above 25° elevation; link active.
T+90s  Terminal sees Sat-A descending; predicted handover within 30s.
T+105s Terminal sends handover-request envelope to constellation
       controller naming Sat-A (source) and Sat-B (predicted destination).
T+106s Controller acks; Sat-A and Sat-B receive coordinated handover
       envelopes with synchronised timestamps.
T+108s Terminal switches its phased array to Sat-B; link continues.
T+109s Sat-A confirms link release; Sat-B confirms link establishment.
       Audit envelopes flow to operations dashboard.
```

End-to-end handover latency target: ≤ 5 seconds. Handover events
exceeding the target emit a warning envelope to operations.

## A.6 Federation across operators

Cross-operator coordination (Starlink ↔ Kuiper ↔ OneWeb) uses the
WIA-SOCIAL Phase 3 §5 federation envelope. Trust lists carry the
peer operators identity, the spectrum-coordination scope, and the
notice cadence for coverage changes affecting peers.


## A.7 Cross-jurisdictional deployment

Satellite-internet operators face per-jurisdiction regulatory
frameworks: FCC (US), Ofcom (UK), MIC (Japan), KCC (Korea), EU
member-state regulators, and ITU-R for spectrum coordination. The
protocol envelopes carry the operator's licensed-jurisdiction set;
consumers verify their requested service falls within the permitted
jurisdictions.

## A.8 Closing protocol note

Satellite-internet protocol exchanges balance regulatory rigour
(spectrum coordination, licensing, debris mitigation) against
operational responsiveness (handover, fade compensation). The
discipline favours rigour at the audit layer while the
operational layer enables sub-5-second handover and minute-scale
spectrum coordination.


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
