# WIA-COMM-005 — Phase 2: API

> Ground-station and user-terminal API surface, link-budget reporting, latency optimisation, and the formal endpoint set integrators code against.

## 5. Ground Station Architecture

### 5.1 Gateway Stations

**Specifications:**
```
Antenna Diameter: 3-13 meters
Frequency: Ka-band (27.5-30 GHz uplink, 17.7-20.2 GHz downlink)
Data Rate: 10-100 Gbps per gateway
Number Needed: 10-50 gateways globally
```

**Gateway Functions:**
- Internet backbone connectivity
- Traffic aggregation
- Network management
- Telemetry and control

### 5.2 Teleport Design

**Components:**
```
1. RF Equipment:
   - High-power amplifiers (HPA)
   - Low-noise amplifiers (LNA)
   - Frequency converters

2. Baseband Processing:
   - Modulation/demodulation (DVB-S2X, 5G NR)
   - Error correction (LDPC, Turbo codes)
   - Traffic shaping

3. Network Interface:
   - Fiber optic backbone (100 Gbps+)
   - CDN peering
   - Internet exchange points
```

### 5.3 Gateway Distribution

**Optimal Placement:**
```
- Near internet exchange points (IXP)
- Low latency to major data centers
- Diverse geographic distribution
- Regulatory-friendly jurisdictions
```

---


## 6. User Terminal Design

### 6.1 Phased-Array Antenna

**Specifications:**
```
Diameter: 30-60 cm (Starlink), 50-70 cm (OneWeb)
Elements: 1,000-2,000 antenna elements
Beam Steering: Electronic (no mechanical movement)
Scanning Range: ±60° from zenith
Polarization: Dual circular or linear
```

**Beam Steering Formula:**
```
θ = arcsin(λ × Δφ / 2πd)
```

Where:
- `θ` = Steering angle
- `λ` = Wavelength
- `Δφ` = Phase shift between elements
- `d` = Element spacing

### 6.2 Modem Specifications

**Physical Layer:**
```
Modulation: QPSK, 8PSK, 16APSK, 32APSK (DVB-S2X)
FEC: LDPC (Low-Density Parity-Check)
Code Rates: 1/4, 1/3, 2/5, 1/2, 3/5, 2/3, 3/4, 4/5, 5/6, 8/9, 9/10
Symbol Rate: 100-500 Msps
```

**Performance:**
```
Download: 100-500 Mbps
Upload: 20-50 Mbps
Latency: 20-40 ms (LEO)
```

### 6.3 Power Consumption

```
Idle: 20-50 W
Active: 50-150 W
Peak: 100-200 W
```

---


## 7. Link Budget and RF Design

### 7.1 Downlink Budget (Satellite to User)

```
P_rx = EIRP_sat - L_path - L_atm - L_rain + G_rx - L_rx
```

**Parameters:**
```
EIRP_sat: 50-60 dBW (satellite EIRP)
L_path: 20 log₁₀(4πd/λ) = 165-175 dB (free-space loss)
L_atm: 0.5-2 dB (atmospheric absorption)
L_rain: 1-10 dB (rain fade, depends on frequency and climate)
G_rx: 35-40 dBi (user terminal antenna gain)
L_rx: 1-2 dB (receiver losses)
```

**Received Power:**
```
P_rx = -90 to -110 dBm
```

**Signal-to-Noise Ratio:**
```
SNR = P_rx - N
N = kTB (thermal noise)
```

Where:
- `k` = Boltzmann constant (1.38 × 10⁻²³ J/K)
- `T` = System noise temperature (100-300 K)
- `B` = Bandwidth (Hz)

**Typical SNR:**
```
Clear sky: 15-25 dB
Rain fade: 5-15 dB
```

### 7.2 Uplink Budget (User to Satellite)

```
EIRP_user: 35-45 dBW (user terminal EIRP)
L_path: 165-175 dB
L_atm: 0.5-2 dB
L_rain: 1-10 dB
G_sat: 25-30 dBi (satellite antenna gain)
```

### 7.3 Adaptive Coding and Modulation (ACM)

```
Clear Sky: 32APSK, rate 9/10 → 500 Mbps
Light Rain: 16APSK, rate 3/4 → 300 Mbps
Heavy Rain: QPSK, rate 1/2 → 100 Mbps
```

---


## 8. Latency Optimization

### 8.1 Round-Trip Time (RTT)

**LEO Latency Components:**
```
Propagation delay: 2 × (550 km / 300,000 km/s) = 3.7 ms
Ground processing: 5-10 ms
Network routing: 5-15 ms
Total: 20-40 ms
```

**Comparison:**
```
LEO: 20-40 ms
MEO: 100-150 ms
GEO: 500-600 ms
Fiber (1000 km): 10-15 ms
```

### 8.2 Optimization Techniques

**1. Inter-Satellite Links:**
```
- Route traffic through space mesh
- Avoid multiple ground hops
- Reduces latency by 50-70%
```

**2. Edge Computing:**
```
- Place CDN nodes at gateways
- Cache popular content in space
- Reduce round-trip latency
```

**3. Protocol Optimization:**
```
- TCP BBR (Bottleneck Bandwidth and RTT)
- QUIC (Quick UDP Internet Connections)
- Custom congestion control
```

---



## A.1 Endpoint reference

```http
POST /satinternet/v1/terminal/register     # register a user terminal
GET  /satinternet/v1/coverage/{lat}/{lon}  # coverage at a location
POST /satinternet/v1/handover/request      # request a handover
GET  /satinternet/v1/link/{id}/budget       # link-budget telemetry
```

Every endpoint follows the discovery convention at
`/.well-known/wia-satellite-internet`.

## A.2 Coverage-prediction endpoint

The coverage endpoint accepts a geographic location and returns
the visible-satellite list with predicted elevation, predicted
link quality, and the next visibility window. This is the input
to user-terminal antenna pointing and to operational dashboards.

## A.3 Link-budget reporting

Link-budget envelopes carry the fundamental link parameters:
transmit power, antenna gain, free-space path loss, atmospheric
loss, rain fade margin, and noise figure. The endpoint is
high-frequency (typically every minute) for active links so
operations teams can watch for fading events.

## A.4 Latency-optimisation endpoint

The latency-optimisation endpoint returns the predicted minimum
round-trip time between two locations using the constellation's
ISL topology. It is the input to operational dashboards comparing
satellite-internet latency against terrestrial alternatives for
specific routes.


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


## A.5 Coverage-prediction worked example

```http
GET /satinternet/v1/coverage/37.5665/126.9780
Accept: application/json

→ 200 OK
{
  "wia_satinternet_version": "1.0.0",
  "location_iso6709": "+37.5665+126.9780/",
  "current_visible_satellites": 22,
  "predicted_minimum_visibility_seconds": 3600,
  "predicted_minimum_link_quality_db": 18.5,
  "next_visibility_window": null
}
```

Coverage envelopes are republished every minute for active service
areas; consumers cache for 60 seconds.

## A.6 Bulk-export endpoint

For audit and regulatory inspection, every host exposes a bulk
export with all link, handover, and conjunction-warning records in
a time window. The export carries a Merkle root commitment for
completeness verification.


## A.7 Operational rate-limits

Public read endpoints (coverage prediction, operator info) are
rate-limited per source IP at 60 req/min default; authenticated
endpoints (terminal control, handover request) are rate-limited
per terminal at 10 req/min. Limits are advisory and vary per
operator; the discovery document declares the actual limits.

## A.8 Webhook subscriptions

Long-running operations (handover sequences, fade events) emit
webhook callbacks on completion. The callback payload is the full
signed envelope with HMAC verification.

## A.9 Glossary expansion

Discovery: standardised endpoint for operator metadata at
`/.well-known/wia-satellite-internet`. Manifest: signed Merkle root
over a bulk-export bundle. Idempotency-Key: HTTP header per IETF
draft preventing duplicate writes on retry. Problem document: RFC
9457 structured error response format used uniformly across the
WIA family.


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
