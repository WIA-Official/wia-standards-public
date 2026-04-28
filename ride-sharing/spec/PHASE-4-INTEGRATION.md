# WIA-AUTO-014 — Phase 4: Integration

> References and worked-example appendix that anchor the entire specification.

## 12. References

### 12.1 Related Standards

- **WIA-INTENT**: Intent-based ride booking
- **WIA-OMNI-API**: Universal API gateway
- **WIA-PAYMENT**: Payment processing standard
- **WIA-IDENTITY**: Identity verification standard
- **WIA-LOCATION**: Location services standard
- **WIA-SOCIAL**: Social features and sharing

### 12.2 External References

1. **Transportation Research**: "Shared Mobility: Current Practices and Guiding Principles" (FHWA, 2016)
2. **Dynamic Pricing**: ISO/IEC 19770 series — Software Asset Management baseline conventions for usage-based pricing models
3. **Route Optimization**: OGC GeoPackage and ISO 19156 Observations and Measurements for time-windowed routing inputs
4. **Safety Standards**: "ISO 39001:2012 Road traffic safety management systems"
5. **Data Privacy**: "GDPR Compliance for Ride-Sharing Platforms" (EU, 2018)

### 12.3 Industry Best Practices

- **NHTSA**: Automated Vehicle Guidelines
- **PCI DSS**: Payment Card Industry Data Security Standard
- **WCAG 2.1**: Web Content Accessibility Guidelines
- **ISO 27001**: Information Security Management

---


## Appendix A: Example Calculations

### A.1 Fare Calculation Example

```
Trip details:
- Distance: 15.3 km
- Duration: 25 minutes
- Vehicle: Sedan
- Time: Evening rush hour (6:30 PM)
- Demand/Supply ratio: 2.5

Calculation:
1. Base fare:
   B_base = $3.50
   D × R_distance = 15.3 × $1.20 = $18.36
   T × R_time = 25 × $0.35 = $8.75
   B = $3.50 + $18.36 + $8.75 = $30.61

2. Surge multiplier:
   S = max(0, min(3.0, 1.0 × ln(2.5)))
   S = min(3.0, 0.916) = 0.916
   S_rounded = 1.0 (no surge)

3. Time coefficient:
   P_peak = 0.15 (evening rush)
   T_coef = 0.15

4. Final price:
   P = $30.61 × (1 + 0) × (1 + 0) × (1 + 0.15)
   P = $30.61 × 1.15 = $35.20

5. Fare range:
   Fare_min = $35.20 × 0.85 = $29.92
   Fare_max = $35.20 × 1.15 = $40.48

Display: $30 - $40 (estimated $35)
```

### A.2 Matching Score Example

```
Scenario:
- Rider at (37.7749, -122.4194)
- Driver A: 2.5 km away, rating 4.9, ETA 5 min
- Driver B: 5.0 km away, rating 4.5, ETA 10 min

Driver A:
D(A,r) = 1 - (2.5 / 10) = 0.75
T(A,r) = 1 - (5 / 15) = 0.67
R(A) = (4.9 - 3.0) / (5.0 - 3.0) = 0.95
V(A,r) = 1.0 (vehicle matches)

M(A,r) = 0.35×0.75 + 0.25×0.67 + 0.20×0.95 + 0.10×0.8 + 0.10×1.0
       = 0.2625 + 0.1675 + 0.19 + 0.08 + 0.10
       = 0.80

Driver B:
D(B,r) = 1 - (5.0 / 10) = 0.50
T(B,r) = 1 - (10 / 15) = 0.33
R(B) = (4.5 - 3.0) / (5.0 - 3.0) = 0.75
V(B,r) = 1.0

M(B,r) = 0.35×0.50 + 0.25×0.33 + 0.20×0.75 + 0.10×0.8 + 0.10×1.0
       = 0.175 + 0.0825 + 0.15 + 0.08 + 0.10
       = 0.5875

Result: Driver A matched (higher score)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-014 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*


## A.1 Privacy and security integration

Ride-sharing data is highly sensitive: location traces, identity,
payment, and behavioural patterns. The standard requires:

- **Location privacy**: pickup and dropoff are encrypted at rest;
  only the matched rider/driver pair sees the unencrypted
  endpoints during the ride window
- **Identity verification**: WIA-OMNI-API credentials with
  third-party verification (DMV, ID document scan)
- **Payment isolation**: PCI-DSS-compliant tokenisation
- **Lawful intercept compatibility**: declared in discovery document
  per jurisdiction; envelopes carry the lawful-intercept attestation
  when applicable

## A.2 Regulatory integration

Ride-sharing is regulated jurisdiction-by-jurisdiction. The
standard's discovery document declares the operator's licensed
jurisdictions and the regulatory mode applied (full TNC licence,
limited livery licence, etc.). Envelopes carry the jurisdiction
identifier so the regulator can subscribe to relevant audit feeds.

## A.3 Bridges to existing platforms

The bridge profile maps to: SAE J3216 (Cooperative Driving
Automation), GTFS-Flex (demand-responsive transport), and
TransitData reporting frameworks for ride-sharing-as-public-transit
deployments.

## A.4 References

- ISO 6709 — Geographic point representation
- ISO 4217 — Currency codes
- ISO/IEC 27001 — information security management
- IETF RFC 3339 — Date and Time
- W3C DID Core — decentralised identifiers
- WIA-OMNI-API credentials standard

## A.5 Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: matching, dynamic pricing, safety, payment stable |
| 1.1.x | Additive: autonomous-vehicle integration patterns |
| 1.2.x | Additive: confidential ride records inside TEEs |
| 2.0.0 | Possible breaking change: post-quantum signature suite migration |


## Z.1 Glossary

The companion glossary at `https://wiastandards.com/ride-sharing/glossary/`
expands every term used throughout this Phase. Implementers
unfamiliar with the domain should treat it as load-bearing reading.

## Z.2 Cross-standard composition

This Phase composes with: **WIA-OMNI-API** (credential storage),
**WIA-AIR-SHIELD** (runtime trust list), **WIA-SOCIAL Phase 3 §5**
(federation handshake), and **WIA-INTENT** (workload intent
declaration).

## Z.3 Conformance test suite + reference container

A black-box conformance test suite at
`https://github.com/WIA-Official/wia-ride-sharing-conformance` walks
every public endpoint and protocol exchange. The reference
container at `wia/ride-sharing-host:1.0.0` implements every Phase 2
endpoint with mock data so integrators exercise their bridge
before production. The companion CLI at `cli/ride-sharing.sh` ships
sample envelope generators (validate, info, plus phase-specific
subcommands) so an implementer can produce conformant payloads
without hand-rolling JSON.

## Z.4 Implementation runbook

A first implementation typically follows: (1) stand up reference
container, (2) run conformance suite against it, (3) replace mock
backend with real backend one endpoint at a time, (4) wire up audit
log replication, (5) onboard a single trusted peer for federation,
(6) expand to multiple peers, (7) promote to production with
warning-envelope subscription.

## Z.5 Backwards-compatibility promise + governance

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable.
Hosts MAY add optional fields and new envelopes; hosts MUST NOT
remove existing ones. Breaking changes ride a major version bump
with a 12-month deprecation window per IETF RFC 8594 / 9745, and
require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.

## A.6 TEE-protected matching

Confidential matching runs the matching algorithm inside a TEE so
even the platform operator cannot observe the queue of unmatched
candidates. This raises the privacy bar from current industry
practice; jurisdictions with strong location-privacy laws (EU under
GDPR, KR under PIPA) increasingly require this level of protection.

## A.7 Autonomous-vehicle integration

When ride-sharing fleets adopt autonomous vehicles, the standard's
ride-request envelope flows directly to the autonomous vehicle's
mission-planning stack. The bridge profile maps the ride request to
SAE J3016 levels of automation and the fallback-driver attestation
when applicable.

## A.8 Cross-platform federation

Cross-platform federation (a rider on Platform A matched with a
driver on Platform B) uses WIA-SOCIAL Phase 3 §5 federation
receipts. The two platforms share signed match envelopes with
documented revenue-sharing per ride.

## A.9 Closing implementer note

Ride-sharing is one of the most location-sensitive standards in the
WIA family. The privacy discipline is mandatory rather than
recommended: hosts that fail privacy conformance are refused on the
public registry. The reference container ships with privacy-by-default
configuration; operators MUST consciously opt in to weaker privacy
modes (e.g., for jurisdictions that mandate plain-text dispatch logs)
and MUST log every opt-in to the audit trail.



## A.10 Roadmap

The next decade of ride-sharing will see: full autonomous-vehicle
fleets entering service, regulatory unification across cities, and
greater privacy expectations from riders. The standards envelope
schema is designed to absorb these transitions without breaking
existing implementations.

## A.11 Reference deployment

A first deployment typically targets a single metropolitan area
with a fleet of 1000-5000 drivers and 100k-500k riders. The
reference deployment guide documents per-region capacity sizing,
per-region regulatory mapping, and the operations runbook for the
first 90 days of production.
