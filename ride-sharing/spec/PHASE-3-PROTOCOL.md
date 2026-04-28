# WIA-AUTO-014 — Phase 3: Protocol

> Privacy, security, and performance-requirement protocol layer. The driver/rider verification chain and the safety-feature exchanges are wire-level with replay defence applied uniformly.

## 10. Privacy and Security

### 10.1 Data Protection

**Personal data minimization:**
- Collect only necessary information
- Anonymize data after 90 days
- Delete inactive accounts after 2 years
- Encrypt all PII (Personally Identifiable Information)

**Encryption standards:**
- Data in transit: TLS 1.3+
- Data at rest: AES-256
- Database: Encrypted columns for sensitive data
- Backups: Encrypted with separate keys

### 10.2 Location Privacy

**Location data handling:**
```
1. Precise location (GPS) shared only during:
   - Active ride request
   - Accepted trip
   - Driver en route or trip in progress

2. After trip completion:
   - Precise locations fuzzy to 100m radius
   - Exact addresses removed after 30 days
   - Only city-level data retained for analytics

3. Location sharing controls:
   - User can disable location when app not in use
   - Must consent to background location (for safety)
   - Can review location history
   - Can request location data deletion
```

### 10.3 Phone Number Privacy

**Anonymous communication:**
```
- Use proxy phone numbers for rider-driver communication
- Calls routed through platform (no number exposure)
- SMS relayed with masked numbers
- Numbers revealed only after mutual consent
- Communication available only during active trip ± 24 hours
```

### 10.4 Payment Security

**PCI DSS compliance:**
- Never store full card numbers
- Use tokenization for payment methods
- 3D Secure for card verification
- Fraud detection algorithms
- Transaction monitoring

### 10.5 Access Control

**Role-based access:**
```
Riders:
- View/edit own profile
- Request rides
- View trip history
- Rate drivers

Drivers:
- View/edit own profile
- Accept/reject rides
- View earnings
- Rate riders

Admins:
- View aggregated data
- Manage accounts
- Resolve disputes
- Access audit logs

Safety Team:
- Access trip data during incidents
- View GPS tracks
- Listen to emergency recordings
- Contact users
```

### 10.6 Audit Logging

```
Log all security-relevant events:
- Login attempts (success/failure)
- Account changes
- Payment transactions
- Trip start/complete
- Emergency activations
- Admin actions
- Data exports
- System access

Retention: 7 years minimum
```

---


## 11. Performance Requirements

### 11.1 API Response Times

```
Endpoint                    Target      Maximum
-------------------------------------------------
Fare estimate              < 200ms     500ms
Ride request               < 500ms     1s
Driver match               < 2s        5s
Location update            < 100ms     300ms
Trip status                < 200ms     500ms
Payment processing         < 1s        3s
```

### 11.2 Availability

```
Service Level Agreement (SLA):
- Uptime: 99.9% (< 8.76 hours downtime/year)
- Peak hours: 99.95%
- Planned maintenance: < 4 hours/month
- Incident response: < 15 minutes
```

### 11.3 Scalability

```
System must support:
- 1M+ concurrent users
- 100K+ active drivers
- 10K+ requests/second
- 1M+ trips/day
- 100GB+ data/day

Auto-scaling triggers:
- CPU > 70%: Scale up
- Request queue > 1000: Add instances
- Response time > target × 2: Alert + scale
```

### 11.4 Location Update Frequency

```
Driver location updates:
- Active trip: Every 5 seconds
- Waiting for trip: Every 30 seconds
- Driver heading to pickup: Every 5 seconds

Rider location (for pickup):
- Shared only when requested
- Updated every 10 seconds
- Stopped after pickup
```

---



## A.1 Privacy-preserving matching

Naive matching reveals every rider's pickup point to every
nearby driver. The privacy-preserving matching protocol shows
each driver only the matched rider's pickup point, never the
queue of unmatched candidates. The protocol envelopes carry
encrypted pickup points; the platform decrypts only the matched
pickup before sending to the matched driver.

## A.2 Safety and security exchanges

Safety-feature exchanges (SOS button, in-trip check-in, share-trip
with trusted contact) flow as signed envelopes audited per the
standard's discipline. The audit log replicates across at least
two storage backends with retention sized to regulatory
requirements (typically 5 years for safety-incident records).

## A.3 Performance requirements

The standard recommends performance targets:

| Operation | Target latency | Failure budget |
|-----------|----------------|----------------|
| Ride request → matched driver | ≤ 30 s | 1% per month |
| Driver location update → rider's app | ≤ 5 s | 0.5% per month |
| SOS button press → safety operations centre | ≤ 3 s | 0.01% per month |
| Trip-end → fare settlement | ≤ 60 s | 1% per month |

Hosts publishing performance below these targets MUST emit a
warning envelope; sustained breach triggers a regulator notification
in jurisdictions with explicit ride-sharing oversight.

## A.4 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second
seen-nonce cache.


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


## Z.6 Closing implementer note

This Phase fits inside the larger ride-sharing standard's four-Phase
architecture. The Phase 1 envelopes are the wire-format
contract; Phase 2 surfaces them through HTTPS; Phase 3 wraps
them in protocol exchanges that cross trust boundaries; Phase
4 integrates with the broader ecosystem of regulators, peer
standards, and existing platform tooling.

A first implementation of this Phase typically takes one
engineer-week for a team already familiar with the underlying
domain; subsequent phases of the same standard build on the
identity, signature, and audit infrastructure laid down here
so each phase is faster than the previous.



## A.5 Federation across operators

Cross-operator ride-share federation lets a rider on platform A be
matched with a driver on platform B. The federation envelope reuses
WIA-SOCIAL Phase 3 §5 receipt shape; trust lists carry the peer
operators identity plus revenue-sharing rules.

## A.6 Audit log replication

Audit envelopes are written to an append-only log replicated across
at least two storage backends. Retention is sized to the longest
applicable regulatory window: 5 years for safety-incident records
in most jurisdictions, 7 years for tax-related ride records.

## A.7 Closing protocol note

Ride-sharing protocols are sensitive to abuse: fake matches, GPS
spoofing, fare manipulation, identity fraud. The standards
discipline is intentionally rigid to limit the abuse surface;
operators weakening it must opt out explicitly and log the opt-out
in the audit trail.
