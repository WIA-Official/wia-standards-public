# WIA-IND-015 — Phase 4: Integration

> Travel-tech canonical Phase 4: ecosystem integration (privacy + IATA + GDS + loyalty + accessibility + future).

# WIA-IND-015: Travel Tech Specification v1.0

> **Standard ID:** WIA-IND-015
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Flight Booking Systems](#2-flight-booking-systems)
3. [Hotel Reservation Protocols](#3-hotel-reservation-protocols)
4. [Multi-Modal Transportation](#4-multi-modal-transportation)
5. [Travel Itinerary Management](#5-travel-itinerary-management)
6. [Real-Time Travel Alerts](#6-real-time-travel-alerts)
7. [Travel Document Verification](#7-travel-document-verification)
8. [Currency Conversion and Payment](#8-currency-conversion-and-payment)
9. [Travel Insurance Integration](#9-travel-insurance-integration)
10. [Loyalty Program Management](#10-loyalty-program-management)
11. [Accessibility Accommodations](#11-accessibility-accommodations)
12. [Data Privacy and Security](#12-data-privacy-and-security)
13. [API Specifications](#13-api-specifications)
14. [Implementation Guidelines](#14-implementation-guidelines)
15. [References](#15-references)

---


## 9. Travel Insurance Integration

### 9.1 Insurance Types

#### 9.1.1 Coverage Categories

```
Insurance Coverage Types:
1. Trip Cancellation:
   - Reimburse non-refundable costs
   - Covered reasons: illness, death, weather
   - Typical limit: 100% of trip cost

2. Medical Coverage:
   - Emergency medical expenses abroad
   - Hospital stays, doctor visits
   - Medical evacuation
   - Typical limit: $50,000 - $500,000

3. Baggage Protection:
   - Lost, stolen, or damaged luggage
   - Per-item limits
   - Typical limit: $2,000 - $5,000

4. Flight Delay:
   - Compensation after X hours delay
   - Covers meals, accommodation
   - Typical: $100-500 per day

5. Comprehensive:
   - All of the above
   - Additional: car rental, adventure sports
   - Most expensive but complete coverage
```

### 9.2 Insurance Recommendation Engine

#### 9.2.1 Policy Matching Algorithm

```javascript
function recommendInsurance(trip) {
  const factors = {
    tripCost: trip.totalPrice,
    destination: trip.destinations,
    duration: calculateNights(trip.startDate, trip.endDate),
    activities: trip.plannedActivities,
    travelers: trip.travelers,
    age: trip.travelers.map(t => calculateAge(t.dateOfBirth)),
    existingCoverage: trip.existingInsurance
  };

  // Calculate risk score
  const riskScore =
    (factors.tripCost > 5000 ? 30 : 15) +
    (isHighRiskDestination(factors.destination) ? 25 : 0) +
    (factors.duration > 14 ? 20 : 10) +
    (hasAdventureSports(factors.activities) ? 25 : 0) +
    (hasSeniors(factors.age) ? 20 : 0);

  // Recommend based on risk
  if (riskScore > 70) {
    return "comprehensive";
  } else if (riskScore > 40) {
    return "standard-plus-medical";
  } else {
    return "basic-cancellation";
  }
}
```

### 9.3 Claims Processing

#### 9.3.1 Claims Workflow

```
Insurance Claim Process:
1. Incident Occurs
   ↓
2. Traveler Files Claim:
   - Online form submission
   - Upload supporting documents
   - Police report (if theft)
   - Medical reports (if injury)
   ↓
3. Initial Review (24-48h):
   - Claim completeness check
   - Request additional documents
   ↓
4. Assessment (3-7 days):
   - Verify coverage
   - Calculate compensation
   - Fraud detection check
   ↓
5. Decision:
   - Approve (full/partial)
   - Deny (with reason)
   - Request more info
   ↓
6. Payment (5-10 days):
   - Bank transfer
   - Check mailing
   - PayPal
```

---



## 10. Loyalty Program Management

### 10.1 Program Integration

#### 10.1.1 Major Loyalty Programs

**Airlines:**
```
Top Airline Programs:
1. American AAdvantage:
   - Members: 100M+
   - Currency: Miles
   - Tiers: Gold, Platinum, Executive Platinum
   - Partners: Oneworld alliance

2. United MileagePlus:
   - Members: 100M+
   - Currency: Miles
   - Tiers: Silver, Gold, Platinum, 1K
   - Partners: Star Alliance

3. Delta SkyMiles:
   - Members: 100M+
   - Currency: Miles
   - Tiers: Silver, Gold, Platinum, Diamond
   - Partners: SkyTeam

4. Air France-KLM Flying Blue:
   - Members: 19M+
   - Currency: Miles + XP
   - Tiers: Silver, Gold, Platinum
```

**Hotels:**
```
Hotel Loyalty Programs:
1. Marriott Bonvoy:
   - Properties: 8,000+
   - Brands: 30 (Marriott, Ritz-Carlton, etc.)
   - Tiers: Silver, Gold, Platinum, Titanium, Ambassador

2. Hilton Honors:
   - Properties: 7,000+
   - Tiers: Silver, Gold, Diamond
   - Points: No expiry with activity

3. IHG Rewards:
   - Properties: 6,000+
   - Brands: Holiday Inn, InterContinental, etc.
   - Tiers: Club, Gold Elite, Platinum, Spire

4. World of Hyatt:
   - Properties: 1,200+
   - Tiers: Discoverist, Explorist, Globalist
   - Guest of Honor: Share status
```

### 10.2 Points and Miles Tracking

#### 10.2.1 Balance Aggregation

```json
{
  "loyaltyAccounts": [
    {
      "program": "American AAdvantage",
      "memberNumber": "ABC123456",
      "tier": "Platinum",
      "balance": {
        "miles": 125000,
        "value": "$1,500 USD"
      },
      "expiring": {
        "miles": 10000,
        "date": "2025-12-31"
      },
      "tierProgress": {
        "current": "Platinum",
        "next": "Executive Platinum",
        "milesNeeded": 25000,
        "segmentsNeeded": 10
      }
    },
    {
      "program": "Marriott Bonvoy",
      "memberNumber": "987654321",
      "tier": "Gold",
      "balance": {
        "points": 250000,
        "value": "$1,250 USD"
      },
      "nights": {
        "lifetime": 120,
        "yearToDate": 25,
        "needed": 25
      }
    }
  ],
  "totalValue": "$2,750 USD"
}
```

### 10.3 Redemption Optimization

#### 10.3.1 Best Value Calculator

```javascript
function findBestRedemption(points, program, destination, dates) {
  const options = [];

  // Option 1: Award flights
  const flights = searchAwardFlights(program, destination, dates);
  flights.forEach(flight => {
    const cpp = flight.cashPrice / flight.pointsRequired; // cents per point
    options.push({
      type: "flight",
      description: flight.route,
      pointsRequired: flight.pointsRequired,
      cashValue: flight.cashPrice,
      centsPerPoint: cpp,
      value: cpp > 1.5 ? "excellent" : cpp > 1.2 ? "good" : "fair"
    });
  });

  // Option 2: Hotel nights
  const hotels = searchAwardHotels(program, destination, dates);
  hotels.forEach(hotel => {
    const cpp = hotel.cashPrice / hotel.pointsRequired;
    options.push({
      type: "hotel",
      description: hotel.name,
      pointsRequired: hotel.pointsRequired,
      cashValue: hotel.cashPrice,
      centsPerPoint: cpp,
      value: cpp > 1.0 ? "excellent" : cpp > 0.8 ? "good" : "fair"
    });
  });

  // Sort by value (highest cpp first)
  return options.sort((a, b) => b.centsPerPoint - a.centsPerPoint);
}
```

---



## 11. Accessibility Accommodations

### 11.1 Accessibility Standards

#### 11.1.1 WCAG Compliance

```
Web Content Accessibility Guidelines (WCAG 2.1):
Level AA Compliance:
1. Perceivable:
   - Alt text for images
   - Captions for videos
   - Color contrast ratio 4.5:1
   - Resizable text (200%)

2. Operable:
   - Keyboard navigation
   - No keyboard traps
   - Skip navigation links
   - Focus indicators

3. Understandable:
   - Clear language
   - Consistent navigation
   - Error identification
   - Help available

4. Robust:
   - Valid HTML
   - ARIA landmarks
   - Screen reader compatible
   - Assistive tech compatible
```

### 11.2 Physical Accessibility

#### 11.2.1 Hotel Accessibility Features

```
Accessible Room Requirements:
1. Mobility:
   - Wheelchair accessible entrance
   - Wide doorways (32" minimum)
   - Roll-in shower or accessible tub
   - Grab bars (toilet, shower)
   - Lowered counters, sinks
   - Accessible closet rods
   - Visual doorbell

2. Visual Impairment:
   - Braille room numbers
   - Tactile signage
   - High contrast markings
   - Audio guidance
   - Large print materials

3. Hearing Impairment:
   - Visual fire alarms
   - Vibrating alarm clock
   - TTY/TDD phones
   - Visual doorbell
   - Closed caption TV

4. Service Animals:
   - Allowed in all areas
   - Relief areas nearby
   - Food/water bowls available
```

#### 11.2.2 Transportation Accessibility

```
Accessible Transport:
1. Flights:
   - Wheelchair assistance (airport, aircraft)
   - Aisle chairs for boarding
   - Priority boarding
   - Accessible lavatories (wide-body aircraft)
   - Service animal accommodation
   - Special meal requests

2. Trains:
   - Wheelchair spaces
   - Accessible toilets
   - Audio/visual announcements
   - Ramps or lifts
   - Assistance available

3. Taxis/Rideshare:
   - Wheelchair accessible vehicles (WAV)
   - Ramps or lifts
   - Secure wheelchair restraints
   - Trained drivers

4. Rental Cars:
   - Hand controls
   - Wheelchair accessible vans
   - Swivel seats
   - Modified controls
```

### 11.3 Special Assistance Requests

#### 11.3.1 Booking Flow Integration

```json
{
  "specialRequirements": {
    "mobility": {
      "wheelchair": {
        "required": true,
        "type": "manual",
        "ownChair": true,
        "dimensions": {
          "width": 26,
          "height": 36,
          "weight": 45
        }
      },
      "assistance": {
        "airport": true,
        "aircraft": true,
        "destination": true
      }
    },
    "visual": {
      "blindOrLowVision": true,
      "assistanceDog": true,
      "brailleMaterials": true
    },
    "hearing": {
      "deaf": false,
      "hardOfHearing": true,
      "signLanguage": "ASL"
    },
    "medical": {
      "oxygen": false,
      "medication": true,
      "refrigeration": false
    },
    "dietary": {
      "allergies": ["peanuts", "shellfish"],
      "preferences": ["vegetarian", "gluten-free"],
      "religious": "halal"
    }
  }
}
```

---



## 12. Data Privacy and Security

### 12.1 Privacy Regulations

#### 12.1.1 GDPR Compliance

```
GDPR Requirements:
1. Lawful Basis:
   - Consent (explicit, freely given)
   - Contract (necessary for service)
   - Legal obligation
   - Vital interests
   - Public interest
   - Legitimate interests

2. User Rights:
   - Right to access data
   - Right to rectification
   - Right to erasure ("right to be forgotten")
   - Right to data portability
   - Right to object
   - Right to restrict processing

3. Data Protection:
   - Privacy by design
   - Data minimization
   - Encryption at rest and in transit
   - Pseudonymization
   - Regular audits

4. Breach Notification:
   - Report to authority within 72 hours
   - Notify affected users
   - Document breach
```

### 12.2 Data Security

#### 12.2.1 Security Measures

```
Security Implementation:
1. Encryption:
   - TLS 1.3 for all connections
   - AES-256 for data at rest
   - End-to-end encryption for sensitive data
   - Certificate pinning (mobile apps)

2. Authentication:
   - Multi-factor authentication (MFA)
   - OAuth 2.0 / OpenID Connect
   - Session management
   - JWT tokens (short-lived)
   - Biometric authentication (mobile)

3. Authorization:
   - Role-based access control (RBAC)
   - Principle of least privilege
   - API key rotation
   - IP whitelisting (admin)

4. Monitoring:
   - Real-time threat detection
   - Intrusion detection system (IDS)
   - Security Information and Event Management (SIEM)
   - Regular penetration testing
   - Vulnerability scanning
```

---



## 15. References

### 15.1 Standards and Specifications

- IATA NDC Schema
- OTA (OpenTravel Alliance) Specifications
- PCI-DSS Requirements
- GDPR Regulation (EU) 2016/679
- WCAG 2.1 Guidelines
- ISO 3166 Country Codes
- ISO 4217 Currency Codes
- ISO 8601 Date/Time Format

### 15.2 Industry Resources

- IATA: International Air Transport Association
- UNWTO: UN World Tourism Organization
- WTTC: World Travel & Tourism Council
- ACI: Airports Council International

### 15.3 WIA Related Standards

- WIA-INTENT: Intent-based travel search
- WIA-OMNI-API: Universal API gateway
- WIA-PAYMENT: Payment processing
- WIA-IDENTITY: Identity verification
- WIA-HEALTH: Health certificates
- WIA-SOCIAL: Social travel features

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-IND-015 Travel Tech Standard v1.0*
*© 2025 SmileStory Inc. / WIA*
*Published: December 27, 2025*
*Status: Active*



---

## A.1 Privacy and data-protection cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| EU general data protection    | GDPR Regulation (EU) 2016/679             |
| US California                 | CCPA + CPRA                               |
| US sectoral (health-adjacent) | HIPAA where applicable                    |
| Korea                         | PIPA                                      |
| Japan                         | APPI                                      |
| China                         | PIPL + Cross-Border Data Transfer rules   |
| Brazil                        | LGPD                                      |
| Canada                        | PIPEDA                                    |
| IATA traveller-data           | IATA Resolution 830                       |
| Card-payment data             | PCI-DSS 4.0                               |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 Partnership and distribution integration

Distribution integration captures the partnership envelope: distribution channel (direct, GDS — Amadeus/Sabre/Travelport/SITA, NDC-Level-3 partner, OTA), the IATA NDC capability tier (1/2/3/4), the dynamic-offer envelope (carrier-tailored offers per IATA NDC), and the SLA. The integration model preserves the operator-of-record concept: the originating carrier or hotel chain remains the authoritative source for the underlying inventory, and the partner surfaces the inventory via reference rather than copy.

## A.3 Loyalty-program integration

Loyalty integration follows the IATA AHM 805 loyalty-data interchange plus the alliance frameworks (Star Alliance, oneworld, SkyTeam) for points-redemption clearing. The integration envelope captures the membership identifier, the tier, the points balance, the redemption catalogue, and the cross-program redemption-clearance reference. The standard prohibits opaque points-devaluation: any change to redemption-rate envelopes requires advance notice consistent with the program's published terms.

## A.4 Accessibility and inclusion integration

Accessibility integration honours the IATA Resolution 700 + AHM 176 framework, the EU Regulation (EC) 1107/2006 (rights of disabled passengers), the ADA in the United States, and the equivalent national frameworks. The integration envelope captures the per-leg accommodation request, the carrier's confirmation status, the equipment-rental envelope, and the trained-staff envelope; the WIA-A11Y attestation chain provides the cross-tenant verification of the accessibility-audit history.

## A.5 Future directions

Active research tracks: end-to-end NDC Level-4 multi-carrier offers; sustainable-aviation-fuel (SAF) per-segment attribution and traveller-side carbon accounting per CORSIA; agentic itinerary-planning with on-device privacy; biometric end-to-end journey (One ID per IATA) with strict consent and revocation; verifiable-credential traveller-IDs as an alternative to passport scans where the destination accepts them. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- IATA Resolutions 700 / 822 / 830 / 024d — passenger services, refunds, traveller data, currency rounding
- IATA AHM 176 / 180 / 805 — accessibility, schedules, loyalty
- ICAO Doc 9303 — machine-readable travel documents
- ICAO PKD — Public Key Directory for ePassport verification
- UIC TAP TSI Technical Document A — rail booking
- CEN TS 16614 (NeTEx) — public-transport network exchange
- GTFS / GTFS-Realtime — General Transit Feed Specification
- PCI-DSS 4.0 — payment-card industry data security standard
- PSD2 RTS on Strong Customer Authentication
- GDPR / CCPA / PIPL / PIPA / APPI / LGPD / PIPEDA — privacy regimes
- EU Regulation (EC) 1107/2006 — rights of disabled persons in air transport


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/travel-tech/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-travel-tech-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/travel-tech-host:1.0.0` ships every travel-tech envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/travel-tech.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Travel-tech deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
