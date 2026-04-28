# WIA-IND-015 — Phase 1: Data Format

> Travel-tech canonical Phase 1: PNR + flight-segment + hotel + multi-modal-ticket + accessibility envelopes.

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


## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive framework for modern travel technology systems, enabling seamless integration of flight booking, hotel reservations, multi-modal transportation, real-time alerts, travel documentation, payment processing, insurance, and accessibility services.

### 1.2 Scope

The standard covers:
- Flight search and booking integration with global airline systems
- Hotel reservation protocols and property management systems
- Multi-modal transportation planning (air, rail, road, sea)
- Real-time travel alerts and disruption management
- Travel document verification (passports, visas, health certificates)
- Currency conversion and international payment processing
- Travel insurance policy selection and claims processing
- Loyalty program aggregation and redemption
- Accessibility accommodations and special assistance
- Sustainable travel options and carbon footprint tracking

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize travel, making it accessible, affordable, and enjoyable for everyone regardless of physical ability, economic status, or geographic location. Travel connects cultures, broadens perspectives, and enriches human experience.

### 1.4 Terminology

- **GDS**: Global Distribution System (Amadeus, Sabre, Travelport)
- **NDC**: New Distribution Capability (IATA standard)
- **PMS**: Property Management System
- **OTA**: Online Travel Agency
- **PNR**: Passenger Name Record
- **IATA**: International Air Transport Association
- **PCI-DSS**: Payment Card Industry Data Security Standard
- **GDPR**: General Data Protection Regulation
- **WCAG**: Web Content Accessibility Guidelines
- **MCT**: Minimum Connection Time
- **LCC**: Low-Cost Carrier
- **FSC**: Full-Service Carrier

---



## 2. Flight Booking Systems

### 2.1 Airline APIs and Integration

#### 2.1.1 Global Distribution Systems

Traditional GDS platforms provide access to global airline inventories:

```
GDS Platforms:
1. Amadeus:
   - Coverage: 900+ airlines globally
   - API: Amadeus Self-Service APIs
   - Features: Real-time pricing, NDC support
   - Format: JSON/REST, XML/SOAP

2. Sabre:
   - Coverage: 400+ airlines, 175 countries
   - API: Sabre Dev Studio
   - Features: Shopping, booking, ticketing
   - Format: REST/JSON, SOAP/XML

3. Travelport (Apollo, Galileo, Worldspan):
   - Coverage: 400+ airlines
   - API: Universal API
   - Features: Consolidated access
   - Format: XML/SOAP
```

#### 2.1.2 IATA NDC Standard

New Distribution Capability enables direct airline connectivity:

**NDC Benefits:**
- Rich content (seat maps, amenities, ancillaries)
- Dynamic pricing and personalization
- Direct airline connection (bypass GDS fees)
- Real-time inventory and pricing

**NDC Schema Structure:**
```xml
<AirShoppingRQ>
  <CoreQuery>
    <OriginDestinations>
      <OriginDestination>
        <Departure>
          <AirportCode>JFK</AirportCode>
          <Date>2025-06-15</Date>
        </Departure>
        <Arrival>
          <AirportCode>LHR</AirportCode>
        </Arrival>
      </OriginDestination>
    </OriginDestinations>
  </CoreQuery>
  <Preferences>
    <CabinPreferences>
      <CabinType>Economy</CabinType>
    </CabinPreferences>
  </Preferences>
</AirShoppingRQ>
```

#### 2.1.3 Low-Cost Carrier APIs

Direct integration with LCC APIs:

| Airline | Region | API Type | Key Features |
|---------|--------|----------|--------------|
| Ryanair | Europe | REST API | Direct booking, no GDS |
| Southwest | USA | REST API | Proprietary booking system |
| AirAsia | Asia | REST API | Dynamic pricing |
| JetBlue | Americas | NDC + REST | Hybrid distribution |

### 2.2 Flight Search Algorithms

#### 2.2.1 Search Parameters

**Required Parameters:**
- Origin airport (IATA code)
- Destination airport (IATA code)
- Departure date (ISO 8601)
- Number of passengers (by type: adult, child, infant)
- Cabin class (economy, premium, business, first)

**Optional Parameters:**
- Return date (for round-trips)
- Maximum stops (0 for nonstop)
- Preferred airlines or alliances
- Departure/arrival time preferences
- Flexible dates (+/- N days)
- Maximum price limit
- Baggage requirements

#### 2.2.2 Search Optimization

**Caching Strategy:**
```
Cache Levels:
1. L1 (In-Memory): Hot routes, 30-second TTL
2. L2 (Redis): Popular searches, 5-minute TTL
3. L3 (Database): Historical prices, 1-hour TTL
4. CDN: Static content (airport data, airline info)

Cache Invalidation:
- Price changes from airline feeds
- Inventory updates (seats sold)
- Schedule changes or cancellations
- Manual override for flash sales
```

**Search Result Ranking:**
```
Ranking Algorithm:
Score = (1 - Price_Normalized) × 0.4 +
        (1 - Duration_Normalized) × 0.3 +
        (Airline_Rating / 10) × 0.2 +
        (1 - Stops / Max_Stops) × 0.1

Where:
- Price_Normalized: Price relative to cheapest option
- Duration_Normalized: Duration relative to fastest
- Airline_Rating: 0-10 safety/service rating
- Stops: Number of connections
```

### 2.3 Booking and Ticketing

#### 2.3.1 Booking Flow

```
Standard Booking Process:
1. Flight Selection
   ↓
2. Passenger Information
   - Names (as on passport)
   - Date of birth
   - Contact details
   - Frequent flyer numbers
   ↓
3. Seat Selection
   - Seat map display
   - Preference indication
   - Extra legroom options
   ↓
4. Ancillary Services
   - Baggage
   - Meals
   - Priority boarding
   - Lounge access
   ↓
5. Payment
   - Payment method selection
   - Secure processing (PCI-DSS)
   - Fraud detection
   ↓
6. Confirmation
   - PNR generation
   - E-ticket issuance
   - Confirmation email/SMS
```

#### 2.3.2 Passenger Name Record (PNR)

**PNR Structure:**
```
PNR Components:
- Record Locator (6 alphanumeric characters)
- Passenger Names (surname/given name)
- Contact Information (phone, email)
- Itinerary Segments (flights)
- Ticket Numbers (13 digits)
- Form of Payment (FOP)
- Special Service Requests (SSR)
- Remarks (OSI, general info)
```

**Example PNR:**
```
RP/NYCAA08AB/NYCAA08AB AA/SU  27DEC25/1245Z   ABC123
1.DOE/JOHN MR
AA 100Y 15JUN JFKLHR HK1  0800  2015
PHONE-1-555-123-4567
TICKET-0012345678901
SEAT-12A
MEAL-VGML
```

### 2.4 Fare Rules and Restrictions

#### 2.4.1 Fare Classes

```
Fare Class Hierarchy (Example):
F  = First Class (full fare)
A  = First Class (discounted)
J  = Business Class (full fare)
C  = Business Class (discounted)
Y  = Economy (full fare)
B  = Economy (high discount)
M  = Economy (medium discount)
K  = Economy (deep discount)
```

#### 2.4.2 Fare Rules

**Standard Fare Conditions:**
```
1. Refundability:
   - Refundable: Full/partial refund available
   - Non-refundable: No refund, may get credit
   - Partially refundable: Cancellation fee applies

2. Changeability:
   - Flexible: Free changes
   - Change fee: Fixed fee for modifications
   - Non-changeable: No changes allowed

3. Advance Purchase:
   - 0-3 days: Walk-up fares
   - 7 days: Standard advance
   - 14-21 days: Discount advance
   - 30+ days: Early bird discount

4. Stay Requirements:
   - Minimum stay: 1 night, Saturday night
   - Maximum stay: 30 days, 60 days, 12 months

5. Routing:
   - Direct only
   - One-way allowed
   - Backtracking restrictions
```

### 2.5 Flight Status and Notifications

#### 2.5.1 Real-Time Flight Tracking

**Data Sources:**
- Airline operational systems
- FlightRadar24, FlightAware APIs
- Airport departure/arrival boards
- ADS-B tracking data

**Status Types:**
```
Flight Status Values:
- Scheduled: On time, no changes
- Delayed: New departure/arrival time
- Departed: Airborne
- En Route: In flight
- Landed: Arrived at destination
- Cancelled: Flight cancelled
- Diverted: Emergency landing elsewhere
- Gate Change: Different departure gate
```

#### 2.5.2 Notification Channels

```
Alert Delivery Methods:
1. Email: Immediate + summary digest
2. SMS: Critical alerts (gate change, cancellation)
3. Push Notification: Mobile app
4. WhatsApp: International travelers
5. In-App: Real-time within travel app
6. API Webhook: For integrated systems
```

---



## 3. Hotel Reservation Protocols

### 3.1 Property Management Systems

#### 3.1.1 PMS Integration

**Major PMS Platforms:**
```
1. Opera (Oracle Hospitality):
   - Market share: 45% of global hotels
   - API: OPERA Cloud API
   - Features: Reservation, guest profile, housekeeping

2. Protel:
   - Coverage: European market leader
   - API: Protel I/O
   - Features: Channel management, POS integration

3. RoomMaster:
   - Coverage: Asia-Pacific
   - API: REST/SOAP
   - Features: Multi-property management

4. Cloudbeds:
   - Type: Cloud-native PMS
   - API: myallocator, Channel Manager
   - Features: All-in-one solution
```

#### 3.1.2 Channel Managers

Distribution across multiple platforms:

```
Channel Distribution:
├── GDS (Amadeus, Sabre, Worldspan)
├── OTA (Booking.com, Expedia, Hotels.com)
├── Metasearch (Google Hotels, Trivago, Kayak)
├── Direct Booking (Hotel website)
└── Wholesalers (Hotelbed, Tourico)

Rate Parity: Same rate across all channels
Inventory Sync: Real-time availability updates
```

### 3.2 Hotel Search and Filtering

#### 3.2.1 Search Parameters

**Location Search:**
```
Location Types:
1. City/Address: Text search with geocoding
2. Coordinates: Latitude/longitude + radius
3. Points of Interest: Near landmarks
4. Airport: Within distance of airport
5. Neighborhood: Specific districts
6. Map Bounds: Search within map viewport
```

**Filter Options:**
```json
{
  "filters": {
    "price": {
      "min": 0,
      "max": 500,
      "currency": "USD"
    },
    "stars": [3, 4, 5],
    "guestRating": {
      "min": 8.0
    },
    "amenities": [
      "wifi",
      "parking",
      "pool",
      "gym",
      "spa",
      "restaurant",
      "breakfast"
    ],
    "accessibility": {
      "wheelchair": true,
      "elevator": true,
      "visualAid": false
    },
    "propertyType": [
      "hotel",
      "resort",
      "apartment",
      "hostel"
    ],
    "cancellation": "free",
    "mealPlan": ["breakfast", "half-board", "all-inclusive"]
  }
}
```

#### 3.2.2 Hotel Scoring Algorithm

```
Hotel Ranking Score:
Score = Price_Score × 0.35 +
        Rating_Score × 0.30 +
        Location_Score × 0.20 +
        Amenity_Score × 0.10 +
        Availability_Score × 0.05

Where:
- Price_Score: Inverse price (cheaper = higher)
- Rating_Score: Guest reviews (0-10 scale)
- Location_Score: Distance to search center
- Amenity_Score: Match to requested amenities
- Availability_Score: Room availability
```

### 3.3 Room Types and Rates

#### 3.3.1 Room Categories

```
Standard Room Types:
1. Standard/Classic:
   - Basic room, standard amenities
   - 20-25 sqm typical size
   - 1-2 guests

2. Superior:
   - Enhanced amenities, better view
   - 25-30 sqm
   - 2 guests

3. Deluxe:
   - Premium amenities, city/water view
   - 30-40 sqm
   - 2-3 guests

4. Suite:
   - Separate living area, bedroom
   - 40-80 sqm
   - 2-4 guests

5. Executive/Club:
   - Lounge access, premium location
   - 35-50 sqm
   - Business amenities
```

#### 3.3.2 Rate Plans

```
Rate Plan Types:
1. BAR (Best Available Rate):
   - Standard public rate
   - Flexible cancellation
   - Breakfast included options

2. Non-Refundable:
   - 10-30% discount vs BAR
   - No cancellation/changes
   - Prepayment required

3. Package Rates:
   - Room + meals
   - Room + spa
   - Room + attractions

4. Corporate:
   - Negotiated company rates
   - Corporate ID required
   - Flexible terms

5. Government/Military:
   - Special rates for gov employees
   - ID verification required

6. Member Rates:
   - Loyalty program discounts
   - Exclusive offers
```

### 3.4 Booking and Confirmation

#### 3.4.1 Booking Process

```
Hotel Booking Flow:
1. Search Results
   ↓
2. Hotel Selection
   - View photos, reviews, location
   - Compare room types
   ↓
3. Room Selection
   - Choose room type
   - Select rate plan
   - Add extras (parking, breakfast)
   ↓
4. Guest Information
   - Guest name(s)
   - Contact details
   - Special requests
   ↓
5. Payment
   - Credit card guarantee
   - Prepayment (if required)
   ↓
6. Confirmation
   - Confirmation number
   - Hotel voucher
   - Cancellation policy
```

#### 3.4.2 Confirmation Details

**Confirmation Email Contents:**
```
Booking Confirmation:
- Confirmation Number: HTL-123456789
- Hotel Name & Address
- Check-in: June 15, 2025 (15:00)
- Check-out: June 22, 2025 (11:00)
- Nights: 7
- Room Type: Deluxe King
- Guest Name: John Doe
- Total Price: $2,100 USD
- Payment: Card ending 1234
- Cancellation: Free until June 13, 23:59
- Hotel Contact: +44-20-1234-5678
- Special Requests: Late check-in (22:00)
```

### 3.5 Cancellation Policies

#### 3.5.1 Standard Policies

```
Cancellation Tiers:
1. Fully Flexible:
   - Free cancellation until 24h before check-in
   - No penalties

2. Moderate:
   - Free cancellation until 3-7 days before
   - 1-night penalty after deadline

3. Strict:
   - Free cancellation until 14-30 days before
   - 50-100% penalty after deadline

4. Non-Refundable:
   - No cancellation allowed
   - No refund under any circumstances
   - May allow date changes (with fee)
```

#### 3.5.2 Force Majeure

**Special Circumstances:**
- Natural disasters (hurricanes, earthquakes)
- Pandemics (COVID-19 policies)
- Civil unrest or war
- Government travel bans
- Medical emergencies (with documentation)

Hotels may offer flexible cancellation or rebooking in these cases.

---




---

## A.1 PNR and reservation envelope

The Phase 1 envelope carries the canonical PNR (Passenger Name Record) shape: PNR identifier, record locator (six-character alphanumeric), passenger-name list with passport-name canonicalisation, contact envelope (email, mobile, secondary), itinerary segments, fare envelope, ancillaries, frequent-flyer associations, special-service requests (SSR codes per IATA AHM), and the chain-of-distribution envelope (issuing PSS, GDS, OTA, agency). PNR data follows IATA Resolution 830 requirements and the GDPR / CCPA / PIPA equivalents for personally identifiable traveller data.

## A.2 Flight-segment envelope

A flight-segment record MUST list: marketing carrier (IATA + ICAO codes), operating carrier, flight number, IATA aircraft-type code, scheduled departure/arrival in local time with IANA-tz reference, scheduled-block-time, origin/destination IATA airport codes, terminal envelope, cabin-class (RBD), fare-basis code, baggage-allowance, and the codeshare-attribution chain. Segments cross-reference SSIM (Standard Schedules Information Manual) for the schedule version and the OAG snapshot at booking time so disputes can be resolved deterministically.

## A.3 Hotel-property envelope

Hotel envelopes carry: property identifier (chain code + property code; or independent UUID), property name in the source language plus IETF BCP 47 language tag, GeoJSON Point geometry in WGS 84, address per UPU S42, star rating (chain self-rating; jurisdiction-issued rating where applicable; user-generated rating), room-type catalogue with the ARI (Availability/Rate/Inventory) breakdown, ancillaries (parking, breakfast, spa, transfers), accessibility envelope per Phase 1 §A.5, and the cancellation-policy envelope.

## A.4 Multi-modal-ticket envelope

Multi-modal tickets unify air, rail (UIC TAP TSI Technical Document A), bus (NeTEx CEN TS 16614), ferry, ride-hail, micro-mobility, and car-rental within a single passenger-journey envelope. Each leg carries the modality, the operator, the booking reference, the start/end stops with GTFS stop_id where applicable, the fare component, and the per-leg refundability. Inter-modal transfers are explicit edges in the journey graph with minimum-connect-time (MCT) requirements.

## A.5 Accessibility and special-service envelope

The accessibility envelope follows IATA AHM 176 and the SSR code catalogue (WCHR, WCHS, WCHC, BLND, DEAF, MEDA, PETC, etc.) plus the equivalent rail (UIC PRM TSI) and surface-transport codes. The envelope captures the per-leg accommodation requested, the carrier's per-leg confirmation status, the equipment-rental envelope (wheelchair, oxygen, mobility-scooter), and the trained-staff envelope. Reservations made under the accessibility envelope cross-reference the WIA-A11Y standard for the underlying accessibility-attestation chain.


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
