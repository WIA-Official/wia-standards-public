# WIA-IND-015 — Phase 3: Protocol

> Travel-tech canonical Phase 3: protocols (itinerary + alerts + document verification + payment + insurance).

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


## 4. Multi-Modal Transportation

### 4.1 Transportation Modes

#### 4.1.1 Mode Classification

```
Transportation Categories:
1. Air:
   - Commercial airlines
   - Charter flights
   - Private aviation

2. Rail:
   - High-speed trains (TGV, Shinkansen, ICE)
   - Regional trains
   - Subway/Metro
   - Light rail

3. Road:
   - Bus/Coach (intercity, city)
   - Taxi/Cab
   - Rideshare (Uber, Lyft)
   - Car rental
   - Bike sharing

4. Sea:
   - Ferry
   - Cruise
   - Water taxi

5. Micro-mobility:
   - E-scooters
   - Bike sharing
   - E-bikes
```

#### 4.1.2 Intermodal Integration

**Journey Planning:**
```
Multi-Modal Trip:
Origin: Home (New York)
   ↓ [Taxi] 20 min
Airport (JFK)
   ↓ [Flight AA100] 7h 15min
Airport (LHR)
   ↓ [Heathrow Express] 15 min
London Paddington
   ↓ [Tube] 10 min
Hotel (Central London)

Total Time: 8h 20min (including connections)
Total Cost: $850 (flight) + $35 (train) + $25 (taxi) = $910
Carbon: 850 kg CO2
```

### 4.2 Rail Integration

#### 4.2.1 Rail Booking APIs

**European Rail:**
```
Rail Providers:
- Trainline: Multi-country aggregator
- SNCF (France): TGV, Intercités
- Deutsche Bahn (Germany): ICE, regional
- Trenitalia (Italy): Frecciarossa
- Renfe (Spain): AVE
- Eurostar: UK-Europe tunnel service
```

**Rail API Features:**
- Seat reservation
- Real-time schedules
- Delay notifications
- Multi-leg journeys
- Railpass integration (Eurail, Interrail)

#### 4.2.2 Rail Pass Support

```
Rail Pass Types:
1. Eurail Global Pass:
   - Coverage: 33 European countries
   - Duration: 4 days to 3 months
   - Age groups: Youth, adult, senior

2. Japan Rail Pass:
   - Coverage: All JR lines
   - Duration: 7/14/21 days
   - Class: Ordinary, Green (first)

3. Britrail (UK):
   - Coverage: England, Scotland, Wales
   - Flexible or consecutive days
```

### 4.3 Ground Transportation

#### 4.3.1 Rideshare Integration

**API Integration:**
```javascript
// Uber API Example
{
  "pickup": {
    "latitude": 40.7580,
    "longitude": -73.9855,
    "address": "Times Square, NYC"
  },
  "dropoff": {
    "latitude": 40.6413,
    "longitude": -73.7781,
    "address": "JFK Airport"
  },
  "products": [
    {
      "product_id": "uberx",
      "display_name": "UberX",
      "capacity": 4,
      "estimate": {
        "fare": "$45-55",
        "duration": 35,
        "distance": 16.5
      }
    }
  ]
}
```

#### 4.3.2 Car Rental

**Major Providers:**
- Enterprise, Hertz, Avis, Budget
- Local providers (country-specific)
- Peer-to-peer (Turo, Getaround)

**Rental Parameters:**
```
Car Rental Booking:
- Pickup location & date/time
- Dropoff location & date/time (one-way allowed)
- Vehicle class (economy, compact, SUV, luxury)
- Transmission (automatic, manual)
- Driver age (young driver fees)
- Insurance options (CDW, LDW, PAI)
- Additional drivers
- GPS, child seats, ski racks
```

### 4.4 Route Optimization

#### 4.4.1 Routing Algorithm

**Dijkstra's Algorithm for Multi-Modal:**
```
Route Finding:
Graph Nodes: Locations (airports, stations, hotels)
Graph Edges: Transport modes with:
  - Cost (price)
  - Duration
  - Comfort score
  - Carbon footprint
  - Reliability

Optimization:
Minimize: α×Cost + β×Time + γ×Discomfort + δ×Carbon
Subject to:
  - Minimum connection time at transfers
  - Operating hours constraints
  - Capacity constraints
  - User preferences
```

#### 4.4.2 Transfer Time Requirements

```
Minimum Connection Times (MCT):
Airport Transfers:
- Domestic to Domestic: 45-60 minutes
- Domestic to International: 90-120 minutes
- International to International: 90-120 minutes
- Terminal change: +15-30 minutes

Train Station Transfers:
- Same station: 5-10 minutes
- Different stations: 20-45 minutes

Intermodal (Air to Rail):
- Airport to train station: 30-60 minutes
```

---



## 5. Travel Itinerary Management

### 5.1 Itinerary Data Model

#### 5.1.1 Itinerary Structure

```json
{
  "id": "itin-abc123",
  "title": "European Summer Vacation",
  "description": "Two weeks exploring London, Paris, and Rome",
  "travelers": [
    {
      "id": "traveler-1",
      "firstName": "John",
      "lastName": "Doe",
      "type": "adult",
      "documents": {
        "passport": {
          "number": "X12345678",
          "country": "US",
          "expiry": "2030-01-15"
        }
      }
    }
  ],
  "startDate": "2025-06-15",
  "endDate": "2025-06-29",
  "destinations": ["London", "Paris", "Rome"],
  "items": [
    {
      "id": "item-1",
      "type": "flight",
      "dateTime": "2025-06-15T08:00:00Z",
      "title": "Flight to London",
      "airline": "AA",
      "flightNumber": "100",
      "route": "JFK-LHR",
      "bookingReference": "ABC123",
      "confirmationNumber": "AA-ABC123",
      "status": "confirmed"
    },
    {
      "id": "item-2",
      "type": "hotel",
      "dateTime": "2025-06-15T15:00:00Z",
      "endDateTime": "2025-06-20T11:00:00Z",
      "title": "The Royal Grand Hotel",
      "location": "Central London",
      "confirmationNumber": "HTL-456789",
      "status": "confirmed"
    }
  ],
  "budget": {
    "planned": 8000,
    "actual": 7650,
    "currency": "USD"
  }
}
```

### 5.2 Itinerary Sharing

#### 5.2.1 Sharing Methods

```
Sharing Options:
1. Link Sharing:
   - Public URL with read-only access
   - Optional password protection
   - Expiry date

2. Email Invitation:
   - Send to specific email addresses
   - Permission levels (view, edit)
   - Notification of updates

3. Calendar Export:
   - iCal format
   - Sync with Google Calendar, Outlook
   - Automatic updates

4. PDF Export:
   - Printable itinerary
   - Include confirmations, maps
   - QR codes for mobile access

5. Social Sharing:
   - Share on Facebook, Instagram
   - Privacy controls
   - Inspire others
```

### 5.3 Collaboration Features

#### 5.3.1 Multi-User Access

```
Permission Levels:
1. Owner:
   - Full control
   - Can delete itinerary
   - Manage permissions

2. Editor:
   - Add/edit/remove items
   - Book services
   - Cannot delete itinerary

3. Viewer:
   - Read-only access
   - Export/print
   - Receive notifications

4. Contributor:
   - Suggest changes
   - Comments only
   - Vote on options
```

### 5.4 Smart Suggestions

#### 5.4.1 AI-Powered Recommendations

```
Recommendation Engine:
1. Activity Suggestions:
   - Based on destination
   - Time of day
   - Weather forecast
   - User interests
   - Budget remaining

2. Restaurant Recommendations:
   - Cuisine preferences
   - Dietary restrictions
   - Proximity to current location
   - Price range
   - Ratings/reviews

3. Transportation:
   - Fastest route
   - Most scenic route
   - Cheapest option
   - Eco-friendly option

4. Gap Filling:
   - Detect free time in itinerary
   - Suggest activities to fill gaps
   - Optimize route to minimize travel
```

---



## 6. Real-Time Travel Alerts

### 6.1 Alert Types

#### 6.1.1 Flight Alerts

```
Flight Alert Categories:
1. Schedule Changes:
   - Departure time change
   - Arrival time change
   - Flight number change
   - Aircraft type change

2. Operational:
   - Delay (with updated time)
   - Cancellation
   - Diversion
   - Gate change
   - Terminal change

3. Boarding:
   - Check-in open
   - Boarding started
   - Final call
   - Gate closing

4. Status:
   - Departed
   - En route
   - Landed
   - Baggage carousel number
```

#### 6.1.2 Weather Alerts

```
Weather Alert Types:
1. Severe Weather:
   - Hurricane/Typhoon
   - Tornado
   - Blizzard
   - Extreme heat/cold

2. Travel Impact:
   - Airport closures
   - Flight cancellations
   - Road closures
   - Ferry cancellations

3. Advisory:
   - Rain forecast
   - High winds
   - Poor visibility
   - Temperature warnings

Alert Severity:
- Low: Minor impact, plan ahead
- Medium: Moderate impact, consider alternatives
- High: Significant disruption likely
- Critical: Dangerous conditions, avoid travel
```

#### 6.1.3 Destination Alerts

```
Destination Advisory Types:
1. Safety & Security:
   - Terrorism threat level
   - Civil unrest
   - Crime warnings
   - Natural disasters

2. Health:
   - Disease outbreaks
   - Vaccination requirements
   - Water quality
   - Air quality

3. Entry Requirements:
   - Visa policy changes
   - COVID-19 restrictions
   - Customs regulations
   - Currency restrictions

4. Events:
   - Public holidays
   - Strikes (transport, services)
   - Major events (traffic impact)
   - Seasonal considerations

Data Sources:
- Government travel advisories
- WHO health alerts
- Local news feeds
- User reports
```

### 6.2 Notification Delivery

#### 6.2.1 Multi-Channel Delivery

```
Notification Channels:
1. Push Notification:
   - Mobile app
   - Real-time delivery
   - Rich content (images, actions)
   - Priority: Critical alerts

2. Email:
   - Detailed information
   - Links to resources
   - Digest option (daily summary)
   - HTML formatted

3. SMS:
   - Critical alerts only
   - Character limit (160)
   - International roaming compatible
   - Fallback if app unavailable

4. WhatsApp:
   - International travelers
   - Rich media support
   - Two-way communication
   - Popular in many regions

5. Voice Call:
   - Extreme emergencies
   - Elderly or accessibility needs
   - When other channels fail
```

#### 6.2.2 Alert Preferences

```json
{
  "alertPreferences": {
    "channels": {
      "push": true,
      "email": true,
      "sms": false,
      "whatsapp": true
    },
    "types": {
      "flightDelays": {
        "enabled": true,
        "minDelay": 30,
        "channels": ["push", "sms"]
      },
      "gateChanges": {
        "enabled": true,
        "channels": ["push"]
      },
      "weather": {
        "enabled": true,
        "severity": ["high", "critical"],
        "channels": ["push", "email"]
      }
    },
    "quietHours": {
      "enabled": true,
      "start": "22:00",
      "end": "07:00",
      "exceptions": ["critical"]
    }
  }
}
```

---



## 7. Travel Document Verification

### 7.1 Passport Requirements

#### 7.1.1 Passport Validity Rules

```
Standard Passport Requirements:
1. Validity Period:
   - Minimum 6 months beyond travel date
   - Some countries: 3 months
   - EU for EU citizens: Valid through stay
   - Check country-specific rules

2. Blank Pages:
   - Minimum 2 blank pages
   - Some countries require more
   - Pages must be consecutive

3. Condition:
   - Undamaged, readable
   - Photo clearly visible
   - Machine-readable zone intact
   - No water damage or tears
```

#### 7.1.2 Passport Verification API

```javascript
// Passport Verification Example
{
  "document": {
    "type": "passport",
    "number": "X12345678",
    "issuingCountry": "US",
    "nationality": "US",
    "issueDate": "2020-01-15",
    "expiryDate": "2030-01-15"
  },
  "travel": {
    "destination": "GB",
    "departureDate": "2025-06-15",
    "returnDate": "2025-06-22",
    "purpose": "tourism"
  },
  "verification": {
    "validityCheck": {
      "valid": true,
      "expiryAfterTravel": "2029-07-22",
      "daysValid": 1680,
      "meetsRequirement": true
    },
    "visaRequirement": {
      "required": false,
      "reason": "Visa waiver for US citizens (up to 6 months)",
      "maxStay": 180
    }
  }
}
```

### 7.2 Visa Requirements

#### 7.2.1 Visa Types

```
Common Visa Categories:
1. Tourist Visa:
   - Purpose: Tourism, vacation
   - Duration: 15 days to 6 months
   - Single/multiple entry
   - No work permitted

2. Business Visa:
   - Purpose: Meetings, conferences
   - Duration: 30 days to 1 year
   - Multiple entry typical
   - No employment allowed

3. Transit Visa:
   - Purpose: Connecting flight
   - Duration: 24-72 hours
   - Airport transit
   - May not need if staying airside

4. Student Visa:
   - Purpose: Education
   - Duration: Length of course
   - May allow part-time work

5. Work Visa:
   - Purpose: Employment
   - Duration: Contract length
   - Employer sponsorship required

6. Visa Waiver:
   - No visa required
   - Automatic permission
   - Limited duration
   - Specific passport countries
```

#### 7.2.2 Visa Requirements Database

**Example Visa Matrix:**

| Passport | Destination | Visa Required | Type | Max Stay | Processing Time |
|----------|-------------|---------------|------|----------|-----------------|
| US | UK | No | Visa Waiver | 6 months | N/A |
| US | China | Yes | Tourist | 30 days | 4-7 days |
| US | Schengen | No | ETIAS (2025) | 90 days | Online |
| UK | US | No | ESTA | 90 days | Online |
| IN | UK | Yes | Standard Visitor | 6 months | 3 weeks |

### 7.3 Health Requirements

#### 7.3.1 Vaccination Requirements

```
Vaccination Categories:
1. Required (Entry Denied Without):
   - Yellow Fever: Africa, South America
   - Polio: Some countries
   - Meningitis: Saudi Arabia (Hajj)

2. Recommended:
   - Hepatitis A/B
   - Typhoid
   - Rabies (rural areas)
   - Japanese Encephalitis

3. Routine:
   - COVID-19 (check current rules)
   - Flu (seasonal)
   - Tetanus

4. COVID-19 Specific:
   - Vaccination certificate
   - PCR test (within 72h)
   - Antigen test (within 48h)
   - Quarantine requirements
```

#### 7.3.2 Health Certificate Verification

```json
{
  "certificate": {
    "type": "covid-vaccination",
    "holder": {
      "name": "John Doe",
      "dateOfBirth": "1985-05-15",
      "passportNumber": "X12345678"
    },
    "vaccination": {
      "vaccine": "Pfizer-BioNTech",
      "doses": 3,
      "dates": [
        "2021-03-15",
        "2021-04-15",
        "2021-11-15"
      ],
      "certificateId": "URN:UVCI:01:NL:ABC123"
    },
    "verification": {
      "valid": true,
      "recognizedBy": ["US", "EU", "UK", "CA"],
      "meetsRequirements": {
        "GB": true,
        "FR": true,
        "CN": false
      }
    }
  }
}
```

---



## 8. Currency Conversion and Payment

### 8.1 Currency Conversion

#### 8.1.1 Exchange Rate Sources

```
Rate Providers:
1. Central Banks:
   - Federal Reserve (US)
   - ECB (European Central Bank)
   - Bank of England
   - Most authoritative, updated daily

2. Financial APIs:
   - OpenExchangeRates.org
   - XE.com API
   - Fixer.io
   - Real-time rates

3. Payment Processors:
   - Stripe
   - PayPal
   - Wise (formerly TransferWise)
   - Commercial rates with markup
```

#### 8.1.2 Conversion Algorithm

```javascript
function convertCurrency(amount, from, to, date = null) {
  // Get exchange rate
  const rate = getExchangeRate(from, to, date);

  // Calculate base conversion
  const baseAmount = amount * rate;

  // Apply spread (markup)
  const spread = 0.02; // 2% markup
  const spreadAmount = baseAmount * spread;

  // Service fee
  const fee = Math.max(1.00, baseAmount * 0.005); // $1 min or 0.5%

  return {
    from: {
      amount: amount,
      currency: from
    },
    to: {
      amount: baseAmount,
      currency: to
    },
    rate: rate,
    spread: spreadAmount,
    fee: fee,
    total: baseAmount - spreadAmount - fee,
    timestamp: new Date().toISOString()
  };
}
```

### 8.2 Payment Processing

#### 8.2.1 Payment Methods

```
Supported Payment Types:
1. Credit/Debit Cards:
   - Visa, Mastercard, Amex, Discover
   - 3D Secure authentication
   - PCI-DSS Level 1 compliance
   - Tokenization for security

2. Digital Wallets:
   - Apple Pay
   - Google Pay
   - PayPal
   - Samsung Pay
   - One-click checkout

3. Bank Transfers:
   - ACH (US)
   - SEPA (Europe)
   - Wire transfer
   - 1-3 day processing

4. Buy Now Pay Later:
   - Affirm
   - Klarna
   - Afterpay
   - Split payment over time

5. Cryptocurrency (Optional):
   - Bitcoin (BTC)
   - Ethereum (ETH)
   - Stablecoins (USDC, USDT)
   - Via payment processor

6. Loyalty Points:
   - Airline miles
   - Hotel points
   - Credit card points
   - Partial payment allowed
```

#### 8.2.2 Payment Security

**PCI-DSS Compliance:**
```
Security Requirements:
1. Data Encryption:
   - TLS 1.3 for transmission
   - AES-256 for storage
   - End-to-end encryption

2. Tokenization:
   - Card data never stored
   - Tokens used for recurring
   - PCI scope reduction

3. Fraud Detection:
   - Address verification (AVS)
   - CVV check
   - 3D Secure (3DS2)
   - Velocity checking
   - Device fingerprinting
   - Machine learning models

4. Data Retention:
   - No full card numbers stored
   - Last 4 digits only
   - Expiry date for convenience
   - Audit logs (12 months)
```

### 8.3 Dynamic Pricing

#### 8.3.1 Price Localization

```
Localized Pricing Strategy:
1. Currency Display:
   - User's local currency
   - Market-specific pricing
   - Tax inclusion/exclusion

2. Price Optimization:
   - Purchasing power parity
   - Local competition
   - Demand-based pricing
   - Seasonal adjustments

3. Example:
   Flight JFK-LHR
   - US market: $850 USD
   - UK market: £690 GBP (~$860)
   - EU market: €780 EUR (~$850)
   - Japan market: ¥124,000 JPY (~$840)
```

---




---

## A.1 Itinerary-construction protocol

Multi-modal itinerary construction follows a graph-based protocol: nodes are stops/airports/stations/terminals; edges are operating segments with attached fare components; the construction algorithm searches for a path that respects the modality preferences, the maximum total time, the maximum number of transfers, the minimum connect time per transfer, the explicit-stops-in-itinerary list, and the budget constraint. The protocol produces a Pareto-optimal frontier across (price, time, transfers, sustainability) so SDKs can offer travellers a structured trade-off rather than a single recommendation.

## A.2 Real-time alerts protocol

The alerts protocol fuses authoritative carrier feeds (IATA SSIM updates, FAA NOTAMs, EUROCONTROL Network Manager messages, weather warnings from national met services), airport operational feeds (gate, belt), and crowdsourced ground signals (e.g., delay reports). Alerts emit through the Phase 2 §A.5 WebSocket with a confidence score and a source attribution; downstream SDKs can choose their alert-display threshold per traveller preference.

## A.3 Travel-document verification protocol

Document verification follows ICAO Doc 9303 (machine-readable travel documents): MRZ parsing per the Type 1/2/3 format with check-digit verification, NFC chip authentication via Basic Access Control + PACE + Active Authentication where supported, and the document-validity cross-check against ICAO PKD when the verifier holds an authorised connection. Verification-result envelopes are signed and tied to the booking record so downstream check-in flows do not re-execute the verification unnecessarily.

## A.4 Currency-conversion and payment protocol

Currency conversion uses the ECB / BoK / Fed reference rates plus the carrier's published rounding policy per IATA Resolution 024d (currency rounding); the rate-of-the-day plus the timestamp is captured in the price envelope so disputes can be resolved deterministically. Payment processing follows PCI-DSS 4.0 with the SAQ profile per the implementation's card-data flow; Strong Customer Authentication per PSD2 RTS is applied for EEA-issued cards, and equivalent local SCA frameworks (KFTC OpenBanking / India RBI eMandate) for the relevant jurisdictions.

## A.5 Insurance and ancillary protocol

Travel-insurance integration covers product enumeration (medical, trip-cancellation, baggage, kidnap-and-ransom, COVID-related coverage), pre-purchase quote with the underwriter-of-record envelope, policy issuance with the digital-policy envelope (signed by the underwriter), claims-initiation endpoint, and the claims-status WebSocket. The protocol prohibits silent insurance bundling: SCA flows must surface the insurance opt-in as an explicit check rather than a pre-checked default per applicable consumer-protection rules.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the booking-creation and cancellation control plane. PNR-modification audit logs are hash-chained per booking so revisions to the booking history can be detected during dispute resolution. Tokens issued from `POST /flights/price` carry a server-side-only random value bound to the search context so token replay across user sessions is detected.


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
