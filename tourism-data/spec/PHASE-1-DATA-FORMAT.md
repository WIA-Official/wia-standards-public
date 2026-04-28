# WIA-IND-017 — Phase 1: Data Format

> Tourism-data canonical Phase 1: attraction + destination + POI + cultural-heritage + accessibility envelopes.

# WIA-IND-017: Tourism Data Specification v1.0

> **Standard ID:** WIA-IND-017
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Core Data Models](#2-core-data-models)
3. [Tourist Attractions](#3-tourist-attractions)
4. [Destination Information](#4-destination-information)
5. [Points of Interest](#5-points-of-interest)
6. [Visitor Analytics](#6-visitor-analytics)
7. [Crowd Density & Real-time Data](#7-crowd-density--real-time-data)
8. [Cultural Heritage](#8-cultural-heritage)
9. [Accessibility Standards](#9-accessibility-standards)
10. [Multi-language Support](#10-multi-language-support)
11. [Seasonality & Trends](#11-seasonality--trends)
12. [Local Experiences](#12-local-experiences)
13. [Safety & Health Information](#13-safety--health-information)
14. [Sustainable Tourism](#14-sustainable-tourism)
15. [Data Quality & Verification](#15-data-quality--verification)
16. [API Specifications](#16-api-specifications)
17. [Security & Privacy](#17-security--privacy)
18. [Implementation Guidelines](#18-implementation-guidelines)
19. [Use Cases](#19-use-cases)
20. [References](#20-references)

---


## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive, interoperable framework for tourism data management. It establishes standardized formats, protocols, and best practices for collecting, storing, sharing, and utilizing tourism-related information across the global travel industry.

### 1.2 Scope

The standard covers:

- Tourist attraction data structures and metadata
- Destination information and profiles
- Visitor statistics and analytics
- Points of interest (POI) databases
- Real-time crowd density monitoring
- Cultural heritage site information
- Accessibility data for inclusive tourism
- Multi-language content management
- Seasonality patterns and forecasting
- Local authentic experiences
- Safety and health information
- Sustainable tourism metrics

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to make tourism accessible to all people regardless of physical ability, language, or economic status, while promoting sustainable practices that preserve cultural heritage and protect natural environments for future generations.

### 1.4 Target Audience

- Tourism boards and destination marketing organizations
- Travel technology companies and platforms
- Hotels, attractions, and service providers
- Mobile app developers
- Government tourism agencies
- Cultural heritage organizations
- Accessibility advocates
- Academic researchers
- Independent travelers

### 1.5 Terminology

**Key Terms:**

- **POI**: Point of Interest - Any location that someone may find useful or interesting
- **Attraction**: A place of interest where tourists visit, typically for its inherent or exhibited natural or cultural value
- **Destination**: A geographic location to which a person travels
- **Visitor Analytics**: Statistical data about tourism traffic and behavior
- **Crowd Density**: The number of people in a given space at a specific time
- **Accessibility**: The design of products, devices, services, or environments for people with disabilities
- **UNESCO**: United Nations Educational, Scientific and Cultural Organization
- **Heritage Site**: A place that has been officially designated as significant
- **Sustainable Tourism**: Tourism that takes full account of current and future economic, social, and environmental impacts

---



## 2. Core Data Models

### 2.1 Geographic Coordinate System

All geographic coordinates MUST use the WGS84 (EPSG:4326) coordinate reference system.

```
Coordinate Format:
{
  "lat": number,    // Latitude in decimal degrees (-90 to 90)
  "lng": number,    // Longitude in decimal degrees (-180 to 180)
  "altitude": number // Optional, in meters above sea level
}
```

**Precision Requirements:**

| Use Case | Decimal Places | Precision |
|----------|----------------|-----------|
| Country-level | 2 | ~1.1 km |
| City-level | 4 | ~11 m |
| Attraction-level | 6 | ~11 cm |
| Indoor positioning | 8 | ~1.1 mm |

### 2.2 Address Format

Addresses MUST follow the international address format with support for local variations:

```json
{
  "street": "5 Avenue Anatole France",
  "city": "Paris",
  "region": "Île-de-France",
  "postalCode": "75007",
  "country": "FR",
  "formatted": "5 Avenue Anatole France, 75007 Paris, France"
}
```

**Country Codes:** ISO 3166-1 alpha-2 (two-letter country codes)

### 2.3 Language Codes

All language identifiers MUST use ISO 639-1 (two-letter) codes, with optional ISO 639-3 (three-letter) for languages not covered by ISO 639-1.

**Primary Languages:**
- en (English)
- es (Spanish)
- fr (French)
- de (German)
- it (Italian)
- pt (Portuguese)
- ru (Russian)
- ja (Japanese)
- ko (Korean)
- zh (Chinese)
- ar (Arabic)
- hi (Hindi)

**Regional Variants:** Use BCP 47 format (e.g., en-US, en-GB, zh-CN, zh-TW)

### 2.4 Currency Codes

All currency identifiers MUST use ISO 4217 three-letter currency codes.

```json
{
  "amount": 26.10,
  "currency": "EUR",
  "formatted": "€26.10"
}
```

### 2.5 Date and Time

All timestamps MUST use ISO 8601 format with timezone information:

```
2025-12-27T14:30:00+01:00
```

### 2.6 Measurement Units

**Distance:** Meters (m) and kilometers (km) as primary units
**Temperature:** Celsius (°C) as primary unit
**Rainfall:** Millimeters (mm)

Implementations MAY provide imperial unit conversions (miles, Fahrenheit, inches) for user display.

---



## 3. Tourist Attractions

### 3.1 Attraction Data Model

Core attraction object structure:

```json
{
  "id": "string",
  "name": "string",
  "names": {
    "en": "string",
    "fr": "string",
    "ja": "string"
  },
  "category": "enum",
  "type": "enum",
  "description": {
    "en": "string",
    "fr": "string"
  },
  "location": {
    "coordinates": {
      "lat": number,
      "lng": number
    },
    "address": {}
  },
  "ratings": {},
  "visitInfo": {},
  "accessibility": {},
  "media": {},
  "contact": {},
  "tags": [],
  "updatedAt": "ISO8601"
}
```

### 3.2 Attraction Categories

**Primary Categories:**

1. **Natural** - Natural formations and environments
   - beach, mountain, park, garden, lake, waterfall, forest, canyon, cave, volcano

2. **Cultural** - Arts and cultural institutions
   - museum, gallery, theater, opera, concert-hall, library, cultural-center

3. **Historical** - Historical sites and monuments
   - monument, castle, palace, ruins, heritage-site, memorial, fort, archaeological-site

4. **Entertainment** - Entertainment venues
   - theme-park, zoo, aquarium, stadium, arena, cinema, arcade, water-park

5. **Religious** - Places of worship and spiritual significance
   - church, cathedral, temple, mosque, synagogue, shrine, monastery, sacred-site

6. **Educational** - Educational institutions and science centers
   - university, science-center, planetarium, observatory, botanical-garden

7. **Sports** - Sports facilities and venues
   - stadium, sports-complex, golf-course, ski-resort, diving-site

8. **Shopping** - Shopping destinations
   - market, mall, shopping-district, bazaar, souk

9. **Nightlife** - Evening entertainment
   - nightclub, bar, lounge, casino, cabaret

10. **Wellness** - Health and wellness facilities
    - spa, hot-spring, wellness-center, yoga-retreat

### 3.3 Rating System

All ratings use a unified 0-5 scale with 0.1 precision:

```
5.0 - Exceptional
4.5-4.9 - Excellent
4.0-4.4 - Very Good
3.5-3.9 - Good
3.0-3.4 - Average
2.5-2.9 - Below Average
2.0-2.4 - Poor
<2.0 - Very Poor
```

**Rating Components:**

```json
{
  "overall": 4.7,
  "count": 125000,
  "breakdown": {
    "5": 85000,
    "4": 30000,
    "3": 8000,
    "2": 1500,
    "1": 500
  },
  "categories": {
    "value": 4.6,
    "atmosphere": 4.8,
    "cleanliness": 4.7,
    "service": 4.5,
    "facilities": 4.6
  }
}
```

### 3.4 Visit Information

Essential information for planning a visit:

```json
{
  "openingHours": {
    "isOpen": true,
    "regular": {
      "monday": "09:00-18:00",
      "tuesday": "09:00-18:00",
      "wednesday": "09:00-21:00",
      "thursday": "09:00-18:00",
      "friday": "09:00-18:00",
      "saturday": "09:00-18:00",
      "sunday": "closed"
    },
    "special": [
      {
        "date": "2025-12-25",
        "hours": "closed",
        "reason": "Christmas Day"
      }
    ]
  },
  "tickets": {
    "free": false,
    "pricing": {
      "adult": {
        "amount": 26.10,
        "currency": "EUR"
      },
      "child": {
        "amount": 13.10,
        "currency": "EUR",
        "notes": "Ages 4-11"
      },
      "student": {
        "amount": 19.50,
        "currency": "EUR",
        "notes": "Valid student ID required"
      },
      "senior": {
        "amount": 19.50,
        "currency": "EUR",
        "notes": "65+ years"
      }
    },
    "bookingUrl": "https://example.com/tickets",
    "reservationRequired": false
  },
  "averageVisitDuration": 120,
  "bestTimeToVisit": {
    "seasons": ["spring", "fall"],
    "months": [4, 5, 9, 10],
    "daysOfWeek": [1, 2, 3, 4],
    "timeOfDay": ["morning", "evening"]
  },
  "guidedTours": true,
  "audioGuideLanguages": ["en", "fr", "es", "de", "it", "ja", "zh"],
  "advanceBookingRequired": false,
  "estimatedWaitTime": 45
}
```

### 3.5 Media Gallery

High-quality visual content for attractions:

```json
{
  "photos": [
    {
      "url": "https://example.com/photo1.jpg",
      "thumbnailUrl": "https://example.com/photo1_thumb.jpg",
      "caption": {
        "en": "Main entrance view",
        "fr": "Vue de l'entrée principale"
      },
      "credit": "Jane Doe Photography",
      "width": 4000,
      "height": 3000,
      "tags": ["exterior", "daytime", "summer"]
    }
  ],
  "videos": [
    {
      "url": "https://youtube.com/watch?v=xxx",
      "thumbnailUrl": "https://example.com/video_thumb.jpg",
      "title": {
        "en": "Virtual Tour"
      },
      "duration": 180,
      "platform": "youtube"
    }
  ],
  "panoramas": [
    {
      "url": "https://example.com/360.jpg",
      "caption": {
        "en": "360° view from observation deck"
      },
      "location": {
        "lat": 48.8584,
        "lng": 2.2945
      }
    }
  ],
  "virtualTour": "https://example.com/virtual-tour"
}
```

### 3.6 Contact Information

```json
{
  "phone": "+33 1 40 20 50 50",
  "email": "info@example.com",
  "whatsapp": "+33612345678",
  "website": "https://example.com",
  "socialMedia": {
    "facebook": "https://facebook.com/example",
    "instagram": "@example",
    "twitter": "@example",
    "youtube": "https://youtube.com/@example",
    "tiktok": "@example",
    "tripadvisor": "https://tripadvisor.com/example"
  }
}
```

---



## 4. Destination Information

### 4.1 Destination Data Model

Comprehensive destination profile:

```json
{
  "id": "paris-france",
  "name": "Paris",
  "names": {
    "en": "Paris",
    "fr": "Paris",
    "ja": "パリ",
    "zh": "巴黎"
  },
  "type": "city",
  "parentId": "france",
  "description": {
    "en": "The City of Light, capital of France...",
    "fr": "La Ville Lumière, capitale de la France..."
  },
  "location": {
    "coordinates": {
      "lat": 48.8566,
      "lng": 2.3522
    },
    "bounds": {
      "north": 48.9,
      "south": 48.8,
      "east": 2.4,
      "west": 2.3
    }
  },
  "tagline": {
    "en": "The City of Light",
    "fr": "La Ville Lumière"
  },
  "population": 2161000,
  "area": 105.4,
  "timeZones": ["Europe/Paris"],
  "currency": {
    "code": "EUR",
    "name": "Euro",
    "symbol": "€"
  },
  "languages": ["fr", "en"],
  "climate": {},
  "bestSeasons": ["spring", "fall"],
  "averageBudget": {
    "amount": 125,
    "currency": "EUR",
    "notes": "Per person per day"
  },
  "safetyRating": 4.0,
  "statistics": {}
}
```

### 4.2 Destination Types

```
Hierarchy:
- country
  - region
    - city
      - neighborhood
        - area
```

### 4.3 Climate Information

Detailed climate data for trip planning:

```json
{
  "type": "Oceanic",
  "temperatures": {
    "jan": 5, "feb": 6, "mar": 9, "apr": 12,
    "may": 16, "jun": 19, "jul": 21, "aug": 21,
    "sep": 17, "oct": 13, "nov": 8, "dec": 5
  },
  "rainfall": {
    "jan": 51, "feb": 41, "mar": 48, "apr": 53,
    "may": 65, "jun": 54, "jul": 63, "aug": 43,
    "sep": 54, "oct": 62, "nov": 51, "dec": 58
  },
  "sunnyDays": {
    "jan": 10, "feb": 11, "mar": 13, "apr": 16,
    "may": 18, "jun": 19, "jul": 20, "aug": 19,
    "sep": 16, "oct": 13, "nov": 10, "dec": 9
  }
}
```

### 4.4 Destination Statistics

Key tourism metrics:

```json
{
  "annualVisitors": 30000000,
  "tourismRevenue": {
    "amount": 17000000000,
    "currency": "EUR"
  },
  "numberOfHotels": 1500,
  "numberOfRestaurants": 40000,
  "numberOfAttractions": 300,
  "averageStayDuration": 3.5,
  "peakMonth": 7,
  "lowMonth": 1
}
```

---



## 5. Points of Interest

### 5.1 POI Categories

**Service POIs:**

1. **Accommodation**
   - hotel, hostel, apartment, guesthouse, resort, camping

2. **Dining**
   - restaurant, cafe, bar, pub, fast-food, bakery, ice-cream

3. **Transportation**
   - airport, train-station, bus-station, metro, taxi-stand, parking, car-rental

4. **Healthcare**
   - hospital, clinic, pharmacy, dentist, medical-center

5. **Financial**
   - bank, atm, currency-exchange, money-transfer

6. **Utilities**
   - gas-station, charging-station, restroom, wifi-hotspot

7. **Services**
   - tourist-info, embassy, consulate, police, post-office, laundry

### 5.2 POI Data Model

```json
{
  "id": "string",
  "name": "string",
  "type": "enum",
  "location": {
    "coordinates": {},
    "address": {}
  },
  "rating": 4.5,
  "priceRange": "$$",
  "openNow": true,
  "openingHours": {},
  "contact": {},
  "features": ["wifi", "outdoor-seating", "wheelchair-accessible"],
  "cuisine": ["french", "mediterranean"],
  "sustainable": true,
  "certifications": ["green-key", "ecolabel"]
}
```

### 5.3 Price Range Classification

```
$ - Budget (under €15 per person)
$$ - Moderate (€15-30 per person)
$$$ - Upscale (€30-60 per person)
$$$$ - Luxury (over €60 per person)
```

---



## 8. Cultural Heritage

### 8.1 Cultural Heritage Site Model

```json
{
  "id": "string",
  "name": "string",
  "description": {},
  "location": {},
  "type": "monument",
  "historicalPeriod": "Medieval",
  "yearEstablished": 1163,
  "unesco": {
    "siteId": 83,
    "inscriptionYear": 1991,
    "category": "cultural",
    "criteria": ["i", "ii", "iv"],
    "endangered": false,
    "url": "https://whc.unesco.org/en/list/83"
  },
  "protectionStatus": {
    "level": "unesco",
    "designation": "World Heritage Site"
  },
  "conservationState": "good",
  "threats": ["climate-change", "pollution"],
  "visitInfo": {},
  "media": {}
}
```

### 8.2 UNESCO World Heritage

**Categories:**
- Cultural
- Natural
- Mixed (both cultural and natural)

**Selection Criteria:**

Cultural:
- (i) Masterpiece of human creative genius
- (ii) Important interchange of human values
- (iii) Unique testimony to cultural tradition
- (iv) Outstanding example of architecture
- (v) Outstanding example of human settlement
- (vi) Associated with events or living traditions

Natural:
- (vii) Superlative natural phenomena
- (viii) Outstanding examples of Earth's history
- (ix) Outstanding examples of ecological processes
- (x) Significant natural habitats for biodiversity

### 8.3 Conservation Status

```
excellent - Perfect condition, no threats
good - Well-maintained, minor issues
fair - Some deterioration, manageable
poor - Significant deterioration, urgent action needed
critical - Severe deterioration, at risk of loss
```

---




---

## A.1 Attraction-record envelope

The canonical attraction envelope carries: attraction identifier (registry-issued; UNESCO WHC reference where applicable), display name in the source language plus IETF BCP 47 language tag, attraction category (natural, cultural, religious, entertainment, gastronomic, sports), GeoJSON Point or Polygon geometry in WGS 84, country/admin-1/admin-2 codes per ISO 3166-1/2 and the regional UN M.49 area code, opening hours in the iCal RRULE format, admission price as a `money` envelope (amount + ISO 4217 currency + tax-included flag), and the canonical media-asset graph (photos, video, panorama with EXIF and rights tags).

## A.2 Destination descriptor

Destination descriptors follow the ISO/TC 228 vocabulary (tourism and related services) and capture: destination identifier, destination type (city, region, route, themed area), parent destination chain, climate envelope (Köppen-Geiger code; representative-month temperature and precipitation; UV-index range), best-visit windows, peak-season flag with month range, infrastructure summary (airport IATA codes within 200 km, rail stations, port codes), and the safety-advisory cross-reference (UK FCDO, US Department of State, KR MOFA travel advisory tier).

## A.3 Point-of-interest envelope

POI envelopes carry: POI identifier, parent attraction or destination, POI type (viewpoint, restaurant, restroom, parking, photography spot, info kiosk, accessibility-equipment loan, charging point), geometry, accessibility flags (wheelchair, hearing-loop, visual-aid, service-animal welcome, large-print signage, sign-language video guide), opening hours, capacity (visitors-at-a-time), queue-time signal availability, and the operator's contact envelope. POIs intended for crowd-density routing cross-reference the Phase 3 §A.2 occupancy stream.

## A.4 Cultural-heritage descriptor

Cultural-heritage descriptors enrich attractions classified as heritage with: heritage status (UNESCO World Heritage; national-monument; provincial-monument; intangible-heritage-list inscription), inscription year, heritage criteria (i-x for cultural, vii-x for natural, mixed), heritage-subject keywords per the Getty Art and Architecture Thesaurus (AAT), conservation-condition rating (good, moderate, at-risk, in-danger), and the responsible-management-authority envelope. Intangible-heritage descriptors carry tradition-bearer references where the bearer has consented to attribution.

## A.5 Accessibility envelope

The accessibility envelope follows the WCAG 2.2 plus ICF-domain (ICF — International Classification of Functioning) decomposition. For each functional domain (mobility, vision, hearing, cognitive, sensory-processing) the envelope carries a numeric accessibility score (0-100), the supported-aid catalogue, the reservation-required flag, the trained-staff availability flag, and the audit date with the auditor's organisation reference. The envelope cross-references the WIA-A11Y standard for the underlying accessibility-attestation chain so downstream consumers can verify the audit record without re-querying.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/tourism-data/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-tourism-data-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/tourism-data-host:1.0.0` ships every tourism-data envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/tourism-data.sh` ships sample envelope generators with no
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
ecosystem. Tourism-data deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
