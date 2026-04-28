# WIA-IND-017 — Phase 3: Protocol

> Tourism-data canonical Phase 3: protocols (visitor-flow + crowd-density + seasonality + verification + sustainable).

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


## 6. Visitor Analytics

### 6.1 Visitor Statistics Data Model

Comprehensive visitor data structure:

```json
{
  "id": "attraction-id",
  "period": {
    "start": "2024-01-01T00:00:00Z",
    "end": "2024-12-31T23:59:59Z"
  },
  "granularity": "monthly",
  "totalVisitors": 7000000,
  "dataPoints": [
    {
      "timestamp": "2024-01-01T00:00:00Z",
      "visitors": 450000,
      "revenue": 8200000,
      "satisfaction": 4.5
    }
  ],
  "demographics": {},
  "origins": {},
  "revenue": {},
  "satisfaction": {}
}
```

### 6.2 Demographics Analysis

```json
{
  "ageGroups": {
    "0-17": 150000,
    "18-24": 700000,
    "25-34": 1800000,
    "35-44": 1600000,
    "45-54": 1300000,
    "55-64": 900000,
    "65+": 550000
  },
  "gender": {
    "male": 3200000,
    "female": 3600000,
    "other": 200000
  },
  "travelType": {
    "solo": 800000,
    "couple": 2500000,
    "family": 2700000,
    "group": 800000,
    "business": 200000
  }
}
```

### 6.3 Origin Analysis

```json
{
  "countries": {
    "US": 1260000,
    "GB": 840000,
    "DE": 700000,
    "IT": 560000,
    "JP": 490000,
    "FR": 2450000
  },
  "regions": {
    "North America": 1400000,
    "Europe": 4200000,
    "Asia": 1000000,
    "Other": 400000
  },
  "domesticVsInternational": {
    "domestic": 2450000,
    "international": 4550000
  }
}
```

### 6.4 Revenue Metrics

```json
{
  "total": {
    "amount": 137000000,
    "currency": "EUR"
  },
  "perVisitor": {
    "amount": 19.57,
    "currency": "EUR"
  },
  "bySource": {
    "tickets": {
      "amount": 100000000,
      "currency": "EUR"
    },
    "merchandise": {
      "amount": 25000000,
      "currency": "EUR"
    },
    "food": {
      "amount": 10000000,
      "currency": "EUR"
    },
    "other": {
      "amount": 2000000,
      "currency": "EUR"
    }
  }
}
```

### 6.5 Satisfaction Metrics

```json
{
  "overall": 4.7,
  "nps": 65,
  "repeatVisitRate": 0.35,
  "wouldRecommend": 0.92,
  "aspectRatings": {
    "value": 4.6,
    "experience": 4.8,
    "facilities": 4.7,
    "service": 4.5,
    "cleanliness": 4.7
  }
}
```

---



## 7. Crowd Density & Real-time Data

### 7.1 Crowd Density Model

Real-time occupancy tracking:

```json
{
  "current": 2500,
  "capacity": 3000,
  "percentage": 83,
  "level": "high",
  "estimatedWaitTime": 45,
  "timestamp": "2025-12-27T14:30:00+01:00",
  "forecast": [
    {
      "time": "2025-12-27T15:00:00+01:00",
      "expectedLevel": "very-high",
      "expectedPercentage": 95,
      "confidence": 0.85
    },
    {
      "time": "2025-12-27T16:00:00+01:00",
      "expectedLevel": "very-high",
      "expectedPercentage": 90,
      "confidence": 0.80
    }
  ],
  "historicalPattern": {
    "typicalForTime": 2200,
    "percentageChange": 14
  }
}
```

### 7.2 Crowd Levels

```
Level Classification:
- low: 0-25% capacity
- moderate: 26-50% capacity
- high: 51-75% capacity
- very-high: 76-90% capacity
- at-capacity: 91-100% capacity
- over-capacity: >100% capacity
```

### 7.3 Wait Time Estimation

Wait times calculated based on:
- Current queue length
- Historical processing times
- Number of service points
- Time of day patterns
- Special events

### 7.4 Data Update Frequency

```
Real-time data: Every 1-5 minutes
Near real-time: Every 15-30 minutes
Periodic updates: Hourly
Historical data: Daily aggregation
```

---



## 11. Seasonality & Trends

### 11.1 Seasonality Data Model

```json
{
  "id": "destination-id",
  "seasons": [
    {
      "name": "peak",
      "months": [6, 7, 8],
      "visitorVolume": 1.0,
      "priceLevel": 1.0,
      "crowdLevel": "very-high",
      "weather": "Warm and sunny",
      "events": ["Summer Festival", "Jazz Festival"],
      "pros": ["Best weather", "All attractions open"],
      "cons": ["Highest prices", "Crowded", "Long wait times"]
    },
    {
      "name": "shoulder",
      "months": [4, 5, 9, 10],
      "visitorVolume": 0.7,
      "priceLevel": 0.8,
      "crowdLevel": "moderate",
      "weather": "Pleasant temperatures",
      "pros": ["Good weather", "Fewer crowds", "Better prices"],
      "cons": ["Some attractions may have reduced hours"]
    },
    {
      "name": "low",
      "months": [11, 12, 1, 2, 3],
      "visitorVolume": 0.4,
      "priceLevel": 0.6,
      "crowdLevel": "low",
      "weather": "Cold, occasional rain",
      "pros": ["Lowest prices", "No crowds", "Authentic experience"],
      "cons": ["Weather", "Shorter days", "Some closures"]
    }
  ],
  "monthly": [
    {
      "month": 1,
      "visitors": 450000,
      "temperature": 5,
      "rainfall": 51,
      "crowdLevel": "low",
      "events": ["New Year celebrations"]
    }
  ]
}
```

### 11.2 Trend Analysis

**Key Metrics:**
- Year-over-year growth
- Month-over-month changes
- Day-of-week patterns
- Hour-of-day patterns
- Event impact analysis
- Weather correlation
- Economic indicators

### 11.3 Forecasting

Prediction models based on:
- Historical data (5+ years)
- Seasonal patterns
- Special events calendar
- Economic trends
- Marketing campaigns
- External factors (pandemics, natural disasters)

---



## 15. Data Quality & Verification

### 15.1 Quality Tiers

**Gold Tier:**
- 100% data coverage
- Real-time updates
- Professional photography
- 20+ languages
- Verified accessibility info
- >99% accuracy

**Silver Tier:**
- >90% data coverage
- Hourly updates
- High-quality photos
- 10+ languages
- Basic accessibility info
- >95% accuracy

**Bronze Tier:**
- >75% data coverage
- Daily updates
- Standard photos
- 5+ languages
- Limited accessibility info
- >90% accuracy

**Basic Tier:**
- >50% data coverage
- Weekly updates
- Basic photos
- 1-2 languages
- No accessibility info
- >80% accuracy

### 15.2 Verification Process

**Data Sources:**
1. Official sources (government, attraction websites)
2. Trusted third-party aggregators
3. User-generated content (verified)
4. On-site verification
5. Partner submissions

**Verification Steps:**
1. Source credibility check
2. Cross-reference multiple sources
3. Logical consistency check
4. Regular re-verification
5. User feedback integration

### 15.3 Update Frequency

```
Critical data (hours, prices): Daily
Seasonal data: Monthly
Annual statistics: Yearly
Infrastructure changes: As they occur
Reviews and ratings: Real-time
```

### 15.4 Data Freshness Indicators

Every data point SHOULD include:
- Last updated timestamp
- Data source
- Verification status
- Confidence level

---




---

## A.1 Visitor-flow protocol

Visitor-flow data ingestion follows the entry-event protocol: entry-token presentation at the gate, server-side validation of the token signature and the time-slot envelope, occupancy-counter increment, dwell-time start, and the audit-event emission. Exit events follow the symmetric protocol with dwell-time finalisation and consent-bound retention. Aggregate flow metrics are computed at k>=20 spatial bin granularity per the privacy envelope at Phase 2 §A.5; raw event streams are retained only inside the operator's tenant boundary.

## A.2 Crowd-density and queue-management protocol

Crowd-density signals fuse counter-based occupancy, computer-vision crowd estimates from approved camera feeds, queue-length minutes derived from RFID/Bluetooth dwell times, and operator-reported caps. The protocol covers the data-fusion weights (Bayesian linear update with sensor-specific noise priors), the smoothing window (rolling 5-minute median), and the alert thresholds for the `WS /crowd-density/stream` push events. Queue-management routes visitors via the route-engine API at WIA-INTENT to the next-best alternative POI when congestion exceeds policy thresholds.

## A.3 Seasonality and trend protocol

Seasonality records track per-attraction monthly visitor counts across rolling 36-month windows, weather-correlated demand, event-driven peaks (festivals, sporting events, holidays), and the de-trended residual. The protocol exposes the forecasted demand at 1-day, 7-day, and 90-day horizons with an associated 80% prediction interval; consumers can subscribe to anomaly events when realised demand departs from the prediction interval.

## A.4 Verification and data-quality protocol

Every attraction record is verified against three sources: the operator's official channel (publisher signature), at least one independent travel directory (DMO partner or open data portal), and the in-product user-feedback signal (with k>=30 user reports per discrepancy). Discrepancies trigger a revalidation event; revalidation pulls operator data, recomputes, and either confirms or marks the record `needs-review`. Heritage records additionally cross-check against UNESCO WHC and the national heritage authority's open-data feed.

## A.5 Sustainable-tourism protocol

Carbon-footprint estimation per visitor follows the GSTC (Global Sustainable Tourism Council) destination criteria together with the IPCC AR6 emission factors. Each attraction carries a per-visitor emission envelope (transport mode breakdown, on-site energy, water, waste). Sustainable-tourism flagging on search results uses the published GSTC criteria scores; over-tourism warnings ride on top of the crowd-density protocol with a destination-policy gate that the DMO operator owns.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the reservation-creation control plane. Telemetry traffic uses mTLS with per-channel monotonic counters; replay attempts are detected and dropped at the broker. Reservation tokens are single-use and encode a server-side-only random value to bind the entry event to the issuing reservation.


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
