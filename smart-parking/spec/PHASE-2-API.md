# WIA-AUTO-013 — Phase 2: API Interface

> Smart-parking canonical Phase 2: API surface (reservation + payment + sessions + WebSocket).

# WIA-AUTO-013: Smart Parking Specification v1.0

> **Standard ID:** WIA-AUTO-013
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Parking Detection Technologies](#2-parking-detection-technologies)
3. [Occupancy States and Transitions](#3-occupancy-states-and-transitions)
4. [Parking Guidance Systems](#4-parking-guidance-systems)
5. [Reservation and Payment Systems](#5-reservation-and-payment-systems)
6. [Real-Time Occupancy Tracking](#6-real-time-occupancy-tracking)
7. [EV Charging Integration](#7-ev-charging-integration)
8. [Data Formats and Protocols](#8-data-formats-and-protocols)
9. [API Specification](#9-api-specification)
10. [Security and Privacy](#10-security-and-privacy)
11. [References](#11-references)

---


## 5. Reservation and Payment Systems

### 5.1 Reservation Workflow

#### 5.1.1 Reservation Request

```json
{
  "reservation_request": {
    "lot_id": "lot-downtown-01",
    "start_time": "2025-12-26T14:00:00Z",
    "duration": 120,
    "vehicle_info": {
      "license_plate": "ABC123",
      "type": "sedan",
      "length": 4.5,
      "requires_ev": true
    },
    "user_info": {
      "user_id": "user-12345",
      "phone": "+1-555-0100",
      "email": "user@example.com"
    },
    "preferences": {
      "proximity": "entrance",
      "covered": true
    }
  }
}
```

#### 5.1.2 Space Allocation Algorithm

```python
def allocate_space(request, available_spaces):
    """
    Allocate optimal parking space for reservation
    """
    # Filter by requirements
    eligible = [
        space for space in available_spaces
        if matches_requirements(space, request)
    ]

    # Check availability during requested timeframe
    eligible = [
        space for space in eligible
        if is_available_during(space, request.start_time, request.duration)
    ]

    # Score and rank
    scored = [
        (space, score_for_reservation(space, request))
        for space in eligible
    ]

    # Sort by score (descending)
    scored.sort(key=lambda x: x[1], reverse=True)

    if scored:
        return scored[0][0]  # Return top-scoring space
    else:
        return None  # No available space
```

#### 5.1.3 Reservation Confirmation

```json
{
  "reservation": {
    "confirmation_code": "RES-20251226-4A8F",
    "space_id": "SP-EV-042",
    "lot_id": "lot-downtown-01",
    "start_time": "2025-12-26T14:00:00Z",
    "end_time": "2025-12-26T16:00:00Z",
    "grace_period": 15,
    "estimated_cost": {
      "amount": 20.00,
      "currency": "USD",
      "breakdown": {
        "parking": 18.00,
        "ev_charging": 2.00,
        "fees": 0.00
      }
    },
    "access_code": "4582",
    "instructions": "Enter code at gate. Space located on Level 2, Section E."
  }
}
```

### 5.2 Payment Integration

#### 5.2.1 Payment Methods

```
Supported Methods:
├── Credit/Debit Cards (Stripe, Square, PayPal)
├── Mobile Wallets (Apple Pay, Google Pay, Samsung Pay)
├── License Plate Payment (automated billing)
├── QR Code Payment (scan and pay)
├── RFID/NFC (tap to pay)
└── Account Balance (prepaid accounts)
```

#### 5.2.2 Pricing Models

**Hourly Pricing**
```
Rate = base_rate × time_hours × multiplier

Multipliers:
- Peak hours (8am-6pm weekdays): 1.5×
- Off-peak: 1.0×
- Overnight (10pm-6am): 0.5×
- Weekend: 1.2×
- Special events: 2.0× - 3.0×
```

**Dynamic Pricing**
```
Rate = base_rate × demand_factor × time_factor

demand_factor = occupancy_rate × surge_multiplier
  - If occupancy < 50%: demand_factor = 1.0
  - If 50% ≤ occupancy < 80%: demand_factor = 1.2
  - If occupancy ≥ 80%: demand_factor = 1.5

time_factor = hours_parked × discount_factor
  - First hour: 1.0
  - 2-4 hours: 0.95 (5% discount)
  - 4+ hours: 0.90 (10% discount)
```

**Subscription Models**
```
- Monthly Unlimited: Fixed price, unlimited parking
- Reserved Space: Dedicated space, guaranteed availability
- Flex Pass: X hours per month, use anytime
- Business Account: Multiple vehicles, corporate billing
```

#### 5.2.3 Payment Processing Flow

```
1. Entry
   ├─→ Scan ticket/plate
   ├─→ Record entry time
   └─→ Issue temporary access token

2. Parking Duration
   ├─→ Real-time cost calculation
   ├─→ Send cost updates (optional)
   └─→ Monitor overstay

3. Pre-Exit
   ├─→ Calculate total charges
   ├─→ Present payment options
   ├─→ Process payment
   └─→ Issue exit authorization (valid 15 min)

4. Exit
   ├─→ Validate exit authorization
   ├─→ Open gate/barrier
   ├─→ Record exit time
   ├─→ Send receipt
   └─→ Close transaction
```

#### 5.2.4 License Plate Recognition (LPR) Payment

```
Flow:
1. Entry camera captures plate
2. OCR extracts plate number
3. Check for registered account
4. If registered:
   - Grant automatic entry
   - Start timer
   - Link to payment method
5. Exit camera captures plate
6. Calculate charges
7. Auto-charge payment method
8. Open exit gate
9. Send receipt via email/SMS

Accuracy Requirements:
- Plate recognition: >98%
- Character recognition: >99.5%
- Validation: Cross-check with vehicle database
```

### 5.3 Violation Management

#### 5.3.1 Overstay Detection

```
Monitoring:
- Compare actual dwell time vs. paid time
- Grace period: 15 minutes
- Warning notification at 80% of time limit
- Overstay alert at expiration + grace period

Penalties:
- First 30 min overstay: 1.5× hourly rate
- 30-60 min: 2.0× hourly rate
- 60+ min: 2.5× hourly rate + potential citation
```

#### 5.3.2 Unauthorized Parking

```
Detection:
- Reserved space occupied without reservation
- Disabled space used without permit
- EV space used by non-EV vehicle
- Parking outside marked lines

Actions:
- Capture evidence (photos, video)
- Issue warning notification
- Generate citation (if no response)
- Tow vehicle (extreme cases)
```

---



## 9. API Specification

### 9.1 Find Available Spaces

#### Request

```http
GET /v1/lots/search?lat=37.7749&lng=-122.4194&radius=1000&vehicle_type=sedan&requires_ev=true
Authorization: Bearer {token}
```

#### Response

```json
{
  "status": "success",
  "results": 15,
  "spaces": [
    {
      "space_id": "SP-EV-042",
      "lot_id": "lot-downtown-01",
      "lot_name": "Downtown Parking Center",
      "location": {
        "lat": 37.7749,
        "lng": -122.4194,
        "distance": 150,
        "distance_unit": "meters"
      },
      "availability": {
        "status": "AVAILABLE",
        "confidence": 0.98,
        "last_updated": "2025-12-26T14:35:00Z"
      },
      "features": {
        "ev_charger": {
          "available": true,
          "type": "Level2",
          "power": 11,
          "connector": "CCS2"
        },
        "covered": true
      },
      "pricing": {
        "hourly_rate": 5.00,
        "ev_charging_rate": 0.35,
        "currency": "USD"
      },
      "score": 92.5
    }
  ]
}
```

### 9.2 Create Reservation

#### Request

```http
POST /v1/reservations
Authorization: Bearer {token}
Content-Type: application/json

{
  "lot_id": "lot-downtown-01",
  "space_id": "SP-EV-042",
  "start_time": "2025-12-26T15:00:00Z",
  "duration": 120,
  "vehicle": {
    "license_plate": "ABC123",
    "type": "sedan"
  },
  "user": {
    "email": "user@example.com",
    "phone": "+1-555-0100"
  },
  "payment_method": "pm_card_visa_4242"
}
```

#### Response

```json
{
  "status": "success",
  "reservation": {
    "id": "res_20251226_4A8F",
    "confirmation_code": "RES-4A8F",
    "space_id": "SP-EV-042",
    "lot_id": "lot-downtown-01",
    "start_time": "2025-12-26T15:00:00Z",
    "end_time": "2025-12-26T17:00:00Z",
    "grace_period": 15,
    "status": "confirmed",
    "cost": {
      "subtotal": 18.00,
      "tax": 1.58,
      "total": 19.58,
      "currency": "USD"
    },
    "payment": {
      "method": "card",
      "last4": "4242",
      "status": "authorized"
    },
    "access": {
      "code": "4582",
      "qr_code": "https://api.parking.example.com/qr/RES-4A8F"
    },
    "created_at": "2025-12-26T14:40:00Z"
  }
}
```

### 9.3 Get Real-Time Occupancy

#### Request

```http
GET /v1/lots/lot-downtown-01/occupancy
Authorization: Bearer {token}
```

#### Response

```json
{
  "status": "success",
  "lot_id": "lot-downtown-01",
  "timestamp": "2025-12-26T14:35:00Z",
  "occupancy": {
    "total": 500,
    "available": 87,
    "occupied": 398,
    "reserved": 12,
    "maintenance": 3,
    "rate": 0.796
  },
  "by_type": {
    "standard": {"total": 450, "available": 75},
    "disabled": {"total": 25, "available": 5},
    "ev": {"total": 25, "available": 7}
  },
  "by_floor": [
    {"floor": 1, "available": 12, "total": 150},
    {"floor": 2, "available": 35, "total": 175},
    {"floor": 3, "available": 40, "total": 175}
  ],
  "ev_charging": {
    "total_chargers": 25,
    "in_use": 18,
    "available": 7
  },
  "prediction": {
    "next_hour": 0.82,
    "trend": "increasing"
  }
}
```

---




---

## A.1 Endpoint reference

```http
GET  /parking/v1/lots/{id}/availability         # current availability
POST /parking/v1/reservations                   # create reservation
GET  /parking/v1/reservations/{id}              # reservation status
POST /parking/v1/reservations/{id}/cancel       # cancel reservation
POST /parking/v1/sessions                       # start parking session
POST /parking/v1/sessions/{id}/end              # end session + settle
GET  /parking/v1/sessions/{id}/receipt          # receipt
WS   /parking/v1/availability/stream            # real-time availability
```

Every endpoint follows the discovery convention at `/.well-known/wia-smart-parking`.

## A.2 Reservation lifecycle

`POST /reservations` allocates a space and returns a confirmation code plus a QR payload. State transitions: pending → confirmed → active → completed (or → cancelled / no-show). No-show occurs after a configurable grace window (default 15 minutes); the space is released and the customer charged a no-show fee per the lot operator's policy.

## A.3 Payment integration

The reservation endpoint accepts pre-tokenized payment-method handles (PCI-DSS 4.0 SAQ-A scope). On confirmation the gateway places a hold for the maximum estimated charge; on session end the actual charge is captured and the hold released. Refund flows are gateway-native (Stripe, Adyen, PortOne, Toss) with the WIA settlement audit log mirroring every state.

## A.4 Session API

`POST /sessions` opens a parking session that tracks entry time, dynamic pricing, and EV-charging coupling (if any). `POST /sessions/{id}/end` closes the session and produces a receipt with itemized line items: parking minutes, EV kWh, validation discounts, taxes, fees. Receipts are signed (Ed25519) so a downstream expense system can verify integrity.

## A.5 Webhook events

Webhooks fire on `reservation.confirmed`, `reservation.cancelled`, `session.started`, `session.ended`, `session.exception`. Delivery is at-least-once with HMAC-SHA256 signing per Phase 4 §A.2 (smart-logistics policy reused). Receivers dedupe on `deliveryId`; retries follow the same 1s/4s/16s backoff as the rest of the WIA family.

## A.6 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. Real-time-availability WebSocket connections count separately and are bounded at 50 simultaneous subscriptions per credential.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-parking/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-parking-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-parking-host:1.0.0` ships every smart-parking envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-parking.sh` ships sample envelope generators with no
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
ecosystem. Smart-parking deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
