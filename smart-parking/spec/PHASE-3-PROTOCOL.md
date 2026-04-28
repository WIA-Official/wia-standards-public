# WIA-AUTO-013 — Phase 3: Protocol

> Smart-parking canonical Phase 3: protocols (guidance + routing + EV-charging coordination).

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


## 4. Parking Guidance Systems

### 4.1 Guidance Objectives

1. **Minimize search time**: Direct drivers to available spaces quickly
2. **Optimize traffic flow**: Reduce congestion within parking facility
3. **Balance occupancy**: Distribute vehicles evenly across facility
4. **User preferences**: Consider accessibility, EV charging, proximity

### 4.2 Guidance Algorithm

#### 4.2.1 Space Selection Criteria

```python
def calculate_space_score(space, user_location, preferences):
    """
    Calculate suitability score for a parking space (0-100)
    """
    score = 0

    # Distance factor (40% weight)
    distance = haversine_distance(user_location, space.location)
    distance_score = max(0, 100 - (distance / 10))  # Decrease by 10 per 100m
    score += 0.4 * distance_score

    # Availability confidence (30% weight)
    confidence_score = space.detection_confidence * 100
    score += 0.3 * confidence_score

    # Price factor (15% weight)
    if preferences.price_range:
        if preferences.price_range[0] <= space.hourly_rate <= preferences.price_range[1]:
            price_score = 100
        else:
            price_score = 50
        score += 0.15 * price_score

    # Size match (10% weight)
    if space.size >= preferences.vehicle_size:
        size_score = 100
    else:
        size_score = 0
    score += 0.1 * size_score

    # Special requirements (5% weight)
    special_score = 0
    if preferences.requires_ev and space.has_ev_charger:
        special_score += 50
    if preferences.requires_accessibility and space.is_disabled:
        special_score += 50
    score += 0.05 * special_score

    return score
```

#### 4.2.2 Route Planning

```
1. Identify top 5 candidate spaces
2. Calculate optimal route to each
3. Consider:
   - Current traffic conditions
   - One-way restrictions
   - Turn difficulty
   - Time of day
4. Select space with best (score + route_efficiency)
5. Provide turn-by-turn navigation
6. Monitor space availability during transit
7. Reroute if space becomes occupied
```

### 4.3 Dynamic Signage Integration

#### 4.3.1 LED Display Protocol

```
Message Format:
┌────────────────────────────┐
│  Floor 2: ↑                │
│  Available: 15             │
│  Floor 3: ↑                │
│  Available: 8              │
│  Floor 4: FULL             │
└────────────────────────────┘

Update Frequency: Every 5 seconds
Communication: MQTT, RS-485, or IP-based
```

#### 4.3.2 Space-Level Indicators

```
Red LED: OCCUPIED
Green LED: AVAILABLE
Blue LED: RESERVED
Amber LED: CHARGING
Flashing Green: ALLOCATED (guidance in progress)
```

### 4.4 Mobile App Guidance

#### 4.4.1 Navigation Features

- **AR Overlay**: Augmented reality directions to parking space
- **Floor Plans**: Interactive maps with real-time availability
- **Voice Navigation**: Turn-by-turn voice instructions
- **Space Locator**: Save location and get return directions

---



## 6. Real-Time Occupancy Tracking

### 6.1 Data Collection Architecture

```
Sensors → Edge Processing → Gateway → Cloud Platform → Client Applications
   ↓           ↓                ↓          ↓              ↓
Detect      Filter          Aggregate   Store         Display
Status      Noise           Data        Analytics     Real-time
            Debounce        Compress    Process       Updates
```

### 6.2 Data Aggregation

#### 6.2.1 Space-Level Data

```json
{
  "space": {
    "id": "SP-042",
    "lot_id": "lot-downtown-01",
    "status": "OCCUPIED",
    "confidence": 0.98,
    "last_updated": "2025-12-26T14:35:22Z",
    "vehicle": {
      "entry_time": "2025-12-26T13:15:10Z",
      "duration_minutes": 80,
      "license_plate": "ABC123",
      "type": "sedan"
    },
    "sensor": {
      "type": "camera",
      "id": "CAM-L2-03",
      "health": "normal"
    }
  }
}
```

#### 6.2.2 Lot-Level Summary

```json
{
  "lot_summary": {
    "lot_id": "lot-downtown-01",
    "timestamp": "2025-12-26T14:35:00Z",
    "total_spaces": 500,
    "available": 87,
    "occupied": 398,
    "reserved": 12,
    "maintenance": 3,
    "occupancy_rate": 0.796,
    "avg_dwell_time": 135,
    "turnover_rate": 3.2,
    "revenue_current_day": 12450.00,
    "by_floor": [
      {
        "floor": 1,
        "total": 150,
        "available": 12,
        "occupancy_rate": 0.92
      },
      {
        "floor": 2,
        "total": 175,
        "available": 35,
        "occupancy_rate": 0.80
      },
      {
        "floor": 3,
        "total": 175,
        "available": 40,
        "occupancy_rate": 0.77
      }
    ],
    "ev_charging": {
      "total_chargers": 25,
      "in_use": 18,
      "available": 7
    }
  }
}
```

### 6.3 Real-Time Updates

#### 6.3.1 WebSocket Protocol

```javascript
// Client connection
const ws = new WebSocket('wss://api.parking.example.com/v1/realtime');

ws.on('open', () => {
  // Subscribe to lot updates
  ws.send(JSON.stringify({
    action: 'subscribe',
    lot_id: 'lot-downtown-01',
    events: ['occupancy_change', 'status_update']
  }));
});

ws.on('message', (data) => {
  const event = JSON.parse(data);

  if (event.type === 'occupancy_change') {
    updateDisplay(event.lot_id, event.available, event.total);
  }

  if (event.type === 'status_update') {
    updateSpaceStatus(event.space_id, event.status);
  }
});
```

#### 6.3.2 Server-Sent Events (SSE)

```
Client: GET /v1/lots/lot-downtown-01/stream
Server:
  Content-Type: text/event-stream
  Cache-Control: no-cache
  Connection: keep-alive

event: occupancy
data: {"available": 87, "total": 500, "timestamp": "2025-12-26T14:35:00Z"}

event: space_change
data: {"space_id": "SP-042", "status": "OCCUPIED", "timestamp": "2025-12-26T14:35:22Z"}
```

### 6.4 Analytics and Reporting

#### 6.4.1 Key Metrics

```
Operational Metrics:
- Occupancy rate (current, hourly, daily, weekly)
- Average dwell time
- Turnover rate (vehicles per space per day)
- Peak occupancy times
- Space utilization efficiency

Financial Metrics:
- Revenue (hourly, daily, monthly, yearly)
- Average transaction value
- Payment method distribution
- Overstay penalties collected

User Metrics:
- Search time (before using guidance)
- User satisfaction score
- Repeat usage rate
- Mobile app adoption
```

#### 6.4.2 Predictive Analytics

```python
def predict_occupancy(lot_id, timestamp):
    """
    Predict parking occupancy using machine learning
    """
    # Features
    features = {
        'hour_of_day': timestamp.hour,
        'day_of_week': timestamp.weekday(),
        'is_holiday': is_holiday(timestamp),
        'weather': get_weather(timestamp),
        'nearby_events': get_events(lot_id, timestamp),
        'historical_avg': get_historical_avg(lot_id, timestamp)
    }

    # Model prediction (trained Random Forest or LSTM)
    predicted_occupancy = model.predict(features)

    return {
        'predicted_available': predicted_occupancy.available,
        'confidence_interval': predicted_occupancy.ci,
        'recommendation': generate_recommendation(predicted_occupancy)
    }
```

---




---

## A.1 Guidance protocol

Guidance signs (LED panels above lot entrances, intra-lot directional signs, end-of-row counters) consume the lot-level aggregate envelope (Phase 1 §A.5) and update at the same 30-second cadence. The protocol drives sign content via a typed message schema so a vendor swap does not require a re-integration. Off-line fallback shows the last-known counter with a "stale" watermark.

## A.2 Routing-to-space protocol

When a reservation is confirmed the customer's app receives a routing payload that combines lot entrance coordinates, in-lot turn-by-turn directions to the assigned space, and a fallback alternate space if the primary is taken when the customer arrives. Routing is delivered through the navigation provider's deep link (Apple Maps, Google Maps, Naver, Kakao) so the customer stays in their preferred app.

## A.3 Real-time occupancy protocol

Occupancy aggregates flow over MQTT topics `parking/{lotId}/{spaceType}/occupancy` with QoS-1 + TLS 1.3 + gzip-compressed JSON. Subscribers include the operator dashboard, the public-facing availability map, the city traffic-management bridge, and the navigation-app feed. Each subscriber receives the same canonical envelope so consistency holds across surfaces.

## A.4 EV-charging coordination

Spaces marked as EV-charging carry a coupled charging session that runs in lockstep with the parking session. The EV-charging protocol exchanges OCPP 2.0.1 (or 1.6J) messages with the charger and surfaces them as a typed event stream on the WIA side. Exception flows (charger unavailable, customer pulls plug early) reconcile both sessions atomically so billing stays consistent.

## A.5 Dwell-time and turnover protocol

The protocol layer derives dwell-time and turnover-rate metrics from the occupancy state machine (Phase 1 §A.3). Dwell-time is published per-space and aggregated per lot at hourly granularity; turnover-rate at daily granularity. The metrics feed dynamic pricing, capacity planning, and city-level transportation-demand models.

## A.6 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for control-plane calls. Telemetry traffic (sensor → broker) is mTLS authenticated; replays at the telemetry layer are mitigated by per-sensor monotonic counters with skew rejection.


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
