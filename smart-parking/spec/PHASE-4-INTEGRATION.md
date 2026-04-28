# WIA-AUTO-013 — Phase 4: Integration

> Smart-parking canonical Phase 4: ecosystem integration (OCPP + city + nav + privacy).

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


## 7. EV Charging Integration

### 7.1 Charging Station Types

```
Level 1 (AC):
- Power: 1.4 kW
- Voltage: 120V
- Charging Time: 40-50 hours (full charge)
- Use Case: Overnight parking

Level 2 (AC):
- Power: 7-22 kW
- Voltage: 240V
- Charging Time: 4-10 hours (full charge)
- Use Case: Workplace, shopping centers

DC Fast Charging:
- Power: 50-350 kW
- Voltage: 400-800V DC
- Charging Time: 20-60 minutes (80% charge)
- Use Case: Highway rest stops, quick stops
```

### 7.2 Connector Standards

```
Connector Types:
├── Type 1 (SAE J1772): North America, Japan (AC)
├── Type 2 (Mennekes): Europe, global (AC)
├── CCS Combo 1: North America (DC)
├── CCS Combo 2: Europe (DC)
├── CHAdeMO: Japan, some global (DC)
├── Tesla Supercharger: Tesla vehicles (DC)
└── GB/T: China (AC and DC)
```

### 7.3 Charging Session Management

#### 7.3.1 Session Initiation

```json
{
  "charging_session": {
    "session_id": "CHG-20251226-7B3D",
    "space_id": "SP-EV-042",
    "charger_id": "EVSE-L2-18",
    "vehicle": {
      "license_plate": "EV-XYZ-789",
      "battery_capacity": 75,
      "current_soc": 35,
      "target_soc": 80,
      "connector_type": "CCS2"
    },
    "start_time": "2025-12-26T14:00:00Z",
    "estimated_completion": "2025-12-26T16:30:00Z",
    "pricing": {
      "energy_rate": 0.35,
      "currency": "USD",
      "unit": "kWh",
      "parking_included": true
    }
  }
}
```

#### 7.3.2 Real-Time Monitoring

```json
{
  "charging_status": {
    "session_id": "CHG-20251226-7B3D",
    "status": "charging",
    "current_power": 11.5,
    "energy_delivered": 22.3,
    "current_soc": 52,
    "estimated_completion": "2025-12-26T16:15:00Z",
    "cost_so_far": 7.81,
    "elapsed_time": 65
  }
}
```

#### 7.3.3 Smart Charging Features

```python
def optimize_charging_schedule(vehicles, grid_capacity, electricity_prices):
    """
    Optimize EV charging schedule to minimize cost and balance load
    """
    # Sort vehicles by departure time
    vehicles.sort(key=lambda v: v.departure_time)

    schedule = []

    for vehicle in vehicles:
        # Calculate required energy
        energy_needed = (vehicle.target_soc - vehicle.current_soc) * vehicle.battery_capacity / 100

        # Find optimal charging slots
        optimal_slots = find_lowest_price_slots(
            start=vehicle.arrival_time,
            end=vehicle.departure_time,
            energy=energy_needed,
            prices=electricity_prices
        )

        # Check grid capacity constraints
        feasible_slots = filter_by_capacity(
            slots=optimal_slots,
            existing_schedule=schedule,
            grid_capacity=grid_capacity
        )

        # Assign charging schedule
        schedule.append({
            'vehicle': vehicle.id,
            'slots': feasible_slots,
            'cost': calculate_cost(feasible_slots, electricity_prices)
        })

    return schedule
```

### 7.4 Vehicle-to-Grid (V2G) Integration

```
Bidirectional Charging:
- Vehicle charges from grid during low-demand periods
- Vehicle supplies power to grid during peak demand
- Balance load and reduce grid stress
- Provide revenue opportunity for EV owners

Requirements:
- Bidirectional charger (ISO 15118)
- V2G-capable vehicle
- Grid interconnection agreement
- Real-time pricing signals
```

---



## 10. Security and Privacy

### 10.1 Data Security

#### 10.1.1 Encryption

```
Data in Transit:
- TLS 1.3 for all API communications
- Certificate pinning for mobile apps
- Mutual TLS for server-to-server

Data at Rest:
- AES-256 encryption for database
- Encrypted backups
- Key rotation every 90 days
```

#### 10.1.2 Authentication & Authorization

```
User Authentication:
- OAuth 2.0 / OpenID Connect
- Multi-factor authentication (MFA)
- Biometric authentication (mobile apps)

API Authentication:
- API keys with rate limiting
- JWT tokens with short expiration
- Role-based access control (RBAC)

Permissions:
├── User: View availability, make reservations, payment
├── Operator: Manage lot, view analytics, set pricing
├── Admin: Full system access, user management
└── API: Read occupancy, limited write access
```

### 10.2 Privacy Protection

#### 10.2.1 Personal Data Handling

```
Data Minimization:
- Collect only necessary information
- Anonymous occupancy tracking (no PII)
- Aggregate analytics data

Data Retention:
- Transaction records: 7 years (compliance)
- Session logs: 90 days
- Video footage: 30 days (security)
- Anonymous analytics: Indefinite

User Rights (GDPR/CCPA):
- Right to access personal data
- Right to deletion (after legal retention)
- Right to data portability
- Right to opt-out of analytics
```

#### 10.2.2 License Plate Data

```
Protection Measures:
- Hash license plates for anonymity
- Limit access to authorized personnel
- Audit all access to plate data
- Automatic deletion after parking session
- Encrypted storage

Legal Compliance:
- Comply with local privacy laws
- Obtain user consent for LPR
- Provide opt-out mechanism
- Regular privacy audits
```

### 10.3 Security Best Practices

```
System Security:
├── Regular security audits
├── Penetration testing
├── Vulnerability scanning
├── Incident response plan
├── DDoS protection
└── Intrusion detection system (IDS)

IoT Security:
├── Secure boot for sensors
├── Firmware signature verification
├── Regular firmware updates
├── Network segmentation
└── Anomaly detection
```

---




---

## A.1 EV-charging integration (OCPP)

Smart-parking integrates with EV chargers via OCPP 1.6J and OCPP 2.0.1. The integration layer handles charger registration, session pairing with the parking session, real-time power-level reporting, and billing reconciliation. Plug-and-charge (ISO 15118) is supported when both the vehicle and the charger advertise the capability.

## A.2 City / traffic-management integration

City integrations expose the occupancy aggregate stream to municipal traffic-management systems via a one-way bridge. The bridge carries the lot id, the operator id, and the aggregate counter; private space ids and reservation details do not cross the bridge. Operators that opt out of the city feed can disable it at the lot level.

## A.3 Navigation app integration

The availability stream feeds Apple Maps, Google Maps, Waze, Naver Map, Kakao Map, and TomTom through their respective parking-data partner programs. Each consumer receives the canonical aggregate envelope (Phase 1 §A.5); per-partner adapters translate the envelope onto the partner's wire format. Adapters live outside the conformance scope but the envelope they consume is conformance-tested.

## A.4 Privacy and compliance cross-walk

| Concern              | Standard / Regulation                          |
|----------------------|------------------------------------------------|
| Personal data (EU)   | EU GDPR + national transpositions              |
| Personal data (US)   | CCPA / CPRA                                    |
| Personal data (KR)   | PIPA                                           |
| Payment              | PCI-DSS 4.0 SAQ-A                              |
| EV-charging billing  | OCPP 2.0.1 + ISO 15118                         |
| Accessible parking   | ADA Title III (US), EN 17210 (EU), KS A 0001 (KR) |

License-plate-based detection (LPR) is opt-in per the lot operator and is gated by a regional privacy-impact assessment when it crosses retention thresholds.

## A.5 Reference container, CLI, governance

The reference container at `wia/smart-parking-host:1.0.0` ships every Phase 2 endpoint with mock data and feeds the conformance suite. The companion CLI at `cli/smart-parking.sh` ships sample envelope generators for sensor readings, reservations, sessions, and aggregate streams. WIA Standards composition: WIA-INTENT for workload intent, WIA-OMNI-API for credential storage, WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for federation handshake.


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
