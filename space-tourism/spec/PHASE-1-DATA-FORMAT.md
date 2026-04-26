# WIA-SPACE-014 — Phase 1: Data Format Specification

**Standard**: WIA-SPACE-014 (Space Tourism)
**Phase**: 1 of 4 — Data Format
**Version**: 1.0.0
**Status**: Draft
**Philosophy**: 弘益人間 — make space tourism safe, accessible, and beneficial.

---

## 1. Scope

Phase 1 defines the data shapes that every WIA-SPACE-014 conformant
operator MUST be able to read and emit, covering the passenger journey
from booking through post-flight debrief.

| Object family | Purpose |
|---------------|---------|
| **Passenger Record** | Identity, medical, training and waiver state for one passenger |
| **Vehicle Manifest** | The vehicle, mission profile, seat assignments and cargo |
| **Mission Profile** | Trajectory class, duration, on-orbit duty roster |
| **Safety Event** | Incident, near-miss, anomaly captured against the mission |
| **Insurance Record** | Coverage scope, beneficiary, claims handling pointer |

Out of scope for Phase 1: API endpoints (Phase 2), federation between
operators (Phase 3), regulatory bridges (Phase 4).

---

## 2. Encoding Rules

* UTF-8 JSON per IETF RFC 8259, key style `snake_case`.
* Timestamps RFC 3339 in UTC.
* Identifiers URI-shaped per RFC 3986.
* Crew/passenger identifiers SHOULD be DID-style URIs to avoid carrying
  passport numbers in flight data.
* Mass values are kilograms with three-decimal precision; volume uses
  cubic metres; force uses Newtons; pressure uses kilopascals; G-load uses
  multiples of standard gravity (`g`, 9.806 65 m/s²).
* Coordinate frames: ITRF (Earth-fixed) for ground positions, J2000 for
  on-orbit positions. The frame MUST be carried in every coordinate object.

### 2.1 Versioning

Every top-level object carries `wia_space_tourism_version` (semver). A
receiver MUST refuse a major version it does not implement.

```json
"wia_space_tourism_version": "1.0.0"
```

---

## 3. Passenger Record

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "passenger_record",
  "passenger_id": "did:wia:passenger:01HXY…",
  "issued_at": "2026-01-15T10:30:00Z",
  "demographics": {
    "age_band": "30-39",
    "height_cm": 178,
    "mass_kg": 76.4,
    "preferred_language": "ko"
  },
  "medical": {
    "fitness_class": "Class-2 Suborbital",
    "last_examined_at": "2025-12-01",
    "examiner_id": "did:wia:flight-surgeon:09…",
    "g_tolerance_g": 4.5,
    "valsalva_pass": true,
    "vestibular_screen": "pass",
    "contraindications": []
  },
  "training": {
    "modules_completed": ["G-tolerance", "Egress", "Suit Donning", "Microgravity Familiarisation"],
    "egress_drill_seconds": 27,
    "centrifuge_max_g": 5.0,
    "neutral_buoyancy_minutes": 90
  },
  "documents": {
    "informed_consent_signed_at": "2025-12-15T09:00:00Z",
    "waiver_version": "WIA-SPACE-014-W-1.0",
    "next_of_kin_contact_id": "did:wia:contact:abc…"
  },
  "accessibility_profile": "did:wia:a11y:passenger:01HXY…"
}
```

### 3.1 Fitness Classes

| Class | Profile envelope |
|-------|------------------|
| Class-1 Sub-orbital | up to 5 g sustained, 4 minutes microgravity |
| Class-2 Sub-orbital | up to 4 g sustained, 6 minutes microgravity |
| Class-3 Orbital     | up to 3.5 g sustained, < 30 days on-orbit |
| Class-4 Lunar       | Class-3 plus radiation tolerance certificate |

### 3.2 Accessibility Linkage

`accessibility_profile` references a WIA-ACCESSIBILITY record describing
seating, suit interface, and on-board communication accommodations. The
operator MUST honour the linked profile or refuse to manifest the
passenger.

---

## 4. Vehicle Manifest

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "vehicle_manifest",
  "manifest_id": "vmfst_01HXY…",
  "vehicle_id": "did:wia:vehicle:lev-3-tail-N12T",
  "mission_id": "msn_2026-04-27-LEV03-01",
  "departure_site": "iata:KSC",
  "departure_at": "2026-04-27T09:30:00Z",
  "return_site": "iata:KSC",
  "expected_return_at": "2026-04-27T10:50:00Z",
  "seats": [
    { "seat_id": "S1", "occupant_id": "did:wia:passenger:01HXY…", "egress_priority": 1 },
    { "seat_id": "S2", "occupant_id": "did:wia:passenger:02HXY…", "egress_priority": 2 }
  ],
  "cargo": [
    { "item_id": "exp-001", "mass_kg": 12.5, "stowage": "lower-bay", "hazmat": false }
  ],
  "consumables": {
    "oxygen_kg": 22.0,
    "potable_water_kg": 8.0,
    "co2_scrubber_hours": 36
  }
}
```

Egress priority numbers are unique per manifest and MUST mirror the
operator's drilled egress order.

---

## 5. Mission Profile

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "mission_profile",
  "mission_id": "msn_2026-04-27-LEV03-01",
  "class": "Sub-orbital",
  "duration_minutes": 80,
  "max_apogee_km": 105,
  "max_g_ascent": 3.8,
  "max_g_descent": 5.4,
  "microgravity_seconds": 240,
  "duty_roster": [
    { "phase": "ingress",  "start_offset_s":   -1800, "duration_s":  1500 },
    { "phase": "ascent",   "start_offset_s":       0, "duration_s":   180 },
    { "phase": "coast",    "start_offset_s":     180, "duration_s":   240 },
    { "phase": "descent",  "start_offset_s":     420, "duration_s":   330 },
    { "phase": "recovery", "start_offset_s":     750, "duration_s":  1200 }
  ]
}
```

`start_offset_s` is relative to engine ignition (T-0). Negative values
denote pre-launch operations.

---

## 6. Safety Event

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "safety_event",
  "event_id": "evt_01HXY…",
  "mission_id": "msn_2026-04-27-LEV03-01",
  "captured_at": "2026-04-27T09:33:12Z",
  "severity": "anomaly",
  "category": "ECLSS",
  "description": "CO2 scrubber bank A drew 12% above predicted current.",
  "affected_seat_ids": [],
  "auto_actions_taken": ["bank-B switched in", "alert dispatched"],
  "narrative_followup_required": true
}
```

Severity uses ICAO-style ladder `observation`, `anomaly`, `incident`,
`accident`. Operators MUST publish at least an `observation`-level summary
within 30 days of every flight.

---

## 7. Insurance Record

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "insurance_record",
  "policy_id": "ins_01HXY…",
  "underwriter_id": "did:wia:insurer:09…",
  "passenger_id": "did:wia:passenger:01HXY…",
  "mission_id": "msn_2026-04-27-LEV03-01",
  "coverage": {
    "life_cover_currency": "USD",
    "life_cover_amount": 1500000,
    "medical_cover_amount": 250000,
    "disability_cover_amount": 750000
  },
  "beneficiary_id": "did:wia:contact:abc…",
  "claims_endpoint": "https://claims.example/wia-space-tourism/policy/ins_01HXY…"
}
```

The standard does not mandate currency. Operators MAY publish FX rules
separately.

---

## 8. Schema Files

JSON Schema 2020-12 documents are served from
`https://wiastandards.com/space-tourism/schemas/`. Implementations
SHOULD bundle local copies for offline validation.

---

## 9. Conformance

A Phase 1 conformant implementation MUST:

1. Round-trip every object family byte-identically through encode/decode.
2. Reject objects missing required fields per JSON Schema.
3. Honour the canonicalisation rules of §2 when computing signatures.
4. Treat unknown optional fields as non-fatal.

---

## 10. References

* IETF RFC 8259 — JSON
* IETF RFC 3339 — Date/Time
* IETF RFC 3986 — URI Generic Syntax
* W3C DID 1.0
* JSON Schema Draft 2020-12
* ICAO Annex 13 — Aircraft accident reporting (severity ladder reference)

---

## Appendix A — Worked Passenger Record (Class-3 Orbital)

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "passenger_record",
  "passenger_id": "did:wia:passenger:01HYG…",
  "issued_at": "2026-02-01T00:00:00Z",
  "demographics": {
    "age_band": "40-49",
    "height_cm": 172,
    "mass_kg": 71.0,
    "preferred_language": "en"
  },
  "medical": {
    "fitness_class": "Class-3 Orbital",
    "last_examined_at": "2026-01-10",
    "examiner_id": "did:wia:flight-surgeon:21…",
    "g_tolerance_g": 3.5,
    "valsalva_pass": true,
    "vestibular_screen": "pass",
    "contraindications": [],
    "radiation_certificate": {
      "issued_at": "2025-11-12",
      "valid_until": "2027-11-12",
      "model": "ICRP-118 / GCR + SPE",
      "lifetime_dose_msv": 412.5,
      "ceiling_msv": 1000.0
    }
  },
  "training": {
    "modules_completed": [
      "G-tolerance",
      "Egress",
      "Suit Donning",
      "Microgravity Familiarisation",
      "Docking Procedures",
      "Emergency Decompression",
      "Fire in Spacecraft",
      "Solar Particle Event Shelter Drill"
    ],
    "egress_drill_seconds": 22,
    "centrifuge_max_g": 5.5,
    "neutral_buoyancy_minutes": 600,
    "evak_certified": true
  },
  "documents": {
    "informed_consent_signed_at": "2026-01-12T10:00:00Z",
    "waiver_version": "WIA-SPACE-014-W-1.0",
    "next_of_kin_contact_id": "did:wia:contact:09…"
  },
  "accessibility_profile": "did:wia:a11y:passenger:01HYG…"
}
```

## Appendix B — Worked Vehicle Manifest (4 passengers + 1 commander)

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "vehicle_manifest",
  "manifest_id": "vmfst_01HYG…",
  "vehicle_id": "did:wia:vehicle:orb-7-tail-N42O",
  "mission_id": "msn_2026-05-12-ORB07-01",
  "departure_site": "iata:KSC",
  "departure_at": "2026-05-12T14:00:00Z",
  "return_site": "iata:KSC",
  "expected_return_at": "2026-05-22T17:30:00Z",
  "seats": [
    { "seat_id": "C1", "occupant_id": "did:wia:crew:cmd-09",         "egress_priority": 1, "role": "commander" },
    { "seat_id": "S1", "occupant_id": "did:wia:passenger:01HYG…",    "egress_priority": 2 },
    { "seat_id": "S2", "occupant_id": "did:wia:passenger:02HYG…",    "egress_priority": 3 },
    { "seat_id": "S3", "occupant_id": "did:wia:passenger:03HYG…",    "egress_priority": 4 },
    { "seat_id": "S4", "occupant_id": "did:wia:passenger:04HYG…",    "egress_priority": 5 }
  ],
  "cargo": [
    { "item_id": "exp-001", "mass_kg": 12.5, "stowage": "lower-bay", "hazmat": false },
    { "item_id": "med-002", "mass_kg":  3.0, "stowage": "med-locker", "hazmat": false }
  ],
  "consumables": {
    "oxygen_kg": 410.0,
    "potable_water_kg": 240.0,
    "co2_scrubber_hours": 540
  }
}
```

## Appendix C — Canonicalisation for Signatures

When a record's signature must be re-verified, implementations MUST:

1. Sort object keys lexicographically by Unicode code point.
2. Use compact JSON form (no insignificant whitespace).
3. Render numbers using shortest round-trip form for floats; integers
   without leading zeros.
4. Encode arrays preserving authored order.
5. UTF-8 encode the result with no BOM.
6. Hash with SHA-256 prefixed by the literal ASCII string
   `WIA-SPACE-014/1.0\n` for domain separation.

Implementations that round-trip the worked records in Appendix A and B
through their encode/decode path MUST produce byte-identical output.

## Appendix D — Reserved Severity & Category Tokens

The following safety-event tokens are reserved by Phase 1; future minor
versions add new tokens but never remove old ones.

Severity ladder (see §6):

| Token | Operator response window |
|-------|--------------------------|
| `observation` | 30-day public summary |
| `anomaly`     | 14-day root-cause review |
| `incident`    | 24-hour regulator notification |
| `accident`    | Immediate regulator + SAR + public notification |

Category tokens (the third-level slug used in `category`):

| Token | Domain |
|-------|--------|
| `ECLSS` | Environmental control / life support |
| `PROP` | Propulsion |
| `GNC` | Guidance, navigation, control |
| `STR` | Structures, pressure vessel |
| `EVA` | Extra-vehicular activity |
| `MED` | Crew or passenger medical |
| `RAD` | Radiation event |
| `COMM` | Communications |
| `RANGE` | Range / launch site safety |
| `SECURITY` | Information security event affecting flight data |

Operators MAY define additional category tokens prefixed with `vendor.`;
peers MUST treat unknown categories as `OTHER` for routing while still
preserving the original token verbatim in stored records.

## Appendix E — Worked Insurance Record (denied claim)

```json
{
  "wia_space_tourism_version": "1.0.0",
  "type": "insurance_record",
  "policy_id": "ins_01HZA…",
  "underwriter_id": "did:wia:insurer:09…",
  "passenger_id": "did:wia:passenger:01HYG…",
  "mission_id": "msn_2026-04-27-LEV03-01",
  "coverage": {
    "life_cover_currency": "USD",
    "life_cover_amount": 1500000,
    "medical_cover_amount": 250000,
    "disability_cover_amount": 750000,
    "exclusions": ["pre-existing-cardiac"]
  },
  "beneficiary_id": "did:wia:contact:abc…",
  "claims_endpoint": "https://claims.example/wia-space-tourism/policy/ins_01HZA…",
  "claim_history": [
    {
      "claim_id": "clm_01HZA…",
      "submitted_at": "2026-05-01T10:00:00Z",
      "status": "denied",
      "reason": "Linked safety_event severity = observation; claim threshold is incident or above per policy schedule."
    }
  ]
}
```

Implementations MUST treat `claim_history` as append-only. Reversal of a
denial is recorded as a new entry whose `status` is `appeal_settled`,
referencing the prior `claim_id` in a `supersedes` field.

弘益人間 — Benefit All Humanity.
