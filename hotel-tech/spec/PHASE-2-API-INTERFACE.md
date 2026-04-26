# WIA-IND-016 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-016
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Smart Room Technology

### 5.1 IoT Device Integration

#### 5.1.1 Supported Device Types

**Climate Control**:
```json
{
  "deviceType": "thermostat",
  "model": "Nest Thermostat E",
  "protocol": "MQTT",
  "capabilities": {
    "modes": ["heat", "cool", "auto", "off"],
    "temperatureRange": {
      "min": 16,
      "max": 30,
      "unit": "celsius"
    },
    "scheduling": true,
    "occupancyDetection": true,
    "remoteControl": true
  }
}
```

**Lighting Control**:
```json
{
  "deviceType": "lighting",
  "model": "Philips Hue",
  "protocol": "Zigbee",
  "capabilities": {
    "zones": ["bedroom", "bathroom", "closet"],
    "dimming": true,
    "colorTemperature": {
      "min": 2000,
      "max": 6500,
      "unit": "kelvin"
    },
    "scenes": ["welcome", "reading", "evening", "sleep"],
    "voiceControl": true
  }
}
```

**Smart Lock**:
```json
{
  "deviceType": "door-lock",
  "model": "Yale Assure Lock",
  "protocol": "Z-Wave",
  "capabilities": {
    "accessMethods": ["mobile-key", "rfid", "pin", "biometric"],
    "autoLock": true,
    "unlockLogging": true,
    "temporaryAccess": true,
    "integrateWithPMS": true
  }
}
```

#### 5.1.2 Room Scenes

**Pre-defined Scenes**:

1. **Welcome Scene** (activated on check-in):
   ```
   - Lights: 80% brightness, warm white (3000K)
   - Temperature: Set to guest preference or 22°C
   - Curtains: 50% open
   - TV: Display welcome message
   - Music: Soft background music (optional)
   ```

2. **Reading Scene**:
   ```
   - Lights: Bedside lamps 100%, others 30%
   - Color temperature: 4000K
   - Climate: Maintain current setting
   ```

3. **Sleep Scene**:
   ```
   - Lights: All off except nightlight (5%)
   - Temperature: Reduce by 1°C
   - Curtains: Fully closed
   - Do Not Disturb: Activated
   ```

4. **Wake Up Scene**:
   ```
   - Lights: Gradual increase over 15 minutes
   - Curtains: Gradual open over 10 minutes
   - Temperature: Increase to day setting
   - Music: Gentle alarm (if set)
   ```

#### 5.1.3 Energy Management

**Occupancy-Based Control**:
```javascript
function adjustRoomEnergyMode(room, occupancyStatus) {
  if (occupancyStatus === 'occupied') {
    return {
      hvac: 'comfort-mode',
      temperatureRange: [20, 24],
      lighting: 'auto',
      powerOutlets: 'enabled'
    };
  } else if (occupancyStatus === 'vacant-reserved') {
    return {
      hvac: 'eco-mode',
      temperatureRange: [18, 26],
      lighting: 'off',
      powerOutlets: 'standby'
    };
  } else { // vacant-available
    return {
      hvac: 'deep-eco-mode',
      temperatureRange: [16, 28],
      lighting: 'off',
      powerOutlets: 'disabled'
    };
  }
}
```

**Energy Savings Calculation**:
```
Daily Energy Consumption:

Occupied Mode: 25 kWh/day
Eco Mode: 10 kWh/day (60% reduction)
Deep Eco Mode: 3 kWh/day (88% reduction)

Annual Savings (100-room hotel, 70% occupancy):
= 30 vacant rooms × 15 kWh savings × 365 days
= 164,250 kWh/year
= ~$24,600/year (at $0.15/kWh)
```

### 5.2 Voice Assistant Integration

#### 5.2.1 Supported Commands

**Room Control**:
- "Set temperature to 22 degrees"
- "Turn on the lights"
- "Close the curtains"
- "Play some relaxing music"
- "Set wake-up call for 7 AM"

**Hotel Services**:
- "Call the front desk"
- "Order room service"
- "Request housekeeping"
- "Book a spa appointment"
- "Extend my check-out time"

**Information**:
- "What time is breakfast?"
- "Where is the gym?"
- "Recommend a nearby restaurant"
- "Check out times"

#### 5.2.2 Multi-Language Support

Required languages:
- English (en-US, en-GB, en-AU)
- Spanish (es-ES, es-MX)
- French (fr-FR)
- German (de-DE)
- Mandarin Chinese (zh-CN)
- Japanese (ja-JP)
- Korean (ko-KR)

---

## 6. Keyless Entry Systems

### 6.1 Mobile Key Technology

#### 6.1.1 Mobile Key Generation

**Key Creation Process**:
```
1. Guest checks in (online or at desk)
2. PMS triggers key generation
3. Encryption key generated
4. Key sent to guest's mobile app
5. Key activated at check-in time
6. Key expires at check-out + buffer period
```

**Mobile Key Data Structure**:
```json
{
  "keyId": "KEY-20250115-ABC123",
  "guestId": "guest-12345",
  "roomNumber": "305",
  "propertyId": "hotel-001",
  "validFrom": "2025-01-15T15:00:00Z",
  "validUntil": "2025-01-18T13:00:00Z",
  "accessLevel": {
    "room": true,
    "elevator": true,
    "pool": true,
    "gym": true,
    "executiveLounge": true,
    "parkingGate": true
  },
  "encryptionKey": "AES256-encrypted-key",
  "issueDate": "2025-01-14T10:30:00Z",
  "revoked": false
}
```

#### 6.1.2 Security Protocols

**Encryption Standards**:
- AES-256 encryption for key data
- TLS 1.3 for transmission
- Time-based one-time passwords (TOTP)
- Bluetooth Low Energy (BLE) with encrypted handshake

**Access Verification**:
```javascript
function verifyMobileKeyAccess(key, door, timestamp) {
  // Check key validity period
  if (timestamp < key.validFrom || timestamp > key.validUntil) {
    return { granted: false, reason: 'Key expired or not yet valid' };
  }

  // Check if key is revoked
  if (key.revoked) {
    return { granted: false, reason: 'Key has been revoked' };
  }

  // Check room access
  if (door.type === 'room' && door.number !== key.roomNumber) {
    return { granted: false, reason: 'Not authorized for this room' };
  }

  // Check area access
  if (door.type === 'facility') {
    const facilityName = door.name.toLowerCase();
    if (!key.accessLevel[facilityName]) {
      return { granted: false, reason: `Not authorized for ${door.name}` };
    }
  }

  // Log access
  logAccess(key.keyId, door, timestamp, true);

  return { granted: true };
}
```

### 6.2 RFID Card Systems

#### 6.2.1 Card Encoding

**RFID Card Format** (MIFARE Classic):
```
Sector 0, Block 0: Manufacturer data (read-only)
Sector 0, Block 1: Hotel ID + Property ID
Sector 0, Block 2: Room number + Valid from/until
Sector 1, Block 0: Guest ID + Access level bits
Sector 1, Block 1: Encryption checksum
```

**Encoding Process**:
```javascript
function encodeRFIDCard(reservation) {
  const cardData = {
    hotelId: 'HTL001',
    propertyId: reservation.propertyId,
    roomNumber: reservation.roomNumber,
    validFrom: reservation.checkIn,
    validUntil: addHours(reservation.checkOut, 2), // 2-hour buffer
    guestId: reservation.guest.guestId,
    accessLevel: calculateAccessLevel(reservation.guest)
  };

  // Encode with AES-128
  const encodedData = encryptAES128(cardData, HOTEL_MASTER_KEY);

  // Write to card
  return writeToRFIDCard(encodedData);
}
```

### 6.3 Access Logging

#### 6.3.1 Access Log Entry

```json
{
  "logId": "access-20250115-001234",
  "timestamp": "2025-01-15T16:45:32Z",
  "keyId": "KEY-20250115-ABC123",
  "guestId": "guest-12345",
  "location": "Room 305",
  "accessMethod": "mobile-key",
  "result": "granted",
  "deviceId": "lock-305",
  "ipAddress": "10.0.1.105"
}
```

#### 6.3.2 Audit Trail

Access logs SHALL be retained for:
- Minimum 90 days for regulatory compliance
- 1 year for security investigations
- Encrypted at rest
- Access restricted to authorized personnel

---

## 7. Housekeeping Management

### 7.1 Room Assignment

#### 7.1.1 Assignment Algorithm

**Priority Factors**:
```javascript
function assignHousekeepingTasks(rooms, staff) {
  const assignments = [];

  rooms.forEach(room => {
    let priority = 0;

    // Check-in today
    if (room.arrivalToday) priority += 100;

    // VIP guest
    if (room.vipGuest) priority += 50;

    // Checkout dirty room
    if (room.status === 'dirty' && room.departureToday) priority += 80;

    // Stayover service
    if (room.status === 'occupied' && room.stayover) priority += 30;

    // Deep clean scheduled
    if (room.deepCleanScheduled) priority += 40;

    room.priority = priority;
  });

  // Sort by priority
  rooms.sort((a, b) => b.priority - a.priority);

  // Assign to available staff
  // ... assignment logic
}
```

#### 7.1.2 Task Types

| Task Type | Duration | Priority | Requirements |
|-----------|----------|----------|--------------|
| Checkout Clean | 30 min | High | Full clean + inspection |
| Stayover Service | 15 min | Medium | Refresh + tidy |
| Deep Clean | 60 min | Medium | Monthly rotation |
| Rush Clean | 20 min | Urgent | For early check-in |
| Turndown Service | 10 min | Low | Evening service |

### 7.2 Cleaning Standards

#### 7.2.1 Checkout Room Cleaning

**Standard Procedure**:
```
1. Knock and announce (wait 30 seconds)
2. Enter and prop door open
3. Remove all linens and towels
4. Empty all trash bins
5. Clean bathroom (15 min):
   - Toilet, sink, shower/tub
   - Mirror and fixtures
   - Restock amenities
6. Dust and vacuum bedroom (10 min)
7. Make bed with fresh linens
8. Restock coffee/tea
9. Final inspection
10. Update room status in system
```

**Quality Checklist**:
- [ ] Bed properly made (hospital corners)
- [ ] Bathroom spotless
- [ ] All surfaces dusted
- [ ] Carpet vacuumed (no visible dirt)
- [ ] Amenities fully stocked
- [ ] TV and AC remotes present
- [ ] No items left behind
- [ ] Room smells fresh

#### 7.2.2 Stayover Service

```
1. Knock and announce
2. Ask guest preferences
3. If guest absent:
   - Make bed (if unmade)
   - Replace towels (if on floor)
   - Empty trash
   - Refresh amenities
   - Quick tidy
4. Update room status
```

### 7.3 Inventory Management

#### 7.3.1 Linen Management

**Par Levels**:
```
Per Room:
- Sheets: 3 sets
- Pillowcases: 6
- Bath towels: 4
- Hand towels: 4
- Face towels: 4
- Bath mat: 2

Total Property (100 rooms):
- Sheets: 300 sets
- Towels: 1,200 bath + 1,200 hand
- Replacement cycle: 12-18 months
```

**Automated Reorder**:
```javascript
function checkInventoryLevels(inventory, parLevels) {
  const reorderList = [];

  for (const [item, quantity] of Object.entries(inventory)) {
    const par = parLevels[item];
    const reorderPoint = par * 0.3; // 30% of par level

    if (quantity < reorderPoint) {
      reorderList.push({
        item: item,
        currentQty: quantity,
        parLevel: par,
        orderQty: par - quantity,
        urgency: quantity < (reorderPoint * 0.5) ? 'urgent' : 'normal'
      });
    }
  }

  return reorderList;
}
```

---

## 8. Revenue Management

### 8.1 Dynamic Pricing

#### 8.1.1 Pricing Factors

**Demand-Based Pricing Model**:
```
OptimalRate = BaseRate × DemandMultiplier × SeasonalMultiplier ×
              CompetitorAdjustment × SpecialEventMultiplier

Where:
DemandMultiplier = f(CurrentOccupancy, PickupRate, DaysToArrival)
SeasonalMultiplier ∈ [0.8, 1.5]
CompetitorAdjustment ∈ [0.9, 1.1]
SpecialEventMultiplier ∈ [1.0, 2.0]
```

**Demand Multiplier Calculation**:
```javascript
function calculateDemandMultiplier(occupancy, pickupRate, daysToArrival) {
  let multiplier = 1.0;

  // Occupancy-based adjustment
  if (occupancy > 0.90) multiplier = 1.30;
  else if (occupancy > 0.80) multiplier = 1.20;
  else if (occupancy > 0.70) multiplier = 1.10;
  else if (occupancy < 0.50) multiplier = 0.85;
  else if (occupancy < 0.60) multiplier = 0.90;

  // Pickup rate adjustment (how fast bookings are coming in)
  if (pickupRate > 1.5) multiplier *= 1.10; // High pickup
  else if (pickupRate < 0.5) multiplier *= 0.95; // Low pickup

  // Days to arrival adjustment
  if (daysToArrival < 3 && occupancy < 0.70) {
    multiplier *= 0.90; // Last-minute discount
  } else if (daysToArrival > 60) {
    multiplier *= 0.95; // Advance purchase discount
  }

  return multiplier;
}
```

#### 8.1.2 Competitive Pricing

**Rate Shopping**:
```javascript
async function shopCompetitorRates(property, date, roomType) {
  const competitors = property.competitorSet;
  const rates = [];

  for (const competitor of competitors) {
    const rate = await fetchCompetitorRate(competitor.id, date, roomType);
    rates.push({
      competitorId: competitor.id,
      competitorName: competitor.name,
      rate: rate,
      starRating: competitor.stars,
      distance: competitor.distanceKm
    });
  }

  // Calculate competitive position
  const myRate = calculateMyRate(property, date, roomType);
  const avgCompRate = rates.reduce((sum, r) => sum + r.rate, 0) / rates.length;

  return {
    myRate: myRate,
    competitorRates: rates,
    avgCompetitorRate: avgCompRate,
    positioning: myRate / avgCompRate, // 1.0 = at parity
    recommendation: generatePricingRecommendation(myRate, rates)
  };
}
```

### 8.2 Yield Optimization

#### 8.2.1 Length of Stay (LOS) Controls

**Minimum LOS Strategy**:
```javascript
function setMinimumLOS(date, occupancy, demandLevel) {


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
