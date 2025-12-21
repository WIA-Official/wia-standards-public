# WIA-CARBON-CREDIT-MICRO Specification v1.0

## Document Information

- **Standard**: WIA-CARBON-CREDIT-MICRO
- **Version**: 1.0
- **Status**: Draft
- **Created**: 2025-12
- **Category**: Sustainability

---

## 1. Introduction

### 1.1 Purpose

WIA-CARBON-CREDIT-MICRO defines a standard for measuring, verifying, and rewarding individual-level carbon reduction actions. It democratizes carbon markets by enabling everyday people to participate in climate action and receive tangible benefits.

### 1.2 Scope

This standard covers:
- Micro-action carbon measurement
- Credit calculation methodology
- Verification mechanisms
- Trading and redemption
- Community pooling
- Corporate integration

### 1.3 Problem Statement

Current carbon markets exclude individuals:
- Minimum thresholds too high for personal actions
- Complex verification inaccessible to consumers
- No incentive for daily sustainable choices
- Corporate offsets lack transparency
- Collective small actions go unmeasured

---

## 2. Philosophy

### 2.1 Hongik Ingan Principles

"Benefit all humanity" - Climate justice requires universal participation.

Core beliefs:
- Every action matters, no matter how small
- Climate benefits should be shared equitably
- Technology can make sustainability rewarding
- Collective small actions create massive impact
- Transparency builds trust in carbon accounting

### 2.2 Design Goals

1. **Accessibility**: Anyone can participate
2. **Accuracy**: Science-based calculations
3. **Simplicity**: Easy logging and tracking
4. **Incentive**: Real rewards for real action
5. **Transparency**: Auditable credit lifecycle

---

## 3. Action Categories

### 3.1 Transport Actions

```
Action              Baseline        Emission Factor    Notes
------------------- --------------- ------------------ ----------------
bike_commute        car             0.23 kg/km saved   GPS verified
public_transit      car             0.15 kg/km saved   Ticket verified
walk_commute        car             0.23 kg/km saved   Step tracker
ev_charge_solar     grid_charge     0.4 kg/kWh saved   Meter verified
carpool             solo_drive      0.12 kg/km/person  App verified
remote_work         commute         full trip saved    Calendar verified
```

### 3.2 Energy Actions

```
Action              Baseline        Emission Factor    Notes
------------------- --------------- ------------------ ----------------
solar_generation    grid            0.5 kg/kWh         Meter required
reduced_ac          normal_usage    0.8 kg/hour saved  Smart thermostat
led_replacement     incandescent    0.05 kg/day        One-time action
cold_wash           hot_wash        0.3 kg/load        Smart washer
line_dry            dryer           0.5 kg/load        App confirmation
off_peak_charging   peak            0.1 kg/kWh         Smart meter
```

### 3.3 Food Actions

```
Action              Baseline        Emission Factor    Notes
------------------- --------------- ------------------ ----------------
plant_based_meal    meat_meal       2.5 kg/meal        Receipt/photo
food_waste_saved    wasted          2.0 kg/kg food     Scale verified
local_produce       imported        0.5 kg/kg food     Receipt verified
home_composting     landfill        0.3 kg/kg waste    Bin sensor
meatless_day        avg_diet        4.0 kg/day         Streak tracked
water_bottle_refill plastic_bottle  0.08 kg/refill     App logged
```

### 3.4 Consumer Actions

```
Action              Baseline        Emission Factor    Notes
------------------- --------------- ------------------ ----------------
secondhand_clothing new_clothing    15 kg/item         Receipt required
repair_device       replace         50 kg/device       Service receipt
reusable_bag        plastic_bag     0.04 kg/use        Store verified
refill_container    new_container   0.5 kg/refill      Store verified
digital_receipt     paper_receipt   0.01 kg/receipt    Auto-tracked
bulk_purchase       packaged        0.2 kg/kg          Receipt verified
```

### 3.5 Digital Actions

```
Action              Baseline        Emission Factor    Notes
------------------- --------------- ------------------ ----------------
reduced_streaming   hd_streaming    0.036 kg/hour      App monitored
dark_mode           light_mode      0.005 kg/hour      Device tracked
device_sleep        always_on       0.02 kg/hour       Power monitor
email_cleanup       stored          0.001 kg/email     Auto-tracked
cloud_optimization  unoptimized     variable           Provider API
video_call_no_video with_video      0.05 kg/hour       App tracked
```

### 3.6 Community Actions

```
Action              Baseline        Emission Factor    Notes
------------------- --------------- ------------------ ----------------
tree_planted        no_tree         25 kg/year/tree    Photo verified
community_cleanup   no_cleanup      0.5 kg/kg trash    Event logged
garden_maintained   lawn            5 kg/sqm/year      Photo verified
carshare_provided   unused_car      0.5 kg/trip        Platform data
education_given     no_education    10 kg/attendee     Event verified
policy_advocacy     no_action       variable           Verified petition
```

---

## 4. Credit Calculation

### 4.1 Core Formula

```
Credits = Baseline_Emission - Actual_Emission

Where:
- Credits in kg CO2e
- 1 MCC (Micro Carbon Credit) = 1 kg CO2e
```

### 4.2 Calculation Object

```json
{
  "calculation_id": "CALC-2025-12345",
  "user_id": "USER-12345",
  "action": {
    "type": "bike_commute",
    "category": "TRANSPORT",
    "timestamp": "2025-12-21T08:30:00Z",
    "duration_minutes": 30,
    "distance_km": 10
  },
  "baseline": {
    "mode": "car_average",
    "emission_factor": 0.23,
    "unit": "kg_co2e_per_km",
    "source": "EPA_2024"
  },
  "actual": {
    "mode": "bicycle",
    "emission_factor": 0,
    "unit": "kg_co2e_per_km"
  },
  "result": {
    "baseline_emission_kg": 2.3,
    "actual_emission_kg": 0,
    "credits_earned_mcc": 2.3,
    "credits_earned_kg_co2e": 2.3
  },
  "verification": {
    "method": "gps_tracking",
    "confidence": 0.95,
    "verified_by": "app_algorithm",
    "evidence_hash": "sha256:abc123..."
  }
}
```

### 4.3 Verification Levels

```
Level    Method              Confidence    Credit Multiplier
-------- ------------------- ------------- -----------------
AUTO     App automatic       0.70          0.7x
SELF     User declaration    0.50          0.5x
DEVICE   IoT sensor          0.90          1.0x
THIRD    External verify     0.95          1.1x
AUDIT    Full audit trail    0.99          1.2x
```

---

## 5. Credit Token Format

### 5.1 Token Structure

```json
{
  "token_id": "MCC-2025-1221-00001",
  "type": "MICRO_CARBON_CREDIT",
  "version": "1.0",
  "value": {
    "amount": 2.3,
    "unit": "kg_co2e",
    "mcc_equivalent": 2.3
  },
  "origin": {
    "action_type": "bike_commute",
    "calculation_id": "CALC-2025-12345",
    "user_id": "USER-12345",
    "timestamp": "2025-12-21T08:30:00Z",
    "location": {
      "country": "KR",
      "city": "Seoul"
    }
  },
  "verification": {
    "level": "DEVICE",
    "confidence": 0.90,
    "verifier": "transit_app_v2"
  },
  "status": "ACTIVE",
  "metadata": {
    "created": "2025-12-21T08:35:00Z",
    "expires": null,
    "tradeable": true,
    "poolable": true
  }
}
```

### 5.2 Token States

```
PENDING     - Awaiting verification
ACTIVE      - Verified and usable
POOLED      - Contributed to community pool
REDEEMED    - Used for reward
TRADED      - Transferred to another user
RETIRED     - Permanently offset
EXPIRED     - Past validity period
CANCELLED   - Invalidated
```

---

## 6. Marketplace

### 6.1 Redemption Options

```json
{
  "redemption_options": [
    {
      "id": "CASH_OUT",
      "name": "Cash Withdrawal",
      "rate": 0.01,
      "currency": "USD",
      "unit": "per_mcc",
      "minimum": 100,
      "processing_days": 3
    },
    {
      "id": "OFFSET_DONATE",
      "name": "Donate to Offset Project",
      "rate": 1.0,
      "bonus": 0.1,
      "projects": ["reforestation", "ocean_cleanup", "renewable"]
    },
    {
      "id": "PRODUCT_EXCHANGE",
      "name": "Sustainable Products",
      "catalog": "https://marketplace.wia.org/products",
      "discount_rate": 0.02
    },
    {
      "id": "TRANSIT_CREDIT",
      "name": "Public Transit Credit",
      "rate": 0.015,
      "partners": ["seoul_metro", "london_tfl", "nyc_mta"]
    }
  ]
}
```

### 6.2 Trading Mechanism

```json
{
  "trade": {
    "trade_id": "TRADE-2025-98765",
    "type": "PEER_TO_PEER",
    "seller": {
      "user_id": "USER-12345",
      "tokens": ["MCC-2025-1221-00001"],
      "amount_mcc": 2.3
    },
    "buyer": {
      "user_id": "USER-67890",
      "payment": {
        "amount": 0.023,
        "currency": "USD"
      }
    },
    "rate": 0.01,
    "fee": 0.001,
    "status": "COMPLETED",
    "timestamp": "2025-12-21T10:00:00Z"
  }
}
```

---

## 7. Community Pools

### 7.1 Pool Structure

```json
{
  "pool_id": "POOL-SEOUL-2025",
  "name": "Seoul Green Challenge 2025",
  "goal": {
    "target_mcc": 1000000,
    "deadline": "2025-12-31"
  },
  "current": {
    "total_mcc": 567890,
    "contributors": 12345,
    "progress_percent": 56.8
  },
  "reward": {
    "type": "TREE_PLANTING",
    "description": "1 tree per 100 MCC pooled",
    "partner": "Seoul Metropolitan Government"
  },
  "leaderboard": [
    {"user_id": "USER-11111", "contributed": 1500},
    {"user_id": "USER-22222", "contributed": 1200}
  ]
}
```

### 7.2 Corporate Pools

```json
{
  "corporate_pool": {
    "pool_id": "CORP-SAMSUNG-2025",
    "company": "Samsung Electronics",
    "program": "Green Employee Challenge",
    "participants": 50000,
    "goal_mcc": 500000,
    "matching": {
      "enabled": true,
      "ratio": 2.0,
      "cap_mcc": 1000000
    },
    "incentives": [
      {"threshold": 100, "reward": "eco_badge"},
      {"threshold": 500, "reward": "extra_pto_day"},
      {"threshold": 1000, "reward": "donation_match"}
    ]
  }
}
```

---

## 8. Gamification

### 8.1 Achievements

```json
{
  "achievements": [
    {
      "id": "FIRST_CREDIT",
      "name": "First Step",
      "description": "Earn your first carbon credit",
      "icon": "[OK]",
      "mcc_reward": 1
    },
    {
      "id": "WEEK_STREAK",
      "name": "Consistent Saver",
      "description": "Log actions for 7 consecutive days",
      "icon": "[7]",
      "mcc_reward": 10
    },
    {
      "id": "CATEGORY_MASTER",
      "name": "All-Rounder",
      "description": "Earn credits in all 6 categories",
      "icon": "[*]",
      "mcc_reward": 50
    },
    {
      "id": "CARBON_NEUTRAL",
      "name": "Net Zero Day",
      "description": "Offset a full day's average emissions",
      "icon": "[0]",
      "mcc_reward": 25
    },
    {
      "id": "COMMUNITY_LEADER",
      "name": "Community Champion",
      "description": "Top 10 in a community pool",
      "icon": "[#]",
      "mcc_reward": 100
    }
  ]
}
```

### 8.2 Levels

```
Level    Title              MCC Required    Perks
-------- ------------------ --------------- ----------------------
1        Seedling           0               Basic tracking
2        Sprout             100             Weekly insights
3        Sapling            500             Trading enabled
4        Tree               2000            Premium features
5        Forest             10000           Verified badge
6        Ecosystem          50000           Ambassador status
```

---

## 9. API Reference

### 9.1 Core Endpoints

```
Action Logging
--------------
POST   /api/v1/actions              Log new action
GET    /api/v1/actions              List user actions
GET    /api/v1/actions/{id}         Get action details

Credits
-------
GET    /api/v1/credits              List user credits
GET    /api/v1/credits/balance      Get credit balance
POST   /api/v1/credits/transfer     Transfer credits

Marketplace
-----------
GET    /api/v1/marketplace/options  List redemption options
POST   /api/v1/marketplace/redeem   Redeem credits
GET    /api/v1/marketplace/trades   List available trades
POST   /api/v1/marketplace/trade    Execute trade

Pools
-----
GET    /api/v1/pools                List community pools
GET    /api/v1/pools/{id}           Get pool details
POST   /api/v1/pools/{id}/join      Join a pool
POST   /api/v1/pools/{id}/contribute  Contribute credits

Stats
-----
GET    /api/v1/stats/user           User statistics
GET    /api/v1/stats/global         Global statistics
GET    /api/v1/stats/leaderboard    Leaderboards
```

### 9.2 SDK Example

```javascript
import { CarbonCredit } from '@anthropic/wia-carbon-credit-micro';

// Initialize
const cc = new CarbonCredit({ apiKey: 'your-key' });

// Log an action
const action = await cc.logAction({
  type: 'bike_commute',
  distance_km: 10,
  verification: {
    method: 'gps',
    data: gpsTrackData
  }
});

// Check balance
const balance = await cc.getBalance();
console.log(`Credits: ${balance.mcc} MCC`);

// Redeem
await cc.redeem({
  option: 'TRANSIT_CREDIT',
  amount_mcc: 50,
  partner: 'seoul_metro'
});

// Join pool
await cc.joinPool('POOL-SEOUL-2025');
await cc.contributeToPool('POOL-SEOUL-2025', 100);
```

---

## 10. Interoperability

### 10.1 With WIA-OCEAN-PLASTIC-TRACK

- Plastic collection earns credits
- Shared environmental metrics
- Combined impact reports

### 10.2 With Transit Systems

- Automatic action logging
- Integrated payment credits
- Route optimization data

### 10.3 With Banking/Fintech

- Green spending rewards
- Carbon footprint in statements
- Investment offset options

### 10.4 External Standards

- GHG Protocol alignment
- IPCC emission factors
- ISO 14064 compatibility
- Verra/Gold Standard offset integration

---

## 11. Privacy & Security

### 11.1 Data Handling

```
Data Type         Storage     Sharing      Retention
----------------- ----------- ------------ -----------
Location data     Encrypted   Anonymized   1 year
Action logs       Encrypted   User consent 5 years
Credit balance    Encrypted   Private      Indefinite
Trading history   Encrypted   Anonymized   5 years
```

### 11.2 User Controls

- Opt-out of location tracking
- Anonymous participation option
- Data export/deletion rights
- Sharing preferences

---

## 12. Implementation Levels

### 12.1 Level 1: Basic

- Manual action logging
- Simple credit calculation
- Basic redemption options

### 12.2 Level 2: Standard

- IoT device integration
- Trading marketplace
- Community pools
- Gamification

### 12.3 Level 3: Complete

- Blockchain ledger
- AI verification
- Corporate integration
- Global marketplace

---

## 13. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12 | Initial release |

---

WIA-CARBON-CREDIT-MICRO: Every action counts. Make yours count more.
