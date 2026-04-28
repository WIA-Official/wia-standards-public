# WIA-IND-008 — Phase 2: API

> Appliance, recipe-management, inventory-tracking, and energy-management API surface — each presented as a worked endpoint specification.

## 7. Appliance Types

### 7.1 Smart Oven

#### 7.1.1 Specifications
```json
{
  "type": "smart-oven",
  "power_rating_w": 3500,
  "voltage": 220,
  "capacity_liters": 70,
  "temperature_range_celsius": [30, 300],
  "temperature_accuracy": 5,
  "heating_elements": {
    "upper": 1500,
    "lower": 1500,
    "convection_fan": 500
  },
  "modes": [
    "conventional",
    "convection",
    "fan-assisted",
    "grill",
    "steam",
    "air-fry",
    "dehydrate",
    "proof",
    "self-clean"
  ],
  "sensors": [
    "temperature_probe",
    "internal_temp_sensor",
    "door_sensor",
    "weight_sensor"
  ],
  "preheating_time": {
    "to_180C": 720,
    "to_220C": 900,
    "to_260C": 1080
  }
}
```

#### 7.1.2 Energy Calculations

```
Preheat Energy (kWh):
  E_preheat = (P × t_preheat) / 3600000
  where P = power rating (W), t = time (seconds)

Cooking Energy (kWh):
  E_cook = (P_avg × t_cook × duty_cycle) / 3600000
  where duty_cycle = 0.3-0.7 (thermostat cycling)

Total Energy:
  E_total = E_preheat + E_cook

Example (Baking at 180°C for 40 minutes):
  E_preheat = (3500 × 720) / 3600000 = 0.7 kWh
  E_cook = (3500 × 2400 × 0.5) / 3600000 = 1.17 kWh
  E_total = 1.87 kWh
```

### 7.2 Smart Cooktop (Induction)

#### 7.2.1 Specifications
```json
{
  "type": "induction-cooktop",
  "power_rating_w": 7400,
  "zones": 4,
  "zone_configuration": [
    {"id": "zone-1", "diameter_cm": 21, "power_w": 2300, "boost_w": 3700},
    {"id": "zone-2", "diameter_cm": 18, "power_w": 1800, "boost_w": 3000},
    {"id": "zone-3", "diameter_cm": 18, "power_w": 1800, "boost_w": 3000},
    {"id": "zone-4", "diameter_cm": 14, "power_w": 1500, "boost_w": 2000}
  ],
  "power_levels": 17,
  "features": [
    "pan-detection",
    "power-boost",
    "keep-warm",
    "timer-per-zone",
    "child-lock",
    "overflow-detection",
    "auto-shutoff"
  ],
  "efficiency": 0.90,
  "response_time_seconds": 3
}
```

#### 7.2.2 Cooking Power Profiles

```
High Heat (Boiling water, searing):
  Power: 100% (2.3 kW)
  Temperature: 200-300°C
  Usage: Short bursts (5-10 min)

Medium-High (Stir-frying):
  Power: 70-80% (1.6-1.8 kW)
  Temperature: 160-200°C
  Usage: Active cooking (10-20 min)

Medium (Sautéing, pan-frying):
  Power: 50-60% (1.1-1.4 kW)
  Temperature: 120-160°C
  Usage: Most cooking (15-30 min)

Low-Medium (Simmering):
  Power: 30-40% (0.7-0.9 kW)
  Temperature: 80-100°C
  Usage: Gentle cooking (30-60 min)

Low (Keep warm):
  Power: 10-20% (0.2-0.5 kW)
  Temperature: 60-80°C
  Usage: Extended periods
```

### 7.3 Smart Refrigerator

#### 7.3.1 Specifications
```json
{
  "type": "smart-refrigerator",
  "power_rating_w": 200,
  "annual_consumption_kwh": 350,
  "total_capacity_liters": 635,
  "zones": [
    {
      "name": "refrigerator-main",
      "capacity_liters": 420,
      "temp_range_celsius": [0, 7],
      "humidity_control": true
    },
    {
      "name": "freezer",
      "capacity_liters": 215,
      "temp_range_celsius": [-24, -15],
      "fast_freeze": true
    },
    {
      "name": "flex-zone",
      "capacity_liters": 80,
      "temp_range_celsius": [-18, 5],
      "convertible": true
    }
  ],
  "smart_features": [
    "internal-cameras",
    "inventory-tracking",
    "expiry-monitoring",
    "door-open-alerts",
    "temperature-alerts",
    "recipe-suggestions",
    "shopping-list"
  ],
  "cameras": 3,
  "door_sensors": 4
}
```

#### 7.3.2 Food Storage Guidelines

```json
{
  "storage_zones": {
    "upper_shelves": {
      "temp_celsius": 4,
      "items": ["leftovers", "drinks", "ready-to-eat"],
      "shelf_life_days": 3-7
    },
    "middle_shelves": {
      "temp_celsius": 4,
      "items": ["dairy", "eggs", "deli-meats"],
      "shelf_life_days": 7-14
    },
    "lower_shelves": {
      "temp_celsius": 2,
      "items": ["raw-meat", "raw-fish", "raw-poultry"],
      "shelf_life_days": 1-3
    },
    "crisper_drawers": {
      "temp_celsius": 4,
      "humidity": "high",
      "items": ["vegetables", "fruits"],
      "shelf_life_days": 5-14
    },
    "door_bins": {
      "temp_celsius": 6,
      "items": ["condiments", "juices", "butter"],
      "shelf_life_days": 30-180
    }
  }
}
```

### 7.4 Smart Dishwasher

#### 7.4.1 Specifications
```json
{
  "type": "smart-dishwasher",
  "power_rating_w": 2400,
  "water_consumption_liters": 9.5,
  "capacity_place_settings": 14,
  "programs": [
    {"name": "eco", "duration_min": 210, "temp_celsius": 50, "energy_kwh": 0.92},
    {"name": "auto", "duration_min": 150, "temp_celsius": 65, "energy_kwh": 1.35},
    {"name": "intensive", "duration_min": 165, "temp_celsius": 70, "energy_kwh": 1.65},
    {"name": "quick", "duration_min": 58, "temp_celsius": 60, "energy_kwh": 1.10},
    {"name": "delicate", "duration_min": 120, "temp_celsius": 45, "energy_kwh": 0.85}
  ],
  "features": [
    "auto-dosing",
    "soil-sensor",
    "load-detection",
    "half-load",
    "delay-start",
    "hygiene-plus",
    "extra-dry"
  ],
  "noise_level_db": 42
}
```

---


## 8. Recipe Management

### 8.1 Recipe Execution Engine

#### 8.1.1 Execution Flow
```
1. Recipe Loading
   ├─ Parse recipe JSON
   ├─ Validate structure
   ├─ Check appliance availability
   └─ Verify ingredient inventory

2. Pre-cooking Preparation
   ├─ Scale recipe if needed
   ├─ Generate mise en place checklist
   ├─ Preheat appliances
   └─ Set up timers

3. Cooking Execution
   ├─ Execute steps sequentially
   ├─ Monitor temperatures
   ├─ Adjust timing based on sensors
   ├─ Provide notifications
   └─ Handle parallel operations

4. Post-cooking
   ├─ Log nutritional data
   ├─ Update inventory
   ├─ Request feedback
   ├─ Suggest pairings
   └─ Clean-up reminders
```

#### 8.1.2 Multi-step Coordination

```typescript
interface CookingCoordinator {
  // Analyze recipe for parallelization
  analyzeSteps(recipe: Recipe): StepGraph;

  // Create optimal timeline
  createTimeline(steps: StepGraph): CookingTimeline;

  // Execute with appliance orchestration
  execute(timeline: CookingTimeline): AsyncGenerator<StepEvent>;

  // Handle dynamic adjustments
  adjustTiming(actual: number, expected: number): void;
}

// Example: Parallel cooking
const timeline = {
  t0: ["preheat_oven_to_180C", "start_rice_cooker"],
  t5: ["prep_vegetables"],
  t10: ["start_boiling_water"],
  t15: ["saute_aromatics"],
  t20: ["combine_ingredients", "put_in_oven"],
  t35: ["remove_from_oven", "rest"],
  t40: ["plate_and_serve"]
};
```

### 8.2 Recipe Database Schema

```sql
-- Recipes table
CREATE TABLE recipes (
  recipe_id VARCHAR(50) PRIMARY KEY,
  name_en VARCHAR(200),
  name_ko VARCHAR(200),
  cuisine VARCHAR(50),
  category VARCHAR(50),
  difficulty ENUM('easy', 'medium', 'hard'),
  servings INT,
  prep_time_min INT,
  cook_time_min INT,
  created_at TIMESTAMP,
  updated_at TIMESTAMP,
  author_id VARCHAR(50),
  rating DECIMAL(3,2),
  times_cooked INT
);

-- Ingredients table
CREATE TABLE ingredients (
  ingredient_id VARCHAR(50) PRIMARY KEY,
  recipe_id VARCHAR(50),
  name VARCHAR(100),
  amount DECIMAL(10,2),
  unit VARCHAR(20),
  optional BOOLEAN,
  notes TEXT,
  FOREIGN KEY (recipe_id) REFERENCES recipes(recipe_id)
);

-- Steps table
CREATE TABLE cooking_steps (
  step_id VARCHAR(50) PRIMARY KEY,
  recipe_id VARCHAR(50),
  step_number INT,
  instruction TEXT,
  duration_min INT,
  temperature_celsius INT,
  appliances JSON,
  technique VARCHAR(50),
  FOREIGN KEY (recipe_id) REFERENCES recipes(recipe_id)
);

-- Nutrition table
CREATE TABLE nutrition_info (
  recipe_id VARCHAR(50) PRIMARY KEY,
  calories INT,
  protein_g DECIMAL(5,1),
  carbs_g DECIMAL(5,1),
  fat_g DECIMAL(5,1),
  fiber_g DECIMAL(5,1),
  sodium_mg INT,
  FOREIGN KEY (recipe_id) REFERENCES recipes(recipe_id)
);
```

---


## 9. Inventory Tracking

### 9.1 Automated Detection

#### 9.1.1 Camera-Based Recognition
```typescript
interface InventoryCamera {
  // Capture images
  captureImage(zone: string): Promise<ImageData>;

  // AI-based recognition
  recognizeItems(image: ImageData): Promise<RecognizedItem[]>;

  // Track changes
  detectChanges(before: ImageData, after: ImageData): ItemChange[];

  // Barcode scanning
  scanBarcode(image: ImageData): Promise<string>;
}

interface RecognizedItem {
  name: string;
  confidence: number; // 0-1
  quantity: number;
  unit: string;
  boundingBox: Rectangle;
  expiryDate?: string; // OCR from package
}
```

#### 9.1.2 Weight-Based Tracking
```
For continuous items (milk, juice, etc.):
  remaining_amount = current_weight / full_weight × original_quantity

For discrete items (eggs, apples, etc.):
  item_count = round(current_weight / average_item_weight)

Consumption rate:
  daily_rate = (initial_amount - current_amount) / days_elapsed
  days_until_empty = current_amount / daily_rate
```

### 9.2 Expiration Management

#### 9.2.1 Shelf Life Database
```json
{
  "dairy": {
    "milk": {"unopened": 7, "opened": 5, "frozen": 90},
    "yogurt": {"unopened": 14, "opened": 7, "frozen": 60},
    "cheese-hard": {"unopened": 180, "opened": 30, "frozen": 180},
    "cheese-soft": {"unopened": 14, "opened": 7, "frozen": null}
  },
  "meat": {
    "beef-raw": {"refrigerated": 3, "frozen": 180},
    "pork-raw": {"refrigerated": 2, "frozen": 120},
    "chicken-raw": {"refrigerated": 2, "frozen": 270},
    "fish-raw": {"refrigerated": 1, "frozen": 90}
  },
  "produce": {
    "lettuce": {"refrigerated": 7, "room-temp": 2},
    "tomatoes": {"refrigerated": 10, "room-temp": 5},
    "bananas": {"refrigerated": 14, "room-temp": 5},
    "apples": {"refrigerated": 30, "room-temp": 7}
  }
}
```

#### 9.2.2 Smart Notifications
```typescript
interface ExpiryNotificationSystem {
  // Priority calculation
  calculatePriority(item: InventoryItem): Priority {
    const daysLeft = item.daysUntilExpiry;
    const value = item.price;
    const perishability = item.category.perishRate;

    return (100 - daysLeft) * value * perishability;
  }

  // Recipe suggestions
  suggestRecipes(expiringItems: InventoryItem[]): Recipe[] {
    return recipeDB.search({
      ingredients: expiringItems.map(i => i.name),
      sortBy: 'ingredient_match_count',
      limit: 10
    });
  }

  // Notification scheduling
  scheduleNotifications(item: InventoryItem): void {
    if (item.daysUntilExpiry == 7) send('low_priority');
    if (item.daysUntilExpiry == 3) send('medium_priority');
    if (item.daysUntilExpiry == 1) send('high_priority');
    if (item.daysUntilExpiry == 0) send('urgent');
  }
}
```

---


## 10. Energy Management

### 10.1 Appliance Energy Profiles

#### 10.1.1 Power Consumption Models

**Oven (Conventional Baking)**
```
P_preheat(t) = P_max                          // Full power during preheat
P_maintain(t) = P_max × duty_cycle            // Cycling to maintain temp
duty_cycle = f(T_target, T_ambient, insulation)

Typical values:
- P_max = 3.5 kW
- duty_cycle = 0.3-0.6 (30-60%)
- Preheat time: 10-15 minutes
- Energy: 0.3 kWh (preheat) + 0.5-1.5 kWh/hour (cooking)
```

**Induction Cooktop**
```
P_zone(t) = P_rated × power_level / max_level × efficiency
efficiency = 0.90 (induction), 0.65 (gas), 0.55 (electric coil)

Energy for boiling 2L water:
- Induction: 0.25 kWh (8 min)
- Electric: 0.35 kWh (12 min)
- Gas: 0.40 kWh (10 min)
```

**Refrigerator**
```
P_avg = (P_compressor × runtime_ratio) + P_lights + P_fans

Annual energy:
E_annual = P_avg × 8760 hours × usage_factor
usage_factor = f(door_openings, ambient_temp, fullness)

Typical: 250-400 kWh/year
```

**Dishwasher**
```
E_cycle = E_water_heating + E_motor + E_heating_dry

E_water_heating = volume × ΔT × 4.186 / 3600
E_motor = P_motor × t_wash
E_heating_dry = P_heater × t_dry

Eco mode: 0.8-1.0 kWh
Normal mode: 1.2-1.5 kWh
Intensive: 1.5-2.0 kWh
```

### 10.2 Energy Optimization Strategies

#### 10.2.1 Load Shifting
```typescript
interface LoadShifter {
  // Analyze energy pricing
  getEnergyPrices(timeRange: TimeRange): PriceSchedule;

  // Find optimal cooking times
  optimizeSchedule(meals: Recipe[], constraints: Constraint[]): Schedule {
    const prices = getEnergyPrices(today);
    const offPeak = prices.filter(p => p.rate < threshold);

    return scheduleWithin(offPeak, meals, constraints);
  }

  // Example: Schedule dishwasher for off-peak
  scheduleDishwasher(mode: 'eco'): void {
    const optimalTime = findLowestRate(next_12_hours);
    dishwasher.setDelayStart(optimalTime);
  }
}

Example schedule:
- Off-peak (23:00-07:00): Dishwasher, slow cooker prep
- Mid-peak (07:00-12:00, 18:00-23:00): Quick cooking
- Peak (12:00-18:00): Minimal heavy appliance use
```

#### 10.2.2 Batch Cooking Optimization
```
Energy savings from batch cooking:

Single meal (roast chicken):
- Preheat: 0.3 kWh
- Cook 1 chicken (1 hour): 1.5 kWh
- Total: 1.8 kWh

Batch cooking (3 chickens):
- Preheat: 0.3 kWh  (same)
- Cook 3 chickens (1.25 hours): 2.0 kWh
- Total: 2.3 kWh

Savings per chicken: (3 × 1.8 - 2.3) / 3 = 1.03 kWh
Efficiency improvement: 43%
```

### 10.3 Energy Monitoring Dashboard

```typescript
interface EnergyDashboard {
  // Real-time monitoring
  currentPower: number;           // Watts
  todayConsumption: number;       // kWh
  monthlyConsumption: number;     // kWh
  projectedMonthly: number;       // kWh

  // Breakdown by appliance
  applianceBreakdown: {
    oven: number,
    cooktop: number,
    refrigerator: number,
    dishwasher: number,
    others: number
  };

  // Cost calculations
  energyCost: {
    today: number,
    month: number,
    projected: number,
    currency: string
  };

  // Recommendations
  suggestions: [
    "Run dishwasher after 11 PM to save 30%",
    "Batch cook on Sunday to save 2.5 kWh/week",
    "Use convection mode to reduce cooking time by 25%"
  ];

  // Comparisons
  comparison: {
    vsLastMonth: number,     // % change
    vsAverage: number,       // % vs similar households
    vsBest: number          // % vs top 10% efficient
  };
}
```

---



## A.1 Endpoint reference

```http
POST /smartkitchen/v1/appliance/discover    # discover appliances on home network
POST /smartkitchen/v1/appliance/{id}/control # send a control command
GET  /smartkitchen/v1/recipe/{id}            # fetch a recipe
POST /smartkitchen/v1/inventory/update       # update fridge/pantry inventory
GET  /smartkitchen/v1/energy/snapshot        # current energy consumption
```

Every endpoint follows the discovery convention at
`/.well-known/wia-smart-kitchen`.

## A.2 Appliance-control endpoint

Appliance control accepts signed commands; the appliance verifies
the signature before acting. Critical commands (oven-heat-on,
range-on) require additional confirmation per the safety protocol
in Phase 3.

## A.3 Recipe-management endpoint

The recipe endpoint serves recipes with i18n support (BCP 47 lang
tags). Recipes are signed by the publishing chef or content
provider; consumers verify before importing.

## A.4 Inventory-tracking endpoint

Inventory tracking accepts updates from fridge cameras, barcode
scanners, and manual entry. The endpoint maintains the canonical
inventory state with reservation tracking when recipe-shopping
mode is active.

## A.5 Energy-management endpoint

Energy management exposes per-appliance and aggregate consumption.
The endpoint feeds into the home energy management system (where
deployed) and into utility-provider demand-response programs.


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-kitchen/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-kitchen-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-kitchen-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/smart-kitchen.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.
