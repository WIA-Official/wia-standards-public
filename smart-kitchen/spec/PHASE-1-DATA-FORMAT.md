# WIA-IND-008 — Phase 1: Data Format

> Smart-kitchen canonical envelopes: terminology, architecture, core component, and appliance-type descriptors.

## 1. Introduction

### 1.1 Purpose

The WIA-IND-008 standard provides a unified framework for smart kitchen technology, enabling:
- Seamless appliance interconnectivity
- Automated cooking with recipe guidance
- Intelligent inventory and food waste management
- Energy optimization across kitchen operations
- Nutritional tracking and dietary compliance
- Enhanced food safety and quality

### 1.2 Philosophy

**弘益人間 (홍익인간)** - "Benefit All Humanity"

This standard embodies the principle that technology should serve humanity by:
- Making nutritious cooking accessible to all skill levels
- Reducing food waste and environmental impact
- Optimizing energy consumption for sustainability
- Ensuring food safety and quality
- Enabling healthy dietary choices
- Fostering culinary education and cultural exchange

### 1.3 Design Principles

1. **Interoperability**: Vendor-neutral protocols for multi-brand ecosystems
2. **User-Centric**: Intuitive interfaces for all age groups and abilities
3. **Safety-First**: Comprehensive safety mechanisms and fail-safes
4. **Sustainability**: Energy efficiency and waste reduction built-in
5. **Privacy**: User data protection and consent management
6. **Extensibility**: Open architecture for future innovations

---


## 2. Scope

### 2.1 Included

- Connected appliance specifications (ovens, cooktops, refrigerators, etc.)
- Recipe data formats and execution protocols
- Inventory tracking and expiration management
- Energy monitoring and optimization algorithms
- Nutritional analysis and dietary tracking
- Safety systems and compliance monitoring
- Integration with home automation platforms
- Multi-language and cultural cuisine support

### 2.2 Excluded

- Specific appliance hardware designs (manufacturer-dependent)
- Food sourcing and supply chain management
- Commercial kitchen and restaurant systems (separate standard)
- Agricultural production and farming technology

---


## 3. Normative References

- ISO 8601: Date and time format
- IEEE 802.11: Wi-Fi standards
- Bluetooth 5.0+: Low-energy communication
- MQTT 3.1.1: IoT messaging protocol
- JSON Schema: Data structure validation
- OAuth 2.0: Authorization framework
- TLS 1.3: Transport security
- USDA FoodData Central: Nutritional database
- IEC 60335: Household appliance safety

---


## 4. Terms and Definitions

### 4.1 Appliance Categories

- **Major Appliances**: Refrigerator, oven, cooktop, dishwasher, range hood
- **Small Appliances**: Microwave, coffee maker, toaster, blender, air fryer
- **Smart Features**: Connectivity, sensors, automation, remote control

### 4.2 Cooking Terms

- **Recipe**: Structured cooking instructions with ingredients and steps
- **Scaling**: Adjusting recipe quantities for different serving sizes
- **Mise en place**: Preparation stage before cooking begins
- **Doneness**: Target state of food (rare, medium, well-done, etc.)
- **Resting**: Post-cooking equilibration period

### 4.3 Technical Terms

- **IoT Gateway**: Hub device for appliance communication
- **Recipe Engine**: Software that interprets and executes recipes
- **Inventory Database**: Storage tracking system with expiration management
- **Energy Profile**: Power consumption characteristics of an appliance
- **Nutritional Parser**: System that calculates meal nutrition from ingredients

---


## 5. Architecture

### 5.1 System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Smart Kitchen Cloud                       │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Recipe DB│  │Inventory │  │Analytics │  │User Prefs│   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
└───────────────────────┬─────────────────────────────────────┘
                        │ HTTPS/MQTT
┌───────────────────────┴─────────────────────────────────────┐
│                    IoT Gateway / Hub                         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Recipe Engine │ Inventory Mgr │ Energy Optimizer    │  │
│  └──────────────────────────────────────────────────────┘  │
└───────┬──────────┬──────────┬──────────┬──────────┬────────┘
        │          │          │          │          │
   ┌────▼───┐ ┌───▼────┐ ┌───▼────┐ ┌───▼────┐ ┌──▼─────┐
   │ Oven   │ │Cooktop │ │ Fridge │ │Dishwash│ │Microwave│
   └────────┘ └────────┘ └────────┘ └────────┘ └─────────┘
```

### 5.2 Communication Layers

#### 5.2.1 Appliance Layer
- Direct appliance control and monitoring
- Sensor data collection
- Status reporting
- Safety monitoring

#### 5.2.2 Gateway Layer
- Protocol translation
- Recipe execution
- Inventory management
- Energy scheduling
- Local processing for low latency

#### 5.2.3 Cloud Layer
- Recipe database and sharing
- Long-term analytics
- Machine learning models
- User account management
- Multi-device synchronization

### 5.3 Data Flow

```
User Input → Recipe Selection → Ingredient Check → Cooking Plan
                                      ↓
                            Inventory Update
                                      ↓
Appliance Control ← Energy Scheduling ← Optimization
         ↓
   Sensor Monitoring → Progress Tracking → Notifications
         ↓
   Completion → Nutritional Log → Analytics → Recommendations
```

---


## 6. Core Components

### 6.1 Smart Appliance Interface

#### 6.1.1 Required Capabilities
```json
{
  "appliance_id": "unique-identifier",
  "type": "oven|cooktop|refrigerator|dishwasher|microwave|...",
  "manufacturer": "brand-name",
  "model": "model-number",
  "firmware_version": "x.y.z",
  "capabilities": {
    "remote_control": true,
    "temperature_control": true,
    "timer_functions": true,
    "sensor_monitoring": true,
    "recipe_execution": true,
    "energy_monitoring": true
  },
  "connectivity": ["wifi", "bluetooth", "zigbee"],
  "power_rating_w": 3500,
  "energy_class": "A+++"
}
```

#### 6.1.2 Control Interface
```typescript
interface ApplianceControl {
  // Power management
  turnOn(): Promise<void>;
  turnOff(): Promise<void>;
  getStatus(): Promise<ApplianceStatus>;

  // Temperature control
  setTemperature(celsius: number): Promise<void>;
  getTemperature(): Promise<number>;

  // Mode control
  setMode(mode: CookingMode): Promise<void>;
  getMode(): Promise<CookingMode>;

  // Timer functions
  setTimer(seconds: number): Promise<void>;
  getRemainingTime(): Promise<number>;

  // Safety
  emergencyStop(): Promise<void>;
  childLock(enabled: boolean): Promise<void>;
}
```

### 6.2 Recipe Data Model

#### 6.2.1 Recipe Structure
```json
{
  "recipe_id": "rec-kimchi-jjigae-001",
  "name": {
    "en": "Kimchi Jjigae",
    "ko": "김치찌개",
    "ja": "キムチチゲ"
  },
  "cuisine": "korean",
  "category": "soup_stew",
  "difficulty": "easy",
  "servings": 4,
  "prep_time_minutes": 15,
  "cook_time_minutes": 30,
  "total_time_minutes": 45,

  "ingredients": [
    {
      "id": "ing-001",
      "name": "kimchi",
      "amount": 300,
      "unit": "g",
      "notes": "well-fermented preferred",
      "optional": false,
      "substitutes": ["fresh-kimchi", "sauerkraut"]
    },
    {
      "id": "ing-002",
      "name": "pork-belly",
      "amount": 200,
      "unit": "g",
      "notes": "thinly sliced",
      "optional": false,
      "substitutes": ["tofu", "beef", "chicken"]
    }
  ],

  "steps": [
    {
      "step_number": 1,
      "instruction": "Cut kimchi and pork belly into bite-size pieces",
      "duration_minutes": 5,
      "appliances": [],
      "temperature": null,
      "technique": "cutting"
    },
    {
      "step_number": 2,
      "instruction": "Sauté pork belly in pot until slightly browned",
      "duration_minutes": 5,
      "appliances": ["cooktop"],
      "temperature": 180,
      "technique": "sauteing",
      "heat_level": "medium-high"
    },
    {
      "step_number": 3,
      "instruction": "Add kimchi and stir-fry for 3 minutes",
      "duration_minutes": 3,
      "appliances": ["cooktop"],
      "temperature": 180,
      "technique": "stir-frying"
    },
    {
      "step_number": 4,
      "instruction": "Add water and bring to boil, then simmer",
      "duration_minutes": 20,
      "appliances": ["cooktop"],
      "temperature": 100,
      "technique": "simmering",
      "heat_level": "medium-low"
    }
  ],

  "nutrition": {
    "per_serving": {
      "calories": 245,
      "protein_g": 18,
      "carbs_g": 12,
      "fat_g": 14,
      "fiber_g": 3,
      "sugar_g": 5,
      "sodium_mg": 890,
      "cholesterol_mg": 45
    }
  },

  "appliances_required": ["cooktop", "cutting-board", "knife", "pot"],
  "tags": ["korean", "spicy", "comfort-food", "quick", "keto-friendly"],
  "allergens": ["pork"],
  "dietary_flags": ["gluten-free", "dairy-free"]
}
```

#### 6.2.2 Recipe Scaling Algorithm

```
For each ingredient:
  scaled_amount = original_amount × (target_servings / original_servings)

For cooking times:
  # Volume-based scaling (for baking, roasting)
  volume_ratio = target_servings / original_servings
  time_multiplier = volume_ratio^(1/3)
  scaled_time = original_time × time_multiplier

  # Surface-area based (for pan-frying, grilling)
  area_ratio = volume_ratio^(2/3)
  scaled_time = original_time × area_ratio

For temperatures:
  # Generally remain constant
  scaled_temperature = original_temperature
```

### 6.3 Inventory Management

#### 6.3.1 Inventory Item Model
```json
{
  "item_id": "inv-001",
  "name": "milk",
  "category": "dairy",
  "quantity": 1,
  "unit": "liter",
  "purchase_date": "2025-12-20",
  "expiry_date": "2025-12-27",
  "days_until_expiry": 0,
  "location": "refrigerator",
  "zone": "main-compartment",
  "barcode": "8801234567890",
  "price": 3500,
  "currency": "KRW",
  "nutritional_info": {
    "calories_per_100ml": 64,
    "protein_g": 3.2,
    "fat_g": 3.6,
    "carbs_g": 4.7
  },
  "storage_temp_celsius": 4,
  "opened": false,
  "opened_date": null,
  "use_within_days_after_opening": 7
}
```

#### 6.3.2 Expiration Monitoring

```
Freshness Index = 100 × (1 - days_elapsed / shelf_life)

Freshness Categories:
- Fresh: 80-100%
- Good: 60-79%
- Fair: 40-59%
- Use Soon: 20-39%
- Expired: 0-19%

Alert Triggers:
- 7 days before expiry: Low priority
- 3 days before expiry: Medium priority
- 1 day before expiry: High priority
- Day of expiry: Critical priority
- After expiry: Urgent removal
```

#### 6.3.3 Shopping List Generation

```typescript
interface ShoppingListGenerator {
  // Analyze meal plan and inventory
  analyzeMealPlan(meals: Recipe[], days: number): Ingredient[];

  // Check current inventory
  checkInventory(required: Ingredient[]): {
    have: Ingredient[],
    need: Ingredient[]
  };

  // Generate optimized list
  generateShoppingList(needed: Ingredient[]): ShoppingList;

  // Organize by store sections
  organizeBySections(list: ShoppingList): SectionizedList;

  // Calculate total cost
  estimateCost(list: ShoppingList): number;
}
```

---



## A.1 Canonical envelope conventions

Every Phase 1 smart-kitchen envelope follows the WIA family baseline:
UTF-8 JSON with RFC 8785 canonicalisation, Ed25519 signatures, ULID
identifiers. Recipe IPs are intellectual property and inherit the
WIA Secure Enclave sealed-data envelope when published under licence.

## A.2 Architecture overview

The smart-kitchen architecture comprises: connected appliances
(oven, fridge, dishwasher, range hood, etc.), recipe management,
inventory tracking (smart-fridge cameras / barcode scanners),
energy management, nutritional analysis, and the integration with
external services (grocery delivery, meal-kit subscriptions).

## A.3 Appliance descriptor envelope

Each appliance publishes a descriptor with manufacturer, model,
firmware version, supported control surfaces, and energy class.
The descriptor is signed by the appliance manufacturer so warranty
and recall management is auditable.

## A.4 Recipe envelope

```json
{
  "wia_smart_kitchen_version": "1.0.0",
  "type": "recipe",
  "recipe_id": "rcp_01HX...",
  "title_i18n": { "en": "...", "ko": "...", "ja": "..." },
  "ingredients": [
    { "name": "eggs", "quantity": 3, "unit": "count" }
  ],
  "instructions": [
    { "step": 1, "appliance_class": "stove_top", "control": "medium-heat", "duration_seconds": 180, "instruction_i18n": { "en": "..." } }
  ],
  "nutrition_per_serving": { "kcal": 300, "protein_g": 22 }
}
```


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
