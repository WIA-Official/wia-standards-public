# WIA-IND-008 — Phase 3: Protocol

> Nutritional-analysis, safety-and-compliance, and integration-protocol layer with full audit and consent envelopes.

## 11. Nutritional Analysis

### 11.1 Nutrient Calculation

#### 11.1.1 Ingredient Database
```json
{
  "ingredient_id": "ing-rice-white",
  "name": "white rice",
  "category": "grains",
  "per_100g": {
    "calories": 130,
    "protein_g": 2.7,
    "carbs_g": 28.2,
    "fat_g": 0.3,
    "fiber_g": 0.4,
    "sugar_g": 0.1,
    "sodium_mg": 1,
    "vitamins": {
      "vitamin_a_iu": 0,
      "vitamin_c_mg": 0,
      "vitamin_d_iu": 0,
      "vitamin_b12_mcg": 0,
      "folate_mcg": 3
    },
    "minerals": {
      "calcium_mg": 10,
      "iron_mg": 0.2,
      "magnesium_mg": 12,
      "potassium_mg": 35,
      "zinc_mg": 0.5
    }
  },
  "cooking_factors": {
    "cooked_weight_multiplier": 2.5,  // Rice absorbs water
    "nutrient_retention": 0.95
  }
}
```

#### 11.1.2 Recipe Nutrition Calculation
```typescript
function calculateRecipeNutrition(recipe: Recipe): NutritionInfo {
  let total = initializeNutrition();

  for (const ingredient of recipe.ingredients) {
    // Get base nutrition per 100g
    const baseNutrition = ingredientDB.get(ingredient.id);

    // Scale to actual amount
    const factor = ingredient.amount / 100;

    // Apply cooking method adjustments
    const retention = cookingRetention[recipe.cookingMethod];

    // Accumulate
    total.calories += baseNutrition.calories * factor;
    total.protein_g += baseNutrition.protein_g * factor * retention.protein;
    total.vitamins.vitamin_c_mg += baseNutrition.vitamins.vitamin_c_mg
                                    * factor * retention.vitamin_c;
    // ... etc for all nutrients
  }

  // Divide by servings for per-serving values
  return divideByServings(total, recipe.servings);
}

// Cooking retention factors
const cookingRetention = {
  raw: 1.0,
  steaming: { protein: 0.95, vitamin_c: 0.85 },
  boiling: { protein: 0.90, vitamin_c: 0.60 },
  frying: { protein: 0.92, vitamin_c: 0.70 },
  roasting: { protein: 0.95, vitamin_c: 0.75 },
  grilling: { protein: 0.90, vitamin_c: 0.65 }
};
```

### 11.2 Dietary Tracking

#### 11.2.1 Daily Goals
```typescript
interface DietaryGoals {
  calories: { target: number, min: number, max: number };
  macros: {
    protein_g: { target: number, percent: number },
    carbs_g: { target: number, percent: number },
    fat_g: { target: number, percent: number }
  };
  fiber_g: number;
  sodium_mg: { max: number };
  sugar_g: { max: number };
  water_ml: number;
}

// Example: Balanced diet for 70kg person
const balancedDiet: DietaryGoals = {
  calories: { target: 2000, min: 1800, max: 2200 },
  macros: {
    protein_g: { target: 105, percent: 21 },  // 420 cal / 2000 = 21%
    carbs_g: { target: 250, percent: 50 },    // 1000 cal / 2000 = 50%
    fat_g: { target: 64, percent: 29 }        // 580 cal / 2000 = 29%
  },
  fiber_g: 30,
  sodium_mg: { max: 2300 },
  sugar_g: { max: 50 },
  water_ml: 2000
};
```

#### 11.2.2 Meal Logging
```typescript
interface MealLog {
  date: string;
  meals: {
    breakfast: {
      time: string,
      items: FoodItem[],
      nutrition: NutritionInfo
    },
    lunch: {
      time: string,
      items: FoodItem[],
      nutrition: NutritionInfo
    },
    dinner: {
      time: string,
      items: FoodItem[],
      nutrition: NutritionInfo
    },
    snacks: {
      times: string[],
      items: FoodItem[],
      nutrition: NutritionInfo
    }
  };
  dailyTotal: NutritionInfo;
  goalProgress: {
    calories: number,      // % of target
    protein: number,
    carbs: number,
    fat: number
  };
  recommendations: string[];
}
```

---


## 12. Safety and Compliance

### 12.1 Safety Systems

#### 12.1.1 Automatic Shutoff
```typescript
interface SafetyMonitor {
  // Time-based shutoff
  maxCookingTime: {
    oven: 240,          // 4 hours
    cooktop: 120,       // 2 hours
    microwave: 60       // 1 hour
  };

  // Temperature limits
  maxTemperatures: {
    oven: 300,          // °C
    cooktop: 280,
    oil_temp: 200       // Safety for deep frying
  };

  // Anomaly detection
  detectAnomalies(): void {
    if (temperature > threshold && no_user_activity) {
      this.emergencyShutoff();
    }
    if (unusual_power_spike) {
      this.alert('ELECTRICAL_FAULT');
    }
    if (smoke_detected) {
      this.shutoffAll();
      this.alert('FIRE_RISK');
    }
  }
}
```

#### 12.1.2 Child Safety
```typescript
interface ChildSafety {
  childLock: boolean;
  allowedAppliances: string[];
  maxTemperature: number;
  requireAdultApproval: boolean;

  authorizeOperation(appliance: string, user: User): boolean {
    if (!this.childLock) return true;
    if (user.age >= 18) return true;
    if (this.allowedAppliances.includes(appliance)) {
      return this.requestParentApproval(user, appliance);
    }
    return false;
  }
}
```

### 12.2 Food Safety

#### 12.2.1 Temperature Monitoring
```json
{
  "safe_cooking_temperatures": {
    "poultry": 74,
    "ground_meat": 71,
    "beef_steaks": 63,
    "pork": 71,
    "fish": 63,
    "eggs": 71,
    "leftovers": 74
  },
  "danger_zone": {
    "min_celsius": 4,
    "max_celsius": 60,
    "max_time_hours": 2
  },
  "hot_holding": {
    "min_celsius": 60
  },
  "cold_storage": {
    "refrigerator": 4,
    "freezer": -18
  }
}
```

#### 12.2.2 Cross-Contamination Prevention
```typescript
interface CrossContaminationTracker {
  // Track cutting board usage
  cuttingBoards: {
    red: { lastUsed: 'raw-meat', cleanedAt: null },
    green: { lastUsed: 'vegetables', cleanedAt: timestamp },
    blue: { lastUsed: 'fish', cleanedAt: null }
  };

  // Alert system
  checkSafety(board: string, foodType: string): Alert | null {
    const board = this.cuttingBoards[color];

    if (board.cleanedAt == null && board.lastUsed != foodType) {
      return {
        level: 'WARNING',
        message: `Board used for ${board.lastUsed}. Clean before using for ${foodType}.`
      };
    }

    return null;
  }
}
```

---


## 13. Integration Protocols

### 13.1 Communication Standards

#### 13.1.1 MQTT Topics
```
wia/kitchen/{home_id}/appliance/{appliance_id}/status
wia/kitchen/{home_id}/appliance/{appliance_id}/command
wia/kitchen/{home_id}/recipe/current
wia/kitchen/{home_id}/inventory/update
wia/kitchen/{home_id}/energy/realtime
wia/kitchen/{home_id}/notifications
```

#### 13.1.2 REST API Endpoints
```
GET    /api/v1/appliances
GET    /api/v1/appliances/{id}
POST   /api/v1/appliances/{id}/control
GET    /api/v1/appliances/{id}/status

GET    /api/v1/recipes
GET    /api/v1/recipes/{id}
POST   /api/v1/recipes/{id}/start
GET    /api/v1/recipes/search?q={query}

GET    /api/v1/inventory
POST   /api/v1/inventory/items
PUT    /api/v1/inventory/items/{id}
DELETE /api/v1/inventory/items/{id}

GET    /api/v1/energy/today
GET    /api/v1/energy/history
GET    /api/v1/energy/forecast

GET    /api/v1/nutrition/daily
POST   /api/v1/nutrition/log
```

### 13.2 WIA Standard Integration

#### 13.2.1 WIA-INTENT Integration
```typescript
// Natural language cooking commands
"Preheat oven to 180 degrees"
  → intent: appliance.control
  → appliance: oven
  → action: set_temperature
  → value: 180

"What's in my fridge?"
  → intent: inventory.query
  → location: refrigerator
  → response: inventory_list

"Suggest dinner recipes"
  → intent: recipe.suggest
  → meal: dinner
  → constraints: available_ingredients
```

#### 13.2.2 WIA-OMNI-API Integration
```typescript
interface WIAKitchenAPI {
  // Universal appliance control
  control(appliance: string, action: Action): Promise<Result>;

  // Recipe execution
  cook(recipe: Recipe): AsyncGenerator<CookingEvent>;

  // Inventory management
  inventory: {
    list(): Promise<InventoryItem[]>,
    add(item: Item): Promise<void>,
    update(id: string, changes: Partial<Item>): Promise<void>
  };

  // Energy monitoring
  energy: {
    current(): Promise<PowerStatus>,
    history(range: DateRange): Promise<EnergyData[]>
  };
}
```

---



## A.1 Nutritional-analysis protocol

The nutritional-analysis protocol takes a meal record (recipe ×
serving count + non-recipe inputs) and computes per-meal and
per-day nutrient totals. The reference data source is USDA FDC
(Food Data Central) for primary nutrient information; Codex
Alimentarius and EU 1169/2011 inform the daily-reference-intake
percentages.

## A.2 Safety-and-compliance protocol

Kitchen-safety events (gas leak, smoke detection, oven left on
beyond programmed time) emit signed safety envelopes that flow
to the home occupants apps, to facility safety systems where
applicable, and (with consent) to emergency contacts.

## A.3 Integration protocol with external services

External services include grocery delivery (Walmart Grocery,
Coupang Eats Mart, etc.), meal-kit subscriptions (HelloFresh,
KurlyMeal, etc.), and recipe content providers. Integration
envelopes carry the external-service identity, the order
identity, and the matched-inventory entries so the closed-loop
between recipe selection and shopping is auditable.

## A.4 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second
seen-nonce cache.


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
