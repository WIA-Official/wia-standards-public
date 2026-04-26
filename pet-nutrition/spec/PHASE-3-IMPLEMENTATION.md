# WIA-PET-009: Phase 3 - Implementation Guidelines

**Version:** 1.0 | **Status:** Final | **Last Updated:** 2025-12-25

## Best Practices

### 1. Data Storage

**Relational Database Schema:**
```sql
CREATE TABLE pets (
  pet_id VARCHAR(50) PRIMARY KEY,
  owner_id VARCHAR(50) NOT NULL,
  name VARCHAR(100),
  species VARCHAR(20) NOT NULL,
  breed VARCHAR(200),
  birth_date DATE NOT NULL,
  sex VARCHAR(20) NOT NULL,
  microchip_id VARCHAR(15),
  current_weight_kg DECIMAL(5,2),
  ideal_weight_kg DECIMAL(5,2),
  activity_level VARCHAR(20),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  INDEX idx_owner_id (owner_id),
  INDEX idx_microchip (microchip_id)
);

CREATE TABLE weight_history (
  id INT AUTO_INCREMENT PRIMARY KEY,
  pet_id VARCHAR(50) NOT NULL,
  measurement_date TIMESTAMP NOT NULL,
  weight_kg DECIMAL(5,2) NOT NULL,
  bcs INT CHECK (bcs BETWEEN 1 AND 9),
  FOREIGN KEY (pet_id) REFERENCES pets(pet_id),
  INDEX idx_pet_date (pet_id, measurement_date)
);

CREATE TABLE allergies (
  id INT AUTO_INCREMENT PRIMARY KEY,
  pet_id VARCHAR(50) NOT NULL,
  allergen VARCHAR(200) NOT NULL,
  allergen_type VARCHAR(50),
  severity VARCHAR(20),
  symptoms TEXT,
  diagnosed_date DATE,
  FOREIGN KEY (pet_id) REFERENCES pets(pet_id)
);
```

### 2. Security Implementation

**API Key Management:**
```javascript
const crypto = require('crypto');

function generateApiKey() {
  return 'pk_live_' + crypto.randomBytes(24).toString('hex');
}

function hashApiKey(apiKey) {
  return crypto.createHash('sha256').update(apiKey).digest('hex');
}
```

**JWT Token Validation:**
```javascript
const jwt = require('jsonwebtoken');

function validateToken(req, res, next) {
  const authHeader = req.headers.authorization;
  if (!authHeader || !authHeader.startsWith('Bearer ')) {
    return res.status(401).json({ error: 'Missing authorization token' });
  }
  
  const token = authHeader.substring(7);
  try {
    const decoded = jwt.verify(token, process.env.JWT_SECRET);
    req.user = decoded;
    next();
  } catch (err) {
    return res.status(401).json({ error: 'Invalid token' });
  }
}
```

### 3. Validation

**JSON Schema Validation (Node.js):**
```javascript
const Ajv = require('ajv');
const ajv = new Ajv({ allErrors: true });

const petProfileSchema = require('./schemas/pet-profile.json');
const validate = ajv.compile(petProfileSchema);

function validatePetProfile(data) {
  const valid = validate(data);
  if (!valid) {
    return {
      valid: false,
      errors: validate.errors.map(err => ({
        field: err.instancePath,
        issue: err.message,
        value: err.data
      }))
    };
  }
  return { valid: true };
}
```

### 4. Energy Calculation

**RER and DER Formulas:**
```python
def calculate_rer(weight_kg):
    """Calculate Resting Energy Requirement"""
    return 70 * (weight_kg ** 0.75)

def calculate_der(weight_kg, life_stage, activity, neutered):
    """Calculate Daily Energy Requirement"""
    rer = calculate_rer(weight_kg)
    
    multipliers = {
        'puppy_0_4months': 3.0,
        'puppy_4_12months': 2.0,
        'adult_sedentary_neutered': 1.2,
        'adult_sedentary_intact': 1.4,
        'adult_moderate_neutered': 1.4,
        'adult_moderate_intact': 1.6,
        'adult_active_neutered': 1.6,
        'adult_active_intact': 1.8,
        'senior_neutered': 1.1,
        'senior_intact': 1.3,
        'weight_loss': 1.0,
        'pregnant_weeks1_4': 1.8,
        'pregnant_weeks5_6': 2.0,
        'pregnant_weeks7_9': 2.5,
        'lactating': 4.0
    }
    
    key = f"{life_stage}_{activity}_{'neutered' if neutered else 'intact'}"
    multiplier = multipliers.get(key, 1.4)
    
    return rer * multiplier
```

### 5. Testing

**Unit Test Example:**
```javascript
const { calculateNutritionalRequirements } = require('../services/nutrition');

describe('Nutritional Requirements Calculation', () => {
  test('calculates correct DER for adult neutered dog', () => {
    const pet = {
      species: 'dog',
      weight: { current: 25.0 },
      activityLevel: 'moderate',
      sex: 'neutered_male',
      birthDate: '2018-05-10'
    };
    
    const result = calculateNutritionalRequirements(pet);
    
    expect(result.dailyCaloricNeeds).toBeCloseTo(1033, 0);
    expect(result.macronutrients.protein.minimum).toBeGreaterThan(40);
  });
  
  test('validates required fields', () => {
    const invalidPet = { species: 'dog' };
    
    expect(() => {
      calculateNutritionalRequirements(invalidPet);
    }).toThrow('Missing required fields');
  });
});
```

## Reference Implementation Walkthrough

The reference engine is structured as four pure modules so it can be embedded in clinic kiosks, mobile apps, and label printers without any service dependency:

```
nutrition-engine/
├── src/
│   ├── species-rules.ts     # AAFCO + FEDIAF rule tables
│   ├── der-calculator.ts    # DER, RER, MER formulas
│   ├── ration-builder.ts    # Convert nutrient targets → grams of recipe
│   └── safety-checks.ts     # Toxic ingredients, life-stage limits
└── tests/
    ├── species-rules.spec.ts
    ├── der-calculator.spec.ts
    ├── ration-builder.spec.ts
    └── safety-checks.spec.ts
```

Engineers integrating the engine MUST run the canonical test fixtures bundled in `tests/fixtures/canonical/` before claiming WIA-PET-009 conformance.

## Performance Budgets

| Operation | p50 | p95 | p99 |
|-----------|-----|-----|-----|
| Single ration calculation | 5 ms | 15 ms | 30 ms |
| Plan generation (7 days) | 25 ms | 80 ms | 150 ms |
| Bulk import (1k pets) | 4 s | 12 s | 25 s |
| Webhook delivery | 200 ms | 800 ms | 2000 ms |

Budgets are measured on a 4 vCPU container with 8 GiB RAM. Implementations that exceed the p99 by more than 25% MUST raise a regression alert.

## Observability

Every long-running operation MUST emit OpenTelemetry spans tagged with:

- `wia.pet_009.species`
- `wia.pet_009.life_stage`
- `wia.pet_009.recipe_count`
- `wia.pet_009.warning_count`

Aggregate metrics SHOULD include `wia_pet009_calc_total{result}` and `wia_pet009_warning_total{type}` exposed via the Prometheus exposition format.

## Caching Guidance

Nutrient tables update at most once per quarter. The reference engine MUST cache the species-rules dataset for at least 24 hours per process and MUST invalidate when the upstream `ETag` changes. Cached entries MUST NOT survive a process restart unless the local persistence honours the upstream signature.

## Life-Stage Algorithm

Life-stage selection is the most error-prone part of ration planning. The reference engine implements:

```typescript
function lifeStage(pet: Pet): LifeStage {
  const ageMonths = pet.age.years * 12 + pet.age.months;
  if (pet.species === "dog") {
    if (ageMonths < 12) return "puppy";
    if (ageMonths < (pet.breed.size === "large" ? 24 : 18)) return "adolescent";
    if (ageMonths >= 84) return "senior";
    return "adult";
  }
  if (pet.species === "cat") {
    if (ageMonths < 12) return "kitten";
    if (ageMonths >= 132) return "senior";
    return "adult";
  }
  // exotic species: fall through to species-specific table
  return speciesRules[pet.species].lifeStageFor(ageMonths);
}
```

Reproductive states (pregnant, lactating) override the chronological life stage and unlock a separate calorie / macro envelope.

## Validation Pipeline

Every input MUST traverse the same pipeline before reaching the calculator:

```
parse → schema-validate → range-check → business-rule-check → safety-check → calculate
```

Failures at any stage MUST short-circuit the pipeline and surface a domain-specific error code (see Phase 2 error catalog) rather than a generic 500.

## Logging Recommendations

Per-request logs MUST include `petId`, `tenantId`, `species`, `lifeStage`, `requestId`, `latencyMs`, and `outcome`. Logs MUST NOT include the owner contact block.

## Failure Modes

| Failure | Detection | Recovery |
|---------|-----------|----------|
| Stale nutrient table | Hash mismatch on hourly self-test | Force refresh, reject calculations until pass |
| Recipe inventory drift | POS webhook reconciliation | Mark missing recipes as unavailable |
| OpenAPI / SDK drift | Contract tests on every release | Block release until contract green |
| Owner unsubscribes during plan generation | Webhook 410 Gone | Stop plan, notify clinic, archive partial output |
| Excessive plan-generation cost | Per-tenant budget alarm | Throttle plan generation, surface 429 with hint |
| Multilingual recipe missing translation | Translation lint in CI | Block recipe publish until reviewed |

## Internationalisation Notes

When generating ration plans for non-Latin scripts (Korean, Japanese, Arabic) the engine MUST:

- Round numeric quantities to one decimal place by default
- Honour locale-aware unit display (g vs ounces, kg vs lb)
- Localise meal-time labels using the tenant's locale resource bundle
- Avoid embedding raw English macro names in user-visible output

## Safety-Check Catalog

The reference engine ships a curated catalogue of contraindications:

| Category | Examples | Severity |
|----------|----------|----------|
| Hard-toxic ingredients | xylitol, theobromine for dogs, allium spp for cats | reject |
| Conditional flags | grain-free + giant breed (DCM concern) | warn-and-confirm |
| Drug-nutrient interaction | warfarin + high vitamin K | clinician approval required |
| Life-stage mismatch | puppy formula for senior dog | warn |
| Body-condition mismatch | high-calorie recipe for obese pet | reject unless explicit override |

Each entry MUST point at a primary regulatory or veterinary nutrition reference (AAFCO, FEDIAF, ACVN, ESVCN). The catalogue updates quarterly and is shipped under `data/safety-catalog.v{N}.json` with semantic versioning so downstream applications can pin to a known-good version while reviewing changes.

## Reference Recipe Library

Bundled recipes serve as the engine's smoke-test corpus:

- `dog-adult-maintenance.recipe.json` — Mid-energy, neutered male, moderate activity
- `dog-puppy-large-breed.recipe.json` — Controlled calcium ratio for skeletal development
- `cat-adult-renal-support.recipe.json` — Reduced phosphorus, increased moisture
- `cat-kitten-growth.recipe.json` — High-protein, taurine-rich
- `rabbit-adult-fiber-rich.recipe.json` — Long-stem fiber emphasis

Each reference recipe MUST pass all safety checks for its declared life stage and serve as the regression target for engine refactors.

## Property-Based Tests

Beyond example-based tests the suite MUST include property-based tests asserting invariants:

```typescript
import fc from "fast-check";

test("DER >= RER for any healthy pet", () => {
  fc.assert(fc.property(
    arbitraryHealthyPet(),
    pet => {
      const r = calculateNutritionalRequirements(pet);
      return r.dailyCaloricNeeds >= r.restingEnergyRequirement;
    }
  ));
});

test("plan total kcal within +/- 5% of DER * days", () => {
  fc.assert(fc.property(
    arbitraryHealthyPet(),
    fc.constantFrom(7, 14, 30),
    (pet, days) => {
      const plan = generatePlan(pet, days);
      const total = plan.days.reduce((s, d) => s + d.totalKcal, 0);
      const target = pet.der * days;
      return Math.abs(total - target) / target <= 0.05;
    }
  ));
});
```

Property tests catch regressions that example tests miss, especially around boundary conditions (very small or very large pets, extreme activity levels).

## Reproducibility & Determinism

Two engine invocations with identical inputs MUST produce identical outputs:

- All randomness (recipe rotation, meal-time jitter) MUST be derived from a seed in the request.
- Floating-point math MUST be IEEE 754 double precision and avoid order-dependent reductions.
- Reference fixtures pin both inputs and expected outputs so any drift is caught in CI.

## Threading Model

The reference engine is purely synchronous and re-entrant. Hosts that need parallelism wrap the engine in a worker pool; the engine itself never spawns threads or maintains shared mutable state.

## Capacity Planning

| Workload | Throughput per pod (4 vCPU) | Notes |
|----------|------------------------------|-------|
| Single calculation | 200 RPS sustained | Mostly CPU-bound |
| Plan generation (7 days) | 40 RPS sustained | Memory + CPU |
| Bulk import (1k pets) | 1 batch every 4 s | I/O-bound on registry |

Operators MUST run a load test against representative traffic shapes before promoting to production.

---

**弘益人間 · Benefit All Humanity**  
© 2025 WIA | WIA-PET-009 v1.0 Phase 3
