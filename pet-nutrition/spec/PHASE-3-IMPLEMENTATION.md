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

---

**弘益人間 · Benefit All Humanity**  
© 2025 WIA | WIA-PET-009 v1.0 Phase 3
