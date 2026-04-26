# WIA-PET-009: 3단계 - 구현 가이드라인

**버전:** 1.0 | **상태:** 최종 | **최종 업데이트:** 2025-12-25

## 모범 사례

### 1. 데이터 저장

**관계형 데이터베이스 스키마:**
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
```

### 2. 보안 구현

**API 키 관리:**
```javascript
const crypto = require('crypto');

function generateApiKey() {
  return 'pk_live_' + crypto.randomBytes(24).toString('hex');
}

function hashApiKey(apiKey) {
  return crypto.createHash('sha256').update(apiKey).digest('hex');
}
```

**JWT 토큰 검증:**
```javascript
const jwt = require('jsonwebtoken');

function validateToken(req, res, next) {
  const authHeader = req.headers.authorization;
  if (!authHeader || !authHeader.startsWith('Bearer ')) {
    return res.status(401).json({ error: '인증 토큰 누락' });
  }
  
  const token = authHeader.substring(7);
  try {
    const decoded = jwt.verify(token, process.env.JWT_SECRET);
    req.user = decoded;
    next();
  } catch (err) {
    return res.status(401).json({ error: '유효하지 않은 토큰' });
  }
}
```

### 3. 유효성 검사

**JSON 스키마 검증 (Node.js):**
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

### 4. 에너지 계산

**RER 및 DER 공식:**
```python
def calculate_rer(weight_kg):
    """휴식 에너지 요구량 계산"""
    return 70 * (weight_kg ** 0.75)

def calculate_der(weight_kg, life_stage, activity, neutered):
    """일일 에너지 요구량 계산"""
    rer = calculate_rer(weight_kg)
    
    multipliers = {
        'puppy_0_4months': 3.0,
        'puppy_4_12months': 2.0,
        'adult_sedentary_neutered': 1.2,
        'adult_moderate_neutered': 1.4,
        'adult_active_neutered': 1.6,
        'senior_neutered': 1.1,
        'weight_loss': 1.0,
    }
    
    key = f"{life_stage}_{activity}_{'neutered' if neutered else 'intact'}"
    multiplier = multipliers.get(key, 1.4)
    
    return rer * multiplier
```

### 5. 테스팅

**단위 테스트 예제:**
```javascript
const { calculateNutritionalRequirements } = require('../services/nutrition');

describe('영양 요구사항 계산', () => {
  test('중성화된 성견의 DER을 올바르게 계산', () => {
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
});
```

---

**弘益人間 · 널리 인간을 이롭게 하라**  
© 2025 WIA | WIA-PET-009 v1.0 단계 3
