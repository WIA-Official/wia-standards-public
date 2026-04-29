# WIA-TIME-029: Quick Start Guide

## 🔄 Time Adaptation Standard

**Theme:** Violet #8B5CF6  
**Version:** 1.0.0  
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Installation

```bash
cd /home/user/wia-standards/standards/WIA-TIME-029
./install.sh
```

---

## CLI Usage Examples

### 1. Create Adaptation Program (Travel to 1920s)
```bash
wia-time-029 create-program \
  --target 1920-01-01 \
  --duration 90 \
  --location "New York"
```

### 2. Assess Culture Shock Risk
```bash
wia-time-029 assess \
  --from 2025-01-01 \
  --to 1920-01-01
```

### 3. Generate Language Training Plan
```bash
wia-time-029 language-plan \
  --era 1920 \
  --native modern-english
```

### 4. Get Historical Context Briefing
```bash
wia-time-029 context-brief \
  --era 1920 \
  --location "New York"
```

### 5. Track Adaptation Progress
```bash
wia-time-029 track-progress \
  --traveler-id T-2025-001 \
  --days 45
```

### 6. Check Readiness
```bash
wia-time-029 check-readiness \
  --traveler-id T-2025-001
```

---

## TypeScript SDK Usage

### Installation
```bash
cd api/typescript
npm install
npm run build
```

### Basic Usage
```typescript
import { TimeAdaptationSDK } from '@wia/time-029';

const sdk = new TimeAdaptationSDK();

// Quick assessment
const result = await sdk.quickAssessment({
  targetYear: 1920,
  currentYear: 2025,
  travelDuration: 90
});

console.log(`Difficulty: ${result.difficulty}/10`);
console.log(`Training: ${result.trainingWeeks} weeks`);
```

### Create Full Adaptation Program
```typescript
const program = await sdk.createAdaptationProgram({
  targetEra: '1920-01-01',
  originEra: '2025-01-01',
  duration: 90,
  travelerProfile: {
    age: 32,
    educationLevel: 'advanced',
    nativeLanguages: ['English'],
    psychologicalProfile: {
      adaptability: 75,
      stressTolerance: 70,
      culturalOpenness: 85,
      socialComfort: 65,
      learningAgility: 80,
      riskTolerance: 'medium'
    }
  }
});

console.log(`Program ID: ${program.data.id}`);
console.log(`Training Modules: ${program.data.trainingModules.length}`);
console.log(`Total Hours: ${program.data.preDeparture.totalHours}`);
```

### Assess Culture Shock Risk
```typescript
const risk = sdk.assessCultureShockRisk({
  temporalDisplacement: -105 * 365 * 24 * 3600, // 105 years back
  culturalDistance: 0.75,
  technologicalGap: 0.85,
  travelerAdaptability: 75
});

console.log(`Risk Level: ${risk.riskLevel}`);
console.log(`Risk Score: ${risk.riskScore}/100`);
console.log(`Success Probability: ${(risk.successProbability * 100).toFixed(1)}%`);
console.log(`Training Required: ${risk.trainingDuration} weeks`);
```

---

## Key Concepts

### Adaptation Phases
1. **Honeymoon** (Days 1-7): Excitement and fascination
2. **Culture Shock** (Days 8-30): Confusion and anxiety
3. **Adjustment** (Days 31-90): Growing understanding
4. **Adaptation** (Days 91-180): Functional competence
5. **Mastery** (Days 181+): Cultural fluency

### Risk Levels
- **Minimal** (0-20): 4-6 weeks training, 95-100% success
- **Low** (20-40): 6-10 weeks training, 85-95% success
- **Moderate** (40-60): 10-16 weeks training, 70-85% success
- **High** (60-75): 16-24 weeks training, 55-70% success
- **Severe** (75-90): 24-36 weeks training, 40-55% success
- **Extreme** (90-100): 36+ weeks training, 20-40% success

### Competency Levels
- **0-40 (Minimal)**: Survival level
- **40-60 (Functional)**: Basic competence
- **60-75 (Proficient)**: Full integration
- **75-90 (Advanced)**: Native-level performance
- **90-100 (Expert)**: Cultural mastery

---

## Documentation

- **Specification**: `/spec/WIA-TIME-029-v1.0.md`
- **README**: `/README.md`
- **TypeScript Types**: `/api/typescript/src/types.ts`
- **SDK Implementation**: `/api/typescript/src/index.ts`

---

## Support

- **GitHub**: https://github.com/WIA-Official/wia-standards
- **Issues**: https://github.com/WIA-Official/wia-standards/issues
- **Website**: https://wiastandards.com/standards/WIA-TIME-029
- **Documentation**: https://docs.wiastandards.com

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 SmileStory Inc. / WIA*  
*MIT License*
