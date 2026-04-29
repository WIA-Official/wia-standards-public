# WIA-MENTAL_WELLNESS: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the data format requirements for MENTAL WELLNESS. All implementations MUST follow these specifications to ensure interoperability across mental health platforms, therapy applications, and wellness tracking systems.

## 2. Data Structures

### 2.1 Mental Wellness Record Format

```json
{
  "type": "WIA-MENTAL_WELLNESSRecord",
  "version": "1.0",
  "id": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "userId": "string (anonymized)",
  "data": {
    "recordType": "mood|assessment|session|intervention",
    "content": {},
    "metadata": {}
  },
  "privacy": {
    "encryptionLevel": "high|medium|standard",
    "consentId": "string (UUID)",
    "dataRetention": "ISO 8601 duration"
  },
  "signature": "string (Ed25519)"
}
```

### 2.2 Mood Record Format

```json
{
  "type": "MoodRecord",
  "moodId": "UUID v4",
  "timestamp": "ISO 8601",
  "userId": "string (anonymized)",
  "mood": {
    "primary": "happy|sad|anxious|calm|angry|neutral|mixed",
    "intensity": 1-10,
    "valence": -10 to 10,
    "arousal": 0-10,
    "dominance": 0-10,
    "tags": ["string"],
    "triggers": ["string"],
    "coping": ["string"]
  },
  "context": {
    "location": "home|work|transit|social|other",
    "activity": "string",
    "social": "alone|family|friends|colleagues|strangers",
    "weather": "string (optional)",
    "sleep": {
      "hours": 0-24,
      "quality": 1-5
    }
  },
  "physiological": {
    "heartRate": "integer (bpm)",
    "hrv": "integer (ms)",
    "respirationRate": "integer",
    "skinConductance": "float",
    "cortisol": "float (optional)"
  }
}
```

### 2.3 Assessment Record Format

```json
{
  "type": "AssessmentRecord",
  "assessmentId": "UUID v4",
  "timestamp": "ISO 8601",
  "userId": "string (anonymized)",
  "assessment": {
    "name": "PHQ-9|GAD-7|PSS|BDI|DASS-21|WHO-5|PCL-5",
    "version": "string",
    "questions": [
      {
        "id": "string",
        "text": "string",
        "response": "integer|string",
        "scale": "0-3|0-4|1-5",
        "timeTaken": "integer (seconds)"
      }
    ],
    "score": {
      "total": "integer",
      "subscales": {},
      "percentile": 0-100,
      "interpretation": "minimal|mild|moderate|severe|extreme",
      "riskLevel": "low|medium|high|critical"
    }
  },
  "clinician": {
    "reviewed": "boolean",
    "reviewerId": "string (optional)",
    "notes": "string (encrypted)",
    "followUp": "boolean"
  }
}
```

### 2.4 Therapy Session Record Format

```json
{
  "type": "TherapySessionRecord",
  "sessionId": "UUID v4",
  "timestamp": "ISO 8601",
  "userId": "string (anonymized)",
  "therapistId": "string (anonymized)",
  "session": {
    "modality": "CBT|DBT|ACT|psychodynamic|humanistic|integrative",
    "type": "individual|group|couple|family",
    "format": "in-person|video|audio|text",
    "duration": "integer (minutes)",
    "sessionNumber": "integer",
    "phase": "assessment|active|maintenance|termination"
  },
  "content": {
    "topics": ["string"],
    "techniques": ["string"],
    "homework": ["string"],
    "goals": [
      {
        "id": "string",
        "description": "string",
        "progress": 0-100,
        "status": "not-started|in-progress|achieved"
      }
    ]
  },
  "outcomes": {
    "clientSatisfaction": 1-5,
    "therapeuticAlliance": 1-7,
    "symptomChange": -10 to 10,
    "functionalImprovement": 1-5
  },
  "notes": {
    "subjective": "string (encrypted)",
    "objective": "string (encrypted)",
    "assessment": "string (encrypted)",
    "plan": "string (encrypted)"
  }
}
```

### 2.5 Crisis Intervention Record Format

```json
{
  "type": "CrisisInterventionRecord",
  "crisisId": "UUID v4",
  "timestamp": "ISO 8601",
  "userId": "string (anonymized)",
  "crisis": {
    "severity": "low|medium|high|imminent",
    "type": "suicidal|self-harm|panic|psychosis|substance|trauma",
    "triggers": ["string"],
    "symptoms": ["string"],
    "riskFactors": ["string"],
    "protectiveFactors": ["string"]
  },
  "assessment": {
    "suicidalIdeation": "none|passive|active|intent|plan|means",
    "selfHarmRisk": "low|medium|high|imminent",
    "homicidalIdeation": "none|present",
    "impulsivity": 1-10,
    "substanceUse": "boolean",
    "support": "strong|moderate|weak|none"
  },
  "intervention": {
    "responder": "ai|counselor|crisis-team|emergency-services",
    "actions": ["string"],
    "safetyPlan": {
      "warning_signs": ["string"],
      "coping_strategies": ["string"],
      "social_contacts": ["string (anonymized)"],
      "professional_contacts": ["string"],
      "environment_safety": ["string"]
    },
    "disposition": "self-care|outpatient|intensive-outpatient|hospitalization"
  },
  "followUp": {
    "scheduled": "ISO 8601",
    "contacts": ["string"],
    "monitoring": "string"
  }
}
```

### 2.6 Meditation/Mindfulness Session Format

```json
{
  "type": "MindfulnessSessionRecord",
  "sessionId": "UUID v4",
  "timestamp": "ISO 8601",
  "userId": "string (anonymized)",
  "session": {
    "type": "meditation|breathing|body-scan|yoga|tai-chi|walking",
    "technique": "mindfulness|transcendental|vipassana|loving-kindness|zen",
    "duration": "integer (seconds)",
    "guided": "boolean",
    "instructor": "string (optional)"
  },
  "experience": {
    "focusQuality": 1-10,
    "distractions": 0-10,
    "emotionalState": {
      "before": "string",
      "after": "string"
    },
    "physicalSensations": ["string"],
    "insights": ["string"]
  },
  "biometrics": {
    "heartRateAverage": "integer",
    "hrvAverage": "integer",
    "respirationRate": "integer",
    "temperature": "float"
  }
}
```

## 3. Field Definitions

### 3.1 Core Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| type | string | Yes | Record type identifier |
| version | string | Yes | Specification version |
| id | UUID | Yes | Unique record identifier |
| timestamp | ISO 8601 | Yes | Record creation time |
| userId | string | Yes | Anonymized user identifier |
| data | object | Yes | Primary data payload |
| privacy | object | Yes | Privacy and consent settings |
| signature | string | No | Cryptographic signature |

### 3.2 Privacy Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| encryptionLevel | enum | Yes | high, medium, standard |
| consentId | UUID | Yes | Reference to consent record |
| dataRetention | ISO 8601 | Yes | Retention period duration |
| deIdentified | boolean | No | Whether data is de-identified |
| hipaaCompliant | boolean | No | HIPAA compliance flag |

## 4. Data Types

### 4.1 Core Types

| Type | Format | Example |
|------|--------|---------|
| String | UTF-8 | "Cognitive behavioral therapy" |
| Number | IEEE 754 | 7.5 |
| Integer | Whole number | 42 |
| Boolean | true/false | true |
| Date | ISO 8601 | "2026-01-12T10:30:00Z" |
| UUID | RFC 4122 | "550e8400-e29b-41d4-a716-446655440000" |
| Duration | ISO 8601 | "P1Y2M10D" or "PT45M" |

### 4.2 Mental Health Specific Types

```typescript
type MoodPrimary =
  | "happy" | "sad" | "anxious" | "calm"
  | "angry" | "neutral" | "mixed" | "depressed"
  | "excited" | "fearful" | "frustrated" | "content";

type AssessmentType =
  | "PHQ-9"      // Depression (Patient Health Questionnaire)
  | "GAD-7"      // Anxiety (Generalized Anxiety Disorder)
  | "PSS"        // Perceived Stress Scale
  | "BDI"        // Beck Depression Inventory
  | "DASS-21"    // Depression Anxiety Stress Scales
  | "WHO-5"      // Well-being Index
  | "PCL-5"      // PTSD Checklist
  | "MDQ"        // Mood Disorder Questionnaire
  | "AUDIT"      // Alcohol Use Disorders Identification Test
  | "ISI";       // Insomnia Severity Index

type TherapyModality =
  | "CBT"              // Cognitive Behavioral Therapy
  | "DBT"              // Dialectical Behavior Therapy
  | "ACT"              // Acceptance and Commitment Therapy
  | "EMDR"             // Eye Movement Desensitization
  | "psychodynamic"
  | "humanistic"
  | "integrative"
  | "mindfulness-based";

type CrisisSeverity = "low" | "medium" | "high" | "imminent";
type RiskLevel = "low" | "medium" | "high" | "critical";
```

## 5. Encoding Requirements

### 5.1 Character Encoding
- All text MUST be UTF-8 encoded
- No BOM (Byte Order Mark) allowed
- Line endings: LF (Unix style)
- Support for international characters and emoji

### 5.2 Binary Encoding
- Use Base64 for binary data in JSON
- Encrypted fields use AES-256-GCM
- End-to-end encryption for sensitive clinical notes
- Zero-knowledge architecture for maximum privacy

### 5.3 Encryption Standards
- Data at rest: AES-256-GCM
- Data in transit: TLS 1.3
- Key exchange: X25519
- Signatures: Ed25519
- Password hashing: Argon2id

## 6. Validation Rules

### 6.1 Schema Validation

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["type", "version", "id", "timestamp", "userId", "data", "privacy"],
  "properties": {
    "type": {
      "type": "string",
      "const": "WIA-MENTAL_WELLNESSRecord"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+$"
    },
    "id": {
      "type": "string",
      "format": "uuid"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "userId": {
      "type": "string",
      "minLength": 1
    },
    "privacy": {
      "type": "object",
      "required": ["encryptionLevel", "consentId", "dataRetention"],
      "properties": {
        "encryptionLevel": {
          "type": "string",
          "enum": ["high", "medium", "standard"]
        },
        "consentId": {
          "type": "string",
          "format": "uuid"
        },
        "dataRetention": {
          "type": "string",
          "pattern": "^P(?:\\d+Y)?(?:\\d+M)?(?:\\d+D)?$"
        }
      }
    }
  }
}
```

### 6.2 Validation Errors

| Code | Message | Resolution |
|------|---------|------------|
| E001 | Invalid record type | Use WIA-MENTAL_WELLNESSRecord |
| E002 | Missing required field | Add all required fields |
| E003 | Invalid mood value | Use valid mood enumeration |
| E004 | Invalid intensity range | Use 1-10 scale |
| E005 | Missing consent | Obtain user consent first |
| E006 | Invalid assessment type | Use standardized assessment names |
| E007 | Encryption level mismatch | Use appropriate encryption |
| E008 | Invalid crisis severity | Use low/medium/high/imminent |

## 7. Privacy and Compliance

### 7.1 HIPAA Compliance
- All PHI (Protected Health Information) encrypted
- Access logs maintained for 7 years
- Breach notification within 60 days
- Business Associate Agreements required

### 7.2 GDPR Compliance
- Right to access: Full export available
- Right to erasure: Complete deletion supported
- Right to portability: Standard JSON format
- Consent management: Granular permissions

### 7.3 Data Minimization
- Collect only necessary data
- Anonymize when possible
- De-identify for research
- Aggregate for analytics

## 8. Versioning

### 8.1 Version Format
- Major.Minor (e.g., 1.0, 2.1)
- Major version: Breaking changes
- Minor version: Backward-compatible additions

### 8.2 Migration Path
- v1.0 → v1.1: Add optional biometric fields
- v1.x → v2.0: Enhanced crisis protocol integration

## 9. Examples

### 9.1 Complete Mood Record
```json
{
  "type": "WIA-MENTAL_WELLNESSRecord",
  "version": "1.0",
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2026-01-12T10:30:00Z",
  "userId": "anon_8f4a9c2b",
  "data": {
    "recordType": "mood",
    "content": {
      "primary": "anxious",
      "intensity": 7,
      "valence": -6,
      "arousal": 8,
      "dominance": 3,
      "tags": ["work", "deadline", "meeting"],
      "triggers": ["upcoming presentation"],
      "coping": ["deep breathing", "took a walk"]
    },
    "metadata": {
      "source": "mobile_app",
      "deviceType": "smartphone"
    }
  },
  "privacy": {
    "encryptionLevel": "high",
    "consentId": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
    "dataRetention": "P2Y"
  },
  "signature": "3045022100f1d5c..."
}
```

### 9.2 Crisis Intervention Record
```json
{
  "type": "WIA-MENTAL_WELLNESSRecord",
  "version": "1.0",
  "id": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
  "timestamp": "2026-01-12T02:15:00Z",
  "userId": "anon_crisis_1234",
  "data": {
    "recordType": "intervention",
    "content": {
      "severity": "high",
      "type": "suicidal",
      "assessment": {
        "suicidalIdeation": "active",
        "selfHarmRisk": "high",
        "impulsivity": 8,
        "support": "weak"
      },
      "intervention": {
        "responder": "crisis-team",
        "actions": [
          "safety_assessment",
          "crisis_counseling",
          "safety_plan_created",
          "emergency_contact_notified"
        ],
        "disposition": "intensive-outpatient"
      }
    },
    "metadata": {
      "urgent": true,
      "responseTime": 90
    }
  },
  "privacy": {
    "encryptionLevel": "high",
    "consentId": "crisis-consent-emergency",
    "dataRetention": "P10Y",
    "hipaaCompliant": true
  },
  "signature": "crisis_verified_2026"
}
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
