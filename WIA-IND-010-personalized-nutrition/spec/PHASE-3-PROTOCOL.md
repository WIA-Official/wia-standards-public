# WIA-IND-010 Phase 3: Protocol Specification
## Personalized Nutrition Standard
### 弘益人間 · Benefit All Humanity

---

## Overview

Phase 3 of WIA-IND-010 defines secure data synchronization protocols, FHIR integration for healthcare providers, and interoperability standards for personalized nutrition systems. These protocols ensure safe, compliant data exchange between nutrition platforms, electronic health records, laboratory information systems, and clinical workflows.

## 1. Data Synchronization Protocol

### 1.1 Sync Architecture

WIA-IND-010 supports both **push** and **pull** synchronization models:

**Push Model**: Client pushes data to server when changes occur
**Pull Model**: Client periodically polls server for updates
**Hybrid Model**: Webhook notifications trigger pull requests (recommended)

### 1.2 Sync Request Format

```http
POST /api/v1/sync
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "syncId": "sync-550e8400-e29b",
  "clientId": "client-123456",
  "lastSyncTimestamp": "2025-12-27T10:00:00Z",
  "dataTypes": ["meals", "biomarkers", "wearables"],
  "changes": [
    {
      "changeId": "change-001",
      "dataType": "meal",
      "operation": "insert|update|delete",
      "timestamp": "2025-12-27T12:30:00Z",
      "data": {...}
    }
  ]
}
```

### 1.3 Sync Response Format

```json
{
  "syncId": "sync-550e8400-e29b",
  "serverTimestamp": "2025-12-27T12:35:00Z",
  "status": "success",
  "processed": 15,
  "conflicts": 0,
  "serverChanges": [
    {
      "changeId": "server-change-001",
      "dataType": "recommendation",
      "operation": "update",
      "timestamp": "2025-12-27T11:00:00Z",
      "data": {...}
    }
  ],
  "nextSyncToken": "token-abc123def456"
}
```

### 1.4 Conflict Resolution

When server and client have conflicting updates:

1. **Last-Write-Wins**: Most recent timestamp prevails (default)
2. **Server-Wins**: Server version always takes precedence
3. **Client-Wins**: Client version preserved
4. **Manual Resolution**: User prompted to resolve conflict

Conflict notification:
```json
{
  "conflicts": [
    {
      "conflictId": "conflict-789",
      "dataType": "meal",
      "recordId": "meal-123456",
      "clientVersion": {...},
      "serverVersion": {...},
      "resolution": "manual_required"
    }
  ]
}
```

## 2. FHIR Integration

### 2.1 FHIR Resource Mapping

WIA-IND-010 data maps to FHIR R4 resources:

| WIA-IND-010 Data | FHIR Resource |
|------------------|---------------|
| Health Profile | Patient |
| Meal Log | NutritionOrder |
| Dietary Assessment | NutritionIntake |
| Biomarkers | Observation |
| Genetic Data | MolecularSequence |
| Nutrition Goals | Goal |
| Recommendations | CarePlan |

### 2.2 Patient Resource Example

```json
{
  "resourceType": "Patient",
  "id": "patient-123456",
  "identifier": [
    {
      "system": "http://wia.org/nutrition-id",
      "value": "550e8400-e29b-41d4-a716-446655440000"
    }
  ],
  "birthDate": "1990-05-15",
  "gender": "female",
  "extension": [
    {
      "url": "http://wia.org/fhir/StructureDefinition/nutrition-profile",
      "extension": [
        {
          "url": "dietaryPreference",
          "valueString": "vegetarian"
        },
        {
          "url": "allergies",
          "valueString": "peanuts, shellfish"
        },
        {
          "url": "healthGoal",
          "valueString": "weight_loss"
        }
      ]
    }
  ]
}
```

### 2.3 NutritionIntake Resource (Meal Log)

```json
{
  "resourceType": "NutritionIntake",
  "id": "meal-789012",
  "status": "completed",
  "subject": {
    "reference": "Patient/patient-123456"
  },
  "occurrenceDateTime": "2025-12-27T12:00:00Z",
  "consumedItem": [
    {
      "nutritionProduct": {
        "concept": {
          "text": "Grilled Salmon"
        }
      },
      "amount": {
        "value": 180,
        "unit": "g"
      },
      "nutrient": [
        {
          "type": {
            "coding": [
              {
                "system": "http://loinc.org",
                "code": "2331-8",
                "display": "Calories"
              }
            ]
          },
          "amount": {
            "value": 340,
            "unit": "kcal"
          }
        },
        {
          "type": {
            "coding": [
              {
                "system": "http://loinc.org",
                "code": "2068-6",
                "display": "Protein"
              }
            ]
          },
          "amount": {
            "value": 42,
            "unit": "g"
          }
        }
      ]
    }
  ]
}
```

### 2.4 Observation Resource (Biomarker)

```json
{
  "resourceType": "Observation",
  "id": "biomarker-345678",
  "status": "final",
  "category": [
    {
      "coding": [
        {
          "system": "http://terminology.hl7.org/CodeSystem/observation-category",
          "code": "laboratory"
        }
      ]
    }
  ],
  "code": {
    "coding": [
      {
        "system": "http://loinc.org",
        "code": "2093-3",
        "display": "Cholesterol [Mass/volume] in Serum or Plasma"
      }
    ],
    "text": "Total Cholesterol"
  },
  "subject": {
    "reference": "Patient/patient-123456"
  },
  "effectiveDateTime": "2025-12-15T08:00:00Z",
  "valueQuantity": {
    "value": 195,
    "unit": "mg/dL",
    "system": "http://unitsofmeasure.org",
    "code": "mg/dL"
  },
  "referenceRange": [
    {
      "low": {
        "value": 125,
        "unit": "mg/dL"
      },
      "high": {
        "value": 200,
        "unit": "mg/dL"
      },
      "type": {
        "text": "Normal Range"
      }
    }
  ]
}
```

### 2.5 CarePlan Resource (Nutrition Recommendations)

```json
{
  "resourceType": "CarePlan",
  "id": "nutrition-plan-567890",
  "status": "active",
  "intent": "plan",
  "category": [
    {
      "coding": [
        {
          "system": "http://snomed.info/sct",
          "code": "410177006",
          "display": "Nutrition support"
        }
      ]
    }
  ],
  "subject": {
    "reference": "Patient/patient-123456"
  },
  "period": {
    "start": "2025-12-27",
    "end": "2026-03-27"
  },
  "goal": [
    {
      "reference": "Goal/weight-loss-goal-001"
    }
  ],
  "activity": [
    {
      "detail": {
        "kind": "NutritionOrder",
        "code": {
          "text": "Personalized Meal Plan"
        },
        "status": "in-progress",
        "description": "Follow personalized meal plan based on genetic profile (MTHFR C677T, FTO AA) and microbiome analysis. Target: 1800 kcal/day, 30% protein, 40% carbs, 30% fat. Emphasize folate-rich foods and portion control."
      }
    }
  ]
}
```

## 3. Healthcare Provider Integration

### 3.1 EHR System Integration Flow

1. **Patient Authorization**: Patient authorizes nutrition data sharing with EHR
2. **OAuth Setup**: EHR system authenticates via OAuth 2.0
3. **FHIR Endpoint Discovery**: EHR discovers FHIR endpoints
4. **Data Synchronization**: Bidirectional sync of relevant resources
5. **Clinical Review**: Provider reviews nutrition data in EHR
6. **Care Coordination**: Provider updates care plans, prescribes nutrition interventions

### 3.2 HL7 FHIR Messaging

**Nutrition Data Update Notification:**
```json
{
  "resourceType": "Bundle",
  "type": "message",
  "entry": [
    {
      "resource": {
        "resourceType": "MessageHeader",
        "eventCoding": {
          "system": "http://wia.org/nutrition-events",
          "code": "nutrition-data-update"
        },
        "source": {
          "name": "WIA Nutrition Platform",
          "endpoint": "https://api.wia-nutrition.org/fhir"
        },
        "focus": [
          {
            "reference": "NutritionIntake/meal-789012"
          }
        ]
      }
    }
  ]
}
```

## 4. Laboratory Integration

### 4.1 LIS (Laboratory Information System) Protocol

WIA-IND-010 supports HL7 v2.x messaging for lab result ingestion:

**ORU^R01 Message (Lab Results):**
```
MSH|^~\&|LAB|HOSPITAL|WIA-NUTRITION|WIA|20251227120000||ORU^R01|MSG001|P|2.5.1
PID|1||550e8400^^^WIA-NUTRITION^MR||DOE^JANE||19900515|F
OBR|1|ORDER001|RESULT001|LIPID^Lipid Panel|||20251215080000
OBX|1|NM|2093-3^Total Cholesterol||195|mg/dL|125-200|N|||F
OBX|2|NM|2089-1^LDL Cholesterol||115|mg/dL|<100|H|||F
OBX|3|NM|2085-9^HDL Cholesterol||58|mg/dL|>60|L|||F
OBX|4|NM|2571-8^Triglycerides||110|mg/dL|<150|N|||F
```

### 4.2 LOINC Code Mapping

Common biomarkers mapped to LOINC codes:

| Biomarker | LOINC Code | Unit |
|-----------|------------|------|
| Total Cholesterol | 2093-3 | mg/dL |
| LDL Cholesterol | 2089-1 | mg/dL |
| HDL Cholesterol | 2085-9 | mg/dL |
| Triglycerides | 2571-8 | mg/dL |
| Glucose (Fasting) | 1558-6 | mg/dL |
| HbA1c | 4548-4 | % |
| Vitamin D | 1989-3 | ng/mL |
| Vitamin B12 | 2132-9 | pg/mL |
| Ferritin | 2276-4 | ng/mL |

## 5. Security and Compliance

### 5.1 Encryption Standards

- **Data at Rest**: AES-256-GCM encryption
- **Data in Transit**: TLS 1.3 with perfect forward secrecy
- **End-to-End Encryption**: Optional for highly sensitive genetic data
- **Key Management**: HSM-based or cloud KMS (AWS KMS, Azure Key Vault)

### 5.2 HIPAA Compliance

WIA-IND-010 implementations handling PHI must comply with:

- **Privacy Rule**: Minimum necessary disclosure, patient consent
- **Security Rule**: Administrative, physical, technical safeguards
- **Breach Notification Rule**: Timely notification of data breaches
- **Business Associate Agreements**: Required for third-party services

Required safeguards:
- Unique user identification
- Emergency access procedures
- Automatic logoff
- Encryption and decryption
- Audit controls and logging
- Integrity controls
- Person or entity authentication
- Transmission security

### 5.3 GDPR Compliance

For European users:

- **Lawful Basis**: Explicit consent for genetic/health data processing
- **Data Minimization**: Collect only necessary data
- **Right to Access**: Users can download all their data
- **Right to Erasure**: Complete data deletion upon request
- **Right to Portability**: Data export in machine-readable format
- **Data Protection Impact Assessment**: Required for genetic data processing

### 5.4 Audit Logging

All data access must be logged:

```json
{
  "logId": "audit-log-123456",
  "timestamp": "2025-12-27T12:45:00Z",
  "userId": "provider-789012",
  "action": "read",
  "resource": "Patient/patient-123456/genetic-profile",
  "ipAddress": "192.168.1.100",
  "userAgent": "EHR-System/5.2",
  "success": true,
  "accessReason": "clinical_review"
}
```

Audit logs must be:
- Retained for minimum 7 years
- Immutable (append-only)
- Regularly reviewed
- Available for compliance audits

## 6. Data Validation and Quality

### 6.1 Validation Rules

Before synchronization, data must be validated:

1. **Schema Validation**: JSON schema compliance
2. **Range Validation**: Physiologically plausible values
3. **Referential Integrity**: Valid foreign key references
4. **Temporal Consistency**: Logical timestamp ordering
5. **Completeness**: Required fields present

### 6.2 Quality Metrics

```json
{
  "dataQualityReport": {
    "userId": "550e8400-e29b-41d4-a716-446655440000",
    "reportDate": "2025-12-27",
    "completeness": {
      "healthProfile": 95,
      "mealLogs_7days": 85,
      "biomarkers_90days": 100,
      "geneticData": 100
    },
    "accuracy": {
      "selfReportedData": 78,
      "deviceData": 98,
      "laboratoryData": 100
    },
    "timeliness": {
      "realtimeData_24h": 92,
      "weeklyData": 88
    },
    "overallScore": 91
  }
}
```

## 7. Disaster Recovery and Business Continuity

### 7.1 Backup Strategy

- **Frequency**: Continuous replication + daily snapshots
- **Retention**: 30 days rolling, 7 yearly snapshots
- **Geographic Redundancy**: Multi-region replication
- **Backup Encryption**: AES-256 encrypted backups
- **Recovery Time Objective (RTO)**: < 1 hour
- **Recovery Point Objective (RPO)**: < 15 minutes

### 7.2 Failover Procedures

In case of primary system failure:

1. Automated health checks detect outage
2. DNS failover to secondary region (< 5 minutes)
3. Database promotion to active status
4. Application services restart in secondary region
5. Validation of data integrity post-failover
6. Notification to users of temporary service impact

## 8. API Versioning and Deprecation

### 8.1 Version Lifecycle

- **Alpha**: Internal testing only
- **Beta**: Limited external testing, may change
- **Stable**: Production-ready, backward compatible
- **Deprecated**: Supported for 12 months, migration encouraged
- **Sunset**: No longer supported

### 8.2 Deprecation Notices

Deprecated endpoints return warning header:
```
Deprecation: true
Sunset: Wed, 31 Dec 2026 23:59:59 GMT
Link: <https://api.wia.org/v2/migration-guide>; rel="alternate"
```

## Conclusion

WIA-IND-010 Phase 3 protocols ensure secure, compliant, and interoperable data synchronization between personalized nutrition systems and healthcare infrastructure. Through FHIR integration, robust security measures, comprehensive audit logging, and adherence to HIPAA/GDPR requirements, these protocols enable safe clinical integration while embodying the principle of 弘益人間 (Benefit All Humanity).

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
WIA-IND-010 v1.0
