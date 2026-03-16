# WIA-ALZHEIMERS - Phase 2: API Interface

> **Version:** 1.0.0
> **Last Updated:** 2025-12-29
> **Status:** Complete
> **Standard ID:** WIA-MED-ALZHEIMERS

---

## 1. Overview

Phase 2 defines the API interfaces for the WIA-ALZHEIMERS standard, enabling seamless integration between healthcare systems, research platforms, and clinical applications. The API supports both RESTful and GraphQL paradigms to accommodate diverse integration requirements.

### 1.1 Design Principles

- **Interoperability**: Compatible with FHIR R4/R5, HL7v2, and CDA standards
- **Security**: OAuth 2.0/OIDC authentication, HIPAA-compliant data handling
- **Scalability**: Designed for high-throughput clinical and research environments
- **Accessibility**: Follows WIA principles of universal access

### 1.2 Base URLs

```
Production: https://api.wia.live/v1/alzheimers
Staging:    https://staging-api.wia.live/v1/alzheimers
Sandbox:    https://sandbox-api.wia.live/v1/alzheimers
```

---

## 2. RESTful API Endpoints

### 2.1 Assessment Endpoints

#### 2.1.1 Create Comprehensive Assessment

```http
POST /api/v1/alzheimers/assess
Content-Type: application/json
Authorization: Bearer {token}
```

**Request Body:**

```json
{
  "subject_id": "550e8400-e29b-41d4-a716-446655440000",
  "assessment_type": "comprehensive",
  "nad_metabolism": {
    "nad_plus": { "value": 22.5, "unit": "μM", "tissue": "blood" },
    "nadh": { "value": 4.5, "unit": "μM" }
  },
  "cognitive_scores": {
    "mmse": 24,
    "moca": 22
  },
  "biomarkers": {
    "abeta42_40_ratio": 0.062,
    "plasma_ptau181": 28.5
  }
}
```

**Response (201 Created):**

```json
{
  "assessment_id": "asmt_123456789",
  "subject_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-29T10:30:00Z",
  "nad_homeostasis_index": {
    "score": 0.68,
    "interpretation": "moderate",
    "percentile": 45
  },
  "cognitive_stage": "mci",
  "risk_assessment": {
    "progression_risk": "moderate",
    "recommended_follow_up_days": 90
  },
  "recommendations": [
    {
      "type": "intervention",
      "priority": "high",
      "description": "Consider NAD+ precursor supplementation (NR 500mg/day)"
    },
    {
      "type": "monitoring",
      "priority": "medium",
      "description": "Repeat NAD+ assessment in 8 weeks"
    }
  ]
}
```

#### 2.1.2 Get NAD+ Homeostasis Index

```http
GET /api/v1/alzheimers/nad-homeostasis/{subject_id}
Authorization: Bearer {token}
```

**Query Parameters:**

| Parameter | Type | Description | Required |
|-----------|------|-------------|----------|
| `timeframe` | string | Data timeframe (1week, 1month, 3months, 6months, 1year) | No |
| `include_history` | boolean | Include historical data | No |

**Response (200 OK):**

```json
{
  "subject_id": "550e8400-e29b-41d4-a716-446655440000",
  "current": {
    "timestamp": "2025-12-29T10:30:00Z",
    "homeostasis_index": 0.72,
    "interpretation": "good",
    "nad_plus": { "value": 28.5, "unit": "μM" },
    "nadh_nad_ratio": 0.18
  },
  "history": [
    {
      "timestamp": "2025-11-29T10:30:00Z",
      "homeostasis_index": 0.62,
      "interpretation": "moderate"
    },
    {
      "timestamp": "2025-10-29T10:30:00Z",
      "homeostasis_index": 0.52,
      "interpretation": "poor"
    }
  ],
  "trend": "improving"
}
```

#### 2.1.3 Get Biomarkers

```http
GET /api/v1/alzheimers/biomarkers/{subject_id}
Authorization: Bearer {token}
```

**Query Parameters:**

| Parameter | Type | Description | Required |
|-----------|------|-------------|----------|
| `marker_type` | string | amyloid, tau, neurodegeneration, inflammation, all | No |
| `format` | string | json, fhir | No |

**Response (200 OK):**

```json
{
  "subject_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-29T10:30:00Z",
  "amyloid": {
    "abeta42_40_ratio": { "value": 0.062, "status": "positive" },
    "plasma_abeta42": { "value": 42.5, "unit": "pg/mL" }
  },
  "tau": {
    "plasma_ptau181": { "value": 28.5, "unit": "pg/mL", "status": "elevated" },
    "plasma_ptau217": { "value": 0.52, "unit": "pg/mL", "status": "elevated" }
  },
  "neurodegeneration": {
    "plasma_nfl": { "value": 22.5, "unit": "pg/mL", "status": "normal" },
    "plasma_gfap": { "value": 145, "unit": "pg/mL", "status": "borderline" }
  }
}
```

### 2.2 Intervention Endpoints

#### 2.2.1 Get Intervention Recommendation

```http
POST /api/v1/alzheimers/intervention/recommend
Content-Type: application/json
Authorization: Bearer {token}
```

**Request Body:**

```json
{
  "subject_id": "550e8400-e29b-41d4-a716-446655440000",
  "nad_homeostasis_index": 0.52,
  "cognitive_stage": "mci",
  "biomarkers": {
    "abeta42_40_ratio": 0.062,
    "plasma_ptau181": 28.5
  },
  "contraindications": [],
  "preferences": {
    "oral_supplements": true,
    "infusion_therapy": false
  }
}
```

**Response (200 OK):**

```json
{
  "recommendation_id": "rec_987654321",
  "subject_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-29T10:30:00Z",
  "primary_intervention": {
    "type": "nad_precursor",
    "agent": "nicotinamide_riboside",
    "dosage": {
      "value": 500,
      "unit": "mg",
      "frequency": "twice_daily"
    },
    "duration": "ongoing",
    "expected_outcomes": {
      "nad_increase_percent": 40,
      "cognitive_improvement_percent": 12,
      "time_to_response_weeks": 8
    },
    "evidence_level": "moderate",
    "references": [
      "Alzheimer's & Dementia: TRC2, 2025"
    ]
  },
  "secondary_interventions": [
    {
      "type": "lifestyle",
      "description": "Aerobic exercise 150 min/week",
      "expected_benefit": "NAD+ level increase, neuroplasticity"
    },
    {
      "type": "dietary",
      "description": "Mediterranean diet pattern",
      "expected_benefit": "Mitochondrial health support"
    }
  ],
  "monitoring_protocol": {
    "initial_assessment_weeks": 4,
    "follow_up_interval_weeks": 8,
    "required_tests": ["plasma_nad", "moca"]
  },
  "anti_amyloid_consideration": {
    "eligible": true,
    "agents": ["lecanemab", "donanemab"],
    "notes": "Consider combination therapy based on amyloid positivity"
  }
}
```

#### 2.2.2 Track Treatment Response

```http
GET /api/v1/alzheimers/track/{subject_id}
Authorization: Bearer {token}
```

**Query Parameters:**

| Parameter | Type | Description | Required |
|-----------|------|-------------|----------|
| `timeframe` | string | 1week, 1month, 3months, 6months, 1year | No |
| `intervention_id` | string | Specific intervention to track | No |

**Response (200 OK):**

```json
{
  "subject_id": "550e8400-e29b-41d4-a716-446655440000",
  "tracking_period": {
    "start": "2025-11-01T00:00:00Z",
    "end": "2025-12-29T10:30:00Z"
  },
  "intervention": {
    "type": "nad_precursor",
    "agent": "NR",
    "dosage": "500mg twice daily",
    "started_at": "2025-11-01T08:00:00Z"
  },
  "progression": [
    {
      "week": 0,
      "nad_homeostasis_index": 0.52,
      "moca_score": 22,
      "notes": "Baseline"
    },
    {
      "week": 4,
      "nad_homeostasis_index": 0.62,
      "moca_score": 23,
      "notes": "Initial response observed"
    },
    {
      "week": 8,
      "nad_homeostasis_index": 0.72,
      "moca_score": 24,
      "notes": "Significant improvement"
    }
  ],
  "outcomes": {
    "responder_status": "full",
    "nad_improvement_percent": 38.5,
    "cognitive_improvement_points": 2,
    "adverse_events": []
  }
}
```

### 2.3 Registry Endpoints

#### 2.3.1 Register Subject

```http
POST /api/v1/alzheimers/registry/subjects
Content-Type: application/json
Authorization: Bearer {token}
```

**Request Body:**

```json
{
  "demographics": {
    "age": 68,
    "sex": "female",
    "ethnicity": "caucasian"
  },
  "genetic": {
    "apoe_genotype": "e3/e4"
  },
  "clinical_site": "site_001",
  "consent_status": "full",
  "consent_date": "2025-12-01T10:00:00Z"
}
```

#### 2.3.2 Search Registry

```http
GET /api/v1/alzheimers/registry/search
Authorization: Bearer {token}
```

**Query Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `stage` | string | preclinical, mci, mild, moderate, severe |
| `nad_index_min` | number | Minimum NAD+ homeostasis index |
| `nad_index_max` | number | Maximum NAD+ homeostasis index |
| `age_min` | integer | Minimum age |
| `age_max` | integer | Maximum age |
| `intervention` | string | Active intervention type |

---

## 3. GraphQL API

### 3.1 Schema Definition

```graphql
type Query {
  # Subject queries
  subject(id: ID!): Subject
  subjects(filter: SubjectFilter, limit: Int, offset: Int): SubjectConnection

  # Assessment queries
  nadHomeostasis(subjectId: ID!): NADHomeostasisIndex
  pathologyProfile(subjectId: ID!): PathologyProfile
  treatmentResponse(subjectId: ID!, interventionId: ID): TreatmentResponse

  # Analytics queries
  populationStats(filter: StatsFilter): PopulationStatistics
  interventionEfficacy(agent: String!): InterventionEfficacy
}

type Mutation {
  # Assessment mutations
  createAssessment(input: AssessmentInput!): Assessment!
  updateAssessment(id: ID!, input: AssessmentInput!): Assessment!

  # Intervention mutations
  startIntervention(input: InterventionInput!): Intervention!
  recordFollowUp(input: FollowUpInput!): FollowUpRecord!

  # Registry mutations
  registerSubject(input: SubjectInput!): Subject!
  updateConsent(subjectId: ID!, consent: ConsentInput!): Subject!
}

type Subscription {
  # Real-time updates
  assessmentCreated(subjectId: ID): Assessment
  interventionUpdated(subjectId: ID): Intervention
  biomarkerAlert(subjectId: ID, threshold: Float): BiomarkerAlert
}
```

### 3.2 Type Definitions

```graphql
type Subject {
  id: ID!
  demographics: Demographics!
  genetic: GeneticProfile
  assessments: [Assessment!]!
  interventions: [Intervention!]!
  latestNADHomeostasis: NADHomeostasisIndex
  latestCognitiveStage: CognitiveStage
}

type NADHomeostasisIndex {
  id: ID!
  subjectId: ID!
  timestamp: DateTime!
  nadPlus: Float!
  nadh: Float!
  nadhNadRatio: Float!
  synthesizingEnzymes: EnzymeProfile
  consumingEnzymes: EnzymeProfile
  mitochondrialFunction: MitochondrialMetrics
  homeostasisScore: Float!
  percentile: Int!
  interpretation: HomeostasisInterpretation!
}

type PathologyProfile {
  id: ID!
  subjectId: ID!
  timestamp: DateTime!
  cognitiveAssessment: CognitiveAssessment!
  amyloidMarkers: AmyloidMarkers
  tauMarkers: TauMarkers
  neurodegenerationMarkers: NeurodegenerationMarkers
  neuroinflammation: NeuroinflammationMarkers
  bloodBrainBarrier: BBBStatus
  synapticFunction: SynapticMetrics
}

type Intervention {
  id: ID!
  subjectId: ID!
  type: InterventionType!
  agent: String!
  dosage: Dosage!
  startedAt: DateTime!
  endedAt: DateTime
  status: InterventionStatus!
  followUps: [FollowUpRecord!]!
  outcomes: TreatmentOutcomes
}

enum HomeostasisInterpretation {
  EXCELLENT
  GOOD
  MODERATE
  POOR
  CRITICAL
}

enum CognitiveStage {
  PRECLINICAL
  MCI
  MILD
  MODERATE
  SEVERE
}

enum InterventionType {
  NAD_PRECURSOR
  ANTI_AMYLOID
  LIFESTYLE
  COMBINATION
}

input AssessmentInput {
  subjectId: ID!
  assessmentType: String!
  nadMetabolism: NADMetabolismInput
  cognitiveScores: CognitiveScoresInput
  biomarkers: BiomarkersInput
}
```

### 3.3 Example Queries

#### Get Complete Subject Profile

```graphql
query GetSubjectProfile($subjectId: ID!) {
  subject(id: $subjectId) {
    id
    demographics {
      age
      sex
    }
    genetic {
      apoeGenotype
    }
    latestNADHomeostasis {
      homeostasisScore
      interpretation
      nadPlus
      nadhNadRatio
      timestamp
    }
    latestCognitiveStage
    assessments(limit: 5) {
      id
      timestamp
      type
    }
    interventions(status: ACTIVE) {
      id
      agent
      dosage {
        value
        unit
        frequency
      }
      startedAt
    }
  }
}
```

#### Create Assessment

```graphql
mutation CreateAssessment($input: AssessmentInput!) {
  createAssessment(input: $input) {
    id
    nadHomeostasis {
      homeostasisScore
      interpretation
    }
    recommendations {
      type
      priority
      description
    }
  }
}
```

---

## 4. Authentication

### 4.1 OAuth 2.0 / OIDC Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={client_id}
&client_secret={client_secret}
&scope=alzheimers:read alzheimers:write
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "alzheimers:read alzheimers:write"
}
```

### 4.2 Scopes

| Scope | Description |
|-------|-------------|
| `alzheimers:read` | Read access to assessments and biomarkers |
| `alzheimers:write` | Create/update assessments |
| `alzheimers:admin` | Full administrative access |
| `alzheimers:research` | Access to anonymized research data |

---

## 5. Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request body |
| `VALIDATION_ERROR` | 400 | Data validation failed |
| `UNAUTHORIZED` | 401 | Invalid or expired token |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `CONFLICT` | 409 | Resource already exists |
| `RATE_LIMITED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |

**Error Response Format:**

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "NAD+ value must be positive",
    "field": "nad_metabolism.nad_plus.value",
    "details": {
      "received": -5,
      "expected": "> 0"
    }
  }
}
```

---

## 6. Rate Limiting

| Tier | Requests/min | Requests/day |
|------|--------------|--------------|
| Basic | 60 | 10,000 |
| Professional | 300 | 100,000 |
| Enterprise | 1,000 | Unlimited |
| Research | 500 | 500,000 |

**Rate Limit Headers:**

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1735470600
```

---

## 7. Webhooks

### 7.1 Available Events

| Event | Description |
|-------|-------------|
| `assessment.created` | New assessment created |
| `assessment.updated` | Assessment data updated |
| `intervention.started` | New intervention started |
| `intervention.response` | Treatment response recorded |
| `biomarker.alert` | Biomarker threshold exceeded |
| `nad.critical` | NAD+ homeostasis index critical |

### 7.2 Webhook Payload

```json
{
  "event": "biomarker.alert",
  "timestamp": "2025-12-29T10:30:00Z",
  "data": {
    "subject_id": "550e8400-e29b-41d4-a716-446655440000",
    "biomarker": "plasma_ptau181",
    "value": 45.2,
    "threshold": 35,
    "severity": "high"
  },
  "signature": "sha256=..."
}
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
