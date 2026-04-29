# WIA-CHRONIC-PAIN Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2026-01-04
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-CHRONIC-PAIN API provides RESTful endpoints for chronic pain assessment, central sensitization evaluation, and neuroplasticity-based treatment planning.

### 1.1 Base URL

```
Production: https://api.wia.live/chronic-pain/v1
Staging:    https://api-staging.wia.live/chronic-pain/v1
```

### 1.2 Authentication

```http
Authorization: Bearer <api_key>
X-WIA-Client-ID: <client_id>
```

---

## 2. OpenAPI Specification

```yaml
openapi: 3.1.0
info:
  title: WIA-CHRONIC-PAIN API
  version: 1.0.0
  description: |
    API for chronic pain assessment with Neuroplasticity Reversal focus.
    弘益人間 (홍익인간) - Benefit All Humanity
  contact:
    name: WIA Standards Team
    url: https://wia.live/chronic-pain
  license:
    name: MIT
    url: https://opensource.org/licenses/MIT

servers:
  - url: https://api.wia.live/chronic-pain/v1
    description: Production
  - url: https://api-staging.wia.live/chronic-pain/v1
    description: Staging

security:
  - BearerAuth: []
  - ApiKeyAuth: []

paths:
  /assess:
    post:
      operationId: createAssessment
      summary: Create comprehensive chronic pain assessment
      description: |
        Performs full chronic pain profile assessment including:
        - Multidimensional pain evaluation
        - Central sensitization testing
        - Psychosocial screening
        - Treatment recommendations
      tags:
        - Assessment
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/AssessmentRequest'
      responses:
        '201':
          description: Assessment created successfully
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/AssessmentResponse'
        '400':
          $ref: '#/components/responses/BadRequest'
        '401':
          $ref: '#/components/responses/Unauthorized'

  /profiles/{patient_id}:
    get:
      operationId: getPatientProfile
      summary: Get patient chronic pain profile
      tags:
        - Profiles
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
        - name: include
          in: query
          description: Include related data
          schema:
            type: array
            items:
              type: string
              enum: [sensitization, neuroimaging, psychosocial, treatments]
      responses:
        '200':
          description: Patient profile retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ChronicPainProfile'

  /sensitization/{patient_id}:
    get:
      operationId: getSensitizationStatus
      summary: Get central sensitization status
      description: |
        Retrieves detailed central sensitization assessment including:
        - CSI score and interpretation
        - QST profile
        - Temporal summation status
        - Conditioned pain modulation
        - Neuroplasticity index
      tags:
        - Central Sensitization
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
      responses:
        '200':
          description: Sensitization status retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/SensitizationResponse'

    post:
      operationId: recordSensitization
      summary: Record new sensitization assessment
      tags:
        - Central Sensitization
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SensitizationRecord'
      responses:
        '201':
          description: Sensitization recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/SensitizationResponse'

  /neuromodulation/plan:
    post:
      operationId: createNeuromodulationPlan
      summary: Generate neuromodulation treatment plan
      description: |
        Creates personalized non-invasive neuromodulation plan based on:
        - Pain phenotype
        - Central sensitization status
        - Neuroimaging findings
        - Treatment history

        Options may include:
        - rTMS (repetitive Transcranial Magnetic Stimulation)
        - tDCS (transcranial Direct Current Stimulation)
        - Focused Ultrasound
        - TENS parameters
      tags:
        - Neuromodulation
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/NeuromodulationPlanRequest'
      responses:
        '200':
          description: Neuromodulation plan generated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/NeuromodulationPlan'

  /treatment/recommend:
    post:
      operationId: getRecommendations
      summary: Get personalized treatment recommendations
      description: |
        Generates evidence-based multimodal treatment recommendations:
        - Pharmacological (opioid-sparing)
        - Neuromodulation
        - Behavioral (CBT, ACT, mindfulness)
        - Physical (exercise, PT)
        - Interventional
      tags:
        - Treatment
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TreatmentRecommendationRequest'
      responses:
        '200':
          description: Recommendations generated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TreatmentRecommendations'

  /pain-scores/{patient_id}:
    get:
      operationId: getPainScores
      summary: Get pain assessment scores over time
      tags:
        - Pain Assessment
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
        - name: from_date
          in: query
          schema:
            type: string
            format: date
        - name: to_date
          in: query
          schema:
            type: string
            format: date
      responses:
        '200':
          description: Pain scores retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/PainScoresResponse'

    post:
      operationId: recordPainScore
      summary: Record new pain score
      tags:
        - Pain Assessment
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/PainScoreRecord'
      responses:
        '201':
          description: Pain score recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/PainScoreResponse'

  /neuroplasticity/index/{patient_id}:
    get:
      operationId: getNeuroplasticityIndex
      summary: Get neuroplasticity reversal index
      description: |
        Calculates composite neuroplasticity index showing:
        - Current maladaptive plasticity state
        - Reversal potential
        - Progress over time
      tags:
        - Neuroplasticity
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
      responses:
        '200':
          description: Neuroplasticity index retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/NeuroplasticityIndex'

  /opioid-risk/{patient_id}:
    get:
      operationId: getOpioidRisk
      summary: Get opioid risk assessment
      description: |
        Evaluates opioid-related risks and tapering readiness
      tags:
        - Opioid Management
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
      responses:
        '200':
          description: Opioid risk assessment retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/OpioidRiskResponse'

  /opioid-taper/plan:
    post:
      operationId: createTaperPlan
      summary: Create opioid tapering plan
      tags:
        - Opioid Management
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TaperPlanRequest'
      responses:
        '200':
          description: Taper plan generated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TaperPlan'

  /predict/chronification:
    post:
      operationId: predictChronification
      summary: Predict chronification risk
      description: |
        Predicts risk of acute pain becoming chronic based on:
        - Pain characteristics
        - Central sensitization markers
        - Psychosocial factors
        - Genetic/epigenetic factors
      tags:
        - Prediction
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ChronificationPredictionRequest'
      responses:
        '200':
          description: Chronification risk prediction
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ChronificationPrediction'

  /predict/response:
    post:
      operationId: predictTreatmentResponse
      summary: Predict treatment response
      description: |
        Predicts likely response to specific treatments
      tags:
        - Prediction
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ResponsePredictionRequest'
      responses:
        '200':
          description: Treatment response prediction
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ResponsePrediction'

components:
  securitySchemes:
    BearerAuth:
      type: http
      scheme: bearer
    ApiKeyAuth:
      type: apiKey
      in: header
      name: X-WIA-API-Key

  schemas:
    AssessmentRequest:
      type: object
      required:
        - patient_id
        - pain_type
      properties:
        patient_id:
          type: string
          format: uuid
        pain_type:
          type: string
          enum: [nociceptive, neuropathic, nociplastic, mixed]
        pain_scores:
          type: object
          properties:
            nrs_current:
              type: integer
            vas_current:
              type: number
            bpi_severity:
              type: number
            bpi_interference:
              type: number
        duration_months:
          type: number
        locations:
          type: array
          items:
            type: object
        sensitization_data:
          type: object
        psychosocial_data:
          type: object

    AssessmentResponse:
      type: object
      properties:
        assessment_id:
          type: string
          format: uuid
        patient_id:
          type: string
          format: uuid
        created_at:
          type: string
          format: date-time
        profile:
          $ref: '#/components/schemas/ChronicPainProfile'
        neuroplasticity_index:
          type: number
        recommendations:
          $ref: '#/components/schemas/TreatmentRecommendations'
        risk_scores:
          type: object
          properties:
            chronification_risk:
              type: number
            opioid_risk:
              type: number
            disability_risk:
              type: number

    SensitizationResponse:
      type: object
      properties:
        patient_id:
          type: string
        assessment_date:
          type: string
          format: date-time
        csi_score:
          type: object
          properties:
            score:
              type: integer
            interpretation:
              type: string
        temporal_summation:
          type: object
          properties:
            present:
              type: boolean
            ratio:
              type: number
        cpm:
          type: object
          properties:
            effect:
              type: number
            status:
              type: string
        neuroplasticity_index:
          type: number
        sensitization_level:
          type: string
          enum: [none, mild, moderate, severe]
        reversal_potential:
          type: number

    NeuromodulationPlanRequest:
      type: object
      required:
        - patient_id
      properties:
        patient_id:
          type: string
          format: uuid
        pain_profile_id:
          type: string
        target_symptoms:
          type: array
          items:
            type: string
        previous_treatments:
          type: array
          items:
            type: object
        contraindications:
          type: array
          items:
            type: string
        preferences:
          type: object
          properties:
            home_based:
              type: boolean
            clinic_based:
              type: boolean
            max_sessions_per_week:
              type: integer

    NeuromodulationPlan:
      type: object
      properties:
        patient_id:
          type: string
        plan_id:
          type: string
        primary_modality:
          type: object
          properties:
            type:
              type: string
              enum: [rtms, tdcs, tens, focused_ultrasound]
            target_region:
              type: string
            parameters:
              type: object
            frequency:
              type: string
            duration_weeks:
              type: integer
            expected_sessions:
              type: integer
        adjunct_modalities:
          type: array
          items:
            type: object
        expected_outcomes:
          type: object
          properties:
            pain_reduction_percent:
              type: number
            function_improvement_percent:
              type: number
        evidence_level:
          type: string
        monitoring_schedule:
          type: array
          items:
            type: object

    TreatmentRecommendations:
      type: object
      properties:
        primary_recommendations:
          type: array
          items:
            type: object
            properties:
              category:
                type: string
                enum: [neuromodulation, behavioral, pharmacological, physical, interventional]
              treatment:
                type: string
              rationale:
                type: string
              evidence_level:
                type: string
              priority:
                type: integer
        opioid_sparing_strategies:
          type: array
          items:
            type: string
        behavioral_interventions:
          type: array
          items:
            type: object
            properties:
              type:
                type: string
              sessions_recommended:
                type: integer
              priority:
                type: integer
        exercise_prescription:
          type: object
          properties:
            type:
              type: array
              items:
                type: string
            frequency:
              type: string
            intensity:
              type: string
            duration:
              type: string
        monitoring_plan:
          type: object

    NeuroplasticityIndex:
      type: object
      properties:
        patient_id:
          type: string
        current_index:
          type: number
          description: 0-100, higher = more maladaptive
        components:
          type: object
          properties:
            central_sensitization:
              type: number
            gray_matter_changes:
              type: number
            functional_connectivity:
              type: number
            psychosocial_factors:
              type: number
        trend:
          type: string
          enum: [improving, stable, worsening]
        reversal_potential:
          type: number
        reversal_timeline_months:
          type: number
        history:
          type: array
          items:
            type: object
            properties:
              date:
                type: string
              index:
                type: number

    OpioidRiskResponse:
      type: object
      properties:
        patient_id:
          type: string
        current_mme:
          type: number
        opioid_risk_tool_score:
          type: integer
        risk_level:
          type: string
          enum: [low, moderate, high]
        tapering_readiness:
          type: object
          properties:
            score:
              type: number
            barriers:
              type: array
              items:
                type: string
            facilitators:
              type: array
              items:
                type: string
        recommendations:
          type: array
          items:
            type: string

    TaperPlan:
      type: object
      properties:
        patient_id:
          type: string
        current_mme:
          type: number
        target_mme:
          type: number
        duration_weeks:
          type: integer
        schedule:
          type: array
          items:
            type: object
            properties:
              week:
                type: integer
              mme:
                type: number
              percentage_reduction:
                type: number
        support_interventions:
          type: array
          items:
            type: string
        monitoring_schedule:
          type: array
          items:
            type: object

    ChronificationPrediction:
      type: object
      properties:
        patient_id:
          type: string
        prediction_date:
          type: string
          format: date-time
        chronification_probability:
          type: number
        risk_level:
          type: string
          enum: [low, moderate, high, very_high]
        contributing_factors:
          type: array
          items:
            type: object
            properties:
              factor:
                type: string
              contribution:
                type: number
        preventive_actions:
          type: array
          items:
            type: string

    ChronicPainProfile:
      $ref: 'PHASE-1-DATA-FORMAT.md#/3.1'

    PainScoresResponse:
      type: object
      properties:
        patient_id:
          type: string
        scores:
          type: array
          items:
            type: object

    ResponsePrediction:
      type: object
      properties:
        treatment:
          type: string
        predicted_response:
          type: string
          enum: [none, partial, good, excellent]
        confidence:
          type: number
        factors:
          type: array
          items:
            type: object

  responses:
    BadRequest:
      description: Invalid request
      content:
        application/json:
          schema:
            type: object
            properties:
              error:
                type: string
              details:
                type: array
    Unauthorized:
      description: Authentication required
      content:
        application/json:
          schema:
            type: object
            properties:
              error:
                type: string
```

---

## 3. Example API Calls

### 3.1 Create Assessment

```bash
curl -X POST https://api.wia.live/chronic-pain/v1/assess \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "patient_id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
    "pain_type": "nociplastic",
    "pain_scores": {
      "nrs_current": 7,
      "bpi_severity": 6.5,
      "bpi_interference": 7.2
    },
    "duration_months": 36,
    "locations": [
      {"body_region": "lower_back", "laterality": "bilateral"}
    ],
    "sensitization_data": {
      "csi_score": 62,
      "temporal_summation_present": true
    }
  }'
```

### 3.2 Get Neuromodulation Plan

```bash
curl -X POST https://api.wia.live/chronic-pain/v1/neuromodulation/plan \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "patient_id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
    "target_symptoms": ["pain_intensity", "central_sensitization"],
    "preferences": {
      "home_based": false,
      "clinic_based": true
    }
  }'
```

### 3.3 Get Neuroplasticity Index

```bash
curl -X GET "https://api.wia.live/chronic-pain/v1/neuroplasticity/index/b2c3d4e5-f6a7-8901-bcde-f12345678901" \
  -H "Authorization: Bearer $API_KEY"
```

---

## 4. Webhooks

### 4.1 Available Events

| Event | Description |
|-------|-------------|
| `assessment.completed` | New pain assessment completed |
| `sensitization.severe` | Severe central sensitization detected |
| `neuroplasticity.improved` | Significant neuroplasticity improvement |
| `opioid.high_risk` | High opioid risk detected |
| `pain.escalation` | Pain score escalation alert |

### 4.2 Webhook Payload

```json
{
  "event": "sensitization.severe",
  "timestamp": "2026-01-04T14:30:00Z",
  "data": {
    "patient_id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
    "csi_score": 72,
    "neuroplasticity_index": 78,
    "recommended_actions": ["urgent_review", "neuromodulation_evaluation"]
  }
}
```

---

## 5. Rate Limits

| Plan | Requests/min | Assessments/day |
|------|--------------|-----------------|
| Free | 10 | 50 |
| Professional | 100 | 500 |
| Enterprise | 1000 | Unlimited |

---

## 6. Error Codes

| Code | Description |
|------|-------------|
| CP-001 | Invalid patient ID |
| CP-002 | Invalid pain type |
| CP-003 | Pain scores out of range |
| CP-004 | Invalid duration (must be ≥3 months) |
| CP-005 | Missing required assessment data |
| CP-006 | Neuromodulation contraindication detected |

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
