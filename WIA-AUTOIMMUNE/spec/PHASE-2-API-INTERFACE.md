# WIA-AUTOIMMUNE Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2026-01-04
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

The WIA-AUTOIMMUNE API provides RESTful endpoints for autoimmune disease assessment, Treg-Microbiome axis analysis, and personalized treatment recommendations.

### 1.1 Base URL

```
Production: https://api.wia.live/autoimmune/v1
Staging:    https://api-staging.wia.live/autoimmune/v1
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
  title: WIA-AUTOIMMUNE API
  version: 1.0.0
  description: |
    API for autoimmune disease assessment with Treg-Microbiome Axis integration.
    弘益人間 (홍익인간) - Benefit All Humanity
  contact:
    name: WIA Standards Team
    url: https://wia.live/autoimmune
  license:
    name: MIT
    url: https://opensource.org/licenses/MIT

servers:
  - url: https://api.wia.live/autoimmune/v1
    description: Production
  - url: https://api-staging.wia.live/autoimmune/v1
    description: Staging

security:
  - BearerAuth: []
  - ApiKeyAuth: []

paths:
  /assess:
    post:
      operationId: createAssessment
      summary: Create comprehensive autoimmune assessment
      description: |
        Performs full autoimmune profile assessment including:
        - Treg functional analysis
        - Microbiome evaluation
        - Disease activity scoring
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
      summary: Get patient autoimmune profile
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
              enum: [treg, microbiome, history, treatments]
      responses:
        '200':
          description: Patient profile retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/AutoimmuneProfile'

  /treg-status/{patient_id}:
    get:
      operationId: getTregStatus
      summary: Get Treg functional status
      description: |
        Retrieves detailed regulatory T cell assessment including:
        - Cell counts and phenotype
        - FOXP3 expression stability
        - Suppressive function assay results
        - GRAIL E3 ligase activity
      tags:
        - Treg Analysis
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
        - name: date_from
          in: query
          schema:
            type: string
            format: date
        - name: date_to
          in: query
          schema:
            type: string
            format: date
      responses:
        '200':
          description: Treg status retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TregStatusResponse'

  /microbiome/{patient_id}:
    get:
      operationId: getMicrobiomeAnalysis
      summary: Get gut microbiome analysis
      description: |
        Retrieves microbiome analysis with focus on:
        - SCFA-producing bacteria abundance
        - Dysbiosis scoring
        - Leaky gut markers
        - Treg-modulating species
      tags:
        - Microbiome
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
        - name: include_taxa
          in: query
          schema:
            type: boolean
            default: true
        - name: scfa_focus
          in: query
          schema:
            type: boolean
            default: true
      responses:
        '200':
          description: Microbiome analysis retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/MicrobiomeResponse'

    post:
      operationId: submitMicrobiomeSample
      summary: Submit new microbiome sample
      tags:
        - Microbiome
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
              $ref: '#/components/schemas/MicrobiomeSampleRequest'
      responses:
        '202':
          description: Sample accepted for processing
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/SampleSubmissionResponse'

  /treatment/recommend:
    post:
      operationId: getRecommendations
      summary: Get personalized treatment recommendations
      description: |
        Generates evidence-based treatment recommendations based on:
        - Current disease activity
        - Treg functional status
        - Microbiome composition
        - Treatment history

        Recommendations may include:
        - Low-dose IL-2 therapy
        - FMT (Fecal Microbiota Transplantation)
        - Dietary interventions (prebiotics, specific fiber)
        - CAR-Treg therapy eligibility
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

  /disease-activity/{patient_id}:
    get:
      operationId: getDiseaseActivity
      summary: Get disease activity scores
      tags:
        - Disease Activity
      parameters:
        - name: patient_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
        - name: disease_type
          in: query
          schema:
            type: string
            enum: [RA, SLE, MS, T1D, IBD, PSO, HT, GD]
      responses:
        '200':
          description: Disease activity retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DiseaseActivityResponse'

    post:
      operationId: recordDiseaseActivity
      summary: Record new disease activity measurement
      tags:
        - Disease Activity
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
              $ref: '#/components/schemas/DiseaseActivityRecord'
      responses:
        '201':
          description: Activity recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DiseaseActivityResponse'

  /flare/predict:
    post:
      operationId: predictFlare
      summary: Predict flare risk
      description: |
        Uses Treg-Microbiome axis data to predict flare probability
      tags:
        - Prediction
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/FlarePredictionRequest'
      responses:
        '200':
          description: Flare prediction generated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/FlarePrediction'

  /remission/assess:
    post:
      operationId: assessRemission
      summary: Assess remission probability
      description: |
        Evaluates likelihood of achieving/maintaining remission
        based on Treg function and microbiome status
      tags:
        - Prediction
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/RemissionAssessmentRequest'
      responses:
        '200':
          description: Remission assessment completed
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RemissionAssessment'

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
        - disease_type
      properties:
        patient_id:
          type: string
          format: uuid
        disease_type:
          type: string
          enum: [RA, SLE, MS, T1D, IBD, PSO, HT, GD]
        immune_panel:
          type: object
          properties:
            treg_count:
              type: number
            foxp3_expression:
              type: number
            suppressive_function:
              type: number
            th17_count:
              type: number
            autoantibodies:
              type: array
              items:
                type: object
            cytokines:
              type: object
        microbiome_sample_id:
          type: string
          description: Reference to existing microbiome sample
        clinical_observations:
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
          $ref: '#/components/schemas/AutoimmuneProfile'
        recommendations:
          $ref: '#/components/schemas/TreatmentRecommendations'
        risk_scores:
          type: object
          properties:
            flare_risk:
              type: number
            progression_risk:
              type: number
            remission_likelihood:
              type: number

    TregStatusResponse:
      type: object
      properties:
        patient_id:
          type: string
        assessment_date:
          type: string
          format: date-time
        treg:
          type: object
          properties:
            count:
              type: object
              properties:
                value:
                  type: number
                unit:
                  type: string
                status:
                  type: string
                  enum: [low, normal, high]
            foxp3_expression:
              type: number
            suppressive_function:
              type: number
            grail_activity:
              type: number
            stability_score:
              type: number
        th17_treg_ratio:
          type: number
        functional_status:
          type: string
          enum: [normal, mildly_impaired, moderately_impaired, severely_impaired]
        clinical_interpretation:
          type: string

    MicrobiomeResponse:
      type: object
      properties:
        patient_id:
          type: string
        sample_date:
          type: string
          format: date-time
        diversity:
          type: object
          properties:
            shannon:
              type: number
            simpson:
              type: number
            status:
              type: string
        scfa_production:
          type: object
          properties:
            butyrate:
              type: object
            propionate:
              type: object
            acetate:
              type: object
            overall_status:
              type: string
        dysbiosis:
          type: object
          properties:
            score:
              type: number
            severity:
              type: string
            key_findings:
              type: array
              items:
                type: string
        leaky_gut:
          type: object
          properties:
            zonulin:
              type: number
            lps:
              type: number
            status:
              type: string
        treg_modulators:
          type: object
          description: Bacteria known to influence Treg function

    TreatmentRecommendations:
      type: object
      properties:
        primary_recommendations:
          type: array
          items:
            type: object
            properties:
              treatment_type:
                type: string
                enum: [low_dose_il2, fmt, car_treg, dietary, immunosuppressant, biologic]
              name:
                type: string
              rationale:
                type: string
              evidence_level:
                type: string
                enum: [A, B, C, D]
              priority:
                type: integer
        dietary_interventions:
          type: array
          items:
            type: object
            properties:
              intervention:
                type: string
              target:
                type: string
              expected_benefit:
                type: string
        monitoring_plan:
          type: object
          properties:
            frequency:
              type: string
            markers_to_track:
              type: array
              items:
                type: string

    FlarePrediction:
      type: object
      properties:
        patient_id:
          type: string
        prediction_date:
          type: string
          format: date-time
        flare_probability:
          type: number
          description: 0-100 probability of flare in next 30 days
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
              trend:
                type: string
        preventive_actions:
          type: array
          items:
            type: string

    AutoimmuneProfile:
      $ref: 'PHASE-1-DATA-FORMAT.md#/3.1'

    DiseaseActivityResponse:
      type: object
      properties:
        patient_id:
          type: string
        disease_type:
          type: string
        current_activity:
          type: object
        history:
          type: array
          items:
            type: object

    RemissionAssessment:
      type: object
      properties:
        remission_probability:
          type: number
        current_status:
          type: string
        factors_supporting:
          type: array
          items:
            type: string
        factors_against:
          type: array
          items:
            type: string
        time_to_remission_estimate:
          type: string

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
curl -X POST https://api.wia.live/autoimmune/v1/assess \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "patient_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "disease_type": "RA",
    "immune_panel": {
      "treg_count": 45.2,
      "foxp3_expression": 68.5,
      "suppressive_function": 42.0,
      "th17_count": 145.0,
      "autoantibodies": [
        {"name": "RF", "value": 85.0, "positive": true},
        {"name": "Anti-CCP", "value": 320.0, "positive": true}
      ],
      "cytokines": {
        "il6": 28.5,
        "il17": 45.2,
        "tnf_alpha": 32.1
      }
    },
    "microbiome_sample_id": "sample_xyz123"
  }'
```

### 3.2 Get Treg Status

```bash
curl -X GET "https://api.wia.live/autoimmune/v1/treg-status/a1b2c3d4-e5f6-7890-abcd-ef1234567890" \
  -H "Authorization: Bearer $API_KEY"
```

### 3.3 Get Treatment Recommendations

```bash
curl -X POST https://api.wia.live/autoimmune/v1/treatment/recommend \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "patient_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "current_profile_id": "profile_abc123",
    "treatment_history": [],
    "preferences": {
      "avoid_immunosuppressants": false,
      "open_to_experimental": true
    }
  }'
```

---

## 4. Webhooks

### 4.1 Available Events

| Event | Description |
|-------|-------------|
| `assessment.completed` | New assessment completed |
| `flare.predicted` | High flare risk detected |
| `remission.achieved` | Patient achieved remission |
| `microbiome.analyzed` | Microbiome sample processed |
| `treg.critical` | Critical Treg dysfunction detected |

### 4.2 Webhook Payload

```json
{
  "event": "flare.predicted",
  "timestamp": "2026-01-04T10:30:00Z",
  "data": {
    "patient_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "flare_probability": 78.5,
    "risk_level": "high",
    "recommended_actions": ["increase_monitoring", "consider_intervention"]
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
| AI-001 | Invalid patient ID |
| AI-002 | Invalid disease type |
| AI-003 | Incomplete immune panel |
| AI-004 | Microbiome sample not found |
| AI-005 | Assessment conflict |

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
