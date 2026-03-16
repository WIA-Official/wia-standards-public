# WIA-CONSCIOUSNESS: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the REST API interface for consciousness measurement, data exchange, and assessment services. All API implementations MUST adhere to these specifications for interoperability.

## 2. Base Configuration

### 2.1 Base URL

```
Production: https://api.wia.live/consciousness/v1
Staging: https://staging-api.wia.live/consciousness/v1
Development: http://localhost:3000/consciousness/v1
```

### 2.2 Authentication

```yaml
securitySchemes:
  BearerAuth:
    type: http
    scheme: bearer
    bearerFormat: JWT
  ApiKeyAuth:
    type: apiKey
    in: header
    name: X-WIA-API-Key
```

### 2.3 Common Headers

```yaml
headers:
  X-WIA-Request-ID:
    description: Unique request identifier
    schema:
      type: string
      format: uuid
  X-WIA-Client-Version:
    description: Client SDK version
    schema:
      type: string
  Accept-Language:
    description: Preferred response language
    schema:
      type: string
      default: en
```

## 3. API Endpoints

### 3.1 Consciousness Measurement

#### POST /measure

Create a new consciousness measurement session.

```yaml
/api/v1/consciousness/measure:
  post:
    summary: Initiate consciousness measurement
    operationId: createMeasurement
    tags:
      - Measurement
    security:
      - BearerAuth: []
    requestBody:
      required: true
      content:
        application/json:
          schema:
            type: object
            required:
              - subject_id
              - method
            properties:
              subject_id:
                type: string
                format: uuid
              method:
                type: string
                enum: [TMS-EEG, fMRI, MEG, iEEG, EEG, combined]
              protocol:
                type: string
                enum: [standard, extended, rapid]
                default: standard
              target_region:
                type: string
                enum: [premotor, parietal, occipital, frontal]
                default: premotor
              parameters:
                type: object
                properties:
                  tms_intensity:
                    type: number
                    minimum: 0
                    maximum: 100
                  pulse_count:
                    type: integer
                    default: 200
                  sampling_rate:
                    type: number
                    default: 5000
    responses:
      201:
        description: Measurement initiated
        content:
          application/json:
            schema:
              type: object
              properties:
                measurement_id:
                  type: string
                  format: uuid
                status:
                  type: string
                  enum: [initiated, in_progress, completed, failed]
                estimated_duration:
                  type: integer
                  description: Estimated duration in seconds
      400:
        description: Invalid request parameters
      401:
        description: Unauthorized
      422:
        description: Unprocessable entity
```

#### GET /measure/{measurement_id}

Retrieve measurement results.

```yaml
/api/v1/consciousness/measure/{measurement_id}:
  get:
    summary: Get measurement results
    operationId: getMeasurement
    tags:
      - Measurement
    security:
      - BearerAuth: []
    parameters:
      - name: measurement_id
        in: path
        required: true
        schema:
          type: string
          format: uuid
      - name: include
        in: query
        schema:
          type: array
          items:
            type: string
            enum: [iit_metrics, gnw_metrics, raw_data, visualizations]
    responses:
      200:
        description: Measurement results
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ConsciousnessIndex'
      404:
        description: Measurement not found
```

### 3.2 State Assessment

#### GET /state/{subject_id}

Get current consciousness state for a subject.

```yaml
/api/v1/consciousness/state/{subject_id}:
  get:
    summary: Get consciousness state
    operationId: getConsciousnessState
    tags:
      - State
    security:
      - BearerAuth: []
    parameters:
      - name: subject_id
        in: path
        required: true
        schema:
          type: string
          format: uuid
      - name: timeframe
        in: query
        schema:
          type: string
          enum: [current, 1h, 24h, 7d]
          default: current
    responses:
      200:
        description: Consciousness state
        content:
          application/json:
            schema:
              type: object
              properties:
                subject_id:
                  type: string
                state:
                  $ref: '#/components/schemas/ConsciousnessState'
                last_updated:
                  type: string
                  format: date-time
                trend:
                  type: string
                  enum: [improving, stable, declining]
```

#### PUT /state/{subject_id}

Update consciousness state (for clinical use).

```yaml
/api/v1/consciousness/state/{subject_id}:
  put:
    summary: Update consciousness state
    operationId: updateConsciousnessState
    tags:
      - State
    security:
      - BearerAuth: []
    requestBody:
      required: true
      content:
        application/json:
          schema:
            type: object
            properties:
              condition:
                type: string
              arousal_level:
                type: number
              awareness_level:
                type: number
              clinical_notes:
                type: string
    responses:
      200:
        description: State updated
      404:
        description: Subject not found
```

### 3.3 PCI Measurement

#### GET /pci/{subject_id}

Get PCI measurement for a subject.

```yaml
/api/v1/consciousness/pci/{subject_id}:
  get:
    summary: Get PCI measurement
    operationId: getPCI
    tags:
      - PCI
    security:
      - BearerAuth: []
    parameters:
      - name: subject_id
        in: path
        required: true
        schema:
          type: string
          format: uuid
      - name: measurement_id
        in: query
        schema:
          type: string
          format: uuid
          description: Specific measurement ID (optional, defaults to latest)
    responses:
      200:
        description: PCI measurement
        content:
          application/json:
            schema:
              type: object
              properties:
                subject_id:
                  type: string
                pci:
                  type: object
                  properties:
                    value:
                      type: number
                      minimum: 0
                      maximum: 1
                    classification:
                      type: string
                      enum: [VS_UWS, MCS_minus, MCS_plus, EMCS, LIS, conscious]
                    threshold_status:
                      type: string
                      enum: [below, at, above]
                    confidence:
                      type: number
                complexity_measures:
                  type: object
                  properties:
                    lempel_ziv:
                      type: number
                    entropy:
                      type: number
                timestamp:
                  type: string
                  format: date-time
```

### 3.4 Phi (Φ) Estimation

#### GET /phi/{subject_id}

Get integrated information (Φ) estimate.

```yaml
/api/v1/consciousness/phi/{subject_id}:
  get:
    summary: Get Φ estimate
    operationId: getPhiEstimate
    tags:
      - IIT
    security:
      - BearerAuth: []
    parameters:
      - name: subject_id
        in: path
        required: true
        schema:
          type: string
          format: uuid
      - name: method
        in: query
        schema:
          type: string
          enum: [exact, approximation, upper_bound]
          default: approximation
    responses:
      200:
        description: Φ estimate
        content:
          application/json:
            schema:
              type: object
              properties:
                subject_id:
                  type: string
                phi_estimate:
                  type: object
                  properties:
                    value:
                      type: number
                    unit:
                      type: string
                      default: bits
                    method:
                      type: string
                    computation_time:
                      type: number
                      description: Computation time in seconds
                iit_metrics:
                  $ref: '#/components/schemas/IITMetrics'
                timestamp:
                  type: string
                  format: date-time
```

### 3.5 Theory Comparison

#### POST /compare

Compare IIT and GNW theory predictions.

```yaml
/api/v1/consciousness/compare:
  post:
    summary: Compare theory predictions
    operationId: compareTheories
    tags:
      - Analysis
    security:
      - BearerAuth: []
    requestBody:
      required: true
      content:
        application/json:
          schema:
            type: object
            required:
              - measurement_id
            properties:
              measurement_id:
                type: string
                format: uuid
              theories:
                type: array
                items:
                  type: string
                  enum: [IIT, GNW, HOT, PP, AST]
                default: [IIT, GNW]
    responses:
      200:
        description: Theory comparison results
        content:
          application/json:
            schema:
              type: object
              properties:
                measurement_id:
                  type: string
                comparisons:
                  type: array
                  items:
                    type: object
                    properties:
                      theory:
                        type: string
                      predictions:
                        type: array
                        items:
                          type: object
                          properties:
                            prediction:
                              type: string
                            observed:
                              type: boolean
                            confidence:
                              type: number
                      overall_support:
                        type: number
                        minimum: 0
                        maximum: 1
```

### 3.6 AI Consciousness Assessment

#### POST /ai/assess

Assess consciousness indicators in an AI system.

```yaml
/api/v1/consciousness/ai/assess:
  post:
    summary: Assess AI consciousness indicators
    operationId: assessAIConsciousness
    tags:
      - AI Assessment
    security:
      - BearerAuth: []
    requestBody:
      required: true
      content:
        application/json:
          schema:
            type: object
            required:
              - system_id
              - system_type
            properties:
              system_id:
                type: string
              system_type:
                type: string
                enum: [LLM, multimodal, embodied, hybrid]
              model_info:
                type: object
                properties:
                  name:
                    type: string
                  version:
                    type: string
                  parameters:
                    type: number
              test_battery:
                type: array
                items:
                  type: string
                  enum: [recurrent, global_workspace, hot, attention, predictive, agency, tom, metacognition]
    responses:
      200:
        description: AI assessment results
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/AIConsciousnessAssessment'
```

### 3.7 Real-time Monitoring

#### WebSocket /monitor/{subject_id}

Real-time consciousness monitoring stream.

```yaml
/ws/consciousness/monitor/{subject_id}:
  description: WebSocket endpoint for real-time monitoring
  parameters:
    - name: subject_id
      in: path
      required: true
      schema:
        type: string
        format: uuid
  messages:
    subscribe:
      payload:
        type: object
        properties:
          action:
            type: string
            enum: [subscribe, unsubscribe]
          metrics:
            type: array
            items:
              type: string
              enum: [pci, phi, gnw, raw_eeg]
    update:
      payload:
        type: object
        properties:
          timestamp:
            type: string
            format: date-time
          pci:
            type: number
          phi_estimate:
            type: number
          state:
            type: string
          alert:
            type: object
            properties:
              type:
                type: string
                enum: [threshold_crossed, state_change, anomaly]
              message:
                type: string
```

## 4. Component Schemas

### 4.1 ConsciousnessIndex

```yaml
components:
  schemas:
    ConsciousnessIndex:
      type: object
      properties:
        subject_id:
          type: string
          format: uuid
        timestamp:
          type: string
          format: date-time
        iit_metrics:
          $ref: '#/components/schemas/IITMetrics'
        gnw_metrics:
          $ref: '#/components/schemas/GNWMetrics'
        practical_measures:
          $ref: '#/components/schemas/PracticalMeasures'
        state:
          $ref: '#/components/schemas/ConsciousnessState'
        measurement:
          $ref: '#/components/schemas/MeasurementInfo'
```

### 4.2 IITMetrics

```yaml
    IITMetrics:
      type: object
      properties:
        phi_estimate:
          type: object
          properties:
            value:
              type: number
            unit:
              type: string
            computation_method:
              type: string
        cause_effect_power:
          type: number
        irreducibility:
          type: number
        intrinsicality:
          type: number
```

### 4.3 GNWMetrics

```yaml
    GNWMetrics:
      type: object
      properties:
        global_ignition:
          type: number
        prefrontal_activation:
          type: number
        broadcast_strength:
          type: number
        workspace_access:
          type: boolean
```

### 4.4 Error Responses

```yaml
    Error:
      type: object
      properties:
        code:
          type: string
        message:
          type: string
        details:
          type: object
        request_id:
          type: string
          format: uuid

    ErrorCodes:
      description: |
        - INVALID_SUBJECT: Subject ID not found
        - INVALID_METHOD: Unsupported measurement method
        - MEASUREMENT_FAILED: Measurement could not be completed
        - COMPUTATION_ERROR: Error during Φ computation
        - RATE_LIMITED: Too many requests
        - UNAUTHORIZED: Invalid or expired token
```

## 5. Rate Limits

| Endpoint Category | Rate Limit | Burst |
|-------------------|------------|-------|
| Measurement | 10/min | 20 |
| State Queries | 60/min | 100 |
| PCI/Phi Queries | 30/min | 50 |
| AI Assessment | 5/min | 10 |
| WebSocket | 1 connection/subject | - |

## 6. SDK Examples

### 6.1 JavaScript/TypeScript

```typescript
import { ConsciousnessClient } from '@wia/consciousness';

const client = new ConsciousnessClient({
  apiKey: 'your-api-key',
  baseUrl: 'https://api.wia.live/consciousness/v1'
});

// Measure consciousness
const measurement = await client.measure({
  subject_id: 'patient-001',
  method: 'TMS-EEG',
  parameters: {
    tms_intensity: 80,
    pulse_count: 200
  }
});

// Get PCI value
const pci = await client.getPCI(measurement.id);
console.log(`PCI: ${pci.value}, State: ${pci.classification}`);

// Real-time monitoring
client.monitor('patient-001', (update) => {
  console.log(`PCI: ${update.pci}, Φ: ${update.phi_estimate}`);
  if (update.alert) {
    console.warn(`Alert: ${update.alert.message}`);
  }
});
```

### 6.2 Python

```python
from wia_consciousness import ConsciousnessClient

client = ConsciousnessClient(
    api_key='your-api-key',
    base_url='https://api.wia.live/consciousness/v1'
)

# Measure consciousness
measurement = client.measure(
    subject_id='patient-001',
    method='TMS-EEG',
    parameters={
        'tms_intensity': 80,
        'pulse_count': 200
    }
)

# Get results
pci = client.get_pci(measurement.id)
print(f"PCI: {pci.value}, State: {pci.classification}")

# AI assessment
ai_result = client.assess_ai(
    system_id='gpt-4',
    system_type='LLM',
    model_info={'name': 'GPT-4', 'parameters': 1.8e12}
)
print(f"Consciousness likelihood: {ai_result.consciousness_likelihood}")
```

---

**弘益人間 (弘益人間) - Benefit All Humanity**
© 2025 WIA Standards · MIT License
