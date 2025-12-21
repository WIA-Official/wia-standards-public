# WIA AI-Human Coexistence Phase 2: API Interface

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-01
**Primary Color:** #10B981 (Emerald)

---

## Table of Contents

1. [Overview](#1-overview)
2. [REST API Endpoints](#2-rest-api-endpoints)
   - 2.1 [Human Detection API](#21-human-detection-api)
   - 2.2 [Zone Management API](#22-zone-management-api)
   - 2.3 [Interaction Mode API](#23-interaction-mode-api)
   - 2.4 [Behavioral Adaptation API](#24-behavioral-adaptation-api)
   - 2.5 [Intent Prediction API](#25-intent-prediction-api)
   - 2.6 [Communication API](#26-communication-api)
3. [WebSocket Interfaces](#3-websocket-interfaces)
4. [SDK Examples](#4-sdk-examples)
   - 4.1 [TypeScript SDK](#41-typescript-sdk)
   - 4.2 [Python SDK](#42-python-sdk)
5. [Authentication & Authorization](#5-authentication--authorization)
6. [Error Handling](#6-error-handling)
7. [Rate Limiting](#7-rate-limiting)
8. [API Versioning](#8-api-versioning)

---

## 1. Overview

WIA AI-Human Coexistence API는 AI 시스템이 인간과 안전하고 조화롭게 상호작용하기 위한 RESTful 및 WebSocket 인터페이스를 제공합니다.

### Base URL
```
https://api.wia-standards.org/ai-human-coexistence/v1
```

### Supported Protocols
- **REST API**: HTTP/2, HTTPS required
- **WebSocket**: WSS for real-time streaming
- **gRPC**: High-performance device communication
- **MQTT**: IoT sensor integration

### Authentication
- **API Key**: Required for all requests
- **OAuth 2.0**: For user-delegated access
- **mTLS**: For device-to-device communication

---

## 2. REST API Endpoints

### 2.1 Human Detection API

#### POST /detection/scan
실시간 인간 감지를 수행합니다.

**Request:**
```typescript
interface DetectionScanRequest {
  device_id: string;
  sensor_data: {
    lidar?: LidarData;
    camera?: CameraData;
    depth?: DepthData;
    thermal?: ThermalData;
  };
  scan_options?: {
    enable_tracking: boolean;
    enable_pose_estimation: boolean;
    enable_vulnerability_detection: boolean;
    confidence_threshold: number;
  };
}
```

**Response:**
```typescript
interface DetectionScanResponse {
  detection_id: string;
  timestamp: string;
  humans: HumanDetection[];
  processing_time_ms: number;
  status: "success" | "partial" | "failed";
}
```

**Example:**
```bash
curl -X POST https://api.wia-standards.org/ai-human-coexistence/v1/detection/scan \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "device_id": "robot-001",
    "sensor_data": {
      "lidar": { "points": [...], "timestamp": "2025-01-15T14:30:45.123Z" }
    },
    "scan_options": {
      "enable_tracking": true,
      "confidence_threshold": 0.8
    }
  }'
```

#### GET /detection/tracking/{tracking_id}
특정 추적 세션의 정보를 조회합니다.

**Response:**
```typescript
interface TrackingResponse {
  tracking_id: string;
  human_id: string;
  first_seen: string;
  last_seen: string;
  current_position: Position3D;
  trajectory_history: Position3D[];
  tracking_quality: "excellent" | "good" | "fair" | "poor";
  predicted_trajectory: PredictedPosition[];
}
```

#### DELETE /detection/tracking/{tracking_id}
추적 세션을 종료합니다.

**Response:**
```typescript
interface TrackingDeleteResponse {
  tracking_id: string;
  deleted: boolean;
  final_duration_seconds: number;
}
```

### 2.2 Zone Management API

#### GET /zones/config
현재 근접 구역 설정을 조회합니다.

**Response:**
```typescript
interface ZoneConfigResponse {
  device_id: string;
  zones: {
    danger_zone: ZoneConfig;
    warning_zone: ZoneConfig;
    collaborative_zone: ZoneConfig;
    safe_zone: ZoneConfig;
  };
  dynamic_adjustments: DynamicAdjustments;
  last_updated: string;
}
```

#### PUT /zones/config
근접 구역 설정을 업데이트합니다.

**Request:**
```typescript
interface ZoneConfigUpdateRequest {
  device_id: string;
  zone_updates: {
    zone_type: ZoneType;
    radius_m?: number;
    max_speed_ms?: number;
    response_action?: string;
  }[];
  reason: string;
}
```

**Response:**
```typescript
interface ZoneConfigUpdateResponse {
  updated: boolean;
  new_config: ZoneConfigResponse;
  validation_warnings: string[];
}
```

#### GET /zones/status
현재 구역 상태를 조회합니다.

**Response:**
```typescript
interface ZoneStatusResponse {
  device_id: string;
  timestamp: string;
  zone_occupancy: {
    danger_zone: { humans: string[]; count: number };
    warning_zone: { humans: string[]; count: number };
    collaborative_zone: { humans: string[]; count: number };
    safe_zone: { humans: string[]; count: number };
  };
  highest_alert_level: ZoneType;
  recommended_action: string;
  closest_human_distance_m: number;
}
```

### 2.3 Interaction Mode API

#### POST /interaction/start
인간과의 상호작용을 시작합니다.

**Request:**
```typescript
interface InteractionStartRequest {
  device_id: string;
  human_id: string;
  interaction_type: InteractionType;
  communication_channels: string[];
  collaboration_params?: {
    task_id: string;
    human_role: string;
    robot_role: string;
    coordination_mode: "human_led" | "robot_led" | "collaborative";
  };
}
```

**Response:**
```typescript
interface InteractionStartResponse {
  interaction_id: string;
  status: "initiated" | "pending_human_confirmation" | "active";
  estimated_duration_seconds?: number;
  safety_parameters: SafetyParameters;
}
```

#### PUT /interaction/{interaction_id}/update
상호작용 상태를 업데이트합니다.

**Request:**
```typescript
interface InteractionUpdateRequest {
  interaction_phase: InteractionPhase;
  human_state?: {
    attention_level: string;
    engagement_score: number;
    comfort_level: string;
  };
  robot_adaptations?: {
    speed_reduction_percent: number;
    communication_frequency_hz: number;
  };
}
```

#### POST /interaction/{interaction_id}/end
상호작용을 종료합니다.

**Request:**
```typescript
interface InteractionEndRequest {
  reason: string;
  completion_status: "completed" | "interrupted" | "failed";
  feedback?: {
    success_rating: number;
    issues_encountered: string[];
  };
}
```

**Response:**
```typescript
interface InteractionEndResponse {
  interaction_id: string;
  ended: boolean;
  total_duration_seconds: number;
  summary: InteractionSummary;
}
```

### 2.4 Behavioral Adaptation API

#### POST /adaptation/evaluate
현재 상황에 대한 행동 적응을 평가합니다.

**Request:**
```typescript
interface AdaptationEvaluateRequest {
  device_id: string;
  current_state: {
    position: Position3D;
    velocity: Velocity3D;
    behavioral_mode: BehavioralMode;
  };
  detected_humans: HumanDetection[];
  social_context: SocialContext;
}
```

**Response:**
```typescript
interface AdaptationEvaluateResponse {
  adaptation_id: string;
  recommended_adaptations: Adaptation[];
  priority_order: string[];
  estimated_safety_improvement: number;
  estimated_comfort_improvement: number;
}
```

#### POST /adaptation/apply
평가된 적응을 적용합니다.

**Request:**
```typescript
interface AdaptationApplyRequest {
  device_id: string;
  adaptation_id: string;
  adaptations_to_apply: string[];
  override_safety_check?: boolean;
}
```

**Response:**
```typescript
interface AdaptationApplyResponse {
  applied: boolean;
  active_adaptations: Adaptation[];
  behavioral_mode: BehavioralMode;
  performance_metrics: PerformanceMetrics;
}
```

#### GET /adaptation/history
적응 이력을 조회합니다.

**Query Parameters:**
- `device_id` (required): Device ID
- `start_time` (optional): Start timestamp
- `end_time` (optional): End timestamp
- `limit` (optional): Maximum results (default: 100)

**Response:**
```typescript
interface AdaptationHistoryResponse {
  device_id: string;
  adaptations: BehavioralAdaptation[];
  total_count: number;
  statistics: {
    total_adaptations: number;
    avg_latency_ms: number;
    success_rate: number;
  };
}
```

### 2.5 Intent Prediction API

#### POST /intent/predict
인간의 의도를 예측합니다.

**Request:**
```typescript
interface IntentPredictRequest {
  device_id: string;
  human_id: string;
  observation_window_s: number;
  behavioral_cues: {
    gaze_direction?: string;
    body_orientation_deg?: number;
    movement_pattern?: string;
    gesture_detected?: boolean;
  };
  contextual_factors?: {
    task_context?: string;
    environmental_constraints?: string[];
    social_factors?: string[];
  };
}
```

**Response:**
```typescript
interface IntentPredictResponse {
  prediction_id: string;
  predicted_intent: {
    primary_intent: IntentType;
    confidence: number;
    time_horizon_s: number;
  };
  intent_alternatives: IntentAlternative[];
  trajectory_prediction: TrajectoryPrediction;
  recommended_actions: RecommendedAction[];
  collision_probability: number;
  interaction_probability: number;
}
```

#### GET /intent/model/info
의도 예측 모델 정보를 조회합니다.

**Response:**
```typescript
interface IntentModelInfoResponse {
  model_type: string;
  model_version: string;
  training_dataset: string;
  last_updated: string;
  supported_intents: string[];
  average_accuracy: number;
  inference_time_ms: number;
}
```

### 2.6 Communication API

#### POST /communication/send
인간에게 메시지를 전송합니다.

**Request:**
```typescript
interface CommunicationSendRequest {
  device_id: string;
  human_id: string;
  modalities: {
    verbal?: {
      message: string;
      language: string;
      tone: string;
      volume_db?: number;
    };
    visual?: {
      led_color?: string;
      led_pattern?: string;
      display_content?: string;
    };
    projection?: {
      projection_type: string;
      projection_color?: string;
    };
    gestural?: {
      gesture_type: string;
    };
  };
  priority: "low" | "normal" | "high" | "urgent";
  expected_response?: string;
}
```

**Response:**
```typescript
interface CommunicationSendResponse {
  communication_id: string;
  sent: boolean;
  timestamp: string;
  delivery_confirmation: boolean;
  estimated_receipt_time_s: number;
}
```

#### GET /communication/{communication_id}/status
통신 상태를 조회합니다.

**Response:**
```typescript
interface CommunicationStatusResponse {
  communication_id: string;
  status: "sent" | "delivered" | "acknowledged" | "failed";
  message_received: boolean;
  message_understood: boolean;
  response_received: boolean;
  response_content?: any;
  effectiveness_score: number;
}
```

---

## 3. WebSocket Interfaces

### 3.1 Real-time Detection Stream

**Endpoint:** `wss://api.wia-standards.org/ai-human-coexistence/v1/stream/detection`

**Connection:**
```typescript
const ws = new WebSocket('wss://api.wia-standards.org/ai-human-coexistence/v1/stream/detection');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'subscribe',
    device_id: 'robot-001',
    options: {
      include_tracking: true,
      include_predictions: true,
      update_frequency_hz: 10
    }
  }));
};

ws.onmessage = (event) => {
  const detection: DetectionStreamMessage = JSON.parse(event.data);
  console.log('Detection update:', detection);
};
```

**Message Format:**
```typescript
interface DetectionStreamMessage {
  type: 'detection_update';
  timestamp: string;
  device_id: string;
  humans: HumanDetection[];
  zone_status: ZoneStatusResponse;
}
```

### 3.2 Zone Alert Stream

**Endpoint:** `wss://api.wia-standards.org/ai-human-coexistence/v1/stream/zones`

**Message Format:**
```typescript
interface ZoneAlertMessage {
  type: 'zone_alert';
  timestamp: string;
  device_id: string;
  alert_level: ZoneType;
  human_id: string;
  zone_entered: ZoneType;
  recommended_action: string;
  urgency: "low" | "medium" | "high" | "critical";
}
```

### 3.3 Interaction Events Stream

**Endpoint:** `wss://api.wia-standards.org/ai-human-coexistence/v1/stream/interactions`

**Message Format:**
```typescript
interface InteractionEventMessage {
  type: 'interaction_event';
  timestamp: string;
  device_id: string;
  interaction_id: string;
  event_type: 'started' | 'phase_changed' | 'ended' | 'error';
  details: any;
}
```

---

## 4. SDK Examples

### 4.1 TypeScript SDK

```typescript
import { AIHumanCoexistenceClient } from '@wia-standards/ai-human-coexistence';

// Initialize client
const client = new AIHumanCoexistenceClient({
  apiKey: process.env.WIA_API_KEY,
  baseUrl: 'https://api.wia-standards.org/ai-human-coexistence/v1',
  deviceId: 'robot-001'
});

// Perform human detection
async function detectHumans() {
  const detection = await client.detection.scan({
    sensor_data: {
      lidar: await getLidarData(),
      camera: await getCameraData()
    },
    scan_options: {
      enable_tracking: true,
      enable_vulnerability_detection: true,
      confidence_threshold: 0.85
    }
  });

  console.log(`Detected ${detection.humans.length} humans`);
  return detection;
}

// Monitor zones in real-time
async function monitorZones() {
  const stream = await client.zones.streamStatus();

  stream.on('zone_alert', (alert) => {
    console.log(`Zone Alert: ${alert.alert_level}`);

    if (alert.alert_level === 'danger') {
      // Emergency stop
      client.adaptation.apply({
        device_id: 'robot-001',
        adaptation_id: 'emergency-stop',
        adaptations_to_apply: ['emergency_stop']
      });
    }
  });
}

// Start collaborative interaction
async function startCollaboration(humanId: string) {
  const interaction = await client.interaction.start({
    device_id: 'robot-001',
    human_id: humanId,
    interaction_type: 'collaborative_work',
    communication_channels: ['verbal', 'visual', 'projection'],
    collaboration_params: {
      task_id: 'task-001',
      human_role: 'operator',
      robot_role: 'assistant',
      coordination_mode: 'human_led'
    }
  });

  console.log(`Interaction started: ${interaction.interaction_id}`);

  // Monitor interaction
  const updates = await client.interaction.streamUpdates(interaction.interaction_id);

  updates.on('update', (state) => {
    console.log(`Interaction phase: ${state.interaction_phase}`);
    console.log(`Engagement score: ${state.human_state.engagement_score}`);
  });

  return interaction;
}

// Predict human intent
async function predictIntent(humanId: string) {
  const prediction = await client.intent.predict({
    device_id: 'robot-001',
    human_id: humanId,
    observation_window_s: 5.0,
    behavioral_cues: {
      gaze_direction: 'forward',
      movement_pattern: 'steady_walk'
    }
  });

  console.log(`Predicted intent: ${prediction.predicted_intent.primary_intent}`);
  console.log(`Confidence: ${prediction.predicted_intent.confidence}`);

  // Apply recommended actions
  for (const action of prediction.recommended_actions) {
    if (action.priority === 1) {
      await executeAction(action);
    }
  }
}

// Adaptive behavior example
async function adaptiveBehavior() {
  // Get current detections
  const detection = await detectHumans();

  // Evaluate necessary adaptations
  const evaluation = await client.adaptation.evaluate({
    device_id: 'robot-001',
    current_state: await getCurrentState(),
    detected_humans: detection.humans,
    social_context: await getSocialContext()
  });

  // Apply adaptations
  if (evaluation.recommended_adaptations.length > 0) {
    await client.adaptation.apply({
      device_id: 'robot-001',
      adaptation_id: evaluation.adaptation_id,
      adaptations_to_apply: evaluation.priority_order
    });
  }
}
```

### 4.2 Python SDK

```python
from wia_standards import AIHumanCoexistenceClient
import asyncio

# Initialize client
client = AIHumanCoexistenceClient(
    api_key=os.getenv('WIA_API_KEY'),
    base_url='https://api.wia-standards.org/ai-human-coexistence/v1',
    device_id='robot-001'
)

# Detect humans
async def detect_humans():
    detection = await client.detection.scan(
        sensor_data={
            'lidar': await get_lidar_data(),
            'camera': await get_camera_data()
        },
        scan_options={
            'enable_tracking': True,
            'enable_vulnerability_detection': True,
            'confidence_threshold': 0.85
        }
    )

    print(f"Detected {len(detection['humans'])} humans")
    return detection

# Monitor proximity zones
async def monitor_zones():
    async for zone_status in client.zones.stream_status():
        print(f"Zone status: {zone_status['highest_alert_level']}")

        if zone_status['highest_alert_level'] == 'danger':
            # Emergency stop
            await client.adaptation.apply(
                device_id='robot-001',
                adaptation_id='emergency-stop',
                adaptations_to_apply=['emergency_stop']
            )

# Start interaction
async def start_interaction(human_id: str):
    interaction = await client.interaction.start(
        device_id='robot-001',
        human_id=human_id,
        interaction_type='collaborative_work',
        communication_channels=['verbal', 'visual'],
        collaboration_params={
            'task_id': 'task-001',
            'human_role': 'operator',
            'robot_role': 'assistant',
            'coordination_mode': 'human_led'
        }
    )

    print(f"Interaction started: {interaction['interaction_id']}")
    return interaction

# Predict intent and adapt
async def predict_and_adapt(human_id: str):
    # Predict intent
    prediction = await client.intent.predict(
        device_id='robot-001',
        human_id=human_id,
        observation_window_s=5.0,
        behavioral_cues={
            'gaze_direction': 'forward',
            'movement_pattern': 'steady_walk'
        }
    )

    print(f"Predicted: {prediction['predicted_intent']['primary_intent']}")
    print(f"Confidence: {prediction['predicted_intent']['confidence']}")

    # Execute recommended actions
    for action in prediction['recommended_actions']:
        if action['priority'] == 1:
            await execute_action(action)

# Complete workflow
async def main():
    # Detect humans
    detection = await detect_humans()

    # Check zones
    zone_status = await client.zones.get_status()

    # If humans in warning zone, predict intent and adapt
    if zone_status['zone_occupancy']['warning_zone']['count'] > 0:
        for human_id in zone_status['zone_occupancy']['warning_zone']['humans']:
            await predict_and_adapt(human_id)

    # Monitor in real-time
    await monitor_zones()

if __name__ == '__main__':
    asyncio.run(main())
```

---

## 5. Authentication & Authorization

### 5.1 API Key Authentication

```bash
curl -X GET https://api.wia-standards.org/ai-human-coexistence/v1/zones/status \
  -H "Authorization: Bearer YOUR_API_KEY"
```

### 5.2 OAuth 2.0 Flow

```typescript
// Authorization request
const authUrl = 'https://auth.wia-standards.org/oauth/authorize?' +
  'client_id=YOUR_CLIENT_ID&' +
  'redirect_uri=YOUR_REDIRECT_URI&' +
  'response_type=code&' +
  'scope=detection:read zones:write interaction:manage';

// Token exchange
const tokenResponse = await fetch('https://auth.wia-standards.org/oauth/token', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    grant_type: 'authorization_code',
    code: authorizationCode,
    client_id: YOUR_CLIENT_ID,
    client_secret: YOUR_CLIENT_SECRET,
    redirect_uri: YOUR_REDIRECT_URI
  })
});
```

---

## 6. Error Handling

### Error Response Format

```typescript
interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
    request_id: string;
    timestamp: string;
  };
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Invalid request parameters |
| `UNAUTHORIZED` | 401 | Missing or invalid authentication |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `CONFLICT` | 409 | Resource conflict |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Internal server error |
| `DETECTION_FAILED` | 500 | Human detection failed |
| `ZONE_VIOLATION` | 409 | Safety zone violation |

---

## 7. Rate Limiting

### Rate Limits

| Tier | Requests/minute | Burst |
|------|----------------|-------|
| Free | 60 | 100 |
| Standard | 600 | 1000 |
| Premium | 6000 | 10000 |
| Enterprise | Custom | Custom |

### Rate Limit Headers

```
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 599
X-RateLimit-Reset: 1642265400
```

---

## 8. API Versioning

- Current version: `v1`
- Version in URL: `/v1/`
- Header-based versioning: `X-API-Version: 1`

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01
**Status**: Draft

---

<div align="center">

**WIA AI-Human Coexistence API Interface**

Harmonious AI-Human Interaction in Shared Spaces

**弘益人間 (홍익인간)** - Benefit All Humanity

</div>
