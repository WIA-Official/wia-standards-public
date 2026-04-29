# WIA-FRAUD_PREVENTION v1.0
## PHASE 2: API INTERFACE SPECIFICATION

**Status:** FULL Specification
**Version:** 1.0.0
**Last Updated:** 2026-01-12
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

---

## 1. Executive Summary

Phase 2 defines the comprehensive API interface for WIA-FRAUD_PREVENTION systems. This specification provides RESTful APIs, WebSocket streaming, and gRPC interfaces for real-time fraud detection, ML model management, behavioral analysis, and reporting.

### Key Features
- Real-time fraud detection API (<100ms response time)
- WebSocket streaming for live transaction monitoring
- Batch processing for historical analysis
- ML model management and deployment
- Comprehensive reporting and analytics
- Webhook-based alert delivery

---

## 2. API Architecture

### 2.1 Base URL Structure

```
Production:  https://api.fraud-prevention.wia.org/v1
Staging:     https://api-staging.fraud-prevention.wia.org/v1
Development: https://api-dev.fraud-prevention.wia.org/v1
```

### 2.2 API Categories

| Category | Base Path | Purpose |
|----------|-----------|---------|
| Detection | `/detect` | Real-time fraud detection |
| Analysis | `/analyze` | Transaction and behavioral analysis |
| Models | `/models` | ML model management |
| Monitoring | `/monitor` | Real-time monitoring and streaming |
| Alerts | `/alerts` | Alert management and configuration |
| Reports | `/reports` | Analytics and reporting |
| Configuration | `/config` | System configuration |
| Users | `/users` | User and profile management |

### 2.3 Authentication

**Supported Methods:**
- **API Key**: `X-API-Key` header
- **OAuth 2.0**: Bearer token authentication
- **JWT**: JSON Web Token for service-to-service

**Example:**
```http
GET /v1/detect/transaction
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
X-API-Key: wia_fp_1234567890abcdef
```

---

## 3. Real-Time Detection APIs

### 3.1 Analyze Single Transaction

Performs real-time fraud analysis on a single transaction.

**Endpoint:** `POST /v1/detect/transaction`

**Request:**
```json
{
  "transaction": {
    "transactionId": "txn_1234567890",
    "amount": 1250.00,
    "currency": "USD",
    "merchantId": "mch_abc123",
    "merchantCategory": "5411",
    "paymentMethod": "credit_card",
    "cardLast4": "4242",
    "timestamp": "2026-01-12T10:30:00.000Z",
    "location": {
      "country": "US",
      "city": "New York",
      "latitude": 40.7128,
      "longitude": -74.0060,
      "ipAddress": "192.168.1.100"
    }
  },
  "user": {
    "userId": "usr_9876543210",
    "deviceFingerprint": "fp_device_12345",
    "sessionId": "ses_abcdef123456"
  },
  "options": {
    "includeExplanation": true,
    "includeRecommendations": true,
    "detectionMethods": ["ml_model", "behavioral", "anomaly"]
  }
}
```

**Response (200 OK):**
```json
{
  "fraudAnalysis": {
    "analysisId": "ana_1234567890abcdef",
    "timestamp": "2026-01-12T10:30:00.123Z",
    "processingTime": 87,

    "verdict": {
      "isFraudulent": false,
      "confidence": 0.92,
      "riskScore": 23,
      "riskLevel": "low",
      "recommendedAction": "allow"
    },

    "detectionResults": [
      {
        "method": "ml_model",
        "modelId": "model_fraud_v3.2",
        "fraudScore": 0.18,
        "confidence": 0.95
      },
      {
        "method": "behavioral",
        "deviationScore": 0.12,
        "confidence": 0.89
      },
      {
        "method": "anomaly",
        "anomalyScore": 0.25,
        "confidence": 0.87
      }
    ],

    "riskFactors": [
      {
        "factor": "new_merchant_category",
        "severity": "low",
        "contribution": 0.15,
        "description": "First transaction in merchant category 5411"
      },
      {
        "factor": "amount_above_average",
        "severity": "low",
        "contribution": 0.08,
        "description": "Transaction amount 15% above user average"
      }
    ],

    "explanation": {
      "summary": "Transaction appears legitimate based on user's behavioral patterns and ML model analysis.",
      "details": [
        "Transaction location matches user's typical locations",
        "Amount within expected range for this merchant category",
        "Device fingerprint recognized and trusted",
        "Transaction time consistent with user's activity patterns"
      ]
    },

    "recommendations": {
      "action": "allow",
      "additionalChecks": [],
      "monitoringLevel": "standard"
    }
  }
}
```

**Response (200 OK - Fraudulent):**
```json
{
  "fraudAnalysis": {
    "analysisId": "ana_fraud_9876543210",
    "timestamp": "2026-01-12T11:45:00.456Z",
    "processingTime": 92,

    "verdict": {
      "isFraudulent": true,
      "confidence": 0.97,
      "riskScore": 89,
      "riskLevel": "critical",
      "recommendedAction": "block"
    },

    "detectionResults": [
      {
        "method": "ml_model",
        "modelId": "model_fraud_v3.2",
        "fraudScore": 0.94,
        "confidence": 0.98
      },
      {
        "method": "behavioral",
        "deviationScore": 0.86,
        "confidence": 0.95
      },
      {
        "method": "anomaly",
        "anomalyScore": 0.91,
        "confidence": 0.93
      }
    ],

    "riskFactors": [
      {
        "factor": "impossible_location_velocity",
        "severity": "critical",
        "contribution": 0.35,
        "description": "Transaction location 3000km from previous transaction 30 minutes ago (requires 6000 km/h velocity)"
      },
      {
        "factor": "unusual_amount",
        "severity": "high",
        "contribution": 0.25,
        "description": "Transaction amount 450% above user's average"
      },
      {
        "factor": "new_device",
        "severity": "high",
        "contribution": 0.20,
        "description": "Unrecognized device fingerprint"
      },
      {
        "factor": "high_risk_merchant",
        "severity": "medium",
        "contribution": 0.09,
        "description": "Merchant has elevated fraud risk score"
      }
    ],

    "explanation": {
      "summary": "Transaction exhibits multiple high-risk indicators suggesting fraudulent activity.",
      "details": [
        "Physically impossible travel velocity detected",
        "Unrecognized device attempting transaction",
        "Amount significantly exceeds user's typical spending",
        "Merchant category never used by this user before",
        "Transaction at unusual time for this user (3:00 AM local time)"
      ]
    },

    "recommendations": {
      "action": "block",
      "additionalChecks": [
        "verify_user_identity",
        "check_recent_account_changes",
        "review_recent_transactions"
      ],
      "monitoringLevel": "enhanced",
      "alertPriority": "critical"
    }
  }
}
```

**Error Responses:**
```json
// 400 Bad Request
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Transaction amount must be a positive number",
    "field": "transaction.amount"
  }
}

// 401 Unauthorized
{
  "error": {
    "code": "UNAUTHORIZED",
    "message": "Invalid or missing API key"
  }
}

// 429 Too Many Requests
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded: 1000 requests per minute",
    "retryAfter": 45
  }
}
```

### 3.2 Batch Transaction Analysis

Analyze multiple transactions in a single request.

**Endpoint:** `POST /v1/detect/batch`

**Request:**
```json
{
  "transactions": [
    {
      "transactionId": "txn_001",
      "amount": 50.00,
      "currency": "USD",
      "userId": "usr_123",
      // ... other fields
    },
    {
      "transactionId": "txn_002",
      "amount": 125.00,
      "currency": "EUR",
      "userId": "usr_456",
      // ... other fields
    }
  ],
  "options": {
    "asyncProcessing": false,
    "includeExplanation": false
  }
}
```

**Response (200 OK):**
```json
{
  "batchAnalysis": {
    "batchId": "batch_1234567890",
    "processedAt": "2026-01-12T12:00:00.000Z",
    "totalTransactions": 2,
    "fraudulentCount": 0,
    "processingTime": 145,

    "results": [
      {
        "transactionId": "txn_001",
        "isFraudulent": false,
        "riskScore": 15,
        "confidence": 0.94
      },
      {
        "transactionId": "txn_002",
        "isFraudulent": false,
        "riskScore": 22,
        "confidence": 0.91
      }
    ]
  }
}
```

---

## 4. Behavioral Analysis APIs

### 4.1 Get User Behavioral Profile

**Endpoint:** `GET /v1/analyze/user/{userId}/profile`

**Response (200 OK):**
```json
{
  "behavioralProfile": {
    "userId": "usr_9876543210",
    "profileVersion": 42,
    "lastUpdated": "2026-01-12T10:00:00.000Z",
    "dataPoints": 1547,

    "transactionPatterns": {
      "averageAmount": 87.50,
      "medianAmount": 45.00,
      "maxAmount": 2500.00,
      "frequencyPerDay": 2.3,
      "preferredTimeWindows": [
        {"start": "09:00", "end": "12:00", "frequency": 0.35},
        {"start": "18:00", "end": "21:00", "frequency": 0.45}
      ]
    },

    "locationPatterns": {
      "primaryCountries": ["US", "CA"],
      "primaryCities": ["New York", "Boston", "Toronto"],
      "averageDistanceBetweenTransactions": 15.2,
      "internationalRate": 0.05
    },

    "merchantPatterns": {
      "topCategories": [
        {"mcc": "5411", "name": "Grocery Stores", "frequency": 0.32},
        {"mcc": "5812", "name": "Restaurants", "frequency": 0.28},
        {"mcc": "5541", "name": "Gas Stations", "frequency": 0.18}
      ],
      "merchantCount": 87,
      "merchantDiversity": 0.72
    },

    "devicePatterns": {
      "trustedDevices": 3,
      "primaryDeviceFingerprint": "fp_device_12345",
      "deviceChangeFrequency": 0.02
    },

    "riskAssessment": {
      "overallRiskScore": 18,
      "riskLevel": "low",
      "accountAge": 847,
      "previousFraudIncidents": 0,
      "verificationLevel": "verified"
    }
  }
}
```

### 4.2 Update Behavioral Profile

**Endpoint:** `POST /v1/analyze/user/{userId}/profile/update`

**Request:**
```json
{
  "newTransaction": {
    "transactionId": "txn_new_001",
    "amount": 65.00,
    "merchantCategory": "5411",
    "location": {"country": "US", "city": "New York"},
    "timestamp": "2026-01-12T14:30:00.000Z"
  },
  "updateMode": "incremental"
}
```

**Response (200 OK):**
```json
{
  "updateResult": {
    "success": true,
    "profileVersion": 43,
    "updatedAt": "2026-01-12T14:30:05.000Z",
    "changedMetrics": [
      {
        "metric": "averageAmount",
        "oldValue": 87.50,
        "newValue": 87.35,
        "percentChange": -0.17
      }
    ]
  }
}
```

### 4.3 Detect Behavioral Anomalies

**Endpoint:** `POST /v1/analyze/anomaly`

**Request:**
```json
{
  "userId": "usr_9876543210",
  "currentTransaction": {
    "amount": 2500.00,
    "merchantCategory": "5999",
    "location": {"country": "RU", "city": "Moscow"},
    "timestamp": "2026-01-12T03:00:00.000Z",
    "deviceFingerprint": "fp_device_unknown"
  },
  "options": {
    "algorithm": "isolation_forest",
    "sensitivity": "high"
  }
}
```

**Response (200 OK):**
```json
{
  "anomalyAnalysis": {
    "detectionId": "anom_1234567890",
    "timestamp": "2026-01-12T15:00:00.000Z",
    "isAnomaly": true,
    "anomalyScore": 0.89,
    "confidence": 0.94,

    "deviations": [
      {
        "feature": "transaction_amount",
        "expected": 87.50,
        "actual": 2500.00,
        "deviationScore": 0.95,
        "severity": "critical"
      },
      {
        "feature": "merchant_category",
        "expected": "5411,5812,5541",
        "actual": "5999",
        "deviationScore": 0.78,
        "severity": "high"
      },
      {
        "feature": "transaction_country",
        "expected": "US,CA",
        "actual": "RU",
        "deviationScore": 0.92,
        "severity": "critical"
      },
      {
        "feature": "transaction_time",
        "expected": "09:00-21:00",
        "actual": "03:00",
        "deviationScore": 0.65,
        "severity": "medium"
      }
    ],

    "explanation": {
      "summary": "Multiple significant deviations from user's established behavioral patterns detected",
      "topContributors": [
        "Unusual transaction amount (28x average)",
        "Unknown merchant category",
        "Unexpected country (no prior transactions in Russia)",
        "Unusual time of day for this user"
      ]
    }
  }
}
```

---

## 5. ML Model Management APIs

### 5.1 List Available Models

**Endpoint:** `GET /v1/models`

**Response (200 OK):**
```json
{
  "models": [
    {
      "modelId": "model_fraud_v3.2",
      "name": "Fraud Detection ML Model v3.2",
      "type": "gradient_boosting",
      "version": "3.2.0",
      "status": "active",
      "accuracy": 0.962,
      "precision": 0.948,
      "recall": 0.971,
      "f1Score": 0.959,
      "deployedAt": "2026-01-01T00:00:00.000Z",
      "trainingDataSize": 10000000,
      "features": ["amount", "location", "time", "merchant", "device", "velocity"]
    },
    {
      "modelId": "model_anomaly_v2.1",
      "name": "Anomaly Detection Model v2.1",
      "type": "isolation_forest",
      "version": "2.1.0",
      "status": "active",
      "accuracy": 0.934,
      "deployedAt": "2025-12-15T00:00:00.000Z"
    }
  ],
  "totalModels": 2
}
```

### 5.2 Get Model Details

**Endpoint:** `GET /v1/models/{modelId}`

**Response (200 OK):**
```json
{
  "model": {
    "modelId": "model_fraud_v3.2",
    "name": "Fraud Detection ML Model v3.2",
    "description": "Advanced gradient boosting model trained on 10M transactions",
    "type": "gradient_boosting",
    "version": "3.2.0",
    "status": "active",

    "performance": {
      "accuracy": 0.962,
      "precision": 0.948,
      "recall": 0.971,
      "f1Score": 0.959,
      "auc": 0.987,
      "falsePositiveRate": 0.052,
      "falseNegativeRate": 0.029
    },

    "training": {
      "trainingDataSize": 10000000,
      "fraudulentSamples": 150000,
      "legitimateSamples": 9850000,
      "trainedAt": "2025-12-28T00:00:00.000Z",
      "trainingDuration": 14400,
      "validationSplit": 0.2,
      "testSplit": 0.1
    },

    "features": [
      {"name": "transaction_amount", "importance": 0.28},
      {"name": "location_velocity", "importance": 0.22},
      {"name": "device_reputation", "importance": 0.18},
      {"name": "merchant_risk", "importance": 0.15},
      {"name": "time_deviation", "importance": 0.10},
      {"name": "amount_deviation", "importance": 0.07}
    ],

    "deployment": {
      "deployedAt": "2026-01-01T00:00:00.000Z",
      "endpoint": "https://ml.fraud-prevention.wia.org/v1/model_fraud_v3.2/predict",
      "averageLatency": 45,
      "throughput": 15000
    },

    "metadata": {
      "framework": "XGBoost",
      "language": "Python 3.11",
      "dependencies": ["xgboost==2.0.3", "scikit-learn==1.4.0"],
      "hardware": "NVIDIA A100 GPU"
    }
  }
}
```

### 5.3 Deploy New Model

**Endpoint:** `POST /v1/models/deploy`

**Request:**
```json
{
  "modelId": "model_fraud_v3.3",
  "modelSource": "s3://wia-ml-models/fraud_detection_v3.3.pkl",
  "deploymentMode": "canary",
  "canaryPercentage": 10,
  "autoRollback": true,
  "rollbackThreshold": {
    "accuracyDrop": 0.05,
    "latencyIncrease": 100
  }
}
```

**Response (202 Accepted):**
```json
{
  "deployment": {
    "deploymentId": "dep_1234567890",
    "modelId": "model_fraud_v3.3",
    "status": "deploying",
    "deploymentMode": "canary",
    "canaryPercentage": 10,
    "startedAt": "2026-01-12T16:00:00.000Z",
    "estimatedCompletion": "2026-01-12T16:15:00.000Z",
    "monitoringUrl": "https://monitor.fraud-prevention.wia.org/deployments/dep_1234567890"
  }
}
```

### 5.4 Train Model

**Endpoint:** `POST /v1/models/train`

**Request:**
```json
{
  "modelName": "Fraud Detection Model v3.4",
  "modelType": "neural_network",
  "trainingDataset": "dataset_2026_q1",
  "hyperparameters": {
    "learningRate": 0.001,
    "batchSize": 256,
    "epochs": 100,
    "layers": [512, 256, 128, 64],
    "dropout": 0.3
  },
  "features": [
    "transaction_amount",
    "location_velocity",
    "device_reputation",
    "merchant_risk",
    "time_deviation",
    "amount_deviation",
    "frequency_pattern"
  ],
  "validationSplit": 0.2,
  "testSplit": 0.1
}
```

**Response (202 Accepted):**
```json
{
  "trainingJob": {
    "jobId": "train_job_1234567890",
    "modelName": "Fraud Detection Model v3.4",
    "status": "queued",
    "queuePosition": 3,
    "estimatedStartTime": "2026-01-12T18:00:00.000Z",
    "estimatedDuration": 18000,
    "monitoringUrl": "https://monitor.fraud-prevention.wia.org/training/train_job_1234567890"
  }
}
```

---

## 6. Real-Time Monitoring APIs

### 6.1 WebSocket Connection

**Endpoint:** `ws://stream.fraud-prevention.wia.org/v1/monitor/stream`

**Connection:**
```javascript
const ws = new WebSocket('ws://stream.fraud-prevention.wia.org/v1/monitor/stream');

ws.on('open', () => {
  // Subscribe to fraud events
  ws.send(JSON.stringify({
    action: 'subscribe',
    channels: ['fraud_events', 'high_risk_transactions'],
    filters: {
      minRiskScore: 70,
      severity: ['high', 'critical']
    }
  }));
});

ws.on('message', (data) => {
  const event = JSON.parse(data);
  console.log('Received fraud event:', event);
});
```

**Stream Event Format:**
```json
{
  "eventType": "fraud_detected",
  "timestamp": "2026-01-12T17:30:00.123Z",
  "channel": "fraud_events",

  "data": {
    "eventId": "evt_9876543210",
    "transactionId": "txn_fraud_001",
    "userId": "usr_123456",
    "riskScore": 94,
    "severity": "critical",
    "confidence": 0.97,

    "transaction": {
      "amount": 5000.00,
      "currency": "USD",
      "merchantName": "Suspicious Merchant LLC"
    },

    "indicators": [
      "impossible_velocity",
      "new_device",
      "unusual_amount"
    ]
  }
}
```

### 6.2 Get Real-Time Metrics

**Endpoint:** `GET /v1/monitor/metrics`

**Query Parameters:**
- `timeRange`: `1h`, `6h`, `24h`, `7d` (default: `1h`)
- `metrics`: Comma-separated list of metrics

**Response (200 OK):**
```json
{
  "metrics": {
    "timeRange": "1h",
    "collectedAt": "2026-01-12T18:00:00.000Z",

    "transactionMetrics": {
      "totalTransactions": 125847,
      "fraudulentTransactions": 342,
      "fraudRate": 0.0027,
      "totalAmount": 8547230.50,
      "fraudulentAmount": 145890.00,
      "averageTransactionAmount": 67.95
    },

    "detectionMetrics": {
      "averageResponseTime": 76,
      "p50ResponseTime": 65,
      "p95ResponseTime": 142,
      "p99ResponseTime": 287,
      "throughput": 34.95,
      "detectionAccuracy": 0.961
    },

    "modelPerformance": {
      "activeModels": 2,
      "modelAccuracy": 0.962,
      "falsePositiveRate": 0.048,
      "falseNegativeRate": 0.032
    },

    "alertMetrics": {
      "alertsSent": 78,
      "criticalAlerts": 12,
      "highAlerts": 28,
      "mediumAlerts": 38,
      "averageResolutionTime": 420
    }
  }
}
```

### 6.3 Start Monitoring Session

**Endpoint:** `POST /v1/monitor/session`

**Request:**
```json
{
  "userId": "usr_9876543210",
  "monitoringLevel": "enhanced",
  "duration": 3600,
  "alertThreshold": {
    "riskScore": 60,
    "confidence": 0.85
  }
}
```

**Response (201 Created):**
```json
{
  "session": {
    "sessionId": "mon_ses_1234567890",
    "userId": "usr_9876543210",
    "startedAt": "2026-01-12T18:30:00.000Z",
    "expiresAt": "2026-01-12T19:30:00.000Z",
    "monitoringLevel": "enhanced",
    "status": "active"
  }
}
```

---

## 7. Alert Management APIs

### 7.1 List Alerts

**Endpoint:** `GET /v1/alerts`

**Query Parameters:**
- `priority`: `low`, `medium`, `high`, `critical`
- `status`: `active`, `acknowledged`, `resolved`
- `startDate`: ISO 8601 timestamp
- `endDate`: ISO 8601 timestamp
- `limit`: Integer (default: 50, max: 500)
- `offset`: Integer (default: 0)

**Response (200 OK):**
```json
{
  "alerts": [
    {
      "alertId": "alert_1234567890",
      "triggeredAt": "2026-01-12T19:00:00.000Z",
      "priority": "critical",
      "status": "active",
      "category": "account_takeover",
      "subject": "Potential Account Takeover Detected",
      "userId": "usr_9876543210",
      "riskScore": 91,
      "relatedTransactions": ["txn_001", "txn_002", "txn_003"],
      "requiresAction": true,
      "actionDeadline": "2026-01-12T20:00:00.000Z"
    }
  ],
  "totalAlerts": 1,
  "hasMore": false
}
```

### 7.2 Get Alert Details

**Endpoint:** `GET /v1/alerts/{alertId}`

**Response (200 OK):**
```json
{
  "alert": {
    "alertId": "alert_1234567890",
    "triggeredAt": "2026-01-12T19:00:00.000Z",
    "priority": "critical",
    "status": "active",
    "category": "account_takeover",

    "subject": "Potential Account Takeover Detected",
    "message": "Multiple high-risk transactions detected from unrecognized device in unusual location",

    "details": {
      "userId": "usr_9876543210",
      "riskScore": 91,
      "confidence": 0.96,

      "relatedEvents": [
        {
          "eventId": "evt_001",
          "eventType": "new_device_login",
          "timestamp": "2026-01-12T18:55:00.000Z",
          "severity": "high"
        },
        {
          "eventId": "evt_002",
          "eventType": "suspicious_transaction",
          "timestamp": "2026-01-12T18:57:00.000Z",
          "severity": "critical"
        },
        {
          "eventId": "evt_003",
          "eventType": "rapid_transactions",
          "timestamp": "2026-01-12T19:00:00.000Z",
          "severity": "high"
        }
      ],

      "indicators": [
        "Unrecognized device fingerprint",
        "Location: Moscow, Russia (user typically in US)",
        "3 transactions in 5 minutes",
        "Total amount: $4,500 (300% above average)"
      ]
    },

    "recommendations": {
      "immediateAction": "Block account and require identity verification",
      "additionalChecks": [
        "Contact user via verified phone number",
        "Review recent password/email changes",
        "Check for unauthorized access attempts"
      ]
    },

    "notifications": [
      {
        "recipientType": "user",
        "channel": "sms",
        "status": "delivered",
        "sentAt": "2026-01-12T19:00:30.000Z"
      },
      {
        "recipientType": "fraud_analyst",
        "channel": "email",
        "status": "delivered",
        "sentAt": "2026-01-12T19:00:15.000Z"
      }
    ]
  }
}
```

### 7.3 Acknowledge Alert

**Endpoint:** `POST /v1/alerts/{alertId}/acknowledge`

**Request:**
```json
{
  "acknowledgedBy": "analyst_john_doe",
  "notes": "Investigating suspected account takeover. User contacted via phone."
}
```

**Response (200 OK):**
```json
{
  "alert": {
    "alertId": "alert_1234567890",
    "status": "acknowledged",
    "acknowledgedAt": "2026-01-12T19:05:00.000Z",
    "acknowledgedBy": "analyst_john_doe"
  }
}
```

### 7.4 Resolve Alert

**Endpoint:** `POST /v1/alerts/{alertId}/resolve`

**Request:**
```json
{
  "resolution": "confirmed_fraud",
  "resolvedBy": "analyst_john_doe",
  "notes": "User confirmed unauthorized transactions. Account secured, cards cancelled.",
  "actions": [
    "account_locked",
    "cards_cancelled",
    "user_verified",
    "refund_initiated"
  ]
}
```

**Response (200 OK):**
```json
{
  "alert": {
    "alertId": "alert_1234567890",
    "status": "resolved",
    "resolution": "confirmed_fraud",
    "resolvedAt": "2026-01-12T19:45:00.000Z",
    "resolvedBy": "analyst_john_doe",
    "resolutionTime": 2700
  }
}
```

### 7.5 Configure Alert Rules

**Endpoint:** `POST /v1/alerts/rules`

**Request:**
```json
{
  "ruleName": "High Value Transaction Alert",
  "description": "Alert on transactions above $1000",
  "enabled": true,
  "priority": "high",

  "conditions": {
    "operator": "AND",
    "rules": [
      {
        "field": "transaction.amount",
        "operator": ">",
        "value": 1000
      },
      {
        "field": "user.riskScore",
        "operator": ">",
        "value": 50
      }
    ]
  },

  "actions": {
    "alert": true,
    "block": false,
    "challenge": true,
    "notify": ["fraud_team", "user"]
  },

  "notificationChannels": {
    "fraud_team": ["email", "slack"],
    "user": ["sms", "push"]
  }
}
```

**Response (201 Created):**
```json
{
  "rule": {
    "ruleId": "rule_1234567890",
    "ruleName": "High Value Transaction Alert",
    "status": "active",
    "createdAt": "2026-01-12T20:00:00.000Z",
    "createdBy": "admin_user"
  }
}
```

---

## 8. Reporting APIs

### 8.1 Generate Fraud Report

**Endpoint:** `POST /v1/reports/fraud`

**Request:**
```json
{
  "reportType": "fraud_summary",
  "period": {
    "start": "2026-01-01T00:00:00.000Z",
    "end": "2026-01-12T23:59:59.999Z"
  },
  "includeCharts": true,
  "includeRecommendations": true,
  "format": "pdf"
}
```

**Response (202 Accepted):**
```json
{
  "report": {
    "reportId": "rpt_1234567890",
    "reportType": "fraud_summary",
    "status": "generating",
    "requestedAt": "2026-01-12T20:30:00.000Z",
    "estimatedCompletion": "2026-01-12T20:35:00.000Z",
    "statusUrl": "https://api.fraud-prevention.wia.org/v1/reports/rpt_1234567890/status",
    "downloadUrl": "https://api.fraud-prevention.wia.org/v1/reports/rpt_1234567890/download"
  }
}
```

### 8.2 Get Report Status

**Endpoint:** `GET /v1/reports/{reportId}/status`

**Response (200 OK):**
```json
{
  "report": {
    "reportId": "rpt_1234567890",
    "status": "completed",
    "generatedAt": "2026-01-12T20:34:23.000Z",
    "fileSize": 2457600,
    "format": "pdf",
    "expiresAt": "2026-01-19T20:34:23.000Z",
    "downloadUrl": "https://api.fraud-prevention.wia.org/v1/reports/rpt_1234567890/download"
  }
}
```

### 8.3 Get Analytics Dashboard

**Endpoint:** `GET /v1/reports/dashboard`

**Query Parameters:**
- `timeRange`: `today`, `week`, `month`, `quarter`, `year`

**Response (200 OK):**
```json
{
  "dashboard": {
    "period": {
      "start": "2026-01-01T00:00:00.000Z",
      "end": "2026-01-12T23:59:59.999Z"
    },
    "generatedAt": "2026-01-12T21:00:00.000Z",

    "summary": {
      "totalTransactions": 1547892,
      "fraudulentTransactions": 4187,
      "fraudRate": 0.0027,
      "preventedLoss": 2847500.00,
      "falsePositives": 1024,
      "detectionAccuracy": 0.961
    },

    "trends": {
      "fraudRateTrend": "decreasing",
      "comparisonToPreviousPeriod": {
        "transactionVolume": "+12.5%",
        "fraudRate": "-8.3%",
        "detectionAccuracy": "+2.1%",
        "responseTime": "-15.7%"
      }
    },

    "topFraudTypes": [
      {
        "type": "card_fraud",
        "count": 1847,
        "percentage": 44.1,
        "totalLoss": 987450.00
      },
      {
        "type": "account_takeover",
        "count": 982,
        "percentage": 23.5,
        "totalLoss": 1245800.00
      },
      {
        "type": "identity_theft",
        "count": 754,
        "percentage": 18.0,
        "totalLoss": 456780.00
      }
    ],

    "modelPerformance": {
      "primaryModel": {
        "modelId": "model_fraud_v3.2",
        "accuracy": 0.962,
        "precision": 0.948,
        "recall": 0.971,
        "f1Score": 0.959
      }
    },

    "geographicDistribution": [
      {"country": "US", "fraudCount": 2547, "fraudRate": 0.0025},
      {"country": "GB", "fraudCount": 654, "fraudRate": 0.0031},
      {"country": "CA", "fraudCount": 487, "fraudRate": 0.0022}
    ]
  }
}
```

---

## 9. Configuration APIs

### 9.1 Get System Configuration

**Endpoint:** `GET /v1/config`

**Response (200 OK):**
```json
{
  "configuration": {
    "version": "1.0.0",
    "lastModified": "2026-01-10T15:00:00.000Z",

    "detection": {
      "realTimeEnabled": true,
      "behavioralAnalysisEnabled": true,
      "anomalyDetectionEnabled": true,
      "mlModelsEnabled": true,

      "thresholds": {
        "fraudScoreBlock": 0.90,
        "fraudScoreChallenge": 0.70,
        "fraudScoreReview": 0.50,
        "anomalyThreshold": 0.75
      },

      "velocityLimits": {
        "transactionsPerHour": 20,
        "amountPerHour": 5000.00,
        "transactionsPerDay": 50,
        "amountPerDay": 10000.00
      }
    },

    "alerts": {
      "enabled": true,
      "channels": ["email", "sms", "push", "webhook"],
      "batchingEnabled": true,
      "batchInterval": 300
    },

    "models": {
      "primaryModelId": "model_fraud_v3.2",
      "autoRetraining": true,
      "retrainingSchedule": "0 2 * * 0",
      "minAccuracy": 0.95
    }
  }
}
```

### 9.2 Update Configuration

**Endpoint:** `PATCH /v1/config`

**Request:**
```json
{
  "detection": {
    "thresholds": {
      "fraudScoreBlock": 0.85,
      "fraudScoreChallenge": 0.65
    }
  },
  "updatedBy": "admin_user"
}
```

**Response (200 OK):**
```json
{
  "configuration": {
    "version": "1.0.1",
    "lastModified": "2026-01-12T21:30:00.000Z",
    "updatedBy": "admin_user",
    "changes": [
      {
        "field": "detection.thresholds.fraudScoreBlock",
        "oldValue": 0.90,
        "newValue": 0.85
      },
      {
        "field": "detection.thresholds.fraudScoreChallenge",
        "oldValue": 0.70,
        "newValue": 0.65
      }
    ]
  }
}
```

---

## 10. Rate Limits

| Endpoint Category | Rate Limit | Burst Limit |
|------------------|------------|-------------|
| Real-Time Detection | 1,000 req/min | 100 req/sec |
| Batch Analysis | 100 req/min | 10 req/sec |
| Behavioral Analysis | 500 req/min | 50 req/sec |
| Model Management | 50 req/min | 10 req/sec |
| Monitoring | 200 req/min | 20 req/sec |
| Alerts | 300 req/min | 30 req/sec |
| Reports | 50 req/min | 10 req/sec |
| Configuration | 20 req/min | 5 req/sec |

**Rate Limit Headers:**
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1736705400
```

---

## 11. Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_REQUEST | 400 | Request validation failed |
| UNAUTHORIZED | 401 | Authentication failed |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| CONFLICT | 409 | Resource conflict |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |
| SERVICE_UNAVAILABLE | 503 | Service temporarily unavailable |
| MODEL_ERROR | 500 | ML model prediction failed |
| DATA_ERROR | 400 | Invalid data format |

---

## 12. Webhooks

### 12.1 Configure Webhook

**Endpoint:** `POST /v1/webhooks`

**Request:**
```json
{
  "url": "https://your-domain.com/fraud-webhook",
  "events": ["fraud_detected", "alert_created", "high_risk_transaction"],
  "secret": "whsec_your_secret_key",
  "enabled": true
}
```

**Response (201 Created):**
```json
{
  "webhook": {
    "webhookId": "wh_1234567890",
    "url": "https://your-domain.com/fraud-webhook",
    "events": ["fraud_detected", "alert_created", "high_risk_transaction"],
    "status": "active",
    "createdAt": "2026-01-12T22:00:00.000Z"
  }
}
```

### 12.2 Webhook Payload

```json
{
  "id": "evt_webhook_1234567890",
  "type": "fraud_detected",
  "timestamp": "2026-01-12T22:05:00.000Z",

  "data": {
    "eventId": "evt_fraud_001",
    "transactionId": "txn_suspicious_123",
    "userId": "usr_9876543210",
    "riskScore": 92,
    "confidence": 0.96,
    "recommendedAction": "block"
  }
}
```

**Webhook Signature:**
```http
X-WIA-Signature: t=1736710500,v1=5257a869e7ecebeda32affa62cdca3fa51cad7e77a0e56ff536d0ce8e108d8bd
```

---

## 13. Implementation Checklist

- [ ] Implement authentication (API Key, OAuth 2.0, JWT)
- [ ] Create all detection endpoints (single, batch)
- [ ] Implement behavioral analysis APIs
- [ ] Create ML model management endpoints
- [ ] Set up WebSocket streaming for real-time monitoring
- [ ] Implement alert management system
- [ ] Create reporting and analytics endpoints
- [ ] Configure webhook support
- [ ] Implement rate limiting
- [ ] Add comprehensive error handling
- [ ] Create API documentation (OpenAPI/Swagger)
- [ ] Set up monitoring and logging
- [ ] Implement health check endpoints
- [ ] Test all API endpoints
- [ ] Performance test under load (10,000+ TPS)

---

## 14. OpenAPI Specification

Available at: `https://api.fraud-prevention.wia.org/v1/openapi.json`

---

**Document Classification:** Public Standard
**License:** Open Standard (Implementable without royalties)

© 2026 WIA (World Industry Association)
弘익人間 (홍익인간) - Benefit All Humanity
