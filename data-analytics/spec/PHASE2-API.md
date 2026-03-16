# WIA-DATA-012: Data Analytics Standard
## PHASE 2 - API SPECIFICATION

### Version: 1.0
### Status: Active
### Last Updated: 2025-12-26

---

## 1. Overview

This document defines the RESTful API specification for WIA-DATA-012 Data Analytics Standard. These APIs enable standardized access to analytics capabilities across different platforms and tools.

**Philosophy**: 弘익人간 (홍익인간) - Benefit All Humanity through accessible, standardized analytics APIs.

## 2. API Design Principles

### 2.1 Core Principles
- **RESTful**: Resource-oriented design
- **Versioned**: API versioning in URL path
- **Stateless**: Each request contains all necessary information
- **HTTPS Only**: Secure communication required
- **JSON**: Request/response in JSON format
- **Pagination**: Support for large result sets
- **Rate Limited**: Protection against abuse

### 2.2 Base URL
```
https://api.wia.org/v1/analytics
```

## 3. Authentication

### 3.1 API Key Authentication
```http
GET /v1/analytics/datasets
Authorization: Bearer YOUR_API_KEY
X-WIA-Client-ID: your_client_id
Content-Type: application/json
```

### 3.2 OAuth 2.0
```http
POST /v1/auth/oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "analytics:read analytics:write"
}
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "analytics:read analytics:write"
}
```

## 4. Descriptive Analytics API

### 4.1 POST /v1/analytics/descriptive
Perform descriptive analytics on a dataset.

**Request**:
```json
{
  "dataset_id": "sales_2025",
  "metrics": ["mean", "median", "std", "quartiles", "count"],
  "group_by": ["category", "region"],
  "filters": {
    "date_range": {
      "start": "2025-01-01",
      "end": "2025-12-26"
    },
    "category": ["electronics", "books"]
  },
  "aggregations": [
    {
      "field": "revenue",
      "operations": ["sum", "avg", "max", "min"]
    }
  ]
}
```

**Response**:
```json
{
  "status": "success",
  "request_id": "REQ-2025-12-26-001",
  "execution_time_ms": 234,
  "results": {
    "summary": {
      "total_records": 15847,
      "filtered_records": 8923,
      "date_range": {
        "start": "2025-01-01T00:00:00Z",
        "end": "2025-12-26T23:59:59Z"
      }
    },
    "statistics": {
      "mean": 151.52,
      "median": 145.00,
      "std": 42.18,
      "quartiles": {
        "q1": 120.00,
        "q2": 145.00,
        "q3": 180.00
      },
      "count": 8923
    },
    "grouped_results": [
      {
        "category": "electronics",
        "region": "US-West",
        "count": 2341,
        "revenue_sum": 524000,
        "revenue_avg": 223.78
      }
    ]
  }
}
```

## 5. Predictive Analytics API

### 5.1 POST /v1/analytics/predictive
Generate predictions using machine learning models.

**Request**:
```json
{
  "model_config": {
    "model_type": "prophet",
    "auto_select": false
  },
  "training_data": {
    "dataset_id": "sales_history",
    "features": ["date", "revenue", "marketing_spend"],
    "target": "revenue",
    "date_column": "date"
  },
  "prediction_config": {
    "horizon": 30,
    "interval": "day",
    "confidence_level": 0.95,
    "include_history": true
  },
  "parameters": {
    "yearly_seasonality": true,
    "weekly_seasonality": true
  }
}
```

**Response**:
```json
{
  "status": "success",
  "model_id": "MODEL-FORECAST-2025-001",
  "training_metrics": {
    "r2_score": 0.89,
    "rmse": 12400,
    "mae": 9800,
    "mape": 6.5
  },
  "predictions": [
    {
      "date": "2025-12-27",
      "predicted_value": 24500,
      "confidence_interval": {
        "lower": 23300,
        "upper": 25700
      }
    },
    {
      "date": "2025-12-28",
      "predicted_value": 25100,
      "confidence_interval": {
        "lower": 23800,
        "upper": 26400
      }
    }
  ],
  "model_artifacts": {
    "download_url": "https://api.wia.org/v1/models/MODEL-FORECAST-2025-001/download",
    "expires_at": "2025-12-28T00:00:00Z"
  }
}
```

## 6. Prescriptive Analytics API

### 6.1 POST /v1/analytics/prescriptive
Get actionable recommendations based on data and objectives.

**Request**:
```json
{
  "objective": {
    "type": "maximize",
    "metric": "revenue"
  },
  "decision_variables": {
    "price": {
      "min": 10,
      "max": 100,
      "current": 50
    },
    "marketing_budget": {
      "min": 0,
      "max": 100000,
      "current": 50000
    }
  },
  "constraints": [
    {
      "type": "budget",
      "value": 150000,
      "operator": "less_than_or_equal"
    },
    {
      "type": "profit_margin",
      "value": 0.20,
      "operator": "greater_than_or_equal"
    }
  ],
  "historical_data": {
    "dataset_id": "sales_with_marketing"
  },
  "simulation": {
    "method": "monte_carlo",
    "iterations": 10000
  }
}
```

**Response**:
```json
{
  "status": "success",
  "recommendations": [
    {
      "rank": 1,
      "confidence": 0.85,
      "variables": {
        "price": 75.50,
        "marketing_budget": 85000
      },
      "expected_outcomes": {
        "revenue": 2450000,
        "profit": 490000,
        "roi": 5.76
      },
      "sensitivity": {
        "price": 0.42,
        "marketing_budget": 0.38
      }
    }
  ],
  "what_if_scenarios": [
    {
      "scenario": "conservative",
      "variables": {"price": 65, "marketing_budget": 60000},
      "expected_revenue": 2100000
    },
    {
      "scenario": "aggressive",
      "variables": {"price": 85, "marketing_budget": 100000},
      "expected_revenue": 2600000
    }
  ]
}
```

## 7. Segmentation API

### 7.1 POST /v1/analytics/segmentation
Perform customer or data segmentation.

**Request**:
```json
{
  "dataset_id": "customer_data",
  "algorithm": "kmeans",
  "config": {
    "n_clusters": 4,
    "normalize": true,
    "random_state": 42
  },
  "features": [
    "purchase_frequency",
    "average_spend",
    "recency_days",
    "lifetime_value"
  ],
  "output": {
    "include_profiles": true,
    "include_recommendations": true
  }
}
```

**Response**:
```json
{
  "status": "success",
  "segmentation_id": "SEG-2025-12-26-001",
  "quality_metrics": {
    "silhouette_score": 0.72,
    "inertia": 15234.56,
    "calinski_harabasz_score": 342.15
  },
  "segments": [
    {
      "segment_id": 0,
      "name": "High Value",
      "size": 1234,
      "percentage": 12.3,
      "profile": {
        "purchase_frequency": 12.5,
        "average_spend": 450.00,
        "recency_days": 7.2,
        "lifetime_value": 5400.00
      },
      "characteristics": [
        "Highly engaged",
        "Premium products",
        "Frequent buyers"
      ],
      "recommendations": [
        "VIP program enrollment",
        "Exclusive product previews",
        "Personalized concierge service"
      ]
    }
  ]
}
```

## 8. Real-time Analytics API

### 8.1 POST /v1/analytics/streams
Create a real-time analytics stream.

**Request**:
```json
{
  "stream_name": "sales_events",
  "source": {
    "type": "kafka",
    "config": {
      "brokers": ["kafka1:9092", "kafka2:9092"],
      "topic": "transactions",
      "consumer_group": "analytics-group"
    }
  },
  "processing": {
    "window": {
      "type": "tumbling",
      "duration": "5m"
    },
    "aggregations": [
      {"metric": "count", "field": "*"},
      {"metric": "sum", "field": "amount"},
      {"metric": "avg", "field": "amount"}
    ]
  },
  "anomaly_detection": {
    "enabled": true,
    "algorithm": "isolation_forest",
    "sensitivity": 0.95
  },
  "alerts": [
    {
      "condition": "sum(amount) > 100000",
      "channels": ["email", "slack"],
      "recipients": ["analytics-team@example.com"]
    }
  ]
}
```

**Response**:
```json
{
  "status": "success",
  "stream_id": "STREAM-2025-12-26-001",
  "websocket_url": "wss://api.wia.org/v1/streams/STREAM-2025-12-26-001",
  "status_endpoint": "/v1/analytics/streams/STREAM-2025-12-26-001/status",
  "created_at": "2025-12-26T10:00:00Z"
}
```

### 8.2 GET /v1/analytics/streams/{stream_id}/metrics
Get metrics for a running stream.

**Response**:
```json
{
  "stream_id": "STREAM-2025-12-26-001",
  "status": "running",
  "uptime_seconds": 3600,
  "metrics": {
    "events_processed": 15847,
    "events_per_second": 142.5,
    "avg_latency_ms": 23,
    "p95_latency_ms": 45,
    "p99_latency_ms": 78,
    "errors_count": 3,
    "anomalies_detected": 2
  },
  "current_window": {
    "start": "2025-12-26T11:00:00Z",
    "end": "2025-12-26T11:05:00Z",
    "count": 712,
    "sum_amount": 54320.50,
    "avg_amount": 76.29
  },
  "last_updated": "2025-12-26T11:05:00Z"
}
```

## 9. Model Management API

### 9.1 GET /v1/models
List all models.

**Response**:
```json
{
  "models": [
    {
      "model_id": "MODEL-FORECAST-2025-001",
      "name": "Sales Forecast Prophet",
      "type": "time_series_forecast",
      "status": "deployed",
      "created_at": "2025-12-26T10:00:00Z",
      "performance": {
        "r2_score": 0.89,
        "rmse": 12400
      }
    }
  ],
  "pagination": {
    "page": 1,
    "page_size": 10,
    "total_pages": 3,
    "total_count": 25
  }
}
```

### 9.2 POST /v1/models/{model_id}/predict
Use a deployed model for prediction.

**Request**:
```json
{
  "input": {
    "date": "2025-12-27",
    "marketing_spend": 5000
  }
}
```

**Response**:
```json
{
  "model_id": "MODEL-FORECAST-2025-001",
  "prediction": {
    "value": 24500,
    "confidence_interval": {
      "lower": 23300,
      "upper": 25700
    }
  },
  "execution_time_ms": 45
}
```

## 10. Error Handling

### 10.1 Error Response Format
```json
{
  "status": "error",
  "error": {
    "code": "INVALID_DATASET",
    "message": "Dataset not found or inaccessible",
    "details": {
      "dataset_id": "abc123"
    },
    "request_id": "REQ-2025-12-26-001",
    "timestamp": "2025-12-26T10:30:00Z"
  }
}
```

### 10.2 Common Error Codes
| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_DATASET | 404 | Dataset not found |
| INSUFFICIENT_DATA | 400 | Not enough data for analysis |
| MODEL_TRAINING_FAILED | 500 | ML model training failed |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| UNAUTHORIZED | 401 | Invalid or missing API key |
| FORBIDDEN | 403 | Insufficient permissions |
| VALIDATION_ERROR | 400 | Invalid request parameters |

## 11. Rate Limiting

### 11.1 Rate Limit Headers
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1735214400
```

### 11.2 Rate Limits by Tier
| Tier | Requests/Hour | Concurrent Streams |
|------|---------------|-------------------|
| Free | 100 | 1 |
| Standard | 1,000 | 5 |
| Premium | 10,000 | 20 |
| Enterprise | Unlimited | Unlimited |

## 12. Webhooks

### 12.1 Webhook Configuration
```json
{
  "url": "https://your-app.com/webhooks/wia-analytics",
  "events": [
    "model.training_complete",
    "stream.anomaly_detected",
    "prediction.complete"
  ],
  "secret": "your_webhook_secret"
}
```

### 12.2 Webhook Payload
```json
{
  "event_type": "model.training_complete",
  "timestamp": "2025-12-26T10:00:00Z",
  "data": {
    "model_id": "MODEL-FORECAST-2025-001",
    "status": "success",
    "metrics": {
      "r2_score": 0.89,
      "rmse": 12400
    }
  }
}
```

## 13. SDK Support

Official SDKs are available for:
- Python: `pip install wia-analytics`
- JavaScript/TypeScript: `npm install @wia/analytics`
- Java: Maven/Gradle
- Go: `go get github.com/wia-official/analytics-go`
- R: `install.packages("wiaanalytics")`

## 14. Conclusion

This PHASE 2 specification defines the RESTful API for WIA-DATA-012, enabling:
- Standardized access to analytics capabilities
- Consistent API patterns across platforms
- Easy integration with existing systems
- Comprehensive error handling and monitoring

**Next Phase**: PHASE 3 - Protocol Specification

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (홍익인간) · Benefit All Humanity
