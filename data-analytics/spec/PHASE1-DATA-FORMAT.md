# WIA-DATA-012: Data Analytics Standard
## PHASE 1 - DATA FORMAT SPECIFICATION

### Version: 1.0
### Status: Active
### Last Updated: 2025-12-26

---

## 1. Overview

This document defines the standardized data formats for the WIA-DATA-012 Data Analytics Standard. These formats ensure interoperability and consistency across different analytics platforms and tools.

**Philosophy**: 弘益人間 (홍익인간) - Benefit All Humanity through standardized, accessible data analytics.

## 2. Data Format Principles

### 2.1 Core Principles
- **Interoperability**: Compatible with common analytics tools
- **Human-Readable**: Text-based formats when possible
- **Machine-Optimized**: Binary formats for performance-critical scenarios
- **Schema Evolution**: Support for versioning and backward compatibility
- **Compression**: Built-in support for data compression

### 2.2 Format Categories
1. **Structured Data**: Tabular data with defined schema
2. **Semi-Structured Data**: JSON, XML for flexible schemas
3. **Unstructured Data**: Text, images, audio, video
4. **Time-Series Data**: Temporal data with timestamps
5. **Graph Data**: Nodes and edges for relationship data

## 3. Structured Data Formats

### 3.1 CSV (Comma-Separated Values)
```csv
# WIA-DATA-012 CSV Format
# version: 1.0
# schema: customer_transactions
# timestamp: 2025-12-26T00:00:00Z

transaction_id,customer_id,amount,timestamp,category
TX001,CUST123,150.50,2025-12-26T10:30:00Z,electronics
TX002,CUST456,75.25,2025-12-26T11:15:00Z,books
```

**Specifications**:
- UTF-8 encoding required
- Header row mandatory
- ISO 8601 datetime format
- Decimal separator: period (.)
- Missing values: empty string or "null"

### 3.2 Parquet
```python
# WIA-DATA-012 Parquet Schema
{
    "type": "record",
    "name": "AnalyticsEvent",
    "namespace": "org.wia.data",
    "fields": [
        {"name": "event_id", "type": "string"},
        {"name": "timestamp", "type": "long"},
        {"name": "user_id", "type": "string"},
        {"name": "event_type", "type": "string"},
        {"name": "properties", "type": {"type": "map", "values": "string"}},
        {"name": "metrics", "type": {"type": "map", "values": "double"}}
    ]
}
```

**Features**:
- Columnar storage for analytical queries
- Built-in compression (Snappy, GZIP)
- Schema embedded in file
- Partition support for large datasets

### 3.3 Avro
```json
{
  "type": "record",
  "name": "AnalyticsMetric",
  "namespace": "org.wia.data.v1",
  "fields": [
    {"name": "metric_id", "type": "string"},
    {"name": "timestamp", "type": "long", "logicalType": "timestamp-millis"},
    {"name": "dimensions", "type": {"type": "map", "values": "string"}},
    {"name": "measures", "type": {"type": "map", "values": "double"}},
    {"name": "metadata", "type": ["null", {"type": "map", "values": "string"}], "default": null}
  ]
}
```

**Use Cases**:
- Streaming data serialization
- Schema evolution support
- Cross-language compatibility

## 4. Semi-Structured Data Formats

### 4.1 JSON
```json
{
  "wia_version": "1.0",
  "format": "analytics_event",
  "data": {
    "event_id": "EVT-12345",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "user": {
      "id": "USER-789",
      "segment": "premium",
      "properties": {
        "age_group": "25-34",
        "region": "US-West"
      }
    },
    "event": {
      "type": "purchase",
      "category": "electronics",
      "value": 299.99,
      "currency": "USD"
    },
    "context": {
      "device": "mobile",
      "platform": "iOS",
      "app_version": "2.5.1"
    }
  }
}
```

**Guidelines**:
- ISO 8601 for timestamps
- UTF-8 encoding
- Numeric values: no quotes
- Boolean values: true/false (lowercase)
- Null values: explicit null

### 4.2 NDJSON (Newline-Delimited JSON)
```json
{"event_id":"E001","timestamp":"2025-12-26T10:00:00Z","value":100}
{"event_id":"E002","timestamp":"2025-12-26T10:01:00Z","value":150}
{"event_id":"E003","timestamp":"2025-12-26T10:02:00Z","value":125}
```

**Features**:
- Streaming-friendly
- One JSON object per line
- Easy to parse incrementally
- Ideal for log files and event streams

## 5. Time-Series Data Format

### 5.1 WIA Time-Series Format
```json
{
  "wia_ts_version": "1.0",
  "metric_name": "cpu_usage",
  "tags": {
    "host": "server-01",
    "region": "us-west-2",
    "env": "production"
  },
  "data_points": [
    {"timestamp": 1735200000000, "value": 45.2},
    {"timestamp": 1735200060000, "value": 47.1},
    {"timestamp": 1735200120000, "value": 42.8}
  ],
  "metadata": {
    "unit": "percent",
    "aggregation": "avg",
    "interval": "60s"
  }
}
```

**Specifications**:
- Timestamps in Unix milliseconds
- Sorted chronologically
- Support for multiple metrics
- Tag-based indexing

## 6. Analytics Results Format

### 6.1 Standard Analytics Response
```json
{
  "wia_analytics_version": "1.0",
  "query_id": "Q-2025-12-26-001",
  "execution_time_ms": 234,
  "status": "success",
  "result_type": "aggregation",
  "data": {
    "summary": {
      "total_records": 15847,
      "date_range": {
        "start": "2025-01-01T00:00:00Z",
        "end": "2025-12-26T23:59:59Z"
      }
    },
    "aggregations": [
      {
        "dimension": "category",
        "metric": "revenue",
        "aggregation": "sum",
        "results": [
          {"key": "electronics", "value": 524000},
          {"key": "books", "value": 189000},
          {"key": "clothing", "value": 312000}
        ]
      }
    ]
  },
  "metadata": {
    "timezone": "UTC",
    "currency": "USD"
  }
}
```

## 7. Model Format

### 7.1 WIA ML Model Metadata
```json
{
  "wia_model_version": "1.0",
  "model_id": "MODEL-SALES-FORECAST-001",
  "model_type": "time_series_forecast",
  "framework": "prophet",
  "created_at": "2025-12-26T10:00:00Z",
  "training": {
    "data_range": {
      "start": "2024-01-01",
      "end": "2025-12-25"
    },
    "records_count": 365,
    "features": ["date", "revenue", "marketing_spend"],
    "target": "revenue"
  },
  "performance": {
    "metrics": {
      "rmse": 12400,
      "mae": 9800,
      "r2_score": 0.89,
      "mape": 6.5
    },
    "validation_method": "time_series_split",
    "test_size": 0.2
  },
  "parameters": {
    "yearly_seasonality": true,
    "weekly_seasonality": true,
    "changepoint_prior_scale": 0.05
  }
}
```

## 8. Data Quality Metadata

### 8.1 Quality Metrics Format
```json
{
  "dataset_id": "sales_2025",
  "quality_check_timestamp": "2025-12-26T10:00:00Z",
  "quality_score": 0.94,
  "dimensions": {
    "completeness": 0.98,
    "accuracy": 0.92,
    "consistency": 0.95,
    "timeliness": 0.99,
    "validity": 0.96
  },
  "issues": [
    {
      "type": "missing_values",
      "field": "customer_email",
      "count": 234,
      "percentage": 1.5
    },
    {
      "type": "outliers",
      "field": "order_amount",
      "count": 12,
      "percentage": 0.08
    }
  ]
}
```

## 9. Compression and Encoding

### 9.1 Supported Compression
- **GZIP**: General purpose, good compression ratio
- **Snappy**: Fast compression, lower ratio
- **LZ4**: Very fast, streaming-friendly
- **Zstandard**: Modern, configurable compression

### 9.2 Encoding Standards
- **Text**: UTF-8 mandatory
- **Numeric**: IEEE 754 for floating point
- **Timestamps**: ISO 8601 or Unix epoch
- **Binary**: Base64 when in JSON

## 10. Validation Rules

### 10.1 Required Fields
All WIA-DATA-012 datasets must include:
- `wia_version`: Standard version
- `timestamp`: Creation/modification time
- `schema_version`: Data schema version

### 10.2 Data Type Validation
```python
# Example validation rules
{
    "event_id": {"type": "string", "pattern": "^[A-Z0-9-]+$"},
    "timestamp": {"type": "integer", "minimum": 0},
    "amount": {"type": "number", "minimum": 0},
    "email": {"type": "string", "format": "email"}
}
```

## 11. Implementation Examples

### 11.1 Python
```python
from datetime import datetime
import json

def create_wia_event(event_id, user_id, event_type, value):
    return {
        "wia_version": "1.0",
        "format": "analytics_event",
        "data": {
            "event_id": event_id,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "user_id": user_id,
            "event_type": event_type,
            "value": value
        }
    }

# Usage
event = create_wia_event("E001", "U123", "purchase", 99.99)
print(json.dumps(event, indent=2))
```

### 11.2 TypeScript
```typescript
interface WIAEvent {
  wia_version: string;
  format: string;
  data: {
    event_id: string;
    timestamp: string;
    user_id: string;
    event_type: string;
    value: number;
  };
}

function createWIAEvent(
  eventId: string,
  userId: string,
  eventType: string,
  value: number
): WIAEvent {
  return {
    wia_version: "1.0",
    format: "analytics_event",
    data: {
      event_id: eventId,
      timestamp: new Date().toISOString(),
      user_id: userId,
      event_type: eventType,
      value: value
    }
  };
}
```

## 12. Conclusion

This PHASE 1 specification establishes the foundational data formats for WIA-DATA-012. Adherence to these formats ensures:
- Interoperability across analytics platforms
- Consistency in data exchange
- Ease of validation and quality assurance
- Future extensibility

**Next Phase**: PHASE 2 - API Specification

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (홍익인간) · Benefit All Humanity
