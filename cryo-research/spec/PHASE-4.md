# WIA-CRYO-010 PHASE 4: INTEGRATION

**Standard**: WIA-CRYO-010  
**Phase**: 4 - System Integration and Deployment  
**Version**: 1.0.0  
**Date**: January 2025  
**Status**: Active  

## Overview

Phase 4 provides guidance for integrating WIA-CRYO-010 into existing laboratory and research systems.

## 4.1 LIMS Integration

### Data Flow
```
LIMS → WIA-CRYO-010 Adapter → Standardized JSON-LD → Repository
```

### Mapping Table
Map LIMS-specific fields to WIA-CRYO-010 schema:
```json
{
  "lims.sample_id": "sampleId",
  "lims.operator_name": "operator",
  "lims.test_date": "assessmentTime"
}
```

### Example Adapters
- LabWare LIMS
- LabVantage
- STARLIMS
- Custom in-house systems

## 4.2 Electronic Lab Notebook (ELN) Integration

### ELN Export
Configure ELN to export entries in WIA-CRYO-010 format:
```python
# LabArchives plugin example
def export_to_wia(entry_id):
    entry = get_entry(entry_id)
    wia_data = {
        "@context": "https://wia.org/standards/cryo-research/v1",
        "@type": "CryoResearchExperiment",
        "experimentId": entry.id,
        "title": entry.title,
        ...
    }
    return wia_data
```

## 4.3 Equipment Integration

### Temperature Logger
Connect via serial, USB, or network:
```python
import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)
while True:
    line = ser.readline().decode('utf-8')
    temp, timestamp = parse_reading(line)
    publish_to_wia({
        "type": "temperature_update",
        "value": temp,
        "timestamp": timestamp
    })
```

### Flow Cytometer
Export FCS files + metadata in WIA format:
```json
{
  "@type": "ViabilityAssessment",
  "method": "FLOW_CYTOMETRY",
  "rawDataFile": "sample001.fcs",
  "results": {...}
}
```

## 4.4 Database Schema

### PostgreSQL Example
```sql
CREATE TABLE experiments (
    experiment_id VARCHAR(255) PRIMARY KEY,
    json_ld JSONB NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    modified_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_experiments_type ON experiments ((json_ld->>'@type'));
CREATE INDEX idx_experiments_status ON experiments ((json_ld->'lifecycle'->>'status'));
```

### MongoDB Example
```javascript
db.createCollection("experiments", {
   validator: {
      $jsonSchema: {
         bsonType: "object",
         required: ["@context", "@type", "experimentId"],
         properties: {
            "@context": {
               bsonType: "string",
               pattern: "^https://wia.org/standards/cryo-research/"
            }
         }
      }
   }
})
```

## 4.5 Backup and Disaster Recovery

### 3-2-1 Rule
- 3 copies of data
- 2 different media types
- 1 off-site backup

### Automated Backup Script
```bash
#!/bin/bash
# Daily backup of WIA-CRYO-010 data
DATE=$(date +%Y%m%d)
pg_dump cryo_research > /backup/cryo_${DATE}.sql
gzip /backup/cryo_${DATE}.sql
aws s3 cp /backup/cryo_${DATE}.sql.gz s3://wia-backups/
```

## 4.6 Security Considerations

### Encryption
- Data at rest: AES-256
- Data in transit: TLS 1.3
- API keys: bcrypt hashed

### Access Control
```yaml
roles:
  - name: researcher
    permissions: [read, write:own]
  - name: data_manager
    permissions: [read, write:all, delete:own]
  - name: admin
    permissions: [read, write:all, delete:all, admin]
```

### Audit Logging
```json
{
  "action": "UPDATE",
  "resourceType": "Experiment",
  "resourceId": "exp-2025-001",
  "userId": "user-123",
  "timestamp": "2025-01-21T10:15:00Z",
  "changes": {...}
}
```

## 4.7 Performance Optimization

### Caching
- Redis for frequently accessed experiments
- CDN for static schemas
- Database query optimization with indexes

### Bulk Operations
Use batch endpoints for large data imports:
```http
POST /bulk-import
Content-Type: application/x-ndjson

{"@type": "CryoResearchExperiment", ...}
{"@type": "CryoResearchExperiment", ...}
```

## 4.8 Monitoring and Alerting

### Metrics to Track
- API response time (p50, p95, p99)
- Error rate
- Data ingestion rate
- Storage usage

### Alerting Rules
```yaml
alerts:
  - name: high_error_rate
    condition: error_rate > 5%
    notify: ops-team@wia.org
  - name: low_viability_detected
    condition: viability < 70%
    notify: researcher@wia.org
```

## 4.9 Deployment Patterns

### Containerization (Docker)
```dockerfile
FROM python:3.10
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .
CMD ["python", "app.py"]
```

### Kubernetes
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-research-api
spec:
  replicas: 3
  template:
    spec:
      containers:
      - name: api
        image: wia/cryo-research-api:1.0.0
        ports:
        - containerPort: 8080
```

## 4.10 Testing

### Unit Tests
```python
def test_viability_calculation():
    result = calculate_viability(87, 100)
    assert result['viability'] == 87.0
    assert len(result['ci_95']) == 2
```

### Integration Tests
```python
def test_api_submit_experiment():
    response = client.post('/experiments', json=experiment_data)
    assert response.status_code == 201
    assert 'experimentId' in response.json()
```

### Load Testing
```bash
# Using Apache Bench
ab -n 1000 -c 10 https://api.wia.org/cryo-research/v1/experiments
```

## References

- Docker: https://www.docker.com/
- Kubernetes: https://kubernetes.io/
- PostgreSQL: https://www.postgresql.org/
- MongoDB: https://www.mongodb.com/

---

**Previous**: [PHASE-3: Protocols](PHASE-3.md)  
**Complete**: All phases documented

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity
