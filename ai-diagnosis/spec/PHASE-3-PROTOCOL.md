# WIA AI Diagnosis Standard - Phase 3: Protocol Specification

> **Version**: 1.0.0
> **Status**: Stable
> **Last Updated**: 2025
> **Standard**: WIA-MED-009

---

## 1. Overview

Phase 3 defines the communication protocols for real-time AI diagnostic systems integrated with Electronic Health Records (EHR), Picture Archiving and Communication Systems (PACS), and clinical workflows. This specification ensures low-latency, reliable diagnostic support in clinical settings.

### 1.1 Design Principles

- **Low Latency**: Minimize diagnostic delay for clinical workflows
- **Reliability**: Handle disconnections and ensure data integrity
- **Clinical Safety**: Built-in safeguards for patient safety
- **Interoperability**: Seamless EHR/PACS integration

---

## 2. Integration Protocols

### 2.1 HL7 FHIR Integration

#### FHIR Resource Mapping

| WIA Diagnostic Field | FHIR Resource | FHIR Element |
|---------------------|---------------|--------------|
| diagnosis | DiagnosticReport | conclusion, conclusionCode |
| patient_context | Patient | demographics, extension |
| clinical_findings | Observation | value, component |
| recommendations | ServiceRequest | code, priority |
| explainability | DiagnosticReport | presentedForm (PDF/text) |

#### FHIR DiagnosticReport Example

```json
{
    "resourceType": "DiagnosticReport",
    "id": "ai-diagnosis-12345",
    "meta": {
        "profile": ["http://wiastandards.com/fhir/StructureDefinition/ai-diagnostic-report"]
    },
    "status": "final",
    "category": [{
        "coding": [{
            "system": "http://terminology.hl7.org/CodeSystem/v2-0074",
            "code": "LAB"
        }]
    }],
    "code": {
        "coding": [{
            "system": "http://loinc.org",
            "code": "11526-1",
            "display": "Pathology study"
        }]
    },
    "subject": {
        "reference": "Patient/encrypted-patient-xyz"
    },
    "effectiveDateTime": "2025-01-15T14:30:00Z",
    "issued": "2025-01-15T14:31:15Z",
    "performer": [{
        "reference": "Organization/wia-ai-diagnostics",
        "display": "WIA Certified AI Diagnostics System"
    }],
    "conclusionCode": [{
        "coding": [{
            "system": "http://hl7.org/fhir/sid/icd-10",
            "code": "I50.9",
            "display": "Heart Failure, Unspecified"
        }]
    }],
    "conclusion": "AI-assisted diagnosis suggests heart failure with 87% confidence. Supporting evidence includes elevated BNP (450 pg/mL) and cardiomegaly on imaging.",
    "extension": [{
        "url": "http://wiastandards.com/fhir/StructureDefinition/ai-confidence",
        "valueDecimal": 0.87
    }, {
        "url": "http://wiastandards.com/fhir/StructureDefinition/ai-explainability",
        "valueString": "Primary contributing factors: BNP level (35%), Cardiomegaly (28%), Dyspnea (22%)"
    }]
}
```

### 2.2 DICOM Integration

#### DICOM Structured Report (SR)

```
# DICOM AI Diagnosis Report Template
(0008,0016) SOP Class UID: 1.2.840.10008.5.1.4.1.1.88.11 [Basic Text SR]
(0008,0018) SOP Instance UID: {generated_uid}
(0020,000D) Study Instance UID: {study_uid}
(0020,000E) Series Instance UID: {series_uid}

# Content Tree
Container: (113701, DCM, "X-Ray Report")
  - Text: (121106, DCM, "Findings")
      Value: "Cardiomegaly with cardiothoracic ratio of 0.58"
  - Code: (121071, DCM, "Finding")
      (59621000, SCT, "Cardiomegaly")
  - Num: AI Confidence
      Value: 0.89
      Unit: (1, UCUM, "1")
  - Container: (126000, DCM, "Imaging Measurement Report")
      - Num: Cardiothoracic Ratio
          Value: 0.58
          Unit: (1, UCUM, "1")
```

#### DICOM Query/Retrieve (C-FIND, C-MOVE)

```python
# Query for patient images
query_dataset = Dataset()
query_dataset.PatientID = 'encrypted-patient-xyz'
query_dataset.StudyDate = '20250115'
query_dataset.Modality = 'CR'  # Chest Radiography

# Retrieve and analyze
images = dicom_scp.find(query_dataset)
for image in images:
    analysis_result = ai_diagnosis_api.analyze_image(image)
    sr_report = create_dicom_sr(analysis_result)
    dicom_scp.store(sr_report)
```

---

## 3. Real-Time Streaming Protocol

### 3.1 WebSocket Connection

#### Connection URL

```
wss://stream.{provider}.com/wia/ai-diagnosis/v1/stream
```

#### Handshake

```http
GET /wia/ai-diagnosis/v1/stream HTTP/1.1
Host: stream.provider.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer {access_token}
X-WIA-Version: 1.0.0
X-Institution-ID: {institution_id}
```

### 3.2 Session Management

#### Session Start

**Client → Server**:
```json
{
    "type": "session.start",
    "session_id": "clinic-session-12345",
    "timestamp": "2025-01-15T14:30:00.000Z",
    "config": {
        "mode": "real_time_assist",
        "specialty": "cardiology",
        "auto_analyze_labs": true,
        "auto_analyze_images": true,
        "alert_thresholds": {
            "critical_urgency": true,
            "min_confidence": 0.7
        }
    },
    "physician": {
        "id": "physician-12345",
        "license": "MD-12345-KR",
        "specialties": ["cardiology"]
    }
}
```

**Server → Client**:
```json
{
    "type": "session.started",
    "session_id": "session-server-67890",
    "timestamp": "2025-01-15T14:30:00.100Z",
    "config_applied": {
        "mode": "real_time_assist",
        "specialty": "cardiology",
        "session_timeout_seconds": 3600
    },
    "available_models": [
        {
            "model_id": "cardio-ai-dx-v2.3",
            "specialties": ["cardiology"],
            "wia_certification": "gold"
        }
    ]
}
```

### 3.3 Data Streaming

#### Clinical Data Update

**Client → Server**:
```json
{
    "type": "clinical_data.update",
    "timestamp": "2025-01-15T14:30:05.000Z",
    "patient_id": "encrypted-patient-xyz",
    "update_type": "lab_results",
    "data": {
        "laboratory_results": [
            {
                "test_name": "BNP",
                "loinc_code": "33762-6",
                "value": 450,
                "unit": "pg/mL",
                "timestamp": "2025-01-15T14:00:00.000Z"
            }
        ]
    }
}
```

#### Diagnosis Event

**Server → Client**:
```json
{
    "type": "diagnosis.generated",
    "event_id": "diag-evt-12345",
    "timestamp": "2025-01-15T14:30:06.000Z",
    "patient_id": "encrypted-patient-xyz",
    "latency_ms": 850,
    "diagnosis": [
        {
            "condition": "I50.9",
            "condition_name": "Heart Failure, Unspecified",
            "confidence": 0.87,
            "urgency": "urgent",
            "differential_rank": 1
        }
    ],
    "alert": {
        "type": "urgent_finding",
        "message": "Urgent: Possible heart failure detected. Recommend cardiology consultation within 48 hours.",
        "severity": "high"
    },
    "recommendations": [
        {
            "type": "specialist_referral",
            "description": "Cardiology consultation",
            "priority": "high",
            "timeframe": "48 hours"
        }
    ]
}
```

#### Critical Alert

**Server → Client**:
```json
{
    "type": "alert.critical",
    "alert_id": "critical-alert-99999",
    "timestamp": "2025-01-15T14:30:06.100Z",
    "patient_id": "encrypted-patient-xyz",
    "severity": "critical",
    "urgency": "life_threatening",
    "diagnosis": {
        "condition": "I21.0",
        "condition_name": "ST-Elevation Myocardial Infarction (STEMI)",
        "confidence": 0.94
    },
    "immediate_actions": [
        "Activate cardiac catheterization lab",
        "Administer aspirin 325mg",
        "Call cardiology STAT"
    ],
    "audio_alert": true,
    "visual_priority": "flash_red"
}
```

---

## 4. EHR Integration Patterns

### 4.1 SMART on FHIR

#### Launch Sequence

```javascript
// EHR launches AI diagnosis app
const smartClient = FHIR.oauth2.authorize({
    client_id: 'wia-ai-diagnosis-app',
    scope: 'launch patient/*.read DiagnosticReport.write',
    redirectUri: 'https://ai-dx.provider.com/launch',
    iss: 'https://ehr.hospital.com/fhir'
});

// Retrieve patient context
const patient = await smartClient.patient.read();
const observations = await smartClient.request(`Observation?patient=${patient.id}`);

// Generate diagnosis
const diagnosis = await aiDiagnosisAPI.diagnose({
    patientContext: mapPatientToWIA(patient),
    clinicalData: mapObservationsToWIA(observations)
});

// Write back to EHR
await smartClient.create({
    resourceType: 'DiagnosticReport',
    ...mapDiagnosisToFHIR(diagnosis)
});
```

### 4.2 CDS Hooks Integration

#### Hook: order-select

```json
{
    "hookInstance": "d1577c69-dfbe-44ad-ba6d-3e05e953b2ea",
    "hook": "order-select",
    "context": {
        "userId": "Practitioner/physician-12345",
        "patientId": "Patient/encrypted-patient-xyz",
        "selections": ["ServiceRequest/imaging-chest-xray"],
        "draftOrders": {
            "resourceType": "Bundle",
            "entry": [{
                "resource": {
                    "resourceType": "ServiceRequest",
                    "code": {
                        "coding": [{
                            "system": "http://loinc.org",
                            "code": "36643-5",
                            "display": "Chest X-ray"
                        }]
                    }
                }
            }]
        }
    }
}
```

#### CDS Service Response

```json
{
    "cards": [{
        "summary": "AI Diagnosis: Possible Heart Failure",
        "indicator": "warning",
        "detail": "Based on patient's symptoms (dyspnea, edema) and elevated BNP (450 pg/mL), AI analysis suggests high probability (87%) of heart failure. Consider echocardiogram in addition to chest X-ray.",
        "source": {
            "label": "WIA AI Diagnosis System",
            "url": "https://ai-dx.provider.com"
        },
        "suggestions": [{
            "label": "Add echocardiogram order",
            "actions": [{
                "type": "create",
                "description": "Add echo to imaging orders",
                "resource": {
                    "resourceType": "ServiceRequest",
                    "code": {
                        "coding": [{
                            "system": "http://loinc.org",
                            "code": "34552-0",
                            "display": "Echocardiogram"
                        }]
                    },
                    "priority": "urgent"
                }
            }]
        }],
        "links": [{
            "label": "View full AI diagnostic report",
            "url": "https://ai-dx.provider.com/reports/diag-evt-12345",
            "type": "absolute"
        }]
    }]
}
```

---

## 5. Asynchronous Processing

### 5.1 Message Queue Integration

#### RabbitMQ/AMQP

```python
import pika

# Producer (EHR system)
connection = pika.BlockingConnection(
    pika.ConnectionParameters('queue.hospital.com')
)
channel = connection.channel()
channel.queue_declare(queue='ai_diagnosis_requests', durable=True)

diagnostic_request = {
    'patient_id': 'encrypted-patient-xyz',
    'clinical_data': {...},
    'priority': 'high',
    'callback_queue': 'ehr_system_responses'
}

channel.basic_publish(
    exchange='',
    routing_key='ai_diagnosis_requests',
    body=json.dumps(diagnostic_request),
    properties=pika.BasicProperties(
        delivery_mode=2,  # persistent
        priority=5
    )
)

# Consumer (AI diagnosis service)
def process_diagnosis_request(ch, method, properties, body):
    request = json.loads(body)
    diagnosis = ai_model.diagnose(request)

    # Send result to callback queue
    ch.basic_publish(
        exchange='',
        routing_key=request['callback_queue'],
        body=json.dumps(diagnosis)
    )
    ch.basic_ack(delivery_tag=method.delivery_tag)

channel.basic_consume(
    queue='ai_diagnosis_requests',
    on_message_callback=process_diagnosis_request
)
```

### 5.2 Kafka Streaming

```python
from kafka import KafkaProducer, KafkaConsumer

# Producer
producer = KafkaProducer(
    bootstrap_servers=['kafka.hospital.com:9092'],
    value_serializer=lambda v: json.dumps(v).encode('utf-8')
)

producer.send('clinical_events', {
    'event_type': 'new_lab_result',
    'patient_id': 'encrypted-patient-xyz',
    'data': {...}
})

# Consumer
consumer = KafkaConsumer(
    'clinical_events',
    bootstrap_servers=['kafka.hospital.com:9092'],
    value_deserializer=lambda m: json.loads(m.decode('utf-8'))
)

for message in consumer:
    if message.value['event_type'] == 'new_lab_result':
        diagnosis = ai_diagnosis_service.analyze_labs(message.value['data'])

        # Publish diagnosis to results topic
        producer.send('ai_diagnosis_results', diagnosis)
```

---

## 6. Security Protocols

### 6.1 TLS/SSL Configuration

```nginx
# NGINX configuration for AI diagnosis API
server {
    listen 443 ssl http2;
    server_name api.ai-diagnosis.com;

    # TLS 1.3 only
    ssl_protocols TLSv1.3;
    ssl_ciphers 'TLS_AES_256_GCM_SHA384:TLS_CHACHA20_POLY1305_SHA256';

    # Certificate
    ssl_certificate /etc/ssl/certs/ai-diagnosis.crt;
    ssl_certificate_key /etc/ssl/private/ai-diagnosis.key;

    # HIPAA compliance
    ssl_prefer_server_ciphers on;
    ssl_session_timeout 10m;
    ssl_session_cache shared:SSL:10m;

    # HSTS
    add_header Strict-Transport-Security "max-age=31536000; includeSubDomains" always;

    location /wia/ai-diagnosis/v1/ {
        proxy_pass http://ai-diagnosis-backend;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;

        # Audit logging
        access_log /var/log/nginx/ai-diagnosis-access.log combined;
    }
}
```

### 6.2 Mutual TLS (mTLS)

```python
import ssl
import requests

# Client certificate authentication
cert = ('/path/to/client.crt', '/path/to/client.key')
ca_bundle = '/path/to/ca-bundle.crt'

response = requests.post(
    'https://api.ai-diagnosis.com/wia/ai-diagnosis/v1/diagnose',
    json=diagnostic_request,
    cert=cert,
    verify=ca_bundle
)
```

### 6.3 End-to-End Encryption

```python
from cryptography.fernet import Fernet

# Encrypt patient data before transmission
encryption_key = Fernet.generate_key()
cipher = Fernet(encryption_key)

encrypted_patient_data = cipher.encrypt(
    json.dumps(patient_context).encode()
)

# Send encrypted data
response = requests.post(
    api_url,
    json={
        'encrypted_data': encrypted_patient_data.decode(),
        'encryption_key_id': 'key-12345'
    }
)

# Decrypt on server side
decrypted_data = cipher.decrypt(encrypted_patient_data)
patient_context = json.loads(decrypted_data.decode())
```

---

## 7. Performance Optimization

### 7.1 Caching Strategy

```python
from redis import Redis
import hashlib

redis_client = Redis(host='cache.hospital.com', port=6379)

def get_cached_diagnosis(patient_context, clinical_data):
    # Generate cache key
    cache_key = hashlib.sha256(
        json.dumps({
            'patient': patient_context,
            'data': clinical_data
        }, sort_keys=True).encode()
    ).hexdigest()

    # Check cache
    cached = redis_client.get(f'diagnosis:{cache_key}')
    if cached:
        return json.loads(cached)

    # Generate new diagnosis
    diagnosis = ai_model.diagnose(patient_context, clinical_data)

    # Cache for 1 hour (clinical data changes)
    redis_client.setex(
        f'diagnosis:{cache_key}',
        3600,
        json.dumps(diagnosis)
    )

    return diagnosis
```

### 7.2 Load Balancing

```yaml
# HAProxy configuration
global
    maxconn 4096

defaults
    mode http
    timeout connect 5s
    timeout client 50s
    timeout server 50s

frontend ai_diagnosis_frontend
    bind *:443 ssl crt /etc/ssl/certs/ai-diagnosis.pem
    default_backend ai_diagnosis_backend

backend ai_diagnosis_backend
    balance roundrobin
    option httpchk GET /health
    server ai-dx-1 10.0.1.10:8080 check
    server ai-dx-2 10.0.1.11:8080 check
    server ai-dx-3 10.0.1.12:8080 check
```

---

## 8. Audit Logging

### 8.1 Structured Audit Log

```json
{
    "timestamp": "2025-01-15T14:30:00.000Z",
    "event_type": "diagnosis_generated",
    "event_id": "audit-log-12345",
    "actor": {
        "type": "physician",
        "id": "physician-12345",
        "name": "Dr. Kim (anonymized)",
        "license": "MD-12345-KR"
    },
    "patient": {
        "id": "encrypted-patient-xyz",
        "age_range": "40-64"
    },
    "action": "AI diagnosis generated",
    "resource": {
        "type": "DiagnosticReport",
        "id": "diag-evt-12345"
    },
    "outcome": "success",
    "diagnosis": {
        "condition": "I50.9",
        "confidence": 0.87
    },
    "ip_address": "10.0.5.123",
    "user_agent": "EHR-System/3.5.0",
    "session_id": "session-abc-123"
}
```

---

**홍익인간 (弘益人間)**: "Benefit all humanity"

The WIA AI Diagnosis Standard belongs to humanity. Free forever.

---

**Copyright 2025 SmileStory Inc. / WIA**
MIT License
