# WIA-IND-002: Smart Textile Standard
## PHASE 4: SYSTEM INTEGRATION SPECIFICATION
### 弘益人間 - Benefit All Humanity

---

## 1. Overview

This document defines system integration specifications for WIA-IND-002 Smart Textile Standard, enabling seamless connectivity with health platforms, medical systems, smart home ecosystems, and enterprise applications.

**Version:** 1.0  
**Status:** Final  
**Last Updated:** 2025-12-27

## 2. Apple HealthKit Integration

### 2.1 Required Capabilities

```swift
HKHealthStore Permissions:
- Read: HKQuantityTypeIdentifier.heartRate
- Read: HKQuantityTypeIdentifier.bodyTemperature
- Read: HKQuantityTypeIdentifier.respiratoryRate
- Write: All above types
- Read/Write: HKWorkoutTypeIdentifier
```

### 2.2 Data Type Mapping

```swift
WIA-IND-002 → HealthKit Mapping:

Temperature:
  HKQuantityType(.bodyTemperature)
  Unit: HKUnit.degreeCelsius()
  
Heart Rate:
  HKQuantityType(.heartRate)
  Unit: HKUnit(from: "count/min")
  Metadata: [
    HKMetadataKeyHeartRateMotionContext: HKHeartRateMotionContext.active
  ]

ECG:
  HKElectrocardiogram
  Classification: sinus_rhythm|atrial_fibrillation|inconclusive
  SamplingFrequency: 512 Hz
  
Respiratory Rate:
  HKQuantityType(.respiratoryRate)
  Unit: HKUnit(from: "count/min")

Steps:
  HKQuantityType(.stepCount)
  Unit: HKUnit.count()
```

### 2.3 Background Delivery

```swift
// Enable background delivery for real-time updates
healthStore.enableBackgroundDelivery(
    for: HKObjectType.quantityType(forIdentifier: .heartRate)!,
    frequency: .immediate
) { success, error in
    // Handle background delivery setup
}
```

### 2.4 Workout Integration

```swift
let workout = HKWorkout(
    activityType: .running,
    start: startDate,
    end: endDate,
    workoutEvents: events,
    totalEnergyBurned: energy,
    totalDistance: distance,
    metadata: [
        "WIA-IND-002-DeviceId": deviceId,
        "Philosophy": "弘益人間"
    ]
)
```

## 3. Google Fit Integration

### 3.1 Data Types

```java
// Google Fit Data Type Mapping
WIA Temperature → TYPE_BODY_TEMPERATURE
  - Field: FIELD_TEMPERATURE (degrees Celsius)
  
WIA Heart Rate → TYPE_HEART_RATE_BPM
  - Field: FIELD_BPM
  - Field: FIELD_CONFIDENCE (optional)

WIA Activity → TYPE_ACTIVITY_SEGMENT
  - Field: FIELD_ACTIVITY
  - Values: walking, running, sleeping, etc.

WIA Steps → TYPE_STEP_COUNT_DELTA
  - Field: FIELD_STEPS

WIA Calories → TYPE_CALORIES_EXPENDED
  - Field: FIELD_CALORIES
```

### 3.2 Data Source Registration

```java
DataSource dataSource = new DataSource.Builder()
    .setAppPackageName(context)
    .setDataType(DataType.TYPE_HEART_RATE_BPM)
    .setName("WIA-IND-002 Smart Textile")
    .setType(DataSource.TYPE_RAW)
    .setDevice(
        new Device("WIA", "SmartShirt", "1.0", Device.TYPE_WATCH)
    )
    .build();
```

### 3.3 Session Recording

```java
Session session = new Session.Builder()
    .setName("Morning Run")
    .setIdentifier("wia-ind-002-session-" + UUID.randomUUID())
    .setActivity(FitnessActivities.RUNNING)
    .setStartTime(startTime, TimeUnit.MILLISECONDS)
    .setEndTime(endTime, TimeUnit.MILLISECONDS)
    .build();
```

### 3.4 Real-Time Data Streaming

```java
SensorRequest sensorRequest = new SensorRequest.Builder()
    .setDataType(DataType.TYPE_HEART_RATE_BPM)
    .setSamplingRate(1, TimeUnit.SECONDS)
    .build();

Fitness.getSensorsClient(context)
    .add(sensorRequest, dataPointListener);
```

## 4. Samsung Health Integration

### 4.1 Data Types

```kotlin
// Samsung Health Data Type Mapping
WIA Temperature → HealthDataStore.BODY_TEMPERATURE
WIA Heart Rate → HealthDataStore.HEART_RATE
WIA Steps → HealthDataStore.STEP_COUNT
WIA Sleep → HealthDataStore.SLEEP
WIA Blood Oxygen → HealthDataStore.OXYGEN_SATURATION
```

### 4.2 Permission Request

```kotlin
val permissions = setOf(
    HealthPermission.HEART_RATE,
    HealthPermission.BODY_TEMPERATURE,
    HealthPermission.OXYGEN_SATURATION
)

healthStore.requestPermissions(permissions)
    .addOnSuccessListener { granted ->
        // Permissions granted
    }
```

### 4.3 Data Insertion

```kotlin
val heartRateData = HeartRateRecord(
    bpm = 72,
    timestamp = Instant.now(),
    metadata = Metadata(
        dataOrigin = DataOrigin("com.wia.smarttextile"),
        clientRecordId = "wia-ind-002-${UUID.randomUUID()}"
    )
)

healthStore.insertRecords(listOf(heartRateData))
```

## 5. FHIR Healthcare Integration

### 5.1 FHIR Resource Mapping

```json
// Observation Resource for Vital Signs
{
  "resourceType": "Observation",
  "id": "wia-ind-002-temp-001",
  "meta": {
    "profile": ["http://hl7.org/fhir/StructureDefinition/vitalsigns"]
  },
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/observation-category",
      "code": "vital-signs"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "8310-5",
      "display": "Body temperature"
    }]
  },
  "subject": {
    "reference": "Patient/example"
  },
  "effectiveDateTime": "2025-12-27T10:00:00Z",
  "valueQuantity": {
    "value": 36.5,
    "unit": "Cel",
    "system": "http://unitsofmeasure.org",
    "code": "Cel"
  },
  "device": {
    "reference": "Device/wia-ind-002-device-001",
    "display": "WIA Smart Textile Sensor"
  },
  "note": [{
    "text": "弘益人間 - Data collected for benefit of all humanity"
  }]
}
```

### 5.2 Device Resource

```json
{
  "resourceType": "Device",
  "id": "wia-ind-002-device-001",
  "identifier": [{
    "system": "http://wia.org/devices",
    "value": "IND-002-12345"
  }],
  "status": "active",
  "manufacturer": "WIA Certified Manufacturer",
  "deviceName": [{
    "name": "WIA Smart Textile Sensor",
    "type": "user-friendly-name"
  }],
  "modelNumber": "WIA-IND-002-v1",
  "type": {
    "coding": [{
      "system": "http://snomed.info/sct",
      "code": "706635000",
      "display": "Wearable vital signs monitor"
    }]
  },
  "version": [{
    "value": "1.0"
  }]
}
```

### 5.3 DiagnosticReport for ECG

```json
{
  "resourceType": "DiagnosticReport",
  "id": "wia-ecg-001",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/v2-0074",
      "code": "CG",
      "display": "Cardiology"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "11524-6",
      "display": "EKG study"
    }]
  },
  "subject": {
    "reference": "Patient/example"
  },
  "effectiveDateTime": "2025-12-27T10:00:00Z",
  "conclusion": "Sinus rhythm, normal ECG",
  "conclusionCode": [{
    "coding": [{
      "system": "http://snomed.info/sct",
      "code": "251120008",
      "display": "Normal sinus rhythm"
    }]
  }]
}
```

## 6. Smart Home Integration

### 6.1 Amazon Alexa Integration

```javascript
// Alexa Smart Home Skill
const WIASmartTextileController = {
    namespace: "Alexa.HealthHub",
    
    async getHealthStatus(userId) {
        // Query WIA-IND-002 device
        const data = await wiaApi.getLatestData(userId);
        
        return {
            heartRate: data.heartRate.value,
            temperature: data.temperature.value,
            timestamp: data.timestamp
        };
    },
    
    async enableMonitoring(userId) {
        await wiaApi.setMonitoringMode(userId, "continuous");
        return {
            status: "Continuous health monitoring enabled. 弘益人間"
        };
    }
};

// Voice Commands:
// "Alexa, what's my heart rate?"
// "Alexa, check my temperature"
// "Alexa, start health monitoring"
```

### 6.2 Google Home Integration

```javascript
// Google Home Action
app.intent('health.status', (conv) => {
    const userId = conv.user.id;
    const data = wiaApi.getLatestData(userId);
    
    conv.ask(`Your current heart rate is ${data.heartRate.value} beats per minute, 
              and your body temperature is ${data.temperature.value} degrees Celsius.`);
});
```

### 6.3 Apple HomeKit Integration

```swift
// HomeKit Accessory Configuration
let service = HMService(type: HMServiceTypeHealthSensor, name: "WIA Smart Textile")

let heartRateCharacteristic = HMCharacteristic()
heartRateCharacteristic.characteristicType = "heartRate"
heartRateCharacteristic.value = currentHeartRate

service.addCharacteristic(heartRateCharacteristic)
```

### 6.4 Home Automation Triggers

```yaml
# Home Assistant Configuration
automation:
  - alias: "High Heart Rate Alert"
    trigger:
      platform: numeric_state
      entity_id: sensor.wia_heart_rate
      above: 100
    action:
      - service: light.turn_on
        entity_id: light.bedroom
        data:
          color_name: red
          brightness: 255
      - service: notify.mobile_app
        data:
          message: "Elevated heart rate detected: 弘益人間"
```

## 7. Enterprise System Integration

### 7.1 Occupational Health Monitoring

```json
// Integration with Enterprise Health Systems
{
  "workerId": "EMP-12345",
  "deviceId": "WIA-IND-002-789",
  "shift": {
    "start": "2025-12-27T08:00:00Z",
    "end": "2025-12-27T16:00:00Z"
  },
  "exposures": {
    "heatStress": {
      "maxTemperature": 38.5,
      "duration": 120,
      "unit": "minutes"
    },
    "physicalStrain": {
      "avgHeartRate": 110,
      "peakHeartRate": 145
    }
  },
  "alerts": [
    {
      "type": "heat_stress_warning",
      "timestamp": "2025-12-27T14:30:00Z",
      "action": "Mandatory rest break initiated"
    }
  ],
  "compliance": "OSHA_29CFR1910"
}
```

### 7.2 Telemedicine Platform Integration

```graphql
mutation CreateTelehealthSession {
  createSession(input: {
    patientId: "patient-123"
    deviceId: "wia-ind-002-device"
    sessionType: REMOTE_MONITORING
    vitalsStreaming: true
  }) {
    sessionId
    streamUrl
    vitals {
      heartRate
      temperature
      respiratoryRate
    }
  }
}
```

### 7.3 Electronic Health Record (EHR) Integration

```xml
<!-- HL7 v2 Message for ADT (Admit/Discharge/Transfer) -->
<ADT_A01>
  <MSH>
    <MSH.3>WIA-IND-002</MSH.3>
    <MSH.9>ADT^A01</MSH.9>
  </MSH>
  <PID>
    <PID.3>Patient-ID</PID.3>
  </PID>
  <OBX>
    <OBX.3>8310-5^Body Temperature^LN</OBX.3>
    <OBX.5>36.5</OBX.5>
    <OBX.6>Cel</OBX.6>
    <OBX.17>WIA-IND-002-Device-ID</OBX.17>
  </OBX>
</ADT_A01>
```

## 8. Cloud Platform Integration

### 8.1 AWS IoT Core

```json
{
  "thingName": "wia-ind-002-device-001",
  "thingTypeName": "SmartTextile",
  "attributes": {
    "standard": "WIA-IND-002",
    "version": "1.0",
    "sensors": "temp,ecg,motion"
  },
  "shadowUpdate": {
    "state": {
      "reported": {
        "temperature": 36.5,
        "heartRate": 72,
        "batteryLevel": 85
      }
    }
  }
}
```

### 8.2 Azure IoT Hub

```csharp
// Device Twin Update
var twin = await registryManager.GetTwinAsync(deviceId);
var patch = @"{
    properties: {
        desired: {
            samplingRate: 100,
            notificationThreshold: {
                heartRate: { min: 50, max: 120 }
            }
        }
    }
}";
await registryManager.UpdateTwinAsync(deviceId, patch, twin.ETag);
```

### 8.3 Google Cloud IoT

```python
# Publish telemetry to Cloud IoT
client = mqtt.Client(client_id=client_id)
client.tls_set(ca_certs=ca_certs)
client.connect(mqtt_bridge_hostname, mqtt_bridge_port)

payload = {
    "deviceId": "wia-ind-002-001",
    "timestamp": datetime.utcnow().isoformat(),
    "sensors": {
        "temperature": 36.5,
        "heartRate": 72
    },
    "philosophy": "弘益人間"
}

client.publish(mqtt_topic, json.dumps(payload), qos=1)
```

## 9. Data Export and Portability

### 9.1 Export Formats

```json
// Standard Export Format
{
  "exportId": "UUID",
  "exportDate": "ISO 8601",
  "standard": "WIA-IND-002",
  "format": "json|csv|fhir|hl7",
  "dateRange": {
    "start": "ISO 8601",
    "end": "ISO 8601"
  },
  "data": [
    // Array of sensor data records
  ],
  "metadata": {
    "deviceId": "UUID",
    "userId": "hashed UUID",
    "recordCount": 10000
  }
}
```

### 9.2 GDPR Compliance

```javascript
// Data Portability API
app.get('/api/export/user-data', authenticate, async (req, res) => {
    const userId = req.user.id;
    
    const data = await wiaApi.exportAllUserData(userId, {
        format: 'json',
        includeMetadata: true,
        philosophy: '弘益人間'
    });
    
    res.setHeader('Content-Disposition', 'attachment; filename=wia-data-export.json');
    res.json(data);
});

// Right to Erasure
app.delete('/api/user-data', authenticate, async (req, res) => {
    const userId = req.user.id;
    await wiaApi.deleteAllUserData(userId);
    res.json({ status: 'Data deleted per GDPR Article 17' });
});
```

## 10. Integration Testing and Certification

### 10.1 Compliance Testing

```yaml
# Integration Test Suite
tests:
  - name: "HealthKit Integration"
    steps:
      - authenticate_with_healthkit
      - write_temperature_sample
      - verify_data_appears_in_health_app
      - read_heart_rate_from_healthkit
    expected:
      - all_steps_succeed
      - data_accuracy_within_tolerance
  
  - name: "FHIR Compatibility"
    steps:
      - generate_fhir_observation
      - validate_against_schema
      - post_to_fhir_server
    expected:
      - valid_fhir_r4_resource
      - successful_server_response
```

### 10.2 Certification Requirements

To achieve WIA-IND-002 certification, devices MUST:
1. Pass all Phase 1-4 compliance tests
2. Demonstrate integration with minimum 2 health platforms
3. Support FHIR R4 data export
4. Implement required security protocols
5. Provide complete API documentation
6. Pass independent security audit

---

**Philosophy:** 弘益人間 - These integration specifications ensure that smart textile data flows seamlessly across health ecosystems, medical systems, and smart environments, enabling technology that benefits all humanity through interoperable, accessible, and secure connectivity.

**License:** Creative Commons BY-SA 4.0  
**Contact:** standards@wia.org

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-002-smart-textile is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-IND-002-smart-textile/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-002-smart-textile/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-002-smart-textile/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
