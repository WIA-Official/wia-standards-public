# WIA-IND-012 PHASE 4: INTEGRATION SPECIFICATION
## Fitness Wearable Standard - 弘益人間 (Benefit All Humanity)

**Version:** 1.0
**Status:** Active
**Category:** IND (Industrial)
**Last Updated:** 2025-01-15

---

## 1. Introduction

Phase 4 of WIA-IND-012 defines platform integrations with major health ecosystems including Apple Health, Google Fit, Strava, health insurance systems, and medical record platforms. The **弘益人間** principle ensures integrations benefit global health through comprehensive data connectivity while maintaining privacy and user control.

### 1.1 Scope

- Apple Health (HealthKit) integration
- Google Fit integration
- Strava and sports platforms
- Health insurance integrations
- Electronic Health Record (EHR) systems
- Clinical decision support systems
- Third-party app ecosystem

---

## 2. Apple Health Integration

### 2.1 HealthKit Framework

**Required Capabilities:**
```swift
import HealthKit

let healthStore = HKHealthStore()

// Request authorization
let typesToRead: Set<HKSampleType> = [
    HKObjectType.quantityType(forIdentifier: .heartRate)!,
    HKObjectType.quantityType(forIdentifier: .stepCount)!,
    HKObjectType.categoryType(forIdentifier: .sleepAnalysis)!
]

let typesToWrite: Set<HKSampleType> = [
    HKObjectType.quantityType(forIdentifier: .heartRate)!,
    HKObjectType.quantityType(forIdentifier: .stepCount)!,
    HKObjectType.workoutType()
]

healthStore.requestAuthorization(toShare: typesToWrite, read: typesToRead) { success, error in
    // Handle authorization
}
```

### 2.2 Data Type Mapping

| WIA-IND-012 Type | HealthKit Type | Unit |
|------------------|----------------|------|
| heart_rate | HKQuantityTypeIdentifierHeartRate | count/min |
| steps | HKQuantityTypeIdentifierStepCount | count |
| distance | HKQuantityTypeIdentifierDistanceWalkingRunning | km |
| calories | HKQuantityTypeIdentifierActiveEnergyBurned | kcal |
| sleep | HKCategoryTypeIdentifierSleepAnalysis | N/A |
| vo2max | HKQuantityTypeIdentifierVO2Max | mL/(kg·min) |

### 2.3 Writing Heart Rate Data

```swift
func writeHeartRate(bpm: Int, date: Date) {
    let heartRateType = HKQuantityType.quantityType(
        forIdentifier: .heartRate
    )!

    let heartRateQuantity = HKQuantity(
        unit: HKUnit.count().unitDivided(by: .minute()),
        doubleValue: Double(bpm)
    )

    let heartRateSample = HKQuantitySample(
        type: heartRateType,
        quantity: heartRateQuantity,
        start: date,
        end: date,
        metadata: [
            "source": "WIA-IND-012",
            "philosophy": "弘益人間"
        ]
    )

    healthStore.save(heartRateSample) { success, error in
        // Handle result
    }
}
```

### 2.4 Workout Integration

```swift
func saveWorkout(type: HKWorkoutActivityType, start: Date, end: Date,
                 distance: Double, calories: Double) {
    let workout = HKWorkout(
        activityType: type,
        start: start,
        end: end,
        duration: end.timeIntervalSince(start),
        totalEnergyBurned: HKQuantity(unit: .kilocalorie(), doubleValue: calories),
        totalDistance: HKQuantity(unit: .kilometer(), doubleValue: distance),
        metadata: [
            "standard": "WIA-IND-012",
            "philosophy": "弘益人間"
        ]
    )

    healthStore.save(workout) { success, error in
        // Handle result
    }
}
```

---

## 3. Google Fit Integration

### 3.1 Google Fit API Setup

**Dependencies (Android):**
```gradle
dependencies {
    implementation 'com.google.android.gms:play-services-fitness:21.0.1'
    implementation 'com.google.android.gms:play-services-auth:20.0.1'
}
```

**Permissions:**
```xml
<uses-permission android:name="android.permission.ACTIVITY_RECOGNITION" />
```

### 3.2 OAuth 2.0 Authentication

```kotlin
val fitnessOptions = FitnessOptions.builder()
    .addDataType(DataType.TYPE_HEART_RATE_BPM, FitnessOptions.ACCESS_WRITE)
    .addDataType(DataType.TYPE_STEP_COUNT_DELTA, FitnessOptions.ACCESS_WRITE)
    .addDataType(DataType.TYPE_SLEEP_SEGMENT, FitnessOptions.ACCESS_WRITE)
    .addDataType(DataType.TYPE_ACTIVITY_SEGMENT, FitnessOptions.ACCESS_WRITE)
    .build()

val account = GoogleSignIn.getAccountForExtension(context, fitnessOptions)

if (!GoogleSignIn.hasPermissions(account, fitnessOptions)) {
    GoogleSignIn.requestPermissions(
        activity,
        GOOGLE_FIT_PERMISSIONS_REQUEST_CODE,
        account,
        fitnessOptions
    )
}
```

### 3.3 Data Type Mapping

| WIA-IND-012 Type | Google Fit Type | Unit |
|------------------|-----------------|------|
| heart_rate | TYPE_HEART_RATE_BPM | bpm |
| steps | TYPE_STEP_COUNT_DELTA | count |
| distance | TYPE_DISTANCE_DELTA | meters |
| calories | TYPE_CALORIES_EXPENDED | kcal |
| sleep | TYPE_SLEEP_SEGMENT | N/A |
| weight | TYPE_WEIGHT | kg |

### 3.4 Writing Data to Google Fit

```kotlin
fun insertHeartRate(bpm: Float, timestamp: Long) {
    val dataSource = DataSource.Builder()
        .setAppPackageName(context.packageName)
        .setDataType(DataType.TYPE_HEART_RATE_BPM)
        .setStreamName("WIA-IND-012 - Heart Rate")
        .setType(DataSource.TYPE_RAW)
        .build()

    val dataPoint = DataPoint.builder(dataSource)
        .setTimeInterval(timestamp, timestamp, TimeUnit.MILLISECONDS)
        .setField(Field.FIELD_BPM, bpm)
        .build()

    val dataSet = DataSet.builder(dataSource)
        .add(dataPoint)
        .build()

    Fitness.getHistoryClient(context, account)
        .insertData(dataSet)
        .addOnSuccessListener {
            Log.d("WIA", "Data inserted: $bpm BPM - 弘益人間")
        }
}
```

### 3.5 Session (Workout) Integration

```kotlin
fun insertSession(activityType: Int, startTime: Long, endTime: Long) {
    val session = Session.Builder()
        .setName("WIA-IND-012 Workout")
        .setIdentifier("wia-session-${System.currentTimeMillis()}")
        .setDescription("弘益人間 - Fitness Session")
        .setActivity(activityType)
        .setStartTime(startTime, TimeUnit.MILLISECONDS)
        .setEndTime(endTime, TimeUnit.MILLISECONDS)
        .build()

    val request = SessionInsertRequest.Builder()
        .setSession(session)
        .build()

    Fitness.getSessionsClient(context, account)
        .insertSession(request)
}
```

---

## 4. Strava Integration

### 4.1 OAuth 2.0 Flow

```http
GET https://www.strava.com/oauth/authorize?
    client_id={client_id}&
    redirect_uri={redirect_uri}&
    response_type=code&
    scope=activity:write,activity:read_all
```

**Token Exchange:**
```http
POST https://www.strava.com/oauth/token
Content-Type: application/json

{
  "client_id": "{client_id}",
  "client_secret": "{client_secret}",
  "code": "{authorization_code}",
  "grant_type": "authorization_code"
}
```

### 4.2 Upload Activity

```javascript
async function uploadToStrava(activity) {
  const response = await fetch('https://www.strava.com/api/v3/activities', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${accessToken}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      name: activity.name,
      type: activity.type,  // 'Run', 'Ride', 'Swim', etc.
      start_date_local: activity.startTime,
      elapsed_time: activity.duration,
      distance: activity.distance,
      description: '弘益人間 - WIA-IND-012 Activity',
      trainer: false,
      commute: false
    })
  });

  return await response.json();
}
```

### 4.3 Upload GPX Track

```javascript
async function uploadGPXToStrava(gpxFile, activity) {
  const formData = new FormData();
  formData.append('file', gpxFile);
  formData.append('name', activity.name);
  formData.append('description', 'WIA-IND-012 - 弘益人間');
  formData.append('trainer', 'false');
  formData.append('commute', 'false');
  formData.append('data_type', 'gpx');
  formData.append('activity_type', activity.type);

  const response = await fetch('https://www.strava.com/api/v3/uploads', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${accessToken}`
    },
    body: formData
  });

  return await response.json();
}
```

---

## 5. Samsung Health Integration

### 5.1 Samsung Health SDK

```kotlin
// Initialize Samsung Health
val healthDataStore = HealthDataStore(context) { error ->
    if (error != null) {
        Log.e("WIA", "Samsung Health init failed: $error")
    }
}

// Request permissions
val permissionSet = HashSet<PermissionKey>()
permissionSet.add(PermissionKey(HealthConstants.StepCount.HEALTH_DATA_TYPE, PermissionType.READ))
permissionSet.add(PermissionKey(HealthConstants.HeartRate.HEALTH_DATA_TYPE, PermissionType.WRITE))

healthDataStore.connectService()
```

### 5.2 Write Heart Rate Data

```kotlin
fun writeHeartRate(bpm: Float, timestamp: Long) {
    val builder = HealthData.Builder(HealthConstants.HeartRate.HEALTH_DATA_TYPE)
    builder.setTime(timestamp)
    builder.set(HealthConstants.HeartRate.HEART_RATE, bpm)
    builder.setSourcePackage(context.packageName)

    val heartRateData = builder.build()

    healthDataStore.insertData(heartRateData) { error ->
        if (error == null) {
            Log.d("WIA", "Heart rate inserted: $bpm - 弘益人間")
        }
    }
}
```

---

## 6. Health Insurance Integration

### 6.1 Wellness Program APIs

**Standard Wellness Data Format:**
```json
{
  "program_id": "wellness-2025-001",
  "participant_id": "participant-123456",
  "reporting_period": {
    "start": "2025-01-01",
    "end": "2025-01-31"
  },
  "metrics": {
    "average_daily_steps": 8500,
    "active_days": 25,
    "total_exercise_minutes": 480,
    "goals_achieved": 4,
    "wellness_score": 85
  },
  "achievements": [
    {
      "type": "10k_steps_streak",
      "days": 14,
      "achieved_date": "2025-01-15"
    }
  ],
  "philosophy": "弘益人間",
  "attestation": {
    "device_verified": true,
    "data_integrity": true,
    "timestamp": "2025-01-31T23:59:59.000Z"
  }
}
```

### 6.2 Privacy and Compliance

**HIPAA Compliance Requirements:**
- Business Associate Agreement (BAA) required
- Data encryption at rest and in transit (AES-256, TLS 1.3)
- Access logging and audit trails
- User consent management
- De-identification for analytics
- Breach notification procedures

**GDPR Compliance:**
- Explicit consent required
- Right to access, rectify, delete data
- Data portability
- Privacy by design and default
- Data processing agreements

### 6.3 Insurance Provider Integration Example

```javascript
class InsuranceIntegration {
  constructor(providerId, apiKey) {
    this.providerId = providerId;
    this.apiKey = apiKey;
    this.baseUrl = 'https://api.insurance-provider.com/v1';
  }

  async submitWellnessData(userId, period, metrics) {
    const payload = {
      provider_id: this.providerId,
      user_id: userId,
      period: period,
      metrics: metrics,
      standard: 'WIA-IND-012',
      philosophy: '弘益人間',
      timestamp: new Date().toISOString()
    };

    const response = await fetch(`${this.baseUrl}/wellness/submit`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'IND-012'
      },
      body: JSON.stringify(payload)
    });

    return await response.json();
  }
}
```

---

## 7. Electronic Health Record (EHR) Integration

### 7.1 HL7 FHIR Standard

**FHIR Observation Resource (Heart Rate):**
```json
{
  "resourceType": "Observation",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/observation-category",
      "code": "vital-signs",
      "display": "Vital Signs"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "8867-4",
      "display": "Heart rate"
    }],
    "text": "Heart Rate - WIA-IND-012 弘益人間"
  },
  "subject": {
    "reference": "Patient/123456"
  },
  "effectiveDateTime": "2025-01-15T10:30:00Z",
  "valueQuantity": {
    "value": 75,
    "unit": "beats/minute",
    "system": "http://unitsofmeasure.org",
    "code": "/min"
  },
  "device": {
    "reference": "Device/wia-device-001",
    "display": "WIA Fitness Wearable"
  },
  "meta": {
    "tag": [{
      "system": "http://wia.org/fhir/tags",
      "code": "IND-012",
      "display": "WIA-IND-012 Standard"
    }]
  }
}
```

### 7.2 FHIR Activity Resource

```json
{
  "resourceType": "Observation",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/observation-category",
      "code": "activity",
      "display": "Activity"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "55423-8",
      "display": "Number of steps"
    }]
  },
  "subject": {
    "reference": "Patient/123456"
  },
  "effectiveDateTime": "2025-01-15",
  "valueQuantity": {
    "value": 10247,
    "unit": "steps",
    "system": "http://unitsofmeasure.org",
    "code": "{steps}"
  },
  "note": [{
    "text": "弘益人間 - Data from WIA-IND-012 compliant device"
  }]
}
```

### 7.3 FHIR API Integration

```python
import requests
from fhirclient import client

# Initialize FHIR client
settings = {
    'app_id': 'wia-fitness-integration',
    'api_base': 'https://fhir.hospital.org/api',
    'philosophy': '弘益人間'
}

smart = client.FHIRClient(settings=settings)

# Create and upload observation
observation = {
    'resourceType': 'Observation',
    'status': 'final',
    'code': {'coding': [{'system': 'http://loinc.org', 'code': '8867-4'}]},
    'subject': {'reference': 'Patient/123456'},
    'effectiveDateTime': '2025-01-15T10:30:00Z',
    'valueQuantity': {'value': 75, 'unit': 'beats/minute'}
}

response = smart.server.post_json('Observation', observation)
```

---

## 8. Clinical Decision Support Systems

### 8.1 CDS Hooks Integration

**Hook: patient-view**
```json
{
  "hookInstance": "hook-instance-123",
  "hook": "patient-view",
  "context": {
    "userId": "Practitioner/123",
    "patientId": "Patient/456",
    "encounterId": "Encounter/789"
  },
  "prefetch": {
    "wearableData": {
      "resourceType": "Bundle",
      "entry": [{
        "resource": {
          "resourceType": "Observation",
          "code": {"coding": [{"code": "8867-4"}]},
          "valueQuantity": {"value": 95}
        }
      }]
    }
  }
}
```

**CDS Card Response:**
```json
{
  "cards": [{
    "summary": "Elevated Resting Heart Rate - WIA-IND-012",
    "detail": "Patient's average resting HR from wearable: 95 BPM (baseline: 65 BPM). Consider evaluation for infection, dehydration, or cardiac issues. 弘益人間",
    "indicator": "warning",
    "source": {
      "label": "WIA Fitness Wearable Integration",
      "url": "https://wia.org/standards/IND-012"
    },
    "suggestions": [{
      "label": "Order cardiac workup",
      "actions": [{
        "type": "create",
        "description": "Order ECG and cardiac biomarkers",
        "resource": { /* FHIR ServiceRequest */ }
      }]
    }]
  }]
}
```

---

## 9. Third-Party App Ecosystem

### 9.1 OAuth 2.0 Scopes

```
Scope Definitions:
- read:profile         : Read user profile
- write:profile        : Update user profile
- read:activities      : Read activity data
- write:activities     : Create activity sessions
- read:heart_rate      : Read heart rate data
- read:sleep          : Read sleep data
- write:goals         : Create/update goals
- read:all            : Read all data types
```

### 9.2 Developer Platform

**App Registration:**
```json
{
  "app_name": "AI Fitness Coach",
  "developer": "Example Corp",
  "redirect_uris": [
    "https://example.com/oauth/callback"
  ],
  "scopes": ["read:activities", "read:heart_rate", "write:goals"],
  "philosophy": "弘益人間",
  "privacy_policy_url": "https://example.com/privacy",
  "terms_of_service_url": "https://example.com/terms"
}
```

**Response:**
```json
{
  "client_id": "wia_app_abc123",
  "client_secret": "secret_xyz789",
  "created_at": "2025-01-15T10:00:00.000Z",
  "status": "active"
}
```

---

## 10. Cross-Platform Data Portability

### 10.1 Universal Export Format

```json
{
  "export_format": "WIA-IND-012-UNIVERSAL",
  "version": "1.0",
  "philosophy": "弘益人間",
  "user": {
    "user_id_hash": "sha256_hash_here",
    "export_date": "2025-01-15T00:00:00.000Z"
  },
  "data": {
    "profile": { /* User profile */ },
    "activities": [ /* Activity sessions */ ],
    "heart_rate": [ /* HR measurements */ ],
    "sleep": [ /* Sleep sessions */ ],
    "goals": [ /* User goals */ ]
  },
  "metadata": {
    "total_records": 125840,
    "date_range": {
      "start": "2024-01-01",
      "end": "2024-12-31"
    },
    "devices": [ /* Device list */ ]
  }
}
```

### 10.2 Import into Other Platforms

```python
def import_wia_data(export_file, target_platform):
    """
    Import WIA-IND-012 data into various platforms
    弘益人間 - Ensuring data portability for all
    """
    with open(export_file) as f:
        wia_data = json.load(f)

    if target_platform == 'apple_health':
        return import_to_healthkit(wia_data)
    elif target_platform == 'google_fit':
        return import_to_google_fit(wia_data)
    elif target_platform == 'strava':
        return import_to_strava(wia_data)
    else:
        raise ValueError(f"Unsupported platform: {target_platform}")
```

---

## 11. Testing and Certification

### 11.1 Integration Testing

**Test Checklist:**
- [ ] OAuth 2.0 authorization flow
- [ ] Data format compliance
- [ ] API rate limit handling
- [ ] Error handling and retry logic
- [ ] Privacy compliance (GDPR, HIPAA)
- [ ] Data accuracy verification
- [ ] Cross-platform interoperability

### 11.2 Certification Process

1. Submit integration documentation
2. Complete security audit
3. Pass automated test suite
4. Demonstrate data privacy compliance
5. Receive WIA-IND-012 integration certification
6. Display certification badge in app/marketing

---

## 12. Conclusion

Phase 4 of WIA-IND-012 enables comprehensive platform integrations, ensuring fitness wearable data benefits users across diverse health ecosystems. By standardizing integrations with major platforms, insurance programs, and clinical systems, we fulfill the **弘益人間** vision of technology serving all humanity's health and wellness.

---

**Document Control**

- Version: 1.0
- Effective Date: 2025-01-15
- Next Review: 2026-01-15
- Contact: standards@wia.org

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
