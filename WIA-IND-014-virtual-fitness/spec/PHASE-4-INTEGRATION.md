# WIA-IND-014: Virtual Fitness Standard
## Phase 4: Integration Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-27
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document defines integration protocols for virtual fitness applications with major XR platforms, fitness services, wearable devices, and health ecosystems. The goal is seamless interoperability enabling users to mix and match hardware, software, and services while maintaining a cohesive experience.

## 2. XR Platform Integration

### 2.1 Meta Quest Integration

#### SDK Setup

```javascript
// Meta Quest SDK initialization
import { OVRPlugin } from 'oculus-sdk';

const questConfig = {
  appId: 'your-meta-app-id',
  features: [
    'hand-tracking',
    'body-tracking',
    'passthrough',
    'spatial-audio'
  ],
  philosophy: '弘益人間'
};

await OVRPlugin.initialize(questConfig);

// Access Quest-specific features
const handTracking = await OVRPlugin.enableHandTracking();
const bodyTracking = await OVRPlugin.enableBodyTracking();
```

#### Guardian System Integration

```javascript
class GuardianManager {
  async setupPlaySpace() {
    const boundary = await OVRPlugin.getBoundary();

    if (!boundary || boundary.area < 2.0) {
      // Minimum 2m² play space required
      this.showBoundaryWarning();
      return false;
    }

    // Configure safety zones
    this.configureZones(boundary);
    return true;
  }

  configureZones(boundary) {
    // Center zone for stationary exercises
    this.centerZone = {
      center: boundary.center,
      radius: 1.0
    };

    // Movement zone with safety margin
    this.movementZone = {
      bounds: boundary.points,
      margin: 0.3 // 30cm safety margin
    };
  }

  onBoundaryProximity(distance) {
    if (distance < 0.5) {
      // Show visual warning
      this.showProximityAlert(distance);

      // Reduce workout intensity
      this.pauseWorkout();
    }
  }
}
```

### 2.2 Apple Vision Pro Integration

#### visionOS App Setup

```swift
import RealityKit
import ARKit
import HealthKit

@main
struct VirtualFitnessApp: App {
    var body: some Scene {
        WindowGroup {
            ImmersiveSpace(id: "FitnessSpace") {
                FitnessEnvironment()
            }
            .immersionStyle(selection: .constant(.full))
        }
    }
}

// Body tracking with Vision Pro
class BodyTrackingManager: ObservableObject {
    private var arSession = ARKitSession()
    private var bodyTracking = BodyTrackingProvider()

    func startTracking() async {
        do {
            try await arSession.run([bodyTracking])

            for await update in bodyTracking.anchorUpdates {
                // Process 33-point skeleton data
                self.processSkeleton(update.anchor)
            }
        } catch {
            print("Body tracking failed: \\(error)")
        }
    }

    func processSkeleton(_ anchor: BodyAnchor) {
        let wiaKeypoints = self.convertToWIAFormat(anchor.skeleton)
        // Send to WIA-IND-014 backend
        self.uploadPoseData(wiaKeypoints)
    }
}
```

#### HealthKit Integration

```swift
class HealthKitBridge {
    let healthStore = HKHealthStore()

    func requestAuthorization() async throws {
        let types: Set<HKSampleType> = [
            HKQuantityType(.heartRate),
            HKQuantityType(.activeEnergyBurned),
            HKQuantityType(.distanceWalkingRunning)
        ]

        try await healthStore.requestAuthorization(
            toShare: types,
            read: types
        )
    }

    func writeWorkout(_ session: WorkoutSession) async throws {
        let workout = HKWorkout(
            activityType: .virtualReality,
            start: session.startDate,
            end: session.endDate,
            duration: session.duration,
            totalEnergyBurned: HKQuantity(
                unit: .kilocalorie(),
                doubleValue: session.calories
            ),
            totalDistance: nil,
            metadata: [
                "WIA-IND-014": true,
                "workoutType": session.type,
                "philosophy": "弘益人間"
            ]
        )

        try await healthStore.save(workout)
    }
}
```

### 2.3 PlayStation VR2 Integration

#### PSVR2 SDK Setup

```cpp
#include <psvr2/psvr2_api.h>

class PSVR2Manager {
public:
    bool initialize() {
        PsvrConfig config;
        config.appId = "your-psvr-app-id";
        config.features = PSVR_FEATURE_EYE_TRACKING |
                         PSVR_FEATURE_HEADSET_TRACKING |
                         PSVR_FEATURE_HAPTIC_FEEDBACK;

        return psvrInitialize(&config) == PSVR_SUCCESS;
    }

    void updateTracking() {
        PsvrTrackingData tracking;
        psvrGetTrackingData(&tracking);

        // Convert to WIA-IND-014 format
        WIAPoseData poseData = convertToWIA(tracking);
        uploadPoseData(poseData);
    }

    void triggerHaptics(WIAHapticCommand command) {
        PsvrHapticData haptics;
        haptics.intensity = command.intensity;
        haptics.duration = command.duration;
        haptics.pattern = convertPattern(command.pattern);

        psvrTriggerHaptics(&haptics);
    }
};
```

## 3. Fitness Platform Integration

### 3.1 Peloton Integration

```javascript
class PelotonBridge {
  constructor(apiKey) {
    this.apiKey = apiKey;
    this.baseURL = 'https://api.onepeloton.com';
  }

  async syncWorkout(wiaSession) {
    // Convert WIA session to Peloton format
    const pelotonWorkout = {
      title: wiaSession.name,
      type: this.mapWorkoutType(wiaSession.type),
      start_time: wiaSession.startTime,
      end_time: wiaSession.endTime,
      total_work: wiaSession.metrics.calories * 4184, // kJ
      device_type: 'vr_headset',
      metrics: {
        avg_heart_rate: wiaSession.metrics.avgHeartRate,
        max_heart_rate: wiaSession.metrics.maxHeartRate,
        calories: wiaSession.metrics.calories
      }
    };

    const response = await fetch(`${this.baseURL}/api/workouts`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(pelotonWorkout)
    });

    return response.json();
  }

  mapWorkoutType(wiaType) {
    const typeMap = {
      'vr-boxing': 'cardio',
      'vr-cycling': 'cycling',
      'virtual-yoga': 'yoga',
      'vr-running': 'running'
    };
    return typeMap[wiaType] || 'other';
  }
}
```

### 3.2 Apple Fitness+ Integration

```swift
class AppleFitnessIntegration {
    func exportToFitness(_ session: WIAWorkoutSession) async {
        // Create Fitness+ compatible workout
        let workout = AFWorkout(
            type: .virtualReality,
            startDate: session.startDate,
            endDate: session.endDate
        )

        // Add metrics
        workout.addMetrics([
            AFMetric(type: .heartRate, samples: session.heartRateSamples),
            AFMetric(type: .activeCalories, value: session.calories),
            AFMetric(type: .duration, value: session.duration)
        ])

        // Sync with Fitness app
        try? await AFManager.shared.save(workout)
    }
}
```

### 3.3 Mirror Integration

```javascript
const MirrorIntegration = {
  async connectDevice() {
    // Discover Mirror on local network
    const mirrors = await this.discoverMirrors();

    if (mirrors.length === 0) {
      throw new Error('No Mirror devices found');
    }

    // Connect to first available Mirror
    return await this.connect(mirrors[0]);
  },

  async startMirrorClass(classId, wiaSession) {
    // Stream WIA workout to Mirror display
    const stream = {
      classId: classId,
      instructor: wiaSession.instructor,
      duration: wiaSession.duration,
      metrics: {
        heartRate: true,
        calories: true,
        form: true
      }
    };

    return await this.mirror.startStream(stream);
  }
};
```

## 4. Wearable Device Integration

### 4.1 Apple Watch Integration

```swift
import WatchConnectivity

class AppleWatchBridge: NSObject, WCSessionDelegate {
    let session = WCSession.default

    func startSession() {
        if WCSession.isSupported() {
            session.delegate = self
            session.activate()
        }
    }

    // Send workout session to Watch
    func sendWorkoutToWatch(_ wiaSession: WIAWorkoutSession) {
        let message = [
            "action": "start_workout",
            "type": wiaSession.type,
            "duration": wiaSession.duration,
            "philosophy": "弘益人間"
        ]

        session.sendMessage(message, replyHandler: nil)
    }

    // Receive heart rate from Watch
    func session(_ session: WCSession,
                 didReceiveMessage message: [String: Any]) {
        if let heartRate = message["heartRate"] as? Int {
            self.updateHeartRate(heartRate)
        }
    }
}
```

### 4.2 Fitbit Integration

```javascript
class FitbitBridge {
  constructor(accessToken) {
    this.accessToken = accessToken;
    this.baseURL = 'https://api.fitbit.com';
  }

  async getHeartRate() {
    const response = await fetch(
      `${this.baseURL}/1/user/-/activities/heart/date/today/1d/1sec.json`,
      {
        headers: {
          'Authorization': `Bearer ${this.accessToken}`
        }
      }
    );

    const data = await response.json();
    return this.parseHeartRateData(data);
  }

  async logWorkout(wiaSession) {
    const fitbitActivity = {
      activityId: 90019, // Virtual Reality
      startTime: wiaSession.startTime,
      durationMillis: wiaSession.duration * 1000,
      distance: wiaSession.distance,
      calories: wiaSession.calories
    };

    await fetch(`${this.baseURL}/1/user/-/activities.json`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.accessToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(fitbitActivity)
    });
  }
}
```

### 4.3 Garmin Integration

```javascript
const GarminConnectIQ = {
  // Connect IQ app data format
  createActivity(wiaSession) {
    return {
      sport: 'VIRTUAL_ACTIVITY',
      start_time: wiaSession.startTime,
      total_elapsed_time: wiaSession.duration,
      total_timer_time: wiaSession.activeTime,
      avg_heart_rate: wiaSession.metrics.avgHeartRate,
      max_heart_rate: wiaSession.metrics.maxHeartRate,
      calories: wiaSession.metrics.calories,
      device: {
        manufacturer: 'WIA',
        product: 'VR-Fitness',
        version: '1.0',
        philosophy: '弘益人間'
      }
    };
  },

  async uploadToGarmin(activity) {
    // Upload via Garmin Connect API
    const response = await fetch('https://connectapi.garmin.com/activity-service/activity', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.garminToken}`
      },
      body: JSON.stringify(activity)
    });

    return response.json();
  }
};
```

## 5. Smart Equipment Integration

### 5.1 ANT+ Device Support

```javascript
class ANTBridge {
  async connectHeartRateMonitor() {
    // Connect to ANT+ HR monitor
    const device = await navigator.bluetooth.requestDevice({
      filters: [{ services: ['heart_rate'] }],
      optionalServices: ['battery_service']
    });

    const server = await device.gatt.connect();
    const service = await server.getPrimaryService('heart_rate');
    const characteristic = await service.getCharacteristic('heart_rate_measurement');

    // Listen for heart rate updates
    await characteristic.startNotifications();
    characteristic.addEventListener('characteristicvaluechanged', (event) => {
      const heartRate = this.parseHeartRate(event.target.value);
      this.updateWorkoutMetrics({ heartRate });
    });
  }

  parseHeartRate(value) {
    const flags = value.getUint8(0);
    const rate = (flags & 0x01) ? value.getUint16(1, true) : value.getUint8(1);
    return rate;
  }
}
```

### 5.2 Smart Resistance Equipment

```javascript
class SmartEquipmentBridge {
  async connectSmartDumbbell() {
    // Connect via Bluetooth
    const device = await navigator.bluetooth.requestDevice({
      filters: [{ namePrefix: 'SmartBell' }],
      optionalServices: ['weight_service']
    });

    const server = await device.gatt.connect();

    // Get current weight setting
    const weightService = await server.getPrimaryService('weight_service');
    const weightChar = await weightService.getCharacteristic('current_weight');

    weightChar.addEventListener('characteristicvaluechanged', (event) => {
      const weight = event.target.value.getFloat32(0, true);
      const reps = event.target.value.getUint16(4, true);
      const tempo = event.target.value.getUint8(6);

      this.logRepetition({ weight, reps, tempo });
    });

    await weightChar.startNotifications();
  }
}
```

## 6. Health Ecosystem Integration

### 6.1 Google Fit Integration

```javascript
const GoogleFitBridge = {
  async initialize() {
    await gapi.client.init({
      apiKey: 'YOUR_API_KEY',
      clientId: 'YOUR_CLIENT_ID',
      discoveryDocs: ['https://www.googleapis.com/discovery/v1/apis/fitness/v1/rest'],
      scope: 'https://www.googleapis.com/auth/fitness.activity.write'
    });
  },

  async uploadWorkout(wiaSession) {
    const dataSource = {
      dataStreamName: 'WIA-IND-014 Virtual Fitness',
      type: 'raw',
      application: {
        packageName: 'com.wia.virtualfitness',
        version: '1.0'
      },
      dataType: {
        name: 'com.google.activity.segment'
      }
    };

    const dataset = {
      dataSourceId: await this.createDataSource(dataSource),
      minStartTimeNs: wiaSession.startTime * 1000000,
      maxEndTimeNs: wiaSession.endTime * 1000000,
      point: [{
        startTimeNanos: wiaSession.startTime * 1000000,
        endTimeNanos: wiaSession.endTime * 1000000,
        dataTypeName: 'com.google.activity.segment',
        value: [{
          intVal: this.mapActivityType(wiaSession.type)
        }]
      }]
    };

    return await gapi.client.fitness.users.dataSources.datasets.patch({
      userId: 'me',
      dataSourceId: dataset.dataSourceId,
      datasetId: `${dataset.minStartTimeNs}-${dataset.maxEndTimeNs}`,
      resource: dataset
    });
  }
};
```

### 6.2 Samsung Health Integration

```kotlin
class SamsungHealthBridge(context: Context) {
    private val healthConnector = HealthConnectionManager(context)

    fun writeWorkout(wiaSession: WIAWorkoutSession) {
        val sessionBuilder = ExerciseInfo.Builder()
            .setExerciseType(ExerciseInfo.EXERCISE_VR)
            .setStartTime(wiaSession.startTime)
            .setEndTime(wiaSession.endTime)
            .setCalorie(wiaSession.calories)
            .setComment("WIA-IND-014 弘益人間")

        healthConnector.insert(sessionBuilder.build())
    }

    fun readHeartRate(callback: (Int) -> Unit) {
        val request = HealthDataResolver.ReadRequest.Builder()
            .setDataType(HealthConstants.HeartRate.HEALTH_DATA_TYPE)
            .build()

        healthConnector.readData(request) { result ->
            result.forEach { data ->
                val heartRate = data.getInt(HealthConstants.HeartRate.HEART_RATE)
                callback(heartRate)
            }
        }
    }
}
```

## 7. Cloud Sync and Backup

### 7.1 Multi-Platform Sync

```javascript
class CloudSyncManager {
  async syncToCloud(wiaData) {
    // Encrypt data before upload
    const encrypted = await this.encrypt(wiaData);

    // Upload to user's preferred cloud
    const providers = ['iCloud', 'Google Drive', 'OneDrive'];

    for (const provider of providers) {
      if (this.isEnabled(provider)) {
        await this.uploadTo(provider, encrypted);
      }
    }
  }

  async restoreFromCloud() {
    // Download from primary cloud provider
    const encrypted = await this.downloadFrom(this.primaryProvider);

    // Decrypt and parse
    const wiaData = await this.decrypt(encrypted);

    // Validate WIA-IND-014 format
    if (wiaData.standard !== 'WIA-IND-014') {
      throw new Error('Invalid data format');
    }

    return wiaData;
  }
}
```

## 8. Testing and Certification

### 8.1 Integration Test Suite

```javascript
describe('WIA-IND-014 Integration Tests', () => {
  test('Meta Quest integration', async () => {
    const quest = new MetaQuestBridge();
    await quest.initialize();

    const session = await quest.startWorkout({
      type: 'vr-boxing',
      duration: 1800
    });

    expect(session.sessionId).toBeDefined();
    expect(session.trackingMode).toBe('full-body');
  });

  test('HealthKit sync', async () => {
    const healthKit = new HealthKitBridge();
    await healthKit.requestAuthorization();

    const workout = createMockWorkout();
    await healthKit.writeWorkout(workout);

    const saved = await healthKit.readWorkout(workout.id);
    expect(saved.calories).toBe(workout.calories);
  });
});
```

---

**Document Version:** 1.0
**Status:** Draft for Public Comment
**Contact:** standards@wia.org
**License:** CC BY-SA 4.0

弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-014-virtual-fitness is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-014-virtual-fitness/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-014-virtual-fitness/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-014-virtual-fitness/simulator/` — interactive browser-based simulator for the PHASE protocol

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
