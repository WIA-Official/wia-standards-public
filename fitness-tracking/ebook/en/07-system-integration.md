# Chapter 7: System Integration

## Overview

This chapter provides detailed guidance for integrating WIA-IND-012 compliant systems with major health platforms, wearable devices, and third-party services. Learn how to connect your implementation to the broader fitness tracking ecosystem.

---

## 7.1 iOS HealthKit Integration

### 7.1.1 Overview

Apple HealthKit is the central health data repository for iOS devices, aggregating data from various apps and devices.

**Key Features:**
- Unified health data store
- Privacy-first design with user permission
- Background data syncing
- Support for 100+ data types

### 7.1.2 Setup and Permissions

**Info.plist Configuration:**
```xml
<key>NSHealthShareUsageDescription</key>
<string>We need access to your fitness data to track your activities and provide personalized insights.</string>

<key>NSHealthUpdateUsageDescription</key>
<string>We need permission to save your workout data to HealthKit.</string>

<key>UIBackgroundModes</key>
<array>
    <string>processing</string>
</array>
```

**Request Permissions:**
```swift
import HealthKit

let healthStore = HKHealthStore()

// Define data types to read
let typesToRead: Set<HKObjectType> = [
    HKObjectType.quantityType(forIdentifier: .stepCount)!,
    HKObjectType.quantityType(forIdentifier: .distanceWalkingRunning)!,
    HKObjectType.quantityType(forIdentifier: .heartRate)!,
    HKObjectType.quantityType(forIdentifier: .activeEnergyBurned)!,
    HKObjectType.workoutType()
]

// Define data types to write
let typesToWrite: Set<HKSampleType> = [
    HKObjectType.quantityType(forIdentifier: .stepCount)!,
    HKObjectType.quantityType(forIdentifier: .activeEnergyBurned)!,
    HKObjectType.workoutType()
]

// Request authorization
healthStore.requestAuthorization(toShare: typesToWrite, read: typesToRead) { success, error in
    if success {
        print("HealthKit authorization granted")
    } else {
        print("HealthKit authorization failed: \(error?.localizedDescription ?? "Unknown error")")
    }
}
```

### 7.1.3 Reading Data

**Query Step Count:**
```swift
func queryStepCount(startDate: Date, endDate: Date, completion: @escaping (Double?) -> Void) {
    guard let stepType = HKQuantityType.quantityType(forIdentifier: .stepCount) else {
        completion(nil)
        return
    }

    let predicate = HKQuery.predicateForSamples(withStart: startDate, end: endDate, options: .strictStartDate)

    let query = HKStatisticsQuery(quantityType: stepType, quantitySamplePredicate: predicate, options: .cumulativeSum) { _, result, error in
        guard let result = result, let sum = result.sumQuantity() else {
            completion(nil)
            return
        }

        let steps = sum.doubleValue(for: HKUnit.count())
        completion(steps)
    }

    healthStore.execute(query)
}
```

**Query Workouts:**
```swift
func queryWorkouts(completion: @escaping ([HKWorkout]?) -> Void) {
    let workoutType = HKObjectType.workoutType()
    let sortDescriptor = NSSortDescriptor(key: HKSampleSortIdentifierStartDate, ascending: false)

    let query = HKSampleQuery(sampleType: workoutType, predicate: nil, limit: 20, sortDescriptors: [sortDescriptor]) { _, samples, error in
        guard let workouts = samples as? [HKWorkout] else {
            completion(nil)
            return
        }

        completion(workouts)
    }

    healthStore.execute(query)
}
```

### 7.1.4 Writing Data

**Save Workout:**
```swift
func saveWorkout(type: HKWorkoutActivityType, start: Date, end: Date, distance: Double, calories: Double) {
    let workout = HKWorkout(
        activityType: type,
        start: start,
        end: end,
        duration: end.timeIntervalSince(start),
        totalEnergyBurned: HKQuantity(unit: .kilocalorie(), doubleValue: calories),
        totalDistance: HKQuantity(unit: .meter(), doubleValue: distance),
        metadata: nil
    )

    healthStore.save(workout) { success, error in
        if success {
            print("Workout saved successfully")
        } else {
            print("Error saving workout: \(error?.localizedDescription ?? "Unknown error")")
        }
    }
}
```

### 7.1.5 Mapping WIA-IND-012 to HealthKit

```swift
// Activity Type Mapping
func mapToHealthKitActivityType(_ wiaType: String) -> HKWorkoutActivityType {
    switch wiaType {
    case "running":
        return .running
    case "walking":
        return .walking
    case "cycling":
        return .cycling
    case "swimming":
        return .swimming
    case "weight_training":
        return .traditionalStrengthTraining
    case "yoga":
        return .yoga
    case "hiit":
        return .highIntensityIntervalTraining
    default:
        return .other
    }
}

// Convert WIA Workout to HealthKit
func convertToHealthKit(wiaWorkout: Workout) -> HKWorkout {
    let activityType = mapToHealthKitActivityType(wiaWorkout.type)

    return HKWorkout(
        activityType: activityType,
        start: wiaWorkout.startTime,
        end: wiaWorkout.endTime,
        duration: TimeInterval(wiaWorkout.duration),
        totalEnergyBurned: HKQuantity(unit: .kilocalorie(), doubleValue: Double(wiaWorkout.calories)),
        totalDistance: wiaWorkout.distance != nil ? HKQuantity(unit: .meter(), doubleValue: wiaWorkout.distance!) : nil,
        metadata: [
            HKMetadataKeyAverageHeartRate: wiaWorkout.heartRate?.avg ?? 0
        ]
    )
}
```

---

## 7.2 Google Fit Integration

### 7.2.1 Overview

Google Fit is Android's health and fitness platform, providing APIs for data storage and retrieval.

**Key Features:**
- REST and Android APIs
- OAuth 2.0 authentication
- Data types for activities, nutrition, location
- Aggregated data queries

### 7.2.2 Setup

**build.gradle:**
```gradle
dependencies {
    implementation 'com.google.android.gms:play-services-fitness:21.1.0'
    implementation 'com.google.android.gms:play-services-auth:20.7.0'
}
```

**AndroidManifest.xml:**
```xml
<uses-permission android:name="android.permission.ACTIVITY_RECOGNITION" />
<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />
<uses-permission android:name="android.permission.BODY_SENSORS" />
```

### 7.2.3 Authentication

**Request Permissions:**
```kotlin
import com.google.android.gms.auth.api.signin.GoogleSignIn
import com.google.android.gms.fitness.FitnessOptions
import com.google.android.gms.fitness.data.DataType

val fitnessOptions = FitnessOptions.builder()
    .addDataType(DataType.TYPE_STEP_COUNT_DELTA, FitnessOptions.ACCESS_READ)
    .addDataType(DataType.TYPE_STEP_COUNT_DELTA, FitnessOptions.ACCESS_WRITE)
    .addDataType(DataType.TYPE_DISTANCE_DELTA, FitnessOptions.ACCESS_READ)
    .addDataType(DataType.TYPE_CALORIES_EXPENDED, FitnessOptions.ACCESS_READ)
    .addDataType(DataType.TYPE_HEART_RATE_BPM, FitnessOptions.ACCESS_READ)
    .addDataType(DataType.TYPE_ACTIVITY_SEGMENT, FitnessOptions.ACCESS_READ)
    .build()

val account = GoogleSignIn.getAccountForExtension(this, fitnessOptions)

if (!GoogleSignIn.hasPermissions(account, fitnessOptions)) {
    GoogleSignIn.requestPermissions(
        this,
        GOOGLE_FIT_PERMISSIONS_REQUEST_CODE,
        account,
        fitnessOptions
    )
}
```

### 7.2.4 Reading Data

**Query Step Count:**
```kotlin
import com.google.android.gms.fitness.Fitness
import com.google.android.gms.fitness.data.DataType
import com.google.android.gms.fitness.request.DataReadRequest
import java.util.concurrent.TimeUnit

fun readStepCount(startTime: Long, endTime: Long) {
    val readRequest = DataReadRequest.Builder()
        .aggregate(DataType.TYPE_STEP_COUNT_DELTA)
        .bucketByTime(1, TimeUnit.DAYS)
        .setTimeRange(startTime, endTime, TimeUnit.MILLISECONDS)
        .build()

    Fitness.getHistoryClient(this, account)
        .readData(readRequest)
        .addOnSuccessListener { response ->
            for (bucket in response.buckets) {
                for (dataSet in bucket.dataSets) {
                    for (dataPoint in dataSet.dataPoints) {
                        val steps = dataPoint.getValue(Field.FIELD_STEPS).asInt()
                        Log.d("GoogleFit", "Steps: $steps")
                    }
                }
            }
        }
        .addOnFailureListener { e ->
            Log.e("GoogleFit", "Error reading step count", e)
        }
}
```

### 7.2.5 Writing Data

**Insert Activity:**
```kotlin
import com.google.android.gms.fitness.data.DataPoint
import com.google.android.gms.fitness.data.DataSet
import com.google.android.gms.fitness.data.DataSource

fun insertActivity(wiaWorkout: Workout) {
    val dataSource = DataSource.Builder()
        .setAppPackageName(this)
        .setDataType(DataType.TYPE_ACTIVITY_SEGMENT)
        .setType(DataSource.TYPE_RAW)
        .build()

    val activityType = mapToGoogleFitActivity(wiaWorkout.type)

    val dataPoint = DataPoint.builder(dataSource)
        .setTimeInterval(
            wiaWorkout.startTime.time,
            wiaWorkout.endTime.time,
            TimeUnit.MILLISECONDS
        )
        .setActivityField(Field.FIELD_ACTIVITY, activityType)
        .build()

    val dataSet = DataSet.builder(dataSource)
        .add(dataPoint)
        .build()

    Fitness.getHistoryClient(this, account)
        .insertData(dataSet)
        .addOnSuccessListener {
            Log.d("GoogleFit", "Activity inserted successfully")
        }
        .addOnFailureListener { e ->
            Log.e("GoogleFit", "Error inserting activity", e)
        }
}
```

### 7.2.6 REST API Integration

**Get Daily Steps (REST):**
```http
POST https://www.googleapis.com/fitness/v1/users/me/dataset:aggregate
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "aggregateBy": [{
    "dataTypeName": "com.google.step_count.delta",
    "dataSourceId": "derived:com.google.step_count.delta:com.google.android.gms:estimated_steps"
  }],
  "bucketByTime": { "durationMillis": 86400000 },
  "startTimeMillis": 1735257600000,
  "endTimeMillis": 1735344000000
}
```

**Response:**
```json
{
  "bucket": [
    {
      "startTimeMillis": "1735257600000",
      "endTimeMillis": "1735344000000",
      "dataset": [
        {
          "dataSourceId": "derived:com.google.step_count.delta:com.google.android.gms:estimated_steps",
          "point": [
            {
              "startTimeNanos": "1735257600000000000",
              "endTimeNanos": "1735344000000000000",
              "dataTypeName": "com.google.step_count.delta",
              "value": [
                {
                  "intVal": 12547
                }
              ]
            }
          ]
        }
      ]
    }
  ]
}
```

---

## 7.3 Samsung Health Integration

### 7.3.1 Overview

Samsung Health provides APIs for Samsung devices to access health and fitness data.

**SDK Setup:**
```gradle
dependencies {
    implementation files('libs/samsung-health-data-1.5.0.jar')
}
```

### 7.3.2 Connection and Permissions

```java
import com.samsung.android.sdk.healthdata.*;

HealthDataStore mStore;

HealthConnectionErrorListener mConnectionListener = new HealthConnectionErrorListener() {
    @Override
    public void onConnectionFailed(HealthConnectionErrorResult error) {
        Log.e("SamsungHealth", "Connection failed: " + error.getErrorCode());
    }
};

mStore = new HealthDataStore(this, mConnectionListener);
mStore.connectService();

// Request permissions
Set<HealthPermission.Type> permissions = new HashSet<>();
permissions.add(new HealthPermission(HealthConstants.StepCount.HEALTH_DATA_TYPE, HealthPermission.Type.READ));
permissions.add(new HealthPermission(HealthConstants.Exercise.HEALTH_DATA_TYPE, HealthPermission.Type.READ));

HealthDataService.requestPermissions(permissions, this)
    .setResultListener(result -> {
        Log.d("SamsungHealth", "Permission result: " + result.getResultMap());
    });
```

---

## 7.4 Wearable Device Integration

### 7.4.1 Bluetooth Low Energy (BLE) Heart Rate

**Connecting to HR Monitor:**
```swift
import CoreBluetooth

class HeartRateManager: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private var centralManager: CBCentralManager!
    private var heartRatePeripheral: CBPeripheral?

    let heartRateServiceUUID = CBUUID(string: "180D")  // Heart Rate Service
    let heartRateMeasurementUUID = CBUUID(string: "2A37")  // Heart Rate Measurement

    func startScanning() {
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            central.scanForPeripherals(withServices: [heartRateServiceUUID])
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        heartRatePeripheral = peripheral
        heartRatePeripheral?.delegate = self
        centralManager.stopScan()
        centralManager.connect(peripheral, options: nil)
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        for characteristic in service.characteristics ?? [] {
            if characteristic.uuid == heartRateMeasurementUUID {
                peripheral.setNotifyValue(true, for: characteristic)
            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value else { return }

        let heartRate = parseHeartRate(from: data)
        print("Heart Rate: \(heartRate) BPM")
    }

    private func parseHeartRate(from data: Data) -> Int {
        let bytes = [UInt8](data)
        let firstBitValue = bytes[0] & 0x01

        if firstBitValue == 0 {
            // Heart Rate is UInt8
            return Int(bytes[1])
        } else {
            // Heart Rate is UInt16
            return Int(bytes[1]) | (Int(bytes[2]) << 8)
        }
    }
}
```

### 7.4.2 ANT+ Device Integration

**Connecting to ANT+ Sensor:**
```java
import com.dsi.ant.plugins.antplus.pcc.*;

AntPlusHeartRatePcc hrPcc;

AntPlusHeartRatePcc.IHeartRateDataReceiver hrDataReceiver = new AntPlusHeartRatePcc.IHeartRateDataReceiver() {
    @Override
    public void onNewHeartRateData(long estTimestamp, int computedHeartRate, long heartBeatCount, BigDecimal heartBeatEventTime, AntPlusHeartRatePcc.DataState dataState) {
        Log.d("ANT+", "Heart Rate: " + computedHeartRate + " BPM");
        // Send to WIA-IND-012 system
        updateHeartRate(computedHeartRate, estTimestamp);
    }
};

// Request access to HR monitor
hrPcc = AntPlusHeartRatePcc.requestAccess(this, deviceNumber, 0,
    new AntPluginPcc.IPluginAccessResultReceiver<AntPlusHeartRatePcc>() {
        @Override
        public void onResultReceived(AntPlusHeartRatePcc result, RequestAccessResult resultCode, DeviceState initialDeviceState) {
            if (resultCode == RequestAccessResult.SUCCESS) {
                hrPcc = result;
                hrPcc.subscribeHeartRateDataEvent(hrDataReceiver);
            }
        }
    },
    stateChangeReceiver
);
```

---

## 7.5 Gym Equipment Integration

### 7.5.1 FTMS (Fitness Machine Service)

**Bluetooth FTMS Protocol:**
```typescript
const FTMS_SERVICE_UUID = '00001826-0000-1000-8000-00805f9b34fb';
const TREADMILL_DATA_UUID = '00002acd-0000-1000-8000-00805f9b34fb';

async function connectToTreadmill() {
  const device = await navigator.bluetooth.requestDevice({
    filters: [{ services: [FTMS_SERVICE_UUID] }]
  });

  const server = await device.gatt.connect();
  const service = await server.getPrimaryService(FTMS_SERVICE_UUID);
  const characteristic = await service.getCharacteristic(TREADMILL_DATA_UUID);

  await characteristic.startNotifications();
  characteristic.addEventListener('characteristicvaluechanged', (event) => {
    const value = event.target.value;
    const treadmillData = parseTreadmillData(value);

    console.log('Speed:', treadmillData.speed, 'km/h');
    console.log('Incline:', treadmillData.incline, '%');
    console.log('Distance:', treadmillData.distance, 'm');
  });
}

function parseTreadmillData(dataView) {
  // Parse FTMS Treadmill Data format
  let flags = dataView.getUint16(0, true);

  return {
    speed: dataView.getUint16(2, true) / 100,  // km/h
    incline: dataView.getInt16(4, true) / 10,  // %
    distance: dataView.getUint24(6, true),     // meters
  };
}
```

---

## 7.6 Third-Party Service Integration

### 7.6.1 Strava API

**Upload Activity to Strava:**
```typescript
async function uploadToStrava(wiaWorkout: Workout, accessToken: string) {
  const stravaActivity = {
    name: wiaWorkout.name || `${wiaWorkout.type} Workout`,
    type: mapToStravaActivityType(wiaWorkout.type),
    start_date_local: wiaWorkout.startTime.toISOString(),
    elapsed_time: wiaWorkout.duration,
    distance: wiaWorkout.distance,
    description: wiaWorkout.notes
  };

  const response = await fetch('https://www.strava.com/api/v3/activities', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${accessToken}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(stravaActivity)
  });

  return await response.json();
}
```

### 7.6.2 TrainingPeaks API

**Sync Workout to TrainingPeaks:**
```typescript
async function syncToTrainingPeaks(wiaWorkout: Workout, apiKey: string) {
  const tpWorkout = {
    WorkoutDay: wiaWorkout.startTime.toISOString().split('T')[0],
    Title: wiaWorkout.name,
    WorkoutType: mapToTPWorkoutType(wiaWorkout.type),
    TotalTime: wiaWorkout.duration,
    Distance: wiaWorkout.distance / 1000,  // km
    TSS: wiaWorkout.tss,
    IntensityFactor: wiaWorkout.intensityFactor
  };

  const response = await fetch('https://api.trainingpeaks.com/v1/workouts', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${apiKey}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(tpWorkout)
  });

  return await response.json();
}
```

---

## 7.7 Healthcare System Integration

### 7.7.1 FHIR Conversion

**Convert WIA Workout to FHIR Observation:**
```typescript
function convertToFHIR(wiaWorkout: Workout): any {
  return {
    resourceType: "Observation",
    status: "final",
    category: [{
      coding: [{
        system: "http://terminology.hl7.org/CodeSystem/observation-category",
        code: "activity",
        display: "Activity"
      }]
    }],
    code: {
      coding: [{
        system: "http://loinc.org",
        code: "55411-3",
        display: "Exercise duration"
      }]
    },
    subject: {
      reference: `Patient/${wiaWorkout.userId}`
    },
    effectivePeriod: {
      start: wiaWorkout.startTime.toISOString(),
      end: wiaWorkout.endTime.toISOString()
    },
    valueQuantity: {
      value: wiaWorkout.duration / 60,
      unit: "min",
      system: "http://unitsofmeasure.org",
      code: "min"
    },
    component: [
      {
        code: {
          coding: [{
            system: "http://loinc.org",
            code: "41950-7",
            display: "Calories burned"
          }]
        },
        valueQuantity: {
          value: wiaWorkout.calories,
          unit: "kcal",
          system: "http://unitsofmeasure.org",
          code: "kcal"
        }
      },
      {
        code: {
          coding: [{
            system: "http://loinc.org",
            code: "55430-3",
            display: "Distance"
          }]
        },
        valueQuantity: {
          value: wiaWorkout.distance / 1000,
          unit: "km",
          system: "http://unitsofmeasure.org",
          code: "km"
        }
      }
    ]
  };
}
```

---

## Key Takeaways

✓ iOS HealthKit provides unified access to health data on Apple devices

✓ Google Fit offers REST and Android APIs for fitness data

✓ Samsung Health requires specific SDK for Samsung devices

✓ Bluetooth LE and ANT+ enable wearable device connectivity

✓ FTMS protocol standardizes gym equipment communication

✓ Strava, TrainingPeaks integrate for social and training features

✓ FHIR conversion enables healthcare system integration

✓ Always respect user privacy and obtain explicit permissions

✓ Handle sync conflicts with timestamp-based resolution

---

**Next:** [Chapter 8: Implementation Guide →](08-implementation.md)

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
