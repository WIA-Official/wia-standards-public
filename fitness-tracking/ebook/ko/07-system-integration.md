# 7장: 시스템 통합

## 개요

이 장은 WIA-IND-012 규격 시스템을 주요 건강 플랫폼, 웨어러블 기기 및 타사 서비스와 통합하기 위한 상세한 지침을 제공합니다.

---

## 7.1 iOS HealthKit 통합

### 7.1.1 개요

Apple HealthKit은 iOS 기기의 중앙 건강 데이터 저장소로, 다양한 앱과 기기의 데이터를 집계합니다.

**주요 기능:**
- 통합 건강 데이터 저장소
- 사용자 권한을 우선하는 개인정보 보호 설계
- 백그라운드 데이터 동기화
- 100개 이상의 데이터 유형 지원

### 7.1.2 설정 및 권한

**Info.plist 구성:**
```xml
<key>NSHealthShareUsageDescription</key>
<string>활동을 추적하고 맞춤형 인사이트를 제공하기 위해 피트니스 데이터에 액세스해야 합니다.</string>

<key>NSHealthUpdateUsageDescription</key>
<string>운동 데이터를 HealthKit에 저장하려면 권한이 필요합니다.</string>
```

**권한 요청:**
```swift
import HealthKit

let healthStore = HKHealthStore()

// 읽을 데이터 유형 정의
let typesToRead: Set<HKObjectType> = [
    HKObjectType.quantityType(forIdentifier: .stepCount)!,
    HKObjectType.quantityType(forIdentifier: .distanceWalkingRunning)!,
    HKObjectType.quantityType(forIdentifier: .heartRate)!,
    HKObjectType.quantityType(forIdentifier: .activeEnergyBurned)!,
    HKObjectType.workoutType()
]

// 쓸 데이터 유형 정의
let typesToWrite: Set<HKSampleType> = [
    HKObjectType.quantityType(forIdentifier: .stepCount)!,
    HKObjectType.quantityType(forIdentifier: .activeEnergyBurned)!,
    HKObjectType.workoutType()
]

// 권한 요청
healthStore.requestAuthorization(toShare: typesToWrite, read: typesToRead) { success, error in
    if success {
        print("HealthKit 권한이 부여되었습니다")
    }
}
```

### 7.1.3 데이터 읽기

**걸음 수 조회:**
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

### 7.1.4 데이터 쓰기

**운동 저장:**
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
            print("운동이 성공적으로 저장되었습니다")
        }
    }
}
```

---

## 7.2 Google Fit 통합

### 7.2.1 개요

Google Fit은 Android의 건강 및 피트니스 플랫폼으로 데이터 저장 및 검색을 위한 API를 제공합니다.

**주요 기능:**
- REST 및 Android API
- OAuth 2.0 인증
- 활동, 영양, 위치 데이터 유형

### 7.2.2 설정

**build.gradle:**
```gradle
dependencies {
    implementation 'com.google.android.gms:play-services-fitness:21.1.0'
    implementation 'com.google.android.gms:play-services-auth:20.7.0'
}
```

### 7.2.3 인증

**권한 요청:**
```kotlin
import com.google.android.gms.auth.api.signin.GoogleSignIn
import com.google.android.gms.fitness.FitnessOptions
import com.google.android.gms.fitness.data.DataType

val fitnessOptions = FitnessOptions.builder()
    .addDataType(DataType.TYPE_STEP_COUNT_DELTA, FitnessOptions.ACCESS_READ)
    .addDataType(DataType.TYPE_DISTANCE_DELTA, FitnessOptions.ACCESS_READ)
    .addDataType(DataType.TYPE_CALORIES_EXPENDED, FitnessOptions.ACCESS_READ)
    .addDataType(DataType.TYPE_HEART_RATE_BPM, FitnessOptions.ACCESS_READ)
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

### 7.2.4 데이터 읽기

**걸음 수 조회:**
```kotlin
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
                        Log.d("GoogleFit", "걸음: $steps")
                    }
                }
            }
        }
}
```

---

## 7.3 Samsung Health 통합

### 7.3.1 개요

Samsung Health는 Samsung 기기를 위한 건강 및 피트니스 데이터 액세스 API를 제공합니다.

**SDK 설정:**
```gradle
dependencies {
    implementation files('libs/samsung-health-data-1.5.0.jar')
}
```

### 7.3.2 연결 및 권한

```java
import com.samsung.android.sdk.healthdata.*;

HealthDataStore mStore;

HealthConnectionErrorListener mConnectionListener = new HealthConnectionErrorListener() {
    @Override
    public void onConnectionFailed(HealthConnectionErrorResult error) {
        Log.e("SamsungHealth", "연결 실패: " + error.getErrorCode());
    }
};

mStore = new HealthDataStore(this, mConnectionListener);
mStore.connectService();
```

---

## 7.4 웨어러블 기기 통합

### 7.4.1 Bluetooth Low Energy (BLE) 심박수

**심박수 모니터에 연결:**
```swift
import CoreBluetooth

class HeartRateManager: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private var centralManager: CBCentralManager!
    private var heartRatePeripheral: CBPeripheral?

    let heartRateServiceUUID = CBUUID(string: "180D")  // 심박수 서비스
    let heartRateMeasurementUUID = CBUUID(string: "2A37")  // 심박수 측정

    func startScanning() {
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            central.scanForPeripherals(withServices: [heartRateServiceUUID])
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value else { return }

        let heartRate = parseHeartRate(from: data)
        print("심박수: \(heartRate) BPM")
    }

    private func parseHeartRate(from data: Data) -> Int {
        let bytes = [UInt8](data)
        let firstBitValue = bytes[0] & 0x01

        if firstBitValue == 0 {
            // 심박수는 UInt8
            return Int(bytes[1])
        } else {
            // 심박수는 UInt16
            return Int(bytes[1]) | (Int(bytes[2]) << 8)
        }
    }
}
```

---

## 7.5 헬스장 장비 통합

### 7.5.1 FTMS (Fitness Machine Service)

**Bluetooth FTMS 프로토콜:**
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

    console.log('속도:', treadmillData.speed, 'km/h');
    console.log('경사:', treadmillData.incline, '%');
  });
}
```

---

## 7.6 타사 서비스 통합

### 7.6.1 Strava API

**Strava에 활동 업로드:**
```typescript
async function uploadToStrava(wiaWorkout: Workout, accessToken: string) {
  const stravaActivity = {
    name: wiaWorkout.name || `${wiaWorkout.type} 운동`,
    type: mapToStravaActivityType(wiaWorkout.type),
    start_date_local: wiaWorkout.startTime.toISOString(),
    elapsed_time: wiaWorkout.duration,
    distance: wiaWorkout.distance
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

---

## 7.7 의료 시스템 통합

### 7.7.1 FHIR 변환

**WIA 운동을 FHIR Observation으로 변환:**
```typescript
function convertToFHIR(wiaWorkout: Workout): any {
  return {
    resourceType: "Observation",
    status: "final",
    category: [{
      coding: [{
        system: "http://terminology.hl7.org/CodeSystem/observation-category",
        code: "activity",
        display: "활동"
      }]
    }],
    code: {
      coding: [{
        system: "http://loinc.org",
        code: "55411-3",
        display: "운동 지속 시간"
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
      unit: "분",
      system: "http://unitsofmeasure.org",
      code: "min"
    },
    component: [
      {
        code: {
          coding: [{
            system: "http://loinc.org",
            code: "41950-7",
            display: "소모 칼로리"
          }]
        },
        valueQuantity: {
          value: wiaWorkout.calories,
          unit: "kcal",
          system: "http://unitsofmeasure.org",
          code: "kcal"
        }
      }
    ]
  };
}
```

---

## 핵심 요점

✓ iOS HealthKit은 Apple 기기의 건강 데이터에 대한 통합 액세스 제공

✓ Google Fit은 피트니스 데이터를 위한 REST 및 Android API 제공

✓ Samsung Health는 Samsung 기기를 위한 특정 SDK 필요

✓ Bluetooth LE 및 ANT+는 웨어러블 기기 연결 가능

✓ FTMS 프로토콜은 헬스장 장비 통신 표준화

✓ Strava, TrainingPeaks는 소셜 및 트레이닝 기능 통합

✓ FHIR 변환은 의료 시스템 통합 가능

✓ 항상 사용자 개인정보 보호를 존중하고 명시적 권한 획득

---

**다음:** [8장: 구현 가이드 →](08-implementation.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
