# 제5장: IoT 센서 및 실시간 모니터링

**WIA-AGRI-016 전자책 시리즈**

---

## 식품 이력 추적의 IoT 혁명

기존 이력 추적은 특정 순간에 개별 이벤트를 캡처합니다. IoT 센서는 공급망 전반에 걸쳐 **지속적인 모니터링**을 가능하게 하여 식품 품질과 안전에 영향을 미치는 조건에 대한 실시간 가시성을 제공합니다.

### 주요 이점

- **사전 예방적 품질 관리:** 문제가 발생하기 전에 감지
- **콜드체인 준수:** 지속적인 온도 모니터링
- **자동화된 데이터 수집:** 수동 입력 오류 감소
- **예측 분석:** 센서 데이터로 훈련된 ML 모델
- **즉각적인 경고:** 이탈 즉시 통보

---

## 식품 이력 추적을 위한 센서 유형

### 1. 온도 센서

**기술:** 서미스터, RTD, 열전대

**사용 사례:**
- 콜드체인 모니터링 (냉장 운송)
- 저장 시설 준수
- 가공 매개변수 검증
- 조리/살균 검증

**사양:**
```
범위: -40°C ~ +85°C
정확도: ±0.2°C
판독 간격: 1-5분
배터리 수명: 30-90일
통신: BLE, LoRaWAN, 셀룰러
```

**구현:**
```javascript
class TemperatureSensor {
  constructor(sensorId, thresholds) {
    this.sensorId = sensorId;
    this.minTemp = thresholds.min;
    this.maxTemp = thresholds.max;
    this.readings = [];
  }

  recordReading(temperature, timestamp) {
    const reading = {
      sensorId: this.sensorId,
      temperature: temperature,
      timestamp: timestamp,
      status: this.checkThreshold(temperature)
    };

    this.readings.push(reading);

    if (reading.status !== 'normal') {
      this.triggerAlert(reading);
    }

    return reading;
  }

  checkThreshold(temperature) {
    if (temperature < this.minTemp) return 'too_cold';
    if (temperature > this.maxTemp) return 'too_warm';
    return 'normal';
  }

  async triggerAlert(reading) {
    await sendAlert({
      severity: 'high',
      type: 'temperature_excursion',
      sensorId: this.sensorId,
      temperature: reading.temperature,
      threshold: { min: this.minTemp, max: this.maxTemp },
      timestamp: reading.timestamp
    });
  }
}
```

### 2. 습도 센서

**응용 분야:**
- 저장 조건 모니터링
- 포장 무결성
- 곰팡이/부패 방지

### 3. GPS/위치 센서

**응용 분야:**
- 실시간 선적 추적
- 경로 최적화
- 지오펜싱 경고
- 원산지 검증

### 4. 충격/충돌 센서

**응용 분야:**
- 취급 품질 모니터링
- 손상 감지
- 책임 결정

---

## EPCIS와의 IoT 데이터 통합

### EPCIS 2.0의 센서 데이터

```json
{
  "type": "ObjectEvent",
  "eventTime": "2025-12-26T10:30:00Z",
  "eventTimeZoneOffset": "+09:00",
  "epcList": ["urn:epc:id:sscc:0123456.1234567890"],
  "action": "OBSERVE",
  "bizStep": "transporting",
  "disposition": "in_transit",

  "sensorElementList": [
    {
      "sensorMetadata": {
        "time": "2025-12-26T10:30:00Z",
        "deviceID": "urn:epc:id:giai:0123456.SENSOR.T123",
        "deviceMetadata": "온도 로거 모델 XYZ v2.1"
      },
      "sensorReport": [
        {
          "type": "Temperature",
          "value": 4.2,
          "uom": "CEL",
          "minValue": 3.8,
          "maxValue": 4.5,
          "meanValue": 4.1,
          "sDev": 0.2
        },
        {
          "type": "Humidity",
          "value": 75,
          "uom": "P1"
        }
      ]
    }
  ]
}
```

### 시계열 데이터베이스 저장

센서 데이터용 특수 데이터베이스 사용:

**InfluxDB 예시:**
```javascript
const { InfluxDB, Point } = require('@influxdata/influxdb-client');

class SensorDataStore {
  constructor(config) {
    this.client = new InfluxDB({
      url: config.url,
      token: config.token
    });
    this.writeApi = this.client.getWriteApi(config.org, config.bucket);
  }

  async recordTemperature(sensorId, batchId, temperature, location) {
    const point = new Point('temperature')
      .tag('sensor_id', sensorId)
      .tag('batch_id', batchId)
      .tag('location', location)
      .floatField('value', temperature)
      .timestamp(new Date());

    this.writeApi.writePoint(point);
    await this.writeApi.flush();
  }

  async queryTemperatureHistory(batchId, hours = 24) {
    const queryApi = this.client.getQueryApi(this.org);

    const query = `
      from(bucket: "traceability")
        |> range(start: -${hours}h)
        |> filter(fn: (r) => r._measurement == "temperature")
        |> filter(fn: (r) => r.batch_id == "${batchId}")
    `;

    const results = [];
    for await (const { values, tableMeta } of queryApi.iterateRows(query)) {
      const row = tableMeta.toObject(values);
      results.push(row);
    }

    return results;
  }
}
```

---

## 콜드체인 모니터링

### 온도 이탈 감지

```javascript
class ColdChainMonitor {
  constructor(batchId, thresholds) {
    this.batchId = batchId;
    this.minTemp = thresholds.min;
    this.maxTemp = thresholds.max;
    this.excursionStart = null;
  }

  async processReading(temperature, timestamp) {
    const outOfRange = temperature < this.minTemp || temperature > this.maxTemp;

    if (outOfRange && !this.excursionStart) {
      // 이탈 시작
      this.excursionStart = timestamp;

      await this.createExcursionRecord({
        batchId: this.batchId,
        type: temperature < this.minTemp ? 'too_cold' : 'too_warm',
        startTime: timestamp,
        temperature: temperature
      });

    } else if (!outOfRange && this.excursionStart) {
      // 이탈 종료
      const duration = (timestamp - this.excursionStart) / 1000; // 초

      await this.finalizeExcursion({
        batchId: this.batchId,
        startTime: this.excursionStart,
        endTime: timestamp,
        duration: duration
      });

      // 품질 영향 계산
      const impact = await this.calculateQualityImpact(duration, temperature);

      if (impact.shelfLifeReduction > 0) {
        await this.updateShelfLife(this.batchId, impact.shelfLifeReduction);
      }

      this.excursionStart = null;
    }
  }

  async calculateQualityImpact(duration, peakTemp) {
    // Q10 모델: 반응 속도는 10°C마다 2배 증가
    const referenceTemp = 4;  // °C
    const q10 = 2.5;
    const tempDiff = peakTemp - referenceTemp;

    const accelerationFactor = Math.pow(q10, tempDiff / 10);
    const effectiveTime = (duration / 3600) * accelerationFactor; // 시간

    return {
      shelfLifeReduction: effectiveTime / 24, // 일
      qualityScore: this.calculateQualityScore(effectiveTime)
    };
  }
}
```

---

## 예측 품질 분석

### ML 모델 훈련

```python
import pandas as pd
from sklearn.ensemble import RandomForestRegressor

class QualityPredictionModel:
    def __init__(self):
        self.model = RandomForestRegressor(n_estimators=100)

    def train(self, historical_data):
        """
        과거 센서 데이터 + 품질 결과로 모델 훈련
        """
        features = historical_data[[
            'avg_temperature',
            'temp_excursion_count',
            'avg_humidity',
            'days_in_transit',
            'shock_events',
            'product_variety'
        ]]

        target = historical_data['final_quality_score']

        self.model.fit(features, target)

    def predict_quality(self, sensor_data):
        """
        현재 센서 데이터를 기반으로 최종 품질 예측
        """
        features = self.extract_features(sensor_data)
        predicted_quality = self.model.predict([features])[0]

        return {
            'predicted_quality_score': round(predicted_quality, 2),
            'confidence': self.calculate_confidence(features),
            'risk_factors': self.identify_risk_factors(features, predicted_quality)
        }
```

---

## 장 요약

IoT 센서는 식품 이력 추적을 수동에서 능동으로 변환합니다:

**주요 센서 유형:**
- 온도 (콜드체인)
- 습도 (저장 조건)
- GPS (위치 추적)
- 충격/충돌 (취급 품질)

**통합:**
- EPCIS 2.0 센서 요소
- 시계열 데이터베이스
- 실시간 대시보드
- 예측 분석

**혜택:**
- 지속적인 모니터링
- 자동화된 경고
- 품질 예측
- 폐기물 감소

---

## 다음 장

**제6장: 리콜 관리 및 위기 대응**

신속한 리콜 시스템을 구현하고 식품 안전 사고를 관리하는 방법을 배웁니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
