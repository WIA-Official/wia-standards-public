# 제4장: 데이터 형식 및 스키마

## Phase 1: 환경 센서를 위한 표준화된 데이터 구조

---

## 학습 목표

이 장을 마치면 다음을 수행할 수 있습니다:

1. WIA-ENE-027 Phase 1 핵심 데이터 모델 구조 설명하기
2. 대기질, 수질, 토양, 기상 센서의 JSON 형식 작성하기
3. 품질 플래그와 보정 메타데이터의 목적 이해하기
4. JSON 스키마를 사용하여 센서 데이터 검증하기
5. 각 센서 유형의 유효 범위와 단위 식별하기
6. 측정 불확실성과 방법 메타데이터 포함하기
7. 완전한 WIA 준수 메시지 생성하기

---

## 4.1 핵심 데이터 모델 구조

이 장은 모든 센서 유형에 대한 완전한 JSON 스키마, 예제 및 검증 규칙을 제공하는 Phase 1 데이터 형식 사양을 자세히 설명합니다.

### 완전한 핵심 메시지 구조

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    // 센서별 데이터
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5,
    "accuracy": 10.0,
    "datum": "WGS84"
  },
  "metadata": {
    "manufacturer": "AirSense Corp",
    "model": "AS-3000",
    "firmware": "v2.4.1",
    "battery": 85,
    "signalStrength": -65,
    "uptime": 86400
  },
  "quality": {
    "overall": "good",
    "flags": [],
    "confidence": 0.95
  },
  "calibration": {
    "lastCalibration": "2024-11-15T09:00:00Z",
    "nextCalibration": "2025-05-15T09:00:00Z",
    "method": "reference_colocated",
    "parameters": {"slope": 1.02, "intercept": -0.5},
    "certificateId": "CAL-2024-1234"
  }
}
```

---

## 4.2 대기질 센서 데이터 형식

### 미세먼지

완전한 PM 센서 읽기 구조:

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm1_0": {
      "value": 8.5,
      "unit": "μg/m³",
      "method": "laser_scattering",
      "uncertainty": 2.0,
      "averaged": 300
    },
    "pm2_5": {
      "value": 15.3,
      "unit": "μg/m³",
      "method": "laser_scattering",
      "uncertainty": 3.0,
      "averaged": 300
    },
    "pm10": {
      "value": 22.8,
      "unit": "μg/m³",
      "method": "laser_scattering",
      "uncertainty": 4.5,
      "averaged": 300
    }
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5
  },
  "quality": {
    "overall": "good",
    "flags": []
  }
}
```

**유효 범위:**
- PM1.0: 0-500 μg/m³
- PM2.5: 0-500 μg/m³
- PM10: 0-1000 μg/m³

### 가스 오염물질

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-002",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "co2": {
      "value": 420,
      "unit": "ppm",
      "method": "NDIR",
      "uncertainty": 30
    },
    "co": {
      "value": 0.5,
      "unit": "ppm",
      "method": "electrochemical",
      "uncertainty": 0.1
    },
    "no2": {
      "value": 25,
      "unit": "ppb",
      "method": "electrochemical",
      "uncertainty": 5
    },
    "o3": {
      "value": 45,
      "unit": "ppb",
      "method": "electrochemical",
      "uncertainty": 10
    },
    "voc": {
      "value": 150,
      "unit": "ppb",
      "method": "metal_oxide",
      "uncertainty": 30
    }
  }
}
```

### 대기질 지수

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-003",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    "pm2_5": {
      "value": 35.5,
      "unit": "μg/m³"
    },
    "aqi": {
      "value": 102,
      "category": "unhealthy_sensitive",
      "pollutant": "pm2_5",
      "standard": "US_EPA"
    }
  }
}
```

**AQI 범주:**
- `"good"`: 0-50
- `"moderate"`: 51-100
- `"unhealthy_sensitive"`: 101-150
- `"unhealthy"`: 151-200
- `"very_unhealthy"`: 201-300
- `"hazardous"`: 301-500

---

## 4.3 수질 센서 데이터 형식

### 물리적 및 화학적 매개변수

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "WATER-SITE-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "water_quality",
  "readings": {
    "ph": {
      "value": 7.2,
      "unit": "pH",
      "method": "glass_electrode",
      "temperature_compensated": true,
      "uncertainty": 0.1
    },
    "dissolved_oxygen": {
      "value": 8.5,
      "unit": "mg/L",
      "method": "optical",
      "saturation": 95,
      "uncertainty": 0.2
    },
    "turbidity": {
      "value": 2.5,
      "unit": "NTU",
      "method": "nephelometric",
      "uncertainty": 0.3
    },
    "conductivity": {
      "value": 450,
      "unit": "μS/cm",
      "temperature": 20.5,
      "specific_conductance": 460,
      "uncertainty": 10
    },
    "temperature": {
      "value": 20.5,
      "unit": "°C",
      "uncertainty": 0.2
    },
    "orp": {
      "value": 250,
      "unit": "mV",
      "uncertainty": 10
    }
  },
  "location": {
    "latitude": 37.5700,
    "longitude": 126.9800,
    "altitude": 25.0
  }
}
```

**유효 범위:**
- pH: 0-14
- 용존산소: 0-20 mg/L
- 탁도: 0-1000 NTU
- 전도도: 0-10,000 μS/cm
- 온도: -2 ~ 40°C
- ORP: -500 ~ +500 mV

---

## 4.4 토양 센서 데이터 형식

### 수분, 온도 및 영양소

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "SOIL-FIELD-A-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "soil",
  "readings": {
    "moisture": {
      "value": 28.5,
      "unit": "%VWC",
      "method": "capacitance",
      "depth": 0.15,
      "uncertainty": 3.0
    },
    "temperature": {
      "value": 18.5,
      "unit": "°C",
      "depth": 0.10,
      "uncertainty": 0.5
    },
    "electrical_conductivity": {
      "value": 520,
      "unit": "μS/cm",
      "depth": 0.15,
      "uncertainty": 30
    },
    "nutrients": {
      "nitrogen": {
        "value": 45,
        "unit": "ppm",
        "form": "nitrate_NO3",
        "method": "ion_selective",
        "uncertainty": 10
      },
      "phosphorus": {
        "value": 12,
        "unit": "ppm",
        "form": "available_P",
        "uncertainty": 3
      },
      "potassium": {
        "value": 85,
        "unit": "ppm",
        "form": "exchangeable_K",
        "uncertainty": 15
      }
    }
  },
  "location": {
    "latitude": 37.5800,
    "longitude": 126.9900,
    "altitude": 45.0
  }
}
```

**유효 범위:**
- 수분: 0-60 %VWC
- 온도: -10 ~ 60°C
- EC: 0-4000 μS/cm
- 질소: 0-100 ppm
- 인: 0-50 ppm
- 칼륨: 0-200 ppm

---

## 4.5 기상 센서 데이터 형식

### 날씨 매개변수

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "WEATHER-STN-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "meteorological",
  "readings": {
    "temperature": {
      "value": 15.5,
      "unit": "°C",
      "uncertainty": 0.3
    },
    "humidity": {
      "value": 65,
      "unit": "%RH",
      "uncertainty": 3
    },
    "pressure": {
      "value": 1013.2,
      "unit": "hPa",
      "type": "station",
      "uncertainty": 0.5
    },
    "wind_speed": {
      "value": 3.5,
      "unit": "m/s",
      "gust": 5.2,
      "uncertainty": 0.3
    },
    "wind_direction": {
      "value": 225,
      "unit": "degrees",
      "uncertainty": 10
    },
    "precipitation": {
      "value": 0.2,
      "unit": "mm",
      "type": "cumulative",
      "period": 3600
    },
    "solar_radiation": {
      "value": 450,
      "unit": "W/m²",
      "type": "global"
    },
    "uv_index": {
      "value": 3,
      "unit": "index"
    }
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 38.5
  }
}
```

---

## 4.6 메타데이터 및 품질 플래그

### 품질 플래그 정의

```json
{
  "quality": {
    "overall": "suspect",
    "flags": [
      "calibration_due",
      "low_battery"
    ],
    "confidence": 0.75
  }
}
```

**표준 품질 플래그:**

| 플래그 | 설명 | 심각도 |
|------|-------------|----------|
| `calibration_due` | 보정 기한 임박 | 경고 |
| `calibration_overdue` | 보정 기한 경과 | 치명적 |
| `out_of_range` | 값이 그럴듯한 범위를 벗어남 | 치명적 |
| `rapid_change` | 비현실적인 변화율 | 경고 |
| `low_battery` | 배터리 < 20% | 경고 |
| `critical_battery` | 배터리 < 10% | 치명적 |
| `poor_signal` | 약한 네트워크 신호 | 경고 |
| `sensor_fault` | 자가 진단 실패 | 치명적 |
| `maintenance_required` | 서비스 간격 초과 | 경고 |
| `estimated` | 보간/추정 값 | 정보 |

---

## 4.7 보정 데이터 구조

### 보정 메타데이터

```json
{
  "calibration": {
    "lastCalibration": "2024-11-15T09:00:00Z",
    "nextCalibration": "2025-05-15T09:00:00Z",
    "method": "reference_colocated",
    "parameters": {
      "slope": 1.02,
      "intercept": -0.5,
      "r_squared": 0.95
    },
    "certificateId": "CAL-2024-1234",
    "calibratedBy": "TechCal Services",
    "referenceDevice": "MetOne BAM-1020"
  }
}
```

**보정 방법:**
- `"factory"`: 공장 보정만
- `"zero_span"`: 제로 및 스팬 가스 보정
- `"reference_colocated"`: 기준 기기와 공동 배치
- `"laboratory"`: 실험실 보정
- `"field"`: 현장 보정 절차

---

## 4.8 JSON 스키마 검증

### 완전한 JSON 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENE-027 Environmental Sensor Data",
  "type": "object",
  "required": ["version", "standard", "deviceId", "timestamp", "sensorType", "readings"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "standard": {
      "type": "string",
      "const": "WIA-ENE-027"
    },
    "deviceId": {
      "type": "string",
      "minLength": 1
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "sensorType": {
      "type": "string",
      "enum": ["air_quality", "water_quality", "soil", "meteorological", "radiation", "noise"]
    },
    "readings": {
      "type": "object"
    },
    "location": {
      "type": "object",
      "properties": {
        "latitude": {"type": "number", "minimum": -90, "maximum": 90},
        "longitude": {"type": "number", "minimum": -180, "maximum": 180},
        "altitude": {"type": "number"},
        "accuracy": {"type": "number", "minimum": 0},
        "datum": {"type": "string", "default": "WGS84"}
      },
      "required": ["latitude", "longitude"]
    },
    "quality": {
      "type": "object",
      "properties": {
        "overall": {
          "type": "string",
          "enum": ["good", "suspect", "bad", "missing"]
        },
        "flags": {
          "type": "array",
          "items": {"type": "string"}
        },
        "confidence": {
          "type": "number",
          "minimum": 0.0,
          "maximum": 1.0
        }
      }
    }
  }
}
```

---

## 4.9 복습 문제 및 핵심 요점

### 복습 문제

1. PM2.5=25 μg/m³, PM10=40 μg/m³, 온도=22°C를 측정하는 대기질 센서를 위한 완전한 WIA-ENE-027 메시지를 생성하세요. 30일 후 보정 기한이 있습니다.

2. pH=3.5, 400일 전에 마지막으로 보정됨, 배터리 15%인 수질 센서에 어떤 품질 플래그를 설정해야 합니까?

3. 세 깊이(10cm, 20cm, 30cm)에서 읽기가 있는 토양 수분 센서 메시지를 설계하세요. 데이터를 어떻게 구조화하시겠습니까?

4. 이 메시지를 검증하세요: `{"deviceId": "TEST", "timestamp": "2025-01-09", "sensorType": "air"}`. 무엇이 누락되었습니까?

5. 미국 EPA 방법을 사용하여 PM2.5=55 μg/m³에 대한 AQI를 계산하세요. 완전한 readings 객체를 생성하세요.

### 핵심 요점

1. **핵심 구조**: 모든 메시지는 기본 준수를 위해 version, standard, deviceId, timestamp, sensorType 및 readings 필드가 필요합니다.

2. **센서별**: 각 센서 유형(대기/수질/토양/기상)은 적절한 매개변수로 readings 객체에 대해 정의된 구조를 가집니다.

3. **품질 메타데이터**: 품질 플래그, 보정 데이터 및 전체 품질 지표는 자동화된 데이터 평가를 가능하게 합니다.

4. **JSON 스키마**: 검증 스키마는 메시지 준수 및 데이터 품질의 자동화된 확인을 가능하게 합니다.

5. **측정 구조**: 개별 측정에는 완전한 문서화를 위해 value, unit, method 및 uncertainty가 포함됩니다.

6. **위치 데이터**: 위도, 경도 및 고도는 환경 데이터의 공간 분석 및 매핑을 가능하게 합니다.

7. **보정**: 보정 메타데이터에는 추적 가능성을 위해 날짜, 방법, 매개변수 및 인증서 ID가 포함됩니다.

8. **확장성**: 형식은 기존 구현을 손상시키지 않고 확장을 통해 새로운 센서 유형 및 매개변수를 지원합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity

**다음 장: [제5장: API 인터페이스](05-api-interface.md)**
