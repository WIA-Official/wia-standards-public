# Phase 1: Pet Care Robot 데이터 포맷 사양

## WIA-PET-CARE-ROBOT 데이터 포맷 표준

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-CARE-ROBOT-PHASE1-001
**Primary Color**: #F59E0B (Amber)

---

## 1. 개요

### 1.1 목적

WIA-PET-CARE-ROBOT은 반려동물 관리, 풍요로움 제공 및 모니터링을 위해 설계된 자율 및 반자율 로봇 시스템을 위한 포괄적인 표준입니다. 이 표준은 펫 케어 로봇, 스마트 홈 시스템, 수의학 플랫폼 및 반려동물 소유자 애플리케이션 간의 상호 운용성을 가능하게 합니다.

**핵심 목표**:
- 로봇 장치 사양 및 기능 표준화
- 자동 급식, 급수 및 배설물 관리 지원
- 상호작용 놀이 및 운동 자동화 지원
- 건강 모니터링 및 행동 추적 통합
- 원격 제어 및 스케줄링 인터페이스 제공
- 감지 및 프로토콜을 통한 반려동물 안전 보장
- 다중 반려동물 가정 운영 조정
- 스마트 홈 생태계 통합 촉진

### 1.2 적용 범위

이 표준은 다음을 포함합니다:

| 도메인 | 설명 |
|--------|------|
| **로봇 사양** | 장치 유형, 센서, 액추에이터 및 기능 |
| **급식 시스템** | 분량 제어 기능이 있는 자동 음식 및 물 분배 |
| **놀이 & 운동** | 상호작용 장난감, 레이저 포인터, 볼 발사기 및 활동 장치 |
| **건강 모니터링** | 체중 추적, 활동 수준, 행동 분석 통합 |
| **안전 프로토콜** | 반려동물 감지, 충돌 회피, 비상 정지 |
| **스케줄링** | 시간 기반 및 이벤트 기반 자동화 루틴 |
| **다중 반려동물 지원** | 개별 반려동물 인식 및 맞춤형 관리 |
| **스마트 홈 통합** | Alexa, Google Home, HomeKit 및 IoT 플랫폼 연결 |

### 1.3 철학

**弘益人間 (홍익인간)** - 인류와 모든 생명체의 이익을 위하여

펫 케어 로봇은 일관된 관리, 풍요로움 및 모니터링을 보장하여 동물 복지를 향상시키며, 반려동물 소유자가 반려동물에게 최상의 관리를 제공할 수 있도록 지원합니다.

---

## 2. 로봇 장치 스키마

### 2.1 핵심 로봇 장치

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PetCareRobot",
  "type": "object",
  "required": [
    "robotId",
    "deviceType",
    "manufacturer",
    "modelNumber",
    "capabilities",
    "status",
    "location",
    "timestamp"
  ],
  "properties": {
    "robotId": {
      "type": "string",
      "pattern": "^PCR-[A-Z0-9]{12}$",
      "description": "고유 로봇 식별자"
    },
    "deviceType": {
      "type": "string",
      "enum": [
        "feeder",
        "water_fountain",
        "litter_box",
        "play_robot",
        "exercise_wheel",
        "treat_dispenser",
        "laser_toy",
        "ball_launcher",
        "camera_monitor",
        "grooming_station",
        "multi_function"
      ]
    },
    "manufacturer": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "model": {"type": "string"},
        "serialNumber": {"type": "string"},
        "firmwareVersion": {"type": "string"},
        "hardwareRevision": {"type": "string"},
        "manufactureDate": {
          "type": "string",
          "format": "date"
        }
      }
    },
    "capabilities": {
      "type": "object",
      "properties": {
        "feeding": {
          "type": "object",
          "properties": {
            "enabled": {"type": "boolean"},
            "portionControl": {"type": "boolean"},
            "scheduledFeeding": {"type": "boolean"},
            "manualDispense": {"type": "boolean"},
            "maxPortionSize": {
              "type": "number",
              "description": "최대 분량 (그램)"
            },
            "containerCapacity": {
              "type": "number",
              "description": "음식 컨테이너 용량 (그램)"
            },
            "foodTypes": {
              "type": "array",
              "items": {
                "type": "string",
                "enum": ["dry_kibble", "wet_food", "treats", "prescription"]
              }
            }
          }
        },
        "watering": {
          "type": "object",
          "properties": {
            "enabled": {"type": "boolean"},
            "waterFiltered": {"type": "boolean"},
            "waterCirculation": {"type": "boolean"},
            "capacityLiters": {"type": "number"},
            "temperatureControl": {"type": "boolean"},
            "minTemperature": {"type": "number"},
            "maxTemperature": {"type": "number"}
          }
        },
        "play": {
          "type": "object",
          "properties": {
            "enabled": {"type": "boolean"},
            "interactiveToys": {"type": "boolean"},
            "laserPointer": {"type": "boolean"},
            "ballLauncher": {"type": "boolean"},
            "motionTracking": {"type": "boolean"},
            "adaptiveDifficulty": {"type": "boolean"},
            "maxPlayDuration": {
              "type": "number",
              "description": "최대 놀이 시간 (분)"
            }
          }
        },
        "monitoring": {
          "type": "object",
          "properties": {
            "camera": {"type": "boolean"},
            "nightVision": {"type": "boolean"},
            "audioRecording": {"type": "boolean"},
            "twoWayAudio": {"type": "boolean"},
            "motionDetection": {"type": "boolean"},
            "petRecognition": {"type": "boolean"},
            "weightScale": {"type": "boolean"},
            "activityTracking": {"type": "boolean"}
          }
        },
        "mobility": {
          "type": "object",
          "properties": {
            "stationary": {"type": "boolean"},
            "wheeled": {"type": "boolean"},
            "maxSpeed": {
              "type": "number",
              "description": "최대 속도 (초당 미터)"
            },
            "obstacleAvoidance": {"type": "boolean"},
            "autonomousNavigation": {"type": "boolean"},
            "chargingStation": {"type": "boolean"}
          }
        }
      }
    },
    "sensors": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "sensorId": {"type": "string"},
          "sensorType": {
            "type": "string",
            "enum": [
              "camera",
              "microphone",
              "weight_scale",
              "proximity",
              "ultrasonic",
              "infrared",
              "temperature",
              "humidity",
              "food_level",
              "water_level",
              "battery",
              "motion_detector"
            ]
          },
          "location": {"type": "string"},
          "status": {
            "type": "string",
            "enum": ["active", "inactive", "error", "calibrating"]
          },
          "lastCalibration": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "status": {
      "type": "object",
      "required": ["operational", "batteryLevel"],
      "properties": {
        "operational": {
          "type": "string",
          "enum": ["online", "offline", "maintenance", "error", "standby"]
        },
        "batteryLevel": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "배터리 백분율"
        },
        "charging": {"type": "boolean"},
        "foodLevel": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "음식 컨테이너 수준 백분율"
        },
        "waterLevel": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "물 컨테이너 수준 백분율"
        },
        "errors": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "errorCode": {"type": "string"},
              "message": {"type": "string"},
              "severity": {
                "type": "string",
                "enum": ["info", "warning", "error", "critical"]
              },
              "timestamp": {
                "type": "string",
                "format": "date-time"
              }
            }
          }
        }
      }
    },
    "location": {
      "type": "object",
      "properties": {
        "room": {"type": "string"},
        "zone": {"type": "string"},
        "coordinates": {
          "type": "object",
          "properties": {
            "x": {"type": "number"},
            "y": {"type": "number"},
            "z": {"type": "number"}
          }
        },
        "fixedPosition": {"type": "boolean"}
      }
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

### 2.2 로봇 유형 비교

| 로봇 유형 | 주요 기능 | 이동성 | 일반 센서 | 전원 |
|-----------|----------|--------|----------|------|
| **자동 급식기** | 일정에 따라 음식 분배 | 고정형 | 음식 수준, 체중계, 카메라 | AC/배터리 |
| **워터 파운틴** | 신선한 여과수 제공 | 고정형 | 물 수준, 유량 센서, 필터 상태 | AC |
| **화장실** | 자동 배설물 제거 | 고정형 | 체중, 모션, 냄새 센서 | AC |
| **놀이 로봇** | 상호작용 참여 | 이동형/고정형 | 카메라, 모션, 근접 센서 | 배터리/AC |
| **볼 발사기** | 운동 및 가져오기 놀이 | 고정형 | 볼 센서, 카메라, 거리 측정기 | AC/배터리 |
| **간식 디스펜서** | 보상 기반 상호작용 | 고정형 | 카메라, 마이크, 분배 센서 | 배터리 |
| **다기능** | 통합 기능 | 가변 | 다중 센서 유형 | AC |

### 2.3 급식 이벤트 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "FeedingEvent",
  "type": "object",
  "required": [
    "eventId",
    "robotId",
    "petId",
    "timestamp",
    "foodType",
    "portionSize",
    "dispensed"
  ],
  "properties": {
    "eventId": {
      "type": "string",
      "pattern": "^FEED-[A-Z0-9]{16}$"
    },
    "robotId": {"type": "string"},
    "petId": {
      "type": "string",
      "description": "반려동물 식별자 또는 'unknown'"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "feedingType": {
      "type": "string",
      "enum": ["scheduled", "manual", "automated", "treat", "supplement"]
    },
    "foodType": {
      "type": "string",
      "enum": ["dry_kibble", "wet_food", "treats", "prescription", "supplement"]
    },
    "portionSize": {
      "type": "number",
      "description": "분량 크기 (그램)"
    },
    "dispensed": {
      "type": "boolean",
      "description": "음식이 성공적으로 분배되었는지 여부"
    },
    "consumed": {
      "type": "object",
      "properties": {
        "detected": {"type": "boolean"},
        "estimatedAmount": {
          "type": "number",
          "description": "추정 소비량 (그램)"
        },
        "duration": {
          "type": "number",
          "description": "섭취 시간 (초)"
        }
      }
    },
    "schedule": {
      "type": "object",
      "properties": {
        "scheduleId": {"type": "string"},
        "plannedTime": {
          "type": "string",
          "format": "date-time"
        },
        "onTime": {"type": "boolean"},
        "delaySeconds": {"type": "number"}
      }
    },
    "nutrition": {
      "type": "object",
      "properties": {
        "calories": {"type": "number"},
        "protein": {"type": "number"},
        "fat": {"type": "number"},
        "carbohydrates": {"type": "number"},
        "fiber": {"type": "number"}
      }
    }
  }
}
```

---

## 3. 놀이 및 운동 데이터

### 3.1 놀이 세션 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PlaySession",
  "type": "object",
  "required": [
    "sessionId",
    "robotId",
    "petId",
    "startTime",
    "endTime",
    "playType"
  ],
  "properties": {
    "sessionId": {
      "type": "string",
      "pattern": "^PLAY-[A-Z0-9]{16}$"
    },
    "robotId": {"type": "string"},
    "petId": {"type": "string"},
    "startTime": {
      "type": "string",
      "format": "date-time"
    },
    "endTime": {
      "type": "string",
      "format": "date-time"
    },
    "duration": {
      "type": "number",
      "description": "지속 시간 (초)"
    },
    "playType": {
      "type": "string",
      "enum": [
        "laser_chase",
        "ball_fetch",
        "treat_puzzle",
        "interactive_toy",
        "motion_game",
        "sound_stimulation",
        "automated_feather",
        "custom"
      ]
    },
    "intensity": {
      "type": "string",
      "enum": ["low", "medium", "high", "adaptive"]
    },
    "engagement": {
      "type": "object",
      "properties": {
        "level": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "참여도 백분율"
        },
        "interactions": {
          "type": "number",
          "description": "감지된 반려동물 상호작용 수"
        },
        "attentionSpan": {
          "type": "number",
          "description": "평균 주의 지속 시간 (초)"
        }
      }
    },
    "activity": {
      "type": "object",
      "properties": {
        "movementDetected": {"type": "boolean"},
        "distanceTraveled": {
          "type": "number",
          "description": "예상 이동 거리 (미터)"
        },
        "averageSpeed": {
          "type": "number",
          "description": "평균 속도 (m/s)"
        },
        "jumps": {"type": "number"},
        "caloriesBurned": {
          "type": "number",
          "description": "예상 소모 칼로리"
        }
      }
    },
    "robotActions": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "timestamp": {
            "type": "string",
            "format": "date-time"
          },
          "action": {
            "type": "string",
            "enum": [
              "laser_move",
              "ball_launch",
              "treat_dispense",
              "sound_play",
              "movement",
              "pause"
            ]
          },
          "parameters": {
            "type": "object",
            "description": "액션별 매개변수"
          }
        }
      }
    },
    "terminationReason": {
      "type": "string",
      "enum": [
        "scheduled_end",
        "manual_stop",
        "pet_disengaged",
        "battery_low",
        "error",
        "timeout"
      ]
    }
  }
}
```

### 3.2 놀이 활동 측정 항목

| 측정 항목 | 단위 | 설명 | 일반 범위 |
|-----------|------|------|-----------|
| **세션 지속 시간** | 분 | 놀이 세션 길이 | 5-30 |
| **참여도 수준** | 백분율 | 반려동물의 관심 및 참여 | 0-100 |
| **이동 거리** | 미터 | 반려동물이 이동한 총 거리 | 10-500 |
| **칼로리 소모** | kcal | 예상 에너지 소비 | 10-150 |
| **상호작용 횟수** | 횟수 | 반려동물-로봇 상호작용 수 | 5-200 |
| **성공률** | 백분율 | 성공적인 잡기/완료 | 0-100 |

---

## 4. 건강 모니터링 통합

### 4.1 건강 관찰 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "HealthObservation",
  "type": "object",
  "required": [
    "observationId",
    "robotId",
    "petId",
    "timestamp",
    "observationType"
  ],
  "properties": {
    "observationId": {
      "type": "string",
      "pattern": "^OBS-[A-Z0-9]{16}$"
    },
    "robotId": {"type": "string"},
    "petId": {"type": "string"},
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "observationType": {
      "type": "string",
      "enum": [
        "weight_measurement",
        "activity_level",
        "eating_behavior",
        "drinking_behavior",
        "elimination",
        "movement_pattern",
        "vocalization",
        "behavioral_change"
      ]
    },
    "weight": {
      "type": "object",
      "properties": {
        "value": {
          "type": "number",
          "description": "체중 (킬로그램)"
        },
        "unit": {
          "type": "string",
          "enum": ["kg", "lbs"]
        },
        "trend": {
          "type": "string",
          "enum": ["increasing", "decreasing", "stable", "unknown"]
        },
        "changePercent": {
          "type": "number",
          "description": "이전 측정값으로부터의 변화"
        }
      }
    },
    "activityLevel": {
      "type": "object",
      "properties": {
        "level": {
          "type": "string",
          "enum": ["very_low", "low", "normal", "high", "very_high"]
        },
        "duration": {
          "type": "number",
          "description": "활동 시간 (분)"
        },
        "intensityScore": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        }
      }
    },
    "eatingBehavior": {
      "type": "object",
      "properties": {
        "appetiteLevel": {
          "type": "string",
          "enum": ["none", "poor", "normal", "increased", "excessive"]
        },
        "eatingSpeed": {
          "type": "string",
          "enum": ["very_slow", "slow", "normal", "fast", "very_fast"]
        },
        "foodLeftover": {
          "type": "number",
          "description": "소비되지 않은 음식의 백분율"
        },
        "unusualBehavior": {
          "type": "boolean"
        }
      }
    },
    "alerts": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "alertType": {
            "type": "string",
            "enum": [
              "weight_change",
              "reduced_activity",
              "eating_change",
              "drinking_change",
              "behavioral_concern"
            ]
          },
          "severity": {
            "type": "string",
            "enum": ["info", "low", "medium", "high", "urgent"]
          },
          "message": {"type": "string"},
          "recommendation": {"type": "string"}
        }
      }
    }
  }
}
```

### 4.2 건강 경고 트리거

| 경고 유형 | 트리거 조건 | 심각도 | 권장 조치 |
|-----------|-------------|--------|----------|
| **급속 체중 감소** | 7일 내 5% 이상 감소 | 높음 | 수의사 상담 |
| **식욕 감소** | 2회 이상 식사 50% 미만 섭취 | 중간 | 모니터링 및 지속 시 수의사 연락 |
| **활동 미감지** | 24시간 내 10분 미만 활동 | 중간 | 반려동물 복지 확인 |
| **과도한 음수** | 기준선 대비 50% 이상 증가 | 중간 | 당뇨병 징후 모니터링 |
| **빠른 섭식** | 60초 미만 식사 완료 | 낮음 | 슬로우 피드 그릇 고려 |
| **체중 증가** | 30일 내 10% 이상 증가 | 중간 | 분량 및 운동 조절 |

---

## 5. 다중 반려동물 관리

### 5.1 반려동물 프로필 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PetProfile",
  "type": "object",
  "required": [
    "petId",
    "name",
    "species",
    "identification"
  ],
  "properties": {
    "petId": {
      "type": "string",
      "pattern": "^PET-[A-Z0-9]{12}$"
    },
    "name": {"type": "string"},
    "species": {
      "type": "string",
      "enum": ["dog", "cat", "rabbit", "bird", "other"]
    },
    "breed": {"type": "string"},
    "birthDate": {
      "type": "string",
      "format": "date"
    },
    "gender": {
      "type": "string",
      "enum": ["male", "female", "unknown"]
    },
    "neutered": {"type": "boolean"},
    "identification": {
      "type": "object",
      "properties": {
        "microchipId": {"type": "string"},
        "rfidTag": {"type": "string"},
        "facialRecognition": {
          "type": "object",
          "properties": {
            "enabled": {"type": "boolean"},
            "modelId": {"type": "string"},
            "confidence": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            }
          }
        },
        "collarTag": {"type": "string"}
      }
    },
    "dietaryRequirements": {
      "type": "object",
      "properties": {
        "foodType": {
          "type": "array",
          "items": {"type": "string"}
        },
        "portionSize": {
          "type": "number",
          "description": "1일 분량 (그램)"
        },
        "feedingFrequency": {
          "type": "number",
          "description": "1일 식사 횟수"
        },
        "allergies": {
          "type": "array",
          "items": {"type": "string"}
        },
        "restrictions": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    },
    "preferences": {
      "type": "object",
      "properties": {
        "favoritePlayType": {
          "type": "array",
          "items": {"type": "string"}
        },
        "playIntensity": {
          "type": "string",
          "enum": ["low", "medium", "high"]
        },
        "treatPreference": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    },
    "medicalConditions": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "condition": {"type": "string"},
          "diagnosed": {
            "type": "string",
            "format": "date"
          },
          "medications": {
            "type": "array",
            "items": {"type": "string"}
          },
          "restrictions": {
            "type": "array",
            "items": {"type": "string"}
          }
        }
      }
    }
  }
}
```

### 5.2 다중 반려동물 가정 시나리오

| 시나리오 | 과제 | 로봇 솔루션 | 구현 |
|----------|------|------------|------|
| **개별 급식** | 반려동물이 서로 음식 훔침 | RFID 트리거 급식 스테이션 | 각 반려동물은 고유 태그 보유; 급식기는 인증된 반려동물에게만 열림 |
| **분량 제어** | 다른 식이 요구 사항 | 반려동물별 분량 분배 | 로봇이 반려동물을 인식하고 올바른 양 분배 |
| **놀이 선호도** | 다른 에너지 수준 | 적응형 놀이 세션 | 로봇이 반려동물 참여도에 따라 강도 조정 |
| **영역 관리** | 자원 보호 | 예약된 접근 시간대 | 공유 자원에 대한 시간 기반 순환 |
| **약물 전달** | 한 반려동물만 보충제 필요 | 대상 간식 분배 | 특정 반려동물에게만 약물 간식 분배 |

---

## 6. 코드 예제

### 6.1 로봇 상태 조회

```python
import requests
import json
from datetime import datetime

class PetCareRobotClient:
    def __init__(self, base_url, api_key):
        self.base_url = base_url
        self.headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json'
        }

    def get_robot_status(self, robot_id):
        """펫 케어 로봇의 현재 상태 조회"""
        endpoint = f"{self.base_url}/api/v1/robots/{robot_id}/status"

        response = requests.get(endpoint, headers=self.headers)
        response.raise_for_status()

        return response.json()

    def check_supplies(self, robot_id):
        """음식 및 물 수준 확인"""
        status = self.get_robot_status(robot_id)

        supplies = {
            'food_level': status['status']['foodLevel'],
            'water_level': status['status']['waterLevel'],
            'battery_level': status['status']['batteryLevel']
        }

        # 공급품이 부족하면 경고
        alerts = []
        if supplies['food_level'] < 20:
            alerts.append('음식 수준 매우 낮음')
        if supplies['water_level'] < 30:
            alerts.append('물 보충 필요')
        if supplies['battery_level'] < 15:
            alerts.append('배터리 부족 - 충전기 연결 필요')

        return {
            'supplies': supplies,
            'alerts': alerts,
            'timestamp': datetime.now().isoformat()
        }

# 사용
client = PetCareRobotClient('https://api.petcare.example.com', 'your-api-key')
status = client.check_supplies('PCR-ABC123456789')
print(json.dumps(status, indent=2))
```

### 6.2 급식 이벤트 예약

```javascript
const axios = require('axios');

class FeedingScheduler {
  constructor(apiUrl, apiKey) {
    this.apiUrl = apiUrl;
    this.apiKey = apiKey;
  }

  async scheduleFeedingEvent(robotId, petId, schedule) {
    const feedingEvent = {
      robotId: robotId,
      petId: petId,
      feedingType: 'scheduled',
      foodType: schedule.foodType || 'dry_kibble',
      portionSize: schedule.portionSize,
      schedule: {
        plannedTime: schedule.time,
        recurring: schedule.recurring || false,
        daysOfWeek: schedule.daysOfWeek || []
      }
    };

    try {
      const response = await axios.post(
        `${this.apiUrl}/api/v1/feeding/schedule`,
        feedingEvent,
        {
          headers: {
            'Authorization': `Bearer ${this.apiKey}`,
            'Content-Type': 'application/json'
          }
        }
      );

      return {
        success: true,
        scheduleId: response.data.scheduleId,
        message: '급식 예약 성공'
      };
    } catch (error) {
      return {
        success: false,
        error: error.message
      };
    }
  }

  async createDailyFeedingSchedule(robotId, petId, meals) {
    const schedules = [];

    for (const meal of meals) {
      const result = await this.scheduleFeedingEvent(robotId, petId, {
        time: meal.time,
        portionSize: meal.portion,
        foodType: meal.type,
        recurring: true,
        daysOfWeek: ['MON', 'TUE', 'WED', 'THU', 'FRI', 'SAT', 'SUN']
      });

      schedules.push(result);
    }

    return schedules;
  }
}

// 사용 예제
const scheduler = new FeedingScheduler('https://api.petcare.example.com', 'api-key');

const dailyMeals = [
  { time: '07:00:00', portion: 100, type: 'dry_kibble' },
  { time: '12:00:00', portion: 50, type: 'wet_food' },
  { time: '18:00:00', portion: 100, type: 'dry_kibble' }
];

scheduler.createDailyFeedingSchedule('PCR-ABC123456789', 'PET-XYZ987654321', dailyMeals)
  .then(results => console.log('급식 일정 생성됨:', results));
```

### 6.3 놀이 세션 자동화

```typescript
interface PlaySessionConfig {
  robotId: string;
  petId: string;
  playType: string;
  duration: number;
  intensity: 'low' | 'medium' | 'high' | 'adaptive';
  startTime?: Date;
}

class PlaySessionManager {
  private apiUrl: string;
  private apiKey: string;

  constructor(apiUrl: string, apiKey: string) {
    this.apiUrl = apiUrl;
    this.apiKey = apiKey;
  }

  async startPlaySession(config: PlaySessionConfig): Promise<any> {
    const sessionData = {
      robotId: config.robotId,
      petId: config.petId,
      playType: config.playType,
      intensity: config.intensity,
      maxDuration: config.duration,
      startTime: config.startTime || new Date()
    };

    const response = await fetch(`${this.apiUrl}/api/v1/play/session/start`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(sessionData)
    });

    if (!response.ok) {
      throw new Error(`놀이 세션 시작 실패: ${response.statusText}`);
    }

    return await response.json();
  }

  async monitorPlaySession(sessionId: string): Promise<any> {
    const response = await fetch(
      `${this.apiUrl}/api/v1/play/session/${sessionId}`,
      {
        headers: {
          'Authorization': `Bearer ${this.apiKey}`
        }
      }
    );

    const sessionData = await response.json();

    // 참여도 수준 확인
    if (sessionData.engagement.level < 30) {
      console.warn('낮은 참여도 감지 - 놀이 유형 변경 고려');
    }

    return sessionData;
  }

  async stopPlaySession(sessionId: string, reason: string): Promise<void> {
    await fetch(`${this.apiUrl}/api/v1/play/session/${sessionId}/stop`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${this.apiKey}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ reason })
    });
  }
}

// 사용
const playManager = new PlaySessionManager('https://api.petcare.example.com', 'api-key');

const sessionConfig: PlaySessionConfig = {
  robotId: 'PCR-ABC123456789',
  petId: 'PET-XYZ987654321',
  playType: 'laser_chase',
  duration: 900, // 15분
  intensity: 'adaptive'
};

playManager.startPlaySession(sessionConfig)
  .then(session => {
    console.log('놀이 세션 시작됨:', session.sessionId);

    // 30초마다 모니터링
    const monitorInterval = setInterval(async () => {
      const status = await playManager.monitorPlaySession(session.sessionId);
      console.log('참여도:', status.engagement.level);

      if (status.endTime) {
        clearInterval(monitorInterval);
        console.log('세션 완료');
      }
    }, 30000);
  });
```

### 6.4 다중 반려동물 인식 및 관리

```python
from typing import List, Dict, Optional
import cv2
import numpy as np

class MultiPetManager:
    def __init__(self, robot_client):
        self.robot = robot_client
        self.pet_profiles = {}
        self.recognition_model = None

    def register_pet(self, pet_profile: Dict) -> str:
        """시스템에 반려동물 등록"""
        pet_id = pet_profile['petId']
        self.pet_profiles[pet_id] = pet_profile

        # 이 반려동물에 대한 로봇 구성
        self.robot.add_pet_configuration(pet_id, {
            'feedingSchedule': pet_profile['dietaryRequirements'],
            'playPreferences': pet_profile['preferences'],
            'identificationMethod': pet_profile['identification']
        })

        return pet_id

    def identify_pet(self, image_data: np.ndarray) -> Optional[Dict]:
        """얼굴 인식을 사용하여 어떤 반려동물이 있는지 식별"""
        # 반려동물 감지를 위한 이미지 처리
        detected_pets = []

        for pet_id, profile in self.pet_profiles.items():
            if profile['identification'].get('facialRecognition', {}).get('enabled'):
                # 얼굴 인식 실행
                confidence = self._run_recognition(image_data, pet_id)

                if confidence > 85:
                    detected_pets.append({
                        'petId': pet_id,
                        'name': profile['name'],
                        'confidence': confidence
                    })

        # 가장 높은 신뢰도 일치 항목 반환
        if detected_pets:
            return max(detected_pets, key=lambda x: x['confidence'])

        return None

    def dispense_food_for_pet(self, robot_id: str, pet_id: str) -> Dict:
        """식별된 반려동물에게 적절한 음식 분량 분배"""
        profile = self.pet_profiles.get(pet_id)

        if not profile:
            return {'error': '반려동물이 등록되지 않음'}

        # 식이 요구 사항 가져오기
        diet = profile['dietaryRequirements']
        portion = diet['portionSize'] / diet['feedingFrequency']

        # 음식 분배
        result = self.robot.dispense_food({
            'robotId': robot_id,
            'petId': pet_id,
            'portionSize': portion,
            'foodType': diet['foodType'][0]
        })

        return result

    def manage_multi_pet_feeding(self, robot_id: str,
                                 detected_pet_ids: List[str]) -> List[Dict]:
        """여러 반려동물이 감지되었을 때 급식 관리"""
        results = []

        for pet_id in detected_pet_ids:
            # 이 반려동물이 이미 최근에 먹었는지 확인
            last_feeding = self.robot.get_last_feeding(pet_id)

            if last_feeding and self._is_recent(last_feeding['timestamp'], hours=4):
                results.append({
                    'petId': pet_id,
                    'action': 'skipped',
                    'reason': '최근에 이미 급식됨'
                })
                continue

            # 음식 분배
            result = self.dispense_food_for_pet(robot_id, pet_id)
            results.append(result)

        return results

    def _run_recognition(self, image: np.ndarray, pet_id: str) -> float:
        """얼굴 인식 모델 실행"""
        # 실제 인식 로직을 위한 플레이스홀더
        # 프로덕션에서는 학습된 모델을 사용할 것입니다
        return 90.0

    def _is_recent(self, timestamp: str, hours: int) -> bool:
        """타임스탬프가 지정된 시간 내에 있는지 확인"""
        from datetime import datetime, timedelta

        event_time = datetime.fromisoformat(timestamp)
        time_diff = datetime.now() - event_time

        return time_diff < timedelta(hours=hours)

# 사용 예제
robot_client = PetCareRobotClient('https://api.petcare.example.com', 'api-key')
manager = MultiPetManager(robot_client)

# 반려동물 등록
pet1 = {
    'petId': 'PET-DOG001',
    'name': 'Max',
    'species': 'dog',
    'identification': {
        'rfidTag': 'RFID-12345',
        'facialRecognition': {'enabled': True}
    },
    'dietaryRequirements': {
        'foodType': ['dry_kibble'],
        'portionSize': 400,
        'feedingFrequency': 2
    },
    'preferences': {
        'favoritePlayType': ['ball_fetch'],
        'playIntensity': 'high'
    }
}

manager.register_pet(pet1)
```

### 6.5 안전 프로토콜 구현

```go
package petcare

import (
    "context"
    "fmt"
    "time"
)

type SafetyMonitor struct {
    robotID        string
    sensors        map[string]Sensor
    emergencyStop  chan bool
    alertChannel   chan Alert
}

type Sensor struct {
    Type     string
    Status   string
    LastRead time.Time
    Value    interface{}
}

type Alert struct {
    Severity    string
    Type        string
    Message     string
    Timestamp   time.Time
    Action      string
}

func NewSafetyMonitor(robotID string) *SafetyMonitor {
    return &SafetyMonitor{
        robotID:       robotID,
        sensors:       make(map[string]Sensor),
        emergencyStop: make(chan bool, 1),
        alertChannel:  make(chan Alert, 100),
    }
}

func (sm *SafetyMonitor) MonitorProximity(ctx context.Context) {
    ticker := time.NewTicker(100 * time.Millisecond)
    defer ticker.Stop()

    for {
        select {
        case <-ctx.Done():
            return
        case <-ticker.C:
            // 근접 센서 읽기
            distance := sm.readProximitySensor()

            if distance < 0.1 { // 10cm 미만
                sm.alertChannel <- Alert{
                    Severity:  "critical",
                    Type:      "collision_imminent",
                    Message:   "10cm 이내 물체 감지",
                    Timestamp: time.Now(),
                    Action:    "emergency_stop",
                }
                sm.emergencyStop <- true
            } else if distance < 0.3 { // 30cm 미만
                sm.alertChannel <- Alert{
                    Severity:  "warning",
                    Type:      "proximity_warning",
                    Message:   "30cm 이내 물체 감지",
                    Timestamp: time.Now(),
                    Action:    "reduce_speed",
                }
            }
        }
    }
}

func (sm *SafetyMonitor) MonitorPetPresence(ctx context.Context) {
    ticker := time.NewTicker(500 * time.Millisecond)
    defer ticker.Stop()

    var petDetectedCount int

    for {
        select {
        case <-ctx.Done():
            return
        case <-ticker.C:
            petDetected := sm.detectPet()

            if petDetected {
                petDetectedCount++

                // 여러 읽기에서 반려동물 존재 확인
                if petDetectedCount >= 3 {
                    sm.adjustOperationForPetPresence()
                }
            } else {
                petDetectedCount = 0
            }
        }
    }
}

func (sm *SafetyMonitor) EmergencyStopHandler(ctx context.Context) {
    for {
        select {
        case <-ctx.Done():
            return
        case <-sm.emergencyStop:
            fmt.Println("비상 정지 활성화")
            sm.executeEmergencyStop()

            // 수동 재설정 대기
            time.Sleep(5 * time.Second)

            // 재개가 안전한지 확인
            if sm.isSafeToResume() {
                sm.resumeOperation()
            }
        }
    }
}

func (sm *SafetyMonitor) readProximitySensor() float64 {
    // 초음파 센서에서 읽기 시뮬레이션
    // 미터 단위로 거리 반환
    return 0.5
}

func (sm *SafetyMonitor) detectPet() bool {
    // 카메라/모션 센서를 사용한 반려동물 감지 시뮬레이션
    return true
}

func (sm *SafetyMonitor) adjustOperationForPetPresence() {
    fmt.Println("반려동물 감지 - 작동 매개변수 조정 중")
    // 속도 감소, 센서 폴링 증가 등
}

func (sm *SafetyMonitor) executeEmergencyStop() {
    // 모든 모터 정지
    // 분배 메커니즘 비활성화
    // 경고 LED/사운드 활성화
    fmt.Println("모든 시스템 정지됨")
}

func (sm *SafetyMonitor) isSafeToResume() bool {
    // 모든 안전 조건 확인
    distance := sm.readProximitySensor()
    return distance > 0.5 // 안전 거리 임계값
}

func (sm *SafetyMonitor) resumeOperation() {
    fmt.Println("정상 작동 재개")
}

// 사용
func main() {
    ctx := context.Background()
    monitor := NewSafetyMonitor("PCR-ABC123456789")

    go monitor.MonitorProximity(ctx)
    go monitor.MonitorPetPresence(ctx)
    go monitor.EmergencyStopHandler(ctx)

    // 경고 처리
    for alert := range monitor.alertChannel {
        fmt.Printf("[%s] %s: %s - 조치: %s\n",
            alert.Severity, alert.Type, alert.Message, alert.Action)
    }
}
```

### 6.6 스마트 홈 통합

```javascript
const { SmartHomeDevice } = require('alexa-smart-home-sdk');

class PetCareRobotAlexaSkill extends SmartHomeDevice {
  constructor(config) {
    super(config);
    this.robotClient = config.robotClient;
  }

  // Alexa 디스커버리
  async discover() {
    const robots = await this.robotClient.listRobots();

    return robots.map(robot => ({
      endpointId: robot.robotId,
      manufacturerName: robot.manufacturer.name,
      friendlyName: `펫 급식기 ${robot.location.room}`,
      description: `WIA 펫 케어 로봇 - ${robot.deviceType}`,
      displayCategories: ['PET_FEEDER'],
      capabilities: [
        {
          type: 'AlexaInterface',
          interface: 'Alexa.PowerController',
          version: '3',
          properties: {
            supported: [{ name: 'powerState' }],
            retrievable: true
          }
        },
        {
          type: 'AlexaInterface',
          interface: 'Alexa.PercentageController',
          version: '3',
          properties: {
            supported: [{ name: 'percentage' }],
            retrievable: true
          }
        },
        {
          type: 'AlexaInterface',
          interface: 'Alexa.EndpointHealth',
          version: '3',
          properties: {
            supported: [{ name: 'connectivity' }],
            retrievable: true
          }
        }
      ]
    }));
  }

  // "Alexa, 반려동물 급식해줘" 처리
  async handleFeedPetCommand(endpointId, portion) {
    const result = await this.robotClient.dispenseFeed({
      robotId: endpointId,
      feedingType: 'manual',
      portionSize: portion || 100
    });

    return {
      namespace: 'Alexa',
      name: result.dispensed ? 'Response' : 'ErrorResponse',
      payload: {
        message: result.dispensed
          ? '반려동물에게 급식되었습니다'
          : '음식을 분배할 수 없습니다 - 로봇 상태 확인'
      }
    };
  }

  // "Alexa, 놀이 세션 시작해줘" 처리
  async handlePlayCommand(endpointId, duration) {
    const result = await this.robotClient.startPlaySession({
      robotId: endpointId,
      playType: 'laser_chase',
      duration: duration || 600,
      intensity: 'adaptive'
    });

    return {
      namespace: 'Alexa',
      name: 'Response',
      payload: {
        message: `${duration || 10}분 놀이 세션을 시작했습니다`
      }
    };
  }

  // "Alexa, 음식 수준 어때?" 처리
  async handleStatusQuery(endpointId) {
    const status = await this.robotClient.getRobotStatus(endpointId);

    const message = `음식 수준은 ${status.status.foodLevel}%, ` +
                   `물 수준은 ${status.status.waterLevel}%, ` +
                   `배터리는 ${status.status.batteryLevel}%입니다`;

    return {
      namespace: 'Alexa',
      name: 'Response',
      payload: { message }
    };
  }
}

// Google Home 통합
class PetCareRobotGoogleHome {
  constructor(robotClient) {
    this.robotClient = robotClient;
  }

  // Google Home SYNC 인텐트 처리
  async onSync(body) {
    const robots = await this.robotClient.listRobots();

    return {
      requestId: body.requestId,
      payload: {
        agentUserId: body.agentUserId,
        devices: robots.map(robot => ({
          id: robot.robotId,
          type: 'action.devices.types.PETFEEDER',
          traits: [
            'action.devices.traits.OnOff',
            'action.devices.traits.Dispense',
            'action.devices.traits.SensorState'
          ],
          name: {
            name: `펫 급식기 ${robot.location.room}`
          },
          willReportState: true,
          attributes: {
            supportedDispenseItems: [
              { item_name: 'Food', item_name_synonyms: ['kibble', 'meal', '음식'] },
              { item_name: 'Treats', item_name_synonyms: ['snacks', '간식'] },
              { item_name: 'Water', item_name_synonyms: ['물'] }
            ],
            supportedDispensePresets: [
              { preset_name: 'Small portion' },
              { preset_name: 'Regular portion' },
              { preset_name: 'Large portion' }
            ]
          },
          deviceInfo: {
            manufacturer: robot.manufacturer.name,
            model: robot.manufacturer.model,
            hwVersion: robot.manufacturer.hardwareRevision,
            swVersion: robot.manufacturer.firmwareVersion
          }
        }))
      }
    };
  }

  // Google Home EXECUTE 인텐트 처리
  async onExecute(body) {
    const commands = [];

    for (const command of body.inputs[0].payload.commands) {
      for (const device of command.devices) {
        for (const execution of command.execution) {
          let result;

          switch (execution.command) {
            case 'action.devices.commands.Dispense':
              result = await this.handleDispense(device.id, execution.params);
              break;
            case 'action.devices.commands.OnOff':
              result = await this.handleOnOff(device.id, execution.params);
              break;
          }

          commands.push({
            ids: [device.id],
            status: result.success ? 'SUCCESS' : 'ERROR',
            states: result.states || {}
          });
        }
      }
    }

    return {
      requestId: body.requestId,
      payload: { commands }
    };
  }

  async handleDispense(robotId, params) {
    const portionMap = {
      'Small portion': 50,
      'Regular portion': 100,
      'Large portion': 150
    };

    const portion = portionMap[params.presetName] || params.amount || 100;

    const result = await this.robotClient.dispenseFeed({
      robotId: robotId,
      feedingType: 'manual',
      portionSize: portion
    });

    return {
      success: result.dispensed,
      states: {
        isDispensing: false,
        dispenseRemaining: 0
      }
    };
  }

  async handleOnOff(robotId, params) {
    // 로봇 작동 상태 토글
    const result = await this.robotClient.setRobotStatus(
      robotId,
      params.on ? 'online' : 'standby'
    );

    return {
      success: true,
      states: { on: params.on }
    };
  }
}

module.exports = {
  PetCareRobotAlexaSkill,
  PetCareRobotGoogleHome
};
```

---

## 7. 데이터 포맷 모범 사례

### 7.1 데이터 검증 규칙

| 필드 유형 | 검증 | 예제 | 오류 처리 |
|-----------|------|------|----------|
| **Robot ID** | 패턴: PCR-[A-Z0-9]{12} | PCR-ABC123456789 | 400 오류로 거부 |
| **분량 크기** | 범위: 1-500 그램 | 100 | 유효 범위로 클램프 |
| **배터리 수준** | 범위: 0-100 | 85 | 범위 벗어나면 센서 오류 보고 |
| **타임스탬프** | ISO 8601 형식 | 2025-12-18T10:30:00Z | 잘못된 날짜 거부 |
| **참여도 수준** | 범위: 0-100 | 75 | 유효하지 않으면 0으로 기본값 |

### 7.2 데이터 보존 정책

| 데이터 유형 | 보존 기간 | 집계 | 목적 |
|-------------|----------|------|------|
| **실시간 상태** | 7일 | 없음 | 현재 모니터링 |
| **급식 이벤트** | 1년 | 90일 후 일일 요약 | 건강 추적 |
| **놀이 세션** | 6개월 | 30일 후 주간 요약 | 활동 분석 |
| **건강 관찰** | 영구 | 1년 후 월간 요약 | 장기 추세 |
| **오류 로그** | 90일 | 없음 | 문제 해결 |
| **비디오 녹화** | 30일 | 없음 | 개인정보 준수 |

### 7.3 개인정보 및 보안

| 측면 | 요구 사항 | 구현 |
|------|----------|------|
| **데이터 암호화** | 저장 시 AES-256, 전송 시 TLS 1.3 | 모든 데이터 암호화 |
| **액세스 제어** | 역할 기반 권한 | 소유자, 가족, 수의사, 서비스 |
| **비디오 프라이버시** | 선택적 녹화만 | 기본적으로 비활성화 |
| **데이터 공유** | 명시적 동의 필요 | 세분화된 권한 |
| **반려동물 식별** | 로컬 처리 선호 | 가능한 경우 엣지 AI |
| **API 인증** | OAuth 2.0 + API 키 | 관리자용 다중 요소 |

---

## 8. 상호 운용성 표준

### 8.1 호환 표준

- **WIA-PET-HEALTH-PASSPORT**: 반려동물 건강 기록 통합
- **WIA-PET-EMOTION**: 상호작용 중 감정 상태
- **WIA-PET-GENOME**: 품종별 관리 권장 사항
- **WIA-SMARTHOME**: 홈 자동화 통합
- **WIA-AI**: 머신러닝 모델 배포
- **WIA-IOT**: 장치 연결 프로토콜

### 8.2 외부 프로토콜 지원

| 프로토콜 | 사용 사례 | 구현 상태 |
|----------|----------|-----------|
| **MQTT** | IoT 메시징 | 필수 |
| **CoAP** | 제한된 장치 | 선택 사항 |
| **Zigbee** | 저전력 센서 | 선택 사항 |
| **Thread** | 메시 네트워킹 | 선택 사항 |
| **Matter** | 스마트 홈 표준 | 권장 |
| **WebRTC** | 비디오 스트리밍 | 카메라 로봇에 필수 |

---

**弘益人間 (홍익인간)** - 인류와 모든 생명체의 이익을 위하여
© 2025 WIA
MIT License
