# 1단계: 반려동물 추적 데이터 형식 사양

## WIA-PET-TRACKING 데이터 형식 표준

**버전**: 1.0.0  
**날짜**: 2025-12-25  
**상태**: 활성  
**표준 ID**: WIA-PET-008-PHASE1  
**기본 색상**: #F59E0B (앰버)

---

## 1. 개요

### 1.1 목적

WIA-PET-TRACKING은 실시간 반려동물 위치 추적, 지오펜싱 및 분실 반려동물 복구 시스템을 위한 포괄적인 표준입니다. 이 1단계 사양은 상호 운용성을 위해 모든 구현이 지원해야 하는 핵심 데이터 형식 및 구조를 정의합니다.

**핵심 목표**:
- 모든 구현에서 위치 데이터 표현 표준화
- 기기, 앱 및 서비스 간의 원활한 데이터 교환 가능
- 여러 GNSS 시스템 및 위치 확인 방법 지원
- 유연하지만 일관된 데이터 구조 제공
- 데이터 처리에서 프라이버시 및 보안 보장

---

## 2. 좌표 시스템 표준

### 2.1 WGS84 데이텀

모든 위치 데이터는 WGS84(World Geodetic System 1984) 좌표 참조 시스템을 사용해야 합니다.

**요구사항**:
- 위도: 십진수 도, -90.0 ~ +90.0
- 경도: 십진수 도, -180.0 ~ +180.0
- 정밀도: 최소 소수점 6자리 (~0.11m 해상도)
- 형식: 십진수 도 (도-분-초 아님)

```json
{
  "latitude": 37.774929,
  "longitude": -122.419418
}
```

---

## 3. 핵심 위치 데이터 구조

### 3.1 LocationUpdate 스키마

```typescript
interface LocationUpdate {
  // 필수 필드
  trackerId: string;           // 고유 추적기 기기 ID
  timestamp: string;           // ISO 8601 UTC 타임스탬프
  location: Location;          // 위치 데이터
  
  // 선택적 필드
  petId?: string;              // 연관된 반려동물 ID
  positioning?: PositioningInfo;
  device?: DeviceStatus;
  metadata?: Metadata;
}

interface Location {
  // 필수
  latitude: number;            // WGS84 십진수 도
  longitude: number;           // WGS84 십진수 도
  accuracy: number;            // 수평 정확도(미터, 68% 신뢰도)
  
  // 선택적
  altitude?: number;           // WGS84 타원체 위의 미터
  altitudeAccuracy?: number;   // 수직 정확도(미터)
  heading?: number;            // 이동 방향(0-360도, 0=북쪽)
  speed?: number;              // 지면 속도(m/s)
}
```

### 3.2 완전한 예제

```json
{
  "trackerId": "TRK-ABC123",
  "petId": "PET-789XYZ",
  "timestamp": "2025-12-25T10:30:45.123Z",
  "location": {
    "latitude": 37.774929,
    "longitude": -122.419418,
    "accuracy": 8.5,
    "altitude": 52.3,
    "heading": 275.5,
    "speed": 1.2
  },
  "positioning": {
    "method": "multi-gnss",
    "gnss": {
      "satellites": 12,
      "systems": ["GPS", "GLONASS", "Galileo"],
      "hdop": 0.9
    },
    "confidence": 0.95
  },
  "device": {
    "battery": {
      "level": 78,
      "charging": false,
      "voltage": 3.87,
      "estimatedHours": 36
    },
    "network": {
      "type": "4G-LTE",
      "signal": -68,
      "carrier": "Global Carrier"
    }
  }
}
```

---

## 4. 지오펜스 데이터 구조

### 4.1 원형 지오펜스

```json
{
  "geofenceId": "GEO-HOME-001",
  "name": "집",
  "type": "circular",
  "active": true,
  "center": {
    "latitude": 37.774929,
    "longitude": -122.419418
  },
  "radius": 100,
  "triggers": {
    "entry": true,
    "exit": true,
    "dwell": {
      "enabled": true,
      "duration": 300
    }
  }
}
```

---

## 5. 타임스탬프 표준

### 5.1 ISO 8601 형식

모든 타임스탬프는 UTC 시간대에서 ISO 8601 형식을 사용해야 합니다:

**형식**: `YYYY-MM-DDTHH:MM:SS.sssZ`

**요구사항**:
- UTC 시간대 (Z 접미사)
- 24시간 형식
- 밀리초 정밀도 권장
- 로컬 시간대 없음

---

## 6. 오류 및 상태 코드

| 코드 | 상태 | 설명 |
|------|------|------|
| 1000 | OK | 작업 성공 |
| 1001 | LOCATION_UPDATED | 위치 성공적으로 업데이트됨 |
| 2000 | WARNING | 중요하지 않은 경고 |
| 2001 | LOW_BATTERY | 배터리가 임계값 미만 |
| 3000 | ERROR | 일반 오류 |
| 3001 | INVALID_DATA | 데이터 검증 실패 |
| 4000 | CRITICAL | 중요한 시스템 오류 |

---

**弘益人間 · 널리 인간을 이롭게 하라**  
© 2025 WIA - World Certification Industry Association | MIT 라이선스
