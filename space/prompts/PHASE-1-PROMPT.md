# Phase 1: Data Format Standard
## Claude Code 작업 프롬프트

---

**Standard**: WIA Space
**Phase**: 1 of 4
**목표**: 우주 기술 데이터의 표준 형식 정의
**난이도**: ★★★☆☆
**예상 작업량**: 스펙 문서 1개 + JSON Schema + 예제 파일

---

## 🎯 Phase 1 목표

### 핵심 질문
```
"다이슨 구체, 화성 테라포밍, 워프 드라이브, 우주 엘리베이터,
 소행성 채굴, 성간 여행...

 각각 다른 형식으로 데이터를 정의하면 호환이 안 된다.

 이걸 하나의 표준 형식으로 통일할 수 있을까?"
```

### 목표
```
우주 기술 유형에 관계없이
모든 프로젝트가 동일한 JSON 형식으로 데이터를 표현하도록
Data Format Standard를 정의한다.
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 우주 기술 조사

아래 기술 유형별로 웹서치하여 실제 데이터 형식을 조사하세요:

| 기술 유형 | 조사 대상 | 웹서치 키워드 |
|----------|----------|--------------|
| **Dyson Sphere** | 에너지 수집 이론 | "Dyson sphere energy collection calculation" |
| **Mars Terraforming** | 화성 환경 개조 | "Mars terraforming atmospheric data model" |
| **Warp Drive** | 알쿠비에레 드라이브 | "Alcubierre warp drive metric parameters" |
| **Space Elevator** | 궤도 엘리베이터 | "space elevator carbon nanotube specifications" |
| **Asteroid Mining** | 소행성 자원 추출 | "asteroid mining resource estimation data" |
| **Interstellar Travel** | 성간 탐사 | "interstellar mission planning parameters" |

### 2단계: 기존 표준/기관 조사

| 표준/기관 | 조사 내용 | 웹서치 키워드 |
|----------|----------|--------------|
| **NASA** | 미션 데이터 형식 | "NASA mission data format JSON" |
| **ESA** | 우주 데이터 표준 | "ESA space data standard" |
| **CCSDS** | 우주 데이터 시스템 | "CCSDS data format specification" |
| **SpaceX** | 발사체 데이터 | "SpaceX Starship specifications" |

### 3단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-1.md`에 다음을 정리:

```markdown
# Phase 1 사전 조사 결과

## 1. Dyson Sphere

### 이론적 배경
- Dyson Sphere 유형: [Type I, II, III]
- 에너지 계산 방식: [조사 내용]
- 필요 데이터 필드: [분석]

### 기존 연구 데이터
- NASA 논문 참조: [조사 내용]
...

## 2. Mars Terraforming

### 기술 현황
- 현재 화성 대기 데이터: [조사 내용]
- 테라포밍 단계: [조사 내용]
...

## 3. Warp Drive
...

## 4. Space Elevator
...

## 5. Asteroid Mining
...

## 6. Interstellar Travel
...

## 7. 공통점 분석
- 모든 기술에 공통으로 필요한 필드: [분석]
- 기술별 고유 필드: [분석]

## 8. 결론
- 표준 형식 설계 방향: [제안]
```

---

## 🏗️ 표준 설계

### 기본 구조 (제안)

```json
{
  "$schema": "https://wia.live/schemas/space/project.schema.json",
  "version": "1.0.0",
  "project": {
    "id": "고유 ID",
    "name": "프로젝트명",
    "type": "기술 유형",
    "status": "상태",
    "trl": "기술 성숙도 레벨 (1-9)"
  },
  "spec": {
    "기술별 고유 데이터"
  },
  "timeline": {
    "estimated_start": "시작 예정",
    "estimated_completion": "완료 예정"
  },
  "resources": {
    "budget_usd": 예산,
    "energy_requirements": 에너지 요구량,
    "materials": [필요 자재]
  },
  "meta": {
    "created_at": "생성일",
    "updated_at": "수정일",
    "author": "작성자"
  }
}
```

### 기술별 `spec` 필드 정의

#### Dyson Sphere
```json
{
  "spec": {
    "dyson_type": "swarm",           // "swarm", "bubble", "shell"
    "star": {
      "name": "Sol",
      "type": "G2V",
      "luminosity_watts": 3.828e26,
      "radius_km": 696340
    },
    "collectors": {
      "count": 1000000000,
      "individual_area_km2": 1000,
      "efficiency": 0.85
    },
    "orbit": {
      "radius_au": 1.0,
      "period_days": 365.25
    },
    "energy_output": {
      "total_watts": 3.25e26,
      "usable_watts": 2.76e26
    }
  }
}
```

#### Mars Terraforming
```json
{
  "spec": {
    "phase": "atmosphere_generation",  // "warming", "atmosphere_generation", "water_cycle", "biosphere"
    "current_conditions": {
      "avg_temp_celsius": -60,
      "pressure_kpa": 0.636,
      "co2_percent": 95.3,
      "o2_percent": 0.13
    },
    "target_conditions": {
      "avg_temp_celsius": 15,
      "pressure_kpa": 101.3,
      "o2_percent": 21.0,
      "n2_percent": 78.0
    },
    "methods": [
      {
        "name": "polar_cap_melting",
        "energy_required_joules": 1e20,
        "estimated_duration_years": 50
      }
    ]
  }
}
```

#### Warp Drive
```json
{
  "spec": {
    "drive_type": "alcubierre",
    "warp_factor": 1.5,              // c 배수
    "bubble": {
      "radius_meters": 200,
      "wall_thickness_meters": 10
    },
    "energy_requirements": {
      "exotic_matter_kg": -1e9,     // 음수 = 음에너지
      "power_watts": 1e18
    },
    "spacetime_metrics": {
      "expansion_factor": 2.0,
      "contraction_factor": 0.5
    }
  }
}
```

#### Space Elevator
```json
{
  "spec": {
    "location": {
      "latitude": 0.0,
      "longitude": -80.0,
      "base_name": "Pacific Platform"
    },
    "tether": {
      "material": "carbon_nanotube",
      "length_km": 100000,
      "diameter_mm": 5,
      "tensile_strength_gpa": 130
    },
    "counterweight": {
      "mass_kg": 1e10,
      "altitude_km": 100000
    },
    "climbers": {
      "max_payload_kg": 20000,
      "speed_kmh": 200,
      "travel_time_hours": 500
    }
  }
}
```

#### Asteroid Mining
```json
{
  "spec": {
    "target": {
      "name": "16 Psyche",
      "type": "M-type",
      "diameter_km": 226,
      "orbit": {
        "semi_major_axis_au": 2.92,
        "eccentricity": 0.14,
        "inclination_deg": 3.1
      }
    },
    "resources": {
      "iron_kg": 1e19,
      "nickel_kg": 1e18,
      "gold_kg": 1e14,
      "platinum_kg": 1e13
    },
    "extraction": {
      "method": "surface_mining",
      "annual_output_kg": 1e9,
      "equipment_mass_kg": 50000
    }
  }
}
```

#### Interstellar Travel
```json
{
  "spec": {
    "mission_type": "flyby",          // "flyby", "orbital", "landing"
    "destination": {
      "name": "Proxima Centauri b",
      "distance_ly": 4.24,
      "star": "Proxima Centauri"
    },
    "spacecraft": {
      "name": "Starshot",
      "mass_kg": 1,
      "propulsion": "light_sail",
      "cruise_velocity_c": 0.2
    },
    "trajectory": {
      "launch_date": "2050-01-01",
      "travel_time_years": 21.2,
      "arrival_date": "2071-03-15"
    }
  }
}
```

---

## 📁 산출물 목록

Phase 1 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-1.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-1-DATA-FORMAT.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 기본 구조 (Base Structure)
4. 기술별 데이터 형식 (Technology-Specific Data)
   - Dyson Sphere
   - Mars Terraforming
   - Warp Drive
   - Space Elevator
   - Asteroid Mining
   - Interstellar Travel
5. 기술 성숙도 (TRL - Technology Readiness Level)
6. 확장성 (Extensibility)
7. 버전 관리 (Versioning)
8. 예제 (Examples)
9. 참고문헌 (References)
```

### 3. JSON Schema 파일
```
/spec/schemas/
├── project.schema.json           (기본 프로젝트 스키마)
├── technology.schema.json        (기술 유형 정의)
├── dyson-sphere.schema.json
├── mars-terraforming.schema.json
├── warp-drive.schema.json
├── space-elevator.schema.json
├── asteroid-mining.schema.json
└── interstellar-travel.schema.json
```

### 4. 예제 데이터 파일
```
/examples/sample-data/
├── dyson-swarm-example.json
├── mars-terraform-example.json
├── alcubierre-drive-example.json
├── pacific-elevator-example.json
├── psyche-mining-example.json
└── starshot-mission-example.json
```

---

## ✅ 완료 체크리스트

Phase 1 완료 전 확인:

```
□ 웹서치로 6개 우주 기술 데이터 형식 조사 완료
□ /spec/RESEARCH-PHASE-1.md 작성 완료
□ /spec/PHASE-1-DATA-FORMAT.md 작성 완료
□ JSON Schema 파일 생성 완료 (기본 + 기술별 6개)
□ 예제 데이터 파일 생성 완료 (6개)
□ JSON Schema로 예제 데이터 검증 통과
□ README 업데이트 (Phase 1 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 우주 기술 및 기존 표준 조사
   ↓
2. /spec/RESEARCH-PHASE-1.md 작성
   ↓
3. 조사 결과 바탕으로 표준 설계
   ↓
4. /spec/PHASE-1-DATA-FORMAT.md 작성
   ↓
5. JSON Schema 파일 생성
   ↓
6. 예제 데이터 파일 생성
   ↓
7. 스키마 검증 테스트
   ↓
8. 완료 체크리스트 확인
   ↓
9. Phase 2 시작 가능
```

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ NASA, ESA 등 실제 우주 기관 데이터 형식 참조
✅ 모든 필드에 명확한 단위 명시 (SI 단위 우선)
✅ 확장 가능한 구조로 설계 (미래 기술 유형 고려)
✅ JSON Schema는 draft-07 표준 사용
✅ 과학적으로 검증된 계산 방식 사용
```

### DON'T (하지 말 것)

```
❌ 추측으로 데이터 형식 정의 (반드시 조사 후)
❌ SF 소설 설정에만 의존하는 설계
❌ 필수 필드와 선택 필드 구분 없이 작성
❌ 과학적 근거 없는 수치 사용
```

---

## 🚀 작업 시작

이제 Phase 1 작업을 시작하세요.

첫 번째 단계: **웹서치로 Dyson Sphere 에너지 수집 계산 방식 조사**

```
검색 키워드: "Dyson sphere energy collection Freeman Dyson calculation"
```

화이팅! 🚀

---

<div align="center">

**Phase 1 of 4**

Data Format Standard

🌟 弘益人間 - Benefit All Humanity 🌟

</div>
