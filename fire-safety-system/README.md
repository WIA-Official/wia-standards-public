# WIA-CITY-013: 화재 안전 시스템 표준 🔥

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

건물 및 시설의 화재 감지, 경보, 소화, 대피를 위한 포괄적 표준입니다.

## 🌟 개요

WIA-CITY-013 화재 안전 시스템 표준은 스마트 빌딩과 도시 인프라의 화재 안전을 보장하기 위한 완전한 프레임워크를 제공합니다. 이 표준은 화재 감지, 경보, 소화 시스템, 대피 관리, 그리고 연동 시스템 제어를 포괄합니다.

### 철학

**홍익인간 (弘益人間) (홍익인간)** - "널리 인간을 이롭게 하라"의 철학을 바탕으로, 이 표준은 화재로부터 생명과 재산을 보호하고, 신속한 대응과 안전한 대피를 통해 피해를 최소화하는 것을 목표로 합니다.

## 📊 현황

### 글로벌 화재 안전 현황 (2024)

- **연간 화재 발생**: 전 세계 약 700만 건
- **인명 피해**: 연간 약 30만 명 사망
- **재산 피해**: 연간 약 $3,000억 규모
- **조기 감지 효과**: 화재 사망률 50% 감소
- **스프링클러 효과**: 재산 피해 60-70% 감소

### 주요 문제점

1. 🔥 **늦은 감지**: 화재 초기 단계 감지 실패
2. ⚠️ **불충분한 경보**: 취약 계층에 대한 접근성 부족
3. 🚪 **대피 혼란**: 명확한 대피 경로 안내 부재
4. 🔧 **점검 소홀**: 소화 장비 정기 점검 미흡
5. 🏢 **시스템 비연동**: HVAC, 엘리베이터 등과의 통합 부족

## 🎯 표준의 가치

### 핵심 목표

- ✅ **조기 감지**: 다중 센서 기반 화재 조기 감지
- ✅ **신속한 경보**: 음향, 시각, 문자 등 다양한 경보 수단
- ✅ **자동 소화**: 스프링클러 시스템 자동 작동
- ✅ **안전한 대피**: 최적 대피 경로 실시간 안내
- ✅ **시스템 연동**: HVAC, 엘리베이터, 출입통제 통합 제어

### 기대 효과

- 💙 **생명 보호**: 조기 감지로 화재 사망률 50% 감소
- 💰 **재산 보호**: 자동 소화로 재산 피해 60% 감소
- ⏱️ **신속 대응**: 평균 대응 시간 5분 → 2분 단축
- 🏥 **부상 감소**: 안전한 대피로 부상자 70% 감소

## 📁 저장소 구조

```
fire-safety-system/
├── README.md              # 본 문서
├── install.sh             # 설치 스크립트
├── spec/
│   └── WIA-CITY-013-v1.0.md  # 상세 기술 명세
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts   # TypeScript 타입 정의
│       │   └── index.ts   # SDK 구현
│       └── package.json   # 패키지 정보
└── cli/
    └── fire-safety-system.sh  # CLI 도구
```

## 🔥 화재 감지 시스템

### 센서 종류

#### 1. 연기 감지기 (Smoke Detector)

**광전식 (Photoelectric)**
- 원리: 빛의 산란을 이용한 연기 감지
- 장점: 느린 연소 화재 조기 감지
- 적용: 침실, 거실, 복도

**이온화식 (Ionization)**
- 원리: 이온화된 공기의 전류 변화 감지
- 장점: 빠른 연소 화재 조기 감지
- 적용: 주방, 다락방

**측정 지표**
- 연기 농도: ppm (parts per million)
- 가시거리: 미터 (m)
- 경보 임계값: 일반적으로 4-12% obscuration/ft

#### 2. 열 감지기 (Heat Detector)

**정온식 (Fixed Temperature)**
- 설정 온도 도달 시 경보 (보통 57°C, 79°C)
- 적용: 주방, 차고, 보일러실

**차동식 (Rate-of-Rise)**
- 온도 상승률 감지 (보통 8-10°C/분)
- 빠른 화재 확산 감지

**측정 지표**
- 현재 온도: °C
- 온도 상승률: °C/분
- 경보 임계값: 57-79°C

#### 3. 화염 감지기 (Flame Detector)

**UV 감지기**
- 자외선 영역의 화염 감지
- 빠른 응답 속도 (< 0.1초)

**IR 감지기**
- 적외선 영역의 화염 감지
- 화염의 깜빡임 패턴 분석

**UV/IR 복합**
- 오경보 최소화
- 높은 신뢰도

**측정 지표**
- UV 강도
- IR 강도
- 감지 범위: 일반적으로 10-50m

#### 4. 일산화탄소 감지기 (CO Detector)

**전기화학식**
- CO 농도 측정 (ppm)
- 경보 기준:
  - 50 ppm: 8시간 노출 시 경보
  - 100 ppm: 90분 노출 시 경보
  - 400 ppm: 즉시 경보

## 💧 스프링클러 시스템

### 시스템 종류

#### 1. 습식 (Wet Pipe System)

**특징**
- 배관에 항상 물이 채워져 있음
- 가장 일반적이고 신뢰성 높음
- 응답 시간 가장 빠름

**장점**
- 간단한 구조
- 유지보수 용이
- 저렴한 비용

**적용**
- 난방 건물
- 사무실, 상업 시설

#### 2. 건식 (Dry Pipe System)

**특징**
- 배관에 압축 공기 또는 질소 충전
- 동결 위험이 있는 장소에 사용

**장점**
- 동결 방지
- 비가열 공간 적용 가능

**적용**
- 주차장, 창고
- 극한 온도 환경

#### 3. 준비작동식 (Preaction System)

**특징**
- 이중 안전 장치
- 화재 감지 + 스프링클러 헤드 작동

**장점**
- 오작동 방지
- 수손 위험 최소화

**적용**
- 데이터센터
- 박물관, 도서관
- 고가 장비 시설

#### 4. 일제살수식 (Deluge System)

**특징**
- 모든 헤드 동시 개방
- 대량 방수

**장점**
- 빠른 화재 진압
- 광범위 화재 대응

**적용**
- 화학 공장
- 유류 저장 시설
- 항공기 격납고

### 스프링클러 헤드

**작동 온도 분류**
- 일반용: 57-77°C (주황색)
- 중간용: 79-107°C (빨간색)
- 고온용: 121-149°C (노란색)
- 초고온용: 163-260°C (파란색)

**방호 면적**
- 일반 건물: 9-18 m²/헤드
- 위험 건물: 6-9 m²/헤드

**유량**
- 일반용: 80-115 L/min
- 고감도용: 40-60 L/min

## 🚨 화재 경보 시스템

### 경보 종류

#### 1. 음향 경보 (Audible Alarm)

**사이렌**
- 음압: 최소 75 dBA @ 3m
- 최대: 120 dBA
- 주파수: 500-2,500 Hz

**벨**
- 음압: 최소 75 dBA @ 3m
- 펄스 패턴: 1초 온, 1초 오프

#### 2. 시각 경보 (Visual Alarm)

**스트로브 라이트**
- 플래시 속도: 1-2 Hz
- 광도: 최소 15-110 candela
- 색상: 일반적으로 백색 또는 적색

**적용**
- 청각 장애인
- 소음이 큰 환경
- 수면 공간

#### 3. 음성 경보 (Voice Alarm)

**특징**
- 명확한 안내 메시지
- 층별, 구역별 선택적 방송

**메시지 예시**
```
"화재가 발생했습니다. 침착하게 가까운 비상구로 대피하시기 바랍니다."
"Fire alarm activated. Please evacuate calmly using the nearest exit."
```

#### 4. 문자 알림 (Text Notification)

**채널**
- SMS
- 모바일 앱 푸시
- 이메일
- 건물 관리 시스템

## 🚪 대피 시스템

### 비상구 종류

#### 1. 주 출구 (Main Exit)
- 일상적으로 사용하는 출구
- 넓은 폭 (최소 1.8m)
- 높은 수용 능력

#### 2. 비상구 (Emergency Exit)
- 화재 시 전용 출구
- 항상 열림 상태 유지
- 비상 조명 설치

#### 3. 화재 대피 계단 (Fire Escape)
- 내화 구조 (최소 2시간)
- 방연 설비
- 독립 환기 시스템

#### 4. 구조용 창 (Rescue Window)
- 최소 크기: 0.5m × 0.6m
- 외부 접근 가능
- 명확한 표시

### 대피 경로 설계

#### 설계 원칙

**이중 경로**
- 최소 2개의 독립적 대피 경로
- 한 경로 차단 시 대안 경로 사용

**이동 거리**
- 일반 건물: 최대 45m
- 스프링클러 설치 건물: 최대 60m
- 고위험 시설: 최대 23m

**통행 능력**
- 계단 폭 1.1m: 60명/분
- 계단 폭 1.4m: 100명/분
- 출구 폭 0.9m: 50명/분

#### 유도 시스템

**유도등**
- 피난구 유도등: 녹색, 출구 표시
- 통로 유도등: 녹색, 화살표
- 비상조명: 백색, 최소 1 lux

**디지털 사이니지**
- 실시간 경로 안내
- 다국어 지원
- 장애인 접근 정보

## 🧯 소화 장비

### 소화기

#### 종류별 적용

| 소화기 종류 | 적용 화재 등급 | 소화 원리 | 용량 |
|------------|---------------|----------|------|
| 물 소화기 | A급 (일반 화재) | 냉각 | 6-9 L |
| 포말 소화기 | A, B급 (유류) | 냉각 + 질식 | 6-9 L |
| 분말 소화기 | A, B, C급 | 질식 + 억제 | 3-10 kg |
| CO₂ 소화기 | B, C급 (전기) | 질식 | 2-5 kg |
| 청정 소화기 | A, B, C급 | 억제 | 2-9 kg |

#### 화재 등급

**A급 (일반 화재)**
- 목재, 종이, 섬유, 플라스틱
- 냉각 소화

**B급 (유류 화재)**
- 휘발유, 오일, 페인트, 용제
- 질식 소화

**C급 (전기 화재)**
- 전기 장비, 배선
- 비전도성 소화제

**D급 (금속 화재)**
- 마그네슘, 나트륨, 리튬
- 특수 소화제

**K급 (주방 화재)**
- 식용유, 동물성 기름
- 습식 화학 소화제

### 소화전

#### 실내 소화전

**구성**
- 호스: 15-25m 길이
- 노즐: 직사, 무상 겸용
- 밸브: 볼 밸브 또는 게이트 밸브

**수압**
- 최소: 3.5 bar
- 권장: 5-7 bar
- 유량: 130-260 L/min

#### 옥외 소화전

**구성**
- 지상식 또는 지하식
- 65mm 또는 100mm 연결구
- 동결 방지 설계

**수압**
- 최소: 4.5 bar
- 유량: 500-1,000 L/min

## 🚪 방화 구획

### 방화문

#### 내화 등급

**FRR-30 (30분 내화)**
- 일반 사무실
- 복도 구획

**FRR-60 (60분 내화)**
- 계단실 출입구
- 보일러실

**FRR-90 (90분 내화)**
- 수직 샤프트
- 위험물 저장실

**FRR-120 (120분 내화)**
- 소방차 진입로
- 특수 방호 구역

#### 자동 폐쇄 장치

**유압식 도어 클로저**
- 천천히 닫힘
- 마지막 10cm 빠른 폐쇄
- 조절 가능

**전자기식 도어 홀더**
- 화재 시 자동 해제
- 방화문 자동 폐쇄

### 방화벽

**내화 성능**
- 최소 2시간 내화
- 비내력벽: 1-2시간
- 내력벽: 3-4시간

**구조**
- 콘크리트 블록 (200mm+)
- 철근 콘크리트 (150mm+)
- 경량 내화 보드 (이중 레이어)

## 🔗 시스템 연동

### HVAC 제어

#### 화재 시 동작

**배기 팬 (Exhaust Fan)**
- 화재 구역: 가동 (연기 배출)
- 인접 구역: 정지 (연기 확산 방지)

**급기 팬 (Supply Fan)**
- 대피 경로: 가동 (가압, 연기 유입 방지)
- 화재 구역: 정지

**댐퍼 (Damper)**
- 방화 댐퍼: 자동 폐쇄
- 연기 댐퍼: 자동 개방 (배연)

#### 배연 모드

**자연 배연**
- 배연창 자동 개방
- 상부 개구부 (천장 면적의 2-5%)

**기계 배연**
- 배연 팬 가동
- 풍량: 6-10 회/시간

### 엘리베이터 제어

#### 화재 모드 동작

**일반 엘리베이터**
1. 현재 층에서 문 즉시 개방
2. 승객 하차
3. 지정 대피 층으로 이동 (보통 1층)
4. 문 개방 후 정지
5. 소방대원 전용 대기

**소방 전용 엘리베이터**
- 소방대원 키로만 작동
- 화재 층 직행
- 비상 전원 공급

### 출입 통제

#### 화재 시 동작

**전자 도어락**
- 모든 출구 자동 해제
- 내부에서 자유롭게 개방
- 외부 진입 차단 (보안 유지)

**게이트/턴스타일**
- 자동 개방
- 양방향 통행 허용

## 📡 API 인터페이스

### 화재 감지

```http
POST /api/v1/fire/detect
Content-Type: application/json

{
  "sensorId": "SMOKE-FL2-A001",
  "sensorType": "SMOKE",
  "location": {
    "floor": "2F",
    "zone": "A",
    "building": "Main Building"
  },
  "readings": {
    "smokeLevel_ppm": 250,
    "temperature_C": 85
  },
  "alarmTriggered": true
}

Response:
{
  "success": true,
  "data": {
    "eventId": "FIRE-2025-001234",
    "timestamp": "2025-12-25T10:30:00Z",
    "severity": "CRITICAL",
    "autoActionsTriggered": [
      "ALARM_ACTIVATED",
      "SPRINKLER_ARMED",
      "FIRE_DEPT_NOTIFIED"
    ]
  }
}
```

### 스프링클러 작동

```http
POST /api/v1/sprinklers/activate

{
  "systemId": "SPR-001",
  "zoneId": "ZONE-A"
}

Response:
{
  "success": true,
  "data": {
    "systemId": "SPR-001",
    "activated": true,
    "activatedHeads": ["SPR-A-001", "SPR-A-002", "SPR-A-003"],
    "waterPressure_bar": 6.5,
    "waterFlow_lpm": 450,
    "timestamp": "2025-12-25T10:30:15Z"
  }
}
```

### 대피 경로 조회

```http
POST /api/v1/evacuation/route

{
  "startLocation": {
    "floor": "3F",
    "zone": "B"
  },
  "accessible": true
}

Response:
{
  "success": true,
  "data": {
    "routeId": "ROUTE-3FB-001",
    "distance_m": 45,
    "estimatedTime_sec": 90,
    "status": "CLEAR",
    "exits": [
      {
        "exitId": "EXIT-3F-EAST",
        "type": "EMERGENCY_EXIT",
        "status": "CLEAR",
        "distance_m": 30
      },
      {
        "exitId": "STAIR-3F-A",
        "type": "FIRE_ESCAPE",
        "status": "CLEAR",
        "distance_m": 15
      }
    ]
  }
}
```

## 🚀 빠른 시작

### 1. 설치

```bash
# 저장소 복제
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/fire-safety-system

# 설치
./install.sh
```

### 2. CLI 사용

```bash
# 화재 감지 보고
./cli/fire-safety-system.sh detect \
  --sensor-id SMOKE-FL2-A001 \
  --sensor-type SMOKE \
  --floor 2F \
  --zone A \
  --smoke-level 250

# 스프링클러 작동
./cli/fire-safety-system.sh activate-sprinkler \
  --system-id SPR-001 \
  --zone-id ZONE-A

# 대피 경로 조회
./cli/fire-safety-system.sh evacuation-route \
  --floor 3F \
  --zone B \
  --accessible

# 시스템 상태 점검
./cli/fire-safety-system.sh health-check
```

### 3. TypeScript SDK

```typescript
import { FireSafetySDK } from '@wia/city-fire-safety';

const client = new FireSafetySDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-013/v1'
});

// 화재 감지
const event = await client.detectFire({
  sensorId: 'SMOKE-FL2-A001',
  sensorType: 'SMOKE',
  location: { floor: '2F', zone: 'A' },
  readings: {
    smokeLevel_ppm: 250,
    temperature_C: 85
  },
  alarmTriggered: true
});

console.log('Fire event:', event.eventId);

// 스프링클러 작동
const result = await client.activateSprinkler('SPR-001', 'ZONE-A');
console.log('Sprinkler activated:', result.activated);

// 대피 경로
const route = await client.getEvacuationRoute(
  { floor: '3F', zone: 'B' },
  true  // accessible route
);
console.log('Evacuation route:', route.routeId);
console.log('Distance:', route.distance_m, 'm');
console.log('Estimated time:', route.estimatedTime_sec, 'sec');
```

## 📊 성과 지표 (KPI)

### 시스템 성능

**감지 시간**
- 목표: < 30초 (화재 발생 → 감지)
- 우수: < 15초

**경보 시간**
- 목표: < 10초 (감지 → 경보)
- 우수: < 5초

**대응 시간**
- 목표: < 2분 (경보 → 초기 진압)
- 우수: < 1분

### 시스템 가용성

**센서 가동률**
- 목표: > 99%
- 우수: > 99.9%

**경보 시스템 가용성**
- 목표: > 99.5%
- 우수: > 99.99%

**스프링클러 시스템 신뢰도**
- 목표: > 95%
- 우수: > 98%

### 안전 성과

**화재 사망률 감소**
- 목표: 50% 감소 (기준 대비)
- 우수: 70% 감소

**재산 피해 감소**
- 목표: 60% 감소
- 우수: 80% 감소

**오경보율**
- 목표: < 5%
- 우수: < 2%

## 🏅 인증 및 준수

### 국제 표준

**NFPA (National Fire Protection Association)**
- NFPA 72: 화재 경보 시스템
- NFPA 13: 스프링클러 시스템
- NFPA 101: 생명 안전 코드

**EN (European Norm)**
- EN 54: 화재 감지 및 경보 시스템
- EN 12845: 스프링클러 시스템

**BS (British Standard)**
- BS 5839: 화재 감지 및 경보 시스템
- BS 9999: 화재 안전 설계

### 인증

**UL (Underwriters Laboratories)**
- UL 268: 연기 감지기
- UL 217: 단독형 연기 경보기
- UL 199: 스프링클러 헤드

**FM Approvals**
- FM 3210: 스프링클러 시스템
- FM 5560: 화재 경보 시스템

## 🤝 기여

이 표준은 화재 안전 커뮤니티의 기여를 환영합니다:

- 기술 피드백 및 개선 제안
- 사례 연구 및 구현 사례
- 추가 언어 번역
- 교육 자료 및 튜토리얼

## 📜 라이선스

© 2025 SmileStory Inc. / WIA - World Certification Industry Association

이 표준은 MIT 라이선스 하에 배포됩니다.

## 🔗 관련 표준

- **WIA-CITY-009**: 3D Printing in Construction (3D 프린팅 건축)
- **WIA-CITY-010**: Smart Lighting System (스마트 조명)
- **WIA-CITY-011**: HVAC System (냉난방공조)
- **WIA-CITY-012**: Building Energy Management (건물 에너지 관리)
- **WIA-ENE**: Energy & Environment Standards (에너지 환경 표준)

## 📞 문의 및 지원

### 공식 채널

- **웹사이트**: https://wia-official.org
- **이메일**: standards@wia-official.org
- **GitHub**: https://github.com/WIA-Official/wia-standards

### 커뮤니티

- 기술 포럼: 구현 관련 질문
- 정기 웨비나 및 교육 세션
- 연례 WIA 표준 컨퍼런스
- 지역별 워킹 그룹

---

## 홍익인간 (弘益人間) (홍익인간)

**널리 인간을 이롭게 하라**

WIA-CITY-013 표준은 홍익인간 (弘益人間)의 철학을 통해 화재로부터 생명과 재산을 보호합니다. 조기 감지, 신속한 경보, 자동 소화, 안전한 대피를 통해 화재 피해를 최소화하고, 모두가 안전한 건물 환경을 만들어갑니다.

**함께 만드는 안전한 세상, 함께 지키는 소중한 생명**

---

*문서 버전: 1.0.0*
*최종 업데이트: 2025-12-25*
*상태: Active*
