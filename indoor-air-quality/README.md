# WIA-ENE-027: 실내 공기질 표준 🏠

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

실내 공간의 공기질 모니터링, 분석, 관리를 위한 포괄적 표준입니다.

## 🌟 개요

WIA-ENE-027 실내 공기질 표준은 주거, 사무실, 교실, 병원 등 실내 공간의 공기질을 체계적으로 모니터링하고 관리하기 위한 완전한 프레임워크를 제공합니다. 이 표준은 센서 데이터 수집, HVAC 연동, 건강 영향 평가, 환기 최적화 전 과정을 포괄합니다.

### 철학

**홍익인간 (弘益人間) (홍익인간)** - "널리 인간을 이롭게 하라"의 철학을 바탕으로, 이 표준은 모든 사람이 건강한 실내 공기를 누릴 수 있도록 하는 것을 목표로 합니다. 우리는 하루의 90% 이상을 실내에서 보내며, 실내 공기질은 건강과 생산성에 직접적인 영향을 미칩니다.

## 📊 실내 공기질의 중요성

### 현황 (2024)
- **실내 체류 시간**: 하루의 90% 이상
- **실내 오염도**: 실외보다 2~5배 높을 수 있음
- **건강 영향**: 호흡기 질환, 알레르기, 천식 악화
- **경제적 손실**: 연간 $160억 (생산성 저하)

### 주요 문제점
1. 😷 **미세먼지 (PM2.5)**: 호흡기 및 심혈관계 질환
2. 💨 **이산화탄소 (CO₂)**: 집중력 저하, 피로감 증가
3. 🧪 **휘발성유기화합물 (VOCs)**: 새집증후군, 화학물질 노출
4. ☢️ **라돈 (Radon)**: 폐암 2위 원인 (흡연 다음)
5. 💧 **습도 문제**: 곰팡이, 진드기 증식
6. 🌬️ **환기 부족**: 오염물질 축적

## 🎯 표준의 가치

### 핵심 목표
- ✅ **실시간 모니터링**: 다양한 공기질 파라미터 실시간 측정
- ✅ **건강 보호**: 건강 위험 평가 및 조기 경보
- ✅ **HVAC 최적화**: 에너지 효율적 환기 및 공조 제어
- ✅ **기준 준수**: 국내외 공기질 기준 자동 검증
- ✅ **데이터 기반 의사결정**: 과학적 근거 기반 관리

### 기대 효과
- 🏥 **건강**: 호흡기 질환 30% 감소
- 💼 **생산성**: 작업 효율 8-11% 향상
- ⚡ **에너지**: HVAC 에너지 비용 15-25% 절감
- 👨‍👩‍👧‍👦 **웰빙**: 재실자 만족도 40% 증가

## 📁 저장소 구조

```
indoor-air-quality/
├── README.md              # 본 문서
├── install.sh             # 설치 스크립트
├── spec/
│   └── WIA-ENE-027-v1.0.md  # 상세 기술 명세
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts   # TypeScript 타입 정의
│       │   └── index.ts   # SDK 구현
│       └── package.json   # 패키지 정보
└── cli/
    └── indoor-air-quality.sh  # CLI 도구
```

## 🔬 측정 파라미터

### 1. 미세먼지 (Particulate Matter)
```
PM2.5: 2.5μm 이하 초미세먼지
  - WHO 기준: 15 μg/m³ (24시간 평균)
  - 건강 영향: 폐 깊숙이 침투, 심혈관계 질환

PM10: 10μm 이하 미세먼지
  - WHO 기준: 45 μg/m³ (24시간 평균)
  - 건강 영향: 호흡기 자극

PM1.0: 1μm 이하 극초미세먼지 (선택)
  - 폐포까지 침투, 혈류 유입 가능
```

### 2. 가스 농도
```
CO₂ (이산화탄소):
  - 쾌적: < 800 ppm
  - 보통: 800-1,000 ppm
  - 나쁨: 1,000-1,500 ppm
  - 매우 나쁨: > 1,500 ppm
  - 영향: 집중력, 의사결정 능력 저하

CO (일산화탄소):
  - 기준: < 9 ppm (8시간 평균)
  - 위험: 산소 운반 방해, 두통, 현기증

NO₂ (이산화질소):
  - 기준: < 40 μg/m³ (연평균)
  - 영향: 기도 염증, 천식 악화

O₃ (오존):
  - 기준: < 100 μg/m³ (8시간 평균)
  - 영향: 폐 조직 손상
```

### 3. 휘발성유기화합물 (VOCs)
```
TVOC (총휘발성유기화합물):
  - 우수: < 220 μg/m³
  - 양호: 220-660 μg/m³
  - 보통: 660-1,200 μg/m³
  - 나쁨: 1,200-2,200 μg/m³
  - 매우 나쁨: > 2,200 μg/m³

포름알데히드:
  - 기준: < 100 μg/m³
  - 출처: 건축 자재, 가구, 접착제
  - 영향: 발암물질 (WHO 분류)

벤젠, 톨루엔, 자일렌:
  - 출처: 페인트, 용제, 세제
  - 영향: 중추신경계, 간, 신장
```

### 4. 라돈 (Radon)
```
측정 단위: Bq/m³ 또는 pCi/L
  - 안전: < 100 Bq/m³ (< 2.7 pCi/L)
  - 조치 권장: 100-200 Bq/m³
  - 조치 필수: > 200 Bq/m³ (> 5.4 pCi/L)

특징:
  - 무색, 무취 방사성 기체
  - 토양, 암반에서 방출
  - 폐암 2위 원인 (흡연 다음)
  - 지하층, 1층에서 높음
```

### 5. 환경 조건
```
온도:
  - 쾌적 범위: 18-24°C (겨울), 23-26°C (여름)
  - ASHRAE 55 기준

습도:
  - 쾌적 범위: 40-60%
  - 낮음 (<30%): 피부 건조, 정전기
  - 높음 (>60%): 곰팡이, 진드기 증식
```

### 6. 환기율
```
ACH (Air Changes per Hour):
  - 주거: 0.35-1.0 ACH
  - 사무실: 2-4 ACH
  - 교실: 3-4 ACH
  - 병원: 6-12 ACH (병실)

외기 공급량:
  - ASHRAE 62.1 기준
  - 사무실: 10 L/s/person
  - 교실: 8 L/s/person
```

## 🔧 HVAC 연동

### 자동 제어
```
공기질 기반 환기 제어:
  - CO₂ > 1,000 ppm → 환기량 증가
  - PM2.5 > 35 μg/m³ → 공기청정기 가동
  - TVOC > 500 μg/m³ → 강제 환기

에너지 최적화:
  - 재실 여부 감지
  - 외기 온도/습도 고려
  - 시간대별 스케줄링
```

### 필터 관리
```
교체 알림:
  - 사용 시간 기반
  - 압력 차이 측정
  - 효율 저하 감지

필터 종류:
  - HEPA: PM2.5, 세균, 바이러스
  - 활성탄: VOCs, 냄새
  - 정전 필터: 대형 먼지
```

## 📡 API 인터페이스

### 공간 등록
```http
POST /api/v1/spaces/register
Content-Type: application/json

{
  "space": {
    "spaceId": "OFFICE-SEL-001",
    "name": "서울 본사 3층",
    "type": "OFFICE",
    "floorArea_m2": 150,
    "ceilingHeight_m": 2.7,
    "occupancy": { "maxOccupancy": 20 }
  }
}

Response:
{
  "spaceId": "OFFICE-SEL-001",
  "dashboardUrl": "https://iaq.wia.org/spaces/OFFICE-SEL-001",
  "apiAccessToken": "..."
}
```

### 측정값 제출
```http
POST /api/v1/readings/submit

{
  "readings": [{
    "spaceId": "OFFICE-SEL-001",
    "timestamp": "2025-12-25T14:30:00Z",
    "particulateMatter": {
      "pm25_ugm3": 12.5,
      "pm10_ugm3": 28.3
    },
    "gasConcentrations": {
      "co2_ppm": 850
    },
    "environmental": {
      "temperature_c": 23.5,
      "humidity_percent": 55
    }
  }]
}
```

### 현재 상태 조회
```http
GET /api/v1/spaces/{spaceId}/status

Response:
{
  "reading": {
    "iaq": {
      "overall": 85,
      "category": "GOOD",
      "healthAdvisory": "공기질이 좋습니다"
    },
    "particulateMatter": { "pm25_ugm3": 12 },
    "gasConcentrations": { "co2_ppm": 850 }
  },
  "alerts": []
}
```

## 🏅 기준 준수

### 국제 표준
- **WHO 2021**: 세계보건기구 공기질 가이드라인
- **ASHRAE 62.1**: 실내 공기질 환기 기준
- **ISO 16000**: 실내 공기 측정 방법
- **EN 16798**: 건물 에너지 성능 및 IAQ

### 국내 기준
- **실내공기질 관리법**: 다중이용시설 기준
- **학교보건법**: 교실 공기질 기준
- **산업안전보건법**: 작업장 환경 기준

### 인증 프로그램
- **WELL Building Standard**: 건강 중심 건축 인증
- **LEED v4**: 지속가능 건축 인증 (IAQ 크레딧)
- **RESET Air**: 실시간 공기질 인증

## 🚀 빠른 시작

### 1. 설치
```bash
# 저장소 복제
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/indoor-air-quality

# 설치
./install.sh
```

### 2. CLI 사용
```bash
# 공간 등록
./cli/indoor-air-quality.sh register \
  --space-id OFFICE-SEL-001 \
  --name "서울 본사 3층" \
  --type OFFICE \
  --area 150 \
  --height 2.7 \
  --occupancy 20

# 현재 상태 조회
./cli/indoor-air-quality.sh status OFFICE-SEL-001

# 측정값 제출
./cli/indoor-air-quality.sh submit \
  --space-id OFFICE-SEL-001 \
  --temp 23.5 \
  --humidity 55 \
  --co2 850 \
  --pm25 12

# 보고서 생성
./cli/indoor-air-quality.sh report OFFICE-SEL-001 WEEKLY
```

### 3. TypeScript SDK
```typescript
import { IndoorAirQualityClient } from '@wia/ene-027';

const client = new IndoorAirQualityClient({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/ene-027/v1'
});

// 공간 등록
const space = await client.registerSpace({
  space: {
    spaceId: 'OFFICE-SEL-001',
    name: '서울 본사 3층',
    type: 'OFFICE',
    location: { lat: 37.5665, lon: 126.9780 },
    floorArea_m2: 150,
    ceilingHeight_m: 2.7,
    volume_m3: 405,
    occupancy: { maxOccupancy: 20 }
  },
  sensors: []
});

// 현재 상태 조회
const status = await client.getCurrentStatus('OFFICE-SEL-001');
console.log('IAQ:', status.reading.iaq?.overall);
console.log('PM2.5:', status.reading.particulateMatter?.pm25_ugm3);
console.log('CO₂:', status.reading.gasConcentrations?.co2_ppm);

// 실시간 모니터링
const ws = client.subscribeToReadings('OFFICE-SEL-001', (reading) => {
  console.log('New reading:', reading);
});
```

## 📊 대시보드 예시

### 주요 지표
```
┌─────────────────────────────────────────┐
│  실내 공기질 지수 (IAQ): 85  [좋음]     │
├─────────────────────────────────────────┤
│  온도:      23.5°C  ✓                   │
│  습도:      55%     ✓                   │
│  CO₂:       850 ppm ✓                   │
│  PM2.5:     12 μg/m³ ✓                  │
│  TVOC:      220 μg/m³ ✓                 │
├─────────────────────────────────────────┤
│  환기율:    2.5 ACH                     │
│  재실자:    15/20명                     │
│  HVAC:      자동 모드                   │
└─────────────────────────────────────────┘
```

### 경보 예시
```
⚠️  경보: CO₂ 수준 높음
   현재값: 1,250 ppm
   기준값: 1,000 ppm
   권장조치: 환기 강화 또는 창문 개방
   시각: 2025-12-25 14:30

ℹ️  알림: 필터 교체 권장
   마지막 교체: 85일 전
   권장 주기: 90일
   필터 수명: 94% 소진
```

## 💡 사용 사례

### 사무실
- 재실자 수 기반 환기 제어
- CO₂ 모니터링으로 회의실 관리
- 에너지 절감 및 생산성 향상

### 학교
- 교실 공기질 실시간 모니터링
- 학부모 대시보드 제공
- 학습 환경 최적화

### 병원
- 병실 감염 관리
- 수술실 청정도 유지
- 환자 안전 보장

### 주거
- 새집증후군 예방
- 라돈 모니터링
- 가족 건강 보호

### 헬스장
- 고강도 운동 시 환기 최적화
- 미세먼지 관리
- 재실자 안전

## 📈 성과 지표

### 공기질 개선
- IAQ 점수: 목표 > 80
- PM2.5: < 15 μg/m³ 유지율 > 95%
- CO₂: < 1,000 ppm 유지율 > 90%
- VOCs: < 500 μg/m³ 유지율 > 95%

### 에너지 효율
- HVAC 에너지: 15-25% 절감
- 운영 비용: 연간 20-30% 절감

### 건강 & 웰빙
- 호흡기 질환: 30% 감소
- 결근율: 20% 감소
- 생산성: 8-11% 향상
- 만족도: 40% 증가

## 🤝 기여

이 표준은 실내 공기질 관리 커뮤니티의 기여를 환영합니다:

- 기술 피드백 및 개선 제안
- 사례 연구 및 구현 사례
- 추가 언어 번역
- 교육 자료 및 튜토리얼
- 연구 결과 및 데이터

## 📜 라이선스

© 2025 SmileStory Inc. / WIA - World Certification Industry Association

이 표준은 MIT 라이선스 하에 배포됩니다.

## 🔗 관련 표준

- **WIA-ENE-017**: Air Quality Monitoring (대기 공기질)
- **WIA-ENE-022**: Waste Management (폐기물 관리)
- **WIA-INTENT**: 의도 표현 표준
- **WIA-OMNI-API**: 통합 API 표준

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

WIA-ENE-027 표준은 홍익인간 (弘益人間)의 철학을 통해 모든 사람이 건강한 실내 공기를 누릴 수 있도록 합니다. 우리가 숨 쉬는 공기는 우리의 건강, 행복, 생산성의 기초입니다. 과학적 모니터링과 스마트 관리를 통해 더 나은 실내 환경을 만들어갑니다.

**건강한 공기, 건강한 삶**

---

*문서 버전: 1.0.0*
*최종 업데이트: 2025-12-25*
*상태: Active*
