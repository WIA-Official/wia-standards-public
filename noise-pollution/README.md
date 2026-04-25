# WIA-ENE-028: 소음 공해 표준 🔊

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

환경 소음을 체계적으로 측정, 모니터링, 관리하기 위한 종합 표준입니다.

---

## 개요

WIA-ENE-028 소음 공해 표준은 도시 환경의 소음을 실시간으로 모니터링하고, 건강 영향을 평가하며, 효과적인 저감 대책을 수립하기 위한 국제 표준입니다.

### 주요 기능

- 🎯 **정밀 측정**: IEC 61672-1 표준 준수, Class 1 정확도
- 📊 **실시간 모니터링**: 1초 단위 연속 측정 및 데이터 전송
- 🏥 **건강 영향 평가**: 노출 인구 및 건강 피해 정량화
- 🗺️ **소음 지도**: 실시간 소음 히트맵 생성
- ⚠️ **경보 시스템**: 기준 초과 시 자동 알림
- 🌙 **야간 보호**: 시간대별 차등 규제 적용
- 🏛️ **정온시설 보호**: 학교, 병원 등 특별 관리
- 🔒 **보안**: TLS 1.3, OAuth 2.0 인증

---

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/noise-pollution

# 설치 스크립트 실행
chmod +x install.sh
./install.sh
```

### 2. API 키 설정

```bash
export WIA_API_KEY="your-api-key-here"
```

### 3. CLI 사용

```bash
# 실시간 소음 데이터 조회
./cli/noise-pollution.sh realtime Seoul

# 시간대별 분석
./cli/noise-pollution.sh time-period "Seoul Station" 2025-12-25

# 건강 영향 평가
./cli/noise-pollution.sh health Seoul 2025-01-01 2025-12-31
```

### 4. TypeScript SDK 사용

```typescript
import { NoisePollutionSDK } from '@wia/ene-028';

const sdk = new NoisePollutionSDK({
  apiKey: process.env.WIA_API_KEY!,
  endpoint: 'https://api.wia.org/ene-028/v1',
});

// 실시간 데이터 조회
const realtimeData = await sdk.getRealtimeData('Seoul');

// 측정소 정보 조회
const station = await sdk.getStation('ST001');

// 건강 영향 평가
const healthImpact = await sdk.getHealthImpact('Seoul', {
  startDate: '2025-01-01T00:00:00Z',
  endDate: '2025-12-31T23:59:59Z',
});
```

---

## 디렉토리 구조

```
noise-pollution/
├── README.md                       # 이 파일
├── install.sh                      # 설치 스크립트
├── spec/
│   └── WIA-ENE-028-v1.0.md        # 기술 명세서 (한글)
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts           # TypeScript 타입 정의
│       │   └── index.ts           # SDK 구현
│       └── package.json
└── cli/
    └── noise-pollution.sh         # CLI 도구
```

---

## 측정 항목

### 소음 레벨

| 지표 | 설명 | 단위 |
|------|------|------|
| Leq | 등가소음도 (평균) | dBA |
| Lmax | 최대소음도 | dBA |
| Lmin | 최소소음도 | dBA |
| L10 | 10% 초과 레벨 | dBA |
| L50 | 50% 초과 레벨 (중앙값) | dBA |
| L90 | 90% 초과 레벨 (배경소음) | dBA |
| Lden | 주야간 등가소음도 | dBA |
| Lnight | 야간 소음도 | dBA |

### 주파수 분석

- **옥타브 밴드**: 31.5Hz ~ 16kHz (10개 대역)
- **1/3 옥타브 밴드**: 25Hz ~ 20kHz (30개 대역)
- **저주파 분석**: 20Hz 이하

---

## 환경 기준

### 대한민국 환경부 기준

| 지역 구분 | 주간 (07:00-18:00) | 저녁 (18:00-22:00) | 야간 (22:00-07:00) |
|----------|-------------------|-------------------|-------------------|
| 일반주거지역 | 55 dBA | 50 dBA | 45 dBA |
| 도로변지역 | 70 dBA | 65 dBA | 60 dBA |
| 상업·공업지역 | 65 dBA | 60 dBA | 55 dBA |
| 녹지지역 | 50 dBA | 45 dBA | 40 dBA |
| 학교·병원 | 50 dBA | 45 dBA | 40 dBA |

### WHO 야간 소음 가이드라인

| 건강 영향 | Lnight | Lmax |
|----------|--------|------|
| 수면 방해 없음 | < 30 dBA | < 45 dBA |
| 경미한 수면 방해 | 30-40 dBA | 45-60 dBA |
| 중간 수면 방해 | 40-55 dBA | 60-70 dBA |
| 심각한 건강 영향 | > 55 dBA | > 70 dBA |

---

## 건강 영향

### 청력 손상 기준

| 노출 레벨 | 노출 시간 | 위험도 |
|----------|----------|--------|
| 85 dBA | 8시간/일 | 위험 시작 |
| 90 dBA | 4시간/일 | 중간 위험 |
| 95 dBA | 2시간/일 | 높은 위험 |
| 100 dBA | 1시간/일 | 매우 높은 위험 |
| 115 dBA | 15분 | 즉각적 손상 |
| 120 dBA | - | 통증 역치 |

### 비청각적 영향

- **수면 방해**: 40 dBA 이상 (야간)
- **심혈관 질환**: 55 dBA 이상 (Lden)
- **인지 기능 저하**: 50 dBA 이상 (학교)
- **스트레스**: 60 dBA 이상
- **불쾌감**: 55 dBA 이상 (Lden)

---

## CLI 명령어

### 측정 데이터 관리

```bash
# 측정 데이터 조회
./cli/noise-pollution.sh measurement get MEASUREMENT_ID

# 측정 데이터 목록
./cli/noise-pollution.sh measurement list --station ST001 --start-date 2025-01-01 --end-date 2025-12-31

# 통계 요약
./cli/noise-pollution.sh measurement stats ST001 2025-01-01 2025-12-31
```

### 측정소 관리

```bash
# 측정소 정보 조회
./cli/noise-pollution.sh station get ST001

# 측정소 목록
./cli/noise-pollution.sh station list --type permanent --status active

# 측정소 상태
./cli/noise-pollution.sh station status ST001
```

### 실시간 모니터링

```bash
# 실시간 소음 데이터
./cli/noise-pollution.sh realtime Seoul

# 소음 히트맵
./cli/noise-pollution.sh heatmap Seoul 2025-12-25 night
```

### 소음원 관리

```bash
# 소음원 조회
./cli/noise-pollution.sh source get SRC001

# 소음원 목록
./cli/noise-pollution.sh source list --category traffic_road --region Seoul
```

### 분석 및 보고

```bash
# 시간대별 분석
./cli/noise-pollution.sh time-period "Seoul Station" 2025-12-25

# 건강 영향 평가
./cli/noise-pollution.sh health Seoul 2025-01-01 2025-12-31

# KPI 대시보드
./cli/noise-pollution.sh kpi Seoul 2025-12-25

# 규제 준수 보고서
./cli/noise-pollution.sh compliance Seoul 2025-01-01 2025-12-31
```

### 경보 및 민원

```bash
# 활성 경보 목록
./cli/noise-pollution.sh alert list high

# 경보 승인
./cli/noise-pollution.sh alert ack ALERT_ID

# 민원 접수
./cli/noise-pollution.sh complaint submit "123 Main St" "Construction" "Loud noise at night"

# 민원 조회
./cli/noise-pollution.sh complaint list --status in_review
```

### 정온구역 및 저감

```bash
# 정온구역 목록
./cli/noise-pollution.sh quiet-zone list Seoul

# 정온구역 확인
./cli/noise-pollution.sh quiet-zone check 37.5665 126.9780

# 저감 대책
./cli/noise-pollution.sh mitigation Seoul
```

---

## TypeScript SDK API

### 초기화

```typescript
import { NoisePollutionSDK } from '@wia/ene-028';

const sdk = new NoisePollutionSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/ene-028/v1',
  timeout: 30000,
  debug: false,
});
```

### 측정 데이터

```typescript
// 측정 데이터 조회
const measurement = await sdk.getMeasurement('M001');

// 측정 데이터 목록
const measurements = await sdk.listMeasurements({
  stationId: 'ST001',
  dateRange: {
    startDate: '2025-01-01T00:00:00Z',
    endDate: '2025-12-31T23:59:59Z',
  },
  pagination: {
    page: 1,
    limit: 100,
  },
});

// 통계 요약
const stats = await sdk.getStationStatistics('ST001', {
  startDate: '2025-01-01T00:00:00Z',
  endDate: '2025-12-31T23:59:59Z',
});
```

### 실시간 모니터링

```typescript
// 실시간 데이터
const realtimeData = await sdk.getRealtimeData('Seoul');

// WebSocket 구독
const wsUrl = sdk.getRealtimeWebSocketURL('ST001');
const ws = new WebSocket(wsUrl);
ws.onmessage = (event) => {
  const update = JSON.parse(event.data);
  console.log(`Station ${update.stationId}: ${update.leq_dBA} dBA`);
};
```

### 건강 영향 평가

```typescript
// 건강 영향 평가
const healthImpact = await sdk.getHealthImpact('Seoul', {
  startDate: '2025-01-01T00:00:00Z',
  endDate: '2025-12-31T23:59:59Z',
});

console.log(`노출 인구 (65dB+): ${healthImpact.data.exposure.exposedPopulation.above65dB}`);
console.log(`수면 방해: ${healthImpact.data.healthEffects.sleepDisturbance.percentage}%`);
console.log(`의료 비용: ${healthImpact.data.economicImpact.healthcareCost}원`);

// 노출 인구 계산
const exposed = await sdk.calculateExposedPopulation('Seoul', 65);
```

### 소음 지도

```typescript
// 소음 히트맵
const heatmap = await sdk.getNoiseHeatmap({
  region: 'Seoul',
  date: '2025-12-25',
  timePeriod: 'night',
  resolution: 100, // 100m 그리드
});

heatmap.data.forEach(point => {
  console.log(`${point.coordinates.latitude}, ${point.coordinates.longitude}: ${point.leq_dBA} dBA (${point.category})`);
});
```

### 경보 및 민원

```typescript
// 활성 경보
const alerts = await sdk.getActiveAlerts('high');

// 경보 승인
await sdk.acknowledgeAlert('ALERT_001');

// 민원 접수
const complaint = await sdk.submitComplaint({
  location: {
    address: '123 Main St',
    coordinates: { latitude: 37.5665, longitude: 126.9780 },
  },
  noiseSource: 'Construction',
  description: 'Loud noise at night',
});
```

### 유틸리티 함수

```typescript
import {
  calculateLden,
  calculateHighlyAnnoyed,
  calculateSleepDisturbance,
  addNoiseLevels,
  subtractNoiseLevels,
  getNoiseCategory,
} from '@wia/ene-028';

// Lden 계산
const lden = calculateLden(65, 60, 55); // Ld, Le, Ln
console.log(`Lden: ${lden.toFixed(1)} dBA`);

// 불쾌감 비율
const annoyed = calculateHighlyAnnoyed(65);
console.log(`심각한 불쾌감: ${annoyed.toFixed(1)}%`);

// 수면 방해 비율
const sleep = calculateSleepDisturbance(50);
console.log(`수면 방해: ${sleep.toFixed(1)}%`);

// 소음 레벨 합산
const total = addNoiseLevels(70, 68, 65);
console.log(`합산 소음도: ${total.toFixed(1)} dBA`);

// 배경 소음 제거
const source = subtractNoiseLevels(72, 65);
console.log(`소음원 레벨: ${source.toFixed(1)} dBA`);

// 카테고리 분류
const category = getNoiseCategory(68);
console.log(`카테고리: ${category}`); // 'high'
```

---

## 사용 사례

### 지방자치단체

- **환경 소음 관리**: 도시 전역 실시간 모니터링
- **정온구역 보호**: 학교, 병원 주변 특별 관리
- **민원 처리**: 소음 민원 접수 및 대응
- **정책 수립**: 데이터 기반 소음 저감 정책

### 건설 현장

- **소음 규제 준수**: 실시간 소음도 확인
- **야간 공사 관리**: 허용 기준 자동 모니터링
- **민원 예방**: 사전 경보 시스템
- **보고서 생성**: 자동 규제 준수 보고서

### 공항 및 항공사

- **항공기 소음 관리**: 이착륙 소음 측정
- **소음 영향권 평가**: 주변 지역 영향 분석
- **운항 시간 관리**: 야간 운항 제한 관리
- **보상 및 대책**: 피해 지역 식별 및 대응

### 연구 기관

- **건강 영향 연구**: 장기 노출 데이터 분석
- **소음 지도 작성**: GIS 기반 소음 분포
- **저감 효과 평가**: 방음벽 등 대책 효과 검증
- **정책 평가**: 규제 기준 효과성 분석

---

## 기술 사양

### 측정 장비 요구사항

- **정확도**: IEC 61672-1 Class 1 이상
- **주파수 범위**: 20 Hz ~ 20 kHz
- **동적 범위**: 30 dBA ~ 130 dBA
- **가중 필터**: A, C, Z
- **시간 가중**: Fast (125ms), Slow (1s)
- **교정 주기**: 6개월 (Class 1)

### 데이터 전송

- **프로토콜**: MQTT v5.0, WebSocket, HTTP/2
- **주기**: 1초 ~ 1분 (설정 가능)
- **QoS**: MQTT QoS 1 또는 2
- **압축**: gzip, brotli
- **보안**: TLS 1.3

### API 사양

- **아키텍처**: RESTful API
- **인증**: OAuth 2.0, JWT
- **데이터 포맷**: JSON (UTF-8)
- **페이징**: 커서 기반
- **속도 제한**: 1000 req/min
- **응답 시간**: p95 < 200ms

---

## 성과 지표 (KPI)

### 환경 KPI

- **기준 초과율 감소**: 50% 목표
- **평균 소음도 감소**: 5 dB 목표
- **야간 소음 개선**: 3 dB 목표
- **조용한 구역 확대**: 20% 증가

### 건강 KPI

- **고소음 노출 인구**: 30% 감소
- **수면 방해**: 20% 감소
- **불쾌감**: 25% 감소
- **건강 비용**: 10% 절감

### 운영 KPI

- **측정소 가동률**: 95% 이상
- **데이터 품질**: 98% 이상
- **민원 처리율**: 100% (7일 이내)
- **규제 준수율**: 95% 이상

---

## 인증

### 인증 등급

- **Bronze**: 기본 요구사항 충족
- **Silver**: Class 1 장비, 실시간 전송
- **Gold**: 주파수 분석, 통합 시스템
- **Platinum**: Class 0/1 장비, AI 분석

### 인증 절차

1. 신청 및 서류 제출
2. 현장 실사 (2일)
3. 데이터 검증 (30일)
4. 전문가 심사 (2주)
5. 인증서 발급
6. 정기 사후 관리 (6개월)

---

## 라이선스

© 2025 WIA (World Certification Industry Association)

본 표준은 Creative Commons Attribution 4.0 International License 하에 배포됩니다.

---

## 연락처

**WIA 표준 사무국**

- 웹사이트: https://wiastandards.com
- 이메일: standards@wia-official.org
- GitHub: https://github.com/WIA-Official/wia-standards

---

## 기여

표준 개선에 참여하실 수 있습니다:

- **표준 개선 제안**: GitHub Issues로 제안
- **코드 예제**: 참조 구현 제출
- **번역**: 다국어 문서 번역
- **사례 연구**: 실제 적용 사례 공유
- **버그 리포트**: 명세서 오류 보고

---

## 인용

학술 논문에서 본 표준을 인용하실 경우:

```
World Certification Industry Association (WIA). (2025).
WIA-ENE-028: Noise Pollution Standard v1.0.
Seoul, South Korea: SmileStory Inc.
https://wiastandards.com/noise-pollution
```

---

## 철학

**弘익人間 (홍익인간) - 널리 인간을 이롭게 하라**

조용한 환경은 인간의 기본적인 권리입니다. WIA-ENE-028 표준은 소음으로부터 모든 사람을 보호하고, 건강하고 평화로운 환경을 만드는 데 기여합니다.

모든 숨결이 우리를 환경과 연결합니다. 모든 소리도 마찬가지입니다. WIA-ENE-028은 그 연결이 조화롭고 건강하도록 보장합니다.

**Together, we create a quieter world for all humanity.**

---

**최신 업데이트와 소식은 [@WIAStandards](https://twitter.com/WIAStandards)를 팔로우하세요.**
