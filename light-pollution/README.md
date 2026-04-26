# WIA-ENE-029: 빛 공해 관리 표준 💡

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--ENE--029-red.svg)](https://wia.org/standards/ene-029)

## 개요

WIA-ENE-029 빛 공해 관리 표준은 인공 조명으로 인한 환경 오염을 체계적으로 관리하고, 어두운 밤하늘을 보호하며, 인간과 생태계의 건강을 증진하기 위한 국제 표준입니다.

### 주요 기능

- 🌌 **어두운 하늘 보호**: Bortle 척도 기반 하늘 밝기 측정 및 관리
- 💡 **조명 규정 준수**: Dark Sky Zone별 조명 기준 및 검증
- 🦇 **생태계 보호**: 야생동물 영향 평가 및 완화 조치
- 😴 **건강 증진**: 일주기 리듬 보호 및 수면 질 개선
- ⚡ **에너지 절감**: 과조명 감소 및 스마트 제어 시스템
- 🌐 **개방형 표준**: 전 세계 빛 공해 관리 시스템 상호운용성 보장

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/light-pollution

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```typescript
import { LightPollutionSDK } from '@wia/ene-029';

const sdk = new LightPollutionSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/ene-029/v1'
});

// 하늘 밝기 측정 제출
const measurement = await sdk.submitMeasurement({
  location: {
    address: '서울시 강남구 테헤란로 123',
    coordinates: { latitude: 37.5665, longitude: 126.9780 },
    zone: DarkSkyZone.E4_SUBURBAN
  },
  skyBrightness: {
    zenith: 18.5,                          // mag/arcsec²
    bortle: BortleScale.BRIGHT_SUBURBAN_SKY,
    sqm: 18.5,
    visibleStars: 250
  },
  illuminance: {
    horizontal: 5.2,                       // lux
    vertical: 3.1,
    unit: 'lux'
  },
  metadata: {
    deviceId: 'SQM-001',
    deviceType: 'Sky Quality Meter',
    operator: 'Dark Sky Korea',
    weather: {
      cloudCover: 0,
      visibility: 20,
      moonPhase: 0.15,
      precipitation: false,
      temperature: 18,
      humidity: 60
    },
    calibrationDate: '2025-12-01T00:00:00Z',
    dataQuality: 95,
    verified: true
  }
});

console.log('측정 ID:', measurement.data?.measurementId);
```

### 3. CLI 도구 사용

```bash
# 측정 데이터 제출
./cli/light-pollution.sh submit-measurement \
  --lat 37.5665 \
  --lon 126.9780 \
  --sky-brightness 18.5 \
  --illuminance 5.2

# 하늘 밝기 조회
./cli/light-pollution.sh get-sky-brightness \
  --lat 37.5665 \
  --lon 126.9780

# 조명 기구 등록
./cli/light-pollution.sh register-fixture \
  --type street_light \
  --wattage 50 \
  --color-temp 2700 \
  --zone E4

# 구역 제한 조회
./cli/light-pollution.sh zone-limits --zone E2

# 성과 보고서 생성
./cli/light-pollution.sh report \
  --region "서울시" \
  --start-date 2025-01-01 \
  --end-date 2025-12-31
```

### 4. 상세 사양 확인

- **스펙 문서**: [`spec/WIA-ENE-029-v1.0.md`](spec/WIA-ENE-029-v1.0.md)

## 저장소 구조

```
light-pollution/
├── README.md              # 본 문서
├── install.sh             # 설치 스크립트
├── spec/
│   └── WIA-ENE-029-v1.0.md  # 상세 스펙
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts    # 타입 정의
│       │   └── index.ts    # SDK 구현
│       └── package.json    # npm 패키지 설정
└── cli/
    └── light-pollution.sh  # CLI 도구
```

## 기술 범위

### 빛 공해 유형

| 유형 | 설명 | 측정 단위 |
|------|------|----------|
| **하늘 빛공해 (Sky Glow)** | 대기 산란으로 인한 야간 하늘 밝아짐 | mag/arcsec² |
| **빛 침입 (Light Trespass)** | 원하지 않는 곳으로의 빛 유입 | lux |
| **눈부심 (Glare)** | 과도한 밝기나 대비로 인한 시각적 불편 | cd/m² |
| **과잉 조명 (Over-illumination)** | 필요 이상의 과도한 밝기 | lux |

### 어두운 하늘 구역 (Dark Sky Zones)

| 구역 | 특성 | 최대 수직 조도 | 색온도 | 소등 시간 |
|------|------|--------------|--------|----------|
| **E1** | 천연 어두움 (국립공원, 천문대) | 0.1 lux | ≤ 2200K | 22:00-06:00 |
| **E2** | 어두움 (농촌 지역) | 1 lux | ≤ 2700K | 23:00-06:00 |
| **E3** | 시골 | 2 lux | ≤ 3000K | - |
| **E4** | 교외 | 5 lux | ≤ 4000K | - |
| **E5** | 도시 | 10 lux | ≤ 5000K | - |

### Bortle 어두운 하늘 척도

| 척도 | 하늘 밝기 (mag/arcsec²) | 육안 별 수 | 상태 |
|------|----------------------|-----------|------|
| 1 | > 21.7 | 5,000+ | 우수한 어두운 하늘 |
| 2 | 21.5-21.7 | 4,000 | 일반적인 어두운 하늘 |
| 3 | 21.3-21.5 | 2,500 | 시골 하늘 |
| 4 | 20.5-21.3 | 1,000 | 시골-교외 경계 |
| 5 | 19.5-20.5 | 500 | 교외 하늘 |
| 6 | 18.5-19.5 | 250 | 밝은 교외 하늘 |
| 7 | 18.0-18.5 | 100 | 교외-도심 경계 |
| 8 | 17.0-18.0 | 50 | 도시 하늘 |
| 9 | < 17.0 | 10 | 도심 하늘 |

### 조명 차폐 기준

| 차폐 유형 | 90° 상향광 | 80° 상향광 | 적용 |
|---------|----------|----------|------|
| 완전 차폐 (Fully Shielded) | 0% | 0% | E1, E2 필수 |
| 완전 차단 (Fully Cutoff) | 0% | ≤ 2.5% | E2, E3 권장 |
| 차단 (Cutoff) | 0% | ≤ 5% | E3, E4 |
| 반차단 (Semi-Cutoff) | ≤ 5% | ≤ 20% | E4, E5 |

## 데이터 포맷 표준

### 빛 공해 측정 이벤트

```json
{
  "measurementId": "LP-2025-001234",
  "timestamp": "2025-12-25T22:30:00Z",
  "location": {
    "address": "서울시 강남구 테헤란로 123",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude": 50
    },
    "zone": "E4"
  },
  "illuminance": {
    "horizontal": 5.2,
    "vertical": 3.1,
    "unit": "lux"
  },
  "skyBrightness": {
    "zenith": 18.5,
    "bortle": 6,
    "sqm": 18.5,
    "visibleStars": 250
  },
  "colorTemperature": {
    "value": 3500,
    "blueLightPercentage": 12,
    "melanopicLux": 3.8
  },
  "pollution": {
    "skyGlow": {
      "severity": "moderate",
      "affectedArea": 25
    }
  },
  "metadata": {
    "deviceId": "SQM-001",
    "deviceType": "Sky Quality Meter",
    "operator": "Dark Sky Korea",
    "weather": {
      "cloudCover": 0,
      "visibility": 20,
      "moonPhase": 0.15,
      "precipitation": false,
      "temperature": 18,
      "humidity": 60
    },
    "dataQuality": 95,
    "verified": true
  }
}
```

### 조명 기구 정보

```json
{
  "fixtureId": "FIX-STR-001234",
  "name": "테헤란로 가로등 #42",
  "type": "street_light",
  "location": {
    "address": "서울시 강남구 테헤란로 123",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "zone": "E4"
  },
  "specifications": {
    "manufacturer": "Samsung LED",
    "model": "SL-3000-WW",
    "wattage": 50,
    "lumens": 5000,
    "efficacy": 100,
    "colorTemperature": 2700,
    "cri": 80,
    "beamAngle": 120,
    "cutoffAngle": 90,
    "shielding": "fully_cutoff",
    "upwardLightRatio": 2.3,
    "ratedLife": 50000,
    "l70": 50000
  },
  "installation": {
    "installDate": "2025-06-15",
    "height": 6,
    "tilt": 0,
    "orientation": 180
  },
  "controls": {
    "dimming": true,
    "dimmingSchedule": [
      {
        "timeStart": "22:00",
        "timeEnd": "06:00",
        "level": 50
      }
    ],
    "motionSensor": false,
    "lightSensor": true,
    "centralControl": true
  },
  "compliance": {
    "compliant": true,
    "zone": "E4",
    "violations": [],
    "certifications": ["IDA", "DarkSky"],
    "lastInspection": "2025-12-01"
  }
}
```

## API 엔드포인트

WIA-ENE-029 표준은 RESTful API 엔드포인트를 정의합니다:

### 측정 관리

- `POST /api/v1/measurements` - 측정 데이터 제출
- `GET /api/v1/measurements/{id}` - 측정 조회
- `GET /api/v1/measurements` - 측정 목록 조회
- `GET /api/v1/measurements/sky-brightness` - 하늘 밝기 조회
- `GET /api/v1/measurements/illuminance` - 조도 조회

### 조명 기구 관리

- `POST /api/v1/fixtures` - 조명 기구 등록
- `GET /api/v1/fixtures/{id}` - 기구 조회
- `PUT /api/v1/fixtures/{id}` - 기구 업데이트
- `DELETE /api/v1/fixtures/{id}` - 기구 삭제
- `GET /api/v1/fixtures/{id}/compliance` - 규정 준수 확인

### 구역 관리

- `GET /api/v1/zones/{zone}/limits` - 구역 제한 조회
- `GET /api/v1/zones/classify` - 위치 기반 구역 분류

### 경보 및 모니터링

- `POST /api/v1/alerts` - 경보 생성
- `GET /api/v1/alerts` - 경보 목록
- `POST /api/v1/alerts/{id}/acknowledge` - 경보 확인
- `POST /api/v1/alerts/{id}/resolve` - 경보 해결

### 분석 및 보고

- `GET /api/v1/analytics/statistics` - 통계 조회
- `POST /api/v1/analytics/report` - 보고서 생성
- `GET /api/v1/analytics/compliance` - 규정 준수 대시보드

## 구현 가이드

### 1단계: 현황 조사 (1-2개월)

1. 기존 조명 인벤토리 작성
2. 하늘 밝기 기준선 측정
3. 빛 공해 핫스팟 식별
4. 지역 구역 분류

### 2단계: 계획 수립 (2-3개월)

1. 조명 마스터 플랜 개발
2. 우선순위 지역 선정
3. 예산 및 일정 수립
4. 이해관계자 참여

### 3단계: 개선 실행 (6-12개월)

1. 과조명 감소 및 차폐 추가
2. 고효율 저온 LED 교체
3. 스마트 제어 시스템 설치
4. 모니터링 네트워크 구축

### 4단계: 검증 및 인증 (3-6개월)

1. 성과 측정 및 검증
2. 규정 준수 확인
3. WIA 인증 신청
4. IDA/DarkSky 인증 추진

### 5단계: 지속적 개선 (지속)

1. 정기 모니터링 및 보고
2. 커뮤니티 교육 프로그램
3. 신기술 도입
4. 정책 업데이트

## 주요 성과 지표 (KPI)

### 환경 성과

- **평균 하늘 밝기**: Bortle ≤ 4
- **어두운 하늘 면적**: 30% 증가 (Bortle ≤ 3)
- **규정 준수율**: 95% 이상
- **에너지 절감**: 40% 감소
- **CO2 감축**: 10,000톤/년

### 생태 성과

- **야생동물 서식지 회복**: 빛 공해 감소 지역 수
- **철새 충돌 감소**: 연간 충돌 사고 건수
- **곤충 다양성**: 야행성 곤충 종 수
- **생물 다양성**: 야행성 종 관측 빈도

### 사회 성과

- **별 관측 가능성**: 은하수 관측 가능 지역 비율
- **수면 질 개선**: 주민 설문 조사
- **민원 감소**: 빛 공해 관련 민원 건수
- **인식 제고**: 빛 공해 인지도

## 통합 예제

### Python

```python
from wia_ene029 import LightPollutionClient

client = LightPollutionClient(
    api_key='your-api-key',
    endpoint='https://api.wia.org/ene-029/v1'
)

# 하늘 밝기 측정 제출
measurement = client.submit_measurement(
    latitude=37.5665,
    longitude=126.9780,
    sky_brightness=18.5,
    illuminance=5.2
)

print(f'측정 ID: {measurement.measurement_id}')
```

### REST API (cURL)

```bash
curl -X POST https://api.wia.org/ene-029/v1/measurements \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "location": {
      "coordinates": {
        "latitude": 37.5665,
        "longitude": 126.9780
      },
      "zone": "E4"
    },
    "skyBrightness": {
      "zenith": 18.5,
      "bortle": 6,
      "sqm": 18.5,
      "visibleStars": 250
    },
    "metadata": {
      "deviceId": "SQM-001",
      "deviceType": "Sky Quality Meter",
      "dataQuality": 95,
      "verified": true
    }
  }'
```

## 측정 장비

### 하늘 밝기 측정

- **SQM (Sky Quality Meter)**: Unihedron SQM-L, SQM-LU-DL
- **측정 범위**: 15-22 mag/arcsec²
- **정확도**: ±0.1 mag/arcsec²

### 조도 측정

- **조도계 (Lux Meter)**: Konica Minolta CL-200A, GOSSEN Mavolux 5032C
- **정확도**: Class A (±5%)
- **측정 범위**: 0.01 - 200,000 lux

### 스펙트럼 측정

- **분광계 (Spectrometer)**: Ocean Optics USB4000, Stellarnet BLUE-Wave
- **파장 범위**: 380-780 nm
- **해상도**: 1-10 nm

## 글로벌 사례

### 서울시, 대한민국

- **배경**: 도심 빛 공해 심각, Bortle 8-9
- **조치**: 가로등 LED 교체 (3000K), 심야 조광 50%
- **결과**: 에너지 35% 절감, 하늘 밝기 0.5 mag/arcsec² 개선

### 영국 엑스무어 국립공원

- **배경**: 유럽 최초 Dark Sky Reserve
- **조치**: E1 구역 지정, 완전 차폐 조명, 소등 시간
- **결과**: Bortle 1-2 유지, 관광 수익 증가

### 플래그스태프, 미국

- **배경**: 세계 최초 IDA Dark Sky City (2001)
- **조치**: 조명 조례, 호박색 LED (2200K), 천문대 보호
- **결과**: 천문 관측 품질 유지, 50년 이상 지속

## 기여하기

WIA-ENE-029 표준 개선에 기여해 주세요:

1. **기술 피드백**: GitHub 이슈로 제안사항 제출
2. **사례 연구**: 구현 경험 공유
3. **번역**: 다른 언어로 문서 번역
4. **데이터 제공**: 측정 데이터 공유

자세한 내용은 [CONTRIBUTING.md](../CONTRIBUTING.md)를 참조하세요.

## 커뮤니티 및 지원

- **웹사이트**: [wia.org/standards/ene-029](https://wia.org/standards/ene-029)
- **문서**: [docs.wia.org/ene-029](https://docs.wia.org/ene-029)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **포럼**: [community.wia.org/light-pollution](https://community.wia.org/light-pollution)
- **이메일**: standards@wia-official.org

## 관련 조직

- **IDA (International Dark-Sky Association)**: https://darksky.org
- **DarkSky International**: https://darksky.org
- **Globe at Night**: https://globeatnight.org
- **Loss of the Night**: https://www.lossofthenight.org

## 라이선스

이 표준은 [MIT License](https://opensource.org/licenses/MIT) 하에 배포됩니다.

다음과 같은 자유가 있습니다:
- **공유**: 자료를 복사하고 재배포
- **수정**: 리믹스, 변환 및 빌드
- **상업적 사용**: 상업적 목적으로 사용

단, 다음 조건을 준수해야 합니다:
- **저작자 표시**: WIA에 적절한 크레딧 제공

## 인용

```bibtex
@standard{wia-ene-029,
  title = {WIA-ENE-029: Light Pollution Standard},
  author = {{World Certification Industry Association}},
  year = {2025},
  version = {1.0},
  url = {https://github.com/WIA-Official/wia-standards/light-pollution}
}
```

## 감사의 말

이 표준은 다음 분들의 기여로 개발되었습니다:
- 국제 어두운 하늘 협회 (IDA)
- 천문학자 및 천문대 운영자
- 조명 설계 전문가
- 생태학자 및 보전 과학자
- 공중 보건 연구자
- 도시 계획 및 정책 입안자
- 시민 과학자 및 별 관측자

## 관련 표준

- **WIA-ENE-001**: 기후 변화 관리
- **WIA-ENE-007**: 스마트 그리드
- **WIA-SOCIAL**: 시민 참여 및 교육
- **WIA-BLOCKCHAIN**: 조명 기구 이력 추적
- **WIA-ENE-BIO**: 생물 다양성 보호

## 변경 이력

### Version 1.0.0 (2025-12-25)

- 초판 발행
- 완전한 문서 패키지
- TypeScript SDK
- CLI 도구
- API 스펙

---

## 홍익인간 (弘益人間) (홍익인간) · 널리 인간을 이롭게 하라

WIA-ENE-029 표준은 홍익인간 (弘益人間)(홍익인간)의 정신을 구현합니다. 빛은 인류 문명의 상징이지만, 과도한 인공 조명은 밤하늘의 어둠을 잃게 하고 생태계를 교란하며 인간 건강을 해칩니다.

지속가능한 조명 관리를 통해 어두운 밤하늘을 보호하고, 에너지를 절약하며, 인류와 자연이 조화롭게 공존하는 미래를 만들어갑니다.

개방형 표준, 투명한 프로토콜, 협력적 개발을 통해 빛 공해 관리가 인류 전체와 지구 생태계의 공동선에 기여하도록 보장합니다.

**함께, 우리는 별이 빛나는 밤하늘을 되찾습니다.**

---

© 2025 SmileStory Inc. / WIA
**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**
