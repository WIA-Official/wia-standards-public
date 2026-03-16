# WIA-ENE-040: 심해 탐사 표준 v1.0 🐙

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 문서 정보

- **표준 번호**: WIA-ENE-040
- **표준 명칭**: Deep Sea Exploration Standard (심해 탐사 표준)
- **버전**: 1.0.0
- **발행일**: 2025-12-25
- **상태**: 정식 표준 (Official Standard)
- **카테고리**: 에너지 및 환경 (Energy & Environment)
- **라이선스**: MIT License

## 목차

1. [개요](#개요)
2. [적용 범위](#적용-범위)
3. [용어 정의](#용어-정의)
4. [심해 깊이 구역](#심해-깊이-구역)
5. [잠수정 및 탐사 장비](#잠수정-및-탐사-장비)
6. [음향 매핑 및 탐지](#음향-매핑-및-탐지)
7. [심해 생물 다양성](#심해-생물-다양성)
8. [열수 분출공 및 지질 구조](#열수-분출공-및-지질-구조)
9. [압력 시스템 및 안전](#압력-시스템-및-안전)
10. [시료 채취 방법](#시료-채취-방법)
11. [연구 탐사 프로토콜](#연구-탐사-프로토콜)
12. [데이터 표준](#데이터-표준)
13. [API 사양](#api-사양)
14. [보안 및 윤리](#보안-및-윤리)
15. [참고 문헌](#참고-문헌)

---

## 1. 개요

### 1.1 목적

WIA-ENE-040 심해 탐사 표준은 해양 심해부(200m 이상 깊이)의 과학적 탐사, 생물다양성 연구, 지질학적 조사, 자원 평가를 위한 국제 표준을 제공합니다.

### 1.2 배경

- 심해는 지구 표면의 약 65%를 차지하지만 5% 미만만 탐사됨
- 심해 생태계는 독특한 생물종과 생화학적 프로세스를 보유
- 열수 분출공은 생명의 기원과 극한 환경 생물학 연구의 핵심
- 심해 광물 자원(망간 단괴, 해저 열수광상)의 지속가능한 개발 필요
- 기후 변화, 해양 산성화, 심해 어업이 심해 생태계에 미치는 영향 연구

### 1.3 적용 대상

- 해양 연구 기관 및 대학
- 심해 탐사 선박 운영자
- ROV/AUV 제조사 및 운영사
- 해양 생물학자 및 지질학자
- 해저 자원 탐사 기업
- 국가 해양 관리 기관
- 환경 영향 평가 기관

---

## 2. 적용 범위

### 2.1 깊이 범위

- **중층대 (Mesopelagic)**: 200-1,000m
- **심해대 (Bathypelagic)**: 1,000-4,000m
- **초심해대 (Abyssopelagic)**: 4,000-6,000m
- **해덕대 (Hadopelagic)**: 6,000-11,000m (해구)

### 2.2 탐사 활동

- 생물다양성 조사 및 종 발견
- 해저 지형 매핑 (멀티빔 음향측심)
- 열수 분출공 탐사 및 모니터링
- 냉수용출대 (Cold Seep) 연구
- 심해 퇴적물 코어링
- 해저 광물 자원 탐사
- 심해 플라스틱 및 오염 조사
- 생물 시료 채취 및 유전자 분석

### 2.3 지리적 범위

- 태평양 (Pacific Ocean)
- 대서양 (Atlantic Ocean)
- 인도양 (Indian Ocean)
- 북극해 (Arctic Ocean)
- 남극해 (Southern Ocean)
- 주요 해구: 마리아나 해구, 통가 해구, 필리핀 해구, 케르마덱 해구

---

## 3. 용어 정의

### 3.1 심해 구역 (Depth Zones)

| 용어 | 깊이 범위 | 특징 |
|------|---------|------|
| **대륙붕 (Continental Shelf)** | 0-200m | 광합성 가능, 높은 생물생산성 |
| **심해저 평원 (Abyssal Plain)** | 3,000-6,000m | 평평한 퇴적층, 낮은 생물밀도 |
| **해저산 (Seamount)** | 다양 | 해저 화산, 높은 생물다양성 |
| **해구 (Trench)** | 6,000-11,000m | 지구 최심부, 극한 압력 |
| **중앙 해령 (Mid-Ocean Ridge)** | 2,000-4,000m | 해저 확장 지대, 열수 분출공 |

### 3.2 잠수정 유형 (Submersible Types)

- **HOV (Human-Occupied Vehicle)**: 유인 잠수정 (예: Alvin, Shinkai 6500, Limiting Factor)
- **ROV (Remotely Operated Vehicle)**: 원격 조종 무인 잠수정 (케이블 연결)
- **AUV (Autonomous Underwater Vehicle)**: 자율 무인 잠수정 (독립 항해)
- **Lander**: 자유낙하 착륙선 (계류형 관측 시스템)
- **Benthic Crawler**: 해저면 이동 로봇

### 3.3 센서 및 장비

- **CTD (Conductivity-Temperature-Depth)**: 수온, 염분, 깊이 측정
- **Multibeam Sonar**: 멀티빔 음향측심기 (해저 지형 매핑)
- **Side-scan Sonar**: 측면주사 음파탐지기
- **Sub-bottom Profiler**: 해저 하부 퇴적층 탐사
- **Manipulator Arm**: 로봇 팔 (시료 채취)
- **Suction Sampler**: 흡입식 채집기
- **Push Core**: 퇴적물 코어 채취기
- **Niskin Bottle**: 해수 채집병

---

## 4. 심해 깊이 구역

### 4.1 Bathyal Zone (대륙사면대, 200-4,000m)

**환경 특성**:
- 수온: 4-15°C
- 압력: 20-400 atm
- 빛: 200-1,000m는 박명대 (Twilight Zone), 1,000m 이하는 완전 암흑
- 용존산소: 중간 수준 (산소 최소대 존재)

**생물학적 특징**:
- 발광 생물 (Bioluminescence) 다수
- 심해 어류: 초롱아귀, 흑대구, 칠성장어
- 심해 오징어, 심해 문어
- 냉수 산호 (Cold-water Coral)
- 해면동물 (Sponges)

**탐사 활동**:
- ROV/AUV 조사
- 트롤 어획 (Trawl Sampling)
- 저서 슬레드 (Benthic Sled)
- 멀티빔 매핑

### 4.2 Abyssal Zone (심해 평원대, 4,000-6,000m)

**환경 특성**:
- 수온: 2-4°C
- 압력: 400-600 atm
- 해저면: 주로 퇴적물 (Ooze, Clay)
- 영양분: 매우 낮음 (Marine Snow에 의존)

**생물학적 특징**:
- 해삼 (Sea Cucumber)
- 불가사리 (Starfish)
- 심해 갯지렁이 (Polychaete Worms)
- 박테리아 매트 (Bacterial Mats)
- 초거대동물군 (Megafauna) 희소

**탐사 활동**:
- HOV 잠수 (수심 허용 시)
- 심해 Lander 배치
- Box Core, Gravity Core 채취
- 장기 모니터링 (Time-lapse Camera)

### 4.3 Hadal Zone (해덕대, 6,000-11,000m)

**환경 특성**:
- 수온: 1-4°C
- 압력: 600-1,100 atm (최대 1,086 atm at 10,994m)
- 위치: 주로 해구 (Trenches)
- 퇴적 속도: 대륙붕보다 높음 (중력류)

**생물학적 특징**:
- 초거대 단각류 (Amphipods, 30cm 이상)
- 심해 달팽이류 (Snailfish, Pseudoliparis swirei)
- 극한 환경 박테리아
- 엔데믹 종(고유종) 비율 높음

**탐사 활동**:
- Full-ocean-depth HOV (Limiting Factor, Fendouzhe)
- Hadal Lander 배치
- 바이트 압력 시료 회수
- eDNA (environmental DNA) 분석

---

## 5. 잠수정 및 탐사 장비

### 5.1 유인 잠수정 (HOV)

#### 5.1.1 주요 사양

| 잠수정 | 최대 수심 | 승무원 | 운영 국가 | 특징 |
|--------|----------|-------|----------|------|
| **Alvin (DSV-2)** | 6,500m | 3명 | 미국 | WHOI 운영, 열수구 발견 |
| **Shinkai 6500** | 6,527m | 3명 | 일본 | JAMSTEC 운영 |
| **Jiaolong (蛟龙号)** | 7,062m | 3명 | 중국 | 마리아나 해구 탐사 |
| **Limiting Factor** | 11,000m | 2명 | 미국 (민간) | Full-ocean-depth, 티타늄 구체 |
| **Fendouzhe (奋斗者号)** | 10,909m | 3명 | 중국 | 마리아나 해구 반복 탐사 |

#### 5.1.2 핵심 시스템

```yaml
hull:
  material: "Titanium Alloy (Ti-6Al-4V)"
  shape: "Sphere (최소 표면적 대 부피비)"
  viewport: "Acrylic Cone (20cm 두께)"

propulsion:
  thrusters: 8개 (전후, 좌우, 상하)
  battery: "Lithium-ion (10-12시간 작동)"

life_support:
  oxygen: "Chemical scrubber (CO2 제거)"
  duration: "72-96시간 (비상시)"

safety:
  ballast: "Steel weights (비상 투하)"
  buoyancy: "Syntactic foam (부력재)"
  communication: "Acoustic modem (수중), Radio (수면)"
```

### 5.2 원격 조종 무인 잠수정 (ROV)

#### 5.2.1 작업급 ROV (Work-Class ROV)

**대표 기종**: Jason (WHOI), ROPOS (CSSF), ROV SuBastian (Schmidt Ocean Institute)

**주요 기능**:
- 7-function 유압 매니퓰레이터 (정밀 작업 가능)
- HD/4K 카메라 (줌, 팬, 틸트)
- LED 조명 (20,000-40,000 루멘)
- CTD, DO, pH 센서 실시간 측정
- 시료 바구니 (Bio-box, Quiver)
- 최대 수심: 6,000-11,000m

**데이터 전송**:
- Fiber-optic tether (광케이블): 실시간 HD 영상, 센서 데이터
- Bandwidth: 1-10 Gbps
- Latency: 100-500ms (수심에 따라)

#### 5.2.2 ROV 표준 센서 구성

```typescript
interface ROVSensorSuite {
  navigation: {
    dvl: "Doppler Velocity Log";           // 해저면 대비 속도 측정
    usbl: "Ultra-Short Baseline";          // 수중 위치 측정
    ins: "Inertial Navigation System";     // 관성 항법
    depth: "Pressure sensor (0.01% FS)";   // 수심 측정
  };

  imaging: {
    mainCamera: "4K HD Camera (30fps)";
    zoomCamera: "10x Optical Zoom";
    stillCamera: "12MP Still Camera";
    laserScaler: "Parallel lasers (10cm apart)"; // 크기 참조
  };

  environmental: {
    ctd: "Conductivity-Temperature-Depth";
    do: "Dissolved Oxygen";
    orp: "Oxidation-Reduction Potential";
    ph: "pH sensor";
    turbidity: "Optical Backscatter";
  };

  sonar: {
    forwardLooking: "Multibeam (120° FOV)";
    imaging: "Dual-frequency imaging sonar";
    profiler: "Sub-bottom profiler";
  };
}
```

### 5.3 자율 무인 잠수정 (AUV)

#### 5.3.1 AUV 특징

- **장점**: 대규모 지역 매핑, 장시간 임무, 케이블 불필요
- **단점**: 실시간 제어 불가, 시료 채취 제한적

**대표 기종**:
- **Sentry (WHOI)**: 6,000m, 멀티빔 매핑 전문
- **ABE (WHOI)**: 열수구 탐지 특화 (현재 퇴역)
- **Hugin (Kongsberg)**: 상업용, 석유/가스 탐사

#### 5.3.2 AUV 임무 프로파일

```yaml
mission_planning:
  survey_type: "Lawn-mower pattern"
  altitude: "50-100m above seafloor"
  speed: "1-3 knots"
  line_spacing: "100-200m (멀티빔 커버리지 고려)"

sensors:
  - "Multibeam Sonar (100-400 kHz)"
  - "Side-scan Sonar"
  - "Sub-bottom Profiler"
  - "CTD"
  - "Magnetometer (자기 이상 감지)"
  - "Camera (트랜식트 촬영)"

navigation:
  surface: "GPS + Compass"
  submerged: "INS + DVL + USBL beacons"

communication:
  underwater: "Acoustic modem (낮은 대역폭)"
  surface: "Iridium satellite (임무 보고)"
```

### 5.4 Lander 시스템

#### 5.4.1 Hadal Lander 설계

```yaml
structure:
  frame: "Aluminum or titanium frame"
  buoyancy: "Syntactic foam spheres"
  ballast: "Steel or iron weights"

deployment:
  method: "Free-fall descent (1-4 hours)"
  navigation: "Acoustic transponder"

recovery:
  trigger: "Acoustic release command"
  ascent: "Ballast drop, buoyancy ascent (2-3 hours)"

instruments:
  camera: "Baited camera trap (time-lapse)"
  sediment_trap: "Particle flux collector"
  water_sampler: "Niskin bottles (multiple depths)"
  sensors: "CTD, DO, Current meter"

duration:
  short_term: "24-72 hours"
  long_term: "6-12 months (battery limited)"
```

---

## 6. 음향 매핑 및 탐지

### 6.1 멀티빔 음향측심 (Multibeam Echosounder)

#### 6.1.1 원리

- 음파 펄스를 부채꼴로 송신 (100-500 빔)
- 해저면 반사 시간 → 깊이 계산
- 음향 강도 (Backscatter) → 해저 물질 특성

#### 6.1.2 시스템 사양

| 주파수 | 최대 수심 | 해상도 | 용도 |
|--------|----------|-------|------|
| **12 kHz** | 11,000m | 낮음 (50-100m) | 전해양 깊이 매핑 |
| **30 kHz** | 5,000m | 중간 (20-50m) | 심해 평원 |
| **100 kHz** | 1,000m | 높음 (1-5m) | 대륙붕, 상세 조사 |
| **400 kHz** | 200m | 매우 높음 (<1m) | 천해, 인공 구조물 |

#### 6.1.3 데이터 처리

```yaml
raw_data:
  format: "XTF (eXtended Triton Format), GSF (Generic Sensor Format)"
  size: "100 GB - 1 TB per day (고해상도)"

processing_steps:
  1_sound_velocity_correction:
    method: "CTD profile-based ray tracing"
    software: "SIS, PDS2000, CARIS HIPS"

  2_tide_correction:
    source: "NOAA tidal models, RTK-GPS"

  3_artifact_removal:
    filters: "Outlier detection, Nadir gap filling"

  4_gridding:
    resolution: "1m, 5m, 10m grids (DTM, DSM)"
    interpolation: "Kriging, Inverse Distance Weighting"

  5_backscatter_processing:
    output: "Acoustic backscatter mosaic"
    classification: "Sediment type (mud, sand, rock, gravel)"

deliverables:
  - "Bathymetric grid (GeoTIFF, NetCDF)"
  - "Backscatter mosaic"
  - "3D surface model"
  - "Slope, aspect, rugosity maps"
```

### 6.2 측면주사 음파탐지 (Side-scan Sonar)

**용도**: 해저면 미세 구조, 난파선, 해저 케이블 탐지

**주파수**:
- 100 kHz: 최대 거리 500m, 해상도 10cm
- 500 kHz: 최대 거리 100m, 해상도 2cm

**산출물**: 음향 이미지 (Acoustic Image), 그림자 분석

### 6.3 서브보텀 프로파일러 (Sub-bottom Profiler)

**원리**: 저주파 음파 (2-15 kHz)가 퇴적층 투과

**목적**:
- 퇴적층 구조 (층서)
- 단층, 가스 치밍 (Gas Chimney) 탐지
- 해저 파이프라인 매설 깊이 확인

**침투 깊이**: 10-100m (퇴적물 유형에 따라)

---

## 7. 심해 생물 다양성

### 7.1 생물 분류 체계

#### 7.1.1 크기 분류

| 분류 | 크기 | 예시 |
|------|------|------|
| **Megafauna** | >1cm (육안 식별 가능) | 어류, 갑각류, 해삼, 불가사리 |
| **Macrofauna** | 0.3-1cm | 다모류, 소형 갑각류 |
| **Meiofauna** | 32μm-0.3mm | 선충류, 유공충 |
| **Microfauna** | <32μm | 박테리아, 고세균 |

#### 7.1.2 생태 분류

- **Pelagic**: 수층 생물 (유영, 부유 생물)
- **Benthic**: 저서 생물 (해저면 서식)
- **Epibenthic**: 해저면 위 (걷기, 기어다님)
- **Infaunal**: 해저 퇴적물 속 (굴 파고 서식)

### 7.2 심해 적응 특성

#### 7.2.1 고압 적응

- **세포막 유동성**: 불포화 지방산 비율 증가
- **압력 적응 단백질**: Piezolytes (TMAO, betaine 등)
- **유전자 발현**: 압력 반응 유전자 (HSP, CSP)

#### 7.2.2 발광 (Bioluminescence)

**메커니즘**:
- **Luciferin-Luciferase 반응**: 화학발광
- **공생 발광 박테리아**: Photobacterium, Vibrio fischeri

**기능**:
- **Counter-illumination**: 위에서 보는 실루엣 제거
- **Lure**: 먹이 유인 (초롱아귀)
- **Communication**: 짝짓기 신호
- **Defense**: 포식자 혼란

#### 7.2.3 대사율 저하

- **낮은 체온**: 주변 수온 (2-4°C)
- **느린 성장**: 수십 년 성성숙
- **낮은 번식률**: 소수 난 생산, 높은 투자

### 7.3 열수 분출공 생물 군집

#### 7.3.1 화학합성 생태계

**에너지원**: H₂S, CH₄, H₂, Fe²⁺ (화학합성 박테리아)

**1차 생산자**: 황 산화 박테리아 (Sulfur-oxidizing bacteria)

**화학 반응**:
```
H₂S + 2O₂ → SO₄²⁻ + 2H⁺ (에너지 생성)
CO₂ + 4H₂ → CH₄ + 2H₂O (메탄 생성)
```

#### 7.3.2 주요 생물

| 생물 | 학명 | 특징 |
|------|------|------|
| **관벌레** | Riftia pachyptila | 최대 2m, 공생 박테리아 보유 |
| **Yeti Crab** | Kiwa hirsuta | 털 속 박테리아 배양 |
| **Scaly-foot Snail** | Chrysomallon squamiferum | 철-황 껍질 |
| **심해 홍합** | Bathymodiolus | 메탄, 황 산화 박테리아 공생 |

### 7.4 생물다양성 조사 방법

#### 7.4.1 영상 기반 조사

**Transect Survey**:
- ROV/HOV 일정 고도(2-5m) 유지
- HD 영상 연속 촬영
- 레이저 스케일러로 크기 측정
- 분석: 개체수 카운팅, 크기 분포, 공간 분포

**Quadrat Sampling**:
- 정해진 면적(0.25-1m²) 집중 촬영
- 종 조성, 피도 분석

#### 7.4.2 시료 채취

**방법**:
- **Slurp Gun**: 흡입식 채집 (작은 생물, 정밀)
- **Scoop Net**: 망으로 채집
- **Box Core**: 퇴적물 블록 채취 (0.25m²)
- **Epibenthic Sled**: 대형 저서 생물 채집

**보존**:
- **냉동**: -80°C (유전자 분석용)
- **에탄올/포르말린**: 형태 보존
- **압력 유지**: Isobaric chambers (심해 압력 유지)

#### 7.4.3 환경 DNA (eDNA)

**절차**:
1. 해수 시료 채취 (Niskin bottle, 1-10L)
2. 현장 여과 (0.22μm 필터)
3. DNA 추출 (Qiagen DNeasy kit)
4. PCR 증폭 (18S rRNA, COI 바코드)
5. 차세대 염기서열 분석 (Illumina, Nanopore)
6. 생물정보학 분석 (OTU clustering, BLAST)

**장점**:
- 비침습적
- 대량 종 검출 (희귀종 포함)
- 유생, 알 등 미세 개체 검출

---

## 8. 열수 분출공 및 지질 구조

### 8.1 열수 분출공 유형

#### 8.1.1 Black Smoker (흑연 분출공)

**특성**:
- 온도: 300-400°C
- 유체: 황화물(FeS, CuS) 풍부 → 흑색 플룸
- 구조: 황화물 굴뚝 (최대 60m 높이)
- 위치: 중앙 해령 (Mid-Ocean Ridge)

**화학 조성**:
- pH: 3-5 (산성)
- H₂S: 1-10 mmol/kg
- CH₄: 0.1-10 mmol/kg
- 금속: Fe, Cu, Zn, Pb, Ag, Au

#### 8.1.2 White Smoker (백연 분출공)

**특성**:
- 온도: 100-300°C
- 유체: 바륨(BaSO₄), 규산(SiO₂) → 백색 플룸
- pH: 5-7

#### 8.1.3 Lost City 유형

**특성**:
- 온도: 40-90°C
- 유체: 알칼리성 (pH 9-11)
- 메커니즘: 감람암(peridotite) 뱀문암화 (serpentinization)
- 에너지원: H₂ (수소) 풍부

**반응식**:
```
Mg₁.₈Fe₀.₂SiO₄ + H₂O → Mg₃Si₂O₅(OH)₄ + Mg(OH)₂ + H₂
(감람석)         (사문석)     (수산화마그네슘) (수소)
```

### 8.2 냉수용출대 (Cold Seep)

**특성**:
- 온도: 주변 해수와 유사 (2-4°C)
- 유체: CH₄, H₂S, 석유, 가스
- 메커니즘: 퇴적층 압축, 가스 하이드레이트 해리

**생물 군집**:
- 메탄 산화 박테리아
- Bathymodiolus 홍합
- Calyptogena 조개
- 관벌레 (일부 종)

### 8.3 해저 화산 및 해저산

**유형**:
- **Active Volcano**: 마그마 활동 중 (화산 가스, 용암)
- **Seamount**: 해저 화산 (정상이 수면 하 >1,000m)
- **Guyot**: 평정 해저산 (침식된 화산섬)

**탐사 목적**:
- 화산 분출 모니터링
- 광물 자원 (망간 피각)
- 고유종 생물 다양성

---

## 9. 압력 시스템 및 안전

### 9.1 압력 계산

**압력 공식**:
```
P(atm) = 1 + (depth_m / 10)
P(bar) = 1 + (depth_m / 10)
P(MPa) = depth_m × 0.1 × ρ × g
```

**예시**:
- 1,000m: 101 atm (10.1 MPa)
- 6,000m: 601 atm (60.1 MPa)
- 11,000m: 1,101 atm (110.1 MPa)

### 9.2 압력 시료 유지

#### 9.2.1 Isobaric Gas-tight Sampler

**원리**:
- 심해에서 시료 채취 후 압력 유지
- 수면 도달 후에도 원래 압력 유지

**용도**:
- 압력 의존 미생물 배양
- 가스 하이드레이트 시료
- 압력-온도 조건 실험

### 9.3 감압병 (Decompression Sickness) - 잠수부

**심해 잠수는 대부분 무인/유인 잠수정이지만, 포화잠수(Saturation Diving) 시**:

- **Saturation Diving**: 200-300m 깊이, 수주~수개월 거주
- **감압 시간**: 1일/30m (예: 300m → 10일 감압)

---

## 10. 시료 채취 방법

### 10.1 퇴적물 코어링

#### 10.1.1 유형별 비교

| 유형 | 침투 깊이 | 직경 | 용도 |
|------|----------|------|------|
| **Push Core** | 0.5-1m | 5-10cm | ROV/HOV, 표층 퇴적물 |
| **Gravity Core** | 5-20m | 10-15cm | 자유낙하, 층서 연구 |
| **Piston Core** | 20-50m | 10-15cm | 피스톤 기구, 고기후 연구 |
| **IODP Drilling** | >1,000m | 10-20cm | 시추선, 심부 지층 |

#### 10.1.2 코어 처리 절차

```yaml
onboard_processing:
  1_core_recovery:
    - "Core liner cutting and capping"
    - "Length measurement"
    - "Photography (whole core)"

  2_whole_core_logging:
    - "Multi-sensor core logger (MSCL)"
    - "Gamma-ray density"
    - "P-wave velocity"
    - "Magnetic susceptibility"

  3_core_splitting:
    - "Longitudinal split (archive + working half)"
    - "Archive half: permanent storage"
    - "Working half: sampling"

  4_visual_description:
    - "Sediment color (Munsell chart)"
    - "Texture, grain size"
    - "Sedimentary structures"
    - "Bioturbation index"

  5_sampling:
    - "Micropaleontology (foraminifera, radiolaria)"
    - "Geochemistry (C/N, δ13C, δ15N)"
    - "Microbiology (DNA, cell counts)"
    - "Dating (14C, 210Pb, tephra)"
```

### 10.2 암석 시료 채취

**방법**:
- **Manipulator Arm**: ROV 로봇팔로 암석 집기
- **Rock Drill**: 심해 드릴 (코어 추출)
- **Grab Sampler**: 그랩 채집기

**대상**:
- 열수 분출공 황화물 굴뚝
- 현무암(Basalt) - 해령 암석
- 망간 단괴 (Polymetallic Nodules)
- 망간 피각 (Ferromanganese Crusts)

### 10.3 생물 시료 채취

#### 10.3.1 ROV Biobox 구성

```yaml
biobox_chambers:
  chamber_1:
    temperature: "In-situ (2-4°C)"
    type: "Insulated, closed"
    use: "Live specimens"

  chamber_2:
    preservative: "95% Ethanol"
    use: "DNA/morphology"

  chamber_3:
    preservative: "10% Formalin"
    use: "Histology, morphology"

  chamber_4:
    type: "Cryogenic (-80°C)"
    use: "Enzyme, protein preservation"
```

---

## 11. 연구 탐사 프로토콜

### 11.1 탐사 계획

#### 11.1.1 사전 조사

```yaml
desktop_study:
  bathymetry: "Satellite altimetry, GEBCO"
  literature: "Previous expeditions, publications"
  permits: "EEZ permissions, UNCLOS compliance"

site_selection:
  criteria:
    - "Scientific priority"
    - "Weather window"
    - "Logistical feasibility"
    - "Safety assessment"
```

#### 11.1.2 임무 일정

**표준 Deep-Sea Expedition (30일)**:

| 일차 | 활동 |
|------|------|
| 1-2 | 항해 (Transit to site) |
| 3 | CTD casts, Multibeam mapping |
| 4-25 | ROV/HOV 잠수 (20회, 1회/일) |
| 26-28 | 추가 매핑, 백업 임무 |
| 29-30 | 귀항 (Return to port) |

**ROV Dive 일정 (8-10시간)**:
```
00:00 - ROV Launch
00:30 - Descent (1,000m/hr)
03:00 - Seafloor arrival
03:00-07:00 - Science operations
07:00-08:00 - Ascent
08:00 - ROV Recovery
```

### 11.2 데이터 관리

#### 11.2.1 데이터 유형

```yaml
navigation:
  format: "USBL .csv, EIVA NaviPac"
  frequency: "1 Hz"

video:
  format: "H.264, ProRes (HD/4K)"
  storage: "10-50 TB per expedition"
  metadata: "Timecode, position, depth, annotations"

sensor:
  ctd: "1 Hz, .cnv (Sea-Bird format)"
  sonar: ".xtf, .gsf"

samples:
  catalog: "Sample ID, location, type, preservation"
  database: "MySQL, PostgreSQL"
```

#### 11.2.2 메타데이터 표준

**Darwin Core**: 생물 시료 메타데이터

**ISO 19115**: 지리공간 메타데이터

**OBIS (Ocean Biodiversity Information System)**: 해양 생물 데이터베이스

### 11.3 안전 프로토콜

#### 11.3.1 HOV 비상 절차

```yaml
emergency_scenarios:
  entanglement:
    action: "Manipulator arm cut, ballast drop"

  power_loss:
    action: "Switch to backup battery, initiate ascent"

  fire:
    action: "CO2 extinguisher, emergency ascent"

  life_support_failure:
    action: "Emergency oxygen, surface immediately"

  stuck_on_bottom:
    action: "Ballast drop, syntactic foam buoyancy (auto-ascent)"
```

#### 11.3.2 ROV 복구

**Tether Breakage**:
1. ROV activates acoustic beacon
2. Side-scan sonar search
3. Grapple hook recovery or free ascent (ballast drop)

---

## 12. 데이터 표준

### 12.1 탐사 기본 정보

```json
{
  "expeditionId": "EXP-2025-001",
  "vessel": {
    "name": "R/V Falkor (too)",
    "operator": "Schmidt Ocean Institute",
    "imo": "9876543",
    "length": 110.0,
    "classification": "Research Vessel"
  },
  "location": {
    "region": "Mariana Trench",
    "area": "Challenger Deep",
    "coordinates": {
      "latitude": 11.3733,
      "longitude": 142.5917
    },
    "eez": "USA (Northern Mariana Islands)"
  },
  "duration": {
    "startDate": "2025-03-01T00:00:00Z",
    "endDate": "2025-03-30T23:59:59Z",
    "daysAtSea": 30
  },
  "permits": [
    {
      "type": "EEZ Research Permit",
      "issuedBy": "NOAA",
      "permitNumber": "NOAA-2025-001",
      "validUntil": "2025-12-31"
    }
  ]
}
```

### 12.2 잠수 기록 (Dive Log)

```json
{
  "diveId": "DIVE-HOV-2025-045",
  "expeditionId": "EXP-2025-001",
  "submersible": {
    "type": "HOV",
    "name": "Limiting Factor",
    "maxDepth": 10908,
    "crew": [
      {"name": "Victor Vescovo", "role": "Pilot"},
      {"name": "Dr. Jane Smith", "role": "Scientist"}
    ]
  },
  "timeline": {
    "launch": "2025-03-15T08:00:00Z",
    "onBottom": "2025-03-15T11:30:00Z",
    "offBottom": "2025-03-15-15:30:00Z",
    "recovery": "2025-03-15T19:00:00Z",
    "bottomTime": 14400
  },
  "navigation": {
    "launchPosition": {"lat": 11.3733, "lon": 142.5917},
    "bottomTrack": "geojson/tracks/dive045.geojson",
    "maxDepth": 10908,
    "areaExplored": 2.5
  },
  "observations": {
    "organisms": 47,
    "newSpecies": 3,
    "samples": 12,
    "rocks": 5
  }
}
```

### 12.3 생물 관찰 기록

```json
{
  "observationId": "OBS-2025-001234",
  "diveId": "DIVE-ROV-2025-102",
  "timestamp": "2025-03-20T14:35:22Z",
  "location": {
    "latitude": 11.3750,
    "longitude": 142.5920,
    "depth": 8340,
    "depthZone": "hadal"
  },
  "organism": {
    "taxonId": "AMPHIPODA-SP001",
    "scientificName": "Hirondellea gigas",
    "commonName": "Supergiant Amphipod",
    "identificationLevel": "species",
    "identifiedBy": "Dr. Alan Jamieson"
  },
  "morphology": {
    "length": 34.0,
    "width": 8.0,
    "color": "Translucent white",
    "notes": "Scavenging on bait"
  },
  "behavior": "Feeding",
  "abundance": "15-20 individuals",
  "associatedSpecies": ["Notoliparis kermadecensis"],
  "media": [
    {"type": "video", "file": "ROV102_14-35-22.mp4", "timecode": "02:35:22"},
    {"type": "photo", "file": "IMG_8340_001234.jpg", "resolution": "4K"}
  ]
}
```

### 12.4 시료 메타데이터

```json
{
  "sampleId": "SAMPLE-2025-MT-001",
  "expeditionId": "EXP-2025-001",
  "diveId": "DIVE-ROV-2025-102",
  "timestamp": "2025-03-20T15:10:00Z",
  "location": {
    "latitude": 11.3755,
    "longitude": 142.5918,
    "depth": 8365,
    "habitat": "Hadal sediment"
  },
  "sampleType": "sediment_core",
  "method": "Push core",
  "dimensions": {
    "length": 45.0,
    "diameter": 6.0
  },
  "preservation": "Frozen (-80°C)",
  "storage": {
    "facility": "Scripps Institution of Oceanography",
    "freezer": "ULT-15",
    "box": "MT-2025-A",
    "position": "A1"
  },
  "analyses": [
    "Grain size",
    "Organic carbon",
    "Microbial DNA (16S rRNA)",
    "Radioisotope dating (210Pb, 14C)"
  ],
  "curator": "Dr. Lisa Levin",
  "availability": "Available upon request"
}
```

### 12.5 열수 분출공 데이터

```json
{
  "ventId": "VENT-EPR-2025-003",
  "ventField": "East Pacific Rise 9°50'N",
  "discoveryDate": "2025-04-10",
  "location": {
    "latitude": 9.8333,
    "longitude": -104.2917,
    "depth": 2520
  },
  "ventType": "black_smoker",
  "structure": {
    "height": 15.5,
    "diameter": 2.3,
    "material": "Sulfide (FeS, CuS, ZnS)"
  },
  "fluidChemistry": {
    "temperature": 365,
    "pH": 3.8,
    "h2s": 6.5,
    "ch4": 0.8,
    "metals": {
      "fe": 120,
      "cu": 45,
      "zn": 78
    }
  },
  "biology": {
    "dominantSpecies": "Riftia pachyptila",
    "abundance": "High density (>100/m²)",
    "community": ["Bathymodiolus thermophilus", "Alvinella pompejana"]
  }
}
```

---

## 13. API 사양

### 13.1 RESTful API 엔드포인트

#### 13.1.1 탐사 관리

```
GET    /api/v1/expeditions
GET    /api/v1/expeditions/{expeditionId}
POST   /api/v1/expeditions
PUT    /api/v1/expeditions/{expeditionId}
DELETE /api/v1/expeditions/{expeditionId}
```

#### 13.1.2 잠수 기록

```
GET    /api/v1/expeditions/{expeditionId}/dives
GET    /api/v1/dives/{diveId}
POST   /api/v1/dives
PUT    /api/v1/dives/{diveId}

# 잠수별 데이터
GET    /api/v1/dives/{diveId}/observations
GET    /api/v1/dives/{diveId}/samples
GET    /api/v1/dives/{diveId}/track
```

#### 13.1.3 생물 관찰

```
GET    /api/v1/observations
GET    /api/v1/observations/{observationId}
POST   /api/v1/observations
PUT    /api/v1/observations/{observationId}

# 필터링
GET    /api/v1/observations?depthZone=hadal&taxon=Amphipoda
GET    /api/v1/observations?minDepth=6000&maxDepth=11000
```

#### 13.1.4 시료 관리

```
GET    /api/v1/samples
GET    /api/v1/samples/{sampleId}
POST   /api/v1/samples
PUT    /api/v1/samples/{sampleId}

# 시료 가용성
GET    /api/v1/samples/available
POST   /api/v1/samples/{sampleId}/request
```

#### 13.1.5 열수 분출공 데이터베이스

```
GET    /api/v1/vents
GET    /api/v1/vents/{ventId}
POST   /api/v1/vents
PUT    /api/v1/vents/{ventId}

# 지역별 검색
GET    /api/v1/vents?region=mid_atlantic_ridge
GET    /api/v1/vents?minTemp=300&ventType=black_smoker
```

#### 13.1.6 멀티빔 데이터

```
POST   /api/v1/bathymetry/upload
GET    /api/v1/bathymetry/{surveyId}
GET    /api/v1/bathymetry/grid?bbox=lon1,lat1,lon2,lat2&resolution=10m

# 3D 모델
GET    /api/v1/bathymetry/{surveyId}/3d-model
```

### 13.2 WebSocket 실시간 스트리밍

#### 13.2.1 실시간 잠수 추적

```javascript
ws://api.wia.org/ene-040/v1/dives/{diveId}/live

// Message format
{
  "timestamp": "2025-03-15T14:35:22Z",
  "position": {"lat": 11.3755, "lon": 142.5918},
  "depth": 8365.2,
  "heading": 270,
  "speed": 0.5,
  "temperature": 2.1,
  "salinity": 34.7,
  "camera": "HD1 active",
  "status": "on_bottom"
}
```

---

## 14. 보안 및 윤리

### 14.1 데이터 접근 권한

**공개 데이터 (Public)**:
- 멀티빔 해저 지형 (저해상도, 50m 그리드)
- 생물 관찰 목록 (종 수준)
- 메타데이터 (탐사 개요, 위치, 날짜)

**제한 데이터 (Restricted)**:
- 고해상도 멀티빔 (<10m) - 2년 엠바고
- 생물 시료 유전자 데이터 - 연구자 동의 필요
- 광물 자원 정밀 위치 - 국가 안보

**비공개 데이터 (Confidential)**:
- 상업 탐사 데이터
- 군사 관련 데이터
- 민감 생태계 정확 좌표 (밀렵 방지)

### 14.2 생물 시료 윤리

**나고야 의정서 (Nagoya Protocol) 준수**:
- 유전자원 접근 및 이익 공유 (ABS)
- 출처국 사전 동의 (PIC)
- 상호 합의 조건 (MAT)

**Best Practices**:
- 최소 침습 채집
- 멸종위기종 보호
- 지역 연구자 협력
- 데이터 및 시료 공유

### 14.3 환경 보호

**UNCLOS (유엔 해양법 협약)**:
- 공해 자유 vs. 환경 보호 의무
- 국제 해저 기구 (ISA) 규정

**탐사 영향 최소화**:
- 해저 접촉 최소화
- 생물 서식지 교란 금지
- 폐기물 해양 투기 금지
- 외래종 유입 방지 (Ballast water management)

---

## 15. 참고 문헌

### 15.1 주요 논문

1. **Jamieson, A.J. (2015)**. "The Hadal Zone: Life in the Deepest Oceans." Cambridge University Press.

2. **Ramirez-Llodra, E. et al. (2010)**. "Deep, diverse and definitely different: unique attributes of the world's largest ecosystem." *Biogeosciences*, 7: 2851-2899.

3. **Van Dover, C.L. (2000)**. "The Ecology of Deep-Sea Hydrothermal Vents." Princeton University Press.

4. **Gallo, N.D. et al. (2015)**. "Submersible- and lander-observed community patterns in the Mariana and New Britain trenches: Influence of productivity and depth on epibenthic and scavenging communities." *Deep Sea Research Part I*, 99: 119-133.

### 15.2 데이터베이스

- **OBIS (Ocean Biodiversity Information System)**: https://obis.org/
- **InterRidge Vents Database**: http://vents-data.interridge.org/
- **GEBCO (General Bathymetric Chart of the Oceans)**: https://www.gebco.net/
- **ChEss (Chemosynthetic Ecosystem Science)**: http://www.noc.soton.ac.uk/chess/

### 15.3 국제 기구

- **IODP (International Ocean Discovery Program)**: https://www.iodp.org/
- **ISA (International Seabed Authority)**: https://www.isa.org.jm/
- **INDEEP (International Network for Scientific Investigation of Deep-Sea Ecosystems)**: https://www.indeep-project.org/

---

## 변경 이력

### Version 1.0.0 (2025-12-25)

- 초판 발행
- 완전한 문서 패키지
- TypeScript SDK
- CLI 도구
- API 사양

---

## 라이선스

이 표준은 [MIT License](https://opensource.org/licenses/MIT) 하에 배포됩니다.

---

## 연락처

- **웹사이트**: https://wia.org/standards/ene-040
- **GitHub**: https://github.com/WIA-Official/wia-standards
- **이메일**: standards@wia.org

---

**© 2025 SmileStory Inc. / WIA**

**弘益人間 (홍익인간) · Benefit All Humanity**

---

*이 문서는 국제 해양 과학 커뮤니티, 심해 탐사 기관, 연구 선박 운영자들과의 협력을 통해 개발되었습니다. 심해는 지구에서 가장 큰 생물권이며, 우리의 탐사와 보호는 미래 세대를 위한 책임입니다.*
