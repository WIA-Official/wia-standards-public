# 제6장: 프로토콜 및 알고리즘

## 가뭄 지수 계산 및 데이터 처리를 위한 과학적 방법

---

## 6.1 대기 보정 프로토콜 (6S 모델)

### 대기 보정의 필요성

위성 센서는 지구 표면이 아닌 대기 상단을 관측합니다. 대기는 다음을 통해 개입합니다:

| 대기 효과 | 영향 | 보정 접근법 |
|----------|------|-----------|
| 레일리 산란 | 청색 안개 추가, 대비 감소 | 분자 산란 모델 |
| 에어로졸 산란 | 안개, 경로 복사 | 에어로졸 광학 두께 추정 |
| 흡수 | 신호 감소 (O₃, H₂O, CO₂) | 가스 투과율 모델 |
| 인접 효과 | 경계 흐림 | 공간 보정 |

보정 없이는 대기 조건만으로 NDVI 값이 20-40% 변동할 수 있어 시간적 비교를 신뢰할 수 없습니다.

### 6S 복사전달 모델

Second Simulation of a Satellite Signal in the Solar Spectrum(6S)은 포괄적인 대기 보정을 제공합니다:

```
6S 모델 구성요소:
================

1. 대기 프로파일
   ├── 표준 대기 (열대, 중위도 등)
   ├── 맞춤 프로파일 (라디오존데)
   └── 위성 유래 수증기

2. 에어로졸 모델
   ├── 대륙
   ├── 해양
   ├── 도시
   ├── 사막
   └── 바이오매스 연소

3. 표면 특성
   ├── 램버트 가정
   ├── BRDF 모델
   └── 고도 보정

4. 기하학적 매개변수
   ├── 태양 천정/방위각
   ├── 관측 천정/방위각
   └── 태양-타겟-센서 기하
```

### 6S 구현 프로토콜

**1단계: 입력 매개변수 준비**

```python
def prepare_6s_inputs(metadata, atmospheric_data):
    """
    6S 대기 보정을 위한 입력 매개변수 준비.

    매개변수:
    -----------
    metadata : dict
        위성 영상 메타데이터 (기하학, 날짜 등)
    atmospheric_data : dict
        대기 매개변수 (AOD, 수증기, 오존)

    반환값:
    --------
    dict : 6S 입력 구성
    """
    inputs = {
        'geometry': {
            'solar_zenith': metadata['sun_elevation'],
            'solar_azimuth': metadata['sun_azimuth'],
            'view_zenith': metadata['view_zenith'],
            'view_azimuth': metadata['view_azimuth']
        },
        'date': {
            'month': metadata['acquisition_date'].month,
            'day': metadata['acquisition_date'].day
        },
        'atmospheric_profile': select_atmosphere(
            metadata['latitude'],
            metadata['acquisition_date']
        ),
        'aerosol': {
            'model': determine_aerosol_model(metadata['location']),
            'optical_depth_550': atmospheric_data.get('aod_550', 0.1)
        },
        'water_vapor_gcm2': atmospheric_data.get('water_vapor', 2.0),
        'ozone_atm_cm': atmospheric_data.get('ozone', 0.3)
    }

    return inputs
```

**2단계: 표면 반사율 계산**

```python
def calculate_surface_reflectance(toa_radiance, wavelength, params):
    """
    6S를 사용하여 TOA 복사휘도에서 표면 반사율 계산.

    반환값:
    --------
    array : 표면 반사율
    """
    # 6S 초기화 및 실행
    s = SixS()
    # 기하학, 대기 프로파일, 에어로졸 설정...
    s.run()

    # 보정 계수 추출
    xa = s.outputs.coef_xa
    xb = s.outputs.coef_xb
    xc = s.outputs.coef_xc

    # 보정 적용
    y = xa * toa_radiance - xb
    surface_reflectance = y / (1 + xc * y)

    # 유효 범위로 제한
    surface_reflectance = np.clip(surface_reflectance, 0, 1)

    return surface_reflectance
```

---

## 6.2 구름 마스킹 알고리즘 (Fmask)

### 구름 감지의 과제

정확한 구름 마스킹은 식생 모니터링에 매우 중요합니다. 구름과 구름 그림자는 다음을 유발할 수 있습니다:

- 거짓 가뭄 신호 (그림자가 스트레스 받은 식생으로 보임)
- 거짓 회복 신호 (밝은 구름이 합성에 영향)
- 너무 공격적이면 데이터 격차
- 너무 보수적이면 데이터 오염

### Fmask 알고리즘 개요

Function of mask(Fmask) 알고리즘은 다단계 접근법을 사용합니다:

```
Fmask 처리 흐름:
================

1단계: 잠재적 구름 계층 (PCL)
├── 기본 구름 테스트 (밝기, 온도)
├── 백색도 테스트 (구름 vs 밝은 표면)
├── HOT (Haze Optimized Transformation)
└── NIR/SWIR 테스트 (눈/구름 구별)

2단계: 구름 확률
├── 육지/물 구별
├── 온도 확률
├── 스펙트럼 변동성 확률
└── 결합 확률

3단계: 구름 그림자 감지
├── 태양 기하학을 사용한 구름 투영
├── 그림자와 구름 매칭
├── 잠재적 그림자 플러드 필
└── 그림자 기하학 검증

4단계: 눈/얼음 감지
├── NDSI 계산
├── 온도 임계값
└── NIR 밝기 테스트

5단계: 최종 분류
├── 모든 마스크 결합
├── 확률 임계값 적용
├── 구름 가장자리 버퍼
└── QA 플래그 생성
```

### Fmask 구현

```python
def fmask_cloud_detection(bands, metadata, params):
    """
    Fmask 구름 감지 알고리즘 구현.

    매개변수:
    -----------
    bands : dict
        스펙트럼 밴드 사전 (blue, green, red, nir, swir1, swir2, thermal)
    metadata : dict
        영상 메타데이터 (태양 기하학, 날짜 등)
    params : dict
        Fmask 매개변수

    반환값:
    --------
    dict : 구름, 그림자, 눈, 물 마스크
    """
    # 밴드 추출
    blue = bands['blue']
    green = bands['green']
    red = bands['red']
    nir = bands['nir']
    swir1 = bands['swir1']
    bt = bands['thermal']  # 밝기 온도

    # 기본 구름 테스트 - 밝고 차가움
    basic_cloud = (
        (blue > params.get('blue_threshold', 0.15)) &
        (bt < params.get('temp_threshold_high', 300))
    )

    # 백색도 테스트 (구름은 스펙트럼적으로 평평)
    mean_vis = (blue + green + red) / 3
    whiteness = (
        np.abs(blue - mean_vis) +
        np.abs(green - mean_vis) +
        np.abs(red - mean_vis)
    ) / mean_vis

    white_cloud = whiteness < params.get('whiteness_threshold', 0.7)

    # 잠재적 구름 계층 결합
    potential_cloud = basic_cloud & white_cloud

    # 온도 확률
    t_prob = calculate_temperature_probability(bt, metadata)

    # 결합된 구름 확률
    cloud_prob = t_prob

    # 임계값 적용
    cloud_mask = (potential_cloud) & (cloud_prob > params.get('cloud_prob_threshold', 0.225))

    # 구름 가장자리 버퍼
    cloud_mask = binary_dilation(cloud_mask, iterations=params.get('cloud_buffer', 3))

    # 구름 그림자 감지
    shadow_mask = detect_cloud_shadows(cloud_mask, bt, nir, metadata, params)

    return {
        'cloud': cloud_mask,
        'shadow': shadow_mask,
        'cloud_probability': cloud_prob,
        'clear_land': ~(cloud_mask | shadow_mask)
    }
```

### Fmask 품질 플래그

```json
{
  "fmask_quality_flags": {
    "bit_0": {"name": "fill", "description": "데이터 없음"},
    "bit_1": {"name": "clear", "description": "맑은 육지/물"},
    "bit_2": {"name": "cloud", "description": "구름 감지됨"},
    "bit_3": {"name": "cloud_shadow", "description": "구름 그림자"},
    "bit_4": {"name": "snow", "description": "눈/얼음"},
    "bit_5": {"name": "water", "description": "수체"},
    "bit_6_7": {
      "name": "cloud_confidence",
      "values": {
        "00": "결정되지 않음",
        "01": "낮은 신뢰도",
        "10": "중간 신뢰도",
        "11": "높은 신뢰도"
      }
    }
  }
}
```

---

## 6.3 NDVI 계산 및 검증

### NDVI 공식 및 이론

정규화 식생지수(NDVI)는 건강한 식생에 의한 적색 흡수와 NIR 반사 간의 대비를 활용합니다:

```
NDVI = (NIR - Red) / (NIR + Red)

여기서:
- NIR: 근적외선 반사율 (일반적으로 0.76-0.90 μm)
- Red: 적색 반사율 (일반적으로 0.63-0.69 μm)

값 범위: -1 ~ +1
일반적인 식생: 0.2 ~ 0.9
```

### 센서별 밴드 사양

| 센서 | 적색 밴드 | NIR 밴드 | 해상도 |
|------|---------|---------|-------|
| Landsat 8/9 OLI | B4 (0.64-0.67 μm) | B5 (0.85-0.88 μm) | 30m |
| Sentinel-2 MSI | B4 (0.65-0.68 μm) | B8 (0.79-0.90 μm) | 10m |
| 천리안2A AMI | - | - | 1km |
| MODIS | B1 (0.62-0.67 μm) | B2 (0.84-0.88 μm) | 250m |

### NDVI 계산 프로토콜

```python
def calculate_ndvi(red, nir, quality_mask, params):
    """
    품질 관리와 함께 NDVI 계산.

    매개변수:
    -----------
    red : array
        적색 밴드 표면 반사율
    nir : array
        NIR 밴드 표면 반사율
    quality_mask : array
        구름 마스킹의 품질 플래그
    params : dict
        처리 매개변수

    반환값:
    --------
    dict : NDVI 배열 및 품질 정보
    """
    # 출력 초기화
    ndvi = np.full_like(red, np.nan, dtype=np.float32)
    quality_flags = np.zeros_like(red, dtype=np.uint8)

    # 유효한 픽셀 식별
    valid = (
        (quality_mask == 0) &  # 맑음
        (red > 0) &
        (nir > 0) &
        (red < 1) &
        (nir < 1)
    )

    # 유효한 픽셀에 대해 NDVI 계산
    denominator = nir[valid] + red[valid]
    numerator = nir[valid] - red[valid]

    # 0으로 나누기 방지
    nonzero_denom = denominator > 0.001
    ndvi[valid][nonzero_denom] = numerator[nonzero_denom] / denominator[nonzero_denom]

    # 품질 검사
    # 물리적으로 비현실적인 NDVI 검사
    unrealistic = (ndvi < -0.5) | (ndvi > 1.0)
    quality_flags[unrealistic] |= 0x04
    ndvi[unrealistic] = np.nan

    # 불확실성 계산
    uncertainty = calculate_ndvi_uncertainty(red, nir, params)

    return {
        'ndvi': ndvi,
        'quality_flags': quality_flags,
        'uncertainty': uncertainty,
        'valid_pixel_count': np.sum(valid & ~np.isnan(ndvi)),
        'total_pixel_count': np.size(ndvi)
    }
```

### NDVI 합성 방법

| 방법 | 설명 | 최적 용도 |
|------|------|----------|
| 최대값 합성 (MVC) | 기간 내 가장 높은 NDVI | 구름 효과 최소화 |
| 중앙값 | 기간 내 중간 값 | 이상치 감소 |
| 평균 | 평균 값 | 시간적 평활화 |
| 최상 품질 | 최상의 QA를 가진 픽셀 | 데이터 품질 우선 |

---

## 6.4 증발산 추정 (FAO-56 Penman-Monteith)

### FAO-56 Penman-Monteith 방정식

FAO-56 Penman-Monteith 방정식은 기준 증발산 계산을 위한 표준 방법입니다:

```
            0.408 Δ(Rn - G) + γ (900/(T+273)) u₂ (es - ea)
ET₀ = ─────────────────────────────────────────────────────
                        Δ + γ(1 + 0.34 u₂)

여기서:
- ET₀ = 기준 증발산 (mm/일)
- Rn = 순복사 (MJ/m²/일)
- G = 토양 열속 (MJ/m²/일)
- T = 평균 일 기온 (°C)
- u₂ = 2m 높이에서의 풍속 (m/s)
- es = 포화 수증기압 (kPa)
- ea = 실제 수증기압 (kPa)
- Δ = 포화 수증기압 곡선의 기울기 (kPa/°C)
- γ = 건습구 상수 (kPa/°C)
```

### FAO-56 구현

```python
import numpy as np

def calculate_et0_pm(weather_data, location, date):
    """
    FAO-56 Penman-Monteith를 사용한 기준 ET 계산.

    매개변수:
    -----------
    weather_data : dict
        일별 기상 관측
    location : dict
        위치 매개변수 (위도, 경도, 고도)
    date : datetime
        계산 날짜

    반환값:
    --------
    dict : ET0 및 중간 계산
    """
    # 입력 추출
    T_max = weather_data['temp_max_c']
    T_min = weather_data['temp_min_c']
    T_mean = (T_max + T_min) / 2

    RH_max = weather_data.get('rh_max', 80)
    RH_min = weather_data.get('rh_min', 40)

    u_2 = weather_data.get('wind_speed_2m', 2.0)
    Rs = weather_data.get('solar_radiation_mj', None)

    lat = location['latitude']
    elevation = location['elevation_m']

    # 연중 일수 계산
    doy = date.timetuple().tm_yday

    # 대기압 (kPa)
    P = 101.3 * ((293 - 0.0065 * elevation) / 293) ** 5.26

    # 건습구 상수 (kPa/°C)
    gamma = 0.665e-3 * P

    # 포화 수증기압 (kPa)
    es_max = 0.6108 * np.exp(17.27 * T_max / (T_max + 237.3))
    es_min = 0.6108 * np.exp(17.27 * T_min / (T_min + 237.3))
    es = (es_max + es_min) / 2

    # 실제 수증기압 (kPa)
    ea = (es_min * RH_max / 100 + es_max * RH_min / 100) / 2

    # 포화 수증기압 곡선의 기울기 (kPa/°C)
    delta = 4098 * es / (T_mean + 237.3) ** 2

    # 태양 계산 (생략)...

    # 기준 ET (mm/일)
    numerator = 0.408 * delta * (Rn - G) + gamma * (900 / (T_mean + 273)) * u_2 * (es - ea)
    denominator = delta + gamma * (1 + 0.34 * u_2)
    ET0 = numerator / denominator

    return {
        'et0_mm_day': ET0,
        'vapor_pressure_deficit_kpa': es - ea
    }
```

---

## 6.5 PDSI 계산 알고리즘

### PDSI 알고리즘 개요

Palmer 가뭄 심각도 지수는 물 수지 접근법을 사용합니다:

```
PDSI 알고리즘 흐름:
==================

1단계: 물 수지 계정
├── 잠재 증발산 (PE)
├── 잠재 충전 (PR)
├── 잠재 손실 (PL)
├── 잠재 유출 (PRO)
└── 실제 값 (ET, R, L, RO)

2단계: 계수 계산
├── α = ET/PE (증발 계수)
├── β = R/PR (충전 계수)
├── γ = RO/PRO (유출 계수)
├── δ = L/PL (손실 계수)

3단계: CAFEC 강수량
├── P̂ = αPE + βPR + γPRO - δPL
├── d = P - P̂ (수분 편차)
├── D = d/average(|d|) (수분 이상 지수)

4단계: PDSI 계산
├── Z = K × d (Z-지수)
├── PDSI = 0.897 × PDSI_prev + Z/3
```

### 자체 보정 PDSI (SC-PDSI)

자체 보정 버전은 PDSI의 지리적 불일치를 해결합니다:

```python
def calculate_sc_pdsi(precip, pet, awc, calibration_period):
    """
    자체 보정 Palmer 가뭄 심각도 지수 계산.

    매개변수:
    -----------
    precip : array
        월간 강수량 (mm)
    pet : array
        월간 잠재 증발산 (mm)
    awc : float
        토양의 유효 저수 용량 (mm)
    calibration_period : tuple
        보정을 위한 (시작_연도, 종료_연도)

    반환값:
    --------
    dict : SC-PDSI 값 및 구성요소
    """
    n_months = len(precip)

    # 물 수지 구성요소 초기화
    soil_moisture = np.zeros(n_months)
    soil_moisture[0] = awc  # 포장용수량에서 시작

    # 1단계: 물 수지 계정
    # ... (구현 세부사항)

    # 2단계: 각 월에 대한 계수 계산
    # ... (구현 세부사항)

    # 3단계: CAFEC 강수량 및 편차 계산
    # ... (구현 세부사항)

    # 4단계: K 인자의 자체 보정
    k = calculate_self_calibrated_k(d[cal_mask])

    # Z-지수 계산
    z = k * d

    # 5단계: Palmer의 재귀 공식으로 PDSI 계산
    pdsi = np.zeros(n_months)
    for i in range(1, n_months):
        pdsi[i] = 0.897 * pdsi[i-1] + z[i] / 3

    return {
        'pdsi': pdsi,
        'z_index': z,
        'departures': d,
        'soil_moisture': soil_moisture
    }
```

---

## 6.6 SPI/SPEI 통계적 방법

### SPI 계산 절차

```python
from scipy.stats import gamma as gamma_dist
from scipy.stats import norm

def calculate_spi(precip_series, time_scale_months, baseline_period):
    """
    표준화 강수지수 계산.

    매개변수:
    -----------
    precip_series : array
        월간 강수량 값
    time_scale_months : int
        누적 기간 (1-48개월)
    baseline_period : tuple
        분포 적합을 위한 (시작_연도, 종료_연도)

    반환값:
    --------
    dict : SPI 값 및 매개변수
    """
    # 1단계: 강수량 누적
    accumulated = accumulate_precipitation(precip_series, time_scale_months)

    # 2단계: 각 월에 대해 감마 분포 적합
    gamma_params = {}
    spi = np.zeros_like(accumulated)

    for month in range(12):
        # 기준선 동안 이 월의 값 추출
        month_values = extract_month_values(accumulated, month, baseline_period)

        # 감마 적합을 위해 0 제거
        nonzero_values = month_values[month_values > 0]
        zero_prob = np.sum(month_values == 0) / len(month_values)

        # 최대 우도를 사용하여 감마 분포 적합
        alpha, loc, beta = gamma_dist.fit(nonzero_values, floc=0)

        gamma_params[month] = {
            'alpha': alpha,
            'beta': beta,
            'zero_probability': zero_prob
        }

        # 이 월의 모든 값에 대해 SPI 계산
        month_mask = np.arange(len(accumulated)) % 12 == month

        for i in np.where(month_mask)[0]:
            if accumulated[i] == 0:
                prob = zero_prob
                spi[i] = norm.ppf(prob)
            else:
                gamma_cdf = gamma_dist.cdf(accumulated[i], alpha, loc=0, scale=beta)
                prob = zero_prob + (1 - zero_prob) * gamma_cdf
                spi[i] = norm.ppf(prob)

    # 극단값 제한
    spi = np.clip(spi, -3.5, 3.5)

    return {
        'spi': spi,
        'accumulated_precip': accumulated,
        'gamma_params': gamma_params,
        'time_scale': time_scale_months
    }
```

---

## 6.7 가뭄 분류 및 심각도 매핑

### 다중 지수 분류

```python
def classify_drought_status(indices, weights=None):
    """
    여러 지수를 가뭄 분류로 결합.

    매개변수:
    -----------
    indices : dict
        지수 이름과 값의 사전
    weights : dict
        각 지수에 대한 선택적 가중치

    반환값:
    --------
    dict : 분류 결과
    """
    if weights is None:
        weights = {
            'pdsi': 0.25,
            'spi_3': 0.20,
            'soil_moisture_pct': 0.25,
            'ndvi_anomaly': 0.15,
            'esi': 0.15
        }

    # 지수를 공통 척도로 정규화 (-4 ~ +4)
    normalized = {}
    # ... (정규화 로직)

    # 가중 복합 계산
    composite = 0
    total_weight = 0

    for idx, value in normalized.items():
        if idx in weights and not np.isnan(value):
            composite += value * weights[idx]
            total_weight += weights[idx]

    if total_weight > 0:
        composite /= total_weight

    # 분류
    if composite >= 0.5:
        category = 'W'
        severity = '습윤'
    elif composite >= -1.0:
        category = 'N'
        severity = '정상'
    elif composite >= -2.0:
        category = 'D0'
        severity = '비정상적_건조'
    elif composite >= -3.0:
        category = 'D1'
        severity = '중간_가뭄'
    elif composite >= -4.0:
        category = 'D2'
        severity = '심한_가뭄'
    elif composite >= -5.0:
        category = 'D3'
        severity = '극심한_가뭄'
    else:
        category = 'D4'
        severity = '예외적_가뭄'

    return {
        'composite_value': composite,
        'category': category,
        'severity': severity,
        'contributing_indices': list(normalized.keys()),
        'confidence': min(total_weight / sum(weights.values()), 1.0)
    }
```

---

## 6.8 품질 관리 및 검증 프로토콜

### 다단계 QC 프레임워크

| 단계 | 검사 | 실패 시 조치 |
|------|------|------------|
| 수준 0 | 형식, 완전성 | 거부 |
| 수준 1 | 물리적 한계 | 플래그 |
| 수준 2 | 공간적 일관성 | 플래그/조사 |
| 수준 3 | 시간적 일관성 | 플래그/보간 |
| 수준 4 | 교차 검증 | 문서화 |

### QC 구현

```python
def quality_control_drought_data(data, qc_config):
    """
    가뭄 데이터에 다단계 품질 관리 적용.

    매개변수:
    -----------
    data : dict
        가뭄 데이터 레코드
    qc_config : dict
        QC 구성 매개변수

    반환값:
    --------
    dict : QC 결과 및 플래그된 데이터
    """
    qc_results = {
        'passed': True,
        'flags': [],
        'level': 4  # 통과한 가장 높은 수준
    }

    # 수준 0: 형식 검증
    if not validate_format(data):
        qc_results['passed'] = False
        qc_results['level'] = 0
        qc_results['flags'].append('INVALID_FORMAT')
        return qc_results

    # 수준 1: 물리적 한계
    value = data.get('value')
    index_type = data.get('index_type', 'pdsi')
    limits = qc_config.get('limits', {}).get(index_type, (-10, 10))

    if value < limits[0] or value > limits[1]:
        qc_results['flags'].append('OUT_OF_RANGE')
        qc_results['level'] = min(qc_results['level'], 1)

    # 수준 2: 공간적 일관성
    neighbors = data.get('neighbor_values', [])
    if len(neighbors) >= 3:
        neighbor_mean = np.mean(neighbors)
        neighbor_std = np.std(neighbors)

        if abs(value - neighbor_mean) > 3 * neighbor_std:
            qc_results['flags'].append('SPATIAL_OUTLIER')
            qc_results['level'] = min(qc_results['level'], 2)

    # 수준 3: 시간적 일관성
    previous_value = data.get('previous_value')
    if previous_value is not None:
        max_change = qc_config.get('max_monthly_change', {}).get(index_type, 2.0)

        if abs(value - previous_value) > max_change:
            qc_results['flags'].append('TEMPORAL_JUMP')
            qc_results['level'] = min(qc_results['level'], 3)

    qc_results['passed'] = len(qc_results['flags']) == 0

    return qc_results
```

---

## 6.9 복습 문제 및 핵심 요점

### 복습 문제

1. **대기 보정**: 시간적 NDVI 분석에 대기 보정이 왜 필수적인지 설명하세요. 다른 날짜의 미보정 NDVI 값을 비교하면 어떻게 됩니까?

2. **구름 마스킹 트레이드오프**: Fmask 알고리즘은 조정 가능한 확률 임계값을 가지고 있습니다. 낮은 임계값(0.1)과 높은 임계값(0.5) 사용 간의 트레이드오프를 설명하세요. 각각 언제 적합합니까?

3. **NDVI 합성**: 16일 NDVI 합성물이 최대값 합성(MVC)을 사용합니다. 이 접근법의 장점과 잠재적 편향은 무엇입니까?

4. **ET₀ 추정**: FAO-56 Penman-Monteith 방정식은 많은 입력이 필요합니다. 온도 데이터만 있다면 ET₀를 어떻게 추정하시겠습니까?

5. **PDSI 한계**: 원래 PDSI는 공간적 불일치로 비판받았습니다. 자체 보정 PDSI(SC-PDSI)가 이 한계를 어떻게 해결하는지 설명하세요.

6. **SPI 시간 척도**: 한 위치가 SPI-1 = +0.5(거의 정상)이지만 SPI-12 = -2.0(극심하게 건조)을 보여줍니다. 가뭄 조건 측면에서 이 패턴을 해석하세요.

7. **다중 지수 분류**: 관개 농업 지역에서 가뭄 분류를 위한 가중 체계를 설계하세요. 천수답 지역과 가중치가 어떻게 다를까요?

8. **품질 관리**: 토양 수분 센서가 인접 센서보다 3 표준편차 떨어진 값을 보고합니다. 따라야 할 QC 조사 과정을 설명하세요.

### 핵심 요점

1. **대기 보정은 근본적**: 6S 모델은 표면 반사율에서 20-40% 변동을 유발할 수 있는 대기 효과를 제거하여 유효한 시간적 비교를 가능하게 합니다.

2. **구름 마스킹은 다단계**: Fmask는 스펙트럼 테스트, 온도 확률 및 그림자 투영을 결합하여 견고한 구름 감지를 달성합니다.

3. **NDVI에는 알려진 한계가 있음**: 센서 차이, 대기 효과 및 밀집 식생에서의 포화는 신중한 보정과 품질 관리가 필요합니다.

4. **FAO-56은 ET₀ 표준**: Penman-Monteith 방정식은 물리적 기반의 기준 ET를 제공하여 증발 수요를 통한 가뭄 평가를 가능하게 합니다.

5. **PDSI는 물 수지 모델링이 필요**: Palmer 지수는 2계층 버킷 모델을 통해 강수량, 온도 및 토양 특성을 통합합니다.

6. **SPI/SPEI는 통계적으로 견고**: 강수량 또는 물 수지를 확률 분포에 적합하면 위치와 시간에 걸쳐 표준화된 비교가 가능합니다.

7. **다중 지수 통합이 평가를 개선**: 기상학적, 농업적 및 수문학적 지수를 결합하면 더 완전한 가뭄 특성화를 제공합니다.

8. **품질 관리는 다단계**: 형식 검증에서 교차 검증까지 점진적 QC는 불확실성을 문서화하면서 데이터 품질을 보장합니다.

9. **알고리즘에는 매개변수가 있음**: 각 알고리즘은 지역 조건과 응용 요구사항에 따라 조정해야 할 조정 가능한 매개변수를 포함합니다.

10. **검증은 필수**: 모든 알고리즘은 정확성을 보장하고 한계를 문서화하기 위해 독립적인 데이터에 대해 검증되어야 합니다.

---

## 장 요약

이 장은 가뭄 모니터링의 기반이 되는 과학적 프로토콜과 알고리즘—위성 데이터 처리에서 지수 계산, 품질 관리까지—을 상세히 설명했습니다. 이러한 방법은 원시 관측을 실행 가능한 가뭄 정보로 변환합니다.

6S 모델을 사용한 대기 보정은 대기 간섭을 제거하여 유효한 다중 시간 분석을 가능하게 합니다. Fmask를 사용한 구름 마스킹은 가뭄 평가를 손상시킬 수 있는 오염된 픽셀을 식별합니다. 이러한 전처리 단계는 함께 위성 데이터를 식생 분석을 위해 준비합니다.

NDVI 계산은 확립된 공식을 따르지만 센서 차이, 포화 효과 및 품질 플래깅에 신중한 주의가 필요합니다. 합성 방법은 구름 효과를 최소화하면서 다중 시간 관측을 집계합니다.

FAO-56 Penman-Monteith를 사용한 증발산 추정은 실제 물 사용이 비교되는 기준을 제공합니다. ET 분율(ETa/ET₀)은 증발 수요와 무관하게 작물 물 스트레스를 나타냅니다.

PDSI와 SPI/SPEI 계산은 물 수지 모델링과 통계적 변환을 통해 표준화된 가뭄 지수를 구현합니다. 자체 보정 PDSI는 공간적 불일치를 해결하고, 다중 척도 SPI는 다양한 시간 지평에서 가뭄을 포착합니다.

품질 관리는 형식 검사에서 독립적인 관측과의 교차 검증까지 다단계 검증을 통해 데이터 신뢰성을 보장합니다. 품질 플래그는 데이터 한계를 사용자에게 전달합니다.

이러한 프로토콜은 가뭄 모니터링 시스템의 과학적 기반을 제공합니다. 기관 간 일관된 구현은 상호 운용 가능한 데이터 교환과 유효한 국경 간 가뭄 평가를 가능하게 합니다.

---

**다음 장: [제7장: 시스템 통합](07-system-integration.md)**
