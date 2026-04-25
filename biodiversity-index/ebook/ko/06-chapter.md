# 제6장: 통합 시스템

## 4단계: GIS, 보전 계획 및 정책 보고

### 생물다양성 데이터를 의사결정 시스템에 연결

---

## 개요

WIA 생물다양성 지수 표준의 4단계는 표준화된 생물다양성 데이터가 의사결정 시스템으로 원활하게 흐르도록 보장합니다. 이 장에서는 GIS 플랫폼, 보전 계획 도구, 정책 보고 프레임워크 및 환경 관리 시스템과의 통합을 다룹니다.

---

## GIS 플랫폼 통합

### ArcGIS 통합

WIA ArcGIS 도구 상자는 Esri 생태계 내에서 생물다양성 분석을 위한 네이티브 도구를 제공합니다.

**시스템 요구사항:**
- ArcGIS Pro 2.9+ 또는 ArcMap 10.8+
- arcpy가 포함된 Python 3.7+
- WIA 생물다양성 SDK 설치
- 유효한 WIA API 자격 증명

**제공 도구:**

| 도구 | 목적 | 입력 | 출력 |
|------|------|------|------|
| 생물다양성 핫스팟 | 다양성 핫스팟 계산 및 매핑 | 지역 폴리곤, 지수 유형 | 핫스팟 피처 클래스 |
| 종 분포 | 종 분포 지도 생성 | 종명, 출현 | 확률 표면 래스터 |
| 시간적 추세 | 다양성 추세 시각화 | 데이터셋, 시간 범위 | 차트 및 추세 레이어 |
| 보호지역 평가 | PA 내/외 다양성 비교 | PA 경계, 출현 | 평가 보고서 |
| 서식지 적합성 | 종 서식지 선호도 모델링 | 환경 레이어, 출현 | 적합성 래스터 |

**예제: 생물다양성 핫스팟 분석**

```python
import arcpy
from wia_biodiversity_arcgis import BiodiversityTools

# 도구 상자 초기화
tools = BiodiversityTools(api_key='your_api_key')

# 분석 매개변수 정의
params = {
    'region': 'Korean_Peninsula',
    'index_type': 'Shannon',
    'cell_size_km': 10,
    'threshold_percentile': 90,
    'output_fc': r'C:\GIS\Projects\Korea\hotspots.gdb\biodiversity_hotspots'
}

# 분석 실행
result = tools.identify_hotspots(**params)

print(f"식별된 핫스팟: {result.hotspot_count}개")
print(f"총 면적: {result.total_area_km2:,.0f} km²")
```

### QGIS 플러그인

WIA 생물다양성 브라우저 플러그인은 QGIS 사용자에게 WIA 데이터 및 분석 도구에 대한 직접 접근을 제공합니다.

**설치:**
1. QGIS 플러그인 관리자 열기
2. 저장소 추가: `https://plugins.qgis.org/plugins/wia-biodiversity`
3. "WIA Biodiversity Browser" 검색
4. 설치 및 QGIS 재시작

**기능:**
- QGIS 인터페이스에서 직접 WIA API 쿼리
- 출현을 벡터 레이어로 로드
- 선택된 폴리곤에 대한 다양성 지수 계산
- 표준 형식으로 내보내기 (GeoPackage, Shapefile, GeoJSON)

**Python API:**

```python
from qgis.core import QgsVectorLayer, QgsProject
from wia_qgis import WIABiodiversity

# 연결 초기화
wia = WIABiodiversity(api_key='your_api_key')

# 출현을 레이어로 로드
layer = wia.load_occurrences(
    species='Naemorhedus caudatus',
    year=2025,
    country='KR',
    layer_name='산양 출현 2025'
)

# 프로젝트에 추가
QgsProject.instance().addMapLayer(layer)
```

---

## 보전 계획 통합

### Marxan 통합

Marxan은 체계적 보전 계획을 위한 선도적인 소프트웨어입니다. WIA 데이터는 보전 목표를 정의하기 위해 원활하게 통합됩니다.

**WIA-to-Marxan 변환기:**

```python
from wia_biodiversity import BiodiversityAPI, MarxanConverter

# 초기화
client = BiodiversityAPI(api_key='your_api_key')
converter = MarxanConverter()

# 계획 지역 정의
region = client.spatial.define_region(
    polygon=[[127.5, 35.5], [128.5, 35.5], [128.5, 36.5], [127.5, 36.5]],
    planning_unit_size_km2=100
)

# WIA에서 종 데이터 로드
species_data = client.species.list(
    region=region,
    iucn_status=['CR', 'EN', 'VU'],
    min_occurrences=5
)

# Marxan 입력 파일 생성
converter.generate_marxan_files(
    region=region,
    species=species_data,
    output_dir='marxan_inputs/',
    species_target=0.30,  # 30% 대표성 목표
    cost_metric='area'
)
```

### Zonation 통합

Zonation은 WIA 종 분포 데이터를 사용하여 공간 보전 우선순위화를 수행합니다.

```python
from wia_biodiversity import ZonationConverter

converter = ZonationConverter()

# 보전 가중치가 있는 종 목록 정의
species_list = [
    {'name': 'Naemorhedus caudatus', 'weight': 1.0, 'iucn': 'VU'},
    {'name': 'Panthera tigris altaica', 'weight': 1.2, 'iucn': 'EN'},
    {'name': 'Moschus moschiferus', 'weight': 0.9, 'iucn': 'VU'}
]

# 출현 데이터에서 종 분포 래스터 생성
converter.create_species_layers(
    species_list=species_list,
    resolution_m=1000,
    output_dir='zonation_inputs/',
    format='geotiff'
)
```

---

## 정책 보고 통합

### CBD (생물다양성협약)

**자동화된 쿤밍-몬트리올 프레임워크 보고:**

```python
from wia_biodiversity import ReportingFramework

reporter = ReportingFramework(api_key='your_api_key')

# 목표 3 보고서 생성 (30x30 보호지역)
target_3 = reporter.cbd.target_3_report(
    country='KR',
    reference_year=2022,
    target_year=2030
)

print(f"현재 육상 PA 커버리지: {target_3.terrestrial_coverage:.1f}%")
print(f"현재 해양 PA 커버리지: {target_3.marine_coverage:.1f}%")
print(f"30% 육상까지 격차: {target_3.terrestrial_gap:.1f}%")

# 목표 4 보고서 생성 (종 회복)
target_4 = reporter.cbd.target_4_report(
    country='KR',
    species_group='all_assessed'
)

# 공식 PDF로 내보내기
reporter.export(
    [target_3, target_4],
    format='pdf',
    template='cbd_official',
    output_file='korea_cbd_report_2025.pdf'
)
```

**사용 가능한 CBD 목표:**

| 목표 | 설명 | WIA 지표 |
|------|------|----------|
| 목표 3 | 2030년까지 30% 보호 | PA 커버리지, PA 내 종 |
| 목표 4 | 종 회복 조치 | 위협 종 추세 |
| 목표 6 | 침입종 감소 | 침입 탐지, 확산 |
| 목표 7 | 오염 감소 | 수질 지수 |
| 목표 8 | 기후 적응 | 분포 이동 감지 |

### IPBES 평가 지원

생물다양성 및 생태계 서비스에 관한 정부간 과학-정책 플랫폼 평가를 위한 데이터 생성.

```python
# 자연이 인간에게 주는 기여 (NCP) 평가
ncp_report = reporter.ipbes.ncp_assessment(
    region='East_Asia',
    ncp_categories=[
        'food_feed',
        'pollination_seed_dispersal',
        'regulation_climate',
        'regulation_water'
    ]
)

# 변화 동인 분석
drivers = reporter.ipbes.drivers_analysis(
    region='Korean_Peninsula',
    time_period=('2000-01-01', '2025-12-31'),
    drivers=['land_use_change', 'climate_change', 'overexploitation']
)
```

### 국가 보고

**자동화된 국가 생물다양성 보고서:**

```python
# 종합 국가 보고서 생성
national_report = reporter.generate_national_report(
    country='KR',
    year=2025,
    sections=[
        'executive_summary',
        'state_and_trends',
        'threats_and_pressures',
        'conservation_actions',
        'policy_integration'
    ],
    language='ko',
    include_maps=True,
    include_charts=True
)

# 내보내기 옵션
national_report.export_pdf('korea_nbsap_2025.pdf')
```

---

## 환경영향평가

### 기준선 평가 모듈

```python
from wia_biodiversity import EIAModule

eia = EIAModule(api_key='your_api_key')

# 프로젝트 지역 및 영향 구역 정의
project_footprint = {
    "type": "Polygon",
    "coordinates": [[[lon, lat] for lon, lat in project_boundary]]
}

impact_zones = {
    "direct_impact": project_footprint,
    "indirect_500m": eia.spatial.buffer(project_footprint, 500),
    "regional_5km": eia.spatial.buffer(project_footprint, 5000)
}

# 기준선 평가 실행
baseline = eia.baseline_assessment(
    zones=impact_zones,
    time_period=('2020-01-01', '2025-12-31'),
    taxa=['birds', 'mammals', 'amphibians', 'reptiles', 'plants']
)

# 기준선 보고서 내용
print(f"문서화된 종: {baseline.total_species}")
print(f"위협 종: {baseline.threatened_species}")
print(f"고유 종: {baseline.endemic_species}")
```

### 영향 예측

```python
# 프로젝트 영향 정의
project_impacts = {
    'habitat_loss': {
        'temperate_forest': 35,  # 손실 헥타르
        'wetland': 12,
        'grassland': 8
    },
    'noise_disturbance_radius_m': 800,
    'lighting_disturbance': True
}

# 생물다양성 영향 예측
impact_prediction = eia.predict_impacts(
    baseline=baseline,
    impacts=project_impacts
)

# 결과
print(f"지역 멸종 위험 종: {impact_prediction.local_extinction_risk}")
print(f"서식지 손실 중요도: {impact_prediction.habitat_significance}")
```

---

## 실시간 경고 시스템

### 멸종위기종 탐지 경고

```python
from wia_biodiversity import AlertSystem

alerts = AlertSystem(api_key='your_api_key')

# 위급종 탐지 구독
subscription = alerts.subscribe(
    event_type='species_detection',
    filters={
        'iucn_status': ['CR', 'EN'],
        'regions': ['Korean_Peninsula'],
        'detection_methods': ['camera_trap', 'edna', 'acoustic']
    },
    notification_channels=['email', 'webhook', 'sms'],
    webhook_url='https://myapp.org/biodiversity-alerts'
)
```

### 임계값 모니터링

```python
# 다양성 임계값 모니터링 설정
threshold_monitor = alerts.threshold_monitor(
    metric='shannon_diversity',
    region='jirisan_national_park',
    calculation_frequency='monthly',
    threshold_type='percent_change',
    threshold_value=-10,  # 10% 감소 시 경고
    consecutive_violations=2
)
```

---

## 대시보드 통합

### 보호지역 대시보드

**구성요소:**
- 종 풍부도 히트맵
- 시간적 추세 차트 (다양성 지수)
- 위협 종 추적기
- 서식지 상태 오버레이 (위성 + 현장 검증)
- 관리 효과성 지표

**기술 스택:**
- 프론트엔드: React + Mapbox GL JS
- 백엔드: WIA API + PostgreSQL/PostGIS
- 업데이트: WebSocket을 통한 실시간

### 국가 생물다양성 대시보드

**주요 시각화:**
- CBD/쿤밍 목표 진행 지표
- 적색목록 지수 추세선
- 보호지역 내 생태계 커버리지
- 데이터 격차 지도 (과소 샘플링된 지역/분류군)
- 보전 자금 대 결과 상관관계

---

## 핵심 내용

1. **GIS 통합**이 ArcGIS 도구 상자, QGIS 플러그인 및 Google Earth Engine을 통해 가능
2. **보전 계획**이 Marxan, Zonation 및 Circuitscape 연결성 분석과 함께
3. **정책 보고** 자동화가 CBD, IPBES 및 국가 생물다양성 전략에 대해
4. **EIA 지원**이 기준선 평가, 영향 예측 및 완화 포함
5. **실시간 경고**가 멸종위기종 탐지 및 다양성 임계값 위반에 대해

## 복습 문제

1. WIA ArcGIS 도구 상자에서 사용 가능한 도구는 무엇입니까?
2. WIA 데이터는 보전 계획을 위해 Marxan과 어떻게 통합됩니까?
3. WIA 프레임워크를 사용하여 자동으로 보고할 수 있는 CBD 목표는 무엇입니까?
4. WIA 지원 EIA 기준선 평가의 구성요소를 설명하세요.
5. 멸종위기종 탐지를 위한 실시간 경고를 어떻게 구성할 수 있습니까?

---

**다음 장 미리보기:** 7장에서는 Shannon, Simpson 및 풍부도 추정기를 포함한 다양성 지수의 수학적 기초를 탐구합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 · 모든 생명을 보존
