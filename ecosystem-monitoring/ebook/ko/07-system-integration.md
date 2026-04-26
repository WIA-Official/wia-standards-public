# 제7장: 시스템 통합

## 학습 목표

이 장을 마친 후 다음을 할 수 있게 됩니다:

1. GIS 플랫폼(ArcGIS, QGIS)과 WIA 데이터 통합
2. 글로벌 생물다양성 데이터베이스(GBIF, iNaturalist)에 연결
3. 클라우드 플랫폼(AWS, Azure, Google Cloud)에 WIA 시스템 배포
4. 의사결정 지원 도구(Marxan, InVEST)와 연결
5. 과학적 인용을 위한 DOI로 데이터셋 게시

---

## 7.1 GIS 플랫폼 통합

### 7.1.1 QGIS 통합

**WIA QGIS 플러그인:**

```python
# WIA 플러그인 설치
# QGIS 플러그인 > 플러그인 관리 및 설치 > "WIA Ecosystem" 검색

# WIA 데이터 레이어 로드
from qgis.core import QgsVectorLayer

# WIA API에 연결
wia_layer = QgsVectorLayer(
    "url='https://api.example.org/v1/observations?format=geojson&bbox=-125,40,-110,50' \
     type=geojson",
    "독수리 관찰",
    "ogr"
)

QgsProject.instance().addMapLayer(wia_layer)

# 검증 상태로 스타일링
from qgis.core import QgsSymbol, QgsCategorizedSymbolRenderer

categories = []
colors = {
    'expert_verified': '#2ecc71',  # 녹색
    'validated': '#3498db',         # 파랑
    'questionable': '#f39c12',      # 주황
    'invalid': '#e74c3c'            # 빨강
}

for status, color in colors.items():
    symbol = QgsSymbol.defaultSymbol(wia_layer.geometryType())
    symbol.setColor(QColor(color))
    category = QgsRendererCategory(status, symbol, status.replace('_', ' ').title())
    categories.append(category)

renderer = QgsCategorizedSymbolRenderer('validation_status', categories)
wia_layer.setRenderer(renderer)
wia_layer.triggerRepaint()
```

**처리 알고리즘:**

```python
# 시간적 밀도 히트맵
processing.run("wia:temporal_density", {
    'INPUT': wia_layer,
    'TIME_FIELD': 'timestamp',
    'INTERVAL': '1 month',
    'OUTPUT': 'memory:density'
})

# 종 풍부도 그리드
processing.run("wia:species_richness", {
    'INPUT': wia_layer,
    'TAXON_FIELD': 'scientific_name',
    'CELL_SIZE': 1000,  # 미터
    'OUTPUT': 'richness.gpkg'
})
```

### 7.1.2 ArcGIS 통합

**ArcGIS REST Feature Service:**

```javascript
// 웹 맵에 WIA 레이어 추가
require([
  "esri/Map",
  "esri/views/MapView",
  "esri/layers/GeoJSONLayer"
], function(Map, MapView, GeoJSONLayer) {

  const wiaLayer = new GeoJSONLayer({
    url: "https://api.example.org/v1/observations?format=geojson",
    copyright: "WIA 생태계 모니터링",
    popupTemplate: {
      title: "{scientific_name}",
      content: `
        <b>일반 이름:</b> {common_name}<br>
        <b>날짜:</b> {timestamp}<br>
        <b>관찰자:</b> {observer.name}<br>
        <b>풍부도:</b> {abundance}<br>
        <b>상태:</b> {validation_status}
      `
    },
    renderer: {
      type: "simple",
      symbol: {
        type: "simple-marker",
        size: 8,
        color: "blue",
        outline: { color: "white", width: 1 }
      }
    }
  });

  const map = new Map({
    basemap: "topo-vector",
    layers: [wiaLayer]
  });

  const view = new MapView({
    container: "viewDiv",
    map: map,
    center: [-122.4, 47.6],
    zoom: 10
  });
});
```

**ArcGIS Field Maps (모바일 데이터 수집):**

```json
{
  "name": "종 관찰",
  "fields": [
    {
      "name": "scientific_name",
      "alias": "학명",
      "type": "esriFieldTypeString",
      "domain": {
        "type": "codedValue",
        "name": "CommonSpecies",
        "codedValues": [
          {"name": "Haliaeetus leucocephalus", "code": "HALL"},
          {"name": "Ursus arctos", "code": "URSAR"}
        ]
      }
    },
    {
      "name": "abundance",
      "alias": "개체수",
      "type": "esriFieldTypeInteger"
    },
    {
      "name": "photo",
      "alias": "사진",
      "type": "esriFieldTypeBlob"
    }
  ]
}
```

### 7.1.3 OGC 웹 서비스

**Web Feature Service (WFS):**

```xml
<!-- GetCapabilities 요청 -->
GET https://api.example.org/ogc/wfs?
  service=WFS&
  version=2.0.0&
  request=GetCapabilities

<!-- GetFeature 요청 -->
GET https://api.example.org/ogc/wfs?
  service=WFS&
  version=2.0.0&
  request=GetFeature&
  typeName=wia:observations&
  bbox=-125,40,-110,50&
  outputFormat=application/json
```

**Web Map Service (WMS):**

```xml
<!-- 시각화를 위한 GetMap 요청 -->
GET https://api.example.org/ogc/wms?
  service=WMS&
  version=1.3.0&
  request=GetMap&
  layers=wia:species_density&
  bbox=-125,40,-110,50&
  width=800&
  height=600&
  crs=EPSG:4326&
  format=image/png
```

---

## 7.2 보존 데이터베이스 통합

### 7.2.1 GBIF 통합

**Darwin Core Archive로 내보내기:**

```python
from wia import Client
from dwca import DarwinCoreArchive

client = Client(api_key='YOUR_KEY')
observations = client.get_observations(
    start_date='2025-01-01',
    end_date='2025-12-31'
)

# WIA를 Darwin Core로 매핑
dwc_records = []
for obs in observations:
    dwc_records.append({
        'occurrenceID': obs['record_id'],
        'basisOfRecord': 'HumanObservation',
        'eventDate': obs['timestamp'],
        'decimalLatitude': obs['location']['latitude'],
        'decimalLongitude': obs['location']['longitude'],
        'scientificName': obs['taxon']['scientific_name'],
        'kingdom': obs['taxon']['kingdom'],
        'phylum': obs['taxon']['phylum'],
        'class': obs['taxon']['class'],
        'order': obs['taxon']['order'],
        'family': obs['taxon']['family'],
        'genus': obs['taxon']['genus'],
        'specificEpithet': obs['taxon']['species'],
        'individualCount': obs['abundance'],
        'samplingProtocol': obs['detection_method'],
        'recordedBy': obs['observer']['name'],
        'identificationQualifier': obs['quality']['validation_status'],
        'coordinateUncertaintyInMeters': obs['location'].get('precision'),
    })

# Darwin Core Archive 생성
archive = DarwinCoreArchive()
archive.add_occurrence_core(dwc_records)
archive.add_eml_metadata({
    'title': 'Pacific Northwest Bird Observations 2025',
    'abstract': '...',
    'creators': [{'name': 'Jane Smith', 'email': 'jane@example.org'}]
})
archive.write('gbif_export.zip')

# GBIF IPT에 업로드 (Integrated Publishing Toolkit)
# https://www.gbif.org/ipt
```

### 7.2.2 iNaturalist 통합

**양방향 동기화:**

```python
import inaturalist

# iNaturalist에서 연구 등급 관찰 가져오기
observations = inaturalist.get_observations(
    taxon_id=3,  # 조류
    quality_grade='research',
    geo=True,
    bbox=(-125, 40, -110, 50),
    d1='2025-01-01',
    d2='2025-12-31'
)

# WIA 형식으로 변환
wia_observations = []
for obs in observations:
    wia_obs = {
        'wia_version': '1.0',
        'schema_type': 'species-observation',
        'record_id': f'inat-{obs["id"]}',
        'timestamp': obs['observed_on'],
        'location': {
            'latitude': obs['latitude'],
            'longitude': obs['longitude'],
            'precision': obs['positional_accuracy']
        },
        'taxon': {
            'scientific_name': obs['taxon']['name'],
            'common_name': obs['taxon']['preferred_common_name'],
            'taxon_authority': 'iNaturalist Taxonomy'
        },
        'detection_method': 'visual_survey',
        'occurrence_status': 'present',
        'observer': {
            'id': f'inat-user-{obs["user"]["id"]}',
            'name': obs['user']['name']
        },
        'quality': {
            'validation_status': 'expert_verified',
            'confidence_level': 1.0 if obs['quality_grade'] == 'research' else 0.7
        }
    }
    wia_observations.append(wia_obs)

# WIA 데이터베이스에 제출
wia_client.submit_observations(wia_observations)

# WIA 관찰을 iNaturalist로 내보내기
# (API 쓰기 액세스 및 사용자 동의 필요)
for wia_obs in my_wia_observations:
    inaturalist.create_observation({
        'species_guess': wia_obs['taxon']['scientific_name'],
        'observed_on_string': wia_obs['timestamp'],
        'latitude': wia_obs['location']['latitude'],
        'longitude': wia_obs['location']['longitude'],
        'description': f"WIA 모니터링 프로그램에서 가져옴"
    })
```

### 7.2.3 eBird 통합

**eBird 체크리스트 가져오기:**

```r
library(auk)
library(wiaR)

# 지역에 대한 eBird 데이터 다운로드
ebd <- auk_ebd("ebd_US-WA_202501_202512_rel2025.txt") %>%
  auk_bbox(c(-125, 40, -110, 50)) %>%
  auk_complete() %>%  # 완전한 체크리스트만
  auk_filter("ebd_filtered.txt")

# 필터링된 데이터 읽기
observations <- read_ebd("ebd_filtered.txt")

# WIA 형식으로 변환
wia_obs <- observations %>%
  mutate(
    wia_version = "1.0",
    schema_type = "species-observation",
    record_id = paste0("ebird-", checklist_id, "-", species_code),
    timestamp = observation_date,
    latitude = latitude,
    longitude = longitude,
    scientific_name = scientific_name,
    common_name = common_name,
    abundance = observation_count,
    detection_method = "visual_survey",
    protocol = protocol_type,
    effort_duration = duration_minutes,
    effort_distance = effort_distance_km
  )

# WIA 시스템에 업로드
wia_client <- wia_connect(api_key = Sys.getenv("WIA_API_KEY"))
wia_submit_observations(wia_client, wia_obs)
```

---

## 7.3 클라우드 플랫폼 통합

### 7.3.1 AWS 배포

**아키텍처:**

```
AWS WIA 생태계 모니터링 스택:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  ┌──────────────┐         ┌──────────────┐         ┌─────────────┐ │
│  │ IoT Core     │────────►│ Lambda       │────────►│ S3          │ │
│  │ (MQTT)       │         │ (검증)       │         │ (원시 데이터)│ │
│  └──────────────┘         └──────────────┘         └─────────────┘ │
│         │                         │                                │
│         │                         ▼                                │
│         │                 ┌──────────────┐                         │
│         │                 │ Aurora       │                         │
│         │                 │ PostgreSQL   │                         │
│         │                 │ + PostGIS    │                         │
│         │                 └──────────────┘                         │
│         │                         │                                │
│         ▼                         ▼                                │
│  ┌──────────────┐         ┌──────────────┐                         │
│  │ SageMaker    │         │ API Gateway  │                         │
│  │ (종 ID)      │         │ (REST API)   │                         │
│  └──────────────┘         └──────────────┘                         │
│                                   │                                │
│                                   ▼                                │
│                           ┌──────────────┐                         │
│                           │ CloudFront   │                         │
│                           │ (CDN)        │                         │
│                           └──────────────┘                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Terraform 배포:**

```hcl
# PostGIS가 있는 RDS PostgreSQL
resource "aws_db_instance" "wia_db" {
  identifier        = "wia-ecosystem-db"
  engine            = "postgres"
  engine_version    = "15.3"
  instance_class    = "db.t3.medium"
  allocated_storage = 100

  db_name  = "ecosystem"
  username = var.db_username
  password = var.db_password

  vpc_security_group_ids = [aws_security_group.db_sg.id]
  db_subnet_group_name   = aws_db_subnet_group.main.name

  backup_retention_period = 7
  multi_az               = true

  tags = {
    Name = "WIA 생태계 모니터링 DB"
  }
}

# 데이터 검증을 위한 Lambda
resource "aws_lambda_function" "validate_observation" {
  filename      = "validate.zip"
  function_name = "wia_validate_observation"
  role          = aws_iam_role.lambda_exec.arn
  handler       = "index.handler"
  runtime       = "python3.11"

  environment {
    variables = {
      DB_HOST = aws_db_instance.wia_db.endpoint
      DB_NAME = "ecosystem"
    }
  }
}

# API Gateway
resource "aws_apigatewayv2_api" "wia_api" {
  name          = "wia-ecosystem-api"
  protocol_type = "HTTP"

  cors_configuration {
    allow_origins = ["*"]
    allow_methods = ["GET", "POST", "PUT", "DELETE"]
    allow_headers = ["*"]
  }
}
```

### 7.3.2 Google Earth Engine 통합

**관찰 지점에서 원격 감지 추출:**

```javascript
// WIA 관찰을 FeatureCollection으로 로드
var observations = ee.FeatureCollection('projects/wia/eagle_observations');

// Landsat 이미지 로드
var landsat = ee.ImageCollection('LANDSAT/LC08/C02/T1_L2')
  .filterBounds(observations)
  .filterDate('2025-01-01', '2025-12-31');

// NDVI 계산
var addNDVI = function(image) {
  var ndvi = image.normalizedDifference(['SR_B5', 'SR_B4']).rename('NDVI');
  return image.addBands(ndvi);
};

var withNDVI = landsat.map(addNDVI);

// 각 관찰 지점에서 NDVI 추출
var extractValues = function(feature) {
  var date = ee.Date(feature.get('timestamp'));

  var image = withNDVI
    .filterDate(date.advance(-7, 'day'), date.advance(7, 'day'))
    .first();

  var ndvi = image.select('NDVI').reduceRegion({
    reducer: ee.Reducer.mean(),
    geometry: feature.geometry(),
    scale: 30
  }).get('NDVI');

  return feature.set('ndvi', ndvi);
};

var enriched = observations.map(extractValues);

// 강화된 데이터셋 내보내기
Export.table.toDrive({
  collection: enriched,
  description: 'eagle_observations_with_ndvi',
  fileFormat: 'CSV'
});
```

---

## 7.4 의사결정 지원 통합

### 7.4.1 Marxan (보존 계획)

**워크플로우:**

```r
library(wiaR)
library(sf)
library(prioritizr)

# 1. WIA 종 관찰 로드
wia_client <- wia_connect(api_key = Sys.getenv("WIA_API_KEY"))
obs <- get_observations(wia_client, bbox = c(-125, 40, -110, 50))
obs_sf <- wia_to_sf(obs)

# 2. 계획 단위 생성 (육각형)
study_area <- st_bbox(obs_sf) %>% st_as_sfc()
pu <- st_make_grid(study_area, cellsize = 0.1, square = FALSE) %>%
  st_sf() %>%
  mutate(id = row_number(), cost = 1)

# 3. 계획 단위당 종 풍부도 계산
species_by_pu <- obs_sf %>%
  st_join(pu) %>%
  group_by(id, scientific_name) %>%
  summarize(presence = 1, .groups = 'drop') %>%
  pivot_wider(names_from = scientific_name, values_from = presence, values_fill = 0)

# 4. 보존 목표 설정 (현재 분포의 20%)
targets <- species_by_pu %>%
  st_drop_geometry() %>%
  summarize(across(-id, sum)) %>%
  pivot_longer(everything()) %>%
  mutate(target = value * 0.2)

# 5. Marxan 스타일 최적화 실행
prob <- problem(pu, species_by_pu %>% select(-id), cost_column = "cost") %>%
  add_min_set_objective() %>%
  add_relative_targets(0.2) %>%
  add_binary_decisions() %>%
  add_default_solver()

solution <- solve(prob)

# 6. 솔루션 맵핑
ggplot(solution) +
  geom_sf(aes(fill = factor(solution_1))) +
  scale_fill_manual(values = c("white", "darkgreen"),
                    labels = c("선택 안 됨", "선택됨")) +
  labs(fill = "보존\n우선순위") +
  theme_minimal()
```

### 7.4.2 InVEST (생태계 서비스)

**서식지 품질 모델 보정:**

```python
from wia import Client
import natcap.invest.habitat_quality
import geopandas as gpd

# 1. WIA 종 관찰 로드
client = Client(api_key='YOUR_KEY')
obs = client.get_observations(
    taxon='Ursus arctos',  # 회색곰
    bbox=(-125, 40, -110, 50)
)
gdf = client.to_geodataframe(obs)

# 2. 종 밀도 래스터 생성
from rasterio import features
from scipy.ndimage import gaussian_filter

# 지점을 밀도 표면으로 변환
density = create_kernel_density(gdf, cell_size=1000, bandwidth=5000)

# 3. WIA 데이터를 검증으로 사용하여 InVEST 서식지 품질 실행
args = {
    'workspace_dir': './invest_output',
    'lulc_cur_path': 'landcover.tif',
    'threats_table_path': 'threats.csv',
    'sensitivity_table_path': 'sensitivity.csv',
    'half_saturation_constant': 0.5,
    'results_suffix': 'grizzly'
}

natcap.invest.habitat_quality.execute(args)

# 4. InVEST 출력을 WIA 관찰과 비교
invest_quality = rasterio.open('./invest_output/quality_grizzly.tif')
predicted_quality = extract_raster_values(invest_quality, gdf)

# 예측된 품질과 관찰된 풍부도 간의 상관관계
correlation = pearsonr(predicted_quality, gdf['abundance'])
print(f"모델 검증: r = {correlation.statistic:.3f}, p = {correlation.pvalue:.4f}")
```

---

## 7.5 데이터 게시 및 인용

### 7.5.1 DOI 할당

**워크플로우:**

```python
import datacite

# 1. 데이터셋 메타데이터 준비
metadata = {
    'identifier': {'identifier': '10.1234/example', 'identifierType': 'DOI'},
    'creators': [
        {
            'name': 'Smith, Jane',
            'nameIdentifiers': [
                {'nameIdentifier': '0000-0002-1825-0097', 'nameIdentifierScheme': 'ORCID'}
            ],
            'affiliation': ['University of Washington']
        }
    ],
    'titles': [{'title': 'Green River Watershed Biodiversity Monitoring 2020-2025'}],
    'publisher': 'WIA 생태계 모니터링 저장소',
    'publicationYear': '2025',
    'resourceType': {'resourceTypeGeneral': 'Dataset'},
    'descriptions': [
        {
            'description': '수생 및 육상 생물다양성의 장기 모니터링...',
            'descriptionType': 'Abstract'
        }
    ],
    'geoLocations': [
        {
            'geoLocationBox': {
                'westBoundLongitude': -122.5,
                'eastBoundLongitude': -121.8,
                'southBoundLatitude': 47.3,
                'northBoundLatitude': 47.8
            }
        }
    ]
}

# 2. DataCite를 통해 DOI 발행
client = datacite.DataCiteClient(username='YOUR_REPO', password='PASSWORD')
doi = client.mint_doi(metadata)

# 3. DOI로 데이터셋 레코드 업데이트
wia_client.update_dataset_metadata(
    dataset_id='green-river-2020-2025',
    doi=doi
)

# 4. 인용 생성
citation = f"""관련 분야 자료. Green River Watershed Biodiversity
Monitoring 2020-2025 [데이터셋]. WIA 생태계 모니터링 저장소.
https://doi.org/{doi}"""
```

### 7.5.2 저장소 통합

**자동화된 예치:**

```python
# DataONE
from d1_client import mnclient

mn_client = mnclient.MemberNodeClient('https://mn.example.org/mn')

# 데이터셋 패키지
science_metadata = generate_eml(wia_dataset)
data_object = create_data_package(
    data_files=['observations.csv', 'sensors.csv'],
    metadata=science_metadata
)

# DataONE 멤버 노드에 업로드
pid = mn_client.create(
    pid='urn:uuid:' + str(uuid.uuid4()),
    obj=data_object,
    sysmeta=system_metadata
)

# Zenodo
from zenodo_client import Zenodo

zenodo = Zenodo(access_token='YOUR_TOKEN')
deposition = zenodo.create_deposition(
    title='Green River Watershed Monitoring',
    creators=[{'name': 'Smith, Jane', 'orcid': '0000-0002-1825-0097'}],
    description='장기 생물다양성 모니터링...',
    upload_type='dataset',
    access_right='open',
    license='cc-by-4.0'
)

# 파일 업로드
zenodo.upload_file(deposition['id'], 'observations.csv')
zenodo.upload_file(deposition['id'], 'metadata.xml')

# 게시
zenodo.publish_deposition(deposition['id'])
```

---

## 7.6 복습 질문

### 질문 1
새 WIA 관찰을 GBIF와 iNaturalist 모두에 자동으로 동기화하는 워크플로우를 설계하시오. 플랫폼 간 분류학적 이름 차이를 어떻게 처리하겠는가?

### 질문 2
5분마다 데이터를 보내는 1000개의 센서가 있는 국가 모니터링 프로그램을 위해 AWS에 WIA 시스템을 배포하고 있습니다. IoT Core, RDS, S3 및 Lambda에 대한 월별 비용을 추정하시오.

### 질문 3
WIA 관찰 데이터를 사용하여 InVEST 서식지 품질 모델을 보정하고 검증하는 방법을 설명하시오. 모델 성능을 평가하기 위해 어떤 메트릭을 사용하겠는가?

### 질문 4
연구자가 10,000개의 조류 관찰 지점에서 ERA5 재분석 데이터로부터 기후 변수를 추출하려고 합니다. 이것을 Google Earth Engine, 로컬 또는 다른 접근방식에서 수행해야 하는가?

### 질문 5
개별 WIA 관찰(데이터셋뿐만 아니라)에 대한 인용 시스템을 설계하시오. 고유하고 영구적인 식별자를 생성하고 적절한 귀속을 장려하는 방법은 무엇인가?

---

## 7.7 주요 요점

| 통합 | 도구 | 목적 |
|-----|-----|-----|
| **GIS** | ArcGIS, QGIS, OGC 서비스 | 공간 분석 및 시각화 |
| **생물다양성** | GBIF, iNaturalist, eBird | 글로벌 데이터 공유 및 발견 |
| **클라우드** | AWS, Azure, GEE | 확장 가능한 인프라 및 분석 |
| **의사결정 지원** | Marxan, InVEST, SMART | 보존 계획 및 관리 |
| **게시** | DataCite, Zenodo, DataONE | 장기 보관 및 인용 |

### 통합 패턴
- **내보내기**: WIA → GeoJSON → GIS 플랫폼
- **동기화**: iNaturalist, eBird와 양방향
- **강화**: 관찰 지점에서 원격 감지 추출
- **검증**: WIA 관찰이 모델 예측 검증
- **게시**: 저장소에 자동 예치

### 다음 장 미리보기

8장은 구현 지침을 제공하며, 배포 로드맵, 인프라 요구사항, 테스트 절차 및 WIA 준수 시스템에 대한 인증 경로를 포함합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
