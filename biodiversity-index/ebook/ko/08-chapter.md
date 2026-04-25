# 제8장: 구현 가이드

## 실제 데이터베이스 설계, SDK 사용 및 시스템 배포

### WIA 준수 생물다양성 데이터 시스템 구축

---

## 개요

이 장에서는 데이터베이스 설계, SDK 통합, 데이터 마이그레이션 및 배포 전략을 포함하여 WIA 생물다양성 지수 표준 준수 시스템을 구현하기 위한 실제 지침을 제공합니다.

---

## 데이터베이스 설계

### PostgreSQL + PostGIS 아키텍처

권장 데이터베이스 아키텍처는 지리공간 기능을 위한 PostGIS 확장과 함께 PostgreSQL을 사용합니다.

**스키마 개요:**

```sql
-- 필수 확장 활성화
CREATE EXTENSION IF NOT EXISTS postgis;
CREATE EXTENSION IF NOT EXISTS pg_trgm;  -- 텍스트 검색용
CREATE EXTENSION IF NOT EXISTS btree_gist;  -- 범위 쿼리용

-- 핵심 테이블
CREATE SCHEMA biodiversity;
SET search_path TO biodiversity, public;
```

### 핵심 테이블

**출현 테이블:**

```sql
CREATE TABLE occurrences (
    occurrence_id VARCHAR(50) PRIMARY KEY,
    species_id INTEGER NOT NULL REFERENCES species(species_id),
    dataset_id INTEGER NOT NULL REFERENCES datasets(dataset_id),

    -- 위치 (PostGIS 기하학)
    location GEOMETRY(Point, 4326) NOT NULL,
    coordinate_uncertainty_m DECIMAL(10, 2),
    elevation_m INTEGER,
    country_code CHAR(2),
    protected_area_id INTEGER REFERENCES protected_areas(pa_id),

    -- 시간
    observation_date TIMESTAMPTZ NOT NULL,
    date_precision VARCHAR(20),

    -- 관찰 세부사항
    individual_count INTEGER DEFAULT 1,
    sex VARCHAR(20),
    life_stage VARCHAR(30),
    behavior TEXT,
    basis_of_record VARCHAR(30) NOT NULL,
    detection_method VARCHAR(30),
    observer_id VARCHAR(50),

    -- 품질
    quality_flag VARCHAR(20) DEFAULT 'unvalidated',
    quality_score DECIMAL(3, 2),

    -- 메타데이터
    created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    raw_data JSONB
);

-- 일반 쿼리용 인덱스
CREATE INDEX idx_occurrences_species ON occurrences(species_id);
CREATE INDEX idx_occurrences_location ON occurrences USING GIST(location);
CREATE INDEX idx_occurrences_date ON occurrences(observation_date);
CREATE INDEX idx_occurrences_country ON occurrences(country_code);
CREATE INDEX idx_occurrences_quality ON occurrences(quality_flag);
```

**종 테이블:**

```sql
CREATE TABLE species (
    species_id SERIAL PRIMARY KEY,
    scientific_name VARCHAR(255) NOT NULL UNIQUE,
    genus VARCHAR(100) NOT NULL,
    species_epithet VARCHAR(100) NOT NULL,

    -- 분류학
    kingdom VARCHAR(50),
    phylum VARCHAR(50),
    class VARCHAR(50),
    "order" VARCHAR(50),
    family VARCHAR(50),

    -- 외부 참조
    gbif_key INTEGER,
    iucn_taxon_id INTEGER,

    -- 보전 상태
    iucn_status VARCHAR(10),
    population_trend VARCHAR(20),

    -- 메타데이터
    common_names JSONB,
    created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
);

-- 인덱스
CREATE INDEX idx_species_name ON species(scientific_name);
CREATE INDEX idx_species_gbif ON species(gbif_key);
CREATE INDEX idx_species_iucn ON species(iucn_status);
```

**다양성 계산 테이블:**

```sql
CREATE TABLE diversity_calculations (
    calculation_id VARCHAR(50) PRIMARY KEY,
    dataset_id INTEGER REFERENCES datasets(dataset_id),

    -- 공간 범위
    spatial_extent GEOMETRY(Polygon, 4326),
    area_km2 DECIMAL(12, 2),

    -- 시간 범위
    start_date DATE,
    end_date DATE,

    -- 입력 요약
    occurrence_count INTEGER,
    species_count INTEGER,

    -- 계산된 지수
    species_richness INTEGER,
    rarefied_richness DECIMAL(8, 2),
    shannon_index DECIMAL(6, 4),
    simpson_d DECIMAL(6, 5),
    chao1_estimate DECIMAL(8, 2),

    -- 신뢰구간 (JSONB로 저장)
    confidence_intervals JSONB,

    -- 계산 메타데이터
    calculation_date TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    calculation_time_ms INTEGER
);
```

### 최적화된 쿼리

**출현에 대한 공간 쿼리:**

```sql
CREATE OR REPLACE FUNCTION get_occurrences_in_polygon(
    polygon_wkt TEXT,
    species_filter INTEGER[] DEFAULT NULL,
    start_date DATE DEFAULT NULL,
    end_date DATE DEFAULT NULL
)
RETURNS TABLE (
    occurrence_id VARCHAR,
    scientific_name VARCHAR,
    latitude DOUBLE PRECISION,
    longitude DOUBLE PRECISION,
    observation_date TIMESTAMPTZ
) AS $$
BEGIN
    RETURN QUERY
    SELECT
        o.occurrence_id,
        s.scientific_name,
        ST_Y(o.location) as latitude,
        ST_X(o.location) as longitude,
        o.observation_date
    FROM occurrences o
    JOIN species s ON o.species_id = s.species_id
    WHERE ST_Within(o.location, ST_GeomFromText(polygon_wkt, 4326))
      AND (species_filter IS NULL OR o.species_id = ANY(species_filter))
      AND (start_date IS NULL OR o.observation_date >= start_date)
      AND (end_date IS NULL OR o.observation_date <= end_date)
      AND o.quality_flag IN ('validated', 'expert_verified');
END;
$$ LANGUAGE plpgsql;
```

---

## SDK 구현

### Python SDK 아키텍처

```python
# wia_biodiversity/client.py
from typing import Optional, List, Dict, Any
import httpx

class BiodiversityAPI:
    """WIA 생물다양성 API용 메인 클라이언트."""

    def __init__(
        self,
        api_key: str,
        base_url: str = "https://api.biodiversity.wia.org/v1",
        timeout: float = 30.0
    ):
        self.api_key = api_key
        self.base_url = base_url
        self._client = httpx.Client(
            base_url=base_url,
            headers={"Authorization": f"Bearer {api_key}"},
            timeout=timeout
        )

        # 서브 클라이언트 초기화
        self.occurrences = OccurrenceClient(self._client)
        self.species = SpeciesClient(self._client)
        self.indices = IndicesClient(self._client)


class OccurrenceClient:
    """출현 작업용 클라이언트."""

    def __init__(self, client: httpx.Client):
        self._client = client

    def list(
        self,
        species: Optional[str] = None,
        country: Optional[str] = None,
        year: Optional[int] = None,
        limit: int = 100
    ) -> List[Dict[str, Any]]:
        """선택적 필터로 출현 목록 조회."""
        params = {"limit": limit}
        if species:
            params["species"] = species
        if country:
            params["country"] = country
        if year:
            params["year"] = year

        response = self._client.get("/occurrences", params=params)
        response.raise_for_status()
        return response.json()["results"]

    def create(
        self,
        species: Dict[str, str],
        location: Dict[str, float],
        observation_date: str
    ) -> Dict[str, Any]:
        """새 출현 생성."""
        data = {
            "species": species,
            "location": location,
            "temporal": {"observation_date": observation_date}
        }
        response = self._client.post("/occurrences", json=data)
        response.raise_for_status()
        return response.json()


class IndicesClient:
    """다양성 지수 계산용 클라이언트."""

    def __init__(self, client: httpx.Client):
        self._client = client

    def calculate(
        self,
        dataset_id: Optional[str] = None,
        spatial_filter: Optional[Dict] = None,
        indices: List[str] = ["shannon", "simpson", "richness"],
        bootstrap_iterations: int = 1000
    ) -> Dict[str, Any]:
        """다양성 지수 계산."""
        data = {
            "indices": indices,
            "bootstrap": {
                "enabled": True,
                "iterations": bootstrap_iterations
            }
        }
        if dataset_id:
            data["dataset_id"] = dataset_id
        if spatial_filter:
            data["spatial_filter"] = spatial_filter

        response = self._client.post("/indices/calculate", json=data)
        response.raise_for_status()
        return response.json()
```

### TypeScript SDK 아키텍처

```typescript
// src/index.ts
import axios, { AxiosInstance } from 'axios';

export interface BiodiversityClientConfig {
    apiKey: string;
    baseUrl?: string;
}

export class BiodiversityAPI {
    private client: AxiosInstance;
    public occurrences: OccurrenceClient;
    public indices: IndicesClient;

    constructor(config: BiodiversityClientConfig) {
        this.client = axios.create({
            baseURL: config.baseUrl || 'https://api.biodiversity.wia.org/v1',
            headers: {
                'Authorization': `Bearer ${config.apiKey}`,
                'Content-Type': 'application/json'
            }
        });

        this.occurrences = new OccurrenceClient(this.client);
        this.indices = new IndicesClient(this.client);
    }
}

export class OccurrenceClient {
    constructor(private client: AxiosInstance) {}

    async list(filter: OccurrenceFilter = {}): Promise<Occurrence[]> {
        const response = await this.client.get('/occurrences', { params: filter });
        return response.data.results;
    }
}
```

---

## 데이터 마이그레이션

### Darwin Core Archive에서 마이그레이션

```python
import zipfile
import pandas as pd
from wia_biodiversity import BiodiversityAPI

def migrate_dwca_to_wia(dwca_path: str, api_key: str, batch_size: int = 1000):
    """
    Darwin Core Archive를 WIA 형식으로 마이그레이션.
    """
    client = BiodiversityAPI(api_key=api_key)

    # DwC-A 압축 해제
    with zipfile.ZipFile(dwca_path, 'r') as zip_ref:
        zip_ref.extractall('temp_dwca/')

    # 출현 파일 읽기
    occurrences_df = pd.read_csv('temp_dwca/occurrence.txt', sep='\t')

    # DwC 필드를 WIA 스키마에 매핑
    field_mapping = {
        'occurrenceID': 'occurrence_id',
        'scientificName': 'species.scientific_name',
        'decimalLatitude': 'location.latitude',
        'decimalLongitude': 'location.longitude',
        'eventDate': 'temporal.observation_date'
    }

    # 배치로 처리
    total = len(occurrences_df)
    success_count = 0

    for i in range(0, total, batch_size):
        batch = occurrences_df.iloc[i:i+batch_size]

        for _, row in batch.iterrows():
            try:
                wia_record = transform_dwc_to_wia(row, field_mapping)
                validation = client.occurrences.validate(wia_record)

                if validation['valid']:
                    client.occurrences.create(**wia_record)
                    success_count += 1
            except Exception as e:
                print(f"오류: {row['occurrenceID']}: {e}")

        print(f"처리됨 {min(i+batch_size, total)}/{total} 레코드")

    print(f"\n마이그레이션 완료: {success_count} 성공")
```

---

## 배포

### Docker Compose 설정

```yaml
# docker-compose.yml
version: '3.8'

services:
  api:
    build: ./api
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=postgresql://wia:wia@db:5432/biodiversity
      - REDIS_URL=redis://redis:6379
    depends_on:
      - db
      - redis

  db:
    image: postgis/postgis:15-3.3
    volumes:
      - postgres_data:/var/lib/postgresql/data
    environment:
      - POSTGRES_DB=biodiversity
      - POSTGRES_USER=wia
      - POSTGRES_PASSWORD=${DB_PASSWORD}

  redis:
    image: redis:7-alpine
    volumes:
      - redis_data:/data

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    depends_on:
      - api

volumes:
  postgres_data:
  redis_data:
```

### Kubernetes 배포

```yaml
# k8s/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-biodiversity-api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-biodiversity
  template:
    metadata:
      labels:
        app: wia-biodiversity
    spec:
      containers:
      - name: api
        image: wia/biodiversity-api:latest
        ports:
        - containerPort: 8000
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
        readinessProbe:
          httpGet:
            path: /health
            port: 8000
```

---

## 핵심 내용

1. **PostgreSQL + PostGIS**가 공간 생물다양성 데이터를 위한 견고한 기반 제공
2. **적절한 인덱싱**이 대규모 쿼리 성능에 중요
3. **SDK**가 Python, TypeScript 및 R에서 통합 단순화
4. **데이터 마이그레이션** 도구가 Darwin Core에서 원활한 전환 가능
5. **컨테이너 오케스트레이션**이 확장 가능하고 탄력적인 배포 가능

## 복습 문제

1. 생물다양성 데이터베이스에 PostGIS가 권장되는 이유는 무엇입니까?
2. 일반적인 출현 쿼리를 위해 어떤 인덱스를 생성해야 합니까?
3. Python SDK는 인증을 어떻게 처리합니까?
4. Darwin Core Archive 데이터 마이그레이션 프로세스는 무엇입니까?
5. 배포에 사용할 수 있는 컨테이너 오케스트레이션 옵션은 무엇입니까?

---

**다음 장 미리보기:** 9장에서는 AI 발전, 신기술 및 글로벌 보전 이니셔티브를 포함한 생물다양성 모니터링의 미래 동향을 탐구합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 · 모든 생명을 보존
