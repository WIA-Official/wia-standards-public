# Chapter 8: Implementation Guide

## Practical Database Design, SDK Usage, and System Deployment

### Building WIA-Compliant Biodiversity Data Systems

---

## Overview

This chapter provides practical guidance for implementing WIA Biodiversity Index Standard-compliant systems, including database design, SDK integration, data migration, and deployment strategies.

---

## Database Design

### PostgreSQL + PostGIS Architecture

The recommended database architecture uses PostgreSQL with the PostGIS extension for geospatial capabilities.

**Schema Overview:**

```sql
-- Enable required extensions
CREATE EXTENSION IF NOT EXISTS postgis;
CREATE EXTENSION IF NOT EXISTS pg_trgm;  -- For text search
CREATE EXTENSION IF NOT EXISTS btree_gist;  -- For range queries

-- Core tables
CREATE SCHEMA biodiversity;
SET search_path TO biodiversity, public;
```

### Core Tables

**Occurrences Table:**

```sql
CREATE TABLE occurrences (
    occurrence_id VARCHAR(50) PRIMARY KEY,
    species_id INTEGER NOT NULL REFERENCES species(species_id),
    dataset_id INTEGER NOT NULL REFERENCES datasets(dataset_id),

    -- Location (PostGIS geometry)
    location GEOMETRY(Point, 4326) NOT NULL,
    coordinate_uncertainty_m DECIMAL(10, 2),
    elevation_m INTEGER,
    depth_m INTEGER,
    country_code CHAR(2),
    protected_area_id INTEGER REFERENCES protected_areas(pa_id),

    -- Temporal
    observation_date TIMESTAMPTZ NOT NULL,
    observation_date_end TIMESTAMPTZ,
    date_precision VARCHAR(20) CHECK (date_precision IN ('year', 'month', 'day', 'time')),

    -- Observation details
    individual_count INTEGER DEFAULT 1,
    sex VARCHAR(20),
    life_stage VARCHAR(30),
    behavior TEXT,
    basis_of_record VARCHAR(30) NOT NULL,
    detection_method VARCHAR(30),
    observer_id VARCHAR(50),

    -- Quality
    quality_flag VARCHAR(20) DEFAULT 'unvalidated',
    quality_score DECIMAL(3, 2),
    verified_by VARCHAR(50),
    verification_date TIMESTAMPTZ,

    -- Metadata
    created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    raw_data JSONB,

    -- Constraints
    CONSTRAINT valid_quality_flag CHECK (
        quality_flag IN ('unvalidated', 'validated', 'expert_verified', 'flagged')
    )
);

-- Indexes for common queries
CREATE INDEX idx_occurrences_species ON occurrences(species_id);
CREATE INDEX idx_occurrences_location ON occurrences USING GIST(location);
CREATE INDEX idx_occurrences_date ON occurrences(observation_date);
CREATE INDEX idx_occurrences_country ON occurrences(country_code);
CREATE INDEX idx_occurrences_quality ON occurrences(quality_flag);
CREATE INDEX idx_occurrences_dataset ON occurrences(dataset_id);

-- Partial index for validated records only
CREATE INDEX idx_occurrences_validated ON occurrences(species_id, observation_date)
    WHERE quality_flag IN ('validated', 'expert_verified');

-- Trigger for updated_at
CREATE OR REPLACE FUNCTION update_modified_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER occurrences_update_timestamp
    BEFORE UPDATE ON occurrences
    FOR EACH ROW
    EXECUTE FUNCTION update_modified_timestamp();
```

**Species Table:**

```sql
CREATE TABLE species (
    species_id SERIAL PRIMARY KEY,
    scientific_name VARCHAR(255) NOT NULL UNIQUE,
    genus VARCHAR(100) NOT NULL,
    species_epithet VARCHAR(100) NOT NULL,
    subspecies VARCHAR(100),
    name_authority VARCHAR(255),

    -- Taxonomy
    kingdom VARCHAR(50),
    phylum VARCHAR(50),
    class VARCHAR(50),
    "order" VARCHAR(50),
    family VARCHAR(50),

    -- External references
    gbif_key INTEGER,
    ncbi_taxon_id INTEGER,
    iucn_taxon_id INTEGER,

    -- Conservation status
    iucn_status VARCHAR(10),
    iucn_assessment_date DATE,
    population_trend VARCHAR(20),

    -- Metadata
    common_names JSONB,
    synonyms TEXT[],
    created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
);

-- Indexes
CREATE INDEX idx_species_name ON species(scientific_name);
CREATE INDEX idx_species_gbif ON species(gbif_key);
CREATE INDEX idx_species_iucn ON species(iucn_status);
CREATE INDEX idx_species_family ON species(family);

-- Full-text search on names
CREATE INDEX idx_species_name_trgm ON species USING GIN(scientific_name gin_trgm_ops);
```

**eDNA Samples Table:**

```sql
CREATE TABLE edna_samples (
    sample_id VARCHAR(50) PRIMARY KEY,
    dataset_id INTEGER NOT NULL REFERENCES datasets(dataset_id),

    -- Collection
    collection_date TIMESTAMPTZ NOT NULL,
    location GEOMETRY(Point, 4326) NOT NULL,
    water_body_name VARCHAR(255),
    water_body_type VARCHAR(50),
    sampling_method VARCHAR(50),
    volume_liters DECIMAL(6, 2),
    filter_pore_size_um DECIMAL(4, 2),
    replicates INTEGER DEFAULT 3,

    -- Environmental conditions
    water_temperature_c DECIMAL(5, 2),
    water_ph DECIMAL(4, 2),
    turbidity_ntu DECIMAL(8, 2),
    conductivity_us_cm DECIMAL(10, 2),

    -- Processing
    extraction_date TIMESTAMPTZ,
    extraction_method VARCHAR(100),
    sequencing_platform VARCHAR(50),
    sequencing_date TIMESTAMPTZ,
    target_gene VARCHAR(20),
    primer_set VARCHAR(100),
    processing_lab VARCHAR(255),

    -- Results
    total_reads INTEGER,
    reads_after_qc INTEGER,
    species_detected INTEGER,

    -- Quality
    negative_control_status VARCHAR(20),
    quality_flag VARCHAR(20) DEFAULT 'pending',

    -- Metadata
    collector_id VARCHAR(50),
    created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    raw_data JSONB
);

CREATE INDEX idx_edna_location ON edna_samples USING GIST(location);
CREATE INDEX idx_edna_date ON edna_samples(collection_date);
CREATE INDEX idx_edna_dataset ON edna_samples(dataset_id);
```

**eDNA Detections Table:**

```sql
CREATE TABLE edna_detections (
    detection_id SERIAL PRIMARY KEY,
    sample_id VARCHAR(50) NOT NULL REFERENCES edna_samples(sample_id),
    species_id INTEGER NOT NULL REFERENCES species(species_id),

    read_count INTEGER NOT NULL,
    relative_abundance DECIMAL(10, 8),
    confidence DECIMAL(4, 3),
    detection_type VARCHAR(20) DEFAULT 'confirmed',

    created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_edna_detections_sample ON edna_detections(sample_id);
CREATE INDEX idx_edna_detections_species ON edna_detections(species_id);
```

**Diversity Calculations Table:**

```sql
CREATE TABLE diversity_calculations (
    calculation_id VARCHAR(50) PRIMARY KEY,
    dataset_id INTEGER REFERENCES datasets(dataset_id),

    -- Spatial extent
    spatial_extent GEOMETRY(Polygon, 4326),
    area_km2 DECIMAL(12, 2),
    centroid GEOMETRY(Point, 4326),

    -- Temporal extent
    start_date DATE,
    end_date DATE,

    -- Input summary
    occurrence_count INTEGER,
    species_count INTEGER,
    sampling_completeness DECIMAL(4, 3),

    -- Calculated indices
    species_richness INTEGER,
    rarefied_richness DECIMAL(8, 2),
    rarefaction_depth INTEGER,
    shannon_index DECIMAL(6, 4),
    shannon_variance DECIMAL(10, 8),
    simpson_d DECIMAL(6, 5),
    simpson_1_minus_d DECIMAL(6, 5),
    chao1_estimate DECIMAL(8, 2),
    chao1_se DECIMAL(8, 2),
    pielou_evenness DECIMAL(5, 4),

    -- Confidence intervals (stored as JSONB)
    confidence_intervals JSONB,

    -- Calculation metadata
    calculation_date TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
    calculation_time_ms INTEGER,
    parameters JSONB,
    software_version VARCHAR(50)
);

CREATE INDEX idx_diversity_spatial ON diversity_calculations USING GIST(spatial_extent);
CREATE INDEX idx_diversity_dataset ON diversity_calculations(dataset_id);
CREATE INDEX idx_diversity_date ON diversity_calculations(calculation_date);
```

### Optimized Queries

**Spatial Query for Occurrences:**

```sql
-- Find occurrences within polygon
CREATE OR REPLACE FUNCTION get_occurrences_in_polygon(
    polygon_wkt TEXT,
    species_filter INTEGER[] DEFAULT NULL,
    start_date DATE DEFAULT NULL,
    end_date DATE DEFAULT NULL,
    quality_filters VARCHAR[] DEFAULT ARRAY['validated', 'expert_verified']
)
RETURNS TABLE (
    occurrence_id VARCHAR,
    scientific_name VARCHAR,
    latitude DOUBLE PRECISION,
    longitude DOUBLE PRECISION,
    observation_date TIMESTAMPTZ,
    individual_count INTEGER
) AS $$
BEGIN
    RETURN QUERY
    SELECT
        o.occurrence_id,
        s.scientific_name,
        ST_Y(o.location) as latitude,
        ST_X(o.location) as longitude,
        o.observation_date,
        o.individual_count
    FROM occurrences o
    JOIN species s ON o.species_id = s.species_id
    WHERE ST_Within(o.location, ST_GeomFromText(polygon_wkt, 4326))
      AND (species_filter IS NULL OR o.species_id = ANY(species_filter))
      AND (start_date IS NULL OR o.observation_date >= start_date)
      AND (end_date IS NULL OR o.observation_date <= end_date)
      AND o.quality_flag = ANY(quality_filters);
END;
$$ LANGUAGE plpgsql;

-- Usage
SELECT * FROM get_occurrences_in_polygon(
    'POLYGON((-60.5 -3.5, -60.0 -3.5, -60.0 -3.0, -60.5 -3.0, -60.5 -3.5))',
    NULL,
    '2025-01-01',
    '2025-12-31'
);
```

**Diversity Calculation Query:**

```sql
-- Calculate diversity indices for a region
CREATE OR REPLACE FUNCTION calculate_diversity(
    polygon_wkt TEXT,
    start_date DATE,
    end_date DATE
)
RETURNS TABLE (
    species_richness INTEGER,
    total_individuals BIGINT,
    shannon_index DOUBLE PRECISION,
    simpson_d DOUBLE PRECISION
) AS $$
DECLARE
    total_n BIGINT;
    shannon DOUBLE PRECISION := 0;
    simpson DOUBLE PRECISION := 0;
BEGIN
    -- Get species abundances
    CREATE TEMP TABLE temp_abundances AS
    SELECT
        o.species_id,
        SUM(o.individual_count) as abundance
    FROM occurrences o
    WHERE ST_Within(o.location, ST_GeomFromText(polygon_wkt, 4326))
      AND o.observation_date BETWEEN start_date AND end_date
      AND o.quality_flag IN ('validated', 'expert_verified')
    GROUP BY o.species_id;

    -- Calculate total individuals
    SELECT SUM(abundance) INTO total_n FROM temp_abundances;

    -- Calculate Shannon and Simpson
    SELECT
        -SUM((abundance::float / total_n) * LN(abundance::float / total_n)),
        SUM((abundance::float / total_n) ^ 2)
    INTO shannon, simpson
    FROM temp_abundances;

    RETURN QUERY
    SELECT
        (SELECT COUNT(*) FROM temp_abundances)::INTEGER as species_richness,
        total_n as total_individuals,
        shannon as shannon_index,
        simpson as simpson_d;

    DROP TABLE temp_abundances;
END;
$$ LANGUAGE plpgsql;
```

---

## SDK Implementation

### Python SDK Architecture

```python
# wia_biodiversity/client.py
from typing import Optional, List, Dict, Any
import httpx
from pydantic import BaseModel
from datetime import datetime

class BiodiversityAPI:
    """Main client for WIA Biodiversity API."""

    def __init__(
        self,
        api_key: str,
        base_url: str = "https://api.biodiversity.wia.org/v1",
        timeout: float = 30.0
    ):
        self.api_key = api_key
        self.base_url = base_url
        self.timeout = timeout
        self._client = httpx.Client(
            base_url=base_url,
            headers={"Authorization": f"Bearer {api_key}"},
            timeout=timeout
        )

        # Initialize sub-clients
        self.occurrences = OccurrenceClient(self._client)
        self.species = SpeciesClient(self._client)
        self.indices = IndicesClient(self._client)
        self.spatial = SpatialClient(self._client)
        self.export = ExportClient(self._client)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._client.close()


class OccurrenceClient:
    """Client for occurrence operations."""

    def __init__(self, client: httpx.Client):
        self._client = client

    def list(
        self,
        species: Optional[str] = None,
        country: Optional[str] = None,
        year: Optional[int] = None,
        bbox: Optional[tuple] = None,
        limit: int = 100,
        offset: int = 0
    ) -> List[Dict[str, Any]]:
        """List occurrences with optional filters."""
        params = {
            "limit": limit,
            "offset": offset
        }
        if species:
            params["species"] = species
        if country:
            params["country"] = country
        if year:
            params["year"] = year
        if bbox:
            params["bbox"] = ",".join(map(str, bbox))

        response = self._client.get("/occurrences", params=params)
        response.raise_for_status()
        return response.json()["results"]

    def get(self, occurrence_id: str) -> Dict[str, Any]:
        """Get a single occurrence by ID."""
        response = self._client.get(f"/occurrences/{occurrence_id}")
        response.raise_for_status()
        return response.json()

    def create(
        self,
        species: Dict[str, str],
        location: Dict[str, float],
        observation_date: str,
        **kwargs
    ) -> Dict[str, Any]:
        """Create a new occurrence."""
        data = {
            "species": species,
            "location": location,
            "temporal": {"observation_date": observation_date},
            **kwargs
        }
        response = self._client.post("/occurrences", json=data)
        response.raise_for_status()
        return response.json()

    def validate(self, occurrence_data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate occurrence data without creating."""
        response = self._client.post("/validate/occurrence", json=occurrence_data)
        response.raise_for_status()
        return response.json()


class IndicesClient:
    """Client for diversity index calculations."""

    def __init__(self, client: httpx.Client):
        self._client = client

    def calculate(
        self,
        dataset_id: Optional[str] = None,
        spatial_filter: Optional[Dict] = None,
        temporal_filter: Optional[Dict] = None,
        indices: List[str] = ["shannon", "simpson", "richness"],
        bootstrap_iterations: int = 1000,
        rarefaction_depth: Optional[int] = None
    ) -> Dict[str, Any]:
        """Calculate diversity indices."""
        data = {
            "indices": indices,
            "bootstrap": {
                "enabled": True,
                "iterations": bootstrap_iterations,
                "confidence_level": 0.95
            }
        }
        if dataset_id:
            data["dataset_id"] = dataset_id
        if spatial_filter:
            data["spatial_filter"] = spatial_filter
        if temporal_filter:
            data["temporal_filter"] = temporal_filter
        if rarefaction_depth:
            data["rarefaction"] = {
                "enabled": True,
                "target_n": rarefaction_depth
            }

        response = self._client.post("/indices/calculate", json=data)
        response.raise_for_status()
        return response.json()

    def get(self, calculation_id: str) -> Dict[str, Any]:
        """Get calculation results."""
        response = self._client.get(f"/indices/{calculation_id}")
        response.raise_for_status()
        return response.json()
```

### TypeScript SDK Architecture

```typescript
// src/index.ts
import axios, { AxiosInstance } from 'axios';

export interface BiodiversityClientConfig {
    apiKey: string;
    baseUrl?: string;
    timeout?: number;
}

export class BiodiversityAPI {
    private client: AxiosInstance;
    public occurrences: OccurrenceClient;
    public species: SpeciesClient;
    public indices: IndicesClient;

    constructor(config: BiodiversityClientConfig) {
        this.client = axios.create({
            baseURL: config.baseUrl || 'https://api.biodiversity.wia.org/v1',
            timeout: config.timeout || 30000,
            headers: {
                'Authorization': `Bearer ${config.apiKey}`,
                'Content-Type': 'application/json'
            }
        });

        this.occurrences = new OccurrenceClient(this.client);
        this.species = new SpeciesClient(this.client);
        this.indices = new IndicesClient(this.client);
    }
}

export interface OccurrenceFilter {
    species?: string;
    country?: string;
    year?: number;
    bbox?: [number, number, number, number];
    limit?: number;
    offset?: number;
}

export class OccurrenceClient {
    constructor(private client: AxiosInstance) {}

    async list(filter: OccurrenceFilter = {}): Promise<Occurrence[]> {
        const params = new URLSearchParams();

        if (filter.species) params.append('species', filter.species);
        if (filter.country) params.append('country', filter.country);
        if (filter.year) params.append('year', String(filter.year));
        if (filter.bbox) params.append('bbox', filter.bbox.join(','));
        params.append('limit', String(filter.limit || 100));
        params.append('offset', String(filter.offset || 0));

        const response = await this.client.get('/occurrences', { params });
        return response.data.results;
    }

    async create(data: CreateOccurrenceInput): Promise<Occurrence> {
        const response = await this.client.post('/occurrences', data);
        return response.data;
    }

    async validate(data: CreateOccurrenceInput): Promise<ValidationResult> {
        const response = await this.client.post('/validate/occurrence', data);
        return response.data;
    }
}

export interface CalculateIndicesInput {
    datasetId?: string;
    spatialFilter?: GeoJSON.Polygon;
    temporalFilter?: { startDate: string; endDate: string };
    indices?: string[];
    bootstrapIterations?: number;
}

export class IndicesClient {
    constructor(private client: AxiosInstance) {}

    async calculate(input: CalculateIndicesInput): Promise<DiversityResults> {
        const response = await this.client.post('/indices/calculate', {
            dataset_id: input.datasetId,
            spatial_filter: input.spatialFilter,
            temporal_filter: input.temporalFilter,
            indices: input.indices || ['shannon', 'simpson', 'richness'],
            bootstrap: {
                enabled: true,
                iterations: input.bootstrapIterations || 1000,
                confidence_level: 0.95
            }
        });
        return response.data;
    }
}
```

---

## Data Migration

### From Darwin Core Archive

```python
import zipfile
import pandas as pd
from wia_biodiversity import BiodiversityAPI

def migrate_dwca_to_wia(dwca_path: str, api_key: str, batch_size: int = 1000):
    """
    Migrate Darwin Core Archive to WIA format.

    Args:
        dwca_path: Path to DwC-A zip file
        api_key: WIA API key
        batch_size: Records per batch
    """
    client = BiodiversityAPI(api_key=api_key)

    # Extract DwC-A
    with zipfile.ZipFile(dwca_path, 'r') as zip_ref:
        zip_ref.extractall('temp_dwca/')

    # Read occurrence file
    occurrences_df = pd.read_csv('temp_dwca/occurrence.txt', sep='\t')

    # Map DwC fields to WIA schema
    field_mapping = {
        'occurrenceID': 'occurrence_id',
        'scientificName': 'species.scientific_name',
        'decimalLatitude': 'location.latitude',
        'decimalLongitude': 'location.longitude',
        'eventDate': 'temporal.observation_date',
        'individualCount': 'observation.individual_count',
        'basisOfRecord': 'observation.basis_of_record',
        'countryCode': 'location.country_code'
    }

    # Process in batches
    total = len(occurrences_df)
    success_count = 0
    error_count = 0

    for i in range(0, total, batch_size):
        batch = occurrences_df.iloc[i:i+batch_size]

        for _, row in batch.iterrows():
            try:
                wia_record = transform_dwc_to_wia(row, field_mapping)

                # Validate first
                validation = client.occurrences.validate(wia_record)

                if validation['valid']:
                    client.occurrences.create(**wia_record)
                    success_count += 1
                else:
                    print(f"Validation failed for {row['occurrenceID']}: {validation['errors']}")
                    error_count += 1

            except Exception as e:
                print(f"Error processing {row['occurrenceID']}: {e}")
                error_count += 1

        print(f"Processed {min(i+batch_size, total)}/{total} records")

    print(f"\nMigration complete: {success_count} success, {error_count} errors")


def transform_dwc_to_wia(row: pd.Series, mapping: dict) -> dict:
    """Transform Darwin Core record to WIA format."""
    wia_record = {
        'species': {
            'scientific_name': row.get('scientificName', ''),
        },
        'location': {
            'latitude': float(row['decimalLatitude']),
            'longitude': float(row['decimalLongitude']),
        },
        'temporal': {
            'observation_date': row['eventDate']
        },
        'observation': {
            'individual_count': int(row.get('individualCount', 1)),
            'basis_of_record': row.get('basisOfRecord', 'observation')
        }
    }

    # Add optional fields
    if pd.notna(row.get('countryCode')):
        wia_record['location']['country_code'] = row['countryCode']

    if pd.notna(row.get('coordinateUncertaintyInMeters')):
        wia_record['location']['coordinate_uncertainty_m'] = float(row['coordinateUncertaintyInMeters'])

    return wia_record
```

---

## Deployment

### Docker Compose Setup

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
      - JWT_SECRET=${JWT_SECRET}
    depends_on:
      - db
      - redis
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3

  db:
    image: postgis/postgis:15-3.3
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./init.sql:/docker-entrypoint-initdb.d/init.sql
    environment:
      - POSTGRES_DB=biodiversity
      - POSTGRES_USER=wia
      - POSTGRES_PASSWORD=${DB_PASSWORD}
    ports:
      - "5432:5432"

  redis:
    image: redis:7-alpine
    volumes:
      - redis_data:/data
    ports:
      - "6379:6379"

  worker:
    build: ./worker
    environment:
      - DATABASE_URL=postgresql://wia:wia@db:5432/biodiversity
      - REDIS_URL=redis://redis:6379
    depends_on:
      - db
      - redis

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./certs:/etc/nginx/certs
    depends_on:
      - api

volumes:
  postgres_data:
  redis_data:
```

### Kubernetes Deployment

```yaml
# k8s/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-biodiversity-api
  labels:
    app: wia-biodiversity
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
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-credentials
              key: url
        - name: REDIS_URL
          valueFrom:
            configMapKeyRef:
              name: app-config
              key: redis_url
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
          initialDelaySeconds: 10
          periodSeconds: 5
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
---
apiVersion: v1
kind: Service
metadata:
  name: wia-biodiversity-api
spec:
  selector:
    app: wia-biodiversity
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8000
  type: LoadBalancer
```

---

## Key Takeaways

1. **PostgreSQL + PostGIS** provides robust foundation for spatial biodiversity data
2. **Proper indexing** critical for query performance at scale
3. **SDKs** simplify integration in Python, TypeScript, and R
4. **Data migration** tools enable smooth transition from Darwin Core
5. **Container orchestration** enables scalable, resilient deployments

## Review Questions

1. Why is PostGIS recommended for biodiversity databases?
2. What indexes should be created for common occurrence queries?
3. How does the Python SDK handle authentication?
4. What is the process for migrating Darwin Core Archive data?
5. What container orchestration options are available for deployment?

---

**Next Chapter Preview:** Chapter 9 explores future trends in biodiversity monitoring, including AI advances, emerging technologies, and global conservation initiatives.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Preserve All Life
