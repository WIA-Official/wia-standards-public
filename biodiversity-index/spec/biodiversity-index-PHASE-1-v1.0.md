# WIA Biodiversity Index Standard - Phase 1: Data Format
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity, Preserve All Life

## Overview

Phase 1 defines standardized data formats for biodiversity information, enabling interoperability across systems and institutions. This specification establishes JSON schemas, validation rules, and data exchange protocols for species occurrences, eDNA samples, habitat classifications, and diversity indices.

## Scope

This phase covers:
- Species occurrence data schema
- eDNA sample data schema
- Habitat classification system
- Diversity index results schema
- Taxonomy reference schema
- Data validation rules
- Serialization formats (JSON, GeoJSON, CSV, Parquet)

## Core Schemas

### 1. Species Occurrence Schema

**Schema ID:** `https://wia.org/schemas/occurrence/v1.0`

**Required Fields:**
- `occurrence_id`: Unique identifier (string, pattern: `^OCC-\d{4}-\d{6}$`)
- `species.scientific_name`: Binomial nomenclature (string, capitalized)
- `location.latitude`: Decimal degrees (-90 to 90)
- `location.longitude`: Decimal degrees (-180 to 180)
- `temporal.observation_date`: ISO 8601 timestamp

**Recommended Fields:**
- `observation.individual_count`: Integer ≥ 1
- `location.habitat_type`: Controlled vocabulary (see Section 3)
- `observation.observer_id`: Observer identifier
- `quality.quality_flag`: Enum [unvalidated, validated, expert_verified]

**Optional Fields:**
- `observation.behavior`: Free text or controlled vocabulary
- `observation.life_stage`: Enum [egg, larva, juvenile, adult, unknown]
- `environmental.*`: Temperature, humidity, weather conditions

**Example:**
```json
{
  "$schema": "https://wia.org/schemas/occurrence/v1.0",
  "occurrence_id": "OCC-2025-123456",
  "species": {
    "scientific_name": "Panthera tigris",
    "common_name": "Bengal Tiger",
    "taxonomy": {
      "kingdom": "Animalia",
      "phylum": "Chordata",
      "class": "Mammalia",
      "order": "Carnivora",
      "family": "Felidae"
    },
    "iucn_status": "EN"
  },
  "location": {
    "latitude": 27.5142,
    "longitude": 88.7597,
    "elevation_m": 350,
    "habitat_type": "tropical_forest",
    "country": "India"
  },
  "temporal": {
    "observation_date": "2025-11-15T09:30:00Z"
  },
  "observation": {
    "individual_count": 1,
    "observer_id": "OBS-2025-042",
    "basis_of_record": "human_observation"
  }
}
```

### 2. eDNA Sample Schema

**Schema ID:** `https://wia.org/schemas/edna/v1.0`

**Required Fields:**
- `sample_id`: Unique identifier (pattern: `^EDNA-\d{4}-[A-Z]{2}-\d{6}$`)
- `collection.date`: ISO 8601 timestamp
- `collection.location`: Geographic coordinates
- `processing.sequencing_platform`: Controlled vocabulary

**Structure:**
```json
{
  "$schema": "https://wia.org/schemas/edna/v1.0",
  "sample_id": "EDNA-2025-UK-089234",
  "collection": {
    "date": "2025-10-22T11:15:00Z",
    "location": {
      "latitude": 51.4545,
      "longitude": -0.9782,
      "water_body_name": "River Thames"
    },
    "sampling_method": "filtration",
    "volume_liters": 2.0,
    "filter_pore_size_um": 0.45
  },
  "processing": {
    "extraction_method": "qiagen_dneasy_powerwater",
    "sequencing_platform": "illumina_novaseq_6000",
    "target_gene": "COI"
  },
  "results": {
    "detected_taxa": [
      {
        "scientific_name": "Salmo trutta",
        "read_count": 15432,
        "confidence": 0.97
      }
    ]
  }
}
```

### 3. Habitat Classification System

Based on IUCN Habitat Classification Scheme with WIA extensions.

**Level 1 Categories:**
1. Forest
2. Savanna
3. Shrubland
4. Grassland
5. Wetlands (inland)
6. Wetlands (coastal/supratidal)
7. Marine Neritic
8. Marine Oceanic
9. Marine Deep Ocean Floor
10. Marine Intertidal
11. Marine Coastal/Supratidal
12. Artificial - Terrestrial
13. Artificial - Aquatic
14. Other
15. Unknown

**Level 2 & 3 Subcategories:** See Appendix A

**Example Classification:**
```json
{
  "habitat_type": "forest",
  "habitat_subtype": "tropical_subtropical_moist_lowland",
  "habitat_detail": "primary_forest",
  "disturbance_level": "minimal",
  "canopy_cover_percent": 85
}
```

### 4. Diversity Index Results Schema

**Schema ID:** `https://wia.org/schemas/diversity-index/v1.0`

**Required Fields:**
- `calculation_id`: Unique identifier
- `indices.*`: At least one index (shannon, simpson, richness, evenness)
- `input_data.occurrence_count`: Integer ≥ 1
- `calculation_date`: ISO 8601 timestamp

**Example:**
```json
{
  "$schema": "https://wia.org/schemas/diversity-index/v1.0",
  "calculation_id": "CALC-2025-123",
  "dataset_id": "DS-AMAZON-2025",
  "spatial_extent": {
    "type": "polygon",
    "area_km2": 125.8
  },
  "indices": {
    "species_richness": {
      "value": 156,
      "rarefied_value": 142.7,
      "confidence_interval_95": [138.2, 147.3]
    },
    "shannon_diversity": {
      "value": 4.127,
      "confidence_interval_95": [3.982, 4.268]
    },
    "simpson_index": {
      "value": 0.0234,
      "diversity_1_minus_d": 0.9766
    }
  }
}
```

## Validation Rules

### Geographic Validation
1. **Coordinate Range:** -90 ≤ latitude ≤ 90, -180 ≤ longitude ≤ 180
2. **Zero-Zero Check:** Flag coordinates at (0, 0) unless Gulf of Guinea
3. **Country Match:** Coordinates must fall within stated country boundaries
4. **Elevation Check:** Elevation must be reasonable for coordinates (DEM lookup)

### Temporal Validation
1. **Future Date:** observation_date must not be in the future
2. **Historical Limit:** observation_date typically not before 1600
3. **Precision Consistency:** Date precision matches stated resolution

### Taxonomic Validation
1. **Nomenclature:** Scientific name follows binomial/trinomial rules
2. **Capitalization:** Genus capitalized, species/subspecies lowercase
3. **Hierarchy Consistency:** Taxonomy matches expected kingdom/phylum/class
4. **Authority Check:** Taxon ID resolves to valid external reference (GBIF, NCBI)

### Ecological Validation
1. **Range Check:** Species occurrence within known geographic range (warning, not error)
2. **Habitat Plausibility:** Habitat type reasonable for species
3. **Count Reasonableness:** Individual count not biologically implausible
4. **Environmental Bounds:** Temperature, humidity within reasonable ranges

## Data Exchange Formats

### JSON (Canonical)
- UTF-8 encoding
- Pretty-printed for human readability (development)
- Minified for production APIs
- Content-Type: `application/json`

### GeoJSON
- Follows RFC 7946
- Feature collections for multiple occurrences
- Properties contain full occurrence metadata
- Content-Type: `application/geo+json`

### CSV
- UTF-8 encoding with BOM
- Header row required
- Nested fields flattened with dot notation (e.g., `species.scientific_name`)
- Missing values: empty string (not "null" or "N/A")
- Content-Type: `text/csv`

### Parquet
- Apache Parquet 2.0+ format
- Columnar storage for big data analytics
- Schema embedded in file
- Compression: Snappy or Gzip

## Compliance Requirements

### Bronze Certification
- 90% of records conform to schema
- All required fields populated
- 95% pass automated validation
- Complete metadata documentation

### Validation Levels
1. **Schema Validation:** JSON Schema compliance
2. **Semantic Validation:** Business rules (e.g., date not in future)
3. **Ecological Validation:** Species range, habitat plausibility
4. **Expert Review:** Manual verification of uncertain identifications

## Implementation Guidance

### Database Design (PostgreSQL + PostGIS)
```sql
CREATE TABLE occurrences (
  occurrence_id TEXT PRIMARY KEY,
  scientific_name TEXT NOT NULL,
  location GEOMETRY(Point, 4326) NOT NULL,
  observation_date TIMESTAMP NOT NULL,
  data JSONB NOT NULL,

  -- Indexes
  CREATE INDEX idx_species ON occurrences(scientific_name);
  CREATE INDEX idx_location ON occurrences USING GIST(location);
  CREATE INDEX idx_date ON occurrences(observation_date);
  CREATE INDEX idx_habitat ON occurrences((data->>'location.habitat_type'));
);
```

### Python Validation Example
```python
import jsonschema
from wia_biodiversity import schemas

# Load schema
schema = schemas.load('occurrence/v1.0')

# Validate data
occurrence_data = {
    "occurrence_id": "OCC-2025-123456",
    "species": {"scientific_name": "Panthera tigris"},
    # ... more fields
}

try:
    jsonschema.validate(occurrence_data, schema)
    print("Valid!")
except jsonschema.ValidationError as e:
    print(f"Validation error: {e.message}")
```

## Migration from Legacy Data

Conversion utilities available at: https://github.com/WIA-Official/biodiversity-tools

**Common Legacy Formats:**
- Darwin Core Archive → WIA JSON
- eBird EBD → WIA Occurrence
- GBIF Download → WIA Occurrence
- Custom CSV → WIA (with field mapping)

## Versioning and Evolution

**Version Format:** MAJOR.MINOR.PATCH (Semantic Versioning)

**Backward Compatibility:**
- MINOR version: Backward compatible additions
- MAJOR version: Breaking changes (deprecated fields removed)
- PATCH version: Bug fixes, clarifications

**Deprecation Policy:**
- 12-month notice before removal
- Migration guides provided
- Legacy version support for 24 months

## References

1. Darwin Core Standard: https://dwc.tdwg.org/
2. IUCN Habitat Classification: https://www.iucnredlist.org/resources/habitat-classification-scheme
3. JSON Schema: https://json-schema.org/
4. GeoJSON RFC 7946: https://tools.ietf.org/html/rfc7946
5. ISO 8601 Date/Time: https://www.iso.org/iso-8601-date-and-time-format.html

## Appendix A: Habitat Classification

(Full IUCN hierarchy with WIA extensions - 150+ habitat types)

## Appendix B: Controlled Vocabularies

**Basis of Record:**
- human_observation
- machine_observation
- specimen
- living_specimen
- fossil_specimen
- material_sample
- occurrence

**Detection Method:**
- visual
- auditory
- camera_trap
- edna
- remote_sensing
- literature
- historical_record

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity · Preserve All Life
