# Chapter 3: Data Formats

## Phase 1: Standardized Biodiversity Data Schemas

### Enabling Global Interoperability Through Structured Data

---

## Overview

Phase 1 of the WIA Biodiversity Index Standard defines standardized data formats for biodiversity information, enabling interoperability across systems and institutions worldwide. This chapter covers JSON schemas, validation rules, and data exchange protocols for species occurrences, eDNA samples, habitat classifications, and diversity indices.

---

## Core Schema Architecture

### Design Principles

The WIA Biodiversity Index data formats follow these core principles:

1. **JSON-Native**: Primary format is JSON for modern API integration
2. **Darwin Core Compatible**: Mappings to established GBIF standards
3. **Extensible**: Core schema with optional extensions
4. **Validated**: Machine-readable validation rules
5. **Versioned**: Semantic versioning with deprecation policies

### Schema Registry

**Schema Base URL:** `https://wia.org/schemas/`

| Schema | ID | Version | Status |
|--------|-----|---------|--------|
| Species Occurrence | `occurrence/v1.0` | 1.0 | Active |
| eDNA Sample | `edna/v1.0` | 1.0 | Active |
| Habitat Classification | `habitat/v1.0` | 1.0 | Active |
| Diversity Index | `diversity-index/v1.0` | 1.0 | Active |
| Taxonomy Reference | `taxonomy/v1.0` | 1.0 | Active |
| Survey Event | `survey-event/v1.0` | 1.0 | Active |

---

## Species Occurrence Schema

### Schema Definition

**Schema ID:** `https://wia.org/schemas/occurrence/v1.0`

The species occurrence schema captures standardized records of when, where, and how a species was detected.

### Field Requirements

**Required Fields:**

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| `occurrence_id` | string | Unique identifier | Pattern: `^OCC-\d{4}-\d{6}$` |
| `species.scientific_name` | string | Binomial nomenclature | Capitalized genus |
| `location.latitude` | number | Decimal degrees | -90 to 90 |
| `location.longitude` | number | Decimal degrees | -180 to 180 |
| `temporal.observation_date` | string | ISO 8601 timestamp | Not in future |

**Recommended Fields:**

| Field | Type | Description | Default |
|-------|------|-------------|---------|
| `observation.individual_count` | integer | Number observed | 1 |
| `location.habitat_type` | string | Habitat classification | null |
| `observation.observer_id` | string | Observer identifier | null |
| `quality.quality_flag` | enum | Validation status | "unvalidated" |

**Optional Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `observation.behavior` | string | Behavioral notes |
| `observation.life_stage` | enum | Developmental stage |
| `environmental.temperature_c` | number | Air/water temperature |
| `environmental.humidity_percent` | number | Relative humidity |
| `observation.photo_url` | string | Evidence photograph |

### Complete Schema Example

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
      "family": "Felidae",
      "genus": "Panthera",
      "species": "tigris"
    },
    "iucn_status": "EN",
    "taxon_id": "GBIF:9694",
    "name_authority": "Linnaeus, 1758"
  },
  "location": {
    "latitude": 27.5142,
    "longitude": 88.7597,
    "coordinate_uncertainty_m": 50,
    "elevation_m": 350,
    "depth_m": null,
    "habitat_type": "tropical_moist_forest",
    "habitat_subtype": "tropical_subtropical_moist_lowland",
    "country": "India",
    "country_code": "IN",
    "state_province": "West Bengal",
    "locality": "Sundarbans National Park",
    "protected_area": {
      "name": "Sundarbans National Park",
      "wdpa_id": 9164,
      "iucn_category": "II"
    }
  },
  "temporal": {
    "observation_date": "2025-11-15T09:30:00Z",
    "observation_date_end": null,
    "date_precision": "day",
    "event_time_zone": "Asia/Kolkata"
  },
  "observation": {
    "individual_count": 1,
    "count_precision": "exact",
    "sex": "male",
    "life_stage": "adult",
    "reproductive_condition": null,
    "behavior": "foraging",
    "basis_of_record": "human_observation",
    "detection_method": "visual",
    "observer_id": "OBS-2025-042",
    "observer_name": "Dr. Ananya Sharma",
    "institution_id": "INST-WII-001",
    "survey_id": "SURV-2025-SUND-001",
    "photo_url": "https://assets.wia.org/occurrences/OCC-2025-123456/photo1.jpg",
    "audio_url": null,
    "specimen_id": null,
    "notes": "Adult male tiger observed near water hole during morning survey"
  },
  "environmental": {
    "temperature_c": 28.5,
    "humidity_percent": 82,
    "wind_speed_ms": 2.3,
    "precipitation": "none",
    "cloud_cover_percent": 40,
    "water_temperature_c": null,
    "water_ph": null,
    "salinity_ppt": null
  },
  "quality": {
    "quality_flag": "expert_verified",
    "quality_score": 0.95,
    "verification_date": "2025-11-16T14:00:00Z",
    "verifier_id": "VER-2025-007",
    "issues": [],
    "georeferencing_method": "GPS",
    "identification_certainty": "high"
  },
  "metadata": {
    "created_at": "2025-11-15T15:45:00Z",
    "updated_at": "2025-11-16T14:05:00Z",
    "version": 2,
    "license": "CC-BY-4.0",
    "rights_holder": "Wildlife Institute of India",
    "dataset_id": "DS-TIGER-INDIA-2025",
    "access_rights": "open"
  }
}
```

### Darwin Core Mapping

The WIA occurrence schema maps to Darwin Core terms:

| WIA Field | Darwin Core Term | Notes |
|-----------|-----------------|-------|
| `occurrence_id` | `occurrenceID` | Direct mapping |
| `species.scientific_name` | `scientificName` | Direct mapping |
| `location.latitude` | `decimalLatitude` | Direct mapping |
| `location.longitude` | `decimalLongitude` | Direct mapping |
| `temporal.observation_date` | `eventDate` | ISO 8601 format |
| `observation.individual_count` | `individualCount` | Direct mapping |
| `observation.basis_of_record` | `basisOfRecord` | Controlled vocabulary |
| `location.country_code` | `countryCode` | ISO 3166-1 alpha-2 |
| `species.taxonomy.*` | Higher taxonomy terms | Kingdom through species |

---

## eDNA Sample Schema

### Schema Definition

**Schema ID:** `https://wia.org/schemas/edna/v1.0`

The eDNA schema captures environmental DNA sample collection, processing, and detection results.

### Sample Collection Fields

```json
{
  "$schema": "https://wia.org/schemas/edna/v1.0",
  "sample_id": "EDNA-2025-UK-089234",
  "collection": {
    "date": "2025-10-22T11:15:00Z",
    "location": {
      "latitude": 51.4545,
      "longitude": -0.9782,
      "coordinate_uncertainty_m": 10,
      "water_body_name": "River Thames",
      "water_body_type": "river",
      "country_code": "GB"
    },
    "sampling_method": "filtration",
    "volume_liters": 2.0,
    "filter_type": "cellulose_nitrate",
    "filter_pore_size_um": 0.45,
    "replicates": 3,
    "negative_control": true,
    "field_conditions": {
      "water_temperature_c": 14.2,
      "water_ph": 7.8,
      "turbidity_ntu": 12.5,
      "conductivity_us_cm": 620,
      "dissolved_oxygen_mg_l": 9.2
    },
    "collector_id": "COL-2025-UK-042",
    "collection_protocol": "WIA-EDNA-AQUATIC-v1"
  },
  "processing": {
    "extraction_date": "2025-10-23T09:00:00Z",
    "extraction_method": "qiagen_dneasy_powerwater",
    "extraction_kit_lot": "LOT-2025-QIA-7834",
    "sequencing_platform": "illumina_novaseq_6000",
    "sequencing_date": "2025-10-28T00:00:00Z",
    "target_gene": "COI",
    "primer_set": "mlCOIintF_jgHCO2198",
    "pcr_cycles": 35,
    "library_prep_method": "nextera_xt",
    "read_length_bp": 250,
    "processing_lab": "NatureMetrics Ltd",
    "lab_id": "LAB-UK-NM-001"
  },
  "bioinformatics": {
    "pipeline_version": "WIA-EDNA-PIPELINE-v2.3",
    "reference_database": "BOLD_v5.0",
    "database_date": "2025-10-01",
    "quality_filtering": {
      "min_read_quality": 30,
      "min_read_length": 200,
      "chimera_removal": true
    },
    "clustering_method": "DADA2",
    "taxonomic_assignment": "RDP_classifier",
    "confidence_threshold": 0.80
  },
  "results": {
    "total_reads": 2847562,
    "reads_after_qc": 2456789,
    "detected_taxa": [
      {
        "scientific_name": "Salmo trutta",
        "common_name": "Brown Trout",
        "taxon_id": "GBIF:2339947",
        "read_count": 15432,
        "relative_abundance": 0.0063,
        "confidence": 0.97,
        "iucn_status": "LC",
        "detection_type": "confirmed"
      },
      {
        "scientific_name": "Esox lucius",
        "common_name": "Northern Pike",
        "taxon_id": "GBIF:2340537",
        "read_count": 8921,
        "relative_abundance": 0.0036,
        "confidence": 0.94,
        "iucn_status": "LC",
        "detection_type": "confirmed"
      },
      {
        "scientific_name": "Anguilla anguilla",
        "common_name": "European Eel",
        "taxon_id": "GBIF:2401986",
        "read_count": 4287,
        "relative_abundance": 0.0017,
        "confidence": 0.91,
        "iucn_status": "CR",
        "detection_type": "confirmed"
      }
    ],
    "species_richness": 24,
    "negative_control_status": "clean"
  },
  "quality": {
    "quality_flag": "validated",
    "quality_score": 0.92,
    "issues": [],
    "validation_date": "2025-11-01T10:00:00Z"
  },
  "metadata": {
    "created_at": "2025-11-01T12:00:00Z",
    "license": "CC-BY-4.0",
    "project_id": "PROJ-UK-THAMES-2025",
    "dataset_id": "DS-EDNA-UK-2025"
  }
}
```

### eDNA Quality Control

**Required QC Checks:**

| Check | Requirement | Action if Failed |
|-------|------------|------------------|
| Negative control | No target DNA detected | Flag sample, investigate contamination |
| Positive control | Expected species detected | Flag batch, re-extract |
| Read depth | Minimum 10,000 reads per sample | Flag for re-sequencing |
| Chimera rate | <5% of total reads | Re-run bioinformatics |
| Extraction blank | No amplification | Flag batch contamination |

---

## Habitat Classification Schema

### IUCN-Based Hierarchy

The WIA Habitat Classification follows the IUCN Habitat Classification Scheme with extensions for detailed ecological characterization.

### Level 1 Categories

| Code | Category | Description |
|------|----------|-------------|
| 1 | Forest | Tree-dominated ecosystems |
| 2 | Savanna | Grassland with scattered trees |
| 3 | Shrubland | Shrub-dominated ecosystems |
| 4 | Grassland | Grass-dominated ecosystems |
| 5 | Wetlands (inland) | Freshwater wetlands |
| 6 | Rocky areas | Caves, cliffs, rock outcrops |
| 7 | Caves and subterranean | Underground habitats |
| 8 | Desert | Arid ecosystems |
| 9 | Marine neritic | Shallow ocean (0-200m) |
| 10 | Marine oceanic | Open ocean (>200m) |
| 11 | Marine deep ocean | Benthic zones |
| 12 | Marine intertidal | Tidal zone |
| 13 | Marine coastal | Coastal ecosystems |
| 14 | Artificial - terrestrial | Human-modified land |
| 15 | Artificial - aquatic | Human-modified water |
| 16 | Introduced vegetation | Planted areas |
| 17 | Other | Unclassified |
| 18 | Unknown | Data deficient |

### Level 2 Forest Subcategories

| Code | Subcategory | Climate Zone |
|------|-------------|--------------|
| 1.1 | Tropical/subtropical moist lowland | Tropical |
| 1.2 | Tropical/subtropical dry | Tropical |
| 1.3 | Tropical/subtropical moist montane | Tropical |
| 1.4 | Temperate | Temperate |
| 1.5 | Subtropical/tropical mangrove | Coastal |
| 1.6 | Subtropical/tropical swamp | Wetland |
| 1.7 | Subtropical/tropical moist coastal | Coastal |
| 1.8 | Boreal | Boreal |
| 1.9 | Subtropical/tropical dry scrub | Semi-arid |

### Habitat Schema Example

```json
{
  "$schema": "https://wia.org/schemas/habitat/v1.0",
  "habitat_id": "HAB-2025-AMAZON-0042",
  "classification": {
    "level_1": {
      "code": 1,
      "name": "Forest"
    },
    "level_2": {
      "code": "1.1",
      "name": "Tropical/subtropical moist lowland forest"
    },
    "level_3": {
      "code": "1.1.1",
      "name": "Primary rainforest"
    }
  },
  "characteristics": {
    "canopy_height_m": 35,
    "canopy_cover_percent": 92,
    "understory_density": "dense",
    "leaf_phenology": "evergreen",
    "dominant_species": [
      "Bertholletia excelsa",
      "Swietenia macrophylla",
      "Ceiba pentandra"
    ],
    "structural_complexity": "high",
    "age_class": "old_growth"
  },
  "condition": {
    "disturbance_level": "minimal",
    "disturbance_type": null,
    "fragmentation": "low",
    "connectivity": "high",
    "invasive_species_presence": false,
    "degradation_score": 0.05,
    "recovery_potential": "high"
  },
  "location": {
    "latitude": -3.4653,
    "longitude": -62.2159,
    "area_hectares": 1250,
    "elevation_m": 85,
    "slope_percent": 2,
    "aspect": "flat",
    "country_code": "BR",
    "protected_area_id": "WDPA-3948"
  },
  "climate": {
    "koppen_class": "Af",
    "annual_precipitation_mm": 2840,
    "mean_annual_temp_c": 26.5,
    "dry_season_months": 0,
    "climate_zone": "equatorial"
  },
  "metadata": {
    "assessment_date": "2025-09-15",
    "assessor_id": "ASS-2025-BR-012",
    "methodology": "WIA-HABITAT-ASSESSMENT-v1",
    "confidence": "high",
    "data_sources": ["field_survey", "satellite_imagery", "lidar"]
  }
}
```

---

## Diversity Index Results Schema

### Schema Definition

**Schema ID:** `https://wia.org/schemas/diversity-index/v1.0`

The diversity index schema captures calculated biodiversity metrics with statistical confidence measures.

### Supported Indices

| Index | Formula | Range | Interpretation |
|-------|---------|-------|----------------|
| Species Richness (S) | Count of species | 0 to ∞ | Higher = more species |
| Shannon Diversity (H') | -Σ(pi × ln(pi)) | 0 to ln(S) | Higher = more diverse |
| Simpson's Index (D) | Σ(pi²) | 0 to 1 | Lower = more diverse |
| Simpson's Diversity (1-D) | 1 - Σ(pi²) | 0 to 1 | Higher = more diverse |
| Inverse Simpson (1/D) | 1 / Σ(pi²) | 1 to S | Higher = more diverse |
| Pielou's Evenness (J') | H' / ln(S) | 0 to 1 | Higher = more even |
| Chao1 Estimator | S + (f₁²/2f₂) | S to ∞ | Estimated true richness |
| ACE | Advanced estimator | S to ∞ | Estimated true richness |

### Results Schema Example

```json
{
  "$schema": "https://wia.org/schemas/diversity-index/v1.0",
  "calculation_id": "CALC-2025-AMAZON-001",
  "input_summary": {
    "dataset_id": "DS-AMAZON-2025",
    "occurrence_count": 15847,
    "species_count": 156,
    "sampling_completeness": 0.82
  },
  "spatial_extent": {
    "type": "polygon",
    "area_km2": 125.8,
    "centroid": {
      "latitude": -3.4653,
      "longitude": -62.2159
    },
    "country_code": "BR",
    "protected_area": "Jaú National Park"
  },
  "temporal_extent": {
    "start_date": "2025-01-01",
    "end_date": "2025-12-31",
    "seasons_covered": ["dry", "wet"]
  },
  "taxonomic_scope": {
    "taxon_group": "birds",
    "taxonomic_level": "species",
    "authority": "Clements_2024"
  },
  "indices": {
    "species_richness": {
      "observed": 156,
      "rarefied_value": 142.7,
      "rarefaction_depth": 10000,
      "confidence_interval_95": {
        "lower": 138.2,
        "upper": 147.3
      },
      "extrapolated_asymptote": 178.4
    },
    "shannon_diversity": {
      "value": 4.127,
      "variance": 0.0052,
      "confidence_interval_95": {
        "lower": 3.982,
        "upper": 4.268
      },
      "effective_number_species": 62.1
    },
    "simpson_index": {
      "dominance_d": 0.0234,
      "diversity_1_minus_d": 0.9766,
      "inverse_d": 42.7,
      "confidence_interval_95": {
        "lower": 0.0198,
        "upper": 0.0271
      }
    },
    "evenness": {
      "pielou_j": 0.818,
      "simpson_evenness": 0.274,
      "confidence_interval_95": {
        "lower": 0.792,
        "upper": 0.844
      }
    },
    "richness_estimators": {
      "chao1": {
        "estimate": 172.4,
        "standard_error": 8.3,
        "confidence_interval_95": {
          "lower": 161.2,
          "upper": 195.7
        }
      },
      "ace": {
        "estimate": 168.9,
        "standard_error": 6.7
      },
      "jackknife1": {
        "estimate": 174.2,
        "standard_error": 5.4
      },
      "bootstrap": {
        "estimate": 165.3,
        "standard_error": 4.2
      }
    },
    "beta_diversity": {
      "spatial_turnover": 0.42,
      "nestedness": 0.18,
      "total_beta": 0.60
    }
  },
  "calculation_parameters": {
    "rarefaction_enabled": true,
    "rarefaction_iterations": 1000,
    "bootstrap_enabled": true,
    "bootstrap_iterations": 1000,
    "confidence_level": 0.95,
    "random_seed": 42
  },
  "quality": {
    "completeness_score": 0.82,
    "evenness_warning": false,
    "sample_size_adequate": true,
    "singleton_ratio": 0.12,
    "doubleton_ratio": 0.08
  },
  "metadata": {
    "calculation_date": "2025-12-20T14:30:00Z",
    "calculation_time_ms": 2847,
    "software_version": "WIA-Biodiversity-SDK-v1.2.0",
    "algorithm_reference": "선행 연구, Ecological Monographs"
  }
}
```

---

## Validation Rules

### Geographic Validation

**Coordinate Validity:**
```python
def validate_coordinates(lat, lon):
    errors = []

    # Range check
    if not (-90 <= lat <= 90):
        errors.append("Latitude out of range [-90, 90]")
    if not (-180 <= lon <= 180):
        errors.append("Longitude out of range [-180, 180]")

    # Zero-zero check (often indicates missing data)
    if lat == 0 and lon == 0:
        errors.append("Warning: Coordinates at (0,0) - verify if correct")

    # Precision check
    decimal_places = len(str(lat).split('.')[-1]) if '.' in str(lat) else 0
    if decimal_places < 3:
        errors.append("Warning: Low coordinate precision (<3 decimal places)")

    return errors
```

**Country Boundary Verification:**
- Coordinates must fall within stated country boundaries
- Uses Natural Earth or GADM administrative boundaries
- Buffer tolerance of 1km for coastal/border areas
- Warning (not error) for near-border coordinates

**Elevation Consistency:**
- Elevation compared to DEM (Digital Elevation Model)
- Tolerance of ±100m for natural variation
- Warning if discrepancy >200m
- Marine species: depth validation against bathymetry

### Temporal Validation

**Date Rules:**
```python
def validate_temporal(observation_date, date_precision):
    errors = []

    # Future date check
    if observation_date > datetime.now():
        errors.append("Observation date cannot be in the future")

    # Historical limit
    if observation_date.year < 1600:
        errors.append("Warning: Date before 1600 - verify historical record")

    # Precision consistency
    if date_precision == "day" and observation_date.hour != 0:
        errors.append("Date precision mismatch")

    return errors
```

### Taxonomic Validation

**Nomenclature Rules:**
1. Scientific name follows binomial/trinomial format
2. Genus capitalized, species/subspecies lowercase
3. No special characters except hyphen in compound epithets
4. Author citation optional but standardized if present

**Taxonomy Hierarchy:**
- Validate kingdom → phylum → class → order → family → genus → species
- Cross-reference against GBIF Backbone Taxonomy
- Flag mismatches between stated and resolved taxonomy
- Warning for synonyms, suggest accepted name

**External Validation:**
```python
def validate_taxon(scientific_name, taxon_id=None):
    # Check GBIF Backbone
    gbif_match = gbif_api.match_name(scientific_name)

    if gbif_match['matchType'] == 'NONE':
        return {"valid": False, "error": "Name not found in GBIF"}

    if gbif_match['matchType'] == 'FUZZY':
        return {
            "valid": True,
            "warning": f"Fuzzy match to {gbif_match['canonicalName']}",
            "confidence": gbif_match['confidence']
        }

    if gbif_match['status'] == 'SYNONYM':
        return {
            "valid": True,
            "warning": f"Synonym of {gbif_match['acceptedName']}",
            "accepted_name": gbif_match['acceptedName']
        }

    return {"valid": True, "taxon_key": gbif_match['usageKey']}
```

### Ecological Validation

**Species Range Check:**
- Compare occurrence to known species distribution
- Use IUCN range maps or modeled distributions
- Flag occurrences outside known range (warning, not error)
- Exception for documented range extensions

**Habitat Plausibility:**
- Cross-reference species habitat preferences with recorded habitat
- Flag marine species in freshwater habitats
- Flag desert species in rainforest habitats
- Allow for edge cases with documentation

---

## Serialization Formats

### JSON (Canonical)

**Primary format for API communication:**
- UTF-8 encoding required
- Pretty-printed for development
- Minified for production
- Content-Type: `application/json`

```json
{
  "occurrence_id": "OCC-2025-123456",
  "species": {
    "scientific_name": "Panthera tigris"
  }
}
```

### GeoJSON

**Spatial data representation:**
- Follows RFC 7946 specification
- Feature collections for multiple occurrences
- Properties contain full occurrence metadata
- Content-Type: `application/geo+json`

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [88.7597, 27.5142]
      },
      "properties": {
        "occurrence_id": "OCC-2025-123456",
        "scientific_name": "Panthera tigris",
        "observation_date": "2025-11-15T09:30:00Z"
      }
    }
  ]
}
```

### CSV

**Tabular data export:**
- UTF-8 encoding with BOM
- Header row required
- Nested fields flattened with dot notation
- Missing values as empty strings
- Content-Type: `text/csv`

```csv
occurrence_id,species.scientific_name,location.latitude,location.longitude,temporal.observation_date
OCC-2025-123456,Panthera tigris,27.5142,88.7597,2025-11-15T09:30:00Z
OCC-2025-123457,Elephas maximus,27.5201,88.7634,2025-11-15T10:15:00Z
```

### Apache Parquet

**Big data analytics:**
- Apache Parquet 2.0+ format
- Columnar storage for efficient queries
- Schema embedded in file
- Compression: Snappy (default) or Gzip
- Ideal for datasets >100,000 records

---

## Key Takeaways

1. **JSON-native schemas** provide modern API integration while maintaining Darwin Core compatibility
2. **Occurrence schema** captures who, what, when, where with standardized validation rules
3. **eDNA schema** covers entire workflow from collection through bioinformatics to detection results
4. **Habitat classification** follows IUCN hierarchy with condition and climate extensions
5. **Diversity index schema** includes statistical confidence measures and richness estimators

## Review Questions

1. What are the five required fields in the WIA species occurrence schema?
2. How does the WIA schema map to Darwin Core terms?
3. What quality control checks are required for eDNA samples?
4. Name three biodiversity indices included in the diversity index schema.
5. What validation rules apply to geographic coordinates?

---

**Next Chapter Preview:** Chapter 4 explores the WIA Biodiversity Index API Interface (Phase 2), covering RESTful endpoints, GraphQL queries, authentication, and SDK support.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Preserve All Life
