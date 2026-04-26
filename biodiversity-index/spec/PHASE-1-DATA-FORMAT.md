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


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.
