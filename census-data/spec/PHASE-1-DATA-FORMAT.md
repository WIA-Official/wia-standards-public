# WIA-SOC-016 PHASE 1: Data Format Specification

## Census Data Standard - Data Format and Structure

**Version:** 1.0
**Status:** PUBLISHED
**Last Updated:** 2025-12-26

---

## 1. Overview

This document specifies the data formats, structures, and schemas for census data under the WIA-SOC-016 standard. It defines how census data should be organized, encoded, and represented across different stages of the census lifecycle.

### 1.1 Objectives

- Establish standard data formats for census collection, processing, storage, and dissemination
- Ensure interoperability across different census systems and jurisdictions
- Support efficient data processing and analysis
- Enable privacy-preserving data transformations
- Facilitate international data exchange

### 1.2 Scope

This specification covers:
- Core demographic data elements
- Housing and household data structures
- Geographic coding schemes
- Metadata formats
- File formats and serialization
- Database schemas

---

## 2. Core Data Elements

### 2.1 Person-Level Data

#### 2.1.1 Demographic Characteristics

```json
{
  "person": {
    "personId": "string (UUID)",
    "householdId": "string (UUID reference)",
    "relationship": "enum (HEAD, SPOUSE, CHILD, PARENT, OTHER_RELATIVE, NONRELATIVE)",
    "age": "integer (0-120)",
    "ageGroup": "enum (0-4, 5-9, 10-14, ..., 85+)",
    "dateOfBirth": "date (YYYY-MM-DD, optional)",
    "sex": "enum (MALE, FEMALE, OTHER, PREFER_NOT_TO_SAY)",
    "gender": "string (optional)",
    "maritalStatus": "enum (SINGLE, MARRIED, DIVORCED, WIDOWED, SEPARATED)",
    "ethnicity": "array of strings",
    "race": "array of strings",
    "citizenship": "enum (CITIZEN_BIRTH, CITIZEN_NATURALIZED, PERMANENT_RESIDENT, TEMPORARY, NONE)",
    "countryOfBirth": "string (ISO 3166-1 alpha-3)",
    "languagesSpoken": ["array of strings (ISO 639-2)"],
    "religion": "string (optional)",
    "disability": "boolean",
    "disabilityTypes": ["array of enums (VISION, HEARING, MOBILITY, COGNITIVE, SELF_CARE, COMMUNICATION)"]
  }
}
```

#### 2.1.2 Education Data

```json
{
  "education": {
    "personId": "string (UUID reference)",
    "highestLevelCompleted": "enum (NONE, PRIMARY, LOWER_SECONDARY, UPPER_SECONDARY, POST_SECONDARY_NON_TERTIARY, SHORT_CYCLE_TERTIARY, BACHELORS, MASTERS, DOCTORATE)",
    "schoolAttendance": "enum (NOT_ATTENDING, PRESCHOOL, ELEMENTARY, SECONDARY, COLLEGE_UNDERGRADUATE, GRADUATE_PROFESSIONAL)",
    "fieldOfStudy": "string (ISCED-F code)",
    "literacy": "boolean"
  }
}
```

#### 2.1.3 Economic Activity

```json
{
  "economicActivity": {
    "personId": "string (UUID reference)",
    "laborForceStatus": "enum (EMPLOYED, UNEMPLOYED, NOT_IN_LABOR_FORCE)",
    "employmentStatus": "enum (EMPLOYED_FULL_TIME, EMPLOYED_PART_TIME, SELF_EMPLOYED, UNEMPLOYED_LOOKING, UNEMPLOYED_NOT_LOOKING, RETIRED, STUDENT, HOMEMAKER, UNABLE_TO_WORK)",
    "occupation": "string (ISCO-08 code)",
    "industry": "string (ISIC Rev.4 code)",
    "workHours": "integer (hours per week)",
    "income": {
      "annual": "integer (in local currency)",
      "currency": "string (ISO 4217)",
      "incomeRange": "enum (0-10K, 10K-25K, 25K-50K, 50K-75K, 75K-100K, 100K+)"
    },
    "commute": {
      "mode": "enum (WALK, BICYCLE, MOTORCYCLE, CAR_ALONE, CARPOOL, BUS, TRAIN, SUBWAY, WORK_FROM_HOME, OTHER)",
      "travelTime": "integer (minutes)",
      "placeOfWork": "string (geographic code)"
    }
  }
}
```

### 2.2 Household-Level Data

```json
{
  "household": {
    "householdId": "string (UUID)",
    "dwellingId": "string (UUID reference)",
    "householdType": "enum (SINGLE_PERSON, NUCLEAR_FAMILY, EXTENDED_FAMILY, NON_FAMILY, INSTITUTIONAL)",
    "householdSize": "integer",
    "numberChildren": "integer",
    "numberAdults": "integer",
    "numberSeniors": "integer",
    "headOfHouseholdId": "string (UUID reference to person)",
    "householdIncome": {
      "total": "integer",
      "currency": "string (ISO 4217)",
      "incomeRange": "enum"
    },
    "receivesBenefits": "boolean",
    "benefitTypes": ["array of enums (UNEMPLOYMENT, DISABILITY, PENSION, HOUSING, FOOD, OTHER)"]
  }
}
```

### 2.3 Housing Data

```json
{
  "dwelling": {
    "dwellingId": "string (UUID)",
    "address": {
      "streetNumber": "string",
      "streetName": "string",
      "apartment": "string",
      "city": "string",
      "stateProvince": "string",
      "postalCode": "string",
      "country": "string (ISO 3166-1 alpha-3)",
      "geocode": {
        "latitude": "decimal",
        "longitude": "decimal",
        "censusBlock": "string",
        "censusTract": "string",
        "county": "string",
        "state": "string"
      }
    },
    "housingType": "enum (DETACHED_HOUSE, SEMI_DETACHED, TOWNHOUSE, APARTMENT, MOBILE_HOME, INSTITUTIONAL, HOMELESS, OTHER)",
    "tenure": "enum (OWNED_OUTRIGHT, OWNED_MORTGAGE, RENTED, PROVIDED_FREE, OTHER)",
    "yearBuilt": "integer",
    "rooms": "integer",
    "bedrooms": "integer",
    "bathrooms": "decimal",
    "floorArea": "integer (square meters)",
    "utilities": {
      "electricity": "boolean",
      "naturalGas": "boolean",
      "water": "boolean",
      "sewage": "boolean",
      "internet": "boolean",
      "internetType": "enum (NONE, DIAL_UP, DSL, CABLE, FIBER, SATELLITE, MOBILE)"
    },
    "heatingType": "enum (ELECTRIC, GAS, OIL, WOOD, SOLAR, HEAT_PUMP, NONE, OTHER)",
    "coolingType": "enum (CENTRAL_AC, WINDOW_AC, EVAPORATIVE, FANS, NONE, OTHER)",
    "monthlyHousingCost": "integer",
    "propertyValue": "integer"
  }
}
```

---

## 3. Geographic Coding

### 3.1 Hierarchical Geographic Structure

```json
{
  "geography": {
    "country": {
      "code": "string (ISO 3166-1 alpha-3)",
      "name": "string"
    },
    "level1": {
      "code": "string",
      "name": "string",
      "type": "enum (STATE, PROVINCE, REGION)"
    },
    "level2": {
      "code": "string",
      "name": "string",
      "type": "enum (COUNTY, DISTRICT, PREFECTURE)"
    },
    "level3": {
      "code": "string",
      "name": "string",
      "type": "enum (CITY, MUNICIPALITY, COMMUNE)"
    },
    "censusBlock": "string",
    "censusTract": "string",
    "enumerationArea": "string",
    "urbanRural": "enum (URBAN, RURAL, MIXED)"
  }
}
```

### 3.2 Spatial Data

Geographic boundaries should be provided in standard GIS formats:
- **Vector:** GeoJSON, Shapefile, GeoPackage
- **Coordinate System:** WGS84 (EPSG:4326) for latitude/longitude
- **Projected Systems:** Local appropriate projections for area calculations

---

## 4. Metadata Standards

### 4.1 Variable-Level Metadata (DDI-Compatible)

```xml
<variable id="v_age" name="age">
  <label>Age of Person</label>
  <description>Age in completed years at time of census</description>
  <universe>All persons</universe>
  <dataType>integer</dataType>
  <range>
    <min>0</min>
    <max>120</max>
  </range>
  <categories>
    <category code="0-4" label="0 to 4 years"/>
    <category code="5-9" label="5 to 9 years"/>
    <!-- Additional age categories -->
  </categories>
  <imputationRate>0.012</imputationRate>
  <disclosureLimitation>Cell suppression for counts < 5</disclosureLimitation>
  <comparability>
    <previousCensus year="2020" variable="AGE2020"/>
    <internationalStandard>UN Principles and Recommendations Rev.3</internationalStandard>
  </comparability>
</variable>
```

### 4.2 Dataset-Level Metadata (SDMX-Compatible)

```xml
<DataSet xmlns="http://www.sdmx.org/resources/sdmxml/schemas/v2_1/message">
  <Header>
    <ID>CENSUS_2025_POPULATION</ID>
    <Test>false</Test>
    <Prepared>2025-12-26T10:00:00</Prepared>
    <Sender id="STATS_AGENCY"/>
    <Receiver id="PUBLIC"/>
    <Source>National Census 2025</Source>
  </Header>
  <DataStructure>
    <Dimension id="GEO" conceptRef="GEOGRAPHY"/>
    <Dimension id="AGE" conceptRef="AGE_GROUP"/>
    <Dimension id="SEX" conceptRef="SEX"/>
    <Measure id="POPULATION" conceptRef="POPULATION_COUNT"/>
  </DataStructure>
  <Annotations>
    <Annotation>
      <AnnotationType>QUALITY</AnnotationType>
      <AnnotationText>Margin of error: ±0.5% at 95% confidence</AnnotationText>
    </Annotation>
    <Annotation>
      <AnnotationType>PRIVACY</AnnotationType>
      <AnnotationText>Differential privacy applied with ε=1.0</AnnotationText>
    </Annotation>
  </Annotations>
</DataSet>
```

---

## 5. File Formats

### 5.1 JSON Format

**Use Case:** API responses, web applications, modern data exchange

**Example:**
```json
{
  "census": {
    "version": "2025",
    "country": "USA",
    "referenceDate": "2025-04-01",
    "data": {
      "households": [...],
      "persons": [...],
      "dwellings": [...]
    },
    "metadata": {
      "collectionMethod": "MULTI_MODE",
      "responseRate": 0.893,
      "privacyMethod": "DIFFERENTIAL_PRIVACY",
      "privacyBudget": 1.0
    }
  }
}
```

### 5.2 CSV Format

**Use Case:** Spreadsheet analysis, statistical software, simple data exchange

**Structure:**
```csv
person_id,household_id,age,sex,education,employment_status,income_range
p001,h001,34,FEMALE,BACHELORS,EMPLOYED_FULL_TIME,50K-75K
p002,h001,36,MALE,MASTERS,EMPLOYED_FULL_TIME,75K-100K
p003,h001,8,FEMALE,ELEMENTARY,NOT_IN_LABOR_FORCE,NA
```

**Requirements:**
- UTF-8 encoding
- Comma separator
- Double-quote text qualifier
- Header row with variable names
- Missing values: NA or empty
- Metadata in separate file

### 5.3 Parquet Format

**Use Case:** Big data analytics, cloud data warehouses, high-performance queries

**Schema:**
```python
schema = StructType([
    StructField("person_id", StringType(), False),
    StructField("household_id", StringType(), False),
    StructField("age", IntegerType(), True),
    StructField("sex", StringType(), True),
    StructField("education", StringType(), True),
    StructField("income", IntegerType(), True)
])
```

**Compression:** Snappy or GZIP
**Partition:** By geographic area and/or year

### 5.4 SDMX-ML Format

**Use Case:** International data exchange, statistical agencies, international organizations

**Example:**
```xml
<GenericData xmlns="http://www.sdmx.org/resources/sdmxml/schemas/v2_1/message">
  <DataSet structureRef="POPULATION_STRUCTURE">
    <Series>
      <SeriesKey>
        <Value concept="GEO" value="USA_CA_LOS_ANGELES"/>
        <Value concept="AGE_GROUP" value="25-34"/>
        <Value concept="SEX" value="FEMALE"/>
      </SeriesKey>
      <Obs>
        <ObsDimension value="2025"/>
        <ObsValue value="987654"/>
      </Obs>
    </Series>
  </DataSet>
</GenericData>
```

---

## 6. Database Schemas

### 6.1 Relational Database (PostgreSQL Example)

```sql
-- Persons table
CREATE TABLE persons (
    person_id UUID PRIMARY KEY,
    household_id UUID REFERENCES households(household_id),
    dwelling_id UUID REFERENCES dwellings(dwelling_id),
    relationship VARCHAR(20),
    age INTEGER CHECK (age >= 0 AND age <= 120),
    sex VARCHAR(20),
    marital_status VARCHAR(20),
    citizenship VARCHAR(30),
    country_of_birth CHAR(3), -- ISO 3166-1 alpha-3
    education_level VARCHAR(50),
    employment_status VARCHAR(30),
    occupation VARCHAR(10), -- ISCO-08 code
    industry VARCHAR(10), -- ISIC code
    income_range VARCHAR(20),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Households table
CREATE TABLE households (
    household_id UUID PRIMARY KEY,
    dwelling_id UUID REFERENCES dwellings(dwelling_id),
    household_type VARCHAR(20),
    household_size INTEGER,
    household_income_range VARCHAR(20),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Dwellings table
CREATE TABLE dwellings (
    dwelling_id UUID PRIMARY KEY,
    address_line1 VARCHAR(255),
    address_line2 VARCHAR(255),
    city VARCHAR(100),
    state_province VARCHAR(100),
    postal_code VARCHAR(20),
    country CHAR(3),
    latitude DECIMAL(10, 8),
    longitude DECIMAL(11, 8),
    census_block VARCHAR(20),
    census_tract VARCHAR(20),
    housing_type VARCHAR(30),
    tenure VARCHAR(30),
    year_built INTEGER,
    rooms INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_persons_household ON persons(household_id);
CREATE INDEX idx_persons_age_sex ON persons(age, sex);
CREATE INDEX idx_dwellings_geo ON dwellings(census_block, census_tract);
CREATE INDEX idx_dwellings_location ON dwellings USING GIST(ST_Point(longitude, latitude));
```

### 6.2 NoSQL Database (MongoDB Example)

```javascript
// Persons collection
{
  _id: ObjectId("..."),
  person_id: "uuid-string",
  household_id: "uuid-string",
  demographics: {
    age: 34,
    sex: "FEMALE",
    marital_status: "MARRIED",
    citizenship: "CITIZEN_BIRTH"
  },
  education: {
    level: "BACHELORS",
    field: "Computer Science"
  },
  employment: {
    status: "EMPLOYED_FULL_TIME",
    occupation: "2511", // ISCO-08
    industry: "J62", // ISIC
    income_range: "50K-75K"
  },
  metadata: {
    collection_method: "INTERNET",
    imputation_flags: {
      income: true
    }
  }
}

// Indexes
db.persons.createIndex({ "household_id": 1 });
db.persons.createIndex({ "demographics.age": 1, "demographics.sex": 1 });
db.persons.createIndex({ "employment.occupation": 1 });
```

---

## 7. Data Quality Indicators

Every census dataset should include quality metadata:

```json
{
  "dataQuality": {
    "responseRate": 0.893,
    "coverageRate": 0.967,
    "imputationRates": {
      "age": 0.005,
      "sex": 0.003,
      "education": 0.018,
      "income": 0.156
    },
    "standardErrors": {
      "totalPopulation": 125000,
      "medianAge": 0.3
    },
    "confidenceLevel": 0.95,
    "privacyMethod": "DIFFERENTIAL_PRIVACY",
    "privacyBudget": 1.0,
    "cellSuppression": {
      "threshold": 5,
      "method": "COMPLEMENTARY"
    }
  }
}
```

---

## 8. Version Control and Compatibility

### 8.1 Semantic Versioning

Census data formats follow semantic versioning: MAJOR.MINOR.PATCH

- **MAJOR:** Incompatible schema changes
- **MINOR:** Backwards-compatible additions
- **PATCH:** Backwards-compatible fixes

### 8.2 Schema Evolution

```json
{
  "schema": {
    "version": "2.1.0",
    "compatibleWith": ["2.0.0", "2.1.0"],
    "deprecated": ["1.x.x"],
    "changelog": [
      {
        "version": "2.1.0",
        "date": "2025-06-01",
        "changes": "Added gender identity field (optional)"
      },
      {
        "version": "2.0.0",
        "date": "2025-01-01",
        "changes": "Restructured geographic coding, breaking change"
      }
    ]
  }
}
```

---

## 9. Compliance and Validation

### 9.1 Validation Rules

Data must pass validation before acceptance:

1. **Required fields:** person_id, household_id, age, sex
2. **Data types:** Match specified types
3. **Value ranges:** Within specified bounds
4. **Referential integrity:** Valid household and dwelling references
5. **Enumeration values:** Match defined enumerations
6. **Consistency:** Logical relationships (e.g., children < 18 in school)

### 9.2 Validation Tools

Reference implementation provides validators in Python, R, and JavaScript for automated validation.

---

## 10. Implementation Guidelines

### 10.1 Migration from Legacy Formats

1. Map legacy variables to standard schema
2. Document transformations and assumptions
3. Validate transformed data
4. Maintain legacy formats during transition
5. Provide crosswalk documentation

### 10.2 Extensibility

Implementations may extend the schema with additional fields using namespaced extensions:

```json
{
  "person": {
    "person_id": "uuid",
    "age": 34,
    "ext:customCountry": {
      "tribalAffiliation": "Cherokee Nation",
      "militaryService": true
    }
  }
}
```

---

**Document Version:** 1.0
**Effective Date:** 2025-01-01
**Next Review:** 2027-01-01

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for census-data is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/census-data/api/` — TypeScript SDK skeleton
- `wia-standards/standards/census-data/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/census-data/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


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
