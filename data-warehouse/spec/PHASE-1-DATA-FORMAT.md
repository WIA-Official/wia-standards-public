# WIA Data Warehouse Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #10B981 (Emerald - DATA domain)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Data Types](#data-types)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Data Warehouse Data Format Standard defines unified formats for enterprise data warehouses, enabling consistent dimensional modeling, fact/dimension tables, and cross-platform compatibility for analytics and business intelligence.

**Core Objectives**:
- Standardize fact and dimension table structures
- Enable cross-platform data warehouse interoperability
- Support star schema and snowflake schema designs
- Facilitate ETL pipeline integration
- Ensure data quality and consistency

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Fact Tables | Transaction facts, periodic snapshots, accumulating snapshots |
| Dimension Tables | Slowly changing dimensions (SCD Types 0-6) |
| Data Types | Standardized column types and constraints |
| Metadata | Table lineage, data dictionaries, business definitions |
| Grain Definition | Level of detail specification |

### 1.3 Design Principles

1. **Dimensional Modeling**: Follow Kimball and Inmon best practices
2. **Denormalization**: Optimize for query performance
3. **Surrogate Keys**: Use system-generated keys for all dimensions
4. **SCD Support**: Handle slowly changing dimensions systematically
5. **Audit Tracking**: Include created/updated timestamps

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Fact Table** | Central table storing measurable business events |
| **Dimension Table** | Descriptive attributes providing context to facts |
| **Grain** | Level of detail in a fact table |
| **Surrogate Key** | System-generated unique identifier |
| **Natural Key** | Business identifier from source systems |
| **SCD** | Slowly Changing Dimension - method to track changes |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `integer` | Whole numbers | `12345` |
| `bigint` | Large integers for keys | `9223372036854775807` |
| `decimal(p,s)` | Fixed precision decimal | `decimal(10,2)` for money |
| `varchar(n)` | Variable length string | `varchar(100)` |
| `date` | Calendar date | `2025-01-15` |
| `timestamp` | Date and time | `2025-01-15 14:30:00` |
| `boolean` | True/false flag | `true`, `false` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Fact Table Format

All fact tables MUST follow this structure:

```json
{
  "table_name": "fact_{business_process}",
  "table_type": "fact",
  "grain": "transaction|daily_snapshot|monthly_snapshot|accumulating",
  "surrogate_key": "{table_name}_key",
  "foreign_keys": [
    "date_key",
    "dimension1_key",
    "dimension2_key"
  ],
  "measures": [
    {
      "name": "measure_name",
      "data_type": "decimal(10,2)|integer|bigint",
      "aggregation": "sum|avg|count|min|max",
      "additivity": "additive|semi-additive|non-additive"
    }
  ],
  "metadata": {
    "created_at": "timestamp",
    "updated_at": "timestamp",
    "etl_batch_id": "bigint"
  }
}
```

### 3.2 Dimension Table Format

All dimension tables MUST follow this structure:

```json
{
  "table_name": "dim_{subject}",
  "table_type": "dimension",
  "surrogate_key": "{table_name}_key",
  "natural_key": "business_identifier",
  "attributes": [
    {
      "name": "attribute_name",
      "data_type": "varchar|integer|date|...",
      "nullable": true|false
    }
  ],
  "scd_type": 0|1|2|3|4|6,
  "scd_columns": {
    "effective_date": "date",
    "expiration_date": "date",
    "is_current": "boolean",
    "version_number": "integer"
  },
  "hierarchies": [
    {
      "name": "hierarchy_name",
      "levels": ["level1", "level2", "level3"]
    }
  ]
}
```

---

## Data Schema

### 4.1 Fact Table Schema Example

```sql
CREATE TABLE fact_sales (
    -- Surrogate Key
    fact_sales_key BIGINT PRIMARY KEY,

    -- Foreign Keys (Dimension References)
    date_key INT NOT NULL,
    store_key INT NOT NULL,
    product_key INT NOT NULL,
    customer_key INT NOT NULL,
    promotion_key INT NOT NULL,

    -- Degenerate Dimensions
    order_number VARCHAR(50),
    invoice_number VARCHAR(50),

    -- Additive Measures
    sales_amount DECIMAL(10,2) NOT NULL,
    cost_amount DECIMAL(10,2) NOT NULL,
    profit_amount DECIMAL(10,2) NOT NULL,
    quantity_sold INTEGER NOT NULL,
    discount_amount DECIMAL(10,2),

    -- Semi-Additive Measures
    inventory_level INTEGER,

    -- Non-Additive Measures
    unit_price DECIMAL(10,2),
    profit_margin DECIMAL(5,2),

    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    etl_batch_id BIGINT,

    -- Foreign Key Constraints
    FOREIGN KEY (date_key) REFERENCES dim_date(date_key),
    FOREIGN KEY (store_key) REFERENCES dim_store(store_key),
    FOREIGN KEY (product_key) REFERENCES dim_product(product_key),
    FOREIGN KEY (customer_key) REFERENCES dim_customer(customer_key),
    FOREIGN KEY (promotion_key) REFERENCES dim_promotion(promotion_key)
);

-- Indexes for Performance
CREATE INDEX idx_fact_sales_date ON fact_sales(date_key);
CREATE INDEX idx_fact_sales_store ON fact_sales(store_key);
CREATE INDEX idx_fact_sales_product ON fact_sales(product_key);
CREATE INDEX idx_fact_sales_customer ON fact_sales(customer_key);
```

### 4.2 Dimension Table Schema Example (SCD Type 2)

```sql
CREATE TABLE dim_customer (
    -- Surrogate Key
    customer_key INT PRIMARY KEY AUTO_INCREMENT,

    -- Natural Key
    customer_id VARCHAR(50) NOT NULL,

    -- Attributes
    customer_name VARCHAR(100) NOT NULL,
    email VARCHAR(100),
    phone VARCHAR(20),
    address VARCHAR(200),
    city VARCHAR(50),
    state VARCHAR(50),
    country VARCHAR(50),
    postal_code VARCHAR(20),

    -- Customer Segmentation
    customer_segment VARCHAR(30),
    customer_tier VARCHAR(20),
    lifetime_value DECIMAL(12,2),

    -- SCD Type 2 Columns
    effective_date DATE NOT NULL,
    expiration_date DATE,
    is_current BOOLEAN DEFAULT TRUE,
    version_number INT DEFAULT 1,

    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Indexes
    INDEX idx_customer_id (customer_id),
    INDEX idx_customer_current (customer_id, is_current),
    INDEX idx_customer_effective (effective_date),

    -- Constraints
    CHECK (effective_date <= COALESCE(expiration_date, '9999-12-31'))
);
```

---

## Field Specifications

### 5.1 Common Fact Table Fields

| Field Name | Type | Required | Description |
|------------|------|----------|-------------|
| `{table}_key` | BIGINT | YES | Surrogate primary key |
| `{dimension}_key` | INT | YES | Foreign key to dimension |
| `{measure}_amount` | DECIMAL(10,2) | YES | Monetary measures |
| `{measure}_quantity` | INTEGER | YES | Count measures |
| `created_at` | TIMESTAMP | YES | Record creation time |
| `updated_at` | TIMESTAMP | YES | Last update time |
| `etl_batch_id` | BIGINT | NO | ETL batch identifier |

### 5.2 Common Dimension Fields

| Field Name | Type | Required | Description |
|------------|------|----------|-------------|
| `{table}_key` | INT | YES | Surrogate primary key |
| `{entity}_id` | VARCHAR(50) | YES | Natural business key |
| `{entity}_name` | VARCHAR(100) | YES | Descriptive name |
| `effective_date` | DATE | CONDITIONAL | SCD Type 2 start date |
| `expiration_date` | DATE | CONDITIONAL | SCD Type 2 end date |
| `is_current` | BOOLEAN | CONDITIONAL | Current record flag |

---

## Data Types

### 6.1 Supported Data Types

```json
{
  "numeric": {
    "integer": "Whole numbers (-2,147,483,648 to 2,147,483,647)",
    "bigint": "Large integers (-9 quintillion to 9 quintillion)",
    "decimal(p,s)": "Fixed precision (p=precision, s=scale)",
    "numeric(p,s)": "Alias for decimal",
    "float": "Floating point (not recommended for money)",
    "double": "Double precision float"
  },
  "string": {
    "varchar(n)": "Variable length string (max n chars)",
    "char(n)": "Fixed length string (exactly n chars)",
    "text": "Unlimited length text"
  },
  "temporal": {
    "date": "Calendar date (YYYY-MM-DD)",
    "time": "Time of day (HH:MI:SS)",
    "timestamp": "Date and time",
    "timestamp with timezone": "Timestamp with timezone info"
  },
  "boolean": {
    "boolean": "True/false value"
  }
}
```

---

## Validation Rules

### 7.1 Fact Table Validations

```json
{
  "structural": [
    "MUST have a surrogate primary key ending in '_key'",
    "MUST have at least one foreign key to a dimension",
    "MUST have at least one measure column",
    "MUST NOT contain descriptive text attributes (use dimensions)"
  ],
  "data_quality": [
    "Foreign keys MUST reference existing dimension records",
    "Measures MUST be numeric",
    "Grain MUST be clearly defined and consistent",
    "NULL values in measures MUST be explicitly handled"
  ],
  "performance": [
    "SHOULD have indexes on all foreign keys",
    "SHOULD partition large tables by date",
    "SHOULD use appropriate data types (no VARCHAR for numbers)"
  ]
}
```

### 7.2 Dimension Validation Rules

```json
{
  "structural": [
    "MUST have a surrogate primary key",
    "MUST have a natural key from source system",
    "SCD Type 2 dimensions MUST have effective_date, expiration_date, is_current",
    "Hierarchies MUST be denormalized in star schema"
  ],
  "data_quality": [
    "Natural keys MUST be indexed",
    "Only one record per natural key CAN have is_current = TRUE",
    "effective_date MUST be <= expiration_date",
    "Text fields SHOULD be trimmed and standardized"
  ]
}
```

---

## Examples

### 8.1 Complete Star Schema Example

```sql
-- Date Dimension (Shared across all facts)
CREATE TABLE dim_date (
    date_key INT PRIMARY KEY,
    full_date DATE NOT NULL UNIQUE,
    day_of_week INT,
    day_name VARCHAR(10),
    day_of_month INT,
    day_of_year INT,
    week_of_year INT,
    month INT,
    month_name VARCHAR(10),
    quarter INT,
    year INT,
    is_weekend BOOLEAN,
    is_holiday BOOLEAN,
    fiscal_year INT,
    fiscal_quarter INT
);

-- Product Dimension
CREATE TABLE dim_product (
    product_key INT PRIMARY KEY AUTO_INCREMENT,
    product_id VARCHAR(50) NOT NULL,
    product_name VARCHAR(100),
    category VARCHAR(50),
    subcategory VARCHAR(50),
    brand VARCHAR(50),
    manufacturer VARCHAR(100),
    unit_cost DECIMAL(10,2),
    unit_price DECIMAL(10,2),
    effective_date DATE,
    expiration_date DATE,
    is_current BOOLEAN DEFAULT TRUE
);

-- Store Dimension
CREATE TABLE dim_store (
    store_key INT PRIMARY KEY AUTO_INCREMENT,
    store_id VARCHAR(20) NOT NULL,
    store_name VARCHAR(100),
    store_type VARCHAR(30),
    city VARCHAR(50),
    state VARCHAR(50),
    country VARCHAR(50),
    region VARCHAR(50),
    district VARCHAR(50),
    manager_name VARCHAR(100),
    opening_date DATE
);

-- Sales Fact Table
CREATE TABLE fact_sales (
    fact_sales_key BIGINT PRIMARY KEY,
    date_key INT NOT NULL,
    product_key INT NOT NULL,
    store_key INT NOT NULL,
    customer_key INT NOT NULL,

    sales_amount DECIMAL(12,2),
    quantity_sold INT,
    discount_amount DECIMAL(10,2),
    cost_amount DECIMAL(12,2),
    profit_amount DECIMAL(12,2),

    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (date_key) REFERENCES dim_date(date_key),
    FOREIGN KEY (product_key) REFERENCES dim_product(product_key),
    FOREIGN KEY (store_key) REFERENCES dim_store(store_key),
    FOREIGN KEY (customer_key) REFERENCES dim_customer(customer_key)
);
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

**License**: MIT
**Copyright**: © 2025 WIA Standards Committee
**Philosophy**: 弘益人間 (Benefit All Humanity)


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
