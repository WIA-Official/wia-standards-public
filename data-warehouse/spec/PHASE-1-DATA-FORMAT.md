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
