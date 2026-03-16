# WIA-DATA-011: PHASE 1 - Data Format Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26

## Overview

This specification defines the standard data formats for data visualization in the WIA-DATA-011 standard. It ensures interoperability between visualization tools, data sources, and analytics platforms.

## Table of Contents

1. [Core Data Formats](#core-data-formats)
2. [Chart Data Schema](#chart-data-schema)
3. [Time Series Format](#time-series-format)
4. [Hierarchical Data](#hierarchical-data)
5. [Geospatial Data](#geospatial-data)
6. [Metadata Specification](#metadata-specification)
7. [Validation Rules](#validation-rules)

---

## 1. Core Data Formats

### 1.1 Supported Formats

WIA-DATA-011 supports the following data formats:

| Format | Extension | MIME Type | Usage |
|--------|-----------|-----------|-------|
| JSON | `.json` | `application/json` | Primary format for web-based visualizations |
| CSV | `.csv` | `text/csv` | Simple tabular data |
| Parquet | `.parquet` | `application/octet-stream` | Large-scale data, columnar storage |
| Arrow | `.arrow` | `application/vnd.apache.arrow.stream` | High-performance data interchange |
| GeoJSON | `.geojson` | `application/geo+json` | Geospatial data |

### 1.2 JSON Chart Data Format

The primary format for chart data is JSON with the following structure:

```json
{
  "metadata": {
    "version": "1.0",
    "created": "2025-12-26T10:30:00Z",
    "source": "sales-database",
    "description": "Quarterly sales data for 2025",
    "license": "CC-BY-4.0"
  },
  "schema": {
    "fields": [
      {
        "name": "quarter",
        "type": "string",
        "description": "Quarter identifier (Q1, Q2, Q3, Q4)"
      },
      {
        "name": "revenue",
        "type": "number",
        "unit": "USD",
        "description": "Total revenue in US dollars"
      },
      {
        "name": "region",
        "type": "string",
        "description": "Geographic region"
      }
    ]
  },
  "data": [
    { "quarter": "Q1", "revenue": 120000, "region": "North" },
    { "quarter": "Q2", "revenue": 150000, "region": "North" },
    { "quarter": "Q3", "revenue": 135000, "region": "South" },
    { "quarter": "Q4", "revenue": 180000, "region": "South" }
  ]
}
```

### 1.3 Data Types

#### Primitive Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | Text data | `"North America"` |
| `number` | Numeric data (integer or float) | `42`, `3.14159` |
| `boolean` | True/false values | `true`, `false` |
| `null` | Absence of value | `null` |
| `date` | ISO 8601 date | `"2025-12-26"` |
| `datetime` | ISO 8601 datetime | `"2025-12-26T10:30:00Z"` |
| `timestamp` | Unix timestamp (milliseconds) | `1735210200000` |

#### Complex Types

| Type | Description | Example |
|------|-------------|---------|
| `array` | Ordered list | `[1, 2, 3, 4]` |
| `object` | Key-value pairs | `{"x": 10, "y": 20}` |
| `geolocation` | Latitude/longitude | `{"lat": 37.7749, "lng": -122.4194}` |

---

## 2. Chart Data Schema

### 2.1 Schema Definition

Every dataset MUST include a schema definition that describes the structure:

```json
{
  "schema": {
    "fields": [
      {
        "name": "field_name",
        "type": "data_type",
        "nullable": false,
        "description": "Human-readable description",
        "unit": "measurement_unit",
        "format": "format_string",
        "constraints": {
          "min": 0,
          "max": 100,
          "pattern": "regex_pattern"
        }
      }
    ]
  }
}
```

### 2.2 Field Attributes

| Attribute | Required | Type | Description |
|-----------|----------|------|-------------|
| `name` | Yes | string | Field identifier |
| `type` | Yes | string | Data type |
| `nullable` | No | boolean | Can be null (default: true) |
| `description` | No | string | Human-readable description |
| `unit` | No | string | Measurement unit |
| `format` | No | string | Format specification |
| `constraints` | No | object | Validation constraints |

### 2.3 Example: Sales Data Schema

```json
{
  "schema": {
    "fields": [
      {
        "name": "date",
        "type": "date",
        "nullable": false,
        "description": "Transaction date",
        "format": "YYYY-MM-DD"
      },
      {
        "name": "revenue",
        "type": "number",
        "nullable": false,
        "description": "Daily revenue",
        "unit": "USD",
        "constraints": {
          "min": 0
        }
      },
      {
        "name": "product_category",
        "type": "string",
        "nullable": false,
        "description": "Product category",
        "constraints": {
          "enum": ["Electronics", "Clothing", "Food", "Other"]
        }
      }
    ]
  }
}
```

---

## 3. Time Series Format

### 3.1 Structure

Time series data uses a standardized format:

```json
{
  "metadata": {
    "granularity": "daily",
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-12-31T23:59:59Z",
    "timezone": "UTC"
  },
  "schema": {
    "fields": [
      { "name": "timestamp", "type": "datetime" },
      { "name": "value", "type": "number" }
    ]
  },
  "data": [
    { "timestamp": "2025-01-01T00:00:00Z", "value": 120 },
    { "timestamp": "2025-01-02T00:00:00Z", "value": 135 },
    { "timestamp": "2025-01-03T00:00:00Z", "value": 142 }
  ]
}
```

### 3.2 Granularity Levels

| Level | Description | Format |
|-------|-------------|--------|
| `millisecond` | Millisecond precision | ISO 8601 with ms |
| `second` | Second precision | `YYYY-MM-DDTHH:mm:ss` |
| `minute` | Minute precision | `YYYY-MM-DDTHH:mm` |
| `hour` | Hourly data | `YYYY-MM-DDTHH` |
| `daily` | Daily data | `YYYY-MM-DD` |
| `weekly` | Weekly data | `YYYY-Www` |
| `monthly` | Monthly data | `YYYY-MM` |
| `quarterly` | Quarterly data | `YYYY-Qq` |
| `yearly` | Yearly data | `YYYY` |

### 3.3 Aggregated Time Series

For multi-metric time series:

```json
{
  "data": [
    {
      "timestamp": "2025-01-01T00:00:00Z",
      "metrics": {
        "min": 100,
        "max": 200,
        "avg": 150,
        "median": 145,
        "stddev": 25,
        "count": 1000
      }
    }
  ]
}
```

---

## 4. Hierarchical Data

### 4.1 Tree Structure

For tree visualizations (treemap, sunburst, dendrogram):

```json
{
  "name": "Root",
  "value": 1000,
  "children": [
    {
      "name": "Category A",
      "value": 600,
      "children": [
        { "name": "A1", "value": 300 },
        { "name": "A2", "value": 300 }
      ]
    },
    {
      "name": "Category B",
      "value": 400,
      "children": [
        { "name": "B1", "value": 200 },
        { "name": "B2", "value": 200 }
      ]
    }
  ]
}
```

### 4.2 Network/Graph Data

For network visualizations:

```json
{
  "nodes": [
    { "id": "node1", "label": "Node 1", "group": 1, "size": 10 },
    { "id": "node2", "label": "Node 2", "group": 1, "size": 15 },
    { "id": "node3", "label": "Node 3", "group": 2, "size": 20 }
  ],
  "edges": [
    { "source": "node1", "target": "node2", "weight": 1.5 },
    { "source": "node2", "target": "node3", "weight": 2.0 }
  ]
}
```

---

## 5. Geospatial Data

### 5.1 GeoJSON Compliance

All geospatial data MUST comply with GeoJSON (RFC 7946):

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [-122.4194, 37.7749]
      },
      "properties": {
        "name": "San Francisco",
        "population": 883305,
        "density": 18838
      }
    }
  ]
}
```

### 5.2 Supported Geometry Types

- `Point`: Single location
- `LineString`: Connected line
- `Polygon`: Closed area
- `MultiPoint`: Multiple points
- `MultiLineString`: Multiple lines
- `MultiPolygon`: Multiple polygons
- `GeometryCollection`: Mixed geometries

---

## 6. Metadata Specification

### 6.1 Required Metadata

All datasets MUST include:

```json
{
  "metadata": {
    "version": "1.0",
    "created": "2025-12-26T10:30:00Z",
    "source": "data-source-identifier"
  }
}
```

### 6.2 Optional Metadata

```json
{
  "metadata": {
    "title": "Human-readable title",
    "description": "Detailed description",
    "author": "Author name or organization",
    "license": "License identifier (e.g., CC-BY-4.0, MIT)",
    "tags": ["tag1", "tag2"],
    "language": "en",
    "update_frequency": "daily",
    "last_updated": "2025-12-26T10:30:00Z",
    "contact": "email@example.com",
    "homepage": "https://example.com/dataset"
  }
}
```

---

## 7. Validation Rules

### 7.1 Data Integrity

- All timestamps MUST be in ISO 8601 format or Unix timestamp
- Numeric fields MUST NOT contain non-numeric values
- Required fields MUST NOT be null unless explicitly allowed
- Field names MUST be unique within a schema
- Date ranges MUST have `start <= end`

### 7.2 Schema Validation

Example JSON Schema for validation:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["metadata", "schema", "data"],
  "properties": {
    "metadata": {
      "type": "object",
      "required": ["version", "created", "source"]
    },
    "schema": {
      "type": "object",
      "required": ["fields"]
    },
    "data": {
      "type": "array",
      "minItems": 1
    }
  }
}
```

### 7.3 File Size Limits

| Format | Recommended Max Size | Hard Limit |
|--------|---------------------|------------|
| JSON | 10 MB | 50 MB |
| CSV | 50 MB | 200 MB |
| Parquet | 1 GB | 10 GB |

For larger datasets, use pagination or streaming.

---

## Appendix A: MIME Types

```
application/json
text/csv
application/octet-stream (Parquet)
application/vnd.apache.arrow.stream
application/geo+json
```

## Appendix B: References

- [JSON Specification](https://www.json.org/)
- [GeoJSON RFC 7946](https://tools.ietf.org/html/rfc7946)
- [Apache Parquet](https://parquet.apache.org/)
- [Apache Arrow](https://arrow.apache.org/)
- [ISO 8601 Date/Time](https://www.iso.org/iso-8601-date-and-time-format.html)

---

**© 2025 WIA (World Certification Industry Association)**
**Standard:** WIA-DATA-011
**Phase:** 1 - Data Format Specification
