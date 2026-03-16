# WIA-DATA-008: Data Lineage Standard
## PHASE 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This phase defines the core data formats and structures for representing data lineage metadata according to the WIA-DATA-008 standard.

## 1. Core Data Structures

### 1.1 Lineage Node

Represents a data asset in the lineage graph.

```json
{
  "node_id": "string (uuid)",
  "node_type": "table | view | column | report | model | file | stream",
  "namespace": "string",
  "name": "string",
  "qualified_name": "string",
  "created_at": "ISO8601 timestamp",
  "updated_at": "ISO8601 timestamp",
  "metadata": {
    "description": "string",
    "owner": "string",
    "tags": ["string"],
    "custom": {}
  },
  "schema": {
    "fields": [
      {
        "name": "string",
        "type": "string",
        "nullable": "boolean",
        "description": "string"
      }
    ]
  }
}
```

**Field Descriptions:**

- `node_id`: Unique identifier (UUID v4)
- `node_type`: Type of data asset
- `namespace`: Logical grouping (e.g., "postgres://warehouse:5432", "s3://bucket")
- `name`: Simple name of the asset
- `qualified_name`: Fully qualified name (namespace + name)
- `created_at`: When the asset was created
- `updated_at`: When the metadata was last updated
- `metadata`: Additional metadata
- `schema`: Schema definition (optional for non-structured data)

### 1.2 Lineage Edge

Represents a transformation or dependency between nodes.

```json
{
  "edge_id": "string (uuid)",
  "source_node_id": "string (uuid)",
  "target_node_id": "string (uuid)",
  "edge_type": "derived_from | aggregated_from | joined_with | filtered_from | projected_from",
  "transformation": {
    "type": "sql | python | spark | custom",
    "logic": "string (SQL, code, or description)",
    "version": "string",
    "git_commit": "string"
  },
  "metadata": {
    "job_name": "string",
    "execution_time": "ISO8601 timestamp",
    "duration_ms": "integer",
    "records_processed": "integer",
    "custom": {}
  },
  "created_at": "ISO8601 timestamp"
}
```

**Edge Types:**

- `derived_from`: Target is derived from source
- `aggregated_from`: Target is aggregation of source
- `joined_with`: Target is join of source with others
- `filtered_from`: Target is filtered subset of source
- `projected_from`: Target is projection (column selection) of source

### 1.3 Lineage Graph

Complete lineage representation.

```json
{
  "graph_id": "string (uuid)",
  "version": "string",
  "generated_at": "ISO8601 timestamp",
  "nodes": [
    "...Lineage Node objects..."
  ],
  "edges": [
    "...Lineage Edge objects..."
  ],
  "metadata": {
    "producer": "string",
    "environment": "dev | staging | prod",
    "custom": {}
  }
}
```

## 2. Column-Level Lineage

For detailed column-level tracking:

```json
{
  "column_lineage_id": "string (uuid)",
  "source_table": "string (qualified name)",
  "source_column": "string",
  "target_table": "string (qualified name)",
  "target_column": "string",
  "transformation_type": "direct | computed | aggregated | joined",
  "transformation_logic": "string",
  "dependencies": [
    {
      "source_table": "string",
      "source_column": "string",
      "expression": "string"
    }
  ],
  "created_at": "ISO8601 timestamp"
}
```

**Transformation Types:**

- `direct`: Direct copy (SELECT a AS b)
- `computed`: Computed from expression (SELECT a + b AS c)
- `aggregated`: Aggregation (SELECT SUM(a) AS total)
- `joined`: Result of join operation

## 3. Provenance Metadata

Extended provenance information:

```json
{
  "provenance_id": "string (uuid)",
  "node_id": "string (uuid)",
  "event_type": "created | modified | accessed | deleted",
  "event_time": "ISO8601 timestamp",
  "agent": {
    "type": "user | service | job",
    "id": "string",
    "name": "string"
  },
  "operation": {
    "type": "read | write | transform | export",
    "method": "string",
    "parameters": {}
  },
  "context": {
    "purpose": "string",
    "legal_basis": "string (for GDPR)",
    "retention_period": "string",
    "sensitivity_level": "public | internal | confidential | restricted"
  }
}
```

## 4. Data Quality Annotations

Quality metadata for lineage nodes:

```json
{
  "quality_id": "string (uuid)",
  "node_id": "string (uuid)",
  "measured_at": "ISO8601 timestamp",
  "metrics": {
    "completeness": "float (0-1)",
    "accuracy": "float (0-1)",
    "consistency": "float (0-1)",
    "timeliness": "float (0-1)"
  },
  "validations": [
    {
      "rule_name": "string",
      "rule_type": "not_null | unique | range | pattern | custom",
      "passed": "boolean",
      "failed_count": "integer",
      "details": "string"
    }
  ],
  "anomalies": [
    {
      "type": "outlier | missing | duplicate | invalid",
      "severity": "low | medium | high | critical",
      "count": "integer",
      "description": "string"
    }
  ]
}
```

## 5. OpenLineage Compatibility

WIA-DATA-008 is compatible with OpenLineage standard:

```json
{
  "eventType": "START | COMPLETE | FAIL | ABORT",
  "eventTime": "ISO8601 timestamp",
  "run": {
    "runId": "string (uuid)",
    "facets": {}
  },
  "job": {
    "namespace": "string",
    "name": "string",
    "facets": {}
  },
  "inputs": [
    {
      "namespace": "string",
      "name": "string",
      "facets": {}
    }
  ],
  "outputs": [
    {
      "namespace": "string",
      "name": "string",
      "facets": {}
    }
  ],
  "producer": "https://wia.standards/DATA-008/v1.0.0"
}
```

## 6. File Formats

### 6.1 JSON

Primary format for API exchange and storage.

**Naming:** `lineage_graph_<timestamp>.json`

### 6.2 JSON Lines (JSONL)

For streaming and incremental updates.

**Naming:** `lineage_events_<date>.jsonl`

Each line is a complete lineage event.

### 6.3 Parquet

For efficient storage and analytics.

**Schema:**
- Nodes table: `lineage_nodes.parquet`
- Edges table: `lineage_edges.parquet`
- Provenance table: `lineage_provenance.parquet`

### 6.4 GraphML

For import/export to graph visualization tools.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<graphml xmlns="http://graphml.graphdrawing.org/xmlns">
  <graph id="lineage" edgedefault="directed">
    <node id="node1">
      <data key="type">table</data>
      <data key="name">customers</data>
    </node>
    <edge source="node1" target="node2">
      <data key="type">derived_from</data>
    </edge>
  </graph>
</graphml>
```

## 7. Validation Rules

All lineage data MUST:

1. **Uniqueness**: Each node and edge has unique ID
2. **Referential Integrity**: Edge source/target must reference existing nodes
3. **Timestamps**: All timestamps in UTC ISO8601 format
4. **Required Fields**: node_id, node_type, name, namespace
5. **DAG Property**: Lineage graph must be acyclic
6. **Namespace Format**: Must follow URI format

## 8. Compression and Encoding

- **Compression**: gzip for JSON files
- **Encoding**: UTF-8 for all text
- **Timestamps**: ISO8601 with timezone
- **Numbers**: JSON number type (IEEE 754 double)

## 9. Versioning

Lineage data format follows semantic versioning:

- **Major**: Breaking changes to structure
- **Minor**: Backward-compatible additions
- **Patch**: Clarifications and fixes

Current version: `1.0.0`

## 10. Examples

### Example 1: Simple Table Lineage

```json
{
  "graph_id": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "generated_at": "2025-12-26T10:00:00Z",
  "nodes": [
    {
      "node_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "node_type": "table",
      "namespace": "postgres://warehouse:5432",
      "name": "raw_customers",
      "qualified_name": "postgres://warehouse:5432/public.raw_customers",
      "created_at": "2025-01-01T00:00:00Z",
      "updated_at": "2025-12-26T09:00:00Z"
    },
    {
      "node_id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
      "node_type": "table",
      "namespace": "postgres://warehouse:5432",
      "name": "cleaned_customers",
      "qualified_name": "postgres://warehouse:5432/public.cleaned_customers",
      "created_at": "2025-01-01T00:00:00Z",
      "updated_at": "2025-12-26T10:00:00Z"
    }
  ],
  "edges": [
    {
      "edge_id": "c3d4e5f6-a7b8-9012-cdef-123456789012",
      "source_node_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
      "target_node_id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
      "edge_type": "derived_from",
      "transformation": {
        "type": "sql",
        "logic": "SELECT * FROM raw_customers WHERE active = true",
        "version": "1.0"
      },
      "created_at": "2025-12-26T10:00:00Z"
    }
  ]
}
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
