# WIA-DATA-015: Graph Database Standard
## PHASE 1: Data Format Specification

**Version:** 1.0  
**Status:** Draft  
**Last Updated:** 2025-01-15

---

## 1. Overview

This document defines the data format specifications for the WIA-DATA-015 Graph Database Standard. It establishes the foundational structures for representing graph data, including nodes, edges, properties, and metadata.

## 2. Graph Data Model

### 2.1 Property Graph Model

The WIA-DATA-015 standard adopts the property graph model as its primary data representation:

- **Nodes (Vertices):** Discrete entities with optional labels and properties
- **Edges (Relationships):** Directed connections between nodes with type and properties
- **Properties:** Key-value pairs attached to nodes and edges
- **Labels:** Categories or types for nodes (multi-label supported)

### 2.2 Data Types

#### Primitive Types
- `string`: UTF-8 encoded text
- `integer`: 64-bit signed integer
- `float`: 64-bit floating point
- `boolean`: true/false
- `null`: Absence of value

#### Temporal Types
- `date`: ISO 8601 date (YYYY-MM-DD)
- `time`: ISO 8601 time with timezone
- `datetime`: ISO 8601 combined date and time
- `duration`: ISO 8601 duration

#### Collection Types
- `list`: Ordered collection of values
- `map`: Key-value pairs (object/dictionary)

#### Spatial Types
- `point`: Geographic or cartesian coordinates
- `polygon`: Bounded region

## 3. Node Specification

### 3.1 Node Structure

```json
{
  "id": "node_001",
  "labels": ["Person", "Employee"],
  "properties": {
    "name": "Alice Johnson",
    "email": "alice@example.com",
    "age": 30,
    "joinedDate": "2020-01-15",
    "verified": true
  },
  "metadata": {
    "created": "2024-01-15T10:30:00Z",
    "updated": "2024-06-20T14:22:00Z",
    "version": 3
  }
}
```

### 3.2 Node Requirements

- **ID:** Unique identifier (string or integer, immutable)
- **Labels:** Array of zero or more label strings
- **Properties:** Object with key-value pairs
- **Metadata:** System-managed tracking information (optional)

### 3.3 Label Conventions

- Use PascalCase for labels: `Person`, `Product`, `Company`
- Labels should be singular nouns
- Multiple labels allowed: `["Person", "Employee", "Manager"]`
- Reserved labels start with underscore: `_System`, `_Temp`

## 4. Edge Specification

### 4.1 Edge Structure

```json
{
  "id": "edge_001",
  "type": "WORKS_FOR",
  "source": "node_001",
  "target": "node_002",
  "properties": {
    "since": "2020-01-15",
    "role": "Software Engineer",
    "department": "Engineering"
  },
  "metadata": {
    "created": "2024-01-15T10:35:00Z",
    "weight": 1.0
  }
}
```

### 4.2 Edge Requirements

- **ID:** Unique identifier (optional, can be auto-generated)
- **Type:** Single relationship type (required)
- **Source:** ID of source node (required)
- **Target:** ID of target node (required)
- **Direction:** Always directed (bidirectional traversal allowed)
- **Properties:** Object with key-value pairs (optional)
- **Metadata:** System-managed information (optional)

### 4.3 Relationship Type Conventions

- Use UPPER_SNAKE_CASE: `WORKS_FOR`, `KNOWS`, `PURCHASED`
- Should be verb phrases indicating the relationship
- Always directional: source→type→target
- Reserved types start with underscore: `_SYSTEM_LINK`

## 5. Property Specifications

### 5.1 Property Naming

- Use camelCase: `firstName`, `emailAddress`, `createdAt`
- Avoid reserved keywords: `id`, `type`, `labels`
- Length: 1-64 characters
- Characters: alphanumeric, underscore, no spaces

### 5.2 Property Constraints

```json
{
  "propertyName": "email",
  "dataType": "string",
  "required": false,
  "unique": true,
  "indexed": true,
  "validation": {
    "pattern": "^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}$",
    "maxLength": 255
  }
}
```

### 5.3 Property Validation Rules

- **String:** maxLength, minLength, pattern (regex)
- **Integer/Float:** min, max, precision
- **List:** maxItems, minItems, itemType
- **Map:** requiredKeys, allowedKeys

## 6. Serialization Formats

### 6.1 JSON Representation

Primary serialization format for data exchange:

```json
{
  "graph": {
    "nodes": [
      {
        "id": "1",
        "labels": ["Person"],
        "properties": {"name": "Alice", "age": 30}
      },
      {
        "id": "2",
        "labels": ["Company"],
        "properties": {"name": "Acme Corp"}
      }
    ],
    "edges": [
      {
        "id": "e1",
        "type": "WORKS_FOR",
        "source": "1",
        "target": "2",
        "properties": {"since": "2020-01-01"}
      }
    ]
  }
}
```

### 6.2 CSV Representation

For bulk import/export:

**Nodes CSV:**
```csv
:ID,name:string,age:int,:LABEL
1,Alice,30,Person
2,Acme Corp,,Company
```

**Edges CSV:**
```csv
:START_ID,:END_ID,:TYPE,since:date
1,2,WORKS_FOR,2020-01-01
```

### 6.3 GraphML Representation

XML-based format for tool interoperability:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<graphml xmlns="http://graphml.graphdrawing.org/xmlns">
  <graph id="G" edgedefault="directed">
    <node id="1">
      <data key="label">Person</data>
      <data key="name">Alice</data>
      <data key="age">30</data>
    </node>
    <edge id="e1" source="1" target="2">
      <data key="type">WORKS_FOR</data>
      <data key="since">2020-01-01</data>
    </edge>
  </graph>
</graphml>
```

## 7. Schema Definition

### 7.1 Schema Format

```json
{
  "schema": {
    "version": "1.0",
    "nodeTypes": [
      {
        "label": "Person",
        "properties": {
          "name": {"type": "string", "required": true},
          "email": {"type": "string", "unique": true},
          "age": {"type": "integer", "min": 0}
        }
      }
    ],
    "edgeTypes": [
      {
        "type": "WORKS_FOR",
        "sourceLabel": "Person",
        "targetLabel": "Company",
        "properties": {
          "since": {"type": "date"},
          "role": {"type": "string"}
        }
      }
    ],
    "constraints": [
      {
        "type": "unique",
        "nodeLabel": "Person",
        "property": "email"
      },
      {
        "type": "existence",
        "nodeLabel": "Person",
        "property": "name"
      }
    ]
  }
}
```

## 8. Metadata and Versioning

### 8.1 Graph Metadata

```json
{
  "metadata": {
    "graphId": "graph_001",
    "name": "Corporate Network",
    "version": "2.1.0",
    "created": "2024-01-01T00:00:00Z",
    "modified": "2024-06-15T10:30:00Z",
    "nodeCount": 1500,
    "edgeCount": 3200,
    "schema": "schema_v1.json",
    "description": "Employee and organizational relationships"
  }
}
```

### 8.2 Versioning Strategy

- **Semantic Versioning:** MAJOR.MINOR.PATCH
- **Change Tracking:** Track schema and data versions independently
- **Migration:** Provide upgrade paths between versions
- **Compatibility:** Maintain backward compatibility within major versions

## 9. Data Integrity

### 9.1 Referential Integrity

- All edges must reference existing nodes
- Node deletion requires orphan edge handling:
  - `CASCADE`: Delete dependent edges
  - `RESTRICT`: Prevent deletion if edges exist
  - `SET_NULL`: Invalid for graphs (edges require endpoints)

### 9.2 Constraints

```json
{
  "constraints": {
    "uniqueness": [
      {"nodeLabel": "User", "property": "username"},
      {"nodeLabel": "Product", "property": "sku"}
    ],
    "existence": [
      {"nodeLabel": "Person", "property": "name"},
      {"edgeType": "PURCHASED", "property": "date"}
    ],
    "nodeKey": [
      {"nodeLabel": "Person", "properties": ["firstName", "lastName", "birthDate"]}
    ]
  }
}
```

## 10. File Format Specifications

### 10.1 WIA Graph Format (.wgf)

Binary format for efficient storage and transmission:

```
Header (32 bytes):
- Magic number: 0x57474631 ('WGF1')
- Version: uint16
- Flags: uint16
- Node count: uint64
- Edge count: uint64
- Property count: uint64

Node Block:
- Node ID: varint
- Label count: uint8
- Labels: [string]
- Property count: uint16
- Properties: [key:string, type:uint8, value:bytes]

Edge Block:
- Edge ID: varint
- Type: string
- Source: varint
- Target: varint
- Property count: uint16
- Properties: [key:string, type:uint8, value:bytes]
```

## 11. Compliance

### 11.1 Required Features

Implementations must support:
- Property graph model
- All primitive data types
- JSON serialization
- Basic constraints (unique, existence)
- Referential integrity

### 11.2 Optional Features

- Binary format support (.wgf)
- GraphML import/export
- Advanced spatial types
- Custom validation rules
- Schema evolution tools

## 12. Security Considerations

### 12.1 Data Protection

- Property encryption support for sensitive fields
- Redaction of PII in exports
- Access control metadata on nodes/edges

### 12.2 Validation

- Input sanitization for injection prevention
- Schema validation before import
- Size limits to prevent DoS

---

## Appendix A: Examples

See `examples/data-format/` directory for complete examples of:
- Node and edge definitions
- Schema files
- Import/export samples
- Validation rules

## Appendix B: References

- ISO/IEC 39075 GQL Standard (draft)
- Property Graph Schema Working Group
- Neo4j Data Format Documentation
- Apache TinkerPop Data Model

---

**License:** CC BY 4.0  
**Maintained by:** WIA Standards Committee  
**Contact:** standards@wia-official.org
