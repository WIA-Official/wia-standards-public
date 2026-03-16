# WIA-CORE-008: Universal Metadata - PHASE 3 Specification

**Version:** 1.0.0
**Status:** Stable
**Last Updated:** December 2025
**Phase:** Extensions & Domain-Specific Features

## Overview

PHASE 3 enables domain-specific extensions, advanced search capabilities, and specialized features. This phase maintains the core interoperability of PHASE 1-2 while allowing unlimited customization for specific use cases.

## 3.1 Extension Mechanism

### Namespaced Extensions

Domain-specific fields use namespace prefixes to avoid conflicts:

**Format:** `namespace:field_name`

```json
{
  "id": "https://hospital.org/records/12345",
  "title": "Patient Clinical Trial Results",
  "domain": "healthcare",

  "medical:patient_id": "P-789012",
  "medical:trial_id": "NCT04567890",
  "medical:trial_phase": "III",
  "medical:enrollment": 1500,
  "medical:primary_endpoint": "Overall survival at 24 months",
  "medical:adverse_events": "Grade 3-4: 12%",

  "research:methodology": "Double-blind randomized controlled trial",
  "research:statistical_power": 0.95,
  "research:p_value": 0.0032
}
```

### Custom Namespaces

Organizations can register custom namespaces:

```json
{
  "@namespaces": {
    "acme": "https://acme.com/metadata/schema/v1",
    "museum": "https://museum-standards.org/metadata/v2"
  },

  "acme:internal_code": "ACME-2025-001",
  "acme:approval_status": "reviewed",
  "acme:business_unit": "research-division"
}
```

## 3.2 Domain-Specific Schemas

### Healthcare Domain

```json
{
  "domain": "healthcare",

  "medical:record_type": "clinical_trial|diagnosis|imaging|prescription",
  "medical:patient_id": "string",
  "medical:provider_id": "string",
  "medical:diagnosis_codes": ["ICD-10 codes"],
  "medical:procedure_codes": ["CPT codes"],
  "medical:medications": ["medication list"],
  "medical:allergies": ["allergy list"],
  "medical:confidentiality_level": "public|internal|restricted|confidential",
  "medical:hipaa_compliant": true,

  "qualityMetrics": {
    "medical:data_accuracy": 0.98,
    "medical:completeness_score": 0.95,
    "medical:peer_review_status": "reviewed|pending|not-reviewed"
  }
}
```

### Research Domain

```json
{
  "domain": "research",

  "research:methodology": "string",
  "research:sample_size": "integer",
  "research:variables": ["list of variables"],
  "research:instruments": ["list of instruments/tools"],
  "research:data_collection_period": {
    "start": "ISO 8601 date",
    "end": "ISO 8601 date"
  },
  "research:funding_source": ["funding organizations"],
  "research:ethics_approval": {
    "approved": true,
    "board": "IRB name",
    "approval_number": "string",
    "approval_date": "ISO 8601 date"
  },
  "research:data_availability": "open|restricted|on-request|closed",
  "research:preregistration": "URL to preregistration",

  "qualityMetrics": {
    "research:statistical_power": 0.95,
    "research:replication_index": 0.88,
    "research:transparency_score": 0.92
  }
}
```

### Museum/Cultural Heritage Domain

```json
{
  "domain": "culture",

  "museum:accession_number": "string",
  "museum:object_type": "artifact|artwork|specimen|document",
  "museum:materials": ["list of materials"],
  "museum:dimensions": {
    "height": {"value": 30, "unit": "cm"},
    "width": {"value": 20, "unit": "cm"},
    "depth": {"value": 5, "unit": "cm"},
    "weight": {"value": 500, "unit": "g"}
  },
  "museum:period": "string",
  "museum:culture": "string",
  "museum:provenance": "detailed history",
  "museum:conservation_status": "excellent|good|fair|poor",
  "museum:exhibition_history": ["list of exhibitions"],
  "museum:rights_holder": "organization or person",
  "museum:digital_surrogates": ["URLs to images/3D scans"]
}
```

### Education Domain

```json
{
  "domain": "education",

  "edu:level": "primary|secondary|undergraduate|graduate|professional",
  "edu:subject_areas": ["mathematics", "science", "literature"],
  "edu:learning_objectives": ["list of objectives"],
  "edu:prerequisites": ["list of prerequisites"],
  "edu:duration": {
    "value": 14,
    "unit": "weeks"
  },
  "edu:credit_hours": 3,
  "edu:assessment_methods": ["exam", "project", "participation"],
  "edu:instructional_methods": ["lecture", "lab", "discussion"],
  "edu:accessibility_features": ["closed-captions", "transcripts", "alt-text"],
  "edu:alignment": {
    "standard": "Common Core",
    "code": "CCSS.MATH.CONTENT.8.G.A.1"
  }
}
```

## 3.3 Advanced Search Features

### Faceted Search Configuration

```json
{
  "searchConfiguration": {
    "facets": [
      {
        "field": "domain",
        "label": "Subject Domain",
        "type": "terms",
        "size": 20
      },
      {
        "field": "contentType",
        "label": "Content Type",
        "type": "terms"
      },
      {
        "field": "created",
        "label": "Creation Date",
        "type": "date_range",
        "ranges": [
          {"to": "now-1M", "label": "Last Month"},
          {"from": "now-1M", "to": "now-1y", "label": "Last Year"},
          {"from": "now-1y", "label": "Older"}
        ]
      },
      {
        "field": "qualityMetrics.overall",
        "label": "Quality Score",
        "type": "range",
        "ranges": [
          {"from": 0.9, "label": "Excellent (0.9+)"},
          {"from": 0.7, "to": 0.9, "label": "Good (0.7-0.9)"},
          {"to": 0.7, "label": "Needs Improvement (<0.7)"}
        ]
      }
    ]
  }
}
```

### Semantic Search Support

```json
{
  "semanticMetadata": {
    "embedding_model": "text-embedding-3-large",
    "embedding_dimensions": 1536,
    "embedding_generated_at": "2025-01-15T10:00:00Z",
    "embedding": [0.0234, -0.0123, ...],

    "entities_extracted": [
      {
        "text": "quantum computing",
        "type": "CONCEPT",
        "confidence": 0.95
      },
      {
        "text": "Example University",
        "type": "ORGANIZATION",
        "confidence": 0.98
      }
    ],

    "topics": [
      {"label": "Quantum Physics", "score": 0.92},
      {"label": "Computer Science", "score": 0.88}
    ]
  }
}
```

## 3.4 Geospatial Metadata

### Point Location

```json
{
  "geospatial": {
    "type": "Point",
    "coordinates": {
      "latitude": 37.7749,
      "longitude": -122.4194
    },
    "coordinateSystem": "WGS84",
    "placeName": "San Francisco, California, USA",
    "precision": "city"
  }
}
```

### Bounding Box

```json
{
  "geospatial": {
    "type": "BoundingBox",
    "coordinates": {
      "north": 49.0,
      "south": 24.5,
      "east": -66.9,
      "west": -125.0
    },
    "coordinateSystem": "WGS84",
    "placeName": "Continental United States"
  }
}
```

### Polygon

```json
{
  "geospatial": {
    "type": "Polygon",
    "coordinates": [
      [
        [-122.4194, 37.7749],
        [-122.4084, 37.7849],
        [-122.4084, 37.7649],
        [-122.4194, 37.7749]
      ]
    ],
    "coordinateSystem": "WGS84"
  }
}
```

## 3.5 Temporal Metadata

### Time Period

```json
{
  "temporal": {
    "type": "Period",
    "start": "2024-01-01T00:00:00Z",
    "end": "2024-12-31T23:59:59Z",
    "label": "Academic Year 2024"
  }
}
```

### Historical Dating

```json
{
  "temporal": {
    "type": "HistoricalPeriod",
    "period": "Ming Dynasty",
    "circa": true,
    "year": 1450,
    "uncertainty": 50,
    "calendar": "Gregorian"
  }
}
```

## 3.6 Provenance Tracking

### W3C PROV Compatible

```json
{
  "provenance": {
    "entity": "https://example.org/dataset-v2",
    "wasGeneratedBy": {
      "activity": "data-processing-pipeline-v3",
      "time": "2025-01-15T10:00:00Z",
      "agent": {
        "name": "Data Processing Service",
        "type": "SoftwareAgent",
        "version": "3.2.1"
      }
    },
    "wasDerivedFrom": [
      "https://example.org/raw-data-source-1",
      "https://example.org/raw-data-source-2"
    ],
    "wasAttributedTo": {
      "name": "Dr. Jane Smith",
      "type": "Person",
      "role": "Principal Investigator"
    },
    "generatedAtTime": "2025-01-15T10:00:00Z",
    "invalidatedAtTime": null
  }
}
```

## 3.7 Access Control

### Fine-Grained Permissions

```json
{
  "accessControl": {
    "visibility": "restricted",
    "permissions": {
      "read": ["group:researchers", "user:alice@example.com"],
      "write": ["group:editors", "user:alice@example.com"],
      "delete": ["user:admin@example.com"],
      "share": ["group:editors"]
    },
    "embargo": {
      "active": true,
      "liftDate": "2026-01-01T00:00:00Z",
      "reason": "Publisher embargo period"
    },
    "conditions": {
      "requireAuthentication": true,
      "requireMFA": false,
      "ipWhitelist": ["10.0.0.0/8", "192.168.1.0/24"],
      "timeRestrictions": {
        "allowedHours": "09:00-17:00",
        "allowedDays": ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday"],
        "timezone": "America/New_York"
      }
    }
  }
}
```

## 3.8 Version Control

### Detailed Version Information

```json
{
  "versionInfo": {
    "current": "2.1.0",
    "previous": "2.0.3",
    "versionHistory": [
      {
        "version": "2.1.0",
        "releaseDate": "2025-01-15T00:00:00Z",
        "changes": "Added new chapter on quantum algorithms",
        "author": "Dr. Jane Smith"
      },
      {
        "version": "2.0.3",
        "releaseDate": "2024-12-01T00:00:00Z",
        "changes": "Fixed typos in chapter 3",
        "author": "Editor Team"
      }
    ],
    "changeLog": "https://example.org/changelog.md",
    "deprecated": false,
    "successor": null
  }
}
```

## 3.9 Complete Extended Example

```json
{
  "standard": "WIA-CORE-008",
  "version": "1.0",

  "id": "https://hospital.org/trials/trial-2025-001",
  "title": "Phase III Clinical Trial: Novel Cancer Treatment",
  "language": "en",
  "domain": "healthcare",
  "created": "2024-06-01T08:00:00Z",
  "modified": "2025-01-15T14:30:00Z",

  "description": "Multi-center phase III randomized controlled trial evaluating efficacy and safety of experimental treatment ABC-123 versus standard of care in patients with advanced colorectal cancer.",

  "keywords": ["clinical trial", "cancer treatment", "phase III", "RCT", "colorectal cancer"],

  "medical:trial_id": "NCT04567890",
  "medical:trial_phase": "III",
  "medical:trial_status": "recruiting",
  "medical:enrollment_target": 1500,
  "medical:enrollment_current": 834,
  "medical:principal_investigator": "Dr. Sarah Chen, MD PhD",
  "medical:sponsor": "Example Pharma Inc",
  "medical:primary_endpoint": "Overall survival at 24 months",
  "medical:secondary_endpoints": [
    "Progression-free survival",
    "Quality of life assessment",
    "Adverse event profile"
  ],

  "research:methodology": "Multi-center, double-blind, randomized controlled trial",
  "research:sample_size": 1500,
  "research:statistical_power": 0.90,
  "research:ethics_approval": {
    "approved": true,
    "board": "Central IRB",
    "approval_number": "IRB-2024-001",
    "approval_date": "2024-05-15"
  },

  "geospatial": {
    "type": "MultiPoint",
    "sites": [
      {"latitude": 37.7749, "longitude": -122.4194, "name": "Site 1: San Francisco"},
      {"latitude": 40.7128, "longitude": -74.0060, "name": "Site 2: New York"},
      {"latitude": 34.0522, "longitude": -118.2437, "name": "Site 3: Los Angeles"}
    ]
  },

  "temporal": {
    "type": "Period",
    "start": "2024-06-01T00:00:00Z",
    "plannedEnd": "2027-06-01T00:00:00Z",
    "label": "Trial Duration"
  },

  "accessControl": {
    "visibility": "restricted",
    "permissions": {
      "read": ["group:trial-investigators", "group:sponsors"],
      "write": ["group:data-managers"],
      "delete": ["user:principal-investigator"]
    },
    "dataUseAgreement": "https://hospital.org/dua/trial-2025-001"
  },

  "qualityMetrics": {
    "completeness": 0.96,
    "accuracy": 0.99,
    "consistency": 0.98,
    "timeliness": 1.0,
    "overall": 0.98,
    "medical:data_quality_score": 0.97,
    "medical:monitoring_status": "on-track"
  }
}
```

## Next Phase

PHASE 4 covers integration patterns, migration strategies, and advanced interoperability. See [PHASE4.md](./PHASE4.md) for details.

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
