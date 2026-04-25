# WIA AI-TRAINING-DATA - PHASE 2: Technical Architecture

## Version 1.0

> 홍익인간 (弘益人間) - Benefit All Humanity

## Document Information

- **Standard**: WIA-AI-TRAINING-DATA
- **Phase**: 2 - Technical Architecture
- **Version**: 1.0.0
- **Status**: Draft
- **Date**: 2026-01-13
- **Authors**: WIA Standards Committee

---

## 📋 Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Data Card Schema](#2-data-card-schema)
3. [Data Quality Architecture](#3-data-quality-architecture)
4. [Provenance System](#4-provenance-system)
5. [Storage Architecture](#5-storage-architecture)
6. [API Specifications](#6-api-specifications)
7. [Security Architecture](#7-security-architecture)
8. [Integration Patterns](#8-integration-patterns)

---

## 1. System Architecture

### 1.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    AI TRAINING DATA MANAGEMENT SYSTEM                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                        USER INTERFACE LAYER                           │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐     │   │
│  │  │  Web UI    │  │   CLI      │  │  API       │  │  SDK       │     │   │
│  │  │  Portal    │  │  Tools     │  │  Gateway   │  │  Libraries │     │   │
│  │  └────────────┘  └────────────┘  └────────────┘  └────────────┘     │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                      │                                       │
│  ┌───────────────────────────────────▼──────────────────────────────────┐   │
│  │                        SERVICE LAYER                                  │   │
│  │                                                                       │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │   │
│  │  │  Catalog    │  │  Quality    │  │  Provenance │  │   Bias      │ │   │
│  │  │  Service    │  │  Service    │  │  Service    │  │   Service   │ │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │   │
│  │                                                                       │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │   │
│  │  │  Access     │  │  Transform  │  │  Validation │  │  Analytics  │ │   │
│  │  │  Control    │  │  Service    │  │  Service    │  │  Service    │ │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │   │
│  └───────────────────────────────────────────────────────────────────────┘   │
│                                      │                                       │
│  ┌───────────────────────────────────▼──────────────────────────────────┐   │
│  │                        DATA LAYER                                     │   │
│  │                                                                       │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │   │
│  │  │  Metadata   │  │  Object     │  │  Lineage    │  │  Metrics    │ │   │
│  │  │  Store      │  │  Storage    │  │  Graph      │  │  Store      │ │   │
│  │  │ (PostgreSQL)│  │  (S3/MinIO) │  │  (Neo4j)    │  │(TimescaleDB)│ │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │   │
│  └───────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Component Descriptions

| Component | Purpose | Technology |
|-----------|---------|------------|
| Catalog Service | Dataset discovery and metadata management | Go + PostgreSQL |
| Quality Service | Data quality assessment and scoring | Python + scikit-learn |
| Provenance Service | Lineage tracking and verification | Go + Neo4j |
| Bias Service | Bias detection and reporting | Python + fairlearn |
| Access Control | Authentication and authorization | Go + OPA |
| Transform Service | Data transformation pipelines | Python + Apache Beam |
| Validation Service | Schema and quality validation | Go + JSON Schema |
| Analytics Service | Usage and quality analytics | Python + Pandas |

### 1.3 Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         DATA FLOW ARCHITECTURE                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│    DATA INGESTION                                                        │
│    ══════════════                                                        │
│                                                                          │
│    ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐         │
│    │  Raw    │────►│  Ingest │────►│Validate │────►│  Store  │         │
│    │  Data   │     │  Queue  │     │  Check  │     │         │         │
│    └─────────┘     └─────────┘     └─────────┘     └─────────┘         │
│         │                               │                               │
│         │                               ▼                               │
│         │                        ┌─────────────┐                        │
│         │                        │  Provenance │                        │
│         └───────────────────────►│   Capture   │                        │
│                                  └─────────────┘                        │
│                                                                          │
│    QUALITY ASSESSMENT                                                    │
│    ══════════════════                                                    │
│                                                                          │
│    ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐         │
│    │ Dataset │────►│ Sample  │────►│ Compute │────►│ Store   │         │
│    │         │     │         │     │ Metrics │     │ Scores  │         │
│    └─────────┘     └─────────┘     └─────────┘     └─────────┘         │
│                                         │                               │
│                                         ▼                               │
│                                  ┌─────────────┐                        │
│                                  │    Bias     │                        │
│                                  │  Assessment │                        │
│                                  └─────────────┘                        │
│                                                                          │
│    PUBLICATION                                                           │
│    ═══════════                                                           │
│                                                                          │
│    ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐         │
│    │  Data   │────►│  Review │────►│ Approve │────►│ Publish │         │
│    │  Card   │     │         │     │         │     │         │         │
│    └─────────┘     └─────────┘     └─────────┘     └─────────┘         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Data Card Schema

### 2.1 Complete Data Card Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://standards.wia.org/schemas/data-card/v1.0",
  "title": "WIA Data Card Schema",
  "description": "Standard schema for AI training dataset documentation",
  "type": "object",
  "required": [
    "dataCardVersion",
    "datasetId",
    "name",
    "version",
    "description",
    "creators",
    "license",
    "dataTypes",
    "size",
    "createdAt"
  ],
  "properties": {
    "dataCardVersion": {
      "type": "string",
      "const": "1.0",
      "description": "Version of the Data Card schema"
    },
    "datasetId": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for the dataset"
    },
    "name": {
      "type": "string",
      "minLength": 1,
      "maxLength": 255,
      "description": "Human-readable name of the dataset"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Semantic version of the dataset"
    },
    "description": {
      "type": "object",
      "required": ["summary"],
      "properties": {
        "summary": {
          "type": "string",
          "minLength": 50,
          "maxLength": 500,
          "description": "Brief summary of the dataset"
        },
        "purpose": {
          "type": "string",
          "description": "Intended purpose and use cases"
        },
        "content": {
          "type": "string",
          "description": "Detailed description of dataset contents"
        },
        "limitations": {
          "type": "array",
          "items": { "type": "string" },
          "description": "Known limitations and caveats"
        }
      }
    },
    "creators": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "object",
        "required": ["name"],
        "properties": {
          "name": { "type": "string" },
          "organization": { "type": "string" },
          "email": { "type": "string", "format": "email" },
          "role": {
            "type": "string",
            "enum": ["lead", "contributor", "annotator", "curator"]
          }
        }
      }
    },
    "license": {
      "type": "object",
      "required": ["identifier"],
      "properties": {
        "identifier": {
          "type": "string",
          "description": "SPDX license identifier or custom license name"
        },
        "url": {
          "type": "string",
          "format": "uri",
          "description": "URL to full license text"
        },
        "permissions": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["commercial", "research", "modification", "distribution", "private"]
          }
        },
        "conditions": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["attribution", "share-alike", "no-derivatives", "notification"]
          }
        },
        "limitations": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["liability", "warranty", "trademark"]
          }
        }
      }
    },
    "dataTypes": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "string",
        "enum": [
          "text", "image", "audio", "video", "tabular",
          "time-series", "graph", "multimodal", "other"
        ]
      }
    },
    "size": {
      "type": "object",
      "required": ["samples"],
      "properties": {
        "samples": {
          "type": "integer",
          "minimum": 0,
          "description": "Number of samples/examples"
        },
        "features": {
          "type": "integer",
          "minimum": 0,
          "description": "Number of features per sample"
        },
        "fileSize": {
          "type": "object",
          "properties": {
            "value": { "type": "number" },
            "unit": {
              "type": "string",
              "enum": ["B", "KB", "MB", "GB", "TB"]
            }
          }
        },
        "compressed": {
          "type": "boolean",
          "default": false
        }
      }
    },
    "schema": {
      "type": "object",
      "description": "Data schema specification",
      "properties": {
        "features": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["name", "type"],
            "properties": {
              "name": { "type": "string" },
              "type": {
                "type": "string",
                "enum": ["string", "integer", "float", "boolean", "datetime", "binary", "array", "object"]
              },
              "description": { "type": "string" },
              "nullable": { "type": "boolean", "default": false },
              "constraints": {
                "type": "object",
                "properties": {
                  "min": { "type": "number" },
                  "max": { "type": "number" },
                  "pattern": { "type": "string" },
                  "enum": { "type": "array" }
                }
              }
            }
          }
        },
        "labels": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["name", "type"],
            "properties": {
              "name": { "type": "string" },
              "type": { "type": "string" },
              "classes": {
                "type": "array",
                "items": {
                  "type": "object",
                  "properties": {
                    "id": { "type": ["string", "integer"] },
                    "name": { "type": "string" },
                    "description": { "type": "string" }
                  }
                }
              }
            }
          }
        }
      }
    },
    "splits": {
      "type": "object",
      "description": "Dataset split information",
      "properties": {
        "train": {
          "type": "object",
          "properties": {
            "samples": { "type": "integer" },
            "percentage": { "type": "number", "minimum": 0, "maximum": 100 }
          }
        },
        "validation": {
          "type": "object",
          "properties": {
            "samples": { "type": "integer" },
            "percentage": { "type": "number", "minimum": 0, "maximum": 100 }
          }
        },
        "test": {
          "type": "object",
          "properties": {
            "samples": { "type": "integer" },
            "percentage": { "type": "number", "minimum": 0, "maximum": 100 }
          }
        }
      }
    },
    "collection": {
      "type": "object",
      "description": "Data collection methodology",
      "properties": {
        "method": {
          "type": "string",
          "enum": ["manual", "automated", "crowdsourced", "scraped", "generated", "licensed", "mixed"]
        },
        "sources": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": { "type": "string" },
              "url": { "type": "string", "format": "uri" },
              "type": { "type": "string" },
              "accessDate": { "type": "string", "format": "date" }
            }
          }
        },
        "timeframe": {
          "type": "object",
          "properties": {
            "start": { "type": "string", "format": "date" },
            "end": { "type": "string", "format": "date" }
          }
        },
        "geography": {
          "type": "array",
          "items": { "type": "string" },
          "description": "Geographic regions covered"
        },
        "samplingStrategy": {
          "type": "string",
          "description": "How samples were selected"
        }
      }
    },
    "annotation": {
      "type": "object",
      "description": "Annotation process details",
      "properties": {
        "process": {
          "type": "string",
          "enum": ["manual", "automated", "semi-automated", "crowdsourced"]
        },
        "platform": {
          "type": "string",
          "description": "Annotation platform used"
        },
        "guidelines": {
          "type": "string",
          "format": "uri",
          "description": "URL to annotation guidelines"
        },
        "annotators": {
          "type": "object",
          "properties": {
            "count": { "type": "integer" },
            "perSample": { "type": "integer" },
            "qualifications": { "type": "string" },
            "compensation": {
              "type": "object",
              "properties": {
                "type": { "type": "string", "enum": ["hourly", "per-item", "salary"] },
                "rate": { "type": "string" }
              }
            }
          }
        },
        "agreement": {
          "type": "object",
          "properties": {
            "metric": { "type": "string" },
            "value": { "type": "number" }
          }
        }
      }
    },
    "quality": {
      "type": "object",
      "description": "Data quality metrics",
      "properties": {
        "overallScore": {
          "type": "number",
          "minimum": 0,
          "maximum": 10
        },
        "dimensions": {
          "type": "object",
          "properties": {
            "accuracy": { "type": "number", "minimum": 0, "maximum": 1 },
            "completeness": { "type": "number", "minimum": 0, "maximum": 1 },
            "consistency": { "type": "number", "minimum": 0, "maximum": 1 },
            "timeliness": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        },
        "validation": {
          "type": "object",
          "properties": {
            "method": { "type": "string" },
            "date": { "type": "string", "format": "date" },
            "samplesReviewed": { "type": "integer" }
          }
        }
      }
    },
    "bias": {
      "type": "object",
      "description": "Bias assessment information",
      "properties": {
        "assessed": { "type": "boolean" },
        "assessmentDate": { "type": "string", "format": "date" },
        "methodology": { "type": "string" },
        "findings": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "dimension": { "type": "string" },
              "description": { "type": "string" },
              "severity": { "type": "string", "enum": ["low", "medium", "high"] },
              "mitigation": { "type": "string" }
            }
          }
        },
        "demographics": {
          "type": "object",
          "description": "Demographic distribution analysis"
        }
      }
    },
    "privacy": {
      "type": "object",
      "description": "Privacy information",
      "properties": {
        "containsPII": { "type": "boolean" },
        "piiTypes": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["name", "email", "phone", "address", "ssn", "financial", "health", "biometric", "other"]
          }
        },
        "anonymization": {
          "type": "object",
          "properties": {
            "applied": { "type": "boolean" },
            "technique": { "type": "string" },
            "verification": { "type": "string" }
          }
        },
        "consent": {
          "type": "object",
          "properties": {
            "obtained": { "type": "boolean" },
            "mechanism": { "type": "string" },
            "scope": { "type": "string" }
          }
        },
        "dataSubjectRights": {
          "type": "object",
          "properties": {
            "accessRequest": { "type": "string", "format": "uri" },
            "deletionRequest": { "type": "string", "format": "uri" }
          }
        }
      }
    },
    "provenance": {
      "type": "object",
      "description": "Data lineage information",
      "properties": {
        "derivedFrom": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "datasetId": { "type": "string" },
              "name": { "type": "string" },
              "version": { "type": "string" },
              "relationship": { "type": "string" }
            }
          }
        },
        "transformations": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "step": { "type": "integer" },
              "operation": { "type": "string" },
              "description": { "type": "string" },
              "code": { "type": "string", "format": "uri" },
              "timestamp": { "type": "string", "format": "date-time" }
            }
          }
        }
      }
    },
    "usage": {
      "type": "object",
      "description": "Usage guidelines",
      "properties": {
        "intendedUses": {
          "type": "array",
          "items": { "type": "string" }
        },
        "outOfScopeUses": {
          "type": "array",
          "items": { "type": "string" }
        },
        "risks": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "risk": { "type": "string" },
              "severity": { "type": "string" },
              "mitigation": { "type": "string" }
            }
          }
        },
        "benchmarks": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "task": { "type": "string" },
              "metric": { "type": "string" },
              "baselineScore": { "type": "number" },
              "humanPerformance": { "type": "number" }
            }
          }
        }
      }
    },
    "distribution": {
      "type": "object",
      "description": "Distribution information",
      "properties": {
        "format": {
          "type": "string",
          "enum": ["csv", "parquet", "json", "jsonl", "tfrecord", "arrow", "custom"]
        },
        "compression": {
          "type": "string",
          "enum": ["none", "gzip", "bz2", "lz4", "zstd"]
        },
        "location": {
          "type": "object",
          "properties": {
            "type": { "type": "string", "enum": ["url", "s3", "gcs", "azure", "huggingface"] },
            "uri": { "type": "string" }
          }
        },
        "checksum": {
          "type": "object",
          "properties": {
            "algorithm": { "type": "string", "enum": ["md5", "sha256", "sha512"] },
            "value": { "type": "string" }
          }
        }
      }
    },
    "maintenance": {
      "type": "object",
      "description": "Maintenance information",
      "properties": {
        "maintainer": {
          "type": "object",
          "properties": {
            "name": { "type": "string" },
            "email": { "type": "string", "format": "email" }
          }
        },
        "updateFrequency": {
          "type": "string",
          "enum": ["never", "annually", "quarterly", "monthly", "weekly", "daily", "continuous"]
        },
        "lastUpdate": { "type": "string", "format": "date" },
        "deprecationDate": { "type": "string", "format": "date" },
        "errata": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "date": { "type": "string", "format": "date" },
              "description": { "type": "string" },
              "affected": { "type": "string" },
              "resolution": { "type": "string" }
            }
          }
        }
      }
    },
    "citations": {
      "type": "array",
      "description": "How to cite the dataset",
      "items": {
        "type": "object",
        "properties": {
          "format": { "type": "string", "enum": ["bibtex", "apa", "mla", "chicago"] },
          "text": { "type": "string" }
        }
      }
    },
    "createdAt": {
      "type": "string",
      "format": "date-time"
    },
    "updatedAt": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

### 2.2 Example Data Card

```yaml
# Example Data Card for Image Classification Dataset
dataCardVersion: "1.0"
datasetId: "550e8400-e29b-41d4-a716-446655440000"
name: "WIA-ImageNet-Subset"
version: "2.1.0"

description:
  summary: "A curated subset of ImageNet with improved labels and demographic balance for image classification research."
  purpose: "Training and evaluating image classification models with reduced bias."
  content: "Contains 500,000 images across 1000 categories with verified labels and demographic metadata."
  limitations:
    - "English-centric category names"
    - "Western cultural bias in object selection"
    - "Limited representation of some geographic regions"

creators:
  - name: "Dr. Jane Smith"
    organization: "WIA Research Lab"
    email: "jane.smith@wia.org"
    role: "lead"
  - name: "John Doe"
    organization: "University of Technology"
    role: "contributor"

license:
  identifier: "CC-BY-4.0"
  url: "https://creativecommons.org/licenses/by/4.0/"
  permissions: ["commercial", "research", "modification", "distribution"]
  conditions: ["attribution"]

dataTypes: ["image"]

size:
  samples: 500000
  fileSize:
    value: 45.2
    unit: "GB"
  compressed: true

schema:
  features:
    - name: "image"
      type: "binary"
      description: "JPEG encoded image, 224x224 pixels"
    - name: "image_id"
      type: "string"
      description: "Unique identifier for the image"
  labels:
    - name: "class_id"
      type: "integer"
      classes:
        - id: 0
          name: "airplane"
          description: "Fixed-wing aircraft"
        - id: 1
          name: "automobile"
          description: "Four-wheeled motor vehicle"

splits:
  train:
    samples: 400000
    percentage: 80
  validation:
    samples: 50000
    percentage: 10
  test:
    samples: 50000
    percentage: 10

collection:
  method: "mixed"
  sources:
    - name: "ImageNet"
      url: "https://image-net.org"
      type: "dataset"
      accessDate: "2025-06-15"
  timeframe:
    start: "2025-01-01"
    end: "2025-12-31"
  geography: ["global"]
  samplingStrategy: "Stratified sampling by class with demographic balancing"

annotation:
  process: "semi-automated"
  platform: "WIA Annotation Platform v3.0"
  guidelines: "https://wia.org/annotation-guidelines/imagenet-v2"
  annotators:
    count: 150
    perSample: 3
    qualifications: "Minimum 100 hours annotation experience"
    compensation:
      type: "hourly"
      rate: "$18/hour"
  agreement:
    metric: "Fleiss' Kappa"
    value: 0.89

quality:
  overallScore: 9.2
  dimensions:
    accuracy: 0.96
    completeness: 0.99
    consistency: 0.97
    timeliness: 0.92
  validation:
    method: "Expert review + automated checks"
    date: "2025-12-01"
    samplesReviewed: 10000

bias:
  assessed: true
  assessmentDate: "2025-11-15"
  methodology: "WIA Bias Assessment Framework v1.0"
  findings:
    - dimension: "geographic"
      description: "North American and European regions overrepresented"
      severity: "medium"
      mitigation: "Added 50,000 images from underrepresented regions"
    - dimension: "gender"
      description: "Male subjects overrepresented in professional categories"
      severity: "medium"
      mitigation: "Balanced to within 5% parity"
  demographics:
    geographic:
      north_america: 0.30
      europe: 0.28
      asia: 0.25
      africa: 0.08
      south_america: 0.06
      oceania: 0.03

privacy:
  containsPII: true
  piiTypes: ["biometric"]
  anonymization:
    applied: true
    technique: "Face blurring for identifiable individuals"
    verification: "Manual review of flagged images"
  consent:
    obtained: true
    mechanism: "Inherited from source licenses + additional consent for new images"
    scope: "AI training and research"

provenance:
  derivedFrom:
    - datasetId: "imagenet-2012"
      name: "ImageNet Large Scale Visual Recognition Challenge 2012"
      version: "1.0"
      relationship: "subset"
  transformations:
    - step: 1
      operation: "filter"
      description: "Removed images with licensing issues"
      timestamp: "2025-06-20T10:30:00Z"
    - step: 2
      operation: "relabel"
      description: "Corrected labels using expert review"
      timestamp: "2025-08-15T14:00:00Z"
    - step: 3
      operation: "balance"
      description: "Added images to balance demographics"
      timestamp: "2025-10-01T09:00:00Z"

usage:
  intendedUses:
    - "Training image classification models"
    - "Evaluating transfer learning approaches"
    - "Benchmarking model fairness"
  outOfScopeUses:
    - "Surveillance applications"
    - "Facial recognition training"
    - "Military applications"
  risks:
    - risk: "Model may inherit remaining biases"
      severity: "medium"
      mitigation: "Use bias evaluation during model training"
  benchmarks:
    - task: "Image Classification"
      metric: "Top-1 Accuracy"
      baselineScore: 0.76
      humanPerformance: 0.95

distribution:
  format: "tfrecord"
  compression: "gzip"
  location:
    type: "s3"
    uri: "s3://wia-datasets/imagenet-subset-v2.1/"
  checksum:
    algorithm: "sha256"
    value: "abc123def456..."

maintenance:
  maintainer:
    name: "WIA Data Team"
    email: "data@wia.org"
  updateFrequency: "quarterly"
  lastUpdate: "2025-12-01"

citations:
  - format: "bibtex"
    text: |
      @dataset{wia_imagenet_2025,
        title={WIA-ImageNet-Subset: A Balanced Image Classification Dataset},
        author={Smith, Jane and Doe, John},
        year={2025},
        publisher={WIA}
      }

createdAt: "2025-06-01T00:00:00Z"
updatedAt: "2025-12-01T00:00:00Z"
```

---

## 3. Data Quality Architecture

### 3.1 Quality Assessment Pipeline

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    QUALITY ASSESSMENT PIPELINE                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│    ┌─────────────────────────────────────────────────────────────────┐  │
│    │                      INPUT VALIDATION                           │  │
│    │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │  │
│    │  │  Schema  │  │  Format  │  │Integrity │  │ Encoding │       │  │
│    │  │  Check   │  │  Check   │  │  Check   │  │  Check   │       │  │
│    │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │  │
│    │       └─────────────┴──────┬──────┴─────────────┘              │  │
│    └─────────────────────────────┼───────────────────────────────────┘  │
│                                  │                                       │
│    ┌─────────────────────────────▼───────────────────────────────────┐  │
│    │                    QUALITY METRICS                              │  │
│    │                                                                  │  │
│    │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │  │
│    │  │    ACCURACY     │  │  COMPLETENESS   │  │  CONSISTENCY    │ │  │
│    │  │                 │  │                 │  │                 │ │  │
│    │  │ • Label accuracy│  │ • Missing values│  │ • Format uniform│ │  │
│    │  │ • Noise level   │  │ • Field coverage│  │ • Value ranges  │ │  │
│    │  │ • Duplicates    │  │ • Required data │  │ • Cross-field   │ │  │
│    │  └─────────────────┘  └─────────────────┘  └─────────────────┘ │  │
│    │                                                                  │  │
│    │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │  │
│    │  │  TIMELINESS     │  │ REPRESENTATIVENESS│ │   RELEVANCE    │ │  │
│    │  │                 │  │                 │  │                 │ │  │
│    │  │ • Data freshness│  │ • Distribution  │  │ • Task fit      │ │  │
│    │  │ • Update recency│  │ • Coverage      │  │ • Domain match  │ │  │
│    │  │ • Temporal range│  │ • Balance       │  │ • Use case align│ │  │
│    │  └─────────────────┘  └─────────────────┘  └─────────────────┘ │  │
│    └─────────────────────────────────────────────────────────────────┘  │
│                                  │                                       │
│    ┌─────────────────────────────▼───────────────────────────────────┐  │
│    │                    SCORING & REPORTING                          │  │
│    │                                                                  │  │
│    │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │  │
│    │  │   Weighted   │  │  Quality     │  │   Report     │          │  │
│    │  │   Scoring    │  │  Score       │  │  Generation  │          │  │
│    │  └──────────────┘  └──────────────┘  └──────────────┘          │  │
│    └─────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Quality Metrics Definitions

```typescript
// Quality Metrics TypeScript Interface
interface QualityMetrics {
  accuracy: AccuracyMetrics;
  completeness: CompletenessMetrics;
  consistency: ConsistencyMetrics;
  timeliness: TimelinessMetrics;
  representativeness: RepresentativenessMetrics;
}

interface AccuracyMetrics {
  labelAccuracy: number;        // 0-1: Proportion of correct labels
  noiseLevel: number;           // 0-1: Proportion of noisy samples
  duplicateRate: number;        // 0-1: Proportion of duplicates
  errorRate: number;            // 0-1: Proportion of errors detected
}

interface CompletenessMetrics {
  featureCoverage: number;      // 0-1: Features with values
  sampleCompleteness: number;   // 0-1: Samples with all required fields
  labelCoverage: number;        // 0-1: Samples with labels
  metadataCoverage: number;     // 0-1: Required metadata present
}

interface ConsistencyMetrics {
  formatCompliance: number;     // 0-1: Samples following format
  valueRangeCompliance: number; // 0-1: Values within expected ranges
  crossFieldConsistency: number;// 0-1: Related fields consistent
  schemaCompliance: number;     // 0-1: Schema validation pass rate
}

interface TimelinessMetrics {
  dataAge: number;              // Days since data collection
  updateRecency: number;        // Days since last update
  temporalCoverage: number;     // 0-1: Coverage of intended timeframe
}

interface RepresentativenessMetrics {
  distributionBalance: number;  // 0-1: Class distribution balance
  demographicBalance: number;   // 0-1: Demographic representation
  geographicCoverage: number;   // 0-1: Geographic representation
  domainCoverage: number;       // 0-1: Domain/topic coverage
}

// Quality Score Calculation
function calculateQualityScore(metrics: QualityMetrics): number {
  const weights = {
    accuracy: 0.30,
    completeness: 0.25,
    consistency: 0.20,
    timeliness: 0.10,
    representativeness: 0.15
  };

  const dimensionScores = {
    accuracy: (
      metrics.accuracy.labelAccuracy * 0.5 +
      (1 - metrics.accuracy.noiseLevel) * 0.2 +
      (1 - metrics.accuracy.duplicateRate) * 0.15 +
      (1 - metrics.accuracy.errorRate) * 0.15
    ),
    completeness: (
      metrics.completeness.featureCoverage * 0.3 +
      metrics.completeness.sampleCompleteness * 0.3 +
      metrics.completeness.labelCoverage * 0.25 +
      metrics.completeness.metadataCoverage * 0.15
    ),
    consistency: (
      metrics.consistency.formatCompliance * 0.3 +
      metrics.consistency.valueRangeCompliance * 0.25 +
      metrics.consistency.crossFieldConsistency * 0.25 +
      metrics.consistency.schemaCompliance * 0.2
    ),
    timeliness: calculateTimelinessScore(metrics.timeliness),
    representativeness: (
      metrics.representativeness.distributionBalance * 0.35 +
      metrics.representativeness.demographicBalance * 0.30 +
      metrics.representativeness.geographicCoverage * 0.20 +
      metrics.representativeness.domainCoverage * 0.15
    )
  };

  return Object.entries(weights).reduce(
    (score, [key, weight]) => score + dimensionScores[key] * weight,
    0
  ) * 10; // Scale to 0-10
}
```

---

## 4. Provenance System

### 4.1 Lineage Graph Schema

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    PROVENANCE GRAPH MODEL                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│    NODE TYPES                                                           │
│    ══════════                                                           │
│                                                                          │
│    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐              │
│    │   Dataset   │    │ Transform   │    │   Agent     │              │
│    │             │    │             │    │             │              │
│    │ • id        │    │ • id        │    │ • id        │              │
│    │ • name      │    │ • operation │    │ • name      │              │
│    │ • version   │    │ • code_ref  │    │ • type      │              │
│    │ • created   │    │ • params    │    │ • org       │              │
│    └─────────────┘    └─────────────┘    └─────────────┘              │
│                                                                          │
│    EDGE TYPES                                                           │
│    ══════════                                                           │
│                                                                          │
│    Dataset ──DERIVED_FROM──► Dataset                                    │
│    Dataset ──GENERATED_BY──► Transform                                  │
│    Transform ──USED──► Dataset                                          │
│    Transform ──EXECUTED_BY──► Agent                                     │
│    Agent ──BELONGS_TO──► Organization                                   │
│                                                                          │
│    EXAMPLE GRAPH                                                        │
│    ═════════════                                                        │
│                                                                          │
│    ┌───────────┐         ┌───────────┐         ┌───────────┐          │
│    │ RawData_1 │─────────│  Filter   │─────────│ Dataset_A │          │
│    └───────────┘  USED   └───────────┘GENERATED└───────────┘          │
│                               │                      │                  │
│    ┌───────────┐              │ EXECUTED_BY          │ DERIVED_FROM    │
│    │ RawData_2 │──────────────┤                      │                  │
│    └───────────┘  USED        ▼                      ▼                  │
│                          ┌─────────┐          ┌───────────┐            │
│                          │ ETL_Bot │          │ Dataset_B │            │
│                          └─────────┘          └───────────┘            │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Provenance Record Format

```typescript
// Provenance Record Interface
interface ProvenanceRecord {
  id: string;
  datasetId: string;
  version: string;
  timestamp: Date;

  // Origin information
  origin: {
    type: 'primary' | 'derived' | 'generated' | 'merged';
    sources: SourceReference[];
  };

  // Transformation chain
  transformations: Transformation[];

  // Agents involved
  agents: AgentRecord[];

  // Verification
  verification: {
    hash: string;
    algorithm: 'sha256' | 'sha512';
    signature?: string;
    signedBy?: string;
  };

  // Compliance markers
  compliance: {
    gdpr?: {
      legalBasis: string;
      dataController: string;
      processingPurpose: string;
    };
    consentIds?: string[];
  };
}

interface SourceReference {
  datasetId: string;
  version: string;
  relationship: 'subset' | 'superset' | 'transform' | 'merge' | 'augment';
  samplesUsed?: number;
  fieldsUsed?: string[];
}

interface Transformation {
  id: string;
  step: number;
  operation: TransformOperation;
  description: string;
  parameters: Record<string, any>;
  codeReference?: {
    repository: string;
    commit: string;
    path: string;
  };
  inputHash: string;
  outputHash: string;
  timestamp: Date;
  executedBy: string;
  duration?: number;
}

enum TransformOperation {
  FILTER = 'filter',
  MAP = 'map',
  AGGREGATE = 'aggregate',
  JOIN = 'join',
  SPLIT = 'split',
  SAMPLE = 'sample',
  DEDUPLICATE = 'deduplicate',
  NORMALIZE = 'normalize',
  AUGMENT = 'augment',
  LABEL = 'label',
  ANONYMIZE = 'anonymize',
  CUSTOM = 'custom'
}

interface AgentRecord {
  id: string;
  type: 'human' | 'system' | 'model';
  name: string;
  organization?: string;
  role: string;
  actions: string[];
  timestamp: Date;
}
```

---

## 5. Storage Architecture

### 5.1 Multi-Tier Storage

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    STORAGE ARCHITECTURE                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│    HOT TIER (Frequent Access)                                           │
│    ══════════════════════════                                           │
│    ┌─────────────────────────────────────────────────────────────────┐  │
│    │  NVMe SSD Storage                                                │  │
│    │  • Active datasets                                               │  │
│    │  • Metadata cache                                                │  │
│    │  • Quality metrics                                               │  │
│    │  Latency: <10ms | Cost: $$$                                     │  │
│    └─────────────────────────────────────────────────────────────────┘  │
│                                                                          │
│    WARM TIER (Moderate Access)                                          │
│    ════════════════════════════                                         │
│    ┌─────────────────────────────────────────────────────────────────┐  │
│    │  Object Storage (S3/MinIO)                                       │  │
│    │  • Published datasets                                            │  │
│    │  • Version history                                               │  │
│    │  • Transformation artifacts                                      │  │
│    │  Latency: <100ms | Cost: $$                                     │  │
│    └─────────────────────────────────────────────────────────────────┘  │
│                                                                          │
│    COLD TIER (Archive)                                                  │
│    ═══════════════════                                                  │
│    ┌─────────────────────────────────────────────────────────────────┐  │
│    │  Glacier/Archive Storage                                         │  │
│    │  • Deprecated datasets                                           │  │
│    │  • Compliance archives                                           │  │
│    │  • Long-term backups                                             │  │
│    │  Latency: hours | Cost: $                                       │  │
│    └─────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 5.2 Data Format Specifications

| Format | Use Case | Pros | Cons |
|--------|----------|------|------|
| Parquet | Tabular data | Columnar, compressed, fast | Complex schema changes |
| TFRecord | TensorFlow training | Optimized for TF | TF-specific |
| Arrow | In-memory, cross-platform | Fast, language-agnostic | Memory overhead |
| JSONL | Flexible structured | Human-readable, flexible | Larger size |
| HDF5 | Large arrays | Efficient for arrays | Complex API |
| WebDataset | Large-scale training | Streaming, sharded | Sequential access |

---

## 6. API Specifications

### 6.1 REST API

```yaml
openapi: 3.1.0
info:
  title: WIA Training Data API
  version: 1.0.0

paths:
  /datasets:
    get:
      summary: List datasets
      parameters:
        - name: type
          in: query
          schema:
            type: string
        - name: quality_min
          in: query
          schema:
            type: number
      responses:
        '200':
          description: Dataset list
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DatasetList'

    post:
      summary: Register dataset
      requestBody:
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/DataCard'
      responses:
        '201':
          description: Dataset registered

  /datasets/{id}:
    get:
      summary: Get dataset details
      responses:
        '200':
          description: Dataset details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DataCard'

  /datasets/{id}/quality:
    get:
      summary: Get quality metrics
      responses:
        '200':
          description: Quality metrics
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/QualityMetrics'

  /datasets/{id}/provenance:
    get:
      summary: Get provenance chain
      responses:
        '200':
          description: Provenance records
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ProvenanceChain'

  /datasets/{id}/bias:
    get:
      summary: Get bias assessment
      responses:
        '200':
          description: Bias report
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/BiasReport'
```

---

## 7. Security Architecture

### 7.1 Access Control Model

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ACCESS CONTROL ARCHITECTURE                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│    ┌─────────────────────────────────────────────────────────────────┐  │
│    │                      AUTHENTICATION                             │  │
│    │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │  │
│    │  │  OAuth   │  │   OIDC   │  │ API Keys │  │  mTLS    │       │  │
│    │  │  2.0     │  │          │  │          │  │          │       │  │
│    │  └──────────┘  └──────────┘  └──────────┘  └──────────┘       │  │
│    └─────────────────────────────────────────────────────────────────┘  │
│                                      │                                   │
│    ┌─────────────────────────────────▼───────────────────────────────┐  │
│    │                      AUTHORIZATION                              │  │
│    │                                                                  │  │
│    │  RBAC (Role-Based)              ABAC (Attribute-Based)         │  │
│    │  ┌────────────────┐             ┌────────────────┐             │  │
│    │  │ Roles:         │             │ Attributes:    │             │  │
│    │  │ • Admin        │             │ • Organization │             │  │
│    │  │ • Publisher    │             │ • License type │             │  │
│    │  │ • Consumer     │             │ • Use purpose  │             │  │
│    │  │ • Auditor      │             │ • Data type    │             │  │
│    │  └────────────────┘             └────────────────┘             │  │
│    └─────────────────────────────────────────────────────────────────┘  │
│                                      │                                   │
│    ┌─────────────────────────────────▼───────────────────────────────┐  │
│    │                      DATA PROTECTION                            │  │
│    │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │  │
│    │  │Encryption│  │  Masking │  │  Audit   │  │   DLP    │       │  │
│    │  │ at Rest  │  │          │  │  Logging │  │          │       │  │
│    │  └──────────┘  └──────────┘  └──────────┘  └──────────┘       │  │
│    └─────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 8. Integration Patterns

### 8.1 ML Framework Integration

```python
# PyTorch DataLoader Integration
from wia_training_data import WIADataset, DataCard

class WIATorchDataset(torch.utils.data.Dataset):
    def __init__(self, dataset_id: str, split: str = 'train'):
        self.dataset = WIADataset.load(dataset_id)
        self.data_card = DataCard.fetch(dataset_id)
        self.split = split
        self.samples = self.dataset.get_split(split)

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        sample = self.samples[idx]
        # Apply transforms based on data card schema
        return self.transform(sample)

    @property
    def quality_score(self):
        return self.data_card.quality.overallScore

    @property
    def bias_warnings(self):
        return self.data_card.bias.findings
```

```python
# TensorFlow Dataset Integration
import tensorflow as tf
from wia_training_data import WIADataset

def create_tf_dataset(dataset_id: str, split: str = 'train'):
    wia_dataset = WIADataset.load(dataset_id)

    def generator():
        for sample in wia_dataset.get_split(split):
            yield sample

    return tf.data.Dataset.from_generator(
        generator,
        output_signature=wia_dataset.tf_signature
    )
```

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2026-01-13 | WIA Standards Committee | Initial release |

---

© 2026 WIA (World Certification Industry Association) / SmileStory Inc.
**홍익인간 (弘益人間) - Benefit All Humanity** 🌍

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-training-data is evaluated across three tiers:

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

- `wia-standards/standards/ai-training-data/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-training-data/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-training-data/simulator/` — interactive browser-based simulator for the PHASE protocol

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
