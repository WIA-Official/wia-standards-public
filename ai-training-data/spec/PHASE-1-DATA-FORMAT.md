# WIA-AI-007 PHASE 1: Data Format & Schema Specification

**Version:** 1.0.0
**Status:** Stable
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Overview

Phase 1 defines standardized data formats, schemas, and metadata structures for AI training datasets. This phase establishes the foundation for interoperable, reproducible, and high-quality training data management.

## Core Data Formats

### Supported Formats

| Format | Use Case | Advantages | Disadvantages |
|--------|----------|------------|---------------|
| JSON | Metadata, small datasets | Human-readable, widely supported | Verbose for large data |
| Parquet | Large tabular data | Columnar, compressed, fast | Binary format |
| TFRecord | TensorFlow pipelines | Native TF integration | Framework-specific |
| HDF5 | Scientific arrays | Hierarchical, partial loading | Complex API |
| Arrow | Cross-language exchange | Zero-copy, language-agnostic | Relatively new |

### Format Selection Guidelines

```python
# Decision tree for format selection
def select_format(data_type, size, framework):
    if data_type == "metadata":
        return "JSON"
    elif data_type == "tabular":
        if size < 1_000_000:
            return "CSV" or "JSON"
        else:
            return "Parquet"
    elif data_type == "image":
        if framework == "tensorflow":
            return "TFRecord"
        else:
            return "HDF5"
    elif data_type == "array":
        return "HDF5" or "Zarr"
```

## Metadata Schema (WIA-AI-007)

### Core Metadata Structure

```json
{
  "wia-ai-007": "1.0",
  "dataset": {
    "id": "unique-dataset-identifier",
    "name": "dataset-name",
    "version": "MAJOR.MINOR.PATCH",
    "created": "ISO-8601-timestamp",
    "updated": "ISO-8601-timestamp",
    "description": "Detailed dataset description",
    "type": "image|text|audio|video|tabular|multimodal",
    "format": "json|parquet|tfrecord|hdf5|arrow",
    "size": {
      "samples": 100000,
      "bytes": 5368709120,
      "files": 150
    },
    "splits": {
      "train": 0.8,
      "validation": 0.1,
      "test": 0.1
    },
    "philosophy": "弘익人間 - Created to benefit humanity"
  },
  "schema": {
    "features": {
      "feature_name": {
        "type": "binary|integer|float|string|categorical",
        "description": "Feature description",
        "required": true,
        "constraints": {}
      }
    },
    "labels": {
      "label_name": {
        "type": "categorical|continuous|multi-label",
        "classes": [],
        "description": "Label description"
      }
    }
  },
  "provenance": {
    "source": {
      "type": "original|derived|synthetic",
      "origin": "Data source description",
      "collection_date": "YYYY-MM-DD",
      "collection_method": "Method description",
      "contributors": []
    },
    "transformations": [],
    "parent_datasets": []
  },
  "quality": {
    "completeness": {"score": 0.0-1.0},
    "consistency": {"score": 0.0-1.0},
    "accuracy": {"score": 0.0-1.0},
    "uniqueness": {"score": 0.0-1.0},
    "overall_score": 0.0-1.0
  },
  "license": {
    "type": "CC-BY-4.0|MIT|Apache-2.0|proprietary",
    "url": "License URL",
    "attribution": "Attribution text",
    "commercial_use": true|false,
    "modifications_allowed": true|false
  },
  "ethics": {
    "consent_obtained": true|false,
    "anonymization": "Anonymization method",
    "bias_assessment": "Performed|Not performed",
    "intended_use": [],
    "prohibited_use": []
  },
  "statistics": {
    "label_distribution": {},
    "feature_statistics": {}
  }
}
```

## Semantic Versioning for Datasets

### Version Number Format: MAJOR.MINOR.PATCH

- **MAJOR**: Incompatible changes (schema changes, format changes)
- **MINOR**: Backward-compatible additions (new samples, new features)
- **PATCH**: Backward-compatible fixes (error corrections, metadata updates)

### Version History Tracking

```json
{
  "version_history": [
    {
      "version": "1.0.0",
      "date": "2025-01-15",
      "changes": ["Initial release"],
      "breaking_changes": false
    },
    {
      "version": "1.1.0",
      "date": "2025-02-01",
      "changes": ["Added 10000 new samples"],
      "breaking_changes": false
    }
  ]
}
```

## Schema Validation

### JSON Schema for Validation

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-AI-007 Dataset Metadata",
  "type": "object",
  "required": ["wia-ai-007", "dataset", "schema", "provenance", "quality", "license"],
  "properties": {
    "wia-ai-007": {
      "type": "string",
      "pattern": "^[0-9]+\\.[0-9]+$"
    },
    "dataset": {
      "type": "object",
      "required": ["id", "name", "version", "type", "format"]
    }
  }
}
```

## Feature Type Definitions

### Numeric Features
- **integer**: Whole numbers
- **float**: Decimal numbers
- **bounded**: Values within specific range

### Categorical Features
- **nominal**: Unordered categories
- **ordinal**: Ordered categories
- **multi-class**: Multiple discrete classes

### Sequential Features
- **text**: Natural language text
- **time-series**: Temporal sequences
- **sequence**: Ordered elements

### Binary Features
- **image**: Image data
- **audio**: Audio waveforms
- **video**: Video sequences

## Best Practices

1. **Always include metadata**: Every dataset must have a complete WIA-AI-007 metadata file
2. **Validate schema**: Use JSON Schema validation for metadata
3. **Version everything**: Track all changes through semantic versioning
4. **Document provenance**: Record complete data lineage
5. **Quality metrics**: Include quality scores for transparency
6. **License clarity**: Clearly specify usage rights
7. **Ethics documentation**: Document consent, anonymization, and bias assessment

## Implementation Example

```python
from wia_ai_007 import DatasetMetadata

# Create metadata
metadata = DatasetMetadata(
    name="medical-images-v1",
    version="1.0.0",
    data_type="image",
    format="hdf5"
)

# Add schema
metadata.add_feature("image", type="binary", format="png")
metadata.add_feature("age", type="integer", min=0, max=120)
metadata.add_label("diagnosis", type="categorical", classes=["normal", "abnormal"])

# Add provenance
metadata.set_provenance(
    source_type="original",
    collection_date="2025-01-10",
    collection_method="Hospital PACS system"
)

# Add quality metrics
metadata.set_quality(
    completeness=0.985,
    consistency=0.992,
    accuracy=0.94
)

# Validate and save
metadata.validate()
metadata.save("metadata.json")

# 弘益人間 - Standardized data for the benefit of all
```

## Compliance Checklist

- [ ] Metadata file created with all required fields
- [ ] Schema defined for all features and labels
- [ ] Provenance information documented
- [ ] Quality metrics calculated and recorded
- [ ] License clearly specified
- [ ] Ethics considerations documented
- [ ] Version number assigned (semantic versioning)
- [ ] Metadata validated against JSON Schema

---

**弘益人間** · Benefit All Humanity
© 2025 SmileStory Inc. / WIA
WIA-AI-007 AI Training Data Standard v1.0
