# WIA AI-TRAINING-DATA - PHASE 3: Implementation Details

## Version 1.0

> 홍익인간 (弘益人間) - Benefit All Humanity

## Document Information

- **Standard**: WIA-AI-TRAINING-DATA
- **Phase**: 3 - Implementation Details
- **Version**: 1.0.0
- **Status**: Draft
- **Date**: 2026-01-13
- **Authors**: WIA Standards Committee

---

## 📋 Table of Contents

1. [Implementation Guide](#1-implementation-guide)
2. [SDK Reference](#2-sdk-reference)
3. [Quality Assessment Tools](#3-quality-assessment-tools)
4. [Bias Detection Implementation](#4-bias-detection-implementation)
5. [Data Card Generator](#5-data-card-generator)
6. [Testing Framework](#6-testing-framework)
7. [CLI Tools](#7-cli-tools)
8. [Best Practices](#8-best-practices)

---

## 1. Implementation Guide

### 1.1 Getting Started

#### 1.1.1 Installation

```bash
# Python SDK
pip install wia-training-data

# Node.js SDK
npm install @wia/training-data

# CLI Tools
pip install wia-training-data[cli]

# With all optional dependencies
pip install wia-training-data[all]
```

#### 1.1.2 Quick Start

```python
from wia_training_data import DataCard, QualityAssessor, Dataset

# Create a Data Card for your dataset
data_card = DataCard(
    name="my-classification-dataset",
    version="1.0.0",
    description={
        "summary": "A dataset for text classification with 100k samples across 10 categories",
        "purpose": "Training text classifiers for customer support routing"
    },
    creators=[{
        "name": "Data Team",
        "organization": "My Company",
        "email": "data@mycompany.com"
    }],
    license={"identifier": "CC-BY-4.0"},
    data_types=["text"],
    size={"samples": 100000}
)

# Assess quality
assessor = QualityAssessor()
quality_report = assessor.assess(dataset_path="./data/", data_card=data_card)

print(f"Quality Score: {quality_report.overall_score}/10")
print(f"Completeness: {quality_report.completeness:.2%}")
print(f"Label Accuracy: {quality_report.label_accuracy:.2%}")

# Save Data Card
data_card.quality = quality_report.to_dict()
data_card.save("./data/DATA_CARD.yaml")
```

### 1.2 Project Structure

```
my-dataset/
├── data/
│   ├── train/
│   │   ├── data.parquet
│   │   └── metadata.json
│   ├── validation/
│   │   ├── data.parquet
│   │   └── metadata.json
│   └── test/
│       ├── data.parquet
│       └── metadata.json
├── annotations/
│   ├── labels.json
│   └── guidelines.md
├── provenance/
│   ├── lineage.json
│   └── transformations/
│       ├── step_001_filter.py
│       └── step_002_clean.py
├── quality/
│   ├── quality_report.json
│   └── bias_report.json
├── DATA_CARD.yaml          # Main documentation
├── README.md
├── LICENSE
└── CHANGELOG.md
```

### 1.3 Step-by-Step Implementation

#### Step 1: Define Dataset Schema

```python
from wia_training_data import Schema, Feature, Label

# Define your data schema
schema = Schema(
    features=[
        Feature(
            name="text",
            type="string",
            description="Input text for classification",
            constraints={"max_length": 5000}
        ),
        Feature(
            name="metadata",
            type="object",
            description="Additional metadata",
            properties={
                "source": {"type": "string"},
                "timestamp": {"type": "datetime"}
            }
        )
    ],
    labels=[
        Label(
            name="category",
            type="categorical",
            classes=[
                {"id": 0, "name": "billing", "description": "Billing inquiries"},
                {"id": 1, "name": "technical", "description": "Technical support"},
                {"id": 2, "name": "general", "description": "General questions"},
                # ... more classes
            ]
        ),
        Label(
            name="sentiment",
            type="categorical",
            classes=[
                {"id": 0, "name": "negative"},
                {"id": 1, "name": "neutral"},
                {"id": 2, "name": "positive"}
            ]
        )
    ]
)
```

#### Step 2: Implement Data Validation

```python
from wia_training_data import DataValidator, ValidationRule

# Create validator with rules
validator = DataValidator(schema=schema)

# Add custom validation rules
validator.add_rule(ValidationRule(
    name="text_not_empty",
    field="text",
    check=lambda x: len(x.strip()) > 0,
    message="Text field cannot be empty"
))

validator.add_rule(ValidationRule(
    name="text_min_length",
    field="text",
    check=lambda x: len(x) >= 10,
    message="Text must be at least 10 characters"
))

validator.add_rule(ValidationRule(
    name="valid_category",
    field="category",
    check=lambda x: 0 <= x < 10,
    message="Category must be between 0 and 9"
))

# Validate dataset
validation_result = validator.validate("./data/train/data.parquet")

print(f"Valid samples: {validation_result.valid_count}")
print(f"Invalid samples: {validation_result.invalid_count}")
print(f"Validation rate: {validation_result.pass_rate:.2%}")

# Export validation report
validation_result.to_json("./quality/validation_report.json")
```

#### Step 3: Track Provenance

```python
from wia_training_data import ProvenanceTracker, Transformation

# Initialize provenance tracker
tracker = ProvenanceTracker(dataset_id="my-classification-dataset")

# Record data sources
tracker.add_source({
    "name": "customer_support_logs",
    "type": "internal_database",
    "access_date": "2025-06-01",
    "samples_extracted": 500000
})

# Record transformations
tracker.add_transformation(Transformation(
    step=1,
    operation="filter",
    description="Filter English-only messages",
    parameters={"language": "en", "confidence_threshold": 0.9},
    code_reference={
        "repository": "https://github.com/mycompany/data-pipelines",
        "commit": "abc123",
        "path": "transformations/language_filter.py"
    },
    input_count=500000,
    output_count=450000
))

tracker.add_transformation(Transformation(
    step=2,
    operation="deduplicate",
    description="Remove duplicate messages using MinHash LSH",
    parameters={"similarity_threshold": 0.85, "num_perm": 128},
    input_count=450000,
    output_count=380000
))

tracker.add_transformation(Transformation(
    step=3,
    operation="anonymize",
    description="Remove PII using regex and NER",
    parameters={"pii_types": ["email", "phone", "name", "ssn"]},
    input_count=380000,
    output_count=380000
))

tracker.add_transformation(Transformation(
    step=4,
    operation="sample",
    description="Stratified sampling by category",
    parameters={"target_size": 100000, "stratify_by": "category"},
    input_count=380000,
    output_count=100000
))

# Generate provenance record
provenance = tracker.generate()
provenance.save("./provenance/lineage.json")

# Verify provenance chain
verification = tracker.verify()
print(f"Provenance verified: {verification.is_valid}")
```

#### Step 4: Assess Quality

```python
from wia_training_data import QualityAssessor, QualityConfig

# Configure quality assessment
config = QualityConfig(
    accuracy_checks={
        "label_validation": {
            "method": "expert_sample",
            "sample_size": 1000,
            "agreement_threshold": 0.9
        },
        "noise_detection": {
            "method": "outlier_detection",
            "contamination": 0.05
        }
    },
    completeness_checks={
        "required_fields": ["text", "category"],
        "optional_fields": ["sentiment", "metadata"]
    },
    consistency_checks={
        "format_validation": True,
        "cross_field_rules": [
            {"if": "category == 'technical'", "then": "text.contains('error') or text.contains('help')"}
        ]
    }
)

# Run assessment
assessor = QualityAssessor(config)
report = assessor.assess("./data/")

# Detailed metrics
print("=== Quality Report ===")
print(f"Overall Score: {report.overall_score:.1f}/10")
print(f"\nAccuracy Metrics:")
print(f"  - Label Accuracy: {report.accuracy.label_accuracy:.2%}")
print(f"  - Noise Level: {report.accuracy.noise_level:.2%}")
print(f"  - Duplicate Rate: {report.accuracy.duplicate_rate:.2%}")
print(f"\nCompleteness Metrics:")
print(f"  - Feature Coverage: {report.completeness.feature_coverage:.2%}")
print(f"  - Sample Completeness: {report.completeness.sample_completeness:.2%}")
print(f"\nConsistency Metrics:")
print(f"  - Format Compliance: {report.consistency.format_compliance:.2%}")
print(f"  - Schema Compliance: {report.consistency.schema_compliance:.2%}")

# Save report
report.to_json("./quality/quality_report.json")
```

#### Step 5: Detect Bias

```python
from wia_training_data import BiasDetector, BiasConfig

# Configure bias detection
bias_config = BiasConfig(
    protected_attributes=["gender", "age_group", "ethnicity"],
    fairness_metrics=["demographic_parity", "equalized_odds"],
    representation_analysis=True,
    intersectional_analysis=True
)

# Run bias detection
detector = BiasDetector(bias_config)
bias_report = detector.analyze("./data/", label_column="category")

# Review findings
print("=== Bias Report ===")
for finding in bias_report.findings:
    print(f"\n{finding.dimension}: {finding.severity.upper()}")
    print(f"  Description: {finding.description}")
    print(f"  Mitigation: {finding.mitigation}")

# Class distribution analysis
print("\n=== Class Distribution ===")
for cls, stats in bias_report.class_distribution.items():
    print(f"  {cls}: {stats.count} samples ({stats.percentage:.1%})")

# Demographic analysis
print("\n=== Demographic Distribution ===")
for attr, distribution in bias_report.demographics.items():
    print(f"\n  {attr}:")
    for value, percentage in distribution.items():
        print(f"    - {value}: {percentage:.1%}")

# Save report
bias_report.to_json("./quality/bias_report.json")
```

#### Step 6: Generate Data Card

```python
from wia_training_data import DataCardGenerator

# Generate comprehensive Data Card
generator = DataCardGenerator()

data_card = generator.generate(
    dataset_path="./data/",
    schema=schema,
    provenance_path="./provenance/lineage.json",
    quality_report_path="./quality/quality_report.json",
    bias_report_path="./quality/bias_report.json",

    # Additional metadata
    name="customer-support-classification",
    version="1.0.0",
    description={
        "summary": "Customer support message classification dataset with 100k samples",
        "purpose": "Training ML models for automatic ticket routing",
        "content": "Customer support messages labeled with category and sentiment",
        "limitations": [
            "English only",
            "US customer data primarily",
            "May not generalize to other domains"
        ]
    },
    creators=[{
        "name": "Data Science Team",
        "organization": "Example Corp",
        "email": "datascience@example.com",
        "role": "lead"
    }],
    license={
        "identifier": "proprietary",
        "permissions": ["internal-research"],
        "conditions": ["attribution", "no-distribution"]
    },
    usage={
        "intended_uses": [
            "Training text classification models",
            "Evaluating NLP approaches for customer support"
        ],
        "out_of_scope_uses": [
            "Customer identification",
            "Sentiment manipulation",
            "External distribution"
        ]
    },
    maintenance={
        "maintainer": {"name": "Data Team", "email": "data@example.com"},
        "update_frequency": "quarterly"
    }
)

# Validate Data Card completeness
validation = data_card.validate()
if validation.is_complete:
    print("✓ Data Card is complete and valid")
else:
    print("✗ Missing fields:", validation.missing_fields)

# Save in multiple formats
data_card.save("./DATA_CARD.yaml", format="yaml")
data_card.save("./DATA_CARD.json", format="json")
```

---

## 2. SDK Reference

### 2.1 Python SDK

```python
"""
WIA Training Data Python SDK
Complete API Reference
"""

# === Core Classes ===

class DataCard:
    """
    Represents a WIA-compliant Data Card for a training dataset.

    Attributes:
        data_card_version (str): Schema version
        dataset_id (str): Unique identifier
        name (str): Dataset name
        version (str): Semantic version
        description (dict): Description fields
        creators (list): Creator information
        license (dict): License details
        data_types (list): Data type categories
        size (dict): Size information
        schema (dict): Data schema
        splits (dict): Train/val/test splits
        collection (dict): Collection methodology
        annotation (dict): Annotation process
        quality (dict): Quality metrics
        bias (dict): Bias assessment
        privacy (dict): Privacy information
        provenance (dict): Lineage information
        usage (dict): Usage guidelines
        distribution (dict): Distribution info
        maintenance (dict): Maintenance info
        citations (list): Citation formats
    """

    def __init__(self, **kwargs):
        """Initialize DataCard with provided fields."""
        pass

    @classmethod
    def load(cls, path: str) -> 'DataCard':
        """Load DataCard from YAML or JSON file."""
        pass

    def save(self, path: str, format: str = 'yaml') -> None:
        """Save DataCard to file."""
        pass

    def validate(self) -> ValidationResult:
        """Validate DataCard against schema."""
        pass

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        pass

    def to_markdown(self) -> str:
        """Generate markdown documentation."""
        pass


class Dataset:
    """
    Represents a training dataset with WIA metadata.

    Methods for loading, iterating, and analyzing datasets.
    """

    @classmethod
    def load(cls, path: str, data_card: DataCard = None) -> 'Dataset':
        """
        Load dataset from path.

        Args:
            path: Path to dataset directory
            data_card: Optional DataCard for metadata

        Returns:
            Dataset instance
        """
        pass

    def get_split(self, split: str) -> Iterator:
        """Get iterator for specific split."""
        pass

    def sample(self, n: int, stratify: str = None) -> 'Dataset':
        """Sample n items from dataset."""
        pass

    def statistics(self) -> DatasetStatistics:
        """Compute dataset statistics."""
        pass


class QualityAssessor:
    """
    Assesses data quality using WIA metrics.

    Computes accuracy, completeness, consistency, timeliness,
    and representativeness metrics.
    """

    def __init__(self, config: QualityConfig = None):
        """Initialize with optional configuration."""
        pass

    def assess(self, dataset_path: str, data_card: DataCard = None) -> QualityReport:
        """
        Run full quality assessment.

        Args:
            dataset_path: Path to dataset
            data_card: Optional DataCard for context

        Returns:
            QualityReport with all metrics
        """
        pass

    def assess_accuracy(self, dataset_path: str) -> AccuracyMetrics:
        """Assess accuracy dimensions only."""
        pass

    def assess_completeness(self, dataset_path: str) -> CompletenessMetrics:
        """Assess completeness dimensions only."""
        pass

    def assess_consistency(self, dataset_path: str) -> ConsistencyMetrics:
        """Assess consistency dimensions only."""
        pass


class BiasDetector:
    """
    Detects and analyzes bias in training datasets.

    Uses statistical methods and fairness metrics to identify
    potential biases and representation issues.
    """

    def __init__(self, config: BiasConfig = None):
        """Initialize with optional configuration."""
        pass

    def analyze(self, dataset_path: str, label_column: str) -> BiasReport:
        """
        Run comprehensive bias analysis.

        Args:
            dataset_path: Path to dataset
            label_column: Name of label column

        Returns:
            BiasReport with findings and recommendations
        """
        pass

    def analyze_distribution(self, column: str) -> DistributionAnalysis:
        """Analyze distribution of specific column."""
        pass

    def analyze_intersectional(self, attributes: list) -> IntersectionalAnalysis:
        """Analyze intersectional bias across attributes."""
        pass


class ProvenanceTracker:
    """
    Tracks data lineage and transformations.

    Creates verifiable provenance records for datasets.
    """

    def __init__(self, dataset_id: str):
        """Initialize tracker for specific dataset."""
        pass

    def add_source(self, source: dict) -> None:
        """Add data source to provenance."""
        pass

    def add_transformation(self, transformation: Transformation) -> None:
        """Add transformation step to provenance."""
        pass

    def generate(self) -> ProvenanceRecord:
        """Generate complete provenance record."""
        pass

    def verify(self) -> VerificationResult:
        """Verify provenance chain integrity."""
        pass


# === Utility Functions ===

def validate_data_card(path: str) -> ValidationResult:
    """Validate Data Card file against WIA schema."""
    pass

def compute_quality_score(metrics: QualityMetrics) -> float:
    """Compute overall quality score from metrics."""
    pass

def detect_duplicates(dataset_path: str, threshold: float = 0.85) -> list:
    """Detect duplicate samples using MinHash LSH."""
    pass

def anonymize_pii(text: str, pii_types: list = None) -> str:
    """Remove PII from text."""
    pass

def compute_label_agreement(annotations: list) -> float:
    """Compute inter-annotator agreement."""
    pass
```

### 2.2 TypeScript SDK

```typescript
/**
 * WIA Training Data TypeScript SDK
 */

import { z } from 'zod';

// === Core Types ===

export interface DataCard {
  dataCardVersion: string;
  datasetId: string;
  name: string;
  version: string;
  description: Description;
  creators: Creator[];
  license: License;
  dataTypes: DataType[];
  size: Size;
  schema?: Schema;
  splits?: Splits;
  collection?: Collection;
  annotation?: Annotation;
  quality?: Quality;
  bias?: Bias;
  privacy?: Privacy;
  provenance?: Provenance;
  usage?: Usage;
  distribution?: Distribution;
  maintenance?: Maintenance;
  citations?: Citation[];
  createdAt: string;
  updatedAt?: string;
}

export interface QualityMetrics {
  overallScore: number;
  accuracy: AccuracyMetrics;
  completeness: CompletenessMetrics;
  consistency: ConsistencyMetrics;
  timeliness: TimelinessMetrics;
  representativeness: RepresentativenessMetrics;
}

export interface BiasReport {
  assessed: boolean;
  assessmentDate: string;
  methodology: string;
  findings: BiasFinding[];
  demographics: Record<string, Record<string, number>>;
  classDistribution: Record<string, ClassStats>;
}

// === SDK Classes ===

export class WIADataCard {
  private data: DataCard;

  constructor(data: Partial<DataCard>) {
    this.data = this.validate(data);
  }

  static async load(path: string): Promise<WIADataCard> {
    const content = await fs.readFile(path, 'utf-8');
    const data = path.endsWith('.yaml')
      ? yaml.parse(content)
      : JSON.parse(content);
    return new WIADataCard(data);
  }

  async save(path: string, format: 'yaml' | 'json' = 'yaml'): Promise<void> {
    const content = format === 'yaml'
      ? yaml.stringify(this.data)
      : JSON.stringify(this.data, null, 2);
    await fs.writeFile(path, content);
  }

  validate(data: Partial<DataCard>): DataCard {
    return DataCardSchema.parse(data);
  }

  toMarkdown(): string {
    return generateMarkdown(this.data);
  }

  get quality(): Quality | undefined {
    return this.data.quality;
  }

  set quality(value: Quality) {
    this.data.quality = value;
  }
}

export class QualityAssessor {
  constructor(private config?: QualityConfig) {}

  async assess(datasetPath: string): Promise<QualityReport> {
    const accuracy = await this.assessAccuracy(datasetPath);
    const completeness = await this.assessCompleteness(datasetPath);
    const consistency = await this.assessConsistency(datasetPath);
    const timeliness = await this.assessTimeliness(datasetPath);

    const overallScore = this.computeScore({
      accuracy,
      completeness,
      consistency,
      timeliness
    });

    return {
      overallScore,
      accuracy,
      completeness,
      consistency,
      timeliness,
      assessmentDate: new Date().toISOString()
    };
  }

  private computeScore(metrics: Partial<QualityMetrics>): number {
    const weights = {
      accuracy: 0.30,
      completeness: 0.25,
      consistency: 0.20,
      timeliness: 0.10,
      representativeness: 0.15
    };

    let score = 0;
    let totalWeight = 0;

    for (const [key, weight] of Object.entries(weights)) {
      if (metrics[key as keyof QualityMetrics]) {
        score += this.dimensionScore(metrics[key as keyof QualityMetrics]) * weight;
        totalWeight += weight;
      }
    }

    return (score / totalWeight) * 10;
  }

  private dimensionScore(metrics: any): number {
    const values = Object.values(metrics).filter(v => typeof v === 'number') as number[];
    return values.reduce((a, b) => a + b, 0) / values.length;
  }
}

export class BiasDetector {
  constructor(private config?: BiasConfig) {}

  async analyze(
    datasetPath: string,
    labelColumn: string
  ): Promise<BiasReport> {
    const dataset = await this.loadDataset(datasetPath);
    const findings: BiasFinding[] = [];

    // Analyze class distribution
    const classDistribution = this.analyzeClassDistribution(dataset, labelColumn);
    const imbalance = this.detectImbalance(classDistribution);
    if (imbalance.severity !== 'none') {
      findings.push({
        dimension: 'class_distribution',
        description: `Class imbalance detected: ${imbalance.description}`,
        severity: imbalance.severity,
        mitigation: 'Consider oversampling minority classes or using class weights'
      });
    }

    // Analyze demographic distribution
    const demographics: Record<string, Record<string, number>> = {};
    for (const attr of this.config?.protectedAttributes || []) {
      demographics[attr] = await this.analyzeAttribute(dataset, attr);
    }

    return {
      assessed: true,
      assessmentDate: new Date().toISOString(),
      methodology: 'WIA Bias Assessment Framework v1.0',
      findings,
      demographics,
      classDistribution
    };
  }
}
```

---

## 3. Quality Assessment Tools

### 3.1 Automated Quality Checks

```python
# quality_checks.py
from dataclasses import dataclass
from typing import List, Callable, Any
import pandas as pd
import numpy as np
from sklearn.ensemble import IsolationForest

@dataclass
class QualityCheck:
    name: str
    description: str
    check_fn: Callable
    severity: str  # 'error', 'warning', 'info'
    auto_fix: Callable = None

class QualityChecker:
    """Automated quality checking framework."""

    def __init__(self):
        self.checks: List[QualityCheck] = []
        self._register_default_checks()

    def _register_default_checks(self):
        """Register default quality checks."""

        # Completeness checks
        self.register(QualityCheck(
            name="missing_values",
            description="Check for missing values in required fields",
            check_fn=self._check_missing_values,
            severity="error"
        ))

        self.register(QualityCheck(
            name="empty_strings",
            description="Check for empty strings in text fields",
            check_fn=self._check_empty_strings,
            severity="warning"
        ))

        # Accuracy checks
        self.register(QualityCheck(
            name="duplicate_detection",
            description="Detect duplicate samples",
            check_fn=self._check_duplicates,
            severity="warning",
            auto_fix=self._fix_duplicates
        ))

        self.register(QualityCheck(
            name="outlier_detection",
            description="Detect statistical outliers",
            check_fn=self._check_outliers,
            severity="info"
        ))

        # Consistency checks
        self.register(QualityCheck(
            name="type_consistency",
            description="Check data type consistency",
            check_fn=self._check_type_consistency,
            severity="error"
        ))

        self.register(QualityCheck(
            name="range_validation",
            description="Check values are within expected ranges",
            check_fn=self._check_ranges,
            severity="warning"
        ))

    def register(self, check: QualityCheck):
        """Register a quality check."""
        self.checks.append(check)

    def run_all(self, df: pd.DataFrame, config: dict = None) -> dict:
        """Run all registered checks."""
        results = {
            'passed': [],
            'failed': [],
            'warnings': [],
            'info': []
        }

        for check in self.checks:
            try:
                result = check.check_fn(df, config)
                if result['passed']:
                    results['passed'].append({
                        'name': check.name,
                        'message': result.get('message', 'Check passed')
                    })
                else:
                    category = 'failed' if check.severity == 'error' else (
                        'warnings' if check.severity == 'warning' else 'info'
                    )
                    results[category].append({
                        'name': check.name,
                        'message': result['message'],
                        'details': result.get('details', {}),
                        'affected_rows': result.get('affected_rows', [])
                    })
            except Exception as e:
                results['failed'].append({
                    'name': check.name,
                    'message': f"Check failed with error: {str(e)}"
                })

        # Calculate overall score
        total_checks = len(self.checks)
        passed_checks = len(results['passed'])
        results['score'] = passed_checks / total_checks if total_checks > 0 else 0

        return results

    def _check_missing_values(self, df: pd.DataFrame, config: dict = None) -> dict:
        """Check for missing values."""
        missing = df.isnull().sum()
        missing_cols = missing[missing > 0]

        if len(missing_cols) == 0:
            return {'passed': True, 'message': 'No missing values found'}

        return {
            'passed': False,
            'message': f'Missing values found in {len(missing_cols)} columns',
            'details': {
                'columns': missing_cols.to_dict(),
                'total_missing': int(missing_cols.sum())
            }
        }

    def _check_duplicates(self, df: pd.DataFrame, config: dict = None) -> dict:
        """Check for duplicate rows."""
        duplicates = df.duplicated()
        num_duplicates = duplicates.sum()

        if num_duplicates == 0:
            return {'passed': True, 'message': 'No duplicates found'}

        return {
            'passed': False,
            'message': f'Found {num_duplicates} duplicate rows ({num_duplicates/len(df):.2%})',
            'affected_rows': df[duplicates].index.tolist()
        }

    def _check_outliers(self, df: pd.DataFrame, config: dict = None) -> dict:
        """Detect outliers using Isolation Forest."""
        numeric_cols = df.select_dtypes(include=[np.number]).columns
        if len(numeric_cols) == 0:
            return {'passed': True, 'message': 'No numeric columns to check'}

        clf = IsolationForest(contamination=0.05, random_state=42)
        outliers = clf.fit_predict(df[numeric_cols].fillna(0))
        num_outliers = (outliers == -1).sum()

        threshold = config.get('outlier_threshold', 0.1) if config else 0.1
        if num_outliers / len(df) < threshold:
            return {
                'passed': True,
                'message': f'Outlier rate ({num_outliers/len(df):.2%}) within acceptable range'
            }

        return {
            'passed': False,
            'message': f'High outlier rate: {num_outliers} outliers ({num_outliers/len(df):.2%})',
            'affected_rows': np.where(outliers == -1)[0].tolist()
        }

    def _check_empty_strings(self, df: pd.DataFrame, config: dict = None) -> dict:
        """Check for empty strings in text columns."""
        string_cols = df.select_dtypes(include=['object']).columns
        empty_counts = {}

        for col in string_cols:
            empty = (df[col].str.strip() == '').sum()
            if empty > 0:
                empty_counts[col] = int(empty)

        if not empty_counts:
            return {'passed': True, 'message': 'No empty strings found'}

        return {
            'passed': False,
            'message': f'Empty strings found in {len(empty_counts)} columns',
            'details': {'columns': empty_counts}
        }

    def _check_type_consistency(self, df: pd.DataFrame, config: dict = None) -> dict:
        """Check data type consistency."""
        issues = []

        for col in df.columns:
            # Check for mixed types
            non_null = df[col].dropna()
            if len(non_null) > 0:
                types = non_null.apply(type).unique()
                if len(types) > 1:
                    issues.append({
                        'column': col,
                        'types': [t.__name__ for t in types]
                    })

        if not issues:
            return {'passed': True, 'message': 'All columns have consistent types'}

        return {
            'passed': False,
            'message': f'Type inconsistencies found in {len(issues)} columns',
            'details': {'issues': issues}
        }

    def _check_ranges(self, df: pd.DataFrame, config: dict = None) -> dict:
        """Check values are within expected ranges."""
        if not config or 'ranges' not in config:
            return {'passed': True, 'message': 'No range constraints defined'}

        violations = []
        for col, (min_val, max_val) in config['ranges'].items():
            if col in df.columns:
                out_of_range = ((df[col] < min_val) | (df[col] > max_val)).sum()
                if out_of_range > 0:
                    violations.append({
                        'column': col,
                        'expected_range': [min_val, max_val],
                        'violations': int(out_of_range)
                    })

        if not violations:
            return {'passed': True, 'message': 'All values within expected ranges'}

        return {
            'passed': False,
            'message': f'Range violations found in {len(violations)} columns',
            'details': {'violations': violations}
        }

    def _fix_duplicates(self, df: pd.DataFrame, config: dict = None) -> pd.DataFrame:
        """Auto-fix: Remove duplicate rows."""
        return df.drop_duplicates()
```

---

## 4. Bias Detection Implementation

### 4.1 Comprehensive Bias Analysis

```python
# bias_detection.py
import pandas as pd
import numpy as np
from scipy import stats
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

class Severity(Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"

@dataclass
class BiasFinding:
    dimension: str
    description: str
    severity: Severity
    mitigation: str
    metrics: Dict[str, Any]

class ComprehensiveBiasDetector:
    """
    Comprehensive bias detection for training datasets.

    Analyzes multiple bias dimensions:
    - Class imbalance
    - Demographic representation
    - Linguistic bias
    - Temporal bias
    - Geographic bias
    """

    def __init__(self, config: Dict = None):
        self.config = config or {}
        self.protected_attributes = self.config.get('protected_attributes', [])
        self.findings: List[BiasFinding] = []

    def analyze(self, df: pd.DataFrame, label_col: str) -> Dict:
        """Run comprehensive bias analysis."""
        self.findings = []

        # 1. Class imbalance analysis
        class_analysis = self._analyze_class_imbalance(df, label_col)
        self.findings.extend(class_analysis['findings'])

        # 2. Demographic bias analysis
        demo_analysis = self._analyze_demographic_bias(df, label_col)
        self.findings.extend(demo_analysis['findings'])

        # 3. Text/linguistic bias (if text data)
        if self._has_text_columns(df):
            text_analysis = self._analyze_text_bias(df)
            self.findings.extend(text_analysis['findings'])

        # 4. Temporal bias (if temporal data)
        if self._has_temporal_columns(df):
            temporal_analysis = self._analyze_temporal_bias(df, label_col)
            self.findings.extend(temporal_analysis['findings'])

        return {
            'assessed': True,
            'total_findings': len(self.findings),
            'high_severity': len([f for f in self.findings if f.severity == Severity.HIGH]),
            'medium_severity': len([f for f in self.findings if f.severity == Severity.MEDIUM]),
            'low_severity': len([f for f in self.findings if f.severity == Severity.LOW]),
            'findings': [self._finding_to_dict(f) for f in self.findings],
            'class_distribution': class_analysis['distribution'],
            'demographic_distribution': demo_analysis.get('distribution', {})
        }

    def _analyze_class_imbalance(self, df: pd.DataFrame, label_col: str) -> Dict:
        """Analyze class distribution imbalance."""
        findings = []
        distribution = df[label_col].value_counts(normalize=True).to_dict()

        # Calculate imbalance metrics
        values = list(distribution.values())
        max_ratio = max(values) / min(values) if min(values) > 0 else float('inf')
        entropy = stats.entropy(values)
        max_entropy = np.log(len(values))
        balance_score = entropy / max_entropy if max_entropy > 0 else 0

        # Determine severity
        if max_ratio > 10 or balance_score < 0.5:
            severity = Severity.HIGH
        elif max_ratio > 5 or balance_score < 0.7:
            severity = Severity.MEDIUM
        elif max_ratio > 2:
            severity = Severity.LOW
        else:
            return {'findings': [], 'distribution': distribution}

        findings.append(BiasFinding(
            dimension="class_imbalance",
            description=f"Class imbalance detected with ratio {max_ratio:.1f}:1 between largest and smallest classes",
            severity=severity,
            mitigation="Consider using oversampling (SMOTE), undersampling, or class weights during training",
            metrics={
                'max_ratio': max_ratio,
                'balance_score': balance_score,
                'majority_class': max(distribution, key=distribution.get),
                'minority_class': min(distribution, key=distribution.get)
            }
        ))

        return {'findings': findings, 'distribution': distribution}

    def _analyze_demographic_bias(self, df: pd.DataFrame, label_col: str) -> Dict:
        """Analyze demographic representation bias."""
        findings = []
        distribution = {}

        for attr in self.protected_attributes:
            if attr not in df.columns:
                continue

            attr_dist = df[attr].value_counts(normalize=True).to_dict()
            distribution[attr] = attr_dist

            # Check for significant imbalance
            values = list(attr_dist.values())
            if len(values) < 2:
                continue

            max_ratio = max(values) / min(values)
            gini = self._calculate_gini(values)

            # Check representation against expected (if provided)
            expected = self.config.get('expected_distributions', {}).get(attr, {})
            if expected:
                max_deviation = max(
                    abs(attr_dist.get(k, 0) - v)
                    for k, v in expected.items()
                )
            else:
                max_deviation = 0

            # Determine severity
            if max_ratio > 5 or max_deviation > 0.3:
                severity = Severity.HIGH
                desc = f"Severe underrepresentation in '{attr}': {min(attr_dist, key=attr_dist.get)} at {min(values):.1%}"
            elif max_ratio > 3 or max_deviation > 0.15:
                severity = Severity.MEDIUM
                desc = f"Moderate imbalance in '{attr}' representation"
            elif max_ratio > 2 or max_deviation > 0.1:
                severity = Severity.LOW
                desc = f"Minor imbalance in '{attr}' representation"
            else:
                continue

            findings.append(BiasFinding(
                dimension=f"demographic_{attr}",
                description=desc,
                severity=severity,
                mitigation=f"Consider stratified sampling or targeted data collection for underrepresented {attr} groups",
                metrics={
                    'attribute': attr,
                    'distribution': attr_dist,
                    'max_ratio': max_ratio,
                    'gini_coefficient': gini
                }
            ))

            # Cross-tabulation with labels (conditional demographic parity)
            cross_tab = pd.crosstab(df[attr], df[label_col], normalize='index')
            for label in cross_tab.columns:
                label_by_attr = cross_tab[label]
                if label_by_attr.std() > 0.1:  # Significant variation
                    findings.append(BiasFinding(
                        dimension=f"conditional_demographic_{attr}_{label}",
                        description=f"Label '{label}' distribution varies significantly across '{attr}' groups",
                        severity=Severity.MEDIUM,
                        mitigation="Review labeling process for potential bias; consider stratified quality checks",
                        metrics={
                            'attribute': attr,
                            'label': label,
                            'distribution_by_group': label_by_attr.to_dict(),
                            'std_deviation': float(label_by_attr.std())
                        }
                    ))

        return {'findings': findings, 'distribution': distribution}

    def _analyze_text_bias(self, df: pd.DataFrame) -> Dict:
        """Analyze linguistic and text-based biases."""
        findings = []
        text_cols = df.select_dtypes(include=['object']).columns

        for col in text_cols:
            # Skip non-text columns
            sample = df[col].dropna().head(100)
            avg_len = sample.str.len().mean()
            if avg_len < 10:  # Probably not text
                continue

            # Language detection bias (simplified)
            # In practice, use langdetect or similar
            non_ascii_ratio = sample.str.count(r'[^\x00-\x7F]').sum() / sample.str.len().sum()

            if non_ascii_ratio < 0.01:
                findings.append(BiasFinding(
                    dimension=f"language_bias_{col}",
                    description=f"Column '{col}' appears to be predominantly ASCII/English text",
                    severity=Severity.LOW,
                    mitigation="Consider if dataset adequately represents non-English content for intended use case",
                    metrics={
                        'column': col,
                        'non_ascii_ratio': non_ascii_ratio
                    }
                ))

            # Length bias
            lengths = df[col].str.len()
            if lengths.std() / lengths.mean() > 1.5:  # High variance
                findings.append(BiasFinding(
                    dimension=f"length_bias_{col}",
                    description=f"High variance in text length for '{col}' (CV={lengths.std()/lengths.mean():.2f})",
                    severity=Severity.LOW,
                    mitigation="Consider if length variations could impact model performance or introduce bias",
                    metrics={
                        'column': col,
                        'mean_length': float(lengths.mean()),
                        'std_length': float(lengths.std()),
                        'cv': float(lengths.std() / lengths.mean())
                    }
                ))

        return {'findings': findings}

    def _analyze_temporal_bias(self, df: pd.DataFrame, label_col: str) -> Dict:
        """Analyze temporal biases."""
        findings = []
        temporal_cols = df.select_dtypes(include=['datetime64']).columns

        for col in temporal_cols:
            timestamps = pd.to_datetime(df[col])

            # Check temporal coverage
            time_range = timestamps.max() - timestamps.min()
            recent_ratio = (timestamps > timestamps.max() - pd.Timedelta(days=30)).mean()

            if recent_ratio > 0.5:
                findings.append(BiasFinding(
                    dimension=f"temporal_recency_{col}",
                    description=f"Data is heavily skewed towards recent dates ({recent_ratio:.1%} from last 30 days)",
                    severity=Severity.MEDIUM,
                    mitigation="Consider if temporal distribution is appropriate for use case",
                    metrics={
                        'column': col,
                        'recent_ratio': recent_ratio,
                        'time_range_days': time_range.days
                    }
                ))

            # Check for label drift over time
            if label_col in df.columns:
                df_temp = df.copy()
                df_temp['_month'] = timestamps.dt.to_period('M')
                monthly_dist = df_temp.groupby('_month')[label_col].value_counts(normalize=True).unstack()

                if monthly_dist.std().mean() > 0.05:
                    findings.append(BiasFinding(
                        dimension=f"temporal_drift_{col}",
                        description="Label distribution changes significantly over time",
                        severity=Severity.HIGH,
                        mitigation="Investigate cause of drift; consider time-aware splitting strategies",
                        metrics={
                            'column': col,
                            'mean_monthly_std': float(monthly_dist.std().mean())
                        }
                    ))

        return {'findings': findings}

    def _calculate_gini(self, values: List[float]) -> float:
        """Calculate Gini coefficient for distribution."""
        sorted_values = sorted(values)
        n = len(sorted_values)
        cumulative = np.cumsum(sorted_values)
        return (2 * np.sum((np.arange(1, n + 1) * sorted_values)) / (n * np.sum(sorted_values))) - (n + 1) / n

    def _has_text_columns(self, df: pd.DataFrame) -> bool:
        """Check if dataframe has text columns."""
        for col in df.select_dtypes(include=['object']).columns:
            if df[col].dropna().str.len().mean() > 20:
                return True
        return False

    def _has_temporal_columns(self, df: pd.DataFrame) -> bool:
        """Check if dataframe has temporal columns."""
        return len(df.select_dtypes(include=['datetime64']).columns) > 0

    def _finding_to_dict(self, finding: BiasFinding) -> Dict:
        """Convert BiasFinding to dictionary."""
        return {
            'dimension': finding.dimension,
            'description': finding.description,
            'severity': finding.severity.value,
            'mitigation': finding.mitigation,
            'metrics': finding.metrics
        }
```

---

## 5. Data Card Generator

```python
# data_card_generator.py
from typing import Dict, List, Optional
import yaml
import json
from datetime import datetime
import hashlib

class DataCardGenerator:
    """Generate comprehensive Data Cards from dataset analysis."""

    def generate(
        self,
        dataset_path: str,
        schema: 'Schema',
        provenance_path: Optional[str] = None,
        quality_report_path: Optional[str] = None,
        bias_report_path: Optional[str] = None,
        **kwargs
    ) -> 'DataCard':
        """Generate a complete Data Card."""

        # Load analysis reports
        quality = self._load_json(quality_report_path) if quality_report_path else {}
        bias = self._load_json(bias_report_path) if bias_report_path else {}
        provenance = self._load_json(provenance_path) if provenance_path else {}

        # Analyze dataset
        stats = self._analyze_dataset(dataset_path, schema)

        # Build Data Card
        data_card = DataCard(
            data_card_version="1.0",
            dataset_id=kwargs.get('dataset_id', self._generate_id(kwargs.get('name', 'unnamed'))),
            name=kwargs['name'],
            version=kwargs['version'],
            description=kwargs.get('description', {}),
            creators=kwargs.get('creators', []),
            license=kwargs.get('license', {}),
            data_types=self._infer_data_types(schema),
            size=stats['size'],
            schema=schema.to_dict() if schema else None,
            splits=stats.get('splits'),
            collection=kwargs.get('collection'),
            annotation=kwargs.get('annotation'),
            quality=self._format_quality(quality),
            bias=self._format_bias(bias),
            privacy=kwargs.get('privacy'),
            provenance=provenance,
            usage=kwargs.get('usage'),
            distribution=self._detect_distribution(dataset_path),
            maintenance=kwargs.get('maintenance'),
            citations=kwargs.get('citations'),
            created_at=datetime.utcnow().isoformat() + 'Z'
        )

        return data_card

    def _generate_id(self, name: str) -> str:
        """Generate unique dataset ID."""
        import uuid
        return str(uuid.uuid5(uuid.NAMESPACE_DNS, f"wia.org/datasets/{name}"))

    def _analyze_dataset(self, path: str, schema: 'Schema') -> Dict:
        """Analyze dataset and compute statistics."""
        import pandas as pd
        import os

        stats = {'size': {}, 'splits': {}}

        # Check for split directories
        for split in ['train', 'validation', 'test']:
            split_path = os.path.join(path, split)
            if os.path.exists(split_path):
                split_stats = self._analyze_split(split_path)
                stats['splits'][split] = split_stats
                stats['size']['samples'] = stats['size'].get('samples', 0) + split_stats['samples']

        # If no splits, analyze root
        if not stats['splits']:
            stats['size'] = self._analyze_split(path)

        # Calculate file size
        total_size = 0
        for root, dirs, files in os.walk(path):
            for f in files:
                total_size += os.path.getsize(os.path.join(root, f))

        stats['size']['fileSize'] = {
            'value': round(total_size / (1024**3), 2),
            'unit': 'GB'
        }

        return stats

    def _analyze_split(self, path: str) -> Dict:
        """Analyze a single data split."""
        import pandas as pd
        import os

        samples = 0
        for f in os.listdir(path):
            if f.endswith('.parquet'):
                df = pd.read_parquet(os.path.join(path, f))
                samples += len(df)
            elif f.endswith('.csv'):
                # Count lines without loading full file
                with open(os.path.join(path, f), 'r') as file:
                    samples += sum(1 for _ in file) - 1

        return {'samples': samples}

    def _infer_data_types(self, schema: 'Schema') -> List[str]:
        """Infer data types from schema."""
        types = set()
        type_mapping = {
            'string': 'text',
            'binary': 'image',  # Could also be audio/video
            'float': 'tabular',
            'integer': 'tabular',
            'datetime': 'time-series'
        }

        if schema:
            for feature in schema.features:
                mapped = type_mapping.get(feature.type, 'other')
                types.add(mapped)

        return list(types) or ['other']

    def _format_quality(self, quality: Dict) -> Dict:
        """Format quality report for Data Card."""
        if not quality:
            return {'assessed': False}

        return {
            'overallScore': quality.get('overall_score', quality.get('score', 0)) * 10,
            'dimensions': {
                'accuracy': quality.get('accuracy', {}).get('score', 0),
                'completeness': quality.get('completeness', {}).get('score', 0),
                'consistency': quality.get('consistency', {}).get('score', 0),
                'timeliness': quality.get('timeliness', {}).get('score', 0)
            },
            'validation': {
                'method': quality.get('methodology', 'automated'),
                'date': quality.get('assessment_date', datetime.utcnow().isoformat())
            }
        }

    def _format_bias(self, bias: Dict) -> Dict:
        """Format bias report for Data Card."""
        if not bias:
            return {'assessed': False}

        return {
            'assessed': True,
            'assessmentDate': bias.get('assessment_date', datetime.utcnow().isoformat()),
            'methodology': bias.get('methodology', 'WIA Bias Assessment Framework'),
            'findings': bias.get('findings', []),
            'demographics': bias.get('demographic_distribution', {})
        }

    def _detect_distribution(self, path: str) -> Dict:
        """Detect distribution format."""
        import os

        formats = []
        for f in os.listdir(path):
            if f.endswith('.parquet'):
                formats.append('parquet')
            elif f.endswith('.csv'):
                formats.append('csv')
            elif f.endswith('.json') or f.endswith('.jsonl'):
                formats.append('jsonl')
            elif f.endswith('.tfrecord'):
                formats.append('tfrecord')

        return {
            'format': formats[0] if formats else 'unknown',
            'location': {
                'type': 'local',
                'uri': path
            }
        }

    def _load_json(self, path: str) -> Dict:
        """Load JSON file."""
        with open(path, 'r') as f:
            return json.load(f)
```

---

## 6. Testing Framework

### 6.1 Unit Tests

```python
# tests/test_quality.py
import pytest
import pandas as pd
import numpy as np
from wia_training_data import QualityAssessor, QualityConfig

class TestQualityAssessor:
    @pytest.fixture
    def sample_df(self):
        return pd.DataFrame({
            'text': ['Hello world', 'Test message', 'Another text', ''],
            'label': [0, 1, 0, 1],
            'score': [0.9, 0.85, 0.95, None]
        })

    @pytest.fixture
    def assessor(self):
        return QualityAssessor()

    def test_completeness_detection(self, assessor, sample_df):
        """Test that missing values are detected."""
        result = assessor.assess_completeness(sample_df)

        assert result.feature_coverage < 1.0
        assert result.sample_completeness < 1.0

    def test_empty_string_detection(self, assessor, sample_df):
        """Test that empty strings are detected."""
        result = assessor.assess_consistency(sample_df)

        assert 'empty_strings' in result.issues
        assert result.issues['empty_strings']['count'] == 1

    def test_overall_score_calculation(self, assessor, sample_df):
        """Test overall quality score calculation."""
        report = assessor.assess(sample_df)

        assert 0 <= report.overall_score <= 10
        assert report.accuracy is not None
        assert report.completeness is not None

    def test_perfect_data(self, assessor):
        """Test scoring of perfect data."""
        perfect_df = pd.DataFrame({
            'text': ['Hello', 'World', 'Test'],
            'label': [0, 1, 2]
        })

        report = assessor.assess(perfect_df)

        assert report.completeness.feature_coverage == 1.0
        assert report.completeness.sample_completeness == 1.0


# tests/test_bias.py
class TestBiasDetector:
    @pytest.fixture
    def imbalanced_df(self):
        return pd.DataFrame({
            'text': ['a'] * 90 + ['b'] * 10,
            'label': [0] * 90 + [1] * 10,
            'gender': ['M'] * 80 + ['F'] * 20
        })

    @pytest.fixture
    def detector(self):
        from wia_training_data import BiasDetector, BiasConfig
        return BiasDetector(BiasConfig(
            protected_attributes=['gender']
        ))

    def test_class_imbalance_detection(self, detector, imbalanced_df):
        """Test detection of class imbalance."""
        report = detector.analyze(imbalanced_df, 'label')

        imbalance_findings = [f for f in report['findings']
                            if f['dimension'] == 'class_imbalance']
        assert len(imbalance_findings) > 0
        assert imbalance_findings[0]['severity'] == 'high'

    def test_demographic_imbalance(self, detector, imbalanced_df):
        """Test detection of demographic imbalance."""
        report = detector.analyze(imbalanced_df, 'label')

        demo_findings = [f for f in report['findings']
                        if 'demographic' in f['dimension']]
        assert len(demo_findings) > 0

    def test_balanced_data(self, detector):
        """Test that balanced data produces no high-severity findings."""
        balanced_df = pd.DataFrame({
            'text': ['a', 'b'] * 50,
            'label': [0, 1] * 50,
            'gender': ['M', 'F'] * 50
        })

        report = detector.analyze(balanced_df, 'label')

        high_severity = [f for f in report['findings']
                        if f['severity'] == 'high']
        assert len(high_severity) == 0
```

---

## 7. CLI Tools

```bash
#!/bin/bash
# wia-data CLI tool usage

# Validate Data Card
wia-data validate ./DATA_CARD.yaml

# Assess quality
wia-data quality assess ./data/ --output ./quality/report.json

# Run bias detection
wia-data bias analyze ./data/ \
  --label-column category \
  --protected-attributes gender,age_group \
  --output ./quality/bias_report.json

# Generate Data Card
wia-data generate \
  --dataset ./data/ \
  --name "my-dataset" \
  --version "1.0.0" \
  --output ./DATA_CARD.yaml

# Verify provenance
wia-data provenance verify ./provenance/lineage.json

# Convert formats
wia-data convert ./data/train.csv ./data/train.parquet

# Check compliance
wia-data compliance check ./DATA_CARD.yaml --standard wia-v1.0
```

---

## 8. Best Practices

### 8.1 Data Collection

1. **Document everything**: Record methodology, sources, timeframes
2. **Obtain proper consent**: Ensure legal basis for data collection
3. **Plan for diversity**: Consider representation from the start
4. **Version control**: Track all changes to collection processes

### 8.2 Quality Management

1. **Continuous monitoring**: Don't just assess once
2. **Set thresholds**: Define minimum quality requirements
3. **Automate checks**: Integrate into CI/CD pipelines
4. **Fix issues early**: Address quality problems before training

### 8.3 Bias Mitigation

1. **Assess before training**: Catch issues in data, not model
2. **Multiple perspectives**: Include diverse reviewers
3. **Intersectional analysis**: Look at attribute combinations
4. **Document limitations**: Be transparent about known biases

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2026-01-13 | WIA Standards Committee | Initial release |

---

© 2026 WIA (World Certification Industry Association) / SmileStory Inc.
**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
