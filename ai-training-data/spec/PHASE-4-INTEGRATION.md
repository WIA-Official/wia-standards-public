# WIA-AI-007 PHASE 4: Integration & Ecosystem Specification

**Version:** 1.0.0
**Status:** Stable
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Overview

Phase 4 defines integration with ML frameworks, cloud platforms, data lakes, labeling tools, and the complete ecosystem for AI training data lifecycle management.

## ML Framework Integration

### PyTorch Integration

```python
from wia_ai_007.integrations import PyTorchDataset
import torch
from torch.utils.data import DataLoader

# Load WIA-AI-007 compliant dataset
dataset = PyTorchDataset.from_wia(
    dataset_id="medical-images-v2",
    version="2.0.0",
    split="train",
    transforms=transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                           std=[0.229, 0.224, 0.225])
    ])
)

# Standard PyTorch DataLoader
dataloader = DataLoader(
    dataset,
    batch_size=32,
    shuffle=True,
    num_workers=4
)

# Training loop
for epoch in range(10):
    for batch_idx, (data, labels) in enumerate(dataloader):
        # Standard PyTorch training
        output = model(data)
        loss = criterion(output, labels)
        loss.backward()
        optimizer.step()

# 弘益人間 - Seamless PyTorch integration
```

### TensorFlow Integration

```python
from wia_ai_007.integrations import TensorFlowDataset
import tensorflow as tf

# Load as tf.data.Dataset
dataset = TensorFlowDataset.from_wia(
    dataset_id="medical-images-v2",
    version="2.0.0",
    split="train"
)

# Apply transformations
dataset = dataset.map(
    lambda x, y: (preprocess_image(x), y),
    num_parallel_calls=tf.data.AUTOTUNE
)

dataset = dataset.batch(32).prefetch(tf.data.AUTOTUNE)

# Training
model.fit(
    dataset,
    epochs=10,
    validation_data=val_dataset
)
```

### Hugging Face Integration

```python
from wia_ai_007.integrations import HuggingFaceDataset
from datasets import load_dataset

# Load as Hugging Face dataset
hf_dataset = HuggingFaceDataset.from_wia(
    dataset_id="nlp-corpus-v1",
    version="1.0.0"
)

# Use with transformers
from transformers import Trainer, TrainingArguments

training_args = TrainingArguments(
    output_dir="./results",
    num_train_epochs=3,
    per_device_train_batch_size=16
)

trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=hf_dataset["train"],
    eval_dataset=hf_dataset["validation"]
)

trainer.train()
```

## Cloud Platform Integration

### AWS S3 Integration

```python
from wia_ai_007.storage import S3Storage

# Configure S3 backend
storage = S3Storage(
    bucket="my-training-data",
    region="us-east-1",
    credentials_provider="iam_role"
)

# Upload dataset
storage.upload_dataset(
    dataset=local_dataset,
    dataset_id="medical-images-v2",
    encryption="AES256",
    storage_class="INTELLIGENT_TIERING"
)

# Download dataset
dataset = storage.download_dataset(
    dataset_id="medical-images-v2",
    version="2.0.0",
    local_path="./data"
)

# Versioning with S3
storage.enable_versioning(bucket="my-training-data")
```

### Google Cloud Storage Integration

```python
from wia_ai_007.storage import GCSStorage

storage = GCSStorage(
    bucket="my-training-data",
    project="my-gcp-project",
    credentials="./service-account.json"
)

storage.upload_dataset(
    dataset=local_dataset,
    dataset_id="medical-images-v2",
    storage_class="STANDARD"
)
```

### Azure Blob Storage Integration

```python
from wia_ai_007.storage import AzureBlobStorage

storage = AzureBlobStorage(
    account_name="myaccount",
    container="training-data",
    sas_token=os.getenv("AZURE_SAS_TOKEN")
)

storage.upload_dataset(
    dataset=local_dataset,
    dataset_id="medical-images-v2",
    tier="Hot"
)
```

## Data Lake Integration

### Delta Lake

```python
from wia_ai_007.integrations import DeltaLakeIntegration
from delta import DeltaTable

# Write to Delta Lake
delta_integration = DeltaLakeIntegration(
    lake_path="s3://my-data-lake/datasets"
)

delta_integration.write_dataset(
    dataset=training_dataset,
    table_name="medical_images",
    mode="overwrite",
    partition_by=["year", "month"]
)

# Time travel with Delta Lake
historical_data = delta_integration.read_dataset(
    table_name="medical_images",
    version_as_of=10  # Version 10
)

# Vacuum old versions
delta_integration.vacuum(
    table_name="medical_images",
    retention_hours=168  # 7 days
)
```

### Apache Iceberg

```python
from wia_ai_007.integrations import IcebergIntegration

iceberg = IcebergIntegration(
    catalog_uri="thrift://localhost:9083",
    warehouse="s3://my-data-lake"
)

# Create Iceberg table
iceberg.create_table(
    table_name="datasets.medical_images",
    schema=dataset_schema,
    partition_spec=["year", "month"]
)

# Write dataset
iceberg.write_dataset(
    dataset=training_dataset,
    table_name="datasets.medical_images"
)

# Snapshot management
snapshot = iceberg.get_snapshot(
    table_name="datasets.medical_images",
    snapshot_id=12345
)
```

## Labeling Tool Integration

### Label Studio

```python
from wia_ai_007.integrations import LabelStudioIntegration

label_studio = LabelStudioIntegration(
    url="http://localhost:8080",
    api_key=os.getenv("LABEL_STUDIO_API_KEY")
)

# Create labeling project
project = label_studio.create_project(
    name="Medical Image Annotation",
    label_config="""
    <View>
      <Image name="image" value="$image"/>
      <Choices name="diagnosis" toName="image">
        <Choice value="normal"/>
        <Choice value="abnormal"/>
      </Choices>
    </View>
    """
)

# Import data for labeling
label_studio.import_tasks(
    project_id=project.id,
    dataset_id="medical-images-raw",
    sampling="random",
    sample_size=1000
)

# Export completed annotations
annotations = label_studio.export_annotations(
    project_id=project.id,
    format="wia-ai-007"
)

# Update dataset with annotations
dataset.update_labels(annotations)
```

### CVAT Integration

```python
from wia_ai_007.integrations import CVATIntegration

cvat = CVATIntegration(
    url="http://localhost:8080",
    username="admin",
    password=os.getenv("CVAT_PASSWORD")
)

# Create annotation task
task = cvat.create_task(
    name="Object Detection Task",
    labels=["car", "pedestrian", "cyclist"],
    dataset_id="autonomous-driving-v1"
)

# Export annotations
annotations = cvat.export_annotations(
    task_id=task.id,
    format="coco"
)
```

## MLOps Platform Integration

### MLflow

```python
from wia_ai_007.integrations import MLflowIntegration
import mlflow

mlflow_integration = MLflowIntegration(
    tracking_uri="http://localhost:5000"
)

# Log dataset metadata
with mlflow.start_run():
    mlflow_integration.log_dataset(
        dataset_id="medical-images-v2",
        version="2.0.0",
        metadata={
            "quality_score": 0.96,
            "samples": 100000,
            "philosophy": "弘익人間"
        }
    )

    # Log data lineage
    mlflow_integration.log_lineage(
        dataset_id="medical-images-v2",
        parent_datasets=["raw-images-v1"],
        transformations=["normalization", "augmentation"]
    )

    # Log quality metrics
    mlflow.log_metrics({
        "completeness": 0.985,
        "consistency": 0.992,
        "accuracy": 0.94
    })
```

### Weights & Biases

```python
from wia_ai_007.integrations import WandBIntegration
import wandb

wandb_integration = WandBIntegration(
    project="medical-diagnosis",
    entity="research-team"
)

# Log dataset as artifact
with wandb.init(project="medical-diagnosis"):
    dataset_artifact = wandb_integration.create_artifact(
        dataset_id="medical-images-v2",
        version="2.0.0",
        type="dataset"
    )

    # Add metadata
    dataset_artifact.add_metadata({
        "quality_score": 0.96,
        "bias_score": 0.15,
        "philosophy": "弘익人間"
    })

    wandb.log_artifact(dataset_artifact)
```

## Data Catalog Integration

### Apache Atlas

```python
from wia_ai_007.integrations import AtlasIntegration

atlas = AtlasIntegration(
    url="http://localhost:21000",
    username="admin",
    password=os.getenv("ATLAS_PASSWORD")
)

# Register dataset in catalog
atlas.register_dataset(
    dataset_id="medical-images-v2",
    metadata={
        "name": "Medical Images v2",
        "type": "training_dataset",
        "owner": "ml-team",
        "classification": "confidential",
        "tags": ["medical", "imaging", "diagnosis"]
    }
)

# Track lineage
atlas.track_lineage(
    dataset_id="medical-images-v2",
    parent_datasets=["raw-images-v1"],
    transformations=["preprocessing-pipeline-v1"]
)
```

## Compute Platform Integration

### Apache Spark

```python
from wia_ai_007.integrations import SparkIntegration
from pyspark.sql import SparkSession

spark = SparkSession.builder.appName("WIA-AI-007").getOrCreate()

spark_integration = SparkIntegration(spark)

# Read dataset as Spark DataFrame
df = spark_integration.read_dataset(
    dataset_id="tabular-data-v1",
    version="1.0.0",
    format="parquet"
)

# Process with Spark
processed_df = df.filter(df.age > 18) \
                 .groupBy("category") \
                 .count()

# Write back to WIA-AI-007
spark_integration.write_dataset(
    dataframe=processed_df,
    dataset_id="processed-tabular-v1",
    version="1.0.0"
)
```

### Ray

```python
from wia_ai_007.integrations import RayIntegration
import ray

ray.init()

ray_integration = RayIntegration()

# Distributed data processing
@ray.remote
def process_batch(batch):
    # Process batch
    return processed_batch

# Load and process dataset
dataset = ray_integration.load_dataset(
    dataset_id="large-dataset-v1",
    version="1.0.0"
)

futures = [process_batch.remote(batch) for batch in dataset.batches()]
results = ray.get(futures)
```

## Monitoring and Observability

### Prometheus Metrics

```python
from wia_ai_007.monitoring import PrometheusExporter

exporter = PrometheusExporter(port=9090)

# Export dataset metrics
exporter.register_dataset_metrics(
    dataset_id="medical-images-v2",
    metrics=[
        "quality_score",
        "sample_count",
        "storage_size",
        "access_count"
    ]
)

# Custom metrics
exporter.gauge("dataset_quality_score", 0.96, labels={"dataset": "medical-images-v2"})
exporter.counter("dataset_access_total", labels={"dataset": "medical-images-v2"})
```

## Best Practices

1. **Use native integrations**: Leverage framework-specific optimizations
2. **Cache datasets**: Implement caching for frequently accessed datasets
3. **Lazy loading**: Load data only when needed
4. **Parallel processing**: Use distributed processing for large datasets
5. **Monitor performance**: Track dataset access patterns and performance
6. **Version everything**: Include framework versions in metadata
7. **Test integrations**: Validate integrations work across updates
8. **Document dependencies**: Clear documentation of required libraries

---

**弘益人間** · Benefit All Humanity
© 2025 SmileStory Inc. / WIA
WIA-AI-007 AI Training Data Standard v1.0

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
