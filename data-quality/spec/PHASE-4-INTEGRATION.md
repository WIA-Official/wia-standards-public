# WIA-DATA-005: Data Quality - Phase 4 Integration Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This document defines integration patterns and specifications for connecting WIA-DATA-005 Data Quality standard with existing data ecosystems, tools, and platforms.

## Integration Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Data Sources                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Database │  │   APIs   │  │  Files   │  │ Streams  │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
└───────┼─────────────┼─────────────┼─────────────┼──────────┘
        │             │             │             │
        └─────────────┴─────────────┴─────────────┘
                            │
        ┌───────────────────▼────────────────────┐
        │    WIA-DATA-005 Quality Engine         │
        │  ┌──────────────────────────────────┐  │
        │  │   Profiling   │   Validation    │  │
        │  ├──────────────────────────────────┤  │
        │  │  Cleansing    │   Monitoring    │  │
        │  └──────────────────────────────────┘  │
        └───────────────────┬────────────────────┘
                            │
        ┌───────────────────▼────────────────────┐
        │         Integration Layer               │
        │  ┌──────────────────────────────────┐  │
        │  │  ETL  │  BI  │  ML  │  Storage  │  │
        │  └──────────────────────────────────┘  │
        └────────────────────────────────────────┘
```

## Data Source Integrations

### 1. Relational Databases

#### JDBC Connection

```yaml
connection:
  type: jdbc
  driver: postgresql
  url: jdbc:postgresql://localhost:5432/mydb
  username: ${DB_USER}
  password: ${DB_PASSWORD}
  properties:
    ssl: true
    poolSize: 10
```

#### Profiling Configuration

```yaml
profiling:
  source:
    type: database
    connection: mydb
    tables:
      - customers
      - orders
  schedule: "0 2 * * *"  # Daily at 2 AM
  sampling:
    method: random
    size: 10000
```

### 2. Cloud Data Warehouses

#### Snowflake Integration

```python
from wia_data_quality import QualityEngine

engine = QualityEngine()
engine.connect_snowflake(
    account="myaccount",
    warehouse="COMPUTE_WH",
    database="ANALYTICS",
    schema="PUBLIC",
    role="DATA_QUALITY"
)

profile = engine.profile_table("CUSTOMERS")
```

#### BigQuery Integration

```python
engine.connect_bigquery(
    project_id="my-project",
    dataset_id="analytics",
    credentials_path="/path/to/credentials.json"
)

engine.validate_table(
    "customers",
    rules=[
        {"column": "email", "type": "format", "pattern": "email"},
        {"column": "age", "type": "range", "min": 18, "max": 120}
    ]
)
```

### 3. Streaming Platforms

#### Apache Kafka

```yaml
kafka:
  bootstrap_servers:
    - kafka1:9092
    - kafka2:9092
  topics:
    input: raw-events
    output: quality-checked-events
  consumer_group: dq-consumer
  quality_checks:
    - schema_validation
    - null_check
    - format_validation
```

#### Apache Flink Integration

```java
StreamExecutionEnvironment env = StreamExecutionEnvironment.getExecutionEnvironment();

DataStream<Event> events = env.addSource(new KafkaSource<>(...));

DataStream<QualityResult> validated = events
    .map(new QualityValidationFunction())
    .filter(result -> result.isValid());

validated.addSink(new KafkaSink<>(...));
```

### 4. Object Storage

#### AWS S3

```yaml
s3:
  bucket: my-data-bucket
  region: us-east-1
  credentials:
    access_key: ${AWS_ACCESS_KEY}
    secret_key: ${AWS_SECRET_KEY}
  profiling:
    file_pattern: "data/*.csv"
    scan_frequency: hourly
```

## ETL Tool Integrations

### Apache Airflow

```python
from airflow import DAG
from airflow.operators.python import PythonOperator
from wia_data_quality.airflow import DataQualityOperator

dag = DAG('data_quality_pipeline', schedule_interval='@daily')

profile_task = DataQualityOperator(
    task_id='profile_customers',
    dataset='customers',
    operation='profile',
    dag=dag
)

validate_task = DataQualityOperator(
    task_id='validate_customers',
    dataset='customers',
    operation='validate',
    rules_file='rules/customers.yaml',
    dag=dag
)

profile_task >> validate_task
```

### dbt Integration

```yaml
# schema.yml
version: 2

models:
  - name: customers
    tests:
      - wia_data_quality.completeness:
          column: email
          threshold: 95
      - wia_data_quality.uniqueness:
          column: customer_id
      - wia_data_quality.validity:
          column: email
          pattern: email
```

### Apache Spark

```scala
import com.wia.dataquality.spark._

val df = spark.read.parquet("s3://bucket/data")

val qualityReport = df
  .withQualityProfile()
  .withValidation("rules.json")
  .checkQuality()

qualityReport
  .filter("status = 'FAIL'")
  .write.parquet("s3://bucket/quality-issues")
```

## BI Tool Integrations

### Tableau

```javascript
// Tableau Web Data Connector
(function() {
    var myConnector = tableau.makeConnector();

    myConnector.getSchema = function(schemaCallback) {
        var cols = [
            {id: "metric_name", dataType: tableau.dataTypeEnum.string},
            {id: "value", dataType: tableau.dataTypeEnum.float},
            {id: "timestamp", dataType: tableau.dataTypeEnum.datetime}
        ];

        var tableSchema = {
            id: "wiaDataQuality",
            alias: "WIA Data Quality Metrics",
            columns: cols
        };

        schemaCallback([tableSchema]);
    };

    myConnector.getData = function(table, doneCallback) {
        $.getJSON("https://api.example.com/wia/data-quality/v1/metrics", function(resp) {
            var tableData = resp.data.metrics;
            table.appendRows(tableData);
            doneCallback();
        });
    };

    tableau.registerConnector(myConnector);
})();
```

### Power BI

```csharp
// Power BI Custom Connector
[DataSource.Kind="WIADataQuality", Publish="WIADataQuality.Publish"]
shared WIADataQuality.Contents = (url as text) =>
    let
        source = Web.Contents(url & "/metrics"),
        json = Json.Document(source),
        data = json[data][metrics],
        table = Table.FromList(data, Splitter.SplitByNothing(), null, null, ExtraValues.Error)
    in
        table;
```

## ML Platform Integrations

### TensorFlow Data Validation (TFDV)

```python
import tensorflow_data_validation as tfdv
from wia_data_quality.ml import TFDVIntegration

# Generate statistics
stats = tfdv.generate_statistics_from_csv('data.csv')

# Convert to WIA format
wia_integration = TFDVIntegration()
wia_profile = wia_integration.convert_statistics(stats)

# Upload to WIA platform
wia_integration.upload_profile(wia_profile)
```

### MLflow

```python
import mlflow
from wia_data_quality.ml import log_quality_metrics

with mlflow.start_run():
    # Train model
    model = train_model(X_train, y_train)
    
    # Log data quality metrics
    log_quality_metrics(
        dataset=X_train,
        metrics=['completeness', 'validity', 'consistency']
    )
    
    mlflow.sklearn.log_model(model, "model")
```

## Monitoring and Observability

### Prometheus Integration

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'wia-data-quality'
    static_configs:
      - targets: ['quality-api:8080']
    metrics_path: '/metrics'
    scrape_interval: 30s
```

Exposed metrics:

```
# HELP dq_validation_pass_rate Percentage of records passing validation
# TYPE dq_validation_pass_rate gauge
dq_validation_pass_rate{dataset="customers",dimension="completeness"} 95.5

# HELP dq_issue_count Number of open quality issues
# TYPE dq_issue_count gauge
dq_issue_count{severity="high"} 5
```

### Grafana Dashboard

```json
{
  "dashboard": {
    "title": "Data Quality Overview",
    "panels": [
      {
        "title": "Quality Score by Dimension",
        "targets": [
          {
            "expr": "avg(dq_dimension_score) by (dimension)",
            "legendFormat": "{{dimension}}"
          }
        ],
        "type": "graph"
      }
    ]
  }
}
```

### Datadog Integration

```yaml
init_config:

instances:
  - url: https://api.example.com/wia/data-quality/v1
    api_key: ${DD_API_KEY}
    collect_default_metrics: true
    tags:
      - env:production
      - team:data
```

## Data Catalog Integration

### Apache Atlas

```python
from wia_data_quality.catalog import AtlasIntegration

atlas = AtlasIntegration(
    host="atlas.example.com",
    port=21000,
    username="admin"
)

# Register quality profile as asset
atlas.register_quality_profile(
    dataset="customers",
    profile_id="uuid",
    lineage=True
)
```

### Collibra

```python
from wia_data_quality.catalog import CollibraIntegration

collibra = CollibraIntegration(url="https://collibra.example.com")

# Sync quality metrics to business glossary
collibra.sync_quality_metrics(
    dataset="customers",
    community_id="uuid",
    domain_id="uuid"
)
```

## Authentication and Authorization

### SSO Integration (SAML)

```xml
<EntityDescriptor>
  <SPSSODescriptor>
    <AssertionConsumerService
      Binding="urn:oasis:names:tc:SAML:2.0:bindings:HTTP-POST"
      Location="https://api.example.com/wia/dq/saml/acs"
      index="0"/>
  </SPSSODescriptor>
</EntityDescriptor>
```

### LDAP Integration

```yaml
ldap:
  url: ldap://ldap.example.com:389
  bind_dn: cn=admin,dc=example,dc=com
  user_search_base: ou=users,dc=example,dc=com
  group_search_base: ou=groups,dc=example,dc=com
  role_mapping:
    data_quality_admin: ROLE_ADMIN
    data_steward: ROLE_STEWARD
```

## Best Practices

1. **Use Connection Pooling**: Maintain persistent connections to data sources
2. **Implement Circuit Breakers**: Protect against cascade failures
3. **Cache Frequently Accessed Data**: Reduce load on source systems
4. **Monitor Integration Health**: Track connection status and errors
5. **Version Control Configurations**: Treat integration configs as code
6. **Test in Isolation**: Validate integrations independently
7. **Document Dependencies**: Maintain up-to-date integration documentation

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘익人間 (홍익인간) · Benefit All Humanity**
