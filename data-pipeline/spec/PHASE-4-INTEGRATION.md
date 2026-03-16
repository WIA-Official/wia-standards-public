# WIA-DATA-004: PHASE 4 - INTEGRATION

## Overview

This phase defines integration patterns with existing systems and best practices for production deployment.

## 1. Common Integrations

### 1.1 Apache Airflow Integration

```python
from airflow import DAG
from airflow.operators.python import PythonOperator
from wia_data_pipeline import WIAPipelineOperator
from datetime import datetime, timedelta

default_args = {
    'owner': 'data-team',
    'depends_on_past': False,
    'start_date': datetime(2025, 1, 1),
    'email_on_failure': True,
    'email_on_retry': False,
    'retries': 3,
    'retry_delay': timedelta(minutes=5)
}

with DAG(
    'wia_daily_sales_pipeline',
    default_args=default_args,
    schedule_interval='0 2 * * *',
    catchup=False
) as dag:

    extract = WIAPipelineOperator(
        task_id='extract_orders',
        pipeline_config={
            'source': {'type': 'postgresql', 'query': 'SELECT * FROM orders'},
            'destination': {'type': 's3', 'location': 's3://bucket/raw/'}
        }
    )

    transform = WIAPipelineOperator(
        task_id='transform_orders',
        pipeline_config={
            'source': {'type': 's3', 'location': 's3://bucket/raw/'},
            'transformation': 'clean_and_aggregate',
            'destination': {'type': 's3', 'location': 's3://bucket/clean/'}
        }
    )

    load = WIAPipelineOperator(
        task_id='load_warehouse',
        pipeline_config={
            'source': {'type': 's3', 'location': 's3://bucket/clean/'},
            'destination': {'type': 'snowflake', 'table': 'daily_sales'}
        }
    )

    extract >> transform >> load
```

### 1.2 dbt Integration

```yaml
# profiles.yml
wia_pipeline:
  target: prod
  outputs:
    prod:
      type: wia_data_pipeline
      connection_string: "wia://api.example.com"
      api_key: "{{ env_var('WIA_API_KEY') }}"
      warehouse: snowflake
```

```sql
-- models/staging/stg_orders.sql
{{
  config(
    materialized='incremental',
    unique_key='order_id',
    wia_quality_checks=[
      {'type': 'not_null', 'column': 'order_id'},
      {'type': 'range', 'column': 'total', 'min': 0}
    ]
  )
}}

SELECT
  order_id,
  user_id,
  total,
  created_at
FROM {{ source('raw', 'orders') }}
{% if is_incremental() %}
WHERE created_at > (SELECT MAX(created_at) FROM {{ this }})
{% endif %}
```

### 1.3 Kafka Integration

```python
from kafka import KafkaConsumer, KafkaProducer
from wia_data_pipeline import StreamProcessor

# Consumer
consumer = KafkaConsumer(
    'user-events',
    bootstrap_servers=['localhost:9092'],
    group_id='wia-pipeline',
    auto_offset_reset='earliest',
    enable_auto_commit=False
)

# Producer
producer = KafkaProducer(
    bootstrap_servers=['localhost:9092'],
    value_serializer=lambda v: json.dumps(v).encode('utf-8')
)

# WIA Stream Processor
processor = StreamProcessor(
    input_stream=consumer,
    output_stream=producer,
    transformations=[
        'clean_data',
        'enrich_with_user_profile',
        'aggregate_hourly'
    ],
    quality_checks=[
        {'type': 'schema_validation'},
        {'type': 'not_null', 'column': 'user_id'}
    ]
)

processor.run()
```

### 1.4 Spark Integration

```python
from pyspark.sql import SparkSession
from wia_data_pipeline.spark import WIADataSource

spark = SparkSession.builder \
    .appName("WIA Pipeline") \
    .config("spark.jars.packages", "com.wia:wia-spark:1.0.0") \
    .getOrCreate()

# Read via WIA
df = spark.read \
    .format("wia") \
    .option("pipeline_id", "pip_abc123") \
    .option("api_key", api_key) \
    .load()

# Transform
df_clean = df.filter(df.total > 0) \
    .groupBy("date", "user_id") \
    .agg(sum("total").alias("revenue"))

# Write via WIA
df_clean.write \
    .format("wia") \
    .option("pipeline_id", "pip_xyz789") \
    .option("quality_checks", "true") \
    .save()
```

## 2. Cloud Platform Integration

### 2.1 AWS Integration

```python
import boto3
from wia_data_pipeline import AWSIntegration

# S3 Integration
wia = AWSIntegration(
    aws_access_key_id='xxx',
    aws_secret_access_key='yyy',
    region='us-east-1'
)

# Extract from RDS
wia.extract(
    source='rds://prod-db.region.rds.amazonaws.com/myapp',
    destination='s3://bucket/raw/orders/'
)

# Transform with Glue
wia.transform(
    source='s3://bucket/raw/',
    transformation_job='glue_job_clean_orders',
    destination='s3://bucket/clean/'
)

# Load to Redshift
wia.load(
    source='s3://bucket/clean/',
    destination='redshift://cluster.region.redshift.amazonaws.com/warehouse'
)
```

### 2.2 GCP Integration

```python
from wia_data_pipeline import GCPIntegration

wia = GCPIntegration(
    project_id='my-project',
    credentials_path='/path/to/credentials.json'
)

# Extract from Cloud SQL
wia.extract(
    source='cloudsql://instance/database',
    destination='gs://bucket/raw/'
)

# Transform with Dataflow
wia.transform(
    source='gs://bucket/raw/',
    transformation='dataflow_job_clean',
    destination='gs://bucket/clean/'
)

# Load to BigQuery
wia.load(
    source='gs://bucket/clean/',
    destination='bigquery://project.dataset.table'
)
```

### 2.3 Azure Integration

```python
from wia_data_pipeline import AzureIntegration

wia = AzureIntegration(
    subscription_id='xxx',
    tenant_id='yyy',
    client_id='zzz',
    client_secret='aaa'
)

# Extract from Azure SQL
wia.extract(
    source='azuresql://server.database.windows.net/mydb',
    destination='abfs://container@storage.dfs.core.windows.net/raw/'
)

# Transform with Databricks
wia.transform(
    source='abfs://container@storage.dfs.core.windows.net/raw/',
    transformation='databricks_notebook',
    destination='abfs://container@storage.dfs.core.windows.net/clean/'
)

# Load to Synapse
wia.load(
    source='abfs://container@storage.dfs.core.windows.net/clean/',
    destination='synapse://workspace.sql.azuresynapse.net/pool'
)
```

## 3. Monitoring Integration

### 3.1 Prometheus Integration

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'wia_pipelines'
    static_configs:
      - targets: ['localhost:9090']
    metrics_path: '/api/v1/metrics'
    params:
      format: ['prometheus']
```

```python
from prometheus_client import Counter, Histogram, Gauge
from wia_data_pipeline import metrics

# Custom metrics
records_processed = Counter(
    'wia_records_processed_total',
    'Total records processed',
    ['pipeline_id', 'stage']
)

processing_duration = Histogram(
    'wia_processing_duration_seconds',
    'Time spent processing',
    ['pipeline_id']
)

pipeline_lag = Gauge(
    'wia_pipeline_lag_seconds',
    'Pipeline lag behind real-time',
    ['pipeline_id']
)
```

### 3.2 Grafana Dashboard

```json
{
  "dashboard": {
    "title": "WIA Data Pipeline",
    "panels": [
      {
        "title": "Records Processed",
        "targets": [
          {
            "expr": "rate(wia_records_processed_total[5m])",
            "legendFormat": "{{pipeline_id}}"
          }
        ]
      },
      {
        "title": "Pipeline Lag",
        "targets": [
          {
            "expr": "wia_pipeline_lag_seconds",
            "legendFormat": "{{pipeline_id}}"
          }
        ]
      }
    ]
  }
}
```

### 3.3 Datadog Integration

```python
from datadog import initialize, statsd
from wia_data_pipeline import DatadogIntegration

options = {
    'api_key': 'xxx',
    'app_key': 'yyy'
}

initialize(**options)

# Send metrics
statsd.increment('wia.pipeline.run', tags=['pipeline:daily_sales'])
statsd.histogram('wia.pipeline.duration', 930, tags=['pipeline:daily_sales'])
statsd.gauge('wia.pipeline.lag', 120, tags=['pipeline:daily_sales'])
```

## 4. CI/CD Integration

### 4.1 GitHub Actions

```yaml
# .github/workflows/deploy-pipeline.yml
name: Deploy Data Pipeline

on:
  push:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2

      - name: Set up Python
        uses: actions/setup-python@v2
        with:
          python-version: '3.9'

      - name: Install dependencies
        run: |
          pip install wia-data-pipeline
          pip install pytest great-expectations

      - name: Run tests
        run: pytest tests/

      - name: Validate pipeline config
        run: wia validate pipeline.yaml

      - name: Run quality checks
        run: great_expectations checkpoint run daily_sales

  deploy:
    needs: test
    runs-on: ubuntu-latest
    steps:
      - name: Deploy to production
        run: |
          wia deploy \
            --pipeline-config pipeline.yaml \
            --environment production \
            --api-key ${{ secrets.WIA_API_KEY }}
```

### 4.2 GitLab CI/CD

```yaml
# .gitlab-ci.yml
stages:
  - test
  - deploy

test:
  stage: test
  script:
    - pip install wia-data-pipeline pytest
    - pytest tests/
    - wia validate pipeline.yaml

deploy:
  stage: deploy
  script:
    - wia deploy --pipeline-config pipeline.yaml --environment production
  only:
    - main
```

## 5. Data Catalog Integration

### 5.1 Apache Atlas Integration

```python
from wia_data_pipeline import AtlasIntegration

atlas = AtlasIntegration(
    endpoint='http://atlas:21000',
    username='admin',
    password='admin'
)

# Register pipeline as Atlas entity
atlas.register_pipeline(
    pipeline_id='pip_abc123',
    name='daily_sales_pipeline',
    inputs=['orders_raw'],
    outputs=['daily_sales'],
    owner='data-team@company.com'
)

# Track lineage
atlas.track_lineage(
    source='orders_raw',
    destination='daily_sales',
    pipeline='pip_abc123',
    transformation='aggregate_daily'
)
```

### 5.2 DataHub Integration

```python
from wia_data_pipeline import DataHubIntegration

datahub = DataHubIntegration(
    gms_endpoint='http://datahub:8080'
)

# Emit metadata
datahub.emit_pipeline_metadata(
    pipeline_id='pip_abc123',
    metadata={
        'name': 'daily_sales_pipeline',
        'description': 'Aggregates daily sales from orders',
        'owner': 'data-team',
        'tags': ['sales', 'daily', 'critical']
    }
)
```

## 6. Secrets Management

### 6.1 HashiCorp Vault

```python
import hvac
from wia_data_pipeline import VaultIntegration

vault = VaultIntegration(
    url='https://vault:8200',
    token='s.xxx'
)

# Retrieve secrets
db_credentials = vault.get_secret('database/postgresql/prod')

# Use in pipeline
pipeline.configure(
    source={
        'type': 'postgresql',
        'host': db_credentials['host'],
        'username': db_credentials['username'],
        'password': db_credentials['password']
    }
)
```

### 6.2 AWS Secrets Manager

```python
import boto3
from wia_data_pipeline import AWSSecretsIntegration

secrets = AWSSecretsIntegration(
    region_name='us-east-1'
)

# Retrieve secrets
db_credentials = secrets.get_secret('prod/database/credentials')

# Auto-rotate secrets
secrets.enable_rotation(
    secret_id='prod/database/credentials',
    rotation_lambda='arn:aws:lambda:...',
    rotation_days=90
)
```

## 7. Testing Integration

### 7.1 Unit Testing

```python
import pytest
from wia_data_pipeline import Pipeline

def test_pipeline_extraction():
    pipeline = Pipeline.from_config('pipeline.yaml')
    result = pipeline.extract(test_data=True)
    assert result.record_count > 0
    assert result.schema.is_valid()

def test_pipeline_transformation():
    pipeline = Pipeline.from_config('pipeline.yaml')
    data = [{'user_id': 1, 'total': 100}]
    result = pipeline.transform(data)
    assert result[0]['total'] == 100

def test_pipeline_idempotency():
    pipeline = Pipeline.from_config('pipeline.yaml')
    result1 = pipeline.run(run_id='test_001')
    result2 = pipeline.run(run_id='test_001')
    assert result1 == result2
```

### 7.2 Integration Testing

```python
@pytest.fixture(scope="session")
def test_database():
    """Spin up test database"""
    db = TestDatabase()
    db.start()
    yield db
    db.stop()

def test_full_pipeline(test_database):
    # Insert test data
    test_database.insert('orders', test_orders)

    # Run pipeline
    pipeline = Pipeline.from_config('pipeline.yaml')
    result = pipeline.run()

    # Verify results
    assert result.status == 'success'
    assert result.records_processed == len(test_orders)

    # Check destination
    dest_data = test_database.query('daily_sales')
    assert len(dest_data) > 0
```

## 8. Best Practices

### 8.1 Configuration Management

```yaml
# config/production.yaml
environment: production
pipelines:
  daily_sales:
    schedule: "0 2 * * *"
    retry_policy:
      max_retries: 3
      backoff_multiplier: 2
    quality_checks:
      enable: true
      fail_on_error: true
    monitoring:
      enable: true
      alert_channel: "#data-alerts"
```

### 8.2 Error Handling

```python
from wia_data_pipeline import Pipeline, PipelineError

try:
    pipeline = Pipeline.from_config('pipeline.yaml')
    result = pipeline.run()
except PipelineError as e:
    logger.error(f"Pipeline failed: {e}")
    send_alert(f"Pipeline failed: {e}")
    raise
finally:
    pipeline.cleanup()
```

### 8.3 Performance Optimization

```python
# Partitioning
pipeline.configure(
    partitioning={
        'column': 'date',
        'strategy': 'daily'
    }
)

# Parallelism
pipeline.configure(
    parallelism={
        'extract': 4,
        'transform': 8,
        'load': 2
    }
)

# Caching
pipeline.configure(
    caching={
        'enable': true,
        'ttl_hours': 24
    }
)
```

---

**Status:** ✅ Complete
**Version:** 1.0.0
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) · Benefit All Humanity
