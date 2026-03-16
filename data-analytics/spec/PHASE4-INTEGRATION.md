# WIA-DATA-012: Data Analytics Standard
## PHASE 4 - INTEGRATION SPECIFICATION

### Version: 1.0
### Status: Active
### Last Updated: 2025-12-26

---

## 1. Overview

This document defines integration patterns and best practices for WIA-DATA-012 Data Analytics Standard. These specifications enable seamless integration with existing systems, tools, and platforms.

**Philosophy**: 弘益人間 (홍익인간) - Benefit All Humanity through accessible, interoperable analytics integration.

## 2. Integration Principles

### 2.1 Core Principles
- **Loose Coupling**: Minimize dependencies
- **Standards-Based**: Use industry standards
- **Backward Compatible**: Support legacy systems
- **Plug-and-Play**: Easy to integrate and remove
- **Observable**: Comprehensive monitoring and logging
- **Resilient**: Fault tolerance and graceful degradation

## 3. Data Source Integration

### 3.1 Database Connectors

**PostgreSQL Integration**:
```python
from wia_analytics import DatabaseConnector

connector = DatabaseConnector(
    type="postgresql",
    host="postgres.example.com",
    port=5432,
    database="analytics_db",
    username="analytics_user",
    password="secure_password",
    ssl_mode="require"
)

# Extract data
query = """
SELECT 
    transaction_id,
    customer_id,
    amount,
    timestamp,
    category
FROM transactions
WHERE timestamp >= CURRENT_DATE - INTERVAL '30 days'
"""

dataset = connector.query_to_dataset(
    query=query,
    dataset_id="recent_transactions"
)
```

**MongoDB Integration**:
```javascript
const { MongoDBConnector } = require('@wia/analytics');

const connector = new MongoDBConnector({
  uri: 'mongodb://mongo.example.com:27017',
  database: 'analytics',
  collection: 'events'
});

// Extract data
const dataset = await connector.aggregateToDataset([
  { $match: { timestamp: { $gte: new Date('2025-01-01') } } },
  { $group: { _id: '$category', total: { $sum: '$amount' } } }
], {
  datasetId: 'category_totals'
});
```

### 3.2 Cloud Storage Integration

**AWS S3**:
```python
from wia_analytics import S3Connector

s3 = S3Connector(
    bucket="analytics-data",
    region="us-west-2",
    access_key_id="YOUR_ACCESS_KEY",
    secret_access_key="YOUR_SECRET_KEY"
)

# Read Parquet files
dataset = s3.read_parquet(
    path="s3://analytics-data/sales/2025/*/",
    dataset_id="sales_2025"
)

# Write results
s3.write_parquet(
    dataset=results,
    path="s3://analytics-data/results/monthly_summary.parquet",
    compression="snappy"
)
```

**Google Cloud Storage**:
```python
from wia_analytics import GCSConnector

gcs = GCSConnector(
    project_id="your-project",
    bucket="analytics-bucket",
    credentials_path="/path/to/credentials.json"
)

dataset = gcs.read_dataset(
    path="gs://analytics-bucket/data/*.json",
    format="json",
    dataset_id="events_data"
)
```

### 3.3 Streaming Source Integration

**Apache Kafka**:
```yaml
# kafka-source.yml
apiVersion: wia.org/v1
kind: StreamSource
metadata:
  name: sales-events
spec:
  type: kafka
  config:
    bootstrap_servers:
      - kafka1:9092
      - kafka2:9092
    topic: sales.events
    consumer_group: analytics-consumers
    auto_offset_reset: earliest
  schema:
    format: avro
    schema_registry: http://schema-registry:8081
  output:
    dataset_id: sales_stream
    buffer_size: 1000
    flush_interval: 5s
```

**AWS Kinesis**:
```json
{
  "source_type": "kinesis",
  "stream_name": "analytics-events",
  "region": "us-east-1",
  "shard_iterator_type": "LATEST",
  "checkpoint": {
    "type": "dynamodb",
    "table_name": "analytics-checkpoints"
  },
  "output": {
    "dataset_id": "kinesis_events"
  }
}
```

## 4. BI Tool Integration

### 4.1 Tableau Integration

**WIA Tableau Connector**:
```xml
<?xml version="1.0" encoding="utf-8"?>
<connector class="wia_analytics" superclass="webdata" version="1.0">
  <connection-dialog file="connection-dialog.html" />
  <connection-metadata>
    <class name="wia_analytics" />
    <api-version>1.0</api-version>
  </connection-metadata>
  <connection-properties>
    <property name="server" label="WIA API Endpoint" default="https://api.wia.org/v1" />
    <property name="api_key" label="API Key" secure="true" />
    <property name="dataset_id" label="Dataset ID" />
  </connection-properties>
</connector>
```

**Custom SQL Support**:
```sql
-- Tableau Custom SQL with WIA-DATA-012
SELECT
    <Parameters.DateField> as date,
    category,
    SUM(amount) as total_revenue,
    COUNT(*) as transaction_count
FROM [WIA:sales_2025]
WHERE <Parameters.DateField> BETWEEN <Parameters.StartDate> AND <Parameters.EndDate>
GROUP BY <Parameters.DateField>, category
```

### 4.2 Power BI Integration

**M Query for Power BI**:
```m
let
    Source = Json.Document(Web.Contents(
        "https://api.wia.org/v1/analytics/descriptive",
        [
            Headers=[
                #"Authorization"="Bearer " & ApiKey,
                #"Content-Type"="application/json"
            ],
            Content=Text.ToBinary("{
                ""dataset_id"": ""sales_2025"",
                ""metrics"": [""mean"", ""sum"", ""count""],
                ""group_by"": [""category""]
            }")
        ]
    )),
    Results = Source[results][grouped_results],
    Table = Table.FromRecords(Results)
in
    Table
```

**Power Query Function**:
```m
(dataset_id as text, start_date as date, end_date as date) =>
let
    ApiEndpoint = "https://api.wia.org/v1/analytics/descriptive",
    RequestBody = "{
        ""dataset_id"": """ & dataset_id & """,
        ""filters"": {
            ""date_range"": {
                ""start"": """ & Date.ToText(start_date, "yyyy-MM-dd") & """,
                ""end"": """ & Date.ToText(end_date, "yyyy-MM-dd") & """
            }
        }
    }",
    Response = Json.Document(Web.Contents(ApiEndpoint, [
        Headers=[#"Authorization"="Bearer " & ApiKey],
        Content=Text.ToBinary(RequestBody)
    ]))
in
    Response
```

## 5. ML Platform Integration

### 5.1 MLflow Integration

**Register WIA Model**:
```python
import mlflow
from wia_analytics import WIAModelWrapper

# Train your model
model = train_prophet_model(data)

# Wrap for WIA-DATA-012 compliance
wia_model = WIAModelWrapper(
    model=model,
    model_type="time_series_forecast",
    wia_version="1.0"
)

# Register with MLflow
with mlflow.start_run():
    mlflow.log_params(model.params)
    mlflow.log_metrics(model.metrics)
    mlflow.wia.log_model(
        wia_model,
        "model",
        registered_model_name="sales_forecast"
    )
```

### 5.2 TensorFlow Integration

**WIA TensorFlow SavedModel**:
```python
import tensorflow as tf
from wia_analytics.tensorflow import WIASavedModelBuilder

# Create TensorFlow model
model = tf.keras.Sequential([...])
model.compile(...)
model.fit(X_train, y_train)

# Export as WIA-compliant SavedModel
builder = WIASavedModelBuilder(
    model=model,
    export_dir="/models/my_model",
    wia_metadata={
        "model_type": "neural_network",
        "framework": "tensorflow",
        "version": "1.0"
    }
)

builder.save()
```

## 6. Cloud Platform Integration

### 6.1 AWS Integration

**Lambda Function**:
```python
import json
from wia_analytics import AnalyticsClient

def lambda_handler(event, context):
    client = AnalyticsClient(
        api_key=os.environ['WIA_API_KEY']
    )
    
    # Extract S3 event details
    bucket = event['Records'][0]['s3']['bucket']['name']
    key = event['Records'][0]['s3']['object']['key']
    
    # Run analytics
    result = client.descriptive_analytics(
        dataset_id=f"s3://{bucket}/{key}",
        metrics=["mean", "count", "sum"]
    )
    
    return {
        'statusCode': 200,
        'body': json.dumps(result)
    }
```

**CloudFormation Template**:
```yaml
AWSTemplateFormatVersion: '2010-09-09'
Resources:
  WIAAnalyticsFunction:
    Type: AWS::Lambda::Function
    Properties:
      FunctionName: wia-analytics-processor
      Runtime: python3.9
      Handler: index.lambda_handler
      Environment:
        Variables:
          WIA_API_KEY: !Ref WIAAPIKey
      Code:
        ZipFile: |
          # Lambda code here
          
  WIAAPIKey:
    Type: AWS::SecretsManager::Secret
    Properties:
      Name: wia-api-key
      SecretString: !Ref APIKeyParameter
```

### 6.2 Google Cloud Integration

**Cloud Function**:
```javascript
const { AnalyticsClient } = require('@wia/analytics');

exports.processAnalytics = async (event, context) => {
  const client = new AnalyticsClient({
    apiKey: process.env.WIA_API_KEY
  });
  
  // Parse Pub/Sub message
  const message = Buffer.from(event.data, 'base64').toString();
  const data = JSON.parse(message);
  
  // Run analytics
  const result = await client.descriptiveAnalytics({
    dataset_id: data.dataset_id,
    metrics: ['mean', 'count', 'sum']
  });
  
  console.log('Analytics result:', result);
};
```

### 6.3 Azure Integration

**Azure Function**:
```csharp
using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using WIA.Analytics;

public class AnalyticsFunction
{
    private readonly ILogger _logger;
    private readonly AnalyticsClient _client;
    
    public AnalyticsFunction(ILoggerFactory loggerFactory)
    {
        _logger = loggerFactory.CreateLogger<AnalyticsFunction>();
        _client = new AnalyticsClient(Environment.GetEnvironmentVariable("WIA_API_KEY"));
    }
    
    [Function("ProcessAnalytics")]
    public async Task Run([EventHubTrigger("analytics-events")] string[] events)
    {
        foreach (var eventData in events)
        {
            var result = await _client.DescriptiveAnalyticsAsync(new DescriptiveRequest
            {
                DatasetId = "sales_2025",
                Metrics = new[] { "mean", "count", "sum" }
            });
            
            _logger.LogInformation($"Analytics completed: {result.RequestId}");
        }
    }
}
```

## 7. Workflow Orchestration

### 7.1 Apache Airflow

**WIA Analytics DAG**:
```python
from airflow import DAG
from airflow.providers.wia.operators.analytics import WIAAnalyticsOperator
from datetime import datetime, timedelta

default_args = {
    'owner': 'analytics-team',
    'depends_on_past': False,
    'start_date': datetime(2025, 1, 1),
    'retries': 1,
    'retry_delay': timedelta(minutes=5),
}

dag = DAG(
    'daily_sales_analytics',
    default_args=default_args,
    schedule_interval='0 2 * * *',  # Daily at 2 AM
    catchup=False
)

# Extract data
extract = WIAAnalyticsOperator(
    task_id='extract_sales_data',
    operation='extract',
    dataset_id='sales_{{ ds }}',
    source_config={
        'type': 'postgresql',
        'query': 'SELECT * FROM sales WHERE date = {{ ds }}'
    },
    dag=dag
)

# Descriptive analytics
descriptive = WIAAnalyticsOperator(
    task_id='descriptive_analysis',
    operation='descriptive',
    dataset_id='sales_{{ ds }}',
    metrics=['mean', 'median', 'std', 'count'],
    group_by=['category', 'region'],
    dag=dag
)

# Predictive analytics
predictive = WIAAnalyticsOperator(
    task_id='sales_forecast',
    operation='predictive',
    model_type='prophet',
    dataset_id='sales_history',
    horizon=7,
    dag=dag
)

extract >> descriptive >> predictive
```

### 7.2 Prefect

**Prefect Flow**:
```python
from prefect import flow, task
from wia_analytics import AnalyticsClient

@task
def extract_data(dataset_id):
    client = AnalyticsClient()
    return client.extract_dataset(dataset_id)

@task
def analyze_data(data):
    client = AnalyticsClient()
    return client.descriptive_analytics(
        data=data,
        metrics=['mean', 'count', 'sum']
    )

@task
def forecast_data(data):
    client = AnalyticsClient()
    return client.predictive_analytics(
        data=data,
        model_type='prophet',
        horizon=7
    )

@flow(name="daily-analytics")
def daily_analytics_flow(dataset_id: str):
    data = extract_data(dataset_id)
    analysis = analyze_data(data)
    forecast = forecast_data(data)
    return {
        'analysis': analysis,
        'forecast': forecast
    }

if __name__ == "__main__":
    daily_analytics_flow("sales_2025")
```

## 8. Monitoring Integration

### 8.1 Prometheus

**Metrics Exporter**:
```python
from prometheus_client import Counter, Histogram, Gauge, start_http_server
from wia_analytics import AnalyticsClient

# Metrics
analytics_requests = Counter(
    'wia_analytics_requests_total',
    'Total analytics requests',
    ['operation', 'status']
)

analytics_duration = Histogram(
    'wia_analytics_duration_seconds',
    'Analytics operation duration',
    ['operation']
)

active_streams = Gauge(
    'wia_active_streams',
    'Number of active analytics streams'
)

# Start metrics server
start_http_server(8000)

# Use in analytics operations
client = AnalyticsClient()

@analytics_duration.time()
def run_analytics(dataset_id):
    try:
        result = client.descriptive_analytics(dataset_id=dataset_id)
        analytics_requests.labels('descriptive', 'success').inc()
        return result
    except Exception as e:
        analytics_requests.labels('descriptive', 'error').inc()
        raise
```

### 8.2 Grafana Dashboard

**Dashboard JSON**:
```json
{
  "dashboard": {
    "title": "WIA Analytics Monitoring",
    "panels": [
      {
        "title": "Analytics Request Rate",
        "targets": [
          {
            "expr": "rate(wia_analytics_requests_total[5m])",
            "legendFormat": "{{operation}} - {{status}}"
          }
        ],
        "type": "graph"
      },
      {
        "title": "Average Response Time",
        "targets": [
          {
            "expr": "histogram_quantile(0.95, wia_analytics_duration_seconds_bucket)",
            "legendFormat": "p95"
          }
        ],
        "type": "graph"
      },
      {
        "title": "Active Streams",
        "targets": [
          {
            "expr": "wia_active_streams",
            "legendFormat": "Active Streams"
          }
        ],
        "type": "stat"
      }
    ]
  }
}
```

## 9. Data Catalog Integration

### 9.1 Apache Atlas

**Lineage Registration**:
```python
from wia_analytics.catalog import AtlasIntegration

atlas = AtlasIntegration(
    endpoint="http://atlas.example.com:21000",
    username="admin",
    password="admin"
)

# Register dataset
atlas.register_dataset(
    dataset_id="sales_2025",
    name="Sales Data 2025",
    description="Daily sales transactions for 2025",
    schema={
        "fields": [
            {"name": "transaction_id", "type": "string"},
            {"name": "amount", "type": "double"},
            {"name": "timestamp", "type": "timestamp"}
        ]
    },
    classification=["PII", "Financial"]
)

# Register lineage
atlas.register_lineage(
    source_dataset="sales_raw",
    target_dataset="sales_2025",
    process="analytics_transformation",
    transformation_sql="SELECT * FROM sales_raw WHERE year = 2025"
)
```

## 10. Testing Integration

### 10.1 Unit Testing

**pytest Example**:
```python
import pytest
from wia_analytics import AnalyticsClient

@pytest.fixture
def analytics_client():
    return AnalyticsClient(api_key="test_key")

def test_descriptive_analytics(analytics_client, mock_dataset):
    result = analytics_client.descriptive_analytics(
        dataset_id="test_dataset",
        metrics=["mean", "count"]
    )
    
    assert result.status == "success"
    assert "mean" in result.statistics
    assert result.statistics["count"] > 0

def test_predictive_analytics(analytics_client, mock_time_series):
    result = analytics_client.predictive_analytics(
        dataset_id="test_timeseries",
        model_type="prophet",
        horizon=7
    )
    
    assert result.status == "success"
    assert len(result.predictions) == 7
    assert all(p.confidence_interval for p in result.predictions)
```

## 11. Deployment Patterns

### 11.1 Docker Deployment

**Dockerfile**:
```dockerfile
FROM python:3.9-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

ENV WIA_API_KEY=""
ENV WIA_ENDPOINT="https://api.wia.org/v1"

CMD ["python", "analytics_service.py"]
```

**docker-compose.yml**:
```yaml
version: '3.8'

services:
  analytics:
    build: .
    environment:
      - WIA_API_KEY=${WIA_API_KEY}
      - KAFKA_BROKERS=kafka:9092
    depends_on:
      - kafka
      - postgres
    ports:
      - "8080:8080"
      
  kafka:
    image: confluentinc/cp-kafka:latest
    ports:
      - "9092:9092"
      
  postgres:
    image: postgres:14
    environment:
      - POSTGRES_DB=analytics
      - POSTGRES_USER=analytics
      - POSTGRES_PASSWORD=password
```

### 11.2 Kubernetes Deployment

**deployment.yaml**:
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-analytics
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-analytics
  template:
    metadata:
      labels:
        app: wia-analytics
    spec:
      containers:
      - name: analytics
        image: wia/analytics:1.0
        env:
        - name: WIA_API_KEY
          valueFrom:
            secretKeyRef:
              name: wia-secrets
              key: api-key
        resources:
          requests:
            memory: "512Mi"
            cpu: "500m"
          limits:
            memory: "1Gi"
            cpu: "1000m"
        ports:
        - containerPort: 8080
---
apiVersion: v1
kind: Service
metadata:
  name: wia-analytics-service
spec:
  selector:
    app: wia-analytics
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8080
  type: LoadBalancer
```

## 12. Best Practices

### 12.1 Error Handling
```python
from wia_analytics import AnalyticsClient, WIAException
import time

def robust_analytics_call(dataset_id, max_retries=3):
    client = AnalyticsClient()
    
    for attempt in range(max_retries):
        try:
            result = client.descriptive_analytics(dataset_id=dataset_id)
            return result
        except WIAException as e:
            if e.code == "RATE_LIMIT_EXCEEDED":
                wait_time = 2 ** attempt  # Exponential backoff
                time.sleep(wait_time)
                continue
            elif e.code == "INVALID_DATASET":
                # Don't retry for invalid dataset
                raise
            else:
                if attempt == max_retries - 1:
                    raise
```

### 12.2 Performance Optimization
- Use connection pooling for database connectors
- Implement caching for frequently accessed datasets
- Batch requests when possible
- Use async/await for concurrent operations
- Monitor and optimize query performance

## 13. Conclusion

This PHASE 4 specification provides comprehensive integration patterns for WIA-DATA-012, enabling:
- Seamless connection to data sources
- Integration with popular BI and ML tools
- Cloud platform deployment
- Workflow orchestration
- Monitoring and observability
- Production-ready deployment patterns

**Implementation**: Organizations can now implement complete analytics solutions using WIA-DATA-012 standards.

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (홍익인간) · Benefit All Humanity
