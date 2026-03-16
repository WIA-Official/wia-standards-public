# WIA-DATA-010: PHASE 4 - Integration Specification

**Version:** 1.0.0  
**Status:** Complete  
**Last Updated:** 2025-01-15

---

## Overview

This document provides end-to-end integration patterns, implementation guidelines, and best practices for WIA-DATA-010 Data Integration Standard.

## 1. Integration Patterns

### 1.1 ETL (Extract, Transform, Load)

**Pattern Overview:**
```
[Source DB] → [Extract] → [Staging Area] → [Transform] → [Load] → [Data Warehouse]
```

**Implementation Steps:**
1. **Extract:** Connect to source, read data (full or incremental)
2. **Stage:** Land data in temporary staging area
3. **Transform:** Apply business rules, cleanse, aggregate
4. **Load:** Bulk insert into target data warehouse
5. **Validate:** Verify data quality and completeness

**Example Code (Python + Airflow):**
```python
from airflow import DAG
from airflow.operators.python import PythonOperator
import pandas as pd
import psycopg2

def extract():
    conn = psycopg2.connect(...)
    df = pd.read_sql("SELECT * FROM customers WHERE updated_at > '2025-01-15'", conn)
    df.to_parquet('/staging/customers.parquet')

def transform():
    df = pd.read_parquet('/staging/customers.parquet')
    # Business logic
    df['full_name'] = df['first_name'] + ' ' + df['last_name']
    df['email'] = df['email'].str.lower()
    df.to_parquet('/staging/customers_transformed.parquet')

def load():
    df = pd.read_parquet('/staging/customers_transformed.parquet')
    engine = create_engine('snowflake://...')
    df.to_sql('customers', engine, if_exists='append')

dag = DAG('customer_etl', schedule_interval='@daily')
extract_task = PythonOperator(task_id='extract', python_callable=extract, dag=dag)
transform_task = PythonOperator(task_id='transform', python_callable=transform, dag=dag)
load_task = PythonOperator(task_id='load', python_callable=load, dag=dag)

extract_task >> transform_task >> load_task
```

### 1.2 ELT (Extract, Load, Transform)

**Pattern Overview:**
```
[Source DB] → [Extract] → [Load] → [Data Warehouse] → [Transform (SQL/dbt)]
```

**Implementation Steps:**
1. **Extract:** Use managed ELT tool (Fivetran, Airbyte)
2. **Load:** Raw data lands in warehouse (Bronze/Raw layer)
3. **Transform:** dbt models transform data (Silver/Gold layers)
4. **Orchestrate:** Airflow/dbt Cloud schedules transformations

**Example (dbt model):**
```sql
-- models/staging/stg_customers.sql
{{ config(materialized='view') }}

WITH source AS (
    SELECT * FROM {{ source('raw', 'customers') }}
),

cleaned AS (
    SELECT
        id AS customer_id,
        TRIM(LOWER(email)) AS email,
        CONCAT(TRIM(first_name), ' ', TRIM(last_name)) AS full_name,
        created_at,
        updated_at
    FROM source
    WHERE email IS NOT NULL
)

SELECT * FROM cleaned
```

### 1.3 Real-Time Streaming

**Pattern Overview:**
```
[App Events] → [Kafka] → [Stream Processor] → [Data Warehouse/Lake]
                                ↓
                          [Real-Time Analytics]
```

**Implementation (Kafka + Flink):**
```java
StreamExecutionEnvironment env = StreamExecutionEnvironment.getExecutionEnvironment();

// Kafka source
FlinkKafkaConsumer<String> consumer = new FlinkKafkaConsumer<>(
    "user-events",
    new SimpleStringSchema(),
    properties
);

DataStream<Event> events = env
    .addSource(consumer)
    .map(json -> parseEvent(json))
    .keyBy(Event::getUserId)
    .window(TumblingEventTimeWindows.of(Time.minutes(5)))
    .aggregate(new EventAggregator());

// Sink to Snowflake
events.addSink(new SnowflakeSink());

env.execute("Real-time Event Processing");
```

### 1.4 API Integration

**Pattern Overview:**
```
[REST API] → [API Client] → [Data Warehouse]
```

**Implementation (Python):**
```python
import requests
import psycopg2
from typing import List, Dict

class SalesforceToPostgresSync:
    def __init__(self, sf_config, pg_config):
        self.sf_base_url = sf_config['instance_url']
        self.sf_token = sf_config['access_token']
        self.pg_conn = psycopg2.connect(**pg_config)

    def fetch_accounts(self) -> List[Dict]:
        headers = {'Authorization': f'Bearer {self.sf_token}'}
        accounts = []
        url = f"{self.sf_base_url}/services/data/v55.0/query"
        
        query = "SELECT Id, Name, Industry FROM Account WHERE LastModifiedDate >= YESTERDAY"
        params = {'q': query}
        
        while url:
            response = requests.get(url, headers=headers, params=params)
            response.raise_for_status()
            data = response.json()
            accounts.extend(data['records'])
            url = data.get('nextRecordsUrl')
            params = None  # Pagination URL includes query
        
        return accounts

    def sync(self):
        accounts = self.fetch_accounts()
        cursor = self.pg_conn.cursor()
        
        for account in accounts:
            cursor.execute("""
                INSERT INTO salesforce_accounts (sf_id, name, industry, synced_at)
                VALUES (%s, %s, %s, NOW())
                ON CONFLICT (sf_id) DO UPDATE SET
                    name = EXCLUDED.name,
                    industry = EXCLUDED.industry,
                    synced_at = EXCLUDED.synced_at
            """, (account['Id'], account['Name'], account.get('Industry')))
        
        self.pg_conn.commit()
        print(f"Synced {len(accounts)} accounts")
```

### 1.5 Change Data Capture (CDC)

**Pattern Overview:**
```
[Source DB] → [CDC Tool] → [Kafka] → [Consumer] → [Target DB]
```

**Implementation (Debezium):**
```json
{
  "name": "postgres-cdc-connector",
  "config": {
    "connector.class": "io.debezium.connector.postgresql.PostgresConnector",
    "database.hostname": "postgres.example.com",
    "database.port": "5432",
    "database.user": "debezium",
    "database.password": "${env:DB_PASSWORD}",
    "database.dbname": "production",
    "database.server.name": "prod_server",
    "table.include.list": "public.customers,public.orders",
    "plugin.name": "pgoutput",
    "publication.autocreate.mode": "filtered",
    "slot.name": "debezium_customers_orders",
    "transforms": "route",
    "transforms.route.type": "org.apache.kafka.connect.transforms.RegexRouter",
    "transforms.route.regex": "([^.]+)\\.([^.]+)\\.([^.]+)",
    "transforms.route.replacement": "$3"
  }
}
```

## 2. Architecture Patterns

### 2.1 Lambda Architecture

```
                    ┌─────────────┐
                    │   Sources   │
                    └──────┬──────┘
                           │
           ┌───────────────┴───────────────┐
           │                               │
      ┌────▼─────┐                    ┌───▼────┐
      │  Batch   │                    │ Stream │
      │  Layer   │                    │ Layer  │
      │ (Spark)  │                    │(Flink) │
      └────┬─────┘                    └───┬────┘
           │                              │
           │      ┌──────────────┐        │
           └─────►│   Serving    │◄───────┘
                  │    Layer     │
                  │  (Druid)     │
                  └──────────────┘
```

**Use Case:** Systems requiring both historical accuracy (batch) and real-time updates (streaming)

### 2.2 Kappa Architecture

```
     ┌─────────┐
     │ Sources │
     └────┬────┘
          │
     ┌────▼────────┐
     │   Kafka     │
     │  (Stream)   │
     └────┬────────┘
          │
    ┌─────▼──────┐
    │  Processor │
    │  (Flink)   │
    └─────┬──────┘
          │
     ┌────▼────┐
     │ Storage │
     └─────────┘
```

**Use Case:** Stream-first organizations, simpler than Lambda

### 2.3 Data Mesh

```
┌──────────────────┐   ┌──────────────────┐   ┌──────────────────┐
│  Sales Domain    │   │ Marketing Domain │   │ Finance Domain   │
│                  │   │                  │   │                  │
│  ┌────────────┐  │   │  ┌────────────┐  │   │  ┌────────────┐  │
│  │Sales Data  │  │   │  │ Campaign   │  │   │  │Transaction │  │
│  │  Product   │  │   │  │    Data    │  │   │  │    Data    │  │
│  └────────────┘  │   │  │  Product   │  │   │  │  Product   │  │
└──────────────────┘   │  └────────────┘  │   │  └────────────┘  │
                       └──────────────────┘   └──────────────────┘
                                │
                    ┌───────────┴───────────┐
                    │  Self-Serve Platform  │
                    │  (dbt, Airflow, etc)  │
                    └───────────────────────┘
```

**Use Case:** Large organizations with multiple domains needing autonomy

### 2.4 Medallion Architecture (Bronze-Silver-Gold)

```
┌─────────┐
│ Sources │
└────┬────┘
     │
┌────▼──────────────┐
│  Bronze (Raw)     │  ← Raw data as-is
│  - No transforms  │
│  - Full history   │
└────┬──────────────┘
     │
┌────▼──────────────┐
│  Silver (Clean)   │  ← Cleaned & validated
│  - Deduplicated   │
│  - Validated      │
└────┬──────────────┘
     │
┌────▼──────────────┐
│  Gold (Curated)   │  ← Business-ready
│  - Aggregated     │
│  - Joined         │
│  - Business logic │
└───────────────────┘
```

## 3. Implementation Checklist

### 3.1 Planning Phase
- [ ] Define business requirements and KPIs
- [ ] Inventory data sources and destinations
- [ ] Assess data volumes and velocity
- [ ] Determine latency requirements
- [ ] Identify compliance and security requirements
- [ ] Choose architecture pattern (ETL, ELT, streaming)
- [ ] Select technology stack

### 3.2 Design Phase
- [ ] Design data models (source → staging → target)
- [ ] Define transformation logic
- [ ] Plan partitioning strategy
- [ ] Design error handling and retry logic
- [ ] Create data quality rules
- [ ] Document data lineage
- [ ] Design monitoring and alerting

### 3.3 Implementation Phase
- [ ] Set up development environment
- [ ] Configure source and destination connectors
- [ ] Implement extraction logic
- [ ] Build transformation pipelines
- [ ] Implement loading mechanism
- [ ] Add data quality checks
- [ ] Implement logging and monitoring
- [ ] Write unit and integration tests

### 3.4 Testing Phase
- [ ] Test with sample data
- [ ] Validate data quality
- [ ] Performance test with realistic volumes
- [ ] Test error scenarios and retries
- [ ] Verify idempotency
- [ ] Test monitoring and alerts
- [ ] User acceptance testing (UAT)

### 3.5 Deployment Phase
- [ ] Deploy to staging environment
- [ ] Run parallel processing (old + new systems)
- [ ] Validate results match
- [ ] Deploy to production
- [ ] Monitor closely for first week
- [ ] Document runbooks and troubleshooting guides

### 3.6 Operations Phase
- [ ] Daily monitoring of pipeline health
- [ ] Weekly data quality reports
- [ ] Monthly performance reviews
- [ ] Quarterly cost optimization reviews
- [ ] Ongoing schema evolution management
- [ ] Regular security audits

## 4. Best Practices

### 4.1 Data Quality
- **Validation at Source:** Check data quality before processing
- **Schema Validation:** Validate against expected schemas
- **Anomaly Detection:** Alert on unexpected distributions
- **Reconciliation:** Match source and destination counts
- **Data Profiling:** Regularly profile data for quality metrics

### 4.2 Performance
- **Incremental Loading:** Load only new/changed data
- **Parallel Processing:** Partition data for parallel processing
- **Batch Size Optimization:** Tune batch sizes for performance
- **Compression:** Use appropriate compression (Snappy, Gzip)
- **Indexing:** Create indexes on frequently queried columns

### 4.3 Reliability
- **Idempotency:** Ensure pipelines can be safely re-run
- **Error Handling:** Graceful failures with retries
- **Dead Letter Queues:** Capture failed messages for investigation
- **Circuit Breakers:** Prevent cascading failures
- **Monitoring:** Track success rates, latency, throughput

### 4.4 Security
- **Encryption:** Encrypt data at rest and in transit
- **Access Control:** Principle of least privilege
- **Secrets Management:** Use vaults (AWS Secrets Manager, Vault)
- **Audit Logging:** Log all data access and changes
- **PII/PHI Handling:** Mask or encrypt sensitive fields

### 4.5 Cost Optimization
- **Right-Sizing:** Match compute resources to workload
- **Auto-Scaling:** Scale down during off-peak hours
- **Spot Instances:** Use for non-critical batch jobs
- **Storage Tiering:** Archive old data to cheaper storage
- **Query Optimization:** Reduce unnecessary data scans

## 5. Common Integration Scenarios

### 5.1 SaaS to Data Warehouse
**Tools:** Fivetran, Airbyte
**Pattern:** ELT
**Frequency:** Real-time to hourly
**Example:** Salesforce → Snowflake

### 5.2 Database Replication
**Tools:** Debezium, AWS DMS
**Pattern:** CDC
**Frequency:** Real-time
**Example:** PostgreSQL → BigQuery

### 5.3 Event Streaming
**Tools:** Kafka, Kinesis
**Pattern:** Streaming
**Frequency:** Real-time
**Example:** Application events → Data Lake

### 5.4 File-Based Integration
**Tools:** Apache NiFi, AWS Glue
**Pattern:** ETL
**Frequency:** Batch (daily, hourly)
**Example:** CSV files → Redshift

### 5.5 API Integration
**Tools:** Custom Python/Node.js
**Pattern:** API polling or webhooks
**Frequency:** Real-time to hourly
**Example:** REST API → Data Warehouse

## 6. Troubleshooting Guide

### 6.1 Pipeline Failures

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Connection timeout | Network issues, firewall | Check connectivity, whitelist IPs |
| Authentication failed | Invalid credentials | Verify API keys, rotate if needed |
| Schema mismatch | Source schema changed | Update schema, implement schema registry |
| Out of memory | Dataset too large | Increase memory, use batching |
| Slow performance | Unoptimized queries | Add indexes, optimize SQL |
| Data quality issues | Source data problems | Add validation, alert on anomalies |

### 6.2 Debugging Steps
1. Check pipeline logs for error messages
2. Verify source and destination connectivity
3. Validate credentials and permissions
4. Check data volumes and sizes
5. Review recent schema changes
6. Examine monitoring metrics (latency, throughput)
7. Test with smaller dataset
8. Enable verbose logging for detailed diagnostics

## 7. Success Metrics

### 7.1 Technical Metrics
- **Uptime:** Pipeline availability percentage
- **Latency:** Time from source change to destination availability
- **Throughput:** Records processed per second
- **Error Rate:** Percentage of failed records
- **Data Freshness:** Time since last update

### 7.2 Business Metrics
- **Time to Insight:** How quickly data becomes available for analysis
- **Data Coverage:** Percentage of required data sources integrated
- **Cost per GB:** Total cost divided by data volume processed
- **User Satisfaction:** Survey scores from data consumers

---

**Previous Phase:** [PHASE 3 - Protocol](PHASE3-PROTOCOL.md)

**Status:** ✅ Complete  
**Compliance:** WIA-DATA-010 v1.0.0  
**Implementation Ready:** Yes

---

## Appendix: Quick Start Templates

### Template 1: Simple ETL Pipeline (Python + Airflow)
See Section 1.1 for full code

### Template 2: ELT with dbt
See Section 1.2 for full code

### Template 3: Real-Time Streaming (Kafka + Flink)
See Section 1.3 for full code

### Template 4: API Integration
See Section 1.4 for full code

### Template 5: CDC Setup (Debezium)
See Section 1.5 for full code

---

**WIA-DATA-010 Complete!**
All phases implemented and ready for production use.

弘益人間 (Hongik Ingan) - Benefit All Humanity
