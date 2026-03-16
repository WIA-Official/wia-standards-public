# WIA Data Warehouse Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #10B981 (Emerald - DATA domain)

---

## Table of Contents

1. [Overview](#overview)
2. [BI Tool Integration](#bi-tool-integration)
3. [ETL Tool Integration](#etl-tool-integration)
4. [Cloud Platform Integration](#cloud-platform-integration)
5. [Data Catalog Integration](#data-catalog-integration)
6. [Machine Learning Integration](#machine-learning-integration)
7. [Monitoring Integration](#monitoring-integration)
8. [Migration and Compatibility](#migration-and-compatibility)

---

## Overview

### 1.1 Purpose

The WIA Data Warehouse Integration Standard defines how data warehouses integrate with BI tools, ETL platforms, cloud services, data catalogs, and ML systems, ensuring seamless interoperability across the modern data stack.

**Integration Domains**:
- Business Intelligence platforms
- ETL/ELT tools and orchestrators
- Cloud data platforms (AWS, Azure, GCP)
- Data catalogs and governance tools
- Machine Learning platforms
- Monitoring and observability systems

---

## BI Tool Integration

### 2.1 Tableau Integration

**Connection Configuration**:
```json
{
  "connector_type": "wia-datawarehouse",
  "connection_string": "jdbc:wia-dw://warehouse.example.com:443/production",
  "authentication": {
    "method": "username_password|oauth",
    "username": "tableau_user",
    "password": "encrypted_password"
  },
  "performance": {
    "initial_sql": "SET query_timeout = 300",
    "enable_live_mode": true,
    "enable_extract_mode": true,
    "connection_pooling": true
  }
}
```

**Semantic Layer Mapping**:
```sql
-- Published data source with calculated fields
CREATE OR REPLACE VIEW tableau_sales_semantic AS
SELECT
    d.full_date as "Order Date",
    d.year as "Year",
    d.quarter as "Quarter",
    s.store_name as "Store",
    s.region as "Region",
    p.product_name as "Product",
    p.category as "Category",
    SUM(f.sales_amount) as "Total Sales",
    SUM(f.quantity_sold) as "Quantity",
    SUM(f.profit_amount) as "Profit",
    SUM(f.profit_amount) / NULLIF(SUM(f.sales_amount), 0) as "Profit Margin"
FROM fact_sales f
JOIN dim_date d ON f.date_key = d.date_key
JOIN dim_store s ON f.store_key = s.store_key
JOIN dim_product p ON f.product_key = p.product_key
GROUP BY 1,2,3,4,5,6,7;
```

### 2.2 Power BI Integration

**DirectQuery Configuration**:
```json
{
  "data_source": {
    "type": "WIA Data Warehouse",
    "server": "warehouse.example.com",
    "database": "production",
    "connection_mode": "DirectQuery|Import"
  },
  "advanced_options": {
    "query_timeout": 300,
    "command_timeout": 1800,
    "connection_pooling": true,
    "query_folding": true
  }
}
```

**DAX Optimization**:
- Create aggregation tables for common queries
- Use DirectQuery for real-time data
- Implement incremental refresh for large tables

### 2.3 Looker Integration

**LookML Model**:
```lookml
connection: "wia_datawarehouse"

explore: sales {
  join: dim_date {
    type: left_outer
    sql_on: ${fact_sales.date_key} = ${dim_date.date_key} ;;
    relationship: many_to_one
  }

  join: dim_product {
    type: left_outer
    sql_on: ${fact_sales.product_key} = ${dim_product.product_key} ;;
    relationship: many_to_one
  }
}

view: fact_sales {
  sql_table_name: public.fact_sales ;;

  dimension: fact_sales_key {
    primary_key: yes
    type: number
    sql: ${TABLE}.fact_sales_key ;;
  }

  measure: total_sales {
    type: sum
    sql: ${TABLE}.sales_amount ;;
    value_format_name: usd
  }
}
```

---

## ETL Tool Integration

### 3.1 Apache Airflow Integration

**DAG Definition**:
```python
from airflow import DAG
from airflow.providers.wia.operators.datawarehouse import WIADataWarehouseOperator
from datetime import datetime

dag = DAG(
    'daily_sales_etl',
    start_date=datetime(2025, 1, 1),
    schedule_interval='@daily'
)

extract = WIADataWarehouseOperator(
    task_id='extract_source_data',
    sql="""
        SELECT * FROM source_orders
        WHERE order_date = '{{ ds }}'
    """,
    connection_id='source_db',
    dag=dag
)

load = WIADataWarehouseOperator(
    task_id='load_to_warehouse',
    sql="""
        INSERT INTO fact_sales
        SELECT * FROM staging_sales
    """,
    connection_id='wia_warehouse',
    dag=dag
)

extract >> load
```

### 3.2 dbt Integration

**dbt Model**:
```sql
-- models/fact_sales.sql
{{ config(
    materialized='incremental',
    unique_key='fact_sales_key',
    on_schema_change='fail'
) }}

WITH source_data AS (
    SELECT
        {{ dbt_utils.surrogate_key(['order_id', 'line_item']) }} as fact_sales_key,
        date_key,
        product_key,
        customer_key,
        store_key,
        sales_amount,
        quantity_sold,
        created_at
    FROM {{ source('staging', 'orders') }}
    {% if is_incremental() %}
        WHERE created_at > (SELECT MAX(created_at) FROM {{ this }})
    {% endif %}
)

SELECT * FROM source_data
```

**dbt Project Configuration**:
```yaml
# dbt_project.yml
name: 'wia_datawarehouse'
version: '1.0.0'
profile: 'wia_warehouse'

models:
  wia_datawarehouse:
    +materialized: table
    staging:
      +materialized: view
    marts:
      +materialized: table
      +on_schema_change: append_new_columns
```

### 3.3 Informatica Integration

**Mapping Configuration**:
```xml
<Mapping name="Load_Fact_Sales">
  <Source qualifier="SQ_Orders" table="ORDERS"/>
  <Transformation type="Expression">
    <Port name="SURROGATE_KEY" expression="NEXTVAL('fact_sales_seq')"/>
  </Transformation>
  <Transformation type="Lookup" table="DIM_CUSTOMER">
    <LookupCondition>CUSTOMER_ID = LKP_CUSTOMER_ID</LookupCondition>
  </Transformation>
  <Target table="FACT_SALES" connection="WIA_WAREHOUSE"/>
</Mapping>
```

---

## Cloud Platform Integration

### 4.1 AWS Integration

**Amazon Redshift**:
```sql
-- Create external schema for S3 data lake
CREATE EXTERNAL SCHEMA data_lake
FROM DATA CATALOG
DATABASE 'wia_datalake'
IAM_ROLE 'arn:aws:iam::123456789012:role/RedshiftS3Access';

-- Load from S3
COPY fact_sales
FROM 's3://wia-data-bucket/sales/'
IAM_ROLE 'arn:aws:iam::123456789012:role/RedshiftS3Access'
FORMAT AS PARQUET;
```

**AWS Glue Integration**:
```python
import boto3

glue = boto3.client('glue')

# Create Glue job
response = glue.create_job(
    Name='wia-sales-etl',
    Role='arn:aws:iam::123456789012:role/GlueServiceRole',
    Command={
        'Name': 'glueetl',
        'ScriptLocation': 's3://wia-scripts/sales_etl.py'
    },
    DefaultArguments={
        '--TARGET_WAREHOUSE': 'wia_datawarehouse',
        '--TARGET_TABLE': 'fact_sales'
    }
)
```

### 4.2 Google Cloud Integration

**BigQuery**:
```sql
-- Create external table
CREATE EXTERNAL TABLE `project.dataset.fact_sales_external`
OPTIONS (
  format = 'PARQUET',
  uris = ['gs://wia-data-bucket/sales/*.parquet']
);

-- Materialized view for performance
CREATE MATERIALIZED VIEW `project.dataset.sales_summary`
AS
SELECT
  DATE_TRUNC(order_date, MONTH) as month,
  product_category,
  SUM(sales_amount) as total_sales
FROM `project.dataset.fact_sales`
GROUP BY 1, 2;
```

### 4.3 Azure Integration

**Azure Synapse Analytics**:
```sql
-- External data source
CREATE EXTERNAL DATA SOURCE AzureStorage
WITH (
    TYPE = HADOOP,
    LOCATION = 'wasbs://wia-data@storageaccount.blob.core.windows.net',
    CREDENTIAL = AzureStorageCredential
);

-- PolyBase load
CREATE EXTERNAL TABLE staging_sales
WITH (
    LOCATION = '/sales/',
    DATA_SOURCE = AzureStorage,
    FILE_FORMAT = ParquetFormat
);

INSERT INTO fact_sales
SELECT * FROM staging_sales;
```

---

## Data Catalog Integration

### 5.1 Apache Atlas Integration

**Metadata Registration**:
```json
{
  "entity": {
    "typeName": "wia_datawarehouse_table",
    "attributes": {
      "name": "fact_sales",
      "qualifiedName": "wia://warehouse.example.com/production/fact_sales",
      "description": "Sales transactions fact table",
      "owner": "data-engineering-team",
      "columns": [
        {
          "name": "sales_amount",
          "dataType": "decimal(12,2)",
          "classification": ["PII", "Financial"]
        }
      ],
      "lineage": {
        "inputs": ["source_system.orders"],
        "outputs": ["sales_summary_mv"]
      }
    }
  }
}
```

### 5.2 AWS Glue Data Catalog

```python
import boto3

glue = boto3.client('glue')

# Register table
glue.create_table(
    DatabaseName='wia_warehouse',
    TableInput={
        'Name': 'fact_sales',
        'StorageDescriptor': {
            'Columns': [
                {'Name': 'fact_sales_key', 'Type': 'bigint'},
                {'Name': 'date_key', 'Type': 'int'},
                {'Name': 'sales_amount', 'Type': 'decimal(12,2)'}
            ],
            'Location': 's3://wia-warehouse/fact_sales/'
        },
        'Parameters': {
            'classification': 'parquet',
            'grain': 'transaction',
            'owner': 'data-team'
        }
    }
)
```

---

## Machine Learning Integration

### 6.1 Feature Store Integration

```python
from feast import Entity, Feature, FeatureView, FileSource

# Define entity
customer = Entity(
    name="customer",
    value_type=ValueType.STRING,
    description="Customer ID"
)

# Define feature view
customer_features = FeatureView(
    name="customer_sales_features",
    entities=["customer"],
    features=[
        Feature(name="total_sales_30d", dtype=ValueType.DOUBLE),
        Feature(name="avg_order_value_30d", dtype=ValueType.DOUBLE),
        Feature(name="purchase_frequency_30d", dtype=ValueType.INT64),
    ],
    batch_source=FileSource(
        path="wia://warehouse/feature_store/customer_sales_features",
        event_timestamp_column="timestamp",
    )
)
```

### 6.2 ML Model Training Integration

```python
import pandas as pd
from sklearn.ensemble import RandomForestRegressor

# Query training data from warehouse
query = """
    SELECT
        c.customer_segment,
        c.customer_tier,
        SUM(f.sales_amount) as total_sales,
        AVG(f.sales_amount) as avg_order_value,
        COUNT(*) as purchase_frequency
    FROM fact_sales f
    JOIN dim_customer c ON f.customer_key = c.customer_key
    WHERE f.date_key >= 20240101
    GROUP BY c.customer_segment, c.customer_tier
"""

df = warehouse_client.query(query).to_pandas()

# Train model
model = RandomForestRegressor()
model.fit(df[['customer_segment', 'customer_tier']], df['total_sales'])

# Store predictions back to warehouse
predictions = model.predict(test_data)
warehouse_client.write_table('ml_customer_predictions', predictions_df)
```

---

## Monitoring Integration

### 7.1 Prometheus/Grafana

**Metrics Exporter**:
```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'wia_datawarehouse'
    static_configs:
      - targets: ['warehouse-metrics:9090']
    metrics_path: '/metrics'
    scrape_interval: 30s
```

**Grafana Dashboard**:
```json
{
  "dashboard": {
    "title": "WIA Data Warehouse Metrics",
    "panels": [
      {
        "title": "Query Throughput",
        "targets": [
          {
            "expr": "rate(dw_query_total[5m])",
            "legendFormat": "Queries/sec"
          }
        ]
      },
      {
        "title": "Query Duration (p99)",
        "targets": [
          {
            "expr": "histogram_quantile(0.99, dw_query_duration_seconds_bucket)",
            "legendFormat": "p99 latency"
          }
        ]
      }
    ]
  }
}
```

### 7.2 DataDog Integration

```python
from datadog import initialize, api

# Track ETL metrics
def track_etl_metrics(job_name, rows_processed, duration_seconds):
    api.Metric.send(
        metric='wia.warehouse.etl.rows_processed',
        points=rows_processed,
        tags=[f'job:{job_name}']
    )
    api.Metric.send(
        metric='wia.warehouse.etl.duration',
        points=duration_seconds,
        tags=[f'job:{job_name}']
    )
```

---

## Migration and Compatibility

### 8.1 Migration from Legacy Systems

**Oracle to WIA Warehouse**:
```sql
-- Export from Oracle
EXPDP user/pass@oracle_db \
    TABLES=sales_fact,customer_dim \
    DIRECTORY=export_dir \
    DUMPFILE=warehouse_export.dmp

-- Convert schema
python convert_oracle_to_wia.py \
    --input warehouse_export.dmp \
    --output wia_schema.sql

-- Import to WIA Warehouse
wia-cli import \
    --schema wia_schema.sql \
    --data warehouse_export.dmp
```

### 8.2 Compatibility Matrix

| Tool/Platform | Version | Status | Notes |
|---------------|---------|--------|-------|
| Tableau | 2021.4+ | ✅ Full Support | Native connector |
| Power BI | Latest | ✅ Full Support | DirectQuery + Import |
| Looker | 7.0+ | ✅ Full Support | LookML integration |
| dbt | 1.0+ | ✅ Full Support | Adapter available |
| Airflow | 2.0+ | ✅ Full Support | Provider package |
| Snowflake | N/A | 🔄 Partial | Via JDBC/ODBC |
| BigQuery | N/A | 🔄 Partial | Data transfer API |
| Redshift | N/A | 🔄 Partial | S3 import/export |

---

**License**: MIT
**Copyright**: © 2025 WIA Standards Committee
**Philosophy**: 弘益人間 (Benefit All Humanity)
