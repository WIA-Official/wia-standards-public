# PHASE 4: Integration Specification

**Standard:** WIA-DATA-007 (Data Catalog)
**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

이 문서는 Data Catalog를 다양한 데이터 시스템 및 도구와 통합하는 방법을 정의합니다. Apache Atlas, DataHub, Collibra 등 주요 플랫폼과의 통합 패턴을 제공합니다.

## 1. Apache Atlas Integration

### 1.1 Overview

Apache Atlas는 Hadoop 에코시스템을 위한 오픈소스 데이터 거버넌스 및 메타데이터 관리 프레임워크입니다.

### 1.2 Connection Configuration

```yaml
atlas:
  url: "http://atlas.example.com:21000"
  auth:
    type: "basic"
    username: "admin"
    password: "${ATLAS_PASSWORD}"
  kafka:
    bootstrap_servers: "kafka.example.com:9092"
    topic: "ATLAS_HOOK"
  ssl:
    enabled: true
    truststore_path: "/path/to/truststore.jks"
    truststore_password: "${TRUSTSTORE_PASSWORD}"
```

### 1.3 Entity Type Mapping

| WIA-DATA-007 | Apache Atlas | Description |
|--------------|--------------|-------------|
| DATASET | hive_table | 테이블 |
| COLUMN | hive_column | 컬럼 |
| DATABASE | hive_db | 데이터베이스 |
| PROCESS | Process | ETL 프로세스 |
| LINEAGE | LineageRelation | 계보 관계 |

### 1.4 Metadata Extraction

#### Python SDK Example

```python
from apache_atlas.client.base_client import AtlasClient
from apache_atlas.model.instance import AtlasEntity

# Atlas client initialization
client = AtlasClient(
    'http://atlas.example.com:21000',
    ('admin', 'password')
)

# Create table entity
table_entity = AtlasEntity(
    typeName='hive_table',
    attributes={
        'name': 'customers',
        'qualifiedName': 'prod_db.public.customers@cluster',
        'description': '고객 정보를 저장하는 메인 테이블',
        'owner': 'data-team@company.com',
        'createTime': 1640000000000,
        'lastAccessTime': 1640500000000,
        'retention': 0,
        'tableType': 'MANAGED_TABLE'
    }
)

# Create column entities
columns = []
for col_name in ['customer_id', 'email', 'full_name']:
    column = AtlasEntity(
        typeName='hive_column',
        attributes={
            'name': col_name,
            'qualifiedName': f'prod_db.public.customers.{col_name}@cluster',
            'type': 'string',
            'comment': f'{col_name} column'
        }
    )
    columns.append(column)

# Set relationship
table_entity.attributes['columns'] = columns

# Create entity in Atlas
response = client.entity.create_entity(table_entity)
```

### 1.5 Lineage Integration

```python
# Create lineage process
lineage_process = AtlasEntity(
    typeName='Process',
    attributes={
        'name': 'customer_etl_pipeline',
        'qualifiedName': 'customer_etl@cluster',
        'inputs': [
            {'typeName': 'hive_table', 'uniqueAttributes': {
                'qualifiedName': 'source_db.customers@cluster'
            }}
        ],
        'outputs': [
            {'typeName': 'hive_table', 'uniqueAttributes': {
                'qualifiedName': 'target_db.customer_analytics@cluster'
            }}
        ]
    }
)

client.entity.create_entity(lineage_process)
```

## 2. DataHub Integration

### 2.1 Overview

DataHub는 LinkedIn에서 개발한 현대적인 데이터 카탈로그 플랫폼입니다.

### 2.2 Connection Configuration

```yaml
datahub:
  gms_url: "http://datahub-gms.example.com:8080"
  kafka:
    bootstrap_servers: "kafka.example.com:9092"
    schema_registry_url: "http://schema-registry.example.com:8081"
  auth:
    token: "${DATAHUB_TOKEN}"
```

### 2.3 Python SDK Integration

```python
from datahub.emitter.mce_builder import make_dataset_urn
from datahub.emitter.rest_emitter import DatahubRestEmitter
from datahub.metadata.schema_classes import (
    DatasetPropertiesClass,
    OwnerClass,
    OwnershipClass,
    OwnershipTypeClass
)

# Initialize emitter
emitter = DatahubRestEmitter(
    gms_server='http://datahub-gms.example.com:8080',
    token='${DATAHUB_TOKEN}'
)

# Create dataset URN
dataset_urn = make_dataset_urn(
    platform='postgres',
    name='prod_db.public.customers',
    env='PROD'
)

# Create dataset properties
dataset_properties = DatasetPropertiesClass(
    description='고객 정보를 저장하는 메인 테이블',
    customProperties={
        'row_count': '500000',
        'size_bytes': '52428800'
    }
)

# Create ownership
ownership = OwnershipClass(
    owners=[
        OwnerClass(
            owner='urn:li:corpuser:data-team',
            type=OwnershipTypeClass.DATAOWNER
        )
    ]
)

# Emit metadata
from datahub.metadata.com.linkedin.pegasus2avro.mxe import MetadataChangeEvent

mce = MetadataChangeEvent(
    proposedSnapshot=DatasetSnapshot(
        urn=dataset_urn,
        aspects=[
            dataset_properties,
            ownership
        ]
    )
)

emitter.emit_mce(mce)
```

### 2.4 Lineage Ingestion

```python
from datahub.metadata.schema_classes import (
    UpstreamLineageClass,
    UpstreamClass,
    DatasetLineageTypeClass
)

# Define upstream lineage
upstream_lineage = UpstreamLineageClass(
    upstreams=[
        UpstreamClass(
            dataset=make_dataset_urn('postgres', 'source_db.customers', 'PROD'),
            type=DatasetLineageTypeClass.TRANSFORMED
        ),
        UpstreamClass(
            dataset=make_dataset_urn('postgres', 'source_db.orders', 'PROD'),
            type=DatasetLineageTypeClass.TRANSFORMED
        )
    ]
)

# Emit lineage
emitter.emit_mce(
    MetadataChangeEvent(
        proposedSnapshot=DatasetSnapshot(
            urn=dataset_urn,
            aspects=[upstream_lineage]
        )
    )
)
```

## 3. Collibra Integration

### 3.1 Overview

Collibra는 엔터프라이즈급 데이터 인텔리전스 플랫폼입니다.

### 3.2 REST API Integration

```python
import requests
from typing import Dict, List

class CollibraClient:
    def __init__(self, base_url: str, username: str, password: str):
        self.base_url = base_url
        self.session = requests.Session()
        self.session.auth = (username, password)
        self.session.headers.update({
            'Content-Type': 'application/json'
        })

    def create_asset(self, data: Dict) -> Dict:
        """Create a new asset in Collibra"""
        url = f"{self.base_url}/rest/2.0/assets"
        response = self.session.post(url, json=data)
        response.raise_for_status()
        return response.json()

    def add_attribute(self, asset_id: str, attribute_type_id: str, value: str):
        """Add attribute to an asset"""
        url = f"{self.base_url}/rest/2.0/attributes"
        data = {
            "assetId": asset_id,
            "typeId": attribute_type_id,
            "value": value
        }
        response = self.session.post(url, json=data)
        response.raise_for_status()
        return response.json()

# Usage
client = CollibraClient(
    base_url='https://collibra.example.com',
    username='admin',
    password='password'
)

# Create table asset
table_asset = client.create_asset({
    "name": "customers",
    "displayName": "customers",
    "typeId": "00000000-0000-0000-0000-000000031007",  # Table type
    "domainId": "customer-domain-id"
})

# Add description
client.add_attribute(
    asset_id=table_asset['id'],
    attribute_type_id="00000000-0000-0000-0000-000000003114",  # Description type
    value="고객 정보를 저장하는 메인 테이블"
)
```

## 4. Database Integration

### 4.1 PostgreSQL Metadata Extraction

```python
import psycopg2
from typing import List, Dict

def extract_postgres_metadata(connection_string: str) -> List[Dict]:
    """Extract metadata from PostgreSQL database"""
    conn = psycopg2.connect(connection_string)
    cur = conn.cursor()

    # Get table metadata
    query = """
    SELECT
        table_schema,
        table_name,
        obj_description((table_schema || '.' || table_name)::regclass) as table_comment
    FROM information_schema.tables
    WHERE table_schema NOT IN ('pg_catalog', 'information_schema')
    AND table_type = 'BASE TABLE'
    """

    cur.execute(query)
    tables = cur.fetchall()

    metadata = []
    for schema, table, comment in tables:
        # Get column information
        col_query = """
        SELECT
            column_name,
            data_type,
            is_nullable,
            col_description((table_schema || '.' || table_name)::regclass, ordinal_position) as column_comment
        FROM information_schema.columns
        WHERE table_schema = %s AND table_name = %s
        ORDER BY ordinal_position
        """

        cur.execute(col_query, (schema, table))
        columns = cur.fetchall()

        metadata.append({
            'schema': schema,
            'table': table,
            'description': comment,
            'columns': [
                {
                    'name': col_name,
                    'data_type': col_type,
                    'nullable': nullable == 'YES',
                    'description': col_comment
                }
                for col_name, col_type, nullable, col_comment in columns
            ]
        })

    cur.close()
    conn.close()

    return metadata
```

### 4.2 MySQL Metadata Extraction

```python
import mysql.connector

def extract_mysql_metadata(host: str, database: str, user: str, password: str):
    """Extract metadata from MySQL database"""
    conn = mysql.connector.connect(
        host=host,
        database=database,
        user=user,
        password=password
    )

    cursor = conn.cursor(dictionary=True)

    # Get table metadata
    cursor.execute("""
        SELECT
            TABLE_NAME,
            TABLE_COMMENT,
            TABLE_ROWS,
            DATA_LENGTH
        FROM information_schema.TABLES
        WHERE TABLE_SCHEMA = DATABASE()
        AND TABLE_TYPE = 'BASE TABLE'
    """)

    tables = cursor.fetchall()

    metadata = []
    for table in tables:
        # Get column information
        cursor.execute("""
            SELECT
                COLUMN_NAME,
                DATA_TYPE,
                IS_NULLABLE,
                COLUMN_KEY,
                COLUMN_COMMENT
            FROM information_schema.COLUMNS
            WHERE TABLE_SCHEMA = DATABASE()
            AND TABLE_NAME = %s
            ORDER BY ORDINAL_POSITION
        """, (table['TABLE_NAME'],))

        columns = cursor.fetchall()

        metadata.append({
            'table': table['TABLE_NAME'],
            'description': table['TABLE_COMMENT'],
            'row_count': table['TABLE_ROWS'],
            'size_bytes': table['DATA_LENGTH'],
            'columns': columns
        })

    cursor.close()
    conn.close()

    return metadata
```

## 5. Cloud Platform Integration

### 5.1 AWS Glue Data Catalog

```python
import boto3

def sync_with_glue_catalog(database_name: str, table_name: str, metadata: Dict):
    """Sync metadata with AWS Glue Data Catalog"""
    glue = boto3.client('glue')

    # Create or update table
    table_input = {
        'Name': table_name,
        'Description': metadata['description'],
        'Owner': metadata['owner'],
        'StorageDescriptor': {
            'Columns': [
                {
                    'Name': col['name'],
                    'Type': col['data_type'],
                    'Comment': col.get('description', '')
                }
                for col in metadata['columns']
            ],
            'Location': metadata['location'],
            'InputFormat': 'org.apache.hadoop.mapred.TextInputFormat',
            'OutputFormat': 'org.apache.hadoop.hive.ql.io.HiveIgnoreKeyTextOutputFormat',
            'SerdeInfo': {
                'SerializationLibrary': 'org.apache.hadoop.hive.serde2.lazy.LazySimpleSerDe'
            }
        },
        'Parameters': {
            'classification': 'csv',
            'quality_score': str(metadata.get('quality_score', 0))
        }
    }

    try:
        glue.update_table(
            DatabaseName=database_name,
            TableInput=table_input
        )
    except glue.exceptions.EntityNotFoundException:
        glue.create_table(
            DatabaseName=database_name,
            TableInput=table_input
        )
```

### 5.2 Google Cloud Data Catalog

```python
from google.cloud import datacatalog_v1

def sync_with_gcp_data_catalog(project_id: str, location: str, metadata: Dict):
    """Sync metadata with Google Cloud Data Catalog"""
    client = datacatalog_v1.DataCatalogClient()

    # Create entry
    entry = datacatalog_v1.Entry()
    entry.display_name = metadata['name']
    entry.description = metadata['description']
    entry.type_ = datacatalog_v1.EntryType.TABLE

    # Set BigQuery table reference
    entry.bigquery_table_spec.table_source_type = (
        datacatalog_v1.TableSourceType.BIGQUERY_TABLE
    )

    # Create entry group
    entry_group = datacatalog_v1.EntryGroup()
    entry_group.display_name = "WIA Data Catalog"

    parent = f"projects/{project_id}/locations/{location}"

    # Create or update entry
    request = datacatalog_v1.CreateEntryRequest(
        parent=f"{parent}/entryGroups/wia_catalog",
        entry_id=metadata['qualified_name'].replace('.', '_'),
        entry=entry
    )

    response = client.create_entry(request=request)

    # Add tags
    tag_template = f"projects/{project_id}/locations/{location}/tagTemplates/wia_metadata"
    tag = datacatalog_v1.Tag()
    tag.template = tag_template
    tag.fields['owner'].string_value = metadata['owner']['email']
    tag.fields['quality_score'].double_value = metadata['quality_score']

    client.create_tag(
        parent=response.name,
        tag=tag
    )
```

## 6. BI Tool Integration

### 6.1 Tableau Integration

```python
import tableauserverclient as TSC

def sync_with_tableau(server_url: str, username: str, password: str, metadata: Dict):
    """Sync metadata with Tableau Server"""
    tableau_auth = TSC.TableauAuth(username, password)
    server = TSC.Server(server_url)

    with server.auth.sign_in(tableau_auth):
        # Get all datasources
        all_datasources, pagination_item = server.datasources.get()

        # Find matching datasource
        for datasource in all_datasources:
            if datasource.name == metadata['name']:
                # Update description
                datasource.description = metadata['description']

                # Add tags
                for tag in metadata['tags']:
                    datasource.tags.add(tag)

                # Update datasource
                server.datasources.update(datasource)

                # Update metadata
                server.datasources.update_hyper_data(
                    datasource.id,
                    metadata['location']
                )
```

### 6.2 Power BI Integration

```python
import requests
from typing import Dict

class PowerBIClient:
    def __init__(self, tenant_id: str, client_id: str, client_secret: str):
        self.tenant_id = tenant_id
        self.client_id = client_id
        self.client_secret = client_secret
        self.access_token = self._get_access_token()

    def _get_access_token(self) -> str:
        """Get Azure AD access token"""
        url = f"https://login.microsoftonline.com/{self.tenant_id}/oauth2/v2.0/token"
        data = {
            'grant_type': 'client_credentials',
            'client_id': self.client_id,
            'client_secret': self.client_secret,
            'scope': 'https://analysis.windows.net/powerbi/api/.default'
        }
        response = requests.post(url, data=data)
        return response.json()['access_token']

    def update_dataset_metadata(self, workspace_id: str, dataset_id: str, metadata: Dict):
        """Update Power BI dataset metadata"""
        url = f"https://api.powerbi.com/v1.0/myorg/groups/{workspace_id}/datasets/{dataset_id}"
        headers = {
            'Authorization': f'Bearer {self.access_token}',
            'Content-Type': 'application/json'
        }
        data = {
            'name': metadata['name'],
            'description': metadata['description'],
            'configuredBy': metadata['owner']['email']
        }
        response = requests.patch(url, headers=headers, json=data)
        response.raise_for_status()
        return response.json()
```

## 7. Workflow Integration

### 7.1 Apache Airflow Integration

```python
from airflow import DAG
from airflow.operators.python import PythonOperator
from datetime import datetime, timedelta
import requests

def extract_metadata(**context):
    """Extract metadata from source system"""
    # Implementation
    pass

def transform_metadata(**context):
    """Transform metadata to WIA-DATA-007 format"""
    # Implementation
    pass

def load_to_catalog(**context):
    """Load metadata to data catalog"""
    catalog_api = "https://catalog.example.com/v1/datasets"
    headers = {"Authorization": f"Bearer {context['params']['api_token']}"}

    metadata = context['ti'].xcom_pull(task_ids='transform_metadata')

    response = requests.post(catalog_api, headers=headers, json=metadata)
    response.raise_for_status()

# Define DAG
default_args = {
    'owner': 'data-team',
    'depends_on_past': False,
    'start_date': datetime(2025, 1, 1),
    'email': ['data-team@company.com'],
    'email_on_failure': True,
    'retries': 3,
    'retry_delay': timedelta(minutes=5)
}

dag = DAG(
    'metadata_sync_to_catalog',
    default_args=default_args,
    description='Sync metadata to data catalog',
    schedule_interval='0 2 * * *',  # Daily at 2 AM
    catchup=False
)

extract_task = PythonOperator(
    task_id='extract_metadata',
    python_callable=extract_metadata,
    dag=dag
)

transform_task = PythonOperator(
    task_id='transform_metadata',
    python_callable=transform_metadata,
    dag=dag
)

load_task = PythonOperator(
    task_id='load_to_catalog',
    python_callable=load_to_catalog,
    params={'api_token': '{{ var.value.catalog_api_token }}'},
    dag=dag
)

extract_task >> transform_task >> load_task
```

### 7.2 dbt Integration

```yaml
# dbt_project.yml
models:
  my_project:
    +meta:
      owner: "data-team@company.com"
      data_catalog:
        sync: true
        tags: ["dbt-model", "analytics"]
        quality_tier: "gold"
```

```python
# Post-dbt hook to sync metadata
import json
from pathlib import Path

def sync_dbt_metadata_to_catalog():
    """Sync dbt model metadata to data catalog"""
    manifest_path = Path("target/manifest.json")

    with open(manifest_path) as f:
        manifest = json.load(f)

    for node_id, node in manifest['nodes'].items():
        if node['resource_type'] == 'model':
            metadata = {
                'name': node['name'],
                'qualified_name': f"{node['database']}.{node['schema']}.{node['name']}",
                'description': node['description'],
                'owner': node['meta'].get('owner'),
                'tags': node['tags'] + node['meta'].get('data_catalog', {}).get('tags', []),
                'columns': [
                    {
                        'name': col_name,
                        'description': col_info['description'],
                        'data_type': col_info['data_type']
                    }
                    for col_name, col_info in node['columns'].items()
                ]
            }

            # Send to catalog
            sync_to_catalog(metadata)
```

## 8. Integration Best Practices

### 8.1 Error Handling

```python
import logging
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10)
)
def sync_metadata_with_retry(metadata: Dict):
    """Sync metadata with automatic retry"""
    try:
        response = requests.post(
            'https://catalog.example.com/v1/datasets',
            json=metadata,
            timeout=30
        )
        response.raise_for_status()
        logger.info(f"Successfully synced {metadata['name']}")
        return response.json()
    except requests.exceptions.RequestException as e:
        logger.error(f"Failed to sync {metadata['name']}: {e}")
        raise
```

### 8.2 Batch Processing

```python
from typing import List, Iterator
import itertools

def batch_sync_metadata(metadata_list: List[Dict], batch_size: int = 100):
    """Sync metadata in batches for better performance"""
    def chunked_iterable(iterable, size):
        it = iter(iterable)
        while True:
            chunk = list(itertools.islice(it, size))
            if not chunk:
                break
            yield chunk

    for batch in chunked_iterable(metadata_list, batch_size):
        response = requests.post(
            'https://catalog.example.com/v1/datasets/batch',
            json={'datasets': batch}
        )
        response.raise_for_status()
        print(f"Synced batch of {len(batch)} datasets")
```

### 8.3 Monitoring & Alerting

```python
from prometheus_client import Counter, Histogram
import time

# Metrics
metadata_sync_total = Counter('metadata_sync_total', 'Total metadata sync attempts')
metadata_sync_success = Counter('metadata_sync_success', 'Successful metadata syncs')
metadata_sync_failed = Counter('metadata_sync_failed', 'Failed metadata syncs')
metadata_sync_duration = Histogram('metadata_sync_duration_seconds', 'Metadata sync duration')

def sync_with_monitoring(metadata: Dict):
    """Sync metadata with monitoring"""
    metadata_sync_total.inc()
    start_time = time.time()

    try:
        sync_metadata(metadata)
        metadata_sync_success.inc()
    except Exception as e:
        metadata_sync_failed.inc()
        raise
    finally:
        duration = time.time() - start_time
        metadata_sync_duration.observe(duration)
```

---

**Complete!** All PHASE specifications have been created.

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
