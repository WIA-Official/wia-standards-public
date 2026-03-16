# WIA-DATA-008: Data Lineage Standard
## PHASE 4: Integration Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This phase defines integration patterns and reference implementations for common data tools and platforms.

## 1. ETL/ELT Tool Integrations

### 1.1 Apache Airflow

**OpenLineage Integration:**

```python
# Install OpenLineage Airflow provider
# pip install openlineage-airflow

# airflow.cfg
[lineage]
backend = openlineage
transport = http://lineage-api:5000

# In your DAG
from airflow import DAG
from airflow.providers.postgres.operators.postgres import PostgresOperator
from openlineage.airflow import DAG as OpenLineageDAG

dag = DAG(
    'customer_etl',
    default_args=default_args,
    schedule_interval='@daily'
)

# Lineage automatically captured
extract = PostgresOperator(
    task_id='extract_customers',
    sql='SELECT * FROM raw.customers WHERE updated_at > {{ ds }}',
    postgres_conn_id='warehouse',
    dag=dag
)

transform = PostgresOperator(
    task_id='transform_customers',
    sql='''
        INSERT INTO staging.customers
        SELECT
            id,
            LOWER(TRIM(email)) as email,
            created_at
        FROM raw.customers
    ''',
    postgres_conn_id='warehouse',
    dag=dag
)

extract >> transform
```

**Custom Lineage Extraction:**

```python
from openlineage.client import OpenLineageClient
from openlineage.client.run import RunEvent, RunState, Run, Job, Dataset

client = OpenLineageClient(url='http://lineage-api:5000')

def emit_lineage(**context):
    run_id = context['run_id']

    event = RunEvent(
        eventType=RunState.START,
        eventTime=datetime.now().isoformat(),
        run=Run(runId=str(run_id)),
        job=Job(
            namespace='airflow',
            name=context['dag'].dag_id
        ),
        inputs=[
            Dataset(
                namespace='postgres://warehouse',
                name='public.raw_customers'
            )
        ],
        outputs=[
            Dataset(
                namespace='postgres://warehouse',
                name='public.staging_customers'
            )
        ]
    )

    client.emit(event)

emit_lineage_task = PythonOperator(
    task_id='emit_lineage',
    python_callable=emit_lineage,
    provide_context=True,
    dag=dag
)
```

### 1.2 dbt (Data Build Tool)

**Automatic Lineage Extraction:**

```python
# dbt generates lineage in manifest.json
# Parse manifest and submit to lineage API

import json
from wia_lineage import LineageClient

client = LineageClient(api_key='your_key')

# Read dbt manifest
with open('target/manifest.json') as f:
    manifest = json.load(f)

# Extract lineage
for node_name, node in manifest['nodes'].items():
    if node['resource_type'] == 'model':
        # Submit node
        client.create_node({
            'node_type': 'table',
            'namespace': f"dbt://{manifest['metadata']['project_name']}",
            'name': node['name'],
            'metadata': {
                'sql': node['raw_sql'],
                'materialization': node['config']['materialized']
            }
        })

        # Submit edges for dependencies
        for dep in node['depends_on']['nodes']:
            client.create_edge({
                'source': dep,
                'target': node_name,
                'type': 'derived_from',
                'transformation': {
                    'type': 'sql',
                    'logic': node['raw_sql']
                }
            })
```

**dbt-Lineage Hook:**

```python
# macros/lineage_hook.sql
{% macro on_run_end(results) %}
  {% if execute %}
    {{ adapter.execute(
        "CALL send_lineage_event('" ~
        invocation_id ~ "', '" ~
        project_name ~ "')"
    ) }}
  {% endif %}
{% endmacro %}
```

### 1.3 Apache Spark

**Spark Listener Integration:**

```scala
import org.apache.spark.sql.util.QueryExecutionListener
import org.apache.spark.sql.execution.QueryExecution

class LineageListener extends QueryExecutionListener {
  override def onSuccess(
    funcName: String,
    qe: QueryExecution,
    durationNs: Long
  ): Unit = {
    val inputs = extractInputs(qe.analyzed)
    val outputs = extractOutputs(qe.analyzed)

    sendLineageEvent(LineageEvent(
      eventType = "COMPLETE",
      job = Job("spark", funcName),
      inputs = inputs,
      outputs = outputs
    ))
  }

  override def onFailure(
    funcName: String,
    qe: QueryExecution,
    exception: Exception
  ): Unit = {
    sendLineageEvent(LineageEvent(
      eventType = "FAIL",
      job = Job("spark", funcName),
      error = exception.getMessage
    ))
  }
}

// Register listener
spark.listenerManager.register(new LineageListener())
```

**Python (PySpark):**

```python
from pyspark.sql import SparkSession
from wia_lineage import SparkLineageExtractor

spark = SparkSession.builder \
    .appName("lineage_example") \
    .config("spark.extraListeners",
            "com.wia.lineage.SparkLineageListener") \
    .getOrCreate()

# Lineage automatically captured
df = spark.read.parquet("s3://bucket/raw/customers")
result = df.filter(df.active == True) \
    .groupBy("country") \
    .count()
result.write.parquet("s3://bucket/processed/customer_counts")
```

## 2. Database Integrations

### 2.1 PostgreSQL

**Query Log Parsing:**

```python
import re
from wia_lineage import LineageClient

client = LineageClient()

# Parse PostgreSQL logs
def parse_query_log(log_line):
    # Extract tables from INSERT/UPDATE/SELECT queries
    pattern = r'(INSERT INTO|UPDATE|FROM|JOIN)\s+([a-z_]+\.[a-z_]+)'
    matches = re.findall(pattern, log_line, re.IGNORECASE)

    tables = [match[1] for match in matches]
    return tables

# Submit lineage
def process_query(query_text, user):
    tables = parse_query_log(query_text)

    if 'INSERT INTO' in query_text:
        target = tables[0]
        sources = tables[1:]

        for source in sources:
            client.create_edge({
                'source': f'postgres://db/{source}',
                'target': f'postgres://db/{target}',
                'type': 'derived_from',
                'metadata': {
                    'query': query_text,
                    'user': user
                }
            })
```

**SQL Function for Lineage:**

```sql
-- Create function to log lineage
CREATE OR REPLACE FUNCTION log_lineage(
    p_source VARCHAR,
    p_target VARCHAR,
    p_transformation TEXT
) RETURNS VOID AS $$
BEGIN
    INSERT INTO lineage_log (
        source_table,
        target_table,
        transformation,
        created_at
    ) VALUES (
        p_source,
        p_target,
        p_transformation,
        NOW()
    );
END;
$$ LANGUAGE plpgsql;

-- Use in ETL
INSERT INTO warehouse.customer_summary
SELECT customer_id, COUNT(*) as order_count
FROM raw.orders
GROUP BY customer_id;

SELECT log_lineage(
    'raw.orders',
    'warehouse.customer_summary',
    'GROUP BY aggregation'
);
```

### 2.2 Snowflake

**Query History Integration:**

```sql
-- Access Snowflake query history
SELECT
    query_id,
    query_text,
    user_name,
    start_time,
    end_time,
    database_name,
    schema_name
FROM snowflake.account_usage.query_history
WHERE start_time >= DATEADD(hour, -1, CURRENT_TIMESTAMP())
  AND query_type IN ('INSERT', 'UPDATE', 'CREATE_TABLE_AS_SELECT');

-- Parse and extract lineage
-- Submit to WIA-DATA-008 API
```

**Snowflake Task Integration:**

```sql
CREATE OR REPLACE TASK lineage_capture_task
  WAREHOUSE = compute_wh
  SCHEDULE = '1 MINUTE'
AS
CALL SYSTEM$SEND_LINEAGE_TO_EXTERNAL_API(
    'https://lineage-api.example.com/v1/events',
    'Bearer YOUR_API_KEY'
);
```

## 3. BI Tool Integrations

### 3.1 Tableau

**Metadata API Integration:**

```python
import tableauserverclient as TSC
from wia_lineage import LineageClient

# Connect to Tableau Server
server = TSC.Server('https://tableau.example.com')
server.auth.sign_in(TSC.TableauAuth('username', 'password'))

lineage_client = LineageClient()

# Extract workbook lineage
for workbook in TSC.Pager(server.workbooks):
    # Get data sources
    server.workbooks.populate_connections(workbook)

    for connection in workbook.connections:
        # Create node for report
        report_node = lineage_client.create_node({
            'node_type': 'report',
            'namespace': 'tableau://server',
            'name': workbook.name,
            'metadata': {
                'owner': workbook.owner_id,
                'project': workbook.project_name
            }
        })

        # Create node for data source
        source_node = lineage_client.create_node({
            'node_type': 'table',
            'namespace': f'{connection.server_address}',
            'name': connection.datasource_name
        })

        # Create edge
        lineage_client.create_edge({
            'source': source_node['node_id'],
            'target': report_node['node_id'],
            'type': 'derived_from'
        })
```

### 3.2 Power BI

**Power BI REST API:**

```python
import requests
from wia_lineage import LineageClient

# Authenticate with Power BI
def get_powerbi_token():
    # OAuth flow
    pass

# Get datasets and reports
def extract_powerbi_lineage():
    token = get_powerbi_token()
    headers = {'Authorization': f'Bearer {token}'}

    # Get all datasets
    datasets = requests.get(
        'https://api.powerbi.com/v1.0/myorg/datasets',
        headers=headers
    ).json()

    lineage_client = LineageClient()

    for dataset in datasets['value']:
        # Get dataset sources
        sources = requests.get(
            f'https://api.powerbi.com/v1.0/myorg/datasets/{dataset["id"]}/datasources',
            headers=headers
        ).json()

        # Submit lineage
        for source in sources['value']:
            lineage_client.create_edge({
                'source': f'{source["connectionDetails"]["server"]}/{source["connectionDetails"]["database"]}',
                'target': f'powerbi://dataset/{dataset["id"]}',
                'type': 'derived_from'
            })
```

### 3.3 Looker

**LookML Parser:**

```python
import lkml
from wia_lineage import LineageClient

# Parse LookML files
with open('views/customers.view.lkml') as f:
    view = lkml.load(f)

client = LineageClient()

# Extract view lineage
for view_def in view['views']:
    view_name = view_def['name']

    # Create node for view
    view_node = client.create_node({
        'node_type': 'view',
        'namespace': 'looker://instance',
        'name': view_name
    })

    # Get sql_table_name for source
    if 'sql_table_name' in view_def:
        source_table = view_def['sql_table_name']

        client.create_edge({
            'source': f'database://{source_table}',
            'target': view_node['node_id'],
            'type': 'derived_from'
        })

    # Parse derived_table for complex lineage
    if 'derived_table' in view_def:
        sql = view_def['derived_table']['sql']
        # Parse SQL to extract source tables
        sources = parse_sql_sources(sql)

        for source in sources:
            client.create_edge({
                'source': source,
                'target': view_node['node_id'],
                'type': 'derived_from',
                'transformation': {'type': 'sql', 'logic': sql}
            })
```

## 4. Data Catalog Integrations

### 4.1 Apache Atlas

**Bidirectional Sync:**

```python
from apache_atlas.client.base_client import AtlasClient
from wia_lineage import LineageClient

atlas_client = AtlasClient('http://atlas:21000', ('admin', 'admin'))
lineage_client = LineageClient()

# Sync Atlas entities to WIA lineage
def sync_atlas_to_wia():
    # Get all tables from Atlas
    tables = atlas_client.search_entities('hive_table')

    for table in tables:
        # Create node in WIA lineage
        lineage_client.create_node({
            'node_type': 'table',
            'namespace': f'hive://{table["attributes"]["qualifiedName"]}',
            'name': table['attributes']['name'],
            'metadata': {
                'atlas_guid': table['guid'],
                'owner': table['attributes']['owner']
            }
        })

# Sync WIA lineage to Atlas
def sync_wia_to_atlas():
    nodes = lineage_client.get_nodes()

    for node in nodes:
        # Create Atlas entity
        entity = {
            'typeName': 'DataSet',
            'attributes': {
                'qualifiedName': node['qualified_name'],
                'name': node['name']
            }
        }
        atlas_client.entity_post(entity)
```

### 4.2 AWS Glue Data Catalog

**Glue Integration:**

```python
import boto3
from wia_lineage import LineageClient

glue = boto3.client('glue')
lineage_client = LineageClient()

# Extract Glue catalog lineage
def sync_glue_catalog():
    # Get all databases
    databases = glue.get_databases()

    for db in databases['DatabaseList']:
        # Get tables in database
        tables = glue.get_tables(DatabaseName=db['Name'])

        for table in tables['TableList']:
            # Create node
            node = lineage_client.create_node({
                'node_type': 'table',
                'namespace': f'glue://{db["Name"]}',
                'name': table['Name'],
                'schema': {
                    'fields': [
                        {
                            'name': col['Name'],
                            'type': col['Type']
                        }
                        for col in table['StorageDescriptor']['Columns']
                    ]
                }
            })

            # Get table lineage from Glue
            if 'Parameters' in table and 'lineage' in table['Parameters']:
                lineage_data = json.loads(table['Parameters']['lineage'])

                for source in lineage_data.get('sources', []):
                    lineage_client.create_edge({
                        'source': source,
                        'target': node['node_id'],
                        'type': 'derived_from'
                    })
```

## 5. ML Platform Integrations

### 5.1 MLflow

**Experiment Tracking Integration:**

```python
import mlflow
from wia_lineage import LineageClient

lineage_client = LineageClient()

# Track model lineage
with mlflow.start_run() as run:
    # Load training data
    df = spark.read.parquet('s3://bucket/training_data')

    # Log lineage
    lineage_client.create_node({
        'node_type': 'model',
        'namespace': 'mlflow://experiments',
        'name': f'model_{run.info.run_id}',
        'metadata': {
            'mlflow_run_id': run.info.run_id,
            'experiment_id': run.info.experiment_id
        }
    })

    lineage_client.create_edge({
        'source': 's3://bucket/training_data',
        'target': f'mlflow://model/{run.info.run_id}',
        'type': 'derived_from',
        'transformation': {
            'type': 'machine_learning',
            'algorithm': 'random_forest'
        }
    })

    # Train model
    model = train_model(df)
    mlflow.sklearn.log_model(model, 'model')
```

### 5.2 Kubeflow

**Pipeline Lineage:**

```python
from kfp import dsl
from wia_lineage import KubeflowLineageLogger

@dsl.pipeline(name='Training Pipeline')
def training_pipeline():
    # Data preparation
    prep_op = dsl.ContainerOp(
        name='prepare_data',
        image='gcr.io/project/prepare:latest'
    )

    # Log lineage
    lineage_logger = KubeflowLineageLogger()
    lineage_logger.log_component_lineage(
        component=prep_op,
        inputs=['gs://bucket/raw_data'],
        outputs=['gs://bucket/prepared_data']
    )

    # Training
    train_op = dsl.ContainerOp(
        name='train_model',
        image='gcr.io/project/train:latest'
    ).after(prep_op)

    lineage_logger.log_component_lineage(
        component=train_op,
        inputs=['gs://bucket/prepared_data'],
        outputs=['gs://bucket/models/model.pkl']
    )
```

## 6. CI/CD Integration

### 6.1 GitHub Actions

```yaml
name: Lineage Impact Analysis

on:
  pull_request:
    paths:
      - 'sql/**'
      - 'dbt/**'

jobs:
  lineage_check:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2

      - name: Analyze Impact
        uses: wia-official/lineage-action@v1
        with:
          api_key: ${{ secrets.LINEAGE_API_KEY }}
          changed_files: ${{ steps.files.outputs.all }}

      - name: Comment PR
        if: steps.analyze.outputs.high_impact == 'true'
        uses: actions/github-script@v5
        with:
          script: |
            github.rest.issues.createComment({
              issue_number: context.issue.number,
              owner: context.repo.owner,
              repo: context.repo.repo,
              body: '⚠️ High impact changes detected!\n\n' +
                    'Affected downstream assets: ' +
                    '${{ steps.analyze.outputs.affected_count }}'
            })
```

### 6.2 GitLab CI

```yaml
lineage-check:
  stage: test
  image: wia/lineage-cli:latest
  script:
    - lineage impact-analysis --files="$CI_MERGE_REQUEST_DIFF_BASE_SHA..$CI_COMMIT_SHA"
  only:
    - merge_requests
  artifacts:
    reports:
      junit: lineage-report.xml
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
