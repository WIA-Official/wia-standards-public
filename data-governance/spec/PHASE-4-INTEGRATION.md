# WIA-DATA-006: Data Governance - PHASE 4: INTEGRATION

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-01-15

---

## Overview

Phase 4 provides comprehensive integration guidance for connecting WIA-DATA-006 Data Governance with existing enterprise systems, tools, and platforms. This phase ensures seamless interoperability across your data ecosystem.

## Integration Architecture

### Reference Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  Data Governance Layer                       │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────┐   │
│  │  Metadata   │  │   Policies   │  │   Workflows     │   │
│  │  Repository │  │   Engine     │  │   Orchestrator  │   │
│  └──────┬──────┘  └───────┬──────┘  └────────┬────────┘   │
└─────────┼──────────────────┼──────────────────┼────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────┐
│              Integration Layer (API Gateway)                 │
└─────────────────────────────────────────────────────────────┘
          │                  │                  │
    ┌─────┴─────┐     ┌─────┴──────┐    ┌─────┴──────┐
    ▼           ▼     ▼            ▼    ▼            ▼
┌───────┐  ┌────────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌────────┐
│Data   │  │Data    │ │Cloud │ │BI    │ │ETL   │ │Security│
│Lake   │  │Catalog │ │DWH   │ │Tools │ │Tools │ │Tools   │
└───────┘  └────────┘ └──────┘ └──────┘ └──────┘ └────────┘
```

## System Integrations

### 1. Data Storage Systems

#### Data Lakes (S3, ADLS, GCS)

**Integration Pattern:** Metadata extraction and cataloging

```python
# AWS S3 Integration
import boto3
from wia_governance import GovernanceClient

s3_client = boto3.client('s3')
gov_client = GovernanceClient()

# Scan S3 bucket and register assets
buckets = s3_client.list_buckets()
for bucket in buckets['Buckets']:
    # Extract metadata
    metadata = extract_s3_metadata(bucket['Name'])

    # Register in governance catalog
    gov_client.assets.create(
        name=bucket['Name'],
        type='s3_bucket',
        location=f"s3://{bucket['Name']}",
        technicalMetadata=metadata
    )
```

**Metadata to Extract:**
- Bucket name, region, creation date
- Object count, total size
- Encryption status
- Access policies
- Lifecycle rules

#### Data Warehouses (Snowflake, Redshift, BigQuery)

**Integration Pattern:** Schema cataloging and lineage tracking

```sql
-- Snowflake Metadata Query
SELECT
    table_catalog as database_name,
    table_schema,
    table_name,
    table_type,
    row_count,
    bytes,
    created,
    last_altered
FROM information_schema.tables
WHERE table_schema NOT IN ('INFORMATION_SCHEMA');
```

**API Integration:**

```python
# Snowflake Integration
from snowflake.connector import connect
from wia_governance import GovernanceClient

# Connect to Snowflake
conn = connect(
    account='your-account',
    user='your-user',
    password='your-password'
)

# Extract metadata
cursor = conn.cursor()
cursor.execute("SELECT * FROM information_schema.tables")

# Register tables
gov_client = GovernanceClient()
for row in cursor:
    gov_client.assets.create({
        'name': f"{row['DATABASE_NAME']}.{row['TABLE_SCHEMA']}.{row['TABLE_NAME']}",
        'type': 'table',
        'technicalMetadata': {
            'rowCount': row['ROW_COUNT'],
            'bytes': row['BYTES'],
            'created': row['CREATED']
        }
    })
```

### 2. Data Catalog Tools

#### Alation Integration

```python
# Alation API Integration
import requests
from wia_governance import GovernanceClient

# Sync metadata from Alation
alation_url = "https://your-instance.alation.com/api/v1"
headers = {"TOKEN": "your-api-token"}

# Get Alation assets
response = requests.get(f"{alation_url}/catalog", headers=headers)
alation_assets = response.json()

# Sync to WIA Governance
gov_client = GovernanceClient()
for asset in alation_assets:
    gov_client.assets.create_or_update({
        'assetId': asset['id'],
        'assetName': asset['title'],
        'description': asset['description'],
        'stewards': asset['stewards']
    })
```

#### Collibra Integration

```python
# Collibra GraphQL Integration
import requests

COLLIBRA_URL = "https://your-instance.collibra.com/graphql/knowledgeGraph/v1"

query = """
query {
  assets(limit: 100) {
    id
    name
    type
    domain
    description
  }
}
"""

response = requests.post(
    COLLIBRA_URL,
    json={'query': query},
    headers={'Authorization': 'Bearer your-token'}
)
```

### 3. BI Tools Integration

#### Tableau Integration

**Integration:** Metadata extraction from Tableau Server

```python
import tableauserverclient as TSC

# Connect to Tableau Server
server = TSC.Server('https://tableau.example.com')
server.auth.sign_in(TSC.TableauAuth('username', 'password'))

# Extract workbook metadata
for workbook in TSC.Pager(server.workbooks):
    gov_client.assets.create({
        'name': workbook.name,
        'type': 'tableau_workbook',
        'owner': workbook.owner_id,
        'technicalMetadata': {
            'projectName': workbook.project_name,
            'createdAt': workbook.created_at,
            'updatedAt': workbook.updated_at
        }
    })
```

#### Power BI Integration

**Integration:** Power BI REST API

```python
import requests

# Power BI API
pbi_url = "https://api.powerbi.com/v1.0/myorg"
headers = {"Authorization": "Bearer your-access-token"}

# Get datasets
response = requests.get(f"{pbi_url}/datasets", headers=headers)
datasets = response.json()['value']

# Register in governance
for dataset in datasets:
    gov_client.assets.create({
        'name': dataset['name'],
        'type': 'powerbi_dataset',
        'assetId': dataset['id']
    })
```

### 4. ETL/ELT Tools Integration

#### Apache Airflow Integration

**Integration:** DAG metadata and lineage tracking

```python
from airflow import DAG
from airflow.operators.python import PythonOperator
from wia_governance import GovernanceClient

def register_lineage(**context):
    """Register data lineage in governance system"""
    gov_client = GovernanceClient()

    # Register transformation
    gov_client.lineage.create({
        'sourceAsset': 'raw_data_table',
        'targetAsset': 'processed_data_table',
        'transformation': {
            'type': 'sql',
            'logic': context['task'].sql
        }
    })

with DAG('data_pipeline', schedule_interval='@daily') as dag:
    extract = PythonOperator(
        task_id='extract',
        python_callable=extract_data
    )

    transform = PythonOperator(
        task_id='transform',
        python_callable=transform_data,
        on_success_callback=register_lineage
    )
```

#### dbt Integration

**Integration:** dbt manifest parsing

```python
import json
from wia_governance import GovernanceClient

# Parse dbt manifest
with open('target/manifest.json') as f:
    manifest = json.load(f)

gov_client = GovernanceClient()

# Register models and lineage
for model_id, model in manifest['nodes'].items():
    if model['resource_type'] == 'model':
        # Register model as asset
        gov_client.assets.create({
            'name': model['name'],
            'type': 'dbt_model',
            'description': model['description'],
            'technicalMetadata': {
                'schema': model['schema'],
                'database': model['database']
            }
        })

        # Register lineage
        for parent in model['depends_on']['nodes']:
            gov_client.lineage.create({
                'sourceAsset': parent,
                'targetAsset': model_id,
                'transformation': {
                    'type': 'sql',
                    'logic': model['raw_sql']
                }
            })
```

### 5. Security Tools Integration

#### IAM Integration

**Integration:** Sync access policies with IAM

```python
import boto3
from wia_governance import GovernanceClient

# AWS IAM Integration
iam = boto3.client('iam')
gov_client = GovernanceClient()

# Sync IAM policies
policies = iam.list_policies(Scope='Local')
for policy in policies['Policies']:
    policy_version = iam.get_policy_version(
        PolicyArn=policy['Arn'],
        VersionId=policy['DefaultVersionId']
    )

    # Register in governance
    gov_client.policies.create({
        'policyName': policy['PolicyName'],
        'policyType': 'access',
        'statement': policy_version['PolicyVersion']['Document']
    })
```

#### SIEM Integration

**Integration:** Send governance events to SIEM

```python
import requests
from wia_governance import GovernanceClient

# Splunk HEC Integration
def send_to_splunk(event):
    splunk_url = "https://splunk.example.com:8088/services/collector"
    headers = {
        "Authorization": "Splunk your-hec-token"
    }

    payload = {
        "event": event,
        "sourcetype": "wia_governance"
    }

    requests.post(splunk_url, json=payload, headers=headers)

# Hook governance events
gov_client = GovernanceClient()
gov_client.webhooks.subscribe(
    events=["access.granted", "policy.violated"],
    callback=send_to_splunk
)
```

### 6. Cloud Platform Integration

#### AWS Integration

**Services:**
- **AWS Glue:** Data catalog sync
- **AWS Lake Formation:** Access control sync
- **AWS CloudTrail:** Audit logging
- **AWS S3:** Data storage metadata

```python
import boto3

# Glue Catalog Integration
glue = boto3.client('glue')

# Sync databases and tables
databases = glue.get_databases()
for db in databases['DatabaseList']:
    tables = glue.get_tables(DatabaseName=db['Name'])

    for table in tables['TableList']:
        gov_client.assets.create({
            'name': f"{db['Name']}.{table['Name']}",
            'type': 'glue_table',
            'technicalMetadata': {
                'columns': table['StorageDescriptor']['Columns'],
                'location': table['StorageDescriptor']['Location']
            }
        })
```

#### Azure Integration

**Services:**
- **Azure Purview:** Data catalog
- **Azure Data Lake:** Storage
- **Azure Active Directory:** Identity management

```python
from azure.purview.catalog import PurviewCatalogClient
from azure.identity import DefaultAzureCredential

# Azure Purview Integration
credential = DefaultAzureCredential()
client = PurviewCatalogClient(
    endpoint="https://your-account.purview.azure.com",
    credential=credential
)

# Sync entities
entities = client.entity.list_entities(type_name="azure_sql_table")
for entity in entities:
    gov_client.assets.create({
        'name': entity['attributes']['name'],
        'type': 'azure_sql_table',
        'classification': entity['classifications']
    })
```

#### GCP Integration

**Services:**
- **BigQuery:** Data warehouse
- **Data Catalog:** Metadata management
- **Cloud IAM:** Access control

```python
from google.cloud import bigquery, datacatalog_v1

# BigQuery Integration
bq_client = bigquery.Client()

# List datasets and tables
for dataset in bq_client.list_datasets():
    for table in bq_client.list_tables(dataset.dataset_id):
        table_ref = bq_client.get_table(table)

        gov_client.assets.create({
            'name': f"{dataset.dataset_id}.{table.table_id}",
            'type': 'bigquery_table',
            'technicalMetadata': {
                'numRows': table_ref.num_rows,
                'numBytes': table_ref.num_bytes,
                'schema': [field.to_api_repr() for field in table_ref.schema]
            }
        })
```

## Integration Patterns

### 1. Real-Time Sync

**Pattern:** Event-driven synchronization

```python
# Kafka Integration for Real-time Events
from kafka import KafkaConsumer
from wia_governance import GovernanceClient

consumer = KafkaConsumer(
    'data-events',
    bootstrap_servers=['kafka:9092']
)

gov_client = GovernanceClient()

for message in consumer:
    event = json.loads(message.value)

    if event['type'] == 'table_created':
        gov_client.assets.create(event['data'])
    elif event['type'] == 'table_updated':
        gov_client.assets.update(event['assetId'], event['data'])
```

### 2. Batch Sync

**Pattern:** Scheduled bulk synchronization

```python
# Daily batch sync
from apscheduler.schedulers.blocking import BlockingScheduler

scheduler = BlockingScheduler()

@scheduler.scheduled_job('cron', hour=2)
def sync_metadata():
    """Daily metadata sync at 2 AM"""
    gov_client = GovernanceClient()

    # Sync from all sources
    sync_from_snowflake(gov_client)
    sync_from_s3(gov_client)
    sync_from_tableau(gov_client)

scheduler.start()
```

### 3. API Gateway Pattern

**Pattern:** Centralized API gateway for all integrations

```yaml
# API Gateway Configuration
apiVersion: v1
kind: Gateway
spec:
  routes:
    - path: /governance/*
      backend: governance-service
    - path: /snowflake/*
      backend: snowflake-connector
    - path: /tableau/*
      backend: tableau-connector
  authentication:
    type: oauth2
  rateLimit:
    requestsPerMinute: 1000
```

## Best Practices

### Integration Guidelines

1. **Use Standard APIs:** Prefer REST APIs over proprietary protocols
2. **Implement Retries:** Handle transient failures gracefully
3. **Log All Operations:** Maintain detailed integration logs
4. **Monitor Performance:** Track sync latency and throughput
5. **Secure Credentials:** Use secrets management (Vault, AWS Secrets Manager)
6. **Version Integrations:** Support multiple API versions
7. **Test Thoroughly:** Test all integration scenarios
8. **Document Everything:** Maintain comprehensive documentation

### Error Handling

```python
import time
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10)
)
def sync_with_retry(source, target):
    """Sync with automatic retry on failure"""
    try:
        data = source.extract()
        target.load(data)
    except Exception as e:
        logging.error(f"Sync failed: {e}")
        raise
```

## Deployment

### Prerequisites

- Kubernetes cluster or equivalent
- API Gateway (Kong, AWS API Gateway)
- Message broker (Kafka, RabbitMQ)
- Secrets management (Vault, AWS Secrets Manager)

### Deployment Steps

1. **Deploy governance services**
2. **Configure integrations**
3. **Test connectivity**
4. **Enable monitoring**
5. **Go live incrementally**

## Next Steps

After completing Phase 4:

1. **Monitor** all integrations
2. **Optimize** performance
3. **Expand** to additional systems
4. **Maintain** and update regularly

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for data-governance is evaluated across three tiers:

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

- `wia-standards/standards/data-governance/api/` — TypeScript SDK skeleton
- `wia-standards/standards/data-governance/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/data-governance/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
