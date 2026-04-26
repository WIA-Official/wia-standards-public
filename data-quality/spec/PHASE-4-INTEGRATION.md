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

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
