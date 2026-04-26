# WIA-DATA-015: Graph Database Standard
## PHASE 4: INTEGRATION Specification

**Version:** 1.0  
**Status:** Draft  
**Last Updated:** 2025-01-15

---

## 1. Overview

This document defines the INTEGRATION specifications for the WIA-DATA-015 Graph Database Standard.

## 2. Database Integration

### 2.1 Relational Database Integration

#### SQL to Graph Mapping

```sql
-- Relational tables
CREATE TABLE persons (
    id INT PRIMARY KEY,
    name VARCHAR(100),
    email VARCHAR(100)
);

CREATE TABLE friendships (
    person_id INT,
    friend_id INT,
    since DATE,
    FOREIGN KEY (person_id) REFERENCES persons(id),
    FOREIGN KEY (friend_id) REFERENCES persons(id)
);
```

```cypher
// Equivalent graph model
CREATE (p:Person {id: 1, name: 'Alice', email: 'alice@example.com'})
CREATE (p)-[:KNOWS {since: date('2020-01-01')}]->(friend:Person)
```

### 2.2 Document Database Integration

#### MongoDB Integration

```javascript
// MongoDB document
{
  "_id": ObjectId("507f1f77bcf86cd799439011"),
  "name": "Alice",
  "email": "alice@example.com",
  "friends": [
    {"friendId": ObjectId("507f191e810c19729de860ea"), "since": "2020-01-01"}
  ]
}

// Graph representation
CREATE (alice:Person {
  mongoId: "507f1f77bcf86cd799439011",
  name: "Alice",
  email: "alice@example.com"
})
CREATE (alice)-[:KNOWS {since: date('2020-01-01')}]->(friend:Person {
  mongoId: "507f191e810c19729de860ea"
})
```

### 2.3 Key-Value Store Integration

#### Redis Integration

```python
# Store graph data in Redis
import redis
r = redis.Redis()

# Node as hash
r.hset("node:123", mapping={
    "labels": "Person",
    "name": "Alice",
    "age": 30
})

# Edge as sorted set
r.zadd("edges:123:out", {
    "edge:456": time.time()
})
```

## 3. Application Framework Integration

### 3.1 Spring Data Neo4j

```java
@Node
public class Person {
    @Id @GeneratedValue
    private Long id;
    
    private String name;
    private Integer age;
    
    @Relationship(type = "KNOWS")
    private Set<Person> friends;
}

@Repository
public interface PersonRepository extends Neo4jRepository<Person, Long> {
    List<Person> findByName(String name);
    
    @Query("MATCH (p:Person)-[:KNOWS*2]->(friend) WHERE p.name = $name RETURN friend")
    List<Person> findFriendsOfFriends(@Param("name") String name);
}
```

### 3.2 Django Integration

```python
from neomodel import StructuredNode, StringProperty, IntegerProperty, RelationshipTo

class Person(StructuredNode):
    name = StringProperty(required=True)
    age = IntegerProperty()
    email = StringProperty(unique_index=True)
    
    knows = RelationshipTo('Person', 'KNOWS')
    
# Usage
alice = Person(name='Alice', age=30).save()
bob = Person(name='Bob', age=28).save()
alice.knows.connect(bob)
```

### 3.3 Express.js Integration

```javascript
const neo4j = require('neo4j-driver');
const express = require('express');

const driver = neo4j.driver('bolt://localhost:7687', neo4j.auth.basic('neo4j', 'password'));
const app = express();

app.get('/person/:name', async (req, res) => {
    const session = driver.session();
    try {
        const result = await session.run(
            'MATCH (p:Person {name: $name}) RETURN p',
            { name: req.params.name }
        );
        res.json(result.records[0].get('p').properties);
    } finally {
        await session.close();
    }
});
```

## 4. ETL Integration

### 4.1 Apache Spark Integration

```scala
import org.neo4j.spark._

val df = spark.read.format("org.neo4j.spark.DataSource")
  .option("url", "bolt://localhost:7687")
  .option("authentication.type", "basic")
  .option("authentication.basic.username", "neo4j")
  .option("authentication.basic.password", "password")
  .option("labels", "Person")
  .load()

df.show()
```

### 4.2 Apache Kafka Integration

```python
from kafka import KafkaConsumer
from neo4j import GraphDatabase

consumer = KafkaConsumer('graph-events')
driver = GraphDatabase.driver("bolt://localhost:7687", auth=("neo4j", "password"))

for message in consumer:
    event = json.loads(message.value)
    
    with driver.session() as session:
        if event['type'] == 'node-created':
            session.run(
                "CREATE (n:Person $props)",
                props=event['properties']
            )
```

## 5. Analytics Integration

### 5.1 Apache Flink Integration

```java
StreamExecutionEnvironment env = StreamExecutionEnvironment.getExecutionEnvironment();

DataStream<Event> events = env.addSource(new Neo4jSource<>());

events
    .keyBy(Event::getNodeId)
    .window(TumblingProcessingTimeWindows.of(Time.minutes(1)))
    .process(new GraphAnalyticsFunction())
    .addSink(new Neo4jSink<>());
```

### 5.2 Jupyter Notebook Integration

```python
import pandas as pd
from neo4j import GraphDatabase

driver = GraphDatabase.driver("bolt://localhost:7687", auth=("neo4j", "password"))

def run_query(query):
    with driver.session() as session:
        result = session.run(query)
        return pd.DataFrame([r.values() for r in result], columns=result.keys())

# Analyze social network
df = run_query("""
    MATCH (p:Person)-[:KNOWS]->(friend)
    RETURN p.name AS person, count(friend) AS friend_count
    ORDER BY friend_count DESC
""")

df.plot(kind='bar', x='person', y='friend_count')
```

## 6. Visualization Integration

### 6.1 D3.js Integration

```javascript
d3.json('/api/graph').then(data => {
    const simulation = d3.forceSimulation(data.nodes)
        .force("link", d3.forceLink(data.links).id(d => d.id))
        .force("charge", d3.forceManyBody())
        .force("center", d3.forceCenter(width / 2, height / 2));
    
    // Render nodes and edges
});
```

### 6.2 Gephi Integration

```python
import networkx as nx
from neo4j import GraphDatabase

def export_to_gephi(driver):
    G = nx.DiGraph()
    
    with driver.session() as session:
        # Export nodes
        result = session.run("MATCH (n) RETURN id(n), labels(n), properties(n)")
        for record in result:
            G.add_node(record[0], labels=record[1], **record[2])
        
        # Export edges
        result = session.run("MATCH ()-[r]->() RETURN id(startNode(r)), id(endNode(r)), type(r)")
        for record in result:
            G.add_edge(record[0], record[1], type=record[2])
    
    nx.write_gexf(G, "graph.gexf")
```

## 7. Machine Learning Integration

### 7.1 TensorFlow/PyTorch Integration

```python
import torch
from torch_geometric.data import Data
from neo4j import GraphDatabase

def load_graph_for_ml(driver):
    with driver.session() as session:
        # Load nodes
        nodes = session.run("MATCH (n:Person) RETURN id(n), n.features").data()
        x = torch.tensor([n['n.features'] for n in nodes])
        
        # Load edges
        edges = session.run("MATCH (a)-[:KNOWS]->(b) RETURN id(a), id(b)").data()
        edge_index = torch.tensor([[e['id(a)'] for e in edges],
                                    [e['id(b)'] for e in edges]])
        
        return Data(x=x, edge_index=edge_index)

# Use in GNN
data = load_graph_for_ml(driver)
model = GCN(num_features=data.num_features, num_classes=7)
```

### 7.2 scikit-learn Integration

```python
from sklearn.ensemble import RandomForestClassifier
import networkx as nx
from neo4j import GraphDatabase

def extract_features(driver):
    G = nx.DiGraph()
    
    with driver.session() as session:
        result = session.run("MATCH (n)-[r]->(m) RETURN id(n), id(m)")
        for record in result:
            G.add_edge(record[0], record[1])
    
    # Compute graph features
    pagerank = nx.pagerank(G)
    clustering = nx.clustering(G)
    
    return pagerank, clustering
```

## 8. Cloud Platform Integration

### 8.1 AWS Integration

```yaml
# CloudFormation template
Resources:
  GraphDatabase:
    Type: AWS::Neptune::DBCluster
    Properties:
      EngineVersion: '1.2.0.0'
      DBClusterIdentifier: my-graph-db
      
  LambdaFunction:
    Type: AWS::Lambda::Function
    Properties:
      Runtime: python3.9
      Handler: index.handler
      Code:
        ZipFile: |
          import boto3
          from gremlin_python.driver import client
          
          def handler(event, context):
              # Query Neptune
              pass
```

### 8.2 Azure Integration

```bicep
resource cosmosDbAccount 'Microsoft.DocumentDB/databaseAccounts@2021-10-15' = {
  name: 'my-graph-db'
  location: location
  properties: {
    capabilities: [
      {
        name: 'EnableGremlin'
      }
    ]
  }
}
```

### 8.3 GCP Integration

```terraform
resource "google_compute_instance" "neo4j" {
  name         = "neo4j-instance"
  machine_type = "n1-standard-4"
  
  boot_disk {
    initialize_params {
      image = "neo4j-enterprise"
    }
  }
}
```

## 9. Monitoring and Observability

### 9.1 Prometheus Integration

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'neo4j'
    metrics_path: '/metrics'
    static_configs:
      - targets: ['localhost:2004']
```

### 9.2 Grafana Dashboards

```json
{
  "dashboard": {
    "title": "Graph Database Metrics",
    "panels": [
      {
        "title": "Query Performance",
        "targets": [
          {
            "expr": "neo4j_transaction_active"
          }
        ]
      }
    ]
  }
}
```

## 10. Deployment

### 10.1 Docker Integration

```yaml
# docker-compose.yml
version: '3'
services:
  neo4j:
    image: neo4j:5.0
    ports:
      - "7474:7474"
      - "7687:7687"
    environment:
      - NEO4J_AUTH=neo4j/password
    volumes:
      - neo4j-data:/data
      
volumes:
  neo4j-data:
```

### 10.2 Kubernetes Integration

```yaml
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: neo4j
spec:
  serviceName: neo4j
  replicas: 3
  template:
    spec:
      containers:
      - name: neo4j
        image: neo4j:5.0-enterprise
        ports:
        - containerPort: 7687
        - containerPort: 7474
```

---

**License:** CC BY 4.0  
**Contact:** standards@wia-official.org


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
