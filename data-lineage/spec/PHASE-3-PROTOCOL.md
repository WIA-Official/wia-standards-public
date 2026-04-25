# WIA-DATA-008: Data Lineage Standard
## PHASE 3: Protocol Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This phase defines the protocols for real-time lineage streaming, event-driven architecture, and inter-system communication.

## 1. Event Streaming Protocol

### 1.1 Apache Kafka

**Topic Naming:**
```
lineage.events.{environment}.{entity_type}

Examples:
- lineage.events.prod.nodes
- lineage.events.prod.edges
- lineage.events.dev.provenance
```

**Message Format:**

```json
{
  "key": "node_id or edge_id",
  "value": {
    "schema_version": "1.0.0",
    "event_type": "created | updated | deleted",
    "timestamp": "ISO8601",
    "entity_type": "node | edge | provenance",
    "entity_id": "uuid",
    "payload": {...}
  },
  "headers": {
    "producer": "service_name",
    "correlation_id": "uuid",
    "causation_id": "uuid"
  }
}
```

**Partitioning Strategy:**
- Partition by namespace for locality
- Ensures ordered processing within namespace

**Retention:**
- Production: 30 days
- Development: 7 days
- Archive to S3/GCS for long-term storage

### 1.2 AWS Kinesis

**Stream Naming:**
```
lineage-events-{environment}
```

**Shard Key:** Hash of qualified_name for even distribution

**Record Format:**
```json
{
  "Data": "base64_encoded_json",
  "PartitionKey": "qualified_name_hash",
  "SequenceNumber": "string"
}
```

### 1.3 Google Cloud Pub/Sub

**Topic Format:**
```
projects/{project}/topics/lineage-{environment}-{entity_type}
```

**Message Attributes:**
- `event_type`: created|updated|deleted
- `entity_type`: node|edge|provenance
- `timestamp`: ISO8601
- `producer`: service_name

## 2. WebSocket Protocol

For real-time lineage updates in web applications.

### 2.1 Connection

```javascript
const ws = new WebSocket('wss://lineage.api.example.com/v1/stream');

ws.addEventListener('open', () => {
  ws.send(JSON.stringify({
    type: 'subscribe',
    channels: ['node.updates', 'edge.updates'],
    filters: {
      namespace: 'postgres://warehouse'
    }
  }));
});
```

### 2.2 Message Format

**Server → Client:**
```json
{
  "type": "event",
  "channel": "node.updates",
  "event": {
    "event_type": "node.updated",
    "node_id": "...",
    "timestamp": "2025-12-26T10:00:00Z",
    "data": {...}
  }
}
```

**Client → Server (Subscription):**
```json
{
  "type": "subscribe",
  "channels": ["node.updates"],
  "filters": {
    "node_type": "table",
    "namespace": "postgres://warehouse"
  }
}
```

**Heartbeat:**
```json
{
  "type": "ping",
  "timestamp": "2025-12-26T10:00:00Z"
}
```

## 3. gRPC Protocol

High-performance RPC for service-to-service communication.

### 3.1 Service Definition

```protobuf
syntax = "proto3";

package wia.lineage.v1;

service LineageService {
  rpc SubmitEvent(LineageEvent) returns (EventResponse);
  rpc StreamEvents(StreamRequest) returns (stream LineageEvent);
  rpc GetNode(NodeRequest) returns (Node);
  rpc GetUpstream(LineageRequest) returns (LineageGraph);
  rpc GetDownstream(LineageRequest) returns (LineageGraph);
}

message LineageEvent {
  string event_type = 1;
  string event_time = 2;
  Run run = 3;
  Job job = 4;
  repeated Dataset inputs = 5;
  repeated Dataset outputs = 6;
}

message Node {
  string node_id = 1;
  string node_type = 2;
  string namespace = 3;
  string name = 4;
  map<string, string> metadata = 5;
}

message LineageGraph {
  repeated Node nodes = 1;
  repeated Edge edges = 2;
}
```

### 3.2 Streaming

**Bidirectional Streaming:**
```protobuf
service LineageService {
  rpc StreamBidirectional(stream LineageEvent)
    returns (stream EventResponse);
}
```

## 4. GraphQL Protocol

For flexible client queries.

### 4.1 Schema

```graphql
type Query {
  node(id: ID!): Node
  searchNodes(query: String, type: NodeType, limit: Int): [Node!]!
  upstream(nodeId: ID!, depth: Int): LineageGraph!
  downstream(nodeId: ID!, depth: Int): LineageGraph!
  columnLineage(table: String!, column: String!): ColumnLineage!
}

type Mutation {
  submitEvent(event: LineageEventInput!): EventResponse!
  updateNode(id: ID!, input: NodeUpdateInput!): Node!
}

type Subscription {
  nodeUpdated(namespace: String): Node!
  edgeCreated(sourceId: ID): Edge!
}

type Node {
  id: ID!
  type: NodeType!
  namespace: String!
  name: String!
  qualifiedName: String!
  metadata: JSON
  schema: Schema
  upstream(depth: Int): [Node!]!
  downstream(depth: Int): [Node!]!
}

type Edge {
  id: ID!
  source: Node!
  target: Node!
  type: EdgeType!
  transformation: Transformation
}

enum NodeType {
  TABLE
  VIEW
  COLUMN
  REPORT
  MODEL
}

enum EdgeType {
  DERIVED_FROM
  AGGREGATED_FROM
  JOINED_WITH
}
```

### 4.2 Example Query

```graphql
query GetTableLineage {
  node(id: "table_123") {
    name
    namespace
    upstream(depth: 2) {
      name
      type
    }
    downstream(depth: 3) {
      name
      type
    }
  }
}
```

### 4.3 Subscription Example

```graphql
subscription WatchNodeUpdates {
  nodeUpdated(namespace: "postgres://warehouse") {
    id
    name
    type
    metadata
  }
}
```

## 5. Message Queue Protocols

### 5.1 RabbitMQ

**Exchange:** `lineage.events`

**Routing Keys:**
```
lineage.node.created
lineage.node.updated
lineage.edge.created
lineage.provenance.logged
```

**Queue Naming:**
```
lineage.{service_name}.{entity_type}
```

**Message Properties:**
- `content_type`: application/json
- `delivery_mode`: 2 (persistent)
- `correlation_id`: For request tracing
- `timestamp`: Unix timestamp

### 5.2 Amazon SQS

**Queue URL:**
```
https://sqs.{region}.amazonaws.com/{account}/lineage-{environment}-{type}
```

**Message Attributes:**
- `EventType`: StringValue
- `EntityType`: StringValue
- `Timestamp`: NumberValue
- `Producer`: StringValue

## 6. Change Data Capture (CDC)

### 6.1 Database Triggers

Capture lineage changes from database operations.

**PostgreSQL Example:**
```sql
CREATE OR REPLACE FUNCTION log_lineage_change()
RETURNS TRIGGER AS $$
BEGIN
  INSERT INTO lineage_changelog (
    table_name,
    operation,
    old_data,
    new_data,
    changed_at
  ) VALUES (
    TG_TABLE_NAME,
    TG_OP,
    row_to_json(OLD),
    row_to_json(NEW),
    NOW()
  );
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER lineage_change_trigger
AFTER INSERT OR UPDATE OR DELETE ON lineage_nodes
FOR EACH ROW EXECUTE FUNCTION log_lineage_change();
```

### 6.2 Debezium Integration

**Connector Configuration:**
```json
{
  "name": "lineage-cdc-connector",
  "config": {
    "connector.class": "io.debezium.connector.postgresql.PostgresConnector",
    "database.hostname": "localhost",
    "database.port": "5432",
    "database.user": "lineage_user",
    "database.dbname": "lineage_db",
    "database.server.name": "lineage",
    "table.include.list": "public.lineage_nodes,public.lineage_edges",
    "topic.prefix": "lineage.cdc"
  }
}
```

## 7. Inter-Service Communication

### 7.1 Service Mesh (Istio/Linkerd)

**Metadata Propagation:**
```yaml
apiVersion: networking.istio.io/v1alpha3
kind: VirtualService
metadata:
  name: lineage-service
spec:
  hosts:
  - lineage.svc.cluster.local
  http:
  - match:
    - headers:
        x-lineage-trace:
          regex: ".*"
    route:
    - destination:
        host: lineage.svc.cluster.local
      headers:
        request:
          add:
            x-lineage-propagate: "true"
```

### 7.2 Distributed Tracing

**OpenTelemetry Integration:**
```python
from opentelemetry import trace
from wia_lineage import LineageClient

tracer = trace.get_tracer(__name__)

with tracer.start_as_current_span("submit_lineage"):
    client = LineageClient()
    client.submit_event({...})
```

## 8. Data Synchronization

### 8.1 Event Sourcing

All lineage changes stored as immutable events.

**Event Store Schema:**
```sql
CREATE TABLE lineage_events (
  event_id UUID PRIMARY KEY,
  event_type VARCHAR(50),
  aggregate_id UUID,
  aggregate_type VARCHAR(50),
  event_data JSONB,
  metadata JSONB,
  created_at TIMESTAMP,
  version INTEGER
);
```

### 8.2 CQRS Pattern

- **Command Side**: Handle writes (event submission)
- **Query Side**: Optimized read models (materialized views)

**Read Model Update:**
```sql
CREATE MATERIALIZED VIEW lineage_graph_view AS
SELECT
  n.node_id,
  n.name,
  n.namespace,
  array_agg(e.target_node_id) as downstream_ids
FROM lineage_nodes n
LEFT JOIN lineage_edges e ON n.node_id = e.source_node_id
GROUP BY n.node_id, n.name, n.namespace;

-- Refresh periodically or on-demand
REFRESH MATERIALIZED VIEW CONCURRENTLY lineage_graph_view;
```

## 9. Security Protocols

### 9.1 mTLS

Mutual TLS for service-to-service authentication.

**Certificate Requirements:**
- X.509 certificates
- 2048-bit RSA or 256-bit ECDSA
- Valid for max 90 days
- Subject Alternative Name (SAN) required

### 9.2 API Gateway

**Kong Configuration:**
```yaml
plugins:
  - name: rate-limiting
    config:
      minute: 100
      policy: local
  - name: jwt
    config:
      secret_is_base64: false
  - name: request-transformer
    config:
      add:
        headers:
          - X-Lineage-Version:1.0.0
```

## 10. Monitoring & Observability

### 10.1 Metrics (Prometheus)

**Exposed Metrics:**
```
# Events processed
lineage_events_total{type="node_created"}
lineage_events_total{type="edge_created"}

# Processing latency
lineage_event_processing_duration_seconds

# Graph size
lineage_graph_nodes_total
lineage_graph_edges_total

# API performance
lineage_api_request_duration_seconds{endpoint="/nodes"}
```

### 10.2 Health Checks

**Kubernetes Probes:**
```yaml
livenessProbe:
  httpGet:
    path: /health/live
    port: 8080
  initialDelaySeconds: 10
  periodSeconds: 5

readinessProbe:
  httpGet:
    path: /health/ready
    port: 8080
  initialDelaySeconds: 5
  periodSeconds: 3
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**

---

## Annex A — Conformance Tier Matrix

WIA conformance for data-lineage is evaluated across three tiers:

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

- `wia-standards/standards/data-lineage/api/` — TypeScript SDK skeleton
- `wia-standards/standards/data-lineage/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/data-lineage/simulator/` — interactive browser-based simulator for the PHASE protocol

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
