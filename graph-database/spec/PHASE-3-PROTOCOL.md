# WIA-DATA-015: Graph Database Standard
## PHASE 3: PROTOCOL Specification

**Version:** 1.0  
**Status:** Draft  
**Last Updated:** 2025-01-15

---

## 1. Overview

This document defines the PROTOCOL specifications for the WIA-DATA-015 Graph Database Standard.

## 2. Bolt Protocol

### 2.1 Connection Establishment

#### Handshake
```
Client sends: 0x6060B017 (magic number)
              0x00000004 (version 4)
              0x00000003 (version 3)
              0x00000000 (unused)
              0x00000000 (unused)

Server responds: 0x00000004 (selected version)
```

### 2.2 Message Structure

```
[Message Length: 2 bytes]
[Message Type: 1 byte]
[Message Data: variable]
[Chunk Marker: 2 bytes]
```

### 2.3 Message Types

- `INIT`: Initialize connection
- `RUN`: Execute query
- `PULL`: Retrieve results
- `DISCARD`: Discard results
- `BEGIN`: Start transaction
- `COMMIT`: Commit transaction
- `ROLLBACK`: Rollback transaction
- `RESET`: Reset connection
- `GOODBYE`: Close connection

## 3. Query Protocol

### 3.1 Cypher Query Execution

```
RUN "MATCH (n:Person) WHERE n.age > $age RETURN n"
    {"age": 25}
    {}

PULL {"n": 1000}
```

### 3.2 Streaming Results

```
RECORD [data]
RECORD [data]
...
SUCCESS {"type": "r"}
```

## 4. Transaction Protocol

### 4.1 Explicit Transactions

```
BEGIN {}
RUN "CREATE (n:Person {name: 'Alice'})" {} {}
RUN "CREATE (n:Person {name: 'Bob'})" {} {}
COMMIT {}
```

### 4.2 Auto-commit Transactions

```
RUN "MATCH (n) RETURN n LIMIT 10" {} {"autocommit": true}
PULL {"n": -1}
```

## 5. Gremlin Protocol

### 5.1 WebSocket Connection

```javascript
ws://localhost:8182/gremlin

// Execute traversal
{
  "requestId": "cb682578-9d92-4499-9ebc-5c6aa73c5397",
  "op": "eval",
  "processor": "",
  "args": {
    "gremlin": "g.V().hasLabel('person').values('name')",
    "language": "gremlin-groovy"
  }
}
```

### 5.2 Response Format

```json
{
  "requestId": "cb682578-9d92-4499-9ebc-5c6aa73c5397",
  "status": {
    "message": "",
    "code": 200
  },
  "result": {
    "data": ["Alice", "Bob", "Carol"],
    "meta": {}
  }
}
```

## 6. GraphQL Protocol

### 6.1 Schema Definition

```graphql
type Person {
  id: ID!
  name: String!
  age: Int
  friends: [Person] @relationship(type: "KNOWS", direction: OUT)
}

type Query {
  person(id: ID!): Person
  people(limit: Int = 10): [Person]
}

type Mutation {
  createPerson(name: String!, age: Int): Person
}
```

### 6.2 Query Execution

```graphql
query GetPerson {
  person(id: "123") {
    name
    age
    friends {
      name
    }
  }
}
```

## 7. Data Streaming

### 7.1 Change Data Capture

```json
{
  "operation": "CREATE",
  "nodeId": "node_123",
  "labels": ["Person"],
  "properties": {"name": "Alice"},
  "timestamp": "2024-01-15T10:30:00Z"
}
```

### 7.2 Event Stream

```
Event-Type: node-created
Event-ID: evt_12345
Data: {"nodeId": "node_123", "labels": ["Person"]}

Event-Type: edge-created
Event-ID: evt_12346
Data: {"edgeId": "edge_456", "type": "KNOWS"}
```

## 8. Security

### 8.1 TLS Encryption

- Minimum TLS 1.2
- Support TLS 1.3
- Strong cipher suites only

### 8.2 Authentication

- Username/password
- OAuth 2.0
- JWT tokens
- Kerberos
- LDAP integration

### 8.3 Authorization

```
GRANT READ ON GRAPH * TO user_alice
GRANT WRITE ON NODE Person TO role_editor
DENY DELETE ON * TO role_viewer
```

## 9. Connection Pooling

### 9.1 Pool Configuration

```json
{
  "maxConnections": 100,
  "minConnections": 10,
  "acquireTimeout": 30000,
  "idleTimeout": 300000,
  "maxLifetime": 3600000
}
```

## 10. Protocol Versioning

- Version negotiation during handshake
- Backward compatibility within major versions
- Deprecation notices for protocol changes

---

**License:** CC BY 4.0  
**Contact:** standards@wia-official.org
