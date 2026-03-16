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
