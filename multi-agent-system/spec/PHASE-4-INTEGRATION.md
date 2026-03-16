# WIA-AI-016 Phase 4: Integration Specification

> 弘益人間 (홍익인간) · Benefit All Humanity

## Overview

Phase 4 defines platform integration, middleware connectors, deployment strategies, and interoperability requirements for WIA-AI-016 multi-agent systems.

## Platform Integration

### Supported Platforms

- **Node.js**: v16+ with TypeScript support
- **Python**: 3.8+ with asyncio
- **Java**: JDK 11+ with Spring Boot
- **Go**: 1.18+ with goroutines
- **Rust**: 1.60+ with tokio
- **Browser**: Modern browsers with WebSocket support

### Integration Patterns

#### Embedded Agents

Agents run within application process:

```typescript
import { Agent, MultiAgentSystem } from '@wia/multi-agent-system';

const system = new MultiAgentSystem({
  platformUrl: 'https://api.example.com',
  authentication: { token: process.env.API_TOKEN }
});

const agent = new MyAgent('agent-001');
await system.registerAgent(agent);
await agent.start();
```

#### Standalone Agents

Agents run as independent services:

```bash
# Deploy as container
docker run -d \
  -e AGENT_ID=agent-001 \
  -e PLATFORM_URL=https://api.example.com \
  -e API_TOKEN=${TOKEN} \
  wia/agent:latest
```

#### Agent Clusters

Multiple agents coordinated by cluster manager:

```yaml
# Kubernetes deployment
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-cluster
spec:
  replicas: 10
  template:
    spec:
      containers:
      - name: agent
        image: wia/agent:latest
        env:
        - name: CLUSTER_MODE
          value: "true"
        - name: COORDINATOR_URL
          value: "http://coordinator:8080"
```

## Middleware Connectors

### Message Queue Integration

#### RabbitMQ

```typescript
import { RabbitMQTransport } from '@wia/transports';

const transport = new RabbitMQTransport({
  url: 'amqp://localhost:5672',
  exchange: 'agent-messages',
  queue: 'agent-001-queue'
});

agent.setTransport(transport);
```

#### Apache Kafka

```typescript
import { KafkaTransport } from '@wia/transports';

const transport = new KafkaTransport({
  brokers: ['localhost:9092'],
  topic: 'agent-messages',
  groupId: 'agent-group-001'
});
```

#### Redis Pub/Sub

```typescript
import { RedisTransport } from '@wia/transports';

const transport = new RedisTransport({
  host: 'localhost',
  port: 6379,
  channel: 'agent-channel'
});
```

### Database Integration

#### PostgreSQL

```typescript
import { PostgresStore } from '@wia/stores';

const store = new PostgresStore({
  host: 'localhost',
  database: 'agents',
  user: 'agent_user',
  password: process.env.DB_PASSWORD
});

agent.setStore(store);
```

#### MongoDB

```typescript
import { MongoStore } from '@wia/stores';

const store = new MongoStore({
  url: 'mongodb://localhost:27017',
  database: 'agents',
  collection: 'agent-data'
});
```

#### Redis

```typescript
import { RedisStore } from '@wia/stores';

const store = new RedisStore({
  host: 'localhost',
  port: 6379,
  db: 0,
  keyPrefix: 'agent:001:'
});
```

## Cloud Platform Integration

### AWS Integration

```typescript
// AWS Lambda Function
exports.handler = async (event) => {
  const agent = new MyAgent('lambda-agent');

  // Process SQS messages
  for (const record of event.Records) {
    const message = JSON.parse(record.body);
    await agent.handleMessage(message);
  }

  return { statusCode: 200 };
};

// AWS IoT Core
const iotTransport = new AWSIoTTransport({
  endpoint: 'xxx.iot.us-east-1.amazonaws.com',
  clientId: 'agent-001',
  certificatePath: './certs/certificate.pem',
  privateKeyPath: './certs/private.key'
});
```

### Google Cloud Integration

```typescript
// Cloud Functions
export const agentHandler = functions.https.onRequest(async (req, res) => {
  const agent = new MyAgent('gcp-agent');
  const result = await agent.processRequest(req.body);
  res.json(result);
});

// Cloud Pub/Sub
const pubsubTransport = new GCPPubSubTransport({
  projectId: 'my-project',
  topicName: 'agent-messages',
  subscriptionName: 'agent-001-sub'
});
```

### Azure Integration

```typescript
// Azure Functions
module.exports = async function (context, req) {
  const agent = new MyAgent('azure-agent');
  const result = await agent.processRequest(req.body);
  context.res = { body: result };
};

// Azure Service Bus
const serviceBusTransport = new AzureServiceBusTransport({
  connectionString: process.env.SERVICE_BUS_CONNECTION,
  queueName: 'agent-messages'
});
```

## API Gateway Integration

### REST API Gateway

```typescript
// Express.js integration
import express from 'express';
import { AgentMiddleware } from '@wia/middleware';

const app = express();
const agentMiddleware = new AgentMiddleware(agent);

app.use('/agent', agentMiddleware.router());
app.listen(3000);
```

### GraphQL Integration

```typescript
import { ApolloServer } from 'apollo-server';
import { AgentResolvers } from '@wia/graphql';

const server = new ApolloServer({
  typeDefs,
  resolvers: AgentResolvers(agent)
});

await server.listen(4000);
```

## Blockchain Integration

### Ethereum Smart Contract

```solidity
// Agent Registry Contract
pragma solidity ^0.8.0;

contract AgentRegistry {
    struct Agent {
        string id;
        address owner;
        string capabilities;
        uint256 reputation;
    }

    mapping(string => Agent) public agents;

    function registerAgent(
        string memory id,
        string memory capabilities
    ) public {
        agents[id] = Agent(id, msg.sender, capabilities, 50);
    }
}
```

### IPFS Integration

```typescript
import { create } from 'ipfs-http-client';

const ipfs = create({ url: 'http://localhost:5001' });

// Store agent knowledge base
const knowledgeBase = JSON.stringify(agent.getKnowledge());
const { cid } = await ipfs.add(knowledgeBase);

// Retrieve from IPFS
const data = await ipfs.cat(cid);
```

## Container Deployment

### Docker

```dockerfile
FROM node:18-alpine

WORKDIR /app

COPY package*.json ./
RUN npm ci --production

COPY . .

ENV NODE_ENV=production
ENV AGENT_ID=agent-001

CMD ["node", "dist/index.js"]
```

### Docker Compose

```yaml
version: '3.8'

services:
  agent-001:
    build: .
    environment:
      - AGENT_ID=agent-001
      - PLATFORM_URL=http://platform:8080
    depends_on:
      - platform
      - rabbitmq
      - postgres

  platform:
    image: wia/platform:latest
    ports:
      - "8080:8080"

  rabbitmq:
    image: rabbitmq:3-management
    ports:
      - "5672:5672"
      - "15672:15672"

  postgres:
    image: postgres:14
    environment:
      - POSTGRES_DB=agents
      - POSTGRES_USER=agent_user
      - POSTGRES_PASSWORD=${DB_PASSWORD}
```

### Kubernetes

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-config
data:
  PLATFORM_URL: "http://platform-service:8080"
  LOG_LEVEL: "info"

---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: multi-agent-system
spec:
  replicas: 5
  selector:
    matchLabels:
      app: agent
  template:
    metadata:
      labels:
        app: agent
    spec:
      containers:
      - name: agent
        image: wia/agent:1.0.0
        envFrom:
        - configMapRef:
            name: agent-config
        - secretRef:
            name: agent-secrets
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"

---
apiVersion: v1
kind: Service
metadata:
  name: agent-service
spec:
  selector:
    app: agent
  ports:
  - port: 8080
    targetPort: 8080
  type: LoadBalancer
```

## Monitoring & Observability

### Prometheus Metrics

```typescript
import { PrometheusExporter } from '@wia/monitoring';

const exporter = new PrometheusExporter({
  port: 9090,
  metrics: {
    tasksCompleted: 'counter',
    avgResponseTime: 'histogram',
    activeAgents: 'gauge'
  }
});

agent.setMetricsExporter(exporter);
```

### OpenTelemetry

```typescript
import { trace } from '@opentelemetry/api';
import { NodeTracerProvider } from '@opentelemetry/sdk-trace-node';

const provider = new NodeTracerProvider();
provider.register();

const tracer = trace.getTracer('multi-agent-system');

// Instrument agent operations
const span = tracer.startSpan('process-task');
await agent.processTask(task);
span.end();
```

### Logging

```typescript
import { createLogger, format, transports } from 'winston';

const logger = createLogger({
  format: format.combine(
    format.timestamp(),
    format.json()
  ),
  transports: [
    new transports.File({ filename: 'agent.log' }),
    new transports.Console()
  ]
});

agent.setLogger(logger);
```

## Testing Integration

### Unit Testing

```typescript
import { MockAgent, MockPlatform } from '@wia/testing';

describe('Agent Communication', () => {
  let agent: MyAgent;
  let platform: MockPlatform;

  beforeEach(() => {
    platform = new MockPlatform();
    agent = new MyAgent('test-agent');
    agent.setPlatform(platform);
  });

  it('should send message successfully', async () => {
    await agent.sendMessage({
      performative: 'inform',
      receiver: ['agent-002'],
      content: { test: true }
    });

    expect(platform.getMessageCount()).toBe(1);
  });
});
```

### Integration Testing

```typescript
import { TestHarness } from '@wia/testing';

describe('Multi-Agent System Integration', () => {
  let harness: TestHarness;

  beforeAll(async () => {
    harness = new TestHarness({
      agents: ['agent-001', 'agent-002', 'agent-003']
    });
    await harness.start();
  });

  it('should coordinate task allocation', async () => {
    const task = { type: 'measurement' };
    const result = await harness.submitTask(task);
    expect(result.status).toBe('completed');
  });
});
```

## Migration & Compatibility

### Version Migration

```typescript
import { Migration } from '@wia/migration';

const migration = new Migration({
  from: '1.0.0',
  to: '2.0.0',
  transforms: [
    {
      path: 'message.performative',
      transform: (value) => value.toLowerCase()
    }
  ]
});

const migratedData = migration.apply(oldData);
```

### Backward Compatibility

```typescript
// Support both old and new message formats
agent.onMessage(async (message) => {
  const normalized = normalizeMessage(message);

  // Handle both versions
  if (message.formatVersion === '1.0.0') {
    return handleV1Message(normalized);
  } else {
    return handleV2Message(normalized);
  }
});
```

## Security Integration

### OAuth 2.0

```typescript
import { OAuth2Client } from 'google-auth-library';

const client = new OAuth2Client(CLIENT_ID);

async function verifyToken(token: string) {
  const ticket = await client.verifyIdToken({
    idToken: token,
    audience: CLIENT_ID
  });
  return ticket.getPayload();
}
```

### JWT Authentication

```typescript
import jwt from 'jsonwebtoken';

const token = jwt.sign(
  { agentId: 'agent-001', capabilities: ['sensing'] },
  process.env.JWT_SECRET,
  { expiresIn: '1h' }
);

// Verify
const decoded = jwt.verify(token, process.env.JWT_SECRET);
```

---

**WIA-AI-016 Phase 4 Specification v1.0**
© 2025 SmileStory Inc. / World Certification Industry Association
弘익人間 (홍익인간) · Benefit All Humanity
