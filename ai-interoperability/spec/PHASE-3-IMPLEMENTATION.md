# WIA AI-INTEROPERABILITY - PHASE 3: Implementation Details

## Version 1.0

> 홍익인간 (弘益人間) - Benefit All Humanity

## Document Information

- **Standard**: WIA-AI-INTEROPERABILITY
- **Phase**: 3 - Implementation Details
- **Version**: 1.0.0
- **Status**: Draft
- **Date**: 2026-01-13
- **Authors**: WIA Standards Committee

---

## 📋 Table of Contents

1. [Implementation Guide](#1-implementation-guide)
2. [Development Environment](#2-development-environment)
3. [Code Examples](#3-code-examples)
4. [Best Practices](#4-best-practices)
5. [Testing Strategy](#5-testing-strategy)
6. [Deployment Guide](#6-deployment-guide)
7. [Troubleshooting](#7-troubleshooting)
8. [Performance Optimization](#8-performance-optimization)

---

## 1. Implementation Guide

### 1.1 Implementation Overview

This section provides step-by-step instructions for implementing AI interoperability capabilities in your systems.

#### 1.1.1 Implementation Phases

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    IMPLEMENTATION ROADMAP                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Phase A: Foundation (Week 1-2)                                         │
│  ├── Set up development environment                                     │
│  ├── Install SDK and dependencies                                       │
│  ├── Configure authentication                                           │
│  └── Implement basic connectivity                                       │
│                                                                          │
│  Phase B: Core Integration (Week 3-4)                                   │
│  ├── Implement UACP message handling                                    │
│  ├── Register with AI Registry                                          │
│  ├── Publish capabilities                                               │
│  └── Basic inference endpoints                                          │
│                                                                          │
│  Phase C: Advanced Features (Week 5-6)                                  │
│  ├── Streaming support                                                  │
│  ├── Semantic mappings                                                  │
│  ├── Federation participation                                           │
│  └── Advanced error handling                                            │
│                                                                          │
│  Phase D: Production Readiness (Week 7-8)                               │
│  ├── Performance optimization                                           │
│  ├── Security hardening                                                 │
│  ├── Monitoring integration                                             │
│  └── Documentation and testing                                          │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

#### 1.1.2 Prerequisites

Before starting implementation, ensure you have:

| Requirement | Minimum Version | Notes |
|-------------|-----------------|-------|
| Node.js | 20.x LTS | For TypeScript SDK |
| Python | 3.11+ | For Python SDK |
| Go | 1.21+ | For Go SDK |
| Rust | 1.75+ | For Rust SDK |
| Docker | 24.x | For containerization |
| Kubernetes | 1.28+ | For orchestration (optional) |

### 1.2 Quick Start

#### 1.2.1 Installation

```bash
# TypeScript/Node.js
npm install @wia/ai-interop-sdk

# Python
pip install wia-ai-interop

# Go
go get github.com/wia-official/ai-interop-go

# Rust
cargo add wia-ai-interop
```

#### 1.2.2 Basic Setup

```typescript
// TypeScript Quick Start
import { AIInteropClient, UAcPConfig } from '@wia/ai-interop-sdk';

// Initialize client
const config: UACPConfig = {
  systemId: 'my-ai-system',
  systemName: 'My AI System',
  version: '1.0.0',
  endpoint: 'https://my-ai.example.com/uacp',
  auth: {
    type: 'jwt',
    credentials: {
      issuer: 'https://auth.example.com',
      audience: 'ai-interop',
      privateKey: process.env.PRIVATE_KEY
    }
  }
};

const client = new AIInteropClient(config);

// Connect to the AI Interop network
await client.connect();

// Register capabilities
await client.registerCapability({
  type: 'text-classification',
  name: 'sentiment-analysis',
  version: '2.0.0',
  inputSchema: {
    type: 'object',
    properties: {
      text: { type: 'string', maxLength: 10000 },
      language: { type: 'string', default: 'en' }
    },
    required: ['text']
  },
  outputSchema: {
    type: 'object',
    properties: {
      sentiment: { type: 'string', enum: ['positive', 'negative', 'neutral'] },
      confidence: { type: 'number', minimum: 0, maximum: 1 }
    }
  },
  performance: {
    latency: { p50: 50, p95: 150, p99: 300 },
    throughput: { requestsPerSecond: 1000 }
  }
});

console.log('AI System registered successfully!');
```

---

## 2. Development Environment

### 2.1 Project Structure

```
my-ai-interop-service/
├── src/
│   ├── config/
│   │   ├── index.ts           # Configuration loader
│   │   ├── schema.ts          # Config validation schema
│   │   └── defaults.ts        # Default values
│   │
│   ├── handlers/
│   │   ├── inference.ts       # Inference request handlers
│   │   ├── streaming.ts       # Streaming handlers
│   │   ├── health.ts          # Health check handlers
│   │   └── registry.ts        # Registry interaction handlers
│   │
│   ├── middleware/
│   │   ├── auth.ts            # Authentication middleware
│   │   ├── validation.ts      # Request validation
│   │   ├── rateLimit.ts       # Rate limiting
│   │   ├── audit.ts           # Audit logging
│   │   └── metrics.ts         # Metrics collection
│   │
│   ├── services/
│   │   ├── uacp/
│   │   │   ├── client.ts      # UACP client
│   │   │   ├── server.ts      # UACP server
│   │   │   ├── message.ts     # Message handling
│   │   │   └── protocol.ts    # Protocol implementation
│   │   │
│   │   ├── semantic/
│   │   │   ├── mapper.ts      # Semantic mapping
│   │   │   ├── ontology.ts    # Ontology management
│   │   │   └── transform.ts   # Data transformation
│   │   │
│   │   └── trust/
│   │       ├── auth.ts        # Authentication
│   │       ├── authz.ts       # Authorization
│   │       └── audit.ts       # Audit trails
│   │
│   ├── models/
│   │   ├── system.ts          # AI System model
│   │   ├── capability.ts      # Capability model
│   │   ├── message.ts         # Message model
│   │   └── types.ts           # Type definitions
│   │
│   ├── utils/
│   │   ├── logger.ts          # Logging utility
│   │   ├── errors.ts          # Error classes
│   │   ├── crypto.ts          # Cryptographic utilities
│   │   └── retry.ts           # Retry logic
│   │
│   └── index.ts               # Application entry point
│
├── tests/
│   ├── unit/                  # Unit tests
│   ├── integration/           # Integration tests
│   ├── e2e/                   # End-to-end tests
│   └── fixtures/              # Test fixtures
│
├── docs/                      # Documentation
├── scripts/                   # Build/deploy scripts
├── docker/                    # Docker configurations
├── k8s/                       # Kubernetes manifests
│
├── package.json
├── tsconfig.json
├── jest.config.js
├── .env.example
└── README.md
```

### 2.2 Configuration Management

```typescript
// src/config/index.ts
import { z } from 'zod';
import { config as dotenvConfig } from 'dotenv';

// Load environment variables
dotenvConfig();

// Configuration schema
const ConfigSchema = z.object({
  // System identification
  system: z.object({
    id: z.string().uuid(),
    name: z.string().min(1).max(255),
    version: z.string().regex(/^\d+\.\d+\.\d+$/),
    vendor: z.string().optional(),
    description: z.string().optional()
  }),

  // Server configuration
  server: z.object({
    host: z.string().default('0.0.0.0'),
    port: z.number().min(1).max(65535).default(8080),
    grpcPort: z.number().min(1).max(65535).default(9090),
    tlsEnabled: z.boolean().default(true),
    tlsCert: z.string().optional(),
    tlsKey: z.string().optional()
  }),

  // Authentication
  auth: z.object({
    type: z.enum(['jwt', 'mtls', 'apikey']),
    jwt: z.object({
      issuer: z.string().url(),
      audience: z.string(),
      jwksUrl: z.string().url(),
      privateKeyPath: z.string().optional()
    }).optional(),
    mtls: z.object({
      caPath: z.string(),
      certPath: z.string(),
      keyPath: z.string()
    }).optional(),
    apiKey: z.object({
      header: z.string().default('X-API-Key'),
      keys: z.array(z.string())
    }).optional()
  }),

  // Registry connection
  registry: z.object({
    url: z.string().url(),
    refreshInterval: z.number().default(60000),
    timeout: z.number().default(5000)
  }),

  // Logging
  logging: z.object({
    level: z.enum(['debug', 'info', 'warn', 'error']).default('info'),
    format: z.enum(['json', 'text']).default('json'),
    destination: z.enum(['stdout', 'file', 'kafka']).default('stdout'),
    kafkaTopic: z.string().optional()
  }),

  // Metrics
  metrics: z.object({
    enabled: z.boolean().default(true),
    port: z.number().default(9091),
    path: z.string().default('/metrics')
  }),

  // Rate limiting
  rateLimit: z.object({
    enabled: z.boolean().default(true),
    windowMs: z.number().default(60000),
    maxRequests: z.number().default(1000)
  }),

  // Circuit breaker
  circuitBreaker: z.object({
    enabled: z.boolean().default(true),
    threshold: z.number().default(5),
    timeout: z.number().default(30000),
    resetTimeout: z.number().default(60000)
  })
});

type Config = z.infer<typeof ConfigSchema>;

// Load and validate configuration
export function loadConfig(): Config {
  const rawConfig = {
    system: {
      id: process.env.SYSTEM_ID,
      name: process.env.SYSTEM_NAME,
      version: process.env.SYSTEM_VERSION,
      vendor: process.env.SYSTEM_VENDOR,
      description: process.env.SYSTEM_DESCRIPTION
    },
    server: {
      host: process.env.SERVER_HOST,
      port: parseInt(process.env.SERVER_PORT || '8080'),
      grpcPort: parseInt(process.env.GRPC_PORT || '9090'),
      tlsEnabled: process.env.TLS_ENABLED === 'true',
      tlsCert: process.env.TLS_CERT_PATH,
      tlsKey: process.env.TLS_KEY_PATH
    },
    auth: {
      type: process.env.AUTH_TYPE || 'jwt',
      jwt: {
        issuer: process.env.JWT_ISSUER,
        audience: process.env.JWT_AUDIENCE,
        jwksUrl: process.env.JWT_JWKS_URL,
        privateKeyPath: process.env.JWT_PRIVATE_KEY_PATH
      }
    },
    registry: {
      url: process.env.REGISTRY_URL,
      refreshInterval: parseInt(process.env.REGISTRY_REFRESH_INTERVAL || '60000'),
      timeout: parseInt(process.env.REGISTRY_TIMEOUT || '5000')
    },
    logging: {
      level: process.env.LOG_LEVEL || 'info',
      format: process.env.LOG_FORMAT || 'json',
      destination: process.env.LOG_DESTINATION || 'stdout'
    },
    metrics: {
      enabled: process.env.METRICS_ENABLED !== 'false',
      port: parseInt(process.env.METRICS_PORT || '9091'),
      path: process.env.METRICS_PATH || '/metrics'
    },
    rateLimit: {
      enabled: process.env.RATE_LIMIT_ENABLED !== 'false',
      windowMs: parseInt(process.env.RATE_LIMIT_WINDOW_MS || '60000'),
      maxRequests: parseInt(process.env.RATE_LIMIT_MAX_REQUESTS || '1000')
    },
    circuitBreaker: {
      enabled: process.env.CIRCUIT_BREAKER_ENABLED !== 'false',
      threshold: parseInt(process.env.CIRCUIT_BREAKER_THRESHOLD || '5'),
      timeout: parseInt(process.env.CIRCUIT_BREAKER_TIMEOUT || '30000'),
      resetTimeout: parseInt(process.env.CIRCUIT_BREAKER_RESET_TIMEOUT || '60000')
    }
  };

  return ConfigSchema.parse(rawConfig);
}

export const config = loadConfig();
```

### 2.3 Environment Setup

```bash
# .env.example
# System Configuration
SYSTEM_ID=550e8400-e29b-41d4-a716-446655440000
SYSTEM_NAME=My AI Service
SYSTEM_VERSION=1.0.0
SYSTEM_VENDOR=MyCompany

# Server Configuration
SERVER_HOST=0.0.0.0
SERVER_PORT=8080
GRPC_PORT=9090
TLS_ENABLED=true
TLS_CERT_PATH=/etc/certs/server.crt
TLS_KEY_PATH=/etc/certs/server.key

# Authentication
AUTH_TYPE=jwt
JWT_ISSUER=https://auth.wia.org
JWT_AUDIENCE=ai-interop
JWT_JWKS_URL=https://auth.wia.org/.well-known/jwks.json
JWT_PRIVATE_KEY_PATH=/etc/secrets/private.key

# Registry
REGISTRY_URL=https://registry.wia.org/api/v1
REGISTRY_REFRESH_INTERVAL=60000
REGISTRY_TIMEOUT=5000

# Logging
LOG_LEVEL=info
LOG_FORMAT=json
LOG_DESTINATION=stdout

# Metrics
METRICS_ENABLED=true
METRICS_PORT=9091
METRICS_PATH=/metrics

# Rate Limiting
RATE_LIMIT_ENABLED=true
RATE_LIMIT_WINDOW_MS=60000
RATE_LIMIT_MAX_REQUESTS=1000

# Circuit Breaker
CIRCUIT_BREAKER_ENABLED=true
CIRCUIT_BREAKER_THRESHOLD=5
CIRCUIT_BREAKER_TIMEOUT=30000
```

---

## 3. Code Examples

### 3.1 UACP Client Implementation

```typescript
// src/services/uacp/client.ts
import { EventEmitter } from 'events';
import WebSocket from 'ws';
import { v4 as uuidv4 } from 'uuid';
import { SignJWT, importPKCS8 } from 'jose';
import {
  UACPMessage,
  UACPHeader,
  MessageType,
  InferenceRequest,
  InferenceResponse,
  SystemIdentifier
} from '../../models/message';
import { config } from '../../config';
import { logger } from '../../utils/logger';
import { CircuitBreaker } from '../../utils/circuitBreaker';
import { RetryPolicy } from '../../utils/retry';

export interface UACPClientOptions {
  endpoint: string;
  systemId: string;
  auth: AuthConfig;
  timeout?: number;
  retries?: number;
  circuitBreaker?: CircuitBreakerConfig;
}

export class UACPClient extends EventEmitter {
  private ws: WebSocket | null = null;
  private pendingRequests: Map<string, {
    resolve: (value: any) => void;
    reject: (error: Error) => void;
    timeout: NodeJS.Timeout;
  }> = new Map();
  private circuitBreaker: CircuitBreaker;
  private retryPolicy: RetryPolicy;
  private connected: boolean = false;

  constructor(private options: UACPClientOptions) {
    super();
    this.circuitBreaker = new CircuitBreaker(options.circuitBreaker);
    this.retryPolicy = new RetryPolicy({
      maxRetries: options.retries || 3,
      baseDelay: 1000,
      maxDelay: 10000,
      backoffMultiplier: 2
    });
  }

  /**
   * Connect to the UACP endpoint
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(this.options.endpoint, {
          headers: {
            'Authorization': `Bearer ${this.getAuthToken()}`
          }
        });

        this.ws.on('open', () => {
          this.connected = true;
          logger.info('UACP connection established', {
            endpoint: this.options.endpoint
          });
          this.emit('connected');
          resolve();
        });

        this.ws.on('message', (data: Buffer) => {
          this.handleMessage(data);
        });

        this.ws.on('close', (code, reason) => {
          this.connected = false;
          logger.warn('UACP connection closed', { code, reason: reason.toString() });
          this.emit('disconnected', { code, reason });
          this.attemptReconnect();
        });

        this.ws.on('error', (error) => {
          logger.error('UACP connection error', { error: error.message });
          this.emit('error', error);
          reject(error);
        });

      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Send an inference request
   */
  async infer(request: InferenceRequest): Promise<InferenceResponse> {
    return this.circuitBreaker.execute(async () => {
      return this.retryPolicy.execute(async () => {
        const messageId = uuidv4();
        const message = this.createMessage(
          MessageType.INFERENCE_REQUEST,
          request,
          { systemId: request.targetSystem }
        );

        return this.sendAndWait<InferenceResponse>(message);
      });
    });
  }

  /**
   * Send streaming inference request
   */
  async *inferStream(request: InferenceRequest): AsyncGenerator<any> {
    const messageId = uuidv4();
    const message = this.createMessage(
      MessageType.STREAM_START,
      request,
      { systemId: request.targetSystem }
    );

    this.send(message);

    // Create async iterator for stream chunks
    const chunkQueue: any[] = [];
    let streamEnded = false;
    let error: Error | null = null;

    const streamHandler = (msg: UACPMessage) => {
      if (msg.header.correlationId !== messageId) return;

      switch (msg.header.messageType) {
        case MessageType.STREAM_CHUNK:
          chunkQueue.push(msg.body);
          break;
        case MessageType.STREAM_END:
          streamEnded = true;
          break;
        case MessageType.INFERENCE_ERROR:
          error = new Error(msg.body.message);
          break;
      }
    };

    this.on('message', streamHandler);

    try {
      while (!streamEnded && !error) {
        if (chunkQueue.length > 0) {
          yield chunkQueue.shift();
        } else {
          await this.wait(10);
        }
      }

      if (error) throw error;

      // Yield remaining chunks
      while (chunkQueue.length > 0) {
        yield chunkQueue.shift();
      }
    } finally {
      this.off('message', streamHandler);
    }
  }

  /**
   * Create UACP message
   */
  private createMessage(
    type: MessageType,
    body: any,
    destination: SystemIdentifier
  ): UACPMessage {
    const header: UACPHeader = {
      version: '1.0',
      messageId: uuidv4(),
      timestamp: new Date().toISOString(),
      source: {
        systemId: this.options.systemId,
        name: config.system.name,
        version: config.system.version
      },
      destination,
      messageType: type,
      contentType: 'application/json',
      encoding: 'utf-8',
      priority: 1,
      metadata: {}
    };

    return { header, body };
  }

  /**
   * Send message and wait for response
   */
  private async sendAndWait<T>(message: UACPMessage): Promise<T> {
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        this.pendingRequests.delete(message.header.messageId);
        reject(new Error('Request timeout'));
      }, this.options.timeout || 30000);

      this.pendingRequests.set(message.header.messageId, {
        resolve,
        reject,
        timeout
      });

      this.send(message);
    });
  }

  /**
   * Send message
   */
  private send(message: UACPMessage): void {
    if (!this.connected || !this.ws) {
      throw new Error('Not connected to UACP endpoint');
    }

    const data = JSON.stringify(message);
    this.ws.send(data);

    logger.debug('UACP message sent', {
      messageId: message.header.messageId,
      type: message.header.messageType
    });
  }

  /**
   * Handle incoming message
   */
  private handleMessage(data: Buffer): void {
    try {
      const message: UACPMessage = JSON.parse(data.toString());

      logger.debug('UACP message received', {
        messageId: message.header.messageId,
        type: message.header.messageType
      });

      // Check for pending request
      const correlationId = message.header.correlationId;
      if (correlationId && this.pendingRequests.has(correlationId)) {
        const pending = this.pendingRequests.get(correlationId)!;
        clearTimeout(pending.timeout);
        this.pendingRequests.delete(correlationId);

        if (message.header.messageType === MessageType.INFERENCE_ERROR) {
          pending.reject(new Error(message.body.message));
        } else {
          pending.resolve(message.body);
        }
      }

      // Emit for stream handling and other listeners
      this.emit('message', message);

    } catch (error) {
      logger.error('Failed to parse UACP message', { error });
    }
  }

  /**
   * Get authentication token
   */
  private async getAuthToken(): Promise<string> {
    // Implementation depends on auth type
    if (this.options.auth.type === 'jwt') {
      const privateKey = await importPKCS8(
        this.options.auth.jwt!.privateKey,
        'ES256'
      );

      return new SignJWT({
        sub: this.options.systemId,
        iss: this.options.auth.jwt!.issuer
      })
        .setProtectedHeader({ alg: 'ES256' })
        .setAudience(this.options.auth.jwt!.audience)
        .setIssuedAt()
        .setExpirationTime('1h')
        .sign(privateKey);
    }

    return this.options.auth.apiKey || '';
  }

  /**
   * Attempt reconnection
   */
  private async attemptReconnect(): Promise<void> {
    const maxAttempts = 5;
    let attempt = 0;

    while (attempt < maxAttempts && !this.connected) {
      attempt++;
      const delay = Math.min(1000 * Math.pow(2, attempt), 30000);

      logger.info(`Attempting reconnection in ${delay}ms`, { attempt });
      await this.wait(delay);

      try {
        await this.connect();
        logger.info('Reconnection successful');
        return;
      } catch (error) {
        logger.warn('Reconnection failed', { attempt, error });
      }
    }

    logger.error('Max reconnection attempts reached');
    this.emit('reconnectFailed');
  }

  /**
   * Utility wait function
   */
  private wait(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  /**
   * Disconnect from UACP endpoint
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.connected = false;
  }
}
```

### 3.2 UACP Server Implementation

```typescript
// src/services/uacp/server.ts
import express, { Express, Request, Response, NextFunction } from 'express';
import { createServer, Server as HttpServer } from 'https';
import { WebSocketServer, WebSocket } from 'ws';
import { readFileSync } from 'fs';
import { v4 as uuidv4 } from 'uuid';
import {
  UACPMessage,
  MessageType,
  InferenceRequest,
  InferenceResponse
} from '../../models/message';
import { config } from '../../config';
import { logger } from '../../utils/logger';
import { authMiddleware } from '../../middleware/auth';
import { rateLimitMiddleware } from '../../middleware/rateLimit';
import { auditMiddleware } from '../../middleware/audit';
import { metricsMiddleware } from '../../middleware/metrics';

export interface InferenceHandler {
  (request: InferenceRequest): Promise<InferenceResponse>;
}

export interface StreamingInferenceHandler {
  (request: InferenceRequest): AsyncGenerator<any>;
}

export class UACPServer {
  private app: Express;
  private server: HttpServer;
  private wss: WebSocketServer;
  private clients: Map<string, WebSocket> = new Map();
  private inferenceHandler?: InferenceHandler;
  private streamingHandler?: StreamingInferenceHandler;

  constructor() {
    this.app = express();
    this.setupMiddleware();
    this.setupRoutes();
    this.server = this.createHttpServer();
    this.wss = this.createWebSocketServer();
  }

  /**
   * Setup Express middleware
   */
  private setupMiddleware(): void {
    this.app.use(express.json({ limit: '10mb' }));
    this.app.use(metricsMiddleware());
    this.app.use(authMiddleware());
    this.app.use(rateLimitMiddleware());
    this.app.use(auditMiddleware());
  }

  /**
   * Setup HTTP routes
   */
  private setupRoutes(): void {
    // Health check
    this.app.get('/health', (req, res) => {
      res.json({ status: 'healthy', timestamp: new Date().toISOString() });
    });

    // Ready check
    this.app.get('/ready', (req, res) => {
      res.json({ ready: true });
    });

    // REST inference endpoint
    this.app.post('/api/v1/inference', async (req, res, next) => {
      try {
        const request: InferenceRequest = req.body;
        const response = await this.handleInference(request);
        res.json(response);
      } catch (error) {
        next(error);
      }
    });

    // Streaming inference endpoint (SSE)
    this.app.post('/api/v1/inference/stream', async (req, res) => {
      res.setHeader('Content-Type', 'text/event-stream');
      res.setHeader('Cache-Control', 'no-cache');
      res.setHeader('Connection', 'keep-alive');

      try {
        const request: InferenceRequest = req.body;
        const generator = this.handleStreamingInference(request);

        for await (const chunk of generator) {
          res.write(`data: ${JSON.stringify(chunk)}\n\n`);
        }

        res.write('data: [DONE]\n\n');
        res.end();
      } catch (error) {
        res.write(`data: ${JSON.stringify({ error: error.message })}\n\n`);
        res.end();
      }
    });

    // Error handler
    this.app.use((err: Error, req: Request, res: Response, next: NextFunction) => {
      logger.error('Request error', { error: err.message, stack: err.stack });
      res.status(500).json({
        error: 'Internal Server Error',
        message: err.message
      });
    });
  }

  /**
   * Create HTTPS server
   */
  private createHttpServer(): HttpServer {
    if (config.server.tlsEnabled) {
      return createServer({
        cert: readFileSync(config.server.tlsCert!),
        key: readFileSync(config.server.tlsKey!)
      }, this.app);
    }
    return this.app.listen(config.server.port) as unknown as HttpServer;
  }

  /**
   * Create WebSocket server
   */
  private createWebSocketServer(): WebSocketServer {
    const wss = new WebSocketServer({ server: this.server });

    wss.on('connection', (ws: WebSocket, req) => {
      const clientId = uuidv4();
      this.clients.set(clientId, ws);

      logger.info('WebSocket client connected', { clientId });

      ws.on('message', async (data: Buffer) => {
        await this.handleWebSocketMessage(clientId, ws, data);
      });

      ws.on('close', () => {
        this.clients.delete(clientId);
        logger.info('WebSocket client disconnected', { clientId });
      });

      ws.on('error', (error) => {
        logger.error('WebSocket error', { clientId, error: error.message });
      });
    });

    return wss;
  }

  /**
   * Handle WebSocket message
   */
  private async handleWebSocketMessage(
    clientId: string,
    ws: WebSocket,
    data: Buffer
  ): Promise<void> {
    try {
      const message: UACPMessage = JSON.parse(data.toString());

      logger.debug('WebSocket message received', {
        clientId,
        messageId: message.header.messageId,
        type: message.header.messageType
      });

      switch (message.header.messageType) {
        case MessageType.INFERENCE_REQUEST:
          await this.handleInferenceRequest(ws, message);
          break;

        case MessageType.STREAM_START:
          await this.handleStreamRequest(ws, message);
          break;

        case MessageType.HEALTH_CHECK:
          this.sendHealthResponse(ws, message);
          break;

        default:
          logger.warn('Unknown message type', {
            type: message.header.messageType
          });
      }

    } catch (error) {
      logger.error('Failed to handle WebSocket message', {
        clientId,
        error: error.message
      });
    }
  }

  /**
   * Handle inference request via WebSocket
   */
  private async handleInferenceRequest(
    ws: WebSocket,
    message: UACPMessage
  ): Promise<void> {
    const startTime = Date.now();

    try {
      const request: InferenceRequest = message.body;
      const response = await this.handleInference(request);

      const responseMessage: UACPMessage = {
        header: {
          version: '1.0',
          messageId: uuidv4(),
          correlationId: message.header.messageId,
          timestamp: new Date().toISOString(),
          source: {
            systemId: config.system.id,
            name: config.system.name,
            version: config.system.version
          },
          destination: message.header.source,
          messageType: MessageType.INFERENCE_RESPONSE,
          contentType: 'application/json',
          encoding: 'utf-8',
          priority: message.header.priority,
          metadata: {
            processingTime: String(Date.now() - startTime)
          }
        },
        body: response
      };

      ws.send(JSON.stringify(responseMessage));

    } catch (error) {
      this.sendErrorResponse(ws, message, error);
    }
  }

  /**
   * Handle streaming request via WebSocket
   */
  private async handleStreamRequest(
    ws: WebSocket,
    message: UACPMessage
  ): Promise<void> {
    try {
      const request: InferenceRequest = message.body;
      const generator = this.handleStreamingInference(request);

      let chunkIndex = 0;
      for await (const chunk of generator) {
        const chunkMessage: UACPMessage = {
          header: {
            version: '1.0',
            messageId: uuidv4(),
            correlationId: message.header.messageId,
            timestamp: new Date().toISOString(),
            source: {
              systemId: config.system.id
            },
            destination: message.header.source,
            messageType: MessageType.STREAM_CHUNK,
            contentType: 'application/json',
            encoding: 'utf-8',
            priority: message.header.priority,
            metadata: {
              chunkIndex: String(chunkIndex++)
            }
          },
          body: chunk
        };

        ws.send(JSON.stringify(chunkMessage));
      }

      // Send stream end
      const endMessage: UACPMessage = {
        header: {
          version: '1.0',
          messageId: uuidv4(),
          correlationId: message.header.messageId,
          timestamp: new Date().toISOString(),
          source: {
            systemId: config.system.id
          },
          destination: message.header.source,
          messageType: MessageType.STREAM_END,
          contentType: 'application/json',
          encoding: 'utf-8',
          priority: message.header.priority,
          metadata: {
            totalChunks: String(chunkIndex)
          }
        },
        body: { completed: true }
      };

      ws.send(JSON.stringify(endMessage));

    } catch (error) {
      this.sendErrorResponse(ws, message, error);
    }
  }

  /**
   * Send error response
   */
  private sendErrorResponse(
    ws: WebSocket,
    originalMessage: UACPMessage,
    error: Error
  ): void {
    const errorMessage: UACPMessage = {
      header: {
        version: '1.0',
        messageId: uuidv4(),
        correlationId: originalMessage.header.messageId,
        timestamp: new Date().toISOString(),
        source: {
          systemId: config.system.id
        },
        destination: originalMessage.header.source,
        messageType: MessageType.INFERENCE_ERROR,
        contentType: 'application/json',
        encoding: 'utf-8',
        priority: originalMessage.header.priority,
        metadata: {}
      },
      body: {
        code: 'INFERENCE_ERROR',
        message: error.message
      }
    };

    ws.send(JSON.stringify(errorMessage));
  }

  /**
   * Send health response
   */
  private sendHealthResponse(ws: WebSocket, message: UACPMessage): void {
    const healthMessage: UACPMessage = {
      header: {
        version: '1.0',
        messageId: uuidv4(),
        correlationId: message.header.messageId,
        timestamp: new Date().toISOString(),
        source: {
          systemId: config.system.id
        },
        destination: message.header.source,
        messageType: MessageType.HEALTH_RESPONSE,
        contentType: 'application/json',
        encoding: 'utf-8',
        priority: message.header.priority,
        metadata: {}
      },
      body: {
        status: 'healthy',
        uptime: process.uptime(),
        memory: process.memoryUsage()
      }
    };

    ws.send(JSON.stringify(healthMessage));
  }

  /**
   * Handle inference request
   */
  private async handleInference(request: InferenceRequest): Promise<InferenceResponse> {
    if (!this.inferenceHandler) {
      throw new Error('Inference handler not registered');
    }
    return this.inferenceHandler(request);
  }

  /**
   * Handle streaming inference
   */
  private handleStreamingInference(request: InferenceRequest): AsyncGenerator<any> {
    if (!this.streamingHandler) {
      throw new Error('Streaming handler not registered');
    }
    return this.streamingHandler(request);
  }

  /**
   * Register inference handler
   */
  onInference(handler: InferenceHandler): void {
    this.inferenceHandler = handler;
  }

  /**
   * Register streaming handler
   */
  onStreamingInference(handler: StreamingInferenceHandler): void {
    this.streamingHandler = handler;
  }

  /**
   * Start server
   */
  start(): void {
    this.server.listen(config.server.port, config.server.host, () => {
      logger.info('UACP server started', {
        host: config.server.host,
        port: config.server.port
      });
    });
  }

  /**
   * Stop server
   */
  stop(): Promise<void> {
    return new Promise((resolve) => {
      this.wss.close();
      this.server.close(() => {
        logger.info('UACP server stopped');
        resolve();
      });
    });
  }
}
```

### 3.3 Python SDK Implementation

```python
# wia_ai_interop/client.py
import asyncio
import json
import uuid
from datetime import datetime
from typing import Any, AsyncGenerator, Callable, Dict, Optional
from dataclasses import dataclass, asdict
import aiohttp
import websockets
from jose import jwt
from cryptography.hazmat.primitives import serialization

@dataclass
class SystemIdentifier:
    system_id: str
    name: Optional[str] = None
    version: Optional[str] = None
    endpoint: Optional[str] = None

@dataclass
class UACPHeader:
    version: str
    message_id: str
    timestamp: str
    source: SystemIdentifier
    destination: SystemIdentifier
    message_type: str
    content_type: str
    encoding: str
    priority: int
    metadata: Dict[str, str]
    correlation_id: Optional[str] = None
    ttl: Optional[int] = None

@dataclass
class UACPMessage:
    header: UACPHeader
    body: Any
    signature: Optional[str] = None

@dataclass
class InferenceRequest:
    capability: str
    input: Any
    target_system: Optional[str] = None
    capability_version: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    context: Optional[Dict[str, Any]] = None

@dataclass
class InferenceResponse:
    request_id: str
    output: Any
    confidence: Optional[float] = None
    alternatives: Optional[list] = None
    metadata: Optional[Dict[str, Any]] = None


class AIInteropClient:
    """
    WIA AI Interoperability Client for Python

    Example usage:
        async with AIInteropClient(config) as client:
            response = await client.infer(InferenceRequest(
                capability='sentiment-analysis',
                input={'text': 'I love this!'}
            ))
            print(response.output)
    """

    def __init__(
        self,
        system_id: str,
        system_name: str,
        version: str,
        endpoint: str,
        auth_config: Dict[str, Any],
        timeout: int = 30,
        max_retries: int = 3
    ):
        self.system_id = system_id
        self.system_name = system_name
        self.version = version
        self.endpoint = endpoint
        self.auth_config = auth_config
        self.timeout = timeout
        self.max_retries = max_retries

        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._pending_requests: Dict[str, asyncio.Future] = {}
        self._connected = False
        self._session: Optional[aiohttp.ClientSession] = None

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.disconnect()

    async def connect(self) -> None:
        """Establish connection to UACP endpoint"""
        token = await self._get_auth_token()

        headers = {
            'Authorization': f'Bearer {token}'
        }

        self._ws = await websockets.connect(
            self.endpoint,
            extra_headers=headers
        )
        self._connected = True

        # Start message handler
        asyncio.create_task(self._message_handler())

        print(f"Connected to UACP endpoint: {self.endpoint}")

    async def disconnect(self) -> None:
        """Close connection"""
        if self._ws:
            await self._ws.close()
            self._ws = None
        self._connected = False

        if self._session:
            await self._session.close()

    async def infer(self, request: InferenceRequest) -> InferenceResponse:
        """
        Send inference request and wait for response

        Args:
            request: InferenceRequest with capability and input

        Returns:
            InferenceResponse with output and metadata
        """
        message_id = str(uuid.uuid4())

        message = self._create_message(
            message_type='inference-request',
            body=asdict(request),
            destination=SystemIdentifier(system_id=request.target_system or ''),
            message_id=message_id
        )

        # Create future for response
        future: asyncio.Future = asyncio.Future()
        self._pending_requests[message_id] = future

        try:
            # Send message
            await self._send(message)

            # Wait for response with timeout
            response = await asyncio.wait_for(future, timeout=self.timeout)

            return InferenceResponse(
                request_id=message_id,
                output=response.get('output'),
                confidence=response.get('confidence'),
                metadata=response.get('metadata')
            )

        finally:
            self._pending_requests.pop(message_id, None)

    async def infer_stream(
        self,
        request: InferenceRequest
    ) -> AsyncGenerator[Any, None]:
        """
        Send streaming inference request

        Args:
            request: InferenceRequest

        Yields:
            Chunks of the streaming response
        """
        message_id = str(uuid.uuid4())

        message = self._create_message(
            message_type='stream-start',
            body=asdict(request),
            destination=SystemIdentifier(system_id=request.target_system or ''),
            message_id=message_id
        )

        # Create queue for chunks
        chunk_queue: asyncio.Queue = asyncio.Queue()
        stream_ended = asyncio.Event()

        async def chunk_handler(msg: UACPMessage):
            if msg.header.correlation_id != message_id:
                return

            if msg.header.message_type == 'stream-chunk':
                await chunk_queue.put(msg.body)
            elif msg.header.message_type == 'stream-end':
                stream_ended.set()
            elif msg.header.message_type == 'inference-error':
                await chunk_queue.put(Exception(msg.body.get('message')))
                stream_ended.set()

        self._chunk_handlers = getattr(self, '_chunk_handlers', [])
        self._chunk_handlers.append(chunk_handler)

        try:
            await self._send(message)

            while not stream_ended.is_set() or not chunk_queue.empty():
                try:
                    chunk = await asyncio.wait_for(
                        chunk_queue.get(),
                        timeout=1.0
                    )

                    if isinstance(chunk, Exception):
                        raise chunk

                    yield chunk

                except asyncio.TimeoutError:
                    continue

        finally:
            self._chunk_handlers.remove(chunk_handler)

    async def register_capability(
        self,
        capability_type: str,
        name: str,
        version: str,
        input_schema: Dict[str, Any],
        output_schema: Dict[str, Any],
        performance: Optional[Dict[str, Any]] = None,
        pricing: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Register a capability with the registry"""
        if not self._session:
            self._session = aiohttp.ClientSession()

        token = await self._get_auth_token()

        registry_url = self.auth_config.get('registry_url', '')

        async with self._session.post(
            f"{registry_url}/capabilities",
            json={
                'system_id': self.system_id,
                'type': capability_type,
                'name': name,
                'version': version,
                'input_schema': input_schema,
                'output_schema': output_schema,
                'performance': performance or {},
                'pricing': pricing or {}
            },
            headers={'Authorization': f'Bearer {token}'}
        ) as response:
            return await response.json()

    def _create_message(
        self,
        message_type: str,
        body: Any,
        destination: SystemIdentifier,
        message_id: Optional[str] = None,
        correlation_id: Optional[str] = None
    ) -> UACPMessage:
        """Create UACP message"""
        header = UACPHeader(
            version='1.0',
            message_id=message_id or str(uuid.uuid4()),
            correlation_id=correlation_id,
            timestamp=datetime.utcnow().isoformat() + 'Z',
            source=SystemIdentifier(
                system_id=self.system_id,
                name=self.system_name,
                version=self.version
            ),
            destination=destination,
            message_type=message_type,
            content_type='application/json',
            encoding='utf-8',
            priority=1,
            metadata={}
        )

        return UACPMessage(header=header, body=body)

    async def _send(self, message: UACPMessage) -> None:
        """Send message via WebSocket"""
        if not self._connected or not self._ws:
            raise ConnectionError("Not connected to UACP endpoint")

        data = json.dumps(asdict(message), default=str)
        await self._ws.send(data)

    async def _message_handler(self) -> None:
        """Handle incoming messages"""
        try:
            async for data in self._ws:
                message_dict = json.loads(data)

                # Reconstruct message
                header = UACPHeader(**message_dict['header'])
                message = UACPMessage(
                    header=header,
                    body=message_dict['body'],
                    signature=message_dict.get('signature')
                )

                # Handle response for pending request
                correlation_id = message.header.correlation_id
                if correlation_id and correlation_id in self._pending_requests:
                    future = self._pending_requests[correlation_id]

                    if message.header.message_type == 'inference-error':
                        future.set_exception(
                            Exception(message.body.get('message', 'Unknown error'))
                        )
                    else:
                        future.set_result(message.body)

                # Handle streaming chunks
                for handler in getattr(self, '_chunk_handlers', []):
                    await handler(message)

        except websockets.ConnectionClosed:
            self._connected = False

    async def _get_auth_token(self) -> str:
        """Get authentication token"""
        auth_type = self.auth_config.get('type', 'jwt')

        if auth_type == 'jwt':
            # Load private key
            private_key_pem = self.auth_config.get('private_key')

            payload = {
                'sub': self.system_id,
                'iss': self.auth_config.get('issuer'),
                'aud': self.auth_config.get('audience'),
                'iat': datetime.utcnow(),
                'exp': datetime.utcnow().timestamp() + 3600
            }

            return jwt.encode(
                payload,
                private_key_pem,
                algorithm='ES256'
            )

        elif auth_type == 'api_key':
            return self.auth_config.get('api_key', '')

        raise ValueError(f"Unknown auth type: {auth_type}")


# Example usage
async def main():
    config = {
        'type': 'jwt',
        'issuer': 'https://auth.wia.org',
        'audience': 'ai-interop',
        'private_key': '...',  # Your private key
        'registry_url': 'https://registry.wia.org/api/v1'
    }

    async with AIInteropClient(
        system_id='my-system-id',
        system_name='My AI System',
        version='1.0.0',
        endpoint='wss://gateway.wia.org/uacp',
        auth_config=config
    ) as client:

        # Register capability
        await client.register_capability(
            capability_type='text-classification',
            name='sentiment-analysis',
            version='1.0.0',
            input_schema={
                'type': 'object',
                'properties': {
                    'text': {'type': 'string'}
                },
                'required': ['text']
            },
            output_schema={
                'type': 'object',
                'properties': {
                    'sentiment': {'type': 'string'},
                    'confidence': {'type': 'number'}
                }
            }
        )

        # Make inference request
        response = await client.infer(InferenceRequest(
            capability='sentiment-analysis',
            input={'text': 'I love this product!'},
            target_system='nlp-service'
        ))

        print(f"Sentiment: {response.output}")

        # Streaming inference
        async for chunk in client.infer_stream(InferenceRequest(
            capability='text-generation',
            input={'prompt': 'Once upon a time'},
            target_system='llm-service'
        )):
            print(chunk, end='', flush=True)


if __name__ == '__main__':
    asyncio.run(main())
```

### 3.4 Rust SDK Implementation

```rust
// src/lib.rs
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{mpsc, Mutex, RwLock};
use tokio_tungstenite::{connect_async, tungstenite::Message};
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};
use jsonwebtoken::{encode, EncodingKey, Header, Algorithm};

/// System identifier for UACP messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemIdentifier {
    pub system_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub endpoint: Option<String>,
}

/// UACP message header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UACPHeader {
    pub version: String,
    pub message_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
    pub timestamp: String,
    pub source: SystemIdentifier,
    pub destination: SystemIdentifier,
    pub message_type: String,
    pub content_type: String,
    pub encoding: String,
    pub priority: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ttl: Option<u32>,
    pub metadata: HashMap<String, String>,
}

/// UACP message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UACPMessage {
    pub header: UACPHeader,
    pub body: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<String>,
}

/// Inference request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InferenceRequest {
    pub capability: String,
    pub input: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_system: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capability_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<InferenceParameters>,
}

/// Inference parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InferenceParameters {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_latency: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_tokens: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature: Option<f64>,
}

/// Inference response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InferenceResponse {
    pub request_id: String,
    pub output: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<ResponseMetadata>,
}

/// Response metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponseMetadata {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_version: Option<String>,
    pub processing_time: u64,
}

/// Client configuration
#[derive(Debug, Clone)]
pub struct ClientConfig {
    pub system_id: String,
    pub system_name: String,
    pub version: String,
    pub endpoint: String,
    pub auth: AuthConfig,
    pub timeout_ms: u64,
    pub max_retries: u32,
}

/// Authentication configuration
#[derive(Debug, Clone)]
pub enum AuthConfig {
    Jwt {
        issuer: String,
        audience: String,
        private_key: String,
    },
    ApiKey {
        key: String,
    },
}

/// AI Interop Client error
#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    #[error("Connection error: {0}")]
    Connection(String),
    #[error("Authentication error: {0}")]
    Auth(String),
    #[error("Request timeout")]
    Timeout,
    #[error("Inference error: {0}")]
    Inference(String),
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),
}

type PendingRequest = tokio::sync::oneshot::Sender<Result<serde_json::Value, ClientError>>;

/// AI Interop Client
pub struct AIInteropClient {
    config: ClientConfig,
    pending_requests: Arc<RwLock<HashMap<String, PendingRequest>>>,
    sender: Arc<Mutex<Option<mpsc::Sender<Message>>>>,
    connected: Arc<RwLock<bool>>,
}

impl AIInteropClient {
    /// Create new client
    pub fn new(config: ClientConfig) -> Self {
        Self {
            config,
            pending_requests: Arc::new(RwLock::new(HashMap::new())),
            sender: Arc::new(Mutex::new(None)),
            connected: Arc::new(RwLock::new(false)),
        }
    }

    /// Connect to UACP endpoint
    pub async fn connect(&self) -> Result<(), ClientError> {
        let token = self.get_auth_token().await?;

        let request = http::Request::builder()
            .uri(&self.config.endpoint)
            .header("Authorization", format!("Bearer {}", token))
            .body(())
            .map_err(|e| ClientError::Connection(e.to_string()))?;

        let (ws_stream, _) = connect_async(request)
            .await
            .map_err(|e| ClientError::Connection(e.to_string()))?;

        let (mut write, mut read) = ws_stream.split();

        // Create channel for sending messages
        let (tx, mut rx) = mpsc::channel::<Message>(100);
        *self.sender.lock().await = Some(tx);
        *self.connected.write().await = true;

        // Spawn write task
        tokio::spawn(async move {
            while let Some(msg) = rx.recv().await {
                if write.send(msg).await.is_err() {
                    break;
                }
            }
        });

        // Spawn read task
        let pending = self.pending_requests.clone();
        tokio::spawn(async move {
            while let Some(Ok(msg)) = read.next().await {
                if let Message::Text(text) = msg {
                    if let Ok(message) = serde_json::from_str::<UACPMessage>(&text) {
                        if let Some(correlation_id) = &message.header.correlation_id {
                            if let Some(sender) = pending.write().await.remove(correlation_id) {
                                let result = if message.header.message_type == "inference-error" {
                                    Err(ClientError::Inference(
                                        message.body.get("message")
                                            .and_then(|v| v.as_str())
                                            .unwrap_or("Unknown error")
                                            .to_string()
                                    ))
                                } else {
                                    Ok(message.body)
                                };
                                let _ = sender.send(result);
                            }
                        }
                    }
                }
            }
        });

        Ok(())
    }

    /// Send inference request
    pub async fn infer(&self, request: InferenceRequest) -> Result<InferenceResponse, ClientError> {
        let message_id = Uuid::new_v4().to_string();

        let message = self.create_message(
            "inference-request",
            serde_json::to_value(&request)?,
            SystemIdentifier {
                system_id: request.target_system.clone().unwrap_or_default(),
                name: None,
                version: None,
                endpoint: None,
            },
            message_id.clone(),
        );

        // Create response channel
        let (tx, rx) = tokio::sync::oneshot::channel();
        self.pending_requests.write().await.insert(message_id.clone(), tx);

        // Send message
        self.send_message(message).await?;

        // Wait for response with timeout
        let result = tokio::time::timeout(
            std::time::Duration::from_millis(self.config.timeout_ms),
            rx
        ).await
            .map_err(|_| ClientError::Timeout)?
            .map_err(|_| ClientError::Connection("Channel closed".to_string()))??;

        Ok(InferenceResponse {
            request_id: message_id,
            output: result.get("output").cloned().unwrap_or(serde_json::Value::Null),
            confidence: result.get("confidence").and_then(|v| v.as_f64()),
            metadata: result.get("metadata").and_then(|v| serde_json::from_value(v.clone()).ok()),
        })
    }

    /// Create UACP message
    fn create_message(
        &self,
        message_type: &str,
        body: serde_json::Value,
        destination: SystemIdentifier,
        message_id: String,
    ) -> UACPMessage {
        UACPMessage {
            header: UACPHeader {
                version: "1.0".to_string(),
                message_id,
                correlation_id: None,
                timestamp: Utc::now().to_rfc3339(),
                source: SystemIdentifier {
                    system_id: self.config.system_id.clone(),
                    name: Some(self.config.system_name.clone()),
                    version: Some(self.config.version.clone()),
                    endpoint: None,
                },
                destination,
                message_type: message_type.to_string(),
                content_type: "application/json".to_string(),
                encoding: "utf-8".to_string(),
                priority: 1,
                ttl: None,
                metadata: HashMap::new(),
            },
            body,
            signature: None,
        }
    }

    /// Send message
    async fn send_message(&self, message: UACPMessage) -> Result<(), ClientError> {
        let sender = self.sender.lock().await;

        if let Some(tx) = sender.as_ref() {
            let json = serde_json::to_string(&message)?;
            tx.send(Message::Text(json))
                .await
                .map_err(|e| ClientError::Connection(e.to_string()))?;
            Ok(())
        } else {
            Err(ClientError::Connection("Not connected".to_string()))
        }
    }

    /// Get authentication token
    async fn get_auth_token(&self) -> Result<String, ClientError> {
        match &self.config.auth {
            AuthConfig::Jwt { issuer, audience, private_key } => {
                #[derive(Serialize)]
                struct Claims {
                    sub: String,
                    iss: String,
                    aud: String,
                    iat: i64,
                    exp: i64,
                }

                let now = Utc::now().timestamp();
                let claims = Claims {
                    sub: self.config.system_id.clone(),
                    iss: issuer.clone(),
                    aud: audience.clone(),
                    iat: now,
                    exp: now + 3600,
                };

                let key = EncodingKey::from_ec_pem(private_key.as_bytes())
                    .map_err(|e| ClientError::Auth(e.to_string()))?;

                encode(&Header::new(Algorithm::ES256), &claims, &key)
                    .map_err(|e| ClientError::Auth(e.to_string()))
            }
            AuthConfig::ApiKey { key } => Ok(key.clone()),
        }
    }

    /// Disconnect
    pub async fn disconnect(&self) {
        *self.connected.write().await = false;
        *self.sender.lock().await = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_create_message() {
        let config = ClientConfig {
            system_id: "test-system".to_string(),
            system_name: "Test System".to_string(),
            version: "1.0.0".to_string(),
            endpoint: "wss://localhost:8080/uacp".to_string(),
            auth: AuthConfig::ApiKey { key: "test-key".to_string() },
            timeout_ms: 30000,
            max_retries: 3,
        };

        let client = AIInteropClient::new(config);

        let message = client.create_message(
            "inference-request",
            serde_json::json!({"text": "test"}),
            SystemIdentifier {
                system_id: "target".to_string(),
                name: None,
                version: None,
                endpoint: None,
            },
            "test-id".to_string(),
        );

        assert_eq!(message.header.message_type, "inference-request");
        assert_eq!(message.header.source.system_id, "test-system");
    }
}
```

---

## 4. Best Practices

### 4.1 Design Patterns

#### 4.1.1 Circuit Breaker Pattern

```typescript
// src/utils/circuitBreaker.ts
export enum CircuitState {
  CLOSED = 'closed',
  OPEN = 'open',
  HALF_OPEN = 'half-open'
}

export interface CircuitBreakerConfig {
  failureThreshold: number;
  successThreshold: number;
  timeout: number;
  resetTimeout: number;
}

export class CircuitBreaker {
  private state: CircuitState = CircuitState.CLOSED;
  private failureCount: number = 0;
  private successCount: number = 0;
  private lastFailureTime: number = 0;
  private config: CircuitBreakerConfig;

  constructor(config: Partial<CircuitBreakerConfig> = {}) {
    this.config = {
      failureThreshold: config.failureThreshold || 5,
      successThreshold: config.successThreshold || 3,
      timeout: config.timeout || 30000,
      resetTimeout: config.resetTimeout || 60000
    };
  }

  async execute<T>(fn: () => Promise<T>): Promise<T> {
    if (this.state === CircuitState.OPEN) {
      if (Date.now() - this.lastFailureTime >= this.config.resetTimeout) {
        this.state = CircuitState.HALF_OPEN;
        this.successCount = 0;
      } else {
        throw new Error('Circuit breaker is open');
      }
    }

    try {
      const result = await Promise.race([
        fn(),
        this.createTimeout()
      ]);

      this.onSuccess();
      return result as T;

    } catch (error) {
      this.onFailure();
      throw error;
    }
  }

  private onSuccess(): void {
    this.failureCount = 0;

    if (this.state === CircuitState.HALF_OPEN) {
      this.successCount++;
      if (this.successCount >= this.config.successThreshold) {
        this.state = CircuitState.CLOSED;
      }
    }
  }

  private onFailure(): void {
    this.failureCount++;
    this.lastFailureTime = Date.now();

    if (this.failureCount >= this.config.failureThreshold) {
      this.state = CircuitState.OPEN;
    }
  }

  private createTimeout(): Promise<never> {
    return new Promise((_, reject) => {
      setTimeout(() => reject(new Error('Timeout')), this.config.timeout);
    });
  }

  getState(): CircuitState {
    return this.state;
  }
}
```

#### 4.1.2 Retry Pattern with Exponential Backoff

```typescript
// src/utils/retry.ts
export interface RetryConfig {
  maxRetries: number;
  baseDelay: number;
  maxDelay: number;
  backoffMultiplier: number;
  retryableErrors?: string[];
}

export class RetryPolicy {
  private config: RetryConfig;

  constructor(config: Partial<RetryConfig> = {}) {
    this.config = {
      maxRetries: config.maxRetries || 3,
      baseDelay: config.baseDelay || 1000,
      maxDelay: config.maxDelay || 30000,
      backoffMultiplier: config.backoffMultiplier || 2,
      retryableErrors: config.retryableErrors || [
        'ECONNRESET',
        'ETIMEDOUT',
        'ECONNREFUSED',
        'SERVICE_UNAVAILABLE'
      ]
    };
  }

  async execute<T>(fn: () => Promise<T>): Promise<T> {
    let lastError: Error;

    for (let attempt = 0; attempt <= this.config.maxRetries; attempt++) {
      try {
        return await fn();
      } catch (error) {
        lastError = error as Error;

        if (!this.isRetryable(error) || attempt === this.config.maxRetries) {
          throw error;
        }

        const delay = this.calculateDelay(attempt);
        await this.sleep(delay);
      }
    }

    throw lastError!;
  }

  private isRetryable(error: any): boolean {
    if (this.config.retryableErrors?.length === 0) {
      return true;
    }

    const errorCode = error.code || error.message;
    return this.config.retryableErrors!.some(
      code => errorCode.includes(code)
    );
  }

  private calculateDelay(attempt: number): number {
    const delay = this.config.baseDelay *
      Math.pow(this.config.backoffMultiplier, attempt);

    // Add jitter (±10%)
    const jitter = delay * 0.1 * (Math.random() * 2 - 1);

    return Math.min(delay + jitter, this.config.maxDelay);
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}
```

### 4.2 Error Handling

```typescript
// src/utils/errors.ts
export enum ErrorCode {
  // Connection errors
  CONNECTION_FAILED = 'CONNECTION_FAILED',
  CONNECTION_TIMEOUT = 'CONNECTION_TIMEOUT',
  CONNECTION_CLOSED = 'CONNECTION_CLOSED',

  // Authentication errors
  AUTH_FAILED = 'AUTH_FAILED',
  AUTH_EXPIRED = 'AUTH_EXPIRED',
  AUTH_INVALID = 'AUTH_INVALID',

  // Request errors
  INVALID_REQUEST = 'INVALID_REQUEST',
  VALIDATION_FAILED = 'VALIDATION_FAILED',
  RATE_LIMITED = 'RATE_LIMITED',

  // Inference errors
  INFERENCE_FAILED = 'INFERENCE_FAILED',
  CAPABILITY_NOT_FOUND = 'CAPABILITY_NOT_FOUND',
  MODEL_ERROR = 'MODEL_ERROR',

  // System errors
  INTERNAL_ERROR = 'INTERNAL_ERROR',
  SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE',
  CIRCUIT_OPEN = 'CIRCUIT_OPEN'
}

export class AIInteropError extends Error {
  constructor(
    public code: ErrorCode,
    message: string,
    public details?: Record<string, any>,
    public cause?: Error
  ) {
    super(message);
    this.name = 'AIInteropError';
    Error.captureStackTrace(this, this.constructor);
  }

  toJSON(): Record<string, any> {
    return {
      name: this.name,
      code: this.code,
      message: this.message,
      details: this.details,
      stack: this.stack
    };
  }
}

// Error handler middleware
export function errorHandler(error: Error): AIInteropError {
  if (error instanceof AIInteropError) {
    return error;
  }

  // Map common errors
  if (error.message.includes('ECONNREFUSED')) {
    return new AIInteropError(
      ErrorCode.CONNECTION_FAILED,
      'Failed to connect to UACP endpoint',
      { originalError: error.message },
      error
    );
  }

  if (error.message.includes('timeout')) {
    return new AIInteropError(
      ErrorCode.CONNECTION_TIMEOUT,
      'Connection timed out',
      { originalError: error.message },
      error
    );
  }

  return new AIInteropError(
    ErrorCode.INTERNAL_ERROR,
    error.message,
    { originalError: error.message },
    error
  );
}
```

### 4.3 Logging Best Practices

```typescript
// src/utils/logger.ts
import pino from 'pino';
import { config } from '../config';

const transport = config.logging.format === 'text'
  ? pino.transport({
      target: 'pino-pretty',
      options: {
        colorize: true,
        translateTime: 'SYS:standard'
      }
    })
  : undefined;

export const logger = pino({
  level: config.logging.level,
  base: {
    service: config.system.name,
    version: config.system.version
  },
  timestamp: pino.stdTimeFunctions.isoTime,
  formatters: {
    level: (label) => ({ level: label })
  },
  redact: {
    paths: [
      'body.password',
      'body.secret',
      'body.token',
      'header.Authorization',
      '*.apiKey'
    ],
    censor: '[REDACTED]'
  }
}, transport);

// Structured logging helpers
export function logInferenceRequest(
  requestId: string,
  capability: string,
  targetSystem: string
): void {
  logger.info({
    event: 'inference_request',
    requestId,
    capability,
    targetSystem,
    timestamp: new Date().toISOString()
  }, 'Inference request initiated');
}

export function logInferenceResponse(
  requestId: string,
  latency: number,
  success: boolean,
  error?: string
): void {
  const logFn = success ? logger.info : logger.error;

  logFn({
    event: 'inference_response',
    requestId,
    latency,
    success,
    error,
    timestamp: new Date().toISOString()
  }, success ? 'Inference completed' : 'Inference failed');
}

export function logAuditEvent(
  action: string,
  sourceSystem: string,
  targetSystem: string,
  details: Record<string, any>
): void {
  logger.info({
    event: 'audit',
    action,
    sourceSystem,
    targetSystem,
    details,
    timestamp: new Date().toISOString()
  }, `Audit: ${action}`);
}
```

---

## 5. Testing Strategy

### 5.1 Unit Testing

```typescript
// tests/unit/uacp-client.test.ts
import { describe, it, expect, beforeEach, afterEach, jest } from '@jest/globals';
import { UACPClient } from '../../src/services/uacp/client';
import { MockWebSocket } from '../mocks/websocket';

jest.mock('ws');

describe('UACPClient', () => {
  let client: UACPClient;
  let mockWs: MockWebSocket;

  beforeEach(() => {
    mockWs = new MockWebSocket();
    (global as any).WebSocket = jest.fn(() => mockWs);

    client = new UACPClient({
      endpoint: 'wss://test.example.com/uacp',
      systemId: 'test-system',
      auth: {
        type: 'jwt',
        credentials: {
          issuer: 'https://auth.example.com',
          audience: 'test',
          privateKey: 'test-key'
        }
      }
    });
  });

  afterEach(() => {
    client.disconnect();
  });

  describe('connect', () => {
    it('should establish WebSocket connection', async () => {
      const connectPromise = client.connect();
      mockWs.simulateOpen();

      await expect(connectPromise).resolves.toBeUndefined();
    });

    it('should handle connection errors', async () => {
      const connectPromise = client.connect();
      mockWs.simulateError(new Error('Connection refused'));

      await expect(connectPromise).rejects.toThrow('Connection refused');
    });
  });

  describe('infer', () => {
    beforeEach(async () => {
      const connectPromise = client.connect();
      mockWs.simulateOpen();
      await connectPromise;
    });

    it('should send inference request and receive response', async () => {
      const inferPromise = client.infer({
        capability: 'sentiment-analysis',
        input: { text: 'Hello world' },
        targetSystem: 'nlp-service'
      });

      // Simulate response
      const sentMessage = JSON.parse(mockWs.lastSentMessage!);
      mockWs.simulateMessage(JSON.stringify({
        header: {
          version: '1.0',
          messageId: 'response-id',
          correlationId: sentMessage.header.messageId,
          timestamp: new Date().toISOString(),
          source: { systemId: 'nlp-service' },
          destination: { systemId: 'test-system' },
          messageType: 'inference-response',
          contentType: 'application/json',
          encoding: 'utf-8',
          priority: 1,
          metadata: {}
        },
        body: {
          output: { sentiment: 'positive', confidence: 0.95 }
        }
      }));

      const response = await inferPromise;

      expect(response.output).toEqual({
        sentiment: 'positive',
        confidence: 0.95
      });
    });

    it('should handle inference errors', async () => {
      const inferPromise = client.infer({
        capability: 'unknown',
        input: {},
        targetSystem: 'service'
      });

      const sentMessage = JSON.parse(mockWs.lastSentMessage!);
      mockWs.simulateMessage(JSON.stringify({
        header: {
          version: '1.0',
          messageId: 'error-id',
          correlationId: sentMessage.header.messageId,
          timestamp: new Date().toISOString(),
          source: { systemId: 'service' },
          destination: { systemId: 'test-system' },
          messageType: 'inference-error',
          contentType: 'application/json',
          encoding: 'utf-8',
          priority: 1,
          metadata: {}
        },
        body: {
          code: 'CAPABILITY_NOT_FOUND',
          message: 'Capability not found'
        }
      }));

      await expect(inferPromise).rejects.toThrow('Capability not found');
    });

    it('should timeout on slow responses', async () => {
      jest.useFakeTimers();

      const inferPromise = client.infer({
        capability: 'slow-service',
        input: {},
        targetSystem: 'service'
      });

      jest.advanceTimersByTime(35000); // Exceed 30s timeout

      await expect(inferPromise).rejects.toThrow('Request timeout');

      jest.useRealTimers();
    });
  });
});
```

### 5.2 Integration Testing

```typescript
// tests/integration/ai-interop.test.ts
import { describe, it, expect, beforeAll, afterAll } from '@jest/globals';
import { GenericContainer, StartedTestContainer } from 'testcontainers';
import { UACPClient } from '../../src/services/uacp/client';
import { UACPServer } from '../../src/services/uacp/server';

describe('AI Interop Integration', () => {
  let server: UACPServer;
  let client: UACPClient;
  let redisContainer: StartedTestContainer;

  beforeAll(async () => {
    // Start Redis container
    redisContainer = await new GenericContainer('redis:7')
      .withExposedPorts(6379)
      .start();

    // Start server
    server = new UACPServer();
    server.onInference(async (request) => {
      // Mock inference handler
      if (request.capability === 'sentiment-analysis') {
        return {
          output: { sentiment: 'positive', confidence: 0.9 },
          metadata: { processingTime: 50 }
        };
      }
      throw new Error('Unknown capability');
    });
    server.start();

    // Create client
    client = new UACPClient({
      endpoint: 'ws://localhost:8080/ws/uacp',
      systemId: 'test-client',
      auth: { type: 'apikey', apiKey: 'test-key' }
    });

    await client.connect();
  }, 60000);

  afterAll(async () => {
    client.disconnect();
    await server.stop();
    await redisContainer.stop();
  });

  it('should complete end-to-end inference', async () => {
    const response = await client.infer({
      capability: 'sentiment-analysis',
      input: { text: 'I love this!' }
    });

    expect(response.output.sentiment).toBe('positive');
    expect(response.output.confidence).toBeGreaterThan(0.8);
  });

  it('should handle multiple concurrent requests', async () => {
    const requests = Array(10).fill(null).map((_, i) =>
      client.infer({
        capability: 'sentiment-analysis',
        input: { text: `Test message ${i}` }
      })
    );

    const responses = await Promise.all(requests);

    expect(responses).toHaveLength(10);
    responses.forEach(r => {
      expect(r.output.sentiment).toBeDefined();
    });
  });
});
```

### 5.3 Load Testing

```typescript
// tests/load/inference-load.test.ts
import autocannon from 'autocannon';

async function runLoadTest() {
  const result = await autocannon({
    url: 'https://localhost:8443/api/v1/inference',
    connections: 100,
    duration: 60,
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Bearer test-token'
    },
    body: JSON.stringify({
      capability: 'sentiment-analysis',
      input: { text: 'Load test message' }
    }),
    setupClient: (client) => {
      client.on('response', (statusCode, resBytes, responseTime) => {
        if (statusCode !== 200) {
          console.error(`Error: ${statusCode}`);
        }
      });
    }
  });

  console.log('Load Test Results:');
  console.log(`  Requests/sec: ${result.requests.average}`);
  console.log(`  Latency (avg): ${result.latency.average}ms`);
  console.log(`  Latency (p99): ${result.latency.p99}ms`);
  console.log(`  Throughput: ${result.throughput.average} bytes/sec`);
  console.log(`  Errors: ${result.errors}`);
  console.log(`  Timeouts: ${result.timeouts}`);

  // Assertions
  if (result.requests.average < 500) {
    throw new Error('Request rate below threshold');
  }
  if (result.latency.p99 > 500) {
    throw new Error('P99 latency above threshold');
  }
}

runLoadTest().catch(console.error);
```

---

## 6. Deployment Guide

### 6.1 Docker Deployment

```dockerfile
# Dockerfile
FROM node:20-alpine AS builder

WORKDIR /app

COPY package*.json ./
RUN npm ci --only=production

COPY tsconfig.json ./
COPY src ./src
RUN npm run build

FROM node:20-alpine AS runner

WORKDIR /app

# Create non-root user
RUN addgroup -g 1001 -S nodejs && \
    adduser -S nodejs -u 1001

COPY --from=builder /app/dist ./dist
COPY --from=builder /app/node_modules ./node_modules
COPY package.json ./

# Set environment
ENV NODE_ENV=production
ENV PORT=8080

# Health check
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
  CMD wget --no-verbose --tries=1 --spider http://localhost:8080/health || exit 1

USER nodejs

EXPOSE 8080 9090 9091

CMD ["node", "dist/index.js"]
```

### 6.2 Kubernetes Deployment

```yaml
# k8s/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: ai-interop-service
  labels:
    app: ai-interop-service
spec:
  replicas: 3
  selector:
    matchLabels:
      app: ai-interop-service
  template:
    metadata:
      labels:
        app: ai-interop-service
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "9091"
    spec:
      serviceAccountName: ai-interop-service
      securityContext:
        runAsNonRoot: true
        runAsUser: 1001
      containers:
        - name: ai-interop-service
          image: wia/ai-interop-service:1.0.0
          ports:
            - name: http
              containerPort: 8080
            - name: grpc
              containerPort: 9090
            - name: metrics
              containerPort: 9091
          env:
            - name: SYSTEM_ID
              valueFrom:
                fieldRef:
                  fieldPath: metadata.name
            - name: REGISTRY_URL
              valueFrom:
                configMapKeyRef:
                  name: ai-interop-config
                  key: registry.url
            - name: JWT_PRIVATE_KEY
              valueFrom:
                secretKeyRef:
                  name: ai-interop-secrets
                  key: jwt-private-key
          resources:
            requests:
              memory: "256Mi"
              cpu: "250m"
            limits:
              memory: "1Gi"
              cpu: "1000m"
          livenessProbe:
            httpGet:
              path: /health
              port: 8080
            initialDelaySeconds: 15
            periodSeconds: 20
          readinessProbe:
            httpGet:
              path: /ready
              port: 8080
            initialDelaySeconds: 5
            periodSeconds: 10
          volumeMounts:
            - name: config
              mountPath: /app/config
            - name: certs
              mountPath: /app/certs
              readOnly: true
      volumes:
        - name: config
          configMap:
            name: ai-interop-config
        - name: certs
          secret:
            secretName: ai-interop-tls

---
apiVersion: v1
kind: Service
metadata:
  name: ai-interop-service
spec:
  selector:
    app: ai-interop-service
  ports:
    - name: http
      port: 80
      targetPort: 8080
    - name: grpc
      port: 9090
      targetPort: 9090
```

---

## 7. Troubleshooting

### 7.1 Common Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| Connection timeout | `ETIMEDOUT` errors | Check network connectivity, firewall rules |
| Authentication failure | 401 responses | Verify JWT token, check key configuration |
| Rate limiting | 429 responses | Reduce request rate, request limit increase |
| Circuit breaker open | Immediate failures | Wait for reset, check downstream service |
| Semantic mismatch | Incorrect outputs | Verify schema mappings, check versions |

### 7.2 Debug Mode

```typescript
// Enable debug logging
process.env.DEBUG = 'wia:*';
process.env.LOG_LEVEL = 'debug';

// Enable request tracing
const client = new UACPClient({
  // ... config
  debug: {
    enabled: true,
    logRequests: true,
    logResponses: true,
    logHeaders: true
  }
});
```

### 7.3 Health Check Script

```bash
#!/bin/bash
# scripts/health-check.sh

ENDPOINT="${UACP_ENDPOINT:-http://localhost:8080}"

echo "Checking AI Interop service health..."

# Health endpoint
HEALTH=$(curl -s -o /dev/null -w "%{http_code}" "$ENDPOINT/health")
if [ "$HEALTH" != "200" ]; then
  echo "❌ Health check failed: $HEALTH"
  exit 1
fi
echo "✅ Health check passed"

# Ready endpoint
READY=$(curl -s -o /dev/null -w "%{http_code}" "$ENDPOINT/ready")
if [ "$READY" != "200" ]; then
  echo "❌ Readiness check failed: $READY"
  exit 1
fi
echo "✅ Readiness check passed"

# Registry connection
REGISTRY=$(curl -s "$ENDPOINT/health" | jq -r '.registry')
if [ "$REGISTRY" != "connected" ]; then
  echo "⚠️  Registry not connected"
fi

echo "All checks passed!"
```

---

## 8. Performance Optimization

### 8.1 Connection Pooling

```typescript
// src/utils/connectionPool.ts
import { Pool } from 'generic-pool';

export class ConnectionPool<T> {
  private pool: Pool<T>;

  constructor(
    private factory: {
      create: () => Promise<T>;
      destroy: (conn: T) => Promise<void>;
      validate?: (conn: T) => Promise<boolean>;
    },
    private options: {
      min: number;
      max: number;
      acquireTimeoutMs: number;
      idleTimeoutMs: number;
    }
  ) {
    this.pool = createPool({
      create: factory.create,
      destroy: factory.destroy,
      validate: factory.validate
    }, {
      min: options.min,
      max: options.max,
      acquireTimeoutMillis: options.acquireTimeoutMs,
      idleTimeoutMillis: options.idleTimeoutMs,
      testOnBorrow: true
    });
  }

  async acquire(): Promise<T> {
    return this.pool.acquire();
  }

  async release(conn: T): Promise<void> {
    return this.pool.release(conn);
  }

  async drain(): Promise<void> {
    await this.pool.drain();
    await this.pool.clear();
  }
}
```

### 8.2 Caching Strategy

```typescript
// src/utils/cache.ts
import Redis from 'ioredis';

export class InferenceCache {
  private redis: Redis;
  private ttl: number;

  constructor(redisUrl: string, ttlSeconds: number = 300) {
    this.redis = new Redis(redisUrl);
    this.ttl = ttlSeconds;
  }

  private generateKey(request: InferenceRequest): string {
    const hash = crypto
      .createHash('sha256')
      .update(JSON.stringify({
        capability: request.capability,
        input: request.input,
        parameters: request.parameters
      }))
      .digest('hex');

    return `inference:${request.capability}:${hash}`;
  }

  async get(request: InferenceRequest): Promise<InferenceResponse | null> {
    const key = this.generateKey(request);
    const cached = await this.redis.get(key);

    if (cached) {
      return JSON.parse(cached);
    }

    return null;
  }

  async set(
    request: InferenceRequest,
    response: InferenceResponse
  ): Promise<void> {
    const key = this.generateKey(request);
    await this.redis.setex(key, this.ttl, JSON.stringify(response));
  }

  async invalidate(capability: string): Promise<void> {
    const keys = await this.redis.keys(`inference:${capability}:*`);
    if (keys.length > 0) {
      await this.redis.del(...keys);
    }
  }
}
```

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2026-01-13 | WIA Standards Committee | Initial release |

---

© 2026 WIA (World Certification Industry Association) / SmileStory Inc.
**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
