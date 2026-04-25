# Chapter 8: Cryogenic Facility Implementation

## Deployment Architecture and Operational Guidelines

This chapter provides comprehensive implementation guidance for the WIA Cryo Facility Standard, including project structure, database schemas, service implementation, deployment configurations, and operational procedures.

---

## Project Structure

### Monorepo Architecture

```typescript
/**
 * WIA Cryo Facility Implementation
 * Monorepo project structure
 */

/*
cryo-facility/
├── packages/
│   ├── core/                    # Core domain models
│   │   ├── src/
│   │   │   ├── models/         # Domain entities
│   │   │   ├── services/       # Business logic
│   │   │   ├── repositories/   # Data access interfaces
│   │   │   └── events/         # Domain events
│   │   └── package.json
│   │
│   ├── api/                     # REST/GraphQL API
│   │   ├── src/
│   │   │   ├── routes/         # API endpoints
│   │   │   ├── controllers/    # Request handlers
│   │   │   ├── middleware/     # Express middleware
│   │   │   ├── graphql/        # GraphQL schema & resolvers
│   │   │   └── websocket/      # WebSocket handlers
│   │   └── package.json
│   │
│   ├── monitoring/              # Equipment monitoring
│   │   ├── src/
│   │   │   ├── collectors/     # Data collectors
│   │   │   ├── processors/     # Data processors
│   │   │   ├── alerts/         # Alert management
│   │   │   └── storage/        # Time-series storage
│   │   └── package.json
│   │
│   ├── integration/             # External integrations
│   │   ├── src/
│   │   │   ├── lims/           # LIMS adapter
│   │   │   ├── bms/            # Building management
│   │   │   ├── erp/            # ERP integration
│   │   │   └── events/         # Event bus
│   │   └── package.json
│   │
│   ├── security/                # Security services
│   │   ├── src/
│   │   │   ├── auth/           # Authentication
│   │   │   ├── authz/          # Authorization
│   │   │   ├── crypto/         # Encryption
│   │   │   └── audit/          # Audit logging
│   │   └── package.json
│   │
│   └── web-ui/                  # Web dashboard
│       ├── src/
│       │   ├── components/     # React components
│       │   ├── pages/          # Page components
│       │   ├── hooks/          # Custom hooks
│       │   └── services/       # API clients
│       └── package.json
│
├── apps/
│   ├── api-server/              # Main API application
│   ├── monitoring-service/      # Monitoring service
│   ├── alert-service/           # Alert processing
│   └── integration-service/     # Integration orchestrator
│
├── infrastructure/
│   ├── kubernetes/              # K8s manifests
│   ├── terraform/               # Infrastructure as code
│   ├── docker/                  # Docker configurations
│   └── scripts/                 # Deployment scripts
│
├── docs/                        # Documentation
├── tests/                       # Integration tests
├── turbo.json                   # Turborepo config
├── package.json                 # Root package.json
└── tsconfig.json                # TypeScript config
*/

// Package.json configuration
const rootPackageJson = {
  name: 'cryo-facility',
  version: '1.0.0',
  private: true,
  workspaces: [
    'packages/*',
    'apps/*'
  ],
  scripts: {
    build: 'turbo run build',
    dev: 'turbo run dev',
    test: 'turbo run test',
    lint: 'turbo run lint',
    'db:migrate': 'prisma migrate deploy',
    'db:generate': 'prisma generate'
  },
  devDependencies: {
    turbo: '^1.10.0',
    typescript: '^5.0.0',
    '@types/node': '^20.0.0',
    eslint: '^8.0.0',
    prettier: '^3.0.0'
  }
};

// Turbo configuration
const turboConfig = {
  $schema: 'https://turbo.build/schema.json',
  pipeline: {
    build: {
      dependsOn: ['^build'],
      outputs: ['dist/**']
    },
    dev: {
      cache: false,
      persistent: true
    },
    test: {
      dependsOn: ['build'],
      outputs: ['coverage/**']
    },
    lint: {}
  }
};
```

---

## Database Schema

### PostgreSQL Database Design

```typescript
/**
 * Database Schema Implementation
 * PostgreSQL with Prisma ORM
 */

// Prisma schema (schema.prisma)
const prismaSchema = `
generator client {
  provider = "prisma-client-js"
}

datasource db {
  provider = "postgresql"
  url      = env("DATABASE_URL")
}

// Facility Management
model Facility {
  id                String           @id @default(uuid())
  name              String
  description       String?
  type              FacilityType
  status            FacilityStatus   @default(OPERATIONAL)

  // Location
  address           String
  city              String
  state             String?
  country           String
  postalCode        String
  latitude          Float
  longitude         Float
  timezone          String

  // Organization
  organizationName  String
  organizationType  String
  registrationNum   String?
  contactName       String
  contactEmail      String
  contactPhone      String
  emergencyPhone    String?

  // Relationships
  zones             Zone[]
  equipment         Equipment[]
  licenses          License[]
  certifications    Certification[]
  incidents         Incident[]
  audits            Audit[]

  // Timestamps
  createdAt         DateTime         @default(now())
  updatedAt         DateTime         @updatedAt

  @@index([status])
  @@index([type])
}

enum FacilityType {
  BIOBANK
  TISSUE_BANK
  FERTILITY_CENTER
  RESEARCH_FACILITY
  HOSPITAL_UNIT
  COMMERCIAL_STORAGE
}

enum FacilityStatus {
  OPERATIONAL
  LIMITED_OPERATIONS
  MAINTENANCE
  EMERGENCY
  OFFLINE
}

// Zone Management
model Zone {
  id                    String              @id @default(uuid())
  facilityId            String
  facility              Facility            @relation(fields: [facilityId], references: [id])
  name                  String
  type                  ZoneType
  classification        CleanroomClass      @default(NON_CLASSIFIED)
  area                  Float
  accessLevel           AccessLevel         @default(RESTRICTED)

  // Environmental requirements
  tempMin               Float
  tempMax               Float
  tempUnit              String              @default("celsius")
  humidityMin           Float
  humidityMax           Float
  particulateSize       Float?
  particulateCount      Float?

  // Relationships
  equipment             Equipment[]
  environmentalReadings EnvironmentalReading[]
  accessLog             AccessLog[]

  createdAt             DateTime            @default(now())
  updatedAt             DateTime            @updatedAt

  @@index([facilityId])
  @@index([type])
}

enum ZoneType {
  STORAGE
  PROCESSING
  QUALITY_CONTROL
  RECEIVING
  SHIPPING
  PREPARATION
  OFFICE
  UTILITY
}

enum CleanroomClass {
  ISO_5
  ISO_6
  ISO_7
  ISO_8
  NON_CLASSIFIED
}

enum AccessLevel {
  PUBLIC
  RESTRICTED
  CONTROLLED
  HIGH_SECURITY
}

// Equipment Management
model Equipment {
  id                String              @id @default(uuid())
  facilityId        String
  facility          Facility            @relation(fields: [facilityId], references: [id])
  zoneId            String?
  zone              Zone?               @relation(fields: [zoneId], references: [id])

  type              EquipmentType
  model             String
  manufacturer      String
  serialNumber      String              @unique
  capacity          Int
  operatingTemp     Float
  status            EquipmentStatus     @default(OPERATIONAL)

  // Installation
  installationDate  DateTime
  installerName     String
  warrantyExpires   DateTime
  warrantyProvider  String

  // Maintenance
  lastService       DateTime
  nextService       DateTime
  serviceProvider   String

  // Relationships
  sensors           Sensor[]
  readings          EquipmentReading[]
  alerts            Alert[]
  maintenanceLog    MaintenanceRecord[]

  createdAt         DateTime            @default(now())
  updatedAt         DateTime            @updatedAt

  @@index([facilityId])
  @@index([zoneId])
  @@index([status])
  @@index([nextService])
}

enum EquipmentType {
  LN2_TANK
  LN2_FREEZER
  MECHANICAL_FREEZER
  CONTROLLED_RATE_FREEZER
  INCUBATOR
}

enum EquipmentStatus {
  OPERATIONAL
  WARNING
  ALARM
  MAINTENANCE
  OFFLINE
  DECOMMISSIONED
}

// Sensor Management
model Sensor {
  id                String              @id @default(uuid())
  equipmentId       String
  equipment         Equipment           @relation(fields: [equipmentId], references: [id])

  type              SensorType
  location          String
  accuracy          Float

  // Calibration
  lastCalibration   DateTime
  nextCalibration   DateTime
  calibrationCert   String?

  // Relationships
  readings          SensorReading[]
  alertConfigs      AlertConfig[]

  createdAt         DateTime            @default(now())
  updatedAt         DateTime            @updatedAt

  @@index([equipmentId])
  @@index([type])
}

enum SensorType {
  TEMPERATURE
  LEVEL
  PRESSURE
  HUMIDITY
}

// Time Series Data
model SensorReading {
  id          String    @id @default(uuid())
  sensorId    String
  sensor      Sensor    @relation(fields: [sensorId], references: [id])
  value       Float
  unit        String
  timestamp   DateTime  @default(now())

  @@index([sensorId, timestamp])
}

model EquipmentReading {
  id          String      @id @default(uuid())
  equipmentId String
  equipment   Equipment   @relation(fields: [equipmentId], references: [id])
  temperature Float
  level       Float?
  pressure    Float?
  timestamp   DateTime    @default(now())

  @@index([equipmentId, timestamp])
}

model EnvironmentalReading {
  id          String    @id @default(uuid())
  zoneId      String
  zone        Zone      @relation(fields: [zoneId], references: [id])
  temperature Float
  humidity    Float
  pressure    Float?
  particulate Float?
  timestamp   DateTime  @default(now())

  @@index([zoneId, timestamp])
}

// Alert Management
model Alert {
  id              String        @id @default(uuid())
  equipmentId     String
  equipment       Equipment     @relation(fields: [equipmentId], references: [id])

  parameter       String
  value           Float
  threshold       Float
  severity        AlertSeverity
  message         String

  status          AlertStatus   @default(ACTIVE)
  acknowledgedAt  DateTime?
  acknowledgedBy  String?
  resolvedAt      DateTime?
  resolvedBy      String?
  resolution      String?

  createdAt       DateTime      @default(now())

  @@index([equipmentId])
  @@index([status])
  @@index([severity])
  @@index([createdAt])
}

enum AlertSeverity {
  INFO
  WARNING
  CRITICAL
}

enum AlertStatus {
  ACTIVE
  ACKNOWLEDGED
  RESOLVED
}

model AlertConfig {
  id          String        @id @default(uuid())
  sensorId    String
  sensor      Sensor        @relation(fields: [sensorId], references: [id])

  parameter   String
  threshold   Float
  condition   String
  severity    AlertSeverity
  recipients  String[]

  // Escalation
  escalationDelay   Int?
  escalationRecipients String[]

  enabled     Boolean       @default(true)

  createdAt   DateTime      @default(now())
  updatedAt   DateTime      @updatedAt

  @@index([sensorId])
}

// Maintenance Management
model MaintenanceRecord {
  id            String      @id @default(uuid())
  equipmentId   String
  equipment     Equipment   @relation(fields: [equipmentId], references: [id])

  type          String
  technician    String
  description   String
  parts         String[]

  scheduledDate DateTime?
  completedDate DateTime?
  nextServiceDate DateTime?

  status        MaintenanceStatus @default(SCHEDULED)

  createdAt     DateTime    @default(now())
  updatedAt     DateTime    @updatedAt

  @@index([equipmentId])
  @@index([status])
  @@index([scheduledDate])
}

enum MaintenanceStatus {
  SCHEDULED
  IN_PROGRESS
  COMPLETED
  CANCELLED
}

// Compliance & Audit
model License {
  id          String    @id @default(uuid())
  facilityId  String
  facility    Facility  @relation(fields: [facilityId], references: [id])

  type        String
  number      String
  issuer      String
  validFrom   DateTime
  validTo     DateTime
  scope       String[]
  status      String

  createdAt   DateTime  @default(now())
  updatedAt   DateTime  @updatedAt

  @@index([facilityId])
  @@index([validTo])
}

model Certification {
  id          String    @id @default(uuid())
  facilityId  String
  facility    Facility  @relation(fields: [facilityId], references: [id])

  name        String
  body        String
  number      String
  scope       String[]
  validFrom   DateTime
  validTo     DateTime
  status      String

  createdAt   DateTime  @default(now())
  updatedAt   DateTime  @updatedAt

  @@index([facilityId])
  @@index([validTo])
}

model Audit {
  id          String    @id @default(uuid())
  facilityId  String
  facility    Facility  @relation(fields: [facilityId], references: [id])

  type        String
  auditor     String
  scope       String[]
  findings    Json
  status      String

  scheduledDate DateTime
  completedDate DateTime?

  createdAt   DateTime  @default(now())
  updatedAt   DateTime  @updatedAt

  @@index([facilityId])
  @@index([scheduledDate])
}

model Incident {
  id          String    @id @default(uuid())
  facilityId  String
  facility    Facility  @relation(fields: [facilityId], references: [id])

  type        String
  severity    String
  description String
  location    String?

  reportedBy  String
  reportedAt  DateTime  @default(now())

  status      String    @default("OPEN")
  resolution  String?
  resolvedAt  DateTime?
  resolvedBy  String?

  rootCause   String?
  preventive  String[]

  createdAt   DateTime  @default(now())
  updatedAt   DateTime  @updatedAt

  @@index([facilityId])
  @@index([status])
  @@index([severity])
}

// Access Control
model User {
  id            String    @id @default(uuid())
  email         String    @unique
  passwordHash  String
  firstName     String
  lastName      String
  department    String?
  badgeNumber   String?   @unique

  accessLevel   Int       @default(1)
  roles         String[]
  mfaEnabled    Boolean   @default(false)
  mfaSecret     String?

  status        String    @default("ACTIVE")
  lastLogin     DateTime?

  createdAt     DateTime  @default(now())
  updatedAt     DateTime  @updatedAt

  accessLog     AccessLog[]
  auditLog      AuditLog[]

  @@index([email])
  @@index([badgeNumber])
  @@index([status])
}

model AccessLog {
  id          String    @id @default(uuid())
  userId      String
  user        User      @relation(fields: [userId], references: [id])
  zoneId      String?
  zone        Zone?     @relation(fields: [zoneId], references: [id])

  action      String
  location    String?
  result      String
  reason      String?

  ipAddress   String?
  userAgent   String?

  timestamp   DateTime  @default(now())

  @@index([userId])
  @@index([zoneId])
  @@index([timestamp])
}

model AuditLog {
  id            String    @id @default(uuid())
  userId        String?
  user          User?     @relation(fields: [userId], references: [id])

  action        String
  resourceType  String
  resourceId    String?
  changes       Json?

  ipAddress     String?
  userAgent     String?

  timestamp     DateTime  @default(now())

  @@index([userId])
  @@index([resourceType, resourceId])
  @@index([timestamp])
}
`;
```

---

## Service Implementation

### Core API Service

```typescript
/**
 * API Service Implementation
 * Express + Apollo Server
 */

import express, { Application, Request, Response, NextFunction } from 'express';
import { ApolloServer } from '@apollo/server';
import { expressMiddleware } from '@apollo/server/express4';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import { useServer } from 'graphql-ws/lib/use/ws';
import { makeExecutableSchema } from '@graphql-tools/schema';
import { PrismaClient } from '@prisma/client';
import cors from 'cors';
import helmet from 'helmet';
import compression from 'compression';
import morgan from 'morgan';

// Application configuration
interface AppConfig {
  port: number;
  env: string;
  cors: {
    origins: string[];
    credentials: boolean;
  };
  rateLimit: {
    windowMs: number;
    max: number;
  };
  jwt: {
    secret: string;
    expiresIn: string;
  };
}

// Main application class
class CryoFacilityAPI {
  private app: Application;
  private httpServer: ReturnType<typeof createServer>;
  private prisma: PrismaClient;
  private apolloServer: ApolloServer;
  private config: AppConfig;

  constructor(config: AppConfig) {
    this.config = config;
    this.app = express();
    this.httpServer = createServer(this.app);
    this.prisma = new PrismaClient();
    this.apolloServer = this.createApolloServer();
  }

  async initialize(): Promise<void> {
    // Connect to database
    await this.prisma.$connect();

    // Configure middleware
    this.configureMiddleware();

    // Configure REST routes
    this.configureRoutes();

    // Configure GraphQL
    await this.configureGraphQL();

    // Configure WebSocket
    this.configureWebSocket();

    // Error handling
    this.configureErrorHandling();
  }

  private configureMiddleware(): void {
    // Security headers
    this.app.use(helmet({
      contentSecurityPolicy: this.config.env === 'production'
    }));

    // CORS
    this.app.use(cors({
      origin: this.config.cors.origins,
      credentials: this.config.cors.credentials
    }));

    // Compression
    this.app.use(compression());

    // Request parsing
    this.app.use(express.json({ limit: '10mb' }));
    this.app.use(express.urlencoded({ extended: true }));

    // Logging
    this.app.use(morgan('combined'));

    // Request ID
    this.app.use((req: Request, res: Response, next: NextFunction) => {
      req.headers['x-request-id'] = req.headers['x-request-id'] ||
        `req-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      res.setHeader('X-Request-ID', req.headers['x-request-id']);
      next();
    });
  }

  private configureRoutes(): void {
    // Health check
    this.app.get('/health', (req, res) => {
      res.json({ status: 'healthy', timestamp: new Date().toISOString() });
    });

    // API routes
    this.app.use('/api/v1/facilities', this.createFacilityRouter());
    this.app.use('/api/v1/equipment', this.createEquipmentRouter());
    this.app.use('/api/v1/monitoring', this.createMonitoringRouter());
    this.app.use('/api/v1/alerts', this.createAlertRouter());
    this.app.use('/api/v1/maintenance', this.createMaintenanceRouter());
    this.app.use('/api/v1/auth', this.createAuthRouter());
  }

  private createFacilityRouter(): express.Router {
    const router = express.Router();
    const controller = new FacilityController(this.prisma);

    router.get('/', controller.list.bind(controller));
    router.get('/:id', controller.get.bind(controller));
    router.post('/', controller.create.bind(controller));
    router.put('/:id', controller.update.bind(controller));
    router.delete('/:id', controller.delete.bind(controller));
    router.get('/:id/status', controller.getStatus.bind(controller));
    router.get('/:id/zones', controller.listZones.bind(controller));

    return router;
  }

  private createEquipmentRouter(): express.Router {
    const router = express.Router();
    const controller = new EquipmentController(this.prisma);

    router.get('/', controller.list.bind(controller));
    router.get('/:id', controller.get.bind(controller));
    router.post('/', controller.create.bind(controller));
    router.put('/:id', controller.update.bind(controller));
    router.get('/:id/readings', controller.getReadings.bind(controller));
    router.get('/:id/alerts', controller.getAlerts.bind(controller));
    router.post('/:id/maintenance', controller.recordMaintenance.bind(controller));

    return router;
  }

  private createMonitoringRouter(): express.Router {
    const router = express.Router();
    const controller = new MonitoringController(this.prisma);

    router.get('/readings', controller.getReadings.bind(controller));
    router.get('/environmental', controller.getEnvironmental.bind(controller));
    router.get('/dashboard', controller.getDashboard.bind(controller));

    return router;
  }

  private createAlertRouter(): express.Router {
    const router = express.Router();
    const controller = new AlertController(this.prisma);

    router.get('/', controller.list.bind(controller));
    router.get('/active', controller.listActive.bind(controller));
    router.post('/:id/acknowledge', controller.acknowledge.bind(controller));
    router.post('/:id/resolve', controller.resolve.bind(controller));

    return router;
  }

  private createMaintenanceRouter(): express.Router {
    const router = express.Router();
    const controller = new MaintenanceController(this.prisma);

    router.get('/', controller.list.bind(controller));
    router.get('/upcoming', controller.listUpcoming.bind(controller));
    router.post('/', controller.schedule.bind(controller));
    router.put('/:id', controller.update.bind(controller));
    router.post('/:id/complete', controller.complete.bind(controller));

    return router;
  }

  private createAuthRouter(): express.Router {
    const router = express.Router();
    const controller = new AuthController(this.prisma);

    router.post('/login', controller.login.bind(controller));
    router.post('/refresh', controller.refresh.bind(controller));
    router.post('/logout', controller.logout.bind(controller));
    router.get('/me', controller.getCurrentUser.bind(controller));

    return router;
  }

  private createApolloServer(): ApolloServer {
    const schema = makeExecutableSchema({
      typeDefs: graphqlTypeDefs,
      resolvers: graphqlResolvers
    });

    return new ApolloServer({
      schema,
      plugins: [
        {
          async serverWillStart() {
            return {
              async drainServer() {
                // Cleanup
              }
            };
          }
        }
      ]
    });
  }

  private async configureGraphQL(): Promise<void> {
    await this.apolloServer.start();

    this.app.use(
      '/graphql',
      expressMiddleware(this.apolloServer, {
        context: async ({ req }) => ({
          prisma: this.prisma,
          user: (req as any).user
        })
      })
    );
  }

  private configureWebSocket(): void {
    const wsServer = new WebSocketServer({
      server: this.httpServer,
      path: '/ws/monitoring'
    });

    // WebSocket connection handler
    wsServer.on('connection', (ws, req) => {
      console.log('WebSocket client connected');

      ws.on('message', (message) => {
        const data = JSON.parse(message.toString());
        this.handleWebSocketMessage(ws, data);
      });

      ws.on('close', () => {
        console.log('WebSocket client disconnected');
      });
    });
  }

  private handleWebSocketMessage(ws: any, message: any): void {
    switch (message.type) {
      case 'subscribe':
        // Handle subscription
        break;
      case 'unsubscribe':
        // Handle unsubscription
        break;
      case 'heartbeat':
        ws.send(JSON.stringify({ type: 'heartbeat', timestamp: new Date().toISOString() }));
        break;
    }
  }

  private configureErrorHandling(): void {
    // 404 handler
    this.app.use((req: Request, res: Response) => {
      res.status(404).json({
        error: {
          code: 'NOT_FOUND',
          message: `Route ${req.method} ${req.path} not found`
        }
      });
    });

    // Error handler
    this.app.use((err: Error, req: Request, res: Response, next: NextFunction) => {
      console.error('Error:', err);

      res.status(500).json({
        error: {
          code: 'INTERNAL_ERROR',
          message: this.config.env === 'production' ?
            'Internal server error' :
            err.message
        }
      });
    });
  }

  async start(): Promise<void> {
    await this.initialize();

    this.httpServer.listen(this.config.port, () => {
      console.log(`Server running on port ${this.config.port}`);
      console.log(`GraphQL endpoint: http://localhost:${this.config.port}/graphql`);
      console.log(`WebSocket endpoint: ws://localhost:${this.config.port}/ws/monitoring`);
    });
  }

  async stop(): Promise<void> {
    await this.apolloServer.stop();
    await this.prisma.$disconnect();
    this.httpServer.close();
  }
}

// Controller implementations (stubs)
class FacilityController {
  constructor(private prisma: PrismaClient) {}
  async list(req: Request, res: Response) { res.json([]); }
  async get(req: Request, res: Response) { res.json({}); }
  async create(req: Request, res: Response) { res.json({}); }
  async update(req: Request, res: Response) { res.json({}); }
  async delete(req: Request, res: Response) { res.status(204).send(); }
  async getStatus(req: Request, res: Response) { res.json({}); }
  async listZones(req: Request, res: Response) { res.json([]); }
}

class EquipmentController {
  constructor(private prisma: PrismaClient) {}
  async list(req: Request, res: Response) { res.json([]); }
  async get(req: Request, res: Response) { res.json({}); }
  async create(req: Request, res: Response) { res.json({}); }
  async update(req: Request, res: Response) { res.json({}); }
  async getReadings(req: Request, res: Response) { res.json([]); }
  async getAlerts(req: Request, res: Response) { res.json([]); }
  async recordMaintenance(req: Request, res: Response) { res.json({}); }
}

class MonitoringController {
  constructor(private prisma: PrismaClient) {}
  async getReadings(req: Request, res: Response) { res.json([]); }
  async getEnvironmental(req: Request, res: Response) { res.json([]); }
  async getDashboard(req: Request, res: Response) { res.json({}); }
}

class AlertController {
  constructor(private prisma: PrismaClient) {}
  async list(req: Request, res: Response) { res.json([]); }
  async listActive(req: Request, res: Response) { res.json([]); }
  async acknowledge(req: Request, res: Response) { res.json({}); }
  async resolve(req: Request, res: Response) { res.json({}); }
}

class MaintenanceController {
  constructor(private prisma: PrismaClient) {}
  async list(req: Request, res: Response) { res.json([]); }
  async listUpcoming(req: Request, res: Response) { res.json([]); }
  async schedule(req: Request, res: Response) { res.json({}); }
  async update(req: Request, res: Response) { res.json({}); }
  async complete(req: Request, res: Response) { res.json({}); }
}

class AuthController {
  constructor(private prisma: PrismaClient) {}
  async login(req: Request, res: Response) { res.json({}); }
  async refresh(req: Request, res: Response) { res.json({}); }
  async logout(req: Request, res: Response) { res.status(204).send(); }
  async getCurrentUser(req: Request, res: Response) { res.json({}); }
}

// GraphQL stubs
const graphqlTypeDefs = `type Query { health: String }`;
const graphqlResolvers = { Query: { health: () => 'OK' } };

export { CryoFacilityAPI };
```

---

## Kubernetes Deployment

### Container Orchestration

```yaml
# kubernetes/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: cryo-facility
  labels:
    app: cryo-facility
    environment: production

---
# kubernetes/configmap.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: cryo-facility-config
  namespace: cryo-facility
data:
  NODE_ENV: "production"
  LOG_LEVEL: "info"
  API_PORT: "3000"
  CORS_ORIGINS: "https://facility.example.com"

---
# kubernetes/secret.yaml (base64 encoded in production)
apiVersion: v1
kind: Secret
metadata:
  name: cryo-facility-secrets
  namespace: cryo-facility
type: Opaque
stringData:
  DATABASE_URL: "postgresql://user:password@postgres:5432/cryofacility"
  JWT_SECRET: "your-jwt-secret-key"
  ENCRYPTION_KEY: "your-encryption-key"

---
# kubernetes/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-facility-api
  namespace: cryo-facility
  labels:
    app: cryo-facility-api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: cryo-facility-api
  template:
    metadata:
      labels:
        app: cryo-facility-api
    spec:
      containers:
      - name: api
        image: cryo-facility/api:latest
        ports:
        - containerPort: 3000
          name: http
        - containerPort: 3001
          name: ws
        envFrom:
        - configMapRef:
            name: cryo-facility-config
        - secretRef:
            name: cryo-facility-secrets
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
        livenessProbe:
          httpGet:
            path: /health
            port: 3000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /health
            port: 3000
          initialDelaySeconds: 5
          periodSeconds: 5
        volumeMounts:
        - name: config
          mountPath: /app/config
          readOnly: true
      volumes:
      - name: config
        configMap:
          name: cryo-facility-config

---
# kubernetes/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: cryo-facility-api
  namespace: cryo-facility
spec:
  selector:
    app: cryo-facility-api
  ports:
  - name: http
    port: 80
    targetPort: 3000
  - name: ws
    port: 81
    targetPort: 3001
  type: ClusterIP

---
# kubernetes/ingress.yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: cryo-facility-ingress
  namespace: cryo-facility
  annotations:
    nginx.ingress.kubernetes.io/ssl-redirect: "true"
    nginx.ingress.kubernetes.io/proxy-body-size: "10m"
    cert-manager.io/cluster-issuer: "letsencrypt-prod"
spec:
  ingressClassName: nginx
  tls:
  - hosts:
    - api.facility.example.com
    secretName: cryo-facility-tls
  rules:
  - host: api.facility.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: cryo-facility-api
            port:
              number: 80

---
# kubernetes/hpa.yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: cryo-facility-api-hpa
  namespace: cryo-facility
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: cryo-facility-api
  minReplicas: 3
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80

---
# kubernetes/pdb.yaml
apiVersion: policy/v1
kind: PodDisruptionBudget
metadata:
  name: cryo-facility-api-pdb
  namespace: cryo-facility
spec:
  minAvailable: 2
  selector:
    matchLabels:
      app: cryo-facility-api
```

---

## Testing Strategy

### Comprehensive Test Suite

```typescript
/**
 * Testing Implementation
 * Unit, Integration, and E2E tests
 */

import { describe, it, expect, beforeAll, afterAll, beforeEach } from 'vitest';
import { PrismaClient } from '@prisma/client';
import request from 'supertest';
import { CryoFacilityAPI } from '../src/app';

// Test configuration
const testConfig = {
  port: 4000,
  env: 'test',
  cors: { origins: ['*'], credentials: true },
  rateLimit: { windowMs: 60000, max: 1000 },
  jwt: { secret: 'test-secret', expiresIn: '1h' }
};

// Unit Tests
describe('Facility Service', () => {
  let prisma: PrismaClient;

  beforeAll(async () => {
    prisma = new PrismaClient();
    await prisma.$connect();
  });

  afterAll(async () => {
    await prisma.$disconnect();
  });

  describe('createFacility', () => {
    it('should create a new facility', async () => {
      const facilityData = {
        name: 'Test Facility',
        type: 'BIOBANK',
        address: '123 Test St',
        city: 'Test City',
        country: 'US',
        postalCode: '12345',
        latitude: 42.3601,
        longitude: -71.0589,
        timezone: 'America/New_York',
        organizationName: 'Test Org',
        organizationType: 'Healthcare',
        contactName: 'John Doe',
        contactEmail: 'john@example.com',
        contactPhone: '+1-555-0100'
      };

      const facility = await prisma.facility.create({
        data: facilityData
      });

      expect(facility).toBeDefined();
      expect(facility.name).toBe('Test Facility');
      expect(facility.status).toBe('OPERATIONAL');
    });
  });

  describe('getEquipmentStatus', () => {
    it('should return equipment with sensors and readings', async () => {
      const equipment = await prisma.equipment.findFirst({
        include: {
          sensors: true,
          readings: {
            take: 10,
            orderBy: { timestamp: 'desc' }
          }
        }
      });

      expect(equipment).toBeDefined();
    });
  });
});

// Integration Tests
describe('API Integration Tests', () => {
  let app: CryoFacilityAPI;
  let server: any;

  beforeAll(async () => {
    app = new CryoFacilityAPI(testConfig);
    await app.initialize();
    server = app;
  });

  afterAll(async () => {
    await app.stop();
  });

  describe('GET /health', () => {
    it('should return healthy status', async () => {
      const response = await request(server)
        .get('/health')
        .expect(200);

      expect(response.body.status).toBe('healthy');
    });
  });

  describe('Facility API', () => {
    let facilityId: string;

    it('POST /api/v1/facilities should create facility', async () => {
      const response = await request(server)
        .post('/api/v1/facilities')
        .send({
          name: 'Integration Test Facility',
          type: 'FERTILITY_CENTER',
          location: {
            address: '456 Test Ave',
            city: 'Boston',
            state: 'MA',
            country: 'US',
            postalCode: '02115',
            coordinates: { latitude: 42.3601, longitude: -71.0589 },
            timezone: 'America/New_York'
          },
          organization: {
            name: 'Test Healthcare',
            type: 'Healthcare',
            contact: {
              name: 'Jane Doe',
              email: 'jane@example.com',
              phone: '+1-555-0200'
            }
          }
        })
        .expect(201);

      expect(response.body.data.id).toBeDefined();
      facilityId = response.body.data.id;
    });

    it('GET /api/v1/facilities/:id should return facility', async () => {
      const response = await request(server)
        .get(`/api/v1/facilities/${facilityId}`)
        .expect(200);

      expect(response.body.data.name).toBe('Integration Test Facility');
    });

    it('GET /api/v1/facilities should list facilities', async () => {
      const response = await request(server)
        .get('/api/v1/facilities')
        .query({ limit: 10 })
        .expect(200);

      expect(Array.isArray(response.body.data)).toBe(true);
    });
  });

  describe('Equipment API', () => {
    it('GET /api/v1/equipment should list equipment', async () => {
      const response = await request(server)
        .get('/api/v1/equipment')
        .expect(200);

      expect(Array.isArray(response.body.data)).toBe(true);
    });

    it('GET /api/v1/equipment/:id/readings should return readings', async () => {
      const response = await request(server)
        .get('/api/v1/equipment/test-equipment-id/readings')
        .query({
          startTime: new Date(Date.now() - 24 * 60 * 60 * 1000).toISOString(),
          endTime: new Date().toISOString()
        })
        .expect(200);

      expect(response.body.data).toBeDefined();
    });
  });

  describe('Alert API', () => {
    it('GET /api/v1/alerts/active should return active alerts', async () => {
      const response = await request(server)
        .get('/api/v1/alerts/active')
        .expect(200);

      expect(Array.isArray(response.body.data)).toBe(true);
    });
  });
});

// Performance Tests
describe('Performance Tests', () => {
  it('should handle concurrent requests', async () => {
    const promises = Array(100).fill(null).map(() =>
      request(server).get('/health')
    );

    const results = await Promise.all(promises);
    const successful = results.filter(r => r.status === 200);

    expect(successful.length).toBe(100);
  });
});
```

---

## Monitoring and Observability

### Prometheus Metrics

```typescript
/**
 * Monitoring Implementation
 * Prometheus metrics and health checks
 */

import { Registry, Counter, Histogram, Gauge } from 'prom-client';

class MetricsService {
  private registry: Registry;

  // HTTP metrics
  private httpRequestsTotal: Counter;
  private httpRequestDuration: Histogram;

  // Equipment metrics
  private equipmentStatus: Gauge;
  private alertsActive: Gauge;
  private temperatureReadings: Gauge;

  // Business metrics
  private specimensStored: Gauge;
  private maintenanceOverdue: Gauge;

  constructor() {
    this.registry = new Registry();
    this.initializeMetrics();
  }

  private initializeMetrics(): void {
    // HTTP metrics
    this.httpRequestsTotal = new Counter({
      name: 'cryo_http_requests_total',
      help: 'Total HTTP requests',
      labelNames: ['method', 'path', 'status'],
      registers: [this.registry]
    });

    this.httpRequestDuration = new Histogram({
      name: 'cryo_http_request_duration_seconds',
      help: 'HTTP request duration',
      labelNames: ['method', 'path'],
      buckets: [0.01, 0.05, 0.1, 0.5, 1, 5],
      registers: [this.registry]
    });

    // Equipment metrics
    this.equipmentStatus = new Gauge({
      name: 'cryo_equipment_status',
      help: 'Equipment status (1=operational, 0=not operational)',
      labelNames: ['equipment_id', 'equipment_type', 'facility_id'],
      registers: [this.registry]
    });

    this.alertsActive = new Gauge({
      name: 'cryo_alerts_active',
      help: 'Number of active alerts',
      labelNames: ['severity', 'facility_id'],
      registers: [this.registry]
    });

    this.temperatureReadings = new Gauge({
      name: 'cryo_temperature_celsius',
      help: 'Current temperature readings',
      labelNames: ['equipment_id', 'sensor_id'],
      registers: [this.registry]
    });

    // Business metrics
    this.specimensStored = new Gauge({
      name: 'cryo_specimens_stored_total',
      help: 'Total specimens stored',
      labelNames: ['facility_id', 'specimen_type'],
      registers: [this.registry]
    });

    this.maintenanceOverdue = new Gauge({
      name: 'cryo_maintenance_overdue',
      help: 'Number of overdue maintenance tasks',
      labelNames: ['facility_id'],
      registers: [this.registry]
    });
  }

  recordHttpRequest(method: string, path: string, status: number, duration: number): void {
    this.httpRequestsTotal.inc({ method, path, status: String(status) });
    this.httpRequestDuration.observe({ method, path }, duration);
  }

  updateEquipmentStatus(
    equipmentId: string,
    equipmentType: string,
    facilityId: string,
    operational: boolean
  ): void {
    this.equipmentStatus.set(
      { equipment_id: equipmentId, equipment_type: equipmentType, facility_id: facilityId },
      operational ? 1 : 0
    );
  }

  updateActiveAlerts(facilityId: string, severity: string, count: number): void {
    this.alertsActive.set({ facility_id: facilityId, severity }, count);
  }

  updateTemperature(equipmentId: string, sensorId: string, temperature: number): void {
    this.temperatureReadings.set({ equipment_id: equipmentId, sensor_id: sensorId }, temperature);
  }

  async getMetrics(): Promise<string> {
    return this.registry.metrics();
  }
}

export { MetricsService };
```

---

## Chapter Summary

This chapter covered comprehensive implementation guidance:

- **Project Structure**: Monorepo architecture with packages and apps
- **Database Schema**: PostgreSQL with Prisma ORM
- **Service Implementation**: Express + Apollo Server API
- **Kubernetes Deployment**: Container orchestration with HPA and PDB
- **Testing Strategy**: Unit, integration, and performance tests
- **Monitoring**: Prometheus metrics and observability

---

*© 2025 World Industry Association. All rights reserved.*

*弘益人間 (Benefit All Humanity)*
