# Chapter 8: Implementation

## Deployment and Operations Guide

This chapter provides comprehensive guidance for implementing the WIA Cryo-Consent Standard, including infrastructure setup, deployment procedures, testing strategies, and operational best practices.

---

## 8.1 Project Structure

```typescript
// Recommended monorepo structure
const projectStructure = `
cryo-consent/
├── packages/
│   ├── core/                     # Core consent domain logic
│   │   ├── src/
│   │   │   ├── consent/          # Consent entity and services
│   │   │   ├── decision/         # Decision engine
│   │   │   ├── proxy/            # Proxy management
│   │   │   ├── document/         # Document handling
│   │   │   └── workflow/         # Workflow engine
│   │   ├── tests/
│   │   └── package.json
│   │
│   ├── api/                      # REST/GraphQL API
│   │   ├── src/
│   │   │   ├── routes/           # API routes
│   │   │   ├── middleware/       # Auth, validation, etc.
│   │   │   ├── graphql/          # GraphQL schema/resolvers
│   │   │   └── websocket/        # Real-time updates
│   │   ├── tests/
│   │   └── package.json
│   │
│   ├── security/                 # Security services
│   │   ├── src/
│   │   │   ├── crypto/           # Cryptographic services
│   │   │   ├── access/           # Access control
│   │   │   ├── audit/            # Audit logging
│   │   │   └── threat/           # Threat detection
│   │   ├── tests/
│   │   └── package.json
│   │
│   ├── integration/              # External integrations
│   │   ├── src/
│   │   │   ├── healthcare/       # HL7/FHIR integration
│   │   │   ├── legal/            # Legal system integration
│   │   │   ├── cryonics/         # Cryonics org integration
│   │   │   └── identity/         # Identity verification
│   │   ├── tests/
│   │   └── package.json
│   │
│   ├── storage/                  # Data persistence
│   │   ├── src/
│   │   │   ├── repositories/     # Data repositories
│   │   │   ├── migrations/       # Database migrations
│   │   │   └── cache/            # Caching layer
│   │   ├── tests/
│   │   └── package.json
│   │
│   └── common/                   # Shared utilities
│       ├── src/
│       │   ├── types/            # Shared types
│       │   ├── utils/            # Utility functions
│       │   └── errors/           # Custom errors
│       └── package.json
│
├── apps/
│   ├── consent-service/          # Main consent service
│   ├── worker-service/           # Background workers
│   ├── notification-service/     # Notification handling
│   └── admin-portal/             # Administration UI
│
├── infrastructure/
│   ├── kubernetes/               # K8s manifests
│   ├── terraform/                # Infrastructure as code
│   ├── docker/                   # Docker configurations
│   └── scripts/                  # Deployment scripts
│
├── docs/
│   ├── api/                      # API documentation
│   ├── architecture/             # Architecture docs
│   └── operations/               # Operational guides
│
├── turbo.json                    # Turborepo config
├── package.json
└── tsconfig.json
`;

// Package.json for root
const rootPackageJson = {
  name: '@wia/cryo-consent',
  version: '1.0.0',
  private: true,
  workspaces: [
    'packages/*',
    'apps/*',
  ],
  scripts: {
    build: 'turbo run build',
    test: 'turbo run test',
    lint: 'turbo run lint',
    dev: 'turbo run dev',
    clean: 'turbo run clean',
    'db:migrate': 'turbo run db:migrate --filter=@wia/storage',
    'db:seed': 'turbo run db:seed --filter=@wia/storage',
  },
  devDependencies: {
    turbo: '^2.0.0',
    typescript: '^5.3.0',
    '@types/node': '^20.0.0',
    eslint: '^8.56.0',
    prettier: '^3.2.0',
    husky: '^9.0.0',
    'lint-staged': '^15.0.0',
  },
};
```

---

## 8.2 Database Schema

```typescript
// PostgreSQL schema for consent management
const databaseSchema = `
-- Enable required extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Patients table
CREATE TABLE patients (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  external_id VARCHAR(255) UNIQUE NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Consents table
CREATE TABLE consents (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  patient_id UUID NOT NULL REFERENCES patients(id),
  organization_id UUID NOT NULL,

  consent_type VARCHAR(50) NOT NULL,
  category VARCHAR(50) NOT NULL,
  subcategories TEXT[] DEFAULT '{}',

  status VARCHAR(50) NOT NULL DEFAULT 'DRAFT',
  effective_date TIMESTAMPTZ,
  expiration_date TIMESTAMPTZ,

  -- Encrypted JSON data
  scope_data BYTEA NOT NULL,
  decisions_data BYTEA NOT NULL,
  authority_data BYTEA NOT NULL,

  -- Encryption metadata
  encryption_key_id VARCHAR(255) NOT NULL,
  encryption_algorithm VARCHAR(50) NOT NULL,

  -- Version tracking
  version INTEGER NOT NULL DEFAULT 1,
  previous_version_id UUID REFERENCES consents(id),

  -- Timestamps
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW(),

  -- Audit
  created_by UUID NOT NULL,
  updated_by UUID,

  -- Indexes
  CONSTRAINT consents_status_check CHECK (status IN (
    'DRAFT', 'PENDING_WITNESS', 'PENDING_NOTARIZATION',
    'PENDING_REVIEW', 'ACTIVE', 'SUSPENDED', 'REVOKED', 'EXPIRED', 'SUPERSEDED'
  ))
);

CREATE INDEX idx_consents_patient ON consents(patient_id);
CREATE INDEX idx_consents_status ON consents(status);
CREATE INDEX idx_consents_category ON consents(category);
CREATE INDEX idx_consents_effective_date ON consents(effective_date);

-- Consent decisions table (denormalized for querying)
CREATE TABLE consent_decisions (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  decision_id VARCHAR(255) NOT NULL,
  decision_type VARCHAR(50) NOT NULL,
  question_hash VARCHAR(64) NOT NULL, -- Hash for searching

  -- Encrypted answer data
  answer_data BYTEA NOT NULL,

  created_at TIMESTAMPTZ DEFAULT NOW(),

  UNIQUE(consent_id, decision_id)
);

CREATE INDEX idx_consent_decisions_consent ON consent_decisions(consent_id);
CREATE INDEX idx_consent_decisions_type ON consent_decisions(decision_type);

-- Consent status history
CREATE TABLE consent_status_history (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  from_status VARCHAR(50) NOT NULL,
  to_status VARCHAR(50) NOT NULL,
  changed_at TIMESTAMPTZ DEFAULT NOW(),
  changed_by UUID NOT NULL,
  reason TEXT,

  -- Integrity
  previous_hash VARCHAR(64),
  record_hash VARCHAR(64) NOT NULL
);

CREATE INDEX idx_consent_status_history_consent ON consent_status_history(consent_id);

-- Proxy designations table
CREATE TABLE proxy_designations (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  patient_id UUID NOT NULL REFERENCES patients(id),

  proxy_type VARCHAR(50) NOT NULL,
  priority_order INTEGER NOT NULL,

  -- Encrypted proxy data
  proxy_data BYTEA NOT NULL,
  authority_scope_data BYTEA NOT NULL,

  acceptance_status VARCHAR(50) NOT NULL DEFAULT 'PENDING',
  acceptance_date TIMESTAMPTZ,

  is_active BOOLEAN DEFAULT FALSE,
  activated_at TIMESTAMPTZ,
  deactivated_at TIMESTAMPTZ,

  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW(),

  UNIQUE(patient_id, priority_order)
);

CREATE INDEX idx_proxy_designations_patient ON proxy_designations(patient_id);
CREATE INDEX idx_proxy_designations_active ON proxy_designations(is_active);

-- Documents table
CREATE TABLE consent_documents (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  document_type VARCHAR(50) NOT NULL,
  title VARCHAR(500) NOT NULL,
  language VARCHAR(10) DEFAULT 'en',

  -- Storage
  storage_type VARCHAR(50) NOT NULL,
  storage_location TEXT NOT NULL,

  -- Encryption
  encrypted BOOLEAN DEFAULT TRUE,
  encryption_key_id VARCHAR(255),

  -- Integrity
  hash_algorithm VARCHAR(50) NOT NULL,
  hash_value VARCHAR(128) NOT NULL,
  additional_hashes JSONB,

  -- Blockchain anchor
  blockchain_network VARCHAR(50),
  blockchain_tx_id VARCHAR(128),
  blockchain_block_number BIGINT,

  -- Version
  version_number VARCHAR(50) NOT NULL,
  is_current BOOLEAN DEFAULT TRUE,

  created_at TIMESTAMPTZ DEFAULT NOW(),
  created_by UUID NOT NULL
);

CREATE INDEX idx_consent_documents_consent ON consent_documents(consent_id);
CREATE INDEX idx_consent_documents_type ON consent_documents(document_type);

-- Review schedule table
CREATE TABLE review_schedules (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  review_frequency VARCHAR(50) NOT NULL,
  review_type VARCHAR(50) NOT NULL,
  next_review_date TIMESTAMPTZ NOT NULL,

  notification_days INTEGER DEFAULT 30,
  reminder_schedule INTEGER[] DEFAULT '{30, 14, 7, 1}',
  default_if_no_response VARCHAR(50) DEFAULT 'MAINTAIN',

  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_review_schedules_consent ON review_schedules(consent_id);
CREATE INDEX idx_review_schedules_next_date ON review_schedules(next_review_date);

-- Review history table
CREATE TABLE review_history (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  scheduled_date TIMESTAMPTZ NOT NULL,
  completed_date TIMESTAMPTZ,

  review_type VARCHAR(50) NOT NULL,
  reviewed_by UUID,

  outcome VARCHAR(50),
  modifications JSONB,
  notes TEXT,

  next_review_scheduled TIMESTAMPTZ
);

CREATE INDEX idx_review_history_consent ON review_history(consent_id);

-- Audit log table (append-only)
CREATE TABLE audit_log (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),

  event_type VARCHAR(100) NOT NULL,
  severity VARCHAR(20) NOT NULL,

  -- Subject
  user_id UUID,
  session_id UUID,
  ip_address INET,
  user_agent TEXT,

  -- Resource
  resource_type VARCHAR(100),
  resource_id UUID,
  patient_id UUID,

  -- Action
  action VARCHAR(100) NOT NULL,
  outcome VARCHAR(50) NOT NULL,

  -- Details (encrypted for sensitive data)
  details_data BYTEA,

  -- Integrity chain
  previous_hash VARCHAR(64),
  record_hash VARCHAR(64) NOT NULL,

  -- Timestamp
  created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Partition audit log by month for performance
CREATE INDEX idx_audit_log_created ON audit_log(created_at);
CREATE INDEX idx_audit_log_user ON audit_log(user_id);
CREATE INDEX idx_audit_log_resource ON audit_log(resource_type, resource_id);
CREATE INDEX idx_audit_log_event ON audit_log(event_type);

-- External sync tracking
CREATE TABLE external_sync (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id),

  target_system VARCHAR(100) NOT NULL,
  external_id VARCHAR(255),

  last_sync_at TIMESTAMPTZ,
  last_sync_status VARCHAR(50),
  last_sync_error TEXT,

  sync_version INTEGER DEFAULT 0,

  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW(),

  UNIQUE(consent_id, target_system)
);

CREATE INDEX idx_external_sync_consent ON external_sync(consent_id);

-- Encryption keys metadata (actual keys in HSM)
CREATE TABLE encryption_keys (
  id VARCHAR(255) PRIMARY KEY,

  purpose VARCHAR(50) NOT NULL,
  algorithm VARCHAR(50) NOT NULL,
  classification_level VARCHAR(50) NOT NULL,

  status VARCHAR(50) NOT NULL DEFAULT 'ACTIVE',

  created_at TIMESTAMPTZ DEFAULT NOW(),
  expires_at TIMESTAMPTZ,
  rotated_at TIMESTAMPTZ,
  rotated_to VARCHAR(255),

  hsm_key_id VARCHAR(255) NOT NULL
);

CREATE INDEX idx_encryption_keys_status ON encryption_keys(status);
CREATE INDEX idx_encryption_keys_level ON encryption_keys(classification_level);
`;

// Migration runner
class DatabaseMigrationRunner {
  private db: Database;
  private migrationsDir: string;

  constructor(db: Database, migrationsDir: string) {
    this.db = db;
    this.migrationsDir = migrationsDir;
  }

  async runMigrations(): Promise<MigrationResult> {
    // Ensure migrations table exists
    await this.ensureMigrationsTable();

    // Get pending migrations
    const applied = await this.getAppliedMigrations();
    const available = await this.getAvailableMigrations();
    const pending = available.filter(m => !applied.includes(m.version));

    // Apply pending migrations
    const results: MigrationResult[] = [];

    for (const migration of pending) {
      try {
        await this.db.transaction(async (tx) => {
          // Run migration
          await tx.query(migration.up);

          // Record migration
          await tx.query(
            'INSERT INTO _migrations (version, name, applied_at) VALUES ($1, $2, NOW())',
            [migration.version, migration.name]
          );
        });

        results.push({
          version: migration.version,
          name: migration.name,
          success: true,
        });
      } catch (error) {
        results.push({
          version: migration.version,
          name: migration.name,
          success: false,
          error: error.message,
        });
        break; // Stop on error
      }
    }

    return {
      applied: results.filter(r => r.success).length,
      failed: results.filter(r => !r.success).length,
      results,
    };
  }

  private async ensureMigrationsTable(): Promise<void> {
    await this.db.query(`
      CREATE TABLE IF NOT EXISTS _migrations (
        id SERIAL PRIMARY KEY,
        version VARCHAR(50) NOT NULL UNIQUE,
        name VARCHAR(255) NOT NULL,
        applied_at TIMESTAMPTZ DEFAULT NOW()
      )
    `);
  }

  private async getAppliedMigrations(): Promise<string[]> {
    const result = await this.db.query('SELECT version FROM _migrations ORDER BY version');
    return result.rows.map(r => r.version);
  }

  private async getAvailableMigrations(): Promise<Migration[]> {
    const files = await fs.readdir(this.migrationsDir);
    const migrations: Migration[] = [];

    for (const file of files.filter(f => f.endsWith('.sql'))) {
      const content = await fs.readFile(path.join(this.migrationsDir, file), 'utf-8');
      const [version, ...nameParts] = file.replace('.sql', '').split('_');

      migrations.push({
        version,
        name: nameParts.join('_'),
        up: content,
      });
    }

    return migrations.sort((a, b) => a.version.localeCompare(b.version));
  }
}
```

---

## 8.3 Service Implementation

```typescript
// Main consent service implementation
import express from 'express';
import { ApolloServer } from '@apollo/server';
import { expressMiddleware } from '@apollo/server/express4';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';

class ConsentServiceApplication {
  private app: express.Application;
  private httpServer: http.Server;
  private apolloServer: ApolloServer;
  private wsServer: WebSocketServer;
  private services: ServiceContainer;

  constructor(config: ApplicationConfig) {
    this.app = express();
    this.services = this.initializeServices(config);
    this.setupMiddleware();
    this.setupRoutes();
    this.setupGraphQL();
    this.setupWebSocket();
  }

  private initializeServices(config: ApplicationConfig): ServiceContainer {
    // Initialize database
    const database = new Database(config.database);

    // Initialize repositories
    const consentRepository = new ConsentRepository(database);
    const proxyRepository = new ProxyRepository(database);
    const documentRepository = new DocumentRepository(database);
    const auditRepository = new AuditRepository(database);

    // Initialize security services
    const cryptoService = new CryptographicService(config.crypto);
    const accessControl = new AccessControlService(config.accessControl);
    const auditService = new SecurityAuditService(auditRepository, config.audit);

    // Initialize core services
    const consentService = new CryoConsentManagementService({
      consentRepository,
      validationService: new ConsentValidationService(),
      documentService: new DocumentService(documentRepository, config.storage),
      notificationService: new NotificationService(config.notifications),
      auditService,
    });

    const proxyService = new ProxyManagementService({
      proxyRepository,
      consentRepository,
      notificationService: new NotificationService(config.notifications),
    });

    const decisionService = new ConsentDecisionQueryService({
      consentRepository,
      interpretationEngine: new ConsentInterpretationEngine(),
    });

    // Initialize workflow engine
    const workflowEngine = new ConsentWorkflowEngine({
      executionRepository: new WorkflowExecutionRepository(database),
    });

    // Initialize integration orchestrator
    const integrationOrchestrator = new ConsentIntegrationOrchestrator(config.integration);

    return {
      database,
      consentService,
      proxyService,
      decisionService,
      workflowEngine,
      integrationOrchestrator,
      cryptoService,
      accessControl,
      auditService,
    };
  }

  private setupMiddleware(): void {
    // Security middleware
    this.app.use(helmet());
    this.app.use(cors(this.getCorsConfig()));

    // Rate limiting
    this.app.use(rateLimit({
      windowMs: 60 * 1000,
      max: 100,
    }));

    // Body parsing
    this.app.use(express.json({ limit: '10mb' }));
    this.app.use(express.urlencoded({ extended: true }));

    // Request logging
    this.app.use(morgan('combined'));

    // Authentication
    this.app.use(authenticationMiddleware(this.services.accessControl));

    // Request context
    this.app.use(requestContextMiddleware());

    // Audit logging
    this.app.use(auditMiddleware(this.services.auditService));
  }

  private setupRoutes(): void {
    // Health check
    this.app.get('/health', healthCheckHandler(this.services));

    // API routes
    const apiRouter = express.Router();

    // Consent routes
    apiRouter.use('/consents', createConsentRouter(this.services));

    // Proxy routes
    apiRouter.use('/proxies', createProxyRouter(this.services));

    // Decision routes
    apiRouter.use('/decisions', createDecisionRouter(this.services));

    // Document routes
    apiRouter.use('/documents', createDocumentRouter(this.services));

    // Patient routes
    apiRouter.use('/patients', createPatientRouter(this.services));

    // Audit routes
    apiRouter.use('/audit', createAuditRouter(this.services));

    this.app.use('/api/v1', apiRouter);
  }

  private async setupGraphQL(): Promise<void> {
    this.apolloServer = new ApolloServer({
      typeDefs,
      resolvers: createResolvers(this.services),
      plugins: [
        ApolloServerPluginLandingPageLocalDefault(),
        createAuditPlugin(this.services.auditService),
      ],
    });

    await this.apolloServer.start();

    this.app.use(
      '/graphql',
      expressMiddleware(this.apolloServer, {
        context: async ({ req }) => ({
          user: req.user,
          dataSources: this.services,
        }),
      })
    );
  }

  private setupWebSocket(): void {
    this.httpServer = createServer(this.app);
    this.wsServer = new ConsentWebSocketServer(this.httpServer, this.services);
  }

  async start(port: number): Promise<void> {
    // Run database migrations
    await this.services.database.runMigrations();

    // Start HTTP server
    this.httpServer.listen(port, () => {
      console.log(`Consent service listening on port ${port}`);
      console.log(`GraphQL endpoint: http://localhost:${port}/graphql`);
      console.log(`WebSocket endpoint: ws://localhost:${port}/ws`);
    });

    // Start background workers
    await this.startBackgroundWorkers();

    // Setup graceful shutdown
    this.setupGracefulShutdown();
  }

  private async startBackgroundWorkers(): Promise<void> {
    // Review reminder worker
    const reviewWorker = new ReviewReminderWorker(this.services);
    reviewWorker.start();

    // Sync worker
    const syncWorker = new ExternalSyncWorker(this.services);
    syncWorker.start();

    // Audit integrity worker
    const integrityWorker = new AuditIntegrityWorker(this.services);
    integrityWorker.start();
  }

  private setupGracefulShutdown(): void {
    const shutdown = async () => {
      console.log('Shutting down gracefully...');

      // Stop accepting new requests
      this.httpServer.close();

      // Close WebSocket connections
      this.wsServer.close();

      // Stop Apollo
      await this.apolloServer.stop();

      // Close database
      await this.services.database.close();

      process.exit(0);
    };

    process.on('SIGTERM', shutdown);
    process.on('SIGINT', shutdown);
  }
}

// Router factory
function createConsentRouter(services: ServiceContainer): express.Router {
  const router = express.Router();

  // Create consent
  router.post('/', async (req, res, next) => {
    try {
      const consent = await services.consentService.createConsent(req.body, {
        createdBy: req.user.id,
        ipAddress: req.ip,
      });

      res.status(201).json({
        success: true,
        data: { consent },
      });
    } catch (error) {
      next(error);
    }
  });

  // Get consent
  router.get('/:id', async (req, res, next) => {
    try {
      const consent = await services.consentService.getConsent(req.params.id);

      if (!consent) {
        return res.status(404).json({
          success: false,
          error: { code: 'NOT_FOUND', message: 'Consent not found' },
        });
      }

      // Check authorization
      const canAccess = await services.accessControl.checkAccess({
        userId: req.user.id,
        resource: 'consent',
        resourceId: consent.id,
        action: 'read',
      });

      if (!canAccess.allowed) {
        return res.status(403).json({
          success: false,
          error: { code: 'UNAUTHORIZED', message: 'Access denied' },
        });
      }

      res.json({
        success: true,
        data: { consent },
      });
    } catch (error) {
      next(error);
    }
  });

  // Update consent
  router.put('/:id', async (req, res, next) => {
    try {
      const consent = await services.consentService.updateConsent(
        req.params.id,
        req.body,
        {
          updatedBy: req.user.id,
          reason: req.body.reason,
          ipAddress: req.ip,
        }
      );

      res.json({
        success: true,
        data: { consent },
      });
    } catch (error) {
      next(error);
    }
  });

  // Revoke consent
  router.delete('/:id', async (req, res, next) => {
    try {
      const revoked = await services.consentService.revokeConsent(
        req.params.id,
        {
          revokedBy: req.user.id,
          reason: req.body.reason,
          ipAddress: req.ip,
        }
      );

      res.json({
        success: true,
        data: { consent: revoked },
      });
    } catch (error) {
      next(error);
    }
  });

  return router;
}
```

---

## 8.4 Kubernetes Deployment

```yaml
# kubernetes/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: cryo-consent
  labels:
    app.kubernetes.io/name: cryo-consent

---
# kubernetes/configmap.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: consent-config
  namespace: cryo-consent
data:
  NODE_ENV: production
  LOG_LEVEL: info
  API_PORT: "3000"
  GRAPHQL_PATH: /graphql
  WS_PATH: /ws

---
# kubernetes/secret.yaml
apiVersion: v1
kind: Secret
metadata:
  name: consent-secrets
  namespace: cryo-consent
type: Opaque
stringData:
  DATABASE_URL: postgresql://consent:password@postgres:5432/consent
  REDIS_URL: redis://redis:6379
  JWT_SECRET: your-jwt-secret
  HSM_CONNECTION: hsm://hsm-service:5696
  ENCRYPTION_KEY_ID: master-key-001

---
# kubernetes/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: consent-service
  namespace: cryo-consent
  labels:
    app: consent-service
spec:
  replicas: 3
  selector:
    matchLabels:
      app: consent-service
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
  template:
    metadata:
      labels:
        app: consent-service
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "9090"
    spec:
      serviceAccountName: consent-service
      securityContext:
        runAsNonRoot: true
        runAsUser: 1000
        fsGroup: 1000
      containers:
        - name: consent-service
          image: registry.example.com/cryo-consent:1.0.0
          imagePullPolicy: Always
          ports:
            - name: http
              containerPort: 3000
            - name: metrics
              containerPort: 9090
          envFrom:
            - configMapRef:
                name: consent-config
            - secretRef:
                name: consent-secrets
          resources:
            requests:
              cpu: 500m
              memory: 512Mi
            limits:
              cpu: 2000m
              memory: 2Gi
          livenessProbe:
            httpGet:
              path: /health/live
              port: http
            initialDelaySeconds: 30
            periodSeconds: 10
            timeoutSeconds: 5
            failureThreshold: 3
          readinessProbe:
            httpGet:
              path: /health/ready
              port: http
            initialDelaySeconds: 10
            periodSeconds: 5
            timeoutSeconds: 3
            failureThreshold: 3
          volumeMounts:
            - name: tmp
              mountPath: /tmp
            - name: keys
              mountPath: /etc/consent/keys
              readOnly: true
          securityContext:
            allowPrivilegeEscalation: false
            readOnlyRootFilesystem: true
            capabilities:
              drop:
                - ALL
      volumes:
        - name: tmp
          emptyDir: {}
        - name: keys
          secret:
            secretName: consent-keys

---
# kubernetes/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: consent-service
  namespace: cryo-consent
  labels:
    app: consent-service
spec:
  type: ClusterIP
  ports:
    - name: http
      port: 80
      targetPort: http
      protocol: TCP
    - name: metrics
      port: 9090
      targetPort: metrics
      protocol: TCP
  selector:
    app: consent-service

---
# kubernetes/ingress.yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: consent-ingress
  namespace: cryo-consent
  annotations:
    kubernetes.io/ingress.class: nginx
    cert-manager.io/cluster-issuer: letsencrypt-prod
    nginx.ingress.kubernetes.io/ssl-redirect: "true"
    nginx.ingress.kubernetes.io/proxy-body-size: "10m"
    nginx.ingress.kubernetes.io/rate-limit: "100"
spec:
  tls:
    - hosts:
        - consent.example.com
      secretName: consent-tls
  rules:
    - host: consent.example.com
      http:
        paths:
          - path: /
            pathType: Prefix
            backend:
              service:
                name: consent-service
                port:
                  number: 80

---
# kubernetes/hpa.yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: consent-service-hpa
  namespace: cryo-consent
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: consent-service
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
  behavior:
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
        - type: Percent
          value: 10
          periodSeconds: 60
    scaleUp:
      stabilizationWindowSeconds: 0
      policies:
        - type: Percent
          value: 100
          periodSeconds: 15

---
# kubernetes/pdb.yaml
apiVersion: policy/v1
kind: PodDisruptionBudget
metadata:
  name: consent-service-pdb
  namespace: cryo-consent
spec:
  minAvailable: 2
  selector:
    matchLabels:
      app: consent-service
```

---

## 8.5 Testing Strategy

```typescript
// Testing framework setup
describe('ConsentService', () => {
  let consentService: CryoConsentManagementService;
  let testDb: TestDatabase;
  let testPatient: Patient;

  beforeAll(async () => {
    testDb = await TestDatabase.create();
    consentService = createTestConsentService(testDb);
  });

  afterAll(async () => {
    await testDb.cleanup();
  });

  beforeEach(async () => {
    await testDb.truncateAll();
    testPatient = await createTestPatient(testDb);
  });

  describe('createConsent', () => {
    it('should create a valid consent record', async () => {
      const input = createTestConsentInput(testPatient.id);

      const consent = await consentService.createConsent(input, {
        createdBy: 'test-user',
      });

      expect(consent).toBeDefined();
      expect(consent.id).toBeDefined();
      expect(consent.patientId).toBe(testPatient.id);
      expect(consent.validity.status).toBe('DRAFT');
      expect(consent.metadata.version).toBe(1);
    });

    it('should validate required fields', async () => {
      const invalidInput = { patientId: testPatient.id };

      await expect(
        consentService.createConsent(invalidInput as any, { createdBy: 'test-user' })
      ).rejects.toThrow(ConsentValidationError);
    });

    it('should encrypt sensitive data', async () => {
      const input = createTestConsentInput(testPatient.id, {
        decisions: [{
          decisionId: 'test-decision',
          decisionType: 'BINARY',
          question: 'Do you consent to preservation?',
          answer: { type: 'BINARY', binaryValue: true },
        }],
      });

      const consent = await consentService.createConsent(input, {
        createdBy: 'test-user',
      });

      // Verify data is encrypted in database
      const rawData = await testDb.query(
        'SELECT decisions_data FROM consents WHERE id = $1',
        [consent.id]
      );

      // Raw data should be encrypted bytes, not readable JSON
      expect(rawData.rows[0].decisions_data).toBeInstanceOf(Buffer);
      expect(() => JSON.parse(rawData.rows[0].decisions_data.toString())).toThrow();
    });

    it('should create audit log entry', async () => {
      const input = createTestConsentInput(testPatient.id);

      const consent = await consentService.createConsent(input, {
        createdBy: 'test-user',
        ipAddress: '192.168.1.1',
      });

      const auditLogs = await testDb.query(
        'SELECT * FROM audit_log WHERE resource_id = $1',
        [consent.id]
      );

      expect(auditLogs.rows).toHaveLength(1);
      expect(auditLogs.rows[0].event_type).toBe('CONSENT_CREATED');
      expect(auditLogs.rows[0].outcome).toBe('SUCCESS');
    });
  });

  describe('getEffectiveConsent', () => {
    it('should return effective consent for decision type', async () => {
      // Create active consent
      const consent = await createAndActivateConsent(
        consentService,
        testPatient.id,
        {
          category: 'PRESERVATION',
          decisions: [{
            decisionId: 'preservation-1',
            decisionType: 'BINARY',
            question: 'Do you consent to whole body preservation?',
            answer: { type: 'BINARY', binaryValue: true },
          }],
        }
      );

      const result = await consentService.getEffectiveConsent(
        testPatient.id,
        'PRESERVATION',
        {}
      );

      expect(result.found).toBe(true);
      expect(result.consent!.id).toBe(consent.id);
      expect(result.confidence).toBeGreaterThan(0.8);
    });

    it('should return not found when no consent exists', async () => {
      const result = await consentService.getEffectiveConsent(
        testPatient.id,
        'REVIVAL',
        {}
      );

      expect(result.found).toBe(false);
      expect(result.recommendations).toContain(
        expect.stringContaining('explicit consent')
      );
    });

    it('should prefer more recent consent', async () => {
      // Create older consent
      const oldConsent = await createAndActivateConsent(
        consentService,
        testPatient.id,
        { category: 'PRESERVATION' }
      );

      // Wait a bit
      await sleep(100);

      // Create newer consent
      const newConsent = await createAndActivateConsent(
        consentService,
        testPatient.id,
        { category: 'PRESERVATION' }
      );

      const result = await consentService.getEffectiveConsent(
        testPatient.id,
        'PRESERVATION',
        {}
      );

      expect(result.consent!.id).toBe(newConsent.id);
    });
  });

  describe('revokeConsent', () => {
    it('should revoke active consent', async () => {
      const consent = await createAndActivateConsent(
        consentService,
        testPatient.id,
        {}
      );

      const revoked = await consentService.revokeConsent(consent.id, {
        revokedBy: testPatient.id,
        reason: 'Changed mind',
      });

      expect(revoked.validity.status).toBe('REVOKED');
      expect(revoked.validity.revocation).toBeDefined();
      expect(revoked.validity.revocation!.reason).toBe('Changed mind');
    });

    it('should not allow unauthorized revocation', async () => {
      const consent = await createAndActivateConsent(
        consentService,
        testPatient.id,
        {}
      );

      await expect(
        consentService.revokeConsent(consent.id, {
          revokedBy: 'unauthorized-user',
          reason: 'Test',
        })
      ).rejects.toThrow(UnauthorizedError);
    });
  });
});

// Integration tests
describe('Consent API Integration', () => {
  let app: TestApplication;
  let authToken: string;

  beforeAll(async () => {
    app = await TestApplication.create();
    authToken = await app.getTestAuthToken('test-patient');
  });

  afterAll(async () => {
    await app.cleanup();
  });

  describe('POST /api/v1/consents', () => {
    it('should create consent via API', async () => {
      const response = await request(app.server)
        .post('/api/v1/consents')
        .set('Authorization', `Bearer ${authToken}`)
        .send(createTestConsentInput(app.testPatientId));

      expect(response.status).toBe(201);
      expect(response.body.success).toBe(true);
      expect(response.body.data.consent.id).toBeDefined();
    });

    it('should reject unauthenticated requests', async () => {
      const response = await request(app.server)
        .post('/api/v1/consents')
        .send(createTestConsentInput(app.testPatientId));

      expect(response.status).toBe(401);
    });

    it('should validate request body', async () => {
      const response = await request(app.server)
        .post('/api/v1/consents')
        .set('Authorization', `Bearer ${authToken}`)
        .send({ invalid: 'data' });

      expect(response.status).toBe(400);
      expect(response.body.error.code).toBe('VALIDATION_ERROR');
    });
  });

  describe('GraphQL', () => {
    it('should query consent via GraphQL', async () => {
      // Create consent first
      const created = await app.createTestConsent();

      const response = await request(app.server)
        .post('/graphql')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          query: `
            query GetConsent($id: ID!) {
              consent(id: $id) {
                id
                category
                validity {
                  status
                }
              }
            }
          `,
          variables: { id: created.id },
        });

      expect(response.status).toBe(200);
      expect(response.body.data.consent.id).toBe(created.id);
    });
  });
});

// Performance tests
describe('Consent Service Performance', () => {
  it('should handle 100 concurrent consent queries', async () => {
    const service = await createPerformanceTestService();
    const patientId = await createPatientWithConsents(service, 10);

    const start = Date.now();

    const promises = Array(100).fill(null).map(() =>
      service.getEffectiveConsent(patientId, 'PRESERVATION', {})
    );

    const results = await Promise.all(promises);
    const duration = Date.now() - start;

    expect(results.every(r => r.found)).toBe(true);
    expect(duration).toBeLessThan(5000); // 5 seconds for 100 queries
  });

  it('should maintain response time under load', async () => {
    const service = await createPerformanceTestService();
    const responseTimes: number[] = [];

    for (let i = 0; i < 50; i++) {
      const patientId = await createTestPatient(service.db);
      await createAndActivateConsent(service, patientId, {});

      const start = Date.now();
      await service.getEffectiveConsent(patientId, 'PRESERVATION', {});
      responseTimes.push(Date.now() - start);
    }

    const avgResponseTime = responseTimes.reduce((a, b) => a + b) / responseTimes.length;
    const p95 = responseTimes.sort((a, b) => a - b)[Math.floor(responseTimes.length * 0.95)];

    expect(avgResponseTime).toBeLessThan(100); // 100ms average
    expect(p95).toBeLessThan(200); // 200ms p95
  });
});
```

---

## 8.6 Monitoring and Observability

```typescript
// Metrics service
class ConsentMetricsService {
  private registry: PrometheusRegistry;
  private metrics: ConsentMetrics;

  constructor() {
    this.registry = new PrometheusRegistry();
    this.metrics = this.initializeMetrics();
  }

  private initializeMetrics(): ConsentMetrics {
    return {
      // Request metrics
      httpRequestTotal: new Counter({
        name: 'consent_http_requests_total',
        help: 'Total HTTP requests',
        labelNames: ['method', 'path', 'status'],
        registers: [this.registry],
      }),

      httpRequestDuration: new Histogram({
        name: 'consent_http_request_duration_seconds',
        help: 'HTTP request duration in seconds',
        labelNames: ['method', 'path'],
        buckets: [0.01, 0.05, 0.1, 0.25, 0.5, 1, 2.5, 5],
        registers: [this.registry],
      }),

      // Consent metrics
      consentOperations: new Counter({
        name: 'consent_operations_total',
        help: 'Total consent operations',
        labelNames: ['operation', 'category', 'status'],
        registers: [this.registry],
      }),

      activeConsents: new Gauge({
        name: 'consent_active_total',
        help: 'Number of active consents',
        labelNames: ['category'],
        registers: [this.registry],
      }),

      consentDecisionQueries: new Counter({
        name: 'consent_decision_queries_total',
        help: 'Total decision queries',
        labelNames: ['decision_type', 'found'],
        registers: [this.registry],
      }),

      decisionQueryDuration: new Histogram({
        name: 'consent_decision_query_duration_seconds',
        help: 'Decision query duration',
        buckets: [0.01, 0.05, 0.1, 0.25, 0.5, 1],
        registers: [this.registry],
      }),

      // Security metrics
      authenticationAttempts: new Counter({
        name: 'consent_auth_attempts_total',
        help: 'Authentication attempts',
        labelNames: ['method', 'result'],
        registers: [this.registry],
      }),

      authorizationDecisions: new Counter({
        name: 'consent_authz_decisions_total',
        help: 'Authorization decisions',
        labelNames: ['resource', 'action', 'result'],
        registers: [this.registry],
      }),

      securityEvents: new Counter({
        name: 'consent_security_events_total',
        help: 'Security events',
        labelNames: ['type', 'severity'],
        registers: [this.registry],
      }),

      // Integration metrics
      externalSyncOperations: new Counter({
        name: 'consent_external_sync_total',
        help: 'External sync operations',
        labelNames: ['target', 'result'],
        registers: [this.registry],
      }),

      externalSyncDuration: new Histogram({
        name: 'consent_external_sync_duration_seconds',
        help: 'External sync duration',
        labelNames: ['target'],
        buckets: [0.1, 0.5, 1, 2.5, 5, 10],
        registers: [this.registry],
      }),

      // Database metrics
      dbQueryDuration: new Histogram({
        name: 'consent_db_query_duration_seconds',
        help: 'Database query duration',
        labelNames: ['operation'],
        buckets: [0.001, 0.005, 0.01, 0.05, 0.1, 0.5],
        registers: [this.registry],
      }),

      dbConnectionPoolSize: new Gauge({
        name: 'consent_db_pool_size',
        help: 'Database connection pool size',
        labelNames: ['state'],
        registers: [this.registry],
      }),
    };
  }

  // Record HTTP request
  recordRequest(method: string, path: string, status: number, duration: number): void {
    this.metrics.httpRequestTotal.inc({ method, path, status });
    this.metrics.httpRequestDuration.observe({ method, path }, duration);
  }

  // Record consent operation
  recordConsentOperation(operation: string, category: string, status: string): void {
    this.metrics.consentOperations.inc({ operation, category, status });
  }

  // Update active consents gauge
  async updateActiveConsentsGauge(): Promise<void> {
    const counts = await this.getActiveConsentCounts();
    for (const [category, count] of Object.entries(counts)) {
      this.metrics.activeConsents.set({ category }, count);
    }
  }

  // Record decision query
  recordDecisionQuery(decisionType: string, found: boolean, duration: number): void {
    this.metrics.consentDecisionQueries.inc({ decision_type: decisionType, found: String(found) });
    this.metrics.decisionQueryDuration.observe(duration);
  }

  // Get metrics endpoint
  async getMetrics(): Promise<string> {
    return this.registry.metrics();
  }
}

// Health check service
class HealthCheckService {
  private checks: Map<string, HealthCheck> = new Map();

  constructor(services: ServiceContainer) {
    this.registerChecks(services);
  }

  private registerChecks(services: ServiceContainer): void {
    // Database check
    this.checks.set('database', {
      name: 'Database',
      check: async () => {
        const result = await services.database.query('SELECT 1');
        return { healthy: result.rows.length > 0 };
      },
    });

    // Redis check
    this.checks.set('redis', {
      name: 'Redis',
      check: async () => {
        const pong = await services.redis.ping();
        return { healthy: pong === 'PONG' };
      },
    });

    // HSM check
    this.checks.set('hsm', {
      name: 'HSM',
      check: async () => {
        const status = await services.cryptoService.healthCheck();
        return { healthy: status.healthy };
      },
    });

    // External integrations
    this.checks.set('integrations', {
      name: 'External Integrations',
      check: async () => {
        const results = await services.integrationOrchestrator.healthCheck();
        return {
          healthy: Object.values(results).every(r => r.healthy),
          details: results,
        };
      },
    });
  }

  // Liveness check (is the service running?)
  async checkLiveness(): Promise<HealthCheckResult> {
    return {
      status: 'UP',
      timestamp: new Date(),
    };
  }

  // Readiness check (is the service ready to accept traffic?)
  async checkReadiness(): Promise<HealthCheckResult> {
    const results: Record<string, ComponentHealth> = {};
    let allHealthy = true;

    for (const [name, check] of this.checks) {
      try {
        const result = await Promise.race([
          check.check(),
          new Promise<{ healthy: false }>((resolve) =>
            setTimeout(() => resolve({ healthy: false }), 5000)
          ),
        ]);

        results[name] = {
          status: result.healthy ? 'UP' : 'DOWN',
          details: result.details,
        };

        if (!result.healthy) {
          allHealthy = false;
        }
      } catch (error) {
        results[name] = {
          status: 'DOWN',
          error: error.message,
        };
        allHealthy = false;
      }
    }

    return {
      status: allHealthy ? 'UP' : 'DOWN',
      timestamp: new Date(),
      components: results,
    };
  }
}

// Logging configuration
const loggingConfig = {
  level: process.env.LOG_LEVEL || 'info',
  format: winston.format.combine(
    winston.format.timestamp(),
    winston.format.errors({ stack: true }),
    winston.format.json()
  ),
  transports: [
    new winston.transports.Console(),
    new winston.transports.File({
      filename: '/var/log/consent/error.log',
      level: 'error',
    }),
    new winston.transports.File({
      filename: '/var/log/consent/combined.log',
    }),
  ],
  defaultMeta: {
    service: 'consent-service',
    version: process.env.VERSION,
  },
};

const logger = winston.createLogger(loggingConfig);

// Structured logging
function logConsentOperation(
  operation: string,
  consentId: string,
  userId: string,
  result: string,
  duration: number,
  metadata?: Record<string, any>
): void {
  logger.info('Consent operation', {
    operation,
    consentId,
    userId,
    result,
    durationMs: duration,
    ...metadata,
  });
}

function logSecurityEvent(
  eventType: string,
  severity: string,
  userId: string,
  details: Record<string, any>
): void {
  logger.warn('Security event', {
    eventType,
    severity,
    userId,
    ...details,
  });
}
```

---

*Next Chapter: Future Trends - Evolution of consent management and emerging technologies*
