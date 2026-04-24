# 제8장: 구현

## 프로젝트 구조 및 배포

본 장에서는 WIA Cryo-Consent 시스템의 구현 세부사항을 다룹니다. 프로젝트 구조, 데이터베이스 스키마, 서비스 구현, Kubernetes 배포, 테스트 전략 및 모니터링을 포함합니다.

---

## 8.1 프로젝트 구조

```typescript
// 모노레포 프로젝트 구조
const projectStructure = {
  root: 'wia-cryo-consent/',
  packages: {
    // 핵심 라이브러리
    '@wia/consent-core': {
      path: 'packages/consent-core',
      description: '핵심 동의 데이터 구조 및 비즈니스 로직',
      exports: [
        'ConsentRecord',
        'ConsentDecision',
        'ConsentAuthority',
        'ConsentValidity',
        'ConsentManagementService',
      ],
    },

    '@wia/consent-crypto': {
      path: 'packages/consent-crypto',
      description: '암호화 서비스 및 키 관리',
      exports: [
        'CryptographicService',
        'KeyManagementService',
        'PostQuantumCryptoService',
        'IntegrityService',
      ],
    },

    '@wia/consent-api': {
      path: 'packages/consent-api',
      description: 'REST 및 GraphQL API 구현',
      exports: [
        'ConsentRouter',
        'GraphQLSchema',
        'WebSocketServer',
        'APIMiddleware',
      ],
    },

    '@wia/consent-integration': {
      path: 'packages/consent-integration',
      description: '외부 시스템 통합 어댑터',
      exports: [
        'HealthcareAdapter',
        'LegalAdapter',
        'CryonicsAdapter',
        'IdentityAdapter',
      ],
    },

    '@wia/consent-workflow': {
      path: 'packages/consent-workflow',
      description: '워크플로우 엔진 및 상태 머신',
      exports: [
        'WorkflowEngine',
        'StateManager',
        'DecisionExecutor',
        'ReviewScheduler',
      ],
    },

    '@wia/consent-security': {
      path: 'packages/consent-security',
      description: '보안 서비스 및 접근 제어',
      exports: [
        'AccessControlService',
        'AuditService',
        'ThreatDetectionService',
        'ComplianceMonitor',
      ],
    },

    '@wia/consent-ui': {
      path: 'packages/consent-ui',
      description: 'React UI 컴포넌트',
      exports: [
        'ConsentForm',
        'DecisionWizard',
        'ProxyManager',
        'DocumentViewer',
      ],
    },
  },

  apps: {
    'consent-service': {
      path: 'apps/consent-service',
      description: '메인 백엔드 서비스',
      port: 3000,
    },

    'consent-web': {
      path: 'apps/consent-web',
      description: '환자 웹 포털',
      port: 4000,
    },

    'consent-admin': {
      path: 'apps/consent-admin',
      description: '관리자 대시보드',
      port: 4001,
    },

    'consent-worker': {
      path: 'apps/consent-worker',
      description: '백그라운드 작업 워커',
    },
  },

  infrastructure: {
    kubernetes: 'infrastructure/kubernetes',
    terraform: 'infrastructure/terraform',
    docker: 'infrastructure/docker',
  },

  docs: {
    api: 'docs/api',
    architecture: 'docs/architecture',
    deployment: 'docs/deployment',
  },
};

// package.json 루트 설정
const rootPackageJson = {
  name: 'wia-cryo-consent',
  version: '1.0.0',
  private: true,
  workspaces: [
    'packages/*',
    'apps/*',
  ],
  scripts: {
    'build': 'turbo run build',
    'dev': 'turbo run dev --parallel',
    'test': 'turbo run test',
    'test:e2e': 'turbo run test:e2e',
    'lint': 'turbo run lint',
    'typecheck': 'turbo run typecheck',
    'db:migrate': 'turbo run db:migrate',
    'db:seed': 'turbo run db:seed',
    'deploy:staging': 'turbo run deploy --filter=@wia/*',
    'deploy:prod': 'turbo run deploy:prod --filter=@wia/*',
  },
  devDependencies: {
    'turbo': '^1.10.0',
    'typescript': '^5.0.0',
    'eslint': '^8.40.0',
    'prettier': '^3.0.0',
    'husky': '^8.0.0',
    'lint-staged': '^13.0.0',
  },
};
```

---

## 8.2 데이터베이스 스키마

```typescript
// PostgreSQL 데이터베이스 스키마
const databaseSchema = `
-- 확장 활성화
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- 열거형 타입
CREATE TYPE consent_type AS ENUM (
  'INITIAL', 'UPDATE', 'RENEWAL', 'REVOCATION', 'PROXY', 'EMERGENCY'
);

CREATE TYPE consent_category AS ENUM (
  'PRESERVATION', 'CARE', 'REVIVAL', 'RESEARCH', 'ASSET', 'COMMUNICATION', 'PROXY_AUTHORITY'
);

CREATE TYPE consent_status AS ENUM (
  'DRAFT', 'PENDING_WITNESS', 'PENDING_NOTARIZATION', 'PENDING_REVIEW',
  'ACTIVE', 'SUSPENDED', 'REVOKED', 'EXPIRED', 'SUPERSEDED'
);

CREATE TYPE decision_type AS ENUM (
  'BINARY', 'CHOICE', 'THRESHOLD', 'PREFERENCE', 'CONDITIONAL', 'DELEGATION'
);

CREATE TYPE proxy_type AS ENUM (
  'INDIVIDUAL', 'ORGANIZATION'
);

-- 환자 테이블
CREATE TABLE patients (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  external_id VARCHAR(255) UNIQUE,
  encrypted_name BYTEA NOT NULL,        -- 암호화된 필드
  encrypted_contact BYTEA NOT NULL,     -- 암호화된 필드
  date_of_birth DATE,
  identity_verification_status VARCHAR(50),
  identity_verification_date TIMESTAMP,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  created_by VARCHAR(255),

  -- 암호화 메타데이터
  encryption_key_id VARCHAR(255) NOT NULL,
  encryption_version INTEGER DEFAULT 1
);

-- 동의 테이블
CREATE TABLE consents (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  patient_id UUID NOT NULL REFERENCES patients(id),
  organization_id UUID NOT NULL,
  consent_type consent_type NOT NULL,
  category consent_category NOT NULL,
  subcategories TEXT[],

  -- 암호화된 범위
  encrypted_scope BYTEA NOT NULL,

  -- 유효성
  status consent_status NOT NULL DEFAULT 'DRAFT',
  effective_date TIMESTAMP,
  expiration_date TIMESTAMP,

  -- 메타데이터
  version INTEGER NOT NULL DEFAULT 1,
  previous_version_id UUID REFERENCES consents(id),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  created_by VARCHAR(255),
  updated_by VARCHAR(255),

  -- 암호화 메타데이터
  encryption_key_id VARCHAR(255) NOT NULL,
  encryption_version INTEGER DEFAULT 1,

  -- 무결성
  integrity_hash VARCHAR(512) NOT NULL,
  blockchain_anchor_tx VARCHAR(255)
);

CREATE INDEX idx_consents_patient ON consents(patient_id);
CREATE INDEX idx_consents_status ON consents(status);
CREATE INDEX idx_consents_category ON consents(category);
CREATE INDEX idx_consents_effective ON consents(effective_date);

-- 의사결정 테이블
CREATE TABLE consent_decisions (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,
  decision_type decision_type NOT NULL,

  -- 암호화된 질문 및 답변
  encrypted_question BYTEA NOT NULL,
  encrypted_answer BYTEA NOT NULL,
  encrypted_reasoning BYTEA,
  encrypted_context BYTEA,

  -- 조건
  conditions JSONB,

  -- 메타데이터
  confidence_level FLOAT,
  requires_periodic_review BOOLEAN DEFAULT FALSE,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

  -- 암호화
  encryption_key_id VARCHAR(255) NOT NULL
);

CREATE INDEX idx_decisions_consent ON consent_decisions(consent_id);
CREATE INDEX idx_decisions_type ON consent_decisions(decision_type);

-- 권한 테이블
CREATE TABLE consent_authorities (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  -- 부여자 정보
  grantor_entity_id VARCHAR(255) NOT NULL,
  grantor_entity_type VARCHAR(50) NOT NULL,
  capacity_confirmed BOOLEAN NOT NULL,
  capacity_date TIMESTAMP,

  -- 암호화된 연락처
  encrypted_contact_info BYTEA,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

  encryption_key_id VARCHAR(255) NOT NULL
);

CREATE INDEX idx_authorities_consent ON consent_authorities(consent_id);

-- 증인 테이블
CREATE TABLE consent_witnesses (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  authority_id UUID NOT NULL REFERENCES consent_authorities(id) ON DELETE CASCADE,

  encrypted_name BYTEA NOT NULL,
  role VARCHAR(100),
  encrypted_credentials BYTEA,
  encrypted_contact BYTEA,

  attestation_date TIMESTAMP,
  attestation_statement TEXT,
  signature BYTEA,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

  encryption_key_id VARCHAR(255) NOT NULL
);

CREATE INDEX idx_witnesses_authority ON consent_witnesses(authority_id);

-- 공증 테이블
CREATE TABLE consent_notarizations (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  authority_id UUID NOT NULL REFERENCES consent_authorities(id) ON DELETE CASCADE,

  notary_id VARCHAR(255),
  notary_name VARCHAR(255),
  jurisdiction VARCHAR(100),
  commission_number VARCHAR(100),
  commission_expiration DATE,

  notarization_date TIMESTAMP,
  seal BYTEA,
  signature BYTEA,
  certificate_url TEXT,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_notarizations_authority ON consent_notarizations(authority_id);

-- 대리인 테이블
CREATE TABLE consent_proxies (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  patient_id UUID NOT NULL REFERENCES patients(id),

  proxy_type proxy_type NOT NULL,
  proxy_order INTEGER NOT NULL,

  -- 개인 대리인 (암호화)
  encrypted_individual_info BYTEA,

  -- 기관 대리인 (암호화)
  encrypted_organization_info BYTEA,

  -- 권한 범위
  authority_categories consent_category[],
  specific_decisions TEXT[],
  exclusions TEXT[],
  financial_limit DECIMAL(15,2),

  -- 유효성
  effective_from TIMESTAMP,
  effective_until TIMESTAMP,
  activation_condition TEXT,

  -- 상태
  acceptance_status VARCHAR(50) DEFAULT 'PENDING',
  acceptance_date TIMESTAMP,
  is_active BOOLEAN DEFAULT FALSE,
  activated_at TIMESTAMP,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

  encryption_key_id VARCHAR(255) NOT NULL
);

CREATE INDEX idx_proxies_patient ON consent_proxies(patient_id);
CREATE INDEX idx_proxies_active ON consent_proxies(patient_id, is_active);
CREATE UNIQUE INDEX idx_proxies_order ON consent_proxies(patient_id, proxy_order);

-- 문서 테이블
CREATE TABLE consent_documents (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  document_type VARCHAR(100) NOT NULL,
  title VARCHAR(500) NOT NULL,
  description TEXT,
  language VARCHAR(10) DEFAULT 'ko',

  -- 저장소
  storage_type VARCHAR(50) NOT NULL,
  primary_location TEXT NOT NULL,
  backup_locations TEXT[],

  -- 암호화
  encrypted BOOLEAN DEFAULT TRUE,
  encryption_algorithm VARCHAR(50),
  encryption_key_id VARCHAR(255),

  -- 버전
  version_number VARCHAR(50) NOT NULL,
  version_date TIMESTAMP NOT NULL,
  is_current BOOLEAN DEFAULT TRUE,
  is_latest BOOLEAN DEFAULT TRUE,
  previous_version_id UUID REFERENCES consent_documents(id),

  -- 무결성
  hash_algorithm VARCHAR(50) NOT NULL,
  hash_value VARCHAR(512) NOT NULL,
  additional_hashes JSONB,
  timestamp_date TIMESTAMP,
  timestamp_authority VARCHAR(255),
  blockchain_anchor JSONB,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  created_by VARCHAR(255)
);

CREATE INDEX idx_documents_consent ON consent_documents(consent_id);
CREATE INDEX idx_documents_type ON consent_documents(document_type);
CREATE INDEX idx_documents_hash ON consent_documents(hash_value);

-- 상태 이력 테이블
CREATE TABLE consent_status_history (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  from_status consent_status,
  to_status consent_status NOT NULL,
  change_date TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  changed_by VARCHAR(255) NOT NULL,
  reason TEXT,

  metadata JSONB
);

CREATE INDEX idx_status_history_consent ON consent_status_history(consent_id);
CREATE INDEX idx_status_history_date ON consent_status_history(change_date);

-- 검토 이력 테이블
CREATE TABLE consent_reviews (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  consent_id UUID NOT NULL REFERENCES consents(id) ON DELETE CASCADE,

  scheduled_date TIMESTAMP,
  completed_date TIMESTAMP,
  review_type VARCHAR(50) NOT NULL,
  reviewed_by VARCHAR(255),

  outcome VARCHAR(50),
  modifications JSONB,
  notes TEXT,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_reviews_consent ON consent_reviews(consent_id);
CREATE INDEX idx_reviews_scheduled ON consent_reviews(scheduled_date);

-- 감사 로그 테이블 (별도 스키마)
CREATE SCHEMA audit;

CREATE TABLE audit.consent_audit_log (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),

  event_type VARCHAR(100) NOT NULL,
  event_category VARCHAR(50) NOT NULL,

  actor_id VARCHAR(255) NOT NULL,
  actor_type VARCHAR(50) NOT NULL,
  actor_roles TEXT[],
  ip_address INET,
  user_agent TEXT,

  target_type VARCHAR(50) NOT NULL,
  target_id UUID,

  action VARCHAR(50) NOT NULL,
  result VARCHAR(50) NOT NULL,

  details JSONB,
  sensitivity INTEGER DEFAULT 0,

  -- 무결성 체인
  hash VARCHAR(512) NOT NULL,
  previous_hash VARCHAR(512),
  signature BYTEA,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_audit_actor ON audit.consent_audit_log(actor_id);
CREATE INDEX idx_audit_target ON audit.consent_audit_log(target_id);
CREATE INDEX idx_audit_date ON audit.consent_audit_log(created_at);
CREATE INDEX idx_audit_type ON audit.consent_audit_log(event_type);

-- 키 관리 테이블 (별도 스키마)
CREATE SCHEMA keys;

CREATE TABLE keys.encryption_keys (
  id VARCHAR(255) PRIMARY KEY,

  purpose VARCHAR(50) NOT NULL,
  algorithm VARCHAR(50) NOT NULL,
  version INTEGER NOT NULL,

  status VARCHAR(50) NOT NULL DEFAULT 'ACTIVE',

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  expires_at TIMESTAMP,
  rotated_at TIMESTAMP,
  destroyed_at TIMESTAMP,

  previous_key_id VARCHAR(255) REFERENCES keys.encryption_keys(id),

  -- HSM 참조
  hsm_key_id VARCHAR(255),
  hsm_slot INTEGER
);

CREATE INDEX idx_keys_status ON keys.encryption_keys(status);
CREATE INDEX idx_keys_purpose ON keys.encryption_keys(purpose);
`;

// 데이터베이스 마이그레이션 서비스
class DatabaseMigrationService {
  async runMigrations(): Promise<void> {
    const migrations = await this.loadMigrations();

    for (const migration of migrations) {
      if (!await this.isMigrationApplied(migration.id)) {
        await this.applyMigration(migration);
      }
    }
  }

  private async applyMigration(migration: Migration): Promise<void> {
    const client = await this.pool.connect();

    try {
      await client.query('BEGIN');

      // 마이그레이션 SQL 실행
      await client.query(migration.up);

      // 마이그레이션 적용 기록
      await client.query(
        'INSERT INTO migrations (id, name, applied_at) VALUES ($1, $2, NOW())',
        [migration.id, migration.name]
      );

      await client.query('COMMIT');
    } catch (error) {
      await client.query('ROLLBACK');
      throw error;
    } finally {
      client.release();
    }
  }
}
```

---

## 8.3 서비스 구현

```typescript
// 메인 애플리케이션 서비스
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

  constructor(private config: ApplicationConfig) {
    this.app = express();
    this.httpServer = createServer(this.app);
  }

  async initialize(): Promise<void> {
    // 미들웨어 설정
    this.setupMiddleware();

    // REST API 라우트 설정
    this.setupRESTRoutes();

    // GraphQL 서버 설정
    await this.setupGraphQL();

    // WebSocket 서버 설정
    this.setupWebSocket();

    // 헬스 체크 설정
    this.setupHealthChecks();

    // 에러 핸들링 설정
    this.setupErrorHandling();
  }

  private setupMiddleware(): void {
    // 보안 미들웨어
    this.app.use(helmet());
    this.app.use(cors(this.config.cors));

    // 요청 파싱
    this.app.use(express.json({ limit: '10mb' }));
    this.app.use(express.urlencoded({ extended: true }));

    // 요청 로깅
    this.app.use(requestLoggingMiddleware);

    // 인증
    this.app.use(authenticationMiddleware);

    // 속도 제한
    this.app.use(rateLimitMiddleware);

    // 요청 검증
    this.app.use(requestValidationMiddleware);
  }

  private setupRESTRoutes(): void {
    const apiRouter = express.Router();

    // 동의 라우트
    apiRouter.use('/consents', consentRouter);

    // 환자 라우트
    apiRouter.use('/patients', patientRouter);

    // 대리인 라우트
    apiRouter.use('/proxies', proxyRouter);

    // 문서 라우트
    apiRouter.use('/documents', documentRouter);

    // 감사 라우트
    apiRouter.use('/audit', auditRouter);

    // 버전 프리픽스 추가
    this.app.use('/api/v1', apiRouter);
  }

  private async setupGraphQL(): Promise<void> {
    this.apolloServer = new ApolloServer({
      schema: consentSchema,
      plugins: [
        ApolloServerPluginDrainHttpServer({ httpServer: this.httpServer }),
        loggingPlugin,
        tracingPlugin,
      ],
    });

    await this.apolloServer.start();

    this.app.use(
      '/graphql',
      expressMiddleware(this.apolloServer, {
        context: async ({ req }) => ({
          user: req.user,
          dataSources: this.getDataSources(),
        }),
      })
    );
  }

  private setupWebSocket(): void {
    this.wsServer = new WebSocketServer({
      server: this.httpServer,
      path: '/ws/consent',
    });

    const consentWsHandler = new ConsentWebSocketHandler(this.config.ws);
    consentWsHandler.attach(this.wsServer);
  }

  private setupHealthChecks(): void {
    // 활성 프로브
    this.app.get('/health/live', (req, res) => {
      res.json({ status: 'UP' });
    });

    // 준비 프로브
    this.app.get('/health/ready', async (req, res) => {
      const checks = await this.performHealthChecks();
      const allHealthy = checks.every(c => c.healthy);

      res.status(allHealthy ? 200 : 503).json({
        status: allHealthy ? 'UP' : 'DOWN',
        checks,
      });
    });
  }

  private async performHealthChecks(): Promise<HealthCheck[]> {
    return Promise.all([
      this.checkDatabase(),
      this.checkCache(),
      this.checkMessageQueue(),
      this.checkHSM(),
    ]);
  }

  private setupErrorHandling(): void {
    // 404 핸들러
    this.app.use((req, res) => {
      res.status(404).json({
        error: {
          code: 'NOT_FOUND',
          message: `${req.method} ${req.path}를 찾을 수 없습니다`,
        },
      });
    });

    // 글로벌 에러 핸들러
    this.app.use((err: Error, req: express.Request, res: express.Response, next: express.NextFunction) => {
      console.error('처리되지 않은 에러:', err);

      // 민감한 에러 정보 로깅
      this.logError(err, req);

      // 클라이언트에 안전한 에러 반환
      const statusCode = err instanceof HTTPError ? err.statusCode : 500;
      const errorResponse = {
        error: {
          code: err.name || 'INTERNAL_ERROR',
          message: this.config.isDevelopment
            ? err.message
            : '내부 서버 오류',
          requestId: req.headers['x-request-id'],
        },
      };

      res.status(statusCode).json(errorResponse);
    });
  }

  async start(): Promise<void> {
    await this.initialize();

    this.httpServer.listen(this.config.port, () => {
      console.log(`동의 서비스가 포트 ${this.config.port}에서 실행 중`);
    });
  }

  async stop(): Promise<void> {
    await this.apolloServer.stop();
    this.wsServer.close();
    this.httpServer.close();
  }
}

// 애플리케이션 시작
const app = new ConsentServiceApplication({
  port: process.env.PORT || 3000,
  cors: {
    origin: process.env.CORS_ORIGINS?.split(',') || ['http://localhost:4000'],
  },
  isDevelopment: process.env.NODE_ENV !== 'production',
});

app.start().catch(console.error);
```

---

## 8.4 Kubernetes 배포

```yaml
# 배포 매니페스트
apiVersion: apps/v1
kind: Deployment
metadata:
  name: consent-service
  namespace: wia-consent
  labels:
    app: consent-service
    version: v1
spec:
  replicas: 3
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
  selector:
    matchLabels:
      app: consent-service
  template:
    metadata:
      labels:
        app: consent-service
        version: v1
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "3000"
        prometheus.io/path: "/metrics"
    spec:
      serviceAccountName: consent-service
      securityContext:
        runAsNonRoot: true
        runAsUser: 1000
        fsGroup: 1000

      containers:
        - name: consent-service
          image: wia-consent/consent-service:latest
          imagePullPolicy: Always

          ports:
            - name: http
              containerPort: 3000
              protocol: TCP
            - name: grpc
              containerPort: 3001
              protocol: TCP

          env:
            - name: NODE_ENV
              value: "production"
            - name: PORT
              value: "3000"
            - name: DATABASE_URL
              valueFrom:
                secretKeyRef:
                  name: consent-secrets
                  key: database-url
            - name: REDIS_URL
              valueFrom:
                secretKeyRef:
                  name: consent-secrets
                  key: redis-url
            - name: HSM_CONFIG
              valueFrom:
                secretKeyRef:
                  name: consent-secrets
                  key: hsm-config
            - name: JWT_SECRET
              valueFrom:
                secretKeyRef:
                  name: consent-secrets
                  key: jwt-secret

          resources:
            requests:
              cpu: "500m"
              memory: "512Mi"
            limits:
              cpu: "2000m"
              memory: "2Gi"

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
            - name: config
              mountPath: /app/config
              readOnly: true
            - name: tls-certs
              mountPath: /app/certs
              readOnly: true

      volumes:
        - name: config
          configMap:
            name: consent-config
        - name: tls-certs
          secret:
            secretName: consent-tls

      affinity:
        podAntiAffinity:
          preferredDuringSchedulingIgnoredDuringExecution:
            - weight: 100
              podAffinityTerm:
                labelSelector:
                  matchExpressions:
                    - key: app
                      operator: In
                      values:
                        - consent-service
                topologyKey: kubernetes.io/hostname

      topologySpreadConstraints:
        - maxSkew: 1
          topologyKey: topology.kubernetes.io/zone
          whenUnsatisfiable: ScheduleAnyway
          labelSelector:
            matchLabels:
              app: consent-service

---
# 서비스
apiVersion: v1
kind: Service
metadata:
  name: consent-service
  namespace: wia-consent
  labels:
    app: consent-service
spec:
  type: ClusterIP
  ports:
    - name: http
      port: 80
      targetPort: http
      protocol: TCP
    - name: grpc
      port: 3001
      targetPort: grpc
      protocol: TCP
  selector:
    app: consent-service

---
# 인그레스
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: consent-service
  namespace: wia-consent
  annotations:
    kubernetes.io/ingress.class: nginx
    nginx.ingress.kubernetes.io/ssl-redirect: "true"
    nginx.ingress.kubernetes.io/proxy-body-size: "10m"
    cert-manager.io/cluster-issuer: letsencrypt-prod
spec:
  tls:
    - hosts:
        - consent.wia.org
      secretName: consent-tls
  rules:
    - host: consent.wia.org
      http:
        paths:
          - path: /
            pathType: Prefix
            backend:
              service:
                name: consent-service
                port:
                  name: http

---
# 수평 파드 오토스케일러
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: consent-service
  namespace: wia-consent
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
      stabilizationWindowSeconds: 60
      policies:
        - type: Percent
          value: 100
          periodSeconds: 15

---
# 파드 중단 예산
apiVersion: policy/v1
kind: PodDisruptionBudget
metadata:
  name: consent-service
  namespace: wia-consent
spec:
  minAvailable: 2
  selector:
    matchLabels:
      app: consent-service
```

---

## 8.5 테스트 전략

```typescript
// 단위 테스트
describe('ConsentManagementService', () => {
  let service: ConsentManagementService;
  let mockRepository: jest.Mocked<ConsentRepository>;
  let mockCryptoService: jest.Mocked<CryptographicService>;

  beforeEach(() => {
    mockRepository = createMockRepository();
    mockCryptoService = createMockCryptoService();

    service = new ConsentManagementService({
      repository: mockRepository,
      cryptoService: mockCryptoService,
    });
  });

  describe('createConsent', () => {
    it('유효한 동의를 생성해야 합니다', async () => {
      const input = createValidConsentInput();
      mockRepository.create.mockResolvedValue(createMockConsent(input));

      const result = await service.createConsent(input, { createdBy: 'user-1' });

      expect(result).toBeDefined();
      expect(result.id).toBeDefined();
      expect(result.status).toBe('DRAFT');
      expect(mockRepository.create).toHaveBeenCalledTimes(1);
    });

    it('유효하지 않은 입력을 거부해야 합니다', async () => {
      const invalidInput = { patientId: '' }; // 잘못된 입력

      await expect(
        service.createConsent(invalidInput as any, { createdBy: 'user-1' })
      ).rejects.toThrow(ValidationError);
    });

    it('민감한 필드를 암호화해야 합니다', async () => {
      const input = createValidConsentInput();
      mockRepository.create.mockResolvedValue(createMockConsent(input));

      await service.createConsent(input, { createdBy: 'user-1' });

      expect(mockCryptoService.encrypt).toHaveBeenCalled();
    });
  });

  describe('getEffectiveConsent', () => {
    it('가장 관련성 높은 동의를 반환해야 합니다', async () => {
      const consents = [
        createMockConsent({ category: 'PRESERVATION', version: 2 }),
        createMockConsent({ category: 'PRESERVATION', version: 1 }),
      ];
      mockRepository.findByPatient.mockResolvedValue(consents);

      const result = await service.getEffectiveConsent(
        'patient-1',
        'PRESERVATION',
        {}
      );

      expect(result.consent?.metadata.version).toBe(2);
    });

    it('동의가 없으면 null을 반환해야 합니다', async () => {
      mockRepository.findByPatient.mockResolvedValue([]);

      const result = await service.getEffectiveConsent(
        'patient-1',
        'PRESERVATION',
        {}
      );

      expect(result.found).toBe(false);
    });
  });
});

// 통합 테스트
describe('Consent API Integration', () => {
  let app: TestApplication;
  let authToken: string;

  beforeAll(async () => {
    app = await createTestApplication();
    authToken = await getTestAuthToken(app, 'test-user');
  });

  afterAll(async () => {
    await app.close();
  });

  describe('POST /api/v1/consents', () => {
    it('새 동의를 생성하고 201을 반환해야 합니다', async () => {
      const input = createValidConsentInput();

      const response = await request(app.getHttpServer())
        .post('/api/v1/consents')
        .set('Authorization', `Bearer ${authToken}`)
        .send(input)
        .expect(201);

      expect(response.body.success).toBe(true);
      expect(response.body.data.consent.id).toBeDefined();
    });

    it('인증 없이 401을 반환해야 합니다', async () => {
      const input = createValidConsentInput();

      await request(app.getHttpServer())
        .post('/api/v1/consents')
        .send(input)
        .expect(401);
    });

    it('잘못된 입력에 대해 400을 반환해야 합니다', async () => {
      const invalidInput = { patientId: '' };

      const response = await request(app.getHttpServer())
        .post('/api/v1/consents')
        .set('Authorization', `Bearer ${authToken}`)
        .send(invalidInput)
        .expect(400);

      expect(response.body.error.code).toBe('VALIDATION_ERROR');
    });
  });

  describe('GET /api/v1/consents/:id', () => {
    it('요청자에게 접근 권한이 있으면 동의를 반환해야 합니다', async () => {
      // 동의 생성
      const created = await createTestConsent(app, authToken);

      const response = await request(app.getHttpServer())
        .get(`/api/v1/consents/${created.id}`)
        .set('Authorization', `Bearer ${authToken}`)
        .expect(200);

      expect(response.body.data.consent.id).toBe(created.id);
    });

    it('권한이 없으면 403을 반환해야 합니다', async () => {
      const otherUserToken = await getTestAuthToken(app, 'other-user');
      const created = await createTestConsent(app, authToken);

      await request(app.getHttpServer())
        .get(`/api/v1/consents/${created.id}`)
        .set('Authorization', `Bearer ${otherUserToken}`)
        .expect(403);
    });
  });
});

// 성능 테스트
describe('Consent Service Performance', () => {
  let app: TestApplication;

  beforeAll(async () => {
    app = await createTestApplication();
  });

  it('대량 동의 조회를 100ms 이내에 처리해야 합니다', async () => {
    const startTime = Date.now();

    await app.get('/api/v1/consents')
      .query({ limit: 100 })
      .set('Authorization', `Bearer ${authToken}`);

    const duration = Date.now() - startTime;
    expect(duration).toBeLessThan(100);
  });

  it('동시 요청을 적절히 처리해야 합니다', async () => {
    const concurrentRequests = 50;
    const requests = Array(concurrentRequests)
      .fill(null)
      .map(() =>
        request(app.getHttpServer())
          .get('/api/v1/consents')
          .set('Authorization', `Bearer ${authToken}`)
      );

    const results = await Promise.all(requests);
    const successCount = results.filter(r => r.status === 200).length;

    expect(successCount).toBe(concurrentRequests);
  });
});
```

---

## 8.6 모니터링

```typescript
// Prometheus 메트릭 설정
import { Counter, Histogram, Gauge, Registry } from 'prom-client';

class MetricsService {
  private registry: Registry;

  // 카운터
  private consentCreatedCounter: Counter;
  private consentUpdatedCounter: Counter;
  private consentRevokedCounter: Counter;
  private apiRequestCounter: Counter;
  private apiErrorCounter: Counter;

  // 히스토그램
  private apiLatencyHistogram: Histogram;
  private dbQueryHistogram: Histogram;
  private cryptoOperationHistogram: Histogram;

  // 게이지
  private activeConsentsGauge: Gauge;
  private pendingReviewsGauge: Gauge;
  private activeSessionsGauge: Gauge;

  constructor() {
    this.registry = new Registry();
    this.initializeMetrics();
  }

  private initializeMetrics(): void {
    // 동의 카운터
    this.consentCreatedCounter = new Counter({
      name: 'consent_created_total',
      help: '생성된 동의 총 수',
      labelNames: ['category', 'type'],
      registers: [this.registry],
    });

    this.consentUpdatedCounter = new Counter({
      name: 'consent_updated_total',
      help: '업데이트된 동의 총 수',
      labelNames: ['category'],
      registers: [this.registry],
    });

    this.consentRevokedCounter = new Counter({
      name: 'consent_revoked_total',
      help: '철회된 동의 총 수',
      labelNames: ['category', 'reason'],
      registers: [this.registry],
    });

    // API 카운터
    this.apiRequestCounter = new Counter({
      name: 'api_requests_total',
      help: 'API 요청 총 수',
      labelNames: ['method', 'path', 'status'],
      registers: [this.registry],
    });

    this.apiErrorCounter = new Counter({
      name: 'api_errors_total',
      help: 'API 오류 총 수',
      labelNames: ['method', 'path', 'error_type'],
      registers: [this.registry],
    });

    // 지연 시간 히스토그램
    this.apiLatencyHistogram = new Histogram({
      name: 'api_request_duration_seconds',
      help: 'API 요청 지연 시간 (초)',
      labelNames: ['method', 'path'],
      buckets: [0.01, 0.05, 0.1, 0.5, 1, 2, 5],
      registers: [this.registry],
    });

    this.dbQueryHistogram = new Histogram({
      name: 'db_query_duration_seconds',
      help: '데이터베이스 쿼리 지연 시간 (초)',
      labelNames: ['operation', 'table'],
      buckets: [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1],
      registers: [this.registry],
    });

    this.cryptoOperationHistogram = new Histogram({
      name: 'crypto_operation_duration_seconds',
      help: '암호화 작업 지연 시간 (초)',
      labelNames: ['operation'],
      buckets: [0.001, 0.005, 0.01, 0.05, 0.1],
      registers: [this.registry],
    });

    // 게이지
    this.activeConsentsGauge = new Gauge({
      name: 'active_consents',
      help: '현재 활성 동의 수',
      labelNames: ['category'],
      registers: [this.registry],
    });

    this.pendingReviewsGauge = new Gauge({
      name: 'pending_reviews',
      help: '보류 중인 검토 수',
      registers: [this.registry],
    });

    this.activeSessionsGauge = new Gauge({
      name: 'active_sessions',
      help: '현재 활성 세션 수',
      registers: [this.registry],
    });
  }

  // 메트릭 기록 메서드
  recordConsentCreated(category: string, type: string): void {
    this.consentCreatedCounter.inc({ category, type });
  }

  recordApiRequest(method: string, path: string, status: number, duration: number): void {
    this.apiRequestCounter.inc({ method, path, status: String(status) });
    this.apiLatencyHistogram.observe({ method, path }, duration);
  }

  recordDbQuery(operation: string, table: string, duration: number): void {
    this.dbQueryHistogram.observe({ operation, table }, duration);
  }

  recordCryptoOperation(operation: string, duration: number): void {
    this.cryptoOperationHistogram.observe({ operation }, duration);
  }

  updateActiveConsents(category: string, count: number): void {
    this.activeConsentsGauge.set({ category }, count);
  }

  // 메트릭 내보내기
  async getMetrics(): Promise<string> {
    return this.registry.metrics();
  }
}

// 알림 규칙
const alertRules = `
groups:
  - name: consent-service-alerts
    rules:
      # 높은 오류율
      - alert: HighErrorRate
        expr: |
          sum(rate(api_errors_total[5m])) /
          sum(rate(api_requests_total[5m])) > 0.05
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: "높은 API 오류율 감지"
          description: "지난 5분간 오류율이 5%를 초과했습니다"

      # 느린 응답
      - alert: SlowResponseTime
        expr: |
          histogram_quantile(0.95, rate(api_request_duration_seconds_bucket[5m])) > 1
        for: 10m
        labels:
          severity: warning
        annotations:
          summary: "느린 API 응답 시간"
          description: "95번째 백분위 응답 시간이 1초를 초과합니다"

      # 높은 메모리 사용
      - alert: HighMemoryUsage
        expr: |
          container_memory_usage_bytes{container="consent-service"} /
          container_spec_memory_limit_bytes{container="consent-service"} > 0.9
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: "높은 메모리 사용량"
          description: "컨테이너 메모리 사용량이 90%를 초과합니다"

      # 데이터베이스 연결 풀 고갈
      - alert: DatabaseConnectionPoolExhausted
        expr: |
          db_pool_connections_idle{service="consent-service"} < 5
        for: 2m
        labels:
          severity: critical
        annotations:
          summary: "데이터베이스 연결 풀 고갈"
          description: "사용 가능한 데이터베이스 연결이 5개 미만입니다"

      # 보류 검토 백로그
      - alert: HighPendingReviews
        expr: pending_reviews > 100
        for: 1h
        labels:
          severity: warning
        annotations:
          summary: "높은 보류 검토 수"
          description: "보류 중인 동의 검토가 100개를 초과합니다"

      # 보안 이벤트
      - alert: SecurityIncident
        expr: |
          increase(security_events_total{severity="critical"}[15m]) > 0
        for: 1m
        labels:
          severity: critical
        annotations:
          summary: "보안 사고 감지"
          description: "중요 보안 이벤트가 지난 15분간 발생했습니다"
`;
```

---

*다음 장: 미래 트렌드 - 동의 관리의 진화*
