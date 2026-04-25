# 제8장: 구현 및 운영

## 8.1 개요

극저온 시설 관리 시스템의 성공적인 구현은 체계적인 프로젝트 구조, 견고한 데이터베이스 설계, 효율적인 배포 전략을 필요로 합니다. 이 장에서는 실제 구현 방법론과 운영 모범 사례를 상세히 다룹니다.

```typescript
// 구현 아키텍처 개요
const implementationArchitecture = {
  projectStructure: {
    type: 'Monorepo',
    packageManager: 'pnpm',
    buildTool: 'Turborepo'
  },
  backend: {
    runtime: 'Node.js 20 LTS',
    framework: 'Express + Apollo Server',
    orm: 'Prisma',
    database: 'PostgreSQL 15'
  },
  frontend: {
    framework: 'React 18',
    stateManagement: 'Zustand',
    styling: 'Tailwind CSS'
  },
  infrastructure: {
    containerization: 'Docker',
    orchestration: 'Kubernetes',
    monitoring: 'Prometheus + Grafana',
    logging: 'ELK Stack'
  },
  deployment: {
    strategy: 'GitOps',
    cicd: 'GitHub Actions',
    registry: 'Harbor'
  }
};
```

## 8.2 프로젝트 구조

### 8.2.1 모노레포 구조

```typescript
// 프로젝트 디렉토리 구조
const projectStructure = `
cryo-facility-platform/
├── apps/
│   ├── api/                    # 백엔드 API 서버
│   │   ├── src/
│   │   │   ├── modules/        # 도메인 모듈
│   │   │   │   ├── facility/
│   │   │   │   ├── equipment/
│   │   │   │   ├── monitoring/
│   │   │   │   ├── maintenance/
│   │   │   │   └── security/
│   │   │   ├── infrastructure/ # 인프라 계층
│   │   │   │   ├── database/
│   │   │   │   ├── messaging/
│   │   │   │   └── cache/
│   │   │   ├── shared/         # 공유 유틸리티
│   │   │   └── main.ts
│   │   ├── prisma/
│   │   │   └── schema.prisma
│   │   └── package.json
│   │
│   ├── web/                    # 웹 대시보드
│   │   ├── src/
│   │   │   ├── components/
│   │   │   ├── pages/
│   │   │   ├── hooks/
│   │   │   └── stores/
│   │   └── package.json
│   │
│   └── mobile/                 # 모바일 앱
│       ├── src/
│       └── package.json
│
├── packages/
│   ├── types/                  # 공유 타입 정의
│   ├── ui/                     # 공유 UI 컴포넌트
│   ├── utils/                  # 공유 유틸리티
│   └── config/                 # 공유 설정
│
├── infrastructure/
│   ├── docker/                 # Docker 설정
│   ├── kubernetes/             # K8s 매니페스트
│   └── terraform/              # IaC 설정
│
├── docs/                       # 문서
├── scripts/                    # 빌드/배포 스크립트
├── turbo.json                  # Turborepo 설정
├── pnpm-workspace.yaml         # pnpm 워크스페이스
└── package.json
`;

// package.json (루트)
const rootPackageJson = {
  "name": "cryo-facility-platform",
  "private": true,
  "workspaces": [
    "apps/*",
    "packages/*"
  ],
  "scripts": {
    "dev": "turbo run dev",
    "build": "turbo run build",
    "test": "turbo run test",
    "lint": "turbo run lint",
    "db:migrate": "pnpm --filter api prisma migrate dev",
    "db:generate": "pnpm --filter api prisma generate",
    "docker:build": "docker-compose build",
    "docker:up": "docker-compose up -d",
    "docker:down": "docker-compose down"
  },
  "devDependencies": {
    "turbo": "^2.0.0",
    "typescript": "^5.4.0",
    "@types/node": "^20.0.0"
  },
  "engines": {
    "node": ">=20.0.0",
    "pnpm": ">=9.0.0"
  }
};

// turbo.json
const turboConfig = {
  "$schema": "https://turbo.build/schema.json",
  "globalDependencies": [".env"],
  "pipeline": {
    "build": {
      "dependsOn": ["^build"],
      "outputs": ["dist/**", ".next/**"]
    },
    "dev": {
      "cache": false,
      "persistent": true
    },
    "test": {
      "dependsOn": ["build"],
      "outputs": ["coverage/**"]
    },
    "lint": {
      "outputs": []
    }
  }
};
```

### 8.2.2 API 서버 구조

```typescript
// apps/api/src/main.ts
import express from 'express';
import { ApolloServer } from '@apollo/server';
import { expressMiddleware } from '@apollo/server/express4';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import { useServer } from 'graphql-ws/lib/use/ws';
import cors from 'cors';
import helmet from 'helmet';
import compression from 'compression';

import { prisma } from './infrastructure/database/client';
import { schema } from './graphql/schema';
import { createContext } from './graphql/context';
import { createAPIRoutes } from './routes';
import { CryoFacilityWebSocketServer } from './websocket/server';
import { setupMiddleware } from './middleware';
import { logger } from './shared/logger';
import { config } from './config';

async function bootstrap(): Promise<void> {
  const app = express();
  const httpServer = createServer(app);

  // 미들웨어 설정
  app.use(helmet({
    contentSecurityPolicy: config.isProduction ? undefined : false
  }));
  app.use(cors(config.cors));
  app.use(compression());
  app.use(express.json({ limit: '10mb' }));

  // 커스텀 미들웨어
  setupMiddleware(app);

  // REST API 라우트
  app.use('/api/v1', createAPIRoutes());

  // GraphQL 서버
  const apolloServer = new ApolloServer({
    schema,
    introspection: !config.isProduction,
    plugins: [
      {
        async serverWillStart() {
          return {
            async drainServer() {
              await serverCleanup.dispose();
            }
          };
        }
      }
    ]
  });

  await apolloServer.start();

  app.use(
    '/graphql',
    expressMiddleware(apolloServer, {
      context: createContext
    })
  );

  // WebSocket 서버 (GraphQL 구독)
  const wsServer = new WebSocketServer({
    server: httpServer,
    path: '/graphql/ws'
  });

  const serverCleanup = useServer({ schema, context: createContext }, wsServer);

  // 실시간 모니터링 WebSocket
  const monitoringWs = new CryoFacilityWebSocketServer(httpServer, '/ws/monitoring');

  // 헬스 체크
  app.get('/health', (req, res) => {
    res.json({ status: 'healthy', timestamp: new Date().toISOString() });
  });

  // 서버 시작
  httpServer.listen(config.port, () => {
    logger.info(`🚀 서버 시작: http://localhost:${config.port}`);
    logger.info(`📊 GraphQL: http://localhost:${config.port}/graphql`);
    logger.info(`🔌 WebSocket: ws://localhost:${config.port}/ws/monitoring`);
  });

  // 종료 처리
  const shutdown = async () => {
    logger.info('서버 종료 중...');

    httpServer.close();
    await apolloServer.stop();
    await prisma.$disconnect();

    logger.info('서버가 정상적으로 종료되었습니다');
    process.exit(0);
  };

  process.on('SIGTERM', shutdown);
  process.on('SIGINT', shutdown);
}

bootstrap().catch((error) => {
  logger.error('서버 시작 실패:', error);
  process.exit(1);
});
```

## 8.3 데이터베이스 설계

### 8.3.1 Prisma 스키마

```prisma
// apps/api/prisma/schema.prisma

generator client {
  provider = "prisma-client-js"
}

datasource db {
  provider = "postgresql"
  url      = env("DATABASE_URL")
}

// 시설 모델
model Facility {
  id          String         @id @default(uuid())
  name        String
  type        FacilityType
  status      FacilityStatus

  // 위치 정보
  address     String
  city        String
  country     String         @db.Char(2)
  postalCode  String
  latitude    Float?
  longitude   Float?

  // 용량 정보
  totalStorageUnits Int
  maxSpecimens      Int
  currentOccupancy  Int      @default(0)

  // 연락처
  email         String
  phone         String
  emergencyPhone String

  // 관계
  zones           Zone[]
  equipment       Equipment[]
  alerts          Alert[]
  maintenanceTasks MaintenanceTask[]
  audits          Audit[]
  users           UserFacility[]
  certifications  Certification[]

  // 타임스탬프
  createdAt   DateTime  @default(now())
  updatedAt   DateTime  @updatedAt
  deletedAt   DateTime?

  @@index([type])
  @@index([status])
  @@map("facilities")
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
  PLANNING
  CONSTRUCTION
  COMMISSIONING
  OPERATIONAL
  MAINTENANCE
  SUSPENDED
  DECOMMISSIONED
}

// 구역 모델
model Zone {
  id             String        @id @default(uuid())
  facilityId     String
  facility       Facility      @relation(fields: [facilityId], references: [id])

  name           String
  type           ZoneType
  cleanroomClass CleanroomClass?

  // 치수
  length         Float?
  width          Float?
  height         Float?
  dimensionUnit  String        @default("meters")

  // 환경 요구사항
  tempMin        Float?
  tempMax        Float?
  tempTarget     Float?
  humidityMin    Float?
  humidityMax    Float?
  humidityTarget Float?

  // 접근 제어
  requiredClearance AccessLevel @default(STAFF)
  requiresTwoPersonRule Boolean @default(false)
  maxOccupancy   Int?

  // 관계
  equipment      Equipment[]
  sensors        Sensor[]

  createdAt      DateTime      @default(now())
  updatedAt      DateTime      @updatedAt

  @@index([facilityId])
  @@map("zones")
}

enum ZoneType {
  STORAGE
  PROCESSING
  RECEIVING
  DISTRIBUTION
  QUALITY_CONTROL
  ADMINISTRATIVE
  UTILITY
  EMERGENCY_ACCESS
}

enum CleanroomClass {
  ISO_1
  ISO_2
  ISO_3
  ISO_4
  ISO_5
  ISO_6
  ISO_7
  ISO_8
  ISO_9
  NON_CLASSIFIED
}

enum AccessLevel {
  PUBLIC
  STAFF
  AUTHORIZED
  RESTRICTED
  CRITICAL
}

// 장비 모델
model Equipment {
  id           String           @id @default(uuid())
  facilityId   String
  facility     Facility         @relation(fields: [facilityId], references: [id])
  zoneId       String
  zone         Zone             @relation(fields: [zoneId], references: [id])

  name         String
  category     EquipmentCategory
  status       EquipmentStatus  @default(OPERATIONAL)

  // 식별 정보
  manufacturer String
  model        String
  serialNumber String
  assetTag     String?

  // 사양 (JSON)
  specifications Json

  // 설치 정보
  installationDate DateTime
  warrantyExpiration DateTime?

  // 관계
  sensors        Sensor[]
  readings       SensorReading[]
  alerts         Alert[]
  maintenanceTasks MaintenanceTask[]
  commandLogs    EquipmentCommandLog[]

  createdAt    DateTime         @default(now())
  updatedAt    DateTime         @updatedAt
  deletedAt    DateTime?

  @@unique([serialNumber])
  @@index([facilityId])
  @@index([zoneId])
  @@index([category])
  @@index([status])
  @@map("equipment")
}

enum EquipmentCategory {
  LN2_TANK
  MECHANICAL_FREEZER
  CONTROLLED_RATE
  INCUBATOR
  CENTRIFUGE
  MONITORING_SYSTEM
  SAFETY_EQUIPMENT
  HANDLING_EQUIPMENT
  QUALITY_CONTROL
}

enum EquipmentStatus {
  OPERATIONAL
  STANDBY
  MAINTENANCE
  CALIBRATION
  FAULT
  DECOMMISSIONED
  QUARANTINE
}

// 센서 모델
model Sensor {
  id           String       @id @default(uuid())
  equipmentId  String
  equipment    Equipment    @relation(fields: [equipmentId], references: [id])
  zoneId       String?
  zone         Zone?        @relation(fields: [zoneId], references: [id])

  type         SensorType
  model        String
  serialNumber String

  // 측정 범위
  measurementMin Float
  measurementMax Float
  measurementUnit String

  // 정확도
  accuracy     Float
  resolution   Float

  // 알람 임계값
  warningLow   Float?
  warningHigh  Float?
  criticalLow  Float?
  criticalHigh Float?

  // 교정 정보
  lastCalibration DateTime?
  nextCalibration DateTime?
  calibrationInterval Int?  // days

  // 샘플링 설정
  samplingInterval Int      @default(60) // seconds

  // 관계
  readings     SensorReading[]

  createdAt    DateTime     @default(now())
  updatedAt    DateTime     @updatedAt

  @@index([equipmentId])
  @@index([type])
  @@map("sensors")
}

enum SensorType {
  TEMPERATURE
  PRESSURE
  LEVEL
  HUMIDITY
  OXYGEN
  NITROGEN
  DOOR_STATUS
  POWER_STATUS
  VIBRATION
}

// 센서 측정값 모델
model SensorReading {
  id          String    @id @default(uuid())
  sensorId    String
  sensor      Sensor    @relation(fields: [sensorId], references: [id])
  equipmentId String
  equipment   Equipment @relation(fields: [equipmentId], references: [id])

  value       Float
  unit        String
  quality     DataQuality @default(GOOD)
  inSpec      Boolean

  timestamp   DateTime  @default(now())

  @@index([sensorId, timestamp])
  @@index([equipmentId, timestamp])
  @@map("sensor_readings")
}

enum DataQuality {
  GOOD
  UNCERTAIN
  BAD
}

// 알림 모델
model Alert {
  id           String       @id @default(uuid())
  facilityId   String
  facility     Facility     @relation(fields: [facilityId], references: [id])
  equipmentId  String?
  equipment    Equipment?   @relation(fields: [equipmentId], references: [id])

  severity     AlertSeverity
  category     String
  message      String

  // 상태
  status       AlertStatus  @default(ACTIVE)

  // 시간
  triggeredAt  DateTime     @default(now())
  acknowledgedAt DateTime?
  acknowledgedBy String?
  resolvedAt   DateTime?
  resolvedBy   String?
  resolution   String?

  // 메타데이터
  metadata     Json?

  @@index([facilityId, status])
  @@index([severity])
  @@index([triggeredAt])
  @@map("alerts")
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

// 유지보수 작업 모델
model MaintenanceTask {
  id           String              @id @default(uuid())
  facilityId   String
  facility     Facility            @relation(fields: [facilityId], references: [id])
  equipmentId  String
  equipment    Equipment           @relation(fields: [equipmentId], references: [id])

  type         MaintenanceType
  title        String
  description  String
  priority     Priority            @default(MEDIUM)

  // 일정
  scheduledDate DateTime
  dueDate       DateTime?

  // 담당자
  assignedTo   String?
  assignee     User?               @relation(fields: [assignedTo], references: [id])

  // 완료 정보
  status       MaintenanceStatus   @default(SCHEDULED)
  completedAt  DateTime?
  completedBy  String?
  workPerformed String?

  // 부품 사용
  partsUsed    Json?

  createdAt    DateTime            @default(now())
  updatedAt    DateTime            @updatedAt

  @@index([facilityId])
  @@index([equipmentId])
  @@index([status])
  @@index([scheduledDate])
  @@map("maintenance_tasks")
}

enum MaintenanceType {
  PREVENTIVE
  CORRECTIVE
  CALIBRATION
  INSPECTION
}

enum MaintenanceStatus {
  SCHEDULED
  IN_PROGRESS
  COMPLETED
  CANCELLED
  OVERDUE
}

enum Priority {
  LOW
  MEDIUM
  HIGH
  CRITICAL
}

// 사용자 모델
model User {
  id           String         @id @default(uuid())
  email        String         @unique
  passwordHash String

  firstName    String
  lastName     String

  status       UserStatus     @default(ACTIVE)

  // MFA
  mfaEnabled   Boolean        @default(false)
  mfaSecret    String?

  // 관계
  roles        UserRole[]
  facilities   UserFacility[]
  sessions     UserSession[]
  maintenanceTasks MaintenanceTask[]
  auditLogs    AuditLog[]

  createdAt    DateTime       @default(now())
  updatedAt    DateTime       @updatedAt
  lastLoginAt  DateTime?

  @@map("users")
}

enum UserStatus {
  ACTIVE
  INACTIVE
  LOCKED
  PENDING
}

// 사용자-역할 관계
model UserRole {
  id        String   @id @default(uuid())
  userId    String
  user      User     @relation(fields: [userId], references: [id])
  role      Role

  assignedAt DateTime @default(now())
  assignedBy String?

  @@unique([userId, role])
  @@map("user_roles")
}

enum Role {
  ADMIN
  FACILITY_MANAGER
  EQUIPMENT_OPERATOR
  LAB_TECHNICIAN
  MAINTENANCE_TECHNICIAN
  QUALITY_MANAGER
  SAFETY_OFFICER
  AUDITOR
  VIEWER
}

// 사용자-시설 관계
model UserFacility {
  id         String   @id @default(uuid())
  userId     String
  user       User     @relation(fields: [userId], references: [id])
  facilityId String
  facility   Facility @relation(fields: [facilityId], references: [id])

  @@unique([userId, facilityId])
  @@map("user_facilities")
}

// 감사 로그 모델
model AuditLog {
  id          String   @id @default(uuid())
  userId      String?
  user        User?    @relation(fields: [userId], references: [id])

  action      String
  entityType  String?
  entityId    String?

  oldValues   Json?
  newValues   Json?

  ipAddress   String?
  userAgent   String?

  timestamp   DateTime @default(now())

  @@index([userId])
  @@index([action])
  @@index([entityType, entityId])
  @@index([timestamp])
  @@map("audit_logs")
}
```

## 8.4 Kubernetes 배포

### 8.4.1 배포 매니페스트

```yaml
# infrastructure/kubernetes/api-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-facility-api
  namespace: cryo-facility
  labels:
    app: cryo-facility-api
    version: v1
spec:
  replicas: 3
  selector:
    matchLabels:
      app: cryo-facility-api
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
  template:
    metadata:
      labels:
        app: cryo-facility-api
        version: v1
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "9090"
        prometheus.io/path: "/metrics"
    spec:
      serviceAccountName: cryo-facility-api

      # 보안 컨텍스트
      securityContext:
        runAsNonRoot: true
        runAsUser: 1000
        fsGroup: 1000

      # 초기화 컨테이너
      initContainers:
        - name: wait-for-db
          image: busybox:1.36
          command: ['sh', '-c', 'until nc -z postgres-service 5432; do echo waiting for postgres; sleep 2; done;']

        - name: run-migrations
          image: cryo-facility-api:latest
          command: ['npx', 'prisma', 'migrate', 'deploy']
          envFrom:
            - secretRef:
                name: cryo-facility-secrets
            - configMapRef:
                name: cryo-facility-config

      containers:
        - name: api
          image: cryo-facility-api:latest
          imagePullPolicy: Always

          ports:
            - name: http
              containerPort: 3000
            - name: metrics
              containerPort: 9090
            - name: ws
              containerPort: 3001

          # 환경 변수
          envFrom:
            - secretRef:
                name: cryo-facility-secrets
            - configMapRef:
                name: cryo-facility-config

          # 리소스 제한
          resources:
            requests:
              memory: "512Mi"
              cpu: "250m"
            limits:
              memory: "2Gi"
              cpu: "1000m"

          # 헬스 체크
          livenessProbe:
            httpGet:
              path: /health
              port: http
            initialDelaySeconds: 30
            periodSeconds: 10
            timeoutSeconds: 5
            failureThreshold: 3

          readinessProbe:
            httpGet:
              path: /health
              port: http
            initialDelaySeconds: 5
            periodSeconds: 5
            timeoutSeconds: 3
            failureThreshold: 3

          # 보안 컨텍스트
          securityContext:
            allowPrivilegeEscalation: false
            readOnlyRootFilesystem: true
            capabilities:
              drop:
                - ALL

          # 볼륨 마운트
          volumeMounts:
            - name: tmp
              mountPath: /tmp
            - name: cache
              mountPath: /app/.cache

      volumes:
        - name: tmp
          emptyDir: {}
        - name: cache
          emptyDir: {}

      # 토폴로지 분산
      topologySpreadConstraints:
        - maxSkew: 1
          topologyKey: topology.kubernetes.io/zone
          whenUnsatisfiable: ScheduleAnyway
          labelSelector:
            matchLabels:
              app: cryo-facility-api

---
# 서비스
apiVersion: v1
kind: Service
metadata:
  name: cryo-facility-api
  namespace: cryo-facility
spec:
  type: ClusterIP
  selector:
    app: cryo-facility-api
  ports:
    - name: http
      port: 80
      targetPort: 3000
    - name: ws
      port: 3001
      targetPort: 3001

---
# Ingress
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: cryo-facility-api
  namespace: cryo-facility
  annotations:
    kubernetes.io/ingress.class: nginx
    cert-manager.io/cluster-issuer: letsencrypt-prod
    nginx.ingress.kubernetes.io/ssl-redirect: "true"
    nginx.ingress.kubernetes.io/proxy-body-size: "50m"
    nginx.ingress.kubernetes.io/websocket-services: cryo-facility-api
spec:
  tls:
    - hosts:
        - api.cryo-facility.example.com
      secretName: cryo-facility-tls
  rules:
    - host: api.cryo-facility.example.com
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
# HPA (Horizontal Pod Autoscaler)
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: cryo-facility-api
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
# PDB (Pod Disruption Budget)
apiVersion: policy/v1
kind: PodDisruptionBudget
metadata:
  name: cryo-facility-api
  namespace: cryo-facility
spec:
  minAvailable: 2
  selector:
    matchLabels:
      app: cryo-facility-api
```

## 8.5 모니터링 및 관측성

### 8.5.1 메트릭스 서비스

```typescript
import { Registry, Counter, Histogram, Gauge, collectDefaultMetrics } from 'prom-client';

// 메트릭스 레지스트리
const registry = new Registry();

// 기본 메트릭스 수집
collectDefaultMetrics({ register: registry });

// 커스텀 메트릭스
export class CryoFacilityMetrics {
  // HTTP 요청 메트릭스
  private httpRequestsTotal: Counter;
  private httpRequestDuration: Histogram;

  // 장비 메트릭스
  private equipmentStatus: Gauge;
  private sensorReadings: Gauge;
  private alertsActive: Gauge;

  // 비즈니스 메트릭스
  private specimensStored: Gauge;
  private storageCapacity: Gauge;
  private maintenanceBacklog: Gauge;

  // 시스템 메트릭스
  private websocketConnections: Gauge;
  private databaseQueryDuration: Histogram;
  private cacheHitRate: Gauge;

  constructor() {
    this.initializeMetrics();
  }

  private initializeMetrics(): void {
    // HTTP 메트릭스
    this.httpRequestsTotal = new Counter({
      name: 'cryo_http_requests_total',
      help: '총 HTTP 요청 수',
      labelNames: ['method', 'path', 'status'],
      registers: [registry]
    });

    this.httpRequestDuration = new Histogram({
      name: 'cryo_http_request_duration_seconds',
      help: 'HTTP 요청 처리 시간',
      labelNames: ['method', 'path'],
      buckets: [0.01, 0.05, 0.1, 0.5, 1, 2, 5],
      registers: [registry]
    });

    // 장비 메트릭스
    this.equipmentStatus = new Gauge({
      name: 'cryo_equipment_status',
      help: '장비 상태 (1=정상, 0=비정상)',
      labelNames: ['facility_id', 'equipment_id', 'category', 'status'],
      registers: [registry]
    });

    this.sensorReadings = new Gauge({
      name: 'cryo_sensor_reading',
      help: '센서 측정값',
      labelNames: ['facility_id', 'equipment_id', 'sensor_id', 'type', 'unit'],
      registers: [registry]
    });

    this.alertsActive = new Gauge({
      name: 'cryo_alerts_active',
      help: '활성 알림 수',
      labelNames: ['facility_id', 'severity'],
      registers: [registry]
    });

    // 비즈니스 메트릭스
    this.specimensStored = new Gauge({
      name: 'cryo_specimens_stored_total',
      help: '보관 중인 검체 수',
      labelNames: ['facility_id'],
      registers: [registry]
    });

    this.storageCapacity = new Gauge({
      name: 'cryo_storage_capacity_percent',
      help: '저장 용량 사용률',
      labelNames: ['facility_id'],
      registers: [registry]
    });

    this.maintenanceBacklog = new Gauge({
      name: 'cryo_maintenance_backlog',
      help: '미완료 유지보수 작업 수',
      labelNames: ['facility_id', 'priority'],
      registers: [registry]
    });

    // 시스템 메트릭스
    this.websocketConnections = new Gauge({
      name: 'cryo_websocket_connections',
      help: 'WebSocket 연결 수',
      labelNames: ['facility_id'],
      registers: [registry]
    });

    this.databaseQueryDuration = new Histogram({
      name: 'cryo_database_query_duration_seconds',
      help: '데이터베이스 쿼리 실행 시간',
      labelNames: ['operation', 'table'],
      buckets: [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1],
      registers: [registry]
    });

    this.cacheHitRate = new Gauge({
      name: 'cryo_cache_hit_rate',
      help: '캐시 히트율',
      labelNames: ['cache_name'],
      registers: [registry]
    });
  }

  // HTTP 메트릭스 기록
  recordHttpRequest(method: string, path: string, status: number, duration: number): void {
    this.httpRequestsTotal.inc({ method, path, status: status.toString() });
    this.httpRequestDuration.observe({ method, path }, duration);
  }

  // 장비 상태 업데이트
  updateEquipmentStatus(
    facilityId: string,
    equipmentId: string,
    category: string,
    status: string,
    isOperational: boolean
  ): void {
    this.equipmentStatus.set(
      { facility_id: facilityId, equipment_id: equipmentId, category, status },
      isOperational ? 1 : 0
    );
  }

  // 센서 측정값 기록
  recordSensorReading(
    facilityId: string,
    equipmentId: string,
    sensorId: string,
    type: string,
    value: number,
    unit: string
  ): void {
    this.sensorReadings.set(
      { facility_id: facilityId, equipment_id: equipmentId, sensor_id: sensorId, type, unit },
      value
    );
  }

  // 활성 알림 업데이트
  updateActiveAlerts(facilityId: string, severity: string, count: number): void {
    this.alertsActive.set({ facility_id: facilityId, severity }, count);
  }

  // 검체 수 업데이트
  updateSpecimensStored(facilityId: string, count: number): void {
    this.specimensStored.set({ facility_id: facilityId }, count);
  }

  // 저장 용량 업데이트
  updateStorageCapacity(facilityId: string, percentage: number): void {
    this.storageCapacity.set({ facility_id: facilityId }, percentage);
  }

  // 유지보수 백로그 업데이트
  updateMaintenanceBacklog(facilityId: string, priority: string, count: number): void {
    this.maintenanceBacklog.set({ facility_id: facilityId, priority }, count);
  }

  // WebSocket 연결 수 업데이트
  updateWebSocketConnections(facilityId: string, count: number): void {
    this.websocketConnections.set({ facility_id: facilityId }, count);
  }

  // 데이터베이스 쿼리 시간 기록
  recordDatabaseQuery(operation: string, table: string, duration: number): void {
    this.databaseQueryDuration.observe({ operation, table }, duration);
  }

  // 캐시 히트율 업데이트
  updateCacheHitRate(cacheName: string, rate: number): void {
    this.cacheHitRate.set({ cache_name: cacheName }, rate);
  }

  // 메트릭스 엔드포인트
  async getMetrics(): Promise<string> {
    return registry.metrics();
  }
}

// Express 메트릭스 미들웨어
export function metricsMiddleware(metrics: CryoFacilityMetrics) {
  return (req: any, res: any, next: any) => {
    const start = process.hrtime.bigint();

    res.on('finish', () => {
      const duration = Number(process.hrtime.bigint() - start) / 1e9;
      metrics.recordHttpRequest(req.method, req.path, res.statusCode, duration);
    });

    next();
  };
}
```

## 8.6 테스트 전략

### 8.6.1 통합 테스트

```typescript
import { describe, it, expect, beforeAll, afterAll, beforeEach } from 'vitest';
import { PrismaClient } from '@prisma/client';
import supertest from 'supertest';
import { createApp } from '../src/app';
import { generateTestToken } from './helpers/auth';

// 테스트용 Prisma 클라이언트
const prisma = new PrismaClient({
  datasources: {
    db: {
      url: process.env.TEST_DATABASE_URL
    }
  }
});

describe('극저온 시설 API 통합 테스트', () => {
  let app: any;
  let request: supertest.SuperTest<supertest.Test>;
  let authToken: string;
  let testFacilityId: string;

  beforeAll(async () => {
    app = await createApp();
    request = supertest(app);
    authToken = await generateTestToken('admin');

    // 테스트 데이터 생성
    const facility = await prisma.facility.create({
      data: {
        name: '테스트 시설',
        type: 'BIOBANK',
        status: 'OPERATIONAL',
        address: '서울시 강남구',
        city: '서울',
        country: 'KR',
        postalCode: '06000',
        totalStorageUnits: 100,
        maxSpecimens: 10000,
        email: 'test@example.com',
        phone: '+82-10-1234-5678',
        emergencyPhone: '+82-10-1234-5679'
      }
    });
    testFacilityId = facility.id;
  });

  afterAll(async () => {
    await prisma.facility.delete({ where: { id: testFacilityId } });
    await prisma.$disconnect();
  });

  describe('시설 API', () => {
    it('시설 목록을 조회할 수 있어야 한다', async () => {
      const response = await request
        .get('/api/v1/facilities')
        .set('Authorization', `Bearer ${authToken}`)
        .expect(200);

      expect(response.body.data).toBeInstanceOf(Array);
      expect(response.body.pagination).toBeDefined();
    });

    it('시설 상세 정보를 조회할 수 있어야 한다', async () => {
      const response = await request
        .get(`/api/v1/facilities/${testFacilityId}`)
        .set('Authorization', `Bearer ${authToken}`)
        .expect(200);

      expect(response.body.data.id).toBe(testFacilityId);
      expect(response.body.data.name).toBe('테스트 시설');
    });

    it('시설 정보를 업데이트할 수 있어야 한다', async () => {
      const response = await request
        .put(`/api/v1/facilities/${testFacilityId}`)
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          name: '업데이트된 시설명'
        })
        .expect(200);

      expect(response.body.data.name).toBe('업데이트된 시설명');
    });

    it('권한 없이 시설에 접근하면 401을 반환해야 한다', async () => {
      await request
        .get('/api/v1/facilities')
        .expect(401);
    });
  });

  describe('장비 API', () => {
    let testEquipmentId: string;
    let testZoneId: string;

    beforeEach(async () => {
      const zone = await prisma.zone.create({
        data: {
          facilityId: testFacilityId,
          name: '테스트 구역',
          type: 'STORAGE',
          cleanroomClass: 'ISO_7'
        }
      });
      testZoneId = zone.id;

      const equipment = await prisma.equipment.create({
        data: {
          facilityId: testFacilityId,
          zoneId: testZoneId,
          name: '테스트 냉동고',
          category: 'MECHANICAL_FREEZER',
          status: 'OPERATIONAL',
          manufacturer: '테스트 제조사',
          model: 'TEST-001',
          serialNumber: 'SN-TEST-001',
          specifications: { temperatureRange: { min: -80, max: -60 } },
          installationDate: new Date()
        }
      });
      testEquipmentId = equipment.id;
    });

    it('장비 목록을 조회할 수 있어야 한다', async () => {
      const response = await request
        .get(`/api/v1/equipment?facilityId=${testFacilityId}`)
        .set('Authorization', `Bearer ${authToken}`)
        .expect(200);

      expect(response.body.data).toBeInstanceOf(Array);
      expect(response.body.data.length).toBeGreaterThan(0);
    });

    it('장비 상태를 조회할 수 있어야 한다', async () => {
      const response = await request
        .get(`/api/v1/equipment/${testEquipmentId}/status`)
        .set('Authorization', `Bearer ${authToken}`)
        .expect(200);

      expect(response.body.data.status).toBe('OPERATIONAL');
    });

    it('장비 명령을 실행할 수 있어야 한다', async () => {
      const response = await request
        .post(`/api/v1/equipment/${testEquipmentId}/commands`)
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          command: 'set-temperature',
          parameters: {
            setpoint: -70,
            unit: 'celsius'
          }
        })
        .expect(200);

      expect(response.body.data.success).toBe(true);
    });
  });

  describe('알림 API', () => {
    it('활성 알림을 조회할 수 있어야 한다', async () => {
      const response = await request
        .get(`/api/v1/alerts/active?facilityId=${testFacilityId}`)
        .set('Authorization', `Bearer ${authToken}`)
        .expect(200);

      expect(response.body.data).toBeInstanceOf(Array);
      expect(response.body.summary).toBeDefined();
    });
  });
});
```

## 8.7 요약

이 장에서는 극저온 시설 관리 시스템의 구현 및 운영 방법을 다루었습니다:

| 영역 | 기술 스택 | 주요 특징 |
|------|----------|----------|
| 프로젝트 구조 | Monorepo + Turborepo | 코드 공유, 일관된 빌드 |
| 백엔드 | Express + Apollo + Prisma | REST + GraphQL + WebSocket |
| 데이터베이스 | PostgreSQL + Prisma | 타입 안전 ORM, 마이그레이션 |
| 배포 | Kubernetes + Helm | 자동 스케일링, 고가용성 |
| 모니터링 | Prometheus + Grafana | 메트릭스 수집 및 시각화 |
| 테스트 | Vitest + Supertest | 통합 테스트, E2E 테스트 |

핵심 운영 원칙:
- GitOps 기반 배포
- 제로 다운타임 업데이트
- 완전한 관측성
- 자동화된 테스트

다음 장에서는 미래 발전 방향을 살펴봅니다.

---

© 2025 WIA Standards. All rights reserved.
