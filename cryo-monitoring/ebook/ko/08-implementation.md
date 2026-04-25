# 08. 구현 및 배포
## Implementation and Deployment

**Version**: 1.0.0
**Last Updated**: 2026-01-11

---

## 목차

1. [구현 개요](#구현-개요)
2. [Docker 컨테이너화](#docker-컨테이너화)
3. [Kubernetes 배포](#kubernetes-배포)
4. [CI/CD 파이프라인](#cicd-파이프라인)
5. [성능 최적화](#성능-최적화)
6. [모니터링 및 관찰성](#모니터링-및-관찰성)
7. [백업 및 복구](#백업-및-복구)
8. [한국 클라우드 환경](#한국-클라우드-환경)

---

## 구현 개요

### 시스템 아키텍처

```typescript
/**
 * WIA Cryo Monitoring 시스템 아키텍처
 */
interface SystemArchitecture {
  // 프론트엔드
  frontend: {
    framework: "React" | "Next.js" | "Vue.js";
    frameworkKr: string;
    language: "TypeScript";
    stateManagement: "Redux" | "Zustand" | "Recoil";
    styling: "Tailwind CSS" | "Material-UI";
    frontendKr: string;
  };

  // 백엔드
  backend: {
    runtime: "Node.js" | "Deno" | "Bun";
    runtimeKr: string;
    framework: "Express" | "Fastify" | "NestJS";
    language: "TypeScript";
    apiStyle: ("REST" | "GraphQL" | "WebSocket")[];
    backendKr: string;
  };

  // 데이터베이스
  database: {
    primary: {
      type: "PostgreSQL" | "MySQL";
      typeKr: string;
      extension: "TimescaleDB" | "None";
      version: string;
      primaryKr: string;
    };
    cache: {
      type: "Redis" | "Memcached";
      typeKr: string;
      version: string;
      cacheKr: string;
    };
    timeSeries: {
      type: "InfluxDB" | "TimescaleDB";
      typeKr: string;
      version: string;
      timeSeriesKr: string;
    };
  };

  // 메시지 큐
  messageQueue: {
    type: "RabbitMQ" | "Apache Kafka" | "Redis Pub/Sub";
    typeKr: string;
    useCases: string[];
    useCasesKr: string[];
  };

  // 인프라
  infrastructure: {
    containerization: "Docker";
    orchestration: "Kubernetes" | "Docker Swarm";
    cloudProvider: "AWS" | "Azure" | "GCP" | "Naver Cloud" | "KT Cloud";
    cloudProviderKr: string;
    infraKr: string;
  };

  archKr: string;
}

const architecture: SystemArchitecture = {
  frontend: {
    framework: "Next.js",
    frameworkKr: "Next.js 14 (App Router)",
    language: "TypeScript",
    stateManagement: "Zustand",
    styling: "Tailwind CSS",
    frontendKr: "Next.js 14 + TypeScript + Tailwind CSS"
  },

  backend: {
    runtime: "Node.js",
    runtimeKr: "Node.js 20 LTS",
    framework: "NestJS",
    language: "TypeScript",
    apiStyle: ["REST", "GraphQL", "WebSocket"],
    backendKr: "NestJS + TypeScript (REST, GraphQL, WebSocket 지원)"
  },

  database: {
    primary: {
      type: "PostgreSQL",
      typeKr: "PostgreSQL",
      extension: "TimescaleDB",
      version: "16",
      primaryKr: "PostgreSQL 16 + TimescaleDB (시계열 데이터)"
    },
    cache: {
      type: "Redis",
      typeKr: "Redis",
      version: "7",
      cacheKr: "Redis 7 (캐싱, 세션, Pub/Sub)"
    },
    timeSeries: {
      type: "TimescaleDB",
      typeKr: "TimescaleDB",
      version: "2.13",
      timeSeriesKr: "TimescaleDB 2.13 (센서 데이터 최적화)"
    }
  },

  messageQueue: {
    type: "RabbitMQ",
    typeKr: "RabbitMQ",
    useCases: [
      "Alert distribution",
      "Sensor data ingestion",
      "Background job processing"
    ],
    useCasesKr: [
      "알림 분배",
      "센서 데이터 수집",
      "백그라운드 작업 처리"
    ]
  },

  infrastructure: {
    containerization: "Docker",
    orchestration: "Kubernetes",
    cloudProvider: "Naver Cloud",
    cloudProviderKr: "네이버 클라우드 플랫폼 (한국 리전)",
    infraKr: "Docker + Kubernetes + 네이버 클라우드"
  },

  archKr: "마이크로서비스 아키텍처 (Next.js + NestJS + PostgreSQL + Redis + RabbitMQ)"
};
```

---

## Docker 컨테이너화

### Dockerfile

```dockerfile
# WIA Cryo Monitoring - Backend Dockerfile
# Node.js 백엔드 서비스

# 멀티 스테이지 빌드
FROM node:20-alpine AS builder

# 작업 디렉토리
WORKDIR /app

# 패키지 파일 복사
COPY package*.json ./
COPY tsconfig.json ./

# 의존성 설치
RUN npm ci --only=production && \
    npm cache clean --force

# 소스 코드 복사
COPY src ./src

# TypeScript 빌드
RUN npm run build

# 프로덕션 이미지
FROM node:20-alpine

# 메타데이터
LABEL maintainer="WIA <support@wia.org>" \
      version="1.0.0" \
      description="WIA Cryo Monitoring Backend Service" \
      org.opencontainers.image.title="WIA Cryo Monitor" \
      org.opencontainers.image.description="극저온 생물자원 모니터링 백엔드" \
      org.opencontainers.image.vendor="WIA"

# 비root 사용자 생성
RUN addgroup -g 1001 -S nodejs && \
    adduser -S nodejs -u 1001

# 작업 디렉토리
WORKDIR /app

# 빌더 스테이지에서 파일 복사
COPY --from=builder --chown=nodejs:nodejs /app/node_modules ./node_modules
COPY --from=builder --chown=nodejs:nodejs /app/dist ./dist
COPY --from=builder --chown=nodejs:nodejs /app/package*.json ./

# 사용자 전환
USER nodejs

# 포트 노출
EXPOSE 3000

# 헬스체크
HEALTHCHECK --interval=30s --timeout=3s --start-period=40s --retries=3 \
  CMD node -e "require('http').get('http://localhost:3000/health', (r) => {process.exit(r.statusCode === 200 ? 0 : 1)})"

# 시작 명령
CMD ["node", "dist/main.js"]
```

### Docker Compose

```yaml
# docker-compose.yml
# WIA Cryo Monitoring 전체 스택

version: '3.8'

services:
  # PostgreSQL + TimescaleDB
  postgres:
    image: timescale/timescaledb:2.13.0-pg16
    container_name: cryo-postgres
    environment:
      POSTGRES_DB: cryo_monitor
      POSTGRES_USER: cryo_user
      POSTGRES_PASSWORD: ${POSTGRES_PASSWORD}
      POSTGRES_INITDB_ARGS: "-E UTF8 --locale=ko_KR.UTF-8"
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./init-db.sql:/docker-entrypoint-initdb.d/init.sql
    networks:
      - cryo-network
    restart: unless-stopped
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U cryo_user -d cryo_monitor"]
      interval: 10s
      timeout: 5s
      retries: 5

  # Redis
  redis:
    image: redis:7-alpine
    container_name: cryo-redis
    command: redis-server --appendonly yes --requirepass ${REDIS_PASSWORD}
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data
    networks:
      - cryo-network
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "redis-cli", "ping"]
      interval: 10s
      timeout: 3s
      retries: 5

  # RabbitMQ
  rabbitmq:
    image: rabbitmq:3.12-management-alpine
    container_name: cryo-rabbitmq
    environment:
      RABBITMQ_DEFAULT_USER: cryo_user
      RABBITMQ_DEFAULT_PASS: ${RABBITMQ_PASSWORD}
      RABBITMQ_DEFAULT_VHOST: /cryo
    ports:
      - "5672:5672"     # AMQP
      - "15672:15672"   # Management UI
    volumes:
      - rabbitmq_data:/var/lib/rabbitmq
    networks:
      - cryo-network
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "rabbitmq-diagnostics", "ping"]
      interval: 30s
      timeout: 10s
      retries: 5

  # Backend API
  backend:
    build:
      context: ./backend
      dockerfile: Dockerfile
    container_name: cryo-backend
    environment:
      NODE_ENV: production
      PORT: 3000
      DATABASE_URL: postgresql://cryo_user:${POSTGRES_PASSWORD}@postgres:5432/cryo_monitor
      REDIS_URL: redis://:${REDIS_PASSWORD}@redis:6379
      RABBITMQ_URL: amqp://cryo_user:${RABBITMQ_PASSWORD}@rabbitmq:5672/cryo
      JWT_SECRET: ${JWT_SECRET}
      ENCRYPTION_KEY: ${ENCRYPTION_KEY}
    ports:
      - "3000:3000"
    depends_on:
      postgres:
        condition: service_healthy
      redis:
        condition: service_healthy
      rabbitmq:
        condition: service_healthy
    networks:
      - cryo-network
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:3000/health"]
      interval: 30s
      timeout: 3s
      retries: 3

  # Frontend
  frontend:
    build:
      context: ./frontend
      dockerfile: Dockerfile
    container_name: cryo-frontend
    environment:
      NEXT_PUBLIC_API_URL: http://backend:3000
      NEXT_PUBLIC_WS_URL: ws://backend:3000
    ports:
      - "80:3000"
    depends_on:
      - backend
    networks:
      - cryo-network
    restart: unless-stopped

  # Nginx (리버스 프록시)
  nginx:
    image: nginx:1.25-alpine
    container_name: cryo-nginx
    ports:
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf:ro
      - ./ssl:/etc/nginx/ssl:ro
    depends_on:
      - frontend
      - backend
    networks:
      - cryo-network
    restart: unless-stopped

volumes:
  postgres_data:
    name: cryo_postgres_data
  redis_data:
    name: cryo_redis_data
  rabbitmq_data:
    name: cryo_rabbitmq_data

networks:
  cryo-network:
    name: cryo_network
    driver: bridge
```

---

## Kubernetes 배포

### Kubernetes 매니페스트

```yaml
# k8s/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: cryo-monitoring
  labels:
    name: cryo-monitoring
    app.kubernetes.io/name: cryo-monitoring
    app.kubernetes.io/version: "1.0.0"

---
# k8s/configmap.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: cryo-config
  namespace: cryo-monitoring
data:
  NODE_ENV: "production"
  PORT: "3000"
  LOG_LEVEL: "info"
  TZ: "Asia/Seoul"

---
# k8s/secret.yaml
apiVersion: v1
kind: Secret
metadata:
  name: cryo-secrets
  namespace: cryo-monitoring
type: Opaque
stringData:
  POSTGRES_PASSWORD: "your-postgres-password"
  REDIS_PASSWORD: "your-redis-password"
  RABBITMQ_PASSWORD: "your-rabbitmq-password"
  JWT_SECRET: "your-jwt-secret"
  ENCRYPTION_KEY: "your-encryption-key"

---
# k8s/postgres-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: postgres
  namespace: cryo-monitoring
spec:
  replicas: 1
  selector:
    matchLabels:
      app: postgres
  template:
    metadata:
      labels:
        app: postgres
    spec:
      containers:
      - name: postgres
        image: timescale/timescaledb:2.13.0-pg16
        env:
        - name: POSTGRES_DB
          value: "cryo_monitor"
        - name: POSTGRES_USER
          value: "cryo_user"
        - name: POSTGRES_PASSWORD
          valueFrom:
            secretKeyRef:
              name: cryo-secrets
              key: POSTGRES_PASSWORD
        ports:
        - containerPort: 5432
        volumeMounts:
        - name: postgres-storage
          mountPath: /var/lib/postgresql/data
        resources:
          requests:
            memory: "1Gi"
            cpu: "500m"
          limits:
            memory: "2Gi"
            cpu: "1000m"
        livenessProbe:
          exec:
            command:
            - pg_isready
            - -U
            - cryo_user
            - -d
            - cryo_monitor
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          exec:
            command:
            - pg_isready
            - -U
            - cryo_user
            - -d
            - cryo_monitor
          initialDelaySeconds: 5
          periodSeconds: 5
      volumes:
      - name: postgres-storage
        persistentVolumeClaim:
          claimName: postgres-pvc

---
# k8s/postgres-service.yaml
apiVersion: v1
kind: Service
metadata:
  name: postgres
  namespace: cryo-monitoring
spec:
  selector:
    app: postgres
  ports:
  - port: 5432
    targetPort: 5432
  type: ClusterIP

---
# k8s/postgres-pvc.yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: postgres-pvc
  namespace: cryo-monitoring
spec:
  accessModes:
  - ReadWriteOnce
  resources:
    requests:
      storage: 100Gi
  storageClassName: standard

---
# k8s/backend-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: backend
  namespace: cryo-monitoring
  labels:
    app: backend
spec:
  replicas: 3
  selector:
    matchLabels:
      app: backend
  template:
    metadata:
      labels:
        app: backend
    spec:
      containers:
      - name: backend
        image: your-registry.com/cryo-backend:1.0.0
        envFrom:
        - configMapRef:
            name: cryo-config
        env:
        - name: DATABASE_URL
          value: "postgresql://cryo_user:$(POSTGRES_PASSWORD)@postgres:5432/cryo_monitor"
        - name: REDIS_URL
          value: "redis://:$(REDIS_PASSWORD)@redis:6379"
        - name: POSTGRES_PASSWORD
          valueFrom:
            secretKeyRef:
              name: cryo-secrets
              key: POSTGRES_PASSWORD
        - name: REDIS_PASSWORD
          valueFrom:
            secretKeyRef:
              name: cryo-secrets
              key: REDIS_PASSWORD
        - name: JWT_SECRET
          valueFrom:
            secretKeyRef:
              name: cryo-secrets
              key: JWT_SECRET
        ports:
        - containerPort: 3000
        resources:
          requests:
            memory: "512Mi"
            cpu: "250m"
          limits:
            memory: "1Gi"
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

---
# k8s/backend-service.yaml
apiVersion: v1
kind: Service
metadata:
  name: backend
  namespace: cryo-monitoring
spec:
  selector:
    app: backend
  ports:
  - port: 3000
    targetPort: 3000
  type: ClusterIP

---
# k8s/backend-hpa.yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: backend-hpa
  namespace: cryo-monitoring
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: backend
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
# k8s/ingress.yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: cryo-ingress
  namespace: cryo-monitoring
  annotations:
    nginx.ingress.kubernetes.io/ssl-redirect: "true"
    nginx.ingress.kubernetes.io/force-ssl-redirect: "true"
    cert-manager.io/cluster-issuer: "letsencrypt-prod"
spec:
  ingressClassName: nginx
  tls:
  - hosts:
    - api.cryo-monitor.wia.org
    - app.cryo-monitor.wia.org
    secretName: cryo-tls-secret
  rules:
  - host: api.cryo-monitor.wia.org
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: backend
            port:
              number: 3000
  - host: app.cryo-monitor.wia.org
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: frontend
            port:
              number: 3000
```

---

## CI/CD 파이프라인

### GitHub Actions 워크플로우

```yaml
# .github/workflows/ci-cd.yml
name: CI/CD Pipeline

on:
  push:
    branches:
      - main
      - develop
  pull_request:
    branches:
      - main
      - develop

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: ${{ github.repository }}

jobs:
  # 테스트 작업
  test:
    name: 테스트 및 린트
    runs-on: ubuntu-latest

    steps:
    - name: 체크아웃
      uses: actions/checkout@v4

    - name: Node.js 설정
      uses: actions/setup-node@v4
      with:
        node-version: '20'
        cache: 'npm'

    - name: 의존성 설치
      run: npm ci

    - name: 린트
      run: npm run lint

    - name: 타입 체크
      run: npm run type-check

    - name: 유닛 테스트
      run: npm run test:unit

    - name: 통합 테스트
      run: npm run test:integration

    - name: 커버리지 업로드
      uses: codecov/codecov-action@v3
      with:
        token: ${{ secrets.CODECOV_TOKEN }}
        files: ./coverage/coverage-final.json

  # 보안 스캔
  security:
    name: 보안 스캔
    runs-on: ubuntu-latest
    needs: test

    steps:
    - name: 체크아웃
      uses: actions/checkout@v4

    - name: 의존성 취약점 스캔
      run: npm audit --audit-level=high

    - name: Snyk 보안 스캔
      uses: snyk/actions/node@master
      env:
        SNYK_TOKEN: ${{ secrets.SNYK_TOKEN }}

  # 빌드 및 푸시
  build:
    name: 빌드 및 푸시
    runs-on: ubuntu-latest
    needs: [test, security]
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'

    permissions:
      contents: read
      packages: write

    steps:
    - name: 체크아웃
      uses: actions/checkout@v4

    - name: 도커 메타데이터
      id: meta
      uses: docker/metadata-action@v5
      with:
        images: ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}
        tags: |
          type=ref,event=branch
          type=ref,event=pr
          type=semver,pattern={{version}}
          type=semver,pattern={{major}}.{{minor}}
          type=sha,prefix={{branch}}-
          type=raw,value=latest,enable={{is_default_branch}}

    - name: 도커 로그인
      uses: docker/login-action@v3
      with:
        registry: ${{ env.REGISTRY }}
        username: ${{ github.actor }}
        password: ${{ secrets.GITHUB_TOKEN }}

    - name: 도커 빌드 및 푸시
      uses: docker/build-push-action@v5
      with:
        context: .
        push: true
        tags: ${{ steps.meta.outputs.tags }}
        labels: ${{ steps.meta.outputs.labels }}
        cache-from: type=gha
        cache-to: type=gha,mode=max

  # 배포
  deploy:
    name: Kubernetes 배포
    runs-on: ubuntu-latest
    needs: build
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'

    steps:
    - name: 체크아웃
      uses: actions/checkout@v4

    - name: Kubernetes 설정
      uses: azure/k8s-set-context@v3
      with:
        method: kubeconfig
        kubeconfig: ${{ secrets.KUBE_CONFIG }}

    - name: Helm 설치
      uses: azure/setup-helm@v3
      with:
        version: '3.13.0'

    - name: Helm 배포
      run: |
        helm upgrade --install cryo-monitor ./helm/cryo-monitor \
          --namespace cryo-monitoring \
          --create-namespace \
          --set image.tag=${{ github.sha }} \
          --set database.password=${{ secrets.DB_PASSWORD }} \
          --set redis.password=${{ secrets.REDIS_PASSWORD }} \
          --wait \
          --timeout 10m

    - name: 배포 확인
      run: |
        kubectl rollout status deployment/backend -n cryo-monitoring
        kubectl get pods -n cryo-monitoring

  # 알림
  notify:
    name: 배포 알림
    runs-on: ubuntu-latest
    needs: deploy
    if: always()

    steps:
    - name: Slack 알림
      uses: 8398a7/action-slack@v3
      with:
        status: ${{ job.status }}
        text: |
          배포 결과: ${{ job.status }}
          브랜치: ${{ github.ref }}
          커밋: ${{ github.sha }}
          작성자: ${{ github.actor }}
        webhook_url: ${{ secrets.SLACK_WEBHOOK }}
      if: always()
```

---

## 성능 최적화

### 데이터베이스 최적화

```typescript
/**
 * PostgreSQL + TimescaleDB 최적화
 */
interface DatabaseOptimization {
  // 인덱싱 전략
  indexing: {
    btree: string[];                    // B-Tree 인덱스
    btreeKr: string;
    hash: string[];                     // Hash 인덱스
    hashKr: string;
    gin: string[];                      // GIN 인덱스 (JSON)
    ginKr: string;
    indexingKr: string;
  };

  // 파티셔닝
  partitioning: {
    strategy: "time-based" | "range" | "list";
    strategyKr: string;
    interval: "1 day" | "1 week" | "1 month";
    intervalKr: string;
    retentionPolicy: string;
    retentionKr: string;
    partitioningKr: string;
  };

  // 압축
  compression: {
    enabled: boolean;
    algorithm: "native" | "gorilla";
    compressionRatio: string;
    compressionKr: string;
  };

  // 연속 집계 (Continuous Aggregates)
  continuousAggregates: {
    enabled: boolean;
    views: {
      name: string;
      nameKr: string;
      interval: string;
      refreshPolicy: string;
    }[];
    aggregatesKr: string;
  };

  optimizationKr: string;
}

const dbOptimization: DatabaseOptimization = {
  indexing: {
    btree: [
      "CREATE INDEX idx_sensor_readings_timestamp ON sensor_readings (timestamp DESC);",
      "CREATE INDEX idx_sensor_readings_sensor_id ON sensor_readings (sensor_id);",
      "CREATE INDEX idx_alerts_facility_id_timestamp ON alerts (facility_id, timestamp DESC);"
    ],
    btreeKr: "시간 및 센서 ID 기반 B-Tree 인덱스",
    hash: [
      "CREATE INDEX idx_sensors_facility_id_hash ON sensors USING HASH (facility_id);"
    ],
    hashKr: "시설 ID Hash 인덱스 (동등 비교 최적화)",
    gin: [
      "CREATE INDEX idx_alerts_metadata_gin ON alerts USING GIN (metadata);",
      "CREATE INDEX idx_sensors_tags_gin ON sensors USING GIN (tags);"
    ],
    ginKr: "JSON 필드 GIN 인덱스 (전문 검색)",
    indexingKr: "복합 인덱싱 전략으로 쿼리 성능 최적화"
  },

  partitioning: {
    strategy: "time-based",
    strategyKr: "시간 기반 파티셔닝",
    interval: "1 day",
    intervalKr: "일별 파티션",
    retentionPolicy: "DELETE FROM sensor_readings WHERE timestamp < NOW() - INTERVAL '1 year';",
    retentionKr: "1년 이상 데이터 자동 삭제",
    partitioningKr: "TimescaleDB 하이퍼테이블 - 일별 파티션, 1년 보관"
  },

  compression: {
    enabled: true,
    algorithm: "native",
    compressionRatio: "10:1",
    compressionKr: "TimescaleDB 네이티브 압축 (10:1 압축률)"
  },

  continuousAggregates: {
    enabled: true,
    views: [
      {
        name: "sensor_readings_hourly",
        nameKr: "시간별 센서 데이터 집계",
        interval: "1 hour",
        refreshPolicy: "REFRESH MATERIALIZED VIEW CONTINUOUSLY"
      },
      {
        name: "sensor_readings_daily",
        nameKr: "일별 센서 데이터 집계",
        interval: "1 day",
        refreshPolicy: "REFRESH MATERIALIZED VIEW CONTINUOUSLY"
      }
    ],
    aggregatesKr: "연속 집계 뷰 - 실시간 통계 쿼리 최적화"
  },

  optimizationKr: "TimescaleDB 기반 시계열 데이터 최적화"
};

// TimescaleDB 하이퍼테이블 생성
const createHypertableSQL = `
-- 센서 읽기 하이퍼테이블
CREATE TABLE sensor_readings (
  reading_id UUID PRIMARY KEY,
  sensor_id UUID NOT NULL,
  value DOUBLE PRECISION NOT NULL,
  unit VARCHAR(50) NOT NULL,
  quality VARCHAR(20) NOT NULL,
  timestamp TIMESTAMPTZ NOT NULL,
  metadata JSONB
);

-- 하이퍼테이블 변환
SELECT create_hypertable('sensor_readings', 'timestamp', chunk_time_interval => INTERVAL '1 day');

-- 압축 정책
ALTER TABLE sensor_readings SET (
  timescaledb.compress,
  timescaledb.compress_segmentby = 'sensor_id'
);

SELECT add_compression_policy('sensor_readings', INTERVAL '7 days');

-- 보관 정책
SELECT add_retention_policy('sensor_readings', INTERVAL '1 year');

-- 연속 집계 (시간별)
CREATE MATERIALIZED VIEW sensor_readings_hourly
WITH (timescaledb.continuous) AS
SELECT
  sensor_id,
  time_bucket('1 hour', timestamp) AS bucket,
  AVG(value) AS avg_value,
  MIN(value) AS min_value,
  MAX(value) AS max_value,
  COUNT(*) AS count
FROM sensor_readings
GROUP BY sensor_id, bucket;

-- 연속 집계 자동 갱신
SELECT add_continuous_aggregate_policy('sensor_readings_hourly',
  start_offset => INTERVAL '3 hours',
  end_offset => INTERVAL '1 hour',
  schedule_interval => INTERVAL '1 hour');
`;
```

### 캐싱 전략

```typescript
/**
 * Redis 캐싱 전략
 */
class CacheStrategy {
  /**
   * 센서 데이터 캐싱
   * 최신 데이터는 Redis에서 빠르게 조회
   */
  async cacheSensorReading(reading: SensorReading): Promise<void> {
    const redis = getRedisClient();

    // 최신 읽기값 캐시 (1시간 TTL)
    await redis.setex(
      `sensor:${reading.sensorId}:latest`,
      3600,
      JSON.stringify(reading)
    );

    // 최근 100개 읽기값 (Sorted Set)
    await redis.zadd(
      `sensor:${reading.sensorId}:recent`,
      new Date(reading.timestamp).getTime(),
      JSON.stringify(reading)
    );

    // 최근 100개만 유지
    await redis.zremrangebyrank(
      `sensor:${reading.sensorId}:recent`,
      0,
      -101
    );

    console.log(`[캐시] 센서 ${reading.sensorId} 최신 데이터 캐시됨`);
  }

  /**
   * 통계 캐싱
   * 자주 조회되는 통계는 사전 계산하여 캐시
   */
  async cacheStatistics(
    sensorId: string,
    timeRange: string,
    statistics: SensorStatistics
  ): Promise<void> {
    const redis = getRedisClient();

    const cacheKey = `stats:${sensorId}:${timeRange}`;

    // 5분 TTL
    await redis.setex(
      cacheKey,
      300,
      JSON.stringify(statistics)
    );

    console.log(`[캐시] 통계 캐시됨: ${cacheKey}`);
  }

  /**
   * 알림 캐싱
   * 활성 알림은 빠른 조회를 위해 캐시
   */
  async cacheActiveAlerts(facilityId: string, alerts: Alert[]): Promise<void> {
    const redis = getRedisClient();

    const cacheKey = `alerts:active:${facilityId}`;

    // 1분 TTL (자주 변경됨)
    await redis.setex(
      cacheKey,
      60,
      JSON.stringify(alerts)
    );

    console.log(`[캐시] 활성 알림 캐시됨: ${facilityId}`);
  }

  /**
   * 캐시 무효화
   */
  async invalidateCache(pattern: string): Promise<void> {
    const redis = getRedisClient();

    const keys = await redis.keys(pattern);

    if (keys.length > 0) {
      await redis.del(...keys);
      console.log(`[캐시 무효화] ${keys.length}개 키 삭제됨`);
    }
  }
}

function getRedisClient(): any {
  // Redis 클라이언트 반환
  return {} as any;
}
```

---

## 모니터링 및 관찰성

### Prometheus + Grafana

```yaml
# k8s/prometheus-config.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: prometheus-config
  namespace: monitoring
data:
  prometheus.yml: |
    global:
      scrape_interval: 15s
      evaluation_interval: 15s

    scrape_configs:
      # Kubernetes Pods
      - job_name: 'kubernetes-pods'
        kubernetes_sd_configs:
          - role: pod
        relabel_configs:
          - source_labels: [__meta_kubernetes_pod_annotation_prometheus_io_scrape]
            action: keep
            regex: true
          - source_labels: [__meta_kubernetes_pod_annotation_prometheus_io_path]
            action: replace
            target_label: __metrics_path__
            regex: (.+)
          - source_labels: [__address__, __meta_kubernetes_pod_annotation_prometheus_io_port]
            action: replace
            regex: ([^:]+)(?::\d+)?;(\d+)
            replacement: $1:$2
            target_label: __address__

      # Backend 서비스
      - job_name: 'cryo-backend'
        static_configs:
          - targets: ['backend.cryo-monitoring.svc.cluster.local:3000']
        metrics_path: /metrics

      # PostgreSQL
      - job_name: 'postgres'
        static_configs:
          - targets: ['postgres-exporter.cryo-monitoring.svc.cluster.local:9187']

      # Redis
      - job_name: 'redis'
        static_configs:
          - targets: ['redis-exporter.cryo-monitoring.svc.cluster.local:9121']

---
# Grafana 대시보드 (JSON)
# grafana-dashboard-cryo.json
{
  "dashboard": {
    "title": "WIA Cryo Monitoring Dashboard",
    "panels": [
      {
        "title": "센서 데이터 수집률",
        "targets": [
          {
            "expr": "rate(sensor_readings_total[5m])"
          }
        ]
      },
      {
        "title": "API 응답 시간",
        "targets": [
          {
            "expr": "histogram_quantile(0.95, rate(http_request_duration_seconds_bucket[5m]))"
          }
        ]
      },
      {
        "title": "활성 알림 수",
        "targets": [
          {
            "expr": "alerts_active_total"
          }
        ]
      },
      {
        "title": "데이터베이스 연결",
        "targets": [
          {
            "expr": "pg_stat_database_numbackends"
          }
        ]
      }
    ]
  }
}
```

---

## 결론

WIA Cryo Monitoring Standard는 프로덕션 환경을 위한 완전한 구현 가이드를 제공합니다.

### 핵심 특징

1. **컨테이너화**: Docker + Kubernetes
2. **CI/CD**: GitHub Actions 자동화
3. **성능**: TimescaleDB + Redis 최적화
4. **관찰성**: Prometheus + Grafana

### 다음 장 예고

다음 장에서는 **미래 트렌드**를 다룹니다:
- AI/ML 예측 분석
- 디지털 트윈
- 엣지 컴퓨팅
- 6G 연결성

---

**© 2026 WIA (World Certification Industry Association)**
**弘益人間 (홍익인간) - Benefit All Humanity**
