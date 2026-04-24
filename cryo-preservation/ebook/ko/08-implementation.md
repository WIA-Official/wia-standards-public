# 구현 및 배포

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 시스템 아키텍처

### 마이크로서비스 아키텍처

```typescript
/**
 * WIA 냉동보존 시스템 아키텍처
 *
 * - 마이크로서비스 기반
 * - 이벤트 드리븐 아키텍처
 * - CQRS 패턴
 * - 서비스 메시 (Service Mesh)
 */

/**
 * 시스템 구성 요소
 */
export interface SystemArchitecture {
  // API Gateway
  apiGateway: {
    service: 'Kong' | 'AWS API Gateway' | 'Azure API Management';
    features: string[];
    endpoints: {
      rest: string;
      graphql: string;
      websocket: string;
    };
  };

  // 마이크로서비스
  services: {
    specimenService: MicroserviceConfig;
    protocolService: MicroserviceConfig;
    storageService: MicroserviceConfig;
    trackingService: MicroserviceConfig;
    authService: MicroserviceConfig;
    auditService: MicroserviceConfig;
    notificationService: MicroserviceConfig;
    reportingService: MicroserviceConfig;
  };

  // 데이터 저장소
  databases: {
    primary: DatabaseConfig;
    cache: CacheConfig;
    timeseries: TimeseriesConfig;
    documentStore: DocumentStoreConfig;
  };

  // 메시징
  messaging: {
    eventBus: 'Kafka' | 'RabbitMQ' | 'Azure Service Bus';
    topics: string[];
  };

  // 모니터링
  monitoring: {
    metrics: 'Prometheus' | 'CloudWatch';
    logging: 'ELK Stack' | 'CloudWatch Logs';
    tracing: 'Jaeger' | 'AWS X-Ray';
    alerting: 'AlertManager' | 'PagerDuty';
  };
}

interface MicroserviceConfig {
  name: string;
  nameKr: string;
  version: string;
  replicas: number;
  resources: {
    cpu: string;
    memory: string;
  };
  autoscaling: {
    enabled: boolean;
    minReplicas: number;
    maxReplicas: number;
    targetCPU: number;
    targetMemory: number;
  };
  healthCheck: {
    path: string;
    interval: number;
    timeout: number;
  };
}

interface DatabaseConfig {
  type: 'PostgreSQL' | 'MySQL' | 'SQL Server';
  version: string;
  replication: {
    enabled: boolean;
    mode: 'sync' | 'async';
    replicas: number;
  };
  backup: {
    enabled: boolean;
    schedule: string;
    retention: number;
  };
}

interface CacheConfig {
  type: 'Redis' | 'Memcached';
  cluster: boolean;
  nodes: number;
  evictionPolicy: string;
}

interface TimeseriesConfig {
  type: 'InfluxDB' | 'TimescaleDB';
  retention: number;
  aggregation: string[];
}

interface DocumentStoreConfig {
  type: 'MongoDB' | 'Cosmos DB';
  replication: number;
}

/**
 * 검체 서비스 구현
 */
export class SpecimenMicroservice {
  private readonly serviceName = 'specimen-service';
  private readonly version = '1.0.0';

  /**
   * 서비스 초기화
   */
  async initialize(): Promise<void> {
    console.log(`[${this.serviceName}] 초기화 시작...`);

    // 1. 데이터베이스 연결
    await this.connectDatabase();

    // 2. 캐시 연결
    await this.connectCache();

    // 3. 메시지 브로커 연결
    await this.connectMessageBroker();

    // 4. 헬스 체크 엔드포인트 설정
    this.setupHealthCheck();

    // 5. 메트릭 수집 시작
    this.startMetricsCollection();

    console.log(`[${this.serviceName}] 초기화 완료`);
  }

  /**
   * 헬스 체크
   */
  async healthCheck(): Promise<{
    status: 'healthy' | 'unhealthy';
    checks: Record<string, boolean>;
    timestamp: string;
  }> {
    const checks = {
      database: await this.checkDatabase(),
      cache: await this.checkCache(),
      messageBroker: await this.checkMessageBroker(),
    };

    const allHealthy = Object.values(checks).every(check => check === true);

    return {
      status: allHealthy ? 'healthy' : 'unhealthy',
      checks,
      timestamp: new Date().toISOString(),
    };
  }

  /**
   * 메트릭 수집
   */
  async collectMetrics(): Promise<{
    requestsPerSecond: number;
    averageResponseTime: number;
    errorRate: number;
    activeConnections: number;
    cacheHitRate: number;
  }> {
    return {
      requestsPerSecond: 150,
      averageResponseTime: 45, // ms
      errorRate: 0.2, // %
      activeConnections: 320,
      cacheHitRate: 92.5, // %
    };
  }

  // Helper methods
  private async connectDatabase(): Promise<void> {
    // 데이터베이스 연결 로직
  }

  private async connectCache(): Promise<void> {
    // 캐시 연결 로직
  }

  private async connectMessageBroker(): Promise<void> {
    // 메시지 브로커 연결 로직
  }

  private setupHealthCheck(): void {
    // 헬스 체크 설정
  }

  private startMetricsCollection(): void {
    // 메트릭 수집 시작
  }

  private async checkDatabase(): Promise<boolean> {
    return true;
  }

  private async checkCache(): Promise<boolean> {
    return true;
  }

  private async checkMessageBroker(): Promise<boolean> {
    return true;
  }
}
```

## 배포 전략

### Kubernetes 배포

```typescript
/**
 * Kubernetes 배포 구성
 */
export class KubernetesDeployment {
  /**
   * 배포 매니페스트 생성
   */
  generateDeploymentManifest(service: {
    name: string;
    image: string;
    replicas: number;
    port: number;
    env: Record<string, string>;
  }): any {
    return {
      apiVersion: 'apps/v1',
      kind: 'Deployment',
      metadata: {
        name: service.name,
        labels: {
          app: service.name,
          version: 'v1',
        },
      },
      spec: {
        replicas: service.replicas,
        selector: {
          matchLabels: {
            app: service.name,
          },
        },
        template: {
          metadata: {
            labels: {
              app: service.name,
              version: 'v1',
            },
          },
          spec: {
            containers: [
              {
                name: service.name,
                image: service.image,
                ports: [
                  {
                    containerPort: service.port,
                  },
                ],
                env: Object.entries(service.env).map(([name, value]) => ({
                  name,
                  value,
                })),
                resources: {
                  requests: {
                    cpu: '500m',
                    memory: '512Mi',
                  },
                  limits: {
                    cpu: '1000m',
                    memory: '1Gi',
                  },
                },
                livenessProbe: {
                  httpGet: {
                    path: '/health',
                    port: service.port,
                  },
                  initialDelaySeconds: 30,
                  periodSeconds: 10,
                },
                readinessProbe: {
                  httpGet: {
                    path: '/ready',
                    port: service.port,
                  },
                  initialDelaySeconds: 5,
                  periodSeconds: 5,
                },
              },
            ],
          },
        },
      },
    };
  }

  /**
   * 서비스 매니페스트 생성
   */
  generateServiceManifest(service: {
    name: string;
    port: number;
    targetPort: number;
    type: 'ClusterIP' | 'LoadBalancer' | 'NodePort';
  }): any {
    return {
      apiVersion: 'v1',
      kind: 'Service',
      metadata: {
        name: service.name,
      },
      spec: {
        type: service.type,
        selector: {
          app: service.name,
        },
        ports: [
          {
            port: service.port,
            targetPort: service.targetPort,
            protocol: 'TCP',
          },
        ],
      },
    };
  }

  /**
   * HorizontalPodAutoscaler 매니페스트 생성
   */
  generateHPAManifest(service: {
    name: string;
    minReplicas: number;
    maxReplicas: number;
    targetCPU: number;
  }): any {
    return {
      apiVersion: 'autoscaling/v2',
      kind: 'HorizontalPodAutoscaler',
      metadata: {
        name: `${service.name}-hpa`,
      },
      spec: {
        scaleTargetRef: {
          apiVersion: 'apps/v1',
          kind: 'Deployment',
          name: service.name,
        },
        minReplicas: service.minReplicas,
        maxReplicas: service.maxReplicas,
        metrics: [
          {
            type: 'Resource',
            resource: {
              name: 'cpu',
              target: {
                type: 'Utilization',
                averageUtilization: service.targetCPU,
              },
            },
          },
        ],
      },
    };
  }

  /**
   * ConfigMap 매니페스트 생성
   */
  generateConfigMapManifest(config: {
    name: string;
    data: Record<string, string>;
  }): any {
    return {
      apiVersion: 'v1',
      kind: 'ConfigMap',
      metadata: {
        name: config.name,
      },
      data: config.data,
    };
  }

  /**
   * Secret 매니페스트 생성
   */
  generateSecretManifest(secret: {
    name: string;
    data: Record<string, string>;
  }): any {
    return {
      apiVersion: 'v1',
      kind: 'Secret',
      metadata: {
        name: secret.name,
      },
      type: 'Opaque',
      data: Object.entries(secret.data).reduce(
        (acc, [key, value]) => {
          acc[key] = Buffer.from(value).toString('base64');
          return acc;
        },
        {} as Record<string, string>
      ),
    };
  }
}

/**
 * CI/CD 파이프라인
 */
export class CICDPipeline {
  /**
   * GitHub Actions 워크플로우
   */
  generateGitHubActionsWorkflow(): any {
    return {
      name: 'WIA Cryo Preservation - CI/CD',
      on: {
        push: {
          branches: ['main', 'develop'],
        },
        pull_request: {
          branches: ['main'],
        },
      },
      jobs: {
        test: {
          'runs-on': 'ubuntu-latest',
          steps: [
            {
              name: 'Checkout',
              uses: 'actions/checkout@v3',
            },
            {
              name: 'Setup Node.js',
              uses: 'actions/setup-node@v3',
              with: {
                'node-version': '18',
              },
            },
            {
              name: 'Install dependencies',
              run: 'npm ci',
            },
            {
              name: 'Run tests',
              run: 'npm test',
            },
            {
              name: 'Run linter',
              run: 'npm run lint',
            },
            {
              name: 'Check types',
              run: 'npm run type-check',
            },
          ],
        },
        build: {
          needs: 'test',
          'runs-on': 'ubuntu-latest',
          steps: [
            {
              name: 'Build Docker image',
              run: 'docker build -t wia-cryo:${{ github.sha }} .',
            },
            {
              name: 'Push to registry',
              run: 'docker push wia-cryo:${{ github.sha }}',
            },
          ],
        },
        deploy: {
          needs: 'build',
          'runs-on': 'ubuntu-latest',
          if: "github.ref == 'refs/heads/main'",
          steps: [
            {
              name: 'Deploy to Kubernetes',
              run: 'kubectl apply -f k8s/',
            },
            {
              name: 'Wait for rollout',
              run: 'kubectl rollout status deployment/specimen-service',
            },
          ],
        },
      },
    };
  }

  /**
   * 카나리 배포
   */
  async canaryDeployment(config: {
    serviceName: string;
    newVersion: string;
    canaryPercentage: number;
    monitoringDuration: number; // minutes
  }): Promise<{
    success: boolean;
    promoted: boolean;
    metrics: any;
  }> {
    console.log(`[Canary] ${config.serviceName} 카나리 배포 시작...`);

    // 1. 카나리 버전 배포
    await this.deployCanary(config.serviceName, config.newVersion);

    // 2. 트래픽 분할 (예: 10% 카나리)
    await this.splitTraffic(config.serviceName, config.canaryPercentage);

    // 3. 모니터링
    const metrics = await this.monitorCanary(
      config.serviceName,
      config.monitoringDuration
    );

    // 4. 성공 판단
    const success = this.evaluateCanary(metrics);

    // 5. 프로모션 또는 롤백
    if (success) {
      await this.promoteCanary(config.serviceName);
      console.log('[Canary] 프로모션 완료');
      return { success: true, promoted: true, metrics };
    } else {
      await this.rollbackCanary(config.serviceName);
      console.log('[Canary] 롤백 완료');
      return { success: false, promoted: false, metrics };
    }
  }

  /**
   * 블루-그린 배포
   */
  async blueGreenDeployment(config: {
    serviceName: string;
    newVersion: string;
  }): Promise<{
    success: boolean;
    switchedToGreen: boolean;
  }> {
    console.log(`[Blue-Green] ${config.serviceName} 배포 시작...`);

    // 1. 그린 환경에 새 버전 배포
    await this.deployToGreen(config.serviceName, config.newVersion);

    // 2. 그린 환경 헬스 체크
    const healthy = await this.checkGreenHealth(config.serviceName);

    if (!healthy) {
      await this.cleanupGreen(config.serviceName);
      return { success: false, switchedToGreen: false };
    }

    // 3. 트래픽을 그린으로 전환
    await this.switchToGreen(config.serviceName);

    // 4. 블루 환경 정리 (일정 시간 후)
    setTimeout(() => {
      this.cleanupBlue(config.serviceName);
    }, 30 * 60 * 1000); // 30분 후

    console.log('[Blue-Green] 배포 완료');
    return { success: true, switchedToGreen: true };
  }

  // Helper methods
  private async deployCanary(serviceName: string, version: string): Promise<void> {
    // 카나리 배포 로직
  }

  private async splitTraffic(serviceName: string, percentage: number): Promise<void> {
    // 트래픽 분할 로직
  }

  private async monitorCanary(serviceName: string, duration: number): Promise<any> {
    // 모니터링 로직
    return {};
  }

  private evaluateCanary(metrics: any): boolean {
    // 카나리 평가 로직
    return true;
  }

  private async promoteCanary(serviceName: string): Promise<void> {
    // 프로모션 로직
  }

  private async rollbackCanary(serviceName: string): Promise<void> {
    // 롤백 로직
  }

  private async deployToGreen(serviceName: string, version: string): Promise<void> {
    // 그린 배포 로직
  }

  private async checkGreenHealth(serviceName: string): Promise<boolean> {
    // 헬스 체크 로직
    return true;
  }

  private async switchToGreen(serviceName: string): Promise<void> {
    // 트래픽 전환 로직
  }

  private async cleanupGreen(serviceName: string): Promise<void> {
    // 그린 정리 로직
  }

  private async cleanupBlue(serviceName: string): Promise<void> {
    // 블루 정리 로직
  }
}
```

## 성능 최적화

### 데이터베이스 최적화

```typescript
/**
 * 데이터베이스 최적화 전략
 */
export class DatabaseOptimization {
  /**
   * 인덱스 전략
   */
  async optimizeIndexes(): Promise<{
    created: string[];
    recommendations: string[];
  }> {
    const recommendations = [
      // 검체 테이블
      'CREATE INDEX idx_specimen_patient ON specimens(patient_id)',
      'CREATE INDEX idx_specimen_type_status ON specimens(type, status)',
      'CREATE INDEX idx_specimen_created_at ON specimens(created_at)',
      'CREATE INDEX idx_specimen_barcode ON specimens(barcode_id) UNIQUE',

      // 위치 테이블
      'CREATE INDEX idx_storage_tank ON storage_locations(tank_id)',
      'CREATE INDEX idx_storage_specimen ON storage_locations(specimen_id)',

      // 감사 로그 테이블
      'CREATE INDEX idx_audit_user ON audit_logs(user_id)',
      'CREATE INDEX idx_audit_timestamp ON audit_logs(timestamp)',
      'CREATE INDEX idx_audit_resource ON audit_logs(resource_type, resource_id)',

      // 복합 인덱스
      'CREATE INDEX idx_specimen_composite ON specimens(type, status, created_at)',
    ];

    return {
      created: recommendations,
      recommendations,
    };
  }

  /**
   * 쿼리 최적화
   */
  async analyzeSlowQueries(): Promise<{
    slowQueries: Array<{
      query: string;
      executionTime: number;
      suggestions: string[];
    }>;
  }> {
    return {
      slowQueries: [
        {
          query: 'SELECT * FROM specimens WHERE type = ? AND status = ?',
          executionTime: 2500, // ms
          suggestions: [
            '복합 인덱스 추가: (type, status)',
            'SELECT * 대신 필요한 컬럼만 선택',
            '캐싱 고려',
          ],
        },
      ],
    };
  }

  /**
   * 파티셔닝 전략
   */
  async implementPartitioning(table: string): Promise<{
    strategy: string;
    partitions: number;
  }> {
    // 시간 기반 파티셔닝 (감사 로그)
    if (table === 'audit_logs') {
      return {
        strategy: 'RANGE partitioning by timestamp (monthly)',
        partitions: 12,
      };
    }

    // 해시 파티셔닝 (검체)
    if (table === 'specimens') {
      return {
        strategy: 'HASH partitioning by specimen_id',
        partitions: 8,
      };
    }

    return {
      strategy: 'No partitioning required',
      partitions: 1,
    };
  }

  /**
   * 연결 풀링
   */
  async configureConnectionPool(): Promise<{
    poolSize: number;
    maxOverflow: number;
    timeout: number;
  }> {
    return {
      poolSize: 20,
      maxOverflow: 10,
      timeout: 30, // seconds
    };
  }
}

/**
 * 캐싱 전략
 */
export class CachingStrategy {
  /**
   * Redis 캐시 구현
   */
  async implementCache(): Promise<{
    strategies: Array<{
      type: string;
      ttl: number;
      pattern: string;
    }>;
  }> {
    return {
      strategies: [
        {
          type: 'Protocol Cache',
          ttl: 3600, // 1 hour
          pattern: 'protocol:{protocolId}',
        },
        {
          type: 'User Session Cache',
          ttl: 28800, // 8 hours
          pattern: 'session:{sessionId}',
        },
        {
          type: 'Specimen Lookup Cache',
          ttl: 300, // 5 minutes
          pattern: 'specimen:{specimenId}',
        },
        {
          type: 'Storage Status Cache',
          ttl: 60, // 1 minute
          pattern: 'storage:{tankId}',
        },
      ],
    };
  }

  /**
   * 캐시 무효화 전략
   */
  async invalidateCache(
    pattern: string,
    event: 'CREATE' | 'UPDATE' | 'DELETE'
  ): Promise<void> {
    console.log(`캐시 무효화: ${pattern} (이벤트: ${event})`);
    // 캐시 무효화 로직
  }

  /**
   * 캐시 워밍
   */
  async warmUpCache(): Promise<{
    warmedEntries: number;
    duration: number;
  }> {
    const start = Date.now();

    // 자주 접근하는 데이터 미리 로드
    // - 활성 프로토콜
    // - 자주 조회되는 검체
    // - 시스템 설정

    const duration = Date.now() - start;

    return {
      warmedEntries: 1500,
      duration,
    };
  }
}
```

## 모니터링 및 관찰성

### Prometheus 메트릭

```typescript
/**
 * Prometheus 메트릭 수집
 */
export class PrometheusMetrics {
  /**
   * 커스텀 메트릭 정의
   */
  defineMetrics(): {
    counters: any[];
    gauges: any[];
    histograms: any[];
  } {
    return {
      counters: [
        {
          name: 'wia_cryo_specimens_total',
          help: '총 등록된 검체 수',
          labels: ['type', 'status', 'facility'],
        },
        {
          name: 'wia_cryo_protocols_executed_total',
          help: '실행된 프로토콜 수',
          labels: ['protocol_type', 'success'],
        },
        {
          name: 'wia_cryo_api_requests_total',
          help: 'API 요청 수',
          labels: ['method', 'endpoint', 'status'],
        },
      ],
      gauges: [
        {
          name: 'wia_cryo_storage_capacity',
          help: '저장 용량',
          labels: ['tank_id'],
        },
        {
          name: 'wia_cryo_active_users',
          help: '활성 사용자 수',
        },
        {
          name: 'wia_cryo_temperature',
          help: '탱크 온도 (°C)',
          labels: ['tank_id'],
        },
      ],
      histograms: [
        {
          name: 'wia_cryo_api_response_time',
          help: 'API 응답 시간 (ms)',
          labels: ['method', 'endpoint'],
          buckets: [10, 50, 100, 200, 500, 1000, 2000, 5000],
        },
        {
          name: 'wia_cryo_protocol_duration',
          help: '프로토콜 실행 시간 (분)',
          labels: ['protocol_type'],
          buckets: [1, 5, 10, 20, 30, 60, 120],
        },
      ],
    };
  }

  /**
   * 알람 규칙
   */
  defineAlertRules(): any[] {
    return [
      {
        alert: 'HighErrorRate',
        expr: 'rate(wia_cryo_api_requests_total{status="5xx"}[5m]) > 0.05',
        for: '5m',
        labels: {
          severity: 'critical',
        },
        annotations: {
          summary: 'API 오류율 높음 (> 5%)',
          description: '5분 동안 API 오류율이 5%를 초과했습니다',
        },
      },
      {
        alert: 'TemperatureAnomaly',
        expr: 'wia_cryo_temperature > -180',
        for: '1m',
        labels: {
          severity: 'critical',
        },
        annotations: {
          summary: '탱크 온도 이상',
          description: '탱크 {{ $labels.tank_id }} 온도가 -180°C 이상입니다',
        },
      },
      {
        alert: 'StorageNearCapacity',
        expr: 'wia_cryo_storage_capacity > 0.9',
        for: '10m',
        labels: {
          severity: 'warning',
        },
        annotations: {
          summary: '저장 공간 부족',
          description: '탱크 {{ $labels.tank_id }} 용량의 90% 이상 사용 중',
        },
      },
    ];
  }
}

/**
 * 로깅 전략
 */
export class LoggingStrategy {
  /**
   * 구조화된 로깅
   */
  logStructured(event: {
    level: 'debug' | 'info' | 'warn' | 'error' | 'fatal';
    message: string;
    context: Record<string, any>;
  }): void {
    const logEntry = {
      timestamp: new Date().toISOString(),
      level: event.level,
      message: event.message,
      ...event.context,
    };

    console.log(JSON.stringify(logEntry));
  }

  /**
   * 로그 집계
   */
  async aggregateLogs(period: {
    from: Date;
    to: Date;
  }): Promise<{
    total: number;
    byLevel: Record<string, number>;
    topErrors: Array<{
      message: string;
      count: number;
    }>;
  }> {
    return {
      total: 125000,
      byLevel: {
        debug: 50000,
        info: 60000,
        warn: 12000,
        error: 2900,
        fatal: 100,
      },
      topErrors: [
        {
          message: 'Database connection timeout',
          count: 450,
        },
        {
          message: 'Invalid authentication token',
          count: 380,
        },
      ],
    };
  }
}
```

---

**문서 버전**: 1.0
**최종 수정**: 2025-01-11
**작성자**: WIA Standards Committee

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
