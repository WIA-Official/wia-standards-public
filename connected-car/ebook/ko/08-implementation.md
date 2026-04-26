# 제8장: 커넥티드카 구현

## 배포, 테스트 및 인증 가이드라인

본 장에서는 개발 환경부터 프로덕션 배포 및 지속적인 유지보수까지 WIA 커넥티드카 표준 구현에 대한 포괄적인 가이드를 제공합니다.

---

## 개발 환경 설정

### 인프라 요구사항

```typescript
// WIA 커넥티드카 개발 환경
// 커넥티드 차량 플랫폼 개발을 위한 완전한 설정

/**
 * 개발 환경 구성
 */
interface DevelopmentEnvironment {
  infrastructure: InfrastructureConfig;  // 인프라 구성
  toolchain: ToolchainConfig;            // 툴체인 구성
  simulators: SimulatorConfig;           // 시뮬레이터 구성
  testBeds: TestBedConfig;               // 테스트베드 구성
  cicd: CICDPipeline;                    // CI/CD 파이프라인
}

interface InfrastructureConfig {
  cloudProvider: CloudProviderConfig;   // 클라우드 프로바이더
  kubernetes: KubernetesConfig;         // 쿠버네티스
  databases: DatabaseConfig[];          // 데이터베이스
  messaging: MessagingConfig;           // 메시징
  monitoring: MonitoringConfig;         // 모니터링
}

/**
 * 쿠버네티스 배포 구성
 */
const kubernetesDeployment = `
# WIA 커넥티드카 플랫폼 - 쿠버네티스 배포
apiVersion: apps/v1
kind: Deployment
metadata:
  name: connected-car-api
  namespace: wia-connected-car
  labels:
    app: connected-car-api
    version: v1.0.0
spec:
  replicas: 3
  selector:
    matchLabels:
      app: connected-car-api
  template:
    metadata:
      labels:
        app: connected-car-api
        version: v1.0.0
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "8080"
        prometheus.io/path: "/metrics"
    spec:
      serviceAccountName: connected-car-api
      securityContext:
        runAsNonRoot: true
        runAsUser: 1000
        fsGroup: 1000
      containers:
      - name: api
        image: wia/connected-car-api:1.0.0
        imagePullPolicy: Always
        ports:
        - containerPort: 8080
          name: http
          protocol: TCP
        - containerPort: 8443
          name: https
          protocol: TCP
        - containerPort: 9090
          name: grpc
          protocol: TCP
        env:
        - name: NODE_ENV
          value: "production"
        - name: LOG_LEVEL
          value: "info"
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: connected-car-secrets
              key: database-url
        - name: REDIS_URL
          valueFrom:
            secretKeyRef:
              name: connected-car-secrets
              key: redis-url
        - name: KAFKA_BROKERS
          valueFrom:
            configMapKeyRef:
              name: connected-car-config
              key: kafka-brokers
        resources:
          requests:
            memory: "512Mi"
            cpu: "250m"
          limits:
            memory: "2Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health/live
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
          timeoutSeconds: 5
          failureThreshold: 3
        readinessProbe:
          httpGet:
            path: /health/ready
            port: 8080
          initialDelaySeconds: 5
          periodSeconds: 5
          timeoutSeconds: 3
          failureThreshold: 3
        volumeMounts:
        - name: tls-certs
          mountPath: /etc/tls
          readOnly: true
        - name: config
          mountPath: /etc/config
          readOnly: true
      volumes:
      - name: tls-certs
        secret:
          secretName: connected-car-tls
      - name: config
        configMap:
          name: connected-car-config
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
                  - connected-car-api
              topologyKey: kubernetes.io/hostname
---
apiVersion: v1
kind: Service
metadata:
  name: connected-car-api
  namespace: wia-connected-car
spec:
  type: ClusterIP
  ports:
  - port: 80
    targetPort: 8080
    name: http
  - port: 443
    targetPort: 8443
    name: https
  - port: 9090
    targetPort: 9090
    name: grpc
  selector:
    app: connected-car-api
---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: connected-car-api-hpa
  namespace: wia-connected-car
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: connected-car-api
  minReplicas: 3
  maxReplicas: 20
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
    scaleUp:
      stabilizationWindowSeconds: 60
      policies:
      - type: Percent
        value: 100
        periodSeconds: 60
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
      - type: Percent
        value: 10
        periodSeconds: 60
`;

/**
 * 텔레매틱스 백엔드 서비스
 */
const telematicsDeployment = `
apiVersion: apps/v1
kind: Deployment
metadata:
  name: telematics-processor
  namespace: wia-connected-car
spec:
  replicas: 5
  selector:
    matchLabels:
      app: telematics-processor
  template:
    metadata:
      labels:
        app: telematics-processor
    spec:
      containers:
      - name: processor
        image: wia/telematics-processor:1.0.0
        env:
        - name: KAFKA_CONSUMER_GROUP
          value: "telematics-processor-group"
        - name: KAFKA_TOPICS
          value: "vehicle-telemetry,vehicle-events,vehicle-commands"
        - name: TIMESCALEDB_URL
          valueFrom:
            secretKeyRef:
              name: connected-car-secrets
              key: timescaledb-url
        - name: REDIS_URL
          valueFrom:
            secretKeyRef:
              name: connected-car-secrets
              key: redis-url
        resources:
          requests:
            memory: "1Gi"
            cpu: "500m"
          limits:
            memory: "4Gi"
            cpu: "2000m"
---
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: kafka
  namespace: wia-connected-car
spec:
  serviceName: kafka-headless
  replicas: 3
  selector:
    matchLabels:
      app: kafka
  template:
    metadata:
      labels:
        app: kafka
    spec:
      containers:
      - name: kafka
        image: confluentinc/cp-kafka:7.5.0
        ports:
        - containerPort: 9092
          name: kafka
        - containerPort: 9093
          name: kafka-internal
        env:
        - name: KAFKA_BROKER_ID
          valueFrom:
            fieldRef:
              fieldPath: metadata.name
        - name: KAFKA_ZOOKEEPER_CONNECT
          value: "zookeeper:2181"
        - name: KAFKA_LISTENER_SECURITY_PROTOCOL_MAP
          value: "INTERNAL:PLAINTEXT,EXTERNAL:SSL"
        - name: KAFKA_AUTO_CREATE_TOPICS_ENABLE
          value: "false"
        - name: KAFKA_NUM_PARTITIONS
          value: "12"
        - name: KAFKA_DEFAULT_REPLICATION_FACTOR
          value: "3"
        - name: KAFKA_MIN_INSYNC_REPLICAS
          value: "2"
        volumeMounts:
        - name: data
          mountPath: /var/lib/kafka/data
  volumeClaimTemplates:
  - metadata:
      name: data
    spec:
      accessModes: ["ReadWriteOnce"]
      storageClassName: fast-ssd
      resources:
        requests:
          storage: 100Gi
`;

/**
 * 개발용 차량 시뮬레이터
 */
class VehicleSimulator {
  private vehicleId: string;
  private config: SimulatorConfig;
  private telematicsEmitter: TelematicsEmitter;
  private commandHandler: CommandHandler;
  private state: VehicleState;

  constructor(vehicleId: string, config: SimulatorConfig) {
    this.vehicleId = vehicleId;
    this.config = config;
    this.state = this.initializeState();
    this.telematicsEmitter = new TelematicsEmitter(config.telemetrics);
    this.commandHandler = new CommandHandler(this.state);
  }

  /**
   * 차량 상태 초기화
   */
  private initializeState(): VehicleState {
    return {
      position: {
        latitude: this.config.initialPosition.latitude,
        longitude: this.config.initialPosition.longitude,
        altitude: this.config.initialPosition.altitude,
        heading: 0,
        speed: 0
      },
      powertrain: {
        engineRunning: false,
        rpm: 0,
        gear: "PARK",
        throttle: 0,
        brake: 0
      },
      battery: {
        stateOfCharge: 80,        // 충전 상태 80%
        stateOfHealth: 98,        // 건강 상태 98%
        voltage: 400,             // 전압 400V
        current: 0,               // 전류 0A
        temperature: 25,          // 온도 25°C
        range: 320,               // 주행 가능 거리 320km
        isCharging: false         // 충전 중 아님
      },
      climate: {
        isOn: false,              // 에어컨 꺼짐
        targetTemp: 22,           // 목표 온도 22°C
        insideTemp: 25,           // 실내 온도 25°C
        outsideTemp: 28,          // 실외 온도 28°C
        fanSpeed: 0               // 팬 속도 0
      },
      security: {
        locked: true,             // 잠금
        alarmArmed: false,        // 경보 비활성화
        doorsOpen: [],            // 열린 문 없음
        windowsOpen: []           // 열린 창문 없음
      },
      timestamp: new Date()
    };
  }

  /**
   * 시뮬레이션 시작
   */
  async start(): Promise<void> {
    console.log(`차량 시뮬레이터 시작: ${this.vehicleId}`);

    // 텔레메트리 발신 시작
    this.startTelemetryEmission();

    // 설정된 경우 주행 시뮬레이션 시작
    if (this.config.drivingProfile) {
      this.startDrivingSimulation();
    }

    // 명령 수신 대기
    this.startCommandListener();
  }

  /**
   * 텔레메트리 데이터 발신
   */
  private startTelemetryEmission(): void {
    setInterval(() => {
      const telemetry = this.generateTelemetry();
      this.telematicsEmitter.emit(telemetry);
    }, this.config.telemetrics.interval);
  }

  /**
   * 현재 상태에서 텔레메트리 생성
   */
  private generateTelemetry(): VehicleTelemetry {
    // 실제 센서 데이터를 시뮬레이션하기 위한 노이즈 추가
    const addNoise = (value: number, variance: number) =>
      value + (Math.random() - 0.5) * variance;

    return {
      vehicleId: this.vehicleId,
      timestamp: new Date().toISOString(),
      location: {
        latitude: addNoise(this.state.position.latitude, 0.00001),
        longitude: addNoise(this.state.position.longitude, 0.00001),
        altitude: addNoise(this.state.position.altitude, 0.5),
        heading: this.state.position.heading,
        speed: addNoise(this.state.position.speed, 0.5),
        accuracy: 3 + Math.random() * 2
      },
      battery: {
        stateOfCharge: this.state.battery.stateOfCharge,
        voltage: addNoise(this.state.battery.voltage, 2),
        current: addNoise(this.state.battery.current, 0.5),
        temperature: addNoise(this.state.battery.temperature, 0.3),
        range: this.state.battery.range,
        isCharging: this.state.battery.isCharging
      },
      powertrain: {
        engineRunning: this.state.powertrain.engineRunning,
        rpm: this.state.powertrain.rpm,
        gear: this.state.powertrain.gear,
        throttle: this.state.powertrain.throttle,
        brake: this.state.powertrain.brake
      },
      climate: {
        isOn: this.state.climate.isOn,
        targetTemp: this.state.climate.targetTemp,
        insideTemp: addNoise(this.state.climate.insideTemp, 0.2),
        outsideTemp: addNoise(this.state.climate.outsideTemp, 0.3)
      },
      security: {
        locked: this.state.security.locked,
        doorsOpen: this.state.security.doorsOpen,
        windowsOpen: this.state.security.windowsOpen
      }
    };
  }

  /**
   * 주행 행동 시뮬레이션
   */
  private startDrivingSimulation(): void {
    const profile = this.config.drivingProfile!;

    // 경로 추적 시뮬레이션
    if (profile.route) {
      this.followRoute(profile.route);
    }

    // 속도와 방향에 따른 위치 업데이트
    setInterval(() => {
      if (this.state.position.speed > 0) {
        this.updatePosition();
        this.updateBatteryConsumption();
      }
    }, 1000);
  }

  private updatePosition(): void {
    const speed = this.state.position.speed / 3600; // km/s
    const heading = this.state.position.heading * (Math.PI / 180);

    // 이동 거리 계산
    const distance = speed; // 1초에 km
    const deltaLat = distance * Math.cos(heading) / 111.32; // 도
    const deltaLon = distance * Math.sin(heading) /
      (111.32 * Math.cos(this.state.position.latitude * Math.PI / 180));

    this.state.position.latitude += deltaLat;
    this.state.position.longitude += deltaLon;
  }

  private updateBatteryConsumption(): void {
    // 간단한 소비 모델: 평균 ~15 kWh/100km
    const speedKmh = this.state.position.speed;
    const consumptionRate = 0.15 + (speedKmh / 1000); // km당 kWh
    const distanceKm = speedKmh / 3600; // 1초에 km

    const energyConsumed = consumptionRate * distanceKm;
    const capacityKwh = 75; // 75 kWh 배터리
    const socDrop = (energyConsumed / capacityKwh) * 100;

    this.state.battery.stateOfCharge = Math.max(
      0,
      this.state.battery.stateOfCharge - socDrop
    );
    this.state.battery.range = (this.state.battery.stateOfCharge / 100) * 400;
  }

  private followRoute(route: RoutePoint[]): void {
    // 경로 추적 구현
  }

  private startCommandListener(): void {
    // 수신 명령 대기
  }

  /**
   * 차량 명령 처리
   */
  async handleCommand(command: VehicleCommand): Promise<CommandResult> {
    return this.commandHandler.execute(command, this.state);
  }
}

interface SimulatorConfig {
  initialPosition: { latitude: number; longitude: number; altitude: number };
  telemetrics: { interval: number };
  drivingProfile?: DrivingProfile;
}

interface DrivingProfile {
  route?: RoutePoint[];
  style: "CALM" | "NORMAL" | "AGGRESSIVE";
}

interface RoutePoint {
  latitude: number;
  longitude: number;
  targetSpeed: number;
}

interface VehicleState {
  position: PositionState;
  powertrain: PowertrainState;
  battery: BatteryState;
  climate: ClimateState;
  security: SecurityState;
  timestamp: Date;
}

// 상태 인터페이스 정의 생략 (영문과 동일)
```

---

## 테스트 프레임워크

### 테스트 전략

```typescript
/**
 * 커넥티드카 테스트 프레임워크
 * 차량 연결성에 대한 포괄적 테스트 커버리지
 */
interface TestingFramework {
  unitTests: UnitTestSuite;             // 유닛 테스트
  integrationTests: IntegrationTestSuite;  // 통합 테스트
  e2eTests: E2ETestSuite;               // E2E 테스트
  performanceTests: PerformanceTestSuite;  // 성능 테스트
  securityTests: SecurityTestSuite;     // 보안 테스트
  complianceTests: ComplianceTestSuite; // 규제 준수 테스트
}

/**
 * 유닛 테스트 예제
 */
class ConnectedCarUnitTests {
  /**
   * 텔레메트리 데이터 검증 테스트
   */
  describe("TelemetryValidator", () => {
    it("올바른 텔레메트리 데이터 검증", () => {
      const validator = new TelemetryValidator();
      const validTelemetry = {
        vehicleId: "test-vehicle-001",
        timestamp: new Date().toISOString(),
        location: {
          latitude: 37.7749,
          longitude: -122.4194,
          altitude: 10,
          heading: 180,
          speed: 65
        },
        battery: {
          stateOfCharge: 75,
          voltage: 398,
          current: -45
        }
      };

      const result = validator.validate(validTelemetry);
      expect(result.valid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it("유효하지 않은 위도 거부", () => {
      const validator = new TelemetryValidator();
      const invalidTelemetry = {
        vehicleId: "test-vehicle-001",
        timestamp: new Date().toISOString(),
        location: {
          latitude: 91,  // 유효하지 않음: > 90
          longitude: -122.4194,
          altitude: 10,
          heading: 180,
          speed: 65
        }
      };

      const result = validator.validate(invalidTelemetry);
      expect(result.valid).toBe(false);
      expect(result.errors).toContain("위도는 -90에서 90 사이여야 합니다");
    });

    it("음수 배터리 SOC 거부", () => {
      const validator = new TelemetryValidator();
      const invalidTelemetry = {
        vehicleId: "test-vehicle-001",
        timestamp: new Date().toISOString(),
        battery: {
          stateOfCharge: -5,  // 유효하지 않음
          voltage: 398
        }
      };

      const result = validator.validate(invalidTelemetry);
      expect(result.valid).toBe(false);
      expect(result.errors).toContain("충전상태는 0에서 100 사이여야 합니다");
    });
  });

  /**
   * 명령 실행 테스트
   */
  describe("CommandExecutor", () => {
    let executor: CommandExecutor;
    let mockVehicleConnection: MockVehicleConnection;

    beforeEach(() => {
      mockVehicleConnection = new MockVehicleConnection();
      executor = new CommandExecutor(mockVehicleConnection);
    });

    it("잠금 명령 성공 실행", async () => {
      mockVehicleConnection.setResponse({ success: true });

      const result = await executor.execute({
        vehicleId: "test-vehicle-001",
        command: "LOCK",
        timeout: 30
      });

      expect(result.status).toBe("COMPLETED");
      expect(mockVehicleConnection.lastCommand).toBe("LOCK");
    });

    it("명령 타임아웃 처리", async () => {
      mockVehicleConnection.setDelay(35000);  // 35초

      await expect(executor.execute({
        vehicleId: "test-vehicle-001",
        command: "LOCK",
        timeout: 30
      })).rejects.toThrow("명령 타임아웃");
    });

    it("명령 파라미터 검증", async () => {
      await expect(executor.execute({
        vehicleId: "test-vehicle-001",
        command: "SET_CHARGE_LIMIT",
        parameters: { limit: 110 }  // 유효하지 않음: > 100
      })).rejects.toThrow("충전 한도는 50에서 100 사이여야 합니다");
    });
  });
}

/**
 * 통합 테스트 예제
 */
class ConnectedCarIntegrationTests {
  /**
   * 차량 등록 흐름 테스트
   */
  describe("차량 등록 흐름", () => {
    let apiClient: ConnectedCarAPIClient;
    let testVehicle: TestVehicle;

    beforeAll(async () => {
      apiClient = await createTestAPIClient();
      testVehicle = await createTestVehicle();
    });

    afterAll(async () => {
      await cleanupTestVehicle(testVehicle);
    });

    it("새 차량 등록", async () => {
      const registration = await apiClient.registerVehicle({
        vin: testVehicle.vin,
        userId: testVehicle.ownerId,
        activationCode: testVehicle.activationCode
      });

      expect(registration.success).toBe(true);
      expect(registration.vehicleId).toBeDefined();
      testVehicle.vehicleId = registration.vehicleId;
    });

    it("등록 후 차량 상태 조회", async () => {
      const status = await apiClient.getVehicleStatus(testVehicle.vehicleId);

      expect(status.online).toBe(true);
      expect(status.vin).toBe(testVehicle.vin);
    });

    it("텔레메트리 데이터 수신", async () => {
      // 텔레메트리 대기
      await new Promise(resolve => setTimeout(resolve, 5000));

      const telemetry = await apiClient.getLatestTelemetry(testVehicle.vehicleId);

      expect(telemetry.timestamp).toBeDefined();
      expect(telemetry.location).toBeDefined();
      expect(telemetry.battery).toBeDefined();
    });
  });

  /**
   * OEM 어댑터 통합 테스트
   */
  describe("OEM 어댑터 통합", () => {
    it("Tesla API 인증", async () => {
      const adapter = new TeslaAdapter();
      const result = await adapter.connect({
        type: "oauth",
        accessToken: process.env.TESLA_TEST_TOKEN!,
        refreshToken: process.env.TESLA_REFRESH_TOKEN!
      });

      expect(result.success).toBe(true);
    });

    it("Tesla 계정에서 차량 목록 조회", async () => {
      const adapter = new TeslaAdapter();
      await adapter.connect({
        type: "oauth",
        accessToken: process.env.TESLA_TEST_TOKEN!
      });

      const vehicles = await adapter.listVehicles();

      expect(vehicles.length).toBeGreaterThan(0);
      expect(vehicles[0].vin).toBeDefined();
    });
  });
}

/**
 * 성능 테스트 프레임워크
 */
class PerformanceTestSuite {
  /**
   * 텔레메트리 수집 부하 테스트
   */
  async runTelemetryLoadTest(config: LoadTestConfig): Promise<LoadTestResult> {
    const results: RequestMetrics[] = [];
    const startTime = Date.now();

    // 가상 차량 생성
    const vehicles = Array.from(
      { length: config.concurrentVehicles },
      (_, i) => `load-test-vehicle-${i}`
    );

    // 텔레메트리 생성기 생성
    const generators = vehicles.map(
      vehicleId => new TelemetryGenerator(vehicleId, config.telemetryRate)
    );

    // 부하 테스트 시작
    const testDuration = config.duration * 1000;
    const promises: Promise<void>[] = [];

    for (const generator of generators) {
      promises.push(
        this.runVehicleSimulation(generator, testDuration, results)
      );
    }

    await Promise.all(promises);

    // 메트릭 계산
    const endTime = Date.now();
    const totalDuration = endTime - startTime;

    return {
      totalRequests: results.length,                    // 총 요청 수
      successfulRequests: results.filter(r => r.success).length,  // 성공 요청
      failedRequests: results.filter(r => !r.success).length,     // 실패 요청
      avgLatency: this.calculateAverage(results.map(r => r.latency)),  // 평균 지연
      p50Latency: this.calculatePercentile(results.map(r => r.latency), 50),  // p50
      p95Latency: this.calculatePercentile(results.map(r => r.latency), 95),  // p95
      p99Latency: this.calculatePercentile(results.map(r => r.latency), 99),  // p99
      throughput: results.length / (totalDuration / 1000),  // 처리량
      errorRate: results.filter(r => !r.success).length / results.length  // 오류율
    };
  }

  private async runVehicleSimulation(
    generator: TelemetryGenerator,
    duration: number,
    results: RequestMetrics[]
  ): Promise<void> {
    const endTime = Date.now() + duration;

    while (Date.now() < endTime) {
      const telemetry = generator.generate();
      const startTime = Date.now();

      try {
        await this.sendTelemetry(telemetry);
        results.push({
          success: true,
          latency: Date.now() - startTime,
          timestamp: new Date()
        });
      } catch (error) {
        results.push({
          success: false,
          latency: Date.now() - startTime,
          timestamp: new Date(),
          error: String(error)
        });
      }

      // 다음 간격 대기
      await new Promise(resolve =>
        setTimeout(resolve, 1000 / generator.rate)
      );
    }
  }

  private async sendTelemetry(telemetry: any): Promise<void> {
    // 텔레메트리 엔드포인트로 전송
  }

  private calculateAverage(values: number[]): number {
    return values.reduce((a, b) => a + b, 0) / values.length;
  }

  private calculatePercentile(values: number[], percentile: number): number {
    const sorted = values.sort((a, b) => a - b);
    const index = Math.ceil((percentile / 100) * sorted.length) - 1;
    return sorted[index];
  }
}

interface LoadTestConfig {
  concurrentVehicles: number;  // 동시 차량 수
  telemetryRate: number;       // 차량당 초당 메시지 수
  duration: number;            // 테스트 시간 (초)
}

interface LoadTestResult {
  totalRequests: number;       // 총 요청 수
  successfulRequests: number;  // 성공 요청 수
  failedRequests: number;      // 실패 요청 수
  avgLatency: number;          // 평균 지연 (ms)
  p50Latency: number;          // p50 지연 (ms)
  p95Latency: number;          // p95 지연 (ms)
  p99Latency: number;          // p99 지연 (ms)
  throughput: number;          // 초당 처리량
  errorRate: number;           // 오류율
}
```

---

## 프로덕션 배포

### 배포 파이프라인

```typescript
/**
 * CI/CD 파이프라인 구성
 */
const cicdPipeline = `
# 커넥티드카 플랫폼 GitHub Actions 워크플로우
name: Connected Car CI/CD

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: wia/connected-car

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Node.js 설정
        uses: actions/setup-node@v4
        with:
          node-version: '20'
          cache: 'npm'

      - name: 의존성 설치
        run: npm ci

      - name: 린팅 실행
        run: npm run lint

      - name: 유닛 테스트 실행
        run: npm run test:unit -- --coverage

      - name: 통합 테스트 실행
        run: npm run test:integration
        env:
          TEST_DATABASE_URL: \${{ secrets.TEST_DATABASE_URL }}

      - name: 커버리지 업로드
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage/lcov.info

  security-scan:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Snyk 보안 스캔 실행
        uses: snyk/actions/node@master
        env:
          SNYK_TOKEN: \${{ secrets.SNYK_TOKEN }}

      - name: SAST 스캔 실행
        uses: github/codeql-action/analyze@v2
        with:
          languages: typescript

      - name: 컨테이너 스캔
        uses: aquasecurity/trivy-action@master
        with:
          image-ref: '\${{ env.REGISTRY }}/\${{ env.IMAGE_NAME }}:latest'
          format: 'sarif'
          output: 'trivy-results.sarif'

  build:
    needs: [test, security-scan]
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Docker Buildx 설정
        uses: docker/setup-buildx-action@v3

      - name: 레지스트리 로그인
        uses: docker/login-action@v3
        with:
          registry: \${{ env.REGISTRY }}
          username: \${{ github.actor }}
          password: \${{ secrets.GITHUB_TOKEN }}

      - name: 메타데이터 추출
        id: meta
        uses: docker/metadata-action@v5
        with:
          images: \${{ env.REGISTRY }}/\${{ env.IMAGE_NAME }}

      - name: 빌드 및 푸시
        uses: docker/build-push-action@v5
        with:
          context: .
          push: true
          tags: \${{ steps.meta.outputs.tags }}
          labels: \${{ steps.meta.outputs.labels }}
          cache-from: type=gha
          cache-to: type=gha,mode=max

  deploy-staging:
    needs: build
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/develop'
    environment: staging
    steps:
      - name: 스테이징 배포
        uses: azure/k8s-deploy@v4
        with:
          namespace: wia-connected-car-staging
          manifests: |
            k8s/staging/deployment.yaml
            k8s/staging/service.yaml
          images: |
            \${{ env.REGISTRY }}/\${{ env.IMAGE_NAME }}:\${{ github.sha }}

      - name: 스모크 테스트 실행
        run: |
          npm run test:smoke -- --env=staging

  deploy-production:
    needs: build
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/main'
    environment: production
    steps:
      - name: 카나리 배포
        uses: azure/k8s-deploy@v4
        with:
          namespace: wia-connected-car-prod
          strategy: canary
          percentage: 10
          manifests: |
            k8s/production/deployment.yaml

      - name: 카나리 메트릭 모니터링
        run: |
          ./scripts/monitor-canary.sh --duration=15m --threshold=0.01

      - name: 프로모션 또는 롤백
        run: |
          if [ "\${{ steps.canary-metrics.outputs.success }}" == "true" ]; then
            kubectl rollout promote deployment/connected-car-api -n wia-connected-car-prod
          else
            kubectl rollout undo deployment/connected-car-api -n wia-connected-car-prod
            exit 1
          fi

      - name: 전체 배포
        uses: azure/k8s-deploy@v4
        with:
          namespace: wia-connected-car-prod
          manifests: |
            k8s/production/deployment.yaml
            k8s/production/service.yaml
          images: |
            \${{ env.REGISTRY }}/\${{ env.IMAGE_NAME }}:\${{ github.sha }}
`;

/**
 * 프로덕션 모니터링 구성
 */
const monitoringConfig = `
# Prometheus 모니터링 규칙
groups:
  - name: connected-car-alerts
    rules:
      - alert: HighErrorRate
        expr: |
          sum(rate(http_requests_total{status=~"5.."}[5m])) /
          sum(rate(http_requests_total[5m])) > 0.01
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: 높은 오류율 감지
          description: 지난 5분간 오류율이 {{ $value | humanizePercentage }}입니다

      - alert: TelemetryIngestionLag
        expr: |
          kafka_consumer_group_lag{group="telematics-processor-group"} > 10000
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: 텔레메트리 수집 지연 감지
          description: Kafka 컨슈머 지연이 {{ $value }} 메시지입니다

      - alert: VehicleConnectionDrops
        expr: |
          rate(vehicle_disconnections_total[5m]) > 100
        for: 2m
        labels:
          severity: warning
        annotations:
          summary: 높은 차량 연결 해제율

      - alert: CommandLatencyHigh
        expr: |
          histogram_quantile(0.95, rate(command_latency_seconds_bucket[5m])) > 30
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: 명령 지연 p95가 30초 초과

      - alert: DatabaseConnectionPool
        expr: |
          pg_stat_activity_count / pg_settings_max_connections > 0.8
        for: 5m
        labels:
          severity: warning
        annotations:
          summary: 데이터베이스 연결 풀 80% 사용 중
`;
```

---

## 인증 프로세스

```typescript
/**
 * WIA 커넥티드카 인증 프레임워크
 */
interface CertificationFramework {
  levels: CertificationLevel[];
  requirements: CertificationRequirement[];
  process: CertificationProcess;
  auditCriteria: AuditCriteria;
}

interface CertificationLevel {
  level: "BASIC" | "STANDARD" | "ADVANCED" | "AUTONOMOUS";
  description: string;
  requirements: string[];
  validityPeriod: number;  // 개월
}

const certificationLevels: CertificationLevel[] = [
  {
    level: "BASIC",
    description: "기본 연결 기능을 위한 입문 수준 인증",
    requirements: [
      "핵심 API 구현",
      "기본 텔레메트리 지원",
      "인증 및 권한 부여",
      "전송 중 데이터 암호화",
      "프라이버시 정책 준수"
    ],
    validityPeriod: 12
  },
  {
    level: "STANDARD",
    description: "전체 커넥티드카 기능을 위한 표준 인증",
    requirements: [
      "모든 BASIC 요구사항",
      "원격 명령 실행",
      "실시간 스트리밍 지원",
      "다중 OEM 어댑터 지원",
      "포괄적 로깅 및 감사",
      "GDPR/CCPA 준수",
      "보안 취약점 스캐닝"
    ],
    validityPeriod: 24
  },
  {
    level: "ADVANCED",
    description: "V2X 기능을 포함한 고급 인증",
    requirements: [
      "모든 STANDARD 요구사항",
      "V2X 메시지 처리 (BSM, SPaT, MAP)",
      "OTA 업데이트 지원",
      "고급 보안 통제 (IDS, HSM)",
      "ISO/SAE 21434 준수",
      "성능 벤치마크 충족",
      "재해 복구 기능"
    ],
    validityPeriod: 24
  },
  {
    level: "AUTONOMOUS",
    description: "자율주행차 지원을 위한 최고 수준 인증",
    requirements: [
      "모든 ADVANCED 요구사항",
      "자율주행 데이터 처리",
      "안전 중요 시스템 통합",
      "기능 안전성 (ISO 26262) 준수",
      "실시간 시스템 요구사항",
      "페일세이프 메커니즘",
      "규제 준수 (UN R155/R156)"
    ],
    validityPeriod: 12
  }
];

/**
 * 인증 감사 체크리스트
 */
interface AuditChecklist {
  category: string;
  items: AuditItem[];
}

interface AuditItem {
  id: string;
  requirement: string;
  verificationMethod: string;
  evidence: string[];
  status: "PASS" | "FAIL" | "NOT_APPLICABLE" | "PENDING";
  notes?: string;
}

const auditChecklist: AuditChecklist[] = [
  {
    category: "API 구현",
    items: [
      {
        id: "API-001",
        requirement: "REST API가 WIA OpenAPI 스펙 준수",
        verificationMethod: "스키마 검증",
        evidence: ["API 스펙 문서", "스키마 검증 결과"],
        status: "PENDING"
      },
      {
        id: "API-002",
        requirement: "모든 엔드포인트가 올바른 상태 코드 반환",
        verificationMethod: "자동화 테스트",
        evidence: ["테스트 결과", "커버리지 리포트"],
        status: "PENDING"
      },
      {
        id: "API-003",
        requirement: "레이트 리미팅 올바르게 구현",
        verificationMethod: "부하 테스트",
        evidence: ["부하 테스트 결과", "레이트 리밋 설정"],
        status: "PENDING"
      }
    ]
  },
  {
    category: "보안",
    items: [
      {
        id: "SEC-001",
        requirement: "모든 통신이 TLS 1.3 사용",
        verificationMethod: "네트워크 스캔",
        evidence: ["TLS 스캔 결과", "인증서 체인"],
        status: "PENDING"
      },
      {
        id: "SEC-002",
        requirement: "OAuth 2.0 PKCE 구현",
        verificationMethod: "보안 테스트",
        evidence: ["OAuth 설정", "PKCE 검증"],
        status: "PENDING"
      },
      {
        id: "SEC-003",
        requirement: "의존성에 크리티컬 취약점 없음",
        verificationMethod: "의존성 스캔",
        evidence: ["Snyk/Dependabot 리포트"],
        status: "PENDING"
      },
      {
        id: "SEC-004",
        requirement: "침투 테스트 완료",
        verificationMethod: "제3자 평가",
        evidence: ["침투 테스트 리포트", "조치 증거"],
        status: "PENDING"
      }
    ]
  },
  {
    category: "프라이버시",
    items: [
      {
        id: "PRI-001",
        requirement: "프라이버시 정책 공개 및 접근 가능",
        verificationMethod: "문서 검토",
        evidence: ["프라이버시 정책 URL", "정책 내용"],
        status: "PENDING"
      },
      {
        id: "PRI-002",
        requirement: "동의 관리 구현",
        verificationMethod: "기능 테스트",
        evidence: ["동의 흐름 스크린샷", "데이터베이스 기록"],
        status: "PENDING"
      },
      {
        id: "PRI-003",
        requirement: "정보주체 권리 요청 처리",
        verificationMethod: "프로세스 검증",
        evidence: ["DSR 프로세스 문서", "샘플 요청 처리"],
        status: "PENDING"
      }
    ]
  },
  {
    category: "성능",
    items: [
      {
        id: "PERF-001",
        requirement: "API 지연 p95 < 500ms",
        verificationMethod: "부하 테스트",
        evidence: ["부하 테스트 결과"],
        status: "PENDING"
      },
      {
        id: "PERF-002",
        requirement: "시스템이 10,000대 동시 차량 처리",
        verificationMethod: "확장성 테스트",
        evidence: ["확장성 테스트 리포트"],
        status: "PENDING"
      },
      {
        id: "PERF-003",
        requirement: "99.9% 가동 시간 SLA 역량",
        verificationMethod: "아키텍처 검토",
        evidence: ["HA 아키텍처 다이어그램", "장애 조치 테스트 결과"],
        status: "PENDING"
      }
    ]
  }
];
```

---

## 요약

| 단계 | 주요 활동 | 기간 |
|------|----------|------|
| **개발** | 환경 설정, 코딩, 유닛 테스트 | 지속적 |
| **테스트** | 통합, 성능, 보안 테스트 | 2-4주 |
| **스테이징** | 사전 프로덕션 검증 | 1-2주 |
| **프로덕션** | 카나리 배포, 전체 롤아웃 | 1주 |
| **인증** | 감사, 문서화, 승인 | 4-8주 |

---

**다음 장:** [제9장: 미래 트렌드](./09-future-trends.md) - 자율주행, MaaS 및 신기술.

---

© 2025 World Industry Association (WIA). All rights reserved.
弘益人間 (홍익인간) · Benefit All Humanity
