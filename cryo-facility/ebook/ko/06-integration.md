# 제6장: 통합 패턴

## 6.1 개요

극저온 시설 관리 시스템은 다양한 외부 시스템과 통합되어야 합니다. 이 장에서는 LIMS, BMS, 모니터링 시스템, 외부 서비스와의 통합 패턴을 상세히 다룹니다.

```typescript
// 통합 아키텍처 개요
const integrationArchitecture = {
  internalSystems: {
    lims: 'Laboratory Information Management System',
    bms: 'Building Management System',
    ems: 'Equipment Management System',
    qms: 'Quality Management System'
  },
  externalServices: {
    supplierPortals: '공급업체 포털',
    regulatoryReporting: '규제 보고 시스템',
    cloudBackup: '클라우드 백업',
    aiAnalytics: 'AI 분석 서비스'
  },
  protocols: [
    'REST API',
    'GraphQL',
    'MQTT',
    'BACnet',
    'HL7 FHIR',
    'WebSocket'
  ],
  patterns: [
    'Event-driven',
    'Request-response',
    'Publish-subscribe',
    'Circuit breaker',
    'Saga pattern'
  ]
};
```

## 6.2 통합 허브

### 6.2.1 통합 허브 아키텍처

```typescript
import { EventEmitter } from 'events';

// 어댑터 인터페이스
interface IntegrationAdapter {
  id: string;
  name: string;
  type: string;
  status: 'connected' | 'disconnected' | 'error';
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  healthCheck(): Promise<HealthCheckResult>;
  send(message: IntegrationMessage): Promise<IntegrationResponse>;
  receive?(handler: (message: IntegrationMessage) => void): void;
}

// 통합 메시지
interface IntegrationMessage {
  id: string;
  type: string;
  source: string;
  target: string;
  payload: any;
  headers?: Record<string, string>;
  timestamp: Date;
  correlationId?: string;
}

// 통합 응답
interface IntegrationResponse {
  success: boolean;
  messageId: string;
  data?: any;
  error?: string;
  latency: number;
}

// 헬스체크 결과
interface HealthCheckResult {
  status: 'healthy' | 'degraded' | 'unhealthy';
  latency: number;
  details?: any;
}

// 회로 차단기 상태
enum CircuitState {
  CLOSED = 'closed',
  OPEN = 'open',
  HALF_OPEN = 'half-open'
}

// 회로 차단기
class CircuitBreaker {
  private state: CircuitState = CircuitState.CLOSED;
  private failureCount: number = 0;
  private successCount: number = 0;
  private lastFailureTime?: Date;
  private nextRetryTime?: Date;

  constructor(
    private config: {
      failureThreshold: number;
      successThreshold: number;
      timeout: number;
      resetTimeout: number;
    }
  ) {}

  async execute<T>(operation: () => Promise<T>): Promise<T> {
    if (this.state === CircuitState.OPEN) {
      if (this.nextRetryTime && new Date() > this.nextRetryTime) {
        this.state = CircuitState.HALF_OPEN;
      } else {
        throw new Error('회로가 열려있습니다');
      }
    }

    try {
      const result = await this.withTimeout(operation, this.config.timeout);
      this.onSuccess();
      return result;
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
        this.successCount = 0;
      }
    }
  }

  private onFailure(): void {
    this.failureCount++;
    this.lastFailureTime = new Date();

    if (this.failureCount >= this.config.failureThreshold) {
      this.state = CircuitState.OPEN;
      this.nextRetryTime = new Date(
        Date.now() + this.config.resetTimeout
      );
    }
  }

  private async withTimeout<T>(
    operation: () => Promise<T>,
    timeout: number
  ): Promise<T> {
    return Promise.race([
      operation(),
      new Promise<never>((_, reject) =>
        setTimeout(() => reject(new Error('시간 초과')), timeout)
      )
    ]);
  }

  getState(): CircuitState {
    return this.state;
  }
}

// 통합 허브
export class IntegrationHub extends EventEmitter {
  private adapters: Map<string, IntegrationAdapter> = new Map();
  private circuitBreakers: Map<string, CircuitBreaker> = new Map();
  private messageQueue: IntegrationMessage[] = [];
  private retryQueue: Map<string, RetryInfo> = new Map();

  constructor(
    private config: IntegrationHubConfig,
    private logger: Logger
  ) {
    super();
    this.startHealthMonitor();
    this.startRetryProcessor();
  }

  // 어댑터 등록
  registerAdapter(adapter: IntegrationAdapter): void {
    this.adapters.set(adapter.id, adapter);

    // 회로 차단기 생성
    this.circuitBreakers.set(adapter.id, new CircuitBreaker({
      failureThreshold: this.config.circuitBreaker.failureThreshold,
      successThreshold: this.config.circuitBreaker.successThreshold,
      timeout: this.config.circuitBreaker.timeout,
      resetTimeout: this.config.circuitBreaker.resetTimeout
    }));

    this.logger.info(`어댑터 등록됨: ${adapter.id}`);
  }

  // 어댑터 연결
  async connectAdapter(adapterId: string): Promise<void> {
    const adapter = this.adapters.get(adapterId);
    if (!adapter) {
      throw new Error(`어댑터를 찾을 수 없습니다: ${adapterId}`);
    }

    try {
      await adapter.connect();
      this.emit('adapter:connected', { adapterId });
      this.logger.info(`어댑터 연결됨: ${adapterId}`);
    } catch (error) {
      this.emit('adapter:error', { adapterId, error });
      throw error;
    }
  }

  // 메시지 전송
  async send(
    targetAdapterId: string,
    message: Omit<IntegrationMessage, 'id' | 'timestamp'>
  ): Promise<IntegrationResponse> {
    const adapter = this.adapters.get(targetAdapterId);
    if (!adapter) {
      throw new Error(`어댑터를 찾을 수 없습니다: ${targetAdapterId}`);
    }

    const circuitBreaker = this.circuitBreakers.get(targetAdapterId)!;

    const fullMessage: IntegrationMessage = {
      ...message,
      id: this.generateMessageId(),
      timestamp: new Date()
    };

    // 메시지 로깅
    this.logger.debug('메시지 전송', {
      adapterId: targetAdapterId,
      messageId: fullMessage.id,
      type: fullMessage.type
    });

    try {
      const startTime = Date.now();

      const response = await circuitBreaker.execute(() =>
        adapter.send(fullMessage)
      );

      // 성공 이벤트
      this.emit('message:sent', {
        adapterId: targetAdapterId,
        message: fullMessage,
        response,
        latency: Date.now() - startTime
      });

      return response;

    } catch (error) {
      // 재시도 큐에 추가
      if (this.shouldRetry(error)) {
        this.addToRetryQueue(targetAdapterId, fullMessage);
      }

      // 실패 이벤트
      this.emit('message:failed', {
        adapterId: targetAdapterId,
        message: fullMessage,
        error
      });

      throw error;
    }
  }

  // 브로드캐스트
  async broadcast(
    message: Omit<IntegrationMessage, 'id' | 'timestamp' | 'target'>,
    targetTypes?: string[]
  ): Promise<Map<string, IntegrationResponse>> {
    const results = new Map<string, IntegrationResponse>();

    const targets = targetTypes
      ? Array.from(this.adapters.entries())
          .filter(([_, adapter]) => targetTypes.includes(adapter.type))
      : Array.from(this.adapters.entries());

    await Promise.allSettled(
      targets.map(async ([adapterId, adapter]) => {
        try {
          const response = await this.send(adapterId, {
            ...message,
            target: adapterId
          });
          results.set(adapterId, response);
        } catch (error) {
          results.set(adapterId, {
            success: false,
            messageId: '',
            error: error instanceof Error ? error.message : '전송 실패',
            latency: 0
          });
        }
      })
    );

    return results;
  }

  // 헬스 모니터링
  private startHealthMonitor(): void {
    setInterval(async () => {
      for (const [adapterId, adapter] of this.adapters) {
        try {
          const health = await adapter.healthCheck();

          if (health.status !== 'healthy') {
            this.emit('adapter:degraded', { adapterId, health });
          }
        } catch (error) {
          this.emit('adapter:unhealthy', { adapterId, error });
        }
      }
    }, this.config.healthCheckInterval);
  }

  // 재시도 처리
  private startRetryProcessor(): void {
    setInterval(async () => {
      for (const [messageId, retryInfo] of this.retryQueue) {
        if (retryInfo.nextRetryAt <= new Date()) {
          try {
            await this.send(retryInfo.adapterId, retryInfo.message);
            this.retryQueue.delete(messageId);
          } catch (error) {
            retryInfo.attempts++;

            if (retryInfo.attempts >= this.config.maxRetries) {
              this.retryQueue.delete(messageId);
              this.emit('message:retry-exhausted', {
                messageId,
                message: retryInfo.message
              });
            } else {
              retryInfo.nextRetryAt = new Date(
                Date.now() + this.calculateBackoff(retryInfo.attempts)
              );
            }
          }
        }
      }
    }, this.config.retryInterval);
  }

  private shouldRetry(error: any): boolean {
    // 재시도 가능한 오류인지 확인
    const retryableErrors = ['ECONNREFUSED', 'ETIMEDOUT', 'ENOTFOUND'];
    return retryableErrors.some(code => error.code === code);
  }

  private addToRetryQueue(
    adapterId: string,
    message: IntegrationMessage
  ): void {
    this.retryQueue.set(message.id, {
      adapterId,
      message,
      attempts: 0,
      nextRetryAt: new Date(Date.now() + this.config.retryDelay)
    });
  }

  private calculateBackoff(attempt: number): number {
    return Math.min(
      this.config.retryDelay * Math.pow(2, attempt),
      this.config.maxRetryDelay
    );
  }

  private generateMessageId(): string {
    return `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  // 상태 조회
  getStatus(): IntegrationHubStatus {
    const adapterStatuses = Array.from(this.adapters.entries()).map(
      ([id, adapter]) => ({
        id,
        name: adapter.name,
        type: adapter.type,
        status: adapter.status,
        circuitState: this.circuitBreakers.get(id)?.getState()
      })
    );

    return {
      adapters: adapterStatuses,
      queueSize: this.messageQueue.length,
      retryQueueSize: this.retryQueue.size
    };
  }
}

// 재시도 정보
interface RetryInfo {
  adapterId: string;
  message: IntegrationMessage;
  attempts: number;
  nextRetryAt: Date;
}

// 통합 허브 설정
interface IntegrationHubConfig {
  healthCheckInterval: number;
  retryInterval: number;
  retryDelay: number;
  maxRetryDelay: number;
  maxRetries: number;
  circuitBreaker: {
    failureThreshold: number;
    successThreshold: number;
    timeout: number;
    resetTimeout: number;
  };
}

// 통합 허브 상태
interface IntegrationHubStatus {
  adapters: any[];
  queueSize: number;
  retryQueueSize: number;
}
```

## 6.3 LIMS 통합

### 6.3.1 LIMS 어댑터

```typescript
// LIMS 어댑터 구현
export class LIMSAdapter implements IntegrationAdapter {
  id = 'lims-adapter';
  name = 'LIMS Integration';
  type = 'lims';
  status: 'connected' | 'disconnected' | 'error' = 'disconnected';

  private client: any;
  private config: LIMSConfig;
  private messageHandlers: Map<string, Function> = new Map();

  constructor(config: LIMSConfig) {
    this.config = config;
  }

  async connect(): Promise<void> {
    try {
      // LIMS 연결 설정
      this.client = await this.createClient();

      // 인증
      await this.authenticate();

      // 이벤트 리스너 설정
      this.setupEventListeners();

      this.status = 'connected';
    } catch (error) {
      this.status = 'error';
      throw error;
    }
  }

  async disconnect(): Promise<void> {
    if (this.client) {
      await this.client.close();
      this.status = 'disconnected';
    }
  }

  async healthCheck(): Promise<HealthCheckResult> {
    const startTime = Date.now();

    try {
      await this.client.ping();

      return {
        status: 'healthy',
        latency: Date.now() - startTime
      };
    } catch (error) {
      return {
        status: 'unhealthy',
        latency: Date.now() - startTime,
        details: { error: error instanceof Error ? error.message : '알 수 없는 오류' }
      };
    }
  }

  async send(message: IntegrationMessage): Promise<IntegrationResponse> {
    const startTime = Date.now();

    try {
      let result: any;

      switch (message.type) {
        case 'specimen-register':
          result = await this.registerSpecimen(message.payload);
          break;

        case 'specimen-update':
          result = await this.updateSpecimen(message.payload);
          break;

        case 'specimen-retrieve':
          result = await this.retrieveSpecimen(message.payload);
          break;

        case 'specimen-transfer':
          result = await this.transferSpecimen(message.payload);
          break;

        case 'storage-location-update':
          result = await this.updateStorageLocation(message.payload);
          break;

        case 'quality-event':
          result = await this.recordQualityEvent(message.payload);
          break;

        default:
          throw new Error(`지원하지 않는 메시지 유형: ${message.type}`);
      }

      return {
        success: true,
        messageId: message.id,
        data: result,
        latency: Date.now() - startTime
      };
    } catch (error) {
      return {
        success: false,
        messageId: message.id,
        error: error instanceof Error ? error.message : '처리 실패',
        latency: Date.now() - startTime
      };
    }
  }

  receive(handler: (message: IntegrationMessage) => void): void {
    this.client.on('message', (data: any) => {
      const message = this.transformIncomingMessage(data);
      handler(message);
    });
  }

  // 검체 등록
  private async registerSpecimen(payload: SpecimenRegistration): Promise<any> {
    const limsRequest = {
      action: 'CREATE_SAMPLE',
      data: {
        sampleId: payload.specimenId,
        patientId: payload.patientId,
        sampleType: this.mapSpecimenType(payload.type),
        collectionDate: payload.collectionDate,
        collectedBy: payload.collectedBy,
        storageRequirements: {
          temperature: payload.storageTemp,
          container: payload.containerType
        },
        metadata: payload.metadata
      }
    };

    const response = await this.client.request(limsRequest);

    return {
      limsId: response.sampleId,
      barcode: response.barcode,
      status: response.status
    };
  }

  // 검체 업데이트
  private async updateSpecimen(payload: SpecimenUpdate): Promise<any> {
    const limsRequest = {
      action: 'UPDATE_SAMPLE',
      data: {
        sampleId: payload.specimenId,
        updates: payload.updates,
        updatedBy: payload.updatedBy,
        reason: payload.reason
      }
    };

    return this.client.request(limsRequest);
  }

  // 검체 조회
  private async retrieveSpecimen(payload: { specimenId: string }): Promise<any> {
    const limsRequest = {
      action: 'GET_SAMPLE',
      data: {
        sampleId: payload.specimenId
      }
    };

    const response = await this.client.request(limsRequest);

    return this.transformSpecimenData(response);
  }

  // 검체 이동
  private async transferSpecimen(payload: SpecimenTransfer): Promise<any> {
    const limsRequest = {
      action: 'TRANSFER_SAMPLE',
      data: {
        sampleId: payload.specimenId,
        fromLocation: payload.fromLocation,
        toLocation: payload.toLocation,
        transferredBy: payload.transferredBy,
        transferDate: payload.transferDate,
        chainOfCustody: payload.chainOfCustody
      }
    };

    return this.client.request(limsRequest);
  }

  // 보관 위치 업데이트
  private async updateStorageLocation(payload: StorageLocationUpdate): Promise<any> {
    const limsRequest = {
      action: 'UPDATE_STORAGE',
      data: {
        sampleIds: payload.specimenIds,
        newLocation: {
          facility: payload.facilityId,
          zone: payload.zoneId,
          equipment: payload.equipmentId,
          rack: payload.rack,
          box: payload.box,
          position: payload.position
        },
        updatedBy: payload.updatedBy
      }
    };

    return this.client.request(limsRequest);
  }

  // 품질 이벤트 기록
  private async recordQualityEvent(payload: QualityEvent): Promise<any> {
    const limsRequest = {
      action: 'RECORD_QC_EVENT',
      data: {
        eventType: payload.type,
        affectedSamples: payload.affectedSpecimens,
        description: payload.description,
        severity: payload.severity,
        temperatureExcursion: payload.temperatureExcursion,
        actionsTaken: payload.actionsTaken,
        recordedBy: payload.recordedBy
      }
    };

    return this.client.request(limsRequest);
  }

  // 데이터 변환
  private mapSpecimenType(type: string): string {
    const typeMap: Record<string, string> = {
      'blood': 'BLD',
      'serum': 'SER',
      'plasma': 'PLS',
      'tissue': 'TIS',
      'cell': 'CEL',
      'dna': 'DNA',
      'rna': 'RNA'
    };
    return typeMap[type] || 'OTH';
  }

  private transformSpecimenData(limsData: any): any {
    return {
      specimenId: limsData.sampleId,
      status: limsData.status,
      location: {
        facility: limsData.storage?.facility,
        equipment: limsData.storage?.equipment,
        position: limsData.storage?.position
      },
      history: limsData.auditTrail?.map((entry: any) => ({
        action: entry.action,
        timestamp: entry.timestamp,
        user: entry.userId
      }))
    };
  }

  private transformIncomingMessage(data: any): IntegrationMessage {
    return {
      id: data.messageId || this.generateId(),
      type: this.mapLIMSEventType(data.eventType),
      source: 'lims',
      target: 'cryo-facility',
      payload: data,
      timestamp: new Date(data.timestamp)
    };
  }

  private mapLIMSEventType(eventType: string): string {
    const eventMap: Record<string, string> = {
      'SAMPLE_CREATED': 'specimen-created',
      'SAMPLE_UPDATED': 'specimen-updated',
      'SAMPLE_DISPOSED': 'specimen-disposed',
      'QC_FAILED': 'quality-failed'
    };
    return eventMap[eventType] || 'unknown';
  }

  private async createClient(): Promise<any> {
    // LIMS 클라이언트 생성 로직
    return {};
  }

  private async authenticate(): Promise<void> {
    // 인증 로직
  }

  private setupEventListeners(): void {
    // 이벤트 리스너 설정
  }

  private generateId(): string {
    return `lims_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// LIMS 설정
interface LIMSConfig {
  host: string;
  port: number;
  apiKey: string;
  version: string;
  timeout: number;
}

// 검체 등록 페이로드
interface SpecimenRegistration {
  specimenId: string;
  patientId?: string;
  type: string;
  collectionDate: Date;
  collectedBy: string;
  storageTemp: number;
  containerType: string;
  metadata?: Record<string, any>;
}

// 검체 업데이트 페이로드
interface SpecimenUpdate {
  specimenId: string;
  updates: Record<string, any>;
  updatedBy: string;
  reason: string;
}

// 검체 이동 페이로드
interface SpecimenTransfer {
  specimenId: string;
  fromLocation: string;
  toLocation: string;
  transferredBy: string;
  transferDate: Date;
  chainOfCustody: any[];
}

// 보관 위치 업데이트 페이로드
interface StorageLocationUpdate {
  specimenIds: string[];
  facilityId: string;
  zoneId: string;
  equipmentId: string;
  rack: string;
  box: string;
  position: string;
  updatedBy: string;
}

// 품질 이벤트 페이로드
interface QualityEvent {
  type: string;
  affectedSpecimens: string[];
  description: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  temperatureExcursion?: {
    min: number;
    max: number;
    duration: number;
  };
  actionsTaken: string[];
  recordedBy: string;
}
```

## 6.4 BMS 통합

### 6.4.1 BMS/BACnet 어댑터

```typescript
// BACnet 어댑터 구현
export class BACnetAdapter implements IntegrationAdapter {
  id = 'bacnet-adapter';
  name = 'BACnet/BMS Integration';
  type = 'bms';
  status: 'connected' | 'disconnected' | 'error' = 'disconnected';

  private client: any;
  private config: BACnetConfig;
  private subscriptions: Map<string, BACnetSubscription> = new Map();
  private pointCache: Map<string, BACnetPoint> = new Map();

  constructor(config: BACnetConfig) {
    this.config = config;
  }

  async connect(): Promise<void> {
    try {
      // BACnet 클라이언트 초기화
      this.client = await this.initializeBACnetClient();

      // 디바이스 검색
      await this.discoverDevices();

      // 포인트 캐시 구축
      await this.buildPointCache();

      this.status = 'connected';
    } catch (error) {
      this.status = 'error';
      throw error;
    }
  }

  async disconnect(): Promise<void> {
    // 모든 구독 해제
    for (const subscription of this.subscriptions.values()) {
      await this.unsubscribe(subscription.id);
    }

    if (this.client) {
      await this.client.close();
      this.status = 'disconnected';
    }
  }

  async healthCheck(): Promise<HealthCheckResult> {
    const startTime = Date.now();

    try {
      // BACnet 네트워크 상태 확인
      const devices = await this.client.whoIs();

      return {
        status: devices.length > 0 ? 'healthy' : 'degraded',
        latency: Date.now() - startTime,
        details: {
          devicesFound: devices.length
        }
      };
    } catch (error) {
      return {
        status: 'unhealthy',
        latency: Date.now() - startTime,
        details: { error: error instanceof Error ? error.message : '알 수 없는 오류' }
      };
    }
  }

  async send(message: IntegrationMessage): Promise<IntegrationResponse> {
    const startTime = Date.now();

    try {
      let result: any;

      switch (message.type) {
        case 'read-point':
          result = await this.readPoint(message.payload);
          break;

        case 'write-point':
          result = await this.writePoint(message.payload);
          break;

        case 'subscribe-cov':
          result = await this.subscribeCOV(message.payload);
          break;

        case 'read-trend':
          result = await this.readTrendLog(message.payload);
          break;

        case 'get-alarms':
          result = await this.getAlarms(message.payload);
          break;

        case 'ack-alarm':
          result = await this.acknowledgeAlarm(message.payload);
          break;

        default:
          throw new Error(`지원하지 않는 메시지 유형: ${message.type}`);
      }

      return {
        success: true,
        messageId: message.id,
        data: result,
        latency: Date.now() - startTime
      };
    } catch (error) {
      return {
        success: false,
        messageId: message.id,
        error: error instanceof Error ? error.message : '처리 실패',
        latency: Date.now() - startTime
      };
    }
  }

  receive(handler: (message: IntegrationMessage) => void): void {
    // COV (Change of Value) 알림 수신
    this.client.on('cov-notification', (notification: any) => {
      const message = this.transformCOVNotification(notification);
      handler(message);
    });

    // 알람 알림 수신
    this.client.on('alarm-notification', (alarm: any) => {
      const message = this.transformAlarmNotification(alarm);
      handler(message);
    });
  }

  // 포인트 읽기
  private async readPoint(payload: {
    deviceId: number;
    objectType: string;
    objectInstance: number;
    propertyId?: string;
  }): Promise<any> {
    const { deviceId, objectType, objectInstance, propertyId } = payload;

    const result = await this.client.readProperty(
      deviceId,
      {
        type: this.mapObjectType(objectType),
        instance: objectInstance
      },
      propertyId || 'presentValue'
    );

    return {
      value: result.value,
      units: result.units,
      status: result.statusFlags,
      timestamp: new Date()
    };
  }

  // 포인트 쓰기
  private async writePoint(payload: {
    deviceId: number;
    objectType: string;
    objectInstance: number;
    value: any;
    priority?: number;
  }): Promise<any> {
    const { deviceId, objectType, objectInstance, value, priority } = payload;

    await this.client.writeProperty(
      deviceId,
      {
        type: this.mapObjectType(objectType),
        instance: objectInstance
      },
      'presentValue',
      value,
      {
        priority: priority || 16 // 기본 우선순위
      }
    );

    return {
      success: true,
      writtenValue: value,
      timestamp: new Date()
    };
  }

  // COV 구독
  private async subscribeCOV(payload: {
    deviceId: number;
    objectType: string;
    objectInstance: number;
    lifetime?: number;
  }): Promise<any> {
    const { deviceId, objectType, objectInstance, lifetime } = payload;

    const subscriptionId = await this.client.subscribeCOV(
      deviceId,
      {
        type: this.mapObjectType(objectType),
        instance: objectInstance
      },
      {
        subscriberProcessId: this.generateProcessId(),
        lifetime: lifetime || 3600 // 기본 1시간
      }
    );

    const subscription: BACnetSubscription = {
      id: subscriptionId,
      deviceId,
      objectType,
      objectInstance,
      subscribedAt: new Date()
    };

    this.subscriptions.set(subscriptionId, subscription);

    return {
      subscriptionId,
      expiresAt: new Date(Date.now() + (lifetime || 3600) * 1000)
    };
  }

  // 구독 해제
  private async unsubscribe(subscriptionId: string): Promise<void> {
    const subscription = this.subscriptions.get(subscriptionId);
    if (!subscription) return;

    await this.client.unsubscribeCOV(
      subscription.deviceId,
      {
        type: this.mapObjectType(subscription.objectType),
        instance: subscription.objectInstance
      }
    );

    this.subscriptions.delete(subscriptionId);
  }

  // 트렌드 로그 읽기
  private async readTrendLog(payload: {
    deviceId: number;
    objectInstance: number;
    startTime: Date;
    endTime: Date;
    maxRecords?: number;
  }): Promise<any> {
    const { deviceId, objectInstance, startTime, endTime, maxRecords } = payload;

    const records = await this.client.readTrendLog(
      deviceId,
      objectInstance,
      {
        startTime,
        endTime,
        count: maxRecords || 1000
      }
    );

    return {
      records: records.map((r: any) => ({
        timestamp: r.timestamp,
        value: r.value,
        status: r.statusFlags
      })),
      totalCount: records.length
    };
  }

  // 알람 조회
  private async getAlarms(payload: {
    deviceId?: number;
    state?: string;
  }): Promise<any> {
    const alarms = await this.client.getAlarmSummary(payload.deviceId);

    return {
      alarms: alarms
        .filter((a: any) => !payload.state || a.state === payload.state)
        .map((a: any) => ({
          objectId: a.objectId,
          alarmState: a.alarmState,
          acknowledgedTransitions: a.acknowledgedTransitions,
          eventTime: a.eventTime
        }))
    };
  }

  // 알람 확인
  private async acknowledgeAlarm(payload: {
    deviceId: number;
    objectType: string;
    objectInstance: number;
    eventState: string;
    acknowledgeTime: Date;
    acknowledgeSource: string;
  }): Promise<any> {
    await this.client.acknowledgeAlarm(
      payload.deviceId,
      {
        type: this.mapObjectType(payload.objectType),
        instance: payload.objectInstance
      },
      payload.eventState,
      {
        timestamp: payload.acknowledgeTime,
        source: payload.acknowledgeSource
      }
    );

    return { success: true };
  }

  // BACnet 클라이언트 초기화
  private async initializeBACnetClient(): Promise<any> {
    // BACnet 클라이언트 초기화 로직
    return {};
  }

  // 디바이스 검색
  private async discoverDevices(): Promise<void> {
    const devices = await this.client.whoIs();

    for (const device of devices) {
      console.log(`BACnet 디바이스 발견: ${device.deviceId}`);
    }
  }

  // 포인트 캐시 구축
  private async buildPointCache(): Promise<void> {
    // 포인트 정보 캐싱
  }

  // 객체 유형 매핑
  private mapObjectType(type: string): number {
    const typeMap: Record<string, number> = {
      'analog-input': 0,
      'analog-output': 1,
      'analog-value': 2,
      'binary-input': 3,
      'binary-output': 4,
      'binary-value': 5,
      'multi-state-input': 13,
      'multi-state-output': 14,
      'trend-log': 20
    };
    return typeMap[type] || 0;
  }

  // COV 알림 변환
  private transformCOVNotification(notification: any): IntegrationMessage {
    return {
      id: this.generateId(),
      type: 'cov-change',
      source: 'bms',
      target: 'cryo-facility',
      payload: {
        deviceId: notification.deviceId,
        objectType: notification.objectType,
        objectInstance: notification.objectInstance,
        values: notification.values
      },
      timestamp: new Date()
    };
  }

  // 알람 알림 변환
  private transformAlarmNotification(alarm: any): IntegrationMessage {
    return {
      id: this.generateId(),
      type: 'bms-alarm',
      source: 'bms',
      target: 'cryo-facility',
      payload: alarm,
      timestamp: new Date()
    };
  }

  private generateProcessId(): number {
    return Math.floor(Math.random() * 65535);
  }

  private generateId(): string {
    return `bacnet_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// BACnet 설정
interface BACnetConfig {
  interface: string;
  port: number;
  broadcastAddress: string;
  deviceId: number;
}

// BACnet 구독 정보
interface BACnetSubscription {
  id: string;
  deviceId: number;
  objectType: string;
  objectInstance: number;
  subscribedAt: Date;
}

// BACnet 포인트 정보
interface BACnetPoint {
  deviceId: number;
  objectType: string;
  objectInstance: number;
  name: string;
  description: string;
  units: string;
}
```

## 6.5 이벤트 버스

### 6.5.1 이벤트 기반 통합

```typescript
import { EventEmitter } from 'events';
import Redis from 'ioredis';

// 이벤트 타입
export enum FacilityEventType {
  // 시설 이벤트
  FACILITY_STATUS_CHANGED = 'facility.status.changed',
  FACILITY_ALERT_TRIGGERED = 'facility.alert.triggered',
  FACILITY_ALERT_RESOLVED = 'facility.alert.resolved',

  // 장비 이벤트
  EQUIPMENT_STATUS_CHANGED = 'equipment.status.changed',
  EQUIPMENT_READING = 'equipment.reading',
  EQUIPMENT_ALARM = 'equipment.alarm',
  EQUIPMENT_COMMAND_EXECUTED = 'equipment.command.executed',

  // 검체 이벤트
  SPECIMEN_REGISTERED = 'specimen.registered',
  SPECIMEN_TRANSFERRED = 'specimen.transferred',
  SPECIMEN_DISPOSED = 'specimen.disposed',

  // 환경 이벤트
  TEMPERATURE_EXCURSION = 'environment.temperature.excursion',
  POWER_EVENT = 'environment.power.event',
  SECURITY_EVENT = 'environment.security.event',

  // 유지보수 이벤트
  MAINTENANCE_SCHEDULED = 'maintenance.scheduled',
  MAINTENANCE_STARTED = 'maintenance.started',
  MAINTENANCE_COMPLETED = 'maintenance.completed'
}

// 이벤트 메시지
interface FacilityEvent {
  id: string;
  type: FacilityEventType;
  source: string;
  facilityId: string;
  timestamp: Date;
  payload: any;
  metadata?: {
    correlationId?: string;
    causedBy?: string;
    version?: string;
  };
}

// 이벤트 핸들러
type EventHandler = (event: FacilityEvent) => Promise<void>;

// 시설 이벤트 버스
export class FacilityEventBus extends EventEmitter {
  private redis: Redis;
  private subscriber: Redis;
  private handlers: Map<FacilityEventType, EventHandler[]> = new Map();
  private deadLetterQueue: FacilityEvent[] = [];

  constructor(private config: EventBusConfig) {
    super();

    // Redis 연결
    this.redis = new Redis(config.redis);
    this.subscriber = new Redis(config.redis);

    this.setupSubscriber();
  }

  // 구독자 설정
  private setupSubscriber(): void {
    this.subscriber.on('message', async (channel, message) => {
      try {
        const event: FacilityEvent = JSON.parse(message);
        await this.processEvent(event);
      } catch (error) {
        console.error('이벤트 처리 오류:', error);
      }
    });

    // 모든 시설 이벤트 채널 구독
    this.subscriber.psubscribe('facility:*');
  }

  // 이벤트 발행
  async publish(event: Omit<FacilityEvent, 'id' | 'timestamp'>): Promise<void> {
    const fullEvent: FacilityEvent = {
      ...event,
      id: this.generateEventId(),
      timestamp: new Date()
    };

    const channel = `facility:${event.facilityId}:${event.type}`;

    // Redis에 발행
    await this.redis.publish(channel, JSON.stringify(fullEvent));

    // 이벤트 저장 (이벤트 소싱)
    await this.storeEvent(fullEvent);

    this.emit('published', fullEvent);
  }

  // 이벤트 구독
  subscribe(eventType: FacilityEventType, handler: EventHandler): void {
    const handlers = this.handlers.get(eventType) || [];
    handlers.push(handler);
    this.handlers.set(eventType, handlers);
  }

  // 구독 해제
  unsubscribe(eventType: FacilityEventType, handler: EventHandler): void {
    const handlers = this.handlers.get(eventType) || [];
    const index = handlers.indexOf(handler);
    if (index !== -1) {
      handlers.splice(index, 1);
    }
  }

  // 이벤트 처리
  private async processEvent(event: FacilityEvent): Promise<void> {
    const handlers = this.handlers.get(event.type) || [];

    for (const handler of handlers) {
      try {
        await this.executeWithRetry(handler, event);
      } catch (error) {
        // 데드 레터 큐에 추가
        this.deadLetterQueue.push(event);
        this.emit('handler-error', { event, error });
      }
    }
  }

  // 재시도 로직
  private async executeWithRetry(
    handler: EventHandler,
    event: FacilityEvent,
    maxRetries: number = 3
  ): Promise<void> {
    let lastError: Error | undefined;

    for (let attempt = 0; attempt < maxRetries; attempt++) {
      try {
        await handler(event);
        return;
      } catch (error) {
        lastError = error instanceof Error ? error : new Error(String(error));
        await this.delay(Math.pow(2, attempt) * 100);
      }
    }

    throw lastError;
  }

  // 이벤트 저장
  private async storeEvent(event: FacilityEvent): Promise<void> {
    const key = `events:${event.facilityId}`;
    const score = event.timestamp.getTime();

    // 정렬된 집합에 저장
    await this.redis.zadd(key, score, JSON.stringify(event));

    // 보존 기간 후 자동 삭제 (90일)
    const cutoff = Date.now() - 90 * 24 * 60 * 60 * 1000;
    await this.redis.zremrangebyscore(key, 0, cutoff);
  }

  // 이벤트 이력 조회
  async getEventHistory(
    facilityId: string,
    options: {
      from?: Date;
      to?: Date;
      types?: FacilityEventType[];
      limit?: number;
    } = {}
  ): Promise<FacilityEvent[]> {
    const key = `events:${facilityId}`;
    const from = options.from?.getTime() || 0;
    const to = options.to?.getTime() || Date.now();
    const limit = options.limit || 100;

    const results = await this.redis.zrangebyscore(
      key,
      from,
      to,
      'LIMIT',
      0,
      limit
    );

    let events = results.map(r => JSON.parse(r) as FacilityEvent);

    if (options.types?.length) {
      events = events.filter(e => options.types!.includes(e.type));
    }

    return events;
  }

  // 이벤트 리플레이
  async replayEvents(
    facilityId: string,
    from: Date,
    to: Date,
    handler: EventHandler
  ): Promise<number> {
    const events = await this.getEventHistory(facilityId, { from, to });

    let processedCount = 0;

    for (const event of events) {
      try {
        await handler(event);
        processedCount++;
      } catch (error) {
        console.error(`이벤트 리플레이 오류: ${event.id}`, error);
      }
    }

    return processedCount;
  }

  // 데드 레터 큐 처리
  async processDeadLetterQueue(): Promise<void> {
    while (this.deadLetterQueue.length > 0) {
      const event = this.deadLetterQueue.shift()!;

      try {
        await this.processEvent(event);
      } catch (error) {
        // 영구 실패 - 알림 또는 수동 처리 필요
        this.emit('permanent-failure', { event, error });
      }
    }
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  private generateEventId(): string {
    return `evt_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  async close(): Promise<void> {
    await this.redis.quit();
    await this.subscriber.quit();
  }
}

// 이벤트 버스 설정
interface EventBusConfig {
  redis: {
    host: string;
    port: number;
    password?: string;
  };
}

// 이벤트 핸들러 등록 예시
async function setupEventHandlers(eventBus: FacilityEventBus): Promise<void> {
  // 온도 이탈 이벤트 핸들러
  eventBus.subscribe(
    FacilityEventType.TEMPERATURE_EXCURSION,
    async (event) => {
      console.log('온도 이탈 감지:', event.payload);

      // 알림 발송
      await notificationService.sendAlert({
        type: 'temperature-excursion',
        facilityId: event.facilityId,
        severity: event.payload.severity,
        message: `온도 이탈: ${event.payload.currentTemp}°C (정상 범위: ${event.payload.minTemp}°C ~ ${event.payload.maxTemp}°C)`
      });

      // LIMS에 품질 이벤트 기록
      await limsAdapter.send({
        id: '',
        type: 'quality-event',
        source: 'cryo-facility',
        target: 'lims',
        payload: {
          type: 'TEMPERATURE_EXCURSION',
          affectedSpecimens: event.payload.affectedSpecimens,
          temperatureExcursion: {
            min: event.payload.minTemp,
            max: event.payload.maxTemp,
            current: event.payload.currentTemp
          }
        },
        timestamp: new Date()
      });
    }
  );

  // 장비 상태 변경 핸들러
  eventBus.subscribe(
    FacilityEventType.EQUIPMENT_STATUS_CHANGED,
    async (event) => {
      console.log('장비 상태 변경:', event.payload);

      // 대시보드 업데이트
      await dashboardService.updateEquipmentStatus(
        event.payload.equipmentId,
        event.payload.newStatus
      );

      // 고장 상태인 경우 유지보수 작업 생성
      if (event.payload.newStatus === 'fault') {
        await maintenanceService.createUrgentTask({
          equipmentId: event.payload.equipmentId,
          type: 'corrective',
          priority: 'high',
          description: `장비 고장: ${event.payload.faultCode}`
        });
      }
    }
  );

  // 검체 이동 핸들러
  eventBus.subscribe(
    FacilityEventType.SPECIMEN_TRANSFERRED,
    async (event) => {
      console.log('검체 이동:', event.payload);

      // LIMS 위치 업데이트
      await limsAdapter.send({
        id: '',
        type: 'storage-location-update',
        source: 'cryo-facility',
        target: 'lims',
        payload: {
          specimenIds: [event.payload.specimenId],
          facilityId: event.facilityId,
          equipmentId: event.payload.toEquipmentId,
          position: event.payload.toPosition
        },
        timestamp: new Date()
      });

      // 감사 로그
      await auditLogger.log({
        action: 'SPECIMEN_TRANSFERRED',
        specimenId: event.payload.specimenId,
        fromLocation: event.payload.fromLocation,
        toLocation: event.payload.toLocation,
        transferredBy: event.payload.userId
      });
    }
  );
}
```

## 6.6 요약

이 장에서는 극저온 시설의 통합 패턴을 상세히 다루었습니다:

| 통합 영역 | 프로토콜 | 주요 기능 |
|----------|----------|----------|
| LIMS | REST API | 검체 등록, 이동, 품질 이벤트 |
| BMS | BACnet | HVAC, 전력, 환경 모니터링 |
| 이벤트 버스 | Redis Pub/Sub | 이벤트 기반 통합 |
| 통합 허브 | 다중 프로토콜 | 회로 차단기, 재시도, 라우팅 |

핵심 통합 패턴:
- 회로 차단기를 통한 장애 격리
- 이벤트 소싱을 통한 감사 추적
- 재시도 로직과 데드 레터 큐
- 실시간 양방향 통신

다음 장에서는 보안 아키텍처를 살펴봅니다.

---

© 2025 WIA Standards. All rights reserved.
