# Chapter 6: Cryogenic Facility System Integration

## Enterprise Integration Patterns and External System Connectivity

This chapter provides comprehensive guidance on integrating cryogenic facility management systems with enterprise systems, laboratory information management systems (LIMS), building automation, and external service providers.

---

## Integration Architecture Overview

### Enterprise Integration Patterns

```typescript
/**
 * WIA Cryo Facility Integration Architecture
 * Enterprise connectivity framework
 */

interface IntegrationArchitecture {
  // Core integration hub
  hub: IntegrationHub;

  // Integration adapters
  adapters: IntegrationAdapter[];

  // Message routing
  routing: MessageRoutingConfig;

  // Data transformation
  transformation: TransformationEngine;

  // Event processing
  events: EventProcessingConfig;
}

interface IntegrationHub {
  name: string;
  type: 'enterprise-service-bus' | 'api-gateway' | 'message-broker';
  endpoints: HubEndpoint[];
  protocols: string[];
  security: SecurityConfig;
  monitoring: MonitoringConfig;
}

interface IntegrationAdapter {
  id: string;
  name: string;
  targetSystem: string;
  type: AdapterType;
  protocol: string;
  config: AdapterConfig;
  mappings: DataMapping[];
  errorHandling: ErrorHandlingConfig;
}

type AdapterType =
  | 'lims'
  | 'building-management'
  | 'erp'
  | 'crm'
  | 'compliance'
  | 'analytics'
  | 'external-service';

interface HubEndpoint {
  name: string;
  protocol: string;
  url: string;
  authentication: AuthenticationConfig;
}

interface AdapterConfig {
  connectionString?: string;
  credentials?: CredentialsConfig;
  timeout: number;
  retryPolicy: RetryPolicy;
  circuitBreaker: CircuitBreakerConfig;
}

interface DataMapping {
  source: FieldMapping;
  target: FieldMapping;
  transformation?: string;
  required: boolean;
}

interface FieldMapping {
  system: string;
  field: string;
  type: string;
}

interface CircuitBreakerConfig {
  failureThreshold: number;
  resetTimeout: number;
  halfOpenRequests: number;
}

// Integration Hub implementation
class CryoFacilityIntegrationHub {
  private adapters: Map<string, IntegrationAdapter> = new Map();
  private messageQueue: MessageQueue;
  private eventBus: EventBus;
  private transformationEngine: TransformationEngine;

  constructor(config: IntegrationHubConfig) {
    this.messageQueue = new MessageQueue(config.queue);
    this.eventBus = new EventBus(config.events);
    this.transformationEngine = new TransformationEngine(config.transformations);
  }

  registerAdapter(adapter: IntegrationAdapter): void {
    this.adapters.set(adapter.id, adapter);
  }

  async sendMessage(
    targetAdapter: string,
    message: IntegrationMessage
  ): Promise<MessageResult> {
    const adapter = this.adapters.get(targetAdapter);
    if (!adapter) {
      throw new Error(`Adapter not found: ${targetAdapter}`);
    }

    // Transform message
    const transformedMessage = await this.transformationEngine.transform(
      message,
      adapter.mappings
    );

    // Send through adapter
    return this.executeAdapterRequest(adapter, transformedMessage);
  }

  async publishEvent(event: IntegrationEvent): Promise<void> {
    await this.eventBus.publish(event);
  }

  subscribeToEvents(
    eventType: string,
    handler: (event: IntegrationEvent) => Promise<void>
  ): void {
    this.eventBus.subscribe(eventType, handler);
  }

  private async executeAdapterRequest(
    adapter: IntegrationAdapter,
    message: IntegrationMessage
  ): Promise<MessageResult> {
    // Circuit breaker check
    const breaker = this.getCircuitBreaker(adapter.id);
    if (breaker.isOpen()) {
      return {
        success: false,
        error: 'Circuit breaker open - service temporarily unavailable'
      };
    }

    try {
      const result = await this.sendToAdapter(adapter, message);
      breaker.recordSuccess();
      return result;
    } catch (error) {
      breaker.recordFailure();
      throw error;
    }
  }

  private circuitBreakers: Map<string, CircuitBreaker> = new Map();

  private getCircuitBreaker(adapterId: string): CircuitBreaker {
    if (!this.circuitBreakers.has(adapterId)) {
      const adapter = this.adapters.get(adapterId)!;
      this.circuitBreakers.set(
        adapterId,
        new CircuitBreaker(adapter.config.circuitBreaker)
      );
    }
    return this.circuitBreakers.get(adapterId)!;
  }

  private async sendToAdapter(
    adapter: IntegrationAdapter,
    message: IntegrationMessage
  ): Promise<MessageResult> {
    // Implementation based on adapter type and protocol
    switch (adapter.protocol) {
      case 'rest':
        return this.sendRestRequest(adapter, message);
      case 'soap':
        return this.sendSoapRequest(adapter, message);
      case 'hl7-fhir':
        return this.sendFhirRequest(adapter, message);
      case 'hl7-v2':
        return this.sendHl7V2Message(adapter, message);
      default:
        throw new Error(`Unsupported protocol: ${adapter.protocol}`);
    }
  }

  private async sendRestRequest(
    adapter: IntegrationAdapter,
    message: IntegrationMessage
  ): Promise<MessageResult> {
    // REST API request implementation
    return { success: true };
  }

  private async sendSoapRequest(
    adapter: IntegrationAdapter,
    message: IntegrationMessage
  ): Promise<MessageResult> {
    // SOAP request implementation
    return { success: true };
  }

  private async sendFhirRequest(
    adapter: IntegrationAdapter,
    message: IntegrationMessage
  ): Promise<MessageResult> {
    // HL7 FHIR request implementation
    return { success: true };
  }

  private async sendHl7V2Message(
    adapter: IntegrationAdapter,
    message: IntegrationMessage
  ): Promise<MessageResult> {
    // HL7 v2 message implementation
    return { success: true };
  }
}

interface IntegrationMessage {
  id: string;
  type: string;
  payload: unknown;
  metadata: MessageMetadata;
  timestamp: string;
}

interface MessageMetadata {
  correlationId?: string;
  replyTo?: string;
  priority?: number;
  ttl?: number;
}

interface MessageResult {
  success: boolean;
  data?: unknown;
  error?: string;
}

interface IntegrationEvent {
  id: string;
  type: string;
  source: string;
  payload: unknown;
  timestamp: string;
}

class CircuitBreaker {
  private failures: number = 0;
  private state: 'closed' | 'open' | 'half-open' = 'closed';
  private lastFailure: Date | null = null;
  private config: CircuitBreakerConfig;

  constructor(config: CircuitBreakerConfig) {
    this.config = config;
  }

  isOpen(): boolean {
    if (this.state === 'open') {
      const elapsed = Date.now() - (this.lastFailure?.getTime() || 0);
      if (elapsed > this.config.resetTimeout) {
        this.state = 'half-open';
        return false;
      }
      return true;
    }
    return false;
  }

  recordSuccess(): void {
    this.failures = 0;
    this.state = 'closed';
  }

  recordFailure(): void {
    this.failures++;
    this.lastFailure = new Date();
    if (this.failures >= this.config.failureThreshold) {
      this.state = 'open';
    }
  }
}

interface IntegrationHubConfig {
  queue: QueueConfig;
  events: EventBusConfig;
  transformations: TransformationConfig;
}

interface QueueConfig {
  type: string;
  connectionString: string;
}

interface EventBusConfig {
  type: string;
  connectionString: string;
}

interface TransformationConfig {
  schemas: string[];
}

class MessageQueue {
  constructor(config: QueueConfig) {}
}

class EventBus {
  private handlers: Map<string, ((event: IntegrationEvent) => Promise<void>)[]> = new Map();

  constructor(config: EventBusConfig) {}

  async publish(event: IntegrationEvent): Promise<void> {
    const handlers = this.handlers.get(event.type) || [];
    for (const handler of handlers) {
      await handler(event);
    }
  }

  subscribe(
    eventType: string,
    handler: (event: IntegrationEvent) => Promise<void>
  ): void {
    const handlers = this.handlers.get(eventType) || [];
    handlers.push(handler);
    this.handlers.set(eventType, handlers);
  }
}

class TransformationEngine {
  constructor(config: TransformationConfig) {}

  async transform(
    message: IntegrationMessage,
    mappings: DataMapping[]
  ): Promise<IntegrationMessage> {
    // Apply transformations
    return message;
  }
}
```

---

## LIMS Integration

### Laboratory Information Management System

```typescript
/**
 * LIMS Integration Adapter
 * Specimen and sample tracking integration
 */

interface LIMSIntegration {
  // Specimen operations
  specimens: SpecimenOperations;

  // Container management
  containers: ContainerOperations;

  // Workflow integration
  workflows: WorkflowOperations;

  // Quality control
  qualityControl: QualityOperations;

  // Reporting
  reporting: ReportingOperations;
}

interface SpecimenOperations {
  register(specimen: SpecimenData): Promise<SpecimenResult>;
  update(id: string, updates: Partial<SpecimenData>): Promise<SpecimenResult>;
  transfer(id: string, destination: LocationData): Promise<TransferResult>;
  retrieve(id: string): Promise<SpecimenData>;
  dispose(id: string, reason: string): Promise<DisposalResult>;
  search(criteria: SearchCriteria): Promise<SpecimenData[]>;
}

interface ContainerOperations {
  register(container: ContainerData): Promise<ContainerResult>;
  updateLocation(id: string, location: LocationData): Promise<void>;
  getContents(id: string): Promise<SpecimenData[]>;
  checkCapacity(id: string): Promise<CapacityResult>;
}

interface WorkflowOperations {
  initiate(workflow: string, context: WorkflowContext): Promise<WorkflowResult>;
  advance(instanceId: string, action: string): Promise<WorkflowResult>;
  getStatus(instanceId: string): Promise<WorkflowStatus>;
}

interface QualityOperations {
  recordResult(test: QualityTestData): Promise<void>;
  getHistory(specimenId: string): Promise<QualityTestData[]>;
  flagDeviation(deviation: DeviationData): Promise<void>;
}

interface ReportingOperations {
  generate(reportType: string, parameters: ReportParams): Promise<Report>;
  schedule(config: ReportSchedule): Promise<void>;
}

class LIMSAdapter implements IntegrationAdapter {
  id = 'lims-adapter';
  name = 'LIMS Integration Adapter';
  targetSystem = 'laboratory-information-system';
  type: AdapterType = 'lims';
  protocol = 'rest';
  config: AdapterConfig;
  mappings: DataMapping[];
  errorHandling: ErrorHandlingConfig;

  private apiClient: LIMSApiClient;

  constructor(config: LIMSAdapterConfig) {
    this.config = config;
    this.mappings = config.mappings;
    this.errorHandling = config.errorHandling;
    this.apiClient = new LIMSApiClient(config.connection);
  }

  // Specimen Operations
  async registerSpecimen(specimen: CryoSpecimen): Promise<LIMSSpecimen> {
    const limsSpecimen = this.mapToLIMSFormat(specimen);

    const response = await this.apiClient.post('/specimens', limsSpecimen);

    return this.mapFromLIMSFormat(response.data);
  }

  async updateSpecimenLocation(
    specimenId: string,
    location: CryoLocation
  ): Promise<void> {
    const limsLocation = this.mapLocationToLIMS(location);

    await this.apiClient.put(`/specimens/${specimenId}/location`, limsLocation);
  }

  async getSpecimen(specimenId: string): Promise<CryoSpecimen> {
    const response = await this.apiClient.get(`/specimens/${specimenId}`);
    return this.mapFromLIMSFormat(response.data);
  }

  async searchSpecimens(criteria: CryoSearchCriteria): Promise<CryoSpecimen[]> {
    const limsQuery = this.buildLIMSQuery(criteria);
    const response = await this.apiClient.get('/specimens', { params: limsQuery });

    return response.data.map((item: any) => this.mapFromLIMSFormat(item));
  }

  async recordTransfer(transfer: SpecimenTransfer): Promise<TransferRecord> {
    const limsTransfer = {
      specimenId: transfer.specimenId,
      fromLocation: this.mapLocationToLIMS(transfer.from),
      toLocation: this.mapLocationToLIMS(transfer.to),
      transferredBy: transfer.performedBy,
      transferredAt: transfer.timestamp,
      reason: transfer.reason,
      conditions: {
        temperature: transfer.temperature,
        transportMethod: transfer.method
      }
    };

    const response = await this.apiClient.post('/transfers', limsTransfer);

    return {
      id: response.data.id,
      status: 'completed',
      ...transfer
    };
  }

  // Container Operations
  async registerContainer(container: CryoContainer): Promise<LIMSContainer> {
    const limsContainer = {
      barcode: container.id,
      type: container.type,
      location: this.mapLocationToLIMS(container.location),
      capacity: container.capacity,
      currentCount: container.specimenCount,
      equipment: container.equipmentId
    };

    const response = await this.apiClient.post('/containers', limsContainer);

    return response.data;
  }

  async getContainerContents(containerId: string): Promise<CryoSpecimen[]> {
    const response = await this.apiClient.get(`/containers/${containerId}/specimens`);
    return response.data.map((item: any) => this.mapFromLIMSFormat(item));
  }

  // Quality Operations
  async recordQualityEvent(event: QualityEvent): Promise<void> {
    const limsEvent = {
      specimenId: event.specimenId,
      eventType: event.type,
      eventTime: event.timestamp,
      parameters: event.parameters,
      result: event.result,
      performedBy: event.performedBy,
      notes: event.notes
    };

    await this.apiClient.post('/quality-events', limsEvent);
  }

  async recordDeviation(deviation: CryoDeviation): Promise<string> {
    const limsDeviation = {
      type: deviation.type,
      description: deviation.description,
      detectedAt: deviation.detectedAt,
      affectedItems: deviation.affectedSpecimens.map(s => ({
        type: 'specimen',
        id: s
      })),
      severity: deviation.severity,
      immediateActions: deviation.immediateActions,
      reportedBy: deviation.reportedBy
    };

    const response = await this.apiClient.post('/deviations', limsDeviation);

    return response.data.deviationId;
  }

  // Workflow Operations
  async triggerLIMSWorkflow(
    workflowType: string,
    context: WorkflowContext
  ): Promise<string> {
    const limsWorkflow = {
      type: workflowType,
      initiatedBy: context.userId,
      parameters: context.parameters,
      relatedSpecimens: context.parameters['specimenIds'] || []
    };

    const response = await this.apiClient.post('/workflows', limsWorkflow);

    return response.data.workflowId;
  }

  // Mapping functions
  private mapToLIMSFormat(specimen: CryoSpecimen): any {
    return {
      barcode: specimen.id,
      sampleType: specimen.type,
      collectionDate: specimen.collectionDate,
      donorId: specimen.donorId,
      location: this.mapLocationToLIMS(specimen.location),
      volume: specimen.volume,
      volumeUnit: specimen.volumeUnit,
      preservationType: specimen.preservationType,
      customAttributes: specimen.metadata
    };
  }

  private mapFromLIMSFormat(limsData: any): CryoSpecimen {
    return {
      id: limsData.barcode,
      type: limsData.sampleType,
      collectionDate: limsData.collectionDate,
      donorId: limsData.donorId,
      location: this.mapLocationFromLIMS(limsData.location),
      volume: limsData.volume,
      volumeUnit: limsData.volumeUnit,
      preservationType: limsData.preservationType,
      status: limsData.status,
      metadata: limsData.customAttributes
    };
  }

  private mapLocationToLIMS(location: CryoLocation): any {
    return {
      facilityCode: location.facilityId,
      equipmentId: location.equipmentId,
      containerBarcode: location.containerId,
      position: location.position,
      zoneId: location.zone
    };
  }

  private mapLocationFromLIMS(limsLocation: any): CryoLocation {
    return {
      facilityId: limsLocation.facilityCode,
      equipmentId: limsLocation.equipmentId,
      containerId: limsLocation.containerBarcode,
      position: limsLocation.position,
      zone: limsLocation.zoneId
    };
  }

  private buildLIMSQuery(criteria: CryoSearchCriteria): any {
    const query: any = {};

    if (criteria.type) query.sampleType = criteria.type;
    if (criteria.status) query.status = criteria.status;
    if (criteria.donorId) query.donorId = criteria.donorId;
    if (criteria.dateRange) {
      query.collectionDateFrom = criteria.dateRange.start;
      query.collectionDateTo = criteria.dateRange.end;
    }
    if (criteria.location) {
      query.facilityCode = criteria.location.facilityId;
      query.equipmentId = criteria.location.equipmentId;
    }

    return query;
  }
}

interface CryoSpecimen {
  id: string;
  type: string;
  collectionDate: string;
  donorId: string;
  location: CryoLocation;
  volume: number;
  volumeUnit: string;
  preservationType: string;
  status?: string;
  metadata?: Record<string, unknown>;
}

interface CryoLocation {
  facilityId: string;
  equipmentId: string;
  containerId: string;
  position: string;
  zone: string;
}

interface CryoContainer {
  id: string;
  type: string;
  location: CryoLocation;
  capacity: number;
  specimenCount: number;
  equipmentId: string;
}

interface SpecimenTransfer {
  specimenId: string;
  from: CryoLocation;
  to: CryoLocation;
  performedBy: string;
  timestamp: string;
  reason: string;
  temperature: number;
  method: string;
}

interface TransferRecord extends SpecimenTransfer {
  id: string;
  status: string;
}

interface QualityEvent {
  specimenId: string;
  type: string;
  timestamp: string;
  parameters: Record<string, unknown>;
  result: string;
  performedBy: string;
  notes?: string;
}

interface CryoDeviation {
  type: string;
  description: string;
  detectedAt: string;
  affectedSpecimens: string[];
  severity: string;
  immediateActions: string[];
  reportedBy: string;
}

interface CryoSearchCriteria {
  type?: string;
  status?: string;
  donorId?: string;
  dateRange?: { start: string; end: string };
  location?: Partial<CryoLocation>;
}

interface LIMSAdapterConfig extends AdapterConfig {
  connection: LIMSConnectionConfig;
  mappings: DataMapping[];
  errorHandling: ErrorHandlingConfig;
}

interface LIMSConnectionConfig {
  baseUrl: string;
  apiKey: string;
  timeout: number;
}

class LIMSApiClient {
  private baseUrl: string;
  private apiKey: string;
  private timeout: number;

  constructor(config: LIMSConnectionConfig) {
    this.baseUrl = config.baseUrl;
    this.apiKey = config.apiKey;
    this.timeout = config.timeout;
  }

  async get(path: string, options?: any): Promise<any> {
    // HTTP GET implementation
    return { data: {} };
  }

  async post(path: string, data: any): Promise<any> {
    // HTTP POST implementation
    return { data: {} };
  }

  async put(path: string, data: any): Promise<any> {
    // HTTP PUT implementation
    return { data: {} };
  }
}

// Additional interfaces for type completeness
interface SpecimenData {}
interface LocationData {}
interface SpecimenResult {}
interface TransferResult {}
interface DisposalResult {}
interface SearchCriteria {}
interface ContainerData {}
interface ContainerResult {}
interface CapacityResult {}
interface WorkflowStatus {}
interface QualityTestData {}
interface DeviationData {}
interface ReportParams {}
interface Report {}
interface ReportSchedule {}
interface ErrorHandlingConfig {}
interface LIMSSpecimen {}
interface LIMSContainer {}
```

---

## Building Management Integration

### BMS/BAS Connectivity

```typescript
/**
 * Building Management System Integration
 * Environmental control and monitoring integration
 */

interface BMSIntegration {
  // Environmental monitoring
  environmental: BMSEnvironmentalOperations;

  // HVAC control
  hvac: HVACOperations;

  // Power management
  power: PowerOperations;

  // Security systems
  security: SecurityOperations;

  // Alarm management
  alarms: AlarmOperations;
}

interface BMSEnvironmentalOperations {
  getReadings(zone: string): Promise<EnvironmentalReadings>;
  getHistory(zone: string, timeRange: TimeRange): Promise<EnvironmentalHistory>;
  setSetpoints(zone: string, setpoints: Setpoints): Promise<void>;
}

interface HVACOperations {
  getStatus(zone: string): Promise<HVACStatus>;
  setMode(zone: string, mode: HVACMode): Promise<void>;
  adjustTemperature(zone: string, target: number): Promise<void>;
}

interface PowerOperations {
  getStatus(): Promise<PowerStatus>;
  getLoad(): Promise<LoadMetrics>;
  switchToBackup(): Promise<void>;
}

interface SecurityOperations {
  getAccessLog(area: string, timeRange: TimeRange): Promise<AccessLogEntry[]>;
  grantAccess(userId: string, area: string, duration: number): Promise<void>;
  revokeAccess(userId: string, area: string): Promise<void>;
}

interface AlarmOperations {
  getActiveAlarms(): Promise<BMSAlarm[]>;
  acknowledgeAlarm(alarmId: string): Promise<void>;
  configureAlarm(config: AlarmConfig): Promise<void>;
}

class BMSAdapter implements IntegrationAdapter {
  id = 'bms-adapter';
  name = 'Building Management System Adapter';
  targetSystem = 'building-automation-system';
  type: AdapterType = 'building-management';
  protocol = 'bacnet';
  config: AdapterConfig;
  mappings: DataMapping[];
  errorHandling: ErrorHandlingConfig;

  private bacnetClient: BACnetClient;

  constructor(config: BMSAdapterConfig) {
    this.config = config;
    this.mappings = config.mappings;
    this.errorHandling = config.errorHandling;
    this.bacnetClient = new BACnetClient(config.connection);
  }

  // Environmental monitoring
  async getEnvironmentalReadings(zoneId: string): Promise<EnvironmentalReadings> {
    const points = await this.getZonePoints(zoneId);

    const readings: EnvironmentalReadings = {
      zoneId,
      timestamp: new Date().toISOString(),
      temperature: await this.bacnetClient.readProperty(points.temperaturePoint),
      humidity: await this.bacnetClient.readProperty(points.humidityPoint),
      pressure: await this.bacnetClient.readProperty(points.pressurePoint),
      airflow: await this.bacnetClient.readProperty(points.airflowPoint),
      particulate: await this.bacnetClient.readProperty(points.particulatePoint)
    };

    return readings;
  }

  async getEnvironmentalHistory(
    zoneId: string,
    timeRange: TimeRange
  ): Promise<EnvironmentalHistory> {
    const points = await this.getZonePoints(zoneId);

    const history = await this.bacnetClient.readTrend(
      points.temperaturePoint,
      timeRange.start,
      timeRange.end
    );

    return {
      zoneId,
      timeRange,
      readings: history.map((record: any) => ({
        timestamp: record.timestamp,
        temperature: record.value,
        humidity: record.humidity,
        pressure: record.pressure
      }))
    };
  }

  async setZoneSetpoints(
    zoneId: string,
    setpoints: EnvironmentalSetpoints
  ): Promise<void> {
    const points = await this.getZonePoints(zoneId);

    if (setpoints.temperature) {
      await this.bacnetClient.writeProperty(
        points.temperatureSetpoint,
        setpoints.temperature
      );
    }

    if (setpoints.humidity) {
      await this.bacnetClient.writeProperty(
        points.humiditySetpoint,
        setpoints.humidity
      );
    }

    if (setpoints.pressure) {
      await this.bacnetClient.writeProperty(
        points.pressureSetpoint,
        setpoints.pressure
      );
    }
  }

  // HVAC control
  async getHVACStatus(zoneId: string): Promise<HVACStatus> {
    const points = await this.getZonePoints(zoneId);

    return {
      zoneId,
      mode: await this.bacnetClient.readProperty(points.hvacMode),
      status: await this.bacnetClient.readProperty(points.hvacStatus),
      supplyAirTemp: await this.bacnetClient.readProperty(points.supplyAirTemp),
      returnAirTemp: await this.bacnetClient.readProperty(points.returnAirTemp),
      fanSpeed: await this.bacnetClient.readProperty(points.fanSpeed),
      damperPosition: await this.bacnetClient.readProperty(points.damperPosition),
      filterStatus: await this.bacnetClient.readProperty(points.filterStatus)
    };
  }

  async setHVACMode(zoneId: string, mode: HVACMode): Promise<void> {
    const points = await this.getZonePoints(zoneId);
    await this.bacnetClient.writeProperty(points.hvacMode, mode);
  }

  // Power management
  async getPowerStatus(): Promise<PowerStatus> {
    const powerPoints = this.config.powerPoints;

    return {
      mainPower: {
        status: await this.bacnetClient.readProperty(powerPoints.mainStatus),
        voltage: await this.bacnetClient.readProperty(powerPoints.mainVoltage),
        current: await this.bacnetClient.readProperty(powerPoints.mainCurrent),
        frequency: await this.bacnetClient.readProperty(powerPoints.mainFrequency)
      },
      ups: {
        status: await this.bacnetClient.readProperty(powerPoints.upsStatus),
        batteryLevel: await this.bacnetClient.readProperty(powerPoints.upsBattery),
        loadPercent: await this.bacnetClient.readProperty(powerPoints.upsLoad),
        estimatedRuntime: await this.bacnetClient.readProperty(powerPoints.upsRuntime)
      },
      generator: {
        status: await this.bacnetClient.readProperty(powerPoints.genStatus),
        fuelLevel: await this.bacnetClient.readProperty(powerPoints.genFuel),
        runtime: await this.bacnetClient.readProperty(powerPoints.genRuntime),
        lastTest: await this.bacnetClient.readProperty(powerPoints.genLastTest)
      }
    };
  }

  async initiateGeneratorStart(): Promise<void> {
    const powerPoints = this.config.powerPoints;
    await this.bacnetClient.writeProperty(powerPoints.genStartCommand, true);
  }

  // Alarm management
  async getActiveAlarms(): Promise<BMSAlarm[]> {
    const alarms = await this.bacnetClient.readAlarmSummary();

    return alarms.map((alarm: any) => ({
      id: alarm.objectIdentifier,
      source: alarm.source,
      type: alarm.alarmType,
      state: alarm.alarmState,
      priority: alarm.priority,
      timestamp: alarm.timestamp,
      description: alarm.description,
      acknowledged: alarm.acknowledged
    }));
  }

  async acknowledgeAlarm(alarmId: string, userId: string): Promise<void> {
    await this.bacnetClient.acknowledgeAlarm(alarmId, userId);
  }

  // Subscribe to real-time updates
  async subscribeToChanges(
    zoneId: string,
    callback: (update: EnvironmentalUpdate) => void
  ): Promise<Subscription> {
    const points = await this.getZonePoints(zoneId);

    const subscription = await this.bacnetClient.subscribeCOV(
      [points.temperaturePoint, points.humidityPoint, points.pressurePoint],
      (notification: any) => {
        callback({
          zoneId,
          point: notification.objectIdentifier,
          value: notification.value,
          timestamp: new Date().toISOString()
        });
      }
    );

    return subscription;
  }

  private async getZonePoints(zoneId: string): Promise<ZonePoints> {
    // Get BACnet point mappings for zone
    return this.config.zoneMappings[zoneId];
  }
}

interface EnvironmentalReadings {
  zoneId: string;
  timestamp: string;
  temperature: number;
  humidity: number;
  pressure: number;
  airflow: number;
  particulate: number;
}

interface EnvironmentalHistory {
  zoneId: string;
  timeRange: TimeRange;
  readings: {
    timestamp: string;
    temperature: number;
    humidity: number;
    pressure: number;
  }[];
}

interface EnvironmentalSetpoints {
  temperature?: number;
  humidity?: number;
  pressure?: number;
}

interface HVACStatus {
  zoneId: string;
  mode: HVACMode;
  status: 'running' | 'stopped' | 'alarm';
  supplyAirTemp: number;
  returnAirTemp: number;
  fanSpeed: number;
  damperPosition: number;
  filterStatus: string;
}

type HVACMode = 'auto' | 'cooling' | 'heating' | 'fan-only' | 'off';

interface PowerStatus {
  mainPower: {
    status: 'normal' | 'failure';
    voltage: number;
    current: number;
    frequency: number;
  };
  ups: {
    status: 'online' | 'battery' | 'bypass';
    batteryLevel: number;
    loadPercent: number;
    estimatedRuntime: number;
  };
  generator: {
    status: 'standby' | 'running' | 'fault';
    fuelLevel: number;
    runtime: number;
    lastTest: string;
  };
}

interface BMSAlarm {
  id: string;
  source: string;
  type: string;
  state: string;
  priority: number;
  timestamp: string;
  description: string;
  acknowledged: boolean;
}

interface EnvironmentalUpdate {
  zoneId: string;
  point: string;
  value: number;
  timestamp: string;
}

interface TimeRange {
  start: string;
  end: string;
}

interface ZonePoints {
  temperaturePoint: string;
  humidityPoint: string;
  pressurePoint: string;
  airflowPoint: string;
  particulatePoint: string;
  temperatureSetpoint: string;
  humiditySetpoint: string;
  pressureSetpoint: string;
  hvacMode: string;
  hvacStatus: string;
  supplyAirTemp: string;
  returnAirTemp: string;
  fanSpeed: string;
  damperPosition: string;
  filterStatus: string;
}

interface BMSAdapterConfig extends AdapterConfig {
  connection: BACnetConnectionConfig;
  zoneMappings: Record<string, ZonePoints>;
  powerPoints: PowerPoints;
  mappings: DataMapping[];
  errorHandling: ErrorHandlingConfig;
}

interface BACnetConnectionConfig {
  networkInterface: string;
  port: number;
  deviceId: number;
}

interface PowerPoints {
  mainStatus: string;
  mainVoltage: string;
  mainCurrent: string;
  mainFrequency: string;
  upsStatus: string;
  upsBattery: string;
  upsLoad: string;
  upsRuntime: string;
  genStatus: string;
  genFuel: string;
  genRuntime: string;
  genLastTest: string;
  genStartCommand: string;
}

interface Subscription {
  unsubscribe(): void;
}

class BACnetClient {
  constructor(config: BACnetConnectionConfig) {}

  async readProperty(point: string): Promise<any> {
    return null;
  }

  async writeProperty(point: string, value: any): Promise<void> {}

  async readTrend(point: string, start: string, end: string): Promise<any[]> {
    return [];
  }

  async readAlarmSummary(): Promise<any[]> {
    return [];
  }

  async acknowledgeAlarm(alarmId: string, userId: string): Promise<void> {}

  async subscribeCOV(points: string[], callback: (notification: any) => void): Promise<Subscription> {
    return { unsubscribe: () => {} };
  }
}
```

---

## Event-Driven Integration

### Event Bus and Messaging

```typescript
/**
 * Event-Driven Integration System
 * Publish-subscribe messaging for facility events
 */

interface FacilityEvent {
  id: string;
  type: FacilityEventType;
  source: EventSource;
  payload: unknown;
  timestamp: string;
  correlationId?: string;
  metadata: EventMetadata;
}

type FacilityEventType =
  // Equipment events
  | 'equipment.temperature.alert'
  | 'equipment.level.alert'
  | 'equipment.status.changed'
  | 'equipment.maintenance.due'
  | 'equipment.calibration.due'
  // Specimen events
  | 'specimen.received'
  | 'specimen.stored'
  | 'specimen.retrieved'
  | 'specimen.transferred'
  | 'specimen.disposed'
  // Environmental events
  | 'environment.threshold.exceeded'
  | 'environment.hvac.alarm'
  | 'environment.power.failure'
  // Operational events
  | 'workflow.started'
  | 'workflow.completed'
  | 'workflow.failed'
  // Quality events
  | 'quality.deviation.detected'
  | 'quality.audit.completed'
  | 'quality.capa.required';

interface EventSource {
  system: string;
  component: string;
  instance?: string;
}

interface EventMetadata {
  version: string;
  producerId: string;
  priority: 'low' | 'normal' | 'high' | 'critical';
  ttl?: number;
  retryCount?: number;
}

class FacilityEventBus {
  private handlers: Map<string, Set<EventHandler>> = new Map();
  private middleware: EventMiddleware[] = [];
  private deadLetterQueue: DeadLetterQueue;
  private eventStore: EventStore;

  constructor(config: EventBusConfig) {
    this.deadLetterQueue = new DeadLetterQueue(config.dlq);
    this.eventStore = new EventStore(config.store);
  }

  // Register middleware
  use(middleware: EventMiddleware): void {
    this.middleware.push(middleware);
  }

  // Subscribe to events
  subscribe(
    eventType: FacilityEventType | '*',
    handler: EventHandler
  ): Subscription {
    const type = eventType === '*' ? '__all__' : eventType;

    if (!this.handlers.has(type)) {
      this.handlers.set(type, new Set());
    }

    this.handlers.get(type)!.add(handler);

    return {
      unsubscribe: () => {
        this.handlers.get(type)?.delete(handler);
      }
    };
  }

  // Publish event
  async publish(event: FacilityEvent): Promise<void> {
    // Store event
    await this.eventStore.store(event);

    // Run middleware
    let processedEvent = event;
    for (const mw of this.middleware) {
      processedEvent = await mw.process(processedEvent);
    }

    // Get handlers for this event type
    const handlers = this.getHandlers(processedEvent.type);

    // Execute handlers
    const results = await Promise.allSettled(
      handlers.map(handler => this.executeHandler(handler, processedEvent))
    );

    // Handle failures
    const failures = results.filter(r => r.status === 'rejected');
    if (failures.length > 0) {
      await this.handleFailures(processedEvent, failures);
    }
  }

  private getHandlers(eventType: FacilityEventType): EventHandler[] {
    const handlers: EventHandler[] = [];

    // Specific handlers
    const specific = this.handlers.get(eventType);
    if (specific) {
      handlers.push(...specific);
    }

    // Wildcard handlers
    const wildcard = this.handlers.get('__all__');
    if (wildcard) {
      handlers.push(...wildcard);
    }

    return handlers;
  }

  private async executeHandler(
    handler: EventHandler,
    event: FacilityEvent
  ): Promise<void> {
    const timeout = event.metadata.ttl || 30000;

    return Promise.race([
      handler.handle(event),
      new Promise<void>((_, reject) =>
        setTimeout(() => reject(new Error('Handler timeout')), timeout)
      )
    ]);
  }

  private async handleFailures(
    event: FacilityEvent,
    failures: PromiseRejectedResult[]
  ): Promise<void> {
    const retryCount = (event.metadata.retryCount || 0) + 1;

    if (retryCount < 3) {
      // Retry with backoff
      const retryEvent = {
        ...event,
        metadata: {
          ...event.metadata,
          retryCount
        }
      };

      setTimeout(
        () => this.publish(retryEvent),
        Math.pow(2, retryCount) * 1000
      );
    } else {
      // Send to dead letter queue
      await this.deadLetterQueue.enqueue({
        event,
        failures: failures.map(f => String(f.reason)),
        timestamp: new Date().toISOString()
      });
    }
  }
}

interface EventHandler {
  handle(event: FacilityEvent): Promise<void>;
}

interface EventMiddleware {
  process(event: FacilityEvent): Promise<FacilityEvent>;
}

class DeadLetterQueue {
  constructor(config: DLQConfig) {}

  async enqueue(item: DeadLetterItem): Promise<void> {
    console.log('Dead letter queue:', item);
  }
}

class EventStore {
  constructor(config: EventStoreConfig) {}

  async store(event: FacilityEvent): Promise<void> {
    console.log('Storing event:', event.id);
  }
}

interface DLQConfig {
  maxSize: number;
  retention: string;
}

interface EventStoreConfig {
  type: string;
  connectionString: string;
}

interface DeadLetterItem {
  event: FacilityEvent;
  failures: string[];
  timestamp: string;
}

// Event handler implementations
class EquipmentAlertHandler implements EventHandler {
  async handle(event: FacilityEvent): Promise<void> {
    if (!event.type.startsWith('equipment.')) return;

    const payload = event.payload as EquipmentAlertPayload;

    console.log(`Equipment alert: ${payload.equipmentId} - ${payload.alertType}`);

    // Process alert
    await this.processAlert(payload);
  }

  private async processAlert(payload: EquipmentAlertPayload): Promise<void> {
    // Alert processing logic
  }
}

class SpecimenEventHandler implements EventHandler {
  private limsAdapter: LIMSAdapter;

  constructor(limsAdapter: LIMSAdapter) {
    this.limsAdapter = limsAdapter;
  }

  async handle(event: FacilityEvent): Promise<void> {
    if (!event.type.startsWith('specimen.')) return;

    const payload = event.payload as SpecimenEventPayload;

    // Sync with LIMS
    switch (event.type) {
      case 'specimen.stored':
        await this.limsAdapter.updateSpecimenLocation(
          payload.specimenId,
          payload.location
        );
        break;
      case 'specimen.transferred':
        await this.limsAdapter.recordTransfer(payload as any);
        break;
    }
  }
}

class QualityEventHandler implements EventHandler {
  async handle(event: FacilityEvent): Promise<void> {
    if (!event.type.startsWith('quality.')) return;

    const payload = event.payload as QualityEventPayload;

    if (event.type === 'quality.deviation.detected') {
      await this.createDeviationReport(payload);
    }
  }

  private async createDeviationReport(payload: QualityEventPayload): Promise<void> {
    console.log('Creating deviation report:', payload);
  }
}

// Logging middleware
class LoggingMiddleware implements EventMiddleware {
  async process(event: FacilityEvent): Promise<FacilityEvent> {
    console.log(`[${event.timestamp}] Event: ${event.type} from ${event.source.system}`);
    return event;
  }
}

// Validation middleware
class ValidationMiddleware implements EventMiddleware {
  async process(event: FacilityEvent): Promise<FacilityEvent> {
    if (!event.id || !event.type || !event.timestamp) {
      throw new Error('Invalid event: missing required fields');
    }
    return event;
  }
}

// Enrichment middleware
class EnrichmentMiddleware implements EventMiddleware {
  async process(event: FacilityEvent): Promise<FacilityEvent> {
    return {
      ...event,
      metadata: {
        ...event.metadata,
        enrichedAt: new Date().toISOString(),
        environment: process.env.NODE_ENV || 'development'
      }
    };
  }
}

interface EquipmentAlertPayload {
  equipmentId: string;
  alertType: string;
  value: number;
  threshold: number;
  timestamp: string;
}

interface SpecimenEventPayload {
  specimenId: string;
  location: CryoLocation;
  performedBy: string;
  timestamp: string;
}

interface QualityEventPayload {
  deviationType: string;
  description: string;
  affectedItems: string[];
  detectedAt: string;
}
```

---

## Chapter Summary

This chapter covered comprehensive system integration patterns:

- **Integration Architecture**: Hub-based connectivity with circuit breakers
- **LIMS Integration**: Specimen and container tracking with quality events
- **BMS Integration**: Environmental monitoring and HVAC control
- **Event-Driven Integration**: Publish-subscribe messaging with middleware

---

*© 2025 World Industry Association. All rights reserved.*

*弘益人間 (Benefit All Humanity)*
