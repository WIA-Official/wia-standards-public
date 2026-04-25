# Chapter 4: Cryogenic Facility API Interface

## RESTful, GraphQL, and Real-Time API Specifications

This chapter provides comprehensive API specifications for the WIA Cryo Facility Standard, enabling seamless integration between facility management systems, monitoring platforms, and external services.

---

## API Architecture Overview

### Multi-Protocol API Design

```typescript
/**
 * WIA Cryo Facility API Architecture
 * Multi-protocol interface design
 */

interface APIArchitecture {
  // REST API for standard CRUD operations
  rest: RESTAPIConfig;

  // GraphQL for flexible queries
  graphql: GraphQLConfig;

  // WebSocket for real-time monitoring
  websocket: WebSocketConfig;

  // Event streaming for integration
  events: EventStreamConfig;
}

interface RESTAPIConfig {
  baseUrl: string;
  version: string;
  authentication: AuthConfig;
  rateLimit: RateLimitConfig;
  endpoints: RESTEndpoint[];
}

interface GraphQLConfig {
  endpoint: string;
  schema: string;
  authentication: AuthConfig;
  subscriptions: boolean;
}

interface WebSocketConfig {
  endpoint: string;
  protocols: string[];
  authentication: AuthConfig;
  heartbeatInterval: number;
  reconnectPolicy: ReconnectPolicy;
}

interface EventStreamConfig {
  broker: string;
  topics: EventTopic[];
  format: 'json' | 'avro' | 'protobuf';
}

interface AuthConfig {
  methods: ('api-key' | 'jwt' | 'oauth2' | 'mtls')[];
  tokenEndpoint?: string;
  scopes?: string[];
}

interface RateLimitConfig {
  windowMs: number;
  maxRequests: number;
  byUser: boolean;
  byEndpoint: boolean;
}

interface ReconnectPolicy {
  maxRetries: number;
  initialDelay: number;
  maxDelay: number;
  backoffMultiplier: number;
}

interface RESTEndpoint {
  path: string;
  method: 'GET' | 'POST' | 'PUT' | 'PATCH' | 'DELETE';
  description: string;
  authentication: boolean;
  rateLimit?: RateLimitConfig;
}

interface EventTopic {
  name: string;
  description: string;
  schema: string;
  retention: string;
}

// API configuration manager
class CryoFacilityAPIConfig {
  private config: APIArchitecture;

  constructor() {
    this.config = {
      rest: {
        baseUrl: '/api/v1',
        version: '1.0.0',
        authentication: {
          methods: ['jwt', 'api-key'],
          tokenEndpoint: '/auth/token',
          scopes: ['facility:read', 'facility:write', 'equipment:read',
                   'equipment:write', 'monitoring:read', 'admin']
        },
        rateLimit: {
          windowMs: 60000,
          maxRequests: 1000,
          byUser: true,
          byEndpoint: true
        },
        endpoints: []
      },
      graphql: {
        endpoint: '/graphql',
        schema: 'facility-schema.graphql',
        authentication: {
          methods: ['jwt'],
          scopes: ['graphql:execute']
        },
        subscriptions: true
      },
      websocket: {
        endpoint: '/ws/monitoring',
        protocols: ['cryo-facility-v1'],
        authentication: {
          methods: ['jwt']
        },
        heartbeatInterval: 30000,
        reconnectPolicy: {
          maxRetries: 10,
          initialDelay: 1000,
          maxDelay: 30000,
          backoffMultiplier: 2
        }
      },
      events: {
        broker: 'kafka://events.facility.local:9092',
        topics: [
          {
            name: 'facility.equipment.alerts',
            description: 'Equipment alert events',
            schema: 'equipment-alert-v1',
            retention: '7d'
          },
          {
            name: 'facility.environmental.readings',
            description: 'Environmental monitoring readings',
            schema: 'environmental-reading-v1',
            retention: '90d'
          },
          {
            name: 'facility.operations.events',
            description: 'Operational events and activities',
            schema: 'operation-event-v1',
            retention: '365d'
          }
        ],
        format: 'avro'
      }
    };
  }

  getRESTConfig(): RESTAPIConfig {
    return this.config.rest;
  }

  getGraphQLConfig(): GraphQLConfig {
    return this.config.graphql;
  }

  getWebSocketConfig(): WebSocketConfig {
    return this.config.websocket;
  }
}
```

---

## REST API Endpoints

### Facility Management APIs

```typescript
/**
 * REST API Implementation
 * Facility management endpoints
 */

import express, { Router, Request, Response, NextFunction } from 'express';

// Request/Response types
interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  meta?: ResponseMeta;
}

interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

interface ResponseMeta {
  timestamp: string;
  requestId: string;
  pagination?: PaginationMeta;
}

interface PaginationMeta {
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

// Facility API Router
class FacilityAPIRouter {
  private router: Router;

  constructor() {
    this.router = express.Router();
    this.setupRoutes();
  }

  private setupRoutes(): void {
    // Facility CRUD operations
    this.router.get('/facilities', this.listFacilities.bind(this));
    this.router.get('/facilities/:id', this.getFacility.bind(this));
    this.router.post('/facilities', this.createFacility.bind(this));
    this.router.put('/facilities/:id', this.updateFacility.bind(this));
    this.router.delete('/facilities/:id', this.deleteFacility.bind(this));

    // Facility zones
    this.router.get('/facilities/:id/zones', this.listZones.bind(this));
    this.router.post('/facilities/:id/zones', this.createZone.bind(this));
    this.router.put('/facilities/:id/zones/:zoneId', this.updateZone.bind(this));
    this.router.delete('/facilities/:id/zones/:zoneId', this.deleteZone.bind(this));

    // Facility status
    this.router.get('/facilities/:id/status', this.getFacilityStatus.bind(this));
    this.router.put('/facilities/:id/status', this.updateFacilityStatus.bind(this));
  }

  /**
   * GET /facilities
   * List all facilities with pagination and filtering
   */
  async listFacilities(req: Request, res: Response): Promise<void> {
    const {
      type,
      status,
      limit = 20,
      offset = 0,
      sort = 'name',
      order = 'asc'
    } = req.query;

    const facilities = await this.facilityService.findAll({
      type: type as string,
      status: status as string,
      pagination: {
        limit: Number(limit),
        offset: Number(offset)
      },
      sort: {
        field: sort as string,
        order: order as 'asc' | 'desc'
      }
    });

    const response: APIResponse<FacilityListItem[]> = {
      success: true,
      data: facilities.items,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string,
        pagination: {
          total: facilities.total,
          limit: Number(limit),
          offset: Number(offset),
          hasMore: facilities.hasMore
        }
      }
    };

    res.json(response);
  }

  /**
   * GET /facilities/:id
   * Get facility by ID with full details
   */
  async getFacility(req: Request, res: Response): Promise<void> {
    const { id } = req.params;
    const { include } = req.query;

    const facility = await this.facilityService.findById(id, {
      include: (include as string)?.split(',') || []
    });

    if (!facility) {
      res.status(404).json({
        success: false,
        error: {
          code: 'FACILITY_NOT_FOUND',
          message: `Facility ${id} not found`
        }
      });
      return;
    }

    res.json({
      success: true,
      data: facility,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  /**
   * POST /facilities
   * Create new facility
   */
  async createFacility(req: Request, res: Response): Promise<void> {
    const facilityData: CreateFacilityRequest = req.body;

    // Validate request
    const validation = this.validateFacilityRequest(facilityData);
    if (!validation.valid) {
      res.status(400).json({
        success: false,
        error: {
          code: 'VALIDATION_ERROR',
          message: 'Invalid facility data',
          details: { errors: validation.errors }
        }
      });
      return;
    }

    const facility = await this.facilityService.create(facilityData);

    res.status(201).json({
      success: true,
      data: facility,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  /**
   * PUT /facilities/:id
   * Update facility
   */
  async updateFacility(req: Request, res: Response): Promise<void> {
    const { id } = req.params;
    const updateData: UpdateFacilityRequest = req.body;

    const facility = await this.facilityService.update(id, updateData);

    if (!facility) {
      res.status(404).json({
        success: false,
        error: {
          code: 'FACILITY_NOT_FOUND',
          message: `Facility ${id} not found`
        }
      });
      return;
    }

    res.json({
      success: true,
      data: facility,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  /**
   * DELETE /facilities/:id
   * Delete facility (soft delete)
   */
  async deleteFacility(req: Request, res: Response): Promise<void> {
    const { id } = req.params;

    const deleted = await this.facilityService.delete(id);

    if (!deleted) {
      res.status(404).json({
        success: false,
        error: {
          code: 'FACILITY_NOT_FOUND',
          message: `Facility ${id} not found`
        }
      });
      return;
    }

    res.status(204).send();
  }

  /**
   * GET /facilities/:id/status
   * Get comprehensive facility status
   */
  async getFacilityStatus(req: Request, res: Response): Promise<void> {
    const { id } = req.params;

    const status = await this.facilityService.getStatus(id);

    res.json({
      success: true,
      data: {
        facilityId: id,
        status: status.overall,
        equipment: status.equipment,
        environmental: status.environmental,
        alerts: status.activeAlerts,
        lastUpdated: status.lastUpdated
      },
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  private facilityService = new FacilityService();

  private validateFacilityRequest(data: CreateFacilityRequest): ValidationResult {
    const errors: ValidationError[] = [];

    if (!data.name || data.name.trim().length === 0) {
      errors.push({ path: 'name', message: 'Name is required' });
    }

    if (!data.type) {
      errors.push({ path: 'type', message: 'Type is required' });
    }

    if (!data.location) {
      errors.push({ path: 'location', message: 'Location is required' });
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined
    };
  }

  getRouter(): Router {
    return this.router;
  }
}

interface CreateFacilityRequest {
  name: string;
  description?: string;
  type: FacilityType;
  location: FacilityLocation;
  organization: Organization;
}

interface UpdateFacilityRequest {
  name?: string;
  description?: string;
  status?: FacilityStatus;
}

interface FacilityListItem {
  id: string;
  name: string;
  type: FacilityType;
  status: FacilityStatus;
  location: {
    city: string;
    country: string;
  };
  createdAt: string;
}

// Facility service (stub)
class FacilityService {
  async findAll(options: any): Promise<{ items: FacilityListItem[]; total: number; hasMore: boolean }> {
    return { items: [], total: 0, hasMore: false };
  }
  async findById(id: string, options: any): Promise<WIACryoFacilityProject | null> {
    return null;
  }
  async create(data: CreateFacilityRequest): Promise<WIACryoFacilityProject> {
    throw new Error('Not implemented');
  }
  async update(id: string, data: UpdateFacilityRequest): Promise<WIACryoFacilityProject | null> {
    return null;
  }
  async delete(id: string): Promise<boolean> {
    return false;
  }
  async getStatus(id: string): Promise<any> {
    return {};
  }
}
```

---

### Equipment Management APIs

```typescript
/**
 * Equipment Management API
 * Cryogenic equipment CRUD and monitoring
 */

class EquipmentAPIRouter {
  private router: Router;

  constructor() {
    this.router = express.Router();
    this.setupRoutes();
  }

  private setupRoutes(): void {
    // Equipment CRUD
    this.router.get('/facilities/:facilityId/equipment', this.listEquipment.bind(this));
    this.router.get('/facilities/:facilityId/equipment/:id', this.getEquipment.bind(this));
    this.router.post('/facilities/:facilityId/equipment', this.createEquipment.bind(this));
    this.router.put('/facilities/:facilityId/equipment/:id', this.updateEquipment.bind(this));
    this.router.delete('/facilities/:facilityId/equipment/:id', this.deleteEquipment.bind(this));

    // Equipment status and monitoring
    this.router.get('/facilities/:facilityId/equipment/:id/status', this.getEquipmentStatus.bind(this));
    this.router.get('/facilities/:facilityId/equipment/:id/readings', this.getEquipmentReadings.bind(this));
    this.router.get('/facilities/:facilityId/equipment/:id/alerts', this.getEquipmentAlerts.bind(this));

    // Maintenance operations
    this.router.get('/facilities/:facilityId/equipment/:id/maintenance', this.getMaintenanceHistory.bind(this));
    this.router.post('/facilities/:facilityId/equipment/:id/maintenance', this.recordMaintenance.bind(this));

    // Sensor management
    this.router.get('/facilities/:facilityId/equipment/:id/sensors', this.listSensors.bind(this));
    this.router.post('/facilities/:facilityId/equipment/:id/sensors', this.addSensor.bind(this));
    this.router.put('/facilities/:facilityId/equipment/:id/sensors/:sensorId', this.updateSensor.bind(this));
    this.router.delete('/facilities/:facilityId/equipment/:id/sensors/:sensorId', this.removeSensor.bind(this));
  }

  /**
   * GET /facilities/:facilityId/equipment
   * List all equipment in facility
   */
  async listEquipment(req: Request, res: Response): Promise<void> {
    const { facilityId } = req.params;
    const {
      type,
      status,
      zone,
      limit = 50,
      offset = 0
    } = req.query;

    const equipment = await this.equipmentService.findByFacility(facilityId, {
      type: type as string,
      status: status as string,
      zone: zone as string,
      pagination: {
        limit: Number(limit),
        offset: Number(offset)
      }
    });

    res.json({
      success: true,
      data: equipment.items,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string,
        pagination: {
          total: equipment.total,
          limit: Number(limit),
          offset: Number(offset),
          hasMore: equipment.hasMore
        }
      }
    });
  }

  /**
   * GET /facilities/:facilityId/equipment/:id
   * Get equipment details
   */
  async getEquipment(req: Request, res: Response): Promise<void> {
    const { facilityId, id } = req.params;

    const equipment = await this.equipmentService.findById(facilityId, id);

    if (!equipment) {
      res.status(404).json({
        success: false,
        error: {
          code: 'EQUIPMENT_NOT_FOUND',
          message: `Equipment ${id} not found in facility ${facilityId}`
        }
      });
      return;
    }

    res.json({
      success: true,
      data: equipment,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  /**
   * POST /facilities/:facilityId/equipment
   * Register new equipment
   */
  async createEquipment(req: Request, res: Response): Promise<void> {
    const { facilityId } = req.params;
    const equipmentData: CreateEquipmentRequest = req.body;

    const equipment = await this.equipmentService.create(facilityId, equipmentData);

    res.status(201).json({
      success: true,
      data: equipment,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  /**
   * GET /facilities/:facilityId/equipment/:id/status
   * Get real-time equipment status
   */
  async getEquipmentStatus(req: Request, res: Response): Promise<void> {
    const { facilityId, id } = req.params;

    const status = await this.equipmentService.getStatus(facilityId, id);

    res.json({
      success: true,
      data: {
        equipmentId: id,
        status: status.status,
        temperature: status.currentTemperature,
        level: status.currentLevel,
        sensors: status.sensorStatus,
        lastReading: status.lastReading,
        activeAlerts: status.activeAlerts
      },
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  /**
   * GET /facilities/:facilityId/equipment/:id/readings
   * Get historical readings
   */
  async getEquipmentReadings(req: Request, res: Response): Promise<void> {
    const { facilityId, id } = req.params;
    const {
      parameter,
      startTime,
      endTime,
      interval = '1m',
      aggregation = 'avg'
    } = req.query;

    const readings = await this.equipmentService.getReadings(facilityId, id, {
      parameter: parameter as string,
      startTime: startTime as string,
      endTime: endTime as string,
      interval: interval as string,
      aggregation: aggregation as 'avg' | 'min' | 'max' | 'count'
    });

    res.json({
      success: true,
      data: {
        equipmentId: id,
        parameter,
        startTime,
        endTime,
        interval,
        readings: readings.data,
        statistics: readings.statistics
      },
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  /**
   * GET /facilities/:facilityId/equipment/:id/alerts
   * Get equipment alerts
   */
  async getEquipmentAlerts(req: Request, res: Response): Promise<void> {
    const { facilityId, id } = req.params;
    const {
      status,
      severity,
      startTime,
      limit = 50,
      offset = 0
    } = req.query;

    const alerts = await this.equipmentService.getAlerts(facilityId, id, {
      status: status as string,
      severity: severity as string,
      startTime: startTime as string,
      pagination: {
        limit: Number(limit),
        offset: Number(offset)
      }
    });

    res.json({
      success: true,
      data: alerts.items,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string,
        pagination: {
          total: alerts.total,
          limit: Number(limit),
          offset: Number(offset),
          hasMore: alerts.hasMore
        }
      }
    });
  }

  /**
   * POST /facilities/:facilityId/equipment/:id/maintenance
   * Record maintenance activity
   */
  async recordMaintenance(req: Request, res: Response): Promise<void> {
    const { facilityId, id } = req.params;
    const maintenanceData: RecordMaintenanceRequest = req.body;

    const record = await this.equipmentService.recordMaintenance(facilityId, id, maintenanceData);

    res.status(201).json({
      success: true,
      data: record,
      meta: {
        timestamp: new Date().toISOString(),
        requestId: req.headers['x-request-id'] as string
      }
    });
  }

  private equipmentService = new EquipmentService();

  getRouter(): Router {
    return this.router;
  }
}

interface CreateEquipmentRequest {
  type: CryoEquipmentType;
  model: string;
  manufacturer: string;
  serialNumber: string;
  location: string;
  capacity: number;
  temperature: number;
}

interface RecordMaintenanceRequest {
  type: string;
  technician: string;
  description: string;
  parts?: string[];
  nextServiceDate?: string;
}

// Equipment service (stub)
class EquipmentService {
  async findByFacility(facilityId: string, options: any): Promise<any> {
    return { items: [], total: 0, hasMore: false };
  }
  async findById(facilityId: string, id: string): Promise<any> {
    return null;
  }
  async create(facilityId: string, data: any): Promise<any> {
    return {};
  }
  async getStatus(facilityId: string, id: string): Promise<any> {
    return {};
  }
  async getReadings(facilityId: string, id: string, options: any): Promise<any> {
    return { data: [], statistics: {} };
  }
  async getAlerts(facilityId: string, id: string, options: any): Promise<any> {
    return { items: [], total: 0, hasMore: false };
  }
  async recordMaintenance(facilityId: string, id: string, data: any): Promise<any> {
    return {};
  }
}
```

---

## GraphQL API

### Schema Definition

```typescript
/**
 * GraphQL Schema for Cryo Facility API
 * Flexible query interface
 */

const typeDefs = `#graphql
  # Scalars
  scalar DateTime
  scalar JSON

  # Enums
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

  enum AlertSeverity {
    INFO
    WARNING
    CRITICAL
  }

  # Input Types
  input FacilityFilterInput {
    type: FacilityType
    status: FacilityStatus
    country: String
    search: String
  }

  input EquipmentFilterInput {
    type: EquipmentType
    status: EquipmentStatus
    zone: String
  }

  input ReadingsFilterInput {
    parameter: String!
    startTime: DateTime!
    endTime: DateTime!
    interval: String
  }

  input PaginationInput {
    limit: Int = 20
    offset: Int = 0
  }

  # Types
  type Coordinates {
    latitude: Float!
    longitude: Float!
  }

  type FacilityLocation {
    address: String!
    city: String!
    state: String
    country: String!
    postalCode: String!
    coordinates: Coordinates!
    timezone: String!
  }

  type Organization {
    name: String!
    type: String!
    registrationNumber: String
    contact: ContactInfo!
  }

  type ContactInfo {
    name: String!
    email: String!
    phone: String!
    emergencyPhone: String
  }

  type License {
    type: String!
    number: String!
    issuer: String!
    validFrom: DateTime!
    validTo: DateTime!
    scope: [String!]!
    status: String!
  }

  type Certification {
    name: String!
    body: String!
    number: String!
    scope: [String!]!
    validFrom: DateTime!
    validTo: DateTime!
    status: String!
  }

  type FacilityZone {
    id: ID!
    name: String!
    type: String!
    classification: String!
    area: Float!
    accessLevel: String!
    equipment: [Equipment!]!
  }

  type CapacityMetrics {
    totalTanks: Int!
    totalCapacity: Int!
    currentSpecimens: Int!
    utilizationPercent: Float!
  }

  type Facility {
    id: ID!
    name: String!
    description: String
    type: FacilityType!
    status: FacilityStatus!
    location: FacilityLocation!
    organization: Organization!
    licenses: [License!]!
    certifications: [Certification!]!
    zones: [FacilityZone!]!
    capacity: CapacityMetrics!
    equipment(filter: EquipmentFilterInput): [Equipment!]!
    alerts(severity: AlertSeverity, active: Boolean): [Alert!]!
    createdAt: DateTime!
    updatedAt: DateTime
  }

  type Sensor {
    id: ID!
    type: String!
    location: String!
    accuracy: Float!
    lastCalibration: DateTime!
    nextCalibration: DateTime!
    currentValue: Float
    status: String!
  }

  type MaintenanceRecord {
    date: DateTime!
    type: String!
    technician: String!
    description: String!
    parts: [String!]
  }

  type Equipment {
    id: ID!
    type: EquipmentType!
    model: String!
    manufacturer: String!
    serialNumber: String!
    location: String!
    zone: FacilityZone
    capacity: Int!
    temperature: Float!
    status: EquipmentStatus!
    sensors: [Sensor!]!
    maintenanceHistory: [MaintenanceRecord!]!
    nextMaintenance: DateTime
    alerts(active: Boolean): [Alert!]!
  }

  type Alert {
    id: ID!
    equipmentId: String!
    equipment: Equipment
    parameter: String!
    value: Float!
    threshold: Float!
    severity: AlertSeverity!
    message: String!
    acknowledgedAt: DateTime
    acknowledgedBy: String
    resolvedAt: DateTime
    createdAt: DateTime!
  }

  type Reading {
    timestamp: DateTime!
    value: Float!
  }

  type ReadingsData {
    parameter: String!
    unit: String!
    readings: [Reading!]!
    statistics: ReadingsStatistics!
  }

  type ReadingsStatistics {
    count: Int!
    min: Float!
    max: Float!
    avg: Float!
    stdDev: Float!
  }

  type FacilityConnection {
    items: [Facility!]!
    total: Int!
    hasMore: Boolean!
  }

  type EquipmentConnection {
    items: [Equipment!]!
    total: Int!
    hasMore: Boolean!
  }

  type AlertConnection {
    items: [Alert!]!
    total: Int!
    hasMore: Boolean!
  }

  # Queries
  type Query {
    # Facility queries
    facility(id: ID!): Facility
    facilities(
      filter: FacilityFilterInput
      pagination: PaginationInput
    ): FacilityConnection!

    # Equipment queries
    equipment(facilityId: ID!, id: ID!): Equipment
    equipmentList(
      facilityId: ID!
      filter: EquipmentFilterInput
      pagination: PaginationInput
    ): EquipmentConnection!

    # Readings queries
    readings(
      facilityId: ID!
      equipmentId: ID!
      filter: ReadingsFilterInput!
    ): ReadingsData!

    # Alert queries
    alerts(
      facilityId: ID
      equipmentId: ID
      severity: AlertSeverity
      active: Boolean
      pagination: PaginationInput
    ): AlertConnection!

    # Status overview
    facilityStatus(id: ID!): FacilityStatusOverview!
  }

  type FacilityStatusOverview {
    facilityId: ID!
    status: FacilityStatus!
    equipmentSummary: EquipmentSummary!
    environmentalSummary: EnvironmentalSummary!
    activeAlerts: Int!
    lastUpdated: DateTime!
  }

  type EquipmentSummary {
    total: Int!
    operational: Int!
    warning: Int!
    alarm: Int!
    offline: Int!
  }

  type EnvironmentalSummary {
    zonesMonitored: Int!
    parametersInRange: Int!
    parametersOutOfRange: Int!
  }

  # Mutations
  type Mutation {
    # Facility mutations
    createFacility(input: CreateFacilityInput!): Facility!
    updateFacility(id: ID!, input: UpdateFacilityInput!): Facility!
    deleteFacility(id: ID!): Boolean!
    updateFacilityStatus(id: ID!, status: FacilityStatus!): Facility!

    # Equipment mutations
    registerEquipment(facilityId: ID!, input: RegisterEquipmentInput!): Equipment!
    updateEquipment(facilityId: ID!, id: ID!, input: UpdateEquipmentInput!): Equipment!
    decommissionEquipment(facilityId: ID!, id: ID!): Equipment!
    recordMaintenance(facilityId: ID!, id: ID!, input: MaintenanceInput!): MaintenanceRecord!

    # Alert mutations
    acknowledgeAlert(id: ID!): Alert!
    resolveAlert(id: ID!, resolution: String!): Alert!
  }

  # Input types for mutations
  input CreateFacilityInput {
    name: String!
    description: String
    type: FacilityType!
    location: LocationInput!
    organization: OrganizationInput!
  }

  input UpdateFacilityInput {
    name: String
    description: String
  }

  input LocationInput {
    address: String!
    city: String!
    state: String
    country: String!
    postalCode: String!
    latitude: Float!
    longitude: Float!
    timezone: String!
  }

  input OrganizationInput {
    name: String!
    type: String!
    registrationNumber: String
    contactName: String!
    contactEmail: String!
    contactPhone: String!
    emergencyPhone: String
  }

  input RegisterEquipmentInput {
    type: EquipmentType!
    model: String!
    manufacturer: String!
    serialNumber: String!
    location: String!
    capacity: Int!
    temperature: Float!
  }

  input UpdateEquipmentInput {
    location: String
    status: EquipmentStatus
  }

  input MaintenanceInput {
    type: String!
    technician: String!
    description: String!
    parts: [String!]
    nextServiceDate: DateTime
  }

  # Subscriptions for real-time updates
  type Subscription {
    # Equipment alerts
    equipmentAlert(facilityId: ID, equipmentId: ID): Alert!

    # Equipment status changes
    equipmentStatusChanged(facilityId: ID): Equipment!

    # Environmental readings
    environmentalReading(facilityId: ID!, zoneId: ID): EnvironmentalReading!
  }

  type EnvironmentalReading {
    zoneId: ID!
    parameter: String!
    value: Float!
    unit: String!
    timestamp: DateTime!
  }
`;

// GraphQL Resolvers
const resolvers = {
  Query: {
    facility: async (_: any, { id }: { id: string }, context: GraphQLContext) => {
      return context.dataSources.facilities.findById(id);
    },

    facilities: async (
      _: any,
      { filter, pagination }: { filter?: any; pagination?: any },
      context: GraphQLContext
    ) => {
      return context.dataSources.facilities.findAll(filter, pagination);
    },

    equipment: async (
      _: any,
      { facilityId, id }: { facilityId: string; id: string },
      context: GraphQLContext
    ) => {
      return context.dataSources.equipment.findById(facilityId, id);
    },

    readings: async (
      _: any,
      { facilityId, equipmentId, filter }: any,
      context: GraphQLContext
    ) => {
      return context.dataSources.readings.getReadings(
        facilityId,
        equipmentId,
        filter
      );
    },

    alerts: async (
      _: any,
      args: any,
      context: GraphQLContext
    ) => {
      return context.dataSources.alerts.findAll(args);
    },

    facilityStatus: async (
      _: any,
      { id }: { id: string },
      context: GraphQLContext
    ) => {
      return context.dataSources.facilities.getStatus(id);
    }
  },

  Mutation: {
    createFacility: async (
      _: any,
      { input }: { input: any },
      context: GraphQLContext
    ) => {
      return context.dataSources.facilities.create(input);
    },

    updateFacility: async (
      _: any,
      { id, input }: { id: string; input: any },
      context: GraphQLContext
    ) => {
      return context.dataSources.facilities.update(id, input);
    },

    acknowledgeAlert: async (
      _: any,
      { id }: { id: string },
      context: GraphQLContext
    ) => {
      return context.dataSources.alerts.acknowledge(id, context.user.id);
    },

    resolveAlert: async (
      _: any,
      { id, resolution }: { id: string; resolution: string },
      context: GraphQLContext
    ) => {
      return context.dataSources.alerts.resolve(id, resolution, context.user.id);
    }
  },

  Subscription: {
    equipmentAlert: {
      subscribe: (_: any, args: any, context: GraphQLContext) => {
        return context.pubsub.asyncIterator(['EQUIPMENT_ALERT']);
      }
    },

    equipmentStatusChanged: {
      subscribe: (_: any, args: any, context: GraphQLContext) => {
        return context.pubsub.asyncIterator(['EQUIPMENT_STATUS_CHANGED']);
      }
    },

    environmentalReading: {
      subscribe: (_: any, args: any, context: GraphQLContext) => {
        return context.pubsub.asyncIterator(['ENVIRONMENTAL_READING']);
      }
    }
  },

  // Field resolvers
  Facility: {
    equipment: async (facility: any, args: any, context: GraphQLContext) => {
      return context.dataSources.equipment.findByFacility(facility.id, args.filter);
    },

    alerts: async (facility: any, args: any, context: GraphQLContext) => {
      return context.dataSources.alerts.findByFacility(facility.id, args);
    }
  },

  Equipment: {
    zone: async (equipment: any, _: any, context: GraphQLContext) => {
      return context.dataSources.zones.findById(equipment.facilityId, equipment.location);
    },

    alerts: async (equipment: any, args: any, context: GraphQLContext) => {
      return context.dataSources.alerts.findByEquipment(equipment.id, args);
    }
  }
};

interface GraphQLContext {
  user: { id: string; roles: string[] };
  dataSources: {
    facilities: any;
    equipment: any;
    zones: any;
    readings: any;
    alerts: any;
  };
  pubsub: any;
}
```

---

## WebSocket Real-Time API

### Real-Time Monitoring Protocol

```typescript
/**
 * WebSocket API for Real-Time Monitoring
 * Live equipment and environmental data
 */

import WebSocket from 'ws';
import { EventEmitter } from 'events';

interface WebSocketMessage {
  type: MessageType;
  payload: unknown;
  timestamp: string;
  messageId: string;
}

type MessageType =
  | 'subscribe'
  | 'unsubscribe'
  | 'reading'
  | 'alert'
  | 'status_change'
  | 'heartbeat'
  | 'ack'
  | 'error';

interface SubscriptionRequest {
  channel: SubscriptionChannel;
  facilityId: string;
  equipmentId?: string;
  parameters?: string[];
}

type SubscriptionChannel =
  | 'equipment.readings'
  | 'equipment.status'
  | 'equipment.alerts'
  | 'environmental.readings'
  | 'facility.status';

class CryoFacilityWebSocketServer {
  private wss: WebSocket.Server;
  private clients: Map<string, WebSocketClient> = new Map();
  private subscriptions: Map<string, Set<string>> = new Map();
  private eventEmitter: EventEmitter;

  constructor(port: number) {
    this.wss = new WebSocket.Server({ port });
    this.eventEmitter = new EventEmitter();
    this.setupServer();
  }

  private setupServer(): void {
    this.wss.on('connection', (ws: WebSocket, req) => {
      const clientId = this.generateClientId();
      const client = new WebSocketClient(clientId, ws, this);
      this.clients.set(clientId, client);

      console.log(`Client connected: ${clientId}`);

      ws.on('close', () => {
        this.handleDisconnect(clientId);
      });
    });

    // Setup internal event handlers
    this.setupInternalEvents();
  }

  private setupInternalEvents(): void {
    // Handle equipment readings from monitoring system
    this.eventEmitter.on('equipment.reading', (data: EquipmentReading) => {
      this.broadcastToChannel('equipment.readings', data.facilityId, {
        type: 'reading',
        payload: {
          equipmentId: data.equipmentId,
          parameter: data.parameter,
          value: data.value,
          unit: data.unit
        },
        timestamp: new Date().toISOString(),
        messageId: this.generateMessageId()
      });
    });

    // Handle equipment status changes
    this.eventEmitter.on('equipment.status', (data: StatusChange) => {
      this.broadcastToChannel('equipment.status', data.facilityId, {
        type: 'status_change',
        payload: {
          equipmentId: data.equipmentId,
          previousStatus: data.previousStatus,
          newStatus: data.newStatus,
          reason: data.reason
        },
        timestamp: new Date().toISOString(),
        messageId: this.generateMessageId()
      });
    });

    // Handle alerts
    this.eventEmitter.on('equipment.alert', (alert: Alert) => {
      this.broadcastToChannel('equipment.alerts', alert.facilityId, {
        type: 'alert',
        payload: alert,
        timestamp: new Date().toISOString(),
        messageId: this.generateMessageId()
      });
    });
  }

  subscribe(
    clientId: string,
    channel: SubscriptionChannel,
    facilityId: string
  ): void {
    const key = `${channel}:${facilityId}`;
    if (!this.subscriptions.has(key)) {
      this.subscriptions.set(key, new Set());
    }
    this.subscriptions.get(key)!.add(clientId);
  }

  unsubscribe(
    clientId: string,
    channel: SubscriptionChannel,
    facilityId: string
  ): void {
    const key = `${channel}:${facilityId}`;
    this.subscriptions.get(key)?.delete(clientId);
  }

  private broadcastToChannel(
    channel: SubscriptionChannel,
    facilityId: string,
    message: WebSocketMessage
  ): void {
    const key = `${channel}:${facilityId}`;
    const subscribers = this.subscriptions.get(key);

    if (subscribers) {
      for (const clientId of subscribers) {
        const client = this.clients.get(clientId);
        if (client) {
          client.send(message);
        }
      }
    }
  }

  private handleDisconnect(clientId: string): void {
    // Remove from all subscriptions
    for (const [_, subscribers] of this.subscriptions) {
      subscribers.delete(clientId);
    }
    this.clients.delete(clientId);
    console.log(`Client disconnected: ${clientId}`);
  }

  private generateClientId(): string {
    return `client-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateMessageId(): string {
    return `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  // Public method to emit events from external sources
  emitReading(reading: EquipmentReading): void {
    this.eventEmitter.emit('equipment.reading', reading);
  }

  emitStatusChange(change: StatusChange): void {
    this.eventEmitter.emit('equipment.status', change);
  }

  emitAlert(alert: Alert): void {
    this.eventEmitter.emit('equipment.alert', alert);
  }
}

class WebSocketClient {
  private id: string;
  private ws: WebSocket;
  private server: CryoFacilityWebSocketServer;
  private heartbeatInterval: NodeJS.Timeout | null = null;
  private isAlive: boolean = true;

  constructor(
    id: string,
    ws: WebSocket,
    server: CryoFacilityWebSocketServer
  ) {
    this.id = id;
    this.ws = ws;
    this.server = server;
    this.setupHandlers();
    this.startHeartbeat();
  }

  private setupHandlers(): void {
    this.ws.on('message', (data: string) => {
      try {
        const message: WebSocketMessage = JSON.parse(data);
        this.handleMessage(message);
      } catch (error) {
        this.sendError('INVALID_MESSAGE', 'Failed to parse message');
      }
    });

    this.ws.on('pong', () => {
      this.isAlive = true;
    });
  }

  private handleMessage(message: WebSocketMessage): void {
    switch (message.type) {
      case 'subscribe':
        this.handleSubscribe(message.payload as SubscriptionRequest);
        break;
      case 'unsubscribe':
        this.handleUnsubscribe(message.payload as SubscriptionRequest);
        break;
      case 'heartbeat':
        this.handleHeartbeat();
        break;
      default:
        this.sendError('UNKNOWN_MESSAGE_TYPE', `Unknown message type: ${message.type}`);
    }
  }

  private handleSubscribe(request: SubscriptionRequest): void {
    this.server.subscribe(this.id, request.channel, request.facilityId);
    this.send({
      type: 'ack',
      payload: {
        action: 'subscribe',
        channel: request.channel,
        facilityId: request.facilityId
      },
      timestamp: new Date().toISOString(),
      messageId: `ack-${Date.now()}`
    });
  }

  private handleUnsubscribe(request: SubscriptionRequest): void {
    this.server.unsubscribe(this.id, request.channel, request.facilityId);
    this.send({
      type: 'ack',
      payload: {
        action: 'unsubscribe',
        channel: request.channel,
        facilityId: request.facilityId
      },
      timestamp: new Date().toISOString(),
      messageId: `ack-${Date.now()}`
    });
  }

  private handleHeartbeat(): void {
    this.send({
      type: 'heartbeat',
      payload: { serverTime: new Date().toISOString() },
      timestamp: new Date().toISOString(),
      messageId: `hb-${Date.now()}`
    });
  }

  private startHeartbeat(): void {
    this.heartbeatInterval = setInterval(() => {
      if (!this.isAlive) {
        this.ws.terminate();
        return;
      }
      this.isAlive = false;
      this.ws.ping();
    }, 30000);
  }

  send(message: WebSocketMessage): void {
    if (this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    }
  }

  private sendError(code: string, message: string): void {
    this.send({
      type: 'error',
      payload: { code, message },
      timestamp: new Date().toISOString(),
      messageId: `err-${Date.now()}`
    });
  }

  cleanup(): void {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
    }
  }
}

interface EquipmentReading {
  facilityId: string;
  equipmentId: string;
  parameter: string;
  value: number;
  unit: string;
}

interface StatusChange {
  facilityId: string;
  equipmentId: string;
  previousStatus: EquipmentStatus;
  newStatus: EquipmentStatus;
  reason?: string;
}

interface Alert {
  id: string;
  facilityId: string;
  equipmentId: string;
  parameter: string;
  value: number;
  threshold: number;
  severity: 'info' | 'warning' | 'critical';
  message: string;
}

// WebSocket client SDK
class CryoFacilityWebSocketClient {
  private ws: WebSocket | null = null;
  private config: WebSocketClientConfig;
  private reconnectAttempts: number = 0;
  private handlers: Map<string, Set<(data: any) => void>> = new Map();

  constructor(config: WebSocketClientConfig) {
    this.config = config;
  }

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.config.url, this.config.protocols);

      this.ws.onopen = () => {
        this.reconnectAttempts = 0;
        resolve();
      };

      this.ws.onerror = (error) => {
        reject(error);
      };

      this.ws.onmessage = (event) => {
        const message: WebSocketMessage = JSON.parse(event.data as string);
        this.handleMessage(message);
      };

      this.ws.onclose = () => {
        this.handleReconnect();
      };
    });
  }

  subscribe(
    channel: SubscriptionChannel,
    facilityId: string,
    callback: (data: any) => void
  ): void {
    const key = `${channel}:${facilityId}`;
    if (!this.handlers.has(key)) {
      this.handlers.set(key, new Set());
    }
    this.handlers.get(key)!.add(callback);

    this.send({
      type: 'subscribe',
      payload: { channel, facilityId },
      timestamp: new Date().toISOString(),
      messageId: `sub-${Date.now()}`
    });
  }

  unsubscribe(channel: SubscriptionChannel, facilityId: string): void {
    const key = `${channel}:${facilityId}`;
    this.handlers.delete(key);

    this.send({
      type: 'unsubscribe',
      payload: { channel, facilityId },
      timestamp: new Date().toISOString(),
      messageId: `unsub-${Date.now()}`
    });
  }

  private handleMessage(message: WebSocketMessage): void {
    if (message.type === 'reading' || message.type === 'alert' || message.type === 'status_change') {
      const payload = message.payload as any;
      // Broadcast to all matching handlers
      for (const [key, handlers] of this.handlers) {
        for (const handler of handlers) {
          handler(payload);
        }
      }
    }
  }

  private send(message: WebSocketMessage): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    }
  }

  private handleReconnect(): void {
    if (this.reconnectAttempts >= this.config.maxRetries) {
      console.error('Max reconnection attempts reached');
      return;
    }

    const delay = Math.min(
      this.config.initialDelay * Math.pow(this.config.backoffMultiplier, this.reconnectAttempts),
      this.config.maxDelay
    );

    this.reconnectAttempts++;

    setTimeout(() => {
      this.connect().catch(() => {
        this.handleReconnect();
      });
    }, delay);
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}

interface WebSocketClientConfig {
  url: string;
  protocols?: string[];
  maxRetries: number;
  initialDelay: number;
  maxDelay: number;
  backoffMultiplier: number;
}
```

---

## API SDK

### TypeScript Client SDK

```typescript
/**
 * WIA Cryo Facility API Client SDK
 * Unified client for all API protocols
 */

class CryoFacilityClient {
  private restClient: RESTClient;
  private graphqlClient: GraphQLClient;
  private wsClient: CryoFacilityWebSocketClient | null = null;

  constructor(config: ClientConfig) {
    this.restClient = new RESTClient(config.rest);
    this.graphqlClient = new GraphQLClient(config.graphql);

    if (config.websocket) {
      this.wsClient = new CryoFacilityWebSocketClient(config.websocket);
    }
  }

  // Facility operations
  async getFacility(id: string): Promise<WIACryoFacilityProject> {
    return this.restClient.get(`/facilities/${id}`);
  }

  async listFacilities(options?: ListOptions): Promise<PaginatedResult<FacilityListItem>> {
    return this.restClient.get('/facilities', options);
  }

  async createFacility(data: CreateFacilityRequest): Promise<WIACryoFacilityProject> {
    return this.restClient.post('/facilities', data);
  }

  async updateFacility(id: string, data: UpdateFacilityRequest): Promise<WIACryoFacilityProject> {
    return this.restClient.put(`/facilities/${id}`, data);
  }

  async deleteFacility(id: string): Promise<void> {
    return this.restClient.delete(`/facilities/${id}`);
  }

  // Equipment operations
  async getEquipment(facilityId: string, equipmentId: string): Promise<CryoStorageEquipment> {
    return this.restClient.get(`/facilities/${facilityId}/equipment/${equipmentId}`);
  }

  async listEquipment(facilityId: string, options?: ListOptions): Promise<PaginatedResult<CryoStorageEquipment>> {
    return this.restClient.get(`/facilities/${facilityId}/equipment`, options);
  }

  async getEquipmentReadings(
    facilityId: string,
    equipmentId: string,
    options: ReadingsOptions
  ): Promise<ReadingsData> {
    return this.restClient.get(
      `/facilities/${facilityId}/equipment/${equipmentId}/readings`,
      options
    );
  }

  // Alert operations
  async listAlerts(options?: AlertListOptions): Promise<PaginatedResult<Alert>> {
    return this.restClient.get('/alerts', options);
  }

  async acknowledgeAlert(alertId: string): Promise<Alert> {
    return this.restClient.post(`/alerts/${alertId}/acknowledge`);
  }

  async resolveAlert(alertId: string, resolution: string): Promise<Alert> {
    return this.restClient.post(`/alerts/${alertId}/resolve`, { resolution });
  }

  // GraphQL operations for complex queries
  async query<T>(query: string, variables?: Record<string, unknown>): Promise<T> {
    return this.graphqlClient.query(query, variables);
  }

  async mutate<T>(mutation: string, variables?: Record<string, unknown>): Promise<T> {
    return this.graphqlClient.mutate(mutation, variables);
  }

  // Real-time subscriptions
  async connectRealTime(): Promise<void> {
    if (!this.wsClient) {
      throw new Error('WebSocket not configured');
    }
    return this.wsClient.connect();
  }

  subscribeToReadings(
    facilityId: string,
    callback: (reading: EquipmentReading) => void
  ): void {
    if (!this.wsClient) {
      throw new Error('WebSocket not configured');
    }
    this.wsClient.subscribe('equipment.readings', facilityId, callback);
  }

  subscribeToAlerts(
    facilityId: string,
    callback: (alert: Alert) => void
  ): void {
    if (!this.wsClient) {
      throw new Error('WebSocket not configured');
    }
    this.wsClient.subscribe('equipment.alerts', facilityId, callback);
  }

  subscribeToStatus(
    facilityId: string,
    callback: (status: StatusChange) => void
  ): void {
    if (!this.wsClient) {
      throw new Error('WebSocket not configured');
    }
    this.wsClient.subscribe('equipment.status', facilityId, callback);
  }

  disconnectRealTime(): void {
    if (this.wsClient) {
      this.wsClient.disconnect();
    }
  }
}

class RESTClient {
  private baseUrl: string;
  private headers: Record<string, string>;

  constructor(config: RESTClientConfig) {
    this.baseUrl = config.baseUrl;
    this.headers = {
      'Content-Type': 'application/json',
      ...config.headers
    };

    if (config.apiKey) {
      this.headers['X-API-Key'] = config.apiKey;
    }
  }

  async get<T>(path: string, params?: Record<string, unknown>): Promise<T> {
    const url = new URL(path, this.baseUrl);
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          url.searchParams.append(key, String(value));
        }
      });
    }

    const response = await fetch(url.toString(), {
      method: 'GET',
      headers: this.headers
    });

    return this.handleResponse(response);
  }

  async post<T>(path: string, body?: unknown): Promise<T> {
    const response = await fetch(`${this.baseUrl}${path}`, {
      method: 'POST',
      headers: this.headers,
      body: body ? JSON.stringify(body) : undefined
    });

    return this.handleResponse(response);
  }

  async put<T>(path: string, body?: unknown): Promise<T> {
    const response = await fetch(`${this.baseUrl}${path}`, {
      method: 'PUT',
      headers: this.headers,
      body: body ? JSON.stringify(body) : undefined
    });

    return this.handleResponse(response);
  }

  async delete<T>(path: string): Promise<T> {
    const response = await fetch(`${this.baseUrl}${path}`, {
      method: 'DELETE',
      headers: this.headers
    });

    return this.handleResponse(response);
  }

  private async handleResponse<T>(response: Response): Promise<T> {
    if (!response.ok) {
      const error = await response.json();
      throw new APIError(error.code, error.message, response.status);
    }

    if (response.status === 204) {
      return undefined as unknown as T;
    }

    const json = await response.json();
    return json.data;
  }
}

class GraphQLClient {
  private endpoint: string;
  private headers: Record<string, string>;

  constructor(config: GraphQLClientConfig) {
    this.endpoint = config.endpoint;
    this.headers = {
      'Content-Type': 'application/json',
      ...config.headers
    };
  }

  async query<T>(query: string, variables?: Record<string, unknown>): Promise<T> {
    return this.execute(query, variables);
  }

  async mutate<T>(mutation: string, variables?: Record<string, unknown>): Promise<T> {
    return this.execute(mutation, variables);
  }

  private async execute<T>(
    query: string,
    variables?: Record<string, unknown>
  ): Promise<T> {
    const response = await fetch(this.endpoint, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({ query, variables })
    });

    const json = await response.json();

    if (json.errors) {
      throw new GraphQLError(json.errors);
    }

    return json.data;
  }
}

class APIError extends Error {
  constructor(
    public code: string,
    message: string,
    public status: number
  ) {
    super(message);
  }
}

class GraphQLError extends Error {
  constructor(public errors: any[]) {
    super(errors.map(e => e.message).join(', '));
  }
}

interface ClientConfig {
  rest: RESTClientConfig;
  graphql: GraphQLClientConfig;
  websocket?: WebSocketClientConfig;
}

interface RESTClientConfig {
  baseUrl: string;
  apiKey?: string;
  headers?: Record<string, string>;
}

interface GraphQLClientConfig {
  endpoint: string;
  headers?: Record<string, string>;
}

interface ListOptions {
  limit?: number;
  offset?: number;
  sort?: string;
  order?: 'asc' | 'desc';
  [key: string]: unknown;
}

interface PaginatedResult<T> {
  items: T[];
  total: number;
  hasMore: boolean;
}

interface ReadingsOptions {
  parameter: string;
  startTime: string;
  endTime: string;
  interval?: string;
  aggregation?: 'avg' | 'min' | 'max' | 'count';
}

interface AlertListOptions extends ListOptions {
  severity?: string;
  active?: boolean;
  facilityId?: string;
  equipmentId?: string;
}

interface ReadingsData {
  parameter: string;
  unit: string;
  readings: { timestamp: string; value: number }[];
  statistics: {
    count: number;
    min: number;
    max: number;
    avg: number;
    stdDev: number;
  };
}
```

---

## Chapter Summary

This chapter covered the comprehensive API interface for the WIA Cryo Facility Standard:

- **REST API**: CRUD operations for facilities, equipment, and alerts
- **GraphQL API**: Flexible queries with subscriptions
- **WebSocket API**: Real-time monitoring and alerts
- **Client SDK**: Unified TypeScript client for all protocols

---

*© 2025 World Industry Association. All rights reserved.*

*弘益人間 (Benefit All Humanity)*
