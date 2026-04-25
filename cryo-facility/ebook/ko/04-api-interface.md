# 제4장: API 인터페이스

## 4.1 개요

극저온 시설 관리 API는 시설의 모든 측면을 제어하고 모니터링하기 위한 포괄적인 인터페이스를 제공합니다. 이 장에서는 REST API, GraphQL, WebSocket 실시간 통신을 상세히 다룹니다.

```typescript
// API 아키텍처 개요
const apiArchitecture = {
  restAPI: {
    version: 'v1',
    baseUrl: '/api/v1',
    authentication: 'JWT + API Key',
    rateLimit: '1000 requests/minute'
  },
  graphQL: {
    endpoint: '/graphql',
    subscriptions: '/graphql/ws',
    introspection: 'development-only'
  },
  webSocket: {
    endpoint: '/ws',
    protocols: ['facility-monitoring', 'equipment-control'],
    heartbeat: '30 seconds'
  },
  features: [
    'Facility management',      // 시설 관리
    'Equipment monitoring',     // 장비 모니터링
    'Alert management',         // 알림 관리
    'Maintenance scheduling',   // 유지보수 스케줄링
    'Audit logging',           // 감사 로깅
    'Report generation'        // 보고서 생성
  ]
};
```

## 4.2 REST API 설계

### 4.2.1 API 라우터 구조

```typescript
import express, { Router, Request, Response, NextFunction } from 'express';
import { z } from 'zod';

// API 버전 관리
const API_VERSION = 'v1';
const API_BASE_PATH = `/api/${API_VERSION}`;

// 메인 API 라우터
export function createCryoFacilityAPIRouter(): Router {
  const router = Router();

  // 라우터 등록
  router.use('/facilities', createFacilityRouter());
  router.use('/equipment', createEquipmentRouter());
  router.use('/zones', createZoneRouter());
  router.use('/sensors', createSensorRouter());
  router.use('/alerts', createAlertRouter());
  router.use('/maintenance', createMaintenanceRouter());
  router.use('/sops', createSOPRouter());
  router.use('/training', createTrainingRouter());
  router.use('/audits', createAuditRouter());
  router.use('/reports', createReportRouter());

  return router;
}

// 요청 유효성 검증 미들웨어
function validateRequest<T>(schema: z.ZodSchema<T>) {
  return (req: Request, res: Response, next: NextFunction) => {
    try {
      const validated = schema.parse({
        body: req.body,
        query: req.query,
        params: req.params
      });
      req.body = validated.body;
      req.query = validated.query;
      req.params = validated.params;
      next();
    } catch (error) {
      if (error instanceof z.ZodError) {
        res.status(400).json({
          error: 'Validation Error',
          details: error.errors.map(e => ({
            path: e.path.join('.'),
            message: e.message
          }))
        });
        return;
      }
      next(error);
    }
  };
}

// 페이지네이션 스키마
const PaginationSchema = z.object({
  page: z.coerce.number().min(1).default(1),
  limit: z.coerce.number().min(1).max(100).default(20),
  sortBy: z.string().optional(),
  sortOrder: z.enum(['asc', 'desc']).default('desc')
});
```

### 4.2.2 시설 API

```typescript
// 시설 관리 라우터
function createFacilityRouter(): Router {
  const router = Router();

  // 시설 목록 조회
  router.get('/',
    validateRequest(z.object({
      query: PaginationSchema.extend({
        type: z.enum(['biobank', 'tissue-bank', 'fertility-center',
                      'research-facility', 'hospital-unit', 'commercial-storage']).optional(),
        status: z.enum(['planning', 'construction', 'commissioning',
                        'operational', 'maintenance', 'suspended', 'decommissioned']).optional(),
        search: z.string().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { page, limit, sortBy, sortOrder, type, status, search } = req.query;

      const facilities = await facilityService.findAll({
        pagination: { page, limit, sortBy, sortOrder },
        filters: { type, status, search }
      });

      res.json({
        data: facilities.items,
        pagination: {
          page: facilities.page,
          limit: facilities.limit,
          total: facilities.total,
          totalPages: facilities.totalPages
        }
      });
    }
  );

  // 시설 상세 조회
  router.get('/:facilityId',
    validateRequest(z.object({
      params: z.object({
        facilityId: z.string().uuid()
      }),
      query: z.object({
        include: z.array(z.enum(['zones', 'equipment', 'contacts', 'certifications']))
          .optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { facilityId } = req.params;
      const { include } = req.query;

      const facility = await facilityService.findById(facilityId, { include });

      if (!facility) {
        res.status(404).json({
          error: 'Not Found',
          message: `시설을 찾을 수 없습니다: ${facilityId}`
        });
        return;
      }

      res.json({ data: facility });
    }
  );

  // 시설 생성
  router.post('/',
    requireRole(['admin', 'facility-manager']),
    validateRequest(z.object({
      body: CreateFacilitySchema
    })),
    async (req: Request, res: Response) => {
      const facility = await facilityService.create(req.body, {
        createdBy: req.user!.id
      });

      res.status(201).json({
        data: facility,
        message: '시설이 성공적으로 생성되었습니다'
      });
    }
  );

  // 시설 업데이트
  router.put('/:facilityId',
    requireRole(['admin', 'facility-manager']),
    validateRequest(z.object({
      params: z.object({
        facilityId: z.string().uuid()
      }),
      body: UpdateFacilitySchema
    })),
    async (req: Request, res: Response) => {
      const { facilityId } = req.params;

      const facility = await facilityService.update(facilityId, req.body, {
        updatedBy: req.user!.id
      });

      res.json({
        data: facility,
        message: '시설이 성공적으로 업데이트되었습니다'
      });
    }
  );

  // 시설 삭제 (소프트 삭제)
  router.delete('/:facilityId',
    requireRole(['admin']),
    validateRequest(z.object({
      params: z.object({
        facilityId: z.string().uuid()
      })
    })),
    async (req: Request, res: Response) => {
      const { facilityId } = req.params;

      await facilityService.softDelete(facilityId, {
        deletedBy: req.user!.id
      });

      res.json({
        message: '시설이 성공적으로 삭제되었습니다'
      });
    }
  );

  // 시설 상태 업데이트
  router.patch('/:facilityId/status',
    requireRole(['admin', 'facility-manager']),
    validateRequest(z.object({
      params: z.object({
        facilityId: z.string().uuid()
      }),
      body: z.object({
        status: z.enum(['planning', 'construction', 'commissioning',
                        'operational', 'maintenance', 'suspended', 'decommissioned']),
        reason: z.string().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { facilityId } = req.params;
      const { status, reason } = req.body;

      const facility = await facilityService.updateStatus(facilityId, status, {
        reason,
        updatedBy: req.user!.id
      });

      res.json({
        data: facility,
        message: `시설 상태가 '${status}'로 변경되었습니다`
      });
    }
  );

  // 시설 대시보드 데이터
  router.get('/:facilityId/dashboard',
    validateRequest(z.object({
      params: z.object({
        facilityId: z.string().uuid()
      }),
      query: z.object({
        timeRange: z.enum(['1h', '6h', '24h', '7d', '30d']).default('24h')
      })
    })),
    async (req: Request, res: Response) => {
      const { facilityId } = req.params;
      const { timeRange } = req.query;

      const dashboard = await facilityService.getDashboard(facilityId, { timeRange });

      res.json({ data: dashboard });
    }
  );

  return router;
}

// 시설 생성 스키마
const CreateFacilitySchema = z.object({
  name: z.string().min(1).max(200),
  type: z.enum(['biobank', 'tissue-bank', 'fertility-center',
                'research-facility', 'hospital-unit', 'commercial-storage']),
  location: z.object({
    address: z.string(),
    city: z.string(),
    country: z.string().length(2),
    postalCode: z.string(),
    coordinates: z.object({
      latitude: z.number().min(-90).max(90),
      longitude: z.number().min(-180).max(180)
    }).optional()
  }),
  capacity: z.object({
    totalStorageUnits: z.number().positive(),
    maxSpecimens: z.number().positive()
  }),
  contact: z.object({
    email: z.string().email(),
    phone: z.string(),
    emergencyPhone: z.string()
  })
});

// 시설 업데이트 스키마
const UpdateFacilitySchema = CreateFacilitySchema.partial();
```

### 4.2.3 장비 API

```typescript
// 장비 관리 라우터
function createEquipmentRouter(): Router {
  const router = Router();

  // 장비 목록 조회
  router.get('/',
    validateRequest(z.object({
      query: PaginationSchema.extend({
        facilityId: z.string().uuid().optional(),
        zoneId: z.string().uuid().optional(),
        category: z.enum(['ln2-tank', 'mechanical-freezer', 'controlled-rate',
                         'incubator', 'centrifuge', 'monitoring-system',
                         'safety-equipment', 'handling-equipment', 'quality-control']).optional(),
        status: z.enum(['operational', 'standby', 'maintenance',
                        'calibration', 'fault', 'decommissioned', 'quarantine']).optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { page, limit, sortBy, sortOrder, ...filters } = req.query;

      const equipment = await equipmentService.findAll({
        pagination: { page, limit, sortBy, sortOrder },
        filters
      });

      res.json({
        data: equipment.items,
        pagination: {
          page: equipment.page,
          limit: equipment.limit,
          total: equipment.total,
          totalPages: equipment.totalPages
        }
      });
    }
  );

  // 장비 상세 조회
  router.get('/:equipmentId',
    validateRequest(z.object({
      params: z.object({
        equipmentId: z.string().uuid()
      }),
      query: z.object({
        include: z.array(z.enum(['sensors', 'maintenance', 'alarms', 'readings']))
          .optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { equipmentId } = req.params;
      const { include } = req.query;

      const equipment = await equipmentService.findById(equipmentId, { include });

      if (!equipment) {
        res.status(404).json({
          error: 'Not Found',
          message: `장비를 찾을 수 없습니다: ${equipmentId}`
        });
        return;
      }

      res.json({ data: equipment });
    }
  );

  // 장비 생성
  router.post('/',
    requireRole(['admin', 'facility-manager', 'equipment-manager']),
    validateRequest(z.object({
      body: CreateEquipmentSchema
    })),
    async (req: Request, res: Response) => {
      const equipment = await equipmentService.create(req.body, {
        createdBy: req.user!.id
      });

      res.status(201).json({
        data: equipment,
        message: '장비가 성공적으로 등록되었습니다'
      });
    }
  );

  // 장비 실시간 상태 조회
  router.get('/:equipmentId/status',
    validateRequest(z.object({
      params: z.object({
        equipmentId: z.string().uuid()
      })
    })),
    async (req: Request, res: Response) => {
      const { equipmentId } = req.params;

      const status = await equipmentService.getCurrentStatus(equipmentId);

      res.json({ data: status });
    }
  );

  // 장비 센서 데이터 조회
  router.get('/:equipmentId/readings',
    validateRequest(z.object({
      params: z.object({
        equipmentId: z.string().uuid()
      }),
      query: z.object({
        sensorId: z.string().uuid().optional(),
        from: z.string().datetime().optional(),
        to: z.string().datetime().optional(),
        resolution: z.enum(['raw', 'minute', 'hour', 'day']).default('minute'),
        limit: z.coerce.number().min(1).max(10000).default(1000)
      })
    })),
    async (req: Request, res: Response) => {
      const { equipmentId } = req.params;
      const { sensorId, from, to, resolution, limit } = req.query;

      const readings = await equipmentService.getReadings(equipmentId, {
        sensorId,
        timeRange: { from, to },
        resolution,
        limit
      });

      res.json({ data: readings });
    }
  );

  // 장비 명령 실행
  router.post('/:equipmentId/commands',
    requireRole(['admin', 'facility-manager', 'equipment-operator']),
    validateRequest(z.object({
      params: z.object({
        equipmentId: z.string().uuid()
      }),
      body: z.object({
        command: z.enum(['start', 'stop', 'setpoint', 'defrost', 'fill', 'calibrate']),
        parameters: z.record(z.any()).optional(),
        reason: z.string().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { equipmentId } = req.params;
      const { command, parameters, reason } = req.body;

      const result = await equipmentService.executeCommand(equipmentId, {
        command,
        parameters,
        reason,
        executedBy: req.user!.id
      });

      res.json({
        data: result,
        message: `명령 '${command}'이(가) 실행되었습니다`
      });
    }
  );

  // 장비 유지보수 기록
  router.get('/:equipmentId/maintenance',
    validateRequest(z.object({
      params: z.object({
        equipmentId: z.string().uuid()
      }),
      query: PaginationSchema.extend({
        type: z.enum(['preventive', 'corrective', 'calibration', 'inspection']).optional(),
        status: z.enum(['scheduled', 'in-progress', 'completed', 'cancelled']).optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { equipmentId } = req.params;
      const { page, limit, sortBy, sortOrder, ...filters } = req.query;

      const maintenance = await maintenanceService.findByEquipment(equipmentId, {
        pagination: { page, limit, sortBy, sortOrder },
        filters
      });

      res.json({
        data: maintenance.items,
        pagination: {
          page: maintenance.page,
          limit: maintenance.limit,
          total: maintenance.total,
          totalPages: maintenance.totalPages
        }
      });
    }
  );

  // 유지보수 작업 생성
  router.post('/:equipmentId/maintenance',
    requireRole(['admin', 'facility-manager', 'maintenance-technician']),
    validateRequest(z.object({
      params: z.object({
        equipmentId: z.string().uuid()
      }),
      body: CreateMaintenanceSchema
    })),
    async (req: Request, res: Response) => {
      const { equipmentId } = req.params;

      const maintenance = await maintenanceService.create({
        ...req.body,
        equipmentId
      }, {
        createdBy: req.user!.id
      });

      res.status(201).json({
        data: maintenance,
        message: '유지보수 작업이 생성되었습니다'
      });
    }
  );

  return router;
}

// 장비 생성 스키마
const CreateEquipmentSchema = z.object({
  name: z.string(),
  category: z.enum(['ln2-tank', 'mechanical-freezer', 'controlled-rate',
                    'incubator', 'centrifuge', 'monitoring-system',
                    'safety-equipment', 'handling-equipment', 'quality-control']),
  facilityId: z.string().uuid(),
  zoneId: z.string().uuid(),
  manufacturer: z.string(),
  model: z.string(),
  serialNumber: z.string(),
  specifications: z.record(z.any()),
  installationDate: z.string().datetime()
});

// 유지보수 작업 생성 스키마
const CreateMaintenanceSchema = z.object({
  type: z.enum(['preventive', 'corrective', 'calibration', 'inspection']),
  title: z.string(),
  description: z.string(),
  scheduledDate: z.string().datetime(),
  assignedTo: z.string().uuid().optional(),
  priority: z.enum(['low', 'medium', 'high', 'critical']).default('medium'),
  estimatedDuration: z.object({
    value: z.number(),
    unit: z.enum(['minutes', 'hours', 'days'])
  }).optional()
});
```

### 4.2.4 알림 API

```typescript
// 알림 관리 라우터
function createAlertRouter(): Router {
  const router = Router();

  // 활성 알림 조회
  router.get('/active',
    validateRequest(z.object({
      query: z.object({
        facilityId: z.string().uuid().optional(),
        equipmentId: z.string().uuid().optional(),
        severity: z.enum(['info', 'warning', 'critical']).optional(),
        category: z.enum(['temperature', 'pressure', 'level', 'power',
                         'door', 'equipment', 'system']).optional()
      })
    })),
    async (req: Request, res: Response) => {
      const filters = req.query;

      const alerts = await alertService.getActiveAlerts(filters);

      res.json({
        data: alerts,
        summary: {
          total: alerts.length,
          critical: alerts.filter(a => a.severity === 'critical').length,
          warning: alerts.filter(a => a.severity === 'warning').length,
          info: alerts.filter(a => a.severity === 'info').length
        }
      });
    }
  );

  // 알림 이력 조회
  router.get('/history',
    validateRequest(z.object({
      query: PaginationSchema.extend({
        facilityId: z.string().uuid().optional(),
        equipmentId: z.string().uuid().optional(),
        severity: z.enum(['info', 'warning', 'critical']).optional(),
        from: z.string().datetime().optional(),
        to: z.string().datetime().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { page, limit, sortBy, sortOrder, ...filters } = req.query;

      const alerts = await alertService.getAlertHistory({
        pagination: { page, limit, sortBy, sortOrder },
        filters
      });

      res.json({
        data: alerts.items,
        pagination: {
          page: alerts.page,
          limit: alerts.limit,
          total: alerts.total,
          totalPages: alerts.totalPages
        }
      });
    }
  );

  // 알림 확인
  router.post('/:alertId/acknowledge',
    requireRole(['admin', 'facility-manager', 'operator']),
    validateRequest(z.object({
      params: z.object({
        alertId: z.string().uuid()
      }),
      body: z.object({
        notes: z.string().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { alertId } = req.params;
      const { notes } = req.body;

      const alert = await alertService.acknowledge(alertId, {
        acknowledgedBy: req.user!.id,
        notes
      });

      res.json({
        data: alert,
        message: '알림이 확인되었습니다'
      });
    }
  );

  // 알림 해제
  router.post('/:alertId/resolve',
    requireRole(['admin', 'facility-manager', 'operator']),
    validateRequest(z.object({
      params: z.object({
        alertId: z.string().uuid()
      }),
      body: z.object({
        resolution: z.string(),
        preventiveAction: z.string().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const { alertId } = req.params;
      const { resolution, preventiveAction } = req.body;

      const alert = await alertService.resolve(alertId, {
        resolvedBy: req.user!.id,
        resolution,
        preventiveAction
      });

      res.json({
        data: alert,
        message: '알림이 해제되었습니다'
      });
    }
  );

  // 알림 규칙 관리
  router.get('/rules',
    validateRequest(z.object({
      query: z.object({
        facilityId: z.string().uuid().optional(),
        enabled: z.coerce.boolean().optional()
      })
    })),
    async (req: Request, res: Response) => {
      const filters = req.query;

      const rules = await alertService.getAlertRules(filters);

      res.json({ data: rules });
    }
  );

  // 알림 규칙 생성
  router.post('/rules',
    requireRole(['admin', 'facility-manager']),
    validateRequest(z.object({
      body: CreateAlertRuleSchema
    })),
    async (req: Request, res: Response) => {
      const rule = await alertService.createAlertRule(req.body, {
        createdBy: req.user!.id
      });

      res.status(201).json({
        data: rule,
        message: '알림 규칙이 생성되었습니다'
      });
    }
  );

  return router;
}

// 알림 규칙 생성 스키마
const CreateAlertRuleSchema = z.object({
  name: z.string(),
  facilityId: z.string().uuid().optional(),
  equipmentId: z.string().uuid().optional(),
  condition: z.object({
    type: z.enum(['threshold', 'rate-of-change', 'pattern', 'absence']),
    metric: z.string(),
    operator: z.enum(['gt', 'gte', 'lt', 'lte', 'eq', 'neq']),
    value: z.number(),
    duration: z.object({
      value: z.number(),
      unit: z.enum(['seconds', 'minutes', 'hours'])
    }).optional()
  }),
  severity: z.enum(['info', 'warning', 'critical']),
  notifications: z.object({
    email: z.array(z.string().email()).optional(),
    sms: z.array(z.string()).optional(),
    webhook: z.string().url().optional()
  }),
  escalation: z.object({
    delay: z.object({
      value: z.number(),
      unit: z.enum(['minutes', 'hours'])
    }),
    contacts: z.array(z.string().uuid())
  }).optional(),
  enabled: z.boolean().default(true)
});
```

## 4.3 GraphQL API

### 4.3.1 GraphQL 스키마 정의

```typescript
import { makeExecutableSchema } from '@graphql-tools/schema';
import { gql } from 'graphql-tag';

// GraphQL 타입 정의
const typeDefs = gql`
  # 스칼라 타입
  scalar DateTime
  scalar JSON

  # 열거형
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

  enum AlertSeverity {
    INFO
    WARNING
    CRITICAL
  }

  # 시설 타입
  type Facility {
    id: ID!
    name: String!
    type: FacilityType!
    status: FacilityStatus!
    location: Location!
    capacity: Capacity!
    zones: [Zone!]!
    equipment: [Equipment!]!
    contacts: [Contact!]!
    certifications: [Certification!]!
    createdAt: DateTime!
    updatedAt: DateTime!
  }

  type Location {
    address: String!
    city: String!
    country: String!
    postalCode: String!
    coordinates: Coordinates
  }

  type Coordinates {
    latitude: Float!
    longitude: Float!
  }

  type Capacity {
    totalStorageUnits: Int!
    currentOccupancy: Int!
    maxSpecimens: Int!
    utilizationRate: Float!
  }

  type Contact {
    role: String!
    name: String!
    phone: String!
    email: String!
    priority: Int!
  }

  type Certification {
    type: String!
    issuer: String!
    validFrom: DateTime!
    validUntil: DateTime!
    certificateNumber: String!
  }

  # 구역 타입
  type Zone {
    id: ID!
    name: String!
    type: String!
    cleanroomClass: String!
    facility: Facility!
    equipment: [Equipment!]!
    environmentalRequirements: EnvironmentalRequirements!
  }

  type EnvironmentalRequirements {
    temperature: TemperatureRange!
    humidity: HumidityRange!
    pressure: PressureSpec
  }

  type TemperatureRange {
    min: Float!
    max: Float!
    target: Float!
    unit: String!
  }

  type HumidityRange {
    min: Float!
    max: Float!
    target: Float!
  }

  type PressureSpec {
    differential: Float!
    unit: String!
  }

  # 장비 타입
  type Equipment {
    id: ID!
    name: String!
    category: EquipmentCategory!
    status: EquipmentStatus!
    manufacturer: String!
    model: String!
    serialNumber: String!
    facility: Facility!
    zone: Zone!
    sensors: [Sensor!]!
    currentReadings: [Reading!]!
    maintenanceHistory: [MaintenanceRecord!]!
    alerts: [Alert!]!
    specifications: JSON!
    installationDate: DateTime!
  }

  type Sensor {
    id: ID!
    type: String!
    model: String!
    serialNumber: String!
    equipment: Equipment!
    calibration: CalibrationInfo!
    currentReading: Reading
    alarmThresholds: AlarmThresholds!
  }

  type CalibrationInfo {
    lastCalibrationDate: DateTime!
    nextCalibrationDate: DateTime!
    calibrationInterval: Duration!
    certificateNumber: String
  }

  type Duration {
    value: Int!
    unit: String!
  }

  type AlarmThresholds {
    warningLow: Float
    warningHigh: Float
    criticalLow: Float
    criticalHigh: Float
  }

  type Reading {
    id: ID!
    sensor: Sensor!
    timestamp: DateTime!
    value: Float!
    unit: String!
    quality: String!
    inSpec: Boolean!
  }

  # 알림 타입
  type Alert {
    id: ID!
    severity: AlertSeverity!
    category: String!
    message: String!
    equipment: Equipment
    facility: Facility!
    triggeredAt: DateTime!
    acknowledgedAt: DateTime
    acknowledgedBy: User
    resolvedAt: DateTime
    resolvedBy: User
    resolution: String
  }

  # 유지보수 타입
  type MaintenanceRecord {
    id: ID!
    equipment: Equipment!
    type: String!
    title: String!
    description: String!
    scheduledDate: DateTime!
    completedDate: DateTime
    assignedTo: User
    status: String!
    workPerformed: String
    partsUsed: [PartUsed!]
  }

  type PartUsed {
    partNumber: String!
    description: String!
    quantity: Int!
  }

  type User {
    id: ID!
    name: String!
    email: String!
    role: String!
  }

  # 대시보드 타입
  type FacilityDashboard {
    facility: Facility!
    equipmentSummary: EquipmentSummary!
    activeAlerts: [Alert!]!
    recentEvents: [Event!]!
    environmentalStatus: [ZoneEnvironmentalStatus!]!
    upcomingMaintenance: [MaintenanceRecord!]!
  }

  type EquipmentSummary {
    total: Int!
    operational: Int!
    maintenance: Int!
    fault: Int!
    byCategory: JSON!
  }

  type Event {
    id: ID!
    type: String!
    description: String!
    timestamp: DateTime!
    user: User
  }

  type ZoneEnvironmentalStatus {
    zone: Zone!
    temperature: ReadingSummary!
    humidity: ReadingSummary
    pressure: ReadingSummary
    compliance: Float!
  }

  type ReadingSummary {
    current: Float!
    min: Float!
    max: Float!
    average: Float!
    inSpec: Boolean!
  }

  # 입력 타입
  input FacilityFilter {
    type: FacilityType
    status: FacilityStatus
    search: String
  }

  input EquipmentFilter {
    facilityId: ID
    zoneId: ID
    category: EquipmentCategory
    status: EquipmentStatus
  }

  input TimeRange {
    from: DateTime!
    to: DateTime!
  }

  input PaginationInput {
    page: Int = 1
    limit: Int = 20
  }

  # 페이지네이션 결과
  type FacilityConnection {
    items: [Facility!]!
    pageInfo: PageInfo!
  }

  type EquipmentConnection {
    items: [Equipment!]!
    pageInfo: PageInfo!
  }

  type AlertConnection {
    items: [Alert!]!
    pageInfo: PageInfo!
  }

  type PageInfo {
    page: Int!
    limit: Int!
    total: Int!
    totalPages: Int!
    hasNextPage: Boolean!
    hasPreviousPage: Boolean!
  }

  # 쿼리
  type Query {
    # 시설 조회
    facility(id: ID!): Facility
    facilities(filter: FacilityFilter, pagination: PaginationInput): FacilityConnection!
    facilityDashboard(facilityId: ID!, timeRange: TimeRange): FacilityDashboard!

    # 장비 조회
    equipment(id: ID!): Equipment
    equipmentList(filter: EquipmentFilter, pagination: PaginationInput): EquipmentConnection!
    equipmentReadings(equipmentId: ID!, timeRange: TimeRange!, resolution: String): [Reading!]!

    # 알림 조회
    activeAlerts(facilityId: ID, severity: AlertSeverity): [Alert!]!
    alertHistory(facilityId: ID, timeRange: TimeRange, pagination: PaginationInput): AlertConnection!

    # 유지보수 조회
    upcomingMaintenance(facilityId: ID, days: Int): [MaintenanceRecord!]!
    maintenanceHistory(equipmentId: ID!, pagination: PaginationInput): [MaintenanceRecord!]!
  }

  # 변형
  type Mutation {
    # 시설 관리
    createFacility(input: CreateFacilityInput!): Facility!
    updateFacility(id: ID!, input: UpdateFacilityInput!): Facility!
    updateFacilityStatus(id: ID!, status: FacilityStatus!, reason: String): Facility!

    # 장비 관리
    createEquipment(input: CreateEquipmentInput!): Equipment!
    updateEquipment(id: ID!, input: UpdateEquipmentInput!): Equipment!
    executeEquipmentCommand(equipmentId: ID!, command: String!, parameters: JSON): CommandResult!

    # 알림 관리
    acknowledgeAlert(alertId: ID!, notes: String): Alert!
    resolveAlert(alertId: ID!, resolution: String!, preventiveAction: String): Alert!

    # 유지보수 관리
    scheduleMaintenance(input: ScheduleMaintenanceInput!): MaintenanceRecord!
    completeMaintenance(id: ID!, input: CompleteMaintenanceInput!): MaintenanceRecord!
  }

  type CommandResult {
    success: Boolean!
    message: String!
    data: JSON
  }

  input CreateFacilityInput {
    name: String!
    type: FacilityType!
    location: LocationInput!
    capacity: CapacityInput!
    contact: ContactInput!
  }

  input LocationInput {
    address: String!
    city: String!
    country: String!
    postalCode: String!
    latitude: Float
    longitude: Float
  }

  input CapacityInput {
    totalStorageUnits: Int!
    maxSpecimens: Int!
  }

  input ContactInput {
    email: String!
    phone: String!
    emergencyPhone: String!
  }

  input UpdateFacilityInput {
    name: String
    location: LocationInput
    capacity: CapacityInput
    contact: ContactInput
  }

  input CreateEquipmentInput {
    name: String!
    category: EquipmentCategory!
    facilityId: ID!
    zoneId: ID!
    manufacturer: String!
    model: String!
    serialNumber: String!
    specifications: JSON!
    installationDate: DateTime!
  }

  input UpdateEquipmentInput {
    name: String
    specifications: JSON
    zoneId: ID
  }

  input ScheduleMaintenanceInput {
    equipmentId: ID!
    type: String!
    title: String!
    description: String!
    scheduledDate: DateTime!
    assignedTo: ID
    priority: String
  }

  input CompleteMaintenanceInput {
    workPerformed: String!
    partsUsed: [PartUsedInput!]
    notes: String
  }

  input PartUsedInput {
    partNumber: String!
    description: String!
    quantity: Int!
  }

  # 구독
  type Subscription {
    # 실시간 센서 데이터
    sensorReadings(equipmentId: ID!): Reading!

    # 알림 구독
    alertTriggered(facilityId: ID, severity: AlertSeverity): Alert!
    alertAcknowledged(facilityId: ID): Alert!
    alertResolved(facilityId: ID): Alert!

    # 장비 상태 변경
    equipmentStatusChanged(facilityId: ID): Equipment!
  }
`;
```

### 4.3.2 GraphQL 리졸버

```typescript
import { PubSub } from 'graphql-subscriptions';

const pubsub = new PubSub();

// 이벤트 토픽
const TOPICS = {
  SENSOR_READING: 'SENSOR_READING',
  ALERT_TRIGGERED: 'ALERT_TRIGGERED',
  ALERT_ACKNOWLEDGED: 'ALERT_ACKNOWLEDGED',
  ALERT_RESOLVED: 'ALERT_RESOLVED',
  EQUIPMENT_STATUS_CHANGED: 'EQUIPMENT_STATUS_CHANGED'
};

// GraphQL 리졸버
const resolvers = {
  Query: {
    // 시설 조회
    facility: async (_: any, { id }: { id: string }, context: GraphQLContext) => {
      return context.services.facility.findById(id);
    },

    facilities: async (
      _: any,
      { filter, pagination }: { filter?: any; pagination?: any },
      context: GraphQLContext
    ) => {
      const result = await context.services.facility.findAll({
        filters: filter,
        pagination: pagination || { page: 1, limit: 20 }
      });

      return {
        items: result.items,
        pageInfo: {
          page: result.page,
          limit: result.limit,
          total: result.total,
          totalPages: result.totalPages,
          hasNextPage: result.page < result.totalPages,
          hasPreviousPage: result.page > 1
        }
      };
    },

    facilityDashboard: async (
      _: any,
      { facilityId, timeRange }: { facilityId: string; timeRange?: any },
      context: GraphQLContext
    ) => {
      return context.services.facility.getDashboard(facilityId, { timeRange });
    },

    // 장비 조회
    equipment: async (_: any, { id }: { id: string }, context: GraphQLContext) => {
      return context.services.equipment.findById(id);
    },

    equipmentList: async (
      _: any,
      { filter, pagination }: { filter?: any; pagination?: any },
      context: GraphQLContext
    ) => {
      const result = await context.services.equipment.findAll({
        filters: filter,
        pagination: pagination || { page: 1, limit: 20 }
      });

      return {
        items: result.items,
        pageInfo: {
          page: result.page,
          limit: result.limit,
          total: result.total,
          totalPages: result.totalPages,
          hasNextPage: result.page < result.totalPages,
          hasPreviousPage: result.page > 1
        }
      };
    },

    equipmentReadings: async (
      _: any,
      { equipmentId, timeRange, resolution }: any,
      context: GraphQLContext
    ) => {
      return context.services.equipment.getReadings(equipmentId, {
        timeRange,
        resolution: resolution || 'minute'
      });
    },

    // 알림 조회
    activeAlerts: async (
      _: any,
      { facilityId, severity }: { facilityId?: string; severity?: string },
      context: GraphQLContext
    ) => {
      return context.services.alert.getActiveAlerts({ facilityId, severity });
    },

    alertHistory: async (
      _: any,
      { facilityId, timeRange, pagination }: any,
      context: GraphQLContext
    ) => {
      const result = await context.services.alert.getAlertHistory({
        filters: { facilityId, timeRange },
        pagination: pagination || { page: 1, limit: 20 }
      });

      return {
        items: result.items,
        pageInfo: {
          page: result.page,
          limit: result.limit,
          total: result.total,
          totalPages: result.totalPages,
          hasNextPage: result.page < result.totalPages,
          hasPreviousPage: result.page > 1
        }
      };
    },

    // 유지보수 조회
    upcomingMaintenance: async (
      _: any,
      { facilityId, days }: { facilityId?: string; days?: number },
      context: GraphQLContext
    ) => {
      return context.services.maintenance.getUpcoming({
        facilityId,
        days: days || 30
      });
    }
  },

  Mutation: {
    // 시설 관리
    createFacility: async (_: any, { input }: any, context: GraphQLContext) => {
      context.requireRole(['admin', 'facility-manager']);
      return context.services.facility.create(input, {
        createdBy: context.user!.id
      });
    },

    updateFacility: async (_: any, { id, input }: any, context: GraphQLContext) => {
      context.requireRole(['admin', 'facility-manager']);
      return context.services.facility.update(id, input, {
        updatedBy: context.user!.id
      });
    },

    updateFacilityStatus: async (
      _: any,
      { id, status, reason }: any,
      context: GraphQLContext
    ) => {
      context.requireRole(['admin', 'facility-manager']);
      return context.services.facility.updateStatus(id, status, {
        reason,
        updatedBy: context.user!.id
      });
    },

    // 장비 관리
    createEquipment: async (_: any, { input }: any, context: GraphQLContext) => {
      context.requireRole(['admin', 'facility-manager', 'equipment-manager']);
      return context.services.equipment.create(input, {
        createdBy: context.user!.id
      });
    },

    executeEquipmentCommand: async (
      _: any,
      { equipmentId, command, parameters }: any,
      context: GraphQLContext
    ) => {
      context.requireRole(['admin', 'facility-manager', 'equipment-operator']);
      return context.services.equipment.executeCommand(equipmentId, {
        command,
        parameters,
        executedBy: context.user!.id
      });
    },

    // 알림 관리
    acknowledgeAlert: async (
      _: any,
      { alertId, notes }: any,
      context: GraphQLContext
    ) => {
      context.requireRole(['admin', 'facility-manager', 'operator']);

      const alert = await context.services.alert.acknowledge(alertId, {
        acknowledgedBy: context.user!.id,
        notes
      });

      pubsub.publish(TOPICS.ALERT_ACKNOWLEDGED, { alertAcknowledged: alert });

      return alert;
    },

    resolveAlert: async (
      _: any,
      { alertId, resolution, preventiveAction }: any,
      context: GraphQLContext
    ) => {
      context.requireRole(['admin', 'facility-manager', 'operator']);

      const alert = await context.services.alert.resolve(alertId, {
        resolvedBy: context.user!.id,
        resolution,
        preventiveAction
      });

      pubsub.publish(TOPICS.ALERT_RESOLVED, { alertResolved: alert });

      return alert;
    },

    // 유지보수 관리
    scheduleMaintenance: async (_: any, { input }: any, context: GraphQLContext) => {
      context.requireRole(['admin', 'facility-manager', 'maintenance-technician']);
      return context.services.maintenance.schedule(input, {
        createdBy: context.user!.id
      });
    },

    completeMaintenance: async (
      _: any,
      { id, input }: any,
      context: GraphQLContext
    ) => {
      context.requireRole(['admin', 'facility-manager', 'maintenance-technician']);
      return context.services.maintenance.complete(id, input, {
        completedBy: context.user!.id
      });
    }
  },

  Subscription: {
    sensorReadings: {
      subscribe: (_: any, { equipmentId }: { equipmentId: string }) => {
        return pubsub.asyncIterator(`${TOPICS.SENSOR_READING}.${equipmentId}`);
      }
    },

    alertTriggered: {
      subscribe: (_: any, { facilityId, severity }: any) => {
        const topic = facilityId
          ? `${TOPICS.ALERT_TRIGGERED}.${facilityId}`
          : TOPICS.ALERT_TRIGGERED;
        return pubsub.asyncIterator(topic);
      },
      resolve: (payload: any, { severity }: any) => {
        if (severity && payload.alertTriggered.severity !== severity) {
          return null;
        }
        return payload.alertTriggered;
      }
    },

    alertAcknowledged: {
      subscribe: (_: any, { facilityId }: any) => {
        const topic = facilityId
          ? `${TOPICS.ALERT_ACKNOWLEDGED}.${facilityId}`
          : TOPICS.ALERT_ACKNOWLEDGED;
        return pubsub.asyncIterator(topic);
      }
    },

    alertResolved: {
      subscribe: (_: any, { facilityId }: any) => {
        const topic = facilityId
          ? `${TOPICS.ALERT_RESOLVED}.${facilityId}`
          : TOPICS.ALERT_RESOLVED;
        return pubsub.asyncIterator(topic);
      }
    },

    equipmentStatusChanged: {
      subscribe: (_: any, { facilityId }: any) => {
        const topic = facilityId
          ? `${TOPICS.EQUIPMENT_STATUS_CHANGED}.${facilityId}`
          : TOPICS.EQUIPMENT_STATUS_CHANGED;
        return pubsub.asyncIterator(topic);
      }
    }
  },

  // 타입 리졸버
  Facility: {
    zones: async (facility: any, _: any, context: GraphQLContext) => {
      return context.services.zone.findByFacility(facility.id);
    },
    equipment: async (facility: any, _: any, context: GraphQLContext) => {
      return context.services.equipment.findByFacility(facility.id);
    }
  },

  Equipment: {
    facility: async (equipment: any, _: any, context: GraphQLContext) => {
      return context.services.facility.findById(equipment.facilityId);
    },
    zone: async (equipment: any, _: any, context: GraphQLContext) => {
      return context.services.zone.findById(equipment.zoneId);
    },
    sensors: async (equipment: any, _: any, context: GraphQLContext) => {
      return context.services.sensor.findByEquipment(equipment.id);
    },
    currentReadings: async (equipment: any, _: any, context: GraphQLContext) => {
      return context.services.reading.getCurrentByEquipment(equipment.id);
    },
    alerts: async (equipment: any, _: any, context: GraphQLContext) => {
      return context.services.alert.getActiveByEquipment(equipment.id);
    }
  },

  Sensor: {
    currentReading: async (sensor: any, _: any, context: GraphQLContext) => {
      return context.services.reading.getLatest(sensor.id);
    }
  }
};

// 실행 가능한 스키마 생성
export const schema = makeExecutableSchema({
  typeDefs,
  resolvers
});
```

## 4.4 WebSocket 실시간 통신

### 4.4.1 WebSocket 서버

```typescript
import { WebSocketServer, WebSocket } from 'ws';
import { IncomingMessage } from 'http';

// WebSocket 메시지 타입
interface WSMessage {
  type: string;
  topic?: string;
  payload?: any;
}

// WebSocket 클라이언트 정보
interface WSClient {
  id: string;
  ws: WebSocket;
  user: User;
  subscriptions: Set<string>;
  facilityId?: string;
  lastPing: number;
}

// 실시간 모니터링 WebSocket 서버
export class CryoFacilityWebSocketServer {
  private wss: WebSocketServer;
  private clients: Map<string, WSClient> = new Map();
  private heartbeatInterval: NodeJS.Timeout;

  constructor(server: any, private authService: AuthService) {
    this.wss = new WebSocketServer({
      server,
      path: '/ws'
    });

    this.wss.on('connection', this.handleConnection.bind(this));

    // 하트비트 체크 (30초마다)
    this.heartbeatInterval = setInterval(() => {
      this.checkHeartbeats();
    }, 30000);
  }

  private async handleConnection(ws: WebSocket, req: IncomingMessage) {
    try {
      // 인증
      const token = this.extractToken(req);
      if (!token) {
        ws.close(4001, '인증 필요');
        return;
      }

      const user = await this.authService.verifyToken(token);
      if (!user) {
        ws.close(4002, '유효하지 않은 토큰');
        return;
      }

      // 클라이언트 등록
      const clientId = this.generateClientId();
      const client: WSClient = {
        id: clientId,
        ws,
        user,
        subscriptions: new Set(),
        lastPing: Date.now()
      };

      this.clients.set(clientId, client);

      // 연결 확인 메시지
      this.send(ws, {
        type: 'connected',
        payload: {
          clientId,
          userId: user.id,
          timestamp: new Date().toISOString()
        }
      });

      // 메시지 핸들러
      ws.on('message', (data) => this.handleMessage(client, data));
      ws.on('close', () => this.handleDisconnect(client));
      ws.on('pong', () => { client.lastPing = Date.now(); });

    } catch (error) {
      console.error('WebSocket 연결 오류:', error);
      ws.close(4000, '연결 오류');
    }
  }

  private handleMessage(client: WSClient, data: any) {
    try {
      const message: WSMessage = JSON.parse(data.toString());

      switch (message.type) {
        case 'subscribe':
          this.handleSubscribe(client, message);
          break;

        case 'unsubscribe':
          this.handleUnsubscribe(client, message);
          break;

        case 'ping':
          this.send(client.ws, { type: 'pong' });
          break;

        case 'command':
          this.handleCommand(client, message);
          break;

        default:
          this.send(client.ws, {
            type: 'error',
            payload: { message: `알 수 없는 메시지 타입: ${message.type}` }
          });
      }
    } catch (error) {
      console.error('메시지 처리 오류:', error);
      this.send(client.ws, {
        type: 'error',
        payload: { message: '메시지 처리 실패' }
      });
    }
  }

  private handleSubscribe(client: WSClient, message: WSMessage) {
    const { topic, payload } = message;

    if (!topic) {
      this.send(client.ws, {
        type: 'error',
        payload: { message: '구독 토픽이 필요합니다' }
      });
      return;
    }

    // 구독 토픽 유효성 검사
    const validTopics = [
      'facility.*',
      'equipment.*',
      'sensor.*',
      'alert.*',
      'maintenance.*'
    ];

    const isValidTopic = validTopics.some(pattern => {
      const regex = new RegExp('^' + pattern.replace('*', '.*') + '$');
      return regex.test(topic);
    });

    if (!isValidTopic) {
      this.send(client.ws, {
        type: 'error',
        payload: { message: `유효하지 않은 토픽: ${topic}` }
      });
      return;
    }

    // 권한 확인
    if (topic.startsWith('facility.')) {
      const facilityId = topic.split('.')[1];
      // 시설 접근 권한 확인 로직
    }

    client.subscriptions.add(topic);

    this.send(client.ws, {
      type: 'subscribed',
      payload: { topic, timestamp: new Date().toISOString() }
    });
  }

  private handleUnsubscribe(client: WSClient, message: WSMessage) {
    const { topic } = message;

    if (topic) {
      client.subscriptions.delete(topic);
    }

    this.send(client.ws, {
      type: 'unsubscribed',
      payload: { topic, timestamp: new Date().toISOString() }
    });
  }

  private async handleCommand(client: WSClient, message: WSMessage) {
    const { payload } = message;

    if (!payload?.equipmentId || !payload?.command) {
      this.send(client.ws, {
        type: 'error',
        payload: { message: '장비 ID와 명령이 필요합니다' }
      });
      return;
    }

    try {
      // 명령 실행 권한 확인
      const hasPermission = await this.authService.checkPermission(
        client.user.id,
        'equipment:command',
        payload.equipmentId
      );

      if (!hasPermission) {
        this.send(client.ws, {
          type: 'error',
          payload: { message: '명령 실행 권한이 없습니다' }
        });
        return;
      }

      // 명령 실행
      const result = await equipmentService.executeCommand(payload.equipmentId, {
        command: payload.command,
        parameters: payload.parameters,
        executedBy: client.user.id
      });

      this.send(client.ws, {
        type: 'command-result',
        payload: result
      });
    } catch (error) {
      this.send(client.ws, {
        type: 'error',
        payload: { message: '명령 실행 실패' }
      });
    }
  }

  private handleDisconnect(client: WSClient) {
    client.subscriptions.clear();
    this.clients.delete(client.id);
  }

  // 특정 토픽으로 메시지 브로드캐스트
  public broadcast(topic: string, message: any) {
    for (const client of this.clients.values()) {
      if (this.matchesTopic(client.subscriptions, topic)) {
        this.send(client.ws, {
          type: 'data',
          topic,
          payload: message
        });
      }
    }
  }

  // 센서 데이터 브로드캐스트
  public broadcastSensorReading(equipmentId: string, reading: any) {
    this.broadcast(`sensor.${reading.sensorId}`, reading);
    this.broadcast(`equipment.${equipmentId}.sensors`, reading);
  }

  // 알림 브로드캐스트
  public broadcastAlert(alert: any) {
    this.broadcast(`alert.${alert.facilityId}`, alert);
    if (alert.equipmentId) {
      this.broadcast(`equipment.${alert.equipmentId}.alerts`, alert);
    }
  }

  private matchesTopic(subscriptions: Set<string>, topic: string): boolean {
    for (const sub of subscriptions) {
      if (sub === topic) return true;
      if (sub.endsWith('.*')) {
        const prefix = sub.slice(0, -2);
        if (topic.startsWith(prefix)) return true;
      }
    }
    return false;
  }

  private send(ws: WebSocket, message: WSMessage) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }

  private checkHeartbeats() {
    const now = Date.now();
    const timeout = 60000; // 60초

    for (const [id, client] of this.clients) {
      if (now - client.lastPing > timeout) {
        client.ws.terminate();
        this.clients.delete(id);
      } else {
        client.ws.ping();
      }
    }
  }

  private extractToken(req: IncomingMessage): string | null {
    const url = new URL(req.url!, `http://${req.headers.host}`);
    return url.searchParams.get('token');
  }

  private generateClientId(): string {
    return `ws_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  public close() {
    clearInterval(this.heartbeatInterval);
    for (const client of this.clients.values()) {
      client.ws.close(1000, '서버 종료');
    }
    this.wss.close();
  }
}
```

## 4.5 요약

이 장에서는 극저온 시설 관리를 위한 종합적인 API 인터페이스를 설계했습니다:

| API 유형 | 용도 | 주요 기능 |
|---------|------|----------|
| REST API | CRUD 작업 | 시설, 장비, 알림, 유지보수 관리 |
| GraphQL | 유연한 쿼리 | 관계형 데이터 조회, 실시간 구독 |
| WebSocket | 실시간 통신 | 센서 데이터, 알림, 명령 실행 |

핵심 API 엔드포인트:
- `/api/v1/facilities` - 시설 관리
- `/api/v1/equipment` - 장비 관리
- `/api/v1/alerts` - 알림 관리
- `/graphql` - GraphQL 엔드포인트
- `/ws` - WebSocket 실시간 연결

다음 장에서는 장비 제어 프로토콜을 상세히 살펴봅니다.

---

© 2025 WIA Standards. All rights reserved.
