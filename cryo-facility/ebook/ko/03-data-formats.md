# 제3장: 데이터 형식 및 스키마

## 3.1 개요

극저온 시설 관리 표준은 시설 운영의 모든 측면을 포괄하는 종합적인 데이터 구조를 정의합니다. 이 장에서는 핵심 스키마, 유효성 검증 규칙, 데이터 모델링 패턴을 상세히 설명합니다.

```typescript
// 데이터 형식 목차
const dataFormatsOverview = {
  coreSchemas: [
    'ProjectMetadata',      // 프로젝트 기본 정보
    'FacilityConfiguration', // 시설 구성
    'EquipmentInventory',   // 장비 인벤토리
    'OperationsManagement', // 운영 관리
    'SafetyManagement',     // 안전 관리
    'QualityManagement',    // 품질 관리
    'EnvironmentalControl'  // 환경 제어
  ],
  supportingSchemas: [
    'ZoneDefinition',       // 구역 정의
    'SensorConfiguration',  // 센서 설정
    'MaintenanceRecord',    // 유지보수 기록
    'IncidentReport',       // 사고 보고서
    'AuditTrail'           // 감사 추적
  ],
  validationRules: [
    'TemperatureRanges',    // 온도 범위
    'CapacityLimits',       // 용량 한계
    'CalibrationIntervals', // 교정 주기
    'SafetyThresholds'      // 안전 임계값
  ]
};
```

## 3.2 핵심 스키마 정의

### 3.2.1 프로젝트 메타데이터 스키마

```typescript
import { z } from 'zod';

// 시설 유형 열거형
export const FacilityTypeSchema = z.enum([
  'biobank',           // 바이오뱅크
  'tissue-bank',       // 조직은행
  'fertility-center',  // 생식의학센터
  'research-facility', // 연구시설
  'hospital-unit',     // 병원 유닛
  'commercial-storage' // 상업적 보관
]);

export type FacilityType = z.infer<typeof FacilityTypeSchema>;

// 시설 상태 열거형
export const FacilityStatusSchema = z.enum([
  'planning',      // 계획 단계
  'construction',  // 건설 중
  'commissioning', // 시운전
  'operational',   // 운영 중
  'maintenance',   // 유지보수 중
  'suspended',     // 일시 중단
  'decommissioned' // 폐쇄
]);

export type FacilityStatus = z.infer<typeof FacilityStatusSchema>;

// 프로젝트 메타데이터 스키마
export const ProjectMetadataSchema = z.object({
  projectId: z.string().uuid(),
  projectName: z.string().min(1).max(200),
  version: z.string().regex(/^\d+\.\d+\.\d+$/),

  facility: z.object({
    facilityId: z.string().uuid(),
    name: z.string().min(1).max(200),
    type: FacilityTypeSchema,
    status: FacilityStatusSchema,

    location: z.object({
      address: z.string(),
      city: z.string(),
      country: z.string().length(2), // ISO 3166-1 alpha-2
      postalCode: z.string(),
      coordinates: z.object({
        latitude: z.number().min(-90).max(90),
        longitude: z.number().min(-180).max(180)
      }).optional()
    }),

    capacity: z.object({
      totalStorageUnits: z.number().positive(),
      currentOccupancy: z.number().min(0),
      maxSpecimens: z.number().positive(),
      utilizationRate: z.number().min(0).max(100)
    }),

    certifications: z.array(z.object({
      type: z.string(),
      issuer: z.string(),
      validFrom: z.string().datetime(),
      validUntil: z.string().datetime(),
      certificateNumber: z.string()
    }))
  }),

  organization: z.object({
    organizationId: z.string().uuid(),
    name: z.string(),
    department: z.string().optional(),
    contact: z.object({
      email: z.string().email(),
      phone: z.string(),
      emergencyPhone: z.string()
    })
  }),

  timestamps: z.object({
    createdAt: z.string().datetime(),
    updatedAt: z.string().datetime(),
    lastAuditAt: z.string().datetime().optional()
  })
});

export type ProjectMetadata = z.infer<typeof ProjectMetadataSchema>;
```

### 3.2.2 시설 구성 스키마

```typescript
// 청정도 등급 (ISO 14644-1)
export const CleanroomClassSchema = z.enum([
  'ISO-1', 'ISO-2', 'ISO-3', 'ISO-4', 'ISO-5',
  'ISO-6', 'ISO-7', 'ISO-8', 'ISO-9', 'non-classified'
]);

// 구역 유형
export const ZoneTypeSchema = z.enum([
  'storage',           // 보관 구역
  'processing',        // 처리 구역
  'receiving',         // 입고 구역
  'distribution',      // 배출 구역
  'quality-control',   // 품질관리 구역
  'administrative',    // 관리 구역
  'utility',           // 유틸리티 구역
  'emergency-access'   // 비상 접근 구역
]);

// 구역 정의 스키마
export const ZoneDefinitionSchema = z.object({
  zoneId: z.string().uuid(),
  name: z.string(),
  type: ZoneTypeSchema,
  cleanroomClass: CleanroomClassSchema,

  dimensions: z.object({
    length: z.number().positive(),
    width: z.number().positive(),
    height: z.number().positive(),
    unit: z.enum(['meters', 'feet'])
  }),

  environmentalRequirements: z.object({
    temperature: z.object({
      min: z.number(),
      max: z.number(),
      target: z.number(),
      unit: z.enum(['celsius', 'fahrenheit'])
    }),
    humidity: z.object({
      min: z.number().min(0).max(100),
      max: z.number().min(0).max(100),
      target: z.number().min(0).max(100)
    }),
    pressure: z.object({
      differential: z.number(),
      unit: z.enum(['pascals', 'inches-wc'])
    }).optional(),
    airChangesPerHour: z.number().positive().optional()
  }),

  accessControl: z.object({
    requiredClearance: z.enum(['public', 'staff', 'authorized', 'restricted', 'critical']),
    requiresTwoPersonRule: z.boolean(),
    requiresBiometric: z.boolean(),
    maxOccupancy: z.number().positive()
  }),

  equipment: z.array(z.string().uuid())
});

export type ZoneDefinition = z.infer<typeof ZoneDefinitionSchema>;

// 인프라 요구사항 스키마
export const InfrastructureRequirementsSchema = z.object({
  electrical: z.object({
    primaryPower: z.object({
      voltage: z.number(),
      frequency: z.number(),
      capacity: z.number(),
      unit: z.literal('kVA')
    }),
    backupPower: z.object({
      type: z.enum(['generator', 'ups', 'dual-feed', 'combined']),
      capacity: z.number(),
      runtime: z.number(),
      runtimeUnit: z.enum(['minutes', 'hours'])
    }),
    upsSystem: z.object({
      capacity: z.number(),
      runtime: z.number(),
      batteryType: z.string()
    }).optional()
  }),

  cryogenicSupply: z.object({
    primarySource: z.enum(['bulk-tank', 'dewar', 'on-site-generator', 'cylinder-manifold']),
    backupSource: z.enum(['bulk-tank', 'dewar', 'cylinder-manifold', 'none']),
    deliverySchedule: z.enum(['daily', 'weekly', 'bi-weekly', 'monthly', 'on-demand']),
    minimumReserve: z.object({
      quantity: z.number(),
      unit: z.enum(['liters', 'gallons', 'days-supply'])
    })
  }),

  hvac: z.object({
    coolingCapacity: z.number(),
    coolingUnit: z.literal('tons'),
    heatingCapacity: z.number().optional(),
    heatingUnit: z.literal('kW').optional(),
    filtration: z.object({
      preFilter: z.string(),
      mainFilter: z.string(),
      hepaFilter: z.boolean()
    }),
    redundancy: z.enum(['none', 'N+1', 'N+2', '2N'])
  }),

  plumbing: z.object({
    waterSupply: z.boolean(),
    drainage: z.boolean(),
    emergencyShower: z.boolean(),
    eyewashStation: z.boolean()
  }),

  fireProtection: z.object({
    detectionType: z.array(z.enum(['smoke', 'heat', 'flame', 'aspiration'])),
    suppressionType: z.enum(['sprinkler', 'gas', 'foam', 'dry-chemical', 'none']),
    alarmSystem: z.boolean(),
    emergencyLighting: z.boolean()
  })
});

export type InfrastructureRequirements = z.infer<typeof InfrastructureRequirementsSchema>;

// 시설 구성 스키마
export const FacilityConfigurationSchema = z.object({
  configurationId: z.string().uuid(),
  facilityId: z.string().uuid(),
  version: z.string(),

  layout: z.object({
    totalArea: z.object({
      value: z.number().positive(),
      unit: z.enum(['square-meters', 'square-feet'])
    }),
    floors: z.number().positive(),
    zones: z.array(ZoneDefinitionSchema)
  }),

  infrastructure: InfrastructureRequirementsSchema,

  operatingHours: z.object({
    regularHours: z.array(z.object({
      dayOfWeek: z.number().min(0).max(6),
      openTime: z.string().regex(/^\d{2}:\d{2}$/),
      closeTime: z.string().regex(/^\d{2}:\d{2}$/)
    })),
    is24x7: z.boolean(),
    holidays: z.array(z.object({
      date: z.string().regex(/^\d{4}-\d{2}-\d{2}$/),
      description: z.string(),
      isOpen: z.boolean()
    }))
  }),

  emergencyContacts: z.array(z.object({
    role: z.string(),
    name: z.string(),
    phone: z.string(),
    email: z.string().email(),
    priority: z.number().positive()
  }))
});

export type FacilityConfiguration = z.infer<typeof FacilityConfigurationSchema>;
```

### 3.2.3 장비 인벤토리 스키마

```typescript
// 장비 카테고리
export const EquipmentCategorySchema = z.enum([
  'ln2-tank',            // 액체질소 탱크
  'mechanical-freezer',  // 기계식 냉동고
  'controlled-rate',     // 제어냉동기
  'incubator',           // 배양기
  'centrifuge',          // 원심분리기
  'monitoring-system',   // 모니터링 시스템
  'safety-equipment',    // 안전 장비
  'handling-equipment',  // 취급 장비
  'quality-control'      // 품질관리 장비
]);

export type EquipmentCategory = z.infer<typeof EquipmentCategorySchema>;

// 장비 상태
export const EquipmentStatusSchema = z.enum([
  'operational',        // 가동 중
  'standby',           // 대기 중
  'maintenance',       // 유지보수 중
  'calibration',       // 교정 중
  'fault',             // 고장
  'decommissioned',    // 퇴역
  'quarantine'         // 격리
]);

export type EquipmentStatus = z.infer<typeof EquipmentStatusSchema>;

// 센서 유형
export const SensorTypeSchema = z.enum([
  'temperature',        // 온도
  'pressure',          // 압력
  'level',             // 수위
  'humidity',          // 습도
  'oxygen',            // 산소
  'nitrogen',          // 질소
  'door-status',       // 도어 상태
  'power-status',      // 전원 상태
  'vibration'          // 진동
]);

export type SensorType = z.infer<typeof SensorTypeSchema>;

// 센서 구성 스키마
export const SensorConfigurationSchema = z.object({
  sensorId: z.string().uuid(),
  type: SensorTypeSchema,
  model: z.string(),
  serialNumber: z.string(),

  location: z.object({
    equipmentId: z.string().uuid(),
    position: z.string(),
    coordinates: z.object({
      x: z.number(),
      y: z.number(),
      z: z.number()
    }).optional()
  }),

  specifications: z.object({
    measurementRange: z.object({
      min: z.number(),
      max: z.number(),
      unit: z.string()
    }),
    accuracy: z.object({
      value: z.number(),
      type: z.enum(['absolute', 'percentage'])
    }),
    resolution: z.number(),
    responseTime: z.object({
      value: z.number(),
      unit: z.enum(['milliseconds', 'seconds'])
    })
  }),

  calibration: z.object({
    lastCalibrationDate: z.string().datetime(),
    nextCalibrationDate: z.string().datetime(),
    calibrationInterval: z.object({
      value: z.number(),
      unit: z.enum(['days', 'months', 'years'])
    }),
    calibrationCertificate: z.string().optional()
  }),

  alarmThresholds: z.object({
    warningLow: z.number().optional(),
    warningHigh: z.number().optional(),
    criticalLow: z.number().optional(),
    criticalHigh: z.number().optional()
  }),

  samplingConfig: z.object({
    interval: z.object({
      value: z.number(),
      unit: z.enum(['seconds', 'minutes'])
    }),
    averagingWindow: z.number().optional(),
    transmissionMode: z.enum(['continuous', 'on-change', 'periodic'])
  })
});

export type SensorConfiguration = z.infer<typeof SensorConfigurationSchema>;

// LN2 탱크 스키마
export const LN2TankSchema = z.object({
  equipmentId: z.string().uuid(),
  category: z.literal('ln2-tank'),

  identification: z.object({
    name: z.string(),
    manufacturer: z.string(),
    model: z.string(),
    serialNumber: z.string(),
    assetTag: z.string()
  }),

  specifications: z.object({
    capacity: z.object({
      value: z.number().positive(),
      unit: z.enum(['liters', 'gallons'])
    }),
    holdTime: z.object({
      value: z.number().positive(),
      unit: z.enum(['days', 'weeks'])
    }),
    evaporationRate: z.object({
      value: z.number(),
      unit: z.literal('liters-per-day')
    }),
    workingPressure: z.object({
      value: z.number(),
      unit: z.enum(['psi', 'bar', 'kPa'])
    }).optional(),
    storagePositions: z.number().positive()
  }),

  currentState: z.object({
    status: EquipmentStatusSchema,
    fillLevel: z.number().min(0).max(100),
    temperature: z.number(),
    lastFillDate: z.string().datetime(),
    estimatedEmptyDate: z.string().datetime()
  }),

  sensors: z.array(SensorConfigurationSchema),

  maintenance: z.object({
    lastMaintenanceDate: z.string().datetime(),
    nextMaintenanceDate: z.string().datetime(),
    maintenanceInterval: z.object({
      value: z.number(),
      unit: z.enum(['months', 'years'])
    }),
    warrantyExpiration: z.string().datetime().optional()
  }),

  location: z.object({
    zoneId: z.string().uuid(),
    position: z.string(),
    installationDate: z.string().datetime()
  })
});

export type LN2Tank = z.infer<typeof LN2TankSchema>;

// 기계식 냉동고 스키마
export const MechanicalFreezerSchema = z.object({
  equipmentId: z.string().uuid(),
  category: z.literal('mechanical-freezer'),

  identification: z.object({
    name: z.string(),
    manufacturer: z.string(),
    model: z.string(),
    serialNumber: z.string(),
    assetTag: z.string()
  }),

  specifications: z.object({
    temperatureRange: z.object({
      min: z.number(),
      max: z.number(),
      unit: z.enum(['celsius', 'fahrenheit'])
    }),
    capacity: z.object({
      internal: z.object({
        value: z.number(),
        unit: z.enum(['cubic-feet', 'liters'])
      }),
      boxCount: z.number().positive()
    }),
    compressorType: z.enum(['single', 'cascade', 'stirling']),
    refrigerant: z.string(),
    powerConsumption: z.object({
      value: z.number(),
      unit: z.literal('watts')
    }),
    co2Backup: z.boolean()
  }),

  currentState: z.object({
    status: EquipmentStatusSchema,
    currentTemperature: z.number(),
    setpointTemperature: z.number(),
    doorStatus: z.enum(['closed', 'open', 'ajar']),
    compressorRunning: z.boolean(),
    defrostActive: z.boolean()
  }),

  sensors: z.array(SensorConfigurationSchema),

  alarms: z.object({
    highTempWarning: z.number(),
    highTempCritical: z.number(),
    doorOpenWarning: z.number(), // seconds
    powerFailureDelay: z.number() // seconds
  }),

  maintenance: z.object({
    lastMaintenanceDate: z.string().datetime(),
    nextMaintenanceDate: z.string().datetime(),
    filterChangeDate: z.string().datetime().optional(),
    defrostSchedule: z.object({
      frequency: z.enum(['daily', 'weekly', 'manual']),
      duration: z.number()
    })
  }),

  location: z.object({
    zoneId: z.string().uuid(),
    position: z.string(),
    installationDate: z.string().datetime()
  })
});

export type MechanicalFreezer = z.infer<typeof MechanicalFreezerSchema>;

// 장비 인벤토리 스키마
export const EquipmentInventorySchema = z.object({
  inventoryId: z.string().uuid(),
  facilityId: z.string().uuid(),
  lastUpdated: z.string().datetime(),

  summary: z.object({
    totalEquipment: z.number(),
    operationalCount: z.number(),
    maintenanceCount: z.number(),
    faultCount: z.number(),
    byCategory: z.record(EquipmentCategorySchema, z.number())
  }),

  ln2Tanks: z.array(LN2TankSchema),
  mechanicalFreezers: z.array(MechanicalFreezerSchema),

  monitoringSystems: z.array(z.object({
    systemId: z.string().uuid(),
    name: z.string(),
    type: z.enum(['wired', 'wireless', 'hybrid']),
    connectedSensors: z.number(),
    communicationProtocol: z.enum(['modbus', 'bacnet', 'mqtt', 'proprietary']),
    status: EquipmentStatusSchema
  })),

  safetyEquipment: z.array(z.object({
    equipmentId: z.string().uuid(),
    type: z.enum(['oxygen-monitor', 'fire-extinguisher', 'emergency-shower',
                  'eyewash', 'first-aid', 'ppe-station', 'spill-kit']),
    location: z.string(),
    lastInspection: z.string().datetime(),
    nextInspection: z.string().datetime(),
    status: z.enum(['ok', 'needs-attention', 'expired', 'missing'])
  }))
});

export type EquipmentInventory = z.infer<typeof EquipmentInventorySchema>;
```

## 3.3 운영 관리 스키마

### 3.3.1 표준운영절차(SOP) 스키마

```typescript
// SOP 카테고리
export const SOPCategorySchema = z.enum([
  'sample-handling',      // 검체 취급
  'equipment-operation',  // 장비 운영
  'quality-control',      // 품질관리
  'safety',              // 안전
  'emergency',           // 비상
  'maintenance',         // 유지보수
  'documentation',       // 문서화
  'training'             // 교육훈련
]);

// SOP 스키마
export const SOPSchema = z.object({
  sopId: z.string().uuid(),
  documentNumber: z.string(),
  title: z.string(),
  category: SOPCategorySchema,

  version: z.object({
    number: z.string(),
    effectiveDate: z.string().datetime(),
    reviewDate: z.string().datetime(),
    retirementDate: z.string().datetime().optional()
  }),

  approval: z.object({
    author: z.object({
      userId: z.string().uuid(),
      name: z.string(),
      role: z.string(),
      date: z.string().datetime()
    }),
    reviewer: z.object({
      userId: z.string().uuid(),
      name: z.string(),
      role: z.string(),
      date: z.string().datetime()
    }),
    approver: z.object({
      userId: z.string().uuid(),
      name: z.string(),
      role: z.string(),
      date: z.string().datetime()
    })
  }),

  scope: z.object({
    applicableFacilities: z.array(z.string().uuid()),
    applicableRoles: z.array(z.string()),
    prerequisites: z.array(z.string())
  }),

  content: z.object({
    purpose: z.string(),
    responsibilities: z.array(z.object({
      role: z.string(),
      duties: z.array(z.string())
    })),
    materials: z.array(z.object({
      item: z.string(),
      specification: z.string().optional(),
      quantity: z.string().optional()
    })),
    procedure: z.array(z.object({
      stepNumber: z.number(),
      action: z.string(),
      details: z.string().optional(),
      criticalStep: z.boolean(),
      warningNotes: z.array(z.string()).optional()
    })),
    references: z.array(z.string())
  }),

  training: z.object({
    requiredTraining: z.boolean(),
    trainingFrequency: z.enum(['once', 'annual', 'biannual', 'as-needed']).optional(),
    competencyAssessment: z.boolean()
  }),

  changeHistory: z.array(z.object({
    version: z.string(),
    date: z.string().datetime(),
    author: z.string(),
    description: z.string()
  }))
});

export type SOP = z.infer<typeof SOPSchema>;
```

### 3.3.2 교육 관리 스키마

```typescript
// 교육 기록 스키마
export const TrainingRecordSchema = z.object({
  recordId: z.string().uuid(),

  trainee: z.object({
    userId: z.string().uuid(),
    name: z.string(),
    employeeId: z.string(),
    department: z.string(),
    role: z.string()
  }),

  training: z.object({
    trainingId: z.string().uuid(),
    title: z.string(),
    type: z.enum(['initial', 'refresher', 'advanced', 'remedial']),
    category: z.enum(['safety', 'equipment', 'procedure', 'quality', 'compliance']),
    deliveryMethod: z.enum(['classroom', 'online', 'on-the-job', 'blended']),
    duration: z.object({
      value: z.number(),
      unit: z.enum(['minutes', 'hours', 'days'])
    })
  }),

  completion: z.object({
    status: z.enum(['scheduled', 'in-progress', 'completed', 'failed', 'expired']),
    startDate: z.string().datetime(),
    completionDate: z.string().datetime().optional(),
    expirationDate: z.string().datetime().optional()
  }),

  assessment: z.object({
    required: z.boolean(),
    assessmentType: z.enum(['written', 'practical', 'both', 'none']),
    passingScore: z.number().min(0).max(100).optional(),
    actualScore: z.number().min(0).max(100).optional(),
    passed: z.boolean().optional()
  }).optional(),

  instructor: z.object({
    instructorId: z.string().uuid(),
    name: z.string(),
    qualification: z.string()
  }).optional(),

  documentation: z.object({
    certificateIssued: z.boolean(),
    certificateNumber: z.string().optional(),
    attachments: z.array(z.object({
      type: z.string(),
      filename: z.string(),
      uploadDate: z.string().datetime()
    }))
  })
});

export type TrainingRecord = z.infer<typeof TrainingRecordSchema>;

// 역량 매트릭스 스키마
export const CompetencyMatrixSchema = z.object({
  matrixId: z.string().uuid(),
  facilityId: z.string().uuid(),
  effectiveDate: z.string().datetime(),

  roles: z.array(z.object({
    roleId: z.string(),
    roleName: z.string(),
    requiredCompetencies: z.array(z.object({
      competencyId: z.string(),
      competencyName: z.string(),
      level: z.enum(['awareness', 'basic', 'proficient', 'expert']),
      mandatory: z.boolean(),
      trainingRequired: z.array(z.string()),
      requalificationPeriod: z.object({
        value: z.number(),
        unit: z.enum(['months', 'years'])
      }).optional()
    }))
  })),

  employees: z.array(z.object({
    userId: z.string().uuid(),
    currentRole: z.string(),
    competencies: z.array(z.object({
      competencyId: z.string(),
      currentLevel: z.enum(['none', 'awareness', 'basic', 'proficient', 'expert']),
      qualifiedDate: z.string().datetime().optional(),
      expirationDate: z.string().datetime().optional(),
      status: z.enum(['qualified', 'expired', 'in-training', 'not-started'])
    }))
  }))
});

export type CompetencyMatrix = z.infer<typeof CompetencyMatrixSchema>;
```

## 3.4 안전 관리 스키마

### 3.4.1 위험성 평가 스키마

```typescript
// 위험성 평가 스키마
export const HazardAssessmentSchema = z.object({
  assessmentId: z.string().uuid(),
  facilityId: z.string().uuid(),

  metadata: z.object({
    assessmentDate: z.string().datetime(),
    assessor: z.object({
      userId: z.string().uuid(),
      name: z.string(),
      qualification: z.string()
    }),
    reviewDate: z.string().datetime(),
    approvedBy: z.string()
  }),

  hazards: z.array(z.object({
    hazardId: z.string().uuid(),
    category: z.enum([
      'cryogenic',      // 극저온
      'asphyxiation',   // 질식
      'pressure',       // 압력
      'biological',     // 생물학적
      'chemical',       // 화학적
      'electrical',     // 전기
      'mechanical',     // 기계적
      'ergonomic',      // 인체공학적
      'fire'            // 화재
    ]),
    description: z.string(),
    location: z.string(),

    riskAssessment: z.object({
      likelihood: z.enum(['rare', 'unlikely', 'possible', 'likely', 'almost-certain']),
      severity: z.enum(['insignificant', 'minor', 'moderate', 'major', 'catastrophic']),
      riskLevel: z.enum(['low', 'medium', 'high', 'extreme']),
      riskScore: z.number().min(1).max(25)
    }),

    existingControls: z.array(z.object({
      controlId: z.string(),
      type: z.enum(['elimination', 'substitution', 'engineering', 'administrative', 'ppe']),
      description: z.string(),
      effectiveness: z.enum(['effective', 'partially-effective', 'ineffective'])
    })),

    residualRisk: z.object({
      likelihood: z.enum(['rare', 'unlikely', 'possible', 'likely', 'almost-certain']),
      severity: z.enum(['insignificant', 'minor', 'moderate', 'major', 'catastrophic']),
      riskLevel: z.enum(['low', 'medium', 'high', 'extreme']),
      riskScore: z.number().min(1).max(25)
    }),

    additionalControls: z.array(z.object({
      controlId: z.string(),
      type: z.enum(['elimination', 'substitution', 'engineering', 'administrative', 'ppe']),
      description: z.string(),
      responsible: z.string(),
      dueDate: z.string().datetime(),
      status: z.enum(['planned', 'in-progress', 'completed', 'verified'])
    }))
  })),

  summary: z.object({
    totalHazards: z.number(),
    byRiskLevel: z.object({
      low: z.number(),
      medium: z.number(),
      high: z.number(),
      extreme: z.number()
    }),
    overallRiskRating: z.enum(['acceptable', 'tolerable', 'unacceptable'])
  })
});

export type HazardAssessment = z.infer<typeof HazardAssessmentSchema>;
```

### 3.4.2 사고 보고서 스키마

```typescript
// 사고 보고서 스키마
export const IncidentReportSchema = z.object({
  reportId: z.string().uuid(),
  facilityId: z.string().uuid(),

  incident: z.object({
    type: z.enum([
      'near-miss',          // 아차사고
      'first-aid',          // 응급처치
      'medical-treatment',  // 치료필요
      'lost-time',          // 업무손실
      'property-damage',    // 재산피해
      'environmental',      // 환경사고
      'specimen-loss'       // 검체손실
    ]),
    category: z.enum([
      'cryogenic-exposure',  // 극저온 노출
      'asphyxiation',        // 질식
      'cold-burn',           // 동상
      'slip-fall',           // 미끄러짐/낙상
      'equipment-failure',   // 장비고장
      'power-failure',       // 정전
      'spill',               // 누출
      'other'                // 기타
    ]),

    dateTime: z.string().datetime(),
    location: z.string(),
    description: z.string(),
    immediateActions: z.string()
  }),

  reporter: z.object({
    userId: z.string().uuid(),
    name: z.string(),
    role: z.string(),
    reportDate: z.string().datetime()
  }),

  affectedPersons: z.array(z.object({
    personId: z.string().uuid().optional(),
    name: z.string(),
    role: z.enum(['employee', 'contractor', 'visitor', 'other']),
    injuryType: z.string().optional(),
    treatmentProvided: z.string().optional()
  })),

  investigation: z.object({
    investigator: z.object({
      userId: z.string().uuid(),
      name: z.string(),
      role: z.string()
    }),
    startDate: z.string().datetime(),
    completionDate: z.string().datetime().optional(),

    rootCause: z.object({
      immediate: z.array(z.string()),
      underlying: z.array(z.string()),
      rootCause: z.string()
    }).optional(),

    contributingFactors: z.array(z.object({
      category: z.enum(['human', 'equipment', 'environment', 'procedure', 'management']),
      description: z.string()
    })).optional(),

    findings: z.string().optional()
  }).optional(),

  correctiveActions: z.array(z.object({
    actionId: z.string().uuid(),
    description: z.string(),
    type: z.enum(['immediate', 'short-term', 'long-term']),
    responsible: z.object({
      userId: z.string().uuid(),
      name: z.string()
    }),
    dueDate: z.string().datetime(),
    completionDate: z.string().datetime().optional(),
    status: z.enum(['open', 'in-progress', 'completed', 'verified', 'closed']),
    effectiveness: z.enum(['effective', 'partially-effective', 'ineffective']).optional()
  })),

  regulatoryReporting: z.object({
    reportable: z.boolean(),
    agencies: z.array(z.string()),
    reportedDate: z.string().datetime().optional(),
    referenceNumber: z.string().optional()
  }),

  status: z.enum(['draft', 'submitted', 'under-investigation', 'closed'])
});

export type IncidentReport = z.infer<typeof IncidentReportSchema>;
```

## 3.5 품질 관리 스키마

### 3.5.1 감사 스키마

```typescript
// 감사 스키마
export const AuditSchema = z.object({
  auditId: z.string().uuid(),
  facilityId: z.string().uuid(),

  auditInfo: z.object({
    type: z.enum(['internal', 'external', 'regulatory', 'certification']),
    scope: z.array(z.enum([
      'facility-management',
      'equipment-management',
      'quality-system',
      'safety',
      'documentation',
      'training',
      'specimen-management'
    ])),
    standard: z.string(),
    scheduledDate: z.string().datetime(),
    actualDate: z.string().datetime().optional(),
    status: z.enum(['planned', 'in-progress', 'completed', 'cancelled'])
  }),

  auditors: z.array(z.object({
    auditorId: z.string().uuid(),
    name: z.string(),
    organization: z.string(),
    role: z.enum(['lead-auditor', 'auditor', 'observer']),
    qualifications: z.array(z.string())
  })),

  auditees: z.array(z.object({
    userId: z.string().uuid(),
    name: z.string(),
    role: z.string(),
    department: z.string()
  })),

  findings: z.array(z.object({
    findingId: z.string().uuid(),
    category: z.enum([
      'major-nonconformity',     // 주요 부적합
      'minor-nonconformity',     // 경미한 부적합
      'observation',             // 관찰사항
      'opportunity-improvement', // 개선기회
      'good-practice'            // 우수사례
    ]),
    clause: z.string(),
    description: z.string(),
    evidence: z.string(),
    response: z.object({
      rootCause: z.string(),
      correction: z.string(),
      correctiveAction: z.string(),
      dueDate: z.string().datetime(),
      responsible: z.string()
    }).optional(),
    status: z.enum(['open', 'responded', 'verified', 'closed']),
    verificationDate: z.string().datetime().optional(),
    verificationNotes: z.string().optional()
  })),

  summary: z.object({
    overallAssessment: z.enum(['satisfactory', 'needs-improvement', 'unsatisfactory']),
    strengths: z.array(z.string()),
    areasForImprovement: z.array(z.string()),
    conclusion: z.string()
  }).optional(),

  documents: z.array(z.object({
    documentId: z.string().uuid(),
    type: z.enum(['audit-plan', 'checklist', 'report', 'evidence', 'response']),
    filename: z.string(),
    uploadDate: z.string().datetime()
  }))
});

export type Audit = z.infer<typeof AuditSchema>;
```

### 3.5.2 CAPA 스키마

```typescript
// CAPA (시정 및 예방조치) 스키마
export const CAPASchema = z.object({
  capaId: z.string().uuid(),
  facilityId: z.string().uuid(),

  initiation: z.object({
    source: z.enum([
      'audit-finding',       // 감사 발견사항
      'incident',            // 사고
      'complaint',           // 불만
      'deviation',           // 일탈
      'trend-analysis',      // 추세분석
      'management-review',   // 경영검토
      'regulatory',          // 규제
      'self-identified'      // 자체 식별
    ]),
    sourceReference: z.string(),
    description: z.string(),
    initiatedBy: z.object({
      userId: z.string().uuid(),
      name: z.string()
    }),
    initiatedDate: z.string().datetime(),
    priority: z.enum(['low', 'medium', 'high', 'critical'])
  }),

  investigation: z.object({
    assignedTo: z.object({
      userId: z.string().uuid(),
      name: z.string()
    }),
    dueDate: z.string().datetime(),

    problemStatement: z.string(),
    containmentActions: z.array(z.object({
      action: z.string(),
      responsible: z.string(),
      dueDate: z.string().datetime(),
      completedDate: z.string().datetime().optional(),
      status: z.enum(['pending', 'in-progress', 'completed'])
    })),

    rootCauseAnalysis: z.object({
      method: z.enum(['5-whys', 'fishbone', 'fault-tree', 'pareto', 'other']),
      analysis: z.string(),
      rootCauses: z.array(z.string())
    }),

    riskAssessment: z.object({
      impactedProcesses: z.array(z.string()),
      impactedProducts: z.array(z.string()),
      riskLevel: z.enum(['low', 'medium', 'high'])
    })
  }),

  correctiveActions: z.array(z.object({
    actionId: z.string().uuid(),
    description: z.string(),
    responsible: z.object({
      userId: z.string().uuid(),
      name: z.string()
    }),
    dueDate: z.string().datetime(),
    completedDate: z.string().datetime().optional(),
    evidence: z.string().optional(),
    status: z.enum(['pending', 'in-progress', 'completed', 'verified'])
  })),

  preventiveActions: z.array(z.object({
    actionId: z.string().uuid(),
    description: z.string(),
    responsible: z.object({
      userId: z.string().uuid(),
      name: z.string()
    }),
    dueDate: z.string().datetime(),
    completedDate: z.string().datetime().optional(),
    status: z.enum(['pending', 'in-progress', 'completed', 'verified'])
  })),

  effectiveness: z.object({
    verificationMethod: z.string(),
    verificationDate: z.string().datetime().optional(),
    verifiedBy: z.object({
      userId: z.string().uuid(),
      name: z.string()
    }).optional(),
    result: z.enum(['effective', 'partially-effective', 'ineffective']).optional(),
    followUpRequired: z.boolean(),
    notes: z.string().optional()
  }),

  closure: z.object({
    closedDate: z.string().datetime().optional(),
    closedBy: z.object({
      userId: z.string().uuid(),
      name: z.string()
    }).optional(),
    closureNotes: z.string().optional()
  }),

  status: z.enum(['open', 'investigation', 'action', 'verification', 'closed'])
});

export type CAPA = z.infer<typeof CAPASchema>;
```

## 3.6 환경 모니터링 스키마

### 3.6.1 환경 데이터 스키마

```typescript
// 환경 측정값 스키마
export const EnvironmentalReadingSchema = z.object({
  readingId: z.string().uuid(),
  sensorId: z.string().uuid(),
  equipmentId: z.string().uuid(),
  zoneId: z.string().uuid(),

  timestamp: z.string().datetime(),

  measurement: z.object({
    type: SensorTypeSchema,
    value: z.number(),
    unit: z.string(),
    quality: z.enum(['good', 'uncertain', 'bad'])
  }),

  status: z.object({
    inSpec: z.boolean(),
    alarmState: z.enum(['normal', 'warning', 'critical', 'acknowledged']),
    acknowledgedBy: z.string().uuid().optional(),
    acknowledgedAt: z.string().datetime().optional()
  })
});

export type EnvironmentalReading = z.infer<typeof EnvironmentalReadingSchema>;

// 환경 추세 데이터 스키마
export const EnvironmentalTrendSchema = z.object({
  trendId: z.string().uuid(),
  sensorId: z.string().uuid(),

  period: z.object({
    start: z.string().datetime(),
    end: z.string().datetime(),
    resolution: z.enum(['minute', 'hour', 'day', 'week', 'month'])
  }),

  statistics: z.object({
    count: z.number(),
    min: z.number(),
    max: z.number(),
    mean: z.number(),
    median: z.number(),
    standardDeviation: z.number(),
    percentile95: z.number(),
    percentile99: z.number()
  }),

  compliance: z.object({
    totalReadings: z.number(),
    inSpecReadings: z.number(),
    complianceRate: z.number().min(0).max(100),
    excursions: z.array(z.object({
      startTime: z.string().datetime(),
      endTime: z.string().datetime(),
      maxDeviation: z.number(),
      type: z.enum(['high', 'low'])
    }))
  }),

  dataPoints: z.array(z.object({
    timestamp: z.string().datetime(),
    value: z.number(),
    min: z.number().optional(),
    max: z.number().optional()
  }))
});

export type EnvironmentalTrend = z.infer<typeof EnvironmentalTrendSchema>;
```

## 3.7 데이터 유효성 검증

### 3.7.1 유효성 검증 유틸리티

```typescript
import { z, ZodError, ZodSchema } from 'zod';

// 유효성 검증 결과 타입
export interface ValidationResult<T> {
  success: boolean;
  data?: T;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  code: string;
}

// 데이터 유효성 검증기
export class CryoFacilityDataValidator {
  // 스키마별 유효성 검증
  static validate<T>(
    schema: ZodSchema<T>,
    data: unknown
  ): ValidationResult<T> {
    try {
      const result = schema.parse(data);
      return {
        success: true,
        data: result
      };
    } catch (error) {
      if (error instanceof ZodError) {
        return {
          success: false,
          errors: error.errors.map(err => ({
            path: err.path.join('.'),
            message: err.message,
            code: err.code
          }))
        };
      }
      throw error;
    }
  }

  // 비동기 유효성 검증
  static async validateAsync<T>(
    schema: ZodSchema<T>,
    data: unknown
  ): Promise<ValidationResult<T>> {
    try {
      const result = await schema.parseAsync(data);
      return {
        success: true,
        data: result
      };
    } catch (error) {
      if (error instanceof ZodError) {
        return {
          success: false,
          errors: error.errors.map(err => ({
            path: err.path.join('.'),
            message: err.message,
            code: err.code
          }))
        };
      }
      throw error;
    }
  }

  // 부분 유효성 검증
  static validatePartial<T>(
    schema: ZodSchema<T>,
    data: unknown,
    paths: string[]
  ): ValidationResult<Partial<T>> {
    const partialSchema = schema.partial();
    const result = this.validate(partialSchema, data);

    if (result.success) {
      // 지정된 경로만 포함
      const filteredData = {} as Partial<T>;
      for (const path of paths) {
        const value = this.getNestedValue(result.data!, path);
        if (value !== undefined) {
          this.setNestedValue(filteredData, path, value);
        }
      }
      return { success: true, data: filteredData };
    }

    return result;
  }

  // 중첩된 값 가져오기
  private static getNestedValue(obj: any, path: string): any {
    return path.split('.').reduce((o, p) => o?.[p], obj);
  }

  // 중첩된 값 설정하기
  private static setNestedValue(obj: any, path: string, value: any): void {
    const parts = path.split('.');
    const last = parts.pop()!;
    const target = parts.reduce((o, p) => {
      if (!(p in o)) o[p] = {};
      return o[p];
    }, obj);
    target[last] = value;
  }
}

// 사용 예시
async function validateFacilityData(): Promise<void> {
  const facilityData = {
    configurationId: '550e8400-e29b-41d4-a716-446655440000',
    facilityId: '550e8400-e29b-41d4-a716-446655440001',
    version: '1.0.0',
    layout: {
      totalArea: { value: 500, unit: 'square-meters' },
      floors: 1,
      zones: []
    },
    infrastructure: {
      electrical: {
        primaryPower: {
          voltage: 380,
          frequency: 60,
          capacity: 200,
          unit: 'kVA'
        },
        backupPower: {
          type: 'combined',
          capacity: 150,
          runtime: 24,
          runtimeUnit: 'hours'
        }
      },
      cryogenicSupply: {
        primarySource: 'bulk-tank',
        backupSource: 'dewar',
        deliverySchedule: 'weekly',
        minimumReserve: { quantity: 7, unit: 'days-supply' }
      },
      hvac: {
        coolingCapacity: 20,
        coolingUnit: 'tons',
        filtration: {
          preFilter: 'MERV-8',
          mainFilter: 'MERV-13',
          hepaFilter: true
        },
        redundancy: 'N+1'
      },
      plumbing: {
        waterSupply: true,
        drainage: true,
        emergencyShower: true,
        eyewashStation: true
      },
      fireProtection: {
        detectionType: ['smoke', 'heat'],
        suppressionType: 'gas',
        alarmSystem: true,
        emergencyLighting: true
      }
    },
    operatingHours: {
      regularHours: [
        { dayOfWeek: 1, openTime: '08:00', closeTime: '18:00' },
        { dayOfWeek: 2, openTime: '08:00', closeTime: '18:00' },
        { dayOfWeek: 3, openTime: '08:00', closeTime: '18:00' },
        { dayOfWeek: 4, openTime: '08:00', closeTime: '18:00' },
        { dayOfWeek: 5, openTime: '08:00', closeTime: '18:00' }
      ],
      is24x7: false,
      holidays: []
    },
    emergencyContacts: [
      {
        role: '시설관리자',
        name: '김철수',
        phone: '+82-10-1234-5678',
        email: 'facility@example.com',
        priority: 1
      }
    ]
  };

  const result = CryoFacilityDataValidator.validate(
    FacilityConfigurationSchema,
    facilityData
  );

  if (result.success) {
    console.log('유효성 검증 성공:', result.data);
  } else {
    console.error('유효성 검증 실패:', result.errors);
  }
}
```

## 3.8 데이터 직렬화 및 역직렬화

### 3.8.1 JSON 직렬화

```typescript
// JSON 직렬화 유틸리티
export class CryoFacilitySerializer {
  // JSON 직렬화
  static toJSON<T>(data: T, pretty: boolean = false): string {
    const replacer = (key: string, value: any) => {
      // Date 객체 처리
      if (value instanceof Date) {
        return value.toISOString();
      }
      // BigInt 처리
      if (typeof value === 'bigint') {
        return value.toString();
      }
      // Buffer 처리
      if (Buffer.isBuffer(value)) {
        return value.toString('base64');
      }
      return value;
    };

    return pretty
      ? JSON.stringify(data, replacer, 2)
      : JSON.stringify(data, replacer);
  }

  // JSON 역직렬화
  static fromJSON<T>(
    json: string,
    schema?: ZodSchema<T>
  ): T {
    const reviver = (key: string, value: any) => {
      // ISO 날짜 문자열을 Date로 변환
      if (typeof value === 'string' &&
          /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}/.test(value)) {
        return new Date(value);
      }
      return value;
    };

    const data = JSON.parse(json, reviver);

    if (schema) {
      const result = CryoFacilityDataValidator.validate(schema, data);
      if (!result.success) {
        throw new Error(`JSON 역직렬화 유효성 검증 실패: ${JSON.stringify(result.errors)}`);
      }
      return result.data!;
    }

    return data as T;
  }

  // 스트림 직렬화
  static *toJSONStream<T>(items: Iterable<T>): Generator<string> {
    yield '[';
    let first = true;
    for (const item of items) {
      if (!first) yield ',';
      yield this.toJSON(item);
      first = false;
    }
    yield ']';
  }
}

// 압축 직렬화
import { gzip, gunzip } from 'zlib';
import { promisify } from 'util';

const gzipAsync = promisify(gzip);
const gunzipAsync = promisify(gunzip);

export class CompressedSerializer {
  // 압축 직렬화
  static async compress<T>(data: T): Promise<Buffer> {
    const json = CryoFacilitySerializer.toJSON(data);
    return gzipAsync(Buffer.from(json, 'utf-8'));
  }

  // 압축 해제 역직렬화
  static async decompress<T>(
    compressed: Buffer,
    schema?: ZodSchema<T>
  ): Promise<T> {
    const buffer = await gunzipAsync(compressed);
    const json = buffer.toString('utf-8');
    return CryoFacilitySerializer.fromJSON(json, schema);
  }
}
```

## 3.9 요약

이 장에서는 극저온 시설 관리의 핵심 데이터 구조를 정의했습니다:

| 스키마 범주 | 주요 구성요소 | 용도 |
|------------|--------------|------|
| 프로젝트 메타데이터 | 시설 정보, 조직, 인증 | 시설 식별 및 기본 정보 |
| 시설 구성 | 구역, 인프라, 운영시간 | 물리적 시설 배치 |
| 장비 인벤토리 | LN2 탱크, 냉동고, 센서 | 장비 관리 및 모니터링 |
| 운영 관리 | SOP, 교육, 문서 | 일상 운영 절차 |
| 안전 관리 | 위험성 평가, 사고 보고 | 안전 관리 체계 |
| 품질 관리 | 감사, CAPA | 품질 보증 활동 |
| 환경 모니터링 | 측정값, 추세 | 환경 조건 추적 |

다음 장에서는 이러한 데이터 구조를 활용하는 API 인터페이스를 살펴봅니다.

---

© 2025 WIA Standards. All rights reserved.
