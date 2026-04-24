# 데이터 형식 및 스키마

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 완전한 Zod 스키마 정의

### 기본 타입 스키마

```typescript
import { z } from 'zod';

/**
 * UUID 스키마
 * RFC 4122 표준 UUID 형식
 */
export const UUIDSchema = z.string().uuid().describe('UUID v4 형식');

/**
 * 날짜시간 스키마
 * ISO 8601 형식 (YYYY-MM-DDTHH:mm:ss.sssZ)
 */
export const DateTimeSchema = z.string().datetime().describe('ISO 8601 날짜시간');

/**
 * 온도 스키마
 * 섭씨 온도 (-273.15°C 이상)
 */
export const TemperatureSchema = z
  .number()
  .min(-273.15)
  .describe('섭씨 온도 (°C)');

/**
 * 백분율 스키마
 * 0-100 사이의 값
 */
export const PercentageSchema = z
  .number()
  .min(0)
  .max(100)
  .describe('백분율 (0-100)');

/**
 * 용량 스키마 (밀리리터)
 */
export const VolumeSchema = z
  .number()
  .positive()
  .describe('용량 (ml)');

/**
 * 바코드 스키마
 * 영숫자 조합, 최소 8자
 */
export const BarcodeSchema = z
  .string()
  .min(8)
  .regex(/^[A-Z0-9]+$/)
  .describe('바코드 (영숫자)');

/**
 * RFID 태그 스키마
 * EPC Gen2 형식
 */
export const RFIDSchema = z
  .string()
  .regex(/^[A-F0-9]{24}$/)
  .describe('RFID 태그 (24자리 16진수)');
```

### 검체 관련 스키마

```typescript
/**
 * 검체 유형 Enum
 */
export const SpecimenTypeEnum = z.enum([
  'SPERM',              // 정자
  'OOCYTE',             // 난자
  'EMBRYO',             // 배아
  'CORD_BLOOD',         // 제대혈
  'TISSUE',             // 조직
  'STEM_CELL',          // 줄기세포
  'OVARIAN_TISSUE',     // 난소조직
  'TESTICULAR_TISSUE',  // 고환조직
  'BONE_MARROW',        // 골수
  'PERIPHERAL_BLOOD',   // 말초혈
]).describe('검체 유형');

/**
 * 검체 상태 Enum
 */
export const SpecimenStatusEnum = z.enum([
  'REGISTERED',    // 등록됨
  'PREPARED',      // 준비됨
  'FREEZING',      // 동결중
  'FROZEN',        // 동결완료
  'THAWING',       // 해동중
  'THAWED',        // 해동완료
  'TRANSFERRED',   // 이식됨
  'DISPOSED',      // 폐기됨
  'QUARANTINE',    // 격리중
]).describe('검체 상태');

/**
 * 보존 방법 Enum
 */
export const PreservationMethodEnum = z.enum([
  'SLOW_FREEZING',    // 완만동결
  'VITRIFICATION',    // 유리화동결
  'RAPID_FREEZING',   // 급속동결
  'CONTROLLED_RATE',  // 프로그램동결
]).describe('보존 방법');

/**
 * 검체 품질 정보 스키마
 */
export const SpecimenQualitySchema = z.object({
  volume: VolumeSchema.describe('검체 용량'),
  concentration: z.number().positive().optional().describe('농도 (cells/ml 또는 million/ml)'),
  motility: PercentageSchema.optional().describe('운동성 (%) - 정자에만 해당'),
  viability: PercentageSchema.describe('생존율 (%)'),
  morphology: z.string().optional().describe('형태학적 평가'),
  pH: z.number().min(0).max(14).optional().describe('pH 값'),
  osmolality: z.number().optional().describe('삼투압 (mOsm/kg)'),
  cellCount: z.number().optional().describe('총 세포 수'),
  nuclearMaturation: z.string().optional().describe('핵 성숙도 - 난자에만 해당'),
}).describe('검체 품질 정보');

/**
 * 보존 정보 스키마
 */
export const PreservationInfoSchema = z.object({
  method: PreservationMethodEnum.describe('보존 방법'),
  methodKr: z.string().describe('보존 방법 한글명'),
  startDate: DateTimeSchema.describe('동결 시작일시'),
  endDate: DateTimeSchema.optional().describe('동결 완료일시'),
  cryoprotectant: z.string().describe('동결보호제 (예: Glycerol 10%)'),
  cryoprotectantConcentration: PercentageSchema.optional().describe('동결보호제 농도'),
  equilibrationTime: z.number().optional().describe('평형 시간 (분)'),
  coolingRate: z.string().describe('냉각 속도 (예: -1°C/min)'),
  finalTemperature: TemperatureSchema.describe('최종 보관 온도'),
  protocolId: UUIDSchema.optional().describe('사용된 프로토콜 ID'),
  deviceId: z.string().optional().describe('동결 장비 ID'),
  performedBy: z.string().describe('시술자 ID'),
}).describe('보존 정보');

/**
 * 저장 위치 스키마
 */
export const StorageLocationSchema = z.object({
  tankId: z.string().describe('탱크 ID (예: TANK-01)'),
  tankName: z.string().optional().describe('탱크 명칭'),
  canisterId: z.string().describe('캐니스터 ID'),
  caneId: z.string().describe('케인 ID'),
  position: z.string().describe('위치 (층-행-열, 예: 1-A-1)'),
  temperature: TemperatureSchema.describe('현재 온도 (°C)'),
  liquidNitrogenLevel: PercentageSchema.optional().describe('액체질소 잔량 (%)'),
  lastRefilled: DateTimeSchema.optional().describe('마지막 충진일시'),
}).describe('저장 위치 정보');

/**
 * 환자 정보 스키마 (암호화됨)
 */
export const PatientInfoSchema = z.object({
  patientId: z.string().describe('환자 ID (암호화)'),
  encryptionMethod: z.string().default('AES-256-GCM').describe('암호화 방식'),
  initials: z.string().optional().describe('이니셜 (예: KIM.JS)'),
  dateOfBirth: z.string().optional().describe('생년월일 (암호화)'),
  gender: z.enum(['M', 'F', 'O']).optional().describe('성별'),
  contactEncrypted: z.string().optional().describe('연락처 (암호화)'),
}).describe('환자 정보 (개인정보 보호)');

/**
 * 완전한 검체 스키마
 */
export const CompleteSpecimenSchema = z.object({
  // 기본 정보
  specimenId: UUIDSchema.describe('검체 고유 ID'),
  patientInfo: PatientInfoSchema.describe('환자 정보'),
  type: SpecimenTypeEnum.describe('검체 유형'),
  typeKr: z.string().describe('검체 유형 한글명'),
  subtype: z.string().optional().describe('세부 유형 (예: 신선배아, 동결배아)'),

  // 수집 정보
  collection: z.object({
    collectionDate: DateTimeSchema.describe('채취일시'),
    collectionMethod: z.string().describe('채취 방법'),
    collectionSite: z.string().optional().describe('채취 부위'),
    collectedBy: z.string().describe('채취자 ID'),
    collectionFacility: z.string().describe('채취 기관'),
    transportMethod: z.string().optional().describe('운송 방법'),
    receivedDate: DateTimeSchema.optional().describe('수령일시'),
  }).describe('수집 정보'),

  // 품질 정보
  quality: SpecimenQualitySchema.describe('품질 정보'),

  // 보존 정보
  preservation: PreservationInfoSchema.describe('보존 정보'),

  // 저장 정보
  storage: StorageLocationSchema.describe('저장 위치'),

  // 상태 정보
  status: SpecimenStatusEnum.describe('현재 상태'),
  statusKr: z.string().describe('상태 한글명'),
  statusHistory: z.array(z.object({
    status: SpecimenStatusEnum,
    statusKr: z.string(),
    changedAt: DateTimeSchema,
    changedBy: z.string(),
    reason: z.string().optional(),
  })).describe('상태 변경 이력'),

  // 추적 정보
  tracking: z.object({
    barcodeId: BarcodeSchema.describe('바코드 ID'),
    rfidTag: RFIDSchema.optional().describe('RFID 태그'),
    qrCode: z.string().optional().describe('QR 코드'),
    chainOfCustody: z.array(z.object({
      timestamp: DateTimeSchema,
      action: z.string(),
      actionKr: z.string(),
      performedBy: z.string(),
      location: z.string(),
    })).describe('관리연속성 기록'),
  }).describe('추적 정보'),

  // 검사 결과
  testResults: z.array(z.object({
    testId: UUIDSchema,
    testType: z.string(),
    testTypeKr: z.string(),
    testDate: DateTimeSchema,
    result: z.string(),
    resultKr: z.string(),
    normalRange: z.string().optional(),
    testedBy: z.string(),
    approved: z.boolean(),
    approvedBy: z.string().optional(),
  })).optional().describe('검사 결과 목록'),

  // 동의서 정보
  consent: z.object({
    consentId: UUIDSchema,
    consentDate: DateTimeSchema,
    consentType: z.array(z.string()).describe('동의 항목'),
    validUntil: DateTimeSchema.optional(),
    withdrawable: z.boolean(),
    witnessId: z.string().optional(),
  }).describe('동의서 정보'),

  // 메타데이터
  metadata: z.object({
    createdAt: DateTimeSchema.describe('생성일시'),
    updatedAt: DateTimeSchema.describe('수정일시'),
    createdBy: z.string().describe('생성자 ID'),
    lastModifiedBy: z.string().describe('최종 수정자 ID'),
    version: z.number().int().positive().describe('버전'),
    notes: z.string().optional().describe('비고'),
    tags: z.array(z.string()).optional().describe('태그'),
    archived: z.boolean().default(false).describe('보관 여부'),
  }).describe('메타데이터'),
}).describe('완전한 검체 스키마');

export type CompleteSpecimen = z.infer<typeof CompleteSpecimenSchema>;
```

### 프로토콜 스키마

```typescript
/**
 * 프로토콜 단계 스키마
 */
export const ProtocolStepSchema = z.object({
  stepNumber: z.number().int().positive().describe('단계 번호'),
  name: z.string().describe('단계 명칭'),
  nameKr: z.string().describe('단계 한글명'),
  duration: z.number().positive().describe('소요 시간 (분)'),
  temperature: TemperatureSchema.optional().describe('목표 온도'),
  coolingRate: z.number().optional().describe('냉각 속도 (°C/min)'),
  warmingRate: z.number().optional().describe('가온 속도 (°C/min)'),
  action: z.string().describe('수행 작업 설명'),
  actionKr: z.string().describe('수행 작업 한글 설명'),
  equipmentRequired: z.array(z.string()).optional().describe('필요 장비'),
  safetyPrecautions: z.array(z.string()).optional().describe('안전 주의사항'),
  qualityChecks: z.array(z.string()).optional().describe('품질 확인사항'),
}).describe('프로토콜 단계');

/**
 * 동결보호제 스키마
 */
export const CryoprotectantSchema = z.object({
  name: z.string().describe('보호제 명칭'),
  nameKr: z.string().describe('보호제 한글명'),
  type: z.enum(['penetrating', 'non-penetrating', 'mixed']).describe('보호제 유형'),
  typeKr: z.string().describe('보호제 유형 한글'),
  components: z.array(z.object({
    component: z.string(),
    componentKr: z.string(),
    concentration: PercentageSchema,
    purpose: z.string(),
  })).describe('구성 성분'),
  concentration: PercentageSchema.describe('최종 농도'),
  equilibrationTime: z.number().positive().describe('평형 시간 (분)'),
  equilibrationTemperature: TemperatureSchema.describe('평형 온도'),
  toxicityLevel: z.enum(['low', 'medium', 'high']).describe('독성 수준'),
  osmolality: z.number().optional().describe('삼투압 (mOsm/kg)'),
  pH: z.number().min(0).max(14).optional().describe('pH'),
}).describe('동결보호제 정보');

/**
 * 품질 관리 기준 스키마
 */
export const QualityControlSchema = z.object({
  preFreezingChecks: z.array(z.object({
    checkItem: z.string(),
    checkItemKr: z.string(),
    criterion: z.string(),
    required: z.boolean(),
  })).describe('동결 전 검사'),

  duringFreezingChecks: z.array(z.object({
    checkItem: z.string(),
    checkItemKr: z.string(),
    checkInterval: z.number().describe('확인 주기 (분)'),
    acceptableRange: z.string(),
  })).describe('동결 중 모니터링'),

  postFreezingChecks: z.array(z.object({
    checkItem: z.string(),
    checkItemKr: z.string(),
    criterion: z.string(),
    required: z.boolean(),
  })).describe('동결 후 검사'),

  expectedOutcomes: z.object({
    minimumViability: PercentageSchema.describe('최소 생존율'),
    targetViability: PercentageSchema.describe('목표 생존율'),
    maximumIceFormation: PercentageSchema.optional().describe('최대 빙결정 형성'),
    postThawRecovery: PercentageSchema.optional().describe('해동 후 회복률'),
  }).describe('예상 결과'),
}).describe('품질 관리 기준');

/**
 * 완전한 동결 프로토콜 스키마
 */
export const CompleteFreezingProtocolSchema = z.object({
  // 기본 정보
  protocolId: UUIDSchema.describe('프로토콜 ID'),
  name: z.string().describe('프로토콜 명칭'),
  nameKr: z.string().describe('프로토콜 한글명'),
  version: z.string().describe('버전 (예: 1.0.0)'),
  status: z.enum(['draft', 'active', 'deprecated']).describe('상태'),

  // 적용 대상
  applicableTo: z.object({
    specimenTypes: z.array(SpecimenTypeEnum).describe('적용 검체 유형'),
    ageRange: z.object({
      min: z.number().optional(),
      max: z.number().optional(),
    }).optional().describe('적용 연령 범위'),
    qualityRequirements: z.string().optional().describe('품질 요구사항'),
  }).describe('적용 대상'),

  // 보존 방법
  method: PreservationMethodEnum.describe('동결 방법'),
  methodKr: z.string().describe('동결 방법 한글명'),

  // 프로토콜 단계
  steps: z.array(ProtocolStepSchema).describe('프로토콜 단계들'),

  // 동결보호제
  cryoprotectant: CryoprotectantSchema.describe('사용 동결보호제'),

  // 품질 관리
  qualityControl: QualityControlSchema.describe('품질 관리 기준'),

  // 장비 요구사항
  equipment: z.array(z.object({
    equipmentType: z.string(),
    equipmentTypeKr: z.string(),
    model: z.string().optional(),
    specifications: z.record(z.string(), z.any()).optional(),
    calibrationRequired: z.boolean(),
    calibrationInterval: z.number().optional().describe('교정 주기 (일)'),
  })).describe('필요 장비'),

  // 안전 정보
  safety: z.object({
    hazards: z.array(z.string()).describe('위험 요소'),
    hazardsKr: z.array(z.string()).describe('위험 요소 한글'),
    ppe: z.array(z.string()).describe('필요 보호구'),
    emergencyProcedures: z.array(z.string()).describe('비상 절차'),
  }).describe('안전 정보'),

  // 성공률 데이터
  performanceData: z.object({
    totalCases: z.number().int().nonnegative().describe('총 시행 건수'),
    successRate: PercentageSchema.describe('성공률'),
    averageViability: PercentageSchema.describe('평균 생존율'),
    complications: z.array(z.object({
      complication: z.string(),
      incidence: PercentageSchema,
    })).optional().describe('합병증'),
  }).optional().describe('성능 데이터'),

  // 승인 정보
  approval: z.object({
    approvedBy: z.string().describe('승인자 ID'),
    approverName: z.string().describe('승인자 명'),
    approverTitle: z.string().describe('승인자 직책'),
    approvedDate: DateTimeSchema.describe('승인일'),
    reviewedBy: z.array(z.string()).optional().describe('검토자 목록'),
    irbApproval: z.boolean().optional().describe('IRB 승인 여부'),
    irbNumber: z.string().optional().describe('IRB 승인 번호'),
  }).describe('승인 정보'),

  // 참고 문헌
  references: z.array(z.object({
    referenceId: z.string(),
    citation: z.string(),
    doi: z.string().optional(),
    url: z.string().url().optional(),
  })).optional().describe('참고 문헌'),

  // 메타데이터
  metadata: z.object({
    createdAt: DateTimeSchema,
    updatedAt: DateTimeSchema,
    createdBy: z.string(),
    lastModifiedBy: z.string(),
    revisionHistory: z.array(z.object({
      version: z.string(),
      date: DateTimeSchema,
      changes: z.string(),
      modifiedBy: z.string(),
    })).describe('개정 이력'),
  }).describe('메타데이터'),
}).describe('완전한 동결 프로토콜');

export type CompleteFreezingProtocol = z.infer<typeof CompleteFreezingProtocolSchema>;
```

### 해동 프로토콜 스키마

```typescript
/**
 * 희석 단계 스키마
 */
export const DilutionStepSchema = z.object({
  stepNumber: z.number().int().positive().describe('단계 번호'),
  solution: z.string().describe('희석액'),
  solutionKr: z.string().describe('희석액 한글명'),
  volume: VolumeSchema.describe('첨가 용량 (ml)'),
  additionRate: z.number().optional().describe('첨가 속도 (ml/min)'),
  duration: z.number().positive().describe('대기 시간 (분)'),
  temperature: TemperatureSchema.optional().describe('온도'),
  mixingMethod: z.string().optional().describe('혼합 방법'),
}).describe('희석 단계');

/**
 * 품질 평가 스키마
 */
export const QualityAssessmentSchema = z.object({
  viabilityTest: z.object({
    required: z.boolean().describe('필수 여부'),
    method: z.string().describe('검사 방법'),
    methodKr: z.string().describe('검사 방법 한글'),
    minimumViability: PercentageSchema.describe('최소 생존율'),
    testTiming: z.string().describe('검사 시점'),
  }).describe('생존율 검사'),

  motilityTest: z.object({
    required: z.boolean().describe('필수 여부'),
    method: z.string().optional().describe('검사 방법'),
    minimumMotility: PercentageSchema.optional().describe('최소 운동성'),
    gradingSystem: z.string().optional().describe('등급 체계'),
  }).optional().describe('운동성 검사 - 정자용'),

  morphologyTest: z.object({
    required: z.boolean().describe('필수 여부'),
    method: z.string().optional().describe('검사 방법'),
    criteria: z.string().optional().describe('평가 기준'),
  }).optional().describe('형태 검사'),

  functionalTests: z.array(z.object({
    testName: z.string(),
    testNameKr: z.string(),
    method: z.string(),
    passCriteria: z.string(),
  })).optional().describe('기능 검사'),
}).describe('품질 평가');

/**
 * 완전한 해동 프로토콜 스키마
 */
export const CompleteThawingProtocolSchema = z.object({
  // 기본 정보
  protocolId: UUIDSchema.describe('프로토콜 ID'),
  name: z.string().describe('프로토콜 명칭'),
  nameKr: z.string().describe('프로토콜 한글명'),
  version: z.string().describe('버전'),
  status: z.enum(['draft', 'active', 'deprecated']).describe('상태'),

  // 적용 대상
  applicableTo: z.object({
    specimenTypes: z.array(SpecimenTypeEnum).describe('적용 검체 유형'),
    frozenWith: PreservationMethodEnum.describe('동결 방법'),
    storageTemperature: TemperatureSchema.describe('보관 온도'),
  }).describe('적용 대상'),

  // 해동 단계
  thawingSteps: z.array(ProtocolStepSchema).describe('해동 단계'),

  // 희석 프로토콜
  dilution: z.object({
    required: z.boolean().describe('희석 필요 여부'),
    steps: z.array(DilutionStepSchema).optional().describe('희석 단계'),
    totalDilutionFactor: z.number().optional().describe('총 희석 배수'),
  }).describe('희석 프로토콜'),

  // 품질 평가
  qualityAssessment: QualityAssessmentSchema.describe('품질 평가'),

  // 사용 시간 제한
  timeConstraints: z.object({
    maxThawToUse: z.number().describe('해동 후 사용까지 최대 시간 (분)'),
    optimalThawToUse: z.number().describe('해동 후 최적 사용 시간 (분)'),
    postThawStorage: z.object({
      allowed: z.boolean(),
      maxDuration: z.number().optional().describe('최대 보관 시간 (시간)'),
      temperature: TemperatureSchema.optional().describe('보관 온도'),
    }).optional().describe('해동 후 보관'),
  }).describe('시간 제약'),

  // 성공률 데이터
  performanceData: z.object({
    totalCases: z.number().int().nonnegative().describe('총 시행 건수'),
    successRate: PercentageSchema.describe('성공률'),
    averagePostThawViability: PercentageSchema.describe('평균 해동 후 생존율'),
    recoveryRate: PercentageSchema.describe('회복률'),
  }).optional().describe('성능 데이터'),

  // 승인 정보
  approval: z.object({
    approvedBy: z.string().describe('승인자 ID'),
    approverName: z.string().describe('승인자 명'),
    approvedDate: DateTimeSchema.describe('승인일'),
  }).describe('승인 정보'),

  // 메타데이터
  metadata: z.object({
    createdAt: DateTimeSchema,
    updatedAt: DateTimeSchema,
    createdBy: z.string(),
    lastModifiedBy: z.string(),
  }).describe('메타데이터'),
}).describe('완전한 해동 프로토콜');

export type CompleteThawingProtocol = z.infer<typeof CompleteThawingProtocolSchema>;
```

### 모니터링 및 경보 스키마

```typescript
/**
 * 센서 데이터 스키마
 */
export const SensorDataSchema = z.object({
  sensorId: z.string().describe('센서 ID'),
  sensorType: z.enum([
    'temperature',
    'liquid_nitrogen_level',
    'humidity',
    'pressure',
    'door_status',
  ]).describe('센서 유형'),
  sensorTypeKr: z.string().describe('센서 유형 한글'),
  location: z.string().describe('설치 위치'),
  reading: z.number().describe('측정값'),
  unit: z.string().describe('단위'),
  timestamp: DateTimeSchema.describe('측정 시각'),
  status: z.enum(['normal', 'warning', 'critical', 'fault']).describe('상태'),
  statusKr: z.string().describe('상태 한글'),
}).describe('센서 데이터');

/**
 * 경보 스키마
 */
export const AlarmSchema = z.object({
  alarmId: UUIDSchema.describe('경보 ID'),
  alarmType: z.enum([
    'temperature_high',
    'temperature_low',
    'liquid_nitrogen_low',
    'power_failure',
    'door_open',
    'equipment_failure',
    'unauthorized_access',
  ]).describe('경보 유형'),
  alarmTypeKr: z.string().describe('경보 유형 한글'),
  severity: z.enum(['info', 'warning', 'critical', 'emergency']).describe('심각도'),
  severityKr: z.string().describe('심각도 한글'),
  triggeredAt: DateTimeSchema.describe('발생 시각'),
  location: z.string().describe('발생 위치'),
  sensorId: z.string().optional().describe('관련 센서 ID'),
  message: z.string().describe('경보 메시지'),
  messageKr: z.string().describe('경보 메시지 한글'),
  notified: z.array(z.string()).describe('알림 수신자 목록'),
  acknowledged: z.boolean().describe('확인 여부'),
  acknowledgedBy: z.string().optional().describe('확인자 ID'),
  acknowledgedAt: DateTimeSchema.optional().describe('확인 시각'),
  resolved: z.boolean().describe('해결 여부'),
  resolvedBy: z.string().optional().describe('해결자 ID'),
  resolvedAt: DateTimeSchema.optional().describe('해결 시각'),
  actionsTaken: z.array(z.string()).optional().describe('조치 사항'),
}).describe('경보 정보');

/**
 * 탱크 모니터링 데이터 스키마
 */
export const TankMonitoringSchema = z.object({
  tankId: z.string().describe('탱크 ID'),
  tankName: z.string().describe('탱크 명칭'),
  location: z.string().describe('위치'),
  capacity: z.number().describe('용량 (검체 수)'),
  currentOccupancy: z.number().describe('현재 사용량'),
  temperature: TemperatureSchema.describe('현재 온도'),
  liquidNitrogenLevel: PercentageSchema.describe('액체질소 잔량'),
  lastRefilled: DateTimeSchema.describe('마지막 충진일시'),
  nextRefillDue: DateTimeSchema.describe('다음 충진 예정일시'),
  sensors: z.array(SensorDataSchema).describe('센서 데이터'),
  alarms: z.array(AlarmSchema).optional().describe('활성 경보'),
  maintenanceHistory: z.array(z.object({
    maintenanceId: UUIDSchema,
    date: DateTimeSchema,
    type: z.string(),
    typeKr: z.string(),
    performedBy: z.string(),
    notes: z.string().optional(),
  })).optional().describe('유지보수 이력'),
  status: z.enum(['operational', 'warning', 'critical', 'offline']).describe('탱크 상태'),
  statusKr: z.string().describe('탱크 상태 한글'),
}).describe('탱크 모니터링 데이터');
```

### 감사 로그 스키마

```typescript
/**
 * 이벤트 유형 Enum
 */
export const EventTypeEnum = z.enum([
  'SPECIMEN_CREATED',
  'SPECIMEN_UPDATED',
  'SPECIMEN_FROZEN',
  'SPECIMEN_THAWED',
  'SPECIMEN_TRANSFERRED',
  'SPECIMEN_DISPOSED',
  'LOCATION_CHANGED',
  'ACCESS_GRANTED',
  'ACCESS_DENIED',
  'PROTOCOL_EXECUTED',
  'ALARM_TRIGGERED',
  'ALARM_ACKNOWLEDGED',
  'SYSTEM_LOGIN',
  'SYSTEM_LOGOUT',
  'BACKUP_CREATED',
  'CONFIGURATION_CHANGED',
]).describe('이벤트 유형');

/**
 * 감사 로그 스키마
 */
export const AuditLogSchema = z.object({
  logId: UUIDSchema.describe('로그 ID'),
  timestamp: DateTimeSchema.describe('발생 시각'),
  eventType: EventTypeEnum.describe('이벤트 유형'),
  eventTypeKr: z.string().describe('이벤트 유형 한글'),
  userId: z.string().describe('사용자 ID'),
  userName: z.string().optional().describe('사용자 이름'),
  userRole: z.string().optional().describe('사용자 역할'),
  ipAddress: z.string().ip().optional().describe('IP 주소'),
  action: z.string().describe('수행 작업'),
  actionKr: z.string().describe('수행 작업 한글'),
  resourceType: z.string().describe('리소스 유형'),
  resourceId: z.string().optional().describe('리소스 ID'),
  changes: z.object({
    before: z.record(z.string(), z.any()).optional().describe('변경 전'),
    after: z.record(z.string(), z.any()).optional().describe('변경 후'),
  }).optional().describe('변경 내용'),
  success: z.boolean().describe('성공 여부'),
  errorMessage: z.string().optional().describe('오류 메시지'),
  sessionId: z.string().optional().describe('세션 ID'),
  metadata: z.record(z.string(), z.any()).optional().describe('추가 메타데이터'),
}).describe('감사 로그');

/**
 * 체인 오브 커스터디 (관리연속성) 스키마
 */
export const ChainOfCustodySchema = z.object({
  custodyId: UUIDSchema.describe('관리연속성 ID'),
  specimenId: UUIDSchema.describe('검체 ID'),
  events: z.array(z.object({
    eventId: UUIDSchema,
    timestamp: DateTimeSchema.describe('시각'),
    eventType: z.string().describe('이벤트 유형'),
    eventTypeKr: z.string().describe('이벤트 유형 한글'),
    custodian: z.string().describe('관리자 ID'),
    custodianName: z.string().describe('관리자 이름'),
    location: z.string().describe('위치'),
    action: z.string().describe('수행 작업'),
    actionKr: z.string().describe('수행 작업 한글'),
    witness: z.string().optional().describe('증인 ID'),
    digitalSignature: z.string().optional().describe('전자 서명'),
    notes: z.string().optional().describe('비고'),
  })).describe('이벤트 목록'),
  verified: z.boolean().describe('검증 여부'),
  verifiedBy: z.string().optional().describe('검증자 ID'),
  verifiedAt: DateTimeSchema.optional().describe('검증 시각'),
}).describe('관리연속성 기록');
```

### 보고서 스키마

```typescript
/**
 * 검체 보고서 스키마
 */
export const SpecimenReportSchema = z.object({
  reportId: UUIDSchema.describe('보고서 ID'),
  reportType: z.enum([
    'freezing',
    'thawing',
    'quality',
    'storage',
    'transfer',
  ]).describe('보고서 유형'),
  reportTypeKr: z.string().describe('보고서 유형 한글'),
  specimenId: UUIDSchema.describe('검체 ID'),
  generatedAt: DateTimeSchema.describe('생성일시'),
  generatedBy: z.string().describe('생성자 ID'),

  // 검체 정보
  specimenInfo: z.object({
    type: SpecimenTypeEnum,
    typeKr: z.string(),
    collectionDate: DateTimeSchema,
    patientInitials: z.string(),
  }).describe('검체 정보'),

  // 프로토콜 정보
  protocolInfo: z.object({
    protocolId: UUIDSchema,
    protocolName: z.string(),
    protocolNameKr: z.string(),
    version: z.string(),
  }).optional().describe('사용 프로토콜'),

  // 결과
  results: z.object({
    preViability: PercentageSchema.optional().describe('동결/해동 전 생존율'),
    postViability: PercentageSchema.optional().describe('동결/해동 후 생존율'),
    recoveryRate: PercentageSchema.optional().describe('회복률'),
    qualityGrade: z.string().optional().describe('품질 등급'),
    complications: z.array(z.string()).optional().describe('합병증'),
  }).describe('결과'),

  // 담당자
  personnel: z.array(z.object({
    role: z.string(),
    roleKr: z.string(),
    name: z.string(),
    qualification: z.string().optional(),
    signature: z.string().optional().describe('서명'),
  })).describe('담당자'),

  // 승인
  approval: z.object({
    approved: z.boolean(),
    approvedBy: z.string().optional(),
    approvedAt: DateTimeSchema.optional(),
    comments: z.string().optional(),
  }).optional().describe('승인 정보'),

  // 문서
  attachments: z.array(z.object({
    fileName: z.string(),
    fileType: z.string(),
    fileSize: z.number(),
    uploadedAt: DateTimeSchema,
    url: z.string().url().optional(),
  })).optional().describe('첨부 파일'),
}).describe('검체 보고서');
```

## 데이터 검증 예제

```typescript
/**
 * 스키마 검증 유틸리티
 */
export class SchemaValidator {
  /**
   * 검체 데이터 검증
   */
  static validateSpecimen(data: unknown): {
    valid: boolean;
    errors?: z.ZodError;
    data?: CompleteSpecimen;
  } {
    try {
      const validated = CompleteSpecimenSchema.parse(data);
      return {
        valid: true,
        data: validated,
      };
    } catch (error) {
      if (error instanceof z.ZodError) {
        return {
          valid: false,
          errors: error,
        };
      }
      throw error;
    }
  }

  /**
   * 동결 프로토콜 검증
   */
  static validateFreezingProtocol(data: unknown): {
    valid: boolean;
    errors?: z.ZodError;
    data?: CompleteFreezingProtocol;
  } {
    try {
      const validated = CompleteFreezingProtocolSchema.parse(data);
      return {
        valid: true,
        data: validated,
      };
    } catch (error) {
      if (error instanceof z.ZodError) {
        return {
          valid: false,
          errors: error,
        };
      }
      throw error;
    }
  }

  /**
   * 에러 메시지 한글화
   */
  static formatErrorsKorean(errors: z.ZodError): string[] {
    return errors.errors.map(err => {
      const path = err.path.join(' > ');
      return `[${path}] ${this.translateError(err)}`;
    });
  }

  private static translateError(err: z.ZodIssue): string {
    switch (err.code) {
      case 'invalid_type':
        return `잘못된 데이터 형식: ${err.expected} 필요, ${err.received} 제공됨`;
      case 'too_small':
        return `값이 너무 작음: 최소 ${err.minimum} 필요`;
      case 'too_big':
        return `값이 너무 큼: 최대 ${err.maximum} 허용`;
      case 'invalid_string':
        return `잘못된 문자열 형식: ${err.validation}`;
      case 'invalid_enum_value':
        return `허용되지 않는 값: ${err.options.join(', ')} 중 하나여야 함`;
      default:
        return err.message;
    }
  }
}
```

---

**문서 버전**: 1.0
**최종 수정**: 2025-01-11
**작성자**: WIA Standards Committee

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
