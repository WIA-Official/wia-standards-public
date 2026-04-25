# 03. 데이터 형식 및 Zod 스키마
## Data Formats and Zod Schemas

**Version**: 1.0.0
**Last Updated**: 2026-01-11

---

## 목차

1. [Zod 스키마 개요](#zod-스키마-개요)
2. [센서 데이터 스키마](#센서-데이터-스키마)
3. [시설 및 탱크 스키마](#시설-및-탱크-스키마)
4. [알림 및 이벤트 스키마](#알림-및-이벤트-스키마)
5. [사용자 및 권한 스키마](#사용자-및-권한-스키마)
6. [감사 로그 스키마](#감사-로그-스키마)
7. [통합 검증 시스템](#통합-검증-시스템)

---

## Zod 스키마 개요

### Zod 라이브러리 설정

```typescript
/**
 * WIA Cryo Monitoring - Zod 스키마
 *
 * 모든 데이터는 Zod를 통해 런타임 검증
 * TypeScript 타입은 자동으로 추론됨
 *
 * @package zod
 * @version 3.22.4
 */

import { z } from "zod";

/**
 * 한국어 필드 유틸리티
 * 모든 enum과 객체에 한국어 설명 추가
 */
const withKorean = <T extends z.ZodTypeAny>(
  schema: T,
  koreanField: string
) => {
  return schema.describe(koreanField);
};

/**
 * 날짜/시간 스키마
 * ISO 8601 형식 또는 Date 객체
 */
const DateTimeSchema = z.union([
  z.string().datetime({ message: "ISO 8601 형식이어야 합니다" }),
  z.date()
]).transform((val) => {
  return typeof val === "string" ? new Date(val) : val;
});

/**
 * UUID 스키마
 * RFC 4122 표준 UUID
 */
const UUIDSchema = z.string().uuid({
  message: "유효한 UUID 형식이어야 합니다"
});

/**
 * 좌표 스키마
 * GPS 좌표 (위도/경도)
 */
const CoordinatesSchema = z.object({
  lat: z.number().min(-90).max(90).describe("위도"),
  lng: z.number().min(-180).max(180).describe("경도")
});

/**
 * 한국 전화번호 스키마
 */
const KoreanPhoneSchema = z.string().regex(
  /^(\+82|0)(1[0-9]|2|3[1-3]|4[1-4]|5[1-5]|6[1-4])-?[0-9]{3,4}-?[0-9]{4}$/,
  { message: "유효한 한국 전화번호 형식이어야 합니다" }
);

/**
 * 이메일 스키마 (한국 도메인 포함)
 */
const EmailSchema = z.string().email({
  message: "유효한 이메일 주소여야 합니다"
});
```

---

## 센서 데이터 스키마

### 센서 유형 및 상태

```typescript
/**
 * 센서 유형 Enum
 */
const SensorTypeSchema = z.enum([
  "temperature",          // 온도 센서
  "level",               // 레벨 센서
  "pressure",            // 압력 센서
  "humidity",            // 습도 센서
  "co2",                 // CO2 센서
  "o2",                  // 산소 센서
  "vibration",           // 진동 센서
  "door",                // 도어 센서
  "power"                // 전원 센서
]).describe("센서 유형");

// TypeScript 타입 자동 추론
type SensorType = z.infer<typeof SensorTypeSchema>;

/**
 * 센서 유형 한국어 매핑
 */
const SensorTypeKr: Record<SensorType, string> = {
  temperature: "온도 센서",
  level: "레벨 센서",
  pressure: "압력 센서",
  humidity: "습도 센서",
  co2: "이산화탄소 센서",
  o2: "산소 센서",
  vibration: "진동 센서",
  door: "도어 개폐 센서",
  power: "전원 상태 센서"
};

/**
 * 센서 상태 Enum
 */
const SensorStatusSchema = z.enum([
  "active",              // 정상 작동
  "calibrating",         // 교정 중
  "maintenance",         // 점검 중
  "error",               // 오류
  "offline",             // 오프라인
  "decommissioned"       // 폐기
]).describe("센서 상태");

type SensorStatus = z.infer<typeof SensorStatusSchema>;

const SensorStatusKr: Record<SensorStatus, string> = {
  active: "정상 작동",
  calibrating: "교정 중",
  maintenance: "점검 중",
  error: "오류 발생",
  offline: "연결 끊김",
  decommissioned: "사용 중지"
};

/**
 * 측정 단위 Enum
 */
const MeasurementUnitSchema = z.enum([
  // 온도
  "celsius",
  "kelvin",
  "fahrenheit",
  // 압력
  "psi",
  "bar",
  "kpa",
  "atm",
  // 레벨
  "percent",
  "liters",
  "meters",
  // 기타
  "ppm",                 // Parts per million
  "rh",                  // Relative humidity
  "boolean"              // On/Off
]).describe("측정 단위");

type MeasurementUnit = z.infer<typeof MeasurementUnitSchema>;

const MeasurementUnitKr: Record<MeasurementUnit, string> = {
  celsius: "섭씨 (°C)",
  kelvin: "켈빈 (K)",
  fahrenheit: "화씨 (°F)",
  psi: "PSI (파운드/제곱인치)",
  bar: "바 (bar)",
  kpa: "킬로파스칼 (kPa)",
  atm: "기압 (atm)",
  percent: "퍼센트 (%)",
  liters: "리터 (L)",
  meters: "미터 (m)",
  ppm: "백만분율 (ppm)",
  rh: "상대습도 (%RH)",
  boolean: "켜짐/꺼짐"
};

/**
 * 센서 설정 스키마
 */
const SensorConfigSchema = z.object({
  // 기본 정보
  sensorId: UUIDSchema.describe("센서 고유 ID"),
  name: z.string().min(1).max(100).describe("센서 이름"),
  nameKr: z.string().min(1).max(100).describe("센서 이름 (한국어)"),
  type: SensorTypeSchema,
  typeKr: z.string().describe("센서 유형 (한국어)"),

  // 위치 정보
  location: z.object({
    facilityId: UUIDSchema.describe("시설 ID"),
    tankId: UUIDSchema.optional().describe("탱크 ID (선택)"),
    zone: z.string().describe("구역"),
    position: z.object({
      x: z.number().describe("X 좌표"),
      y: z.number().describe("Y 좌표"),
      z: z.number().describe("Z 좌표 (높이)")
    }),
    coordinates: CoordinatesSchema.optional().describe("GPS 좌표"),
    locationKr: z.string().describe("위치 설명 (한국어)")
  }),

  // 측정 사양
  specification: z.object({
    unit: MeasurementUnitSchema,
    unitKr: z.string().describe("단위 (한국어)"),
    minValue: z.number().describe("최소값"),
    maxValue: z.number().describe("최대값"),
    accuracy: z.number().positive().describe("정확도"),
    resolution: z.number().positive().describe("해상도"),
    samplingRate: z.number().positive().describe("샘플링 레이트 (Hz)"),
    specKr: z.string().describe("사양 설명 (한국어)")
  }),

  // 교정 정보
  calibration: z.object({
    lastCalibration: DateTimeSchema.describe("마지막 교정 날짜"),
    nextCalibration: DateTimeSchema.describe("다음 교정 예정일"),
    calibrationInterval: z.number().positive().describe("교정 주기 (일)"),
    calibratedBy: z.string().describe("교정 담당자"),
    certificate: z.string().url().optional().describe("교정 인증서 URL"),
    calibrationKr: z.string().describe("교정 정보 (한국어)")
  }),

  // 제조사 정보
  manufacturer: z.object({
    name: z.string().describe("제조사명"),
    nameKr: z.string().describe("제조사명 (한국어)"),
    model: z.string().describe("모델명"),
    serialNumber: z.string().describe("시리얼 번호"),
    manufactureDate: DateTimeSchema.describe("제조 날짜"),
    warrantyExpiry: DateTimeSchema.describe("보증 만료일"),
    mfgKr: z.string().describe("제조사 정보 (한국어)")
  }),

  // 통신 설정
  communication: z.object({
    protocol: z.enum([
      "modbus-rtu",
      "modbus-tcp",
      "mqtt",
      "http",
      "4-20ma",
      "hart",
      "io-link",
      "lorawan",
      "nb-iot"
    ]).describe("통신 프로토콜"),
    address: z.string().describe("주소 (IP, Modbus 주소 등)"),
    port: z.number().int().min(1).max(65535).optional().describe("포트"),
    updateInterval: z.number().positive().describe("업데이트 간격 (초)"),
    timeout: z.number().positive().describe("타임아웃 (초)"),
    commKr: z.string().describe("통신 설정 (한국어)")
  }),

  // 알림 임계값
  thresholds: z.object({
    warning: z.object({
      upper: z.number().describe("경고 상한선"),
      lower: z.number().describe("경고 하한선"),
      warningKr: z.string().describe("경고 임계값 (한국어)")
    }),
    critical: z.object({
      upper: z.number().describe("위험 상한선"),
      lower: z.number().describe("위험 하한선"),
      criticalKr: z.string().describe("위험 임계값 (한국어)")
    }),
    emergency: z.object({
      upper: z.number().describe("긴급 상한선"),
      lower: z.number().describe("긴급 하한선"),
      emergencyKr: z.string().describe("긴급 임계값 (한국어)")
    })
  }),

  // 상태
  status: SensorStatusSchema,
  statusKr: z.string().describe("상태 (한국어)"),

  // 메타데이터
  metadata: z.object({
    installDate: DateTimeSchema.describe("설치 날짜"),
    installedBy: z.string().describe("설치 담당자"),
    notes: z.string().optional().describe("비고"),
    tags: z.array(z.string()).default([]).describe("태그"),
    metadataKr: z.string().describe("메타데이터 (한국어)")
  }),

  // 타임스탬프
  createdAt: DateTimeSchema.describe("생성 시간"),
  updatedAt: DateTimeSchema.describe("수정 시간")
});

type SensorConfig = z.infer<typeof SensorConfigSchema>;

/**
 * 센서 데이터 읽기 스키마
 */
const SensorReadingSchema = z.object({
  // 기본 정보
  readingId: UUIDSchema.describe("읽기 ID"),
  sensorId: UUIDSchema.describe("센서 ID"),

  // 측정값
  value: z.number().describe("측정값"),
  unit: MeasurementUnitSchema,
  unitKr: z.string().describe("단위 (한국어)"),

  // 품질 지표
  quality: z.enum([
    "good",              // 좋음 (정확도 범위 내)
    "fair",              // 보통 (정확도 범위 약간 벗어남)
    "poor",              // 나쁨 (신뢰할 수 없음)
    "uncertain"          // 불확실 (센서 오류 가능)
  ]).describe("데이터 품질"),
  qualityKr: z.string().describe("품질 (한국어)"),

  // 신호 강도 (무선 센서)
  signalStrength: z.number().min(0).max(100).optional().describe("신호 강도 (%)"),

  // 배터리 (배터리 구동 센서)
  batteryLevel: z.number().min(0).max(100).optional().describe("배터리 (%)"),

  // 플래그
  flags: z.object({
    calibrated: z.boolean().describe("교정됨"),
    validated: z.boolean().describe("검증됨"),
    anomaly: z.boolean().describe("이상 감지"),
    flagsKr: z.string().describe("플래그 (한국어)")
  }),

  // 타임스탬프
  timestamp: DateTimeSchema.describe("측정 시간"),
  receivedAt: DateTimeSchema.describe("수신 시간"),

  // 메타데이터
  metadata: z.record(z.unknown()).optional().describe("추가 메타데이터")
});

type SensorReading = z.infer<typeof SensorReadingSchema>;

const SensorReadingQualityKr = {
  good: "좋음 (정확도 범위 내)",
  fair: "보통 (정확도 약간 벗어남)",
  poor: "나쁨 (신뢰할 수 없음)",
  uncertain: "불확실 (센서 오류 가능)"
};

/**
 * 센서 읽기 배치 스키마
 * 여러 센서의 데이터를 한번에 전송
 */
const SensorReadingBatchSchema = z.object({
  batchId: UUIDSchema.describe("배치 ID"),
  readings: z.array(SensorReadingSchema).min(1).describe("센서 읽기 배열"),
  totalCount: z.number().int().positive().describe("총 개수"),
  timestamp: DateTimeSchema.describe("배치 타임스탬프"),
  checksum: z.string().optional().describe("체크섬 (데이터 무결성)")
});

type SensorReadingBatch = z.infer<typeof SensorReadingBatchSchema>;
```

### 센서 통계 스키마

```typescript
/**
 * 센서 통계 스키마
 * 시간 범위 내 센서 데이터 통계
 */
const SensorStatisticsSchema = z.object({
  sensorId: UUIDSchema.describe("센서 ID"),

  // 시간 범위
  timeRange: z.object({
    start: DateTimeSchema.describe("시작 시간"),
    end: DateTimeSchema.describe("종료 시간"),
    duration: z.number().positive().describe("기간 (초)"),
    rangeKr: z.string().describe("시간 범위 (한국어)")
  }),

  // 기본 통계
  statistics: z.object({
    count: z.number().int().nonnegative().describe("데이터 개수"),
    min: z.number().describe("최소값"),
    max: z.number().describe("최대값"),
    mean: z.number().describe("평균"),
    median: z.number().describe("중앙값"),
    mode: z.number().optional().describe("최빈값"),
    stdDev: z.number().nonnegative().describe("표준편차"),
    variance: z.number().nonnegative().describe("분산"),
    statsKr: z.string().describe("통계 (한국어)")
  }),

  // 백분위수
  percentiles: z.object({
    p5: z.number().describe("5th 백분위수"),
    p25: z.number().describe("25th 백분위수 (1사분위)"),
    p50: z.number().describe("50th 백분위수 (중앙값)"),
    p75: z.number().describe("75th 백분위수 (3사분위)"),
    p95: z.number().describe("95th 백분위수"),
    p99: z.number().describe("99th 백분위수"),
    percentilesKr: z.string().describe("백분위수 (한국어)")
  }),

  // 추세 분석
  trend: z.object({
    direction: z.enum(["increasing", "stable", "decreasing"]).describe("방향"),
    directionKr: z.string().describe("방향 (한국어)"),
    slope: z.number().describe("기울기 (단위/시간)"),
    correlation: z.number().min(-1).max(1).describe("상관계수"),
    trendKr: z.string().describe("추세 (한국어)")
  }),

  // 이상치
  outliers: z.object({
    count: z.number().int().nonnegative().describe("이상치 개수"),
    percentage: z.number().min(0).max(100).describe("이상치 비율 (%)"),
    values: z.array(z.number()).describe("이상치 값들"),
    outliersKr: z.string().describe("이상치 (한국어)")
  }),

  // 알림 발생 횟수
  alerts: z.object({
    warning: z.number().int().nonnegative().describe("경고 횟수"),
    critical: z.number().int().nonnegative().describe("위험 횟수"),
    emergency: z.number().int().nonnegative().describe("긴급 횟수"),
    total: z.number().int().nonnegative().describe("총 알림 횟수"),
    alertsKr: z.string().describe("알림 (한국어)")
  }),

  // 데이터 품질
  quality: z.object({
    good: z.number().min(0).max(100).describe("좋음 (%)"),
    fair: z.number().min(0).max(100).describe("보통 (%)"),
    poor: z.number().min(0).max(100).describe("나쁨 (%)"),
    uncertain: z.number().min(0).max(100).describe("불확실 (%)"),
    qualityKr: z.string().describe("품질 분포 (한국어)")
  }),

  // 타임스탬프
  calculatedAt: DateTimeSchema.describe("계산 시간")
});

type SensorStatistics = z.infer<typeof SensorStatisticsSchema>;

const TrendDirectionKr = {
  increasing: "상승 추세",
  stable: "안정",
  decreasing: "하강 추세"
};
```

---

## 시설 및 탱크 스키마

### 시설 스키마

```typescript
/**
 * 시설 유형 Enum
 */
const FacilityTypeSchema = z.enum([
  "cord_blood_bank",     // 제대혈 은행
  "fertility_clinic",    // 난임 클리닉
  "biobank",             // 바이오뱅크
  "hospital",            // 병원
  "research_institute",  // 연구소
  "pharmaceutical",      // 제약회사
  "logistics"            // 물류 센터
]).describe("시설 유형");

type FacilityType = z.infer<typeof FacilityTypeSchema>;

const FacilityTypeKr: Record<FacilityType, string> = {
  cord_blood_bank: "제대혈 은행",
  fertility_clinic: "난임 클리닉",
  biobank: "바이오뱅크",
  hospital: "병원",
  research_institute: "연구소",
  pharmaceutical: "제약회사",
  logistics: "콜드체인 물류 센터"
};

/**
 * 시설 스키마
 */
const FacilitySchema = z.object({
  // 기본 정보
  facilityId: UUIDSchema.describe("시설 ID"),
  name: z.string().min(1).max(200).describe("시설명"),
  nameKr: z.string().min(1).max(200).describe("시설명 (한국어)"),
  type: FacilityTypeSchema,
  typeKr: z.string().describe("시설 유형 (한국어)"),

  // 연락처
  contact: z.object({
    address: z.string().describe("주소"),
    addressKr: z.string().describe("주소 (한국어)"),
    coordinates: CoordinatesSchema.describe("GPS 좌표"),
    phone: KoreanPhoneSchema.describe("전화번호"),
    fax: KoreanPhoneSchema.optional().describe("팩스"),
    email: EmailSchema.describe("이메일"),
    website: z.string().url().optional().describe("웹사이트"),
    contactKr: z.string().describe("연락처 (한국어)")
  }),

  // 운영 정보
  operations: z.object({
    operatingHours: z.object({
      weekday: z.string().describe("평일 운영 시간"),
      weekend: z.string().describe("주말 운영 시간"),
      holiday: z.string().describe("공휴일 운영 시간"),
      hoursKr: z.string().describe("운영 시간 (한국어)")
    }),
    emergencyContact: z.object({
      name: z.string().describe("비상 연락 담당자"),
      phone: KoreanPhoneSchema.describe("비상 연락처"),
      email: EmailSchema.describe("이메일"),
      emergencyKr: z.string().describe("비상 연락처 (한국어)")
    }),
    capacity: z.object({
      tanks: z.number().int().nonnegative().describe("탱크 수"),
      samples: z.number().int().nonnegative().describe("샘플 보관 용량"),
      currentSamples: z.number().int().nonnegative().describe("현재 샘플 수"),
      utilizationRate: z.number().min(0).max(100).describe("사용률 (%)"),
      capacityKr: z.string().describe("용량 정보 (한국어)")
    }),
    opsKr: z.string().describe("운영 정보 (한국어)")
  }),

  // 인증 및 규제
  certifications: z.object({
    licenses: z.array(z.object({
      type: z.string().describe("인증 유형"),
      typeKr: z.string().describe("인증 유형 (한국어)"),
      number: z.string().describe("인증 번호"),
      issuer: z.string().describe("발급 기관"),
      issuerKr: z.string().describe("발급 기관 (한국어)"),
      issueDate: DateTimeSchema.describe("발급일"),
      expiryDate: DateTimeSchema.describe("만료일"),
      documentUrl: z.string().url().optional().describe("인증서 URL")
    })).describe("인증 목록"),
    accreditations: z.array(z.string()).describe("인가"),
    accreditationsKr: z.array(z.string()).describe("인가 (한국어)"),
    certKr: z.string().describe("인증 정보 (한국어)")
  }),

  // WIA 표준 채택
  wiaCompliance: z.object({
    adopted: z.boolean().describe("WIA 표준 채택 여부"),
    version: z.string().describe("WIA 표준 버전"),
    implementationDate: DateTimeSchema.optional().describe("구현 날짜"),
    certificationDate: DateTimeSchema.optional().describe("인증 날짜"),
    complianceKr: z.string().describe("WIA 준수 (한국어)")
  }),

  // 메타데이터
  metadata: z.object({
    established: DateTimeSchema.describe("설립일"),
    registrationNumber: z.string().describe("사업자 등록번호"),
    director: z.string().describe("대표자"),
    employees: z.number().int().nonnegative().describe("직원 수"),
    notes: z.string().optional().describe("비고"),
    tags: z.array(z.string()).default([]).describe("태그"),
    metadataKr: z.string().describe("메타데이터 (한국어)")
  }),

  // 타임스탬프
  createdAt: DateTimeSchema.describe("생성 시간"),
  updatedAt: DateTimeSchema.describe("수정 시간")
});

type Facility = z.infer<typeof FacilitySchema>;
```

### 탱크 스키마

```typescript
/**
 * 탱크 유형 Enum
 */
const TankTypeSchema = z.enum([
  "ln2_storage",         // 액체질소 저장 탱크
  "ln2_dewar",          // 듀어 플라스크
  "freezer_minus_80",   // -80°C 초저온 냉동고
  "freezer_minus_150",  // -150°C 냉동고
  "dry_shipper",        // 드라이 시퍼 (운송용)
  "vapor_shipper"       // 증기 시퍼
]).describe("탱크 유형");

type TankType = z.infer<typeof TankTypeSchema>;

const TankTypeKr: Record<TankType, string> = {
  ln2_storage: "액체질소 저장 탱크",
  ln2_dewar: "듀어 플라스크 (실험용)",
  freezer_minus_80: "-80°C 초저온 냉동고",
  freezer_minus_150: "-150°C 초저온 냉동고",
  dry_shipper: "드라이 시퍼 (운송용)",
  vapor_shipper: "증기 시퍼 (운송용)"
};

/**
 * 탱크 상태 Enum
 */
const TankStatusSchema = z.enum([
  "operational",         // 정상 운영
  "filling",             // 보충 중
  "maintenance",         // 점검 중
  "warning",             // 경고
  "critical",            // 위험
  "offline"              // 오프라인
]).describe("탱크 상태");

type TankStatus = z.infer<typeof TankStatusSchema>;

const TankStatusKr: Record<TankStatus, string> = {
  operational: "정상 운영",
  filling: "액체질소 보충 중",
  maintenance: "정기 점검 중",
  warning: "경고 - 주의 필요",
  critical: "위험 - 즉시 조치 필요",
  offline: "오프라인"
};

/**
 * 탱크 스키마
 */
const TankSchema = z.object({
  // 기본 정보
  tankId: UUIDSchema.describe("탱크 ID"),
  facilityId: UUIDSchema.describe("시설 ID"),
  name: z.string().min(1).max(100).describe("탱크명"),
  nameKr: z.string().min(1).max(100).describe("탱크명 (한국어)"),
  type: TankTypeSchema,
  typeKr: z.string().describe("탱크 유형 (한국어)"),

  // 사양
  specifications: z.object({
    manufacturer: z.string().describe("제조사"),
    manufacturerKr: z.string().describe("제조사 (한국어)"),
    model: z.string().describe("모델명"),
    serialNumber: z.string().describe("시리얼 번호"),
    manufactureDate: DateTimeSchema.describe("제조 날짜"),
    capacity: z.object({
      total: z.number().positive().describe("총 용량 (리터)"),
      usable: z.number().positive().describe("사용 가능 용량 (리터)"),
      capacityKr: z.string().describe("용량 (한국어)")
    }),
    dimensions: z.object({
      height: z.number().positive().describe("높이 (cm)"),
      diameter: z.number().positive().describe("직경 (cm)"),
      weight: z.number().positive().describe("무게 (kg)"),
      dimensionsKr: z.string().describe("치수 (한국어)")
    }),
    specsKr: z.string().describe("사양 (한국어)")
  }),

  // 운영 파라미터
  operations: z.object({
    targetTemperature: z.number().describe("목표 온도 (°C)"),
    temperatureRange: z.object({
      min: z.number().describe("최소 온도"),
      max: z.number().describe("최대 온도"),
      tempRangeKr: z.string().describe("온도 범위 (한국어)")
    }),
    targetPressure: z.number().describe("목표 압력 (PSI)"),
    pressureRange: z.object({
      min: z.number().describe("최소 압력"),
      max: z.number().describe("최대 압력"),
      pressureRangeKr: z.string().describe("압력 범위 (한국어)")
    }),
    ln2ConsumptionRate: z.object({
      daily: z.number().nonnegative().describe("일일 소비율 (L/day)"),
      weekly: z.number().nonnegative().describe("주간 소비율"),
      monthly: z.number().nonnegative().describe("월간 소비율"),
      consumptionKr: z.string().describe("소비율 (한국어)")
    }),
    opsKr: z.string().describe("운영 파라미터 (한국어)")
  }),

  // 현재 상태
  currentState: z.object({
    status: TankStatusSchema,
    statusKr: z.string().describe("상태 (한국어)"),
    temperature: z.number().describe("현재 온도 (°C)"),
    pressure: z.number().describe("현재 압력 (PSI)"),
    ln2Level: z.number().min(0).max(100).describe("액체질소 레벨 (%)"),
    sampleCount: z.number().int().nonnegative().describe("보관 샘플 수"),
    lastRefill: DateTimeSchema.describe("마지막 보충 날짜"),
    nextScheduledRefill: DateTimeSchema.describe("다음 예정 보충일"),
    stateKr: z.string().describe("현재 상태 (한국어)")
  }),

  // 센서 연결
  sensors: z.object({
    temperature: z.array(UUIDSchema).describe("온도 센서 ID들"),
    pressure: z.array(UUIDSchema).describe("압력 센서 ID들"),
    level: z.array(UUIDSchema).describe("레벨 센서 ID들"),
    other: z.array(UUIDSchema).optional().describe("기타 센서들"),
    sensorsKr: z.string().describe("연결된 센서 (한국어)")
  }),

  // 보관 내용물
  inventory: z.object({
    racks: z.number().int().nonnegative().describe("랙 수"),
    canes: z.number().int().nonnegative().describe("캔 수"),
    vials: z.number().int().nonnegative().describe("바이알 수"),
    totalSamples: z.number().int().nonnegative().describe("총 샘플 수"),
    sampleTypes: z.array(z.object({
      type: z.string().describe("샘플 유형"),
      typeKr: z.string().describe("샘플 유형 (한국어)"),
      count: z.number().int().nonnegative().describe("개수")
    })).describe("샘플 유형별 개수"),
    inventoryKr: z.string().describe("보관 내용 (한국어)")
  }),

  // 유지보수
  maintenance: z.object({
    lastMaintenance: DateTimeSchema.describe("마지막 점검 날짜"),
    nextMaintenance: DateTimeSchema.describe("다음 점검 예정일"),
    maintenanceInterval: z.number().positive().describe("점검 주기 (일)"),
    maintenanceHistory: z.array(z.object({
      date: DateTimeSchema.describe("날짜"),
      type: z.string().describe("점검 유형"),
      typeKr: z.string().describe("점검 유형 (한국어)"),
      performedBy: z.string().describe("담당자"),
      notes: z.string().optional().describe("비고"),
      documentUrl: z.string().url().optional().describe("문서 URL")
    })).describe("점검 이력"),
    maintenanceKr: z.string().describe("유지보수 (한국어)")
  }),

  // 알림 설정
  alertConfiguration: z.object({
    temperatureAlerts: z.boolean().describe("온도 알림"),
    pressureAlerts: z.boolean().describe("압력 알림"),
    levelAlerts: z.boolean().describe("레벨 알림"),
    doorAlerts: z.boolean().describe("도어 알림"),
    powerAlerts: z.boolean().describe("전원 알림"),
    recipients: z.array(EmailSchema).describe("알림 수신자"),
    configKr: z.string().describe("알림 설정 (한국어)")
  }),

  // 메타데이터
  metadata: z.object({
    installDate: DateTimeSchema.describe("설치 날짜"),
    warrantyExpiry: DateTimeSchema.describe("보증 만료일"),
    location: z.object({
      building: z.string().describe("건물"),
      floor: z.string().describe("층"),
      room: z.string().describe("실"),
      coordinates: CoordinatesSchema.optional().describe("GPS 좌표"),
      locationKr: z.string().describe("위치 (한국어)")
    }),
    notes: z.string().optional().describe("비고"),
    tags: z.array(z.string()).default([]).describe("태그"),
    metadataKr: z.string().describe("메타데이터 (한국어)")
  }),

  // 타임스탬프
  createdAt: DateTimeSchema.describe("생성 시간"),
  updatedAt: DateTimeSchema.describe("수정 시간")
});

type Tank = z.infer<typeof TankSchema>;
```

---

## 알림 및 이벤트 스키마

### 알림 스키마

```typescript
/**
 * 알림 심각도 Enum
 */
const AlertSeveritySchema = z.enum([
  "info",                // 정보
  "warning",             // 경고
  "critical",            // 위험
  "emergency"            // 긴급
]).describe("알림 심각도");

type AlertSeverity = z.infer<typeof AlertSeveritySchema>;

const AlertSeverityKr: Record<AlertSeverity, string> = {
  info: "정보",
  warning: "경고",
  critical: "위험",
  emergency: "긴급"
};

/**
 * 알림 상태 Enum
 */
const AlertStatusSchema = z.enum([
  "active",              // 활성
  "acknowledged",        // 확인됨
  "resolved",            // 해결됨
  "escalated",           // 에스컬레이션됨
  "suppressed"           // 억제됨 (점검 중 등)
]).describe("알림 상태");

type AlertStatus = z.infer<typeof AlertStatusSchema>;

const AlertStatusKr: Record<AlertStatus, string> = {
  active: "활성 - 조치 필요",
  acknowledged: "확인됨 - 처리 중",
  resolved: "해결됨",
  escalated: "에스컬레이션됨 - 상급자 통보",
  suppressed: "억제됨 - 점검 중"
};

/**
 * 알림 스키마
 */
const AlertSchema = z.object({
  // 기본 정보
  alertId: UUIDSchema.describe("알림 ID"),
  ruleId: UUIDSchema.describe("알림 규칙 ID"),

  // 연관 객체
  sensorId: UUIDSchema.optional().describe("센서 ID"),
  tankId: UUIDSchema.optional().describe("탱크 ID"),
  facilityId: UUIDSchema.describe("시설 ID"),

  // 심각도 및 상태
  severity: AlertSeveritySchema,
  severityKr: z.string().describe("심각도 (한국어)"),
  status: AlertStatusSchema,
  statusKr: z.string().describe("상태 (한국어)"),

  // 알림 내용
  title: z.string().min(1).max(200).describe("제목"),
  titleKr: z.string().min(1).max(200).describe("제목 (한국어)"),
  message: z.string().describe("메시지"),
  messageKr: z.string().describe("메시지 (한국어)"),

  // 측정값
  measurement: z.object({
    parameter: z.string().describe("파라미터"),
    parameterKr: z.string().describe("파라미터 (한국어)"),
    value: z.number().describe("측정값"),
    unit: MeasurementUnitSchema,
    unitKr: z.string().describe("단위 (한국어)"),
    threshold: z.number().describe("임계값"),
    deviation: z.number().describe("편차"),
    measurementKr: z.string().describe("측정 정보 (한국어)")
  }).optional(),

  // 발생 정보
  occurrence: z.object({
    firstOccurred: DateTimeSchema.describe("최초 발생 시간"),
    lastOccurred: DateTimeSchema.describe("최근 발생 시간"),
    count: z.number().int().positive().describe("발생 횟수"),
    duration: z.number().nonnegative().describe("지속 시간 (초)"),
    occurrenceKr: z.string().describe("발생 정보 (한국어)")
  }),

  // 처리 정보
  handling: z.object({
    acknowledgedBy: z.string().optional().describe("확인자"),
    acknowledgedAt: DateTimeSchema.optional().describe("확인 시간"),
    resolvedBy: z.string().optional().describe("해결자"),
    resolvedAt: DateTimeSchema.optional().describe("해결 시간"),
    resolutionNotes: z.string().optional().describe("해결 메모"),
    handlingKr: z.string().describe("처리 정보 (한국어)")
  }),

  // 에스컬레이션
  escalation: z.object({
    level: z.number().int().nonnegative().describe("에스컬레이션 레벨"),
    escalatedAt: DateTimeSchema.optional().describe("에스컬레이션 시간"),
    escalatedTo: z.array(z.string()).default([]).describe("에스컬레이션 대상자"),
    escalationKr: z.string().describe("에스컬레이션 (한국어)")
  }),

  // 알림 전송
  notifications: z.array(z.object({
    channel: z.enum(["email", "sms", "push", "webhook", "voice"]).describe("채널"),
    channelKr: z.string().describe("채널 (한국어)"),
    recipient: z.string().describe("수신자"),
    sentAt: DateTimeSchema.describe("전송 시간"),
    delivered: z.boolean().describe("전달 여부"),
    error: z.string().optional().describe("오류 메시지")
  })).describe("알림 전송 내역"),

  // 자동 조치
  automation: z.object({
    executed: z.boolean().describe("자동 조치 실행 여부"),
    script: z.string().optional().describe("스크립트명"),
    executedAt: DateTimeSchema.optional().describe("실행 시간"),
    result: z.string().optional().describe("실행 결과"),
    automationKr: z.string().describe("자동 조치 (한국어)")
  }),

  // 메타데이터
  metadata: z.object({
    priority: z.number().int().min(1).max(10).describe("우선순위 (1=highest)"),
    category: z.string().optional().describe("카테고리"),
    categoryKr: z.string().optional().describe("카테고리 (한국어)"),
    tags: z.array(z.string()).default([]).describe("태그"),
    relatedAlerts: z.array(UUIDSchema).default([]).describe("관련 알림"),
    attachments: z.array(z.object({
      type: z.string().describe("유형"),
      url: z.string().url().describe("URL"),
      description: z.string().optional().describe("설명")
    })).default([]).describe("첨부파일"),
    metadataKr: z.string().describe("메타데이터 (한국어)")
  }),

  // 타임스탬프
  createdAt: DateTimeSchema.describe("생성 시간"),
  updatedAt: DateTimeSchema.describe("수정 시간")
});

type Alert = z.infer<typeof AlertSchema>;
```

---

## 사용자 및 권한 스키마

### 사용자 스키마

```typescript
/**
 * 사용자 역할 Enum
 */
const UserRoleSchema = z.enum([
  "super_admin",         // 슈퍼 관리자
  "facility_admin",      // 시설 관리자
  "engineer",            // 엔지니어
  "technician",          // 기술자
  "clinician",           // 임상의
  "researcher",          // 연구자
  "auditor",             // 감사자
  "viewer"               // 조회자
]).describe("사용자 역할");

type UserRole = z.infer<typeof UserRoleSchema>;

const UserRoleKr: Record<UserRole, string> = {
  super_admin: "슈퍼 관리자",
  facility_admin: "시설 관리자",
  engineer: "엔지니어",
  technician: "기술자",
  clinician: "임상의",
  researcher: "연구자",
  auditor: "감사자",
  viewer: "조회자"
};

/**
 * 사용자 스키마
 */
const UserSchema = z.object({
  // 기본 정보
  userId: UUIDSchema.describe("사용자 ID"),
  username: z.string().min(3).max(50).describe("사용자명"),
  email: EmailSchema.describe("이메일"),

  // 개인 정보
  profile: z.object({
    firstName: z.string().min(1).max(50).describe("이름"),
    lastName: z.string().min(1).max(50).describe("성"),
    fullNameKr: z.string().min(1).max(100).describe("한국어 이름"),
    phone: KoreanPhoneSchema.optional().describe("전화번호"),
    mobile: KoreanPhoneSchema.describe("휴대전화"),
    department: z.string().optional().describe("부서"),
    departmentKr: z.string().optional().describe("부서 (한국어)"),
    jobTitle: z.string().optional().describe("직책"),
    jobTitleKr: z.string().optional().describe("직책 (한국어)"),
    profileKr: z.string().describe("프로필 (한국어)")
  }),

  // 역할 및 권한
  authorization: z.object({
    role: UserRoleSchema,
    roleKr: z.string().describe("역할 (한국어)"),
    facilities: z.array(UUIDSchema).describe("접근 가능 시설"),
    permissions: z.array(z.string()).describe("권한"),
    permissionsKr: z.array(z.string()).describe("권한 (한국어)"),
    authKr: z.string().describe("권한 정보 (한국어)")
  }),

  // 계정 상태
  account: z.object({
    status: z.enum(["active", "inactive", "suspended", "locked"]).describe("상태"),
    statusKr: z.string().describe("상태 (한국어)"),
    emailVerified: z.boolean().describe("이메일 인증"),
    phoneVerified: z.boolean().describe("전화 인증"),
    twoFactorEnabled: z.boolean().describe("2단계 인증"),
    lastLogin: DateTimeSchema.optional().describe("마지막 로그인"),
    passwordChangedAt: DateTimeSchema.describe("비밀번호 변경일"),
    passwordExpiresAt: DateTimeSchema.optional().describe("비밀번호 만료일"),
    accountKr: z.string().describe("계정 상태 (한국어)")
  }),

  // 알림 설정
  notifications: z.object({
    email: z.boolean().describe("이메일 알림"),
    sms: z.boolean().describe("SMS 알림"),
    push: z.boolean().describe("푸시 알림"),
    severityFilter: z.array(AlertSeveritySchema).describe("알림 심각도 필터"),
    quietHours: z.object({
      enabled: z.boolean().describe("방해금지 시간 사용"),
      start: z.string().regex(/^([01]\d|2[0-3]):([0-5]\d)$/).describe("시작 시간 (HH:MM)"),
      end: z.string().regex(/^([01]\d|2[0-3]):([0-5]\d)$/).describe("종료 시간 (HH:MM)"),
      quietKr: z.string().describe("방해금지 시간 (한국어)")
    }).optional(),
    notificationsKr: z.string().describe("알림 설정 (한국어)")
  }),

  // 메타데이터
  metadata: z.object({
    employeeId: z.string().optional().describe("직원 번호"),
    licenseNumber: z.string().optional().describe("면허 번호"),
    certifications: z.array(z.string()).default([]).describe("자격증"),
    certificationsKr: z.array(z.string()).default([]).describe("자격증 (한국어)"),
    notes: z.string().optional().describe("비고"),
    tags: z.array(z.string()).default([]).describe("태그"),
    metadataKr: z.string().describe("메타데이터 (한국어)")
  }),

  // 타임스탬프
  createdAt: DateTimeSchema.describe("생성 시간"),
  updatedAt: DateTimeSchema.describe("수정 시간")
});

type User = z.infer<typeof UserSchema>;
```

---

## 감사 로그 스키마

```typescript
/**
 * 감사 로그 액션 Enum
 */
const AuditActionSchema = z.enum([
  // 인증
  "auth.login",
  "auth.logout",
  "auth.failed_login",
  // 센서
  "sensor.create",
  "sensor.update",
  "sensor.delete",
  "sensor.calibrate",
  // 탱크
  "tank.create",
  "tank.update",
  "tank.delete",
  "tank.refill",
  // 알림
  "alert.create",
  "alert.acknowledge",
  "alert.resolve",
  "alert.escalate",
  // 설정
  "config.update",
  "config.export",
  "config.import",
  // 데이터
  "data.export",
  "data.delete",
  "data.backup",
  // 사용자
  "user.create",
  "user.update",
  "user.delete",
  "user.role_change"
]).describe("감사 액션");

type AuditAction = z.infer<typeof AuditActionSchema>;

const AuditActionKr: Partial<Record<AuditAction, string>> = {
  "auth.login": "로그인",
  "auth.logout": "로그아웃",
  "auth.failed_login": "로그인 실패",
  "sensor.create": "센서 생성",
  "sensor.update": "센서 수정",
  "sensor.delete": "센서 삭제",
  "sensor.calibrate": "센서 교정",
  "tank.create": "탱크 생성",
  "tank.update": "탱크 수정",
  "tank.delete": "탱크 삭제",
  "tank.refill": "탱크 보충",
  "alert.create": "알림 생성",
  "alert.acknowledge": "알림 확인",
  "alert.resolve": "알림 해결",
  "alert.escalate": "알림 에스컬레이션",
  "config.update": "설정 변경",
  "config.export": "설정 내보내기",
  "config.import": "설정 가져오기",
  "data.export": "데이터 내보내기",
  "data.delete": "데이터 삭제",
  "data.backup": "데이터 백업",
  "user.create": "사용자 생성",
  "user.update": "사용자 수정",
  "user.delete": "사용자 삭제",
  "user.role_change": "사용자 역할 변경"
};

/**
 * 감사 로그 스키마
 */
const AuditLogSchema = z.object({
  // 기본 정보
  logId: UUIDSchema.describe("로그 ID"),
  action: AuditActionSchema,
  actionKr: z.string().describe("액션 (한국어)"),

  // 사용자 정보
  user: z.object({
    userId: UUIDSchema.describe("사용자 ID"),
    username: z.string().describe("사용자명"),
    email: EmailSchema.describe("이메일"),
    role: UserRoleSchema.describe("역할"),
    roleKr: z.string().describe("역할 (한국어)"),
    userKr: z.string().describe("사용자 정보 (한국어)")
  }),

  // 대상 객체
  target: z.object({
    type: z.string().describe("객체 유형"),
    typeKr: z.string().describe("객체 유형 (한국어)"),
    id: z.string().describe("객체 ID"),
    name: z.string().optional().describe("객체명"),
    targetKr: z.string().describe("대상 (한국어)")
  }).optional(),

  // 변경 내용
  changes: z.object({
    before: z.record(z.unknown()).optional().describe("변경 전"),
    after: z.record(z.unknown()).optional().describe("변경 후"),
    changesKr: z.string().describe("변경 내용 (한국어)")
  }).optional(),

  // 요청 정보
  request: z.object({
    ipAddress: z.string().ip().describe("IP 주소"),
    userAgent: z.string().optional().describe("User Agent"),
    method: z.enum(["GET", "POST", "PUT", "PATCH", "DELETE"]).optional().describe("HTTP 메소드"),
    endpoint: z.string().optional().describe("API 엔드포인트"),
    requestKr: z.string().describe("요청 정보 (한국어)")
  }),

  // 결과
  result: z.object({
    success: z.boolean().describe("성공 여부"),
    statusCode: z.number().int().optional().describe("HTTP 상태 코드"),
    errorMessage: z.string().optional().describe("오류 메시지"),
    resultKr: z.string().describe("결과 (한국어)")
  }),

  // 메타데이터
  metadata: z.object({
    facilityId: UUIDSchema.optional().describe("시설 ID"),
    sessionId: z.string().optional().describe("세션 ID"),
    traceId: z.string().optional().describe("추적 ID"),
    duration: z.number().nonnegative().optional().describe("처리 시간 (ms)"),
    tags: z.array(z.string()).default([]).describe("태그"),
    metadataKr: z.string().describe("메타데이터 (한국어)")
  }),

  // 타임스탬프
  timestamp: DateTimeSchema.describe("발생 시간")
});

type AuditLog = z.infer<typeof AuditLogSchema>;
```

---

## 통합 검증 시스템

### 데이터 검증 유틸리티

```typescript
/**
 * WIA Cryo Monitoring 데이터 검증 시스템
 */
class CryoDataValidator {
  /**
   * 센서 설정 검증
   */
  static validateSensorConfig(data: unknown): SensorConfig {
    try {
      return SensorConfigSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        const errors = error.errors.map(e => ({
          path: e.path.join("."),
          message: e.message,
          messageKr: this.translateError(e)
        }));

        throw new Error(
          `센서 설정 검증 실패:\n${errors.map(e => `- ${e.path}: ${e.messageKr}`).join("\n")}`
        );
      }
      throw error;
    }
  }

  /**
   * 센서 읽기 검증
   */
  static validateSensorReading(data: unknown): SensorReading {
    try {
      return SensorReadingSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        const errors = error.errors.map(e => ({
          path: e.path.join("."),
          message: e.message,
          messageKr: this.translateError(e)
        }));

        throw new Error(
          `센서 읽기 검증 실패:\n${errors.map(e => `- ${e.path}: ${e.messageKr}`).join("\n")}`
        );
      }
      throw error;
    }
  }

  /**
   * 시설 검증
   */
  static validateFacility(data: unknown): Facility {
    try {
      return FacilitySchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        const errors = error.errors.map(e => ({
          path: e.path.join("."),
          message: e.message,
          messageKr: this.translateError(e)
        }));

        throw new Error(
          `시설 정보 검증 실패:\n${errors.map(e => `- ${e.path}: ${e.messageKr}`).join("\n")}`
        );
      }
      throw error;
    }
  }

  /**
   * 탱크 검증
   */
  static validateTank(data: unknown): Tank {
    try {
      return TankSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        const errors = error.errors.map(e => ({
          path: e.path.join("."),
          message: e.message,
          messageKr: this.translateError(e)
        }));

        throw new Error(
          `탱크 정보 검증 실패:\n${errors.map(e => `- ${e.path}: ${e.messageKr}`).join("\n")}`
        );
      }
      throw error;
    }
  }

  /**
   * 알림 검증
   */
  static validateAlert(data: unknown): Alert {
    try {
      return AlertSchema.parse(data);
    } catch (error) {
      if (error instanceof z.ZodError) {
        const errors = error.errors.map(e => ({
          path: e.path.join("."),
          message: e.message,
          messageKr: this.translateError(e)
        }));

        throw new Error(
          `알림 정보 검증 실패:\n${errors.map(e => `- ${e.path}: ${e.messageKr}`).join("\n")}`
        );
      }
      throw error;
    }
  }

  /**
   * 오류 메시지 한국어 번역
   */
  private static translateError(error: z.ZodIssue): string {
    switch (error.code) {
      case "invalid_type":
        return `타입이 올바르지 않습니다. ${error.expected} 타입이어야 하지만 ${error.received} 타입입니다.`;
      case "invalid_string":
        return `문자열 형식이 올바르지 않습니다.`;
      case "too_small":
        return `값이 너무 작습니다. 최소 ${error.minimum}이어야 합니다.`;
      case "too_big":
        return `값이 너무 큽니다. 최대 ${error.maximum}이어야 합니다.`;
      case "invalid_enum_value":
        return `허용되지 않는 값입니다. ${error.options.join(", ")} 중 하나여야 합니다.`;
      case "invalid_date":
        return `유효하지 않은 날짜입니다.`;
      default:
        return error.message;
    }
  }

  /**
   * 배치 검증
   * 여러 객체를 한번에 검증
   */
  static validateBatch<T>(
    schema: z.ZodSchema<T>,
    data: unknown[]
  ): { valid: T[]; invalid: { index: number; error: string }[] } {
    const valid: T[] = [];
    const invalid: { index: number; error: string }[] = [];

    data.forEach((item, index) => {
      try {
        valid.push(schema.parse(item));
      } catch (error) {
        if (error instanceof z.ZodError) {
          invalid.push({
            index,
            error: error.errors.map(e => `${e.path.join(".")}: ${e.message}`).join(", ")
          });
        }
      }
    });

    return { valid, invalid };
  }
}

// 사용 예시
const rawSensorData = {
  sensorId: "123e4567-e89b-12d3-a456-426614174000",
  name: "Tank A1 Temperature Sensor",
  nameKr: "탱크 A1 온도 센서",
  type: "temperature",
  typeKr: "온도 센서",
  // ... 나머지 필드
};

try {
  const validatedSensor = CryoDataValidator.validateSensorConfig(rawSensorData);
  console.log("센서 검증 성공:", validatedSensor.nameKr);
} catch (error) {
  console.error("센서 검증 실패:", error.message);
}
```

---

## 결론

WIA Cryo Monitoring Standard는 Zod를 활용한 완전한 타입 안전성과 런타임 검증을 제공합니다.

### 핵심 특징

1. **완전한 타입 안전성**: TypeScript + Zod
2. **한국어 지원**: 모든 필드에 한국어 설명
3. **런타임 검증**: API 경계에서 데이터 검증
4. **상세한 오류 메시지**: 한국어 오류 메시지

### 다음 장 예고

다음 장에서는 **API 인터페이스**를 다룹니다:
- REST API 엔드포인트
- GraphQL 스키마
- WebSocket 실시간 스트리밍
- 한국어 API 문서

---

**© 2026 WIA (World Certification Industry Association)**
**弘益人間 (홍익인간) - Benefit All Humanity**
