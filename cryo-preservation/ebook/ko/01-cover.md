# WIA 냉동보존 표준 (WIA Cryo Preservation Standard)

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 표준 개요

WIA 냉동보존 표준은 생식세포, 배아, 조직, 제대혈 등 생물학적 검체의 냉동보존 전 과정을 표준화하여 안전하고 효율적인 보존 시스템을 제공합니다.

### 핵심 가치

- **생명 보존**: 생명의 가능성을 미래로 연장
- **품질 보증**: 검체 생존율과 품질 보장
- **추적성**: 전체 생명주기 추적 및 기록
- **규정 준수**: 국제 및 국내 규정 완벽 준수
- **접근성**: 모든 의료기관이 활용 가능한 표준

## TypeScript 핵심 타입 정의

```typescript
/**
 * WIA 냉동보존 표준 v1.0
 *
 * 이 표준은 생물학적 검체의 냉동보존 프로세스를 표준화합니다.
 * - 검체 유형 분류 및 관리
 * - 동결 프로토콜 정의
 * - 보존 환경 모니터링
 * - 품질 관리 및 추적
 */

import { z } from 'zod';

/**
 * 검체 유형
 * 냉동보존 가능한 생물학적 검체의 분류
 */
export enum SpecimenType {
  SPERM = 'SPERM',                    // 정자
  OOCYTE = 'OOCYTE',                  // 난자
  EMBRYO = 'EMBRYO',                  // 배아
  CORD_BLOOD = 'CORD_BLOOD',          // 제대혈
  TISSUE = 'TISSUE',                  // 조직
  STEM_CELL = 'STEM_CELL',            // 줄기세포
  OVARIAN_TISSUE = 'OVARIAN_TISSUE',  // 난소조직
  TESTICULAR_TISSUE = 'TESTICULAR_TISSUE', // 고환조직
  BONE_MARROW = 'BONE_MARROW',        // 골수
  PERIPHERAL_BLOOD = 'PERIPHERAL_BLOOD', // 말초혈
}

/**
 * 검체 유형 한글명
 */
export const SpecimenTypeKr: Record<SpecimenType, string> = {
  [SpecimenType.SPERM]: '정자',
  [SpecimenType.OOCYTE]: '난자',
  [SpecimenType.EMBRYO]: '배아',
  [SpecimenType.CORD_BLOOD]: '제대혈',
  [SpecimenType.TISSUE]: '조직',
  [SpecimenType.STEM_CELL]: '줄기세포',
  [SpecimenType.OVARIAN_TISSUE]: '난소조직',
  [SpecimenType.TESTICULAR_TISSUE]: '고환조직',
  [SpecimenType.BONE_MARROW]: '골수',
  [SpecimenType.PERIPHERAL_BLOOD]: '말초혈',
};

/**
 * 보존 방법
 * 냉동보존 기술 분류
 */
export enum PreservationMethod {
  SLOW_FREEZING = 'SLOW_FREEZING',        // 완만동결
  VITRIFICATION = 'VITRIFICATION',        // 유리화동결
  RAPID_FREEZING = 'RAPID_FREEZING',      // 급속동결
  CONTROLLED_RATE = 'CONTROLLED_RATE',    // 프로그램동결
}

/**
 * 보존 방법 한글명
 */
export const PreservationMethodKr: Record<PreservationMethod, string> = {
  [PreservationMethod.SLOW_FREEZING]: '완만동결',
  [PreservationMethod.VITRIFICATION]: '유리화동결',
  [PreservationMethod.RAPID_FREEZING]: '급속동결',
  [PreservationMethod.CONTROLLED_RATE]: '프로그램동결',
};

/**
 * 검체 상태
 */
export enum SpecimenStatus {
  REGISTERED = 'REGISTERED',              // 등록됨
  PREPARED = 'PREPARED',                  // 준비됨
  FREEZING = 'FREEZING',                  // 동결중
  FROZEN = 'FROZEN',                      // 동결완료
  THAWING = 'THAWING',                    // 해동중
  THAWED = 'THAWED',                      // 해동완료
  TRANSFERRED = 'TRANSFERRED',            // 이식됨
  DISPOSED = 'DISPOSED',                  // 폐기됨
  QUARANTINE = 'QUARANTINE',              // 격리중
}

/**
 * 검체 상태 한글명
 */
export const SpecimenStatusKr: Record<SpecimenStatus, string> = {
  [SpecimenStatus.REGISTERED]: '등록됨',
  [SpecimenStatus.PREPARED]: '준비됨',
  [SpecimenStatus.FREEZING]: '동결중',
  [SpecimenStatus.FROZEN]: '동결완료',
  [SpecimenStatus.THAWING]: '해동중',
  [SpecimenStatus.THAWED]: '해동완료',
  [SpecimenStatus.TRANSFERRED]: '이식됨',
  [SpecimenStatus.DISPOSED]: '폐기됨',
  [SpecimenStatus.QUARANTINE]: '격리중',
};

/**
 * 검체 정보 스키마
 */
export const SpecimenSchema = z.object({
  // 기본 정보
  specimenId: z.string().uuid().describe('검체 고유 ID'),
  patientId: z.string().describe('환자 ID (암호화)'),
  type: z.nativeEnum(SpecimenType).describe('검체 유형'),
  typeKr: z.string().describe('검체 유형 한글명'),

  // 수집 정보
  collectionDate: z.string().datetime().describe('채취일시'),
  collectionMethod: z.string().describe('채취 방법'),
  collectedBy: z.string().describe('채취자 ID'),

  // 품질 정보
  quality: z.object({
    volume: z.number().describe('용량 (ml)'),
    concentration: z.number().optional().describe('농도'),
    motility: z.number().min(0).max(100).optional().describe('운동성 (%)'),
    viability: z.number().min(0).max(100).describe('생존율 (%)'),
    morphology: z.string().optional().describe('형태학적 평가'),
  }),

  // 보존 정보
  preservation: z.object({
    method: z.nativeEnum(PreservationMethod).describe('보존 방법'),
    methodKr: z.string().describe('보존 방법 한글명'),
    startDate: z.string().datetime().describe('동결 시작일시'),
    endDate: z.string().datetime().optional().describe('동결 완료일시'),
    cryoprotectant: z.string().describe('동결보호제'),
    coolingRate: z.string().describe('냉각 속도'),
  }),

  // 저장 위치
  storage: z.object({
    tankId: z.string().describe('탱크 ID'),
    canisterId: z.string().describe('캐니스터 ID'),
    caneId: z.string().describe('케인 ID'),
    position: z.string().describe('위치 (층-행-열)'),
    temperature: z.number().describe('저장 온도 (°C)'),
  }),

  // 상태 정보
  status: z.nativeEnum(SpecimenStatus).describe('검체 상태'),
  statusKr: z.string().describe('검체 상태 한글명'),

  // 추적 정보
  barcodeId: z.string().describe('바코드 ID'),
  rfidTag: z.string().optional().describe('RFID 태그'),

  // 메타데이터
  metadata: z.object({
    createdAt: z.string().datetime(),
    updatedAt: z.string().datetime(),
    createdBy: z.string(),
    notes: z.string().optional(),
  }),
});

export type Specimen = z.infer<typeof SpecimenSchema>;

/**
 * 동결 프로토콜 스키마
 */
export const FreezingProtocolSchema = z.object({
  protocolId: z.string().uuid().describe('프로토콜 ID'),
  name: z.string().describe('프로토콜 명칭'),
  nameKr: z.string().describe('프로토콜 한글명'),
  specimenType: z.nativeEnum(SpecimenType).describe('적용 검체 유형'),
  method: z.nativeEnum(PreservationMethod).describe('동결 방법'),

  // 프로토콜 단계
  steps: z.array(z.object({
    stepNumber: z.number().describe('단계 번호'),
    name: z.string().describe('단계명'),
    nameKr: z.string().describe('단계 한글명'),
    duration: z.number().describe('소요 시간 (분)'),
    temperature: z.number().describe('목표 온도 (°C)'),
    coolingRate: z.number().optional().describe('냉각 속도 (°C/min)'),
    action: z.string().describe('수행 작업'),
    actionKr: z.string().describe('수행 작업 한글'),
  })),

  // 동결보호제
  cryoprotectant: z.object({
    name: z.string().describe('보호제 명칭'),
    concentration: z.number().describe('농도 (%)'),
    equilibrationTime: z.number().describe('평형 시간 (분)'),
  }),

  // 품질 관리
  qualityControl: z.object({
    preFreezingCheck: z.array(z.string()).describe('동결 전 검사 항목'),
    postFreezingCheck: z.array(z.string()).describe('동결 후 검사 항목'),
    expectedViability: z.number().describe('예상 생존율 (%)'),
  }),

  // 메타데이터
  version: z.string().describe('프로토콜 버전'),
  approvedBy: z.string().describe('승인자'),
  approvedDate: z.string().datetime().describe('승인일'),
});

export type FreezingProtocol = z.infer<typeof FreezingProtocolSchema>;

/**
 * 해동 프로토콜 스키마
 */
export const ThawingProtocolSchema = z.object({
  protocolId: z.string().uuid().describe('프로토콜 ID'),
  name: z.string().describe('프로토콜 명칭'),
  nameKr: z.string().describe('프로토콜 한글명'),
  specimenType: z.nativeEnum(SpecimenType).describe('적용 검체 유형'),

  // 해동 단계
  steps: z.array(z.object({
    stepNumber: z.number().describe('단계 번호'),
    name: z.string().describe('단계명'),
    nameKr: z.string().describe('단계 한글명'),
    duration: z.number().describe('소요 시간 (분)'),
    temperature: z.number().describe('목표 온도 (°C)'),
    action: z.string().describe('수행 작업'),
    actionKr: z.string().describe('수행 작업 한글'),
  })),

  // 희석 프로토콜
  dilution: z.object({
    solution: z.string().describe('희석액'),
    steps: z.array(z.object({
      stepNumber: z.number(),
      volume: z.number().describe('첨가 용량 (ml)'),
      duration: z.number().describe('대기 시간 (분)'),
    })),
  }),

  // 품질 평가
  qualityAssessment: z.object({
    viabilityTest: z.boolean().describe('생존율 검사 필요'),
    motilityTest: z.boolean().optional().describe('운동성 검사 필요'),
    morphologyTest: z.boolean().optional().describe('형태 검사 필요'),
    minimumViability: z.number().describe('최소 생존율 (%)'),
  }),

  // 메타데이터
  version: z.string().describe('프로토콜 버전'),
  approvedBy: z.string().describe('승인자'),
  approvedDate: z.string().datetime().describe('승인일'),
});

export type ThawingProtocol = z.infer<typeof ThawingProtocolSchema>;
```

## 검체 유형별 특성

### 1. 생식세포 (Gametes)

#### 정자 (Sperm)
```typescript
/**
 * 정자 검체 관리
 */
export class SpermSpecimenManager {
  /**
   * 정자 검체 등록
   *
   * @param data 정자 검체 데이터
   * @returns 등록된 검체 정보
   */
  async registerSperm(data: {
    patientId: string;
    collectionDate: Date;
    volume: number;            // ml
    concentration: number;     // million/ml
    motility: number;          // %
    morphology: string;        // WHO 기준
  }): Promise<Specimen> {
    // 품질 평가
    const quality = this.assessSpermQuality({
      concentration: data.concentration,
      motility: data.motility,
      morphology: data.morphology,
    });

    // 동결 방법 결정
    const method = this.selectPreservationMethod(quality);

    // 검체 등록
    const specimen: Specimen = {
      specimenId: crypto.randomUUID(),
      patientId: this.encryptPatientId(data.patientId),
      type: SpecimenType.SPERM,
      typeKr: '정자',
      collectionDate: data.collectionDate.toISOString(),
      collectionMethod: '자위법',
      collectedBy: 'current-user-id',
      quality: {
        volume: data.volume,
        concentration: data.concentration,
        motility: data.motility,
        viability: this.calculateViability(data),
        morphology: data.morphology,
      },
      preservation: {
        method,
        methodKr: PreservationMethodKr[method],
        startDate: new Date().toISOString(),
        cryoprotectant: 'Glycerol 10%',
        coolingRate: '-1°C/min to -80°C, then to -196°C',
      },
      storage: {
        tankId: await this.allocateTank(),
        canisterId: await this.allocateCanister(),
        caneId: await this.allocateCane(),
        position: await this.allocatePosition(),
        temperature: -196,
      },
      status: SpecimenStatus.REGISTERED,
      statusKr: '등록됨',
      barcodeId: this.generateBarcode(),
      metadata: {
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
        createdBy: 'current-user-id',
      },
    };

    return specimen;
  }

  /**
   * 정자 품질 평가
   * WHO 2021 기준
   */
  private assessSpermQuality(params: {
    concentration: number;
    motility: number;
    morphology: string;
  }): 'excellent' | 'good' | 'fair' | 'poor' {
    // WHO 정상 기준:
    // - 농도: ≥ 16 million/ml
    // - 운동성: ≥ 42%
    // - 정상 형태: ≥ 4%

    if (params.concentration >= 40 && params.motility >= 60) {
      return 'excellent';
    } else if (params.concentration >= 20 && params.motility >= 50) {
      return 'good';
    } else if (params.concentration >= 16 && params.motility >= 42) {
      return 'fair';
    } else {
      return 'poor';
    }
  }

  private selectPreservationMethod(quality: string): PreservationMethod {
    // 품질이 우수한 경우 완만동결, 그 외 유리화동결
    return quality === 'excellent' || quality === 'good'
      ? PreservationMethod.SLOW_FREEZING
      : PreservationMethod.VITRIFICATION;
  }

  private calculateViability(data: any): number {
    // 생존율 계산 (간이 공식)
    return Math.min(100, data.motility * 1.2);
  }

  private encryptPatientId(patientId: string): string {
    // 환자 ID 암호화 (실제로는 AES-256 사용)
    return `encrypted_${patientId}`;
  }

  private async allocateTank(): Promise<string> {
    return 'TANK-01';
  }

  private async allocateCanister(): Promise<string> {
    return 'CAN-A1';
  }

  private async allocateCane(): Promise<string> {
    return 'CANE-001';
  }

  private async allocatePosition(): Promise<string> {
    return '1-A-1';
  }

  private generateBarcode(): string {
    return `BC${Date.now()}${Math.random().toString(36).substr(2, 9)}`;
  }
}
```

#### 난자 (Oocyte)
```typescript
/**
 * 난자 성숙도
 */
export enum OocyteMaturity {
  GV = 'GV',        // 난포기 (Germinal Vesicle)
  MI = 'MI',        // 제1감수분열 (Metaphase I)
  MII = 'MII',      // 제2감수분열 (Metaphase II) - 성숙
}

/**
 * 난자 검체 관리
 */
export class OocyteSpecimenManager {
  /**
   * 난자 검체 등록
   */
  async registerOocyte(data: {
    patientId: string;
    retrievalDate: Date;
    maturity: OocyteMaturity;
    morphology: string;
  }): Promise<Specimen> {
    const specimen: Specimen = {
      specimenId: crypto.randomUUID(),
      patientId: this.encryptPatientId(data.patientId),
      type: SpecimenType.OOCYTE,
      typeKr: '난자',
      collectionDate: data.retrievalDate.toISOString(),
      collectionMethod: '초음파 유도 난자 채취술',
      collectedBy: 'embryologist-id',
      quality: {
        volume: 0.001, // 난자는 부피가 매우 작음
        viability: this.assessOocyteViability(data.maturity),
        morphology: data.morphology,
      },
      preservation: {
        method: PreservationMethod.VITRIFICATION, // 난자는 주로 유리화동결
        methodKr: '유리화동결',
        startDate: new Date().toISOString(),
        cryoprotectant: 'EG+DMSO mixture',
        coolingRate: '-23,000°C/min (vitrification)',
      },
      storage: {
        tankId: 'TANK-OO-01',
        canisterId: 'CAN-OO-A1',
        caneId: 'CANE-OO-001',
        position: '1-A-1',
        temperature: -196,
      },
      status: SpecimenStatus.REGISTERED,
      statusKr: '등록됨',
      barcodeId: this.generateBarcode(),
      metadata: {
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
        createdBy: 'embryologist-id',
        notes: `성숙도: ${data.maturity}`,
      },
    };

    return specimen;
  }

  private assessOocyteViability(maturity: OocyteMaturity): number {
    // MII (성숙 난자)가 가장 높은 생존율
    switch (maturity) {
      case OocyteMaturity.MII:
        return 95;
      case OocyteMaturity.MI:
        return 70;
      case OocyteMaturity.GV:
        return 50;
      default:
        return 0;
    }
  }

  private encryptPatientId(patientId: string): string {
    return `encrypted_${patientId}`;
  }

  private generateBarcode(): string {
    return `OO${Date.now()}${Math.random().toString(36).substr(2, 9)}`;
  }
}
```

### 2. 배아 (Embryo)

```typescript
/**
 * 배아 발달 단계
 */
export enum EmbryoStage {
  ZYGOTE = 'ZYGOTE',              // 접합체 (수정 후 1일)
  CLEAVAGE_2 = 'CLEAVAGE_2',      // 2세포기
  CLEAVAGE_4 = 'CLEAVAGE_4',      // 4세포기
  CLEAVAGE_8 = 'CLEAVAGE_8',      // 8세포기
  MORULA = 'MORULA',              // 상실배 (16세포 이상)
  EARLY_BLASTOCYST = 'EARLY_BLASTOCYST',    // 초기 포배
  BLASTOCYST = 'BLASTOCYST',      // 포배 (5-6일)
  EXPANDED_BLASTOCYST = 'EXPANDED_BLASTOCYST', // 확장 포배
}

/**
 * 배아 등급 (Gardner 분류)
 */
export interface EmbryoGrade {
  expansion: number;      // 1-6 (포배 팽창도)
  icm: 'A' | 'B' | 'C';  // Inner Cell Mass 품질
  te: 'A' | 'B' | 'C';   // Trophectoderm 품질
}

/**
 * 배아 검체 관리
 */
export class EmbryoSpecimenManager {
  /**
   * 배아 검체 등록
   */
  async registerEmbryo(data: {
    patientId: string;
    fertilizationDate: Date;
    stage: EmbryoStage;
    grade: EmbryoGrade;
    geneticTestResult?: string;
  }): Promise<Specimen> {
    const quality = this.assessEmbryoQuality(data.grade);

    const specimen: Specimen = {
      specimenId: crypto.randomUUID(),
      patientId: this.encryptPatientId(data.patientId),
      type: SpecimenType.EMBRYO,
      typeKr: '배아',
      collectionDate: data.fertilizationDate.toISOString(),
      collectionMethod: 'IVF/ICSI',
      collectedBy: 'embryologist-id',
      quality: {
        volume: 0.0001,
        viability: quality.viability,
        morphology: `${data.stage} - ${data.grade.expansion}${data.grade.icm}${data.grade.te}`,
      },
      preservation: {
        method: PreservationMethod.VITRIFICATION,
        methodKr: '유리화동결',
        startDate: new Date().toISOString(),
        cryoprotectant: 'EG+DMSO+Sucrose',
        coolingRate: '-23,000°C/min',
      },
      storage: {
        tankId: 'TANK-EM-01',
        canisterId: 'CAN-EM-A1',
        caneId: 'CANE-EM-001',
        position: '1-A-1',
        temperature: -196,
      },
      status: SpecimenStatus.REGISTERED,
      statusKr: '등록됨',
      barcodeId: this.generateBarcode(),
      metadata: {
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
        createdBy: 'embryologist-id',
        notes: data.geneticTestResult
          ? `PGT 결과: ${data.geneticTestResult}`
          : undefined,
      },
    };

    return specimen;
  }

  /**
   * 배아 품질 평가
   * Gardner 분류 기준
   */
  private assessEmbryoQuality(grade: EmbryoGrade): {
    viability: number;
    implantationRate: number;
  } {
    let baseViability = 90;

    // ICM 등급에 따른 조정
    if (grade.icm === 'A') baseViability += 5;
    else if (grade.icm === 'C') baseViability -= 10;

    // TE 등급에 따른 조정
    if (grade.te === 'A') baseViability += 5;
    else if (grade.te === 'C') baseViability -= 10;

    // 팽창도에 따른 조정
    if (grade.expansion >= 4) baseViability += 5;

    const implantationRate = baseViability * 0.6; // 착상률은 생존율의 약 60%

    return {
      viability: Math.min(100, Math.max(0, baseViability)),
      implantationRate: Math.min(100, Math.max(0, implantationRate)),
    };
  }

  private encryptPatientId(patientId: string): string {
    return `encrypted_${patientId}`;
  }

  private generateBarcode(): string {
    return `EM${Date.now()}${Math.random().toString(36).substr(2, 9)}`;
  }
}
```

## 한국 시장 적용

### 규제 준수

```typescript
/**
 * 한국 생명윤리법 준수 모듈
 */
export class KoreaBioethicsCompliance {
  /**
   * 생명윤리 및 안전에 관한 법률 준수 확인
   */
  async validateBioethicsLaw(specimen: Specimen): Promise<{
    compliant: boolean;
    violations: string[];
  }> {
    const violations: string[] = [];

    // 1. 배아 보존 기간 제한 (5년, 연장 가능)
    if (specimen.type === SpecimenType.EMBRYO) {
      const preservationDuration = this.calculateDuration(
        new Date(specimen.preservation.startDate),
        new Date()
      );

      if (preservationDuration > 5 * 365) {
        violations.push('배아 보존 기간 5년 초과');
      }
    }

    // 2. 동의서 확인
    const consentValid = await this.verifyConsent(specimen.patientId);
    if (!consentValid) {
      violations.push('유효한 동의서 없음');
    }

    // 3. 시설 등록 확인
    const facilityRegistered = await this.verifyFacilityRegistration();
    if (!facilityRegistered) {
      violations.push('보건복지부 미등록 시설');
    }

    return {
      compliant: violations.length === 0,
      violations,
    };
  }

  /**
   * IRB 승인 확인
   */
  async verifyIRBApproval(protocolId: string): Promise<boolean> {
    // IRB (기관생명윤리위원회) 승인 여부 확인
    return true; // 실제로는 DB 조회
  }

  private calculateDuration(start: Date, end: Date): number {
    return Math.floor((end.getTime() - start.getTime()) / (1000 * 60 * 60 * 24));
  }

  private async verifyConsent(patientId: string): Promise<boolean> {
    // 동의서 유효성 확인
    return true;
  }

  private async verifyFacilityRegistration(): Promise<boolean> {
    // 시설 등록 확인
    return true;
  }
}
```

---

**문서 버전**: 1.0
**최종 수정**: 2025-01-11
**작성자**: WIA Standards Committee

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
