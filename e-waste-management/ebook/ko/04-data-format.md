# 제4장: 데이터 형식 사양

## 학습 목표

이 장을 완료하면 다음을 수행할 수 있습니다:

1. 제품 식별을 위한 WIA 기기 ID (WDID) 스키마 구현
2. 재료 구성 데이터 구조 설계
3. 생애주기 이벤트 기록 생성
4. 관리 연속성 문서 구축
5. 규정 준수 보고 형식 생성

---

## 4.1 기기 식별 스키마

### 4.1.1 WIA 기기 ID (WDID) 구조

WIA 기기 ID는 전자기기의 전체 생애주기에 걸쳐 보편적 식별을 제공합니다:

```typescript
// WIA 기기 ID 구조
interface WIADeviceId {
  // 형식: WIA-[생산자]-[범주]-[연도]-[일련번호]-[검증]
  // 예시: WIA-SAMSUNG-SM-2024-A1B2C3D4-7X

  prefix: "WIA";                    // 표준 접두사
  producerCode: string;             // 3-8자 제조사 코드
  categoryCode: string;             // 2-4자 범주 코드
  productionYear: number;           // 4자리 연도
  serialNumber: string;             // 8-12자 고유 식별자
  checkDigit: string;               // 2자 검증 코드
}

// 범주 코드
const CategoryCodes = {
  // IT 및 통신
  SM: "스마트폰",
  TB: "태블릿",
  LP: "노트북",
  DT: "데스크톱 컴퓨터",
  MN: "모니터",
  PR: "프린터",
  NW: "네트워크 장비",
  SV: "서버",

  // 소비자 가전
  TV: "텔레비전",
  AU: "오디오 장비",
  GM: "게임 콘솔",
  CM: "카메라",

  // 대형 가전
  RF: "냉장고",
  WM: "세탁기",
  DW: "식기세척기",
  AC: "에어컨",
  OV: "오븐",

  // 소형 가전
  VC: "청소기",
  KA: "주방 가전",
  PC: "개인 관리",

  // 조명
  LE: "LED 조명",
  FL: "형광등",

  // 배터리
  LI: "리튬이온 배터리",
  LA: "납축전지",
  NC: "니켈카드뮴 배터리"
};
```

### 4.1.2 기기 마스터 레코드 스키마

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia-standards.org/e-waste/device-master.json",
  "title": "WIA 전자폐기물 기기 마스터 레코드",
  "type": "object",
  "required": ["deviceId", "producer", "category", "production", "materials"],
  "properties": {
    "deviceId": {
      "type": "string",
      "pattern": "^WIA-[A-Z0-9]{3,8}-[A-Z]{2,4}-[0-9]{4}-[A-Z0-9]{8,12}-[A-Z0-9]{2}$",
      "description": "WIA 기기 ID"
    },
    "producer": {
      "type": "object",
      "required": ["id", "name", "country"],
      "properties": {
        "id": {"type": "string", "description": "WIA 생산자 레지스트리 ID"},
        "name": {"type": "string"},
        "country": {"type": "string", "pattern": "^[A-Z]{2}$"},
        "brand": {"type": "string"},
        "model": {"type": "string"},
        "modelNumber": {"type": "string"}
      }
    },
    "category": {
      "type": "object",
      "required": ["wiaCategory", "koreaEPRCategory"],
      "properties": {
        "wiaCategory": {"type": "string"},
        "koreaEPRCategory": {"type": "string", "description": "한국 EPR 품목 분류"},
        "productType": {"type": "string"},
        "subCategory": {"type": "string"}
      }
    },
    "production": {
      "type": "object",
      "required": ["date", "facility"],
      "properties": {
        "date": {"type": "string", "format": "date"},
        "facility": {"type": "string"},
        "country": {"type": "string", "pattern": "^[A-Z]{2}$"},
        "batchNumber": {"type": "string"},
        "serialNumber": {"type": "string"}
      }
    },
    "physical": {
      "type": "object",
      "properties": {
        "weightKg": {"type": "number", "minimum": 0},
        "dimensionsCm": {
          "type": "object",
          "properties": {
            "length": {"type": "number"},
            "width": {"type": "number"},
            "height": {"type": "number"}
          }
        }
      }
    },
    "materials": {
      "$ref": "#/$defs/materialComposition"
    },
    "hazardous": {
      "$ref": "#/$defs/hazardousSubstances"
    },
    "recyclability": {
      "type": "object",
      "properties": {
        "score": {"type": "integer", "minimum": 0, "maximum": 100},
        "grade": {"type": "string", "enum": ["A", "B", "C", "D", "F"]},
        "disassemblyTime": {"type": "integer", "description": "분"},
        "recyclablePercent": {"type": "number"},
        "recoverablePercent": {"type": "number"}
      }
    }
  }
}
```

---

## 4.2 재료 구성 스키마

### 4.2.1 재료 분류 구조

```typescript
// 재료 구성 인터페이스
interface MaterialComposition {
  // 요약
  totalWeightKg: number;
  measurementMethod: "actual" | "calculated" | "estimated";
  measurementDate: string;
  measurementUncertainty: number;  // 백분율

  // 재료 분류
  materials: MaterialEntry[];

  // 집계 범주
  aggregates: {
    metals: {
      ferrous: number;           // kg
      nonFerrousBase: number;    // kg (구리, 알루미늄, 아연)
      preciousMetals: number;    // kg (금, 은, 팔라듐, 백금)
      rareEarth: number;         // kg
    };
    plastics: {
      total: number;
      byType: PlasticBreakdown[];
    };
    glass: number;
    ceramics: number;
    wood: number;
    other: number;
  };

  // 구성요소
  components: ComponentEntry[];
}

interface MaterialEntry {
  materialId: string;            // WIA 재료 코드
  materialName: string;
  category: MaterialCategory;
  weightKg: number;
  percentage: number;
  location: string;              // 기기 내 위치
  recyclable: boolean;
  recoveryMethod: string;        // 권장 회수 방법
  hazardous: boolean;
  notes?: string;
}

interface ComponentEntry {
  componentId: string;
  componentType: string;         // 배터리, PCB, 디스플레이 등
  description: string;
  weightKg: number;
  materials: MaterialEntry[];
  removable: boolean;
  removalDifficulty: "easy" | "medium" | "difficult" | "destructive";
  hazardousContent: boolean;
  specialHandling?: string;
}
```

### 4.2.2 재료 구성 JSON 예시

```json
{
  "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
  "materialComposition": {
    "totalWeightKg": 0.187,
    "measurementMethod": "actual",
    "measurementDate": "2024-01-15",
    "measurementUncertainty": 3.0,

    "aggregates": {
      "metals": {
        "ferrous": 0.025,
        "nonFerrousBase": 0.042,
        "preciousMetals": 0.00035,
        "rareEarth": 0.0008
      },
      "plastics": {
        "total": 0.028,
        "byType": [
          {"type": "PC", "weightKg": 0.015},
          {"type": "ABS", "weightKg": 0.008},
          {"type": "TPU", "weightKg": 0.005}
        ]
      },
      "glass": 0.035,
      "ceramics": 0.002,
      "other": 0.054
    },

    "components": [
      {
        "componentId": "BAT-001",
        "componentType": "battery",
        "description": "리튬이온 배터리 팩",
        "weightKg": 0.048,
        "materials": [
          {"materialId": "LI-001", "materialName": "리튬", "weightKg": 0.003},
          {"materialId": "CO-001", "materialName": "코발트", "weightKg": 0.008},
          {"materialId": "AL-001", "materialName": "알루미늄", "weightKg": 0.012},
          {"materialId": "CU-001", "materialName": "구리", "weightKg": 0.006}
        ],
        "removable": false,
        "removalDifficulty": "destructive",
        "hazardousContent": true,
        "specialHandling": "리튬 배터리 취급 프로토콜 필요"
      },
      {
        "componentId": "PCB-001",
        "componentType": "pcb",
        "description": "메인 로직 보드",
        "weightKg": 0.025,
        "materials": [
          {"materialId": "AU-001", "materialName": "금", "weightKg": 0.00025},
          {"materialId": "AG-001", "materialName": "은", "weightKg": 0.0008},
          {"materialId": "PD-001", "materialName": "팔라듐", "weightKg": 0.00002},
          {"materialId": "CU-001", "materialName": "구리", "weightKg": 0.008}
        ],
        "removable": true,
        "removalDifficulty": "medium",
        "hazardousContent": true,
        "specialHandling": "브롬화 난연제 포함 가능"
      }
    ]
  }
}
```

### 4.2.3 재료 코드 참조

| 코드 | 재료 | 범주 | 회수 방법 |
|-----|-----|-----|---------|
| AU-001 | 금 | 귀금속 | 습식제련 |
| AG-001 | 은 | 귀금속 | 습식제련 |
| PD-001 | 팔라듐 | 귀금속 | 습식제련 |
| PT-001 | 백금 | 귀금속 | 습식제련 |
| CU-001 | 구리 | 비철금속 | 건식제련 |
| AL-001 | 알루미늄 | 비철금속 | 제련 |
| FE-001 | 철/강철 | 철금속 | 자기 분리 |
| PB-001 | 납 | 유해금속 | 전문 처리 |
| HG-001 | 수은 | 유해금속 | 증류 |
| CD-001 | 카드뮴 | 유해금속 | 전문 처리 |
| LI-001 | 리튬 | 배터리 재료 | 습식제련 |
| CO-001 | 코발트 | 배터리 재료 | 습식제련 |
| ND-001 | 네오디뮴 | 희토류 | 전문 분리 |

---

## 4.3 생애주기 이벤트 기록

### 4.3.1 이벤트 스키마

```typescript
// 수거 이벤트 데이터
interface CollectionEventData {
  collectionMethod: "drop_off" | "pickup" | "retail_return" | "municipal";
  collectionPointId: string;
  collectionPointType: string;

  deviceCondition: {
    functional: boolean;
    physicalCondition: "excellent" | "good" | "fair" | "poor" | "damaged";
    completeness: "complete" | "missing_accessories" | "missing_parts";
    batteryPresent: boolean;
    dataWiped: boolean | "unknown";
  };

  weight: {
    measured: number;
    unit: "kg";
    method: "scale" | "estimated";
  };

  initialRouting: "reuse" | "refurbishment" | "recycling" | "assessment_needed";

  consumerInfo?: {
    consentGiven: boolean;
    recyclingCertificateRequested: boolean;
    anonymousCollection: boolean;
  };
}

// 처리 이벤트 데이터
interface ProcessingEventData {
  processType: "disassembly" | "shredding" | "separation" | "refining";
  facilityId: string;
  processLine: string;

  input: {
    batchId: string;
    weightKg: number;
    itemCount?: number;
    categories: string[];
  };

  output: OutputFraction[];

  processMetrics: {
    duration: number;           // 분
    energyUsed: number;         // kWh
    waterUsed: number;          // 리터
    chemicalsUsed?: ChemicalUsage[];
  };

  qualityControl: {
    samplingDone: boolean;
    contaminationLevel: number; // 백분율
    qualityGrade: string;
  };
}

// 재료 회수 이벤트 데이터
interface MaterialRecoveryEventData {
  recoveryProcess: string;
  inputFractions: string[];    // 분획 ID

  recoveredMaterials: {
    materialCode: string;
    materialName: string;
    weightKg: number;
    purity: number;
    qualityCertification?: string;
    buyer?: string;
    pricePerKg?: number;
  }[];

  residuals: {
    type: string;
    weightKg: number;
    disposalMethod: string;
    disposalFacility?: string;
  }[];

  recoveryRate: {
    overall: number;           // 백분율
    byMaterial: {material: string; rate: number}[];
  };

  environmentalMetrics: {
    co2Avoided: number;        // kg CO2e
    energySaved: number;       // kWh
    waterRecycled: number;     // 리터
  };
}
```

---

## 4.4 규정 준수 보고 형식

### 4.4.1 한국 EPR 규정 준수 보고서

```typescript
// 생산자 연간 규정 준수 보고서
interface KoreaEPRComplianceReport {
  reportId: string;
  reportingPeriod: {
    start: string;  // "2024-01-01"
    end: string;    // "2024-12-31"
  };
  submissionDate: string;
  submissionTo: "한국환경공단";

  producer: {
    id: string;
    name: string;
    registrationNumber: string;
    eprRegistration: string;
  };

  출고량: {
    byCategory: {
      category: string;        // "냉장고", "세탁기" 등
      koreaEPRCode: string;    // 한국 EPR 품목 코드
      quantity: number;
      weightKg: number;
    }[];
    totalQuantity: number;
    totalWeightKg: number;
  };

  의무이행: {
    byCategory: {
      category: string;
      obligationRate: number;   // 의무비율 (%)
      obligationKg: number;     // 의무량
      collectedKg: number;      // 수거량
      achievedRate: number;     // 달성률 (%)
      compliant: boolean;
    }[];
  };

  재활용실적: {
    byCategory: {
      category: string;
      recycledKg: number;
      recoveredMaterials: {
        material: string;
        weightKg: number;
      }[];
      recoveryRate: number;
    }[];
  };

  분담금: {
    totalPaid: number;
    currency: "KRW";
    paymentDates: string[];
    consortium: string;        // 공제조합명
  };

  인증: {
    certifiedBy: string;
    certificationDate: string;
    auditorName: string;
    auditorRegistration: string;
    nextAuditDue: string;
  };
}
```

---

## 4.5 복습 문제

### 문제 1
2025년에 삼성에서 제조한 노트북 컴퓨터에 대해 일련번호가 XY789012인 WIA 기기 ID를 설계하시오. ID의 각 구성요소를 설명하시오.

### 문제 2
스마트폰의 무게가 187g이고 금 0.25mg, 은 0.8mg, 리튬 3g, 코발트 8g을 포함합니다. 이 기기에 대한 재료 구성 항목을 생성하시오.

### 문제 3
소매 반납 지점에서 수거되는 텔레비전에 대한 완전한 생애주기 이벤트 기록을 작성하시오. 모든 필수 필드와 적절한 이벤트 데이터를 포함하시오.

### 문제 4
수거업체에서 처리업체로 이전되는 500개의 혼합 IT 기기 배치(850kg)에 대한 관리 연속성 문서를 설계하시오.

### 문제 5
생산자가 한국 시장에 10만 대의 스마트폰(15,000kg)을 출시하고 12,000kg (80%)을 수거했습니다. 목표 달성을 보여주는 한국 EPR 규정 준수 보고서 요약을 생성하시오.

---

## 4.6 핵심 요약

| 데이터 유형 | 핵심 스키마 요소 | 주요 용도 |
|-----------|---------------|---------|
| 기기 ID (WDID) | 생산자-범주-연도-일련번호-검증 | 보편적 식별 |
| 재료 구성 | 구성요소, 재료, 유해물질 | 회수 계획 |
| 생애주기 이벤트 | 이벤트 유형, 행위자, 타임스탬프, 체인 링크 | 추적성 |
| 관리 연속성 | 이전자/피이전자, 품목, 운송, 서명 | 법적 준수 |
| 규정 준수 보고서 | 출고량, 수거량, 회수량, 재정 | 규제 보고 |

### 스키마 설계 원칙
- **고유 식별**: 모든 기기 추적 가능
- **구성 보존**: 재료 데이터가 기기와 함께 이동
- **이벤트 연결**: 생산에서 회수까지 끊김 없는 체인
- **검증 지원**: 모든 기록에 증거 첨부 가능
- **형식 유연성**: JSON/XML 상호 교환 가능

### 다음 장 미리보기

제5장에서는 인증, 엔드포인트, 통합 패턴을 포함하여 WIA 전자폐기물 관리 시스템과 상호작용하기 위한 API 인터페이스 사양을 다룹니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
