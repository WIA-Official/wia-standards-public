# 제6장: 운영 프로토콜

## 학습 목표

이 장을 완료하면 다음을 수행할 수 있습니다:
- 전자폐기물 수거 및 운송 프로토콜 설계
- 관리 연속성(Chain of Custody) 문서화 구현
- 처리 시설 운영 절차 수립
- 품질 보증 및 감사 프로토콜 적용
- 한국 EPR 제도 연계 운영 절차 이해

---

## 6.1 전자폐기물 수거 프로토콜

### 6.1.1 수거 채널 유형

WIA 전자폐기물 관리 표준은 다양한 수거 채널을 정의합니다:

```typescript
// 수거 채널 정의
enum CollectionChannelType {
  PRODUCER_TAKEBACK = "producer_takeback",      // 생산자 직접 회수
  RETAILER_COLLECTION = "retailer_collection",  // 판매점 수거
  MUNICIPAL_CENTER = "municipal_center",        // 지자체 수거센터
  MOBILE_COLLECTION = "mobile_collection",      // 이동 수거
  DOORSTEP_PICKUP = "doorstep_pickup",          // 방문 수거
  MAIL_BACK = "mail_back",                      // 우편 반송
  CONSORTIUM_NETWORK = "consortium_network"     // 공제조합 네트워크
}

interface CollectionChannel {
  channelId: string;
  channelType: CollectionChannelType;
  operator: {
    id: string;
    name: string;
    license: string;
    eprRegistration?: string;
  };
  location: {
    address: string;
    coordinates: { lat: number; lng: number };
    serviceArea: string[];  // 서비스 지역 목록
  };
  acceptedCategories: WasteCategory[];
  operatingHours: {
    dayOfWeek: string;
    openTime: string;
    closeTime: string;
  }[];
  capacity: {
    dailyLimit: number;
    currentUtilization: number;
    unit: "kg" | "units";
  };
  certifications: string[];
}
```

### 6.1.2 수거 요청 프로토콜

```typescript
// 수거 요청 처리 프로토콜
interface CollectionRequestProtocol {
  // 1단계: 요청 접수
  requestSubmission: {
    channels: ("web" | "mobile" | "phone" | "api")[];
    requiredInfo: {
      contactInfo: ContactInfo;
      deviceInfo: DeviceDescription[];
      preferredSchedule: SchedulePreference;
      locationInfo: PickupLocation;
    };
    validation: {
      addressVerification: boolean;
      categoryValidation: boolean;
      quantityCheck: boolean;
    };
  };

  // 2단계: 요청 평가
  requestEvaluation: {
    automaticApproval: {
      maxQuantity: number;
      allowedCategories: WasteCategory[];
      serviceAreaCheck: boolean;
    };
    manualReview: {
      triggers: string[];
      reviewerRole: string;
      maxResponseTime: string;  // ISO 8601 duration
    };
  };

  // 3단계: 일정 확정
  scheduling: {
    confirmationMethod: ("sms" | "email" | "app_push")[];
    reminderSchedule: string[];  // e.g., ["24h", "2h"]
    reschedulingAllowed: boolean;
    cancellationPolicy: {
      freeUntil: string;  // ISO 8601 duration before pickup
      fee?: number;
    };
  };

  // 4단계: 수거 실행
  collection: {
    identification: {
      verifyAddress: boolean;
      verifyContact: boolean;
      photoRequired: boolean;
    };
    inspection: {
      visualCheck: boolean;
      categoryVerification: boolean;
      hazardAssessment: boolean;
    };
    documentation: {
      receiptGeneration: boolean;
      weightRecording: boolean;
      signatureCapture: boolean;
    };
  };
}
```

### 6.1.3 한국 수거 네트워크 연계

```typescript
// 한국 EPR 수거 네트워크 연계
interface KoreaCollectionNetwork {
  // 전자제품자원순환공제조합 네트워크
  ecoasNetwork: {
    collectionCenters: {
      centerId: string;
      name: string;
      address: string;
      region: string;  // 광역시/도
      acceptedItems: string[];
      operatingHours: string;
      contactNumber: string;
    }[];

    mobileCollection: {
      scheduleUrl: string;
      regions: string[];
      bookingRequired: boolean;
    };

    retailerPartners: {
      retailerId: string;
      retailerName: string;
      storeCount: number;
      acceptedCategories: string[];
      incentiveProgram?: string;
    }[];
  };

  // 지자체 수거 시스템
  municipalSystem: {
    largeBulkWaste: {
      applicationMethod: "online" | "phone" | "visit";
      feeStructure: {
        category: string;
        sizeRange: string;
        fee: number;
      }[];
      processingTime: string;
    };

    designatedCollectionDays: {
      dayOfWeek: string;
      items: string[];
      preparationGuidelines: string;
    }[];

    dropOffLocations: {
      name: string;
      address: string;
      acceptedItems: string[];
    }[];
  };

  // 생산자 직접 회수
  producerTakeback: {
    programs: {
      producerId: string;
      producerName: string;
      programName: string;
      targetProducts: string[];
      incentives?: {
        type: "discount" | "voucher" | "points";
        value: number;
        conditions: string;
      };
      registrationUrl: string;
    }[];
  };
}

// 수거 채널 선택 알고리즘
function selectOptimalCollectionChannel(
  request: CollectionRequest,
  availableChannels: CollectionChannel[],
  userPreferences: UserPreferences
): CollectionChannelRecommendation {
  const recommendations: ChannelScore[] = [];

  for (const channel of availableChannels) {
    let score = 0;

    // 카테고리 적합성
    const categoryMatch = request.devices.every(d =>
      channel.acceptedCategories.includes(d.category)
    );
    if (!categoryMatch) continue;

    // 거리 점수 (최대 30점)
    const distance = calculateDistance(request.location, channel.location);
    score += Math.max(0, 30 - distance * 2);

    // 편의성 점수 (최대 25점)
    if (channel.channelType === "doorstep_pickup") score += 25;
    else if (channel.channelType === "retailer_collection") score += 20;
    else if (channel.channelType === "municipal_center") score += 15;

    // 인센티브 점수 (최대 20점)
    if (channel.incentiveProgram) {
      score += channel.incentiveProgram.value / 1000 * 20;
    }

    // 가용성 점수 (최대 15점)
    const utilization = channel.capacity.currentUtilization / channel.capacity.dailyLimit;
    score += (1 - utilization) * 15;

    // 사용자 선호도 점수 (최대 10점)
    if (userPreferences.preferredChannelTypes?.includes(channel.channelType)) {
      score += 10;
    }

    recommendations.push({ channel, score });
  }

  recommendations.sort((a, b) => b.score - a.score);

  return {
    primary: recommendations[0]?.channel,
    alternatives: recommendations.slice(1, 4).map(r => r.channel),
    reasoning: generateRecommendationReasoning(recommendations[0])
  };
}
```

---

## 6.2 운송 프로토콜

### 6.2.1 운송 차량 요구사항

```typescript
// 전자폐기물 운송 차량 사양
interface TransportVehicleSpecification {
  vehicleClass: "small" | "medium" | "large" | "specialized";

  requirements: {
    // 기본 요구사항
    enclosedCargo: boolean;
    weatherProtection: boolean;
    secureStorage: boolean;

    // 안전 장비
    safetyEquipment: {
      fireExtinguisher: { type: string; quantity: number };
      spillKit: boolean;
      firstAidKit: boolean;
      ppe: string[];
    };

    // 추적 장비
    tracking: {
      gps: boolean;
      temperatureMonitoring?: boolean;
      tamperDetection?: boolean;
    };

    // 적재 장비
    loadingEquipment: {
      liftGate?: boolean;
      palletJack?: boolean;
      secureStrapping: boolean;
    };
  };

  capacityLimits: {
    maxWeight: number;  // kg
    maxVolume: number;  // cubic meters
    maxUnits: number;
  };

  certifications: {
    vehicleInspection: {
      date: string;
      validUntil: string;
      inspector: string;
    };
    hazmatCertification?: {
      classes: string[];
      validUntil: string;
    };
  };
}

// 운송 분류
interface TransportClassification {
  domesticTransport: {
    standard: {
      maxDistance: number;  // km
      vehicleClass: string[];
      documentation: string[];
      transitTime: { min: number; max: number; unit: "hours" };
    };

    hazardous: {
      batteriesOnly: {
        packaging: string;
        labeling: string[];
        documentation: string[];
        driverCertification: string;
      };

      mercuryContaining: {
        packaging: string;
        containerType: string;
        documentation: string[];
        specialHandling: string[];
      };
    };
  };

  internationalTransport: {
    baselConvention: {
      notificationRequired: boolean;
      consentProcedure: string;
      documentation: string[];
      prohibitedDestinations: string[];
    };

    oecdDecision: {
      greenList: string[];
      amberList: string[];
      controlProcedures: string;
    };
  };
}
```

### 6.2.2 운송 문서화

```typescript
// 운송 매니페스트 (적하목록)
interface TransportManifest {
  manifestId: string;
  manifestType: "collection" | "transfer" | "export";

  shipment: {
    origin: {
      facilityId: string;
      facilityName: string;
      address: string;
      contactPerson: string;
      contactPhone: string;
    };

    destination: {
      facilityId: string;
      facilityName: string;
      address: string;
      contactPerson: string;
      contactPhone: string;
      facilityType: "collection" | "consolidation" | "processing" | "recycling";
    };

    transporter: {
      companyId: string;
      companyName: string;
      vehicleId: string;
      vehiclePlate: string;
      driverId: string;
      driverName: string;
      driverLicense: string;
    };
  };

  cargo: {
    items: {
      itemId: string;
      description: string;
      category: WasteCategory;
      quantity: number;
      unit: "units" | "kg" | "pallets";
      packaging: string;
      hazardClass?: string;
    }[];

    totals: {
      totalItems: number;
      totalWeight: number;
      totalVolume: number;
    };
  };

  schedule: {
    plannedDeparture: string;  // ISO 8601
    plannedArrival: string;
    actualDeparture?: string;
    actualArrival?: string;
    route?: string;
  };

  signatures: {
    shipper: {
      name: string;
      signature: string;  // base64 or reference
      timestamp: string;
    };
    transporter: {
      name: string;
      signature: string;
      timestamp: string;
    };
    receiver?: {
      name: string;
      signature: string;
      timestamp: string;
      discrepancyNotes?: string;
    };
  };

  // 한국 규정 준수
  koreaCompliance: {
    allbaro: {
      reportingRequired: boolean;
      reportId?: string;
      reportTimestamp?: string;
    };

    wasteControlNumber?: string;  // 폐기물관리번호

    eprTracking?: {
      producerId: string;
      obligationCategory: string;
    };
  };
}

// 운송 추적 이벤트
interface TransportTrackingEvent {
  eventId: string;
  manifestId: string;
  timestamp: string;

  eventType:
    | "departed_origin"
    | "in_transit"
    | "checkpoint_passed"
    | "delay_reported"
    | "incident_reported"
    | "arrived_destination"
    | "unloading_started"
    | "unloading_completed"
    | "manifest_signed";

  location: {
    coordinates: { lat: number; lng: number };
    address?: string;
    checkpoint?: string;
  };

  details: {
    description: string;
    reportedBy: string;
    photos?: string[];

    delay?: {
      reason: string;
      estimatedDuration: string;
      newEta?: string;
    };

    incident?: {
      type: "accident" | "spill" | "theft" | "damage" | "other";
      severity: "low" | "medium" | "high" | "critical";
      description: string;
      actionsTaken: string[];
      authoritiesNotified?: boolean;
    };
  };
}
```

---

## 6.3 처리 시설 운영 프로토콜

### 6.3.1 입고 절차

```typescript
// 시설 입고 프로토콜
interface FacilityReceivingProtocol {
  // 1단계: 도착 확인
  arrivalCheck: {
    manifestVerification: boolean;
    vehicleInspection: boolean;
    appointmentConfirmation: boolean;

    procedures: {
      checkIn: {
        location: string;
        documentation: string[];
        waitingArea: string;
      };

      inspection: {
        visualInspection: boolean;
        radiationCheck?: boolean;
        photoDocumentation: boolean;
      };
    };
  };

  // 2단계: 하역 및 검수
  unloadingInspection: {
    supervision: {
      requiredPersonnel: string[];
      safetyBriefing: boolean;
    };

    inspection: {
      quantityVerification: boolean;
      conditionAssessment: boolean;
      categoryValidation: boolean;
      discrepancyHandling: {
        threshold: number;  // percentage
        escalationProcedure: string;
        documentationRequired: string[];
      };
    };

    handling: {
      equipmentRequired: string[];
      safetyPrecautions: string[];
      segregationRules: {
        byCategory: boolean;
        byHazardClass: boolean;
        byProcessingPriority: boolean;
      };
    };
  };

  // 3단계: 등록 및 저장
  registrationStorage: {
    registration: {
      systemEntry: boolean;
      labelingRequired: boolean;
      labelFormat: string;
      barcodeGeneration: boolean;
    };

    storage: {
      zoneAssignment: string;
      stackingRules: string;
      maxStorageDuration: string;  // ISO 8601 duration
      environmentalControls?: {
        temperature?: { min: number; max: number };
        humidity?: { min: number; max: number };
        ventilation: boolean;
      };
    };

    documentation: {
      receivingReport: boolean;
      photographicEvidence: boolean;
      systemRecordId: string;
    };
  };
}

// 검수 체크리스트
interface ReceivingChecklist {
  checklistId: string;
  manifestId: string;
  inspector: string;
  inspectionTimestamp: string;

  checks: {
    documentation: {
      manifestPresent: boolean;
      manifestComplete: boolean;
      signaturesValid: boolean;
      regulatoryDocuments: boolean;
    };

    quantity: {
      declaredQuantity: number;
      actualQuantity: number;
      variance: number;
      varianceAcceptable: boolean;
    };

    condition: {
      packagingIntact: boolean;
      noVisibleDamage: boolean;
      noLeaksOrSpills: boolean;
      labelingIntact: boolean;
    };

    category: {
      categoriesMatchManifest: boolean;
      prohibitedItemsFound: boolean;
      hazardousItemsIdentified: boolean;
      unknownItemsFound: boolean;
    };

    safety: {
      radiationCheckPassed?: boolean;
      noObviousHazards: boolean;
      ppeUsedCorrectly: boolean;
    };
  };

  findings: {
    discrepancies: {
      type: string;
      description: string;
      severity: "minor" | "major" | "critical";
      resolution: string;
    }[];

    prohibitedItems?: {
      itemDescription: string;
      actionTaken: string;
    }[];

    photos: {
      photoId: string;
      description: string;
      timestamp: string;
    }[];
  };

  approval: {
    accepted: boolean;
    acceptedWithConditions?: string[];
    rejected?: {
      reason: string;
      returnAuthorized: boolean;
    };
    approverSignature: string;
    approvalTimestamp: string;
  };
}
```

### 6.3.2 처리 작업 프로토콜

```typescript
// 처리 작업 지시
interface ProcessingWorkOrder {
  workOrderId: string;
  batchId: string;
  facilityId: string;

  input: {
    items: {
      itemId: string;
      deviceId?: string;  // WDID if available
      category: WasteCategory;
      quantity: number;
      sourceManifest: string;
      storageLocation: string;
    }[];

    totalWeight: number;
    estimatedRecoverables: {
      material: string;
      estimatedWeight: number;
    }[];
  };

  processing: {
    processingType: "manual_disassembly" | "mechanical_shredding" | "combined";

    workstations: {
      stationId: string;
      stationType: string;
      assignedPersonnel: string[];
      equipmentRequired: string[];
    }[];

    procedures: {
      stepNumber: number;
      description: string;
      duration: string;
      qualityCheckpoint?: boolean;
      safetyPrecautions?: string[];
    }[];

    outputTargets: {
      material: string;
      targetQuantity: number;
      qualityStandard: string;
      destinationBin: string;
    }[];
  };

  schedule: {
    scheduledStart: string;
    scheduledEnd: string;
    priority: "low" | "normal" | "high" | "urgent";
  };

  safety: {
    requiredPPE: string[];
    hazardsIdentified: string[];
    emergencyProcedure: string;
    firstAidLocation: string;
  };

  qualityControl: {
    inspectionPoints: {
      pointId: string;
      description: string;
      criteria: string;
      method: string;
      frequency: string;
    }[];

    samplingRequirements?: {
      sampleSize: number;
      samplingMethod: string;
      testsRequired: string[];
    };
  };
}

// 처리 결과 보고
interface ProcessingResult {
  workOrderId: string;
  completionTimestamp: string;

  input: {
    actualWeight: number;
    itemCount: number;
  };

  output: {
    recoveredMaterials: {
      material: string;
      weight: number;
      purity: number;
      qualityGrade: string;
      destinationBin: string;
      marketValue?: number;
    }[];

    residuals: {
      type: string;
      weight: number;
      disposalMethod: string;
      disposalFacility?: string;
    }[];

    hazardousExtracted: {
      substance: string;
      weight: number;
      containerType: string;
      storageLocation: string;
      disposalRequired: boolean;
    }[];
  };

  metrics: {
    totalInputWeight: number;
    totalRecoveredWeight: number;
    recoveryRate: number;  // percentage
    processingTime: string;  // ISO 8601 duration
    energyConsumed: number;  // kWh
    laborHours: number;
  };

  quality: {
    inspectionResults: {
      checkpointId: string;
      passed: boolean;
      notes?: string;
    }[];

    nonConformances?: {
      type: string;
      description: string;
      correctiveAction: string;
    }[];
  };

  personnel: {
    supervisorId: string;
    operators: string[];
    qualityInspector: string;
  };

  certification: {
    supervisorApproval: {
      name: string;
      signature: string;
      timestamp: string;
    };

    qualityApproval: {
      name: string;
      signature: string;
      timestamp: string;
    };
  };
}
```

### 6.3.3 유해물질 처리 프로토콜

```typescript
// 유해물질 관리 프로토콜
interface HazardousMaterialProtocol {
  // 배터리 처리
  batteryHandling: {
    identification: {
      types: ("lithium_ion" | "lithium_polymer" | "nickel_cadmium" | "nickel_metal_hydride" | "lead_acid")[];
      visualInspection: string[];
      testingRequired: boolean;
    };

    riskAssessment: {
      damageInspection: boolean;
      swellingCheck: boolean;
      leakageCheck: boolean;
      temperatureMonitoring: boolean;
    };

    handling: {
      ppe: string[];
      tools: string[];
      handlingProcedures: string[];
      prohibitedActions: string[];
    };

    storage: {
      containerType: string;
      fillMaterial: string;  // e.g., "vermiculite", "sand"
      temperatureLimit: number;
      segregation: string;
      inspectionFrequency: string;
    };

    emergencyResponse: {
      thermalRunaway: string[];
      fire: string[];
      spill: string[];
      exposure: string[];
    };
  };

  // 수은 함유 장치
  mercuryDevices: {
    types: ("lcd_backlights" | "fluorescent_lamps" | "switches" | "batteries")[];

    handling: {
      containment: string;
      breakageProtocol: string;
      personalProtection: string[];
    };

    storage: {
      containerType: string;
      labelingRequired: string[];
      maxStorageTime: string;
    };

    disposal: {
      licensedFacility: boolean;
      documentationRequired: string[];
    };
  };

  // CRT (브라운관) 처리
  crtHandling: {
    hazards: {
      leadContent: string;
      vacuumImplosion: boolean;
      phosphorDust: boolean;
    };

    processing: {
      safetyMeasures: string[];
      cuttingMethod: string;
      glassSegregation: {
        funnelGlass: string;
        panelGlass: string;
      };
    };
  };

  // 냉매 (CFCs, HCFCs, HFCs)
  refrigerantHandling: {
    equipment: ("refrigerators" | "air_conditioners" | "dehumidifiers")[];

    extraction: {
      certificationRequired: string;
      equipmentRequired: string[];
      procedure: string[];
    };

    storage: {
      cylinderType: string;
      pressureLimits: { min: number; max: number };
      temperatureLimits: { min: number; max: number };
    };

    disposal: {
      destructionMethod: string;
      licensedFacility: boolean;
      documentationRequired: string[];
    };

    koreaRegulation: {
      montrealProtocol: boolean;
      domesticReporting: string;
      authorizedFacilities: string[];
    };
  };
}

// 유해물질 사고 대응
interface HazardousIncidentResponse {
  incidentId: string;
  timestamp: string;
  location: string;

  incident: {
    type: "spill" | "fire" | "explosion" | "exposure" | "release";
    substance: string;
    quantity: string;
    affectedArea: string;
    personnelInvolved: number;
  };

  immediateResponse: {
    evacuation: boolean;
    containment: string[];
    notification: {
      facilityManager: boolean;
      emergencyServices: boolean;
      environmentalAuthority: boolean;  // 환경부
      occupationalSafety: boolean;  // 고용노동부
    };
    firstAid: string[];
  };

  cleanup: {
    method: string[];
    equipmentUsed: string[];
    wasteGenerated: {
      type: string;
      quantity: string;
      disposal: string;
    }[];
    verification: string;
  };

  investigation: {
    rootCause: string;
    contributingFactors: string[];
    correctiveActions: string[];
    preventiveMeasures: string[];
  };

  reporting: {
    internalReport: {
      submittedTo: string;
      submissionDate: string;
    };
    regulatoryReport?: {
      authority: string;
      reportId: string;
      submissionDate: string;
    };
  };
}
```

---

## 6.4 관리 연속성 (Chain of Custody) 프로토콜

### 6.4.1 전체 추적 체계

```typescript
// 관리 연속성 기록
interface ChainOfCustodyRecord {
  recordId: string;
  deviceId: string;  // WDID

  custodyChain: {
    sequence: number;
    custodian: {
      entityId: string;
      entityName: string;
      entityType: "producer" | "distributor" | "collector" | "transporter" | "processor" | "recycler";
      facilityId?: string;
      licenseNumber?: string;
    };

    custody: {
      receivedFrom: string;
      receivedTimestamp: string;
      receivedCondition: string;

      releasedTo?: string;
      releasedTimestamp?: string;
      releasedCondition?: string;
    };

    activities: {
      activityType: string;
      description: string;
      timestamp: string;
      outcome?: string;
    }[];

    documentation: {
      manifestId?: string;
      receiptId?: string;
      photos?: string[];
      signatures?: {
        name: string;
        role: string;
        signature: string;
        timestamp: string;
      }[];
    };

    verification: {
      method: "visual" | "barcode" | "rfid" | "blockchain";
      verifiedBy: string;
      verificationTimestamp: string;
      discrepancies?: string;
    };
  }[];

  currentStatus: {
    custodian: string;
    location: string;
    condition: string;
    lastUpdate: string;
  };

  finalDisposition?: {
    dispositionType: "recycled" | "recovered" | "disposed" | "exported";
    dispositionDate: string;
    certificate?: string;
    materialRecovery?: {
      material: string;
      quantity: number;
    }[];
  };
}

// 관리 연속성 검증
interface CustodyVerification {
  verificationId: string;
  deviceId: string;
  timestamp: string;

  verification: {
    method: "manual" | "automated" | "audit";
    verifier: {
      id: string;
      name: string;
      role: string;
    };

    checks: {
      identityConfirmed: boolean;
      documentationComplete: boolean;
      timelineConsistent: boolean;
      noUnauthorizedGaps: boolean;
      conditionTracked: boolean;
    };

    findings: {
      compliant: boolean;
      gaps?: {
        description: string;
        timeframe: string;
        severity: "minor" | "major" | "critical";
      }[];
      recommendations?: string[];
    };
  };

  certification?: {
    certificationId: string;
    issuedBy: string;
    validUntil: string;
    scope: string;
  };
}
```

### 6.4.2 블록체인 기반 추적 (선택적)

```typescript
// 블록체인 추적 레코드
interface BlockchainCustodyRecord {
  transactionId: string;
  blockNumber: number;
  timestamp: string;

  device: {
    wdid: string;
    deviceHash: string;  // SHA-256 of device attributes
  };

  event: {
    eventType: CustodyEventType;
    from: {
      entityId: string;
      entityHash: string;
    };
    to: {
      entityId: string;
      entityHash: string;
    };

    metadata: {
      manifestHash?: string;
      locationHash?: string;
      conditionCode?: string;
      additionalData?: Record<string, string>;
    };
  };

  signatures: {
    sender: string;
    receiver: string;
  };

  verification: {
    previousTransactionId: string;
    merkleRoot: string;
    consensusValidated: boolean;
  };
}

enum CustodyEventType {
  PRODUCED = "PRODUCED",
  SOLD = "SOLD",
  COLLECTED = "COLLECTED",
  TRANSPORTED = "TRANSPORTED",
  RECEIVED = "RECEIVED",
  PROCESSED = "PROCESSED",
  MATERIAL_RECOVERED = "MATERIAL_RECOVERED",
  DISPOSED = "DISPOSED",
  EXPORTED = "EXPORTED"
}

// 블록체인 조회 인터페이스
interface BlockchainQueryInterface {
  // 장치 이력 조회
  getDeviceHistory(wdid: string): Promise<BlockchainCustodyRecord[]>;

  // 특정 이벤트 검증
  verifyEvent(transactionId: string): Promise<{
    verified: boolean;
    blockConfirmations: number;
    timestamp: string;
  }>;

  // 관리 연속성 인증서 생성
  generateCertificate(wdid: string): Promise<{
    certificateId: string;
    issueTimestamp: string;
    custodyChainHash: string;
    verificationUrl: string;
  }>;
}
```

---

## 6.5 품질 보증 프로토콜

### 6.5.1 품질 관리 체계

```typescript
// 품질 관리 프로그램
interface QualityManagementProgram {
  facilityId: string;
  programVersion: string;
  effectiveDate: string;

  standards: {
    certifications: {
      iso9001: boolean;
      iso14001: boolean;
      r2: boolean;
      eStewards: boolean;
      koreaEnvironmentMark?: boolean;
    };

    internalStandards: {
      standardId: string;
      name: string;
      version: string;
      applicableAreas: string[];
    }[];
  };

  processes: {
    incomingInspection: {
      frequency: "every_shipment" | "sampling";
      samplingRate?: number;
      criteria: string[];
      responsibleRole: string;
    };

    inProcessControl: {
      checkpoints: {
        name: string;
        location: string;
        frequency: string;
        criteria: string[];
        method: string;
      }[];
    };

    outgoingInspection: {
      materialTypes: string[];
      qualityCriteria: Record<string, {
        parameter: string;
        specification: string;
        testMethod: string;
      }[]>;
    };
  };

  documentation: {
    requiredRecords: string[];
    retentionPeriod: string;
    storageMethod: "electronic" | "paper" | "both";
    accessControl: string[];
  };

  continuousImprovement: {
    reviewFrequency: string;
    performanceIndicators: string[];
    feedbackMechanisms: string[];
    correctiveActionProcess: string;
  };
}

// 품질 검사 결과
interface QualityInspectionResult {
  inspectionId: string;
  inspectionType: "incoming" | "in_process" | "outgoing" | "audit";
  timestamp: string;
  inspector: {
    id: string;
    name: string;
    certification?: string;
  };

  subject: {
    type: "shipment" | "batch" | "material" | "product";
    identifier: string;
    description: string;
  };

  results: {
    criteria: {
      criterionId: string;
      description: string;
      specification: string;
      actualValue: string | number;
      passed: boolean;
      notes?: string;
    }[];

    overallResult: "pass" | "conditional_pass" | "fail";

    deviations?: {
      criterionId: string;
      deviation: string;
      severity: "minor" | "major" | "critical";
      disposition: "accept" | "rework" | "reject";
    }[];
  };

  disposition: {
    decision: "accept" | "conditional_accept" | "hold" | "reject";
    conditions?: string[];
    followUpRequired?: boolean;
    followUpActions?: string[];
  };

  signatures: {
    inspector: string;
    reviewer?: string;
    approver?: string;
  };
}
```

### 6.5.2 감사 프로토콜

```typescript
// 내부 감사 프로토콜
interface InternalAuditProtocol {
  auditId: string;
  auditType: "scheduled" | "unscheduled" | "follow_up";
  scope: string[];

  planning: {
    objectives: string[];
    criteria: string[];
    schedule: {
      plannedStart: string;
      plannedEnd: string;
    };
    auditTeam: {
      leadAuditor: {
        id: string;
        name: string;
        qualifications: string[];
      };
      teamMembers: {
        id: string;
        name: string;
        role: string;
      }[];
    };
    resourceRequirements: string[];
  };

  execution: {
    activities: {
      activity: string;
      date: string;
      participants: string[];
      findings: string;
    }[];

    evidenceCollected: {
      type: "document" | "interview" | "observation" | "record";
      description: string;
      reference: string;
    }[];
  };

  findings: {
    conformities: {
      area: string;
      description: string;
      evidence: string;
    }[];

    nonConformities: {
      ncId: string;
      area: string;
      description: string;
      evidence: string;
      severity: "minor" | "major";
      rootCause?: string;
      correctiveAction: string;
      dueDate: string;
      responsible: string;
    }[];

    observations: {
      area: string;
      description: string;
      recommendation: string;
    }[];

    opportunities: {
      area: string;
      description: string;
      potentialBenefit: string;
    }[];
  };

  conclusion: {
    summary: string;
    overallAssessment: "effective" | "partially_effective" | "ineffective";
    recommendations: string[];
    followUpAuditRequired: boolean;
  };

  approval: {
    preparedBy: { name: string; date: string; signature: string };
    reviewedBy: { name: string; date: string; signature: string };
    approvedBy: { name: string; date: string; signature: string };
  };
}

// 외부 감사 대응
interface ExternalAuditPreparation {
  auditType: "certification" | "regulatory" | "customer" | "insurance";
  auditor: {
    organization: string;
    auditorNames: string[];
    scheduledDate: string;
  };

  preparation: {
    documentationReview: {
      documentsRequired: string[];
      lastUpdated: string;
      gaps: string[];
    };

    facilityPreparation: {
      areasToBeAudited: string[];
      cleanupRequired: string[];
      equipmentCalibration: string[];
      safetyCheck: string[];
    };

    personnelBriefing: {
      sessionDate: string;
      topics: string[];
      attendees: string[];
    };

    previousFindings: {
      findingId: string;
      description: string;
      correctiveAction: string;
      status: "open" | "closed";
      evidence: string;
    }[];
  };

  duringAudit: {
    escorts: {
      area: string;
      escort: string;
      alternateEscort: string;
    }[];

    documentControl: {
      designatedRoom: string;
      documentCustodian: string;
      copyPolicy: string;
    };

    issueEscalation: {
      firstLevel: string;
      secondLevel: string;
      emergencyContact: string;
    };
  };

  postAudit: {
    responsePlan: {
      findingsReviewMeeting: string;
      correctionActionDays: number;
      appealProcess?: string;
    };
  };
}
```

---

## 6.6 한국 규제 준수 프로토콜

### 6.6.1 올바로 시스템 연계

```typescript
// 올바로 시스템 연계 프로토콜
interface AllbaroIntegrationProtocol {
  // 시스템 개요
  systemInfo: {
    name: "올바로시스템";
    operator: "한국환경공단";
    purpose: "폐기물 인계·인수 및 처리 실적 전자 신고";
    legalBasis: "폐기물관리법 제36조의2";
  };

  // 사업장 등록
  businessRegistration: {
    requiredFor: [
      "폐기물 배출사업장",
      "폐기물 수집운반업체",
      "폐기물 처리업체"
    ];
    registrationProcess: {
      application: string;
      requiredDocuments: string[];
      approvalTime: string;
      renewalPeriod: string;
    };
    userAccount: {
      adminAccount: boolean;
      operatorAccounts: number;
      accessControl: string;
    };
  };

  // 전자인계서 작성
  electronicManifest: {
    workflow: {
      step1: {
        actor: "배출사업장";
        action: "인계서 작성 및 발행";
        deadline: "인계 전";
      };
      step2: {
        actor: "수집운반업체";
        action: "인계서 수령 확인";
        deadline: "인계시";
      };
      step3: {
        actor: "처리업체";
        action: "인수 확인";
        deadline: "인수시";
      };
      step4: {
        actor: "처리업체";
        action: "처리 완료 보고";
        deadline: "처리 완료 후 즉시";
      };
    };

    requiredFields: {
      폐기물정보: {
        폐기물종류: string;
        폐기물코드: string;
        성상: string;
        수량: number;
        단위: string;
      };
      배출자정보: {
        사업장명: string;
        사업자번호: string;
        주소: string;
        담당자: string;
      };
      운반자정보: {
        업체명: string;
        허가번호: string;
        차량번호: string;
        운전자: string;
      };
      처리자정보: {
        업체명: string;
        허가번호: string;
        처리방법: string;
      };
    };
  };

  // API 연동
  apiIntegration: {
    connectionMethod: "웹서비스 (SOAP/REST)";
    authenticationMethod: "공인인증서 / API Key";

    endpoints: {
      인계서발행: string;
      인계서조회: string;
      실적보고: string;
      통계조회: string;
    };

    dataFormat: "XML";

    errorHandling: {
      retryPolicy: string;
      manualFallback: string;
      supportContact: string;
    };
  };

  // 보고 의무
  reportingObligations: {
    daily: {
      인계인수: boolean;
      deadline: string;
    };
    monthly: {
      처리실적: boolean;
      deadline: string;
    };
    annual: {
      사업장실적: boolean;
      deadline: string;
    };
  };
}

// 올바로 전자인계서 데이터 구조
interface AllbaroManifestData {
  // 인계서 기본정보
  인계서번호: string;
  작성일자: string;
  인계일자: string;

  // 폐기물 정보
  폐기물: {
    종류코드: string;
    종류명: string;
    성상: "고체" | "액체" | "슬러지" | "기타";
    수량: number;
    단위: "kg" | "ton" | "L" | "㎥";
  };

  // 관계자 정보
  배출자: {
    사업장관리번호: string;
    사업장명: string;
    대표자: string;
    사업자등록번호: string;
    주소: string;
    연락처: string;
    담당자: string;
  };

  수집운반자: {
    허가번호: string;
    업체명: string;
    대표자: string;
    사업자등록번호: string;
    주소: string;
    연락처: string;
    차량번호: string;
    운전자: string;
  };

  처리자: {
    허가번호: string;
    업체명: string;
    대표자: string;
    사업자등록번호: string;
    주소: string;
    연락처: string;
    처리방법: string;
    처리시설: string;
  };

  // 전자서명
  전자서명: {
    배출자서명: string;
    배출일시: string;
    수집운반자서명: string;
    수령일시: string;
    처리자서명: string;
    인수일시: string;
    처리완료서명?: string;
    처리완료일시?: string;
  };
}
```

### 6.6.2 EPR 실적 보고 프로토콜

```typescript
// EPR 실적 보고 프로토콜
interface EPRReportingProtocol {
  // 보고 주체
  reportingEntity: {
    entityType: "생산자" | "수입자";
    registrationNumber: string;  // EPR 등록번호
    consortium?: {
      name: string;
      membershipNumber: string;
    };
  };

  // 보고 주기
  reportingSchedule: {
    quarterly: {
      출고량보고: {
        deadline: "분기 종료 후 45일 이내";
        submitTo: "한국환경공단";
        method: "EPR 통합관리시스템";
      };
    };

    annual: {
      재활용실적보고: {
        deadline: "다음 연도 3월 31일까지";
        submitTo: "한국환경공단";
        method: "EPR 통합관리시스템";
      };

      분담금정산: {
        deadline: "다음 연도 6월 30일까지";
        submitTo: "해당 공제조합";
        method: "공제조합 시스템";
      };
    };
  };

  // 보고 데이터
  reportData: {
    출고량: {
      기간: { 시작: string; 종료: string };
      품목별: {
        품목코드: string;
        품목명: string;
        출고량: number;
        단위: string;
        의무재활용률: number;
      }[];
    };

    재활용실적: {
      기간: { 시작: string; 종료: string };
      처리업체별: {
        업체명: string;
        허가번호: string;
        처리방법: string;
        처리량: number;
        인증서번호?: string;
      }[];

      품목별요약: {
        품목코드: string;
        의무량: number;
        이행량: number;
        이행률: number;
        과부족: number;
      }[];
    };
  };

  // 검증 및 인증
  verification: {
    selfVerification: {
      checklist: string[];
      signOff: string;
    };

    thirdPartyVerification?: {
      verifier: string;
      verificationDate: string;
      certificateNumber: string;
    };

    regulatoryAudit?: {
      auditDate: string;
      auditor: string;
      findings: string[];
    };
  };
}

// EPR 분담금 계산
interface EPRFeeCalculation {
  calculationPeriod: string;
  producer: {
    id: string;
    name: string;
    eprNumber: string;
  };

  calculation: {
    품목별: {
      품목코드: string;
      품목명: string;
      출고량_kg: number;
      의무재활용률: number;
      의무량_kg: number;
      자체이행량_kg: number;
      공제조합이행량_kg: number;
      미이행량_kg: number;
    }[];

    분담금: {
      기본분담금: number;
      변동분담금: number;
      가산금?: number;
      감면액?: number;
      총분담금: number;
    };

    정산: {
      기납부액: number;
      추가납부액: number;
      환급액: number;
      최종정산액: number;
    };
  };

  납부정보: {
    납부기한: string;
    납부계좌: string;
    납부방법: string[];
  };
}
```

---

## 6.7 요약 및 핵심 사항

### 핵심 요약

| 프로토콜 영역 | 핵심 요소 | 한국 특화 사항 |
|--------------|----------|---------------|
| 수거 | 채널 다양화, 스케줄링 | 공제조합 네트워크, 대형폐기물 신고 |
| 운송 | 매니페스트, GPS 추적 | 올바로 시스템 연계 |
| 입고 | 검수 체크리스트, 등록 | 폐기물관리번호 부여 |
| 처리 | 작업지시서, 결과보고 | 유해물질 관리 기준 |
| 관리연속성 | 전체 추적, 검증 | 블록체인 선택적 적용 |
| 품질보증 | 검사, 감사 | R2/e-Stewards 인증 |
| 규제준수 | 보고, 실적관리 | EPR 분담금 정산 |

### 운영 체크리스트

**일일 운영:**
- [ ] 수거 요청 처리 및 스케줄링
- [ ] 운송 매니페스트 발행/수령
- [ ] 입고 검수 및 등록
- [ ] 처리 작업 실행 및 기록
- [ ] 올바로 시스템 인계인수 신고

**주간 운영:**
- [ ] 재고 현황 점검
- [ ] 품질 검사 결과 검토
- [ ] 유해물질 저장 상태 점검
- [ ] 운송 차량 점검

**월간 운영:**
- [ ] 처리 실적 집계 및 보고
- [ ] 품질 지표 분석
- [ ] 내부 감사 (해당시)
- [ ] 교육 훈련 실시

**분기 운영:**
- [ ] EPR 출고량 보고
- [ ] 관리 연속성 검증
- [ ] 프로토콜 검토 및 갱신
- [ ] 비상 대응 훈련

---

## 복습 질문

1. WIA 전자폐기물 수거 채널 유형 중 "공제조합 네트워크"의 역할과 장점은 무엇인가요?

2. 운송 매니페스트에 포함되어야 하는 필수 정보와 한국 규정 준수를 위한 추가 항목은 무엇인가요?

3. 시설 입고 프로토콜의 3단계 절차를 설명하고, 각 단계에서 수행되는 주요 활동은 무엇인가요?

4. 리튬이온 배터리 처리 시 준수해야 할 안전 프로토콜과 비상 대응 절차는 무엇인가요?

5. 관리 연속성(Chain of Custody) 기록의 목적과 블록체인 기반 추적의 이점은 무엇인가요?

6. 올바로 시스템의 전자인계서 작성 워크플로우를 단계별로 설명하세요.

7. EPR 실적 보고 프로토콜에서 분기별 보고와 연간 보고의 차이점은 무엇인가요?

---

## 다음 장 예고

제7장에서는 **시스템 통합**을 다룹니다. 한국 환경부 시스템, 공제조합 시스템, 기업 ERP와의 연계 방법과 데이터 동기화 전략을 학습합니다.

---

*WIA 전자폐기물 관리 표준 - 제6장 완료*
