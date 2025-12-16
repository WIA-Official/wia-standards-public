# Phase 4: Pet Health Passport Integration Specification

## WIA-PET-HEALTH-PASSPORT Ecosystem Integration

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
**Standard ID**: WIA-PET-HEALTH-PASSPORT-PHASE4-001

---

## 1. 개요

WIA-PET-HEALTH-PASSPORT는 WIA 대가족 생태계와 완벽히 통합되어,
반려동물과 보호자에게 최적의 경험을 제공합니다.

### 1.1 WIA 대가족 통합

```
┌──────────────────────────────────────────────────────────────────┐
│                       WIA 대가족 구조                             │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  👨 아버지 (INTENT)                                               │
│     └─→ "우리 강아지 백신 언제 맞아야 해?" → 자동 일정 안내      │
│                                                                  │
│  👩 어머니 (OMNI-API)                                             │
│     └─→ 전 세계 검역 시스템 하나로 통합                          │
│                                                                  │
│  💪 삼촌 (AIR-POWER)                                              │
│     └─→ 저전력 마이크로칩 리더기 지원                            │
│                                                                  │
│  🛡️ 이모 (AIR-SHIELD)                                            │
│     └─→ 반려동물 건강 데이터 프라이버시 보호                     │
│                                                                  │
│  🔐 사촌 (PQ-CRYPTO)                                              │
│     └─→ 양자 내성 암호화로 기록 보호                             │
│                                                                  │
│  🌐 조카 (SOCIAL)                                                 │
│     └─→ 반려동물 커뮤니티 연결                                   │
│                                                                  │
│  🏠 집 (HOME)                                                     │
│     └─→ 반려동물 건강 관리 홈페이지                              │
│                                                                  │
│  🐾 PET-HEALTH-PASSPORT                                          │
│     └─→ 전 세계 어디서든 인정되는 건강 기록                      │
│                                                                  │
│  🎫 REFUGEE-CREDENTIAL (구조 재활용)                              │
│     └─→ 분산 저장 + 동료 검증 시스템                             │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. WIA-INTENT 통합

### 2.1 음성 명령 인터페이스

```typescript
import { WIAIntent } from '@wia/intent';
import { PetPassport } from '@wia/pet-passport';

// INTENT를 통한 자연어 반려동물 관리
const intent = new WIAIntent();
const passport = new PetPassport();

// 음성 명령 처리
intent.on('command', async (cmd) => {
  switch (cmd.intent) {
    case 'pet.vaccine.check':
      // "우리 강아지 백신 언제 맞아야 해?"
      const schedule = await passport.getVaccinationSchedule();
      return formatVaccineSchedule(schedule);

    case 'pet.travel.check':
      // "일본 여행 갈 수 있어?"
      const eligibility = await passport.checkTravelEligibility({
        destination: cmd.parameters.country
      });
      return formatEligibility(eligibility);

    case 'pet.emergency.info':
      // "응급 정보 보여줘"
      const emergency = await passport.getEmergencyInfo();
      return formatEmergencyInfo(emergency);

    case 'pet.vet.find':
      // "근처 동물병원 찾아줘"
      const vets = await passport.findNearbyVets({
        location: cmd.parameters.location,
        specialty: cmd.parameters.specialty
      });
      return formatVetList(vets);
  }
});

// 예시 대화
// 사용자: "맥스 광견병 백신 유효기간이 언제까지야?"
// INTENT: { intent: 'pet.vaccine.status', parameters: { pet: 'Max', vaccine: 'rabies' } }
// 응답: "맥스의 광견병 백신은 2026년 12월 16일까지 유효합니다."
```

### 2.2 상황 인식 자동 알림

```typescript
interface ContextAwarePetCare {
  // INTENT가 감지한 상황
  context: {
    location: 'home' | 'traveling' | 'at_vet' | 'outdoor';
    activity: 'daily' | 'travel_planning' | 'emergency';
    nearbyServices: VeterinaryService[];
  };

  // 자동 알림 설정
  notifications: {
    vaccineDueReminder: boolean;      // 백신 접종일 알림
    travelDocumentReminder: boolean;  // 여행 서류 준비 알림
    medicationReminder: boolean;      // 투약 알림
    vetAppointmentReminder: boolean;  // 진료 예약 알림
  };
}

// 컨텍스트 기반 자동 알림
const contextRules: ContextRule[] = [
  {
    name: 'travel_planning_detected',
    conditions: {
      activity: 'travel_planning',
      hasUpcomingTrip: true
    },
    actions: async (passport, trip) => {
      const eligibility = await passport.checkTravelEligibility(trip);
      if (!eligibility.eligible) {
        notify('여행 준비 필요', eligibility.missingItems);
      }
    }
  },
  {
    name: 'vaccine_expiring_soon',
    conditions: {
      vaccineExpiringWithin: 30  // days
    },
    actions: async (passport, vaccine) => {
      notify('백신 만료 임박', `${vaccine.name} 백신이 ${vaccine.daysRemaining}일 후 만료됩니다`);
      suggestNearbyVets();
    }
  },
  {
    name: 'at_new_country',
    conditions: {
      location: 'traveling',
      countryChanged: true
    },
    actions: async (passport, country) => {
      const localVets = await findEmergencyVets(country);
      cacheOffline('emergency_vets', localVets);
      notify('새 국가 도착', `${country}의 응급 동물병원 정보를 저장했습니다`);
    }
  }
];
```

---

## 3. WIA-OMNI-API 통합

### 3.1 글로벌 검역 시스템 통합

```typescript
import { OmniAPI } from '@wia/omni-api';
import { PetPassport } from '@wia/pet-passport';

// 어머니(OMNI-API)가 모든 검역 시스템을 하나로
const omni = new OmniAPI();

// 각국 검역 시스템 어댑터 등록
omni.registerAdapter('eu', new EUPetPassportAdapter());
omni.registerAdapter('usda', new USDAAPHISAdapter());
omni.registerAdapter('uk', new UKDefraAdapter());
omni.registerAdapter('jp', new JapanMaffAdapter());
omni.registerAdapter('au', new AustraliaDAFFAdapter());
omni.registerAdapter('kr', new KoreaQIAAdapter());

// 통합 인터페이스
const quarantine = omni.quarantine;

// 동일한 API로 모든 국가 요구사항 조회
const requirements = await quarantine.getRequirements({
  origin: 'KR',
  destination: 'AU',
  species: 'dog'
});

// 자동 서류 생성
const documents = await quarantine.generateDocuments({
  passport: myPetPassport,
  destination: 'AU',
  travelDate: '2025-06-01'
});

// 결과: EU, USDA, DEFRA 등 형식 자동 변환
console.log(documents);
// {
//   euFormat: { ... },
//   usdaFormat: { ... },
//   australiaFormat: { ... }
// }
```

### 3.2 국가별 시스템 매핑

```typescript
interface CountrySystemMapping {
  eu: {
    system: 'TRACES NT';
    petPassportFormat: 'EU Regulation 576/2013';
    certificateType: 'ITAHC';
  };

  us: {
    system: 'USDA APHIS';
    certificateType: 'APHIS Form 7001';
    endorsement: 'Required';
  };

  uk: {
    system: 'IPAFFS';
    certificateType: 'AHC';
    postBrexit: true;
  };

  jp: {
    system: 'NACCS';
    inspectionSystem: 'AQS';
    quarantinePeriod: 180;  // 최대 일수
  };

  au: {
    system: 'BICON';
    permitSystem: 'Import Permit';
    quarantinePeriod: 10;
  };

  kr: {
    system: 'QIA';
    certificateType: '동물검역증명서';
    microchipRegistry: 'KARA';
  };
}

// WIA 형식에서 각국 형식으로 자동 변환
function convertToNationalFormat(
  wiaPassport: PetHealthPassport,
  targetCountry: ISO3166Alpha2
): NationalDocument {
  const mapping = COUNTRY_MAPPINGS[targetCountry];

  return {
    format: mapping.certificateType,
    data: transformData(wiaPassport, mapping),
    validation: validateForCountry(wiaPassport, targetCountry)
  };
}
```

---

## 4. WIA-REFUGEE-CREDENTIAL 구조 재활용

### 4.1 분산 저장 아키텍처

```typescript
// REFUGEE-CREDENTIAL의 분산 저장 구조 재활용
interface DistributedPetPassport {
  // 핵심 데이터: WIA 분산 네트워크에 저장
  core: {
    identity: PetIdentity;
    microchip: MicrochipInfo;
    rabiesVaccination: VaccinationRecord;  // 필수
  };

  // 상세 데이터: IPFS에 암호화 저장
  extended: {
    fullMedicalHistory: string;  // IPFS CID
    geneticData: string;         // IPFS CID
    photos: string[];            // IPFS CIDs
  };

  // 검증 증거: 블록체인 앵커
  verification: {
    rootHash: string;            // Merkle Root
    blockchainAnchors: BlockchainAnchor[];
  };
}

// 분산 저장의 이점
// 1. 단일 장애점 없음 (보호소/병원 폐업해도 기록 유지)
// 2. 국가 붕괴해도 데이터 보존
// 3. 오프라인에서도 핵심 정보 접근 가능
```

### 4.2 동료 검증 시스템

```typescript
// REFUGEE-CREDENTIAL의 동료 검증 재활용
interface PeerVerification {
  // 검증자 유형
  verifierTypes: {
    veterinarian: {
      trust: 'high';
      weight: 1.0;
      requirements: ['valid_license', 'direct_examination'];
    };
    shelter: {
      trust: 'medium';
      weight: 0.7;
      requirements: ['registered_shelter', 'intake_record'];
    };
    previousOwner: {
      trust: 'medium';
      weight: 0.5;
      requirements: ['identity_verified', 'ownership_record'];
    };
    community: {
      trust: 'low';
      weight: 0.3;
      requirements: ['wia_member', 'physical_verification'];
    };
  };

  // 합의 알고리즘
  consensus: {
    minimumVerifiers: 2;
    minimumTrustScore: 1.5;
    timeWindow: '30days';
  };
}

// 예시: 길에서 발견된 유기동물의 기록 재구성
async function reconstructLostPetRecord(
  microchipId: string
): Promise<ReconstructedRecord> {
  // 1. 마이크로칩으로 기존 기록 검색
  const existingRecord = await searchByMicrochip(microchipId);

  // 2. 동료 검증 요청
  const verifications = await requestPeerVerification({
    microchipId,
    photosForVerification: capturedPhotos,
    physicalCharacteristics: observedTraits
  });

  // 3. 합의 도달 시 기록 복구
  if (calculateTrustScore(verifications) >= 1.5) {
    return mergeVerifiedRecords(existingRecord, verifications);
  }

  // 4. 새 기록 생성 (검증 필요 표시)
  return createNewRecord({
    microchipId,
    verificationPending: true,
    partialData: extractFromVerifications(verifications)
  });
}
```

---

## 5. WIA-AIR-SHIELD 통합

### 5.1 반려동물 데이터 프라이버시

```typescript
import { AirShield } from '@wia/air-shield';
import { PetPassport } from '@wia/pet-passport';

// 이모(AIR-SHIELD)가 지켜줌
const shield = new AirShield();
const passport = new PetPassport();

// 프라이버시 보호 설정
shield.protect(passport, {
  // 데이터 익명화
  anonymization: {
    guardianInfo: 'pseudonymize',      // 보호자 정보 가명화
    location: 'coarse',                // 대략적 위치만
    financialData: 'remove',           // 비용 정보 제거
  },

  // 접근 제어
  access: {
    guardian: ['all'],
    veterinarian: ['medical', 'emergency'],
    shelter: ['identity', 'vaccination'],
    quarantine: ['vaccination', 'travel'],
    researcher: ['anonymized_aggregate'],
    insurance: ['with_consent']
  },

  // 데이터 공유 정책
  sharing: {
    requireConsent: true,
    logAllAccess: true,
    notifyOnAccess: ['emergency', 'transfer'],
    expireSharedLinks: '7days'
  }
});

// 접근 시 자동 보호
passport.on('access', async (event) => {
  const protected = await shield.filterData(event);
  auditLog(event);
  if (event.type === 'emergency') {
    notifyGuardian(event);
  }
  return protected;
});
```

### 5.2 동물복지 학대 방지

```typescript
interface AbusePreventionSystem {
  // 이상 패턴 감지
  anomalyDetection: {
    frequentOwnershipTransfers: {
      threshold: 3,
      period: '12months',
      action: 'flag_for_review'
    };
    repeatedInjuries: {
      threshold: 2,
      period: '6months',
      action: 'alert_authorities'
    };
    neglectedVaccinations: {
      threshold: 'core_vaccines_expired',
      action: 'welfare_check_suggestion'
    };
  };

  // 신고 시스템
  reporting: {
    anonymousReporting: true;
    evidenceAttachment: true;
    crossReferenceRecords: true;
  };

  // 블랙리스트
  blacklist: {
    bannedFromOwnership: string[];      // 소유 금지자 목록
    bannedFromBreeding: string[];       // 번식 금지자 목록
    watchList: string[];                // 감시 대상
  };
}

// 소유권 이전 시 자동 검사
async function validateOwnershipTransfer(
  passport: PetHealthPassport,
  newGuardian: GuardianInfo
): Promise<TransferValidation> {
  // 1. 신규 보호자 블랙리스트 확인
  const blacklistCheck = await checkBlacklist(newGuardian.id);
  if (blacklistCheck.banned) {
    return {
      allowed: false,
      reason: 'Guardian is banned from pet ownership',
      reportTo: 'animal_welfare_authority'
    };
  }

  // 2. 빈번한 이전 확인
  const transferHistory = passport.travel?.history || [];
  const recentTransfers = transferHistory.filter(
    t => withinMonths(t.date, 12)
  );
  if (recentTransfers.length >= 3) {
    flagForReview(passport.id, 'frequent_transfers');
  }

  // 3. 동물 상태 확인
  const recentDiagnostics = passport.medicalRecords.diagnostics
    .filter(d => withinMonths(d.performedAt, 6));
  const injuries = recentDiagnostics.filter(d => d.testType === 'injury');
  if (injuries.length >= 2) {
    alertAuthorities(passport.id, 'suspected_abuse');
  }

  return { allowed: true };
}
```

---

## 6. WIA-SOCIAL 통합

### 6.1 반려동물 커뮤니티

```typescript
import { WIASocial } from '@wia/social';
import { PetPassport } from '@wia/pet-passport';

// 조카(SOCIAL)로 커뮤니티 연결
const social = new WIASocial();
const passport = new PetPassport();

// 반려동물 프로필 생성
const petProfile = await social.createPetProfile({
  passportId: passport.id,
  publicInfo: {
    name: passport.identity.name,
    species: passport.identity.species,
    breed: passport.identity.breed,
    photos: passport.identity.photos
  },
  privacy: {
    showLocation: 'city_only',
    showAge: true,
    showHealth: false
  }
});

// 품종 커뮤니티 가입
const breedCommunity = await social.joinCommunity('golden_retriever_lovers');

// 수의사 추천 공유
await breedCommunity.share({
  type: 'vet_recommendation',
  content: {
    vetName: 'Happy Pet Clinic',
    rating: 4.8,
    specialty: 'Golden Retriever hip dysplasia expert',
    location: 'Seoul, Korea'
  }
});

// 건강 팁 공유 (익명화)
await breedCommunity.share({
  type: 'health_tip',
  content: {
    condition: 'Hip dysplasia',
    tip: 'Swimming is great exercise with low joint impact',
    source: 'My vet recommendation'
  },
  anonymous: true
});
```

### 6.2 실종 반려동물 네트워크

```typescript
interface LostPetNetwork {
  // 실종 신고
  reportLost: {
    passportId: string;
    lastSeenLocation: GeoLocation;
    lastSeenTime: ISO8601;
    photos: string[];
    description: string;
    reward?: number;
  };

  // 목격 신고
  reportSighting: {
    microchipId?: string;
    location: GeoLocation;
    time: ISO8601;
    photos: string[];
    description: string;
  };

  // 자동 매칭
  matching: {
    byMicrochip: true;
    byPhoto: true;              // AI 이미지 매칭
    byDescription: true;
    byLocation: true;           // 지리적 근접성
  };
}

// 실종 신고 시 자동 알림 범위
async function createLostPetAlert(
  passport: PetHealthPassport,
  lastSeen: GeoLocation
): Promise<Alert> {
  // 1. 반경 내 WIA 사용자에게 알림
  const nearbyUsers = await social.findUsersNear(lastSeen, '10km');
  await social.broadcast(nearbyUsers, {
    type: 'lost_pet',
    petInfo: {
      name: passport.identity.name,
      species: passport.identity.species,
      breed: passport.identity.breed,
      photos: passport.identity.photos,
      microchipId: passport.microchip?.chipNumber
    },
    lastSeen: lastSeen,
    contactMethod: 'in_app'
  });

  // 2. 주변 동물병원에 알림
  const nearbyVets = await findNearbyVets(lastSeen, '20km');
  await notifyVets(nearbyVets, passport);

  // 3. 보호소에 알림
  const nearbyShelters = await findNearbyShelters(lastSeen, '50km');
  await notifyShelters(nearbyShelters, passport);

  // 4. 마이크로칩 스캔 모니터링 등록
  await registerMicrochipAlert(passport.microchip?.chipNumber);

  return {
    alertId: generateAlertId(),
    reachCount: nearbyUsers.length + nearbyVets.length + nearbyShelters.length,
    status: 'active'
  };
}
```

---

## 7. WIA-HOME 통합

### 7.1 반려동물 건강 대시보드

```typescript
import { WIAHome } from '@wia/home';
import { PetPassport } from '@wia/pet-passport';

// 집(HOME)에서 반려동물 관리
const home = new WIAHome({
  name: 'My Pet Dashboard',
  template: 'pet_health_management'
});

// 메인 대시보드
home.addPage('dashboard', {
  widgets: [
    // 반려동물 프로필
    {
      type: 'pet_card',
      source: passport.identity,
      display: ['photo', 'name', 'age', 'breed']
    },

    // 백신 상태
    {
      type: 'vaccine_status',
      source: passport.medicalRecords.vaccinations,
      alerts: ['expiring_soon', 'overdue']
    },

    // 다음 예정 일정
    {
      type: 'upcoming_schedule',
      sources: [
        passport.getVaccinationSchedule,
        passport.getMedicationSchedule,
        passport.getVetAppointments
      ]
    },

    // 건강 점수
    {
      type: 'health_score',
      source: passport.getHealthRiskAssessment
    },

    // 응급 QR 코드
    {
      type: 'emergency_qr',
      source: passport.getEmergencyQR
    }
  ]
});

// 의료 기록 페이지
home.addPage('medical', {
  sections: [
    { type: 'vaccination_history', source: passport.medicalRecords.vaccinations },
    { type: 'surgery_history', source: passport.medicalRecords.surgeries },
    { type: 'allergy_list', source: passport.medicalRecords.allergies },
    { type: 'medication_list', source: passport.medicalRecords.medications }
  ]
});

// 여행 페이지
home.addPage('travel', {
  tools: [
    { type: 'travel_eligibility_checker', handler: passport.checkTravelEligibility },
    { type: 'document_generator', handler: passport.generateTravelDocuments },
    { type: 'vet_finder', handler: passport.findVetsByLocation }
  ]
});

await home.start();
// 🏠 mypet.wia.home 에서 반려동물 관리!
```

### 7.2 가족 공유

```typescript
interface FamilySharing {
  // 가족 구성원 역할
  roles: {
    primary_guardian: {
      permissions: ['all'];
      canTransferOwnership: true;
    };
    family_member: {
      permissions: ['read', 'schedule', 'emergency'];
      canAddRecords: false;
    };
    pet_sitter: {
      permissions: ['read_basic', 'emergency'];
      temporaryAccess: true;
      expiresAt: ISO8601;
    };
  };

  // 공유 설정
  sharing: {
    calendarSync: boolean;      // 일정 동기화
    notifications: boolean;     // 알림 공유
    locationSharing: boolean;   // 위치 공유
  };
}

// 펫시터에게 임시 접근 권한 부여
async function grantPetSitterAccess(
  passport: PetHealthPassport,
  sitter: SitterInfo,
  duration: Duration
): Promise<AccessGrant> {
  return await passport.grantAccess({
    userId: sitter.id,
    role: 'pet_sitter',
    permissions: ['read_basic', 'emergency', 'medication_reminder'],
    expiresAt: addDuration(now(), duration),
    restrictions: {
      cannotModify: true,
      cannotTransfer: true,
      cannotAccessGenetics: true
    },
    includes: {
      emergencyContacts: true,
      currentMedications: true,
      allergies: true,
      vetInfo: true,
      feedingSchedule: true
    }
  });
}
```

---

## 8. 수의사 시스템 통합

### 8.1 수의사 워크스테이션

```typescript
interface VeterinaryWorkstation {
  // 환자 관리
  patients: {
    searchByMicrochip: (chipNumber: string) => PetHealthPassport;
    searchByOwner: (guardianId: string) => PetHealthPassport[];
    recentPatients: () => PetHealthPassport[];
  };

  // 의료 기록 작성
  records: {
    addVaccination: (data: VaccinationData) => VaccinationRecord;
    addSurgery: (data: SurgeryData) => SurgeryRecord;
    addDiagnostic: (data: DiagnosticData) => DiagnosticRecord;
    prescribeMedication: (data: MedicationData) => MedicationRecord;
  };

  // 인증서 발급
  certificates: {
    issueHealthCertificate: (passportId: string, destination: string) => Certificate;
    issueTiterTest: (passportId: string, result: TiterResult) => Certificate;
    signRecord: (recordId: string) => DigitalSignature;
  };

  // WIA 통합
  wiaIntegration: {
    verifyPassport: (passportId: string) => VerificationResult;
    checkBlockchain: (recordId: string) => BlockchainVerification;
    syncWithRegistry: (passportId: string) => SyncResult;
  };
}

// 진료 워크플로우
async function veterinaryConsultation(
  microchipId: string
): Promise<ConsultationResult> {
  const workstation = new VeterinaryWorkstation();

  // 1. 마이크로칩으로 환자 조회
  const passport = await workstation.patients.searchByMicrochip(microchipId);

  // 2. 의료 이력 확인
  const history = passport.medicalRecords;
  displayMedicalHistory(history);

  // 3. 알레르기 경고
  const activeAllergies = history.allergies.filter(a => a.status === 'active');
  if (activeAllergies.length > 0) {
    showAllergyWarning(activeAllergies);
  }

  // 4. 약물 상호작용 체크
  const currentMeds = history.medications.filter(isActive);
  const interactions = checkDrugInteractions(currentMeds);
  if (interactions.length > 0) {
    showInteractionWarning(interactions);
  }

  // 5. 진료 기록 작성
  const consultation = await createConsultation({
    passportId: passport.id,
    date: now(),
    findings: /* 진료 내용 */,
    diagnosis: /* 진단 */,
    treatment: /* 치료 */
  });

  // 6. 디지털 서명
  await workstation.certificates.signRecord(consultation.id);

  // 7. 블록체인 기록
  await anchorToBlockchain(consultation);

  return consultation;
}
```

### 8.2 병원 정보 시스템(HIS) 연동

```typescript
interface HISIntegration {
  // HL7 FHIR 지원
  fhir: {
    endpoint: string;
    version: 'R4';
    resources: ['Patient', 'Observation', 'Immunization', 'Procedure'];
  };

  // 데이터 동기화
  sync: {
    direction: 'bidirectional';
    frequency: 'realtime';
    conflictResolution: 'newest_wins';
  };
}

// FHIR Patient 리소스로 반려동물 표현
const petAsFHIR: fhir.Patient = {
  resourceType: 'Patient',
  identifier: [{
    system: 'urn:wia:pet:passport',
    value: '01H5XNQK4PJXV8RQFY7D6KTHNW'
  }, {
    system: 'urn:iso:std:iso:11784',
    value: '985141234567890'
  }],
  name: [{
    text: 'Max'
  }],
  gender: 'male',
  birthDate: '2022-03-15',
  extension: [{
    url: 'http://wia.world/fhir/pet-species',
    valueCode: 'dog'
  }, {
    url: 'http://wia.world/fhir/pet-breed',
    valueString: 'Golden Retriever'
  }]
};
```

---

## 9. 정부/검역 시스템 통합

### 9.1 국가 검역 시스템 연동

```typescript
interface GovernmentIntegration {
  // 한국 QIA
  korea: {
    system: 'QIA (검역원)';
    microchipRegistry: 'KARA (동물등록)';
    importPermit: '수입검역증명서';
    exportCertificate: '수출검역증명서';
  };

  // EU TRACES
  eu: {
    system: 'TRACES NT';
    petPassport: 'EU Pet Passport';
    healthCertificate: 'ITAHC';
    identificationDocument: 'IDD';
  };

  // 미국 USDA
  us: {
    system: 'USDA APHIS';
    healthCertificate: 'APHIS Form 7001';
    endorsement: 'VS Form 7001';
  };
}

// 자동 서류 제출
async function submitToQuarantineAuthority(
  passport: PetHealthPassport,
  destination: ISO3166Alpha2,
  travelDate: ISO8601
): Promise<SubmissionResult> {
  const authority = getQuarantineAuthority(destination);

  // 1. WIA 형식에서 국가 형식으로 변환
  const nationalFormat = await convertToNationalFormat(passport, destination);

  // 2. 서류 제출
  const submission = await authority.submit({
    documents: nationalFormat,
    travelDate,
    applicant: passport.guardian
  });

  // 3. 추적 번호 저장
  await passport.addTravelDocument({
    type: 'quarantine_submission',
    referenceNumber: submission.referenceNumber,
    status: 'pending',
    destination
  });

  // 4. 상태 모니터링 등록
  await registerStatusMonitoring(submission.referenceNumber, (status) => {
    updatePassportTravelStatus(passport.id, status);
    notifyGuardian(status);
  });

  return submission;
}
```

### 9.2 마이크로칩 레지스트리 연동

```typescript
interface MicrochipRegistryIntegration {
  // 글로벌 레지스트리
  registries: {
    petlink: 'petlink.net';           // US
    europetnet: 'europetnet.com';     // EU
    petmaxx: 'petmaxx.com';           // Global
    kara: 'animal.go.kr';             // Korea
  };

  // 통합 조회
  unifiedSearch: async (chipNumber: string) => {
    const results = await Promise.all([
      searchPetlink(chipNumber),
      searchEuropetnet(chipNumber),
      searchPetmaxx(chipNumber),
      searchKARA(chipNumber)
    ]);
    return mergeResults(results);
  };
}

// 마이크로칩 등록 동기화
async function syncMicrochipRegistration(
  passport: PetHealthPassport
): Promise<SyncResult> {
  const chipNumber = passport.microchip?.chipNumber;
  if (!chipNumber) {
    return { success: false, reason: 'No microchip registered' };
  }

  // 1. 해당 국가 레지스트리에 등록
  const countryRegistry = getRegistryForCountry(passport.guardian.address.country);
  await countryRegistry.register({
    chipNumber,
    petName: passport.identity.name,
    species: passport.identity.species,
    breed: passport.identity.breed,
    guardianInfo: passport.guardian
  });

  // 2. 글로벌 레지스트리에 연동
  await globalRegistry.link({
    chipNumber,
    wiaPassportId: passport.passportId,
    primaryRegistry: countryRegistry.name
  });

  return { success: true, registries: [countryRegistry.name, 'WIA Global'] };
}
```

---

## 10. 모바일 SDK

### 10.1 iOS SDK

```swift
import WIAPetPassport

class PetPassportViewController: UIViewController {
    let passport = WIAPetPassportManager.shared

    override func viewDidLoad() {
        super.viewDidLoad()
        loadPassport()
    }

    // 여권 로드
    func loadPassport() {
        passport.load(passportId: "01H5XNQK...") { result in
            switch result {
            case .success(let pet):
                self.displayPetInfo(pet)
            case .failure(let error):
                self.showError(error)
            }
        }
    }

    // 마이크로칩 스캔
    @IBAction func scanMicrochip(_ sender: Any) {
        let scanner = MicrochipScanner()
        scanner.scan { chipNumber in
            self.passport.searchByMicrochip(chipNumber) { result in
                // 결과 표시
            }
        }
    }

    // QR 코드 스캔
    @IBAction func scanQRCode(_ sender: Any) {
        let qrScanner = QRScanner()
        qrScanner.scan { qrData in
            self.passport.loadFromQR(qrData) { result in
                // 결과 표시
            }
        }
    }

    // 여행 적격성 체크
    @IBAction func checkTravel(_ sender: Any) {
        passport.checkTravelEligibility(
            destination: "DE",
            travelDate: Date().addingDays(30)
        ) { result in
            switch result {
            case .success(let eligibility):
                if eligibility.eligible {
                    self.showSuccess("Travel approved!")
                } else {
                    self.showRequirements(eligibility.missingItems)
                }
            case .failure(let error):
                self.showError(error)
            }
        }
    }
}
```

### 10.2 Android SDK

```kotlin
import com.wia.petpassport.*

class PetPassportActivity : AppCompatActivity() {
    private lateinit var passport: WIAPetPassport

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        passport = WIAPetPassport.getInstance(this)
    }

    // NFC 마이크로칩 스캔
    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        if (NfcAdapter.ACTION_TAG_DISCOVERED == intent.action) {
            val chipNumber = passport.readMicrochip(intent)
            chipNumber?.let { searchByMicrochip(it) }
        }
    }

    // 백신 상태 확인
    fun checkVaccinationStatus() {
        passport.getVaccinationSchedule { result ->
            result.onSuccess { schedule ->
                displaySchedule(schedule)

                // 만료 임박 알림
                schedule.expiringSoon.forEach { vaccine ->
                    showNotification(
                        "Vaccine Expiring",
                        "${vaccine.name} expires in ${vaccine.daysRemaining} days"
                    )
                }
            }
        }
    }

    // 응급 정보 위젯
    fun setupEmergencyWidget() {
        passport.getEmergencyInfo { emergency ->
            EmergencyWidget.update(this, emergency)
        }
    }
}
```

### 10.3 Flutter SDK

```dart
import 'package:wia_pet_passport/wia_pet_passport.dart';

class PetPassportScreen extends StatefulWidget {
  @override
  _PetPassportScreenState createState() => _PetPassportScreenState();
}

class _PetPassportScreenState extends State<PetPassportScreen> {
  final passport = WIAPetPassport();
  PetHealthPassport? currentPassport;

  @override
  void initState() {
    super.initState();
    _loadPassport();
  }

  Future<void> _loadPassport() async {
    final result = await passport.load('01H5XNQK...');
    setState(() => currentPassport = result);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text('Pet Passport')),
      body: currentPassport == null
        ? CircularProgressIndicator()
        : SingleChildScrollView(
            child: Column(
              children: [
                // 반려동물 정보 카드
                PetInfoCard(pet: currentPassport!.identity),

                // 백신 상태
                VaccineStatusCard(
                  vaccinations: currentPassport!.medicalRecords.vaccinations,
                ),

                // QR 코드
                EmergencyQRCard(passport: currentPassport!),

                // 빠른 액션
                QuickActions(
                  onCheckTravel: () => _checkTravel(),
                  onFindVet: () => _findNearbyVet(),
                  onEmergency: () => _showEmergencyInfo(),
                ),
              ],
            ),
          ),
    );
  }

  Future<void> _checkTravel() async {
    final destination = await showCountryPicker(context);
    if (destination != null) {
      final result = await passport.checkTravelEligibility(
        passportId: currentPassport!.passportId,
        destination: destination,
        travelDate: DateTime.now().add(Duration(days: 30)),
      );

      showEligibilityResult(context, result);
    }
  }
}
```

---

## 11. IoT 기기 통합

### 11.1 스마트 펫 기기 연동

```typescript
interface SmartPetDeviceIntegration {
  // 지원 기기
  devices: {
    smartFeeder: {
      feedingSchedule: true;
      portionControl: true;
      dietaryRestrictions: true;  // 알레르기 연동
    };
    activityTracker: {
      dailyActivity: true;
      sleepPatterns: true;
      healthMetrics: true;
    };
    smartLitter: {
      usagePatterns: true;
      healthIndicators: true;    // 소변 분석
    };
    petCamera: {
      behaviorMonitoring: true;
      emergencyDetection: true;
    };
  };

  // 건강 데이터 연동
  healthSync: {
    activityToPassport: true;
    weightTracking: true;
    anomalyDetection: true;
  };
}

// 스마트 급식기 연동
async function syncSmartFeeder(
  passport: PetHealthPassport,
  feeder: SmartFeeder
): Promise<void> {
  // 알레르기 정보 동기화
  const allergies = passport.medicalRecords.allergies
    .filter(a => a.allergenType === 'FOOD')
    .map(a => a.allergen);

  await feeder.setDietaryRestrictions(allergies);

  // 약물 투약 시간에 맞춘 급식
  const medications = passport.medicalRecords.medications
    .filter(m => m.dosage.route === 'oral' && needsFood(m));

  for (const med of medications) {
    await feeder.addFeedingReminder({
      time: med.dosage.time,
      portion: 'small',
      reason: `Medication: ${med.medication.name}`
    });
  }
}

// 활동 추적기 연동
async function syncActivityTracker(
  passport: PetHealthPassport,
  tracker: ActivityTracker
): Promise<void> {
  // 건강 상태에 따른 활동 목표 설정
  const conditions = passport.medicalRecords.conditions
    .filter(c => c.status === 'active');

  let activityGoal = 'normal';

  if (conditions.some(c => c.conditionName.includes('Obesity'))) {
    activityGoal = 'high';
  } else if (conditions.some(c => c.conditionName.includes('Arthritis'))) {
    activityGoal = 'low_impact';
  }

  await tracker.setActivityGoal(activityGoal);

  // 활동 데이터를 여권에 기록
  tracker.on('daily_summary', async (summary) => {
    await passport.addDiagnostic({
      testType: 'activity_monitoring',
      results: [
        { parameter: 'steps', value: summary.steps },
        { parameter: 'active_minutes', value: summary.activeMinutes },
        { parameter: 'rest_quality', value: summary.restQuality }
      ],
      performedAt: summary.date
    });
  });
}
```

### 11.2 NFC 펫 태그

```typescript
interface NFCPetTagIntegration {
  // 태그 유형
  tagTypes: {
    basic: {
      storage: '144 bytes';
      data: ['passport_url', 'emergency_phone'];
    };
    standard: {
      storage: '888 bytes';
      data: ['passport_url', 'emergency_info', 'allergies', 'medications'];
    };
    premium: {
      storage: '4KB';
      data: ['full_emergency_profile', 'encrypted_medical_summary'];
    };
  };

  // 기능
  features: {
    offlineAccess: true;        // 오프라인에서도 기본 정보 접근
    tamperEvident: true;        // 위변조 감지
    updateable: true;           // 정보 업데이트 가능
  };
}

// NFC 태그 쓰기
async function writePetTag(
  passport: PetHealthPassport,
  tagType: 'basic' | 'standard' | 'premium'
): Promise<NFCWriteResult> {
  const nfcData = await generateNFCPayload(passport, tagType);

  return await nfcWriter.write({
    records: [
      // Record 1: 여권 URL
      {
        type: 'U',
        payload: `https://pet.wia.world/p/${passport.passportId}`
      },
      // Record 2: 응급 정보 (오프라인)
      {
        type: 'T',
        payload: JSON.stringify({
          name: passport.identity.name,
          species: passport.identity.species,
          allergies: passport.medicalRecords.allergies
            .filter(a => a.status === 'active' && a.reactionSeverity !== 'mild')
            .map(a => a.allergen),
          emergencyPhone: passport.guardian.emergencyContacts[0]?.phone
        })
      },
      // Record 3: 서명
      {
        type: 'application/wia-signature',
        payload: await signData(nfcData)
      }
    ]
  });
}
```

---

## 12. 인증 및 배지

### 12.1 WIA 인증 조건

```typescript
interface WIACertification {
  // 인증 등급
  levels: {
    bronze: {
      requirements: [
        'complete_vaccination',
        'microchip_registered',
        'basic_profile'
      ];
      badge: 'WIA Pet Passport - Bronze';
    };
    silver: {
      requirements: [
        'bronze_requirements',
        'annual_checkup',
        'no_overdue_vaccines',
        'emergency_info_complete'
      ];
      badge: 'WIA Pet Passport - Silver';
    };
    gold: {
      requirements: [
        'silver_requirements',
        'genetic_testing',
        'all_records_verified',
        'travel_ready'
      ];
      badge: 'WIA Pet Passport - Gold';
    };
  };

  // 특별 인증
  special: {
    travel_certified: {
      requirements: ['meets_international_requirements'];
      validDestinations: ISO3166Alpha2[];
    };
    therapy_animal: {
      requirements: ['behavior_certification', 'health_clearance'];
    };
    breeding_certified: {
      requirements: ['genetic_clear', 'health_screening', 'registered_breeder'];
    };
  };
}

// 인증 상태 확인
async function checkCertificationStatus(
  passport: PetHealthPassport
): Promise<CertificationStatus> {
  const status = {
    currentLevel: null as string | null,
    progress: {} as Record<string, boolean>,
    nextLevel: null as string | null,
    missingRequirements: [] as string[]
  };

  // Bronze 체크
  const bronzeReqs = {
    complete_vaccination: hasCompleteVaccination(passport),
    microchip_registered: !!passport.microchip,
    basic_profile: hasBasicProfile(passport)
  };

  if (Object.values(bronzeReqs).every(v => v)) {
    status.currentLevel = 'bronze';

    // Silver 체크
    const silverReqs = {
      annual_checkup: hasRecentCheckup(passport),
      no_overdue_vaccines: !hasOverdueVaccines(passport),
      emergency_info_complete: hasCompleteEmergencyInfo(passport)
    };

    if (Object.values(silverReqs).every(v => v)) {
      status.currentLevel = 'silver';

      // Gold 체크
      const goldReqs = {
        genetic_testing: !!passport.genetics,
        all_records_verified: allRecordsVerified(passport),
        travel_ready: await isTravelReady(passport)
      };

      if (Object.values(goldReqs).every(v => v)) {
        status.currentLevel = 'gold';
      } else {
        status.nextLevel = 'gold';
        status.missingRequirements = Object.entries(goldReqs)
          .filter(([_, v]) => !v)
          .map(([k, _]) => k);
      }
    } else {
      status.nextLevel = 'silver';
      status.missingRequirements = Object.entries(silverReqs)
        .filter(([_, v]) => !v)
        .map(([k, _]) => k);
    }
  } else {
    status.nextLevel = 'bronze';
    status.missingRequirements = Object.entries(bronzeReqs)
      .filter(([_, v]) => !v)
      .map(([k, _]) => k);
  }

  return status;
}
```

---

## 13. 로드맵

### 13.1 구현 계획

```
2025 Q1:
├── Phase 1-4 스펙 완성
├── Rust API 핵심 기능
├── 기본 모바일 SDK
└── 한국 QIA 연동

2025 Q2:
├── EU TRACES 연동
├── 글로벌 마이크로칩 레지스트리 통합
├── 수의사 워크스테이션 v1
└── iOS/Android 앱 출시

2025 Q3:
├── USDA APHIS 연동
├── 스마트 펫 기기 연동
├── AI 건강 위험 예측
└── 실종 동물 네트워크

2025 Q4:
├── 일본/호주 검역 시스템 연동
├── 보험사 연동
├── 동물복지 단체 연동
└── 글로벌 출시
```

---

**Document ID**: WIA-PET-HEALTH-PASSPORT-PHASE4-001
**Version**: 1.0.0
**Last Updated**: 2025-12-16
**Copyright**: © 2025 WIA - MIT License

**홍익인간 (弘益人間)** - 인류와 동물 모두를 이롭게 하라
