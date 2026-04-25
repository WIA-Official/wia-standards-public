# Phase 4: WIA Ecosystem Integration
## 반려동물 감정 통합 표준 (Pet Emotion Integration Standard)

---

**버전 (Version)**: 1.0.0
**상태 (Status)**: Draft
**날짜 (Date)**: 2025-12-18
**작성자 (Authors)**: WIA Standards Committee
**라이선스 (License)**: MIT
**주요 색상 (Primary Color)**: #F59E0B (Amber)

---

## 목차 (Table of Contents)

1. [개요](#개요-overview)
2. [WIA 표준 통합](#wia-표준-통합-wia-standards-integration)
3. [반려동물 케어 생태계](#반려동물-케어-생태계-pet-care-ecosystem)
4. [수의학 통합](#수의학-통합-veterinary-integration)
5. [스마트홈 통합](#스마트홈-통합-smart-home-integration)
6. [AI 모델 통합](#ai-모델-통합-ai-model-integration)
7. [제3자 서비스](#제3자-서비스-third-party-services)
8. [데이터 교환](#데이터-교환-data-exchange)
9. [참조 구현](#참조-구현-reference-implementations)
10. [배포 예제](#배포-예제-deployment-examples)

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

Phase 4는 WIA Pet Emotion Standard가 더 넓은 WIA 생태계, 외부 반려동물 케어 시스템, 수의학 서비스, 스마트홈 플랫폼 및 제3자 애플리케이션과 어떻게 통합되는지를 정의합니다.

**통합 목표 (Integration Goals)**:
- 다른 WIA 표준과의 원활한 상호운용성
- 수의학 건강 기록 통합
- 감정 기반 스마트홈 자동화
- AI/ML 모델 마켓플레이스 연결
- 제3자 앱 생태계
- 클라우드 서비스 통합

### 1.2 통합 아키텍처 (Integration Architecture)

```
┌─────────────────────────────────────────────────────────────┐
│                  WIA Pet Emotion 핵심                        │
│              (Phases 1-3: 데이터, API, 프로토콜)              │
└────────────────────┬────────────────────────────────────────┘
                     │
        ┌────────────┼────────────┐
        │            │            │
        ▼            ▼            ▼
┌──────────────┬──────────────┬──────────────┐
│ WIA 표준     │ 반려동물      │ 외부         │
│ 생태계       │ 케어 시스템   │ 플랫폼       │
└──────────────┴──────────────┴──────────────┘
        │            │            │
        ▼            ▼            ▼
┌──────────────┬──────────────┬──────────────┐
│ Pet Health   │ 수의 병원     │ 스마트홈     │
│ Pet Genome   │ 반려동물 상점 │ IoT 장치     │
│ Pet Care Bot │ 훈련사        │ 클라우드 AI  │
│ Pet Legacy   │              │ 분석         │
└──────────────┴──────────────┴──────────────┘
```

### 1.3 통합 수준 (Integration Levels)

| 수준 (Level) | 설명 (Description) | 예시 (Example) |
|------------|-------------------|--------------|
| **데이터 (Data)** | 감정 데이터 형식 공유 | 수의학 시스템으로 감정 기록 내보내기 |
| **API** | 프로그래밍 통합 | 모바일 앱이 감정 API 호출 |
| **프로토콜 (Protocol)** | 실시간 통신 | 스마트홈 허브로 감정 스트리밍 |
| **워크플로 (Workflow)** | 비즈니스 프로세스 통합 | 감정 알림이 수의사 예약 트리거 |
| **분석 (Analytics)** | 시스템 간 인사이트 | 감정과 건강 기록 상관관계 분석 |

---

## 2. WIA 표준 통합 (WIA Standards Integration)

### 2.1 WIA-PET-HEALTH-PASSPORT 통합

감정 데이터를 포괄적인 건강 기록과 연결합니다.

#### 2.1.1 데이터 연결 (Data Linking)

```typescript
interface PetEmotionHealthLink {
  // 감정 기록 (Emotion record)
  emotionRecordId: string;
  petId: string;

  // 건강 여권으로의 링크 (Link to health passport)
  healthPassportId: string;  // WIA-PET-HEALTH-PASSPORT ID
  linkType: 'reference' | 'embedded';

  // 맥락적 건강 데이터 (Contextual health data)
  healthContext: {
    // 최근 의료 이벤트
    recentVaccinations?: VaccinationRecord[];
    recentSurgeries?: SurgeryRecord[];
    activeMedications?: MedicationRecord[];
    activeConditions?: ChronicCondition[];

    // 건강 지표
    weight?: number;
    lastVetVisit?: ISO8601;
    nextVetVisit?: ISO8601;
  };

  // 상관관계 분석 (Correlation analysis)
  correlation?: {
    emotionHealthCorrelation: number;  // -1 ~ 1
    significantFindings: string[];
    recommendations: string[];
  };
}
```

#### 2.1.2 감정-건강 상관관계 (Emotion-Health Correlation)

```typescript
interface EmotionHealthCorrelation {
  petId: string;
  analysisId: string;
  timeRange: TimeRange;

  // 발견된 상관관계 (Correlations found)
  correlations: {
    healthFactor: 'medication' | 'condition' | 'pain' | 'surgery' | 'diet';
    healthFactorId: string;
    emotionImpact: {
      emotion: CoreEmotion;
      correlation_coefficient: number;  // 상관계수
      p_value: number;
      significant: boolean;
    }[];

    // 시간적 관계 (Temporal relationship)
    lag_time?: number;  // milliseconds
    description: string;
  }[];

  // 인사이트 (Insights)
  insights: {
    insight_type: 'behavioral_change' | 'stress_indicator' | 'pain_detection';
    confidence: number;
    description: string;
    actionable: boolean;
    recommendation?: string;
  }[];
}
```

#### 2.1.3 통합 예제 (Integration Example)

```typescript
import { PetEmotion } from 'wia-pet-emotion';
import { PetHealthPassport } from 'wia-pet-health-passport';

// 두 시스템 초기화 (Initialize both systems)
const emotionSystem = new PetEmotion();
const healthSystem = new PetHealthPassport();

// 반려동물 기록 연결 (Link pet records)
const petId = 'PET-001';
const passportId = await healthSystem.getPassportId(petId);

// 건강 맥락과 함께 감정 데이터 가져오기 (Get emotion data with health context)
const emotionData = await emotionSystem.getCurrentEmotion(petId);
const healthData = await healthSystem.getHealthSummary(passportId);

// 상관관계 분석 (Analyze correlation)
const correlation = await analyzeEmotionHealthCorrelation(
  emotionData,
  healthData
);

if (correlation.insights.some(i => i.actionable)) {
  // 수의사에게 알림 (Alert veterinarian)
  await notifyVeterinarian({
    petId,
    emotionState: emotionData,
    healthConcerns: correlation.insights,
    urgency: 'medium'
  });
}
```

### 2.2 WIA-PET-GENOME 통합

감정과 유전적 소인을 상관시킵니다.

#### 2.2.1 유전-감정 매핑 (Genetic-Emotion Mapping)

```typescript
interface GeneticEmotionProfile {
  petId: string;
  genomeProfileId: string;  // WIA-PET-GENOME ID

  // 유전적 소인 (Genetic predispositions)
  predispositions: {
    trait: 'anxiety_prone' | 'stress_resilient' | 'sociable' | 'fearful';
    genes: string[];
    risk_level: 'low' | 'moderate' | 'high';
    confidence: number;

    // 관찰 vs 예상 (Observed vs expected)
    genetic_expectation: {
      expected_emotion: CoreEmotion;
      expected_frequency: number;
    };
    actual_observation: {
      observed_emotion: CoreEmotion;
      observed_frequency: number;
    };
    variance: number;  // 분산
  }[];

  // 품종 전형적 감정 (Breed-typical emotions)
  breedProfile?: {
    breed: string;
    typical_emotions: CoreEmotion[];
    deviation_from_norm: number;  // 표준 편차
  };

  // 권장사항 (Recommendations)
  recommendations: {
    category: 'environmental' | 'behavioral' | 'medical';
    recommendation: string;
    genetic_basis: string;
  }[];
}
```

#### 2.2.2 통합 예제 (Integration Example)

```typescript
import { PetGenome } from 'wia-pet-genome';

const genomeSystem = new PetGenome();

// 유전자 프로필 가져오기 (Get genetic profile)
const geneticProfile = await genomeSystem.getGeneticProfile(petId);

// 불안 관련 유전자 확인 (Check for anxiety-related genes)
const anxietyGenes = geneticProfile.healthMarkers.filter(
  marker => marker.condition.includes('anxiety')
);

if (anxietyGenes.some(g => g.riskLevel === 'at_risk')) {
  // 불안을 더 면밀히 모니터링 (Monitor anxiety more closely)
  await emotionSystem.configure({
    detection: {
      emotions_to_monitor: ['anxious', 'fearful', 'stressed'],
      monitoring_frequency: 60,  // 60 Hz로 증가
      alert_threshold: 0.6  // 조기 감지를 위한 낮은 임계값
    }
  });
}
```

### 2.3 WIA-PET-CARE-ROBOT 통합

감정 인식 로봇 반려동물 케어.

#### 2.3.1 감정 기반 로봇 행동 (Emotion-Driven Robot Behavior)

```typescript
interface EmotionRobotBehavior {
  robotId: string;
  petId: string;

  // 현재 반려동물 감정 (Current pet emotion)
  currentEmotion: EmotionState;

  // 로봇 반응 (Robot response)
  behaviorAdaptation: {
    movement: 'approach' | 'maintain_distance' | 'retreat' | 'slow';
    interaction_style: 'gentle' | 'playful' | 'calm' | 'minimal';
    voice_tone: 'soothing' | 'cheerful' | 'neutral';
    speed: number;  // 0.0-1.0
  };

  // 작업 조정 (Task adjustments)
  taskModification: {
    originalTask: string;
    modifiedTask: string;
    reason: string;
  }[];

  // 안전 오버라이드 (Safety overrides)
  safetyMode: boolean;
  safetyReason?: string;
}
```

#### 2.3.2 통합 예제 (Integration Example)

```typescript
import { PetCareRobot } from 'wia-pet-care-robot';

const robot = new PetCareRobot('ROBOT-001');

// 감정 변화 구독 (Subscribe to emotion changes)
emotionSystem.on('emotion_changed', async (event) => {
  const emotion = event.current.emotion;

  // 감정에 따라 로봇 행동 조정 (Adjust robot behavior based on emotion)
  if (emotion === 'fearful' || emotion === 'anxious') {
    // 진정 모드로 전환 (Switch to calming mode)
    await robot.setBehaviorMode('calming');
    await robot.setMovementSpeed(0.3);  // 천천히 움직임
    await robot.playSound('soothing_music');

  } else if (emotion === 'playful' || emotion === 'excited') {
    // 놀이 모드로 전환 (Switch to play mode)
    await robot.setBehaviorMode('playful');
    await robot.initiatePlaySession();

  } else if (emotion === 'tired') {
    // 최소한의 상호작용 (Minimal interaction)
    await robot.setBehaviorMode('minimal');
    await robot.createQuietZone();
  }
});

// 로봇도 감정 감지를 트리거할 수 있음 (Robot can also trigger emotion detection)
robot.on('interaction_started', async () => {
  const emotionBefore = await emotionSystem.getCurrentEmotion(petId);
  // ... 상호작용 ...
  const emotionAfter = await emotionSystem.getCurrentEmotion(petId);

  // 상호작용 성공 평가 (Evaluate interaction success)
  const interactionSuccess = emotionAfter.dimensions.valence >
                            emotionBefore.dimensions.valence;

  robot.logInteraction({
    success: interactionSuccess,
    emotionChange: {
      before: emotionBefore,
      after: emotionAfter
    }
  });
});
```

---

## 3. 반려동물 케어 생태계 (Pet Care Ecosystem)

### 3.1 수의 병원 통합 (Veterinary Clinic Integration)

#### 3.1.1 진단을 위한 감정 데이터 (Emotion Data for Diagnosis)

```typescript
interface VeterinaryEmotionReport {
  reportId: string;
  petId: string;
  generatedAt: ISO8601;

  // 병원 정보 (Clinic information)
  clinic: {
    clinicId: string;
    veterinarianId: string;
    visitId: string;
  };

  // 감정 요약 (Emotion summary)
  emotionSummary: {
    period: TimeRange;
    dominantEmotions: CoreEmotion[];
    abnormalPatterns: string[];
    stressIndicators: StressIndicator[];
  };

  // 임상적 관련성 (Clinical relevance)
  clinicalFindings: {
    finding: string;
    emotionCorrelation: number;
    significance: 'low' | 'medium' | 'high';
    requiresAttention: boolean;
  }[];

  // 수의사를 위한 권장사항 (Recommendations for vet)
  veterinaryNotes: {
    behavioralConcerns: string[];
    suggestedTests: string[];
    followUpRequired: boolean;
  };

  // 수의학 소프트웨어용 형식 (Formatted for veterinary software)
  exportFormats: {
    hl7: string;      // HL7 FHIR 형식
    vetsFirst: string; // VetsFirst 형식
    cornerstone: string; // Cornerstone 형식
    custom: any;
  };
}
```

#### 3.1.2 방문 전 감정 분석 (Pre-Visit Emotion Analysis)

```typescript
class VeterinaryIntegration {
  // 방문 전 보고서 생성 (Generate pre-visit report)
  async generatePreVisitReport(
    petId: string,
    appointmentDate: Date
  ): Promise<VeterinaryEmotionReport> {
    // 지난 30일 동안의 감정 데이터 가져오기 (Get emotion data for past 30 days)
    const emotionHistory = await emotionSystem.getEmotionHistory(petId, {
      start: new Date(appointmentDate.getTime() - 30 * 24 * 60 * 60 * 1000),
      end: appointmentDate
    });

    // 스트레스 패턴 감지 (Detect stress patterns)
    const stressPatterns = await emotionSystem.analyzePattern(petId, {
      patternType: 'stress',
      minSignificance: 0.7
    });

    // 건강 이벤트와 상관관계 분석 (Correlate with health events)
    const healthData = await healthSystem.getRecentEvents(petId);
    const correlation = await correlateEmotionHealth(emotionHistory, healthData);

    return {
      reportId: generateId(),
      petId,
      generatedAt: new Date().toISOString(),
      emotionSummary: summarizeEmotions(emotionHistory),
      clinicalFindings: correlation.findings,
      veterinaryNotes: generateVeterinaryNotes(stressPatterns, correlation)
    };
  }

  // 실시간 병원 감정 모니터링 (Real-time clinic emotion monitoring)
  async monitorClinicVisit(
    petId: string,
    visitId: string
  ): Promise<void> {
    // 연속 모니터링 시작 (Start continuous monitoring)
    await emotionSystem.startContinuousDetection(petId, {
      frequency: 60,  // 병원용 높은 빈도
      sources: ['camera', 'wearable'],
      alerts: {
        enabled: true,
        conditions: [
          {
            type: 'high_stress',
            stress_threshold: 0.8,
            callback: async (alert) => {
              // 병원 직원에게 즉시 알림 (Notify vet staff immediately)
              await notifyClinicStaff(visitId, alert);
            }
          }
        ]
      }
    });

    // 방문 중 감정 기록 (Log emotions during visit)
    emotionSystem.on('emotion_detected', (event) => {
      clinicSystem.logVisitEmotion(visitId, event);
    });
  }
}
```

### 3.2 반려동물 훈련사 통합 (Pet Trainer Integration)

```typescript
interface TrainingSessionEmotions {
  sessionId: string;
  petId: string;
  trainerId: string;

  // 세션 세부사항 (Session details)
  session: {
    startTime: ISO8601;
    endTime: ISO8601;
    duration: number;
    trainingType: 'obedience' | 'agility' | 'behavioral' | 'socialization';
  };

  // 감정적 진행 (Emotional progression)
  emotionTimeline: {
    timestamp: ISO8601;
    emotion: CoreEmotion;
    intensity: number;
    trigger: string;
  }[];

  // 학습 지표 (Learning indicators)
  learningMetrics: {
    engagement: number;      // 참여도 (0-1)
    stress_level: number;    // 스트레스 수준 (0-1)
    frustration: number;     // 좌절감 (0-1)
    success_emotion: number; // 성공 시 긍정 감정
    optimal_state_time: number; // 최적 학습 상태 시간
  };

  // 권장사항 (Recommendations)
  trainerRecommendations: {
    optimal_session_length: number;  // 최적 세션 길이
    best_training_time: string;      // 최적 훈련 시간
    stress_triggers: string[];       // 스트레스 유발 요인
    motivation_factors: string[];    // 동기 요인
  };
}
```

### 3.3 반려동물 어린이집/호텔 통합 (Pet Daycare/Boarding Integration)

```typescript
interface DaycareEmotionMonitoring {
  facilityId: string;
  petId: string;
  checkInTime: ISO8601;
  checkOutTime?: ISO8601;

  // 실시간 모니터링 (Real-time monitoring)
  currentEmotion: EmotionState;
  emotionHistory: EmotionTimeline;

  // 사회적 상호작용 (Social interactions)
  interactions: SocialInteractionEvent[];
  playGroups: {
    groupId: string;
    participants: string[];
    groupEmotion: 'positive' | 'neutral' | 'negative';
    recommendedDuration: number;
  }[];

  // 보호자를 위한 일일 보고서 (Daily report for owner)
  dailyReport: {
    overall_mood: CoreEmotion;
    happy_time_percentage: number;
    social_engagement: number;
    stress_events: number;
    notable_moments: {
      timestamp: ISO8601;
      description: string;
      emotion: CoreEmotion;
      photo?: string;
    }[];
  };

  // 직원 알림 (Alerts for staff)
  staffAlerts: {
    timestamp: ISO8601;
    alertType: 'separation_anxiety' | 'aggression' | 'illness' | 'injury';
    severity: 'low' | 'medium' | 'high';
    action_taken: string;
  }[];
}
```

---

## 4. 스마트홈 통합 (Smart Home Integration)

### 4.1 감정 기반 홈 자동화 (Home Automation Based on Emotions)

```typescript
interface SmartHomeEmotionAutomation {
  homeId: string;
  petId: string;

  // 자동화 규칙 (Automation rules)
  rules: {
    ruleId: string;
    name: string;
    enabled: boolean;

    // 트리거 (Trigger)
    trigger: {
      emotion: CoreEmotion;
      minIntensity?: number;
      duration?: number;  // milliseconds
    };

    // 액션 (Actions)
    actions: {
      device: string;
      action: string;
      parameters: Record<string, any>;
    }[];

    // 일정 (Schedule)
    schedule?: {
      daysOfWeek: number[];
      startTime: string;
      endTime: string;
    };
  }[];
}
```

#### 4.1.1 자동화 예제 (Example Automations)

```typescript
const automations: SmartHomeEmotionAutomation = {
  homeId: 'HOME-001',
  petId: 'PET-001',
  rules: [
    {
      ruleId: 'RULE-001',
      name: '불안한 반려동물 진정시키기',
      enabled: true,
      trigger: {
        emotion: 'anxious',
        minIntensity: 0.7,
        duration: 60000  // 1분
      },
      actions: [
        {
          device: 'philips-hue-living-room',
          action: 'set_scene',
          parameters: { scene: 'relax', brightness: 50 }
        },
        {
          device: 'sonos-living-room',
          action: 'play_playlist',
          parameters: { playlist: 'calming-pet-music', volume: 30 }
        },
        {
          device: 'nest-thermostat',
          action: 'set_temperature',
          parameters: { temperature: 22 }  // 편안한 온도
        }
      ]
    },
    {
      ruleId: 'RULE-002',
      name: '장난스러운 반려동물 - 상호작용 모드',
      enabled: true,
      trigger: {
        emotion: 'playful',
        minIntensity: 0.6
      },
      actions: [
        {
          device: 'petcube-play',
          action: 'activate_laser',
          parameters: { pattern: 'random', duration: 300 }
        },
        {
          device: 'furbo-treat-dispenser',
          action: 'dispense_treat',
          parameters: { count: 1 }
        }
      ]
    }
  ]
};
```

---

## 5. AI 모델 통합 (AI Model Integration)

### 5.1 모델 레지스트리 (Model Registry)

```typescript
interface ModelRegistry {
  // 모델 등록 (Register a model)
  registerModel(model: EmotionDetectionModel): Promise<string>;

  // 사용 가능한 모델 조회 (Get available models)
  listModels(filters?: ModelFilter): Promise<ModelInfo[]>;

  // 모델 다운로드 (Download model)
  downloadModel(modelId: string, version?: string): Promise<ModelPackage>;

  // 모델 메타데이터 (Model metadata)
  getModelInfo(modelId: string): Promise<ModelInfo>;
}

interface EmotionDetectionModel {
  modelId: string;
  name: string;
  version: string;
  author: string;

  // 모델 사양 (Model specifications)
  architecture: 'cnn' | 'lstm' | 'transformer' | 'ensemble';
  inputModalities: ('image' | 'audio' | 'sensor')[];
  outputEmotions: CoreEmotion[];

  // 훈련 정보 (Training info)
  trainingData: {
    species: PetSpecies[];
    datasetSize: number;
    datasetName?: string;
  };

  // 성능 (Performance)
  performance: {
    accuracy: number;          // 정확도
    precision: number;         // 정밀도
    recall: number;            // 재현율
    f1_score: number;
    inference_time: number;    // 추론 시간 (milliseconds)
  };

  // 요구사항 (Requirements)
  requirements: {
    minImageResolution?: [number, number];
    minAudioSampleRate?: number;
    gpu_required: boolean;
    memory_mb: number;
  };

  // 라이선싱 (Licensing)
  license: string;
  price?: {
    model: 'free' | 'one_time' | 'subscription';
    amount?: number;
    currency?: string;
  };
}
```

---

## 6. 제3자 서비스 (Third-Party Services)

### 6.1 반려동물 보험 통합 (Pet Insurance Integration)

```typescript
interface InsuranceEmotionData {
  policyId: string;
  petId: string;

  // 감정 데이터로부터의 웰니스 지표 (Wellness metrics from emotion data)
  wellnessScore: {
    score: number;  // 0-100
    factors: {
      emotional_stability: number;  // 감정 안정성
      stress_level: number;          // 스트레스 수준
      activity_level: number;        // 활동 수준
      social_health: number;         // 사회적 건강
    };
    trend: 'improving' | 'stable' | 'declining';
  };

  // 행동 위험 평가 (Behavioral risk assessment)
  riskAssessment: {
    anxiety_risk: 'low' | 'medium' | 'high';
    aggression_risk: 'low' | 'medium' | 'high';
    separation_anxiety: boolean;     // 분리 불안
    noise_sensitivity: boolean;      // 소음 민감성
  };

  // 보험료 조정 (Premium adjustments)
  premiumImpact: {
    current_premium: number;         // 현재 보험료
    wellness_discount: number;       // 웰니스 할인
    adjusted_premium: number;        // 조정된 보험료
    discount_reason: string[];
  };
}
```

### 6.2 반려동물 간식/영양 서비스 (Pet Food/Nutrition Services)

```typescript
interface NutritionEmotionLink {
  petId: string;

  // 감정 기반 식이 권장사항 (Emotion-based dietary recommendations)
  recommendations: {
    // 진정/불안 보충제 (Calm/anxiety supplements)
    supplements: {
      name: string;
      purpose: 'anxiety_reduction' | 'stress_management' | 'mood_support';
      dosage: string;
      evidence: 'emotion_pattern_analysis';
    }[];

    // 식단 조정 (Diet adjustments)
    dietaryChanges: {
      change: string;
      reason: string;
      emotionTrigger: CoreEmotion;
    }[];
  };

  // 상관관계 추적 (Correlation tracking)
  foodEmotionCorrelation: {
    foodType: string;
    emotionImpact: {
      emotion: CoreEmotion;
      change: number;  // 빈도의 +/- 변화
    }[];
  }[];
}
```

---

## 7. 참조 구현 (Reference Implementations)

### 7.1 완전한 모바일 앱 통합 (Complete Mobile App Integration)

```typescript
// 모바일 앱 예제 (React Native)
import { PetEmotion, CameraAdapter } from 'wia-pet-emotion';
import { PetHealthPassport } from 'wia-pet-health-passport';

class PetEmotionApp {
  private emotion: PetEmotion;
  private health: PetHealthPassport;
  private camera: CameraAdapter;

  async initialize() {
    // 시스템 초기화 (Initialize systems)
    this.emotion = new PetEmotion({
      detection: { mode: 'realtime', frequency: 30 },
      storage: { backend: 'sqlite', retention_days: 90 }
    });

    this.health = new PetHealthPassport();
    await this.emotion.initialize();

    // 카메라 설정 (Setup camera)
    this.camera = new CameraAdapter({ device: 'front' });
    await this.camera.connect();
  }

  async monitorPet(petId: string) {
    // 연속 감지 시작 (Start continuous detection)
    await this.emotion.startContinuousDetection(petId, {
      frequency: 30,
      sources: ['camera'],
      alerts: {
        enabled: true,
        conditions: [
          {
            type: 'stress_alert',
            stress_threshold: 0.8,
            callback: this.handleStressAlert.bind(this)
          }
        ]
      }
    });

    // 실시간 감정 표시 (Display real-time emotion)
    this.emotion.on('emotion_detected', (event) => {
      this.updateUI(event.result);
    });
  }

  async handleStressAlert(alert: StressAlertEvent) {
    // 알림 표시 (Show notification)
    await this.showNotification({
      title: '높은 스트레스 감지됨',
      body: `${alert.petName}이(가) 높은 스트레스 징후를 보이고 있습니다`,
      actions: ['세부정보 보기', '수의사에게 연락']
    });

    // 건강 기록에 기록 (Log to health record)
    await this.health.addBehavioralNote(alert.petId, {
      note: `높은 스트레스 에피소드 감지됨`,
      timestamp: alert.timestamp,
      severity: alert.severity
    });
  }

  async generateWeeklyReport(petId: string) {
    const summary = await this.emotion.getEmotionSummary(petId, {
      type: 'week'
    });

    const healthData = await this.health.getWeeklySummary(petId);

    return {
      emotion: summary,
      health: healthData,
      insights: await this.generateInsights(summary, healthData)
    };
  }
}
```

---

## 8. 배포 예제 (Deployment Examples)

### 8.1 클라우드 배포 (Cloud Deployment - AWS)

```yaml
# AWS CloudFormation 템플릿
AWSTemplateFormatVersion: '2010-09-09'
Description: WIA Pet Emotion Detection Service

Resources:
  # API Gateway
  PetEmotionAPI:
    Type: AWS::ApiGatewayV2::Api
    Properties:
      Name: PetEmotionAPI
      ProtocolType: WEBSOCKET

  # Lambda Function for emotion detection
  EmotionDetectionFunction:
    Type: AWS::Lambda::Function
    Properties:
      FunctionName: PetEmotionDetection
      Runtime: python3.11
      Handler: index.handler
      MemorySize: 3008
      Timeout: 30
      Environment:
        Variables:
          MODEL_BUCKET: !Ref ModelBucket
          DATA_TABLE: !Ref EmotionDataTable

  # DynamoDB for emotion data
  EmotionDataTable:
    Type: AWS::DynamoDB::Table
    Properties:
      TableName: PetEmotionData
      BillingMode: PAY_PER_REQUEST
      AttributeDefinitions:
        - AttributeName: petId
          AttributeType: S
        - AttributeName: timestamp
          AttributeType: N
      KeySchema:
        - AttributeName: petId
          KeyType: HASH
        - AttributeName: timestamp
          KeyType: RANGE

  # S3 for model storage
  ModelBucket:
    Type: AWS::S3::Bucket
    Properties:
      BucketName: wia-pet-emotion-models
```

---

**문서 ID (Document ID)**: WIA-PET-EMOTION-PHASE4-001
**버전 (Version)**: 1.0.0
**최종 업데이트 (Last Updated)**: 2025-12-18
**저작권 (Copyright)**: © 2025 WIA - MIT License

---
**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
