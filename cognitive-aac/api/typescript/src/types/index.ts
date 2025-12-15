/**
 * WIA Cognitive AAC - Type Definitions
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

// ============================================================================
// Cognitive Level Types
// ============================================================================

export enum CognitiveLevel {
  PROFOUND = 1,
  SEVERE = 2,
  MODERATE = 3,
  MILD = 4,
  TYPICAL = 5,
}

export type CognitiveLevelKey = keyof typeof CognitiveLevel;

// ============================================================================
// Cognitive Profile Types (from Phase 1)
// ============================================================================

export interface MemoryDomain {
  immediateRecall: {
    level: CognitiveLevel;
    span?: number;
    assessmentDate?: string;
  };
  workingMemory: {
    level: CognitiveLevel;
    complexity?: number;
  };
  longTermMemory: {
    episodic: CognitiveLevel;
    semantic: CognitiveLevel;
    procedural: CognitiveLevel;
  };
}

export interface AttentionDomain {
  sustained: {
    level: CognitiveLevel;
    durationMinutes?: number;
  };
  selective: {
    level: CognitiveLevel;
    distractibility?: number;
  };
  divided: {
    level: CognitiveLevel;
    maxTasks?: number;
  };
  shifting: {
    level: CognitiveLevel;
    transitionTimeSeconds?: number;
  };
}

export interface LanguageDomain {
  receptive: {
    level: CognitiveLevel;
    complexityLevel?: number;
  };
  expressive: {
    level: CognitiveLevel;
    complexityLevel?: number;
  };
  naming?: {
    level: CognitiveLevel;
    accuracy?: number;
  };
  repetition?: {
    level: CognitiveLevel;
    maxLength?: number;
  };
}

export interface ExecutiveFunctionDomain {
  planning: {
    level: CognitiveLevel;
    maxSteps?: number;
  };
  inhibition: {
    level: CognitiveLevel;
    errorRate?: number;
  };
  flexibility: {
    level: CognitiveLevel;
    adaptationSpeed?: number;
  };
  problemSolving: {
    level: CognitiveLevel;
    maxComplexity?: number;
  };
}

export interface VisualSpatialDomain {
  visualPerception: {
    level: CognitiveLevel;
    acuity?: string;
  };
  spatialOrientation: {
    level: CognitiveLevel;
  };
  construction: {
    level: CognitiveLevel;
  };
}

export interface SocialCognitionDomain {
  emotionRecognition: {
    level: CognitiveLevel;
    modalities?: ('face' | 'voice' | 'body')[];
  };
  theoryOfMind: {
    level: CognitiveLevel;
  };
  socialJudgment: {
    level: CognitiveLevel;
  };
}

export interface CognitiveDomains {
  memory: MemoryDomain;
  attention: AttentionDomain;
  language: LanguageDomain;
  executive: ExecutiveFunctionDomain;
  visualSpatial: VisualSpatialDomain;
  socialCognition: SocialCognitionDomain;
}

export interface ProfileSummary {
  overallLevel: CognitiveLevel;
  strengths: string[];
  challenges: string[];
}

export interface CognitiveProfile {
  id: string;
  version: string;
  createdAt: string;
  updatedAt: string;
  subject: {
    id: string;
    birthYear?: number;
    primaryDiagnosis?: string;
  };
  domains: CognitiveDomains;
  summary: ProfileSummary;
  aacRecommendations?: AACAdaptation;
  assessment: {
    tool: string;
    assessor?: string;
    date: string;
    notes?: string;
  };
}

// ============================================================================
// Autism Profile Types
// ============================================================================

export type SensoryReactivity = 'hypo' | 'typical' | 'hyper' | 'mixed';

export interface SensoryProfile {
  reactivity: SensoryReactivity;
  seekingBehaviors?: string[];
  avoidanceBehaviors?: string[];
  specificTriggers?: string[];
  copingStrategies?: string[];
}

export interface AutismProfile extends CognitiveProfile {
  profileType: 'autism';
  diagnosis: {
    supportLevel: 1 | 2 | 3;
    languageStatus: 'nonverbal' | 'minimally_verbal' | 'verbal_limited' | 'verbal_fluent';
    intellectualStatus: 'intellectual_disability' | 'borderline' | 'average' | 'above_average';
    diagnosisAge?: number;
    comorbidities?: string[];
  };
  sensoryProcessing: {
    visual: SensoryProfile;
    auditory: SensoryProfile;
    tactile: SensoryProfile;
    vestibular: SensoryProfile;
    proprioceptive: SensoryProfile;
  };
  specialInterests?: {
    topics: { topic: string; intensity: number; functionalUse: boolean }[];
    usableForAAC: boolean;
  };
  routineAdherence?: {
    rigidityLevel: number;
    transitionDifficulty: number;
  };
}

// ============================================================================
// Dementia Profile Types
// ============================================================================

export type DementiaType = 'alzheimers' | 'vascular' | 'lewy_body' | 'frontotemporal' | 'mixed' | 'parkinsons' | 'other';
export type DementiaStage = 'preclinical' | 'mci' | 'early' | 'middle' | 'late';

export interface DementiaProfile extends CognitiveProfile {
  profileType: 'dementia';
  diagnosis: {
    type: DementiaType;
    stage: DementiaStage;
    diagnosisDate: string;
    lastAssessmentDate: string;
    cognitiveScores?: {
      mmse?: number;
      moca?: number;
      cdr?: number;
    };
  };
  preservedAbilities?: {
    proceduralMemory?: { preserved: boolean; examples?: string[] };
    emotionalProcessing?: { preserved: boolean; respondsToMusic?: boolean };
    musicalAbility?: { preserved: boolean; preferredMusic?: string[] };
    remoteMemory?: { preserved: boolean; accessiblePeriods?: string[] };
    readingAbility?: { preserved: boolean; level?: string };
  };
  personalHistory?: {
    familyMembers?: { relationship: string; name: string; recognitionLevel: string }[];
    preferences?: {
      favoriteMusic?: string[];
      dailyRoutines?: string[];
    };
  };
  behavioralSymptoms?: {
    sundowning?: boolean;
    bestTimeOfDay?: string;
    worstTimeOfDay?: string;
  };
}

// ============================================================================
// UI Configuration Types
// ============================================================================

export type SymbolType = 'photos' | 'pcs' | 'widgit' | 'arasaac' | 'text' | 'mixed';
export type CellSize = 'small' | 'medium' | 'large' | 'xlarge';
export type LabelPosition = 'above' | 'below' | 'none';
export type LayoutType = 'grid' | 'list' | 'scene' | 'categorical';

export interface GridConfig {
  columns: number;
  rows: number;
  cellSize: CellSize;
  gap?: number;
}

export interface SymbolConfig {
  type: SymbolType;
  labelPosition: LabelPosition;
  fontSize: number;
  highContrast: boolean;
  borderRadius?: number;
}

export interface InteractionConfig {
  dwellTime: number;
  confirmationRequired: boolean;
  audioFeedback: boolean;
  hapticFeedback: boolean;
  highlightOnHover: boolean;
}

export interface CognitiveLoadConfig {
  maxVisibleItems: number;
  progressiveDisclosure: boolean;
  distractionFilter: number;
  simplificationLevel: number;
}

export interface UIConfiguration {
  grid: GridConfig;
  symbols: SymbolConfig;
  interaction: InteractionConfig;
  cognitiveLoad: CognitiveLoadConfig;
  theme?: ThemeConfig;
}

export interface ThemeConfig {
  colorScheme: 'light' | 'dark' | 'high-contrast';
  primaryColor: string;
  backgroundColor: string;
  textColor: string;
  accentColor: string;
  reducedMotion: boolean;
}

// ============================================================================
// AAC Adaptation Types
// ============================================================================

export interface AACAdaptation {
  interface: {
    maxItemsPerScreen: number;
    layoutType: LayoutType;
    iconSize: CellSize;
    animationsEnabled: boolean;
  };
  symbols: {
    preferredType: SymbolType;
    abstractionLevel: 1 | 2 | 3;
    labelPosition: LabelPosition;
  };
  input: {
    methods: ('touch' | 'switch' | 'eyeGaze' | 'voice')[];
    dwellTimeMs?: number;
    scanSpeed?: number;
  };
  feedback: {
    auditory: boolean;
    visual: boolean;
    haptic: boolean;
    confirmationRequired: boolean;
  };
  pacing: {
    autoAdvance: boolean;
    delayBetweenItemsMs: number;
    timeoutEnabled: boolean;
    timeoutMs?: number;
  };
}

// ============================================================================
// Autism-specific UI Adjustments
// ============================================================================

export interface AutismUIAdjustments {
  sensoryAccommodations: {
    reducedAnimation: boolean;
    mutedColors: boolean;
    noFlashing: boolean;
    quietMode: boolean;
  };
  predictability: {
    consistentLayout: boolean;
    transitionWarnings: boolean;
    visualSchedule: boolean;
    progressIndicator: boolean;
  };
  specialInterests: {
    enabled: boolean;
    interests: string[];
    integratedInUI: boolean;
  };
}

// ============================================================================
// Dementia-specific UI Adjustments
// ============================================================================

export interface FamiliarPerson {
  name: string;
  relationship: string;
  photoUrl?: string;
  recognitionLevel: 'full' | 'sometimes' | 'familiar' | 'none';
}

export interface DementiaUIAdjustments {
  familiarity: {
    usePersonalPhotos: boolean;
    familiarFaces: FamiliarPerson[];
    personalLocations: string[];
    lifeStoryIntegration: boolean;
  };
  simplification: {
    minimalNavigation: boolean;
    largeText: boolean;
    reducedChoices: number;
    clearLabeling: boolean;
  };
  temporalSupport: {
    dayNightAwareness: boolean;
    mealTimeReminders: boolean;
    routineBasedSuggestions: boolean;
    showDateTime: boolean;
  };
  errorPrevention: {
    confirmBeforeAction: boolean;
    undoSupport: boolean;
    noDestructiveActions: boolean;
  };
}

// ============================================================================
// Event Types
// ============================================================================

export interface UserInteractionEvent {
  type: 'selection' | 'navigation' | 'error' | 'timeout' | 'hesitation';
  timestamp: number;
  targetId?: string;
  duration?: number;
  successful?: boolean;
  metadata?: Record<string, unknown>;
}

export interface UIAdjustment {
  type: 'grid' | 'symbols' | 'interaction' | 'cognitiveLoad' | 'theme';
  changes: Partial<UIConfiguration>;
  reason: string;
  confidence: number;
}

export interface UsageHistory {
  sessions: SessionData[];
  aggregatedMetrics: AggregatedMetrics;
}

export interface SessionData {
  id: string;
  startTime: string;
  endTime: string;
  events: UserInteractionEvent[];
  configuration: UIConfiguration;
}

export interface AggregatedMetrics {
  averageSelectionTime: number;
  errorRate: number;
  successRate: number;
  mostUsedSymbols: string[];
  preferredGridSize: { columns: number; rows: number };
  optimalDwellTime: number;
}

// ============================================================================
// Symbol Item Types
// ============================================================================

export interface SymbolItem {
  id: string;
  label: string;
  imageUrl?: string;
  audioUrl?: string;
  category?: string;
  subcategory?: string;
  action?: 'speak' | 'navigate' | 'execute';
  targetPageId?: string;
  metadata?: Record<string, unknown>;
}

export interface SymbolPage {
  id: string;
  title: string;
  symbols: SymbolItem[];
  parentPageId?: string;
  layout?: Partial<GridConfig>;
}
