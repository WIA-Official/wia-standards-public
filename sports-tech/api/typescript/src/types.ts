/**
 * WIA-IND-011: Sports Tech Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @author WIA Technical Committee - Sports Technology Division
 * @license MIT
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * This file contains comprehensive TypeScript type definitions for the
 * WIA-IND-011 Sports Technology standard, covering performance tracking,
 * smart equipment, injury prevention, training optimization, and broadcasting.
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Standard identifier for all WIA-IND-011 data
 */
export const STANDARD_ID = 'WIA-IND-011' as const;
export const VERSION = '1.0.0' as const;

/**
 * UUID string type (RFC 4122)
 */
export type UUID = string;

/**
 * ISO 8601 timestamp string
 */
export type ISOTimestamp = string;

/**
 * ISO 8601 date string (YYYY-MM-DD)
 */
export type ISODate = string;

/**
 * IANA timezone string (e.g., "America/New_York")
 */
export type Timezone = string;

// ============================================================================
// Athlete Profile Types
// ============================================================================

/**
 * Gender options following inclusive practices
 */
export type Gender = 'male' | 'female' | 'other' | 'prefer-not-to-say';

/**
 * Experience level classification
 */
export type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced' | 'elite' | 'professional';

/**
 * Dominant side for athletes
 */
export type DominantSide = 'left' | 'right' | 'ambidextrous';

/**
 * Personal information (sensitive, should be encrypted)
 */
export interface PersonalInfo {
  firstName?: string;  // Optional, encrypted
  lastName?: string;   // Optional, encrypted
  dateOfBirth?: ISODate;  // Optional, encrypted
  gender: Gender;
  nationality?: string;  // ISO 3166-1 alpha-2 country code
}

/**
 * Biometric measurements
 */
export interface Biometrics {
  height: number;  // cm
  weight: number;  // kg
  bodyFatPercentage?: number;  // 0-100
  restingHeartRate: number;  // bpm
  maxHeartRate?: number;  // bpm
  vo2max?: number;  // ml/kg/min
  lactateThreshold?: number;  // bpm
  ftp?: number;  // Functional Threshold Power (watts, for cycling)
}

/**
 * Sport-specific information
 */
export interface SportInfo {
  primarySport: string;  // From sport taxonomy
  position?: string;  // Sport-specific position
  experience: ExperienceLevel;
  yearsActive: number;
  dominantSide?: DominantSide;
}

/**
 * Goal types
 */
export type GoalType = 'performance' | 'health' | 'skill';

/**
 * Goal status
 */
export type GoalStatus = 'active' | 'achieved' | 'abandoned';

/**
 * Athlete goal
 */
export interface AthleteGoal {
  goalId: UUID;
  type: GoalType;
  description: string;
  targetDate: ISODate;
  targetMetric: string;
  targetValue: number;
  status: GoalStatus;
}

/**
 * Injury severity classification
 */
export type InjurySeverity = 'minor' | 'moderate' | 'severe' | 'critical';

/**
 * Injury history record
 */
export interface InjuryRecord {
  injuryId: UUID;
  date: ISODate;
  type: string;  // ICD-11 code
  area: string;  // Body part
  severity: InjurySeverity;
  recoveryDays: number;
  notes?: string;  // Encrypted
}

/**
 * Equipment/device status
 */
export type DeviceStatus = 'active' | 'inactive' | 'maintenance';

/**
 * Equipment record
 */
export interface Equipment {
  deviceId: UUID;
  type: string;  // From device taxonomy
  manufacturer: string;
  model: string;
  serialNumber: string;
  purchaseDate: ISODate;
  lastCalibration: ISOTimestamp;
  status: DeviceStatus;
}

/**
 * Data sharing preferences
 */
export type DataSharing = 'private' | 'team-only' | 'public';

/**
 * Privacy settings
 */
export interface PrivacySettings {
  dataSharing: DataSharing;
  allowResearch: boolean;
  allowCommercial: boolean;
  retentionPeriod: number;  // years
  blockchainConsent?: string;  // Transaction hash
}

/**
 * Complete athlete profile
 */
export interface AthleteProfile {
  athleteId: UUID;
  version: string;
  created: ISOTimestamp;
  updated: ISOTimestamp;
  personalInfo?: PersonalInfo;
  biometrics: Biometrics;
  sportInfo: SportInfo;
  goals: AthleteGoal[];
  injuryHistory: InjuryRecord[];
  equipment: Equipment[];
  privacy: PrivacySettings;
}

// ============================================================================
// Session Data Types
// ============================================================================

/**
 * Session type classification
 */
export type SessionType = 'training' | 'match' | 'competition' | 'testing' | 'recovery';

/**
 * Venue type
 */
export type VenueType = 'indoor' | 'outdoor';

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
  altitude: number;  // meters above sea level
}

/**
 * Venue information
 */
export interface Venue {
  venueId?: UUID;
  name: string;
  type: VenueType;
  surface: string;  // grass, turf, track, court, etc.
  coordinates?: Coordinates;
}

/**
 * Weather conditions
 */
export type WeatherCondition = 'sunny' | 'cloudy' | 'rainy' | 'snowy' | 'other';

/**
 * Environmental conditions
 */
export interface Environment {
  temperature: number;  // Celsius
  humidity: number;  // 0-100%
  pressure?: number;  // hPa
  windSpeed?: number;  // km/h
  windDirection?: number;  // degrees, 0-360
  weatherCondition?: WeatherCondition;
  airQuality?: number;  // AQI
}

/**
 * Session timestamp information
 */
export interface SessionTimestamp {
  start: ISOTimestamp;
  end?: ISOTimestamp;
  duration?: number;  // seconds
  timezone: Timezone;
}

/**
 * Participant role in session
 */
export type ParticipantRole = 'athlete' | 'opponent' | 'teammate' | 'coach' | 'referee';

/**
 * Session participant
 */
export interface Participant {
  athleteId: UUID;
  role: ParticipantRole;
  teamId?: UUID;
}

/**
 * Device used in session
 */
export interface SessionDevice {
  deviceId: UUID;
  type: string;
  samplingRate: number;  // Hz
  batteryStart: number;  // 0-100%
  batteryEnd: number;  // 0-100%
  firmwareVersion: string;
  dataQuality: number;  // 0-1 quality score
}

/**
 * Session importance level
 */
export type Importance = 'low' | 'medium' | 'high' | 'critical';

/**
 * Session outcome
 */
export type Outcome = 'win' | 'loss' | 'draw' | 'dnf' | 'na';

/**
 * Session metadata
 */
export interface SessionMetadata {
  importance: Importance;
  outcome?: Outcome;
  score?: string;  // Sport-specific format
  notes?: string;  // Encrypted
  tags?: string[];
}

/**
 * Complete session data
 */
export interface Session {
  sessionId: UUID;
  athleteId: UUID;
  standardId: typeof STANDARD_ID;
  version: typeof VERSION;
  timestamp: SessionTimestamp;
  sessionType: SessionType;
  sport: string;
  venue?: Venue;
  environment?: Environment;
  participants?: Participant[];
  devices: SessionDevice[];
  metadata?: SessionMetadata;
}

// ============================================================================
// Performance Metrics Types
// ============================================================================

/**
 * Distance breakdown by intensity
 */
export interface DistanceByIntensity {
  walking: number;  // meters
  jogging: number;
  running: number;
  sprinting: number;
}

/**
 * Heart rate zones (% of max HR)
 */
export interface DistanceByZone {
  zone1: number;  // < 50% max HR
  zone2: number;  // 50-60%
  zone3: number;  // 60-70%
  zone4: number;  // 70-80%
  zone5: number;  // 80-90%
  zone6: number;  // > 90%
}

/**
 * Distance metrics
 */
export interface DistanceMetrics {
  total: number;  // meters
  byIntensity?: DistanceByIntensity;
  byZone?: DistanceByZone;
}

/**
 * Time series data point
 */
export interface TimeSeriesPoint<T = number> {
  timestamp: ISOTimestamp;
  value: T;
}

/**
 * Speed metrics
 */
export interface SpeedMetrics {
  average: number;  // km/h
  max: number;
  median?: number;
  p95?: number;  // 95th percentile
  timeSeries?: TimeSeriesPoint[];
}

/**
 * Heart Rate Variability metrics
 */
export interface HRVMetrics {
  rmssd: number;  // ms (root mean square of successive differences)
  sdnn: number;   // ms (standard deviation of NN intervals)
  pnn50: number;  // % (percentage of successive NN intervals > 50ms)
}

/**
 * Time in heart rate zones
 */
export interface TimeInZones {
  zone1: number;  // seconds
  zone2: number;
  zone3: number;
  zone4: number;
  zone5: number;
}

/**
 * Heart rate time series point (with optional RR interval)
 */
export interface HeartRatePoint {
  timestamp: ISOTimestamp;
  value: number;  // bpm
  rrInterval?: number;  // ms
}

/**
 * Heart rate metrics
 */
export interface HeartRateMetrics {
  average: number;  // bpm
  max: number;
  min: number;
  resting?: number;
  recovery?: number;  // HR 1 minute post-session
  hrv?: HRVMetrics;
  timeInZones?: TimeInZones;
  timeSeries?: HeartRatePoint[];
}

/**
 * Power metrics
 */
export interface PowerMetrics {
  average: number;  // watts
  max: number;
  normalized?: number;  // Normalized Power
  intensityFactor?: number;  // 0-2 (NP / FTP)
  trainingStressScore?: number;  // TSS
  timeSeries?: TimeSeriesPoint[];
}

/**
 * Left-right balance
 */
export interface LeftRightBalance {
  left: number;  // percent
  right: number;
}

/**
 * Left-right measurement (generic)
 */
export interface LeftRightMeasurement<T = number> {
  left: T;
  right: T;
}

/**
 * Cadence metrics
 */
export interface CadenceMetrics {
  average: number;  // steps/min or rpm
  max: number;
}

/**
 * Stride length metrics
 */
export interface StrideLengthMetrics {
  average: number;  // meters
  leftRight?: LeftRightMeasurement<number>;
}

/**
 * Ground contact time metrics
 */
export interface GroundContactTimeMetrics {
  average: number;  // ms
  leftRight?: LeftRightMeasurement<number>;
}

/**
 * Vertical oscillation metrics
 */
export interface VerticalOscillationMetrics {
  average: number;  // cm
  ratio?: number;   // cm/meter
}

/**
 * Balance metrics
 */
export interface BalanceMetrics {
  leftRight: LeftRightBalance;
  asymmetryIndex: number;  // 0-1
}

/**
 * Joint angle measurements
 */
export interface JointAngles {
  flexion: number;  // degrees
  extension: number;
  abduction?: number;
}

/**
 * Joint angles collection
 */
export interface JointAnglesCollection {
  hip?: JointAngles;
  knee?: JointAngles;
  ankle?: {
    dorsiFlex: number;
    plantarFlex: number;
  };
  shoulder?: JointAngles;
}

/**
 * Biomechanics metrics
 */
export interface BiomechanicsMetrics {
  cadence?: CadenceMetrics;
  strideLength?: StrideLengthMetrics;
  groundContactTime?: GroundContactTimeMetrics;
  verticalOscillation?: VerticalOscillationMetrics;
  balance?: BalanceMetrics;
  jointAngles?: JointAnglesCollection;
}

/**
 * Metabolic metrics
 */
export interface MetabolicMetrics {
  caloriesBurned: number;  // kcal
  fatBurned?: number;  // grams
  carbsBurned?: number;  // grams
  respiratoryRate?: number;  // breaths/min
  coreTemperature?: number;  // Celsius
  hydrationLoss?: number;  // ml estimated
  sweatRate?: number;  // L/hour
}

/**
 * Fatigue assessment
 */
export interface FatigueAssessment {
  muscular: number;  // 0-100 score
  central: number;   // 0-100 score
  overall: number;   // 0-100 score
}

/**
 * Neuromuscular metrics
 */
export interface NeuromuscularMetrics {
  reactionTime?: number;  // ms
  explosiveness?: number;  // 0-100 score
  fatigue?: FatigueAssessment;
  readiness?: number;  // 0-100 score
}

/**
 * Complete performance metrics
 */
export interface PerformanceMetrics {
  sessionId: UUID;
  timestamp: ISOTimestamp;
  distance?: DistanceMetrics;
  speed?: SpeedMetrics;
  heartRate?: HeartRateMetrics;
  power?: PowerMetrics;
  biomechanics?: BiomechanicsMetrics;
  metabolic?: MetabolicMetrics;
  neuromuscular?: NeuromuscularMetrics;
}

// ============================================================================
// Sport-Specific Skill Metrics
// ============================================================================

/**
 * Soccer pass metrics
 */
export interface SoccerPassMetrics {
  total: number;
  successful: number;
  accuracy: number;  // percent
  avgDistance?: number;  // meters
  byType?: {
    short: number;
    medium: number;
    long: number;
  };
}

/**
 * Shot result
 */
export type ShotResult = 'goal' | 'on-target' | 'off-target' | 'blocked';

/**
 * Shot position on field
 */
export interface ShotPosition {
  x: number;  // field coordinates
  y: number;
  result: ShotResult;
}

/**
 * Soccer shot metrics
 */
export interface SoccerShotMetrics {
  total: number;
  onTarget: number;
  goals: number;
  avgSpeed?: number;  // km/h
  avgSpin?: number;  // rpm
  positions?: ShotPosition[];
}

/**
 * Dribble metrics
 */
export interface DribbleMetrics {
  attempted: number;
  successful: number;
  successRate: number;  // percent
}

/**
 * Tackle metrics
 */
export interface TackleMetrics {
  attempted: number;
  successful: number;
  fouls: number;
}

/**
 * Aerial duel metrics
 */
export interface AerialDuelMetrics {
  attempted: number;
  won: number;
  winRate: number;  // percent
}

/**
 * Soccer-specific skill metrics
 */
export interface SoccerSkillMetrics {
  ballTouches: number;
  passes: SoccerPassMetrics;
  shots: SoccerShotMetrics;
  dribbles?: DribbleMetrics;
  tackles?: TackleMetrics;
  aerialDuels?: AerialDuelMetrics;
}

/**
 * Basketball shot type metrics
 */
export interface BasketballShotTypeMetrics {
  attempted: number;
  made: number;
  percentage: number;
}

/**
 * Basketball shot metrics
 */
export interface BasketballShotMetrics {
  freeThrows: BasketballShotTypeMetrics;
  twoPoint: BasketballShotTypeMetrics;
  threePoint: BasketballShotTypeMetrics;
}

/**
 * Basketball rebound metrics
 */
export interface BasketballReboundMetrics {
  offensive: number;
  defensive: number;
  total: number;
}

/**
 * Basketball-specific skill metrics
 */
export interface BasketballSkillMetrics {
  shots: BasketballShotMetrics;
  rebounds: BasketballReboundMetrics;
  assists: number;
  steals: number;
  blocks: number;
  turnovers: number;
}

/**
 * Running split
 */
export interface RunningSplit {
  distance: number;  // meters
  time: number;  // seconds
  pace: number;  // min/km
}

/**
 * Elevation data
 */
export interface ElevationData {
  gain: number;  // meters
  loss: number;
  maxGrade: number;  // percent
}

/**
 * Running-specific skill metrics
 */
export interface RunningSkillMetrics {
  splits?: RunningSplit[];
  elevation?: ElevationData;
}

/**
 * Pedaling metrics (cycling)
 */
export interface PedalingMetrics {
  pedalSmoothness: number;  // percent
  torqueEffectiveness: number;  // percent
  leftRightBalance: LeftRightBalance;
}

/**
 * Climb category
 */
export type ClimbCategory = '4' | '3' | '2' | '1' | 'HC';

/**
 * Cycling climb
 */
export interface CyclingClimb {
  distance: number;  // meters
  elevation: number;  // meters
  gradient: number;  // percent
  category: ClimbCategory;
}

/**
 * Cycling-specific skill metrics
 */
export interface CyclingSkillMetrics {
  pedalingMetrics?: PedalingMetrics;
  climbs?: CyclingClimb[];
}

/**
 * Tennis serve type metrics
 */
export interface TennisServeTypeMetrics {
  attempted: number;
  in: number;
  percentage: number;
  avgSpeed: number;  // km/h
  maxSpeed?: number;
}

/**
 * Tennis serve metrics
 */
export interface TennisServeMetrics {
  first: TennisServeTypeMetrics;
  second: TennisServeTypeMetrics;
  aces: number;
  doubleFaults: number;
}

/**
 * Tennis groundstroke metrics
 */
export interface TennisGroundstrokeMetrics {
  total: number;
  winners: number;
  errors: number;
  avgSpeed: number;  // km/h
  avgSpin: number;  // rpm
}

/**
 * Tennis groundstrokes
 */
export interface TennisGroundstrokes {
  forehand: TennisGroundstrokeMetrics;
  backhand: TennisGroundstrokeMetrics;
}

/**
 * Tennis movement metrics
 */
export interface TennisMovementMetrics {
  courtCoverage: number;  // m²
  shotsWhileMoving: number;
  shotsStatic: number;
}

/**
 * Tennis-specific skill metrics
 */
export interface TennisSkillMetrics {
  serves: TennisServeMetrics;
  groundStrokes: TennisGroundstrokes;
  movement?: TennisMovementMetrics;
}

/**
 * Skill metrics (sport-specific)
 */
export interface SkillMetrics {
  sessionId: UUID;
  sport: string;
  soccer?: SoccerSkillMetrics;
  basketball?: BasketballSkillMetrics;
  running?: RunningSkillMetrics;
  cycling?: CyclingSkillMetrics;
  tennis?: TennisSkillMetrics;
  [sport: string]: any;  // Extensible for other sports
}

// ============================================================================
// Injury Risk Assessment Types
// ============================================================================

/**
 * Risk level classification
 */
export type RiskLevel = 'low' | 'medium' | 'high' | 'critical';

/**
 * Contributing factor to injury risk
 */
export interface RiskFactor {
  factor: string;  // fatigue, asymmetry, load, movement, etc.
  severity: number;  // 0-1
  description: string;
}

/**
 * Area-specific injury risk
 */
export interface AreaSpecificRisk {
  bodyPart: string;  // hamstring, acl, ankle, shoulder, etc.
  riskScore: number;  // 0-1 probability
  contributingFactors: RiskFactor[];
}

/**
 * Fatigue metrics for injury risk
 */
export interface FatigueMetrics {
  acute: number;  // 0-1 current fatigue
  chronic: number;  // 0-1 ongoing fatigue
  acuteChronicRatio: number;  // ACWR
  monotony?: number;  // Training monotony index
  strain?: number;  // Training strain index
}

/**
 * Compensatory movement pattern
 */
export interface CompensatoryPattern {
  pattern: string;
  severity: 'mild' | 'moderate' | 'severe';
}

/**
 * Movement quality assessment
 */
export interface MovementQuality {
  overallScore: number;  // 0-100
  asymmetryDetected: boolean;
  asymmetryMagnitude?: number;  // percent difference
  compensatoryPatterns?: CompensatoryPattern[];
}

/**
 * Alert severity
 */
export type AlertSeverity = 'info' | 'warning' | 'critical';

/**
 * Alert type
 */
export type AlertType = 'fatigue' | 'asymmetry' | 'overload' | 'movement' | 'equipment';

/**
 * Injury risk alert
 */
export interface InjuryAlert {
  alertId: UUID;
  timestamp: ISOTimestamp;
  severity: AlertSeverity;
  type: AlertType;
  message: string;
  recommendations: string[];
}

/**
 * Historical context for injury risk
 */
export interface HistoricalContext {
  previousInjuries: number;  // count in same area
  daysSinceLastInjury?: number;
  currentRecoveryStatus?: string;
}

/**
 * Injury risk recommendations
 */
export interface RiskRecommendations {
  immediate: string[];  // Actions to take now
  shortTerm: string[];  // Actions in next 48 hours
  longTerm: string[];   // Training adjustments
}

/**
 * ML model type
 */
export type ModelType = 'deep-learning' | 'random-forest' | 'ensemble';

/**
 * Model metadata
 */
export interface ModelMetadata {
  modelVersion: string;
  modelType: ModelType;
  confidence: number;  // 0-1
  trainingDataSize: number;  // number of samples
  lastUpdated: ISOTimestamp;
}

/**
 * Complete injury risk assessment
 */
export interface InjuryRisk {
  sessionId: UUID;
  athleteId: UUID;
  timestamp: ISOTimestamp;
  overallRisk: number;  // 0-1 probability
  riskLevel: RiskLevel;
  areaSpecificRisk: AreaSpecificRisk[];
  fatigueMetrics: FatigueMetrics;
  movementQuality?: MovementQuality;
  alerts?: InjuryAlert[];
  historicalContext?: HistoricalContext;
  recommendations: RiskRecommendations;
  modelMetadata: ModelMetadata;
}

// ============================================================================
// Training Plan Types
// ============================================================================

/**
 * Training goal type
 */
export type TrainingGoalType = 'performance' | 'health' | 'skill' | 'competition';

/**
 * Target event
 */
export interface TargetEvent {
  name: string;
  date: ISODate;
  importance: Importance;
}

/**
 * Training plan goal
 */
export interface TrainingGoal {
  type: TrainingGoalType;
  description: string;
  targetEvent?: TargetEvent;
}

/**
 * Season phase
 */
export type SeasonPhase =
  | 'off-season'
  | 'pre-season'
  | 'early-season'
  | 'mid-season'
  | 'late-season'
  | 'playoffs'
  | 'recovery';

/**
 * Mesocycle focus
 */
export type MesocycleFocus =
  | 'endurance'
  | 'strength'
  | 'speed'
  | 'skill'
  | 'taper'
  | 'recovery';

/**
 * Exercise definition
 */
export interface Exercise {
  exerciseId: UUID;
  name: string;
  sets?: number;
  reps?: number;
  duration?: number;  // seconds
  intensity: number;  // 1-10 RPE scale
  rest?: number;  // seconds between sets
  notes?: string;
}

/**
 * Target metrics for session
 */
export interface TargetMetrics {
  distance?: number;  // meters
  heartRateZone?: number;  // 1-5
  power?: number;  // watts
  pace?: number;  // min/km
}

/**
 * Day of week
 */
export type DayOfWeek =
  | 'monday'
  | 'tuesday'
  | 'wednesday'
  | 'thursday'
  | 'friday'
  | 'saturday'
  | 'sunday';

/**
 * Training session in plan
 */
export interface TrainingSession {
  sessionId: UUID;
  dayOfWeek: DayOfWeek;
  sessionType: SessionType;
  sport: string;
  duration: number;  // minutes
  intensity: number;  // 1-10 RPE scale
  focus: string;  // endurance, intervals, strength, etc.
  exercises?: Exercise[];
  targetMetrics?: TargetMetrics;
  notes?: string;
}

/**
 * Microcycle (typically 1 week)
 */
export interface Microcycle {
  microcycleId: UUID;
  weekNumber: number;
  startDate: ISODate;
  plannedLoad: number;  // arbitrary units
  sessions: TrainingSession[];
}

/**
 * Mesocycle (typically 3-6 weeks)
 */
export interface Mesocycle {
  mesocycleId: UUID;
  name: string;
  startDate: ISODate;
  endDate: ISODate;
  focus: MesocycleFocus;
  targetLoad: number;  // AU per week
  microcycles: Microcycle[];
}

/**
 * Plan adjustment record
 */
export interface PlanAdjustment {
  date: ISODate;
  reason: string;  // injury, illness, fatigue, etc.
  changes: string;
}

/**
 * Progress tracking
 */
export interface ProgressTracking {
  completedSessions: number;
  totalPlannedSessions: number;
  adherenceRate: number;  // percent
  adjustments?: PlanAdjustment[];
}

/**
 * Complete training plan
 */
export interface TrainingPlan {
  planId: UUID;
  athleteId: UUID;
  created: ISOTimestamp;
  validFrom: ISODate;
  validTo: ISODate;
  goal: TrainingGoal;
  seasonPhase: SeasonPhase;
  mesocycles: Mesocycle[];
  progressTracking?: ProgressTracking;
  coachNotes?: string;  // Encrypted
}

// ============================================================================
// Smart Equipment Types
// ============================================================================

/**
 * Ball kinematics
 */
export interface BallKinematics {
  speed: {
    initial: number;  // km/h
    peak: number;
    unit: 'km/h';
  };
  spin: {
    rate: number;  // rpm
    axis: { x: number; y: number; z: number };
    unit: 'rpm';
  };
  trajectory: {
    angle: number;  // degrees
    curve: 'left' | 'right' | 'straight';
    curveMagnitude: number;
  };
}

/**
 * Ball impact data
 */
export interface BallImpact {
  force: number;  // Newtons
  location: { x: number; y: number };  // Normalized 0-1
  contactTime: number;  // ms
  unit: {
    force: 'N';
    contactTime: 'ms';
  };
}

/**
 * Kick classification
 */
export interface KickClassification {
  kickType: string;  // instep, side-foot, chip, etc.
  confidence: number;  // 0-1
  bodyPart: string;  // right_foot, left_foot, head, etc.
}

/**
 * Smart ball event
 */
export interface SmartBallEvent {
  ballId: string;
  timestamp: ISOTimestamp;
  eventType: 'kick' | 'throw' | 'catch' | 'impact';
  kinematics: BallKinematics;
  impact: BallImpact;
  classification: KickClassification;
}

/**
 * Smart shoe pressure data
 */
export interface ShoePressureData {
  timestamp: ISOTimestamp;
  leftFoot: number[];  // Array of 16 pressure sensor values (0-100)
  rightFoot: number[];
  footStrike: 'heel' | 'midfoot' | 'forefoot';
  pronation: number;  // degrees (positive = overpronation)
}

// ============================================================================
// Broadcasting Types
// ============================================================================

/**
 * Camera angle
 */
export type CameraAngle = 'wide' | 'close-up' | 'aerial' | 'pov' | 'tactical';

/**
 * Camera information
 */
export interface CameraInfo {
  cameraId: string;
  angle: CameraAngle;
  timecode: string;  // HH:MM:SS.mmm
  fileSegment: string;
}

/**
 * Replay playback instruction
 */
export interface ReplayPlayback {
  camera: string;  // Camera ID
  duration: number;  // seconds
  speed: number;  // 1.0 = normal, 0.25 = slow-motion, etc.
}

/**
 * Replay event
 */
export interface ReplayEvent {
  eventId: string;
  timestamp: ISOTimestamp;
  type: string;  // goal, foul, highlight, etc.
  cameras: CameraInfo[];
  playbackSequence: ReplayPlayback[];
}

/**
 * AR overlay configuration
 */
export interface AROverlayConfig {
  playerStats: boolean;
  heatmaps: boolean;
  speedometer: boolean;
  trajectoryPrediction: boolean;
}

/**
 * Streaming configuration
 */
export interface StreamingConfig {
  platform: 'multi' | 'youtube' | 'twitch' | 'custom';
  quality: 'adaptive' | '4k' | '1080p' | '720p';
  overlays: AROverlayConfig;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Standard API response wrapper
 */
export interface APIResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    requestId: string;
    timestamp: ISOTimestamp;
    version: string;
  };
}

/**
 * Pagination metadata
 */
export interface Pagination {
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

/**
 * List response with pagination
 */
export interface ListResponse<T = any> {
  items: T[];
  pagination: Pagination;
}

/**
 * Session creation request
 */
export interface CreateSessionRequest {
  athleteId: UUID;
  sessionType: SessionType;
  sport: string;
  timestamp: {
    start: ISOTimestamp;
  };
  venue?: Venue;
  environment?: Environment;
}

/**
 * Session creation response
 */
export interface CreateSessionResponse {
  sessionId: UUID;
  uploadUrl: string;  // Presigned URL for data upload
}

/**
 * Export format
 */
export type ExportFormat = 'wia-json' | 'fit' | 'tcx' | 'gpx';

/**
 * Data export request
 */
export interface ExportRequest {
  athleteId: UUID;
  format: ExportFormat;
  startDate?: ISODate;
  endDate?: ISODate;
  sessionIds?: UUID[];
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Sensor configuration
 */
export interface SensorConfig {
  enabled: boolean;
  frequency?: number;  // Hz
  type?: string;
  model?: string;
}

/**
 * Data collector configuration
 */
export interface DataCollectorConfig {
  athleteId: UUID;
  sport: string;
  position?: string;
  sensors: {
    gps?: SensorConfig;
    heartRate?: SensorConfig;
    imu?: SensorConfig;
    smartBall?: SensorConfig;
    [key: string]: SensorConfig | undefined;
  };
  privacy: PrivacySettings;
}

/**
 * Injury predictor configuration
 */
export interface InjuryPredictorConfig {
  model: string;  // Model version
  alertThreshold: number;  // 0-1 probability threshold
  monitoredAreas: string[];  // Body parts to monitor
}

/**
 * Training optimizer configuration
 */
export interface TrainingOptimizerConfig {
  athleteProfile: {
    age: number;
    position: string;
    experience: ExperienceLevel;
    injuryHistory: string[];
    goals: string[];
  };
  seasonPlan: {
    phase: SeasonPhase;
    gamesPerWeek: number;
    peakDate: ISODate;
  };
}

// ============================================================================
// Event Listener Types
// ============================================================================

/**
 * Metric update event data
 */
export interface MetricUpdateEvent {
  distance_covered?: number;
  sprint_count?: number;
  top_speed?: number;
  current_hr_zone?: number;
  avg_power?: number;
  ball_touches?: number;
}

/**
 * High risk detection event
 */
export interface HighRiskEvent {
  area: string;
  probability: number;
  action: string;
  details: string;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Deep partial (makes all properties optional recursively)
 */
export type DeepPartial<T> = {
  [P in keyof T]?: T[P] extends object ? DeepPartial<T[P]> : T[P];
};

/**
 * Required fields helper
 */
export type RequiredFields<T, K extends keyof T> = T & Required<Pick<T, K>>;

/**
 * Omit multiple keys
 */
export type OmitMultiple<T, K extends keyof T> = Omit<T, K>;

/**
 * Timestamp range query
 */
export interface TimestampRange {
  from: ISOTimestamp;
  to: ISOTimestamp;
}

/**
 * Date range query
 */
export interface DateRange {
  from: ISODate;
  to: ISODate;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Heart rate zones as percentage of max HR
 */
export const HR_ZONES = {
  ZONE_1: { min: 0, max: 50 },
  ZONE_2: { min: 50, max: 60 },
  ZONE_3: { min: 60, max: 70 },
  ZONE_4: { min: 70, max: 80 },
  ZONE_5: { min: 80, max: 90 },
  ZONE_6: { min: 90, max: 100 },
} as const;

/**
 * Training load thresholds (ACWR)
 */
export const ACWR_THRESHOLDS = {
  LOW_RISK: { min: 0.8, max: 1.3 },
  MODERATE_RISK: { min: 0.5, max: 0.8 },
  HIGH_RISK: { min: 1.5, max: Infinity },
} as const;

/**
 * Injury risk thresholds
 */
export const INJURY_RISK_THRESHOLDS = {
  LOW: 0.15,
  MODERATE: 0.40,
  HIGH: 0.70,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  UUID,
  ISOTimestamp,
  ISODate,
  Timezone,

  // Athlete
  Gender,
  ExperienceLevel,
  DominantSide,
  GoalType,
  GoalStatus,
  InjurySeverity,
  DeviceStatus,
  DataSharing,

  // Session
  SessionType,
  VenueType,
  WeatherCondition,
  ParticipantRole,
  Importance,
  Outcome,

  // Performance
  TimeSeriesPoint,
  HeartRatePoint,
  LeftRightBalance,
  LeftRightMeasurement,
  ClimbCategory,

  // Injury
  RiskLevel,
  AlertSeverity,
  AlertType,
  ModelType,

  // Training
  TrainingGoalType,
  SeasonPhase,
  MesocycleFocus,
  DayOfWeek,

  // Broadcasting
  CameraAngle,

  // API
  ExportFormat,
};

/**
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * These types enable standardized sports technology data
 * that benefits athletes at all levels, from weekend warriors
 * to Olympic champions.
 */
