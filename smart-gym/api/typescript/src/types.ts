/**
 * WIA-IND-014: Smart Gym - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Fitness Technology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Member ID - unique identifier for gym member
 */
export type MemberId = string;

/**
 * Facility ID - unique identifier for gym facility
 */
export type FacilityId = string;

/**
 * Equipment ID - unique identifier for gym equipment
 */
export type EquipmentId = string;

/**
 * Session ID - unique identifier for workout session
 */
export type SessionId = string;

/**
 * Class ID - unique identifier for group class
 */
export type ClassId = string;

// ============================================================================
// Facility Types
// ============================================================================

/**
 * Smart Gym Facility configuration
 */
export interface SmartGymFacility {
  /** Unique facility identifier */
  facilityId: FacilityId;

  /** Facility name */
  name: string;

  /** Physical location */
  location: {
    address: string;
    city: string;
    state: string;
    country: string;
    postalCode: string;
    coordinates?: {
      latitude: number;
      longitude: number;
    };
  };

  /** Facility capacity and size */
  capacity: {
    maxMembers: number;
    currentMembers: number;
    maxOccupancy: number;
    floorArea: number; // square meters
  };

  /** Operating hours */
  hours: {
    monday?: { open: string; close: string };
    tuesday?: { open: string; close: string };
    wednesday?: { open: string; close: string };
    thursday?: { open: string; close: string };
    friday?: { open: string; close: string };
    saturday?: { open: string; close: string };
    sunday?: { open: string; close: string };
    is24Hours: boolean;
  };

  /** Equipment inventory */
  equipment: {
    treadmills: number;
    bikes: number;
    rowers: number;
    ellipticals: number;
    weightMachines: number;
    freeWeightArea: boolean;
    functionalArea: boolean;
    groupExerciseRooms: number;
  };

  /** Available features */
  features: {
    virtualTraining: boolean;
    iotSensors: boolean;
    aiCoaching: boolean;
    memberApp: boolean;
    childcare: boolean;
    sauna: boolean;
    pool: boolean;
    personalTraining: boolean;
  };
}

/**
 * Real-time facility status
 */
export interface FacilityStatus {
  /** Current timestamp */
  timestamp: Date;

  /** Occupancy information */
  occupancy: {
    current: number;
    max: number;
    percentage: number;
    trend: 'increasing' | 'stable' | 'decreasing';
    forecast: {
      nextHour: number;
      peakTimeToday: { time: string; expected: number };
    };
  };

  /** Zone-specific occupancy */
  zoneOccupancy: {
    cardio: { current: number; max: number };
    weights: { current: number; max: number };
    functional: { current: number; max: number };
    groupClasses: { current: number; max: number };
    lockerRooms: { current: number; max: number };
  };

  /** Environmental conditions */
  environment: {
    temperature: number; // Celsius
    humidity: number; // percentage
    co2: number; // ppm
    pm25?: number; // μg/m³
    airQualityStatus: 'excellent' | 'good' | 'acceptable' | 'poor' | 'bad';
  };

  /** Equipment availability */
  equipment: {
    available: {
      treadmills: number;
      bikes: number;
      rowers: number;
      ellipticals: number;
    };
    inUse: {
      treadmills: number;
      bikes: number;
      rowers: number;
      ellipticals: number;
    };
    outOfService: {
      treadmills: number;
      bikes: number;
      rowers: number;
      ellipticals: number;
    };
  };
}

// ============================================================================
// Member Types
// ============================================================================

/**
 * Member profile with comprehensive information
 */
export interface MemberProfile {
  /** Unique member identifier */
  memberId: MemberId;

  /** Personal information */
  personalInfo: {
    name: string;
    email: string;
    phone: string;
    dateOfBirth: Date;
    gender: 'male' | 'female' | 'other' | 'prefer-not-to-say';
    emergencyContact: {
      name: string;
      phone: string;
      relationship: string;
    };
  };

  /** Membership details */
  membership: {
    type: 'basic' | 'premium' | 'vip' | 'corporate' | 'student' | 'family';
    status: 'active' | 'frozen' | 'expired' | 'cancelled';
    startDate: Date;
    expiryDate: Date;
    billingCycle: 'monthly' | 'quarterly' | 'annual';
    autoRenew: boolean;
    accessHours: string;
    includedFeatures: string[];
  };

  /** Physical metrics */
  physicalProfile: {
    height: number; // cm
    weight: number; // kg
    bodyFatPercentage?: number;
    muscleMass?: number;
    bmi: number;
    restingHeartRate?: number;
    bloodPressure?: {
      systolic: number;
      diastolic: number;
    };
    vo2Max?: number;
    measurementHistory: Array<{
      date: Date;
      weight: number;
      bodyFat?: number;
      muscleMass?: number;
    }>;
  };

  /** Fitness profile */
  fitnessProfile: {
    experience: 'beginner' | 'intermediate' | 'advanced' | 'athlete';
    primaryGoals: Array<
      | 'weight-loss'
      | 'muscle-gain'
      | 'strength'
      | 'endurance'
      | 'flexibility'
      | 'general-fitness'
      | 'sport-specific'
      | 'rehabilitation'
    >;
    currentFitnessLevel: {
      cardiovascular: 1 | 2 | 3 | 4 | 5;
      strength: 1 | 2 | 3 | 4 | 5;
      flexibility: 1 | 2 | 3 | 4 | 5;
      overall: number;
    };
    preferredActivities: string[];
    dislikedActivities: string[];
  };

  /** Medical information */
  medicalInfo: {
    medicalClearance: boolean;
    clearanceDate?: Date;
    clearanceExpiry?: Date;
    conditions: string[];
    injuries: InjuryRecord[];
    medications: string[];
    allergies: string[];
    restrictions: string[];
  };

  /** Training preferences */
  preferences: {
    workoutFrequency: number; // days per week
    sessionDuration: number; // minutes
    preferredTime: 'morning' | 'afternoon' | 'evening' | 'flexible';
    trainingStyle: 'solo' | 'group-class' | 'personal-trainer' | 'mixed';
    equipment: {
      cardio: boolean;
      freeWeights: boolean;
      machines: boolean;
      functional: boolean;
      groupClasses: string[];
    };
    notifications: {
      workoutReminders: boolean;
      classBookings: boolean;
      achievements: boolean;
      challenges: boolean;
      marketing: boolean;
    };
  };

  /** Activity tracking */
  activity: {
    totalWorkouts: number;
    totalMinutes: number;
    currentStreak: number;
    longestStreak: number;
    lastVisit: Date;
    avgWorkoutsPerWeek: number;
    favoriteEquipment: string[];
    peakWorkoutTime: string;
  };
}

/**
 * Injury record
 */
export interface InjuryRecord {
  type: string;
  area: string;
  date: Date;
  severity: 'minor' | 'moderate' | 'severe';
  restrictions: string[];
  recoveryStatus: 'acute' | 'recovery' | 'chronic' | 'healed';
  notes?: string;
}

/**
 * Member check-in record
 */
export interface CheckInRecord {
  checkInId: string;
  memberId: MemberId;
  facilityId: FacilityId;
  method: 'rfid' | 'mobile-app' | 'biometric' | 'qr-code' | 'manual';
  timestamp: Date;
  checkOutTime?: Date;
  duration?: number; // seconds
  location: string;
}

// ============================================================================
// Equipment Types
// ============================================================================

/**
 * Equipment type categories
 */
export type EquipmentType =
  | 'treadmill'
  | 'bike'
  | 'rower'
  | 'elliptical'
  | 'cable-machine'
  | 'leg-press'
  | 'chest-press'
  | 'lat-pulldown'
  | 'smith-machine'
  | 'free-weights'
  | 'functional-trainer'
  | 'other';

/**
 * Equipment status
 */
export type EquipmentStatus = 'available' | 'in-use' | 'reserved' | 'maintenance' | 'out-of-service';

/**
 * Connected equipment device
 */
export interface Equipment {
  /** Unique equipment identifier */
  equipmentId: EquipmentId;

  /** Equipment type */
  type: EquipmentType;

  /** Brand and model */
  brand: string;
  model: string;

  /** Current status */
  status: EquipmentStatus;

  /** Location within facility */
  location: {
    zone: string;
    position: string;
    coordinates?: { x: number; y: number };
  };

  /** Capabilities */
  capabilities: {
    heartRate: boolean;
    powerMeter: boolean;
    incline?: boolean;
    resistance?: boolean;
    programs?: string[];
    connectivity: ('bluetooth' | 'wifi' | 'nfc' | 'rfid')[];
  };

  /** Technical specifications */
  specs: {
    maxWeight?: number; // kg
    maxSpeed?: number; // km/h
    resistanceLevels?: number;
    weightCapacity?: number; // kg
  };

  /** Maintenance information */
  maintenance: {
    lastService: Date;
    nextService: Date;
    totalHours: number;
    totalSessions: number;
    healthScore: number; // 0-100
  };

  /** Current user (if in use) */
  currentUser?: {
    memberId: MemberId;
    startTime: Date;
    estimatedEndTime: Date;
  };
}

/**
 * Equipment reservation
 */
export interface EquipmentReservation {
  reservationId: string;
  equipmentId: EquipmentId;
  memberId: MemberId;
  startTime: Date;
  duration: number; // minutes
  status: 'pending' | 'confirmed' | 'in-progress' | 'completed' | 'cancelled';
}

// ============================================================================
// Workout Session Types
// ============================================================================

/**
 * Workout session type
 */
export type WorkoutType =
  | 'strength-training'
  | 'cardio'
  | 'hiit'
  | 'functional'
  | 'flexibility'
  | 'mixed'
  | 'group-class'
  | 'personal-training';

/**
 * Complete workout session
 */
export interface WorkoutSession {
  /** Unique session identifier */
  sessionId: SessionId;

  /** Member and facility */
  memberId: MemberId;
  facilityId: FacilityId;

  /** Timing */
  timestamp: {
    start: Date;
    end?: Date;
    duration?: number; // seconds
  };

  /** Session type */
  type: WorkoutType;

  /** Exercises performed */
  exercises: Exercise[];

  /** Session summary */
  summary?: {
    totalExercises: number;
    totalSets: number;
    totalReps: number;
    totalVolume: number; // kg
    avgHeartRate?: number;
    maxHeartRate?: number;
    caloriesBurned: number;
    timeUnderTension?: number; // seconds
    avgRestTime?: number; // seconds
  };

  /** Pre and post metrics */
  preWorkout?: {
    weight: number;
    heartRate: number;
    bloodPressure?: { systolic: number; diastolic: number };
    readinessScore?: number;
  };

  postWorkout?: {
    weight: number;
    heartRate: number;
    perceivedExertion: number; // 1-10
    muscleSoreness: number; // 1-10
  };

  /** Achievements unlocked */
  achievements?: Achievement[];

  /** Session notes */
  notes?: string;
}

/**
 * Individual exercise within session
 */
export interface Exercise {
  /** Exercise ID */
  exerciseId: string;

  /** Exercise name */
  name: string;

  /** Equipment used */
  equipment: EquipmentType | string;

  /** Muscle groups targeted */
  muscleGroups: string[];

  /** Sets performed */
  sets: ExerciseSet[];

  /** Total volume for exercise */
  totalVolume: number; // kg

  /** Estimated 1RM */
  estimated1RM?: number;

  /** Notes */
  notes?: string;
}

/**
 * Individual set within exercise
 */
export interface ExerciseSet {
  /** Set number */
  setNumber: number;

  /** Set type */
  type: 'warmup' | 'working' | 'dropset' | 'backoff' | 'amrap' | 'failure';

  /** Weight used */
  weight: number; // kg

  /** Repetitions completed */
  reps: number;

  /** Rate of Perceived Exertion (1-10) */
  rpe?: number;

  /** Rest time after set */
  restTime?: number; // seconds

  /** Form score (0-10) from computer vision */
  formScore?: number;

  /** Velocity metrics (if available) */
  avgVelocity?: number; // m/s
  peakVelocity?: number; // m/s

  /** Range of motion */
  rangeOfMotion?: 'full' | 'partial' | string;

  /** Tempo (seconds for each phase) */
  tempo?: {
    eccentric: number;
    pause: number;
    concentric: number;
  };

  /** Notes for this set */
  notes?: string;
}

// ============================================================================
// Cardio Equipment Data Types
// ============================================================================

/**
 * Treadmill workout data
 */
export interface TreadmillData {
  timestamp: Date;
  deviceId: EquipmentId;
  memberId: MemberId;

  /** Speed in km/h */
  speed: number;

  /** Incline percentage */
  incline: number;

  /** Distance in meters */
  distance: number;

  /** Duration in seconds */
  duration: number;

  /** Heart rate in bpm */
  heartRate?: number;

  /** Calories burned */
  caloriesBurned: number;

  /** Step metrics */
  stepCount?: number;
  avgStepLength?: number; // meters
  cadence?: number; // steps per minute

  /** Form analysis */
  footstrikePattern?: 'heel' | 'midfoot' | 'forefoot';
  verticalOscillation?: number; // cm
  groundContactTime?: number; // milliseconds
}

/**
 * Exercise bike workout data
 */
export interface BikeData {
  timestamp: Date;
  deviceId: EquipmentId;
  memberId: MemberId;

  /** Resistance level */
  resistance: number;

  /** Cadence in RPM */
  cadence: number;

  /** Power output in watts */
  power: number;

  /** Distance in meters */
  distance: number;

  /** Duration in seconds */
  duration: number;

  /** Heart rate in bpm */
  heartRate?: number;

  /** Calories burned */
  caloriesBurned: number;

  /** Performance metrics */
  ftp?: number; // Functional Threshold Power
  normalizedPower?: number;
  intensityFactor?: number;
  trainingStressScore?: number;
}

/**
 * Rowing machine workout data
 */
export interface RowerData {
  timestamp: Date;
  deviceId: EquipmentId;
  memberId: MemberId;

  /** Stroke rate (strokes per minute) */
  strokeRate: number;

  /** Pace (time per 500m) */
  pace: string; // "1:52.3" format

  /** Distance in meters */
  distance: number;

  /** Duration in seconds */
  duration: number;

  /** Power output in watts */
  power: number;

  /** Calories per hour */
  caloriesPerHour: number;

  /** Heart rate in bpm */
  heartRate?: number;

  /** Split times */
  splitTimes?: {
    '500m': string[];
    '1000m': string[];
  };

  /** Stroke analysis */
  strokeAnalysis?: {
    driveTime: number; // seconds
    recoveryTime: number; // seconds
    driveRecoveryRatio: number;
    peakForce: number; // newtons
  };
}

// ============================================================================
// Workout Plan Types
// ============================================================================

/**
 * AI-generated workout plan
 */
export interface WorkoutPlan {
  /** Plan ID */
  planId: string;

  /** Member ID */
  memberId: MemberId;

  /** Plan metadata */
  name: string;
  description: string;
  duration: number; // weeks

  /** Goals */
  goals: string[];

  /** Difficulty level */
  difficulty: 'beginner' | 'intermediate' | 'advanced';

  /** Weekly schedule */
  weeks: WeeklyPlan[];

  /** Created by AI or trainer */
  createdBy: 'ai' | 'trainer';
  createdAt: Date;

  /** Plan status */
  status: 'active' | 'completed' | 'paused' | 'cancelled';

  /** Progress tracking */
  progress?: {
    currentWeek: number;
    completedWorkouts: number;
    totalWorkouts: number;
    adherenceRate: number; // percentage
  };
}

/**
 * Weekly plan structure
 */
export interface WeeklyPlan {
  weekNumber: number;
  focus: string;
  sessions: WorkoutSessionPlan[];
  notes?: string;
}

/**
 * Individual workout session plan
 */
export interface WorkoutSessionPlan {
  day: string; // "Monday", "Tuesday", etc.
  type: WorkoutType;
  focus: string; // "Upper Body Strength", "HIIT Cardio", etc.
  duration: number; // minutes
  exercises: PlannedExercise[];
  targetMetric?: string;
  targetValue?: number | string;
  trainingLoad?: number; // Arbitrary Units
}

/**
 * Planned exercise
 */
export interface PlannedExercise {
  name: string;
  equipment: EquipmentType | string;
  muscleGroups: string[];
  sets: number;
  reps: number | string; // can be "8-12" or "AMRAP"
  intensity: string; // "70% 1RM", "RPE 7", etc.
  restTime: number; // seconds
  tempo?: string; // "3-1-1" format
  notes?: string;
  alternatives?: string[]; // alternative exercises
}

// ============================================================================
// Virtual Training Types
// ============================================================================

/**
 * Group fitness class
 */
export interface GroupClass {
  /** Class ID */
  classId: ClassId;

  /** Class details */
  name: string;
  description: string;
  type: 'spin' | 'yoga' | 'hiit' | 'pilates' | 'strength' | 'dance' | 'other';
  difficulty: 'beginner' | 'intermediate' | 'advanced' | 'all-levels';

  /** Instructor */
  instructor: {
    id: string;
    name: string;
    bio?: string;
    specialties: string[];
  };

  /** Schedule */
  schedule: {
    date: Date;
    startTime: string;
    endTime: string;
    duration: number; // minutes
  };

  /** Capacity */
  capacity: {
    max: number;
    current: number;
    waitlist: number;
  };

  /** Location */
  location: {
    facilityId?: FacilityId;
    room?: string;
    virtual: boolean;
  };

  /** Equipment required */
  equipmentRequired: string[];

  /** Participants */
  participants: MemberId[];
  waitlist: MemberId[];

  /** Status */
  status: 'scheduled' | 'in-progress' | 'completed' | 'cancelled';

  /** Recording (if virtual) */
  recording?: {
    available: boolean;
    url?: string;
    expiryDate?: Date;
  };
}

/**
 * Live streaming session
 */
export interface LiveStreamSession {
  sessionId: string;
  classId: ClassId;

  /** Stream details */
  streamUrl: string;
  chatEnabled: boolean;
  quality: '720p' | '1080p' | '4K';

  /** Participants */
  liveParticipants: number;
  peakViewers: number;

  /** Interaction */
  reactions: Record<string, number>; // emoji reactions
  messages: ChatMessage[];

  /** Metrics overlay */
  metricsOverlay: boolean;
  leaderboardEnabled: boolean;
}

/**
 * Chat message
 */
export interface ChatMessage {
  messageId: string;
  memberId: MemberId;
  memberName: string;
  message: string;
  timestamp: Date;
  type: 'text' | 'emoji' | 'system';
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Member analytics
 */
export interface MemberAnalytics {
  memberId: MemberId;
  period: string; // "7days", "30days", "90days", "1year"

  /** Workout statistics */
  workouts: {
    total: number;
    byType: Record<WorkoutType, number>;
    avgPerWeek: number;
    totalMinutes: number;
    avgDuration: number;
  };

  /** Performance metrics */
  performance: {
    strengthProgress: number; // percentage increase
    cardioProgress: number; // percentage increase
    avgHeartRate: number;
    totalCalories: number;
    totalVolume: number; // kg
  };

  /** Attendance */
  attendance: {
    daysAttended: number;
    currentStreak: number;
    longestStreak: number;
    consistencyScore: number; // 0-100
  };

  /** Body composition */
  bodyComposition?: {
    weightChange: number; // kg
    bodyFatChange: number; // percentage
    muscleMassChange: number; // kg
  };

  /** Personal records */
  personalRecords: PersonalRecord[];
}

/**
 * Personal record
 */
export interface PersonalRecord {
  exercise: string;
  metric: 'weight' | 'reps' | 'distance' | 'time';
  value: number;
  date: Date;
  previousRecord?: {
    value: number;
    date: Date;
  };
}

/**
 * Facility analytics
 */
export interface FacilityAnalytics {
  facilityId: FacilityId;
  period: string;

  /** Membership statistics */
  membership: {
    total: number;
    active: number;
    new: number;
    cancelled: number;
    retentionRate: number; // percentage
  };

  /** Attendance */
  attendance: {
    totalVisits: number;
    uniqueMembers: number;
    avgVisitsPerMember: number;
    peakHours: Array<{ hour: string; avgOccupancy: number }>;
    peakDays: Array<{ day: string; avgOccupancy: number }>;
  };

  /** Equipment usage */
  equipmentUsage: {
    mostPopular: Array<{ type: string; hours: number }>;
    utilizationRate: number; // percentage
    avgWaitTime: number; // minutes
  };

  /** Revenue */
  revenue?: {
    total: number;
    byMembershipType: Record<string, number>;
    personalTraining: number;
    otherServices: number;
  };

  /** Classes */
  classes: {
    totalClasses: number;
    totalAttendees: number;
    avgAttendance: number;
    popularClasses: Array<{ name: string; avgAttendance: number }>;
  };
}

// ============================================================================
// Safety & Monitoring Types
// ============================================================================

/**
 * Form analysis result
 */
export interface FormAnalysis {
  sessionId: SessionId;
  exerciseId: string;
  timestamp: Date;

  /** Computer vision pose detection */
  poseKeypoints: Record<string, { x: number; y: number; confidence: number }>;

  /** Joint angles */
  angles: {
    kneeAngle?: { left: number; right: number };
    hipAngle?: { left: number; right: number };
    spineAngle?: number;
    shoulderAngle?: { left: number; right: number };
  };

  /** Form score */
  formScore: {
    overall: number; // 0-10
    components: {
      depth?: number;
      barPath?: number;
      symmetry?: number;
      tempo?: number;
      range?: number;
    };
  };

  /** Warnings */
  warnings: Array<{
    severity: 'info' | 'warning' | 'danger';
    issue: string;
    description: string;
    recommendation: string;
  }>;

  /** Injury risk */
  injuryRisk: {
    overall: number; // 0-1
    areas: Record<string, number>;
    recommendation: 'continue' | 'reduce-weight' | 'stop-immediately';
  };
}

/**
 * Injury risk assessment
 */
export interface InjuryRiskAssessment {
  memberId: MemberId;
  timestamp: Date;

  /** Input factors */
  inputFactors: {
    trainingLoad: {
      acuteLoad: number;
      chronicLoad: number;
      acuteChronicRatio: number;
    };
    recoveryMetrics: {
      sleepQuality: number;
      sleepDuration: number;
      muscularSoreness: number;
      hrvScore?: number;
      restingHeartRate?: number;
    };
    biomechanics: {
      movementAsymmetry: number;
      formScoreAvg: number;
      rangeOfMotionDeficits: string[];
    };
    history: {
      previousInjuries: InjuryRecord[];
      ageFactorRisk: number;
    };
  };

  /** Risk scores */
  riskScores: {
    overall: number; // 0-1
    areas: Record<string, number>;
    timeframe: '24hours' | '3days' | '1week';
  };

  /** Recommendations */
  recommendations: {
    action: 'train-normally' | 'modify-workout' | 'active-recovery' | 'full-rest';
    modifications: string[];
    focusRecovery: string[];
    medicalConsult: boolean;
  };
}

/**
 * Equipment maintenance prediction
 */
export interface MaintenancePrediction {
  equipmentId: EquipmentId;
  timestamp: Date;

  /** Usage metrics */
  usageMetrics: {
    totalHours: number;
    totalSessions: number;
    avgLoadPerSession: number;
    peakLoad: number;
  };

  /** Sensor data */
  sensorData: {
    vibration?: number;
    temperature?: number;
    soundLevel?: number;
    calibrationDrift?: number;
  };

  /** Health score */
  healthScore: number; // 0-100

  /** Prediction */
  prediction: {
    estimatedRemainingLife: number; // hours
    failureRisk: {
      next7Days: number;
      next30Days: number;
      next90Days: number;
    };
    recommendedAction: {
      urgency: 'immediate' | 'soon' | 'scheduled' | 'monitor';
      task: 'repair' | 'replace-part' | 'full-service' | 'calibration';
      estimatedDowntime: number; // hours
      costEstimate?: number;
    };
  };

  /** Maintenance history */
  maintenanceHistory: Array<{
    date: Date;
    type: 'routine' | 'repair' | 'replacement';
    description: string;
    cost: number;
    performedBy: string;
  }>;
}

// ============================================================================
// Achievement Types
// ============================================================================

/**
 * Member achievement
 */
export interface Achievement {
  achievementId: string;
  type:
    | 'personal-record'
    | 'streak'
    | 'milestone'
    | 'volume'
    | 'consistency'
    | 'class-attendance'
    | 'special';

  name: string;
  description: string;
  dateEarned: Date;

  /** Achievement details */
  details?: {
    exercise?: string;
    value?: number;
    unit?: string;
    category?: string;
  };

  /** Badge/icon */
  badge?: {
    icon: string;
    color: string;
    tier: 'bronze' | 'silver' | 'gold' | 'platinum';
  };
}

// ============================================================================
// Configuration Types
// ============================================================================

/**
 * Smart Gym SDK configuration
 */
export interface SmartGymConfig {
  facilityId: FacilityId;
  apiEndpoint?: string;
  apiKey?: string;
  enableRealtime?: boolean;
  enableAnalytics?: boolean;
  enableVideoAnalysis?: boolean;
}

/**
 * Equipment manager configuration
 */
export interface EquipmentManagerConfig {
  autoReservation?: boolean;
  maxReservationDuration?: number; // minutes
  notifyOnAvailable?: boolean;
}

/**
 * Member manager configuration
 */
export interface MemberManagerConfig {
  autoCheckout?: boolean;
  checkoutTimeout?: number; // minutes
  sendNotifications?: boolean;
}

/**
 * Workout planner configuration
 */
export interface WorkoutPlannerConfig {
  model?: string;
  personalization?: boolean;
  adaptiveDifficulty?: boolean;
  injuryConsideration?: boolean;
}

/**
 * Facility monitor configuration
 */
export interface FacilityMonitorConfig {
  updateInterval?: number; // seconds
  enableEnvironmental?: boolean;
  enableOccupancy?: boolean;
  enableEquipment?: boolean;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Real-time event types
 */
export type SmartGymEvent =
  | { type: 'member:checkin'; data: CheckInRecord }
  | { type: 'member:checkout'; data: CheckInRecord }
  | { type: 'equipment:status'; data: Equipment }
  | { type: 'workout:start'; data: WorkoutSession }
  | { type: 'workout:update'; data: WorkoutSession }
  | { type: 'workout:complete'; data: WorkoutSession }
  | { type: 'form:warning'; data: FormAnalysis }
  | { type: 'injury:risk'; data: InjuryRiskAssessment }
  | { type: 'facility:occupancy'; data: FacilityStatus }
  | { type: 'class:start'; data: GroupClass }
  | { type: 'achievement:unlocked'; data: Achievement };

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Standard API response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Date;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalItems: number;
    totalPages: number;
  };
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  // Add any additional exports as needed
};
