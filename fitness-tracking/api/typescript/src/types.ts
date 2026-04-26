/**
 * WIA-IND-012: Fitness Tracking - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Health & Fitness Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * User profile for personalized fitness calculations
 * 사용자 프로필 - 개인화된 피트니스 계산용
 */
export interface UserProfile {
  /** Unique user identifier */
  userId: string;

  /** Age in years */
  age: number;

  /** Gender for calculation adjustments */
  gender: 'male' | 'female' | 'other';

  /** Weight in kilograms */
  weight: number;

  /** Height in centimeters */
  height: number;

  /** Resting heart rate in BPM */
  restingHeartRate?: number;

  /** Maximum heart rate in BPM (measured or calculated) */
  maxHeartRate?: number;

  /** VO2 max in ml/kg/min */
  vo2Max?: number;

  /** Fitness level */
  fitnessLevel?: 'beginner' | 'intermediate' | 'advanced' | 'elite';

  /** Activity level */
  activityLevel?: 'sedentary' | 'lightly_active' | 'moderately_active' | 'very_active' | 'extremely_active';

  /** User timezone */
  timezone?: string;

  /** Preferred units */
  preferences?: {
    distanceUnit: 'km' | 'miles';
    weightUnit: 'kg' | 'lbs';
    temperatureUnit: 'celsius' | 'fahrenheit';
  };
}

/**
 * Device information for tracking source
 * 디바이스 정보 - 추적 소스
 */
export interface DeviceInfo {
  /** Device unique identifier */
  deviceId: string;

  /** Device name/model */
  name: string;

  /** Device type */
  type: 'smartphone' | 'smartwatch' | 'fitness_tracker' | 'heart_rate_monitor' | 'smart_scale' | 'other';

  /** Manufacturer */
  manufacturer?: string;

  /** Firmware/software version */
  version?: string;

  /** Sensor capabilities */
  capabilities: {
    gps: boolean;
    heartRate: boolean;
    accelerometer: boolean;
    gyroscope: boolean;
    barometer: boolean;
    temperature: boolean;
  };

  /** Battery level (0-100) */
  batteryLevel?: number;

  /** Last sync timestamp */
  lastSync?: Date;
}

// ============================================================================
// Activity Tracking Types
// ============================================================================

/**
 * Activity type enumeration
 * 활동 유형
 */
export enum ActivityType {
  // Cardio activities
  WALKING = 'walking',
  RUNNING = 'running',
  CYCLING = 'cycling',
  SWIMMING = 'swimming',
  HIKING = 'hiking',
  ROWING = 'rowing',
  ELLIPTICAL = 'elliptical',
  STAIR_CLIMBING = 'stair_climbing',
  DANCING = 'dancing',

  // Strength training
  WEIGHT_TRAINING = 'weight_training',
  BODYWEIGHT = 'bodyweight',
  CROSSFIT = 'crossfit',
  POWERLIFTING = 'powerlifting',
  OLYMPIC_LIFTING = 'olympic_lifting',

  // Sports
  BASKETBALL = 'basketball',
  SOCCER = 'soccer',
  TENNIS = 'tennis',
  GOLF = 'golf',
  VOLLEYBALL = 'volleyball',
  BASEBALL = 'baseball',
  FOOTBALL = 'football',
  HOCKEY = 'hockey',
  MARTIAL_ARTS = 'martial_arts',
  BOXING = 'boxing',

  // Mind-body
  YOGA = 'yoga',
  PILATES = 'pilates',
  TAI_CHI = 'tai_chi',
  MEDITATION = 'meditation',
  STRETCHING = 'stretching',

  // Other
  HIIT = 'hiit',
  CIRCUIT_TRAINING = 'circuit_training',
  SPORTS_GENERAL = 'sports_general',
  OTHER = 'other'
}

/**
 * Activity intensity levels
 * 활동 강도 수준
 */
export enum ActivityIntensity {
  VERY_LIGHT = 'very_light',
  LIGHT = 'light',
  MODERATE = 'moderate',
  VIGOROUS = 'vigorous',
  VERY_VIGOROUS = 'very_vigorous',
  MAXIMUM = 'maximum'
}

/**
 * GPS coordinate point
 * GPS 좌표 포인트
 */
export interface GPSPoint {
  /** Latitude in decimal degrees */
  latitude: number;

  /** Longitude in decimal degrees */
  longitude: number;

  /** Elevation in meters above sea level */
  elevation?: number;

  /** Accuracy in meters (±) */
  accuracy?: number;

  /** Timestamp of the measurement */
  timestamp: Date;

  /** Speed in m/s (device-calculated) */
  speed?: number;

  /** Heading in degrees (0-360, where 0 is North) */
  heading?: number;
}

/**
 * Activity tracking data
 * 활동 추적 데이터
 */
export interface Activity {
  /** Unique activity identifier */
  id: string;

  /** User identifier */
  userId: string;

  /** Device used for tracking */
  deviceId?: string;

  /** Activity type */
  type: ActivityType;

  /** Activity name/title */
  name?: string;

  /** Start timestamp */
  startTime: Date;

  /** End timestamp */
  endTime: Date;

  /** Total duration in seconds */
  duration: number;

  /** Active duration (excluding pauses) in seconds */
  activeDuration?: number;

  /** Distance covered in meters */
  distance?: number;

  /** Step count */
  steps?: number;

  /** Average pace in minutes per kilometer */
  avgPace?: number;

  /** Maximum pace (fastest) in minutes per kilometer */
  maxPace?: number;

  /** Average speed in km/h */
  avgSpeed?: number;

  /** Maximum speed in km/h */
  maxSpeed?: number;

  /** Calories burned */
  calories: number;

  /** Elevation data */
  elevation?: {
    /** Total elevation gain in meters */
    gain: number;
    /** Total elevation loss in meters */
    loss: number;
    /** Minimum elevation in meters */
    min?: number;
    /** Maximum elevation in meters */
    max?: number;
  };

  /** GPS route data */
  route?: GPSPoint[];

  /** Activity metadata */
  metadata?: {
    /** Weather conditions */
    weather?: WeatherCondition;
    /** Equipment used */
    equipment?: string[];
    /** Location/venue name */
    location?: string;
    /** Notes/description */
    notes?: string;
    /** Tags */
    tags?: string[];
  };

  /** Data quality indicators */
  quality?: {
    /** GPS signal quality (0-100) */
    gpsAccuracy?: number;
    /** Data completeness (0-100) */
    completeness?: number;
    /** Reliability score (0-100) */
    reliability?: number;
  };
}

/**
 * Weather conditions during activity
 * 활동 중 날씨 상태
 */
export interface WeatherCondition {
  /** Temperature in Celsius */
  temperature: number;

  /** Feels-like temperature in Celsius */
  feelsLike?: number;

  /** Humidity percentage (0-100) */
  humidity?: number;

  /** Wind speed in m/s */
  windSpeed?: number;

  /** Wind direction in degrees (0-360) */
  windDirection?: number;

  /** Weather condition */
  condition?: 'clear' | 'cloudy' | 'rain' | 'snow' | 'fog' | 'storm';

  /** UV index */
  uvIndex?: number;

  /** Air quality index */
  aqi?: number;
}

/**
 * Step tracking data
 * 걸음 수 추적 데이터
 */
export interface StepData {
  /** Date of the step count */
  date: Date;

  /** Total steps for the day */
  steps: number;

  /** Distance calculated from steps in meters */
  distance: number;

  /** Calories burned from steps */
  calories: number;

  /** Active minutes */
  activeMinutes: number;

  /** Hourly breakdown */
  hourlyData?: Array<{
    hour: number;
    steps: number;
    calories: number;
  }>;

  /** Goals */
  goal?: {
    target: number;
    achieved: boolean;
    progress: number; // 0-100%
  };
}

// ============================================================================
// Heart Rate Types
// ============================================================================

/**
 * Heart rate zones
 * 심박수 구간
 */
export enum HeartRateZone {
  ZONE_1_RECOVERY = 1,    // 50-60% max HR
  ZONE_2_AEROBIC = 2,     // 60-70% max HR
  ZONE_3_TEMPO = 3,       // 70-80% max HR
  ZONE_4_THRESHOLD = 4,   // 80-90% max HR
  ZONE_5_MAXIMUM = 5      // 90-100% max HR
}

/**
 * Heart rate measurement
 * 심박수 측정
 */
export interface HeartRateMeasurement {
  /** Timestamp of measurement */
  timestamp: Date;

  /** Heart rate in BPM */
  bpm: number;

  /** Heart rate zone */
  zone?: HeartRateZone;

  /** Measurement quality (0-100) */
  quality?: number;

  /** Measurement source */
  source?: 'chest_strap' | 'wrist' | 'finger' | 'camera' | 'other';
}

/**
 * Heart rate zone distribution
 * 심박수 구간 분포
 */
export interface HeartRateZones {
  /** Time in Zone 1 (Recovery) in seconds */
  zone1: number;

  /** Time in Zone 2 (Aerobic) in seconds */
  zone2: number;

  /** Time in Zone 3 (Tempo) in seconds */
  zone3: number;

  /** Time in Zone 4 (Threshold) in seconds */
  zone4: number;

  /** Time in Zone 5 (Maximum) in seconds */
  zone5: number;

  /** Percentage distribution */
  distribution?: {
    zone1Percent: number;
    zone2Percent: number;
    zone3Percent: number;
    zone4Percent: number;
    zone5Percent: number;
  };
}

/**
 * Heart rate summary for activity
 * 활동에 대한 심박수 요약
 */
export interface HeartRateSummary {
  /** Average heart rate in BPM */
  avg: number;

  /** Maximum heart rate in BPM */
  max: number;

  /** Minimum heart rate in BPM */
  min: number;

  /** Resting heart rate in BPM */
  resting?: number;

  /** Heart rate reserve (max - resting) */
  reserve?: number;

  /** Zone distribution */
  zones: HeartRateZones;

  /** Detailed measurements */
  measurements?: HeartRateMeasurement[];

  /** Recovery heart rate (1-minute drop) */
  recovery?: {
    /** HR immediately after exercise */
    immediate: number;
    /** HR 1 minute after */
    oneMinute: number;
    /** Drop in BPM */
    drop: number;
  };
}

/**
 * Heart Rate Variability (HRV) data
 * 심박 변이도 데이터
 */
export interface HRVData {
  /** Timestamp of measurement */
  timestamp: Date;

  /** RMSSD (Root Mean Square of Successive Differences) in ms */
  rmssd: number;

  /** SDNN (Standard Deviation of NN intervals) in ms */
  sdnn?: number;

  /** pNN50 (% of successive RR intervals > 50ms) */
  pnn50?: number;

  /** Average RR interval in ms */
  avgRR?: number;

  /** Measurement quality (0-100) */
  quality?: number;

  /** Recovery status based on HRV */
  recoveryStatus?: 'excellent' | 'good' | 'fair' | 'poor';
}

// ============================================================================
// Calorie & Energy Types
// ============================================================================

/**
 * Calorie calculation parameters
 * 칼로리 계산 매개변수
 */
export interface CalorieCalculationParams {
  /** User profile */
  profile: UserProfile;

  /** Activity type */
  activityType: ActivityType;

  /** Duration in seconds */
  duration: number;

  /** Distance in meters (if applicable) */
  distance?: number;

  /** Intensity level */
  intensity: ActivityIntensity;

  /** Average heart rate (for HR-based calculation) */
  avgHeartRate?: number;

  /** MET value (if known) */
  met?: number;

  /** Include EPOC (Excess Post-Exercise Oxygen Consumption) */
  includeEPOC?: boolean;
}

/**
 * Calorie calculation result
 * 칼로리 계산 결과
 */
export interface CalorieResult {
  /** Total calories burned */
  total: number;

  /** Active calories (from exercise) */
  active: number;

  /** Resting calories (BMR component) */
  resting: number;

  /** EPOC calories (afterburn) */
  epoc?: number;

  /** Calculation method used */
  method: 'met' | 'heart_rate' | 'vo2' | 'hybrid';

  /** Confidence level (0-100) */
  confidence: number;

  /** Breakdown by macronutrient source (estimated) */
  macroBreakdown?: {
    /** Calories from carbohydrates */
    carbs: number;
    /** Calories from fat */
    fat: number;
    /** Calories from protein */
    protein: number;
  };
}

/**
 * Basal Metabolic Rate (BMR) calculation
 * 기초대사량 계산
 */
export interface BMRResult {
  /** BMR in kcal/day */
  bmr: number;

  /** Calculation method */
  method: 'mifflin_st_jeor' | 'harris_benedict' | 'katch_mcardle';

  /** Total Daily Energy Expenditure (TDEE) */
  tdee: number;

  /** Activity factor used */
  activityFactor: number;
}

/**
 * MET (Metabolic Equivalent of Task) value
 * 대사당량
 */
export interface METValue {
  /** Activity type */
  activity: ActivityType;

  /** MET value (1 MET = 3.5 ml O₂/kg/min) */
  value: number;

  /** Intensity level */
  intensity: ActivityIntensity;

  /** Description */
  description?: string;
}

// ============================================================================
// Workout Types
// ============================================================================

/**
 * Workout session
 * 운동 세션
 */
export interface Workout {
  /** Unique workout identifier */
  id: string;

  /** User identifier */
  userId: string;

  /** Workout type */
  type: ActivityType;

  /** Workout name/title */
  name: string;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Total duration in seconds */
  duration: number;

  /** Distance (for cardio workouts) in meters */
  distance?: number;

  /** Step count */
  steps?: number;

  /** Heart rate data */
  heartRate?: HeartRateSummary;

  /** Calories burned */
  calories: number;

  /** Elevation data */
  elevation?: {
    gain: number;
    loss: number;
  };

  /** Performance metrics */
  performance?: {
    /** Average pace (min/km) */
    avgPace?: number;
    /** Average speed (km/h) */
    avgSpeed?: number;
    /** Average cadence (steps/min or RPM) */
    avgCadence?: number;
    /** Average power (watts) */
    avgPower?: number;
    /** Normalized power */
    normalizedPower?: number;
  };

  /** GPS route */
  route?: GPSPoint[];

  /** Intervals (for interval training) */
  intervals?: Interval[];

  /** Exercises (for strength training) */
  exercises?: Exercise[];

  /** Rate of Perceived Exertion (1-10 scale) */
  perceivedExertion?: number;

  /** Training load metrics */
  trainingLoad?: {
    /** TRIMP (Training Impulse) */
    trimp?: number;
    /** Training Stress Score */
    tss?: number;
    /** Intensity Factor */
    intensityFactor?: number;
  };

  /** Notes/description */
  notes?: string;

  /** Weather conditions */
  weather?: WeatherCondition;

  /** Equipment used */
  equipment?: string[];

  /** Workout plan reference */
  planId?: string;

  /** Completed status */
  completed: boolean;
}

/**
 * Interval training segment
 * 인터벌 트레이닝 구간
 */
export interface Interval {
  /** Interval number */
  number: number;

  /** Interval type */
  type: 'warmup' | 'work' | 'rest' | 'cooldown';

  /** Duration in seconds */
  duration: number;

  /** Distance in meters */
  distance?: number;

  /** Target metrics */
  target?: {
    /** Target pace (min/km) */
    pace?: number;
    /** Target heart rate (BPM) */
    heartRate?: number;
    /** Target power (watts) */
    power?: number;
    /** Target speed (km/h) */
    speed?: number;
  };

  /** Actual achieved metrics */
  actual?: {
    /** Average pace */
    avgPace?: number;
    /** Average heart rate */
    avgHeartRate?: number;
    /** Average power */
    avgPower?: number;
    /** Average speed */
    avgSpeed?: number;
    /** Calories burned */
    calories?: number;
  };

  /** Completed status */
  completed: boolean;
}

/**
 * Strength training exercise
 * 근력 운동
 */
export interface Exercise {
  /** Exercise name */
  name: string;

  /** Exercise category */
  category: 'chest' | 'back' | 'legs' | 'shoulders' | 'arms' | 'core' | 'full_body';

  /** Muscle groups targeted */
  muscleGroups: string[];

  /** Equipment required */
  equipment?: string;

  /** Sets performed */
  sets: ExerciseSet[];

  /** Total volume (weight × reps) */
  totalVolume?: number;

  /** Notes */
  notes?: string;
}

/**
 * Exercise set
 * 운동 세트
 */
export interface ExerciseSet {
  /** Set number */
  setNumber: number;

  /** Repetitions completed */
  reps: number;

  /** Weight used (kg) */
  weight?: number;

  /** Duration (for isometric holds) in seconds */
  duration?: number;

  /** Rest time before next set in seconds */
  restTime?: number;

  /** Rate of Perceived Exertion (1-10) */
  perceivedExertion?: number;

  /** Target achieved */
  targetAchieved?: boolean;

  /** Completed status */
  completed: boolean;

  /** Notes */
  notes?: string;
}

// ============================================================================
// Health Metrics Types
// ============================================================================

/**
 * Body composition measurement
 * 신체 구성 측정
 */
export interface BodyComposition {
  /** Measurement timestamp */
  timestamp: Date;

  /** Weight in kilograms */
  weight: number;

  /** Height in centimeters */
  height: number;

  /** BMI (Body Mass Index) */
  bmi: number;

  /** Body fat percentage */
  bodyFatPercentage?: number;

  /** Body fat mass in kg */
  bodyFatMass?: number;

  /** Lean mass in kg */
  leanMass?: number;

  /** Muscle mass in kg */
  muscleMass?: number;

  /** Bone mass in kg */
  boneMass?: number;

  /** Water percentage */
  waterPercentage?: number;

  /** Visceral fat level (1-59) */
  visceralFat?: number;

  /** Basal Metabolic Rate (kcal/day) */
  basalMetabolicRate?: number;

  /** Metabolic age */
  metabolicAge?: number;

  /** Body measurements */
  measurements?: {
    /** Neck circumference in cm */
    neck?: number;
    /** Chest circumference in cm */
    chest?: number;
    /** Waist circumference in cm */
    waist?: number;
    /** Hip circumference in cm */
    hips?: number;
    /** Thigh circumference in cm */
    thigh?: number;
    /** Arm circumference in cm */
    arm?: number;
  };

  /** Measurement method */
  method?: 'bioelectrical_impedance' | 'dexa' | 'calipers' | 'manual';

  /** Device used */
  deviceId?: string;
}

/**
 * Sleep tracking data
 * 수면 추적 데이터
 */
export interface SleepSession {
  /** Unique sleep session identifier */
  id: string;

  /** User identifier */
  userId: string;

  /** Sleep start time */
  startTime: Date;

  /** Sleep end time */
  endTime: Date;

  /** Total time in bed (minutes) */
  totalDuration: number;

  /** Actual sleep time (minutes) */
  sleepDuration: number;

  /** Sleep stages */
  stages: {
    /** Awake time in minutes */
    awake: number;
    /** Light sleep in minutes */
    light: number;
    /** Deep sleep in minutes */
    deep: number;
    /** REM sleep in minutes */
    rem: number;
  };

  /** Sleep quality metrics */
  quality: {
    /** Overall sleep score (0-100) */
    score: number;
    /** Sleep efficiency (sleep time / time in bed) */
    efficiency: number;
    /** Number of interruptions */
    interruptions: number;
    /** Restlessness (movement count) */
    restlessness: number;
    /** Time to fall asleep (minutes) */
    latency?: number;
  };

  /** Heart rate during sleep */
  heartRate?: {
    /** Average HR */
    avg: number;
    /** Minimum HR */
    min: number;
    /** Maximum HR */
    max: number;
  };

  /** HRV during sleep */
  hrv?: {
    /** Average RMSSD */
    avg: number;
  };

  /** Respiratory rate (breaths per minute) */
  respiratoryRate?: number;

  /** Oxygen saturation (SpO2 %) */
  oxygenSaturation?: number;

  /** Environmental conditions */
  environment?: {
    /** Temperature in Celsius */
    temperature?: number;
    /** Humidity % */
    humidity?: number;
    /** Noise level in dB */
    noise?: number;
  };

  /** Notes */
  notes?: string;

  /** Device used */
  deviceId?: string;
}

/**
 * Recovery score calculation
 * 회복 점수 계산
 */
export interface RecoveryScore {
  /** Date of recovery score */
  date: Date;

  /** Overall recovery score (0-100) */
  score: number;

  /** Recovery status */
  status: 'fully_recovered' | 'well_recovered' | 'moderately_recovered' | 'poorly_recovered' | 'not_recovered';

  /** Component scores */
  components: {
    /** HRV score (0-100) */
    hrvScore: number;
    /** Sleep score (0-100) */
    sleepScore: number;
    /** Resting HR score (0-100) */
    rhrScore: number;
  };

  /** Recommendations */
  recommendations?: string[];

  /** Training readiness */
  trainingReadiness: 'high' | 'moderate' | 'low';
}

// ============================================================================
// Goal & Achievement Types
// ============================================================================

/**
 * Goal types
 * 목표 유형
 */
export enum GoalType {
  DAILY_STEPS = 'daily_steps',
  WEEKLY_DISTANCE = 'weekly_distance',
  MONTHLY_WORKOUTS = 'monthly_workouts',
  ACTIVE_MINUTES = 'active_minutes',
  CALORIE_BURN = 'calorie_burn',
  WEIGHT_LOSS = 'weight_loss',
  BODY_FAT = 'body_fat_reduction',
  RACE_TIME = 'race_time',
  WORKOUT_STREAK = 'workout_streak',
  CUSTOM = 'custom'
}

/**
 * Fitness goal
 * 피트니스 목표
 */
export interface FitnessGoal {
  /** Goal identifier */
  id: string;

  /** User identifier */
  userId: string;

  /** Goal type */
  type: GoalType;

  /** Goal name/title */
  name: string;

  /** Target value */
  target: number;

  /** Current value */
  current: number;

  /** Unit of measurement */
  unit: string;

  /** Goal period */
  period: 'daily' | 'weekly' | 'monthly' | 'yearly' | 'one-time';

  /** Start date */
  startDate: Date;

  /** End date (for time-bound goals) */
  endDate?: Date;

  /** Progress percentage (0-100) */
  progress: number;

  /** Goal status */
  status: 'active' | 'completed' | 'abandoned' | 'paused';

  /** Reminders */
  reminders?: Reminder[];

  /** Notes */
  notes?: string;

  /** Created timestamp */
  createdAt: Date;

  /** Last updated timestamp */
  updatedAt: Date;
}

/**
 * Reminder for goals
 * 목표 알림
 */
export interface Reminder {
  /** Reminder time */
  time: Date | string; // Can be time of day or specific date

  /** Reminder message */
  message: string;

  /** Reminder type */
  type: 'push' | 'email' | 'sms';

  /** Enabled status */
  enabled: boolean;
}

/**
 * Achievement badge
 * 성취 배지
 */
export interface Achievement {
  /** Achievement identifier */
  id: string;

  /** Achievement name */
  name: string;

  /** Description */
  description: string;

  /** Category */
  category: 'distance' | 'duration' | 'frequency' | 'milestone' | 'special';

  /** Tier/level */
  tier: 'bronze' | 'silver' | 'gold' | 'platinum';

  /** Icon/emoji */
  icon?: string;

  /** Requirements to earn */
  requirements: {
    /** Metric to track */
    metric: string;
    /** Target value */
    value: number;
    /** Comparison operator */
    operator: '>=' | '<=' | '==' | '>';
  };

  /** Date earned (null if not earned yet) */
  earnedDate?: Date;

  /** Progress toward achievement (0-100) */
  progress: number;

  /** Points awarded */
  points?: number;
}

// ============================================================================
// Summary & Analytics Types
// ============================================================================

/**
 * Daily fitness summary
 * 일일 피트니스 요약
 */
export interface DailySummary {
  /** Date */
  date: Date;

  /** Total steps */
  steps: number;

  /** Total distance in meters */
  distance: number;

  /** Total active minutes */
  activeMinutes: number;

  /** Total calories burned */
  calories: number;

  /** Number of workouts */
  workoutCount: number;

  /** Heart rate data */
  heartRate?: {
    resting: number;
    avg: number;
    max: number;
  };

  /** Sleep data */
  sleep?: {
    duration: number;
    score: number;
  };

  /** Goals achieved */
  goalsAchieved: string[];

  /** Recovery score */
  recoveryScore?: number;
}

/**
 * Weekly fitness summary
 * 주간 피트니스 요약
 */
export interface WeeklySummary {
  /** Week start date */
  weekStart: Date;

  /** Week end date */
  weekEnd: Date;

  /** Total steps */
  totalSteps: number;

  /** Total distance in meters */
  totalDistance: number;

  /** Total active minutes */
  totalActiveMinutes: number;

  /** Total calories burned */
  totalCalories: number;

  /** Number of workouts */
  totalWorkouts: number;

  /** Active days */
  activeDays: number;

  /** Average daily steps */
  avgDailySteps: number;

  /** Longest workout */
  longestWorkout?: {
    duration: number;
    type: ActivityType;
    date: Date;
  };

  /** Personal records */
  personalRecords?: PersonalRecord[];

  /** Weekly trend */
  trend: 'improving' | 'stable' | 'declining';
}

/**
 * Personal record (PR)
 * 개인 기록
 */
export interface PersonalRecord {
  /** Activity type */
  activityType: ActivityType;

  /** Metric type */
  metric: 'fastest_pace' | 'longest_distance' | 'most_calories' | 'highest_elevation';

  /** Value */
  value: number;

  /** Unit */
  unit: string;

  /** Date achieved */
  date: Date;

  /** Previous record */
  previousValue?: number;
}

// ============================================================================
// Synchronization & Export Types
// ============================================================================

/**
 * Data synchronization status
 * 데이터 동기화 상태
 */
export interface SyncStatus {
  /** Last sync timestamp */
  lastSync: Date;

  /** Sync status */
  status: 'synced' | 'syncing' | 'conflict' | 'error';

  /** Pending changes count */
  pendingChanges: number;

  /** Error message (if any) */
  error?: string;

  /** Device sync status */
  devices: Array<{
    deviceId: string;
    lastSync: Date;
    status: 'synced' | 'pending' | 'error';
  }>;
}

/**
 * Data export format
 * 데이터 내보내기 형식
 */
export enum ExportFormat {
  JSON = 'json',
  CSV = 'csv',
  GPX = 'gpx',
  TCX = 'tcx',
  FIT = 'fit'
}

/**
 * Export request
 * 내보내기 요청
 */
export interface ExportRequest {
  /** User identifier */
  userId: string;

  /** Export format */
  format: ExportFormat;

  /** Data types to export */
  dataTypes: Array<'activities' | 'workouts' | 'heart_rate' | 'sleep' | 'body_composition'>;

  /** Date range */
  dateRange: {
    start: Date;
    end: Date;
  };

  /** Include GPS data */
  includeGPS?: boolean;

  /** Include personal information */
  includePersonalInfo?: boolean;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Standard API response
 * 표준 API 응답
 */
export interface ApiResponse<T> {
  /** Response data */
  data: T;

  /** Success status */
  success: boolean;

  /** Error message (if any) */
  error?: string;

  /** Metadata */
  metadata?: {
    /** Request timestamp */
    timestamp: Date;
    /** API version */
    version: string;
    /** Request ID */
    requestId?: string;
  };
}

/**
 * Paginated response
 * 페이지네이션 응답
 */
export interface PaginatedResponse<T> {
  /** Data items */
  items: T[];

  /** Total count */
  total: number;

  /** Current page */
  page: number;

  /** Page size */
  pageSize: number;

  /** Total pages */
  totalPages: number;

  /** Has next page */
  hasNext: boolean;

  /** Has previous page */
  hasPrevious: boolean;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Date range
 * 날짜 범위
 */
export interface DateRange {
  start: Date;
  end: Date;
}

/**
 * Error codes
 * 오류 코드
 */
export enum ErrorCode {
  INVALID_INPUT = 'INVALID_INPUT',
  UNAUTHORIZED = 'UNAUTHORIZED',
  NOT_FOUND = 'NOT_FOUND',
  SYNC_ERROR = 'SYNC_ERROR',
  DEVICE_ERROR = 'DEVICE_ERROR',
  CALCULATION_ERROR = 'CALCULATION_ERROR',
  INTERNAL_ERROR = 'INTERNAL_ERROR'
}

/**
 * Validation result
 * 검증 결과
 */
export interface ValidationResult {
  /** Valid status */
  valid: boolean;

  /** Validation errors */
  errors?: string[];

  /** Validation warnings */
  warnings?: string[];
}
