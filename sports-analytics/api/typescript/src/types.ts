/**
 * WIA-IND-013: Sports Analytics TypeScript Type Definitions
 *
 * @standard WIA-IND-013
 * @version 1.0.0
 * @description Comprehensive type definitions for sports analytics
 *
 * 弘益人間 (Benefit All Humanity)
 * Making sports analytics accessible to all levels of competition
 *
 * © 2025 SmileStory Inc. / WIA
 * MIT License
 */

// ============================================================================
// Core Types
// ============================================================================

export type SportType =
  | 'football' // Soccer
  | 'basketball'
  | 'baseball'
  | 'american_football'
  | 'ice_hockey'
  | 'tennis'
  | 'cricket'
  | 'volleyball'
  | 'rugby'
  | 'handball';

export type PositionType =
  // Football/Soccer
  | 'goalkeeper'
  | 'defender'
  | 'midfielder'
  | 'forward'
  | 'center_back'
  | 'fullback'
  | 'wing_back'
  | 'defensive_midfielder'
  | 'central_midfielder'
  | 'attacking_midfielder'
  | 'winger'
  | 'striker'
  // Basketball
  | 'point_guard'
  | 'shooting_guard'
  | 'small_forward'
  | 'power_forward'
  | 'center'
  // Baseball
  | 'pitcher'
  | 'catcher'
  | 'first_base'
  | 'second_base'
  | 'third_base'
  | 'shortstop'
  | 'outfield';

export type EventType =
  | 'pass'
  | 'shot'
  | 'goal'
  | 'save'
  | 'tackle'
  | 'interception'
  | 'foul'
  | 'card'
  | 'substitution'
  | 'offside'
  | 'corner'
  | 'free_kick'
  | 'penalty'
  | 'throw_in'
  | 'goal_kick'
  | 'dribble'
  | 'cross'
  | 'clearance'
  | 'block'
  | 'aerial_duel'
  | 'turnover'
  | 'recovery';

export type EventOutcome = 'success' | 'failure' | 'neutral';

export type CardType = 'yellow' | 'red' | 'yellow_red';

export type BodyPart = 'right_foot' | 'left_foot' | 'head' | 'chest' | 'other';

export type PassType =
  | 'short'
  | 'long'
  | 'through_ball'
  | 'cross'
  | 'corner'
  | 'free_kick'
  | 'throw_in'
  | 'chipped'
  | 'back_pass'
  | 'progressive';

// ============================================================================
// Player Types
// ============================================================================

export interface PlayerId {
  id: string;
  name: string;
}

export interface Player {
  id: string;
  name: string;
  jerseyNumber: number;
  position: PositionType;
  dateOfBirth: string; // ISO 8601 date
  nationality: string;
  height: number; // cm
  weight: number; // kg
  preferredFoot?: 'left' | 'right' | 'both';
  marketValue?: number; // in euros
  contractExpiry?: string; // ISO 8601 date
}

export interface PlayerPerformanceRating {
  playerId: string;
  ppr: number; // 0-100
  breakdown: {
    skills: number; // 0-100
    impact: number; // 0-100
    consistency: number; // 0-100
    fitness: number; // 0-100
  };
  sportSpecific: Record<string, number>;
  percentile: number; // vs position peers
  trend: 'improving' | 'stable' | 'declining';
  confidence: number; // 0-1
}

export interface PlayerStatistics {
  playerId: string;
  matchId?: string;
  season?: string;

  // General
  gamesPlayed: number;
  minutesPlayed: number;
  gamesStarted: number;

  // Offensive (Football)
  goals?: number;
  assists?: number;
  shots?: number;
  shotsOnTarget?: number;
  xG?: number; // Expected goals
  xA?: number; // Expected assists
  keyPasses?: number;
  bigChances?: number;

  // Passing
  passes?: number;
  passesCompleted?: number;
  passAccuracy?: number; // percentage
  progressivePasses?: number;
  longPasses?: number;
  crosses?: number;

  // Defensive
  tackles?: number;
  tacklesWon?: number;
  interceptions?: number;
  clearances?: number;
  blocks?: number;
  aerialDuels?: number;
  aerialDuelsWon?: number;

  // Physical
  distanceCovered?: number; // km
  sprintDistance?: number; // km
  topSpeed?: number; // km/h
  accelerations?: number;
  decelerations?: number;

  // Disciplinary
  foulsCommitted?: number;
  foulsSuffered?: number;
  yellowCards?: number;
  redCards?: number;

  // Goalkeeper specific
  saves?: number;
  cleanSheets?: number;
  goalsConceded?: number;
  savePercentage?: number;
  xGAgainst?: number;

  // Basketball specific
  points?: number;
  rebounds?: number;
  offensiveRebounds?: number;
  defensiveRebounds?: number;
  steals?: number;
  turnovers?: number;
  fieldGoalPercentage?: number;
  threePointPercentage?: number;
  freeThrowPercentage?: number;
}

export interface PlayerDevelopment {
  playerId: string;
  currentRating: number; // 0-100
  potentialRating: number; // 0-100
  peakAge: number;
  currentAge: number;
  growthCurve: {
    age: number;
    expectedRating: number;
  }[];
  developmentRate: number; // rating points per year
  confidence: number; // 0-1
}

export interface PlayerComparison {
  players: PlayerId[];
  metrics: {
    name: string;
    values: number[]; // one per player
    normalized: number[]; // percentile or z-score
  }[];
  similarityScores?: number[][]; // pairwise similarity matrix
  visualization: 'radar' | 'bar' | 'scatter' | 'table';
}

// ============================================================================
// Team Types
// ============================================================================

export interface Team {
  id: string;
  name: string;
  shortName: string;
  logo?: string;
  founded?: number;
  venue?: string;
  league?: string;
  country: string;
}

export interface TeamPerformance {
  teamId: string;
  season: string;

  // Overall ratings
  teamStrength: number; // 0-100
  attackRating: number; // 0-100
  defenseRating: number; // 0-100
  synergyScore: number; // 0-100

  // League performance
  position?: number;
  played: number;
  won: number;
  drawn: number;
  lost: number;
  goalsFor: number;
  goalsAgainst: number;
  goalDifference: number;
  points: number;
  pointsPerGame: number;

  // Advanced metrics
  xGFor: number;
  xGAgainst: number;
  xGDifference: number;
  possessionAvg: number; // percentage
  ppda: number; // Passes allowed per defensive action

  // Form
  last5: ('W' | 'D' | 'L')[];
  homeRecord: { W: number; D: number; L: number };
  awayRecord: { W: number; D: number; L: number };
}

export interface Formation {
  formation: string; // e.g., "4-3-3", "4-4-2"
  players: {
    playerId: string;
    position: PositionType;
    averagePosition: Coordinates;
  }[];
  compactness: {
    horizontal: number; // meters
    vertical: number; // meters
  };
  defensiveLineHeight: number; // meters from own goal
  confidence: number; // 0-1
}

// ============================================================================
// Match Types
// ============================================================================

export interface Match {
  id: string;
  date: string; // ISO 8601 datetime
  competition: string;
  season: string;
  round?: string;

  homeTeam: Team;
  awayTeam: Team;

  venue: string;
  attendance?: number;
  referee?: string;

  status: 'scheduled' | 'in_progress' | 'completed' | 'postponed' | 'cancelled';

  score?: {
    home: number;
    away: number;
    halfTime?: { home: number; away: number };
  };

  weather?: WeatherConditions;
}

export interface WeatherConditions {
  temperature: number; // Celsius
  humidity: number; // percentage
  windSpeed: number; // km/h
  precipitation: number; // mm
  condition: 'clear' | 'cloudy' | 'rain' | 'snow' | 'fog';
}

export interface Coordinates {
  x: number; // meters
  y: number; // meters
  z?: number; // meters (for ball height, jumps)
}

export interface MatchEvent {
  id: string;
  matchId: string;
  timestamp: number; // milliseconds from kickoff
  minute: number;
  second: number;
  period: 1 | 2 | 3 | 4; // half/quarter depending on sport

  type: EventType;
  team: Team;
  player: Player;

  location: Coordinates;
  endLocation?: Coordinates;

  outcome: EventOutcome;

  metadata?: {
    passType?: PassType;
    bodyPart?: BodyPart;
    xG?: number;
    shotSpeed?: number; // km/h
    distance?: number; // meters
    duration?: number; // seconds
    progressive?: boolean;
    underPressure?: boolean;
    assistedBy?: string; // player ID
    cardType?: CardType;
  };

  relatedEvents?: string[]; // event IDs

  context: {
    score: { home: number; away: number };
    possessionTeam: string; // team ID
  };
}

export interface TrackingFrame {
  matchId: string;
  timestamp: number; // milliseconds from kickoff
  frameNumber: number;

  ball: {
    position: Coordinates;
    velocity?: Coordinates; // m/s
    possession?: string; // player ID or null
  };

  players: {
    playerId: string;
    teamId: string;
    position: Coordinates;
    velocity?: number; // m/s
    acceleration?: number; // m/s²
    direction?: number; // degrees (0-360)
    heartRate?: number; // bpm
  }[];
}

export interface MatchAnalysis {
  matchId: string;

  possession: {
    home: number; // percentage
    away: number; // percentage
    byThird: {
      home: { own: number; middle: number; attacking: number };
      away: { own: number; middle: number; attacking: number };
    };
  };

  xG: {
    home: number;
    away: number;
  };

  shots: {
    home: { total: number; onTarget: number; offTarget: number; blocked: number };
    away: { total: number; onTarget: number; offTarget: number; blocked: number };
  };

  passing: {
    home: { total: number; completed: number; accuracy: number };
    away: { total: number; completed: number; accuracy: number };
  };

  tackles: {
    home: { total: number; won: number };
    away: { total: number; won: number };
  };

  formations: {
    home: Formation;
    away: Formation;
  };

  momentum: {
    current: number; // -1 (fully away) to +1 (fully home)
    timeline: { timestamp: number; value: number }[];
  };

  keyMoments: {
    timestamp: number;
    type: 'goal' | 'penalty' | 'red_card' | 'substitution' | 'var_decision';
    description: string;
    importanceScore: number; // 0-1
  }[];
}

// ============================================================================
// Prediction Types
// ============================================================================

export interface MatchPrediction {
  matchId: string;
  predictedAt: string; // ISO 8601 datetime

  winProbabilities: {
    home: number; // 0-100 percentage
    draw: number;
    away: number;
  };

  expectedScore: {
    home: number;
    away: number;
  };

  scoreProbabilities: {
    score: string; // e.g., "2-1"
    probability: number;
  }[];

  confidence: number; // 0-1

  factors: {
    eloRatings: { home: number; away: number };
    recentForm: { home: number; away: number };
    headToHead: string;
    homeAdvantage: number;
    injuries?: string[];
    weather?: WeatherConditions;
  };
}

export interface LiveWinProbability {
  matchId: string;
  timestamp: number;
  currentScore: { home: number; away: number };

  winProbabilities: {
    home: number;
    draw: number;
    away: number;
  };

  changeFromKickoff: {
    home: number; // percentage points
    draw: number;
    away: number;
  };

  momentum: number; // -1 to +1
}

export interface PlayerPerformancePrediction {
  playerId: string;
  matchId: string;

  predicted: {
    ppr: number;
    goals: number;
    assists: number;
    shots: number;
    keyPasses: number;
    tackles: number;
    minutesPlayed: number;
  };

  confidenceIntervals: {
    metric: string;
    lower: number; // 10th percentile
    expected: number; // 50th percentile
    upper: number; // 90th percentile
  }[];

  factors: {
    recentForm: number;
    opponentStrength: number;
    venue: 'home' | 'away';
    fatigue: number; // 0-10
  };
}

export interface SeasonProjection {
  teamId: string;
  season: string;

  projectedPoints: number;
  projectedPosition: number;

  positionProbabilities: {
    position: number;
    probability: number;
  }[];

  outcomes: {
    championsProbability?: number;
    europeProbability?: number;
    relegationProbability?: number;
  };

  remainingMatches: {
    opponent: string;
    date: string;
    winProb: number;
    drawProb: number;
    lossProb: number;
    expectedPoints: number;
  }[];

  confidence: number; // 0-1
}

// ============================================================================
// Injury & Health Types
// ============================================================================

export interface InjuryRisk {
  playerId: string;
  assessmentDate: string;

  riskScore: number; // 0-10
  riskLevel: 'low' | 'moderate' | 'high' | 'very_high';

  factors: {
    workload: number; // 0-10
    fatigue: number; // 0-10
    history: number; // 0-10
    biomechanics: number; // 0-10
    age: number; // 0-10
  };

  acwr: number; // Acute:Chronic Workload Ratio

  recommendations: {
    action: string;
    priority: 'low' | 'medium' | 'high';
  }[];

  nextAssessment: string; // ISO 8601 date
}

export interface WorkloadData {
  playerId: string;
  date: string;

  external: {
    totalDistance: number; // km
    highSpeedDistance: number; // km
    sprintDistance: number; // km
    accelerations: number;
    decelerations: number;
    playerLoad: number; // arbitrary units
  };

  internal: {
    averageHeartRate: number; // bpm
    maxHeartRate: number; // bpm
    hrvRmssd: number; // ms
    sessionRPE: number; // 0-10
    duration: number; // minutes
  };

  acuteLoad: number; // 7-day rolling
  chronicLoad: number; // 28-day rolling
  acwr: number;
}

export interface RecoveryMetrics {
  playerId: string;
  date: string;

  recoveryScore: number; // 0-100

  sleep: {
    duration: number; // hours
    quality: number; // 0-100
    deepSleep: number; // percentage
    remSleep: number; // percentage
  };

  hrv: {
    rmssd: number; // ms
    baselineDeviation: number; // percentage
  };

  soreness: {
    overall: number; // 0-10
    specific?: {
      bodyPart: string;
      level: number; // 0-10
    }[];
  };

  mood: number; // 0-10
  readiness: number; // 0-10

  recommendation: 'full_training' | 'modified_training' | 'recovery_session' | 'rest';
}

export interface BiomechanicalData {
  playerId: string;
  sessionDate: string;

  gait: {
    strideLength: number; // meters
    strideFrequency: number; // steps/min
    groundContactTime: number; // ms
    flightTime: number; // ms
    verticalOscillation: number; // cm
    legStiffness: number; // kN/m
    asymmetryIndex: number; // percentage
  };

  jump: {
    cmjHeight: number; // cm
    rsi: number; // reactive strength index
    landingAsymmetry: number; // percentage
    kneeValgus: number; // degrees
  };

  mobility: {
    hipFlexion: number; // degrees
    ankleFlexion: number; // degrees
    shoulderRotation: number; // degrees
  };

  riskFlags: {
    issue: string;
    severity: 'low' | 'medium' | 'high';
  }[];
}

// ============================================================================
// Video Analytics Types
// ============================================================================

export interface VideoMetadata {
  videoId: string;
  matchId: string;
  duration: number; // seconds
  resolution: string; // e.g., "1920x1080"
  fps: number;

  camera: {
    type: 'broadcast' | 'tactical' | 'behind_goal' | 'player_cam';
    angle: number; // degrees
    height: number; // meters
  };
}

export interface AutomatedEventDetection {
  videoId: string;

  detectedEvents: {
    eventType: EventType;
    timestamp: number; // seconds into video
    confidence: number; // 0-1
    boundingBox?: {
      x: number;
      y: number;
      width: number;
      height: number;
    };
  }[];

  accuracy: {
    precision: number;
    recall: number;
    f1Score: number;
  };
}

export interface HeatMap {
  playerId: string;
  matchId: string;

  grid: {
    resolution: { x: number; y: number }; // cells
    cells: {
      x: number;
      y: number;
      value: number; // time or weighted actions
    }[];
  };

  pitchDimensions: {
    length: number; // meters
    width: number; // meters
  };

  normalized: boolean; // per 90 minutes
}

export interface PassNetwork {
  matchId: string;
  teamId: string;

  nodes: {
    playerId: string;
    position: Coordinates;
    size: number; // total passes
    color?: string;
  }[];

  edges: {
    from: string; // player ID
    to: string; // player ID
    passes: number;
    completionRate: number; // 0-100
    thickness: number; // visual weight
  }[];

  metrics: {
    networkDensity: number;
    averagePathLength: number;
    keyPlaymakers: string[]; // player IDs by centrality
  };
}

export interface ShotMap {
  matchId: string;
  teamId?: string;

  shots: {
    location: Coordinates;
    xG: number;
    outcome: 'goal' | 'on_target' | 'off_target' | 'blocked';
    playerId: string;
    assistedBy?: string;
    bodyPart: BodyPart;
  }[];

  aggregateMetrics: {
    totalShots: number;
    totalxG: number;
    conversionRate: number; // percentage
    averageDistance: number; // meters
  };
}

export interface HighlightClip {
  clipId: string;
  videoId: string;
  matchId: string;

  startTime: number; // seconds
  endTime: number;
  duration: number; // seconds

  event: MatchEvent;
  importanceScore: number; // 0-1

  cameraAngles: string[]; // video URLs
  includeReplay: boolean;

  tags: string[];
}

// ============================================================================
// Scouting & Recruitment Types
// ============================================================================

export interface ScoutingProfile {
  playerId: string;
  scoutId: string;
  reportDate: string;

  currentAbility: PlayerPerformanceRating;
  potential: PlayerDevelopment;

  strengths: {
    skill: string;
    percentile: number;
    description: string;
  }[];

  weaknesses: {
    skill: string;
    percentile: number;
    improvementPath: string;
  }[];

  tacticalFit: {
    positions: PositionType[];
    formations: string[];
    playstyle: string[];
    versatility: number; // 0-100
  };

  marketAnalysis: {
    currentValue: number; // euros
    transferFeeRange: { min: number; max: number };
    wageExpectation: number; // per year
    competition: string[]; // other interested clubs
  };

  recommendation: {
    priority: 1 | 2 | 3 | 4 | 5;
    confidence: number; // 0-1
    alternatives: string[]; // player IDs
    notes: string;
  };

  videoHighlights?: string[]; // URLs
}

export interface TransferValue {
  playerId: string;
  valuationDate: string;

  estimatedValue: number; // euros

  breakdown: {
    baseValue: number;
    ageFactor: number;
    performanceFactor: number;
    potentialFactor: number;
    contractFactor: number;
    marketFactor: number;
  };

  range: {
    minimum: number;
    maximum: number;
  };

  trend: 'rising' | 'stable' | 'falling';
  trendChange: number; // percentage vs 6 months ago

  comparables: {
    playerId: string;
    transferDate: string;
    fee: number;
    similarity: number; // 0-1
  }[];
}

export interface PlayerSearch {
  filters: {
    sport: SportType;
    positions?: PositionType[];
    ageRange?: { min: number; max: number };
    nationalities?: string[];
    leagues?: string[];
    minPPR?: number;
    maxPPR?: number;
    marketValueRange?: { min: number; max: number };
    contractStatus?: 'expiring' | 'long_term' | 'any';
    availability?: 'transfer_listed' | 'release_clause' | 'any';
  };

  sorting: {
    field: string;
    direction: 'asc' | 'desc';
  };

  pagination: {
    page: number;
    limit: number;
  };
}

export interface SimilarPlayer {
  playerId: string;
  player: Player;
  similarityScore: number; // 0-1

  matchedAttributes: {
    attribute: string;
    referenceValue: number;
    playerValue: number;
    similarity: number; // 0-1
  }[];

  keyDifferences: {
    attribute: string;
    description: string;
  }[];
}

// ============================================================================
// Configuration & Settings Types
// ============================================================================

export interface SportsAnalyticsConfig {
  apiKey: string;
  sport: SportType;
  region?: string;
  league?: string;

  features: {
    realTimeTracking: boolean;
    videoAnalytics: boolean;
    predictions: boolean;
    injuryMonitoring: boolean;
    scouting: boolean;
  };

  dataRetention: {
    trackingData: number; // days
    videoData: number; // days
    aggregatedStats: number; // days
  };

  privacy: {
    anonymizePublicData: boolean;
    gdprCompliant: boolean;
    dataShareConsent: boolean;
  };
}

export interface AnalyticsPreferences {
  userId: string;

  favoriteTeams: string[]; // team IDs
  favoritePlayers: string[]; // player IDs

  defaultMetrics: string[];

  notifications: {
    goalAlerts: boolean;
    matchStart: boolean;
    injuryUpdates: boolean;
    transferNews: boolean;
  };

  visualizationDefaults: {
    heatMapType: 'time' | 'actions' | 'combined';
    colorScheme: 'team_colors' | 'heatmap' | 'monochrome';
    preferredCharts: ('radar' | 'bar' | 'line' | 'scatter')[];
  };
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: Record<string, unknown>;
  };
  metadata?: {
    timestamp: string;
    requestId: string;
    version: string;
  };
}

export interface PaginatedResponse<T> {
  items: T[];
  pagination: {
    page: number;
    limit: number;
    total: number;
    hasMore: boolean;
  };
}

// ============================================================================
// Utility Types
// ============================================================================

export type DateRange = {
  start: string; // ISO 8601
  end: string; // ISO 8601
};

export type PerformanceTrend = {
  metric: string;
  values: { date: string; value: number }[];
  trend: 'improving' | 'stable' | 'declining';
  changeRate: number; // percentage change per period
};

export type Percentile = number; // 0-100

export type ConfidenceInterval = {
  lower: number;
  median: number;
  upper: number;
  confidence: number; // e.g., 0.95 for 95%
};

// ============================================================================
// Export all types
// ============================================================================

export default {
  // Re-export all types for convenient importing
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 *
 * These type definitions enable accessible, standardized sports analytics
 * for teams at all levels, promoting data-driven decision-making,
 * athlete welfare, and fair competition.
 *
 * WIA - World Certification Industry Association
 * © 2025 SmileStory Inc. / WIA
 * MIT License
 */
