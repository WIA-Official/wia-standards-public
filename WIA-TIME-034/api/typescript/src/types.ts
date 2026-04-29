/**
 * WIA-TIME-034: Future Prediction - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geometry and Time Types
// ============================================================================

/**
 * Temporal coordinates
 */
export interface TemporalCoordinates {
  /** Time point */
  time: Date | string;

  /** Temporal uncertainty (±seconds) */
  uncertainty?: number;

  /** Reference timeline */
  timeline?: string;
}

/**
 * State vector for system state
 */
export interface StateVector {
  /** Named state variables */
  [key: string]: number | number[] | StateVector;
}

// ============================================================================
// Prediction Types
// ============================================================================

/**
 * Prediction status
 */
export type PredictionStatus =
  | 'pending'
  | 'computing'
  | 'completed'
  | 'validated'
  | 'invalidated'
  | 'expired';

/**
 * Prediction confidence level
 */
export type ConfidenceLevel =
  | 'very_low'    // < 30%
  | 'low'         // 30-50%
  | 'medium'      // 50-70%
  | 'high'        // 70-90%
  | 'very_high';  // > 90%

/**
 * Main prediction result
 */
export interface Prediction {
  /** Unique prediction identifier */
  id: string;

  /** When prediction was made */
  timestamp: Date;

  /** Start time of prediction window */
  startTime: Date;

  /** End time of prediction window */
  endTime: Date;

  /** Prediction horizon (seconds) */
  horizon: number;

  /** Overall confidence (0-1) */
  confidence: number;

  /** Confidence level category */
  confidenceLevel: ConfidenceLevel;

  /** Possible future scenarios */
  scenarios: Scenario[];

  /** Most likely scenario */
  mostLikelyScenario: Scenario;

  /** Timeline branch points */
  branchPoints: BranchPoint[];

  /** Prediction status */
  status: PredictionStatus;

  /** Valid until this time */
  validUntil: Date;

  /** Model used for prediction */
  model: string;

  /** Model version */
  modelVersion: string;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Probability model configuration
 */
export interface ProbabilityModel {
  /** Model name */
  name: string;

  /** Model type */
  type: 'bayesian' | 'monte_carlo' | 'markov_chain' | 'ensemble';

  /** Model parameters */
  parameters: {
    /** Sample count for MC methods */
    samples?: number;

    /** Prior distribution parameters */
    priors?: Record<string, unknown>;

    /** Convergence threshold */
    convergence?: number;

    /** Maximum iterations */
    maxIterations?: number;
  };

  /** Historical data used */
  historicalData?: {
    /** Start of historical period */
    startDate: Date;

    /** End of historical period */
    endDate: Date;

    /** Number of data points */
    dataPoints: number;
  };
}

// ============================================================================
// Timeline and Branch Types
// ============================================================================

/**
 * Timeline representation
 */
export interface Timeline {
  /** Unique timeline identifier */
  id: string;

  /** Timeline name */
  name: string;

  /** Timeline type */
  type: 'primary' | 'branch' | 'alternate' | 'hypothetical';

  /** Parent timeline (if branched) */
  parentTimeline?: string;

  /** Branch point where this timeline diverged */
  branchPoint?: BranchPoint;

  /** Timeline probability (0-1) */
  probability: number;

  /** State evolution over time */
  stateHistory: TimeSeriesData[];

  /** Final state */
  finalState?: StateVector;

  /** Timeline status */
  status: 'active' | 'converged' | 'terminated' | 'hypothetical';
}

/**
 * Timeline branch point
 */
export interface BranchPoint {
  /** Unique branch point identifier */
  id: string;

  /** Time of branch */
  time: Date;

  /** Type of branch point */
  type: 'decision' | 'quantum' | 'chaos' | 'external';

  /** Description of branch event */
  description: string;

  /** Number of resulting branches */
  branchCount: number;

  /** Resulting timelines */
  branches: Timeline[];

  /** Probability distribution over branches */
  probabilities: number[];

  /** Decision variables (if applicable) */
  decisionVariables?: string[];

  /** Importance score (0-1) */
  importance: number;

  /** Convergence probability (if branches may reconverge) */
  convergenceProbability?: number;
}

/**
 * Timeline convergence point
 */
export interface ConvergencePoint {
  /** Unique convergence point identifier */
  id: string;

  /** Time of convergence */
  time: Date;

  /** Timelines that converge */
  convergingTimelines: string[];

  /** Resulting merged timeline */
  mergedTimeline: Timeline;

  /** Convergence probability */
  probability: number;

  /** Convergence tolerance (how close states must be) */
  tolerance: number;
}

// ============================================================================
// Causality Types
// ============================================================================

/**
 * Causal node in causality chain
 */
export interface CausalNode {
  /** Node identifier */
  id: string;

  /** Event or state */
  event: string;

  /** Time of event */
  time: Date;

  /** Event type */
  type: 'cause' | 'effect' | 'mediator';

  /** State variables at this node */
  state?: StateVector;

  /** Node importance (0-1) */
  importance: number;
}

/**
 * Causal link between nodes
 */
export interface CausalLink {
  /** Link identifier */
  id: string;

  /** Source node */
  from: string;

  /** Target node */
  to: string;

  /** Causal strength (0-1) */
  strength: number;

  /** Time delay */
  delay: number;

  /** Link type */
  type: 'direct' | 'indirect' | 'feedback';

  /** Confidence in this causal relationship */
  confidence: number;

  /** Granger causality p-value (if applicable) */
  grangerPValue?: number;
}

/**
 * Complete causality chain
 */
export interface CausalityChain {
  /** Chain identifier */
  id: string;

  /** Nodes in the chain */
  nodes: CausalNode[];

  /** Links between nodes */
  links: CausalLink[];

  /** Chain start time */
  startTime: Date;

  /** Chain end time */
  endTime: Date;

  /** Chain length (number of nodes) */
  length: number;

  /** Overall chain confidence */
  overallConfidence: number;

  /** Total causal strength */
  totalStrength: number;

  /** Feedback loops detected */
  feedbackLoops: FeedbackLoop[];
}

/**
 * Feedback loop in causal chain
 */
export interface FeedbackLoop {
  /** Loop identifier */
  id: string;

  /** Nodes in the loop */
  nodes: string[];

  /** Loop strength */
  strength: number;

  /** Loop type */
  type: 'positive' | 'negative';

  /** Stability */
  stable: boolean;

  /** Loop period (if periodic) */
  period?: number;
}

// ============================================================================
// Scenario Types
// ============================================================================

/**
 * Future scenario
 */
export interface Scenario {
  /** Unique scenario identifier */
  id: string;

  /** Scenario name */
  name: string;

  /** Scenario description */
  description: string;

  /** Probability of this scenario (0-1) */
  probability: number;

  /** Timeline for this scenario */
  timeline: Timeline;

  /** Final outcome state */
  outcome: StateVector;

  /** Key events in this scenario */
  keyEvents: ScenarioEvent[];

  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high' | 'extreme';

  /** Opportunity score (0-100) */
  opportunityScore: number;

  /** Scenario type */
  type: 'optimistic' | 'pessimistic' | 'baseline' | 'alternative';
}

/**
 * Event within a scenario
 */
export interface ScenarioEvent {
  /** Event identifier */
  id: string;

  /** Event name */
  name: string;

  /** Event description */
  description: string;

  /** Event time */
  time: Date;

  /** Event probability */
  probability: number;

  /** Event impact magnitude */
  impact: number;

  /** Affected variables */
  affectedVariables: string[];
}

// ============================================================================
// Confidence Interval Types
// ============================================================================

/**
 * Confidence interval for prediction
 */
export interface ConfidenceInterval {
  /** Central estimate */
  mean: number;

  /** Lower bound */
  lower: number;

  /** Upper bound */
  upper: number;

  /** Confidence level (e.g., 0.95 for 95%) */
  level: number;

  /** Standard deviation */
  stdDev: number;

  /** Interval type */
  type: 'parametric' | 'bootstrap' | 'bayesian_credible';
}

/**
 * Time-varying confidence
 */
export interface TimeVaryingConfidence {
  /** Time points */
  times: Date[];

  /** Confidence values at each time */
  confidence: number[];

  /** Confidence decay model */
  decayModel: {
    /** Model type */
    type: 'exponential' | 'power_law' | 'linear';

    /** Decay parameters */
    parameters: {
      /** Initial confidence */
      initial: number;

      /** Decay rate/exponent */
      decay: number;

      /** Time constant */
      timeConstant?: number;
    };
  };
}

// ============================================================================
// Butterfly Effect Types
// ============================================================================

/**
 * Butterfly effect analysis result
 */
export interface ButterflyEffect {
  /** Analysis identifier */
  id: string;

  /** Initial perturbation */
  initialPerturbation: StateVector;

  /** Perturbation magnitude */
  perturbationMagnitude: number;

  /** Lyapunov exponent */
  lyapunovExponent: number;

  /** Time evolution of magnification */
  magnification: TimeSeries;

  /** Doubling time (seconds) */
  doublingTime: number;

  /** Chaotic regime indicator */
  chaotic: boolean;

  /** Predictability horizon (seconds) */
  predictabilityHorizon: number;

  /** Sensitivity map */
  sensitivityMap?: SensitivityMap;
}

/**
 * Time series data
 */
export interface TimeSeries {
  /** Time points */
  times: Date[];

  /** Values at each time point */
  values: number[];

  /** Units */
  units?: string;

  /** Label */
  label?: string;
}

/**
 * Time series with state vectors
 */
export interface TimeSeriesData {
  /** Time point */
  time: Date;

  /** State vector at this time */
  state: StateVector;

  /** Confidence at this time */
  confidence?: number;
}

/**
 * Sensitivity map
 */
export interface SensitivityMap {
  /** Variables measured */
  variables: string[];

  /** Sensitivity matrix (variables × variables) */
  matrix: number[][];

  /** High sensitivity regions */
  criticalRegions: CriticalRegion[];
}

/**
 * Critical region in state space
 */
export interface CriticalRegion {
  /** Region identifier */
  id: string;

  /** Variable ranges defining region */
  bounds: Record<string, [number, number]>;

  /** Average sensitivity in region */
  sensitivity: number;

  /** Importance score */
  importance: number;
}

// ============================================================================
// Validation Types
// ============================================================================

/**
 * Prediction validation result
 */
export interface ValidationResult {
  /** Validation identifier */
  id: string;

  /** Prediction being validated */
  predictionId: string;

  /** Validation timestamp */
  timestamp: Date;

  /** Actual observed data */
  actualData: StateVector;

  /** Predicted data */
  predictedData: StateVector;

  /** Accuracy metrics */
  metrics: AccuracyMetrics;

  /** Validation status */
  status: 'passed' | 'failed' | 'partial';

  /** Validation notes */
  notes?: string;
}

/**
 * Prediction accuracy metrics
 */
export interface AccuracyMetrics {
  /** Root Mean Square Error */
  rmse: number;

  /** Mean Absolute Error */
  mae: number;

  /** Mean Absolute Percentage Error */
  mape: number;

  /** R-squared (coefficient of determination) */
  rSquared: number;

  /** Skill score vs baseline */
  skillScore: number;

  /** Coverage (% of actuals within confidence interval) */
  coverage: number;

  /** Calibration score */
  calibration: number;
}

/**
 * Prediction accuracy over time
 */
export interface PredictionAccuracy {
  /** Time horizon evaluated */
  horizon: number;

  /** Number of predictions */
  count: number;

  /** Accuracy by horizon */
  byHorizon: {
    /** Prediction horizon (seconds) */
    horizon: number;

    /** Accuracy percentage (0-100) */
    accuracy: number;

    /** Sample size */
    samples: number;
  }[];

  /** Overall accuracy */
  overall: number;

  /** Confidence calibration curve */
  calibrationCurve?: CalibrationCurve;
}

/**
 * Calibration curve
 */
export interface CalibrationCurve {
  /** Predicted probabilities */
  predicted: number[];

  /** Observed frequencies */
  observed: number[];

  /** Perfect calibration line (y=x) */
  perfect: number[];

  /** Brier score */
  brierScore: number;
}

// ============================================================================
// Future Event Types
// ============================================================================

/**
 * Future event
 */
export interface FutureEvent {
  /** Event identifier */
  id: string;

  /** Event name */
  name: string;

  /** Event description */
  description: string;

  /** Event category */
  category: string;

  /** Predicted time */
  predictedTime: Date;

  /** Time confidence interval */
  timeInterval: ConfidenceInterval;

  /** Event probability */
  probability: number;

  /** Event impact */
  impact: {
    /** Impact magnitude (0-1) */
    magnitude: number;

    /** Affected domains */
    domains: string[];

    /** Positive or negative */
    valence: 'positive' | 'negative' | 'neutral';
  };

  /** Precursor events */
  precursors?: string[];

  /** Consequence events */
  consequences?: string[];
}

// ============================================================================
// Risk Assessment Types
// ============================================================================

/**
 * Risk assessment
 */
export interface RiskAssessment {
  /** Assessment identifier */
  id: string;

  /** Action/decision being assessed */
  action: string;

  /** Risk level */
  riskLevel: 'low' | 'medium' | 'high' | 'extreme';

  /** Risk score (0-100) */
  riskScore: number;

  /** Probability of negative outcome */
  negativeProbability: number;

  /** Potential impact if negative */
  potentialImpact: number;

  /** Risk factors */
  riskFactors: RiskFactor[];

  /** Mitigation strategies */
  mitigations?: MitigationStrategy[];

  /** Timeline impact horizon */
  impactHorizon: number;
}

/**
 * Risk factor
 */
export interface RiskFactor {
  /** Factor name */
  name: string;

  /** Factor description */
  description: string;

  /** Contribution to overall risk (0-1) */
  contribution: number;

  /** Likelihood (0-1) */
  likelihood: number;

  /** Severity (0-1) */
  severity: number;
}

/**
 * Mitigation strategy
 */
export interface MitigationStrategy {
  /** Strategy name */
  name: string;

  /** Strategy description */
  description: string;

  /** Effectiveness (0-1) */
  effectiveness: number;

  /** Cost/difficulty (0-1) */
  cost: number;

  /** Risk reduction */
  riskReduction: number;
}

// ============================================================================
// Request/Response Types
// ============================================================================

/**
 * Prediction request
 */
export interface PredictionRequest {
  /** Start time of prediction window */
  startTime: Date;

  /** End time of prediction window */
  endTime: Date;

  /** Initial conditions */
  initialConditions: StateVector;

  /** Variables to predict */
  variables?: string[];

  /** Number of scenarios to generate */
  scenarioCount?: number;

  /** Minimum confidence threshold */
  confidenceThreshold?: number;

  /** Probability model to use */
  model?: ProbabilityModel;

  /** Include branch analysis */
  includeBranches?: boolean;

  /** Maximum branch depth */
  branchDepth?: number;
}

/**
 * Branch analysis request
 */
export interface BranchAnalysisRequest {
  /** Time to analyze */
  time: Date;

  /** Analysis depth (number of levels) */
  depth: number;

  /** Minimum branch probability to include */
  minProbability?: number;

  /** State variables to track */
  variables?: string[];
}

/**
 * Causality analysis request
 */
export interface CausalityAnalysisRequest {
  /** Event to analyze */
  event: string;

  /** Start time for analysis */
  startTime: Date;

  /** Maximum chain depth */
  depth: number;

  /** Minimum causal strength threshold */
  minStrength?: number;

  /** Include feedback loops */
  includeFeedback?: boolean;
}

/**
 * Butterfly effect request
 */
export interface ButterflyEffectRequest {
  /** Action/perturbation to analyze */
  action: StateVector;

  /** Time span to analyze (seconds) */
  timespan: number;

  /** Temporal resolution (seconds) */
  resolution?: number;

  /** Variables to track */
  variables?: string[];
}

/**
 * Scenario comparison request
 */
export interface ScenarioComparisonRequest {
  /** Scenarios to compare */
  scenarioIds: string[];

  /** Metrics to compare */
  metrics?: string[];

  /** Comparison time points */
  timePoints?: Date[];
}

/**
 * Scenario comparison result
 */
export interface ScenarioComparisonResult {
  /** Comparison identifier */
  id: string;

  /** Scenarios compared */
  scenarios: Scenario[];

  /** Divergence times */
  divergenceTimes: Date[];

  /** Outcome differences */
  outcomeDifferences: Record<string, number>;

  /** Path differences */
  pathDifferences: number[];

  /** Most different scenarios */
  mostDifferent: [string, string];

  /** Most similar scenarios */
  mostSimilar: [string, string];
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical and system constants
 */
export const PREDICTION_CONSTANTS = {
  /** Speed of light (m/s) */
  SPEED_OF_LIGHT: 299792458,

  /** Minimum prediction horizon (seconds) */
  MIN_HORIZON: 1,

  /** Maximum prediction horizon (seconds) */
  MAX_HORIZON: 3.154e9, // 100 years

  /** Default confidence threshold */
  DEFAULT_CONFIDENCE: 0.75,

  /** Maximum scenario count */
  MAX_SCENARIOS: 100,

  /** Maximum branch depth */
  MAX_BRANCH_DEPTH: 10,

  /** Maximum causality depth */
  MAX_CAUSALITY_DEPTH: 20,

  /** Minimum probability for branch inclusion */
  MIN_BRANCH_PROBABILITY: 0.01,

  /** Chaos threshold (Lyapunov exponent) */
  CHAOS_THRESHOLD: 0.01,

  /** Typical decay time constants (seconds) */
  DECAY_CONSTANTS: {
    SHORT_TERM: 86400, // 1 day
    MEDIUM_TERM: 2.628e6, // 1 month
    LONG_TERM: 3.154e7, // 1 year
  },

  /** Confidence levels */
  CONFIDENCE_LEVELS: {
    VERY_LOW: 0.3,
    LOW: 0.5,
    MEDIUM: 0.7,
    HIGH: 0.9,
    VERY_HIGH: 0.95,
  },
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-TIME-034 error codes
 */
export enum PredictionErrorCode {
  INSUFFICIENT_DATA = 'P001',
  HORIZON_TOO_LONG = 'P002',
  CONFIDENCE_TOO_LOW = 'P003',
  CHAOTIC_REGION = 'P004',
  BRANCH_LIMIT_EXCEEDED = 'P005',
  CAUSALITY_LOOP = 'P006',
  ETHICAL_VIOLATION = 'P007',
  MODEL_FAILURE = 'P008',
  VALIDATION_FAILED = 'P009',
  TIMELINE_CONFLICT = 'P010',
}

/**
 * Prediction error class
 */
export class PredictionError extends Error {
  constructor(
    public code: PredictionErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'PredictionError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  TemporalCoordinates,
  StateVector,

  // Prediction
  PredictionStatus,
  ConfidenceLevel,
  Prediction,
  ProbabilityModel,

  // Timeline
  Timeline,
  BranchPoint,
  ConvergencePoint,

  // Causality
  CausalNode,
  CausalLink,
  CausalityChain,
  FeedbackLoop,

  // Scenarios
  Scenario,
  ScenarioEvent,

  // Confidence
  ConfidenceInterval,
  TimeVaryingConfidence,

  // Butterfly Effect
  ButterflyEffect,
  TimeSeries,
  TimeSeriesData,
  SensitivityMap,
  CriticalRegion,

  // Validation
  ValidationResult,
  AccuracyMetrics,
  PredictionAccuracy,
  CalibrationCurve,

  // Events and Risk
  FutureEvent,
  RiskAssessment,
  RiskFactor,
  MitigationStrategy,

  // Requests
  PredictionRequest,
  BranchAnalysisRequest,
  CausalityAnalysisRequest,
  ButterflyEffectRequest,
  ScenarioComparisonRequest,
  ScenarioComparisonResult,
};

export { PREDICTION_CONSTANTS, PredictionErrorCode, PredictionError };
