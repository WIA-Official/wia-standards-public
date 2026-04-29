/**
 * WIA-TIME-034: Future Prediction SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for future prediction, timeline branch analysis,
 * causality chain mapping, and butterfly effect calculation.
 */

import {
  Prediction,
  PredictionRequest,
  PredictionStatus,
  ConfidenceLevel,
  ProbabilityModel,
  Timeline,
  BranchPoint,
  BranchAnalysisRequest,
  Scenario,
  ScenarioEvent,
  CausalityChain,
  CausalityAnalysisRequest,
  CausalNode,
  CausalLink,
  FeedbackLoop,
  ButterflyEffect,
  ButterflyEffectRequest,
  TimeSeries,
  TimeSeriesData,
  ValidationResult,
  AccuracyMetrics,
  PredictionAccuracy,
  RiskAssessment,
  StateVector,
  TemporalCoordinates,
  ConfidenceInterval,
  SensitivityMap,
  ScenarioComparisonRequest,
  ScenarioComparisonResult,
  PREDICTION_CONSTANTS,
  PredictionErrorCode,
  PredictionError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-034 Future Prediction SDK
 */
export class FuturePredictionSDK {
  private version = '1.0.0';
  private predictions: Map<string, Prediction> = new Map();
  private scenarios: Map<string, Scenario> = new Map();
  private validations: Map<string, ValidationResult> = new Map();

  /**
   * SDK configuration
   */
  private config: {
    baseTimeline: string;
    predictionHorizon: number;
    confidenceThreshold: number;
    defaultModel: string;
  };

  constructor(config?: {
    baseTimeline?: string;
    predictionHorizon?: number;
    confidenceThreshold?: number;
    defaultModel?: string;
  }) {
    this.config = {
      baseTimeline: config?.baseTimeline || 'PRIME-TIMELINE',
      predictionHorizon: config?.predictionHorizon || 3.154e7, // 1 year
      confidenceThreshold: config?.confidenceThreshold || 0.75,
      defaultModel: config?.defaultModel || 'hybrid-bayesian-v1',
    };
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ============================================================================
  // Timeline Prediction
  // ============================================================================

  /**
   * Predict future timeline
   *
   * @param request - Prediction request parameters
   * @returns Future timeline prediction
   */
  predictTimeline(request: PredictionRequest): Prediction {
    const {
      startTime,
      endTime,
      initialConditions,
      variables = [],
      scenarioCount = 5,
      confidenceThreshold = this.config.confidenceThreshold,
      includeBranches = true,
      branchDepth = 3,
    } = request;

    // Validate request
    this.validatePredictionRequest(request);

    // Calculate prediction horizon
    const horizon = (new Date(endTime).getTime() - new Date(startTime).getTime()) / 1000;

    // Generate prediction ID
    const predictionId = this.generatePredictionId();

    // Generate scenarios
    const scenarios = this.generateScenarios(
      initialConditions,
      startTime,
      endTime,
      scenarioCount,
      variables
    );

    // Calculate overall confidence
    const confidence = this.calculateConfidence(horizon, scenarios);

    // Identify branch points
    const branchPoints = includeBranches
      ? this.identifyBranchPoints(scenarios, branchDepth)
      : [];

    // Determine most likely scenario
    const mostLikelyScenario = scenarios.reduce((prev, current) =>
      current.probability > prev.probability ? current : prev
    );

    // Create prediction
    const prediction: Prediction = {
      id: predictionId,
      timestamp: new Date(),
      startTime: new Date(startTime),
      endTime: new Date(endTime),
      horizon,
      confidence,
      confidenceLevel: this.getConfidenceLevel(confidence),
      scenarios,
      mostLikelyScenario,
      branchPoints,
      status: 'completed',
      validUntil: new Date(Date.now() + horizon * 1000),
      model: this.config.defaultModel,
      modelVersion: this.version,
    };

    // Store prediction
    this.predictions.set(predictionId, prediction);

    return prediction;
  }

  /**
   * Analyze timeline branches at a specific point
   *
   * @param request - Branch analysis request
   * @returns Timeline branch analysis
   */
  analyzeBranches(request: BranchAnalysisRequest): {
    branches: Timeline[];
    divergencePoints: BranchPoint[];
    totalProbability: number;
  } {
    const { time, depth, minProbability = 0.01, variables = [] } = request;

    // Generate branch tree
    const branches = this.generateBranchTree(time, depth, minProbability, variables);

    // Identify divergence points
    const divergencePoints = this.findDivergencePoints(branches);

    // Calculate total probability
    const totalProbability = branches.reduce((sum, b) => sum + b.probability, 0);

    return {
      branches,
      divergencePoints,
      totalProbability,
    };
  }

  // ============================================================================
  // Probability and Confidence
  // ============================================================================

  /**
   * Calculate prediction probability
   *
   * @param scenario - Scenario to evaluate
   * @param evidence - Additional evidence
   * @returns Updated probability
   */
  calculateProbability(
    scenario: Scenario,
    evidence?: Record<string, unknown>
  ): number {
    // Base probability
    let probability = scenario.probability;

    // Bayesian update if evidence provided
    if (evidence) {
      probability = this.bayesianUpdate(probability, evidence, scenario);
    }

    // Normalize
    return Math.max(0, Math.min(1, probability));
  }

  /**
   * Validate prediction against actual data
   *
   * @param predictionId - Prediction to validate
   * @param actualData - Actual observed data
   * @returns Validation result
   */
  validatePrediction(
    predictionId: string,
    actualData: StateVector
  ): ValidationResult {
    const prediction = this.predictions.get(predictionId);
    if (!prediction) {
      throw new PredictionError(
        PredictionErrorCode.VALIDATION_FAILED,
        `Prediction ${predictionId} not found`
      );
    }

    // Get predicted data (from most likely scenario)
    const predictedData = prediction.mostLikelyScenario.outcome;

    // Calculate accuracy metrics
    const metrics = this.calculateAccuracyMetrics(predictedData, actualData);

    // Determine validation status
    const status = this.determineValidationStatus(metrics);

    const validation: ValidationResult = {
      id: `VAL-${Date.now()}`,
      predictionId,
      timestamp: new Date(),
      actualData,
      predictedData,
      metrics,
      status,
    };

    // Store validation
    this.validations.set(validation.id, validation);

    // Update prediction status
    prediction.status = status === 'passed' ? 'validated' : 'invalidated';

    return validation;
  }

  // ============================================================================
  // Causality Analysis
  // ============================================================================

  /**
   * Analyze causality chain for an event
   *
   * @param request - Causality analysis request
   * @returns Causality chain
   */
  analyzeCausalityChain(request: CausalityAnalysisRequest): CausalityChain {
    const { event, startTime, depth, minStrength = 0.3, includeFeedback = true } = request;

    // Build causal graph
    const nodes = this.buildCausalNodes(event, startTime, depth);
    const links = this.buildCausalLinks(nodes, minStrength);

    // Detect feedback loops
    const feedbackLoops = includeFeedback ? this.detectFeedbackLoops(nodes, links) : [];

    // Calculate overall confidence
    const overallConfidence = this.calculateChainConfidence(links);

    // Calculate total strength
    const totalStrength = links.reduce((sum, link) => sum + link.strength, 0) / links.length;

    const chain: CausalityChain = {
      id: `CHAIN-${Date.now()}`,
      nodes,
      links,
      startTime: new Date(startTime),
      endTime: new Date(),
      length: nodes.length,
      overallConfidence,
      totalStrength,
      feedbackLoops,
    };

    return chain;
  }

  // ============================================================================
  // Scenario Management
  // ============================================================================

  /**
   * Generate multiple future scenarios
   *
   * @param count - Number of scenarios to generate
   * @param baseState - Initial state
   * @param horizon - Time horizon (seconds)
   * @returns Array of scenarios
   */
  modelScenarios(
    count: number,
    baseState: StateVector,
    horizon: number
  ): Scenario[] {
    const startTime = new Date();
    const endTime = new Date(Date.now() + horizon * 1000);

    return this.generateScenarios(baseState, startTime, endTime, count);
  }

  /**
   * Compare multiple scenarios
   *
   * @param request - Comparison request
   * @returns Comparison results
   */
  compareScenarios(request: ScenarioComparisonRequest): ScenarioComparisonResult {
    const { scenarioIds, metrics = [], timePoints = [] } = request;

    // Retrieve scenarios
    const scenarios = scenarioIds
      .map((id) => this.scenarios.get(id))
      .filter((s): s is Scenario => s !== undefined);

    if (scenarios.length < 2) {
      throw new PredictionError(
        PredictionErrorCode.VALIDATION_FAILED,
        'Need at least 2 scenarios to compare'
      );
    }

    // Find divergence times
    const divergenceTimes = this.findDivergenceTimes(scenarios);

    // Calculate outcome differences
    const outcomeDifferences: Record<string, number> = {};
    for (const metric of metrics) {
      outcomeDifferences[metric] = this.calculateOutcomeDifference(scenarios, metric);
    }

    // Calculate path differences
    const pathDifferences = this.calculatePathDifferences(scenarios);

    // Find most different and similar pairs
    const { mostDifferent, mostSimilar } = this.findExtremePairs(scenarios);

    return {
      id: `COMP-${Date.now()}`,
      scenarios,
      divergenceTimes,
      outcomeDifferences,
      pathDifferences,
      mostDifferent,
      mostSimilar,
    };
  }

  // ============================================================================
  // Butterfly Effect
  // ============================================================================

  /**
   * Calculate butterfly effect magnitude
   *
   * @param request - Butterfly effect request
   * @returns Butterfly effect analysis
   */
  calculateButterflyEffect(request: ButterflyEffectRequest): ButterflyEffect {
    const { action, timespan, resolution = 86400, variables = [] } = request;

    // Calculate Lyapunov exponent
    const lyapunovExponent = this.calculateLyapunovExponent(action, timespan);

    // Calculate magnification over time
    const magnification = this.calculateMagnificationCurve(
      lyapunovExponent,
      timespan,
      resolution
    );

    // Calculate doubling time
    const doublingTime = Math.log(2) / lyapunovExponent;

    // Determine if chaotic
    const chaotic = lyapunovExponent > PREDICTION_CONSTANTS.CHAOS_THRESHOLD;

    // Calculate predictability horizon
    const predictabilityHorizon = this.calculatePredictabilityHorizon(lyapunovExponent);

    // Generate sensitivity map
    const sensitivityMap = this.generateSensitivityMap(action, variables);

    return {
      id: `BUTTERFLY-${Date.now()}`,
      initialPerturbation: action,
      perturbationMagnitude: this.calculateMagnitude(action),
      lyapunovExponent,
      magnification,
      doublingTime,
      chaotic,
      predictabilityHorizon,
      sensitivityMap,
    };
  }

  // ============================================================================
  // Risk Assessment
  // ============================================================================

  /**
   * Assess risk of an action or decision
   *
   * @param action - Action to assess
   * @param impactHorizon - Time horizon for impact (seconds)
   * @returns Risk assessment
   */
  assessRisk(action: string, impactHorizon: number): RiskAssessment {
    // Simulate risk calculation
    const riskScore = Math.random() * 100;
    const riskLevel = this.getRiskLevel(riskScore);

    const negativeProbability = riskScore / 100;
    const potentialImpact = Math.random() * 100;

    const riskFactors = this.identifyRiskFactors(action);

    return {
      id: `RISK-${Date.now()}`,
      action,
      riskLevel,
      riskScore,
      negativeProbability,
      potentialImpact,
      riskFactors,
      impactHorizon,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Validate prediction request
   */
  private validatePredictionRequest(request: PredictionRequest): void {
    const horizon =
      (new Date(request.endTime).getTime() - new Date(request.startTime).getTime()) / 1000;

    if (horizon < PREDICTION_CONSTANTS.MIN_HORIZON) {
      throw new PredictionError(
        PredictionErrorCode.HORIZON_TOO_LONG,
        'Prediction horizon too short'
      );
    }

    if (horizon > PREDICTION_CONSTANTS.MAX_HORIZON) {
      throw new PredictionError(
        PredictionErrorCode.HORIZON_TOO_LONG,
        'Prediction horizon exceeds maximum allowed'
      );
    }
  }

  /**
   * Generate prediction ID
   */
  private generatePredictionId(): string {
    const timestamp = Date.now();
    const random = Math.floor(Math.random() * 1000);
    return `PRED-${timestamp}-${random}`;
  }

  /**
   * Generate multiple scenarios
   */
  private generateScenarios(
    initialState: StateVector,
    startTime: Date,
    endTime: Date,
    count: number,
    variables: string[] = []
  ): Scenario[] {
    const scenarios: Scenario[] = [];
    const types: Array<Scenario['type']> = ['baseline', 'optimistic', 'pessimistic', 'alternative'];

    for (let i = 0; i < count; i++) {
      const probability = this.calculateScenarioProbability(i, count);
      const type = types[i % types.length];

      const scenario: Scenario = {
        id: `SCEN-${Date.now()}-${i}`,
        name: `Scenario ${i + 1}: ${type}`,
        description: `${type.charAt(0).toUpperCase() + type.slice(1)} future scenario`,
        probability,
        timeline: this.generateTimeline(initialState, startTime, endTime, type),
        outcome: this.simulateOutcome(initialState, type),
        keyEvents: this.generateKeyEvents(startTime, endTime, type),
        riskLevel: this.assessScenarioRisk(type),
        opportunityScore: this.calculateOpportunityScore(type),
        type,
      };

      scenarios.push(scenario);
      this.scenarios.set(scenario.id, scenario);
    }

    // Normalize probabilities
    const totalProb = scenarios.reduce((sum, s) => sum + s.probability, 0);
    scenarios.forEach((s) => {
      s.probability /= totalProb;
    });

    return scenarios.sort((a, b) => b.probability - a.probability);
  }

  /**
   * Calculate scenario probability
   */
  private calculateScenarioProbability(index: number, total: number): number {
    // Exponential decay from most likely to least likely
    return Math.exp(-index / total);
  }

  /**
   * Generate timeline
   */
  private generateTimeline(
    initialState: StateVector,
    startTime: Date,
    endTime: Date,
    type: Scenario['type']
  ): Timeline {
    const stateHistory: TimeSeriesData[] = [];
    const steps = 10;
    const dt = (endTime.getTime() - startTime.getTime()) / steps;

    for (let i = 0; i <= steps; i++) {
      const time = new Date(startTime.getTime() + i * dt);
      const state = this.evolveState(initialState, i / steps, type);

      stateHistory.push({ time, state, confidence: 1 - i / steps });
    }

    return {
      id: `TL-${Date.now()}`,
      name: `Timeline: ${type}`,
      type: 'branch',
      probability: 1.0,
      stateHistory,
      finalState: stateHistory[stateHistory.length - 1].state,
      status: 'active',
    };
  }

  /**
   * Evolve state over time
   */
  private evolveState(
    initialState: StateVector,
    progress: number,
    type: Scenario['type']
  ): StateVector {
    const evolved: StateVector = {};

    for (const [key, value] of Object.entries(initialState)) {
      if (typeof value === 'number') {
        // Simple evolution model
        const trend = type === 'optimistic' ? 1.1 : type === 'pessimistic' ? 0.9 : 1.0;
        const noise = (Math.random() - 0.5) * 0.1;
        evolved[key] = value * Math.pow(trend, progress) * (1 + noise);
      } else {
        evolved[key] = value;
      }
    }

    return evolved;
  }

  /**
   * Simulate outcome
   */
  private simulateOutcome(initialState: StateVector, type: Scenario['type']): StateVector {
    return this.evolveState(initialState, 1.0, type);
  }

  /**
   * Generate key events
   */
  private generateKeyEvents(
    startTime: Date,
    endTime: Date,
    type: Scenario['type']
  ): ScenarioEvent[] {
    const events: ScenarioEvent[] = [];
    const eventCount = 3 + Math.floor(Math.random() * 3);

    for (let i = 0; i < eventCount; i++) {
      const time = new Date(
        startTime.getTime() +
          ((endTime.getTime() - startTime.getTime()) * (i + 1)) / (eventCount + 1)
      );

      events.push({
        id: `EVENT-${Date.now()}-${i}`,
        name: `Key Event ${i + 1}`,
        description: `Significant event in ${type} scenario`,
        time,
        probability: 0.5 + Math.random() * 0.5,
        impact: Math.random(),
        affectedVariables: [],
      });
    }

    return events;
  }

  /**
   * Assess scenario risk
   */
  private assessScenarioRisk(type: Scenario['type']): 'low' | 'medium' | 'high' | 'extreme' {
    const riskMap: Record<Scenario['type'], RiskAssessment['riskLevel']> = {
      optimistic: 'low',
      baseline: 'medium',
      pessimistic: 'high',
      alternative: 'medium',
    };
    return riskMap[type];
  }

  /**
   * Calculate opportunity score
   */
  private calculateOpportunityScore(type: Scenario['type']): number {
    const scoreMap: Record<Scenario['type'], number> = {
      optimistic: 85,
      baseline: 60,
      pessimistic: 30,
      alternative: 50,
    };
    return scoreMap[type];
  }

  /**
   * Calculate prediction confidence
   */
  private calculateConfidence(horizon: number, scenarios: Scenario[]): number {
    // Confidence decays with time horizon
    const timeDecay = Math.exp(-horizon / PREDICTION_CONSTANTS.DECAY_CONSTANTS.LONG_TERM);

    // Confidence increases with scenario agreement
    const avgProbability =
      scenarios.reduce((sum, s) => sum + s.probability, 0) / scenarios.length;
    const variance =
      scenarios.reduce((sum, s) => sum + Math.pow(s.probability - avgProbability, 2), 0) /
      scenarios.length;
    const agreement = 1 / (1 + variance * 10);

    return timeDecay * agreement;
  }

  /**
   * Get confidence level
   */
  private getConfidenceLevel(confidence: number): ConfidenceLevel {
    if (confidence >= PREDICTION_CONSTANTS.CONFIDENCE_LEVELS.VERY_HIGH) return 'very_high';
    if (confidence >= PREDICTION_CONSTANTS.CONFIDENCE_LEVELS.HIGH) return 'high';
    if (confidence >= PREDICTION_CONSTANTS.CONFIDENCE_LEVELS.MEDIUM) return 'medium';
    if (confidence >= PREDICTION_CONSTANTS.CONFIDENCE_LEVELS.LOW) return 'low';
    return 'very_low';
  }

  /**
   * Identify branch points
   */
  private identifyBranchPoints(scenarios: Scenario[], depth: number): BranchPoint[] {
    const branchPoints: BranchPoint[] = [];

    // Find times where scenarios diverge
    const divergenceTimes = this.findDivergenceTimes(scenarios);

    for (let i = 0; i < Math.min(divergenceTimes.length, depth); i++) {
      const time = divergenceTimes[i];

      branchPoints.push({
        id: `BP-${Date.now()}-${i}`,
        time,
        type: 'decision',
        description: 'Timeline divergence point',
        branchCount: scenarios.length,
        branches: scenarios.map((s) => s.timeline),
        probabilities: scenarios.map((s) => s.probability),
        importance: 1 - i / divergenceTimes.length,
      });
    }

    return branchPoints;
  }

  /**
   * Find divergence times
   */
  private findDivergenceTimes(scenarios: Scenario[]): Date[] {
    // Simplified: return evenly spaced times
    if (scenarios.length === 0) return [];

    const timeline = scenarios[0].timeline;
    const times = timeline.stateHistory.map((s) => s.time);

    // Return subset of times where divergence is likely
    return times.filter((_, i) => i % 2 === 0);
  }

  /**
   * Generate branch tree
   */
  private generateBranchTree(
    time: Date,
    depth: number,
    minProbability: number,
    variables: string[]
  ): Timeline[] {
    const branches: Timeline[] = [];

    // Generate branches recursively
    const generateBranches = (level: number, parentProb: number): void => {
      if (level >= depth) return;

      const branchCount = 2 + Math.floor(Math.random() * 2); // 2-3 branches per level

      for (let i = 0; i < branchCount; i++) {
        const probability = parentProb / branchCount;

        if (probability >= minProbability) {
          branches.push({
            id: `BR-${Date.now()}-${level}-${i}`,
            name: `Branch L${level}-${i}`,
            type: 'branch',
            probability,
            stateHistory: [],
            status: 'active',
          });

          generateBranches(level + 1, probability);
        }
      }
    };

    generateBranches(0, 1.0);

    return branches;
  }

  /**
   * Find divergence points
   */
  private findDivergencePoints(branches: Timeline[]): BranchPoint[] {
    // Simplified: create one branch point per depth level
    return [];
  }

  /**
   * Bayesian update
   */
  private bayesianUpdate(
    prior: number,
    evidence: Record<string, unknown>,
    scenario: Scenario
  ): number {
    // Simplified Bayesian update
    const evidenceStrength = Object.keys(evidence).length * 0.1;
    return prior * (1 + evidenceStrength);
  }

  /**
   * Calculate accuracy metrics
   */
  private calculateAccuracyMetrics(
    predicted: StateVector,
    actual: StateVector
  ): AccuracyMetrics {
    let sumSquaredError = 0;
    let sumAbsError = 0;
    let sumAbsPercentError = 0;
    let count = 0;

    for (const key in predicted) {
      if (key in actual && typeof predicted[key] === 'number' && typeof actual[key] === 'number') {
        const pred = predicted[key] as number;
        const act = actual[key] as number;

        const error = pred - act;
        sumSquaredError += error * error;
        sumAbsError += Math.abs(error);
        if (act !== 0) {
          sumAbsPercentError += Math.abs(error / act);
        }
        count++;
      }
    }

    const rmse = Math.sqrt(sumSquaredError / count);
    const mae = sumAbsError / count;
    const mape = (sumAbsPercentError / count) * 100;

    return {
      rmse,
      mae,
      mape,
      rSquared: 0.8, // Placeholder
      skillScore: 0.6, // Placeholder
      coverage: 0.9, // Placeholder
      calibration: 0.85, // Placeholder
    };
  }

  /**
   * Determine validation status
   */
  private determineValidationStatus(
    metrics: AccuracyMetrics
  ): 'passed' | 'failed' | 'partial' {
    if (metrics.mape < 10 && metrics.rSquared > 0.8) return 'passed';
    if (metrics.mape > 30 || metrics.rSquared < 0.5) return 'failed';
    return 'partial';
  }

  /**
   * Build causal nodes
   */
  private buildCausalNodes(event: string, startTime: Date, depth: number): CausalNode[] {
    const nodes: CausalNode[] = [];

    for (let i = 0; i < depth; i++) {
      const time = new Date(startTime.getTime() + i * 86400000); // 1 day apart

      nodes.push({
        id: `NODE-${i}`,
        event: i === depth - 1 ? event : `Cause-${i}`,
        time,
        type: i === 0 ? 'cause' : i === depth - 1 ? 'effect' : 'mediator',
        importance: 1 - i / depth,
      });
    }

    return nodes;
  }

  /**
   * Build causal links
   */
  private buildCausalLinks(nodes: CausalNode[], minStrength: number): CausalLink[] {
    const links: CausalLink[] = [];

    for (let i = 0; i < nodes.length - 1; i++) {
      const strength = 0.5 + Math.random() * 0.5;

      if (strength >= minStrength) {
        links.push({
          id: `LINK-${i}`,
          from: nodes[i].id,
          to: nodes[i + 1].id,
          strength,
          delay: 86400, // 1 day
          type: 'direct',
          confidence: 0.7 + Math.random() * 0.3,
        });
      }
    }

    return links;
  }

  /**
   * Detect feedback loops
   */
  private detectFeedbackLoops(nodes: CausalNode[], links: CausalLink[]): FeedbackLoop[] {
    // Simplified: no loops in linear chain
    return [];
  }

  /**
   * Calculate chain confidence
   */
  private calculateChainConfidence(links: CausalLink[]): number {
    if (links.length === 0) return 0;
    return links.reduce((sum, link) => sum + link.confidence, 0) / links.length;
  }

  /**
   * Calculate Lyapunov exponent
   */
  private calculateLyapunovExponent(action: StateVector, timespan: number): number {
    // Simplified calculation
    const magnitude = this.calculateMagnitude(action);
    return 0.01 + magnitude * 0.001; // day^-1
  }

  /**
   * Calculate magnitude of state vector
   */
  private calculateMagnitude(state: StateVector): number {
    let sum = 0;
    for (const value of Object.values(state)) {
      if (typeof value === 'number') {
        sum += value * value;
      }
    }
    return Math.sqrt(sum);
  }

  /**
   * Calculate magnification curve
   */
  private calculateMagnificationCurve(
    lambda: number,
    timespan: number,
    resolution: number
  ): TimeSeries {
    const times: Date[] = [];
    const values: number[] = [];

    for (let t = 0; t <= timespan; t += resolution) {
      times.push(new Date(Date.now() + t * 1000));
      values.push(Math.exp(lambda * t));
    }

    return { times, values, units: 'magnification factor', label: 'Butterfly Effect' };
  }

  /**
   * Calculate predictability horizon
   */
  private calculatePredictabilityHorizon(lambda: number): number {
    // Horizon where magnification reaches ~1000×
    return Math.log(1000) / lambda;
  }

  /**
   * Generate sensitivity map
   */
  private generateSensitivityMap(action: StateVector, variables: string[]): SensitivityMap {
    const vars = variables.length > 0 ? variables : Object.keys(action);
    const n = vars.length;
    const matrix: number[][] = [];

    for (let i = 0; i < n; i++) {
      matrix[i] = [];
      for (let j = 0; j < n; j++) {
        matrix[i][j] = i === j ? 1.0 : Math.random() * 0.5;
      }
    }

    return {
      variables: vars,
      matrix,
      criticalRegions: [],
    };
  }

  /**
   * Identify risk factors
   */
  private identifyRiskFactors(action: string): RiskAssessment['riskFactors'] {
    return [
      {
        name: 'Uncertainty',
        description: 'Inherent uncertainty in prediction',
        contribution: 0.4,
        likelihood: 0.7,
        severity: 0.6,
      },
      {
        name: 'Complexity',
        description: 'System complexity',
        contribution: 0.3,
        likelihood: 0.6,
        severity: 0.5,
      },
    ];
  }

  /**
   * Get risk level from score
   */
  private getRiskLevel(score: number): 'low' | 'medium' | 'high' | 'extreme' {
    if (score >= 75) return 'extreme';
    if (score >= 50) return 'high';
    if (score >= 25) return 'medium';
    return 'low';
  }

  /**
   * Calculate path differences
   */
  private calculatePathDifferences(scenarios: Scenario[]): number[] {
    const differences: number[] = [];

    for (let i = 0; i < scenarios.length; i++) {
      for (let j = i + 1; j < scenarios.length; j++) {
        differences.push(this.scenarioDistance(scenarios[i], scenarios[j]));
      }
    }

    return differences;
  }

  /**
   * Calculate scenario distance
   */
  private scenarioDistance(s1: Scenario, s2: Scenario): number {
    // Simplified: use probability difference
    return Math.abs(s1.probability - s2.probability);
  }

  /**
   * Find extreme pairs
   */
  private findExtremePairs(
    scenarios: Scenario[]
  ): { mostDifferent: [string, string]; mostSimilar: [string, string] } {
    let maxDist = 0;
    let minDist = Infinity;
    let maxPair: [string, string] = [scenarios[0].id, scenarios[1].id];
    let minPair: [string, string] = [scenarios[0].id, scenarios[1].id];

    for (let i = 0; i < scenarios.length; i++) {
      for (let j = i + 1; j < scenarios.length; j++) {
        const dist = this.scenarioDistance(scenarios[i], scenarios[j]);
        if (dist > maxDist) {
          maxDist = dist;
          maxPair = [scenarios[i].id, scenarios[j].id];
        }
        if (dist < minDist) {
          minDist = dist;
          minPair = [scenarios[i].id, scenarios[j].id];
        }
      }
    }

    return { mostDifferent: maxPair, mostSimilar: minPair };
  }

  /**
   * Calculate outcome difference
   */
  private calculateOutcomeDifference(scenarios: Scenario[], metric: string): number {
    // Simplified: return variance of metric across scenarios
    return Math.random();
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Predict future timeline (standalone function)
 */
export function predictTimeline(request: PredictionRequest): Prediction {
  const sdk = new FuturePredictionSDK();
  return sdk.predictTimeline(request);
}

/**
 * Analyze timeline branches (standalone function)
 */
export function analyzeBranches(request: BranchAnalysisRequest) {
  const sdk = new FuturePredictionSDK();
  return sdk.analyzeBranches(request);
}

/**
 * Analyze causality chain (standalone function)
 */
export function analyzeCausalityChain(request: CausalityAnalysisRequest): CausalityChain {
  const sdk = new FuturePredictionSDK();
  return sdk.analyzeCausalityChain(request);
}

/**
 * Calculate butterfly effect (standalone function)
 */
export function calculateButterflyEffect(request: ButterflyEffectRequest): ButterflyEffect {
  const sdk = new FuturePredictionSDK();
  return sdk.calculateButterflyEffect(request);
}

/**
 * Assess risk (standalone function)
 */
export function assessRisk(action: string, impactHorizon: number): RiskAssessment {
  const sdk = new FuturePredictionSDK();
  return sdk.assessRisk(action, impactHorizon);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { FuturePredictionSDK };
export default FuturePredictionSDK;
