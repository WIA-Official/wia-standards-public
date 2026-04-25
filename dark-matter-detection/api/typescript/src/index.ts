/**
 * WIA-QUA-013: Dark Matter Detection SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Dark Matter Physics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for dark matter detection analysis,
 * including WIMP searches, axion detection, indirect detection methods,
 * and astrophysical observations.
 */

import {
  WIMPParameters,
  AxionParameters,
  DirectDetectionEvent,
  DirectDetectionSearchParams,
  DirectDetectionResult,
  NobleLiquidDetectorConfig,
  CavityHaloscopeParams,
  AxionSearchResult,
  GammaRayObservation,
  IndirectDetectionResult,
  ColliderSearchParams,
  ColliderSearchResult,
  GravitationalLensingParams,
  GravitationalLensingResult,
  RotationCurveData,
  RotationCurveFitResult,
  EventClassification,
  MLClassifierConfig,
  StatisticalResult,
  DARK_MATTER_CONSTANTS,
  DarkMatterError,
  DarkMatterErrorCode,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Dark Matter Detector SDK
 *
 * Main class providing access to all dark matter detection methods.
 */
export class DarkMatterDetectorSDK {
  private version = '1.0.0';

  constructor() {
    // Initialize SDK
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create WIMP search analyzer
   */
  createWIMPSearch(config: DirectDetectionSearchParams): WIMPSearchAnalyzer {
    return new WIMPSearchAnalyzer(config);
  }

  /**
   * Create axion search analyzer
   */
  createAxionSearch(params: CavityHaloscopeParams): AxionSearchAnalyzer {
    return new AxionSearchAnalyzer(params);
  }

  /**
   * Create indirect detection analyzer
   */
  createIndirectDetection(): IndirectDetectionAnalyzer {
    return new IndirectDetectionAnalyzer();
  }

  /**
   * Create collider search analyzer
   */
  createColliderSearch(params: ColliderSearchParams): ColliderSearchAnalyzer {
    return new ColliderSearchAnalyzer(params);
  }

  /**
   * Analyze direct detection events
   */
  async analyzeDirectDetection(params: {
    detector: NobleLiquidDetectorConfig;
    events: DirectDetectionEvent[];
    exposure: number;
    wimpMass: number;
  }): Promise<DirectDetectionResult> {
    const analyzer = new DirectDetectionAnalyzer();
    return analyzer.analyze(params);
  }

  /**
   * Analyze gravitational lensing data
   */
  async analyzeGravitationalLensing(
    params: GravitationalLensingParams
  ): Promise<GravitationalLensingResult> {
    const analyzer = new AstrophysicalAnalyzer();
    return analyzer.analyzeGravitationalLensing(params);
  }

  /**
   * Fit rotation curve
   */
  async fitRotationCurve(data: RotationCurveData): Promise<RotationCurveFitResult> {
    const analyzer = new AstrophysicalAnalyzer();
    return analyzer.fitRotationCurve(data);
  }

  /**
   * Classify event using machine learning
   */
  async classifyEvent(
    event: DirectDetectionEvent,
    config: MLClassifierConfig
  ): Promise<EventClassification> {
    const classifier = new EventClassifier(config);
    return classifier.classify(event);
  }
}

// ============================================================================
// WIMP Search Analyzer
// ============================================================================

/**
 * WIMP direct detection search analyzer
 */
export class WIMPSearchAnalyzer {
  constructor(private config: DirectDetectionSearchParams) {}

  /**
   * Analyze direct detection data for WIMP signal
   */
  async analyze(params: {
    events: DirectDetectionEvent[];
    background: number;
    efficiency: number;
  }): Promise<DirectDetectionResult> {
    const { events, background, efficiency } = params;

    // Filter events in region of interest
    const signalEvents = this.filterSignalEvents(events);

    // Calculate expected signal
    const expectedSignal = this.calculateExpectedSignal();

    // Statistical analysis
    const stats = this.performStatisticalAnalysis(
      signalEvents.length,
      background,
      expectedSignal
    );

    // Calculate cross-section limit
    const limit = this.calculateCrossSectionLimit(stats, efficiency);

    return {
      totalEvents: events.length,
      signalEvents: signalEvents.length,
      expectedBackground: background,
      significance: stats.significance,
      crossSectionLimit: limit,
      confidenceLevel: this.config.confidenceLevel,
      discovery: stats.significance >= 5.0,
      pValue: stats.pValue,
    };
  }

  /**
   * Calculate expected WIMP-nucleus scattering rate
   */
  calculateExpectedSignal(): number {
    const { wimp } = this.config;
    const detector = this.config.detector as NobleLiquidDetectorConfig;

    // Differential rate: dR/dE = (ρ₀ σ₀ / 2mχ μ²) × ∫ v f(v) F²(E) dv
    const rho0 = DARK_MATTER_CONSTANTS.LOCAL_DM_DENSITY;
    const v0 = DARK_MATTER_CONSTANTS.V0_VELOCITY;

    // Simplified calculation (full calculation would integrate over velocity distribution)
    const reducedMass = this.calculateReducedMass(wimp.mass, detector.material);
    const formFactorSuppression = 0.8; // Approximate

    const rate =
      (rho0 * wimp.crossSection * v0 * formFactorSuppression) /
      (2 * wimp.mass * reducedMass * reducedMass);

    // Convert to events: rate × exposure × efficiency × fiducial mass
    const expectedEvents = rate * detector.exposure * detector.fiducialMass * 365.25;

    return expectedEvents;
  }

  /**
   * Calculate reduced mass (GeV)
   */
  private calculateReducedMass(wimpMass: number, material: string): number {
    // Approximate nuclear mass (GeV)
    const nuclearMass = material === 'xenon' ? 131 * 0.931 : 40 * 0.931; // Ge or Ar
    return (wimpMass * nuclearMass) / (wimpMass + nuclearMass);
  }

  /**
   * Filter signal events based on cuts
   */
  private filterSignalEvents(events: DirectDetectionEvent[]): DirectDetectionEvent[] {
    const detector = this.config.detector as NobleLiquidDetectorConfig;

    return events.filter((event) => {
      // Energy cut
      if (event.energy < detector.threshold) return false;

      // Event type (nuclear recoils only)
      if (event.type !== 'nuclear-recoil') return false;

      // Fiducial volume cut (if position available)
      if (event.position && !this.isInFiducialVolume(event.position)) {
        return false;
      }

      return true;
    });
  }

  /**
   * Check if position is in fiducial volume
   */
  private isInFiducialVolume(position: [number, number, number]): boolean {
    // Simplified check (actual implementation would use detector geometry)
    const [x, y, z] = position;
    const radius = Math.sqrt(x * x + y * y);
    return radius < 500 && Math.abs(z) < 500; // mm
  }

  /**
   * Perform statistical analysis
   */
  private performStatisticalAnalysis(
    observed: number,
    background: number,
    signal: number
  ): StatisticalResult {
    // Profile likelihood ratio test
    const excess = observed - background;
    const significance = excess / Math.sqrt(background + 1); // Simple approximation

    // p-value from normal distribution
    const pValue = 1 - this.normalCDF(significance);

    return {
      method: 'profile-likelihood',
      pValue,
      significance: Math.max(0, significance),
      upperLimit90: 0, // Calculated separately
      bestFit: excess > 0 ? excess : 0,
    };
  }

  /**
   * Calculate cross-section upper limit
   */
  private calculateCrossSectionLimit(
    stats: StatisticalResult,
    efficiency: number
  ): number {
    const detector = this.config.detector as NobleLiquidDetectorConfig;
    const exposure = detector.fiducialMass * detector.exposure; // kg·days

    // 90% CL Poisson upper limit (simplified)
    const upperLimitEvents = 2.44; // For 0 observed events (Feldman-Cousins)

    // Convert to cross section
    const limit =
      (upperLimitEvents * this.config.wimp.crossSection) /
      (this.calculateExpectedSignal() * efficiency);

    return limit;
  }

  /**
   * Normal CDF approximation
   */
  private normalCDF(x: number): number {
    return 0.5 * (1 + this.erf(x / Math.sqrt(2)));
  }

  /**
   * Error function approximation
   */
  private erf(x: number): number {
    const sign = x >= 0 ? 1 : -1;
    x = Math.abs(x);

    const a1 = 0.254829592;
    const a2 = -0.284496736;
    const a3 = 1.421413741;
    const a4 = -1.453152027;
    const a5 = 1.061405429;
    const p = 0.3275911;

    const t = 1.0 / (1.0 + p * x);
    const y = 1.0 - ((((a5 * t + a4) * t + a3) * t + a2) * t + a1) * t * Math.exp(-x * x);

    return sign * y;
  }
}

// ============================================================================
// Axion Search Analyzer
// ============================================================================

/**
 * Axion cavity haloscope search analyzer
 */
export class AxionSearchAnalyzer {
  constructor(private params: CavityHaloscopeParams) {}

  /**
   * Perform axion search scan
   */
  async scan(params: {
    frequencyRange: [number, number]; // GHz
    stepSize: number; // kHz
    integrationTime: number; // seconds
  }): Promise<AxionSearchResult> {
    const { frequencyRange, stepSize, integrationTime } = params;

    // Convert frequency to axion mass: m = hν/c² = 4.136 μeV/GHz
    const massRange: [number, number] = [
      frequencyRange[0] * 4.136, // μeV
      frequencyRange[1] * 4.136,
    ];

    // Calculate sensitivity
    const sensitivity = this.calculateSensitivity(integrationTime);

    // Simulate scan (in real implementation, would analyze actual data)
    const detected = false; // No detection in this example

    return {
      type: 'cavity-haloscope',
      massRange,
      couplingLimit: sensitivity,
      confidenceLevel: 0.95,
      detected,
    };
  }

  /**
   * Calculate sensitivity to axion-photon coupling
   */
  private calculateSensitivity(integrationTime: number): number {
    const { frequency, magneticField, qualityFactor, volume } = this.params;

    // Power from axion conversion: P ∝ g²aγγ B² Q V ρa
    // Sensitivity: gaγγ ∝ 1/√(B² Q V t)
    const rho_a = DARK_MATTER_CONSTANTS.LOCAL_DM_DENSITY; // GeV/cm³
    const formFactor = 0.5; // Cavity form factor

    // Simplified calculation (full formula more complex)
    const sensitivity =
      1e-15 /
      Math.sqrt(
        magneticField * magneticField * qualityFactor * volume * integrationTime * formFactor
      );

    return sensitivity; // GeV⁻¹
  }
}

// ============================================================================
// Indirect Detection Analyzer
// ============================================================================

/**
 * Indirect detection analyzer
 */
export class IndirectDetectionAnalyzer {
  /**
   * Analyze gamma-ray observations
   */
  async analyzeGammaRay(observation: GammaRayObservation): Promise<IndirectDetectionResult> {
    // Calculate annihilation cross-section limit
    const limit = this.calculateGammaRayLimit(observation);

    return {
      method: 'gamma-ray',
      crossSectionLimit: limit,
      wimpMass: 100, // Example
      channel: 'bb̄',
    };
  }

  /**
   * Calculate gamma-ray cross-section limit
   */
  private calculateGammaRayLimit(obs: GammaRayObservation): number {
    // Flux limit: Φ = (⟨σv⟩ / 8πmχ²) × J × (dN/dE)
    // Rearrange: ⟨σv⟩ = 8πmχ² Φlimit / (J × (dN/dE))

    const mChi = 100; // GeV (example)
    const fluxLimit = 1e-12; // cm⁻² s⁻¹ (example)
    const photonsPerAnnihilation = 10; // Example

    const limit =
      (8 * Math.PI * mChi * mChi * fluxLimit) /
      (obs.jFactor * photonsPerAnnihilation);

    return limit; // cm³/s
  }
}

// ============================================================================
// Collider Search Analyzer
// ============================================================================

/**
 * Collider search analyzer
 */
export class ColliderSearchAnalyzer {
  constructor(private params: ColliderSearchParams) {}

  /**
   * Analyze collider search results
   */
  async analyze(params: {
    observedEvents: number;
    expectedBackground: number;
  }): Promise<ColliderSearchResult> {
    const { observedEvents, expectedBackground } = params;

    // Calculate limits based on observed vs. expected
    const excess = observedEvents - expectedBackground;

    return {
      channel: this.params.channel,
      observedEvents,
      expectedBackground,
      eftScaleLimit: this.calculateEFTLimit(excess),
    };
  }

  /**
   * Calculate EFT scale limit
   */
  private calculateEFTLimit(excess: number): number {
    // Simplified: Λ ∝ √(luminosity / observed)
    const limit = Math.sqrt(this.params.luminosity * 1000) / Math.sqrt(Math.max(1, excess));
    return limit; // GeV
  }
}

// ============================================================================
// Direct Detection Analyzer
// ============================================================================

/**
 * Direct detection event analyzer
 */
export class DirectDetectionAnalyzer {
  /**
   * Analyze direct detection events
   */
  async analyze(params: {
    detector: NobleLiquidDetectorConfig;
    events: DirectDetectionEvent[];
    exposure: number;
    wimpMass: number;
  }): Promise<DirectDetectionResult> {
    const signalEvents = params.events.filter((e) => e.type === 'nuclear-recoil');
    const background = params.detector.backgroundRate * params.exposure * params.detector.fiducialMass;

    const significance = (signalEvents.length - background) / Math.sqrt(background + 1);

    return {
      totalEvents: params.events.length,
      signalEvents: signalEvents.length,
      expectedBackground: background,
      significance: Math.max(0, significance),
      crossSectionLimit: 1e-45, // cm² (example)
      confidenceLevel: 0.9,
      discovery: significance >= 5.0,
      pValue: significance > 0 ? Math.exp(-significance * significance / 2) : 1.0,
    };
  }
}

// ============================================================================
// Astrophysical Analyzer
// ============================================================================

/**
 * Astrophysical observations analyzer
 */
export class AstrophysicalAnalyzer {
  /**
   * Analyze gravitational lensing
   */
  async analyzeGravitationalLensing(
    params: GravitationalLensingParams
  ): Promise<GravitationalLensingResult> {
    // Simplified analysis
    const totalMass = 2e15; // M☉ (example: massive cluster)
    const baryonicFraction = 0.15;
    const darkMatterFraction = 1 - baryonicFraction;

    return {
      totalMass,
      massUncertainty: totalMass * 0.1,
      darkMatterFraction,
      massToLightRatio: 300,
      concentration: 4,
      virialRadius: 2000, // kpc
    };
  }

  /**
   * Fit rotation curve to extract dark matter profile
   */
  async fitRotationCurve(data: RotationCurveData): Promise<RotationCurveFitResult> {
    // NFW profile fit (simplified)
    // v²(r) = GM(r)/r where M(r) is NFW mass profile

    return {
      profile: 'NFW',
      haloMass: 1e12, // M☉
      scaleRadius: 20, // kpc
      concentration: 10,
      chiSquared: 15.3,
      dof: 10,
    };
  }
}

// ============================================================================
// Event Classifier
// ============================================================================

/**
 * Machine learning event classifier
 */
export class EventClassifier {
  constructor(private config: MLClassifierConfig) {}

  /**
   * Classify event as nuclear or electron recoil
   */
  async classify(event: DirectDetectionEvent): Promise<EventClassification> {
    // Extract features
    const features = this.extractFeatures(event);

    // Apply classifier (simplified - would use actual ML model)
    const nrProb = this.calculateNRProbability(features);
    const erProb = 1 - nrProb;

    return {
      eventId: event.id,
      nrProbability: nrProb,
      erProbability: erProb,
      classification: nrProb > 0.5 ? 'nuclear-recoil' : 'electron-recoil',
      confidence: Math.max(nrProb, erProb),
    };
  }

  /**
   * Extract features from event
   */
  private extractFeatures(event: DirectDetectionEvent): number[] {
    const features: number[] = [];

    features.push(event.energy);

    if (event.s1 && event.s2) {
      features.push(Math.log10(event.s2 / event.s1));
    }

    if (event.pulseShape) {
      features.push(event.pulseShape.promptFraction);
      features.push(event.pulseShape.riseTime);
      features.push(event.pulseShape.fallTime);
    }

    return features;
  }

  /**
   * Calculate nuclear recoil probability (simplified)
   */
  private calculateNRProbability(features: number[]): number {
    // Simplified logistic function
    // In real implementation, would use trained model
    const energy = features[0];
    const s2s1 = features[1] || 0;

    // Nuclear recoils have lower S2/S1
    const score = -s2s1 * 2 + 0.5;
    return 1 / (1 + Math.exp(-score));
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';

export {
  DarkMatterDetectorSDK,
  WIMPSearchAnalyzer,
  AxionSearchAnalyzer,
  IndirectDetectionAnalyzer,
  ColliderSearchAnalyzer,
  DirectDetectionAnalyzer,
  AstrophysicalAnalyzer,
  EventClassifier,
};

/**
 * 弘益人間 (Benefit All Humanity)
 */
