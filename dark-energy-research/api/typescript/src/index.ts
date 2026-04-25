/**
 * WIA-QUA-014: Dark Energy Research SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Dark Energy Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for dark energy research including:
 * - Cosmological distance calculations
 * - Equation of state evolution
 * - Parameter fitting to observational data
 * - Model comparison
 * - Future fate predictions
 */

import {
  CosmologicalModel,
  LCDMParameters,
  EquationOfStateParams,
  CosmologicalDistances,
  HubbleMeasurement,
  SupernovaDataset,
  SupernovaObservation,
  BAODistance,
  FittingConfig,
  FittingResult,
  ModelComparison,
  FuturePrediction,
  CosmologicalFate,
  GrowthFactor,
  VacuumEnergy,
  DARK_ENERGY_CONSTANTS,
  DarkEnergyErrorCode,
  DarkEnergyError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-QUA-014 Dark Energy Research SDK
 */
export class DarkEnergyResearchSDK {
  private version = '1.0.0';
  private models: Map<string, CosmologicalModelImpl> = new Map();

  constructor() {
    // Initialize SDK
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // Model Creation
  // ==========================================================================

  /**
   * Create cosmological model
   *
   * @param type - Model type
   * @param params - Model parameters
   * @returns Cosmological model instance
   */
  createModel(
    type: CosmologicalModel,
    params: LCDMParameters | EquationOfStateParams
  ): CosmologicalModelImpl {
    const model = new CosmologicalModelImpl(type, params);
    this.models.set(model.id, model);
    return model;
  }

  /**
   * Get model by ID
   */
  getModel(id: string): CosmologicalModelImpl | undefined {
    return this.models.get(id);
  }

  // ==========================================================================
  // Equation of State
  // ==========================================================================

  /**
   * Compute equation of state evolution
   *
   * @param params - Equation of state parameters
   * @returns w(z) values at specified redshifts
   */
  computeEquationOfState(
    params: EquationOfStateParams & { redshifts: number[] }
  ): Array<{ z: number; w: number }> {
    const { model, w0, wa, redshifts } = params;

    return redshifts.map((z) => ({
      z,
      w: this.wOfZ(z, model, w0, wa),
    }));
  }

  /**
   * Calculate w(z) for different parameterizations
   */
  private wOfZ(
    z: number,
    model: string,
    w0: number,
    wa: number = 0
  ): number {
    switch (model) {
      case 'constant':
        return w0;

      case 'CPL':
        // Chevallier-Polarski-Linder: w(z) = w0 + wa*z/(1+z)
        return w0 + (wa * z) / (1 + z);

      case 'linear':
        // w(a) = w0 + wa*(1-a)
        const a = 1 / (1 + z);
        return w0 + wa * (1 - a);

      default:
        return w0;
    }
  }

  // ==========================================================================
  // Data Loading
  // ==========================================================================

  /**
   * Load supernova dataset
   *
   * @param name - Dataset name ('Pantheon', 'JLA', etc.)
   * @returns Supernova dataset
   */
  async loadSupernovaData(name: string): Promise<SupernovaDataset> {
    // Simulate loading data
    // In real implementation, would fetch from file/API
    const mockData = this.generateMockSupernovaData(name);
    return mockData;
  }

  /**
   * Load BAO measurements
   */
  async loadBAOData(): Promise<BAODistance[]> {
    // Mock BAO data from SDSS, eBOSS
    return [
      {
        z: 0.32,
        DVOverRS: 8.25,
        errors: { DVOverRS: 0.15 },
        survey: 'SDSS-BOSS',
      },
      {
        z: 0.57,
        DVOverRS: 13.77,
        errors: { DVOverRS: 0.13 },
        survey: 'SDSS-BOSS',
      },
      {
        z: 0.698,
        DMOverRS: 17.65,
        errors: { DMOverRS: 0.3 },
        survey: 'eBOSS',
      },
    ];
  }

  /**
   * Load Hubble measurements
   */
  async loadHubbleData(): Promise<HubbleMeasurement[]> {
    return [
      {
        z: 0,
        H: 67.36,
        errorH: 0.54,
        method: 'CMB',
        reference: 'Planck 2018',
      },
      {
        z: 0,
        H: 73.04,
        errorH: 1.04,
        method: 'Cepheid',
        reference: 'SH0ES 2022',
      },
    ];
  }

  // ==========================================================================
  // Fitting and Parameter Estimation
  // ==========================================================================

  /**
   * Fit cosmological model to data
   *
   * @param data - Observational data
   * @param config - Fitting configuration
   * @returns Fitting result with best-fit parameters
   */
  fitCosmology(
    data: SupernovaDataset | BAODistance[],
    config: FittingConfig
  ): FittingResult {
    // Simplified fitting - in real implementation would use proper MCMC/optimization
    const { model, parameters } = config;

    // Simulate best-fit values based on model
    const bestFit = this.simulateBestFit(model, parameters);

    // Calculate chi-squared
    const chiSquared = this.calculateChiSquared(data, bestFit, model);

    // Calculate degrees of freedom
    const nData = Array.isArray(data) ? data.length : data.count;
    const dof = nData - parameters.length;

    return {
      bestFit,
      errors: this.calculateErrors(parameters),
      chiSquared,
      dof,
      reducedChiSquared: chiSquared / dof,
      BIC: chiSquared + parameters.length * Math.log(nData),
      AIC: chiSquared + 2 * parameters.length,
      logLikelihood: -chiSquared / 2,
    };
  }

  /**
   * Compare multiple models
   */
  compareModels(
    data: SupernovaDataset,
    models: CosmologicalModel[]
  ): ModelComparison {
    const results = models.map((model) => {
      const params =
        model === 'LCDM' ? ['H0', 'OmegaM'] : ['H0', 'OmegaM', 'w0'];
      const fit = this.fitCosmology(data, {
        model,
        parameters: params,
        priors: {},
        datasets: ['SNeIa'],
      });

      return {
        model,
        chiSquared: fit.chiSquared,
        BIC: fit.BIC,
      };
    });

    // Find best model (lowest BIC)
    const bicValues = results.map((r) => r.BIC);
    const minBIC = Math.min(...bicValues);
    const deltaBIC = bicValues.map((bic) => bic - minBIC);

    const bestIdx = bicValues.indexOf(minBIC);

    return {
      models: models,
      chiSquared: results.map((r) => r.chiSquared),
      BIC: bicValues,
      deltaBIC,
      bestModel: models[bestIdx],
      evidenceStrength: this.interpretDeltaBIC(
        Math.max(...deltaBIC.filter((_, i) => i !== bestIdx))
      ),
    };
  }

  // ==========================================================================
  // Future Predictions
  // ==========================================================================

  /**
   * Predict cosmological fate
   *
   * @param params - Model parameters with equation of state
   * @returns Future prediction
   */
  predictFate(params: {
    model: CosmologicalModel;
    w: number;
    timespan?: number;
  }): FuturePrediction {
    const { model, w, timespan = 100e9 } = params;

    // Determine scenario based on w
    let scenario: CosmologicalFate;
    let timeToEvent: number | undefined;
    let description: string;

    if (w === -1) {
      scenario = 'de-sitter';
      description =
        'Universe expands exponentially forever (de Sitter space). Distant galaxies redshift beyond cosmic horizon.';
    } else if (w > -1) {
      scenario = 'big-freeze';
      description =
        'Universe expands forever, cooling to absolute zero. Heat death after star formation ends.';
    } else if (w < -1) {
      scenario = 'big-rip';
      const H0 = 70; // km/s/Mpc
      timeToEvent = (2 / (3 * H0 * Math.abs(1 + w))) * 9.78e11; // Convert to years
      description = `Phantom energy tears apart all structures. Big Rip occurs in ${(
        timeToEvent / 1e9
      ).toFixed(1)} billion years.`;
    } else {
      scenario = 'big-freeze';
      description = 'Universe fate uncertain - continued expansion expected.';
    }

    const milestones = this.calculateMilestones(scenario, w, timeToEvent);

    return {
      scenario,
      w0: w,
      timeToEvent,
      galaxyHorizonExit: 150e9,
      starFormationEnd: 1e12,
      description,
      milestones,
    };
  }

  /**
   * Calculate vacuum energy
   */
  calculateVacuumEnergy(cutoffType: 'Planck' | 'SUSY' | 'TeV'): VacuumEnergy {
    const cutoffs = {
      Planck: DARK_ENERGY_CONSTANTS.PLANCK_MASS,
      SUSY: 1000, // 1 TeV
      TeV: 1000,
    };

    const cutoff = cutoffs[cutoffType];

    // Convert GeV to J/m³: (GeV/c²)⁴ → J/m³
    const conversionFactor = 1.783e-9; // (GeV/c²) → kg
    const c = 3e8; // m/s
    const hbar = 1.055e-34; // J·s

    // Approximate ρ ~ (cutoff)⁴ in natural units
    const rho = Math.pow(cutoff * conversionFactor * c * c, 4) / Math.pow(hbar * c, 3);

    const observed = 6e-10; // J/m³ (observed dark energy)
    const discrepancy = rho / observed;

    return {
      rho,
      cutoff,
      cutoffType,
      observed,
      discrepancy,
    };
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Generate mock supernova data
   */
  private generateMockSupernovaData(name: string): SupernovaDataset {
    const count = name === 'Pantheon' ? 1048 : 500;
    const data: SupernovaObservation[] = [];

    for (let i = 0; i < Math.min(count, 100); i++) {
      // Generate limited sample
      const z = 0.01 + Math.random() * 1.5;
      const dL = this.mockLuminosityDistance(z);
      const mB = 5 * Math.log10(dL) + 25; // distance modulus

      data.push({
        id: `SN${i.toString().padStart(4, '0')}`,
        z,
        errorZ: z * 0.001,
        mB,
        errorMB: 0.15,
        survey: name,
      });
    }

    return {
      name,
      count: data.length,
      data,
      redshiftRange: {
        min: 0.01,
        max: Math.max(...data.map((d) => d.z)),
      },
    };
  }

  /**
   * Mock luminosity distance calculation
   */
  private mockLuminosityDistance(z: number): number {
    // ΛCDM approximation
    const H0 = 70;
    const OmegaM = 0.3;
    const OmegaL = 0.7;

    const c = DARK_ENERGY_CONSTANTS.SPEED_OF_LIGHT;
    const DH = c / H0;

    // Numerical integration approximation
    const nSteps = 100;
    let integral = 0;
    for (let i = 0; i < nSteps; i++) {
      const zi = (z * i) / nSteps;
      const E = Math.sqrt(OmegaM * Math.pow(1 + zi, 3) + OmegaL);
      integral += (z / nSteps) / E;
    }

    return DH * (1 + z) * integral;
  }

  /**
   * Simulate best-fit parameters
   */
  private simulateBestFit(
    model: CosmologicalModel,
    parameters: string[]
  ): { [key: string]: number } {
    const defaults: { [key: string]: number } = {
      H0: 70.0,
      OmegaM: 0.3,
      OmegaLambda: 0.7,
      w0: -1.0,
      wa: 0.0,
    };

    const bestFit: { [key: string]: number } = {};
    parameters.forEach((param) => {
      bestFit[param] =
        defaults[param] !== undefined
          ? defaults[param] + (Math.random() - 0.5) * 0.1
          : 1.0;
    });

    return bestFit;
  }

  /**
   * Calculate chi-squared
   */
  private calculateChiSquared(
    data: any,
    params: { [key: string]: number },
    model: CosmologicalModel
  ): number {
    // Simplified - would calculate actual residuals
    const nData = Array.isArray(data) ? data.length : data.count;
    return nData * 0.95 + Math.random() * nData * 0.1;
  }

  /**
   * Calculate parameter errors
   */
  private calculateErrors(
    parameters: string[]
  ): { [key: string]: { lower: number; upper: number } } {
    const errors: { [key: string]: { lower: number; upper: number } } = {};

    const errorMap: { [key: string]: number } = {
      H0: 0.5,
      OmegaM: 0.01,
      OmegaLambda: 0.01,
      w0: 0.03,
      wa: 0.1,
    };

    parameters.forEach((param) => {
      const err = errorMap[param] || 0.1;
      errors[param] = {
        lower: err,
        upper: err,
      };
    });

    return errors;
  }

  /**
   * Interpret delta BIC
   */
  private interpretDeltaBIC(
    deltaBIC: number
  ): 'weak' | 'moderate' | 'strong' | 'decisive' {
    if (deltaBIC < 2) return 'weak';
    if (deltaBIC < 6) return 'moderate';
    if (deltaBIC < 10) return 'strong';
    return 'decisive';
  }

  /**
   * Calculate future milestones
   */
  private calculateMilestones(
    scenario: CosmologicalFate,
    w: number,
    ripTime?: number
  ): Array<{ time: number; event: string }> {
    if (scenario === 'big-rip' && ripTime) {
      return [
        { time: ripTime - 1e9, event: 'Galaxy clusters unbound' },
        { time: ripTime - 60e6, event: 'Galaxies torn apart' },
        { time: ripTime - 0.00008, event: 'Solar systems disrupted' },
        { time: ripTime - 0.000000057, event: 'Stars explode' },
        { time: ripTime, event: 'Atoms ripped apart - Big Rip' },
      ];
    }

    if (scenario === 'de-sitter' || scenario === 'big-freeze') {
      return [
        { time: 150e9, event: 'Distant galaxies exit cosmic horizon' },
        { time: 1e12, event: 'Star formation ends (no gas)' },
        { time: 1e14, event: 'Lowest mass stars burn out' },
        { time: 1e40, event: 'Black holes evaporate via Hawking radiation' },
      ];
    }

    return [];
  }
}

// ============================================================================
// Cosmological Model Implementation
// ============================================================================

/**
 * Cosmological model implementation
 */
export class CosmologicalModelImpl {
  public id: string;
  private type: CosmologicalModel;
  private params: LCDMParameters | EquationOfStateParams;

  constructor(type: CosmologicalModel, params: any) {
    this.id = `model-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    this.type = type;
    this.params = params;
  }

  /**
   * Calculate Hubble parameter at redshift z
   */
  hubbleParameter(z: number): number {
    const params = this.params as LCDMParameters;
    const { H0, OmegaM, OmegaLambda } = params;
    const OmegaK = params.OmegaK || 0;
    const OmegaR = params.OmegaR || 0;

    const E2 =
      OmegaR * Math.pow(1 + z, 4) +
      OmegaM * Math.pow(1 + z, 3) +
      OmegaK * Math.pow(1 + z, 2) +
      OmegaLambda;

    return H0 * Math.sqrt(E2);
  }

  /**
   * Calculate luminosity distance
   */
  luminosityDistance(z: number): number {
    const c = DARK_ENERGY_CONSTANTS.SPEED_OF_LIGHT;
    const params = this.params as LCDMParameters;
    const H0 = params.H0;

    // Numerical integration of comoving distance
    const dc = this.comovingDistance(z);

    return (1 + z) * dc;
  }

  /**
   * Calculate angular diameter distance
   */
  angularDiameterDistance(z: number): number {
    const dL = this.luminosityDistance(z);
    return dL / Math.pow(1 + z, 2);
  }

  /**
   * Calculate comoving distance
   */
  comovingDistance(z: number): number {
    const c = DARK_ENERGY_CONSTANTS.SPEED_OF_LIGHT;
    const params = this.params as LCDMParameters;
    const H0 = params.H0;

    // Numerical integration: ∫ dz/H(z)
    const nSteps = 1000;
    let integral = 0;

    for (let i = 0; i < nSteps; i++) {
      const zi = (z * i) / nSteps;
      const H = this.hubbleParameter(zi);
      integral += (z / nSteps) / H;
    }

    return c * integral;
  }

  /**
   * Calculate all distances
   */
  getDistances(z: number): CosmologicalDistances {
    const dC = this.comovingDistance(z);
    const dL = this.luminosityDistance(z);
    const dA = this.angularDiameterDistance(z);
    const distMod = 5 * Math.log10(dL) + 25;

    // Calculate lookback time and age
    const lookbackTime = this.lookbackTime(z);
    const age = DARK_ENERGY_CONSTANTS.AGE_UNIVERSE - lookbackTime;

    return {
      z,
      comovingDistance: dC,
      luminosityDistance: dL,
      angularDiameterDistance: dA,
      distanceModulus: distMod,
      lookbackTime,
      age,
    };
  }

  /**
   * Calculate lookback time to redshift z
   */
  private lookbackTime(z: number): number {
    const H0 = (this.params as LCDMParameters).H0;
    const hubbleTime = 9.78 / (H0 / 100); // Gyr

    // Numerical integration: ∫ dz/(1+z)/H(z)
    const nSteps = 1000;
    let integral = 0;

    for (let i = 0; i < nSteps; i++) {
      const zi = (z * i) / nSteps;
      const H = this.hubbleParameter(zi);
      integral += (z / nSteps) / ((1 + zi) * H);
    }

    return hubbleTime * integral * 100; // Convert to Gyr
  }

  /**
   * Calculate growth factor
   */
  growthFactor(z: number): GrowthFactor {
    const params = this.params as LCDMParameters;
    const OmegaM = params.OmegaM;

    // Approximate growth factor for ΛCDM
    const a = 1 / (1 + z);
    const OmegaMz = this.omegaMofZ(z);

    // Growth factor normalized to D(z=0) = 1
    const D = Math.exp(-integral(0, z, (zp) => this.growthIntegrand(zp)));

    // Growth rate f = Ω_m^γ (approximation)
    const gamma = 0.55;
    const f = Math.pow(OmegaMz, gamma);

    return {
      z,
      D,
      f,
    };
  }

  /**
   * Omega_m(z)
   */
  private omegaMofZ(z: number): number {
    const params = this.params as LCDMParameters;
    const { OmegaM, OmegaLambda } = params;

    const E2 = OmegaM * Math.pow(1 + z, 3) + OmegaLambda;
    return (OmegaM * Math.pow(1 + z, 3)) / E2;
  }

  /**
   * Growth integrand
   */
  private growthIntegrand(z: number): number {
    const OmegaMz = this.omegaMofZ(z);
    return (1 + z) * Math.pow(OmegaMz, -1 / 0.55);
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate Hubble parameter for ΛCDM
 */
export function hubbleParameterLCDM(
  z: number,
  H0: number,
  OmegaM: number,
  OmegaLambda: number
): number {
  const E2 = OmegaM * Math.pow(1 + z, 3) + OmegaLambda;
  return H0 * Math.sqrt(E2);
}

/**
 * Calculate equation of state (CPL)
 */
export function equationOfStateCPL(z: number, w0: number, wa: number): number {
  return w0 + (wa * z) / (1 + z);
}

/**
 * Simple numerical integration
 */
function integral(
  a: number,
  b: number,
  f: (x: number) => number,
  n: number = 1000
): number {
  let sum = 0;
  const dx = (b - a) / n;

  for (let i = 0; i < n; i++) {
    const x = a + i * dx;
    sum += f(x) * dx;
  }

  return sum;
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { DarkEnergyResearchSDK, CosmologicalModelImpl };
export default DarkEnergyResearchSDK;

/**
 * 弘益人間 (Benefit All Humanity)
 */
