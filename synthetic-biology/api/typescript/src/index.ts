/**
 * WIA-BIO-012: Synthetic Biology SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for synthetic biology including:
 * - Genetic circuit design and simulation
 * - Promoter strength calculations
 * - Metabolic pathway optimization
 * - DNA assembly planning
 * - Biosafety assessment
 */

import {
  BioBrickPart,
  GeneticCircuit,
  PromoterParameters,
  PromoterStrength,
  GeneExpressionParameters,
  GeneExpressionResult,
  PathwayOptimizationParams,
  PathwayOptimizationResult,
  AssemblyPlan,
  AssemblyResult,
  SimulationParameters,
  SimulationResult,
  BiosafetAssessment,
  RiskAssessment,
  BIO_CONSTANTS,
  BioErrorCode,
  SyntheticBiologyError,
  TimeSeriesData,
  ChassisOrganism,
  AssemblyMethod,
  BiosafetLevel,
  MetabolicPathway,
  EnzymeReaction,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-012 Synthetic Biology SDK
 */
export class SyntheticBiologySDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Calculate promoter strength based on inducer concentration
   *
   * @param params - Promoter parameters
   * @returns Promoter strength and characteristics
   */
  calculatePromoterStrength(params: PromoterParameters): PromoterStrength {
    const {
      pmax,
      inducerConcentration = 0,
      kd = 1e-7,
      hillCoefficient = 1,
      basalLevel = pmax * 0.01,
    } = params;

    // Validate inputs
    if (pmax <= 0) {
      throw new SyntheticBiologyError(
        BioErrorCode.INVALID_PART,
        'Maximum promoter strength must be positive'
      );
    }

    if (inducerConcentration < 0) {
      throw new SyntheticBiologyError(
        BioErrorCode.INVALID_PART,
        'Inducer concentration cannot be negative'
      );
    }

    // Calculate using Hill equation: P = Pmax × [I]^n / (Kd + [I]^n)
    let strength: number;

    if (inducerConcentration === 0) {
      strength = basalLevel;
    } else {
      const inducerTerm = Math.pow(inducerConcentration, hillCoefficient);
      const kdTerm = Math.pow(kd, hillCoefficient);
      strength = pmax * (inducerTerm / (kdTerm + inducerTerm));

      // Add basal level
      strength = Math.max(strength, basalLevel);
    }

    // Calculate fold induction
    const foldInduction = basalLevel > 0 ? strength / basalLevel : 0;

    // Calculate saturation percentage
    const saturation = (strength / pmax) * 100;

    // Determine if fully induced (>95% of max)
    const isFullyInduced = saturation > 95;

    return {
      strength,
      foldInduction,
      saturation,
      isFullyInduced,
    };
  }

  /**
   * Calculate gene expression dynamics over time
   *
   * @param params - Gene expression parameters
   * @returns Expression dynamics and steady-state values
   */
  calculateGeneExpression(params: GeneExpressionParameters): GeneExpressionResult {
    const {
      transcriptionRate,
      translationRate,
      mRNADegradation,
      proteinDegradation,
      duration,
      timeStep = 1,
    } = params;

    // Validate inputs
    if (transcriptionRate < 0 || translationRate < 0) {
      throw new SyntheticBiologyError(
        BioErrorCode.INVALID_PART,
        'Rates must be non-negative'
      );
    }

    // Calculate steady-state values
    const steadyStateMRNA = transcriptionRate / mRNADegradation;
    const steadyStateProtein =
      (translationRate * steadyStateMRNA) / proteinDegradation;

    // Calculate half-lives
    const mRNAHalfLife = Math.log(2) / mRNADegradation;
    const proteinHalfLife = Math.log(2) / proteinDegradation;

    // Time to 90% steady state (approximately 2.3 time constants)
    const steadyStateTime = 2.3 / Math.min(mRNADegradation, proteinDegradation);

    // Simulate dynamics
    const timeSeries: TimeSeriesData[] = [];
    let mRNA = 0;
    let protein = 0;

    for (let t = 0; t <= duration; t += timeStep) {
      // Record current state
      timeSeries.push({ time: t, mRNA, protein });

      // Update using Euler method
      // dmRNA/dt = k_tx - k_deg_m * mRNA
      const dmRNA = transcriptionRate - mRNADegradation * mRNA;

      // dProtein/dt = k_tl * mRNA - k_deg_p * Protein
      const dProtein = translationRate * mRNA - proteinDegradation * protein;

      mRNA += dmRNA * timeStep;
      protein += dProtein * timeStep;

      // Ensure non-negative
      mRNA = Math.max(0, mRNA);
      protein = Math.max(0, protein);
    }

    return {
      finalmRNAConcentration: mRNA,
      finalProteinConcentration: protein,
      steadyStateTime,
      mRNAHalfLife,
      proteinHalfLife,
      timeSeries,
    };
  }

  /**
   * Design a genetic circuit from BioBrick parts
   *
   * @param parts - Array of BioBrick part IDs
   * @param host - Chassis organism
   * @param purpose - Circuit purpose
   * @returns Circuit design with predictions
   */
  designGeneticCircuit(
    parts: string[],
    host: ChassisOrganism,
    purpose: string
  ): GeneticCircuit {
    // Validate inputs
    if (parts.length === 0) {
      throw new SyntheticBiologyError(
        BioErrorCode.INVALID_PART,
        'At least one part required'
      );
    }

    // Generate circuit ID
    const id = `CIR-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    // Estimate circuit size (assume average part size)
    const estimatedSize = parts.length * 500; // 500 bp average

    // Determine biosafety level (default BSL-1 for basic constructs)
    let biosafety: BiosafetLevel = 'BSL-1';

    // Create circuit
    const circuit: GeneticCircuit = {
      id,
      name: purpose,
      parts,
      host,
      purpose,
      assemblyMethod: 'biobrick',
      size: estimatedSize,
      biosafety,
      created: new Date(),
    };

    return circuit;
  }

  /**
   * Optimize a metabolic pathway for target product
   *
   * @param params - Optimization parameters
   * @returns Optimized pathway with gene modifications
   */
  optimizeMetabolicPathway(
    params: PathwayOptimizationParams
  ): PathwayOptimizationResult {
    const { target, substrate, host, objective, constraints } = params;

    // This is a simplified optimization
    // In reality, would use FBA, constraint-based modeling, etc.

    // Example pathway (simplified)
    const reactions: EnzymeReaction[] = [
      {
        id: 'R1',
        enzyme: 'Glucose kinase',
        substrates: [{ name: substrate, coefficient: 1 }],
        products: [{ name: 'G6P', coefficient: 1 }],
      },
      {
        id: 'R2',
        enzyme: 'Product synthase',
        substrates: [{ name: 'G6P', coefficient: 1 }],
        products: [{ name: target, coefficient: 1 }],
      },
    ];

    const pathway: MetabolicPathway = {
      id: `PATH-${Date.now()}`,
      name: `${substrate} to ${target}`,
      substrate,
      product: target,
      reactions,
      theoreticalYield: 1.0, // mol/mol (simplified)
      host,
      classification: 'heterologous',
    };

    // Identify genes to modify
    const genesToOverexpress = ['glucose_kinase', 'product_synthase'];
    const genesToDelete = ['competing_pathway_enzyme'];

    // Calculate predicted yield (simplified)
    const predictedYield = 0.85; // 85% of theoretical

    // Identify bottlenecks
    const bottlenecks = ['R1']; // First reaction is rate-limiting

    // Flux distribution (simplified)
    const fluxDistribution: Record<string, number> = {
      R1: 10.0, // mmol/gDW/h
      R2: 8.5, // mmol/gDW/h
    };

    // Calculate optimization score
    const score = predictedYield * 100;

    return {
      pathway,
      genesToOverexpress,
      genesToDelete,
      predictedYield,
      fluxDistribution,
      bottlenecks,
      score,
    };
  }

  /**
   * Plan DNA assembly for a set of parts
   *
   * @param parts - BioBrick parts to assemble
   * @param method - Assembly method to use
   * @returns Assembly plan with instructions
   */
  planDNAAssembly(parts: BioBrickPart[], method: AssemblyMethod): AssemblyPlan {
    // Validate inputs
    if (parts.length < 2) {
      throw new SyntheticBiologyError(
        BioErrorCode.ASSEMBLY_FAILED,
        'At least 2 parts required for assembly'
      );
    }

    // Generate assembly ID
    const id = `ASM-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    // Calculate total size
    const expectedSize = parts.reduce((sum, part) => sum + part.length, 0);

    // Check size limit (typical plasmid limit ~20 kb)
    if (expectedSize > 20000) {
      throw new SyntheticBiologyError(
        BioErrorCode.SIZE_EXCEEDED,
        `Assembly size (${expectedSize} bp) exceeds typical plasmid limit (20 kb)`
      );
    }

    // Assembly order (sequential)
    const order = parts.map((_, i) => i);

    // Determine reagents based on method
    let reagents: string[] = [];
    let estimatedTime: number;
    let successProbability: number;

    switch (method) {
      case 'biobrick':
        reagents = ['EcoRI', 'XbaI', 'SpeI', 'PstI', 'T4 DNA ligase', 'Buffer'];
        estimatedTime = 4; // hours
        successProbability = 0.85;
        break;

      case 'golden-gate':
        reagents = ['BsaI', 'T4 DNA ligase', 'Buffer', 'ATP'];
        estimatedTime = 2;
        successProbability = 0.95;
        break;

      case 'gibson':
        reagents = ['Gibson Assembly Master Mix'];
        estimatedTime = 1;
        successProbability = 0.90;
        break;

      default:
        reagents = ['Standard cloning reagents'];
        estimatedTime = 6;
        successProbability = 0.75;
    }

    return {
      id,
      method,
      parts,
      order,
      expectedSize,
      reagents,
      estimatedTime,
      successProbability,
    };
  }

  /**
   * Assess biosafety level and risks for a genetic construct
   *
   * @param circuit - Genetic circuit to assess
   * @returns Biosafety assessment and recommendations
   */
  assessBiosafety(circuit: GeneticCircuit): BiosafetAssessment {
    // Default to BSL-1 for basic constructs
    let level: BiosafetLevel = 'BSL-1';
    let riskScore = 1;
    const hazards: string[] = [];
    const containment: string[] = ['Standard lab practices'];
    let killSwitchRequired = false;
    let ethicsReviewRequired = false;
    let dualUseConcern = false;

    // Check for concerning parts (simplified)
    const concerningKeywords = [
      'toxin',
      'pathogen',
      'virulence',
      'antibiotic',
      'resistance',
    ];

    const circuitString = circuit.parts.join(' ').toLowerCase();

    for (const keyword of concerningKeywords) {
      if (circuitString.includes(keyword)) {
        level = 'BSL-2';
        riskScore = Math.min(riskScore + 5, 25);
        hazards.push(keyword);
        containment.push('Biosafety cabinet required');

        if (keyword === 'toxin' || keyword === 'pathogen') {
          killSwitchRequired = true;
          ethicsReviewRequired = true;
        }

        if (keyword === 'virulence') {
          dualUseConcern = true;
        }
      }
    }

    // Determine risk category
    let riskCategory: BiosafetAssessment['riskCategory'];
    if (riskScore <= 5) riskCategory = 'low';
    else if (riskScore <= 10) riskCategory = 'medium';
    else if (riskScore <= 15) riskCategory = 'high';
    else riskCategory = 'extreme';

    // Mitigation strategies
    const mitigation: string[] = [];
    if (killSwitchRequired) {
      mitigation.push('Implement toxin-antitoxin kill switch');
      mitigation.push('Use auxotrophic strain for containment');
    }
    if (dualUseConcern) {
      mitigation.push('Restrict access to biological materials');
      mitigation.push('Implement institutional review before publication');
    }

    return {
      level,
      riskScore,
      riskCategory,
      hazards: hazards as any,
      containment,
      killSwitchRequired,
      ethicsReviewRequired,
      dualUseConcern,
      mitigation,
    };
  }

  /**
   * Simulate genetic circuit behavior over time
   *
   * @param params - Simulation parameters
   * @returns Time-series simulation results
   */
  simulateCircuit(params: SimulationParameters): SimulationResult {
    const startTime = Date.now();

    const { circuit, duration, timeStep, method = 'deterministic' } = params;

    // For demonstration, simulate a simple gene expression circuit
    const timeSeries: TimeSeriesData[] = [];

    // Simple parameters for demonstration
    const k_tx = 0.5; // transcription rate
    const k_tl = 0.1; // translation rate
    const k_deg_m = 0.00385; // mRNA degradation (3 min half-life)
    const k_deg_p = 0.0069; // protein degradation (100 min half-life)

    let mRNA = 0;
    let protein = 0;

    // Simulate
    for (let t = 0; t <= duration; t += timeStep) {
      timeSeries.push({ time: t, mRNA, protein });

      // Simple ODE integration (Euler method)
      const dmRNA = k_tx - k_deg_m * mRNA;
      const dProtein = k_tl * mRNA - k_deg_p * protein;

      mRNA += dmRNA * timeStep;
      protein += dProtein * timeStep;

      mRNA = Math.max(0, mRNA);
      protein = Math.max(0, protein);
    }

    // Final state
    const finalState: Record<string, number> = {
      mRNA,
      protein,
    };

    // Check if steady state reached (within 5% of theoretical)
    const steadyStateMRNA = k_tx / k_deg_m;
    const steadyStateProtein = (k_tl * steadyStateMRNA) / k_deg_p;

    const mRNADiff = Math.abs(mRNA - steadyStateMRNA) / steadyStateMRNA;
    const proteinDiff = Math.abs(protein - steadyStateProtein) / steadyStateProtein;

    const steadyStateReached = mRNADiff < 0.05 && proteinDiff < 0.05;

    // Time to steady state (if reached)
    let timeToSteadyState: number | undefined;
    if (steadyStateReached) {
      for (let i = 0; i < timeSeries.length; i++) {
        const point = timeSeries[i];
        const mDiff = Math.abs(point.mRNA - steadyStateMRNA) / steadyStateMRNA;
        const pDiff = Math.abs(point.protein - steadyStateProtein) / steadyStateProtein;

        if (mDiff < 0.05 && pDiff < 0.05) {
          timeToSteadyState = point.time;
          break;
        }
      }
    }

    const computationTime = Date.now() - startTime;

    return {
      id: `SIM-${Date.now()}`,
      success: true,
      timeSeries,
      finalState,
      steadyStateReached,
      timeToSteadyState,
      warnings: [],
      computationTime,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate Codon Adaptation Index (CAI)
   */
  calculateCAI(sequence: string, organism: ChassisOrganism): number {
    // Simplified CAI calculation
    // In reality, would use organism-specific codon usage tables

    // For demonstration, return a random value between 0.5 and 1.0
    // Higher CAI = better codon optimization
    return 0.5 + Math.random() * 0.5;
  }

  /**
   * Calculate GC content of DNA sequence
   */
  calculateGCContent(sequence: string): number {
    const gcCount = (sequence.match(/[GC]/gi) || []).length;
    return (gcCount / sequence.length) * 100;
  }

  /**
   * Validate DNA sequence
   */
  validateSequence(sequence: string): boolean {
    // Check if sequence contains only valid nucleotides
    return /^[ATGC]+$/i.test(sequence);
  }

  /**
   * Calculate protein molecular weight from sequence
   */
  calculateMolecularWeight(aminoAcidSequence: string): number {
    // Simplified calculation (average amino acid weight ≈ 110 Da)
    return aminoAcidSequence.length * 110;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate promoter strength (standalone function)
 */
export function calculatePromoterStrength(
  params: PromoterParameters
): PromoterStrength {
  const sdk = new SyntheticBiologySDK();
  return sdk.calculatePromoterStrength(params);
}

/**
 * Calculate gene expression (standalone function)
 */
export function calculateGeneExpression(
  params: GeneExpressionParameters
): GeneExpressionResult {
  const sdk = new SyntheticBiologySDK();
  return sdk.calculateGeneExpression(params);
}

/**
 * Design genetic circuit (standalone function)
 */
export function designGeneticCircuit(
  parts: string[],
  host: ChassisOrganism,
  purpose: string
): GeneticCircuit {
  const sdk = new SyntheticBiologySDK();
  return sdk.designGeneticCircuit(parts, host, purpose);
}

/**
 * Optimize metabolic pathway (standalone function)
 */
export function optimizeMetabolicPathway(
  params: PathwayOptimizationParams
): PathwayOptimizationResult {
  const sdk = new SyntheticBiologySDK();
  return sdk.optimizeMetabolicPathway(params);
}

/**
 * Plan DNA assembly (standalone function)
 */
export function planDNAAssembly(
  parts: BioBrickPart[],
  method: AssemblyMethod
): AssemblyPlan {
  const sdk = new SyntheticBiologySDK();
  return sdk.planDNAAssembly(parts, method);
}

/**
 * Assess biosafety (standalone function)
 */
export function assessBiosafety(circuit: GeneticCircuit): BiosafetAssessment {
  const sdk = new SyntheticBiologySDK();
  return sdk.assessBiosafety(circuit);
}

/**
 * Simulate circuit (standalone function)
 */
export function simulateCircuit(params: SimulationParameters): SimulationResult {
  const sdk = new SyntheticBiologySDK();
  return sdk.simulateCircuit(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { SyntheticBiologySDK };
export default SyntheticBiologySDK;
