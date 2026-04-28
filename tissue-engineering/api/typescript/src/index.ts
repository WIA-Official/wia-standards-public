/**
 * WIA-BIO-006: Tissue Engineering SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biomedical Engineering Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for tissue engineering including:
 * - Scaffold design and optimization
 * - Bioreactor culture optimization
 * - Mechanical property assessment
 * - Biocompatibility validation
 * - Protocol generation
 */

import {
  ScaffoldDesign,
  Scaffold,
  TissueConstruct,
  CultureConditions,
  BioreactorConfig,
  QualityAssessment,
  ImplantReadiness,
  BiocompatibilityTest,
  TissueEngineeringProtocol,
  MechanicalProperties,
  BioprintingParams,
  BioprintingResult,
  TISSUE_ENGINEERING_CONSTANTS,
  BioErrorCode,
  TissueEngineeringError,
  BiomaterialType,
  TissueType,
  CellType,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-006 Tissue Engineering SDK
 */
export class TissueEngineeringSDK {
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
   * Design a scaffold for tissue engineering
   *
   * @param design - Scaffold design parameters
   * @returns Designed scaffold with properties
   */
  designScaffold(design: ScaffoldDesign): Scaffold {
    // Validate inputs
    this.validateScaffoldDesign(design);

    const { dimensions, porosity, poreSize } = design;

    // Calculate volume
    const volume = dimensions.length * dimensions.width * dimensions.height; // mm³

    // Calculate void volume based on porosity
    const voidVolume = volume * (porosity / 100);

    // Calculate surface area (simplified rectangular prism)
    const surfaceArea =
      2 *
      (dimensions.length * dimensions.width +
        dimensions.length * dimensions.height +
        dimensions.width * dimensions.height); // mm²

    // Estimate cell capacity based on void volume and pore size
    const cellVolume = (4 / 3) * Math.PI * Math.pow(10 / 2, 3); // Assume ~10 μm cell diameter
    const cellCapacity = Math.floor((voidVolume * 1e9) / cellVolume); // Convert mm³ to μm³

    // Estimate mechanical properties based on material and porosity
    const mechanicalProperties = this.estimateMechanicalProperties(
      design.material,
      porosity
    );

    // Recommend fabrication method
    const fabricationMethod = this.recommendFabricationMethod(
      design.material,
      poreSize,
      porosity
    );

    // Calculate quality score
    const qualityScore = this.calculateScaffoldQuality(design);

    // Generate unique ID
    const id = `SCF-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    return {
      id,
      design,
      volume,
      surfaceArea,
      voidVolume,
      cellCapacity,
      mechanicalProperties,
      fabricationMethod,
      qualityScore,
      created: new Date(),
    };
  }

  /**
   * Optimize bioreactor culture conditions
   *
   * @param params - Optimization parameters
   * @returns Optimized culture conditions
   */
  optimizeCulture(params: {
    tissueConstruct: TissueConstruct;
    cellType: CellType;
    duration: number;
  }): {
    conditions: CultureConditions;
    predictedMaturation: number;
    qualityScore: number;
  } {
    const { tissueConstruct, cellType, duration } = params;

    // Determine optimal bioreactor type
    const bioreactorType = this.selectBioreactorType(
      tissueConstruct.scaffold.design.tissueType
    );

    // Calculate optimal flow rate for perfusion
    const flowRate = this.calculateFlowRate(
      tissueConstruct.scaffold.volume,
      tissueConstruct.seedingDensity
    );

    // Set oxygen level based on tissue type
    const oxygenLevel = this.getOptimalOxygenLevel(
      tissueConstruct.scaffold.design.tissueType
    );

    // Create bioreactor configuration
    const bioreactor: BioreactorConfig = {
      id: `BR-${Date.now()}`,
      type: bioreactorType,
      volume: tissueConstruct.scaffold.volume * 10, // 10x scaffold volume
      flowRate,
      temperature: TISSUE_ENGINEERING_CONSTANTS.CULTURE_TEMPERATURE,
      pH: TISSUE_ENGINEERING_CONSTANTS.CULTURE_PH,
      oxygenLevel,
      co2Level: TISSUE_ENGINEERING_CONSTANTS.CULTURE_CO2,
      cultureDuration: duration,
    };

    // Create culture conditions
    const conditions: CultureConditions = {
      bioreactor,
      medium: {
        baseMedium: 'DMEM/F12',
        serum: {
          type: 'FBS',
          percentage: 10,
        },
        antibiotics: ['penicillin', 'streptomycin'],
        glucose: 5.5, // mM
        buffer: 'HEPES',
      },
      feedingSchedule: {
        frequency: 2, // days
        exchangePercentage: 50,
        perfusionRate: bioreactorType === 'perfusion' ? flowRate : undefined,
      },
      monitoring: {
        temperature: {
          type: 'RTD',
          samplingFrequency: 300, // 5 minutes
          accuracy: 0.1,
        },
        pH: {
          type: 'glass-electrode',
          samplingFrequency: 600, // 10 minutes
          accuracy: 0.05,
        },
        dissolvedOxygen: {
          type: 'optical',
          samplingFrequency: 300,
          accuracy: 2,
        },
      },
    };

    // Predict maturation time
    const predictedMaturation = this.predictMaturation(
      tissueConstruct,
      conditions,
      duration
    );

    // Calculate quality score
    const qualityScore = this.calculateCultureQuality(conditions);

    return {
      conditions,
      predictedMaturation,
      qualityScore,
    };
  }

  /**
   * Assess mechanical properties of tissue construct
   *
   * @param params - Assessment parameters
   * @returns Mechanical test results
   */
  assessMechanicalProperties(params: {
    force: number; // N
    area: number; // m²
    displacement: number; // m
    initialLength: number; // m
  }): {
    stress: number; // Pa
    strain: number;
    youngsModulus: number; // Pa
    matchPercentage: number;
  } {
    const { force, area, displacement, initialLength } = params;

    // Calculate stress: σ = F / A
    const stress = force / area;

    // Calculate strain: ε = ΔL / L₀
    const strain = displacement / initialLength;

    // Calculate Young's modulus: E = σ / ε
    const youngsModulus = strain !== 0 ? stress / strain : 0;

    // Compare to typical values (assuming bone for example)
    const typicalModulus = 15e9; // Pa for bone
    const matchPercentage = Math.min(
      100,
      (youngsModulus / typicalModulus) * 100
    );

    return {
      stress,
      strain,
      youngsModulus,
      matchPercentage,
    };
  }

  /**
   * Validate biocompatibility of material/construct
   *
   * @param params - Validation parameters
   * @returns Biocompatibility assessment
   */
  validateBiocompatibility(params: {
    material: BiomaterialType;
    cellType: CellType;
    viabilityPercentage: number;
  }): BiocompatibilityTest {
    const { material, cellType, viabilityPercentage } = params;

    // Cytotoxicity assessment (ISO 10993-5)
    const cytotoxicity = {
      viabilityPercentage,
      passed: viabilityPercentage >= 70,
    };

    // Simulate immunogenicity (would be actual measurements in real implementation)
    const cytokineLevels = this.estimateCytokineLevels(material);
    const foldIncrease = Math.max(...Object.values(cytokineLevels));

    const immunogenicity = {
      cytokineLevels,
      foldIncrease,
      passed: foldIncrease < 2.0,
    };

    // Overall biocompatibility
    const passed = cytotoxicity.passed && immunogenicity.passed;

    return {
      cytotoxicity,
      immunogenicity,
      passed,
    };
  }

  /**
   * Generate comprehensive tissue engineering protocol
   *
   * @param params - Protocol parameters
   * @returns Complete protocol
   */
  generateProtocol(params: {
    tissueType: TissueType;
    scaffoldDesign: ScaffoldDesign;
    cellType: CellType;
    duration: number;
  }): TissueEngineeringProtocol {
    const { tissueType, scaffoldDesign, cellType, duration } = params;

    const id = `PROTO-${Date.now()}`;
    const title = `${tissueType.toUpperCase()} Tissue Engineering Protocol`;

    // Generate seeding protocol
    const seeding = {
      cellType,
      cellSource: this.getCellSource(cellType),
      density: this.getOptimalSeedingDensity(tissueType),
      method: this.getOptimalSeedingMethod(tissueType),
      incubationTime: 4, // hours
      steps: this.generateSeedingSteps(cellType, scaffoldDesign),
    };

    // Generate culture protocol
    const culture = {
      conditions: this.optimizeCulture({
        tissueConstruct: {
          id: 'temp',
          scaffold: this.designScaffold(scaffoldDesign),
          cellType,
          seedingDensity: seeding.density,
          seedingMethod: seeding.method,
          cellViability: 0.9,
          maturationTime: duration,
          created: new Date(),
          updated: new Date(),
        },
        cellType,
        duration,
      }).conditions,
      duration,
      mediumChanges: this.generateMediumChangeSchedule(duration),
      monitoring: this.generateMonitoringSchedule(),
      milestones: this.generateMilestones(tissueType, duration),
    };

    // Generate quality control protocol
    const qualityControl = {
      sterility: {
        frequency: 'every batch',
        methods: ['USP <71>', 'LAL assay', 'mycoplasma PCR'],
      },
      viability: {
        timepoints: this.generateViabilityTimepoints(duration),
        method: 'live-dead',
        acceptanceCriteria: 80,
      },
      mechanical: {
        timepoint: duration,
        method: 'compression',
        acceptanceCriteria: this.getTargetMechanicalProperties(tissueType),
      },
      functional: {
        timepoints: [duration],
        assays: this.getTissueSpecificAssays(tissueType),
      },
    };

    return {
      id,
      title,
      tissueType,
      scaffold: scaffoldDesign,
      seeding,
      culture,
      qualityControl,
      timeline: duration,
      successRate: this.estimateSuccessRate(tissueType),
      references: this.getReferences(tissueType),
      created: new Date(),
    };
  }

  /**
   * Perform bioprinting simulation
   *
   * @param params - Bioprinting parameters
   * @returns Bioprinting result
   */
  simulateBioprinting(params: BioprintingParams): BioprintingResult {
    const { bioink, nozzleDiameter, printSpeed, layerHeight } = params;

    // Estimate print fidelity based on parameters
    const viscosityFactor = Math.min(1, 1000 / bioink.viscosity);
    const speedFactor = Math.max(0, 1 - printSpeed / 100);
    const fidelity = (viscosityFactor + speedFactor) / 2;

    // Estimate cell viability post-print
    // Lower pressure and slower speed = higher viability
    const pressure = params.pressure || 50;
    const shearStress = (pressure * printSpeed) / nozzleDiameter;
    const viabilityLoss = Math.min(0.2, shearStress / 1000);
    const cellViability = Math.max(0.7, 0.95 - viabilityLoss);

    // Estimate print duration (simplified)
    const estimatedVolume = 1000; // mm³
    const volumePerSecond = (Math.PI * Math.pow(nozzleDiameter / 1000, 2) * printSpeed) / 4;
    const duration = volumePerSecond > 0 ? estimatedVolume / volumePerSecond : 0;

    // Determine success
    const success = fidelity > 0.8 && cellViability > 0.8;

    const messages: string[] = [];
    if (fidelity < 0.8) {
      messages.push('Print fidelity below 80%. Consider optimizing bioink viscosity or print speed.');
    }
    if (cellViability < 0.8) {
      messages.push('Cell viability below 80%. Reduce pressure or print speed to minimize shear stress.');
    }
    if (success) {
      messages.push('Bioprinting simulation successful!');
    }

    return {
      id: `PRINT-${Date.now()}`,
      parameters: params,
      fidelity,
      cellViability,
      duration,
      success,
      messages,
      timestamp: new Date(),
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Validate scaffold design parameters
   */
  private validateScaffoldDesign(design: ScaffoldDesign): void {
    if (design.porosity < TISSUE_ENGINEERING_CONSTANTS.MIN_POROSITY) {
      throw new TissueEngineeringError(
        BioErrorCode.INSUFFICIENT_POROSITY,
        `Porosity ${design.porosity}% below minimum ${TISSUE_ENGINEERING_CONSTANTS.MIN_POROSITY}%`
      );
    }

    if (design.porosity > TISSUE_ENGINEERING_CONSTANTS.MAX_POROSITY) {
      throw new TissueEngineeringError(
        BioErrorCode.INSUFFICIENT_POROSITY,
        `Porosity ${design.porosity}% above maximum ${TISSUE_ENGINEERING_CONSTANTS.MAX_POROSITY}%`
      );
    }

    if (design.dimensions.length <= 0 || design.dimensions.width <= 0 || design.dimensions.height <= 0) {
      throw new TissueEngineeringError(
        BioErrorCode.INVALID_PARAMETERS,
        'Scaffold dimensions must be positive'
      );
    }
  }

  /**
   * Estimate mechanical properties based on material and porosity
   */
  private estimateMechanicalProperties(
    material: BiomaterialType,
    porosity: number
  ): MechanicalProperties {
    // Base properties for each material (from literature)
    const baseProperties: { [key in BiomaterialType]?: MechanicalProperties } = {
      collagen: {
        youngsModulus: 5e6, // 5 MPa
        ultimateStrength: 0.5e6,
        strainAtFailure: 0.1,
      },
      pcl: {
        youngsModulus: 300e6, // 300 MPa
        ultimateStrength: 20e6,
        strainAtFailure: 0.05,
      },
      'pcl-collagen': {
        youngsModulus: 100e6, // 100 MPa
        ultimateStrength: 10e6,
        strainAtFailure: 0.08,
      },
      pla: {
        youngsModulus: 3e9, // 3 GPa
        ultimateStrength: 50e6,
        strainAtFailure: 0.03,
      },
      gelatin: {
        youngsModulus: 0.5e6, // 0.5 MPa
        ultimateStrength: 0.1e6,
        strainAtFailure: 0.3,
      },
    };

    const base = baseProperties[material] || {
      youngsModulus: 10e6,
      ultimateStrength: 1e6,
      strainAtFailure: 0.1,
    };

    // Adjust for porosity (Gibson-Ashby model)
    const densityRatio = 1 - porosity / 100;
    const porosityCorrectionE = Math.pow(densityRatio, 2);
    const porosityCorrectionS = Math.pow(densityRatio, 1.5);

    return {
      youngsModulus: base.youngsModulus * porosityCorrectionE,
      ultimateStrength: base.ultimateStrength * porosityCorrectionS,
      strainAtFailure: base.strainAtFailure,
    };
  }

  /**
   * Recommend fabrication method based on material and properties
   */
  private recommendFabricationMethod(
    material: BiomaterialType,
    poreSize: number,
    porosity: number
  ): Scaffold['fabricationMethod'] {
    if (poreSize < 50) return 'electrospinning';
    if (material === 'collagen' || material === 'gelatin') {
      return 'freeze-drying';
    }
    if (material === 'pcl' || material === 'pla') {
      return '3d-printing';
    }
    if (porosity > 85) return 'gas-foaming';
    return 'bioprinting';
  }

  /**
   * Calculate scaffold quality score
   */
  private calculateScaffoldQuality(design: ScaffoldDesign): number {
    let score = 1.0;

    // Porosity score
    const optimalPorosity = 75;
    const porosityDev = Math.abs(design.porosity - optimalPorosity) / optimalPorosity;
    score *= Math.max(0.5, 1 - porosityDev);

    // Pore size score
    const tissue = design.tissueType;
    const optimalRange = TISSUE_ENGINEERING_CONSTANTS.PORE_SIZES.FIBROBLASTS;
    const inRange = design.poreSize >= optimalRange.min && design.poreSize <= optimalRange.max;
    score *= inRange ? 1.0 : 0.8;

    // Interconnectivity score
    const interconnectivity = design.interconnectivity || 90;
    score *= interconnectivity / 100;

    return Math.max(0, Math.min(1, score));
  }

  /**
   * Select optimal bioreactor type for tissue
   */
  private selectBioreactorType(tissue: TissueType): BioreactorConfig['type'] {
    const bioreactorMap: { [key in TissueType]?: BioreactorConfig['type'] } = {
      bone: 'perfusion',
      cartilage: 'rotating-wall',
      cardiac: 'perfusion',
      liver: 'perfusion',
      skin: 'spinner-flask',
    };
    return bioreactorMap[tissue] || 'spinner-flask';
  }

  /**
   * Calculate optimal flow rate for perfusion bioreactor
   */
  private calculateFlowRate(volume: number, cellDensity: number): number {
    // Flow rate = f(volume, cell density)
    // Typical: 0.1-5 mL/min
    const baseFlow = 0.5; // mL/min per cm³
    return Math.min(5, baseFlow * (volume / 1000)); // volume in mm³, convert to cm³
  }

  /**
   * Get optimal oxygen level for tissue type
   */
  private getOptimalOxygenLevel(tissue: TissueType): number {
    const oxygenMap: { [key in TissueType]?: number } = {
      bone: 10,
      cartilage: 5,
      liver: 15,
      cardiac: 10,
      skin: 20,
    };
    return oxygenMap[tissue] || 10;
  }

  /**
   * Predict tissue maturation time
   */
  private predictMaturation(
    construct: TissueConstruct,
    conditions: CultureConditions,
    duration: number
  ): number {
    // Simple heuristic: higher cell density and optimal conditions = faster maturation
    const baseTime = duration;
    const densityFactor = construct.seedingDensity / 1e7; // Normalized
    const maturationTime = baseTime / Math.max(0.5, Math.min(2, densityFactor));
    return Math.round(maturationTime);
  }

  /**
   * Calculate culture quality score
   */
  private calculateCultureQuality(conditions: CultureConditions): number {
    let score = 1.0;

    // Temperature control
    if (conditions.bioreactor.temperature === TISSUE_ENGINEERING_CONSTANTS.CULTURE_TEMPERATURE) {
      score *= 1.0;
    } else {
      score *= 0.9;
    }

    // pH control
    if (
      conditions.bioreactor.pH >= 7.2 &&
      conditions.bioreactor.pH <= 7.6
    ) {
      score *= 1.0;
    } else {
      score *= 0.8;
    }

    // Oxygen level (reasonable range)
    if (
      conditions.bioreactor.oxygenLevel >= 2 &&
      conditions.bioreactor.oxygenLevel <= 20
    ) {
      score *= 1.0;
    } else {
      score *= 0.7;
    }

    return Math.max(0, Math.min(1, score));
  }

  /**
   * Estimate cytokine levels for immunogenicity
   */
  private estimateCytokineLevels(material: BiomaterialType): { [key: string]: number } {
    // Natural materials typically have lower immunogenicity
    const naturalMaterials: BiomaterialType[] = ['collagen', 'gelatin', 'chitosan', 'alginate'];
    const isNatural = naturalMaterials.includes(material);

    return {
      'IL-1β': isNatural ? 1.2 : 1.5,
      'IL-6': isNatural ? 1.1 : 1.6,
      'TNF-α': isNatural ? 1.15 : 1.7,
    };
  }

  /**
   * Get cell source description
   */
  private getCellSource(cellType: CellType): string {
    const sources: { [key in CellType]?: string } = {
      osteoblasts: 'Bone marrow or periosteum',
      chondrocytes: 'Articular cartilage',
      fibroblasts: 'Dermal tissue',
      hepatocytes: 'Liver biopsy',
      'mesenchymal-stem': 'Bone marrow or adipose tissue',
    };
    return sources[cellType] || 'Primary tissue or cell line';
  }

  /**
   * Get optimal seeding density for tissue type
   */
  private getOptimalSeedingDensity(tissue: TissueType): number {
    const densityMap: { [key in TissueType]?: number } = {
      bone: 1e7,
      cartilage: 4e7,
      skin: 2e6,
      liver: 1e8,
      cardiac: 5e7,
    };
    return densityMap[tissue] || 1e7; // cells/cm³
  }

  /**
   * Get optimal seeding method for tissue type
   */
  private getOptimalSeedingMethod(tissue: TissueType): 'static' | 'dynamic' | 'vacuum' | 'centrifugal' {
    const methodMap: { [key in TissueType]?: 'static' | 'dynamic' | 'vacuum' | 'centrifugal' } = {
      bone: 'vacuum',
      cartilage: 'centrifugal',
      skin: 'static',
      liver: 'perfusion' as 'dynamic',
    };
    return methodMap[tissue] || 'dynamic';
  }

  /**
   * Generate seeding steps
   */
  private generateSeedingSteps(cellType: CellType, design: ScaffoldDesign): string[] {
    return [
      `1. Harvest ${cellType} from source tissue`,
      '2. Expand cells in culture to sufficient numbers',
      `3. Prepare cell suspension at optimal density`,
      `4. Pre-wet scaffold with culture medium`,
      `5. Seed cells onto scaffold using recommended method`,
      '6. Incubate for 2-4 hours to allow cell attachment',
      '7. Add culture medium and transfer to bioreactor',
    ];
  }

  /**
   * Generate medium change schedule
   */
  private generateMediumChangeSchedule(duration: number): Array<{ day: number; action: string }> {
    const schedule = [];
    for (let day = 2; day <= duration; day += 2) {
      schedule.push({
        day,
        action: 'Change 50% of culture medium',
      });
    }
    return schedule;
  }

  /**
   * Generate monitoring schedule
   */
  private generateMonitoringSchedule(): Array<{ parameter: string; frequency: string }> {
    return [
      { parameter: 'Temperature', frequency: 'Continuous (every 5 min)' },
      { parameter: 'pH', frequency: 'Continuous (every 10 min)' },
      { parameter: 'Dissolved oxygen', frequency: 'Continuous (every 5 min)' },
      { parameter: 'Cell viability', frequency: 'Every 7 days' },
      { parameter: 'Glucose/lactate', frequency: 'Daily' },
    ];
  }

  /**
   * Generate milestones for tissue development
   */
  private generateMilestones(tissue: TissueType, duration: number): Array<{ day: number; milestone: string }> {
    const milestones = [
      { day: 1, milestone: 'Initial cell attachment' },
      { day: 3, milestone: 'Cell spreading and proliferation begins' },
      { day: 7, milestone: 'Cell confluency reached' },
    ];

    if (duration >= 14) {
      milestones.push({ day: 14, milestone: 'ECM deposition initiated' });
    }
    if (duration >= 21) {
      milestones.push({ day: 21, milestone: 'Tissue maturation and remodeling' });
    }
    if (duration >= 28) {
      milestones.push({ day: duration, milestone: 'Final tissue construct ready for assessment' });
    }

    return milestones;
  }

  /**
   * Generate viability testing timepoints
   */
  private generateViabilityTimepoints(duration: number): number[] {
    const timepoints = [1, 7];
    if (duration >= 14) timepoints.push(14);
    if (duration >= 21) timepoints.push(21);
    if (duration >= 28) timepoints.push(duration);
    return timepoints;
  }

  /**
   * Get target mechanical properties for tissue type
   */
  private getTargetMechanicalProperties(tissue: TissueType): MechanicalProperties {
    const targets: { [key in TissueType]?: MechanicalProperties } = {
      bone: {
        youngsModulus: 15e9, // 15 GPa
        ultimateStrength: 150e6, // 150 MPa
        strainAtFailure: 0.02,
      },
      cartilage: {
        youngsModulus: 1e6, // 1 MPa
        ultimateStrength: 15e6, // 15 MPa
        strainAtFailure: 0.15,
      },
      skin: {
        youngsModulus: 1e6, // 1 MPa
        ultimateStrength: 10e6, // 10 MPa
        strainAtFailure: 0.5,
      },
    };
    return (
      targets[tissue] || {
        youngsModulus: 10e6,
        ultimateStrength: 5e6,
        strainAtFailure: 0.1,
      }
    );
  }

  /**
   * Get tissue-specific functional assays
   */
  private getTissueSpecificAssays(tissue: TissueType): string[] {
    const assays: { [key in TissueType]?: string[] } = {
      bone: ['Alkaline phosphatase', 'Osteocalcin', 'Calcium deposition (Alizarin Red)'],
      cartilage: ['GAG content (Alcian Blue)', 'Collagen type II', 'Aggrecan'],
      liver: ['Albumin secretion', 'Urea synthesis', 'CYP450 activity'],
      cardiac: ['Troponin expression', 'Connexin-43', 'Contractility'],
    };
    return assays[tissue] || ['Cell proliferation', 'ECM production', 'Gene expression'];
  }

  /**
   * Estimate success rate for tissue type
   */
  private estimateSuccessRate(tissue: TissueType): number {
    const rates: { [key in TissueType]?: number } = {
      skin: 0.9,
      bone: 0.85,
      cartilage: 0.8,
      liver: 0.65,
      cardiac: 0.6,
    };
    return rates[tissue] || 0.75;
  }

  /**
   * Get references for tissue type
   */
  private getReferences(tissue: TissueType): string[] {
    return [
      'Langer, R., Vacanti, J.P. (1993). Tissue Engineering. Science.',
      'Murphy, S.V., Atala, A. (2014). 3D Bioprinting. Nature Biotechnology.',
      'Hollister, S.J. (2005). Porous Scaffold Design. Nature Materials.',
    ];
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Design scaffold (standalone function)
 */
export function designScaffold(design: ScaffoldDesign): Scaffold {
  const sdk = new TissueEngineeringSDK();
  return sdk.designScaffold(design);
}

/**
 * Optimize culture (standalone function)
 */
export function optimizeCulture(params: {
  tissueConstruct: TissueConstruct;
  cellType: CellType;
  duration: number;
}): {
  conditions: CultureConditions;
  predictedMaturation: number;
  qualityScore: number;
} {
  const sdk = new TissueEngineeringSDK();
  return sdk.optimizeCulture(params);
}

/**
 * Assess mechanical properties (standalone function)
 */
export function assessMechanicalProperties(params: {
  force: number;
  area: number;
  displacement: number;
  initialLength: number;
}): {
  stress: number;
  strain: number;
  youngsModulus: number;
  matchPercentage: number;
} {
  const sdk = new TissueEngineeringSDK();
  return sdk.assessMechanicalProperties(params);
}

/**
 * Validate biocompatibility (standalone function)
 */
export function validateBiocompatibility(params: {
  material: BiomaterialType;
  cellType: CellType;
  viabilityPercentage: number;
}): BiocompatibilityTest {
  const sdk = new TissueEngineeringSDK();
  return sdk.validateBiocompatibility(params);
}

/**
 * Generate protocol (standalone function)
 */
export function generateProtocol(params: {
  tissueType: TissueType;
  scaffoldDesign: ScaffoldDesign;
  cellType: CellType;
  duration: number;
}): TissueEngineeringProtocol {
  const sdk = new TissueEngineeringSDK();
  return sdk.generateProtocol(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { TissueEngineeringSDK };
export default TissueEngineeringSDK;
