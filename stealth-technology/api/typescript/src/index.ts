/**
 * WIA-DEF-009: Stealth Technology SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for stealth technology including:
 * - Radar cross-section calculations
 * - Infrared signature analysis
 * - Acoustic signature assessment
 * - Visual camouflage evaluation
 * - Multi-spectrum stealth optimization
 */

import {
  RCSRequest,
  RCSResponse,
  IRSignatureRequest,
  IRSignatureResponse,
  AcousticSignatureRequest,
  AcousticSignatureResponse,
  VisualSignatureRequest,
  VisualSignatureResponse,
  StealthPlatform,
  StealthPerformance,
  ThreatEnvironment,
  OptimizationParams,
  OptimizationResult,
  RAMConfiguration,
  RAMPerformance,
  STEALTH_CONSTANTS,
  StealthErrorCode,
  StealthTechError,
  StealthClassification,
  AcousticClassification,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-009 Stealth Technology SDK
 */
export class StealthTechSDK {
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
   * Calculate radar cross-section (RCS)
   *
   * @param params - RCS calculation parameters
   * @returns RCS calculation result
   */
  calculateRCS(params: RCSRequest): RCSResponse {
    const {
      frequency,
      targetShape,
      dimensions,
      surfaceArea,
      ramCoating,
      incidenceAngle,
      polarization = 'HH',
    } = params;

    // Validate inputs
    if (frequency <= 0) {
      throw new StealthTechError(
        StealthErrorCode.INVALID_FREQUENCY,
        'Frequency must be positive'
      );
    }

    if (surfaceArea <= 0) {
      throw new StealthTechError(
        StealthErrorCode.IMPOSSIBLE_GEOMETRY,
        'Surface area must be positive'
      );
    }

    const c = STEALTH_CONSTANTS.SPEED_OF_LIGHT;
    const wavelength = c / frequency;

    // Determine region
    const targetSize = Math.max(dimensions.length, dimensions.width, dimensions.height);
    let region: 'rayleigh' | 'mie' | 'optical';

    if (targetSize < wavelength / 10) {
      region = 'rayleigh';
    } else if (targetSize > wavelength * 10) {
      region = 'optical';
    } else {
      region = 'mie';
    }

    // Calculate baseline RCS based on shape
    let baselineRCS: number;

    switch (targetShape) {
      case 'sphere': {
        const radius = Math.cbrt((3 * surfaceArea) / (4 * Math.PI));
        if (region === 'optical') {
          baselineRCS = Math.PI * radius * radius;
        } else {
          // Rayleigh scattering
          baselineRCS = Math.pow((2 * Math.PI * radius) / wavelength, 4) * radius * radius;
        }
        break;
      }

      case 'flat-plate': {
        if (region === 'optical') {
          const angle = (incidenceAngle * Math.PI) / 180;
          baselineRCS = (4 * Math.PI * surfaceArea * surfaceArea * Math.cos(angle) * Math.cos(angle)) / (wavelength * wavelength);
        } else {
          baselineRCS = surfaceArea;
        }
        break;
      }

      case 'cylinder': {
        const radius = dimensions.width / 2;
        const length = dimensions.length;
        if (region === 'optical') {
          baselineRCS = (2 * radius * length * length) / wavelength;
        } else {
          baselineRCS = Math.PI * radius * length;
        }
        break;
      }

      case 'faceted': {
        // Faceted design with optimized angles
        const avgFacetArea = surfaceArea / 20; // Assume 20 facets
        const angle = (incidenceAngle * Math.PI) / 180;
        baselineRCS = (4 * Math.PI * avgFacetArea * avgFacetArea * Math.cos(angle) * Math.cos(angle)) / (wavelength * wavelength);
        // Apply reduction factor for faceting
        baselineRCS *= 0.1;
        break;
      }

      case 'complex':
      default: {
        // Complex shape - estimate from surface area
        baselineRCS = surfaceArea * 0.5;
        break;
      }
    }

    // Apply RAM coating reduction if applicable
    let rcs = baselineRCS;
    let ramReduction = 1.0;

    if (ramCoating) {
      // RAM typically provides 10-20 dB reduction
      ramReduction = 0.1; // -10 dB
      rcs *= ramReduction;
    }

    // Apply incidence angle dependency
    const angle = (incidenceAngle * Math.PI) / 180;
    const angleFactor = Math.pow(Math.cos(angle), 2);
    rcs *= (angleFactor + 0.1); // Prevent zero RCS

    // Calculate aspect dependency
    const frontalRCS = rcs;
    const sideRCS = rcs * 2.0; // Side aspect typically 2-3x larger
    const rearRCS = rcs * 1.5;  // Rear aspect slightly larger

    // Convert to dBsm
    const dBsm = 10 * Math.log10(rcs);

    // Classify RCS
    let classification: StealthClassification;
    if (dBsm < -30) classification = 'very-low';
    else if (dBsm < -20) classification = 'low';
    else if (dBsm < 0) classification = 'moderate';
    else if (dBsm < 10) classification = 'high';
    else classification = 'very-high';

    // Calculate reduction factor
    const reductionFactor = baselineRCS / rcs;
    const reductionDB = 10 * Math.log10(reductionFactor);

    return {
      value: rcs,
      dBsm,
      wavelength,
      classification,
      reductionFactor,
      reductionDB,
      aspectDependency: {
        frontal: frontalRCS,
        side: sideRCS,
        rear: rearRCS,
      },
      region,
      contributions: {
        geometric: baselineRCS,
        material: rcs / baselineRCS,
        shaping: ramCoating ? 0.1 : 1.0,
      },
    };
  }

  /**
   * Evaluate infrared signature
   *
   * @param params - IR signature parameters
   * @returns IR signature evaluation
   */
  evaluateIRSignature(params: IRSignatureRequest): IRSignatureResponse {
    const {
      surfaceTemp,
      emissivity,
      surfaceArea,
      coolingSystem = 'none',
      exhaustTemp,
      exhaustArea,
      ambientTemp = 273,
      lowECoating = false,
    } = params;

    // Validate inputs
    if (surfaceTemp < 0 || surfaceTemp > 3000) {
      throw new StealthTechError(
        StealthErrorCode.TEMPERATURE_OUT_OF_BOUNDS,
        'Temperature must be between 0 and 3000 K'
      );
    }

    if (emissivity < 0 || emissivity > 1) {
      throw new StealthTechError(
        StealthErrorCode.INVALID_PARAMETERS,
        'Emissivity must be between 0 and 1'
      );
    }

    const sigma = STEALTH_CONSTANTS.STEFAN_BOLTZMANN;

    // Apply low-E coating adjustment
    let effectiveEmissivity = emissivity;
    if (lowECoating) {
      effectiveEmissivity *= 0.5; // Low-E coating reduces emissivity by ~50%
    }

    // Calculate airframe thermal radiation
    const airframePower = effectiveEmissivity * sigma * surfaceArea * Math.pow(surfaceTemp, 4);

    // Calculate exhaust power if applicable
    let exhaustPower = 0;
    if (exhaustTemp && exhaustArea) {
      exhaustPower = 0.9 * sigma * exhaustArea * Math.pow(exhaustTemp, 4);
    }

    // Apply cooling system reduction
    let coolingFactor = 1.0;
    if (coolingSystem === 'passive') {
      coolingFactor = 0.7; // 30% reduction
    } else if (coolingSystem === 'active') {
      coolingFactor = 0.4; // 60% reduction
    }

    const totalPower = (airframePower + exhaustPower) * coolingFactor;

    // Distribute power across IR bands (approximate)
    const mwirFraction = 0.3; // 30% in MWIR (3-5 μm)
    const lwirFraction = 0.6; // 60% in LWIR (8-12 μm)
    const swirFraction = 0.1; // 10% in SWIR (1-3 μm)

    const mwirPower = totalPower * mwirFraction;
    const lwirPower = totalPower * lwirFraction;
    const swirPower = totalPower * swirFraction;

    // Estimate detection range (simplified model)
    // Range proportional to fourth root of power
    const mwirRange = 1000 * Math.pow(mwirPower / 1000, 0.25);
    const lwirRange = 800 * Math.pow(lwirPower / 1000, 0.25);

    // Classify IR signature
    let classification: StealthClassification;
    if (totalPower < 500) classification = 'very-low';
    else if (totalPower < 1500) classification = 'low';
    else if (totalPower < 5000) classification = 'moderate';
    else if (totalPower < 15000) classification = 'high';
    else classification = 'very-high';

    const temperatureContrast = surfaceTemp - ambientTemp;

    return {
      totalPower,
      mwirPower,
      lwirPower,
      swirPower,
      detectionRange: {
        mwir: mwirRange,
        lwir: lwirRange,
      },
      classification,
      temperatureContrast,
      components: {
        airframe: airframePower * coolingFactor,
        exhaust: exhaustPower * coolingFactor,
        engine: exhaustPower * coolingFactor * 0.5,
      },
    };
  }

  /**
   * Assess acoustic signature
   *
   * @param params - Acoustic signature parameters
   * @returns Acoustic signature assessment
   */
  assessAcousticSignature(params: AcousticSignatureRequest): AcousticSignatureResponse {
    const {
      sourcePower,
      frequency,
      distance,
      dampening = 0,
      environment,
      backgroundNoise = 40,
    } = params;

    // Validate inputs
    if (sourcePower <= 0) {
      throw new StealthTechError(
        StealthErrorCode.INVALID_PARAMETERS,
        'Source power must be positive'
      );
    }

    const pRef = STEALTH_CONSTANTS.ACOUSTIC_REF_POWER;

    // Calculate sound power level
    const soundPowerLevel = 10 * Math.log10(sourcePower / pRef);

    // Calculate sound pressure level at distance
    // L_p = L_w - 20*log10(r) - 11
    let soundPressureLevel = soundPowerLevel - 20 * Math.log10(distance) - 11;

    // Apply dampening
    soundPressureLevel -= dampening;

    // Environmental attenuation
    let atmosphericAttenuation = 0;
    if (environment === 'air') {
      // Air absorption increases with frequency and distance
      atmosphericAttenuation = (frequency / 1000) * (distance / 100) * 0.5;
    } else if (environment === 'water') {
      // Less attenuation in water
      atmosphericAttenuation = (frequency / 10000) * (distance / 1000) * 0.2;
    }

    soundPressureLevel -= atmosphericAttenuation;

    // Calculate detection range (where SPL equals background noise + 10 dB)
    const detectionThreshold = backgroundNoise + 10;
    const detectionRange = distance * Math.pow(10, (soundPressureLevel - detectionThreshold) / 20);

    // Classify acoustic signature
    let classification: AcousticClassification;
    if (soundPressureLevel < 50) classification = 'very-quiet';
    else if (soundPressureLevel < 65) classification = 'quiet';
    else if (soundPressureLevel < 80) classification = 'moderate';
    else if (soundPressureLevel < 95) classification = 'loud';
    else classification = 'very-loud';

    // Signal-to-noise ratio
    const signalToNoise = soundPressureLevel - backgroundNoise;

    // Frequency analysis (simplified)
    const frequencyAnalysis = {
      lowFrequency: frequency < 500 ? soundPressureLevel : soundPressureLevel - 10,
      midFrequency: frequency >= 500 && frequency <= 2000 ? soundPressureLevel : soundPressureLevel - 5,
      highFrequency: frequency > 2000 ? soundPressureLevel : soundPressureLevel - 15,
    };

    return {
      soundPowerLevel,
      soundPressureLevel,
      detectionRange: Math.max(0, detectionRange),
      classification,
      signalToNoise,
      frequencyAnalysis,
    };
  }

  /**
   * Evaluate visual signature
   *
   * @param params - Visual signature parameters
   * @returns Visual signature evaluation
   */
  evaluateVisualSignature(params: VisualSignatureRequest): VisualSignatureResponse {
    const {
      targetSize,
      luminance,
      contrast,
      visibilityThreshold = 0.05,
      camouflageType = 'solid',
      atmosphere,
    } = params;

    // Calculate base detection range
    // Simplified model: R = k * sqrt(Size * Contrast / Threshold)
    let baseDetectionRange = 1000 * Math.sqrt((targetSize * contrast) / visibilityThreshold);

    // Apply atmospheric effects
    let atmosphericFactor = 1.0;
    if (atmosphere) {
      const visibilityKm = atmosphere.visibility;
      atmosphericFactor = Math.min(1.0, visibilityKm / 10);

      if (atmosphere.haze) {
        atmosphericFactor *= 0.7;
      }
    }

    baseDetectionRange *= atmosphericFactor;

    // Apply camouflage effectiveness
    let camouflageEffectiveness = 0;
    let detectionRange = baseDetectionRange;

    switch (camouflageType) {
      case 'solid':
        camouflageEffectiveness = 0.2;
        detectionRange *= 0.8;
        break;
      case 'disruptive':
        camouflageEffectiveness = 0.5;
        detectionRange *= 0.5;
        break;
      case 'adaptive':
        camouflageEffectiveness = 0.8;
        detectionRange *= 0.2;
        break;
      case 'digital':
        camouflageEffectiveness = 0.6;
        detectionRange *= 0.4;
        break;
      case 'multispectral':
        camouflageEffectiveness = 0.9;
        detectionRange *= 0.1;
        break;
    }

    // Recognition and identification ranges (typically fractions of detection range)
    const recognitionRange = detectionRange * 0.6;
    const identificationRange = detectionRange * 0.3;

    // Color matching quality (if adaptive camouflage)
    let colorMatching;
    if (camouflageType === 'adaptive' || camouflageType === 'multispectral') {
      const deltaE = (1 - camouflageEffectiveness) * 10; // CIE Delta E
      let match: 'excellent' | 'good' | 'fair' | 'poor';
      if (deltaE < 1) match = 'excellent';
      else if (deltaE < 2) match = 'good';
      else if (deltaE < 4) match = 'fair';
      else match = 'poor';

      colorMatching = { deltaE, match };
    }

    return {
      detectionRange,
      recognitionRange,
      identificationRange,
      camouflageEffectiveness,
      colorMatching,
    };
  }

  /**
   * Evaluate comprehensive stealth performance
   *
   * @param params - Stealth evaluation parameters
   * @returns Comprehensive stealth performance assessment
   */
  evaluateStealthPerformance(params: {
    platform: StealthPlatform;
    threatEnvironment: ThreatEnvironment;
  }): StealthPerformance {
    const { platform, threatEnvironment } = params;

    // Calculate RCS for primary threat radar band
    const primaryRadarBand = threatEnvironment.radarBands[0] || 'X-band';
    const radarFreq = STEALTH_CONSTANTS.RADAR_BANDS[primaryRadarBand].min;

    const rcs = this.calculateRCS({
      frequency: radarFreq,
      targetShape: 'faceted',
      dimensions: platform.dimensions,
      surfaceArea: platform.surfaceArea,
      ramCoating: platform.stealthFeatures.rcsReduction.ramCoating,
      incidenceAngle: 0,
    });

    // Calculate IR signature
    const irSignature = this.evaluateIRSignature({
      surfaceTemp: 320, // Estimated
      emissivity: platform.stealthFeatures.irSuppression.lowEmissivityCoating ? 0.25 : 0.85,
      surfaceArea: platform.surfaceArea,
      coolingSystem: platform.stealthFeatures.irSuppression.exhaustCooling ? 'active' : 'none',
      lowECoating: platform.stealthFeatures.irSuppression.lowEmissivityCoating,
    });

    // Calculate acoustic signature
    const acousticSignature = this.assessAcousticSignature({
      sourcePower: platform.type === 'aircraft' ? 1000 : 500,
      frequency: 500,
      distance: 100,
      dampening: platform.stealthFeatures.acousticDampening.engineInsulation ? 15 : 0,
      environment: platform.type === 'naval' ? 'water' : 'air',
    });

    // Calculate visual signature
    const visualSignature = this.evaluateVisualSignature({
      targetSize: platform.surfaceArea,
      luminance: 100,
      contrast: platform.stealthFeatures.visualCamouflage.matteFinish ? 0.2 : 0.5,
      camouflageType: platform.stealthFeatures.visualCamouflage.disruptivePattern ? 'disruptive' : 'solid',
    });

    // Calculate multi-spectrum scores (0-1, higher is better)
    const radarScore = Math.max(0, Math.min(1, (30 + rcs.dBsm) / 60)); // -30 to +30 dBsm
    const infraredScore = Math.max(0, Math.min(1, 1 - irSignature.totalPower / 10000));
    const acousticScore = Math.max(0, Math.min(1, (100 - acousticSignature.soundPressureLevel) / 50));
    const visualScore = Math.max(0, Math.min(1, visualSignature.camouflageEffectiveness));

    const combinedScore = (radarScore + infraredScore + acousticScore + visualScore) / 4;

    // Overall rating (0-10)
    const overallRating = combinedScore * 10;

    // Survivability enhancement factor
    const survivabilityFactor = 1 + (combinedScore * 9); // 1x to 10x

    // Generate recommendations
    const recommendations: string[] = [];

    if (rcs.dBsm > -20) {
      recommendations.push('Consider improving RCS through enhanced shaping or RAM coating');
    }
    if (irSignature.totalPower > 2000) {
      recommendations.push('Implement active cooling system to reduce IR signature');
    }
    if (acousticSignature.soundPressureLevel > 70) {
      recommendations.push('Add acoustic dampening to engine and airframe');
    }
    if (visualSignature.camouflageEffectiveness < 0.5) {
      recommendations.push('Upgrade to adaptive or multispectral camouflage system');
    }

    return {
      platform,
      rcs,
      irSignature,
      acousticSignature,
      visualSignature,
      overallRating,
      multiSpectrumScore: {
        radar: radarScore,
        infrared: infraredScore,
        acoustic: acousticScore,
        visual: visualScore,
        combined: combinedScore,
      },
      survivabilityFactor,
      recommendations,
    };
  }

  /**
   * Optimize stealth design for threat environment
   *
   * @param params - Optimization parameters
   * @returns Optimization result
   */
  optimizeStealthDesign(params: OptimizationParams): OptimizationResult {
    const { platform, threatEnvironment, priorities } = params;

    // Calculate baseline performance
    const baseline = this.evaluateStealthPerformance({
      platform,
      threatEnvironment,
    });

    // Create optimized platform with enhanced features
    const optimizedPlatform: StealthPlatform = {
      ...platform,
      stealthFeatures: {
        rcsReduction: {
          shaping: true,
          ramCoating: true,
          edgeTreatment: 'serrated',
          internalWeapons: true,
        },
        irSuppression: {
          exhaustCooling: true,
          lowEmissivityCoating: true,
          serpentineDucts: true,
        },
        acousticDampening: {
          engineInsulation: true,
          airframeTreatment: true,
          activeNoiseCancellation: true,
        },
        visualCamouflage: {
          matteFinish: true,
          disruptivePattern: true,
          adaptiveDisplays: true,
        },
      },
    };

    // Calculate optimized performance
    const optimized = this.evaluateStealthPerformance({
      platform: optimizedPlatform,
      threatEnvironment,
    });

    // Calculate improvements
    const improvements = {
      rcsReduction: baseline.rcs.dBsm - optimized.rcs.dBsm,
      irReduction: ((baseline.irSignature.totalPower - optimized.irSignature.totalPower) / baseline.irSignature.totalPower) * 100,
      acousticReduction: baseline.acousticSignature.soundPressureLevel - optimized.acousticSignature.soundPressureLevel,
      visualImprovement: ((optimized.visualSignature.camouflageEffectiveness - baseline.visualSignature.camouflageEffectiveness) / baseline.visualSignature.camouflageEffectiveness) * 100,
    };

    // Generate modification recommendations
    const modifications: string[] = [
      'Apply advanced RAM coating for 10-15 dB RCS reduction',
      'Install active IR suppression system',
      'Implement serpentine exhaust ducts',
      'Add multi-layer acoustic insulation',
      'Integrate adaptive camouflage system',
      'Align all edges to minimize RCS spikes',
      'Use internal weapons carriage',
    ];

    return {
      baseline,
      optimized,
      improvements,
      modifications,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate RCS (standalone function)
 */
export function calculateRCS(params: RCSRequest): RCSResponse {
  const sdk = new StealthTechSDK();
  return sdk.calculateRCS(params);
}

/**
 * Evaluate IR signature (standalone function)
 */
export function evaluateIRSignature(params: IRSignatureRequest): IRSignatureResponse {
  const sdk = new StealthTechSDK();
  return sdk.evaluateIRSignature(params);
}

/**
 * Assess acoustic signature (standalone function)
 */
export function assessAcousticSignature(params: AcousticSignatureRequest): AcousticSignatureResponse {
  const sdk = new StealthTechSDK();
  return sdk.assessAcousticSignature(params);
}

/**
 * Evaluate visual signature (standalone function)
 */
export function evaluateVisualSignature(params: VisualSignatureRequest): VisualSignatureResponse {
  const sdk = new StealthTechSDK();
  return sdk.evaluateVisualSignature(params);
}

/**
 * Evaluate stealth performance (standalone function)
 */
export function evaluateStealthPerformance(params: {
  platform: StealthPlatform;
  threatEnvironment: ThreatEnvironment;
}): StealthPerformance {
  const sdk = new StealthTechSDK();
  return sdk.evaluateStealthPerformance(params);
}

/**
 * Optimize stealth design (standalone function)
 */
export function optimizeStealthDesign(params: OptimizationParams): OptimizationResult {
  const sdk = new StealthTechSDK();
  return sdk.optimizeStealthDesign(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { StealthTechSDK };
export default StealthTechSDK;
