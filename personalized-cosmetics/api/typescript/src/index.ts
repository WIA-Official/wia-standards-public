/**
 * WIA-IND-006: Personalized Cosmetics Standard - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry & Biotech Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// Export all types
export * from './types';

// SDK version
export const VERSION = '1.0.0';
export const STANDARD_ID = 'WIA-IND-006';
export const STANDARD_NAME = 'Personalized Cosmetics';

/**
 * Calculate overall skin health score
 *
 * Score = (H × 0.25) + (E × 0.25) + (O × 0.20) + (S × 0.15) + (P × 0.15)
 *
 * @param hydration - Hydration level (0-100)
 * @param elasticity - Elasticity score (0-100)
 * @param oilBalance - Oil balance (0-100, 50 = optimal)
 * @param sensitivity - Sensitivity index (0-100, lower is better)
 * @param pigmentation - Pigmentation uniformity (0-100)
 * @returns Skin health score (0-100)
 */
export function calculateSkinHealthScore(
  hydration: number,
  elasticity: number,
  oilBalance: number,
  sensitivity: number,
  pigmentation: number
): number {
  // Oil balance deviation from optimal (50)
  const oilDeviation = Math.abs(oilBalance - 50);
  const oilScore = Math.max(0, 100 - oilDeviation * 2);

  // Sensitivity is inverse (lower is better)
  const sensitivityScore = 100 - sensitivity;

  // Weighted score
  const score =
    hydration * 0.25 +
    elasticity * 0.25 +
    oilScore * 0.2 +
    sensitivityScore * 0.15 +
    pigmentation * 0.15;

  return Math.round(score * 10) / 10;
}

/**
 * Calculate adjusted ingredient concentration
 *
 * Adjusted = Base × (1 + Skin Factor × Max Adjustment)
 *
 * @param baseConcentration - Base concentration (%)
 * @param skinFactor - Skin factor (-1.0 to +1.0)
 * @param maxAdjustment - Maximum adjustment (default: 0.3)
 * @returns Adjusted concentration (%)
 */
export function calculateIngredientConcentration(
  baseConcentration: number,
  skinFactor: number,
  maxAdjustment: number = 0.3
): number {
  const adjustment = 1 + skinFactor * maxAdjustment;
  return Math.round(baseConcentration * adjustment * 1000) / 1000;
}

/**
 * Calculate biological skin age from genetics
 *
 * Biological Age = Chronological + (Genetic Risk × 5) - (Care Score × 0.3)
 *
 * @param chronologicalAge - Actual age in years
 * @param geneticRisk - Genetic risk score (0-10)
 * @param careScore - Skincare quality score (0-100)
 * @returns Biological skin age in years
 */
export function calculateGeneticSkinAge(
  chronologicalAge: number,
  geneticRisk: number,
  careScore: number
): number {
  const geneticImpact = geneticRisk * 5;
  const careImpact = careScore * 0.3;
  const biologicalAge = chronologicalAge + geneticImpact - careImpact;

  return Math.max(0, Math.round(biologicalAge * 10) / 10);
}

/**
 * Calculate product efficacy score
 *
 * Efficacy = (Target Achievement / Baseline) × 100 × Compliance Factor
 *
 * @param targetAchievement - Measured improvement
 * @param baselineImprovement - Expected baseline improvement
 * @param complianceRate - Usage compliance (0-100%)
 * @returns Efficacy score (%)
 */
export function calculateEfficacy(
  targetAchievement: number,
  baselineImprovement: number,
  complianceRate: number
): number {
  const complianceFactor = complianceRate / 100;
  const efficacy = (targetAchievement / baselineImprovement) * 100 * complianceFactor;

  return Math.round(efficacy * 10) / 10;
}

/**
 * Calculate ingredient compatibility score
 *
 * Compatibility = 100 - (Interaction + Sensitivity + Stability)
 *
 * @param interactionPenalty - Interaction penalty (0-100)
 * @param sensitivityRisk - Sensitivity risk (0-100)
 * @param stabilityIssues - Stability issues (0-100)
 * @returns Compatibility score (0-100)
 */
export function calculateCompatibility(
  interactionPenalty: number,
  sensitivityRisk: number,
  stabilityIssues: number
): number {
  const totalPenalty = interactionPenalty + sensitivityRisk + stabilityIssues;
  return Math.max(0, 100 - totalPenalty);
}

/**
 * Determine Baumann skin type from measurements
 *
 * @param sebumLevel - Sebum level (μg/cm²)
 * @param erythemaIndex - Erythema index
 * @param pigmentationRisk - Pigmentation risk score
 * @param elasticityScore - Elasticity score
 * @returns Baumann type string (e.g., "DSPT")
 */
export function determineBaumannType(
  sebumLevel: number,
  erythemaIndex: number,
  pigmentationRisk: number,
  elasticityScore: number
): string {
  const oilType = sebumLevel > 150 ? 'O' : 'D';
  const sensType = erythemaIndex > 250 ? 'S' : 'R';
  const pigType = pigmentationRisk > 75 ? 'P' : 'N';
  const wrinkleType = elasticityScore < 70 ? 'W' : 'T';

  return `${oilType}${sensType}${pigType}${wrinkleType}`;
}

/**
 * Calculate skin factor modifiers from measurements
 *
 * @param measurements - Instrumental measurements
 * @returns Skin factor modifiers object
 */
export function calculateSkinFactors(measurements: {
  hydration: number;
  sebumLevel: number;
  erythemaIndex: number;
  elasticity: number;
  melaninUniformity: number;
}): {
  hydrationFactor: number;
  oilBalanceFactor: number;
  sensitivityFactor: number;
  agingFactor: number;
  pigmentationFactor: number;
} {
  const hydrationFactor = (50 - measurements.hydration) / 50;
  const oilBalanceFactor = (measurements.sebumLevel - 100) / 100;
  const sensitivityFactor = (measurements.erythemaIndex - 250) / 250;
  const agingFactor = (100 - measurements.elasticity) / 100;
  const pigmentationFactor = (measurements.melaninUniformity - 75) / 25;

  return {
    hydrationFactor: Math.max(-1, Math.min(1, hydrationFactor)),
    oilBalanceFactor: Math.max(-1, Math.min(1, oilBalanceFactor)),
    sensitivityFactor: Math.max(-1, Math.min(1, sensitivityFactor)),
    agingFactor: Math.max(0, Math.min(1, agingFactor)),
    pigmentationFactor: Math.max(-1, Math.min(1, pigmentationFactor)),
  };
}

/**
 * PersonalizedCosmeticsSDK class for advanced functionality
 */
export class PersonalizedCosmeticsSDK {
  private apiKey?: string;
  private apiEndpoint?: string;

  constructor(config?: { apiKey?: string; apiEndpoint?: string }) {
    this.apiKey = config?.apiKey;
    this.apiEndpoint = config?.apiEndpoint || 'https://api.wiastandards.com/ind-006';
  }

  /**
   * Analyze skin profile from provided data
   */
  async analyzeSkinProfile(data: any): Promise<any> {
    // Implementation would make API call to analyze skin
    // For now, return placeholder
    return {
      success: true,
      message: 'Skin analysis complete',
      skinProfileID: `PROFILE-${Date.now()}`,
    };
  }

  /**
   * Generate custom formulation
   */
  async generateFormulation(data: any): Promise<any> {
    // Implementation would make API call to generate formulation
    return {
      success: true,
      message: 'Formulation generated',
      formulationID: `FORM-${Date.now()}`,
    };
  }

  /**
   * Track efficacy over time
   */
  async trackEfficacy(data: any): Promise<any> {
    // Implementation would make API call to track efficacy
    return {
      success: true,
      message: 'Efficacy tracked',
    };
  }
}

// Default export
export default {
  VERSION,
  STANDARD_ID,
  STANDARD_NAME,
  calculateSkinHealthScore,
  calculateIngredientConcentration,
  calculateGeneticSkinAge,
  calculateEfficacy,
  calculateCompatibility,
  determineBaumannType,
  calculateSkinFactors,
  PersonalizedCosmeticsSDK,
};
