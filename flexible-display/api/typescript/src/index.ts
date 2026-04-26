/**
 * WIA-SEMI-011 Flexible Display SDK
 * © 2025 World Certification Industry Association
 *
 * TypeScript SDK for flexible display calculations and validation
 */

export * from './types';

export {
  FlexibleDisplay,
  BendRadiusCalculator,
  DurabilityCalculator,
  HingeAnalyzer,
  CertificationValidator
} from './types';

// Re-export types for convenience
export type {
  DisplayType,
  FoldConfiguration,
  SubstrateType,
  HingeType,
  FlexibleDisplaySpecs,
  FoldableDisplaySpecs,
  RollableDisplaySpecs,
  StretchableDisplaySpecs,
  MaterialProperties,
  TestResult,
  CertificationTier
} from './types';

/**
 * WIA-SEMI-011 Standard Constants
 */
export const WIA_CONSTANTS = {
  MIN_BEND_RADIUS: 3, // mm
  MIN_FOLD_CYCLES: 200000,
  MAX_CREASE_DEPTH: 0.5, // mm
  MIN_BRIGHTNESS: 400, // cd/m²
  MIN_UNIFORMITY: 85, // percentage
  MIN_RESPONSE_TIME: 10, // ms
  TEMP_RANGE: {
    min: -10, // °C
    max: 50   // °C
  },
  HUMIDITY_RANGE: {
    min: 10,  // %RH
    max: 90   // %RH
  }
} as const;

/**
 * Helper function to create a flexible display instance
 */
export function createFlexibleDisplay(specs: any) {
  return new FlexibleDisplay(specs);
}

/**
 * Validate if a display meets WIA-SEMI-011 standards
 */
export function validateDisplay(specs: any): boolean {
  const display = new FlexibleDisplay(specs);
  return display.meetsWIAStandard();
}

/**
 * Calculate expected lifespan in years
 */
export function calculateLifespan(
  foldCycles: number,
  dailyUsage: number
): number {
  return DurabilityCalculator.calculateLifespanYears(foldCycles, dailyUsage);
}

/**
 * Get certification tier for given specifications
 */
export function getCertificationTier(specs: any): string | null {
  const result = CertificationValidator.validateStandard(specs);
  return result.tier;
}
