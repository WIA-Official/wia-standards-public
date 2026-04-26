/**
 * WIA-SEMI-011 Flexible Display SDK - Type Definitions
 * © 2025 World Certification Industry Association
 */

export type DisplayType = 'foldable' | 'rollable' | 'stretchable';
export type FoldConfiguration = 'in-fold' | 'out-fold' | 'clamshell';
export type SubstrateType = 'utg' | 'polyimide' | 'pet' | 'pdms' | 'tpu';
export type HingeType = 'multi-cam' | 'waterdrop' | 'teardrop' | '360-degree';

export interface FlexibleDisplaySpecs {
  type: DisplayType;
  bendRadius: number; // mm
  foldCycles: number;
  creaseDepth: number; // mm
  screenSize: number; // inches
  resolution: {
    width: number;
    height: number;
  };
  brightness: number; // cd/m²
  refreshRate: number; // Hz
}

export interface FoldableDisplaySpecs extends FlexibleDisplaySpecs {
  type: 'foldable';
  foldConfiguration: FoldConfiguration;
  hingeType: HingeType;
  hingeTorque: number; // N·m
  gapWhenClosed: number; // mm
}

export interface RollableDisplaySpecs extends FlexibleDisplaySpecs {
  type: 'rollable';
  rollerDiameter: number; // mm
  extensionRatio: number; // e.g., 2.0 for 2x expansion
  rollSpeed: number; // mm/s
  motorPower: number; // W
}

export interface StretchableDisplaySpecs extends FlexibleDisplaySpecs {
  type: 'stretchable';
  maxStrain: number; // percentage
  stretchCycles: number;
  recovery: number; // percentage
  elasticModulus: number; // MPa
}

export interface MaterialProperties {
  substrate: SubstrateType;
  thickness: number; // μm
  youngsModulus: number; // GPa
  thermalExpansion: number; // ppm/°C
  transparency: number; // percentage
}

export interface TestResult {
  testName: string;
  passed: boolean;
  cycles: number;
  degradation: number; // percentage
  notes: string;
}

export interface CertificationTier {
  tier: 'standard' | 'premium' | 'ultra';
  minFoldCycles: number;
  creaseLimit: number; // mm
  opticalRequirements: {
    luminance: number;
    uniformity: number;
    colorGamut: string;
  };
}

export class FlexibleDisplay {
  constructor(public specs: FlexibleDisplaySpecs) {}

  calculateBendStrain(neutralPlaneOffset: number): number {
    return (this.specs.bendRadius > 0)
      ? (neutralPlaneOffset / this.specs.bendRadius) * 100
      : 0;
  }

  estimateLifespan(dailyFolds: number): number {
    return Math.floor(this.specs.foldCycles / dailyFolds);
  }

  meetsWIAStandard(): boolean {
    return this.specs.bendRadius < 3 &&
           this.specs.foldCycles >= 200000 &&
           this.specs.creaseDepth < 0.5;
  }
}

export class BendRadiusCalculator {
  static calculateMinRadius(
    substrate: SubstrateType,
    thickness: number,
    modulusGPa: number
  ): number {
    const modulusMap: Record<SubstrateType, number> = {
      utg: 70,
      polyimide: 3.5,
      pet: 4.0,
      pdms: 0.001,
      tpu: 0.01
    };

    const modulus = modulusMap[substrate] || modulusGPa;
    return (thickness / 1000) * modulus * 1.5; // mm
  }

  static calculateStrain(thickness: number, radius: number): number {
    return (thickness / (2 * radius)) * 100; // percentage
  }
}

export class DurabilityCalculator {
  static estimateFoldCycles(
    baseCycles: number,
    degradationFactor: number,
    tempFactor: number,
    humidityFactor: number
  ): number {
    return Math.floor(baseCycles * degradationFactor * tempFactor * humidityFactor);
  }

  static calculateLifespanYears(
    totalCycles: number,
    dailyFolds: number
  ): number {
    const days = totalCycles / dailyFolds;
    return days / 365;
  }
}

export class HingeAnalyzer {
  static analyzeStress(
    hingeType: HingeType,
    foldAngle: number,
    torque: number,
    gap: number
  ): {
    creaseFactor: number;
    stressLevel: number;
    compliance: 'PASS' | 'WARNING' | 'FAIL';
  } {
    const creaseFactor = gap * 5;
    const stressLevel = torque / 100;

    const compliance = (foldAngle >= 180 && torque <= 200 && gap <= 0.2)
      ? 'PASS'
      : (foldAngle >= 150 && torque <= 250)
      ? 'WARNING'
      : 'FAIL';

    return { creaseFactor, stressLevel, compliance };
  }
}

export class CertificationValidator {
  static validateStandard(specs: FlexibleDisplaySpecs): {
    tier: CertificationTier['tier'] | null;
    passed: boolean;
    issues: string[];
  } {
    const issues: string[] = [];

    if (specs.bendRadius >= 3) {
      issues.push('Bend radius exceeds 3mm limit');
    }

    if (specs.foldCycles < 200000) {
      issues.push('Fold cycles below 200,000 minimum');
    }

    if (specs.creaseDepth >= 0.5) {
      issues.push('Crease depth exceeds 0.5mm limit');
    }

    const passed = issues.length === 0;
    let tier: CertificationTier['tier'] | null = null;

    if (passed) {
      if (specs.foldCycles >= 500000) {
        tier = 'ultra';
      } else if (specs.foldCycles >= 300000) {
        tier = 'premium';
      } else {
        tier = 'standard';
      }
    }

    return { tier, passed, issues };
  }
}
