/**
 * WIA-SEMI-010 MicroLED Standard - TypeScript SDK
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

export * from './types';

import {
  ChipSpecification,
  ChipSize,
  LEDColor,
  TransferSpecification,
  DisplaySpecification,
  QualityMetrics,
  DefectMap,
  CalibrationData,
} from './types';

/**
 * MicroLED Calculator - Utility functions for MicroLED display calculations
 */
export class MicroLEDCalculator {
  /**
   * Calculate total number of LED chips required for a display
   */
  static calculateTotalChips(
    widthPixels: number,
    heightPixels: number,
    subpixelsPerPixel: number = 3
  ): number {
    return widthPixels * heightPixels * subpixelsPerPixel;
  }

  /**
   * Calculate chips per wafer
   */
  static calculateChipsPerWafer(
    waferDiameter: number, // inches
    chipSize: number, // micrometers
    efficiency: number = 0.7 // usable area efficiency
  ): number {
    const waferRadiusMicrons = (waferDiameter * 25.4 * 1000) / 2;
    const waferArea = Math.PI * waferRadiusMicrons * waferRadiusMicrons;
    const chipArea = chipSize * chipSize;
    return Math.floor((waferArea / chipArea) * efficiency);
  }

  /**
   * Calculate transfer yield required for target final yield
   */
  static calculateRequiredTransferYield(
    targetFinalYield: number,
    repairRate: number = 0.95
  ): number {
    // Simplified model: accounts for repair capability
    const defectBudget = 1 - targetFinalYield;
    const unrepairedDefects = defectBudget / repairRate;
    return 1 - unrepairedDefects;
  }

  /**
   * Calculate display brightness given chip parameters
   */
  static calculateDisplayBrightness(
    chipLuminousIntensity: number, // millicandelas
    fillFactor: number, // 0.0 - 1.0
    pixelPitch: number // millimeters
  ): number {
    // Simplified calculation
    const pixelArea = pixelPitch * pixelPitch; // mm²
    const areaPerCd = pixelArea / (chipLuminousIntensity / 1000);
    const nits = (1 / (areaPerCd * Math.PI)) * 1000000; // cd/m²
    return nits * fillFactor;
  }

  /**
   * Calculate power consumption
   */
  static calculatePowerConsumption(
    totalChips: number,
    forwardVoltage: number, // volts
    current: number, // milliamperes
    efficiency: number = 0.4 // 40% efficiency typical
  ): number {
    const powerPerChip = forwardVoltage * (current / 1000); // watts
    const totalPower = powerPerChip * totalChips;
    return totalPower / efficiency; // account for driver and system losses
  }
}

/**
 * Defect Analysis - Tools for analyzing defect data
 */
export class DefectAnalyzer {
  /**
   * Classify display quality based on defect count
   */
  static classifyQuality(defectMap: DefectMap): 'I' | 'II' | 'III' | 'IV' {
    const deadPPM = (defectMap.classification.deadPixels / defectMap.totalPixels) * 1_000_000;
    const brightPPM = (defectMap.classification.brightPixels / defectMap.totalPixels) * 1_000_000;

    if (deadPPM === 0 && brightPPM === 0) return 'I';
    if (deadPPM <= 2 && brightPPM <= 2) return 'II';
    if (deadPPM <= 5 && brightPPM <= 15) return 'III';
    return 'IV';
  }

  /**
   * Calculate defect rate in PPM
   */
  static calculateDefectPPM(defectCount: number, totalPixels: number): number {
    return (defectCount / totalPixels) * 1_000_000;
  }

  /**
   * Analyze defect clustering
   */
  static analyzeDefectClustering(defectMap: DefectMap, clusterRadius: number = 15): number {
    // Simplified clustering analysis
    // Returns number of cluster violations (>2 defects within radius)
    let violations = 0;
    const defects = defectMap.defectivePixels;

    for (let i = 0; i < defects.length; i++) {
      let nearbyDefects = 0;
      for (let j = 0; j < defects.length; j++) {
        if (i === j) continue;
        const distance = Math.sqrt(
          Math.pow(defects[i].address.row - defects[j].address.row, 2) +
          Math.pow(defects[i].address.column - defects[j].address.column, 2)
        );
        if (distance <= clusterRadius) nearbyDefects++;
      }
      if (nearbyDefects >= 2) violations++;
    }

    return violations;
  }
}

/**
 * Calibration Tools - Utilities for display calibration
 */
export class CalibrationTools {
  /**
   * Generate gamma lookup table
   */
  static generateGammaLUT(targetGamma: number, bitDepth: number = 8): number[] {
    const maxValue = Math.pow(2, bitDepth) - 1;
    const lut: number[] = [];

    for (let i = 0; i <= maxValue; i++) {
      const normalized = i / maxValue;
      const corrected = Math.pow(normalized, targetGamma);
      lut.push(Math.round(corrected * maxValue));
    }

    return lut;
  }

  /**
   * Calculate color uniformity (Delta E)
   */
  static calculateDeltaE(
    L1: number, a1: number, b1: number,
    L2: number, a2: number, b2: number
  ): number {
    // CIE76 Delta E (simplified)
    return Math.sqrt(
      Math.pow(L2 - L1, 2) +
      Math.pow(a2 - a1, 2) +
      Math.pow(b2 - b1, 2)
    );
  }

  /**
   * Calculate brightness uniformity percentage
   */
  static calculateBrightnessUniformity(measurements: number[]): number {
    const mean = measurements.reduce((a, b) => a + b, 0) / measurements.length;
    const deviations = measurements.map(m => Math.abs(m - mean));
    const maxDeviation = Math.max(...deviations);
    return (maxDeviation / mean) * 100;
  }
}

/**
 * Validator - Validate specifications against WIA-SEMI-010 standard
 */
export class SpecificationValidator {
  /**
   * Validate chip specification
   */
  static validateChipSpec(spec: ChipSpecification): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    // Size validation
    const size = spec.dimensions.width;
    if (spec.size === ChipSize.ULTRA_MICRO && size >= 10) {
      errors.push('Ultra-micro chips must be <10μm');
    }
    if (spec.size === ChipSize.SMALL && (size < 10 || size > 30)) {
      errors.push('Small chips must be 10-30μm');
    }

    // Wavelength validation
    if (spec.color === LEDColor.BLUE) {
      if (spec.opticalProperties.peakWavelength < 450 || spec.opticalProperties.peakWavelength > 470) {
        errors.push('Blue LED wavelength must be 450-470nm');
      }
    }

    // EQE validation
    if (spec.color === LEDColor.BLUE && spec.opticalProperties.externalQuantumEfficiency < 55) {
      errors.push('Blue LED EQE should be >55%');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Validate transfer yield requirement
   */
  static validateTransferYield(spec: TransferSpecification): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (spec.yieldRequirement < 99.9) {
      errors.push('Transfer yield should be ≥99.9% for production');
    }

    if (spec.placementAccuracy.position > 2) {
      errors.push('Placement accuracy should be ≤±2μm');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Validate quality metrics
   */
  static validateQualityMetrics(metrics: QualityMetrics): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (metrics.defectRate.total > 1000) {
      errors.push('Total defect rate exceeds 1000 ppm limit for standard displays');
    }

    if (metrics.uniformity.brightness > 5) {
      errors.push('Brightness uniformity should be <5%');
    }

    if (metrics.uniformity.color > 2) {
      errors.push('Color uniformity should be ΔE<2');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }
}

/**
 * Cost Estimator - Estimate manufacturing costs
 */
export class CostEstimator {
  /**
   * Estimate chip fabrication cost
   */
  static estimateChipCost(
    waferCost: number,
    chipsPerWafer: number,
    yield: number
  ): number {
    return (waferCost / chipsPerWafer) / yield;
  }

  /**
   * Estimate transfer cost
   */
  static estimateTransferCost(
    totalChips: number,
    costPerChip: number,
    yieldLoss: number
  ): number {
    const wastedChips = totalChips * yieldLoss;
    return (totalChips + wastedChips) * costPerChip;
  }

  /**
   * Estimate display manufacturing cost
   */
  static estimateDisplayCost(
    chipCost: number,
    transferCost: number,
    tftCost: number,
    assemblyCost: number,
    overheadMultiplier: number = 1.3
  ): number {
    const directCost = chipCost + transferCost + tftCost + assemblyCost;
    return directCost * overheadMultiplier;
  }
}

// Version and metadata
export const SDK_VERSION = '1.0.0';
export const STANDARD_VERSION = 'WIA-SEMI-010 v1.0';
export const AUTHOR = 'WIA (World Certification Industry Association)';
export const LICENSE = 'MIT';
