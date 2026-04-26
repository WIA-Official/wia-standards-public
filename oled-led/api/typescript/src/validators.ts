/**
 * Validation functions for WIA-SEMI-009
 */

import { DisplaySpecification, CertificationLevel, CertificationResult, TestResult, ApplicationCategory } from './types';
import { getStandardRequirements } from './standards';

/**
 * Validate a display specification
 */
export function validateDisplay(
  spec: DisplaySpecification,
  targetLevel: CertificationLevel
): CertificationResult {
  const tests: TestResult[] = [];
  const requirements = getStandardRequirements(spec.application, targetLevel);
  
  // Luminance tests
  if (requirements.minFullScreen) {
    tests.push({
      testName: 'Full-Screen Luminance',
      passed: spec.luminance.fullScreen >= requirements.minFullScreen,
      measuredValue: spec.luminance.fullScreen,
      specLimit: requirements.minFullScreen,
      unit: 'Cd/m²',
    });
  }
  
  if (requirements.minPeak) {
    tests.push({
      testName: 'Peak Brightness (10%)',
      passed: spec.luminance.peak10Percent >= requirements.minPeak,
      measuredValue: spec.luminance.peak10Percent,
      specLimit: requirements.minPeak,
      unit: 'Cd/m²',
    });
  }
  
  // Contrast test
  if (requirements.minContrast) {
    tests.push({
      testName: 'ANSI Contrast',
      passed: spec.contrast.ansi >= requirements.minContrast,
      measuredValue: spec.contrast.ansi,
      specLimit: requirements.minContrast,
      unit: ':1',
    });
  }
  
  // Color gamut test
  if (requirements.minColorGamut) {
    const gamutValue = spec.color.gamut['DCI-P3'] || 0;
    tests.push({
      testName: 'Color Gamut (DCI-P3)',
      passed: gamutValue >= requirements.minColorGamut,
      measuredValue: gamutValue,
      specLimit: requirements.minColorGamut,
      unit: '%',
    });
  }
  
  // Color accuracy test
  if (requirements.maxDeltaE) {
    tests.push({
      testName: 'Color Accuracy (ΔE)',
      passed: spec.color.accuracy.average <= requirements.maxDeltaE,
      measuredValue: spec.color.accuracy.average,
      specLimit: requirements.maxDeltaE,
      unit: 'ΔE2000',
    });
  }
  
  // Response time test
  if (requirements.maxResponseTime) {
    tests.push({
      testName: 'Response Time',
      passed: spec.responseTime.grayToGray <= requirements.maxResponseTime,
      measuredValue: spec.responseTime.grayToGray,
      specLimit: requirements.maxResponseTime,
      unit: 'ms',
    });
  }
  
  // OLED lifetime test
  if (spec.oledLifetime && requirements.minLifetime) {
    tests.push({
      testName: 'OLED Lifetime (LT95)',
      passed: spec.oledLifetime.lt95 >= requirements.minLifetime,
      measuredValue: spec.oledLifetime.lt95,
      specLimit: requirements.minLifetime,
      unit: 'hours',
    });
  }
  
  // Determine if all tests passed
  const allPassed = tests.every(test => test.passed);
  
  return {
    level: targetLevel,
    certified: allPassed,
    tests,
    date: new Date(),
    certificateNumber: allPassed ? generateCertificateNumber() : undefined,
  };
}

/**
 * Generate a certificate number
 */
function generateCertificateNumber(): string {
  const date = new Date();
  const year = date.getFullYear();
  const month = String(date.getMonth() + 1).padStart(2, '0');
  const random = Math.floor(Math.random() * 100000).toString().padStart(5, '0');
  return `WIA-SEMI-009-${year}${month}-${random}`;
}

/**
 * Validate luminance specification
 */
export function validateLuminance(spec: any): boolean {
  return (
    typeof spec.fullScreen === 'number' && spec.fullScreen > 0 &&
    typeof spec.peak10Percent === 'number' && spec.peak10Percent > 0 &&
    typeof spec.blackLevel === 'number' && spec.blackLevel >= 0 &&
    spec.peak10Percent >= spec.fullScreen &&
    spec.fullScreen > spec.blackLevel
  );
}

/**
 * Validate color performance specification
 */
export function validateColorPerformance(spec: any): boolean {
  return (
    spec.gamut && typeof spec.gamut === 'object' &&
    spec.accuracy && typeof spec.accuracy.average === 'number' &&
    spec.accuracy.average >= 0 &&
    typeof spec.accuracy.maximum === 'number' &&
    spec.accuracy.maximum >= spec.accuracy.average
  );
}
