/**
 * WIA-SEMI-009: OLED/LED Display Standards SDK
 * 
 * @packageDocumentation
 */

export * from './types';
export * from './calculations';
export * from './validators';
export * from './standards';

import { DisplaySpecification, CertificationLevel, CertificationResult } from './types';
import { validateDisplay } from './validators';
import { getStandardRequirements } from './standards';

/**
 * Main SDK class for WIA-SEMI-009
 */
export class WIASEMI009 {
  /**
   * Validate a display specification against WIA-SEMI-009 standards
   * 
   * @param spec - Display specification to validate
   * @param targetLevel - Target certification level (default: STANDARD)
   * @returns Certification result
   */
  static validate(
    spec: DisplaySpecification,
    targetLevel: CertificationLevel = CertificationLevel.STANDARD
  ): CertificationResult {
    return validateDisplay(spec, targetLevel);
  }

  /**
   * Get standard requirements for a specific application
   * 
   * @param application - Application category
   * @param level - Certification level
   * @returns Standard requirements
   */
  static getRequirements(
    application: string,
    level: CertificationLevel = CertificationLevel.STANDARD
  ) {
    return getStandardRequirements(application, level);
  }

  /**
   * Get SDK version
   */
  static getVersion(): string {
    return '1.0.0';
  }
}

/**
 * Default export
 */
export default WIASEMI009;
