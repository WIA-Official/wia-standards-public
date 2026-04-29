/**
 * WIA-TIME-030: Time Travel Ethics SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Ethics Committee
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for ethical compliance in time travel operations:
 * - Operation validation and ethics checking
 * - Interference level assessment
 * - Observer protocol management
 * - Violation tracking and reporting
 * - Compliance monitoring
 */

import {
  EthicalPrinciple,
  InterferenceLevel,
  OperationPurpose,
  ProtectionLevel,
  HistoricalSignificance,
  ProhibitedActionCategory,
  ReviewType,
  ReviewDecision,
  ObserverStatus,
  ViolationSeverity,
  CertificationLevel,
  TemporalOperationRequest,
  ProtectedPeriod,
  ProtectedEvent,
  ProhibitedAction,
  EthicalReview,
  EthicsValidationResult,
  ObserverProtocol,
  InterferenceAssessment,
  InterferenceResult,
  ViolationRecord,
  ComplianceReport,
  TrainingCertification,
  TimelineIntegrityCheck,
  EmergencyIntervention,
  ETHICS_CONSTANTS,
  EthicsErrorCode,
  TimeTravelEthicsError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-030 Time Travel Ethics SDK
 */
export class TimeTravelEthicsSDK {
  private version = '1.0.0';
  private initialized = false;
  private protectedEvents: Map<string, ProtectedEvent> = new Map();
  private protectedPeriods: Map<string, ProtectedPeriod> = new Map();
  private prohibitedActions: Map<string, ProhibitedAction> = new Map();

  constructor() {
    this.initialized = true;
    this.loadProtectedData();
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Load protected events, periods, and prohibited actions
   */
  private loadProtectedData(): void {
    // Load critical protected events
    this.addProtectedEvent({
      id: 'wwii-end',
      name: 'End of World War II',
      date: new Date('1945-05-08'),
      location: { lat: 52.5200, lon: 13.4050, description: 'Berlin, Germany' },
      protectionLevel: ProtectionLevel.ABSOLUTE,
      significance: HistoricalSignificance.CRITICAL,
      category: 'war',
      affectedPopulation: 2000000000,
      protectionReason: 'Major historical event affecting global timeline',
    });

    this.addProtectedEvent({
      id: 'moon-landing',
      name: 'Apollo 11 Moon Landing',
      date: new Date('1969-07-20'),
      location: { lat: 0.6875, lon: 23.4333, description: 'Sea of Tranquility, Moon' },
      protectionLevel: ProtectionLevel.ENHANCED,
      significance: HistoricalSignificance.CRITICAL,
      category: 'scientific',
      protectionReason: 'Landmark scientific achievement',
    });

    // Load protected periods
    this.addProtectedPeriod({
      id: 'recent-history',
      name: 'Recent History',
      startDate: new Date(Date.now() - ETHICS_CONSTANTS.RECENT_HISTORY_YEARS * 365.25 * 24 * 3600 * 1000),
      endDate: new Date(),
      protectionLevel: ProtectionLevel.ENHANCED,
      reason: 'Living memory and contemporary events require enhanced protection',
      significance: HistoricalSignificance.HIGH,
      geographicScope: { regions: [], global: true },
    });

    // Load prohibited actions
    this.addProhibitedAction({
      id: 'stock-trading',
      category: ProhibitedActionCategory.FINANCIAL_EXPLOITATION,
      description: 'Trading stocks or securities using future knowledge',
      examples: [
        'Buying stocks before major announcements',
        'Short-selling before crashes',
        'Using future financial data for trading',
      ],
      severity: ViolationSeverity.CRITICAL,
    });

    this.addProhibitedAction({
      id: 'assassination-prevention',
      category: ProhibitedActionCategory.HISTORICAL_MANIPULATION,
      description: 'Preventing or causing assassination of historical figures',
      examples: [
        'Preventing Lincoln assassination',
        'Preventing JFK assassination',
        'Altering outcomes of political violence',
      ],
      severity: ViolationSeverity.CATASTROPHIC,
    });
  }

  /**
   * Add protected event to database
   */
  addProtectedEvent(event: ProtectedEvent): void {
    this.protectedEvents.set(event.id, event);
  }

  /**
   * Add protected period to database
   */
  addProtectedPeriod(period: ProtectedPeriod): void {
    this.protectedPeriods.set(period.id, period);
  }

  /**
   * Add prohibited action to database
   */
  addProhibitedAction(action: ProhibitedAction): void {
    this.prohibitedActions.set(action.id, action);
  }

  // ============================================================================
  // Operation Validation
  // ============================================================================

  /**
   * Validate temporal operation for ethical compliance
   *
   * @param operation - Operation request to validate
   * @returns Validation result with approval status and conditions
   */
  async validateOperation(
    operation: TemporalOperationRequest
  ): Promise<EthicsValidationResult> {
    const violations: string[] = [];
    const warnings: string[] = [];
    const conditions: string[] = [];
    const monitoring: string[] = [];
    const prohibitedActionsList: ProhibitedAction[] = [];

    // Check target date
    const targetDate = new Date(operation.targetDate);
    if (isNaN(targetDate.getTime())) {
      violations.push('Invalid target date');
    }

    // Check if targeting protected period
    const protectedPeriod = this.checkProtectedPeriod(targetDate);
    if (protectedPeriod) {
      if (protectedPeriod.protectionLevel === ProtectionLevel.ABSOLUTE) {
        violations.push(
          `Target period "${protectedPeriod.name}" is absolutely protected`
        );
      } else if (protectedPeriod.protectionLevel === ProtectionLevel.ENHANCED) {
        warnings.push(
          `Target period "${protectedPeriod.name}" requires enhanced review`
        );
        conditions.push('Enhanced ERB approval required');
      }
    }

    // Check if targeting protected event
    const nearbyEvents = this.findNearbyProtectedEvents(
      targetDate,
      operation.location
    );
    for (const event of nearbyEvents) {
      if (event.protectionLevel === ProtectionLevel.ABSOLUTE) {
        violations.push(
          `Protected event "${event.name}" is within operation timeframe`
        );
      } else {
        warnings.push(`Near protected event: ${event.name}`);
      }
    }

    // Check intervention level
    const interferenceLevel = this.determineInterferenceLevel(operation);
    if (interferenceLevel === InterferenceLevel.LEVEL_1_ABSOLUTE) {
      violations.push('Operation requires Level 1 interference (prohibited)');
    }

    // Check for prohibited actions based on purpose and intervention
    if (
      operation.purpose.toLowerCase().includes('financial') ||
      operation.purpose.toLowerCase().includes('profit')
    ) {
      const action = this.prohibitedActions.get('stock-trading');
      if (action) {
        violations.push('Financial exploitation detected');
        prohibitedActionsList.push(action);
      }
    }

    // Check traveler certification (placeholder - would check database)
    conditions.push('Valid temporal ethics certification required');

    // Determine monitoring requirements
    monitoring.push('Real-time location tracking');
    monitoring.push('Activity logging');
    if (operation.interventionLevel !== 'none') {
      monitoring.push('Enhanced compliance monitoring');
      monitoring.push('Timeline integrity checks');
    }

    // Assess risk level
    let riskLevel: 'low' | 'medium' | 'high' | 'extreme';
    if (violations.length > 0) {
      riskLevel = 'extreme';
    } else if (
      interferenceLevel === InterferenceLevel.LEVEL_2_CONDITIONAL &&
      warnings.length > 2
    ) {
      riskLevel = 'high';
    } else if (interferenceLevel === InterferenceLevel.LEVEL_2_CONDITIONAL) {
      riskLevel = 'medium';
    } else {
      riskLevel = 'low';
    }

    // Determine approval and recommendation
    const approved = violations.length === 0;
    let recommendation: 'proceed' | 'proceed-with-caution' | 'modify' | 'abort';
    if (!approved) {
      recommendation = 'abort';
    } else if (warnings.length > 3 || riskLevel === 'high') {
      recommendation = 'modify';
    } else if (warnings.length > 0 || conditions.length > 2) {
      recommendation = 'proceed-with-caution';
    } else {
      recommendation = 'proceed';
    }

    // Build rationale
    let rationale = '';
    if (approved) {
      rationale = `Operation approved with ${conditions.length} conditions. Risk level: ${riskLevel}. `;
      if (warnings.length > 0) {
        rationale += `${warnings.length} warnings issued. `;
      }
      rationale += 'Ensure all conditions are met and monitoring is in place.';
    } else {
      rationale = `Operation denied due to ${violations.length} violations. `;
      rationale += violations.join('; ') + '. ';
      rationale += 'Review operation parameters and resubmit.';
    }

    return {
      approved,
      riskLevel,
      interferenceLevel,
      violations,
      warnings,
      conditions,
      monitoring,
      prohibitedActions: prohibitedActionsList,
      recommendation,
      rationale,
    };
  }

  /**
   * Check if date falls within protected period
   */
  private checkProtectedPeriod(date: Date): ProtectedPeriod | null {
    for (const period of this.protectedPeriods.values()) {
      if (date >= period.startDate && date <= period.endDate) {
        return period;
      }
    }
    return null;
  }

  /**
   * Find protected events near target date/location
   */
  private findNearbyProtectedEvents(
    date: Date,
    location: { lat: number; lon: number }
  ): ProtectedEvent[] {
    const nearby: ProtectedEvent[] = [];
    const timeWindow = 7 * 24 * 3600 * 1000; // 7 days in ms

    for (const event of this.protectedEvents.values()) {
      const timeDiff = Math.abs(event.date.getTime() - date.getTime());
      if (timeDiff < timeWindow) {
        // Check geographic proximity (simple distance check)
        const distance = this.haversineDistance(
          location.lat,
          location.lon,
          event.location.lat,
          event.location.lon
        );
        if (distance < 1000) {
          // Within 1000 km
          nearby.push(event);
        }
      }
    }

    return nearby;
  }

  /**
   * Calculate haversine distance between two points
   */
  private haversineDistance(
    lat1: number,
    lon1: number,
    lat2: number,
    lon2: number
  ): number {
    const R = 6371; // Earth's radius in km
    const dLat = ((lat2 - lat1) * Math.PI) / 180;
    const dLon = ((lon2 - lon1) * Math.PI) / 180;
    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos((lat1 * Math.PI) / 180) *
        Math.cos((lat2 * Math.PI) / 180) *
        Math.sin(dLon / 2) *
        Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
  }

  /**
   * Determine interference level for operation
   */
  private determineInterferenceLevel(
    operation: TemporalOperationRequest
  ): InterferenceLevel {
    if (
      operation.interventionLevel === 'major' ||
      operation.interventionLevel === 'catastrophic'
    ) {
      return InterferenceLevel.LEVEL_1_ABSOLUTE;
    }

    if (
      operation.interventionLevel === 'moderate' ||
      operation.interventionLevel === 'minimal'
    ) {
      return InterferenceLevel.LEVEL_2_CONDITIONAL;
    }

    // None intervention
    return InterferenceLevel.LEVEL_2_CONDITIONAL; // Still requires review for observation
  }

  // ============================================================================
  // Interference Assessment
  // ============================================================================

  /**
   * Assess interference level for a specific action
   *
   * @param assessment - Interference assessment parameters
   * @returns Assessment result with level and recommendations
   */
  assessInterferenceLevel(
    assessment: InterferenceAssessment
  ): InterferenceResult {
    let level: InterferenceLevel;
    let permitted = false;
    let reviewType: ReviewType;
    let riskScore = 0;
    const potentialConsequences: string[] = [];
    const requiredSafeguards: string[] = [];
    const alternatives: string[] = [];

    // Assess based on historical significance
    if (assessment.historicalSignificance === HistoricalSignificance.CRITICAL) {
      level = InterferenceLevel.LEVEL_1_ABSOLUTE;
      reviewType = ReviewType.SUPREME_COUNCIL;
      riskScore = 95;
      permitted = false;
      potentialConsequences.push('Major timeline disruption');
      potentialConsequences.push('Paradox creation risk');
      alternatives.push('Remote observation only');
      alternatives.push('Historical record analysis');
    } else if (
      assessment.historicalSignificance === HistoricalSignificance.HIGH
    ) {
      level = InterferenceLevel.LEVEL_2_CONDITIONAL;
      reviewType = ReviewType.ENHANCED;
      riskScore = 65;
      permitted = true;
      potentialConsequences.push('Moderate timeline impact');
      requiredSafeguards.push('Strict observer protocols');
      requiredSafeguards.push('Real-time monitoring');
      alternatives.push('Archival research');
    } else {
      level = InterferenceLevel.LEVEL_2_CONDITIONAL;
      reviewType = ReviewType.STANDARD;
      riskScore = 35;
      permitted = true;
      requiredSafeguards.push('Standard observer protocols');
      requiredSafeguards.push('Activity logging');
    }

    // Adjust for cascading potential
    if (assessment.cascadingPotential === 'extreme') {
      riskScore = Math.min(100, riskScore + 30);
      potentialConsequences.push('High probability of cascade effects');
      requiredSafeguards.push('Enhanced timeline monitoring');
    } else if (assessment.cascadingPotential === 'high') {
      riskScore = Math.min(100, riskScore + 20);
      potentialConsequences.push('Moderate cascade effect risk');
    }

    // Adjust for reversibility
    if (assessment.reversibility === 'not-reversible') {
      riskScore = Math.min(100, riskScore + 15);
      potentialConsequences.push('Irreversible timeline changes');
      requiredSafeguards.push('Pre-action timeline baseline');
    }

    // Calculate timeline impact probability
    const timelineImpactProbability = riskScore / 100;

    // Generate recommendation
    let recommendation: string;
    if (permitted && riskScore < 50) {
      recommendation = 'Operation can proceed with standard safeguards';
    } else if (permitted && riskScore < 75) {
      recommendation = 'Operation permitted but requires enhanced review and monitoring';
    } else if (permitted) {
      recommendation = 'Operation permitted only under strict conditions with Supreme Council oversight';
    } else {
      recommendation = 'Operation prohibited due to unacceptable risk to timeline integrity';
    }

    return {
      level,
      permitted,
      reviewType,
      riskScore,
      timelineImpactProbability,
      potentialConsequences,
      requiredSafeguards,
      alternatives,
      recommendation,
    };
  }

  // ============================================================================
  // Observer Protocols
  // ============================================================================

  /**
   * Generate observer protocol for operation
   *
   * @param operation - Temporal operation request
   * @returns Observer protocol configuration
   */
  generateObserverProtocol(
    operation: TemporalOperationRequest
  ): ObserverProtocol {
    let status: ObserverStatus;
    let physicalInteractionAllowed = false;
    let communicationAllowed = false;
    const equipmentRestrictions: string[] = [];
    let emergencyInterventionPermitted = false;

    // Determine status based on intervention level
    if (operation.interventionLevel === 'none') {
      status = ObserverStatus.PASSIVE_OBSERVER;
      equipmentRestrictions.push('No visible recording devices');
      equipmentRestrictions.push('Stealth mode required');
    } else if (operation.interventionLevel === 'minimal') {
      status = ObserverStatus.LIMITED_INTERACTION;
      communicationAllowed = true;
      equipmentRestrictions.push('Period-appropriate equipment only');
      equipmentRestrictions.push('Concealed modern devices');
    } else if (operation.interventionLevel === 'moderate') {
      status = ObserverStatus.ACTIVE_PARTICIPANT;
      physicalInteractionAllowed = true;
      communicationAllowed = true;
      emergencyInterventionPermitted = true;
    } else {
      status = ObserverStatus.EMERGENCY_INTERVENTION;
      physicalInteractionAllowed = true;
      communicationAllowed = true;
      emergencyInterventionPermitted = true;
    }

    // Stealth requirements
    const stealthRequirements = {
      visualConcealment: status === ObserverStatus.PASSIVE_OBSERVER,
      audioConcealment: status === ObserverStatus.PASSIVE_OBSERVER,
      emConcealment: true, // Always required
      biologicalIsolation: true, // Always required
    };

    // Maximum duration based on purpose
    const maxDuration =
      operation.purpose === OperationPurpose.SCIENTIFIC_RESEARCH
        ? ETHICS_CONSTANTS.MAX_OBSERVATION_DURATION * 3600
        : operation.duration || 3600;

    // Required certifications
    const requiredCertifications = ['basic-temporal-ethics'];
    if (status !== ObserverStatus.PASSIVE_OBSERVER) {
      requiredCertifications.push('advanced-temporal-ethics');
    }
    if (emergencyInterventionPermitted) {
      requiredCertifications.push('emergency-intervention');
    }

    return {
      status,
      physicalInteractionAllowed,
      communicationAllowed,
      equipmentRestrictions,
      stealthRequirements,
      emergencyInterventionPermitted,
      maxDuration,
      requiredCertifications,
    };
  }

  // ============================================================================
  // Violation Management
  // ============================================================================

  /**
   * Create violation record
   *
   * @param violation - Violation details
   * @returns Created violation record
   */
  createViolationRecord(
    violation: Omit<ViolationRecord, 'id' | 'violationDate'>
  ): ViolationRecord {
    const record: ViolationRecord = {
      id: `VIO-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      violationDate: new Date(),
      ...violation,
    };

    // Auto-determine consequences based on severity
    if (!violation.consequences) {
      record.consequences = this.calculateConsequences(violation.severity);
    }

    return record;
  }

  /**
   * Calculate consequences based on violation severity
   */
  private calculateConsequences(
    severity: ViolationSeverity
  ): {
    suspension: number;
    fine: number;
    criminalCharges: boolean;
    permanentRecord: boolean;
  } {
    switch (severity) {
      case ViolationSeverity.MINOR:
        return {
          suspension: 0,
          fine: 0,
          criminalCharges: false,
          permanentRecord: false,
        };

      case ViolationSeverity.MODERATE:
        return {
          suspension: 9, // 9 months
          fine: 50000,
          criminalCharges: false,
          permanentRecord: true,
        };

      case ViolationSeverity.SEVERE:
        return {
          suspension: 84, // 7 years
          fine: 500000,
          criminalCharges: true,
          permanentRecord: true,
        };

      case ViolationSeverity.CRITICAL:
        return {
          suspension: -1, // Permanent
          fine: 1000000,
          criminalCharges: true,
          permanentRecord: true,
        };

      case ViolationSeverity.CATASTROPHIC:
        return {
          suspension: -1, // Permanent
          fine: 10000000,
          criminalCharges: true,
          permanentRecord: true,
        };
    }
  }

  // ============================================================================
  // Compliance Monitoring
  // ============================================================================

  /**
   * Generate compliance report
   *
   * @param subject - Traveler or organization ID
   * @param startDate - Report period start
   * @param endDate - Report period end
   * @returns Compliance report
   */
  generateComplianceReport(
    subject: string,
    startDate: Date,
    endDate: Date
  ): ComplianceReport {
    // This would query actual database - placeholder implementation
    const report: ComplianceReport = {
      id: `RPT-${Date.now()}`,
      subject,
      period: { start: startDate, end: endDate },
      operations: {
        total: 10,
        approved: 8,
        denied: 2,
        pending: 0,
      },
      violations: {
        total: 1,
        minor: 1,
        moderate: 0,
        severe: 0,
        critical: 0,
        catastrophic: 0,
      },
      training: {
        current: true,
        lastUpdated: new Date(Date.now() - 30 * 24 * 3600 * 1000),
        hoursCompleted: 48,
        hoursRequired: 40,
      },
      complianceScore: 92,
      status: 'compliant',
      recommendations: [
        'Continue current compliance practices',
        'Consider advanced certification',
      ],
      nextAudit: new Date(Date.now() + 365 * 24 * 3600 * 1000),
    };

    return report;
  }

  // ============================================================================
  // Training and Certification
  // ============================================================================

  /**
   * Validate training certification
   *
   * @param certification - Certification to validate
   * @returns Whether certification is valid
   */
  validateCertification(certification: TrainingCertification): boolean {
    if (!certification.isCurrent) {
      return false;
    }

    if (certification.expirationDate < new Date()) {
      return false;
    }

    // Check continuing education
    const requiredHours = ETHICS_CONSTANTS.CONTINUING_ED_HOURS;
    if (certification.continuingEducation.hoursThisYear < requiredHours) {
      return false;
    }

    return true;
  }

  /**
   * Check if traveler can conduct operation based on certification
   *
   * @param certification - Traveler's certification
   * @param operation - Requested operation
   * @returns Whether traveler is qualified
   */
  checkTravelerQualification(
    certification: TrainingCertification,
    operation: TemporalOperationRequest
  ): { qualified: boolean; reasons: string[] } {
    const reasons: string[] = [];

    if (!this.validateCertification(certification)) {
      reasons.push('Certification expired or invalid');
      return { qualified: false, reasons };
    }

    // Check certification level vs operation requirements
    if (
      operation.interventionLevel !== 'none' &&
      certification.level === CertificationLevel.BASIC
    ) {
      reasons.push('Advanced certification required for intervention operations');
    }

    // Check restrictions
    if (certification.restrictions) {
      for (const restriction of certification.restrictions) {
        if (restriction.includes(operation.purpose)) {
          reasons.push(`Certification has restriction: ${restriction}`);
        }
      }
    }

    return {
      qualified: reasons.length === 0,
      reasons,
    };
  }

  // ============================================================================
  // Emergency Intervention
  // ============================================================================

  /**
   * Authorize emergency intervention
   *
   * @param request - Emergency intervention request
   * @returns Authorization with conditions
   */
  authorizeEmergencyIntervention(
    request: Omit<EmergencyIntervention, 'id' | 'status' | 'expiration'>
  ): EmergencyIntervention {
    const authorization: EmergencyIntervention = {
      id: `EMG-${Date.now()}`,
      ...request,
      status: 'pending',
      expiration: new Date(Date.now() + ETHICS_CONSTANTS.EMERGENCY_REVIEW_HOURS * 3600 * 1000),
    };

    // Auto-approve low-severity emergencies with minimal impact
    if (
      request.threat.severity === 'low' &&
      request.estimatedImpact === 'minimal'
    ) {
      authorization.status = 'approved';
      authorization.approver = 'auto-approval-system';
      authorization.approvalTimestamp = new Date();
      authorization.conditions = [
        'Real-time monitoring required',
        'Immediate reporting of any deviations',
        'Timeline verification post-intervention',
      ];
    }

    return authorization;
  }
}

// ============================================================================
// Validator Class
// ============================================================================

/**
 * Ethics validator for quick checks
 */
export class EthicsValidator {
  private sdk: TimeTravelEthicsSDK;

  constructor() {
    this.sdk = new TimeTravelEthicsSDK();
  }

  /**
   * Quick validation of operation
   */
  async validateOperation(
    operation: TemporalOperationRequest
  ): Promise<EthicsValidationResult> {
    return this.sdk.validateOperation(operation);
  }

  /**
   * Check interference level
   */
  assessInterferenceLevel(
    assessment: InterferenceAssessment
  ): InterferenceResult {
    return this.sdk.assessInterferenceLevel(assessment);
  }

  /**
   * Check if action is prohibited
   */
  isActionProhibited(action: string, category: ProhibitedActionCategory): boolean {
    // Simple check - would query full database in production
    const prohibited = [
      'stock trading',
      'betting',
      'assassination',
      'technology transfer',
    ];

    return prohibited.some((p) => action.toLowerCase().includes(p));
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate ethics score for operation
 */
export function calculateEthicsScore(
  scientificMerit: number,
  ethicalCompliance: number,
  feasibility: number,
  riskBenefit: number
): number {
  return scientificMerit + ethicalCompliance + feasibility + riskBenefit;
}

/**
 * Format violation severity for display
 */
export function formatSeverity(severity: ViolationSeverity): string {
  const icons = {
    [ViolationSeverity.MINOR]: '⚠️',
    [ViolationSeverity.MODERATE]: '⚠️⚠️',
    [ViolationSeverity.SEVERE]: '🚫',
    [ViolationSeverity.CRITICAL]: '🚫🚫',
    [ViolationSeverity.CATASTROPHIC]: '☢️',
  };

  return `${icons[severity]} ${severity.toUpperCase()}`;
}

/**
 * Check if date is in recent history
 */
export function isRecentHistory(date: Date): boolean {
  const yearsAgo =
    (Date.now() - date.getTime()) / (365.25 * 24 * 3600 * 1000);
  return yearsAgo <= ETHICS_CONSTANTS.RECENT_HISTORY_YEARS;
}

// ============================================================================
// Export All
// ============================================================================

export {
  // Main classes
  TimeTravelEthicsSDK,
  EthicsValidator,

  // Re-export types
  EthicalPrinciple,
  InterferenceLevel,
  OperationPurpose,
  ProtectionLevel,
  HistoricalSignificance,
  ProhibitedActionCategory,
  ReviewType,
  ReviewDecision,
  ObserverStatus,
  ViolationSeverity,
  CertificationLevel,
  EthicsErrorCode,
  TimeTravelEthicsError,
  ETHICS_CONSTANTS,
};

export type {
  TemporalOperationRequest,
  ProtectedPeriod,
  ProtectedEvent,
  ProhibitedAction,
  EthicalReview,
  EthicsValidationResult,
  ObserverProtocol,
  InterferenceAssessment,
  InterferenceResult,
  ViolationRecord,
  ComplianceReport,
  TrainingCertification,
  TimelineIntegrityCheck,
  EmergencyIntervention,
};
