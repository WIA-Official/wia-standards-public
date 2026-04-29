/**
 * WIA-TIME-031: Temporal Law SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Legal Committee
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for temporal law compliance:
 * - Jurisdiction determination
 * - Traveler registration and status
 * - Property claim filing and verification
 * - Contract validation and enforcement
 * - Crime reporting and tracking
 * - Court proceeding management
 */

import {
  TimeCrimeClass,
  TimeCrimeType,
  LegalStatus,
  JurisdictionLevel,
  CourtLevel,
  PropertyType,
  ContractType,
  CaseType,
  SentencingType,
  TreatyType,
  RemedyType,
  TemporalJurisdiction,
  JurisdictionCheckRequest,
  TimeCrime,
  CrimeReport,
  TravelerRights,
  TravelerRegistration,
  LegalStatusCheck,
  PropertyClaim,
  PropertyTransfer,
  TemporalContract,
  ContractValidation,
  CourtCase,
  Sentencing,
  LawEnforcement,
  InternationalTreaty,
  LegalRemedy,
  LEGAL_CONSTANTS,
  LegalErrorCode,
  TemporalLawError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-031 Temporal Law SDK
 */
export class TemporalLawSDK {
  private version = '1.0.0';
  private initialized = false;
  private timeCrimes: Map<string, TimeCrime> = new Map();
  private treaties: Map<string, InternationalTreaty> = new Map();

  constructor() {
    this.initialized = true;
    this.loadLegalData();
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Load legal data (crimes, treaties, etc.)
   */
  private loadLegalData(): void {
    // Load major time crimes
    this.addTimeCrime({
      id: 'TC-001',
      type: TimeCrimeType.TIMELINE_DESTRUCTION,
      class: TimeCrimeClass.CLASS_A_CATASTROPHIC,
      description: 'Intentional destruction or collapse of timeline',
      statute: 'ITC § 1001',
      penaltyRange: {
        minimum: 'Life imprisonment',
        maximum: 'Temporal erasure',
      },
      elements: [
        'Intentional act',
        'Resulting in timeline destruction or severe damage',
        'Knowledge of consequences',
        'No lawful authorization',
      ],
      statuteOfLimitations: -1,
    });

    this.addTimeCrime({
      id: 'TC-002',
      type: TimeCrimeType.TEMPORAL_EXPLOITATION,
      class: TimeCrimeClass.CLASS_C_SERIOUS,
      description: 'Using time travel for financial exploitation',
      statute: 'ITC § 3015',
      penaltyRange: {
        minimum: '10 years',
        maximum: '20 years + restitution',
      },
      elements: [
        'Use of temporal knowledge',
        'For financial gain',
        'Unfair advantage',
        'Market manipulation',
      ],
      defenses: ['Lack of knowledge', 'Novikov consistency', 'Good faith'],
      statuteOfLimitations: 20,
    });

    this.addTimeCrime({
      id: 'TC-003',
      type: TimeCrimeType.OBSERVER_PROTOCOL_VIOLATION,
      class: TimeCrimeClass.CLASS_D_MODERATE,
      description: 'Violation of temporal observer protocols',
      statute: 'ITC § 4020',
      penaltyRange: {
        minimum: '1 year',
        maximum: '5 years',
      },
      elements: [
        'Temporal travel occurred',
        'Observer protocol violation',
        'Intentional or negligent',
      ],
      defenses: ['Emergency circumstances', 'Impossibility', 'Duress'],
      statuteOfLimitations: 10,
    });

    // Load key treaties
    this.addTreaty({
      id: 'TREATY-001',
      name: 'Treaty on Prevention of Timeline Destruction',
      type: TreatyType.UNIVERSAL,
      signatories: ['All temporal nations'],
      subject: 'Prevention of timeline destruction and catastrophic temporal events',
      provisions: [
        'Prohibition on timeline destruction weapons',
        'International inspection regime',
        'Universal jurisdiction for violations',
        'Mandatory cooperation in enforcement',
      ],
      adoptionDate: new Date('2024-01-01'),
      entryIntoForce: new Date('2024-06-01'),
      status: 'in-force',
      enforcement: {
        body: 'Supreme Temporal Tribunal',
        procedures: ['Inspection', 'Investigation', 'Prosecution'],
        sanctions: ['Asset freeze', 'Timeline access ban', 'Criminal prosecution'],
      },
    });
  }

  /**
   * Add time crime to database
   */
  addTimeCrime(crime: TimeCrime): void {
    this.timeCrimes.set(crime.id, crime);
  }

  /**
   * Add treaty to database
   */
  addTreaty(treaty: InternationalTreaty): void {
    this.treaties.set(treaty.id, treaty);
  }

  // ============================================================================
  // Jurisdiction Methods
  // ============================================================================

  /**
   * Check temporal jurisdiction for a traveler and activity
   *
   * @param request - Jurisdiction check request
   * @returns Jurisdiction determination
   */
  async checkJurisdiction(
    request: JurisdictionCheckRequest
  ): Promise<TemporalJurisdiction> {
    const applicableLaws: string[] = [];
    const conflicts: any[] = [];

    // Determine primary jurisdiction
    let primary: JurisdictionLevel;
    const secondary: JurisdictionLevel[] = [];

    // Origin timeline has primary jurisdiction for its citizens
    primary = JurisdictionLevel.ORIGIN_TIMELINE;
    applicableLaws.push(`Law of ${request.originTimeline} (Origin timeline)`);

    // Destination timeline has concurrent jurisdiction
    secondary.push(JurisdictionLevel.DESTINATION_TIMELINE);
    applicableLaws.push(`Law of ${request.destinationTimeline} (Destination timeline)`);

    // Check for international issues
    if (this.isInternationalMatter(request)) {
      secondary.push(JurisdictionLevel.INTERNATIONAL_TEMPORAL_COURT);
      applicableLaws.push('International Temporal Law');
    }

    // Check for catastrophic issues (Supreme Tribunal jurisdiction)
    if (request.activityType?.includes('catastrophic') ||
        request.activityType?.includes('timeline-destruction')) {
      primary = JurisdictionLevel.SUPREME_TEMPORAL_TRIBUNAL;
      applicableLaws.push('Supreme Temporal Law');
    }

    // Build rationale
    let rationale = `Primary jurisdiction: ${primary}. `;
    rationale += `Traveler from ${request.originTimeline} visiting ${request.destinationTimeline}. `;
    rationale += `Origin timeline law applies for personal status and citizenship. `;
    rationale += `Destination timeline law applies for conduct while present. `;

    if (secondary.length > 0) {
      rationale += `Secondary jurisdictions: ${secondary.join(', ')}. `;
    }

    return {
      primary,
      secondary,
      applicableLaws,
      rationale,
      conflicts: conflicts.length > 0 ? conflicts : undefined,
    };
  }

  /**
   * Check if matter involves international jurisdiction
   */
  private isInternationalMatter(request: JurisdictionCheckRequest): boolean {
    // Simplified check - would be more complex in reality
    return (
      request.activityType?.includes('international') ||
      request.purpose?.includes('cross-timeline') ||
      false
    );
  }

  // ============================================================================
  // Traveler Registration and Status
  // ============================================================================

  /**
   * Register temporal traveler
   *
   * @param registration - Registration details
   * @returns Registration result
   */
  async registerTraveler(
    registration: Omit<TravelerRegistration, 'registrationDate' | 'status' | 'protections'>
  ): Promise<TravelerRegistration> {
    // Validate registration
    if (!registration.travelerId || !registration.citizenship) {
      throw new TemporalLawError(
        LegalErrorCode.PROCEDURAL_ERROR,
        'Missing required registration fields'
      );
    }

    // Determine legal protections
    const protections = this.determineProtections(registration.purpose);

    const result: TravelerRegistration = {
      ...registration,
      registrationDate: new Date(),
      status: 'approved', // Simplified - would go through review
      protections,
      restrictions: this.determineRestrictions(registration.purpose),
    };

    return result;
  }

  /**
   * Determine legal protections based on purpose
   */
  private determineProtections(purpose: string): string[] {
    const protections = [
      'Right to legal counsel',
      'Right to fair trial',
      'Right to return to origin timeline',
      'Consular protection',
    ];

    if (purpose.includes('research') || purpose.includes('educational')) {
      protections.push('Academic freedom protections');
      protections.push('Research confidentiality');
    }

    if (purpose.includes('humanitarian')) {
      protections.push('Humanitarian worker protections');
      protections.push('Immunity for relief work');
    }

    return protections;
  }

  /**
   * Determine restrictions based on purpose
   */
  private determineRestrictions(purpose: string): string[] | undefined {
    const restrictions: string[] = [];

    if (purpose.includes('commercial') || purpose.includes('trade')) {
      restrictions.push('No historical market manipulation');
      restrictions.push('Fair trade requirements apply');
    }

    if (purpose.includes('observation')) {
      restrictions.push('Observer protocols must be followed');
      restrictions.push('No interference permitted');
    }

    return restrictions.length > 0 ? restrictions : undefined;
  }

  /**
   * Check legal status of traveler
   *
   * @param travelerId - Traveler identifier
   * @param timeline - Timeline to check status in
   * @returns Legal status information
   */
  async checkLegalStatus(
    travelerId: string,
    timeline: string | Date
  ): Promise<LegalStatusCheck> {
    // Simplified implementation - would query actual database
    const travelerRights: TravelerRights = {
      legalCounsel: true,
      fairTrial: true,
      timelineIntegrity: true,
      property: true,
      contracts: true,
      appeal: true,
      refuge: true,
      returnRight: true,
    };

    return {
      travelerId,
      timeline,
      status: LegalStatus.VISITOR,
      citizenship: {
        primary: 'USA-2025',
        stateless: false,
      },
      criminalRecord: {
        hasRecord: false,
      },
      availableRights: travelerRights,
      lastUpdated: new Date(),
    };
  }

  // ============================================================================
  // Property Claims
  // ============================================================================

  /**
   * File property claim
   *
   * @param claim - Property claim details
   * @returns Filed claim with status
   */
  async filePropertyClaim(
    claim: Omit<PropertyClaim, 'id' | 'status' | 'filedDate'>
  ): Promise<PropertyClaim> {
    // Validate claim
    if (!claim.claimant || !claim.propertyType || !claim.description) {
      throw new TemporalLawError(
        LegalErrorCode.PROPERTY_CLAIM_DENIED,
        'Insufficient claim information'
      );
    }

    // Check if claim is valid
    const validation = this.validatePropertyClaim(claim);
    if (!validation.valid) {
      throw new TemporalLawError(
        LegalErrorCode.PROPERTY_CLAIM_DENIED,
        `Claim denied: ${validation.reason}`
      );
    }

    const filedClaim: PropertyClaim = {
      id: `PC-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      ...claim,
      status: 'under-review',
      filedDate: new Date(),
    };

    return filedClaim;
  }

  /**
   * Validate property claim
   */
  private validatePropertyClaim(claim: Omit<PropertyClaim, 'id' | 'status' | 'filedDate'>): {
    valid: boolean;
    reason?: string;
  } {
    // Cannot claim historical artifacts without authorization
    if (claim.propertyType === PropertyType.CULTURAL_ARTIFACT) {
      if (claim.basis !== 'original-ownership') {
        return {
          valid: false,
          reason: 'Cultural artifacts cannot be claimed except by original owners',
        };
      }
    }

    // Real property claims restricted
    if (claim.propertyType === PropertyType.REAL_PROPERTY) {
      return {
        valid: false,
        reason: 'Real property cannot be claimed across timelines',
      };
    }

    return { valid: true };
  }

  /**
   * Process property transfer
   *
   * @param transfer - Transfer details
   * @returns Transfer result with validity
   */
  async processPropertyTransfer(
    transfer: Omit<PropertyTransfer, 'id' | 'validity'>
  ): Promise<PropertyTransfer> {
    // Validate transfer
    const validity = this.validateTransfer(transfer);

    const result: PropertyTransfer = {
      id: `PT-${Date.now()}`,
      ...transfer,
      validity,
    };

    return result;
  }

  /**
   * Validate property transfer
   */
  private validateTransfer(transfer: Omit<PropertyTransfer, 'id' | 'validity'>): {
    valid: boolean;
    issues?: string[];
  } {
    const issues: string[] = [];

    // Check if parties are valid
    if (!transfer.transferor || !transfer.transferee) {
      issues.push('Missing party information');
    }

    // Check consideration for sales
    if (transfer.type === 'sale' && !transfer.consideration) {
      issues.push('Sale requires consideration');
    }

    return {
      valid: issues.length === 0,
      issues: issues.length > 0 ? issues : undefined,
    };
  }

  // ============================================================================
  // Temporal Contracts
  // ============================================================================

  /**
   * Validate temporal contract
   *
   * @param contract - Contract to validate
   * @returns Validation result
   */
  async validateContract(contract: TemporalContract): Promise<ContractValidation> {
    const issues: string[] = [];
    const recommendations: string[] = [];

    // Check parties
    if (contract.parties.length < 2) {
      issues.push('Contract requires at least 2 parties');
    }

    // Check terms
    if (!contract.terms || contract.terms.trim().length === 0) {
      issues.push('Contract must have defined terms');
    }

    // Check consideration
    if (!contract.consideration || contract.consideration.trim().length === 0) {
      issues.push('Contract must have consideration');
    }

    // Check timeline compatibility
    const timelineCompatibility = this.checkTimelineCompatibility(contract);
    const legalCompliance = this.checkContractCompliance(contract);

    // Determine enforceability
    let enforceability: 'fully-enforceable' | 'partially-enforceable' | 'unenforceable';
    if (issues.length === 0 && timelineCompatibility.compatible && legalCompliance.compliant) {
      enforceability = 'fully-enforceable';
    } else if (issues.length > 0) {
      enforceability = 'unenforceable';
    } else {
      enforceability = 'partially-enforceable';
    }

    // Add recommendations
    if (!contract.governingLaw) {
      recommendations.push('Specify governing law for clarity');
    }
    if (!contract.disputeResolution) {
      recommendations.push('Include dispute resolution clause');
    }

    let rationale = '';
    if (enforceability === 'fully-enforceable') {
      rationale = 'Contract meets all legal requirements and is fully enforceable across timelines.';
    } else if (enforceability === 'partially-enforceable') {
      rationale = 'Contract has some issues but core terms may be enforceable.';
    } else {
      rationale = `Contract is unenforceable due to: ${issues.join(', ')}`;
    }

    return {
      valid: issues.length === 0,
      issues,
      enforceability,
      rationale,
      requiredModifications: issues.length > 0 ? issues : undefined,
      timelineCompatibility,
      legalCompliance,
      recommendations,
    };
  }

  /**
   * Check timeline compatibility of contract
   */
  private checkTimelineCompatibility(contract: TemporalContract): {
    compatible: boolean;
    conflicts?: string[];
  } {
    const conflicts: string[] = [];

    // Check if execution timeline makes sense
    const origin = new Date(contract.originTimeline);
    const execution = new Date(contract.executionTimeline);

    // Execution should be after or equal to origin (usually)
    if (execution < origin) {
      conflicts.push('Execution timeline precedes origin timeline');
    }

    return {
      compatible: conflicts.length === 0,
      conflicts: conflicts.length > 0 ? conflicts : undefined,
    };
  }

  /**
   * Check legal compliance of contract
   */
  private checkContractCompliance(contract: TemporalContract): {
    compliant: boolean;
    violations?: string[];
  } {
    const violations: string[] = [];

    // Check for illegal purposes
    if (contract.terms.toLowerCase().includes('financial exploitation')) {
      violations.push('Contract purpose involves prohibited financial exploitation');
    }

    if (contract.terms.toLowerCase().includes('historical manipulation')) {
      violations.push('Contract purpose involves prohibited historical manipulation');
    }

    return {
      compliant: violations.length === 0,
      violations: violations.length > 0 ? violations : undefined,
    };
  }

  // ============================================================================
  // Crime Reporting
  // ============================================================================

  /**
   * Report time crime
   *
   * @param report - Crime report details
   * @returns Filed report
   */
  async reportCrime(
    report: Omit<CrimeReport, 'id' | 'filedDate' | 'lastUpdated' | 'status'>
  ): Promise<CrimeReport> {
    // Validate report
    if (!report.reporter || !report.crime) {
      throw new TemporalLawError(
        LegalErrorCode.PROCEDURAL_ERROR,
        'Incomplete crime report'
      );
    }

    const result: CrimeReport = {
      id: `CR-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      ...report,
      status: 'filed',
      filedDate: new Date(),
      lastUpdated: new Date(),
    };

    return result;
  }

  /**
   * Get time crime definition
   *
   * @param crimeType - Type of crime
   * @returns Crime definition
   */
  getCrimeDefinition(crimeType: TimeCrimeType): TimeCrime | undefined {
    for (const crime of this.timeCrimes.values()) {
      if (crime.type === crimeType) {
        return crime;
      }
    }
    return undefined;
  }

  // ============================================================================
  // Court Proceedings
  // ============================================================================

  /**
   * File court case
   *
   * @param caseInfo - Case information
   * @returns Filed case
   */
  async fileCourtCase(
    caseInfo: Omit<CourtCase, 'caseId' | 'caseNumber' | 'filingDate' | 'status'>
  ): Promise<CourtCase> {
    const caseNumber = this.generateCaseNumber(caseInfo.court, caseInfo.type);

    const courtCase: CourtCase = {
      caseId: `CASE-${Date.now()}`,
      caseNumber,
      filingDate: new Date(),
      status: CaseStatus.FILED,
      ...caseInfo,
    };

    return courtCase;
  }

  /**
   * Generate case number
   */
  private generateCaseNumber(court: CourtLevel, type: CaseType): string {
    const year = new Date().getFullYear();
    const courtPrefix = this.getCourtPrefix(court);
    const typePrefix = this.getCaseTypePrefix(type);
    const sequential = Math.floor(Math.random() * 100000).toString().padStart(5, '0');

    return `${courtPrefix}-${typePrefix}-${year}-${sequential}`;
  }

  /**
   * Get court prefix for case number
   */
  private getCourtPrefix(court: CourtLevel): string {
    const prefixes = {
      [CourtLevel.LOCAL_TEMPORAL]: 'LTC',
      [CourtLevel.REGIONAL_APPEALS]: 'RAC',
      [CourtLevel.NATIONAL_SUPREME]: 'NSC',
      [CourtLevel.INTERNATIONAL]: 'ITC',
      [CourtLevel.SUPREME_TRIBUNAL]: 'STT',
    };
    return prefixes[court];
  }

  /**
   * Get case type prefix
   */
  private getCaseTypePrefix(type: CaseType): string {
    const prefixes = {
      [CaseType.CRIMINAL]: 'CR',
      [CaseType.CIVIL]: 'CV',
      [CaseType.ADMINISTRATIVE]: 'AD',
      [CaseType.CONSTITUTIONAL]: 'CO',
      [CaseType.APPELLATE]: 'AP',
    };
    return prefixes[type];
  }

  // ============================================================================
  // Sentencing
  // ============================================================================

  /**
   * Calculate recommended sentence
   *
   * @param conviction - Conviction details
   * @param aggravating - Aggravating factors
   * @param mitigating - Mitigating factors
   * @returns Recommended sentence
   */
  calculateSentence(
    conviction: { crime: TimeCrimeType; class: TimeCrimeClass; counts: number },
    aggravating: string[] = [],
    mitigating: string[] = []
  ): {
    type: SentencingType[];
    duration?: number;
    rationale: string;
  } {
    const types: SentencingType[] = [];
    let duration = 0;

    // Base sentence on crime class
    const minMonths = LEGAL_CONSTANTS.MINIMUM_PENALTIES[conviction.class];

    if (minMonths === -1) {
      // Life sentence
      types.push(SentencingType.IMPRISONMENT);
      duration = -1;
    } else if (minMonths > 0) {
      types.push(SentencingType.IMPRISONMENT);
      duration = minMonths;

      // Adjust for aggravating/mitigating factors
      const adjustment = aggravating.length * 6 - mitigating.length * 3;
      duration = Math.max(minMonths, duration + adjustment);
    } else {
      // Minor crime - probation and fines
      types.push(SentencingType.PROBATION);
      types.push(SentencingType.FINES);
      duration = 12; // 12 months probation
    }

    // Add community service for minor crimes
    if (conviction.class === TimeCrimeClass.CLASS_E_MINOR) {
      types.push(SentencingType.COMMUNITY_SERVICE);
    }

    // Add restitution if exploitation involved
    if (conviction.crime === TimeCrimeType.TEMPORAL_EXPLOITATION) {
      types.push(SentencingType.RESTITUTION);
    }

    let rationale = `Based on ${conviction.class} classification. `;
    rationale += `Minimum penalty: ${minMonths === -1 ? 'Life' : minMonths + ' months'}. `;
    if (aggravating.length > 0) {
      rationale += `Aggravating factors (${aggravating.length}) increase sentence. `;
    }
    if (mitigating.length > 0) {
      rationale += `Mitigating factors (${mitigating.length}) reduce sentence. `;
    }

    return { type: types, duration, rationale };
  }

  // ============================================================================
  // Law Enforcement
  // ============================================================================

  /**
   * Issue law enforcement authorization
   *
   * @param action - Enforcement action type
   * @param target - Target of action
   * @param justification - Legal justification
   * @returns Authorization details
   */
  async authorizeEnforcement(
    action: string,
    target: string,
    justification: string
  ): Promise<LawEnforcement> {
    // Simplified implementation
    const authorization: LawEnforcement = {
      id: `ENF-${Date.now()}`,
      action: action as any,
      target,
      timeline: new Date().toISOString(),
      jurisdiction: JurisdictionLevel.ORIGIN_TIMELINE,
      authority: {
        type: 'court-order',
        issuedBy: 'Temporal Court',
        date: new Date(),
        expirationDate: new Date(Date.now() + 30 * 24 * 3600 * 1000), // 30 days
      },
      agency: 'Temporal Law Enforcement',
      operationDate: new Date(),
    };

    return authorization;
  }

  // ============================================================================
  // Treaties
  // ============================================================================

  /**
   * Get treaty by ID
   *
   * @param treatyId - Treaty identifier
   * @returns Treaty or undefined
   */
  getTreaty(treatyId: string): InternationalTreaty | undefined {
    return this.treaties.get(treatyId);
  }

  /**
   * Check if country is signatory to treaty
   *
   * @param treatyId - Treaty identifier
   * @param country - Country name
   * @returns True if signatory
   */
  isSignatory(treatyId: string, country: string): boolean {
    const treaty = this.treaties.get(treatyId);
    if (!treaty) return false;

    return treaty.signatories.includes(country) ||
           treaty.signatories.includes('All temporal nations');
  }

  // ============================================================================
  // Legal Remedies
  // ============================================================================

  /**
   * Calculate damages
   *
   * @param harm - Description of harm
   * @param amount - Base amount
   * @param timeline - Timeline for valuation
   * @returns Calculated damages
   */
  calculateDamages(
    harm: string,
    amount: number,
    timeline: string
  ): LegalRemedy {
    // Simplified calculation - would involve timeline economic adjustments
    return {
      type: RemedyType.COMPENSATORY_DAMAGES,
      amount: {
        value: amount,
        currency: 'USD',
        timeline,
      },
      description: `Compensatory damages for: ${harm}`,
    };
  }

  /**
   * Generate injunction
   *
   * @param prohibited - Prohibited action
   * @param duration - Duration of injunction (days)
   * @returns Injunction remedy
   */
  generateInjunction(prohibited: string, duration?: number): LegalRemedy {
    return {
      type: RemedyType.INJUNCTION,
      description: `Injunction prohibiting: ${prohibited}`,
      complianceDeadline: duration
        ? new Date(Date.now() + duration * 24 * 3600 * 1000)
        : undefined,
      enforcement: 'Contempt of court sanctions for violations',
    };
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Format case number
 */
export function formatCaseNumber(caseNumber: string): string {
  return caseNumber.toUpperCase();
}

/**
 * Calculate statute of limitations expiration
 */
export function calculateSOLExpiration(
  crimeDate: Date,
  crimeClass: TimeCrimeClass
): Date | null {
  const years = LEGAL_CONSTANTS.STATUTE_OF_LIMITATIONS[crimeClass];

  if (years === -1) {
    return null; // No expiration
  }

  const expiration = new Date(crimeDate);
  expiration.setFullYear(expiration.getFullYear() + years);
  return expiration;
}

/**
 * Check if case is within statute of limitations
 */
export function isWithinSOL(crimeDate: Date, crimeClass: TimeCrimeClass): boolean {
  const expiration = calculateSOLExpiration(crimeDate, crimeClass);

  if (expiration === null) {
    return true; // No SOL
  }

  return new Date() <= expiration;
}

/**
 * Format legal citation
 */
export function formatCitation(
  court: CourtLevel,
  caseNumber: string,
  year: number
): string {
  return `${formatCaseNumber(caseNumber)} (${court}, ${year})`;
}

/**
 * Timeline-adjust currency amount
 */
export function timelineAdjustCurrency(
  amount: number,
  fromTimeline: string,
  toTimeline: string
): number {
  // Simplified - would use actual economic data
  const fromYear = new Date(fromTimeline).getFullYear();
  const toYear = new Date(toTimeline).getFullYear();

  // Assume 3% annual inflation
  const years = toYear - fromYear;
  const adjusted = amount * Math.pow(1.03, years);

  return Math.round(adjusted * 100) / 100;
}

// ============================================================================
// Export All
// ============================================================================

export {
  // Main class
  TemporalLawSDK,

  // Re-export types
  TimeCrimeClass,
  TimeCrimeType,
  LegalStatus,
  JurisdictionLevel,
  CourtLevel,
  PropertyType,
  ContractType,
  CaseType,
  SentencingType,
  TreatyType,
  RemedyType,
  LegalErrorCode,
  TemporalLawError,
  LEGAL_CONSTANTS,
};

export type {
  TemporalJurisdiction,
  JurisdictionCheckRequest,
  TimeCrime,
  CrimeReport,
  TravelerRights,
  TravelerRegistration,
  LegalStatusCheck,
  PropertyClaim,
  PropertyTransfer,
  TemporalContract,
  ContractValidation,
  CourtCase,
  Sentencing,
  LawEnforcement,
  InternationalTreaty,
  LegalRemedy,
};
