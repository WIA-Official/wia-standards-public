# Chapter 3: Data Formats - Zod Schemas and TypeScript Types

## Complete Legal Data Schema Library

This chapter provides comprehensive Zod schemas and TypeScript types for the WIA Cryo Legal Standard, covering legal frameworks, contracts, disputes, compliance, and reporting.

## Core Legal Framework Schemas

```typescript
/**
 * WIA Cryo Legal Standard - Core Legal Framework Schemas
 * Comprehensive type definitions with Zod validation
 */

import { z } from 'zod';

// ============================================================================
// Base Types and Common Schemas
// ============================================================================

export const ContactInfoSchema = z.object({
  name: z.string().min(1, 'Contact name is required').max(200),
  email: z.string().email('Valid email is required'),
  phone: z.string().optional(),
});

export type ContactInfo = z.infer<typeof ContactInfoSchema>;

export const LegalCounselSchema = z.object({
  name: z.string().min(1, 'Counsel name is required'),
  firm: z.string().optional(),
  bar: z.string().min(1, 'Bar number is required'),
  contact: ContactInfoSchema,
});

export type LegalCounsel = z.infer<typeof LegalCounselSchema>;

export const JurisdictionSchema = z.object({
  country: z.string().length(2, 'Country must be ISO 3166-1 alpha-2'),
  state: z.string().optional(),
  type: z.enum(['primary', 'secondary']),
  applicableLaws: z.array(z.string()).min(1, 'At least one applicable law required'),
});

export type Jurisdiction = z.infer<typeof JurisdictionSchema>;

export const ProjectStatusSchema = z.enum(['active', 'pending', 'suspended', 'archived']);
export type ProjectStatus = z.infer<typeof ProjectStatusSchema>;

// ============================================================================
// Organization Schema
// ============================================================================

export const OrganizationTypeSchema = z.enum(['facility', 'law-firm', 'authority', 'association']);
export type OrganizationType = z.infer<typeof OrganizationTypeSchema>;

export const OrganizationSchema = z.object({
  name: z.string().min(1).max(300),
  type: OrganizationTypeSchema,
  country: z.string().length(2),
  registrationNumber: z.string().optional(),
  contact: ContactInfoSchema,
  legalCounsel: LegalCounselSchema.optional(),
});

export type Organization = z.infer<typeof OrganizationSchema>;

// ============================================================================
// Project Metadata Schema
// ============================================================================

export const ProjectMetadataSchema = z.object({
  id: z.string().uuid('Project ID must be a valid UUID'),
  name: z.string().min(1).max(200),
  description: z.string().optional(),
  organization: OrganizationSchema,
  jurisdictions: z.array(JurisdictionSchema).min(1, 'At least one jurisdiction required'),
  createdAt: z.string().datetime(),
  updatedAt: z.string().datetime().optional(),
  status: ProjectStatusSchema,
});

export type ProjectMetadata = z.infer<typeof ProjectMetadataSchema>;

// ============================================================================
// Regulation Schema
// ============================================================================

export const RegulationCategorySchema = z.enum([
  'tissue-banking',
  'reproductive',
  'research',
  'transplantation',
  'privacy',
  'consumer-protection',
]);

export type RegulationCategory = z.infer<typeof RegulationCategorySchema>;

export const RegulationStatusSchema = z.enum(['current', 'amended', 'superseded']);
export type RegulationStatus = z.infer<typeof RegulationStatusSchema>;

export const RegulationSchema = z.object({
  id: z.string().min(1),
  name: z.string().min(1).max(300),
  jurisdiction: z.string().min(1),
  category: RegulationCategorySchema,
  requirements: z.array(z.string()).min(1),
  penalties: z.array(z.string()),
  effectiveDate: z.string().datetime(),
  lastReview: z.string().datetime(),
  status: RegulationStatusSchema,
});

export type Regulation = z.infer<typeof RegulationSchema>;

// Validation helper for regulation creation
export class RegulationValidator {
  static validate(data: unknown): Regulation {
    return RegulationSchema.parse(data);
  }

  static validatePartial(data: unknown): Partial<Regulation> {
    return RegulationSchema.partial().parse(data);
  }

  static safeValidate(data: unknown): z.SafeParseReturnType<unknown, Regulation> {
    return RegulationSchema.safeParse(data);
  }
}

// ============================================================================
// Legal Rights Schema
// ============================================================================

export const RightHolderSchema = z.enum(['donor', 'patient', 'facility', 'beneficiary']);
export type RightHolder = z.infer<typeof RightHolderSchema>;

export const LegalRightSchema = z.object({
  id: z.string().min(1),
  name: z.string().min(1).max(200),
  holder: RightHolderSchema,
  description: z.string().min(10),
  basis: z.array(z.string()).min(1, 'Legal basis required'),
  limitations: z.array(z.string()).optional(),
  enforcement: z.string().min(1),
});

export type LegalRight = z.infer<typeof LegalRightSchema>;

// ============================================================================
// Legal Obligations Schema
// ============================================================================

export const LegalObligationSchema = z.object({
  id: z.string().min(1),
  name: z.string().min(1).max(200),
  obligor: z.string().min(1),
  description: z.string().min(10),
  basis: z.string().min(1),
  deadline: z.string().optional(),
  penalty: z.string().optional(),
});

export type LegalObligation = z.infer<typeof LegalObligationSchema>;

// ============================================================================
// Liability Schema
// ============================================================================

export const LiabilityTypeSchema = z.enum([
  'strict',
  'negligence',
  'contractual',
  'statutory',
  'product',
]);

export type LiabilityType = z.infer<typeof LiabilityTypeSchema>;

export const LiabilitySchema = z.object({
  id: z.string().min(1),
  type: LiabilityTypeSchema,
  description: z.string().min(10),
  coverage: z.string().min(1),
  limit: z.number().positive().optional(),
  insurance: z.string().optional(),
  mitigation: z.array(z.string()).min(1),
});

export type Liability = z.infer<typeof LiabilitySchema>;

// ============================================================================
// Legal Precedent Schema
// ============================================================================

export const LegalPrecedentSchema = z.object({
  id: z.string().min(1),
  case: z.string().min(1).max(300),
  court: z.string().min(1),
  date: z.string().datetime(),
  jurisdiction: z.string().min(1),
  ruling: z.string().min(10),
  relevance: z.string().min(10),
  citation: z.string().min(1),
});

export type LegalPrecedent = z.infer<typeof LegalPrecedentSchema>;

// ============================================================================
// Complete Legal Framework Schema
// ============================================================================

export const LegalFrameworkSchema = z.object({
  regulations: z.array(RegulationSchema),
  rights: z.array(LegalRightSchema),
  obligations: z.array(LegalObligationSchema),
  liabilities: z.array(LiabilitySchema),
  precedents: z.array(LegalPrecedentSchema),
});

export type LegalFramework = z.infer<typeof LegalFrameworkSchema>;

// Legal Framework Manager with validation
export class LegalFrameworkManager {
  private framework: LegalFramework;

  constructor(initialData?: Partial<LegalFramework>) {
    this.framework = {
      regulations: initialData?.regulations || [],
      rights: initialData?.rights || [],
      obligations: initialData?.obligations || [],
      liabilities: initialData?.liabilities || [],
      precedents: initialData?.precedents || [],
    };
  }

  addRegulation(regulation: unknown): Regulation {
    const validated = RegulationSchema.parse(regulation);

    // Check for duplicate ID
    if (this.framework.regulations.some(r => r.id === validated.id)) {
      throw new Error(`Regulation with ID ${validated.id} already exists`);
    }

    this.framework.regulations.push(validated);
    return validated;
  }

  addRight(right: unknown): LegalRight {
    const validated = LegalRightSchema.parse(right);

    if (this.framework.rights.some(r => r.id === validated.id)) {
      throw new Error(`Right with ID ${validated.id} already exists`);
    }

    this.framework.rights.push(validated);
    return validated;
  }

  addObligation(obligation: unknown): LegalObligation {
    const validated = LegalObligationSchema.parse(obligation);

    if (this.framework.obligations.some(o => o.id === validated.id)) {
      throw new Error(`Obligation with ID ${validated.id} already exists`);
    }

    this.framework.obligations.push(validated);
    return validated;
  }

  addLiability(liability: unknown): Liability {
    const validated = LiabilitySchema.parse(liability);

    if (this.framework.liabilities.some(l => l.id === validated.id)) {
      throw new Error(`Liability with ID ${validated.id} already exists`);
    }

    this.framework.liabilities.push(validated);
    return validated;
  }

  addPrecedent(precedent: unknown): LegalPrecedent {
    const validated = LegalPrecedentSchema.parse(precedent);

    if (this.framework.precedents.some(p => p.id === validated.id)) {
      throw new Error(`Precedent with ID ${validated.id} already exists`);
    }

    this.framework.precedents.push(validated);
    return validated;
  }

  getFramework(): LegalFramework {
    return LegalFrameworkSchema.parse(this.framework);
  }

  findRegulationsByCategory(category: RegulationCategory): Regulation[] {
    return this.framework.regulations.filter(r => r.category === category);
  }

  findRightsByHolder(holder: RightHolder): LegalRight[] {
    return this.framework.rights.filter(r => r.holder === holder);
  }

  findLiabilitiesByType(type: LiabilityType): Liability[] {
    return this.framework.liabilities.filter(l => l.type === type);
  }
}
```

## Contract Management Schemas

```typescript
/**
 * Contract Management Schema Definitions
 * Complete contract lifecycle data structures
 */

// ============================================================================
// Contract Types
// ============================================================================

export const ContractTypeSchema = z.enum([
  'storage-agreement',
  'consent-form',
  'service-agreement',
  'transfer-agreement',
  'research-agreement',
  'donation-agreement',
]);

export type ContractType = z.infer<typeof ContractTypeSchema>;

export const ContractStatusSchema = z.enum([
  'draft',
  'pending',
  'active',
  'expired',
  'terminated',
  'disputed',
]);

export type ContractStatus = z.infer<typeof ContractStatusSchema>;

// ============================================================================
// Clause Schema
// ============================================================================

export const ClauseTypeSchema = z.enum(['standard', 'optional', 'negotiable']);
export type ClauseType = z.infer<typeof ClauseTypeSchema>;

export const ClauseSchema = z.object({
  id: z.string().min(1),
  title: z.string().min(1).max(200),
  text: z.string().min(10),
  type: ClauseTypeSchema,
  mandatory: z.boolean(),
  category: z.string().min(1),
});

export type Clause = z.infer<typeof ClauseSchema>;

// ============================================================================
// Contract Template Schema
// ============================================================================

export const TemplateStatusSchema = z.enum(['draft', 'approved', 'active', 'deprecated']);
export type TemplateStatus = z.infer<typeof TemplateStatusSchema>;

export const ContractTemplateSchema = z.object({
  id: z.string().min(1),
  name: z.string().min(1).max(200),
  type: ContractTypeSchema,
  version: z.string().regex(/^\d+\.\d+\.\d+$/, 'Version must be semver format'),
  jurisdiction: z.string().min(1),
  clauses: z.array(ClauseSchema).min(1, 'Template must have at least one clause'),
  approvedBy: z.string().min(1),
  approvedAt: z.string().datetime(),
  status: TemplateStatusSchema,
});

export type ContractTemplate = z.infer<typeof ContractTemplateSchema>;

// ============================================================================
// Party Schema
// ============================================================================

export const PartyTypeSchema = z.enum(['individual', 'organization']);
export type PartyType = z.infer<typeof PartyTypeSchema>;

export const PartyRoleSchema = z.enum(['primary', 'counterparty', 'guarantor', 'witness']);
export type PartyRole = z.infer<typeof PartyRoleSchema>;

export const PartySchema = z.object({
  id: z.string().min(1),
  type: PartyTypeSchema,
  name: z.string().min(1).max(300),
  role: PartyRoleSchema,
  representative: z.string().optional(),
  contact: ContactInfoSchema,
});

export type Party = z.infer<typeof PartySchema>;

// ============================================================================
// Contract Subject Schema
// ============================================================================

export const ContractSubjectTypeSchema = z.enum(['specimen', 'service', 'facility', 'research']);
export type ContractSubjectType = z.infer<typeof ContractSubjectTypeSchema>;

export const ContractSubjectSchema = z.object({
  type: ContractSubjectTypeSchema,
  identifier: z.string().min(1),
  description: z.string().min(10),
});

export type ContractSubject = z.infer<typeof ContractSubjectSchema>;

// ============================================================================
// Fee Schema
// ============================================================================

export const FeeFrequencySchema = z.enum(['one-time', 'monthly', 'annual']);
export type FeeFrequency = z.infer<typeof FeeFrequencySchema>;

export const FeeSchema = z.object({
  type: z.string().min(1),
  amount: z.number().nonnegative(),
  currency: z.string().length(3, 'Currency must be ISO 4217 code'),
  frequency: FeeFrequencySchema,
  due: z.string().min(1),
});

export type Fee = z.infer<typeof FeeSchema>;

// ============================================================================
// Contract Terms Schema
// ============================================================================

export const RenewalTypeSchema = z.enum(['automatic', 'manual', 'none']);
export type RenewalType = z.infer<typeof RenewalTypeSchema>;

export const ContractTermsSchema = z.object({
  duration: z.string().min(1),
  renewal: RenewalTypeSchema,
  fees: z.array(FeeSchema).optional(),
  obligations: z.array(z.object({
    party: z.string().min(1),
    obligation: z.string().min(1),
  })),
  warranties: z.array(z.string()),
  limitations: z.array(z.string()),
  governing: z.string().min(1),
  venue: z.string().min(1),
});

export type ContractTerms = z.infer<typeof ContractTermsSchema>;

// ============================================================================
// Signature Schema
// ============================================================================

export const SignatureMethodSchema = z.enum(['wet', 'electronic', 'digital']);
export type SignatureMethod = z.infer<typeof SignatureMethodSchema>;

export const SignatureSchema = z.object({
  party: z.string().min(1),
  signatory: z.string().min(1),
  date: z.string().datetime(),
  method: SignatureMethodSchema,
  witness: z.string().optional(),
  notarized: z.boolean().optional(),
});

export type Signature = z.infer<typeof SignatureSchema>;

// ============================================================================
// Contract Event Schema
// ============================================================================

export const ContractEventSchema = z.object({
  id: z.string().min(1),
  type: z.string().min(1),
  date: z.string().datetime(),
  actor: z.string().min(1),
  description: z.string().min(1),
  documents: z.array(z.string()).optional(),
});

export type ContractEvent = z.infer<typeof ContractEventSchema>;

// ============================================================================
// Complete Contract Schema
// ============================================================================

export const ContractSchema = z.object({
  id: z.string().min(1),
  templateId: z.string().optional(),
  type: ContractTypeSchema,
  parties: z.array(PartySchema).min(2, 'Contract requires at least two parties'),
  subject: ContractSubjectSchema.optional(),
  terms: ContractTermsSchema,
  signatures: z.array(SignatureSchema),
  effectiveDate: z.string().datetime(),
  expiryDate: z.string().datetime().optional(),
  status: ContractStatusSchema,
  history: z.array(ContractEventSchema),
});

export type Contract = z.infer<typeof ContractSchema>;

// Contract validation with business rules
export class ContractValidator {
  static validate(data: unknown): Contract {
    const contract = ContractSchema.parse(data);

    // Business rule: Active contracts must have signatures from all parties
    if (contract.status === 'active') {
      const partyIds = new Set(contract.parties.map(p => p.id));
      const signedParties = new Set(contract.signatures.map(s => s.party));

      for (const partyId of partyIds) {
        if (!signedParties.has(partyId)) {
          throw new Error(`Party ${partyId} has not signed the contract`);
        }
      }
    }

    // Business rule: Expiry date must be after effective date
    if (contract.expiryDate) {
      const effective = new Date(contract.effectiveDate);
      const expiry = new Date(contract.expiryDate);

      if (expiry <= effective) {
        throw new Error('Expiry date must be after effective date');
      }
    }

    return contract;
  }

  static validateForExecution(data: unknown): Contract {
    const contract = this.validate(data);

    // Additional checks for contract execution
    if (contract.parties.length < 2) {
      throw new Error('Contract requires at least two parties');
    }

    const hasPrimary = contract.parties.some(p => p.role === 'primary');
    const hasCounterparty = contract.parties.some(p => p.role === 'counterparty');

    if (!hasPrimary || !hasCounterparty) {
      throw new Error('Contract must have both primary and counterparty');
    }

    return contract;
  }
}

// ============================================================================
// Standard Terms Schema
// ============================================================================

export const StandardTermsSchema = z.object({
  storage: z.array(z.string()),
  liability: z.array(z.string()),
  termination: z.array(z.string()),
  dispute: z.array(z.string()),
  privacy: z.array(z.string()),
});

export type StandardTerms = z.infer<typeof StandardTermsSchema>;

// ============================================================================
// Amendment Process Schema
// ============================================================================

export const AmendmentProcessSchema = z.object({
  allowed: z.boolean(),
  procedure: z.string(),
  approval: z.array(z.string()),
  documentation: z.array(z.string()),
});

export type AmendmentProcess = z.infer<typeof AmendmentProcessSchema>;

// ============================================================================
// Termination Procedures Schema
// ============================================================================

export const TerminationProceduresSchema = z.object({
  notice: z.string().min(1),
  grounds: z.array(z.string()).min(1),
  procedure: z.array(z.string()).min(1),
  consequences: z.array(z.string()),
});

export type TerminationProcedures = z.infer<typeof TerminationProceduresSchema>;

// ============================================================================
// Contract Management Schema
// ============================================================================

export const ContractManagementSchema = z.object({
  templates: z.array(ContractTemplateSchema),
  contracts: z.array(ContractSchema),
  terms: StandardTermsSchema,
  amendments: AmendmentProcessSchema,
  termination: TerminationProceduresSchema,
});

export type ContractManagement = z.infer<typeof ContractManagementSchema>;
```

## Dispute Resolution Schemas

```typescript
/**
 * Dispute Resolution Schema Definitions
 * Complete dispute handling data structures
 */

// ============================================================================
// Resolution Mechanism Schema
// ============================================================================

export const ResolutionTypeSchema = z.enum([
  'negotiation',
  'mediation',
  'arbitration',
  'litigation',
]);

export type ResolutionType = z.infer<typeof ResolutionTypeSchema>;

export const ResolutionMechanismSchema = z.object({
  type: ResolutionTypeSchema,
  order: z.number().int().positive(),
  mandatory: z.boolean(),
  provider: z.string().optional(),
  rules: z.string().optional(),
  timeframe: z.string().min(1),
  costs: z.string().min(1),
});

export type ResolutionMechanism = z.infer<typeof ResolutionMechanismSchema>;

// ============================================================================
// Dispute Type and Status
// ============================================================================

export const DisputeTypeSchema = z.enum([
  'contract',
  'consent',
  'ownership',
  'negligence',
  'regulatory',
]);

export type DisputeType = z.infer<typeof DisputeTypeSchema>;

export const DisputeStatusSchema = z.enum([
  'filed',
  'under-review',
  'negotiation',
  'mediation',
  'arbitration',
  'litigation',
  'resolved',
  'dismissed',
]);

export type DisputeStatus = z.infer<typeof DisputeStatusSchema>;

// ============================================================================
// Resolution Schema
// ============================================================================

export const ResolutionSchema = z.object({
  date: z.string().datetime(),
  mechanism: z.string().min(1),
  outcome: z.string().min(10),
  terms: z.array(z.string()),
  binding: z.boolean(),
  enforcement: z.string().optional(),
});

export type Resolution = z.infer<typeof ResolutionSchema>;

// ============================================================================
// Dispute Event Schema
// ============================================================================

export const DisputeEventSchema = z.object({
  date: z.string().datetime(),
  type: z.string().min(1),
  description: z.string().min(1),
  actor: z.string().min(1),
  documents: z.array(z.string()).optional(),
});

export type DisputeEvent = z.infer<typeof DisputeEventSchema>;

// ============================================================================
// Dispute Schema
// ============================================================================

export const DisputeSchema = z.object({
  id: z.string().min(1),
  type: DisputeTypeSchema,
  parties: z.array(z.string()).min(2, 'Dispute requires at least two parties'),
  subject: z.string().min(1),
  description: z.string().min(10),
  filedDate: z.string().datetime(),
  status: DisputeStatusSchema,
  currentMechanism: z.string().optional(),
  resolution: ResolutionSchema.optional(),
  history: z.array(DisputeEventSchema),
});

export type Dispute = z.infer<typeof DisputeSchema>;

// Dispute validation with status rules
export class DisputeValidator {
  static validate(data: unknown): Dispute {
    const dispute = DisputeSchema.parse(data);

    // Business rule: Resolved disputes must have resolution
    if (dispute.status === 'resolved' && !dispute.resolution) {
      throw new Error('Resolved disputes must have resolution details');
    }

    // Business rule: Active mechanism must match status
    if (dispute.currentMechanism) {
      const statusMechanismMap: Record<string, string[]> = {
        'negotiation': ['negotiation'],
        'mediation': ['mediation'],
        'arbitration': ['arbitration'],
        'litigation': ['litigation'],
      };

      const expectedMechanisms = statusMechanismMap[dispute.status] || [];
      if (expectedMechanisms.length > 0 &&
          !expectedMechanisms.includes(dispute.currentMechanism)) {
        throw new Error(
          `Current mechanism ${dispute.currentMechanism} does not match status ${dispute.status}`
        );
      }
    }

    return dispute;
  }

  static validateStatusTransition(
    currentStatus: DisputeStatus,
    newStatus: DisputeStatus
  ): boolean {
    const validTransitions: Record<DisputeStatus, DisputeStatus[]> = {
      'filed': ['under-review', 'dismissed'],
      'under-review': ['negotiation', 'mediation', 'dismissed'],
      'negotiation': ['mediation', 'arbitration', 'resolved', 'dismissed'],
      'mediation': ['arbitration', 'litigation', 'resolved', 'dismissed'],
      'arbitration': ['litigation', 'resolved'],
      'litigation': ['resolved', 'dismissed'],
      'resolved': [],
      'dismissed': [],
    };

    return validTransitions[currentStatus]?.includes(newStatus) || false;
  }
}

// ============================================================================
// Escalation Path Schema
// ============================================================================

export const EscalationLevelSchema = z.object({
  level: z.number().int().positive(),
  mechanism: z.string().min(1),
  trigger: z.string().min(1),
  timeframe: z.string().min(1),
});

export type EscalationLevel = z.infer<typeof EscalationLevelSchema>;

export const EscalationPathSchema = z.object({
  levels: z.array(EscalationLevelSchema).min(1),
  finalResort: z.string().min(1),
});

export type EscalationPath = z.infer<typeof EscalationPathSchema>;

// ============================================================================
// Dispute Documentation Schema
// ============================================================================

export const DisputeDocumentationSchema = z.object({
  required: z.array(z.string()).min(1),
  retention: z.string().min(1),
  confidentiality: z.string().min(1),
});

export type DisputeDocumentation = z.infer<typeof DisputeDocumentationSchema>;

// ============================================================================
// Complete Dispute Resolution Schema
// ============================================================================

export const DisputeResolutionSchema = z.object({
  mechanisms: z.array(ResolutionMechanismSchema).min(1),
  disputes: z.array(DisputeSchema),
  escalation: EscalationPathSchema,
  documentation: DisputeDocumentationSchema,
});

export type DisputeResolution = z.infer<typeof DisputeResolutionSchema>;
```

## Compliance Management Schemas

```typescript
/**
 * Compliance Management Schema Definitions
 * Complete regulatory compliance data structures
 */

// ============================================================================
// Compliance Requirement Schema
// ============================================================================

export const ComplianceStatusSchema = z.enum([
  'compliant',
  'non-compliant',
  'partial',
  'pending',
]);

export type ComplianceStatus = z.infer<typeof ComplianceStatusSchema>;

export const ComplianceRequirementSchema = z.object({
  id: z.string().min(1),
  regulation: z.string().min(1),
  requirement: z.string().min(10),
  responsible: z.string().min(1),
  evidence: z.array(z.string()).min(1),
  frequency: z.string().min(1),
  lastCheck: z.string().datetime(),
  status: ComplianceStatusSchema,
});

export type ComplianceRequirement = z.infer<typeof ComplianceRequirementSchema>;

// ============================================================================
// Audit Program Schema
// ============================================================================

export const InternalAuditSchema = z.object({
  frequency: z.string().min(1),
  scope: z.array(z.string()).min(1),
  team: z.string().min(1),
});

export type InternalAudit = z.infer<typeof InternalAuditSchema>;

export const ExternalAuditSchema = z.object({
  frequency: z.string().min(1),
  auditor: z.string().min(1),
  scope: z.array(z.string()).min(1),
});

export type ExternalAudit = z.infer<typeof ExternalAuditSchema>;

export const RegulatoryAuditSchema = z.object({
  agencies: z.array(z.string()).min(1),
  schedule: z.string().min(1),
});

export type RegulatoryAudit = z.infer<typeof RegulatoryAuditSchema>;

export const AuditProgramSchema = z.object({
  internal: InternalAuditSchema,
  external: ExternalAuditSchema,
  regulatory: RegulatoryAuditSchema,
});

export type AuditProgram = z.infer<typeof AuditProgramSchema>;

// ============================================================================
// Violation Management Schema
// ============================================================================

export const ViolationManagementSchema = z.object({
  reporting: z.string().min(1),
  investigation: z.string().min(1),
  remediation: z.string().min(1),
  disclosure: z.string().min(1),
  tracking: z.boolean(),
});

export type ViolationManagement = z.infer<typeof ViolationManagementSchema>;

// ============================================================================
// Compliance Training Schema
// ============================================================================

export const ComplianceTrainingSchema = z.object({
  topics: z.array(z.string()).min(1),
  frequency: z.string().min(1),
  mandatory: z.boolean(),
  records: z.boolean(),
});

export type ComplianceTraining = z.infer<typeof ComplianceTrainingSchema>;

// ============================================================================
// Complete Compliance Management Schema
// ============================================================================

export const ComplianceManagementSchema = z.object({
  requirements: z.array(ComplianceRequirementSchema),
  audits: AuditProgramSchema,
  violations: ViolationManagementSchema,
  training: ComplianceTrainingSchema,
});

export type ComplianceManagement = z.infer<typeof ComplianceManagementSchema>;

// Compliance tracking utilities
export class ComplianceTracker {
  private requirements: Map<string, ComplianceRequirement> = new Map();

  constructor(initialRequirements: ComplianceRequirement[] = []) {
    for (const req of initialRequirements) {
      this.requirements.set(req.id, req);
    }
  }

  addRequirement(requirement: unknown): ComplianceRequirement {
    const validated = ComplianceRequirementSchema.parse(requirement);
    this.requirements.set(validated.id, validated);
    return validated;
  }

  updateStatus(id: string, status: ComplianceStatus, evidence?: string[]): void {
    const req = this.requirements.get(id);
    if (!req) {
      throw new Error(`Requirement ${id} not found`);
    }

    req.status = status;
    req.lastCheck = new Date().toISOString();

    if (evidence) {
      req.evidence = [...req.evidence, ...evidence];
    }

    this.requirements.set(id, req);
  }

  getComplianceScore(): ComplianceScore {
    const total = this.requirements.size;
    let compliant = 0;
    let nonCompliant = 0;
    let partial = 0;
    let pending = 0;

    for (const req of this.requirements.values()) {
      switch (req.status) {
        case 'compliant': compliant++; break;
        case 'non-compliant': nonCompliant++; break;
        case 'partial': partial++; break;
        case 'pending': pending++; break;
      }
    }

    const score = total > 0
      ? ((compliant + partial * 0.5) / total) * 100
      : 0;

    return {
      total,
      compliant,
      nonCompliant,
      partial,
      pending,
      score: Math.round(score * 100) / 100,
      rating: this.getRating(score),
    };
  }

  private getRating(score: number): string {
    if (score >= 95) return 'Excellent';
    if (score >= 85) return 'Good';
    if (score >= 70) return 'Satisfactory';
    if (score >= 50) return 'Needs Improvement';
    return 'Critical';
  }

  getOverdueRequirements(thresholdDays: number = 30): ComplianceRequirement[] {
    const threshold = new Date();
    threshold.setDate(threshold.getDate() - thresholdDays);

    return Array.from(this.requirements.values()).filter(req => {
      const lastCheck = new Date(req.lastCheck);
      return lastCheck < threshold;
    });
  }

  getNonCompliantRequirements(): ComplianceRequirement[] {
    return Array.from(this.requirements.values())
      .filter(req => req.status === 'non-compliant');
  }
}

export interface ComplianceScore {
  total: number;
  compliant: number;
  nonCompliant: number;
  partial: number;
  pending: number;
  score: number;
  rating: string;
}
```

## Legal Reporting Schemas

```typescript
/**
 * Legal Reporting Schema Definitions
 * Complete reporting and documentation structures
 */

// ============================================================================
// Internal Reporting Schema
// ============================================================================

export const InternalReportingSchema = z.object({
  schedule: z.string().min(1),
  recipients: z.array(z.string()).min(1),
  content: z.array(z.string()).min(1),
});

export type InternalReporting = z.infer<typeof InternalReportingSchema>;

// ============================================================================
// Regulatory Reporting Schema
// ============================================================================

export const AgencyReportSchema = z.object({
  name: z.string().min(1),
  reports: z.array(z.object({
    type: z.string().min(1),
    frequency: z.string().min(1),
  })).min(1),
});

export type AgencyReport = z.infer<typeof AgencyReportSchema>;

export const RegulatoryReportingSchema = z.object({
  agencies: z.array(AgencyReportSchema),
  format: z.string().min(1),
  retention: z.string().min(1),
});

export type RegulatoryReporting = z.infer<typeof RegulatoryReportingSchema>;

// ============================================================================
// Incident Reporting Schema
// ============================================================================

export const IncidentReportingSchema = z.object({
  types: z.array(z.string()).min(1),
  timeframe: z.string().min(1),
  authorities: z.array(z.string()),
  procedure: z.string().min(1),
});

export type IncidentReporting = z.infer<typeof IncidentReportingSchema>;

// ============================================================================
// Complete Legal Reporting Schema
// ============================================================================

export const LegalReportingSchema = z.object({
  internal: InternalReportingSchema,
  regulatory: RegulatoryReportingSchema,
  incidents: IncidentReportingSchema,
});

export type LegalReporting = z.infer<typeof LegalReportingSchema>;

// ============================================================================
// Complete WIA Cryo Legal Project Schema
// ============================================================================

export const WIACryoLegalProjectSchema = z.object({
  standard: z.literal('WIA-CRYO-LEGAL'),
  version: z.string().regex(/^\d+\.\d+\.\d+$/),
  metadata: ProjectMetadataSchema,
  legalFramework: LegalFrameworkSchema,
  contracts: ContractManagementSchema,
  disputes: DisputeResolutionSchema,
  compliance: ComplianceManagementSchema,
  reporting: LegalReportingSchema,
  extensions: z.record(z.unknown()).optional(),
});

export type WIACryoLegalProject = z.infer<typeof WIACryoLegalProjectSchema>;

// Project factory with validation
export class CryoLegalProjectFactory {
  static create(data: unknown): WIACryoLegalProject {
    return WIACryoLegalProjectSchema.parse(data);
  }

  static createEmpty(orgId: string, name: string): WIACryoLegalProject {
    const now = new Date().toISOString();

    return WIACryoLegalProjectSchema.parse({
      standard: 'WIA-CRYO-LEGAL',
      version: '1.0.0',
      metadata: {
        id: crypto.randomUUID(),
        name,
        organization: {
          name: 'Organization',
          type: 'facility',
          country: 'US',
          contact: {
            name: 'Contact',
            email: 'contact@org.com',
          },
        },
        jurisdictions: [{
          country: 'US',
          type: 'primary',
          applicableLaws: ['FDA-CGTP'],
        }],
        createdAt: now,
        status: 'pending',
      },
      legalFramework: {
        regulations: [],
        rights: [],
        obligations: [],
        liabilities: [],
        precedents: [],
      },
      contracts: {
        templates: [],
        contracts: [],
        terms: {
          storage: [],
          liability: [],
          termination: [],
          dispute: [],
          privacy: [],
        },
        amendments: {
          allowed: true,
          procedure: 'Written agreement by all parties',
          approval: ['Legal counsel'],
          documentation: ['Amendment form'],
        },
        termination: {
          notice: '30 days written notice',
          grounds: ['Breach of contract', 'Mutual agreement'],
          procedure: ['Written notice', 'Cure period', 'Final notice'],
          consequences: ['Return of specimens', 'Fee settlement'],
        },
      },
      disputes: {
        mechanisms: [{
          type: 'negotiation',
          order: 1,
          mandatory: true,
          timeframe: '30 days',
          costs: 'Each party bears own costs',
        }],
        disputes: [],
        escalation: {
          levels: [{
            level: 1,
            mechanism: 'negotiation',
            trigger: 'Dispute filed',
            timeframe: '30 days',
          }],
          finalResort: 'Litigation',
        },
        documentation: {
          required: ['Dispute form', 'Evidence'],
          retention: '7 years',
          confidentiality: 'Confidential',
        },
      },
      compliance: {
        requirements: [],
        audits: {
          internal: {
            frequency: 'Annual',
            scope: ['Policies', 'Procedures'],
            team: 'Compliance team',
          },
          external: {
            frequency: 'Biennial',
            auditor: 'External auditor',
            scope: ['Full compliance review'],
          },
          regulatory: {
            agencies: ['FDA'],
            schedule: 'As required',
          },
        },
        violations: {
          reporting: 'Immediate to compliance officer',
          investigation: 'Within 48 hours',
          remediation: 'Within 30 days',
          disclosure: 'As required by law',
          tracking: true,
        },
        training: {
          topics: ['CGTP', 'Privacy'],
          frequency: 'Annual',
          mandatory: true,
          records: true,
        },
      },
      reporting: {
        internal: {
          schedule: 'Monthly',
          recipients: ['Management', 'Board'],
          content: ['Compliance status', 'Issues'],
        },
        regulatory: {
          agencies: [{
            name: 'FDA',
            reports: [{
              type: 'Annual registration',
              frequency: 'Annual',
            }],
          }],
          format: 'As specified by agency',
          retention: '7 years',
        },
        incidents: {
          types: ['Adverse events', 'Deviations'],
          timeframe: '24 hours',
          authorities: ['FDA'],
          procedure: 'Per SOP',
        },
      },
    });
  }
}
```

---

## Chapter Summary

This chapter provided complete Zod schemas for the WIA Cryo Legal Standard:

- **Core Schemas**: Organization, Jurisdiction, Legal Framework
- **Contract Schemas**: Templates, Parties, Terms, Signatures
- **Dispute Schemas**: Mechanisms, Status, Resolution
- **Compliance Schemas**: Requirements, Audits, Violations
- **Reporting Schemas**: Internal, Regulatory, Incidents

---

**Next Chapter**: [API Interface - REST, GraphQL, and WebSocket](./04-api-interface.md)
