# 3장: 데이터 형식 - Zod 스키마 및 TypeScript 타입

## 완전한 법률 데이터 스키마 라이브러리

이 장에서는 WIA 극저온 법률 표준을 위한 포괄적인 Zod 스키마와 TypeScript 타입을 제공합니다. 법률 프레임워크, 계약, 분쟁, 준수, 보고를 포함합니다.

## 핵심 법률 프레임워크 스키마

```typescript
/**
 * WIA 극저온 법률 표준 - 핵심 법률 프레임워크 스키마
 * Zod 유효성 검사를 포함한 포괄적 타입 정의
 */

import { z } from 'zod';

// ============================================================================
// 기본 타입 및 공통 스키마
// ============================================================================

// 연락처 정보 스키마
export const ContactInfoSchema = z.object({
  name: z.string().min(1, '연락처 이름 필수').max(200),
  email: z.string().email('유효한 이메일 필수'),
  phone: z.string().optional(),
});

export type ContactInfo = z.infer<typeof ContactInfoSchema>;

// 법률 고문 스키마
export const LegalCounselSchema = z.object({
  name: z.string().min(1, '고문 이름 필수'),
  firm: z.string().optional(),
  bar: z.string().min(1, '변호사 번호 필수'),
  contact: ContactInfoSchema,
});

export type LegalCounsel = z.infer<typeof LegalCounselSchema>;

// 관할권 스키마
export const JurisdictionSchema = z.object({
  country: z.string().length(2, '국가는 ISO 3166-1 alpha-2 형식'),
  state: z.string().optional(),
  type: z.enum(['primary', 'secondary']),
  applicableLaws: z.array(z.string()).min(1, '최소 하나의 적용 법률 필수'),
});

export type Jurisdiction = z.infer<typeof JurisdictionSchema>;

// 프로젝트 상태 스키마
export const ProjectStatusSchema = z.enum(['active', 'pending', 'suspended', 'archived']);
export type ProjectStatus = z.infer<typeof ProjectStatusSchema>;

// ============================================================================
// 조직 스키마
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
// 프로젝트 메타데이터 스키마
// ============================================================================

export const ProjectMetadataSchema = z.object({
  id: z.string().uuid('프로젝트 ID는 유효한 UUID여야 함'),
  name: z.string().min(1).max(200),
  description: z.string().optional(),
  organization: OrganizationSchema,
  jurisdictions: z.array(JurisdictionSchema).min(1, '최소 하나의 관할권 필수'),
  createdAt: z.string().datetime(),
  updatedAt: z.string().datetime().optional(),
  status: ProjectStatusSchema,
});

export type ProjectMetadata = z.infer<typeof ProjectMetadataSchema>;

// ============================================================================
// 규정 스키마
// ============================================================================

export const RegulationCategorySchema = z.enum([
  'tissue-banking',      // 조직 은행
  'reproductive',        // 생식의학
  'research',            // 연구
  'transplantation',     // 이식
  'privacy',             // 개인정보
  'consumer-protection', // 소비자 보호
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

// 규정 유효성 검사 헬퍼
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
// 법적 권리 스키마
// ============================================================================

export const RightHolderSchema = z.enum(['donor', 'patient', 'facility', 'beneficiary']);
export type RightHolder = z.infer<typeof RightHolderSchema>;

export const LegalRightSchema = z.object({
  id: z.string().min(1),
  name: z.string().min(1).max(200),
  holder: RightHolderSchema,
  description: z.string().min(10),
  basis: z.array(z.string()).min(1, '법적 근거 필수'),
  limitations: z.array(z.string()).optional(),
  enforcement: z.string().min(1),
});

export type LegalRight = z.infer<typeof LegalRightSchema>;

// ============================================================================
// 법적 의무 스키마
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
// 책임 스키마
// ============================================================================

export const LiabilityTypeSchema = z.enum([
  'strict',       // 무과실 책임
  'negligence',   // 과실 책임
  'contractual',  // 계약 책임
  'statutory',    // 법정 책임
  'product',      // 제조물 책임
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
// 법적 선례 스키마
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
// 완전한 법률 프레임워크 스키마
// ============================================================================

export const LegalFrameworkSchema = z.object({
  regulations: z.array(RegulationSchema),
  rights: z.array(LegalRightSchema),
  obligations: z.array(LegalObligationSchema),
  liabilities: z.array(LiabilitySchema),
  precedents: z.array(LegalPrecedentSchema),
});

export type LegalFramework = z.infer<typeof LegalFrameworkSchema>;

// 유효성 검사 포함 법률 프레임워크 관리자
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

    // 중복 ID 확인
    if (this.framework.regulations.some(r => r.id === validated.id)) {
      throw new Error(`ID ${validated.id}의 규정이 이미 존재함`);
    }

    this.framework.regulations.push(validated);
    return validated;
  }

  addRight(right: unknown): LegalRight {
    const validated = LegalRightSchema.parse(right);

    if (this.framework.rights.some(r => r.id === validated.id)) {
      throw new Error(`ID ${validated.id}의 권리가 이미 존재함`);
    }

    this.framework.rights.push(validated);
    return validated;
  }

  addObligation(obligation: unknown): LegalObligation {
    const validated = LegalObligationSchema.parse(obligation);

    if (this.framework.obligations.some(o => o.id === validated.id)) {
      throw new Error(`ID ${validated.id}의 의무가 이미 존재함`);
    }

    this.framework.obligations.push(validated);
    return validated;
  }

  addLiability(liability: unknown): Liability {
    const validated = LiabilitySchema.parse(liability);

    if (this.framework.liabilities.some(l => l.id === validated.id)) {
      throw new Error(`ID ${validated.id}의 책임이 이미 존재함`);
    }

    this.framework.liabilities.push(validated);
    return validated;
  }

  addPrecedent(precedent: unknown): LegalPrecedent {
    const validated = LegalPrecedentSchema.parse(precedent);

    if (this.framework.precedents.some(p => p.id === validated.id)) {
      throw new Error(`ID ${validated.id}의 선례가 이미 존재함`);
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

## 계약 관리 스키마

```typescript
/**
 * 계약 관리 스키마 정의
 * 완전한 계약 생명주기 데이터 구조
 */

// ============================================================================
// 계약 유형
// ============================================================================

export const ContractTypeSchema = z.enum([
  'storage-agreement',   // 보관 계약
  'consent-form',        // 동의서
  'service-agreement',   // 서비스 계약
  'transfer-agreement',  // 이전 계약
  'research-agreement',  // 연구 계약
  'donation-agreement',  // 기증 계약
]);

export type ContractType = z.infer<typeof ContractTypeSchema>;

export const ContractStatusSchema = z.enum([
  'draft',      // 초안
  'pending',    // 대기 중
  'active',     // 활성
  'expired',    // 만료
  'terminated', // 해지
  'disputed',   // 분쟁 중
]);

export type ContractStatus = z.infer<typeof ContractStatusSchema>;

// ============================================================================
// 조항 스키마
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
// 계약 템플릿 스키마
// ============================================================================

export const TemplateStatusSchema = z.enum(['draft', 'approved', 'active', 'deprecated']);
export type TemplateStatus = z.infer<typeof TemplateStatusSchema>;

export const ContractTemplateSchema = z.object({
  id: z.string().min(1),
  name: z.string().min(1).max(200),
  type: ContractTypeSchema,
  version: z.string().regex(/^\d+\.\d+\.\d+$/, '버전은 semver 형식이어야 함'),
  jurisdiction: z.string().min(1),
  clauses: z.array(ClauseSchema).min(1, '템플릿에는 최소 하나의 조항 필요'),
  approvedBy: z.string().min(1),
  approvedAt: z.string().datetime(),
  status: TemplateStatusSchema,
});

export type ContractTemplate = z.infer<typeof ContractTemplateSchema>;

// ============================================================================
// 당사자 스키마
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
// 계약 대상 스키마
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
// 수수료 스키마
// ============================================================================

export const FeeFrequencySchema = z.enum(['one-time', 'monthly', 'annual']);
export type FeeFrequency = z.infer<typeof FeeFrequencySchema>;

export const FeeSchema = z.object({
  type: z.string().min(1),
  amount: z.number().nonnegative(),
  currency: z.string().length(3, '통화는 ISO 4217 코드여야 함'),
  frequency: FeeFrequencySchema,
  due: z.string().min(1),
});

export type Fee = z.infer<typeof FeeSchema>;

// ============================================================================
// 계약 조건 스키마
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
// 서명 스키마
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
// 계약 이벤트 스키마
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
// 완전한 계약 스키마
// ============================================================================

export const ContractSchema = z.object({
  id: z.string().min(1),
  templateId: z.string().optional(),
  type: ContractTypeSchema,
  parties: z.array(PartySchema).min(2, '계약에는 최소 두 당사자 필요'),
  subject: ContractSubjectSchema.optional(),
  terms: ContractTermsSchema,
  signatures: z.array(SignatureSchema),
  effectiveDate: z.string().datetime(),
  expiryDate: z.string().datetime().optional(),
  status: ContractStatusSchema,
  history: z.array(ContractEventSchema),
});

export type Contract = z.infer<typeof ContractSchema>;

// 비즈니스 규칙을 포함한 계약 유효성 검사
export class ContractValidator {
  static validate(data: unknown): Contract {
    const contract = ContractSchema.parse(data);

    // 비즈니스 규칙: 활성 계약은 모든 당사자의 서명 필요
    if (contract.status === 'active') {
      const partyIds = new Set(contract.parties.map(p => p.id));
      const signedParties = new Set(contract.signatures.map(s => s.party));

      for (const partyId of partyIds) {
        if (!signedParties.has(partyId)) {
          throw new Error(`당사자 ${partyId}가 계약에 서명하지 않음`);
        }
      }
    }

    // 비즈니스 규칙: 만료일은 발효일 이후여야 함
    if (contract.expiryDate) {
      const effective = new Date(contract.effectiveDate);
      const expiry = new Date(contract.expiryDate);

      if (expiry <= effective) {
        throw new Error('만료일은 발효일 이후여야 함');
      }
    }

    return contract;
  }

  static validateForExecution(data: unknown): Contract {
    const contract = this.validate(data);

    // 계약 실행을 위한 추가 검사
    if (contract.parties.length < 2) {
      throw new Error('계약에는 최소 두 당사자 필요');
    }

    const hasPrimary = contract.parties.some(p => p.role === 'primary');
    const hasCounterparty = contract.parties.some(p => p.role === 'counterparty');

    if (!hasPrimary || !hasCounterparty) {
      throw new Error('계약에는 주 당사자와 상대방이 모두 필요');
    }

    return contract;
  }
}
```

## 분쟁 해결 스키마

```typescript
/**
 * 분쟁 해결 스키마 정의
 * 완전한 분쟁 처리 데이터 구조
 */

// ============================================================================
// 해결 메커니즘 스키마
// ============================================================================

export const ResolutionTypeSchema = z.enum([
  'negotiation',  // 협상
  'mediation',    // 조정
  'arbitration',  // 중재
  'litigation',   // 소송
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
// 분쟁 유형 및 상태
// ============================================================================

export const DisputeTypeSchema = z.enum([
  'contract',    // 계약 분쟁
  'consent',     // 동의 분쟁
  'ownership',   // 소유권 분쟁
  'negligence',  // 과실 분쟁
  'regulatory',  // 규제 분쟁
]);

export type DisputeType = z.infer<typeof DisputeTypeSchema>;

export const DisputeStatusSchema = z.enum([
  'filed',        // 제출됨
  'under-review', // 검토 중
  'negotiation',  // 협상 중
  'mediation',    // 조정 중
  'arbitration',  // 중재 중
  'litigation',   // 소송 중
  'resolved',     // 해결됨
  'dismissed',    // 기각됨
]);

export type DisputeStatus = z.infer<typeof DisputeStatusSchema>;

// ============================================================================
// 해결 스키마
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
// 분쟁 이벤트 스키마
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
// 분쟁 스키마
// ============================================================================

export const DisputeSchema = z.object({
  id: z.string().min(1),
  type: DisputeTypeSchema,
  parties: z.array(z.string()).min(2, '분쟁에는 최소 두 당사자 필요'),
  subject: z.string().min(1),
  description: z.string().min(10),
  filedDate: z.string().datetime(),
  status: DisputeStatusSchema,
  currentMechanism: z.string().optional(),
  resolution: ResolutionSchema.optional(),
  history: z.array(DisputeEventSchema),
});

export type Dispute = z.infer<typeof DisputeSchema>;

// 상태 규칙을 포함한 분쟁 유효성 검사
export class DisputeValidator {
  static validate(data: unknown): Dispute {
    const dispute = DisputeSchema.parse(data);

    // 비즈니스 규칙: 해결된 분쟁은 해결 세부정보 필요
    if (dispute.status === 'resolved' && !dispute.resolution) {
      throw new Error('해결된 분쟁에는 해결 세부정보 필요');
    }

    // 비즈니스 규칙: 활성 메커니즘은 상태와 일치해야 함
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
          `현재 메커니즘 ${dispute.currentMechanism}이(가) 상태 ${dispute.status}와 일치하지 않음`
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
// 에스컬레이션 경로 스키마
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
// 완전한 분쟁 해결 스키마
// ============================================================================

export const DisputeResolutionSchema = z.object({
  mechanisms: z.array(ResolutionMechanismSchema).min(1),
  disputes: z.array(DisputeSchema),
  escalation: EscalationPathSchema,
  documentation: z.object({
    required: z.array(z.string()).min(1),
    retention: z.string().min(1),
    confidentiality: z.string().min(1),
  }),
});

export type DisputeResolution = z.infer<typeof DisputeResolutionSchema>;
```

## 준수 관리 스키마

```typescript
/**
 * 준수 관리 스키마 정의
 * 완전한 규제 준수 데이터 구조
 */

// ============================================================================
// 준수 요구사항 스키마
// ============================================================================

export const ComplianceStatusSchema = z.enum([
  'compliant',     // 준수
  'non-compliant', // 미준수
  'partial',       // 부분 준수
  'pending',       // 대기 중
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
// 감사 프로그램 스키마
// ============================================================================

export const AuditProgramSchema = z.object({
  internal: z.object({
    frequency: z.string().min(1),
    scope: z.array(z.string()).min(1),
    team: z.string().min(1),
  }),
  external: z.object({
    frequency: z.string().min(1),
    auditor: z.string().min(1),
    scope: z.array(z.string()).min(1),
  }),
  regulatory: z.object({
    agencies: z.array(z.string()).min(1),
    schedule: z.string().min(1),
  }),
});

export type AuditProgram = z.infer<typeof AuditProgramSchema>;

// ============================================================================
// 완전한 준수 관리 스키마
// ============================================================================

export const ComplianceManagementSchema = z.object({
  requirements: z.array(ComplianceRequirementSchema),
  audits: AuditProgramSchema,
  violations: z.object({
    reporting: z.string().min(1),
    investigation: z.string().min(1),
    remediation: z.string().min(1),
    disclosure: z.string().min(1),
    tracking: z.boolean(),
  }),
  training: z.object({
    topics: z.array(z.string()).min(1),
    frequency: z.string().min(1),
    mandatory: z.boolean(),
    records: z.boolean(),
  }),
});

export type ComplianceManagement = z.infer<typeof ComplianceManagementSchema>;

// 준수 추적 유틸리티
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
      throw new Error(`요구사항 ${id}을(를) 찾을 수 없음`);
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
    if (score >= 95) return '우수';
    if (score >= 85) return '양호';
    if (score >= 70) return '보통';
    if (score >= 50) return '개선 필요';
    return '위험';
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

---

## 장 요약

이 장에서는 WIA 극저온 법률 표준의 완전한 Zod 스키마를 제공했습니다:

- **핵심 스키마**: 조직, 관할권, 법률 프레임워크
- **계약 스키마**: 템플릿, 당사자, 조건, 서명
- **분쟁 스키마**: 메커니즘, 상태, 해결
- **준수 스키마**: 요구사항, 감사, 위반
- **보고 스키마**: 내부, 규제, 사고

---

**다음 장**: [API 인터페이스 - REST, GraphQL, WebSocket](./04-api-interface.md)
