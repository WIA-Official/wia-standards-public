# WIA 극저온 법률 표준 - 완벽 기술 가이드

## 극저온 보존 법률 프레임워크 관리 시스템

**표준 코드**: WIA-CRYO-LEGAL v1.0.0

**철학**: 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라

---

## 요약

WIA 극저온 법률 표준은 극저온 보존의 법적 측면을 관리하기 위한 포괄적인 프레임워크를 제공합니다. 이 표준은 규제 준수, 계약 관리, 분쟁 해결, 책임 관리를 포함하며, 극저온 보존 시설, 생물은행, 생식의학 센터가 강력한 법적 프레임워크 내에서 운영되도록 보장합니다.

## 표준 개요

### 목적과 범위

```typescript
/**
 * WIA 극저온 법률 표준 - 핵심 프로젝트 인터페이스
 * 극저온 보존을 위한 포괄적 법률 관리
 */

import { z } from 'zod';

// 핵심 표준 프로젝트 스키마
export const WIACryoLegalProjectSchema = z.object({
  standard: z.literal('WIA-CRYO-LEGAL'),
  version: z.string().regex(/^\d+\.\d+\.\d+$/),
  metadata: z.object({
    id: z.string().uuid(),
    name: z.string().min(1).max(200),
    description: z.string().optional(),
    organization: z.object({
      name: z.string().min(1),
      type: z.enum(['facility', 'law-firm', 'authority', 'association']),
      country: z.string().length(2), // ISO 3166-1 alpha-2
      registrationNumber: z.string().optional(),
      contact: z.object({
        name: z.string().min(1),
        email: z.string().email(),
        phone: z.string().optional(),
      }),
      legalCounsel: z.object({
        name: z.string().min(1),
        firm: z.string().optional(),
        bar: z.string().min(1),
        contact: z.object({
          name: z.string().min(1),
          email: z.string().email(),
          phone: z.string().optional(),
        }),
      }).optional(),
    }),
    jurisdictions: z.array(z.object({
      country: z.string().length(2),
      state: z.string().optional(),
      type: z.enum(['primary', 'secondary']),
      applicableLaws: z.array(z.string()),
    })).min(1),
    createdAt: z.string().datetime(),
    updatedAt: z.string().datetime().optional(),
    status: z.enum(['active', 'pending', 'suspended', 'archived']),
  }),
  legalFramework: z.lazy(() => LegalFrameworkSchema),
  contracts: z.lazy(() => ContractManagementSchema),
  disputes: z.lazy(() => DisputeResolutionSchema),
  compliance: z.lazy(() => ComplianceManagementSchema),
  reporting: z.lazy(() => LegalReportingSchema),
  extensions: z.record(z.unknown()).optional(),
});

export type WIACryoLegalProject = z.infer<typeof WIACryoLegalProjectSchema>;
```

### 대상 사용자

이 표준의 대상:

1. **극저온 보존 시설** - 생물은행, 조직은행, 제대혈은행
2. **생식의학 센터** - IVF 클리닉, 생식력 보존 서비스
3. **법률 전문가** - 의료법 변호사, 생명윤리 전문가
4. **규제 기관** - 보건 당국, 감독 위원회
5. **보험 제공자** - 보존 서비스 책임 보험
6. **연구 기관** - 극저온 연구 수행 학술 의료 센터

### 핵심 컴포넌트

```typescript
/**
 * 핵심 법률 관리 컴포넌트
 * 포괄적 극저온 법률 운영 구현
 */

export interface CryoLegalManager {
  // 법률 프레임워크 관리
  framework: LegalFrameworkManager;

  // 계약 운영
  contracts: ContractManager;

  // 분쟁 처리
  disputes: DisputeManager;

  // 준수 추적
  compliance: ComplianceManager;

  // 법률 보고
  reporting: ReportingManager;
}

export class CryoLegalManagerImpl implements CryoLegalManager {
  public readonly framework: LegalFrameworkManager;
  public readonly contracts: ContractManager;
  public readonly disputes: DisputeManager;
  public readonly compliance: ComplianceManager;
  public readonly reporting: ReportingManager;

  constructor(private readonly config: CryoLegalConfig) {
    this.framework = new LegalFrameworkManagerImpl(config);
    this.contracts = new ContractManagerImpl(config);
    this.disputes = new DisputeManagerImpl(config);
    this.compliance = new ComplianceManagerImpl(config);
    this.reporting = new ReportingManagerImpl(config);
  }

  async initialize(): Promise<void> {
    await Promise.all([
      this.framework.loadRegulations(),
      this.contracts.loadTemplates(),
      this.compliance.loadRequirements(),
    ]);
  }

  async shutdown(): Promise<void> {
    await Promise.all([
      this.framework.unload(),
      this.contracts.closeConnections(),
      this.disputes.archiveOpen(),
      this.compliance.generateFinalReport(),
    ]);
  }
}

export interface CryoLegalConfig {
  organization: OrganizationConfig;
  jurisdictions: JurisdictionConfig[];
  storage: StorageConfig;
  notifications: NotificationConfig;
  integrations: IntegrationConfig;
}

export interface OrganizationConfig {
  id: string;
  name: string;
  type: 'facility' | 'law-firm' | 'authority' | 'association';
  country: string;
  primaryContact: ContactConfig;
  legalCounsel?: LegalCounselConfig;
}

export interface ContactConfig {
  name: string;
  email: string;
  phone?: string;
  emergencyContact?: boolean;
}

export interface LegalCounselConfig {
  name: string;
  firm?: string;
  barNumber: string;
  specializations: string[];
  contact: ContactConfig;
}

export interface JurisdictionConfig {
  country: string;
  state?: string;
  isPrimary: boolean;
  regulatoryBodies: string[];
  applicableLaws: LawReference[];
}

export interface LawReference {
  name: string;
  code: string;
  version: string;
  effectiveDate: Date;
  url?: string;
}

export interface StorageConfig {
  type: 'postgresql' | 'mongodb' | 'sqlite';
  connectionString: string;
  encryptionKey?: string;
  retentionPeriodYears: number;
}

export interface NotificationConfig {
  enabled: boolean;
  channels: ('email' | 'sms' | 'push' | 'webhook')[];
  escalationMatrix: EscalationLevel[];
}

export interface EscalationLevel {
  level: number;
  trigger: string;
  recipients: string[];
  timeframeMins: number;
}

export interface IntegrationConfig {
  legalDatabase: boolean;
  courtFiling: boolean;
  compliancePortal: boolean;
  documentManagement: boolean;
}
```

## 조직 유형 및 역할

### 시설 조직 관리

```typescript
/**
 * 극저온 보존 법률 운영을 위한 조직 관리
 * 시설, 법률 사무소, 당국, 협회 지원
 */

export const OrganizationSchema = z.object({
  name: z.string().min(1).max(300),
  type: z.enum(['facility', 'law-firm', 'authority', 'association']),
  country: z.string().length(2),
  registrationNumber: z.string().optional(),
  contact: z.object({
    name: z.string().min(1),
    email: z.string().email(),
    phone: z.string().optional(),
  }),
  legalCounsel: z.object({
    name: z.string().min(1),
    firm: z.string().optional(),
    bar: z.string().min(1),
    contact: z.object({
      name: z.string().min(1),
      email: z.string().email(),
      phone: z.string().optional(),
    }),
  }).optional(),
});

export type Organization = z.infer<typeof OrganizationSchema>;

export class OrganizationManager {
  private organizations: Map<string, Organization> = new Map();
  private relationships: Map<string, OrganizationRelationship[]> = new Map();

  constructor(private readonly storage: OrganizationStorage) {}

  async registerOrganization(org: Organization): Promise<string> {
    const validated = OrganizationSchema.parse(org);

    // 고유 조직 ID 생성
    const orgId = this.generateOrganizationId(validated);

    // 등록번호 유효성 검사
    if (validated.registrationNumber) {
      await this.validateRegistrationNumber(
        validated.country,
        validated.registrationNumber
      );
    }

    // 법률 고문 변호사 자격 검증
    if (validated.legalCounsel) {
      await this.validateBarMembership(validated.legalCounsel);
    }

    // 조직 저장
    await this.storage.save(orgId, validated);
    this.organizations.set(orgId, validated);

    return orgId;
  }

  private generateOrganizationId(org: Organization): string {
    const typePrefix = {
      'facility': 'FAC',
      'law-firm': 'LAW',
      'authority': 'AUTH',
      'association': 'ASSOC',
    }[org.type];

    const timestamp = Date.now().toString(36).toUpperCase();
    const random = Math.random().toString(36).substring(2, 8).toUpperCase();

    return `${typePrefix}-${org.country}-${timestamp}-${random}`;
  }

  private async validateRegistrationNumber(
    country: string,
    regNumber: string
  ): Promise<void> {
    const validators: Record<string, RegExp> = {
      'US': /^\d{2}-\d{7}$/,           // EIN 형식
      'KR': /^\d{3}-\d{2}-\d{5}$/,     // 한국 사업자등록번호
      'GB': /^[A-Z]{2}\d{6}$/,          // 영국 회사 번호
      'DE': /^HRB\s?\d{5,6}$/,         // 독일 상업 등기
      'JP': /^\d{4}-\d{2}-\d{6}$/,     // 일본 법인 번호
    };

    const validator = validators[country];
    if (validator && !validator.test(regNumber)) {
      throw new Error(`${country}에 대한 잘못된 등록번호 형식`);
    }
  }

  private async validateBarMembership(
    counsel: NonNullable<Organization['legalCounsel']>
  ): Promise<void> {
    // 변호사 협회 API와 연동하여 자격 검증
    const barValidation = await this.checkBarDatabase(counsel.bar, counsel.name);

    if (!barValidation.isActive) {
      throw new Error(`변호사 자격 ${counsel.bar}이(가) 활성화되지 않음`);
    }

    if (barValidation.isDisbarred) {
      throw new Error(`${counsel.name} 변호사가 자격 박탈됨`);
    }
  }

  private async checkBarDatabase(
    barNumber: string,
    name: string
  ): Promise<BarValidation> {
    // 변호사 협회 데이터베이스 연동
    return {
      isActive: true,
      isDisbarred: false,
      memberSince: new Date('2010-01-01'),
      specializations: ['healthcare-law', 'bioethics'],
    };
  }

  async getOrganizationsByType(
    type: Organization['type']
  ): Promise<Organization[]> {
    const orgs: Organization[] = [];

    for (const [_, org] of this.organizations) {
      if (org.type === type) {
        orgs.push(org);
      }
    }

    return orgs;
  }

  async addRelationship(
    sourceOrgId: string,
    targetOrgId: string,
    relationshipType: RelationshipType
  ): Promise<void> {
    const relationship: OrganizationRelationship = {
      sourceOrgId,
      targetOrgId,
      type: relationshipType,
      establishedAt: new Date(),
      status: 'active',
    };

    const existing = this.relationships.get(sourceOrgId) || [];
    existing.push(relationship);
    this.relationships.set(sourceOrgId, existing);

    await this.storage.saveRelationship(relationship);
  }
}

export interface OrganizationRelationship {
  sourceOrgId: string;
  targetOrgId: string;
  type: RelationshipType;
  establishedAt: Date;
  terminatedAt?: Date;
  status: 'active' | 'suspended' | 'terminated';
}

export type RelationshipType =
  | 'legal-counsel'        // 법률 고문
  | 'regulatory-oversight' // 규제 감독
  | 'insurance-provider'   // 보험 제공자
  | 'affiliate'            // 계열사
  | 'subsidiary'           // 자회사
  | 'partner';             // 파트너

export interface BarValidation {
  isActive: boolean;
  isDisbarred: boolean;
  memberSince: Date;
  specializations: string[];
}

export interface OrganizationStorage {
  save(id: string, org: Organization): Promise<void>;
  load(id: string): Promise<Organization | null>;
  saveRelationship(rel: OrganizationRelationship): Promise<void>;
  findByType(type: Organization['type']): Promise<Organization[]>;
}
```

### 관할권 관리

```typescript
/**
 * 다중 관할권 법률 프레임워크 지원
 * 지역별 다양한 법적 요구사항 처리
 */

export const JurisdictionSchema = z.object({
  country: z.string().length(2),
  state: z.string().optional(),
  type: z.enum(['primary', 'secondary']),
  applicableLaws: z.array(z.string()).min(1),
});

export type Jurisdiction = z.infer<typeof JurisdictionSchema>;

export class JurisdictionManager {
  private jurisdictions: Map<string, JurisdictionDetails> = new Map();
  private lawDatabase: Map<string, Law> = new Map();

  constructor(private readonly config: JurisdictionConfig) {}

  async loadJurisdictions(): Promise<void> {
    // 구성된 모든 관할권 로드
    for (const juris of this.config.defaultJurisdictions) {
      await this.addJurisdiction(juris);
    }
  }

  async addJurisdiction(jurisdiction: Jurisdiction): Promise<string> {
    const validated = JurisdictionSchema.parse(jurisdiction);

    const jurisdictionId = this.generateJurisdictionId(validated);

    // 적용 가능한 법률 로드
    const laws = await this.loadApplicableLaws(validated.applicableLaws);

    const details: JurisdictionDetails = {
      ...validated,
      id: jurisdictionId,
      laws,
      lastUpdated: new Date(),
      complianceStatus: 'unknown',
    };

    this.jurisdictions.set(jurisdictionId, details);

    return jurisdictionId;
  }

  private generateJurisdictionId(jurisdiction: Jurisdiction): string {
    const parts = [jurisdiction.country];
    if (jurisdiction.state) {
      parts.push(jurisdiction.state);
    }
    return parts.join('-').toUpperCase();
  }

  private async loadApplicableLaws(lawCodes: string[]): Promise<Law[]> {
    const laws: Law[] = [];

    for (const code of lawCodes) {
      const law = await this.fetchLaw(code);
      if (law) {
        laws.push(law);
        this.lawDatabase.set(code, law);
      }
    }

    return laws;
  }

  private async fetchLaw(code: string): Promise<Law | null> {
    // 법률 데이터베이스 조회
    const lawDefinitions: Record<string, Law> = {
      'KR-BIOETHICS': {
        code: 'KR-BIOETHICS',
        name: '생명윤리 및 안전에 관한 법률',
        jurisdiction: 'KR',
        category: 'research',
        version: '2008',
        effectiveDate: new Date('2008-12-01'),
        requirements: [
          '인간 대상 연구 IRB 승인',
          '서면 동의서 문서화',
          '유전정보 보호',
          '연구 윤리 준수',
        ],
        penalties: [
          '5년 이하 징역',
          '5천만원 이하 벌금',
        ],
      },
      'KR-PIPA': {
        code: 'KR-PIPA',
        name: '개인정보 보호법',
        jurisdiction: 'KR',
        category: 'privacy',
        version: '2011',
        effectiveDate: new Date('2011-09-30'),
        requirements: [
          '수집/이용 동의',
          '목적 제한',
          '보안 조치',
          '국외 이전 제한',
        ],
        penalties: [
          '3억원 이하 과태료',
          '5년 이하 징역, 5천만원 이하 벌금',
        ],
      },
      'US-HIPAA': {
        code: 'US-HIPAA',
        name: '건강보험 이동성 및 책임에 관한 법률',
        jurisdiction: 'US',
        category: 'privacy',
        version: '1996',
        effectiveDate: new Date('1996-08-21'),
        requirements: [
          '보호 건강 정보 보호',
          '환자 기록 접근',
          '최소 필요 기준',
          '사업 제휴 계약',
        ],
        penalties: [
          '위반당 최대 $50,000 민사 벌금',
          '고의적 위반 시 형사 처벌',
        ],
      },
    };

    return lawDefinitions[code] || null;
  }

  async getApplicableLaws(
    jurisdictionId: string,
    category?: LawCategory
  ): Promise<Law[]> {
    const jurisdiction = this.jurisdictions.get(jurisdictionId);
    if (!jurisdiction) {
      return [];
    }

    let laws = jurisdiction.laws;

    if (category) {
      laws = laws.filter(law => law.category === category);
    }

    return laws;
  }

  async checkJurisdictionConflicts(
    jurisdictions: string[]
  ): Promise<JurisdictionConflict[]> {
    const conflicts: JurisdictionConflict[] = [];

    // 관할권 간 법률 충돌 비교
    for (let i = 0; i < jurisdictions.length; i++) {
      for (let j = i + 1; j < jurisdictions.length; j++) {
        const juris1 = this.jurisdictions.get(jurisdictions[i]);
        const juris2 = this.jurisdictions.get(jurisdictions[j]);

        if (juris1 && juris2) {
          const detected = await this.detectConflicts(juris1, juris2);
          conflicts.push(...detected);
        }
      }
    }

    return conflicts;
  }

  private async detectConflicts(
    juris1: JurisdictionDetails,
    juris2: JurisdictionDetails
  ): Promise<JurisdictionConflict[]> {
    const conflicts: JurisdictionConflict[] = [];

    // 개인정보 법률 충돌 확인
    const privacy1 = juris1.laws.filter(l => l.category === 'privacy');
    const privacy2 = juris2.laws.filter(l => l.category === 'privacy');

    if (privacy1.length > 0 && privacy2.length > 0) {
      // 국경 간 이전 제한 확인
      const hasGDPR = privacy1.some(l => l.code.includes('GDPR')) ||
                      privacy2.some(l => l.code.includes('GDPR'));

      if (hasGDPR) {
        conflicts.push({
          type: 'cross-border-transfer',
          jurisdictions: [juris1.id, juris2.id],
          description: 'GDPR 국경 간 데이터 이전 제한 적용 가능',
          severity: 'high',
          resolution: '표준 계약 조항 또는 적정성 결정 필요',
        });
      }
    }

    return conflicts;
  }
}

export interface JurisdictionDetails extends Jurisdiction {
  id: string;
  laws: Law[];
  lastUpdated: Date;
  complianceStatus: 'compliant' | 'non-compliant' | 'partial' | 'unknown';
}

export interface Law {
  code: string;
  name: string;
  jurisdiction: string;
  category: LawCategory;
  version: string;
  effectiveDate: Date;
  requirements: string[];
  penalties: string[];
}

export type LawCategory =
  | 'tissue-banking'       // 조직 은행
  | 'reproductive'         // 생식의학
  | 'research'             // 연구
  | 'transplantation'      // 이식
  | 'privacy'              // 개인정보
  | 'consumer-protection'; // 소비자 보호

export interface JurisdictionConflict {
  type: string;
  jurisdictions: string[];
  description: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  resolution: string;
}

export interface JurisdictionConfig {
  defaultJurisdictions: Jurisdiction[];
  autoLoadLaws: boolean;
  updateFrequency: 'daily' | 'weekly' | 'monthly';
}
```

## 프로젝트 상태 워크플로우

```typescript
/**
 * 법률 프로젝트 상태 관리
 * 극저온 법률 프로젝트의 생명주기 추적
 */

export type ProjectStatus = 'active' | 'pending' | 'suspended' | 'archived';

export class ProjectStatusManager {
  private statusHistory: Map<string, StatusTransition[]> = new Map();
  private validTransitions: Map<ProjectStatus, ProjectStatus[]> = new Map([
    ['pending', ['active', 'archived']],
    ['active', ['suspended', 'archived']],
    ['suspended', ['active', 'archived']],
    ['archived', []], // 최종 상태
  ]);

  constructor(private readonly storage: StatusStorage) {}

  async transitionStatus(
    projectId: string,
    currentStatus: ProjectStatus,
    newStatus: ProjectStatus,
    reason: string,
    actor: string
  ): Promise<StatusTransitionResult> {
    // 전환 유효성 검사
    const validTargets = this.validTransitions.get(currentStatus) || [];

    if (!validTargets.includes(newStatus)) {
      return {
        success: false,
        error: `${currentStatus}에서 ${newStatus}로 전환할 수 없음`,
        validTransitions: validTargets,
      };
    }

    // 상태에 따른 추가 검증
    const validation = await this.validateTransition(
      projectId,
      currentStatus,
      newStatus
    );

    if (!validation.allowed) {
      return {
        success: false,
        error: validation.reason!,
      };
    }

    // 전환 기록
    const transition: StatusTransition = {
      id: crypto.randomUUID(),
      projectId,
      fromStatus: currentStatus,
      toStatus: newStatus,
      reason,
      actor,
      timestamp: new Date(),
    };

    await this.storage.saveTransition(transition);

    // 상태별 작업 실행
    await this.executeStatusActions(transition);

    return {
      success: true,
      transition,
    };
  }

  private async validateTransition(
    projectId: string,
    from: ProjectStatus,
    to: ProjectStatus
  ): Promise<TransitionValidation> {
    // 보관 전 보류 중인 법적 조치 확인
    if (to === 'archived') {
      const pendingActions = await this.checkPendingActions(projectId);
      if (pendingActions.length > 0) {
        return {
          allowed: false,
          reason: `보관 불가: ${pendingActions.length}개의 보류 중인 법적 조치`,
        };
      }
    }

    // 정지 전 활성 분쟁 확인
    if (to === 'suspended') {
      const activeDisputes = await this.checkActiveDisputes(projectId);
      if (activeDisputes.length > 0) {
        return {
          allowed: false,
          reason: `정지 불가: ${activeDisputes.length}개의 활성 분쟁이 처리 필요`,
        };
      }
    }

    return { allowed: true };
  }

  private async executeStatusActions(transition: StatusTransition): Promise<void> {
    switch (transition.toStatus) {
      case 'active':
        await this.activateProject(transition.projectId);
        break;
      case 'suspended':
        await this.suspendProject(transition.projectId, transition.reason);
        break;
      case 'archived':
        await this.archiveProject(transition.projectId);
        break;
    }
  }

  private async activateProject(projectId: string): Promise<void> {
    // 모든 법률 워크플로우 활성화
    // 준수 모니터링 재개
    // 이해관계자 알림
  }

  private async suspendProject(projectId: string, reason: string): Promise<void> {
    // 비중요 워크플로우 일시 중지
    // 필수 준수 유지
    // 법률 고문 알림
  }

  private async archiveProject(projectId: string): Promise<void> {
    // 아카이브 스토리지로 이전
    // 읽기 전용 접근 유지
    // 보존 일정 설정
  }

  private async checkPendingActions(projectId: string): Promise<string[]> {
    return [];
  }

  private async checkActiveDisputes(projectId: string): Promise<string[]> {
    return [];
  }
}

export interface StatusTransition {
  id: string;
  projectId: string;
  fromStatus: ProjectStatus;
  toStatus: ProjectStatus;
  reason: string;
  actor: string;
  timestamp: Date;
}

export interface StatusTransitionResult {
  success: boolean;
  error?: string;
  validTransitions?: ProjectStatus[];
  transition?: StatusTransition;
}

export interface TransitionValidation {
  allowed: boolean;
  reason?: string;
}

export interface StatusStorage {
  saveTransition(transition: StatusTransition): Promise<void>;
  getHistory(projectId: string): Promise<StatusTransition[]>;
}
```

---

## 장 탐색

| 장 | 제목 | 설명 |
|---|------|------|
| 01 | 표지 및 소개 | 표준 개요 및 핵심 개념 |
| 02 | 시장 분석 | 산업 현황 및 규제 환경 |
| 03 | 데이터 형식 | Zod 스키마 및 TypeScript 타입 |
| 04 | API 인터페이스 | REST, GraphQL, WebSocket API |
| 05 | 계약 관리 | 템플릿, 조항, 생명주기 |
| 06 | 분쟁 해결 | 메커니즘 및 절차 |
| 07 | 보안 및 준수 | 데이터 보호 및 감사 |
| 08 | 구현 | 배포 및 통합 |
| 09 | 미래 트렌드 | 신흥 기술 |

---

**문서 버전**: 1.0.0
**최종 업데이트**: 2025
**표준**: WIA-CRYO-LEGAL
**철학**: 弘益人間 (홍익인간)
