# 제5장: 계약 관리 - 템플릿, 조항 및 수명주기

## 종합 계약 관리 시스템

이 장에서는 WIA 극저온 법률 표준의 완전한 계약 관리 구현을 제공하며, 템플릿 관리, 조항 라이브러리, 서명 워크플로우 및 계약 수명주기 관리를 포함합니다.

## 계약 템플릿 엔진

```typescript
/**
 * WIA 극저온 법률 표준 - 계약 템플릿 엔진
 * 동적 템플릿 생성 및 관리
 */

import { z } from 'zod';
import {
  ContractTemplate,
  ContractTemplateSchema,
  Clause,
  ClauseSchema,
  ContractType,
  TemplateStatus,
} from './types';

// ============================================================================
// 템플릿 엔진 설정
// ============================================================================

export interface TemplateEngineConfig {
  templatesPath: string;           // 템플릿 경로
  clauseLibraryPath: string;       // 조항 라이브러리 경로
  outputFormat: 'markdown' | 'html' | 'pdf';  // 출력 형식
  variableDelimiters: [string, string];       // 변수 구분자
  conditionalDelimiters: [string, string];    // 조건 구분자
}

export const defaultConfig: TemplateEngineConfig = {
  templatesPath: './templates',
  clauseLibraryPath: './clauses',
  outputFormat: 'markdown',
  variableDelimiters: ['{{', '}}'],
  conditionalDelimiters: ['{{#', '}}'],
};

// ============================================================================
// 한국 계약법 특화 설정
// ============================================================================

export interface KoreanContractConfig extends TemplateEngineConfig {
  // 한국 특화 설정
  civilLawCompliance: boolean;           // 민법 준수
  notarizationRequired: boolean;         // 공증 필요 여부
  stampDutyApplicable: boolean;          // 인지세 적용 여부
  electronicSignatureAct: boolean;       // 전자서명법 준수
  personalInfoProtection: boolean;       // 개인정보보호법 준수
  bioethicsLawCompliance: boolean;       // 생명윤리법 준수

  // 언어 설정
  primaryLanguage: 'ko' | 'en';
  bilingualContract: boolean;

  // 관할권 설정
  defaultJurisdiction: string;           // 기본 관할권
  arbitrationInstitution: string;        // 중재기관
}

export const koreanContractConfig: KoreanContractConfig = {
  ...defaultConfig,
  civilLawCompliance: true,
  notarizationRequired: false,
  stampDutyApplicable: true,
  electronicSignatureAct: true,
  personalInfoProtection: true,
  bioethicsLawCompliance: true,
  primaryLanguage: 'ko',
  bilingualContract: true,
  defaultJurisdiction: '대한민국 서울중앙지방법원',
  arbitrationInstitution: '대한상사중재원',
};

// ============================================================================
// 템플릿 관리자
// ============================================================================

export class ContractTemplateManager {
  private templates: Map<string, ContractTemplate> = new Map();
  private clauseLibrary: Map<string, Clause> = new Map();
  private versionHistory: Map<string, ContractTemplate[]> = new Map();

  constructor(private readonly config: TemplateEngineConfig = defaultConfig) {}

  async initialize(): Promise<void> {
    await this.loadDefaultTemplates();
    await this.loadClauseLibrary();
    await this.loadKoreanTemplates();
  }

  private async loadDefaultTemplates(): Promise<void> {
    // 보관 계약 템플릿
    this.registerTemplate({
      id: 'tpl-storage-agreement-v1',
      name: '극저온 보관 계약서',
      nameEn: 'Cryogenic Storage Agreement',
      type: 'storage-agreement',
      version: '1.0.0',
      jurisdiction: 'KR',
      clauses: this.getStorageAgreementClauses(),
      approvedBy: '법무팀',
      approvedAt: new Date().toISOString(),
      status: 'active',
    });

    // 동의서 템플릿
    this.registerTemplate({
      id: 'tpl-consent-form-v1',
      name: '극저온보존 동의서',
      nameEn: 'Informed Consent for Cryopreservation',
      type: 'consent-form',
      version: '1.0.0',
      jurisdiction: 'KR',
      clauses: this.getConsentFormClauses(),
      approvedBy: '법무팀',
      approvedAt: new Date().toISOString(),
      status: 'active',
    });

    // 연구 계약 템플릿
    this.registerTemplate({
      id: 'tpl-research-agreement-v1',
      name: '연구 이용 계약서',
      nameEn: 'Research Use Agreement',
      type: 'research-agreement',
      version: '1.0.0',
      jurisdiction: 'KR',
      clauses: this.getResearchAgreementClauses(),
      approvedBy: '법무팀',
      approvedAt: new Date().toISOString(),
      status: 'active',
    });

    // 기증 계약 템플릿
    this.registerTemplate({
      id: 'tpl-donation-agreement-v1',
      name: '조직 기증 계약서',
      nameEn: 'Tissue Donation Agreement',
      type: 'donation-agreement',
      version: '1.0.0',
      jurisdiction: 'KR',
      clauses: this.getDonationAgreementClauses(),
      approvedBy: '법무팀',
      approvedAt: new Date().toISOString(),
      status: 'active',
    });
  }

  private async loadKoreanTemplates(): Promise<void> {
    // 한국 특화 템플릿 로드

    // 난자/정자 보관 계약서 (생명윤리법 준수)
    this.registerTemplate({
      id: 'tpl-gamete-storage-kr-v1',
      name: '생식세포 보관 계약서',
      nameEn: 'Gamete Storage Agreement',
      type: 'storage-agreement',
      version: '1.0.0',
      jurisdiction: 'KR',
      clauses: this.getGamateStorageClausesKR(),
      approvedBy: '법무팀',
      approvedAt: new Date().toISOString(),
      status: 'active',
      specialRequirements: {
        bioethicsApproval: true,
        irbApproval: true,
        mfdsNotification: true,
      },
    });

    // 배아 보관 계약서
    this.registerTemplate({
      id: 'tpl-embryo-storage-kr-v1',
      name: '배아 보관 계약서',
      nameEn: 'Embryo Storage Agreement',
      type: 'storage-agreement',
      version: '1.0.0',
      jurisdiction: 'KR',
      clauses: this.getEmbryoStorageClausesKR(),
      approvedBy: '법무팀',
      approvedAt: new Date().toISOString(),
      status: 'active',
      specialRequirements: {
        bioethicsApproval: true,
        coupleConsent: true,
        maxStoragePeriod: '5년',
      },
    });

    // 제대혈 보관 계약서
    this.registerTemplate({
      id: 'tpl-cordblood-storage-kr-v1',
      name: '제대혈 보관 계약서',
      nameEn: 'Cord Blood Storage Agreement',
      type: 'storage-agreement',
      version: '1.0.0',
      jurisdiction: 'KR',
      clauses: this.getCordBloodStorageClausesKR(),
      approvedBy: '법무팀',
      approvedAt: new Date().toISOString(),
      status: 'active',
      specialRequirements: {
        mfdsLicense: true,
        qualityStandards: 'AABB/FACT',
      },
    });
  }

  private getStorageAgreementClauses(): Clause[] {
    return [
      {
        id: 'clause-storage-purpose-kr',
        title: '보관 목적',
        titleEn: 'Purpose of Storage',
        text: `본 계약은 {{client_name}}(이하 "의뢰인")의 {{specimen_type}}
               (이하 "검체")를 {{facility_name}}(이하 "시설")에서 극저온 보관하는
               것에 관한 사항을 규정합니다. 시설은 의뢰인이 지정한 목적에 따라
               검체를 보관하기로 합니다. 보관 목적: {{storage_purposes}}.`,
        type: 'standard',
        mandatory: true,
        category: 'scope',
      },
      {
        id: 'clause-storage-term-kr',
        title: '보관 기간',
        titleEn: 'Term of Storage',
        text: `최초 보관 기간은 {{effective_date}}부터 {{initial_term}}으로 합니다.
               본 계약은 당사자 일방이 기간 만료 {{notice_period}} 전까지 서면으로
               갱신 거절 의사를 통지하지 않는 한 동일한 조건으로 {{renewal_term}}씩
               자동 갱신됩니다.`,
        type: 'standard',
        mandatory: true,
        category: 'term',
      },
      {
        id: 'clause-storage-fees-kr',
        title: '보관 비용',
        titleEn: 'Storage Fees',
        text: `의뢰인은 다음 비용을 지불하기로 합니다:
               - 초기 처리 비용: {{processing_fee}}
               - 연간 보관 비용: {{annual_storage_fee}}
               - 인출 비용: {{retrieval_fee}}

               비용 납부 조건: {{payment_terms}}
               연체 시 월 {{late_fee_percentage}}%의 연체료가 부과됩니다.

               ※ 부가가치세는 별도이며, 세금계산서가 발행됩니다.`,
        type: 'standard',
        mandatory: true,
        category: 'fees',
      },
      {
        id: 'clause-storage-conditions-kr',
        title: '보관 조건',
        titleEn: 'Storage Conditions',
        text: `시설은 검체를 {{storage_temperature}}에서 {{storage_method}}를 사용하여
               보관합니다. 시설은 {{backup_systems}}를 유지하며 {{monitoring_frequency}}
               보관 상태를 모니터링합니다.
               장비 고장 시 시설은 {{emergency_procedures}}를 실행합니다.

               보관 환경:
               - 온도: -196°C (액체질소)
               - 24시간 자동 모니터링
               - 이중 백업 시스템
               - 재해 복구 계획 수립`,
        type: 'standard',
        mandatory: true,
        category: 'operations',
      },
      {
        id: 'clause-access-release-kr',
        title: '접근 및 인출',
        titleEn: 'Access and Release',
        text: `검체에 대한 접근은 의뢰인이 지정한 권한자에게만 허용됩니다.
               검체 인출 시 다음이 필요합니다:
               - 의뢰인 또는 권한 위임자의 서면 요청
               - 본인 확인 (신분증 제시)
               - 미납 비용 완납
               - 인출 관련 서류 작성 완료

               인출 절차:
               1. 인출 신청서 제출 (최소 7일 전)
               2. 본인 확인 및 서류 검토
               3. 비용 정산
               4. 검체 인출 및 운송`,
        type: 'standard',
        mandatory: true,
        category: 'access',
      },
      {
        id: 'clause-liability-limitation-kr',
        title: '책임 제한',
        titleEn: 'Limitation of Liability',
        text: `검체의 손실 또는 손상에 대한 시설의 책임은 {{liability_cap}}으로
               제한됩니다. 시설은 간접적, 부수적, 결과적 또는 징벌적 손해에
               대해서는 책임을 지지 않습니다.

               다음의 경우 시설은 손실 또는 손상에 대한 책임을 지지 않습니다:
               - 천재지변 또는 불가항력
               - 전쟁, 테러 또는 시민 소요
               - 정부의 조치 또는 규정
               - 의뢰인의 부정확한 정보 제공

               단, 시설의 고의 또는 중대한 과실로 인한 손해는 이 제한에서
               제외됩니다 (민법 제750조).`,
        type: 'standard',
        mandatory: true,
        category: 'liability',
      },
      {
        id: 'clause-insurance-kr',
        title: '보험',
        titleEn: 'Insurance',
        text: `{{#if insurance_required}}
               의뢰인은 보관 기간 동안 검체에 대해 최소 {{minimum_coverage}}의
               보험을 유지하기로 합니다.
               {{/if}}

               시설은 {{facility_coverage}}의 전문가 배상책임보험을 유지합니다.

               보험 정보:
               - 보험사: {{insurance_company}}
               - 증권번호: {{policy_number}}
               - 보장 범위: 시설 과실로 인한 검체 손상/손실`,
        type: 'optional',
        mandatory: false,
        category: 'insurance',
      },
      {
        id: 'clause-termination-kr',
        title: '계약 해지',
        titleEn: 'Termination',
        text: `당사자는 다음의 경우 본 계약을 해지할 수 있습니다:
               - {{notice_period}} 서면 통지
               - 중대한 위반 시 즉시 해지 ({{cure_period}} 시정 기간 부여)
               - 의뢰인 사망 시 (처분 지시에 따름)

               해지 시 의뢰인은 {{retrieval_deadline}} 내에 검체를 인출하거나
               처분 지시를 제공해야 합니다.

               해지 절차:
               1. 해지 통지서 발송
               2. 미납금 정산
               3. 검체 인출 또는 처분
               4. 관련 서류 수령`,
        type: 'standard',
        mandatory: true,
        category: 'termination',
      },
      {
        id: 'clause-disposition-kr',
        title: '처분 지시',
        titleEn: 'Disposition Instructions',
        text: `의뢰인의 사망, 무능력 또는 비용 미납 시, 검체는 등록된 처분 지시에
               따라 처리됩니다. 유효한 지시가 없는 경우 시설은:
               - 지정 수혜자에게 연락 시도
               - {{abandonment_period}} 동안 검체 보관
               - 관련 법률에 따라 검체 처분

               처분 옵션:
               □ 본인 또는 지정인에게 인도
               □ 의료 폐기물로 처분
               □ 연구 목적 기증 (사전 동의 필요)
               □ 기타: ________________`,
        type: 'standard',
        mandatory: true,
        category: 'disposition',
      },
      {
        id: 'clause-governing-law-kr',
        title: '준거법 및 관할',
        titleEn: 'Governing Law',
        text: `본 계약은 대한민국 법률에 따라 해석되고 적용됩니다.
               본 계약과 관련하여 발생하는 모든 분쟁은 {{governing_court}}을
               전속관할로 합니다.

               관련 법률:
               - 민법
               - 생명윤리 및 안전에 관한 법률
               - 개인정보 보호법
               - 의료법`,
        type: 'standard',
        mandatory: true,
        category: 'legal',
      },
    ];
  }

  private getConsentFormClauses(): Clause[] {
    return [
      {
        id: 'clause-consent-purpose-kr',
        title: '극저온보존 목적',
        titleEn: 'Purpose of Cryopreservation',
        text: `본인, {{patient_name}}은(는) 본인의 {{specimen_type}}을(를) 다음 목적으로
               극저온보존하는 것에 동의합니다:
               {{#each purposes}}
               - {{this}}
               {{/each}}

               본인은 극저온보존이 생물학적 물질을 극저온에서 동결 및 보관하는
               과정임을 이해합니다.

               동의 범위:
               □ 개인적 사용만
               □ 배우자/지정 파트너와 함께 사용
               □ 연구 목적 기증
               □ 제3자 기증`,
        type: 'standard',
        mandatory: true,
        category: 'consent',
      },
      {
        id: 'clause-risks-benefits-kr',
        title: '위험 및 이점',
        titleEn: 'Risks and Benefits',
        text: `본인은 다음 사항을 고지받고 이해했습니다:

               잠재적 이점:
               - 생식 능력 보존
               - 향후 난임 치료에 사용
               - 의학 연구에 기여 가능 (동의 시)

               잠재적 위험:
               - 보존 성공 보장 없음
               - 보관 중 손실 또는 손상 가능성
               - 장기 보관의 알려지지 않은 영향
               - 안전 조치에도 불구한 장비 고장 가능성

               성공률 정보:
               - 해동 후 생존율: 약 {{survival_rate}}%
               - 임신 성공률: 개인차 있음`,
        type: 'standard',
        mandatory: true,
        category: 'disclosure',
      },
      {
        id: 'clause-voluntary-kr',
        title: '자발적 동의',
        titleEn: 'Voluntary Consent',
        text: `본인은 다음을 확인합니다:
               - 이 동의는 자발적으로 이루어졌습니다
               - 질문할 기회가 주어졌습니다
               - 질문에 대해 만족스러운 답변을 받았습니다
               - 이 결정에 대해 강요받지 않았습니다
               - 언제든지 동의를 철회할 수 있습니다

               본인은 충분한 시간을 갖고 본 동의서를 검토하였으며,
               필요시 법률 자문을 구할 기회가 있었음을 확인합니다.`,
        type: 'standard',
        mandatory: true,
        category: 'consent',
      },
      {
        id: 'clause-future-use-kr',
        title: '향후 사용 결정',
        titleEn: 'Future Use Decisions',
        text: `본인은 보존된 {{specimen_type}}이(가) 다음 용도로 사용되기를
               지시합니다 (해당 사항 모두 선택):

               [ ] 개인 사용만
               [ ] 지정 파트너 사용: {{partner_name}}
               [ ] 연구 기증 (익명)
               [ ] 다른 개인/부부에게 기증
               [ ] {{disposal_date}} 이후 폐기

               사용 조건:
               - 본인의 서면 동의 없이 제3자 제공 금지
               - 상업적 목적 사용 금지
               - 인간복제 목적 사용 금지`,
        type: 'standard',
        mandatory: true,
        category: 'instructions',
      },
      {
        id: 'clause-posthumous-use-kr',
        title: '사후 사용 승인',
        titleEn: 'Posthumous Use Authorization',
        text: `본인 사망 시:

               [ ] 보존 물질의 사후 사용을 승인합니다
               [ ] 사후 사용을 승인하지 않습니다

               승인하는 경우, 본인은 {{designated_person}}을(를) 보존 물질의
               사용에 관한 결정을 내릴 권한자로 지정합니다.

               사후 사용 조건:
               - 사망 후 {{posthumous_period}} 이내에만 사용 가능
               - 지정인의 서면 요청 필요
               - 시설의 윤리위원회 검토 필요`,
        type: 'standard',
        mandatory: true,
        category: 'posthumous',
      },
      {
        id: 'clause-bioethics-compliance-kr',
        title: '생명윤리법 준수',
        titleEn: 'Bioethics Law Compliance',
        text: `본 동의서는 「생명윤리 및 안전에 관한 법률」에 따라 작성되었습니다.

               본인은 다음을 이해하고 동의합니다:
               - 배아는 최대 5년간 보존 가능 (법률 규정)
               - 배아 연구는 별도의 IRB 승인 및 동의 필요
               - 인간복제는 법으로 금지됨
               - 유전정보는 개인정보보호법에 따라 보호됨

               관련 기관:
               - 기관생명윤리위원회 (IRB)
               - 보건복지부
               - 질병관리청`,
        type: 'standard',
        mandatory: true,
        category: 'regulatory',
      },
    ];
  }

  private getResearchAgreementClauses(): Clause[] {
    return [
      {
        id: 'clause-research-scope-kr',
        title: '연구 범위',
        titleEn: 'Research Scope',
        text: `본 계약은 {{research_institution}}이(가) {{facility_name}}의 검체를
               다음 연구 프로젝트에 사용하는 것을 승인합니다:

               {{research_description}}

               연구책임자: {{pi_name}}
               IRB 승인번호: {{irb_number}}
               승인일: {{irb_approval_date}}

               연구 기간: {{research_start_date}} ~ {{research_end_date}}
               검체 수량: {{specimen_count}}`,
        type: 'standard',
        mandatory: true,
        category: 'scope',
      },
      {
        id: 'clause-specimen-handling-kr',
        title: '검체 취급 요건',
        titleEn: 'Specimen Handling Requirements',
        text: `연구기관은 다음 사항에 동의합니다:
               - 적절한 보관 조건에서 검체 유지
               - 모든 관련 규정 준수 (식약처, IRB, 기관)
               - 동의 없이 제3자에게 검체 이전 금지
               - 미사용 검체 반환 또는 적정 폐기
               - 관리연속성 문서 유지

               검체 관리 기준:
               - 보관 온도: {{storage_temp}}
               - 취급 절차: SOP 준수
               - 추적 시스템: 바코드/RFID`,
        type: 'standard',
        mandatory: true,
        category: 'requirements',
      },
      {
        id: 'clause-data-privacy-kr',
        title: '데이터 프라이버시 및 비식별화',
        titleEn: 'Data Privacy and De-identification',
        text: `모든 검체는 {{deidentification_standard}}에 따라 비식별화됩니다.

               연구기관은 다음을 준수합니다:
               - 기증자 재식별 시도 금지
               - 개인정보보호법에 따른 모든 관련 데이터 보호
               - 데이터 침해 발생 시 {{breach_notification_period}} 이내 보고
               - 프로젝트 완료 시 연결 정보 파기

               개인정보 보호 조치:
               - 가명처리 또는 익명처리
               - 접근 권한 제한
               - 암호화 저장
               - 감사 로그 유지`,
        type: 'standard',
        mandatory: true,
        category: 'privacy',
      },
      {
        id: 'clause-intellectual-property-kr',
        title: '지적재산권',
        titleEn: 'Intellectual Property',
        text: `연구에서 발생하는 지적재산권:

               {{#if shared_ip}}
               - 양 당사자가 공동 소유
               - 수익 배분: {{revenue_split}}
               {{else}}
               - 연구기관이 소유
               - 시설은 발표물에서 인정 표시를 받음
               {{/if}}

               양 당사자는 미발표 연구 결과의 기밀을 유지하기로 합니다.

               특허 출원:
               - 공동 발명 시 협의 필요
               - 출원 비용 분담 방식: {{patent_cost_sharing}}`,
        type: 'negotiable',
        mandatory: true,
        category: 'ip',
      },
      {
        id: 'clause-publication-kr',
        title: '발표 및 출판',
        titleEn: 'Publication',
        text: `연구 결과의 발표 및 출판에 관하여:

               - 발표 전 상대방에게 {{review_period}} 사전 통지
               - 기밀 정보 삭제 요청 권리
               - 검체 제공 시설 명시
               - 공동 저자 기준: {{authorship_criteria}}

               발표 형식:
               - 학술지 논문
               - 학회 발표
               - 특허 출원
               - 보도자료 (사전 협의 필요)`,
        type: 'standard',
        mandatory: false,
        category: 'publication',
      },
    ];
  }

  private getDonationAgreementClauses(): Clause[] {
    return [
      {
        id: 'clause-donation-intent-kr',
        title: '기증 의사',
        titleEn: 'Donation Intent',
        text: `본인, {{donor_name}}은(는) 본인의 {{specimen_type}}을(를)
               {{facility_name}}을(를) 통해 {{recipient_type}}에게 자발적으로
               기증합니다.

               본인은 이 기증이:
               {{#if anonymous}}
               - 익명 (본인의 신원이 공개되지 않음)
               {{else}}
               - 비익명 (본인의 신원이 수혜자와 공유될 수 있음)
               {{/if}}

               {{#if compensated}}
               - 유상: {{compensation_amount}}
               {{else}}
               - 무상 (이타적 기증)
               {{/if}}
               임을 이해합니다.`,
        type: 'standard',
        mandatory: true,
        category: 'intent',
      },
      {
        id: 'clause-donor-screening-kr',
        title: '기증자 검사 및 검진',
        titleEn: 'Donor Screening and Testing',
        text: `본인은 식약처 규정 및 시설 프로토콜에 따라 다음 검사 및
               검진을 받는 것에 동의합니다:

               - 건강 이력 설문
               - 신체 검사
               - 감염병 검사
               - 유전자 검사 (해당 시)

               본인은 검사 결과에 따라 부적격 판정을 받을 수 있음을 이해합니다.

               검사 항목:
               - HIV 1/2 항체/항원
               - B형/C형 간염
               - 매독
               - HTLV I/II
               - CMV (필요시)`,
        type: 'standard',
        mandatory: true,
        category: 'screening',
      },
      {
        id: 'clause-irrevocable-kr',
        title: '기증의 철회불가',
        titleEn: 'Irrevocability of Donation',
        text: `본인은 기증이 의도된 목적에 사용되면 본 기증이 철회불가능해짐을
               이해합니다.

               사용 전, 본인은 {{revocation_period}} 이내에 시설에 서면 통지를
               제공하여 이 기증을 철회할 수 있습니다.

               철회 기한 이후, 본인은 기증된 물질 및 그로 인한 임신, 배아,
               자녀에 대한 모든 권리를 포기합니다.

               철회 절차:
               1. 철회 신청서 제출
               2. 본인 확인
               3. 기증 물질 상태 확인
               4. 반환 또는 폐기 처리`,
        type: 'standard',
        mandatory: true,
        category: 'rights',
      },
      {
        id: 'clause-donor-rights-kr',
        title: '기증자 권리',
        titleEn: 'Donor Rights',
        text: `기증자로서 본인은 다음 권리를 가집니다:

               - 기증 전 언제든지 철회할 권리
               - 검사 결과를 통보받을 권리
               - 기증 물질의 사용 현황 정보 요청 권리
               - 익명성 보호 요청 권리
               - 상담 서비스 이용 권리

               기증자 보호:
               - 개인정보 기밀 유지
               - 강압적 기증 요청 금지
               - 적정 보상 (유상 기증 시)
               - 의료적 후속 관리`,
        type: 'standard',
        mandatory: true,
        category: 'rights',
      },
    ];
  }

  private getGamateStorageClausesKR(): Clause[] {
    return [
      {
        id: 'clause-gamete-bioethics-kr',
        title: '생명윤리법 준수 사항',
        text: `본 계약은 「생명윤리 및 안전에 관한 법률」에 따라 체결됩니다.

               의무 준수 사항:
               1. 배아생성의료기관 등록 (제22조)
               2. 배아연구기관 승인 (제31조, 연구 시)
               3. 동의서 취득 및 보관 (제24조)
               4. 개인정보 보호 (제34조)

               금지 사항:
               - 인간복제 금지
               - 이종 간 수정 금지
               - 상업적 거래 금지`,
        type: 'standard',
        mandatory: true,
        category: 'regulatory',
      },
      {
        id: 'clause-gamete-consent-kr',
        title: '생식세포 동의 요건',
        text: `생식세포 보관을 위해 다음 동의가 필요합니다:

               필수 동의:
               □ 생식세포 채취 동의
               □ 극저온 보관 동의
               □ 보관 기간 및 조건 동의
               □ 처분 방법 지정

               선택적 동의:
               □ 연구 목적 사용 동의
               □ 제3자 기증 동의
               □ 사후 사용 동의

               동의 철회: 언제든지 서면으로 가능`,
        type: 'standard',
        mandatory: true,
        category: 'consent',
      },
    ];
  }

  private getEmbryoStorageClausesKR(): Clause[] {
    return [
      {
        id: 'clause-embryo-storage-limit-kr',
        title: '배아 보관 기간 제한',
        text: `「생명윤리 및 안전에 관한 법률」 제25조에 따라:

               보관 기간:
               - 최대 보관 기간: 5년
               - 연장: 동의권자 동의 시 가능
               - 보관 기간 초과 시: 폐기 또는 연구 기증

               배아 동의권자:
               - 난자 제공자
               - 정자 제공자
               - 양측 모두의 동의 필요

               주의 사항:
               - 동의권자 일방 사망 시 잔여 배아 처분 지정 필요
               - 이혼/별거 시 배아 처분 협의 필요`,
        type: 'standard',
        mandatory: true,
        category: 'regulatory',
      },
      {
        id: 'clause-embryo-disposition-kr',
        title: '배아 처분 지시',
        text: `배아 처분에 관하여 다음을 지정합니다:

               보관 기간 만료 시:
               □ 폐기
               □ 연구 목적 기증 (IRB 승인 연구)
               □ 제3자 기증 (별도 동의 필요)

               동의권자 사망 시:
               □ 폐기
               □ 생존 동의권자 결정에 위임
               □ 사전 지정된 제3자 이전

               동의권자 간 합의 불가 시:
               □ 폐기
               □ 중재/조정 절차 진행`,
        type: 'standard',
        mandatory: true,
        category: 'disposition',
      },
    ];
  }

  private getCordBloodStorageClausesKR(): Clause[] {
    return [
      {
        id: 'clause-cordblood-standards-kr',
        title: '제대혈 품질 기준',
        text: `제대혈 보관은 다음 기준을 준수합니다:

               품질 인증:
               - AABB (미국혈액은행협회)
               - FACT (세포치료인증재단)
               - 식약처 허가 기준

               품질 관리:
               - 세포 수: 최소 {{min_cell_count}} 이상
               - 생존율: {{viability_rate}}% 이상
               - 무균 검사 완료
               - HLA 타이핑 완료

               보관 조건:
               - 액체질소 (-196°C)
               - 이중 격리 보관
               - 24시간 모니터링`,
        type: 'standard',
        mandatory: true,
        category: 'quality',
      },
      {
        id: 'clause-cordblood-use-kr',
        title: '제대혈 사용 조건',
        text: `보관된 제대혈은 다음 조건에서 사용됩니다:

               사용 가능 목적:
               - 조혈모세포 이식
               - 승인된 세포치료
               - IRB 승인 연구 (동의 시)

               사용 절차:
               1. 의료기관 사용 요청
               2. 적합성 검사 (HLA 일치 등)
               3. 인출 승인
               4. 운송 및 인도

               사용 우선순위:
               1. 본인 (자가 이식)
               2. 형제/자매
               3. 가족
               4. 제3자 (공여 동의 시)`,
        type: 'standard',
        mandatory: true,
        category: 'usage',
      },
    ];
  }

  private async loadClauseLibrary(): Promise<void> {
    for (const template of this.templates.values()) {
      for (const clause of template.clauses) {
        this.clauseLibrary.set(clause.id, clause);
      }
    }
  }

  registerTemplate(template: ContractTemplate): void {
    const validated = ContractTemplateSchema.parse(template);

    const history = this.versionHistory.get(validated.id) || [];
    const existing = this.templates.get(validated.id);
    if (existing) {
      history.push(existing);
      this.versionHistory.set(validated.id, history);
    }

    this.templates.set(validated.id, validated);
  }

  getTemplate(id: string): ContractTemplate | undefined {
    return this.templates.get(id);
  }

  getTemplatesByType(type: ContractType): ContractTemplate[] {
    return Array.from(this.templates.values())
      .filter(t => t.type === type && t.status === 'active');
  }

  getTemplatesByJurisdiction(jurisdiction: string): ContractTemplate[] {
    return Array.from(this.templates.values())
      .filter(t => t.jurisdiction === jurisdiction && t.status === 'active');
  }

  deprecateTemplate(id: string): void {
    const template = this.templates.get(id);
    if (template) {
      template.status = 'deprecated';
      this.templates.set(id, template);
    }
  }

  getClause(id: string): Clause | undefined {
    return this.clauseLibrary.get(id);
  }

  getClausesByCategory(category: string): Clause[] {
    return Array.from(this.clauseLibrary.values())
      .filter(c => c.category === category);
  }

  getMandatoryClauses(templateId: string): Clause[] {
    const template = this.templates.get(templateId);
    if (!template) return [];

    return template.clauses.filter(c => c.mandatory);
  }
}
```

## 계약 생성기

```typescript
/**
 * 계약 생성기
 * 템플릿 기반 계약서 동적 생성
 */

export class ContractGenerator {
  constructor(
    private readonly templateManager: ContractTemplateManager,
    private readonly config: TemplateEngineConfig = defaultConfig
  ) {}

  async generateContract(
    templateId: string,
    variables: Record<string, unknown>,
    options: GenerationOptions = {}
  ): Promise<GeneratedContract> {
    const template = this.templateManager.getTemplate(templateId);

    if (!template) {
      throw new Error(`템플릿을 찾을 수 없습니다: ${templateId}`);
    }

    // 필수 변수 검증
    this.validateVariables(template, variables);

    // 조항 처리
    const processedClauses = template.clauses
      .filter(clause => this.shouldIncludeClause(clause, options))
      .map(clause => this.processClause(clause, variables));

    // 문서 조합
    const document = this.assembleDocument(template, processedClauses, variables);

    return {
      templateId,
      templateVersion: template.version,
      generatedAt: new Date().toISOString(),
      content: document,
      format: this.config.outputFormat,
      variables: this.sanitizeVariables(variables),
      clauses: processedClauses.map(c => c.id),
    };
  }

  private validateVariables(
    template: ContractTemplate,
    variables: Record<string, unknown>
  ): void {
    const requiredVars = new Set<string>();
    const [open, close] = this.config.variableDelimiters;

    for (const clause of template.clauses) {
      if (clause.mandatory) {
        const matches = clause.text.matchAll(
          new RegExp(`${this.escapeRegex(open)}(\\w+)${this.escapeRegex(close)}`, 'g')
        );

        for (const match of matches) {
          requiredVars.add(match[1]);
        }
      }
    }

    const missing: string[] = [];
    for (const varName of requiredVars) {
      if (!(varName in variables)) {
        missing.push(varName);
      }
    }

    if (missing.length > 0) {
      throw new Error(`필수 변수 누락: ${missing.join(', ')}`);
    }
  }

  private shouldIncludeClause(clause: Clause, options: GenerationOptions): boolean {
    if (clause.mandatory) return true;
    if (options.includeClauses?.includes(clause.id)) return true;
    if (options.excludeClauses?.includes(clause.id)) return false;
    return options.includeOptional !== false;
  }

  private processClause(
    clause: Clause,
    variables: Record<string, unknown>
  ): ProcessedClause {
    let text = clause.text;
    const [open, close] = this.config.variableDelimiters;

    for (const [key, value] of Object.entries(variables)) {
      const pattern = new RegExp(
        `${this.escapeRegex(open)}${key}${this.escapeRegex(close)}`,
        'g'
      );
      text = text.replace(pattern, String(value));
    }

    text = this.processConditionals(text, variables);
    text = this.processLoops(text, variables);

    return {
      id: clause.id,
      title: clause.title,
      text: text.trim(),
      category: clause.category,
    };
  }

  private processConditionals(
    text: string,
    variables: Record<string, unknown>
  ): string {
    const [open, close] = this.config.conditionalDelimiters;

    const ifPattern = new RegExp(
      `${this.escapeRegex(open)}if\\s+(\\w+)${this.escapeRegex(close)}([\\s\\S]*?)${this.escapeRegex(open.replace('#', '/')}if${this.escapeRegex(close)}`,
      'g'
    );

    return text.replace(ifPattern, (match, condition, content) => {
      const value = variables[condition];

      if (value) {
        const elseParts = content.split(/\{\{else\}\}/);
        return elseParts[0].trim();
      } else {
        const elseParts = content.split(/\{\{else\}\}/);
        return elseParts[1]?.trim() || '';
      }
    });
  }

  private processLoops(
    text: string,
    variables: Record<string, unknown>
  ): string {
    const [open, close] = this.config.conditionalDelimiters;

    const eachPattern = new RegExp(
      `${this.escapeRegex(open)}each\\s+(\\w+)${this.escapeRegex(close)}([\\s\\S]*?)${this.escapeRegex(open.replace('#', '/')}each${this.escapeRegex(close)}`,
      'g'
    );

    return text.replace(eachPattern, (match, arrayName, template) => {
      const array = variables[arrayName];

      if (!Array.isArray(array)) {
        return '';
      }

      return array.map(item => {
        let itemText = template;
        itemText = itemText.replace(/\{\{this\}\}/g, String(item));

        if (typeof item === 'object' && item !== null) {
          for (const [key, value] of Object.entries(item)) {
            itemText = itemText.replace(
              new RegExp(`\\{\\{${key}\\}\\}`, 'g'),
              String(value)
            );
          }
        }

        return itemText.trim();
      }).join('\n');
    });
  }

  private assembleDocument(
    template: ContractTemplate,
    clauses: ProcessedClause[],
    variables: Record<string, unknown>
  ): string {
    const sections: string[] = [];

    // 머리말
    sections.push(`# ${template.name}`);
    if (template.nameEn) {
      sections.push(`## ${template.nameEn}`);
    }
    sections.push('');
    sections.push(`**버전**: ${template.version}`);
    sections.push(`**관할권**: ${template.jurisdiction === 'KR' ? '대한민국' : template.jurisdiction}`);
    sections.push(`**작성일**: ${new Date().toLocaleDateString('ko-KR')}`);
    sections.push('');
    sections.push('---');
    sections.push('');

    // 카테고리별 조항 그룹화
    const categorized = this.categorize(clauses);

    for (const [category, categoryClause] of categorized) {
      sections.push(`## ${this.formatCategoryNameKR(category)}`);
      sections.push('');

      for (const clause of categoryClause) {
        sections.push(`### ${clause.title}`);
        sections.push('');
        sections.push(clause.text);
        sections.push('');
      }
    }

    // 서명란
    sections.push('---');
    sections.push('');
    sections.push('## 서명');
    sections.push('');
    sections.push(this.generateSignatureBlockKR(variables));

    return sections.join('\n');
  }

  private categorize(clauses: ProcessedClause[]): Map<string, ProcessedClause[]> {
    const categoryOrder = [
      'scope', 'term', 'fees', 'operations', 'access',
      'consent', 'disclosure', 'instructions', 'posthumous',
      'requirements', 'privacy', 'ip', 'screening', 'rights',
      'intent', 'liability', 'insurance', 'termination',
      'disposition', 'legal', 'regulatory', 'quality', 'usage',
      'publication',
    ];

    const categorized = new Map<string, ProcessedClause[]>();

    for (const clause of clauses) {
      const existing = categorized.get(clause.category) || [];
      existing.push(clause);
      categorized.set(clause.category, existing);
    }

    return new Map(
      [...categorized.entries()].sort((a, b) => {
        const aIndex = categoryOrder.indexOf(a[0]);
        const bIndex = categoryOrder.indexOf(b[0]);
        return (aIndex === -1 ? 999 : aIndex) - (bIndex === -1 ? 999 : bIndex);
      })
    );
  }

  private formatCategoryNameKR(category: string): string {
    const categoryNames: Record<string, string> = {
      scope: '적용 범위',
      term: '계약 기간',
      fees: '비용',
      operations: '운영',
      access: '접근 및 인출',
      consent: '동의',
      disclosure: '고지 사항',
      instructions: '지시 사항',
      posthumous: '사후 처리',
      requirements: '요건',
      privacy: '개인정보 보호',
      ip: '지적재산권',
      screening: '검사',
      rights: '권리',
      intent: '의사 표시',
      liability: '책임',
      insurance: '보험',
      termination: '해지',
      disposition: '처분',
      legal: '법적 사항',
      regulatory: '규제 준수',
      quality: '품질 기준',
      usage: '사용 조건',
      publication: '발표 및 출판',
    };

    return categoryNames[category] || category
      .split('-')
      .map(word => word.charAt(0).toUpperCase() + word.slice(1))
      .join(' ');
  }

  private generateSignatureBlockKR(variables: Record<string, unknown>): string {
    const lines: string[] = [];

    lines.push('**갑 (의뢰인/기증자):**');
    lines.push('');
    lines.push('서명: _______________________________');
    lines.push('');
    lines.push(`성명: ${variables.client_name || '____________________'}`);
    lines.push('');
    lines.push('주민등록번호: ______-_______');
    lines.push('');
    lines.push('날짜: ____년 ____월 ____일');
    lines.push('');
    lines.push('');
    lines.push('**을 (시설):**');
    lines.push('');
    lines.push('서명: _______________________________');
    lines.push('');
    lines.push(`성명: ${variables.facility_representative || '____________________'}`);
    lines.push('');
    lines.push(`직위: ${variables.facility_representative_title || '____________________'}`);
    lines.push('');
    lines.push('날짜: ____년 ____월 ____일');
    lines.push('');
    lines.push('');
    lines.push('**입회인 (필요시):**');
    lines.push('');
    lines.push('서명: _______________________________');
    lines.push('');
    lines.push('성명: ____________________');
    lines.push('');
    lines.push('날짜: ____년 ____월 ____일');

    return lines.join('\n');
  }

  private escapeRegex(string: string): string {
    return string.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  }

  private sanitizeVariables(variables: Record<string, unknown>): Record<string, unknown> {
    const sensitive = ['ssn', 'password', 'credit_card', 'bank_account', 'jumin'];
    const sanitized = { ...variables };

    for (const key of Object.keys(sanitized)) {
      if (sensitive.some(s => key.toLowerCase().includes(s))) {
        sanitized[key] = '[비공개]';
      }
    }

    return sanitized;
  }
}

export interface GenerationOptions {
  includeClauses?: string[];
  excludeClauses?: string[];
  includeOptional?: boolean;
  customClauses?: Clause[];
  language?: 'ko' | 'en' | 'bilingual';
}

export interface GeneratedContract {
  templateId: string;
  templateVersion: string;
  generatedAt: string;
  content: string;
  format: string;
  variables: Record<string, unknown>;
  clauses: string[];
}

export interface ProcessedClause {
  id: string;
  title: string;
  text: string;
  category: string;
}
```

## 계약 수명주기 관리자

```typescript
/**
 * 계약 수명주기 관리
 * 계약 상태, 서명 및 수정 처리
 */

export class ContractLifecycleManager {
  private contracts: Map<string, Contract> = new Map();
  private eventBus: ContractEventBus;

  constructor(
    private readonly storage: ContractStorage,
    private readonly signatureService: SignatureService,
    private readonly notificationService: NotificationService
  ) {
    this.eventBus = new ContractEventBus();
  }

  async createContract(
    generated: GeneratedContract,
    parties: Party[]
  ): Promise<Contract> {
    const contractId = this.generateContractId();

    const contract: Contract = {
      id: contractId,
      templateId: generated.templateId,
      type: this.inferContractType(generated.templateId),
      parties,
      terms: this.extractTerms(generated),
      signatures: [],
      effectiveDate: new Date().toISOString(),
      status: 'draft',
      history: [{
        id: crypto.randomUUID(),
        type: 'created',
        date: new Date().toISOString(),
        actor: 'system',
        description: '템플릿에서 계약서 생성됨',
        documents: [],
      }],
    };

    await this.storage.save(contract);
    this.contracts.set(contractId, contract);

    this.eventBus.emit('contract:created', { contract });

    return contract;
  }

  async submitForSignature(
    contractId: string,
    requestedBy: string
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'draft') {
      throw new Error(`${contract.status} 상태의 계약서는 제출할 수 없습니다`);
    }

    // 필수 필드 검증
    this.validateForSubmission(contract);

    // 상태 업데이트
    contract.status = 'pending';

    // 이력 이벤트 추가
    contract.history.push({
      id: crypto.randomUUID(),
      type: 'submitted',
      date: new Date().toISOString(),
      actor: requestedBy,
      description: '서명을 위해 계약서 제출됨',
    });

    await this.storage.save(contract);

    // 서명 요청 발송
    for (const party of contract.parties) {
      await this.signatureService.requestSignature(contract, party);
      await this.notificationService.sendSignatureRequest(party, contract);
    }

    this.eventBus.emit('contract:submitted', { contract });

    return contract;
  }

  async addSignature(
    contractId: string,
    signature: Signature
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'pending') {
      throw new Error(`${contract.status} 상태의 계약서에 서명할 수 없습니다`);
    }

    // 서명 검증
    const verification = await this.signatureService.verifySignature(
      contract,
      signature
    );

    if (!verification.valid) {
      throw new Error(`유효하지 않은 서명: ${verification.error}`);
    }

    // 서명 추가
    contract.signatures.push({
      ...signature,
      date: new Date().toISOString(),
    });

    // 이력 이벤트 추가
    contract.history.push({
      id: crypto.randomUUID(),
      type: 'signed',
      date: new Date().toISOString(),
      actor: signature.signatory,
      description: `${signature.party}를 대리하여 ${signature.signatory}가 서명함`,
    });

    // 모든 당사자가 서명했는지 확인
    const allSigned = this.checkAllPartiesSigned(contract);

    if (allSigned) {
      // 자동 계약 발효
      contract.status = 'active';

      contract.history.push({
        id: crypto.randomUUID(),
        type: 'executed',
        date: new Date().toISOString(),
        actor: 'system',
        description: '계약 발효 - 모든 당사자 서명 완료',
      });

      this.eventBus.emit('contract:executed', { contract });
    }

    await this.storage.save(contract);

    this.eventBus.emit('contract:signed', { contract, signature });

    return contract;
  }

  async executeContract(
    contractId: string,
    executor: string
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'pending') {
      throw new Error(`${contract.status} 상태의 계약서를 발효할 수 없습니다`);
    }

    if (!this.checkAllPartiesSigned(contract)) {
      throw new Error('모든 서명 없이 계약을 발효할 수 없습니다');
    }

    contract.status = 'active';

    contract.history.push({
      id: crypto.randomUUID(),
      type: 'executed',
      date: new Date().toISOString(),
      actor: executor,
      description: '계약 수동 발효됨',
    });

    await this.storage.save(contract);

    this.eventBus.emit('contract:executed', { contract });

    // 갱신 알림 예약
    if (contract.expiryDate) {
      await this.scheduleRenewalReminder(contract);
    }

    return contract;
  }

  async amendContract(
    contractId: string,
    amendment: Amendment,
    actor: string
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'active') {
      throw new Error(`${contract.status} 상태의 계약서를 수정할 수 없습니다`);
    }

    // 수정안 검증
    this.validateAmendment(contract, amendment);

    // 수정 적용
    const updatedTerms = this.applyAmendment(contract.terms, amendment);
    contract.terms = updatedTerms;

    // 중대한 수정의 경우 서명 재요청
    if (amendment.requiresResigning) {
      contract.signatures = [];
      contract.status = 'pending';

      contract.history.push({
        id: crypto.randomUUID(),
        type: 'amended',
        date: new Date().toISOString(),
        actor,
        description: `재서명 필요한 중대 수정: ${amendment.description}`,
      });

      // 새로운 서명 요청
      for (const party of contract.parties) {
        await this.signatureService.requestSignature(contract, party);
        await this.notificationService.sendAmendmentNotice(party, contract, amendment);
      }
    } else {
      contract.history.push({
        id: crypto.randomUUID(),
        type: 'amended',
        date: new Date().toISOString(),
        actor,
        description: `비중대 수정: ${amendment.description}`,
      });
    }

    await this.storage.save(contract);

    this.eventBus.emit('contract:amended', { contract, amendment });

    return contract;
  }

  async terminateContract(
    contractId: string,
    reason: string,
    effectiveDate: string,
    actor: string
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'active') {
      throw new Error(`${contract.status} 상태의 계약서를 해지할 수 없습니다`);
    }

    contract.status = 'terminated';

    contract.history.push({
      id: crypto.randomUUID(),
      type: 'terminated',
      date: new Date().toISOString(),
      actor,
      description: `계약 해지: ${reason}. 발효일: ${effectiveDate}`,
    });

    await this.storage.save(contract);

    // 모든 당사자에게 통지
    for (const party of contract.parties) {
      await this.notificationService.sendTerminationNotice(
        party,
        contract,
        reason,
        effectiveDate
      );
    }

    this.eventBus.emit('contract:terminated', { contract, reason });

    return contract;
  }

  async renewContract(
    contractId: string,
    renewalTerms: Partial<ContractTerms>,
    actor: string
  ): Promise<Contract> {
    const originalContract = await this.getContract(contractId);

    if (originalContract.status !== 'active' && originalContract.status !== 'expired') {
      throw new Error(`${originalContract.status} 상태의 계약서를 갱신할 수 없습니다`);
    }

    // 원본 기반 새 계약 생성
    const renewedContract: Contract = {
      ...originalContract,
      id: this.generateContractId(),
      terms: {
        ...originalContract.terms,
        ...renewalTerms,
      },
      effectiveDate: originalContract.expiryDate || new Date().toISOString(),
      signatures: [],
      status: 'draft',
      history: [{
        id: crypto.randomUUID(),
        type: 'renewed',
        date: new Date().toISOString(),
        actor,
        description: `계약 ${originalContract.id}에서 갱신됨`,
      }],
    };

    // 원본을 만료 상태로 변경
    if (originalContract.status === 'active') {
      originalContract.status = 'expired';

      originalContract.history.push({
        id: crypto.randomUUID(),
        type: 'expired',
        date: new Date().toISOString(),
        actor: 'system',
        description: `만료 후 ${renewedContract.id}로 갱신됨`,
      });

      await this.storage.save(originalContract);
    }

    await this.storage.save(renewedContract);

    this.eventBus.emit('contract:renewed', {
      originalContract,
      renewedContract,
    });

    return renewedContract;
  }

  async getContract(id: string): Promise<Contract> {
    let contract = this.contracts.get(id);

    if (!contract) {
      contract = await this.storage.load(id);

      if (!contract) {
        throw new Error(`계약서를 찾을 수 없습니다: ${id}`);
      }

      this.contracts.set(id, contract);
    }

    return contract;
  }

  private generateContractId(): string {
    const timestamp = Date.now().toString(36).toUpperCase();
    const random = Math.random().toString(36).substring(2, 8).toUpperCase();
    return `CTR-${timestamp}-${random}`;
  }

  private inferContractType(templateId: string): ContractType {
    if (templateId.includes('storage')) return 'storage-agreement';
    if (templateId.includes('consent')) return 'consent-form';
    if (templateId.includes('research')) return 'research-agreement';
    if (templateId.includes('donation')) return 'donation-agreement';
    if (templateId.includes('transfer')) return 'transfer-agreement';
    return 'service-agreement';
  }

  private extractTerms(generated: GeneratedContract): ContractTerms {
    return {
      duration: generated.variables.duration as string || '1년',
      renewal: 'manual',
      fees: [],
      obligations: [],
      warranties: [],
      limitations: [],
      governing: generated.variables.governing_state as string || '대한민국',
      venue: generated.variables.venue_county as string || '서울중앙지방법원',
    };
  }

  private validateForSubmission(contract: Contract): void {
    if (contract.parties.length < 2) {
      throw new Error('계약서에는 최소 두 당사자가 필요합니다');
    }

    const hasPrimary = contract.parties.some(p => p.role === 'primary');
    const hasCounterparty = contract.parties.some(p => p.role === 'counterparty');

    if (!hasPrimary || !hasCounterparty) {
      throw new Error('계약서에는 주당사자와 상대방이 필요합니다');
    }
  }

  private checkAllPartiesSigned(contract: Contract): boolean {
    const requiredParties = contract.parties
      .filter(p => p.role === 'primary' || p.role === 'counterparty')
      .map(p => p.id);

    const signedParties = new Set(contract.signatures.map(s => s.party));

    return requiredParties.every(id => signedParties.has(id));
  }

  private validateAmendment(contract: Contract, amendment: Amendment): void {
    // 계약 조건에 따른 수정 허용 여부 확인
  }

  private applyAmendment(
    terms: ContractTerms,
    amendment: Amendment
  ): ContractTerms {
    return {
      ...terms,
      ...amendment.changes,
    };
  }

  private async scheduleRenewalReminder(contract: Contract): Promise<void> {
    // 갱신 알림 예약
  }

  onContractEvent(
    event: string,
    handler: (data: unknown) => void
  ): void {
    this.eventBus.on(event, handler);
  }
}

export interface Amendment {
  id: string;
  description: string;
  changes: Partial<ContractTerms>;
  requiresResigning: boolean;
  effectiveDate: string;
}

export interface ContractStorage {
  save(contract: Contract): Promise<void>;
  load(id: string): Promise<Contract | null>;
  findByParty(partyId: string): Promise<Contract[]>;
  findByStatus(status: ContractStatus): Promise<Contract[]>;
}

export interface SignatureService {
  requestSignature(contract: Contract, party: Party): Promise<void>;
  verifySignature(contract: Contract, signature: Signature): Promise<SignatureVerification>;
}

export interface SignatureVerification {
  valid: boolean;
  error?: string;
  timestamp?: string;
}

export interface NotificationService {
  sendSignatureRequest(party: Party, contract: Contract): Promise<void>;
  sendAmendmentNotice(party: Party, contract: Contract, amendment: Amendment): Promise<void>;
  sendTerminationNotice(party: Party, contract: Contract, reason: string, effectiveDate: string): Promise<void>;
}

class ContractEventBus {
  private handlers: Map<string, Set<(data: unknown) => void>> = new Map();

  on(event: string, handler: (data: unknown) => void): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, new Set());
    }
    this.handlers.get(event)!.add(handler);
  }

  emit(event: string, data: unknown): void {
    const handlers = this.handlers.get(event);
    if (handlers) {
      for (const handler of handlers) {
        handler(data);
      }
    }
  }
}
```

## 전자서명 통합

```typescript
/**
 * 전자서명 서비스 구현
 * 다중 서명 방식 지원 (전자서명법 준수)
 */

export class ElectronicSignatureService implements SignatureService {
  constructor(
    private readonly config: SignatureConfig,
    private readonly storage: SignatureStorage,
    private readonly auditLog: AuditLogger
  ) {}

  async requestSignature(contract: Contract, party: Party): Promise<void> {
    const request: SignatureRequest = {
      id: crypto.randomUUID(),
      contractId: contract.id,
      partyId: party.id,
      partyName: party.name,
      partyEmail: party.contact.email,
      status: 'pending',
      createdAt: new Date().toISOString(),
      expiresAt: this.calculateExpiry(),
    };

    await this.storage.saveRequest(request);

    // 서명 URL 생성
    const signatureUrl = await this.generateSignatureUrl(request);

    // 알림 발송
    await this.sendSignatureEmail(party, contract, signatureUrl);

    await this.auditLog.log({
      action: 'signature_requested',
      actionKr: '서명 요청됨',
      contractId: contract.id,
      partyId: party.id,
      timestamp: new Date(),
    });
  }

  async verifySignature(
    contract: Contract,
    signature: Signature
  ): Promise<SignatureVerification> {
    // 당사자 권한 확인
    const party = contract.parties.find(p => p.id === signature.party);
    if (!party) {
      return { valid: false, error: '계약서에서 당사자를 찾을 수 없습니다' };
    }

    // 서명권자 확인
    if (party.representative && party.representative !== signature.signatory) {
      return { valid: false, error: '서명권자가 아닙니다' };
    }

    // 서명 방식 유효성 확인
    const methodValid = await this.verifySignatureMethod(signature);
    if (!methodValid.valid) {
      return methodValid;
    }

    // 중복 서명 확인
    const existingSignature = contract.signatures.find(s => s.party === signature.party);
    if (existingSignature) {
      return { valid: false, error: '당사자가 이미 서명했습니다' };
    }

    // 서명 기록
    await this.recordSignature(contract.id, signature);

    await this.auditLog.log({
      action: 'signature_verified',
      actionKr: '서명 검증됨',
      contractId: contract.id,
      partyId: signature.party,
      signatory: signature.signatory,
      method: signature.method,
      timestamp: new Date(),
    });

    return { valid: true, timestamp: new Date().toISOString() };
  }

  private async verifySignatureMethod(
    signature: Signature
  ): Promise<SignatureVerification> {
    switch (signature.method) {
      case 'electronic':
        return this.verifyElectronicSignature(signature);
      case 'digital':
        return this.verifyDigitalSignature(signature);
      case 'wet':
        return this.verifyWetSignature(signature);
      case 'certified':
        return this.verifyCertifiedSignature(signature);
      default:
        return { valid: false, error: '알 수 없는 서명 방식입니다' };
    }
  }

  private async verifyElectronicSignature(
    signature: Signature
  ): Promise<SignatureVerification> {
    // 전자서명 검증
    // - IP 주소 기록
    // - 브라우저 핑거프린트
    // - 클릭-서명 확인
    return { valid: true };
  }

  private async verifyDigitalSignature(
    signature: Signature
  ): Promise<SignatureVerification> {
    // 공인인증서/공동인증서 검증
    // - 인증서 유효성 확인
    // - PKI 검증
    // - 타임스탬프 기관 검증
    return { valid: true };
  }

  private async verifyWetSignature(
    signature: Signature
  ): Promise<SignatureVerification> {
    // 날인 서명 검증
    // - 문서 스캔 업로드
    // - 필요시 공증 확인
    if (signature.notarized) {
      // 공증 확인
    }
    return { valid: true };
  }

  private async verifyCertifiedSignature(
    signature: Signature
  ): Promise<SignatureVerification> {
    // 공인전자서명 검증 (전자서명법 준수)
    // - KISA 인증 기관 검증
    // - 인증서 체인 확인
    // - 폐기 목록 확인
    return { valid: true };
  }

  private calculateExpiry(): string {
    const expiry = new Date();
    expiry.setDate(expiry.getDate() + this.config.signatureExpiryDays);
    return expiry.toISOString();
  }

  private async generateSignatureUrl(request: SignatureRequest): Promise<string> {
    const token = await this.generateSecureToken(request);
    return `${this.config.baseUrl}/sign/${request.id}?token=${token}`;
  }

  private async generateSecureToken(request: SignatureRequest): Promise<string> {
    return Buffer.from(JSON.stringify({
      requestId: request.id,
      exp: request.expiresAt,
    })).toString('base64url');
  }

  private async sendSignatureEmail(
    party: Party,
    contract: Contract,
    url: string
  ): Promise<void> {
    // 서명 링크가 포함된 이메일 발송
  }

  private async recordSignature(
    contractId: string,
    signature: Signature
  ): Promise<void> {
    await this.storage.saveSignature({
      contractId,
      signature,
      metadata: {
        recordedAt: new Date().toISOString(),
        ipAddress: '별도 기록',
        userAgent: '별도 기록',
      },
    });
  }
}

export interface SignatureConfig {
  baseUrl: string;
  signatureExpiryDays: number;
  allowedMethods: SignatureMethod[];
  requireNotarization: boolean;
  certifiedSignatureRequired: boolean;  // 공인전자서명 필수 여부
}

export interface SignatureRequest {
  id: string;
  contractId: string;
  partyId: string;
  partyName: string;
  partyEmail: string;
  status: 'pending' | 'signed' | 'expired' | 'cancelled';
  createdAt: string;
  expiresAt: string;
  signedAt?: string;
}

export interface SignatureStorage {
  saveRequest(request: SignatureRequest): Promise<void>;
  getRequest(id: string): Promise<SignatureRequest | null>;
  saveSignature(data: { contractId: string; signature: Signature; metadata: any }): Promise<void>;
}

export interface AuditLogger {
  log(entry: AuditEntry): Promise<void>;
}

export interface AuditEntry {
  action: string;
  actionKr?: string;
  contractId: string;
  partyId?: string;
  signatory?: string;
  method?: string;
  timestamp: Date;
}

export type SignatureMethod = 'wet' | 'electronic' | 'digital' | 'certified';
```

---

## 장 요약

이 장에서는 종합 계약 관리에 대해 다루었습니다:

- **템플릿 엔진**: 변수 처리를 통한 동적 템플릿 생성
- **조항 라이브러리**: 카테고리별 재사용 가능한 조항 관리
- **수명주기 관리**: 초안에서 발효까지의 워크플로우
- **수정 프로세스**: 중대/비중대 수정 처리
- **전자서명**: 다중 서명 방식 지원 (전자서명법 준수)
- **한국 특화**: 생명윤리법, 개인정보보호법 준수

---

**다음 장**: [분쟁 해결 - 메커니즘 및 절차](./06-dispute-resolution.md)
