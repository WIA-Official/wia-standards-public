# 제6장: 통합

## 시스템 통합 패턴

본 장에서는 WIA Cryo-Consent 시스템을 의료 시스템, 법적 프레임워크, 냉동보존 기관 및 규제 기관과 연결하기 위한 통합 패턴을 정의합니다.

---

## 6.1 통합 아키텍처

```typescript
// 통합 오케스트레이터
class ConsentIntegrationOrchestrator {
  private adapters: Map<string, IntegrationAdapter> = new Map();
  private eventBus: EventBus;
  private transformerService: DataTransformerService;
  private auditService: AuditService;

  constructor(config: IntegrationConfig) {
    this.eventBus = new EventBus(config.eventBus);
    this.transformerService = new DataTransformerService();
    this.auditService = new AuditService(config.audit);

    // 어댑터 등록
    this.registerAdapter('healthcare', new HealthcareSystemAdapter(config.healthcare));
    this.registerAdapter('legal', new LegalSystemAdapter(config.legal));
    this.registerAdapter('cryonics', new CryonicsOrganizationAdapter(config.cryonics));
    this.registerAdapter('identity', new IdentityVerificationAdapter(config.identity));
    this.registerAdapter('document', new DocumentManagementAdapter(config.document));
    this.registerAdapter('notification', new NotificationAdapter(config.notification));
    this.registerAdapter('blockchain', new BlockchainAdapter(config.blockchain));
  }

  registerAdapter(name: string, adapter: IntegrationAdapter): void {
    this.adapters.set(name, adapter);
    adapter.setEventBus(this.eventBus);
  }

  // 시스템 간 동의 동기화
  async synchronizeConsent(
    consent: ConsentRecord,
    targets: string[]
  ): Promise<SynchronizationResult> {
    const results: SystemSyncResult[] = [];

    for (const target of targets) {
      const adapter = this.adapters.get(target);
      if (!adapter) {
        results.push({
          system: target,
          success: false,
          error: `${target}에 대한 어댑터를 찾을 수 없습니다`,
        });
        continue;
      }

      try {
        // 동의를 대상 형식으로 변환
        const transformed = await this.transformerService.transform(
          consent,
          adapter.getTargetFormat()
        );

        // 대상 시스템으로 전송
        const syncResult = await adapter.syncConsent(transformed);

        results.push({
          system: target,
          success: syncResult.success,
          externalId: syncResult.externalId,
          timestamp: syncResult.timestamp,
        });

        // 동기화 기록
        await this.auditService.logSync(consent.id, target, syncResult);
      } catch (error) {
        results.push({
          system: target,
          success: false,
          error: error.message,
        });
      }
    }

    return {
      consentId: consent.id,
      synchronizedAt: new Date(),
      results,
      allSuccessful: results.every(r => r.success),
    };
  }

  // 외부 시스템에서 들어오는 이벤트 처리
  async handleExternalEvent(event: ExternalIntegrationEvent): Promise<void> {
    const adapter = this.adapters.get(event.sourceSystem);
    if (!adapter) {
      throw new Error(`알 수 없는 소스 시스템: ${event.sourceSystem}`);
    }

    // 이벤트 처리
    const processedEvent = await adapter.processIncomingEvent(event);

    // 적절한 핸들러로 라우팅
    switch (processedEvent.type) {
      case 'CONSENT_UPDATE_REQUEST':
        await this.handleExternalConsentUpdate(processedEvent);
        break;

      case 'CONSENT_QUERY':
        await this.handleExternalConsentQuery(processedEvent);
        break;

      case 'STATUS_UPDATE':
        await this.handleExternalStatusUpdate(processedEvent);
        break;

      case 'DOCUMENT_RECEIVED':
        await this.handleDocumentReceived(processedEvent);
        break;

      default:
        await this.eventBus.emit(processedEvent);
    }
  }

  private async handleExternalConsentUpdate(
    event: ProcessedIntegrationEvent
  ): Promise<void> {
    // 외부 업데이트 요청 검증
    const validation = await this.validateExternalUpdate(event);
    if (!validation.valid) {
      await this.notifyUpdateRejection(event, validation.reason);
      return;
    }

    // 업데이트 적용
    await this.consentService.processExternalUpdate(
      event.consentId,
      event.updates,
      {
        source: event.sourceSystem,
        externalReference: event.externalReference,
      }
    );
  }
}

// 기본 어댑터 인터페이스
interface IntegrationAdapter {
  setEventBus(eventBus: EventBus): void;
  getTargetFormat(): string;
  syncConsent(consent: TransformedConsent): Promise<SyncResult>;
  processIncomingEvent(event: ExternalIntegrationEvent): Promise<ProcessedIntegrationEvent>;
  healthCheck(): Promise<HealthCheckResult>;
}

interface SynchronizationResult {
  consentId: string;
  synchronizedAt: Date;
  results: SystemSyncResult[];
  allSuccessful: boolean;
}

interface SystemSyncResult {
  system: string;
  success: boolean;
  externalId?: string;
  timestamp?: Date;
  error?: string;
}
```

---

## 6.2 의료 시스템 통합

```typescript
// 의료 시스템 어댑터 (HL7 FHIR 호환)
class HealthcareSystemAdapter implements IntegrationAdapter {
  private fhirClient: FHIRClient;
  private hl7Client: HL7v2Client;
  private eventBus: EventBus;

  constructor(config: HealthcareConfig) {
    this.fhirClient = new FHIRClient(config.fhirEndpoint, config.credentials);
    this.hl7Client = new HL7v2Client(config.hl7Endpoint, config.credentials);
  }

  setEventBus(eventBus: EventBus): void {
    this.eventBus = eventBus;
  }

  getTargetFormat(): string {
    return 'FHIR_R4';
  }

  // 의료 시스템으로 동의 동기화
  async syncConsent(consent: TransformedConsent): Promise<SyncResult> {
    // FHIR Consent 리소스로 변환
    const fhirConsent = this.toFHIRConsent(consent);

    // 존재 여부 확인
    const existing = await this.fhirClient.search('Consent', {
      identifier: consent.id,
    });

    let result: any;
    if (existing.total > 0) {
      // 기존 업데이트
      result = await this.fhirClient.update(
        'Consent',
        existing.entry[0].resource.id,
        fhirConsent
      );
    } else {
      // 새로 생성
      result = await this.fhirClient.create('Consent', fhirConsent);
    }

    // 관련 리소스도 동기화
    await this.syncRelatedResources(consent, result.id);

    return {
      success: true,
      externalId: result.id,
      timestamp: new Date(),
    };
  }

  // WIA 동의를 FHIR Consent 리소스로 변환
  private toFHIRConsent(consent: TransformedConsent): FHIRConsent {
    return {
      resourceType: 'Consent',
      identifier: [{
        system: 'https://wia.org/consent',
        value: consent.id,
      }],
      status: this.mapStatusToFHIR(consent.validity.status),
      scope: {
        coding: [{
          system: 'http://terminology.hl7.org/CodeSystem/consentscope',
          code: this.mapCategoryToFHIRScope(consent.category),
        }],
      },
      category: [{
        coding: [{
          system: 'https://wia.org/consent-category',
          code: consent.category,
          display: this.getCategoryDisplay(consent.category),
        }],
      }],
      patient: {
        reference: `Patient/${consent.patientId}`,
      },
      dateTime: consent.validity.effectiveDate.toISOString(),
      performer: consent.authority.witnesses?.map(w => ({
        reference: `Practitioner/${w.witnessId}`,
      })),
      organization: [{
        reference: `Organization/${consent.organizationId}`,
      }],
      sourceAttachment: consent.documents?.map(d => ({
        contentType: 'application/pdf',
        url: d.storage.primaryLocation.locationUri,
        title: d.title,
      })),
      policy: [{
        authority: 'https://wia.org',
        uri: 'https://wia.org/consent-policy/v1',
      }],
      provision: this.mapDecisionsToProvisions(consent.decisions),
    };
  }

  private mapDecisionsToProvisions(decisions: ConsentDecision[]): FHIRProvision {
    return {
      type: 'permit',
      period: {
        start: new Date().toISOString(),
      },
      actor: [],
      action: decisions.map(d => ({
        coding: [{
          system: 'https://wia.org/consent-actions',
          code: d.decisionType,
          display: d.question,
        }],
      })),
      purpose: decisions
        .filter(d => d.context?.scenarioDescription)
        .map(d => ({
          coding: [{
            system: 'https://wia.org/consent-purpose',
            code: 'CRYONICS',
            display: d.context!.scenarioDescription,
          }],
        })),
      data: [],
      provision: decisions.map(d => this.mapDecisionToProvision(d)),
    };
  }

  private mapDecisionToProvision(decision: ConsentDecision): FHIRProvision {
    const baseProvision: FHIRProvision = {
      type: decision.answer.binaryValue === false ? 'deny' : 'permit',
    };

    if (decision.decisionType === 'CONDITIONAL' && decision.answer.conditionalRules) {
      baseProvision.provision = decision.answer.conditionalRules.map(rule => ({
        type: 'permit',
        code: [{
          coding: [{
            system: 'https://wia.org/consent-conditions',
            code: rule.ruleId,
            display: rule.condition,
          }],
        }],
      }));
    }

    return baseProvision;
  }

  // 관련 리소스 동기화 (환자, 의료인 참조)
  private async syncRelatedResources(
    consent: TransformedConsent,
    fhirConsentId: string
  ): Promise<void> {
    // 환자 존재 확인
    await this.ensurePatientExists(consent.patientId);

    // 문서에 대한 DocumentReference 리소스 동기화
    for (const doc of consent.documents || []) {
      await this.syncDocumentReference(doc, fhirConsentId);
    }

    // 대리인에 대한 RelatedPerson 동기화
    for (const proxy of consent.authority.proxyChain || []) {
      await this.syncProxyAsRelatedPerson(proxy, consent.patientId);
    }
  }

  private async syncDocumentReference(
    doc: ConsentDocument,
    consentId: string
  ): Promise<void> {
    const docRef: FHIRDocumentReference = {
      resourceType: 'DocumentReference',
      identifier: [{
        system: 'https://wia.org/documents',
        value: doc.documentId,
      }],
      status: 'current',
      type: {
        coding: [{
          system: 'http://loinc.org',
          code: '59284-0',
          display: '동의 문서',
        }],
      },
      category: [{
        coding: [{
          system: 'https://wia.org/document-category',
          code: doc.documentType,
        }],
      }],
      subject: {
        reference: `Consent/${consentId}`,
      },
      date: doc.version.versionDate.toISOString(),
      content: [{
        attachment: {
          contentType: this.getContentType(doc),
          url: doc.storage.primaryLocation.locationUri,
          hash: doc.integrity.hashValue,
          title: doc.title,
        },
      }],
      context: {
        related: [{
          reference: `Consent/${consentId}`,
        }],
      },
    };

    await this.fhirClient.create('DocumentReference', docRef);
  }

  // 의료 시스템에서 들어오는 이벤트 처리
  async processIncomingEvent(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    // HL7 또는 FHIR 메시지 파싱
    let parsedEvent: any;

    if (event.format === 'HL7V2') {
      parsedEvent = await this.parseHL7Event(event.payload);
    } else if (event.format === 'FHIR') {
      parsedEvent = await this.parseFHIREvent(event.payload);
    }

    // 내부 이벤트 형식으로 매핑
    return {
      type: this.mapEventType(parsedEvent.type),
      sourceSystem: 'healthcare',
      consentId: parsedEvent.consentId,
      patientId: parsedEvent.patientId,
      data: parsedEvent.data,
      timestamp: new Date(),
    };
  }

  private async parseHL7Event(payload: string): Promise<any> {
    const message = await this.hl7Client.parse(payload);

    // 다양한 HL7 메시지 유형 처리
    switch (message.messageType) {
      case 'ADT^A01': // 입원
        return {
          type: 'PATIENT_ADMISSION',
          patientId: message.patient.id,
          data: {
            admissionDate: message.admission.date,
            facility: message.admission.facility,
          },
        };

      case 'ADT^A03': // 퇴원
        return {
          type: 'PATIENT_DISCHARGE',
          patientId: message.patient.id,
          data: {
            dischargeDate: message.discharge.date,
            disposition: message.discharge.disposition,
          },
        };

      case 'ADT^A08': // 환자 정보 업데이트
        return {
          type: 'PATIENT_UPDATE',
          patientId: message.patient.id,
          data: message.patient,
        };

      default:
        return {
          type: 'UNKNOWN',
          data: message,
        };
    }
  }

  async healthCheck(): Promise<HealthCheckResult> {
    const fhirHealth = await this.fhirClient.ping();
    const hl7Health = await this.hl7Client.ping();

    return {
      healthy: fhirHealth.healthy && hl7Health.healthy,
      components: {
        fhir: fhirHealth,
        hl7: hl7Health,
      },
      lastCheck: new Date(),
    };
  }
}

// FHIR 타입 정의
interface FHIRConsent {
  resourceType: 'Consent';
  identifier: FHIRIdentifier[];
  status: string;
  scope: FHIRCodeableConcept;
  category: FHIRCodeableConcept[];
  patient: FHIRReference;
  dateTime: string;
  performer?: FHIRReference[];
  organization: FHIRReference[];
  sourceAttachment?: FHIRAttachment[];
  policy: FHIRConsentPolicy[];
  provision?: FHIRProvision;
}

interface FHIRProvision {
  type: 'permit' | 'deny';
  period?: FHIRPeriod;
  actor?: FHIRProvisionActor[];
  action?: FHIRCodeableConcept[];
  purpose?: FHIRCoding[];
  data?: FHIRProvisionData[];
  provision?: FHIRProvision[];
  code?: FHIRCodeableConcept[];
}
```

---

## 6.3 법률 시스템 통합

```typescript
// 법률 시스템 어댑터
class LegalSystemAdapter implements IntegrationAdapter {
  private courtFilingClient: CourtFilingClient;
  private notaryService: NotaryServiceClient;
  private legalDocumentService: LegalDocumentServiceClient;
  private eventBus: EventBus;

  constructor(config: LegalConfig) {
    this.courtFilingClient = new CourtFilingClient(config.courtFiling);
    this.notaryService = new NotaryServiceClient(config.notary);
    this.legalDocumentService = new LegalDocumentServiceClient(config.documents);
  }

  setEventBus(eventBus: EventBus): void {
    this.eventBus = eventBus;
  }

  getTargetFormat(): string {
    return 'LEGAL_DOCUMENT';
  }

  // 동의를 법적 문서로 동기화
  async syncConsent(consent: TransformedConsent): Promise<SyncResult> {
    // 법적 문서 생성
    const legalDoc = await this.generateLegalDocument(consent);

    // 공증이 필요한 경우 시작
    if (consent.authority.notarization) {
      await this.processNotarization(consent, legalDoc);
    }

    // 적절한 법원/등기소에 등록
    const registrations = await this.registerWithLegalAuthorities(consent, legalDoc);

    return {
      success: true,
      externalId: legalDoc.documentId,
      timestamp: new Date(),
      metadata: {
        registrations,
      },
    };
  }

  // 종합 법적 문서 생성
  private async generateLegalDocument(
    consent: TransformedConsent
  ): Promise<LegalDocument> {
    // 관할권에 맞는 템플릿 가져오기
    const template = await this.legalDocumentService.getTemplate(
      consent.scope.jurisdictions[0] || 'KR',
      'ADVANCE_DIRECTIVE'
    );

    // 템플릿 채우기
    const populatedDoc = this.populateTemplate(template, consent);

    // PDF 생성
    const pdf = await this.legalDocumentService.generatePDF(populatedDoc);

    return {
      documentId: generateDocumentId(),
      type: 'CRYONICS_CONSENT',
      jurisdiction: consent.scope.jurisdictions[0] || 'KR',
      content: populatedDoc,
      pdfData: pdf,
      signatures: [],
      notarization: null,
      createdAt: new Date(),
    };
  }

  private populateTemplate(
    template: LegalTemplate,
    consent: TransformedConsent
  ): PopulatedLegalDocument {
    return {
      templateId: template.id,
      templateVersion: template.version,

      sections: template.sections.map(section => ({
        sectionId: section.id,
        title: section.title,
        content: this.populateSectionContent(section, consent),
      })),

      declarations: this.generateDeclarations(consent),
      schedules: this.generateSchedules(consent),

      metadata: {
        consentId: consent.id,
        patientId: consent.patientId,
        generatedAt: new Date(),
      },
    };
  }

  private generateDeclarations(consent: TransformedConsent): LegalDeclaration[] {
    const declarations: LegalDeclaration[] = [];

    // 의향 선언
    declarations.push({
      declarationType: 'INTENT',
      text: `본인은 정신적으로 건전하고 법적 능력이 있는 상태에서, 본 문서에 명시된 조건에 따라 법적 사망 시 냉동보존될 의향을 선언합니다.`,
      required: true,
    });

    // 이해 선언
    declarations.push({
      declarationType: 'UNDERSTANDING',
      text: `본인은 냉동보존이 미래 부활을 보장하지 않으며, 부활 기술이 현재 존재하지 않음을 이해합니다. 이러한 불확실성을 충분히 인지한 상태에서 이 결정을 내립니다.`,
      required: true,
    });

    // 구체적 결정 선언
    for (const decision of consent.decisions) {
      if (decision.decisionType === 'BINARY' && decision.answer.binaryValue) {
        declarations.push({
          declarationType: 'SPECIFIC_CONSENT',
          text: `본인은 다음에 동의합니다: ${decision.question}`,
          decisionReference: decision.decisionId,
          required: false,
        });
      }
    }

    // 대리인 선언
    if (consent.authority.proxyChain?.length > 0) {
      declarations.push({
        declarationType: 'PROXY_DESIGNATION',
        text: `본인은 스스로 결정을 내릴 수 없는 경우, 나열된 순서대로 다음 개인/기관을 대리인으로 지정합니다.`,
        required: false,
      });
    }

    return declarations;
  }

  private generateSchedules(consent: TransformedConsent): LegalSchedule[] {
    const schedules: LegalSchedule[] = [];

    // 별표 A: 보존 선호도
    schedules.push({
      scheduleId: 'A',
      title: '보존 선호도',
      content: this.generatePreservationSchedule(consent),
    });

    // 별표 B: 대리인 지정
    if (consent.authority.proxyChain?.length > 0) {
      schedules.push({
        scheduleId: 'B',
        title: '대리인 지정',
        content: this.generateProxySchedule(consent),
      });
    }

    // 별표 C: 부활 선호도
    const revivalDecisions = consent.decisions.filter(
      d => d.context?.scenarioDescription?.toLowerCase().includes('부활') ||
           d.context?.scenarioDescription?.toLowerCase().includes('revival')
    );
    if (revivalDecisions.length > 0) {
      schedules.push({
        scheduleId: 'C',
        title: '부활 선호도',
        content: this.generateRevivalSchedule(consent, revivalDecisions),
      });
    }

    // 별표 D: 자산 지침
    const assetDecisions = consent.decisions.filter(
      d => d.context?.scenarioDescription?.toLowerCase().includes('자산') ||
           d.context?.scenarioDescription?.toLowerCase().includes('asset')
    );
    if (assetDecisions.length > 0) {
      schedules.push({
        scheduleId: 'D',
        title: '자산 관리 지침',
        content: this.generateAssetSchedule(consent, assetDecisions),
      });
    }

    return schedules;
  }

  // 공증 처리
  async processNotarization(
    consent: TransformedConsent,
    legalDoc: LegalDocument
  ): Promise<NotarizationResult> {
    // 공증 요청 시작
    const notaryRequest = {
      documentId: legalDoc.documentId,
      documentType: 'ADVANCE_DIRECTIVE',
      signatoryName: consent.patientId, // 실제 이름으로 해석됨
      documentHash: await this.calculateHash(legalDoc.pdfData),
      jurisdiction: consent.scope.jurisdictions[0] || 'KR',
      requestedDate: new Date(),
    };

    // 공증 서비스에 제출
    const notaryResponse = await this.notaryService.requestNotarization(notaryRequest);

    return {
      notarizationId: notaryResponse.id,
      status: notaryResponse.status,
      notaryId: notaryResponse.notaryId,
      appointmentDate: notaryResponse.appointmentDate,
    };
  }

  // 법적 기관에 등록
  async registerWithLegalAuthorities(
    consent: TransformedConsent,
    legalDoc: LegalDocument
  ): Promise<LegalRegistration[]> {
    const registrations: LegalRegistration[] = [];

    for (const jurisdiction of consent.scope.jurisdictions || []) {
      // 관할권에 적합한 등기소 결정
      const registry = this.getRegistryForJurisdiction(jurisdiction);

      if (registry) {
        try {
          const registration = await this.registerWithRegistry(
            registry,
            consent,
            legalDoc
          );
          registrations.push(registration);
        } catch (error) {
          console.error(`${jurisdiction} 등록 실패:`, error);
        }
      }
    }

    return registrations;
  }

  private getRegistryForJurisdiction(jurisdiction: string): LegalRegistry | null {
    const registries: Record<string, LegalRegistry> = {
      'KR': {
        name: '대한민국 사전의료의향서 등록기관',
        endpoint: 'https://registry.mohw.go.kr/advance-directive',
        documentTypes: ['ADVANCE_DIRECTIVE'],
      },
      'US-CA': {
        name: 'California Advance Healthcare Directive Registry',
        endpoint: 'https://registry.ca.gov/ahcd',
        documentTypes: ['ADVANCE_DIRECTIVE'],
      },
      'US-NY': {
        name: 'New York Health Care Proxy Registry',
        endpoint: 'https://health.ny.gov/proxy-registry',
        documentTypes: ['ADVANCE_DIRECTIVE', 'HEALTH_CARE_PROXY'],
      },
      // 더 많은 관할권 추가
    };

    return registries[jurisdiction] || null;
  }

  private async registerWithRegistry(
    registry: LegalRegistry,
    consent: TransformedConsent,
    legalDoc: LegalDocument
  ): Promise<LegalRegistration> {
    // 등기소에 제출
    const response = await this.courtFilingClient.submitToRegistry(
      registry.endpoint,
      {
        documentType: 'ADVANCE_DIRECTIVE',
        document: legalDoc.pdfData,
        metadata: {
          consentId: consent.id,
          patientId: consent.patientId,
          effectiveDate: consent.validity.effectiveDate,
        },
      }
    );

    return {
      registryName: registry.name,
      registrationNumber: response.registrationNumber,
      registrationDate: response.registrationDate,
      expirationDate: response.expirationDate,
      status: response.status,
    };
  }

  async processIncomingEvent(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    switch (event.type) {
      case 'NOTARIZATION_COMPLETE':
        return this.processNotarizationComplete(event);

      case 'REGISTRATION_UPDATE':
        return this.processRegistrationUpdate(event);

      case 'LEGAL_CHALLENGE':
        return this.processLegalChallenge(event);

      default:
        return {
          type: 'UNKNOWN',
          sourceSystem: 'legal',
          data: event,
          timestamp: new Date(),
        };
    }
  }

  private async processNotarizationComplete(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    return {
      type: 'CONSENT_UPDATE_REQUEST',
      sourceSystem: 'legal',
      consentId: event.payload.consentId,
      updates: {
        'authority.notarization': {
          notaryId: event.payload.notaryId,
          notaryName: event.payload.notaryName,
          notarizationDate: event.payload.notarizationDate,
          seal: event.payload.seal,
          signature: event.payload.signature,
        },
      },
      timestamp: new Date(),
    };
  }

  async healthCheck(): Promise<HealthCheckResult> {
    const courtHealth = await this.courtFilingClient.ping();
    const notaryHealth = await this.notaryService.ping();

    return {
      healthy: courtHealth.healthy && notaryHealth.healthy,
      components: {
        court: courtHealth,
        notary: notaryHealth,
      },
      lastCheck: new Date(),
    };
  }
}

interface LegalDocument {
  documentId: string;
  type: string;
  jurisdiction: string;
  content: PopulatedLegalDocument;
  pdfData: Buffer;
  signatures: SignatureRecord[];
  notarization: NotarizationRecord | null;
  createdAt: Date;
}

interface LegalDeclaration {
  declarationType: string;
  text: string;
  decisionReference?: string;
  required: boolean;
}

interface LegalSchedule {
  scheduleId: string;
  title: string;
  content: string;
}

interface LegalRegistration {
  registryName: string;
  registrationNumber: string;
  registrationDate: Date;
  expirationDate?: Date;
  status: string;
}
```

---

## 6.4 냉동보존 기관 통합

```typescript
// 냉동보존 기관 어댑터
class CryonicsOrganizationAdapter implements IntegrationAdapter {
  private organizationClients: Map<string, CryonicsOrgClient> = new Map();
  private eventBus: EventBus;

  constructor(config: CryonicsConfig) {
    // 알려진 기관에 대한 클라이언트 초기화
    for (const org of config.organizations) {
      this.organizationClients.set(
        org.id,
        new CryonicsOrgClient(org.endpoint, org.credentials)
      );
    }
  }

  setEventBus(eventBus: EventBus): void {
    this.eventBus = eventBus;
  }

  getTargetFormat(): string {
    return 'CRYONICS_ORG';
  }

  // 냉동보존 기관과 동의 동기화
  async syncConsent(consent: TransformedConsent): Promise<SyncResult> {
    const orgId = consent.scope.cryonicsOrganization;
    const client = this.organizationClients.get(orgId);

    if (!client) {
      throw new Error(`알 수 없는 냉동보존 기관: ${orgId}`);
    }

    // 기관 형식으로 변환
    const orgConsent = this.transformToOrgFormat(consent, orgId);

    // 기관과 동기화
    const result = await client.syncMemberConsent(orgConsent);

    return {
      success: result.success,
      externalId: result.membershipId,
      timestamp: new Date(),
      metadata: {
        organizationId: orgId,
        membershipStatus: result.membershipStatus,
      },
    };
  }

  private transformToOrgFormat(
    consent: TransformedConsent,
    orgId: string
  ): CryonicsOrgConsent {
    return {
      consentId: consent.id,
      memberId: consent.patientId,

      preservationPreferences: {
        type: this.extractPreservationType(consent),
        procedurePreferences: this.extractProcedurePreferences(consent),
        standbyPreferences: this.extractStandbyPreferences(consent),
        transportPreferences: this.extractTransportPreferences(consent),
      },

      revivalPreferences: {
        minimumTechnology: this.extractMinimumTechnology(consent),
        qualityOfLifeRequirements: this.extractQoLRequirements(consent),
        identityCriteria: this.extractIdentityCriteria(consent),
      },

      proxyDesignations: consent.authority.proxyChain?.map(p => ({
        order: p.order,
        name: p.proxy.individual?.name || p.proxy.organization?.name,
        contactInfo: p.proxy.individual?.contactInfo || p.proxy.organization?.contactInfo,
        relationship: p.proxy.individual?.relationship,
        authority: p.proxy.authorityScope.categories,
      })),

      documents: consent.documents?.map(d => ({
        type: d.documentType,
        title: d.title,
        url: d.storage.primaryLocation.locationUri,
        hash: d.integrity.hashValue,
      })),

      effectiveDate: consent.validity.effectiveDate,
      status: consent.validity.status,

      metadata: {
        wiaConsentId: consent.id,
        version: consent.metadata.version,
        lastUpdated: consent.metadata.updatedAt,
      },
    };
  }

  private extractPreservationType(consent: TransformedConsent): string {
    const preservationDecision = consent.decisions.find(
      d => d.question.toLowerCase().includes('보존 유형') ||
           d.question.toLowerCase().includes('preservation type')
    );

    if (preservationDecision?.answer.selectedOption) {
      return preservationDecision.answer.selectedOption;
    }

    // 기본값
    return 'WHOLE_BODY';
  }

  private extractProcedurePreferences(
    consent: TransformedConsent
  ): ProcedurePreferences {
    const preferences: ProcedurePreferences = {
      allowExperimentalProcedures: true,
      cryoprotectantPreferences: [],
      coolingPreferences: null,
    };

    for (const decision of consent.decisions) {
      if (decision.question.toLowerCase().includes('실험적') ||
          decision.question.toLowerCase().includes('experimental')) {
        preferences.allowExperimentalProcedures = decision.answer.binaryValue ?? true;
      }

      if (decision.question.toLowerCase().includes('냉동보호제') ||
          decision.question.toLowerCase().includes('cryoprotectant')) {
        if (decision.answer.selectedOptions) {
          preferences.cryoprotectantPreferences = decision.answer.selectedOptions;
        }
      }

      if (decision.question.toLowerCase().includes('냉각') ||
          decision.question.toLowerCase().includes('cooling')) {
        if (decision.answer.selectedOption) {
          preferences.coolingPreferences = decision.answer.selectedOption;
        }
      }
    }

    return preferences;
  }

  private extractStandbyPreferences(
    consent: TransformedConsent
  ): StandbyPreferences {
    const preferences: StandbyPreferences = {
      standbyRequired: true,
      standbyTeamPreference: null,
      maxStandbyDuration: null,
      relocateForStandby: true,
    };

    for (const decision of consent.decisions) {
      if (decision.question.toLowerCase().includes('대기') ||
          decision.question.toLowerCase().includes('standby')) {
        if (decision.decisionType === 'BINARY') {
          preferences.standbyRequired = decision.answer.binaryValue ?? true;
        }

        if (decision.decisionType === 'THRESHOLD') {
          preferences.maxStandbyDuration = decision.answer.thresholdValue;
        }

        if (decision.decisionType === 'CHOICE') {
          preferences.standbyTeamPreference = decision.answer.selectedOption;
        }
      }

      if (decision.question.toLowerCase().includes('이전') ||
          decision.question.toLowerCase().includes('relocate')) {
        preferences.relocateForStandby = decision.answer.binaryValue ?? true;
      }
    }

    return preferences;
  }

  // 기관 이벤트 처리
  async processIncomingEvent(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    switch (event.type) {
      case 'MEMBERSHIP_STATUS_CHANGE':
        return this.processMembershipChange(event);

      case 'STANDBY_ACTIVATED':
        return this.processStandbyActivation(event);

      case 'PRESERVATION_INITIATED':
        return this.processPreservationInitiated(event);

      case 'FACILITY_TRANSFER':
        return this.processFacilityTransfer(event);

      default:
        return {
          type: 'UNKNOWN',
          sourceSystem: 'cryonics',
          data: event,
          timestamp: new Date(),
        };
    }
  }

  private async processMembershipChange(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    // 멤버십 상태 변경은 동의 유효성에 영향을 줄 수 있음
    return {
      type: 'STATUS_UPDATE',
      sourceSystem: 'cryonics',
      patientId: event.payload.memberId,
      data: {
        membershipStatus: event.payload.newStatus,
        effectiveDate: event.payload.effectiveDate,
        reason: event.payload.reason,
      },
      timestamp: new Date(),
    };
  }

  private async processStandbyActivation(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    // 대기 활성화는 동의 검토를 트리거
    return {
      type: 'CONSENT_QUERY',
      sourceSystem: 'cryonics',
      patientId: event.payload.memberId,
      data: {
        queryType: 'EFFECTIVE_CONSENT',
        decisionType: 'STANDBY',
        context: {
          standbyReason: event.payload.reason,
          location: event.payload.location,
          prognosis: event.payload.prognosis,
        },
        urgency: 'HIGH',
      },
      timestamp: new Date(),
    };
  }

  private async processPreservationInitiated(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    // 중요 이벤트 - 모든 동의 결정이 문서화되었는지 확인
    return {
      type: 'CONSENT_UPDATE_REQUEST',
      sourceSystem: 'cryonics',
      patientId: event.payload.memberId,
      consentId: event.payload.consentId,
      updates: {
        'preservationRecord': {
          initiatedAt: event.payload.timestamp,
          location: event.payload.location,
          team: event.payload.team,
          procedures: event.payload.procedures,
        },
      },
      timestamp: new Date(),
    };
  }

  async healthCheck(): Promise<HealthCheckResult> {
    const results: Record<string, HealthCheckResult> = {};

    for (const [orgId, client] of this.organizationClients) {
      results[orgId] = await client.healthCheck();
    }

    const allHealthy = Object.values(results).every(r => r.healthy);

    return {
      healthy: allHealthy,
      components: results,
      lastCheck: new Date(),
    };
  }
}

interface CryonicsOrgConsent {
  consentId: string;
  memberId: string;
  preservationPreferences: {
    type: string;
    procedurePreferences: ProcedurePreferences;
    standbyPreferences: StandbyPreferences;
    transportPreferences: TransportPreferences;
  };
  revivalPreferences: {
    minimumTechnology: string[];
    qualityOfLifeRequirements: QoLRequirements;
    identityCriteria: IdentityCriteria;
  };
  proxyDesignations: any[];
  documents: any[];
  effectiveDate: Date;
  status: ConsentStatus;
  metadata: any;
}

interface ProcedurePreferences {
  allowExperimentalProcedures: boolean;
  cryoprotectantPreferences: string[];
  coolingPreferences: string | null;
}

interface StandbyPreferences {
  standbyRequired: boolean;
  standbyTeamPreference: string | null;
  maxStandbyDuration: number | null;
  relocateForStandby: boolean;
}

interface TransportPreferences {
  maxTransportTime: number | null;
  preferredTransportMethod: string | null;
  internationalTransportAllowed: boolean;
}
```

---

## 6.5 신원 확인 통합

```typescript
// 신원 확인 어댑터
class IdentityVerificationAdapter implements IntegrationAdapter {
  private biometricService: BiometricService;
  private documentVerificationService: DocumentVerificationService;
  private knowledgeBasedAuthService: KBAService;
  private blockchainIdentityService: BlockchainIdentityService;
  private eventBus: EventBus;

  constructor(config: IdentityConfig) {
    this.biometricService = new BiometricService(config.biometric);
    this.documentVerificationService = new DocumentVerificationService(config.documents);
    this.knowledgeBasedAuthService = new KBAService(config.kba);
    this.blockchainIdentityService = new BlockchainIdentityService(config.blockchain);
  }

  setEventBus(eventBus: EventBus): void {
    this.eventBus = eventBus;
  }

  getTargetFormat(): string {
    return 'IDENTITY';
  }

  // 동의 작업을 위한 신원 확인
  async verifyIdentity(
    request: IdentityVerificationRequest
  ): Promise<IdentityVerificationResult> {
    const verifications: VerificationComponent[] = [];

    // 문서 확인
    if (request.documents && request.documents.length > 0) {
      const docResult = await this.verifyDocuments(request.documents);
      verifications.push({
        type: 'DOCUMENT',
        result: docResult,
        weight: 0.3,
      });
    }

    // 생체 인식 확인
    if (request.biometricData) {
      const bioResult = await this.verifyBiometrics(
        request.userId,
        request.biometricData
      );
      verifications.push({
        type: 'BIOMETRIC',
        result: bioResult,
        weight: 0.4,
      });
    }

    // 지식 기반 인증
    if (request.kbaResponses) {
      const kbaResult = await this.verifyKBA(
        request.userId,
        request.kbaResponses
      );
      verifications.push({
        type: 'KBA',
        result: kbaResult,
        weight: 0.2,
      });
    }

    // 블록체인 신원 확인
    if (request.blockchainIdentity) {
      const bcResult = await this.verifyBlockchainIdentity(
        request.userId,
        request.blockchainIdentity
      );
      verifications.push({
        type: 'BLOCKCHAIN',
        result: bcResult,
        weight: 0.1,
      });
    }

    // 전체 점수 계산
    const overallScore = this.calculateOverallScore(verifications);
    const verified = overallScore >= request.requiredConfidence;

    return {
      verified,
      overallScore,
      components: verifications,
      timestamp: new Date(),
      validUntil: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24시간
      verificationId: generateVerificationId(),
    };
  }

  private async verifyDocuments(
    documents: VerificationDocument[]
  ): Promise<ComponentVerificationResult> {
    const results: DocumentVerificationResult[] = [];

    for (const doc of documents) {
      const result = await this.documentVerificationService.verify(doc);
      results.push(result);
    }

    const allValid = results.every(r => r.valid);
    const avgConfidence = results.reduce((sum, r) => sum + r.confidence, 0) / results.length;

    return {
      passed: allValid,
      confidence: avgConfidence,
      details: results,
    };
  }

  private async verifyBiometrics(
    userId: string,
    biometricData: BiometricData
  ): Promise<ComponentVerificationResult> {
    // 저장된 생체 템플릿 가져오기
    const storedTemplate = await this.biometricService.getTemplate(userId);

    if (!storedTemplate) {
      return {
        passed: false,
        confidence: 0,
        details: { error: '등록된 생체 템플릿이 없습니다' },
      };
    }

    // 비교
    const matchResult = await this.biometricService.compare(
      biometricData,
      storedTemplate
    );

    return {
      passed: matchResult.match,
      confidence: matchResult.score,
      details: {
        biometricType: biometricData.type,
        matchScore: matchResult.score,
        threshold: matchResult.threshold,
      },
    };
  }

  private async verifyKBA(
    userId: string,
    responses: KBAResponse[]
  ): Promise<ComponentVerificationResult> {
    const result = await this.knowledgeBasedAuthService.verify(userId, responses);

    return {
      passed: result.passed,
      confidence: result.score,
      details: {
        questionsAsked: result.totalQuestions,
        correctAnswers: result.correctAnswers,
        requiredCorrect: result.requiredCorrect,
      },
    };
  }

  private async verifyBlockchainIdentity(
    userId: string,
    blockchainId: BlockchainIdentity
  ): Promise<ComponentVerificationResult> {
    // 온체인 신원 확인
    const verification = await this.blockchainIdentityService.verify(
      userId,
      blockchainId
    );

    return {
      passed: verification.valid,
      confidence: verification.valid ? 1.0 : 0,
      details: {
        network: blockchainId.network,
        address: blockchainId.address,
        attestations: verification.attestations,
      },
    };
  }

  private calculateOverallScore(verifications: VerificationComponent[]): number {
    const totalWeight = verifications.reduce((sum, v) => sum + v.weight, 0);
    const weightedScore = verifications.reduce(
      (sum, v) => sum + (v.result.passed ? v.result.confidence * v.weight : 0),
      0
    );

    return totalWeight > 0 ? weightedScore / totalWeight : 0;
  }

  // 신원 레코드 생성/업데이트
  async createIdentityRecord(
    userId: string,
    identity: IdentityData
  ): Promise<IdentityRecord> {
    // 먼저 신원 확인
    const verification = await this.verifyIdentity({
      userId,
      documents: identity.documents,
      biometricData: identity.biometrics,
      requiredConfidence: 0.9,
    });

    if (!verification.verified) {
      throw new VerificationError('신원 확인 실패');
    }

    // 생체 템플릿 저장
    if (identity.biometrics) {
      await this.biometricService.storeTemplate(userId, identity.biometrics);
    }

    // 블록체인 신원 앵커 생성
    const blockchainAnchor = await this.blockchainIdentityService.createAnchor(
      userId,
      {
        documentHashes: identity.documents.map(d => d.hash),
        biometricHash: identity.biometrics?.hash,
        verificationId: verification.verificationId,
      }
    );

    return {
      userId,
      verificationId: verification.verificationId,
      verificationDate: new Date(),
      verificationLevel: this.calculateVerificationLevel(verification),
      blockchainAnchor,
      documents: identity.documents.map(d => d.type),
      biometricTypes: identity.biometrics ? [identity.biometrics.type] : [],
    };
  }

  // 정기 재확인
  async scheduleReverification(
    userId: string,
    schedule: ReverificationSchedule
  ): Promise<void> {
    await this.scheduler.schedule({
      type: 'IDENTITY_REVERIFICATION',
      userId,
      scheduledFor: schedule.nextReverification,
      recurrence: schedule.frequency,
    });
  }

  async syncConsent(consent: TransformedConsent): Promise<SyncResult> {
    // 동기화 전 환자 신원 확인
    const verification = await this.verifyIdentity({
      userId: consent.patientId,
      requiredConfidence: 0.8,
    });

    if (!verification.verified) {
      return {
        success: false,
        error: '신원 확인 실패',
      };
    }

    return {
      success: true,
      externalId: verification.verificationId,
      timestamp: new Date(),
    };
  }

  async processIncomingEvent(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    return {
      type: 'IDENTITY_EVENT',
      sourceSystem: 'identity',
      data: event,
      timestamp: new Date(),
    };
  }

  async healthCheck(): Promise<HealthCheckResult> {
    const bioHealth = await this.biometricService.healthCheck();
    const docHealth = await this.documentVerificationService.healthCheck();
    const kbaHealth = await this.knowledgeBasedAuthService.healthCheck();
    const bcHealth = await this.blockchainIdentityService.healthCheck();

    return {
      healthy: bioHealth.healthy && docHealth.healthy && kbaHealth.healthy && bcHealth.healthy,
      components: {
        biometric: bioHealth,
        document: docHealth,
        kba: kbaHealth,
        blockchain: bcHealth,
      },
      lastCheck: new Date(),
    };
  }
}

interface IdentityVerificationRequest {
  userId: string;
  documents?: VerificationDocument[];
  biometricData?: BiometricData;
  kbaResponses?: KBAResponse[];
  blockchainIdentity?: BlockchainIdentity;
  requiredConfidence: number;
}

interface IdentityVerificationResult {
  verified: boolean;
  overallScore: number;
  components: VerificationComponent[];
  timestamp: Date;
  validUntil: Date;
  verificationId: string;
}

interface VerificationComponent {
  type: string;
  result: ComponentVerificationResult;
  weight: number;
}

interface ComponentVerificationResult {
  passed: boolean;
  confidence: number;
  details: any;
}

interface BiometricData {
  type: 'FINGERPRINT' | 'FACE' | 'IRIS' | 'VOICE';
  data: Buffer;
  hash?: string;
}

interface BlockchainIdentity {
  network: string;
  address: string;
  signature?: string;
}
```

---

## 6.6 이벤트 기반 통합

```typescript
// 시스템 간 통신을 위한 이벤트 버스
class ConsentEventBus {
  private subscribers: Map<string, EventSubscriber[]> = new Map();
  private messageQueue: MessageQueueClient;
  private deadLetterQueue: DeadLetterQueue;

  constructor(config: EventBusConfig) {
    this.messageQueue = new MessageQueueClient(config.messageQueue);
    this.deadLetterQueue = new DeadLetterQueue(config.deadLetter);
  }

  // 이벤트 구독
  subscribe(eventType: string, subscriber: EventSubscriber): void {
    if (!this.subscribers.has(eventType)) {
      this.subscribers.set(eventType, []);
    }
    this.subscribers.get(eventType)!.push(subscriber);
  }

  // 이벤트 발행
  async emit(event: ConsentEvent): Promise<void> {
    // 이벤트 보강
    const enrichedEvent = this.enrichEvent(event);

    // 내구성을 위해 메시지 큐에 저장
    await this.messageQueue.publish(enrichedEvent);

    // 로컬 구독자에게 알림
    const subscribers = this.subscribers.get(event.type) || [];
    const allSubscribers = [
      ...subscribers,
      ...(this.subscribers.get('*') || []), // 와일드카드 구독자
    ];

    for (const subscriber of allSubscribers) {
      try {
        await subscriber.handle(enrichedEvent);
      } catch (error) {
        await this.handleSubscriberError(subscriber, enrichedEvent, error);
      }
    }
  }

  private enrichEvent(event: ConsentEvent): EnrichedConsentEvent {
    return {
      ...event,
      eventId: generateEventId(),
      timestamp: event.timestamp || new Date(),
      source: event.source || 'consent-system',
      correlationId: event.correlationId || generateCorrelationId(),
    };
  }

  private async handleSubscriberError(
    subscriber: EventSubscriber,
    event: EnrichedConsentEvent,
    error: Error
  ): Promise<void> {
    console.error(`구독자 ${subscriber.name}가 이벤트 ${event.eventId}에서 실패:`, error);

    // 재시도 로직
    if (subscriber.retryPolicy) {
      const shouldRetry = await this.shouldRetry(subscriber, event);
      if (shouldRetry) {
        await this.scheduleRetry(subscriber, event);
        return;
      }
    }

    // 데드 레터 큐로 전송
    await this.deadLetterQueue.add({
      event,
      subscriber: subscriber.name,
      error: error.message,
      timestamp: new Date(),
    });
  }

  // 메시지 큐에서 소비 시작
  async startConsuming(): Promise<void> {
    await this.messageQueue.consume(async (message) => {
      const event = JSON.parse(message.body) as EnrichedConsentEvent;
      await this.processEvent(event);
      await message.ack();
    });
  }

  private async processEvent(event: EnrichedConsentEvent): Promise<void> {
    const subscribers = [
      ...(this.subscribers.get(event.type) || []),
      ...(this.subscribers.get('*') || []),
    ];

    await Promise.all(
      subscribers.map(s => this.safeHandle(s, event))
    );
  }

  private async safeHandle(
    subscriber: EventSubscriber,
    event: EnrichedConsentEvent
  ): Promise<void> {
    try {
      await subscriber.handle(event);
    } catch (error) {
      await this.handleSubscriberError(subscriber, event, error);
    }
  }
}

// 이벤트 타입
interface ConsentEvent {
  type: ConsentEventType;
  consentId?: string;
  patientId?: string;
  data: Record<string, any>;
  source?: string;
  timestamp?: Date;
  correlationId?: string;
}

interface EnrichedConsentEvent extends ConsentEvent {
  eventId: string;
  timestamp: Date;
  source: string;
  correlationId: string;
}

enum ConsentEventType {
  // 동의 생명주기
  CONSENT_CREATED = 'CONSENT_CREATED',
  CONSENT_UPDATED = 'CONSENT_UPDATED',
  CONSENT_ACTIVATED = 'CONSENT_ACTIVATED',
  CONSENT_SUSPENDED = 'CONSENT_SUSPENDED',
  CONSENT_REVOKED = 'CONSENT_REVOKED',
  CONSENT_EXPIRED = 'CONSENT_EXPIRED',

  // 의사결정 이벤트
  DECISION_QUERIED = 'DECISION_QUERIED',
  DECISION_EXECUTED = 'DECISION_EXECUTED',
  DECISION_ESCALATED = 'DECISION_ESCALATED',

  // 대리인 이벤트
  PROXY_DESIGNATED = 'PROXY_DESIGNATED',
  PROXY_ACTIVATED = 'PROXY_ACTIVATED',
  PROXY_DEACTIVATED = 'PROXY_DEACTIVATED',

  // 검토 이벤트
  REVIEW_SCHEDULED = 'REVIEW_SCHEDULED',
  REVIEW_REMINDER = 'REVIEW_REMINDER',
  REVIEW_COMPLETED = 'REVIEW_COMPLETED',
  REVIEW_OVERDUE = 'REVIEW_OVERDUE',

  // 통합 이벤트
  SYNC_COMPLETED = 'SYNC_COMPLETED',
  SYNC_FAILED = 'SYNC_FAILED',
  EXTERNAL_UPDATE = 'EXTERNAL_UPDATE',

  // 응급 이벤트
  EMERGENCY_ACTIVATED = 'EMERGENCY_ACTIVATED',
  EMERGENCY_DECISION = 'EMERGENCY_DECISION',
}

// 이벤트 구독자 인터페이스
interface EventSubscriber {
  name: string;
  handle(event: EnrichedConsentEvent): Promise<void>;
  retryPolicy?: RetryPolicy;
}

interface RetryPolicy {
  maxRetries: number;
  backoffStrategy: 'LINEAR' | 'EXPONENTIAL';
  initialDelay: number;
  maxDelay: number;
}

// 예제 이벤트 핸들러
class AuditEventHandler implements EventSubscriber {
  name = 'AuditEventHandler';

  constructor(private auditService: AuditService) {}

  async handle(event: EnrichedConsentEvent): Promise<void> {
    await this.auditService.logEvent({
      eventId: event.eventId,
      eventType: event.type,
      consentId: event.consentId,
      patientId: event.patientId,
      data: event.data,
      source: event.source,
      timestamp: event.timestamp,
      correlationId: event.correlationId,
    });
  }
}

class NotificationEventHandler implements EventSubscriber {
  name = 'NotificationEventHandler';

  constructor(private notificationService: NotificationService) {}

  async handle(event: EnrichedConsentEvent): Promise<void> {
    const notificationConfig = this.getNotificationConfig(event.type);

    if (!notificationConfig) return;

    const recipients = await this.getRecipients(event);

    for (const recipient of recipients) {
      await this.notificationService.send({
        recipientId: recipient.id,
        channel: recipient.preferredChannel,
        template: notificationConfig.template,
        data: {
          eventType: event.type,
          ...event.data,
        },
      });
    }
  }

  private getNotificationConfig(eventType: ConsentEventType): NotificationConfig | null {
    const configs: Partial<Record<ConsentEventType, NotificationConfig>> = {
      [ConsentEventType.CONSENT_ACTIVATED]: {
        template: 'consent-activated',
        channels: ['EMAIL', 'SMS'],
      },
      [ConsentEventType.CONSENT_REVOKED]: {
        template: 'consent-revoked',
        channels: ['EMAIL', 'SMS', 'PUSH'],
      },
      [ConsentEventType.REVIEW_REMINDER]: {
        template: 'review-reminder',
        channels: ['EMAIL'],
      },
      [ConsentEventType.PROXY_DESIGNATED]: {
        template: 'proxy-designation',
        channels: ['EMAIL'],
      },
      [ConsentEventType.EMERGENCY_ACTIVATED]: {
        template: 'emergency-alert',
        channels: ['SMS', 'PUSH', 'PHONE'],
      },
    };

    return configs[eventType] || null;
  }

  private async getRecipients(event: EnrichedConsentEvent): Promise<Recipient[]> {
    // 환자 가져오기
    const recipients: Recipient[] = [];

    if (event.patientId) {
      const patient = await this.patientService.get(event.patientId);
      if (patient) {
        recipients.push({
          id: patient.id,
          preferredChannel: patient.preferredNotificationChannel || 'EMAIL',
        });
      }
    }

    // 특정 이벤트에 대해 대리인 가져오기
    if ([
      ConsentEventType.EMERGENCY_ACTIVATED,
      ConsentEventType.CONSENT_ACTIVATED,
    ].includes(event.type as ConsentEventType)) {
      const proxies = await this.proxyService.getProxiesForPatient(event.patientId!);
      for (const proxy of proxies) {
        recipients.push({
          id: proxy.proxyId,
          preferredChannel: proxy.individual?.contactInfo.preferredChannel || 'EMAIL',
        });
      }
    }

    return recipients;
  }
}

class SyncEventHandler implements EventSubscriber {
  name = 'SyncEventHandler';
  retryPolicy = {
    maxRetries: 3,
    backoffStrategy: 'EXPONENTIAL' as const,
    initialDelay: 1000,
    maxDelay: 30000,
  };

  constructor(private integrationOrchestrator: ConsentIntegrationOrchestrator) {}

  async handle(event: EnrichedConsentEvent): Promise<void> {
    // 특정 이벤트에서만 동기화
    if (![
      ConsentEventType.CONSENT_CREATED,
      ConsentEventType.CONSENT_UPDATED,
      ConsentEventType.CONSENT_ACTIVATED,
      ConsentEventType.CONSENT_REVOKED,
    ].includes(event.type as ConsentEventType)) {
      return;
    }

    // 동의 가져오기
    const consent = await this.consentService.get(event.consentId!);
    if (!consent) return;

    // 구성된 모든 시스템에 동기화
    await this.integrationOrchestrator.synchronizeConsent(consent, [
      'healthcare',
      'legal',
      'cryonics',
    ]);
  }
}
```

---

*다음 장: 보안 - 동의 데이터 보호를 위한 종합 보안 조치*
