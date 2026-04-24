# Chapter 6: Integration

## System Integration Patterns

This chapter defines integration patterns for connecting the WIA Cryo-Consent system with healthcare systems, legal frameworks, cryonics organizations, and regulatory bodies.

---

## 6.1 Integration Architecture

```typescript
// Integration orchestrator
class ConsentIntegrationOrchestrator {
  private adapters: Map<string, IntegrationAdapter> = new Map();
  private eventBus: EventBus;
  private transformerService: DataTransformerService;
  private auditService: AuditService;

  constructor(config: IntegrationConfig) {
    this.eventBus = new EventBus(config.eventBus);
    this.transformerService = new DataTransformerService();
    this.auditService = new AuditService(config.audit);

    // Register adapters
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

  // Synchronize consent across systems
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
          error: `Adapter not found for ${target}`,
        });
        continue;
      }

      try {
        // Transform consent to target format
        const transformed = await this.transformerService.transform(
          consent,
          adapter.getTargetFormat()
        );

        // Send to target system
        const syncResult = await adapter.syncConsent(transformed);

        results.push({
          system: target,
          success: syncResult.success,
          externalId: syncResult.externalId,
          timestamp: syncResult.timestamp,
        });

        // Log synchronization
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

  // Handle incoming events from external systems
  async handleExternalEvent(event: ExternalIntegrationEvent): Promise<void> {
    const adapter = this.adapters.get(event.sourceSystem);
    if (!adapter) {
      throw new Error(`Unknown source system: ${event.sourceSystem}`);
    }

    // Process event
    const processedEvent = await adapter.processIncomingEvent(event);

    // Route to appropriate handler
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
    // Validate external update request
    const validation = await this.validateExternalUpdate(event);
    if (!validation.valid) {
      await this.notifyUpdateRejection(event, validation.reason);
      return;
    }

    // Apply update
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

// Base adapter interface
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

## 6.2 Healthcare System Integration

```typescript
// Healthcare system adapter (HL7 FHIR compatible)
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

  // Sync consent to healthcare system
  async syncConsent(consent: TransformedConsent): Promise<SyncResult> {
    // Convert to FHIR Consent resource
    const fhirConsent = this.toFHIRConsent(consent);

    // Check if exists
    const existing = await this.fhirClient.search('Consent', {
      identifier: consent.id,
    });

    let result: any;
    if (existing.total > 0) {
      // Update existing
      result = await this.fhirClient.update(
        'Consent',
        existing.entry[0].resource.id,
        fhirConsent
      );
    } else {
      // Create new
      result = await this.fhirClient.create('Consent', fhirConsent);
    }

    // Also sync related resources
    await this.syncRelatedResources(consent, result.id);

    return {
      success: true,
      externalId: result.id,
      timestamp: new Date(),
    };
  }

  // Convert WIA consent to FHIR Consent resource
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

  // Sync related resources (Patient, Practitioner references)
  private async syncRelatedResources(
    consent: TransformedConsent,
    fhirConsentId: string
  ): Promise<void> {
    // Ensure Patient exists
    await this.ensurePatientExists(consent.patientId);

    // Sync DocumentReference resources for documents
    for (const doc of consent.documents || []) {
      await this.syncDocumentReference(doc, fhirConsentId);
    }

    // Sync RelatedPerson for proxies
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
          display: 'Consent Document',
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

  // Process incoming events from healthcare system
  async processIncomingEvent(
    event: ExternalIntegrationEvent
  ): Promise<ProcessedIntegrationEvent> {
    // Parse HL7 or FHIR message
    let parsedEvent: any;

    if (event.format === 'HL7V2') {
      parsedEvent = await this.parseHL7Event(event.payload);
    } else if (event.format === 'FHIR') {
      parsedEvent = await this.parseFHIREvent(event.payload);
    }

    // Map to internal event format
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

    // Handle different HL7 message types
    switch (message.messageType) {
      case 'ADT^A01': // Admission
        return {
          type: 'PATIENT_ADMISSION',
          patientId: message.patient.id,
          data: {
            admissionDate: message.admission.date,
            facility: message.admission.facility,
          },
        };

      case 'ADT^A03': // Discharge
        return {
          type: 'PATIENT_DISCHARGE',
          patientId: message.patient.id,
          data: {
            dischargeDate: message.discharge.date,
            disposition: message.discharge.disposition,
          },
        };

      case 'ADT^A08': // Update patient info
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

// FHIR type definitions
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

## 6.3 Legal System Integration

```typescript
// Legal system adapter
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

  // Sync consent as legal document
  async syncConsent(consent: TransformedConsent): Promise<SyncResult> {
    // Generate legal document
    const legalDoc = await this.generateLegalDocument(consent);

    // If notarization required, initiate
    if (consent.authority.notarization) {
      await this.processNotarization(consent, legalDoc);
    }

    // Register with appropriate courts/registries
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

  // Generate comprehensive legal document
  private async generateLegalDocument(
    consent: TransformedConsent
  ): Promise<LegalDocument> {
    // Get template for jurisdiction
    const template = await this.legalDocumentService.getTemplate(
      consent.scope.jurisdictions[0] || 'US',
      'ADVANCE_DIRECTIVE'
    );

    // Populate template
    const populatedDoc = this.populateTemplate(template, consent);

    // Generate PDF
    const pdf = await this.legalDocumentService.generatePDF(populatedDoc);

    return {
      documentId: generateDocumentId(),
      type: 'CRYONICS_CONSENT',
      jurisdiction: consent.scope.jurisdictions[0] || 'US',
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

    // Declaration of intent
    declarations.push({
      declarationType: 'INTENT',
      text: `I, the undersigned, being of sound mind and legal capacity, do hereby declare my intent to be cryonically preserved upon my legal death, in accordance with the terms set forth in this document.`,
      required: true,
    });

    // Declaration of understanding
    declarations.push({
      declarationType: 'UNDERSTANDING',
      text: `I understand that cryonics preservation does not guarantee future revival, and that the technology for revival does not currently exist. I make this decision with full knowledge of these uncertainties.`,
      required: true,
    });

    // Specific decision declarations
    for (const decision of consent.decisions) {
      if (decision.decisionType === 'BINARY' && decision.answer.binaryValue) {
        declarations.push({
          declarationType: 'SPECIFIC_CONSENT',
          text: `I consent to: ${decision.question}`,
          decisionReference: decision.decisionId,
          required: false,
        });
      }
    }

    // Proxy declarations
    if (consent.authority.proxyChain?.length > 0) {
      declarations.push({
        declarationType: 'PROXY_DESIGNATION',
        text: `I designate the following individuals/organizations to make decisions on my behalf in the order listed, should I be unable to make decisions myself.`,
        required: false,
      });
    }

    return declarations;
  }

  private generateSchedules(consent: TransformedConsent): LegalSchedule[] {
    const schedules: LegalSchedule[] = [];

    // Schedule A: Preservation Preferences
    schedules.push({
      scheduleId: 'A',
      title: 'Preservation Preferences',
      content: this.generatePreservationSchedule(consent),
    });

    // Schedule B: Proxy Designations
    if (consent.authority.proxyChain?.length > 0) {
      schedules.push({
        scheduleId: 'B',
        title: 'Proxy Designations',
        content: this.generateProxySchedule(consent),
      });
    }

    // Schedule C: Revival Preferences
    const revivalDecisions = consent.decisions.filter(
      d => d.context?.scenarioDescription?.toLowerCase().includes('revival')
    );
    if (revivalDecisions.length > 0) {
      schedules.push({
        scheduleId: 'C',
        title: 'Revival Preferences',
        content: this.generateRevivalSchedule(consent, revivalDecisions),
      });
    }

    // Schedule D: Asset Instructions
    const assetDecisions = consent.decisions.filter(
      d => d.context?.scenarioDescription?.toLowerCase().includes('asset')
    );
    if (assetDecisions.length > 0) {
      schedules.push({
        scheduleId: 'D',
        title: 'Asset Management Instructions',
        content: this.generateAssetSchedule(consent, assetDecisions),
      });
    }

    return schedules;
  }

  // Process notarization
  async processNotarization(
    consent: TransformedConsent,
    legalDoc: LegalDocument
  ): Promise<NotarizationResult> {
    // Initiate notarization request
    const notaryRequest = {
      documentId: legalDoc.documentId,
      documentType: 'ADVANCE_DIRECTIVE',
      signatoryName: consent.patientId, // Would resolve to actual name
      documentHash: await this.calculateHash(legalDoc.pdfData),
      jurisdiction: consent.scope.jurisdictions[0] || 'US',
      requestedDate: new Date(),
    };

    // Submit to notary service
    const notaryResponse = await this.notaryService.requestNotarization(notaryRequest);

    return {
      notarizationId: notaryResponse.id,
      status: notaryResponse.status,
      notaryId: notaryResponse.notaryId,
      appointmentDate: notaryResponse.appointmentDate,
    };
  }

  // Register with legal authorities
  async registerWithLegalAuthorities(
    consent: TransformedConsent,
    legalDoc: LegalDocument
  ): Promise<LegalRegistration[]> {
    const registrations: LegalRegistration[] = [];

    for (const jurisdiction of consent.scope.jurisdictions || []) {
      // Determine appropriate registry for jurisdiction
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
          console.error(`Failed to register in ${jurisdiction}:`, error);
        }
      }
    }

    return registrations;
  }

  private getRegistryForJurisdiction(jurisdiction: string): LegalRegistry | null {
    const registries: Record<string, LegalRegistry> = {
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
      // Add more jurisdictions
    };

    return registries[jurisdiction] || null;
  }

  private async registerWithRegistry(
    registry: LegalRegistry,
    consent: TransformedConsent,
    legalDoc: LegalDocument
  ): Promise<LegalRegistration> {
    // Submit to registry
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

## 6.4 Cryonics Organization Integration

```typescript
// Cryonics organization adapter
class CryonicsOrganizationAdapter implements IntegrationAdapter {
  private organizationClients: Map<string, CryonicsOrgClient> = new Map();
  private eventBus: EventBus;

  constructor(config: CryonicsConfig) {
    // Initialize clients for known organizations
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

  // Sync consent with cryonics organization
  async syncConsent(consent: TransformedConsent): Promise<SyncResult> {
    const orgId = consent.scope.cryonicsOrganization;
    const client = this.organizationClients.get(orgId);

    if (!client) {
      throw new Error(`Unknown cryonics organization: ${orgId}`);
    }

    // Transform to organization's format
    const orgConsent = this.transformToOrgFormat(consent, orgId);

    // Sync with organization
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
      d => d.question.toLowerCase().includes('preservation type')
    );

    if (preservationDecision?.answer.selectedOption) {
      return preservationDecision.answer.selectedOption;
    }

    // Default
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
      if (decision.question.toLowerCase().includes('experimental')) {
        preferences.allowExperimentalProcedures = decision.answer.binaryValue ?? true;
      }

      if (decision.question.toLowerCase().includes('cryoprotectant')) {
        if (decision.answer.selectedOptions) {
          preferences.cryoprotectantPreferences = decision.answer.selectedOptions;
        }
      }

      if (decision.question.toLowerCase().includes('cooling')) {
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
      if (decision.question.toLowerCase().includes('standby')) {
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

      if (decision.question.toLowerCase().includes('relocate')) {
        preferences.relocateForStandby = decision.answer.binaryValue ?? true;
      }
    }

    return preferences;
  }

  // Handle organization events
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
    // Membership status change may affect consent validity
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
    // Standby activation triggers consent review
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
    // Major event - ensure all consent decisions are documented
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

## 6.5 Identity Verification Integration

```typescript
// Identity verification adapter
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

  // Verify identity for consent operations
  async verifyIdentity(
    request: IdentityVerificationRequest
  ): Promise<IdentityVerificationResult> {
    const verifications: VerificationComponent[] = [];

    // Document verification
    if (request.documents && request.documents.length > 0) {
      const docResult = await this.verifyDocuments(request.documents);
      verifications.push({
        type: 'DOCUMENT',
        result: docResult,
        weight: 0.3,
      });
    }

    // Biometric verification
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

    // Knowledge-based authentication
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

    // Blockchain identity verification
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

    // Calculate overall score
    const overallScore = this.calculateOverallScore(verifications);
    const verified = overallScore >= request.requiredConfidence;

    return {
      verified,
      overallScore,
      components: verifications,
      timestamp: new Date(),
      validUntil: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24 hours
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
    // Get stored biometric template
    const storedTemplate = await this.biometricService.getTemplate(userId);

    if (!storedTemplate) {
      return {
        passed: false,
        confidence: 0,
        details: { error: 'No biometric template on file' },
      };
    }

    // Compare
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
    // Verify on-chain identity
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

  // Create/update identity record
  async createIdentityRecord(
    userId: string,
    identity: IdentityData
  ): Promise<IdentityRecord> {
    // Verify identity first
    const verification = await this.verifyIdentity({
      userId,
      documents: identity.documents,
      biometricData: identity.biometrics,
      requiredConfidence: 0.9,
    });

    if (!verification.verified) {
      throw new VerificationError('Identity verification failed');
    }

    // Store biometric template
    if (identity.biometrics) {
      await this.biometricService.storeTemplate(userId, identity.biometrics);
    }

    // Create blockchain identity anchor
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

  // Periodic re-verification
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
    // Verify patient identity before syncing
    const verification = await this.verifyIdentity({
      userId: consent.patientId,
      requiredConfidence: 0.8,
    });

    if (!verification.verified) {
      return {
        success: false,
        error: 'Identity verification failed',
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

## 6.6 Event-Driven Integration

```typescript
// Event bus for cross-system communication
class ConsentEventBus {
  private subscribers: Map<string, EventSubscriber[]> = new Map();
  private messageQueue: MessageQueueClient;
  private deadLetterQueue: DeadLetterQueue;

  constructor(config: EventBusConfig) {
    this.messageQueue = new MessageQueueClient(config.messageQueue);
    this.deadLetterQueue = new DeadLetterQueue(config.deadLetter);
  }

  // Subscribe to events
  subscribe(eventType: string, subscriber: EventSubscriber): void {
    if (!this.subscribers.has(eventType)) {
      this.subscribers.set(eventType, []);
    }
    this.subscribers.get(eventType)!.push(subscriber);
  }

  // Publish event
  async emit(event: ConsentEvent): Promise<void> {
    // Enrich event
    const enrichedEvent = this.enrichEvent(event);

    // Persist to message queue for durability
    await this.messageQueue.publish(enrichedEvent);

    // Notify local subscribers
    const subscribers = this.subscribers.get(event.type) || [];
    const allSubscribers = [
      ...subscribers,
      ...(this.subscribers.get('*') || []), // Wildcard subscribers
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
    console.error(`Subscriber ${subscriber.name} failed for event ${event.eventId}:`, error);

    // Retry logic
    if (subscriber.retryPolicy) {
      const shouldRetry = await this.shouldRetry(subscriber, event);
      if (shouldRetry) {
        await this.scheduleRetry(subscriber, event);
        return;
      }
    }

    // Send to dead letter queue
    await this.deadLetterQueue.add({
      event,
      subscriber: subscriber.name,
      error: error.message,
      timestamp: new Date(),
    });
  }

  // Start consuming from message queue
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

// Event types
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
  // Consent lifecycle
  CONSENT_CREATED = 'CONSENT_CREATED',
  CONSENT_UPDATED = 'CONSENT_UPDATED',
  CONSENT_ACTIVATED = 'CONSENT_ACTIVATED',
  CONSENT_SUSPENDED = 'CONSENT_SUSPENDED',
  CONSENT_REVOKED = 'CONSENT_REVOKED',
  CONSENT_EXPIRED = 'CONSENT_EXPIRED',

  // Decision events
  DECISION_QUERIED = 'DECISION_QUERIED',
  DECISION_EXECUTED = 'DECISION_EXECUTED',
  DECISION_ESCALATED = 'DECISION_ESCALATED',

  // Proxy events
  PROXY_DESIGNATED = 'PROXY_DESIGNATED',
  PROXY_ACTIVATED = 'PROXY_ACTIVATED',
  PROXY_DEACTIVATED = 'PROXY_DEACTIVATED',

  // Review events
  REVIEW_SCHEDULED = 'REVIEW_SCHEDULED',
  REVIEW_REMINDER = 'REVIEW_REMINDER',
  REVIEW_COMPLETED = 'REVIEW_COMPLETED',
  REVIEW_OVERDUE = 'REVIEW_OVERDUE',

  // Integration events
  SYNC_COMPLETED = 'SYNC_COMPLETED',
  SYNC_FAILED = 'SYNC_FAILED',
  EXTERNAL_UPDATE = 'EXTERNAL_UPDATE',

  // Emergency events
  EMERGENCY_ACTIVATED = 'EMERGENCY_ACTIVATED',
  EMERGENCY_DECISION = 'EMERGENCY_DECISION',
}

// Event subscriber interface
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

// Example event handlers
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
    // Get patient
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

    // Get proxies for certain events
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
    // Only sync on certain events
    if (![
      ConsentEventType.CONSENT_CREATED,
      ConsentEventType.CONSENT_UPDATED,
      ConsentEventType.CONSENT_ACTIVATED,
      ConsentEventType.CONSENT_REVOKED,
    ].includes(event.type as ConsentEventType)) {
      return;
    }

    // Get consent
    const consent = await this.consentService.get(event.consentId!);
    if (!consent) return;

    // Sync to all configured systems
    await this.integrationOrchestrator.synchronizeConsent(consent, [
      'healthcare',
      'legal',
      'cryonics',
    ]);
  }
}
```

---

*Next Chapter: Security - Comprehensive security measures for consent data protection*
