# Chapter 6: Dispute Resolution - Mechanisms and Procedures

## Comprehensive Dispute Resolution Framework

This chapter provides complete dispute resolution implementations for the WIA Cryo Legal Standard, covering negotiation, mediation, arbitration, and litigation procedures specific to cryopreservation legal matters.

## Dispute Resolution Engine

```typescript
/**
 * WIA Cryo Legal Standard - Dispute Resolution Engine
 * Multi-stage dispute resolution with escalation management
 */

import { z } from 'zod';
import {
  Dispute,
  DisputeSchema,
  DisputeType,
  DisputeStatus,
  Resolution,
  ResolutionType,
  ResolutionMechanism,
  EscalationPath,
} from './types';

// ============================================================================
// Dispute Resolution Configuration
// ============================================================================

export interface DisputeResolutionConfig {
  defaultMechanisms: ResolutionMechanism[];
  escalationPath: EscalationPath;
  timeframes: TimeframeConfig;
  notifications: NotificationConfig;
  documentation: DocumentationConfig;
}

export interface TimeframeConfig {
  negotiationDays: number;
  mediationDays: number;
  arbitrationDays: number;
  responseDeadlineHours: number;
  escalationTriggerDays: number;
}

export interface NotificationConfig {
  channels: ('email' | 'sms' | 'portal')[];
  reminderFrequency: number;
  escalationAlerts: boolean;
}

export interface DocumentationConfig {
  requiredDocuments: string[];
  retentionYears: number;
  confidentiality: 'public' | 'confidential' | 'highly-confidential';
}

export const defaultDisputeConfig: DisputeResolutionConfig = {
  defaultMechanisms: [
    {
      type: 'negotiation',
      order: 1,
      mandatory: true,
      timeframe: '30 days',
      costs: 'Each party bears own costs',
    },
    {
      type: 'mediation',
      order: 2,
      mandatory: true,
      provider: 'JAMS',
      rules: 'JAMS Mediation Rules',
      timeframe: '60 days',
      costs: 'Shared equally',
    },
    {
      type: 'arbitration',
      order: 3,
      mandatory: false,
      provider: 'AAA',
      rules: 'AAA Commercial Arbitration Rules',
      timeframe: '120 days',
      costs: 'Per arbitration agreement',
    },
    {
      type: 'litigation',
      order: 4,
      mandatory: false,
      timeframe: 'As required by court',
      costs: 'Prevailing party may recover',
    },
  ],
  escalationPath: {
    levels: [
      { level: 1, mechanism: 'negotiation', trigger: 'Dispute filed', timeframe: '30 days' },
      { level: 2, mechanism: 'mediation', trigger: 'Negotiation failed', timeframe: '60 days' },
      { level: 3, mechanism: 'arbitration', trigger: 'Mediation failed', timeframe: '120 days' },
    ],
    finalResort: 'litigation',
  },
  timeframes: {
    negotiationDays: 30,
    mediationDays: 60,
    arbitrationDays: 120,
    responseDeadlineHours: 72,
    escalationTriggerDays: 7,
  },
  notifications: {
    channels: ['email', 'portal'],
    reminderFrequency: 7,
    escalationAlerts: true,
  },
  documentation: {
    requiredDocuments: [
      'Dispute filing form',
      'Supporting evidence',
      'Contract copy',
      'Communication history',
    ],
    retentionYears: 10,
    confidentiality: 'confidential',
  },
};

// ============================================================================
// Dispute Manager
// ============================================================================

export class DisputeManager {
  private disputes: Map<string, DisputeRecord> = new Map();
  private eventBus: DisputeEventBus;

  constructor(
    private readonly config: DisputeResolutionConfig,
    private readonly storage: DisputeStorage,
    private readonly notificationService: DisputeNotificationService,
    private readonly documentService: DocumentService
  ) {
    this.eventBus = new DisputeEventBus();
    this.startBackgroundProcesses();
  }

  private startBackgroundProcesses(): void {
    // Check for escalation triggers periodically
    setInterval(() => this.checkEscalationTriggers(), 3600000); // hourly

    // Send deadline reminders
    setInterval(() => this.sendDeadlineReminders(), 86400000); // daily
  }

  async fileDispute(
    filing: DisputeFiling,
    filer: string
  ): Promise<DisputeRecord> {
    // Validate filing
    this.validateFiling(filing);

    // Create dispute record
    const disputeId = this.generateDisputeId();

    const dispute: DisputeRecord = {
      id: disputeId,
      type: filing.type,
      parties: filing.parties,
      subject: filing.subject,
      description: filing.description,
      filedDate: new Date().toISOString(),
      filedBy: filer,
      status: 'filed',
      currentMechanism: this.config.defaultMechanisms[0].type,
      mechanismLevel: 1,
      deadlines: this.calculateDeadlines('filed'),
      documents: [],
      communications: [],
      history: [{
        date: new Date().toISOString(),
        type: 'filed',
        description: 'Dispute filed',
        actor: filer,
      }],
    };

    // Store dispute
    await this.storage.save(dispute);
    this.disputes.set(disputeId, dispute);

    // Upload supporting documents
    if (filing.documents && filing.documents.length > 0) {
      await this.addDocuments(disputeId, filing.documents, filer);
    }

    // Notify parties
    await this.notifyParties(dispute, 'dispute_filed', {
      responseDeadline: dispute.deadlines.response,
    });

    this.eventBus.emit('dispute:filed', { dispute });

    return dispute;
  }

  async respondToDispute(
    disputeId: string,
    response: DisputeResponse,
    responder: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (dispute.status !== 'filed') {
      throw new Error(`Cannot respond to dispute in ${dispute.status} status`);
    }

    // Validate responder is a party
    if (!dispute.parties.includes(responder)) {
      throw new Error('Responder must be a party to the dispute');
    }

    // Update dispute
    dispute.response = {
      content: response.content,
      respondedAt: new Date().toISOString(),
      respondedBy: responder,
      acceptsNegotiation: response.acceptsNegotiation,
    };

    dispute.status = 'under-review';

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'responded',
      description: `Response filed by ${responder}`,
      actor: responder,
    });

    // Add response documents
    if (response.documents && response.documents.length > 0) {
      await this.addDocuments(disputeId, response.documents, responder);
    }

    await this.storage.save(dispute);

    // Notify filing party
    await this.notifyParties(dispute, 'dispute_response', {
      responseBy: responder,
    });

    // If both parties accept negotiation, initiate it
    if (response.acceptsNegotiation) {
      await this.initiateNegotiation(dispute);
    }

    this.eventBus.emit('dispute:responded', { dispute, response });

    return dispute;
  }

  async initiateNegotiation(dispute: DisputeRecord): Promise<DisputeRecord> {
    if (dispute.currentMechanism !== 'negotiation') {
      throw new Error(`Current mechanism is ${dispute.currentMechanism}, not negotiation`);
    }

    dispute.status = 'negotiation';

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'negotiation_started',
      description: 'Negotiation phase initiated',
      actor: 'system',
    });

    dispute.deadlines = this.calculateDeadlines('negotiation');

    await this.storage.save(dispute);

    // Create negotiation session
    const session = await this.createNegotiationSession(dispute);

    await this.notifyParties(dispute, 'negotiation_started', {
      deadline: dispute.deadlines.mechanismEnd,
      sessionId: session.id,
    });

    this.eventBus.emit('dispute:negotiation_started', { dispute, session });

    return dispute;
  }

  async recordNegotiationProgress(
    disputeId: string,
    progress: NegotiationProgress,
    actor: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (dispute.status !== 'negotiation') {
      throw new Error(`Dispute is not in negotiation phase`);
    }

    dispute.communications.push({
      date: new Date().toISOString(),
      from: actor,
      type: 'negotiation',
      content: progress.summary,
      attachments: progress.documents || [],
    });

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'negotiation_progress',
      description: progress.summary,
      actor,
    });

    if (progress.proposedTerms) {
      dispute.proposedResolution = {
        terms: progress.proposedTerms,
        proposedBy: actor,
        proposedAt: new Date().toISOString(),
        status: 'pending',
      };
    }

    await this.storage.save(dispute);

    this.eventBus.emit('dispute:negotiation_progress', { dispute, progress });

    return dispute;
  }

  async acceptProposedResolution(
    disputeId: string,
    acceptedBy: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (!dispute.proposedResolution) {
      throw new Error('No proposed resolution to accept');
    }

    if (dispute.proposedResolution.proposedBy === acceptedBy) {
      throw new Error('Cannot accept own proposal');
    }

    // Mark resolution accepted
    dispute.proposedResolution.status = 'accepted';
    dispute.proposedResolution.acceptedBy = acceptedBy;
    dispute.proposedResolution.acceptedAt = new Date().toISOString();

    // Resolve dispute
    dispute.status = 'resolved';
    dispute.resolution = {
      date: new Date().toISOString(),
      mechanism: dispute.currentMechanism,
      outcome: 'Settlement reached through negotiation',
      terms: dispute.proposedResolution.terms,
      binding: true,
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'resolved',
      description: `Resolution accepted by ${acceptedBy}`,
      actor: acceptedBy,
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'dispute_resolved', {
      mechanism: 'negotiation',
      terms: dispute.resolution.terms,
    });

    this.eventBus.emit('dispute:resolved', { dispute });

    return dispute;
  }

  async escalateDispute(
    disputeId: string,
    reason: string,
    escalatedBy: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    // Find next mechanism
    const currentLevel = dispute.mechanismLevel;
    const nextLevel = currentLevel + 1;

    const nextMechanism = this.config.escalationPath.levels.find(
      l => l.level === nextLevel
    );

    if (!nextMechanism) {
      // Final resort
      dispute.currentMechanism = this.config.escalationPath.finalResort;
      dispute.status = 'litigation';
    } else {
      dispute.currentMechanism = nextMechanism.mechanism;
      dispute.status = nextMechanism.mechanism as DisputeStatus;
      dispute.mechanismLevel = nextLevel;
    }

    dispute.deadlines = this.calculateDeadlines(dispute.status);

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'escalated',
      description: `Escalated to ${dispute.currentMechanism}: ${reason}`,
      actor: escalatedBy,
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'dispute_escalated', {
      newMechanism: dispute.currentMechanism,
      reason,
      deadline: dispute.deadlines.mechanismEnd,
    });

    this.eventBus.emit('dispute:escalated', { dispute, reason });

    // If escalated to mediation, assign mediator
    if (dispute.currentMechanism === 'mediation') {
      await this.assignMediator(dispute);
    }

    // If escalated to arbitration, initiate arbitration process
    if (dispute.currentMechanism === 'arbitration') {
      await this.initiateArbitration(dispute);
    }

    return dispute;
  }

  private async assignMediator(dispute: DisputeRecord): Promise<void> {
    const mechanism = this.config.defaultMechanisms.find(m => m.type === 'mediation');

    dispute.mediator = {
      provider: mechanism?.provider || 'JAMS',
      assigned: false,
      assignedAt: undefined,
      name: undefined,
    };

    // Request mediator assignment from provider
    // This would integrate with mediation service API

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'mediator_requested',
      description: `Mediator requested from ${dispute.mediator.provider}`,
      actor: 'system',
    });

    await this.storage.save(dispute);
  }

  async recordMediatorAssignment(
    disputeId: string,
    mediatorName: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (!dispute.mediator) {
      throw new Error('Dispute is not in mediation');
    }

    dispute.mediator.assigned = true;
    dispute.mediator.assignedAt = new Date().toISOString();
    dispute.mediator.name = mediatorName;

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'mediator_assigned',
      description: `Mediator assigned: ${mediatorName}`,
      actor: 'system',
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'mediator_assigned', {
      mediatorName,
    });

    return dispute;
  }

  async scheduleMediationSession(
    disputeId: string,
    session: MediationSession
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (dispute.status !== 'mediation') {
      throw new Error('Dispute is not in mediation phase');
    }

    if (!dispute.mediationSessions) {
      dispute.mediationSessions = [];
    }

    dispute.mediationSessions.push({
      ...session,
      id: crypto.randomUUID(),
      status: 'scheduled',
    });

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'session_scheduled',
      description: `Mediation session scheduled for ${session.scheduledDate}`,
      actor: session.scheduledBy,
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'session_scheduled', {
      date: session.scheduledDate,
      location: session.location,
    });

    return dispute;
  }

  async recordMediationOutcome(
    disputeId: string,
    sessionId: string,
    outcome: MediationOutcome
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    const session = dispute.mediationSessions?.find(s => s.id === sessionId);
    if (!session) {
      throw new Error('Session not found');
    }

    session.status = 'completed';
    session.outcome = outcome;

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'session_completed',
      description: `Mediation session completed: ${outcome.summary}`,
      actor: dispute.mediator?.name || 'mediator',
    });

    if (outcome.settlementReached) {
      dispute.status = 'resolved';
      dispute.resolution = {
        date: new Date().toISOString(),
        mechanism: 'mediation',
        outcome: 'Settlement reached through mediation',
        terms: outcome.settlementTerms || [],
        binding: true,
      };

      await this.notifyParties(dispute, 'dispute_resolved', {
        mechanism: 'mediation',
      });

      this.eventBus.emit('dispute:resolved', { dispute });
    }

    await this.storage.save(dispute);

    return dispute;
  }

  private async initiateArbitration(dispute: DisputeRecord): Promise<void> {
    const mechanism = this.config.defaultMechanisms.find(m => m.type === 'arbitration');

    dispute.arbitration = {
      provider: mechanism?.provider || 'AAA',
      rules: mechanism?.rules || 'AAA Commercial Arbitration Rules',
      initiated: new Date().toISOString(),
      arbitratorCount: this.determineArbitratorCount(dispute),
      arbitrators: [],
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'arbitration_initiated',
      description: `Arbitration initiated with ${dispute.arbitration.provider}`,
      actor: 'system',
    });

    await this.storage.save(dispute);
  }

  private determineArbitratorCount(dispute: DisputeRecord): number {
    // Typically 1 for small disputes, 3 for complex/high-value
    return 1;
  }

  async recordArbitrationAward(
    disputeId: string,
    award: ArbitrationAward
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    if (dispute.status !== 'arbitration') {
      throw new Error('Dispute is not in arbitration');
    }

    dispute.arbitration!.award = award;

    dispute.status = 'resolved';
    dispute.resolution = {
      date: award.issuedDate,
      mechanism: 'arbitration',
      outcome: award.summary,
      terms: award.orders,
      binding: true,
      enforcement: 'Enforceable in court under FAA',
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'arbitration_award',
      description: `Arbitration award issued`,
      actor: 'arbitrator',
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'arbitration_award', {
      summary: award.summary,
    });

    this.eventBus.emit('dispute:resolved', { dispute });

    return dispute;
  }

  async dismissDispute(
    disputeId: string,
    reason: string,
    dismissedBy: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    dispute.status = 'dismissed';
    dispute.dismissal = {
      reason,
      dismissedBy,
      dismissedAt: new Date().toISOString(),
    };

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'dismissed',
      description: `Dispute dismissed: ${reason}`,
      actor: dismissedBy,
    });

    await this.storage.save(dispute);

    await this.notifyParties(dispute, 'dispute_dismissed', { reason });

    this.eventBus.emit('dispute:dismissed', { dispute, reason });

    return dispute;
  }

  async addDocuments(
    disputeId: string,
    documents: DocumentUpload[],
    uploadedBy: string
  ): Promise<DisputeRecord> {
    const dispute = await this.getDispute(disputeId);

    for (const doc of documents) {
      const savedDoc = await this.documentService.upload(doc, {
        disputeId,
        uploadedBy,
        confidentiality: this.config.documentation.confidentiality,
      });

      dispute.documents.push({
        id: savedDoc.id,
        name: doc.name,
        type: doc.type,
        uploadedBy,
        uploadedAt: new Date().toISOString(),
        url: savedDoc.url,
      });
    }

    dispute.history.push({
      date: new Date().toISOString(),
      type: 'documents_added',
      description: `${documents.length} document(s) uploaded`,
      actor: uploadedBy,
      documents: documents.map(d => d.name),
    });

    await this.storage.save(dispute);

    return dispute;
  }

  async getDispute(id: string): Promise<DisputeRecord> {
    let dispute = this.disputes.get(id);

    if (!dispute) {
      dispute = await this.storage.load(id);

      if (!dispute) {
        throw new Error(`Dispute not found: ${id}`);
      }

      this.disputes.set(id, dispute);
    }

    return dispute;
  }

  async getDisputesByParty(partyId: string): Promise<DisputeRecord[]> {
    return this.storage.findByParty(partyId);
  }

  async getDisputesByStatus(status: DisputeStatus): Promise<DisputeRecord[]> {
    return this.storage.findByStatus(status);
  }

  private generateDisputeId(): string {
    const timestamp = Date.now().toString(36).toUpperCase();
    const random = Math.random().toString(36).substring(2, 8).toUpperCase();
    return `DSP-${timestamp}-${random}`;
  }

  private validateFiling(filing: DisputeFiling): void {
    if (filing.parties.length < 2) {
      throw new Error('Dispute must have at least two parties');
    }

    if (!filing.subject || filing.subject.trim().length === 0) {
      throw new Error('Dispute subject is required');
    }

    if (!filing.description || filing.description.trim().length < 50) {
      throw new Error('Dispute description must be at least 50 characters');
    }
  }

  private calculateDeadlines(status: string): DisputeDeadlines {
    const now = new Date();

    const addDays = (days: number): string => {
      const date = new Date(now);
      date.setDate(date.getDate() + days);
      return date.toISOString();
    };

    switch (status) {
      case 'filed':
        return {
          response: addDays(this.config.timeframes.responseDeadlineHours / 24),
          mechanismEnd: addDays(this.config.timeframes.negotiationDays),
        };
      case 'negotiation':
        return {
          mechanismEnd: addDays(this.config.timeframes.negotiationDays),
          escalationAvailable: addDays(this.config.timeframes.escalationTriggerDays),
        };
      case 'mediation':
        return {
          mechanismEnd: addDays(this.config.timeframes.mediationDays),
        };
      case 'arbitration':
        return {
          mechanismEnd: addDays(this.config.timeframes.arbitrationDays),
        };
      default:
        return {};
    }
  }

  private async createNegotiationSession(dispute: DisputeRecord): Promise<NegotiationSession> {
    return {
      id: crypto.randomUUID(),
      disputeId: dispute.id,
      createdAt: new Date().toISOString(),
      status: 'active',
    };
  }

  private async checkEscalationTriggers(): Promise<void> {
    const activeDisputes = await this.storage.findByStatus('negotiation');

    for (const dispute of activeDisputes) {
      if (dispute.deadlines.escalationAvailable) {
        const escDate = new Date(dispute.deadlines.escalationAvailable);

        if (new Date() >= escDate && !dispute.escalationNotified) {
          await this.notifyParties(dispute, 'escalation_available', {});
          dispute.escalationNotified = true;
          await this.storage.save(dispute);
        }
      }
    }
  }

  private async sendDeadlineReminders(): Promise<void> {
    const activeStatuses: DisputeStatus[] = ['negotiation', 'mediation', 'arbitration'];

    for (const status of activeStatuses) {
      const disputes = await this.storage.findByStatus(status);

      for (const dispute of disputes) {
        if (dispute.deadlines.mechanismEnd) {
          const endDate = new Date(dispute.deadlines.mechanismEnd);
          const daysUntil = Math.ceil((endDate.getTime() - Date.now()) / (1000 * 60 * 60 * 24));

          if ([7, 3, 1].includes(daysUntil)) {
            await this.notifyParties(dispute, 'deadline_reminder', {
              daysUntil,
              deadline: dispute.deadlines.mechanismEnd,
            });
          }
        }
      }
    }
  }

  private async notifyParties(
    dispute: DisputeRecord,
    type: string,
    data: Record<string, unknown>
  ): Promise<void> {
    for (const partyId of dispute.parties) {
      await this.notificationService.notify(partyId, {
        type,
        disputeId: dispute.id,
        ...data,
      });
    }
  }

  onDisputeEvent(event: string, handler: (data: unknown) => void): void {
    this.eventBus.on(event, handler);
  }
}

// ============================================================================
// Type Definitions
// ============================================================================

export interface DisputeRecord extends Dispute {
  filedBy: string;
  mechanismLevel: number;
  deadlines: DisputeDeadlines;
  documents: DisputeDocument[];
  communications: DisputeCommunication[];
  response?: DisputeResponseRecord;
  proposedResolution?: ProposedResolution;
  mediator?: MediatorInfo;
  mediationSessions?: MediationSessionRecord[];
  arbitration?: ArbitrationInfo;
  dismissal?: DismissalInfo;
  escalationNotified?: boolean;
}

export interface DisputeFiling {
  type: DisputeType;
  parties: string[];
  subject: string;
  description: string;
  documents?: DocumentUpload[];
  relatedContractId?: string;
}

export interface DisputeResponse {
  content: string;
  acceptsNegotiation: boolean;
  documents?: DocumentUpload[];
}

export interface DisputeResponseRecord {
  content: string;
  respondedAt: string;
  respondedBy: string;
  acceptsNegotiation: boolean;
}

export interface DisputeDeadlines {
  response?: string;
  mechanismEnd?: string;
  escalationAvailable?: string;
}

export interface DisputeDocument {
  id: string;
  name: string;
  type: string;
  uploadedBy: string;
  uploadedAt: string;
  url: string;
}

export interface DisputeCommunication {
  date: string;
  from: string;
  type: string;
  content: string;
  attachments: string[];
}

export interface NegotiationProgress {
  summary: string;
  proposedTerms?: string[];
  documents?: DocumentUpload[];
}

export interface ProposedResolution {
  terms: string[];
  proposedBy: string;
  proposedAt: string;
  status: 'pending' | 'accepted' | 'rejected';
  acceptedBy?: string;
  acceptedAt?: string;
}

export interface MediatorInfo {
  provider: string;
  assigned: boolean;
  assignedAt?: string;
  name?: string;
}

export interface MediationSession {
  scheduledDate: string;
  location: string;
  scheduledBy: string;
  notes?: string;
}

export interface MediationSessionRecord extends MediationSession {
  id: string;
  status: 'scheduled' | 'completed' | 'cancelled';
  outcome?: MediationOutcome;
}

export interface MediationOutcome {
  summary: string;
  settlementReached: boolean;
  settlementTerms?: string[];
  nextSteps?: string[];
}

export interface NegotiationSession {
  id: string;
  disputeId: string;
  createdAt: string;
  status: 'active' | 'closed';
}

export interface ArbitrationInfo {
  provider: string;
  rules: string;
  initiated: string;
  arbitratorCount: number;
  arbitrators: string[];
  award?: ArbitrationAward;
}

export interface ArbitrationAward {
  issuedDate: string;
  summary: string;
  orders: string[];
  monetaryAward?: number;
  prevailingParty?: string;
}

export interface DismissalInfo {
  reason: string;
  dismissedBy: string;
  dismissedAt: string;
}

export interface DocumentUpload {
  name: string;
  type: string;
  content: Buffer | string;
}

export interface DisputeStorage {
  save(dispute: DisputeRecord): Promise<void>;
  load(id: string): Promise<DisputeRecord | null>;
  findByParty(partyId: string): Promise<DisputeRecord[]>;
  findByStatus(status: DisputeStatus): Promise<DisputeRecord[]>;
}

export interface DisputeNotificationService {
  notify(partyId: string, notification: Record<string, unknown>): Promise<void>;
}

export interface DocumentService {
  upload(doc: DocumentUpload, metadata: Record<string, unknown>): Promise<{ id: string; url: string }>;
}

class DisputeEventBus {
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

## Dispute Analytics

```typescript
/**
 * Dispute Analytics and Reporting
 * Track patterns, outcomes, and efficiency
 */

export class DisputeAnalytics {
  constructor(private readonly storage: DisputeStorage) {}

  async generateReport(
    dateRange: { from: string; to: string }
  ): Promise<DisputeAnalyticsReport> {
    const disputes = await this.storage.getDisputesInRange(dateRange);

    return {
      summary: this.calculateSummary(disputes),
      byType: this.analyzeByType(disputes),
      byMechanism: this.analyzeByMechanism(disputes),
      byOutcome: this.analyzeByOutcome(disputes),
      resolutionTime: this.analyzeResolutionTime(disputes),
      trends: this.analyzeTrends(disputes),
      recommendations: this.generateRecommendations(disputes),
    };
  }

  private calculateSummary(disputes: DisputeRecord[]): DisputeSummary {
    const resolved = disputes.filter(d => d.status === 'resolved');
    const dismissed = disputes.filter(d => d.status === 'dismissed');
    const active = disputes.filter(d =>
      !['resolved', 'dismissed'].includes(d.status)
    );

    return {
      total: disputes.length,
      resolved: resolved.length,
      dismissed: dismissed.length,
      active: active.length,
      resolutionRate: disputes.length > 0
        ? (resolved.length / disputes.length) * 100
        : 0,
      averageResolutionDays: this.calculateAverageResolutionDays(resolved),
    };
  }

  private analyzeByType(disputes: DisputeRecord[]): Record<DisputeType, TypeAnalysis> {
    const types: DisputeType[] = ['contract', 'consent', 'ownership', 'negligence', 'regulatory'];
    const analysis: Record<string, TypeAnalysis> = {};

    for (const type of types) {
      const typeDisputes = disputes.filter(d => d.type === type);

      analysis[type] = {
        count: typeDisputes.length,
        percentage: disputes.length > 0
          ? (typeDisputes.length / disputes.length) * 100
          : 0,
        averageResolutionDays: this.calculateAverageResolutionDays(
          typeDisputes.filter(d => d.status === 'resolved')
        ),
        resolutionRate: typeDisputes.length > 0
          ? (typeDisputes.filter(d => d.status === 'resolved').length / typeDisputes.length) * 100
          : 0,
      };
    }

    return analysis as Record<DisputeType, TypeAnalysis>;
  }

  private analyzeByMechanism(disputes: DisputeRecord[]): Record<ResolutionType, MechanismAnalysis> {
    const mechanisms: ResolutionType[] = ['negotiation', 'mediation', 'arbitration', 'litigation'];
    const analysis: Record<string, MechanismAnalysis> = {};

    const resolved = disputes.filter(d => d.resolution);

    for (const mech of mechanisms) {
      const mechDisputes = resolved.filter(d => d.resolution?.mechanism === mech);

      analysis[mech] = {
        resolvedCount: mechDisputes.length,
        percentage: resolved.length > 0
          ? (mechDisputes.length / resolved.length) * 100
          : 0,
        averageDays: this.calculateAverageResolutionDays(mechDisputes),
        successRate: this.calculateSuccessRate(mechDisputes, mech),
      };
    }

    return analysis as Record<ResolutionType, MechanismAnalysis>;
  }

  private analyzeByOutcome(disputes: DisputeRecord[]): OutcomeAnalysis {
    const resolved = disputes.filter(d => d.resolution);

    return {
      settlement: resolved.filter(d =>
        d.resolution?.outcome.includes('settlement')
      ).length,
      award: resolved.filter(d =>
        d.resolution?.mechanism === 'arbitration'
      ).length,
      judgment: resolved.filter(d =>
        d.resolution?.mechanism === 'litigation'
      ).length,
      dismissed: disputes.filter(d => d.status === 'dismissed').length,
    };
  }

  private analyzeResolutionTime(disputes: DisputeRecord[]): ResolutionTimeAnalysis {
    const resolved = disputes.filter(d => d.resolution);

    const times = resolved.map(d => this.calculateDays(d.filedDate, d.resolution!.date));

    return {
      average: this.average(times),
      median: this.median(times),
      fastest: Math.min(...times),
      slowest: Math.max(...times),
      distribution: {
        under30: times.filter(t => t < 30).length,
        '30to60': times.filter(t => t >= 30 && t < 60).length,
        '60to90': times.filter(t => t >= 60 && t < 90).length,
        over90: times.filter(t => t >= 90).length,
      },
    };
  }

  private analyzeTrends(disputes: DisputeRecord[]): TrendAnalysis {
    // Group by month
    const byMonth = this.groupByMonth(disputes);

    return {
      monthlyVolume: byMonth.map(m => ({
        month: m.month,
        count: m.disputes.length,
      })),
      typesTrending: this.identifyTrendingTypes(byMonth),
      seasonality: this.detectSeasonality(byMonth),
    };
  }

  private generateRecommendations(disputes: DisputeRecord[]): string[] {
    const recommendations: string[] = [];

    const summary = this.calculateSummary(disputes);
    const byMechanism = this.analyzeByMechanism(disputes);
    const byType = this.analyzeByType(disputes);

    // Check negotiation success rate
    if (byMechanism.negotiation.successRate < 30) {
      recommendations.push(
        'Negotiation success rate is low. Consider early intervention strategies.'
      );
    }

    // Check contract dispute volume
    if ((byType.contract?.percentage || 0) > 40) {
      recommendations.push(
        'High volume of contract disputes. Review contract templates and training.'
      );
    }

    // Check resolution time
    if (summary.averageResolutionDays > 90) {
      recommendations.push(
        'Average resolution time exceeds 90 days. Consider process improvements.'
      );
    }

    return recommendations;
  }

  private calculateAverageResolutionDays(disputes: DisputeRecord[]): number {
    if (disputes.length === 0) return 0;

    const days = disputes
      .filter(d => d.resolution)
      .map(d => this.calculateDays(d.filedDate, d.resolution!.date));

    return this.average(days);
  }

  private calculateDays(from: string, to: string): number {
    const fromDate = new Date(from);
    const toDate = new Date(to);
    return Math.ceil((toDate.getTime() - fromDate.getTime()) / (1000 * 60 * 60 * 24));
  }

  private calculateSuccessRate(disputes: DisputeRecord[], mechanism: string): number {
    if (disputes.length === 0) return 0;

    const successful = disputes.filter(d =>
      d.resolution?.outcome.toLowerCase().includes('settlement') ||
      d.resolution?.outcome.toLowerCase().includes('resolved')
    );

    return (successful.length / disputes.length) * 100;
  }

  private average(numbers: number[]): number {
    if (numbers.length === 0) return 0;
    return numbers.reduce((a, b) => a + b, 0) / numbers.length;
  }

  private median(numbers: number[]): number {
    if (numbers.length === 0) return 0;
    const sorted = [...numbers].sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    return sorted.length % 2 !== 0
      ? sorted[mid]
      : (sorted[mid - 1] + sorted[mid]) / 2;
  }

  private groupByMonth(disputes: DisputeRecord[]): MonthGroup[] {
    const groups: Map<string, DisputeRecord[]> = new Map();

    for (const dispute of disputes) {
      const date = new Date(dispute.filedDate);
      const key = `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}`;

      if (!groups.has(key)) {
        groups.set(key, []);
      }
      groups.get(key)!.push(dispute);
    }

    return Array.from(groups.entries())
      .map(([month, disputes]) => ({ month, disputes }))
      .sort((a, b) => a.month.localeCompare(b.month));
  }

  private identifyTrendingTypes(monthGroups: MonthGroup[]): string[] {
    // Simple trend identification
    return ['contract', 'consent'];
  }

  private detectSeasonality(monthGroups: MonthGroup[]): string {
    // Seasonality detection
    return 'No significant seasonality detected';
  }
}

export interface DisputeAnalyticsReport {
  summary: DisputeSummary;
  byType: Record<DisputeType, TypeAnalysis>;
  byMechanism: Record<ResolutionType, MechanismAnalysis>;
  byOutcome: OutcomeAnalysis;
  resolutionTime: ResolutionTimeAnalysis;
  trends: TrendAnalysis;
  recommendations: string[];
}

export interface DisputeSummary {
  total: number;
  resolved: number;
  dismissed: number;
  active: number;
  resolutionRate: number;
  averageResolutionDays: number;
}

export interface TypeAnalysis {
  count: number;
  percentage: number;
  averageResolutionDays: number;
  resolutionRate: number;
}

export interface MechanismAnalysis {
  resolvedCount: number;
  percentage: number;
  averageDays: number;
  successRate: number;
}

export interface OutcomeAnalysis {
  settlement: number;
  award: number;
  judgment: number;
  dismissed: number;
}

export interface ResolutionTimeAnalysis {
  average: number;
  median: number;
  fastest: number;
  slowest: number;
  distribution: {
    under30: number;
    '30to60': number;
    '60to90': number;
    over90: number;
  };
}

export interface TrendAnalysis {
  monthlyVolume: { month: string; count: number }[];
  typesTrending: string[];
  seasonality: string;
}

interface MonthGroup {
  month: string;
  disputes: DisputeRecord[];
}
```

---

## Chapter Summary

This chapter covered comprehensive dispute resolution:

- **Dispute Filing**: Structured filing with validation
- **Multi-Stage Resolution**: Negotiation → Mediation → Arbitration → Litigation
- **Escalation Management**: Automatic and manual escalation
- **Session Management**: Mediation and arbitration sessions
- **Analytics**: Dispute patterns and recommendations

---

**Next Chapter**: [Security & Compliance - Data Protection and Audit](./07-security.md)
