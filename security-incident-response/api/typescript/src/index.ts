/**
 * WIA Security Incident Response - TypeScript SDK
 * Based on NIST 800-61 and SANS Incident Response Framework
 *
 * @module @wia/security-incident-response
 * @version 1.0.0
 */

import {
  SecurityIncident,
  IncidentType,
  IncidentSeverity,
  IncidentState,
  ResponseRole,
  ResponseTeamMember,
  ContainmentAction,
  ContainmentStrategy,
  EradicationAction,
  ForensicEvidence,
  EvidenceType,
  CommunicationRecord,
  CommunicationPriority,
  PostIncidentReview,
  IncidentResponseConfig,
  IncidentEvent,
  IncidentStatistics,
  IndicatorOfCompromise,
  ChainOfCustodyEntry,
  ImprovementRecommendation
} from './types';

export * from './types';

/**
 * Event Types for Incident Response
 */
export enum WIAIncidentEventType {
  INCIDENT_CREATED = 'incident.created',
  INCIDENT_UPDATED = 'incident.updated',
  INCIDENT_STATE_CHANGED = 'incident.state_changed',
  INCIDENT_ESCALATED = 'incident.escalated',
  CONTAINMENT_EXECUTED = 'containment.executed',
  ERADICATION_COMPLETED = 'eradication.completed',
  EVIDENCE_COLLECTED = 'evidence.collected',
  COMMUNICATION_SENT = 'communication.sent',
  REVIEW_COMPLETED = 'review.completed'
}

/**
 * Event Handler Type
 */
export type WIAIncidentEventHandler = (event: {
  type: WIAIncidentEventType;
  data: any;
  timestamp: Date;
}) => void | Promise<void>;

/**
 * Main WIA Incident Response Class
 *
 * Implements the complete NIST/SANS incident response lifecycle:
 * 1. Preparation
 * 2. Detection & Analysis
 * 3. Containment, Eradication & Recovery
 * 4. Post-Incident Activity
 */
export class WIAIncidentResponse {
  private config: IncidentResponseConfig;
  private incidents: Map<string, SecurityIncident>;
  private containmentActions: Map<string, ContainmentAction[]>;
  private eradicationActions: Map<string, EradicationAction[]>;
  private evidence: Map<string, ForensicEvidence[]>;
  private communications: Map<string, CommunicationRecord[]>;
  private reviews: Map<string, PostIncidentReview>;
  private eventHandlers: Map<WIAIncidentEventType, WIAIncidentEventHandler[]>;

  constructor(config: IncidentResponseConfig) {
    this.config = config;
    this.incidents = new Map();
    this.containmentActions = new Map();
    this.eradicationActions = new Map();
    this.evidence = new Map();
    this.communications = new Map();
    this.reviews = new Map();
    this.eventHandlers = new Map();
  }

  // ============================================================================
  // DETECTION & ANALYSIS (NIST Phase 2)
  // ============================================================================

  /**
   * Detect and create a new security incident
   */
  async detectIncident(params: {
    type: IncidentType;
    severity: IncidentSeverity;
    title: string;
    description: string;
    reportedBy: string;
    affectedSystems: string[];
    affectedData?: string[];
    indicators?: IndicatorOfCompromise[];
  }): Promise<SecurityIncident> {
    const incident: SecurityIncident = {
      id: this.generateIncidentId(),
      type: params.type,
      severity: params.severity,
      state: IncidentState.DETECTED,
      title: params.title,
      description: params.description,
      detectedAt: new Date(),
      reportedBy: params.reportedBy,
      affectedSystems: params.affectedSystems,
      affectedData: params.affectedData,
      indicators: params.indicators || [],
      timeline: [{
        timestamp: new Date(),
        actor: params.reportedBy,
        action: 'incident_detected',
        description: 'Incident detected and reported',
        state: IncidentState.DETECTED
      }]
    };

    this.incidents.set(incident.id, incident);
    await this.emitEvent(WIAIncidentEventType.INCIDENT_CREATED, incident);

    // Auto-assign based on severity
    await this.autoAssignResponders(incident);

    return incident;
  }

  /**
   * Classify and triage an incident
   */
  async triageIncident(incidentId: string, params: {
    severity?: IncidentSeverity;
    assignedTo?: ResponseRole[];
    additionalContext?: string;
  }): Promise<SecurityIncident> {
    const incident = this.getIncident(incidentId);

    if (params.severity) {
      incident.severity = params.severity;
    }

    if (params.assignedTo) {
      incident.assignedTo = params.assignedTo;
    }

    incident.state = IncidentState.TRIAGED;
    incident.timeline.push({
      timestamp: new Date(),
      actor: 'system',
      action: 'incident_triaged',
      description: params.additionalContext || 'Incident triaged and prioritized',
      state: IncidentState.TRIAGED
    });

    await this.emitEvent(WIAIncidentEventType.INCIDENT_STATE_CHANGED, incident);

    return incident;
  }

  /**
   * Perform deep analysis on an incident
   */
  async analyzeIncident(incidentId: string, params: {
    analyst: string;
    findings: string;
    additionalIOCs?: IndicatorOfCompromise[];
    impactAssessment?: string;
  }): Promise<SecurityIncident> {
    const incident = this.getIncident(incidentId);

    incident.state = IncidentState.ANALYZING;

    if (params.additionalIOCs) {
      incident.indicators = [...(incident.indicators || []), ...params.additionalIOCs];
    }

    incident.timeline.push({
      timestamp: new Date(),
      actor: params.analyst,
      action: 'analysis_completed',
      description: params.findings,
      metadata: { impactAssessment: params.impactAssessment }
    });

    await this.emitEvent(WIAIncidentEventType.INCIDENT_UPDATED, incident);

    return incident;
  }

  // ============================================================================
  // CONTAINMENT (NIST Phase 3)
  // ============================================================================

  /**
   * Execute containment strategy
   */
  async executeContainment(incidentId: string, params: {
    strategy: ContainmentStrategy;
    description: string;
    implementedBy: string;
    affectedSystems: string[];
  }): Promise<ContainmentAction> {
    const incident = this.getIncident(incidentId);

    const action: ContainmentAction = {
      id: this.generateId('containment'),
      incidentId,
      strategy: params.strategy,
      description: params.description,
      implementedBy: params.implementedBy,
      implementedAt: new Date(),
      affectedSystems: params.affectedSystems,
      success: true
    };

    const actions = this.containmentActions.get(incidentId) || [];
    actions.push(action);
    this.containmentActions.set(incidentId, actions);

    incident.state = IncidentState.CONTAINED;
    incident.timeline.push({
      timestamp: new Date(),
      actor: params.implementedBy,
      action: 'containment_executed',
      description: `Containment strategy: ${params.strategy}`,
      state: IncidentState.CONTAINED
    });

    await this.emitEvent(WIAIncidentEventType.CONTAINMENT_EXECUTED, { incident, action });

    return action;
  }

  // ============================================================================
  // ERADICATION (NIST Phase 3)
  // ============================================================================

  /**
   * Execute eradication actions
   */
  async executeEradication(incidentId: string, params: {
    action: 'remove_malware' | 'patch_system' | 'rebuild_system' | 'rotate_credentials' | 'other';
    description: string;
    implementedBy: string;
  }): Promise<EradicationAction> {
    const incident = this.getIncident(incidentId);

    const eradication: EradicationAction = {
      id: this.generateId('eradication'),
      incidentId,
      action: params.action,
      description: params.description,
      implementedBy: params.implementedBy,
      implementedAt: new Date(),
      verified: false
    };

    const actions = this.eradicationActions.get(incidentId) || [];
    actions.push(eradication);
    this.eradicationActions.set(incidentId, actions);

    incident.state = IncidentState.ERADICATING;
    incident.timeline.push({
      timestamp: new Date(),
      actor: params.implementedBy,
      action: 'eradication_executed',
      description: `Eradication action: ${params.action}`,
      state: IncidentState.ERADICATING
    });

    await this.emitEvent(WIAIncidentEventType.ERADICATION_COMPLETED, { incident, eradication });

    return eradication;
  }

  /**
   * Verify eradication was successful
   */
  async verifyEradication(incidentId: string, eradicationId: string, params: {
    verifiedBy: string;
    success: boolean;
  }): Promise<EradicationAction> {
    const actions = this.eradicationActions.get(incidentId) || [];
    const eradication = actions.find(a => a.id === eradicationId);

    if (!eradication) {
      throw new Error(`Eradication action ${eradicationId} not found`);
    }

    eradication.verified = params.success;
    eradication.verifiedBy = params.verifiedBy;
    eradication.verifiedAt = new Date();

    return eradication;
  }

  // ============================================================================
  // EVIDENCE PRESERVATION (FORENSICS)
  // ============================================================================

  /**
   * Collect and preserve forensic evidence
   */
  async collectEvidence(incidentId: string, params: {
    type: EvidenceType;
    description: string;
    collectedBy: string;
    location: string;
    hash?: string;
    size?: number;
    metadata?: Record<string, any>;
  }): Promise<ForensicEvidence> {
    const incident = this.getIncident(incidentId);

    const evidence: ForensicEvidence = {
      id: this.generateId('evidence'),
      incidentId,
      type: params.type,
      description: params.description,
      collectedBy: params.collectedBy,
      collectedAt: new Date(),
      chainOfCustody: [{
        timestamp: new Date(),
        custodian: params.collectedBy,
        action: 'collected',
        location: params.location
      }],
      location: params.location,
      hash: params.hash,
      size: params.size,
      metadata: params.metadata
    };

    const evidenceList = this.evidence.get(incidentId) || [];
    evidenceList.push(evidence);
    this.evidence.set(incidentId, evidenceList);

    incident.timeline.push({
      timestamp: new Date(),
      actor: params.collectedBy,
      action: 'evidence_collected',
      description: `Collected ${params.type} evidence`
    });

    await this.emitEvent(WIAIncidentEventType.EVIDENCE_COLLECTED, { incident, evidence });

    return evidence;
  }

  /**
   * Update chain of custody for evidence
   */
  async updateChainOfCustody(incidentId: string, evidenceId: string, entry: ChainOfCustodyEntry): Promise<ForensicEvidence> {
    const evidenceList = this.evidence.get(incidentId) || [];
    const evidence = evidenceList.find(e => e.id === evidenceId);

    if (!evidence) {
      throw new Error(`Evidence ${evidenceId} not found`);
    }

    evidence.chainOfCustody.push(entry);

    return evidence;
  }

  // ============================================================================
  // COMMUNICATION MANAGEMENT
  // ============================================================================

  /**
   * Send incident communication
   */
  async sendCommunication(incidentId: string, params: {
    priority: CommunicationPriority;
    recipients: string[];
    channel: 'email' | 'slack' | 'phone' | 'sms' | 'meeting';
    subject: string;
    message: string;
    sentBy: string;
    escalation?: boolean;
  }): Promise<CommunicationRecord> {
    const incident = this.getIncident(incidentId);

    const communication: CommunicationRecord = {
      id: this.generateId('comm'),
      incidentId,
      priority: params.priority,
      recipients: params.recipients,
      channel: params.channel,
      subject: params.subject,
      message: params.message,
      sentBy: params.sentBy,
      sentAt: new Date(),
      escalation: params.escalation
    };

    const comms = this.communications.get(incidentId) || [];
    comms.push(communication);
    this.communications.set(incidentId, comms);

    incident.timeline.push({
      timestamp: new Date(),
      actor: params.sentBy,
      action: 'communication_sent',
      description: `Sent ${params.channel} communication: ${params.subject}`
    });

    await this.emitEvent(WIAIncidentEventType.COMMUNICATION_SENT, { incident, communication });

    if (params.escalation) {
      await this.emitEvent(WIAIncidentEventType.INCIDENT_ESCALATED, incident);
    }

    return communication;
  }

  // ============================================================================
  // POST-INCIDENT ACTIVITY (NIST Phase 4)
  // ============================================================================

  /**
   * Conduct post-incident review
   */
  async conductPostIncidentReview(incidentId: string, params: {
    participants: string[];
    rootCause: string;
    lessonsLearned: string[];
    improvements: ImprovementRecommendation[];
  }): Promise<PostIncidentReview> {
    const incident = this.getIncident(incidentId);

    const detectionTime = this.calculateDetectionTime(incident);
    const responseTime = this.calculateResponseTime(incident);
    const containmentTime = this.calculateContainmentTime(incident);
    const recoveryTime = this.calculateRecoveryTime(incident);

    const review: PostIncidentReview = {
      incidentId,
      conductedAt: new Date(),
      participants: params.participants,
      timeline: incident.timeline,
      rootCause: params.rootCause,
      lessonsLearned: params.lessonsLearned,
      improvements: params.improvements,
      successMetrics: {
        detectionTime,
        responseTime,
        containmentTime,
        recoveryTime
      }
    };

    this.reviews.set(incidentId, review);

    incident.state = IncidentState.POST_INCIDENT;
    incident.timeline.push({
      timestamp: new Date(),
      actor: 'system',
      action: 'post_incident_review',
      description: 'Post-incident review completed',
      state: IncidentState.POST_INCIDENT
    });

    await this.emitEvent(WIAIncidentEventType.REVIEW_COMPLETED, { incident, review });

    return review;
  }

  /**
   * Close an incident
   */
  async closeIncident(incidentId: string, closedBy: string): Promise<SecurityIncident> {
    const incident = this.getIncident(incidentId);

    incident.state = IncidentState.CLOSED;
    incident.timeline.push({
      timestamp: new Date(),
      actor: closedBy,
      action: 'incident_closed',
      description: 'Incident closed',
      state: IncidentState.CLOSED
    });

    await this.emitEvent(WIAIncidentEventType.INCIDENT_STATE_CHANGED, incident);

    return incident;
  }

  // ============================================================================
  // QUERY & REPORTING
  // ============================================================================

  /**
   * Get incident by ID
   */
  getIncident(incidentId: string): SecurityIncident {
    const incident = this.incidents.get(incidentId);
    if (!incident) {
      throw new Error(`Incident ${incidentId} not found`);
    }
    return incident;
  }

  /**
   * List all incidents
   */
  listIncidents(filter?: {
    state?: IncidentState;
    severity?: IncidentSeverity;
    type?: IncidentType;
  }): SecurityIncident[] {
    let incidents = Array.from(this.incidents.values());

    if (filter) {
      incidents = incidents.filter(inc => {
        if (filter.state && inc.state !== filter.state) return false;
        if (filter.severity && inc.severity !== filter.severity) return false;
        if (filter.type && inc.type !== filter.type) return false;
        return true;
      });
    }

    return incidents;
  }

  /**
   * Get incident statistics
   */
  getStatistics(period: { start: Date; end: Date }): IncidentStatistics {
    const incidents = Array.from(this.incidents.values()).filter(
      inc => inc.detectedAt >= period.start && inc.detectedAt <= period.end
    );

    const stats: IncidentStatistics = {
      period,
      totalIncidents: incidents.length,
      bySeverity: {} as Record<IncidentSeverity, number>,
      byType: {} as Record<IncidentType, number>,
      byState: {} as Record<IncidentState, number>,
      averageResponseTime: 0,
      averageContainmentTime: 0,
      averageRecoveryTime: 0
    };

    incidents.forEach(inc => {
      stats.bySeverity[inc.severity] = (stats.bySeverity[inc.severity] || 0) + 1;
      stats.byType[inc.type] = (stats.byType[inc.type] || 0) + 1;
      stats.byState[inc.state] = (stats.byState[inc.state] || 0) + 1;
    });

    // Calculate averages
    const responseTimes = incidents.map(inc => this.calculateResponseTime(inc)).filter(t => t > 0);
    const containmentTimes = incidents.map(inc => this.calculateContainmentTime(inc)).filter(t => t > 0);
    const recoveryTimes = incidents.map(inc => this.calculateRecoveryTime(inc)).filter(t => t > 0);

    stats.averageResponseTime = responseTimes.length > 0
      ? responseTimes.reduce((a, b) => a + b, 0) / responseTimes.length
      : 0;
    stats.averageContainmentTime = containmentTimes.length > 0
      ? containmentTimes.reduce((a, b) => a + b, 0) / containmentTimes.length
      : 0;
    stats.averageRecoveryTime = recoveryTimes.length > 0
      ? recoveryTimes.reduce((a, b) => a + b, 0) / recoveryTimes.length
      : 0;

    return stats;
  }

  // ============================================================================
  // EVENT HANDLING
  // ============================================================================

  /**
   * Register event handler
   */
  on(eventType: WIAIncidentEventType, handler: WIAIncidentEventHandler): void {
    const handlers = this.eventHandlers.get(eventType) || [];
    handlers.push(handler);
    this.eventHandlers.set(eventType, handlers);
  }

  /**
   * Remove event handler
   */
  off(eventType: WIAIncidentEventType, handler: WIAIncidentEventHandler): void {
    const handlers = this.eventHandlers.get(eventType) || [];
    const index = handlers.indexOf(handler);
    if (index > -1) {
      handlers.splice(index, 1);
    }
  }

  // ============================================================================
  // PRIVATE HELPER METHODS
  // ============================================================================

  private async emitEvent(type: WIAIncidentEventType, data: any): Promise<void> {
    const handlers = this.eventHandlers.get(type) || [];
    const event = { type, data, timestamp: new Date() };

    for (const handler of handlers) {
      await handler(event);
    }
  }

  private generateIncidentId(): string {
    return `INC-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateId(prefix: string): string {
    return `${prefix}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private async autoAssignResponders(incident: SecurityIncident): Promise<void> {
    const roles: ResponseRole[] = [ResponseRole.INCIDENT_MANAGER, ResponseRole.SECURITY_ANALYST];

    if (incident.severity === IncidentSeverity.CRITICAL || incident.severity === IncidentSeverity.HIGH) {
      roles.push(ResponseRole.FORENSIC_INVESTIGATOR);
      roles.push(ResponseRole.EXECUTIVE_SPONSOR);
    }

    incident.assignedTo = roles;
  }

  private calculateDetectionTime(incident: SecurityIncident): number {
    // Time from incident occurrence to detection (simplified)
    return 0; // Would need actual occurrence time
  }

  private calculateResponseTime(incident: SecurityIncident): number {
    const triagedEvent = incident.timeline.find(e => e.state === IncidentState.TRIAGED);
    if (!triagedEvent) return 0;

    return Math.floor((triagedEvent.timestamp.getTime() - incident.detectedAt.getTime()) / 60000);
  }

  private calculateContainmentTime(incident: SecurityIncident): number {
    const containedEvent = incident.timeline.find(e => e.state === IncidentState.CONTAINED);
    if (!containedEvent) return 0;

    return Math.floor((containedEvent.timestamp.getTime() - incident.detectedAt.getTime()) / 60000);
  }

  private calculateRecoveryTime(incident: SecurityIncident): number {
    const recoveredEvent = incident.timeline.find(e => e.state === IncidentState.RECOVERING);
    if (!recoveredEvent) return 0;

    return Math.floor((recoveredEvent.timestamp.getTime() - incident.detectedAt.getTime()) / 60000);
  }
}

/**
 * Factory function to create WIA Incident Response instance
 */
export function createIncidentResponse(config: IncidentResponseConfig): WIAIncidentResponse {
  return new WIAIncidentResponse(config);
}

/**
 * Default export
 */
export default WIAIncidentResponse;
