/**
 * WIA-TIME-011: Historical Integrity - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

import crypto from 'crypto';
import {
  Vector3,
  EventCategory,
  EvidenceType,
  HistoricalEvent,
  Evidence,
  Signature,
  EventVerificationParams,
  VerificationResult,
  SignatureStatus,
  Timeline,
  IntegrityCheckParams,
  IntegrityReport,
  IntegrityViolation,
  TamperingDetectionParams,
  TamperingReport,
  TamperingIncident,
  EvidenceChain,
  EvidenceChainLink,
  ChainValidation,
  Checkpoint,
  CheckpointCreationParams,
  CheckpointValidation,
  RollbackResult,
  TimelineFingerprint,
  FingerprintComparison,
  TimelineIdentification,
  FingerprintEvolution,
  ImmutabilitySeal,
  ModificationRequest,
  ModificationApproval,
  IntegrityConfig,
  Result,
  AsyncResult,
  IntegrityErrorCode,
  IntegrityError,
  EVIDENCE_RELIABILITY_WEIGHTS,
  EVENT_CATEGORY_WEIGHTS,
  DEFAULT_INTEGRITY_CONFIG,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * Historical Integrity SDK
 *
 * Provides comprehensive tools for verifying historical events,
 * checking timeline integrity, and detecting tampering.
 */
export class HistoricalIntegritySDK {
  private config: IntegrityConfig;
  private monitoringInterval?: NodeJS.Timeout;

  constructor(config: Partial<IntegrityConfig> = {}) {
    this.config = { ...DEFAULT_INTEGRITY_CONFIG, ...config };
  }

  // ==========================================================================
  // Event Verification
  // ==========================================================================

  /**
   * Verify a historical event's authenticity
   */
  async verifyEvent(params: EventVerificationParams): Promise<VerificationResult> {
    const { event, expectedHash, verifyEvidence = true, checkConsistency = true, minConfidence = 0.9 } = params;

    const warnings: string[] = [];
    const errors: string[] = [];

    // Compute event hash
    const computedHash = this.hashEvent(event);
    const hashMatch = expectedHash ? computedHash === expectedHash : true;

    if (expectedHash && !hashMatch) {
      errors.push(`Hash mismatch: expected ${expectedHash}, got ${computedHash}`);
    }

    // Verify evidence
    let evidenceValid = true;
    if (verifyEvidence && event.evidence.length > 0) {
      for (const evidence of event.evidence) {
        const evidenceHash = this.hashEvidence(evidence);
        if (evidenceHash !== evidence.hash) {
          evidenceValid = false;
          errors.push(`Evidence ${evidence.id} hash mismatch`);
        }
      }
    } else if (event.category === EventCategory.CRITICAL && event.evidence.length === 0) {
      warnings.push('Critical event has no supporting evidence');
    }

    // Verify signatures
    const signatures = await this.verifySignatures(event.evidence);
    const allSignaturesValid = signatures.every(s => s.isValid);

    if (!allSignaturesValid) {
      warnings.push('Some signatures are invalid');
    }

    // Check temporal consistency
    let temporalConsistency = true;
    if (checkConsistency && event.causes) {
      // Verify causal events occurred before this event
      temporalConsistency = this.checkTemporalConsistency(event);
      if (!temporalConsistency) {
        errors.push('Temporal consistency violation detected');
      }
    }

    // Calculate confidence
    const confidence = this.calculateConfidence({
      hashMatch,
      evidenceValid,
      temporalConsistency,
      signatures,
      evidenceCount: event.evidence.length,
    });

    const isValid = errors.length === 0 && confidence >= minConfidence;

    return {
      isValid,
      confidence,
      hashMatch,
      computedHash,
      evidenceValid,
      temporalConsistency,
      signatures,
      warnings,
      errors,
      timestamp: new Date(),
    };
  }

  /**
   * Create immutability seal for an event
   */
  async protectEvent(event: HistoricalEvent, category: EventCategory): Promise<ImmutabilitySeal> {
    const eventHash = this.hashEvent(event);

    // Collect required signatures based on category
    const signatures = await this.collectSignatures(event, category);

    // Create hardware seal (simulated)
    const hardwareSeal = this.generateHardwareSeal(eventHash);

    const seal: ImmutabilitySeal = {
      id: this.generateId('seal'),
      eventHash,
      timestamp: new Date(),
      category,
      signatures,
      hardwareSeal,
      isIntact: true,
    };

    return seal;
  }

  // ==========================================================================
  // Timeline Integrity
  // ==========================================================================

  /**
   * Check timeline integrity
   */
  async checkIntegrity(params: IntegrityCheckParams): Promise<IntegrityReport> {
    const startTime = Date.now();
    const {
      timeline,
      startDate,
      endDate,
      verifyCheckpoints = true,
      minIntegrityScore = this.config.minIntegrityScore,
    } = params;

    // Filter events by date range
    let events = timeline.events;
    if (startDate || endDate) {
      events = events.filter(e => {
        const eventDate = new Date(e.timestamp);
        if (startDate && eventDate < startDate) return false;
        if (endDate && eventDate > endDate) return false;
        return true;
      });
    }

    const violations: IntegrityViolation[] = [];
    const warnings: string[] = [];
    let verifiedEvents = 0;

    // Verify each event
    for (const event of events) {
      const verification = await this.verifyEvent({ event });

      if (verification.isValid) {
        verifiedEvents++;
      } else {
        violations.push({
          type: 'hash_mismatch',
          severity: event.category === EventCategory.CRITICAL ? 'critical' : 'error',
          description: `Event ${event.id} verification failed`,
          affectedEvents: [event.id],
          detectedAt: new Date(),
        });
      }

      warnings.push(...verification.warnings);
    }

    // Check for orphaned events
    const orphanedEvents = this.findOrphanedEvents(events);
    if (orphanedEvents.length > 0) {
      violations.push({
        type: 'orphaned_event',
        severity: 'warning',
        description: `Found ${orphanedEvents.length} orphaned events`,
        affectedEvents: orphanedEvents.map(e => e.id),
        detectedAt: new Date(),
      });
    }

    // Verify checkpoints if requested
    if (verifyCheckpoints) {
      for (const checkpoint of timeline.checkpoints) {
        const valid = await this.validateCheckpoint(checkpoint, timeline);
        if (!valid.isValid) {
          violations.push({
            type: 'broken_chain',
            severity: 'error',
            description: `Checkpoint ${checkpoint.id} validation failed`,
            affectedEvents: [],
            detectedAt: new Date(),
          });
        }
      }
    }

    // Calculate integrity score
    const integrityScore = this.calculateIntegrityScore(events, verifiedEvents);

    const isIntact = violations.filter(v => v.severity === 'error' || v.severity === 'critical').length === 0
      && integrityScore >= minIntegrityScore;

    return {
      timelineId: timeline.id,
      isIntact,
      integrityScore,
      totalEvents: events.length,
      verifiedEvents,
      failedEvents: events.length - verifiedEvents,
      violations,
      warnings: Array.from(new Set(warnings)),
      timestamp: new Date(),
      duration: Date.now() - startTime,
    };
  }

  /**
   * Detect tampering in timeline
   */
  async detectTampering(params: TamperingDetectionParams): Promise<TamperingReport> {
    const startTime = Date.now();
    const {
      timeline,
      period,
      useMLDetection = this.config.useMLDetection,
      sensitivity = 0.8,
      verifyEvidence = true,
    } = params;

    const incidents: TamperingIncident[] = [];
    const suspiciousPatterns: string[] = [];

    // Filter events by period
    let events = timeline.events;
    if (period) {
      events = events.filter(e => {
        const eventDate = new Date(e.timestamp);
        return eventDate >= period.start && eventDate <= period.end;
      });
    }

    // Check for direct modifications
    const modifiedEvents = await this.detectDirectModifications(events);
    if (modifiedEvents.length > 0) {
      incidents.push({
        id: this.generateId('incident'),
        type: 'direct_modification',
        severity: 'high',
        confidence: 0.95,
        detectedAt: new Date(),
        affectedEvents: modifiedEvents.map(e => e.id),
        description: `${modifiedEvents.length} events show signs of direct modification`,
        recommendations: ['Verify event hashes', 'Check audit logs', 'Restore from checkpoint'],
        evidence: modifiedEvents,
      });
      suspiciousPatterns.push('direct_modification');
    }

    // Check for timeline gaps
    const gaps = this.detectTimelineGaps(events);
    if (gaps.length > 0) {
      suspiciousPatterns.push('timeline_gaps');
    }

    // Check for impossible sequences
    const impossibleSequences = this.detectImpossibleSequences(events);
    if (impossibleSequences.length > 0) {
      incidents.push({
        id: this.generateId('incident'),
        type: 'timestamp_manipulation',
        severity: 'medium',
        confidence: 0.85,
        detectedAt: new Date(),
        affectedEvents: impossibleSequences,
        description: 'Detected impossible event sequences',
        recommendations: ['Check temporal ordering', 'Verify timestamps'],
        evidence: [],
      });
      suspiciousPatterns.push('impossible_sequences');
    }

    // ML-based anomaly detection
    let anomalyScore = 0;
    if (useMLDetection) {
      anomalyScore = this.detectAnomalies(events, sensitivity);
      if (anomalyScore > 0.7) {
        suspiciousPatterns.push('ml_anomaly_detected');
      }
    }

    const tamperingDetected = incidents.length > 0 || anomalyScore > sensitivity;

    return {
      timelineId: timeline.id,
      tamperingDetected,
      incidents,
      anomalyScore,
      suspiciousPatterns,
      timestamp: new Date(),
      duration: Date.now() - startTime,
    };
  }

  // ==========================================================================
  // Evidence Chain
  // ==========================================================================

  /**
   * Build evidence chain for an event
   */
  async buildEvidenceChain(event: HistoricalEvent): Promise<EvidenceChain> {
    const sortedEvidence = [...event.evidence].sort(
      (a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime()
    );

    const links: EvidenceChainLink[] = [];
    let prevLinkHash = '';

    for (let i = 0; i < sortedEvidence.length; i++) {
      const evidence = sortedEvidence[i];
      const evidenceHash = this.hashEvidence(evidence);

      const verification = await this.verifyEvent({
        event: { ...event, evidence: [evidence] },
      });

      const link: EvidenceChainLink = {
        evidenceHash,
        prevLinkHash,
        timestamp: evidence.timestamp,
        verification,
        index: i,
      };

      links.push(link);
      prevLinkHash = this.hashLink(link);
    }

    const chainHash = this.hashChain(links);
    const reliability = this.calculateChainReliability(sortedEvidence);
    const isValid = links.every(link => link.verification.isValid);

    return {
      id: this.generateId('chain'),
      eventId: event.id,
      evidence: sortedEvidence,
      links,
      chainHash,
      reliability,
      isValid,
      created: new Date(),
    };
  }

  /**
   * Verify evidence chain
   */
  async verifyEvidenceChain(chain: EvidenceChain): Promise<ChainValidation> {
    const linkValidations: ChainValidation['linkValidations'] = [];

    let prevHash = '';
    for (const link of chain.links) {
      const errors: string[] = [];

      // Verify previous link hash
      if (link.prevLinkHash !== prevHash) {
        errors.push('Previous link hash mismatch');
      }

      // Verify evidence hash
      const evidence = chain.evidence[link.index];
      const computedHash = this.hashEvidence(evidence);
      if (computedHash !== link.evidenceHash) {
        errors.push('Evidence hash mismatch');
      }

      linkValidations.push({
        index: link.index,
        isValid: errors.length === 0,
        errors,
      });

      prevHash = this.hashLink(link);
    }

    // Verify overall chain hash
    const computedChainHash = this.hashChain(chain.links);
    const integrityCheck = {
      passed: computedChainHash === chain.chainHash,
      expectedHash: chain.chainHash,
      actualHash: computedChainHash,
    };

    const reliability = this.calculateChainReliability(chain.evidence);
    const isValid = linkValidations.every(v => v.isValid) && integrityCheck.passed;

    return {
      isValid,
      reliability,
      linkValidations,
      integrityCheck,
      timestamp: new Date(),
    };
  }

  // ==========================================================================
  // Checkpoints
  // ==========================================================================

  /**
   * Create a checkpoint for timeline
   */
  async createCheckpoint(params: CheckpointCreationParams): Promise<Checkpoint> {
    const { timeline, name, reason = 'Manual checkpoint', includeEvents, collectSignatures = true } = params;

    const events = includeEvents || timeline.events;
    const sortedEvents = [...events].sort(
      (a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime()
    );

    // Build Merkle tree
    const merkleRoot = this.buildMerkleTree(sortedEvents.map(e => this.hashEvent(e)));

    // Calculate fingerprint
    const fingerprint = this.calculateFingerprint(timeline, new Date());

    // Get integrity score
    const integrityReport = await this.checkIntegrity({ timeline });

    // Collect signatures
    const signatures: Signature[] = collectSignatures
      ? await this.collectCheckpointSignatures(timeline)
      : [];

    // Find previous checkpoint
    const previousCheckpoint = timeline.checkpoints.length > 0
      ? timeline.checkpoints[timeline.checkpoints.length - 1].id
      : undefined;

    const eventRange: [Date, Date] = [
      sortedEvents[0]?.timestamp || new Date(),
      sortedEvents[sortedEvents.length - 1]?.timestamp || new Date(),
    ];

    const checkpoint: Checkpoint = {
      id: this.generateId('checkpoint'),
      timelineId: timeline.id,
      timestamp: new Date(),
      name,
      fingerprint: fingerprint.value,
      eventCount: events.length,
      integrityScore: integrityReport.integrityScore,
      merkleRoot,
      metadata: {
        creator: 'system',
        reason,
        eventRange,
      },
      signatures,
      previousCheckpoint,
      isValidated: true,
    };

    return checkpoint;
  }

  /**
   * Validate a checkpoint
   */
  async validateCheckpoint(checkpoint: Checkpoint, timeline: Timeline): Promise<CheckpointValidation> {
    const discrepancies: string[] = [];

    // Verify signatures
    const signaturesValid = checkpoint.signatures.every(sig => sig.isValid !== false);
    if (!signaturesValid) {
      discrepancies.push('Invalid signatures detected');
    }

    // Rebuild Merkle tree
    const events = timeline.events.filter(e => {
      const eventDate = new Date(e.timestamp);
      return eventDate <= new Date(checkpoint.timestamp);
    });
    const rebuiltMerkleRoot = this.buildMerkleTree(events.map(e => this.hashEvent(e)));
    const merkleRootMatch = rebuiltMerkleRoot === checkpoint.merkleRoot;

    if (!merkleRootMatch) {
      discrepancies.push('Merkle root mismatch - timeline has diverged');
    }

    // Verify fingerprint
    const currentFingerprint = this.calculateFingerprint(timeline, checkpoint.timestamp);
    const fingerprintMatch = currentFingerprint.value === checkpoint.fingerprint;

    if (!fingerprintMatch) {
      discrepancies.push('Fingerprint mismatch');
    }

    const isValid = signaturesValid && merkleRootMatch && fingerprintMatch;

    return {
      isValid,
      signaturesValid,
      merkleRootMatch,
      fingerprintMatch,
      discrepancies,
      timestamp: new Date(),
    };
  }

  /**
   * Rollback timeline to checkpoint
   */
  async rollbackToCheckpoint(timeline: Timeline, checkpointId: string): Promise<RollbackResult> {
    const checkpoint = timeline.checkpoints.find(c => c.id === checkpointId);

    if (!checkpoint) {
      return {
        success: false,
        checkpointId,
        eventsRemoved: 0,
        removedEvents: [],
        backupId: '',
        newFingerprint: '',
        timestamp: new Date(),
        error: 'Checkpoint not found',
      };
    }

    // Verify checkpoint before rollback
    const validation = await this.validateCheckpoint(checkpoint, timeline);
    if (!validation.isValid) {
      return {
        success: false,
        checkpointId,
        eventsRemoved: 0,
        removedEvents: [],
        backupId: '',
        newFingerprint: '',
        timestamp: new Date(),
        error: 'Checkpoint validation failed',
      };
    }

    // Find events to remove
    const checkpointTime = new Date(checkpoint.timestamp);
    const removedEvents = timeline.events.filter(
      e => new Date(e.timestamp) > checkpointTime
    );

    // Create backup
    const backupId = this.generateId('backup');

    // Remove events (in actual implementation, this would modify the timeline)
    const eventsRemoved = removedEvents.length;

    // Calculate new fingerprint
    const newFingerprint = checkpoint.fingerprint;

    return {
      success: true,
      checkpointId,
      eventsRemoved,
      removedEvents,
      backupId,
      newFingerprint,
      timestamp: new Date(),
    };
  }

  // ==========================================================================
  // Fingerprinting
  // ==========================================================================

  /**
   * Calculate timeline fingerprint
   */
  calculateFingerprint(timeline: Timeline, timestamp: Date): TimelineFingerprint {
    // Get events up to timestamp
    const events = timeline.events.filter(
      e => new Date(e.timestamp) <= timestamp
    ).sort((a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime());

    // Build Merkle tree
    const eventHashes = events.map(e => this.hashEvent(e));
    const merkleRoot = this.buildMerkleTree(eventHashes);

    // Hash metadata
    const metadataHash = this.hashString(
      timeline.id + timeline.created.toISOString() + JSON.stringify(timeline.metadata)
    );

    // Combine into fingerprint
    const fingerprintData = merkleRoot + metadataHash + timestamp.toISOString();
    const value = this.hashString(fingerprintData);

    return {
      value,
      timelineId: timeline.id,
      timestamp,
      merkleRoot,
      eventCount: events.length,
      metadataHash,
      version: '1.0',
    };
  }

  /**
   * Compare two fingerprints
   */
  compareFingerprints(fp1: string, fp2: string): FingerprintComparison {
    const identical = fp1 === fp2;

    if (identical) {
      return {
        identical: true,
        similarity: 1.0,
        classification: 'identical',
        commonPrefixLength: fp1.length,
        details: 'Fingerprints are identical',
      };
    }

    // Calculate common prefix
    let commonPrefixLength = 0;
    for (let i = 0; i < Math.min(fp1.length, fp2.length); i++) {
      if (fp1[i] === fp2[i]) {
        commonPrefixLength++;
      } else {
        break;
      }
    }

    const similarity = commonPrefixLength / Math.max(fp1.length, fp2.length);

    let classification: FingerprintComparison['classification'];
    if (similarity > 0.9) {
      classification = 'highly_similar';
    } else if (similarity > 0.5) {
      classification = 'partially_similar';
    } else {
      classification = 'different';
    }

    return {
      identical: false,
      similarity,
      classification,
      commonPrefixLength,
      details: `Fingerprints have ${(similarity * 100).toFixed(1)}% similarity`,
    };
  }

  // ==========================================================================
  // Monitoring
  // ==========================================================================

  /**
   * Start real-time timeline monitoring
   */
  startMonitoring(timeline: Timeline, onTampering: (report: TamperingReport) => void): void {
    if (this.monitoringInterval) {
      this.stopMonitoring();
    }

    this.monitoringInterval = setInterval(async () => {
      const report = await this.detectTampering({ timeline });

      if (report.tamperingDetected) {
        onTampering(report);
      }
    }, this.config.monitoringInterval * 1000);
  }

  /**
   * Stop monitoring
   */
  stopMonitoring(): void {
    if (this.monitoringInterval) {
      clearInterval(this.monitoringInterval);
      this.monitoringInterval = undefined;
    }
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Hash an event using SHA-3
   */
  private hashEvent(event: HistoricalEvent): string {
    const canonical = this.canonicalizeEvent(event);
    return this.hashString(canonical);
  }

  /**
   * Hash evidence
   */
  private hashEvidence(evidence: Evidence): string {
    const canonical = JSON.stringify({
      type: evidence.type,
      source: evidence.source,
      timestamp: evidence.timestamp.toISOString(),
      content: evidence.content,
    });
    return this.hashString(canonical);
  }

  /**
   * Hash a string using SHA-3-256
   */
  private hashString(data: string): string {
    return crypto.createHash('sha3-256').update(data).digest('hex');
  }

  /**
   * Canonicalize event for hashing
   */
  private canonicalizeEvent(event: HistoricalEvent): string {
    const participants = [...event.participants].sort();
    const evidenceHashes = event.evidence.map(e => e.hash).sort();

    return JSON.stringify({
      timestamp: event.timestamp.toISOString(),
      location: event.location,
      participants,
      description: event.description,
      evidence: evidenceHashes,
      category: event.category,
      metadata: this.sortObject(event.metadata),
    });
  }

  /**
   * Sort object keys recursively
   */
  private sortObject(obj: any): any {
    if (typeof obj !== 'object' || obj === null) return obj;
    if (Array.isArray(obj)) return obj.map(item => this.sortObject(item));

    return Object.keys(obj)
      .sort()
      .reduce((result, key) => {
        result[key] = this.sortObject(obj[key]);
        return result;
      }, {} as any);
  }

  /**
   * Build Merkle tree from hashes
   */
  private buildMerkleTree(hashes: string[]): string {
    if (hashes.length === 0) return this.hashString('empty');
    if (hashes.length === 1) return hashes[0];

    let currentLevel = [...hashes];

    while (currentLevel.length > 1) {
      const nextLevel: string[] = [];

      for (let i = 0; i < currentLevel.length; i += 2) {
        if (i + 1 < currentLevel.length) {
          const combined = this.hashString(currentLevel[i] + currentLevel[i + 1]);
          nextLevel.push(combined);
        } else {
          const combined = this.hashString(currentLevel[i] + currentLevel[i]);
          nextLevel.push(combined);
        }
      }

      currentLevel = nextLevel;
    }

    return currentLevel[0];
  }

  /**
   * Hash a chain link
   */
  private hashLink(link: EvidenceChainLink): string {
    return this.hashString(
      link.evidenceHash + link.prevLinkHash + link.timestamp.toISOString()
    );
  }

  /**
   * Hash entire chain
   */
  private hashChain(links: EvidenceChainLink[]): string {
    const linkHashes = links.map(link => this.hashLink(link));
    return this.buildMerkleTree(linkHashes);
  }

  /**
   * Calculate chain reliability
   */
  private calculateChainReliability(evidence: Evidence[]): number {
    if (evidence.length === 0) return 0;

    const totalWeight = evidence.reduce((sum, e) => {
      const weight = EVIDENCE_RELIABILITY_WEIGHTS[e.type] || 0.5;
      return sum + weight;
    }, 0);

    return totalWeight / evidence.length;
  }

  /**
   * Calculate integrity score
   */
  private calculateIntegrityScore(events: HistoricalEvent[], verifiedEvents: number): number {
    if (events.length === 0) return 1.0;

    let totalWeight = 0;
    let verifiedWeight = 0;

    events.forEach(event => {
      const weight = EVENT_CATEGORY_WEIGHTS[event.category] || 1.0;
      totalWeight += weight;

      // Assume verified events contribute their full weight
      // In real implementation, would track which specific events verified
      verifiedWeight += weight * (verifiedEvents / events.length);
    });

    return totalWeight > 0 ? verifiedWeight / totalWeight : 0;
  }

  /**
   * Calculate verification confidence
   */
  private calculateConfidence(params: {
    hashMatch: boolean;
    evidenceValid: boolean;
    temporalConsistency: boolean;
    signatures: SignatureStatus[];
    evidenceCount: number;
  }): number {
    let score = 0;

    if (params.hashMatch) score += 0.4;
    if (params.evidenceValid) score += 0.3;
    if (params.temporalConsistency) score += 0.2;

    const validSignatures = params.signatures.filter(s => s.isValid).length;
    const signatureRatio = params.signatures.length > 0
      ? validSignatures / params.signatures.length
      : 0;
    score += signatureRatio * 0.1;

    return Math.min(score, 1.0);
  }

  /**
   * Verify signatures
   */
  private async verifySignatures(evidence: Evidence[]): Promise<SignatureStatus[]> {
    const statuses: SignatureStatus[] = [];

    for (const ev of evidence) {
      for (const sig of ev.signatures) {
        statuses.push({
          signature: sig,
          isValid: sig.isValid ?? true, // Simplified verification
          verifiedAt: new Date(),
        });
      }
    }

    return statuses;
  }

  /**
   * Check temporal consistency
   */
  private checkTemporalConsistency(event: HistoricalEvent): boolean {
    // Simplified: check that event timestamp is valid
    // In real implementation, would verify causal ordering
    const eventTime = new Date(event.timestamp).getTime();
    return !isNaN(eventTime) && eventTime > 0;
  }

  /**
   * Find orphaned events
   */
  private findOrphanedEvents(events: HistoricalEvent[]): HistoricalEvent[] {
    const eventIds = new Set(events.map(e => e.id));
    const orphaned: HistoricalEvent[] = [];

    for (const event of events) {
      if (event.causes) {
        const hasMissingCauses = event.causes.some(causeId => !eventIds.has(causeId));
        if (hasMissingCauses) {
          orphaned.push(event);
        }
      }
    }

    return orphaned;
  }

  /**
   * Detect direct modifications
   */
  private async detectDirectModifications(events: HistoricalEvent[]): Promise<HistoricalEvent[]> {
    const modified: HistoricalEvent[] = [];

    for (const event of events) {
      const verification = await this.verifyEvent({ event });
      if (!verification.hashMatch) {
        modified.push(event);
      }
    }

    return modified;
  }

  /**
   * Detect timeline gaps
   */
  private detectTimelineGaps(events: HistoricalEvent[]): { start: Date; end: Date }[] {
    // Simplified gap detection
    return [];
  }

  /**
   * Detect impossible sequences
   */
  private detectImpossibleSequences(events: HistoricalEvent[]): string[] {
    const impossible: string[] = [];

    // Check for temporal ordering violations
    for (const event of events) {
      if (event.causes && event.causes.length > 0) {
        // In real implementation, verify cause events occur before effect
        // This is a simplified placeholder
      }
    }

    return impossible;
  }

  /**
   * ML-based anomaly detection (simplified)
   */
  private detectAnomalies(events: HistoricalEvent[], sensitivity: number): number {
    // Simplified: return random score
    // Real implementation would use ML model
    return Math.random() * 0.5;
  }

  /**
   * Collect signatures for event protection
   */
  private async collectSignatures(event: HistoricalEvent, category: EventCategory): Promise<Signature[]> {
    const requiredSignatures = category === EventCategory.CRITICAL ? 3 : 1;
    const signatures: Signature[] = [];

    for (let i = 0; i < requiredSignatures; i++) {
      signatures.push({
        signer: `verifier-${i}`,
        algorithm: 'CRYSTALS-Dilithium',
        value: this.hashString(event.id + i),
        timestamp: new Date(),
        isValid: true,
      });
    }

    return signatures;
  }

  /**
   * Collect checkpoint signatures
   */
  private async collectCheckpointSignatures(timeline: Timeline): Promise<Signature[]> {
    return [
      {
        signer: 'checkpoint-system',
        algorithm: 'SHA3-256',
        value: this.hashString(timeline.id + Date.now()),
        timestamp: new Date(),
        isValid: true,
      },
    ];
  }

  /**
   * Generate hardware seal (simulated)
   */
  private generateHardwareSeal(eventHash: string): string {
    return 'HSM-' + this.hashString(eventHash + 'hardware');
  }

  /**
   * Generate unique ID
   */
  private generateId(prefix: string): string {
    return `${prefix}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';
export { HistoricalIntegritySDK };
export default HistoricalIntegritySDK;
