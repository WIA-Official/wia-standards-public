/**
 * WIA-TIME-033: Historical Archive SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for historical archive management,
 * including event recording, timeline versioning, verification, and
 * cross-timeline reconciliation.
 */

import {
  ArchiveConfig,
  ArchiveQuery,
  ArchiveEntry,
  HistoricalRecord,
  Timeline,
  TimelineID,
  TimelineVersion,
  Event,
  Evidence,
  Verification,
  Metadata,
  IntegrityData,
  QueryResult,
  ReconciliationParams,
  ReconciliationResult,
  AlterationLog,
  VerificationRequest,
  VerificationResult,
  IntegrityCheck,
  ExportRequest,
  ExportResult,
  ResearchSession,
  AuthRequest,
  AuthResponse,
  ARCHIVE_CONSTANTS,
  ArchiveErrorCode,
  ArchiveError,
  ReconciliationStrategy,
  TimelineDifference,
  DivergenceStatistics,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-033 Historical Archive SDK
 */
export class HistoricalArchiveSDK {
  private version = '1.0.0';
  private config: ArchiveConfig;
  private records: Map<string, Map<string, HistoricalRecord>> = new Map();
  private timelines: Map<TimelineID, Timeline> = new Map();
  private sessions: Map<string, ResearchSession> = new Map();

  constructor(config: ArchiveConfig) {
    this.config = config;
    this.initializeArchive();
  }

  /**
   * Initialize archive
   */
  private initializeArchive(): void {
    // Create default timeline if specified
    if (this.config.timeline) {
      this.createTimeline(this.config.timeline, 'Default Timeline');
    }
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // Timeline Management
  // ==========================================================================

  /**
   * Create a new timeline
   */
  createTimeline(timelineId: TimelineID, name: string, parent?: TimelineID): Timeline {
    if (this.timelines.has(timelineId)) {
      throw new ArchiveError(
        ArchiveErrorCode.TIMELINE_NOT_FOUND,
        `Timeline ${timelineId} already exists`
      );
    }

    const timeline: Timeline = {
      id: timelineId,
      name,
      parent,
      created: new Date(),
      status: 'active',
      recordCount: 0,
    };

    this.timelines.set(timelineId, timeline);
    this.records.set(timelineId, new Map());

    return timeline;
  }

  /**
   * Get timeline by ID
   */
  getTimeline(timelineId: TimelineID): Timeline | undefined {
    return this.timelines.get(timelineId);
  }

  /**
   * List all timelines
   */
  listTimelines(): Timeline[] {
    return Array.from(this.timelines.values());
  }

  /**
   * Get timeline version info
   */
  getTimelineVersion(timelineId: TimelineID): TimelineVersion {
    const timeline = this.timelines.get(timelineId);
    if (!timeline) {
      throw new ArchiveError(
        ArchiveErrorCode.TIMELINE_NOT_FOUND,
        `Timeline ${timelineId} not found`
      );
    }

    const records = this.records.get(timelineId);
    const lastRecord = records ? Array.from(records.values()).pop() : undefined;

    return {
      timeline: timelineId,
      version: this.version,
      lastModified: new Date(),
      recordCount: timeline.recordCount,
      hash: lastRecord?.integrity.hash || '',
    };
  }

  // ==========================================================================
  // Event Recording
  // ==========================================================================

  /**
   * Record a historical event
   */
  async recordEvent(params: {
    event: Event;
    timeline: TimelineID;
    evidence?: Evidence;
    verification?: Partial<Verification>;
    metadata?: Metadata;
  }): Promise<HistoricalRecord> {
    const { event, timeline, evidence, verification, metadata } = params;

    // Verify timeline exists
    const timelineObj = this.timelines.get(timeline);
    if (!timelineObj) {
      throw new ArchiveError(
        ArchiveErrorCode.TIMELINE_NOT_FOUND,
        `Timeline ${timeline} not found`
      );
    }

    // Get timeline records
    const timelineRecords = this.records.get(timeline)!;
    const recordCount = timelineRecords.size;

    // Get previous hash
    const previousHash = recordCount > 0
      ? Array.from(timelineRecords.values())[recordCount - 1].integrity.hash
      : '0'.repeat(128);

    // Create record ID
    const recordId = this.generateRecordId(timeline, event.date.start as Date);

    // Create verification
    const recordVerification: Verification = {
      method: verification?.method || 'multi-witness',
      verifiers: verification?.verifiers || [],
      confidence: verification?.confidence || 0.95,
      timestamp: new Date(),
      signature: verification?.signature,
      notes: verification?.notes,
    };

    // Create evidence
    const recordEvidence: Evidence = evidence || {
      primary: [],
      secondary: [],
      sources: [],
    };

    // Calculate integrity hash
    const recordData = {
      timeline,
      event,
      evidence: recordEvidence,
      metadata: metadata || {},
    };

    const hash = this.calculateHash(recordData, previousHash);

    // Create integrity data
    const integrity: IntegrityData = {
      hash,
      previousHash,
      blockHeight: recordCount,
      sealed: true,
    };

    // Create complete record
    const record: HistoricalRecord = {
      id: recordId,
      timeline,
      version: this.version,
      created: new Date(),
      event,
      evidence: recordEvidence,
      verification: recordVerification,
      metadata: metadata || {},
      integrity,
      status: recordVerification.confidence >= ARCHIVE_CONSTANTS.VERIFICATION_THRESHOLD
        ? 'verified'
        : 'unverified',
    };

    // Store record
    timelineRecords.set(recordId, record);
    timelineObj.recordCount++;

    return record;
  }

  /**
   * Get record by ID
   */
  getRecord(timeline: TimelineID, recordId: string): HistoricalRecord | undefined {
    const timelineRecords = this.records.get(timeline);
    return timelineRecords?.get(recordId);
  }

  // ==========================================================================
  // Archive Querying
  // ==========================================================================

  /**
   * Query archive
   */
  async query(params: ArchiveQuery): Promise<QueryResult> {
    const startTime = Date.now();
    const timeline = params.timeline || this.config.timeline;

    if (!timeline) {
      throw new ArchiveError(
        ArchiveErrorCode.INVALID_QUERY,
        'Timeline must be specified'
      );
    }

    const timelineRecords = this.records.get(timeline);
    if (!timelineRecords) {
      return {
        total: 0,
        returned: 0,
        page: 1,
        records: [],
        executionTime: Date.now() - startTime,
      };
    }

    // Filter records
    let filtered = Array.from(timelineRecords.values());

    // Date range filter
    if (params.dateRange) {
      filtered = filtered.filter(record => {
        const recordDate = new Date(record.event.date.start);
        const start = new Date(params.dateRange!.start);
        const end = params.dateRange!.end ? new Date(params.dateRange!.end) : new Date();
        return recordDate >= start && recordDate <= end;
      });
    }

    // Event type filter
    if (params.eventType) {
      const types = Array.isArray(params.eventType) ? params.eventType : [params.eventType];
      filtered = filtered.filter(record => types.includes(record.event.type));
    }

    // Importance filter
    if (params.importance) {
      const levels = Array.isArray(params.importance) ? params.importance : [params.importance];
      filtered = filtered.filter(record =>
        record.metadata.importance && levels.includes(record.metadata.importance)
      );
    }

    // Keyword search
    if (params.keywords && params.keywords.length > 0) {
      filtered = filtered.filter(record =>
        params.keywords!.some(keyword =>
          record.event.title.toLowerCase().includes(keyword.toLowerCase()) ||
          record.event.description.toLowerCase().includes(keyword.toLowerCase())
        )
      );
    }

    // Text search
    if (params.text) {
      const searchText = params.text.toLowerCase();
      filtered = filtered.filter(record =>
        record.event.title.toLowerCase().includes(searchText) ||
        record.event.description.toLowerCase().includes(searchText)
      );
    }

    // Verification filter
    if (params.verified !== undefined) {
      filtered = filtered.filter(record =>
        params.verified ? record.status === 'verified' : record.status !== 'verified'
      );
    }

    // Sort results
    if (params.sortBy) {
      filtered.sort((a, b) => {
        let comparison = 0;
        if (params.sortBy === 'date') {
          comparison = new Date(a.event.date.start).getTime() - new Date(b.event.date.start).getTime();
        } else if (params.sortBy === 'importance') {
          const importanceOrder = { critical: 4, high: 3, medium: 2, low: 1 };
          const aImp = importanceOrder[a.metadata.importance || 'low'];
          const bImp = importanceOrder[b.metadata.importance || 'low'];
          comparison = bImp - aImp;
        }
        return params.sortOrder === 'desc' ? -comparison : comparison;
      });
    }

    // Pagination
    const total = filtered.length;
    const limit = params.limit || 100;
    const offset = params.offset || 0;
    const page = Math.floor(offset / limit) + 1;

    const paginated = filtered.slice(offset, offset + limit);

    return {
      total,
      returned: paginated.length,
      page,
      records: paginated,
      executionTime: Date.now() - startTime,
    };
  }

  // ==========================================================================
  // Verification
  // ==========================================================================

  /**
   * Verify a record
   */
  async verifyRecord(request: VerificationRequest): Promise<VerificationResult> {
    const record = this.getRecord(request.timeline, request.recordId);
    if (!record) {
      throw new ArchiveError(
        ArchiveErrorCode.RECORD_NOT_FOUND,
        `Record ${request.recordId} not found in timeline ${request.timeline}`
      );
    }

    // Perform verification based on method
    let confidence = 0;
    let details = '';

    switch (request.method) {
      case 'cryptographic':
        confidence = this.verifyCryptographic(record);
        details = 'Cryptographic signature verified';
        break;

      case 'multi-witness':
        confidence = this.verifyMultiWitness(record, request.evidence || []);
        details = 'Multi-witness verification completed';
        break;

      case 'scientific':
        confidence = this.verifyScientific(record);
        details = 'Scientific verification completed';
        break;

      case 'consensus':
        confidence = this.verifyConsensus(record);
        details = 'Consensus verification completed';
        break;
    }

    const result: VerificationResult = {
      verificationId: `VER-${Date.now()}`,
      recordId: request.recordId,
      verified: confidence >= ARCHIVE_CONSTANTS.VERIFICATION_THRESHOLD,
      confidence,
      timestamp: new Date(),
      verifier: request.verifier.userId,
      details,
    };

    // Update record verification if confidence improved
    if (confidence > record.verification.confidence) {
      record.verification.confidence = confidence;
      record.verification.verifiers.push(request.verifier.userId);
      record.status = result.verified ? 'verified' : 'unverified';
    }

    return result;
  }

  /**
   * Check record integrity
   */
  checkIntegrity(timeline: TimelineID, recordId: string): IntegrityCheck {
    const record = this.getRecord(timeline, recordId);
    if (!record) {
      throw new ArchiveError(
        ArchiveErrorCode.RECORD_NOT_FOUND,
        `Record ${recordId} not found`
      );
    }

    // Recalculate hash
    const calculatedHash = this.calculateHash(
      {
        timeline: record.timeline,
        event: record.event,
        evidence: record.evidence,
        metadata: record.metadata,
      },
      record.integrity.previousHash
    );

    const hashValid = calculatedHash === record.integrity.hash;

    // Check chain integrity
    const timelineRecords = Array.from(this.records.get(timeline)!.values());
    const recordIndex = timelineRecords.findIndex(r => r.id === recordId);
    let chainIntact = true;

    if (recordIndex > 0) {
      const prevRecord = timelineRecords[recordIndex - 1];
      chainIntact = record.integrity.previousHash === prevRecord.integrity.hash;
    }

    const issues: string[] = [];
    if (!hashValid) issues.push('Hash mismatch');
    if (!chainIntact) issues.push('Chain broken');

    return {
      recordId,
      valid: hashValid && chainIntact,
      hashValid,
      chainIntact,
      issues: issues.length > 0 ? issues : undefined,
      timestamp: new Date(),
    };
  }

  // ==========================================================================
  // Timeline Reconciliation
  // ==========================================================================

  /**
   * Reconcile two timelines
   */
  async reconcileTimelines(params: ReconciliationParams): Promise<ReconciliationResult> {
    const { timeline1, timeline2, strategy, divergencePoint } = params;

    // Get timelines
    const t1Records = Array.from(this.records.get(timeline1)?.values() || []);
    const t2Records = Array.from(this.records.get(timeline2)?.values() || []);

    // Find divergence point
    const divPoint = divergencePoint
      ? this.findDivergenceByDate(t1Records, t2Records, divergencePoint)
      : this.findDivergence(t1Records, t2Records);

    // Calculate differences
    const differences = this.calculateDifferences(
      t1Records.slice(divPoint.index),
      t2Records.slice(divPoint.index)
    );

    // Calculate statistics
    const statistics = this.calculateDivergenceStatistics(differences);

    const result: ReconciliationResult = {
      divergencePoint: divPoint,
      differences,
      statistics,
    };

    // Apply strategy
    if (strategy === 'merge') {
      const mergedTimeline = await this.mergeTimelines(timeline1, timeline2, divPoint.index);
      result.mergedTimeline = mergedTimeline;
    }

    return result;
  }

  /**
   * Find divergence point between timelines
   */
  private findDivergence(
    records1: HistoricalRecord[],
    records2: HistoricalRecord[]
  ): { date: Date; recordId: string; confidence: number; index: number } {
    let left = 0;
    let right = Math.min(records1.length, records2.length);

    while (left < right) {
      const mid = Math.floor((left + right) / 2);

      if (this.recordsEqual(records1[mid], records2[mid])) {
        left = mid + 1;
      } else {
        right = mid;
      }
    }

    const divergenceIndex = left;
    const record = records1[divergenceIndex] || records2[divergenceIndex];

    return {
      date: new Date(record.event.date.start),
      recordId: record.id,
      confidence: 0.99,
      index: divergenceIndex,
    };
  }

  /**
   * Find divergence by specific date
   */
  private findDivergenceByDate(
    records1: HistoricalRecord[],
    records2: HistoricalRecord[],
    date: Date
  ): { date: Date; recordId: string; confidence: number; index: number } {
    const index = records1.findIndex(r => new Date(r.event.date.start) >= date);
    const record = records1[index];

    return {
      date: new Date(record.event.date.start),
      recordId: record.id,
      confidence: 1.0,
      index,
    };
  }

  /**
   * Calculate differences between record sets
   */
  private calculateDifferences(
    records1: HistoricalRecord[],
    records2: HistoricalRecord[]
  ): TimelineDifference {
    const ids1 = new Set(records1.map(r => r.id));
    const ids2 = new Set(records2.map(r => r.id));

    const eventsAdded = records2.filter(r => !ids1.has(r.id));
    const eventsRemoved = records1.filter(r => !ids2.has(r.id));

    const eventsModified = records1
      .filter(r => ids2.has(r.id))
      .map(r1 => {
        const r2 = records2.find(r => r.id === r1.id)!;
        if (!this.recordsEqual(r1, r2)) {
          return {
            timeline1: r1,
            timeline2: r2,
            changes: this.findRecordChanges(r1, r2),
          };
        }
        return null;
      })
      .filter((m): m is NonNullable<typeof m> => m !== null);

    return {
      eventsAdded,
      eventsRemoved,
      eventsModified,
    };
  }

  /**
   * Calculate divergence statistics
   */
  private calculateDivergenceStatistics(
    differences: TimelineDifference
  ): DivergenceStatistics {
    const totalDifferences =
      differences.eventsAdded.length +
      differences.eventsRemoved.length +
      differences.eventsModified.length;

    const significantDifferences = [
      ...differences.eventsAdded,
      ...differences.eventsRemoved,
      ...differences.eventsModified.map(m => m.timeline1),
    ].filter(r => r.metadata.importance === 'critical' || r.metadata.importance === 'high')
      .length;

    const minorDifferences = totalDifferences - significantDifferences;

    let divergenceSeverity: 'minor' | 'moderate' | 'major' | 'catastrophic';
    if (totalDifferences < 10 && significantDifferences === 0) {
      divergenceSeverity = 'minor';
    } else if (totalDifferences < 100 && significantDifferences < 10) {
      divergenceSeverity = 'moderate';
    } else if (totalDifferences < 1000) {
      divergenceSeverity = 'major';
    } else {
      divergenceSeverity = 'catastrophic';
    }

    return {
      totalDifferences,
      significantDifferences,
      minorDifferences,
      divergenceSeverity,
    };
  }

  /**
   * Merge two timelines
   */
  private async mergeTimelines(
    timeline1: TimelineID,
    timeline2: TimelineID,
    divergenceIndex: number
  ): Promise<Timeline> {
    const mergedId = `MERGED-${Date.now()}`;
    const merged = this.createTimeline(mergedId, `Merged ${timeline1} + ${timeline2}`);

    const t1Records = Array.from(this.records.get(timeline1)?.values() || []);
    const t2Records = Array.from(this.records.get(timeline2)?.values() || []);

    // Add common records
    const commonRecords = t1Records.slice(0, divergenceIndex);
    for (const record of commonRecords) {
      await this.recordEvent({
        event: record.event,
        timeline: mergedId,
        evidence: record.evidence,
        verification: record.verification,
        metadata: record.metadata,
      });
    }

    // Add unique records from both timelines
    const divergent1 = t1Records.slice(divergenceIndex);
    const divergent2 = t2Records.slice(divergenceIndex);

    // Simple merge: add all records, mark conflicts
    for (const record of [...divergent1, ...divergent2]) {
      await this.recordEvent({
        event: record.event,
        timeline: mergedId,
        evidence: record.evidence,
        verification: record.verification,
        metadata: {
          ...record.metadata,
          keywords: [...(record.metadata.keywords || []), 'merged-timeline'],
        },
      });
    }

    return merged;
  }

  // ==========================================================================
  // Alteration Preservation
  // ==========================================================================

  /**
   * Preserve timeline alteration
   */
  preserveAlteration(params: {
    timeline: TimelineID;
    alteration: Partial<AlterationLog>;
  }): AlterationLog {
    const { timeline, alteration } = params;

    // Create preserved copy of original timeline
    const preservedId = `${timeline}-PRESERVED-${Date.now()}`;
    const preservedTimeline = this.createTimeline(
      preservedId,
      `Preserved ${timeline}`,
      timeline
    );

    // Copy all records
    const originalRecords = this.records.get(timeline);
    if (originalRecords) {
      this.records.set(preservedId, new Map(originalRecords));
      preservedTimeline.recordCount = originalRecords.size;
    }

    // Create alteration log
    const log: AlterationLog = {
      id: alteration.id || `ALT-${Date.now()}`,
      timeline,
      detectedAt: new Date(),
      divergencePoint: alteration.divergencePoint || {
        date: new Date(),
        event: 'Unknown',
      },
      alterationType: alteration.alterationType || 'event-modification',
      severity: alteration.severity || 'moderate',
      affectedRecords: alteration.affectedRecords || 0,
      temporalEnergy: alteration.temporalEnergy,
      source: alteration.source || {
        method: 'manual',
        detected: 'user-reported',
        confidence: 0.95,
      },
      preservation: {
        originalTimelineId: preservedId,
        alteredTimelineId: timeline,
        preservationMethod: 'complete-snapshot',
        storageNodes: this.config.storage.nodes || ['local'],
      },
    };

    return log;
  }

  // ==========================================================================
  // Export and Research Access
  // ==========================================================================

  /**
   * Export archive data
   */
  async exportArchive(request: ExportRequest): Promise<ExportResult> {
    const records = await this.query({
      timeline: request.timeline,
      dateRange: request.dateRange,
    });

    // Simulate export
    const exportId = `EXP-${Date.now()}`;
    const downloadUrl = `https://archive.wiastandards.com/exports/${exportId}.${request.format}`;

    return {
      exportId,
      downloadUrl,
      fileSize: records.total * 1024, // Estimate
      recordCount: records.total,
      createdAt: new Date(),
      expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000), // 7 days
    };
  }

  /**
   * Authenticate user
   */
  async authenticate(request: AuthRequest): Promise<AuthResponse> {
    // Simulate authentication
    const sessionToken = `SESSION-${Date.now()}-${Math.random().toString(36).substring(7)}`;

    const rateLimit = ARCHIVE_CONSTANTS.QUERY_LIMITS[request.accessLevel.toUpperCase() as keyof typeof ARCHIVE_CONSTANTS.QUERY_LIMITS] ||
      ARCHIVE_CONSTANTS.QUERY_LIMITS.PUBLIC;

    return {
      sessionToken,
      expiresAt: new Date(Date.now() + ARCHIVE_CONSTANTS.SESSION_TIMEOUT),
      permissions: this.getPermissionsForLevel(request.accessLevel),
      rateLimit,
    };
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Generate record ID
   */
  private generateRecordId(timeline: TimelineID, date: Date): string {
    const year = date.getFullYear();
    const sequence = this.records.get(timeline)?.size || 0;
    return `REC-${timeline}-${year}-${sequence.toString().padStart(6, '0')}`;
  }

  /**
   * Calculate hash of record data
   */
  private calculateHash(data: unknown, previousHash: string): string {
    // Simplified hash calculation (in production, use SHA3-512)
    const dataString = JSON.stringify(data) + previousHash;
    let hash = 0;
    for (let i = 0; i < dataString.length; i++) {
      const char = dataString.charCodeAt(i);
      hash = (hash << 5) - hash + char;
      hash = hash & hash; // Convert to 32bit integer
    }

    // Pad to 128 hex characters (simulating SHA3-512)
    const hashStr = Math.abs(hash).toString(16).padStart(8, '0');
    return hashStr.repeat(16);
  }

  /**
   * Check if two records are equal
   */
  private recordsEqual(r1: HistoricalRecord, r2: HistoricalRecord): boolean {
    return r1.integrity.hash === r2.integrity.hash;
  }

  /**
   * Find changes between two records
   */
  private findRecordChanges(r1: HistoricalRecord, r2: HistoricalRecord): Array<{
    field: string;
    timeline1Value: unknown;
    timeline2Value: unknown;
  }> {
    const changes: Array<{
      field: string;
      timeline1Value: unknown;
      timeline2Value: unknown;
    }> = [];

    if (r1.event.title !== r2.event.title) {
      changes.push({
        field: 'event.title',
        timeline1Value: r1.event.title,
        timeline2Value: r2.event.title,
      });
    }

    if (r1.event.description !== r2.event.description) {
      changes.push({
        field: 'event.description',
        timeline1Value: r1.event.description,
        timeline2Value: r2.event.description,
      });
    }

    return changes;
  }

  /**
   * Verify cryptographic signature
   */
  private verifyCryptographic(record: HistoricalRecord): number {
    // Simplified verification
    return record.verification.signature ? 0.99 : 0.80;
  }

  /**
   * Verify with multiple witnesses
   */
  private verifyMultiWitness(record: HistoricalRecord, additionalEvidence: any[]): number {
    const witnessCount = record.verification.verifiers.length + additionalEvidence.length;
    return Math.min(0.7 + witnessCount * 0.05, 0.99);
  }

  /**
   * Verify using scientific methods
   */
  private verifyScientific(record: HistoricalRecord): number {
    // Simplified: check if evidence includes scientific data
    const hasScientific = record.evidence.sources.some(s => s.type === 'scientific');
    return hasScientific ? 0.95 : 0.85;
  }

  /**
   * Verify using consensus
   */
  private verifyConsensus(record: HistoricalRecord): number {
    // Simplified: use existing confidence
    return record.verification.confidence;
  }

  /**
   * Get permissions for access level
   */
  private getPermissionsForLevel(level: string): Array<{ resource: string; actions: string[] }> {
    const permissions: Record<string, Array<{ resource: string; actions: string[] }>> = {
      public: [{ resource: 'records', actions: ['read'] }],
      researcher: [
        { resource: 'records', actions: ['read', 'query'] },
        { resource: 'timelines', actions: ['read'] },
      ],
      curator: [
        { resource: 'records', actions: ['read', 'query', 'create', 'update'] },
        { resource: 'timelines', actions: ['read', 'create'] },
      ],
      verifier: [
        { resource: 'records', actions: ['read', 'query', 'verify'] },
        { resource: 'timelines', actions: ['read'] },
      ],
      admin: [
        { resource: '*', actions: ['*'] },
      ],
    };

    return permissions[level] || permissions.public;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Record event (standalone function)
 */
export async function recordEvent(
  archive: HistoricalArchiveSDK,
  event: Event,
  timeline: TimelineID
): Promise<HistoricalRecord> {
  return archive.recordEvent({ event, timeline });
}

/**
 * Query archive (standalone function)
 */
export async function queryArchive(
  archive: HistoricalArchiveSDK,
  query: ArchiveQuery
): Promise<QueryResult> {
  return archive.query(query);
}

/**
 * Verify record (standalone function)
 */
export async function verifyRecord(
  archive: HistoricalArchiveSDK,
  request: VerificationRequest
): Promise<VerificationResult> {
  return archive.verifyRecord(request);
}

/**
 * Reconcile timelines (standalone function)
 */
export async function reconcileTimelines(
  archive: HistoricalArchiveSDK,
  params: ReconciliationParams
): Promise<ReconciliationResult> {
  return archive.reconcileTimelines(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { HistoricalArchiveSDK };
export default HistoricalArchiveSDK;
