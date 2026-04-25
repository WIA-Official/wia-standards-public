/**
 * WIA-TIME-010: Paradox Prevention SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive paradox prevention mechanisms including:
 * - Real-time paradox detection
 * - Severity classification
 * - Prevention strategies
 * - Timeline branching
 * - Emergency rollback
 */

import {
  ParadoxType,
  ParadoxSeverity,
  RecommendedAction,
  ParadoxDetection,
  ParadoxAnalysis,
  Entity,
  Action,
  Timeline,
  TimelineEvent,
  CausalChain,
  Contradiction,
  PreventionStrategy,
  NovikovCheck,
  ProtectedEvent,
  ObserverMode,
  TimelineBranch,
  TimelineDivergence,
  TimelineSnapshot,
  RollbackRequest,
  RollbackResult,
  SelectiveRollbackRequest,
  MonitoringConfig,
  MonitoringStatus,
  CausalLoop,
  LoopStabilization,
  ParadoxErrorCode,
  ParadoxPreventionError,
  PARADOX_CONSTANTS,
  DetectionAlgorithm,
  Alert,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-TIME-010 Paradox Prevention SDK
 */
export class ParadoxPreventionSDK {
  private version = '1.0.0';
  private initialized = false;
  private protectedEvents: Map<string, ProtectedEvent> = new Map();
  private snapshots: Map<string, TimelineSnapshot> = new Map();
  private monitoring: MonitoringStatus | null = null;

  constructor() {
    this.initialized = true;
    this.initializeProtectedEvents();
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ============================================================================
  // Paradox Detection
  // ============================================================================

  /**
   * Detect potential paradox in an action
   *
   * @param params - Detection parameters
   * @returns Paradox detection result
   */
  detectParadox(params: {
    action: Action;
    timeline: Timeline;
    actor: Entity;
    targetTime?: Date;
  }): ParadoxDetection {
    const { action, timeline, actor } = params;
    const targetTime = params.targetTime || action.targetTime;

    // Run multiple detection algorithms
    const checks = {
      grandfather: this.checkGrandfatherParadox(action, actor, timeline),
      bootstrap: this.checkBootstrapParadox(action, timeline),
      causalViolation: this.checkCausalViolation(action, timeline),
      protected: this.checkProtectedEvent(action),
      novikov: this.checkNovikovConsistency(action, timeline),
    };

    // Find highest severity paradox
    let paradoxType: ParadoxType | null = null;
    let maxSeverity = ParadoxSeverity.SAFE;
    let description = 'No paradox detected';
    let affectedEntities: Entity[] = [];
    let preventionStrategy: PreventionStrategy | null = null;

    if (checks.grandfather.detected) {
      paradoxType = ParadoxType.GRANDFATHER;
      maxSeverity = ParadoxSeverity.CRITICAL;
      description = checks.grandfather.description;
      affectedEntities = checks.grandfather.affected;
    } else if (checks.protected.detected) {
      paradoxType = ParadoxType.CAUSAL_VIOLATION;
      maxSeverity = ParadoxSeverity.SEVERE;
      description = checks.protected.description;
    } else if (checks.bootstrap.detected) {
      paradoxType = ParadoxType.BOOTSTRAP;
      maxSeverity = ParadoxSeverity.MINOR;
      description = checks.bootstrap.description;
    } else if (checks.causalViolation.detected) {
      paradoxType = ParadoxType.CAUSAL_VIOLATION;
      maxSeverity = ParadoxSeverity.MODERATE;
      description = checks.causalViolation.description;
    }

    // Get prevention strategy
    if (paradoxType) {
      preventionStrategy = this.getPreventionStrategy(paradoxType, maxSeverity);
    }

    // Build causal chain
    const causalChain = this.buildCausalChain(action, timeline);

    // Perform detailed analysis
    const analysis = this.analyzeParadox(
      paradoxType,
      maxSeverity,
      action,
      actor,
      timeline
    );

    // Determine recommendation
    const recommendation = this.getRecommendation(maxSeverity);

    return {
      paradoxDetected: maxSeverity > ParadoxSeverity.SAFE,
      type: paradoxType,
      severity: maxSeverity,
      description,
      affectedEntities,
      causalChain,
      recommendation,
      preventionStrategy,
      detectedAt: new Date(),
      analysis,
    };
  }

  /**
   * Classify paradox severity
   */
  classifyParadoxSeverity(params: {
    type: ParadoxType;
    affectedCount: number;
    timelineStability: number;
    timeSpanYears: number;
    threatensExistence: boolean;
    causalityViolation: number;
  }): {
    severity: ParadoxSeverity;
    levelName: string;
    impactScore: number;
    recommendation: RecommendedAction;
  } {
    const { type, affectedCount, timelineStability, timeSpanYears, threatensExistence, causalityViolation } = params;

    // Base severity from type
    const typeWeights: Record<ParadoxType, number> = {
      [ParadoxType.GRANDFATHER]: 4,
      [ParadoxType.POLCHINSKI]: 4,
      [ParadoxType.BOOTSTRAP]: 1,
      [ParadoxType.PREDESTINATION]: 1,
      [ParadoxType.ONTOLOGICAL]: 2,
      [ParadoxType.INFORMATION]: 2,
      [ParadoxType.CAUSAL_VIOLATION]: 3,
      [ParadoxType.TIMELINE_CORRUPTION]: 4,
    };

    const baseSeverity = typeWeights[type] || 2;

    // Calculate impact factors
    const impactFactors = {
      travelerExistence: threatensExistence ? 4 : 0,
      timelineStability: (1 - timelineStability) * 3,
      affectedCount: Math.min(affectedCount / 100, 1) * 2,
      temporalScope: Math.min(timeSpanYears / 100, 1) * 2,
      causalityViolation: causalityViolation * 3,
    };

    const totalImpact = Object.values(impactFactors).reduce((sum, val) => sum + val, 0);

    // Calculate weighted severity
    const weightedSeverity = (baseSeverity + totalImpact) / 2;
    const finalSeverity = Math.min(4, Math.round(weightedSeverity)) as ParadoxSeverity;

    return {
      severity: finalSeverity,
      levelName: this.getSeverityName(finalSeverity),
      impactScore: totalImpact,
      recommendation: this.getRecommendation(finalSeverity),
    };
  }

  // ============================================================================
  // Prevention Mechanisms
  // ============================================================================

  /**
   * Prevent action based on paradox detection
   */
  preventAction(detection: ParadoxDetection): {
    prevented: boolean;
    reason: string;
    alternative?: string;
  } {
    if (detection.severity >= ParadoxSeverity.SEVERE) {
      return {
        prevented: true,
        reason: `Action blocked: ${detection.description}`,
        alternative: detection.preventionStrategy?.implementation,
      };
    }

    return {
      prevented: false,
      reason: 'Action allowed',
    };
  }

  /**
   * Create observer mode for safe time travel
   */
  createObserverMode(params: {
    traveler: Entity;
    targetTime: Date;
  }): ObserverMode {
    const { traveler, targetTime } = params;

    return {
      active: true,
      traveler,
      targetTime,
      allowedActions: ['observe', 'record', 'measure'],
      blockedActions: ['touch', 'speak', 'modify', 'create', 'destroy'],
      observationLog: [],
      cloaking: PARADOX_CONSTANTS.DEFAULT_CLOAKING,
    };
  }

  /**
   * Check Novikov self-consistency
   */
  checkNovikovConsistency(action: Action, timeline: Timeline): NovikovCheck {
    const contradictions: Contradiction[] = [];
    const violations: string[] = [];

    // Simulate action
    const simulatedTimeline = this.simulateAction(action, timeline);

    // Find contradictions
    const foundContradictions = this.findContradictions(timeline, simulatedTimeline);
    contradictions.push(...foundContradictions);

    // Check if action creates paradox
    if (contradictions.length > 0) {
      violations.push('Action creates timeline contradictions');
    }

    // Calculate probability (Novikov: paradox probability = 0)
    const probability = contradictions.length === 0 ? 1.0 : 0.0;

    return {
      isConsistent: contradictions.length === 0,
      probability,
      contradictions,
      violations,
      recommendations: contradictions.length > 0 
        ? ['Modify action to avoid contradictions', 'Use timeline branching'] 
        : [],
    };
  }

  /**
   * Stabilize causal loop
   */
  stabilizeCausalLoop(loop: CausalLoop): LoopStabilization {
    // Verify loop closure
    const closed = this.verifyLoopClosure(loop);
    if (!closed) {
      return {
        stable: false,
        stability: 0,
        entropyChange: 0,
        informationConserved: false,
        perturbations: [],
        recommendations: ['Adjust final state to match initial state'],
        error: 'Loop does not close properly',
      };
    }

    // Calculate entropy change
    const entropyChange = this.calculateEntropyChange(loop);
    if (entropyChange < 0) {
      return {
        stable: false,
        stability: 0,
        entropyChange,
        informationConserved: false,
        perturbations: [],
        recommendations: ['Entropy decrease violates thermodynamics'],
        error: 'Negative entropy change detected',
      };
    }

    // Check information conservation
    const infoConserved = this.checkInformationConservation(loop);

    // Detect perturbations
    const perturbations = loop.perturbations || [];

    // Calculate stability
    const stability = perturbations.length === 0 && infoConserved ? 0.98 : 0.75;

    return {
      stable: stability > PARADOX_CONSTANTS.MIN_STABILITY,
      stability,
      entropyChange,
      informationConserved: infoConserved,
      perturbations,
      recommendations: stability > PARADOX_CONSTANTS.MIN_STABILITY 
        ? ['Loop is stable'] 
        : ['Isolate loop from external influences'],
    };
  }

  // ============================================================================
  // Timeline Management
  // ============================================================================

  /**
   * Create timeline branch to resolve paradox
   */
  createTimelineBranch(params: {
    timeline: Timeline;
    branchPoint: Date;
    reason: string;
  }): TimelineBranch {
    const { timeline, branchPoint, reason } = params;

    // Generate branch ID
    const branchId = this.generateId('TL-BRANCH');

    // Calculate probability (simplified MWI)
    const probability = 0.5; // In real MWI, calculated from quantum amplitudes

    // Calculate divergence
    const divergence = 0; // Initial divergence at branch point

    const branch: TimelineBranch = {
      id: branchId,
      parentId: timeline.id,
      branchPoint,
      reason,
      probability,
      divergence,
      created: new Date(),
      stable: true,
    };

    return branch;
  }

  /**
   * Calculate timeline divergence
   */
  calculateTimelineDivergence(
    original: Timeline,
    current: Timeline
  ): TimelineDivergence {
    let divergence = 0;
    let majorChanges = 0;
    let minorChanges = 0;

    // Compare events
    const originalEvents = new Set(original.events.map((e) => e.id));
    const currentEvents = new Set(current.events.map((e) => e.id));

    for (const event of original.events) {
      if (!currentEvents.has(event.id)) {
        if (event.type === 'major' || event.type === 'critical') {
          majorChanges++;
          divergence += 10;
        } else {
          minorChanges++;
          divergence += 1;
        }
      }
    }

    // Compare entities
    const entityDiff = Math.abs(original.entities.length - current.entities.length);
    divergence += entityDiff * 5;

    // Normalize divergence (0-1)
    const maxPossibleDivergence = original.events.length * 10 + original.entities.length * 5;
    const normalizedDivergence = Math.min(1, divergence / Math.max(1, maxPossibleDivergence));

    // Classify severity
    let severity: TimelineDivergence['severity'];
    if (normalizedDivergence < 0.1) severity = 'negligible';
    else if (normalizedDivergence < 0.3) severity = 'minor';
    else if (normalizedDivergence < 0.5) severity = 'moderate';
    else if (normalizedDivergence < 0.8) severity = 'severe';
    else severity = 'critical';

    return {
      divergence: normalizedDivergence,
      severity,
      majorChanges,
      minorChanges,
      stable: normalizedDivergence < PARADOX_CONSTANTS.MAX_DIVERGENCE,
      breakdown: {
        events: majorChanges + minorChanges,
        entities: entityDiff,
        physicalLaws: 0,
      },
    };
  }

  // ============================================================================
  // Snapshot and Rollback
  // ============================================================================

  /**
   * Create timeline snapshot
   */
  createSnapshot(timeline: Timeline, label = 'auto'): TimelineSnapshot {
    const snapshotId = this.generateId('SNAP');

    const snapshot: TimelineSnapshot = {
      id: snapshotId,
      timelineId: timeline.id,
      label,
      timestamp: new Date(),
      eventCount: timeline.events.length,
      entityCount: timeline.entities.length,
      checksum: this.calculateChecksum(timeline),
      verified: true,
      sizeBytes: this.estimateSize(timeline),
      data: this.serializeTimeline(timeline),
    };

    // Store snapshot
    this.snapshots.set(snapshotId, snapshot);

    return snapshot;
  }

  /**
   * Initiate timeline rollback
   */
  async initiateTimelineRollback(params: RollbackRequest): Promise<RollbackResult> {
    const { targetSnapshot, preserveMemories, createBackup } = params;

    // Retrieve snapshot
    const snapshot = this.snapshots.get(targetSnapshot);

    if (!snapshot) {
      throw new ParadoxPreventionError(
        ParadoxErrorCode.SNAPSHOT_CORRUPTED,
        `Snapshot ${targetSnapshot} not found`
      );
    }

    // Verify integrity
    if (!snapshot.verified) {
      throw new ParadoxPreventionError(
        ParadoxErrorCode.SNAPSHOT_CORRUPTED,
        'Snapshot integrity check failed'
      );
    }

    // Create backup if requested
    let backupId: string | undefined;
    if (createBackup) {
      const backup = this.createSnapshot(
        this.deserializeTimeline(snapshot),
        'pre_rollback_backup'
      );
      backupId = backup.id;
    }

    // Execute rollback (simulation)
    const affectedEntities = snapshot.entityCount;
    const eventsRemoved = 0; // Would be calculated in real implementation

    return {
      success: true,
      snapshotId: targetSnapshot,
      rollbackTime: snapshot.timestamp,
      backupId,
      affectedEntities,
      eventsRemoved,
      memoriesPreserved: preserveMemories,
    };
  }

  /**
   * Selective event rollback
   */
  selectiveRollback(params: SelectiveRollbackRequest): {
    success: boolean;
    eventsRolledBack: number;
    cascadingEffects: number;
    error?: string;
  } {
    const { eventIds, targetTime, includeCascading } = params;

    // Simulate selective rollback
    const cascadingEffects = includeCascading ? eventIds.length * 2 : 0;

    return {
      success: true,
      eventsRolledBack: eventIds.length,
      cascadingEffects,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private checkGrandfatherParadox(action: Action, actor: Entity, timeline: Timeline): {
    detected: boolean;
    description: string;
    affected: Entity[];
  } {
    // Check if action affects actor's ancestors
    if (actor.ancestors && actor.ancestors.length > 0) {
      const ancestors = actor.ancestors;
      const targetEntity = typeof action.target === 'string' ? action.target : action.target?.id;

      if (targetEntity && ancestors.includes(targetEntity)) {
        return {
          detected: true,
          description: `Action would affect ancestor ${targetEntity}, creating grandfather paradox`,
          affected: [actor],
        };
      }
    }

    return { detected: false, description: '', affected: [] };
  }

  private checkBootstrapParadox(action: Action, timeline: Timeline): {
    detected: boolean;
    description: string;
  } {
    // Check for information/object without origin
    if (action.type === 'create') {
      // Simplified check: if creator is also recipient
      const creator = typeof action.actor === 'string' ? action.actor : action.actor.id;
      const target = typeof action.target === 'string' ? action.target : action.target?.id;

      if (creator === target) {
        return {
          detected: true,
          description: 'Bootstrap paradox: entity creates itself',
        };
      }
    }

    return { detected: false, description: '' };
  }

  private checkCausalViolation(action: Action, timeline: Timeline): {
    detected: boolean;
    description: string;
  } {
    // Check if action violates causality
    // Simplified: check if effect precedes cause
    if (action.expectedEffects) {
      for (const effect of action.expectedEffects) {
        if (effect.time < action.targetTime) {
          return {
            detected: true,
            description: 'Causal violation: effect precedes cause',
          };
        }
      }
    }

    return { detected: false, description: '' };
  }

  private checkProtectedEvent(action: Action): {
    detected: boolean;
    description: string;
  } {
    const targetId = typeof action.target === 'string' ? action.target : action.target?.id;

    if (targetId) {
      const protected_event = this.protectedEvents.get(targetId);
      if (protected_event && protected_event.locked) {
        return {
          detected: true,
          description: `Protected event: ${protected_event.justification}`,
        };
      }
    }

    return { detected: false, description: '' };
  }

  private buildCausalChain(action: Action, timeline: Timeline): CausalChain {
    return {
      id: this.generateId('CC'),
      events: [],
      length: 0,
      timeSpan: 0,
      isClosed: false,
      isConsistent: true,
      contradictions: [],
    };
  }

  private analyzeParadox(
    type: ParadoxType | null,
    severity: ParadoxSeverity,
    action: Action,
    actor: Entity,
    timeline: Timeline
  ): ParadoxAnalysis {
    return {
      baseSeverity: severity,
      impactScore: severity * 2,
      factors: {
        travelerExistence: severity === ParadoxSeverity.CRITICAL ? 1 : 0,
        timelineStability: timeline.integrity,
        affectedCount: 1,
        temporalScope: 1,
        causalityViolation: severity >= ParadoxSeverity.MODERATE ? 0.5 : 0,
      },
      probability: severity === ParadoxSeverity.CRITICAL ? 0 : 0.8,
      selfConsistent: severity <= ParadoxSeverity.MINOR,
      novikovCompatible: severity === ParadoxSeverity.SAFE,
    };
  }

  private getPreventionStrategy(type: ParadoxType, severity: ParadoxSeverity): PreventionStrategy {
    const strategies: Record<ParadoxType, PreventionStrategy> = {
      [ParadoxType.GRANDFATHER]: {
        name: 'Grandfather Paradox Prevention',
        type: 'novikov',
        description: 'Prevent actions that eliminate traveler existence',
        implementation: 'Block action with probability 0 (Novikov principle)',
        successProbability: 1.0,
        sideEffects: ['Action physically impossible to complete'],
      },
      [ParadoxType.BOOTSTRAP]: {
        name: 'Bootstrap Loop Management',
        type: 'timeline_branch',
        description: 'Allow if causally consistent, track information origin',
        implementation: 'Monitor causal loop stability',
        successProbability: 0.95,
        sideEffects: ['Entropy considerations required'],
      },
      [ParadoxType.PREDESTINATION]: {
        name: 'Predestination Loop Acceptance',
        type: 'novikov',
        description: 'Allow self-consistent predestination paradoxes',
        implementation: 'Verify loop consistency',
        successProbability: 0.98,
        sideEffects: ['Free will considerations'],
      },
      [ParadoxType.ONTOLOGICAL]: {
        name: 'Ontological Paradox Tracking',
        type: 'observer_mode',
        description: 'Track object origin, monitor stability',
        implementation: 'Maintain object history log',
        successProbability: 0.85,
        sideEffects: ['Origin uncertainty'],
      },
      [ParadoxType.POLCHINSKI]: {
        name: 'Polchinski Resolution',
        type: 'novikov',
        description: 'Find self-consistent trajectory',
        implementation: 'Calculate glancing collision angle',
        successProbability: 0.90,
        sideEffects: ['Multiple solutions possible'],
      },
      [ParadoxType.INFORMATION]: {
        name: 'Information Conservation',
        type: 'timeline_branch',
        description: 'Ensure entropy does not decrease',
        implementation: 'Monitor entropy changes',
        successProbability: 0.92,
        sideEffects: ['Thermodynamic constraints'],
      },
      [ParadoxType.CAUSAL_VIOLATION]: {
        name: 'Causality Enforcement',
        type: 'protected_event',
        description: 'Prevent violation of cause-effect ordering',
        implementation: 'Block actions that create causal loops',
        successProbability: 0.95,
        sideEffects: ['Limited action freedom'],
      },
      [ParadoxType.TIMELINE_CORRUPTION]: {
        name: 'Timeline Restoration',
        type: 'block_action',
        description: 'Emergency rollback to stable state',
        implementation: 'Restore from latest snapshot',
        successProbability: 0.88,
        sideEffects: ['Data loss possible'],
      },
    };

    return strategies[type];
  }

  private getRecommendation(severity: ParadoxSeverity): RecommendedAction {
    const recommendations: Record<ParadoxSeverity, RecommendedAction> = {
      [ParadoxSeverity.SAFE]: RecommendedAction.PROCEED,
      [ParadoxSeverity.MINOR]: RecommendedAction.PROCEED_WITH_MONITORING,
      [ParadoxSeverity.MODERATE]: RecommendedAction.PROCEED_WITH_CAUTION,
      [ParadoxSeverity.SEVERE]: RecommendedAction.PREVENT_ACTION,
      [ParadoxSeverity.CRITICAL]: RecommendedAction.EMERGENCY_ABORT,
    };

    return recommendations[severity];
  }

  private getSeverityName(severity: ParadoxSeverity): string {
    const names: Record<ParadoxSeverity, string> = {
      [ParadoxSeverity.SAFE]: 'SAFE',
      [ParadoxSeverity.MINOR]: 'MINOR',
      [ParadoxSeverity.MODERATE]: 'MODERATE',
      [ParadoxSeverity.SEVERE]: 'SEVERE',
      [ParadoxSeverity.CRITICAL]: 'CRITICAL',
    };

    return names[severity];
  }

  private simulateAction(action: Action, timeline: Timeline): Timeline {
    // Simplified simulation: return copy of timeline
    return { ...timeline, updated: new Date() };
  }

  private findContradictions(original: Timeline, simulated: Timeline): Contradiction[] {
    // Simplified: would compare timelines and find logical contradictions
    return [];
  }

  private verifyLoopClosure(loop: CausalLoop): boolean {
    // Verify first and last events match
    if (loop.events.length < 2) return false;

    const first = loop.events[0];
    const last = loop.events[loop.events.length - 1];

    // Simplified check
    return first.id === last.id;
  }

  private calculateEntropyChange(loop: CausalLoop): number {
    // Simplified: return stored entropy change or 0
    return loop.entropyChange || 0;
  }

  private checkInformationConservation(loop: CausalLoop): boolean {
    // Simplified: return stored value or true
    return loop.informationConserved !== false;
  }

  private initializeProtectedEvents() {
    // Initialize some critical protected events
    // In real implementation, would load from database
  }

  private generateId(prefix: string): string {
    return `${prefix}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private calculateChecksum(timeline: Timeline): string {
    // Simplified checksum
    return `checksum-${timeline.id}-${timeline.events.length}`;
  }

  private estimateSize(timeline: Timeline): number {
    // Simplified size estimation
    return timeline.events.length * 1000 + timeline.entities.length * 500;
  }

  private serializeTimeline(timeline: Timeline): unknown {
    return timeline;
  }

  private deserializeTimeline(snapshot: TimelineSnapshot): Timeline {
    return snapshot.data as Timeline;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Detect paradox (standalone function)
 */
export function detectParadox(params: {
  action: Action;
  timeline: Timeline;
  actor: Entity;
  targetTime?: Date;
}): ParadoxDetection {
  const sdk = new ParadoxPreventionSDK();
  return sdk.detectParadox(params);
}

/**
 * Prevent grandfather paradox (standalone function)
 */
export function preventGrandfatherParadox(
  action: Action,
  actor: Entity,
  timeline: Timeline
): { prevented: boolean; reason: string } {
  const sdk = new ParadoxPreventionSDK();
  const detection = sdk.detectParadox({ action, timeline, actor });

  if (detection.type === ParadoxType.GRANDFATHER) {
    return sdk.preventAction(detection);
  }

  return { prevented: false, reason: 'No grandfather paradox detected' };
}

/**
 * Resolve bootstrap paradox (standalone function)
 */
export function resolveBootstrapParadox(
  loop: CausalLoop
): LoopStabilization {
  const sdk = new ParadoxPreventionSDK();
  return sdk.stabilizeCausalLoop(loop);
}

/**
 * Classify paradox severity (standalone function)
 */
export function classifyParadoxSeverity(params: {
  type: ParadoxType;
  affectedCount: number;
  timelineStability: number;
  timeSpanYears: number;
  threatensExistence: boolean;
  causalityViolation: number;
}): {
  severity: ParadoxSeverity;
  levelName: string;
  impactScore: number;
  recommendation: RecommendedAction;
} {
  const sdk = new ParadoxPreventionSDK();
  return sdk.classifyParadoxSeverity(params);
}

/**
 * Initiate timeline rollback (standalone function)
 */
export async function initiateTimelineRollback(
  params: RollbackRequest
): Promise<RollbackResult> {
  const sdk = new ParadoxPreventionSDK();
  return sdk.initiateTimelineRollback(params);
}

/**
 * Create timeline branch (standalone function)
 */
export function createTimelineBranch(params: {
  timeline: Timeline;
  branchPoint: Date;
  reason: string;
}): TimelineBranch {
  const sdk = new ParadoxPreventionSDK();
  return sdk.createTimelineBranch(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { ParadoxPreventionSDK };
export default ParadoxPreventionSDK;
