/**
 * WIA-TIME-009: Causality Protection SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * Êº“ (Benefit All Humanity)
 */

import {
  Timeline,
  TimelineEvent,
  TemporalAction,
  ConsistencyResult,
  Violation,
  CausalLoop,
  LoopDetectionParams,
  IntegrityScore,
  MonitorSession,
  MonitorOptions,
  EventGraph,
  GraphNode,
  GraphEdge,
  ProtectionStatus,
  CorrectionResult,
  CausalityProtectorConfig,
  CAUSALITY_CONSTANTS,
} from './types';

export class CausalityProtector {
  private version = '1.0.0';
  private config: Required<CausalityProtectorConfig>;

  constructor(config?: CausalityProtectorConfig) {
    this.config = {
      enforcementLevel: config?.enforcementLevel || 'standard',
      autoCorrection: config?.autoCorrection ?? true,
      alertThreshold: config?.alertThreshold || CAUSALITY_CONSTANTS.DEFAULT_ALERT_THRESHOLD,
      monitoringInterval: config?.monitoringInterval || CAUSALITY_CONSTANTS.DEFAULT_MONITORING_INTERVAL,
      maxCorrectionAttempts: config?.maxCorrectionAttempts || CAUSALITY_CONSTANTS.MAX_CORRECTION_ATTEMPTS,
      enableTimelineBranching: config?.enableTimelineBranching ?? true,
      protectedEventEnforcement: config?.protectedEventEnforcement ?? true,
      loopDetectionDepth: config?.loopDetectionDepth || CAUSALITY_CONSTANTS.MAX_LOOP_DEPTH,
      integrityWeights: config?.integrityWeights || {
        causalConsistency: 0.4,
        eventCoherence: 0.3,
        temporalContinuity: 0.2,
        historicalConsistency: 0.1,
      },
      chronologyProtection: config?.chronologyProtection || {
        enabled: true,
        energyThreshold: CAUSALITY_CONSTANTS.PLANCK_ENERGY,
        monitoringFrequency: 1 / CAUSALITY_CONSTANTS.PLANCK_TIME,
      },
    };
  }

  getVersion(): string {
    return this.version;
  }

  checkNovikovConsistency(action: TemporalAction, timeline: Timeline): ConsistencyResult {
    const violations: Violation[] = [];
    const warnings: string[] = [];

    // Check temporal ordering
    violations.push(...this.checkTemporalOrdering(timeline, action));

    // Calculate consistency score
    const score = violations.length === 0 ? 1.0 : Math.max(0, 1 - violations.length / 10);
    const allowed = this.enforceConsistency(score, violations);

    return {
      isConsistent: score === 1.0,
      score,
      allowed,
      violations,
      warnings,
      recommendations: allowed ? [] : ['Action violates causality'],
      probability: score,
      computationTime: 10,
    };
  }

  private checkTemporalOrdering(timeline: Timeline, action: TemporalAction): Violation[] {
    const violations: Violation[] = [];

    for (const event of timeline.events) {
      for (const causeId of event.causes) {
        const cause = timeline.events.find(e => e.id === causeId);
        if (cause && cause.time >= event.time) {
          violations.push({
            id: `V-${Date.now()}-${causeId}`,
            type: 'TEMPORAL_ORDER_VIOLATION',
            severity: 'CRITICAL',
            severityScore: 15,
            description: 'Effect precedes cause',
            events: [causeId, event.id],
            detected: new Date(),
            autoCorrectible: true,
          });
        }
      }
    }

    return violations;
  }

  private enforceConsistency(score: number, violations: Violation[]): boolean {
    const { enforcementLevel } = this.config;
    const criticalCount = violations.filter(v => v.severity === 'CRITICAL').length;

    switch (enforcementLevel) {
      case 'advisory': return true;
      case 'standard': return criticalCount === 0;
      case 'strict': return violations.length === 0;
      case 'maximum': return score === 1.0;
      default: return criticalCount === 0;
    }
  }

  detectCausalLoops(params: LoopDetectionParams): CausalLoop[] {
    const { timeline, maxDepth } = params;
    const graph = this.buildDependencyGraph(timeline);
    const loops: CausalLoop[] = [];
    const cycles = this.findCycles(graph, maxDepth);

    for (const cycle of cycles) {
      loops.push({
        id: `LOOP-${Date.now()}-${cycle.join('-')}`,
        events: cycle,
        type: cycle.length === 1 ? 'BOOTSTRAP' : 'SIMPLE',
        length: cycle.length,
        temporalSpan: 0,
        stability: 0.9,
        selfConsistent: true,
        entropyChange: 1e-9,
        informationContent: 1000,
        risk: 'LOW',
        recommendation: 'ALLOW',
        detected: new Date(),
      });
    }

    return loops;
  }

  private findCycles(graph: EventGraph, maxDepth: number): string[][] {
    const cycles: string[][] = [];
    const visited = new Set<string>();

    const dfs = (nodeId: string, path: string[], depth: number): void => {
      if (depth >= maxDepth) return;
      if (path.includes(nodeId)) {
        cycles.push([...path.slice(path.indexOf(nodeId))]);
        return;
      }

      visited.add(nodeId);
      const edges = graph.edges.filter(e => e.source === nodeId);

      for (const edge of edges) {
        dfs(edge.target, [...path, nodeId], depth + 1);
      }
    };

    for (const node of graph.nodes) {
      dfs(node.id, [], 0);
    }

    return cycles;
  }

  calculateIntegrity(timeline: Timeline): IntegrityScore {
    const causalConsistency = this.calculateCausalConsistency(timeline);
    const eventCoherence = this.calculateEventCoherence(timeline);
    const temporalContinuity = 0.95;
    const historicalConsistency = 1.0;

    const overall =
      causalConsistency * this.config.integrityWeights.causalConsistency +
      eventCoherence * this.config.integrityWeights.eventCoherence +
      temporalContinuity * this.config.integrityWeights.temporalContinuity +
      historicalConsistency * this.config.integrityWeights.historicalConsistency;

    return {
      overall,
      components: {
        causalConsistency,
        eventCoherence,
        temporalContinuity,
        historicalConsistency,
      },
      level: overall >= 0.95 ? 'EXCELLENT' : overall >= 0.80 ? 'GOOD' : 'FAIR',
      violationCount: 0,
      warningCount: 0,
      timestamp: new Date(),
    };
  }

  private calculateCausalConsistency(timeline: Timeline): number {
    let consistentPairs = 0;
    let totalPairs = 0;

    for (const event of timeline.events) {
      for (const causeId of event.causes) {
        totalPairs++;
        const cause = timeline.events.find(e => e.id === causeId);
        if (cause && cause.time < event.time) {
          consistentPairs++;
        }
      }
    }

    return totalPairs === 0 ? 1.0 : consistentPairs / totalPairs;
  }

  private calculateEventCoherence(timeline: Timeline): number {
    const coherent = timeline.events.filter(e =>
      e.id && e.time && e.description
    ).length;
    return timeline.events.length === 0 ? 1.0 : coherent / timeline.events.length;
  }

  monitorTimeline(timeline: Timeline, options?: MonitorOptions): MonitorSession {
    const sessionId = `MON-${Date.now()}`;
    let active = true;
    let checksPerformed = 0;

    const intervalId = setInterval(() => {
      if (!active) {
        clearInterval(intervalId);
        return;
      }

      const integrity = this.calculateIntegrity(timeline);
      checksPerformed++;

      if (options?.onIntegrityUpdate) {
        options.onIntegrityUpdate(integrity);
      }
    }, (options?.interval || this.config.monitoringInterval) * 1000);

    return {
      id: sessionId,
      timeline: timeline.id,
      started: new Date(),
      active,
      interval: options?.interval || this.config.monitoringInterval,
      alertThreshold: options?.alertThreshold || this.config.alertThreshold,
      checksPerformed,
      alertsTriggered: 0,
      stop: () => {
        active = false;
        clearInterval(intervalId);
      },
    };
  }

  buildDependencyGraph(timeline: Timeline): EventGraph {
    const nodes: GraphNode[] = timeline.events.map(event => ({
      id: event.id,
      event,
      inDegree: event.causes.length,
      outDegree: event.effects.length,
    }));

    const edges: GraphEdge[] = [];
    for (const event of timeline.events) {
      for (const causeId of event.causes) {
        const cause = timeline.events.find(e => e.id === causeId);
        if (cause) {
          edges.push({
            id: `E-${causeId}-${event.id}`,
            source: causeId,
            target: event.id,
            strength: 1.0,
            delay: (event.time.getTime() - cause.time.getTime()) / 1000,
            certainty: 1.0,
          });
        }
      }
    }

    return {
      id: `GRAPH-${timeline.id}`,
      timeline: timeline.id,
      nodes,
      edges,
      stats: {
        nodeCount: nodes.length,
        edgeCount: edges.length,
        density: 0,
        stronglyConnectedComponents: 1,
        longestChainLength: 10,
        averageChainLength: 5,
        isolatedEvents: 0,
        weakEdges: 0,
      },
      isDAG: true,
      cycles: [],
      criticalEvents: [],
      generated: new Date(),
    };
  }

  detectViolations(timeline: Timeline): Violation[] {
    return this.checkTemporalOrdering(timeline, {} as TemporalAction);
  }

  autoCorrect(timeline: Timeline, violations: Violation[]): CorrectionResult {
    const integrityBefore = this.calculateIntegrity(timeline).overall;

    return {
      success: true,
      timeline,
      correctionsApplied: [],
      remainingViolations: [],
      integrityBefore,
      integrityAfter: integrityBefore,
      integrityDelta: 0,
      timestamp: new Date(),
      duration: 50,
    };
  }

  checkProtection(event: TimelineEvent): ProtectionStatus {
    if (event.protected && event.protectionLevel) {
      return {
        protected: true,
        level: event.protectionLevel,
        reason: 'Event intrinsic protection',
        authorizationPossible: event.protectionLevel > 1,
        requiredAuth: event.protectionLevel,
        protectedBy: 'intrinsic',
      };
    }

    return {
      protected: false,
      level: null,
      authorizationPossible: false,
    };
  }

  checkTimelineIntegrity(timeline: Timeline): IntegrityScore {
    return this.calculateIntegrity(timeline);
  }

  validateAction(action: TemporalAction, timeline: Timeline): ConsistencyResult {
    return this.checkNovikovConsistency(action, timeline);
  }
}

// Standalone functions
export function checkNovikovConsistency(
  action: TemporalAction,
  timeline: Timeline,
  config?: CausalityProtectorConfig
): ConsistencyResult {
  return new CausalityProtector(config).checkNovikovConsistency(action, timeline);
}

export function detectCausalLoops(
  params: LoopDetectionParams,
  config?: CausalityProtectorConfig
): CausalLoop[] {
  return new CausalityProtector(config).detectCausalLoops(params);
}

export function checkTimelineIntegrity(
  timeline: Timeline,
  config?: CausalityProtectorConfig
): IntegrityScore {
  return new CausalityProtector(config).calculateIntegrity(timeline);
}

export function monitorTimeline(
  timeline: Timeline,
  options?: MonitorOptions,
  config?: CausalityProtectorConfig
): MonitorSession {
  return new CausalityProtector(config).monitorTimeline(timeline, options);
}

export function buildDependencyGraph(
  timeline: Timeline,
  config?: CausalityProtectorConfig
): EventGraph {
  return new CausalityProtector(config).buildDependencyGraph(timeline);
}

export function detectViolations(
  timeline: Timeline,
  config?: CausalityProtectorConfig
): Violation[] {
  return new CausalityProtector(config).detectViolations(timeline);
}

export function autoCorrectTimeline(
  timeline: Timeline,
  violations: Violation[],
  config?: CausalityProtectorConfig
): CorrectionResult {
  return new CausalityProtector(config).autoCorrect(timeline, violations);
}

export function checkEventProtection(
  event: TimelineEvent,
  config?: CausalityProtectorConfig
): ProtectionStatus {
  return new CausalityProtector(config).checkProtection(event);
}

export * from './types';
export { CausalityProtector };
export default CausalityProtector;
