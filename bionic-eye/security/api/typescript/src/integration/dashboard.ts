/**
 * WIA Security Dashboard Integration
 * Metrics, aggregation, and visualization support
 */

import { WiaSecurityEvent, EventType } from '../types';

// ============================================================================
// Types
// ============================================================================

export interface DashboardMetrics {
  totalEvents: number;
  eventsByType: Record<string, number>;
  eventsBySeverity: SeverityDistribution;
  eventsBySource: Record<string, number>;
  timeSeriesData: TimeSeriesPoint[];
  topMitreTactics: TacticCount[];
  topMitreTechniques: TechniqueCount[];
  meanTimeToDetect?: number;
  meanTimeToRespond?: number;
}

export interface SeverityDistribution {
  critical: number;
  high: number;
  medium: number;
  low: number;
  informational: number;
}

export interface TimeSeriesPoint {
  timestamp: string;
  count: number;
  severity?: number;
}

export interface TacticCount {
  tactic: string;
  tacticId: string;
  count: number;
}

export interface TechniqueCount {
  technique: string;
  techniqueId: string;
  tactic: string;
  count: number;
}

export interface AlertStats {
  total: number;
  open: number;
  acknowledged: number;
  resolved: number;
  falsePositives: number;
}

export interface ThreatLandscape {
  activeThreats: number;
  newIndicators: number;
  compromisedAssets: number;
  topThreatActors: string[];
  topMalwareFamilies: string[];
}

export interface ComplianceStatus {
  framework: string;
  controls: ControlStatus[];
  overallScore: number;
  lastAssessment: string;
}

export interface ControlStatus {
  controlId: string;
  name: string;
  status: 'compliant' | 'non_compliant' | 'partial' | 'not_assessed';
  score: number;
  findings: number;
}

// ============================================================================
// Dashboard Aggregator
// ============================================================================

export class DashboardAggregator {
  private events: WiaSecurityEvent[] = [];
  private timeWindow: number; // milliseconds

  constructor(options?: { timeWindowHours?: number }) {
    this.timeWindow = (options?.timeWindowHours || 24) * 60 * 60 * 1000;
  }

  /**
   * Add event to aggregator
   */
  addEvent(event: WiaSecurityEvent): void {
    this.events.push(event);
    this.pruneOldEvents();
  }

  /**
   * Add multiple events
   */
  addEvents(events: WiaSecurityEvent[]): void {
    this.events.push(...events);
    this.pruneOldEvents();
  }

  /**
   * Get dashboard metrics
   */
  getMetrics(): DashboardMetrics {
    const now = Date.now();
    const relevantEvents = this.events.filter(e =>
      new Date(e.timestamp).getTime() > now - this.timeWindow
    );

    return {
      totalEvents: relevantEvents.length,
      eventsByType: this.countByType(relevantEvents),
      eventsBySeverity: this.countBySeverity(relevantEvents),
      eventsBySource: this.countBySource(relevantEvents),
      timeSeriesData: this.buildTimeSeries(relevantEvents),
      topMitreTactics: this.getTopTactics(relevantEvents),
      topMitreTechniques: this.getTopTechniques(relevantEvents),
      meanTimeToDetect: this.calculateMTTD(relevantEvents),
      meanTimeToRespond: this.calculateMTTR(relevantEvents)
    };
  }

  /**
   * Get alert statistics
   */
  getAlertStats(): AlertStats {
    const alerts = this.events.filter(e => e.type === 'alert');

    return {
      total: alerts.length,
      open: alerts.filter(a => (a as any).status === 'open').length,
      acknowledged: alerts.filter(a => (a as any).status === 'acknowledged').length,
      resolved: alerts.filter(a => (a as any).status === 'resolved').length,
      falsePositives: alerts.filter(a => (a as any).status === 'false_positive').length
    };
  }

  /**
   * Get threat landscape
   */
  getThreatLandscape(): ThreatLandscape {
    const threatIntel = this.events.filter(e => e.type === 'threat_intel');
    const incidents = this.events.filter(e => e.type === 'incident');

    const threatActors = new Set<string>();
    const malwareFamilies = new Set<string>();

    threatIntel.forEach(e => {
      const ti = e as any;
      if (ti.threatActor) threatActors.add(ti.threatActor);
      if (ti.malwareFamily) malwareFamilies.add(ti.malwareFamily);
    });

    return {
      activeThreats: threatIntel.filter(t => {
        const ti = t as any;
        return !ti.validUntil || new Date(ti.validUntil) > new Date();
      }).length,
      newIndicators: threatIntel.filter(t => {
        const oneDay = 24 * 60 * 60 * 1000;
        return Date.now() - new Date(t.timestamp).getTime() < oneDay;
      }).length,
      compromisedAssets: new Set(
        incidents.map(i => (i as any).affectedAssets || []).flat()
      ).size,
      topThreatActors: Array.from(threatActors).slice(0, 5),
      topMalwareFamilies: Array.from(malwareFamilies).slice(0, 5)
    };
  }

  /**
   * Get severity trend
   */
  getSeverityTrend(intervals: number = 24): TimeSeriesPoint[] {
    const now = Date.now();
    const intervalMs = this.timeWindow / intervals;
    const points: TimeSeriesPoint[] = [];

    for (let i = 0; i < intervals; i++) {
      const start = now - (intervals - i) * intervalMs;
      const end = start + intervalMs;

      const intervalEvents = this.events.filter(e => {
        const eventTime = new Date(e.timestamp).getTime();
        return eventTime >= start && eventTime < end;
      });

      const avgSeverity = intervalEvents.length > 0
        ? intervalEvents.reduce((sum, e) => sum + e.severity, 0) / intervalEvents.length
        : 0;

      points.push({
        timestamp: new Date(start).toISOString(),
        count: intervalEvents.length,
        severity: avgSeverity
      });
    }

    return points;
  }

  /**
   * Export data for visualization
   */
  exportForVisualization(): {
    events: WiaSecurityEvent[];
    metrics: DashboardMetrics;
    alertStats: AlertStats;
    threatLandscape: ThreatLandscape;
    severityTrend: TimeSeriesPoint[];
  } {
    return {
      events: [...this.events],
      metrics: this.getMetrics(),
      alertStats: this.getAlertStats(),
      threatLandscape: this.getThreatLandscape(),
      severityTrend: this.getSeverityTrend()
    };
  }

  /**
   * Clear all events
   */
  clear(): void {
    this.events = [];
  }

  // -------------------------------------------------------------------------
  // Private Methods
  // -------------------------------------------------------------------------

  private pruneOldEvents(): void {
    const cutoff = Date.now() - this.timeWindow;
    this.events = this.events.filter(e =>
      new Date(e.timestamp).getTime() > cutoff
    );
  }

  private countByType(events: WiaSecurityEvent[]): Record<string, number> {
    const counts: Record<string, number> = {};
    events.forEach(e => {
      counts[e.type] = (counts[e.type] || 0) + 1;
    });
    return counts;
  }

  private countBySeverity(events: WiaSecurityEvent[]): SeverityDistribution {
    const dist: SeverityDistribution = {
      critical: 0,
      high: 0,
      medium: 0,
      low: 0,
      informational: 0
    };

    events.forEach(e => {
      if (e.severity >= 9) dist.critical++;
      else if (e.severity >= 7) dist.high++;
      else if (e.severity >= 4) dist.medium++;
      else if (e.severity >= 1) dist.low++;
      else dist.informational++;
    });

    return dist;
  }

  private countBySource(events: WiaSecurityEvent[]): Record<string, number> {
    const counts: Record<string, number> = {};
    events.forEach(e => {
      const source = e.source?.name || 'unknown';
      counts[source] = (counts[source] || 0) + 1;
    });
    return counts;
  }

  private buildTimeSeries(events: WiaSecurityEvent[], intervals: number = 24): TimeSeriesPoint[] {
    const now = Date.now();
    const intervalMs = this.timeWindow / intervals;
    const points: TimeSeriesPoint[] = [];

    for (let i = 0; i < intervals; i++) {
      const start = now - (intervals - i) * intervalMs;
      const end = start + intervalMs;

      const count = events.filter(e => {
        const eventTime = new Date(e.timestamp).getTime();
        return eventTime >= start && eventTime < end;
      }).length;

      points.push({
        timestamp: new Date(start).toISOString(),
        count
      });
    }

    return points;
  }

  private getTopTactics(events: WiaSecurityEvent[], limit: number = 10): TacticCount[] {
    const counts = new Map<string, { name: string; count: number }>();

    events.forEach(e => {
      if (e.mitre?.tactic) {
        const key = e.mitre.tactic;
        const existing = counts.get(key);
        if (existing) {
          existing.count++;
        } else {
          counts.set(key, {
            name: this.formatTacticName(e.mitre.tactic),
            count: 1
          });
        }
      }
    });

    return Array.from(counts.entries())
      .map(([id, data]) => ({
        tacticId: id,
        tactic: data.name,
        count: data.count
      }))
      .sort((a, b) => b.count - a.count)
      .slice(0, limit);
  }

  private getTopTechniques(events: WiaSecurityEvent[], limit: number = 10): TechniqueCount[] {
    const counts = new Map<string, { name: string; tactic: string; count: number }>();

    events.forEach(e => {
      if (e.mitre?.technique) {
        const key = e.mitre.technique;
        const existing = counts.get(key);
        if (existing) {
          existing.count++;
        } else {
          counts.set(key, {
            name: this.formatTechniqueName(e.mitre.technique),
            tactic: e.mitre.tactic || 'unknown',
            count: 1
          });
        }
      }
    });

    return Array.from(counts.entries())
      .map(([id, data]) => ({
        techniqueId: id,
        technique: data.name,
        tactic: data.tactic,
        count: data.count
      }))
      .sort((a, b) => b.count - a.count)
      .slice(0, limit);
  }

  private calculateMTTD(events: WiaSecurityEvent[]): number | undefined {
    // Mean Time to Detect - difference between event occurrence and detection
    const detectionTimes: number[] = [];

    events.forEach(e => {
      if (e.meta?.detectedAt && e.meta?.occurredAt) {
        const detected = new Date(e.meta.detectedAt).getTime();
        const occurred = new Date(e.meta.occurredAt).getTime();
        if (detected > occurred) {
          detectionTimes.push((detected - occurred) / 1000 / 60); // minutes
        }
      }
    });

    if (detectionTimes.length === 0) return undefined;
    return detectionTimes.reduce((a, b) => a + b, 0) / detectionTimes.length;
  }

  private calculateMTTR(events: WiaSecurityEvent[]): number | undefined {
    // Mean Time to Respond - difference between detection and resolution
    const responseTimes: number[] = [];

    events.forEach(e => {
      const meta = e.meta as any;
      if (meta?.detectedAt && meta?.resolvedAt) {
        const detected = new Date(meta.detectedAt).getTime();
        const resolved = new Date(meta.resolvedAt).getTime();
        if (resolved > detected) {
          responseTimes.push((resolved - detected) / 1000 / 60); // minutes
        }
      }
    });

    if (responseTimes.length === 0) return undefined;
    return responseTimes.reduce((a, b) => a + b, 0) / responseTimes.length;
  }

  private formatTacticName(tacticId: string): string {
    // TA0001 -> Initial Access
    const tactics: Record<string, string> = {
      'TA0001': 'Initial Access',
      'TA0002': 'Execution',
      'TA0003': 'Persistence',
      'TA0004': 'Privilege Escalation',
      'TA0005': 'Defense Evasion',
      'TA0006': 'Credential Access',
      'TA0007': 'Discovery',
      'TA0008': 'Lateral Movement',
      'TA0009': 'Collection',
      'TA0010': 'Exfiltration',
      'TA0011': 'Command and Control',
      'TA0040': 'Impact',
      'TA0042': 'Resource Development',
      'TA0043': 'Reconnaissance'
    };
    return tactics[tacticId] || tacticId;
  }

  private formatTechniqueName(techniqueId: string): string {
    // T1059 -> Command and Scripting Interpreter
    // This would be a large mapping - simplified here
    return techniqueId;
  }
}

// ============================================================================
// Compliance Tracker
// ============================================================================

export class ComplianceTracker {
  private frameworks: Map<string, ComplianceStatus> = new Map();

  /**
   * Add or update framework status
   */
  updateFramework(status: ComplianceStatus): void {
    this.frameworks.set(status.framework, status);
  }

  /**
   * Get framework status
   */
  getFramework(framework: string): ComplianceStatus | undefined {
    return this.frameworks.get(framework);
  }

  /**
   * Get all frameworks
   */
  getAllFrameworks(): ComplianceStatus[] {
    return Array.from(this.frameworks.values());
  }

  /**
   * Map WIA Security event to compliance control
   */
  mapEventToControls(event: WiaSecurityEvent, framework: string): string[] {
    // Maps events to relevant compliance controls
    const mappings: Record<string, Record<string, string[]>> = {
      'NIST': {
        'auth_event': ['AC-2', 'AC-7', 'IA-5'],
        'network_event': ['SC-7', 'SI-4', 'AU-12'],
        'endpoint_event': ['CM-7', 'SI-3', 'SI-4'],
        'vulnerability': ['RA-5', 'SI-2', 'CM-8'],
        'incident': ['IR-4', 'IR-5', 'IR-6'],
        'threat_intel': ['RA-3', 'SI-5', 'PM-16'],
        'alert': ['SI-4', 'AU-6', 'IR-4']
      },
      'CIS': {
        'auth_event': ['CIS-4.1', 'CIS-4.2', 'CIS-16.1'],
        'network_event': ['CIS-12.1', 'CIS-13.1', 'CIS-9.1'],
        'endpoint_event': ['CIS-2.1', 'CIS-8.1', 'CIS-10.1'],
        'vulnerability': ['CIS-3.1', 'CIS-7.1', 'CIS-3.4'],
        'incident': ['CIS-19.1', 'CIS-19.2', 'CIS-19.3'],
        'threat_intel': ['CIS-17.1', 'CIS-17.2'],
        'alert': ['CIS-6.1', 'CIS-6.2', 'CIS-19.1']
      },
      'ISO27001': {
        'auth_event': ['A.9.2', 'A.9.4', 'A.12.4'],
        'network_event': ['A.13.1', 'A.12.4', 'A.12.6'],
        'endpoint_event': ['A.12.2', 'A.12.5', 'A.8.3'],
        'vulnerability': ['A.12.6', 'A.14.2', 'A.18.2'],
        'incident': ['A.16.1', 'A.16.2', 'A.16.3'],
        'threat_intel': ['A.6.1', 'A.18.1'],
        'alert': ['A.12.4', 'A.16.1']
      }
    };

    return mappings[framework]?.[event.type] || [];
  }

  /**
   * Calculate compliance score from events
   */
  calculateScoreFromEvents(
    events: WiaSecurityEvent[],
    framework: string
  ): number {
    // Simple scoring based on event severity and control coverage
    const controlsWithFindings = new Set<string>();
    const allControls = new Set<string>();

    events.forEach(e => {
      const controls = this.mapEventToControls(e, framework);
      controls.forEach(c => {
        allControls.add(c);
        if (e.severity >= 7) {
          controlsWithFindings.add(c);
        }
      });
    });

    if (allControls.size === 0) return 100;

    const affectedRatio = controlsWithFindings.size / allControls.size;
    return Math.round((1 - affectedRatio) * 100);
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

export function createDashboardAggregator(options?: {
  timeWindowHours?: number;
}): DashboardAggregator {
  return new DashboardAggregator(options);
}

export function createComplianceTracker(): ComplianceTracker {
  return new ComplianceTracker();
}
