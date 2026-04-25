/**
 * WIA-AI-018: AI Decision Audit SDK
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 *
 * TypeScript SDK for implementing AI decision audit systems according to
 * the WIA-AI-018 standard.
 *
 * @version 1.0.0
 * @license MIT
 */

import * as crypto from 'crypto';
import {
  DecisionAuditLog,
  ComplianceRule,
  ComplianceReport,
  RiskScore,
  BiasAnalysis,
  DriftAnalysis,
  AnomalyDetection,
  AuditQuery,
  AuditStatistics,
  AuditLoggerConfig,
  HashChainVerification,
  HealthCheckResult,
  DashboardMetrics
} from './types';

export * from './types';

/**
 * Main audit logger class
 */
export class AuditLogger {
  private config: AuditLoggerConfig;
  private latestHash: string = '0'.repeat(64);
  private buffer: DecisionAuditLog[] = [];
  private flushTimer?: NodeJS.Timeout;

  constructor(config: Partial<AuditLoggerConfig> = {}) {
    this.config = {
      async_logging: true,
      batch_size: 100,
      flush_interval_ms: 5000,
      enable_encryption: true,
      enable_hash_chaining: true,
      storage_backend: 'postgresql',
      ...config
    };

    if (this.config.async_logging) {
      this.startFlushTimer();
    }
  }

  /**
   * Log an AI decision
   */
  async log(decision: Partial<DecisionAuditLog>): Promise<void> {
    const log = this.prepareLog(decision);

    if (this.config.async_logging) {
      this.buffer.push(log);

      if (this.buffer.length >= this.config.batch_size) {
        await this.flush();
      }
    } else {
      await this.persistLog(log);
    }
  }

  /**
   * Prepare log entry with required metadata
   */
  private prepareLog(decision: Partial<DecisionAuditLog>): DecisionAuditLog {
    const log: DecisionAuditLog = {
      decision_id: decision.decision_id || this.generateId(),
      timestamp: decision.timestamp || new Date().toISOString(),
      timezone: decision.timezone || Intl.DateTimeFormat().resolvedOptions().timeZone,
      processing_duration_ms: decision.processing_duration_ms || 0,
      system: decision.system!,
      model: decision.model!,
      input: decision.input!,
      output: decision.output!,
      reasoning: decision.reasoning,
      context: decision.context!,
      compliance: decision.compliance!,
      audit: {
        log_version: '1.0',
        integrity_hash: '',
        previous_hash: this.config.enable_hash_chaining ? this.latestHash : undefined
      }
    };

    // Calculate integrity hash
    if (this.config.enable_hash_chaining) {
      log.audit.integrity_hash = this.calculateHash(log);
      this.latestHash = log.audit.integrity_hash;
    }

    return log;
  }

  /**
   * Calculate cryptographic hash for integrity verification
   */
  private calculateHash(log: DecisionAuditLog): string {
    const content = JSON.stringify({
      decision_id: log.decision_id,
      timestamp: log.timestamp,
      model: log.model,
      input: log.input,
      output: log.output,
      context: log.context,
      previous_hash: log.audit.previous_hash || '0'
    });

    return crypto.createHash('sha256').update(content).digest('hex');
  }

  /**
   * Generate unique ID
   */
  private generateId(): string {
    return crypto.randomUUID();
  }

  /**
   * Flush buffered logs to storage
   */
  async flush(): Promise<void> {
    if (this.buffer.length === 0) return;

    const logs = this.buffer.splice(0);
    await this.persistBatch(logs);
  }

  /**
   * Persist single log entry
   */
  private async persistLog(log: DecisionAuditLog): Promise<void> {
    // Implementation depends on storage backend
    // This is a placeholder
    console.log('Persisting log:', log.decision_id);
  }

  /**
   * Persist batch of log entries
   */
  private async persistBatch(logs: DecisionAuditLog[]): Promise<void> {
    // Implementation depends on storage backend
    // This is a placeholder
    console.log('Persisting batch:', logs.length, 'logs');
  }

  /**
   * Start automatic flush timer
   */
  private startFlushTimer(): void {
    this.flushTimer = setInterval(
      () => this.flush(),
      this.config.flush_interval_ms
    );
  }

  /**
   * Stop flush timer and flush remaining logs
   */
  async shutdown(): Promise<void> {
    if (this.flushTimer) {
      clearInterval(this.flushTimer);
    }
    await this.flush();
  }

  /**
   * Query audit logs
   */
  async query(params: AuditQuery): Promise<DecisionAuditLog[]> {
    // Implementation depends on storage backend
    throw new Error('Query method must be implemented');
  }

  /**
   * Get audit statistics
   */
  async getStatistics(dateRange?: { start: string; end: string }): Promise<AuditStatistics> {
    // Implementation depends on storage backend
    throw new Error('getStatistics method must be implemented');
  }

  /**
   * Verify hash chain integrity
   */
  async verifyHashChain(logs: DecisionAuditLog[]): Promise<HashChainVerification> {
    let expectedHash = '0'.repeat(64);
    let verified = 0;
    let firstInvalidIndex: number | undefined;

    for (let i = 0; i < logs.length; i++) {
      const log = logs[i];

      // Verify previous hash matches
      if (log.audit.previous_hash !== expectedHash) {
        if (firstInvalidIndex === undefined) {
          firstInvalidIndex = i;
        }
        continue;
      }

      // Recalculate hash
      const calculatedHash = this.calculateHash(log);

      // Verify hash matches
      if (log.audit.integrity_hash !== calculatedHash) {
        if (firstInvalidIndex === undefined) {
          firstInvalidIndex = i;
        }
        continue;
      }

      verified++;
      expectedHash = log.audit.integrity_hash;
    }

    return {
      valid: verified === logs.length,
      verified_count: verified,
      total_count: logs.length,
      first_invalid_index: firstInvalidIndex,
      integrity_score: (verified / logs.length) * 100
    };
  }

  /**
   * Health check
   */
  async healthCheck(): Promise<HealthCheckResult> {
    const checks = [];

    // Database connectivity
    try {
      // Placeholder - implement based on storage backend
      checks.push({
        name: 'database',
        status: 'pass' as const,
        latency_ms: 10
      });
    } catch (error) {
      checks.push({
        name: 'database',
        status: 'fail' as const,
        message: error instanceof Error ? error.message : 'Unknown error'
      });
    }

    // Storage backend
    try {
      checks.push({
        name: 'storage',
        status: 'pass' as const,
        latency_ms: 5
      });
    } catch (error) {
      checks.push({
        name: 'storage',
        status: 'fail' as const,
        message: error instanceof Error ? error.message : 'Unknown error'
      });
    }

    const allPassed = checks.every(c => c.status === 'pass');

    return {
      status: allPassed ? 'healthy' : 'degraded',
      checks,
      timestamp: new Date().toISOString()
    };
  }
}

/**
 * Compliance rule engine
 */
export class ComplianceRuleEngine {
  private rules: Map<string, ComplianceRule> = new Map();

  /**
   * Register a compliance rule
   */
  registerRule(rule: ComplianceRule): void {
    this.rules.set(rule.id, rule);
  }

  /**
   * Unregister a compliance rule
   */
  unregisterRule(ruleId: string): void {
    this.rules.delete(ruleId);
  }

  /**
   * Check decision compliance
   */
  async checkCompliance(log: DecisionAuditLog): Promise<ComplianceReport> {
    const results = [];
    const violations = [];

    for (const rule of this.rules.values()) {
      const result = rule.check(log);
      results.push(result);

      if (!result.compliant) {
        violations.push({
          rule_id: rule.id,
          rule_name: rule.name,
          severity: rule.severity,
          message: result.message,
          evidence: result.evidence,
          timestamp: new Date().toISOString()
        });

        // Attempt automated remediation
        if (rule.remediation) {
          await rule.remediation(log);
        }
      }
    }

    return {
      decision_id: log.decision_id,
      overall_compliant: violations.length === 0,
      violations,
      results,
      checked_at: new Date().toISOString()
    };
  }

  /**
   * Get all registered rules
   */
  getRules(): ComplianceRule[] {
    return Array.from(this.rules.values());
  }
}

/**
 * Risk assessment engine
 */
export class RiskAssessmentEngine {
  /**
   * Assess decision risk
   */
  async assessRisk(log: DecisionAuditLog): Promise<RiskScore> {
    const dimensions = {
      confidence_risk: this.assessConfidenceRisk(log),
      bias_risk: 0,  // Requires historical analysis
      data_quality_risk: this.assessDataQualityRisk(log),
      drift_risk: 0,  // Requires historical analysis
      impact_risk: this.assessImpactRisk(log),
      compliance_risk: 0,  // Requires compliance check
      fairness_risk: 0,  // Requires demographic analysis
      explainability_risk: this.assessExplainabilityRisk(log)
    };

    const overall = this.calculateOverallRisk(dimensions);
    const flags = this.identifyRiskFlags(log, dimensions);
    const severity = this.determineSeverity(overall);

    return { overall, dimensions, flags, severity };
  }

  private assessConfidenceRisk(log: DecisionAuditLog): number {
    const confidence = log.output.confidence;

    if (confidence >= 0.95) return 5;
    if (confidence >= 0.85) return 20;
    if (confidence >= 0.70) return 50;
    if (confidence >= 0.60) return 75;
    return 95;
  }

  private assessDataQualityRisk(log: DecisionAuditLog): number {
    // Placeholder implementation
    return 10;
  }

  private assessImpactRisk(log: DecisionAuditLog): number {
    const baseRisk = {
      critical: 90,
      high: 70,
      medium: 40,
      low: 15
    }[log.context.business_impact];

    return baseRisk;
  }

  private assessExplainabilityRisk(log: DecisionAuditLog): number {
    if (!log.reasoning || !log.reasoning.explanation) {
      return 80;
    }

    if (log.reasoning.explanation.length < 50) {
      return 60;
    }

    if (!log.reasoning.key_features || log.reasoning.key_features.length === 0) {
      return 50;
    }

    return 10;
  }

  private calculateOverallRisk(dimensions: Record<string, number>): number {
    const weights = {
      confidence_risk: 0.15,
      bias_risk: 0.20,
      data_quality_risk: 0.10,
      drift_risk: 0.10,
      impact_risk: 0.15,
      compliance_risk: 0.15,
      fairness_risk: 0.10,
      explainability_risk: 0.05
    };

    let score = 0;
    for (const [key, weight] of Object.entries(weights)) {
      score += dimensions[key] * weight;
    }

    return Math.round(score);
  }

  private identifyRiskFlags(log: DecisionAuditLog, dimensions: Record<string, number>): string[] {
    const flags: string[] = [];

    if (dimensions.confidence_risk > 70) flags.push('low_confidence');
    if (dimensions.impact_risk > 70) flags.push('high_impact');
    if (dimensions.explainability_risk > 60) flags.push('insufficient_explanation');

    return flags;
  }

  private determineSeverity(score: number): 'low' | 'medium' | 'high' | 'critical' {
    if (score >= 80) return 'critical';
    if (score >= 60) return 'high';
    if (score >= 40) return 'medium';
    return 'low';
  }
}

/**
 * Main SDK export
 */
export const WIA_AI_018 = {
  AuditLogger,
  ComplianceRuleEngine,
  RiskAssessmentEngine,
  version: '1.0.0'
};

export default WIA_AI_018;
