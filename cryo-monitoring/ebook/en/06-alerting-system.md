# Chapter 6: Alerting System
## Intelligent Alert Management and Escalation

**弘益人間 (Hongik Ingan)** - Protecting samples through intelligent alerts

---

## 1. Introduction to Alert Management

The WIA Cryo Monitoring alerting system provides comprehensive, intelligent alert generation, management, escalation, and notification. This system ensures that critical conditions are detected immediately and the right people are notified through the right channels.

### 1.1 Alert Architecture

```typescript
/**
 * WIA Cryo Monitoring Alert System
 *
 * Comprehensive alerting with rules engine, escalation, and
 * multi-channel notification
 */

import { z } from 'zod';

/**
 * Alert Rule Engine
 *
 * Flexible rule system for defining alert conditions
 */
export interface AlertRuleEngine {
  ruleEngineId: string;
  facilityId: string;
  enabled: boolean;

  rules: AlertRule[];
  globalSettings: GlobalAlertSettings;
  ruleGroups: AlertRuleGroup[];
}

export interface AlertRule {
  ruleId: string;
  name: string;
  description: string;
  enabled: boolean;
  priority: number; // Higher number = higher priority

  // Trigger conditions
  trigger: {
    type: 'threshold' | 'rate-of-change' | 'anomaly' | 'pattern' | 'composite';

    // Threshold-based triggers
    threshold?: {
      parameter: string;
      operator: 'gt' | 'lt' | 'eq' | 'ne' | 'gte' | 'lte' | 'between';
      value?: number;
      min?: number;
      max?: number;
      unit: string;
    };

    // Rate of change triggers
    rateOfChange?: {
      parameter: string;
      maxRate: number; // units per second
      timeWindow: number; // seconds
      direction?: 'increasing' | 'decreasing' | 'any';
    };

    // Anomaly detection triggers
    anomaly?: {
      method: 'zscore' | 'iqr' | 'isolation-forest' | 'ml-model';
      sensitivity: 'low' | 'medium' | 'high';
      trainingWindow: number; // days
      threshold: number; // anomaly score threshold
    };

    // Pattern matching triggers
    pattern?: {
      type: 'sequence' | 'frequency' | 'correlation';
      pattern: string;
      timeWindow: number; // seconds
      matchCount: number;
    };

    // Composite triggers (combine multiple conditions)
    composite?: {
      logic: 'AND' | 'OR' | 'NOT';
      conditions: AlertRule['trigger'][];
    };
  };

  // Scope (what to monitor)
  scope: {
    equipmentIds?: string[];
    sensorIds?: string[];
    facilityIds?: string[];
    equipmentTypes?: string[];
    sensorTypes?: string[];
    tags?: string[];
    allEquipment?: boolean;
  };

  // Conditions
  conditions: {
    // Time-based conditions
    timeWindow?: {
      start: string; // HH:mm
      end: string; // HH:mm
      timezone: string;
      daysOfWeek?: number[]; // 0=Sunday, 6=Saturday
    };

    // Duration requirement
    duration?: {
      minimum: number; // seconds - condition must persist
      continuous: boolean; // true = continuous, false = cumulative
    };

    // Suppression during maintenance
    suppressDuringMaintenance: boolean;
  };

  // Alert severity
  severity: 'critical' | 'high' | 'medium' | 'low' | 'info';

  // Notification configuration
  notification: {
    channels: string[]; // channel IDs
    template?: string;
    customMessage?: string;
    includeData: boolean;
    includeHistory: boolean;
    attachments?: string[];
  };

  // Actions to execute
  actions: AlertActionConfig[];

  // Escalation
  escalation?: {
    enabled: boolean;
    escalationPolicyId: string;
  };

  // Auto-resolution
  autoResolve?: {
    enabled: boolean;
    conditions: {
      returnToNormal: boolean;
      duration?: number; // seconds
      requireAcknowledgment: boolean;
    };
  };

  // Metadata
  metadata: {
    created: Date;
    createdBy: string;
    modified: Date;
    modifiedBy: string;
    tags: string[];
    category: string;
  };
}

export interface AlertActionConfig {
  actionId: string;
  actionType: 'notification' | 'webhook' | 'script' | 'api-call' | 'emergency-protocol';
  enabled: boolean;
  priority: number;

  config: {
    // Notification action
    notificationChannelId?: string;
    recipients?: string[];

    // Webhook action
    webhookUrl?: string;
    method?: 'GET' | 'POST' | 'PUT';
    headers?: Record<string, string>;
    body?: string;

    // Script action
    scriptPath?: string;
    arguments?: string[];
    timeout?: number;

    // API call action
    apiEndpoint?: string;
    apiMethod?: string;
    apiHeaders?: Record<string, string>;
    apiBody?: string;

    // Emergency protocol
    protocolId?: string;
    autoExecute?: boolean;
  };

  // Retry configuration
  retry?: {
    enabled: boolean;
    maxRetries: number;
    retryInterval: number; // seconds
    backoffMultiplier?: number;
  };

  // Timeout
  timeout?: number; // seconds
}

export interface AlertRuleGroup {
  groupId: string;
  name: string;
  description: string;
  rules: string[]; // rule IDs
  enabled: boolean;

  // Group-level settings
  settings: {
    aggregation?: {
      enabled: boolean;
      timeWindow: number; // seconds
      maxAlerts: number;
    };
    suppression?: {
      enabled: boolean;
      duration: number; // seconds
    };
  };
}

export interface GlobalAlertSettings {
  // Global notification settings
  notification: {
    defaultChannels: string[];
    quietHours?: {
      enabled: boolean;
      start: string; // HH:mm
      end: string; // HH:mm
      timezone: string;
      exceptCritical: boolean;
    };
  };

  // Rate limiting
  rateLimit: {
    enabled: boolean;
    maxAlertsPerMinute: number;
    maxAlertsPerHour: number;
    action: 'queue' | 'drop' | 'aggregate';
  };

  // Deduplication
  deduplication: {
    enabled: boolean;
    timeWindow: number; // seconds
    fields: string[]; // fields to match for deduplication
  };

  // Auto-acknowledgment
  autoAcknowledge?: {
    enabled: boolean;
    conditions: {
      resolved: boolean;
      duration?: number; // seconds
    };
  };
}

/**
 * Alert Rule Engine Implementation
 */
export class AlertRuleEngineImpl {
  private rules: Map<string, AlertRule> = new Map();
  private activeAlerts: Map<string, Alert> = new Map();
  private settings: GlobalAlertSettings;

  constructor(settings: GlobalAlertSettings) {
    this.settings = settings;
  }

  /**
   * Add alert rule
   */
  public addRule(rule: AlertRule): void {
    this.validateRule(rule);
    this.rules.set(rule.ruleId, rule);
  }

  /**
   * Evaluate sensor reading against all rules
   */
  public async evaluateReading(reading: SensorReading): Promise<Alert[]> {
    const triggeredAlerts: Alert[] = [];

    for (const rule of this.rules.values()) {
      if (!rule.enabled) continue;

      // Check scope
      if (!this.isInScope(reading, rule.scope)) continue;

      // Check time conditions
      if (!this.checkTimeConditions(rule.conditions)) continue;

      // Evaluate trigger
      const triggered = await this.evaluateTrigger(reading, rule.trigger);

      if (triggered) {
        // Check duration requirement
        if (rule.conditions.duration) {
          const durationMet = await this.checkDurationRequirement(
            reading,
            rule.ruleId,
            rule.conditions.duration
          );
          if (!durationMet) continue;
        }

        // Create alert
        const alert = this.createAlert(reading, rule);
        triggeredAlerts.push(alert);

        // Execute actions
        await this.executeActions(alert, rule.actions);

        // Start escalation if configured
        if (rule.escalation?.enabled) {
          await this.startEscalation(alert, rule.escalation.escalationPolicyId);
        }
      }
    }

    return triggeredAlerts;
  }

  /**
   * Check if reading is in rule scope
   */
  private isInScope(reading: SensorReading, scope: AlertRule['scope']): boolean {
    if (scope.allEquipment) return true;

    if (scope.equipmentIds && !scope.equipmentIds.includes(reading.equipmentId)) {
      return false;
    }

    if (scope.sensorIds && !scope.sensorIds.includes(reading.sensorId)) {
      return false;
    }

    if (scope.facilityIds && !scope.facilityIds.includes(reading.facilityId)) {
      return false;
    }

    return true;
  }

  /**
   * Check time-based conditions
   */
  private checkTimeConditions(conditions: AlertRule['conditions']): boolean {
    if (!conditions.timeWindow) return true;

    const now = new Date();
    const currentTime = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`;

    // Check time window
    if (currentTime < conditions.timeWindow.start || currentTime > conditions.timeWindow.end) {
      return false;
    }

    // Check day of week
    if (conditions.timeWindow.daysOfWeek) {
      const currentDay = now.getDay();
      if (!conditions.timeWindow.daysOfWeek.includes(currentDay)) {
        return false;
      }
    }

    return true;
  }

  /**
   * Evaluate trigger condition
   */
  private async evaluateTrigger(
    reading: SensorReading,
    trigger: AlertRule['trigger']
  ): Promise<boolean> {
    switch (trigger.type) {
      case 'threshold':
        return this.evaluateThreshold(reading, trigger.threshold!);

      case 'rate-of-change':
        return await this.evaluateRateOfChange(reading, trigger.rateOfChange!);

      case 'anomaly':
        return await this.evaluateAnomaly(reading, trigger.anomaly!);

      case 'pattern':
        return await this.evaluatePattern(reading, trigger.pattern!);

      case 'composite':
        return await this.evaluateComposite(reading, trigger.composite!);

      default:
        return false;
    }
  }

  /**
   * Evaluate threshold condition
   */
  private evaluateThreshold(
    reading: SensorReading,
    threshold: NonNullable<AlertRule['trigger']['threshold']>
  ): boolean {
    const value = reading.measurement.value;

    switch (threshold.operator) {
      case 'gt':
        return value > threshold.value!;
      case 'lt':
        return value < threshold.value!;
      case 'eq':
        return value === threshold.value!;
      case 'ne':
        return value !== threshold.value!;
      case 'gte':
        return value >= threshold.value!;
      case 'lte':
        return value <= threshold.value!;
      case 'between':
        return value >= threshold.min! && value <= threshold.max!;
      default:
        return false;
    }
  }

  /**
   * Evaluate rate of change
   */
  private async evaluateRateOfChange(
    reading: SensorReading,
    rateConfig: NonNullable<AlertRule['trigger']['rateOfChange']>
  ): Promise<boolean> {
    // Get previous readings within time window
    const previousReadings = await this.getPreviousReadings(
      reading.sensorId,
      rateConfig.timeWindow
    );

    if (previousReadings.length === 0) return false;

    // Calculate rate of change
    const oldestReading = previousReadings[0];
    const timeDiff = (reading.timestamp.getTime() - oldestReading.timestamp.getTime()) / 1000;
    const valueDiff = reading.measurement.value - oldestReading.measurement.value;
    const rate = Math.abs(valueDiff / timeDiff);

    // Check direction if specified
    if (rateConfig.direction) {
      if (rateConfig.direction === 'increasing' && valueDiff <= 0) return false;
      if (rateConfig.direction === 'decreasing' && valueDiff >= 0) return false;
    }

    return rate > rateConfig.maxRate;
  }

  /**
   * Evaluate anomaly detection
   */
  private async evaluateAnomaly(
    reading: SensorReading,
    anomalyConfig: NonNullable<AlertRule['trigger']['anomaly']>
  ): Promise<boolean> {
    // Get training data
    const trainingData = await this.getTrainingData(
      reading.sensorId,
      anomalyConfig.trainingWindow
    );

    if (trainingData.length < 100) return false; // Not enough data

    // Calculate anomaly score based on method
    let anomalyScore = 0;

    switch (anomalyConfig.method) {
      case 'zscore':
        anomalyScore = this.calculateZScore(reading.measurement.value, trainingData);
        break;

      case 'iqr':
        anomalyScore = this.calculateIQRScore(reading.measurement.value, trainingData);
        break;

      case 'isolation-forest':
        anomalyScore = await this.calculateIsolationForestScore(reading, trainingData);
        break;

      case 'ml-model':
        anomalyScore = await this.calculateMLScore(reading, trainingData);
        break;
    }

    // Adjust threshold based on sensitivity
    let threshold = anomalyConfig.threshold;
    if (anomalyConfig.sensitivity === 'low') threshold *= 1.5;
    if (anomalyConfig.sensitivity === 'high') threshold *= 0.7;

    return anomalyScore > threshold;
  }

  /**
   * Evaluate pattern matching
   */
  private async evaluatePattern(
    reading: SensorReading,
    patternConfig: NonNullable<AlertRule['trigger']['pattern']>
  ): Promise<boolean> {
    // Implementation would depend on pattern type
    return false; // Placeholder
  }

  /**
   * Evaluate composite condition
   */
  private async evaluateComposite(
    reading: SensorReading,
    composite: NonNullable<AlertRule['trigger']['composite']>
  ): Promise<boolean> {
    const results = await Promise.all(
      composite.conditions.map(condition => this.evaluateTrigger(reading, condition))
    );

    switch (composite.logic) {
      case 'AND':
        return results.every(r => r);
      case 'OR':
        return results.some(r => r);
      case 'NOT':
        return !results[0];
      default:
        return false;
    }
  }

  /**
   * Check duration requirement
   */
  private async checkDurationRequirement(
    reading: SensorReading,
    ruleId: string,
    duration: NonNullable<AlertRule['conditions']['duration']>
  ): Promise<boolean> {
    // Track condition start time
    const key = `${ruleId}-${reading.sensorId}`;
    // Implementation would track when condition first became true
    // and check if it has persisted for required duration
    return true; // Placeholder
  }

  /**
   * Create alert from triggered rule
   */
  private createAlert(reading: SensorReading, rule: AlertRule): Alert {
    const alert: Alert = {
      alertId: crypto.randomUUID(),
      alertRuleId: rule.ruleId,

      source: {
        sensorId: reading.sensorId,
        equipmentId: reading.equipmentId,
        facilityId: reading.facilityId,
        readingId: reading.readingId
      },

      classification: {
        type: 'threshold-exceeded', // Simplified
        severity: rule.severity,
        category: 'temperature' // Would come from rule
      },

      timing: {
        detected: new Date(),
        triggered: new Date()
      },

      condition: {
        parameter: reading.measurement.parameter,
        currentValue: reading.measurement.value,
        threshold: 0, // Would come from rule
        operator: 'gt',
        unit: reading.measurement.unit,
        description: `${rule.name}: ${rule.description}`
      },

      state: {
        status: 'active'
      },

      notifications: [],
      actions: [],

      schemaVersion: '1.0.0'
    };

    this.activeAlerts.set(alert.alertId, alert);
    return alert;
  }

  /**
   * Execute alert actions
   */
  private async executeActions(alert: Alert, actions: AlertActionConfig[]): Promise<void> {
    for (const action of actions) {
      if (!action.enabled) continue;

      try {
        await this.executeAction(alert, action);
      } catch (error) {
        console.error(`Failed to execute action ${action.actionId}:`, error);

        // Retry if configured
        if (action.retry?.enabled) {
          await this.retryAction(alert, action);
        }
      }
    }
  }

  /**
   * Execute single action
   */
  private async executeAction(alert: Alert, action: AlertActionConfig): Promise<void> {
    switch (action.actionType) {
      case 'notification':
        await this.sendNotification(alert, action.config);
        break;

      case 'webhook':
        await this.callWebhook(alert, action.config);
        break;

      case 'script':
        await this.executeScript(alert, action.config);
        break;

      case 'api-call':
        await this.callAPI(alert, action.config);
        break;

      case 'emergency-protocol':
        await this.executeEmergencyProtocol(alert, action.config);
        break;
    }
  }

  /**
   * Retry action execution
   */
  private async retryAction(alert: Alert, action: AlertActionConfig): Promise<void> {
    const retryConfig = action.retry!;
    let attempt = 0;

    while (attempt < retryConfig.maxRetries) {
      attempt++;

      // Wait before retry
      const delay = retryConfig.retryInterval * Math.pow(
        retryConfig.backoffMultiplier || 1,
        attempt - 1
      );
      await new Promise(resolve => setTimeout(resolve, delay * 1000));

      try {
        await this.executeAction(alert, action);
        return; // Success
      } catch (error) {
        console.error(`Retry ${attempt} failed for action ${action.actionId}:`, error);
      }
    }

    console.error(`All retries exhausted for action ${action.actionId}`);
  }

  /**
   * Start escalation process
   */
  private async startEscalation(alert: Alert, policyId: string): Promise<void> {
    // Escalation implementation
    console.log(`Starting escalation for alert ${alert.alertId} with policy ${policyId}`);
  }

  /**
   * Send notification
   */
  private async sendNotification(alert: Alert, config: AlertActionConfig['config']): Promise<void> {
    // Notification implementation
    console.log(`Sending notification for alert ${alert.alertId}`);
  }

  /**
   * Call webhook
   */
  private async callWebhook(alert: Alert, config: AlertActionConfig['config']): Promise<void> {
    // Webhook implementation
    const response = await fetch(config.webhookUrl!, {
      method: config.method || 'POST',
      headers: config.headers,
      body: config.body || JSON.stringify(alert)
    });

    if (!response.ok) {
      throw new Error(`Webhook call failed: ${response.statusText}`);
    }
  }

  /**
   * Execute script
   */
  private async executeScript(alert: Alert, config: AlertActionConfig['config']): Promise<void> {
    // Script execution implementation
    console.log(`Executing script ${config.scriptPath} for alert ${alert.alertId}`);
  }

  /**
   * Call API
   */
  private async callAPI(alert: Alert, config: AlertActionConfig['config']): Promise<void> {
    // API call implementation
    const response = await fetch(config.apiEndpoint!, {
      method: config.apiMethod,
      headers: config.apiHeaders,
      body: config.apiBody
    });

    if (!response.ok) {
      throw new Error(`API call failed: ${response.statusText}`);
    }
  }

  /**
   * Execute emergency protocol
   */
  private async executeEmergencyProtocol(alert: Alert, config: AlertActionConfig['config']): Promise<void> {
    // Emergency protocol implementation
    console.log(`Executing emergency protocol ${config.protocolId} for alert ${alert.alertId}`);
  }

  /**
   * Validate alert rule
   */
  private validateRule(rule: AlertRule): void {
    if (!rule.ruleId) throw new Error('Rule ID is required');
    if (!rule.name) throw new Error('Rule name is required');
    if (!rule.trigger) throw new Error('Trigger configuration is required');
    if (!rule.severity) throw new Error('Severity is required');
  }

  /**
   * Helper methods (placeholders)
   */
  private async getPreviousReadings(sensorId: string, timeWindow: number): Promise<SensorReading[]> {
    return []; // Database query
  }

  private async getTrainingData(sensorId: string, days: number): Promise<number[]> {
    return []; // Database query
  }

  private calculateZScore(value: number, data: number[]): number {
    const mean = data.reduce((a, b) => a + b, 0) / data.length;
    const stddev = Math.sqrt(
      data.map(x => Math.pow(x - mean, 2)).reduce((a, b) => a + b, 0) / data.length
    );
    return Math.abs((value - mean) / stddev);
  }

  private calculateIQRScore(value: number, data: number[]): number {
    const sorted = [...data].sort((a, b) => a - b);
    const q1 = sorted[Math.floor(sorted.length * 0.25)];
    const q3 = sorted[Math.floor(sorted.length * 0.75)];
    const iqr = q3 - q1;
    const lowerBound = q1 - 1.5 * iqr;
    const upperBound = q3 + 1.5 * iqr;

    if (value < lowerBound || value > upperBound) {
      return Math.max(
        Math.abs(value - lowerBound) / iqr,
        Math.abs(value - upperBound) / iqr
      );
    }
    return 0;
  }

  private async calculateIsolationForestScore(reading: SensorReading, data: number[]): Promise<number> {
    // ML implementation would go here
    return 0;
  }

  private async calculateMLScore(reading: SensorReading, data: number[]): Promise<number> {
    // ML model implementation would go here
    return 0;
  }
}
```

---

## 2. Escalation Management

### 2.1 Escalation Policies

```typescript
/**
 * Alert Escalation System
 *
 * Multi-level escalation with time-based progression
 */

export interface EscalationPolicy {
  policyId: string;
  name: string;
  description: string;
  enabled: boolean;

  // Escalation levels
  levels: EscalationLevel[];

  // Escalation behavior
  behavior: {
    repeatCycle: boolean; // Repeat from level 1 if all levels exhausted
    notifyAll: boolean; // Notify all levels simultaneously
    requireAcknowledgment: boolean; // Require acknowledgment to stop escalation
  };

  // Scope
  scope: {
    facilities?: string[];
    equipmentTypes?: string[];
    severities?: ('critical' | 'high' | 'medium' | 'low')[];
  };

  // Metadata
  metadata: {
    created: Date;
    createdBy: string;
    modified: Date;
    modifiedBy: string;
  };
}

export interface EscalationLevel {
  level: number;
  name: string;

  // Delay before this level activates
  delayMinutes: number;

  // Recipients at this level
  recipients: EscalationRecipient[];

  // Notification settings for this level
  notification: {
    channels: ('email' | 'sms' | 'phone' | 'push' | 'pager')[];
    message?: string;
    priority: 'high' | 'normal' | 'low';
    requireConfirmation: boolean;
  };

  // Actions to execute at this level
  actions?: {
    actionType: string;
    config: Record<string, any>;
  }[];
}

export interface EscalationRecipient {
  recipientId: string;
  type: 'user' | 'group' | 'role' | 'external';

  // User/group identification
  userId?: string;
  groupId?: string;
  roleId?: string;
  externalContact?: {
    name: string;
    email?: string;
    phone?: string;
  };

  // Availability
  availability?: {
    schedule: 'always' | 'business-hours' | 'custom';
    customSchedule?: {
      timezone: string;
      workingHours: {
        day: number; // 0=Sunday
        start: string; // HH:mm
        end: string; // HH:mm
      }[];
    };
  };
}

export interface EscalationState {
  escalationId: string;
  alertId: string;
  policyId: string;

  // Current state
  currentLevel: number;
  status: 'active' | 'paused' | 'completed' | 'cancelled';

  // Timing
  startedAt: Date;
  lastEscalation?: Date;
  nextEscalation?: Date;
  completedAt?: Date;

  // History
  history: EscalationEvent[];

  // Resolution
  resolvedBy?: string;
  resolvedAt?: Date;
  resolution?: string;
}

export interface EscalationEvent {
  eventId: string;
  timestamp: Date;
  level: number;
  action: 'escalated' | 'notified' | 'acknowledged' | 'paused' | 'resumed' | 'cancelled';
  recipient?: string;
  channel?: string;
  success: boolean;
  error?: string;
  notes?: string;
}

/**
 * Escalation Manager
 */
export class EscalationManager {
  private policies: Map<string, EscalationPolicy> = new Map();
  private activeEscalations: Map<string, EscalationState> = new Map();
  private timers: Map<string, NodeJS.Timeout> = new Map();

  /**
   * Add escalation policy
   */
  public addPolicy(policy: EscalationPolicy): void {
    this.validatePolicy(policy);
    this.policies.set(policy.policyId, policy);
  }

  /**
   * Start escalation for alert
   */
  public startEscalation(alert: Alert, policyId: string): EscalationState {
    const policy = this.policies.get(policyId);
    if (!policy || !policy.enabled) {
      throw new Error(`Policy ${policyId} not found or disabled`);
    }

    // Check if alert matches policy scope
    if (!this.matchesScope(alert, policy.scope)) {
      throw new Error('Alert does not match policy scope');
    }

    const escalation: EscalationState = {
      escalationId: crypto.randomUUID(),
      alertId: alert.alertId,
      policyId,
      currentLevel: 0,
      status: 'active',
      startedAt: new Date(),
      history: []
    };

    this.activeEscalations.set(escalation.escalationId, escalation);

    // Start with level 1
    if (policy.behavior.notifyAll) {
      // Notify all levels simultaneously
      policy.levels.forEach(level => {
        this.executeLevel(escalation, policy, level);
      });
    } else {
      // Start with first level
      this.scheduleNextLevel(escalation, policy);
    }

    return escalation;
  }

  /**
   * Acknowledge escalation
   */
  public acknowledgeEscalation(
    escalationId: string,
    acknowledgedBy: string,
    notes?: string
  ): void {
    const escalation = this.activeEscalations.get(escalationId);
    if (!escalation) {
      throw new Error(`Escalation ${escalationId} not found`);
    }

    const policy = this.policies.get(escalation.policyId);
    if (!policy) return;

    // Record acknowledgment
    escalation.history.push({
      eventId: crypto.randomUUID(),
      timestamp: new Date(),
      level: escalation.currentLevel,
      action: 'acknowledged',
      recipient: acknowledgedBy,
      success: true,
      notes
    });

    // Stop escalation if policy requires acknowledgment
    if (policy.behavior.requireAcknowledgment) {
      this.stopEscalation(escalationId);
    }
  }

  /**
   * Pause escalation
   */
  public pauseEscalation(escalationId: string, reason?: string): void {
    const escalation = this.activeEscalations.get(escalationId);
    if (!escalation) return;

    escalation.status = 'paused';

    // Cancel scheduled timer
    const timer = this.timers.get(escalationId);
    if (timer) {
      clearTimeout(timer);
      this.timers.delete(escalationId);
    }

    escalation.history.push({
      eventId: crypto.randomUUID(),
      timestamp: new Date(),
      level: escalation.currentLevel,
      action: 'paused',
      success: true,
      notes: reason
    });
  }

  /**
   * Resume escalation
   */
  public resumeEscalation(escalationId: string): void {
    const escalation = this.activeEscalations.get(escalationId);
    if (!escalation || escalation.status !== 'paused') return;

    escalation.status = 'active';

    escalation.history.push({
      eventId: crypto.randomUUID(),
      timestamp: new Date(),
      level: escalation.currentLevel,
      action: 'resumed',
      success: true
    });

    // Resume escalation
    const policy = this.policies.get(escalation.policyId);
    if (policy) {
      this.scheduleNextLevel(escalation, policy);
    }
  }

  /**
   * Stop escalation
   */
  public stopEscalation(escalationId: string, resolution?: string): void {
    const escalation = this.activeEscalations.get(escalationId);
    if (!escalation) return;

    escalation.status = 'completed';
    escalation.completedAt = new Date();
    escalation.resolution = resolution;

    // Cancel scheduled timer
    const timer = this.timers.get(escalationId);
    if (timer) {
      clearTimeout(timer);
      this.timers.delete(escalationId);
    }

    escalation.history.push({
      eventId: crypto.randomUUID(),
      timestamp: new Date(),
      level: escalation.currentLevel,
      action: 'cancelled',
      success: true,
      notes: resolution
    });
  }

  /**
   * Schedule next escalation level
   */
  private scheduleNextLevel(escalation: EscalationState, policy: EscalationPolicy): void {
    const nextLevel = escalation.currentLevel + 1;

    if (nextLevel > policy.levels.length) {
      if (policy.behavior.repeatCycle) {
        // Restart from level 1
        escalation.currentLevel = 0;
        this.scheduleNextLevel(escalation, policy);
      } else {
        // Escalation complete
        escalation.status = 'completed';
        escalation.completedAt = new Date();
      }
      return;
    }

    const level = policy.levels[nextLevel - 1];
    const delayMs = level.delayMinutes * 60 * 1000;

    const timer = setTimeout(() => {
      this.executeLevel(escalation, policy, level);
      this.scheduleNextLevel(escalation, policy);
    }, delayMs);

    this.timers.set(escalation.escalationId, timer);

    escalation.currentLevel = nextLevel;
    escalation.nextEscalation = new Date(Date.now() + delayMs);
  }

  /**
   * Execute escalation level
   */
  private async executeLevel(
    escalation: EscalationState,
    policy: EscalationPolicy,
    level: EscalationLevel
  ): Promise<void> {
    console.log(`Executing escalation level ${level.level} for ${escalation.alertId}`);

    escalation.lastEscalation = new Date();

    // Notify all recipients at this level
    for (const recipient of level.recipients) {
      // Check availability
      if (!this.isAvailable(recipient)) continue;

      // Send notifications through configured channels
      for (const channel of level.notification.channels) {
        try {
          await this.sendEscalationNotification(escalation, recipient, channel, level);

          escalation.history.push({
            eventId: crypto.randomUUID(),
            timestamp: new Date(),
            level: level.level,
            action: 'notified',
            recipient: recipient.recipientId,
            channel,
            success: true
          });
        } catch (error) {
          escalation.history.push({
            eventId: crypto.randomUUID(),
            timestamp: new Date(),
            level: level.level,
            action: 'notified',
            recipient: recipient.recipientId,
            channel,
            success: false,
            error: (error as Error).message
          });
        }
      }
    }

    // Execute level actions
    if (level.actions) {
      for (const action of level.actions) {
        try {
          await this.executeEscalationAction(escalation, action);
        } catch (error) {
          console.error(`Failed to execute escalation action:`, error);
        }
      }
    }

    escalation.history.push({
      eventId: crypto.randomUUID(),
      timestamp: new Date(),
      level: level.level,
      action: 'escalated',
      success: true
    });
  }

  /**
   * Check if recipient is available
   */
  private isAvailable(recipient: EscalationRecipient): boolean {
    if (!recipient.availability) return true;

    if (recipient.availability.schedule === 'always') return true;

    const now = new Date();
    const currentDay = now.getDay();
    const currentTime = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`;

    if (recipient.availability.schedule === 'business-hours') {
      // Simple business hours check (Mon-Fri, 9am-5pm)
      if (currentDay === 0 || currentDay === 6) return false;
      if (currentTime < '09:00' || currentTime > '17:00') return false;
      return true;
    }

    if (recipient.availability.schedule === 'custom' && recipient.availability.customSchedule) {
      const schedule = recipient.availability.customSchedule;
      const daySchedule = schedule.workingHours.find(h => h.day === currentDay);
      if (!daySchedule) return false;

      if (currentTime < daySchedule.start || currentTime > daySchedule.end) {
        return false;
      }
      return true;
    }

    return true;
  }

  /**
   * Send escalation notification
   */
  private async sendEscalationNotification(
    escalation: EscalationState,
    recipient: EscalationRecipient,
    channel: string,
    level: EscalationLevel
  ): Promise<void> {
    // Implementation would send notification through specified channel
    console.log(`Sending ${channel} notification to ${recipient.recipientId} for escalation level ${level.level}`);
  }

  /**
   * Execute escalation action
   */
  private async executeEscalationAction(
    escalation: EscalationState,
    action: NonNullable<EscalationLevel['actions']>[0]
  ): Promise<void> {
    // Implementation would execute the specified action
    console.log(`Executing escalation action ${action.actionType}`);
  }

  /**
   * Check if alert matches policy scope
   */
  private matchesScope(alert: Alert, scope: EscalationPolicy['scope']): boolean {
    if (scope.facilities && !scope.facilities.includes(alert.source.facilityId)) {
      return false;
    }

    if (scope.severities && !scope.severities.includes(alert.classification.severity)) {
      return false;
    }

    return true;
  }

  /**
   * Validate escalation policy
   */
  private validatePolicy(policy: EscalationPolicy): void {
    if (!policy.policyId) throw new Error('Policy ID is required');
    if (!policy.name) throw new Error('Policy name is required');
    if (!policy.levels || policy.levels.length === 0) {
      throw new Error('At least one escalation level is required');
    }

    // Validate levels are in order
    for (let i = 1; i < policy.levels.length; i++) {
      if (policy.levels[i].level !== policy.levels[i-1].level + 1) {
        throw new Error('Escalation levels must be sequential');
      }
    }
  }
}
```

---

## 3. Notification Channels

### 3.1 Multi-Channel Notification System

```typescript
/**
 * Notification Channel Management
 *
 * Support for email, SMS, phone, push, webhook, and pager notifications
 */

export interface NotificationChannel {
  channelId: string;
  type: 'email' | 'sms' | 'phone' | 'push' | 'webhook' | 'pager';
  name: string;
  description: string;
  enabled: boolean;

  // Channel-specific configuration
  config: EmailConfig | SMSConfig | PhoneConfig | PushConfig | WebhookConfig | PagerConfig;

  // Delivery settings
  delivery: {
    priority: 'high' | 'normal' | 'low';
    retries: number;
    retryInterval: number; // seconds
    timeout: number; // seconds
  };

  // Rate limiting
  rateLimit?: {
    enabled: boolean;
    maxPerMinute: number;
    maxPerHour: number;
  };

  // Testing
  testMode: boolean;
  testRecipients?: string[];
}

export interface EmailConfig {
  provider: 'smtp' | 'ses' | 'sendgrid' | 'mailgun';
  from: string;
  replyTo?: string;

  // SMTP settings
  smtp?: {
    host: string;
    port: number;
    secure: boolean;
    username: string;
    password: string;
  };

  // Cloud provider settings
  apiKey?: string;
  region?: string;

  // Template settings
  templates: {
    critical: string;
    high: string;
    medium: string;
    low: string;
  };
}

export interface SMSConfig {
  provider: 'twilio' | 'aws-sns' | 'nexmo';
  from: string;
  accountSid?: string;
  authToken?: string;
  apiKey?: string;

  // Message settings
  maxLength: number;
  useShortLinks: boolean;
}

export interface PhoneConfig {
  provider: 'twilio' | 'aws-connect';
  from: string;
  accountSid?: string;
  authToken?: string;

  // Voice message settings
  voice: 'male' | 'female';
  language: string;
  repeatCount: number;
}

export interface PushConfig {
  provider: 'fcm' | 'apns' | 'onesignal';
  apiKey: string;
  appId?: string;

  // Push notification settings
  sound: string;
  badge: boolean;
  priority: 'high' | 'normal';
}

export interface WebhookConfig {
  url: string;
  method: 'GET' | 'POST' | 'PUT';
  headers?: Record<string, string>;

  // Authentication
  auth?: {
    type: 'none' | 'basic' | 'bearer' | 'apikey';
    username?: string;
    password?: string;
    token?: string;
    apiKey?: string;
    apiKeyHeader?: string;
  };

  // Payload format
  payloadFormat: 'json' | 'xml' | 'form-urlencoded';
  template?: string;
}

export interface PagerConfig {
  provider: 'pagerduty' | 'opsgenie' | 'victorops';
  apiKey: string;
  routingKey?: string;
  serviceKey?: string;

  // Escalation settings
  escalationPolicy?: string;
  urgency: 'high' | 'low';
}

/**
 * Notification Manager
 */
export class NotificationManager {
  private channels: Map<string, NotificationChannel> = new Map();

  /**
   * Add notification channel
   */
  public addChannel(channel: NotificationChannel): void {
    this.validateChannel(channel);
    this.channels.set(channel.channelId, channel);
  }

  /**
   * Send notification
   */
  public async sendNotification(
    alert: Alert,
    channelId: string,
    recipients: string[]
  ): Promise<void> {
    const channel = this.channels.get(channelId);
    if (!channel || !channel.enabled) {
      throw new Error(`Channel ${channelId} not found or disabled`);
    }

    // Check rate limit
    if (channel.rateLimit?.enabled) {
      const allowed = await this.checkRateLimit(channel);
      if (!allowed) {
        throw new Error(`Rate limit exceeded for channel ${channelId}`);
      }
    }

    // Use test recipients if in test mode
    const finalRecipients = channel.testMode && channel.testRecipients
      ? channel.testRecipients
      : recipients;

    // Send based on channel type
    switch (channel.type) {
      case 'email':
        await this.sendEmail(alert, channel, finalRecipients);
        break;
      case 'sms':
        await this.sendSMS(alert, channel, finalRecipients);
        break;
      case 'phone':
        await this.makePhoneCall(alert, channel, finalRecipients);
        break;
      case 'push':
        await this.sendPushNotification(alert, channel, finalRecipients);
        break;
      case 'webhook':
        await this.sendWebhook(alert, channel);
        break;
      case 'pager':
        await this.sendPagerAlert(alert, channel);
        break;
    }
  }

  /**
   * Send email notification
   */
  private async sendEmail(
    alert: Alert,
    channel: NotificationChannel,
    recipients: string[]
  ): Promise<void> {
    const config = channel.config as EmailConfig;

    const subject = `[${alert.classification.severity.toUpperCase()}] ${alert.condition.description}`;
    const body = this.formatEmailBody(alert, config);

    console.log(`Sending email to ${recipients.join(', ')}`);
    console.log(`Subject: ${subject}`);
    // Actual email sending implementation would go here
  }

  /**
   * Send SMS notification
   */
  private async sendSMS(
    alert: Alert,
    channel: NotificationChannel,
    recipients: string[]
  ): Promise<void> {
    const config = channel.config as SMSConfig;
    const message = this.formatSMSMessage(alert, config);

    for (const recipient of recipients) {
      console.log(`Sending SMS to ${recipient}: ${message}`);
      // Actual SMS sending implementation would go here
    }
  }

  /**
   * Make phone call
   */
  private async makePhoneCall(
    alert: Alert,
    channel: NotificationChannel,
    recipients: string[]
  ): Promise<void> {
    const config = channel.config as PhoneConfig;
    const message = this.formatVoiceMessage(alert);

    for (const recipient of recipients) {
      console.log(`Calling ${recipient} with message: ${message}`);
      // Actual phone call implementation would go here
    }
  }

  /**
   * Send push notification
   */
  private async sendPushNotification(
    alert: Alert,
    channel: NotificationChannel,
    recipients: string[]
  ): Promise<void> {
    const config = channel.config as PushConfig;

    const notification = {
      title: `${alert.classification.severity.toUpperCase()} Alert`,
      body: alert.condition.description,
      data: {
        alertId: alert.alertId,
        severity: alert.classification.severity
      }
    };

    console.log(`Sending push notification to ${recipients.length} devices`);
    // Actual push notification implementation would go here
  }

  /**
   * Send webhook
   */
  private async sendWebhook(
    alert: Alert,
    channel: NotificationChannel
  ): Promise<void> {
    const config = channel.config as WebhookConfig;

    const payload = config.template
      ? this.formatWebhookPayload(alert, config.template)
      : alert;

    const headers = {
      'Content-Type': config.payloadFormat === 'json' ? 'application/json' : 'application/x-www-form-urlencoded',
      ...config.headers
    };

    // Add authentication
    if (config.auth) {
      switch (config.auth.type) {
        case 'basic':
          const credentials = btoa(`${config.auth.username}:${config.auth.password}`);
          headers['Authorization'] = `Basic ${credentials}`;
          break;
        case 'bearer':
          headers['Authorization'] = `Bearer ${config.auth.token}`;
          break;
        case 'apikey':
          headers[config.auth.apiKeyHeader || 'X-API-Key'] = config.auth.apiKey!;
          break;
      }
    }

    console.log(`Sending webhook to ${config.url}`);

    const response = await fetch(config.url, {
      method: config.method,
      headers,
      body: JSON.stringify(payload)
    });

    if (!response.ok) {
      throw new Error(`Webhook failed: ${response.statusText}`);
    }
  }

  /**
   * Send pager alert
   */
  private async sendPagerAlert(
    alert: Alert,
    channel: NotificationChannel
  ): Promise<void> {
    const config = channel.config as PagerConfig;

    console.log(`Sending pager alert via ${config.provider}`);
    // Actual pager service integration would go here
  }

  /**
   * Format email body
   */
  private formatEmailBody(alert: Alert, config: EmailConfig): string {
    // Use template based on severity
    const template = config.templates[alert.classification.severity];

    return `
      Alert: ${alert.condition.description}

      Severity: ${alert.classification.severity.toUpperCase()}
      Equipment: ${alert.source.equipmentId}
      Sensor: ${alert.source.sensorId}

      Current Value: ${alert.condition.currentValue} ${alert.condition.unit}
      Threshold: ${alert.condition.threshold} ${alert.condition.unit}

      Detected: ${alert.timing.detected.toISOString()}

      Alert ID: ${alert.alertId}
    `;
  }

  /**
   * Format SMS message
   */
  private formatSMSMessage(alert: Alert, config: SMSConfig): string {
    let message = `[${alert.classification.severity.toUpperCase()}] ${alert.condition.description}`;

    if (message.length > config.maxLength) {
      message = message.substring(0, config.maxLength - 3) + '...';
    }

    return message;
  }

  /**
   * Format voice message
   */
  private formatVoiceMessage(alert: Alert): string {
    return `This is a ${alert.classification.severity} alert. ${alert.condition.description}. Please acknowledge.`;
  }

  /**
   * Format webhook payload
   */
  private formatWebhookPayload(alert: Alert, template: string): any {
    // Template processing would go here
    return alert;
  }

  /**
   * Check rate limit
   */
  private async checkRateLimit(channel: NotificationChannel): Promise<boolean> {
    // Rate limiting implementation
    return true;
  }

  /**
   * Validate notification channel
   */
  private validateChannel(channel: NotificationChannel): void {
    if (!channel.channelId) throw new Error('Channel ID is required');
    if (!channel.type) throw new Error('Channel type is required');
    if (!channel.config) throw new Error('Channel configuration is required');
  }
}
```

---

## Conclusion

The WIA Cryo Monitoring alerting system provides comprehensive, intelligent alert management with flexible rules, multi-level escalation, and multi-channel notifications. This ensures that critical conditions are detected immediately and the right people are notified through the most appropriate channels.

**弘益人間 (Hongik Ingan)** - Protecting samples through intelligent, reliable alerting.

---

© 2026 World Industry Association
Licensed under Apache 2.0
