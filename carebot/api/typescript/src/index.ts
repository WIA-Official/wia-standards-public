/**
 * WIA Carebot Standard - SDK Implementation
 * @packageDocumentation
 * @module wia-carebot
 */

import { EventEmitter } from 'events';
import * as types from './types';

export * from './types';

export class WIACarebot extends EventEmitter {
  private config: types.CarebotConfig;
  private carebots: Map<string, types.CarebotInfo> = new Map();
  private patients: Map<string, types.Patient> = new Map();
  private tasks: Map<string, types.CareTask> = new Map();
  private alerts: Map<string, types.CareAlert> = new Map();
  private vitals: Map<string, types.VitalSigns[]> = new Map();

  constructor(config: types.CarebotConfig) {
    super();
    this.config = config;
  }

  async registerCarebot(carebot: Omit<types.CarebotInfo, 'id'>): Promise<types.CarebotInfo> {
    const newCarebot: types.CarebotInfo = { ...carebot, id: `bot-${Date.now()}` };
    this.carebots.set(newCarebot.id, newCarebot);
    this.emit('carebot-registered', newCarebot);
    return newCarebot;
  }

  getCarebot(carebotId: string): types.CarebotInfo | undefined {
    return this.carebots.get(carebotId);
  }

  async registerPatient(patient: Omit<types.Patient, 'id'>): Promise<types.Patient> {
    const newPatient: types.Patient = { ...patient, id: `patient-${Date.now()}` };
    this.patients.set(newPatient.id, newPatient);
    this.emit('patient-registered', newPatient);
    return newPatient;
  }

  getPatient(patientId: string): types.Patient | undefined {
    return this.patients.get(patientId);
  }

  async assignCarebot(patientId: string, carebotId: string): Promise<void> {
    const patient = this.patients.get(patientId);
    const carebot = this.carebots.get(carebotId);
    if (!patient || !carebot) throw new Error('Patient or carebot not found');

    patient.assignedCarebot = carebotId;
    this.emit('carebot-assigned', { patientId, carebotId });
  }

  async recordVitals(vitals: types.VitalSigns): Promise<types.VitalSigns> {
    vitals.status = this.evaluateVitals(vitals);

    const history = this.vitals.get(vitals.patientId) || [];
    history.push(vitals);
    this.vitals.set(vitals.patientId, history);

    if (vitals.status !== 'normal') {
      await this.createAlert({
        patientId: vitals.patientId,
        type: 'vital_sign',
        priority: vitals.status === 'critical' ? types.AlertPriority.Critical : types.AlertPriority.High,
        message: `Abnormal vital signs detected: ${vitals.status}`,
        data: vitals as unknown as Record<string, unknown>
      });
    }

    this.emit('vitals-recorded', vitals);
    return vitals;
  }

  private evaluateVitals(vitals: types.VitalSigns): 'normal' | 'warning' | 'critical' {
    const t = this.config.alertThresholds;

    if (vitals.heartRate && (vitals.heartRate < t.heartRateLow || vitals.heartRate > t.heartRateHigh)) {
      return vitals.heartRate < t.heartRateLow * 0.8 || vitals.heartRate > t.heartRateHigh * 1.2 ? 'critical' : 'warning';
    }

    if (vitals.spo2 && vitals.spo2 < t.spo2Low) {
      return vitals.spo2 < 90 ? 'critical' : 'warning';
    }

    if (vitals.temperature && vitals.temperature > t.temperatureHigh) {
      return vitals.temperature > 40 ? 'critical' : 'warning';
    }

    return 'normal';
  }

  getVitalsHistory(patientId: string, limit?: number): types.VitalSigns[] {
    const history = this.vitals.get(patientId) || [];
    return limit ? history.slice(-limit) : history;
  }

  async createTask(task: Omit<types.CareTask, 'id' | 'status'>): Promise<types.CareTask> {
    const newTask: types.CareTask = { ...task, id: `task-${Date.now()}`, status: 'pending' };
    this.tasks.set(newTask.id, newTask);
    this.emit('task-created', newTask);
    return newTask;
  }

  async startTask(taskId: string): Promise<void> {
    const task = this.tasks.get(taskId);
    if (!task) throw new Error('Task not found');

    task.status = 'in_progress';
    this.emit('task-started', task);
  }

  async completeTask(taskId: string, result: types.TaskResult): Promise<void> {
    const task = this.tasks.get(taskId);
    if (!task) throw new Error('Task not found');

    task.status = result.success ? 'completed' : 'failed';
    task.result = result;
    this.emit('task-completed', task);
  }

  getPatientTasks(patientId: string): types.CareTask[] {
    return Array.from(this.tasks.values()).filter(t => t.patientId === patientId);
  }

  async createAlert(alert: Omit<types.CareAlert, 'id' | 'timestamp' | 'actions'>): Promise<types.CareAlert> {
    const newAlert: types.CareAlert = {
      ...alert,
      id: `alert-${Date.now()}`,
      timestamp: new Date(),
      actions: []
    };

    // Auto-notify emergency contacts for critical alerts
    if (alert.priority === types.AlertPriority.Critical) {
      const patient = this.patients.get(alert.patientId);
      if (patient) {
        const primary = patient.emergencyContacts.find(c => c.isPrimary);
        if (primary) {
          newAlert.actions.push({
            type: 'notify',
            target: primary.phone,
            status: 'pending'
          });
        }
      }
    }

    this.alerts.set(newAlert.id, newAlert);
    this.emit('alert-created', newAlert);
    return newAlert;
  }

  async acknowledgeAlert(alertId: string, userId: string): Promise<void> {
    const alert = this.alerts.get(alertId);
    if (!alert) throw new Error('Alert not found');

    alert.acknowledgedAt = new Date();
    alert.acknowledgedBy = userId;
    this.emit('alert-acknowledged', alert);
  }

  async resolveAlert(alertId: string): Promise<void> {
    const alert = this.alerts.get(alertId);
    if (!alert) throw new Error('Alert not found');

    alert.resolvedAt = new Date();
    this.emit('alert-resolved', alert);
  }

  getActiveAlerts(patientId?: string): types.CareAlert[] {
    let alerts = Array.from(this.alerts.values()).filter(a => !a.resolvedAt);
    if (patientId) alerts = alerts.filter(a => a.patientId === patientId);
    return alerts.sort((a, b) => {
      const priorityOrder = { critical: 0, high: 1, medium: 2, low: 3 };
      return priorityOrder[a.priority] - priorityOrder[b.priority];
    });
  }

  async recordInteraction(interaction: Omit<types.Interaction, 'id' | 'timestamp'>): Promise<types.Interaction> {
    const newInteraction: types.Interaction = {
      ...interaction,
      id: `int-${Date.now()}`,
      timestamp: new Date()
    };

    this.emit('interaction-recorded', newInteraction);
    return newInteraction;
  }

  async generateDailyReport(patientId: string, date: Date): Promise<types.DailyReport> {
    const patient = this.patients.get(patientId);
    if (!patient) throw new Error('Patient not found');

    const dayVitals = (this.vitals.get(patientId) || []).filter(v =>
      v.timestamp.toDateString() === date.toDateString()
    );

    const dayTasks = Array.from(this.tasks.values()).filter(t =>
      t.patientId === patientId && t.scheduledTime?.toDateString() === date.toDateString()
    );

    const completedMeds = dayTasks.filter(t => t.category === types.TaskCategory.Medication && t.status === 'completed').length;
    const totalMeds = dayTasks.filter(t => t.category === types.TaskCategory.Medication).length;

    return {
      patientId,
      date,
      vitalsSummary: {
        readings: dayVitals.length,
        avgHeartRate: dayVitals.length ? dayVitals.reduce((sum, v) => sum + (v.heartRate || 0), 0) / dayVitals.length : undefined,
        anomalies: dayVitals.filter(v => v.status !== 'normal').length
      },
      medicationCompliance: totalMeds > 0 ? (completedMeds / totalMeds) * 100 : 100,
      activityLevel: 0,
      tasks: dayTasks,
      interactions: [],
      alerts: Array.from(this.alerts.values()).filter(a =>
        a.patientId === patientId && a.timestamp.toDateString() === date.toDateString()
      )
    };
  }

  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    tests.push({
      testName: 'Configuration Validation',
      passed: this.config.apiEndpoint !== undefined,
      notes: 'API endpoint must be defined'
    });

    tests.push({
      testName: 'Alert Thresholds',
      passed: this.config.alertThresholds !== undefined,
      notes: 'Alert thresholds must be configured'
    });

    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Fall Detection',
        passed: this.config.enableFallDetection === true,
        notes: 'Fall detection required for Silver/Gold'
      });
    }

    if (targetLevel === types.CertificationLevel.Gold) {
      tests.push({
        testName: 'Vitals Monitoring',
        passed: this.config.enableVitalsMonitoring === true,
        notes: 'Vitals monitoring required for Gold'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-CAREBOT',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

export default { WIACarebot };
