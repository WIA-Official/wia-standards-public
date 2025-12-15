/**
 * WIA Exoskeleton Therapist Dashboard
 *
 * Main dashboard implementation for therapist patient management
 */

import {
  Patient,
  PatientProfile,
  PatientStatus,
  ProgramAssignment,
  SessionSummary,
  SessionStatus,
  AssessmentRecord,
  AssessmentType,
  ProgressData,
  ProgressSummary,
  MetricProgress,
  Milestone,
  PatientAlert,
  PatientAlertType,
  AlertSeverity,
  ProgressReport,
  LiveSessionView,
  LiveSessionStatus,
  DashboardState,
  DashboardFilters,
  DashboardView,
  TherapistUser,
  ScheduledSession,
  DateRange,
  Condition,
} from './types';

// ============================================================================
// Dashboard Interface
// ============================================================================

export interface ITherapistDashboard {
  // Patient Management
  getPatients(filters?: Partial<DashboardFilters>): Promise<Patient[]>;
  getPatient(patientId: string): Promise<Patient | null>;
  createPatient(profile: PatientProfile): Promise<Patient>;
  updatePatient(patientId: string, updates: Partial<PatientProfile>): Promise<Patient>;
  archivePatient(patientId: string): Promise<void>;

  // Program Management
  assignProgram(patientId: string, programId: string, startDate: Date): Promise<ProgramAssignment>;
  modifyProgram(assignmentId: string, modifications: object): Promise<ProgramAssignment>;
  unassignProgram(assignmentId: string): Promise<void>;

  // Session Management
  getSessionHistory(patientId: string, dateRange?: DateRange): Promise<SessionSummary[]>;
  scheduleSession(patientId: string, session: Partial<ScheduledSession>): Promise<ScheduledSession>;
  cancelSession(scheduleId: string, reason: string): Promise<void>;

  // Progress Tracking
  getProgress(patientId: string, dateRange?: DateRange): Promise<ProgressData>;
  getMetricHistory(patientId: string, metricId: string): Promise<MetricProgress>;
  getMilestones(patientId: string): Promise<Milestone[]>;

  // Assessments
  recordAssessment(patientId: string, assessment: Partial<AssessmentRecord>): Promise<AssessmentRecord>;
  getAssessmentHistory(patientId: string): Promise<AssessmentRecord[]>;

  // Alerts
  getAlerts(filters?: { patientId?: string; severity?: AlertSeverity[] }): Promise<PatientAlert[]>;
  acknowledgeAlert(alertId: string): Promise<void>;

  // Reports
  generateProgressReport(patientId: string, period: DateRange): Promise<ProgressReport>;
  exportReport(reportId: string, format: 'pdf' | 'csv' | 'json'): Promise<Blob>;

  // Live Monitoring
  getLiveSessions(): Promise<LiveSessionView[]>;
  watchSession(sessionId: string): Promise<LiveSessionView>;
  sendAlert(sessionId: string, message: string): Promise<void>;
}

// ============================================================================
// Dashboard Implementation
// ============================================================================

export class TherapistDashboard implements ITherapistDashboard {
  private state: DashboardState;
  private apiEndpoint: string;

  constructor(user: TherapistUser, apiEndpoint: string = '/api/v1') {
    this.apiEndpoint = apiEndpoint;
    this.state = {
      currentUser: user,
      patients: [],
      selectedPatient: null,
      activeSessions: [],
      alerts: [],
      schedule: [],
      filters: {
        patientStatus: [PatientStatus.ACTIVE],
        conditions: [],
        dateRange: {
          start: new Date(Date.now() - 30 * 24 * 60 * 60 * 1000),
          end: new Date(),
        },
        searchQuery: '',
      },
      view: DashboardView.PATIENT_LIST,
    };
  }

  // --------------------------------------------------------------------------
  // Patient Management
  // --------------------------------------------------------------------------

  async getPatients(filters?: Partial<DashboardFilters>): Promise<Patient[]> {
    const appliedFilters = { ...this.state.filters, ...filters };

    // In real implementation, this would call the API
    let patients = [...this.state.patients];

    // Apply filters
    if (appliedFilters.patientStatus.length > 0) {
      patients = patients.filter(p =>
        appliedFilters.patientStatus.includes(p.status)
      );
    }

    if (appliedFilters.conditions.length > 0) {
      patients = patients.filter(p =>
        appliedFilters.conditions.includes(p.profile.diagnosis.condition)
      );
    }

    if (appliedFilters.searchQuery) {
      const query = appliedFilters.searchQuery.toLowerCase();
      patients = patients.filter(p =>
        p.profile.firstName.toLowerCase().includes(query) ||
        p.profile.lastName.toLowerCase().includes(query) ||
        p.profile.medicalRecordNumber.includes(query)
      );
    }

    return patients;
  }

  async getPatient(patientId: string): Promise<Patient | null> {
    const patient = this.state.patients.find(p => p.patientId === patientId);
    if (patient) {
      this.state.selectedPatient = patient;
    }
    return patient || null;
  }

  async createPatient(profile: PatientProfile): Promise<Patient> {
    const newPatient: Patient = {
      patientId: this.generateId('PAT'),
      profile,
      currentProgram: null,
      sessions: [],
      assessments: [],
      alerts: [],
      status: PatientStatus.ACTIVE,
    };

    this.state.patients.push(newPatient);
    return newPatient;
  }

  async updatePatient(patientId: string, updates: Partial<PatientProfile>): Promise<Patient> {
    const patient = this.state.patients.find(p => p.patientId === patientId);
    if (!patient) {
      throw new Error(`Patient not found: ${patientId}`);
    }

    patient.profile = { ...patient.profile, ...updates };
    return patient;
  }

  async archivePatient(patientId: string): Promise<void> {
    const patient = this.state.patients.find(p => p.patientId === patientId);
    if (!patient) {
      throw new Error(`Patient not found: ${patientId}`);
    }

    patient.status = PatientStatus.DISCHARGED;
  }

  // --------------------------------------------------------------------------
  // Program Management
  // --------------------------------------------------------------------------

  async assignProgram(
    patientId: string,
    programId: string,
    startDate: Date
  ): Promise<ProgramAssignment> {
    const patient = await this.getPatient(patientId);
    if (!patient) {
      throw new Error(`Patient not found: ${patientId}`);
    }

    const assignment: ProgramAssignment = {
      assignmentId: this.generateId('ASN'),
      programId,
      programName: '', // Would be fetched from program
      startDate,
      endDate: new Date(startDate.getTime() + 12 * 7 * 24 * 60 * 60 * 1000), // 12 weeks
      currentPhase: 1,
      currentSession: 0,
      completedSessions: 0,
      totalSessions: 36, // 3x per week for 12 weeks
      progressPercent: 0,
      modifications: [],
    };

    patient.currentProgram = assignment;
    return assignment;
  }

  async modifyProgram(
    assignmentId: string,
    modifications: object
  ): Promise<ProgramAssignment> {
    // Find assignment and apply modifications
    for (const patient of this.state.patients) {
      if (patient.currentProgram?.assignmentId === assignmentId) {
        Object.assign(patient.currentProgram, modifications);
        return patient.currentProgram;
      }
    }
    throw new Error(`Assignment not found: ${assignmentId}`);
  }

  async unassignProgram(assignmentId: string): Promise<void> {
    for (const patient of this.state.patients) {
      if (patient.currentProgram?.assignmentId === assignmentId) {
        patient.currentProgram = null;
        return;
      }
    }
    throw new Error(`Assignment not found: ${assignmentId}`);
  }

  // --------------------------------------------------------------------------
  // Session Management
  // --------------------------------------------------------------------------

  async getSessionHistory(
    patientId: string,
    dateRange?: DateRange
  ): Promise<SessionSummary[]> {
    const patient = await this.getPatient(patientId);
    if (!patient) {
      return [];
    }

    let sessions = [...patient.sessions];

    if (dateRange) {
      sessions = sessions.filter(s =>
        s.date >= dateRange.start && s.date <= dateRange.end
      );
    }

    return sessions.sort((a, b) => b.date.getTime() - a.date.getTime());
  }

  async scheduleSession(
    patientId: string,
    session: Partial<ScheduledSession>
  ): Promise<ScheduledSession> {
    const patient = await this.getPatient(patientId);
    if (!patient) {
      throw new Error(`Patient not found: ${patientId}`);
    }

    const scheduled: ScheduledSession = {
      scheduleId: this.generateId('SCH'),
      patientId,
      patientName: `${patient.profile.lastName} ${patient.profile.firstName}`,
      therapistId: this.state.currentUser.userId,
      scheduledTime: session.scheduledTime || new Date(),
      duration: session.duration || 60,
      sessionType: session.sessionType || 'regular',
      notes: session.notes,
      status: 'scheduled',
    };

    this.state.schedule.push(scheduled);
    return scheduled;
  }

  async cancelSession(scheduleId: string, reason: string): Promise<void> {
    const session = this.state.schedule.find(s => s.scheduleId === scheduleId);
    if (!session) {
      throw new Error(`Scheduled session not found: ${scheduleId}`);
    }

    session.status = 'cancelled';
    session.notes = `${session.notes || ''}\nCancellation reason: ${reason}`;
  }

  // --------------------------------------------------------------------------
  // Progress Tracking
  // --------------------------------------------------------------------------

  async getProgress(patientId: string, dateRange?: DateRange): Promise<ProgressData> {
    const patient = await this.getPatient(patientId);
    if (!patient) {
      throw new Error(`Patient not found: ${patientId}`);
    }

    const range = dateRange || this.state.filters.dateRange;

    // Calculate progress from session data
    const summary = this.calculateProgressSummary(patient, range);
    const metrics = this.calculateMetricProgress(patient, range);
    const milestones = await this.getMilestones(patientId);

    return {
      patientId,
      dateRange: range,
      summary,
      metrics,
      milestones,
      trends: [], // Would calculate trend lines
    };
  }

  private calculateProgressSummary(patient: Patient, range: DateRange): ProgressSummary {
    const sessions = patient.sessions.filter(
      s => s.date >= range.start && s.date <= range.end
    );

    if (sessions.length === 0) {
      return {
        overallProgress: 0,
        weeklyChange: 0,
        trend: 'stable',
        projectedCompletion: null,
        keyAchievements: [],
        areasForFocus: [],
      };
    }

    const overallProgress = patient.currentProgram?.progressPercent || 0;

    // Calculate weekly change
    const oneWeekAgo = new Date(range.end.getTime() - 7 * 24 * 60 * 60 * 1000);
    const recentSessions = sessions.filter(s => s.date >= oneWeekAgo);
    const olderSessions = sessions.filter(s => s.date < oneWeekAgo);

    const recentAvg = recentSessions.length > 0
      ? recentSessions.reduce((sum, s) => sum + s.metrics.walkingSpeed, 0) / recentSessions.length
      : 0;
    const olderAvg = olderSessions.length > 0
      ? olderSessions.reduce((sum, s) => sum + s.metrics.walkingSpeed, 0) / olderSessions.length
      : recentAvg;

    const weeklyChange = olderAvg > 0 ? ((recentAvg - olderAvg) / olderAvg) * 100 : 0;

    return {
      overallProgress,
      weeklyChange,
      trend: weeklyChange > 2 ? 'improving' : weeklyChange < -2 ? 'declining' : 'stable',
      projectedCompletion: patient.currentProgram?.endDate || null,
      keyAchievements: this.identifyAchievements(patient, sessions),
      areasForFocus: this.identifyAreasForFocus(patient, sessions),
    };
  }

  private calculateMetricProgress(patient: Patient, range: DateRange): MetricProgress[] {
    const sessions = patient.sessions.filter(
      s => s.date >= range.start && s.date <= range.end
    );

    if (sessions.length === 0) {
      return [];
    }

    const firstSession = sessions[sessions.length - 1];
    const lastSession = sessions[0];

    return [
      {
        metricId: 'walking_speed',
        name: 'Walking Speed',
        nameKorean: '보행 속도',
        baseline: firstSession.metrics.walkingSpeed,
        current: lastSession.metrics.walkingSpeed,
        target: 0.8,
        unit: 'm/s',
        percentChange: this.calculatePercentChange(
          firstSession.metrics.walkingSpeed,
          lastSession.metrics.walkingSpeed
        ),
        trend: lastSession.metrics.walkingSpeed > firstSession.metrics.walkingSpeed ? 'up' : 'stable',
        history: sessions.map(s => ({
          timestamp: s.date,
          value: s.metrics.walkingSpeed,
        })),
      },
      {
        metricId: 'walking_distance',
        name: 'Walking Distance',
        nameKorean: '보행 거리',
        baseline: firstSession.metrics.walkingDistance,
        current: lastSession.metrics.walkingDistance,
        target: 200,
        unit: 'm',
        percentChange: this.calculatePercentChange(
          firstSession.metrics.walkingDistance,
          lastSession.metrics.walkingDistance
        ),
        trend: lastSession.metrics.walkingDistance > firstSession.metrics.walkingDistance ? 'up' : 'stable',
        history: sessions.map(s => ({
          timestamp: s.date,
          value: s.metrics.walkingDistance,
        })),
      },
      {
        metricId: 'assistance_level',
        name: 'Assistance Level',
        nameKorean: '보조 수준',
        baseline: firstSession.metrics.assistanceLevel,
        current: lastSession.metrics.assistanceLevel,
        target: 20,
        unit: '%',
        percentChange: this.calculatePercentChange(
          firstSession.metrics.assistanceLevel,
          lastSession.metrics.assistanceLevel
        ),
        trend: lastSession.metrics.assistanceLevel < firstSession.metrics.assistanceLevel ? 'down' : 'stable',
        history: sessions.map(s => ({
          timestamp: s.date,
          value: s.metrics.assistanceLevel,
        })),
      },
    ];
  }

  private calculatePercentChange(baseline: number, current: number): number {
    if (baseline === 0) return 0;
    return ((current - baseline) / baseline) * 100;
  }

  private identifyAchievements(patient: Patient, sessions: SessionSummary[]): string[] {
    const achievements: string[] = [];

    if (sessions.length >= 5) {
      achievements.push('5개 이상 세션 완료');
    }

    const lastSession = sessions[0];
    if (lastSession && lastSession.metrics.walkingSpeed > 0.5) {
      achievements.push('보행 속도 0.5m/s 달성');
    }

    return achievements;
  }

  private identifyAreasForFocus(patient: Patient, sessions: SessionSummary[]): string[] {
    const areas: string[] = [];

    if (sessions.length > 0) {
      const avgSymmetry = sessions.reduce(
        (sum, s) => sum + s.metrics.gaitSymmetry, 0
      ) / sessions.length;

      if (avgSymmetry < 0.8) {
        areas.push('보행 대칭성 개선 필요');
      }
    }

    return areas;
  }

  async getMetricHistory(patientId: string, metricId: string): Promise<MetricProgress> {
    const progress = await this.getProgress(patientId);
    const metric = progress.metrics.find(m => m.metricId === metricId);
    if (!metric) {
      throw new Error(`Metric not found: ${metricId}`);
    }
    return metric;
  }

  async getMilestones(patientId: string): Promise<Milestone[]> {
    const patient = await this.getPatient(patientId);
    if (!patient) {
      return [];
    }

    // Generate milestones based on program goals
    const milestones: Milestone[] = [
      {
        milestoneId: 'ms-1',
        name: 'First 10 Minutes Walking',
        nameKorean: '첫 10분 보행',
        targetDate: new Date(),
        achieved: patient.sessions.some(s => s.metrics.walkingTime >= 10),
        metric: 'walking_time',
        targetValue: 10,
        currentValue: patient.sessions.length > 0
          ? Math.max(...patient.sessions.map(s => s.metrics.walkingTime))
          : 0,
      },
      {
        milestoneId: 'ms-2',
        name: 'Community Walking Speed',
        nameKorean: '지역사회 보행 속도',
        targetDate: new Date(Date.now() + 60 * 24 * 60 * 60 * 1000),
        achieved: patient.sessions.some(s => s.metrics.walkingSpeed >= 0.8),
        metric: 'walking_speed',
        targetValue: 0.8,
        currentValue: patient.sessions.length > 0
          ? Math.max(...patient.sessions.map(s => s.metrics.walkingSpeed))
          : 0,
      },
    ];

    return milestones;
  }

  // --------------------------------------------------------------------------
  // Assessments
  // --------------------------------------------------------------------------

  async recordAssessment(
    patientId: string,
    assessment: Partial<AssessmentRecord>
  ): Promise<AssessmentRecord> {
    const patient = await this.getPatient(patientId);
    if (!patient) {
      throw new Error(`Patient not found: ${patientId}`);
    }

    const newAssessment: AssessmentRecord = {
      assessmentId: this.generateId('ASM'),
      date: assessment.date || new Date(),
      type: assessment.type || AssessmentType.PROGRESS,
      assessorId: this.state.currentUser.userId,
      assessorName: this.state.currentUser.name,
      results: assessment.results || {},
      notes: assessment.notes || '',
    };

    patient.assessments.push(newAssessment);
    return newAssessment;
  }

  async getAssessmentHistory(patientId: string): Promise<AssessmentRecord[]> {
    const patient = await this.getPatient(patientId);
    if (!patient) {
      return [];
    }

    return patient.assessments.sort((a, b) => b.date.getTime() - a.date.getTime());
  }

  // --------------------------------------------------------------------------
  // Alerts
  // --------------------------------------------------------------------------

  async getAlerts(
    filters?: { patientId?: string; severity?: AlertSeverity[] }
  ): Promise<PatientAlert[]> {
    let alerts = [...this.state.alerts];

    if (filters?.patientId) {
      alerts = alerts.filter(a => a.patientId === filters.patientId);
    }

    if (filters?.severity && filters.severity.length > 0) {
      alerts = alerts.filter(a => filters.severity!.includes(a.severity));
    }

    return alerts.sort((a, b) => b.createdAt.getTime() - a.createdAt.getTime());
  }

  async acknowledgeAlert(alertId: string): Promise<void> {
    const alert = this.state.alerts.find(a => a.alertId === alertId);
    if (!alert) {
      throw new Error(`Alert not found: ${alertId}`);
    }

    alert.acknowledged = true;
    alert.acknowledgedBy = this.state.currentUser.userId;
    alert.acknowledgedAt = new Date();
  }

  // --------------------------------------------------------------------------
  // Reports
  // --------------------------------------------------------------------------

  async generateProgressReport(
    patientId: string,
    period: DateRange
  ): Promise<ProgressReport> {
    const patient = await this.getPatient(patientId);
    if (!patient) {
      throw new Error(`Patient not found: ${patientId}`);
    }

    const progress = await this.getProgress(patientId, period);
    const sessions = await this.getSessionHistory(patientId, period);
    const assessments = await this.getAssessmentHistory(patientId);

    const report: ProgressReport = {
      reportId: this.generateId('RPT'),
      patientId,
      generatedAt: new Date(),
      generatedBy: this.state.currentUser.name,
      period,
      executiveSummary: {
        status: this.determineStatus(progress.summary.overallProgress),
        overallProgress: progress.summary.overallProgress,
        keyHighlights: progress.summary.keyAchievements,
        concerns: progress.summary.areasForFocus,
      },
      sessionSummary: {
        sessionsCompleted: sessions.filter(s => s.status === SessionStatus.COMPLETED).length,
        sessionsScheduled: sessions.length,
        attendanceRate: this.calculateAttendanceRate(sessions),
        totalTrainingTime: sessions.reduce((sum, s) => sum + s.duration, 0),
        averageSessionDuration: sessions.length > 0
          ? sessions.reduce((sum, s) => sum + s.duration, 0) / sessions.length
          : 0,
      },
      metrics: {
        categories: [
          {
            category: 'Functional',
            categoryKorean: '기능적',
            metrics: progress.metrics,
            overallChange: progress.metrics.reduce((sum, m) => sum + m.percentChange, 0) / progress.metrics.length,
          },
        ],
        highlights: [],
      },
      assessments: {
        assessments: assessments.filter(
          a => a.date >= period.start && a.date <= period.end
        ),
        changes: [],
      },
      goals: {
        goals: progress.milestones.map(m => ({
          goalId: m.milestoneId,
          description: m.name,
          descriptionKorean: m.nameKorean,
          baseline: 0,
          target: m.targetValue,
          current: m.currentValue,
          percentAchieved: (m.currentValue / m.targetValue) * 100,
          status: m.achieved ? 'achieved' : 'in_progress',
        })),
        overallAchievement: progress.milestones.filter(m => m.achieved).length /
          progress.milestones.length * 100,
      },
      recommendations: this.generateRecommendations(patient, progress),
      nextSteps: ['다음 평가 예정', '보조 수준 점진적 감소'],
    };

    return report;
  }

  private determineStatus(progress: number): 'on_track' | 'ahead' | 'behind' | 'complete' {
    if (progress >= 100) return 'complete';
    if (progress >= 80) return 'ahead';
    if (progress >= 50) return 'on_track';
    return 'behind';
  }

  private calculateAttendanceRate(sessions: SessionSummary[]): number {
    if (sessions.length === 0) return 0;
    const completed = sessions.filter(
      s => s.status === SessionStatus.COMPLETED
    ).length;
    return (completed / sessions.length) * 100;
  }

  private generateRecommendations(patient: Patient, progress: ProgressData): string[] {
    const recommendations: string[] = [];

    if (progress.summary.trend === 'declining') {
      recommendations.push('프로그램 난이도 조정 고려');
    }

    if (progress.summary.overallProgress > 80) {
      recommendations.push('고급 프로그램으로 전환 고려');
    }

    return recommendations;
  }

  async exportReport(reportId: string, format: 'pdf' | 'csv' | 'json'): Promise<Blob> {
    // In real implementation, this would generate the actual file
    const content = JSON.stringify({ reportId, format });
    return new Blob([content], { type: 'application/json' });
  }

  // --------------------------------------------------------------------------
  // Live Monitoring
  // --------------------------------------------------------------------------

  async getLiveSessions(): Promise<LiveSessionView[]> {
    return this.state.activeSessions;
  }

  async watchSession(sessionId: string): Promise<LiveSessionView> {
    const session = this.state.activeSessions.find(s => s.sessionId === sessionId);
    if (!session) {
      throw new Error(`Live session not found: ${sessionId}`);
    }
    return session;
  }

  async sendAlert(sessionId: string, message: string): Promise<void> {
    const session = this.state.activeSessions.find(s => s.sessionId === sessionId);
    if (!session) {
      throw new Error(`Live session not found: ${sessionId}`);
    }

    session.alerts.push({
      alertId: this.generateId('ALT'),
      type: 'PATIENT_REQUEST' as any,
      severity: AlertSeverity.INFO,
      message,
      timestamp: new Date(),
      acknowledged: false,
    });
  }

  // --------------------------------------------------------------------------
  // Utility Methods
  // --------------------------------------------------------------------------

  private generateId(prefix: string): string {
    const timestamp = Date.now().toString(36);
    const random = Math.random().toString(36).substring(2, 8);
    return `${prefix}-${timestamp}-${random}`;
  }

  // --------------------------------------------------------------------------
  // State Management
  // --------------------------------------------------------------------------

  getState(): DashboardState {
    return { ...this.state };
  }

  setView(view: DashboardView): void {
    this.state.view = view;
  }

  setFilters(filters: Partial<DashboardFilters>): void {
    this.state.filters = { ...this.state.filters, ...filters };
  }
}

// ============================================================================
// Factory Function
// ============================================================================

export function createDashboard(
  user: TherapistUser,
  apiEndpoint?: string
): ITherapistDashboard {
  return new TherapistDashboard(user, apiEndpoint);
}
