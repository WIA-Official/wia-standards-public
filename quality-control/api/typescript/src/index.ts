/**
 * WIA-IND-025: Quality Control Standard - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import type {
  QualityControlConfig,
  Inspection,
  InspectionType,
  Measurement,
  Defect,
  SPCData,
  SPCCalculationRequest,
  ControlChart,
  ControlChartType,
  NCR,
  CAPA,
  CalibrationRecord,
  Audit,
  QualityMetrics,
  APIResponse,
  PaginatedResponse,
  FilterOptions,
  DefectSeverity,
  Disposition,
} from './types';

export * from './types';

/**
 * WIA-IND-025 Quality Control SDK
 */
export class QualityControlSDK {
  private client: AxiosInstance;
  private config: QualityControlConfig;

  constructor(config: QualityControlConfig) {
    this.config = {
      apiEndpoint: 'https://api.wiastandards.com/v1/quality-control',
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.client = axios.create({
      baseURL: this.config.apiEndpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.config.apiKey}`,
        'X-WIA-Standard': 'IND-025',
        'X-WIA-Version': '1.0.0',
      },
    });

    // Request interceptor for debugging
    if (this.config.debug) {
      this.client.interceptors.request.use((request) => {
        console.log('Request:', request.method?.toUpperCase(), request.url);
        return request;
      });
    }

    // Response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      async (error) => {
        if (this.config.retries && error.config && !error.config.__retryCount) {
          error.config.__retryCount = 0;
        }

        if (
          error.config &&
          error.config.__retryCount < (this.config.retries || 0) &&
          error.response?.status >= 500
        ) {
          error.config.__retryCount++;
          await this.delay(1000 * error.config.__retryCount);
          return this.client(error.config);
        }

        return Promise.reject(error);
      }
    );
  }

  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  // ============================================================================
  // Inspection Methods
  // ============================================================================

  /**
   * Create a quality inspection
   */
  async createInspection(data: {
    inspectionType: InspectionType;
    productSku: string;
    batchNumber: string;
    quantity: number;
    inspector: {
      employeeId: string;
      name: string;
    };
    measurements?: Measurement[];
    defects?: Partial<Defect>[];
    lotNumber?: string;
    serialNumbers?: string[];
  }): Promise<Inspection> {
    const response = await this.client.post<APIResponse<Inspection>>(
      '/inspections',
      data
    );
    return response.data.data!;
  }

  /**
   * Get inspection by ID
   */
  async getInspection(inspectionId: string): Promise<Inspection> {
    const response = await this.client.get<APIResponse<Inspection>>(
      `/inspections/${inspectionId}`
    );
    return response.data.data!;
  }

  /**
   * List inspections with filters
   */
  async listInspections(
    filters?: FilterOptions
  ): Promise<PaginatedResponse<Inspection>> {
    const response = await this.client.get<
      APIResponse<PaginatedResponse<Inspection>>
    >('/inspections', { params: filters });
    return response.data.data!;
  }

  /**
   * Update inspection
   */
  async updateInspection(
    inspectionId: string,
    updates: Partial<Inspection>
  ): Promise<Inspection> {
    const response = await this.client.put<APIResponse<Inspection>>(
      `/inspections/${inspectionId}`,
      updates
    );
    return response.data.data!;
  }

  /**
   * Add measurement to inspection
   */
  async addMeasurement(
    inspectionId: string,
    measurement: Measurement
  ): Promise<Inspection> {
    const response = await this.client.post<APIResponse<Inspection>>(
      `/inspections/${inspectionId}/measurements`,
      measurement
    );
    return response.data.data!;
  }

  /**
   * Add defect to inspection
   */
  async addDefect(
    inspectionId: string,
    defect: Partial<Defect>
  ): Promise<Inspection> {
    const response = await this.client.post<APIResponse<Inspection>>(
      `/inspections/${inspectionId}/defects`,
      defect
    );
    return response.data.data!;
  }

  /**
   * Approve inspection
   */
  async approveInspection(
    inspectionId: string,
    approver: {
      employeeId: string;
      name: string;
      role: string;
    }
  ): Promise<Inspection> {
    const response = await this.client.post<APIResponse<Inspection>>(
      `/inspections/${inspectionId}/approve`,
      approver
    );
    return response.data.data!;
  }

  // ============================================================================
  // Statistical Process Control (SPC) Methods
  // ============================================================================

  /**
   * Calculate SPC metrics (Cp, Cpk, Sigma, DPMO)
   */
  async calculateSPC(request: SPCCalculationRequest): Promise<SPCData> {
    const response = await this.client.post<APIResponse<SPCData>>(
      '/spc/calculate',
      request
    );
    return response.data.data!;
  }

  /**
   * Create control chart
   */
  async createControlChart(data: {
    type: ControlChartType;
    characteristic: string;
    productSku?: string;
    process?: string;
    subgroupSize?: number;
    targetValue?: number;
  }): Promise<ControlChart> {
    const response = await this.client.post<APIResponse<ControlChart>>(
      '/spc/control-charts',
      data
    );
    return response.data.data!;
  }

  /**
   * Get control chart data
   */
  async getControlChart(chartId: string): Promise<ControlChart> {
    const response = await this.client.get<APIResponse<ControlChart>>(
      `/spc/control-charts/${chartId}`
    );
    return response.data.data!;
  }

  /**
   * Add data point to control chart
   */
  async addControlChartPoint(
    chartId: string,
    dataPoint: {
      value: number;
      timestamp?: Date | string;
      sampleNumber?: number;
    }
  ): Promise<ControlChart> {
    const response = await this.client.post<APIResponse<ControlChart>>(
      `/spc/control-charts/${chartId}/points`,
      dataPoint
    );
    return response.data.data!;
  }

  /**
   * Get process capability analysis
   */
  async getProcessCapability(data: {
    characteristic: string;
    productSku?: string;
    process?: string;
    period?: {
      start: Date | string;
      end: Date | string;
    };
  }): Promise<SPCData> {
    const response = await this.client.post<APIResponse<SPCData>>(
      '/spc/capability',
      data
    );
    return response.data.data!;
  }

  /**
   * Report out-of-control condition
   */
  async reportOutOfControl(data: {
    chartId: string;
    characteristic: string;
    value: number;
    rule: string;
    containmentAction: string;
  }): Promise<void> {
    await this.client.post('/spc/out-of-control', data);
  }

  // ============================================================================
  // Defect Detection Methods
  // ============================================================================

  /**
   * AI-powered defect detection from image
   */
  async detectDefects(data: {
    imageUrl: string;
    productModel: string;
    aiModel?: string;
    confidenceThreshold?: number;
  }): Promise<Defect[]> {
    const response = await this.client.post<APIResponse<Defect[]>>(
      '/defects/detect',
      {
        ...data,
        aiModel: data.aiModel || 'yolov8-defect-detection',
        confidenceThreshold: data.confidenceThreshold || 0.85,
      }
    );
    return response.data.data!;
  }

  /**
   * Classify defect type and severity
   */
  async classifyDefect(data: {
    description: string;
    imageUrl?: string;
    productModel?: string;
  }): Promise<{
    type: string;
    category: string;
    severity: DefectSeverity;
    confidence: number;
  }> {
    const response = await this.client.post<
      APIResponse<{
        type: string;
        category: string;
        severity: DefectSeverity;
        confidence: number;
      }>
    >('/defects/classify', data);
    return response.data.data!;
  }

  // ============================================================================
  // Non-Conformance Report (NCR) Methods
  // ============================================================================

  /**
   * Create Non-Conformance Report
   */
  async createNCR(data: {
    productSku: string;
    batchNumber: string;
    quantityAffected: number;
    issueDescription: string;
    severity: DefectSeverity;
    detectionPoint: string;
    reportedBy: {
      employeeId: string;
      name: string;
      department: string;
    };
    containmentAction?: {
      description: string;
      implementedBy: string;
    };
    disposition?: Disposition;
  }): Promise<NCR> {
    const response = await this.client.post<APIResponse<NCR>>('/ncr', data);
    return response.data.data!;
  }

  /**
   * Get NCR by ID
   */
  async getNCR(ncrId: string): Promise<NCR> {
    const response = await this.client.get<APIResponse<NCR>>(`/ncr/${ncrId}`);
    return response.data.data!;
  }

  /**
   * List NCRs with filters
   */
  async listNCRs(filters?: FilterOptions): Promise<PaginatedResponse<NCR>> {
    const response = await this.client.get<APIResponse<PaginatedResponse<NCR>>>(
      '/ncr',
      { params: filters }
    );
    return response.data.data!;
  }

  /**
   * Update NCR
   */
  async updateNCR(ncrId: string, updates: Partial<NCR>): Promise<NCR> {
    const response = await this.client.put<APIResponse<NCR>>(
      `/ncr/${ncrId}`,
      updates
    );
    return response.data.data!;
  }

  /**
   * Set NCR disposition
   */
  async setNCRDisposition(
    ncrId: string,
    disposition: {
      decision: Disposition;
      justification: string;
      approvedBy: string;
      reworkProcedure?: string;
    }
  ): Promise<NCR> {
    const response = await this.client.post<APIResponse<NCR>>(
      `/ncr/${ncrId}/disposition`,
      disposition
    );
    return response.data.data!;
  }

  /**
   * Close NCR
   */
  async closeNCR(
    ncrId: string,
    data: {
      closedBy: string;
      verification?: string;
    }
  ): Promise<NCR> {
    const response = await this.client.post<APIResponse<NCR>>(
      `/ncr/${ncrId}/close`,
      data
    );
    return response.data.data!;
  }

  // ============================================================================
  // CAPA (Corrective and Preventive Action) Methods
  // ============================================================================

  /**
   * Create CAPA
   */
  async createCAPA(data: {
    type: 'corrective' | 'preventive' | 'both';
    problemStatement: string;
    source: string;
    ncrId?: string;
    initiator: {
      employeeId: string;
      name: string;
      department: string;
    };
    rootCauseAnalysis?: {
      method: string;
      findings: string[];
      rootCause: string;
    };
    correctiveAction?: {
      description: string;
      assignedTo: string;
      dueDate: Date | string;
    };
    preventiveAction?: {
      description: string;
      assignedTo: string;
      dueDate: Date | string;
    };
  }): Promise<CAPA> {
    const response = await this.client.post<APIResponse<CAPA>>('/capa', data);
    return response.data.data!;
  }

  /**
   * Get CAPA by ID
   */
  async getCAPA(capaId: string): Promise<CAPA> {
    const response = await this.client.get<APIResponse<CAPA>>(
      `/capa/${capaId}`
    );
    return response.data.data!;
  }

  /**
   * List CAPAs with filters
   */
  async listCAPAs(filters?: FilterOptions): Promise<PaginatedResponse<CAPA>> {
    const response = await this.client.get<
      APIResponse<PaginatedResponse<CAPA>>
    >('/capa', { params: filters });
    return response.data.data!;
  }

  /**
   * Update CAPA
   */
  async updateCAPA(capaId: string, updates: Partial<CAPA>): Promise<CAPA> {
    const response = await this.client.put<APIResponse<CAPA>>(
      `/capa/${capaId}`,
      updates
    );
    return response.data.data!;
  }

  /**
   * Verify CAPA implementation
   */
  async verifyCAPA(
    capaId: string,
    verification: {
      method: string;
      responsible: string;
      verified: boolean;
      comments?: string;
      evidence?: string[];
    }
  ): Promise<CAPA> {
    const response = await this.client.post<APIResponse<CAPA>>(
      `/capa/${capaId}/verify`,
      verification
    );
    return response.data.data!;
  }

  /**
   * Check CAPA effectiveness
   */
  async checkEffectiveness(
    capaId: string,
    data: {
      effective: boolean;
      metricsData?: Array<{
        metric: string;
        before: number;
        after: number;
        improvement: number;
      }>;
      conclusion?: string;
    }
  ): Promise<CAPA> {
    const response = await this.client.post<APIResponse<CAPA>>(
      `/capa/${capaId}/effectiveness`,
      data
    );
    return response.data.data!;
  }

  /**
   * Close CAPA
   */
  async closeCAPA(
    capaId: string,
    data: {
      closedBy: string;
      lessonsLearned?: string;
    }
  ): Promise<CAPA> {
    const response = await this.client.post<APIResponse<CAPA>>(
      `/capa/${capaId}/close`,
      data
    );
    return response.data.data!;
  }

  // ============================================================================
  // Calibration Methods
  // ============================================================================

  /**
   * Create calibration record
   */
  async createCalibration(data: {
    equipmentId: string;
    calibrationDate: Date | string;
    interval: number;
    calibratedBy: string;
    standardsUsed: Array<{
      standardId: string;
      description: string;
      traceability: string;
    }>;
    asFoundCondition: any;
    asLeftCondition: any;
  }): Promise<CalibrationRecord> {
    const response = await this.client.post<APIResponse<CalibrationRecord>>(
      '/calibration',
      data
    );
    return response.data.data!;
  }

  /**
   * Get calibration record
   */
  async getCalibration(calibrationId: string): Promise<CalibrationRecord> {
    const response = await this.client.get<APIResponse<CalibrationRecord>>(
      `/calibration/${calibrationId}`
    );
    return response.data.data!;
  }

  /**
   * Schedule calibration
   */
  async scheduleCalibration(data: {
    equipmentId: string;
    equipmentType: string;
    calibrationStandard: string;
    interval: number;
    vendor?: string;
    criticalEquipment?: boolean;
  }): Promise<{
    equipmentId: string;
    nextDueDate: Date;
    status: string;
    scheduled: boolean;
  }> {
    const response = await this.client.post<
      APIResponse<{
        equipmentId: string;
        nextDueDate: Date;
        status: string;
        scheduled: boolean;
      }>
    >('/calibration/schedule', data);
    return response.data.data!;
  }

  /**
   * Get calibration status for equipment
   */
  async getCalibrationStatus(equipmentId: string): Promise<{
    equipmentId: string;
    current: string;
    nextDueDate: Date;
    daysRemaining: number;
    overdue: boolean;
  }> {
    const response = await this.client.get<
      APIResponse<{
        equipmentId: string;
        current: string;
        nextDueDate: Date;
        daysRemaining: number;
        overdue: boolean;
      }>
    >(`/calibration/status/${equipmentId}`);
    return response.data.data!;
  }

  /**
   * Get upcoming calibrations
   */
  async getUpcomingCalibrations(
    days: number = 30
  ): Promise<CalibrationRecord[]> {
    const response = await this.client.get<APIResponse<CalibrationRecord[]>>(
      '/calibration/due',
      { params: { days } }
    );
    return response.data.data!;
  }

  /**
   * Generate calibration certificate
   */
  async generateCalibrationCertificate(calibrationId: string): Promise<{
    certificateUrl: string;
    certificateNumber: string;
  }> {
    const response = await this.client.post<
      APIResponse<{ certificateUrl: string; certificateNumber: string }>
    >(`/calibration/${calibrationId}/certificate`);
    return response.data.data!;
  }

  // ============================================================================
  // Audit Methods
  // ============================================================================

  /**
   * Create audit
   */
  async createAudit(data: {
    type: string;
    standard: string;
    scope: string;
    auditDate: Date | string;
    leadAuditor: string;
    auditors: string[];
    auditee: {
      organization: string;
      location: string;
      representatives: string[];
    };
  }): Promise<Audit> {
    const response = await this.client.post<APIResponse<Audit>>(
      '/audits',
      data
    );
    return response.data.data!;
  }

  /**
   * Get audit by ID
   */
  async getAudit(auditId: string): Promise<Audit> {
    const response = await this.client.get<APIResponse<Audit>>(
      `/audits/${auditId}`
    );
    return response.data.data!;
  }

  /**
   * Add finding to audit
   */
  async addFinding(
    auditId: string,
    finding: {
      severity: string;
      clause: string;
      process: string;
      description: string;
      objectiveEvidence: string;
      requirement: string;
    }
  ): Promise<Audit> {
    const response = await this.client.post<APIResponse<Audit>>(
      `/audits/${auditId}/findings`,
      finding
    );
    return response.data.data!;
  }

  /**
   * Generate audit report
   */
  async generateAuditReport(auditId: string): Promise<{
    reportUrl: string;
    reportId: string;
  }> {
    const response = await this.client.post<
      APIResponse<{ reportUrl: string; reportId: string }>
    >(`/audits/${auditId}/report`);
    return response.data.data!;
  }

  /**
   * Plan audit schedule
   */
  async planAudit(data: {
    standard: string;
    auditType: string;
    scope: string;
    auditor: string;
    startDate: Date | string;
    duration: number;
    clauses?: string[];
  }): Promise<Audit> {
    const response = await this.client.post<APIResponse<Audit>>(
      '/audits/plan',
      data
    );
    return response.data.data!;
  }

  // ============================================================================
  // Metrics and Analytics Methods
  // ============================================================================

  /**
   * Get quality metrics dashboard
   */
  async getQualityMetrics(period?: {
    start: Date | string;
    end: Date | string;
  }): Promise<QualityMetrics> {
    const response = await this.client.get<APIResponse<QualityMetrics>>(
      '/metrics/dashboard',
      { params: period }
    );
    return response.data.data!;
  }

  /**
   * Get defect rate trend
   */
  async getDefectRateTrend(data: {
    productSku?: string;
    period: {
      start: Date | string;
      end: Date | string;
    };
    granularity?: 'day' | 'week' | 'month';
  }): Promise<Array<{ date: string; rate: number }>> {
    const response = await this.client.get<
      APIResponse<Array<{ date: string; rate: number }>>
    >('/metrics/defect-rate', { params: data });
    return response.data.data!;
  }

  /**
   * Get first pass yield
   */
  async getFirstPassYield(data: {
    productSku?: string;
    process?: string;
    period: {
      start: Date | string;
      end: Date | string;
    };
  }): Promise<{ fpy: number; trend: string }> {
    const response = await this.client.get<
      APIResponse<{ fpy: number; trend: string }>
    >('/metrics/fpy', { params: data });
    return response.data.data!;
  }

  /**
   * Get Cpk trend
   */
  async getCpkTrend(data: {
    characteristic: string;
    productSku?: string;
    period: {
      start: Date | string;
      end: Date | string;
    };
  }): Promise<Array<{ date: string; cpk: number }>> {
    const response = await this.client.get<
      APIResponse<Array<{ date: string; cpk: number }>>
    >('/metrics/cpk', { params: data });
    return response.data.data!;
  }

  /**
   * Get OEE metrics
   */
  async getOEE(data: {
    line?: string;
    period: {
      start: Date | string;
      end: Date | string;
    };
  }): Promise<{
    oee: number;
    availability: number;
    performance: number;
    quality: number;
  }> {
    const response = await this.client.get<
      APIResponse<{
        oee: number;
        availability: number;
        performance: number;
        quality: number;
      }>
    >('/metrics/oee', { params: data });
    return response.data.data!;
  }

  /**
   * Get cost of quality
   */
  async getCostOfQuality(period: {
    start: Date | string;
    end: Date | string;
  }): Promise<{
    totalCOQ: number;
    internalFailure: number;
    externalFailure: number;
    appraisal: number;
    prevention: number;
    percentage: number;
  }> {
    const response = await this.client.get<
      APIResponse<{
        totalCOQ: number;
        internalFailure: number;
        externalFailure: number;
        appraisal: number;
        prevention: number;
        percentage: number;
      }>
    >('/metrics/cost-of-quality', { params: period });
    return response.data.data!;
  }

  /**
   * Check ISO 9001 compliance
   */
  async checkCompliance(data: {
    standard: string;
    requirements?: string[];
  }): Promise<{
    compliant: boolean;
    score: number;
    gaps: Array<{
      requirement: string;
      status: string;
      action: string;
    }>;
  }> {
    const response = await this.client.post<
      APIResponse<{
        compliant: boolean;
        score: number;
        gaps: Array<{
          requirement: string;
          status: string;
          action: string;
        }>;
      }>
    >('/compliance/check', data);
    return response.data.data!;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate process capability (Cp)
 */
export function calculateCp(usl: number, lsl: number, stdDev: number): number {
  return (usl - lsl) / (6 * stdDev);
}

/**
 * Calculate process capability index (Cpk)
 */
export function calculateCpk(
  mean: number,
  usl: number,
  lsl: number,
  stdDev: number
): number {
  const cpu = (usl - mean) / (3 * stdDev);
  const cpl = (mean - lsl) / (3 * stdDev);
  return Math.min(cpu, cpl);
}

/**
 * Calculate Six Sigma level from Cpk
 */
export function calculateSigmaLevel(cpk: number): number {
  return cpk * 3;
}

/**
 * Calculate DPMO (Defects Per Million Opportunities)
 */
export function calculateDPMO(
  defects: number,
  opportunities: number
): number {
  return (defects / opportunities) * 1_000_000;
}

/**
 * Calculate First Pass Yield
 */
export function calculateFPY(passed: number, total: number): number {
  return (passed / total) * 100;
}

/**
 * Calculate Overall Equipment Effectiveness (OEE)
 */
export function calculateOEE(
  availability: number,
  performance: number,
  quality: number
): number {
  return availability * performance * quality;
}

/**
 * Determine process capability rating
 */
export function getCapabilityRating(cpk: number): string {
  if (cpk >= 1.67) return 'excellent';
  if (cpk >= 1.33) return 'capable';
  if (cpk >= 1.0) return 'marginal';
  return 'not-capable';
}

/**
 * Calculate control limits for X-bar chart
 */
export function calculateXbarLimits(
  grandMean: number,
  avgRange: number,
  subgroupSize: number
): { ucl: number; lcl: number; centerline: number } {
  const A2 = getA2Constant(subgroupSize);
  return {
    ucl: grandMean + A2 * avgRange,
    lcl: grandMean - A2 * avgRange,
    centerline: grandMean,
  };
}

/**
 * Get A2 constant for X-bar chart (subgroup size 2-10)
 */
function getA2Constant(n: number): number {
  const A2_TABLE: { [key: number]: number } = {
    2: 1.88,
    3: 1.023,
    4: 0.729,
    5: 0.577,
    6: 0.483,
    7: 0.419,
    8: 0.373,
    9: 0.337,
    10: 0.308,
  };
  return A2_TABLE[n] || 0;
}

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
