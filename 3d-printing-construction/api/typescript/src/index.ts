/**
 * WIA-CITY-008: 3D Printing Construction Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import {
  ApiResponse,
  ProjectRegistrationRequest,
  ProjectRegistrationResponse,
  PrintJobSubmissionRequest,
  PrintJobSubmissionResponse,
  ConstructionProject,
  PrintJob,
  PrinterSpec,
  MaterialBatch,
  StructuralTest,
  Inspection,
  ComplianceChecklist,
  ProjectStatistics,
  ProjectTimeline,
  CostTracking,
} from './types';

export * from './types';

// ============================================================================
// Client Configuration
// ============================================================================

/**
 * Client configuration options
 */
export interface City3DPrintClientConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  retries?: number;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  endpoint: 'https://api.wia.org/city-008/v1',
  timeout: 30000,
  retries: 3,
};

// ============================================================================
// 3D Printing Construction Client
// ============================================================================

/**
 * WIA-CITY-008 3D Printing Construction API Client
 */
export class City3DPrintClient {
  private config: Required<City3DPrintClientConfig>;

  constructor(config: City3DPrintClientConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
    };
  }

  /**
   * Make HTTP request
   */
  private async request<T>(
    method: string,
    path: string,
    data?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
    };

    try {
      const response = await fetch(url, {
        method,
        headers,
        body: data ? JSON.stringify(data) : undefined,
      });

      const result = await response.json();
      return result as ApiResponse<T>;
    } catch (error: any) {
      return {
        success: false,
        error: {
          code: 'REQUEST_FAILED',
          message: error.message,
          details: error,
        },
      };
    }
  }

  // ========================================================================
  // Project Management
  // ========================================================================

  /**
   * Register a new construction project
   */
  async registerProject(
    request: ProjectRegistrationRequest
  ): Promise<ApiResponse<ProjectRegistrationResponse>> {
    return this.request<ProjectRegistrationResponse>('POST', '/projects/register', request);
  }

  /**
   * Get project information
   */
  async getProject(projectId: string): Promise<ApiResponse<ConstructionProject>> {
    return this.request<ConstructionProject>('GET', `/projects/${projectId}`);
  }

  /**
   * Update project information
   */
  async updateProject(
    projectId: string,
    updates: Partial<ConstructionProject>
  ): Promise<ApiResponse<ConstructionProject>> {
    return this.request<ConstructionProject>('PUT', `/projects/${projectId}`, updates);
  }

  /**
   * List projects
   */
  async listProjects(filters?: {
    status?: string;
    location?: string;
    buildingType?: string;
    limit?: number;
    offset?: number;
  }): Promise<ApiResponse<ConstructionProject[]>> {
    const params = new URLSearchParams(filters as any);
    return this.request<ConstructionProject[]>('GET', `/projects?${params}`);
  }

  /**
   * Delete project
   */
  async deleteProject(projectId: string): Promise<ApiResponse<void>> {
    return this.request<void>('DELETE', `/projects/${projectId}`);
  }

  // ========================================================================
  // Print Job Management
  // ========================================================================

  /**
   * Submit print job
   */
  async submitPrintJob(
    request: PrintJobSubmissionRequest
  ): Promise<ApiResponse<PrintJobSubmissionResponse>> {
    return this.request<PrintJobSubmissionResponse>('POST', '/print-jobs/submit', request);
  }

  /**
   * Get print job status
   */
  async getPrintJob(jobId: string): Promise<ApiResponse<PrintJob>> {
    return this.request<PrintJob>('GET', `/print-jobs/${jobId}`);
  }

  /**
   * Pause print job
   */
  async pausePrintJob(jobId: string): Promise<ApiResponse<PrintJob>> {
    return this.request<PrintJob>('POST', `/print-jobs/${jobId}/pause`);
  }

  /**
   * Resume print job
   */
  async resumePrintJob(jobId: string): Promise<ApiResponse<PrintJob>> {
    return this.request<PrintJob>('POST', `/print-jobs/${jobId}/resume`);
  }

  /**
   * Cancel print job
   */
  async cancelPrintJob(jobId: string): Promise<ApiResponse<PrintJob>> {
    return this.request<PrintJob>('POST', `/print-jobs/${jobId}/cancel`);
  }

  // ========================================================================
  // Printer Management
  // ========================================================================

  /**
   * Register printer
   */
  async registerPrinter(printer: Omit<PrinterSpec, 'printerId'>): Promise<ApiResponse<PrinterSpec>> {
    return this.request<PrinterSpec>('POST', '/printers/register', printer);
  }

  /**
   * Get printer information
   */
  async getPrinter(printerId: string): Promise<ApiResponse<PrinterSpec>> {
    return this.request<PrinterSpec>('GET', `/printers/${printerId}`);
  }

  /**
   * List available printers
   */
  async listPrinters(filters?: {
    type?: string;
    status?: string;
    location?: string;
  }): Promise<ApiResponse<PrinterSpec[]>> {
    const params = new URLSearchParams(filters as any);
    return this.request<PrinterSpec[]>('GET', `/printers?${params}`);
  }

  /**
   * Update printer status
   */
  async updatePrinter(
    printerId: string,
    updates: Partial<PrinterSpec>
  ): Promise<ApiResponse<PrinterSpec>> {
    return this.request<PrinterSpec>('PUT', `/printers/${printerId}`, updates);
  }

  // ========================================================================
  // Material Management
  // ========================================================================

  /**
   * Register material batch
   */
  async registerMaterial(material: Omit<MaterialBatch, 'batchId'>): Promise<ApiResponse<MaterialBatch>> {
    return this.request<MaterialBatch>('POST', '/materials/register', material);
  }

  /**
   * Get material batch
   */
  async getMaterial(batchId: string): Promise<ApiResponse<MaterialBatch>> {
    return this.request<MaterialBatch>('GET', `/materials/${batchId}`);
  }

  /**
   * List materials
   */
  async listMaterials(filters?: {
    type?: string;
    supplier?: string;
    available?: boolean;
  }): Promise<ApiResponse<MaterialBatch[]>> {
    const params = new URLSearchParams(filters as any);
    return this.request<MaterialBatch[]>('GET', `/materials?${params}`);
  }

  // ========================================================================
  // Quality & Compliance
  // ========================================================================

  /**
   * Submit structural test result
   */
  async submitStructuralTest(
    test: Omit<StructuralTest, 'testId'>
  ): Promise<ApiResponse<StructuralTest>> {
    return this.request<StructuralTest>('POST', '/quality/structural-test', test);
  }

  /**
   * Get structural test results
   */
  async getStructuralTests(projectId: string): Promise<ApiResponse<StructuralTest[]>> {
    return this.request<StructuralTest[]>('GET', `/projects/${projectId}/structural-tests`);
  }

  /**
   * Submit inspection report
   */
  async submitInspection(
    inspection: Omit<Inspection, 'inspectionId'>
  ): Promise<ApiResponse<Inspection>> {
    return this.request<Inspection>('POST', '/quality/inspection', inspection);
  }

  /**
   * Get inspection reports
   */
  async getInspections(projectId: string): Promise<ApiResponse<Inspection[]>> {
    return this.request<Inspection[]>('GET', `/projects/${projectId}/inspections`);
  }

  /**
   * Get compliance checklist
   */
  async getComplianceChecklist(projectId: string): Promise<ApiResponse<ComplianceChecklist>> {
    return this.request<ComplianceChecklist>('GET', `/projects/${projectId}/compliance`);
  }

  /**
   * Update compliance checklist
   */
  async updateComplianceChecklist(
    projectId: string,
    checklist: Partial<ComplianceChecklist>
  ): Promise<ApiResponse<ComplianceChecklist>> {
    return this.request<ComplianceChecklist>('PUT', `/projects/${projectId}/compliance`, checklist);
  }

  // ========================================================================
  // Project Tracking
  // ========================================================================

  /**
   * Get project timeline
   */
  async getTimeline(projectId: string): Promise<ApiResponse<ProjectTimeline>> {
    return this.request<ProjectTimeline>('GET', `/projects/${projectId}/timeline`);
  }

  /**
   * Update project milestone
   */
  async updateMilestone(
    projectId: string,
    milestoneId: string,
    updates: any
  ): Promise<ApiResponse<ProjectTimeline>> {
    return this.request<ProjectTimeline>(
      'PUT',
      `/projects/${projectId}/milestones/${milestoneId}`,
      updates
    );
  }

  /**
   * Get cost tracking
   */
  async getCostTracking(projectId: string): Promise<ApiResponse<CostTracking>> {
    return this.request<CostTracking>('GET', `/projects/${projectId}/costs`);
  }

  /**
   * Update cost tracking
   */
  async updateCostTracking(
    projectId: string,
    costs: Partial<CostTracking>
  ): Promise<ApiResponse<CostTracking>> {
    return this.request<CostTracking>('PUT', `/projects/${projectId}/costs`, costs);
  }

  // ========================================================================
  // Statistics & Reporting
  // ========================================================================

  /**
   * Get project statistics
   */
  async getStatistics(filters: {
    startDate: string;
    endDate: string;
    location?: string;
    buildingType?: string;
  }): Promise<ApiResponse<ProjectStatistics>> {
    const params = new URLSearchParams(filters as any);
    return this.request<ProjectStatistics>('GET', `/statistics?${params}`);
  }

  /**
   * Get environmental impact report
   */
  async getEnvironmentalReport(projectId: string): Promise<ApiResponse<any>> {
    return this.request('GET', `/projects/${projectId}/environmental-report`);
  }

  // ========================================================================
  // Utilities
  // ========================================================================

  /**
   * Estimate project cost
   */
  async estimateProjectCost(design: any): Promise<ApiResponse<{
    material_cost_USD: number;
    labor_cost_USD: number;
    equipment_cost_USD: number;
    total_cost_USD: number;
    cost_per_m2_USD: number;
  }>> {
    return this.request('POST', '/utilities/estimate-cost', { design });
  }

  /**
   * Estimate print time
   */
  async estimatePrintTime(design: any, parameters: any): Promise<ApiResponse<{
    total_hours: number;
    total_layers: number;
    material_required_kg: number;
  }>> {
    return this.request('POST', '/utilities/estimate-time', { design, parameters });
  }

  /**
   * Validate building code compliance
   */
  async validateBuildingCode(projectId: string): Promise<ApiResponse<{
    compliant: boolean;
    violations: Array<{
      code: string;
      description: string;
      severity: string;
    }>;
  }>> {
    return this.request('POST', `/projects/${projectId}/validate-building-code`);
  }

  /**
   * Find nearby printers
   */
  async findNearbyPrinters(
    lat: number,
    lon: number,
    radius_km: number = 50
  ): Promise<ApiResponse<PrinterSpec[]>> {
    const params = new URLSearchParams({
      lat: lat.toString(),
      lon: lon.toString(),
      radius: radius_km.toString(),
    });
    return this.request<PrinterSpec[]>('GET', `/utilities/nearby-printers?${params}`);
  }
}

// ============================================================================
// WebSocket Client for Real-time Monitoring
// ============================================================================

/**
 * Real-time print job monitoring client
 */
export class PrintJobMonitor {
  private ws: WebSocket | null = null;
  private endpoint: string;
  private apiKey: string;
  private callbacks: Map<string, (data: any) => void> = new Map();

  constructor(config: { endpoint?: string; apiKey: string }) {
    this.endpoint = config.endpoint || 'wss://api.wia.org/city-008/v1/monitor';
    this.apiKey = config.apiKey;
  }

  /**
   * Connect to WebSocket
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(`${this.endpoint}?apiKey=${this.apiKey}`);

      this.ws.onopen = () => {
        console.log('Connected to print job monitor');
        resolve();
      };

      this.ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        reject(error);
      };

      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.handleMessage(data);
        } catch (error) {
          console.error('Failed to parse message:', error);
        }
      };
    });
  }

  /**
   * Subscribe to print job updates
   */
  subscribe(jobId: string, callback: (update: any) => void): void {
    this.callbacks.set(jobId, callback);
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({
        action: 'subscribe',
        jobId,
      }));
    }
  }

  /**
   * Unsubscribe from print job updates
   */
  unsubscribe(jobId: string): void {
    this.callbacks.delete(jobId);
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({
        action: 'unsubscribe',
        jobId,
      }));
    }
  }

  /**
   * Handle incoming message
   */
  private handleMessage(data: any): void {
    if (data.jobId && this.callbacks.has(data.jobId)) {
      const callback = this.callbacks.get(data.jobId);
      if (callback) {
        callback(data);
      }
    }
  }

  /**
   * Disconnect
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Calculate total project area
 */
export function calculateTotalArea(length_m: number, width_m: number, floors: number): number {
  return length_m * width_m * floors;
}

/**
 * Estimate material quantity
 */
export function estimateMaterialQuantity(
  wall_area_m2: number,
  wall_thickness_mm: number,
  material_density_kg_m3: number
): number {
  const volume_m3 = wall_area_m2 * (wall_thickness_mm / 1000);
  return volume_m3 * material_density_kg_m3;
}

/**
 * Calculate cost savings vs traditional construction
 */
export function calculateCostSavings(
  print_cost_USD: number,
  traditional_cost_USD: number
): {
  savings_USD: number;
  savings_percent: number;
} {
  const savings_USD = traditional_cost_USD - print_cost_USD;
  const savings_percent = (savings_USD / traditional_cost_USD) * 100;
  return { savings_USD, savings_percent };
}

/**
 * Calculate construction time reduction
 */
export function calculateTimeReduction(
  print_days: number,
  traditional_days: number
): {
  reduction_days: number;
  reduction_percent: number;
} {
  const reduction_days = traditional_days - print_days;
  const reduction_percent = (reduction_days / traditional_days) * 100;
  return { reduction_days, reduction_percent };
}

/**
 * Format duration in hours to readable string
 */
export function formatDuration(hours: number): string {
  const days = Math.floor(hours / 24);
  const remainingHours = Math.floor(hours % 24);
  const minutes = Math.floor((hours * 60) % 60);

  if (days > 0) {
    return `${days}d ${remainingHours}h ${minutes}m`;
  } else if (remainingHours > 0) {
    return `${remainingHours}h ${minutes}m`;
  } else {
    return `${minutes}m`;
  }
}

/**
 * Calculate material waste percentage
 */
export function calculateWastePercentage(used_kg: number, wasted_kg: number): number {
  return (wasted_kg / (used_kg + wasted_kg)) * 100;
}

// ============================================================================
// Export default client
// ============================================================================

export default City3DPrintClient;
