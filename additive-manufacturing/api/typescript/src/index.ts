/**
 * WIA-IND-029: Additive Manufacturing - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';
import WebSocket from 'ws';
import * as Types from './types';

// Re-export all types
export * from './types';

/**
 * Main SDK class for Additive Manufacturing
 */
export class AdditiveManufacturingSDK {
  private config: Types.SDKConfig;
  private api: AxiosInstance;
  private ws?: WebSocket;

  constructor(config: Types.SDKConfig) {
    this.config = {
      apiEndpoint: 'http://localhost:8080',
      timeout: 30000,
      retries: 3,
      ...config,
    };

    this.api = axios.create({
      baseURL: this.config.apiEndpoint,
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        ...(this.config.apiKey && { 'Authorization': `Bearer ${this.config.apiKey}` }),
      },
    });
  }

  // ========================================================================
  // Model Management
  // ========================================================================

  /**
   * Upload a 3D model file
   */
  async uploadModel(upload: Types.ModelUpload): Promise<Types.Model> {
    const formData = new FormData();
    // In a real implementation, this would handle file upload
    const response = await this.api.post('/models/upload', {
      filePath: upload.filePath,
      format: upload.format,
      units: upload.units || 'mm',
      name: upload.name,
      description: upload.description,
    });
    return response.data;
  }

  /**
   * Get model by ID
   */
  async getModel(modelId: Types.ModelId): Promise<Types.Model> {
    const response = await this.api.get(`/models/${modelId}`);
    return response.data;
  }

  /**
   * List all models
   */
  async listModels(filters?: {
    format?: Types.FileFormat;
    searchTerm?: string;
    limit?: number;
    offset?: number;
  }): Promise<Types.Model[]> {
    const response = await this.api.get('/models', { params: filters });
    return response.data;
  }

  /**
   * Delete a model
   */
  async deleteModel(modelId: Types.ModelId): Promise<void> {
    await this.api.delete(`/models/${modelId}`);
  }

  // ========================================================================
  // Slicing Operations
  // ========================================================================

  /**
   * Slice a 3D model with specified profile
   */
  async sliceModel(params: {
    modelId: Types.ModelId;
    profile: Types.SlicingProfile | string;
    material: string;
    printer: Types.PrinterId;
  }): Promise<Types.SlicedModel> {
    const response = await this.api.post('/slicing/slice', params);
    return response.data;
  }

  /**
   * Get slicing profile by name
   */
  async getSlicingProfile(profileName: string): Promise<Types.SlicingProfile> {
    const response = await this.api.get(`/slicing/profiles/${profileName}`);
    return response.data;
  }

  /**
   * Create custom slicing profile
   */
  async createSlicingProfile(
    name: string,
    profile: Types.SlicingProfile
  ): Promise<void> {
    await this.api.post('/slicing/profiles', { name, ...profile });
  }

  /**
   * AI-powered slicing optimization
   */
  async optimizeSlicing(params: Types.AISlicingOptimization): Promise<Types.SlicedModel> {
    const response = await this.api.post('/slicing/optimize', params);
    return response.data;
  }

  /**
   * Generate support structures with AI
   */
  async generateSupports(params: {
    modelId: Types.ModelId;
    algorithm: 'tree-support-ai' | 'standard';
    overhangAngle: number;
    density: number;
    optimization?: 'minimal-contact' | 'easy-removal';
  }): Promise<any> {
    const response = await this.api.post('/slicing/supports', params);
    return response.data;
  }

  // ========================================================================
  // Print Job Management
  // ========================================================================

  /**
   * Submit a print job
   */
  async submitPrintJob(config: Types.PrintJobConfig): Promise<Types.PrintJob> {
    const response = await this.api.post('/jobs/submit', config);
    return response.data;
  }

  /**
   * Get print job status
   */
  async getPrintStatus(jobId: Types.JobId): Promise<Types.PrintJob> {
    const response = await this.api.get(`/jobs/${jobId}`);
    return response.data;
  }

  /**
   * Get real-time telemetry for a print job
   */
  async getTelemetry(jobId: Types.JobId): Promise<Types.PrintTelemetry> {
    const response = await this.api.get(`/jobs/${jobId}/telemetry`);
    return response.data;
  }

  /**
   * Pause a print job
   */
  async pauseJob(jobId: Types.JobId): Promise<void> {
    await this.api.post(`/jobs/${jobId}/pause`);
  }

  /**
   * Resume a paused print job
   */
  async resumeJob(jobId: Types.JobId): Promise<void> {
    await this.api.post(`/jobs/${jobId}/resume`);
  }

  /**
   * Cancel a print job
   */
  async cancelJob(jobId: Types.JobId, reason?: string): Promise<void> {
    await this.api.post(`/jobs/${jobId}/cancel`, { reason });
  }

  /**
   * List print jobs
   */
  async listJobs(filters?: {
    status?: Types.JobStatus;
    printerId?: Types.PrinterId;
    priority?: Types.JobPriority;
    limit?: number;
    offset?: number;
  }): Promise<Types.PrintJob[]> {
    const response = await this.api.get('/jobs', { params: filters });
    return response.data;
  }

  /**
   * Submit multi-material print job
   */
  async submitMultiMaterialJob(config: Types.MultiMaterialConfig): Promise<Types.PrintJob> {
    const response = await this.api.post('/jobs/multi-material', config);
    return response.data;
  }

  // ========================================================================
  // Printer Management
  // ========================================================================

  /**
   * Get printer by ID
   */
  async getPrinter(printerId: Types.PrinterId): Promise<Types.Printer> {
    const response = await this.api.get(`/printers/${printerId}`);
    return response.data;
  }

  /**
   * List all printers
   */
  async listPrinters(filters?: {
    status?: Types.PrinterStatus;
    technology?: Types.PrintingTechnology;
    location?: string;
  }): Promise<Types.Printer[]> {
    const response = await this.api.get('/printers', { params: filters });
    return response.data;
  }

  /**
   * Preheat printer
   */
  async preheatPrinter(
    printerId: Types.PrinterId,
    temperatures: {
      hotend?: number;
      bed?: number;
      chamber?: number;
    }
  ): Promise<void> {
    await this.api.post(`/printers/${printerId}/preheat`, temperatures);
  }

  /**
   * Home printer axes
   */
  async homePrinter(printerId: Types.PrinterId, axes?: ('x' | 'y' | 'z')[]): Promise<void> {
    await this.api.post(`/printers/${printerId}/home`, { axes });
  }

  /**
   * Move printer to position
   */
  async movePrinter(
    printerId: Types.PrinterId,
    position: Partial<Types.Vector3D>
  ): Promise<void> {
    await this.api.post(`/printers/${printerId}/move`, position);
  }

  // ========================================================================
  // Material Management
  // ========================================================================

  /**
   * Get material specification
   */
  async getMaterial(materialName: string): Promise<Types.Material> {
    const response = await this.api.get(`/materials/${materialName}`);
    return response.data;
  }

  /**
   * List all materials
   */
  async listMaterials(filters?: {
    category?: Types.MaterialCategory;
    technology?: Types.PrintingTechnology;
  }): Promise<Types.Material[]> {
    const response = await this.api.get('/materials', { params: filters });
    return response.data;
  }

  /**
   * Get material spool/inventory
   */
  async getMaterialSpool(spoolId: Types.SpoolId): Promise<Types.MaterialSpool> {
    const response = await this.api.get(`/spools/${spoolId}`);
    return response.data;
  }

  /**
   * List material spools
   */
  async listMaterialSpools(filters?: {
    material?: string;
    status?: 'available' | 'in-use' | 'low-stock' | 'empty';
    location?: string;
  }): Promise<Types.MaterialSpool[]> {
    const response = await this.api.get('/spools', { params: filters });
    return response.data;
  }

  /**
   * Add new material spool to inventory
   */
  async addMaterialSpool(spool: Omit<Types.MaterialSpool, 'spoolId'>): Promise<Types.MaterialSpool> {
    const response = await this.api.post('/spools', spool);
    return response.data;
  }

  /**
   * Update material spool
   */
  async updateMaterialSpool(
    spoolId: Types.SpoolId,
    updates: Partial<Types.MaterialSpool>
  ): Promise<Types.MaterialSpool> {
    const response = await this.api.patch(`/spools/${spoolId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Quality Assurance
  // ========================================================================

  /**
   * Perform quality inspection on printed part
   */
  async inspectPart(config: Types.InspectionConfig): Promise<Types.InspectionResult> {
    const response = await this.api.post('/quality/inspect', config);
    return response.data;
  }

  /**
   * Get inspection result
   */
  async getInspectionResult(inspectionId: string): Promise<Types.InspectionResult> {
    const response = await this.api.get(`/quality/inspections/${inspectionId}`);
    return response.data;
  }

  /**
   * Comprehensive multi-type inspection
   */
  async comprehensiveInspection(params: {
    jobId: Types.JobId;
    inspections: Array<{
      type: Types.InspectionType;
      method?: string;
      criteria?: any;
    }>;
  }): Promise<Types.InspectionResult[]> {
    const response = await this.api.post('/quality/comprehensive', params);
    return response.data;
  }

  // ========================================================================
  // Print Farm Management
  // ========================================================================

  /**
   * Get print farm status
   */
  async getPrintFarm(farmId?: Types.FarmId): Promise<Types.PrintFarm> {
    const id = farmId || this.config.printFarmId;
    const response = await this.api.get(`/farms/${id}`);
    return response.data;
  }

  /**
   * Get farm status overview
   */
  async getFarmStatus(farmId?: Types.FarmId): Promise<Types.PrintFarm> {
    return this.getPrintFarm(farmId);
  }

  /**
   * Optimize farm schedule
   */
  async optimizeFarmSchedule(params: {
    jobs: Types.JobId[];
    printers: Types.PrinterId[];
    optimization: 'minimize-makespan' | 'maximize-utilization' | 'balance-load';
    constraints?: Types.LoadBalancingConfig['constraints'];
  }): Promise<Types.FarmOptimization> {
    const response = await this.api.post('/farms/optimize', params);
    return response.data;
  }

  /**
   * Get farm queue
   */
  async getFarmQueue(farmId?: Types.FarmId): Promise<Types.PrintJob[]> {
    const id = farmId || this.config.printFarmId;
    const response = await this.api.get(`/farms/${id}/queue`);
    return response.data;
  }

  // ========================================================================
  // Certification
  // ========================================================================

  /**
   * Generate certification for printed part
   */
  async generateCertification(
    config: Types.CertificationConfig
  ): Promise<Types.Certification> {
    const response = await this.api.post('/certification/generate', config);
    return response.data;
  }

  /**
   * Get certification record
   */
  async getCertification(certId: string): Promise<Types.Certification> {
    const response = await this.api.get(`/certification/${certId}`);
    return response.data;
  }

  /**
   * Verify certification
   */
  async verifyCertification(certId: string, blockchainHash?: string): Promise<boolean> {
    const response = await this.api.post(`/certification/${certId}/verify`, {
      blockchainHash,
    });
    return response.data.valid;
  }

  // ========================================================================
  // Real-Time Updates
  // ========================================================================

  /**
   * Subscribe to real-time events
   */
  subscribeToEvents(
    eventTypes: Types.EventType[],
    callback: (event: Types.Event) => void
  ): void {
    const wsUrl = this.config.apiEndpoint?.replace('http', 'ws') + '/events';
    this.ws = new WebSocket(wsUrl);

    this.ws.on('open', () => {
      this.ws?.send(JSON.stringify({ type: 'subscribe', events: eventTypes }));
    });

    this.ws.on('message', (data: string) => {
      const event = JSON.parse(data) as Types.Event;
      callback(event);
    });

    this.ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });
  }

  /**
   * Unsubscribe from real-time events
   */
  unsubscribeFromEvents(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = undefined;
    }
  }

  // ========================================================================
  // Analytics
  // ========================================================================

  /**
   * Get printer analytics
   */
  async getPrinterAnalytics(
    printerId: Types.PrinterId,
    period: '24h' | '7d' | '30d' | '90d'
  ): Promise<any> {
    const response = await this.api.get(`/analytics/printer/${printerId}`, {
      params: { period },
    });
    return response.data;
  }

  /**
   * Get material usage analytics
   */
  async getMaterialAnalytics(material: string, period: string): Promise<any> {
    const response = await this.api.get(`/analytics/material/${material}`, {
      params: { period },
    });
    return response.data;
  }

  /**
   * Get farm efficiency metrics
   */
  async getFarmAnalytics(farmId?: Types.FarmId, period?: string): Promise<any> {
    const id = farmId || this.config.printFarmId;
    const response = await this.api.get(`/analytics/farm/${id}`, {
      params: { period },
    });
    return response.data;
  }

  // ========================================================================
  // Post-Processing
  // ========================================================================

  /**
   * Schedule post-processing for a job
   */
  async schedulePostProcessing(
    jobId: Types.JobId,
    steps: Types.PostProcessingStep[]
  ): Promise<void> {
    await this.api.post(`/jobs/${jobId}/post-processing`, { steps });
  }

  /**
   * Get post-processing status
   */
  async getPostProcessingStatus(jobId: Types.JobId): Promise<any> {
    const response = await this.api.get(`/jobs/${jobId}/post-processing`);
    return response.data;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate OEE (Overall Equipment Effectiveness) for 3D printer
 */
export function calculateOEE(params: {
  plannedTime: number;      // seconds
  downtime: number;         // seconds
  targetOutput: number;     // parts
  actualOutput: number;     // parts
  goodParts: number;        // parts
}): {
  overall: number;
  availability: number;
  performance: number;
  quality: number;
} {
  const operatingTime = params.plannedTime - params.downtime;
  const availability = (operatingTime / params.plannedTime) * 100;

  const performance = (params.actualOutput / params.targetOutput) * 100;

  const quality = (params.goodParts / params.actualOutput) * 100;

  const overall = (availability * performance * quality) / 10000;

  return {
    overall: Math.round(overall * 100) / 100,
    availability: Math.round(availability * 100) / 100,
    performance: Math.round(performance * 100) / 100,
    quality: Math.round(quality * 100) / 100,
  };
}

/**
 * Estimate print time based on model volume and settings
 */
export function estimatePrintTime(params: {
  volume: number;           // mm³
  layerHeight: number;      // mm
  printSpeed: number;       // mm/s
  infillDensity: number;    // percentage
}): number {
  // Simplified estimation formula
  const layers = params.volume / (100 * 100 * params.layerHeight);
  const timePerLayer = (100 / params.printSpeed) * (1 + params.infillDensity / 100);
  return layers * timePerLayer;
}

/**
 * Estimate material usage
 */
export function estimateMaterialUsage(params: {
  volume: number;           // mm³
  infillDensity: number;    // percentage
  wallThickness: number;    // mm
  density: number;          // g/cm³
}): number {
  // Simplified estimation
  const solidVolume = params.volume * (params.infillDensity / 100);
  const volumeCm3 = solidVolume / 1000;
  return volumeCm3 * params.density;
}

/**
 * Calculate support material percentage
 */
export function calculateSupportPercentage(params: {
  modelVolume: number;
  supportVolume: number;
}): number {
  return (params.supportVolume / params.modelVolume) * 100;
}

/**
 * Validate slicing profile
 */
export function validateSlicingProfile(profile: Types.SlicingProfile): {
  valid: boolean;
  errors: string[];
} {
  const errors: string[] = [];

  if (profile.layerHeight <= 0 || profile.layerHeight > 1) {
    errors.push('Layer height must be between 0 and 1mm');
  }

  if (profile.infillDensity < 0 || profile.infillDensity > 100) {
    errors.push('Infill density must be between 0 and 100%');
  }

  if (profile.printSpeed <= 0 || profile.printSpeed > 500) {
    errors.push('Print speed must be between 0 and 500 mm/s');
  }

  if (profile.wallThickness <= 0) {
    errors.push('Wall thickness must be greater than 0');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Convert units
 */
export function convertUnits(
  value: number,
  from: Types.Units,
  to: Types.Units
): number {
  const toMm: Record<Types.Units, number> = {
    mm: 1,
    cm: 10,
    inches: 25.4,
  };

  return (value * toMm[from]) / toMm[to];
}

/**
 * 弘익人間 (Benefit All Humanity)
 */
