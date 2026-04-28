/**
 * WIA-ENE-026: Radioactive Waste Management Standard - SDK Implementation
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Client SDK for radioactive waste tracking and management
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import type {
  ClientConfig,
  RadioactiveWastePackage,
  CreatePackageRequest,
  CreatePackageResponse,
  UpdateLocationRequest,
  PackageQueryParams,
  MonitoringReading,
  MonitoringAlert,
  DoseCalculationRequest,
  DoseCalculationResponse,
  DecayCalculationRequest,
  DecayCalculationResponse,
  StorageFacility,
  FacilityInventory,
  PaginatedResponse,
  APIError,
} from './types';

export * from './types';

/**
 * Radioactive Waste Management Client
 */
export class RadioactiveWasteClient {
  private config: Required<ClientConfig>;
  private baseURL: string;

  constructor(config: ClientConfig) {
    this.config = {
      apiKey: config.apiKey,
      endpoint: config.endpoint || 'https://api.wia.org/ene-026/v1',
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      facility: config.facility,
    };
    this.baseURL = this.config.endpoint;
  }

  /**
   * Make HTTP request with retry logic
   */
  private async request<T>(
    method: string,
    path: string,
    body?: any,
    params?: Record<string, any>
  ): Promise<T> {
    const url = new URL(path, this.baseURL);

    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined && value !== null) {
          url.searchParams.append(key, String(value));
        }
      });
    }

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      'X-API-Key': this.config.apiKey,
      'User-Agent': 'WIA-ENE-026-SDK/1.0.0',
    };

    if (this.config.facility) {
      headers['X-Facility-ID'] = this.config.facility.id;
      headers['X-Facility-License'] = this.config.facility.license;
    }

    let lastError: Error | null = null;

    for (let attempt = 0; attempt <= this.config.retries; attempt++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

        const response = await fetch(url.toString(), {
          method,
          headers,
          body: body ? JSON.stringify(body) : undefined,
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        if (!response.ok) {
          const error: APIError = await response.json();
          throw new Error(`API Error (${response.status}): ${error.message}`);
        }

        return await response.json();
      } catch (error) {
        lastError = error as Error;
        if (attempt < this.config.retries) {
          // Exponential backoff
          await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        }
      }
    }

    throw lastError || new Error('Request failed');
  }

  // ==================== Package Management ====================

  /**
   * Register a new radioactive waste package
   */
  async registerPackage(request: CreatePackageRequest): Promise<CreatePackageResponse> {
    return this.request<CreatePackageResponse>('POST', '/packages', request);
  }

  /**
   * Get package details by ID
   */
  async getPackage(packageId: string): Promise<RadioactiveWastePackage> {
    return this.request<RadioactiveWastePackage>('GET', `/packages/${packageId}`);
  }

  /**
   * Query packages with filters
   */
  async queryPackages(params: PackageQueryParams): Promise<PaginatedResponse<RadioactiveWastePackage>> {
    return this.request<PaginatedResponse<RadioactiveWastePackage>>('GET', '/packages', undefined, params);
  }

  /**
   * Update package location
   */
  async updatePackageLocation(request: UpdateLocationRequest): Promise<void> {
    await this.request('PUT', `/packages/${request.packageId}/location`, request);
  }

  /**
   * Delete/dispose a package (mark as disposed)
   */
  async disposePackage(packageId: string, disposalInfo: any): Promise<void> {
    await this.request('DELETE', `/packages/${packageId}`, disposalInfo);
  }

  // ==================== Monitoring ====================

  /**
   * Submit monitoring reading
   */
  async submitMonitoringReading(reading: MonitoringReading): Promise<void> {
    await this.request('POST', '/monitoring/readings', reading);
  }

  /**
   * Get monitoring readings
   */
  async getMonitoringReadings(params: {
    sensorId?: string;
    packageId?: string;
    startTime?: string;
    endTime?: string;
    threshold?: number;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<MonitoringReading>> {
    return this.request<PaginatedResponse<MonitoringReading>>('GET', '/monitoring/readings', undefined, params);
  }

  /**
   * Get active alerts
   */
  async getAlerts(params: {
    severity?: 'info' | 'warning' | 'alarm' | 'emergency';
    status?: 'active' | 'acknowledged' | 'resolved';
    facilityId?: string;
    page?: number;
    limit?: number;
  }): Promise<PaginatedResponse<MonitoringAlert>> {
    return this.request<PaginatedResponse<MonitoringAlert>>('GET', '/monitoring/alerts', undefined, params);
  }

  /**
   * Acknowledge an alert
   */
  async acknowledgeAlert(alertId: string, acknowledgedBy: string): Promise<void> {
    await this.request('PUT', `/monitoring/alerts/${alertId}/acknowledge`, { acknowledgedBy });
  }

  // ==================== Dose Calculations ====================

  /**
   * Calculate radiation dose
   */
  async calculateDose(request: DoseCalculationRequest): Promise<DoseCalculationResponse> {
    return this.request<DoseCalculationResponse>('POST', '/dose/calculate', request);
  }

  /**
   * Calculate radioactive decay
   */
  async calculateDecay(request: DecayCalculationRequest): Promise<DecayCalculationResponse> {
    return this.request<DecayCalculationResponse>('POST', '/dose/decay', request);
  }

  /**
   * Calculate future activity for a package
   */
  async calculatePackageDecay(packageId: string, futureDate: string): Promise<DecayCalculationResponse[]> {
    return this.request<DecayCalculationResponse[]>('POST', `/packages/${packageId}/decay`, { futureDate });
  }

  // ==================== Facility Management ====================

  /**
   * Register a new storage facility
   */
  async registerFacility(facility: Omit<StorageFacility, 'facilityId'>): Promise<{ facilityId: string }> {
    return this.request<{ facilityId: string }>('POST', '/facilities', facility);
  }

  /**
   * Get facility details
   */
  async getFacility(facilityId: string): Promise<StorageFacility> {
    return this.request<StorageFacility>('GET', `/facilities/${facilityId}`);
  }

  /**
   * Get facility capacity
   */
  async getFacilityCapacity(facilityId: string): Promise<any> {
    return this.request('GET', `/facilities/${facilityId}/capacity`);
  }

  /**
   * Get facility inventory
   */
  async getFacilityInventory(facilityId: string): Promise<FacilityInventory> {
    return this.request<FacilityInventory>('GET', `/facilities/${facilityId}/inventory`);
  }

  /**
   * Update facility performance metrics
   */
  async updateFacilityPerformance(facilityId: string, performance: any): Promise<void> {
    await this.request('PUT', `/facilities/${facilityId}/performance`, performance);
  }

  // ==================== Search & Analytics ====================

  /**
   * Search packages by isotope
   */
  async searchByIsotope(isotope: string, params?: PackageQueryParams): Promise<PaginatedResponse<RadioactiveWastePackage>> {
    return this.queryPackages({ ...params, isotope });
  }

  /**
   * Get packages by waste class
   */
  async getPackagesByClass(wasteClass: string, params?: PackageQueryParams): Promise<PaginatedResponse<RadioactiveWastePackage>> {
    return this.queryPackages({ ...params, wasteClass: wasteClass as any });
  }

  /**
   * Get packages by facility
   */
  async getPackagesByFacility(facilityId: string, params?: PackageQueryParams): Promise<PaginatedResponse<RadioactiveWastePackage>> {
    return this.queryPackages({ ...params, facilityId });
  }

  // ==================== Utility Methods ====================

  /**
   * Calculate half-lives elapsed
   */
  calculateHalfLivesElapsed(halfLifeYears: number, timeElapsedYears: number): number {
    return timeElapsedYears / halfLifeYears;
  }

  /**
   * Calculate remaining activity after decay
   */
  calculateRemainingActivity(initialActivity: number, halfLifeYears: number, timeElapsedYears: number): number {
    const halfLivesElapsed = this.calculateHalfLivesElapsed(halfLifeYears, timeElapsedYears);
    return initialActivity * Math.pow(0.5, halfLivesElapsed);
  }

  /**
   * Convert activity units
   */
  convertActivity(value: number, fromUnit: string, toUnit: string): number {
    const toBq: Record<string, number> = {
      'Bq': 1,
      'kBq': 1e3,
      'MBq': 1e6,
      'GBq': 1e9,
      'TBq': 1e12,
      'Ci': 3.7e10,
      'mCi': 3.7e7,
      'µCi': 3.7e4,
    };

    const bqValue = value * (toBq[fromUnit] || 1);
    return bqValue / (toBq[toUnit] || 1);
  }

  /**
   * Convert dose units
   */
  convertDose(value: number, fromUnit: string, toUnit: string): number {
    const toSv: Record<string, number> = {
      'Sv': 1,
      'mSv': 1e-3,
      'µSv': 1e-6,
      'rem': 0.01,
      'mrem': 1e-5,
    };

    const svValue = value * (toSv[fromUnit] || 1);
    return svValue / (toSv[toUnit] || 1);
  }

  /**
   * Validate package data before submission
   */
  validatePackage(pkg: Partial<RadioactiveWastePackage>): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!pkg.wasteClass) {
      errors.push('Waste class is required');
    }

    if (!pkg.radiologicalData?.totalActivity) {
      errors.push('Total activity is required');
    }

    if (!pkg.containerInfo?.containerType) {
      errors.push('Container type is required');
    }

    if (pkg.radiologicalData?.isotopes?.length === 0) {
      errors.push('At least one isotope must be specified');
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }
}

/**
 * Create a new Radioactive Waste Management client instance
 */
export function createClient(config: ClientConfig): RadioactiveWasteClient {
  return new RadioactiveWasteClient(config);
}

/**
 * Default export
 */
export default RadioactiveWasteClient;
