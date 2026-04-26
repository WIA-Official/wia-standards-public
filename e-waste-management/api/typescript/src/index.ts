/**
 * WIA-ENE-025: E-Waste Management Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import type {
  ApiResponse,
  DeviceRegistrationRequest,
  DeviceRegistrationResponse,
  DeviceTracking,
  ProcessingSubmissionRequest,
  ProcessingSubmissionResponse,
  FacilityMetrics,
  RegionalStatistics,
  MRVReport,
  MRVReportType,
  DeviceEntry,
  ProcessingRecord,
  Timestamp,
} from './types';

/**
 * Client configuration
 */
export interface EWasteClientConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  headers?: Record<string, string>;
}

/**
 * Default configuration
 */
const DEFAULT_CONFIG = {
  endpoint: 'https://api.wia.org/ene-025/v1',
  timeout: 30000,
};

/**
 * E-Waste Management Client
 *
 * @example
 * ```typescript
 * const client = new EWasteClient({
 *   apiKey: 'your-api-key',
 *   endpoint: 'https://api.wia.org/ene-025/v1'
 * });
 *
 * // Register a device
 * const device = await client.registerDevice({
 *   device: {
 *     category: 'IT_EQUIPMENT',
 *     type: 'SMARTPHONE',
 *     brand: 'Samsung',
 *     model: 'Galaxy S23',
 *     weight_kg: 0.168,
 *     condition: 'NON_FUNCTIONAL'
 *   },
 *   owner: {
 *     type: 'INDIVIDUAL',
 *     country: 'KR'
 *   },
 *   collectionPoint: {
 *     facilityId: 'CP-SEL-042',
 *     name: '강남 재활용센터',
 *     location: { lat: 37.4979, lon: 127.0276 }
 *   }
 * });
 *
 * console.log('Device ID:', device.deviceId);
 * console.log('Tracking URL:', device.trackingUrl);
 * ```
 */
export class EWasteClient {
  private config: Required<EWasteClientConfig>;

  constructor(config: EWasteClientConfig) {
    this.config = {
      ...DEFAULT_CONFIG,
      ...config,
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${config.apiKey}`,
        ...config.headers,
      },
    };
  }

  // ============================================================================
  // Device Registration
  // ============================================================================

  /**
   * Register a new e-waste device
   *
   * @param request - Device registration information
   * @returns Device registration response with ID and tracking URL
   */
  async registerDevice(
    request: DeviceRegistrationRequest
  ): Promise<DeviceRegistrationResponse> {
    const response = await this.post<DeviceRegistrationResponse>(
      '/devices/register',
      request
    );
    return response;
  }

  /**
   * Get device information by ID
   *
   * @param deviceId - Device ID
   * @returns Device entry information
   */
  async getDevice(deviceId: string): Promise<DeviceEntry> {
    const response = await this.get<DeviceEntry>(`/devices/${deviceId}`);
    return response;
  }

  /**
   * Track device status and history
   *
   * @param deviceId - Device ID
   * @returns Device tracking information
   */
  async trackDevice(deviceId: string): Promise<DeviceTracking> {
    const response = await this.get<DeviceTracking>(
      `/devices/${deviceId}/tracking`
    );
    return response;
  }

  // ============================================================================
  // Processing
  // ============================================================================

  /**
   * Submit processing record for a device
   *
   * @param request - Processing information
   * @returns Processing submission response with certificate
   */
  async submitProcessing(
    request: ProcessingSubmissionRequest
  ): Promise<ProcessingSubmissionResponse> {
    const response = await this.post<ProcessingSubmissionResponse>(
      '/processing/submit',
      request
    );
    return response;
  }

  /**
   * Get processing record by ID
   *
   * @param processingId - Processing record ID
   * @returns Processing record
   */
  async getProcessingRecord(processingId: string): Promise<ProcessingRecord> {
    const response = await this.get<ProcessingRecord>(
      `/processing/${processingId}`
    );
    return response;
  }

  // ============================================================================
  // Facility Monitoring
  // ============================================================================

  /**
   * Get real-time facility metrics
   *
   * @param facilityId - Facility ID
   * @returns Facility metrics
   */
  async getFacilityMetrics(facilityId: string): Promise<FacilityMetrics> {
    const response = await this.get<FacilityMetrics>(
      `/facilities/${facilityId}/metrics`
    );
    return response;
  }

  /**
   * Subscribe to real-time facility monitoring via WebSocket
   *
   * @param facilityId - Facility ID
   * @param onMessage - Callback for incoming metrics
   * @returns WebSocket connection
   */
  subscribeToFacility(
    facilityId: string,
    onMessage: (metrics: FacilityMetrics) => void
  ): WebSocket {
    const wsEndpoint = this.config.endpoint
      .replace('http://', 'ws://')
      .replace('https://', 'wss://');
    const ws = new WebSocket(
      `${wsEndpoint}/facilities/${facilityId}/monitor`
    );

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      onMessage(data);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return ws;
  }

  // ============================================================================
  // Statistics & Reporting
  // ============================================================================

  /**
   * Get regional statistics
   *
   * @param region - Region code
   * @param startDate - Start date (ISO 8601)
   * @param endDate - End date (ISO 8601)
   * @returns Regional statistics
   */
  async getRegionalStatistics(
    region: string,
    startDate: Timestamp,
    endDate: Timestamp
  ): Promise<RegionalStatistics> {
    const response = await this.get<RegionalStatistics>(
      `/statistics/regional?region=${region}&start=${startDate}&end=${endDate}`
    );
    return response;
  }

  /**
   * Generate MRV report
   *
   * @param reportType - Report type (daily, weekly, monthly, etc.)
   * @param startDate - Start date (ISO 8601)
   * @param endDate - End date (ISO 8601)
   * @param facilityId - Optional facility ID filter
   * @returns MRV report
   */
  async generateMRVReport(
    reportType: MRVReportType,
    startDate: Timestamp,
    endDate: Timestamp,
    facilityId?: string
  ): Promise<MRVReport> {
    const params = new URLSearchParams({
      type: reportType,
      start: startDate,
      end: endDate,
    });
    if (facilityId) {
      params.append('facilityId', facilityId);
    }

    const response = await this.get<MRVReport>(
      `/mrv/report?${params.toString()}`
    );
    return response;
  }

  /**
   * Submit report for third-party verification
   *
   * @param reportId - Report ID
   * @returns Verification submission response
   */
  async submitForVerification(reportId: string): Promise<{
    verificationId: string;
    status: string;
    estimatedCompletion: Timestamp;
  }> {
    const response = await this.post<{
      verificationId: string;
      status: string;
      estimatedCompletion: Timestamp;
    }>('/verification/submit', { reportId });
    return response;
  }

  // ============================================================================
  // Utilities
  // ============================================================================

  /**
   * Calculate estimated material value for a device
   *
   * @param deviceType - Device type
   * @param weight_kg - Device weight in kg
   * @returns Estimated value in USD
   */
  async calculateEstimatedValue(
    deviceType: string,
    weight_kg: number
  ): Promise<{
    gold_g: number;
    silver_g: number;
    copper_g: number;
    totalValue_USD: number;
  }> {
    const response = await this.post<{
      gold_g: number;
      silver_g: number;
      copper_g: number;
      totalValue_USD: number;
    }>('/utilities/estimate-value', { deviceType, weight_kg });
    return response;
  }

  /**
   * Find nearest collection point
   *
   * @param lat - Latitude
   * @param lon - Longitude
   * @param radius_km - Search radius in kilometers
   * @returns List of nearby collection points
   */
  async findNearbyCollectionPoints(
    lat: number,
    lon: number,
    radius_km: number = 10
  ): Promise<Array<{
    facilityId: string;
    name: string;
    distance_km: number;
    location: { lat: number; lon: number };
  }>> {
    const response = await this.get<Array<{
      facilityId: string;
      name: string;
      distance_km: number;
      location: { lat: number; lon: number };
    }>>(`/utilities/nearby?lat=${lat}&lon=${lon}&radius=${radius_km}`);
    return response;
  }

  // ============================================================================
  // HTTP Helpers
  // ============================================================================

  private async get<T>(path: string): Promise<T> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, body?: any): Promise<T> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body?: any): Promise<T> {
    return this.request<T>('PUT', path, body);
  }

  private async delete<T>(path: string): Promise<T> {
    return this.request<T>('DELETE', path);
  }

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<T> {
    const url = `${this.config.endpoint}${path}`;
    const options: RequestInit = {
      method,
      headers: this.config.headers,
    };

    if (body) {
      options.body = JSON.stringify(body);
    }

    try {
      const response = await fetch(url, options);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          errorData.error?.message || `HTTP ${response.status}: ${response.statusText}`
        );
      }

      const apiResponse: ApiResponse<T> = await response.json();

      if (!apiResponse.success) {
        throw new Error(
          apiResponse.error?.message || 'API request failed'
        );
      }

      return apiResponse.data as T;
    } catch (error) {
      if (error instanceof Error) {
        throw error;
      }
      throw new Error('Unknown error occurred');
    }
  }
}

// ============================================================================
// Export types
// ============================================================================

export * from './types';

// ============================================================================
// Re-export for convenience
// ============================================================================

export default EWasteClient;
