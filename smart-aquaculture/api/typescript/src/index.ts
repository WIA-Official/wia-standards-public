/**
 * WIA Smart Aquaculture Standard - TypeScript SDK
 * @version 1.0.0
 * @license MIT
 */

import {
  AquacultureFarm,
  WaterQualitySensorData,
  FishHealthBiomass,
  FeedingSystemData,
  HarvestRecord,
  ApiResponse,
  AquacultureQuery,
} from './types';

/**
 * Configuration options for the Smart Aquaculture API client
 */
export interface SmartAquacultureConfig {
  /**
   * Base URL for the API endpoint
   */
  baseUrl: string;

  /**
   * API key for authentication
   */
  apiKey?: string;

  /**
   * Request timeout in milliseconds
   * @default 30000
   */
  timeout?: number;
}

/**
 * Smart Aquaculture API Client
 *
 * Provides methods to interact with the WIA Smart Aquaculture standard,
 * enabling water quality monitoring, fish health tracking, feeding automation,
 * and sustainable seafood production management.
 *
 * @example
 * ```typescript
 * const client = new SmartAquacultureClient({
 *   baseUrl: 'https://api.example.com/aquaculture',
 *   apiKey: 'your-api-key'
 * });
 *
 * // Get farm information
 * const farm = await client.getFarm('farm-uuid-123');
 * console.log(farm.data);
 * ```
 */
export class SmartAquacultureClient {
  private config: SmartAquacultureConfig;
  private headers: Record<string, string>;

  /**
   * Creates a new Smart Aquaculture API client
   *
   * @param config - Configuration options
   */
  constructor(config: SmartAquacultureConfig) {
    this.config = {
      timeout: 30000,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
    };

    if (config.apiKey) {
      this.headers['Authorization'] = `Bearer ${config.apiKey}`;
    }
  }

  /**
   * Makes an HTTP request to the API
   *
   * @param endpoint - API endpoint path
   * @param options - Fetch options
   * @returns Promise resolving to the response data
   */
  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.baseUrl}${endpoint}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(url, {
        ...options,
        headers: {
          ...this.headers,
          ...options.headers,
        },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return {
        success: true,
        data,
        timestamp: new Date().toISOString(),
      };
    } catch (error) {
      clearTimeout(timeoutId);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        timestamp: new Date().toISOString(),
      };
    }
  }

  // ==================== Farm Management ====================

  /**
   * Get aquaculture farm information
   *
   * @param farmId - Farm ID
   * @returns Promise resolving to farm data
   */
  async getFarm(farmId: string): Promise<ApiResponse<AquacultureFarm>> {
    return this.request<AquacultureFarm>(`/farms/${farmId}`);
  }

  /**
   * Create a new aquaculture farm
   *
   * @param farm - Farm data to create
   * @returns Promise resolving to created farm data
   */
  async createFarm(farm: AquacultureFarm): Promise<ApiResponse<AquacultureFarm>> {
    return this.request<AquacultureFarm>('/farms', {
      method: 'POST',
      body: JSON.stringify(farm),
    });
  }

  /**
   * Update farm information
   *
   * @param farmId - Farm ID to update
   * @param farm - Updated farm data
   * @returns Promise resolving to updated farm data
   */
  async updateFarm(
    farmId: string,
    farm: Partial<AquacultureFarm>
  ): Promise<ApiResponse<AquacultureFarm>> {
    return this.request<AquacultureFarm>(`/farms/${farmId}`, {
      method: 'PUT',
      body: JSON.stringify(farm),
    });
  }

  /**
   * Delete a farm
   *
   * @param farmId - Farm ID to delete
   * @returns Promise resolving to deletion confirmation
   */
  async deleteFarm(farmId: string): Promise<ApiResponse<void>> {
    return this.request<void>(`/farms/${farmId}`, {
      method: 'DELETE',
    });
  }

  /**
   * List farms with optional filtering
   *
   * @param query - Query parameters for filtering
   * @returns Promise resolving to list of farms
   */
  async listFarms(query?: AquacultureQuery): Promise<ApiResponse<AquacultureFarm[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/farms${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<AquacultureFarm[]>(endpoint);
  }

  // ==================== Water Quality Monitoring ====================

  /**
   * Get water quality sensor data
   *
   * @param sensorId - Sensor ID
   * @returns Promise resolving to sensor data
   */
  async getWaterQuality(sensorId: string): Promise<ApiResponse<WaterQualitySensorData>> {
    return this.request<WaterQualitySensorData>(`/water-quality/${sensorId}`);
  }

  /**
   * Submit water quality sensor data
   *
   * @param data - Water quality data
   * @returns Promise resolving to submitted data
   */
  async submitWaterQuality(
    data: WaterQualitySensorData
  ): Promise<ApiResponse<WaterQualitySensorData>> {
    return this.request<WaterQualitySensorData>('/water-quality', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  /**
   * Get water quality history for a tank
   *
   * @param tankId - Tank ID
   * @param query - Query parameters
   * @returns Promise resolving to water quality data list
   */
  async getWaterQualityHistory(
    tankId: string,
    query?: AquacultureQuery
  ): Promise<ApiResponse<WaterQualitySensorData[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/tanks/${tankId}/water-quality${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<WaterQualitySensorData[]>(endpoint);
  }

  /**
   * Get active water quality alerts
   *
   * @param farmId - Farm ID
   * @returns Promise resolving to list of alerts
   */
  async getWaterQualityAlerts(farmId: string): Promise<ApiResponse<WaterQualitySensorData[]>> {
    return this.request<WaterQualitySensorData[]>(`/farms/${farmId}/alerts`);
  }

  // ==================== Fish Health & Biomass ====================

  /**
   * Get fish health and biomass data
   *
   * @param batchId - Batch ID
   * @returns Promise resolving to fish health data
   */
  async getFishHealth(batchId: string): Promise<ApiResponse<FishHealthBiomass>> {
    return this.request<FishHealthBiomass>(`/fish-health/${batchId}`);
  }

  /**
   * Submit fish health and biomass data
   *
   * @param data - Fish health data
   * @returns Promise resolving to submitted data
   */
  async submitFishHealth(data: FishHealthBiomass): Promise<ApiResponse<FishHealthBiomass>> {
    return this.request<FishHealthBiomass>('/fish-health', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  /**
   * Update fish health data
   *
   * @param batchId - Batch ID
   * @param data - Updated health data
   * @returns Promise resolving to updated data
   */
  async updateFishHealth(
    batchId: string,
    data: Partial<FishHealthBiomass>
  ): Promise<ApiResponse<FishHealthBiomass>> {
    return this.request<FishHealthBiomass>(`/fish-health/${batchId}`, {
      method: 'PUT',
      body: JSON.stringify(data),
    });
  }

  /**
   * Get growth projections for a batch
   *
   * @param batchId - Batch ID
   * @returns Promise resolving to growth projections
   */
  async getGrowthProjections(
    batchId: string
  ): Promise<ApiResponse<{ projectedHarvestDate: string; projectedWeight: number }>> {
    return this.request<{ projectedHarvestDate: string; projectedWeight: number }>(
      `/fish-health/${batchId}/projections`
    );
  }

  // ==================== Feeding System ====================

  /**
   * Get feeding system data
   *
   * @param feedingId - Feeding ID
   * @returns Promise resolving to feeding data
   */
  async getFeedingData(feedingId: string): Promise<ApiResponse<FeedingSystemData>> {
    return this.request<FeedingSystemData>(`/feeding/${feedingId}`);
  }

  /**
   * Submit feeding data
   *
   * @param data - Feeding system data
   * @returns Promise resolving to submitted data
   */
  async submitFeedingData(data: FeedingSystemData): Promise<ApiResponse<FeedingSystemData>> {
    return this.request<FeedingSystemData>('/feeding', {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  /**
   * Get feeding history for a tank
   *
   * @param tankId - Tank ID
   * @param query - Query parameters
   * @returns Promise resolving to feeding history
   */
  async getFeedingHistory(
    tankId: string,
    query?: AquacultureQuery
  ): Promise<ApiResponse<FeedingSystemData[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/tanks/${tankId}/feeding${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<FeedingSystemData[]>(endpoint);
  }

  // ==================== Harvest Management ====================

  /**
   * Get harvest record
   *
   * @param harvestId - Harvest ID
   * @returns Promise resolving to harvest data
   */
  async getHarvest(harvestId: string): Promise<ApiResponse<HarvestRecord>> {
    return this.request<HarvestRecord>(`/harvests/${harvestId}`);
  }

  /**
   * Record a harvest
   *
   * @param harvest - Harvest data
   * @returns Promise resolving to recorded harvest
   */
  async recordHarvest(harvest: HarvestRecord): Promise<ApiResponse<HarvestRecord>> {
    return this.request<HarvestRecord>('/harvests', {
      method: 'POST',
      body: JSON.stringify(harvest),
    });
  }

  /**
   * List harvests for a farm
   *
   * @param farmId - Farm ID
   * @param query - Query parameters
   * @returns Promise resolving to list of harvests
   */
  async listHarvests(
    farmId: string,
    query?: AquacultureQuery
  ): Promise<ApiResponse<HarvestRecord[]>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/farms/${farmId}/harvests${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<HarvestRecord[]>(endpoint);
  }

  // ==================== Analytics & Reporting ====================

  /**
   * Get farm performance analytics
   *
   * @param farmId - Farm ID
   * @param query - Query parameters
   * @returns Promise resolving to analytics data
   */
  async getFarmAnalytics(
    farmId: string,
    query?: AquacultureQuery
  ): Promise<ApiResponse<any>> {
    const params = new URLSearchParams();
    if (query) {
      Object.entries(query).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, String(value));
        }
      });
    }

    const endpoint = `/farms/${farmId}/analytics${params.toString() ? `?${params.toString()}` : ''}`;
    return this.request<any>(endpoint);
  }
}

// Export all types
export * from './types';

// Export default client
export default SmartAquacultureClient;
