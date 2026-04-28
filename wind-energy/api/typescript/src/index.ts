/**
 * WIA-ENE-006 Wind Energy TypeScript SDK
 *
 * @module @wia/ene-006
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 * @philosophy 弘益人間 (홍익인간) - Benefit All Humanity
 */

import {
  ClientConfig,
  TurbineDataPacket,
  TurbineConfig,
  WindFarmConfig,
  WindFarmStatus,
  ControlCommand,
  ControlResponse,
  PerformanceAnalysisRequest,
  PerformanceAnalysisResult,
  ForecastRequest,
  ForecastResult,
  ExportConfig,
  ExportResult
} from './types';

/**
 * WIA-ENE-006 Wind Energy Client
 */
export class WindEnergyClient {
  private config: ClientConfig;
  private baseUrl: string;
  private headers: Record<string, string>;

  /**
   * Initialize Wind Energy Client
   * @param config Client configuration
   */
  constructor(config: ClientConfig) {
    this.config = {
      timeout: 30000,
      retry: {
        maxRetries: 3,
        retryDelay: 1000
      },
      logging: false,
      ...config
    };

    this.baseUrl = config.endpoint.replace(/\/$/, '');
    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${config.apiKey}`,
      'X-WIA-Philosophy': '弘益人間'
    };
  }

  // ========================================================================
  // Turbine Operations
  // ========================================================================

  /**
   * Get real-time turbine data
   * @param turbineId Turbine identifier
   * @returns Current turbine data packet
   */
  async getTurbineData(turbineId: string): Promise<TurbineDataPacket> {
    return this.request<TurbineDataPacket>('GET', `/turbines/${turbineId}/data`);
  }

  /**
   * Get turbine configuration
   * @param turbineId Turbine identifier
   * @returns Turbine configuration
   */
  async getTurbineConfig(turbineId: string): Promise<TurbineConfig> {
    return this.request<TurbineConfig>('GET', `/turbines/${turbineId}/config`);
  }

  /**
   * Update turbine configuration
   * @param turbineId Turbine identifier
   * @param config Updated configuration
   * @returns Updated configuration
   */
  async updateTurbineConfig(turbineId: string, config: Partial<TurbineConfig>): Promise<TurbineConfig> {
    return this.request<TurbineConfig>('PATCH', `/turbines/${turbineId}/config`, config);
  }

  /**
   * Send control command to turbine
   * @param command Control command
   * @returns Control response
   */
  async sendControlCommand(command: ControlCommand): Promise<ControlResponse> {
    return this.request<ControlResponse>('POST', `/turbines/${command.turbineId}/control`, command);
  }

  // ========================================================================
  // Wind Farm Operations
  // ========================================================================

  /**
   * Get wind farm status
   * @param windFarmId Wind farm identifier
   * @returns Current wind farm status
   */
  async getWindFarmStatus(windFarmId: string): Promise<WindFarmStatus> {
    return this.request<WindFarmStatus>('GET', `/windfarms/${windFarmId}/status`);
  }

  /**
   * Get wind farm configuration
   * @param windFarmId Wind farm identifier
   * @returns Wind farm configuration
   */
  async getWindFarmConfig(windFarmId: string): Promise<WindFarmConfig> {
    return this.request<WindFarmConfig>('GET', `/windfarms/${windFarmId}/config`);
  }

  /**
   * List all turbines in wind farm
   * @param windFarmId Wind farm identifier
   * @returns Array of turbine configurations
   */
  async listTurbines(windFarmId: string): Promise<TurbineConfig[]> {
    return this.request<TurbineConfig[]>('GET', `/windfarms/${windFarmId}/turbines`);
  }

  // ========================================================================
  // Analytics
  // ========================================================================

  /**
   * Analyze performance
   * @param request Performance analysis request
   * @returns Analysis results
   */
  async analyzePerformance(request: PerformanceAnalysisRequest): Promise<PerformanceAnalysisResult> {
    return this.request<PerformanceAnalysisResult>('POST', '/analytics/performance', request);
  }

  /**
   * Get production forecast
   * @param request Forecast request
   * @returns Production forecast
   */
  async getForecast(request: ForecastRequest): Promise<ForecastResult> {
    return this.request<ForecastResult>('POST', '/analytics/forecast', request);
  }

  // ========================================================================
  // Data Export
  // ========================================================================

  /**
   * Export data
   * @param config Export configuration
   * @returns Export result with download URL
   */
  async exportData(config: ExportConfig): Promise<ExportResult> {
    return this.request<ExportResult>('POST', '/export', config);
  }

  // ========================================================================
  // Utility Methods
  // ========================================================================

  /**
   * Calculate wind power
   * @param windSpeed Wind speed (m/s)
   * @param airDensity Air density (kg/m³)
   * @param rotorArea Rotor swept area (m²)
   * @returns Available power (W)
   */
  calculateWindPower(windSpeed: number, airDensity: number, rotorArea: number): number {
    // Power = 0.5 × ρ × A × v³
    return 0.5 * airDensity * rotorArea * Math.pow(windSpeed, 3);
  }

  /**
   * Calculate capacity factor
   * @param actualEnergy Actual energy produced (kWh)
   * @param ratedPower Rated power (kW)
   * @param hours Time period (hours)
   * @returns Capacity factor (0-1)
   */
  calculateCapacityFactor(actualEnergy: number, ratedPower: number, hours: number): number {
    const potentialEnergy = ratedPower * hours;
    return actualEnergy / potentialEnergy;
  }

  /**
   * Estimate wind speed at height
   * @param knownSpeed Wind speed at known height (m/s)
   * @param knownHeight Known height (m)
   * @param targetHeight Target height (m)
   * @param shearExponent Shear exponent (default: 0.14)
   * @returns Estimated wind speed at target height (m/s)
   */
  estimateWindSpeedAtHeight(
    knownSpeed: number,
    knownHeight: number,
    targetHeight: number,
    shearExponent: number = 0.14
  ): number {
    // V₂ = V₁ × (H₂ / H₁)^α
    return knownSpeed * Math.pow(targetHeight / knownHeight, shearExponent);
  }

  // ========================================================================
  // Private Methods
  // ========================================================================

  /**
   * Make HTTP request
   * @param method HTTP method
   * @param path API path
   * @param body Request body
   * @returns Response data
   */
  private async request<T>(method: string, path: string, body?: any): Promise<T> {
    const url = `${this.baseUrl}${path}`;
    let lastError: Error | null = null;

    const maxRetries = this.config.retry?.maxRetries || 3;
    const retryDelay = this.config.retry?.retryDelay || 1000;

    for (let attempt = 0; attempt <= maxRetries; attempt++) {
      try {
        if (this.config.logging && attempt > 0) {
          console.log(`Retry attempt ${attempt} for ${method} ${path}`);
        }

        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), this.config.timeout);

        const response = await fetch(url, {
          method,
          headers: this.headers,
          body: body ? JSON.stringify(body) : undefined,
          signal: controller.signal
        });

        clearTimeout(timeout);

        if (!response.ok) {
          throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }

        const data = await response.json();
        return data as T;

      } catch (error) {
        lastError = error as Error;

        if (attempt < maxRetries) {
          await this.sleep(retryDelay * Math.pow(2, attempt));
          continue;
        }
      }
    }

    throw lastError || new Error('Request failed');
  }

  /**
   * Sleep utility
   * @param ms Milliseconds to sleep
   */
  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ========================================================================
// Export all types
// ========================================================================

export * from './types';

// ========================================================================
// Philosophy
// ========================================================================

/**
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * This SDK embodies the principle of broadly benefiting humanity by:
 * - Providing open, accessible wind energy technology
 * - Enabling efficient clean energy generation
 * - Supporting the global transition to renewable energy
 * - Facilitating knowledge sharing and innovation
 *
 * By using this SDK, you contribute to a sustainable future for all.
 */
export const PHILOSOPHY = '弘益人間 (홍익인간) - Benefit All Humanity';
