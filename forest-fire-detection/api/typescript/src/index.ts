/**
 * WIA-ENE-032: Forest Fire Detection Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  FireDetectionEvent,
  FireSpreadPrediction,
  EvacuationZone,
  FirefightingResource,
  FireAlert,
  ThermalDetection,
  DroneThermalDetection,
  SmokeDetectionEvent,
  FuelMoisture,
  FireDangerCalculation,
  FireDangerLevel,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  CertificationStatus,
  DetectionMethod,
  AlertLevel,
  ResourceType,
  ResourceStatus,
  EvacuationStatus,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface ForestFireDetectionSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class ForestFireDetectionSDK {
  private config: Required<ForestFireDetectionSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: ForestFireDetectionSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-032',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Fire Detection APIs
  // ==========================================================================

  /**
   * Report a new fire detection event
   */
  async reportFireDetection(
    event: Omit<FireDetectionEvent, 'eventId' | 'timestamp'>
  ): Promise<ApiResponse<FireDetectionEvent>> {
    return this.post<FireDetectionEvent>('/api/v1/fire/detect', event);
  }

  /**
   * Get fire detection event by ID
   */
  async getFireEvent(eventId: string): Promise<ApiResponse<FireDetectionEvent>> {
    return this.get<FireDetectionEvent>(`/api/v1/fire/${eventId}`);
  }

  /**
   * Get all active fires
   */
  async getActiveFires(
    params?: PaginationParams
  ): Promise<PaginatedResponse<FireDetectionEvent>> {
    return this.get<FireDetectionEvent[]>('/api/v1/fire/active', params);
  }

  /**
   * Get fire history within date range
   */
  async getFireHistory(
    filter: DateRangeFilter,
    params?: PaginationParams
  ): Promise<PaginatedResponse<FireDetectionEvent>> {
    return this.get<FireDetectionEvent[]>('/api/v1/fire/history', {
      ...filter,
      ...params,
    });
  }

  /**
   * Update fire event status
   */
  async updateFireEvent(
    eventId: string,
    updates: Partial<FireDetectionEvent>
  ): Promise<ApiResponse<FireDetectionEvent>> {
    return this.put<FireDetectionEvent>(`/api/v1/fire/${eventId}`, updates);
  }

  // ==========================================================================
  // Fire Spread Prediction APIs
  // ==========================================================================

  /**
   * Request fire spread prediction
   */
  async predictFireSpread(
    fireEventId: string,
    forecastHorizon: number = 24
  ): Promise<ApiResponse<FireSpreadPrediction>> {
    return this.post<FireSpreadPrediction>('/api/v1/prediction/spread', {
      fireEventId,
      forecastHorizon,
    });
  }

  /**
   * Get prediction by ID
   */
  async getPrediction(predictionId: string): Promise<ApiResponse<FireSpreadPrediction>> {
    return this.get<FireSpreadPrediction>(`/api/v1/prediction/${predictionId}`);
  }

  /**
   * Get latest prediction for a fire event
   */
  async getLatestPrediction(
    fireEventId: string
  ): Promise<ApiResponse<FireSpreadPrediction>> {
    return this.get<FireSpreadPrediction>(`/api/v1/prediction/latest/${fireEventId}`);
  }

  // ==========================================================================
  // Fire Danger Rating APIs
  // ==========================================================================

  /**
   * Calculate fire danger level
   */
  async calculateFireDanger(
    calculation: FireDangerCalculation
  ): Promise<ApiResponse<{ dangerLevel: FireDangerLevel; fwi: number }>> {
    return this.post<{ dangerLevel: FireDangerLevel; fwi: number }>(
      '/api/v1/danger/calculate',
      calculation
    );
  }

  /**
   * Get current fire danger for a region
   */
  async getFireDanger(region: string): Promise<ApiResponse<{
    region: string;
    dangerLevel: FireDangerLevel;
    fwi: number;
    timestamp: string;
  }>> {
    return this.get<any>(`/api/v1/danger/region/${region}`);
  }

  // ==========================================================================
  // Thermal Detection APIs
  // ==========================================================================

  /**
   * Submit thermal camera detection
   */
  async submitThermalDetection(
    detection: Omit<ThermalDetection, 'detectionTime'>
  ): Promise<ApiResponse<ThermalDetection>> {
    return this.post<ThermalDetection>('/api/v1/thermal/camera', detection);
  }

  /**
   * Submit drone thermal detection
   */
  async submitDroneThermalDetection(
    detection: Omit<DroneThermalDetection, 'detectionTime'>
  ): Promise<ApiResponse<DroneThermalDetection>> {
    return this.post<DroneThermalDetection>('/api/v1/thermal/drone', detection);
  }

  // ==========================================================================
  // Smoke Detection APIs
  // ==========================================================================

  /**
   * Submit smoke detection event
   */
  async submitSmokeDetection(
    event: Omit<SmokeDetectionEvent, 'detectionTime'>
  ): Promise<ApiResponse<SmokeDetectionEvent>> {
    return this.post<SmokeDetectionEvent>('/api/v1/smoke/detect', event);
  }

  // ==========================================================================
  // Fuel Moisture APIs
  // ==========================================================================

  /**
   * Submit fuel moisture measurement
   */
  async submitFuelMoisture(
    measurement: Omit<FuelMoisture, 'measurementId' | 'timestamp'>
  ): Promise<ApiResponse<FuelMoisture>> {
    return this.post<FuelMoisture>('/api/v1/fuel/moisture', measurement);
  }

  /**
   * Get latest fuel moisture for a location
   */
  async getFuelMoisture(
    latitude: number,
    longitude: number
  ): Promise<ApiResponse<FuelMoisture>> {
    return this.get<FuelMoisture>('/api/v1/fuel/moisture/latest', {
      latitude,
      longitude,
    });
  }

  // ==========================================================================
  // Evacuation Zone APIs
  // ==========================================================================

  /**
   * Get evacuation zones for a fire event
   */
  async getEvacuationZones(
    fireEventId: string
  ): Promise<ApiResponse<EvacuationZone[]>> {
    return this.get<EvacuationZone[]>(`/api/v1/evacuation/zones/${fireEventId}`);
  }

  /**
   * Update evacuation zone status
   */
  async updateEvacuationZone(
    zoneId: string,
    status: EvacuationStatus
  ): Promise<ApiResponse<EvacuationZone>> {
    return this.put<EvacuationZone>(`/api/v1/evacuation/zone/${zoneId}`, { status });
  }

  /**
   * Get evacuation routes for a zone
   */
  async getEvacuationRoutes(
    zoneId: string
  ): Promise<ApiResponse<{ routes: string[]; optimized: boolean }>> {
    return this.get<any>(`/api/v1/evacuation/routes/${zoneId}`);
  }

  // ==========================================================================
  // Firefighting Resources APIs
  // ==========================================================================

  /**
   * Get available firefighting resources
   */
  async getAvailableResources(
    type?: ResourceType
  ): Promise<ApiResponse<FirefightingResource[]>> {
    const params = type ? { type } : undefined;
    return this.get<FirefightingResource[]>('/api/v1/resources/available', params);
  }

  /**
   * Dispatch resource to fire event
   */
  async dispatchResource(
    resourceId: string,
    fireEventId: string
  ): Promise<ApiResponse<FirefightingResource>> {
    return this.post<FirefightingResource>('/api/v1/resources/dispatch', {
      resourceId,
      fireEventId,
    });
  }

  /**
   * Update resource status
   */
  async updateResourceStatus(
    resourceId: string,
    status: ResourceStatus,
    location?: { latitude: number; longitude: number }
  ): Promise<ApiResponse<FirefightingResource>> {
    return this.put<FirefightingResource>(`/api/v1/resources/${resourceId}`, {
      status,
      location,
    });
  }

  /**
   * Track resource in real-time
   */
  async trackResource(resourceId: string): Promise<ApiResponse<FirefightingResource>> {
    return this.get<FirefightingResource>(`/api/v1/resources/track/${resourceId}`);
  }

  // ==========================================================================
  // Alert System APIs
  // ==========================================================================

  /**
   * Send fire alert
   */
  async sendAlert(
    alert: Omit<FireAlert, 'alertId' | 'timestamp' | 'deliveryStatus'>
  ): Promise<ApiResponse<FireAlert>> {
    return this.post<FireAlert>('/api/v1/alert/send', alert);
  }

  /**
   * Get alert status
   */
  async getAlertStatus(alertId: string): Promise<ApiResponse<FireAlert>> {
    return this.get<FireAlert>(`/api/v1/alert/${alertId}`);
  }

  /**
   * Get alerts for a fire event
   */
  async getFireAlerts(
    fireEventId: string
  ): Promise<ApiResponse<FireAlert[]>> {
    return this.get<FireAlert[]>(`/api/v1/alert/fire/${fireEventId}`);
  }

  // ==========================================================================
  // Certification APIs
  // ==========================================================================

  /**
   * Get certification status
   */
  async getCertificationStatus(): Promise<ApiResponse<CertificationStatus>> {
    return this.get<CertificationStatus>('/api/v1/certification/status');
  }

  /**
   * Submit certification audit data
   */
  async submitAuditData(
    data: {
      uptime: number;
      avgDetectionTime: number;
      falsePositiveRate: number;
      missedDetectionRate: number;
    }
  ): Promise<ApiResponse<{ submitted: boolean }>> {
    return this.post<{ submitted: boolean }>('/api/v1/certification/audit', data);
  }

  // ==========================================================================
  // HTTP Methods (Private)
  // ==========================================================================

  private async get<T>(
    path: string,
    params?: Record<string, any>
  ): Promise<ApiResponse<T> | PaginatedResponse<T>> {
    const url = new URL(path, this.config.endpoint);
    if (params) {
      Object.keys(params).forEach((key) => {
        if (params[key] !== undefined) {
          url.searchParams.append(key, String(params[key]));
        }
      });
    }

    return this.fetch<T>(url.toString(), {
      method: 'GET',
      headers: this.headers,
    });
  }

  private async post<T>(
    path: string,
    body: any
  ): Promise<ApiResponse<T>> {
    const url = new URL(path, this.config.endpoint);

    return this.fetch<T>(url.toString(), {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify(body),
    });
  }

  private async put<T>(
    path: string,
    body: any
  ): Promise<ApiResponse<T>> {
    const url = new URL(path, this.config.endpoint);

    return this.fetch<T>(url.toString(), {
      method: 'PUT',
      headers: this.headers,
      body: JSON.stringify(body),
    });
  }

  private async delete<T>(
    path: string
  ): Promise<ApiResponse<T>> {
    const url = new URL(path, this.config.endpoint);

    return this.fetch<T>(url.toString(), {
      method: 'DELETE',
      headers: this.headers,
    });
  }

  private async fetch<T>(
    url: string,
    options: RequestInit
  ): Promise<ApiResponse<T> | PaginatedResponse<T>> {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      if (this.config.debug) {
        console.log(`[WIA-ENE-032] ${options.method} ${url}`);
      }

      const response = await fetch(url, {
        ...options,
        signal: controller.signal,
      });

      clearTimeout(timeout);

      const data = await response.json();

      if (!response.ok) {
        if (this.config.debug) {
          console.error(`[WIA-ENE-032] Error: ${response.status}`, data);
        }
        return {
          success: false,
          error: {
            code: String(response.status),
            message: data.message || response.statusText,
          },
        };
      }

      if (this.config.debug) {
        console.log(`[WIA-ENE-032] Success:`, data);
      }

      return {
        success: true,
        ...data,
      };
    } catch (error: any) {
      clearTimeout(timeout);

      if (this.config.debug) {
        console.error(`[WIA-ENE-032] Fetch error:`, error);
      }

      return {
        success: false,
        error: {
          code: 'NETWORK_ERROR',
          message: error.message || 'Network request failed',
        },
      };
    }
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate Fire Weather Index (FWI)
 * Simplified version - actual implementation would use full FWI System
 */
export function calculateFWI(
  temperature: number,     // °C
  humidity: number,        // %
  windSpeed: number,       // km/h
  precipitation: number    // mm (last 24h)
): number {
  // Simplified FWI calculation
  // Real FWI requires moisture codes (FFMC, DMC, DC)
  const tempFactor = Math.max(0, temperature - 10) / 30;
  const humidityFactor = Math.max(0, 100 - humidity) / 75;
  const windFactor = Math.min(windSpeed / 50, 1);
  const rainFactor = Math.max(0, 1 - precipitation / 10);

  const fwi = (tempFactor * 0.3 + humidityFactor * 0.3 + windFactor * 0.2 + rainFactor * 0.2) * 50;

  return Math.round(fwi * 10) / 10;
}

/**
 * Determine fire danger level from FWI
 */
export function fwiToFireDangerLevel(fwi: number): FireDangerLevel {
  if (fwi < 5) return FireDangerLevel.LOW;
  if (fwi < 12) return FireDangerLevel.MODERATE;
  if (fwi < 22) return FireDangerLevel.HIGH;
  if (fwi < 38) return FireDangerLevel.VERY_HIGH;
  return FireDangerLevel.EXTREME;
}

/**
 * Calculate distance between two coordinates (Haversine formula)
 */
export function calculateDistance(
  lat1: number,
  lon1: number,
  lat2: number,
  lon2: number
): number {
  const R = 6371; // Earth's radius in km
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
    Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

/**
 * Estimate fire arrival time based on spread rate and distance
 */
export function estimateArrivalTime(
  spreadRateMetersPerMin: number,
  distanceKm: number
): Date {
  const distanceMeters = distanceKm * 1000;
  const minutesToArrive = distanceMeters / spreadRateMetersPerMin;
  return new Date(Date.now() + minutesToArrive * 60 * 1000);
}

/**
 * Convert brightness temperature to Fire Radiative Power (FRP)
 * Simplified relationship - actual conversion is sensor-specific
 */
export function brightnessToFRP(brightness: number, area: number): number {
  // Simplified formula: FRP ≈ k × (T^4 - T_bg^4) × A
  const k = 4.34e-19; // Stefan-Boltzmann constant adjusted
  const T_bg = 300; // Background temperature (K)
  const frp = k * (Math.pow(brightness, 4) - Math.pow(T_bg, 4)) * area;
  return Math.round(frp * 100) / 100;
}
