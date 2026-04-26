/**
 * WIA-ENE-003 Carbon Capture & Storage Standard - TypeScript SDK
 * Version: 1.0.0
 * Philosophy: 弘益人間 (홍익인간) - Benefit All Humanity
 * 
 * This SDK provides a comprehensive interface for integrating with
 * WIA-ENE-003 compliant carbon capture and storage systems.
 */

import axios, { AxiosInstance } from 'axios';
import type {
  APIConfig,
  CaptureSystemData,
  StorageSiteData,
  CarbonCredit,
  ComplianceReport,
  PerformanceMetrics
} from './types';

export * from './types';

export class WIACarbonCapture {
  private client: AxiosInstance;
  private facilityId: string;

  constructor(config: APIConfig) {
    this.client = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: {
        'Authorization': `Bearer ${config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'ENE-003',
        'X-WIA-Version': '1.0.0'
      }
    });

    this.facilityId = '';
  }

  /**
   * Initialize SDK with facility ID
   */
  setFacility(facilityId: string): void {
    this.facilityId = facilityId;
  }

  /**
   * Get real-time capture system data
   */
  async getRealTimeData(): Promise<CaptureSystemData> {
    const response = await this.client.get(`/facilities/${this.facilityId}/realtime`);
    return response.data;
  }

  /**
   * Get historical capture data
   */
  async getHistoricalData(params: {
    start: string;
    end: string;
    aggregation?: 'hourly' | 'daily' | 'monthly';
  }): Promise<CaptureSystemData[]> {
    const response = await this.client.get(`/facilities/${this.facilityId}/historical`, {
      params
    });
    return response.data;
  }

  /**
   * Get storage site status
   */
  async getStorageStatus(siteId: string): Promise<StorageSiteData> {
    const response = await this.client.get(`/storage-sites/${siteId}/status`);
    return response.data;
  }

  /**
   * Get performance analytics
   */
  async getPerformanceMetrics(period: 'daily' | 'monthly' | 'annual'): Promise<PerformanceMetrics> {
    const response = await this.client.get(
      `/facilities/${this.facilityId}/analytics/performance`,
      { params: { period } }
    );
    return response.data;
  }

  /**
   * Get carbon credits
   */
  async getCarbonCredits(): Promise<CarbonCredit[]> {
    const response = await this.client.get(`/facilities/${this.facilityId}/carbon-credits`);
    return response.data;
  }

  /**
   * Submit carbon credits for verification
   */
  async submitForVerification(data: {
    tonnes: number;
    period: { start: string; end: string };
  }): Promise<{ creditId: string; status: string }> {
    const response = await this.client.post(
      `/facilities/${this.facilityId}/carbon-credits/verify`,
      data
    );
    return response.data;
  }

  /**
   * Get compliance report
   */
  async getComplianceReport(reportId: string): Promise<ComplianceReport> {
    const response = await this.client.get(`/facilities/${this.facilityId}/reports/${reportId}`);
    return response.data;
  }

  /**
   * Generate new compliance report
   */
  async generateReport(params: {
    reportType: 'daily' | 'monthly' | 'quarterly' | 'annual';
    startDate: string;
    endDate: string;
    format?: 'pdf' | 'json' | 'csv';
  }): Promise<{ reportId: string; downloadUrl: string }> {
    const response = await this.client.post(
      `/facilities/${this.facilityId}/reports/generate`,
      params
    );
    return response.data;
  }

  /**
   * Subscribe to real-time alerts (WebSocket)
   */
  subscribeToAlerts(callback: (alert: any) => void): void {
    // WebSocket implementation would go here
    console.log('Alert subscription for facility:', this.facilityId);
    // This is a placeholder - actual implementation would use WebSocket
  }

  /**
   * Update capture system setpoint
   */
  async updateSetpoint(params: {
    parameter: string;
    value: number;
    reason?: string;
  }): Promise<{ success: boolean; message: string }> {
    const response = await this.client.post(
      `/facilities/${this.facilityId}/operations/setpoint`,
      params
    );
    return response.data;
  }
}

// Example usage:
// const client = new WIACarbonCapture({
//   baseURL: 'https://api.carbon-capture.example.com/v1',
//   apiKey: process.env.WIA_API_KEY
// });
// client.setFacility('WIA-ENE-003-001');
// const data = await client.getRealTimeData();
