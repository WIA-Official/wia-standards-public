/**
 * WIA-SENIOR-009: Senior Mobility Standard
 * @philosophy 弘益人間
 */

export interface Senior_MobilityConfig {
  apiKey: string;
  apiUrl?: string;
  enableRealTimeMonitoring?: boolean;
}

export interface Senior_MobilityData {
  id: string;
  timestamp: Date;
  data: any;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; };
}
