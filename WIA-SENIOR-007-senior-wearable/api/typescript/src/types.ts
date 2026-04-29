/**
 * WIA-SENIOR-007: Senior Wearable Standard
 * @philosophy 弘益人間
 */

export interface Senior_WearableConfig {
  apiKey: string;
  apiUrl?: string;
  enableRealTimeMonitoring?: boolean;
}

export interface Senior_WearableData {
  id: string;
  timestamp: Date;
  data: any;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; };
}
