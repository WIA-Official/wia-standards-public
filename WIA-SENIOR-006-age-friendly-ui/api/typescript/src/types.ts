/**
 * WIA-SENIOR-006: Age-Friendly UI Standard
 * @philosophy 弘益人間
 */

export interface Age_Friendly_UIConfig {
  apiKey: string;
  apiUrl?: string;
  enableRealTimeMonitoring?: boolean;
}

export interface Age_Friendly_UIData {
  id: string;
  timestamp: Date;
  data: any;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; };
}
