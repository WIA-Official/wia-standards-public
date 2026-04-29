/**
 * WIA-SENIOR-002: Dementia Care Standard
 * @philosophy 弘益人間
 */

export interface Dementia_CareConfig {
  apiKey: string;
  apiUrl?: string;
  enableRealTimeMonitoring?: boolean;
}

export interface Dementia_CareData {
  id: string;
  timestamp: Date;
  data: any;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; };
}
