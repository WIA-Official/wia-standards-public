/**
 * WIA-SENIOR-005: Loneliness Prevention Standard
 * @philosophy 弘益人間
 */

export interface Loneliness_PreventionConfig {
  apiKey: string;
  apiUrl?: string;
  enableRealTimeMonitoring?: boolean;
}

export interface Loneliness_PreventionData {
  id: string;
  timestamp: Date;
  data: any;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; };
}
