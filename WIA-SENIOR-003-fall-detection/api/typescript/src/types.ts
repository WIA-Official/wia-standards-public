/**
 * WIA-SENIOR-003: Fall Detection Standard
 * @philosophy 弘益人間
 */

export interface Fall_DetectionConfig {
  apiKey: string;
  apiUrl?: string;
  enableRealTimeMonitoring?: boolean;
}

export interface Fall_DetectionData {
  id: string;
  timestamp: Date;
  data: any;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; };
}
