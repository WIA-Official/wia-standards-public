/**
 * WIA-SENIOR-004: Aging in Place Standard
 * @philosophy 弘益人間
 */

export interface Aging_in_PlaceConfig {
  apiKey: string;
  apiUrl?: string;
  enableRealTimeMonitoring?: boolean;
}

export interface Aging_in_PlaceData {
  id: string;
  timestamp: Date;
  data: any;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; };
}
