/**
 * WIA-SENIOR-010: Intergenerational Standard
 * @philosophy 弘益人間
 */

export interface IntergenerationalConfig {
  apiKey: string;
  apiUrl?: string;
  enableRealTimeMonitoring?: boolean;
}

export interface IntergenerationalData {
  id: string;
  timestamp: Date;
  data: any;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: { code: string; message: string; };
}
