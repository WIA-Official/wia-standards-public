/**
 * WIA SPACE Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type Status = 'active' | 'inactive' | 'pending' | 'completed';

export interface DataRecord {
  id: string;
  timestamp: string;
  status: Status;
  metadata?: Record<string, any>;
}

export interface WIAConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
}
