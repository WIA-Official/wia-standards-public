/**
 * WIA-COMP-002: Storage - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

export interface SystemConfig {
  id: string;
  name: string;
  version: string;
}

export interface PerformanceMetrics {
  throughput: number;
  latency: number;
  efficiency: number;
}

export interface AnalysisResult {
  success: boolean;
  metrics: PerformanceMetrics;
  recommendations: string[];
}

export class StorageError extends Error {
  constructor(message: string, public code: string) {
    super(message);
    this.name = 'StorageError';
  }
}

export const CONSTANTS = {
  VERSION: '1.0.0',
  MAX_RETRIES: 3,
} as const;
