/**
 * WIA-COMP-019: RTOS - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

export interface RTOSConfig {
  projectName: string;
  version: string;
  outputPath?: string;
}

export interface RTOSResult {
  success: boolean;
  message: string;
  data?: any;
  errors?: string[];
}

export type RTOSStatus = 'pending' | 'processing' | 'completed' | 'failed';

export class RTOSError extends Error {
  constructor(message: string, public code?: string) {
    super(message);
    this.name = 'RTOSError';
  }
}

export const RTOS_VERSION = '1.0.0';
