/**
 * WIA-COMP-020: LowCode - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

export interface LowCodeConfig {
  projectName: string;
  version: string;
  outputPath?: string;
}

export interface LowCodeResult {
  success: boolean;
  message: string;
  data?: any;
  errors?: string[];
}

export type LowCodeStatus = 'pending' | 'processing' | 'completed' | 'failed';

export class LowCodeError extends Error {
  constructor(message: string, public code?: string) {
    super(message);
    this.name = 'LowCodeError';
  }
}

export const LowCode_VERSION = '1.0.0';
