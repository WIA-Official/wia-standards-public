/**
 * WIA-COMP-018: Embedded - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

export interface EmbeddedConfig {
  projectName: string;
  version: string;
  outputPath?: string;
}

export interface EmbeddedResult {
  success: boolean;
  message: string;
  data?: any;
  errors?: string[];
}

export type EmbeddedStatus = 'pending' | 'processing' | 'completed' | 'failed';

export class EmbeddedError extends Error {
  constructor(message: string, public code?: string) {
    super(message);
    this.name = 'EmbeddedError';
  }
}

export const Embedded_VERSION = '1.0.0';
