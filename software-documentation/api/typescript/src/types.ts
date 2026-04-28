/**
 * WIA-COMP-017: Documentation - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

export interface DocumentationConfig {
  projectName: string;
  version: string;
  outputPath?: string;
}

export interface DocumentationResult {
  success: boolean;
  message: string;
  data?: any;
  errors?: string[];
}

export type DocumentationStatus = 'pending' | 'processing' | 'completed' | 'failed';

export class DocumentationError extends Error {
  constructor(message: string, public code?: string) {
    super(message);
    this.name = 'DocumentationError';
  }
}

export const Documentation_VERSION = '1.0.0';
