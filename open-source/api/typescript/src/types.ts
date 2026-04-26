/**
 * WIA-COMP-015: Open Source - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Computing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

export type Status = 'pending' | 'running' | 'success' | 'failed';
export type Environment = 'development' | 'staging' | 'production';

export interface Config {
  name: string;
  version: string;
  environment: Environment;
}

export interface Result {
  id: string;
  status: Status;
  message: string;
  timestamp: Date;
}

export enum ErrorCode {
  INVALID_CONFIG = 'E001',
  EXECUTION_FAILED = 'E002',
  NOT_FOUND = 'E003',
}

export class StandardError extends Error {
  constructor(
    public code: ErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'StandardError';
  }
}

export type { Config, Result, Status, Environment };
export { ErrorCode, StandardError };
