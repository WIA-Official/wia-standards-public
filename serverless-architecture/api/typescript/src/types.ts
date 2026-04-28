/**
 * WIA-COMP-008: Serverless Architecture - Type Definitions
 * @version 1.0.0
 * 弘益人間 (Benefit All Humanity)
 */

export type FunctionState = 'deploying' | 'active' | 'inactive' | 'error';
export type Runtime = 'nodejs18' | 'python3.11' | 'go1.21' | 'java17';
export type TriggerType = 'http' | 'queue' | 'schedule' | 'storage';

export interface FunctionConfig {
  name: string;
  runtime: Runtime;
  handler: string;
  memory?: string;
  timeout?: number;
  environment?: Record<string, string>;
  triggers?: TriggerConfig[];
}

export interface TriggerConfig {
  type: TriggerType;
  path?: string;
  schedule?: string;
  queue?: string;
}

export interface FunctionInfo {
  id: string;
  name: string;
  state: FunctionState;
  runtime: Runtime;
  invocations: number;
  lastInvoked?: Date;
}

export interface InvocationResult {
  statusCode: number;
  body: any;
  duration: number;
  memoryUsed: number;
}

export enum ServerlessErrorCode {
  FUNCTION_NOT_FOUND = 'S001',
  DEPLOYMENT_FAILED = 'S002',
  INVOCATION_FAILED = 'S003',
}

export class ServerlessError extends Error {
  constructor(public code: ServerlessErrorCode, message: string) {
    super(message);
    this.name = 'ServerlessError';
  }
}
