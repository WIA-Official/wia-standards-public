/**
 * WIA-COMP-009: Microservices - Type Definitions
 * @version 1.0.0
 * 弘益人間 (Benefit All Humanity)
 */

export type ServiceState = 'starting' | 'running' | 'stopping' | 'stopped';
export type Protocol = 'http' | 'grpc' | 'amqp';

export interface ServiceConfig {
  name: string;
  port: number;
  protocol?: Protocol;
  healthCheck?: string;
  dependencies?: string[];
  metadata?: Record<string, string>;
}

export interface ServiceInfo {
  id: string;
  name: string;
  state: ServiceState;
  host: string;
  port: number;
  registered: Date;
  healthy: boolean;
}

export interface ServiceCall {
  service: string;
  endpoint: string;
  method?: string;
  body?: any;
  headers?: Record<string, string>;
}

export enum MicroservicesErrorCode {
  SERVICE_NOT_FOUND = 'M001',
  REGISTRATION_FAILED = 'M002',
  CALL_FAILED = 'M003',
}

export class MicroservicesError extends Error {
  constructor(public code: MicroservicesErrorCode, message: string) {
    super(message);
    this.name = 'MicroservicesError';
  }
}
