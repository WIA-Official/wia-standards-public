/**
 * WIA-COMP-010: API Gateway - Type Definitions
 * @version 1.0.0
 * 弘益人間 (Benefit All Humanity)
 */

export type AuthType = 'none' | 'api-key' | 'jwt' | 'oauth';

export interface GatewayConfig {
  name: string;
  port: number;
  ssl?: boolean;
  cors?: boolean;
}

export interface RouteConfig {
  path: string;
  target: string;
  methods?: string[];
  auth?: { type: AuthType; config?: any };
  rateLimit?: { requests: number; period: string };
  transform?: boolean;
  cache?: { ttl: number };
}

export interface GatewayInfo {
  id: string;
  name: string;
  port: number;
  routes: number;
  requests: number;
}

export interface RouteInfo {
  id: string;
  path: string;
  target: string;
  requests: number;
  latency: number;
}

export enum GatewayErrorCode {
  GATEWAY_NOT_FOUND = 'G001',
  ROUTE_EXISTS = 'G002',
  INVALID_CONFIG = 'G003',
}

export class GatewayError extends Error {
  constructor(public code: GatewayErrorCode, message: string) {
    super(message);
    this.name = 'GatewayError';
  }
}
