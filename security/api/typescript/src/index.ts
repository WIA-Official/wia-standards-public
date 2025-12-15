/**
 * WIA Security SDK - TypeScript
 * Main entry point
 */

// Types
export * from './types';

// Validator
export { validateEvent, ValidationResult } from './validator';
export {
  isAlertEvent,
  isThreatIntelEvent,
  isVulnerabilityEvent,
  isIncidentEvent,
  isNetworkEvent,
  isEndpointEvent,
  isAuthEvent
} from './validator';

// Builder
export {
  EventBuilder,
  AlertBuilder,
  ThreatIntelBuilder,
  VulnerabilityBuilder,
  IncidentBuilder,
  NetworkEventBuilder,
  EndpointEventBuilder,
  AuthEventBuilder,
  createAlert,
  createThreatIntel,
  createVulnerability,
  createIncident,
  createNetworkEvent,
  createEndpointEvent,
  createAuthEvent
} from './builder';

// Converter
export {
  toStixBundle,
  toStixIndicator,
  fromStixIndicator,
  toEcsEvent,
  toOcsfEvent,
  toSplunkEvent,
  toElasticEvent
} from './converter';

// Client
export { SecurityClient, SecurityClientConfig } from './client';

// Protocol (Phase 3)
export * from './protocol';

// Integration (Phase 4)
export * from './integration';

// Version
export const VERSION = '1.0.0';
export const SCHEMA_URL = 'https://wia.live/security/v1/schema.json';
