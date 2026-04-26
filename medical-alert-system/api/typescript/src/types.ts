/**
 * WIA-MED-014: Medical Alert System Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Type Aliases
// ============================================================================

export type Timestamp = string;
export type AlertID = string;
export type PatientID = string;
export type RecipientID = string;

// ============================================================================
// Enums
// ============================================================================

export enum AlertType {
  EMERGENCY = 'emergency',
  URGENT = 'urgent',
  ROUTINE = 'routine',
  INFO = 'info',
}

export enum AlertPriority {
  P1 = 'p1',
  P2 = 'p2',
  P3 = 'p3',
  P4 = 'p4',
}

export enum DeliveryChannel {
  SMS = 'sms',
  VOICE = 'voice',
  PUSH = 'push',
  EMAIL = 'email',
  PAGER = 'pager',
}

export enum AlertStatus {
  PENDING = 'pending',
  SENT = 'sent',
  DELIVERED = 'delivered',
  ACKNOWLEDGED = 'acknowledged',
  ESCALATED = 'escalated',
  RESOLVED = 'resolved',
  CANCELLED = 'cancelled',
}

export enum EscalationTrigger {
  TIMEOUT = 'timeout',
  NO_RESPONSE = 'no_response',
  MANUAL = 'manual',
  CONDITION_WORSENED = 'condition_worsened',
}

// ============================================================================
// Alert Types
// ============================================================================

export interface MedicalAlert {
  alertId: AlertID;
  patientId: PatientID;
  type: AlertType;
  priority: AlertPriority;
  status: AlertStatus;
  title: string;
  message: string;
  createdAt: Timestamp;
  updatedAt: Timestamp;
  expiresAt?: Timestamp;
  recipients: Recipient[];
  escalationPolicy?: EscalationPolicy;
  vitalSigns?: VitalSnapshot;
  location?: AlertLocation;
  metadata?: Record<string, unknown>;
}

export interface AlertLocation {
  latitude: number;
  longitude: number;
  address?: string;
  facility?: string;
  room?: string;
}

// ============================================================================
// Recipient Types
// ============================================================================

export interface Recipient {
  recipientId: RecipientID;
  name: string;
  role: string;
  channels: DeliveryChannel[];
  priority: number;
  acknowledged: boolean;
  acknowledgedAt?: Timestamp;
  deliveryStatus: DeliveryStatus[];
}

export interface DeliveryStatus {
  channel: DeliveryChannel;
  status: 'pending' | 'sent' | 'delivered' | 'failed';
  sentAt?: Timestamp;
  deliveredAt?: Timestamp;
  error?: string;
}

// ============================================================================
// Escalation Types
// ============================================================================

export interface EscalationPolicy {
  policyId: string;
  name: string;
  levels: EscalationLevel[];
  defaultTimeout: number;
}

export interface EscalationLevel {
  level: number;
  recipients: RecipientID[];
  waitTimeSeconds: number;
  channels: DeliveryChannel[];
  trigger: EscalationTrigger;
}

export interface EscalationEvent {
  eventId: string;
  alertId: AlertID;
  fromLevel: number;
  toLevel: number;
  trigger: EscalationTrigger;
  timestamp: Timestamp;
  reason?: string;
}

// ============================================================================
// Vital Signs Types
// ============================================================================

export interface VitalSnapshot {
  timestamp: Timestamp;
  heartRate?: number;
  bloodPressure?: {
    systolic: number;
    diastolic: number;
  };
  temperature?: number;
  oxygenSaturation?: number;
  respiratoryRate?: number;
  glucoseLevel?: number;
}

// ============================================================================
// Request/Response Types
// ============================================================================

export interface CreateAlertRequest {
  patientId: PatientID;
  type: AlertType;
  priority: AlertPriority;
  title: string;
  message: string;
  recipients: RecipientID[];
  escalationPolicyId?: string;
  vitalSigns?: VitalSnapshot;
  location?: AlertLocation;
}

export interface CreateAlertResponse {
  alertId: AlertID;
  status: AlertStatus;
  sentTo: number;
  estimatedDelivery: Timestamp;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}

export class MedicalAlertError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'MedicalAlertError';
  }
}
