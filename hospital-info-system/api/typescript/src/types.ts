/**
 * WIA-MED-007: Hospital Information System Standard - TypeScript Types
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
export type PatientMRN = string;
export type AppointmentID = string;
export type AdmissionID = string;
export type EncounterID = string;

// ============================================================================
// Enums
// ============================================================================

export enum PatientStatus {
  REGISTERED = 'registered',
  ADMITTED = 'admitted',
  IN_TREATMENT = 'in_treatment',
  DISCHARGED = 'discharged',
}

export enum AppointmentStatus {
  SCHEDULED = 'scheduled',
  CONFIRMED = 'confirmed',
  CHECKED_IN = 'checked_in',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  CANCELLED = 'cancelled',
  NO_SHOW = 'no_show',
}

export enum OrderStatus {
  PENDING = 'pending',
  IN_PROGRESS = 'in_progress',
  COMPLETED = 'completed',
  CANCELLED = 'cancelled',
}

export enum BedStatus {
  AVAILABLE = 'available',
  OCCUPIED = 'occupied',
  RESERVED = 'reserved',
  MAINTENANCE = 'maintenance',
}

// ============================================================================
// Patient Types
// ============================================================================

export interface Patient {
  mrn: PatientMRN;
  name: PatientName;
  dateOfBirth: Timestamp;
  gender: 'male' | 'female' | 'other';
  contact: ContactInfo;
  insurance?: InsuranceInfo;
  allergies: Allergy[];
  status: PatientStatus;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

export interface PatientName {
  firstName: string;
  lastName: string;
  middleName?: string;
}

export interface ContactInfo {
  phone?: string;
  email?: string;
  address?: Address;
}

export interface Address {
  street1: string;
  city: string;
  state: string;
  zipCode: string;
  country: string;
}

export interface InsuranceInfo {
  provider: string;
  policyNumber: string;
  groupNumber?: string;
}

export interface Allergy {
  allergen: string;
  reaction: string;
  severity: 'mild' | 'moderate' | 'severe';
}

// ============================================================================
// Appointment Types
// ============================================================================

export interface Appointment {
  appointmentId: AppointmentID;
  patientMrn: PatientMRN;
  department: string;
  provider: string;
  scheduledTime: Timestamp;
  durationMinutes: number;
  reason: string;
  status: AppointmentStatus;
  notes?: string;
}

// ============================================================================
// Admission Types
// ============================================================================

export interface Admission {
  admissionId: AdmissionID;
  patientMrn: PatientMRN;
  admissionDate: Timestamp;
  dischargeDate?: Timestamp;
  department: string;
  attendingPhysician: string;
  diagnosis?: string;
  roomNumber?: string;
  bedNumber?: string;
  status: 'active' | 'discharged';
}

export interface BedAssignment {
  roomNumber: string;
  bedNumber: string;
  department: string;
  bedType: 'standard' | 'icu' | 'isolation' | 'maternity';
  status: BedStatus;
  patientMrn?: PatientMRN;
  assignedAt?: Timestamp;
}

// ============================================================================
// Clinical Order Types
// ============================================================================

export interface ClinicalOrder {
  orderId: string;
  patientMrn: PatientMRN;
  orderType: 'lab' | 'imaging' | 'medication';
  orderDescription: string;
  priority: 'routine' | 'urgent' | 'stat';
  orderedBy: string;
  orderedAt: Timestamp;
  status: OrderStatus;
}

// ============================================================================
// API Types
// ============================================================================

export interface HISConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  status: number;
  success: boolean;
  message: string;
  data?: T;
  timestamp: Timestamp;
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
