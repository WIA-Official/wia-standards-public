/**
 * WIA-UNI-009 TypeScript Type Definitions
 * Healthcare Integration Standard
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type Region = 'north' | 'south';

export type Gender = 'male' | 'female' | 'other' | 'unknown';

export type Priority = 'routine' | 'urgent' | 'emergency' | 'critical';

export type RecordStatus = 'active' | 'inactive' | 'archived';

export type DiseaseSeverity = 'low' | 'moderate' | 'high' | 'critical';

export type TrustAnchorType = 'rok-moh' | 'dprk-moph' | 'who-observer' | 'icrc';

export type AccessPurpose =
  | 'emergency-treatment'
  | 'consultation'
  | 'followup-care'
  | 'research'
  | 'public-health';

// ============================================================================
// Patient & Identity
// ============================================================================

export interface PatientIdentity {
  patientId: string;
  region: Region;
  publicKey?: string;
  verified: boolean;
}

export interface Patient extends PatientIdentity {
  name?: string; // Protected/Encrypted
  dateOfBirth?: string; // Protected
  gender?: Gender;
  bloodType?: string;
  address?: Address;
  phone?: string;
  email?: string;
  registeredAt: Date;
  lastVisit?: Date;
}

export interface Address {
  region: Region;
  city?: string;
  text?: string; // Protected
}

// ============================================================================
// Medical Records
// ============================================================================

export interface MedicalRecord {
  id: string;
  patientId: string;
  recordType: 'encounter' | 'condition' | 'procedure' | 'observation';
  date: Date;
  facility: string;
  provider?: string;
  data: any; // FHIR resource
  encrypted: boolean;
  verificationHash: string;
}

export interface Allergy {
  id: string;
  allergen: string;
  type: 'drug' | 'food' | 'environmental';
  severity: 'mild' | 'moderate' | 'severe';
  reaction?: string;
  onsetDate?: Date;
}

export interface Condition {
  id: string;
  code: string; // ICD-10 or SNOMED
  name: string;
  category: 'chronic' | 'acute' | 'resolved';
  onsetDate: Date;
  resolvedDate?: Date;
  severity?: string;
}

export interface VitalSigns {
  timestamp: Date;
  bloodPressure?: {
    systolic: number;
    diastolic: number;
    unit: 'mmHg';
  };
  heartRate?: {
    value: number;
    unit: 'bpm';
  };
  temperature?: {
    value: number;
    unit: 'celsius' | 'fahrenheit';
  };
  respiratoryRate?: {
    value: number;
    unit: 'breaths/min';
  };
  oxygenSaturation?: {
    value: number;
    unit: 'percent';
  };
}

// ============================================================================
// Medications & Prescriptions
// ============================================================================

export interface Medication {
  id: string;
  genericName: string;
  brandNames: string[];
  activeIngredients: Ingredient[];
  therapeuticClass: string;
  controlledSubstance: boolean;
  approvedRegions: Region[];
}

export interface Ingredient {
  name: string;
  strength: string;
  unit: string;
}

export interface Prescription {
  id: string;
  patientId: string;
  medicationId: string;
  prescriber: string;
  facility: string;
  dosage: string;
  frequency: string;
  route: string;
  duration?: string;
  quantity: number;
  refills: number;
  prescribedDate: Date;
  startDate: Date;
  endDate?: Date;
  status: 'active' | 'completed' | 'cancelled';
  instructions?: string;
}

export interface DrugInteraction {
  drug1: string;
  drug2: string;
  severity: 'major' | 'moderate' | 'minor';
  type: 'pharmacokinetic' | 'pharmacodynamic';
  description: string;
  recommendation: string;
}

// ============================================================================
// Disease Surveillance
// ============================================================================

export interface DiseaseReport {
  id: string;
  disease: string;
  diseaseCode: string; // ICD-10
  caseCount: number;
  region: string;
  facility?: string;
  severity: DiseaseSeverity;
  reportDate: Date;
  reportedBy: string;
  verified: boolean;
  notes?: string;
}

export interface DiseaseAlert {
  id: string;
  disease: string;
  region: string;
  severity: DiseaseSeverity;
  activeCases: number;
  trend: 'increasing' | 'stable' | 'decreasing';
  alertLevel: 1 | 2 | 3 | 4; // WHO alert levels
  recommendations: string[];
  issuedDate: Date;
  expiryDate?: Date;
}

export interface OutbreakResponse {
  outbreakId: string;
  disease: string;
  affectedRegions: string[];
  coordinatedBy: TrustAnchorType[];
  status: 'investigating' | 'responding' | 'contained' | 'resolved';
  measures: ResponseMeasure[];
  startDate: Date;
  endDate?: Date;
}

export interface ResponseMeasure {
  type: 'quarantine' | 'vaccination' | 'contact-tracing' | 'public-health-messaging';
  description: string;
  implementedDate: Date;
  effectiveness?: string;
}

// ============================================================================
// Hospital Network
// ============================================================================

export interface Hospital {
  id: string;
  name: string;
  region: Region;
  type: 'general' | 'specialized' | 'emergency' | 'research';
  capacity: HospitalCapacity;
  specialties: string[];
  telemedicineEnabled: boolean;
  emergencyServices: boolean;
  location: {
    latitude: number;
    longitude: number;
    address: string;
  };
}

export interface HospitalCapacity {
  totalBeds: number;
  availableBeds: number;
  icuBeds: number;
  availableICUBeds: number;
  emergencyRoomCapacity: number;
  lastUpdated: Date;
}

export interface Consultation {
  id: string;
  patientId: string;
  requestingFacility: string;
  consultingFacility: string;
  specialty: string;
  urgency: Priority;
  scheduledTime?: Date;
  status: 'requested' | 'scheduled' | 'in-progress' | 'completed' | 'cancelled';
  method: 'video' | 'phone' | 'in-person' | 'asynchronous';
  notes?: string;
}

export interface PatientTransfer {
  id: string;
  patientId: string;
  fromFacility: string;
  toFacility: string;
  reason: string;
  urgency: Priority;
  transportMethod: 'ambulance' | 'helicopter' | 'patient-transport' | 'walking';
  status: 'requested' | 'approved' | 'in-transit' | 'completed' | 'cancelled';
  requestDate: Date;
  transferDate?: Date;
  arrivalDate?: Date;
}

// ============================================================================
// Authentication & Authorization
// ============================================================================

export interface AuthToken {
  token: string;
  type: 'Bearer';
  expiresAt: Date;
  refreshToken?: string;
}

export interface UserCredentials {
  userId: string;
  facility: string;
  region: Region;
  role: UserRole;
  privateKey?: string;
}

export type UserRole =
  | 'physician'
  | 'nurse'
  | 'pharmacist'
  | 'administrator'
  | 'researcher'
  | 'public-health-officer';

export interface AccessAuthorization {
  patientId: string;
  purpose: AccessPurpose;
  requestedBy: string;
  facility: string;
  duration?: number; // in minutes
  dataScope?: string[]; // specific data elements to access
}

// ============================================================================
// Blockchain & Verification
// ============================================================================

export interface VerificationProof {
  transactionId: string;
  blockHash: string;
  timestamp: Date;
  verifiedBy: TrustAnchorType[];
  signatures: TrustAnchorSignature[];
  dataHash: string;
  verified: boolean;
}

export interface TrustAnchorSignature {
  anchor: TrustAnchorType;
  signature: string;
  publicKey: string;
  timestamp: Date;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  metadata?: {
    requestId: string;
    timestamp: Date;
    version: string;
  };
}

export interface APIError {
  code: string;
  message: string;
  details?: any;
}

export interface PaginationParams {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

// ============================================================================
// Configuration
// ============================================================================

export interface HealthcareIntegrationConfig {
  apiKey: string;
  apiUrl?: string;
  region: Region;
  facility: string;
  environment?: 'development' | 'staging' | 'production';
  trustAnchors?: TrustAnchorType[];
  encryption?: 'military-grade' | 'standard';
  fhirVersion?: 'R4' | 'R5';
  privacyLevel?: 'maximum' | 'standard';
  timeout?: number;
}
