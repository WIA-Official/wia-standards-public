/**
 * WIA-ORGAN-SHORTAGE TypeScript Type Definitions
 * @version 1.0.0
 * @license MIT
 * @description Type definitions for the WIA-ORGAN-SHORTAGE Standard
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export type OrganType = 'kidney' | 'liver' | 'heart' | 'lung' | 'pancreas' | 'intestine';

export type UrgencyLevel = 'urgent' | 'high' | 'medium' | 'stable';

export type BloodType = 'A' | 'B' | 'AB' | 'O';

export type RhFactor = 'positive' | 'negative';

export type HLATypingMethod = 'serology' | 'molecular-sso' | 'molecular-ssp' | 'ngs';

export type HLAResolution = 'low' | 'intermediate' | 'high';

export type CellSource = 'autologous' | 'allogeneic' | 'xenogeneic' | 'ipsc';

export type VascularizationStatus = 'none' | 'partial' | 'complete';

export type GeneModificationType = 'knockout' | 'insertion' | 'inactivation';

export type MessageType =
  | 'organ.available'
  | 'organ.accepted'
  | 'organ.declined'
  | 'match.request'
  | 'match.result'
  | 'eligibility.check'
  | 'eligibility.result'
  | 'alert.urgent'
  | 'heartbeat.ping'
  | 'heartbeat.pong';

export type MessagePriority = 'critical' | 'high' | 'normal' | 'low';

// ============================================================================
// HLA Typing
// ============================================================================

export interface HLAClass {
  A?: string[];
  B?: string[];
  C?: string[];
}

export interface HLAClassII {
  DR?: string[];
  DQ?: string[];
  DP?: string[];
}

export interface HLATyping {
  classI?: HLAClass;
  classII?: HLAClassII;
  typingMethod?: HLATypingMethod;
  resolution?: HLAResolution;
}

// ============================================================================
// Immunological Profile
// ============================================================================

export interface CrossmatchResult {
  donorId: string;
  result: 'positive' | 'negative';
  date: string;
  method: string;
}

export interface ImmunologicalProfile {
  bloodType: BloodType;
  rhFactor?: RhFactor;
  hlaTyping?: HLATyping;
  praPercent?: number;
  crossmatchHistory?: CrossmatchResult[];
  unacceptableAntigens?: string[];
}

// ============================================================================
// Waitlist Status
// ============================================================================

export interface WaitlistStatus {
  registeredDate: string;
  unosStatus?: string;
  waitingTimeDays: number;
  geographicRegion?: string;
  listingCenter?: string;
  multiListing?: boolean;
}

// ============================================================================
// Alternative Options
// ============================================================================

export interface AlternativeOptions {
  xenotransplantEligible: boolean;
  bioprintedOrganEligible: boolean;
  livingDonorAvailable: boolean;
  dominoTransplantOption: boolean;
  pairedExchangeEnrolled?: boolean;
}

// ============================================================================
// Xenotransplant Profile
// ============================================================================

export interface GeneticModification {
  gene: string;
  modificationType: GeneModificationType;
  purpose: string;
}

export interface PigSource {
  breed: string;
  facility: string;
  certification?: string;
}

export interface XenotransplantProfile {
  geneticModifications?: GeneticModification[];
  trialEnrollment?: string;
  immunosuppressionProtocol?: string;
  sourceFacility?: string;
  pigSource?: PigSource;
}

// ============================================================================
// Bioprinted Organ Profile
// ============================================================================

export interface QualityMetrics {
  cellViability: number;
  structuralIntegrity: number;
  vascularPerfusion?: number;
}

export interface BioprintedOrganProfile {
  organType: string;
  vascularizationStatus: VascularizationStatus;
  maturationDays: number;
  functionalityScore: number;
  cellSource: CellSource;
  bioinkComposition?: string[];
  printingFacility?: string;
  qualityMetrics?: QualityMetrics;
}

// ============================================================================
// Organ Profile
// ============================================================================

export interface OrganProfile {
  patientId: string;
  organNeeded: OrganType;
  urgency: UrgencyLevel;
  waitlistStatus?: WaitlistStatus;
  immunological: ImmunologicalProfile;
  alternativeOptions?: AlternativeOptions;
  xenotransplant?: XenotransplantProfile;
  bioprintedOrgan?: BioprintedOrganProfile;
}

// ============================================================================
// API Types
// ============================================================================

export interface PaginationParams {
  limit?: number;
  offset?: number;
}

export interface PaginationResponse {
  total: number;
  limit: number;
  offset: number;
  hasMore: boolean;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: PaginationResponse;
}

export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
  requestId: string;
}

// ============================================================================
// Patient Types
// ============================================================================

export interface CreatePatientRequest {
  organNeeded: OrganType;
  urgency: UrgencyLevel;
  immunological: ImmunologicalProfile;
  listingCenter?: string;
}

export interface PatientSummary {
  id: string;
  organNeeded: OrganType;
  urgency: UrgencyLevel;
  bloodType: BloodType;
  waitingDays: number;
  praPercent?: number;
  createdAt: string;
}

// ============================================================================
// Eligibility Types
// ============================================================================

export interface EligibilityRequest {
  patientId: string;
  organNeeded: OrganType;
  praPercent?: number;
  waitingYears?: number;
  contraindications?: string[];
}

export interface ClinicalTrial {
  id: string;
  sponsor: string;
  organ: OrganType;
  phase: string;
  status: 'recruiting' | 'active' | 'completed' | 'suspended';
  locations?: string[];
}

export interface EligibilityResult {
  eligible: boolean;
  score: number;
  reasons: string[];
  availableTrials: ClinicalTrial[];
  recommendations: string[];
}

// ============================================================================
// Matching Types
// ============================================================================

export interface MatchRequest {
  patientId: string;
  includeTraditional?: boolean;
  includeXenotransplant?: boolean;
  includeBioprinted?: boolean;
  maxResults?: number;
}

export interface Match {
  type: 'deceased_donor' | 'living_donor' | 'xenotransplant' | 'bioprinted';
  donorId?: string;
  matchScore: number;
  bloodCompatible: boolean;
  hlaMatches?: number;
  crossmatch?: 'positive' | 'negative' | 'pending';
  distanceKm?: number;
  coldIschemiaHours?: number;
  trialId?: string;
  availability?: string;
}

export interface MatchResult {
  matches: Match[];
  algorithmVersion: string;
  computedAt: string;
}

// ============================================================================
// Bioprint Order Types
// ============================================================================

export interface BioprintOrderRequest {
  patientId: string;
  organType: OrganType;
  cellSource: CellSource;
  priority: 'urgent' | 'standard';
  specifications?: {
    sizeMl?: number;
    vascularizationRequired?: boolean;
  };
}

export interface BioprintOrder {
  orderId: string;
  status: 'queued' | 'printing' | 'maturation' | 'quality_check' | 'ready' | 'delivered';
  progressPercent?: number;
  currentPhase?: string;
  qualityMetrics?: QualityMetrics;
  estimatedCompletion?: string;
  facility?: string;
  trackingUrl?: string;
}

// ============================================================================
// Protocol Types
// ============================================================================

export interface MessageHeader {
  version: string;
  messageId: string;
  type: MessageType;
  timestamp: string;
  source?: {
    id: string;
    type: string;
  };
  destination?: {
    id: string;
    type: string;
  };
  priority?: MessagePriority;
  ttl?: number;
}

export interface MessageSecurity {
  encryption?: 'none' | 'AES-256-GCM';
  keyId?: string;
  iv?: string;
  signature?: string;
}

export interface ProtocolMessage<T = unknown> {
  header: MessageHeader;
  payload: T;
  security?: MessageSecurity;
}

// ============================================================================
// Webhook Types
// ============================================================================

export type WebhookEventType =
  | 'organ.available'
  | 'match.found'
  | 'trial.opening'
  | 'bioprint.complete';

export interface WebhookPayload<T = unknown> {
  id: string;
  type: WebhookEventType;
  timestamp: string;
  data: T;
  signature: string;
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WiaOrganShortageConfig {
  apiKey: string;
  environment?: 'production' | 'sandbox';
  baseUrl?: string;
  timeout?: number;
  retry?: {
    maxRetries: number;
    baseDelay: number;
  };
}

// ============================================================================
// Metadata
// ============================================================================

export interface Metadata {
  standard: string;
  version: string;
  philosophy: string;
  createdAt?: string;
  updatedAt?: string;
}

export default {};
