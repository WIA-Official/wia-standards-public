/**
 * WIA Global Health Data Standard - TypeScript Types
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 * © 2025 SmileStory Inc. / WIA
 */

export type Timestamp = string;
export type Status = 'active' | 'inactive' | 'pending' | 'completed';

// ============================================================================
// Disease Surveillance Types
// ============================================================================

export interface DiseaseCase {
  caseId: string;
  diseaseCode: string;
  diseaseName: string;
  icdCode: string;
  patientId: string;
  reportDate: Timestamp;
  onsetDate: Timestamp;
  diagnosisDate: Timestamp;
  location: GeoLocation;
  severity: Severity;
  outcome: Outcome;
  status: CaseStatus;
}

export interface GeoLocation {
  country: string;
  region: string;
  city?: string;
  latitude?: number;
  longitude?: number;
}

export enum Severity {
  MILD = 'mild',
  MODERATE = 'moderate',
  SEVERE = 'severe',
  CRITICAL = 'critical',
}

export enum Outcome {
  RECOVERED = 'recovered',
  ONGOING = 'ongoing',
  HOSPITALIZED = 'hospitalized',
  ICU = 'icu',
  DECEASED = 'deceased',
  UNKNOWN = 'unknown',
}

export enum CaseStatus {
  SUSPECTED = 'suspected',
  PROBABLE = 'probable',
  CONFIRMED = 'confirmed',
  DISCARDED = 'discarded',
}

// ============================================================================
// Epidemiology Types
// ============================================================================

export interface EpidemiologicalData {
  reportId: string;
  disease: string;
  region: string;
  period: ReportingPeriod;
  cases: number;
  deaths: number;
  recoveries: number;
  incidenceRate: number;
  mortalityRate: number;
  cfr: number;
  demographics: Demographics;
}

export interface ReportingPeriod {
  startDate: Timestamp;
  endDate: Timestamp;
  type: 'daily' | 'weekly' | 'monthly' | 'annual';
}

export interface Demographics {
  ageGroups: AgeGroup[];
  genderDistribution: GenderDistribution;
  riskFactors: RiskFactor[];
}

export interface AgeGroup {
  range: string;
  cases: number;
  percentage: number;
}

export interface GenderDistribution {
  male: number;
  female: number;
  other: number;
}

export interface RiskFactor {
  factor: string;
  prevalence: number;
  relativeRisk: number;
}

// ============================================================================
// Outbreak Types
// ============================================================================

export interface Outbreak {
  outbreakId: string;
  disease: string;
  startDate: Timestamp;
  endDate?: Timestamp;
  affectedRegions: string[];
  totalCases: number;
  totalDeaths: number;
  status: OutbreakStatus;
  responseLevel: ResponseLevel;
  timeline: OutbreakEvent[];
}

export enum OutbreakStatus {
  ONGOING = 'ongoing',
  CONTAINED = 'contained',
  ENDED = 'ended',
  MONITORING = 'monitoring',
}

export enum ResponseLevel {
  LOCAL = 'local',
  NATIONAL = 'national',
  INTERNATIONAL = 'international',
  PHEIC = 'pheic',
}

export interface OutbreakEvent {
  timestamp: Timestamp;
  type: string;
  description: string;
  source: string;
}

// ============================================================================
// Vaccination Types
// ============================================================================

export interface VaccinationData {
  region: string;
  date: Timestamp;
  vaccine: string;
  dosesAdministered: number;
  firstDose: number;
  secondDose: number;
  booster: number;
  coveragePercentage: number;
}

export interface VaccineInfo {
  vaccineId: string;
  name: string;
  manufacturer: string;
  type: VaccineType;
  doses: number;
  efficacy: number;
  storageTemp: number;
  approvals: string[];
}

export enum VaccineType {
  MRNA = 'mrna',
  VIRAL_VECTOR = 'viral_vector',
  INACTIVATED = 'inactivated',
  PROTEIN_SUBUNIT = 'protein_subunit',
  LIVE_ATTENUATED = 'live_attenuated',
}

// ============================================================================
// Health Facility Types
// ============================================================================

export interface HealthFacility {
  facilityId: string;
  name: string;
  type: FacilityType;
  location: GeoLocation;
  capacity: FacilityCapacity;
  services: string[];
  status: 'operational' | 'limited' | 'closed';
}

export enum FacilityType {
  HOSPITAL = 'hospital',
  CLINIC = 'clinic',
  LABORATORY = 'laboratory',
  PHARMACY = 'pharmacy',
  VACCINATION_CENTER = 'vaccination_center',
}

export interface FacilityCapacity {
  totalBeds: number;
  icuBeds: number;
  ventilators: number;
  occupancy: number;
}

// ============================================================================
// Data Record Types
// ============================================================================

export interface DataRecord {
  id: string;
  timestamp: Timestamp;
  status: Status;
  source: DataSource;
  quality: DataQuality;
  metadata?: Record<string, unknown>;
}

export interface DataSource {
  name: string;
  type: 'government' | 'hospital' | 'laboratory' | 'research';
  country: string;
  reliability: number;
}

export interface DataQuality {
  completeness: number;
  accuracy: number;
  timeliness: number;
  consistency: number;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
  requestId: string;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
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
