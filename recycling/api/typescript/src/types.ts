/**
 * WIA-ENE-023: Recycling System Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license CC BY 4.0
 * @description Type definitions for recycling data structures, facilities, and products
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ==================== Material Codes ====================

/**
 * Plastic material codes (PL)
 */
export type PlasticCode =
  | 'PL-01' // PET (Polyethylene Terephthalate)
  | 'PL-02' // HDPE (High-Density Polyethylene)
  | 'PL-03' // PVC (Polyvinyl Chloride)
  | 'PL-04' // LDPE (Low-Density Polyethylene)
  | 'PL-05' // PP (Polypropylene)
  | 'PL-06' // PS (Polystyrene)
  | 'PL-07'; // Other plastics

/**
 * Metal material codes (ME)
 */
export type MetalCode =
  | 'ME-01' // Aluminum
  | 'ME-02' // Iron/Steel
  | 'ME-03' // Copper
  | 'ME-04' // Brass
  | 'ME-05' // Stainless Steel
  | 'ME-06'; // Lead

/**
 * Paper material codes (PA)
 */
export type PaperCode =
  | 'PA-01' // Cardboard
  | 'PA-02' // Newspaper
  | 'PA-03' // Office Paper
  | 'PA-04' // Magazine/Glossy
  | 'PA-05' // Coated Paper
  | 'PA-06'; // Mixed Paper

/**
 * Glass material codes (GL)
 */
export type GlassCode =
  | 'GL-01' // Clear Glass
  | 'GL-02' // Green Glass
  | 'GL-03' // Brown Glass
  | 'GL-04'; // Mixed Glass

/**
 * Organic material codes (OR)
 */
export type OrganicCode =
  | 'OR-01' // Food Waste
  | 'OR-02' // Yard Waste
  | 'OR-03' // Wood
  | 'OR-04'; // Textile

/**
 * Electronic waste codes (EW)
 */
export type EWasteCode =
  | 'EW-01' // Large Appliances
  | 'EW-02' // Small Appliances
  | 'EW-03' // IT Equipment
  | 'EW-04' // Batteries
  | 'EW-05'; // Lighting

/**
 * All material codes
 */
export type MaterialCode =
  | PlasticCode
  | MetalCode
  | PaperCode
  | GlassCode
  | OrganicCode
  | EWasteCode;

// ==================== Core Data Structures ====================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
}

/**
 * Address information
 */
export interface Address {
  street: string;
  city: string;
  state: string;
  country: string; // ISO 3166-1 alpha-2 code
  postalCode: string;
}

/**
 * Location information
 */
export interface Location {
  facilityId: string;
  facilityName: string;
  address: Address;
  coordinates: Coordinates;
}

/**
 * Quantity measurement
 */
export interface Quantity {
  value: number;
  unit: 'kg' | 'tonnes' | 'lbs' | 'tons' | 'm3' | 'pieces';
}

/**
 * Quality grade classification
 */
export type QualityGrade = 'A' | 'B' | 'C' | 'D';

/**
 * Material quality information
 */
export interface MaterialQuality {
  grade: QualityGrade;
  contamination: number; // percentage (0-100)
  moisture: number; // percentage (0-100)
}

/**
 * Material source type
 */
export type SourceType = 'residential' | 'commercial' | 'industrial';

/**
 * Event type classification
 */
export type EventType = 'collection' | 'sorting' | 'processing' | 'remanufacturing';

/**
 * Material information in an event
 */
export interface MaterialInfo {
  materialCode: MaterialCode;
  materialName: string;
  quantity: Quantity;
  quality: MaterialQuality;
  source: SourceType;
  batchId: string;
}

/**
 * Performance metrics
 */
export interface Metrics {
  totalWeight: number; // kg
  recoveryRate: number; // percentage (0-100)
  contaminationRate: number; // percentage (0-100)
  processingTime: number; // minutes
}

/**
 * Certification information
 */
export interface Certification {
  certified: boolean;
  certificationBody: string;
  certificationNumber: string;
  validUntil: string; // ISO 8601 date
}

/**
 * Tracking chain of custody
 */
export interface Tracking {
  previousEventId?: string;
  nextFacilityId?: string;
  chainOfCustody: 'verified' | 'pending' | 'broken';
}

/**
 * Recycling event data structure
 */
export interface RecyclingEvent {
  eventId: string;
  timestamp: string; // ISO 8601 datetime
  eventType: EventType;
  location: Location;
  materials: MaterialInfo[];
  metrics: Metrics;
  certification: Certification;
  tracking: Tracking;
}

// ==================== Facility Structures ====================

/**
 * Facility type classification
 */
export type FacilityType = 'MRF' | 'sorting_center' | 'processor' | 'remanufacturer';

/**
 * Operational status
 */
export type OperationalStatus = 'active' | 'inactive' | 'maintenance';

/**
 * Capacity information
 */
export interface Capacity {
  daily: Quantity;
  annual: Quantity;
}

/**
 * Technology/equipment information
 */
export interface Technology {
  name: string;
  type: 'optical_sorter' | 'ai_inspection' | 'magnetic_separator' | 'eddy_current' | 'manual' | 'other';
  manufacturer: string;
  installDate: string; // ISO 8601 date
  accuracy: number; // percentage (0-100)
}

/**
 * Facility certification entry
 */
export interface FacilityCertification {
  type: 'ISO_14001' | 'ISO_14021' | 'ISO_15270' | 'WIA_ENE_023' | 'GRS' | 'SCS' | 'other';
  number: string;
  issueDate: string; // ISO 8601 date
  expiryDate: string; // ISO 8601 date
}

/**
 * Facility performance metrics
 */
export interface FacilityPerformance {
  averageRecoveryRate: number; // percentage (0-100)
  averageContamination: number; // percentage (0-100)
  averageUptime: number; // percentage (0-100)
  monthlyThroughput: number; // tonnes
}

/**
 * Contact information
 */
export interface Contact {
  manager: string;
  phone: string;
  email: string;
  website?: string;
}

/**
 * Recycling facility information
 */
export interface Facility {
  facilityId: string;
  facilityName: string;
  facilityType: FacilityType;
  operationalStatus: OperationalStatus;
  capacity: Capacity;
  technologies: Technology[];
  acceptedMaterials: MaterialCode[];
  certifications: FacilityCertification[];
  performance: FacilityPerformance;
  contact: Contact;
  location?: {
    address: Address;
    coordinates: Coordinates;
  };
}

// ==================== Recycled Product Structures ====================

/**
 * Manufacturer information
 */
export interface Manufacturer {
  id: string;
  name: string;
  country: string; // ISO 3166-1 alpha-2 code
}

/**
 * Material composition
 */
export interface Composition {
  materialCode: MaterialCode;
  percentage: number; // 0-100
  source: 'pre-consumer' | 'post-consumer';
  recycledContent: number; // percentage (0-100)
}

/**
 * Material properties
 */
export interface MaterialProperties {
  density?: number; // g/cm³
  tensileStrength?: number; // MPa
  meltingPoint?: number; // °C
  colorFastness?: number; // 1-5 scale
  [key: string]: number | undefined;
}

/**
 * Product certification
 */
export interface ProductCertification {
  type: 'GRS' | 'SCS' | 'Cradle2Cradle' | 'WIA_ENE_023' | 'other';
  number: string;
  recycledContent?: number; // percentage (0-100)
}

/**
 * Carbon footprint information
 */
export interface CarbonFootprint {
  totalEmissions: number; // kgCO2e per unit
  unit: string; // e.g., "kgCO2e/kg"
  savingsVsVirgin: number; // percentage (0-100)
}

/**
 * Blockchain traceability
 */
export interface BlockchainTrace {
  enabled: boolean;
  network?: 'Ethereum' | 'Polygon' | 'Hyperledger' | 'other';
  contractAddress?: string;
  tokenId?: string;
}

/**
 * Traceability information
 */
export interface Traceability {
  sourceBatches: string[];
  processingFacilities: string[];
  blockchain?: BlockchainTrace;
}

/**
 * Recycled product information
 */
export interface RecycledProduct {
  productId: string;
  productName: string;
  manufacturer: Manufacturer;
  composition: Composition[];
  properties: MaterialProperties;
  certifications: ProductCertification[];
  carbonFootprint: CarbonFootprint;
  traceability: Traceability;
}

// ==================== Analytics & Statistics ====================

/**
 * Time period for analytics
 */
export type AnalyticsPeriod = 'daily' | 'weekly' | 'monthly' | 'yearly';

/**
 * Recovery rate statistics
 */
export interface RecoveryRateStats {
  materialCode: MaterialCode;
  period: AnalyticsPeriod;
  country?: string;
  recoveryRate: number; // percentage (0-100)
  totalRecycled: number; // tonnes
  totalGenerated: number; // tonnes
  trend: 'increasing' | 'stable' | 'decreasing';
}

/**
 * Carbon savings calculation
 */
export interface CarbonSavings {
  facilityId: string;
  startDate: string; // ISO 8601 date
  endDate: string; // ISO 8601 date
  totalSavings: number; // kgCO2e
  savingsByMaterial: {
    materialCode: MaterialCode;
    savings: number; // kgCO2e
    quantity: number; // kg
  }[];
}

/**
 * Contamination alert
 */
export interface ContaminationAlert {
  facilityId: string;
  timestamp: string; // ISO 8601 datetime
  materialCode: MaterialCode;
  contaminationRate: number; // percentage (0-100)
  threshold: number; // percentage (0-100)
  severity: 'low' | 'medium' | 'high' | 'critical';
  action: string;
}

// ==================== API Request/Response Types ====================

/**
 * Create event request
 */
export interface CreateEventRequest {
  facilityId: string;
  eventType: EventType;
  materials: Omit<MaterialInfo, 'materialName'>[];
  metrics?: Partial<Metrics>;
}

/**
 * Create event response
 */
export interface CreateEventResponse {
  eventId: string;
  timestamp: string;
  status: 'success' | 'error';
  message?: string;
}

/**
 * Query parameters for events
 */
export interface EventQueryParams {
  facilityId?: string;
  materialCode?: MaterialCode;
  startDate?: string; // ISO 8601 date
  endDate?: string; // ISO 8601 date
  page?: number;
  limit?: number;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
  };
}

/**
 * Update facility performance request
 */
export interface UpdatePerformanceRequest {
  facilityId: string;
  performance: Partial<FacilityPerformance>;
}

/**
 * Trace chain response
 */
export interface TraceChain {
  productId: string;
  chain: {
    eventId: string;
    timestamp: string;
    facilityId: string;
    facilityName: string;
    eventType: EventType;
    materialCode: MaterialCode;
    quantity: Quantity;
  }[];
}

// ==================== Webhook Types ====================

/**
 * Webhook event types
 */
export type WebhookEventType =
  | 'event.created'
  | 'event.updated'
  | 'facility.performance.updated'
  | 'product.created'
  | 'alert.contamination.high';

/**
 * Webhook payload
 */
export interface WebhookPayload<T = any> {
  event: WebhookEventType;
  timestamp: string; // ISO 8601 datetime
  data: T;
}

// ==================== Client Configuration ====================

/**
 * API client configuration
 */
export interface ClientConfig {
  apiKey: string;
  endpoint?: string;
  timeout?: number; // milliseconds
  retries?: number;
}

/**
 * API error response
 */
export interface APIError {
  code: string;
  message: string;
  details?: any;
  timestamp: string;
}

// ==================== Export All ====================

export default {
  // Types are exported individually above
};
