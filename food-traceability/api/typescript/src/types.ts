/**
 * WIA Food Traceability Standard - TypeScript Types
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

export type Timestamp = string;
export type TraceEventType = 'HARVEST' | 'PROCESSING' | 'PACKAGING' | 'STORAGE' | 'TRANSPORT' | 'DISTRIBUTION' | 'RETAIL' | 'SALE';
export type VerificationStatus = 'VERIFIED' | 'PENDING' | 'FAILED' | 'EXPIRED';

// ============================================================================
// Product Types
// ============================================================================

export interface TraceableProduct {
  productId: string;
  gtin: string;
  productName: string;
  category: ProductCategory;
  origin: Origin;
  harvestDate: Timestamp;
  batchNumber: string;
  expiryDate: Timestamp;
  certifications: ProductCertification[];
  attributes: ProductAttribute[];
}

export enum ProductCategory {
  PRODUCE = 'produce',
  MEAT = 'meat',
  SEAFOOD = 'seafood',
  DAIRY = 'dairy',
  GRAINS = 'grains',
  BEVERAGES = 'beverages',
  PROCESSED = 'processed',
}

export interface Origin {
  country: string;
  region: string;
  farm?: FarmInfo;
}

export interface FarmInfo {
  farmId: string;
  name: string;
  location: GeoLocation;
  certifications: string[];
  practices: string[];
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  address: string;
}

export interface ProductCertification {
  type: CertificationType;
  issuer: string;
  issueDate: Timestamp;
  expiryDate: Timestamp;
  certificateNumber: string;
  verified: boolean;
}

export enum CertificationType {
  ORGANIC = 'organic',
  FAIR_TRADE = 'fair_trade',
  NON_GMO = 'non_gmo',
  RAINFOREST_ALLIANCE = 'rainforest_alliance',
  HALAL = 'halal',
  KOSHER = 'kosher',
  GAP = 'gap',
}

export interface ProductAttribute {
  name: string;
  value: string;
  unit?: string;
}

// ============================================================================
// Trace Event Types
// ============================================================================

export interface TraceEvent {
  eventId: string;
  productId: string;
  batchNumber: string;
  eventType: TraceEventType;
  timestamp: Timestamp;
  location: GeoLocation;
  actor: Actor;
  description: string;
  inputs?: TraceInput[];
  outputs?: TraceOutput[];
  conditions?: EnvironmentalConditions;
  blockchainHash?: string;
  documents?: Document[];
}

export interface Actor {
  actorId: string;
  name: string;
  type: ActorType;
  certifications?: string[];
  contact?: ContactInfo;
}

export enum ActorType {
  FARMER = 'farmer',
  PROCESSOR = 'processor',
  PACKAGER = 'packager',
  TRANSPORTER = 'transporter',
  DISTRIBUTOR = 'distributor',
  RETAILER = 'retailer',
  INSPECTOR = 'inspector',
}

export interface ContactInfo {
  email?: string;
  phone?: string;
  address?: string;
}

export interface TraceInput {
  productId: string;
  quantity: number;
  unit: string;
}

export interface TraceOutput {
  productId: string;
  quantity: number;
  unit: string;
}

export interface EnvironmentalConditions {
  temperature?: number;
  humidity?: number;
  light?: number;
  atmosphereComposition?: string;
}

export interface Document {
  documentId: string;
  type: DocumentType;
  name: string;
  url: string;
  hash?: string;
}

export enum DocumentType {
  CERTIFICATE = 'certificate',
  INVOICE = 'invoice',
  SHIPPING = 'shipping',
  INSPECTION = 'inspection',
  LAB_RESULT = 'lab_result',
}

// ============================================================================
// Trace Chain Types
// ============================================================================

export interface TraceChain {
  productId: string;
  batchNumber: string;
  events: TraceEvent[];
  totalEvents: number;
  verified: boolean;
  verificationDate?: Timestamp;
  completeness: number;
  gaps: TraceGap[];
}

export interface TraceGap {
  from: TraceEventType;
  to: TraceEventType;
  expectedDuration: number;
  actualDuration?: number;
  severity: 'minor' | 'major' | 'critical';
}

// ============================================================================
// QR and Verification Types
// ============================================================================

export interface QRCodeData {
  qrId: string;
  productId: string;
  batchNumber: string;
  expiryDate: Timestamp;
  traceUrl: string;
  generatedAt: Timestamp;
  scanCount: number;
}

export interface BlockchainRecord {
  transactionHash: string;
  blockNumber: number;
  network: string;
  timestamp: Timestamp;
  contractAddress: string;
  data: Record<string, unknown>;
  verified: boolean;
}

export interface VerifiableCredential {
  '@context': string[];
  type: string[];
  issuer: string;
  issuanceDate: Timestamp;
  expirationDate?: Timestamp;
  credentialSubject: CredentialSubject;
  proof: Proof;
}

export interface CredentialSubject {
  id: string;
  productId: string;
  batchNumber: string;
  origin: string;
  certifications?: string[];
}

export interface Proof {
  type: string;
  created: Timestamp;
  proofPurpose: string;
  verificationMethod: string;
  proofValue: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  timestamp: Timestamp;
  requestId: string;
}

export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
}

export interface TraceabilityQuery {
  productId?: string;
  batchNumber?: string;
  eventType?: TraceEventType;
  actor?: string;
  startDate?: Timestamp;
  endDate?: Timestamp;
  limit?: number;
  offset?: number;
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  pageSize: number;
  totalPages: number;
}

export interface SDKConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}
