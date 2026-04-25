/**
 * WIA-SOC-013 Public Document Standard - TypeScript Type Definitions
 *
 * @packageDocumentation
 */

export type DocumentType =
  | 'birthCertificate'
  | 'deathCertificate'
  | 'marriageCertificate'
  | 'passport'
  | 'nationalID'
  | 'driverLicense'
  | 'educationalDiploma'
  | 'propertyDeed'
  | 'taxDocument'
  | 'courtOrder';

export type SignatureAlgorithm =
  | 'ECDSA-SHA256'
  | 'RSA-SHA256'
  | 'EdDSA-Ed25519'
  | 'CRYSTALS-Dilithium';

export type BlockchainNetwork =
  | 'ethereum-mainnet'
  | 'polygon'
  | 'solana'
  | 'hyperledger-fabric';

export type DocumentFormat =
  | 'PDF/A-2b'
  | 'PDF/A-3b'
  | 'TIFF'
  | 'JSON'
  | 'XML';

export interface DocumentIdentifier {
  documentId: string; // Format: doc:wia:soc013:{type}:{country}:{year}:{uid}
  version: string;
  type: DocumentType;
}

export interface DocumentSubject {
  name: string;
  nameLocal?: string;
  nameRomanized?: string;
  dateOfBirth?: string; // YYYY-MM-DD
  placeOfBirth?: string;
  nationality?: string; // ISO 3166-1 alpha-3
  [key: string]: any;
}

export interface DocumentImage {
  pageNumber: number;
  format: 'image/jpeg' | 'image/png' | 'image/tiff';
  resolution: number; // DPI
  url: string;
  sha256: string;
}

export interface DublinCoreMetadata {
  title: string | Record<string, string>;
  creator: string;
  subject?: string | string[];
  description?: string | Record<string, string>;
  publisher?: string;
  contributor?: string | string[];
  date: string; // ISO 8601
  type: string;
  format: string;
  identifier: string;
  source?: string;
  language: string; // ISO 639-1/2
  relation?: string | string[];
  coverage?: string;
  rights?: string;
}

export interface PREMISMetadata {
  objectIdentifier: string;
  preservationLevel: 'full' | 'partial' | 'minimal';
  significantProperties: string[];
  environment: {
    software?: Record<string, any>[];
    hardware?: Record<string, any>;
  };
  events?: PREMISEvent[];
  agents?: PREMISAgent[];
  rights?: Record<string, any>;
}

export interface PREMISEvent {
  eventIdentifier: string;
  eventType: string;
  eventDateTime: string; // ISO 8601
  eventOutcome: 'success' | 'failure' | 'partial';
  eventDetail?: string;
  linkingAgentIdentifier?: string;
}

export interface PREMISAgent {
  agentIdentifier: string;
  agentName: string;
  agentType: 'person' | 'organization' | 'software';
  agentRole: string;
}

export interface DigitalSignature {
  algorithm: SignatureAlgorithm;
  value: string; // Base64-encoded signature
  publicKey: string; // PEM-encoded
  certificateChain: string[]; // Base64-encoded certificates
  timestamp: string; // RFC 3161 timestamp token
}

export interface BlockchainAnchor {
  network: BlockchainNetwork;
  transactionHash: string;
  blockNumber?: number;
  timestamp: string; // ISO 8601
  documentHash: string; // SHA-256
  smartContract?: string;
}

export interface PublicDocument {
  documentId: string;
  version: string;
  type: DocumentType;
  issuer: string; // DID format
  issuedAt: string; // ISO 8601
  expiresAt?: string; // ISO 8601
  subject: DocumentSubject;
  images: DocumentImage[];
  metadata: {
    dublinCore: DublinCoreMetadata;
    premis?: PREMISMetadata;
    custom?: Record<string, any>;
  };
  signatures: DigitalSignature[];
  blockchainAnchors: BlockchainAnchor[];
}

export interface DocumentCreationOptions {
  type: DocumentType;
  subject: DocumentSubject;
  language?: string;
  images?: File[] | string[];
  metadata?: Partial<DublinCoreMetadata>;
}

export interface DigitizationOptions {
  imageUrl: string | File;
  documentType: DocumentType;
  ocrLanguage?: string;
  outputFormat?: DocumentFormat;
  resolution?: number; // DPI
  deskew?: boolean;
  denoise?: boolean;
}

export interface SigningOptions {
  algorithm?: SignatureAlgorithm;
  privateKey: string | CryptoKey;
  certificateChain?: string[];
  timestamp?: boolean;
}

export interface VerificationOptions {
  checkRevocation?: boolean;
  validateSignature?: boolean;
  verifyBlockchainAnchor?: boolean;
  validateTimestamp?: boolean;
}

export interface VerificationResult {
  valid: boolean;
  signatureValid: boolean;
  notRevoked: boolean;
  blockchainValid: boolean;
  timestampValid: boolean;
  errors: string[];
}

export interface AccessControlPolicy {
  subject: {
    role?: string;
    department?: string;
    clearanceLevel?: number;
    [key: string]: any;
  };
  resource: {
    documentType: DocumentType;
    classification?: string;
  };
  action: 'create' | 'read' | 'update' | 'delete' | 'sign' | 'verify';
  environment?: {
    time?: string;
    location?: string;
    ipRange?: string;
    [key: string]: any;
  };
  decision: 'allow' | 'deny';
}

export interface AuditLogEntry {
  timestamp: string; // ISO 8601
  event: string;
  actor: string; // DID or username
  documentId: string;
  action: string;
  result: 'success' | 'failure';
  ipAddress?: string;
  userAgent?: string;
  details?: Record<string, any>;
}

export interface OCRResult {
  text: string;
  confidence: number;
  boundingBoxes: Array<{
    text: string;
    x: number;
    y: number;
    width: number;
    height: number;
    confidence: number;
  }>;
  language: string;
}

export interface MetadataExtraction {
  entities: Array<{
    type: 'PERSON' | 'DATE' | 'LOCATION' | 'ID_NUMBER';
    value: string;
    confidence: number;
  }>;
}

export interface ConfigOptions {
  apiUrl?: string;
  apiKey?: string;
  blockchain?: {
    network: BlockchainNetwork;
    rpcUrl?: string;
    contractAddress?: string;
  };
  storage?: {
    provider: 's3' | 'azure' | 'gcp';
    bucket: string;
    credentials?: any;
  };
}

export interface QualityMetrics {
  resolution: number;
  ocrConfidence: number;
  skewAngle: number;
  sharpness: number;
  passed: boolean;
  failures: Array<{
    check: string;
    expected: any;
    actual: any;
  }>;
}
