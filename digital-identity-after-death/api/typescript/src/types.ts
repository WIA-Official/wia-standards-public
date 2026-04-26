/**
 * WIA-LEG-010 Digital Identity After Death Standard - TypeScript Type Definitions
 * @module @wia/digital-identity-after-death
 */

/**
 * Network/blockchain identifier
 */
export type Network = 'mainnet' | 'testnet' | 'devnet';

/**
 * Jurisdiction code (ISO 3166-1 alpha-2 + subdivision)
 */
export type Jurisdiction = string;

/**
 * Death verification method
 */
export type VerificationMethod = 'api' | 'certificate' | 'manual';

/**
 * Verification source type
 */
export type SourceType = 'government' | 'medical' | 'legal' | 'commercial' | 'social';

/**
 * Executor credential type
 */
export type ExecutorRole = 'primary_executor' | 'co_executor' | 'beneficiary' | 'advisor';

/**
 * AI Persona type
 */
export type PersonaType = 'static' | 'retrieval' | 'generative_text' | 'multimodal' | 'full_avatar';

/**
 * Memorial type
 */
export type MemorialType = 'public' | 'private' | 'family_only' | 'invitation_only';

/**
 * Client configuration options
 */
export interface ClientConfig {
  /** API key for authentication */
  apiKey: string;
  /** Base URL for API endpoints */
  baseUrl?: string;
  /** Network environment */
  network?: Network;
  /** Default jurisdiction for operations */
  jurisdiction?: Jurisdiction;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Enable debug logging */
  debug?: boolean;
}

/**
 * Death certificate data structure
 */
export interface DeathCertificate {
  /** Unique certificate identifier */
  certificateId: string;
  /** Jurisdiction where death occurred */
  jurisdiction: Jurisdiction;
  /** Issuing authority */
  issuingAuthority: string;
  /** Deceased person's identity information */
  deceasedIdentity: {
    legalName: string;
    dateOfBirth: string;
    nationalId: string;
    digitalIdentifiers?: DigitalIdentifier[];
  };
  /** Death event information */
  deathEvent: {
    dateOfDeath: string;
    placeOfDeath?: {
      jurisdiction: Jurisdiction;
      facility?: string;
    };
  };
  /** Certification information */
  certification: {
    certifierType: 'physician' | 'medical_examiner' | 'coroner';
    certifierName: string;
    certificationDate: string;
    digitalSignature?: string;
  };
  /** Verification metadata */
  verification?: VerificationResult;
}

/**
 * Digital identifier (email, phone, DID, etc.)
 */
export interface DigitalIdentifier {
  type: 'DID' | 'email' | 'phoneNumber' | 'governmentID';
  value: string;
  verified: boolean;
}

/**
 * Death verification result
 */
export interface VerificationResult {
  verified: boolean;
  verificationMethod: VerificationMethod;
  verificationSources: string[];
  consensusLevel: number;
  consensusScore: number;
  verificationTimestamp: string;
  blockchainAnchor?: BlockchainAnchor;
}

/**
 * Blockchain proof anchor
 */
export interface BlockchainAnchor {
  chain: string;
  contract?: string;
  transactionHash: string;
  blockNumber: number;
  timestamp: string;
}

/**
 * Proof-of-death structure
 */
export interface ProofOfDeath {
  proofType: string;
  subjectDID: string;
  certificateHash: string;
  verificationHash: string;
  consensusScore: number;
  timestamp: string;
  merkleRoot: string;
  signature: string;
  blockchain: BlockchainAnchor;
}

/**
 * Executor credential
 */
export interface ExecutorCredential {
  credentialId: string;
  credentialType: 'executor_access';
  executorId: string;
  deceasedId: string;
  issuedAt: string;
  expiresAt: string;
  permissions: ('read' | 'export' | 'delete' | 'modify')[];
  requiresMultiSig: boolean;
  multiSigConfig?: {
    requiredSignatures: number;
    signers: string[];
  };
  legalBasis: {
    documentType: string;
    documentId: string;
    issuingCourt?: string;
    verificationHash: string;
  };
}

/**
 * Credential revocation request
 */
export interface RevocationRequest {
  subjectId: string;
  scope: 'global' | 'service_specific';
  serviceIds?: string[];
  reason?: string;
}

/**
 * Credential revocation result
 */
export interface RevocationResult {
  totalDiscovered: number;
  successfulRevocations: number;
  failedRevocations: number;
  successRate: number;
  revokedCredentials: string[];
  failedCredentials: string[];
  auditLog: RevocationAuditEntry[];
  timestamp: string;
}

/**
 * Revocation audit log entry
 */
export interface RevocationAuditEntry {
  credential: string;
  status: 'revoked' | 'failed' | 'error';
  timestamp: string;
  method?: string;
  reason?: string;
  error?: string;
}

/**
 * AI Persona configuration
 */
export interface PersonaConfig {
  personaId?: string;
  sourceId: string;
  personaType: PersonaType;
  trainingDataSources: string[];
  excludedData?: string[];
  allowedInteractions: {
    textChat: boolean;
    voiceConversation: boolean;
    publicDisplay: boolean;
    commercialUse: boolean;
  };
  authorizedUsers: {
    family?: string[];
    friends?: string[];
    public?: boolean;
  };
  ethicalBoundaries: {
    noHarmfulContent: boolean;
    noMisinformation: boolean;
    noCommercialEndorsements: boolean;
    requireAttributionAsAI: boolean;
  };
}

/**
 * AI Persona
 */
export interface Persona {
  personaId: string;
  sourceId: string;
  createdAt: string;
  model: string;
  trainingData: {
    sources: string[];
    size: string;
    privacy: 'encrypted' | 'anonymized' | 'public';
  };
  interactionRules: {
    consentRequired: boolean;
    familyOnly: boolean;
    maxSessionDuration?: number;
  };
  metadata?: Record<string, any>;
}

/**
 * Memorial configuration
 */
export interface MemorialConfig {
  deceasedId: string;
  memorialType: MemorialType;
  name: string;
  biography?: string;
  lifespan: {
    birth: string;
    death: string;
  };
  preservationDuration?: 'permanent' | string; // e.g., "10-years"
  publicAccess?: boolean;
  familyMembers?: string[];
  authorizedViewers?: string[];
}

/**
 * Memorial platform
 */
export interface Memorial {
  memorialId: string;
  deceasedId: string;
  memorialType: MemorialType;
  createdAt: string;
  updatedAt: string;
  name: string;
  biography?: string;
  lifespan: {
    birth: string;
    death: string;
  };
  contentCount: {
    photos: number;
    videos: number;
    tributes: number;
  };
  url?: string;
}

/**
 * Tribute/memory submission
 */
export interface Tribute {
  tributeId?: string;
  memorialId: string;
  contributorId: string;
  contributorName: string;
  content: string;
  type: 'text' | 'photo' | 'video' | 'audio';
  timestamp?: string;
  relationshipToDeceased?: string;
  approved?: boolean;
}

/**
 * Audit log entry for posthumous access
 */
export interface AuditLogEntry {
  auditId: string;
  timestamp: string;
  eventType: string;
  deceasedId: string;
  accessorId: string;
  accessorRole: ExecutorRole;
  operation: string;
  resourceId: string;
  resourceType: string;
  authorizationMethod: string;
  multiSigRequired?: boolean;
  multiSigSignatures?: Array<{
    signerId: string;
    timestamp: string;
  }>;
  legalBasis?: {
    documentType: string;
    jurisdiction: Jurisdiction;
    courtId?: string;
  };
  accessDetails?: {
    duration: number;
    dataAccessed: string;
    dataExported: boolean;
    ipAddress: string;
    userAgent: string;
  };
  blockchainAnchor?: BlockchainAnchor;
  signature?: string;
}

/**
 * API Response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: string;
    requestId: string;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page?: number;
  limit?: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}
