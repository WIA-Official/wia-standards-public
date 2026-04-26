/**
 * WIA-EDU-012 Micro-Credential Standard - TypeScript Type Definitions
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Micro-Credential Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '1.2' | '2.0';

/**
 * Competency proficiency levels
 */
export type ProficiencyLevel = 'novice' | 'developing' | 'proficient' | 'advanced' | 'expert';

/**
 * Credential visibility settings
 */
export type VisibilityLevel = 'public' | 'private' | 'selective' | 'restricted';

/**
 * Evidence types
 */
export type EvidenceType = 'Portfolio' | 'Assessment' | 'Project' | 'Performance' | 'Interview' | 'Observation';

/**
 * Open Badge Credential (Open Badges 3.0)
 */
export interface OpenBadgeCredential {
  '@context': string | string[];
  type: 'OpenBadgeCredential';
  id: string;
  issuer: IssuerProfile;
  issuanceDate: string;
  expirationDate?: string;
  credentialSubject: CredentialSubject;
  proof?: CryptographicProof;
  credentialStatus?: CredentialStatus;
  refreshService?: RefreshService;
  termsOfUse?: TermsOfUse[];
}

/**
 * Issuer Profile
 */
export interface IssuerProfile {
  id: string;
  type: 'Profile';
  name: string;
  url?: string;
  email?: string;
  phone?: string;
  description?: string;
  image?: string;
}

/**
 * Credential Subject (recipient and achievement)
 */
export interface CredentialSubject {
  id: string;
  type: 'AchievementSubject';
  achievement: Achievement;
  identifier?: Identifier[];
  image?: string;
  narrative?: string;
}

/**
 * Achievement Definition
 */
export interface Achievement {
  id: string;
  type: 'Achievement';
  name: string | LocalizedString;
  description: string | LocalizedString;
  criteria: Criteria;
  image?: string;
  achievementType?: string;
  alignment?: Alignment[];
  tag?: string[];
}

/**
 * Localized String (multi-language support)
 */
export interface LocalizedString {
  [languageCode: string]: string;
}

/**
 * Achievement Criteria
 */
export interface Criteria {
  narrative?: string;
  id?: string;
}

/**
 * Competency Alignment
 */
export interface Alignment {
  type: 'Alignment';
  targetName: string;
  targetUrl?: string;
  targetDescription?: string;
  targetFramework?: string;
  targetCode?: string;
}

/**
 * Identifier
 */
export interface Identifier {
  type: 'IdentityObject';
  identityHash: string;
  identityType: string;
  hashed: boolean;
  salt?: string;
}

/**
 * Cryptographic Proof
 */
export interface CryptographicProof {
  type: string;
  created: string;
  proofPurpose: string;
  verificationMethod: string;
  proofValue: string;
}

/**
 * Credential Status (for revocation checking)
 */
export interface CredentialStatus {
  id: string;
  type: string;
  statusPurpose?: string;
  statusListIndex?: string;
  statusListCredential?: string;
}

/**
 * Refresh Service (for renewable credentials)
 */
export interface RefreshService {
  id: string;
  type: string;
}

/**
 * Terms of Use
 */
export interface TermsOfUse {
  type: string;
  id?: string;
}

/**
 * WIA Extension: Competency Mapping
 */
export interface WIACompetency {
  framework: string;
  competencyId: string;
  competencyName: string | LocalizedString;
  proficiencyLevel: ProficiencyLevel;
  levelDefinition?: string;
  assessmentDate?: string;
}

/**
 * WIA Extension: Stackability Information
 */
export interface WIAStacking {
  stacksToward?: StackableCredential[];
  prerequisites?: PrerequisiteCredential[];
  equivalencies?: string[];
}

/**
 * Stackable Credential Reference
 */
export interface StackableCredential {
  credentialId: string;
  credentialName: string;
  progress?: string;
  weight?: number;
  required?: boolean;
}

/**
 * Prerequisite Credential
 */
export interface PrerequisiteCredential {
  credentialId: string;
  credentialName?: string;
  required: boolean;
  alternatives?: string[];
}

/**
 * WIA Extension: Evidence Attachment
 */
export interface WIAEvidence {
  type: EvidenceType;
  url?: string;
  description: string;
  verificationMethod?: string;
  score?: string;
  assessmentType?: string;
  assessor?: string;
  date?: string;
}

/**
 * WIA Extension: Privacy Controls
 */
export interface WIAPrivacy {
  visibility: VisibilityLevel;
  shareableWith?: string[];
  exposeEvidence: boolean;
  expirationDate?: string | null;
  allowBackdating: boolean;
}

/**
 * Complete WIA Micro-Credential with Extensions
 */
export interface WIAMicroCredential extends OpenBadgeCredential {
  'wia:competencies'?: WIACompetency[];
  'wia:stacking'?: WIAStacking;
  'wia:evidence'?: WIAEvidence[];
  'wia:privacy'?: WIAPrivacy;
  'wia:version'?: WIAVersion;
}

/**
 * Credential Issuance Request
 */
export interface CredentialIssuanceRequest {
  recipientId: string;
  recipientEmail?: string;
  achievementId: string;
  issuanceDate?: string;
  expirationDate?: string;
  evidence?: WIAEvidence[];
  competencies?: WIACompetency[];
  privacy?: WIAPrivacy;
  customMetadata?: Record<string, any>;
}

/**
 * Credential Verification Result
 */
export interface VerificationResult {
  valid: boolean;
  credentialId: string;
  issuer: string;
  recipient: string;
  issuedDate: string;
  expirationDate?: string;
  revoked: boolean;
  expired: boolean;
  signatureValid: boolean;
  message?: string;
  warnings?: string[];
}

/**
 * Batch Issuance Request
 */
export interface BatchIssuanceRequest {
  credentials: CredentialIssuanceRequest[];
  batchMetadata?: Record<string, any>;
}

/**
 * Batch Issuance Response
 */
export interface BatchIssuanceResponse {
  batchId: string;
  total: number;
  successful: number;
  failed: number;
  credentials: {
    credentialId?: string;
    status: 'issued' | 'failed';
    error?: string;
  }[];
}

/**
 * Credential Search Parameters
 */
export interface CredentialSearchParams {
  recipientId?: string;
  issuerId?: string;
  achievementId?: string;
  competencyFramework?: string;
  issuedAfter?: string;
  issuedBefore?: string;
  limit?: number;
  offset?: number;
  sortBy?: 'issuanceDate' | 'name' | 'issuer';
  sortOrder?: 'asc' | 'desc';
}

/**
 * Comprehensive Learner Record (CLR)
 */
export interface ComprehensiveLearnerRecord {
  '@context': string | string[];
  type: 'ClrCredential';
  id: string;
  credentialSubject: {
    id: string;
    verifiableCredential: WIAMicroCredential[];
    achievement?: Achievement[];
    association?: any[];
  };
  issuer: IssuerProfile;
  issuanceDate: string;
  proof?: CryptographicProof;
}

/**
 * API Client Configuration
 */
export interface WIAClientConfig {
  apiKey: string;
  baseURL?: string;
  environment?: 'production' | 'staging' | 'development';
  timeout?: number;
  retries?: number;
}

/**
 * Webhook Configuration
 */
export interface WebhookConfig {
  url: string;
  events: WebhookEvent[];
  secret: string;
  active?: boolean;
}

/**
 * Webhook Event Types
 */
export type WebhookEvent = 
  | 'credential.issued'
  | 'credential.verified'
  | 'credential.revoked'
  | 'credential.expired'
  | 'credential.renewed';

/**
 * Error Response
 */
export interface APIError {
  error: string;
  message: string;
  statusCode: number;
  details?: Record<string, any>;
}

// 弘益人間 - Benefit All Humanity
