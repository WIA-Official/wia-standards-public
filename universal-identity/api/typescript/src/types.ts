/**
 * WIA-CORE-001: Universal Identity - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Identity Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Trust levels for progressive authentication
 */
export type TrustLevel =
  | 0  // Anonymous - No verification
  | 1  // Email Verified - Basic verification
  | 2  // Multi-Factor - Enhanced security
  | 3  // Identity Verified - Government ID checked
  | 4  // Biometric Verified - Biometric authentication
  | 5; // Legally Bound - Digital signature with legal standing

/**
 * Identity types supported by the standard
 */
export type IdentityType =
  | 'anonymous'
  | 'email'
  | 'social'
  | 'enterprise'
  | 'government'
  | 'decentralized';

/**
 * Federation protocols
 */
export type FederationProtocol =
  | 'oauth2'
  | 'oidc'
  | 'saml'
  | 'webauthn'
  | 'did';

// ============================================================================
// Identity Core
// ============================================================================

/**
 * Universal identity
 */
export interface UniversalIdentity {
  /** Unique identifier (DID or UUID) */
  id: string;

  /** Identity type */
  type: IdentityType;

  /** Display name */
  displayName?: string;

  /** Trust level */
  trustLevel: TrustLevel;

  /** Identity attributes */
  attributes: IdentityAttributes;

  /** Linked providers */
  linkedProviders: LinkedProvider[];

  /** Verifiable credentials */
  credentials: VerifiableCredential[];

  /** Created timestamp */
  createdAt: Date;

  /** Last updated timestamp */
  updatedAt: Date;

  /** Status */
  status: 'active' | 'suspended' | 'revoked';
}

/**
 * Identity attributes
 */
export interface IdentityAttributes {
  /** Email address */
  email?: string;

  /** Email verified */
  emailVerified?: boolean;

  /** Phone number */
  phone?: string;

  /** Phone verified */
  phoneVerified?: boolean;

  /** Full name */
  name?: string;

  /** Given name */
  givenName?: string;

  /** Family name */
  familyName?: string;

  /** Date of birth */
  birthDate?: string;

  /** Gender */
  gender?: string;

  /** Profile picture URL */
  picture?: string;

  /** Locale */
  locale?: string;

  /** Timezone */
  timezone?: string;

  /** Custom attributes */
  custom?: Record<string, unknown>;
}

/**
 * Linked identity provider
 */
export interface LinkedProvider {
  /** Provider type */
  type: FederationProtocol;

  /** Provider name */
  provider: string;

  /** Provider-specific user ID */
  providerId: string;

  /** Provider username */
  username?: string;

  /** Linked timestamp */
  linkedAt: Date;

  /** Last used timestamp */
  lastUsedAt?: Date;

  /** Provider-specific metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Decentralized Identifiers (DIDs)
// ============================================================================

/**
 * DID (Decentralized Identifier)
 */
export interface DID {
  /** DID string (e.g., did:web:example.com:users:alice) */
  id: string;

  /** DID method */
  method: 'web' | 'key' | 'ethr' | 'ion' | string;

  /** DID document */
  document: DIDDocument;

  /** Created timestamp */
  createdAt: Date;

  /** Deactivated */
  deactivated?: boolean;
}

/**
 * DID Document (W3C DID Core)
 */
export interface DIDDocument {
  /** DID subject */
  id: string;

  /** Also known as */
  alsoKnownAs?: string[];

  /** Controller DIDs */
  controller?: string | string[];

  /** Verification methods */
  verificationMethod?: VerificationMethod[];

  /** Authentication methods */
  authentication?: (string | VerificationMethod)[];

  /** Assertion methods */
  assertionMethod?: (string | VerificationMethod)[];

  /** Key agreement */
  keyAgreement?: (string | VerificationMethod)[];

  /** Capability invocation */
  capabilityInvocation?: (string | VerificationMethod)[];

  /** Capability delegation */
  capabilityDelegation?: (string | VerificationMethod)[];

  /** Service endpoints */
  service?: ServiceEndpoint[];

  /** Context */
  '@context'?: string | string[];
}

/**
 * Verification method
 */
export interface VerificationMethod {
  /** Method ID */
  id: string;

  /** Type */
  type: string;

  /** Controller DID */
  controller: string;

  /** Public key JWK */
  publicKeyJwk?: JsonWebKey;

  /** Public key multibase */
  publicKeyMultibase?: string;

  /** Blockchain account ID */
  blockchainAccountId?: string;
}

/**
 * Service endpoint
 */
export interface ServiceEndpoint {
  /** Service ID */
  id: string;

  /** Service type */
  type: string;

  /** Service endpoint URL */
  serviceEndpoint: string | string[] | Record<string, unknown>;

  /** Description */
  description?: string;
}

// ============================================================================
// Verifiable Credentials
// ============================================================================

/**
 * Verifiable credential (W3C VC)
 */
export interface VerifiableCredential {
  /** Context */
  '@context': string | string[];

  /** Credential ID */
  id: string;

  /** Credential types */
  type: string[];

  /** Issuer DID */
  issuer: string;

  /** Issuance date */
  issuanceDate: Date;

  /** Expiration date */
  expirationDate?: Date;

  /** Credential subject */
  credentialSubject: CredentialSubject;

  /** Proof */
  proof?: CredentialProof;

  /** Status */
  credentialStatus?: CredentialStatus;

  /** Terms of use */
  termsOfUse?: TermsOfUse[];

  /** Evidence */
  evidence?: Evidence[];

  /** Refresh service */
  refreshService?: RefreshService;
}

/**
 * Credential subject
 */
export interface CredentialSubject {
  /** Subject DID */
  id?: string;

  /** Claims */
  [key: string]: unknown;
}

/**
 * Credential proof
 */
export interface CredentialProof {
  /** Proof type */
  type: string;

  /** Created timestamp */
  created: Date;

  /** Verification method */
  verificationMethod: string;

  /** Proof purpose */
  proofPurpose: string;

  /** JWS signature */
  jws?: string;

  /** Proof value */
  proofValue?: string;
}

/**
 * Credential status
 */
export interface CredentialStatus {
  /** Status ID */
  id: string;

  /** Status type */
  type: string;

  /** Status list index */
  statusListIndex?: string;

  /** Status list credential */
  statusListCredential?: string;
}

/**
 * Terms of use
 */
export interface TermsOfUse {
  /** Type */
  type: string;

  /** Terms ID */
  id?: string;

  /** Profile */
  profile?: string;

  /** Prohibition */
  prohibition?: Prohibition[];
}

/**
 * Prohibition
 */
export interface Prohibition {
  /** Assigner */
  assigner?: string;

  /** Assignee */
  assignee?: string;

  /** Target */
  target?: string;

  /** Action */
  action?: string[];
}

/**
 * Evidence
 */
export interface Evidence {
  /** Evidence ID */
  id?: string;

  /** Evidence type */
  type: string[];

  /** Verifier */
  verifier?: string;

  /** Evidence document */
  evidenceDocument?: string;

  /** Subject presence */
  subjectPresence?: string;

  /** Document presence */
  documentPresence?: string;
}

/**
 * Refresh service
 */
export interface RefreshService {
  /** Service ID */
  id: string;

  /** Service type */
  type: string;
}

// ============================================================================
// Verifiable Presentation
// ============================================================================

/**
 * Verifiable presentation
 */
export interface VerifiablePresentation {
  /** Context */
  '@context': string | string[];

  /** Presentation ID */
  id?: string;

  /** Presentation types */
  type: string[];

  /** Verifiable credentials */
  verifiableCredential: VerifiableCredential[];

  /** Holder DID */
  holder?: string;

  /** Proof */
  proof?: CredentialProof;
}

// ============================================================================
// OAuth 2.0 / 2.1
// ============================================================================

/**
 * OAuth provider configuration
 */
export interface OAuthProvider {
  /** Client ID */
  clientId: string;

  /** Client secret */
  clientSecret: string;

  /** Authorization URL */
  authorizationUrl: string;

  /** Token URL */
  tokenUrl: string;

  /** Revocation URL */
  revocationUrl?: string;

  /** Introspection URL */
  introspectionUrl?: string;

  /** Scope */
  scope?: string[];

  /** PKCE enabled */
  pkce?: boolean;
}

/**
 * OAuth authorization request
 */
export interface OAuthAuthorizationRequest {
  /** Response type */
  responseType: 'code' | 'token';

  /** Client ID */
  clientId: string;

  /** Redirect URI */
  redirectUri: string;

  /** Scope */
  scope?: string[];

  /** State */
  state: string;

  /** Code challenge (PKCE) */
  codeChallenge?: string;

  /** Code challenge method (PKCE) */
  codeChallengeMethod?: 'S256' | 'plain';
}

/**
 * OAuth token response
 */
export interface OAuthTokenResponse {
  /** Access token */
  accessToken: string;

  /** Token type */
  tokenType: 'Bearer';

  /** Expires in (seconds) */
  expiresIn: number;

  /** Refresh token */
  refreshToken?: string;

  /** Scope */
  scope?: string;

  /** ID token (OIDC) */
  idToken?: string;
}

// ============================================================================
// OpenID Connect
// ============================================================================

/**
 * OIDC provider configuration
 */
export interface OIDCProvider extends OAuthProvider {
  /** Issuer URL */
  issuer: string;

  /** UserInfo URL */
  userInfoUrl: string;

  /** JWKS URI */
  jwksUri: string;

  /** End session URL */
  endSessionUrl?: string;
}

/**
 * OIDC ID token claims
 */
export interface IDTokenClaims {
  /** Issuer */
  iss: string;

  /** Subject */
  sub: string;

  /** Audience */
  aud: string | string[];

  /** Expiration */
  exp: number;

  /** Issued at */
  iat: number;

  /** Auth time */
  authTime?: number;

  /** Nonce */
  nonce?: string;

  /** ACR */
  acr?: string;

  /** AMR */
  amr?: string[];

  /** AZP */
  azp?: string;

  /** Profile claims */
  name?: string;
  givenName?: string;
  familyName?: string;
  middleName?: string;
  nickname?: string;
  preferredUsername?: string;
  profile?: string;
  picture?: string;
  website?: string;
  email?: string;
  emailVerified?: boolean;
  gender?: string;
  birthdate?: string;
  zoneinfo?: string;
  locale?: string;
  phoneNumber?: string;
  phoneNumberVerified?: boolean;
  address?: Address;
  updatedAt?: number;
}

/**
 * Address
 */
export interface Address {
  /** Formatted address */
  formatted?: string;

  /** Street address */
  streetAddress?: string;

  /** Locality */
  locality?: string;

  /** Region */
  region?: string;

  /** Postal code */
  postalCode?: string;

  /** Country */
  country?: string;
}

// ============================================================================
// SAML 2.0
// ============================================================================

/**
 * SAML provider configuration
 */
export interface SAMLProvider {
  /** Entry point (SSO URL) */
  entryPoint: string;

  /** Issuer (Entity ID) */
  issuer: string;

  /** Certificate */
  cert: string;

  /** Private key */
  privateKey?: string;

  /** Assertion consumer service URL */
  assertionConsumerServiceUrl: string;

  /** Single logout URL */
  singleLogoutUrl?: string;

  /** Name ID format */
  nameIdFormat?: string;

  /** Sign requests */
  signRequests?: boolean;

  /** Want assertions signed */
  wantAssertionsSigned?: boolean;
}

/**
 * SAML assertion
 */
export interface SAMLAssertion {
  /** Issuer */
  issuer: string;

  /** Subject */
  subject: string;

  /** Attributes */
  attributes: Record<string, string | string[]>;

  /** Session index */
  sessionIndex?: string;

  /** Not before */
  notBefore?: Date;

  /** Not on or after */
  notOnOrAfter?: Date;
}

// ============================================================================
// WebAuthn / FIDO2
// ============================================================================

/**
 * WebAuthn credential
 */
export interface WebAuthnCredential {
  /** Credential ID */
  id: string;

  /** Public key */
  publicKey: ArrayBuffer;

  /** Counter */
  counter: number;

  /** Authenticator */
  authenticator: Authenticator;

  /** Created timestamp */
  createdAt: Date;

  /** Last used timestamp */
  lastUsedAt?: Date;
}

/**
 * Authenticator
 */
export interface Authenticator {
  /** AAGUID */
  aaguid: string;

  /** Credential ID */
  credentialId: ArrayBuffer;

  /** Credential public key */
  credentialPublicKey: ArrayBuffer;

  /** Counter */
  counter: number;

  /** Transports */
  transports?: ('usb' | 'nfc' | 'ble' | 'internal')[];
}

// ============================================================================
// Sessions & Tokens
// ============================================================================

/**
 * Authentication session
 */
export interface AuthSession {
  /** Session ID */
  id: string;

  /** User identity */
  identity: string;

  /** Trust level */
  trustLevel: TrustLevel;

  /** Created timestamp */
  createdAt: Date;

  /** Expires timestamp */
  expiresAt: Date;

  /** Last activity timestamp */
  lastActivityAt: Date;

  /** IP address */
  ipAddress?: string;

  /** User agent */
  userAgent?: string;

  /** MFA verified */
  mfaVerified: boolean;

  /** Session data */
  data?: Record<string, unknown>;
}

/**
 * Access token
 */
export interface AccessToken {
  /** Token value */
  token: string;

  /** Token type */
  type: 'Bearer' | 'JWT';

  /** Scope */
  scope: string[];

  /** Expires at */
  expiresAt: Date;

  /** Subject */
  subject: string;

  /** Audience */
  audience?: string[];

  /** Issuer */
  issuer?: string;
}

// ============================================================================
// Consent & Privacy
// ============================================================================

/**
 * Consent record
 */
export interface ConsentRecord {
  /** Consent ID */
  id: string;

  /** User identity */
  identity: string;

  /** Service */
  service: string;

  /** Purpose */
  purpose: string;

  /** Data types */
  dataTypes: string[];

  /** Granted */
  granted: boolean;

  /** Granted at */
  grantedAt?: Date;

  /** Revoked at */
  revokedAt?: Date;

  /** Expires at */
  expiresAt?: Date;
}

/**
 * Privacy settings
 */
export interface PrivacySettings {
  /** Allow analytics */
  allowAnalytics: boolean;

  /** Allow marketing */
  allowMarketing: boolean;

  /** Allow third party sharing */
  allowThirdPartySharing: boolean;

  /** Data retention period (days) */
  dataRetentionDays: number;

  /** Visible attributes */
  visibleAttributes: string[];
}

// ============================================================================
// Selective Disclosure
// ============================================================================

/**
 * Selective disclosure request
 */
export interface SelectiveDisclosureRequest {
  /** Attribute name */
  attribute: string;

  /** Predicate */
  predicate?: 'equals' | 'greaterThan' | 'lessThan' | 'contains';

  /** Value */
  value?: unknown;

  /** Reveal value */
  revealValue: boolean;
}

/**
 * Selective disclosure proof
 */
export interface SelectiveDisclosureProof {
  /** Attribute name */
  attribute: string;

  /** Predicate satisfied */
  predicateSatisfied: boolean;

  /** Revealed value */
  revealedValue?: unknown;

  /** Proof */
  proof: string;

  /** Issuer DID */
  issuer: string;

  /** Created timestamp */
  createdAt: Date;
}

// ============================================================================
// Federation & Linking
// ============================================================================

/**
 * Federation request
 */
export interface FederationRequest {
  /** Identity */
  identity: string;

  /** Provider */
  provider: string;

  /** Protocol */
  protocol: FederationProtocol;

  /** Scope */
  scope?: string[];

  /** Redirect URI */
  redirectUri?: string;

  /** State */
  state?: string;
}

/**
 * Federation response
 */
export interface FederationResponse {
  /** Success */
  success: boolean;

  /** Authorization URL */
  authorizationUrl?: string;

  /** Linked provider */
  linkedProvider?: LinkedProvider;

  /** Tokens */
  tokens?: OAuthTokenResponse;

  /** Error */
  error?: string;
}

// ============================================================================
// Validation & Verification
// ============================================================================

/**
 * Credential verification result
 */
export interface CredentialVerificationResult {
  /** Valid */
  valid: boolean;

  /** Issuer */
  issuer: string;

  /** Subject */
  subject?: string;

  /** Claims */
  claims: Record<string, unknown>;

  /** Issued at */
  issuedAt: Date;

  /** Expires at */
  expiresAt?: Date;

  /** Revoked */
  revoked: boolean;

  /** Errors */
  errors: string[];

  /** Warnings */
  warnings: string[];
}

/**
 * Identity validation result
 */
export interface IdentityValidationResult {
  /** Valid */
  valid: boolean;

  /** Identity */
  identity: UniversalIdentity;

  /** Trust level */
  trustLevel: TrustLevel;

  /** Verified attributes */
  verifiedAttributes: string[];

  /** Missing attributes */
  missingAttributes: string[];

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-CORE-001 error codes
 */
export enum IdentityErrorCode {
  INVALID_IDENTITY = 'CORE001',
  IDENTITY_NOT_FOUND = 'CORE002',
  PROVIDER_NOT_LINKED = 'CORE003',
  CREDENTIAL_INVALID = 'CORE004',
  CREDENTIAL_EXPIRED = 'CORE005',
  CREDENTIAL_REVOKED = 'CORE006',
  TRUST_LEVEL_INSUFFICIENT = 'CORE007',
  CONSENT_REQUIRED = 'CORE008',
  FEDERATION_FAILED = 'CORE009',
  DID_RESOLUTION_FAILED = 'CORE010',
}

/**
 * Identity error
 */
export class IdentityError extends Error {
  constructor(
    public code: IdentityErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'IdentityError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  UniversalIdentity,
  IdentityAttributes,
  LinkedProvider,

  // DIDs
  DID,
  DIDDocument,
  VerificationMethod,
  ServiceEndpoint,

  // Verifiable Credentials
  VerifiableCredential,
  CredentialSubject,
  CredentialProof,
  CredentialStatus,
  VerifiablePresentation,
  Evidence,
  TermsOfUse,
  Prohibition,
  RefreshService,

  // OAuth
  OAuthProvider,
  OAuthAuthorizationRequest,
  OAuthTokenResponse,

  // OIDC
  OIDCProvider,
  IDTokenClaims,
  Address,

  // SAML
  SAMLProvider,
  SAMLAssertion,

  // WebAuthn
  WebAuthnCredential,
  Authenticator,

  // Sessions
  AuthSession,
  AccessToken,

  // Privacy
  ConsentRecord,
  PrivacySettings,
  SelectiveDisclosureRequest,
  SelectiveDisclosureProof,

  // Federation
  FederationRequest,
  FederationResponse,

  // Validation
  CredentialVerificationResult,
  IdentityValidationResult,
};

export {
  IdentityErrorCode,
  IdentityError,
};
