/**
 * WIA-AI-017: AI Content Authentication TypeScript SDK
 * Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @philosophy 弘益人間 (홍익인간) · Benefit All Humanity
 */

/**
 * Supported signature algorithms
 */
export type SignatureAlgorithm = 'Ed25519' | 'ECDSA-P256' | 'ECDSA-P384' | 'RSA-PSS-3072';

/**
 * Supported hash algorithms
 */
export type HashAlgorithm = 'SHA-256' | 'SHA-3-256' | 'SHA-512';

/**
 * Fingerprint algorithms
 */
export type FingerprintAlgorithm = 'pHash' | 'dHash' | 'wHash' | 'neuralHash';

/**
 * Media types
 */
export type MediaType = 'image' | 'video' | 'audio' | 'text';

/**
 * Content fingerprint
 */
export interface ContentFingerprint {
    algorithm: FingerprintAlgorithm;
    hash: string;
    size: number;
    timestamp: string;
}

/**
 * Digital signature
 */
export interface DigitalSignature {
    algorithm: SignatureAlgorithm;
    signature: string;
    publicKey: string;
    timestamp: string;
    certificate?: string;
}

/**
 * Watermark configuration
 */
export interface WatermarkConfig {
    type: 'text' | 'logo' | 'metadata' | 'fingerprint';
    data: string;
    strength: number; // 1-10
    method: 'dct' | 'dwt' | 'lsb' | 'spread-spectrum';
}

/**
 * Watermark detection result
 */
export interface WatermarkResult {
    detected: boolean;
    data?: string;
    confidence: number;
    imperceptibility: number;
    robustness: number;
}

/**
 * C2PA Action
 */
export interface C2PAAction {
    action: string;
    when: string;
    softwareAgent?: string;
    parameters?: Record<string, any>;
}

/**
 * C2PA Assertion
 */
export interface C2PAAssertion {
    label: string;
    data: Record<string, any>;
}

/**
 * C2PA Manifest
 */
export interface C2PAManifest {
    claim_generator: string;
    title?: string;
    format?: string;
    instance_id: string;
    assertions: C2PAAssertion[];
    signature_info?: {
        alg: string;
        issuer: string;
    };
}

/**
 * Provenance record
 */
export interface ProvenanceRecord {
    contentId: string;
    creator: string;
    timestamp: string;
    aiModel?: string;
    actions: C2PAAction[];
    manifest: C2PAManifest;
}

/**
 * Deepfake detection result
 */
export interface DeepfakeDetectionResult {
    isSynthetic: boolean;
    confidence: number;
    analysis: {
        visualArtifacts?: number;
        temporalConsistency?: number;
        audioMismatch?: number;
        biologicalSignals?: number;
    };
    suspiciousRegions?: Array<{
        x: number;
        y: number;
        width: number;
        height: number;
        score: number;
    }>;
    detectorUsed: string;
}

/**
 * Authentication options
 */
export interface AuthenticationOptions {
    signatureAlgorithm?: SignatureAlgorithm;
    hashAlgorithm?: HashAlgorithm;
    embedWatermark?: boolean;
    watermarkConfig?: WatermarkConfig;
    generateFingerprint?: boolean;
    fingerprintAlgorithm?: FingerprintAlgorithm;
    includeTimestamp?: boolean;
}

/**
 * Authentication result
 */
export interface AuthenticationResult {
    authenticatedContent: Buffer;
    signature: DigitalSignature;
    fingerprints?: ContentFingerprint[];
    watermark?: WatermarkResult;
    manifest: C2PAManifest;
    success: boolean;
}

/**
 * Verification options
 */
export interface VerificationOptions {
    checkSignature?: boolean;
    checkWatermark?: boolean;
    checkFingerprint?: boolean;
    runDeepfakeDetection?: boolean;
}

/**
 * Verification result
 */
export interface VerificationResult {
    authentic: boolean;
    signatureValid: boolean;
    watermarkDetected: boolean;
    fingerprintMatch?: boolean;
    deepfakeDetection?: DeepfakeDetectionResult;
    manifest?: C2PAManifest;
    confidence: number;
    timestamp: string;
}

/**
 * Error types
 */
export class WIAAuthError extends Error {
    constructor(
        message: string,
        public code: string,
        public details?: any
    ) {
        super(message);
        this.name = 'WIAAuthError';
    }
}

/**
 * Configuration for WIA authentication service
 */
export interface WIAConfig {
    apiEndpoint?: string;
    privateKey?: string;
    publicKey?: string;
    certificatePath?: string;
    hsmEnabled?: boolean;
    hsmConfig?: {
        provider: string;
        slot: number;
        pin: string;
    };
    cacheEnabled?: boolean;
    cacheTTL?: number;
}

/**
 * LSH Index configuration
 */
export interface LSHConfig {
    numTables: number;
    hashSize: number;
    threshold: number;
}

/**
 * Blockchain anchor proof
 */
export interface BlockchainProof {
    blockchain: string;
    network: string;
    transactionHash: string;
    blockNumber: number;
    merkleRoot: string;
    timestamp: string;
}

/**
 * Content metadata
 */
export interface ContentMetadata {
    title: string;
    creator: string;
    timestamp: string;
    format: string;
    aiModel?: string;
    prompt?: string;
    parameters?: Record<string, any>;
}

/**
 * Batch authentication request
 */
export interface BatchAuthRequest {
    items: Array<{
        content: Buffer;
        metadata: ContentMetadata;
        options?: AuthenticationOptions;
    }>;
}

/**
 * Batch authentication result
 */
export interface BatchAuthResult {
    results: AuthenticationResult[];
    totalProcessed: number;
    successful: number;
    failed: number;
    duration: number;
}
