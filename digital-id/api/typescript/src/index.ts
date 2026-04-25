/**
 * WIA-SOC-002: Digital ID Standard - TypeScript SDK
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import axios, { AxiosInstance } from 'axios';
import { ed25519 } from '@noble/ed25519';
import { secp256k1 } from '@noble/secp256k1';
import * as jsonld from 'jsonld';
import {
  DIDDocument,
  VerifiableCredential,
  VerifiablePresentation,
  WIAClientConfig,
  DIDMethodOptions,
  DIDCreationResult,
  CredentialRequest,
  VerificationResult,
  ZKProof,
  ProofStatement,
  PresentationRequest,
  AuthenticationProof,
  WIAError
} from './types';

export * from './types';

/**
 * Main WIA Digital ID Client
 */
export class WiaDigitalID {
  private client: AxiosInstance;
  private config: WIAClientConfig;

  constructor(config: WIAClientConfig) {
    this.config = config;
    this.client = axios.create({
      baseURL: config.apiUrl,
      timeout: config.timeout || 30000,
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey && { 'Authorization': `Bearer ${config.apiKey}` })
      }
    });

    // Add error interceptor
    this.client.interceptors.response.use(
      response => response,
      error => {
        if (error.response) {
          const wiaError: WIAError = error.response.data.error;
          throw new Error(`[${wiaError.code}] ${wiaError.message}`);
        }
        throw error;
      }
    );
  }

  /**
   * Create a new DID
   */
  async createDID(options: DIDMethodOptions): Promise<DIDCreationResult> {
    const response = await this.client.post('/did/create', {
      method: options.method,
      network: options.network || this.config.network || 'mainnet',
      options: {
        keyType: options.keyType || 'Ed25519VerificationKey2020',
        controller: options.controller,
        service: options.service
      }
    });

    return response.data;
  }

  /**
   * Resolve a DID to its DID Document
   */
  async resolveDID(did: string): Promise<DIDDocument> {
    const response = await this.client.get(`/did/${encodeURIComponent(did)}`);
    return response.data;
  }

  /**
   * Update an existing DID Document
   */
  async updateDID(
    did: string,
    didDocument: DIDDocument,
    proof: any
  ): Promise<DIDDocument> {
    const response = await this.client.put(`/did/${encodeURIComponent(did)}`, {
      didDocument,
      proof
    });
    return response.data.didDocument;
  }

  /**
   * Deactivate a DID
   */
  async deactivateDID(did: string): Promise<void> {
    await this.client.delete(`/did/${encodeURIComponent(did)}`);
  }

  /**
   * Issue a new Verifiable Credential
   */
  async issueCredential(
    request: CredentialRequest
  ): Promise<VerifiableCredential> {
    const response = await this.client.post('/credentials/issue', request);
    return response.data;
  }

  /**
   * Verify a Verifiable Credential
   */
  async verifyCredential(
    credential: VerifiableCredential,
    options?: {
      checkRevocation?: boolean;
      checkExpiration?: boolean;
      verifySignature?: boolean;
    }
  ): Promise<VerificationResult> {
    const response = await this.client.post('/credentials/verify', {
      verifiableCredential: credential,
      options: options || {
        checkRevocation: true,
        checkExpiration: true,
        verifySignature: true
      }
    });
    return response.data;
  }

  /**
   * Get a credential by ID
   */
  async getCredential(id: string): Promise<VerifiableCredential> {
    const response = await this.client.get(
      `/credentials/${encodeURIComponent(id)}`
    );
    return response.data;
  }

  /**
   * Revoke a credential
   */
  async revokeCredential(
    credentialId: string,
    reason: string,
    proof: any
  ): Promise<void> {
    await this.client.post('/credentials/revoke', {
      credentialId,
      reason,
      proof
    });
  }

  /**
   * Create a Verifiable Presentation
   */
  async createPresentation(
    request: PresentationRequest
  ): Promise<VerifiablePresentation> {
    const response = await this.client.post('/presentations/create', request);
    return response.data;
  }

  /**
   * Verify a Verifiable Presentation
   */
  async verifyPresentation(
    presentation: VerifiablePresentation,
    challenge?: string,
    domain?: string
  ): Promise<VerificationResult> {
    const response = await this.client.post('/presentations/verify', {
      verifiablePresentation: presentation,
      challenge,
      domain
    });
    return response.data;
  }

  /**
   * Create a Zero-Knowledge Proof
   */
  async createZKProof(options: {
    credentialId: string;
    statement: ProofStatement;
    reveal?: string[];
    proofSystem?: 'groth16' | 'plonk' | 'bulletproofs';
  }): Promise<ZKProof> {
    const response = await this.client.post('/proofs/create', options);
    return response.data.zkProof;
  }

  /**
   * Verify a Zero-Knowledge Proof
   */
  async verifyZKProof(
    zkProof: ZKProof,
    verificationKey?: any
  ): Promise<boolean> {
    const response = await this.client.post('/proofs/verify', {
      zkProof,
      verificationKey
    });
    return response.data.verified;
  }
}

/**
 * Utility functions for cryptographic operations
 */
export class CryptoUtils {
  /**
   * Generate Ed25519 key pair
   */
  static async generateEd25519KeyPair(): Promise<{
    privateKey: Uint8Array;
    publicKey: Uint8Array;
  }> {
    const privateKey = ed25519.utils.randomPrivateKey();
    const publicKey = await ed25519.getPublicKey(privateKey);
    return { privateKey, publicKey };
  }

  /**
   * Generate secp256k1 key pair (Ethereum compatible)
   */
  static generateSecp256k1KeyPair(): {
    privateKey: Uint8Array;
    publicKey: Uint8Array;
  } {
    const privateKey = secp256k1.utils.randomPrivateKey();
    const publicKey = secp256k1.getPublicKey(privateKey);
    return { privateKey, publicKey };
  }

  /**
   * Sign message with Ed25519
   */
  static async signEd25519(
    message: Uint8Array,
    privateKey: Uint8Array
  ): Promise<Uint8Array> {
    return ed25519.sign(message, privateKey);
  }

  /**
   * Verify Ed25519 signature
   */
  static async verifyEd25519(
    signature: Uint8Array,
    message: Uint8Array,
    publicKey: Uint8Array
  ): Promise<boolean> {
    return ed25519.verify(signature, message, publicKey);
  }

  /**
   * Sign message with secp256k1
   */
  static signSecp256k1(
    messageHash: Uint8Array,
    privateKey: Uint8Array
  ): Uint8Array {
    return secp256k1.sign(messageHash, privateKey).toCompactRawBytes();
  }

  /**
   * Verify secp256k1 signature
   */
  static verifySecp256k1(
    signature: Uint8Array,
    messageHash: Uint8Array,
    publicKey: Uint8Array
  ): boolean {
    return secp256k1.verify(signature, messageHash, publicKey);
  }

  /**
   * Generate random nonce
   */
  static generateNonce(length: number = 32): string {
    const bytes = new Uint8Array(length);
    crypto.getRandomValues(bytes);
    return Buffer.from(bytes).toString('hex');
  }

  /**
   * Hash data with SHA-256
   */
  static async sha256(data: Uint8Array): Promise<Uint8Array> {
    const hashBuffer = await crypto.subtle.digest('SHA-256', data);
    return new Uint8Array(hashBuffer);
  }
}

/**
 * DID Document Builder
 */
export class DIDDocumentBuilder {
  private document: Partial<DIDDocument>;

  constructor(did: string) {
    this.document = {
      '@context': ['https://www.w3.org/ns/did/v1'],
      id: did
    };
  }

  addVerificationMethod(
    id: string,
    type: string,
    controller: string,
    publicKey: string
  ): this {
    if (!this.document.verificationMethod) {
      this.document.verificationMethod = [];
    }

    this.document.verificationMethod.push({
      id,
      type,
      controller,
      publicKeyMultibase: publicKey
    });

    return this;
  }

  addAuthentication(methodId: string): this {
    if (!this.document.authentication) {
      this.document.authentication = [];
    }
    this.document.authentication.push(methodId);
    return this;
  }

  addAssertionMethod(methodId: string): this {
    if (!this.document.assertionMethod) {
      this.document.assertionMethod = [];
    }
    this.document.assertionMethod.push(methodId);
    return this;
  }

  addService(id: string, type: string, serviceEndpoint: string): this {
    if (!this.document.service) {
      this.document.service = [];
    }

    this.document.service.push({
      id,
      type,
      serviceEndpoint
    });

    return this;
  }

  build(): DIDDocument {
    return this.document as DIDDocument;
  }
}

/**
 * Credential Builder
 */
export class CredentialBuilder {
  private credential: Partial<VerifiableCredential>;

  constructor() {
    this.credential = {
      '@context': [
        'https://www.w3.org/2018/credentials/v1',
        'https://wiastandards.com/credentials/v1'
      ],
      type: ['VerifiableCredential']
    };
  }

  setId(id: string): this {
    this.credential.id = id;
    return this;
  }

  addType(type: string): this {
    if (!this.credential.type) {
      this.credential.type = ['VerifiableCredential'];
    }
    this.credential.type.push(type);
    return this;
  }

  setIssuer(issuer: string | { id: string; name?: string }): this {
    this.credential.issuer = issuer;
    return this;
  }

  setIssuanceDate(date: string): this {
    this.credential.issuanceDate = date;
    return this;
  }

  setExpirationDate(date: string): this {
    this.credential.expirationDate = date;
    return this;
  }

  setCredentialSubject(subject: any): this {
    this.credential.credentialSubject = subject;
    return this;
  }

  addCredentialStatus(status: any): this {
    this.credential.credentialStatus = status;
    return this;
  }

  build(): Partial<VerifiableCredential> {
    return this.credential;
  }
}

/**
 * Presentation Builder
 */
export class PresentationBuilder {
  private presentation: Partial<VerifiablePresentation>;

  constructor(holder: string) {
    this.presentation = {
      '@context': ['https://www.w3.org/2018/credentials/v1'],
      type: 'VerifiablePresentation',
      holder
    };
  }

  setId(id: string): this {
    this.presentation.id = id;
    return this;
  }

  addCredential(credential: VerifiableCredential): this {
    if (!this.presentation.verifiableCredential) {
      this.presentation.verifiableCredential = [];
    }

    if (Array.isArray(this.presentation.verifiableCredential)) {
      this.presentation.verifiableCredential.push(credential);
    } else {
      this.presentation.verifiableCredential = [
        this.presentation.verifiableCredential,
        credential
      ];
    }

    return this;
  }

  build(): Partial<VerifiablePresentation> {
    return this.presentation;
  }
}

/**
 * DID Parser
 */
export class DIDParser {
  static parse(did: string): {
    method: string;
    methodSpecificId: string;
    network?: string;
  } {
    const parts = did.split(':');

    if (parts.length < 3 || parts[0] !== 'did') {
      throw new Error('Invalid DID format');
    }

    return {
      method: parts[1],
      methodSpecificId: parts.slice(2).join(':'),
      network: parts[2]
    };
  }

  static isValid(did: string): boolean {
    try {
      DIDParser.parse(did);
      return true;
    } catch {
      return false;
    }
  }
}

/**
 * Credential Status Checker
 */
export class StatusChecker {
  static async checkRevocation(
    credential: VerifiableCredential
  ): Promise<boolean> {
    if (!credential.credentialStatus) {
      return false; // Not revocable
    }

    // Implementation would check the status list
    // This is a simplified version
    return false;
  }

  static isExpired(credential: VerifiableCredential): boolean {
    if (!credential.expirationDate) {
      return false;
    }

    const now = new Date();
    const expiration = new Date(credential.expirationDate);

    return now > expiration;
  }
}

/**
 * Export main class and utilities
 */
export default WiaDigitalID;
