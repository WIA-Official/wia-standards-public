/**
 * WIA-UNI-001 TypeScript SDK
 * Inter-Korean Data Exchange Standard
 *
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

import { EventEmitter } from 'events';
import * as crypto from 'crypto';
import { v4 as uuidv4 } from 'uuid';
import axios, { AxiosInstance } from 'axios';

import * as Types from './types';

export * from './types';

// ============================================================================
// Main Exchange Class
// ============================================================================

export class InterKoreanExchange extends EventEmitter {
  private config: Types.ExchangeConfig;
  private credentials?: Types.AuthCredentials;
  private client: AxiosInstance;
  private websocket?: WebSocket;

  constructor(config: Types.ExchangeConfig) {
    super();
    this.config = {
      timeout: 30000,
      retries: 3,
      ...config
    };

    this.client = axios.create({
      baseURL: config.endpoints?.exchange || 'https://api.wia-uni-001.org',
      timeout: this.config.timeout,
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Version': Types.VERSION
      }
    });
  }

  // ==========================================================================
  // Authentication
  // ==========================================================================

  async authenticate(credentials: Types.AuthCredentials): Promise<boolean> {
    this.credentials = credentials;

    try {
      const response = await this.client.post('/auth/login', {
        region: credentials.region,
        userId: credentials.userId,
        publicKey: this.extractPublicKey(credentials.privateKey),
        signature: this.signAuthRequest(credentials)
      });

      if (response.data.success) {
        this.client.defaults.headers.common['Authorization'] =
          `Bearer ${response.data.token}`;
        this.connectWebSocket();
        return true;
      }

      return false;
    } catch (error) {
      this.handleError(error);
      return false;
    }
  }

  // ==========================================================================
  // Message Operations
  // ==========================================================================

  async sendMessage(params: {
    to: Types.UserIdentity;
    content: Types.MessageContent;
    type?: Types.MessageType;
    humanitarian?: boolean;
    priority?: Types.Priority;
  }): Promise<Types.SendMessageResponse> {
    if (!this.credentials) {
      throw new Error('Not authenticated');
    }

    const message: Types.Message = {
      id: uuidv4(),
      version: Types.VERSION,
      timestamp: new Date(),
      from: {
        userId: this.credentials.userId,
        region: this.credentials.region,
        publicKey: this.extractPublicKey(this.credentials.privateKey),
        verified: true
      },
      to: params.to,
      type: params.type || 'text',
      humanitarian: params.humanitarian || false,
      priority: params.priority || 'normal',
      payload: await this.encryptContent(params.content, params.to.publicKey),
      signatures: {
        sender: await this.signMessage(params.content),
        trustAnchors: []
      }
    };

    try {
      const response = await this.client.post('/messages/send', message);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  async receiveMessages(): Promise<Types.Message[]> {
    if (!this.credentials) {
      throw new Error('Not authenticated');
    }

    try {
      const response = await this.client.get('/messages/inbox');
      return response.data.messages;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  async decrypt(payload: Types.EncryptedPayload): Promise<Types.MessageContent> {
    if (!this.credentials) {
      throw new Error('Not authenticated');
    }

    const decrypted = await this.decryptPayload(
      payload,
      this.credentials.privateKey
    );
    return JSON.parse(decrypted);
  }

  // ==========================================================================
  // Family Reunification
  // ==========================================================================

  async searchFamily(
    criteria: Partial<Types.FamilyMember['personalInfo']> & {
      relationship?: Types.FamilyRelationship;
    }
  ): Promise<Types.SearchFamilyResponse> {
    try {
      const response = await this.client.post('/family/search', criteria);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  async registerFamily(member: Types.FamilyMember): Promise<{ success: boolean; memberId: string }> {
    try {
      const response = await this.client.post('/family/register', member);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  async sendReunificationRequest(params: {
    matchId: string;
    relationship: Types.FamilyRelationship;
    message: string;
  }): Promise<Types.SendMessageResponse> {
    try {
      const response = await this.client.post('/family/reunion-request', params);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ==========================================================================
  // Humanitarian & Emergency
  // ==========================================================================

  async reportEmergency(alert: Omit<Types.EmergencyAlert, 'id' | 'reportedAt' | 'status' | 'responses'>): Promise<{
    success: boolean;
    alertId: string;
  }> {
    try {
      const response = await this.client.post('/emergency/report', alert);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  async grantHumanitarianStatus(
    userId: string,
    category: Types.HumanitarianCategory
  ): Promise<{ success: boolean; verificationId: string }> {
    try {
      const response = await this.client.post('/humanitarian/grant', {
        userId,
        category
      });
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ==========================================================================
  // Verification & Audit
  // ==========================================================================

  async verifyTransaction(transactionId: string): Promise<Types.VerificationResponse> {
    try {
      const response = await this.client.get(`/verify/${transactionId}`);
      return response.data;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  async getVerificationProof(transactionId: string): Promise<Types.VerificationProof> {
    try {
      const response = await this.client.get(`/blockchain/proof/${transactionId}`);
      return response.data.proof;
    } catch (error) {
      throw this.handleError(error);
    }
  }

  // ==========================================================================
  // Cryptography
  // ==========================================================================

  async generateKeyPair(options?: {
    algorithm?: 'ECDH-P256' | 'CRYSTALS-Kyber-768';
  }): Promise<Types.KeyPair> {
    const algorithm = options?.algorithm || 'ECDH-P256';

    if (algorithm === 'ECDH-P256') {
      const { publicKey, privateKey } = crypto.generateKeyPairSync('ec', {
        namedCurve: 'prime256v1',
        publicKeyEncoding: { type: 'spki', format: 'pem' },
        privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
      });

      return {
        publicKey: Buffer.from(publicKey).toString('base64'),
        privateKey: Buffer.from(privateKey).toString('base64'),
        algorithm
      };
    }

    throw new Error('Post-quantum cryptography not yet implemented');
  }

  private async encryptContent(
    content: Types.MessageContent,
    recipientPublicKey: string
  ): Promise<Types.EncryptedPayload> {
    const plaintext = JSON.stringify(content);

    // Generate ephemeral key pair for perfect forward secrecy
    const ephemeralKeyPair = await this.generateKeyPair();

    // Derive shared secret using ECDH
    const sharedSecret = this.deriveSharedSecret(
      ephemeralKeyPair.privateKey,
      recipientPublicKey
    );

    // Encrypt with AES-256-GCM
    const iv = crypto.randomBytes(12);
    const cipher = crypto.createCipheriv('aes-256-gcm', sharedSecret, iv);

    let encrypted = cipher.update(plaintext, 'utf8', 'base64');
    encrypted += cipher.final('base64');

    const authTag = cipher.getAuthTag();

    return {
      encrypted: Buffer.concat([iv, Buffer.from(encrypted, 'base64')]).toString('base64'),
      ephemeralKey: ephemeralKeyPair.publicKey,
      authTag: authTag.toString('base64'),
      algorithm: 'AES-256-GCM'
    };
  }

  private async decryptPayload(
    payload: Types.EncryptedPayload,
    privateKey: string
  ): Promise<string> {
    const encryptedBuffer = Buffer.from(payload.encrypted, 'base64');
    const iv = encryptedBuffer.slice(0, 12);
    const ciphertext = encryptedBuffer.slice(12);

    // Derive shared secret
    const sharedSecret = this.deriveSharedSecret(
      privateKey,
      payload.ephemeralKey
    );

    // Decrypt
    const decipher = crypto.createDecipheriv('aes-256-gcm', sharedSecret, iv);
    decipher.setAuthTag(Buffer.from(payload.authTag, 'base64'));

    let decrypted = decipher.update(ciphertext, undefined, 'utf8');
    decrypted += decipher.final('utf8');

    return decrypted;
  }

  private deriveSharedSecret(privateKey: string, publicKey: string): Buffer {
    const privKey = crypto.createPrivateKey({
      key: Buffer.from(privateKey, 'base64'),
      format: 'pem',
      type: 'pkcs8'
    });

    const pubKey = crypto.createPublicKey({
      key: Buffer.from(publicKey, 'base64'),
      format: 'pem',
      type: 'spki'
    });

    const sharedSecret = crypto.diffieHellman({
      privateKey: privKey,
      publicKey: pubKey
    });

    // Derive 256-bit key using SHA-256
    return crypto.createHash('sha256').update(sharedSecret).digest();
  }

  private async signMessage(content: Types.MessageContent): Promise<string> {
    if (!this.credentials) {
      throw new Error('Not authenticated');
    }

    const message = JSON.stringify(content);
    const sign = crypto.createSign('SHA256');
    sign.update(message);
    sign.end();

    const signature = sign.sign(
      Buffer.from(this.credentials.privateKey, 'base64'),
      'base64'
    );

    return signature;
  }

  private extractPublicKey(privateKey: string): string {
    const privKey = crypto.createPrivateKey({
      key: Buffer.from(privateKey, 'base64'),
      format: 'pem',
      type: 'pkcs8'
    });

    const publicKey = crypto.createPublicKey(privKey);

    return publicKey.export({ type: 'spki', format: 'pem' }).toString('base64');
  }

  private signAuthRequest(credentials: Types.AuthCredentials): string {
    const message = `${credentials.region}:${credentials.userId}:${Date.now()}`;
    const sign = crypto.createSign('SHA256');
    sign.update(message);
    sign.end();

    return sign.sign(
      Buffer.from(credentials.privateKey, 'base64'),
      'base64'
    );
  }

  // ==========================================================================
  // WebSocket Connection
  // ==========================================================================

  private connectWebSocket() {
    if (typeof WebSocket === 'undefined') {
      return; // Not in browser environment
    }

    const wsUrl = this.config.endpoints?.exchange?.replace('http', 'ws') || 'wss://api.wia-uni-001.org';

    this.websocket = new WebSocket(`${wsUrl}/ws`);

    this.websocket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      this.emit('message', data);
    };

    this.websocket.onerror = (error) => {
      this.emit('error', error);
    };

    this.websocket.onclose = () => {
      setTimeout(() => this.connectWebSocket(), 5000); // Reconnect after 5s
    };
  }

  // ==========================================================================
  // Utility
  // ==========================================================================

  private handleError(error: any): Error {
    const errorResponse: Types.ErrorResponse = {
      code: error.response?.status || 500,
      message: error.response?.data?.message || error.message,
      details: error.response?.data?.details,
      retryable: error.response?.status >= 500
    };

    this.emit('error', { type: 'error', error: errorResponse });

    return new Error(errorResponse.message);
  }

  disconnect() {
    if (this.websocket) {
      this.websocket.close();
    }
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

export function generateId(): string {
  return uuidv4();
}

export function validateRegion(region: string): region is Types.Region {
  return region === 'north' || region === 'south';
}

export function isHumanitarianMessage(message: Types.Message): message is Types.HumanitarianMessage {
  return message.humanitarian === true;
}

export function calculateMatchConfidence(
  member1: Types.FamilyMember,
  member2: Types.FamilyMember
): number {
  let confidence = 0;

  // Name matching
  if (member1.personalInfo.familyName === member2.personalInfo.familyName) {
    confidence += 0.3;
  }

  // Age correlation
  const ageDiff = Math.abs(
    member1.personalInfo.birthYear - member2.personalInfo.birthYear
  );
  if (ageDiff <= 2) {
    confidence += 0.2;
  }

  // Location matching
  if (
    member1.personalInfo.lastKnownLocation &&
    member2.personalInfo.lastKnownLocation &&
    member1.personalInfo.lastKnownLocation === member2.personalInfo.lastKnownLocation
  ) {
    confidence += 0.3;
  }

  // Relationship compatibility
  if (member1.searchingFor.relationship && member2.searchingFor.relationship) {
    const compatible = areRelationshipsCompatible(
      member1.searchingFor.relationship,
      member2.searchingFor.relationship
    );
    if (compatible) {
      confidence += 0.2;
    }
  }

  return Math.min(confidence, 1.0);
}

function areRelationshipsCompatible(
  rel1: Types.FamilyRelationship,
  rel2: Types.FamilyRelationship
): boolean {
  const compatible: Record<Types.FamilyRelationship, Types.FamilyRelationship[]> = {
    'parent': ['child'],
    'child': ['parent'],
    'sibling': ['sibling'],
    'spouse': ['spouse'],
    'grandparent': ['grandchild'],
    'grandchild': ['grandparent'],
    'aunt-uncle': ['niece-nephew'],
    'niece-nephew': ['aunt-uncle'],
    'cousin': ['cousin'],
    'other': ['other']
  };

  return compatible[rel1]?.includes(rel2) || false;
}

// ============================================================================
// Default Export
// ============================================================================

export default InterKoreanExchange;
