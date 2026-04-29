/**
 * WIA DPKI Standard - TypeScript SDK
 * Decentralized Public Key Infrastructure
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

import axios, { AxiosInstance } from 'axios';

export * from './types';
import type {
  APIConfig, WIADPKI, InfrastructureResponse, DecentralizedIdentity, KeyPair,
  DistributedCertificate, TrustAnchor, RevocationEntry, ValidationResult, PaginatedResponse,
} from './types';

export class WIADPKIClient {
  private axios: AxiosInstance;

  constructor(config: APIConfig) {
    this.axios = axios.create({
      baseURL: config.baseURL,
      timeout: config.timeout || 30000,
      headers: { 'Content-Type': 'application/json', ...(config.apiKey && { Authorization: `Bearer ${config.apiKey}` }) },
    });
  }

  async createInfrastructure(dpki: WIADPKI): Promise<InfrastructureResponse> {
    const response = await this.axios.post<InfrastructureResponse>('/infrastructures', dpki);
    return response.data;
  }

  async getInfrastructure(id: string): Promise<WIADPKI> {
    const response = await this.axios.get<WIADPKI>(`/infrastructures/${id}`);
    return response.data;
  }

  async listInfrastructures(params?: { type?: string; status?: string; limit?: number }): Promise<PaginatedResponse<InfrastructureResponse>> {
    const response = await this.axios.get<PaginatedResponse<InfrastructureResponse>>('/infrastructures', { params });
    return response.data;
  }

  async updateInfrastructure(id: string, updates: Partial<WIADPKI>): Promise<InfrastructureResponse> {
    const response = await this.axios.put<InfrastructureResponse>(`/infrastructures/${id}`, updates);
    return response.data;
  }

  async registerIdentity(infraId: string, identity: DecentralizedIdentity): Promise<DecentralizedIdentity> {
    const response = await this.axios.post<DecentralizedIdentity>(`/infrastructures/${infraId}/identities`, identity);
    return response.data;
  }

  async resolveIdentity(infraId: string, did: string): Promise<DecentralizedIdentity> {
    const response = await this.axios.get<DecentralizedIdentity>(`/infrastructures/${infraId}/identities/${encodeURIComponent(did)}`);
    return response.data;
  }

  async updateIdentity(infraId: string, did: string, updates: Partial<DecentralizedIdentity>): Promise<DecentralizedIdentity> {
    const response = await this.axios.put<DecentralizedIdentity>(`/infrastructures/${infraId}/identities/${encodeURIComponent(did)}`, updates);
    return response.data;
  }

  async deactivateIdentity(infraId: string, did: string): Promise<void> {
    await this.axios.delete(`/infrastructures/${infraId}/identities/${encodeURIComponent(did)}`);
  }

  async registerKey(infraId: string, key: Omit<KeyPair, 'id' | 'fingerprint' | 'status' | 'usage'>): Promise<KeyPair> {
    const response = await this.axios.post<KeyPair>(`/infrastructures/${infraId}/keys`, key);
    return response.data;
  }

  async getKey(infraId: string, keyId: string): Promise<KeyPair> {
    const response = await this.axios.get<KeyPair>(`/infrastructures/${infraId}/keys/${keyId}`);
    return response.data;
  }

  async listKeys(infraId: string, owner?: string): Promise<KeyPair[]> {
    const response = await this.axios.get<KeyPair[]>(`/infrastructures/${infraId}/keys`, { params: { owner } });
    return response.data;
  }

  async rotateKey(infraId: string, keyId: string, newPublicKey: string): Promise<KeyPair> {
    const response = await this.axios.post<KeyPair>(`/infrastructures/${infraId}/keys/${keyId}/rotate`, { newPublicKey });
    return response.data;
  }

  async issueCertificate(infraId: string, cert: Omit<DistributedCertificate, 'id' | 'status'>): Promise<DistributedCertificate> {
    const response = await this.axios.post<DistributedCertificate>(`/infrastructures/${infraId}/certificates`, cert);
    return response.data;
  }

  async getCertificate(infraId: string, certId: string): Promise<DistributedCertificate> {
    const response = await this.axios.get<DistributedCertificate>(`/infrastructures/${infraId}/certificates/${certId}`);
    return response.data;
  }

  async verifyCertificate(infraId: string, certId: string): Promise<{ valid: boolean; chain: string[]; issues?: string[] }> {
    const response = await this.axios.get(`/infrastructures/${infraId}/certificates/${certId}/verify`);
    return response.data;
  }

  async addTrustAnchor(infraId: string, anchor: Omit<TrustAnchor, 'id' | 'created'>): Promise<TrustAnchor> {
    const response = await this.axios.post<TrustAnchor>(`/infrastructures/${infraId}/trust-anchors`, anchor);
    return response.data;
  }

  async listTrustAnchors(infraId: string): Promise<TrustAnchor[]> {
    const response = await this.axios.get<TrustAnchor[]>(`/infrastructures/${infraId}/trust-anchors`);
    return response.data;
  }

  async revokeKey(infraId: string, keyId: string, reason: string): Promise<RevocationEntry> {
    const response = await this.axios.post<RevocationEntry>(`/infrastructures/${infraId}/keys/${keyId}/revoke`, { reason });
    return response.data;
  }

  async revokeCertificate(infraId: string, certId: string, reason: string): Promise<RevocationEntry> {
    const response = await this.axios.post<RevocationEntry>(`/infrastructures/${infraId}/certificates/${certId}/revoke`, { reason });
    return response.data;
  }

  async checkRevocationStatus(infraId: string, targetId: string): Promise<{ revoked: boolean; entry?: RevocationEntry }> {
    const response = await this.axios.get(`/infrastructures/${infraId}/revocations/${targetId}`);
    return response.data;
  }

  async getRevocationList(infraId: string): Promise<RevocationEntry[]> {
    const response = await this.axios.get<RevocationEntry[]>(`/infrastructures/${infraId}/revocations`);
    return response.data;
  }

  async sign(infraId: string, keyId: string, data: string): Promise<{ signature: string; algorithm: string }> {
    const response = await this.axios.post(`/infrastructures/${infraId}/keys/${keyId}/sign`, { data });
    return response.data;
  }

  async verify(infraId: string, keyId: string, data: string, signature: string): Promise<{ valid: boolean }> {
    const response = await this.axios.post(`/infrastructures/${infraId}/keys/${keyId}/verify`, { data, signature });
    return response.data;
  }

  async getNetworkStatus(infraId: string): Promise<{ nodes: number; consensus: string; synced: boolean }> {
    const response = await this.axios.get(`/infrastructures/${infraId}/network/status`);
    return response.data;
  }

  validateDPKI(dpki: WIADPKI): ValidationResult {
    const errors: { path: string; message: string }[] = [];
    if (dpki.standard !== 'WIA-DPKI') errors.push({ path: 'standard', message: 'Invalid standard' });
    if (!dpki.infrastructure?.id) errors.push({ path: 'infrastructure.id', message: 'Infrastructure ID required' });
    if (!dpki.infrastructure?.network?.nodes?.length) errors.push({ path: 'infrastructure.network.nodes', message: 'At least one node required' });
    return { valid: errors.length === 0, errors: errors.length > 0 ? errors : undefined };
  }
}

export function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    return (c === 'x' ? r : (r & 0x3) | 0x8).toString(16);
  });
}

export function generateDID(method: string = 'dpki'): string {
  return `did:${method}:${generateUUID().replace(/-/g, '')}`;
}

export function createMinimalDPKI(name: string): WIADPKI {
  const nodeId = generateUUID();
  return {
    standard: 'WIA-DPKI',
    version: '1.0.0',
    infrastructure: {
      id: generateUUID(), name, type: 'distributed-ledger', status: 'initializing', createdAt: new Date().toISOString(),
      network: { protocol: 'libp2p', nodes: [{ id: nodeId, address: 'localhost:9000', role: 'validator', status: 'active', lastSeen: new Date().toISOString() }], replicationFactor: 3, partitionTolerance: true },
      consensus: { algorithm: 'pbft', threshold: 0.67, timeout: 5000, finality: 'instant' },
    },
    identities: [],
    keys: [],
    certificates: [],
    trustAnchors: [],
    revocations: [],
    governance: {
      model: 'decentralized', policies: [], participants: [],
      proposals: [], voting: { quorum: 0.5, threshold: 0.67, duration: 86400, delegationAllowed: true },
    },
  };
}

export default { WIADPKIClient, generateUUID, generateDID, createMinimalDPKI };
