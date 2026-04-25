/**
 * WIA AI Embodiment Ethics — TypeScript SDK
 * Ethical-evaluation and consent tracking for embodied AI systems
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/embodiment-ethics/v1
 */

export interface Agent {
  id?: string;
  embodimentType?: string;
  ethicsProfile?: string;
  certifiedAt?: string;
}

export interface EthicsCheck {
  id?: string;
  agentId?: string;
  policyId?: string;
  verdict?: string;
  score?: number;
}

export interface Consent {
  id?: string;
  subjectId?: string;
  agentId?: string;
  scope?: string;
  status?: string;
}

export interface Policy {
  id?: string;
  name?: string;
  version?: string;
  rules?: unknown[];
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class AiEmbodimentEthicsClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/embodiment-ethics/v1";
    this.apiKey = opts.apiKey;
    this.fetchImpl = opts.fetch ?? fetch;
  }

  private async request<T>(method: string, path: string, body?: unknown): Promise<T> {
    const headers: Record<string, string> = { "Content-Type": "application/json" };
    if (this.apiKey) headers["X-API-Key"] = this.apiKey;
    const res = await this.fetchImpl(this.baseUrl + path, {
      method,
      headers,
      body: body !== undefined ? JSON.stringify(body) : undefined,
    });
    if (!res.ok) throw new Error(`HTTP ${res.status} ${res.statusText}`);
    if (res.status === 204) return undefined as T;
    return (await res.json()) as T;
  }

  async listAgent(limit = 20, offset = 0): Promise<{ items: Agent[]; total: number }> {
  return this.request("GET", `/agents?limit=${limit}&offset=${offset}`);
}
async getAgent(id: string): Promise<Agent> {
  return this.request("GET", `/agents/${encodeURIComponent(id)}`);
}
async createAgent(body: Agent): Promise<Agent> {
  return this.request("POST", `/agents`, body);
}
async replaceAgent(id: string, body: Agent): Promise<Agent> {
  return this.request("PUT", `/agents/${encodeURIComponent(id)}`, body);
}
async deleteAgent(id: string): Promise<void> {
  return this.request("DELETE", `/agents/${encodeURIComponent(id)}`);
}

  async listEthicsCheck(limit = 20, offset = 0): Promise<{ items: EthicsCheck[]; total: number }> {
  return this.request("GET", `/ethics-checks?limit=${limit}&offset=${offset}`);
}
async getEthicsCheck(id: string): Promise<EthicsCheck> {
  return this.request("GET", `/ethics-checks/${encodeURIComponent(id)}`);
}
async createEthicsCheck(body: EthicsCheck): Promise<EthicsCheck> {
  return this.request("POST", `/ethics-checks`, body);
}
async replaceEthicsCheck(id: string, body: EthicsCheck): Promise<EthicsCheck> {
  return this.request("PUT", `/ethics-checks/${encodeURIComponent(id)}`, body);
}
async deleteEthicsCheck(id: string): Promise<void> {
  return this.request("DELETE", `/ethics-checks/${encodeURIComponent(id)}`);
}

  async listConsent(limit = 20, offset = 0): Promise<{ items: Consent[]; total: number }> {
  return this.request("GET", `/consents?limit=${limit}&offset=${offset}`);
}
async getConsent(id: string): Promise<Consent> {
  return this.request("GET", `/consents/${encodeURIComponent(id)}`);
}
async createConsent(body: Consent): Promise<Consent> {
  return this.request("POST", `/consents`, body);
}
async replaceConsent(id: string, body: Consent): Promise<Consent> {
  return this.request("PUT", `/consents/${encodeURIComponent(id)}`, body);
}
async deleteConsent(id: string): Promise<void> {
  return this.request("DELETE", `/consents/${encodeURIComponent(id)}`);
}

  async listPolicy(limit = 20, offset = 0): Promise<{ items: Policy[]; total: number }> {
  return this.request("GET", `/policies?limit=${limit}&offset=${offset}`);
}
async getPolicy(id: string): Promise<Policy> {
  return this.request("GET", `/policies/${encodeURIComponent(id)}`);
}
async createPolicy(body: Policy): Promise<Policy> {
  return this.request("POST", `/policies`, body);
}
async replacePolicy(id: string, body: Policy): Promise<Policy> {
  return this.request("PUT", `/policies/${encodeURIComponent(id)}`, body);
}
async deletePolicy(id: string): Promise<void> {
  return this.request("DELETE", `/policies/${encodeURIComponent(id)}`);
}
}

export default AiEmbodimentEthicsClient;
