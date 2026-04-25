/**
 * WIA AI Human Coexistence — TypeScript SDK
 * Human detection, zone management, and safety arbitration for shared AI/human spaces
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/ai-human-coexistence/v1
 */

export interface Detection {
  id?: string;
  trackingId?: string;
  confidence?: number;
  bbox?: unknown[];
  timestamp?: string;
}

export interface Zone {
  id?: string;
  name?: string;
  type?: string;
  polygon?: unknown[];
}

export interface Incident {
  id?: string;
  zoneId?: string;
  severity?: string;
  description?: string;
}

export interface Policy {
  id?: string;
  zoneId?: string;
  rule?: string;
  priority?: number;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class AiHumanCoexistenceClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/ai-human-coexistence/v1";
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

  async listDetection(limit = 20, offset = 0): Promise<{ items: Detection[]; total: number }> {
  return this.request("GET", `/detections?limit=${limit}&offset=${offset}`);
}
async getDetection(id: string): Promise<Detection> {
  return this.request("GET", `/detections/${encodeURIComponent(id)}`);
}
async createDetection(body: Detection): Promise<Detection> {
  return this.request("POST", `/detections`, body);
}
async replaceDetection(id: string, body: Detection): Promise<Detection> {
  return this.request("PUT", `/detections/${encodeURIComponent(id)}`, body);
}
async deleteDetection(id: string): Promise<void> {
  return this.request("DELETE", `/detections/${encodeURIComponent(id)}`);
}

  async listZone(limit = 20, offset = 0): Promise<{ items: Zone[]; total: number }> {
  return this.request("GET", `/zones?limit=${limit}&offset=${offset}`);
}
async getZone(id: string): Promise<Zone> {
  return this.request("GET", `/zones/${encodeURIComponent(id)}`);
}
async createZone(body: Zone): Promise<Zone> {
  return this.request("POST", `/zones`, body);
}
async replaceZone(id: string, body: Zone): Promise<Zone> {
  return this.request("PUT", `/zones/${encodeURIComponent(id)}`, body);
}
async deleteZone(id: string): Promise<void> {
  return this.request("DELETE", `/zones/${encodeURIComponent(id)}`);
}

  async listIncident(limit = 20, offset = 0): Promise<{ items: Incident[]; total: number }> {
  return this.request("GET", `/incidents?limit=${limit}&offset=${offset}`);
}
async getIncident(id: string): Promise<Incident> {
  return this.request("GET", `/incidents/${encodeURIComponent(id)}`);
}
async createIncident(body: Incident): Promise<Incident> {
  return this.request("POST", `/incidents`, body);
}
async replaceIncident(id: string, body: Incident): Promise<Incident> {
  return this.request("PUT", `/incidents/${encodeURIComponent(id)}`, body);
}
async deleteIncident(id: string): Promise<void> {
  return this.request("DELETE", `/incidents/${encodeURIComponent(id)}`);
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

export default AiHumanCoexistenceClient;
