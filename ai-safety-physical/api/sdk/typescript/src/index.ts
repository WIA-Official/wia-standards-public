/**
 * WIA AI Safety Physical — TypeScript SDK
 * Safety-zone definition and enforcement for physically-acting AI systems
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/ai-safety-physical/v1
 */

export interface SafetyZone {
  id?: string;
  name?: string;
  polygon?: unknown[];
  hazardLevel?: string;
}

export interface Hazard {
  id?: string;
  zoneId?: string;
  type?: string;
  severity?: string;
}

export interface Incident {
  id?: string;
  zoneId?: string;
  agentId?: string;
  description?: string;
}

export interface EmergencyStop {
  id?: string;
  zoneId?: string;
  triggeredBy?: string;
  triggeredAt?: string;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class AiSafetyPhysicalClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/ai-safety-physical/v1";
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

  async listSafetyZone(limit = 20, offset = 0): Promise<{ items: SafetyZone[]; total: number }> {
  return this.request("GET", `/safety-zones?limit=${limit}&offset=${offset}`);
}
async getSafetyZone(id: string): Promise<SafetyZone> {
  return this.request("GET", `/safety-zones/${encodeURIComponent(id)}`);
}
async createSafetyZone(body: SafetyZone): Promise<SafetyZone> {
  return this.request("POST", `/safety-zones`, body);
}
async replaceSafetyZone(id: string, body: SafetyZone): Promise<SafetyZone> {
  return this.request("PUT", `/safety-zones/${encodeURIComponent(id)}`, body);
}
async deleteSafetyZone(id: string): Promise<void> {
  return this.request("DELETE", `/safety-zones/${encodeURIComponent(id)}`);
}

  async listHazard(limit = 20, offset = 0): Promise<{ items: Hazard[]; total: number }> {
  return this.request("GET", `/hazards?limit=${limit}&offset=${offset}`);
}
async getHazard(id: string): Promise<Hazard> {
  return this.request("GET", `/hazards/${encodeURIComponent(id)}`);
}
async createHazard(body: Hazard): Promise<Hazard> {
  return this.request("POST", `/hazards`, body);
}
async replaceHazard(id: string, body: Hazard): Promise<Hazard> {
  return this.request("PUT", `/hazards/${encodeURIComponent(id)}`, body);
}
async deleteHazard(id: string): Promise<void> {
  return this.request("DELETE", `/hazards/${encodeURIComponent(id)}`);
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

  async listEmergencyStop(limit = 20, offset = 0): Promise<{ items: EmergencyStop[]; total: number }> {
  return this.request("GET", `/emergency-stops?limit=${limit}&offset=${offset}`);
}
async getEmergencyStop(id: string): Promise<EmergencyStop> {
  return this.request("GET", `/emergency-stops/${encodeURIComponent(id)}`);
}
async createEmergencyStop(body: EmergencyStop): Promise<EmergencyStop> {
  return this.request("POST", `/emergency-stops`, body);
}
async replaceEmergencyStop(id: string, body: EmergencyStop): Promise<EmergencyStop> {
  return this.request("PUT", `/emergency-stops/${encodeURIComponent(id)}`, body);
}
async deleteEmergencyStop(id: string): Promise<void> {
  return this.request("DELETE", `/emergency-stops/${encodeURIComponent(id)}`);
}
}

export default AiSafetyPhysicalClient;
