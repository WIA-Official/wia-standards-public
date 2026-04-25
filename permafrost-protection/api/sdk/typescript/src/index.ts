/**
 * WIA Permafrost Protection — TypeScript SDK
 * Permafrost monitoring station registration, thermal observations, and degradation alerts
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/permafrost/v1
 */

export interface Station {
  id?: string;
  name?: string;
  location?: Record<string, unknown>;
  activeLayerDepth?: number;
}

export interface Observation {
  id?: string;
  stationId?: string;
  temperature?: number;
  depth?: number;
  timestamp?: string;
}

export interface Alert {
  id?: string;
  stationId?: string;
  type?: string;
  severity?: string;
}

export interface Borehole {
  id?: string;
  stationId?: string;
  totalDepth?: number;
  instruments?: unknown[];
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class PermafrostProtectionClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/permafrost/v1";
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

  async listStation(limit = 20, offset = 0): Promise<{ items: Station[]; total: number }> {
  return this.request("GET", `/stations?limit=${limit}&offset=${offset}`);
}
async getStation(id: string): Promise<Station> {
  return this.request("GET", `/stations/${encodeURIComponent(id)}`);
}
async createStation(body: Station): Promise<Station> {
  return this.request("POST", `/stations`, body);
}
async replaceStation(id: string, body: Station): Promise<Station> {
  return this.request("PUT", `/stations/${encodeURIComponent(id)}`, body);
}
async deleteStation(id: string): Promise<void> {
  return this.request("DELETE", `/stations/${encodeURIComponent(id)}`);
}

  async listObservation(limit = 20, offset = 0): Promise<{ items: Observation[]; total: number }> {
  return this.request("GET", `/observations?limit=${limit}&offset=${offset}`);
}
async getObservation(id: string): Promise<Observation> {
  return this.request("GET", `/observations/${encodeURIComponent(id)}`);
}
async createObservation(body: Observation): Promise<Observation> {
  return this.request("POST", `/observations`, body);
}
async replaceObservation(id: string, body: Observation): Promise<Observation> {
  return this.request("PUT", `/observations/${encodeURIComponent(id)}`, body);
}
async deleteObservation(id: string): Promise<void> {
  return this.request("DELETE", `/observations/${encodeURIComponent(id)}`);
}

  async listAlert(limit = 20, offset = 0): Promise<{ items: Alert[]; total: number }> {
  return this.request("GET", `/alerts?limit=${limit}&offset=${offset}`);
}
async getAlert(id: string): Promise<Alert> {
  return this.request("GET", `/alerts/${encodeURIComponent(id)}`);
}
async createAlert(body: Alert): Promise<Alert> {
  return this.request("POST", `/alerts`, body);
}
async replaceAlert(id: string, body: Alert): Promise<Alert> {
  return this.request("PUT", `/alerts/${encodeURIComponent(id)}`, body);
}
async deleteAlert(id: string): Promise<void> {
  return this.request("DELETE", `/alerts/${encodeURIComponent(id)}`);
}

  async listBorehole(limit = 20, offset = 0): Promise<{ items: Borehole[]; total: number }> {
  return this.request("GET", `/boreholes?limit=${limit}&offset=${offset}`);
}
async getBorehole(id: string): Promise<Borehole> {
  return this.request("GET", `/boreholes/${encodeURIComponent(id)}`);
}
async createBorehole(body: Borehole): Promise<Borehole> {
  return this.request("POST", `/boreholes`, body);
}
async replaceBorehole(id: string, body: Borehole): Promise<Borehole> {
  return this.request("PUT", `/boreholes/${encodeURIComponent(id)}`, body);
}
async deleteBorehole(id: string): Promise<void> {
  return this.request("DELETE", `/boreholes/${encodeURIComponent(id)}`);
}
}

export default PermafrostProtectionClient;
