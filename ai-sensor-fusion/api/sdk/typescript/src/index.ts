/**
 * WIA AI Sensor Fusion — TypeScript SDK
 * Multi-sensor fusion pipeline registration and fused-state retrieval
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/sensor-fusion/v1
 */

export interface Sensor {
  id?: string;
  type?: string;
  rate?: number;
  status?: string;
}

export interface FusionPipeline {
  id?: string;
  name?: string;
  sensors?: unknown[];
  algorithm?: string;
}

export interface FusedState {
  pipelineId?: string;
  timestamp?: string;
  state?: Record<string, unknown>;
  covariance?: unknown[];
}

export interface Calibration {
  id?: string;
  sensorId?: string;
  transform?: Record<string, unknown>;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class AiSensorFusionClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/sensor-fusion/v1";
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

  async listSensor(limit = 20, offset = 0): Promise<{ items: Sensor[]; total: number }> {
  return this.request("GET", `/sensors?limit=${limit}&offset=${offset}`);
}
async getSensor(id: string): Promise<Sensor> {
  return this.request("GET", `/sensors/${encodeURIComponent(id)}`);
}
async createSensor(body: Sensor): Promise<Sensor> {
  return this.request("POST", `/sensors`, body);
}
async replaceSensor(id: string, body: Sensor): Promise<Sensor> {
  return this.request("PUT", `/sensors/${encodeURIComponent(id)}`, body);
}
async deleteSensor(id: string): Promise<void> {
  return this.request("DELETE", `/sensors/${encodeURIComponent(id)}`);
}

  async listFusionPipeline(limit = 20, offset = 0): Promise<{ items: FusionPipeline[]; total: number }> {
  return this.request("GET", `/pipelines?limit=${limit}&offset=${offset}`);
}
async getFusionPipeline(id: string): Promise<FusionPipeline> {
  return this.request("GET", `/pipelines/${encodeURIComponent(id)}`);
}
async createFusionPipeline(body: FusionPipeline): Promise<FusionPipeline> {
  return this.request("POST", `/pipelines`, body);
}
async replaceFusionPipeline(id: string, body: FusionPipeline): Promise<FusionPipeline> {
  return this.request("PUT", `/pipelines/${encodeURIComponent(id)}`, body);
}
async deleteFusionPipeline(id: string): Promise<void> {
  return this.request("DELETE", `/pipelines/${encodeURIComponent(id)}`);
}

  async listFusedState(limit = 20, offset = 0): Promise<{ items: FusedState[]; total: number }> {
  return this.request("GET", `/fused-states?limit=${limit}&offset=${offset}`);
}
async getFusedState(id: string): Promise<FusedState> {
  return this.request("GET", `/fused-states/${encodeURIComponent(id)}`);
}
async createFusedState(body: FusedState): Promise<FusedState> {
  return this.request("POST", `/fused-states`, body);
}
async replaceFusedState(id: string, body: FusedState): Promise<FusedState> {
  return this.request("PUT", `/fused-states/${encodeURIComponent(id)}`, body);
}
async deleteFusedState(id: string): Promise<void> {
  return this.request("DELETE", `/fused-states/${encodeURIComponent(id)}`);
}

  async listCalibration(limit = 20, offset = 0): Promise<{ items: Calibration[]; total: number }> {
  return this.request("GET", `/calibrations?limit=${limit}&offset=${offset}`);
}
async getCalibration(id: string): Promise<Calibration> {
  return this.request("GET", `/calibrations/${encodeURIComponent(id)}`);
}
async createCalibration(body: Calibration): Promise<Calibration> {
  return this.request("POST", `/calibrations`, body);
}
async replaceCalibration(id: string, body: Calibration): Promise<Calibration> {
  return this.request("PUT", `/calibrations/${encodeURIComponent(id)}`, body);
}
async deleteCalibration(id: string): Promise<void> {
  return this.request("DELETE", `/calibrations/${encodeURIComponent(id)}`);
}
}

export default AiSensorFusionClient;
