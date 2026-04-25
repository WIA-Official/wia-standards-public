/**
 * WIA AI Robot Interface — TypeScript SDK
 * Unified REST + WSS interface for state, sensor streams, and capability discovery
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/ai-robot-interface/v1
 */

export interface Robot {
  id?: string;
  name?: string;
  model?: string;
  capabilities?: unknown[];
}

export interface State {
  robotId?: string;
  pose?: Record<string, unknown>;
  status?: string;
  timestamp?: string;
}

export interface Sensor {
  id?: string;
  robotId?: string;
  type?: string;
  rate?: number;
}

export interface Capability {
  name?: string;
  description?: string;
  schema?: Record<string, unknown>;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class AiRobotInterfaceClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/ai-robot-interface/v1";
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

  async listRobot(limit = 20, offset = 0): Promise<{ items: Robot[]; total: number }> {
  return this.request("GET", `/robots?limit=${limit}&offset=${offset}`);
}
async getRobot(id: string): Promise<Robot> {
  return this.request("GET", `/robots/${encodeURIComponent(id)}`);
}
async createRobot(body: Robot): Promise<Robot> {
  return this.request("POST", `/robots`, body);
}
async replaceRobot(id: string, body: Robot): Promise<Robot> {
  return this.request("PUT", `/robots/${encodeURIComponent(id)}`, body);
}
async deleteRobot(id: string): Promise<void> {
  return this.request("DELETE", `/robots/${encodeURIComponent(id)}`);
}

  async listState(limit = 20, offset = 0): Promise<{ items: State[]; total: number }> {
  return this.request("GET", `/states?limit=${limit}&offset=${offset}`);
}
async getState(id: string): Promise<State> {
  return this.request("GET", `/states/${encodeURIComponent(id)}`);
}
async createState(body: State): Promise<State> {
  return this.request("POST", `/states`, body);
}
async replaceState(id: string, body: State): Promise<State> {
  return this.request("PUT", `/states/${encodeURIComponent(id)}`, body);
}
async deleteState(id: string): Promise<void> {
  return this.request("DELETE", `/states/${encodeURIComponent(id)}`);
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

  async listCapability(limit = 20, offset = 0): Promise<{ items: Capability[]; total: number }> {
  return this.request("GET", `/capabilities?limit=${limit}&offset=${offset}`);
}
async getCapability(id: string): Promise<Capability> {
  return this.request("GET", `/capabilities/${encodeURIComponent(id)}`);
}
async createCapability(body: Capability): Promise<Capability> {
  return this.request("POST", `/capabilities`, body);
}
async replaceCapability(id: string, body: Capability): Promise<Capability> {
  return this.request("PUT", `/capabilities/${encodeURIComponent(id)}`, body);
}
async deleteCapability(id: string): Promise<void> {
  return this.request("DELETE", `/capabilities/${encodeURIComponent(id)}`);
}
}

export default AiRobotInterfaceClient;
