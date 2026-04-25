/**
 * WIA Agricultural Robot — TypeScript SDK
 * Autonomous agricultural-robot fleet registration, telemetry, and task assignment
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/agricultural-robot/v1
 */

export interface Robot {
  id?: string;
  model?: string;
  status?: string;
  battery?: number;
  location?: Record<string, unknown>;
}

export interface Telemetry {
  robotId?: string;
  timestamp?: string;
  metrics?: Record<string, unknown>;
}

export interface Task {
  id?: string;
  robotId?: string;
  type?: string;
  status?: string;
  priority?: number;
}

export interface FieldMap {
  id?: string;
  name?: string;
  boundary?: Record<string, unknown>;
  zones?: unknown[];
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class AgriculturalRobotClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/agricultural-robot/v1";
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

  async listTelemetry(limit = 20, offset = 0): Promise<{ items: Telemetry[]; total: number }> {
  return this.request("GET", `/telemetry?limit=${limit}&offset=${offset}`);
}
async getTelemetry(id: string): Promise<Telemetry> {
  return this.request("GET", `/telemetry/${encodeURIComponent(id)}`);
}
async createTelemetry(body: Telemetry): Promise<Telemetry> {
  return this.request("POST", `/telemetry`, body);
}
async replaceTelemetry(id: string, body: Telemetry): Promise<Telemetry> {
  return this.request("PUT", `/telemetry/${encodeURIComponent(id)}`, body);
}
async deleteTelemetry(id: string): Promise<void> {
  return this.request("DELETE", `/telemetry/${encodeURIComponent(id)}`);
}

  async listTask(limit = 20, offset = 0): Promise<{ items: Task[]; total: number }> {
  return this.request("GET", `/tasks?limit=${limit}&offset=${offset}`);
}
async getTask(id: string): Promise<Task> {
  return this.request("GET", `/tasks/${encodeURIComponent(id)}`);
}
async createTask(body: Task): Promise<Task> {
  return this.request("POST", `/tasks`, body);
}
async replaceTask(id: string, body: Task): Promise<Task> {
  return this.request("PUT", `/tasks/${encodeURIComponent(id)}`, body);
}
async deleteTask(id: string): Promise<void> {
  return this.request("DELETE", `/tasks/${encodeURIComponent(id)}`);
}

  async listFieldMap(limit = 20, offset = 0): Promise<{ items: FieldMap[]; total: number }> {
  return this.request("GET", `/field-maps?limit=${limit}&offset=${offset}`);
}
async getFieldMap(id: string): Promise<FieldMap> {
  return this.request("GET", `/field-maps/${encodeURIComponent(id)}`);
}
async createFieldMap(body: FieldMap): Promise<FieldMap> {
  return this.request("POST", `/field-maps`, body);
}
async replaceFieldMap(id: string, body: FieldMap): Promise<FieldMap> {
  return this.request("PUT", `/field-maps/${encodeURIComponent(id)}`, body);
}
async deleteFieldMap(id: string): Promise<void> {
  return this.request("DELETE", `/field-maps/${encodeURIComponent(id)}`);
}
}

export default AgriculturalRobotClient;
