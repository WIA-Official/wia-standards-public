/**
 * WIA AI Motor Control — TypeScript SDK
 * Motor registration, command issuance, and trajectory tracking for AI-driven actuators
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/motor-control/v1
 */

export interface Motor {
  id?: string;
  type?: string;
  status?: string;
  position?: number;
  velocity?: number;
}

export interface Command {
  id?: string;
  motorId?: string;
  type?: string;
  payload?: Record<string, unknown>;
  status?: string;
}

export interface Trajectory {
  id?: string;
  motorId?: string;
  waypoints?: unknown[];
  duration?: number;
}

export interface Calibration {
  id?: string;
  motorId?: string;
  offsets?: Record<string, unknown>;
  calibratedAt?: string;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class AiMotorControlClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/motor-control/v1";
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

  async listMotor(limit = 20, offset = 0): Promise<{ items: Motor[]; total: number }> {
  return this.request("GET", `/motors?limit=${limit}&offset=${offset}`);
}
async getMotor(id: string): Promise<Motor> {
  return this.request("GET", `/motors/${encodeURIComponent(id)}`);
}
async createMotor(body: Motor): Promise<Motor> {
  return this.request("POST", `/motors`, body);
}
async replaceMotor(id: string, body: Motor): Promise<Motor> {
  return this.request("PUT", `/motors/${encodeURIComponent(id)}`, body);
}
async deleteMotor(id: string): Promise<void> {
  return this.request("DELETE", `/motors/${encodeURIComponent(id)}`);
}

  async listCommand(limit = 20, offset = 0): Promise<{ items: Command[]; total: number }> {
  return this.request("GET", `/commands?limit=${limit}&offset=${offset}`);
}
async getCommand(id: string): Promise<Command> {
  return this.request("GET", `/commands/${encodeURIComponent(id)}`);
}
async createCommand(body: Command): Promise<Command> {
  return this.request("POST", `/commands`, body);
}
async replaceCommand(id: string, body: Command): Promise<Command> {
  return this.request("PUT", `/commands/${encodeURIComponent(id)}`, body);
}
async deleteCommand(id: string): Promise<void> {
  return this.request("DELETE", `/commands/${encodeURIComponent(id)}`);
}

  async listTrajectory(limit = 20, offset = 0): Promise<{ items: Trajectory[]; total: number }> {
  return this.request("GET", `/trajectories?limit=${limit}&offset=${offset}`);
}
async getTrajectory(id: string): Promise<Trajectory> {
  return this.request("GET", `/trajectories/${encodeURIComponent(id)}`);
}
async createTrajectory(body: Trajectory): Promise<Trajectory> {
  return this.request("POST", `/trajectories`, body);
}
async replaceTrajectory(id: string, body: Trajectory): Promise<Trajectory> {
  return this.request("PUT", `/trajectories/${encodeURIComponent(id)}`, body);
}
async deleteTrajectory(id: string): Promise<void> {
  return this.request("DELETE", `/trajectories/${encodeURIComponent(id)}`);
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

export default AiMotorControlClient;
