/**
 * WIA Pet Care Robot — TypeScript SDK
 * Companion-pet robotic care: feeding, monitoring, and welfare interaction
 *
 * Production endpoint: https://api.wia.live/pet-care-robot/v1
 */

export interface Robot {
  id?: string;
  model?: string;
  status?: string;
  battery?: number;
}
export interface Pet {
  id?: string;
  species?: string;
  name?: string;
  ownerId?: string;
}
export interface CareSession {
  id?: string;
  robotId?: string;
  petId?: string;
  startedAt?: string;
  activities?: unknown[];
}
export interface Schedule {
  id?: string;
  petId?: string;
  type?: string;
  cron?: string;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class PetCareRobotClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/pet-care-robot/v1";
    this.apiKey = opts.apiKey;
    this.fetchImpl = opts.fetch ?? fetch;
  }

  private async request<T>(method: string, path: string, body?: unknown): Promise<T> {
    const headers: Record<string, string> = { "Content-Type": "application/json" };
    if (this.apiKey) headers["X-API-Key"] = this.apiKey;
    const res = await this.fetchImpl(this.baseUrl + path, {
      method, headers,
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
  async listPet(limit = 20, offset = 0): Promise<{ items: Pet[]; total: number }> {
    return this.request("GET", `/pets?limit=${limit}&offset=${offset}`);
  }
  async getPet(id: string): Promise<Pet> {
    return this.request("GET", `/pets/${encodeURIComponent(id)}`);
  }
  async createPet(body: Pet): Promise<Pet> {
    return this.request("POST", `/pets`, body);
  }
  async replacePet(id: string, body: Pet): Promise<Pet> {
    return this.request("PUT", `/pets/${encodeURIComponent(id)}`, body);
  }
  async deletePet(id: string): Promise<void> {
    return this.request("DELETE", `/pets/${encodeURIComponent(id)}`);
  }
  async listCareSession(limit = 20, offset = 0): Promise<{ items: CareSession[]; total: number }> {
    return this.request("GET", `/care-sessions?limit=${limit}&offset=${offset}`);
  }
  async getCareSession(id: string): Promise<CareSession> {
    return this.request("GET", `/care-sessions/${encodeURIComponent(id)}`);
  }
  async createCareSession(body: CareSession): Promise<CareSession> {
    return this.request("POST", `/care-sessions`, body);
  }
  async replaceCareSession(id: string, body: CareSession): Promise<CareSession> {
    return this.request("PUT", `/care-sessions/${encodeURIComponent(id)}`, body);
  }
  async deleteCareSession(id: string): Promise<void> {
    return this.request("DELETE", `/care-sessions/${encodeURIComponent(id)}`);
  }
  async listSchedule(limit = 20, offset = 0): Promise<{ items: Schedule[]; total: number }> {
    return this.request("GET", `/schedules?limit=${limit}&offset=${offset}`);
  }
  async getSchedule(id: string): Promise<Schedule> {
    return this.request("GET", `/schedules/${encodeURIComponent(id)}`);
  }
  async createSchedule(body: Schedule): Promise<Schedule> {
    return this.request("POST", `/schedules`, body);
  }
  async replaceSchedule(id: string, body: Schedule): Promise<Schedule> {
    return this.request("PUT", `/schedules/${encodeURIComponent(id)}`, body);
  }
  async deleteSchedule(id: string): Promise<void> {
    return this.request("DELETE", `/schedules/${encodeURIComponent(id)}`);
  }
}

export default PetCareRobotClient;
