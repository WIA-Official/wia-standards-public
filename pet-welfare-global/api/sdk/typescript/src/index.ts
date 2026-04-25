/**
 * WIA Pet Welfare Global — TypeScript SDK
 * Global companion-animal welfare indicators and incident reporting
 *
 * Production endpoint: https://api.wia.live/pet-welfare-global/v1
 */

export interface Region {
  id?: string;
  country?: string;
  subdivision?: string;
}
export interface WelfareIndicator {
  id?: string;
  regionId?: string;
  metric?: string;
  value?: number;
  year?: number;
}
export interface Incident {
  id?: string;
  regionId?: string;
  category?: string;
  severity?: string;
}
export interface Organization {
  id?: string;
  name?: string;
  type?: string;
  regionId?: string;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class PetWelfareGlobalClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/pet-welfare-global/v1";
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

    async listRegion(limit = 20, offset = 0): Promise<{ items: Region[]; total: number }> {
    return this.request("GET", `/regions?limit=${limit}&offset=${offset}`);
  }
  async getRegion(id: string): Promise<Region> {
    return this.request("GET", `/regions/${encodeURIComponent(id)}`);
  }
  async createRegion(body: Region): Promise<Region> {
    return this.request("POST", `/regions`, body);
  }
  async replaceRegion(id: string, body: Region): Promise<Region> {
    return this.request("PUT", `/regions/${encodeURIComponent(id)}`, body);
  }
  async deleteRegion(id: string): Promise<void> {
    return this.request("DELETE", `/regions/${encodeURIComponent(id)}`);
  }
  async listWelfareIndicator(limit = 20, offset = 0): Promise<{ items: WelfareIndicator[]; total: number }> {
    return this.request("GET", `/welfare-indicators?limit=${limit}&offset=${offset}`);
  }
  async getWelfareIndicator(id: string): Promise<WelfareIndicator> {
    return this.request("GET", `/welfare-indicators/${encodeURIComponent(id)}`);
  }
  async createWelfareIndicator(body: WelfareIndicator): Promise<WelfareIndicator> {
    return this.request("POST", `/welfare-indicators`, body);
  }
  async replaceWelfareIndicator(id: string, body: WelfareIndicator): Promise<WelfareIndicator> {
    return this.request("PUT", `/welfare-indicators/${encodeURIComponent(id)}`, body);
  }
  async deleteWelfareIndicator(id: string): Promise<void> {
    return this.request("DELETE", `/welfare-indicators/${encodeURIComponent(id)}`);
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
  async listOrganization(limit = 20, offset = 0): Promise<{ items: Organization[]; total: number }> {
    return this.request("GET", `/organizations?limit=${limit}&offset=${offset}`);
  }
  async getOrganization(id: string): Promise<Organization> {
    return this.request("GET", `/organizations/${encodeURIComponent(id)}`);
  }
  async createOrganization(body: Organization): Promise<Organization> {
    return this.request("POST", `/organizations`, body);
  }
  async replaceOrganization(id: string, body: Organization): Promise<Organization> {
    return this.request("PUT", `/organizations/${encodeURIComponent(id)}`, body);
  }
  async deleteOrganization(id: string): Promise<void> {
    return this.request("DELETE", `/organizations/${encodeURIComponent(id)}`);
  }
}

export default PetWelfareGlobalClient;
