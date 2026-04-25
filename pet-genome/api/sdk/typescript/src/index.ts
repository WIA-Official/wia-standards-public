/**
 * WIA Pet Genome — TypeScript SDK
 * Companion-animal genome data submission and breed/disease screening
 *
 * Production endpoint: https://api.wia.live/pet-genome/v1
 */

export interface Sample {
  id?: string;
  petId?: string;
  collectedAt?: string;
  status?: string;
}
export interface Sequence {
  id?: string;
  sampleId?: string;
  platform?: string;
  coverage?: number;
}
export interface BreedReport {
  id?: string;
  sampleId?: string;
  primaryBreed?: string;
  composition?: Record<string, unknown>;
}
export interface HealthMarker {
  id?: string;
  sampleId?: string;
  marker?: string;
  riskCategory?: string;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class PetGenomeClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/pet-genome/v1";
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

    async listSample(limit = 20, offset = 0): Promise<{ items: Sample[]; total: number }> {
    return this.request("GET", `/samples?limit=${limit}&offset=${offset}`);
  }
  async getSample(id: string): Promise<Sample> {
    return this.request("GET", `/samples/${encodeURIComponent(id)}`);
  }
  async createSample(body: Sample): Promise<Sample> {
    return this.request("POST", `/samples`, body);
  }
  async replaceSample(id: string, body: Sample): Promise<Sample> {
    return this.request("PUT", `/samples/${encodeURIComponent(id)}`, body);
  }
  async deleteSample(id: string): Promise<void> {
    return this.request("DELETE", `/samples/${encodeURIComponent(id)}`);
  }
  async listSequence(limit = 20, offset = 0): Promise<{ items: Sequence[]; total: number }> {
    return this.request("GET", `/sequences?limit=${limit}&offset=${offset}`);
  }
  async getSequence(id: string): Promise<Sequence> {
    return this.request("GET", `/sequences/${encodeURIComponent(id)}`);
  }
  async createSequence(body: Sequence): Promise<Sequence> {
    return this.request("POST", `/sequences`, body);
  }
  async replaceSequence(id: string, body: Sequence): Promise<Sequence> {
    return this.request("PUT", `/sequences/${encodeURIComponent(id)}`, body);
  }
  async deleteSequence(id: string): Promise<void> {
    return this.request("DELETE", `/sequences/${encodeURIComponent(id)}`);
  }
  async listBreedReport(limit = 20, offset = 0): Promise<{ items: BreedReport[]; total: number }> {
    return this.request("GET", `/breed-reports?limit=${limit}&offset=${offset}`);
  }
  async getBreedReport(id: string): Promise<BreedReport> {
    return this.request("GET", `/breed-reports/${encodeURIComponent(id)}`);
  }
  async createBreedReport(body: BreedReport): Promise<BreedReport> {
    return this.request("POST", `/breed-reports`, body);
  }
  async replaceBreedReport(id: string, body: BreedReport): Promise<BreedReport> {
    return this.request("PUT", `/breed-reports/${encodeURIComponent(id)}`, body);
  }
  async deleteBreedReport(id: string): Promise<void> {
    return this.request("DELETE", `/breed-reports/${encodeURIComponent(id)}`);
  }
  async listHealthMarker(limit = 20, offset = 0): Promise<{ items: HealthMarker[]; total: number }> {
    return this.request("GET", `/health-markers?limit=${limit}&offset=${offset}`);
  }
  async getHealthMarker(id: string): Promise<HealthMarker> {
    return this.request("GET", `/health-markers/${encodeURIComponent(id)}`);
  }
  async createHealthMarker(body: HealthMarker): Promise<HealthMarker> {
    return this.request("POST", `/health-markers`, body);
  }
  async replaceHealthMarker(id: string, body: HealthMarker): Promise<HealthMarker> {
    return this.request("PUT", `/health-markers/${encodeURIComponent(id)}`, body);
  }
  async deleteHealthMarker(id: string): Promise<void> {
    return this.request("DELETE", `/health-markers/${encodeURIComponent(id)}`);
  }
}

export default PetGenomeClient;
