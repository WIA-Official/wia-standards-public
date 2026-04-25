/**
 * WIA Pet Emotion — TypeScript SDK
 * Pet emotion detection and behavioral state classification
 *
 * Production endpoint: https://api.wia.live/pet-emotion/v1
 */

export interface Pet {
  id?: string;
  species?: string;
  name?: string;
  breed?: string;
}
export interface EmotionReading {
  id?: string;
  petId?: string;
  emotion?: string;
  confidence?: number;
  timestamp?: string;
}
export interface BehaviorEvent {
  id?: string;
  petId?: string;
  category?: string;
  intensity?: number;
}
export interface Model {
  id?: string;
  species?: string;
  version?: string;
  metricsUrl?: string;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class PetEmotionClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/pet-emotion/v1";
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
  async listEmotionReading(limit = 20, offset = 0): Promise<{ items: EmotionReading[]; total: number }> {
    return this.request("GET", `/emotion-readings?limit=${limit}&offset=${offset}`);
  }
  async getEmotionReading(id: string): Promise<EmotionReading> {
    return this.request("GET", `/emotion-readings/${encodeURIComponent(id)}`);
  }
  async createEmotionReading(body: EmotionReading): Promise<EmotionReading> {
    return this.request("POST", `/emotion-readings`, body);
  }
  async replaceEmotionReading(id: string, body: EmotionReading): Promise<EmotionReading> {
    return this.request("PUT", `/emotion-readings/${encodeURIComponent(id)}`, body);
  }
  async deleteEmotionReading(id: string): Promise<void> {
    return this.request("DELETE", `/emotion-readings/${encodeURIComponent(id)}`);
  }
  async listBehaviorEvent(limit = 20, offset = 0): Promise<{ items: BehaviorEvent[]; total: number }> {
    return this.request("GET", `/behavior-events?limit=${limit}&offset=${offset}`);
  }
  async getBehaviorEvent(id: string): Promise<BehaviorEvent> {
    return this.request("GET", `/behavior-events/${encodeURIComponent(id)}`);
  }
  async createBehaviorEvent(body: BehaviorEvent): Promise<BehaviorEvent> {
    return this.request("POST", `/behavior-events`, body);
  }
  async replaceBehaviorEvent(id: string, body: BehaviorEvent): Promise<BehaviorEvent> {
    return this.request("PUT", `/behavior-events/${encodeURIComponent(id)}`, body);
  }
  async deleteBehaviorEvent(id: string): Promise<void> {
    return this.request("DELETE", `/behavior-events/${encodeURIComponent(id)}`);
  }
  async listModel(limit = 20, offset = 0): Promise<{ items: Model[]; total: number }> {
    return this.request("GET", `/models?limit=${limit}&offset=${offset}`);
  }
  async getModel(id: string): Promise<Model> {
    return this.request("GET", `/models/${encodeURIComponent(id)}`);
  }
  async createModel(body: Model): Promise<Model> {
    return this.request("POST", `/models`, body);
  }
  async replaceModel(id: string, body: Model): Promise<Model> {
    return this.request("PUT", `/models/${encodeURIComponent(id)}`, body);
  }
  async deleteModel(id: string): Promise<void> {
    return this.request("DELETE", `/models/${encodeURIComponent(id)}`);
  }
}

export default PetEmotionClient;
