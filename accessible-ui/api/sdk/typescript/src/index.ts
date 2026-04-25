/**
 * WIA Accessible UI — TypeScript SDK
 * WCAG 2.1 AAA accessibility validation, contrast checking, ARIA management
 *
 * Generated from openapi-3.0.yaml. Production endpoint:
 *   https://api.wia.live/accessible-ui/v1
 */

export interface Audit {
  id?: string;
  url?: string;
  level?: string;
  violations?: number;
  status?: string;
}

export interface Violation {
  id?: string;
  ruleId?: string;
  impact?: string;
  element?: string;
  message?: string;
}

export interface Component {
  id?: string;
  type?: string;
  framework?: string;
  ariaScore?: number;
}

export interface ContrastCheck {
  id?: string;
  foreground?: string;
  background?: string;
  ratio?: number;
  passes?: boolean;
}

export interface ClientOptions {
  baseUrl?: string;
  apiKey?: string;
  fetch?: typeof fetch;
}

export class AccessibleUiClient {
  private baseUrl: string;
  private apiKey?: string;
  private fetchImpl: typeof fetch;

  constructor(opts: ClientOptions = {}) {
    this.baseUrl = opts.baseUrl ?? "https://api.wia.live/accessible-ui/v1";
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

  async listAudit(limit = 20, offset = 0): Promise<{ items: Audit[]; total: number }> {
  return this.request("GET", `/audits?limit=${limit}&offset=${offset}`);
}
async getAudit(id: string): Promise<Audit> {
  return this.request("GET", `/audits/${encodeURIComponent(id)}`);
}
async createAudit(body: Audit): Promise<Audit> {
  return this.request("POST", `/audits`, body);
}
async replaceAudit(id: string, body: Audit): Promise<Audit> {
  return this.request("PUT", `/audits/${encodeURIComponent(id)}`, body);
}
async deleteAudit(id: string): Promise<void> {
  return this.request("DELETE", `/audits/${encodeURIComponent(id)}`);
}

  async listViolation(limit = 20, offset = 0): Promise<{ items: Violation[]; total: number }> {
  return this.request("GET", `/violations?limit=${limit}&offset=${offset}`);
}
async getViolation(id: string): Promise<Violation> {
  return this.request("GET", `/violations/${encodeURIComponent(id)}`);
}
async createViolation(body: Violation): Promise<Violation> {
  return this.request("POST", `/violations`, body);
}
async replaceViolation(id: string, body: Violation): Promise<Violation> {
  return this.request("PUT", `/violations/${encodeURIComponent(id)}`, body);
}
async deleteViolation(id: string): Promise<void> {
  return this.request("DELETE", `/violations/${encodeURIComponent(id)}`);
}

  async listComponent(limit = 20, offset = 0): Promise<{ items: Component[]; total: number }> {
  return this.request("GET", `/components?limit=${limit}&offset=${offset}`);
}
async getComponent(id: string): Promise<Component> {
  return this.request("GET", `/components/${encodeURIComponent(id)}`);
}
async createComponent(body: Component): Promise<Component> {
  return this.request("POST", `/components`, body);
}
async replaceComponent(id: string, body: Component): Promise<Component> {
  return this.request("PUT", `/components/${encodeURIComponent(id)}`, body);
}
async deleteComponent(id: string): Promise<void> {
  return this.request("DELETE", `/components/${encodeURIComponent(id)}`);
}

  async listContrastCheck(limit = 20, offset = 0): Promise<{ items: ContrastCheck[]; total: number }> {
  return this.request("GET", `/contrast-checks?limit=${limit}&offset=${offset}`);
}
async getContrastCheck(id: string): Promise<ContrastCheck> {
  return this.request("GET", `/contrast-checks/${encodeURIComponent(id)}`);
}
async createContrastCheck(body: ContrastCheck): Promise<ContrastCheck> {
  return this.request("POST", `/contrast-checks`, body);
}
async replaceContrastCheck(id: string, body: ContrastCheck): Promise<ContrastCheck> {
  return this.request("PUT", `/contrast-checks/${encodeURIComponent(id)}`, body);
}
async deleteContrastCheck(id: string): Promise<void> {
  return this.request("DELETE", `/contrast-checks/${encodeURIComponent(id)}`);
}
}

export default AccessibleUiClient;
