// index.ts — Reference TypeScript SDK skeleton for WIA `nuclear-energy`.
// Demonstrates the request/response contract documented under spec/.
// Production deployments should regenerate from the canonical OpenAPI doc.

import type { ListResponse, ResourceRef, WiaError } from "./types";

export interface WiaClientConfig {
  baseUrl: string;
  token?: string;
  version?: string;
  fetchImpl?: typeof fetch;
}

export class WiaClient {
  private cfg: Required<Pick<WiaClientConfig, "baseUrl">> & WiaClientConfig;

  constructor(cfg: WiaClientConfig) {
    this.cfg = {
      version: "1.0.0",
      fetchImpl: globalThis.fetch,
      ...cfg,
    };
  }

  private async request<T>(method: string, path: string, body?: unknown): Promise<T> {
    const f = this.cfg.fetchImpl ?? globalThis.fetch;
    const res = await f(`${this.cfg.baseUrl}${path}`, {
      method,
      headers: {
        "content-type": "application/json",
        "x-wia-nuclear-energy-version": this.cfg.version!,
        ...(this.cfg.token ? { authorization: `Bearer ${this.cfg.token}` } : {}),
      },
      body: body ? JSON.stringify(body) : undefined,
    });
    if (!res.ok) {
      const err = (await res.json().catch(() => null)) as WiaError | null;
      throw Object.assign(new Error(err?.title ?? `HTTP ${res.status}`), { wia: err });
    }
    return (await res.json()) as T;
  }

  list(): Promise<ListResponse<ResourceRef>> {
    return this.request("GET", "/resources");
  }

  get(id: string): Promise<ResourceRef> {
    return this.request("GET", `/resources/${id}`);
  }

  create(payload: unknown): Promise<ResourceRef> {
    return this.request("POST", "/resources", payload);
  }
}

export { } from "./types";
