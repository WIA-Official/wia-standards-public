/**
 * WIA Rainforest Conservation Standard — Reference TypeScript SDK
 * Standard: WIA-RAINFOREST v1.0.0
 * Philosophy: 弘益人間 — Benefit All Humanity
 *
 * Reference implementation. Mirrors the Phase 2 HTTP surface defined in
 * spec/PHASE-2-API.md. Production implementations should pin to a
 * specific minor version of the standard and verify the host's
 * /.well-known/wia-rainforest-conservation discovery document on
 * startup.
 */

export interface WiaRainforestVersion {
  major: 1;
  minor: 0;
  patch: 0;
}

export interface DiscoveryDocument {
  wia_rainforest_conservation_version: string;
  operator_id: string;
  endpoints: Record<string, string>;
  supported_signatures: string[];
}

export class RainforestConservationClient {
  constructor(private baseUrl: string, private signingKey: Uint8Array) {}

  async discover(): Promise<DiscoveryDocument> {
    const res = await fetch(`${this.baseUrl}/.well-known/wia-rainforest-conservation`);
    if (!res.ok) throw new Error(`discovery failed: ${res.status}`);
    return await res.json();
  }
}
