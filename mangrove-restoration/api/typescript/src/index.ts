// @wia-official/mangrove-restoration — reference SDK skeleton (TypeScript)

export type WiaTier = "Tier 1 — Self-declared" | "Tier 2 — Third-party assessed" | "Tier 3 — Accredited";

export interface RestorationSite {
  id: string;
  name: string;
  geometry_pointer: string;
  baseline_carbon_pointer: string;
  active: boolean;
}

export interface RestorationRecord {
  envelope_version: "wia-mangrove-restoration/1";
  declared_tier: WiaTier;
  sites: RestorationSite[];
  monitoring_cadence: "monthly" | "quarterly" | "annual";
  retention_years: number;
  crosswalk: ReadonlyArray<string>;
}

export class MangroveRestorationClient {
  constructor(private readonly baseUrl: string, private readonly token?: string) {}
  private headers(): Record<string, string> {
    const h: Record<string, string> = { "Content-Type": "application/json" };
    if (this.token) h["Authorization"] = `Bearer ${this.token}`;
    return h;
  }
  async listSites(): Promise<RestorationSite[]> {
    const r = await fetch(`${this.baseUrl}/sites`, { headers: this.headers() });
    if (!r.ok) throw new Error(`list failed: ${r.status}`);
    return r.json();
  }
  async submitRecord(rec: RestorationRecord): Promise<{ accepted: boolean; receipt: string }> {
    const r = await fetch(`${this.baseUrl}/records`, {
      method: "POST",
      headers: this.headers(),
      body: JSON.stringify(rec),
    });
    if (!r.ok) throw new Error(`submit failed: ${r.status}`);
    return r.json();
  }
}

export const CROSSWALK = [
  "ISO/IEC 17065:2012",
  "ISO 14001:2015",
  "ISO 14064-2:2019",
  "IETF RFC 9457",
  "W3C PROV-DM",
] as const;
