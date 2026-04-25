// @wia-official/polar-region-protection — reference SDK skeleton (TypeScript)

export type WiaTier = "Tier 1 — Self-declared" | "Tier 2 — Third-party assessed" | "Tier 3 — Accredited";

export interface ProtectedZone {
  id: string;
  name: string;
  region: "arctic" | "antarctic" | "subarctic" | "subantarctic";
  geometry_pointer: string;
  active: boolean;
}

export interface ConservationRecord {
  envelope_version: "wia-polar-region-protection/1";
  declared_tier: WiaTier;
  zones: ProtectedZone[];
  observation_pointer: string;
  retention_years: number;
  crosswalk: ReadonlyArray<string>;
}

export class PolarRegionProtectionClient {
  constructor(private readonly baseUrl: string, private readonly token?: string) {}
  private headers(): Record<string, string> {
    const h: Record<string, string> = { "Content-Type": "application/json" };
    if (this.token) h["Authorization"] = `Bearer ${this.token}`;
    return h;
  }
  async listZones(): Promise<ProtectedZone[]> {
    const r = await fetch(`${this.baseUrl}/zones`, { headers: this.headers() });
    if (!r.ok) throw new Error(`list failed: ${r.status}`);
    return r.json();
  }
  async submitRecord(rec: ConservationRecord): Promise<{ accepted: boolean; receipt: string }> {
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
  "IETF RFC 9457",
  "W3C PROV-DM",
] as const;
