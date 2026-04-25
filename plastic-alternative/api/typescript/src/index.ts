// @wia-official/plastic-alternative — reference SDK skeleton (TypeScript)
//
// This skeleton demonstrates the WIA Plastic Alternative standard's
// request/response contract documented under ../../spec/PHASE-2-API.md.
// It is informative reference material; production deployments may diverge
// as long as they preserve the PHASE contract.

export type WiaTier = "Tier 1 — Self-declared" | "Tier 2 — Third-party assessed" | "Tier 3 — Accredited";

export interface MaterialProfile {
  id: string;
  name: string;
  feedstock: "PLA" | "PHA" | "starch-blend" | "cellulose" | "lignin" | "other";
  biodegradation_pathway: "industrial-compost" | "home-compost" | "marine" | "soil" | "anaerobic";
  certified_under?: string[];
}

export interface ConformanceRecord {
  envelope_version: "wia-plastic-alt/1";
  declared_tier: WiaTier;
  material: MaterialProfile;
  test_evidence_pointer: string;
  retention_years: number;
  crosswalk: ReadonlyArray<string>;
}

export class PlasticAlternativeClient {
  constructor(private readonly baseUrl: string, private readonly token?: string) {}

  private headers(): Record<string, string> {
    const h: Record<string, string> = { "Content-Type": "application/json" };
    if (this.token) h["Authorization"] = `Bearer ${this.token}`;
    return h;
  }

  async listMaterials(): Promise<MaterialProfile[]> {
    const r = await fetch(`${this.baseUrl}/materials`, { headers: this.headers() });
    if (!r.ok) throw new Error(`list failed: ${r.status}`);
    return r.json();
  }

  async getMaterial(id: string): Promise<MaterialProfile> {
    const r = await fetch(`${this.baseUrl}/materials/${encodeURIComponent(id)}`, { headers: this.headers() });
    if (!r.ok) throw new Error(`get failed: ${r.status}`);
    return r.json();
  }

  async submitConformance(rec: ConformanceRecord): Promise<{ accepted: boolean; receipt: string }> {
    const r = await fetch(`${this.baseUrl}/conformance`, {
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
  "ISO 17088:2021",
  "ISO 14855-1:2012",
  "ASTM D6400",
  "EN 13432:2000",
  "IETF RFC 9457",
  "W3C PROV-DM",
] as const;
