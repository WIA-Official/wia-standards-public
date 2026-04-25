// @wia-official/pet-legacy — reference SDK skeleton (TypeScript)
//
// This skeleton demonstrates the WIA Pet Legacy standard's request/response
// contract documented under ../../spec/PHASE-2-API.md. It is informative
// reference material; production deployments may diverge as long as they
// preserve the PHASE contract.

export type WiaTier = "Tier 1 — Self-declared" | "Tier 2 — Third-party assessed" | "Tier 3 — Accredited";

export interface PetRecord {
  id: string;
  name: string;
  species: "dog" | "cat" | "bird" | "rabbit" | "other";
  date_of_birth?: string;
  date_of_passing?: string;
  guardian_id: string;
}

export interface LegacyAsset {
  id: string;
  pet_id: string;
  asset_type: "photograph" | "video" | "audio" | "document" | "biometric-template";
  storage_pointer: string;
  retention_years: number;
}

export interface InheritanceDirective {
  id: string;
  pet_id: string;
  designated_steward: string;
  trigger_event: "guardian_passing" | "guardian_incapacitation" | "explicit_request";
  notarization_pointer?: string;
}

export interface ConformanceRecord {
  envelope_version: "wia-pet-legacy/1";
  declared_tier: WiaTier;
  records: PetRecord[];
  assets: LegacyAsset[];
  directives: InheritanceDirective[];
  crosswalk: ReadonlyArray<string>;
}

export class PetLegacyClient {
  constructor(private readonly baseUrl: string, private readonly token?: string) {}

  private headers(): Record<string, string> {
    const h: Record<string, string> = { "Content-Type": "application/json" };
    if (this.token) h["Authorization"] = `Bearer ${this.token}`;
    return h;
  }

  async listPets(): Promise<PetRecord[]> {
    const r = await fetch(`${this.baseUrl}/pets`, { headers: this.headers() });
    if (!r.ok) throw new Error(`list failed: ${r.status}`);
    return r.json();
  }

  async createDirective(d: InheritanceDirective): Promise<{ accepted: boolean; receipt: string }> {
    const r = await fetch(`${this.baseUrl}/directives`, {
      method: "POST",
      headers: this.headers(),
      body: JSON.stringify(d),
    });
    if (!r.ok) throw new Error(`create directive failed: ${r.status}`);
    return r.json();
  }
}

export const CROSSWALK = [
  "ISO/IEC 17065:2012",
  "ISO/IEC 27001:2022",
  "IETF RFC 9457",
  "IETF RFC 7519",
  "W3C PROV-DM",
] as const;
