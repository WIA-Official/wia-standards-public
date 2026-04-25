// @wia-official/identity-management — reference SDK skeleton (TypeScript)

export type WiaTier = "Tier 1 — Self-declared" | "Tier 2 — Third-party assessed" | "Tier 3 — Accredited";

export interface Subject {
  id: string;
  identifier_type: "uri" | "did" | "opaque";
  display_name?: string;
  attributes?: Record<string, string>;
  active: boolean;
}

export interface AccessToken {
  iss: string;
  aud: string;
  sub: string;
  iat: number;
  exp: number;
  scope: string;
  wia_tier: WiaTier;
}

export interface AuditEvent {
  id: string;
  occurred_at: string;
  actor: string;
  action: "authenticate" | "authorize" | "lifecycle.create" | "lifecycle.update" | "lifecycle.deactivate" | "lifecycle.erase";
  outcome: "success" | "failure" | "denied";
  evidence_pointer?: string;
}

export class IdentityManagementClient {
  constructor(private readonly baseUrl: string, private readonly token?: string) {}
  private headers(): Record<string, string> {
    const h: Record<string, string> = { "Content-Type": "application/json" };
    if (this.token) h["Authorization"] = `Bearer ${this.token}`;
    return h;
  }
  async getSubject(id: string): Promise<Subject> {
    const r = await fetch(`${this.baseUrl}/subjects/${encodeURIComponent(id)}`, { headers: this.headers() });
    if (!r.ok) throw new Error(`get failed: ${r.status}`);
    return r.json();
  }
  async listAuditEvents(cursor?: string): Promise<{ events: AuditEvent[]; next?: string }> {
    const url = cursor ? `${this.baseUrl}/audit?cursor=${encodeURIComponent(cursor)}` : `${this.baseUrl}/audit`;
    const r = await fetch(url, { headers: this.headers() });
    if (!r.ok) throw new Error(`list failed: ${r.status}`);
    return r.json();
  }
}

export const CROSSWALK = [
  "ISO/IEC 17065:2012",
  "ISO/IEC 27001:2022",
  "IETF RFC 6749",
  "IETF RFC 7519",
  "IETF RFC 7636",
  "IETF RFC 7644",
  "IETF RFC 9457",
  "W3C DID Core v1.0",
  "W3C VC Data Model v1.1",
  "W3C PROV-DM",
  "OpenID Connect Core 1.0",
] as const;
