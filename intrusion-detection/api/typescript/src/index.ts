// @wia-official/intrusion-detection — reference SDK skeleton (TypeScript)
//
// This skeleton demonstrates the WIA Intrusion Detection standard's
// request/response contract documented under ../../spec/PHASE-2-API.md.
// It is informative reference material; production deployments may diverge
// as long as they preserve the PHASE contract.

export type WiaTier = "Tier 1 — Self-declared" | "Tier 2 — Third-party assessed" | "Tier 3 — Accredited";

export type AlertSeverity = "informational" | "low" | "medium" | "high" | "critical";

export interface DetectionAlert {
  id: string;
  detected_at: string;
  source: string;
  alert_type: "signature" | "anomaly" | "behavioral" | "ml-classification";
  severity: AlertSeverity;
  ioc_pointers?: string[];
  evidence: string[];
}

export interface ResponsePlaybook {
  id: string;
  name: string;
  triggered_by: string[];
  steps: Array<{ step: string; owner: string; rfc9457_error_type?: string }>;
}

export interface ConformanceRecord {
  envelope_version: "wia-intrusion-detection/1";
  declared_tier: WiaTier;
  alerts: DetectionAlert[];
  playbooks: ResponsePlaybook[];
  crosswalk: ReadonlyArray<string>;
}

export class IntrusionDetectionClient {
  constructor(private readonly baseUrl: string, private readonly token?: string) {}

  private headers(): Record<string, string> {
    const h: Record<string, string> = { "Content-Type": "application/json" };
    if (this.token) h["Authorization"] = `Bearer ${this.token}`;
    return h;
  }

  async listAlerts(cursor?: string): Promise<{ alerts: DetectionAlert[]; next?: string }> {
    const url = cursor ? `${this.baseUrl}/alerts?cursor=${encodeURIComponent(cursor)}` : `${this.baseUrl}/alerts`;
    const r = await fetch(url, { headers: this.headers() });
    if (!r.ok) throw new Error(`list failed: ${r.status}`);
    return r.json();
  }

  async submitAlert(a: DetectionAlert): Promise<{ accepted: boolean; receipt: string }> {
    const r = await fetch(`${this.baseUrl}/alerts`, {
      method: "POST",
      headers: this.headers(),
      body: JSON.stringify(a),
    });
    if (!r.ok) throw new Error(`submit failed: ${r.status}`);
    return r.json();
  }
}

export const CROSSWALK = [
  "ISO/IEC 27001:2022",
  "ISO/IEC 27035-1:2023",
  "NIST SP 800-94",
  "IETF RFC 9457",
  "IETF RFC 7519",
  "W3C PROV-DM",
] as const;
