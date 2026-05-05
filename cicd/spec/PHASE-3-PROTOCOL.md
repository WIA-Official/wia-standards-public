# WIA-CICD — PHASE 3: Protocol

> 弘益人間 (Benefit All Humanity)

This phase defines the wire protocols WIA-CICD agents speak to each other and to upstream telemetry collectors, registries, and signing services.

---

## 1. OTLP/JSON over HTTPS (Primary Transport)

WIA-CICD signals (`wia-cicd-signal-v1`) are emitted as OTLP/JSON `LogRecord` envelopes for compatibility with OpenTelemetry collectors.

```
POST /v1/logs HTTP/1.1
Host: otel-collector.example
Content-Type: application/json
Authorization: Bearer <token>

{
  "resourceLogs": [{
    "resource": { "attributes": [
       { "key": "service.name", "value": { "stringValue": "wia-cicd" } },
       { "key": "wia.spec",     "value": { "stringValue": "v1.0.0" } }
    ]},
    "scopeLogs": [{
      "scope": { "name": "wia.cicd" },
      "logRecords": [{
         "timeUnixNano": "1746407911000000000",
         "severityText": "INFO",
         "body":  { "stringValue": "<wia-cicd-signal-v1 JSON>" },
         "attributes": [
           { "key": "wia.signal", "value": { "stringValue": "wia-cicd-signal-v1" } }
         ]
      }]
    }]
  }]
}
```

OTLP/gRPC (`opentelemetry.proto.collector.logs.v1`) is also supported. Implementations MUST send at least one of `TRACES`, `METRICS`, `LOGS`, `PROFILES` (`OTEL_SIGNALS`).

---

## 2. Sigstore Cosign — Keyless Signing Flow

Container images and provenance attestations are signed via Sigstore. Keyless flow:

```
   ┌─────────┐  1.OIDC token   ┌──────────────┐
   │ Builder │ ──────────────► │ Fulcio CA    │
   └─────────┘ ◄────────────── └──────────────┘
        │       2. short-lived cert
        │
        │ 3. cosign sign --identity-token=$TOKEN <image>
        ▼
   ┌─────────┐
   │ Rekor   │  4. signed entry → transparency log
   └─────────┘
```

Conformant pipelines MUST submit signed entries to a public Rekor log (or an internal ledger that exposes equivalent inclusion proofs). Verification:

```
cosign verify <image> \
  --certificate-identity-regexp='^https://github.com/.*' \
  --certificate-oidc-issuer='https://token.actions.githubusercontent.com'
```

---

## 3. Argo CD GitOps Sync Protocol

WIA-CICD continuous delivery is GitOps-aligned. The reconciliation loop:

```
   ┌────────────┐     pull     ┌────────────┐    apply   ┌────────────┐
   │  Git repo  │ ───────────► │  Argo CD   │ ─────────► │  Cluster   │
   └────────────┘              └────────────┘            └────────────┘
       ▲                              │                         │
       │ 4. write-back (image SHA)    │ 3. status               │ 2. observe drift
       └──────────────────────────────┴─────────────────────────┘
```

Sync is `automated` for non-prod, `manual` for prod when `policy=ENFORCE`. Drift detection windows MUST be ≤ 3 minutes.

---

## 4. KrCERT / KISA Incident Reporting (Korea-only Context)

When a pipeline supplies services classified as critical information infrastructure under the Korean Information and Communications Network Act (정보통신망법), security incidents MUST be reported to the Korea Internet & Security Agency (KISA) / KrCERT/CC channels within statutory windows.

Submission envelope (locally produced):

```json
{
  "wia_cicd_signal": "wia-cicd-signal-v1",
  "incident": {
    "ts":          "2026-05-05T00:18:31Z",
    "severity":    "P1",
    "type":        "supply_chain_compromise",
    "scope":       ["pipeline", "registry"],
    "evidence":    [{ "uri": "rekor://entry/<id>" }]
  },
  "korea_context": {
    "act":         "KCN_ACT",
    "categories":  ["CII", "PIPA"],
    "reporter":    "krcert@example.kr"
  }
}
```

The body is forwarded to the appropriate KrCERT mailbox / portal via the operator's existing reporting workflow. WIA-CICD does not transmit personal data without the operator's explicit configuration.

---

## 5. Container Registry Pull/Push (OCI Distribution)

Artifacts are exchanged via the OCI Distribution Spec v1.1 (`/v2/...`). Provenance and SBOM are stored as `referrers` of the image manifest.

```
GET  /v2/<name>/referrers/<digest>?artifactType=application/vnd.in-toto+json
```

Conformant clients MUST resolve and verify referrers before deployment.

---

## 6. Webhook Envelope (CI Notifier)

Generic webhook delivered to chat / paging targets:

```http
POST /hooks/wia-cicd HTTP/1.1
Content-Type: application/json
X-WIA-Signature: sha256=<hmac>

{
  "event":   "pipeline.completed",
  "ts":      "2026-05-05T00:18:31Z",
  "outcome": "PASS",
  "score":   1.0,
  "links": {
    "logs":       "https://ci.example/logs/<id>",
    "provenance": "https://rekor.example/api/v1/log/entries/<id>"
  }
}
```

HMAC computed with SHA-256 over the raw body; the receiver MUST reject deliveries lacking a valid signature.

---

© 2026 WIA · MIT License · 弘益人間
