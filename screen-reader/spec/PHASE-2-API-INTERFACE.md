# WIA Screen Reader — Phase 2: API Interface

**Standard**: WIA Screen Reader
**Phase**: 2 of 4 — API Interface
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 2 specifies the local IPC and HTTP surfaces that a host (NVDA
addon, VoiceOver enabler, TalkBack module, browser extension) exposes
for the data families defined in Phase 1.

Out of scope: federation between vendors (Phase 3), platform bridges
(Phase 4).

---

## 2. Transports

WIA Screen Reader runs in three deployment modes; each defines its own
transport binding for the same JSON envelopes.

| Mode | Transport | Audience |
|------|-----------|----------|
| **Local addon** | Unix domain socket / Windows named pipe | NVDA, VoiceOver, Orca |
| **Browser extension** | Chrome / WebExtensions message-passing | Chrome / Firefox / Edge |
| **Cloud relay** | HTTPS over TLS 1.3 (IETF RFC 8446) | Mobile, federated dashboards |

The cloud-relay binding is what this document mainly specifies. Local
and browser-extension bindings carry the same envelope shapes through
their respective IPC channels.

---

## 3. Discovery

```
GET https://<host>/.well-known/wia-screen-reader

{
  "wia_screen_reader_version": "1.0.0",
  "host_id": "did:wia:reader-host:nvda-cloud",
  "endpoints": {
    "pronunciation": "https://reader.example/sr/pronunciation",
    "tree":          "https://reader.example/sr/tree",
    "reading-order": "https://reader.example/sr/reading-order",
    "braille":       "https://reader.example/sr/braille",
    "profile":       "https://reader.example/sr/profile",
    "telemetry":     "https://reader.example/sr/telemetry"
  },
  "supported_signatures": ["Ed25519"],
  "supported_languages": ["ko-KR", "en-US", "ja-JP", "zh-CN", "zh-Hant-HK", "..."],
  "supported_braille_grades": ["g1", "g2", "wia"],
  "rate_limits": {
    "anonymous":     { "rps": 1,  "burst": 5 },
    "authenticated": { "rps": 50, "burst": 200 }
  }
}
```

Discovery responses are cacheable for 300 s.

---

## 4. Pronunciation Endpoints

```
POST /sr/pronunciation              — submit a pronunciation hint (signed)
GET  /sr/pronunciation/{hint_id}
GET  /sr/pronunciation?language={tag}&token={url-encoded}
```

`GET` with a `(language, token)` pair returns the host's authoritative
hint (if any), allowing readers to defer to a domain-curated
pronunciation rather than guessing from rules.

---

## 5. Accessibility Tree Endpoints

```
POST /sr/tree                       — bulk submit a page's tree (signed)
GET  /sr/tree/{tree_id}
GET  /sr/tree?page_url_hash={sha256}
```

Trees are addressed by a hash of the page URL plus a salt so that the
host does not log raw URLs. The hash is computed as
`sha256("wia-sr|" + page_url)`.

---

## 6. Reading Order Endpoints

```
POST /sr/reading-order
GET  /sr/reading-order/{plan_id}
```

Readers POST a plan when the user has manually re-ordered the page
(skip-region edits, landmark preferences). The host stores the plan
under the user's profile and returns it on subsequent visits.

---

## 7. Braille Endpoints

```
POST /sr/braille                    — translate text to braille
GET  /sr/braille/{mapping_id}
```

The POST body carries `source_text`, `source_language`, and `grade`.
The host returns a `braille_mapping` envelope. Translation is
deterministic; identical inputs MUST yield identical outputs (so the
endpoint is idempotent without an explicit Idempotency-Key).

---

## 8. Profile Endpoints

```
PUT  /sr/profile/{user_id}
GET  /sr/profile/{user_id}
```

Profile writes require HTTP Message Signatures per IETF RFC 9421
covering method, target URI, `Date`, `Content-Digest`,
`Content-Length`. Reads are restricted to the user themselves and to
authorised platform extensions on the user's enrolled device list.

---

## 9. Telemetry Endpoint

```
POST /sr/telemetry
```

Submissions MUST be unauthenticated; the standard treats reader
telemetry as anonymous engagement data. The host MUST refuse
submissions whose payload contains any field outside the §8 enum or
that includes a user identifier.

---

## 10. Authentication and Errors

* Writes (pronunciation, tree, reading-order, profile) require
  HTTP Message Signatures per IETF RFC 9421.
* Reads MAY be anonymous subject to the discovery rate limit.
* Errors follow IETF RFC 9457 problem details. Reserved error types
  live under `https://wiastandards.com/screen-reader/errors/`.

```json
{
  "type": "https://wiastandards.com/screen-reader/errors/unsupported-language",
  "title": "Language tag not supported",
  "status": 400,
  "detail": "language=xx-YY not in this host's supported_languages",
  "instance": "/sr/pronunciation"
}
```

---

## 11. Conformance

A Phase 2 conformant host MUST:

1. Publish `/.well-known/wia-screen-reader` with all endpoints.
2. Implement pronunciation / tree / reading-order / braille / profile /
   telemetry routes with the documented status codes.
3. Refuse telemetry submissions with non-enum action tokens.
4. Emit problem-detail JSON for 4xx/5xx responses.
5. Honour the user's profile verbosity setting in every response that
   contains an accessibility tree node.

---

## 12. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 9457 — Problem Details for HTTP APIs
* W3C WebExtensions
* W3C WAI-ARIA 1.2
* JSON Schema Draft 2020-12

---

## Appendix A — IPC Binding for Local Addons

For the local-addon mode (NVDA / VoiceOver / Orca), the JSON envelopes
ride over a Unix domain socket (Linux/macOS) or Windows named pipe.
Frame format:

```
[4 bytes BE length][JSON payload]
```

Frames carry the same envelope shape as the HTTP body. Authentication
is by the OS-level pipe permissions — the addon and the WIA daemon
share a per-user pipe.

## Appendix B — Browser Extension Binding

WebExtensions message-passing carries the same envelopes through
`browser.runtime.sendMessage`. The extension's manifest MUST declare
the WIA permission scope `wia-screen-reader` for cross-origin
pronunciation and braille requests.

## Appendix C — Reference Request / Response Pairs

### C.1 Submit a pronunciation hint

```http
POST /sr/pronunciation HTTP/1.1
Host: reader.example
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_screen_reader_version":"1.0.0",
  "type":"pronunciation_hint",
  "language":"ko-KR",
  "token":"WIA",
  "ipa":"ˈwiː.ɑ",
  "wihp":"위아",
  "stress_pattern":"PRIMARY",
  "issued_at":"2026-04-27T10:00:00Z",
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 201 Created
Location: /sr/pronunciation/ph_01HXY
RateLimit-Limit: 50
RateLimit-Remaining: 49
```

### C.2 Read a pronunciation hint by language and token

```http
GET /sr/pronunciation?language=ko-KR&token=WIA HTTP/1.1
Host: reader.example
Accept: application/json

HTTP/1.1 200 OK
Content-Type: application/json

{ "wia_screen_reader_version":"1.0.0",
  "type":"pronunciation_hint",
  "language":"ko-KR",
  "token":"WIA",
  "wihp":"위아",
  "ipa":"ˈwiː.ɑ" }
```

### C.3 Move a profile

```http
PUT /sr/profile/did%3Awia%3Areader%3A01HXY HTTP/1.1
Host: voiceover-enabler.example
Content-Type: application/json
Authorization: WIA-Sig keyid="…",signature="…"

{ "wia_screen_reader_version":"1.0.0",
  "type":"profile_move",
  "user_id":"did:wia:reader:01HXY",
  "from_host":"did:wia:reader-host:nvda-cloud",
  "to_host":"did:wia:reader-host:voiceover-enabler",
  "moved_at":"2026-04-27T10:00:00Z",
  "signature":{ "alg":"Ed25519", "value":"…" } }

HTTP/1.1 200 OK
```

## Appendix D — Conformance Test Suite

A black-box test suite is published at
`https://github.com/WIA-Official/wia-screen-reader-conformance` and
walks through:

1. Discovery document round-trip.
2. Pronunciation hint publish, fetch, conflict-resolution chain.
3. Accessibility tree submit + page-URL-hash retrieval.
4. Reading-order plan create / replace.
5. Braille translation idempotency for `g1`, `g2`, `wia` grades.
6. Profile read / write / move across hosts.
7. Telemetry consent enforcement: refused without consent, accepted
   with valid consent envelope, rejected for fields outside the §8
   enum.
8. Authentication failure paths returning RFC 9457 problem details.
9. Rate-limit headers and exhaustion behaviour.

Hosts publishing `bridge_profile=Full` SHOULD additionally pass the
suite's WIHP and WIA Braille extension tests.

## Appendix E — Operational Recommendations

* Hosts MUST persist incoming hint and profile writes durably before
  acknowledging; an at-most-once acknowledgement followed by storage
  loss would silently drop a curator's contribution.
* Hosts SHOULD provide a separate read-replica endpoint for high-volume
  hint-fetch traffic so writes from curators stay responsive.
* Hosts SHOULD expose a `/sr/health` liveness probe outside the
  public rate-limit accounting.
* Hosts SHOULD strip IP addresses from telemetry payloads at ingestion
  to honour the anonymous-by-design contract.

## Appendix F — Operational Telemetry

A conformant host SHOULD expose Prometheus-style metrics on a separate
port:

| Metric | Type | Labels |
|--------|------|--------|
| `wia_sr_pronunciation_publishes_total` | counter | `language` |
| `wia_sr_pronunciation_fetches_total` | counter | `language`, `cache_outcome` |
| `wia_sr_braille_translations_total` | counter | `grade`, `source_language` |
| `wia_sr_profile_moves_total` | counter | `outcome` |
| `wia_sr_telemetry_consent_violations_total` | counter | none |
| `wia_sr_request_duration_seconds` | histogram | `route`, `status` |
| `wia_sr_seen_nonce_cache_size` | gauge | none |

Telemetry MUST NOT include user identifiers, page URLs, or page
content as label values.

## Appendix G — Backwards-Compatibility Promise

Within the 1.x line every endpoint listed in this document MUST remain
reachable and MUST continue to honour the documented status codes and
content shapes. Hosts MAY add optional query parameters, response
fields, new endpoints, or media types. Hosts MUST NOT remove or
repurpose existing ones. Breaking changes ride a major version bump
and MUST be preceded by a 12-month deprecation window per IETF
RFC 8594 and RFC 9745; deprecated routes MUST emit `Deprecation` and
`Sunset` headers throughout the window.

## Appendix H — CORS and Cross-Origin Hint Fetch

Browser extensions that fetch hints across origins (e.g. extension
running on example.com fetches Korean medical hints from
`reader.example`) need CORS pre-flight to succeed. Hint hosts MUST
respond to `OPTIONS /sr/pronunciation` with:

```
Access-Control-Allow-Origin: <extension origin>
Access-Control-Allow-Methods: GET, POST, OPTIONS
Access-Control-Allow-Headers: Content-Type, Authorization, Idempotency-Key
Access-Control-Max-Age: 600
```

For anonymous hint fetches the response MAY use
`Access-Control-Allow-Origin: *` since no credentials are involved.
For authenticated fetches the host MUST validate the request `Origin`
against an allow-list configured by the host operator; the allow-list
MUST be explicit (no wildcards for credentialed requests).

## Appendix I — Health and Readiness Probes

A conformant host SHOULD expose three liveness signals on a separate
port (default 9091, distinct from the public service port):

| Probe | Path | Payload |
|-------|------|---------|
| Liveness  | `/healthz`  | `{"status":"ok"}` |
| Readiness | `/readyz`   | `{"deps":{"hint_store":"ok","profile_store":"ok","queue":"ok"}}` |
| Startup   | `/startupz` | `{"phase":"warming","ready_at":"2026-…"}` |

These MUST NOT be advertised in `/.well-known/wia-screen-reader` and
MUST NOT be subject to public rate limits. Internal observability
tooling consumes them; nothing external should rely on their presence.

The readiness endpoint's `deps` map MUST include every external
dependency that, if down, would cause the host to fail to serve
authenticated writes. For a typical host this includes the hint
store backing database, the profile store, the message queue used
for telemetry ingestion, and the federation receipt store.

弘益人間 — Benefit All Humanity.
