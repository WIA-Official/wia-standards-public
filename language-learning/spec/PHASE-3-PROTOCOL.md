# WIA-language-learning PHASE 3 — Protocol Specification

**Standard:** WIA-language-learning
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used by
WIA-language-learning participants for discovery,
xAPI / Caliper transport, QTI assessment delivery,
LTI launches, remote-proctoring data flow, credential
verification, and revocation distribution.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IEEE 1484.20.1 xAPI 1.0.3 (LRS contract)
- IMS Caliper 1.2 (Sensor API)
- IMS LTI 1.3 / Advantage (Deep Linking, NRPS, AGS)
- IMS QTI 3.0 (Question and Test Interoperability)
- W3C VC 2.0 verification, W3C DID 1.0 resolution

---

## §1 Scope

This PHASE defines the on-the-wire behaviour between
clients and language-learning registries; between
authoring tools and providers; between sovereign
testing platforms and remote-proctoring services; and
between credential issuers and downstream verifiers.

## §2 Discovery

A WIA-language-learning registry serves a discovery
document at:

```
GET /.well-known/wia/language-learning
```

Response (`application/json`):

```json
{
  "registry": "https://ll.wiastandards.com",
  "openapi": "https://ll.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/learners", "/v1/courses",
                      "/v1/sessions", "/v1/assessments",
                      "/v1/scores", "/v1/credentials",
                      "/v1/accommodations", "/v1/registry"],
  "lti": {
    "platform": "https://ll.wiastandards.com/lti",
    "deepLinking": "/lti/dl",
    "nrps": "/lti/nrps", "ags": "/lti/ags"
  },
  "frameworks": ["CEFR", "ACTFL", "ALTE", "ICAO-Doc-9835"],
  "keySet": "https://ll.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421).

## §3 Transport

HTTPS with TLS 1.3 (RFC 8446) and HSTS preload
mandatory. WebSocket signalling for live remote-
proctoring uses WSS over TLS 1.3.

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/json`                    | record bodies, xAPI statements           |
| `application/x-imscaliper+json`       | Caliper envelopes                        |
| `application/qti+xml`                 | QTI 3.0 tests                            |
| `application/vc+jwt`                  | Verifiable Credentials (JWT proof)       |
| `application/vc+ld+json`              | Verifiable Credentials (Data Integrity)  |
| `application/openbadgecredential+json`| Open Badges 3.0                          |
| `application/problem+json`            | error                                    |

## §5 xAPI transport

xAPI statements are POSTed to
`/v1/sessions/{sessionRef}/xapi` per IEEE 1484.20.1
Section 9 (LRS). The session is the LRS context for
the statements. Statement IDs (UUIDs) are
deterministic where possible to support deduplication.

## §6 Caliper transport

Caliper events are POSTed as Sensor envelopes per IMS
Caliper 1.2. Each envelope carries `sensor`,
`sendTime`, `dataVersion`, and a `data[]` array of
event objects.

## §7 LTI 1.3 launch

LTI 1.3 launches use the OpenID Connect login
initiation flow. `id_token` is signed with the
platform's signing key from the JWKS. Tool services
(NRPS, AGS, Deep Linking 2.0) use OAuth 2.0
client-credentials with JWT-bearer grants.

## §8 QTI 3.0 delivery

QTI tests are delivered as a signed package containing
`assessment-test.xml`, `assessment-section.xml`,
`assessment-item.xml`, and `manifest.xml`. The package
is signed with detached JWS over the canonical archive.

## §9 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `learnerRef`       | UUID (RFC 4122)                                 |
| `courseRef`        | URI                                             |
| `sessionRef`       | UUID                                            |
| `assessmentRef`    | UUID                                            |
| `scoreRef`         | UUID                                            |
| `credentialRef`    | URI per W3C VC 2.0                              |
| `issuerRef`        | DID per W3C DID 1.0                             |

## §10 Caching and immutability

Score and credential records are immutable once
published. They carry `Cache-Control: public,
max-age=31536000, immutable`. Course catalogue
records are mutable and carry strong `ETag` plus
short cache TTLs.

## §11 Revocation distribution

Credential revocations are distributed via the W3C VC
2.0 BitstringStatusList profile:

```
GET /v1/credentials/revocation/{listId}
```

Verifiers fetch and update the list on a cadence the
issuer declares (default 1h). Stale list responses are
rejected by verifiers.

## §12 Replay and unpublishing

Score records are immutable; corrections create a new
`scoreRef` linked to the original via the re-rate
operation. Replay attacks on signed score submissions
are prevented by `iat`/`exp` JWS claims (max 24h on
issuance).

## §13 Remote proctoring data flow

Remote-proctored assessments emit a parallel evidence
stream:

- candidate webcam (encrypted at rest, retention per
  policy);
- screen capture (sampled);
- keyboard / mouse events (statistical only);
- environment-scan video at session start.

The evidence stream is bound to the assessment record
via `proctoringRef`. Access is gated by ISO/IEC 23988
roles: candidate, proctor, supervisor, auditor.

## §14 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 304  | conditional GET unchanged                            |
| 400  | malformed BCP 47 tag, framework code                 |
| 401  | missing or invalid token                             |
| 403  | role mismatch (e.g. learner viewing other learner)   |
| 410  | revoked credential                                   |
| 422  | QTI / xAPI / Caliper schema violation                |
| 426  | TLS upgrade required                                 |
| 503  | LTI platform unavailable                             |

## §15 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.ll.operation`, `wia.ll.framework`,
`wia.ll.targetTag`, and `wia.ll.level` attributes.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served, scores
  signed.
- **Tier 2 — Verified:** LTI / QTI / xAPI interop
  tested; framework alignment audited.
- **Tier 3 — Anchored:** continuous evidence stream
  per PHASE-4 Annex G.

## Annex B — xAPI statement constraints

xAPI statements MUST carry `actor` referencing the
learner via `account` (homePage =
`https://ll.wiastandards.com`, name = `learnerRef`).
Personal-data fields (mbox, mbox_sha1sum) are
forbidden. `verb`, `object`, `result` follow
IEEE 1484.20.1.

## Annex C — Caliper sensor configuration

Caliper sensors emit envelopes at minute granularity
with batch size up to 50 events. Envelope size is
limited to 1 MB; oversized batches are split.

## Annex D — Discovery document signature

The signature over `/.well-known/wia/language-learning`
covers `@authority`, `@path`, `content-digest`
(RFC 9530), and `content-type`.

## Annex E — Cross-Origin Resource Sharing

Read endpoints serve `Access-Control-Allow-Origin: *`
with `ETag` and `Cache-Control` exposed. Write
endpoints require preflight.

## Annex F — Federation

Federation between national or sovereign-authority
registries follows the discovery contract. Trust pairs
are mutually signed and revocable via tombstone.

## Annex G — Trust anchor rotation

Signing keys rotate on a 24-month rolling schedule
with a 6-month overlap.

## Annex H — Replay protection on assessment submission

QTI response submissions carry a monotonically
increasing sequence number per assessment. Receivers
maintain a sliding window of the last 1024 sequence
numbers.

## Annex I — Hardware-backed candidate identity

For high-stakes assessments, the candidate identity
MAY be bound to a hardware authenticator (FIDO2 with
attestation). Loss of the authenticator triggers an
incident record on the assessment.

## Annex J — Cross-Origin Embedded Scoring

Embedded scoring widgets (e.g. an in-browser pronunciation
practice tool) launched from an LMS use a postMessage
channel keyed to the LTI launch nonce. The host frame
MUST verify the embedded origin and the launch nonce
before forwarding any candidate response.

## Annex K — Time-bounded session tokens

Session tokens for in-progress assessments expire 5
minutes after the assessment's published end time. A
token presented past the expiry returns 410 Gone. This
prevents long-tail proctoring violations from extending
the candidate's effective writing time.

## Annex L — Connection management

Long-lived xAPI / Caliper transmissions use HTTP/2
streams. A single TCP connection MAY serve multiple
assessment sessions only when the LRS contract permits
multiplexing; if the LRS is shared across deployments,
the registry mandates per-deployment TCP isolation.

## Annex M — Replay-resistant credential verification

Credential verification carries a verifier nonce
(challenge) that is included in the Verifiable
Presentation. The holder signs the presentation over
both the credential payload and the nonce so that a
captured presentation cannot be replayed against a
second verifier.

## Annex N — Captioning and listening item delivery

Listening assessment items are delivered as
`audio/opus` or `audio/g722` streams with optional
WebVTT captions for accommodations. Captions are
disabled for the construct-relevant portions of
listening tests; the accommodation policy declares
which sections support caption assistance.

## Annex O — QTI candidate state recovery

Mid-assessment state is checkpointed to the registry
on every item transition. If a candidate's connection
drops, the resume endpoint returns the last
checkpointed state plus the time-bank balance. State
expiry follows the assessment's declared end time.

## Annex P0 — Server-Sent Events for live scoring

Live formative scoring (e.g. pronunciation feedback in
the browser) uses Server-Sent Events from the scoring
service to the candidate. The SSE stream carries
per-token confidence and feedback actions. When the
candidate's connection drops, the SSE stream resumes
from the last `Last-Event-ID` header.

## Annex P — Trust anchor publication cadence

Sovereign authorities publish their JWKS at the URL
declared in their issuer DID document. Cadence is
hourly for authorities issuing high-stakes
credentials and daily otherwise. Verifiers cache
JWKS responses with the `Cache-Control: max-age`
declared by the publisher.

## Annex Q — Recording retention windows

Speaking-skill assessments retain audio under the
deployment's privacy policy. Defaults: 30 days for
formative practice; 6 years for high-stakes scores
where appeal windows or sovereign retention require
it; 90 days for medium-stakes course-internal
assessments. The retention window is recorded with
the score and verified at audit.

弘益人間 (Hongik Ingan) — Benefit All Humanity
