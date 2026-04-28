# WIA-lms PHASE 3 — Protocol Specification

**Standard:** WIA-lms
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used by
WIA-lms participants for discovery, LTI 1.3 launch
flow, OneRoster sync, Caliper / xAPI transport,
QTI delivery, content-package distribution, signed
publication, and credential verification.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 6797 (HSTS)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9421 (HTTP Message Signatures), RFC 9530 (Digest Fields)
- IMS LTI 1.3 / Advantage (Deep Linking 2.0, NRPS 2.0, AGS 2.0)
- IMS Caliper 1.2, IMS QTI 3.0
- IMS OneRoster 1.2 (REST and CSV)
- IEEE 1484.20.1 xAPI 1.0.3
- W3C VC 2.0, W3C BitstringStatusList, W3C SHACL

---

## §1 Scope

This PHASE defines the on-the-wire behaviour between
LMS deployments, LTI tools, SIS systems, content
publishers, credential verifiers, and audit
authorities.

## §2 Discovery

A WIA-lms deployment serves a discovery document at:

```
GET /.well-known/wia/lms
```

Response (`application/json`):

```json
{
  "lms": "https://lms.wiastandards.com",
  "openapi": "https://lms.wiastandards.com/openapi.json",
  "lti": {
    "platform": "https://lms.wiastandards.com/lti",
    "deepLinking": "/lti/dl",
    "nrps": "/lti/nrps",
    "ags": "/lti/ags"
  },
  "oneroster": "https://lms.wiastandards.com/v1/oneroster",
  "caliperEndpoint": "https://lms.wiastandards.com/v1/caliper",
  "keySet": "https://lms.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421).

## §3 Transport

HTTPS with TLS 1.3 and HSTS preload. LTI 1.3
launches use HTTPS with form-post bindings; Caliper
push uses HTTP/2 streams.

## §4 Content negotiation

| Accept                                 | Use                                     |
|----------------------------------------|-----------------------------------------|
| `application/json`                     | record bodies                           |
| `application/vnd.imscc+zip`            | Common Cartridge 1.3                    |
| `application/qti+xml`                  | QTI 3.0 tests                           |
| `application/x-imscaliper+json`        | Caliper envelopes                       |
| `application/vc+jwt`                   | Verifiable Credentials (JWT proof)      |
| `application/vc+ld+json`               | Verifiable Credentials (Data Integrity) |
| `application/vnd.ims.oneroster+csv`    | OneRoster CSV bundle                    |
| `application/openbadgecredential+json` | Open Badges 3.0                         |
| `application/problem+json`             | error                                   |

## §5 LTI 1.3 launch flow

LTI 1.3 launches use OpenID Connect 1.0 login
initiation:

1. Tool initiates login at
   `loginInitiationUrl` with `iss`, `client_id`,
   `target_link_uri`, `lti_message_hint`.
2. Platform returns an authentication response with
   the `id_token` JWT signed by the platform.
3. Tool validates the `id_token` against the
   platform's JWKS.
4. Tool launches the activity at `target_link_uri`.

The `id_token` MUST include `nonce` and `state` to
prevent replay.

## §6 LTI Advantage services

| Service                               | Purpose                                  |
|---------------------------------------|------------------------------------------|
| Deep Linking 2.0                      | Tool returns content selection           |
| NRPS 2.0 (Names and Role Provisioning)| Tool retrieves course roster             |
| AGS 2.0 (Assignment and Grade)        | Tool posts scores back to gradebook      |
| Proctoring Services 1.0               | Tool participates in proctored events    |

Service calls authenticate with OAuth 2.0
client-credentials grants using JWT-bearer
assertions signed by the tool's JWKS.

## §7 OneRoster sync

OneRoster 1.2 REST endpoints follow the IMS
specification (`/users`, `/classes`, `/enrollments`,
`/academicSessions`). The CSV binding is supported
for nightly bulk sync and uses the standard CSV
bundle layout with manifest.

## §8 Caliper / xAPI transport

Caliper Sensor envelopes POST to the LMS Caliper
endpoint with Bearer JWT auth. xAPI statements POST
to the embedded LRS at
`/v1/sessions/{sessionRef}/xapi`.

## §9 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `institutionRef`   | UUID (RFC 4122)                                 |
| `courseRef`        | URI                                             |
| `enrolmentRef`     | UUID                                            |
| `contentRef`       | URI                                             |
| `activityRef`      | URI                                             |
| `assessmentRef`    | UUID                                            |
| `gradebookRef`     | UUID                                            |
| `credentialRef`    | URI per W3C VC 2.0                              |
| `toolRef`          | URI                                             |

## §10 Caching and immutability

Content packages and credentials are immutable;
they carry `Cache-Control: public, max-age=31536000,
immutable`. Course catalogue, enrolment, and
gradebook records are mutable with strong `ETag`.

## §11 Replay and tombstoning

Credential revocations are distributed via the
W3C VC 2.0 BitstringStatusList. Enrolment
withdrawals tombstone the prior enrolment record.
Gradebook entries are append-only; corrections emit
a new entry with `corrects` set to the prior entry.

## §12 Error semantics

Errors are `application/problem+json` (RFC 9457).
Protocol-level codes:

| Code | Meaning                                              |
|------|------------------------------------------------------|
| 200  | success                                              |
| 304  | conditional GET unchanged                            |
| 400  | malformed LTI / OneRoster / QTI                      |
| 401  | missing or invalid token                             |
| 403  | role mismatch                                        |
| 410  | revoked credential, withdrawn enrolment              |
| 422  | schema violation                                     |
| 503  | LTI tool unavailable                                 |

## §13 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.lms.operation`, `wia.lms.courseRef`,
`wia.lms.toolRef`, and `wia.lms.actor`
(pseudonymised) attributes.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served,
  LTI 1.3 platform exposed.
- **Tier 2 — Verified:** LTI Advantage / OneRoster
  / QTI / Caliper interop tested.
- **Tier 3 — Anchored:** continuous evidence stream
  per PHASE-4 Annex G.

## Annex B — LTI 1.3 token validation rules

- `iss` matches the platform's identity URL;
- `aud` matches the tool's `client_id`;
- `nonce` is unique within a 24h window;
- `iat` within 5 minutes of receipt clock;
- `exp` not yet passed;
- signature verified against platform JWKS.

## Annex C — Discovery document signature

The signature over `/.well-known/wia/lms` covers
`@authority`, `@path`, `content-digest` (RFC 9530),
and `content-type`.

## Annex D — Cross-Origin Resource Sharing

LMS read endpoints serve `Access-Control-Allow-
Origin: *` with `ETag`, `Link`, and Caliper-related
headers exposed.

## Annex E — Trust anchor rotation

Platform signing keys rotate on a 24-month rolling
schedule with a 6-month overlap window.

## Annex F — Replay protection on AGS scoring

AGS score posts carry `iat`/`exp` claims (max 1h)
in their JWT bearer. The platform rejects scores
whose `iat` exceeds the freshness window.

## Annex G — Connection management

Caliper sensors use HTTP/2 streams. A sensor
emitting a continuous Caliper envelope stream
SHOULD reuse a single connection over multiple
courses; the LMS terminates idle connections after
60 seconds.

## Annex H — Federation hop cap

Federated NRPS 2.0 queries carry an
`X-WIA-Federation-Hops` header; queries with
`Hops > 3` are dropped.

## Annex I — Cross-Origin LTI launch

Browser launches MAY cross origins; the LMS
reflects launch origins in its `Sec-Fetch-Site`
allowlist for the configured tool's origin only.

## Annex J — TLS profile baseline

TLS 1.3 with PFS-only cipher suites; NIST SP 800-52
Rev. 2 baseline. TLS 1.2 acceptable as fallback for
legacy SCORM tools but only over the
`/v1/legacy/scorm` endpoint.

## Annex K — Common Cartridge ingest pipeline

Common Cartridge 1.3 archives are ingested
asynchronously. The pipeline:

1. Verify the archive signature.
2. Validate the manifest against the IMS schema.
3. Transcode SCORM 2004 4th Edition assets to cmi5
   where present.
4. Materialise courses, content packages, and
   activities into the LMS records.
5. Emit a `course.imported` webhook event.

Failures at any step abort the import and surface
a Problem Details report to the publisher.

## Annex L — AGS line-item idempotency

AGS score posts MUST carry an `Idempotency-Key`
header. The platform persists the key for 24 hours
so that retried posts do not duplicate gradebook
entries.

## Annex M — Caliper firehose backpressure

When the Caliper sensor sustains a rate above the
deployment's quota, the LMS responds with `429 Too
Many Requests` and `Retry-After` per the standard
back-pressure pattern. Sensors back off and resume
on the indicated time.

## Annex N — LTI tool deep-linking response signing

Deep Linking 2.0 response payloads are signed by
the tool with `state` and `nonce` matching the
launch. The platform verifies the signature
against the tool's JWKS before accepting selected
content into the course.

## Annex O — iCalendar feed signing

iCalendar feeds carry an HTTP Message Signature
(RFC 9421) covering `@authority`, `@path`,
`content-digest`, and `content-type`. Calendar
clients that consume the signed feed verify the
signature against the deployment's JWKS before
trusting the events.

## Annex P — Discussion thread anti-spam

Discussion threads accept posts only from enrolled
learners or course staff. Anonymous posts are
disabled by default; the institution may enable
pseudonymous posting where the underlying enrolment
is preserved server-side. Rate-limit headers apply
to posting.

## Annex Q — Background-job semantics

Long-running jobs (cartridge import, gradebook
recalculation, bulk credential issuance) accept a
202 Accepted with a polling URL and a webhook
subscription option. Polling responses include a
progress percentage and an estimated completion
timestamp.

## Annex R — Notification delivery channels

Notifications use three delivery channels: in-LMS
inbox (always), email (per recipient subscription),
push (per recipient device registration). The
deployment's notification policy declares which
channels are mandatory for which severities.

Each channel attempt is recorded so that delivery
SLAs can be audited; delivery to an opted-out
channel is suppressed and recorded as
`delivery.suppressed`.

## Annex S — Course-copy idempotency

Course-copy jobs accept an `Idempotency-Key`
header. The platform persists the key for 24 hours
so that retried copies do not duplicate the target
course. The key is namespaced per source-target
pair.

## Annex T — Sec-Fetch and CSP coordination

LMS pages set Content-Security-Policy directives
that allow only the configured LTI tool origins for
`frame-src` and `connect-src`. Browser
`Sec-Fetch-Site` headers verify that cross-origin
requests originate from the registered tool's
origin. Mismatches surface as `csp.violation` and
`sec-fetch.mismatch` events in the audit feed.

## Annex U — Service worker boundary

Service workers MAY cache static course assets but
MUST NOT cache gradebook, AGS, or NRPS responses.
The LMS sets `Cache-Control: no-store, private` on
mutating-data endpoints to enforce the boundary.

弘益人間 (Hongik Ingan) — Benefit All Humanity
