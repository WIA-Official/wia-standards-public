# WIA-language-bridge PHASE 3 — Protocol Specification

**Standard:** WIA-language-bridge
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the wire-level protocols used by
WIA-language-bridge participants for content negotiation,
real-time interpretation streaming, sign-language video
relay, segment signing, and conformance evidence
transport. The protocol is layered over HTTP and WebRTC
so that existing CDN, real-time-communication, and
captioning infrastructure carries the workload without
modification.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP Semantics), RFC 9112 (HTTP/1.1), RFC 9114 (HTTP/3)
- IETF RFC 8446 (TLS 1.3), RFC 9001 (QUIC)
- IETF RFC 8615 (Well-Known URIs), RFC 7517 (JWK), RFC 7515 (JWS)
- IETF RFC 9110 §12 (Content Negotiation)
- IETF RFC 8259 (JSON), RFC 8785 (JCS)
- W3C WebRTC 1.0, W3C MediaStream Recording, W3C WebVTT 1.0
- ITU-T H.264, H.265, H.266; ITU-T G.711, G.722, Opus (RFC 6716)
- W3C ITS 2.0
- ISO 18841 (rotation, fatigue rules)

---

## §1 Scope

This PHASE defines the on-the-wire behaviour between
clients and language-bridge registries; between
authoring tools and translation services; between
conference-platform clients and remote simultaneous
interpreters; and between sign-language relay services
and downstream captioning consumers.

## §2 Discovery

A WIA-language-bridge registry serves a discovery
document at:

```
GET /.well-known/wia/language-bridge
```

Response (`application/json`):

```json
{
  "registry": "https://lb.wiastandards.com",
  "openapi": "https://lb.wiastandards.com/openapi.json",
  "operationGroups": ["/v1/segments", "/v1/jobs",
                      "/v1/practitioners", "/v1/tm",
                      "/v1/glossaries", "/v1/quality",
                      "/v1/sessions", "/v1/registry"],
  "rsi": {
    "signalling": "wss://rsi.wiastandards.com",
    "iceServers": ["stun:stun.wiastandards.com:3478"],
    "supportedCodecs": ["opus", "g722", "g711", "h264", "vp9", "av1"]
  },
  "keySet": "https://lb.wiastandards.com/.well-known/jwks.json"
}
```

The discovery document is signed (RFC 9421) using the
registry's signing key.

## §3 Transport

HTTP traffic uses HTTPS with TLS 1.3 (RFC 8446) and HSTS
preload. RSI traffic uses WebRTC over DTLS-SRTP. Audio
is Opus (RFC 6716) by default; video is VP9 or AV1.

## §4 Content negotiation

| Accept                                | Use                                      |
|---------------------------------------|------------------------------------------|
| `application/json`                    | record bodies                            |
| `application/x-xliff+xml`             | XLIFF 2.1 documents                      |
| `application/x-tmx+xml`               | TMX 1.4b                                 |
| `application/x-tbx+xml`               | TBX-Basic                                |
| `text/vtt`                            | WebVTT cues                              |
| `application/x-subrip`                | SRT subtitles                            |
| `audio/opus`, `audio/g722`            | RSI audio                                |
| `video/H264`, `video/AV1`             | sign-language relay                      |
| `application/problem+json`            | error                                    |

Content negotiation also takes the requested target
language tag from the `Accept-Language` header per
RFC 4647 lookup matching.

## §5 Real-time simultaneous interpretation

### 5.1 Signalling

Clients establish a signalling channel over a secure
WebSocket to the registry's `/rsi` endpoint. The
exchange follows ITU-T F.745 conversational service
patterns.

### 5.2 Roles

- `floor`     — original speaker; the source-language stream
- `relay`     — pivot interpreter; consumed by §5.3 outputs
- `output[]`  — interpreters delivering each target language
- `listener`  — end users; subscribe to one or more outputs

### 5.3 Rotation

ISO 18841 partner-rotation: a single interpreter MUST
NOT serve simultaneous mode for longer than 30 minutes.
The signalling channel emits `rotation-due` events 60s
before the deadline; failure to rotate triggers a
recorded conformance violation in the session record.

### 5.4 Sign-language overlay

Sign-language relay subscribers receive a video track
keyed to the chosen ISO 639-3 sign code. Frame rate is
≥25 fps; bitrate floor is 1.5 Mbps for legibility.

## §6 Segment signing

Segments are signed with detached JWS (RFC 7515) over
the canonical JSON form (RFC 8785). The `kid` references
the practitioner key in the registry's JWKS. A segment
without a valid signature is treated as `claimed` and
SHOULD NOT be used in audit-grade pipelines.

## §7 Identifiers

| Identifier         | Format                                          |
|--------------------|-------------------------------------------------|
| `segmentRef`       | UUID (RFC 4122)                                 |
| `jobRef`           | UUID                                            |
| `practitionerRef`  | UUID; the registry-internal mapping to sovereign |
|                    | certificates is auditable                       |
| `sessionRef`       | UUID                                            |
| BCP 47 tag         | RFC 5646 canonical                              |
| `tmxRef`, `tbxRef` | URI                                             |

## §8 Caching

Glossary and TM documents are immutable once registered;
they carry `Cache-Control: public, max-age=31536000,
immutable`. Practitioner, job, and session records are
mutable and carry strong `ETag` plus
`Cache-Control: public, max-age=60,
stale-while-revalidate=30`.

## §9 Federation

Federated registries form a directed graph in the
discovery document. Cross-registry queries follow the
graph and carry an `X-WIA-Federation-Path` header. Trust
between peers is anchored in the JWKS sets exposed at
`/.well-known/jwks.json`.

## §10 Replay and unpublishing

Registered segments and TM/TBX documents are immutable.
Errata create a new version; the prior version is
tombstoned with `unpublishedAt`. Replay attacks on
signed jobs are prevented by `iat`/`exp` JWS claims
(max 24h on job creation, max 1h on session signalling).

## §11 Error semantics

Errors are `application/problem+json` (RFC 9457). The
PHASE-2 §11 catalogue applies. Protocol-level codes:

| Code  | Meaning                                              |
|-------|------------------------------------------------------|
| 200   | success                                              |
| 304   | conditional GET unchanged                            |
| 400   | malformed BCP 47 tag                                 |
| 401   | missing or invalid token                             |
| 403   | practitioner pair not authorised for the job         |
| 410   | tombstone                                            |
| 422   | XLIFF / TMX / TBX schema violation                   |
| 426   | TLS upgrade required                                 |
| 503   | RSI signalling unavailable                           |

## §12 Observability

Servers SHOULD emit OpenTelemetry traces with
`wia.lb.operation`, `wia.lb.sourceTag`,
`wia.lb.targetTag`, and `wia.lb.practitioner`
attributes. RSI rooms emit a `wia.lb.session` span
spanning the entire interpretation session.

## Annex A — Conformance levels

- **Tier 1 — Self-declared:** discovery served, segments
  signed.
- **Tier 2 — Verified:** federation interop tested;
  rotation rule enforced; ISO 17100 process audit
  passed.
- **Tier 3 — Anchored:** continuous evidence stream
  captured per PHASE-4 Annex G.

## Annex B — RSI protocol stack

```
SIP/WebRTC  ←  signalling (control)
DTLS-SRTP   ←  media
Opus / AV1  ←  codec
```

## Annex C — Captioning hand-off

A real-time interpretation session MAY emit captions on
a parallel WebVTT stream. Cues are timestamped against
the floor speaker's media timeline so that downstream
captioning consumers can align audio, video, and text.

## Annex D — Sign-language frame budget

| Aspect       | Floor                | Recommended            |
|--------------|----------------------|------------------------|
| Resolution   | 720p                 | 1080p                  |
| Frame rate   | 25 fps               | 50 fps for high-speed  |
|              |                      | finger-spelling        |
| Bitrate      | ≥1.5 Mbps            | 3–5 Mbps               |
| Latency end-to-end | ≤500ms         | ≤300ms                 |

## Annex E — Discovery document signature

The signature over `/.well-known/wia/language-bridge`
covers `@authority`, `@path`, `content-digest`
(RFC 9530), and `content-type`. Verifiers reject
signatures whose covered components do not include all
four.

## Annex F — Session recording retention

Recording retention windows depend on the setting:

- Legal interpretation: per court rules, often 7 years.
- Medical interpretation: 6 years (HIPAA-equivalent).
- Conference: 30 days unless extended.
- Asylum / consular: per sovereign asylum directive.

The retention window is declared at session open and
written into the session record. Auditors verify
deletion against the declared window.

## Annex G — Codec fallback

When a peer cannot negotiate Opus or G.722, the session
falls back to G.711. The fallback MUST be logged with a
`codec-fallback` event in the session record.

## Annex H — Cross-registry trust

Two registries form a trust pair by mutually signing a
peer assertion published at `/.well-known/wia/peer/{id}`.
The assertion declares the trusted operation groups,
the trust expiry, and the JWKS URL of the peer. Trust
assertions are revoked by tombstoning the assertion;
clients re-resolve trust on every cache TTL boundary.

## Annex I — Caption timing

Captions emitted by simultaneous interpretation MUST be
timed against the floor speaker's RTP wallclock per
RFC 7273. Drift exceeding 500 ms within a 30-second
window triggers a recorded conformance event.

## Annex J — Multimodal channel binding

Audio, video, captioning, and chat are bound by a
common `sessionRef`. The binding is authoritative: a
caption emitted in a session that does not match the
session's declared `targetTag` is rejected by the
captioning relay.

## Annex K — Quality-of-Service classes

| Class       | Loss   | Latency  | Jitter  | Use                  |
|-------------|--------|----------|---------|----------------------|
| `legal`     | <0.1%  | <150 ms  | <30 ms  | court interpretation |
| `medical`   | <0.5%  | <250 ms  | <50 ms  | bedside / triage     |
| `conference`| <1.0%  | <400 ms  | <80 ms  | event interpretation |
| `community` | <2.0%  | <500 ms  | <120 ms | community sessions   |

QoS class is declared at session open and is recorded
in the session record. Auditors verify per-session
metrics against the declared class.

## Annex L — Replay protection on RSI signalling

WebSocket signalling messages carry a monotonically
increasing sequence number per session. Receivers
maintain a sliding window of the last 1024 sequence
numbers and reject messages whose number is below the
window floor. This is in addition to the WebSocket
masking and TLS protection.

## Annex M — Practitioner key proof-of-possession

Practitioners may bind a hardware-backed key (e.g.
FIDO2/WebAuthn `att=direct` attestation, smart card)
to their record. When a hardware-backed key is bound,
the registry accepts only signatures whose `cnf` claim
references the key's attestation. Loss of the hardware
key is reported via `keyRevocation` and a fresh
practitioner record amendment.

弘益人間 (Hongik Ingan) — Benefit All Humanity
