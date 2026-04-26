# WIA-SOCIAL — Phase 1: Data Format Specification

**Standard**: WIA-SOCIAL (World Interoperable Accessible Social Network)
**Phase**: 1 of 4 — Data Format
**Version**: 1.0.0
**Status**: Draft
**Philosophy**: 弘益人間 — Connect humanity through social bonds

---

## 1. Scope

Phase 1 defines the on-the-wire and at-rest data shapes that every WIA-SOCIAL
participant MUST be able to read. It covers four object families:

| Family | Purpose |
|--------|---------|
| **Identity Bundle** | A subject's portable identity, signed by the subject |
| **Claim** | A single attribute attached to a subject (display name, network handle, key, etc.) |
| **Federation Receipt** | An evidence object exchanged between two peers when a bundle is accepted |
| **Activity Envelope** | A wrapper for cross-network posts, comments, reactions and revocations |

Out of scope for Phase 1: API endpoints (Phase 2), federation protocol semantics
(Phase 3), bridges to legacy networks (Phase 4).

---

## 2. Encoding Rules

* All payloads are encoded as UTF-8 JSON conforming to IETF RFC 8259.
* Object keys are `snake_case` ASCII. Implementations MUST reject keys outside
  the printable ASCII range.
* Timestamps use IETF RFC 3339 with timezone, in UTC (`Z` suffix). Example:
  `"2025-01-15T10:30:00Z"`.
* Binary fields (signature bytes, public keys) are encoded as base64url
  without padding per IETF RFC 4648 §5.
* Identifiers are URI-shaped per IETF RFC 3986. Subject identifiers SHOULD use
  `acct:` (e.g. `acct:alice@example.org`) per IETF RFC 7565.
* Numeric ranges: integer fields MUST fit in a signed 64-bit value; floats use
  IEEE 754 double precision.

### 2.1 Versioning

Every top-level object MUST carry a `wia_social_version` field. Major versions
are breaking; minor versions are additive. A receiver MUST refuse a major
version it does not implement and SHOULD ignore unknown minor fields.

```json
"wia_social_version": "1.0.0"
```

---

## 3. Identity Bundle

The identity bundle is the central object of WIA-SOCIAL. It binds a subject
to a set of claims and a signing key.

```json
{
  "wia_social_version": "1.0.0",
  "type": "identity_bundle",
  "subject": "acct:alice@example.org",
  "issued_at": "2025-01-15T10:30:00Z",
  "expires_at": "2026-01-15T10:30:00Z",
  "claims": [
    { "type": "display_name", "value": "Alice", "audience": "public" },
    { "type": "avatar_url",   "value": "https://example.org/avatar.png", "audience": "public" },
    { "type": "network",      "value": "instagram", "handle": "@alice", "audience": "public" },
    { "type": "network",      "value": "twitter",   "handle": "@alice", "audience": "public" },
    { "type": "network",      "value": "tiktok",    "handle": "@alice_tt", "audience": "friends" }
  ],
  "key": {
    "alg": "Ed25519",
    "public": "MCowBQYDK2VwAyEA…",
    "fingerprint": "sha256:7d8f2c…"
  },
  "audience": "public",
  "signature": {
    "alg": "Ed25519",
    "value": "AY3o…",
    "covers": ["wia_social_version", "subject", "issued_at", "expires_at", "claims", "key", "audience"]
  }
}
```

### 3.1 Required Fields

| Field | Type | Purpose |
|-------|------|---------|
| `wia_social_version` | string | Spec version |
| `type` | string | MUST be `identity_bundle` |
| `subject` | URI | The identified entity |
| `issued_at` | RFC 3339 timestamp | When the bundle was created |
| `claims` | array | At least one claim |
| `key` | object | Signing key descriptor |
| `signature` | object | Detached signature over `covers` |

### 3.2 Optional Fields

| Field | Type | Default | Purpose |
|-------|------|---------|---------|
| `expires_at` | timestamp | none | If absent, the bundle is valid until revoked |
| `audience` | string | `public` | Default audience for claims that omit theirs |
| `previous_signature` | string | none | Pointer to the prior bundle for rotation |

### 3.3 Subject URIs

Recommended subject schemes:

* `acct:user@host` — fediverse-style account (default)
* `did:wia:…` — DID method "wia" (Phase 4 bridge)
* `https://example.org/users/alice` — HTTPS URL identity

Implementations MUST NOT compare subjects byte-wise across schemes; they
MUST normalize per RFC 3986 §6 before equality checks.

---

## 4. Claims

A claim is the atom of WIA-SOCIAL identity. Phase 1 reserves the following
claim types.

### 4.1 Reserved Claim Types

| `type` | `value` shape | Notes |
|--------|---------------|-------|
| `display_name` | string ≤ 64 chars | Free-form, Unicode allowed |
| `avatar_url` | https URL | MUST resolve to an image MIME type |
| `bio` | string ≤ 512 chars | Plain text or basic Markdown subset |
| `pronouns` | string ≤ 32 chars | Free-form |
| `language` | BCP 47 tag | RFC 5646 language tag |
| `network` | enum | See 4.2 |
| `email` | RFC 5321 mailbox | Subject MUST control the mailbox |
| `phone` | E.164 number | Subject MUST control the number |
| `key` | base64url public key | Auxiliary keys (e.g. encryption) |
| `attestation` | object | Signed third-party claim (Phase 3) |

Implementations MUST ignore claim types they do not recognise rather than
reject the bundle. Future minor versions add types, never remove them.

### 4.2 Network Claim Values

Network claims MUST use one of the registered network identifiers. The
initial registry includes: `instagram`, `tiktok`, `twitter`, `facebook`,
`youtube`, `linkedin`, `github`, `mastodon`, `bluesky`, `wia-social-native`.
Additional values follow the WIA-SOCIAL Registry process (see Phase 4).

### 4.3 Per-Claim Audience

Each claim MAY override the bundle's default `audience`:

| Audience | Meaning |
|----------|---------|
| `public` | Anyone, including unauthenticated readers |
| `friends` | Mutual federation peers only |
| `verified` | Federation peers who have proven control of a verified address |
| `private` | The subject only — useful for self-asserted recovery hints |

Higher restrictions narrow visibility; they never widen the bundle default.

---

## 5. Federation Receipt

When peer **A** accepts a bundle from peer **B**, **A** MUST emit a receipt
that **B** can store as proof of acceptance.

```json
{
  "wia_social_version": "1.0.0",
  "type": "federation_receipt",
  "accepted_subject": "acct:alice@example.org",
  "accepted_signature": "AY3o…",
  "by_peer": "https://example.com/wia-social",
  "accepted_at": "2025-01-15T10:30:05Z",
  "receipt_id": "rcpt_01HXY…",
  "signature": {
    "alg": "Ed25519",
    "value": "ZK0p…"
  }
}
```

Receipts are append-only. A peer MAY publish a `revocation_receipt` later
(see Phase 3 §4) but MUST NOT silently delete past receipts.

---

## 6. Activity Envelope

Cross-network actions (post, comment, reaction, follow, block, revoke) ride
inside an activity envelope.

```json
{
  "wia_social_version": "1.0.0",
  "type": "activity",
  "activity_id": "act_01HXY…",
  "actor": "acct:alice@example.org",
  "verb": "post",
  "created_at": "2025-01-15T10:30:00Z",
  "audience": "public",
  "content": {
    "format": "text/markdown",
    "body": "Hello, federated world."
  },
  "attachments": [
    { "media_type": "image/jpeg", "url": "https://example.org/img/1.jpg", "alt_text": "WIA logo on a wall" }
  ],
  "fanout": [
    { "network": "twitter",   "format": "tweet" },
    { "network": "instagram", "format": "post" }
  ],
  "in_reply_to": null,
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

### 6.1 Verbs

`post`, `comment`, `reaction`, `follow`, `unfollow`, `block`, `unblock`,
`share`, `delete`, `edit`, `revoke`. Phase 3 defines verb semantics.
Phase 1 only fixes their wire shape.

### 6.2 Content Format

`format` MUST be a registered IANA media type. Implementations MUST support at
minimum `text/plain`, `text/markdown` (CommonMark), `text/html` (sanitised).

### 6.3 Attachments

Each attachment MUST include a `media_type` and a resolvable `url`. For
images, `alt_text` is REQUIRED for accessibility (WCAG 2.1 §1.1.1).

---

## 7. Schema Files

Machine-readable JSON Schema files (Draft 2020-12) for all four object
families MUST be served from:

```
https://wiastandards.com/social/schemas/identity-bundle.json
https://wiastandards.com/social/schemas/claim.json
https://wiastandards.com/social/schemas/federation-receipt.json
https://wiastandards.com/social/schemas/activity.json
```

Implementations are RECOMMENDED to bundle a copy of these schemas locally
to avoid runtime network dependency.

---

## 8. Examples

### 8.1 Minimal Identity Bundle

```json
{
  "wia_social_version": "1.0.0",
  "type": "identity_bundle",
  "subject": "acct:bob@example.org",
  "issued_at": "2025-01-15T10:30:00Z",
  "claims": [{ "type": "display_name", "value": "Bob" }],
  "key": { "alg": "Ed25519", "public": "MCowBQYDK2VwAyEA…" },
  "signature": { "alg": "Ed25519", "value": "Yz1p…" }
}
```

### 8.2 Cross-Posted Activity

```json
{
  "wia_social_version": "1.0.0",
  "type": "activity",
  "activity_id": "act_01HX…",
  "actor": "acct:bob@example.org",
  "verb": "post",
  "created_at": "2025-01-15T11:00:00Z",
  "audience": "public",
  "content": { "format": "text/plain", "body": "Coffee." },
  "fanout": [
    { "network": "twitter",   "format": "tweet" },
    { "network": "mastodon",  "format": "post" }
  ],
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

---

## 9. Conformance

A Phase 1 conformant implementation MUST:

1. Accept and emit bundles, claims, receipts and activities matching the
   shapes in §3, §4, §5 and §6.
2. Reject objects that are missing required fields or that fail JSON Schema
   validation.
3. Treat unknown optional fields as non-fatal.
4. Round-trip a bundle byte-identically through encode/decode when no
   normalisation is requested.
5. Provide a `--validate` mode that exits non-zero for invalid input.

The companion CLI (`cli/social.sh`) demonstrates this with the
`validate` and `bundle` subcommands.

---

## 10. References

* IETF RFC 8259 — JSON
* IETF RFC 3339 — Date/Time
* IETF RFC 4648 — base64
* IETF RFC 3986 — URI Generic Syntax
* IETF RFC 7565 — `acct:` URI scheme
* IETF RFC 5646 — BCP 47 language tags
* IETF RFC 8032 — EdDSA / Ed25519
* W3C WCAG 2.1 — Accessibility
* JSON Schema Draft 2020-12

---

## Appendix A — Canonicalisation Algorithm

When a bundle's signature must be re-verified, implementations MUST
serialise the covered fields using the following deterministic algorithm:

1. Sort object keys lexicographically by Unicode code point.
2. Encode strings using JSON's compact form: no insignificant whitespace
   between tokens, `\uXXXX` only for code points outside the printable ASCII
   range, lower-case hex digits.
3. Render integers without leading zeros, no decimal point, no exponent.
4. Render floats using the shortest round-trip representation (`%.17g`)
   that re-parses to the same double.
5. Encode arrays element-by-element preserving authored order.
6. UTF-8 encode the result with no BOM.

The hash input for the signature is the SHA-256 of step 6's bytes prefixed
with the literal ASCII string `WIA-SOCIAL/1.0\n` to give domain
separation from other WIA family signatures.

## Appendix B — Reserved Audience Tokens

Future WIA-SOCIAL minor versions MAY add audience tokens. The following
prefixes are reserved and MUST NOT be used by extensions:

| Prefix | Reserved for |
|--------|--------------|
| `wia.` | Cross-WIA-family scopes |
| `vendor.` | Bridge-vendor specific scopes (Phase 4) |
| `private.` | Subject-only scopes |
| `experiment.` | Non-stable, may change without notice |

Implementations MUST treat unknown audience tokens as more restrictive than
`public` and SHOULD log them for operator review.

## Appendix C — Test Vectors

The reference test-vector bundle is available at:

```
https://wiastandards.com/social/test-vectors/bundle-001.json
```

It contains a minimal identity bundle, a network claim, and an
Ed25519 signature computed against the canonicalisation algorithm in
Appendix A. Implementations MUST round-trip this vector
byte-identically through their encode/decode path; deviation indicates a
canonicalisation defect that will eventually break federation.

弘益人間 — Benefit All Humanity.
