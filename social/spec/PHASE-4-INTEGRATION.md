# WIA-SOCIAL — Phase 4: Integration Specification

**Standard**: WIA-SOCIAL
**Phase**: 4 of 4 — Integration with Existing Networks
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 4 specifies how WIA-SOCIAL integrates with three classes of existing
systems:

1. **Legacy social networks** (Instagram, Twitter, TikTok, YouTube, …) via
   bridges that translate WIA-SOCIAL ↔ vendor APIs.
2. **Open federation protocols** (ActivityPub, AT Protocol, Nostr) via
   adapter shims.
3. **WIA family standards** (WIA-OMNI-API for credentials, WIA-AIR-SHIELD
   for transport, WIA-INTENT for action verbs) for shared infrastructure.

The aim is to let a subject sign in once with a WIA-SOCIAL identity and
reach every network they care about, without forcing other operators to
adopt WIA-SOCIAL natively.

---

## 2. Bridge Architecture

A *bridge* is an HTTP service that:

* Accepts WIA-SOCIAL activities on its inbound endpoint (Phase 2 §5.1).
* Translates each activity into the destination network's native API call.
* Maintains per-subject credentials needed by the destination network
  (OAuth tokens, app passwords, etc.).
* Reports back delivery outcome via a `delivery_receipt` activity.

```
   Subject's origin host
            │
            ▼
   ┌────────────────────┐
   │  WIA-SOCIAL Bridge │── OAuth ──▶  Network vendor API
   │  (per-network)     │            (Twitter, Instagram, …)
   └────────────────────┘
            ▲
            │  delivery_receipt
            │
   Subject's origin host
```

Bridges MUST be discoverable via the host's `/.well-known/wia-social`
document. Bridges MUST NOT see the subject's signing key — they receive
already-signed activities and only need credentials for the destination
network.

---

## 3. Per-Network Bridge Profiles

For each supported network, the bridge implements:

| Operation | Mapping |
|-----------|---------|
| `post` (text ≤ 280) | Vendor "create post" with text body |
| `post` (text > 280) | Vendor thread / long-form variant if available, else truncate with link to canonical activity |
| `comment` | Vendor reply to mapped parent id |
| `reaction` | Vendor like/heart equivalent |
| `share` | Vendor repost / quote-tweet |
| `delete` | Vendor delete; if vendor refuses, bridge marks `delivery_receipt.status = unsupported` |
| `edit` | Vendor edit if supported, else delete + repost |

Phase 4 does not enumerate every vendor's quirks. The reference profile
documents (`/spec/profiles/<network>.md`) describe the per-vendor mapping.

### 3.1 Quote-Tweet & Boost Semantics

`share` translates to:

* Twitter / X: native retweet or quote-tweet depending on whether the
  activity carries `comment` content.
* Mastodon: native boost or quote (for Mastodon ≥ 4.4 quote support).
* Bluesky: AT Protocol `app.bsky.feed.repost`.
* Instagram: not supported — bridge MUST emit `delivery_receipt` with
  `status = unsupported, reason = no_share_primitive`.

### 3.2 Cross-Posting Best Practices

Bridges SHOULD:

1. Preserve the canonical link to the WIA-SOCIAL activity in a subtle
   appendix (e.g. ` — wia.social/a/01HX…`) to avoid identity laundering.
2. Down-convert `text/markdown` to plain text per vendor.
3. Trim attachments to the vendor's per-post limit, oldest-first.
4. Surface vendor-side rate limit errors back as `delivery_receipt` with
   `retry_after` so the origin host can re-queue.

---

## 4. Open Federation Adapters

### 4.1 ActivityPub

An adapter exposes WIA-SOCIAL subjects as ActivityPub `Actor` objects:

* `inbox` — receives ActivityPub `Create`, `Follow`, `Like` activities and
  translates them to WIA-SOCIAL `post`, `follow`, `reaction`.
* `outbox` — emits ActivityPub `Create` for each WIA-SOCIAL `post` with
  audience `public` or `friends`.
* `featured` — exposes pinned activities.
* `publicKey` — re-publishes the subject's Ed25519 key as an
  `application/activity+json` `publicKey` object.

The adapter MUST honour ActivityPub `Follow` semantics: every
`Follow` is mirrored as a WIA-SOCIAL `follow` activity into the subject's
origin host.

### 4.2 AT Protocol (Bluesky)

An adapter performs:

* `com.atproto.identity.resolveHandle` ↔ WIA-SOCIAL discovery.
* `app.bsky.feed.post` ↔ WIA-SOCIAL `post`.
* `app.bsky.feed.like` ↔ WIA-SOCIAL `reaction` with `value=like`.
* PDS storage of the subject's repository, with WIA-SOCIAL signature
  alongside the AT Protocol commit signature.

### 4.3 Nostr

An adapter mirrors:

* `kind:1` events ↔ WIA-SOCIAL `post`.
* `kind:7` reactions ↔ WIA-SOCIAL `reaction`.
* `kind:5` deletions ↔ WIA-SOCIAL `delete`.

The adapter MUST verify Nostr secp256k1 signatures and re-sign in Ed25519
for the WIA-SOCIAL side. Subjects who federate via Nostr MUST keep both
keys; the standard does not assume key reuse across curves.

---

## 5. WIA Family Integration

### 5.1 WIA-OMNI-API

The "mother" standard provides credential storage. WIA-SOCIAL bridges
SHOULD fetch vendor credentials through WIA-OMNI-API rather than storing
OAuth tokens locally. The handshake is:

```
GET https://omni.example/credential/{subject}/{network}
Authorization: WIA-Sig …

→ 200 { "vendor_token": "…", "expires_at": "…" }
```

### 5.2 WIA-AIR-SHIELD

The "aunt" standard provides transport hardening. WIA-SOCIAL hosts MAY
enrol in WIA-AIR-SHIELD's reputation feed to refuse handshakes from peers
with low trust scores. Enforcement is host-policy; the standard does not
mandate a specific score threshold.

### 5.3 WIA-INTENT

The "father" standard provides high-level user intents. A WIA-INTENT verb
of `share-with-everyone` SHOULD be lowered to a WIA-SOCIAL activity with
`audience=public` and `fanout` of every registered network for the
subject. This is how a single user gesture becomes a federated post.

### 5.4 WIA-ACCESSIBILITY

For visual / audio attachments, WIA-SOCIAL implementations MUST honour the
WIA-ACCESSIBILITY profile:

* Image attachments: `alt_text` is REQUIRED (Phase 1 §6.3).
* Video attachments: captions track (`text/vtt`) is REQUIRED.
* Audio attachments: transcript URL is RECOMMENDED.

These align with W3C WCAG 2.1 success criteria 1.1.1, 1.2.2, 1.2.4.

---

## 6. Migration Paths

### 6.1 From a Legacy Account

A user owning a legacy Twitter account can migrate to WIA-SOCIAL by:

1. Creating a fresh identity bundle on a WIA-SOCIAL origin host.
2. Adding a `network` claim of `twitter` with their handle.
3. Posting a one-time verification tweet referencing the bundle's
   `signature.value`.
4. Granting the origin host an OAuth token (via WIA-OMNI-API).

Once verified, the bridge handles bidirectional posting automatically.

### 6.2 Between WIA-SOCIAL Hosts

A subject moves origin host by:

1. Publishing a `move` activity on the old origin pointing at the new
   origin's URL.
2. Publishing a fresh identity bundle on the new origin whose
   `previous_signature` chains back to the bundle on the old origin.
3. The new origin re-issues federation handshakes to existing peers.

Old origin hosts MUST honour the move for at least 12 months by responding
`301 Moved Permanently` with the new bundle URL.

---

## 7. Observability for Operators

Operators of bridges and adapters SHOULD expose:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_social_bridge_deliveries_total{network,status}` | counter | Per-network delivery outcomes |
| `wia_social_bridge_oauth_refreshes_total{network}` | counter | OAuth token refresh frequency |
| `wia_social_adapter_inbox_received_total{protocol}` | counter | Inbound from ActivityPub / AT / Nostr |
| `wia_social_bridge_queue_depth{network}` | gauge | Pending activities per bridge |

These metrics are intended for SRE dashboards and SHOULD NOT include
subject-identifying labels.

---

## 8. Conformance Profiles

A bridge or adapter is conformant at one of three levels:

| Level | Required networks |
|-------|-------------------|
| **Minimal** | At least one network with `post`, `delete` |
| **Core**    | Plus `comment`, `reaction`, `follow` |
| **Full**    | Plus `share`, `edit`, threaded `comment` |

Implementations MUST publish their level in their `/.well-known/wia-social`
document under `bridge_profile`.

---

## 9. Worked Example — Cross-Posting to Twitter and Mastodon

```
Subject     : acct:alice@origin.example
Origin host : https://origin.example/wia-social
Bridges     : https://twb.example  (twitter), https://mab.example (mastodon)
```

1. Alice posts via her client (Phase 2 §5.1):

   ```json
   { "type":"activity","verb":"post",
     "actor":"acct:alice@origin.example",
     "content":{"format":"text/plain","body":"Coffee."},
     "fanout":[{"network":"twitter"},{"network":"mastodon"}] }
   ```

2. Origin signs the activity, fans out to `twb.example` and `mab.example`.
3. Each bridge fetches the appropriate OAuth token from WIA-OMNI-API.
4. Each bridge calls the vendor API, then emits a `delivery_receipt`:

   ```json
   { "type":"delivery_receipt",
     "for_activity":"act_01HX…",
     "network":"twitter",
     "status":"delivered",
     "vendor_id":"https://twitter.com/alice/status/1234567890",
     "delivered_at":"2025-01-15T10:30:02Z" }
   ```

5. Origin records the receipt and exposes it on the activity's read
   endpoint for client UIs to render.

---

## 10. Security Considerations

* Bridges hold vendor credentials and are therefore high-value targets.
  Operators SHOULD use WIA-OMNI-API for credential storage, hardware
  security modules for OAuth refresh tokens, and short-lived access
  tokens whenever the vendor permits.
* Adapters that receive cryptographic material from a foreign protocol
  (e.g. Nostr secp256k1 signatures) MUST verify them strictly; a
  forged inbound signature would taint the subject's WIA-SOCIAL feed.
* Cross-network rate limits MUST NOT be aggregated — a single subject's
  rate quota is enforced per-network by the bridge's vendor agreement.

---

## 11. Conformance Checklist

A Phase 4 conformant integration MUST:

1. Publish a discovery document listing supported networks and the
   `bridge_profile` level.
2. Honour the delivery-receipt contract for every activity it accepts.
3. Implement `move` migration as the source side (§6.2).
4. Surface vendor errors as `delivery_receipt` rather than swallowing them.
5. Honour WIA-ACCESSIBILITY requirements for media attachments.

---

## 12. References

* W3C ActivityPub
* AT Protocol (Bluesky)
* Nostr NIP-01, NIP-09, NIP-25
* W3C WCAG 2.1
* IETF RFC 6749 — OAuth 2.0
* IETF RFC 9421 — HTTP Message Signatures
* WIA-OMNI-API standard
* WIA-AIR-SHIELD standard
* WIA-INTENT standard
* WIA-ACCESSIBILITY standard

---

## Appendix A — Bridge Configuration File

A reference bridge configuration uses TOML:

```toml
[bridge]
network = "twitter"
profile = "Core"
discovery_url = "https://twb.example/.well-known/wia-social"

[credentials]
provider = "wia-omni-api"
omni_endpoint = "https://omni.example"

[delivery]
max_retries = 5
backoff = "exponential"
initial_delay_ms = 250
max_delay_ms = 30000

[telemetry]
prometheus_port = 9091
```

Bridge operators MAY embed additional vendor-specific sections; conformant
implementations MUST ignore unknown sections rather than refuse to start.

## Appendix B — Adapter State Machines

ActivityPub `Follow`:

```
remote → POST /actor/inbox  Activity={Follow}
inbox  → translate → WIA-SOCIAL { verb:"follow", actor:remote, object:local }
origin → record edge, emit Accept back to remote (ActivityPub)
```

AT Protocol `like`:

```
relay → app.bsky.feed.like { subject: at://… }
adapter → translate → WIA-SOCIAL { verb:"reaction", value:"like" }
origin → fan-out to subscribers
```

Adapter implementations MUST round-trip native objects through the
WIA-SOCIAL representation; loss of metadata between protocols is permitted
only where the destination protocol cannot represent the source's semantics
(e.g. ActivityPub `Question` polls have no clean WIA-SOCIAL mapping in
v1.0).

## Appendix C — Worked Migration Example

Bob moves from `origin-old.example` to `origin-new.example`:

1. On the old origin, Bob signs a `move` activity:

   ```json
   { "type":"activity","verb":"move",
     "actor":"acct:bob@origin-old.example",
     "moved_to":"https://origin-new.example/wia-social/bundle/acct:bob@origin-new.example",
     "signature":{"alg":"Ed25519","value":"…"} }
   ```

2. On the new origin, Bob publishes a fresh bundle whose
   `previous_signature` equals the prior bundle's `signature.value`.
3. The new origin issues handshakes to every peer that previously knew
   `acct:bob@origin-old.example`, presenting the chain.
4. Each peer rebinds its local edges to the new subject identifier.
5. The old origin returns `301 Moved Permanently` for every
   `/bundle/acct:bob@origin-old.example` read for at least 12 months.

This pattern preserves Bob's social graph across host migrations without
requiring re-follow from each contact.

弘益人間 — Benefit All Humanity.
