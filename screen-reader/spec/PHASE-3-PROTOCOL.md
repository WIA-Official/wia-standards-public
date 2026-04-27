# WIA Screen Reader — Phase 3: Protocol

**Standard**: WIA Screen Reader
**Phase**: 3 of 4 — Protocol
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 3 specifies how independent WIA Screen Reader hosts (NVDA cloud,
VoiceOver enabler service, browser-extension publisher, accessibility
NGO mirror) build trust over time, share pronunciation hints, replay
accessibility tree decisions across devices, and handle the privacy
boundary that screen-reader telemetry necessarily crosses.

---

## 2. Roles

| Role | Description |
|------|-------------|
| **Reader** | NVDA / VoiceOver / TalkBack / Orca / browser extension running on a user device |
| **Hint host** | Holds pronunciation hints curated by language communities |
| **Profile host** | Holds the user's portable profile (preferred voice, rate, braille grade) |
| **Telemetry aggregator** | Holds anonymous engagement metrics for accessibility research |
| **Custodian** | Holds the user's signed consent envelope governing what telemetry may flow |

A single legal entity MAY play multiple roles. Trust between roles is
established by federation handshake and recorded in signed receipts.

---

## 3. Federation Handshake

```
   IDLE
     │ peer presents credential + ephemeral key
     ▼
   PENDING (origin verifies signature)
     │ valid
     ▼
   ACCEPTED (origin issues federation receipt; both sides persist)
     │ optional revocation
     ▼
   REVOKED
```

The handshake reuses the WIA-SOCIAL Phase 3 §5 receipt shape so that
vendor implementations can share their federation library across
multiple WIA family standards.

---

## 4. Pronunciation Hint Federation

Pronunciation hints are the most-shared resource in the standard. A
language community (Korean educators publishing technical-term hints,
medical professionals publishing drug-name hints) curates a hint set;
any reader can subscribe and consume the hints in real time.

```
Reader → Hint host: GET /sr/pronunciation?language=ko-KR&since=cursor
Hint host → Reader: SSE stream of new / updated hints
```

Hints are signed by the curator. Readers MUST verify the curator
signature before applying a hint; an unverified hint MUST fall back to
the rule-based engine.

### 4.1 Conflict Resolution

When two curators publish conflicting hints for the same `(language,
token)` pair, the reader resolves by:

1. User's explicit override (in `wia_pronunciation_hint_id` of the
   user profile), or
2. Curator the user has subscribed to, or
3. Curator with the highest community trust score (see §9).

The standard does not pick winners — it encodes the precedence so the
reader picks deterministically.

---

## 5. Profile Portability

A user moving between devices (a desktop NVDA at work, an iOS
VoiceOver at home) carries the profile via:

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "profile_move",
  "user_id": "did:wia:reader:01HXY",
  "from_host": "did:wia:reader-host:nvda-cloud",
  "to_host":   "did:wia:reader-host:voiceover-enabler",
  "moved_at": "2026-04-27T10:00:00Z",
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

The new host fetches the profile from the prior host and verifies the
move signature. The old host returns `301 Moved Permanently` for the
profile URL for at least 12 months.

---

## 6. Telemetry Consent

Telemetry is the only category of cross-host data flow that requires a
positive user consent. The consent envelope:

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "telemetry_consent",
  "user_id": "did:wia:reader:01HXY",
  "scopes": ["aggregate_engagement", "language_quality"],
  "valid_from": "2026-04-27T00:00:00Z",
  "valid_until": "2026-10-27T00:00:00Z",
  "signature": { "alg": "Ed25519", "value": "Vr3w…" }
}
```

Telemetry hosts MUST refuse submissions from users whose consent is
absent, expired, or whose scope does not include the planned use. The
default is no consent — the user must opt in.

Even with consent, the standard's telemetry frames (Phase 1 §8) are
strictly anonymous: no page URL, no user identifier, no page content.
The aggregator can study which actions users take but never which pages
they read.

---

## 7. Replay Defence

Each signed envelope (hint, profile write, profile move, consent)
carries a 96-bit nonce and an RFC 3339 timestamp. Receivers MUST:

1. Reject envelopes with skew > ±300 s.
2. Reject envelopes whose `(signer, nonce)` tuple has been seen within
   the last 600 s.
3. Maintain the seen-nonce cache for at least 600 s.

For high-volume hint federation the cache MAY be a Bloom filter; false
positives MUST trigger re-fetch via cursor rather than silent drops.

---

## 8. Audience Controls

| Audience | Visibility |
|----------|------------|
| `public`            | published pronunciation hints, tree templates |
| `reader`            | own profile, own braille mappings |
| `curator`           | publish hints under their identity |
| `aggregator`        | anonymous telemetry under telemetry consent |
| `accessibility_research` | aggregated telemetry under explicit research consent |

Hosts MUST refuse cross-class enrichment that defeats the matrix. For
example, joining telemetry frames with profile data on behalf of the
research audience is prohibited even when both are individually
permitted.

---

## 9. Curator Trust Score

Curators are assigned a community trust score in `[0.0, 1.0]` published
in the host's trust list. Score factors:

* Endorsements from other curators in the same language.
* Acceptance rate of the curator's hints by readers (rolling 90 days).
* Absence of revocation events.

Trust scores are advisory. Readers ultimately follow user preference
and explicit subscription; the score informs the conflict-resolution
default in §4.1.

---

## 10. Cryptographic Suite

| Use | Algorithm | Reference |
|-----|-----------|-----------|
| Identity signing | Ed25519 | IETF RFC 8032 |
| HTTP message signing | Ed25519 over RFC 9421 | RFC 9421 |
| Hashing | SHA-256 | FIPS 180-4 |
| Transport | TLS 1.3 | IETF RFC 8446 |

---

## 11. Conformance

A Phase 3 conformant implementation MUST:

1. Implement the federation handshake state machine.
2. Honour replay-defence bounds.
3. Enforce telemetry consent before accepting submissions.
4. Apply audience-based read controls.
5. Resolve hint conflicts per §4.1 deterministically.

---

## 12. References

* IETF RFC 8032 — EdDSA
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* FIPS 180-4 — SHA family
* W3C WAI-ARIA 1.2

---

## Appendix A — Worked Hint-Federation Trace

```
α = Korean medical-terminology curator did:wia:curator:kma-medlex
β = browser-extension reader on user device
λ = user did:wia:reader:01HXY

T-1d  α publishes hints for 200 new drug names (each hint signed)
T+0   β: GET /sr/pronunciation?language=ko-KR&since=cur_AAAA
T+0   α: SSE stream of new hints
T+1s  β: verifies each hint's curator signature, applies to local cache
T+10s λ: opens a clinical decision-support web app
T+10s β: encounters drug name "Apixaban", finds hint, applies WIHP "아픽사반"
T+11s β: emits anonymous telemetry "pronunciation_hint_applied" (under consent)
```

If α's signature fails verification (e.g. α's key has rotated), β
falls back to the rule-based engine and surfaces a UI nudge to
re-subscribe. The reader never silently mis-pronounces a hinted token.

## Appendix B — Replay Cache Sizing

For a busy hint host receiving 100 hint envelopes per second across
all curators, the seen-nonce cache must hold roughly
`100 × 600 = 60 000` entries to enforce §7's 600-second window.
With 16-byte nonce keys plus a 4-byte timestamp, the strict cache
footprint is approximately `60 000 × 24 ≈ 1.5 MiB`. Hosts SHOULD
provision at least double this to absorb curator-batch publishes
and SHOULD persist the cache across restarts so a failover does
not re-open the window for a duplicated hint.

## Appendix C — Privacy Threats and Mitigations

| Threat | Mitigation |
|--------|------------|
| Telemetry aggregator correlates frames by IP | Hosts MUST strip IP at ingestion; aggregator MUST refuse frames carrying IP |
| Profile leakage between devices via federated profile move | Move envelopes are signed by the user; hosts MUST verify the move signature against the user's published key |
| Curator impersonation | Hint envelopes are signed by the curator; readers MUST verify before applying |
| Side-channel timing on signature verify | Constant-time Ed25519 implementations REQUIRED |
| Page-content disclosure via tree submission | Trees are addressed by salted page-URL hash, not raw URL; hosts MUST NOT log raw URLs |

## Appendix D — Trust List Maintenance

Each host maintains a signed trust list:

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "trust_list",
  "host_id": "did:wia:reader-host:nvda-cloud",
  "issued_at": "2026-04-01T00:00:00Z",
  "valid_until": "2026-05-01T00:00:00Z",
  "entries": [
    { "peer_id": "did:wia:curator:kma-medlex", "role": "curator", "score": 0.94 },
    { "peer_id": "did:wia:curator:nvda-en-tech", "role": "curator", "score": 0.91 },
    { "peer_id": "did:wia:aggregator:wia-research", "role": "aggregator", "score": 1.00 }
  ],
  "signature": { "alg": "Ed25519", "value": "…" }
}
```

Trust lists are republished at least monthly; peers refuse stale lists
older than 60 days. A peer may self-publish a `revocation` envelope
to immediately drop trust between list refresh windows.

## Appendix E — Operator Failover

When a hint host fails over from primary to standby region, the
standby MUST:

1. Reload the persistent seen-nonce cache before resuming envelope
   processing — failure to do so re-opens a 600-second window for an
   attacker to replay a previously-seen hint envelope and corrupt
   the curator's published set.
2. Re-issue handshakes to peers whose receipts are not present in
   the standby's storage.
3. Replay any missed hint envelopes from the primary's append-only
   log before serving subscribers.
4. Notify peers via a `notice` envelope that primary→standby
   switchover has occurred, with an estimated `restore_at`.

## Appendix F — Audience Decision Matrix

| Caller | pronunciation_hint | a11y_node | reading_order | profile | telemetry |
|--------|--------------------|-----------|---------------|---------|-----------|
| Public anonymous | published hints only | ✗ | ✗ | ✗ | ✗ |
| Reader (own profile) | full + own private hints | own page hash | own plans | own | own (consent) |
| Curator (own hints) | own | ✗ | ✗ | ✗ | ✗ |
| Aggregator | ✗ | ✗ | ✗ | ✗ | aggregate (consent) |
| Accessibility research | ✗ | ✗ | ✗ | ✗ | aggregate (research consent) |

Hosts MUST refuse cross-class enrichment that defeats the matrix —
for example, joining telemetry frames with profile data on behalf of
the research audience is prohibited even when both are individually
permitted.

## Appendix G — Curator Onboarding Flow

A new curator joining the federation:

1. Curator generates an Ed25519 keypair locally (HSM-backed if
   possible).
2. Curator publishes their public key + identity claim envelope to
   their chosen hint host's curator-onboarding endpoint.
3. Hint host adds the curator to its trust list at score 0.5 (default
   starting score for unverified curators).
4. Curator publishes their first signed hint envelopes; readers who
   subscribe see them flow.
5. As reader subscriptions and acceptance rate grow, the curator's
   trust score rises (per §9 factors).
6. Other curators may endorse the new curator; endorsements raise
   the score faster.

A curator may operate at any score above zero; the score affects
default conflict-resolution but does not gate publishing.

## Appendix H — Operator Notes for Curators

* Curators SHOULD batch hint publishes into envelopes of no more than
  100 entries each to keep the publishing window under the host's
  rate-limit ceiling.
* Curators SHOULD attach a short justification field to controversial
  hints (e.g. medical drug name pronunciations that have multiple
  competing forms in clinical practice) so that readers and other
  curators can understand the reasoning behind the chosen
  pronunciation.
* Curators MUST NOT use the standard to publish hints that mislead
  the user (e.g. hints intended to make a brand name read as a
  competitor's name); doing so will result in revocation by the
  host and a permanent reduction in trust score.

弘益人間 — Benefit All Humanity.
