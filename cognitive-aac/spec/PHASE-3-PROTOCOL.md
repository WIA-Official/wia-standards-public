# WIA Cognitive AAC · Phase 3 — Prediction, Learning & Protocol

**Document Version:** 1.1.0
**Status:** Active
**Last Updated:** 2026-04-25
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

### 1.1 Purpose

Phase 3 specifies the **personalised prediction and learning engine** that powers augmentative and alternative communication (AAC) for users with cognitive disabilities (autism spectrum, dementia, intellectual disability, post-stroke aphasia), together with the **wire protocols** that sync the engine across devices and caregivers. All learning is on-device; all data leaving the device is opt-in and cryptographically attributable.

### 1.2 Core principles

1. **On-device learning** — every per-user model trains locally; raw utterance history never leaves the device.
2. **Privacy-first federation** — if caregivers opt in, only differentially-private gradients cross the boundary.
3. **Adaptive prediction** — the model topology reflects the user's condition (autism routine, dementia reminiscence, etc.).
4. **Transparent reasoning** — every prediction carries a user-comprehensible `explanation` string.
5. **No loss of voice** — prediction ranking NEVER removes an unused symbol; it only reorders.

### 1.3 Normative references

- RFC 8446 — TLS 1.3
- RFC 7515 / 8785 — JWS and JSON Canonicalization
- RFC 7519 — JSON Web Token
- W3C WebSocket Protocol
- OpenTelemetry semantic conventions for user-facing applications
- McSherry (2009) — Differentially Private Mechanisms for Learning
- ISO/IEC 29100 — Privacy framework
- Dwork & Roth (2014) — The Algorithmic Foundations of Differential Privacy

---

## 2. Prediction model

### 2.1 Multi-source blend

```
P(s_next = x) = Σ_i w_i · P_i(x | ctx)        with Σ w_i = 1
```

| Source      | `P_i(·)` comes from          | Default weight |
|-------------|-------------------------------|:--------------:|
| frequency   | global symbol usage frequency | 0.25           |
| sequence    | n-gram over prior symbols     | 0.30           |
| context     | location, activity, interlocutor | 0.25        |
| time        | hour, day-of-week, routine    | 0.20           |

Weights are adjusted per-profile (§ 3.3–3.4) and MUST be transparent in the user-facing explanation pane.

### 2.2 Predicted-symbol record

```ts
interface PredictedSymbol {
  symbol_id:   string;
  probability: 0..1;
  source:      "frequency" | "sequence" | "context" | "time"
             | "routine"                   // autism profile
             | "reminiscence";             // dementia profile
  explanation: string;    // BCP-47 aware; e.g. "아침에 자주 사용"
  rank:        uint;
}
```

### 2.3 N-gram probability (canonical)

```
P(s_n | s_{n-k}…s_{n-1}) = (count(s_{n-k}…s_n) + α) / (count(s_{n-k}…s_{n-1}) + α·V)
```

- `k ≤ 3` (max 4-gram) — bounded to fit 10 MiB model budget (§ 7.3).
- Laplace smoothing with `α = 0.01`.
- Time decay applied every midnight: `count ← count · 0.95^days`.
- Counts below 2 after decay are pruned (saves ≈ 40 % of parameters on observation).

---

## 3. Condition-specific profiles

### 3.1 Autism spectrum

```ts
interface AutismPredictionModel {
  routine_weight: 1.5;            // boost symbols in today's routine
  transition_prediction: {
    warn_before_min: 5;
    suggest_transition_symbols: true;
  };
  consistent_predictions: true;   // identical context → identical order
  stim_tolerance: 0.2;            // allow 20% variation for curiosity
}
```

### 3.2 Dementia

```ts
interface DementiaPredictionModel {
  long_term_memory_weight: 1.3;   // favour symbols the user has known for years
  decay_factor: 0.99;             // forget slowly
  allow_repetition: true;         // do not suppress repeated queries
  orientation_support: {
    show_datetime: true;
    show_location: true;
    show_caregiver_name: true;
  };
  crisis_escalation_symbols: ["pain","dizzy","lost","fear"];
}
```

### 3.3 Intellectual disability and post-stroke aphasia

Profiles share a core of **concrete nouns first**, **motor-reduced UI** (min. 44 × 44 pt targets per WCAG 2.2 AAA), and **latency budget ≤ 150 ms** for speech synthesis trigger.

---

## 4. Patterns and storage

### 4.1 Temporal patterns

```ts
interface TemporalPattern {
  hourly_freq:   Map<0..23, SymbolFrequency[]>;
  day_of_week:   Map<0..6,  SymbolFrequency[]>;
  contextual:    Map<string, SymbolFrequency[]>;   // "meal", "class" …
}
```

### 4.2 Sequence patterns

```ts
interface SymbolChain {
  symbols:       string[];   // e.g. ["hello", "thanks"]
  frequency:     uint;
  avg_interval_ms: uint;
  probability:   0..1;
}
```

### 4.3 Contextual patterns

```ts
interface ContextualPattern {
  location_based: Map<string, SymbolFrequency[]>;
  person_based:   Map<string, SymbolFrequency[]>;
  activity_based: Map<string, SymbolFrequency[]>;
}
```

### 4.4 Storage envelope

All pattern stores are persisted as **encrypted** SQLite databases (SQLCipher, AES-256). The encryption key is derived from a device-bound passkey (FIDO2 / Secure Enclave); loss of the passkey destroys the model rather than leaking it.

---

## 5. Wire protocol

The prediction engine communicates with the AAC board (in-process or cross-process) via a structured protocol.

### 5.1 In-process IPC

A local Cap'n Proto/Protocol Buffers schema is used by the React Native / Electron / Web runtime:

```proto
message PredictRequest  { string user_id = 1; Context context = 2; repeated string recent = 3; uint32 top_k = 4; }
message PredictResponse { repeated PredictedSymbol symbols = 1; string explanation_locale = 2; }
message TrainEvent      { string user_id = 1; string symbol_id = 2; Context context = 3; uint32 response_ms = 4; }
message ErrorReport     { ErrorCode code = 1; string detail = 2; }
```

Transport is a Unix domain socket on desktop/mobile or MessagePort in browsers; the same schema is re-used across platforms so a single engine binary covers all UIs.

### 5.2 Caregiver sync (optional, opt-in)

```
wss://sync.cognitive-aac.wia.org/v1/ws
```

Frame envelope (JSON over WebSocket, JWS-signed):

```json
{
  "op":        "pattern_update",
  "device_id": "caac-dev-0042",
  "user_id":   "usr-pseudo-…",
  "payload":   { "type":"ngram_delta",
                 "epsilon": 2.0,
                 "delta":   1e-5,
                 "coeffs":  [ {"ngram":"…","delta":0.03}, … ] },
  "jws":       "eyJhb…"
}
```

Each frame is signed with the caregiver's ES256 key; the receiver verifies the signature and applies the delta only if the user has granted the `aac.pattern.receive` scope.

### 5.3 Message authentication

- **TLS 1.3** with ECDHE; certificate pinning on mobile clients.
- **JWS ES256** over canonicalised payload (RFC 8785) — verifies who produced the delta, independent of transport.
- **JWT access tokens** with claims `user_id`, `role` (`user` | `caregiver` | `clinician`), `scope`, 15-minute expiry, rotating refresh.

### 5.4 Replay and idempotency

Each frame carries a monotonic `sequence` and a UUIDv7 idempotency key; receivers keep a 1024-entry replay window per (device_id, user_id).

---

## 6. Differentially-private federation (optional)

### 6.1 Mechanism

Caregivers consenting to pooled improvement contribute **DP-SGD-style gradient deltas** rather than raw counts. For each reporting epoch:

```
noise  ~  Laplace(Δ/ε)       with Δ = 1.0, ε ≤ 2.0 per epoch
delta  =  clip(gradient, C=1.0) + noise
```

Per-user privacy budget `ε_total ≤ 8.0` per year; once exhausted, the device refuses further uploads.

### 6.2 Secure aggregation

Gradients from at least 50 devices are combined via a secure-aggregation protocol so that the coordinator only observes the sum, never individual contributions. The aggregate is published to clients as a signed model delta. Implementations rely on standard threshold-cryptography building blocks (Shamir secret sharing + Diffie-Hellman pairwise masking).

### 6.3 Opt-out

A single-click opt-out purges any pending upload queue and sets `federation=off` in the device policy. Caregivers cannot re-enable federation on behalf of the user — only the user or their legally designated advocate can.

---

## 7. Performance budget

### 7.1 Latency (P95)

| Operation             | Target | Hard cap |
|-----------------------|--------|----------|
| Next-symbol prediction|  50 ms |  100 ms  |
| Phrase completion     | 100 ms |  200 ms  |
| Learning update       |  10 ms |   50 ms  |
| Voice synthesis start | 120 ms |  200 ms  |

### 7.2 Accuracy

Measured on the AAC-3K validation set (3000 sessions × 8 predictions/ui-update):

| Metric       | Target |
|--------------|-------:|
| Top-1        | ≥ 30 % |
| Top-5        | ≥ 60 % |
| Top-8        | ≥ 75 % |
| Top-5 (autism, routine-constant days) | ≥ 80 % |

### 7.3 Resource envelope

| Resource         | Limit                  |
|------------------|------------------------|
| Memory (prediction) | < 50 MiB            |
| Model size       | < 10 MiB               |
| Battery impact   | < 5 % per hour of active use |
| CPU              | < 10 % on 2019-era mobile SoCs |

---

## 8. Adaptation lifecycle

### 8.1 Cold-start (new user)

- Week 1: demographic prior (age band, diagnosis, caregiver-tagged topics).
- Weeks 2–4: frequency learning enabled; sequence weights kept low (`0.5`).
- Month 2+: full multi-source blend; transitions and reminiscence activated per profile.

### 8.2 Confidence gating

```
confidence = min(1.0, usage_count / 100)
applied_weight = base_weight · (0.5 + 0.5 · confidence)
```

### 8.3 Model drift monitoring

The engine exports `wia_aac_prediction_top1_accuracy_windowed` (7-day window). If it drops by > 10 % week-over-week, the app nudges the user/caregiver to review the routine — never silently retrains on suspect data.

### 8.4 OTA model refresh

Framework-level prompts and global utility models (e.g., phoneme pronunciations, icon-symbol map) ship as signed bundles:

- ECDSA P-256 signature by the WIA AAC CA.
- `min_model_version` field prevents downgrade attacks.
- Delta-packaged (bsdiff) to keep updates ≤ 200 KiB on cellular connections.

---

## 9. Integration surface

### 9.1 Eye-gaze AAC

Engines MUST consume a Tobii/Irisbond-style gaze stream (60 Hz, position + dwell) and fold dwell confidence into candidate scoring so that users who select by gaze are not penalised by motor noise.

### 9.2 Voice-banking

Pre-recorded personal voices (patient, family) are kept in a `VoiceBank` table. The playback protocol MUST deliver the banked recording byte-identical; synthesised fallback is explicitly labelled in the UI.

### 9.3 Core-vocabulary packs

Vocabulary packs (LAMP, PODD, TouchChat-style) are loaded from WIA AAC-CLDR JSON files; ISO 639 locale + ICU message-format ensure the same icon → phrase mapping works across locales. Updates are additive only; the user's board layout is preserved across pack version bumps.

### 9.4 Cross-standard interop

- **WIA-MED-BE-001 Bionic Eye** — OCR results from § 4 (Phase 1) feed the AAC prediction as `text_source` context.
- **WIA-MED-BCI-001 BCI** — intention detection raises `crisis_escalation_symbols` when distress biomarker fires.
- **WIA-MED-ASSIST-001 Assistive Wearable** — haptic acknowledgements mirror every selection.

---

## 10. Reference APIs

### 10.1 TypeScript

```ts
const engine = PredictionEngine.forAutism({ locale: "ko-KR" });
const predictions = engine.predictNext(context, recentSymbols, { topK: 8 });
engine.recordSymbolSelection(sym, context, { responseMs: 410 });
```

### 10.2 Python

```python
from wia_cognitive_aac import PredictionEngine, Context

engine = PredictionEngine.for_dementia(locale="en-US")
preds  = engine.predict_next(Context(location="kitchen", time=datetime.now()), recent=["hungry"])
engine.record_symbol_selection("oatmeal", context, response_ms=620)
```

### 10.3 Explanation renderer

```tsx
<Explanation prediction={p}>
  {/* auto-translates to BCP-47 locale */}
  {p.source === "reminiscence" && "익숙한 기억과 연결돼 있어요"}
  {p.source === "context"      && `${p.context_label}에서 자주 사용돼요`}
</Explanation>
```

---

## 11. Compliance checklist

- [ ] On-device learning verified; no raw utterances leave the device.
- [ ] JWS-signed WebSocket frames; TLS 1.3 with pinning.
- [ ] Differential-privacy budget per device enforced and displayed to user.
- [ ] OTA model bundles signed by the WIA AAC CA and downgrade-proof.
- [ ] Profile-specific weights (autism / dementia / ID / aphasia) active.
- [ ] Latency budgets met on 2019-era mobile SoCs.
- [ ] Accuracy targets met on AAC-3K validation set.
- [ ] Crisis-escalation symbols always reachable in ≤ 2 taps.
- [ ] Opt-out purges federation queue immediately.

---

## 12. Normative References

1. W3C WCAG 2.2 AAA — target sizes and motor accessibility.
2. ISO/IEC 29100 — Privacy framework.
3. RFC 8949 — CBOR; RFC 9052 — COSE.
4. NIST SP 800-208 — Stateful hash-based signatures.
5. HIPAA Privacy Rule (45 CFR Part 164).
6. EU GDPR 2016/679 — special category data.
8. RFC 7515 (JWS); RFC 8785 (JSON Canonicalization); RFC 8446 (TLS 1.3).

---

**Document Status:** ACTIVE
**Effective Date:** April 25, 2026
**Review Date:** April 25, 2028

© 2026 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
