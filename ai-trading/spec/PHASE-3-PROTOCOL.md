# WIA-FIN-017 PHASE 3 — Protocol

**Standard:** WIA-FIN-017 AI Trading
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `WIA-FIN-017-spec-v1.0.md` §2
(Architecture) and §6 (Security Requirements)

This document defines the protocols connecting the trading platform,
model registry, exchange gateway, and audit log. References ISO/IEC
27001:2022 controls and FIX 5.0 SP2 messaging without redefining
them; protocol schemas reflect the message types defined in those
documents but do not duplicate them.

References:
- FIX Protocol 5.0 SP2 (FIX Trading Community)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27002:2022 (controls)
- IETF RFC 8446 (TLS 1.3)

---

## 2. Architecture

### 2.1 System Components

```
┌─────────────────────────────────────────────────────────┐
│                    AI Trading System                     │
├─────────────────────────────────────────────────────────┤
│  ┌───────────┐  ┌───────────┐  ┌────────────┐         │
│  │   Data    │  │    ML     │  │  Strategy  │         │
│  │ Ingestion │→ │  Models   │→ │  Execution │         │
│  └───────────┘  └───────────┘  └────────────┘         │
│       ↓               ↓              ↓                  │
│  ┌───────────────────────────────────────────┐         │
│  │         Risk Management & Monitoring       │         │
│  └───────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────┘
```

### 2.2 Data Layer

**Market Data Sources:**
- Real-time quotes and trades
- Order book snapshots (L2/L3 data)
- Historical OHLCV (Open, High, Low, Close, Volume)
- News and sentiment data
- Alternative data sources

**Data Format:**
```json
{
  "symbol": "BTC/USDT",
  "timestamp": 1704067200000,
  "type": "quote",
  "bid": 42150.50,
  "ask": 42151.00,
  "bid_size": 1.5,
  "ask_size": 2.3,
  "exchange": "binance"
}
```

### 2.3 ML Model Layer

**Supported Model Types:**
- Supervised Learning (LSTM, GRU, Transformers)
- Reinforcement Learning (DQN, PPO, SAC)
- Ensemble Methods (Random Forest, XGBoost)

**Model Metadata:**
```json
{
  "model_id": "lstm_btc_predictor_v1",
  "version": "1.0.0",
  "type": "lstm",
  "input_features": 50,
  "output_type": "regression",
  "training_data": {
    "start_date": "2023-01-01",
    "end_date": "2024-12-31",
    "samples": 250000
  },
  "performance": {
    "sharpe_ratio": 2.34,
    "max_drawdown": -0.124,
    "win_rate": 0.68
  }
}
```

### 2.4 Strategy Execution

**Signal Format:**
```json
{
  "timestamp": 1704067200000,
  "symbol": "BTC/USDT",
  "signal": "buy",
  "strength": 0.85,
  "confidence": 0.92,
  "price": 42150.50,
  "size": 0.5,
  "strategy_id": "ml_momentum_v1"
}
```

**Order Specification:**
```json
{
  "order_id": "uuid-12345",
  "symbol": "BTC/USDT",
  "side": "buy",
  "type": "limit",
  "quantity": 0.5,
  "price": 42150.00,
  "time_in_force": "GTC",
  "reduce_only": false
}
```


## 6. Security Requirements

### 6.1 Authentication

- API keys MUST use minimum 256-bit encryption
- Multi-factor authentication REQUIRED for live trading
- Key rotation RECOMMENDED every 90 days

### 6.2 Data Protection

- All data in transit MUST use TLS 1.3+
- Sensitive data at rest MUST be encrypted (AES-256)
- PII MUST comply with GDPR/CCPA


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
