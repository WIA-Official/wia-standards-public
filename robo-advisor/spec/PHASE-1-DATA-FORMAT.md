# PHASE 1 — Data Format

> Robo-advisor canonical envelopes: investor profile, portfolio,
> rebalance, fee disclosure, and tax-loss harvesting events.
> All envelopes are signed with Ed25519 over the canonical JSON
> form (RFC 8785 JCS).

## 1.1 Investor profile envelope

The `investor_profile` envelope captures the suitability inputs that
drive every subsequent allocation decision. It is signed by the
sponsoring advisor and re-signed at every material change.

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "investor_profile",
  "investor_id": "did:wia:investor:...",
  "advisor_id":  "did:wia:advisor:...",
  "captured_at": "RFC 3339",
  "kyc_jurisdiction": "ISO 3166-1 alpha-2",
  "risk_tolerance": "conservative" | "moderate" | "aggressive",
  "risk_capacity": {
    "horizon_years": 0,
    "liquidity_floor_pct": 0,
    "loss_tolerance_pct_24m": 0
  },
  "investment_objectives": [
    "retirement", "education", "first-home", "wealth-preservation",
    "wealth-accumulation", "income"
  ],
  "income_band": "USD-equiv tier",
  "net_worth_band": "USD-equiv tier",
  "tax_residence": "ISO 3166-1 alpha-2",
  "constraints": {
    "esg_filters": [],
    "sector_excludes": [],
    "single_position_max_pct": 0
  },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `risk_tolerance` and `risk_capacity` are intentionally split:
tolerance is a behavioural input (how the investor *feels* about
loss) while capacity is a structural input (how much loss the
investor can *absorb* without violating an objective). Conflating
them is the most common source of mis-suitability findings.

## 1.2 Portfolio envelope

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "portfolio",
  "portfolio_id": "...",
  "investor_id": "did:wia:investor:...",
  "as_of": "RFC 3339",
  "base_ccy": "ISO 4217",
  "total_value_base_ccy": 0,
  "holdings": [
    {
      "instrument_ref": "ISO 6166 ISIN | FIGI | tenant-private",
      "quantity": 0,
      "market_value_base_ccy": 0,
      "weight_pct": 0,
      "asset_class": "equity" | "fixed-income" | "alternative"
                  | "cash" | "real-estate",
      "tax_lot_count": 0
    }
  ],
  "performance": {
    "twr_pct_ytd": 0,
    "twr_pct_1y":  0,
    "twr_pct_3y_ann": 0,
    "drawdown_pct_max_1y": 0
  },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`twr_pct` is the time-weighted return; the standard requires TWR rather
than money-weighted return for portfolio reporting because TWR is
robust to advisor-driven contributions and withdrawals and is the
basis for cross-advisor comparability under GIPS conventions.

## 1.3 Rebalance event

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "rebalance",
  "rebalance_id": "...",
  "portfolio_id": "...",
  "trigger": "drift_threshold" | "scheduled" | "policy_change"
           | "tax_loss_harvest" | "investor_initiated",
  "initiated_at": "RFC 3339",
  "target_weights": [ { "instrument_ref": "...", "weight_pct": 0 } ],
  "executed_trades": [
    {
      "instrument_ref": "...",
      "side": "buy" | "sell",
      "quantity": 0,
      "executed_price": 0,
      "venue": "MIC code (ISO 10383)",
      "executed_at": "RFC 3339"
    }
  ],
  "settlement_summary": {
    "cash_delta_base_ccy": 0,
    "fees_base_ccy": 0,
    "tax_impact_estimate_base_ccy": 0
  },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `tax_impact_estimate_base_ccy` is mandatory because rebalancing
without tax-impact attestation is the single largest source of
post-trade investor surprise. The estimate is signed by the advisor;
the actual tax outcome is reconciled by the investor's tax preparer.

## 1.4 Fee disclosure envelope

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "fee_disclosure",
  "investor_id": "did:wia:investor:...",
  "billing_period_start": "RFC 3339",
  "billing_period_end":   "RFC 3339",
  "advisory_fee_bps_ann": 0,
  "advisory_fee_charged_base_ccy": 0,
  "expense_ratios_underlying_pct_weighted": 0,
  "transaction_costs_base_ccy": 0,
  "fx_costs_base_ccy": 0,
  "tax_drag_estimate_pct_ann": 0,
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Fee disclosure is recurring (typically quarterly), not just at
onboarding, because advisor billing changes silently otherwise.
Underlying expense ratios MUST be weighted by the holdings active
during the billing period, not by current holdings.

## 1.5 Tax-loss harvesting event

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "tax_loss_harvest",
  "tlh_id": "...",
  "portfolio_id": "...",
  "harvested_at": "RFC 3339",
  "lots_sold": [
    {
      "instrument_ref": "...",
      "lot_id": "...",
      "acquired_at": "RFC 3339",
      "cost_basis_base_ccy": 0,
      "proceeds_base_ccy": 0,
      "realised_loss_base_ccy": 0
    }
  ],
  "wash_sale_avoided_until": "RFC 3339",
  "replacement_holding": "instrument_ref",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`wash_sale_avoided_until` is mandatory in jurisdictions that recognise
wash-sale or bed-and-breakfast rules (US §1091, UK 30-day, KR
유사매매); the advisor MUST NOT acquire the same or substantially
identical security before this date.

## 1.6 Compliance and references

Phase 1 envelopes are designed for direct binding to:

- **SEC Form ADV** disclosure obligations
- **FINRA Rule 2111** suitability evaluation
- **MiFID II** suitability assessment (Articles 24–25)
- **GIPS 2020** performance presentation conventions
- **FATF Travel Rule** for cross-border movement
- **ISO 20022** payment messaging for settlement linkage

The wire format does not redefine these obligations; it provides a
canonical envelope so that one signed event can satisfy multiple
jurisdictional reporting forms without re-keying.

## 1.7 Suitability re-assessment cadence

A suitability re-assessment is REQUIRED at the earliest of:

- 24 months from the most recent assessment;
- a material life event reported by the investor (marriage, divorce,
  birth of a dependent, retirement, change of jurisdiction);
- a portfolio drift of more than 10 percentage points from the
  target allocation that persists for more than 60 calendar days;
- a regulatory change that materially affects the assumptions of
  the original assessment.

The re-assessment is itself an `investor_profile` envelope; the
chain of envelopes for an investor forms a complete audit trail
of the advisor's understanding of the investor over time.

## 1.8 Custody binding

Each `portfolio` envelope is bound to one or more custodians via the
`custody_binding` sub-envelope:

```
{
  "custodian_id": "did:wia:custodian:...",
  "account_ref": "opaque",
  "scope": ["all-assets" | "specific-instruments"],
  "discretion": "discretionary" | "non-discretionary",
  "signature_by_custodian": "Ed25519"
}
```

A custodian's counter-signature on the binding establishes the
chain of authority for the advisor to place trades on the account.
The standard does not require a single custodian per portfolio; an
investor may hold positions across multiple custodians under one
advisor's discretion, and the bindings are independent.

## 1.9 Investor consent

Every investor-driven action (initial account opening, allocation
change, withdrawal, account closure) is recorded as an
`investor_consent` envelope referencing the relevant portfolio and
the action being consented to. The envelope is signed by the
investor; the advisor never speaks for the investor on consent.

## 1.10 Algorithm transparency envelope

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "algorithm_attestation",
  "algorithm_id": "...",
  "algorithm_kind": "mean-variance" | "risk-parity" | "black-litterman"
                  | "factor-tilt" | "glide-path" | "ml-derived",
  "rebalance_policy": "drift-band" | "calendar" | "tax-aware",
  "drift_threshold_pct": 0,
  "version": "semver",
  "training_data_window": { "from": "RFC 3339", "to": "RFC 3339" },
  "validation_metrics": {
    "out_of_sample_sharpe_estimate": 0,
    "max_drawdown_in_sample_pct": 0
  },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`algorithm_attestation` envelopes are published whenever the live
algorithm changes — version, kind, training window, or any policy
parameter. The investor's portfolio events reference the
`algorithm_id` active at the time of the event; reconciling outcomes
across algorithm versions is the core of advisor-side performance
attribution.

## 1.11 ESG and exclusion filters

`investor_profile.constraints.esg_filters` accepts canonical filter
identifiers maintained at `https://wiastandards.com/robo-advisor/esg/`:

- `tobacco`, `controversial_weapons`, `thermal_coal`, `oil_sands`
- `gambling`, `adult_entertainment`
- `un_global_compact_violators`
- `paris_aligned_benchmark` (positive screen)
- `eu_taxonomy_aligned_revenue_pct ≥ N`

Each filter is a deterministic, auditable rule. Vendors MAY add
proprietary filters; portfolios constructed under proprietary
filters MUST disclose the proprietary identifier in the portfolio's
algorithm attestation so that downstream comparability remains
honest.

## 1.12 Currency and FX handling

Portfolios are reported in a `base_ccy` chosen at portfolio creation
and never silently changed. Foreign-currency holdings are
translated using the WM/Refinitiv 4 p.m. London fix or the local
equivalent (e.g., KRX 마감환율 for KRW base portfolios), with the
fix source recorded in the `portfolio.fx_source` field.

Currency hedging is treated as a holding, not a portfolio overlay:
hedge contracts (FX forwards, currency-hedged share classes)
appear in the holdings array with `asset_class: "fx-hedge"` and
their notional contribution to base-currency exposure is summed
into the portfolio's net currency table.

## 1.13 Glide-path and lifecycle envelopes

For age-based or goal-based glide paths (target-date, retirement
income), the `glide_path_attestation` envelope describes the
schedule of target weights as a function of horizon years
remaining. The schedule is a piecewise-linear interpolation between
named anchor ages; the standard does not mandate the anchors but
does require that the schedule be deterministic and signed.

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "glide_path_attestation",
  "glide_path_id": "...",
  "anchors": [
    { "horizon_years": 40, "weights": { "equity": 0.90, "fixed": 0.10 } },
    { "horizon_years": 20, "weights": { "equity": 0.65, "fixed": 0.35 } },
    { "horizon_years":  5, "weights": { "equity": 0.40, "fixed": 0.60 } },
    { "horizon_years":  0, "weights": { "equity": 0.30, "fixed": 0.70 } }
  ],
  "interpolation": "piecewise-linear",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## 1.14 Suitability mismatch event

When a portfolio's measured risk exposure drifts beyond the
investor's `risk_capacity` for more than 30 calendar days, the
advisor MUST emit a `suitability_mismatch` envelope. The envelope
names the drift, the elapsed days, the proposed remediation, and
the investor-acknowledgement requirement. The mismatch envelope is
a regulatory artefact in many jurisdictions (FINRA Rule 2111
suitability supervision; MiFID II Article 25 ongoing suitability
review; KR 자본시장법 적합성 원칙).

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "suitability_mismatch",
  "investor_id": "did:wia:investor:...",
  "portfolio_id": "...",
  "detected_at": "RFC 3339",
  "metric": "drawdown_24m_pct" | "concentration_pct"
         | "currency_exposure_pct" | "esg_filter_breach",
  "observed": 0,
  "tolerance": 0,
  "remediation_proposed": "rebalance" | "reprofile" | "withdrawal",
  "investor_ack_required_by": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## 1.15 Audit and tamper evidence

Every envelope's signature covers the canonical JSON form of the
envelope's contents excluding the signature itself. The advisor
SHOULD maintain an append-only Merkle log of all emitted envelopes
and publish the log root daily via a `daily_attestation_root`
envelope so that consumers can detect retroactive modification of
historical envelopes.

The Merkle log discipline is what allows a regulator examining a
six-year-old envelope to verify that the envelope existed in that
form on its emission date, without requiring the regulator to have
captured the envelope at emission time.

弘益人間 — Benefit All Humanity. The standard exists so that the smallest organisation can sign one envelope and be heard by the largest.
