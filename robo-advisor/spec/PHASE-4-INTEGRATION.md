# PHASE 4 — Integration

> Integration with custodian, market data, regulatory reporting,
> and consumer-facing systems. Designed so that one signed envelope
> stream feeds many downstream consumers without re-signing.

## 4.1 Custodian integration

Custodian integration follows the binding model defined in Phase 1
§1.8. The advisor publishes `rebalance` envelopes carrying
trade-level detail (instrument, side, quantity, executed price,
venue MIC, timestamp); the custodian counter-signs the
`settlement_confirmation` envelope upon clearing.

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "settlement_confirmation",
  "rebalance_id": "...",
  "custodian_id": "did:wia:custodian:...",
  "settled_at": "RFC 3339",
  "settlement_currency": "ISO 4217",
  "fees_charged_base_ccy": 0,
  "tax_withheld_base_ccy": 0,
  "signature_by_custodian": "Ed25519",
  "signature_by_advisor":   "Ed25519"
}
```

The dual signature establishes that both advisor and custodian
agree on the settled facts. Discrepancies are recorded as
`reconciliation_break` envelopes referencing the differing
inputs.

## 4.2 Market data integration

Market data providers (Bloomberg BPipe, Refinitiv RDP, ICE
Consolidated Feed, Korea Exchange KRX market data) supply pricing
and corporate-action input. The integration carries the upstream
data via `market_data_snapshot` envelopes referenced by the
`portfolio` envelope's `as_of` timestamp; corporate actions
(splits, dividends, mergers) are carried by
`corporate_action_event` envelopes that the portfolio rebalancer
processes deterministically.

## 4.3 Regulatory reporting integration

Regulatory filings — SEC Form ADV, FINRA U4/U5, MiFID II RTS 28,
KR 분기 영업보고서, JP 金商法 reports — consume bundles of envelopes
rather than tabular extracts. The advisor packages the relevant
`portfolio`, `fee_disclosure`, and `algorithm_attestation`
envelopes for the reporting period into a `regulatory_bundle`
signed by the chief compliance officer.

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "regulatory_bundle",
  "filing_kind": "FORM-ADV-PT2A" | "MIFID-II-RTS-28"
              | "KR-FSS-QUARTERLY" | "JP-FSA-ANNUAL" | ...,
  "filing_period_start": "RFC 3339",
  "filing_period_end":   "RFC 3339",
  "evidence_refs": [ "...", "..." ],
  "signed_by_cco": "did:wia:officer:cco-...",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## 4.4 Tax preparation integration

End-of-year tax preparation consumes the `tax_loss_harvest` and
`portfolio` envelope streams to produce realised gain/loss
schedules. The standard provides a `tax_year_summary` envelope
that aggregates per-investor totals in the formats required by the
investor's tax residence jurisdiction (US Form 8949 inputs, KR
양도소득세 inputs, EU national-format inputs).

## 4.5 Consumer-facing integration

Consumer-facing dashboards consume the read-only credentials
defined in Phase 2 §2.16 plus the freshness contract defined in
Phase 3 §3.8. The combination ensures that consumer apps display
authentic, fresh, scope-bound data without needing the advisor's
full signing keys.

## 4.6 WIA family integration

This standard plugs into the broader WIA family at three points:

- **WIA-OMNI-API** — credential storage for advisor tenant keys;
- **WIA Money** — funding, withdrawal, and FX legs of advisor
  operations;
- **WIA-AIR-SHIELD** — transport hardening for federation
  endpoints exposed to public networks.

The integration is loosely coupled: advisors that do not use the
WIA family can still implement this standard end-to-end by
substituting equivalent infrastructure.

## 4.7 ISO 20022 settlement linkage

`settlement_confirmation` envelopes carry an optional
`iso20022_camt053_ref` field pointing to the corresponding
ISO 20022 camt.053 statement entry at the custodian. The pointer
allows treasury operations to reconcile the advisor's view with
the bank-statement view without manual matching.

## 4.8 Korean integration notes (마이데이터)

Korea's 마이데이터 (MyData) framework, supervised by the Financial
Services Commission via the Korea Financial Telecommunications and
Clearings Institute (KFTC), defines specific consent-management,
data-portability, and aggregator-licensing requirements. Advisors
operating in Korea bind to MyData by:

- registering as a 마이데이터 사업자 with the FSC;
- emitting `freshness_attestation` envelopes consistent with the
  MyData freshness SLA;
- handling `investor_consent_revocation` within the MyData-defined
  60-second SLA (which aligns with this standard's §3.5);
- mapping the MyData transaction taxonomy to this standard's
  envelope types in the published implementation guide.

The standard does not redefine MyData; it provides the
envelope binding that satisfies the data-content side of the
MyData specification.

## 4.9 European integration notes (PSD2 / DORA)

PSD2 account information service providers (AISPs) consume the
read-only credentials path; the strong customer authentication
(SCA) requirement is satisfied by the investor's hardware-bound
key (Phase 3 §3.15). DORA (Digital Operational Resilience Act)
requirements for ICT third-party risk management consume the
`algorithm_attestation` and `regulatory_bundle` envelope streams.

## 4.10 Settlement reconciliation discipline

Reconciliation runs daily at the close of the custodian's
settlement day. Each `settlement_confirmation` envelope is matched
against the originating `rebalance` envelope by `rebalance_id`;
unmatched confirmations and unconfirmed rebalances older than
T+5 are escalated as `reconciliation_break` envelopes.

The break envelope carries the difference vector (advisor's
expected settlement vs custodian's confirmed settlement) and a
proposed resolution; both parties counter-sign the resolution to
close the break. The chain of break envelopes is the operational
audit trail for back-office disputes.

## 4.11 Operational considerations

Custodian connectivity is the most operationally fragile integration
point in the standard. Implementations SHOULD maintain a secondary
custodian connection path for disaster recovery and SHOULD test
failover quarterly. The `reconciliation_break` envelope volume is
the leading indicator of an unhealthy primary connection.

Market-data licensing is a per-tenant concern; the standard does
not redistribute upstream-licensed data, only references it by
provider identifier and timestamp. Tenants are responsible for
their own market-data contracts.

## 4.12 Backwards compatibility

Implementations migrating from FIX 4.4 trade messaging to envelope-
based settlement confirmation MAY emit FIX execution reports in
parallel with `settlement_confirmation` envelopes during a
transitional window. The envelope is canonical for downstream
audit; the FIX message remains canonical for the trading-side
matching engine.

## 4.13 Performance attribution integration

Performance attribution decomposes portfolio returns into asset
allocation, security selection, and currency contribution. The
standard publishes `attribution_report` envelopes per
investor per period using either Brinson-Fachler (single-period)
or Brinson-Hood-Beebower (multi-period) methodologies, with the
chosen methodology declared in the envelope.

```
{
  "wia_robo_advisor_version": "1.0.0",
  "type": "attribution_report",
  "portfolio_id": "...",
  "period_start": "RFC 3339",
  "period_end":   "RFC 3339",
  "methodology": "brinson-fachler" | "brinson-hood-beebower",
  "components": {
    "asset_allocation_bps": 0,
    "security_selection_bps": 0,
    "currency_contribution_bps": 0,
    "interaction_bps": 0,
    "total_bps": 0
  },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Attribution reports are referenced from regulatory bundles and
investor-facing performance dashboards. The standard does not
mandate a methodology — both Brinson variants are widely accepted —
but does mandate that the chosen methodology be stated explicitly.

## 4.14 Reference list

- ISO 20022 — payment messaging
- ISO 6166 — ISIN identifier
- ISO 10383 — MIC venue codes
- ISO 4217 — currency codes
- ISO 3166-1 — country codes
- GIPS 2020 — Global Investment Performance Standards
- SEC Rule 17a-4 — broker-dealer record retention
- MiFID II RTS 28 — best execution disclosure
- KR 자본시장법 시행령 — Korean Capital Markets Act
- JP 金融商品取引法 — Japanese FIEA
- FATF Travel Rule — cross-border value transfer

## 4.15 Operational considerations (continued)

End-of-day cutover times vary by custodian and by market. The
standard recommends explicit documentation of cutover windows in
the capability advertisement (Phase 2 §2.8) so that downstream
consumers know when the day's portfolio envelope reflects settled
state versus pending settlement.

Holiday calendars are sourced per-jurisdiction from the
capability advertisement's `business_calendar_ref` field. The
standard does not redistribute calendars; it provides the binding
so that scheduled rebalances do not fire on local holidays
unless explicitly overridden.

## 4.16 Backwards compatibility (continued)

Custodian APIs predating envelope-based settlement MAY expose a
translator that consumes the custodian's native message format
(SWIFT MT54x, FIX, custodian-proprietary) and emits
`settlement_confirmation` envelopes. The translator is operated by
the advisor or a trusted third party; the custodian itself is not
required to upgrade for this standard to be useful.

## 4.17 Worked example: end-to-end onboarding

Day 0 — Investor signs up. Advisor emits `investor_profile` with
risk tolerance, capacity, and constraints. Investor counter-signs
the consent.

Day 1 — Advisor selects an algorithm and emits
`algorithm_attestation`. Custodian binding established via signed
`custody_binding`.

Day 2 — Initial portfolio constructed; advisor emits `portfolio`
and the corresponding `rebalance` for opening trades. Custodian
counter-signs `settlement_confirmation` at T+2.

Day 30 — First quarterly fee disclosure emitted as
`fee_disclosure`. Aggregator A's federation handshake completes;
investor counter-signs the consent. Aggregator A begins receiving
read-only `portfolio` envelopes.

Day 90 — Tax-loss harvesting opportunity detected. Advisor emits
`tax_loss_harvest` envelope; replacement holding acquired
respecting the wash-sale window. Custodian counter-signs
settlements at T+2.

Day 365 — Year-end. Advisor emits `tax_year_summary` for the
investor's tax residence. Investor downloads the bundle, signs the
disclosure, and forwards to their tax preparer. Regulatory bundle
emitted to FSC / SEC / FCA per applicable jurisdiction.

The above flow involves 0 phone calls, 0 paper documents, and 0
re-keyed data. Every transition is signed; every signature is
verifiable; every artefact is reproducible from the envelope
stream.

## 4.18 Cross-reference summary

This Phase ties together the envelope set: Phase 1 defines the
envelopes; Phase 2 defines how to publish, query, and stream them;
Phase 3 defines how to share them across organisations under
investor consent; Phase 4 defines how to plug them into the
business-to-business systems (custodians, market data providers,
regulators, tax preparers) that turn the envelopes into the
day-to-day operations of an investment advisory business.

The total surface is small enough that a competent engineering
team can implement a Federated-Read-Write conforming advisor in
roughly six engineer-months of focused work, including custodian
integration. The standard's brevity is deliberate; every field
that exists has been earned, and every field that does not exist
has been deliberately excluded to keep the implementation cost
proportional to the value delivered.

## 4.19 Final implementer guidance

Begin with Phase 1 envelope generation and Phase 2 read endpoints.
Treat federation, regulatory bundle generation, and ISO 20022
linkage as Phase 4 work that follows after the read path is solid.
Resist the temptation to skip directly to federation features
before the underlying envelope discipline is exercised in
production; aggregator-facing endpoints amplify any defects in
envelope generation by the size of the federation.

弘益人間 — Benefit All Humanity. The standard exists so that the smallest organisation can sign one envelope and be heard by the largest.
