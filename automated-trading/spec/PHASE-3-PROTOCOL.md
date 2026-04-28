# WIA-automated-trading PHASE 3 — PROTOCOL Specification

**Standard:** WIA-automated-trading
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
automated-trading operator: the algorithmic-trading
governance discipline (MiFID II Article 17 + RTS 6
organisational requirements; SEC Reg SCI policies
and procedures; FINRA Rule 3110 supervisory system);
the pre-trade-and-at-trade risk-control discipline
(SEC Rule 15c3-5; MiFID II RTS 6 Articles 9 to 11;
CFTC 17 CFR 1.81); the conformance-testing discipline
that gates production deployment of an algorithm
(MiFID II RTS 6 Article 5; venue conformance-test
agreements); the time-and-order-record-keeping
discipline (MiFID II Article 50 / RTS 25 clock
synchronisation; SEC Rule 17a-4 retention); the kill-
switch discipline; the market-abuse surveillance
discipline (EU MAR; FINRA Rule 6140 + Rule 5210;
KR 자본시장법 시세조종·미공개정보 enforcement); and
the operational-resilience discipline (Reg SCI;
MiFID II RTS 7; DORA Regulation (EU) 2022/2554 for
ICT-third-party risk management).

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO 20022, ISO 4217, ISO 3166-1, ISO 9362, ISO
  10383 MIC, ISO 17442 LEI
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)
- FIX 5.0 SP2 + FIX FAST + FIX Orchestra
- EU MiFID II Articles 17, 18, 47, 48, 49, 50, 51
- EU MiFIR Articles 26, 27
- EU Commission Delegated Regulation (EU) 2017/589
  RTS 6 (algorithmic-trading organisational
  requirements)
- EU Commission Delegated Regulation (EU) 2017/584
  RTS 7 (trading-venue resilience)
- EU Commission Delegated Regulation (EU) 2017/574
  RTS 25 (clock-synchronisation)
- EU MAR Articles 12, 14, 15, 16, 18
- EU DORA (Regulation (EU) 2022/2554) ICT-third-
  party-risk management, incident reporting, threat-
  led penetration testing, resilience testing
- US SEC Reg SCI (17 CFR Part 242, Rules 1000–1006)
- US SEC Rule 15c3-5 (market-access rule)
- US SEC Reg ATS (17 CFR Part 242, Rules 300–303)
- US SEC Reg NMS (17 CFR Part 242, Rules 600–612)
- US SEC Rule 17a-4 (broker-dealer retention)
- US SEC Rule 613 (Consolidated Audit Trail)
- US FINRA Rule 3110, 4511, 5210, 6140
- US CFTC 17 CFR 1.81; CFTC Reg AT discontinued and
  replaced by 17 CFR 38, 40 risk-control discipline
- KR 자본시장법 + KRX 시장감시 + 자동화 매매 시스템
  점검 + 한국거래소 회원규정

---

## §1 Algorithmic-Trading Governance Discipline

The governance discipline aligns with MiFID II
Article 17 and RTS 6 Articles 1 to 4:

- The firm maintains an algorithmic-trading
  governance committee with documented
  responsibilities and the authority to approve,
  pause, or retire algorithms.
- Every algorithm is registered in the inventory
  (PHASE-1 §3) with the algorithm name, kind, input
  parameters, risk controls, and certification
  status before production deployment.
- Every algorithm change (parameter change, code
  change, model-data refresh) is reviewed under the
  firm's change-management discipline; material
  changes trigger re-certification.
- Annual self-assessment under MiFID II RTS 6
  Article 9 — the firm reviews the operating
  performance of its algorithmic-trading
  organisation and reports the self-assessment to
  the Member-State NCA.

For SCI entities under SEC Reg SCI the discipline
adds: SCI policies-and-procedures coverage of SCI
systems and SCI security systems; SCI events
reporting (Rule 1002 24-hour notification, Rule
1003 disclosure to members); annual SCI review;
table-top exercises.

## §2 Pre-Trade and At-Trade Risk-Control Discipline

The risk-control lifecycle:

1. The firm defines per-algorithm and firm-wide
   pre-trade controls — max-order-size, max-order-
   value, fat-finger price-collar, max-aggregate-
   position, max-credit-exposure, max-message-rate.
2. Every order entering production passes through
   the controls before reaching the venue (SEC Rule
   15c3-5(c)(1)(i) regulatory and credit-exposure
   controls; MiFID II RTS 6 Article 11(1)(a)
   pre-trade controls; CFTC 17 CFR 1.81(a)).
3. Threshold breaches trigger immediate order
   rejection, an audit-event recording, and
   (depending on severity) the kill-switch.
4. Threshold updates require documented approval
   from the firm's risk function and are recorded
   for examination.
5. The firm performs annual stress tests of the
   risk-control regime under RTS 6 Article 9.

## §3 Conformance-Testing Discipline

Algorithm certification gates production deployment:

- The firm engages the trading venue's conformance-
  test environment (RTS 6 Article 5; venue's
  published conformance-test agreement).
- The conformance test exercises the algorithm
  against the venue's sample-message scenarios
  (rejected, partially-filled, cancelled, error
  paths).
- Stress tests under RTS 6 Article 6 — high-message-
  rate scenarios, latency-degradation scenarios,
  market-data feed-failover scenarios.
- Production-shadow run — the algorithm runs in
  parallel with the incumbent for a defined window
  before promotion.
- The certification report references the test
  outcomes, the residual risks, and the firm's
  governance-committee approval.

## §4 Time-and-Order-Record-Keeping Discipline

MiFID II Article 50 + RTS 25 require business clocks
synchronised with UTC traceable to a national
metrology institute reference; the granularity for
algorithmic trading is 1 microsecond and the maximum
divergence from UTC is 100 microseconds. The firm's
time-discipline records the synchronisation source,
the divergence-monitoring outputs, and the per-event
timestamp emitted for orders, modifications,
cancellations, and executions.

Record-keeping spans:

- MiFID II RTS 6 Article 8 — five-year retention of
  the algorithm description, trading parameters, and
  testing.
- MiFIR Article 26(1) — five-year retention of
  transaction reports.
- SEC Rule 17a-4 — three- or six-year retention
  depending on record kind; SEC Rule 17a-4(f) WORM
  retention discipline applies to electronic
  records.
- FINRA Rule 4511 — six-year retention.
- KR 자본시장법 보존 의무 — KR-jurisdiction retention
  per the FSC's published guidance.

## §5 Kill-Switch Discipline

The kill-switch discipline:

- Trader-engaged — the trader engages the kill switch
  for their own algorithm.
- Supervisor-engaged — the FINRA Rule 3110 supervisor
  engages the kill switch for any algorithm under
  their oversight.
- Risk-officer-engaged — the firm's risk function
  engages the kill switch firm-wide where
  appropriate.
- Auto-engaged — the risk engine engages the kill
  switch when a hard threshold is breached.
- Venue-engaged — the trading venue engages the
  kill switch on its side under RTS 7 Article 18
  emergency-stop authority.

Release follows the four-eyes discipline — the
engaging party cannot single-handedly release; an
independent reviewer verifies remediation before
release.

## §6 Market-Abuse Surveillance Discipline

The surveillance discipline aligns with EU MAR and
the equivalent US / KR regimes:

- Pattern detection — layering, spoofing, wash-
  trading, marking-the-close, ramping, insider-
  trading-window, front-running, trade-through,
  quote-stuffing, momentum-ignition.
- Order-book reconstruction — full depth-of-book
  reconstruction at micro-second granularity from
  the order-and-execution record.
- Communications review — chat / email / voice
  surveillance for the registered persons covered
  by MAR Article 16(2) market-soundings discipline
  and MAR Article 14 insider-dealing discipline.
- Suspicious-transaction-and-order reporting (STOR)
  — the firm's compliance officer files the STOR
  to the Member-State NCA without delay (MAR
  Article 16); FINRA Rule 6140 / 5210 prompts; KR
  자본시장법 self-reporting.
- Tipping-off discipline — the firm does not
  disclose the STOR to the customer or to a third
  party.

## §7 Operational-Resilience Discipline

The operational-resilience discipline aligns with
SEC Reg SCI for SCI entities and with EU DORA for
EU-regulated firms:

- ICT-third-party-risk management — the firm maps
  its critical ICT third-party providers, evaluates
  concentration risk, and contracts the resilience
  obligations into the third-party agreement (DORA
  Articles 28 to 30).
- Incident reporting — major ICT incidents are
  reported to the Member-State NCA within DORA
  Article 19 timeframes; SCI events are notified
  to the SEC under Rule 1002 (immediate notice for
  systems disruption / intrusion / compliance issue)
  and reported under Rule 1003.
- Threat-led penetration testing — TIBER-EU
  framework testing on the operator's cadence
  (DORA Article 26).
- Resilience testing — annual digital-operational-
  resilience testing programme (DORA Articles 24
  to 27).
- Disaster recovery — RTO and RPO targets aligned
  with the operating jurisdiction's expectations;
  drill cadence documented.

## §8 Direct-Electronic-Access Discipline

For DEA / sponsored-access clients (MiFID II Article
17(5); SEC Rule 15c3-5 sponsored-access controls):

- Naked access is prohibited under SEC Rule 15c3-5;
  every DEA / SA order passes through the broker-
  dealer's pre-trade risk controls before reaching
  the venue.
- Client agreements documenting the allowed-and-
  prohibited algorithm list, the client's pre-trade
  risk parameters, and the client's compliance
  attestations.
- Annual review — the firm reviews each DEA / SA
  client's continued eligibility; clients with
  patterns inconsistent with their documented
  trading purpose are escalated for review.

## §9 Identity, Time and Audit Discipline

NTPv4 stratum-2 or better feeds the firm's clock
discipline; UTC traceability per RTS 25. Audit-
events are emitted for every order entry, modify,
cancel, execution, kill-switch engagement, risk-
control breach, surveillance alert, conformance
test, resilience drill, and DEA / SA client agreement
update. Audit logs are integrity-protected per the
operator's tamper-evident mechanism; SEC Rule 17a-
4(f) WORM discipline applies to electronic records.

## §10 Crypto-Asset Trading Venue Discipline

For crypto-asset trading venues operating under EU
MiCA Regulation (EU) 2023/1114:

- The crypto-asset trading platform's authorisation
  and operating-rules discipline apply (MiCA Title
  V).
- The white-paper-based admission discipline applies
  for asset-referenced and e-money tokens (MiCA
  Title III + IV).
- Market-abuse rules under MiCA Title VI extend the
  EU MAR discipline to crypto-assets.

## §11 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
maintain the algorithm inventory and certification
record, exercise the pre-trade risk controls before
every venue-bound order, satisfy the time-discipline
under RTS 25, exercise the kill-switch with four-
eyes discipline, file STORs to the Member-State NCA
within the statutory window, and exercise the
operational-resilience programme on the published
cadence.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-automated-trading
- **Last Updated:** 2026-04-28
