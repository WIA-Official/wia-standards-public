# WIA-automated-trading PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-automated-trading
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer
for WIA-automated-trading. The standard covers
persistent record shapes for the lifecycle of
algorithmic, automated, and direct-electronic-access
order flow at an investment firm or trading venue —
the firm and its registered persons; the algorithm
inventory and its certification record; the order's
parent-and-child decomposition; the venue routing and
execution record; the pre-trade and at-trade risk-
control activations; the kill-switch and circuit-
breaker engagement record; the post-trade transaction
report; the surveillance alert record; and the
operational-resilience and conformance-test artefact
record. Records are consumed by the firm's algorithmic
trading governance committee, the firm's compliance
and risk function, the trading venue (regulated
market, MTF, OTF, ATS, exchange) for member-level
discipline, the supervisory authority for the operating
jurisdiction's market-conduct examination (ESMA and
the Member-State NCA in the EU; SEC, FINRA, and CFTC
in the US; KR FSC and FSS), the venue's market-
surveillance function, and external auditors.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO 4217 (currency codes), ISO 3166-1 (country
  codes), ISO 9362 (BIC), ISO 17442 (LEI)
- ISO 20022 (financial industry message scheme;
  cited for post-trade and reference-data messaging)
- ISO 10383 (Market Identifier Code, MIC; cited for
  venue-level identification)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- FIX 5.0 SP2 + FIX FAST + FIXatdl 1.1 + FIX Orchestra
  (the operating wire-format for order entry and
  market-data dissemination at most regulated venues)
- EU MiFID II (Directive 2014/65/EU) Article 17
  (algorithmic trading), Article 18 (operating
  conditions for trading venues), Article 47
  (organisational requirements for trading venues),
  Article 48 (systems resilience, circuit breakers
  and electronic trading), Article 49 (tick sizes),
  Article 50 (synchronisation of business clocks),
  Article 51 (admission of financial instruments to
  trading)
- EU MiFIR (Regulation (EU) 600/2014) Articles 26
  (transaction reporting) and 27 (reference data)
- EU Commission Delegated Regulation (EU) 2017/589
  RTS 6 (organisational requirements of investment
  firms engaged in algorithmic trading)
- EU Commission Delegated Regulation (EU) 2017/584
  RTS 7 (organisational requirements of trading
  venues for systems resilience, circuit breakers,
  electronic trading and tick-size regimes)
- EU MAR (Regulation (EU) 596/2014) Articles 12, 14,
  15, 16, 18 (market-abuse offences and
  surveillance)
- EU MiCA (Regulation (EU) 2023/1114) for crypto-
  asset trading-platform operators where applicable
- US SEC Reg SCI (17 CFR Part 242, Rules 1000–1006)
  for SCI entities operating critical trading
  systems
- US SEC Rule 15c3-5 (market-access rule, the
  broker-dealer pre-trade risk-control rule)
- US SEC Reg ATS (17 CFR Part 242, Rules 300–303)
  for alternative trading systems
- US SEC Reg NMS (17 CFR Part 242, Rules 600–612)
- US FINRA Rule 3110 (supervisory system) and
  Rule 5210 (publication of transactions and quotations)
- US CFTC 17 CFR 1.81 (futures commission merchant
  risk management)
- KR 자본시장법 (Financial Investment Services and
  Capital Markets Act) and KRX (한국거래소) market-
  surveillance rules including the 자동화 매매
  시스템 점검 discipline

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
an automated-trading operator (an investment firm,
a broker-dealer, a futures commission merchant, a
proprietary-trading firm, or a trading venue)
maintains:

- The firm-and-registered-persons record.
- The algorithm inventory and certification record.
- The order-and-execution record (parent-and-child).
- The venue-routing and best-execution record.
- The pre-trade-risk-control engagement record.
- The kill-switch and circuit-breaker engagement
  record.
- The transaction report (MiFIR Article 26 and US
  CAT-equivalent).
- The market-abuse surveillance alert record.
- The conformance-test and disaster-recovery drill
  record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("investment-firm" |
                       "broker-dealer" |
                       "proprietary-trading-firm" |
                       "futures-commission-merchant"
                       | "trading-venue-rm" |
                       "trading-venue-mtf" |
                       "trading-venue-otf" |
                       "trading-venue-ats" |
                       "trading-venue-exchange" |
                       "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
operatorBic          : string (ISO 9362; absent for
                       non-SWIFT-connected operators)
operatorLei          : string (ISO 17442)
governingFrameworks  : array of enum ("EU-MIFID-II" |
                       "EU-MIFIR" | "EU-MIFID-II-RTS-6"
                       | "EU-MIFID-II-RTS-7" |
                       "EU-MAR" | "EU-MICA" |
                       "US-SEC-REG-SCI" |
                       "US-SEC-RULE-15C3-5" |
                       "US-SEC-REG-ATS" |
                       "US-SEC-REG-NMS" |
                       "US-FINRA-RULE-3110" |
                       "US-FINRA-RULE-5210" |
                       "US-CFTC-17-CFR-1-81" |
                       "KR-자본시장법" |
                       "KR-KRX-자동화-점검" |
                       "ISO-20022" | "FIX-5-0-SP2"
                       | "user-defined")
assetClassesTraded   : array of enum ("equity" |
                       "fixed-income" | "fx" |
                       "listed-derivatives" | "otc-
                       derivatives" | "etf" |
                       "structured-product" |
                       "crypto-asset" | "user-
                       defined")
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Algorithm Inventory and Certification Record

The algorithm inventory aligns with MiFID II RTS 6
Article 1 (general organisational requirements) and
the SEC Reg SCI definition of "SCI systems":

```
algorithmRecord:
  algorithmId        : string (uuidv7; the firm's
                       internal algorithm identifier)
  algorithmName      : string
  algorithmKind      : enum ("execution-algorithm" |
                       "smart-order-router" |
                       "market-making" | "arbitrage"
                       | "trend-following" |
                       "mean-reversion" |
                       "statistical-arbitrage" |
                       "liquidity-seeking" |
                       "iceberg-pegged" |
                       "user-defined")
  inputParameters    : array of object (the parameter
                       set the algorithm accepts at
                       order initiation)
  riskControlsRef    : array of string (PHASE-1 §6
                       reference for each pre-trade
                       and at-trade control bound to
                       the algorithm)
  certificationStatus : enum ("development" |
                       "stress-tested" |
                       "conformance-passed" |
                       "production-approved" |
                       "rolled-back" | "retired")
  conformanceTestRef : string (URI of the venue
                       conformance-test report; MiFID
                       II RTS 6 Article 5 requires
                       venues to provide a testing
                       environment)
  approvedBy         : string (the firm's algorithmic-
                       trading governance committee
                       reference)
  approvedAt         : string (ISO 8601)
  retiredAt          : string (ISO 8601; absent until
                       retired)
```

## §4 Registered-Person and Direct-Electronic-Access
       Record

```
registeredPerson:
  personId           : string (uuidv7)
  jurisdictionalLicence : array of object (FINRA
                       CRD identifier in the US; ESMA
                       MiFID-II registered-person
                       identifier in the EU; KR
                       금융투자전문인력 등록 in KR)
  algorithmsCovered  : array of string (algorithm
                       references the person is
                       authorised to deploy)
  supervisorRef      : string (the FINRA Rule 3110
                       supervisor; absent in non-US
                       jurisdictions)

directElectronicAccessClient:
  clientId           : string (uuidv7)
  clientLei          : string (ISO 17442)
  accessKind         : enum ("direct-electronic-access
                       -dea-mifid-2" | "sponsored-
                       access-sec" | "naked-access-
                       prohibited" | "user-defined")
  preTradeRiskRef    : string (PHASE-1 §6 reference;
                       SEC Rule 15c3-5 mandates
                       broker-dealer pre-trade
                       controls; MiFID II Article
                       17(5) DEA controls)
  agreementRef       : string (URI of the DEA / SA
                       client agreement)
```

## §5 Order and Execution Record

```
orderRecord:
  orderId            : string (uuidv7)
  parentOrderRef     : string (uuidv7; absent for
                       parent orders)
  algorithmRef       : string (PHASE-1 §3)
  registeredPersonRef : string (PHASE-1 §4)
  clientOrderId      : string (FIX ClOrdID 11=)
  symbolRef          : object (ISIN + MIC + FIX
                       SecurityID + venue listing
                       reference)
  orderSide          : enum ("buy" | "sell" | "sell-
                       short" | "buy-cover")
  orderTimeInForce   : enum ("day" | "gtc" | "ioc"
                       | "fok" | "gtd" | "moc" |
                       "loc" | "user-defined")
  orderType          : enum ("market" | "limit" |
                       "stop" | "stop-limit" |
                       "pegged" | "iceberg" |
                       "user-defined")
  orderQuantity      : object (UCUM-equivalent
                       quantified)
  limitPrice         : object (currency-quantified;
                       absent for market orders)
  submittedAt        : string (ISO 8601 with at-least
                       millisecond precision; MiFID II
                       Article 50 / RTS 25 requires
                       UTC and clock-synchronisation
                       on the firm's business clocks)
  venueRoutedTo      : string (ISO 10383 MIC)

executionRecord:
  executionId        : string (uuidv7)
  orderRef           : string
  executedQuantity   : object
  executedPrice      : object
  venueExecutionId   : string (the venue's published
                       execution identifier)
  executedAt         : string (ISO 8601 millisecond)
  liquidityFlag      : enum ("added" | "removed" |
                       "auction" | "cross" | "user-
                       defined")
  feeStructure       : array of object (per-fee-line
                       reference; venue maker-taker
                       schedule)
```

## §6 Pre-Trade and At-Trade Risk-Control Record

The risk-control record encodes the SEC Rule 15c3-5
broker-dealer pre-trade controls and the MiFID II
RTS 6 Articles 9 to 11 organisational risk-control
requirements:

```
riskControlRecord:
  controlId          : string (uuidv7)
  controlKind        : enum ("max-order-size" |
                       "max-order-value" |
                       "fat-finger-price-collar" |
                       "max-aggregate-position" |
                       "max-credit-exposure" |
                       "max-message-rate" |
                       "duplicate-order-check" |
                       "wash-trade-block" |
                       "spoofing-pattern-block" |
                       "self-trade-prevention" |
                       "regulatory-restricted-list-
                       block" | "user-defined")
  thresholdValue     : object (the configured
                       threshold)
  scope              : enum ("per-algorithm" |
                       "per-trader" | "per-client" |
                       "per-firm" | "per-venue" |
                       "per-symbol")
  activatedAt        : string (ISO 8601)
  lastBreachAt       : string (ISO 8601; absent if
                       never breached)
  breachActionRef    : string (the breach-handling
                       record reference)
```

## §7 Kill-Switch and Circuit-Breaker Engagement Record

```
killSwitchEngagement:
  engagementId       : string (uuidv7)
  triggerKind        : enum ("manual-trader" |
                       "manual-supervisor" |
                       "manual-risk-officer" |
                       "automated-risk-breach" |
                       "venue-suspended-symbol" |
                       "venue-circuit-breaker" |
                       "exchange-trading-halt" |
                       "user-defined")
  scope              : enum ("single-algorithm" |
                       "single-venue-connection" |
                       "single-symbol" | "single-
                       trader" | "firm-wide" |
                       "user-defined")
  engagedAt          : string (ISO 8601)
  releasedAt         : string (ISO 8601; absent
                       until released)
  rationaleRef       : string (URI of the rationale
                       narrative)
```

## §8 Transaction Report Record (MiFIR Article 26 +
       US CAT-equivalent)

```
transactionReport:
  reportId           : string (uuidv7)
  executionRef       : string
  reportingRegime    : enum ("eu-mifir-art-26" |
                       "us-sec-rule-613-cat" |
                       "us-finra-trace" |
                       "us-finra-oats-superseded" |
                       "kr-자본시장법-거래보고" |
                       "user-defined")
  reportedAt         : string (ISO 8601; MiFIR
                       Article 26(1) requires
                       reporting "as quickly as
                       possible, and no later than
                       the close of the following
                       working day")
  arrangedRoutedExecuted : enum ("arranged" |
                       "routed" | "executed")
  shortSaleIndicator : enum ("short" | "short-
                       exempt" | "long" | "n/a")
  waiverIndicator    : array of string (the MiFID II
                       pre-trade transparency waivers
                       claimed)
  reportingDestination : string (ARM, APA, or the
                       supervisor's transaction
                       reporting endpoint)
```

## §9 Market-Abuse Surveillance Alert Record

The surveillance discipline aligns with EU MAR
Articles 12 (market manipulation), 14 (insider
dealing), 15 (market manipulation prohibited),
16 (suspicious-transaction-and-order reporting), and
18 (insider list); FINRA Rule 6140 + Rule 5210; and
the KR 자본시장법 시세조종·미공개정보 이용 enforcement.

```
surveillanceAlert:
  alertId            : string (uuidv7)
  detectedAt         : string (ISO 8601)
  patternKind        : enum ("layering-spoofing" |
                       "wash-trading" | "marking-
                       the-close" | "ramping" |
                       "insider-trading-window" |
                       "front-running" | "trade-
                       through" | "quote-stuffing"
                       | "momentum-ignition" |
                       "user-defined")
  evidenceRef        : string (URI of the
                       evidence package — order
                       book reconstruction, message
                       audit, communications
                       review)
  reviewerRef        : string (the surveillance
                       analyst)
  outcomeKind        : enum ("closed-no-action" |
                       "closed-coaching" |
                       "escalated-compliance" |
                       "stor-filed-mar-art-16" |
                       "sar-filed-fincen" |
                       "self-reported-supervisor"
                       | "user-defined")
  storRef            : string (URI of the
                       suspicious-transaction-and-
                       order report filed with the
                       Member-State NCA per MAR
                       Article 16; absent unless
                       filed)
```

## §10 Conformance-Test and Resilience-Drill Record

```
conformanceTestRecord:
  testId             : string (uuidv7)
  algorithmRef       : string
  venueRef           : string (the venue's
                       conformance-test environment)
  testKind           : enum ("venue-conformance-
                       test" | "self-test-stress" |
                       "regression" | "back-test"
                       | "production-shadow" |
                       "user-defined")
  startedAt          : string (ISO 8601)
  completedAt        : string (ISO 8601)
  outcomeKind        : enum ("passed" | "failed" |
                       "passed-with-conditions")
  reportRef          : string (URI of the test
                       report)

resilienceDrill:
  drillId            : string (uuidv7)
  drillKind          : enum ("dr-failover" |
                       "kill-switch-exercise" |
                       "venue-disconnection" |
                       "data-feed-failover" |
                       "circuit-breaker-test" |
                       "tabletop-incident" |
                       "user-defined")
  conductedAt        : string (ISO 8601)
  rtoMet             : boolean (the drill's actual
                       recovery-time matched the
                       firm's documented RTO)
  rpoMet             : boolean
  reportRef          : string
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
each of the records defined above for every deployed
algorithm and every order, preserve the order-and-
execution and transaction-report records under the
operating jurisdiction's record-keeping discipline
(MiFID II RTS 6 Article 8 + RTS 25 five years; SEC
Rule 17a-4 retention; FINRA Rule 4511 retention; KR
자본시장법 보존 의무), exercise venue conformance
testing before promoting an algorithm to production,
and report market-abuse alerts to the operating
jurisdiction's supervisory authority within the
statutory window.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-automated-trading
- **Last Updated:** 2026-04-28
