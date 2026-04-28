# WIA-inventory-management PHASE 3 — PROTOCOL Specification

**Standard:** WIA-inventory-management
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an inventory-
management programme: GS1 GTIN / GLN / SSCC stewardship,
EPCIS 2.0 / CBV 2.0 vocabulary discipline, lot- and serial-
control workflows for regulated products, cold-chain
governance, hazardous-materials segregation, controlled-
substances accountability, cycle-count cadence, physical-
inventory governance, and end-of-programme decommissioning.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 22000:2018 (food safety management)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17025:2017 (calibration laboratories)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- GS1 General Specifications
- GS1 EPCIS 2.0 / CBV 2.0
- 21 CFR Part 11 (US FDA electronic records and signatures)
- 21 CFR Part 211 (US FDA cGMP for finished pharmaceuticals)
- 21 CFR Part 1300-1316 (US DEA controlled-substance
  accountability)
- EU GDP (Good Distribution Practice for medicinal products,
  2013/C 343/01)
- IATA Dangerous Goods Regulations
- USP <1079> Good Storage and Distribution Practices
- HACCP (Codex Alimentarius CXC 1-1969 Rev. 2020) for food
  safety hazard analysis

---

## §1 GS1 Identifier Stewardship

The operator obtains and maintains GS1 GTIN, GLN, SSCC, GRAI,
and GIAI assignments per the operating jurisdiction's GS1
member organisation rules. Identifier stewardship discipline:

- per-product GTIN assignment follows the GS1 GTIN
  Allocation Rules (significant change requires new GTIN);
- per-site GLN assignment follows the operator's site
  hierarchy and is published in the GS1 Global Location
  Registry where the operator opts in;
- per-shipping-unit SSCC assignment is unique and never
  reused for the GS1-recommended period;
- per-asset GRAI / GIAI assignment supports the operator's
  reusable-transport-item or instance-specific tracking
  needs.

Identifier withdrawals (item discontinuation, site closure)
follow GS1's recommended decommission window before reuse.

## §2 EPCIS 2.0 / CBV 2.0 Vocabulary Discipline

Movement events (PHASE-1 §7) use the GS1 CBV 2.0 vocabulary
for `bizStep` and `disposition`. The operator's per-programme
vocabulary profile records:

- the subset of CBV 2.0 `bizStep` values the operator emits;
- the subset of `disposition` values the operator emits;
- per-event-kind validation rules (e.g. `AggregationEvent`
  requires `parentEpc`; `TransformationEvent` requires both
  input and output EPC sets);
- the per-source-system mapping from the source system's
  internal vocabulary to CBV 2.0.

Vocabulary extensions beyond CBV 2.0 require explicit
operator profile registration; ad-hoc vocabulary terms in
emitted events are rejected at the API gate.

## §3 Lot- and Serial-Control Workflow

Lot-controlled and serial-controlled items follow the
operator's regulated-product workflow:

- per-lot manufacturer / supplier record at receipt with
  manufacturer GLN and country of origin;
- per-lot QA release certificate ingestion before stock
  becomes available for sale (the API gates the disposition
  transition);
- per-serial-number aggregation under SSCC at packaging
  for downstream chain-of-custody;
- per-serial warranty registration at shipment to the
  end customer.

Recall workflows trigger movement events with
`disposition=recalled` against affected lots and serials,
plus operator-driven communications to affected trading
partners and end customers.

## §4 Cold-Chain Governance

Cold-chain items (PHASE-1 §3 `coldChainProfile` non-
ambient) are governed by the operator's cold-chain SOP:

- per-shipment temperature monitoring through trip data
  loggers (USP <1079>-aligned excursion thresholds);
- per-receiving-event cold-chain integrity verification
  before put-away into temperature-controlled locations;
- per-storage-location continuous monitoring with mapped
  alarm thresholds and excursion-response procedures;
- per-shipment temperature-history attachment to the
  outbound shipping record.

Excursions trigger the operator's quality-deviation workflow;
affected stock may be quarantined under
`disposition=non_sellable_other` pending QA disposition.

## §5 Hazardous-Materials Segregation

Items classified as hazardous (PHASE-1 §3
`hazardClassification` UN-class 1-9) are stored and handled
per IATA DGR and the operating jurisdiction's hazardous-
materials regulations. Segregation discipline:

- per-class storage zones with documented separation
  distances per the operator's incompatibility matrix;
- per-class personal-protective-equipment training
  requirements for the warehouse associates with access
  to the zone;
- per-class spill-response procedures with location-marked
  spill kits;
- per-shipment hazardous-materials shipping documentation
  per the carrier's mode-specific requirements (IMDG for
  ocean, IATA DGR for air, ADR for road in Europe, 49 CFR
  for road in the US).

Segregation violations during put-away or picking emit
`urn:wia:inventory-management:hazmat-segregation-violation`
problem responses and halt the affected operation pending
resolution.

## §6 Controlled-Substances Accountability

Controlled-substance items (US DEA Schedule II-V, EU
narcotic-and-psychotropic-substances equivalents, KFDA
controlled-substance categories) are subject to per-
jurisdiction accountability rules:

- per-substance per-warehouse perpetual inventory ledger
  with reconciliation on every receipt, transfer,
  dispense, and destruction;
- per-jurisdiction reporting cadence (DEA ARCOS in the US,
  EU competent authority filings, KFDA quarterly reporting
  in Korea);
- per-substance physical-security requirements (locked
  cage, dual-control access, surveillance);
- per-discrepancy investigation procedure with regulator
  notification when discrepancy exceeds the regulator's
  threshold.

The operator's quality dossier records the per-jurisdiction
licence references and the per-substance ledger reconciliation
cadence.

## §7 Cycle-Count Cadence

Cycle counts (PHASE-1 §9 `countKind` cycle variants) follow
the operator's ABC-classification cadence:

- A-class items (top revenue / criticality): counted at
  least monthly;
- B-class items: counted at least quarterly;
- C-class items: counted at least annually.

Counting workflows alternate between blind counts (counter
does not see system quantity) and check counts (counter
verifies system quantity); the operator's count-quality
metric tracks the variance distribution and identifies
counters or zones with anomalous variance patterns.

## §8 Physical-Inventory Governance

Wall-to-wall physical inventory (PHASE-1 §9
`countKind=wall-to-wall-physical`) follows the operator's
physical-inventory SOP:

- per-event preparation review covering the cut-off time,
  the in-transit-inventory snapshot, the unmoved-inventory
  identification, and the per-team count-zone assignments;
- two-pass count discipline (independent counts by two
  associates with a third re-count for variance
  investigation);
- per-event variance approval by the warehouse manager
  and the operator's accounting team before the count
  posts to the financial-inventory ledger.

## §9 Records Retention

Programme records — every item / location / lot / serial /
movement event / receiving / shipping / count record, the
API audit logs, and the regulator submissions — retain per
the operating jurisdiction's commercial-records-retention
rules (typically 7 years for non-regulated, longer for
regulated products such as pharmaceuticals where 21 CFR
Part 211 rules apply).

## §10 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so that
EPCIS event times across multiple sites and partner systems
are consistent. Cold-chain monitoring devices use the
operator's time-sync source so that excursion timestamps
correlate against the WMS's movement-event timestamps.

## §11 Cross-Jurisdictional Operation

Multi-jurisdiction operators (multi-country distribution
networks, cross-border consignment relationships) honour
each jurisdiction's regulator rules: GS1 member-organisation
rules, controlled-substance regulator rules, hazardous-
materials regulator rules, food-safety competent-authority
rules, and pharmaceutical regulator rules. Per-record
governing-jurisdiction tagging supports downstream
regulator-specific reporting.

## §12 Quality Dossier

The operator's quality dossier records the GS1 member-
organisation membership, the per-jurisdiction regulator
licences, the per-class HACCP analyses (for food handling),
the per-class cold-chain SOPs, the per-class hazmat
segregation matrix, the per-warehouse cycle-count cadence,
and the operator's incident history. The dossier is
reviewed at least annually by the operator's quality
manager.

## §13 Recall and Withdrawal Discipline

Product recalls (manufacturer-initiated, regulator-mandated,
operator-initiated quality holds) follow the operator's
recall SOP:

- per-recall classification (Class I imminent health risk,
  Class II potential health risk, Class III non-health-
  risk-related; aligned with the operating jurisdiction's
  recall classification);
- per-recall affected-lot / affected-serial identification
  through the operator's lot-and-serial trace graph;
- per-recall communications to affected trading partners
  and end customers through the channels recorded in the
  partner integration profile;
- per-recall regulator notification per the jurisdiction's
  required cadence;
- per-recall quarantine of affected stock through movement
  events with `disposition=recalled` until the recall is
  closed.

## §14 Mock-Recall Exercises

Operators that handle regulated products conduct mock-recall
exercises at the cadence the regulator requires (typically
annually for pharmaceutical and food operators). Mock
recalls test the operator's trace-graph end-to-end against
a randomly-selected lot or serial, measuring the time-to-
identification and time-to-quarantine. Results are recorded
in the operator's quality dossier.

## §15 Inventory Valuation Discipline

Inventory valuation flows from movement events into the
operator's accounting system per the operator's chosen
costing method (FIFO, weighted average, standard cost,
specific identification for serial-controlled high-value
items). Valuation discipline:

- per-receipt unit-cost capture from the supplier invoice;
- per-issue unit-cost computation per the costing method;
- per-period revaluation when material standard-cost changes
  occur;
- per-period reconciliation between the WMS-side stock
  balance and the financial-inventory ledger.

Reconciliation discrepancies that exceed the operator's
materiality threshold trigger an investigation; root causes
typically resolve to receiving / shipping miscounts,
unposted variances from cycle counts, or in-transit
inventory recognition lag.

## §16 Sustainability and Packaging Stewardship

Operators that publish sustainability disclosures track per-
shipment packaging weights, packaging recyclability classes,
and per-route transportation emissions. Per-class packaging
records support extended-producer-responsibility (EPR)
reporting in jurisdictions where EPR rules apply (EU
PPWR, KR Act on the Promotion of Saving and Recycling of
Resources, equivalent regimes elsewhere).

## §17 Conformance and Auditing

A programme conformant with WIA-inventory-management
publishes its GS1 prefix assignments, its per-jurisdiction
regulator licences, the catalogue of operating sites, and
the per-quarter cycle-count summary, and answers an annual
self-assessment that maps each clause of this PHASE to the
operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-inventory-management
- **Last Updated:** 2026-04-28
