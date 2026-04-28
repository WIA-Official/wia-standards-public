# WIA-inter-korean-data-exchange PHASE 3 — PROTOCOL Specification

**Standard:** WIA-inter-korean-data-exchange
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an authorised
inter-Korean exchange operator: Ministry of Unification
authorisation, sanctions screening against UN Security Council
Resolutions on the DPRK, humanitarian-aid neutrality
discipline, family-reunion administration, liaison-channel
coordination, joint-venture custody, records retention,
political-suspension handling, and operator wind-down.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 17025:2017 (calibration / testing laboratories;
  cited where humanitarian-aid laboratory verification is
  performed)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- UN Security Council Resolutions on the DPRK (cited
  normatively for sanctions-screening references; the operator
  monitors the consolidated list as updated by the relevant
  Sanctions Committee)
- Sphere Standards 2018 (humanitarian charter and minimum
  standards in humanitarian response)
- UN OCHA Common Operational Datasets

---

## §1 Ministry of Unification Authorisation

An operator MAY claim conformance to WIA-inter-korean-data-
exchange only after the South Korean Ministry of Unification
has issued a valid authorisation under the Inter-Korean
Exchange and Cooperation Act for the exchange domains the
operator supports. Authorisations are scoped per domain
(family-reunion, humanitarian-aid, liaison-correspondence,
joint-venture-inventory, cultural-exchange, scholarly-
exchange, sport-exchange) and per shipment for the
humanitarian-aid domain.

Authorisations that the Ministry suspends or revokes
immediately freeze the affected exchange domain; the API
enforces the freeze by refusing to advance any record past
its current status under the suspended domain.

## §2 UN Sanctions Screening

The DPRK is subject to a comprehensive UN sanctions regime.
All cargo lines, all named counterparties, and all financial
flows associated with an exchange artefact are screened
against the UN Security Council Sanctions Committee's
consolidated list and against any applicable south-side
implementation rules.

Screening hits trigger the operator's sanctions workflow:
investigation, Ministry of Unification notification, and
shipment hold pending the Sanctions Committee's exemption
ruling where one is sought. Operators do NOT release
sanctioned goods or sanctioned counterparties under any
operator-internal exception; only the Sanctions Committee's
formal exemption authorises release.

## §3 Humanitarian-Aid Neutrality Discipline

Humanitarian-aid programmes operate under the Sphere
Standards humanitarian charter principles: humanity,
impartiality, neutrality, and independence. Operators record
their adherence in the quality dossier and conduct at least
annual reviews against the Sphere Standards' Core Humanitarian
Standard on Quality and Accountability.

The beneficiary classification recorded in PHASE-1 §5 is
governed by the Sphere Standards' vulnerability framework
(infants under 5, pregnant and lactating, school-age children,
elderly, disaster-affected, mixed). Operators do not direct
aid to non-beneficiary recipients (including counterpart
political organisations) and document distribution evidence
through the north-side counterpart's distribution reports.

## §4 Family-Reunion Administration

Family-reunion lists are administered by the Korea Red Cross
on the south side; the north-side counterpart is the DPRK Red
Cross. Reunion-round scheduling is governed by political-
level negotiations between the two governments and is not
within the operator's scheduling authority.

Reunion-round operating logistics (transport coordination,
venue setup at Mt Kumgang or other agreed venues, video-
conference relay equipment) are recorded against each
reunion round; the API does not arbitrate scheduling but
records the round identifier so that downstream consumers
can correlate operating logistics against the political-
level schedule.

## §5 Liaison-Channel Coordination

The inter-Korean liaison channel (when operating) is the
primary conduit for liaison correspondence (PHASE-1 §6).
Channel availability has been intermittent (Joint Liaison
Office at Kaesong opened 2018, demolished 2020); when the
primary channel is unavailable, operators fall back to the
Panmunjom exchange route or to ministry-direct courier
through third-country conduits.

The active channel per correspondence is recorded; channel
fallback events are recorded as audit events so that
downstream investigators can trace correspondence routes
across periods of channel disruption.

## §6 Joint-Venture Custody

Joint-venture inventories (PHASE-1 §7) are subject to the
joint-venture's bilateral arrangements: who owns what, who
holds custody, and what the wind-down disposition is when the
joint venture is suspended. Operators record the custody
location and the per-line valuation; valuation disputes are
referred to the joint-venture's bilateral dispute-resolution
mechanism, which is recorded in the venture's registration.

Custody locations recorded in PHASE-1 §7 reflect the actual
locations that joint ventures have used: Kaesong Industrial
Complex (closed 2016), Mt Kumgang Tourist Zone (closed 2008),
Rason Special Economic Zone (the few ventures that operated
through China-DPRK conduits), and bonded warehouses on either
side.

## §7 Records Retention

Programme records — every record defined in PHASE-1, the API
audit logs, the Ministry of Unification authorisations, the
sanctions-screening sweep summaries, and the inter-Korean
liaison correspondence — retain per the South Korean
Government Records Management Act and the Ministry of
Unification's records-retention guidance. Family-reunion
records for elderly applicants retain indefinitely so that
descendant requests for ancestral reunion records can be
served decades after the original applicant's passing.

## §8 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) against the
Korea Research Institute of Standards and Science (KRISS)
stratum-1 service (UTC+9 Korea Standards Time). The DPRK
adopted Pyongyang Time (UTC+8:30) between 2015 and 2018, then
re-aligned to UTC+9; correspondence and reunion records
record the active north-side civil time at the time of the
event so that historical records remain interpretable.

## §9 Privacy

Personal identity associated with reunion applicants and
correspondence signatories is held in the operator's CRM under
the South Korean Personal Information Protection Act
(K-PIPA). The DATA-FORMAT layer (PHASE-1) carries only opaque
tokens; the API rejects PII submissions at the body validation
gate. Subject-access requests are honoured through the
operator's K-PIPA-aligned subject-access workflow.

North-side personal identity is, by virtue of the cross-DMZ
isolation, recorded only as an opaque match token; the
operator never holds a north-side data-protection
authorisation and so cannot honour a north-side subject-
access request directly.

## §10 Political-Suspension Handling

Inter-Korean relations are subject to political volatility.
Suspension events (Ministry of Unification suspension orders,
DPRK-side withdrawal from cooperation agreements,
international escalation) trigger the operator's suspension
workflow: in-flight artefacts freeze at their current status,
new artefact authorisations are not requested, and the
operator publishes a suspension notification through its
discovery document so that downstream consumers can pause
their integrations.

Suspension lift events restart the affected exchange domain
under fresh Ministry of Unification authorisations; the
operator records the suspension-and-lift cycle so that
audit consumers can reconstruct the operating timeline
across periods of disruption.

## §11 Quality Dossier

The operator's quality dossier records the Ministry of
Unification authorisations, the Sphere Standards review
outcomes, the sanctions-screening counterpart, the inter-
Korean liaison-channel coordination history, the suspension-
and-lift events, and the operator's incident history. The
dossier is reviewed at least annually by the operator's
quality manager.

## §12 Cross-DMZ Data-Protection Asymmetry

The DPRK does not host a personal-data-protection regime that
the south-side operator can rely on. As a consequence, the
operator's policy is to never carry north-side personal data
across the DMZ on south-side systems beyond what the matched
reunion or correspondence record strictly requires; north-side
identity is recorded only as an opaque match token (PHASE-1
§3) and the operator's south-side records-management workflow
never enriches that token with additional north-side personal
attributes.

Subject-access requests received from north-side individuals
through international intermediaries are referred to the
Ministry of Unification's policy guidance for handling; the
operator does not unilaterally honour the request without
the Ministry's approval.

## §13 Aid-Diversion Risk Management

Humanitarian-aid programmes operate under the recognised risk
that aid may be diverted from the intended beneficiaries by
north-side authorities. Operators record their diversion-risk
mitigation measures in the quality dossier:

- per-shipment beneficiary classification with Sphere-aligned
  vulnerability ranking;
- per-shipment monitoring-and-evaluation arrangements with the
  north-side counterpart (where the bilateral arrangement
  permits monitoring);
- per-programme diversion-incident response procedures.

Confirmed diversion events trigger Ministry of Unification
notification and may suspend the programme until the operator
documents corrective measures acceptable to the Ministry.

## §14 Cultural-Exchange Provenance Discipline

Heritage repatriation programmes (PHASE-1 §8 kind
`heritage-repatriation`) record provenance per artefact: the
artefact's date and place of original creation (where known),
the chain of custodianship through the post-1945 division and
the Korean War, and the south-side custodial history before
the repatriation. Provenance gaps are flagged in the artefact
catalogue and may halt repatriation until the south-side
custodian and the Cultural Heritage Administration agree the
gap is acceptable.

Joint exhibitions of split-custody artefacts are subject to
both the south-side custodian's loan-out conditions and the
north-side custodian's reciprocal commitments; both sets of
conditions are recorded against the exchange.

## §15 Conformance and Auditing

A programme conformant with WIA-inter-korean-data-exchange
publishes its Ministry of Unification authorisations, the
catalogue of exchange domains it supports, the suspension
status of each domain, and the catalogue of reunion rounds
and aid shipments it has supported, and answers an annual
self-assessment that maps each clause of this PHASE to the
operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-inter-korean-data-exchange
- **Last Updated:** 2026-04-28
