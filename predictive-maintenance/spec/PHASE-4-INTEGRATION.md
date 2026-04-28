# WIA-predictive-maintenance PHASE 4 — Integration Specification

**Standard:** WIA-predictive-maintenance (WIA-IND-026)
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-predictive-maintenance
integrates with adjacent ecosystems (CMMS / EAM,
ERP, CMMS-vendor SaaS, OEM service platforms,
process-safety regimes, and downstream WIA
standards), how conformance evidence is produced,
and how deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012 (Conformity assessment)
- ISO/IEC 27001:2022, ISO/IEC 27017, ISO/IEC 27701:2019
- ISO 55000 / 55001 / 55002, ISO 14224:2016, ISO 13374, ISO 13381-1
- ISO 17359:2018, ISO 18436-1..-8, ISO 10816 / 20816
- IEC 62443 series, IEC 61508, IEC 61511
- IEC 62541 (OPC UA), IEC 63278-1 (Asset Administration Shell)
- ISA-95 / IEC 62264, ISA-88
- MTConnect Specification 2.4, MQTT v5.0, AMQP 1.0
- W3C VC 2.0, W3C SHACL

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-predictive-maintenance
consumes upstream specifications, how downstream
WIA standards reference predictive-maintenance
artefacts, and how conformance is assessed.

## §2 Upstream integration

### 2.1 ISO TC 108 (Vibration)

ISO 10816 / 20816 vibration severity classes feed
into the §5 condition record. ISO 13373-3
threshold tables underpin the alert mapping.

### 2.2 ISO TC 108 / SC 5 (Condition monitoring)

ISO 13374-1..-7 condition-monitoring data
processing layers map directly to the WIA-pdm
condition / anomaly / prognosis record set.

### 2.3 ISO TC 251 (Asset management)

ISO 55001 asset-management policy frames the
operator's strategic context. Maintenance orders
align with ISO 55001 §6.2 objectives.

### 2.4 OPC Foundation / MTConnect / IEC

OPC UA Companion Specifications (Pumps, Robotics,
Machinery, Process Automation Devices) are the
canonical field-level model. MTConnect provides
the same model for discrete-machining shop floors.

### 2.5 IEC TC 65 (Industrial cybersecurity)

IEC 62443-3-3 system requirements and IEC 62443-4-1
secure development lifecycle gate the deployment's
allowable conduit and zone configurations.

## §3 CMMS / EAM integration

CMMS / EAM systems (SAP PM, IBM Maximo, IFS,
Infor, Oracle Fusion, Sovereign-equivalent) consume
order and completion records via the bulk export
endpoint or via direct webhook subscription. Order
acceptance posts back to the CMMS so that the
record system of truth remains the CMMS.

## §4 ERP integration

ERP systems consume parts-list and completion
records for stock and labour cost tracking. ERP
binding is informative; ISA-95 / IEC 62264 provides
the canonical hierarchy.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared ISO 14224 taxonomy version;
- declared ISO 13374 / 13381 framework versions;
- declared OPC UA / MTConnect Companion versions;
- the test-vector matrix per Annex G of each
  PHASE;
- the JWS signing key set used.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. ISO 14224 failure-mode codes are correctly
   applied across anomaly records.
2. ISO 13381 prognostic framework references match
   the model card.
3. Calibration cadence is honoured.
4. IEC 62443 zone segregation is preserved at
   conduit boundaries.
5. CMMS / EAM round-trip is reproducible.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                  | Integration point                      |
|---------------------------|----------------------------------------|
| WIA-energy-management     | per-asset energy KPIs                  |
| WIA-supply-chain          | spare-parts logistics                  |
| WIA-cybersecurity-industrial | IEC 62443 zone binding              |
| WIA-language-bridge       | multilingual work orders               |
| WIA-learning-analytics    | technician training analytics          |

## §7 Privacy

Personal data of technicians (identity,
qualifications, working hours) is processed under
the operator's privacy regime.

## §8 Security

The registry operates a vulnerability-disclosure
programme aligned with ISO/IEC 30111. OEM signing
key compromise triggers immediate JWKS rotation.

## §9 Process-safety boundary

Records derived from safety-critical zones (SIL-
rated, Seveso-tier installations) are gated by
the operator's process-safety regime. The registry
does not override the regime; instead, it carries
zone metadata so that downstream consumers can
reason about the boundary.

## §10 Localisation

Maintenance order narratives are localised in BCP
47 form. Failure-mode codes remain in the ISO 14224
canonical form across locales.

## §11 Accessibility

Operator-facing dashboards conform to WCAG 2.2 AA.
Field-mobile interfaces support voice prompts and
large-font modes for technicians wearing PPE.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence
URL from the README.

## Annex B — Worked completion record (informative)

```json
{
  "completionRef": "f63f4f04-...",
  "orderRef": "order-072",
  "closedAt": "2026-04-28T13:30:00+09:00",
  "failureModeCode": "FOF-BRG",
  "replacedParts": [
    {"partNumber": "BRG-6307-2RS", "serial": "12345"}
  ]
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with ISO 14224 / 13374 / 13381 major
revisions.

## Annex D — Open governance

Issues at
`github.com/WIA-Official/wia-standards/issues` with
the `predictive-maintenance` label.

## Annex E — Withdrawal procedure

Tombstone the evidence package; tombstones are
immutable.

## Annex F — Reproducibility

Evidence is reproducible from publicly available
inputs: ISO taxonomy versions, OPC UA Companion
versions, calibration registry, signing key set.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative vector.

## Annex H — Sustainability

Operators SHOULD declare their estimated energy
savings from condition-based vs preventive
maintenance using ISO 50001 methodology. The
declaration is informative.

## Annex I — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| OEM signing key compromise            | Immediate JWKS rotation   |
| Calibration drift undetected          | Cadence enforcement       |
| ISA-95 hierarchy mis-mapping          | Audit against asset graph |
| IEC 62443 conduit breach              | Zone segregation gate     |
| Anomaly false-positive flood          | Confidence-threshold tuning|
| Spare-parts mis-binding               | EAN / GTIN match check    |

## Annex J — Industry binding catalogue

| Industry segment   | Bound profile                           |
|--------------------|-----------------------------------------|
| Pumps              | OPC UA Companion for Pumps              |
| Robotics           | OPC UA Companion for Robotics           |
| Discrete machining | MTConnect 2.4                           |
| Process            | OPC UA Companion for Process Automation |
| Wind / renewable   | IEC 61400-25 (informative)              |
| Aerospace MRO      | ATA Spec 2000 (informative)             |

## Annex K — OEM service platform

OEM service platforms (Siemens MindSphere, GE
Predix, ABB Ability, etc.) integrate via the
federation contract; OEM platforms surface as
peer registries in the discovery graph.

## Annex L — Continuous improvement programme

Each deployment publishes an annual improvement
plan addressing false-positive reduction,
calibration cadence, and CMMS round-trip latency.

## Annex M — Operator personnel qualification

Personnel performing condition monitoring carry
ISO 18436-1..-8 qualification certificates. The
operator's HR system catalogues the
qualifications; the registry references the
catalogue when an order assigns a technician.

## Annex N — Reference implementation

Reference implementations are published under
Apache-2.0 at the WIA Standards GitHub umbrella
under `wia-pdm-reference`, covering the OPC UA
ingest adapter, the anomaly engine skeleton, and
the CMMS callback adapter.

## Annex O — Vendor neutrality

WIA-predictive-maintenance does not endorse a
particular CMMS / EAM vendor. The conformance
programme is open to commercial, open-source, and
sovereign-developed implementations on identical
terms.

## Annex P — Annual ecosystem report

The registry publishes an annual ecosystem report
summarising asset coverage, sensor density, anomaly
detection rates, prognostic accuracy benchmarks,
and CMMS round-trip latencies across registered
deployments. The report is informative.

## Annex Q — Continuous fairness audit for ML models

Predictive maintenance models that incorporate
machine learning declare a fairness audit URL on
the model card. Audits cover unequal performance
across asset families, manufacturing batches, or
operating regions. Audits renew annually.

## Annex R — Researcher access programme

Operators may make anonymised reliability data
available to academic researchers under a
documented data-use agreement. The agreement is
catalogued at `/v1/registry/researcher-access`.

## Annex S — Disaster recovery

Registry deployments declare RPO ≤ 24h and RTO ≤
8h targets. DR drills run annually with results
in the audit feed.

## Annex T — Industry binding catalogue

| Industry segment       | Bound profile                          |
|------------------------|----------------------------------------|
| Wind turbine           | IEC 61400-25 + ISO 14224               |
| Aviation MRO           | ATA Spec 2000                          |
| Rail rolling stock     | UIC Code 615 / IEC 61133 (informative) |
| Marine                 | IACS UR-Z23 (informative)              |
| Oil & Gas              | API 692 / API 691 (informative)        |
| Power generation       | IEEE C37 (informative)                 |
| Process industries     | IEC 61511 SIL                          |
| Pharmaceutical         | GAMP 5 (informative)                   |

## Annex U — Open-source SDK catalogue

Reference SDKs are published per language and
runtime (Python, Java, Go, .NET, Rust) at
`/v1/registry/sdks`. SDKs simplify integration
against the WIA-pdm contract under permissive
open-source licenses.

## Annex V — Vendor certification programme

Vendors apply to the certification programme by
submitting their evidence package and undergoing
auditor review at the declared tier. Certified
vendors are catalogued at
`/v1/registry/certified-vendors` with their tier,
audit window, and supported asset taxonomies.

## Annex W — Workforce analytics

Aggregate technician workload, qualification mix,
and order completion rates feed into the
operator's workforce analytics dashboard. The
dashboard honours WCAG 2.2 AA and respects per-
technician privacy by aggregating to crew-level
counters by default.

## Annex X — Open governance forum

A quarterly open governance forum is held online
where operators, OEMs, and auditors discuss
emerging needs and proposed errata. Forum minutes
are published alongside release notes.

弘益人間 (Hongik Ingan) — Benefit All Humanity
