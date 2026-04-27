# WIA-longevity-gene PHASE 4 — Integration Specification

**Standard:** WIA-longevity-gene
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how a longevity-gene deployment integrates
the data, APIs, and protocols of PHASEs 1–3 with broader
operational systems: clinical laboratories, biobanks, ageing-
research cohorts, EHR genomics tabs, clinical-decision-support,
patient-facing apps, regulator audit pipelines, GA4GH discovery
networks, pharmaceutical-industry research partners, and
genetic-counselling services.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 Genomics Reporting IG, Subscriptions, Bulk Data
- GA4GH Beacon v2, Passports, Phenopackets, VRS, DRS, htsget
- ClinVar, dbSNP, ClinGen actionability framework
- ACMG SF v3.2 — secondary-findings reportable list
- ISO 13485, ISO 14971, ISO 20691, ISO/TS 22692
- US GINA, EU GDPR Article 9, K-PIPA 유전정보 특별보호
- WIA-medical-data-privacy, WIA-medical-iot, WIA-medication-adherence,
  WIA-network-security, WIA-pq-crypto

---

## §1 EHR integration

EHR systems consume genomic data via:

- **FHIR subscription** — for medically-actionable
  interpretations (per ACMG SF v3.2 reportable list); the
  EHR's genomic-actionable-results worklist receives them
- **FHIR bulk export** — for population-genomics analytics
- **FHIR DiagnosticReport** — for finalised genomic reports
  delivered to the EHR's results inbox

Genomic data inherits the EHR's most-restrictive PHI
controls; EHR access requires a genomic-data role.

## §2 Clinical-laboratory integration

Clinical laboratories integrate via:

- Sample submission with structured request forms
- Laboratory information system (LIS) FHIR or HL7 v2
  ORU^R01 messages
- BAM/VCF/methylation-array file delivery via DRS
- Laboratory accreditation evidence stored in the
  boundary's accreditation registry
- Periodic QC summary reports per ISO/TS 22692

The boundary refreshes laboratory accreditation status
daily; expired accreditations block new ingestion.

## §3 Biobank integration

Biobanks integrate via:

- Sample inventory exchange with double-pseudonymous identifiers
- Phenopacket exchange for phenotypic context
- Research-cohort aggregation with linkage authorisations
- Specimen tracking through aliquot lifecycle

Cross-biobank research-cohort access uses GA4GH Passports;
the boundary verifies passport visa signatures against
each issuing biobank.

## §4 Research-cohort access

Researchers access cohort data via:

- Beacon v2 for variant-discovery queries
- htsget for selective sequencing data streaming
- Bulk export for full-cohort analysis (with explicit
  research authorisation)
- Phenopacket bundles for phenotype-aware analyses

Each access emits an audit event; cohort-level summary of
access patterns is shared with the cohort's governance
committee on a documented cadence.

## §5 Patient-facing app integration

The patient-facing app surfaces:

- Variant interpretations the subject has consented to receive
- PRS results with population-context disclaimers
- Bio-age trajectories
- Telomere-length trajectories
- Educational resources tied to specific findings
- Genetic-counsellor scheduling for return-of-results
  conversations

Apps are SMART-launched against the deployment's identity
provider. Subject self-service emits AuditEvents to detect
impersonation patterns.

## §6 Regulator audit integration

Regulators receive structured exports for:

- Clinical-laboratory accreditation compliance
- Adverse-event reporting (e.g., misinterpreted variant
  leading to clinical harm)
- Genetic-information privacy compliance per jurisdiction
- Aggregate population-level findings reporting where
  required (e.g., national newborn-screening programmes)

## §7 Pharmaceutical post-market surveillance

Pharmaceutical companies engaged in longevity-related drug
development receive:

- Aggregate PRS distributions for trial-eligibility
  population characterisation
- Bio-age effect-size summaries from registry studies
  (where consented for industry research purpose)
- Adverse-event correlation with genomic context

Aggregation thresholds prevent re-identification: cohorts
fewer than 50 patients are suppressed (more conservative
than other domains because genomic data is exceptionally
re-identifiable).

## §8 Operational SLAs

| Concern                                          | Default SLA                |
|--------------------------------------------------|----------------------------|
| Variant interpretation submission acceptance p95 | ≤ 2 s                       |
| Laboratory ingest acceptance p95                 | ≤ 60 s for ≤ 100 MB submission |
| EHR FHIR subscription delivery p95               | ≤ 5 s                       |
| Bulk export for ≤ 10K observations               | ≤ 60 s                      |
| Beacon query response p95                        | ≤ 1 s                       |
| Reference-sequence retrieval p95                 | ≤ 500 ms                    |
| Audit chain entry available                       | ≤ 10 s                      |

## §9 Quarterly compliance report

The boundary emits a quarterly compliance report covering:

- Laboratory accreditation status across the partner roster
- Sample ingestion volumes per laboratory
- Variant interpretation throughput
- ACMG/AMP classification distribution
- PRS / bio-age / telomere observation rates
- Beacon-network query metrics
- Cross-domain references honoured vs. refused
- Audit-chain integrity check results

## §10 Acceptance criteria

A deployment claims conformance when:

1. Laboratory accreditation roster is current
2. Every sample in the past quarter has a matching audit
   chain entry with verifiable inclusion proof
3. Genetic-information consent is granular and current
4. EHR integration delivers within SLA across at least 95%
   of medically-actionable interpretations
5. Quarterly compliance report has no integrity-check
   failures
6. Cross-domain references resolve at the partner boundary
   for ≥ 99% of bound observations

## §11 Common pitfalls (informative)

- **PRS ancestry mismatch** — most published PRS models are
  trained on European-ancestry cohorts; deployments serving
  non-European populations SHOULD prefer ancestry-matched
  models or apply explicit disclaimers
- **VRS canonicalisation drift** — VRS spec evolves; the
  deployment SHOULD track VRS version updates and re-canonicalise
  legacy records on minor-version updates
- **Bisulfite conversion variability** — methylation arrays
  show batch effects; the deployment SHOULD track batch ID and
  apply ComBat-style correction
- **Telomere method variability** — qPCR T/S ratio is not
  comparable across labs; the deployment SHOULD prefer
  internally-consistent methods within a longitudinal study
- **Return-of-results category drift** — ACMG SF reportable
  list updates periodically; the deployment SHOULD review
  consent-class definitions on each ACMG SF update

## §12 Decommissioning

When a deployment is decommissioned:

1. Active interpretations transferred to receiving deployment
2. Biobank samples retained per biobank's retention policy
3. EHR integration wound down with downstream sign-off
4. Patient-facing app data retained per consent scope
5. Audit chain sealed and final root published
6. Regulator notification filed if required

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table (informative)

| Reference                  | Use site                                                  | Gate applied                                         |
|----------------------------|-----------------------------------------------------------|------------------------------------------------------|
| WIA-medical-data-privacy   | every record references a genetic-data consent class      | medical-side consent + genetic-data category         |
| WIA-medical-iot            | wearables that contribute to bio-age inputs               | medical-iot device association                       |
| WIA-medication-adherence   | longitudinal pharmacological exposures                    | medication-side consent                              |
| WIA-network-security       | TLS cipher-suite floor                                    | network-security-side floor on each session          |
| WIA-pq-crypto              | post-quantum migration phase                              | pq-crypto-side phase declaration current             |

## Annex B — ACMG SF v3.2 actionable-list update workflow (informative)

When ACMG publishes an SF v3.x update:

1. Geneticist team reviews the new actionable list
2. Updates the deployment's return-of-results category
   definitions
3. Subjects with active consent for actionable findings
   receive a notification and re-consent prompt
4. Researchers with active cohort access receive notification
   of the catalogue change
5. Quarterly compliance report tracks the update cycle
   completion across the subject base

## Annex C — Decommissioning checklist (informative)

- [ ] Active interpretations transferred
- [ ] Biobank samples retained per policy
- [ ] EHR integration wound down
- [ ] Patient-facing app data retained per consent
- [ ] Audit chain sealed
- [ ] Regulator notification filed

## Annex D — Conformance disclosure

Sections §1, §2, §3, §5, §8, §10 are mandatory. §4 (research-
cohort) is mandatory if the deployment hosts research data.
§6, §7 are mandatory where the corresponding flow is offered.

## Annex E — Biobank linkage worked example (informative)

A research consortium queries linked biobank samples:

1. Researcher submits a cohort definition (age range, sex,
   phenotype filters) to the boundary
2. Boundary computes the cohort across consented samples
3. Boundary issues study-specific release identifiers
4. Researcher queries variant data via Beacon or htsget
   using release identifiers
5. Boundary records each query in the audit chain
6. Cohort governance committee receives quarterly
   access-pattern summary

Re-identification is impossible to the researcher unless
the biobank curator approves a linkage authorisation
(WIA-medical-data-privacy PHASE 1 §5).

## Annex F — Population-genomics reference cohort (informative)

The deployment maintains references to well-curated
population-genomics cohorts for PRS recalibration:

| Population reference        | Coverage                              |
|-----------------------------|---------------------------------------|
| 1000 Genomes Project        | Global (26 populations)               |
| gnomAD v4.x                 | Global with ancestry stratification   |
| UK Biobank                  | UK population (predominantly EUR)     |
| KOREA-1K / Genome Korea     | Korean population                     |
| J-MICC / Tohoku Medical Megabank | Japanese population              |
| All of Us                   | US diverse-ancestry cohort           |
| H3Africa                    | African-ancestry cohorts             |

PRS models cross-reference their training population; the
boundary refuses to display a score against a non-matching
ancestry without a disclaimer.

## Annex G — ACMG SF reportable-list lifecycle (informative)

Each ACMG SF version (v3.0, v3.1, v3.2, v3.3, ...) updates
the actionable-gene list:

| Version | Release year | Genes added              | Genes removed |
|---------|-------------|--------------------------|---------------|
| v3.0    | 2021        | Initial ACMG SF v3       | -             |
| v3.1    | 2022        | TMEM127, MAX             | -             |
| v3.2    | 2023        | ATP7B, MFAP5             | -             |
| (later) | (TBD)       | (per ClinGen WG)         | (per ClinGen WG) |

The boundary tracks the version per consent and per
interpretation; subjects with a v3.0-era consent receive
re-consent prompts when v3.2 adds new genes that fall
within their consent class.

## Annex H — Decommissioning checklist (informative)

- [ ] Active interpretations transferred or archived
- [ ] Biobank samples retained per biobank policy
- [ ] EHR integration wound down
- [ ] Patient app data preserved per consent
- [ ] Audit chain sealed
- [ ] Regulator notification filed if required
- [ ] GA4GH discovery network membership withdrawn

An anchored conformance level is preferred for deployments serving clinical genomics at scale.
