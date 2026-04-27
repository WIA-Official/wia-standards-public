# WIA-longevity-gene PHASE 2 — API Interface Specification

**Standard:** WIA-longevity-gene
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the HTTP API surface a longevity-gene boundary
exposes to clinical-genomics laboratories, ageing-research cohorts,
biobanks, clinical-decision-support tools, regulator audit
clients, and the patient-facing apps that surface ageing-related
genomic insight to consented individuals. The shape is FHIR R5
RESTful with the Genomics Reporting profile, layered with PRS,
biological-age, and telomere-length operations.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Genomics Reporting IG, RESTful API,
  MolecularSequence, Observation, DiagnosticReport, Bundle,
  Subscription, OperationOutcome, CapabilityStatement
- HL7 SMART App Launch 2.2
- GA4GH VRS Service API (refget for reference sequences)
- GA4GH Beacon v2 (where the deployment participates in
  variant-discovery networks)
- IETF RFC 9457 (Problem Details), RFC 8615 (well-known URIs),
  RFC 7515 (JWS), RFC 7519 (JWT), RFC 8259 (JSON), RFC 3339

---

## §1 Capability discovery

```
GET /metadata HTTP/1.1
Accept: application/fhir+json
```

FHIR CapabilityStatement augmented with WIA extensions:
declared genome assemblies, supported PRS model versions,
supported biological-age clock versions, telomere-length
methods, and Beacon-participation declaration.

```
GET /.well-known/wia/longevity-gene HTTP/1.1
```

Returns deployment policy summary: cohort references,
laboratory accreditations (CLIA, CAP, ISO 15189), genetic-
counsellor availability, return-of-results policy.

## §2 Variant interpretation lifecycle

```
POST /Observation HTTP/1.1
Content-Type: application/fhir+json
WIA-Purpose-Of-Use: HRESCH or TREAT or HOPERAT
```

Body is a variant interpretation per PHASE 1 §3. Boundary
verifies:

1. Active consent for genetic-data class
2. ACMG/AMP criteria are present and consistent
3. VRS identifier canonicalises to a known variant or is
   admitted to the deployment's variant registry
4. Subject's declared ancestry is compatible with PRS model
   if a PRS observation accompanies the interpretation

```
PUT /Observation/<id>/$reinterpret HTTP/1.1
```

Re-interpretation creates a new observation linked to the
prior; the prior remains version-history-preserved.

## §3 Polygenic-risk score operations

```
POST /Observation HTTP/1.1
GET  /Patient/<self>/$prs-summary?modelRef=<URN>
POST /PRSModel/<modelRef>/$validate HTTP/1.1
```

`$validate` allows pre-computation validation: a PRS model
with mismatched population reference returns
`urn:wia:long:problem:prs-population-mismatch` with the
disclaimer text the boundary attaches to results.

## §4 Biological-age operations

```
POST /Observation HTTP/1.1
GET  /Patient/<self>/$biological-age-trajectory
POST /BioAgeModel/<modelRef>/$qc-evaluate HTTP/1.1
```

The trajectory operation returns longitudinal bio-age values
with each declared clock model version, supporting clinical
review of acceleration trends.

## §5 Telomere-length operations

```
POST /Observation HTTP/1.1
GET  /Patient/<self>/$telomere-trajectory
```

## §6 Phenopacket linkage

```
POST /Phenopacket HTTP/1.1
GET  /Patient/<self>/Phenopacket/<id>
PUT  /Phenopacket/<id>/$amend HTTP/1.1
```

The boundary stores Phenopackets per GA4GH Phenopacket Schema
2.0; each Observation cross-references the relevant Phenopacket.

## §7 Search filtering and minimum necessary

```
GET /Observation?subject=<subjectRef>&category=genomics&code=53037-8
```

Search results are filtered to consented subset. Genetic-
information consent is exceptionally granular (variant-class
opt-in, return-of-results opt-in per condition class) so the
boundary may filter individual observations even within a
single subject's record.

## §8 Error model

| URI                                                       | Status | Meaning                                            |
|-----------------------------------------------------------|-------:|----------------------------------------------------|
| `urn:wia:long:problem:acmg-criteria-required`             | 422    | ACMG/AMP criteria absent on interpretation         |
| `urn:wia:long:problem:vrs-canonicalisation-failed`        | 422    | VRS identifier could not be canonicalised          |
| `urn:wia:long:problem:prs-population-mismatch`            | 200    | Accepted with mandatory disclaimer                 |
| `urn:wia:long:problem:bio-age-qc-required`                | 422    | Bio-age QC metrics absent                          |
| `urn:wia:long:problem:telomere-method-required`           | 422    | Telomere-length method reference missing           |
| `urn:wia:long:problem:no-active-consent`                  | 403    | No genetic-data consent matches purpose            |
| `urn:wia:long:problem:return-of-results-not-authorised`   | 403    | Subject has not authorised return of this category |

## §9 Bulk export

```
GET /$export?_type=Observation,Phenopacket&purpose=HRESCH&consentBundle=<id>
```

Genomic bulk exports respect granular consent: each
observation in the export must be authorised for the
declared purpose; aggregated PRS or bio-age summaries
require their own consent class.

## §10 Beacon v2 participation

Where the deployment participates in a variant-discovery
network (e.g., GA4GH Beacon v2), the boundary exposes a
Beacon-style endpoint:

```
GET /beacon/v2/variants?gene=APOE&assemblyId=GRCh38 HTTP/1.1
```

Beacon responses report variant *presence* in the cohort
without disclosing subject identity. Network-policy
restricts which gene panels are queryable; some categories
(non-clinically-actionable, conditions of stigma concern)
may be excluded by deployment policy.

## §11 Subject self-service

```
GET  /Patient/<self>/$genomic-summary
POST /Patient/<self>/$opt-in-result-class
PUT  /Patient/<self>/$revoke-consent-class
```

Subjects manage their granular consent for return-of-results
per category (medically actionable, carrier status,
pharmacogenomic, ancestry, ageing-relevant). The boundary
honours the most-recent consent on subsequent requests.

## §12 Subscriptions for clinical-decision-support

CDS tools subscribe to medically-actionable interpretations
in their patient cohort:

```
POST /Subscription HTTP/1.1
{
  "topic": "https://wia.example/SubscriptionTopic/longevity-medically-actionable",
  "channel": {"type": "rest-hook", "endpoint": "https://cds.example/wia-webhook"}
}
```

Delivery uses TLS 1.3 with detached JWS in `Wia-Signature`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked PRS validation (informative)

```
POST /PRSModel/urn:wia:long:prs-model:PGS001234:v1.0/$validate HTTP/1.1
{
  "subjectAncestry": "EAS"
}
```

Response if model was trained on EUR cohort:

```
200 OK
{
  "compatibility": "limited",
  "disclaimer": "Model PGS001234 v1.0 was trained on participants of European ancestry. Risk estimation in subjects of East Asian ancestry has not been validated and may be biased. Use with clinical judgment.",
  "alternativeModels": [{"modelRef": "urn:wia:long:prs-model:KOREA-GENE-PRS-CAD:v2.0", "compatibility": "validated"}]
}
```

## Annex B — Pagination and rate limiting (informative)

List endpoints paginate at ≤ 100 observations per page (smaller
than sibling standards because each observation is a substantial
genomic record). Per-token rate limits default to 10
interpretation submissions per minute; bulk-import for
laboratory data uses a separate batch endpoint with declared
batch size.

## Annex C — Conformance disclosure

Sections §1, §2, §3, §4, §6, §7, §11 are mandatory. §5 (telomere)
is mandatory if the deployment provides telomere measurement.
§9 (bulk export) is mandatory where the deployment shares with
research warehouses. §10 (Beacon) is mandatory where the
deployment participates in a discovery network. §12
(subscriptions) is mandatory where CDS integration is offered.

## Annex D — Worked Beacon v2 query (informative)

```
GET /beacon/v2/g_variants?geneId=APOE&assemblyId=GRCh38 HTTP/1.1
Authorization: Bearer <researcher-jwt>
Accept: application/json
```

```json
{
  "responseSummary": {"exists": true, "numTotalResults": 142},
  "response": {
    "resultSets": [
      {
        "id": "longevity-cohort-1",
        "type": "dataset",
        "exists": true,
        "resultsCount": 142,
        "results": []
      }
    ]
  },
  "info": {
    "disclaimerNote": "Results aggregated across cohort; subject identities not disclosed."
  }
}
```

## Annex E — Genetic-counselling integration

Genetic-counsellor scheduling integrates with the deployment's
clinical scheduling system:

```
POST /Patient/<self>/$schedule-genetic-counsellor HTTP/1.1
{
  "preferredChannel": "video",
  "preferredWindow": {"start": "2026-05-01T09:00:00+09:00", "end": "2026-05-01T17:00:00+09:00"},
  "topicCategory": "actionable-finding-return"
}
```

The counsellor app receives the request, schedules a
session, and delivers a calendar invite to the subject.

## Annex F — Granular consent worked example (informative)

A subject's genetic-data consent might enable:

- `medically-actionable` (ACMG SF v3.2 list) — return ON
- `carrier-status` — return ON
- `pharmacogenomic` — return ON
- `ancestry` — return OFF (subject does not want)
- `ageing-relevant` — return ON
- `non-clinically-actionable-research-only` — return OFF
- `psychiatric-condition-stigma` — return OFF (subject opt-out)

Each category emission is gated independently. A change in
the ACMG SF list of actionable genes triggers a re-consent
prompt because the subject's prior consent referenced an
older catalogue.

## Annex G — Idempotency

All write endpoints accept `Idempotency-Key`. The boundary
stores keys for 30 days. Replays return the original
response. Different body with same key returns
`urn:wia:long:problem:idempotency-conflict` (409).

## Annex H — Capability cache freshness

Capability documents include a `fresh-until` timestamp;
clients refresh on or before that timestamp. Critical
changes (PRS model recalibration, ACMG SF update, return-
of-results policy change) trigger push notification to
subscribed clients so cache invalidates eagerly.

## Annex I — Phenopacket bundle worked example (informative)

```json
{
  "resourceType": "Bundle",
  "type": "collection",
  "entry": [
    {
      "resource": {
        "resourceType": "Phenopacket",
        "id": "urn:wia:long:phenopacket:p-91a7",
        "subject": {"id": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e", "ageAtCollection": "P67Y"},
        "phenotypicFeatures": [
          {"type": {"id": "HP:0011463", "label": "Childhood-onset short stature"}, "excluded": true},
          {"type": {"id": "HP:0001640", "label": "Cardiomegaly"}, "modifiers": [{"id": "HP:0012828", "label": "Severe"}]}
        ],
        "diseases": [
          {"term": {"id": "MONDO:0007020", "label": "Familial hypertrophic cardiomyopathy"}, "onset": {"age": "P54Y"}}
        ],
        "interpretations": [{"id": "i-1", "diagnosis": {"genomicInterpretations": [{"variantInterpretation": {"variant": "urn:wia:long:variant:..."}}]}}]
      }
    }
  ]
}
```

## Annex J — Conformance level discovery

```
GET /.well-known/wia/longevity-gene/conformance HTTP/1.1
```

Returns the deployment's declared level (Surface / Verified /
Anchored), most recent audit date, and auditor signature.
Anchored deployments include the inclusion-proof URI for
the audit-chain witness that downstream regulators rely on.
