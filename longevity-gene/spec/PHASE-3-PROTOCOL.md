# WIA-longevity-gene PHASE 3 — Protocol Specification

**Standard:** WIA-longevity-gene
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding data formats
(PHASE 1) and API surface (PHASE 2) to operational exchanges:
authentication of clinicians, geneticists, laboratory
principals, biobank staff, and patients; clinical-laboratory
data ingestion (BAM/VCF/CRAM); reference-sequence resolution
via GA4GH refget; QC verification per ISO/TS 22692; biobank
identifier discipline; time discipline; audit-chain
construction.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7525, RFC 9162 (CT pattern)
- IETF RFC 7515 (JWS), RFC 7519 (JWT), RFC 8705 (mTLS-bound JWT)
- GA4GH Refget v2 — reference-sequence retrieval
- GA4GH htsget v1.3 — secure data streaming for genomic files
- GA4GH Data Repository Service (DRS) v1.4
- GA4GH Passports / Visa specification — for cross-organisation
  research-cohort access
- ISO 20691:2022 — NGS data formats
- ISO/TS 22692:2020 — NGS quality control
- WHO Genome standards reference

---

## §1 Authentication

Principals authenticate via JWS-signed JWTs:

- `iss`, `sub`, `aud`, `iat`, `exp`
- `wia.role` — `clinician`, `geneticist`, `genetic-counsellor`,
  `lab-staff`, `biobank-curator`, `patient`, `researcher`,
  `regulator`, `auditor`
- `wia.holderRef` — for clinician/lab tokens, the organisation
  whose subjects/samples may be accessed
- `wia.researchCohortRefs[]` — for researcher tokens, the
  cohorts the principal is authorised within
- `cnf` — confirmation claim binding to TLS client certificate

Researchers may carry GA4GH Passport visas alongside the JWT
for cross-organisation cohort access; the boundary verifies
the visa signatures against the issuing authority's published
keys.

## §2 Laboratory data ingestion

Clinical laboratories submit sequencing-derived results via:

- **BAM/CRAM file** — alignment data; the boundary stores via
  GA4GH DRS-style addressable references and exposes htsget
  for secure streaming to authorised analysts
- **VCF file** — variant calls; the boundary indexes for
  variant-level lookup and cross-references to public
  registries (ClinVar, dbSNP)
- **Methylation array file** — for epigenetic-clock
  computation; the boundary normalises per the deployment's
  preprocessing pipeline (background subtraction,
  normalisation, batch effect correction)

```
POST /Laboratory/<labRef>/$ingest HTTP/1.1
Authorization: Bearer <lab-jwt>
Content-Type: application/json

{
  "sampleRef": "urn:wia:long:sample:lab-001:s-91a7",
  "subjectRef": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e",
  "files": [
    {"kind": "VCF", "drsRef": "drs://lab-001/v-91a7.vcf.gz"},
    {"kind": "BAM", "drsRef": "drs://lab-001/b-91a7.bam"}
  ],
  "qcReport": "drs://lab-001/qc-91a7.json"
}
```

The boundary verifies:

1. The submitting laboratory has accreditation valid for the
   sample type (CLIA, CAP, ISO 15189, KOLAS-accredited
   medical laboratory, etc.)
2. The QC report meets ISO/TS 22692 acceptance thresholds
3. Sample chain-of-custody is intact

## §3 Reference-sequence resolution (refget)

The boundary operates a refget v2 endpoint or proxies one
upstream:

```
GET /sequence/<refget-id>?start=<n>&end=<m> HTTP/1.1
Accept: text/vnd.ga4gh.refget.v2.0+plain
```

Reference sequences for supported assemblies (GRCh38.p14,
GRCh37.p13, T2T-CHM13v2.0) are addressable; the deployment
caches popular subsequences locally for performance.

## §4 Variant canonicalisation

The boundary canonicalises submitted variants via VRS:

1. Parse HGVS or VCF representation
2. Resolve to genomic coordinates against the declared assembly
3. Normalise the variant (left-align, trim shared bases)
4. Compute the VRS deterministic identifier
5. Look up against existing records; merge if found

Canonicalisation failure (ambiguous representation, invalid
coordinates) returns
`urn:wia:long:problem:vrs-canonicalisation-failed`.

## §5 QC discipline

Each ingested sample carries a QC report following ISO/TS
22692:

- Per-sample: mean coverage depth, % bases ≥ Q30,
  contamination estimate, sex check vs. self-declared
- Per-variant-call: GATK VQSR or DeepVariant filter status,
  call quality, allele balance
- Per-array: bisulfite conversion efficiency,
  detection-p-value rate, sample mismatch rate

Samples failing QC are flagged in the boundary's record;
downstream interpretation refuses to proceed without an
explicit override signed by the laboratory director.

## §6 Biobank identifier discipline

Biobank samples carry double-pseudonymous identifiers:

- `biobankSampleRef` — opaque biobank-internal reference
- `boundaryReleaseId` — the boundary's mapping to a study-
  specific identifier visible to researchers

The mapping is held in a separate mapping store accessible
only to biobank curators and the deployment's identity
broker. Researchers see only the release-id; re-identification
requires a linkage authorisation per WIA-medical-data-privacy
PHASE 1 §5.

## §7 Time discipline

Boundary clock: NTPv4 stratum-2. Laboratory clocks: stratum-3
or better. Acquisition timestamps on samples are critical
for longitudinal studies; the boundary records both the
laboratory-declared timestamp and the boundary-receive
timestamp on every record so retrospective analysis can
detect lag.

## §8 Audit chain

Every interpretation, PRS computation, bio-age computation,
telomere measurement, biobank linkage, and Beacon-network
release emits an AuditEvent. Chain construction follows the
same Merkle pattern as WIA-medical-iot PHASE 3 §5.

For high-volume biobank deployments, the chain is sharded
by `biobankSampleRef` hash prefix.

## §9 Replay protection

All write endpoints accept `Idempotency-Key`; the boundary
stores keys for 30 days. Bulk laboratory ingestion uses
sample-id-based idempotency: a re-submission with the same
sample-id returns the original ingestion record.

## §10 Disaster recovery

Boundary outage during laboratory ingestion: laboratories
buffer locally and retry on reconnection. Each laboratory
has a deployment-declared retention of buffered samples
sufficient to bridge typical outages.

For the audit chain, recovery follows the
WIA-medical-data-privacy PHASE 3 disaster-recovery pattern;
gaps are flagged for manual review.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Algorithm choices (informative)

| Concern              | Default                            | Notes                                       |
|----------------------|------------------------------------|---------------------------------------------|
| Token signing        | ES256                              | mTLS-bound (RFC 8705)                       |
| TLS                  | 1.3 (RFC 8446)                     | hybrid groups when WIA-pq-crypto hybrid     |
| Audit hash           | SHA-256                            |                                             |
| File integrity       | SHA-256 + DRS metadata             | matches GA4GH DRS v1.4                       |

## Annex B — Worked variant ingestion (informative)

```
1. Lab submits BAM + VCF + QC bundle via $ingest
2. Boundary verifies lab accreditation status (CLIA active)
3. Boundary verifies QC bundle meets ISO/TS 22692 (mean coverage 32.5x, %Q30 = 91%)
4. Boundary parses VCF; canonicalises 1247 variants via VRS
5. Boundary cross-references each variant against ClinVar
6. Variants in genes on the deployment's longevity panel are
   flagged for genetic-counsellor review
7. Counsellor finalises interpretations with ACMG/AMP criteria
8. Subject-side consent is checked for return-of-results
   on the relevant categories
9. Authorised results are released to the subject's app
10. Audit chain captures every step
```

## Annex C — Negative-test vectors (informative)

| Stimulus                                                  | Expected outcome                                    |
|-----------------------------------------------------------|-----------------------------------------------------|
| Lab-ingest from non-accredited lab                         | refused with `lab-not-accredited`                   |
| QC report missing required ISO/TS 22692 fields             | refused with `qc-incomplete`                        |
| VRS canonicalisation failure                               | refused with `vrs-canonicalisation-failed`          |
| Subject-side consent revoked between ingest and release    | release blocked; record retained but redacted      |
| Beacon query for excluded gene panel                       | returned with `beacon-gene-excluded`                |

## Annex D — htsget streaming (informative)

For high-fidelity data access, the boundary supports htsget
v1.3:

```
GET /htsget/v1/reads/<drsRef>?referenceName=chr19&start=44900000&end=45100000 HTTP/1.1
```

The htsget protocol returns a manifest of byte-range URLs;
the client fetches only the relevant region. Boundary records
the access in the audit chain so retrospective review can
confirm research-cohort access patterns.

## Annex E — GA4GH Passport visa verification (informative)

When a researcher presents a GA4GH Passport JWT alongside
their primary auth JWT:

1. Boundary extracts the Passport claims
2. For each Visa, the boundary fetches the issuing
   organisation's published JWKS
3. Verifies the visa signature
4. Verifies the visa's expiry has not passed
5. Verifies the visa's authorised research-cohort scope
   matches the requested cohort

Visa-verification failures are recorded as audit events;
persistent verification failure on a single issuer triggers
a partner-roster review.

## Annex F — Methylation-array preprocessing pipeline

Standard methylation-array preprocessing per the deployment:

1. Background subtraction (Noob method or equivalent)
2. Quantile normalisation across samples
3. Cell-type composition adjustment (Houseman or IDOL)
4. Batch-effect correction (ComBat) if multiple plates
5. Detection-p-value filtering (probes with p > 0.01 set
   to missing)
6. Cross-reactive probe removal per published lists
7. SNP-affected probe removal where required by clock model

The pipeline configuration is recorded with each bio-age
observation so retrospective re-analysis is possible.

## Annex G — DRS metadata shape

```json
{
  "id": "drs://lab-001/v-91a7.vcf.gz",
  "name": "v-91a7.vcf.gz",
  "size": 1452890234,
  "checksums": [
    {"type": "sha-256", "checksum": "..."},
    {"type": "md5", "checksum": "..."}
  ],
  "self_uri": "drs://lab-001/v-91a7.vcf.gz",
  "version": "1",
  "created_time": "2026-04-28T10:00:00Z",
  "updated_time": "2026-04-28T10:00:00Z",
  "access_methods": [
    {"type": "https", "access_url": {"url": "https://lab-001.example/files/v-91a7.vcf.gz"}}
  ]
}
```

The boundary verifies the SHA-256 checksum on every retrieval
to detect tampering or transmission corruption.

## Annex H — Batch-effect detection (informative)

For methylation-array deployments processing samples on
multiple plates over time:

1. Boundary ingests batch metadata with each sample
2. Periodic batch-effect analysis runs against accumulated
   samples
3. Significant batch effects trigger ComBat correction; the
   correction parameters are recorded with each sample
4. Cross-batch comparisons require the corrected values
5. Quarterly compliance report tracks batch-effect cumulative
   distortion

## Annex I — Boundary-clock health (informative)

The boundary publishes clock-health metrics in the capability
discovery document:

- Primary time source (NTPv4 stratum-2 server reference)
- Secondary time source
- Most recent successful sync timestamp
- Current drift estimate (microseconds)
- Continuous-monitoring status

Partners verify boundary-clock health before accepting
time-sensitive products (longitudinal biological-age trends,
sample chain-of-custody).

## Annex J — Refget caching policy (informative)

The boundary caches reference-sequence subsequences locally
to reduce upstream traffic and improve client latency:

- Cache key = `<assembly-id>:<refget-id>:<start>:<end>`
- TTL = 30 days (reference sequences are immutable)
- Eviction policy = LRU when cache exceeds deployment-declared
  storage budget
- On cache miss, the boundary fetches from the canonical
  source and verifies the integrity hash before caching

A cache-integrity verification failure forces the cached
copy to be evicted; the boundary records the event for
operations review.
