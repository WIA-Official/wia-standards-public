# WIA-multiverse-interface PHASE 4 — Integration Specification

**Standard:** WIA-multiverse-interface (WIA-QUA-017)
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-multiverse-interface
integrates with adjacent ecosystems (HPC clusters,
observation arrays, ethics-review boards, peer-
review repositories, and downstream WIA standards),
how conformance evidence is produced, and how
deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012, ISO/IEC 27001:2022, ISO/IEC 27701:2019
- ISO/IEC 23053:2022, ISO/IEC 23894:2023
- IETF RFC 7515 (JWS), RFC 9421 (HTTP Message Signatures)
- W3C PROV-O 1.0, W3C VC 2.0, W3C DID 1.0
- HDF5, Zarr v3, Apache Parquet 2 (informative)
- IRB / IEC / REC ethics review framework (sovereign)
- FAIR Data Principles (Wilkinson et al., 2016, informative)
- DataCite Metadata Schema 4.4 (informative)

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-multiverse-interface
consumes upstream specifications (PROV-O, FAIR,
DataCite), how downstream WIA standards reference
multiverse-interface artefacts, and how
conformance is assessed, recorded, and audited.

## §2 Upstream integration

### 2.1 W3C PROV-O

Provenance graphs are PROV-O 1.0 documents. The
graph version is recorded in every signed
publication.

### 2.2 FAIR / DataCite

Datasets are described per FAIR Data Principles and
catalogued with DataCite Metadata Schema 4.4 for
DOI minting where the consortium issues DOIs.

### 2.3 ISO

ISO/IEC 23053 and 23894 frame the AI-assisted
analysis components (model cards, fairness audits)
where simulations consume ML models.

### 2.4 HDF Group / Zarr / Parquet

Storage formats are consumed at their published
versions; new releases are absorbed editorially
when they preserve backwards compatibility.

## §3 HPC integration

Simulation runs target HPC clusters declared in the
runtime profile. The registry records the
hardware fingerprint (CPU/GPU/QPU model, count,
interconnect) so that reproducibility evidence
captures the compute environment.

## §4 Peer-review integration

Hypotheses reference peer-reviewed papers via DOI
or ArXiv identifier. The registry validates the
reference against CrossRef / DataCite resolution
where possible.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared protocol register with ethics-review
  references;
- declared hypothesis register with peer-reviewed
  paper references;
- declared scenario register with simulation
  engine versions;
- the test-vector matrix per Annex G of each
  PHASE;
- the JWS signing key set used to sign records.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. Ethics-review references resolve at the issuing
   IRB / IEC / REC.
2. Hypothesis paper references resolve at CrossRef
   / DataCite / ArXiv.
3. Checkpoint hashes match the stored payload.
4. Provenance graph is well-formed PROV-O.
5. Identity-preservation claims carry active
   consent.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                       | Integration point                |
|--------------------------------|----------------------------------|
| WIA-quantum-computing          | actual quantum infrastructure    |
| WIA-multiverse-coordination    | consortium-level coordination    |
| WIA-prompts                    | prompted research narrative      |
| WIA-language-bridge            | multilingual research output     |
| WIA-learning-analytics         | training-set provenance          |

## §7 Privacy

Personal data of research participants is
processed under the consortium's privacy regime.
Pseudonymisation is the default; re-identification
keys are held by the principal investigator.

## §8 Security

The registry operates a vulnerability-disclosure
programme aligned with ISO/IEC 30111. Consortium
signing key compromise triggers JWKS rotation and
a public revocation list update.

## §9 Ethics oversight

Every research protocol references an active
ethics review (IRB / IEC / REC). The registry
verifies that the review is in force at every
mutating operation; expired reviews block further
publication until renewed.

## §10 Localisation

Research records carry a primary language tag and
optional translations. Multilingual narrative
fields are localised in BCP 47 form.

## §11 Accessibility

Public-facing summaries of research artefacts
conform to WCAG 2.2 AA. Assistive-tech-friendly
representations of figures are provided where
feasible (alt text, structured tables for plots).

## §12 Open science

Where the consortium permits, hypotheses,
scenarios, and observations are published under
open-access terms (CC0, CC-BY) with DOIs minted
via DataCite. The registry tracks the open-access
state in each record.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence
URL from their landing page.

## Annex B — Worked PROV-O fragment (informative)

```json
{
  "@context": "https://www.w3.org/ns/prov.jsonld",
  "@graph": [
    {"@id": "wasGeneratedBy", "@type": "prov:Activity"},
    {"@id": "actedOnBehalfOf", "@type": "prov:Agent"}
  ]
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with PROV-O / FAIR / DataCite major
releases.

## Annex D — Open governance

Issues at
`github.com/WIA-Official/wia-standards/issues` with
the `multiverse-interface` label.

## Annex E — Withdrawal procedure

Tombstone the evidence package; tombstones are
immutable.

## Annex F — Reproducibility

Reproducibility is the central conformance
criterion: a checkpoint hash must reproduce given
the recorded inputs, simulation engine version,
and runtime profile. Failed reproductions are
recorded for transparency.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative vector
under `tests/phase-vectors/`.

## Annex H — Sustainability

Simulation runs declare their estimated energy
cost per checkpoint and per scenario hour, using
ISO 14064 or the Green Software Foundation SCI
specification.

## Annex I — Risk register

| Risk                                   | Mitigation                |
|----------------------------------------|---------------------------|
| Ethics-review expiry undetected        | Mutating-op gate          |
| Peer-review reference invalid          | DOI / CrossRef gate       |
| Checkpoint payload tampering           | Digest verification       |
| Provenance graph malformed             | SHACL gate                |
| Identity-preservation consent withdraw | Tombstone + notify        |
| Simulation engine version drift        | Hardware fingerprint log  |

## Annex J — Sovereign-scale coordination

Where multiple national consortia collaborate,
federation peers are catalogued at
`/v1/registry/federation` with their accepted
operation groups and trust anchor JWKS URLs. Cross-
border data-sharing agreements are catalogued at
`/v1/registry/agreements`.

## Annex K — Continuous improvement programme

Each consortium publishes an annual improvement
plan addressing reproducibility findings, ethics-
review renewals, and storage-format migrations.

## Annex L — Open-access mandates

Public-funded research that falls under open-access
mandates (UKRI, NIH, EU Horizon, sovereign
equivalents) flags the artefact in the `mandates`
field. The registry surfaces a public landing page
for each mandated artefact within the funder's
declared embargo window.

## Annex M — Long-term archival

Checkpoint and observation payloads are archived in
preservation systems (CERN OpenData, sovereign
national archives, institutional repositories) per
the consortium's data-management plan. Archival
events emit `payload.archived` records with the
archive identifier and confirmation signature.

## Annex N — Reference implementation

A reference implementation is published under
Apache-2.0 at the WIA Standards GitHub umbrella
under `wia-multiverse-interface-reference`,
covering the full PHASE contract including the
PROV-O graph builder and the storage-format
negotiation layer.

## Annex O — Citation discipline

This standard does not opine on physical theory.
Where a record references a peer-reviewed paper,
the registry resolves the DOI / ArXiv identifier
but does not endorse the conclusions. Consortia
remain responsible for the soundness of their
research; the registry's role is interoperability.

## Annex P — Boundary with empirical claims

This PHASE catalogues research artefacts and their
provenance. It does not validate empirical
correctness; that responsibility falls on the
underlying scientific community via peer review.
Consortia that wish to assert empirical findings
publish their evidence packages alongside the WIA
records, with the WIA records serving as the
machine-readable provenance trail.

## Annex Q — Researcher onboarding

New researcher identities are registered via DID
documents controlled by their issuing institution.
The institution attests to the researcher's
affiliation and ethics-training completion.
Researcher records carry the affiliation
attestation and the ethics-training certificate
identifier.

## Annex R — Funding-agency reporting

Funding agencies (UKRI, NIH, NSF, EU Horizon,
sovereign equivalents) consume aggregate counters
from `/v1/registry/funding-summary`. The summary
reports artefact counts, publication rates,
reproducibility findings, and ethics-review
renewals on the cadence the agency requests.

## Annex S — Sustainability dashboards

Energy, hardware, and storage cost summaries are
exposed at `/v1/registry/sustainability` so that
consortia and funding agencies can compare
research impact against environmental cost.

## Annex T — Public engagement and outreach

Where the consortium wishes to make research
artefacts accessible to the general public, plain-
language summaries accompany the technical records.
Summaries are localised in BCP 47 form and follow
the consortium's style guide. The registry hosts a
public-engagement endpoint surfacing the summaries
without exposing raw research data.

## Annex U — Withdrawal and post-publication notes

Authors may attach post-publication notes to a
record (corrections, clarifications, retractions).
Notes are signed by the principal investigator and
catalogued alongside the record without altering
the original signature.

## Annex V — Industry binding catalogue

| Industry segment            | Bound standard                    |
|-----------------------------|-----------------------------------|
| HPC / Exascale              | ISO/IEC HPC profile               |
| Cosmological observation    | IVOA VOEvent / VOTable            |
| Quantum simulation          | OpenQASM 3.0 (informative)        |
| Particle physics            | HEPMC, ROOT (informative)         |
| Geophysics                  | ObsPy / FDSN (informative)        |

## Annex W — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Ethics-review expiry undetected       | Mutating-op gate          |
| Empirical claim overreach             | Boundary disclaimer       |
| Researcher key compromise             | Immediate JWKS rotation   |
| Pseudonymisation key exposure         | Two-custodian recovery    |
| Checkpoint payload tampering          | Streamed digest verify    |
| Federation hop loop                   | Hops > 4 dropped          |
| Storage-format obsolescence           | Annual format review      |

弘益人間 (Hongik Ingan) — Benefit All Humanity
