# WIA-multiverse-interface PHASE 1 — Data Format Specification

**Standard:** WIA-multiverse-interface (WIA-QUA-017)
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-multiverse-interface, the speculative-research
coordination standard for multiverse-interface
experimentation, simulation, and inter-research-team
data exchange. The records bind every universe
hypothesis, simulation scenario, observation event,
identity preservation claim, and timeline checkpoint
to a documented research protocol, an ethics-review
ID, and a provenance chain so that multi-team
research consortia can coordinate without
methodological drift.

This standard is intentionally framed as a research-
coordination interchange, not a claim about
empirical physics. Where research teams operate over
real infrastructure (HPC clusters, observation
arrays, distributed simulation platforms), the
records reference real authoritative specifications
for transport, identity, and signing.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8259 (JSON), RFC 8785 (JSON Canonicalisation Scheme)
- IETF RFC 7515 (JWS), RFC 7517 (JWK), RFC 4122 (UUID)
- IETF RFC 8615 (Well-Known URIs), RFC 9421 (HTTP Message Signatures)
- ISO 8601 (Date and time), ISO/IEC 27001:2022, ISO/IEC 27701:2019
- ISO/IEC 23053:2022 (Framework for AI systems using ML)
- ISO/IEC 23894:2023 (AI guidance on risk management)
- IEEE 1857.9 (immersive media coding, informative)
- W3C RDF 1.1, W3C SHACL, W3C JSON-LD 1.1
- W3C VC 2.0, W3C DID 1.0
- OASIS Open Provenance Model (OPM), W3C PROV-O 1.0
- HDF5 (The HDF Group, informative storage), Zarr v3 (informative)
- ISO 19115-2:2019 (geospatial / observational metadata, informative)
- IRB / IEC / REC ethics review framework (sovereign)

---

## §1 Scope

This PHASE applies to records that describe the
inputs, outputs, and provenance of speculative
multiverse-interface research. The records support
publishing hypotheses, simulation scenarios,
observation events, identity-preservation claims,
timeline checkpoints, and decoherence event logs.

In scope: research-protocol record, universe-
hypothesis record, scenario record, simulation
checkpoint record, observation record, identity-
preservation record, timeline record, decoherence-
event log, and the cross-references binding each
record to its ethics review and to the consortium
that produced it.

Out of scope: any claim about empirical physics
beyond what the underlying observation hardware can
substantiate; any claim that this PHASE replaces
peer review in the relevant scientific journals.

## §2 Research-protocol record

Every research artefact carries:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `protocolRef`        | UUID (RFC 4122) opaque                          |
| `name`               | localised protocol name (BCP 47 keys)           |
| `consortiumRef`      | URI                                             |
| `ethicsReviewRef`    | IRB / IEC / REC approval reference              |
| `dataManagementPlan` | URI                                             |
| `principalInvestigator` | DID per W3C DID 1.0                          |
| `funding[]`          | grant identifiers                               |
| `signingKeyRef`      | JWKS URL                                        |

## §3 Universe-hypothesis record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `hypothesisRef`      | URI                                             |
| `protocolRef`        | this PHASE §2                                   |
| `interpretation`     | many-worlds, decoherent-histories, modal,       |
|                      | string-landscape, simulation-frame              |
| `parameterSet`       | JSON document; canonicalised per RFC 8785       |
| `predictionSet[]`    | named predictions with confidence intervals     |
| `references[]`       | DOIs / ArXiv IDs of the underlying papers       |
| `falsifiability`     | declared falsification criterion                |

The `interpretation` enumeration is intentionally
limited to interpretations published in peer-
reviewed literature; novel interpretations require
a referenced paper before they can be admitted.

## §4 Scenario record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `scenarioRef`        | URI                                             |
| `hypothesisRef`      | this PHASE §3                                   |
| `description`        | localised scenario description                  |
| `inputBindings[]`    | named inputs with type and SHACL shape          |
| `expectedOutputs[]`  | named outputs with type and SHACL shape         |
| `simulationEngine`   | engine identifier (informative)                 |
| `runtimeProfile`     | hardware / cluster profile reference            |
| `dataset[]`          | input dataset references                        |

## §5 Simulation checkpoint record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `checkpointRef`      | UUID                                            |
| `scenarioRef`        | this PHASE §4                                   |
| `wallClockTime`      | ISO 8601                                        |
| `simulatedTime`      | dimensionless tick or domain-specific value     |
| `stateHash`          | SHA-512 of the checkpoint payload               |
| `storageRef`         | URI to HDF5 / Zarr / Parquet payload            |
| `compute`            | hardware fingerprint (CPU/GPU/QPU)              |
| `entropyEstimate`    | optional Shannon entropy estimate of the state  |

Checkpoints are immutable; replays produce new
checkpoint records.

## §6 Observation record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `observationRef`     | UUID                                            |
| `protocolRef`        | this PHASE §2                                   |
| `instrumentRef`      | URI to instrument metadata                      |
| `acquiredAt`         | ISO 8601                                        |
| `geo`                | optional geospatial reference per ISO 19115-2   |
| `dataRef`            | URI to raw data                                 |
| `calibrationRef`     | URI to calibration record                       |
| `signature`          | RFC 7515 detached JWS signed by the operator    |

## §7 Identity-preservation record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `identityRef`        | UUID                                            |
| `subject`            | DID per W3C DID 1.0                             |
| `claim`              | identity claim narrative                        |
| `evidence[]`         | observation / checkpoint references             |
| `riskAssessmentRef`  | ISO/IEC 23894 risk assessment                   |
| `consentRef`         | active consent record                           |

This record is intentionally narrow: it captures
research participants' consent and risk-assessment
provenance; it does not assert any metaphysical
claim about identity continuation beyond the
research protocol's documented framework.

## §8 Timeline record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `timelineRef`        | URI                                             |
| `protocolRef`        | this PHASE §2                                   |
| `branchPoints[]`     | scenario references that diverge the timeline   |
| `mergePoints[]`      | scenario references that merge timelines        |
| `clockSync`          | clock-synchronisation method (NTPv4, PTP)       |
| `provenanceGraph`    | W3C PROV-O reference                            |

## §9 Decoherence-event log

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `eventRef`           | UUID                                            |
| `scenarioRef`        | this PHASE §4                                   |
| `kind`               | `coherence-loss`, `entanglement-break`,         |
|                      | `measurement-collapse`                          |
| `instrumentRef`      | optional instrument metadata                    |
| `magnitude`          | dimensionless severity (0..1)                   |
| `notes`              | researcher notes                                |

This log is informative and is intended for cross-
team comparison of how decoherence-related events
are operationalised across protocols.

## §10 Cross-domain references (informative)

- WIA-quantum-computing — actual quantum infra
- WIA-multiverse-coordination — consortium-level
- WIA-prompts — prompted research narrative
- WIA-language-bridge — multilingual research output

## Annex A — Conformance disclosure

Implementations declare the schema versions they
support, the canonicalisation form (RFC 8785), and
the JWS key set used to sign protocol, hypothesis,
scenario, and observation records.

## Annex B — Worked scenario record (informative)

```json
{
  "scenarioRef": "https://muv.example.org/scenarios/cosmo-001",
  "hypothesisRef": "https://muv.example.org/hyp/many-worlds",
  "description": "Cosmological constant variance across reference frames",
  "simulationEngine": "wia-qua-sim/2.4",
  "runtimeProfile": "wia-cluster/A100x128"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with the consortium's published
data-management plan.

## Annex D — Conformance level

Conformance is "Core" (protocol + hypothesis +
scenario + checkpoint + observation) or "Full"
(adds identity-preservation, timeline, and
decoherence-event logs).

## Annex E — Privacy and consent

Personal data of research participants is
processed under the consortium's privacy regime
(GDPR, K-PIPA, sovereign equivalents). Consent is
recorded under a documented research protocol and
verified at audit.

## Annex F — Provenance graph

W3C PROV-O 1.0 records bind every research artefact
to the activities, agents, and datasets that
produced it. The provenance graph is signed jointly
by the principal investigator and the consortium's
ethics-review chair.

## Annex G — Reproducibility

Reproducibility evidence includes the simulation
engine version, the runtime profile, the dataset
digests, and the checkpoint hashes. Reproductions
that diverge from the original are recorded as new
checkpoints with `corrects` set to the original.

## Annex H — Storage format trade-offs (informative)

| Format    | Use                                              |
|-----------|--------------------------------------------------|
| HDF5      | dense N-D arrays, mature ecosystem               |
| Zarr v3   | chunked cloud-native, S3 / GCS / Azure friendly  |
| Parquet   | columnar tabular, analytics-friendly             |
| FITS      | astronomical observation legacy                  |

Consortia choose per scenario; the registry
records the selection so that consumers can
configure their reader without round-tripping.

## Annex I — Pseudonymisation registry

Researcher identities (DIDs) MAY be pseudonymised
for cross-consortium publication. The
pseudonymisation key is held by the principal
investigator under the consortium's data-management
plan. Re-identification requires a documented
court or ethics-review order.

## Annex J — Cohort and group identifiers

Group records bundle multiple identity-preservation
records under a shared cohort tag. Cohort
membership is governed by the protocol's consent
record set; a learner can only belong to a cohort
if their consent covers the cohort's declared
research purposes.

## Annex K — Reference frame metadata

Observation records carry reference-frame metadata:
the spatial frame (geocentric, heliocentric, ICRF),
the temporal frame (TAI, UTC, TT), the
gravitational potential reference, and any frame
transformations applied. Frame metadata is
informative; it documents the operator's choices
without endorsing a particular cosmological
framework.

## Annex L — Versioned dataset identifiers

Datasets carry a DOI minted via DataCite when the
consortium issues DOIs. The dataset record carries
a versioned identifier: each correction to the
dataset emits a new DOI with a `previous` link.

## Annex M — Pseudonymisation key escrow

Pseudonymisation keys are escrowed under the
consortium's data-management plan with at least
two key custodians. Recovery requires both
custodians' signatures; single-custodian recovery
is forbidden to prevent insider misuse.

弘益人間 (Hongik Ingan) — Benefit All Humanity
