# WIA-protein-dynamics PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-protein-dynamics
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-protein-dynamics.
Protein dynamics studies the time-resolved conformational behaviour of
proteins under physiologically relevant conditions: equilibrium fluctuations,
folding pathways, allosteric transitions, ligand binding kinetics, and
mechanically induced unfolding. The format captures the structural starting
point, the simulation or experimental conditions, the trajectory or
spectroscopic observable, the analysis pipeline that reduces raw data to
interpretable observables, and the metadata that supports reproducibility
and citation of the result.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- IETF RFC 4122 (UUID URN namespace)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- W3C XML Schema Definition 1.1 (used only for legacy CIF/PDBx envelope)
- HL7 FHIR R5 (only for biospecimen provenance metadata when the protein
  preparation derives from a clinical biospecimen)

---

## §1 Scope

This PHASE document defines the persistent shapes for protein-dynamics
records: the input structure description, the simulation or experimental
configuration, the trajectory or time-resolved observable, the reduced
observables produced by analysis, and the provenance metadata that links
the result to its source biospecimen and to the laboratory that produced
it.

Implementations covered by this PHASE include:

- Computational dynamics platforms that emit molecular-dynamics
  trajectories, enhanced-sampling free-energy surfaces, or Markov
  state models.
- Experimental laboratories that emit time-resolved spectroscopy
  (NMR relaxation, single-molecule FRET, hydrogen-deuterium exchange
  mass spectrometry, time-resolved X-ray scattering) and that need a
  common envelope to publish their observables.
- Reference repositories that ingest published trajectories and
  spectroscopic observables and serve them to downstream consumers.
- Citation tools that resolve a published result to its underlying
  records.

Static structural records (single-conformer X-ray, cryo-EM, NMR ensembles
without dynamics) are out of scope; those are governed by community
formats (mmCIF / PDBx) that this PHASE references rather than redefining.

## §2 Study Identifier

Every dynamics study carries a stable identifier. The identifier is a
UUIDv7 (RFC 4122 / ISO/IEC 11578) so that records sort by acquisition
order in lexicographic storage.

```
studyId           : string (uuidv7)
studyCreatedAt    : string (ISO 8601 / RFC 3339, UTC, second precision)
studyAuthor       : string (institutional author identifier)
studyDomain       : enum  ("computational" | "nmr" | "single-molecule" |
                       "hdx-ms" | "time-resolved-xray" | "ir-spectroscopy" |
                       "raman" | "neutron-spin-echo")
studyStatus       : enum  ("draft" | "in-progress" | "complete" |
                       "withdrawn" | "deprecated")
```

The `studyDomain` selects which subset of the format applies. A study that
combines computational and experimental observables emits one study record
per domain that share the same `studyId` prefix and differ in their
`studyDomain` field.

## §3 Protein Preparation

The preparation record describes the protein construct and its source.

```
preparation:
  preparationId    : string (uuidv7)
  studyId          : string (uuidv7)
  uniprotAccession : string (UniProt primary accession; absent for
                       de novo designed sequences, in which case
                       sequenceRef is mandatory)
  sequenceRef      : string (content-addressed URI of the FASTA file;
                       SHA-256 in the URL fragment)
  constructTags    : array of string ("his6", "tev-cleavable", "gst-fusion",
                       etc.; empty when the construct is the natural sequence)
  expressionSystem : enum ("ecoli" | "yeast" | "insect-cell" |
                       "mammalian-cell" | "cell-free" | "synthetic-peptide")
  purification:
    chromatography : array of string (recipe codes for the columns used)
    finalPurityPct : number (percent purity by SDS-PAGE densitometry)
    sec_mals_kda   : number (apparent molecular weight from SEC-MALS, kDa)
  buffer:
    composition    : string (recipe code; the recipe register is governed
                       by PHASE-3)
    pH             : number
    temperatureK   : number (sample temperature in Kelvin)
    additives      : array of string (cofactor / inhibitor recipe codes)
  biospecimenLink  : string (FHIR Specimen.id when the preparation derives
                       from a clinical biospecimen; absent otherwise)
```

A preparation derived from a clinical biospecimen carries its FHIR R5
`Specimen.id` so that consents and access controls can be checked against
the source institution's biobank governance.

## §4 Starting Structure Record

The starting structure for a computational study, or the resting-state
structure for an experimental study, is recorded with full provenance.

```
structureRef:
  structureId      : string (uuidv7)
  studyId          : string (uuidv7)
  source           : enum ("pdb" | "alphafold-prediction" |
                       "internal-cryo-em" | "internal-xray" |
                       "internal-solution-nmr" | "homology-model")
  archiveAccession : string (PDB accession or model archive identifier;
                       absent for internal structures, in which case
                       artefactRef is mandatory)
  artefactRef      : string (content-addressed URI of the mmCIF file)
  experimentalReso : number (resolution in Angstroms; absent for predictions)
  refinementRsr    : number (refinement R-factor; absent for predictions)
  ramachandranOk   : number (percent residues in favoured Ramachandran
                       regions)
```

## §5 Computational Trajectory Record

For computational studies, the simulation record captures the engine,
the force field, and the integrator parameters; the trajectory record
references the produced coordinates.

```
simulation:
  simulationId     : string (uuidv7)
  studyId          : string (uuidv7)
  structureId      : string (uuidv7, references §4)
  engineFamily     : enum ("classical-md" | "enhanced-sampling" |
                       "qm-mm" | "coarse-grained" | "monte-carlo")
  forceFieldRef    : string (force-field family + version; e.g.
                       "amber-ff19sb")
  waterModel       : enum ("tip3p" | "tip4p-ew" | "spc-e" | "opc" |
                       "explicit-other" | "implicit")
  ensemble         : enum ("nve" | "nvt" | "npt" | "lambda-dynamics")
  thermostat       : string (recipe code; absent for nve)
  barostat         : string (recipe code; absent for nve / nvt)
  integratorStepFs : number (integrator timestep in femtoseconds)
  totalDurationNs  : number (cumulative simulation length in nanoseconds)
  enhancedSampling : EnhancedSampling (present iff engineFamily =
                       "enhanced-sampling")

trajectory:
  trajectoryId     : string (uuidv7)
  simulationId     : string
  artefactRef      : string (content-addressed URI; XTC / DCD / TRR
                       formats accepted)
  saveStrideFs     : number (interval between saved frames)
  framesCount      : integer
```

## §6 Spectroscopic Observable Record

For experimental studies, the observable record captures the measurement
modality and the time-resolved signal.

```
observable:
  observableId     : string (uuidv7)
  studyId          : string (uuidv7)
  preparationId    : string (uuidv7, references §3)
  modality         : enum ("nmr-relaxation" | "smfret" | "hdx-ms" |
                       "trxss" | "ir-spectroscopy" | "raman" |
                       "neutron-spin-echo")
  instrumentRef    : string (instrument register entry, see §10)
  laboratoryRef    : string (ISO/IEC 17025-accredited laboratory ID)
  raw:
    artefactRef    : string (content-addressed URI of the raw data archive)
    encoding       : enum ("hdf5" | "netcdf" | "vendor-binary")
  reduced:
    artefactRef    : string (content-addressed URI of the reduced
                       observable)
    encoding       : enum ("json" | "hdf5" | "csv")
  uncertainty      : Uncertainty (type-A / type-B contributions per
                       JCGM 100)
```

## §7 Analysis Pipeline Record

The analysis pipeline reduces a trajectory or raw observable to
interpretable quantities (RMSF, RMSD, secondary-structure occupancy,
free-energy surfaces, Markov state models, FRET-efficiency histograms,
residence-time distributions). The pipeline is recorded so that the
reduction can be reproduced.

```
analysis:
  analysisId       : string (uuidv7)
  studyId          : string (uuidv7)
  inputs           : array of string (URIs of trajectory or observable
                       artefacts)
  pipelineRef      : string (content-addressed URI of the pipeline
                       script bundle, including version and dependencies)
  outputs          : array of AnalysisOutput
  softwareEnvRef   : string (content-addressed URI of the locked
                       dependency manifest)

AnalysisOutput:
  outputId         : string (uuidv7)
  kind             : enum ("rmsd" | "rmsf" | "secondary-structure" |
                       "fes" | "msm" | "fret-efficiency" |
                       "exchange-protection" | "structure-ensemble" |
                       "rate-distribution")
  artefactRef      : string (content-addressed URI of the output)
  encoding         : enum ("json" | "hdf5" | "csv")
  legend           : object (column / dimension definitions)
```

## §8 Cross-Validation Record

Studies that combine computational and experimental observables emit a
cross-validation record that compares the two. The record carries the
analysis outputs from each domain, the comparison methodology, and the
agreement metric.

```
crossValidation:
  crossValidationId: string (uuidv7)
  studyId          : string (uuidv7)
  computationalRef : string (analysisOutput URI from §7)
  experimentalRef  : string (analysisOutput URI from §7)
  metric           : enum ("chi-square" | "rmsd" | "kl-divergence" |
                       "earth-mover" | "custom")
  metricValue      : number
  reportRef        : string (content-addressed URI of the comparison report)
```

## §9 Provenance and Consent

Records that derive from clinical biospecimens carry the chain of consent,
expressed as a hash chain over the consent documents and the biobank
governance entries that authorised use of the specimen for the named
study. The hash chain is verifiable against the biobank's governance
service via the API defined in PHASE-2 §9.

## §10 Instrument and Software Register

Each instrument that contributes raw experimental data and each software
environment that contributes computational data carries a stable
identifier in the register. The register is exported as part of the
evidence package described in PHASE-4 §3.

## §11 Ensemble and Replica Records

Studies that publish ensemble-level results (e.g. multiple-replica
simulations, multi-temperature replica exchange, parallel single-molecule
acquisitions) emit an ensemble record that groups the constituent
simulations or observables. The ensemble record carries the experimental
or computational rationale for the grouping, the grouping criterion, and
the aggregation methodology.

```
ensemble:
  ensembleId       : string (uuidv7)
  studyId          : string (uuidv7)
  groupingCriterion: enum ("replica" | "temperature" | "salt" |
                       "ph-titration" | "ligand-titration" | "mutant-series")
  members          : array of EnsembleMember
  aggregationRef   : string (content-addressed URI of the aggregation
                       script)

EnsembleMember:
  memberRef        : string (URI of a constituent simulation or observable)
  groupingValue    : string (the value of the grouping criterion at this
                       member; e.g. "300K", "150mM-NaCl", "L99A-mutant")
  weight           : number (mixing weight in the ensemble; defaults to 1)
```

Ensembles may not nest; a study that wants nested grouping (replicas
across multiple temperatures, for example) emits one outer ensemble and
references the inner ensembles as its members.

## §12 Conformance

Implementations claiming PHASE-1 conformance MUST emit each of the records
defined above for every dynamics study, MUST honour the content-addressing
rules in §3-§7, and MUST fail validation cleanly with a Problem Details
(RFC 9457) response when an export is requested for a study that contains
incomplete records. Conformance against this PHASE is independent of
conformance against PHASE-2 / PHASE-3 / PHASE-4, but Deep certification
under WIA-protein-dynamics requires all four PHASE attestations.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-protein-dynamics
- **Last Updated:** 2026-04-27
