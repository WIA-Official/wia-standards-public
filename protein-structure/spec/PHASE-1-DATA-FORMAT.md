# WIA-protein-structure PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-protein-structure
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-protein-structure. The standard covers exchange of protein
structural records — experimental and computationally predicted —
among structural-biology laboratories, archive consortia,
structure-prediction service operators, downstream consumers
(drug-discovery, enzyme-engineering, vaccine-design teams),
journals that cite structural results, and the regulators that
ingest structural evidence in regulatory dossiers. The format is
a thin envelope around community-standard structural representations
(mmCIF / PDBx, BCIF, the Chemical Component Dictionary, and modern
predicted-structure outputs) so that interoperation across
deposit, prediction, and consumer systems is preserved.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- HL7 FHIR R5 (`MolecularSequence` resource for sequence references
  carried alongside structural records when relevant to clinical
  context)
- W3C XML Schema Definition 1.1 (legacy PDBML envelope)

The standard cites — but does not redefine — the wwPDB-managed
mmCIF / PDBx dictionary, the BCIF binary serialisation of CIF, the
Chemical Component Dictionary (CCD), and the published output
schemas of contemporary structure-prediction services (AlphaFold,
ESMFold, RoseTTAFold, and successors). Tooling such as Foldseek and
DALI is referenced normatively for similarity-search pipelines.

---

## §1 Scope

This PHASE document defines persistent shapes for the records that
flow during deposit, prediction, citation, and downstream
consumption of protein structural data. Implementations covered
include:

- Structural-biology laboratories that deposit experimental
  structures (X-ray crystallography, cryo-EM, solution NMR,
  micro-electron diffraction, neutron diffraction).
- Structure-prediction service operators that publish predicted
  structures and their per-residue and inter-residue confidence
  estimates.
- Archive consortia (the wwPDB and its member archives, model
  archives, predicted-structure archives) that ingest, validate,
  and re-publish structural records.
- Similarity-search and structure-mining services (Foldseek, DALI,
  domain-classification systems) that index records into derived
  resources.
- Regulators and downstream consumers that ingest structural
  evidence as part of broader submissions.

Sequence-only deposits (UniProt, NCBI), molecular-dynamics
trajectory exchange (governed by adjacent WIA-protein-dynamics),
and small-molecule crystallography deposits (governed by adjacent
small-molecule standards) are out of scope here.

## §2 Structure Identifier

```
structureId       : string (uuidv7)
structureCreatedAt: string (ISO 8601 / RFC 3339)
structureAuthor   : string (institutional author identifier)
provenanceClass   : enum  ("experimental" | "predicted" |
                       "integrative-hybrid")
experimentalMethod: enum ("xray" | "cryo-em" | "cryo-et" |
                       "solution-nmr" | "solid-state-nmr" |
                       "micro-electron-diffraction" |
                       "neutron-diffraction" |
                       "small-angle-scattering" | "n/a")
predictionEngine  : string (engine family + version when
                       provenanceClass = "predicted"; e.g.
                       "alphafold-3", "esmfold-1.0",
                       "rosettafold-2"; absent for experimental)
externalArchiveAccession : string (PDB / EMDB / model-archive
                       accession when the structure is mirrored
                       into an external archive; absent for
                       internal-only structures)
```

A structure that is both predicted and subsequently confirmed
experimentally emits one record per provenance class; the records
are linked by `crossRef` entries (§9).

## §3 Coordinate Record

The coordinate payload is carried as a content-addressed CIF or
BCIF artefact rather than inlined into JSON. The record references
the artefact and carries the metadata needed to interpret it.

```
coordinateArtefact:
  structureId     : string (uuidv7)
  format          : enum ("mmcif" | "bcif" | "pdb-legacy" |
                       "pdbml-legacy")
  artefactRef     : string (content-addressed URI of the coordinate
                       file)
  contentDigest   : string (SHA-256 of the coordinate file)
  cifDictionaryVersion : string (mmCIF dictionary version the
                       coordinate file conforms to)
  ccdVersion      : string (Chemical Component Dictionary version
                       in force at deposit time)
  asymUnitCount   : integer (number of asymmetric units in the
                       file)
  totalChainCount : integer
  totalResidueCount : integer
  totalAtomCount  : integer
```

Coordinate files MUST conform to the cited CIF dictionary version;
deposits whose CIF parsing fails are rejected with a Problem-Details
(RFC 9457) response of type
`urn:wia:protein-structure:cif-parse-failure`.

## §4 Sequence and Composition

Sequence and composition records describe the polymeric chains and
non-polymer ligands present in the structure.

```
chain:
  chainId         : string (matches CIF `_atom_site.label_asym_id`
                       or successor)
  structureId     : string (uuidv7)
  polymerKind     : enum ("polypeptide-l" | "polypeptide-d" |
                       "polynucleotide-dna" |
                       "polynucleotide-rna" |
                       "polysaccharide" | "other")
  oneLetterSequence : string (when applicable; absent for non-
                       polymer chains)
  uniprotAccession  : string (when the sequence corresponds to a
                       UniProt entry)
  fhirSequenceRef   : string (FHIR R5 MolecularSequence.id when
                       the sequence is held in a clinical-genomics
                       context)
  modifiedResidues  : array of ModifiedResidue (CCD identifiers
                       for non-standard residues)

ligand:
  ligandId        : string
  structureId     : string (uuidv7)
  ccdComponentId  : string (CCD three-letter code or extended
                       identifier)
  bindingChainId  : string (the chain whose binding site holds the
                       ligand)
  isCovalentlyAttached : boolean
```

## §5 Experimental Evidence Record (when applicable)

```
experimentalEvidence:
  evidenceId      : string (uuidv7)
  structureId     : string (uuidv7)
  facility        : string (institutional identifier of the
                       beamline / microscope / spectrometer)
  startedAt       : string (ISO 8601)
  endedAt         : string (ISO 8601)
  rawDataRef      : string (content-addressed URI of the raw data
                       archive)
  processingPipelineRef : string (content-addressed URI of the
                       processing pipeline manifest)
  resolutionAngstrom : number (when applicable)
  rWork           : number (X-ray refinement R-factor)
  rFree           : number
  ramachandranOk_pct : number
  bondGeometryOutliers_per1000 : number
```

## §6 Prediction Confidence Record (when applicable)

```
predictionConfidence:
  confidenceId    : string (uuidv7)
  structureId     : string (uuidv7)
  perResidueRef   : string (content-addressed URI of per-residue
                       confidence; pLDDT for AlphaFold-family
                       models, equivalent quantities for others)
  pairwiseRef     : string (content-addressed URI of pairwise
                       confidence; PAE for AlphaFold-family
                       models)
  globalConfidence: object (model-specific summary; ipTM, pTM,
                       global pLDDT median, etc.)
  inputMSARef     : string (content-addressed URI of the input
                       multiple-sequence alignment when the
                       prediction engine consumes one)
  inputTemplateRef : string (content-addressed URI of input
                       templates when the prediction engine
                       consumes them)
```

## §7 Validation Report

```
validationReport:
  reportId        : string (uuidv7)
  structureId     : string (uuidv7)
  reportRef       : string (content-addressed URI of the wwPDB-style
                       validation report PDF and JSON)
  overallQuality  : enum ("good" | "acceptable" | "borderline" |
                       "rejected")
  outlierResidues : array of ResidueRef
  outlierLigands  : array of LigandRef
```

## §8 Similarity and Annotation Record

```
similarity:
  similarityId    : string (uuidv7)
  structureId     : string (uuidv7)
  searchTool      : enum ("foldseek" | "dali" | "tm-align" |
                       "structure-search-service-other")
  searchToolVersion : string
  searchedAt      : string (ISO 8601)
  hits            : array of SimilarityHit

SimilarityHit:
  hitStructureId  : string (UUID or external accession)
  scoreKind       : enum ("tm-score" | "lddt" | "rmsd" |
                       "z-score" | "user-defined")
  scoreValue      : number
  alignedRange    : object (per-chain alignment ranges)
```

## §9 Cross-References

```
crossRef:
  fromStructureId : string (uuidv7)
  relationship    : enum ("predicted-of-experimental" |
                       "experimental-of-predicted" |
                       "alternate-conformation" |
                       "successor" | "predecessor" |
                       "covers-same-uniprot")
  toStructureId   : string (uuidv7 or external accession)
  notes           : string (free text)
```

## §10 Modification and Post-Translational State Record

Modified residues, glycans, and post-translational modifications
catalogued in the Chemical Component Dictionary are recorded
against the chain to which they belong. Modifications observed in
experimental data but not yet present in the CCD are flagged
`provisional-ccd` and emit a request to the dictionary maintainer.

```
postTranslationalState:
  ptmId           : string (uuidv7)
  chainId         : string
  residuePosition : integer (residue number in author numbering)
  modificationCcd : string (CCD identifier; may be `provisional-
                       ccd-<requestId>` while pending)
  modificationKind: enum ("phosphorylation" | "glycosylation" |
                       "methylation" | "acetylation" |
                       "ubiquitination" | "lipidation" |
                       "disulfide" | "covalent-other" |
                       "deletion-cleavage")
  evidenceRef     : string (reference to experimental-evidence
                       record or prediction-confidence record that
                       supports the modification)
```

## §11 Quaternary Assembly Record

For multi-chain or multi-copy structures, the quaternary assembly
record describes the biological assembly the depositor or predictor
believes the structure represents.

```
quaternaryAssembly:
  assemblyId      : string (uuidv7)
  structureId     : string (uuidv7)
  assemblyKind    : enum ("monomer" | "homo-dimer" |
                       "hetero-dimer" | "trimer" | "tetramer" |
                       "higher-multimer" | "fibril-or-filament")
  symmetryOperators : array of string (BIOMT-style operators or
                       successor representations)
  evidenceRef     : string (reference to the experimental or
                       predicted evidence underlying the assembly
                       claim)
  alternateAssemblyRef : string (URI of an alternate assembly when
                       multiple are plausible)
```

## §12 Domain and Function Annotation

Protein chains may be annotated with domain assignments (e.g. SCOP,
CATH, ECOD), functional sites (active-site residues, ligand-
binding pockets, allosteric sites), and Gene Ontology biological-
process / molecular-function / cellular-component terms.
Annotations are content-addressed and link the chain to the
external annotation source.

```
annotation:
  annotationId    : string (uuidv7)
  chainId         : string
  source          : enum ("scop" | "cath" | "ecod" | "go" |
                       "interpro" | "uniprot-feature" |
                       "user-defined")
  sourceVersion   : string
  artefactRef     : string (content-addressed URI of the
                       annotation artefact)
```

## §13 Withdrawal and Supersession Record

Structures may be withdrawn (incorrect refinement, retracted
manuscript, sequencing error in the deposited chain) or superseded
(refined version, alternate conformation, prediction with updated
inputs). The withdrawal/supersession record captures the
relationship and the reason.

```
withdrawalSupersession:
  recordId        : string (uuidv7)
  affectedStructureId : string (uuidv7)
  kind            : enum ("withdrawn" | "superseded")
  successorStructureId : string (UUID or external accession;
                       absent for `withdrawn` without a successor)
  reason          : enum ("refinement-revision" |
                       "sequence-correction" |
                       "model-improvement" | "manuscript-retracted"
                       | "incorrect-ligand" | "user-defined")
  effectiveAt     : string (ISO 8601)
  publicNoticeRef : string (URI of the public notice the
                       operating programme issued)
```

Withdrawn or superseded structures remain addressable at their
content-addressed URLs so that historical citations resolve
unambiguously, but the public catalogue marks them with the
withdrawal/supersession status so that consumers do not consume
them in error.

## §14 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every released structure and honour the
content-addressing rules in §3-§8.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-protein-structure
- **Last Updated:** 2026-04-27
