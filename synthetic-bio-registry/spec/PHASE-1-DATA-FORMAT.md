# WIA-BIO-021 — Phase 1: Data Format

> Synthetic-bio-registry canonical Phase 1: part classification + metadata + sequence formats + SBOL.

# WIA-BIO-021: Synthetic Biology Registry Specification v1.0

> **Standard ID:** WIA-BIO-021
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Part Classification System](#2-part-classification-system)
3. [Metadata Standards](#3-metadata-standards)
4. [Sequence Formats](#4-sequence-formats)
5. [SBOL Compliance](#5-sbol-compliance)
6. [Characterization Data](#6-characterization-data)
7. [Version Control](#7-version-control)
8. [Access Control and Licensing](#8-access-control-and-licensing)
9. [Registry Interoperability](#9-registry-interoperability)
10. [Implementation Guidelines](#10-implementation-guidelines)

---


## 1. Introduction

### 1.1 Purpose

This specification defines a unified framework for synthetic biology part registration, storage, and sharing. It enables interoperability between major biological part registries while maintaining data integrity and reproducibility.

### 1.2 Scope

The standard covers:
- BioBrick and non-BioBrick part classification
- Metadata schema for biological parts
- Sequence storage and format conversion
- Part characterization data structures
- Version control mechanisms
- Access control and licensing frameworks
- Integration with existing registries (iGEM, Addgene, SynBioHub)

### 1.3 Philosophy

**弘益인間 (Benefit All Humanity)** - This standard promotes open science and global collaboration in synthetic biology by providing accessible, standardized tools for sharing biological knowledge.

### 1.4 Terminology

- **Part**: A discrete DNA/RNA sequence with defined function
- **BioBrick**: Standardized genetic part with RFC10 or RFC25 assembly
- **Device**: Combination of multiple parts with integrated function
- **SBOL**: Synthetic Biology Open Language
- **RFC**: Request for Comments (BioBrick assembly standards)
- **Characterization**: Experimental data describing part behavior

---



## 2. Part Classification System

### 2.1 Primary Part Types

Parts are classified into seven primary categories:

#### 2.1.1 Promoters (Type: PRO)
Regulatory sequences that initiate transcription.

**Naming Convention**: `BBa_P[number]` or `BBa_J[number]`

**Properties**:
- Strength (relative fluorescence units)
- Inducibility (constitutive/inducible)
- Organism specificity
- Regulatory mechanism

**Examples**:
- `BBa_J23100`: Strong constitutive promoter
- `BBa_R0011`: LacI-regulated promoter

#### 2.1.2 Ribosome Binding Sites (Type: RBS)
Translation initiation sequences.

**Naming Convention**: `BBa_B[number]`

**Properties**:
- Translation initiation rate
- Organism compatibility
- Spacing requirements
- Secondary structure

**Examples**:
- `BBa_B0034`: Medium-strength RBS
- `BBa_B0030`: Strong RBS

#### 2.1.3 Coding Sequences (Type: CDS)
Protein-encoding sequences.

**Naming Convention**: `BBa_C[number]` or `BBa_E[number]`

**Properties**:
- Protein product
- Molecular weight
- Function/activity
- Optimal expression conditions
- Codon optimization status

**Examples**:
- `BBa_E0040`: Green Fluorescent Protein (GFP)
- `BBa_E1010`: mRFP1 (Red Fluorescent Protein)

#### 2.1.4 Terminators (Type: TER)
Transcription termination sequences.

**Naming Convention**: `BBa_B[number]` or `BBa_T[number]`

**Properties**:
- Termination efficiency
- Directionality (forward/reverse)
- Context dependency

**Examples**:
- `BBa_B0015`: Double terminator
- `BBa_B0010`: T1 from E. coli rrnB

#### 2.1.5 Plasmid Backbones (Type: VEC)
Vector sequences for part maintenance and propagation.

**Naming Convention**: `BBa_pSB[number]`

**Properties**:
- Antibiotic resistance
- Origin of replication
- Copy number
- Cloning sites
- Size (bp)

**Examples**:
- `BBa_pSB1C3`: Chloramphenicol resistance, high copy
- `BBa_pSB1A3`: Ampicillin resistance, high copy

#### 2.1.6 Composite Parts (Type: COM)
Multi-part assemblies with integrated function.

**Naming Convention**: `BBa_K[number]`

**Properties**:
- Component parts list
- Assembly method
- Expected function
- Validation status

**Examples**:
- `BBa_K123456`: GFP expression device
- `BBa_K654321`: Inducible protein production system

#### 2.1.7 Regulatory Elements (Type: REG)
Protein binding sites, operators, and other regulatory sequences.

**Properties**:
- Binding protein/molecule
- Regulatory mechanism
- Affinity/binding constant

### 2.2 Part Status

Each part has a lifecycle status:

| Status | Code | Description |
|--------|------|-------------|
| Planning | PL | Part design phase |
| Ordered | OR | DNA synthesis ordered |
| Available | AV | Part available in registry |
| Characterized | CH | Experimental data available |
| Validated | VA | Independently verified |
| Deprecated | DP | Superseded or problematic |
| Deleted | DL | Removed from registry |

### 2.3 Safety Classification

Parts are categorized by biosafety level:

```
BSL-1: Minimal risk (e.g., GFP, standard promoters)
BSL-2: Moderate risk (e.g., mammalian cell work)
BSL-3: Serious risk (e.g., pathogenic organism parts)
BSL-4: Extreme risk (restricted access)
```

---



## 3. Metadata Standards

### 3.1 Required Metadata

Every registered part MUST include:

```json
{
  "partId": "BBa_K123456",
  "partName": "Strong GFP Expression Device",
  "type": "composite",
  "subtype": "reporter",
  "status": "available",
  "safetyLevel": "BSL-1",
  "author": {
    "name": "Team iGEM 2024",
    "affiliation": "MIT",
    "email": "igem@mit.edu"
  },
  "created": "2024-06-15T10:30:00Z",
  "modified": "2024-08-20T14:45:00Z",
  "version": "1.2.0",
  "license": "CC-BY-4.0"
}
```

### 3.2 Sequence Metadata

```json
{
  "sequence": {
    "nucleotides": "ATGCGTAAAGGAGAAGAACTTTTC...",
    "length": 1234,
    "format": "raw|fasta|genbank|sbol",
    "sequenceType": "dna|rna|protein",
    "checksum": "sha256:abc123...",
    "features": [
      {
        "type": "promoter",
        "start": 1,
        "end": 35,
        "strand": "+",
        "name": "J23100"
      },
      {
        "type": "RBS",
        "start": 40,
        "end": 52,
        "strand": "+",
        "name": "B0034"
      }
    ]
  }
}
```

### 3.3 Biological Context

```json
{
  "organism": "Escherichia coli K12",
  "strain": "DH5α",
  "growthConditions": {
    "temperature": 37,
    "medium": "LB",
    "antibiotics": ["chloramphenicol"],
    "inducers": []
  },
  "expression": {
    "expressionHost": "E. coli",
    "compartment": "cytoplasm",
    "tags": ["His6"],
    "fusionPartners": []
  }
}
```

### 3.4 Documentation

```json
{
  "description": "Detailed part description...",
  "designNotes": "Engineering considerations...",
  "references": [
    {
      "type": "publication",
      "doi": "10.1038/nature12345",
      "title": "Novel promoter characterization",
      "authors": ["Smith J", "Doe A"],
      "year": 2024
    }
  ],
  "externalLinks": [
    {
      "database": "GenBank",
      "accession": "AB123456"
    },
    {
      "database": "iGEM",
      "url": "http://parts.igem.org/Part:BBa_K123456"
    }
  ]
}
```

---



## 4. Sequence Formats

### 4.1 Supported Formats

The registry MUST support the following sequence formats:

#### 4.1.1 Raw DNA/RNA
Plain nucleotide sequence:
```
ATGCGTAAAGGAGAAGAACTTTTCACTGGAGTTGTCCCAATTCTTGTTGAATTAGATGGTGATGTTAATGGGCAC
```

#### 4.1.2 FASTA Format
```
>BBa_E0040 Green Fluorescent Protein
ATGCGTAAAGGAGAAGAACTTTTCACTGGAGTTGTCCCAATTCTTGTTGAATTAGATGGT
GATGTTAATGGGCACAAATTTTCTGTCAGTGGAGAGGGTGAAGGTGATGCAACATACGGA
AAACTTACCCTTAAATTTATTTGCACTACTGGAAAACTACCTGTTCCATGGCCAACACTT
```

#### 4.1.3 GenBank Format
```
LOCUS       BBa_E0040                720 bp    DNA     linear   SYN 15-JUN-2024
DEFINITION  Green Fluorescent Protein from jellyfish Aequorea victoria
ACCESSION   BBa_E0040
VERSION     BBa_E0040.1
KEYWORDS    .
SOURCE      synthetic construct
  ORGANISM  synthetic construct
FEATURES             Location/Qualifiers
     source          1..720
                     /organism="synthetic construct"
     CDS             1..720
                     /codon_start=1
                     /product="green fluorescent protein"
                     /protein_id="BBa_E0040"
                     /translation="MSKGEELFTGVVPILVELDGDVNGHKFSVSGEGEGDAT..."
ORIGIN
        1 atgcgtaaag gagaagaact tttcactgga gttgtcccaa ttcttgttga attagatggt
       61 gatgttaatg ggcacaaatt ttctgtcagt ggagagggtg aaggtgatgc aacatacgga
```

#### 4.1.4 SBOL (RDF/XML)
Synthetic Biology Open Language format for maximum interoperability.

### 4.2 Format Conversion

Implementations MUST provide bidirectional conversion between all supported formats.

### 4.3 Sequence Validation

All sequences MUST pass validation:

1. **Character Validation**: Only valid nucleotide codes (A, T, C, G, U, N, etc.)
2. **Length Validation**: Minimum 1 bp, maximum 1,000,000 bp
3. **Checksum Verification**: SHA-256 hash for integrity
4. **Feature Consistency**: Features must not extend beyond sequence boundaries
5. **Biological Validity**: Optional checks for forbidden sequences, restriction sites

---



## 5. SBOL Compliance

### 5.1 SBOL Version Support

Registry MUST support SBOL 2.3.0 or higher.

### 5.2 ComponentDefinition

Each part is represented as an SBOL ComponentDefinition:

```xml
<sbol:ComponentDefinition rdf:about="http://wiastandards.com/BBa_K123456">
    <sbol:persistentIdentity rdf:resource="http://wiastandards.com/BBa_K123456"/>
    <sbol:displayId>BBa_K123456</sbol:displayId>
    <sbol:version>1.2.0</sbol:version>
    <dcterms:title>Strong GFP Expression Device</dcterms:title>
    <dcterms:description>Composite part for constitutive GFP expression</dcterms:description>
    <sbol:type rdf:resource="http://www.biopax.org/release/biopax-level3.owl#DnaRegion"/>
    <sbol:role rdf:resource="http://identifiers.org/so/SO:0000804"/>
    <sbol:sequence rdf:resource="http://wiastandards.com/BBa_K123456_sequence"/>
</sbol:ComponentDefinition>
```

### 5.3 Sequence Object

```xml
<sbol:Sequence rdf:about="http://wiastandards.com/BBa_K123456_sequence">
    <sbol:persistentIdentity rdf:resource="http://wiastandards.com/BBa_K123456_sequence"/>
    <sbol:displayId>BBa_K123456_sequence</sbol:displayId>
    <sbol:version>1.2.0</sbol:version>
    <sbol:elements>ATGCGTAAAGGAGAAGAACTTTTC...</sbol:elements>
    <sbol:encoding rdf:resource="http://www.chem.qmul.ac.uk/iubmb/misc/naseq.html"/>
</sbol:Sequence>
```

### 5.4 Component Hierarchy

Composite parts MUST define component relationships:

```xml
<sbol:Component rdf:about="http://wiastandards.com/BBa_K123456/promoter">
    <sbol:definition rdf:resource="http://wiastandards.com/BBa_J23100"/>
    <sbol:access rdf:resource="http://sbols.org/v2#public"/>
</sbol:Component>
```

---




---

## A.1 Part-classification envelope

The canonical part envelope carries: part identifier (BBa- or registry-local prefix), part name, part type (promoter, ribosome-binding site, coding sequence, terminator, regulatory element, composite), DNA sequence, length, GC content, codon-usage profile (where relevant), and the chassis organisms it has been characterised in. Part types follow the SBOL Component Definition vocabulary plus the IGEM Registry conventions; new types require a registry-level proposal and review.

## A.2 Metadata standard

Metadata for every part includes: design provenance (designer, date, design tool), lab provenance (constructing lab, batch, date), sequencing verification (Sanger or NGS read coverage, variant calls), characterisation context (chassis, growth medium, induction conditions), and licensing terms (Phase 3 §A.2). Metadata is signed by the contributing institution's key so downstream consumers can verify provenance.

## A.3 Sequence formats

The registry accepts FASTA (sequence only), GenBank (.gb / .gbk; sequence + features + annotations), FASTQ (raw reads), BAM/CRAM (aligned reads), and SBOL 3.x JSON (structured component data). Internal canonical storage normalises to SBOL 3.x; export endpoints emit any supported format on demand. Round-trip preservation of comments, dates, and free-text notes is guaranteed for GenBank and SBOL.

## A.4 SBOL conformance

WIA-conformant registries support SBOL 2.3.0 and SBOL 3.x. Import accepts both; export defaults to SBOL 3.x with a compatibility flag for clients that still require 2.3.0. Required SBOL classes include `Component`, `Sequence`, `SequenceFeature`, and `Implementation`. Compositional relationships use `subComponent` rather than the deprecated `Module`.

## A.5 Identifier and versioning

Part identifiers carry a registry prefix (e.g., `BBa_` for the iGEM Parts Registry, `WIA_` for WIA-managed registries), a unique alphanumeric body, and an optional version suffix. New versions of an existing part receive a new identifier; the prior version's record is preserved with a `replacedBy` link so consumers can pick the version they intend.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/synthetic-bio-registry/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-synthetic-bio-registry-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/synthetic-bio-registry-host:1.0.0` ships every synthetic-bio-registry envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/synthetic-bio-registry.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Synthetic-bio-registry deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
