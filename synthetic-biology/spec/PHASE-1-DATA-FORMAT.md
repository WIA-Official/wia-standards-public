# WIA-BIO-012 — Phase 1: Data Format

> Synthetic-biology canonical Phase 1: genetic-part standards + chassis descriptors.

# WIA-BIO-012: Synthetic Biology Specification v1.0

> **Standard ID:** WIA-BIO-012
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Genetic Part Standards](#2-genetic-part-standards)
3. [DNA Assembly Methods](#3-dna-assembly-methods)
4. [Genetic Circuit Design](#4-genetic-circuit-design)
5. [Metabolic Engineering](#5-metabolic-engineering)
6. [Cell-Free Systems](#6-cell-free-systems)
7. [Computational Design Tools](#7-computational-design-tools)
8. [Biosafety and Biosecurity](#8-biosafety-and-biosecurity)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---


## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for synthetic biology, enabling the design, construction, and optimization of engineered biological systems for beneficial applications.

### 1.2 Scope

The standard covers:
- Standardized genetic parts (BioBricks, MoClo, etc.)
- DNA assembly protocols
- Genetic circuit modeling and design
- Metabolic pathway engineering
- Cell-free expression systems
- Biosafety and ethical guidelines

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize synthetic biology while ensuring responsible development that benefits humanity and protects the environment.

### 1.4 Terminology

- **BioBrick**: Standardized DNA part with defined prefix/suffix sequences
- **Genetic Circuit**: Network of regulatory genetic elements
- **Metabolic Flux**: Rate of flow through a metabolic pathway
- **Chassis Organism**: Host organism for engineered genetic systems
- **Part Registry**: Repository of characterized biological parts

---



## 2. Genetic Part Standards

### 2.1 BioBrick Standard (RFC10)

The BioBrick standard defines standardized DNA parts with compatible restriction sites:

```
Prefix:  EcoRI - XbaI
Part:    [Genetic Element]
Suffix:  SpeI - PstI
```

Assembly format:
```
5'-GAATTC-GCGGCCGC-T-TCTAGA-[Part]-TACTAG-AGCGGCCGC-CTGCAG-3'
     EcoRI   NotI      XbaI          SpeI    NotI    PstI
```

### 2.2 Part Categories

#### 2.2.1 Promoters
Control transcription initiation:

| Part | Type | Strength | Organism |
|------|------|----------|----------|
| BBa_J23100 | Constitutive | 1791 RPU | E. coli |
| BBa_J23119 | Constitutive | 1 RPU | E. coli |
| BBa_R0011 | Inducible (LacI) | Variable | E. coli |
| BBa_R0040 | Inducible (TetR) | Variable | E. coli |

Strength calculation:
```
P = Pmax × f(inducer)
```

For Hill function regulation:
```
P = Pmax × [I]ⁿ / (Kd + [I]ⁿ)
```

Where:
- `P` = Promoter activity (RPU)
- `Pmax` = Maximum activity
- `[I]` = Inducer concentration
- `Kd` = Dissociation constant
- `n` = Hill coefficient (cooperativity)

#### 2.2.2 Ribosome Binding Sites (RBS)
Control translation initiation:

| Part | Type | Strength | ΔG (kcal/mol) |
|------|------|----------|---------------|
| BBa_B0034 | Strong | High | -12.1 |
| BBa_B0032 | Medium | Medium | -9.8 |
| BBa_B0033 | Weak | Low | -6.5 |

Translation initiation rate:
```
TIR = K × exp(-ΔG / RT)
```

Where:
- `TIR` = Translation Initiation Rate
- `K` = Proportionality constant
- `ΔG` = Free energy of ribosome binding
- `R` = Gas constant (1.987 cal/mol·K)
- `T` = Temperature (K)

#### 2.2.3 Coding Sequences (CDS)
Protein-coding regions:

| Part | Protein | Function | Size (bp) |
|------|---------|----------|-----------|
| BBa_E0040 | GFP | Fluorescence | 720 |
| BBa_E1010 | RFP | Fluorescence | 678 |
| BBa_K082003 | Luciferase | Bioluminescence | 1653 |

Codon optimization:
```
CAI = exp(Σ ln(w_i) / L)
```

Where:
- `CAI` = Codon Adaptation Index
- `w_i` = Relative adaptiveness of codon i
- `L` = Number of codons

#### 2.2.4 Terminators
Stop transcription:

| Part | Type | Efficiency |
|------|------|------------|
| BBa_B0015 | Double | >95% |
| BBa_B0010 | Single | ~85% |

Termination efficiency:
```
η = (T_background - T_part) / T_background
```

Where:
- `η` = Termination efficiency
- `T_background` = Background transcription
- `T_part` = Transcription with terminator

### 2.3 MoClo (Modular Cloning) Standard

Hierarchical assembly using Type IIS restriction enzymes:

```
Level 0: Individual parts
Level 1: Transcription units (promoter-RBS-CDS-terminator)
Level 2: Multi-gene constructs
Level 3+: Complex assemblies
```

Golden Gate reaction:
```
BsaI/BpiI digestion + T4 ligase
37°C (digestion) / 16°C (ligation) cycles
```

---




---

## A.1 Genetic-part envelope (canonical)

The canonical genetic-part envelope carries: part identifier (BBa- or registry-local prefix), part type (promoter, RBS, CDS, terminator, regulatory, composite), DNA sequence, prefix/suffix specification (BioBrick RFC10 / RFC25 / RFC1000 compliance), characterisation context (chassis, medium, induction), and the responsible-use licensing block. Parts conforming to multiple assembly standards declare each conformance separately so downstream tooling can select the right assembly path.

## A.2 BioBrick (RFC10) prefix and suffix

RFC10 BioBricks carry the canonical 22-bp prefix `GAATTCGCGGCCGCTTCTAG` and 21-bp suffix `TACTAGTAGCGGCCGCTGCAG`, with EcoRI/XbaI/SpeI/PstI restriction sites. The prefix immediately precedes the part body; the suffix immediately follows. Part bodies MUST NOT contain internal occurrences of the four canonical sites; sequence-scrubbing tooling is part of the validation pipeline.

## A.3 MoClo (RFC1000) overhang specification

MoClo Type IIS assembly uses BsaI / BsmBI-mediated digestion to expose 4-bp overhangs with defined identity per part position. Position 1 (left fusion) overhangs follow the canonical MoClo Level 0 conventions; the registry stores the overhang per part so downstream MoClo planning is trivially correct. Each genetic part declaration MUST list its position-class so that mis-ordered Level 1 builds can be flagged before synthesis.

## A.4 Chassis organism descriptor

Chassis descriptors carry the species (E. coli K-12 MG1655, BL21(DE3), W3110; B. subtilis 168; S. cerevisiae BY4741, CEN.PK; Y. lipolytica; mammalian HEK-293, CHO; cell-free PURE, S30 lysate), the growth medium, the temperature, the induction profile, and the safety classification (BSL-1, BSL-2, BSL-3, BSL-4 per WHO LBM4).

## A.5 Sequence-format and digital-twin descriptor

Genetic parts are stored in SBOL 3.x with cross-references to GenBank flat files. Digital twin descriptors capture the kinetic-parameter envelope (transcription rate, translation rate, plasmid copy number, mRNA half-life, protein half-life, degradation rate) so simulation tooling consumes the same canonical record without independent re-entry of values.

## A.6 Promoter-strength characterisation

Promoter strength is measured in Relative Promoter Units (RPU) anchored to a reference promoter (typically BBa_J23101). The characterisation envelope carries: chassis, medium, induction state, fluorescence-protein reporter, plate-reader OD-normalisation pipeline, and the computed RPU. The envelope is published with raw fluorescence vs. OD time series so downstream consumers can re-derive the RPU under their own normalisation.

## A.7 RBS strength and Shine-Dalgarno

Ribosome-binding-site (RBS) strength descriptors carry: Shine-Dalgarno consensus distance to the start codon, predicted translation-initiation rate (Salis-lab RBS Calculator output or equivalent), and the chassis the prediction is calibrated for. Implementations MUST list the calibration model used so downstream tools can reconcile divergent predictions across calculators.

## A.8 Cell-free reaction descriptor

Cell-free reaction descriptors carry the lysate type (E. coli S30, PURE, wheat germ, rabbit reticulocyte), the energy-regeneration mix (creatine phosphate-creatine kinase, 3-PGA-pyruvate kinase, glucose-hexokinase), the magnesium and potassium concentrations, the polyethylene-glycol crowding agent concentration, the dialysis-vs-batch flag, and the temperature profile. The reactor descriptor (volumetric, pin, microfluidic continuous flow) is part of the same envelope.

## A.9 Genetic-circuit motif catalogue

Standard motifs the registry recognises: toggle switch, repressilator (3-element negative-feedback ring), feed-forward loop (coherent and incoherent), pulse generator, oscillator, band-pass filter, AND/OR/NAND/NOR logic gates, fold-change detector, and integrator. Each motif carries a canonical SBOL design plus example characterisation data so users can pick a starting circuit instead of designing from scratch.

## A.10 Strain-engineering descriptor

Strain-engineering descriptors capture: parental strain (lineage to a reference genome), genomic modifications (insertions, deletions, point mutations) with chromosomal coordinates, plasmid load, antibiotic-resistance markers, and auxotrophic requirements. Descriptors include a verification block (whole-genome sequencing reference, copy-number qPCR, phenotypic confirmation) so downstream consumers can audit the strain claims.

## A.11 CRISPR-Cas editing descriptor

CRISPR-Cas-mediated edits carry: Cas variant (Cas9 SpCas9 / SaCas9 / Cas12a Cpf1 / Cas13 RNA-targeting / base editor / prime editor), target site (chromosomal coordinates, strand), guide-RNA sequence, predicted off-targets (with predicted scoring tool reference), repair template (HDR-donor sequence) where applicable, and the verification panel that confirmed the edit. Editing events are recorded as first-class envelopes so the strain history is fully auditable.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/synthetic-biology/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-synthetic-biology-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/synthetic-biology-host:1.0.0` ships every synthetic-biology envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/synthetic-biology.sh` ships sample envelope generators with no
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
ecosystem. Synthetic-biology deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
