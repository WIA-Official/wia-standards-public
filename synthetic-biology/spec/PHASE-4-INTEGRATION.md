# WIA-BIO-012 — Phase 4: Integration

> Synthetic-biology canonical Phase 4: ecosystem integration (biosafety + implementation + reproducibility).

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


## 8. Biosafety and Biosecurity

### 8.1 Biosafety Levels

| Level | Organisms | Containment |
|-------|-----------|-------------|
| BSL-1 | Non-pathogenic | Basic lab practices |
| BSL-2 | Moderate risk | BSL-1 + biosafety cabinet |
| BSL-3 | Serious disease | BSL-2 + specialized equipment |
| BSL-4 | Lethal pathogens | Maximum containment |

### 8.2 Risk Assessment Matrix

```
Risk = Likelihood × Severity

Likelihood: 1 (rare) to 5 (common)
Severity: 1 (negligible) to 5 (catastrophic)

Risk Level:
1-5: Low (acceptable)
6-10: Medium (review required)
11-15: High (mitigation required)
16-25: Extreme (prohibit)
```

### 8.3 Biocontainment Strategies

#### 8.3.1 Kill Switches
```
Toxin-antitoxin system:
- Constitutive toxin expression
- Inducible antitoxin expression
- Removal of inducer → cell death
```

Examples:
- ccdB/ccdA (DNA gyrase inhibition)
- mazF/mazE (mRNA cleavage)
- hok/sok (membrane depolarization)

#### 8.3.2 Auxotrophy
```
Delete essential biosynthesis genes:
- ΔthyA (thymine)
- ΔdapA (diaminopimelic acid)
- Multiple deletions for redundancy
```

#### 8.3.3 Semantic Containment
```
Recode organism with non-standard genetic code:
- Replace UAG (amber) codon function
- Require synthetic amino acids
- Orthogonal translation system
```

### 8.4 Dual-Use Research Concerns

Screen for:
- Enhanced pathogenicity
- Increased transmissibility
- Antibiotic resistance transfer
- Toxin production
- Immune evasion

Mitigation:
- Institutional Biosafety Committee (IBC) review
- Ethics committee approval
- Secure data/materials handling
- Publication review

---



## 9. Implementation Guidelines

### 9.1 Required Components

Any WIA-BIO-012 compliant system must include:

1. **Part Registry**: Documented biological parts
2. **Assembly Protocol**: DNA construction method
3. **Circuit Model**: Computational predictions
4. **Safety Assessment**: Risk analysis
5. **Documentation**: Complete characterization data

### 9.2 API Interface

#### 9.2.1 Calculate Promoter Strength
```typescript
interface PromoterParams {
  pmax: number;              // Maximum strength (RPU)
  inducerConcentration: number; // M
  kd: number;                // Dissociation constant (M)
  hillCoefficient: number;   // Cooperativity
}

interface PromoterResponse {
  strength: number;          // Current strength (RPU)
  foldInduction: number;     // Induced/basal ratio
  saturation: number;        // % of maximum (0-100)
}
```

#### 9.2.2 Design Genetic Circuit
```typescript
interface CircuitDesign {
  parts: string[];           // BioBrick IDs
  host: string;              // Chassis organism
  purpose: string;           // Circuit function
  optimization?: 'speed' | 'yield' | 'robustness';
}

interface CircuitResult {
  expectedExpression: number;
  plasmidSize: number;
  warnings: string[];
  assemblyMethod: string;
}
```

#### 9.2.3 Optimize Pathway
```typescript
interface PathwayOptimization {
  target: string;            // Product
  substrate: string;         // Input
  host: string;              // Organism
  constraints: {
    maxGenes?: number;
    targetYield?: number;    // mol/mol
    cofactorBalance?: boolean;
  };
}

interface PathwayResult {
  genes: string[];
  theoreticalYield: number;
  fluxDistribution: Record<string, number>;
  bottlenecks: string[];
}
```

### 9.3 Data Formats

#### 9.3.1 BioBrick Part
```json
{
  "id": "BBa_J23100",
  "type": "promoter",
  "sequence": "TTGACAGCTAGCTCAGTCCTAGGTATTATGCTAGC",
  "length": 35,
  "strength": 1791,
  "organism": "E. coli",
  "characterization": {
    "method": "flow_cytometry",
    "units": "RPU",
    "conditions": "LB, 37C, exponential phase"
  }
}
```

#### 9.3.2 Genetic Circuit
```json
{
  "id": "CIR-001",
  "name": "Inducible GFP",
  "parts": [
    "BBa_R0011",
    "BBa_B0034",
    "BBa_E0040",
    "BBa_B0015"
  ],
  "host": "E. coli DH5α",
  "plasmid": "pSB1C3",
  "inducer": {
    "name": "IPTG",
    "concentration": 1e-3,
    "units": "M"
  }
}
```

### 9.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| B001 | Invalid part combination | Check compatibility |
| B002 | Assembly method mismatch | Update part design |
| B003 | Exceeded size limit | Reduce construct size |
| B004 | Biosafety violation | Revise design |
| B005 | Expression too high | Weaken promoter/RBS |
| B006 | Pathway imbalance | Adjust enzyme levels |

---



## 10. References

### 10.1 Foundational Papers

1. Elowitz, M.B. & Leibler, S. (2000). "A synthetic oscillatory network of transcriptional regulators"
3. Endy, D. (2005). "Foundations for engineering biology"
4. Keasling, J.D. (2008). "Synthetic biology for synthetic chemistry"

### 10.2 Assembly Standards

- RFC10: BioBrick Assembly
- RFC25: Freiburg Assembly
- RFC1000: MoClo Assembly
- RFC43: iBrick Assembly

### 10.3 Biological Constants

| Constant | Symbol | Value | Unit |
|----------|--------|-------|------|
| RNA polymerase speed | v_RNAP | 40-80 | nt/s |
| Ribosome speed | v_rib | 15-20 | aa/s |
| mRNA half-life | t₁/₂ | 3-8 | min |
| Protein half-life | t₁/₂ | 10-120 | min |
| Plasmid copy number | n | 15-300 | copies/cell |

### 10.4 WIA Standards

- WIA-INTENT: Intent-based biological queries
- WIA-OMNI-API: Universal API gateway
- WIA-DATA: Data management standards
- WIA-SOCIAL: Collaborative engineering

---

## Appendix A: Example Calculations

### A.1 Promoter Strength Calculation

```
Given:
- Pmax = 1000 RPU (BBa_J23100)
- [IPTG] = 1 mM = 1×10⁻³ M
- Kd = 100 nM = 1×10⁻⁷ M
- n = 2 (Hill coefficient)

Calculation:
P = 1000 × (1×10⁻³)² / (1×10⁻⁷ + (1×10⁻³)²)
P = 1000 × 1×10⁻⁶ / (1×10⁻⁷ + 1×10⁻⁶)
P = 1000 × 1×10⁻⁶ / 1.1×10⁻⁶
P ≈ 909 RPU

Result: ~91% of maximum strength
```

### A.2 Gene Expression Dynamics

```
Given:
- k_tx = 0.5 /s (transcription)
- k_tl = 0.1 /s (translation)
- k_deg_m = 0.231 /min (mRNA, t₁/₂ = 3 min)
- k_deg_p = 0.0069 /min (protein, t₁/₂ = 100 min)

Steady state:
[mRNA]_ss = k_tx / k_deg_m = 0.5 / 0.00385 = 130 molecules
[Protein]_ss = k_tl × [mRNA]_ss / k_deg_p
             = 0.1 × 130 / 0.0069
             ≈ 1,884 molecules

Time to 90% steady state:
t_90 ≈ 2.3 / k_deg_p = 2.3 / 0.0069 ≈ 333 minutes
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-012 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*



---

## A.1 Biosafety regulatory cross-walk

| Concern                          | Standard / Regulation                         |
|----------------------------------|-----------------------------------------------|
| Recombinant DNA research (US)    | NIH Guidelines for Recombinant DNA Research   |
| Recombinant DNA research (EU)    | EU Directive 2009/41/EC (contained use)       |
| Recombinant DNA research (KR)    | LMO Act (생명공학기술의 안전관리에 관한 법률)    |
| Biosafety levels                 | WHO Laboratory Biosafety Manual 4th edition   |
| Dual-use research                | WHO Framework for Responsible Life Sciences   |
| Sequence-order screening         | IGSC Harmonized Screening Protocol            |
| Cross-border movement (LMO)      | Cartagena Protocol on Biosafety               |
| Genetic resources access         | Nagoya Protocol on Access and Benefit-sharing |
| Lab competence                   | ISO/IEC 17025                                 |

## A.2 Implementation guidelines (operational)

A first deployment typically follows: (1) stand up the reference container; (2) import a canonical chassis library and a curated part set; (3) wire computational design adapters (Cello, j5, RAVEN); (4) onboard a sequencing-verification pipeline; (5) connect the synthesis vendor with IGSC screening; (6) onboard the institutional biosafety committee approval workflow; (7) promote to production with the runbook in §Z.5 followed.

## A.3 Reproducibility and FAIR data

Synthetic-biology data is published under the FAIR principles (Findable, Accessible, Interoperable, Reusable). The registry endpoints (Phase 2) plus the metadata block (Phase 1) plus the licensing block (Phase 3) collectively satisfy FAIR for parts, characterisation, and circuit designs. Data citations follow the DataCite metadata schema with a DOI minted per dataset.

## A.4 Reference container, CLI, governance

The reference container at `wia/synthetic-biology-host:1.0.0` ships every Phase 2 endpoint with mock data and feeds the conformance suite. The companion CLI at `cli/synthetic-biology.sh` ships sample envelope generators for parts, circuits, simulations, and assembly plans. WIA Standards composition: WIA-INTENT for workload intent, WIA-OMNI-API for credential storage, WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for federation handshake.

## A.5 Reference list

- SBOL 3.x specification (Synthetic Biology Open Language)
- BioBricks Foundation Request for Comments (RFC) series — RFC10, RFC25, RFC43, RFC1000
- NCBI GenBank flat-file specification
- COBRA Toolbox / cobrapy — flux-balance analysis
- WHO Laboratory Biosafety Manual, 4th edition (LBM4)
- NIH Guidelines for Research Involving Recombinant or Synthetic Nucleic Acid Molecules
- IGSC Harmonized Screening Protocol
- Cartagena Protocol on Biosafety
- Nagoya Protocol on Access and Benefit-sharing
- ISO/IEC 17025 — competence of testing and calibration laboratories
- DataCite Metadata Schema 4.4


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
