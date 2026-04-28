# WIA-BIO-012 — Phase 3: Protocol

> Synthetic-biology canonical Phase 3: protocols (metabolic engineering + cell-free + characterisation).

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


## 5. Metabolic Engineering

### 5.1 Flux Balance Analysis (FBA)

Optimization problem:
```
Maximize: c^T × v
Subject to: S × v = 0
            v_min ≤ v ≤ v_max
```

Where:
- `v` = Flux vector
- `S` = Stoichiometric matrix
- `c` = Objective function coefficients

### 5.2 Michaelis-Menten Kinetics

Enzyme reaction rate:
```
v = (v_max × [S]) / (Km + [S])
```

For multiple substrates:
```
v = (v_max × [A] × [B]) / ((Ka + [A]) × (Kb + [B]))
```

### 5.3 Metabolic Control Analysis

Flux control coefficient:
```
C_E^J = (∂J/∂E) × (E/J)
```

Where:
- `C_E^J` = Control coefficient of enzyme E on flux J
- `∂J/∂E` = Change in flux per change in enzyme
- `E/J` = Normalization factor

Summation theorem:
```
Σ C_i^J = 1
```

### 5.4 Pathway Optimization

Dynamic programming approach:

```
1. Identify rate-limiting steps
2. Calculate flux control coefficients
3. Overexpress high-control enzymes
4. Delete competing pathways
5. Balance cofactor usage
6. Optimize gene expression levels
```

Theoretical maximum yield:
```
Y_max = (mol product / mol substrate) × stoichiometry
```

---



## 6. Cell-Free Systems

### 6.1 PURE System Components

Minimal protein synthesis system:

| Component | Concentration | Function |
|-----------|---------------|----------|
| T7 RNAP | 100 nM | Transcription |
| Ribosomes | 1.5 μM | Translation |
| Amino acids | 2 mM each | Protein building blocks |
| NTPs | 1.5 mM each | Energy, transcription |
| tRNAs | Variable | Translation |
| Factors | Variable | Initiation, elongation |

### 6.2 Crude Extract Systems

E. coli S30 extract:

```
Protein synthesis rate = 1-5 μg/mL/hr
Duration: 4-8 hours
Yield: 100-2000 μg/mL
```

Optimization parameters:
- Mg²⁺ concentration: 12-18 mM
- K⁺ concentration: 80-150 mM
- pH: 7.2-7.8
- Temperature: 30-37°C

### 6.3 Applications

1. **Rapid Prototyping**: Test genetic circuits in hours
2. **Toxic Proteins**: Express without killing cells
3. **Non-Natural Chemistry**: Use non-standard amino acids
4. **Metabolic Engineering**: Cell-free metabolic pathways
5. **Biosensors**: In vitro diagnostic systems

---




---

## A.1 Metabolic-engineering protocol

Metabolic engineering combines flux-balance analysis (FBA), 13C metabolic flux analysis (MFA), and dynamic kinetic modelling to redirect carbon and energy flux toward target products. The protocol layer wraps COBRA Toolbox / cobrapy interfaces and exposes them through the WIA envelope so users get the same envelope shape regardless of the underlying solver.

## A.2 Flux-balance analysis

FBA solves `max c^T v` subject to `S v = 0` and `lb ≤ v ≤ ub`, where `S` is the stoichiometric matrix, `v` is the flux vector, and `c` is the objective vector. Reference genome-scale models: iJO1366 (E. coli), iML1515 (E. coli BW25113), Yeast-GEM (S. cerevisiae), CHO-GEM (Chinese hamster ovary). The protocol exposes single-objective and multi-objective FBA.

## A.3 Cell-free reaction protocol

Cell-free reactions follow a canonical recipe: lysate (E. coli S30, PURE), energy regeneration (creatine phosphate or 3-PGA + nucleotides), tRNA mix (or pre-tRNA-loaded synthetases for PURE), DNA template, magnesium and potassium salts, polyethylene glycol crowding agent. The protocol layer exposes the recipe envelope so downstream automation (acoustic dispensers, multi-channel pipettors) consumes one canonical record.

## A.4 Genetic-circuit characterisation protocol

Characterisation runs follow a standard schedule: overnight pre-culture → dilution to OD600 0.05 → growth to mid-log → induction at 50% time-point → sampling at 6 time points over 8 hours → instrument readout (plate reader fluorescence, flow cytometry single-cell, qPCR transcript). Results carry the protocol identifier so re-runs can be compared apples-to-apples.

## A.5 Biosafety and biosecurity protocol

Risk assessment is mandatory before construction. The protocol covers chassis-level risk (BSL classification per WHO LBM4 4th edition), part-level risk (toxin sequences, virulence genes, dual-use sequences screened against the IGSC Harmonized Screening Protocol), and project-level risk (NIH Guidelines categories, institutional biosafety committee approval). Synthesis orders are routed only to vendors that perform IGSC screening.

## A.6 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for control-plane calls. Lab-execution requests are idempotent on the assembly-plan hash; duplicates short-circuit to the cached lab job.

## A.7 Reproducibility protocol

Every characterisation run is assigned a reproducibility identifier that captures the software stack version, the chassis lot, the medium recipe, the instrument calibration date, and the ambient laboratory conditions (temperature, humidity). Re-runs that target the same reproducibility identifier must hit the same canonical recipe; deviations are flagged and curated.

## A.8 Lab-automation integration protocol

Lab-automation integration covers acoustic dispensers (Echo, Mosquito), multi-channel pipettors (Hamilton STAR, Tecan Fluent), thermocyclers (Bio-Rad, Eppendorf), plate readers (Tecan Spark, BMG ClarioStar), and flow cytometers (BD FACSAria, Cytek Aurora). Each instrument exposes a canonical command envelope with the instrument-specific extensions in a `vendor:` block; consumers handle the canonical part and ignore vendor extensions unless they target that vendor.

## A.9 IBC and IRB workflow protocol

Institutional Biosafety Committee (IBC) and Institutional Review Board (IRB) approvals integrate as a pre-condition on the assembly-execution endpoint: a request without an active IBC approval is rejected with a typed `compliance.ibc_required` error. The IBC approval envelope carries the protocol identifier, the approval expiry, the approving committee's institutional identifier, and a signed approval document hash.

## A.10 Failure-mode escalation

Common failure modes in synthetic-biology execution: assembly-yield collapse (low ligation efficiency), sequence verification mismatch (Sanger reads disagree with design), expression-level deviation (>3× from predicted), unexpected toxicity (chassis growth defect). Each failure mode emits a typed `failure.{mode}` event with the suggested remediation drawn from the canonical troubleshooting matrix. Operations integration routes critical events (toxicity, biosafety-relevant) to the on-call IBC liaison.

## A.11 Iterative-design loop

Design-build-test-learn cycles consume the canonical envelopes at every stage. The iterative-design loop carries: design version, build success/failure, test characterisation envelope, learn-step parameter updates. Each iteration is signed by the responsible lab and stamped with a sequence number; the loop history is queryable so downstream researchers can see what was already tried and why it didn't work, before reproducing the failure independently.

## A.12 Cell-free reaction-execution protocol

Cell-free reactions follow a canonical execution sequence: thaw lysate → assemble reaction (lysate + energy mix + tRNA + Mg²⁺ + K⁺ + DNA template) → optionally pre-warm → run isothermally at 30°C (E. coli) or 37°C (PURE high-temp variant) for the design duration (typically 4–16 h) → quench at 4°C → readout (fluorescence kinetics during the run, or end-point Western, mass spec, activity assay). Microfluidic continuous-flow variants follow the same envelope with the addition of flow-rate and residence-time fields.

## A.13 Sequence-verification protocol

Verification follows: extract DNA → PCR amplify the assembled region → Sanger sequence (≥2× coverage) or Illumina amplicon sequencing (≥30× coverage) → align to design → call variants → emit a per-position concordance map. Discordances above the noise floor block release of the construct to characterisation; the lab must either repeat the build or accept the variant after IBC review.

## A.14 Strain-banking and storage protocol

Verified strains are deposited in a strain bank: -80°C glycerol stock at 25% glycerol (v/v), cryotube labelled with strain-id, parental strain, plasmid(s), antibiotic resistance, source date, and depositor. Backup vials live in a geographically separated facility. Public deposits go to ATCC, DSMZ, or NBRC according to the consortium policy; private deposits stay institutional.

## A.15 Decommissioning and biological-waste protocol

End-of-life strains and constructs are inactivated by autoclaving (121°C / 15 min for liquid waste, 134°C / 30 min for solid waste containing recombinant DNA) before disposal. Destruction events are logged with the strain-id, the operator, the autoclave-cycle log, and a witness signature. The destruction record is preserved indefinitely so that downstream researchers cannot accidentally retrieve a decommissioned strain.


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
