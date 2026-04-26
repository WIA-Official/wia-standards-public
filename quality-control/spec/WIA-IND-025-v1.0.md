# WIA-IND-025: Quality Control Standard v1.0

**Standard ID:** WIA-IND-025
**Title:** Quality Control Standard
**Version:** 1.0.0
**Status:** Active
**Date:** 2025-12-27
**Category:** IND (Industry)
**Color:** Amber (#F59E0B)
**Emoji:** ✅

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Quality Management System](#5-quality-management-system)
6. [Inspection Protocols](#6-inspection-protocols)
7. [Statistical Process Control (SPC)](#7-statistical-process-control-spc)
8. [Defect Detection and Classification](#8-defect-detection-and-classification)
9. [Quality Metrics](#9-quality-metrics)
10. [Calibration Management](#10-calibration-management)
11. [Non-Conformance Handling](#11-non-conformance-handling)
12. [Corrective and Preventive Action (CAPA)](#12-corrective-and-preventive-action-capa)
13. [Audit Management](#13-audit-management)
14. [Document Control](#14-document-control)
15. [Quality Certifications](#15-quality-certifications)
16. [Data Models](#16-data-models)
17. [API Specifications](#17-api-specifications)
18. [Security and Privacy](#18-security-and-privacy)
19. [Implementation Guidelines](#19-implementation-guidelines)
20. [Compliance and Validation](#20-compliance-and-validation)

---

## 1. Introduction

### 1.1 Purpose

This standard defines a comprehensive framework for quality control (QC) and quality assurance (QA) in manufacturing, production, and service environments. It provides standardized protocols, metrics, and procedures to ensure product quality, safety, and regulatory compliance.

### 1.2 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to protect consumers, enhance product safety, reduce waste, and promote manufacturing excellence worldwide. Quality control is not just about meeting specifications; it's about delivering products and services that improve lives.

### 1.3 Key Objectives

- Establish standardized inspection protocols
- Implement statistical process control (SPC) methodologies
- Enable AI-powered defect detection
- Define industry-standard quality metrics
- Ensure equipment calibration and accuracy
- Systematize non-conformance handling
- Track corrective and preventive actions
- Facilitate quality audits and certifications
- Maintain controlled quality documentation
- Support ISO 9001, ISO 13485, IATF 16949, and other quality standards

### 1.4 Benefits

- **Reduced Defects**: Early detection and prevention
- **Cost Savings**: Lower scrap, rework, and warranty costs
- **Customer Satisfaction**: Consistent quality and reliability
- **Regulatory Compliance**: Meet industry standards and regulations
- **Continuous Improvement**: Data-driven decision making
- **Risk Mitigation**: Proactive issue identification
- **Brand Protection**: Maintain reputation and trust

---

## 2. Scope

### 2.1 Application Domains

This standard applies to:

- Manufacturing (discrete and process)
- Medical device production
- Automotive manufacturing
- Aerospace and defense
- Electronics assembly
- Pharmaceutical manufacturing
- Food and beverage processing
- Chemical production
- Textiles and apparel
- Consumer goods

### 2.2 Quality Control Activities

- Receiving inspection (incoming materials)
- In-process inspection (during production)
- First article inspection (FAI)
- Final inspection (finished goods)
- Statistical process control (SPC)
- Gage R&R (measurement system analysis)
- Process capability studies
- Defect detection and classification
- Root cause analysis
- Corrective and preventive action
- Supplier quality management
- Quality audits (internal and external)

### 2.3 Out of Scope

- Product design and development (covered by WIA-DES standards)
- Supply chain logistics (covered by WIA-IND-023)
- Environmental management systems (ISO 14001)
- Occupational health and safety (ISO 45001)

---

## 3. Normative References

### 3.1 ISO Standards

- **ISO 9001:2015** - Quality Management Systems - Requirements
- **ISO 9000:2015** - Quality Management Systems - Fundamentals and Vocabulary
- **ISO 10012:2003** - Measurement Management Systems
- **ISO 13485:2016** - Medical Devices - Quality Management Systems
- **ISO/TS 16949** - Automotive Quality Management (superseded by IATF 16949)
- **ISO 2859-1** - Sampling Procedures for Inspection by Attributes
- **ISO 3951** - Sampling Procedures for Inspection by Variables

### 3.2 Industry Standards

- **IATF 16949:2016** - Automotive Quality Management System
- **AS9100** - Aerospace Quality Management System
- **FDA 21 CFR Part 820** - Quality System Regulation (Medical Devices)
- **GMP** - Good Manufacturing Practice (Pharmaceuticals)
- **HACCP** - Hazard Analysis and Critical Control Points (Food Safety)
- **IPC Standards** - Electronics Manufacturing (IPC-A-610, IPC-J-STD-001)

### 3.3 Quality Methodologies

- **Six Sigma** - DMAIC (Define, Measure, Analyze, Improve, Control)
- **Lean Manufacturing** - Waste reduction and efficiency
- **Total Quality Management (TQM)** - Organization-wide quality focus
- **APQP** - Advanced Product Quality Planning
- **PPAP** - Production Part Approval Process
- **FMEA** - Failure Mode and Effects Analysis
- **MSA** - Measurement Systems Analysis

---

## 4. Terms and Definitions

### 4.1 Quality Control (QC)

Operational techniques and activities used to fulfill quality requirements. Focuses on detecting defects in finished products.

### 4.2 Quality Assurance (QA)

Part of quality management focused on providing confidence that quality requirements will be fulfilled. Focuses on preventing defects during development and production.

### 4.3 Inspection

Conformity evaluation by observation and judgment, accompanied as appropriate by measurement, testing, or gauging.

### 4.4 Non-Conformance (NC)

Non-fulfillment of a specified requirement. A defect or departure from specified requirements.

### 4.5 Defect

Non-fulfillment of a requirement related to an intended or specified use. More severe than non-conformance.

### 4.6 Critical Defect

A defect that could result in hazardous or unsafe conditions for individuals using, maintaining, or depending upon the product.

### 4.7 Major Defect

A defect that is likely to result in failure or reduced usability of the product.

### 4.8 Minor Defect

A defect that does not reduce the usability of the product for its intended purpose but deviates from specifications.

### 4.9 Statistical Process Control (SPC)

Use of statistical methods to monitor and control a process to ensure it operates at its full potential.

### 4.10 Control Chart

A graphical display of process data over time with statistically determined control limits.

### 4.11 Process Capability

Statistical measure of the inherent process variability relative to specification limits.

### 4.12 Cp (Process Capability)

Ratio of specification tolerance to process variation: Cp = (USL - LSL) / (6σ)

### 4.13 Cpk (Process Capability Index)

Process capability index accounting for process centering: Cpk = min[(USL - μ)/(3σ), (μ - LSL)/(3σ)]

### 4.14 Six Sigma

Quality management methodology targeting 3.4 defects per million opportunities (DPMO).

### 4.15 DPMO (Defects Per Million Opportunities)

Number of defects per million opportunities for a defect to occur.

### 4.16 First Article Inspection (FAI)

Complete inspection of the first unit produced to verify the manufacturing process is capable.

### 4.17 Acceptance Sampling

Inspection of a sample from a lot to decide acceptance or rejection of the entire lot.

### 4.18 AQL (Acceptable Quality Level)

Maximum percentage of defects considered acceptable in acceptance sampling.

### 4.19 Calibration

Set of operations that establish the relationship between measurement standard and measuring instrument.

### 4.20 Traceability

Ability to trace measurement results to national or international standards.

### 4.21 Gage R&R (Repeatability and Reproducibility)

Study to assess the variation in measurement system.

### 4.22 Root Cause Analysis (RCA)

Systematic process for identifying root causes of problems or events.

### 4.23 CAPA (Corrective and Preventive Action)

Improvements to an organization's processes to eliminate causes of non-conformities.

### 4.24 NCR (Non-Conformance Report)

Documented identification and description of a non-conforming product or process.

### 4.25 OEE (Overall Equipment Effectiveness)

Metric for measuring manufacturing productivity: OEE = Availability × Performance × Quality

---

## 5. Quality Management System

### 5.1 QMS Framework

A Quality Management System (QMS) shall include:

1. **Quality Policy** - Top management commitment to quality
2. **Quality Objectives** - Measurable quality goals
3. **Process Approach** - Interrelated processes
4. **Risk-Based Thinking** - Identify and mitigate risks
5. **Continual Improvement** - Ongoing enhancement
6. **Evidence-Based Decision Making** - Data-driven decisions
7. **Relationship Management** - Stakeholder engagement

### 5.2 QMS Documentation

Required documentation includes:

- Quality manual
- Quality policy and objectives
- Documented procedures
- Work instructions
- Quality records
- Forms and templates
- Process flow diagrams
- Control plans

### 5.3 Process Approach

Quality management processes shall be defined, documented, and controlled:

```
PLAN → DO → CHECK → ACT (PDCA Cycle)
  ↑                        ↓
  └────────────────────────┘
```

### 5.4 Context of the Organization

Organizations shall determine:

- Internal and external issues affecting QMS
- Interested parties and their requirements
- Scope of the QMS
- QMS processes and their interactions

### 5.5 Leadership and Commitment

Top management shall:

- Take accountability for QMS effectiveness
- Establish quality policy and objectives
- Ensure QMS integration into business processes
- Promote process approach and risk-based thinking
- Ensure resources are available
- Communicate importance of quality
- Support continuous improvement

### 5.6 Quality Policy

Quality policy shall:

- Be appropriate to organization's purpose
- Provide framework for quality objectives
- Include commitment to satisfy requirements
- Include commitment to continual improvement
- Be communicated and understood
- Be available to interested parties

### 5.7 Roles and Responsibilities

Clear definition of:

- Quality Manager responsibilities
- Process owners
- Inspection personnel
- Calibration technicians
- Internal auditors
- Management representative
- CAPA coordinators

---

## 6. Inspection Protocols

### 6.1 Types of Inspection

#### 6.1.1 Receiving Inspection

Inspection of incoming materials, components, or products from suppliers.

**Objectives:**
- Verify conformance to purchase order specifications
- Prevent defective materials from entering production
- Assess supplier quality performance

**Activities:**
- Visual inspection
- Dimensional verification
- Material certification review
- Functional testing (if applicable)
- Documentation review

**Sampling Plans:**
- Use ISO 2859-1 (AQL sampling)
- Typical AQL levels: 0.65, 1.0, 1.5, 2.5, 4.0
- Adjust sampling based on supplier history

#### 6.1.2 First Article Inspection (FAI)

Complete inspection of first production unit to verify process capability.

**When Required:**
- New product introduction
- Engineering change implementation
- Process change or relocation
- After extended production break
- Supplier change

**Requirements:**
- 100% dimensional inspection
- Material verification
- Functional testing
- Documentation per AS9102 (aerospace) or customer requirements
- Approval before full production

**FAI Report Contents:**
- Part identification
- Revision level
- Inspection results
- Material certifications
- Process verification
- Engineering approval

#### 6.1.3 In-Process Inspection

Monitoring during production to detect and prevent defects.

**Frequency:**
- Continuous (automated systems)
- Periodic (hourly, per batch)
- Statistical sampling
- Risk-based approach

**Methods:**
- Visual inspection
- Dimensional checks
- Functional tests
- SPC monitoring
- Automated inspection (vision systems)

**Control Points:**
- Critical operations
- After key process steps
- Before irreversible operations
- Prior to assembly

#### 6.1.4 Final Inspection

Verification of finished product before shipment.

**Inspection Activities:**
- Visual inspection (cosmetic defects)
- Dimensional verification
- Functional testing
- Packaging verification
- Label and marking check
- Documentation review
- Serialization (if required)

**Release Criteria:**
- All inspections passed
- NCRs resolved
- Documentation complete
- Customer-specific requirements met
- Certificate of Conformance (CoC) issued

### 6.2 Inspection Planning

#### 6.2.1 Inspection Plan

Document defining:

- Inspection points
- Characteristics to inspect
- Acceptance criteria
- Sample size and frequency
- Measurement methods
- Gages and equipment
- Reaction plans for non-conformance

#### 6.2.2 Control Plan

Living document linking:

- Process steps
- Critical characteristics
- Control methods
- Specifications
- Measurement systems
- Sample size and frequency
- Reaction plans

**Control Plan Format:**
```
| Step | Characteristic | Spec | Method | Sample | Freq | Chart | Reaction |
|------|---------------|------|--------|--------|------|-------|----------|
```

### 6.3 Sampling Strategies

#### 6.3.1 Acceptance Sampling (Attributes)

ISO 2859-1 sampling plans:

- **Single Sampling**: Inspect sample, accept/reject based on defects found
- **Double Sampling**: May require second sample if first is inconclusive
- **Multiple Sampling**: Sequential sampling until decision reached

**AQL (Acceptable Quality Level):**
- 0.065 - Critical defects (safety)
- 0.65 - Major defects (functionality)
- 2.5 - Minor defects (cosmetic)

#### 6.3.2 Variables Sampling

ISO 3951 sampling plans:

- Based on measurements (continuous data)
- Smaller sample sizes than attributes
- Requires knowledge of distribution
- More powerful for detecting shifts

#### 6.3.3 100% Inspection

When to use:
- Critical safety characteristics
- High-value products
- Customer requirement
- Process not capable
- Automated inspection available

### 6.4 Inspection Equipment

#### 6.4.1 Measurement Instruments

Common inspection equipment:

- Calipers and micrometers
- Height gages
- Coordinate Measuring Machines (CMM)
- Optical comparators
- Surface roughness testers
- Hardness testers
- Torque wrenches
- Pressure/temperature gages
- Vision systems
- X-ray inspection
- Computed Tomography (CT) scanning

#### 6.4.2 Calibration Requirements

All measurement equipment shall:

- Be calibrated at defined intervals
- Be traceable to national/international standards
- Have calibration status identified
- Be protected from damage
- Be stored properly when not in use

### 6.5 Inspector Qualification

Inspectors shall:

- Be trained on inspection procedures
- Demonstrate competence
- Have vision tested (if applicable)
- Understand specifications and drawings
- Know reaction plans for non-conformance
- Be certified for specialized inspections (NDT, etc.)

---

## 7. Statistical Process Control (SPC)

### 7.1 SPC Overview

Statistical Process Control uses statistical methods to monitor and control processes, ensuring they operate at full potential with minimum variation.

### 7.2 Control Charts

#### 7.2.1 Variables Control Charts

**X-bar and R Chart**

Monitor process mean and range:

```
X-bar Chart (Process Mean):
UCL = X̄̄ + A₂R̄
CL = X̄̄
LCL = X̄̄ - A₂R̄

R Chart (Process Range):
UCL = D₄R̄
CL = R̄
LCL = D₃R̄
```

Where:
- X̄̄ = Grand average of subgroup means
- R̄ = Average of subgroup ranges
- A₂, D₃, D₄ = Constants based on subgroup size

**X-bar and S Chart**

Similar to X-bar and R, but uses standard deviation:

```
X-bar Chart:
UCL = X̄̄ + A₃S̄
CL = X̄̄
LCL = X̄̄ - A₃S̄

S Chart:
UCL = B₄S̄
CL = S̄
LCL = B₃S̄
```

**Individual and Moving Range (I-MR) Chart**

For individual measurements:

```
I Chart:
UCL = X̄ + 2.66MR̄
CL = X̄
LCL = X̄ - 2.66MR̄

MR Chart:
UCL = 3.267MR̄
CL = MR̄
LCL = 0
```

#### 7.2.2 Attributes Control Charts

**P Chart (Proportion Defective)**

```
UCL = p̄ + 3√(p̄(1-p̄)/n)
CL = p̄
LCL = p̄ - 3√(p̄(1-p̄)/n)
```

Where:
- p̄ = Average proportion defective
- n = Sample size

**NP Chart (Number Defective)**

```
UCL = np̄ + 3√(np̄(1-p̄))
CL = np̄
LCL = np̄ - 3√(np̄(1-p̄))
```

**C Chart (Count of Defects)**

```
UCL = c̄ + 3√c̄
CL = c̄
LCL = c̄ - 3√c̄
```

Where c̄ = Average defect count

**U Chart (Defects per Unit)**

```
UCL = ū + 3√(ū/n)
CL = ū
LCL = ū - 3√(ū/n)
```

#### 7.2.3 Special Control Charts

**CUSUM (Cumulative Sum)**

Detects small process shifts:

```
CUSUM⁺ = max[0, CUSUM⁺ᵢ₋₁ + (xᵢ - μ₀) - k]
CUSUM⁻ = max[0, CUSUM⁻ᵢ₋₁ - (xᵢ - μ₀) - k]
```

Where k = allowance (typically 0.5σ)

**EWMA (Exponentially Weighted Moving Average)**

Detects small shifts with weighted average:

```
EWMAᵢ = λxᵢ + (1-λ)EWMAᵢ₋₁

UCL = μ₀ + L√(λσ²/(2-λ))
CL = μ₀
LCL = μ₀ - L√(λσ²/(2-λ))
```

Where:
- λ = weighting factor (0 < λ ≤ 1, typically 0.2)
- L = control limit multiplier (typically 3)

### 7.3 Process Capability Analysis

#### 7.3.1 Capability Indices

**Cp (Process Capability)**

Measures potential capability:

```
Cp = (USL - LSL) / (6σ)
```

**Cpk (Process Capability Index)**

Accounts for process centering:

```
Cpk = min[(USL - μ)/(3σ), (μ - LSL)/(3σ)]
```

**Interpretation:**
- Cpk < 1.00: Process not capable
- Cpk = 1.00 - 1.33: Marginally capable
- Cpk = 1.33 - 1.67: Capable
- Cpk ≥ 1.67: Excellent capability

**Pp and Ppk (Process Performance)**

Similar to Cp/Cpk but use total variation:

```
Pp = (USL - LSL) / (6s)
Ppk = min[(USL - X̄)/(3s), (X̄ - LSL)/(3s)]
```

Where s = standard deviation of all data (not subgroup)

#### 7.3.2 Six Sigma Level

```
Sigma Level = Cpk × 3

DPMO = Φ(-Sigma Level) × 1,000,000
```

Where Φ = Standard normal cumulative distribution

**Six Sigma Quality Levels:**
```
Sigma | DPMO    | Yield  | Cpk
------|---------|--------|------
2σ    | 308,537 | 69.1%  | 0.67
3σ    | 66,807  | 93.3%  | 1.00
4σ    | 6,210   | 99.4%  | 1.33
5σ    | 233     | 99.98% | 1.67
6σ    | 3.4     | 99.9997% | 2.00
```

### 7.4 Control Chart Interpretation

#### 7.4.1 Out of Control Signals

**Western Electric Rules:**

1. **One point beyond 3σ** - Special cause variation
2. **9 points in a row on same side of centerline** - Process shift
3. **6 points in a row steadily increasing/decreasing** - Trend
4. **14 points alternating up and down** - Systematic variation
5. **2 of 3 points beyond 2σ** - Process shift
6. **4 of 5 points beyond 1σ** - Process shift
7. **15 points in a row within 1σ** - Reduced variation (investigate)
8. **8 points in a row beyond 1σ** - Mixture or stratification

#### 7.4.2 Reaction Plans

When out-of-control:

1. **Immediate**: Stop production (if critical)
2. **Investigate**: Identify special cause
3. **Contain**: Quarantine suspect product
4. **Correct**: Eliminate special cause
5. **Verify**: Confirm process returns to control
6. **Document**: Record in control chart log
7. **CAPA**: Prevent recurrence if needed

### 7.5 SPC Implementation

#### 7.5.1 Selecting Characteristics

Prioritize characteristics that are:

- Critical to quality (CTQ)
- Critical to function
- Safety-related
- Difficult to meet specification
- High-cost impact
- Customer-specified

#### 7.5.2 Selecting Control Charts

```
                Is data continuous
                (measurements)?
                /            \
              YES            NO
               /              \
        Subgroup > 1?      Sample size
         /      \          constant?
       YES      NO          /      \
        /        \        YES      NO
  X̄-R or X̄-S   I-MR      /        \
                      Defective  Defects
                       or OK?    counted?
                        /  \      /    \
                       /    \    /      \
                      P    NP   C       U
```

#### 7.5.3 Establishing Control Limits

1. Collect baseline data (25-30 subgroups minimum)
2. Verify process stability (no special causes)
3. Calculate control limits
4. Validate limits with trial period
5. Implement for ongoing monitoring
6. Recalculate periodically (quarterly or after process changes)

---

## 8. Defect Detection and Classification

### 8.1 Defect Categories

#### 8.1.1 Severity Classification

**Critical Defects**
- Could cause injury or death
- Product completely unusable
- Non-compliance with safety regulations
- Examples: Missing safety guards, electrical hazard

**Major Defects**
- Significantly reduces usability
- Likely to cause product failure
- Customer would reject
- Examples: Dimensional out of spec, functional failure

**Minor Defects**
- Small deviation from specification
- Does not affect usability
- Customer might accept with concession
- Examples: Small cosmetic blemish, off-center label

**Cosmetic Defects**
- Appearance only
- No functional impact
- Within engineering tolerance
- Examples: Light surface mark, color variation

#### 8.1.2 Defect Types

**Dimensional Defects:**
- Oversize / undersize
- Out of tolerance
- Wrong geometry
- Misalignment

**Surface Defects:**
- Scratches
- Dents
- Corrosion
- Contamination
- Porosity
- Inclusions
- Cracks

**Assembly Defects:**
- Missing component
- Wrong component
- Reversed component
- Loose fastener
- Incomplete assembly

**Functional Defects:**
- Performance failure
- Leakage
- Malfunction
- Electrical failure
- Software bug

**Material Defects:**
- Wrong material
- Contamination
- Improper heat treatment
- Material property failure

**Cosmetic Defects:**
- Discoloration
- Blemish
- Finish defect
- Appearance issue

### 8.2 Visual Inspection

#### 8.2.1 Visual Inspection Standards

- **MIL-STD-2000** - Visual inspection standards
- **IPC-A-610** - Acceptability of Electronic Assemblies
- **ASME Y14.5** - Geometric Dimensioning and Tolerancing

#### 8.2.2 Inspection Conditions

- **Lighting**: 1000-1500 lux minimum
- **Viewing Distance**: 300-450mm
- **Viewing Angle**: 30-45 degrees
- **Inspector Vision**: 20/20 corrected, color vision tested
- **Break Frequency**: 15 minutes every 2 hours

#### 8.2.3 Visual Aids

- Magnifying glasses (2x-10x)
- Microscopes (20x-100x)
- Borescopes (internal inspection)
- Vision systems (automated)
- Limit samples (accept/reject standards)

### 8.3 Automated Inspection

#### 8.3.1 Machine Vision Systems

**Components:**
- Camera (2D or 3D)
- Lighting (structured, backlighting, etc.)
- Image processing software
- Decision logic
- Rejection mechanism

**Applications:**
- Presence/absence detection
- Dimensional measurement
- Surface defect detection
- OCR (label verification)
- Color matching
- Orientation verification

#### 8.3.2 AI-Powered Defect Detection

**Deep Learning Models:**
- Convolutional Neural Networks (CNN)
- YOLO (You Only Look Once)
- Faster R-CNN
- Mask R-CNN
- EfficientDet

**Training Process:**
1. Collect defect images (1000+ per class)
2. Label defects (bounding boxes, segmentation)
3. Train model with data augmentation
4. Validate on test set
5. Deploy to production
6. Continuous learning with new defects

**Performance Metrics:**
- **Accuracy**: (TP + TN) / (TP + TN + FP + FN)
- **Precision**: TP / (TP + FP)
- **Recall**: TP / (TP + FN)
- **F1 Score**: 2 × (Precision × Recall) / (Precision + Recall)

**Target Performance:**
- Precision ≥ 99% (minimize false positives)
- Recall ≥ 95% (catch most defects)
- Inference time < 100ms per image

#### 8.3.3 X-Ray Inspection

**Applications:**
- Internal defects (voids, cracks)
- Solder joint inspection (PCB)
- Foreign object detection
- Assembly verification

**Technologies:**
- 2D X-ray
- 3D X-ray (CT scanning)
- Automated X-ray Inspection (AXI)

### 8.4 Destructive Testing

When non-destructive methods insufficient:

- **Tensile Testing**: Material strength
- **Hardness Testing**: Surface/core hardness
- **Impact Testing**: Fracture resistance
- **Fatigue Testing**: Cyclic load endurance
- **Sectioning**: Internal inspection
- **Chemical Analysis**: Material composition

**Sampling Plan**: Per ISO 2859-1 or customer specification

### 8.5 Non-Destructive Testing (NDT)

- **Ultrasonic Testing (UT)**: Internal defects
- **Magnetic Particle Inspection (MPI)**: Surface/subsurface cracks
- **Dye Penetrant Inspection (DPI)**: Surface cracks
- **Eddy Current Testing (ECT)**: Conductivity changes
- **Radiographic Testing (RT)**: Internal structure
- **Acoustic Emission (AE)**: Active defect detection

**Inspector Certification**: Per ASNT SNT-TC-1A or ISO 9712

---

## 9. Quality Metrics

### 9.1 Key Performance Indicators (KPIs)

#### 9.1.1 Defect Metrics

**Defect Rate**
```
Defect Rate (%) = (Defective Units / Total Units) × 100

Target: < 1%
```

**Defects Per Million Opportunities (DPMO)**
```
DPMO = (Total Defects / Total Opportunities) × 1,000,000

Six Sigma Target: < 3.4 DPMO
```

**Parts Per Million (PPM)**
```
PPM = (Defective Parts / Total Parts) × 1,000,000

Automotive Target: < 100 PPM
```

**First Pass Yield (FPY)**
```
FPY (%) = (Units Passed First Time / Total Units) × 100

Target: ≥ 99%
```

**Final Yield**
```
Final Yield (%) = (Good Units / Total Units Started) × 100

Target: ≥ 95%
```

**Rolled Throughput Yield (RTY)**
```
RTY = Y₁ × Y₂ × Y₃ × ... × Yₙ

Where Y = Yield at each process step
```

#### 9.1.2 Process Capability Metrics

**Process Capability Index (Cpk)**
```
Cpk = min[(USL - μ)/(3σ), (μ - LSL)/(3σ)]

Target: ≥ 1.33 (≥ 1.67 for critical characteristics)
```

**Sigma Level**
```
Sigma Level = Cpk × 3

Target: ≥ 4.0 (≥ 5.0 for Six Sigma)
```

#### 9.1.3 Cost of Quality (COQ)

**Internal Failure Costs:**
- Scrap
- Rework
- Re-inspection
- Downtime
- Yield losses

**External Failure Costs:**
- Warranty claims
- Returns
- Recalls
- Liability
- Lost sales

**Appraisal Costs:**
- Inspection
- Testing
- Calibration
- Quality audits

**Prevention Costs:**
- Quality planning
- Training
- Process capability studies
- Preventive maintenance

```
Total COQ = Internal Failure + External Failure + Appraisal + Prevention

COQ (%) = (Total COQ / Revenue) × 100

Target: < 3% of revenue
```

**Optimal Distribution:**
- Prevention: 40%
- Appraisal: 30%
- Internal Failure: 20%
- External Failure: 10%

#### 9.1.4 Inspection Metrics

**Inspection Effectiveness**
```
Effectiveness (%) = (Defects Found / Total Defects) × 100

Target: ≥ 95%
```

**Inspection Efficiency**
```
Efficiency = Units Inspected / Inspector-Hour

Monitor trend for productivity
```

**Escape Rate**
```
Escape Rate (%) = (Defects Found by Customer / Total Defects) × 100

Target: < 5%
```

#### 9.1.5 Supplier Quality Metrics

**Incoming Quality (IQ) Rate**
```
IQ Rate (%) = (Lots Accepted / Lots Received) × 100

Target: ≥ 98%
```

**Supplier PPM**
```
Supplier PPM = (Defective Parts / Total Parts) × 1,000,000

Target: < 500 PPM
```

**On-Time Delivery (OTD)**
```
OTD (%) = (On-Time Deliveries / Total Deliveries) × 100

Target: ≥ 95%
```

### 9.2 Overall Equipment Effectiveness (OEE)

```
OEE = Availability × Performance × Quality

Availability = (Operating Time / Planned Production Time)
Performance = (Actual Output / Theoretical Output)
Quality = (Good Units / Total Units)

World-Class OEE: ≥ 85%
```

**Example:**
- Availability: 90% (540 min operating / 600 min planned)
- Performance: 95% (950 units / 1000 theoretical)
- Quality: 98% (931 good / 950 total)
- OEE: 0.90 × 0.95 × 0.98 = 83.8%

### 9.3 Quality Dashboards

#### 9.3.1 Real-Time Monitoring

Display metrics on visual management boards:

```
┌─────────────────────────────────────────┐
│  Quality Dashboard - Production Line A  │
├─────────────────────────────────────────┤
│  FPY:  98.5%  ✅ Target: 99%            │
│  PPM:  85     ✅ Target: < 100          │
│  Cpk:  1.45   ✅ Target: ≥ 1.33         │
│  OEE:  87.2%  ✅ Target: ≥ 85%          │
│  NCRs: 2      ⚠️  Trending up           │
└─────────────────────────────────────────┘
```

#### 9.3.2 Trend Analysis

Monitor metrics over time:

- Daily/weekly/monthly trends
- Pareto charts (top defects)
- Control charts for metrics
- Correlation analysis
- Predictive analytics

---

## 10. Calibration Management

### 10.1 Calibration Requirements

All measurement and test equipment shall be:

1. **Identified**: Unique ID number
2. **Calibrated**: At defined intervals
3. **Traceable**: To national/international standards
4. **Status Labeled**: Calibration due date visible
5. **Protected**: From damage and misuse
6. **Documented**: Calibration records maintained

### 10.2 Calibration Standards

#### 10.2.1 Traceability

```
International Standards (SI Units)
         ↓
National Standards (NIST, NPL, PTB)
         ↓
Accredited Calibration Labs (ISO/IEC 17025)
         ↓
Working Standards
         ↓
Measurement Equipment
```

#### 10.2.2 Uncertainty Requirements

**Test Uncertainty Ratio (TUR)**
```
TUR = Product Tolerance / Measurement Uncertainty

Required TUR: ≥ 4:1 (some industries require 10:1)
```

**Example:**
- Product tolerance: ±0.05 mm
- Measurement uncertainty: ±0.010 mm
- TUR: 0.05 / 0.010 = 5:1 ✅

### 10.3 Calibration Intervals

#### 10.3.1 Determining Intervals

Factors to consider:

- Manufacturer recommendations
- Usage frequency
- Environmental conditions
- Measurement criticality
- Historical drift data
- Regulatory requirements

#### 10.3.2 Typical Intervals

| Equipment Type | Interval | Notes |
|----------------|----------|-------|
| Micrometers, calipers | 6-12 months | Based on usage |
| CMM | 12 months | Temperature controlled |
| Pressure gages | 6-12 months | Based on accuracy |
| Torque wrenches | 6-12 months | High usage: 6 months |
| Temperature sensors | 12 months | Critical: 6 months |
| Hardness testers | 12 months | With test blocks |
| Balances/scales | 6-12 months | Based on accuracy |
| Oscilloscopes | 12-24 months | Low drift |
| Master gages | 12-36 months | Reference standards |

#### 10.3.3 Interval Adjustment

Shorten interval if:
- Out-of-tolerance found at calibration
- Equipment dropped or damaged
- High usage
- Critical application

Extend interval if:
- Always well within tolerance
- Low usage
- Stable environment
- Sufficient data (3+ cycles)

### 10.4 Calibration Procedures

#### 10.4.1 Calibration Process

1. **Preparation**
   - Clean equipment
   - Allow stabilization (temperature)
   - Gather standards and documentation

2. **As-Found Condition**
   - Test equipment before adjustment
   - Record as-found readings
   - Determine if in-tolerance

3. **Calibration/Adjustment**
   - Adjust to standards
   - Verify multiple points across range
   - Document adjustments made

4. **As-Left Condition**
   - Record as-left readings
   - Verify within tolerance
   - Calculate uncertainty

5. **Documentation**
   - Calibration certificate
   - Next due date
   - Affix calibration label
   - Update database

#### 10.4.2 Out-of-Tolerance Handling

If equipment found out-of-tolerance:

1. **Immediate**: Quarantine equipment
2. **Investigate**: Determine impact on measurements
3. **Evaluate**: Products measured since last calibration
4. **Disposition**: Accept, rework, or scrap affected products
5. **Root Cause**: Determine why out-of-tolerance
6. **Corrective Action**: Prevent recurrence
7. **Document**: NCR and investigation results

### 10.5 Calibration Records

Required information:

- Equipment ID and description
- Calibration date
- Due date
- Calibration lab or technician
- Standards used (ID and traceability)
- As-found and as-left data
- Uncertainty statement
- Environmental conditions
- Calibration certificate number
- Approval signature

**Retention**: Minimum 2-3 calibration cycles or per regulation

### 10.6 Measurement System Analysis (MSA)

#### 10.6.1 Gage R&R (Repeatability and Reproducibility)

**Repeatability**: Variation from same operator, multiple measurements
**Reproducibility**: Variation between different operators

**Method**: AIAG MSA Manual or ISO 22514-7

**Procedure:**
1. Select 10 parts spanning process variation
2. 3 operators measure each part 3 times
3. Randomize measurement order
4. Calculate repeatability, reproducibility, and total Gage R&R

**Gage R&R Calculation:**
```
%Gage R&R = (Gage R&R / Total Variation) × 100

Acceptance:
< 10%: Excellent measurement system
10-30%: Acceptable (marginal if critical)
> 30%: Unacceptable, improve measurement system
```

**Components:**
```
Total Variation = Equipment Variation (EV) + Appraiser Variation (AV) + Part Variation (PV)

%EV = (EV / Total Variation) × 100
%AV = (AV / Total Variation) × 100
%PV = (PV / Total Variation) × 100
```

#### 10.6.2 Bias and Linearity

**Bias**: Difference between observed average and reference value

**Linearity**: Change in bias across operating range

**Study**: Measure reference standards across range, calculate bias at each point

---

## 11. Non-Conformance Handling

### 11.1 Non-Conformance Identification

Non-conformances may be detected at:

- Receiving inspection
- In-process inspection
- Final inspection
- Customer complaint
- Internal audit
- Supplier audit
- Field failure

### 11.2 Non-Conformance Report (NCR)

#### 11.2.1 NCR Contents

Required information:

- NCR number (unique identifier)
- Date reported
- Reported by (name, department)
- Product/process identification
- Quantity affected
- Description of non-conformance
- Detection point
- Severity classification
- Photographic evidence (if applicable)
- Root cause (preliminary)
- Containment action
- Disposition
- Approvals

#### 11.2.2 NCR Workflow

```
Detection → Report → Contain → Investigate → Disposition → Verify → Close
    ↓         ↓         ↓          ↓            ↓           ↓        ↓
  Inspector  NCR    Quarantine   RCA      Decision    Verification  CAPA
                                                                    (if needed)
```

### 11.3 Containment Actions

Immediate actions to prevent non-conforming product from proceeding:

- **Quarantine**: Physically segregate affected material
- **Tag/Label**: Clearly identify as non-conforming
- **Stop Production**: If issue is ongoing
- **Notification**: Alert relevant parties (production, shipping, customer)
- **100% Inspection**: Screen remaining inventory
- **Hold Shipments**: Prevent customer impact

### 11.4 Disposition Options

#### 11.4.1 Scrap

Product cannot be corrected or used:

- Destroy or recycle material
- Document scrap quantity and value
- Update inventory
- Investigate root cause

#### 11.4.2 Rework

Product can be corrected to meet specification:

- Define rework procedure
- Verify rework effectiveness
- Re-inspect to original specification
- Document rework performed
- Monitor rework rates (excessive rework indicates process issue)

#### 11.4.3 Use-As-Is

Product does not meet specification but is usable:

- Obtain engineering approval
- Document justification
- Customer approval (if required)
- Update inspection criteria (if recurring)
- Limited to non-critical characteristics

#### 11.4.4 Repair

Product modified to acceptable (but not original specification):

- Engineering approval required
- Define repair procedure
- Verify functionality
- Document repair
- Typically for high-value items

#### 11.4.5 Return to Supplier

Product from supplier does not meet requirements:

- Document defects
- Return for credit or replacement
- Supplier corrective action required
- Update supplier rating

### 11.5 Material Review Board (MRB)

#### 11.5.1 MRB Charter

Cross-functional team to disposition non-conformances:

**Members:**
- Quality Engineer (chair)
- Manufacturing Engineer
- Design Engineer
- Production Supervisor
- Purchasing (if supplier issue)
- Customer representative (if required)

**Responsibilities:**
- Review NCRs
- Determine disposition
- Approve deviations
- Ensure customer notification (if required)
- Trend analysis

#### 11.5.2 MRB Meeting Frequency

- Weekly (or as needed based on volume)
- Emergency MRB for urgent issues
- All major/critical NCRs reviewed

### 11.6 NCR Trending and Analysis

#### 11.6.1 Metrics

- NCR count (total, by line, by product)
- NCR rate (NCRs per 1000 units)
- Cost of non-conformance
- Top defect types (Pareto)
- Repeat NCRs
- Time to close NCRs

#### 11.6.2 Pareto Analysis

Identify top contributors:

```
Defect Type      | Count | % | Cumulative %
-----------------|-------|---|-------------
Scratch          |   45  | 30% |   30%
Undersize        |   30  | 20% |   50%
Incomplete       |   25  | 17% |   67%
Wrong color      |   20  | 13% |   80%  ← 80/20 rule
Missing label    |   15  | 10% |   90%
Other            |   15  | 10% |  100%
```

Focus improvement efforts on top 20% of causes.

---

## 12. Corrective and Preventive Action (CAPA)

### 12.1 CAPA Overview

**Corrective Action**: Eliminate cause of detected non-conformance
**Preventive Action**: Eliminate cause of potential non-conformance

### 12.2 CAPA Process

```
1. Problem Identification
         ↓
2. Immediate Containment
         ↓
3. Root Cause Analysis
         ↓
4. Corrective Action Plan
         ↓
5. Implementation
         ↓
6. Verification
         ↓
7. Effectiveness Check
         ↓
8. Preventive Action
         ↓
9. Documentation & Close
```

### 12.3 Root Cause Analysis Methods

#### 12.3.1 5 Whys

Ask "why" repeatedly to drill down to root cause:

**Example:**
- **Problem**: Dimension out of specification
- **Why 1?** Tool worn
- **Why 2?** Tool change interval too long
- **Why 3?** No tool life monitoring
- **Why 4?** Preventive maintenance system incomplete
- **Why 5?** Lack of resources for PM program
- **Root Cause**: Inadequate preventive maintenance resources

#### 12.3.2 Fishbone Diagram (Ishikawa)

Categorize potential causes:

```
Man          Machine        Material       Method
  \            /               \            /
   \          /                 \          /
    \        /                   \        /
     \      /                     \      /
      \    /                       \    /
       \  /                         \  /
        \/                           \/
        ─────────────────────────────────→ Defect
        /\                           /\
       /  \                         /  \
      /    \                       /    \
     /      \                     /      \
    /        \                   /        \
   /          \                 /          \
  /            \               /            \
Measurement   Environment
```

**Categories (6M):**
- Man (People)
- Machine (Equipment)
- Material
- Method (Process)
- Measurement
- Mother Nature (Environment)

#### 12.3.3 Fault Tree Analysis (FTA)

Logical tree of events leading to failure:

```
                   Top Event (Defect)
                         |
                    OR Gate
                    /        \
            Event A           Event B
               |                  |
           AND Gate            OR Gate
           /      \            /      \
    Cause 1    Cause 2   Cause 3   Cause 4
```

#### 12.3.4 Failure Mode and Effects Analysis (FMEA)

Systematic evaluation of potential failure modes:

**Process FMEA Table:**
```
| Process Step | Failure Mode | Effect | Severity | Cause | Occurrence | Detection | RPN |
|--------------|--------------|--------|----------|-------|------------|-----------|-----|
```

**Risk Priority Number (RPN):**
```
RPN = Severity × Occurrence × Detection

Severity: 1-10 (1=minor, 10=catastrophic)
Occurrence: 1-10 (1=rare, 10=almost certain)
Detection: 1-10 (1=almost certain to detect, 10=almost impossible)

High RPN (>100): Requires action
```

#### 12.3.5 8D Problem Solving

**Eight Disciplines:**

1. **D1**: Form cross-functional team
2. **D2**: Describe the problem
3. **D3**: Implement interim containment
4. **D4**: Identify root cause
5. **D5**: Choose permanent corrective actions
6. **D6**: Implement and validate corrective actions
7. **D7**: Prevent recurrence (systemic changes)
8. **D8**: Congratulate team

### 12.4 CAPA Documentation

#### 12.4.1 CAPA Report Contents

- CAPA number
- Date initiated
- Initiator
- Problem statement
- Linked NCR (if applicable)
- Root cause analysis
  - Method used
  - Findings
  - Root cause identified
- Corrective action plan
  - Actions to be taken
  - Responsible person
  - Due date
  - Resources required
- Implementation evidence
- Verification method
- Effectiveness check
  - Follow-up period (30/60/90 days)
  - Metrics monitored
  - Effectiveness confirmed
- Preventive actions (systemic improvements)
- Lessons learned
- Closure date and signature

#### 12.4.2 CAPA Workflow

```
Initiated → Assigned → Root Cause → Action Plan → Implemented → Verified → Effective → Closed
   ↓          ↓           ↓            ↓              ↓           ↓          ↓         ↓
  QE        Owner       RCA         Approval       Execute     Test      Monitor    Document
```

**Status Tracking:**
- Open: CAPA initiated
- In Progress: Actions being implemented
- Verification: Checking if actions work
- Effectiveness: Monitoring for recurrence
- Closed: Confirmed effective, documented

### 12.5 CAPA Metrics

#### 12.5.1 CAPA Performance

- Open CAPAs (total)
- Overdue CAPAs
- Average time to close
- Repeat CAPAs (same issue)
- Effectiveness rate (% confirmed effective)

#### 12.5.2 CAPA Tracking

Dashboard view:

```
┌─────────────────────────────────────────┐
│  CAPA Metrics                           │
├─────────────────────────────────────────┤
│  Open CAPAs:       12                   │
│  Overdue:          3  ⚠️                │
│  Avg Time to Close: 45 days             │
│  Effectiveness:    95%  ✅              │
│  Repeat Rate:      5%   ✅              │
└─────────────────────────────────────────┘
```

### 12.6 Effectiveness Verification

Verify corrective action effectiveness:

1. **Immediate**: Verify action implemented correctly
2. **Short-term (30 days)**: Monitor metrics for improvement
3. **Long-term (60-90 days)**: Confirm sustained improvement
4. **Metrics**: Compare before/after data

**Example:**
- **Problem**: High defect rate (5%)
- **CAPA**: Revised inspection procedure
- **Verification**:
  - Week 1: New procedure implemented ✅
  - Week 4: Defect rate reduced to 2% ✅
  - Week 12: Defect rate sustained at 1.5% ✅
  - **Result**: CAPA effective, close

If problem recurs, re-open CAPA and investigate further.

---

## 13. Audit Management

### 13.1 Types of Audits

#### 13.1.1 Internal Audits

Self-assessment of QMS effectiveness:

- Conducted by trained internal auditors
- Cover all QMS processes annually
- Risk-based approach (more frequent for critical areas)
- Findings lead to CAPAs

#### 13.1.2 External Audits

Third-party assessment:

- Certification audits (ISO 9001, ISO 13485, etc.)
- Surveillance audits (annual)
- Recertification audits (every 3 years)
- Witness audits (customer observes)

#### 13.1.3 Supplier Audits

Evaluate supplier quality systems:

- Pre-qualification audits (new suppliers)
- Periodic audits (annual or risk-based)
- For-cause audits (quality issues)
- Process audits (specific operations)

#### 13.1.4 Customer Audits

Customer assesses your quality system:

- Pre-award audits (new business)
- Periodic audits (per agreement)
- For-cause audits (quality concerns)
- Renewal audits (recertification)

#### 13.1.5 Regulatory Audits

Government/regulatory body inspections:

- FDA inspections (medical devices, pharma)
- FAA audits (aerospace)
- EPA audits (environmental)
- OSHA inspections (safety)

### 13.2 Audit Planning

#### 13.2.1 Annual Audit Schedule

Risk-based audit frequency:

| Process | Risk | Frequency |
|---------|------|-----------|
| Design control | High | Quarterly |
| Production | High | Semi-annual |
| Purchasing | Medium | Annual |
| Calibration | Medium | Annual |
| Document control | Low | Annual |
| Training | Low | Biennial |

#### 13.2.2 Audit Preparation

1. **Scope Definition**
   - Processes to audit
   - Standards/requirements
   - Locations
   - Duration

2. **Audit Team**
   - Lead auditor
   - Auditors (competent, independent)
   - Technical experts (if needed)

3. **Audit Plan**
   - Schedule
   - Audit checklist
   - Areas to cover
   - Personnel to interview

4. **Documentation Review**
   - Previous audit reports
   - CAPAs from last audit
   - Process documentation
   - Recent changes

### 13.3 Audit Execution

#### 13.3.1 Opening Meeting

- Introductions
- Confirm scope and schedule
- Explain audit process
- Logistics (escorts, access, etc.)

#### 13.3.2 Audit Activities

- **Document Review**: Procedures, records
- **Interviews**: Personnel at all levels
- **Observations**: Process execution
- **Sampling**: Select records to review
- **Verification**: Check implementation

**Audit Techniques:**
- Ask open-ended questions
- Verify with evidence (records, observations)
- Follow process trails
- Check effectiveness, not just existence
- Sample across time periods

#### 13.3.3 Finding Classification

**Major Non-Conformance:**
- Absence of required procedure
- Total breakdown of process
- Significant risk to quality/safety
- Repeat minor finding

**Minor Non-Conformance:**
- Isolated lapse in procedure
- Documentation error
- Does not significantly impact quality

**Observation/Opportunity for Improvement:**
- Not a non-conformance
- Suggestion for improvement
- Best practice sharing

#### 13.3.4 Closing Meeting

- Present findings
- Discuss root causes
- Agree on timelines for CAPAs
- Clarify any questions
- Thank auditee for cooperation

### 13.4 Audit Reporting

#### 13.4.1 Audit Report Contents

- Audit scope and objectives
- Audit team
- Auditee
- Audit dates
- Standards/requirements audited
- Executive summary
- Detailed findings
  - Conformances
  - Non-conformances (major/minor)
  - Observations
- Overall conclusion
- Required actions and timelines
- Distribution list

#### 13.4.2 Finding Description

Clear, specific, factual:

**Good Example:**
"NC-2025-001: Procedure QP-005 requires calibration every 12 months. Micrometer #M-0123 was last calibrated on 2023-11-15 and is overdue by 13 months. Evidence: Calibration sticker on equipment."

**Poor Example:**
"Calibration not being done properly."

### 13.5 Audit Follow-Up

#### 13.5.1 CAPA Response

Auditee shall:
1. Root cause analysis (within 7 days)
2. Corrective action plan (within 14 days)
3. Implementation (per agreed timeline)
4. Evidence of completion (documentation, photos, etc.)

#### 13.5.2 Verification

Auditor verifies:
- Actions implemented as planned
- Effective in addressing root cause
- Preventive measures in place

Methods:
- Document review
- Follow-up audit
- Remote verification (photos, screen share)

#### 13.5.3 Closure

Close when:
- All actions completed
- Effectiveness verified
- Documentation approved
- Lessons learned captured

### 13.6 Auditor Qualification

#### 13.6.1 Competency Requirements

- Education (technical field)
- Quality system knowledge
- Auditing skills training
- Industry/process knowledge
- Communication skills
- Objectivity and ethics

#### 13.6.2 Training

- Auditing principles (ISO 19011)
- Specific standard (ISO 9001, ISO 13485, etc.)
- Industry requirements (IATF, AS9100, etc.)
- Interview techniques
- Root cause analysis
- Report writing

#### 13.6.3 Certification

Optional certifications:
- ASQ Certified Quality Auditor (CQA)
- IRCA Certified Auditor (ISO standards)
- Industry-specific (e.g., IATF auditor)

#### 13.6.4 Auditor Evaluation

Annual evaluation:
- Number of audits conducted
- Quality of audit reports
- Finding accuracy
- Auditee feedback
- Continuing education

---

## 14. Document Control

### 14.1 Document Management System

#### 14.1.1 Document Hierarchy

```
Level 1: Quality Manual (QMS overview)
         ↓
Level 2: Procedures (how processes work)
         ↓
Level 3: Work Instructions (detailed steps)
         ↓
Level 4: Forms and Records (evidence)
```

#### 14.1.2 Document Types

**Controlled Documents:**
- Quality manual
- Procedures
- Work instructions
- Specifications
- Drawings
- Control plans
- Inspection plans

**Records (not revised, evidence only):**
- Inspection reports
- Calibration certificates
- Training records
- Audit reports
- NCRs, CAPAs
- Meeting minutes

### 14.2 Document Control Process

#### 14.2.1 Document Creation

1. **Draft**: Author creates document
2. **Review**: Technical review for accuracy
3. **Approval**: Management approval
4. **Release**: Document control issues
5. **Distribution**: Make available to users
6. **Training**: Train personnel on new document

#### 14.2.2 Document Identification

Each controlled document shall have:

- Document number (unique)
- Title
- Revision level (A, B, C or 1.0, 2.0, etc.)
- Effective date
- Author
- Approver
- Page numbers (Page X of Y)

**Example Format:**
```
Document #: QP-001
Title: Receiving Inspection Procedure
Revision: D
Effective Date: 2026-01-15
Author: Quality Engineer
Approved: Quality Manager
Page: 5 of 12
```

#### 14.2.3 Document Revision

When to revise:
- Process changes
- Regulatory changes
- Audit findings
- CAPA actions
- Continuous improvement
- Periodic review (annual)

**Revision Process:**
1. Identify need for change
2. Draft revised document
3. Highlight changes (revision bars, summary of changes)
4. Review and approve
5. Update revision level
6. Replace obsolete version
7. Train on changes
8. Retain superseded version per retention policy

#### 14.2.4 Version Control

**Document Status:**
- Draft (red watermark)
- Under Review (yellow watermark)
- Approved (green watermark)
- Obsolete (purple watermark, "OBSOLETE")

**Electronic System:**
- Check-in/check-out
- Automatic version numbering
- Audit trail (who, when, what)
- Access controls (read/write permissions)

### 14.3 Document Distribution

#### 14.3.1 Controlled Copies

- **Master Copy**: Document control maintains
- **Controlled Copies**: Issued to users, tracked
- **Uncontrolled Copies**: For reference only (watermarked)

#### 14.3.2 Distribution Methods

- Electronic (intranet, document management system)
- Printed (stamped "CONTROLLED COPY", tracked)
- Read-only access for reference
- Training on document location and access

#### 14.3.3 Obsolete Document Control

- Remove from work areas immediately
- Stamp "OBSOLETE" and date
- Retain one copy for historical reference
- Destroy other copies (or watermark "OBSOLETE")

### 14.4 Document Review and Approval

#### 14.4.1 Review Cycle

Periodic review to ensure continued adequacy:

- Procedures: Annually
- Work instructions: Annually or as needed
- Quality manual: Every 2 years
- Forms: As needed (no expiration)

#### 14.4.2 Approval Authority

| Document Level | Approver |
|----------------|----------|
| Quality Manual | Top Management |
| Procedures | Quality Manager |
| Work Instructions | Department Manager |
| Forms | Quality Engineer |

### 14.5 Training Records

#### 14.5.1 Training Requirements

Personnel shall be trained on:

- Documents applicable to their role
- New or revised documents
- Document location and access

#### 14.5.2 Training Documentation

Training record shall include:

- Trainee name and ID
- Document number and revision
- Training date
- Trainer name
- Assessment (if applicable)
- Signature (trainee and trainer)

#### 14.5.3 Training Effectiveness

Verify training effectiveness:

- Competency assessment (test, observation)
- On-the-job monitoring
- Audit findings (improper procedure usage)
- Retraining if gaps identified

### 14.6 External Documents

#### 14.6.1 External Document Control

Control external documents relevant to QMS:

- Customer specifications
- Industry standards (ISO, ASTM, etc.)
- Regulatory requirements
- Supplier quality manuals

**Control Methods:**
- Identify and list external documents
- Maintain current versions
- Periodic review for updates (annually)
- Distribution as needed
- Mark as external origin

#### 14.6.2 External Document Sources

- Standards organizations (ISO, ASTM, SAE)
- Customer portals
- Regulatory agency websites
- Supplier communications

---

## 15. Quality Certifications

### 15.1 ISO 9001:2015 - Quality Management System

#### 15.1.1 Overview

International standard for quality management systems applicable to all organizations.

**Key Principles:**
1. Customer focus
2. Leadership
3. Engagement of people
4. Process approach
5. Improvement
6. Evidence-based decision making
7. Relationship management

#### 15.1.2 Standard Structure (High-Level Structure)

- **Clause 4**: Context of the organization
- **Clause 5**: Leadership
- **Clause 6**: Planning
- **Clause 7**: Support
- **Clause 8**: Operation
- **Clause 9**: Performance evaluation
- **Clause 10**: Improvement

#### 15.1.3 Certification Process

1. **Preparation**: Implement QMS per ISO 9001
2. **Internal Audit**: Verify compliance
3. **Management Review**: Ensure system effectiveness
4. **Application**: Select accredited certification body
5. **Stage 1 Audit**: Documentation review
6. **Stage 2 Audit**: On-site assessment
7. **Certification**: Issue certificate (3-year validity)
8. **Surveillance**: Annual audits
9. **Recertification**: Every 3 years

**Timeline**: 6-12 months from start to certification

### 15.2 ISO 13485:2016 - Medical Devices

#### 15.2.1 Overview

Quality management system specific to medical device industry, harmonized with global regulations.

**Key Differences from ISO 9001:**
- Greater emphasis on risk management (ISO 14971)
- Design control requirements
- Traceability requirements
- Sterility and packaging controls
- Regulatory compliance emphasis
- Less emphasis on customer satisfaction (more on safety)

#### 15.2.2 Key Requirements

- **Design Controls**: Design and development per 21 CFR 820.30
- **Risk Management**: Per ISO 14971
- **Traceability**: Lot/serial tracking
- **Validation**: Process validation, software validation
- **Sterility**: If applicable (ISO 11135, ISO 11137)
- **Complaints**: Medical device reporting (MDR)
- **Post-Market Surveillance**: Monitor device performance

#### 15.2.3 Regulatory Alignment

ISO 13485 aligns with:
- FDA 21 CFR Part 820 (USA)
- MDR 2017/745 (Europe)
- MDSAP (Medical Device Single Audit Program)
- CMDR (Canada)

### 15.3 IATF 16949:2016 - Automotive Quality

#### 15.3.1 Overview

Automotive quality management system standard, supplements ISO 9001 with automotive-specific requirements.

**Applicable to:** Automotive component suppliers (OEMs use different systems)

#### 15.3.2 Key Requirements Beyond ISO 9001

- **APQP**: Advanced Product Quality Planning
- **PPAP**: Production Part Approval Process
- **MSA**: Measurement Systems Analysis (Gage R&R)
- **SPC**: Statistical Process Control
- **FMEA**: Failure Mode and Effects Analysis
- **Control Plan**: Process control planning
- **Layered Process Audit**: Frequent verification
- **Problem Solving**: 8D methodology

#### 15.3.3 Customer-Specific Requirements (CSR)

Additional requirements from:
- GM (Global Manufacturing System)
- Ford (Q1)
- FCA (Chrysler)
- VW
- Others per customer

#### 15.3.4 PPAP Process

**Production Part Approval Process** - 18 elements:

1. Design records
2. Engineering change documents
3. Customer engineering approval
4. DFMEA (Design FMEA)
5. Process flow diagram
6. PFMEA (Process FMEA)
7. Control plan
8. MSA studies
9. Dimensional results
10. Material/performance test results
11. Initial process capability studies
12. Qualified laboratory documentation
13. Appearance approval report (AAR)
14. Sample products
15. Master sample
16. Checking aids
17. Customer-specific requirements
18. Part Submission Warrant (PSW)

**PPAP Levels:**
- Level 1: PSW only
- Level 2: PSW + limited documentation
- Level 3: PSW + complete documentation
- Level 4: PSW + complete documentation + customer verification
- Level 5: PSW + complete documentation + on-site assessment

### 15.4 AS9100 - Aerospace Quality

#### 15.4.1 Overview

Aerospace quality management system, supplements ISO 9001 with aerospace/defense requirements.

**Applicable to:** Aerospace component suppliers, aircraft manufacturers, MRO (maintenance, repair, overhaul)

#### 15.4.2 Key Requirements

- **Configuration Management**: Control of design changes
- **Counterfeit Parts Prevention**: Ensure authentic components
- **First Article Inspection (FAI)**: Per AS9102
- **Key Characteristics**: Identification and control
- **Special Processes**: Welding, heat treat, NDT, coating
- **FOD Prevention**: Foreign Object Damage/Debris
- **Traceability**: Complete serialization
- **Suppliers**: Approved supplier list, flow-down requirements

#### 15.4.3 AS9102 - First Article Inspection

**FAI Report Contents:**
- Part number and revision
- 100% dimensional inspection
- Material certifications
- Special process certifications (heat treat, plating, etc.)
- Functional test results
- Balloon drawing (with measured values)

### 15.5 Other Quality Standards

#### 15.5.1 GMP (Good Manufacturing Practice)

Pharmaceutical and food manufacturing:

- FDA 21 CFR Part 211 (Pharma)
- FDA 21 CFR Part 110/117 (Food)
- ICH Q7 (API manufacturing)

#### 15.5.2 HACCP (Hazard Analysis Critical Control Points)

Food safety management:

- Identify hazards (biological, chemical, physical)
- Determine critical control points (CCPs)
- Establish critical limits
- Monitor CCPs
- Corrective actions
- Verification
- Documentation

#### 15.5.3 IPC Standards (Electronics)

- **IPC-A-610**: Acceptability of Electronic Assemblies
- **IPC-J-STD-001**: Soldering requirements
- **IPC-6012**: PCB qualification
- **IPC-7711/7721**: Rework and repair

### 15.6 Certification Maintenance

#### 15.6.1 Surveillance Audits

- **Frequency**: Annual (or semi-annual for some standards)
- **Scope**: Subset of QMS processes
- **Purpose**: Verify continued compliance

#### 15.6.2 Recertification

- **Frequency**: Every 3 years
- **Scope**: Complete QMS reassessment
- **Purpose**: Renew certificate

#### 15.6.3 Maintaining Compliance

- Keep QMS updated
- Conduct internal audits
- Address non-conformances promptly
- Continuous improvement
- Management review
- Monitor regulatory changes

---

## 16. Data Models

### 16.1 Inspection Data Model

```typescript
interface Inspection {
  inspectionId: string;
  inspectionType: 'receiving' | 'first-article' | 'in-process' | 'final';
  timestamp: Date;
  productSku: string;
  batchNumber: string;
  lotNumber?: string;
  serialNumbers?: string[];
  quantity: number;
  inspector: {
    employeeId: string;
    name: string;
    certification?: string;
  };
  measurements: Measurement[];
  defects: Defect[];
  result: 'pass' | 'fail' | 'conditional';
  disposition?: 'accept' | 'rework' | 'scrap' | 'ncr';
  ncrId?: string;
  spcData?: SPCData;
  attachments?: string[];
  approvals?: Approval[];
}

interface Measurement {
  characteristicId: string;
  characteristic: string;
  nominal: number;
  measured: number;
  tolerance: number;
  unit: string;
  usl?: number; // Upper Spec Limit
  lsl?: number; // Lower Spec Limit
  status: 'pass' | 'fail';
  equipment?: string;
  method?: string;
}

interface Defect {
  defectId: string;
  type: string;
  severity: 'critical' | 'major' | 'minor' | 'cosmetic';
  location: string;
  count: number;
  description: string;
  imageUrl?: string;
}

interface SPCData {
  cp: number;
  cpk: number;
  pp?: number;
  ppk?: number;
  mean: number;
  stdDev: number;
  sigmaLevel: number;
  dpmo: number;
  outOfControl: boolean;
  controlLimits?: {
    ucl: number;
    lcl: number;
    centerline: number;
  };
}

interface Approval {
  role: string;
  approver: string;
  timestamp: Date;
  approved: boolean;
  comments?: string;
}
```

### 16.2 Non-Conformance Report (NCR) Data Model

```typescript
interface NCR {
  ncrId: string;
  ncrNumber: string; // e.g., "NCR-2025-001234"
  status: 'open' | 'investigation' | 'disposition' | 'verification' | 'closed';
  dateReported: Date;
  reportedBy: {
    employeeId: string;
    name: string;
    department: string;
  };
  productInfo: {
    sku: string;
    description: string;
    batchNumber?: string;
    lotNumber?: string;
    serialNumbers?: string[];
    quantityAffected: number;
  };
  issueDescription: string;
  detectionPoint: 'receiving' | 'in-process' | 'final' | 'customer' | 'audit';
  severity: 'critical' | 'major' | 'minor' | 'cosmetic';
  defectType: string;
  rootCause?: {
    method: '5-why' | 'fishbone' | 'fta' | 'fmea' | '8d';
    analysis: string;
    rootCause: string;
  };
  containmentAction: {
    description: string;
    implementedBy: string;
    implementedDate: Date;
    effective: boolean;
  };
  disposition: {
    decision: 'scrap' | 'rework' | 'use-as-is' | 'repair' | 'return-to-supplier';
    justification: string;
    approvedBy: string;
    approvalDate: Date;
    reworkProcedure?: string;
  };
  customerImpact: 'none' | 'potential' | 'actual';
  customerNotified: boolean;
  capaRequired: boolean;
  capaId?: string;
  attachments: string[];
  mrb?: {
    reviewDate: Date;
    members: string[];
    decision: string;
  };
  costImpact?: {
    scrapCost: number;
    reworkCost: number;
    laborCost: number;
    totalCost: number;
  };
  closedBy?: string;
  closedDate?: Date;
}
```

### 16.3 CAPA Data Model

```typescript
interface CAPA {
  capaId: string;
  capaNumber: string; // e.g., "CAPA-2025-001234"
  type: 'corrective' | 'preventive' | 'both';
  status: 'open' | 'in-progress' | 'verification' | 'effectiveness' | 'closed';
  dateInitiated: Date;
  initiator: {
    employeeId: string;
    name: string;
    department: string;
  };
  source: 'ncr' | 'audit' | 'complaint' | 'management-review' | 'trend';
  linkedRecords: {
    ncrIds?: string[];
    auditIds?: string[];
    complaintIds?: string[];
  };
  problemStatement: string;
  rootCauseAnalysis: {
    method: '5-why' | 'fishbone' | 'fta' | 'fmea' | '8d';
    findings: string[];
    rootCause: string;
    supportingData?: string[];
  };
  correctiveAction?: {
    description: string;
    assignedTo: string;
    dueDate: Date;
    status: 'planned' | 'in-progress' | 'completed';
    completionDate?: Date;
    evidence?: string[];
  };
  preventiveAction?: {
    description: string;
    assignedTo: string;
    dueDate: Date;
    status: 'planned' | 'in-progress' | 'completed';
    completionDate?: Date;
    evidence?: string[];
  };
  verification: {
    method: string;
    responsible: string;
    verificationDate?: Date;
    verified: boolean;
    comments?: string;
  };
  effectivenessCheck: {
    followUpPeriod: number; // days
    metricsMonitored: string[];
    checkDate?: Date;
    effective: boolean;
    data?: {
      metric: string;
      before: number;
      after: number;
      improvement: number;
    }[];
  };
  lessonsLearned?: string;
  closedBy?: string;
  closedDate?: Date;
}
```

### 16.4 Calibration Data Model

```typescript
interface CalibrationRecord {
  calibrationId: string;
  equipmentId: string;
  equipmentInfo: {
    description: string;
    manufacturer: string;
    model: string;
    serialNumber: string;
    location: string;
    criticality: 'critical' | 'non-critical' | 'reference-standard';
  };
  calibrationDate: Date;
  nextDueDate: Date;
  interval: number; // days
  calibrationLab: {
    name: string;
    accreditation?: string; // e.g., "ISO/IEC 17025"
    certificate?: string;
  };
  calibratedBy: string;
  standardsUsed: {
    standardId: string;
    description: string;
    traceability: string; // e.g., "NIST"
    certificateNumber: string;
    expirationDate: Date;
  }[];
  asFoundCondition: {
    inTolerance: boolean;
    readings: {
      point: string;
      expected: number;
      measured: number;
      tolerance: number;
      status: 'pass' | 'fail';
    }[];
  };
  adjustmentsMade: string;
  asLeftCondition: {
    inTolerance: boolean;
    readings: {
      point: string;
      expected: number;
      measured: number;
      tolerance: number;
      status: 'pass' | 'fail';
    }[];
  };
  uncertainty: {
    value: number;
    unit: string;
    confidenceLevel: number; // e.g., 95
  };
  environmentalConditions: {
    temperature: number;
    humidity: number;
  };
  status: 'in-calibration' | 'overdue' | 'out-of-tolerance';
  certificateNumber: string;
  approvedBy: string;
}
```

### 16.5 Audit Data Model

```typescript
interface Audit {
  auditId: string;
  auditNumber: string; // e.g., "AUD-2025-Q4-001"
  type: 'internal' | 'external' | 'supplier' | 'customer' | 'regulatory';
  standard: string; // e.g., "ISO-9001:2015"
  scope: string;
  auditDate: Date;
  auditTeam: {
    leadAuditor: string;
    auditors: string[];
    technicalExperts?: string[];
  };
  auditee: {
    organization: string;
    location: string;
    representatives: string[];
  };
  agenda: {
    time: string;
    process: string;
    clause?: string;
    auditor: string;
  }[];
  findings: Finding[];
  observations: string[];
  strengths: string[];
  overallConclusion: string;
  reportIssueDate: Date;
  followUpRequired: boolean;
  followUpDate?: Date;
  status: 'planned' | 'in-progress' | 'reported' | 'capa' | 'closed';
}

interface Finding {
  findingId: string;
  severity: 'major-nc' | 'minor-nc' | 'observation';
  clause: string;
  process: string;
  description: string;
  objectiveEvidence: string;
  requirement: string;
  capaRequired: boolean;
  capaId?: string;
  capaDueDate?: Date;
  capaStatus?: string;
  verificationDate?: Date;
  verified: boolean;
}
```

---

## 17. API Specifications

### 17.1 RESTful API Endpoints

Base URL: `https://api.wiastandards.com/v1/quality-control`

#### 17.1.1 Inspection Endpoints

```
POST   /inspections              Create inspection
GET    /inspections/{id}         Get inspection by ID
GET    /inspections              List inspections (with filters)
PUT    /inspections/{id}         Update inspection
DELETE /inspections/{id}         Delete inspection

POST   /inspections/{id}/measurements    Add measurement
POST   /inspections/{id}/defects         Add defect
POST   /inspections/{id}/approve         Approve inspection
```

#### 17.1.2 SPC Endpoints

```
POST   /spc/calculate            Calculate SPC metrics
GET    /spc/control-charts       Get control chart data
POST   /spc/control-charts       Create control chart
GET    /spc/capability           Get process capability
POST   /spc/out-of-control       Report out-of-control condition
```

#### 17.1.3 NCR Endpoints

```
POST   /ncr                      Create NCR
GET    /ncr/{id}                 Get NCR by ID
GET    /ncr                      List NCRs (with filters)
PUT    /ncr/{id}                 Update NCR
POST   /ncr/{id}/disposition     Set disposition
POST   /ncr/{id}/close           Close NCR
```

#### 17.1.4 CAPA Endpoints

```
POST   /capa                     Create CAPA
GET    /capa/{id}                Get CAPA by ID
GET    /capa                     List CAPAs (with filters)
PUT    /capa/{id}                Update CAPA
POST   /capa/{id}/verify         Verify CAPA
POST   /capa/{id}/effectiveness  Check effectiveness
POST   /capa/{id}/close          Close CAPA
```

#### 17.1.5 Calibration Endpoints

```
POST   /calibration              Create calibration record
GET    /calibration/{id}         Get calibration by ID
GET    /calibration              List calibrations
PUT    /calibration/{id}         Update calibration
GET    /calibration/due          Get upcoming calibrations
POST   /calibration/schedule     Schedule calibration
```

#### 17.1.6 Audit Endpoints

```
POST   /audits                   Create audit
GET    /audits/{id}              Get audit by ID
GET    /audits                   List audits
PUT    /audits/{id}              Update audit
POST   /audits/{id}/findings     Add finding
POST   /audits/{id}/report       Generate audit report
POST   /audits/{id}/close        Close audit
```

#### 17.1.7 Metrics Endpoints

```
GET    /metrics/dashboard        Get quality metrics dashboard
GET    /metrics/defect-rate      Get defect rate trend
GET    /metrics/fpy              Get first pass yield
GET    /metrics/cpk              Get Cpk trend
GET    /metrics/oee              Get OEE metrics
GET    /metrics/cost-of-quality  Get cost of quality
```

### 17.2 Request/Response Examples

#### 17.2.1 Create Inspection

**Request:**
```http
POST /v1/quality-control/inspections
Content-Type: application/json
Authorization: Bearer {token}

{
  "inspectionType": "first-article",
  "productSku": "WIDGET-A100",
  "batchNumber": "BATCH-20251227-01",
  "quantity": 1,
  "inspector": {
    "employeeId": "EMP-5678",
    "name": "Jane Smith"
  },
  "measurements": [
    {
      "characteristic": "diameter",
      "nominal": 50.0,
      "measured": 50.02,
      "tolerance": 0.05,
      "unit": "mm",
      "usl": 50.05,
      "lsl": 49.95
    }
  ]
}
```

**Response:**
```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "inspectionId": "INS-2025-001234",
  "result": "pass",
  "timestamp": "2025-12-27T10:30:00Z",
  "spcData": {
    "cpk": 1.67,
    "cp": 1.85,
    "mean": 50.01,
    "stdDev": 0.009,
    "sigmaLevel": 5.01,
    "dpmo": 0.57
  },
  "status": "approved"
}
```

#### 17.2.2 Calculate SPC

**Request:**
```http
POST /v1/quality-control/spc/calculate
Content-Type: application/json

{
  "characteristic": "diameter",
  "measurements": [50.02, 49.98, 50.01, 50.03, 49.99, 50.00, 50.01],
  "specification": {
    "nominal": 50.0,
    "usl": 50.05,
    "lsl": 49.95
  }
}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "cp": 1.85,
  "cpk": 1.67,
  "mean": 50.006,
  "stdDev": 0.0090,
  "sigmaLevel": 5.01,
  "dpmo": 0.57,
  "outOfControl": false,
  "controlLimits": {
    "ucl": 50.033,
    "lcl": 49.967,
    "centerline": 50.0
  }
}
```

#### 17.2.3 Create NCR

**Request:**
```http
POST /v1/quality-control/ncr
Content-Type: application/json

{
  "productSku": "WIDGET-A100",
  "batchNumber": "BATCH-20251227-01",
  "quantityAffected": 50,
  "issueDescription": "Diameter out of tolerance",
  "severity": "major",
  "detectionPoint": "final",
  "reportedBy": {
    "employeeId": "EMP-5678",
    "name": "Jane Smith"
  },
  "containmentAction": {
    "description": "Quarantine all units from shift",
    "implementedBy": "EMP-5678"
  }
}
```

**Response:**
```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "ncrId": "NCR-2025-001234",
  "status": "open",
  "dateReported": "2025-12-27T10:30:00Z",
  "capaRequired": true,
  "estimatedCost": 2500.00
}
```

### 17.3 Authentication

Use Bearer token authentication:

```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 17.4 Error Handling

Standard HTTP status codes:

```
200 OK                  Successful request
201 Created             Resource created
400 Bad Request         Invalid input
401 Unauthorized        Authentication required
403 Forbidden           Insufficient permissions
404 Not Found           Resource not found
422 Unprocessable       Validation error
500 Internal Error      Server error
```

Error response format:

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid measurement data",
    "details": [
      {
        "field": "measurements[0].measured",
        "issue": "Value must be a number"
      }
    ]
  }
}
```

---

## 18. Security and Privacy

### 18.1 Data Security

#### 18.1.1 Access Control

- Role-based access control (RBAC)
- Principle of least privilege
- Multi-factor authentication (MFA) for critical functions
- Regular access reviews

**User Roles:**
- **Inspector**: Create/view inspections
- **Quality Engineer**: NCR, CAPA, analysis
- **Quality Manager**: Approvals, audits
- **Administrator**: System configuration

#### 18.1.2 Data Encryption

- **In Transit**: TLS 1.3
- **At Rest**: AES-256 encryption
- **Database**: Encrypted backups
- **Credentials**: Hashed with bcrypt/Argon2

#### 18.1.3 Audit Trail

Log all critical actions:

- User ID and timestamp
- Action performed (create, update, delete, approve)
- Before/after values (for updates)
- IP address
- Retention: 7 years minimum

### 18.2 Data Privacy

#### 18.2.1 Personal Data

Limit collection of personal data:

- Employee ID (pseudonymized if possible)
- Name (for accountability)
- No sensitive personal data unless required

#### 18.2.2 GDPR Compliance

If applicable:

- Data minimization
- Right to access
- Right to erasure (where appropriate)
- Data portability
- Consent management

#### 18.2.3 Data Retention

Per industry requirements:

- **General**: 7 years
- **Medical Devices**: Lifetime of device + 7 years
- **Automotive**: 15 years (IATF)
- **Aerospace**: Lifetime of aircraft
- **Regulatory**: Per specific requirement

### 18.3 Integrity

#### 18.3.1 Data Integrity (ALCOA+)

- **Attributable**: Who performed action
- **Legible**: Clear and readable
- **Contemporaneous**: Recorded at time of activity
- **Original**: First recording or true copy
- **Accurate**: No errors, correct
- **Complete**: All data present
- **Consistent**: Sequential, chronological
- **Enduring**: Durable, cannot be deleted
- **Available**: Accessible when needed

#### 18.3.2 Change Control

- No deletion of approved records (archive only)
- Version control for documents
- Approval workflow for changes
- Audit trail for all modifications

---

## 19. Implementation Guidelines

### 19.1 Implementation Roadmap

#### 19.1.1 Phase 1: Foundation (Months 1-3)

- Define quality policy and objectives
- Document core processes
- Train quality team
- Implement basic inspections
- Start NCR process

#### 19.1.2 Phase 2: Expansion (Months 4-6)

- Implement SPC for critical processes
- Establish calibration program
- Deploy CAPA system
- Conduct first internal audit
- Begin metrics tracking

#### 19.1.3 Phase 3: Optimization (Months 7-12)

- AI-powered defect detection
- Advanced analytics
- Supplier quality integration
- Continuous improvement projects
- Certification preparation (if applicable)

### 19.2 Best Practices

#### 19.2.1 Organizational

- Top management commitment
- Cross-functional collaboration
- Empower quality team
- Recognize quality achievements
- Continuous training

#### 19.2.2 Technical

- Standardize procedures
- Automate where possible
- Use statistical methods
- Leverage technology (AI, IoT)
- Benchmark against industry

#### 19.2.3 Cultural

- Quality is everyone's responsibility
- Prevention over detection
- Data-driven decisions
- Encourage reporting (no blame culture)
- Celebrate improvements

### 19.3 Common Pitfalls

- **Documentation Burden**: Keep it simple and practical
- **Lack of Training**: Invest in people
- **Ignoring Data**: Use metrics to drive improvement
- **Reactive Mode**: Focus on prevention
- **Silos**: Break down departmental barriers

---

## 20. Compliance and Validation

### 20.1 Regulatory Compliance

Ensure compliance with applicable regulations:

- FDA 21 CFR Part 820 (Medical Devices)
- FDA 21 CFR Part 211 (Pharmaceuticals)
- FAA regulations (Aerospace)
- Automotive customer requirements
- Environmental regulations
- Export control regulations

### 20.2 Validation

#### 20.2.1 Process Validation

For critical processes:

- **IQ (Installation Qualification)**: Equipment installed correctly
- **OQ (Operational Qualification)**: Equipment operates per specification
- **PQ (Performance Qualification)**: Process produces acceptable product

**Validation Protocol:**
1. Define process parameters
2. Identify critical parameters
3. Establish acceptance criteria
4. Execute validation runs
5. Analyze data
6. Document results
7. Approve and release

#### 20.2.2 Software Validation

For quality systems software:

- Requirements specification
- Design specification
- Code review
- Unit testing
- Integration testing
- User acceptance testing (UAT)
- Validation summary report

### 20.3 Continuous Compliance

- Monitor regulatory changes
- Update procedures as needed
- Train personnel on changes
- Conduct gap analyses
- Maintain certifications

---

## Appendix A: Glossary

See Section 4 (Terms and Definitions)

## Appendix B: Forms and Templates

Available in implementation package:

- Inspection checklists
- NCR form
- CAPA form
- Control plan template
- Audit checklist
- Calibration record

## Appendix C: Statistical Tables

- Control chart constants (A2, D3, D4, etc.)
- Sample size codes (ISO 2859-1)
- Sigma to DPMO conversion table

## Appendix D: References

- ISO 9001:2015
- ISO 13485:2016
- IATF 16949:2016
- AS9100 Rev D
- AIAG manuals (SPC, MSA, FMEA, APQP, PPAP)

---

**Document History**

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-27 | WIA Industry Research Group | Initial release |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
