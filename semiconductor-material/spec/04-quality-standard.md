# WIA-SEMI-018 Quality Management Standard v1.0

## Semiconductor Material Quality Assurance Requirements

**Document Number**: WIA-SEMI-018-SPEC-004
**Version**: 1.0
**Effective Date**: January 1, 2025

---

## 1. Scope

This specification defines quality management system (QMS) requirements for semiconductor material suppliers and fab quality control programs, ensuring consistent material quality and traceability throughout the supply chain.

---

## 2. Quality Management System Requirements

### 2.1 Supplier QMS Certification
**Mandatory**: ISO 9001:2015 or equivalent
**Recommended**: IATF 16949 (for automotive semiconductor applications)

**Audit Requirements**:
- Initial certification audit
- Surveillance audits: Annual
- Re-certification: Every 3 years

### 2.2 Process Control
**Statistical Process Control (SPC)**:
- Control charts for critical parameters
- Cpk ≥ 1.33 for all critical-to-quality (CTQ) characteristics
- Cpk ≥ 1.67 for safety-critical parameters

**Deviation Management**:
- All out-of-specification events documented
- Root cause analysis: 5-Why or Fishbone method
- Corrective action: Implemented within 30 days
- Preventive action: Continuous improvement program

---

## 3. Material Characterization Requirements

### 3.1 Silicon Wafers

**100% Inspection**:
- Dimensional verification (diameter, thickness)
- Particle scanning (laser scattering)
- Visual inspection (automated optical)

**Sampling Inspection** (per lot):
- Surface roughness (AFM): 5 wafers
- Purity analysis (SIMS): 1 wafer
- Electrical properties: 25 wafers
- Flatness (TTV, bow, warp): 25 wafers

**Test Equipment Calibration**:
- Frequency: Per manufacturer recommendation (typically annual)
- Standards: NIST-traceable or equivalent
- Documentation: Calibration certificates maintained

### 3.2 Photoresists

**Per-Batch Testing**:
- Viscosity: ±3% of specification
- Particle count: <10 particles/mL (>0.2 µm)
- Density/Refractive index: As specified
- pH (if applicable): Within range

**Functional Testing** (per batch):
- Sensitivity verification: Dose-to-size on monitor wafer
- Contrast curve generation
- Resolution capability (periodic, monthly)

**Stability Testing**:
- Accelerated aging: 40°C for 2 months = 6 months at 4°C
- Real-time aging: Shelf life verification

### 3.3 Specialty Gases

**Per-Batch Analysis**:
- Gas Chromatography-Mass Spectrometry (GC-MS) or equivalent
- Moisture analysis: Tunable diode laser or similar
- Impurity breakdown: All elements >0.01 ppm reported

**Certificate of Analysis (CoA)**:
- Batch number and fill date
- Purity (%) and impurity levels (ppm)
- Test methods and date of analysis
- Expiration date (if applicable)
- Approved by: QA manager signature

---

## 4. Incoming Quality Control (IQC)

### 4.1 Receipt Inspection
**100% Items**:
- Physical condition (damage, leaks, contamination)
- Label verification (lot number, quantity, specifications)
- Documentation (CoA, packing list, delivery note)

**Sampling Inspection**:
- Material verification testing (5-10% of lots)
- Comparison to CoA data
- Functional testing on monitor/test wafers

### 4.2 Acceptance Sampling Plan
**AQL (Acceptable Quality Limit)**: 0.15% (MIL-STD-105E, Level II)

**Example** (Silicon wafers, lot size 500):
- Sample size: 80 wafers
- Accept: 0-1 defects
- Reject: 2+ defects

**Quarantine**:
- All materials held in quarantine until released
- MES system control: Cannot use until IQC complete
- Typical release time: 24-48 hours

### 4.3 Non-Conformance Reporting
**NCR (Non-Conformance Report)**:
- Issued for any material failing acceptance criteria
- Contains: Description, quantity, lot numbers, failure mode
- Disposition: Return, use-as-is (with waiver), or scrap
- Supplier notification: Within 24 hours

**Supplier Corrective Action Request (SCAR)**:
- Required for all NCRs
- Supplier response: 8D report within 2 weeks
- Verification: Close-out only after effectiveness confirmed

---

## 5. Traceability Requirements

### 5.1 Material Identification
**Unique Identifiers**:
- Supplier lot number (batch code)
- Fab receiving ID number
- Sublot IDs (if split)
- Wafer serial numbers (laser-marked on 300mm wafers)

**Barcode Standards**: Code 128 or QR code
**Data Capture**: Automated scanners at all points of use

### 5.2 Genealogy Database
**Minimum Data Retention**: 10 years
**Required Data**:
- Material type and specification
- Supplier name and certification
- Lot numbers and quantities
- Receipt date and IQC results
- Usage history (tools, processes, wafer lots)
- Disposition (accepted, rejected, consumed)

**Genealogy Reconstruction**:
- Forward tracing: Material → Devices
- Backward tracing: Device → All materials used
- Response time: <4 hours for critical quality events

### 5.3 Audit Trail
**Electronic Records**:
- Tamper-proof (audit trail for all changes)
- 21 CFR Part 11 compliant (for medical/automotive)
- Backup and disaster recovery: Daily backups, off-site storage

---

## 6. Supplier Performance Management

### 6.1 Supplier Scorecard
**Monthly Metrics** (weighted):

**Quality (40%)**:
- Defect rate (ppm): Target <50 ppm
- Lot reject rate: Target <0.5%
- CoA accuracy: Target 100%

**Delivery (30%)**:
- On-time delivery: Target >98%
- Order fill rate: Target >99%
- Lead time adherence: ±1 day

**Service (20%)**:
- Technical responsiveness: Target <24 hours
- Documentation completeness: Target 100%
- Proactive communication: Subjective rating 1-5

**Cost (10%)**:
- Price competitiveness: vs. benchmark

**Scoring**:
- 90-100 points: Preferred supplier (eligible for volume increases)
- 75-89 points: Qualified supplier (maintain current volume)
- 60-74 points: Conditional (improvement plan required within 90 days)
- <60 points: Disqualify (find alternative supplier)

### 6.2 Quarterly Business Reviews (QBR)
**Attendees**: Supplier QA/Engineering + Customer Procurement/Quality

**Agenda**:
- Scorecard review and trend analysis
- Quality issues and resolution status
- Continuous improvement projects
- Capacity and roadmap alignment
- Commercial terms and pricing

**Documentation**: Meeting minutes, action items with owners and due dates

### 6.3 Audits
**Types**:
- **Process audit**: Verify QMS compliance, annual
- **Product audit**: Random sampling verification, quarterly
- **System audit**: Full QMS review, every 2-3 years
- **For-cause audit**: Triggered by quality issues

**Audit Reports**:
- Findings: Major (non-compliance), Minor (deviation), Observation
- Corrective action plan: Required for all findings
- Follow-up: Verification audit within 6 months

---

## 7. Continuous Improvement

### 7.1 Plan-Do-Check-Act (PDCA) Cycle
**Annual Improvement Goals**:
- Defect reduction: 10-20% year-over-year
- Cpk improvement: Trend toward Cpk >1.67
- Cost reduction: 2-5% annually (total cost of ownership)

**Kaizen Events**:
- Frequency: Quarterly or as needed
- Focus: Waste reduction, process optimization
- Team: Cross-functional (fab, supplier, equipment vendor)

### 7.2 Failure Analysis and Root Cause Investigation
**Trigger Events**:
- Customer returns
- Yield excursions (>3σ from baseline)
- Defect density spikes
- Reliability failures

**Methodology**:
- 8D problem solving
- Failure analysis flow (Section 6.1 in main spec)
- Containment → Root cause → Corrective action → Verification

**Closure Criteria**:
- Root cause confirmed with data
- Corrective action implemented
- Effectiveness verified (3 months of stable production)

---

## 8. Training and Competency

### 8.1 Personnel Qualification
**Roles Requiring Certification**:
- Quality inspectors
- Material handlers
- Test equipment operators
- Failure analysis engineers

**Training Requirements**:
- Initial training: Role-specific curriculum (8-40 hours)
- Hands-on practice: Supervised until competent
- Written test: 80% pass mark
- Practical demonstration: Evaluated by supervisor

**Re-certification**:
- Annual refresher training (4-8 hours)
- Competency re-assessment every 2 years

### 8.2 Training Records
**Documentation**:
- Employee name and ID
- Training course title and date
- Instructor name
- Test scores
- Certification expiration date

**Retention**: Duration of employment + 3 years

---

## 9. Document and Record Control

### 9.1 Document Management
**Controlled Documents**:
- Specifications (this document and related)
- Standard Operating Procedures (SOPs)
- Work instructions
- Test methods

**Control Requirements**:
- Version control: Major.Minor format (e.g., 1.0, 1.1, 2.0)
- Approval: QA manager or designee
- Distribution: Electronic system (latest version always available)
- Obsolete versions: Clearly marked and removed from use

### 9.2 Record Retention
**Minimum Retention Periods**:
- CoAs and test data: 10 years
- IQC records: 10 years
- Calibration certificates: Life of equipment + 5 years
- Non-conformance reports: 10 years
- Supplier audits: 10 years
- Training records: Employee tenure + 3 years

**Storage**: Electronic preferred, with backup and disaster recovery

---

## 10. Risk Management

### 10.1 Supply Chain Risk Assessment
**Frequency**: Annual review, update as needed

**Risk Categories**:
- Supplier financial stability
- Single-source dependencies
- Geopolitical factors (export controls, tariffs, conflicts)
- Natural disaster exposure
- Technology obsolescence

**Mitigation Strategies**:
- Dual sourcing (70/30 split for critical materials)
- Strategic inventory (30-90 days for critical items)
- Long-term supply agreements (LTSAs)
- Alternate material qualification (backup options ready)

### 10.2 Material Quality Risk
**FMEA (Failure Mode and Effects Analysis)**:
- Identify potential failure modes
- Assess severity, occurrence, detection (SOD)
- Calculate Risk Priority Number (RPN = S × O × D)
- Prioritize mitigation: RPN >100 requires action plan

**Example**:
- Failure mode: High wafer defect density
- Severity: 8 (yield impact)
- Occurrence: 3 (rare with current supplier)
- Detection: 7 (detected at IQC, but some escape)
- RPN: 8 × 3 × 7 = 168 → High priority for mitigation

**Mitigation**: Upgrade IQC defect scanner, increase sampling, audit supplier process

---

## 11. Measurement System Analysis (MSA)

### 11.1 Gage R&R (Repeatability & Reproducibility)
**Frequency**: Annual for critical measurements

**Acceptance Criteria**:
- %GRR <10%: Excellent
- %GRR 10-30%: Acceptable
- %GRR >30%: Unacceptable (improve measurement system)

**Example**: Wafer thickness measurement
- 10 wafers, 3 operators, 3 repeat measurements each
- Calculate variance: Repeatability (equipment) + Reproducibility (operator) vs. Total
- If %GRR = 15%: Acceptable, but monitor and seek improvement

### 11.2 Correlation Studies
**Between-Tool Correlation**:
- Compare results from different test equipment
- Acceptance: ±5% agreement
- Frequency: Quarterly

**Supplier-Customer Correlation**:
- Compare supplier CoA data to fab IQC results
- Acceptance: ±10% agreement
- Frequency: Annual, or when discrepancies arise

---

## 12. Key Performance Indicators (KPIs)

### 12.1 Material Quality KPIs
- **Defect density** (wafers): <0.08 defects/cm² (monthly average)
- **Lot reject rate**: <0.5%
- **Material-related yield loss**: <0.2%
- **Supplier ppm defect rate**: <50 ppm

### 12.2 Process KPIs
- **IQC cycle time**: <24 hours (average)
- **Cpk for critical parameters**: >1.33
- **SPC in-control percentage**: >95%

### 12.3 Supply Chain KPIs
- **On-time delivery**: >98%
- **Inventory days of supply**: 7-14 days (optimal range)
- **Inventory turnover**: >26 per year
- **Stockout incidents**: 0 per year

**Reporting**: Monthly dashboards, quarterly reviews with management

---

## 13. Compliance and Regulatory

### 13.1 Industry Standards
**Compliance Required**:
- SEMI Standards (M1, C1, S2, etc.)
- ASTM Standards (F84, F1530, etc.)
- ISO Standards (9001, 14001, 45001)

### 13.2 Regulatory Compliance
**As Applicable**:
- REACH (EU chemicals regulation)
- RoHS (restriction of hazardous substances)
- TSCA (US Toxic Substances Control Act)
- Conflict minerals (Dodd-Frank Act)
- Export controls (EAR, ITAR where applicable)

### 13.3 Customer-Specific Requirements
**Automotive**: IATF 16949, AEC-Q100 qualified materials
**Medical**: ISO 13485, 21 CFR Part 11 electronic records
**Aerospace**: AS9100, DFARS compliance

---

## 14. Revision History

| Version | Date | Changes | Approved By |
|---------|------|---------|-------------|
| 1.0 | 2025-01-01 | Initial release | WIA Standards Committee |

---

## 15. Appendices

### Appendix A: Glossary
- **AQL**: Acceptable Quality Limit
- **CoA**: Certificate of Analysis
- **Cpk**: Process Capability Index (centered)
- **IQC**: Incoming Quality Control
- **NCR**: Non-Conformance Report
- **SCAR**: Supplier Corrective Action Request
- **SPC**: Statistical Process Control

### Appendix B: Forms and Templates
- Supplier Scorecard Template
- Non-Conformance Report Form
- 8D Report Template
- Audit Checklist
- Training Record Form

### Appendix C: Contact Information
WIA Semiconductor Standards Division
- Email: semiconductor@wia-standards.org
- Web: https://wia-standards.org/semi-018
- Technical Support: support@wia-standards.org
- Certification: certification@wia-standards.org

---

© 2025 SmileStory Inc. / World Certification Industry Association (WIA)
弘益人間 (홍익인간) · Benefit All Humanity

Document Control:
- Filename: WIA-SEMI-018-SPEC-004-Quality-v1.0.md
- SHA-256: [To be calculated upon final release]
