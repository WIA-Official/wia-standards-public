# Chapter 9: Implementation Roadmap

## Putting WIA-SEMI-018 into Practice

---

## 🎯 **Learning Objectives**

By the end of this chapter, you will understand:

- Step-by-step implementation plan for WIA-SEMI-018 adoption
- Material qualification and validation timelines
- Cost-benefit analysis and ROI calculations
- Change management and training requirements
- Continuous improvement frameworks
- Success metrics and KPIs for material programs

---

## 1. Implementation Strategy Overview

### **1.1 Phased Approach**

WIA-SEMI-018 implementation follows a four-phase roadmap aligned with typical semiconductor development cycles:

**Phase 1: Assessment & Planning (3-6 months)**
- Current state gap analysis
- Supplier evaluation and selection
- Budget and resource allocation
- Project team formation
- Detailed implementation plan

**Phase 2: Qualification & Validation (6-12 months)**
- Material characterization and testing
- Process integration trials
- Yield and reliability evaluation
- Quality system alignment
- Documentation and training

**Phase 3: Production Ramp (3-6 months)**
- Limited production introduction (10-30% volume)
- Statistical process control setup
- Supply chain activation
- Continuous monitoring and optimization
- Issue resolution and fine-tuning

**Phase 4: Full Deployment (Ongoing)**
- 100% volume production
- Continuous improvement programs
- Technology refresh cycles
- Supplier performance management
- Standards evolution tracking

**Total timeline**: 12-24 months from initiation to full production

---

## 2. Phase 1: Assessment & Planning

### **2.1 Current State Gap Analysis**

**Assessment framework**:

**Material Quality**:
- Current wafer specifications vs. WIA-SEMI-018 targets
  - Defect density: ___ defects/cm² (target: <0.1)
  - Silicon purity: ___ nines (target: 11-9s minimum)
  - Dimensional specs: TTV, bow, warp compliance
- Photoresist performance
  - Resolution capability: ___ nm (target: node requirement)
  - LER: ___ nm 3σ (target: <3.0 nm for ArF, <1.5 nm for EUV)
  - Defectivity: ___ defects/cm² (target: <0.01)
- Specialty gas purity
  - Current grade: ___ N (target: 6N minimum for critical gases)
  - Impurity levels vs. specification

**Quality Systems**:
- Supplier qualification process maturity (1-5 scale)
- IQC capabilities (equipment, staffing, procedures)
- Traceability system completeness
- SPC implementation extent
- Failure analysis resources

**Supply Chain**:
- Number of qualified suppliers per material
- Geographic concentration risk
- Inventory levels and turnover rates
- Lead times and reliability metrics
- Total cost of ownership visibility

**Compliance**:
- Environmental permits and reporting
- Safety systems (gas detection, scrubbers, PPE)
- Regulatory compliance (REACH, TSCA, RoHS)
- Training and certification programs

**Gap analysis template**:

| Parameter | Current State | WIA-SEMI-018 Target | Gap | Priority | Action Required |
|-----------|---------------|---------------------|-----|----------|-----------------|
| Wafer defect density | 0.15/cm² | <0.1/cm² | -33% | High | Qualify new supplier or upgrade incoming QC |
| Si purity | 10-9s | 11-9s | 1 nine | High | Require CoA verification, audit supplier process |
| Photoresist LER | 3.5 nm | <3.0 nm | 14% | Medium | Evaluate next-gen resist formulations |
| Gas purity (PH₃) | 5N | 6N | 1 nine | Medium | Switch to higher-purity grade, validate process |
| IQC staffing | 2 FTE | 4 FTE | 2 FTE | Medium | Hire and train additional inspectors |
| Traceability system | Manual logs | Automated MES | Major | High | Implement barcode scanning, database integration |

### **2.2 Budget and Resource Planning**

**Typical implementation costs** (50K wspm fab):

**Capital expenditures**:
- Advanced characterization tools (AFM, SIMS access): $200K-500K
- Automated inspection systems (particle scanners): $500K-2M
- IT infrastructure (traceability database, SPC software): $100K-300K
- Gas detection and safety systems upgrades: $200K-500K
- **Subtotal**: $1M-3M

**Operating expenses (annual)**:
- Material cost increase (higher-grade materials): $2M-10M (varies significantly)
- Additional IQC testing: $500K-1M
- Supplier qualification programs: $200K-500K
- Training and documentation: $100K-200K
- Consulting and project management: $200K-500K
- **Subtotal**: $3M-12M (first year, ongoing costs ~50% lower)

**Personnel requirements**:
- Project manager: 1 FTE (full-time for 12 months)
- Material engineers: 2-4 FTE (qualification and optimization)
- Quality engineers: 2-3 FTE (IQC setup, SPC implementation)
- Supply chain specialists: 1-2 FTE (supplier management)
- Training coordinator: 0.5 FTE

**Total investment**: $4M-15M over 12-24 months (varies by fab size and current maturity)

**ROI drivers**:
- Yield improvement: 1-5% absolute gain (value: $10M-100M annually for large fab)
- Defect reduction: Lower scrap and rework costs ($2M-10M)
- Reliability improvement: Reduced field failures (10-100× fab costs avoided)
- Supply chain optimization: 5-15% material cost reduction through TCO focus
- **Expected ROI**: 2-10× over 3-5 years

---

## 3. Phase 2: Qualification & Validation

### **3.1 Material Qualification Workflow**

**Silicon Wafer Qualification** (3-6 months):

**Week 1-4: Supplier Selection & Sampling**
- Issue RFQ (request for quotation) to 3-5 potential suppliers
- Request sample lots (minimum 3 lots, 25 wafers each)
- Review CoAs (certificates of analysis) and manufacturing capabilities

**Week 5-12: Comprehensive Characterization**
- **Dimensional verification** (100% of samples):
  - Diameter, thickness, TTV, bow, warp
  - Pass/fail: Compare to SEMI M1-0320 spec and internal requirements
- **Defect inspection** (100%):
  - Automated particle scanner (>0.09 µm)
  - Target: <0.08 defects/cm² (20% margin below spec)
- **Surface quality** (sampling):
  - AFM surface roughness on 5 wafers: Ra <0.2 nm
  - Optical inspection for pits, scratches
- **Purity verification** (1 wafer per lot):
  - SIMS analysis: Confirm 11-9s purity (all elements <spec)
  - Focus on critical impurities: B, P, Fe, Cu, Ni
- **Electrical properties** (25 wafers per lot):
  - Four-point probe: Resistivity uniformity (±5% across wafer)
  - Minority carrier lifetime (if applicable)

**Week 13-20: Process Integration**
- Introduce sample wafers into production line
- Monitor 5-10 key process steps:
  - Oxidation (oxide thickness, defectivity)
  - Photolithography (CD uniformity, resist adhesion)
  - Etch (uniformity, selectivity)
  - CMP (removal rate, defects)
  - Metrology (no interference with measurements)
- **Acceptance criteria**: Performance within ±3% of baseline (existing supplier)

**Week 21-24: Reliability Testing**
- Build test structures (capacitors, transistors, interconnects)
- Electrical stress testing:
  - Time-dependent dielectric breakdown (TDDB)
  - Hot carrier injection (HCI)
  - Negative bias temperature instability (NBTI)
- **Acceptance**: Meet reliability targets (10-year lifetime at use conditions)

**Week 25-26: Final Review & Approval**
- Compile all data into qualification report
- Risk assessment: Identify any open issues or concerns
- Management review and approval
- Issue formal supplier approval letter
- Negotiate long-term supply agreement

**Photoresist Qualification** (4-6 months):

**Similar workflow**, with emphasis on:
- Sensitivity (dose-to-size): ±5% of target
- Contrast curve analysis: Contrast >5.0 (ArF), >6.0 (KrF)
- Resolution capability: Meet or exceed node requirement
- LER measurement: SEM or AFM on 50-100 lines
- Defect density: Automated inspection after develop, <0.01 defects/cm²
- Shelf life verification: Accelerated aging (40°C for 2 months = 6 months at 4°C)
- Outgassing compatibility: For EUV, measure outgassing rate (<10⁻¹⁰ Torr·L/s)

**Specialty Gas Qualification** (2-4 months):

**Streamlined process** (if supplier has strong track record):
- CoA review and verification (purity, impurity breakdown)
- Analytical spot-check: GC-MS or RGA on 1-2 cylinders
- Process compatibility: Introduce into 1-2 tools, monitor results (deposition rate, film quality, defects)
- Safety system verification: Ensure gas detection and scrubber compatible

### **3.2 Validation Metrics**

**Qualification success criteria**:

**Technical performance**:
- All material specifications met (100% compliance)
- Process integration successful (within ±3% of baseline)
- Reliability targets achieved (pass all stress tests)

**Quality consistency**:
- Lot-to-lot variation <30% of baseline supplier
- Defect density stable across multiple lots (Cpk >1.33)

**Supply chain**:
- On-time delivery >95% during qualification
- Documentation complete and accurate (100% of shipments)
- Responsive technical support (<24 hour response)

**Cost**:
- Total cost of ownership competitive (within 10% of target)

**Risk assessment**:
- No showstopper issues identified
- Mitigation plans in place for all medium/high risks

**Decision criteria**:
- **Approve**: All criteria met, proceed to production ramp
- **Conditional approve**: Minor issues, approve with documented risk mitigation plan
- **Reject**: Major technical or quality issues, cannot use for production

---

## 4. Phase 3: Production Ramp

### **4.1 Limited Introduction Strategy**

**10-30% volume introduction** (months 1-3):

**Allocation plan**:
- Month 1: 10% of wafers use new material
- Month 2: 20% (if Month 1 successful)
- Month 3: 30% (if Month 2 successful)
- Decision point: Full deployment or troubleshooting

**Monitoring plan**:
- **Daily**: Defect density, critical dimension (CD) uniformity
- **Weekly**: Yield by lot, process tool matching
- **Monthly**: Reliability stress testing, supplier performance scorecard

**Control strategy**:
- Run new material lots alongside baseline (matched pairs)
- Compare yield, electrical parameters, defectivity
- Statistical analysis: t-test or ANOVA (p<0.05 for significance)

**Success criteria for volume increase**:
- Yield: Within 1% of baseline (not statistically different)
- Defectivity: <120% of baseline average
- Customer returns: No increase attributed to material change

**Example ramp**:

| Month | Volume (%) | Lots Run | Yield (Avg) | Defects/cm² | Decision |
|-------|-----------|----------|-------------|-------------|----------|
| 1 | 10 | 50 | 87.2% (baseline: 87.5%) | 0.08 (baseline: 0.07) | Proceed to 20% |
| 2 | 20 | 100 | 87.8% | 0.07 | Proceed to 30% |
| 3 | 30 | 150 | 88.1% | 0.06 | Approve full deployment |

### **4.2 SPC Implementation**

**Control chart setup**:

**Identify critical parameters**:
- Wafer defect density
- Photoresist CD uniformity (±3σ)
- Gas flow rate / purity (for process gases)
- IQC test results (SIMS purity, particle count, etc.)

**Establish control limits**:
- Collect baseline data (minimum 25 data points)
- Calculate mean (X̄) and standard deviation (σ)
- Set control limits at X̄ ± 3σ

**Example: Wafer defect density control chart**
- Data: 50 lots, average = 0.065 defects/cm², σ = 0.015
- UCL = 0.065 + 3(0.015) = 0.110
- LCL = 0.065 - 3(0.015) = 0.020
- **Action rule**: Investigate if any lot >UCL or <LCL, or if 7+ consecutive lots above/below mean

**Automation**:
- Integrate with MES (manufacturing execution system)
- Automatic alerts when out-of-control conditions detected
- Dashboard for real-time visualization

---

## 5. Phase 4: Full Deployment & Continuous Improvement

### **5.1 Transition to Full Production**

**Volume transition**:
- Month 4: 50-70% (if ramp successful)
- Month 5: 80-100%
- Month 6+: 100% (baseline material phased out)

**Lessons learned review**:
- Document all issues encountered and resolutions
- Update qualification procedures based on learnings
- Share best practices with other sites (multi-site deployments)

**Supplier alignment**:
- Transition from qualification support to routine production mode
- Establish QBR (quarterly business review) cadence
- Agree on continuous improvement targets (5-10% defect reduction annually)

### **5.2 Continuous Improvement Framework**

**Plan-Do-Check-Act (PDCA) cycle**:

**Plan**:
- Set annual improvement goals (e.g., reduce defect density by 20%, improve LER by 10%)
- Identify root causes of current issues (Pareto analysis)
- Design experiments or process changes

**Do**:
- Implement changes on limited volume (10-20% of production)
- Collect data rigorously (sample size adequate for statistical significance)

**Check**:
- Analyze results (compare to baseline, statistical tests)
- Verify improvement is real and sustainable (not sampling error)

**Act**:
- If successful: Deploy broadly, update standard procedures
- If unsuccessful: Investigate failure, iterate with new plan

**Example improvement project**: Reduce wafer defect density

**Current state**: 0.08 defects/cm² average, target: <0.06 defects/cm²

**Root cause analysis**:
- Pareto chart shows 60% of defects are particles (vs. pits, scratches)
- Particle size distribution: 70% are 0.09-0.15 µm (likely from wafer handling)

**Hypothesis**: Improved wafer cassette cleaning will reduce particle defects

**Experiment**:
- Clean cassettes with enhanced procedure (ultrasonics + SPM clean)
- Run 25 lots with enhanced-cleaned cassettes, 25 with standard cleaning (control)

**Results**:
- Enhanced cleaning: 0.052 defects/cm² average (20% particles, 80% pits/scratches)
- Standard cleaning: 0.078 defects/cm² average (60% particles, 40% pits/scratches)
- **Conclusion**: Enhanced cleaning effective, deploy to all cassettes

**Implementation**:
- Train all technicians on new cassette cleaning procedure
- Update SOP (standard operating procedure)
- Purchase ultrasonic cleaner for each wafer prep area ($20K total)
- **Result**: Defect density reduced to 0.060 defects/cm² (25% improvement, target achieved)
- **ROI**: $20K investment, savings = $1.5M annually (from improved yield)

### **5.3 Technology Refresh Cycles**

**Semiconductor materials evolve** as technology nodes advance:

**Trigger events for material updates**:
- New technology node introduction (e.g., 7nm → 5nm → 3nm)
- Supplier end-of-life notification (formulation changes, facility closures)
- Performance degradation (yield decline, new defect mechanisms)
- Cost optimization opportunities (new, lower-cost suppliers qualified)
- Regulatory changes (RoHS, REACH restrictions on current materials)

**Refresh frequency**:
- **Silicon wafers**: Every 2-5 years (driven by node transitions, supplier optimization)
- **Photoresists**: Every 1-3 years (frequent formulation updates for new nodes)
- **Specialty gases**: Every 3-7 years (more stable, driven by purity/cost improvements)

**Refresh process**: Abbreviated re-qualification
- Leverage existing qualification framework
- Focus on deltas (what changed vs. current material)
- Accelerated timeline (3-6 months vs. 6-12 for new qualification)

---

## 6. Success Metrics and KPIs

### **6.1 Material Quality Metrics**

**Leading indicators** (predict future performance):
- Supplier quality audit scores (quarterly)
- IQC defect detection rate (% of lots with issues caught at receiving)
- Material specification compliance (% of parameters within spec)
- SPC control chart: % of time in statistical control

**Lagging indicators** (outcome measures):
- Wafer defect density trend (monthly average)
- Photoresist defectivity (defects per wafer after develop)
- Lot reject rate (% of lots rejected due to material issues)
- Yield impact (% yield loss attributed to materials)

**Targets** (example):
- IQC detection rate: >90% (catch issues before production)
- Specification compliance: >99.5% of parameters
- SPC in-control: >95% of time
- Defect density: <0.08 defects/cm² (moving average)
- Lot reject rate: <0.5%
- Material-related yield loss: <0.2%

### **6.2 Supply Chain Metrics**

**Delivery performance**:
- On-time delivery (OTD): >98%
- Order fill rate: >99.5%
- Lead time: Actual vs. committed (track trends)

**Inventory efficiency**:
- Inventory days of supply (DOS): Target range (e.g., 7-14 days for wafers)
- Inventory turns: >26 per year (bi-weekly replenishment)
- Obsolescence rate: <1% of inventory value

**Cost metrics**:
- Purchase price variance (PPV): Actual vs. budget
- Total cost of ownership (TCO): Trending down 2-5% annually
- Cost per wafer (CPW): Material cost component

**Supplier performance**:
- Supplier scorecard (composite): >85/100 points
- Audit findings: <3 major findings per year
- Technical support responsiveness: <24 hours average

### **6.3 Business Impact Metrics**

**Yield**:
- Overall yield (baseline vs. post-implementation): +1-5% absolute
- Material-related defect density contribution: Decreasing trend

**Reliability**:
- Field failure rate: Stable or decreasing (no material-related excursions)
- Reliability stress test pass rate: >95%

**Cost savings**:
- Material cost reduction: $2M-10M annually (TCO optimization)
- Yield improvement value: $10M-100M annually (large fab)
- Scrap/rework reduction: $1M-5M annually

**Risk reduction**:
- Supply disruption incidents: 0 per year (vs. X in previous year)
- Non-compliance findings: 0 (regulatory, customer audits)

**Overall ROI**:
- Formula: (Total annual benefits - Annual costs) / Total implementation investment
- Target: >200% (2× return) over 3 years, >500% (5×) over 5 years

**Example calculation**:
- Implementation cost: $8M (capital + project expenses)
- Annual benefits: $25M (yield improvement + cost reduction)
- Annual ongoing costs: $5M
- Net annual benefit: $20M
- 3-year ROI: ($20M × 3 - $8M) / $8M = 650%

---

## 7. Change Management and Training

### **7.1 Stakeholder Engagement**

**Key stakeholders**:
- Executive leadership (budget approval, strategic alignment)
- Operations management (production impact, resource allocation)
- Engineering teams (technical implementation)
- Quality assurance (new procedures, acceptance criteria)
- Procurement (supplier negotiations, contracts)
- EHS (safety, environmental compliance)

**Communication plan**:
- **Kickoff meeting**: Present business case, timeline, roles
- **Monthly steering committee**: Progress updates, issue escalation
- **Weekly working group**: Technical details, hands-on coordination
- **Quarterly all-hands**: Celebrate milestones, share results

### **7.2 Training Programs**

**Material handling training** (all technicians, 4 hours):
- Wafer cassette loading/unloading procedures
- Contamination prevention (glove protocol, cleanroom discipline)
- Photoresist storage and dispensing (temperature control, FIFO)
- Gas cylinder changeout procedures (safety, leak check)
- Hands-on practice with supervision

**Inspection and testing** (QC inspectors, 16 hours):
- Operation of dimensional metrology tools
- Particle scanner interpretation
- AFM basics (for surface roughness spot checks)
- CoA review and database entry
- Non-conformance reporting

**SPC and data analysis** (engineers, 8 hours):
- Control chart interpretation
- Out-of-control detection rules
- Capability index calculations (Cp, Cpk)
- Software training (Minitab, JMP, or similar)

**Supplier management** (procurement, 8 hours):
- WIA-SEMI-018 requirements overview
- Supplier scorecard methodology
- Audit checklists and techniques
- Contract negotiation best practices

**Certification**:
- Written test (80% pass mark)
- Practical demonstration (for hands-on skills)
- Annual refresher training (4 hours)
- Document training records (regulatory compliance)

---

## 8. Common Pitfalls and How to Avoid Them

### **8.1 Implementation Challenges**

**Inadequate planning**:
- **Symptom**: Delays, budget overruns, scope creep
- **Prevention**: Detailed project plan with milestones, contingency budget (20%), executive sponsorship

**Supplier qualification shortcuts**:
- **Symptom**: Early production issues, yield excursions, field failures
- **Prevention**: Follow full qualification protocol, no steps skipped even under schedule pressure

**Poor change management**:
- **Symptom**: Resistance from technicians, errors due to unfamiliarity
- **Prevention**: Involve frontline staff early, comprehensive training, clear communication of benefits

**Insufficient statistical rigor**:
- **Symptom**: Approve materials that later cause yield issues (false positives)
- **Prevention**: Adequate sample sizes (power analysis), statistical tests for significance, peer review of data

**Lack of traceability**:
- **Symptom**: Cannot identify root cause when issues occur
- **Prevention**: Implement MES integration from day one, enforce barcode scanning discipline

### **8.2 Sustaining Momentum**

**Post-deployment complacency**:
- **Risk**: After successful deployment, attention shifts elsewhere, continuous improvement stops
- **Mitigation**: Set annual improvement targets, tie to performance reviews, celebrate successes

**Supplier performance degradation**:
- **Risk**: Over time, supplier quality or service declines
- **Mitigation**: Quarterly scorecards, annual audits, escalation process for issues

**Knowledge loss**:
- **Risk**: Key personnel leave, tribal knowledge disappears
- **Mitigation**: Document all procedures (SOPs), cross-train team members, knowledge management system

---

## ✅ **Chapter Summary**

**Key Takeaways**:

1. WIA-SEMI-018 implementation follows four phases over 12-24 months: Assessment & Planning → Qualification & Validation → Production Ramp → Full Deployment

2. Typical investment is $4M-15M (capital and operating costs), with ROI of 2-10× over 3-5 years driven by yield improvement and cost optimization

3. Material qualification requires 3-6 months (wafers), 4-6 months (photoresists), 2-4 months (gases) with comprehensive characterization and process integration testing

4. Production ramp follows gradual volume increase (10% → 20% → 30% → 100%) with statistical process control and continuous monitoring

5. Success metrics span material quality (defect density, spec compliance), supply chain (OTD, inventory turns), and business impact (yield, cost savings, ROI)

6. Change management and training are critical: engage all stakeholders, certify personnel, maintain training records

7. Continuous improvement through PDCA cycles sustains benefits and drives ongoing performance gains (5-10% annual defect reduction target)

**Implementation Checklist**:

✅ Gap analysis completed, priorities identified
✅ Budget approved, resources allocated
✅ Project team formed, roles defined
✅ Suppliers selected, samples requested
✅ Qualification plan developed, timelines set
✅ Characterization tools and procedures ready
✅ Process integration tests designed
✅ SPC system configured
✅ Training materials prepared
✅ Communication plan in place
✅ Risk mitigation strategies documented
✅ Success criteria and KPIs agreed upon

**You are now ready to implement WIA-SEMI-018 and elevate your semiconductor material program to world-class standards!**

---

## 📚 **Additional Resources**

**WIA-SEMI-018 Support**:
- Technical support: support@wia-standards.org
- Implementation consulting: consulting@wia-standards.org
- Training programs: training@wia-standards.org
- Certification: certification@wia-standards.org

**Standards and References**:
- SEMI Standards (www.semi.org): M1, C1, S2, F49
- ITRS/IRDS Roadmap (irds.ieee.org): Technology scaling trends
- ISO 9001, IATF 16949: Quality management systems

**Industry Organizations**:
- SEMI (Semiconductor Equipment and Materials International)
- IEEE (Institute of Electrical and Electronics Engineers)
- IMEC (Research and innovation in nanoelectronics)
- SEMATECH (Semiconductor Manufacturing Technology consortium)

**Publications**:
- Semiconductor International
- Solid State Technology
- Journal of the Electrochemical Society
- IEEE Transactions on Semiconductor Manufacturing

---

## 🌟 **Closing Message**

Implementing WIA-SEMI-018 represents a commitment to excellence in semiconductor material management. By following the roadmap outlined in this chapter and throughout this ebook, your organization will:

✓ Achieve world-class material quality and consistency
✓ Optimize total cost of ownership
✓ Reduce supply chain risk and improve resilience
✓ Enable successful technology node transitions
✓ Meet or exceed customer expectations for reliability and performance

The journey requires investment, discipline, and sustained effort—but the rewards in yield improvement, cost savings, and competitive advantage are well worth it.

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

Through rigorous material standards and continuous improvement, we enable the semiconductor industry to deliver the advanced chips that power human progress—from smartphones to supercomputers, from medical devices to sustainable energy systems.

Thank you for investing your time in mastering WIA-SEMI-018. We wish you great success in your implementation journey!

---

**About the Author**

This ebook was developed by the WIA Semiconductor Standards Committee in collaboration with leading fabs, material suppliers, equipment vendors, and research institutions worldwide. Special thanks to the hundreds of engineers and scientists who contributed their expertise to make WIA-SEMI-018 a comprehensive and practical standard.

**Feedback Welcome**

We continuously improve WIA-SEMI-018 based on user feedback. Please share your implementation experiences, suggestions, and questions:
- Email: semiconductor@wia-standards.org
- GitHub: https://github.com/WIA-Official/wia-standards
- Ebook purchase: https://wiabooks.store/tag/wia-semiconductor-material/

---

**© 2025 SmileStory Inc. / World Certification Industry Association (WIA)**

*Licensed under CC BY-SA 4.0 - Free to share and adapt with attribution*

**弘益人間** · Benefit All Humanity
