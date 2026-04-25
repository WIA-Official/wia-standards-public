# WIA-MED-023: Medical AI Ethics Standard v1.0.0

## Overview

**Standard ID:** WIA-MED-023
**Title:** Medical AI Ethics Standard
**Version:** 1.0.0
**Status:** Published
**Date:** 2025-12-26
**Organization:** WIA (World Certification Industry Association)
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Abstract

This standard establishes a comprehensive ethical framework for artificial intelligence systems used in medical diagnosis, treatment planning, and patient care. It addresses critical issues including algorithmic bias, fairness, informed consent, liability, transparency, patient autonomy, human oversight, and global ethical frameworks.

## Scope

### In Scope
- AI systems for medical imaging analysis
- Clinical decision support systems
- Diagnostic AI algorithms
- Treatment recommendation systems
- Patient risk stratification AI
- AI-powered medical devices
- Healthcare resource allocation algorithms
- Predictive analytics in medicine

### Out of Scope
- Non-medical AI applications
- Research-only AI systems not deployed in clinical care
- Traditional (non-AI) medical software
- Administrative healthcare AI (billing, scheduling)

## 1. Core Ethical Principles

### 1.1 Patient-Centricity
- Patient welfare is the paramount consideration in all AI design and deployment
- AI must augment, not replace, patient-centered care
- Patient preferences and values must be integrated into AI-assisted decisions

### 1.2 Fairness and Non-Discrimination
- AI systems MUST perform equitably across all demographic groups
- Algorithmic bias MUST be actively detected and mitigated
- No patient population may be systematically disadvantaged by AI

**Technical Requirements:**
- Fairness metrics must be evaluated on stratified test sets
- Maximum allowable performance gap: 5% across protected groups
- Annual fairness audits required

### 1.3 Transparency and Explainability
- AI decision-making processes MUST be understandable to qualified healthcare professionals
- Explanation mechanisms MUST be provided appropriate to the audience (clinicians, patients, regulators)
- Algorithms, training data characteristics, and limitations MUST be documented

**Technical Requirements:**
- Implement at least one XAI (Explainable AI) technique (SHAP, LIME, saliency maps, etc.)
- Provide feature importance rankings for predictions
- Document confidence intervals and uncertainty estimates

### 1.4 Accountability
- Clear responsibility structures MUST exist for AI-related errors
- Mechanisms for reporting and investigating AI failures MUST be established
- Continuous quality monitoring MUST be implemented

### 1.5 Privacy and Data Protection
- Patient data MUST be protected according to applicable regulations (HIPAA, GDPR, etc.)
- Data minimization principle MUST be applied
- De-identification MUST be used when possible
- Patient consent for data use MUST be obtained

### 1.6 Safety and Efficacy
- AI systems MUST undergo rigorous clinical validation before deployment
- Performance must meet or exceed existing standard of care
- Post-market surveillance MUST monitor real-world performance
- Adverse event reporting systems MUST be in place

### 1.7 Human Autonomy and Oversight
- Humans MUST retain ultimate decision-making authority
- AI MUST serve as decision support, not autonomous decision-maker (except in pre-approved emergency scenarios)
- Mechanisms to override AI recommendations MUST exist
- Automation bias risks MUST be mitigated through training and system design

### 1.8 Equity and Sustainability
- AI technology SHOULD be accessible to resource-limited settings
- Environmental impact of AI systems SHOULD be minimized
- Long-term societal benefits MUST be considered

## 2. Bias and Fairness Requirements

### 2.1 Data Requirements
- Training data MUST include representative samples from all intended use populations
- Minimum representation: Each demographic subgroup ≥ 10% of total or ≥ 500 samples, whichever is larger
- Data provenance and demographic distributions MUST be documented
- Historical bias in data MUST be identified and addressed

### 2.2 Fairness Metrics
Medical AI systems MUST evaluate and report at least three of the following fairness metrics:

1. **Demographic Parity:** P(Ŷ=1|A=a) ≈ P(Ŷ=1|A=b)
2. **Equal Opportunity:** TPR_a ≈ TPR_b
3. **Equalized Odds:** TPR_a ≈ TPR_b AND FPR_a ≈ FPR_b
4. **Predictive Parity:** PPV_a ≈ PPV_b
5. **Calibration:** E[Y|Score=s,A=a] ≈ s for all groups

**Acceptance Criteria:**
- Chosen fairness metrics must not differ by more than 5% between any two groups
- Justification MUST be provided for fairness metric selection based on clinical context

### 2.3 Bias Mitigation
- Pre-deployment bias testing MUST be conducted
- If bias exceeds thresholds, mitigation techniques MUST be applied:
  - Data rebalancing
  - Algorithmic fairness constraints
  - Post-processing calibration
- Post-deployment monitoring for emergent bias MUST occur quarterly

## 3. Informed Consent Requirements

### 3.1 Disclosure
Patients MUST be informed when AI is used in their care, including:
- That AI is being used
- Purpose of AI use (diagnosis, treatment recommendation, etc.)
- AI accuracy and error rates
- Known limitations and potential for bias
- Alternatives to AI-assisted care

### 3.2 Consent Process
- Consent MAY be tiered (basic vs. detailed) to accommodate patient preferences
- Visual aids and plain language SHOULD be used
- Patients MUST have the option to decline AI use
- Separate consent REQUIRED for use of patient data in AI training

### 3.3 Special Populations
- Pediatric patients: Parent/guardian consent + age-appropriate assent
- Cognitively impaired: Legal representative consent + patient best interest
- Emergency situations: Waiver allowed with post-hoc explanation

## 4. Transparency and Explainability

### 4.1 Model Documentation
All medical AI systems MUST provide:
- Algorithm type and architecture
- Training data characteristics (size, source, demographics)
- Performance metrics (accuracy, sensitivity, specificity, AUC)
- Validation methodology
- Intended use and limitations
- Known failure modes

### 4.2 Explainability Methods
At least ONE of the following MUST be implemented:
- Feature importance scores
- Saliency maps (for image-based AI)
- SHAP values
- LIME explanations
- Counterfactual explanations
- Attention visualizations

### 4.3 Explanation Levels
Different explanation depths MUST be available for:
- **Patients:** Non-technical, outcome-focused
- **Clinicians:** Clinical reasoning, key factors, confidence levels
- **Researchers:** Technical details, algorithm specifics
- **Regulators:** Validation data, performance benchmarks

## 5. Liability and Accountability

### 5.1 Responsibility Structure
A clear chain of responsibility MUST define:
- Who is responsible for AI errors (developer, manufacturer, healthcare institution, clinician)
- How liability is allocated in multi-party scenarios
- Insurance and compensation mechanisms

### 5.2 Incident Reporting
- All AI-related adverse events MUST be reported to designated authority
- Serious incidents (patient harm) MUST be reported within 24 hours
- Root cause analysis MUST be conducted
- Corrective actions MUST be implemented and documented

### 5.3 Audit Trail
- All AI predictions and recommendations MUST be logged
- Logs MUST include input data, prediction, confidence, timestamp
- Logs MUST be retained for minimum 7 years
- Logs MUST be auditable by regulators

## 6. Human Oversight

### 6.1 Oversight Requirements
- AI recommendations MUST be reviewed by qualified healthcare professionals before implementation
- Clinicians MUST be trained on AI capabilities and limitations
- Mechanisms to override AI MUST be easily accessible
- Automation bias mitigation training REQUIRED

### 6.2 Automation Levels
Medical AI SHOULD operate at Levels 3-4 (decision support) and MUST NOT exceed Level 4 (requires human approval):

**Level 3:** AI proposes multiple options
**Level 4:** AI selects one option, human approves/rejects
**Level 5:** Fully autonomous (PROHIBITED except pre-approved emergency scenarios)

## 7. Validation and Testing

### 7.1 Pre-Deployment Validation
- Internal validation on held-out test set
- External validation on independent dataset
- Prospective clinical trial (for high-risk applications)
- Subgroup analysis across demographics

### 7.2 Performance Benchmarks
AI MUST demonstrate:
- Non-inferiority to existing standard of care (p < 0.05)
- OR superiority with clinically meaningful improvement (≥ 5% increase in primary outcome)

### 7.3 Post-Deployment Monitoring
- Continuous performance tracking in real-world use
- Monthly performance reports
- Quarterly fairness audits
- Alert system for performance degradation (> 5% drop)

## 8. Data Governance

### 8.1 Data Collection
- Minimum Data Necessary principle
- Explicit consent for each use case
- Secure storage with encryption at rest and in transit
- Access controls and audit logs

### 8.2 Data Retention
- Clinical use data: Per institutional policy (typically 7+ years)
- Training data: Retained for model lifecycle
- Deletion upon patient request (where feasible)

### 8.3 Data Sharing
- De-identification REQUIRED for external sharing
- Data use agreements REQUIRED
- Compliance with jurisdictional regulations (GDPR, HIPAA, etc.)

## 9. Regulatory Compliance

### 9.1 Jurisdictional Requirements
Medical AI MUST comply with applicable regulations:
- **FDA (US):** AI/ML-based Software as Medical Device guidance
- **EU:** AI Act + Medical Device Regulation (MDR)
- **UK:** MHRA guidance on AI as medical device
- **Others:** As applicable by deployment region

### 9.2 Classification
Medical AI systems are classified as high-risk and subject to:
- Pre-market approval or clearance
- Quality management systems (ISO 13485)
- Post-market surveillance
- Periodic re-certification

## 10. Ethical Review

### 10.1 Ethics Committee
All medical AI development projects SHOULD undergo review by an independent ethics committee composed of:
- Clinical experts (≥2)
- AI/ML expert (≥1)
- Bioethicist (≥1)
- Patient representative (≥1)
- Legal expert (≥1)
- Diversity/equity specialist (≥1)

### 10.2 Review Points
- Initial project approval
- Pre-deployment review
- Annual review of deployed systems
- Review after significant algorithm changes

## 11. Global Harmonization

WIA-MED-023 aligns with and synthesizes:
- WHO Ethics and Governance of AI for Health (2021)
- EU AI Act (2024)
- IEEE Ethically Aligned Design
- UNESCO Recommendation on Ethics of AI (2021)
- National guidelines (US, UK, Canada, Singapore, Japan, Korea, China, etc.)

## 12. Certification

Organizations implementing WIA-MED-023 compliant medical AI MAY seek certification:
- **WIA Certified Medical AI - Level 1:** Basic compliance
- **WIA Certified Medical AI - Level 2:** Advanced compliance with enhanced fairness and transparency
- **WIA Certified Medical AI - Level 3:** Gold standard with continuous monitoring and global best practices

Certification body: [WIA Certification Authority](https://cert.wiastandards.com)

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-26 | Initial release |

## References

1. WHO. (2021). Ethics and governance of artificial intelligence for health.
2. European Commission. (2024). EU AI Act.
3. FDA. (2021). AI/ML-Based Software as Medical Device Action Plan.
4. IEEE. (2019). Ethically Aligned Design.
6. Chouldechova, A. (2017). Fair prediction with disparate impact. Big Data, 5(2), 153-163.

## Contact

**WIA (World Certification Industry Association)**
Website: https://wiastandards.com
Certification: https://cert.wiastandards.com
Email: standards@wiastandards.com
GitHub: https://github.com/WIA-Official/wia-standards

---

© 2025 WIA. Licensed under MIT License.

弘益人間 · Benefit All Humanity
