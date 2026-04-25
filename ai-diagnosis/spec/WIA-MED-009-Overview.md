# WIA-MED-009: AI Diagnosis Standard
## Official Specification v1.0.0

**Published:** 2025-01-18
**Status:** Active
**License:** MIT
**Organization:** World Certification Industry Association (WIA)

---

## 1. Executive Summary

WIA-MED-009 defines international standards for AI-powered medical diagnosis systems. This standard ensures safety, accuracy, fairness, and regulatory compliance for AI systems that assist or perform medical diagnosis.

### 1.1 Scope

This standard applies to:
- Clinical Decision Support Systems (CDSS)
- Medical image analysis AI
- Natural language processing for medical records
- Multi-modal diagnostic AI systems
- Predictive and preventive medicine AI

### 1.2 Key Principles

1. **Safety First** — Patient safety is paramount
2. **Transparency** — AI decisions must be explainable
3. **Fairness** — Equitable performance across all populations
4. **Human-Centered** — AI augments, not replaces, physicians
5. **Continuous Improvement** — Ongoing monitoring and updates

---

## 2. Technical Requirements

### 2.1 Performance Metrics

#### Minimum Standards

| Metric | Class II (Assistive) | Class III (Autonomous) |
|--------|---------------------|----------------------|
| Sensitivity | ≥ 85% | ≥ 90% |
| Specificity | ≥ 85% | ≥ 90% |
| AUC-ROC | ≥ 0.90 | ≥ 0.95 |
| Accuracy | ≥ 90% | ≥ 95% |

### 2.2 Data Requirements

#### Training Data

- **Minimum Size:**
  - Simple tasks: ≥ 1,000 samples
  - Complex tasks: ≥ 10,000 samples
  - Deep learning: ≥ 50,000 samples

- **Data Quality:**
  - Label accuracy: ≥ 95% (expert-verified)
  - Missing data: < 10%
  - Class balance: 1:10 ratio or use resampling

- **Diversity:**
  - Multiple demographics (age, gender, ethnicity)
  - Geographic diversity
  - Different acquisition devices/protocols

#### Validation Data

- **Independent Test Set:** ≥ 15% of total data
- **Multi-Institution:** ≥ 3 independent sites
- **Temporal Validation:** Data from different time periods

### 2.3 Model Architecture

#### Supported Approaches

1. **Machine Learning:**
   - Random Forest, XGBoost, SVM
   - Logistic Regression (baseline)
   - Ensemble methods

2. **Deep Learning:**
   - CNN (ResNet, DenseNet, EfficientNet)
   - Transformers (BERT, Clinical BERT)
   - Multi-modal networks

3. **Hybrid Systems:**
   - Rule-based + ML
   - Symbolic AI + Neural Networks

### 2.4 Explainability (XAI)

**Mandatory Requirements:**

- **Global Interpretability:** Feature importance scores
- **Local Interpretability:** Instance-level explanations
- **Visual Explanations:** Grad-CAM, attention maps (for imaging)
- **Text Explanations:** Natural language justifications

**Approved Methods:**
- LIME (Local Interpretable Model-agnostic Explanations)
- SHAP (SHapley Additive exPlanations)
- Grad-CAM (Gradient-weighted Class Activation Mapping)
- Attention Mechanisms

---

## 3. Fairness and Bias

### 3.1 Subgroup Analysis

**Required Demographic Breakdowns:**

- **Gender:** Male, Female, Non-binary
- **Age:** Pediatric (< 18), Adult (18-65), Elderly (> 65)
- **Ethnicity:** Major represented groups (≥ 500 samples each)

**Fairness Criteria:**

- Performance difference between groups: < 5%
- Minimum performance in all groups: ≥ 85%

### 3.2 Bias Detection

**Required Tests:**

1. **Equalized Odds:** Equal TPR and FPR across groups
2. **Demographic Parity:** Equal positive prediction rates
3. **Calibration:** Predicted probabilities match actual outcomes

### 3.3 Bias Mitigation

**Strategies:**

- Pre-processing: Resampling, augmentation
- In-processing: Fairness constraints during training
- Post-processing: Threshold optimization per group

---

## 4. Clinical Validation

### 4.1 Study Types

1. **Retrospective Studies:**
   - Historical patient data
   - Minimum: 500 patients
   - Use: Initial validation

2. **Prospective Studies:**
   - Real-time data collection
   - Minimum: 300 patients
   - Use: Real-world validation

3. **Randomized Controlled Trials (RCT):**
   - AI vs. Standard of Care
   - Power analysis required
   - Use: Class III devices, novel indications

### 4.2 Endpoints

**Primary:**
- Diagnostic accuracy (sensitivity, specificity)
- Clinical utility (change in management)
- Patient outcomes (morbidity, mortality)

**Secondary:**
- Time to diagnosis
- Cost-effectiveness
- Physician satisfaction
- Patient satisfaction

---

## 5. Regulatory Compliance

### 5.1 FDA (United States)

**Pathways:**

- **510(k):** Substantial equivalence to predicate
- **De Novo:** Novel low-to-moderate risk device
- **PMA:** High-risk devices requiring clinical trials

**Documentation:**
- Intended use statement
- Algorithm description
- Training/validation data
- Performance metrics
- Risk analysis
- Software verification & validation

### 5.2 MFDS (South Korea)

**Classification:**

- Class 1: Notification
- Class 2: Certification (3-6 months)
- Class 3-4: Approval (6-18 months)

**Requirements:**
- Technical documentation (Korean)
- Clinical data (domestic or foreign)
- Quality management (GMP or ISO 13485)
- User manual (Korean)

### 5.3 CE Mark (Europe)

**MDR Compliance:**

- Risk classification (Rule 11)
- Conformity assessment (Notified Body)
- Clinical evaluation (mandatory)
- Post-market surveillance

---

## 6. Security and Privacy

### 6.1 Data Protection

**Requirements:**

- **Encryption:** AES-256 (data at rest and in transit)
- **De-identification:** HIPAA Safe Harbor (remove 18 identifiers)
- **Access Control:** Role-Based Access Control (RBAC)
- **Audit Logs:** All data access tracked

### 6.2 Compliance

- **HIPAA** (USA)
- **GDPR** (EU)
- **PIPA** (Korea)

### 6.3 Cybersecurity

**Standards:**

- IEC 62443 (Industrial Cybersecurity)
- FDA Cybersecurity Guidance
- NIST Cybersecurity Framework

**Requirements:**
- Penetration testing
- Vulnerability assessments
- Incident response plan
- Software updates

---

## 7. Post-Market Surveillance

### 7.1 Performance Monitoring

**Metrics to Track:**

- Accuracy (monthly)
- Response time (real-time)
- Uptime/availability (99.9%)
- Physician acceptance rate

**Triggers for Action:**

- Performance drop > 5%
- Adverse events
- User complaints

### 7.2 Adverse Event Reporting

**Requirements:**

- Serious injury/death: 15 days
- Malfunction: 30 days
- Corrective actions: As needed (recall, update)

---

## 8. WIA Certification

### 8.1 Certification Levels

| Level | Requirements | Badge |
|-------|-------------|-------|
| **Bronze** | Basic performance standards | 🥉 WIA Bronze |
| **Silver** | + Fairness evaluation | 🥈 WIA Silver |
| **Gold** | + Clinical trials + Multi-site | 🥇 WIA Gold |
| **Platinum** | + FDA/MFDS + RCT | 💎 WIA Platinum |

### 8.2 Application Process

1. Online application: https://cert.wiastandards.com
2. Documentation submission
3. Expert review (2-4 weeks)
4. Site inspection (if needed)
5. Certificate issuance
6. Registry listing

### 8.3 Costs

- Bronze: $1,000 USD
- Silver: $2,500 USD
- Gold: $5,000 USD
- Platinum: $10,000 USD
- Startup/Academic: 50% discount (eligible)

---

## 9. Implementation Checklist

### 9.1 Development Phase

- [ ] Define intended use and target population
- [ ] Collect diverse, high-quality training data
- [ ] Implement model with XAI
- [ ] Evaluate performance (test set)
- [ ] Conduct fairness analysis
- [ ] Document all decisions

### 9.2 Validation Phase

- [ ] Multi-institution validation
- [ ] Prospective study
- [ ] Clinical trial (if Class III)
- [ ] Regulatory submission
- [ ] Security assessment

### 9.3 Deployment Phase

- [ ] EHR/PACS integration
- [ ] Physician training
- [ ] Performance monitoring
- [ ] Adverse event reporting
- [ ] Continuous improvement

---

## 10. References

### 10.1 Standards

- ISO 13485: Medical Devices Quality Management
- ISO 14971: Risk Management for Medical Devices
- IEC 62304: Medical Device Software Lifecycle
- IEC 62443: Cybersecurity for Medical Devices

### 10.2 Guidelines

- FDA Software as Medical Device (SaMD)
- FDA Artificial Intelligence/Machine Learning (AI/ML)
- EU MDR 2017/745
- MFDS Medical Device Act

### 10.3 Literature

- 선행 연구 Dermatologist-level classification
- 선행 연구 Diabetic retinopathy detection
- 선행 연구 CheXNet pneumonia detection
- 선행 연구 Breast cancer screening

---

## Contact

**WIA Certification Office**
Email: cert@wiastandards.com
Web: https://wiastandards.com
GitHub: https://github.com/WIA-Official/wia-standards

**Philosophy:** 弘益人間 (Benefit All Humanity)

**License:** MIT License
**Copyright:** © 2025 World Certification Industry Association

---

*For full ebook with detailed implementation examples, visit https://wiabook.com*
