# WIA-FIN-020: Credit Scoring Standard v1.0

**Status:** Final  
**Date:** 2025-01-15  
**Category:** Finance/Economy  
**ID:** WIA-FIN-020  
**Slug:** credit-scoring  
**Emoji:** 📊  

## Abstract

The WIA-FIN-020 Credit Scoring Standard defines a comprehensive framework for implementing fair, accurate, and explainable credit assessment systems using AI/ML technologies while maintaining regulatory compliance.

## 1. Introduction

### 1.1 Purpose
This standard establishes guidelines for:
- AI-powered credit scoring models
- Alternative data integration
- Fairness and bias mitigation
- Regulatory compliance (FCRA, ECOA, GDPR)
- Model explainability and transparency

### 1.2 Scope
Applies to all credit assessment systems including:
- Consumer lending (personal loans, credit cards, mortgages)
- Commercial lending (SMB, business credit)
- Fintech applications (BNPL, digital lending)
- Behavioral scoring and account management

## 2. Architecture

### 2.1 System Components
1. **Data Ingestion Layer**: Credit bureaus, alternative data, applicant input
2. **Feature Engineering**: Transform raw data into 500+ predictive features
3. **ML Model Service**: XGBoost/Neural network ensemble
4. **Decision Engine**: Policy rules and risk-based pricing
5. **Explainability Service**: SHAP-based factor analysis
6. **Monitoring**: Performance tracking and drift detection

### 2.2 Data Flow
```
Application → Data Collection → Feature Engineering → 
Model Inference → Decision Logic → Score + Explanation
```

## 3. Data Requirements

### 3.1 Traditional Credit Bureau Data
- Payment history (24+ months)
- Credit utilization
- Account mix and ages
- Inquiries and derogatory marks

### 3.2 Alternative Data (Optional)
- Bank transaction data (Open Banking)
- Utility and rent payments
- Employment verification
- Telecom payment history

### 3.3 Data Quality Standards
- Missing value rate < 5% for critical features
- Data freshness: Bureau data < 30 days old
- Validation: 99.9% accuracy on key identifiers (SSN, DOB)

## 4. Model Requirements

### 4.1 Performance Metrics
- **Minimum AUC-ROC:** 0.80
- **Precision:** ≥ 0.75
- **Recall:** ≥ 0.70
- **Calibration:** Brier score < 0.15

### 4.2 Stability Requirements
- **PSI (Population Stability Index):** < 0.25 (warning at 0.10)
- **Out-of-time validation:** Performance decline < 5%
- **Stress testing:** Maintains AUC ≥ 0.75 in recession scenario

### 4.3 Approved Model Types
- Logistic regression (baseline)
- Gradient boosting (XGBoost, LightGBM)
- Neural networks (with explainability)
- Ensemble methods

## 5. Fairness Requirements

### 5.1 Prohibited Factors
MUST NOT use directly:
- Race, color, ethnicity
- Gender, sex
- Religion
- National origin
- Age (except as legally required)

### 5.2 Fairness Metrics
- **Disparate Impact Ratio:** ≥ 0.80 for all protected classes
- **Demographic Parity:** Approval rate variance < 10% across groups
- **Equal Opportunity:** TPR difference < 5% across groups
- **Calibration:** Predicted vs. actual default rates match across demographics

### 5.3 Proxy Variable Monitoring
- Regular correlation analysis with protected classes
- ZIP code, name-based features require justification
- Monthly fairness audits

## 6. Explainability Requirements

### 6.1 Score Factors
Provide top 3-5 factors for each decision:
- Factor name (consumer-friendly language)
- Impact magnitude (numerical or categorical)
- Direction (positive/negative influence)

### 6.2 Adverse Action Notices
When declining credit:
- Primary reason (most impactful negative factor)
- 2-4 secondary reasons
- How to obtain credit report
- Dispute process information

### 6.3 Technical Explainability
- SHAP values for ML models
- Feature importance tracking
- Contribution verification (sum = prediction - baseline)

## 7. API Specification

### 7.1 Score Request
```json
POST /api/v1/score
{
  "applicant": {
    "ssn": "***-**-1234",
    "dob": "1990-01-15",
    "income": 75000
  },
  "loan_request": {
    "amount": 25000,
    "term": 36,
    "purpose": "debt_consolidation"
  }
}
```

### 7.2 Score Response
```json
{
  "score": 745,
  "grade": "good",
  "decision": "approve",
  "approval_probability": 0.87,
  "default_probability": 0.023,
  "factors": [
    {
      "name": "Payment History",
      "impact": "high",
      "direction": "positive"
    }
  ],
  "suggested_terms": {
    "apr": 0.065,
    "max_amount": 30000
  }
}
```

## 8. Monitoring Requirements

### 8.1 Model Performance
Track weekly:
- AUC-ROC, precision, recall
- Approval rate, default rate
- Score distribution

### 8.2 Data Drift Detection
- Feature distribution shifts (KS test)
- Target variable drift
- Automatic alerts when PSI > 0.10

### 8.3 Business Metrics
- Origination volume
- Revenue per account
- Loss rates by score band

## 9. Compliance

### 9.1 Regulatory Requirements
- **FCRA:** Permissible purpose, adverse action notices, dispute resolution
- **ECOA:** No discrimination, adverse action reasons
- **GDPR:** Right to explanation, data minimization, consent
- **Model Risk Management (SR 11-7):** Documentation, validation, governance

### 9.2 Documentation
Required:
- Model development methodology
- Data sources and quality assessment
- Performance metrics and validation
- Fairness testing results
- Monitoring procedures

## 10. Security

### 10.1 Data Protection
- Encryption: TLS 1.3 in transit, AES-256 at rest
- Access control: Role-based, principle of least privilege
- Audit logging: All scoring decisions logged immutably

### 10.2 API Security
- Authentication: OAuth 2.0 or API keys
- Rate limiting: 1000 requests/minute per client
- Input validation: Strict schema enforcement

## 11. Versioning

This specification uses semantic versioning:
- MAJOR: Incompatible API changes
- MINOR: Backward-compatible functionality
- PATCH: Backward-compatible bug fixes

Current version: **1.0.0**

## 12. References

- Fair Credit Reporting Act (15 U.S.C. § 1681)
- Equal Credit Opportunity Act (15 U.S.C. § 1691)
- SR 11-7: Guidance on Model Risk Management
- EU General Data Protection Regulation (GDPR)

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (홍익인간) · Benefit All Humanity
