# WIA-FIN-020: Credit Scoring Standard v2.0

**Status:** Final  
**Date:** 2025-12-01  
**Breaking Changes:** New API schema, enhanced fairness requirements, blockchain integration

## Major Changes in v2.0

### Breaking Changes
1. **API Schema Update:** New required fields in score request/response
2. **Fairness Requirements:** Stricter disparate impact threshold (0.85 vs 0.80)
3. **Model Performance:** Minimum AUC increased to 0.82 (from 0.80)

### New Features
1. **Blockchain Credit Identity:** Support for decentralized credit records
2. **Continuous Scoring:** Real-time score updates based on transaction monitoring
3. **Multi-Modal AI:** Support for document image analysis
4. **Federated Learning:** Privacy-preserving model training across institutions

## 2.3 Blockchain Integration (NEW)

### Supported Protocols
- Ethereum-based identity (EIP-1056: Lightweight Identity)
- Verifiable Credentials (W3C standard)
- DID (Decentralized Identifier) resolution

### Use Cases
- Portable credit history across borders
- Self-sovereign credit identity
- Immutable payment records
- DeFi lending integration

API Support:
```json
POST /api/v2/score
{
  "identity": {
    "type": "did",
    "value": "did:ethr:0x123...",
    "credentials": ["vc_payment_history_123"]
  }
}
```

## 3.5 Multi-Modal Data Sources (NEW)

### Document Analysis
- Income verification (pay stubs, W-2s)
- Bank statements (OCR + validation)
- Utility bills
- Rental agreements

**ML Models:** Vision transformers, OCR pipelines  
**Accuracy Requirement:** 95%+ field extraction accuracy

### Transaction Pattern Analysis
- Natural language processing for transaction descriptions
- Spending category classification
- Financial health indicators
- Fraud pattern detection

## 4.4 Continuous Scoring (NEW)

Real-time score updates triggered by:
- New bank transactions
- Credit report updates
- Payment events
- Account balance changes

**Update Frequency:** Daily for active accounts  
**Score Volatility Control:** Maximum 10-point change per day

API Subscription:
```json
POST /api/v2/continuous-scoring/subscribe
{
  "account_id": "acc_123",
  "webhook_url": "https://...",
  "update_frequency": "daily"
}
```

## 5.3 Enhanced Fairness Requirements (NEW)

### Stricter Thresholds (v2.0)
- **Disparate Impact:** ≥ 0.85 (increased from 0.80)
- **Demographic Parity:** < 5% variance (decreased from 10%)
- **Equal Opportunity:** < 3% TPR difference (decreased from 5%)

### Intersectional Fairness
Test combinations of protected classes:
- Race × Gender
- Age × Income
- Geography × Ethnicity

### Mandatory Quarterly Audits
- Third-party fairness testing
- Public disclosure of key metrics
- Remediation plan for violations

## 7. API v2 Specification

### Enhanced Score Response
```json
{
  "version": "2.0",
  "score": 745,
  "confidence_interval": [732, 758],
  "grade": "good",
  "decision": {
    "outcome": "approve",
    "confidence": 0.87,
    "alternative_scenarios": [...]
  },
  "factors": [...],
  "continuous_monitoring": {
    "enabled": true,
    "next_update": "2025-12-02T00:00:00Z"
  },
  "explainability": {
    "shap_values": {...},
    "counterfactuals": [...]
  }
}
```

### Backwards Compatibility
- v1.x endpoints deprecated but supported until 2026-12-01
- Migration guide: `/docs/migration-v1-to-v2`

## 10. Advanced Features

### 10.1 Federated Learning
Participate in consortium model training:
- Privacy-preserving aggregation
- Contribution without data sharing
- Improved model accuracy across industry

### 10.2 AI-Generated Synthetic Data
- Training data augmentation
- Fairness testing scenarios
- Edge case simulation

### 10.3 Explainable AI Dashboard
Real-time visualization:
- SHAP waterfall plots
- Feature importance trends
- Counterfactual examples
- Demographic fairness metrics

---

**Migration Timeline:**
- 2025-12-01: v2.0 released
- 2026-06-01: v1.x deprecated
- 2026-12-01: v1.x support ends

© 2025 SmileStory Inc. / WIA  
弘益人間 (홍익인간) · Benefit All Humanity
