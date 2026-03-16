# WIA-FIN-020: Credit Scoring Standard v1.2

**Status:** Final  
**Date:** 2025-06-15  
**Changes:** Open Banking integration, enhanced explainability

## Changes in v1.2

### Added
- Section 3.4: Open Banking API requirements
- Section 6.4: Consumer credit dashboard specifications
- Enhanced ML model requirements (neural networks)

## 3.4 Open Banking Integration (NEW)

### API Standards
- PSD2 compliance (EU)
- Account Information Service Provider (AISP) authorization
- Secure Customer Authentication (SCA)

### Data Elements
Required from bank APIs:
- Account balances (checking, savings)
- Transaction history (90 days minimum)
- Income deposits (categorized)
- Recurring expenses identification
- Overdraft history

### Privacy Requirements
- Explicit consumer consent via OAuth 2.0
- Data retention: Maximum 13 months
- Right to revoke access at any time
- Data deletion within 30 days of revocation

## 6.4 Consumer Credit Dashboard (NEW)

Provide consumers with:
- Current credit score and trend (6 months)
- Top factors affecting score
- Personalized recommendations for improvement
- Score simulator ("what-if" scenarios)
- Credit monitoring alerts

API endpoint:
```
GET /api/v1/consumer/dashboard/{applicant_id}
```

---

© 2025 SmileStory Inc. / WIA
