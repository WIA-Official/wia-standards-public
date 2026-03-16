# WIA-FIN-020: Credit Scoring Standard v1.1

**Status:** Final  
**Date:** 2025-03-01  
**Changes from v1.0:** Added alternative data guidelines, enhanced fairness metrics

## Changes in v1.1

### Added
- Section 3.2.1: Specific alternative data provider requirements
- Section 5.2.1: Calibration metric (detailed by demographic group)
- Section 7.3: Webhook notification endpoints

### Enhanced
- Fairness metrics now include calibration testing
- Alternative data consent management requirements
- Real-time monitoring specifications

[Rest of spec inherits from v1.0 with above additions]

## 3.2.1 Alternative Data Providers (NEW)

Approved alternative data sources:
- **Bank Transactions:** Plaid, Yodlee, Finicity, MX
- **Rent Payments:** RentTrack, ClearNow, PayYourRent
- **Utilities:** Experian Boost, eCredable, PRBC
- **Employment:** The Work Number, Truework
- **Education:** National Student Clearinghouse

Data freshness requirements:
- Bank transactions: < 7 days
- Employment verification: < 30 days  
- Rent history: < 60 days

## 5.2.1 Calibration by Demographic (NEW)

For each protected group, verify:
```
|Predicted_Default_Rate - Actual_Default_Rate| < 0.02
```

Test across score bands (e.g., 600-650, 650-700, etc.)

Report calibration metrics quarterly to stakeholders.

## 7.3 Webhook Endpoints (NEW)

```json
POST /api/v1/webhooks/register
{
  "url": "https://yourapp.com/webhooks/credit",
  "events": ["score.completed", "score.updated", "alert.delinquency"],
  "secret": "webhook_secret_key"
}
```

---

© 2025 SmileStory Inc. / WIA
