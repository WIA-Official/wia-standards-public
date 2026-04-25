# Chapter 6: Protocol Specifications

## Overview

This chapter details the operational protocols for implementing KYC/AML compliance procedures, including Customer Due Diligence (CDD), Enhanced Due Diligence (EDD), screening workflows, and transaction monitoring rules.

---

## Customer Due Diligence (CDD) Protocol

### Standard CDD Workflow

**Applicable To:**
- Low to medium risk customers
- Standard financial products
- Typical transaction patterns

**Process Steps:**

```
Step 1: Customer Information Collection
    ↓
Step 2: Identity Verification
    ↓
Step 3: Screening (Sanctions, PEP)
    ↓
Step 4: Risk Assessment
    ↓
Step 5: Approval Decision
    ↓
Step 6: Ongoing Monitoring
```

### Step-by-Step CDD Protocol

#### Step 1: Customer Information Collection

**Individual Customers - Required Information:**

| Category | Required Fields | Optional Fields |
|----------|----------------|-----------------|
| **Personal Info** | First name, Last name, Date of birth, Nationality | Middle name, Former names, Place of birth |
| **Identity Document** | Document type, Document number, Issue/expiry dates, Issuing country | Place of issue |
| **Contact** | Email, Phone, Residential address | Mailing address (if different) |
| **Financial** | Source of funds, Expected account activity | Source of wealth, Net worth estimate |
| **Employment** | Occupation | Employer name, Industry |

**Corporate Customers (KYB) - Required Information:**

| Category | Required Fields | Optional Fields |
|----------|----------------|-----------------|
| **Company Info** | Legal name, Registration number, Registration country, Incorporation date | Trading names, Former names |
| **Business Activity** | Industry, Business description | Annual revenue, Employee count |
| **Ownership** | Beneficial owners (>25% ownership), Ultimate beneficial owners | Intermediate entities |
| **Control** | Directors, Authorized signers | Board members |
| **Documentation** | Certificate of incorporation, Proof of registered address | Articles of association, Good standing certificate |

**Data Quality Standards:**
- ✅ All required fields must be complete
- ✅ Data format must conform to WIA schema
- ✅ Cross-field consistency (e.g., age matches DOB)
- ✅ No placeholder or dummy values
- ✅ Addresses must be complete (not just city/country)

#### Step 2: Identity Verification

**Individual Customers:**

**Level 1: Basic Verification (Low Risk)**
```
Required:
- Email verification (OTP)
- Phone verification (SMS OTP)
- Database cross-check (credit bureau or equivalent)

Acceptance Criteria:
- Email delivered and OTP validated
- Phone active and OTP validated
- Database match on at least 2 of 3: name, DOB, address
```

**Level 2: Standard Verification (Medium Risk)**
```
Required:
- Level 1 requirements +
- Government-issued ID document
  * Document authenticity check
  * Data extraction and validation
  * Expiry check (>3 months remaining)
- Address verification
  * Utility bill, bank statement, or government document
  * Issued within last 3 months
  * Name and address match

Acceptance Criteria:
- Document passes authenticity checks (confidence >85%)
- Extracted data matches provided information (>95% match)
- Address document verified
```

**Level 3: Enhanced Verification (High Risk)**
```
Required:
- Level 2 requirements +
- Biometric verification
  * Liveness detection (active or passive)
  * Face matching between document and live selfie
  * Match confidence >85%
- Video verification OR in-person verification (for highest risk)

Acceptance Criteria:
- Liveness check passed
- Face match confidence >85%
- Video/in-person verification completed if required
```

**Corporate Customers (KYB):**

```
Company Verification:
1. Business Registry Check
   - Confirm company exists and is active
   - Verify registration details
   - Check for dissolutions or strikes-off

2. Documentation Verification
   - Certificate of incorporation authentic
   - Articles of association provided
   - Good standing certificate (if available)

3. Address Verification
   - Confirm registered address
   - Verify operational address(es)

4. Ownership Verification
   - Identify beneficial owners (>25% threshold)
   - Verify each beneficial owner's identity (using individual CDD)
   - Document ownership structure

5. Control Person Verification
   - Identify directors and authorized signers
   - Verify each person's identity
   - Confirm authorities and powers
```

#### Step 3: Screening

**Sanctions Screening:**

```
Process:
1. Extract screening data:
   - Full name
   - Date of birth
   - Nationality
   - Identity document numbers
   - Addresses

2. Screen against lists:
   - OFAC SDN (US)
   - OFAC Non-SDN
   - UN Security Council Consolidated List
   - EU Sanctions List
   - UK HM Treasury Financial Sanctions
   - Jurisdiction-specific lists

3. Fuzzy matching:
   - Name variations (80%+ match threshold)
   - Transliterations
   - Aliases and nicknames
   - DOB tolerance (±1 year for review)

4. Result evaluation:
   Match Score      Action
   100%             Hard stop - manual review required
   85-99%           Soft stop - manual review required
   70-84%           Queue for review
   <70%             Clear - proceed

5. False positive review:
   - Compare DOB (exact match = likely true match)
   - Compare nationality
   - Compare addresses
   - Check aliases
   - Review additional identifiers
```

**PEP Screening:**

```
Process:
1. Screen against PEP databases
   - Government officials (all levels)
   - State-owned enterprise executives
   - Political party officials
   - Family members and close associates

2. PEP Tier Classification:
   Tier 1: Heads of state, senior politicians, senior government officials
   Tier 2: Mid-level government officials, military officers
   Tier 3: Family members, close business associates

3. Match Review:
   - Exact/high confidence match → Classify as PEP
   - Medium confidence → Manual review
   - Low confidence → Clear

4. If PEP Confirmed:
   - Elevate to Enhanced Due Diligence (EDD)
   - Document PEP status and tier
   - Obtain senior approval for onboarding
```

**Adverse Media Screening:**

```
Process:
1. Search negative news sources (past 5 years)
   Categories:
   - Financial crimes (money laundering, fraud)
   - Corruption
   - Terrorism
   - Organized crime
   - Regulatory enforcement actions
   - Criminal convictions

2. Relevance Assessment:
   - Confirm identity match (not same-name individual)
   - Assess severity and recency
   - Determine impact on risk rating

3. Action Based on Findings:
   - Major negative (e.g., conviction): Decline or EDD + senior approval
   - Minor negative (e.g., unproven allegation): Document and continue with CDD
   - No negative: Proceed
```

#### Step 4: Risk Assessment

**Risk Scoring Process:**

```
1. Geographic Risk Assessment:
   Input:
   - Customer residence country
   - Citizenship/nationality
   - Business operation countries
   - Expected transaction countries

   Scoring:
   - Low risk jurisdictions (FATF compliant): 0-20
   - Medium risk: 21-50
   - High risk (FATF high-risk list): 51-80
   - Prohibited (sanctioned countries): 81-100

2. Product/Service Risk Assessment:
   Input:
   - Account types requested
   - Services enabled (international wire, cash, etc.)
   - Expected transaction volumes

   Scoring:
   - Basic accounts (no international): 0-20
   - Standard with international wire: 21-40
   - High-value or complex products: 41-70
   - High-risk services (e.g., correspondent banking): 71-100

3. Customer Type Risk Assessment:
   Input:
   - Individual vs. business
   - Occupation / industry
   - Public exposure (PEP status)

   Scoring:
   - Individual, employed professional, not PEP: 0-20
   - Individual, self-employed or business: 21-40
   - SME, legitimate industry: 21-40
   - High-risk industry (MSB, crypto, gaming): 41-70
   - PEP: +20 points
   - Shell company indicators: 71-100

4. Behavioral Risk Assessment:
   Input:
   - Source of funds clarity
   - Transaction purpose rationale
   - Consistency with profile

   Scoring:
   - Clear, well-documented sources: 0-20
   - Partially documented: 21-40
   - Vague or inconsistent explanations: 41-70
   - Cannot explain or verify: 71-100

5. Relationship Risk Assessment:
   Input:
   - Known associates
   - Related entities
   - Screening results

   Scoring:
   - No adverse findings: 0-20
   - Minor adverse media: 21-40
   - High-risk associations: 41-70
   - Sanctions match or major criminal history: 71-100

6. Calculate Overall Risk Score:
   Weighted Average:
   Overall Score = (Geographic × 0.25) +
                   (Product × 0.20) +
                   (Customer Type × 0.20) +
                   (Behavioral × 0.20) +
                   (Relationship × 0.15)

7. Assign Risk Category:
   Score 0-39:    Low Risk
   Score 40-69:   Medium Risk
   Score 70-89:   High Risk
   Score 90-100:  Prohibited
```

#### Step 5: Approval Decision

**Decision Matrix:**

| Risk Category | Screening Result | Identity Verification | Decision |
|---------------|------------------|----------------------|----------|
| **Low** | Clear | Pass | Auto-approve |
| **Low** | PEP/Adverse | Pass | Manual review → Approve/Decline |
| **Medium** | Clear | Pass | Auto-approve or Manual review* |
| **Medium** | PEP/Adverse | Pass | Manual review → Approve/Decline |
| **High** | Clear | Pass | Senior approval required |
| **High** | PEP | Pass | Senior approval + EDD required |
| **High** | Sanctions match | Any | Decline |
| **Prohibited** | Any | Any | Auto-decline |
| Any | Any | Fail | Decline or request re-verification |

*Manual review threshold configurable by institution

**Approval Authorities:**

| Risk Category | Authority | Documentation Required |
|---------------|-----------|----------------------|
| **Low** | System auto-approval | Standard CDD checklist |
| **Medium** | Compliance analyst | CDD checklist + risk assessment |
| **High** | Compliance manager | EDD documentation + senior sign-off |

#### Step 6: Ongoing Monitoring

**Monitoring Requirements:**

| Risk Category | Transaction Monitoring | Periodic Review | Re-Screening |
|---------------|----------------------|----------------|--------------|
| **Low** | Standard rules | Every 2 years | Weekly (automated) |
| **Medium** | Standard rules | Annually | Daily (automated) |
| **High** | Enhanced rules + manual oversight | Quarterly | Daily (automated) |

---

## Enhanced Due Diligence (EDD) Protocol

### When EDD is Required

**Mandatory EDD Triggers:**
1. ✅ Customer is PEP (any tier)
2. ✅ High-risk country involvement
3. ✅ High-risk industry (MSB, crypto exchanges, casinos, etc.)
4. ✅ Complex ownership structure
5. ✅ High transaction volumes (>threshold defined by institution)
6. ✅ Adverse media findings (serious allegations)
7. ✅ Unusual or suspicious activity detected
8. ✅ Senior management or regulatory directive

### EDD Additional Requirements

**Beyond Standard CDD:**

```
1. Source of Wealth (not just source of funds)
   Required:
   - Detailed explanation of wealth accumulation
   - Supporting documentation (tax returns, business records, inheritance docs)
   - Verification of major wealth sources

2. Purpose of Account/Relationship
   Required:
   - Detailed business plan or account usage description
   - Rationale for specific products/services
   - Explanation for jurisdictions involved

3. Enhanced Identity Verification
   Required:
   - Video verification or in-person meeting
   - Additional document cross-checks
   - Independent verification of employment/business

4. Beneficial Ownership Deep Dive (for entities)
   Required:
   - Full ownership chain to natural persons
   - Investigation of nominee structures
   - Verification of each layer
   - Documentation of control mechanisms

5. Independent Information Sources
   Required:
   - Public records search
   - Internet/media search
   - Corporate registry verification
   - Third-party reference checks

6. Senior Management Approval
   Required:
   - Compliance manager review
   - Risk committee approval (for highest risk)
   - Documentation of approval rationale

7. Enhanced Monitoring
   Required:
   - Lower transaction monitoring thresholds
   - Manual review of large/unusual transactions
   - Quarterly account reviews
   - Daily automated screening updates
```

### EDD Documentation Checklist

**For PEP Customers:**
```
☐ PEP status confirmed and documented
☐ Tier classification assigned
☐ Source of wealth documented and verified
☐ Current position and responsibilities documented
☐ Purpose of relationship clearly articulated
☐ Senior approval obtained
☐ Enhanced monitoring rules configured
☐ Quarterly review schedule set
```

**For High-Risk Jurisdiction Customers:**
```
☐ Reason for jurisdiction involvement documented
☐ Additional identity verification completed
☐ Source of funds from high-risk country explained and verified
☐ Purpose of transactions to/from high-risk jurisdiction documented
☐ Compliance with sanctions verified
☐ Enhanced transaction monitoring enabled
```

**For High-Value/Complex Entities:**
```
☐ Complete ownership structure mapped
☐ All beneficial owners (>25%) identified and verified
☐ Control persons identified and verified
☐ Business model and revenue sources verified
☐ Major customers/suppliers identified
☐ Financial statements reviewed
☐ Tax compliance verified
☐ Licenses and registrations verified
```

---

## Ongoing Monitoring Protocol

### Transaction Monitoring Rules

**Structuring / Smurfing Detection:**

```
Rule: Multiple transactions just below reporting threshold

Configuration:
- Threshold: $10,000 (CTR threshold) or institution-defined
- Lookback period: 30 days
- Trigger: 3+ transactions within 10% of threshold

Alert Logic:
IF (SUM(deposits, 30_days) > threshold × 3) AND
   (EACH deposit BETWEEN threshold × 0.90 AND threshold × 0.99) AND
   (COUNT(deposits) >= 3)
THEN generate_alert(severity: HIGH)

Example:
Deposits: $9,500, $9,800, $9,900 over 30 days
Total: $29,200 (>$30,000)
Each: 90-99% of $10,000
Count: 3
Result: ALERT
```

**Rapid Funds Movement:**

```
Rule: Large deposit followed by quick withdrawal

Configuration:
- Minimum amount: $5,000 or configurable
- Time window: 24-48 hours
- Withdrawal threshold: 90% of deposit

Alert Logic:
IF (credit_amount > minimum) AND
   (debit_within_hours < time_window) AND
   (debit_amount > credit_amount × 0.90)
THEN generate_alert(severity: HIGH)

Example:
Credit: $50,000 at 10:00 AM Day 1
Debit: $48,000 at 2:00 PM Day 1
Time: 4 hours (<24)
Percentage: 96% (>90%)
Result: ALERT
```

**Geographic Red Flags:**

```
Rule: Transactions involving high-risk countries

Configuration:
- High-risk country list: FATF high-risk jurisdictions + institution-defined
- Minimum amount: $1,000 or configurable
- Risk multiplier for PEP customers

Alert Logic:
IF (transaction_country IN high_risk_list) AND
   (amount > minimum)
THEN generate_alert(severity: MEDIUM to HIGH based on country)

If customer is PEP:
   severity = HIGH

Example:
Wire to country X (on FATF high-risk list)
Amount: $5,000
Customer: Not PEP
Result: ALERT (severity: MEDIUM)
```

**Volume Anomaly Detection:**

```
Rule: Unusual transaction volumes compared to profile

Configuration:
- Baseline: Average monthly activity over past 6 months
- Threshold: 200% of baseline
- Minimum for alert: $10,000 absolute increase

Alert Logic:
IF (current_month_volume > baseline × 2.0) AND
   (current_month_volume - baseline > 10000)
THEN generate_alert(severity: MEDIUM)

Example:
Baseline: $15,000/month average
Current month: $35,000 (as of mid-month)
Projected: $70,000
Increase: $55,000 (>$10,000)
Multiple: 4.7x (>2x)
Result: ALERT
```

**Round-Tripping Detection:**

```
Rule: Circular movement of funds

Configuration:
- Lookback period: 90 days
- Minimum amount: $5,000
- Match tolerance: 95% of original amount

Alert Logic:
Identify transaction chains:
A → B → C → ... → A
WHERE (final_amount > original_amount × 0.95) AND
      (original_amount > minimum)
THEN generate_alert(severity: HIGH)

Example:
Account A sends $10,000 to Account B
Account B sends $9,800 to Account C
Account C sends $9,600 to Account A
Result: ALERT (round-trip detected)
```

### Periodic Review Protocol

**Low Risk Customers (Biennial Review):**

```
Review Checklist:
☐ Verify customer contact information current
☐ Re-screen against sanctions, PEP lists
☐ Review transaction activity for consistency with profile
☐ Update risk score
☐ Confirm no adverse media
☐ Document review completion

Action:
- If no changes: Continue with low-risk classification
- If risk indicators changed: Re-assess risk category
```

**Medium Risk Customers (Annual Review):**

```
Review Checklist:
☐ All items from low-risk review +
☐ Review source of funds/wealth (any changes?)
☐ Verify employment/business status
☐ Review transaction patterns in detail
☐ Assess unusual activity (even if below alert threshold)
☐ Update customer profile
☐ Re-assess risk score
☐ Document review and findings

Action:
- If profile unchanged and activity consistent: Continue medium-risk
- If risk increased: Elevate to high-risk, consider EDD
- If risk decreased: Consider downgrade to low-risk
```

**High Risk Customers (Quarterly Review):**

```
Review Checklist:
☐ All items from medium-risk review +
☐ Deep dive on large transactions (even if not alerted)
☐ Review all monitoring alerts (closed and open)
☐ Verify PEP status if applicable
☐ Check for new adverse media
☐ Review source of wealth documentation
☐ Assess continued business rationale
☐ Update EDD documentation
☐ Senior management review of relationship
☐ Decision: Continue, enhance monitoring, or exit

Action:
- Document decision with rationale
- If exiting: Follow de-risking protocol
```

---

## Suspicious Activity Reporting (SAR) Protocol

### When to File a SAR

**Mandatory Filing Triggers:**

1. **Transaction(s) above threshold involving known or suspected criminal activity**
   - US: Transactions >$5,000
   - Varies by jurisdiction

2. **Transactions designed to evade BSA requirements**
   - Structuring
   - False statements
   - Use of nominee accounts

3. **Transactions with no apparent lawful purpose**
   - No economic rationale
   - Unusual complexity
   - Customer cannot or will not explain

4. **Computer intrusion or theft**
   - Unauthorized access
   - Account takeover

5. **Patterns indicating money laundering**
   - Layering schemes
   - Integration of illicit funds

### SAR Filing Process

```
Step 1: Alert/Suspicious Activity Identified
    ↓
Step 2: Preliminary Investigation (24-48 hours)
    ↓
Step 3: Full Investigation (5-10 business days)
    ↓
Step 4: SAR Decision
    ├─ No SAR: Document decision, close case
    └─ File SAR: Proceed to Step 5
        ↓
Step 5: SAR Preparation
    ↓
Step 6: Supervisor Review
    ↓
Step 7: SAR Filing (within 30 days of detection)
    ↓
Step 8: Law Enforcement Notification (if urgent)
    ↓
Step 9: Continue Monitoring (no customer notification)
```

### SAR Narrative Requirements

**Must Include:**

1. **Who**: Subject(s) of SAR with complete identifying information
2. **What**: Description of suspicious activity
3. **When**: Dates and timeline of activity
4. **Where**: Locations involved
5. **Why**: Reason for suspicion (what makes it unusual/suspicious)
6. **How**: Method used to conduct activity

**Narrative Structure:**

```
Introduction:
- Brief summary of suspicious activity
- Subject identification
- Total amount involved

Background:
- Customer relationship history
- Normal account activity patterns
- When suspicious activity began

Suspicious Activity Details:
- Chronological description of transactions/behavior
- Specific red flags observed
- Deviations from expected activity

Investigation:
- Steps taken to investigate
- Customer contact (if any) and response
- External information gathered

Analysis:
- Why activity is considered suspicious
- Typologies or patterns matched
- Possible criminal activity suspected

Conclusion:
- Summary of basis for SAR filing
- Ongoing monitoring plans
```

### SAR Filing Timeline

```
Day 0: Suspicious activity detected
    ↓
Day 1-2: Preliminary review
    ↓
Day 3-10: Full investigation
    ↓
Day 11-15: SAR preparation and review
    ↓
Day 30: Filing deadline (maximum)
```

**Note**: Extend to 60 days if identity of subject unknown, but must document extension reason.

---

## Key Takeaways

1. 📋 **Standard CDD**: 6-step process for low-medium risk customers
2. 🔍 **EDD**: Additional requirements for high-risk scenarios
3. 🎯 **Risk-based approach**: Tailor due diligence to actual risk
4. 📊 **Transaction monitoring**: Multiple rule typologies for detection
5. 🔄 **Periodic reviews**: Frequency based on risk category
6. 🚨 **SAR protocols**: Clear triggers and filing procedures
7. ✅ **Documentation**: Comprehensive audit trail required

---

**Previous**: [← Chapter 5 - API Interface](05-api-interface.md) | **Next**: [Chapter 7 - System Integration →](07-system-integration.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
