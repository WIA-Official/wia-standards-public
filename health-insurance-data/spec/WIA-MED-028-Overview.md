# WIA-MED-028: Health Insurance Data Standards

**Version:** 1.0.0
**Status:** Official Standard
**Published:** 2025
**License:** MIT

---

## Executive Summary

WIA-MED-028 defines international standards for health insurance data exchange, covering claims processing, EDI X12 standards, premium calculation, eligibility verification, prior authorization, explanation of benefits (EOB), fraud detection, and cross-payer interoperability.

## Scope

This standard applies to:
- Health insurance payers (commercial, government, self-insured)
- Healthcare providers (hospitals, physicians, clinics)
- Clearinghouses and intermediaries
- Health information exchanges (HIEs)
- Medical billing software vendors
- Pharmacy benefit managers (PBMs)

## Core Principles

### 1. **Interoperability**
Seamless data exchange across all stakeholders in the healthcare ecosystem

### 2. **Transparency**
Clear, understandable cost and benefit information for patients

### 3. **Efficiency**
Automated workflows to reduce administrative burden and costs

### 4. **Security**
HIPAA-compliant data protection and privacy preservation

### 5. **Global Compatibility**
International code mapping and currency handling

### 6. **Patient-Centric**
Empowering patients with access to their health insurance data

## Key Components

### 1. Claims Data Formats

#### CMS-1500 (Professional Claims)
- **Purpose**: Physician and outpatient services
- **Key Fields**:
  - Patient demographics (1-13)
  - Diagnosis codes ICD-10-CM (21)
  - Service lines with CPT codes (24)
  - Provider information (25-33)

#### UB-04 (Institutional Claims)
- **Purpose**: Hospital and facility services
- **Key Fields**:
  - Claim type and dates (4-6)
  - Revenue codes (42)
  - Diagnosis codes with POA (67-72)
  - Procedure codes ICD-10-PCS (74)

#### 837 Electronic Transactions
- **837P**: Professional claims (CMS-1500 electronic)
- **837I**: Institutional claims (UB-04 electronic)
- **837D**: Dental claims

### 2. EDI X12 Standards

#### Transaction Sets

| Transaction | Name | Purpose |
|-------------|------|---------|
| **837** | Health Care Claim | Submit claims for payment |
| **835** | Payment/Remittance | Explain claim payments |
| **270** | Eligibility Inquiry | Check insurance coverage |
| **271** | Eligibility Response | Return coverage details |
| **276** | Claim Status Inquiry | Check claim processing status |
| **277** | Claim Status Response | Return claim status |
| **278** | Authorization | Request/respond prior auth |
| **834** | Enrollment | Member enrollment/changes |
| **820** | Premium Payment | Premium payment information |

#### Segment Structure
```
ISA (Interchange Header)
  GS (Functional Group)
    ST (Transaction Set)
      BHT (Beginning of Transaction)
      Loop structures (2000A, 2000B, 2300, 2400)
    SE (Transaction End)
  GE (Group End)
IEA (Interchange End)
```

### 3. Premium Calculation

#### Actuarial Formula
```
Premium = Expected Claims + Administrative Costs + Profit Margin + Risk Margin
       = (Frequency × Severity) / (1 - Loading Factor)
```

#### Risk Adjustment Models
- **CMS-HCC**: Medicare Advantage (87 categories)
- **HHS-HCC**: ACA Marketplace (127 categories)
- **RxHCC**: Medicare Part D (prescription drugs)
- **DxCG**: Commercial insurance

#### Rating Factors
- Age (max 3:1 ratio)
- Geography (regional cost variations)
- Tobacco use (max 1.5:1 ratio)
- Family composition

### 4. Eligibility Verification

#### Real-time 270/271 Exchange
- **Response Time**: < 5 seconds
- **Information Returned**:
  - Active coverage status
  - Coverage dates
  - Copay, coinsurance, deductible
  - Out-of-pocket maximum
  - Prior authorization requirements

#### Service Type Codes
- `30`: Health Benefit Plan Coverage
- `98`: Professional (Physician)
- `AL`: Vision (Optometry)
- `F6`: Maternity
- `35`: Dental Care

### 5. Prior Authorization

#### Medical Necessity Criteria
- **Appropriate**: Service matches diagnosis
- **Evidence-based**: Clinical guidelines support
- **Not Experimental**: FDA-approved or standard of care
- **Cost-effective**: No equivalent cheaper alternative

#### Decision Codes (278 Response)
- **A1**: Certified (approved)
- **A2**: Modified (conditional approval)
- **A3**: Denied
- **A4**: Pended (additional information needed)

### 6. Explanation of Benefits (EOB)

#### Standard Components
1. **Header**: Payer, member ID, claim number, date
2. **Provider**: Name, service date, location
3. **Services**: Description, billed amount, allowed amount
4. **Adjustments**: Discount, deductible, copay, coinsurance
5. **Summary**: Total billed, plan paid, patient owes

#### 835 Adjustment Codes (CARC)
- `1`: Deductible amount
- `2`: Coinsurance amount
- `3`: Copay amount
- `45`: Charge exceeds fee schedule
- `50`: Non-covered service
- `97`: Benefit maximum reached

### 7. Fraud Detection

#### Common Fraud Patterns
1. **Phantom Billing**: Claiming services not provided
2. **Upcoding**: Using higher-level codes than justified
3. **Unbundling**: Separating bundled services
4. **Kickbacks**: Referrals for financial gain

#### AI Detection Techniques
- **Anomaly Detection**: Statistical outliers (Z-score, Isolation Forest)
- **Supervised Learning**: Trained on historical fraud (Random Forest, Neural Networks)
- **Network Analysis**: Provider-patient relationship graphs
- **Real-time Scoring**: Prepayment risk scores (Green/Yellow/Red)

#### Key Metrics
- **ROI**: Recovery / Investigation Cost (target 10:1)
- **Detection Rate**: Fraud Found / Total Claims (5-10%)
- **False Positive Rate**: < 20%

### 8. Cross-Payer Data Exchange

#### Coordination of Benefits (COB)
- **Primary/Secondary Determination**: Birthday Rule, employment status
- **Payment Coordination**: Primary pays first, secondary covers remaining
- **Data Exchange**: 837 with Loop 2320 (Other Payer)

#### Health Information Exchange (HIE)
- **Models**: Centralized, Federated, Hybrid
- **Standards**: HL7 FHIR, IHE XDS
- **Security**: OAuth 2.0, TLS 1.3, ATNA audit logs

#### International Claims
- **Code Mapping**: ICD-10-CM ↔ ICD-10-WHO ↔ KCD-8
- **Currency**: Real-time exchange rates with PPP adjustment
- **Workflow**: Local payment → Reimbursement → Translation → Processing

## Implementation Requirements

### Mandatory Features
- ✅ EDI X12 support (837, 835, 270/271, 278)
- ✅ Real-time eligibility verification (< 5 sec response)
- ✅ Automated claim validation (CCI edits)
- ✅ Secure data transmission (TLS 1.3)
- ✅ HIPAA compliance
- ✅ Audit logging

### Recommended Features
- ⭐ AI-powered fraud detection
- ⭐ Patient-friendly EOB design
- ⭐ Mobile app integration
- ⭐ Multi-language support
- ⭐ HL7 FHIR compatibility

## Certification

Organizations implementing WIA-MED-028 can obtain certification at:
**https://cert.wiastandards.com**

### Certification Levels
- **Bronze**: Basic EDI compliance
- **Silver**: Automated workflows + real-time verification
- **Gold**: AI fraud detection + FHIR integration
- **Platinum**: International interoperability

## Compliance Timeline

- **Phase 1 (2025 Q2)**: EDI X12 mandatory
- **Phase 2 (2025 Q4)**: Real-time eligibility required
- **Phase 3 (2026 Q2)**: Fraud detection systems
- **Phase 4 (2026 Q4)**: Full international compatibility

## References

- HIPAA EDI Standards: [CMS.gov](https://www.cms.gov/regulations-and-guidance/administrative-simplification/hipaa-edi)
- HL7 FHIR: [fhir.org](https://www.hl7.org/fhir/)
- ICD-10-CM: [CDC.gov](https://www.cdc.gov/nchs/icd/icd-10-cm.htm)
- CPT Codes: [AMA-assn.org](https://www.ama-assn.org/practice-management/cpt)

---

**Contact**: standards@wiastandards.com
**Website**: https://wiastandards.com
**GitHub**: https://github.com/WIA-Official/wia-standards

© 2025 WIA (World Certification Industry Association) · MIT License
**弘益人間 · Benefit All Humanity**
