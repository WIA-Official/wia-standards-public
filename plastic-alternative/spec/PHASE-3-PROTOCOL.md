# WIA-ENE-048: Plastic Alternative - Phase 3 Protocol

## Overview

**Standard ID:** WIA-ENE-048
**Category:** Energy & Environment (ENE)
**Phase:** 3 - Protocol
**Version:** 1.0

This document defines the operational protocols for certification, testing, verification, and lifecycle management of plastic alternative materials.

## Certification Protocol

### 1. Application Process

#### Step 1: Pre-Application Assessment
**Duration:** 1-2 weeks

```
Actions:
1. Review WIA-ENE-048 requirements
2. Conduct internal material assessment
3. Identify target certification level
4. Prepare preliminary documentation
```

**Required Documents:**
- Material composition disclosure
- Manufacturing process description
- Preliminary test data (if available)
- Company registration documents

#### Step 2: Formal Application Submission
**Duration:** 1 week

```
POST /api/v1/certifications/apply
{
  "applicant": {
    "companyName": "string",
    "did": "string",
    "contact": {...}
  },
  "material": {
    "name": "string",
    "type": "enum",
    "composition": {...}
  },
  "targetLevel": "enum [basic, advanced, premium, marine]",
  "previousCertifications": []
}
```

**Application Fee:**
- Basic: $2,500 USD
- Advanced: $5,000 USD
- Premium: $10,000 USD
- Marine: $7,500 USD

#### Step 3: Document Review
**Duration:** 2-3 weeks

WIA Certification Authority reviews:
- Material composition completeness
- Manufacturing process sustainability
- Preliminary data quality
- Regulatory compliance

**Outcomes:**
- ✅ Approved → Proceed to testing
- ⚠️ Revisions Required → Resubmit within 30 days
- ❌ Rejected → Provide detailed feedback

#### Step 4: Testing Assignment
**Duration:** 1 week

```
Certification Authority:
1. Assigns approved testing laboratory
2. Provides testing protocol
3. Coordinates sample delivery
4. Sets timeline expectations
```

**Sample Requirements:**
- Quantity: 5-10 kg (depending on tests)
- Packaging: As-produced form
- Labeling: Batch number, production date
- Traceability: Full supply chain documentation

### 2. Testing Protocol

#### Mandatory Tests (All Levels)

**Test 1: Bio-based Content Analysis**
- **Method:** ASTM D6866 (Radiocarbon dating)
- **Duration:** 2-3 weeks
- **Sample size:** 100 grams
- **Pass criteria:** Meets claimed bio-based percentage ± 5%

```
Process:
1. Sample preparation and purification
2. Radiocarbon measurement
3. Percentage calculation
4. Uncertainty analysis
5. Report generation
```

**Test 2: Biodegradation Testing**
- **Method:** ISO 14855-1 or ASTM D5338
- **Duration:** 180 days minimum
- **Sample size:** 500 grams
- **Pass criteria:** ≥90% conversion to CO2

```
Protocol:
1. Sample preparation (powder or film, <2mm)
2. Inoculum preparation (compost)
3. Bioreactor setup (controlled temp, moisture)
4. CO2 monitoring (continuous)
5. Data analysis (% biodegradation)
6. Statistical validation
```

**Test 3: Ecotoxicity Assessment**
- **Method:** ISO 20200 (Plant and earthworm)
- **Duration:** 28 days
- **Pass criteria:** ≥90% germination, no harmful effects

```
Plant Test (Lepidium sativum):
1. Material composting completion
2. Soil mixture preparation
3. Seed planting
4. Growth monitoring (21 days)
5. Germination and biomass measurement

Earthworm Test (Eisenia fetida):
1. Compost material preparation
2. Earthworm introduction
3. Survival monitoring (28 days)
4. Reproduction assessment
5. Behavioral observations
```

**Test 4: Heavy Metal Analysis**
- **Method:** EN 13432, ICP-MS
- **Duration:** 1 week
- **Pass criteria:** Total < 100 ppm

```
Elements Tested:
- Lead (Pb): < 50 ppm
- Cadmium (Cd): < 0.5 ppm
- Mercury (Hg): < 0.5 ppm
- Hexavalent Chromium (Cr VI): < 50 ppm
- Additional: As, Ni, Se, Mo
```

#### Additional Tests (Advanced/Premium)

**Test 5: Compostability Certification**
- **Method:** ASTM D6400 or EN 13432
- **Duration:** 180 days
- **Includes:**
  - Disintegration test (90 days)
  - Full biodegradation confirmation
  - Quality of resulting compost
  - No visible residue requirement

**Test 6: Mechanical Properties**
- **Methods:** ISO 527 (tensile), ISO 178 (flexural), ISO 179 (impact)
- **Duration:** 1-2 weeks
- **Purpose:** Performance benchmarking

**Test 7: Thermal Analysis**
- **Methods:** DSC, TGA, HDT
- **Duration:** 1 week
- **Purpose:** Processing and application guidance

#### Marine-Safe Additional Tests

**Test 8: Marine Biodegradation**
- **Method:** ASTM D7081
- **Duration:** 365 days
- **Conditions:** Seawater at 30°C
- **Pass criteria:** ≥70% degradation

```
Protocol:
1. Seawater collection and preparation
2. Sample preparation (film specimens)
3. Bioreactor setup (marine conditions)
4. Monthly monitoring (weight loss, CO2)
5. Microplastic analysis
6. Long-term degradation tracking
```

**Test 9: Marine Ecotoxicity**
- **Species:** Algae (Chlorella vulgaris), Crustaceans (Daphnia magna), Fish (Danio rerio)
- **Duration:** 4-8 weeks
- **Pass criteria:** No toxic effects (LC50, EC50)

### 3. Evaluation & Certification Decision

#### Data Analysis (2-3 weeks)

```
Certification Committee Reviews:
1. All test reports from laboratories
2. Compliance with pass criteria
3. Statistical significance of results
4. Any anomalies or concerns
5. Applicant's responses to findings
```

**Decision Matrix:**

| Criterion | Basic | Advanced | Premium | Marine |
|-----------|-------|----------|---------|---------|
| Bio-based content | ≥50% | ≥80% | ≥95% | ≥80% |
| Biodegradation (180d) | ≥90% | ≥90% | ≥90% | - |
| Marine biodegradation (365d) | - | - | - | ≥70% |
| Compostability cert | No | Yes | Yes | Yes |
| Home compostable | No | No | Yes | No |
| Ecotoxicity | Pass | Pass | Pass | Pass (marine) |
| Heavy metals | <100ppm | <100ppm | <100ppm | <100ppm |
| Mechanical testing | Basic | Full | Full | Full |
| Carbon footprint | - | - | <2.0 kg CO2/kg | - |

#### Certification Outcomes

**✅ Certified:**
```json
{
  "status": "certified",
  "certificateId": "CERT-ENE048-XXXXX",
  "level": "advanced",
  "issueDate": "2025-01-15",
  "expiryDate": "2026-01-15",
  "conditions": [],
  "nextAudit": "2025-07-15"
}
```

**⚠️ Conditional Approval:**
```
Minor deficiencies identified
- Must address within 60 days
- Re-testing of specific parameters
- Temporary certificate issued
```

**❌ Not Certified:**
```
Significant failures
- Detailed feedback provided
- Opportunity to reformulate
- Can reapply after 6 months
```

### 4. Certificate Issuance

#### Digital Certificate Generation

```
Certificate Components:
1. Unique Certificate ID (CERT-ENE048-XXXXX)
2. Material identification
3. Certification level
4. Validity period
5. Scope of certification
6. QR code for verification
7. Digital signatures
8. Blockchain anchor
```

#### Verifiable Credential Creation

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/credentials/plastic-alternative/v1"
  ],
  "id": "https://wia.org/credentials/ENE048/XXXXX",
  "type": ["VerifiableCredential", "PlasticAlternativeCertificate"],
  "issuer": {
    "id": "did:wia:certifier:001",
    "name": "WIA Certification Authority"
  },
  "issuanceDate": "2025-01-15T00:00:00Z",
  "expirationDate": "2026-01-15T00:00:00Z",
  "credentialSubject": {
    "id": "did:wia:material:PLA-2025-001",
    "materialType": "PLA",
    "certificationLevel": "WIA-ENE-048-ADVANCED",
    "properties": {...},
    "testResults": {...}
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T00:00:00Z",
    "verificationMethod": "did:wia:certifier:001#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z5a3..."
  }
}
```

#### Blockchain Anchoring

```
Process:
1. Generate certificate hash (SHA-256)
2. Create blockchain transaction
3. Record on Ethereum mainnet
4. Store transaction hash in certificate
5. Enable public verification
```

```solidity
// Smart Contract Function
function anchorCertificate(
    string memory certificateId,
    bytes32 certificateHash,
    string memory materialId
) public onlyAuthorizedCertifier returns (uint256) {
    // Record certificate on blockchain
    // Emit event for indexing
    // Return block number
}
```

## Surveillance & Monitoring Protocol

### Annual Surveillance Audit

**Required for all certified materials**

```
Schedule:
- Month 6: Mid-term check-in
- Month 12: Full renewal assessment
```

**Audit Components:**

1. **Document Review**
   - Production records
   - Quality control data
   - Customer complaints
   - Any formulation changes

2. **Sample Testing**
   - Random batch selection
   - Repeat key tests (bio-based, biodegradation)
   - Verify consistency with certified data

3. **Facility Inspection** (for Premium level)
   - Manufacturing process verification
   - Quality management system review
   - Supply chain audit

**Outcomes:**
- ✅ Renewal approved
- ⚠️ Conditional renewal (address issues within 90 days)
- ❌ Suspension (major non-compliance)
- ❌ Revocation (fraud or safety issues)

### Market Surveillance

```
WIA Authority conducts:
1. Random market sampling (2% of certified products)
2. Consumer complaint investigation
3. Competitor challenge review
4. Third-party audits
```

### Complaint Protocol

```
POST /api/v1/complaints/submit
{
  "complainant": {...},
  "certificateId": "CERT-ENE048-XXXXX",
  "complaintType": "enum [performance, safety, fraud, other]",
  "description": "string",
  "evidence": ["URLs or file attachments"]
}
```

**Investigation Process:**
1. Initial review (5 business days)
2. Notification to certificate holder
3. Evidence collection
4. Testing if required
5. Committee decision (30 days)
6. Outcome communication

## Verification Protocol

### Public Verification

**Anyone can verify a certificate:**

```
GET /api/v1/certifications/verify
Query: certificateId=CERT-ENE048-XXXXX

Response:
{
  "valid": true,
  "status": "active",
  "material": {...},
  "issueDate": "2025-01-15",
  "expiryDate": "2026-01-15",
  "blockchainVerified": true,
  "txHash": "0x1a2b3c..."
}
```

**QR Code Verification:**
```
1. Scan QR code on product
2. Redirect to verification URL
3. Display certificate details
4. Show blockchain proof
5. Provide download option
```

### Blockchain Verification

```javascript
// Verify on blockchain
async function verifyOnBlockchain(certificateId) {
  const contract = await getContract('WIA_ENE_048_Registry');
  const record = await contract.getCertificate(certificateId);

  return {
    exists: record.exists,
    hash: record.certificateHash,
    timestamp: record.timestamp,
    issuer: record.issuerDID,
    status: record.status
  };
}
```

## Appeals Protocol

### Appeal Submission

```
Applicant can appeal within 30 days of decision

POST /api/v1/certifications/appeal
{
  "certificateId": "string",
  "applicant": {...},
  "appealType": "enum [test_results, evaluation, procedure]",
  "grounds": "string (detailed justification)",
  "supportingEvidence": [...]
}
```

### Appeal Review Process

**Step 1: Administrative Review (1 week)**
- Verify appeal is within scope
- Check submission completeness
- Assign independent reviewer

**Step 2: Technical Review (2-4 weeks)**
- Independent expert panel (3 members)
- Review original test data
- Consider new evidence
- Optional: Independent re-testing

**Step 3: Appeal Decision (1 week)**
- Panel recommendation
- WIA Authority final decision
- Written justification

**Possible Outcomes:**
1. Appeal upheld → Certificate issued/restored
2. Partial upheld → Conditional certification
3. Appeal denied → Original decision stands
4. Remand → Send back for re-evaluation

## Revocation Protocol

### Grounds for Revocation

1. **Fraud or Misrepresentation**
   - False test data
   - Undisclosed material changes
   - Fake certifications

2. **Safety Issues**
   - Toxicity discovered
   - Consumer harm
   - Environmental damage

3. **Non-compliance**
   - Failed surveillance audit
   - Repeated violations
   - Refusal to cooperate

### Revocation Process

```
1. Investigation initiation
2. Notification to certificate holder (10 days to respond)
3. Hearing opportunity
4. Committee decision
5. Immediate suspension if safety concern
6. Final revocation decision
7. Public notification
8. Database update
9. Blockchain status update
```

**Post-Revocation:**
- Certificate holder must cease using WIA-ENE-048 mark
- Notify customers within 30 days
- Remove products from market (if safety issue)
- Can reapply after 1 year (non-fraud cases)

## Laboratory Accreditation Protocol

### Becoming an Approved Testing Laboratory

**Requirements:**
1. ISO/IEC 17025 accreditation
2. Specific test method competency
3. Equipment and facility adequacy
4. Proficiency testing participation
5. No conflicts of interest

**Application Process:**
```
POST /api/v1/laboratories/apply
{
  "labName": "string",
  "accreditation": {...},
  "capabilities": ["ISO 14855", "ASTM D6866", ...],
  "equipment": [...],
  "qualityManual": "URL"
}
```

**Approval Process:**
1. Document review (2 weeks)
2. Facility audit (1 day)
3. Proficiency test (blind sample)
4. Committee evaluation
5. Approval decision (4-6 weeks total)

**Ongoing Requirements:**
- Annual proficiency testing
- Biannual audits
- Timely reporting
- Quality incidents disclosure

---

## Protocol Version Control

**Version:** 1.0
**Effective Date:** 2025-01-15
**Review Cycle:** Annual
**Next Review:** 2026-01-15

## Protocol Updates

All protocol changes:
1. Announced 90 days in advance
2. Stakeholder consultation period
3. Grandfather clause for existing certificates
4. Training provided to laboratories
5. Updated documentation published

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
