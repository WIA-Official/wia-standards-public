# WIA-SEC-024: GDPR Compliance - Phase 1 Core Specification

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-12-25
**Standard ID:** WIA-SEC-024
**Category:** Security (SEC)
**Color:** #8B5CF6

---

## 1. Introduction

### 1.1 Purpose

WIA-SEC-024 defines a comprehensive framework for GDPR (General Data Protection Regulation) compliance, enabling organizations to:

- **Protect Personal Data**: Implement technical and organizational measures for data protection
- **Respect Data Subject Rights**: Fulfill all GDPR-mandated rights efficiently and transparently
- **Maintain Compliance**: Demonstrate ongoing compliance through documentation and audit trails
- **Enable Transparency**: Provide clear information about data processing activities
- **Manage Consent**: Track and manage consent across all processing purposes

### 1.2 Scope

This Phase 1 specification covers:

- Core GDPR concepts and principles
- Data subject rights implementation
- Consent management frameworks
- Lawful basis for processing
- Data protection impact assessments (DPIA)
- Breach notification procedures
- Data processing agreements
- Records of processing activities

Out of scope for Phase 1 (covered in later phases):
- Advanced cross-border transfer mechanisms
- Machine learning-based compliance automation
- Blockchain-based consent ledgers
- AI-powered data discovery

### 1.3 Philosophy

弘益人間 (Hongik Ingan) - Benefit All Humanity

Data protection is a fundamental human right enshrined in the EU Charter of Fundamental Rights. Our GDPR compliance framework ensures:

- **Privacy by Design**: Privacy embedded in systems from inception
- **Privacy by Default**: Strictest privacy settings applied automatically
- **Transparency**: Clear communication about data processing
- **Accountability**: Demonstrable compliance with regulatory requirements
- **Empowerment**: Individuals control their personal data

---

## 2. Core Principles

### 2.1 GDPR Principles (Article 5)

#### 2.1.1 Lawfulness, Fairness, and Transparency

Personal data must be processed lawfully, fairly, and in a transparent manner.

**Implementation Requirements:**
- Document lawful basis for each processing activity
- Provide clear, accessible privacy notices
- Maintain processing records (Article 30)
- Enable data subjects to understand processing

**Example Privacy Notice:**
```markdown
# Privacy Notice

**Controller**: Example Corp Ltd
**DPO Contact**: dpo@example.com

## What data we collect
- Name, email address, phone number
- Account information and preferences
- Usage data and analytics

## Why we collect it (Legal Basis)
- Contract performance (Art. 6(1)(b))
- Legitimate interests (Art. 6(1)(f))
- Consent (Art. 6(1)(a)) for marketing

## How long we keep it
- Account data: Duration of relationship + 6 years
- Marketing data: Until consent withdrawn
- Analytics: 26 months

## Your rights
You have the right to access, rectify, erase, restrict,
port your data, and object to processing.

Contact: privacy@example.com
```

#### 2.1.2 Purpose Limitation

Data must be collected for specified, explicit, and legitimate purposes.

**Implementation:**
```json
{
  "processingActivity": "customer_relationship_management",
  "purposes": [
    {
      "id": "purpose_001",
      "name": "Contract Performance",
      "description": "Manage customer accounts and fulfill orders",
      "legalBasis": "contract",
      "retentionPeriod": "P6Y",
      "dataCategories": ["contact_info", "order_history"]
    },
    {
      "id": "purpose_002",
      "name": "Marketing Communications",
      "description": "Send promotional emails and newsletters",
      "legalBasis": "consent",
      "retentionPeriod": "until_withdrawal",
      "dataCategories": ["email", "preferences"]
    }
  ]
}
```

#### 2.1.3 Data Minimization

Only data adequate, relevant, and necessary for the purpose should be collected.

**Minimization Checklist:**
- [ ] Each data field has documented necessity
- [ ] No "nice to have" data collected
- [ ] Default forms collect minimum data
- [ ] Optional fields clearly marked
- [ ] Regular review of data requirements

#### 2.1.4 Accuracy

Personal data must be accurate and kept up to date.

**Accuracy Controls:**
```json
{
  "dataAccuracyPolicy": {
    "verificationRequired": true,
    "verificationMethods": ["email_confirmation", "phone_otp"],
    "updateFrequency": "P12M",
    "rectificationSLA": "P5D",
    "qualityMetrics": {
      "emailBounceRate": {"threshold": 5, "action": "flag_for_review"},
      "phoneInvalidRate": {"threshold": 3, "action": "request_update"}
    }
  }
}
```

#### 2.1.5 Storage Limitation

Data must not be kept longer than necessary.

**Retention Schedule:**
```json
{
  "retentionSchedule": [
    {
      "dataCategory": "customer_data",
      "purpose": "contract_performance",
      "retentionPeriod": "P6Y",
      "triggerEvent": "contract_end",
      "deletionMethod": "secure_erasure"
    },
    {
      "dataCategory": "marketing_consent",
      "purpose": "marketing",
      "retentionPeriod": "until_withdrawn",
      "reviewFrequency": "P24M",
      "deletionMethod": "anonymization"
    }
  ]
}
```

#### 2.1.6 Integrity and Confidentiality

Data must be processed securely with appropriate technical and organizational measures.

**Security Measures:**
- Encryption at rest (AES-256) and in transit (TLS 1.3+)
- Access controls with role-based permissions
- Audit logging of all data access
- Regular security assessments
- Incident response procedures
- Staff training and awareness

#### 2.1.7 Accountability

Controllers must demonstrate compliance with GDPR principles.

**Accountability Framework:**
- Data Protection Impact Assessments (DPIAs)
- Records of Processing Activities (RoPA)
- Data Processing Agreements (DPAs)
- Training records
- Audit trails
- Compliance monitoring

---

## 3. Lawful Basis for Processing

### 3.1 Legal Bases (Article 6)

#### 3.1.1 Consent (Article 6(1)(a))

**Requirements:**
- Freely given
- Specific
- Informed
- Unambiguous indication of wishes
- Clear affirmative action

**Consent Record Format:**
```json
{
  "consentId": "consent_abc123",
  "dataSubjectId": "user_12345",
  "timestamp": "2025-12-25T10:30:00Z",
  "version": "2.1",
  "purposes": ["marketing_email", "analytics"],
  "consentText": "I agree to receive marketing emails and allow analytics tracking",
  "mechanism": "checkbox",
  "proof": {
    "ipAddress": "192.168.1.100",
    "userAgent": "Mozilla/5.0...",
    "timestamp": "2025-12-25T10:30:00Z"
  },
  "withdrawalUrl": "https://example.com/consent/withdraw/abc123",
  "status": "active"
}
```

#### 3.1.2 Contract (Article 6(1)(b))

Processing necessary for contract performance or pre-contractual measures.

**Example:**
```json
{
  "legalBasis": "contract",
  "contractType": "service_agreement",
  "necessityJustification": "Email address required to send order confirmations and account recovery",
  "dataProcessed": ["email", "name", "shipping_address"],
  "alternativesConsidered": "Anonymous pickup rejected as impractical for online orders"
}
```

#### 3.1.3 Legal Obligation (Article 6(1)(c))

Processing required to comply with legal obligations.

**Example:** Tax records retention for 7 years

#### 3.1.4 Vital Interests (Article 6(1)(d))

Processing necessary to protect life or physical integrity.

**Example:** Emergency medical data sharing

#### 3.1.5 Public Task (Article 6(1)(e))

Processing for tasks in public interest or official authority.

**Example:** Government services

#### 3.1.6 Legitimate Interests (Article 6(1)(f))

Processing necessary for legitimate interests (not available to public authorities).

**Legitimate Interest Assessment (LIA):**
```json
{
  "purpose": "fraud_prevention",
  "legitimateInterest": "Protect business and customers from fraudulent transactions",
  "necessity": "Automated fraud detection reduces financial losses by 87%",
  "balancingTest": {
    "organizationInterests": "Prevent €2M annual fraud losses",
    "dataSubjectInterests": "Minimal privacy impact using pseudonymized data",
    "safeguards": [
      "Data minimization",
      "Pseudonymization",
      "Access restrictions",
      "Regular audits"
    ],
    "conclusion": "Legitimate interest justified with appropriate safeguards"
  }
}
```

---

## 4. Data Subject Rights

### 4.1 Right of Access (Article 15)

Data subjects can request:
- Confirmation of processing
- Copy of personal data
- Information about processing

**Response Format:**
```json
{
  "requestType": "access",
  "requestId": "req_789",
  "dataSubjectId": "user_12345",
  "receivedDate": "2025-12-01T09:00:00Z",
  "deadline": "2025-12-31T23:59:59Z",
  "response": {
    "personalData": {
      "profile": {
        "name": "John Doe",
        "email": "john@example.com",
        "phone": "+44 20 1234 5678"
      },
      "orders": [...],
      "preferences": {...}
    },
    "processingInfo": {
      "purposes": ["contract_performance", "marketing"],
      "legalBases": ["contract", "consent"],
      "recipients": ["Payment Processor Ltd", "Email Service Inc"],
      "retentionPeriod": "6 years post-contract",
      "rights": "You may request rectification, erasure, restriction, or portability"
    }
  }
}
```

### 4.2 Right to Rectification (Article 16)

Correct inaccurate or incomplete personal data.

**SLA:** 5 business days (extendable to 30 days with notification)

### 4.3 Right to Erasure (Article 17)

"Right to be forgotten" when:
- Data no longer necessary
- Consent withdrawn
- Objection to processing
- Unlawful processing
- Legal obligation to erase

**Erasure Workflow:**
```json
{
  "erasureRequest": {
    "requestId": "req_rtbf_456",
    "dataSubjectId": "user_12345",
    "grounds": "consent_withdrawn",
    "scope": "all_data",
    "exceptions": [
      {
        "dataType": "financial_records",
        "reason": "legal_obligation_tax",
        "retentionUntil": "2031-12-31"
      }
    ],
    "deletionSteps": [
      "Anonymize marketing database",
      "Delete user profile",
      "Purge session logs",
      "Remove from backups (next cycle)"
    ],
    "completionDate": "2025-12-10T14:30:00Z",
    "confirmationSent": true
  }
}
```

### 4.4 Right to Data Portability (Article 20)

Receive personal data in structured, commonly used, machine-readable format.

**Export Format:**
```json
{
  "export": {
    "format": "JSON",
    "standard": "WIA-SEC-024",
    "timestamp": "2025-12-25T10:00:00Z",
    "dataSubject": "user_12345",
    "data": {
      "profile": {...},
      "orders": [...],
      "preferences": {...}
    },
    "metadata": {
      "exportedFields": 47,
      "recordCount": 234,
      "dataRange": "2020-01-01 to 2025-12-25"
    }
  }
}
```

### 4.5 Right to Restriction (Article 18)

Restrict processing while:
- Accuracy is verified
- Objection is assessed
- Data needed for legal claims

### 4.6 Right to Object (Article 21)

Object to:
- Processing based on legitimate interests
- Direct marketing (absolute right)
- Profiling

---

## 5. Consent Management

### 5.1 Valid Consent Requirements

**Consent Validity Checklist:**
```json
{
  "consentValidation": {
    "freelyGiven": {
      "check": "No conditional service access",
      "valid": true
    },
    "specific": {
      "check": "Separate consent for each purpose",
      "valid": true
    },
    "informed": {
      "check": "Clear information provided",
      "elements": ["identity", "purposes", "data_types", "rights"]
    },
    "unambiguous": {
      "check": "Clear affirmative action required",
      "mechanism": "checkbox",
      "preTickedBoxes": false
    },
    "withdrawable": {
      "check": "Easy withdrawal mechanism",
      "url": "https://example.com/consent/withdraw"
    },
    "documented": {
      "check": "Proof of consent recorded",
      "proofElements": ["timestamp", "ip", "user_agent", "consent_text"]
    }
  }
}
```

### 5.2 Consent Withdrawal

**Withdrawal Process:**
```json
{
  "withdrawal": {
    "consentId": "consent_abc123",
    "withdrawalDate": "2025-12-25T15:00:00Z",
    "withdrawalMethod": "one_click_link",
    "processingActions": [
      "Stop marketing emails immediately",
      "Anonymize preference data within 24 hours",
      "Update consent status in CRM",
      "Notify marketing automation platform"
    ],
    "confirmationSent": true,
    "effectiveDate": "2025-12-25T15:00:01Z"
  }
}
```

---

## 6. Data Protection Impact Assessment (DPIA)

### 6.1 When DPIA Required (Article 35)

Required when processing likely results in high risk, especially:
- Systematic monitoring of public areas
- Large-scale processing of special categories
- Automated decision-making with legal effects

### 6.2 DPIA Template

```markdown
# Data Protection Impact Assessment

## Project Details
- **Project Name**: Customer Behavioral Analytics
- **Date**: 2025-12-25
- **Assessor**: Jane Smith, DPO

## Description
Implement ML-based behavioral analytics to personalize product recommendations.

## Necessity and Proportionality
- **Purpose**: Improve user experience and conversion rates
- **Legal Basis**: Legitimate interest with consent for profiling
- **Necessity**: A/B testing shows 23% increase in satisfaction
- **Proportionality**: Use only behavioral data, no special categories

## Risks to Rights and Freedoms
| Risk | Likelihood | Severity | Impact |
|------|------------|----------|--------|
| Discriminatory recommendations | Medium | Medium | Medium |
| Data breach exposing preferences | Low | High | Medium |
| Lack of transparency | Medium | Low | Low |

## Mitigation Measures
1. **Bias Testing**: Regular algorithm audits for discriminatory patterns
2. **Encryption**: AES-256 for preference data at rest
3. **Transparency**: Clear explanations of recommendation logic
4. **Opt-out**: Easy mechanism to disable personalization

## Stakeholder Consultation
- Data subjects: Survey of 500 users, 78% support with transparency
- DPO: Approved with recommended safeguards
- IT Security: Confirmed encryption standards

## Conclusion
Processing may proceed with implemented safeguards. Review in 12 months.

**DPO Approval**: Jane Smith, 2025-12-25
```

---

## 7. Breach Notification

### 7.1 Breach Assessment (Article 33-34)

**Assessment Criteria:**
```json
{
  "breachAssessment": {
    "incidentId": "breach_2025_001",
    "detectedDate": "2025-12-25T08:00:00Z",
    "nature": "Unauthorized access to customer database",
    "affectedRecords": 5000,
    "dataTypes": ["name", "email", "encrypted_passwords"],
    "riskLevel": "high",
    "notificationRequired": {
      "supervisoryAuthority": true,
      "deadline": "2025-12-28T08:00:00Z",
      "dataSubjects": true,
      "threshold": "High risk to rights and freedoms"
    }
  }
}
```

### 7.2 72-Hour Notification to Supervisory Authority

**Notification Template:**
```json
{
  "breachNotification": {
    "recipient": "ICO (UK Supervisory Authority)",
    "submissionDate": "2025-12-26T10:00:00Z",
    "hoursFromDiscovery": 50,
    "details": {
      "nature": "SQL injection vulnerability allowed unauthorized access",
      "dataCategories": ["contact_info", "authentication_credentials"],
      "affectedCount": "Approximately 5,000 data subjects",
      "likelyConsequences": "Risk of phishing attacks and credential stuffing",
      "measuresTaken": [
        "Patched vulnerability immediately",
        "Forced password resets for affected accounts",
        "Enhanced monitoring deployed",
        "Forensic investigation ongoing"
      ],
      "contactPoint": "dpo@example.com, +44 20 1234 5678"
    }
  }
}
```

### 7.3 Data Subject Notification

When breach likely to result in high risk:

**Notification Content:**
```markdown
Subject: Important Security Notice - Action Required

Dear [Name],

We are writing to inform you of a security incident that may affect your personal data.

**What Happened**
On 2025-12-25, we discovered unauthorized access to our customer database through a security vulnerability.

**What Data Was Affected**
- Your name and email address
- Your encrypted password (not in readable form)

**What We're Doing**
- The vulnerability has been fixed
- We've reset your password as a precaution
- We've enhanced our security monitoring
- We've reported this to the ICO

**What You Should Do**
1. Check your email for password reset link
2. Create a new, unique password
3. Be alert for phishing emails
4. Contact us with questions: privacy@example.com

**Your Rights**
You have the right to lodge a complaint with the ICO: https://ico.org.uk

We sincerely apologize for this incident.

Example Corp Data Protection Officer
dpo@example.com
```

---

## 8. Technical Implementation

### 8.1 Consent Management API

**Grant Consent:**
```javascript
POST /api/v1/consent/grant

{
  "userId": "user_12345",
  "purposes": ["marketing_email", "analytics"],
  "consentText": "I agree to...",
  "version": "2.1",
  "timestamp": "2025-12-25T10:30:00Z",
  "proof": {
    "ipAddress": "192.168.1.100",
    "userAgent": "Mozilla/5.0..."
  }
}

Response 201 Created:
{
  "consentId": "consent_abc123",
  "status": "active",
  "withdrawalUrl": "https://example.com/consent/withdraw/abc123"
}
```

**Withdraw Consent:**
```javascript
POST /api/v1/consent/withdraw

{
  "consentId": "consent_abc123",
  "timestamp": "2025-12-25T15:00:00Z"
}

Response 200 OK:
{
  "status": "withdrawn",
  "effectiveDate": "2025-12-25T15:00:01Z",
  "processingStoppedAt": "2025-12-25T15:00:01Z"
}
```

### 8.2 Data Subject Request API

**Submit DSAR:**
```javascript
POST /api/v1/dsar/submit

{
  "requestType": "access",
  "dataSubjectId": "user_12345",
  "verificationToken": "token_verified",
  "deliveryMethod": "email",
  "deliveryAddress": "john@example.com"
}

Response 202 Accepted:
{
  "requestId": "req_789",
  "status": "pending",
  "deadline": "2025-12-31T23:59:59Z",
  "estimatedCompletion": "2025-12-15T12:00:00Z"
}
```

---

## 9. Compliance Checklist

### 9.1 Initial Compliance

- [ ] Appoint Data Protection Officer (if required)
- [ ] Create Records of Processing Activities (Article 30)
- [ ] Document lawful basis for all processing
- [ ] Review and update privacy notices
- [ ] Implement consent management system
- [ ] Establish data subject request workflows
- [ ] Create Data Processing Agreements with processors
- [ ] Conduct Data Protection Impact Assessments
- [ ] Implement breach notification procedures
- [ ] Train staff on GDPR requirements

### 9.2 Ongoing Compliance

- [ ] Regular DPIA reviews (annually)
- [ ] Update Records of Processing Activities
- [ ] Audit consent records quarterly
- [ ] Review retention schedules
- [ ] Update DPA agreements with vendors
- [ ] Test data subject request procedures
- [ ] Conduct staff training (annually)
- [ ] Monitor regulatory guidance updates
- [ ] Maintain audit logs
- [ ] Review and update security measures

---

## 10. Conclusion

WIA-SEC-024 Phase 1 provides the foundational framework for GDPR compliance. Organizations implementing this standard will establish robust data protection practices that respect individual rights while enabling legitimate data processing.

**Next Steps:**
- Implement Phase 1 core requirements
- Prepare for Phase 2 advanced features
- Engage with WIA community for best practices

---

弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
World Certification Industry Association


## Annex E — Implementation Notes for PHASE-1-CORE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-CORE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-CORE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-core/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-CORE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-CORE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-CORE.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-CORE. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-CORE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
