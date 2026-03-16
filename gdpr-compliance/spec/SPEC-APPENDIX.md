# WIA-SEC-024: GDPR Compliance - Appendix

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-12-25
**Standard ID:** WIA-SEC-024
**Category:** Security (SEC)
**Color:** #8B5CF6

---

## Appendix A: GDPR Articles Quick Reference

### A.1 Key Articles Overview

| Article | Title | Summary |
|---------|-------|---------|
| Art. 5 | Principles | Lawfulness, fairness, transparency, purpose limitation, data minimization, accuracy, storage limitation, integrity/confidentiality, accountability |
| Art. 6 | Lawful Basis | Consent, contract, legal obligation, vital interests, public task, legitimate interests |
| Art. 7 | Consent Conditions | Demonstrable, clear, distinguishable, withdrawable |
| Art. 12 | Transparent Information | Clear, concise, accessible, written plainly |
| Art. 13 | Info at Collection | Identity, purposes, legal basis, recipients, retention, rights |
| Art. 15 | Right of Access | Confirmation, copy, processing information |
| Art. 16 | Right to Rectification | Correct inaccurate data |
| Art. 17 | Right to Erasure | Delete when no longer necessary/consent withdrawn |
| Art. 18 | Right to Restriction | Restrict processing during disputes |
| Art. 20 | Right to Portability | Receive data in machine-readable format |
| Art. 21 | Right to Object | Object to legitimate interest/direct marketing |
| Art. 22 | Automated Decisions | Not subject to solely automated decisions with legal effects |
| Art. 25 | Data Protection by Design | Privacy by design and default |
| Art. 28 | Processor Obligations | Written contract, security, sub-processors, assistance |
| Art. 30 | Records of Processing | Maintain processing records |
| Art. 32 | Security Measures | Appropriate technical/organizational measures |
| Art. 33 | Breach Notification to Authority | 72-hour notification requirement |
| Art. 34 | Breach Notification to Subjects | When high risk to rights/freedoms |
| Art. 35 | DPIA | When high risk processing |
| Art. 37 | DPO Designation | Required for public authorities, large-scale processing |
| Art. 44-50 | International Transfers | Adequacy, safeguards, derogations |

---

## Appendix B: GDPR Fines and Penalties

### B.1 Fine Tiers

**Tier 1 (Up to €10 million or 2% of global turnover):**
- Violations of processor obligations (Art. 28-29)
- Violations of certification body obligations (Art. 42-43)
- Violations of monitoring body obligations (Art. 41)

**Tier 2 (Up to €20 million or 4% of global turnover):**
- Violations of basic principles (Art. 5, 6, 7, 9)
- Violations of data subject rights (Art. 12-22)
- Violations of transfer provisions (Art. 44-49)
- Non-compliance with supervisory authority orders

### B.2 Notable GDPR Fines (Examples)

| Organization | Year | Fine | Violation |
|-------------|------|------|-----------|
| Amazon | 2021 | €746M | Unlawful processing, inadequate consent |
| WhatsApp | 2021 | €225M | Transparency violations |
| Google | 2019 | €50M | Lack of transparency, invalid consent |
| British Airways | 2020 | €22M | Security breach, insufficient measures |
| H&M | 2020 | €35M | Excessive employee surveillance |

### B.3 Factors Influencing Fines (Article 83)

**Aggravating Factors:**
- Intentional or negligent violation
- Large number of data subjects affected
- Special category data involved
- Systematic or repeated violations
- Lack of cooperation with supervisory authority
- Previous violations

**Mitigating Factors:**
- Self-reporting and cooperation
- Prompt remediation measures
- Compliance programs in place
- No previous violations
- Limited data subjects affected
- Demonstrable technical/organizational measures

---

## Appendix C: Standard Contractual Clauses (SCCs)

### C.1 SCC Modules (2021 Version)

**Module 1: Controller to Controller**
- Use when: Two controllers jointly determine processing
- Example: Research collaboration, joint marketing campaigns

**Module 2: Controller to Processor**
- Use when: Controller engages processor for data processing services
- Example: Cloud service provider, payroll processor
- **Most Common Module**

**Module 3: Processor to Processor**
- Use when: Processor engages sub-processor
- Example: Email service using cloud infrastructure provider

**Module 4: Processor to Controller**
- Use when: Processor transfers data to controller
- Example: Analytics processor providing aggregated insights

### C.2 SCC Annexes

**Annex I: Parties and Transfer Details**
- A: List of parties (names, contacts, roles)
- B: Description of transfer (purpose, data categories, subjects, recipients, retention)
- C: Competent supervisory authority

**Annex II: Technical and Organizational Measures**
- Security measures description
- Encryption standards
- Access controls
- Incident response
- Audit procedures

**Annex III: Sub-processors (Module 2 only)**
- List of authorized sub-processors
- Notification procedure
- Objection mechanism

### C.3 SCC Key Clauses

**Clause 8: Data Protection Safeguards**
- Purpose limitation
- Data quality and minimization
- Storage limitation
- Security measures

**Clause 9: Documentation and Compliance**
- Maintain processing records
- Cooperation with supervisory authorities
- Compliance audits

**Clause 13: Supervision**
- Supervisory authority oversight
- Right to audit

**Clause 16: Non-Compliance and Termination**
- Breach notification
- Suspension rights
- Termination rights

---

## Appendix D: Data Protection Impact Assessment (DPIA) Template

### D.1 Comprehensive DPIA Template

```markdown
# Data Protection Impact Assessment (DPIA)

## Section 1: Basic Information

**Project/System Name**: ________________________________
**DPIA Reference**: ________________________________
**Assessment Date**: ________________________________
**Review Date**: ________________________________
**Assessor(s)**: ________________________________
**DPO Involvement**: Yes / No / N/A

## Section 2: Description of Processing

### 2.1 Purpose and Context
**What is the processing for?**
_____________________________________________________________
_____________________________________________________________

**What is the nature of the processing?**
- [ ] Collection    - [ ] Storage    - [ ] Analysis
- [ ] Sharing       - [ ] Deletion   - [ ] Other: _______

**Who will use the data?**
_____________________________________________________________

**What technology is involved?**
- [ ] Database      - [ ] AI/ML       - [ ] CCTV
- [ ] Biometrics    - [ ] IoT         - [ ] Cloud
- [ ] Other: ___________________________________________

### 2.2 Data Elements
**What personal data will be processed?**
- [ ] Contact details (email, phone, address)
- [ ] Identification (name, ID number, passport)
- [ ] Financial (bank account, credit card, salary)
- [ ] Location data
- [ ] Online identifiers (IP, cookies, device ID)
- [ ] Special category data (see 2.3)
- [ ] Criminal conviction data
- [ ] Other: ___________________________________________

### 2.3 Special Category Data (Article 9)
Does processing involve:
- [ ] Racial or ethnic origin
- [ ] Political opinions
- [ ] Religious or philosophical beliefs
- [ ] Trade union membership
- [ ] Genetic data
- [ ] Biometric data (for unique identification)
- [ ] Health data
- [ ] Sex life or sexual orientation

**If yes, what is the Article 9(2) condition?**
_____________________________________________________________

### 2.4 Data Subjects
- [ ] Customers/Clients
- [ ] Employees
- [ ] Children (under 16)
- [ ] Vulnerable individuals
- [ ] Other: ___________________________________________

**Approximate number of data subjects**: _______________

### 2.5 Data Lifecycle
**How is data collected?**
_____________________________________________________________

**How long is data retained?**
_____________________________________________________________

**How is data deleted/anonymized?**
_____________________________________________________________

## Section 3: Necessity and Proportionality

### 3.1 Legal Basis (Article 6)
- [ ] Consent (6(1)(a))
- [ ] Contract (6(1)(b))
- [ ] Legal obligation (6(1)(c))
- [ ] Vital interests (6(1)(d))
- [ ] Public task (6(1)(e))
- [ ] Legitimate interests (6(1)(f))

**Justification**:
_____________________________________________________________
_____________________________________________________________

### 3.2 Necessity
**Is each data element necessary for the purpose?**
_____________________________________________________________

**Could the purpose be achieved with less data?**
_____________________________________________________________

### 3.3 Proportionality
**Is the processing proportionate to the purpose?**
_____________________________________________________________

**Have less intrusive alternatives been considered?**
_____________________________________________________________

## Section 4: Risk Assessment

### 4.1 Identify Risks to Data Subjects

| Risk | Likelihood | Severity | Overall Risk |
|------|------------|----------|--------------|
| Unauthorized access/disclosure | Low/Med/High | Low/Med/High | Low/Med/High |
| Data loss or corruption | Low/Med/High | Low/Med/High | Low/Med/High |
| Discrimination or unfair treatment | Low/Med/High | Low/Med/High | Low/Med/High |
| Identity theft or fraud | Low/Med/High | Low/Med/High | Low/Med/High |
| Reputational damage | Low/Med/High | Low/Med/High | Low/Med/High |
| Financial loss | Low/Med/High | Low/Med/High | Low/Med/High |
| Loss of confidentiality | Low/Med/High | Low/Med/High | Low/Med/High |
| Other: _____________ | Low/Med/High | Low/Med/High | Low/Med/High |

**Risk Scoring**:
- **Likelihood**: Low (unlikely), Medium (possible), High (probable)
- **Severity**: Low (minimal impact), Medium (significant impact), High (severe impact)
- **Overall Risk**: Low (acceptable), Medium (requires mitigation), High (unacceptable without strong mitigation)

### 4.2 Risk Detail

**Risk 1: ___________________________________________**
- **Likelihood**: __________ **Severity**: __________
- **Description**: _____________________________________
- **Affected data subjects**: __________________________

**Risk 2: ___________________________________________**
- **Likelihood**: __________ **Severity**: __________
- **Description**: _____________________________________
- **Affected data subjects**: __________________________

## Section 5: Mitigation Measures

### 5.1 Technical Measures
- [ ] Encryption at rest (algorithm: _____________)
- [ ] Encryption in transit (TLS 1.3+)
- [ ] Pseudonymization/anonymization
- [ ] Access controls (RBAC, MFA)
- [ ] Audit logging
- [ ] Data loss prevention (DLP)
- [ ] Intrusion detection/prevention
- [ ] Regular security testing
- [ ] Other: ___________________________________________

### 5.2 Organizational Measures
- [ ] Privacy by design/default
- [ ] Staff training
- [ ] Data protection policies
- [ ] Incident response plan
- [ ] Data Processing Agreement with processors
- [ ] Regular audits/reviews
- [ ] DPO oversight
- [ ] Other: ___________________________________________

### 5.3 Data Subject Safeguards
- [ ] Clear privacy notice
- [ ] Easy consent withdrawal
- [ ] Data subject rights portal
- [ ] Transparency about automated decisions
- [ ] Human review of automated decisions
- [ ] Opt-out mechanisms
- [ ] Other: ___________________________________________

### 5.4 Mitigation Effectiveness

| Risk | Original Risk | Mitigation Measures | Residual Risk |
|------|---------------|---------------------|---------------|
| Risk 1 | High | Encryption, access controls | Medium |
| Risk 2 | Medium | Audit logging, training | Low |
| Risk 3 | ... | ... | ... |

## Section 6: Stakeholder Consultation

### 6.1 Data Protection Officer
**Consulted**: Yes / No / N/A
**Comments**: ___________________________________________
**Approval**: Yes / No / Conditional

### 6.2 Data Subjects
**Consulted**: Yes / No / Not feasible
**Method**: Survey / Focus group / Public consultation
**Feedback**: ___________________________________________

### 6.3 Other Stakeholders
**IT Security**: ___________________________________________
**Legal**: ___________________________________________
**Business Owner**: ___________________________________________

## Section 7: Compliance Check

- [ ] Lawful basis documented
- [ ] Privacy notice updated
- [ ] Consent mechanism (if applicable)
- [ ] Records of Processing Activities updated
- [ ] Data Processing Agreements in place
- [ ] Security measures implemented
- [ ] Staff trained
- [ ] Breach notification procedures ready
- [ ] Data subject rights procedures ready
- [ ] International transfer safeguards (if applicable)

## Section 8: Decision

**Overall Risk Level**: Low / Medium / High

**Decision**:
- [ ] **Proceed** - Risks acceptable with implemented mitigations
- [ ] **Proceed with conditions** - Implement additional measures: _____________
- [ ] **Consult supervisory authority** - High residual risk
- [ ] **Do not proceed** - Unacceptable risk

**DPO Sign-Off**:
Name: _____________________ Date: _____________
Signature: _____________________

**Senior Management Approval**:
Name: _____________________ Date: _____________
Signature: _____________________

## Section 9: Review

**Next Review Date**: _____________
**Review Triggers**:
- [ ] Change in processing purposes
- [ ] New technology introduced
- [ ] Change in legal requirements
- [ ] Security incident
- [ ] Annual review
- [ ] Other: ___________________________________________

**Review History**:
| Date | Reviewer | Changes | Outcome |
|------|----------|---------|---------|
|      |          |         |         |
```

---

## Appendix E: Privacy Notice Template

### E.1 Comprehensive Privacy Notice

```markdown
# Privacy Notice

**Last Updated**: [Date]

## 1. Who We Are

**Data Controller**: [Company Name]
**Registered Address**: [Address]
**Contact**: [Email/Phone]
**Data Protection Officer**: [Name]
**DPO Contact**: [Email]

## 2. What Personal Data We Collect

We collect and process the following personal data:

| Data Category | Examples | Source |
|---------------|----------|--------|
| Contact Information | Name, email, phone, address | You provide directly |
| Account Information | Username, password, preferences | You provide directly |
| Transaction Data | Purchase history, payment details | You provide, payment processor |
| Usage Data | Pages visited, clicks, time spent | Automated collection |
| Device Information | IP address, browser type, device ID | Automated collection |
| Location Data | IP-based location, GPS (if enabled) | Automated collection |
| Communications | Email correspondence, chat logs | You provide, customer service |

**Special Category Data**: We do not collect special category data (race, religion, health, etc.) unless specifically indicated below:
[If applicable, list special category data and Article 9(2) condition]

## 3. How We Use Your Personal Data

| Purpose | Data Used | Legal Basis | Retention Period |
|---------|-----------|-------------|------------------|
| Provide services/fulfill orders | Contact, transaction data | Contract (Art. 6(1)(b)) | 6 years post-contract |
| Customer support | Contact, communications | Contract (Art. 6(1)(b)) | 3 years |
| Marketing communications | Contact, preferences | Consent (Art. 6(1)(a)) | Until consent withdrawn |
| Analytics & improvements | Usage data, device info | Legitimate interests (Art. 6(1)(f)) | 26 months |
| Fraud prevention | Transaction, device data | Legitimate interests (Art. 6(1)(f)) | 6 years |
| Legal compliance | All relevant data | Legal obligation (Art. 6(1)(c)) | As required by law |

**Legitimate Interests**: Where we rely on legitimate interests, we have balanced our interests against your rights and freedoms. You have the right to object (see section 7).

## 4. Who We Share Your Data With

We share your personal data with:

| Recipient | Purpose | Safeguards |
|-----------|---------|------------|
| Payment Processors (e.g., Stripe) | Process payments | PCI-DSS compliance, DPA |
| Email Service (e.g., MailChimp) | Send marketing emails | Standard Contractual Clauses, DPA |
| Cloud Hosting (e.g., AWS) | Store data | Standard Contractual Clauses, encryption |
| Analytics (e.g., Google Analytics) | Website analytics | Anonymization, DPA |
| Customer Support (e.g., Zendesk) | Provide support | Standard Contractual Clauses, DPA |

**Third Country Transfers**: Some recipients are located outside the EU/EEA. We use the following safeguards:
- Standard Contractual Clauses (2021)
- Adequacy decisions (UK, Canada commercial)
- Additional security measures (encryption, access controls)

**No Sale of Data**: We do not sell your personal data to third parties.

## 5. How Long We Keep Your Data

| Data Type | Retention Period | Reason |
|-----------|------------------|--------|
| Account data | Duration of account + 6 years | Legal requirement (tax, contracts) |
| Marketing data | Until consent withdrawn + 1 year | Prove consent was obtained |
| Support tickets | 3 years after closure | Business need, quality assurance |
| Analytics data | 26 months | Industry standard, legitimate interest |
| CCTV footage | 30 days | Security need balanced with privacy |

**Deletion**: After the retention period, data is securely deleted or anonymized.

## 6. How We Protect Your Data

We implement appropriate technical and organizational measures:

**Technical Measures**:
- Encryption at rest (AES-256) and in transit (TLS 1.3)
- Access controls and authentication (MFA)
- Regular security testing and vulnerability scanning
- Intrusion detection and prevention systems
- Secure backup and disaster recovery

**Organizational Measures**:
- Staff training on data protection
- Confidentiality agreements
- Access on need-to-know basis
- Data protection policies and procedures
- Regular audits and compliance reviews

**Breach Notification**: In case of a data breach likely to result in high risk to your rights, we will notify you without undue delay.

## 7. Your Rights

You have the following rights under GDPR:

### Right of Access (Article 15)
Request a copy of your personal data and information about processing.
**How to exercise**: Email privacy@[company].com

### Right to Rectification (Article 16)
Correct inaccurate or incomplete personal data.
**How to exercise**: Update in account settings or email privacy@[company].com

### Right to Erasure (Article 17)
Request deletion of your personal data when:
- No longer necessary for the purpose
- You withdraw consent
- You object to processing
- Processing is unlawful

**Exceptions**: We may retain data if required by law or for legal claims.
**How to exercise**: Email privacy@[company].com

### Right to Restriction (Article 18)
Request restriction of processing while we verify accuracy or assess your objection.
**How to exercise**: Email privacy@[company].com

### Right to Data Portability (Article 20)
Receive your data in machine-readable format (JSON/CSV) and transmit to another controller.
**How to exercise**: Request via privacy@[company].com

### Right to Object (Article 21)
Object to processing based on legitimate interests or for direct marketing.
**Direct marketing**: Opt-out at any time using unsubscribe link or privacy@[company].com
**Other processing**: Email privacy@[company].com with reasons

### Rights Related to Automated Decision-Making (Article 22)
You have the right not to be subject to solely automated decisions with legal or similarly significant effects.
**Automated decisions we make**: [List any automated decisions, e.g., credit scoring, fraud detection]
**Safeguards**: Human review, right to contest, explanation of logic

**Response Time**: We will respond to your request within 1 month (extendable to 3 months for complex requests).

**Verification**: We may request additional information to verify your identity before responding to requests.

**No Fee**: Exercising your rights is free unless requests are manifestly unfounded or excessive.

## 8. Cookies and Tracking

We use cookies and similar technologies:

| Cookie Type | Purpose | Duration | Opt-Out |
|-------------|---------|----------|---------|
| Essential | Website functionality | Session/1 year | Not possible (required for service) |
| Analytics | Understand usage patterns | 2 years | Cookie settings / Browser settings |
| Marketing | Personalized ads | 1 year | Cookie settings / Opt-out |
| Preferences | Remember your choices | 1 year | Browser settings |

**Manage Cookies**: [Link to cookie settings]

**Third-Party Cookies**: We use Google Analytics, Facebook Pixel, etc. See their privacy policies:
- [Links to third-party policies]

## 9. Children's Privacy

Our services are not directed to children under 16. We do not knowingly collect data from children.

**If you are a parent/guardian** and believe we have collected your child's data, contact us immediately for deletion.

## 10. Changes to This Notice

We may update this privacy notice from time to time. Changes will be posted on this page with an updated "Last Updated" date.

**Significant changes**: We will notify you by email or prominent notice on our website.

## 11. Your Right to Complain

You have the right to lodge a complaint with a supervisory authority:

**Lead Supervisory Authority**: [Name]
**Contact**: [Address, email, phone]
**Website**: [URL]

**Other EU Authorities**: https://edpb.europa.eu/about-edpb/board/members_en

## 12. Contact Us

Questions or concerns about this privacy notice or our data practices:

**Email**: privacy@[company].com
**Phone**: [Phone number]
**Post**: Data Protection Officer, [Company Name], [Address]

---

© [Year] [Company Name]. All rights reserved.
```

---

## Appendix F: Sample Forms and Templates

### F.1 Data Subject Access Request Form

```markdown
# Data Subject Access Request (DSAR) Form

## Personal Information
**Full Name**: _________________________________________
**Email Address**: _________________________________________
**Phone Number**: _________________________________________
**Previous Email/Username** (if different): _________________________________________

## Verification
To protect your privacy, we need to verify your identity.

**Please provide ONE of the following**:
- [ ] Copy of government-issued ID (passport, driver's license)
- [ ] Last 4 digits of payment method on file
- [ ] Answer to security question: _________________________

**Document Upload**: [Upload button]

## Request Details

**What information do you want?**
- [ ] All personal data we hold about you
- [ ] Specific information (please specify): ___________________

**Delivery Method**:
- [ ] Secure email attachment (encrypted PDF)
- [ ] Secure download link
- [ ] Postal mail (additional verification required)

**Additional Information**:
_____________________________________________________________
_____________________________________________________________

## Declaration
I confirm that I am the data subject or authorized representative and that the information provided is accurate.

**Signature**: _____________________ **Date**: _____________

## For Official Use Only
**Request ID**: _______________
**Received**: _______________
**Deadline**: _______________
**Assigned to**: _______________
**Status**: Pending / In Progress / Completed
```

### F.2 Consent Withdrawal Form

```markdown
# Consent Withdrawal Form

## Personal Information
**Name**: _________________________________________
**Email**: _________________________________________

## Consent to Withdraw

**I wish to withdraw my consent for**:
- [ ] Marketing emails
- [ ] SMS marketing
- [ ] Personalized advertising
- [ ] Data sharing with partners
- [ ] Profiling for recommendations
- [ ] Other: _____________________________________

## Effective Date
- [ ] Immediately
- [ ] From specific date: _______________

## Acknowledgment
I understand that:
- Withdrawal will not affect processing based on consent before withdrawal
- Some services may no longer be available if consent is necessary for provision
- I can re-consent at any time

**Signature**: _____________________ **Date**: _____________

## Confirmation
Your consent withdrawal has been processed.
**Reference**: _______________
**Processed by**: _______________
**Date**: _______________
```

### F.3 Data Breach Notification Template (to Data Subjects)

```markdown
Subject: Important Security Notice - Data Breach Notification

Dear [Name],

We are writing to inform you of a data security incident that may affect your personal information.

## What Happened
On [date], we discovered [description of incident, e.g., "unauthorized access to our customer database through a security vulnerability"].

## What Information Was Involved
The following personal information may have been accessed:
- [List data types, e.g., name, email address, encrypted passwords]

The following information was NOT affected:
- [List, e.g., payment card details, social security numbers]

## What We Are Doing
We have taken the following steps:
- Immediately secured the vulnerability
- Launched a forensic investigation
- Notified the [Supervisory Authority name]
- [Other measures, e.g., reset passwords, enhanced monitoring]

## What You Should Do
We recommend you take the following precautions:
1. [Specific action, e.g., "Change your password immediately using the link below"]
2. [E.g., "Be alert for phishing emails pretending to be from us"]
3. [E.g., "Monitor your accounts for suspicious activity"]
4. [E.g., "Consider enabling two-factor authentication"]

**Password Reset Link**: [Secure link]

## Your Rights
You have the right to:
- Lodge a complaint with the supervisory authority: [Name, contact]
- Request more information about the breach
- Exercise your data protection rights (access, erasure, etc.)

## Contact Us
If you have questions or concerns:
- **Email**: [security@company.com]
- **Phone**: [Phone number]
- **Reference Number**: [Incident ID]

We sincerely apologize for this incident and the concern it may cause.

[Company Name]
Data Protection Officer
[Date]
```

---

## Appendix G: Implementation Checklist

### G.1 90-Day GDPR Compliance Roadmap

**Days 1-30: Foundation**
- [ ] Appoint Data Protection Officer (if required)
- [ ] Form GDPR compliance team
- [ ] Conduct data inventory audit
- [ ] Map data flows
- [ ] Identify lawful basis for all processing
- [ ] Review and update privacy notices
- [ ] Create Records of Processing Activities (RoPA)

**Days 31-60: Implementation**
- [ ] Implement consent management system
- [ ] Update forms and collection points
- [ ] Create data subject request workflows
- [ ] Draft Data Processing Agreements
- [ ] Conduct vendor due diligence
- [ ] Implement technical security measures
- [ ] Develop breach notification procedures
- [ ] Create staff training program

**Days 61-90: Testing and Documentation**
- [ ] Test data subject request procedures
- [ ] Conduct Data Protection Impact Assessments
- [ ] Finalize all documentation
- [ ] Train all staff
- [ ] Set up compliance monitoring
- [ ] Create audit schedule
- [ ] Establish ongoing review processes
- [ ] Document compliance decisions

---

弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
World Certification Industry Association
