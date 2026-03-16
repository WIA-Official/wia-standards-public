# WIA-BIO-001: Security & Privacy Specification

## Version 1.0 | 2025-01-15

---

## Table of Contents
1. [Overview](#overview)
2. [Regulatory Compliance](#regulatory-compliance)
3. [Data Encryption](#data-encryption)
4. [Access Control](#access-control)
5. [De-identification](#de-identification)
6. [Consent Management](#consent-management)
7. [Audit Logging](#audit-logging)
8. [Genomic Privacy Risks](#genomic-privacy-risks)

---

## Overview

Genomic data represents the most sensitive form of personal information, containing immutable identifiers with implications for individuals, biological relatives, and future generations. WIA-BIO-001 mandates comprehensive security and privacy controls that exceed standard PHI (Protected Health Information) requirements.

### Unique Genomic Privacy Challenges

| Challenge | Description | Mitigation |
|-----------|-------------|------------|
| **Immutability** | Cannot change genome if compromised | Lifetime encryption, limited access |
| **Familial Implications** | Reveals information about relatives | Family consent, restricted sharing |
| **Re-identification Risk** | Linkage to public databases | Differential privacy, controlled access |
| **Temporal Risk** | Future reanalysis reveals new info | Dynamic consent, data recalls |
| **Discrimination Risk** | Genetic discrimination in insurance/employment | Legal protections (GINA), strict usage policies |

---

## Regulatory Compliance

### HIPAA (Health Insurance Portability and Accountability Act)

**Applicability:** All genomic data linked to identifiable individuals in the United States.

**Key Requirements:**
1. **Privacy Rule:** De-identification of 18 identifiers
2. **Security Rule:** Administrative, physical, technical safeguards
3. **Breach Notification:** Report breaches affecting >500 individuals

**Genomic Data as PHI:**
```
Genomic sequence data is considered PHI when it includes:
- Sample/patient identifiers
- Date of collection
- Institution/provider information
- Any other of the 18 HIPAA identifiers
```

**Safe Harbor De-identification (18 Identifiers to Remove):**
1. Names
2. Geographic subdivisions smaller than state
3. Dates (except year)
4. Telephone/fax numbers
5. Email addresses
6. SSN
7. Medical record numbers
8. Health plan beneficiary numbers
9. Account numbers
10. Certificate/license numbers
11. Vehicle identifiers
12. Device identifiers/serial numbers
13. URLs
14. IP addresses
15. Biometric identifiers
16. Full-face photos
17. Any unique identifying numbers
18. **Genomic sequences (unless expert determination applied)**

### GDPR (General Data Protection Regulation)

**Applicability:** Genomic data of EU citizens, regardless of processing location.

**Classification:** Genetic data is **special category data** under Article 9.

**Requirements:**
- **Explicit consent:** Not implied, must be specific
- **Right to erasure:** "Right to be forgotten" (challenging for genomic data)
- **Data portability:** Must provide data in machine-readable format
- **Data protection by design:** Privacy built into systems from inception

**Lawful Basis for Processing:**
```
Genetic data processing requires:
1. Explicit consent, OR
2. Medical diagnosis/treatment, OR
3. Public health necessity, OR
4. Scientific research (with safeguards)
```

### GINA (Genetic Information Nondiscrimination Act)

**Protections:**
- Prohibits genetic discrimination in health insurance and employment (U.S.)
- Does NOT cover life, disability, or long-term care insurance

**Limitations:**
```
GINA does not prevent:
- Discrimination in life insurance underwriting
- Use of family history (vs. genetic testing) in employment decisions
- Military or veterans' health programs
```

### CAP/CLIA (Clinical Laboratory Standards)

**For clinical genomic testing:**
- **CAP:** Accreditation, quality standards, proficiency testing
- **CLIA:** Federal certification for clinical labs
- **Validation:** Analytical and clinical validity required

---

## Data Encryption

### Encryption at Rest

**Standard:** AES-256 (Advanced Encryption Standard, 256-bit key)

**Implementation:**
```bash
# Encrypt FASTQ file
openssl enc -aes-256-cbc -salt -in reads.fastq -out reads.fastq.enc -k <password>

# Decrypt
openssl enc -d -aes-256-cbc -in reads.fastq.enc -out reads.fastq -k <password>
```

**Recommended:** Use key management systems (KMS)
```python
# AWS KMS example
import boto3

kms = boto3.client('kms', region_name='us-east-1')

# Encrypt data key
response = kms.encrypt(
    KeyId='arn:aws:kms:us-east-1:123456789:key/abc123',
    Plaintext=b'data_encryption_key'
)
ciphertext = response['CiphertextBlob']
```

### Encryption in Transit

**Standard:** TLS 1.3 (minimum TLS 1.2)

**Configuration (nginx example):**
```nginx
server {
    listen 443 ssl http2;
    ssl_protocols TLSv1.2 TLSv1.3;
    ssl_ciphers 'ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384';
    ssl_prefer_server_ciphers on;
    ssl_certificate /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;
}
```

### Genomic Data Encryption Best Practices

**1. Separate Metadata from Sequence Data:**
```
encrypted/
├── metadata_encrypted.json  (AES-256, Key A)
├── variants_encrypted.vcf.gz (AES-256, Key B)
└── alignments_encrypted.bam  (AES-256, Key C)
```

**2. Key Rotation:**
- Rotate encryption keys every 90 days minimum
- Use versioned keys to maintain access to old data

**3. Homomorphic Encryption (Experimental):**
```
Allows computation on encrypted data without decryption
Use cases:
- Encrypted GWAS analysis
- Secure genomic queries across institutions
- Privacy-preserving variant matching
```

---

## Access Control

### Role-Based Access Control (RBAC)

**Standard Roles:**

| Role | Permissions | Use Case |
|------|-------------|----------|
| **Sequencer** | Upload raw data only | Lab technicians |
| **Analyst** | Read raw + aligned data, write variants | Bioinformaticians |
| **Clinician** | Read clinical reports, limited variant access | Physicians |
| **Researcher** | Read de-identified data | Population studies |
| **Administrator** | All permissions | System admins |
| **Patient** | Read own data, export, delete | Data subject |

### Attribute-Based Access Control (ABAC)

**Fine-grained control based on attributes:**
```json
{
  "policy": {
    "effect": "allow",
    "principal": "user:analyst_001",
    "action": ["read", "download"],
    "resource": "genomic-data/*",
    "condition": {
      "dataType": "de-identified",
      "consentStatus": "granted",
      "projectApproved": true
    }
  }
}
```

### Multi-Factor Authentication (MFA)

**Mandatory for:**
- Access to identified genomic data
- Administrative functions
- Data export operations

**Acceptable Methods:**
1. TOTP (Time-based One-Time Password) - Google Authenticator, Authy
2. Hardware tokens - YubiKey, Titan
3. Biometric + PIN

**Implementation (example using AWS):**
```python
import boto3

iam = boto3.client('iam')

# Enforce MFA for data access
policy = {
    "Version": "2012-10-17",
    "Statement": [{
        "Effect": "Deny",
        "Action": "s3:GetObject",
        "Resource": "arn:aws:s3:::genomic-data/*",
        "Condition": {
            "BoolIfExists": {"aws:MultiFactorAuthPresent": "false"}
        }
    }]
}
```

### Data Use Agreements (DUAs)

**Required for all data access:**
```
1. Purpose limitation: Specify allowed research uses
2. Re-identification prohibition: No attempts to identify individuals
3. No redistribution: Cannot share with third parties
4. Publication restrictions: Aggregate results only, n>10 for cells
5. Destruction clause: Delete data after project completion
```

---

## De-identification

### HIPAA Safe Harbor Method

**Process:**
1. Remove all 18 HIPAA identifiers
2. Expert determination (if needed for genomic sequences)
3. Document de-identification process

**Example De-identified VCF Header:**
```vcf
##fileformat=VCFv4.3
##fileDate=2025  # Year only, no month/day
##source=GATK_HaplotypeCaller
##reference=GRCh38
##sampleID=DEIDENTIFIED_001  # Pseudonymized ID
##contig=<ID=chr1,length=248956422>
#CHROM	POS	ID	REF	ALT	QUAL	FILTER	INFO	FORMAT	DEIDENTIFIED_001
```

### K-Anonymity for Genomic Data

**Concept:** Each individual is indistinguishable from at least k-1 others.

**Challenge:** Rare variants make k-anonymity difficult.

**Approach:**
```
- Suppress rare variants (MAF < 0.01)
- Generalize genomic regions (report region, not exact position)
- Aggregate small cells in GWAS results
```

**Example:**
```
Before: chr1:12345 G>A (unique to 1 individual)
After:  chr1:12000-13000 region contains rare variant (shared by 5 individuals)
```

### Differential Privacy

**Add calibrated noise to protect individual privacy while preserving population statistics.**

**Laplace Mechanism:**
```
noise = Laplace(0, Δf/ε)

where:
ε = privacy budget (smaller = more privacy)
Δf = sensitivity of query
```

**Example (Allele Frequency):**
```python
import numpy as np

def differentially_private_allele_freq(genotypes, epsilon=0.1):
    """
    genotypes: array of 0/1/2 (ref/het/hom)
    epsilon: privacy budget
    """
    true_freq = np.mean(genotypes) / 2  # Convert to allele frequency
    sensitivity = 1 / (2 * len(genotypes))  # Max change from one person
    noise = np.random.laplace(0, sensitivity / epsilon)
    return max(0, min(1, true_freq + noise))  # Clip to [0, 1]

# Usage
genotypes = np.array([0, 0, 1, 1, 2, 0, 0, 1])  # 8 individuals
private_freq = differentially_private_allele_freq(genotypes, epsilon=0.1)
print(f"Private allele frequency: {private_freq:.3f}")
```

### Re-identification Attack Mitigations

**Threat Model:**
1. **Genealogical DB Matching:** Attacker uploads DNA to GEDmatch, finds relatives
2. **Triangulation:** Cross-reference with public variants (gnomAD, 1000 Genomes)
3. **Rare Variant Fingerprinting:** Unique combination of rare variants

**Mitigations:**
```
1. Controlled access tiers (open, registered, controlled)
2. Query restrictions (no single-person queries)
3. Beacon-style responses (yes/no only, no genotypes)
4. Rate limiting and audit logging
5. Prohibition of genealogical database uploads in DUA
```

---

## Consent Management

### Tiered Consent Model

**Level 1: Clinical Care Only**
- Use for diagnosis and treatment only
- No research, no data sharing

**Level 2: Clinical + Affiliated Research**
- Institutional research projects only
- No external sharing

**Level 3: Clinical + Broad Research**
- Any health-related research
- Controlled-access data repositories

**Level 4: Unrestricted Research**
- Any research purpose (including commercial)
- Public data release (de-identified)

### Dynamic Consent

**Allows participants to update preferences over time.**

**Implementation Requirements:**
```json
{
  "consent": {
    "participantId": "PARTICIPANT_001",
    "initialDate": "2025-01-15",
    "currentStatus": "active",
    "permissions": {
      "clinicalUse": true,
      "research": true,
      "dataSharing": "controlled-access",
      "recontact": true,
      "commercialUse": false
    },
    "history": [
      {
        "date": "2025-01-15",
        "action": "initial_consent",
        "changes": "enrolled with broad research consent"
      },
      {
        "date": "2025-06-20",
        "action": "updated",
        "changes": "withdrew commercial use permission"
      }
    ]
  }
}
```

### Return of Results

**Categories:**
1. **Clinically Actionable Findings:** ACMG secondary findings (73 genes)
2. **Research Findings:** Discoveries during study
3. **Incidental Findings:** Unrelated to study purpose

**Consent Questions:**
```
Do you want to receive:
[ ] Actionable genetic findings related to your health?
[ ] Research results from this study?
[ ] Incidental findings unrelated to this study?
[ ] Updates as new discoveries are made about your genome?
```

---

## Audit Logging

### Required Log Events

| Event Type | Details to Log | Retention |
|------------|----------------|-----------|
| **Access** | User, timestamp, resource, action | 7 years |
| **Export** | User, files, destination, purpose | 7 years |
| **Modification** | User, changes made, reason | 7 years |
| **Consent Changes** | Participant, old/new status | Lifetime |
| **Deletion** | User, data deleted, reason | Lifetime |
| **Breach Attempts** | IP, user, failed action | Indefinite |

### Log Format

```json
{
  "timestamp": "2025-01-15T14:32:10Z",
  "eventType": "DATA_ACCESS",
  "user": {
    "id": "user_001",
    "role": "analyst",
    "authentication": "mfa_verified"
  },
  "resource": {
    "type": "vcf",
    "id": "sample_001_variants.vcf.gz",
    "classification": "identified_genomic_data"
  },
  "action": "download",
  "outcome": "success",
  "ipAddress": "203.0.113.42",
  "justification": "Clinical variant interpretation for patient case #12345"
}
```

### Automated Monitoring

**Alert Conditions:**
```
1. Bulk download (>10 samples in 1 hour)
2. After-hours access (outside 6am-10pm)
3. Failed authentication (>3 attempts)
4. Access by terminated user
5. Unusual geographic access (VPN required for remote)
6. Data access without active consent
```

**SIEM Integration (Security Information and Event Management):**
```bash
# Forward logs to Splunk
/opt/splunk/bin/splunk add forward-server splunk.example.com:9997
/opt/splunk/bin/splunk add monitor /var/log/genomic-access.log
```

---

## Genomic Privacy Risks

### Case Study: Re-identification Attack

**Scenario:** Researcher re-identifies "anonymous" genome from 1000 Genomes Project.

**Method:**
1. Downloaded de-identified genome (publicly available)
2. Focused on rare variants (MAF < 0.001)
3. Queried genealogical database (GEDmatch) with subset of variants
4. Found 3rd cousin match, built family tree
5. Cross-referenced with public records → identified individual

**WIA-BIO-001 Mitigations:**
```
1. Controlled-access tiers (no public identified data)
2. Data Use Agreement prohibiting re-identification
3. Suppression of ultra-rare variants (singletons)
4. Geographical restrictions for high-risk populations
5. Return agreements (data destruction after project)
```

### Emerging Privacy Technologies

**1. Secure Multi-Party Computation (MPC):**
```
Multiple parties jointly compute function without revealing inputs
Example: Determine if patient has rare variant without revealing full genome
```

**2. Federated Learning:**
```
Train ML models across institutions without sharing raw data
Example: Polygenic risk score development on distributed datasets
```

**3. Trusted Execution Environments (TEE):**
```
Hardware-based isolated execution (Intel SGX, ARM TrustZone)
Example: Variant analysis inside secure enclave, only results exported
```

**4. Blockchain for Consent:**
```
Immutable, auditable consent trails
Smart contracts automatically enforce data use restrictions
```

---

## Incident Response Plan

### Breach Notification Timeline

**Immediate (within 1 hour):**
- Contain breach (revoke credentials, isolate systems)
- Assess scope (number of affected individuals)

**Within 24 hours:**
- Notify security team and legal counsel
- Begin forensic investigation

**Within 60 days (HIPAA requirement):**
- Notify affected individuals
- Notify HHS (if ≥500 individuals)
- Public notification (if ≥500 individuals)

### Sample Breach Notification

```
Subject: Notification of Genomic Data Security Incident

Dear [Participant],

We are writing to inform you of a security incident involving your genomic data.

What Happened:
On [DATE], unauthorized access to our genomic database occurred, potentially exposing
de-identified genomic data of [NUMBER] participants, including yours.

What Information Was Involved:
- De-identified genomic variants (VCF format)
- Sequencing quality metrics
- No names, dates, or contact information were exposed

What We Are Doing:
- Immediately revoked access and secured systems
- Engaged cybersecurity forensics firm
- Reported to law enforcement
- Enhanced security measures (MFA, encryption)

What You Can Do:
- We are offering 2 years of genetic counseling services at no cost
- You may withdraw consent and request data deletion
- Contact us at privacy@example.com with questions

We sincerely apologize for this incident.
```

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0
