# WIA-SEC-017: Security Audit
## Glossary of Terms

**Standard ID:** WIA-SEC-017
**Category:** Security (SEC)
**Version:** 1.0.0
**Last Updated:** 2025-12-25

---

## A

**Access Control**
The selective restriction of access to resources based on authentication and authorization policies. In audit context, all access control decisions are logged.

**Actor**
The entity (user, service, or system) that initiates an auditable action. Every audit entry must identify the actor responsible for the event.

**Anomaly Detection**
The use of statistical analysis and machine learning to identify unusual patterns in audit logs that may indicate security threats or policy violations.

**Audit Chain**
A cryptographically linked sequence of audit entries where each entry contains a hash of the previous entry, ensuring tamper-proof logging similar to blockchain technology.

**Audit Entry**
A single record in the audit log containing information about a security-relevant event, including who, what, when, where, and how.

**Audit Trail**
A chronological record of system activities that enables reconstruction and examination of events for compliance and security purposes.

---

## B

**Batch Submission**
The practice of submitting multiple audit events together as a single transaction to improve performance and reduce network overhead.

**Blockchain-Inspired Chain**
An audit log structure where each entry is cryptographically linked to the previous entry, making it mathematically infeasible to modify past entries without detection.

**Breach Notification**
The legal requirement under regulations like GDPR to notify affected parties and authorities within a specific timeframe (typically 72 hours) when a data breach occurs.

---

## C

**Cold Storage**
Long-term archive storage for audit logs that are infrequently accessed but must be retained for compliance. Typically lower cost but with slower retrieval times.

**Compliance Framework**
A structured set of guidelines and requirements for security and data protection, such as SOC2, ISO27001, GDPR, HIPAA, or PCI-DSS.

**Compliance Tag**
A metadata label attached to audit entries indicating which compliance frameworks the event is relevant to (e.g., "SOC2", "GDPR").

**Cryptographic Hash**
A one-way mathematical function that produces a fixed-size output (hash) from variable-size input data. Used to ensure audit log integrity.

**Cryptographic Signature**
A digital signature created using private key cryptography to prove authenticity and integrity of audit entries.

---

## D

**Data Classification**
The categorization of data based on sensitivity level (PUBLIC, INTERNAL, CONFIDENTIAL, RESTRICTED) to determine appropriate security controls and audit requirements.

**Data Subject**
Under GDPR, an identifiable natural person whose personal data is being processed. Audit logs must track all processing of personal data.

**Digital Signature**
See Cryptographic Signature.

**Distributed Ledger**
A decentralized database that maintains identical copies across multiple locations, used in some audit systems for enhanced integrity and availability.

---

## E

**Event Type**
The category of auditable activity (e.g., AUTHENTICATION, DATA_ACCESS, CONFIGURATION_CHANGE) that determines the severity and compliance requirements.

**Evidence Collection**
The process of gathering audit logs and supporting documentation to demonstrate compliance with regulatory requirements during audits.

---

## F

**Flush**
The operation of sending buffered audit events from client applications to the central audit storage system.

**Forensic Analysis**
The detailed examination of audit logs to investigate security incidents, policy violations, or compliance issues.

---

## G

**GDPR (General Data Protection Regulation)**
EU regulation (2016/679) that governs the processing of personal data and requires comprehensive audit trails for compliance.

**Geo-Location**
The physical location (country, region, city) of an actor at the time of an auditable event, often used for anomaly detection.

---

## H

**Hash Chain**
See Audit Chain.

**HIPAA (Health Insurance Portability and Accountability Act)**
US healthcare regulation requiring secure handling of protected health information (PHI) with comprehensive audit logging.

**Hot Storage**
High-performance storage for recent audit logs that require fast access, typically using SSDs or in-memory databases.

---

## I

**Immutability**
The property of audit logs that prevents modification or deletion after creation, ensuring long-term integrity and trustworthiness.

**Impossible Travel**
An anomaly detection pattern where the same user appears to access systems from two geographic locations that are physically impossible to travel between in the given timeframe.

**Incident Response**
The systematic approach to addressing and managing security events, with all response actions being logged in the audit system.

**Integrity Verification**
The process of validating that audit logs have not been tampered with by checking cryptographic hashes and signatures.

**ISO 27001**
International standard (ISO/IEC 27001) for information security management systems, requiring comprehensive audit logging (especially controls A.12.4.*).

---

## J

**JWT (JSON Web Token)**
A compact token format often used for authentication, with creation and validation events being auditable.

---

## K

**Key Management**
The processes and procedures for generating, distributing, storing, rotating, and retiring cryptographic keys used in audit signing and encryption.

**Key Rotation**
The practice of periodically replacing cryptographic keys to limit exposure if a key is compromised. Recommended every 90 days for audit systems.

---

## L

**Least Privilege**
Security principle of granting minimum necessary permissions, with all privilege grants and escalations being logged.

**Log Aggregation**
The collection and centralization of audit logs from multiple sources into a unified audit system.

**Log Retention**
The period for which audit logs must be kept based on compliance requirements, typically ranging from 1 to 7 years.

---

## M

**Machine Learning**
AI techniques used in audit systems for anomaly detection, pattern recognition, and predictive analytics.

**Metadata**
Additional contextual information attached to audit entries, such as compliance tags, data classification, and custom fields.

**Multi-Factor Authentication (MFA)**
Authentication requiring two or more verification factors, with all MFA events being logged with higher detail.

---

## N

**NIST (National Institute of Standards and Technology)**
US government agency that publishes cybersecurity standards and guidelines, including recommendations for audit logging.

**Non-Repudiation**
The assurance that someone cannot deny the validity of their actions, achieved through cryptographic signatures in audit logs.

---

## O

**Operator Logs**
Audit records specifically tracking actions by system administrators and operators, separated from regular user activity logs.

---

## P

**PCI-DSS (Payment Card Industry Data Security Standard)**
Security standard for organizations handling credit card data, requiring comprehensive audit logging (Requirement 10).

**PII (Personally Identifiable Information)**
Information that can identify an individual, requiring special audit logging under privacy regulations.

**PHI (Protected Health Information)**
Health information covered under HIPAA requiring enhanced security and audit logging.

**Policy Decision Point (PDP)**
Component that evaluates access requests against policies, with decisions being logged for audit purposes.

**Policy Enforcement Point (PEP)**
Component that enforces access control decisions, logging all enforcement actions.

**Privilege Escalation**
The act of gaining higher access privileges than normally assigned, always logged as high-severity events.

---

## Q

**Query Language**
A structured syntax for searching and analyzing audit logs, similar to SQL but designed for audit-specific use cases.

---

## R

**Real-Time Monitoring**
Continuous analysis of audit events as they occur, enabling immediate detection and response to security events.

**Retention Policy**
Rules defining how long different types of audit logs must be kept based on compliance requirements and business needs.

**Risk Score**
A numerical value (typically 0-100) indicating the risk level of an audit event based on multiple factors.

**Role-Based Access Control (RBAC)**
Access control model based on user roles, with role assignments and changes being logged.

---

## S

**Severity Level**
The classification of audit events by importance: INFO, LOW, MEDIUM, HIGH, or CRITICAL.

**SIEM (Security Information and Event Management)**
Security platform that collects, analyzes, and correlates security events from multiple sources, often integrated with WIA-SEC-017 audit systems.

**SOC 2 (Service Organization Control 2)**
Audit framework for service providers focusing on security, availability, processing integrity, confidentiality, and privacy.

**Statement of Applicability (SOA)**
ISO 27001 document listing which security controls are applicable and implemented, supported by audit evidence.

---

## T

**Tamper-Proof**
Property of audit logs protected by cryptography that makes unauthorized modification detectable.

**Threat Intelligence**
Information about security threats used to enrich audit logs with context about IP addresses, attack patterns, and threat actors.

**Timestamp**
The precise date and time of an auditable event, synchronized across all systems using NTP.

**TLS (Transport Layer Security)**
Cryptographic protocol for secure communications, required for all audit data transmission.

**Trust Score**
A calculated value indicating the trustworthiness of an authentication or access request based on multiple factors.

---

## U

**User Agent**
The software application (typically browser) used by an actor, logged for context and anomaly detection.

---

## V

**Verifiable Credential (VC)**
A tamper-evident credential using W3C standards, can be used to represent audit receipts.

**Verification**
The process of checking cryptographic signatures and hash chains to ensure audit log integrity.

---

## W

**WAQL (WIA Audit Query Language)**
SQL-like query language designed specifically for searching and analyzing WIA-SEC-017 audit logs.

**Warm Storage**
Medium-term storage for audit logs that are occasionally accessed, balancing cost and performance.

**WORM (Write Once, Read Many)**
Storage technology that prevents modification or deletion after initial write, ideal for audit logs.

---

## X

**X.509 Certificate**
Digital certificate standard used for authenticating systems and signing audit logs.

---

## Z

**Zero Trust**
Security model assuming no implicit trust, with every access requiring verification and logging (related: WIA-SEC-005).

---

## Acronyms Quick Reference

| Acronym | Full Term |
|---------|-----------|
| API | Application Programming Interface |
| ARIMA | AutoRegressive Integrated Moving Average |
| DLP | Data Loss Prevention |
| DPO | Data Protection Officer |
| DPIA | Data Protection Impact Assessment |
| GDPR | General Data Protection Regulation |
| HIPAA | Health Insurance Portability and Accountability Act |
| HSM | Hardware Security Module |
| IDS | Intrusion Detection System |
| IPS | Intrusion Prevention System |
| ISO | International Organization for Standardization |
| JWT | JSON Web Token |
| LSTM | Long Short-Term Memory |
| MFA | Multi-Factor Authentication |
| NIST | National Institute of Standards and Technology |
| NTP | Network Time Protocol |
| PCI-DSS | Payment Card Industry Data Security Standard |
| PDP | Policy Decision Point |
| PEP | Policy Enforcement Point |
| PHI | Protected Health Information |
| PII | Personally Identifiable Information |
| RBAC | Role-Based Access Control |
| SIEM | Security Information and Event Management |
| SLA | Service Level Agreement |
| SOA | Statement of Applicability |
| SOC | Service Organization Control |
| SSO | Single Sign-On |
| TLS | Transport Layer Security |
| VC | Verifiable Credential |
| WAQL | WIA Audit Query Language |
| WIA | World Certification Industry Association |
| WORM | Write Once, Read Many |

---

## Compliance Framework Mapping

| Framework | Key Audit Requirements |
|-----------|------------------------|
| SOC 2 Type II | CC6.1-CC6.3 (Monitoring), CC7.1-CC7.2 (Detection & Response) |
| ISO 27001 | A.12.4.1-A.12.4.4 (Logging, Protection, Admin logs, Clock sync) |
| GDPR | Art. 30 (Records), Art. 32 (Security), Art. 33 (Breach notification) |
| HIPAA | §164.308(a)(1)(ii)(D), §164.312(b), §164.308(a)(5)(ii)(C) |
| PCI-DSS | Requirement 10 (Track and monitor all access) |

---

## Related WIA Standards

- **WIA-SEC-005** - Zero Trust Architecture
- **WIA-SEC-006** - Blockchain Security
- **WIA-SEC-009** - Identity Management
- **WIA-SEC-010** - Multi-Factor Authentication
- **WIA-SEC-011** - Biometric Authentication

---

**Previous:** [SPEC-APPENDIX.md](./SPEC-APPENDIX.md)
**Back to:** [PHASE-1-CORE.md](./PHASE-1-CORE.md)

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
