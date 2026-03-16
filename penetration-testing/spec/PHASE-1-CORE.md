# WIA-SEC-019: Penetration Testing - Phase 1 Core Specification

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-12-25
**Standard ID:** WIA-SEC-019
**Category:** Security (SEC)
**Color:** #8B5CF6
**Emoji:** 🎯

---

## 1. Introduction

### 1.1 Purpose

WIA-SEC-019 establishes a comprehensive, standardized framework for conducting penetration testing and security assessments. This specification provides:

- **Unified Methodology**: Integration of PTES (Penetration Testing Execution Standard), OWASP, and MITRE ATT&CK frameworks
- **Ethical Guidelines**: Clear boundaries for responsible security testing and disclosure
- **Systematic Approach**: Structured phases from reconnaissance to reporting
- **Quality Assurance**: Standardized metrics, deliverables, and success criteria
- **Professional Standards**: Certification requirements and code of conduct for penetration testers

### 1.2 Scope

This Phase 1 specification covers:

- Core penetration testing methodologies and execution frameworks
- Pre-engagement processes and legal considerations
- Information gathering and reconnaissance techniques
- Vulnerability assessment and exploitation procedures
- Post-exploitation activities and persistence mechanisms
- Professional reporting and remediation guidance
- Integration with security tools and platforms

Out of scope for Phase 1 (covered in later phases):
- Advanced red team operations and adversary simulation
- Automated continuous penetration testing platforms
- Machine learning-based attack pattern recognition
- Cloud-native and container-specific testing methodologies

### 1.3 Philosophy

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

Ethical hacking serves to strengthen security by proactively identifying vulnerabilities before malicious actors can exploit them. Security testing must be:

- **Authorized**: Only test systems with explicit, documented permission
- **Ethical**: Follow responsible disclosure and professional conduct guidelines
- **Systematic**: Use proven methodologies to ensure comprehensive coverage
- **Constructive**: Provide actionable remediation guidance, not just criticism
- **Educational**: Share knowledge to elevate the security community

---

## 2. Core Concepts

### 2.1 Penetration Testing Phases (PTES)

The Penetration Testing Execution Standard defines seven key phases:

#### 2.1.1 Pre-Engagement Interactions

**Purpose**: Establish scope, rules of engagement, and legal authorization

**Key Activities**:
- Scope definition and boundary establishment
- Legal authorization and liability waivers
- Rules of engagement documentation
- Communication protocols and escalation procedures
- Success criteria and deliverables agreement

**Example Scope Document**:
```json
{
  "engagement": {
    "id": "PENTEST-2025-001",
    "client": "Example Corporation",
    "type": "External Network Penetration Test",
    "scope": {
      "inScope": [
        "203.0.113.0/24",
        "*.example.com",
        "app.example.com"
      ],
      "outOfScope": [
        "partner.example.com",
        "legacy.example.com"
      ]
    },
    "authorization": {
      "documentId": "AUTH-2025-001",
      "authorizedBy": "John Doe, CISO",
      "signatureDate": "2025-01-15",
      "validUntil": "2025-03-15"
    },
    "rulesOfEngagement": {
      "testingHours": "Monday-Friday 09:00-17:00 UTC",
      "denialOfService": "prohibited",
      "socialEngineering": "email phishing only, approved templates",
      "dataExfiltration": "simulation only, no actual data removal",
      "notificationThreshold": "critical vulnerabilities found"
    }
  }
}
```

#### 2.1.2 Intelligence Gathering

**Purpose**: Collect information about the target using passive and active techniques

**Passive Reconnaissance**:
- OSINT (Open Source Intelligence) collection
- DNS enumeration and subdomain discovery
- WHOIS and registration data analysis
- Social media and public records research
- Search engine reconnaissance

**Active Reconnaissance**:
- Network scanning and port enumeration
- Service version detection
- Operating system fingerprinting
- Application discovery and mapping

**Example Reconnaissance Data**:
```json
{
  "target": "example.com",
  "reconnaissance": {
    "passive": {
      "dnsRecords": {
        "A": ["203.0.113.10", "203.0.113.11"],
        "MX": ["mail.example.com"],
        "TXT": ["v=spf1 include:_spf.google.com ~all"]
      },
      "subdomains": [
        "www.example.com",
        "app.example.com",
        "api.example.com",
        "admin.example.com"
      ],
      "technologies": [
        "Nginx 1.21.0",
        "PHP 8.1",
        "MySQL",
        "WordPress 6.1"
      ]
    },
    "active": {
      "openPorts": [
        {"port": 22, "service": "SSH", "version": "OpenSSH 8.9"},
        {"port": 80, "service": "HTTP", "version": "nginx/1.21.0"},
        {"port": 443, "service": "HTTPS", "version": "nginx/1.21.0"},
        {"port": 3306, "service": "MySQL", "version": "8.0.28"}
      ],
      "operatingSystem": {
        "family": "Linux",
        "version": "Ubuntu 22.04 LTS",
        "confidence": 95
      }
    }
  }
}
```

#### 2.1.3 Threat Modeling

**Purpose**: Identify attack vectors and prioritize testing based on business impact

**Components**:
- Asset identification and classification
- Threat actor profiling
- Attack surface mapping
- Risk prioritization matrix

**Example Threat Model**:
```json
{
  "threatModel": {
    "assets": [
      {
        "id": "ASSET-001",
        "name": "Customer Database",
        "classification": "CRITICAL",
        "businessImpact": "HIGH",
        "location": "mysql://db.internal.example.com"
      },
      {
        "id": "ASSET-002",
        "name": "Payment Processing API",
        "classification": "CRITICAL",
        "businessImpact": "CRITICAL",
        "location": "https://api.example.com/payment"
      }
    ],
    "threatActors": [
      {
        "profile": "External Opportunistic Attacker",
        "capability": "MEDIUM",
        "motivation": "Financial gain",
        "resources": "Automated tools, public exploits"
      },
      {
        "profile": "Advanced Persistent Threat",
        "capability": "HIGH",
        "motivation": "Espionage, data theft",
        "resources": "Custom malware, zero-days"
      }
    ],
    "attackVectors": [
      {
        "vector": "SQL Injection",
        "likelihood": "HIGH",
        "impact": "CRITICAL",
        "priority": 1
      },
      {
        "vector": "Authentication Bypass",
        "likelihood": "MEDIUM",
        "impact": "HIGH",
        "priority": 2
      }
    ]
  }
}
```

#### 2.1.4 Vulnerability Analysis

**Purpose**: Identify security weaknesses through automated and manual testing

**Techniques**:
- Automated vulnerability scanning
- Configuration review and hardening assessment
- Patch level analysis
- Security control testing
- OWASP Top 10 testing for web applications

**OWASP Top 10 (2021) Testing**:

1. **A01:2021 – Broken Access Control**
   - Test for IDOR (Insecure Direct Object References)
   - Verify horizontal and vertical privilege escalation
   - Check for missing function-level access controls

2. **A02:2021 – Cryptographic Failures**
   - Test for weak encryption algorithms
   - Verify TLS/SSL configuration
   - Check for sensitive data exposure

3. **A03:2021 – Injection**
   - SQL injection testing
   - Command injection
   - LDAP/XPath/XML injection

4. **A04:2021 – Insecure Design**
   - Architecture and design flaw analysis
   - Business logic vulnerability testing

5. **A05:2021 – Security Misconfiguration**
   - Default credentials testing
   - Unnecessary features enabled
   - Error handling and stack trace exposure

**Example Vulnerability Report**:
```json
{
  "vulnerability": {
    "id": "VULN-2025-001",
    "title": "SQL Injection in Login Form",
    "severity": "CRITICAL",
    "cvss": {
      "version": "3.1",
      "vector": "CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:C/C:H/I:H/A:H",
      "score": 10.0
    },
    "description": "The login form is vulnerable to SQL injection, allowing unauthenticated attackers to bypass authentication and execute arbitrary SQL queries.",
    "location": "https://app.example.com/login.php",
    "parameter": "username",
    "payload": "admin' OR '1'='1' -- ",
    "evidence": {
      "request": "POST /login.php\nusername=admin' OR '1'='1' --&password=test",
      "response": "HTTP/1.1 302 Found\nLocation: /dashboard.php"
    },
    "impact": {
      "confidentiality": "HIGH",
      "integrity": "HIGH",
      "availability": "HIGH",
      "businessImpact": "Complete database compromise, unauthorized access to customer data"
    },
    "remediation": {
      "shortTerm": "Disable vulnerable endpoint immediately",
      "longTerm": "Implement prepared statements/parameterized queries",
      "verification": "Retest after remediation to confirm fix"
    },
    "references": [
      "OWASP A03:2021 - Injection",
      "CWE-89: Improper Neutralization of Special Elements used in an SQL Command",
      "MITRE ATT&CK T1190: Exploit Public-Facing Application"
    ]
  }
}
```

#### 2.1.5 Exploitation

**Purpose**: Demonstrate vulnerability impact through controlled exploitation

**Principles**:
- Least privilege: Minimize system impact
- Documentation: Record all actions taken
- Reversibility: Ability to undo changes
- Client notification: Communicate critical findings immediately

**Exploitation Workflow**:
```json
{
  "exploitation": {
    "vulnerability": "VULN-2025-001",
    "objective": "Demonstrate authentication bypass and database access",
    "approach": {
      "method": "SQL Injection",
      "tool": "Manual testing with Burp Suite",
      "technique": "Union-based SQL injection"
    },
    "execution": {
      "timestamp": "2025-01-20T14:30:00Z",
      "payload": "admin' UNION SELECT username,password FROM users--",
      "result": "SUCCESS",
      "evidenceCollected": [
        "screenshot_001.png",
        "http_traffic_capture.pcap",
        "database_schema.txt"
      ]
    },
    "postExploitation": {
      "accessLevel": "Database Administrator",
      "dataAccessed": "User credentials (hashed), customer records",
      "persistenceEstablished": false,
      "lateralMovement": false
    },
    "cleanup": {
      "actionsRequired": [
        "Remove test data inserted during exploitation",
        "Clear application logs of test payloads"
      ],
      "completed": true
    }
  }
}
```

#### 2.1.6 Post-Exploitation

**Purpose**: Assess the full impact of a successful compromise

**Activities**:
- Privilege escalation attempts
- Lateral movement within the network
- Persistence mechanism testing
- Data exfiltration simulation
- Impact assessment

**Example Post-Exploitation Report**:
```json
{
  "postExploitation": {
    "initialAccess": {
      "vector": "SQL Injection",
      "timestamp": "2025-01-20T14:30:00Z",
      "privileges": "www-data (low privilege)"
    },
    "privilegeEscalation": {
      "technique": "Sudo misconfiguration",
      "fromUser": "www-data",
      "toUser": "root",
      "success": true,
      "evidence": "sudo -l shows NOPASSWD for /usr/bin/vim"
    },
    "lateralMovement": {
      "attempted": true,
      "targetsScanned": ["192.168.1.10", "192.168.1.20"],
      "successfulCompromise": ["192.168.1.10"],
      "method": "SSH key reuse"
    },
    "persistence": {
      "established": true,
      "mechanism": "Cron job backdoor (simulation only)",
      "location": "/etc/cron.d/system-update",
      "removedAfterTest": true
    },
    "dataExfiltration": {
      "simulated": true,
      "sensitiveDataIdentified": [
        "Customer PII (500,000 records)",
        "Payment card data (encrypted)",
        "Internal credentials"
      ],
      "actualExfiltration": false,
      "note": "Data identified but not removed per RoE"
    }
  }
}
```

#### 2.1.7 Reporting

**Purpose**: Communicate findings with actionable remediation guidance

**Report Structure**:

1. **Executive Summary**
   - High-level overview for non-technical stakeholders
   - Risk rating and business impact
   - Key recommendations

2. **Technical Findings**
   - Detailed vulnerability descriptions
   - Exploitation procedures and evidence
   - CVSS scoring and risk classification

3. **Remediation Guidance**
   - Prioritized fix recommendations
   - Implementation steps
   - Verification procedures

4. **Compliance Mapping**
   - OWASP Top 10
   - MITRE ATT&CK
   - Industry-specific frameworks (PCI DSS, HIPAA, etc.)

**Example Executive Summary**:
```markdown
# Executive Summary

## Engagement Overview
A penetration test was conducted on Example Corporation's web application infrastructure from January 15-20, 2025. The assessment identified **3 critical**, **7 high**, and **12 medium** severity vulnerabilities.

## Risk Rating: CRITICAL

The overall security posture presents **critical risk** to the organization. The most severe finding (SQL Injection) allows unauthenticated attackers to:
- Bypass authentication controls
- Access sensitive customer data
- Potentially compromise backend systems

## Key Findings

| Severity | Count | Business Impact |
|----------|-------|-----------------|
| Critical | 3 | Data breach, authentication bypass, complete system compromise |
| High | 7 | Privilege escalation, sensitive information disclosure |
| Medium | 12 | Security misconfiguration, information leakage |

## Immediate Actions Required

1. **URGENT**: Patch SQL injection vulnerability in login form (VULN-2025-001)
2. **HIGH PRIORITY**: Implement multi-factor authentication
3. **HIGH PRIORITY**: Update outdated software components with known vulnerabilities

## Timeline for Remediation

- Critical vulnerabilities: **Within 7 days**
- High vulnerabilities: **Within 30 days**
- Medium vulnerabilities: **Within 90 days**

Retest recommended after critical and high vulnerabilities are remediated.
```

---

## 3. CVSS Scoring System

### 3.1 Common Vulnerability Scoring System (CVSS v3.1)

CVSS provides a standardized method for rating vulnerability severity.

**Base Score Metrics**:

1. **Attack Vector (AV)**
   - Network (N): 0.85
   - Adjacent (A): 0.62
   - Local (L): 0.55
   - Physical (P): 0.2

2. **Attack Complexity (AC)**
   - Low (L): 0.77
   - High (H): 0.44

3. **Privileges Required (PR)**
   - None (N): 0.85
   - Low (L): 0.62
   - High (H): 0.27

4. **User Interaction (UI)**
   - None (N): 0.85
   - Required (R): 0.62

5. **Scope (S)**
   - Unchanged (U): Impact remains in same security authority
   - Changed (C): Impact extends beyond the vulnerable component

6. **Confidentiality Impact (C)**
   - High (H): 0.56
   - Low (L): 0.22
   - None (N): 0

7. **Integrity Impact (I)**
   - High (H): 0.56
   - Low (L): 0.22
   - None (N): 0

8. **Availability Impact (A)**
   - High (H): 0.56
   - Low (L): 0.22
   - None (N): 0

**Severity Ratings**:
- 0.0: None
- 0.1-3.9: Low
- 4.0-6.9: Medium
- 7.0-8.9: High
- 9.0-10.0: Critical

---

## 4. MITRE ATT&CK Integration

### 4.1 Tactics and Techniques Mapping

Map penetration testing activities to MITRE ATT&CK framework:

**Reconnaissance**:
- T1595: Active Scanning
- T1592: Gather Victim Host Information
- T1589: Gather Victim Identity Information

**Initial Access**:
- T1190: Exploit Public-Facing Application
- T1133: External Remote Services
- T1566: Phishing

**Execution**:
- T1059: Command and Scripting Interpreter
- T1203: Exploitation for Client Execution

**Persistence**:
- T1098: Account Manipulation
- T1136: Create Account
- T1053: Scheduled Task/Job

**Privilege Escalation**:
- T1068: Exploitation for Privilege Escalation
- T1548: Abuse Elevation Control Mechanism

**Defense Evasion**:
- T1070: Indicator Removal on Host
- T1562: Impair Defenses

**Credential Access**:
- T1110: Brute Force
- T1003: OS Credential Dumping

**Discovery**:
- T1087: Account Discovery
- T1046: Network Service Scanning

**Lateral Movement**:
- T1021: Remote Services
- T1550: Use Alternate Authentication Material

**Collection**:
- T1005: Data from Local System
- T1039: Data from Network Shared Drive

**Exfiltration**:
- T1041: Exfiltration Over C2 Channel
- T1048: Exfiltration Over Alternative Protocol

**Impact**:
- T1485: Data Destruction
- T1486: Data Encrypted for Impact

---

## 5. Testing Methodologies

### 5.1 Web Application Testing (OWASP)

Follow OWASP Web Security Testing Guide (WSTG):

**Information Gathering**:
- Conduct search engine discovery
- Fingerprint web server
- Review webserver metafiles
- Enumerate applications on webserver
- Review webpage content for information leakage

**Configuration Management**:
- Test network infrastructure configuration
- Test application platform configuration
- Test file extensions handling
- Review old backup and unreferenced files
- Enumerate infrastructure and application admin interfaces

**Identity Management**:
- Test role definitions
- Test user registration process
- Test account provisioning process
- Test for weak or unenforced username policy

**Authentication**:
- Test for credentials transported over encrypted channel
- Test for default credentials
- Test for weak lock out mechanism
- Test for bypassing authentication schema
- Test for browser cache weaknesses
- Test for weak password policy
- Test for remember password functionality

**Authorization**:
- Test directory traversal file include
- Test for bypassing authorization schema
- Test for privilege escalation
- Test for insecure direct object references

**Session Management**:
- Test for session management schema
- Test for cookies attributes
- Test for session fixation
- Test for exposed session variables
- Test for Cross Site Request Forgery (CSRF)

### 5.2 Network Penetration Testing

**External Network Testing**:
1. Port and service enumeration
2. Vulnerability scanning
3. Banner grabbing and service identification
4. SSL/TLS configuration testing
5. Firewall and IDS/IPS detection
6. External service exploitation

**Internal Network Testing**:
1. Network segmentation assessment
2. Active Directory enumeration and testing
3. Windows/Linux privilege escalation
4. Lateral movement techniques
5. Internal phishing and social engineering
6. Wireless network security assessment

---

## 6. Reporting Standards

### 6.1 Vulnerability Classification

**Severity Levels**:

- **Critical**: Immediate threat to business operations, easy to exploit, high business impact
- **High**: Significant security risk, moderately difficult to exploit, substantial business impact
- **Medium**: Notable security concern, requires specific conditions, moderate business impact
- **Low**: Minor security issue, difficult to exploit, minimal business impact
- **Informational**: No immediate security risk, best practice recommendations

### 6.2 Required Report Sections

1. **Cover Page**: Engagement details, confidentiality notice
2. **Table of Contents**: Navigable report structure
3. **Executive Summary**: Non-technical overview
4. **Scope and Methodology**: What was tested and how
5. **Technical Findings**: Detailed vulnerability descriptions
6. **Risk Assessment**: CVSS scores and business impact
7. **Remediation Recommendations**: Prioritized fix guidance
8. **Appendices**: Raw data, tool outputs, screenshots

### 6.3 Evidence Requirements

For each vulnerability, provide:
- Detailed description
- Steps to reproduce
- Screenshot evidence
- HTTP request/response (for web vulnerabilities)
- Exploitation code/payload
- Business impact assessment
- Remediation guidance with verification steps

---

## 7. Philosophy and Ethics

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

### 7.1 Code of Conduct

Penetration testers must:
- Only test systems with explicit written authorization
- Respect scope boundaries and rules of engagement
- Protect confidentiality of client information
- Report vulnerabilities responsibly
- Minimize disruption to business operations
- Maintain professional integrity

### 7.2 Responsible Disclosure

When vulnerabilities are discovered:
1. Notify client immediately for critical findings
2. Provide secure communication channels
3. Allow reasonable time for remediation
4. Coordinate public disclosure if applicable
5. Avoid exposing client to additional risk

---

## 8. Integration and Interoperability

### 8.1 Tool Integration

WIA-SEC-019 integrates with:
- **Nmap**: Network scanning and reconnaissance
- **Metasploit Framework**: Exploitation and post-exploitation
- **Burp Suite**: Web application security testing
- **OWASP ZAP**: Automated web application scanning
- **Wireshark**: Network traffic analysis
- **Custom scripts**: Python, Bash, PowerShell automation

### 8.2 API Endpoints

```json
{
  "api": {
    "baseUrl": "https://api.wia.org/pentest/v1",
    "endpoints": {
      "createEngagement": "POST /engagements",
      "submitFinding": "POST /findings",
      "generateReport": "GET /reports/{engagementId}",
      "updateStatus": "PATCH /findings/{findingId}"
    },
    "authentication": "Bearer token (OAuth 2.0)"
  }
}
```

---

## 9. Compliance and Certification

### 9.1 Recognized Certifications

- OSCP (Offensive Security Certified Professional)
- CEH (Certified Ethical Hacker)
- GPEN (GIAC Penetration Tester)
- CREST Registered Penetration Tester
- PNPT (Practical Network Penetration Tester)

### 9.2 Regulatory Compliance

WIA-SEC-019 supports compliance with:
- PCI DSS Requirement 11.3 (Penetration Testing)
- HIPAA Security Rule
- SOC 2 Type II
- ISO 27001
- NIST Cybersecurity Framework

---

## 10. Conclusion

WIA-SEC-019 provides a comprehensive, ethical, and professional framework for conducting penetration testing. By following this standard, organizations can systematically identify and remediate security vulnerabilities, strengthening their overall security posture.

**Next Steps**:
- Phase 2: Advanced Red Team Operations
- Phase 3: Automated Continuous Testing
- Phase 4: Cloud and Container Security Testing

---

**Document Control**:
- Version: 1.0
- Last Updated: 2025-12-25
- Next Review: 2026-06-25
- Maintained by: WIA Security Standards Committee

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
