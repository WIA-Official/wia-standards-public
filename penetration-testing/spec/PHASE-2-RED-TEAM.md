# WIA-SEC-019: Penetration Testing — Phase 2 Specification

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-12-25
**Standard ID:** WIA-SEC-019
**Category:** Security (SEC)
**Color:** #8B5CF6

---

## Phase 2: Advanced Red Team Operations

### 2.1 Overview

Phase 2 extends penetration testing into sophisticated adversary simulation, modeling real-world attack campaigns with advanced persistent threat (APT) capabilities.

### 2.2 Red Team vs Penetration Testing

**Penetration Testing**:
- Time-boxed assessment
- Focused on finding vulnerabilities
- Known to defenders
- Compliance-driven

**Red Team Operations**:
- Long-term campaigns (weeks to months)
- Tests detection and response capabilities
- Covert operations
- Realistic threat simulation

### 2.3 Red Team Engagement Framework

```json
{
  "redTeamEngagement": {
    "id": "REDTEAM-2025-001",
    "objective": "Assess organization's ability to detect and respond to APT-style attacks",
    "duration": "90 days",
    "threatActorProfile": {
      "name": "Advanced Financial Threat Actor",
      "capability": "SOPHISTICATED",
      "tactics": ["Spear Phishing", "Custom Malware", "Living off the Land"],
      "goals": ["Data Exfiltration", "Persistence", "Lateral Movement"]
    },
    "rules": {
      "stealthLevel": "HIGH",
      "impactThreshold": "No production outages",
      "notification": "Critical escalation only",
      "cleanup": "Complete removal of all artifacts"
    },
    "phases": [
      {"phase": "Initial Compromise",   "techniques": ["T1566.001"], "timeline": "Week 1-2"},
      {"phase": "Establish Foothold",   "techniques": ["T1053", "T1547"], "timeline": "Week 2-3"},
      {"phase": "Privilege Escalation", "techniques": ["T1068"], "timeline": "Week 3-4"},
      {"phase": "Lateral Movement",     "techniques": ["T1021.001"], "timeline": "Week 4-8"},
      {"phase": "Objective Achievement","techniques": ["T1041"], "timeline": "Week 8-12"}
    ]
  }
}
```

### 2.4 Advanced Tactics, Techniques, and Procedures (TTPs)

#### 2.4.1 Custom Implant Capabilities

**Purpose**: Evade signature-based detection.

```json
{
  "implant": {
    "name": "WIA-RedTeam-Implant",
    "type": "Custom Backdoor",
    "capabilities": [
      "Remote Command Execution",
      "File Transfer",
      "Keylogging",
      "Screenshot Capture",
      "Credential Harvesting"
    ],
    "evasion": {
      "techniques": [
        "Process Injection",
        "API Hooking",
        "Anti-Debugging",
        "Sandbox Detection",
        "String Obfuscation"
      ]
    },
    "c2": {
      "protocol": "HTTPS",
      "encryption": "AES-256-GCM",
      "beaconing": "Random intervals (300-900 seconds)",
      "domainFronting": true
    }
  }
}
```

#### 2.4.2 Social Engineering Campaigns

```json
{
  "campaign": {
    "type": "Spear Phishing",
    "targets": ["Finance Department", "Executive Leadership", "IT Administrators"],
    "pretext": "Annual benefits enrollment update",
    "deliveryMethod": "Email with malicious attachment",
    "successMetricsLabels": [
      "emailsDelivered",
      "opened",
      "clicked",
      "executed",
      "shellsObtained"
    ]
  }
}
```

Physical engagements MAY include tailgating, USB drop, rogue wireless access points, and badge cloning. The Rules of Engagement document MUST explicitly authorise each physical technique with named scope.

#### 2.4.3 Living Off The Land (LOTL)

**Windows LOTL Tools**: PowerShell, WMI, PsExec, Certutil, Rundll32.
**Linux LOTL Tools**: Bash, curl/wget, cron, SSH, Docker.

**Example LOTL Reconnaissance** (Windows):

```bash
whoami /all
net user /domain
net group "Domain Admins" /domain
nltest /dclist:
```

### 2.5 Purple Team Operations

Collaborative security improvement combining red and blue teams. Each test cycle MUST capture:

1. **redAction** — what the offensive team executed.
2. **blueDetection** — what the defensive team observed.
3. **result** — DETECTED / PARTIALLY DETECTED / MISSED with timing.
4. **improvement** — concrete control or telemetry change.

### 2.6 MITRE ATT&CK Coverage Matrix

WIA-SEC-019 Phase 2 engagements MUST report coverage as a structured matrix mapping every technique exercised to a tactic column:

| Tactic | Sample techniques | Required artefacts |
|--------|-------------------|---------------------|
| Initial Access | T1566 (Phishing), T1190 (Public-facing exploit) | Phishing email, exploit PoC |
| Execution | T1059 (Cmd & Scripting), T1106 (Native API) | Command log |
| Persistence | T1547 (Boot/Logon Autostart), T1053 (Scheduled Task) | Registry/scheduler proof |
| Privilege Escalation | T1068 (Exploit), T1055 (Process Injection) | Beacon under elevated context |
| Defense Evasion | T1027 (Obfuscation), T1562 (Disable Defenses) | EDR telemetry diff |
| Credential Access | T1003 (OS Credential Dumping) | Hash dump, ticket files |
| Discovery | T1018 (Remote System), T1087 (Account) | Output captures |
| Lateral Movement | T1021 (Remote Services) | Pivot logs |
| Collection | T1005 (Local Data) | File listing |
| Command and Control | T1071 (Application Layer Protocol) | C2 capture |
| Exfiltration | T1041 (C2 channel), T1567 (Web Service) | Bytes-out diff |
| Impact (out-of-scope by default) | T1486 (Encryption for Impact) | Forbidden unless ROE allows |

### 2.7 Rules of Engagement (RoE) Schema

```json
{
  "roe": {
    "engagementId": "REDTEAM-2025-001",
    "stakeholders": [
      {"name": "Security VP", "role": "Approver"},
      {"name": "CISO", "role": "Reviewer"},
      {"name": "Red Team Lead", "role": "Operator"}
    ],
    "scope": {
      "ipRanges": ["10.0.0.0/8"],
      "applications": ["payments.example.com"],
      "exclusions": ["production database write paths", "PII exfiltration of real records"]
    },
    "timeWindow": {"from": "2025-12-25T00:00:00Z", "to": "2026-03-25T23:59:59Z"},
    "stopWord": "NEPTUNE",
    "evidenceRetention": "365 days, encrypted at rest"
  }
}
```

The RoE MUST be signed by the customer (digital signature or wet ink) before any active operation begins. Loss of authorization MUST trigger immediate teardown using the `stopWord`.

### 2.8 Reporting Requirements

Each Phase 2 engagement MUST produce:

- **Executive Summary** — business impact, attack chain narrative.
- **Technical Findings** — every technique with prerequisites, observed result, and remediation.
- **Detection Gap Map** — list of techniques not detected by current tooling.
- **Remediation Backlog** — Jira-style stories with priority and owner.
- **Re-test Plan** — schedule for verification within 90 days.

### 2.9 Operator Safety & Containment

Every red-team operation MUST plan for blast-radius containment:

- **Network kill-switch** — a documented procedure to revoke C2 connectivity within 5 minutes.
- **Identity revocation** — the ability to invalidate any operator account or API key used during the exercise.
- **Evidence sanctity** — captured artefacts MUST be encrypted, hashed (SHA-256), and stored in a tamper-evident vault.
- **Daily situation report** — a private channel with the customer's incident commander summarising the last 24 h of activity.
- **Operational pause** — the customer MUST be able to halt activity instantly with the agreed stop word. Acknowledge within 60 seconds.

### 2.10 Sample Engagement Schedule

```
Week 0   Kickoff, RoE signing, scope freeze
Week 1   Passive recon, OSINT
Week 2   Initial access attempts (phishing, exposed services)
Week 3-5 Foothold, persistence, escalation
Week 6-8 Lateral movement, sensitive-data discovery
Week 9   Objective achievement (controlled exfiltration to operator vault)
Week 10  Detection-gap workshop with blue team
Week 11  Cleanup verification, RoE closeout
Week 12  Final report, executive readout, remediation backlog handoff
```

### 2.11 Normative References

- ISO/IEC 27001:2022 — Information Security Management
- ISO/IEC 27037:2012 — Identification, collection, acquisition and preservation of digital evidence
- NIST SP 800-115 — Technical Guide to Information Security Testing
- MITRE ATT&CK Enterprise Matrix — Tactics & Techniques
- OWASP Application Security Verification Standard (ASVS) 4.0
- IETF RFC 8259 — JSON Data Interchange Format

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
