# WIA-SEC-020: Security Incident Response - Phase 1 Core Specification

**Standard ID:** WIA-SEC-020
**Version:** 1.0
**Status:** ACTIVE
**Last Updated:** 2025-12-25

---

## 1. Introduction

### 1.1 Purpose

This specification defines the core foundation for security incident response capabilities based on NIST SP 800-61 Rev. 2 and ISO/IEC 27035. It establishes the fundamental requirements for Computer Security Incident Response Teams (CSIRT) to effectively detect, analyze, contain, eradicate, recover from, and learn from security incidents.

### 1.2 Scope

Phase 1 covers:
- Incident response framework and lifecycle
- CSIRT organizational structure
- Incident classification and severity rating
- Detection and analysis procedures
- Initial containment strategies
- Evidence collection and preservation
- Communication protocols
- Basic forensic procedures

### 1.3 References

- NIST SP 800-61 Rev. 2: Computer Security Incident Handling Guide
- ISO/IEC 27035-1:2016: Information security incident management
- FIRST CSIRT Services Framework v2.1
- MITRE ATT&CK Framework
- SANS Incident Handler's Handbook

---

## 2. Incident Response Lifecycle

### 2.1 Six-Phase Model

The WIA-SEC-020 incident response lifecycle follows the NIST framework with six distinct phases:

#### Phase 1: Preparation
**Objective:** Establish and maintain incident response capability

**Requirements:**
- Deploy and configure monitoring systems (SIEM, IDS/IPS, EDR)
- Develop incident response plan and playbooks
- Establish CSIRT team with defined roles
- Conduct regular training and tabletop exercises
- Maintain incident response toolkit and jump bags
- Create communication templates and escalation paths
- Document asset inventory and critical systems
- Establish baseline normal behavior

**Deliverables:**
- Incident Response Plan (IRP)
- CSIRT Charter and contact roster
- Response playbooks for common scenarios
- Communication templates
- Asset criticality matrix

#### Phase 2: Detection and Analysis
**Objective:** Identify and validate potential security incidents

**Requirements:**
- Monitor security alerts from multiple sources
- Analyze logs, network traffic, and system behavior
- Correlate events across different security tools
- Determine if incident is actual or false positive
- Classify incident type and severity
- Document initial findings
- Assign incident tracking ID
- Notify appropriate stakeholders

**Detection Sources:**
- SIEM alerts and correlation rules
- IDS/IPS signatures and anomaly detection
- EDR behavioral analytics
- User reports (help desk, email)
- Antivirus/anti-malware alerts
- DLP (Data Loss Prevention) alerts
- Threat intelligence feeds
- Log analysis and anomaly detection

**Incident Classification:**
```
Category            | Examples
--------------------|------------------------------------------
Malware            | Virus, worm, trojan, ransomware, rootkit
Unauthorized Access| Brute force, credential theft, privilege escalation
Data Breach        | Exfiltration, exposure, unauthorized disclosure
Denial of Service  | DDoS, resource exhaustion, flooding
Web Attack         | SQL injection, XSS, RCE, defacement
Insider Threat     | Sabotage, fraud, IP theft
Phishing           | Email phishing, spear phishing, whaling
Physical Security  | Unauthorized physical access, theft
```

**Severity Ratings:**
```
Level    | Impact                           | Response Time
---------|----------------------------------|---------------
CRITICAL | Mission-critical systems down    | < 15 minutes
         | Active data exfiltration         |
         | Ransomware encryption in progress|
---------|----------------------------------|---------------
HIGH     | Significant data breach          | < 1 hour
         | Compromise of critical systems   |
         | Advanced persistent threat (APT) |
---------|----------------------------------|---------------
MEDIUM   | Limited unauthorized access      | < 4 hours
         | Malware contained to single host |
         | Suspicious activity detected     |
---------|----------------------------------|---------------
LOW      | Policy violation                 | < 24 hours
         | Attempted attack blocked         |
         | Reconnaissance activity          |
```

#### Phase 3: Containment
**Objective:** Limit the damage and prevent incident from spreading

**Short-term Containment:**
- Isolate affected systems from network
- Disable compromised accounts
- Block malicious IP addresses/domains
- Apply emergency patches or workarounds
- Preserve evidence for forensics
- Maintain business continuity

**Long-term Containment:**
- Rebuild compromised systems
- Apply security patches systematically
- Implement enhanced monitoring
- Deploy compensating controls
- Update firewall rules
- Strengthen authentication requirements

**Containment Decision Criteria:**
```python
def determine_containment_strategy(incident):
    """
    Determine appropriate containment strategy based on incident characteristics
    """
    if incident.severity == "CRITICAL":
        if incident.type == "RANSOMWARE":
            return {
                "action": "IMMEDIATE_ISOLATION",
                "steps": [
                    "Disconnect network cables",
                    "Disable wireless",
                    "Power off if encryption active",
                    "Preserve memory if possible"
                ],
                "timeline": "< 5 minutes"
            }
        elif incident.type == "DATA_EXFILTRATION":
            return {
                "action": "NETWORK_SEGMENTATION",
                "steps": [
                    "Block outbound traffic from affected systems",
                    "Capture network traffic for analysis",
                    "Identify exfiltration channels",
                    "Preserve evidence"
                ],
                "timeline": "< 15 minutes"
            }

    elif incident.severity == "HIGH":
        return {
            "action": "CONTROLLED_CONTAINMENT",
            "steps": [
                "Isolate affected network segment",
                "Monitor for lateral movement",
                "Collect forensic evidence",
                "Apply temporary patches"
            ],
            "timeline": "< 1 hour"
        }

    # Default strategy
    return {
        "action": "MONITOR_AND_CONTAIN",
        "steps": [
            "Enhanced monitoring",
            "Limit privileges",
            "Prepare for isolation if escalates"
        ],
        "timeline": "< 4 hours"
    }
```

#### Phase 4: Eradication
**Objective:** Remove the threat from the environment

**Requirements:**
- Identify and remove all traces of malware
- Close attack vectors and vulnerabilities
- Disable compromised accounts permanently
- Apply security patches and updates
- Strengthen security configurations
- Validate threat removal

**Eradication Procedures:**
1. **Malware Removal:**
   - Use updated anti-malware tools
   - Perform offline scans
   - Remove persistence mechanisms (registry keys, scheduled tasks)
   - Clean infected files or restore from backup

2. **Account Security:**
   - Disable compromised accounts
   - Force password resets for affected users
   - Review and revoke unauthorized access grants
   - Implement stronger authentication (MFA)

3. **Vulnerability Remediation:**
   - Patch exploited vulnerabilities
   - Fix misconfigurations
   - Remove unnecessary services
   - Harden system configurations

4. **Verification:**
   - Scan systems with multiple AV engines
   - Monitor for signs of re-infection
   - Validate security controls
   - Confirm threat indicators absent

#### Phase 5: Recovery
**Objective:** Restore systems to normal operations

**Requirements:**
- Restore systems from clean backups
- Rebuild compromised systems from scratch
- Reset all credentials and cryptographic keys
- Implement enhanced monitoring
- Gradually restore services
- Monitor for anomalies during recovery
- Validate business functionality

**Recovery Procedures:**
```
1. Backup Validation
   - Verify backup integrity
   - Confirm backups are clean (pre-infection)
   - Test restoration process

2. System Restoration
   - Restore from verified clean backups
   - OR rebuild from known-good images
   - Apply all security patches
   - Reconfigure security settings

3. Credential Reset
   - Reset all passwords
   - Regenerate SSH keys
   - Rotate API keys and tokens
   - Update certificates if compromised

4. Monitoring Enhancement
   - Deploy additional logging
   - Increase SIEM alert sensitivity
   - Monitor for re-infection indicators
   - Watch for similar attack patterns

5. Phased Return to Production
   - Start with non-critical systems
   - Monitor closely for 48-72 hours
   - Gradually restore full functionality
   - Maintain incident response readiness
```

#### Phase 6: Post-Incident Activity
**Objective:** Learn from the incident and improve defenses

**Requirements:**
- Conduct lessons learned meeting within 2 weeks
- Document complete incident timeline
- Analyze root cause
- Identify gaps in detection and response
- Update incident response procedures
- Implement preventive measures
- Share threat intelligence (anonymized)
- Update security awareness training

**Lessons Learned Report Template:**
```markdown
# Incident Post-Mortem Report

## Incident Summary
- Incident ID: [INC-YYYY-NNNNNN]
- Date Range: [Start] - [End]
- Severity: [CRITICAL/HIGH/MEDIUM/LOW]
- Category: [Type]

## Timeline
- Detection Time:
- Containment Time:
- Eradication Time:
- Recovery Time:
- Total Duration:

## Root Cause Analysis
[What was the initial entry vector? What vulnerabilities were exploited?]

## Impact Assessment
- Systems Affected:
- Data Compromised:
- Downtime:
- Financial Impact:

## Response Effectiveness
### What Went Well
- [Positive aspects of response]

### What Needs Improvement
- [Gaps and weaknesses identified]

## Recommendations
1. [Preventive measures]
2. [Detection improvements]
3. [Response enhancements]
4. [Training needs]

## Action Items
| Action | Owner | Deadline | Status |
|--------|-------|----------|--------|
|        |       |          |        |
```

---

## 3. CSIRT Organization

### 3.1 Team Structure

**Core Roles:**

1. **CSIRT Manager**
   - Oversees incident response operations
   - Coordinates with executive management
   - Makes strategic decisions
   - Manages resources and budget

2. **Incident Response Lead**
   - Leads active incident response
   - Coordinates response activities
   - Assigns tasks to team members
   - Maintains incident timeline

3. **Security Analysts (Tier 1)**
   - Monitor security alerts
   - Perform initial triage
   - Escalate confirmed incidents
   - Document findings

4. **Incident Handlers (Tier 2)**
   - Investigate incidents
   - Perform containment actions
   - Coordinate with system owners
   - Execute response playbooks

5. **Forensic Investigators (Tier 3)**
   - Conduct deep forensic analysis
   - Preserve and analyze evidence
   - Perform malware analysis
   - Support legal proceedings

6. **Communications Coordinator**
   - Manage internal/external communications
   - Prepare incident notifications
   - Interface with media (if needed)
   - Coordinate with legal/PR teams

### 3.2 On-Call Requirements

**24/7 Coverage:**
- Primary on-call responder
- Secondary on-call responder
- Escalation to CSIRT Lead if needed
- Response time SLAs by severity

**Escalation Matrix:**
```
Severity   | Initial Response | Escalation After | Notify Management
-----------|------------------|------------------|-------------------
CRITICAL   | Immediate        | 15 minutes       | Immediately
HIGH       | < 30 minutes     | 1 hour           | Within 1 hour
MEDIUM     | < 2 hours        | 4 hours          | Within 4 hours
LOW        | < 8 hours        | 24 hours         | Daily summary
```

### 3.3 Training Requirements

**Annual Requirements:**
- 40 hours incident response training
- 4 tabletop exercises
- 2 red team exercises
- Industry certifications (GCIH, GCIA, CISSP, etc.)
- Continuous learning through CTFs and labs

---

## 4. Evidence Collection and Preservation

### 4.1 Order of Volatility

Collect evidence in order from most to least volatile:

1. **CPU Registers, Cache** (nanoseconds)
2. **RAM/Memory** (minutes without power)
3. **Network Connections** (seconds to hours)
4. **Running Processes** (seconds to hours)
5. **Disk/Storage** (months to years)
6. **Backups and Archives** (years)
7. **Physical Configuration** (permanent)

### 4.2 Evidence Collection Procedures

**Memory Acquisition:**
```bash
# Linux memory dump
sudo insmod lime.ko "path=/mnt/evidence/memory.lime format=lime"

# Windows memory dump (Administrator)
winpmem.exe -o memory.raw

# Calculate hash immediately
sha256sum memory.lime > memory.lime.sha256
```

**Disk Imaging:**
```bash
# Create forensic disk image
dd if=/dev/sda of=/mnt/evidence/disk.img bs=64K conv=noerror,sync status=progress

# Calculate hash
sha256sum disk.img > disk.img.sha256

# Verify image
dd if=/mnt/evidence/disk.img | sha256sum
```

**Network Traffic Capture:**
```bash
# Capture traffic for analysis
tcpdump -i eth0 -w /mnt/evidence/traffic.pcap -s 65535

# Capture for specific timeframe (10 minutes)
timeout 600 tcpdump -i eth0 -w traffic.pcap
```

### 4.3 Chain of Custody

**Evidence Record Template:**
```json
{
  "evidence_id": "EVD-2025-001234-001",
  "incident_id": "INC-2025-001234",
  "type": "DISK_IMAGE",
  "description": "Forensic disk image of web server",
  "source_system": {
    "hostname": "web-prod-01",
    "ip_address": "10.0.1.50",
    "location": "Datacenter A, Rack 23, Server 5"
  },
  "collected_by": "John Smith",
  "collection_timestamp": "2025-12-25T15:00:00Z",
  "collection_method": "dd command with SHA-256 hashing",
  "hash": {
    "algorithm": "SHA-256",
    "value": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
  },
  "storage_location": "Evidence Locker A, Shelf 3",
  "chain_of_custody": [
    {
      "timestamp": "2025-12-25T15:00:00Z",
      "custodian": "John Smith",
      "action": "COLLECTED",
      "location": "Datacenter A",
      "signature": "J.Smith"
    },
    {
      "timestamp": "2025-12-25T15:30:00Z",
      "custodian": "Evidence Vault",
      "action": "STORED",
      "location": "Secure Storage Facility",
      "signature": "Auto-logged"
    }
  ]
}
```

---

## 5. Communication Protocols

### 5.1 Internal Communication

**Status Updates:**
- Hourly updates for CRITICAL incidents
- Every 4 hours for HIGH incidents
- Daily for MEDIUM/LOW incidents

**Communication Channels:**
- Secure messaging (Signal, encrypted email)
- Dedicated incident response Slack/Teams channel
- Conference bridge for major incidents
- War room for critical incidents

### 5.2 External Communication

**Stakeholder Notifications:**
1. **Executive Management**
   - Critical incidents: immediate
   - High incidents: within 1 hour
   - Summary reports: daily

2. **Legal/Compliance**
   - Data breach: immediate
   - Regulatory impact: within 2 hours
   - Evidence preservation needs: immediate

3. **Public Relations**
   - Public disclosure needed: coordinate timing
   - Media inquiries: route through PR
   - Approved statements only

4. **Law Enforcement**
   - Criminal activity: coordinate with legal
   - Jurisdiction considerations
   - Evidence preservation requirements

5. **Customers/Partners**
   - If their data affected: per SLA/regulations
   - Coordinate with legal on timing/content
   - Provide actionable guidance

### 5.3 Notification Requirements

**Regulatory Obligations:**
- GDPR: 72 hours for personal data breach
- HIPAA: 60 days for health data breach
- PCI-DSS: Immediately for card data breach
- State breach laws: varies by jurisdiction

---

## 6. Incident Tracking and Metrics

### 6.1 Key Performance Indicators (KPIs)

**Detection Metrics:**
- Mean Time to Detect (MTTD)
- False Positive Rate
- Detection Source Distribution

**Response Metrics:**
- Mean Time to Respond (MTTR)
- Mean Time to Contain (MTTC)
- Mean Time to Recover (MTTR)
- Incident Escalation Rate

**Quality Metrics:**
- Incident Recurrence Rate
- Post-incident Review Completion Rate
- Action Item Completion Rate
- Training Completion Rate

### 6.2 Incident Tracking System

**Required Fields:**
```
- Incident ID (unique)
- Date/Time Opened
- Severity Level
- Category/Type
- Status (New, Investigating, Contained, Eradicated, Closed)
- Assigned To
- Affected Systems
- Impact Assessment
- Timeline of Events
- Actions Taken
- Evidence Collected
- Root Cause
- Resolution
- Lessons Learned
- Related Incidents
```

---

## 7. Compliance and Legal Considerations

### 7.1 Legal Requirements

**Evidence Admissibility:**
- Follow chain of custody procedures
- Use forensically sound tools
- Document all actions
- Avoid contaminating evidence
- Work with legal counsel

**Attorney-Client Privilege:**
- Involve legal counsel early
- Mark communications as privileged where appropriate
- Understand when privilege applies

### 7.2 Regulatory Compliance

**Data Protection:**
- GDPR, CCPA, HIPAA compliance
- Data breach notification requirements
- Data subject rights (access, deletion)

**Industry Standards:**
- PCI-DSS incident response requirements
- SOC 2 security incident procedures
- ISO 27001 incident management

---

## 8. Implementation Checklist

### Phase 1 Core Implementation:

- [ ] Establish CSIRT team with defined roles
- [ ] Deploy SIEM and configure correlation rules
- [ ] Create incident response plan document
- [ ] Develop response playbooks for top 5 scenarios
- [ ] Implement incident tracking system
- [ ] Configure evidence storage and chain of custody
- [ ] Establish communication templates and protocols
- [ ] Conduct initial CSIRT training
- [ ] Perform first tabletop exercise
- [ ] Document asset inventory and criticality
- [ ] Create baseline monitoring and alerting
- [ ] Establish 24/7 on-call rotation
- [ ] Test evidence collection tools
- [ ] Document escalation procedures
- [ ] Establish relationships with external resources (FBI, CERT, etc.)

---

## 9. Appendix

### 9.1 Incident Response Toolkit

**Software Tools:**
- SIEM: Splunk, ELK, QRadar, Sentinel
- EDR: CrowdStrike, Carbon Black, SentinelOne
- Forensics: EnCase, FTK, Autopsy, Volatility
- Network Analysis: Wireshark, tcpdump, Zeek
- Malware Analysis: IDA Pro, Ghidra, Cuckoo Sandbox
- Memory Analysis: Volatility, Rekall
- Incident Tracking: TheHive, RTIR, ServiceNow

**Hardware:**
- Forensic workstations
- Write blockers
- External storage for evidence
- Network TAPs
- Faraday bags for mobile devices

### 9.2 Reference Documents

- [NIST SP 800-61 Rev. 2](https://nvlpubs.nist.gov/nistpubs/SpecialPublications/NIST.SP.800-61r2.pdf)
- [ISO/IEC 27035](https://www.iso.org/standard/78973.html)
- [FIRST CSIRT Framework](https://www.first.org/standards/frameworks/csirts/)
- [MITRE ATT&CK](https://attack.mitre.org/)
- [SANS Incident Handler's Handbook](https://www.sans.org/reading-room/whitepapers/incident/incident-handlers-handbook-33901)

---

**Document Control:**
- Version: 1.0
- Status: ACTIVE
- Last Review: 2025-12-25
- Next Review: 2026-06-25
- Owner: WIA Security Standards Committee

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
