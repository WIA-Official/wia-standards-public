# WIA-SEC-020: Security Incident Response - Glossary

**Standard ID:** WIA-SEC-020
**Version:** 1.0
**Last Updated:** 2025-12-25

---

## A

**Advanced Persistent Threat (APT)**
A prolonged and targeted cyberattack in which an intruder gains access to a network and remains undetected for an extended period. APTs are typically carried out by well-funded, sophisticated threat actors such as nation-states or organized crime groups.

**Anomaly Detection**
The identification of patterns or events that do not conform to expected behavior. In cybersecurity, this involves detecting deviations from normal system or user activity that may indicate a security incident.

**Attack Surface**
The sum of all possible points where an unauthorized user could try to enter data to or extract data from an environment. Includes network interfaces, applications, APIs, and human factors.

**Attack Vector**
The path or means by which an attacker gains access to a computer or network server to deliver a malicious payload or outcome. Examples include email attachments, web pages, and USB drives.

---

## B

**Baseline**
A reference point used for comparison. In security, it represents normal system behavior, configuration, or performance metrics used to identify anomalies.

**Beacon/Beaconing**
Regular communication between malware on a compromised system and a command-and-control (C2) server. Often occurs at consistent intervals, making it detectable through network traffic analysis.

**Behavioral Analytics**
The use of statistical and machine learning techniques to identify anomalous behavior patterns that may indicate security threats. Includes User and Entity Behavior Analytics (UEBA).

**Blockchain Anchor**
A cryptographic hash of data stored on a blockchain to provide tamper-evident proof of existence at a specific time. Used in evidence preservation to prove integrity.

**Blue Team**
The defensive security team responsible for protecting an organization's information systems by implementing security measures and responding to incidents. Contrasts with Red Team (offensive).

**Breach**
An incident where data, applications, networks, or devices are accessed or used in an unauthorized manner, potentially resulting in the disclosure of information, modification of data, or denial of service.

---

## C

**CEF (Common Event Format)**
A standardized log format that enables different security devices to share and correlate event data. Widely used in SIEM systems for log aggregation.

**Certificate Authority (CA)**
A trusted entity that issues digital certificates used to verify the identity of entities and enable secure communications.

**Chain of Custody**
The chronological documentation showing the seizure, custody, control, transfer, analysis, and disposition of evidence. Critical for legal admissibility of digital evidence.

**Command and Control (C2/C&C)**
Infrastructure used by attackers to maintain communication with compromised systems. Allows remote control of malware and exfiltration of stolen data.

**Computer Security Incident Response Team (CSIRT)**
A team of IT professionals responsible for responding to computer security incidents. Also known as CERT (Computer Emergency Response Team).

**Containment**
Actions taken to limit the scope and magnitude of a security incident. Includes short-term actions (isolation) and long-term actions (system rebuilding).

**Correlation**
The process of linking multiple security events that may be related to the same incident or attack campaign. Critical function of SIEM systems.

**Credential Harvesting**
The act of collecting authentication credentials (usernames, passwords) through various means such as phishing, keylogging, or database breaches.

**CSIRT (Computer Security Incident Response Team)**
See "Computer Security Incident Response Team" above.

---

## D

**Data Breach**
An incident involving unauthorized access to or disclosure of sensitive, protected, or confidential data. May trigger legal notification requirements under various regulations.

**Data Exfiltration**
The unauthorized transfer of data from a computer or network. Often the ultimate goal of advanced persistent threats and insider attacks.

**DDoS (Distributed Denial of Service)**
A cyberattack that makes a system or network resource unavailable by overwhelming it with traffic from multiple sources. Differs from DoS which uses a single source.

**Detection**
The process of identifying potential security incidents through monitoring, analysis of alerts, and investigation of anomalies.

**Digital Forensics**
The process of collecting, preserving, analyzing, and presenting digital evidence in a manner that is legally admissible. Includes computer, network, mobile, and memory forensics.

**Disk Image**
A bit-by-bit copy of a storage device created for forensic analysis. Preserves all data including deleted files and slack space.

**DNS Tunneling**
A method of encoding data within DNS queries and responses to bypass security controls or exfiltrate data. Often used in command-and-control communications.

---

## E

**EDR (Endpoint Detection and Response)**
Security technology that continuously monitors and responds to threats on endpoints (computers, servers, mobile devices). Provides visibility into endpoint activities.

**Eradication**
The process of removing the threat from the environment, including malware removal, closing vulnerabilities, and eliminating attacker access.

**Escalation**
The process of elevating an incident to higher levels of management or expertise based on severity, impact, or inability to resolve at current level.

**Evidence**
Information or objects that may be used to prove or disprove facts related to an incident. In digital forensics, includes files, logs, network traffic, and memory dumps.

**Exploit**
Code or technique that takes advantage of a vulnerability to gain unauthorized access or cause unintended behavior in a system or application.

---

## F

**False Positive**
A security alert that incorrectly identifies benign activity as malicious. High false positive rates reduce the effectiveness of security monitoring.

**FIRST (Forum of Incident Response and Security Teams)**
Global organization bringing together CSIRTs to cooperate on incident prevention and response. Publishes CSIRT Services Framework.

**Forensic Image**
See "Disk Image" - a complete copy of a storage medium created using forensically sound methods that preserve evidence integrity.

**Forensic Soundness**
The principle that digital evidence collection and analysis methods must preserve the integrity of evidence and be defensible in court.

---

## G

**GDPR (General Data Protection Regulation)**
European Union regulation governing data protection and privacy. Requires notification of data breaches to supervisory authorities within 72 hours.

**GIAC (Global Information Assurance Certification)**
Certifications offered by SANS Institute for information security professionals, including GCIH (Incident Handler) and GCFA (Forensic Analyst).

---

## H

**Hash/Hashing**
A cryptographic function that produces a fixed-size string (hash value) from input data. Used to verify data integrity. Common algorithms: MD5, SHA-1, SHA-256.

**HIPAA (Health Insurance Portability and Accountability Act)**
US law requiring protection of health information. Breach notification must occur within 60 days of discovery.

**Honeypot**
A decoy system designed to attract attackers and study their techniques. Used for threat intelligence and early warning.

---

## I

**IDS (Intrusion Detection System)**
Security technology that monitors network or system activities for malicious activities or policy violations. Generates alerts but does not block.

**Incident**
An event or series of events that adversely affects the confidentiality, integrity, or availability of an information system or the information it processes, stores, or transmits.

**Incident Handler**
A security professional responsible for investigating and responding to security incidents. Requires technical skills and knowledge of response procedures.

**Incident Response Plan (IRP)**
A documented set of procedures for detecting, responding to, and recovering from security incidents. Should be tested regularly through exercises.

**Indicator of Compromise (IOC)**
Observable artifact or evidence that suggests a system has been compromised. Examples: suspicious IP addresses, file hashes, registry keys, domain names.

**IPS (Intrusion Prevention System)**
Security technology that monitors network traffic and can automatically block detected threats. Active defense compared to passive IDS.

**ISO/IEC 27035**
International standard for information security incident management. Provides guidelines for incident response processes.

**Isolation**
The act of separating a compromised system from the network to prevent spread of the incident. Key containment strategy.

---

## J

**Jump Bag/Kit**
A pre-assembled collection of tools and equipment needed for incident response. Should be portable and readily available for rapid deployment.

---

## K

**Kill Chain**
A model describing the stages of a cyberattack, from reconnaissance to data exfiltration. The Cyber Kill Chain was developed by Lockheed Martin. MITRE ATT&CK provides a more detailed framework.

**KPI (Key Performance Indicator)**
Measurable value demonstrating effectiveness of security operations. Examples: Mean Time to Detect (MTTD), Mean Time to Respond (MTTR).

---

## L

**Lateral Movement**
Technique used by attackers to progressively move through a network searching for target data and systems. Often follows initial compromise.

**Lessons Learned**
Post-incident review process to identify what went well, what needs improvement, and actions to prevent recurrence. Critical for continuous improvement.

**Log Aggregation**
The collection and centralization of log data from multiple sources for analysis and correlation. Primary function of SIEM systems.

---

## M

**Malware**
Malicious software designed to damage, disrupt, or gain unauthorized access to systems. Types include viruses, worms, trojans, ransomware, spyware.

**Mean Time to Detect (MTTD)**
Average time between when an incident occurs and when it is detected. Lower values indicate more effective detection capabilities.

**Mean Time to Respond (MTTR)**
Average time between detection of an incident and beginning of response actions. Key performance metric for incident response teams.

**Memory Forensics**
Analysis of volatile memory (RAM) to identify malware, extract passwords, recover deleted data, and understand system state at time of incident.

**MITRE ATT&CK**
Knowledge base of adversary tactics, techniques, and procedures based on real-world observations. Used for threat modeling and gap analysis.

**MTTC (Mean Time to Contain)**
Average time from detection to successful containment of an incident. Indicates effectiveness of containment procedures.

---

## N

**NIST SP 800-61**
National Institute of Standards and Technology Special Publication providing computer security incident handling guide. Industry-standard framework.

**Network Forensics**
Monitoring and analysis of network traffic to gather evidence, detect intrusions, or investigate security incidents.

---

## O

**Order of Volatility**
Principle in digital forensics that evidence should be collected from most volatile (quickly changing/lost) to least volatile sources.

---

## P

**Patient Zero**
The initial system or user compromised in an incident. Identifying patient zero helps understand attack vector and scope.

**PCI-DSS (Payment Card Industry Data Security Standard)**
Security standard for organizations handling credit card information. Requires immediate notification of breaches.

**Persistence**
Techniques used by attackers to maintain access to a compromised system across reboots and security measures. Examples: registry keys, scheduled tasks.

**Phishing**
Social engineering attack using fraudulent emails or messages to trick recipients into revealing sensitive information or installing malware.

**Playbook**
Pre-defined set of procedures for responding to specific types of incidents. Enables faster, more consistent response.

**Privilege Escalation**
Exploitation of vulnerabilities to gain elevated access rights on a system. Common step in attack progression.

---

## Q

**Quarantine**
Isolation of potentially infected files or systems to prevent spread while preserving them for analysis.

---

## R

**Ransomware**
Malware that encrypts files or locks systems and demands payment for restoration. Major threat requiring rapid response.

**Recovery**
The process of restoring systems to normal operations following an incident. Includes restoration from backups and system rebuilding.

**Recovery Point Objective (RPO)**
Maximum acceptable amount of data loss measured in time. Example: RPO of 1 hour means you can lose up to 1 hour of data.

**Recovery Time Objective (RTO)**
Target time to restore a system or service after an incident. Drives recovery prioritization and resource allocation.

**Red Team**
Offensive security team that simulates attacks to test defenses. Complements Blue Team (defensive) activities.

**Remediation**
Actions taken to address vulnerabilities or security gaps identified during or after an incident.

**Rootkit**
Malware designed to hide its presence and activities from users and security software. Often operates at kernel level.

---

## S

**SANS (SysAdmin, Audit, Network, and Security)**
Organization providing cybersecurity training and certifications. Publisher of Incident Handler's Handbook.

**Security Information and Event Management (SIEM)**
Technology that aggregates and analyzes security data from multiple sources to detect incidents and support compliance.

**Security Orchestration, Automation and Response (SOAR)**
Technology that coordinates security tools and automates response workflows to improve incident response speed and consistency.

**Severity**
Classification of an incident's potential impact on business operations. Typical levels: Critical, High, Medium, Low.

**STIX (Structured Threat Information Expression)**
Standardized language for representing threat intelligence information. Used with TAXII for sharing indicators.

**Super Timeline**
Comprehensive timeline combining events from multiple sources (filesystem, logs, registry, etc.) to understand incident sequence.

---

## T

**Tactics, Techniques, and Procedures (TTPs)**
Patterns of activities or methods associated with specific threat actors. Used to attribute attacks and predict future behavior.

**TAXII (Trusted Automated Exchange of Intelligence Information)**
Protocol for exchanging cyber threat intelligence in a standardized format (typically STIX).

**Threat Hunting**
Proactive search for threats that have evaded automated detection systems. Uses hypothesis-driven approach based on threat intelligence.

**Threat Intelligence**
Information about threats and threat actors used to inform security decisions. Sources include commercial feeds, ISACs, and open source.

**Triage**
Initial assessment and prioritization of security alerts and incidents based on severity and potential impact.

---

## U

**UEBA (User and Entity Behavior Analytics)**
Security technology that uses machine learning to detect anomalous behavior by users and entities (devices, applications, servers).

---

## V

**VERIS (Vocabulary for Event Recording and Incident Sharing)**
Framework for describing security incidents in a structured manner. Facilitates sharing and analysis of incident data.

**Volatility**
1. Property of data that determines how quickly it is lost (high volatility = lost quickly)
2. Popular open-source memory forensics framework

**Vulnerability**
Weakness in a system, application, or process that can be exploited by a threat actor to gain unauthorized access or cause harm.

---

## W

**War Room**
Physical or virtual space where incident response team coordinates during major incidents. Facilitates communication and decision-making.

**Whaling**
Targeted phishing attack directed at senior executives or other high-profile targets. Often more sophisticated than general phishing.

**Write Blocker**
Hardware or software tool that prevents modification of storage media during forensic acquisition. Ensures evidence integrity.

---

## X

**XDR (Extended Detection and Response)**
Evolution of EDR that integrates data from multiple security products (endpoints, network, cloud) for comprehensive threat detection and response.

---

## Y

**YARA**
Tool used to identify and classify malware based on pattern matching rules. Essential for malware analysis and threat hunting.

---

## Z

**Zero-Day**
Previously unknown vulnerability or exploit. "Zero-day" refers to developers having zero days to fix it before it's exploited.

**Zero Trust**
Security model based on the principle of "never trust, always verify." Requires authentication and authorization for every access request regardless of location.

---

## Acronyms Quick Reference

| Acronym | Full Term |
|---------|-----------|
| APT | Advanced Persistent Threat |
| AV | Antivirus |
| BCP | Business Continuity Planning |
| C2/C&C | Command and Control |
| CA | Certificate Authority |
| CEF | Common Event Format |
| CERT | Computer Emergency Response Team |
| CISO | Chief Information Security Officer |
| CSIRT | Computer Security Incident Response Team |
| DDoS | Distributed Denial of Service |
| DLP | Data Loss Prevention |
| DNS | Domain Name System |
| DoS | Denial of Service |
| EDR | Endpoint Detection and Response |
| FIRST | Forum of Incident Response and Security Teams |
| GDPR | General Data Protection Regulation |
| GIAC | Global Information Assurance Certification |
| HIPAA | Health Insurance Portability and Accountability Act |
| IDS | Intrusion Detection System |
| IOC | Indicator of Compromise |
| IPS | Intrusion Prevention System |
| IR | Incident Response |
| IRP | Incident Response Plan |
| ISO | International Organization for Standardization |
| KPI | Key Performance Indicator |
| MISP | Malware Information Sharing Platform |
| MITRE | Massachusetts Institute of Technology Research and Engineering |
| MTTC | Mean Time to Contain |
| MTTD | Mean Time to Detect |
| MTTR | Mean Time to Respond/Recover |
| NIST | National Institute of Standards and Technology |
| PCI-DSS | Payment Card Industry Data Security Standard |
| RPO | Recovery Point Objective |
| RTO | Recovery Time Objective |
| SANS | SysAdmin, Audit, Network, and Security |
| SIEM | Security Information and Event Management |
| SOC | Security Operations Center |
| SOAR | Security Orchestration, Automation and Response |
| STIX | Structured Threat Information Expression |
| TAXII | Trusted Automated Exchange of Intelligence Information |
| TTP | Tactics, Techniques, and Procedures |
| UEBA | User and Entity Behavior Analytics |
| VERIS | Vocabulary for Event Recording and Incident Sharing |
| XDR | Extended Detection and Response |

---

## References

1. **NIST Computer Security Incident Handling Guide**
   - SP 800-61 Rev. 2
   - https://nvlpubs.nist.gov/nistpubs/SpecialPublications/NIST.SP.800-61r2.pdf

2. **ISO/IEC 27035 Information Security Incident Management**
   - https://www.iso.org/standard/78973.html

3. **MITRE ATT&CK Framework**
   - https://attack.mitre.org/

4. **FIRST CSIRT Services Framework**
   - https://www.first.org/standards/frameworks/csirts/

5. **SANS Incident Handler's Handbook**
   - https://www.sans.org/reading-room/whitepapers/incident/incident-handlers-handbook-33901

6. **VERIS Framework**
   - http://veriscommunity.net/

7. **Cyber Kill Chain**
   - Lockheed Martin
   - https://www.lockheedmartin.com/en-us/capabilities/cyber/cyber-kill-chain.html

---

**Document Control:**
- Version: 1.0
- Status: ACTIVE
- Last Review: 2025-12-25
- Next Review: 2026-06-25
- Owner: WIA Security Standards Committee

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
