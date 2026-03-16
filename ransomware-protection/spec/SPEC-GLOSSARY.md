# WIA-SEC-022: Ransomware Protection Standard
## GLOSSARY - TERMS & DEFINITIONS

**Standard ID:** WIA-SEC-022
**Version:** 1.0.0
**Last Updated:** 2025-12-25

---

## A

**Advanced Persistent Threat (APT)**
A prolonged and targeted cyberattack in which an intruder gains access to a network and remains undetected for an extended period. APT groups often deploy ransomware as part of multi-stage attacks.

**AES (Advanced Encryption Standard)**
Symmetric encryption algorithm used widely for securing data. AES-256-GCM is the recommended encryption standard for backup data in WIA-SEC-022.

**Air-Gapped Backup**
A backup copy that is physically isolated from the network, preventing remote access. Ransomware cannot encrypt data it cannot access over the network.

**Anomaly Detection**
Security approach that identifies deviations from normal behavior patterns. Used to detect zero-day ransomware that doesn't match known signatures.

---

## B

**Backup Window**
The scheduled time period during which backup operations occur, typically during off-peak hours to minimize performance impact.

**Behavioral Analysis**
Technique that monitors system and user actions to identify malicious behavior patterns characteristic of ransomware (e.g., mass file encryption, shadow copy deletion).

**BitLocker**
Full-disk encryption feature included in Windows. Protects against offline attacks but does not prevent ransomware from encrypting accessible files.

**Business Continuity Plan (BCP)**
Documented procedures for maintaining essential business functions during and after a disaster, including ransomware attacks.

---

## C

**C2 (Command and Control)**
Infrastructure used by attackers to communicate with compromised systems. Ransomware often contacts C2 servers to retrieve encryption keys or exfiltrate data.

**Canary Token**
Decoy data (files, credentials, URLs) embedded in systems to detect unauthorized access. Any interaction with a canary token triggers an immediate security alert.

**CEF (Common Event Format)**
Standardized log format for SIEM integration, enabling consistent event parsing across security tools.

**Credential Guard**
Windows security feature that uses virtualization-based security to isolate and protect credentials from theft.

**Crypto-Locker**
Ransomware family first identified in 2013. Term is often used generically to describe file-encrypting ransomware.

**Cryptographic Entropy**
Measure of randomness in data. Encrypted files have high entropy (7.9-8.0 bits per byte), making entropy analysis effective for ransomware detection.

---

## D

**Data Exfiltration**
Unauthorized transfer of data from a system. Modern ransomware often exfiltrates sensitive data before encryption to enable "double extortion."

**Disaster Recovery (DR)**
Process of restoring systems and data after a catastrophic event. WIA-SEC-022 emphasizes rapid DR with RTO targets <15 minutes for critical systems.

**Double Extortion**
Ransomware tactic where attackers both encrypt data AND threaten to publish stolen data if ransom is not paid.

**DPI (Deep Packet Inspection)**
Network analysis technique that examines packet contents beyond headers, enabling detection of malicious payloads and C2 communications.

---

## E

**EDR (Endpoint Detection and Response)**
Security solution that continuously monitors endpoints (workstations, servers) for threats, providing visibility and response capabilities.

**Encryption Key**
Cryptographic key used to encrypt or decrypt data. Ransomware generates random encryption keys for each infected system.

**Entropy**
In information theory, a measure of randomness or unpredictability. Shannon entropy formula:
```
H(X) = -Σ P(xi) log₂ P(xi)
```
where P(xi) is the probability of byte value xi occurring.

**Exploit Kit**
Malicious toolkit that automates exploitation of software vulnerabilities, often used to deliver ransomware payloads.

---

## F

**False Positive**
Alert triggered by benign activity mistakenly identified as malicious. WIA-SEC-022 targets FPR <0.1% to minimize operational disruption.

**File System Monitoring**
Real-time observation of file operations (create, modify, delete, rename) to detect suspicious patterns.

**Forensic Analysis**
Detailed investigation of a security incident to determine root cause, attack timeline, and scope of compromise.

---

## G

**GDPR (General Data Protection Regulation)**
European data protection law. Ransomware attacks may trigger breach notification requirements under GDPR Article 33 (72-hour notification).

**Glacier**
Amazon S3 storage class designed for long-term archival with lower costs. Suitable for ransomware backup retention beyond 30 days.

---

## H

**Hash**
Fixed-length cryptographic fingerprint of data. SHA-256 hashes verify backup integrity by detecting any data modification.

**HIDS (Host-based Intrusion Detection System)**
Security software that monitors a single system for malicious activity, as opposed to network-wide monitoring (NIDS).

**Honeypot**
Decoy system or file designed to attract and detect attackers. WIA-SEC-022 recommends deploying honeypot files to trigger early ransomware alerts.

**HSM (Hardware Security Module)**
Physical device that securely generates, stores, and manages cryptographic keys. Recommended for protecting backup encryption keys.

---

## I

**Immutable Storage**
Storage that cannot be modified or deleted once written. Essential for ransomware-proof backups. Implemented via WORM technology or object locking.

**Incident Response**
Organized approach to managing the aftermath of a security breach. WIA-SEC-022 defines automated response protocols executing in <250ms.

**Indicator of Compromise (IOC)**
Artifact observed on a network or system that indicates a security breach. Examples: file hashes, IP addresses, domain names, registry keys.

**Intrusion Detection System (IDS)**
Security technology that monitors network traffic or system activities for suspicious behavior.

---

## J

**JIT (Just-In-Time) Access**
Security model where privileged access is granted temporarily only when needed, reducing the window of exposure.

**JSON (JavaScript Object Notation)**
Lightweight data format used for SIEM integration, alert messages, and backup manifests in WIA-SEC-022.

---

## K

**Kerberos**
Network authentication protocol. Ransomware attackers often target Kerberos tickets (Golden Ticket attacks) for persistence and lateral movement.

**KMS (Key Management Service)**
Cloud service for managing encryption keys. AWS KMS and Azure Key Vault provide centralized key management for backups.

---

## L

**Lateral Movement**
Technique where attackers move through a network after initial compromise, seeking valuable targets. Ransomware operators use lateral movement to maximize infection scope.

**LockBit**
Ransomware-as-a-Service (RaaS) operation, one of the most prolific ransomware families since 2020. Known for fast encryption and affiliate program.

**LUKS (Linux Unified Key Setup)**
Standard for Linux disk encryption, similar to Windows BitLocker.

---

## M

**Machine Learning (ML)**
AI technique used in ransomware detection to identify malicious patterns. WIA-SEC-022 employs Random Forest and Neural Network models.

**Malware**
Malicious software designed to harm systems or users. Ransomware is a specific type of malware.

**MITRE ATT&CK**
Knowledge base of adversary tactics and techniques. Technique T1486 (Data Encrypted for Impact) describes ransomware behavior.

**MTTR (Mean Time to Recovery)**
Average time required to restore service after an incident. WIA-SEC-022 targets MTTR <15 minutes for critical systems.

---

## N

**Network Segmentation**
Dividing a network into smaller isolated segments to limit lateral movement and contain threats.

**NIDS (Network-based Intrusion Detection System)**
Monitors network traffic for suspicious activity across multiple systems.

**NIST (National Institute of Standards and Technology)**
U.S. government agency that develops cybersecurity standards and guidelines, including the Cybersecurity Framework.

---

## O

**Object Lock**
AWS S3 feature that prevents object deletion or modification for a specified retention period. Implements WORM storage for immutable backups.

**Offline Backup**
Backup stored on media disconnected from the network (tape, external drive). Provides air-gap protection against ransomware.

---

## P

**Payload**
Malicious component of malware that performs the intended harmful action (e.g., file encryption in ransomware).

**Phishing**
Social engineering attack that uses fraudulent emails to trick users into revealing credentials or downloading malware. Primary ransomware delivery method.

**Privilege Escalation**
Technique where attackers gain higher-level permissions than initially obtained, often required for ransomware to encrypt system files.

**Process Injection**
Technique where malicious code is injected into a legitimate process to evade detection. Some ransomware families use process injection.

---

## Q

**Quarantine**
Isolation of suspected malicious files or systems to prevent spread while allowing analysis.

---

## R

**RaaS (Ransomware-as-a-Service)**
Business model where ransomware developers lease their malware to affiliates, who conduct attacks and share ransom profits.

**Ransom Note**
Text file left by ransomware attackers with payment instructions. Common filenames: README.txt, HOW_TO_DECRYPT.html.

**RBAC (Role-Based Access Control)**
Access control method where permissions are assigned based on user roles, limiting potential ransomware damage.

**RDP (Remote Desktop Protocol)**
Protocol for remote system access. Commonly exploited by ransomware attackers via brute force or credential stuffing.

**Recovery Point Objective (RPO)**
Maximum acceptable data loss measured in time. WIA-SEC-022 specifies RPO <5 minutes for critical systems.

**Recovery Time Objective (RTO)**
Maximum acceptable downtime. WIA-SEC-022 targets RTO <15 minutes for critical systems.

**REvil (Sodinokibi)**
Sophisticated ransomware family known for high-profile attacks and double extortion tactics.

**Rootkit**
Malware that hides its presence by modifying operating system components. Some ransomware includes rootkit capabilities.

**RSA**
Asymmetric encryption algorithm. Ransomware typically uses RSA to encrypt the symmetric key used for file encryption.

**Ryuk**
Ransomware family known for targeting large organizations and demanding high ransoms ($1M+).

---

## S

**Salsa20**
Stream cipher used by some ransomware families (e.g., REvil) for fast file encryption.

**Shadow Copy**
Windows feature for creating file system snapshots. Ransomware often deletes shadow copies to prevent recovery.

**SIEM (Security Information and Event Management)**
Platform that aggregates and analyzes security logs from multiple sources. Essential for centralized ransomware detection.

**Snapshot**
Point-in-time copy of a file system or volume. Immutable snapshots enable rapid recovery from ransomware.

**SOAR (Security Orchestration, Automation and Response)**
Technology that automates incident response workflows, integrating multiple security tools.

**SQL Injection**
Attack technique exploiting database vulnerabilities. Can be used to gain initial access for ransomware deployment.

---

## T

**Tabletop Exercise**
Discussion-based simulation of incident response procedures without actual system changes. Validates ransomware recovery plans.

**Threat Intelligence**
Information about current and emerging threats. WIA-SEC-022 integrates feeds from Abuse.ch, AlienVault OTX, and other sources.

**TLS (Transport Layer Security)**
Cryptographic protocol for secure communications. Required for endpoint agent-to-server communications in WIA-SEC-022.

**Tor (The Onion Router)**
Anonymizing network used by ransomware operators for C2 communications and ransom payment sites.

**TPM (Trusted Platform Module)**
Hardware chip that provides cryptographic functions. Used with Secure Boot to prevent bootkit infections.

**Triple Extortion**
Evolution of double extortion where attackers also threaten DDoS attacks against the victim.

---

## U

**UEBA (User and Entity Behavior Analytics)**
Technology that establishes behavioral baselines and detects anomalies indicating compromised accounts or insider threats.

**UEFI Secure Boot**
Security standard ensuring only trusted software loads during boot. Prevents bootkit and rootkit infections.

---

## V

**VDI (Virtual Desktop Infrastructure)**
Centralized desktop delivery system. Simplifies ransomware recovery by reverting to clean images.

**Verifiable Credential**
Cryptographically secure credential that can be independently verified. Used in WIA-SEC-022 for backup integrity verification.

**VirusTotal**
Free online service for analyzing suspicious files and URLs using multiple antivirus engines.

**VLAN (Virtual Local Area Network)**
Network segmentation technique isolating groups of systems at Layer 2.

**VSS (Volume Shadow Copy Service)**
Windows service for creating backup snapshots. Primary ransomware target for deletion.

---

## W

**Watering Hole Attack**
Compromise of a website frequented by target victims to deliver malware.

**WannaCry**
Ransomware worm that spread globally in May 2017, exploiting EternalBlue vulnerability (MS17-010).

**Whitelist (Allow-List)**
List of explicitly permitted applications, preventing execution of unauthorized software including ransomware.

**WORM (Write-Once-Read-Many)**
Storage technology that prevents data modification after initial write. Foundation of immutable backup strategy.

---

## X

**XDR (Extended Detection and Response)**
Security approach integrating multiple detection and response technologies (EDR, NDR, cloud) for unified visibility.

---

## Y

**YARA**
Pattern matching tool for identifying malware based on textual or binary patterns. Used for ransomware signature detection.

---

## Z

**Zero Trust**
Security model assuming no implicit trust, requiring continuous verification. Limits ransomware lateral movement.

**Zero-Day**
Previously unknown vulnerability or malware variant. Behavioral detection is essential for zero-day ransomware.

---

## Acronyms Quick Reference

| Acronym | Full Term |
|---------|-----------|
| **AES** | Advanced Encryption Standard |
| **APT** | Advanced Persistent Threat |
| **C2** | Command and Control |
| **CEF** | Common Event Format |
| **DR** | Disaster Recovery |
| **EDR** | Endpoint Detection and Response |
| **GDPR** | General Data Protection Regulation |
| **HIDS** | Host-based Intrusion Detection System |
| **HSM** | Hardware Security Module |
| **IDS** | Intrusion Detection System |
| **IOC** | Indicator of Compromise |
| **IPS** | Intrusion Prevention System |
| **JIT** | Just-In-Time (Access) |
| **KMS** | Key Management Service |
| **LUKS** | Linux Unified Key Setup |
| **ML** | Machine Learning |
| **MTTR** | Mean Time to Recovery |
| **NIDS** | Network-based Intrusion Detection System |
| **NIST** | National Institute of Standards and Technology |
| **RaaS** | Ransomware-as-a-Service |
| **RBAC** | Role-Based Access Control |
| **RDP** | Remote Desktop Protocol |
| **RPO** | Recovery Point Objective |
| **RTO** | Recovery Time Objective |
| **SIEM** | Security Information and Event Management |
| **SOAR** | Security Orchestration, Automation and Response |
| **TLS** | Transport Layer Security |
| **TPM** | Trusted Platform Module |
| **UEBA** | User and Entity Behavior Analytics |
| **UEFI** | Unified Extensible Firmware Interface |
| **VDI** | Virtual Desktop Infrastructure |
| **VLAN** | Virtual Local Area Network |
| **VSS** | Volume Shadow Copy Service |
| **WORM** | Write-Once-Read-Many |
| **XDR** | Extended Detection and Response |

---

## Mathematical Formulas

### Shannon Entropy
```
H(X) = -Σ P(xi) × log₂ P(xi)

Where:
- H(X) = entropy in bits per byte
- P(xi) = probability of byte value xi
- Σ = sum over all possible byte values (0-255)

Interpretation:
- 0 bits: No randomness (all same byte)
- 8 bits: Maximum randomness (perfectly random)
- 7.9-8.0 bits: Indicates encryption
```

### Risk Score Calculation
```
Risk Score = Σ (Weight[i] × Indicator[i])

Where:
- Weight[i] = severity weight for indicator i
- Indicator[i] = binary (0 or 1) or normalized value

Example:
Risk = (30 × FilesModified) +
       (25 × ExtensionChanges) +
       (25 × ShadowDeletions) +
       (20 × HighEntropy)

Threshold:
- 0-50: Low risk
- 51-69: Medium risk
- 70-89: High risk
- 90-100: Critical risk (trigger immediate response)
```

### Backup Compression Ratio
```
Compression Ratio = Compressed Size / Original Size

Example:
Original: 2.4 GB
Compressed: 1.8 GB
Ratio = 1.8 / 2.4 = 0.75 (25% reduction)
```

---

## Common File Extensions Used by Ransomware

```
.encrypted    Generic encrypted file
.locked       Generic locked file
.crypted      Generic encrypted file
.crypt        Generic encrypted file

# Family-specific extensions:
.lockbit      LockBit ransomware
.ryuk         Ryuk ransomware
.sodinokibi   REvil/Sodinokibi ransomware
.conti        Conti ransomware
.hive         Hive ransomware
.alphv        BlackCat/ALPHV ransomware
.babyk        BabyLocker ransomware
.cerber       Cerber ransomware
.locky        Locky ransomware
.wannacry     WannaCry ransomware

# Random extensions (LockBit 3.0 and others):
.[random]     8-character random extension
```

---

## Common Ransom Note Filenames

```
README.txt
README.html
HOW_TO_DECRYPT.txt
HOW_TO_DECRYPT.html
DECRYPT_INSTRUCTIONS.txt
RESTORE_FILES.txt
FILES_ENCRYPTED.txt
YOUR_FILES_ARE_ENCRYPTED.txt
_readme.txt
!_HOW_TO_DECRYPT_!.txt
#_DECRYPT_MY_FILES_#.html
```

---

## Registry Keys Modified by Ransomware

```
# Persistence:
HKLM\Software\Microsoft\Windows\CurrentVersion\Run
HKCU\Software\Microsoft\Windows\CurrentVersion\Run
HKLM\Software\Microsoft\Windows\CurrentVersion\RunOnce

# Shadow Copy Deletion:
HKLM\SOFTWARE\Microsoft\Windows\CurrentVersion\Policies\System
HKLM\SYSTEM\CurrentControlSet\Services\VSS

# Boot Configuration:
HKLM\BCD00000000

# Startup Modifications:
HKLM\Software\Microsoft\Windows NT\CurrentVersion\Winlogon
```

---

## Common C2 Indicators

```
# Network Ports:
- 443/TCP (HTTPS) - Most common for encrypted C2
- 80/TCP (HTTP) - Legacy or less sophisticated
- 8080/TCP - Alternative web port
- 9050/TCP - Tor SOCKS proxy
- 53/UDP - DNS tunneling

# Domains:
- .onion domains (Tor hidden services)
- Recently registered domains (<30 days)
- Domains with DGA (Domain Generation Algorithm) patterns
- Free dynamic DNS services (no-ip.com, duckdns.org)

# IP Addresses:
- Bulletproof hosting providers
- Known ransomware infrastructure (from threat feeds)
- Anomalous geographic locations
```

---

*© 2025 WIA - World Certification Industry Association*
*弘익人間 · Benefit All Humanity*
