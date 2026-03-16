# WIA-SEC-016: Intrusion Detection Standard
## GLOSSARY

**Standard ID:** WIA-SEC-016
**Title:** Intrusion Detection - Terminology Glossary
**Version:** 1.0.0
**Last Updated:** 2025-12-25

---

## A

**Anomaly Detection**
Detection method that identifies deviations from established baseline of normal behavior. More effective against zero-day attacks than signature-based detection, but typically generates higher false positive rates.

**Alert**
Notification generated when IDS/IPS detects suspicious or malicious activity. Contains details such as timestamp, source/destination IPs, signature matched, and severity level.

**Attack Signature**
Pre-defined pattern or rule used to identify known attack methods. Similar to antivirus signatures but focused on network traffic patterns rather than file signatures.

**Authentication Bypass**
Attack technique that circumvents authentication mechanisms to gain unauthorized access to systems or applications.

**Aho-Corasick Algorithm**
Efficient string-matching algorithm used in IDS/IPS for parallel pattern matching. Allows simultaneous search for thousands of signatures in network traffic.

---

## B

**Baseline**
Statistical profile of normal network behavior established during learning period. Used as reference point for anomaly detection. Typically requires 30-90 days of traffic analysis.

**Behavioral Analysis**
Detection method that analyzes patterns of user or system behavior to identify anomalies. Includes User and Entity Behavior Analytics (UEBA).

**Blacklist**
List of known malicious IP addresses, domains, or URLs that should be blocked. Maintained through threat intelligence feeds.

**Brute Force Attack**
Attack method that systematically tries all possible combinations to guess passwords or encryption keys. Detected by monitoring failed authentication attempts.

**BPF (Berkeley Packet Filter)**
Filtering mechanism for capturing network packets at kernel level. Allows efficient packet selection before deep inspection.

---

## C

**C2 (Command and Control)**
Communication channel between compromised system and attacker's server. IDS detects C2 through beaconing patterns, unusual protocols, or domain reputation.

**CEF (Common Event Format)**
Standardized log format for security events, developed by ArcSight. Widely supported by SIEM platforms.

**Classifier**
Machine learning model that categorizes network traffic into predefined classes (e.g., benign, malicious, specific attack types).

**Cleartext**
Unencrypted data transmitted over network. IDS can inspect cleartext protocols (HTTP, FTP, Telnet) but not encrypted traffic (HTTPS, SFTP).

**Correlation**
Process of linking related security events to identify complex attack patterns. Key feature of SIEM systems.

**CVE (Common Vulnerabilities and Exposures)**
Standardized identifier for known security vulnerabilities. IDS rules often reference CVE numbers.

---

## D

**DDoS (Distributed Denial of Service)**
Attack that overwhelms target system with traffic from multiple sources. IDS detects through traffic volume analysis and pattern recognition.

**Deep Packet Inspection (DPI)**
Network traffic analysis technique that examines packet headers and payload content. Required for signature-based detection.

**Detection Engine**
Core component of IDS/IPS responsible for analyzing packets and identifying threats. Implements signature matching, protocol analysis, and anomaly detection.

**DPDK (Data Plane Development Kit)**
Framework for fast packet processing in user space, bypassing kernel. Significantly improves IDS/IPS throughput.

**Drop (Packet)**
IPS action to silently discard malicious packet. More subtle than reject, which sends notification back to sender.

---

## E

**EER (Equal Error Rate)**
Point where False Accept Rate equals False Reject Rate in biometric or anomaly detection systems. Used to evaluate detection accuracy.

**Evasion Technique**
Method used by attackers to bypass IDS/IPS detection. Examples: packet fragmentation, encoding, encryption, timing manipulation.

**Event**
Single occurrence of security-relevant activity detected by IDS. Multiple events may be correlated to identify incidents.

**Exploit**
Code or technique that takes advantage of vulnerability to compromise system. IDS detects exploit attempts through signatures and behavioral patterns.

---

## F

**False Negative**
Malicious traffic that IDS/IPS fails to detect. Most dangerous type of error as attacks go unnoticed.

**False Positive**
Benign traffic incorrectly flagged as malicious. High false positive rates reduce effectiveness and analyst confidence.

**Flow**
Sequence of packets sharing common characteristics (same source/destination IPs, ports, protocol) representing single communication session.

**FPGA (Field-Programmable Gate Array)**
Hardware accelerator for signature matching. Provides 10-100x performance improvement over software-based detection.

---

## G

**GeoIP**
Database mapping IP addresses to geographic locations. Used to enrich alerts and detect anomalous login locations.

**Greylisting**
Temporary blocking of suspicious sources pending further analysis. More cautious than immediate blacklisting.

---

## H

**Heuristic Analysis**
Detection method using rules-of-thumb or educated guesses rather than exact signatures. Effective against variants of known attacks.

**HIDS (Host-based Intrusion Detection System)**
IDS deployed on individual host to monitor system calls, file integrity, logs, and process execution. Complements network-based IDS.

**Honeypot**
Decoy system designed to attract attackers. IDS monitors honeypot to detect attack techniques and collect threat intelligence.

---

## I

**ICMP (Internet Control Message Protocol)**
Network protocol for diagnostic messages (ping, traceroute). Often abused for reconnaissance or DDoS attacks.

**Incident**
Security event or series of events requiring investigation and response. Not all alerts constitute incidents.

**Indicator of Compromise (IoC)**
Artifact observed in network/host that suggests system compromise. Examples: malicious IP, suspicious domain, file hash.

**Inline Mode**
IPS deployment where traffic flows through sensor, allowing real-time blocking. Opposite of passive mode.

**IPFIX (IP Flow Information Export)**
Standardized protocol for exporting flow data, successor to NetFlow. Provides richer metadata than NetFlow v5/v9.

---

## K

**Kill Chain**
Stages of cyber attack: reconnaissance, weaponization, delivery, exploitation, installation, command & control, actions on objectives. IDS aims to detect early stages.

---

## L

**Lateral Movement**
Attacker's progression through network after initial compromise. Detected by monitoring east-west traffic patterns.

**LEEF (Log Event Extended Format)**
Structured log format developed by IBM for QRadar SIEM. Alternative to CEF.

**Libpcap**
Standard library for packet capture on Unix/Linux systems. Foundation for most IDS/IPS software.

**Liveness Detection**
Technique to verify real user presence (vs. replay attack). In IDS context, detects live attacks vs. recorded traffic.

---

## M

**Machine Learning (ML)**
Algorithms that learn patterns from data without explicit programming. Used in IDS for anomaly detection and zero-day identification.

**Man-in-the-Middle (MitM)**
Attack where attacker intercepts communication between two parties. IDS detects through ARP spoofing, certificate anomalies.

**MITRE ATT&CK**
Knowledge base of adversary tactics and techniques. IDS alerts can be mapped to ATT&CK framework for threat context.

**MTTR (Mean Time to Respond)**
Average time from alert to containment. Key performance metric for incident response.

---

## N

**NetFlow**
Cisco protocol for exporting flow-level network statistics. Provides less detail than full packet capture but more scalable.

**NIDS (Network-based Intrusion Detection System)**
IDS that monitors network traffic via tap or SPAN port. Provides visibility across entire network segment.

**NIST (National Institute of Standards and Technology)**
US agency that publishes cybersecurity standards, including SP 800-94 on IDS/IPS.

**NFIQ (NIST Fingerprint Image Quality)**
Quality metric for fingerprint images. In IDS context, analogous to packet quality metrics.

---

## P

**Passive Mode**
IDS deployment where sensor monitors copy of traffic without affecting flow. Cannot block attacks but zero risk of network disruption.

**PCAP (Packet Capture)**
File format for storing captured network packets. De facto standard for network forensics.

**Payload**
Application-layer data in network packet. IDS inspects payload for malicious content.

**Port Scan**
Reconnaissance technique to identify open ports and services. IDS detects by tracking connection attempts across port ranges.

**Precision**
Metric measuring proportion of true positives among all positive predictions: TP / (TP + FP). Target: >90%.

**Preprocessor**
IDS component that normalizes and reassembles traffic before detection. Examples: TCP stream reassembly, HTTP normalization.

---

## Q

**QRadar**
IBM SIEM platform supporting IDS integration via syslog, LEEF format.

**Quarantine**
Isolation of suspicious host from network pending investigation. Automated response to high-severity alerts.

---

## R

**Rate Limiting**
IPS technique to slow suspicious traffic rather than blocking entirely. Less disruptive than complete block.

**Recall**
Metric measuring proportion of actual attacks detected: TP / (TP + FN). Also called True Positive Rate. Target: >95%.

**Reconnaissance**
Pre-attack information gathering phase. Includes port scanning, banner grabbing, DNS enumeration. Early kill chain stage.

**Regex (Regular Expression)**
Pattern matching syntax for text. Used in IDS rules via PCRE (Perl Compatible Regular Expressions).

**Reputation**
Score indicating trustworthiness of IP address or domain based on historical behavior. Used to prioritize alerts.

**Rootkit**
Malware that hides presence by modifying OS. HIDS can detect through file integrity monitoring and system call analysis.

**Rule**
Signature definition specifying traffic pattern to detect. Snort/Suricata use text-based rule syntax.

---

## S

**Sensor**
IDS/IPS device deployed to monitor network segment or host. Reports to central management console.

**Severity**
Importance level assigned to alert. Typical scale: Low, Medium, High, Critical. Determines response priority.

**Signature**
See Attack Signature.

**SIEM (Security Information and Event Management)**
Platform that aggregates and correlates logs from multiple sources including IDS, firewall, endpoints.

**Sinkhole**
DNS redirect technique to capture traffic destined for malicious domains. IDS monitors sinkhole traffic for infected hosts.

**SNMP (Simple Network Management Protocol)**
Protocol for network device management. IDS can send SNMP traps for alerts.

**Snort**
Popular open-source NIDS. Industry-standard rule syntax widely adopted by other IDS/IPS.

**SOAR (Security Orchestration, Automation, and Response)**
Platform that automates incident response workflows based on IDS alerts.

**SPAN Port (Switched Port Analyzer)**
Cisco switch feature that mirrors traffic to monitoring port for IDS. Also called port mirroring.

**Spoofing**
Falsifying source address in packets. IDS detects through inconsistency checks and baseline deviation.

**SQL Injection**
Web attack inserting malicious SQL code into input fields. IDS detects through payload signatures.

**STIX (Structured Threat Information eXpression)**
Standardized language for describing cyber threat information. Used in threat intelligence feeds.

**Suricata**
Multi-threaded open-source IDS/IPS. Supports GPU acceleration and advanced protocol analysis.

**Syslog**
Standard protocol for sending log messages. Primary method for IDS to forward alerts to SIEM.

---

## T

**TAXII (Trusted Automated eXchange of Indicator Information)**
Protocol for sharing cyber threat intelligence. IDS can consume TAXII feeds for updated IoCs.

**TCP Reassembly**
Process of reconstructing TCP stream from individual packets. Required to detect attacks split across multiple packets.

**Threat Intelligence**
Information about threats, adversaries, and their tactics. IDS integrates threat feeds to enhance detection.

**Threshold**
Rule parameter to suppress alerts until event occurs N times in X seconds. Reduces false positives from noisy signatures.

**TLS (Transport Layer Security)**
Encryption protocol for secure communications. IDS cannot inspect encrypted payload without TLS inspection (man-in-the-middle).

**True Positive**
Correctly detected malicious traffic. Desired outcome of IDS.

**TTL (Time to Live)**
IP header field indicating packet hop limit. Anomalous TTL values may indicate spoofing or OS fingerprinting.

**Tuning**
Process of adjusting IDS rules and thresholds to reduce false positives while maintaining detection effectiveness.

---

## U

**UEBA (User and Entity Behavior Analytics)**
Security analysis technique that detects anomalies in user and device behavior. Advanced form of behavioral analysis.

**URI (Uniform Resource Identifier)**
Web address in HTTP requests. IDS inspects URIs for SQL injection, XSS, directory traversal attacks.

**User-Agent**
HTTP header identifying client software. IDS analyzes for suspicious patterns (e.g., penetration testing tools).

---

## V

**Vulnerability**
Weakness in system that can be exploited. IDS detects exploitation attempts, not vulnerabilities themselves.

---

## W

**WAF (Web Application Firewall)**
Security device protecting web applications. Often used alongside IDS for defense-in-depth.

**Whitelist**
List of trusted sources exempted from certain detection rules. Reduces false positives from known-good traffic.

---

## X

**XSS (Cross-Site Scripting)**
Web attack injecting malicious scripts into web pages. IDS detects through payload signatures and encoding analysis.

---

## Z

**Zeek (formerly Bro)**
Network security monitor providing high-level analysis of network traffic. Complements signature-based IDS.

**Zero-Day**
Previously unknown vulnerability or attack. Signature-based IDS ineffective; requires anomaly detection or behavioral analysis.

**Zone**
Network segment with specific security policy. IDS sensors typically deployed at zone boundaries to monitor inter-zone traffic.

---

## Acronyms

| Acronym | Full Form |
|---------|-----------|
| ACL | Access Control List |
| AES | Advanced Encryption Standard |
| API | Application Programming Interface |
| APT | Advanced Persistent Threat |
| ARP | Address Resolution Protocol |
| BPF | Berkeley Packet Filter |
| BPS | Bytes Per Second |
| C2 | Command and Control |
| CEF | Common Event Format |
| CIA | Confidentiality, Integrity, Availability |
| CIDR | Classless Inter-Domain Routing |
| CPU | Central Processing Unit |
| CVE | Common Vulnerabilities and Exposures |
| DDoS | Distributed Denial of Service |
| DHCP | Dynamic Host Configuration Protocol |
| DNS | Domain Name System |
| DoS | Denial of Service |
| DPI | Deep Packet Inspection |
| DPDK | Data Plane Development Kit |
| EER | Equal Error Rate |
| FAR | False Accept Rate |
| FIPS | Federal Information Processing Standards |
| FRR | False Reject Rate |
| FTP | File Transfer Protocol |
| GDPR | General Data Protection Regulation |
| GPU | Graphics Processing Unit |
| HA | High Availability |
| HEC | HTTP Event Collector (Splunk) |
| HIDS | Host-based Intrusion Detection System |
| HTTP | Hypertext Transfer Protocol |
| HTTPS | HTTP Secure |
| ICMP | Internet Control Message Protocol |
| IDS | Intrusion Detection System |
| IoC | Indicator of Compromise |
| IP | Internet Protocol |
| IPFIX | IP Flow Information Export |
| IPS | Intrusion Prevention System |
| IPv4 | Internet Protocol version 4 |
| IPv6 | Internet Protocol version 6 |
| ISO | International Organization for Standardization |
| JSON | JavaScript Object Notation |
| LEEF | Log Event Extended Format |
| MAC | Media Access Control |
| MFA | Multi-Factor Authentication |
| MitM | Man-in-the-Middle |
| ML | Machine Learning |
| MTTR | Mean Time to Respond |
| NIDS | Network-based Intrusion Detection System |
| NIST | National Institute of Standards and Technology |
| NMS | Network Management System |
| NOC | Network Operations Center |
| NTP | Network Time Protocol |
| OS | Operating System |
| PCAP | Packet Capture |
| PCRE | Perl Compatible Regular Expressions |
| PCI DSS | Payment Card Industry Data Security Standard |
| PPS | Packets Per Second |
| QoS | Quality of Service |
| RAM | Random Access Memory |
| REST | Representational State Transfer |
| RFC | Request for Comments |
| RFI | Remote File Inclusion |
| ROI | Return on Investment |
| SIEM | Security Information and Event Management |
| SLA | Service Level Agreement |
| SMS | Short Message Service |
| SMTP | Simple Mail Transfer Protocol |
| SNMP | Simple Network Management Protocol |
| SOC | Security Operations Center |
| SOAR | Security Orchestration, Automation, Response |
| SQL | Structured Query Language |
| SSH | Secure Shell |
| SSL | Secure Sockets Layer |
| STIX | Structured Threat Information eXpression |
| SVM | Support Vector Machine |
| TCP | Transmission Control Protocol |
| TAXII | Trusted Automated eXchange of Indicator Information |
| TLS | Transport Layer Security |
| TPR | True Positive Rate |
| TTL | Time to Live |
| UEBA | User and Entity Behavior Analytics |
| UDP | User Datagram Protocol |
| URI | Uniform Resource Identifier |
| URL | Uniform Resource Locator |
| VPN | Virtual Private Network |
| VLAN | Virtual Local Area Network |
| WAF | Web Application Firewall |
| WIA | World Certification Industry Association |
| XSS | Cross-Site Scripting |

---

**Document Control:**
- Author: WIA Security Standards Committee
- Effective Date: 2025-12-25
- Review Cycle: Annual
- Next Review: 2026-12-25

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association (WIA)
