# WIA-DEF-004 PHASE 1 — Data Format Specification

**Standard:** WIA-DEF-004
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-DEF-004: Cyber Weapon Specification v1.0

> **Standard ID:** WIA-DEF-004
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Malware Classification](#2-malware-classification)
3. [Attack Vector Analysis](#3-attack-vector-analysis)
4. [Network Exploitation](#4-network-exploitation)
5. [Information Warfare](#5-information-warfare)
6. [Vulnerability Exploitation](#6-vulnerability-exploitation)
7. [Attribution Methodology](#7-attribution-methodology)
8. [Legal Frameworks](#8-legal-frameworks)
9. [Ethical Guidelines](#9-ethical-guidelines)
10. [Defense Strategies](#10-defense-strategies)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for understanding, analyzing, and defending against cyber weapons in accordance with international law and ethical principles.

### 1.2 Scope

The standard covers:
- Malware taxonomy and classification
- Attack vector identification and analysis
- Network exploitation techniques
- Information warfare concepts and countermeasures
- Vulnerability exploitation patterns
- Threat actor attribution methods
- Legal and ethical frameworks
- Defensive strategies and best practices

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard provides defensive knowledge to protect humanity from cyber threats while maintaining strict ethical boundaries. All information is for defensive purposes only and must comply with international law.

### 1.4 Terminology

- **Cyber Weapon**: Software, hardware, or operational capability designed to cause harm through digital means
- **Malware**: Malicious software designed to damage, disrupt, or gain unauthorized access
- **APT (Advanced Persistent Threat)**: Sophisticated, sustained cyber attack campaign
- **Zero-Day**: Previously unknown vulnerability with no available patch
- **TTPs**: Tactics, Techniques, and Procedures used by threat actors
- **Attribution**: Process of identifying the source of a cyber attack
- **IOC (Indicator of Compromise)**: Evidence of potential security breach

---

## 2. Malware Classification

### 2.1 Primary Categories

#### 2.1.1 Virus

**Definition**: Self-replicating malware that requires host program

Characteristics:
```
Propagation = Infection × Execution
Detection_Difficulty = Polymorphism + Encryption
```

Common Types:
- **File Infector**: Attaches to executable files
- **Boot Sector**: Infects boot records
- **Macro**: Embedded in document macros
- **Polymorphic**: Changes code to evade detection
- **Metamorphic**: Rewrites itself completely

Defense:
- Signature-based antivirus
- Behavioral analysis
- Application whitelisting
- Regular scanning

#### 2.1.2 Worm

**Definition**: Self-replicating malware that spreads independently

Characteristics:
```
Spread_Rate = Vulnerability_Exploitation × Network_Connectivity
Impact = Infected_Systems × Payload_Severity
```

Famous Examples:
- **Morris Worm** (1988): First major internet worm
- **ILOVEYOU** (2000): Email-based propagation
- **Conficker** (2008): Multi-vector exploitation
- **WannaCry** (2017): SMB EternalBlue exploit
- **NotPetya** (2017): Wiper disguised as ransomware

Defense:
- Network segmentation
- Patch management
- Traffic monitoring
- Automated containment

#### 2.1.3 Trojan Horse

**Definition**: Malware disguised as legitimate software

Types:
```
Trojan Types:
├── Backdoor: Remote access tools (RAT)
├── Downloader: Fetches additional malware
├── Banker: Steals financial credentials
├── Keylogger: Records keystrokes
├── Rootkit: Hides malicious activity
├── Ransomware: Encrypts data for extortion
└── Spyware: Surveillance and data theft
```

Common Delivery Methods:
- Fake software updates
- Pirated applications
- Malicious email attachments
- Drive-by downloads
- Social engineering

Defense:
- Software verification (digital signatures)
- User awareness training
- Application sandboxing
- Network traffic analysis

#### 2.1.4 Ransomware

**Definition**: Malware that encrypts data and demands payment

Ransomware Evolution:
```
Generation 1 (2005-2012): Simple file encryption
Generation 2 (2013-2016): Crypto ransomware (CryptoLocker)
Generation 3 (2017-2019): Worm capabilities (WannaCry)
Generation 4 (2020-2023): Double extortion (data theft + encryption)
Generation 5 (2024+): Triple extortion (DDoS + data + encryption)
```

Attack Chain:
```
Initial Access → Lateral Movement → Data Exfiltration → Encryption → Ransom Demand
```

Notable Families:
| Family | First Seen | Encryption | Ransom | Notes |
|--------|-----------|------------|--------|-------|
| WannaCry | 2017 | RSA-2048 | $300-$600 BTC | EternalBlue exploit |
| Ryuk | 2018 | RSA-4096 + AES-256 | $100K-$5M | Targeted attacks |
| REvil/Sodinokibi | 2019 | Salsa20 + ECC | Variable | RaaS model |
| Conti | 2020 | AES-256 | Variable | Affiliate network |
| LockBit | 2021 | AES + RSA | Variable | Fastest encryption |

Defense Strategy:
```
Prevention:
- Offline backups (3-2-1 rule)
- Email filtering
- Endpoint protection
- Privilege restriction

Detection:
- Behavioral analysis
- File integrity monitoring
- Network anomaly detection
- Honeypots

Response:
- Isolation procedures
- Forensic preservation
- Law enforcement notification
- NO PAYMENT policy
```

#### 2.1.5 Advanced Persistent Threat (APT)

**Definition**: Sophisticated, sustained cyber espionage campaign

APT Lifecycle:
```
Phase 1: Reconnaissance → Target profiling, infrastructure mapping
Phase 2: Initial Compromise → Spear phishing, zero-day exploitation
Phase 3: Establish Foothold → Backdoor installation, persistence
Phase 4: Escalate Privileges → Credential theft, lateral movement
Phase 5: Internal Reconnaissance → Network mapping, data discovery
Phase 6: Complete Mission → Data exfiltration, system manipulation
Phase 7: Maintain Presence → Long-term access, cover tracks
```

Nation-State APT Groups (Defensive Knowledge):
| Group | Origin | Targets | TTPs | Notable Campaigns |
|-------|--------|---------|------|-------------------|
| APT1 (Comment Crew) | China | Defense, Tech | Spear phishing, RATs | Operation Aurora |
| APT28 (Fancy Bear) | Russia | Government, Military | Zero-days, DNC breach | Olympic Destroyer |
| APT29 (Cozy Bear) | Russia | Government, Think tanks | Stealth, persistence | SolarWinds |
| Lazarus Group | North Korea | Financial, Media | Wipers, ransomware | Sony Pictures, WannaCry |
| APT33 (Elfin) | Iran | Energy, Aviation | Destructive malware | Shamoon |
| Equation Group | Unknown | Global | Advanced implants | Stuxnet collaboration |

APT Indicators:
```
Technical:
- Custom malware families
- Zero-day exploits
- Living-off-the-land techniques
- Advanced evasion tactics

Operational:
- Long-term persistence (months/years)
- Resource-intensive operations
- Coordinated multi-stage attacks
- Professional tradecraft

Strategic:
- Nation-state aligned targets
- Geopolitical timing
- High-value intelligence focus
- Attribution obfuscation
```

Defense:
- Assume breach mentality
- Zero trust architecture
- Threat hunting programs
- Advanced EDR/XDR
- Threat intelligence integration

### 2.2 Malware Behavior Analysis

#### 2.2.1 Persistence Mechanisms

Common Techniques (MITRE ATT&CK):
```
Windows:
- Registry Run Keys: HKLM\Software\Microsoft\Windows\CurrentVersion\Run
- Scheduled Tasks: schtasks /create
- WMI Event Subscriptions: Fileless persistence
- Service Creation: sc create
- DLL Hijacking: Search order manipulation
- Boot/Logon Scripts: Group Policy objects

Linux:
- Cron Jobs: /etc/crontab, /var/spool/cron
- Init Scripts: /etc/init.d/, systemd services
- Profile Files: .bashrc, .bash_profile
- SSH Keys: ~/.ssh/authorized_keys
- LD_PRELOAD: Library injection

macOS:
- Launch Agents/Daemons: ~/Library/LaunchAgents
- Login Items: System Preferences
- Kernel Extensions: Deprecated in modern macOS
```

Detection:
```bash
# Windows: Monitor autoruns
autorunsc -a * -c -h -accepteula

# Linux: Check cron jobs
crontab -l
ls -la /etc/cron.*

# macOS: List launch agents
launchctl list
```

#### 2.2.2 Evasion Techniques

Anti-Analysis Methods:
```
1. Anti-Debugging:
   - IsDebuggerPresent() API checks
   - PEB (Process Environment Block) inspection
   - Timing attacks (rdtsc instruction)
   - Exception-based detection

2. Anti-VM:
   - VMware/VirtualBox artifact detection
   - CPU instruction differences
   - MAC address checks (00:0C:29 = VMware)
   - Registry key inspection

3. Code Obfuscation:
   - Packing/Compression (UPX, Themida)
   - Encryption (XOR, AES)
   - Polymorphism (code mutation)
   - Metamorphism (code rewriting)

4. Sandbox Evasion:
   - Sleep/delay before execution
   - User interaction requirements
   - Environmental checks (files, processes)
   - Resource-intensive operations
```

Entropy Analysis:
```
Entropy = -Σ(p(x) × log₂(p(x)))

Where:
- p(x) = Probability of byte value x
- High entropy (>7.0) = Likely encrypted/packed
- Low entropy (<5.0) = Likely plaintext
```

#### 2.2.3 Payload Classification

Payload Types:
```
Destructive:
- Wiper: Data destruction (disk formatting, file deletion)
- Logic Bomb: Time/condition-triggered damage
- Bootkiller: MBR/boot sector corruption

Espionage:
- Keylogger: Keystroke recording
- Screen Capture: Screenshot collection
- Credential Stealer: Password/token theft
- Document Harvester: Sensitive file collection

Financial:
- Banking Trojan: Transaction manipulation
- Cryptocurrency Miner: Resource hijacking
- POS Malware: Payment card theft

Disruption:
- DDoS Bot: Distributed denial of service
- Spam Bot: Email/message flooding
- Proxy Bot: Traffic routing
```

---

## 3. Attack Vector Analysis

### 3.1 Network-Based Vectors

#### 3.1.1 Exploitation Over Network

Common Services Targeted:
```
Port 21 (FTP): Anonymous access, credential theft
Port 22 (SSH): Brute force, key theft
Port 23 (Telnet): Cleartext credentials
Port 25 (SMTP): Email relay abuse
Port 80/443 (HTTP/HTTPS): Web application attacks
Port 135-139, 445 (SMB): EternalBlue, credential relay
Port 1433 (MSSQL): SQL injection, xp_cmdshell
Port 3306 (MySQL): SQL injection
Port 3389 (RDP): Brute force, BlueKeep
Port 5432 (PostgreSQL): SQL injection
```

Exploitation Frameworks (Defensive Knowledge):
```
Metasploit:
- Modular architecture
- 2,000+ exploits
- Post-exploitation modules
- Payload generation

Cobalt Strike:
- Adversary simulation
- Beacon payloads
- Lateral movement
- Often abused by threat actors

Empire/PowerShell Empire:
- PowerShell-based
- Fileless attacks
- Post-exploitation
```

Defense:
```
- Network segmentation (VLANs, firewalls)
- Intrusion Detection/Prevention Systems (IDS/IPS)
- Port security and monitoring
- Regular vulnerability scanning
- Disable unnecessary services
```

#### 3.1.2 Man-in-the-Middle (MitM)

Attack Types:
```
1. ARP Spoofing:
   Tool: arpspoof, ettercap
   Mitigation: Dynamic ARP Inspection (DAI)

2. DNS Spoofing:
   Tool: dnsspoof
   Mitigation: DNSSEC, DNS over HTTPS

3. SSL Stripping:
   Tool: sslstrip
   Mitigation: HSTS, certificate pinning

4. Session Hijacking:
   Method: Cookie theft, TCP hijacking
   Mitigation: Session encryption, token rotation
```

Detection:
```python
# Detect ARP spoofing
def detect_arp_spoof():
    arp_table = get_arp_table()
    for ip, mac in arp_table.items():
        if mac != expected_mac[ip]:
            alert(f"Possible ARP spoofing: {ip} -> {mac}")
```

#### 3.1.3 Denial of Service (DoS/DDoS)

Attack Categories:
```
Volume-Based:
- UDP Flood: High packet rate
- ICMP Flood: Ping flood
- DNS Amplification: 1:50 amplification ratio

Protocol-Based:
- SYN Flood: TCP handshake exhaustion
- Ping of Death: Oversized packets
- Smurf Attack: ICMP broadcast

Application-Layer:
- HTTP Flood: GET/POST requests
- Slowloris: Slow HTTP connections
- DNS Query Flood: Recursive queries
```

DDoS Calculation:
```
Attack_Bandwidth = Bots × Bot_Bandwidth × Amplification_Factor

Example:
- Bots: 100,000 (Mirai botnet)
- Bot Bandwidth: 1 Mbps
- Amplification: 50× (DNS)
- Total: 100,000 × 1 × 50 = 5 Tbps


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
