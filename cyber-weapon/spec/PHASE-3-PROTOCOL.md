# WIA-DEF-004 PHASE 3 — Protocol Specification

**Standard:** WIA-DEF-004
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

2. Emotional Manipulation:
   - Fear amplification
   - Outrage farming
   - Tribalism exploitation

3. Trust Erosion:
   - Media credibility attacks
   - Institution delegitimization
   - Expert discreditation

4. Narrative Hijacking:
   - Hashtag hijacking
   - Topic sliding
   - Forum dilution
```

Countermeasures:
```
Individual:
- Media literacy
- Source verification
- Emotional awareness
- Diverse information diet

Organizational:
- Rapid response teams
- Fact-checking partnerships
- Transparent communication
- Trust-building initiatives
```

---

## 6. Vulnerability Exploitation

### 6.1 Vulnerability Classification

#### 6.1.1 CVSS Scoring

Common Vulnerability Scoring System v3.1:
```
Base Score = f(Exploitability, Impact)

Exploitability Metrics:
- Attack Vector (AV): Network, Adjacent, Local, Physical
- Attack Complexity (AC): Low, High
- Privileges Required (PR): None, Low, High
- User Interaction (UI): None, Required

Impact Metrics:
- Confidentiality (C): None, Low, High
- Integrity (I): None, Low, High
- Availability (A): None, Low, High

Temporal Metrics:
- Exploit Code Maturity
- Remediation Level
- Report Confidence

Environmental Metrics:
- Modified Base Metrics
- Confidentiality/Integrity/Availability Requirements
```

Severity Ratings:
| Score | Rating | Action |
|-------|--------|--------|
| 0.0 | None | No action required |
| 0.1-3.9 | Low | Schedule patching |
| 4.0-6.9 | Medium | Patch within 30 days |
| 7.0-8.9 | High | Patch within 7 days |
| 9.0-10.0 | Critical | Patch within 24 hours |

#### 6.1.2 Zero-Day Vulnerabilities

Zero-Day Lifecycle:
```
Discovery → Weaponization → Disclosure → Patch → Deployment

Threat Window:
Time = Disclosure → Patch_Available → Patch_Deployed
Average: 0-day disclosure to patch = 7-30 days
Average: Patch available to deployed = 30-90 days
```

Notable Zero-Days:
| Year | Name | CVE | Impact | CVSS |
|------|------|-----|--------|------|
| 2014 | Heartbleed | CVE-2014-0160 | SSL/TLS memory leak | 7.5 |
| 2017 | EternalBlue | CVE-2017-0144 | SMB RCE | 9.3 |
| 2021 | Log4Shell | CVE-2021-44228 | JNDI injection RCE | 10.0 |
| 2021 | ProxyLogon | CVE-2021-26855 | Exchange Server RCE | 9.8 |
| 2022 | Spring4Shell | CVE-2022-22965 | Spring RCE | 9.8 |

Defense:
```
- Virtual patching (WAF rules)
- Network segmentation
- Compensating controls
- Threat intelligence monitoring
- Rapid patching processes
- Zero trust architecture
```

### 6.2 Exploitation Techniques

#### 6.2.1 Memory Corruption

Buffer Overflow:
```c
// Vulnerable code (educational example)
void vulnerable_function(char *input) {
    char buffer[64];
    strcpy(buffer, input);  // No bounds checking!
}

// Attack payload structure:
[NOP sled][Shellcode][Return address (overwrite)]

// Defense:
// 1. Use safe functions
strncpy(buffer, input, sizeof(buffer) - 1);

// 2. Stack canaries (compiler protection)
// 3. ASLR (Address Space Layout Randomization)
// 4. DEP/NX (Data Execution Prevention)
```

Heap Spray:
```
Technique: Fill heap with NOP sleds and shellcode
Goal: Increase exploitation reliability
Defense: Heap ASLR, heap integrity checks
```

#### 6.2.2 Privilege Escalation

Vertical Escalation:
```
User → Admin/Root

Common Techniques:
- SUID binary exploitation
- Kernel vulnerabilities
- Misconfigured sudo
- DLL hijacking (Windows)
- Token impersonation (Windows)
```

Horizontal Escalation:
```
User A → User B (same privilege level)

Common Techniques:
- Session hijacking
- Credential theft
- CSRF attacks
```

Linux Privilege Escalation Checks:
```bash
# SUID binaries
find / -perm -4000 -type f 2>/dev/null

# Sudo permissions
sudo -l

# Kernel version
uname -a
searchsploit kernel

# Cron jobs
cat /etc/crontab
ls -la /etc/cron.*

# Writable /etc/passwd
ls -la /etc/passwd
```

---

## 7. Attribution Methodology

### 7.1 Technical Attribution

#### 7.1.1 Malware Analysis

Forensic Indicators:
```
Static Analysis:
- File hashes (MD5, SHA-256)
- Compilation timestamps
- Code signing certificates
- Embedded strings
- Resource sections
- PE headers

Dynamic Analysis:
- Network indicators (C2 domains, IPs)
- File system artifacts
- Registry modifications
- Process behavior
- API calls
```

Code Similarity:
```
Techniques:
- Function hashing (TLSH, ssdeep)
- Control flow graph comparison
- String matching
- Compiler artifact analysis
- Code reuse detection

Tools:
- IDA Pro
- Ghidra
- Binary Ninja
- YARA rules
```

#### 7.1.2 Infrastructure Analysis

Network Attribution:
```
Infrastructure Fingerprinting:
- IP address geolocation
- Domain registration (WHOIS)
- SSL certificate analysis
- Hosting provider identification
- ASN (Autonomous System Number) tracking

Shared Infrastructure:
- C2 server reuse
- Common infrastructure patterns
- Operational security mistakes
- Timing analysis (timezone correlation)
```

### 7.2 Operational Attribution

#### 7.2.1 TTP Analysis

MITRE ATT&CK Mapping:
```
Tactics (What):
- Initial Access
- Execution
- Persistence
- Privilege Escalation
- Defense Evasion
- Credential Access
- Discovery
- Lateral Movement
- Collection
- Command and Control
- Exfiltration
- Impact

Techniques (How):
- 200+ specific techniques
- Sub-techniques for granularity
- Procedure examples from real incidents

Procedures (Implementation):
- Specific implementation details
- Tool usage patterns
- Operational sequences
```

Actor Profiling:
```
Behavioral Patterns:
- Target selection (industry, geography)
- Campaign timing (working hours, holidays)
- Tool preferences
- Language artifacts (comments, UI)
- Operational security level

Capability Assessment:
- Technical sophistication
- Resource availability
- Custom tool development
- Zero-day access
- Persistence duration
```

#### 7.2.2 Attribution Confidence

Confidence Levels:
```
Low (20-40%):
- Limited technical indicators
- Common tools/techniques
- No operational pattern match
- High false flag potential

Medium (40-70%):
- Multiple technical indicators
- Some TTP correlation
- Partial infrastructure overlap
- Moderate confidence in motivation

High (70-90%):
- Extensive technical evidence
- Strong TTP correlation
- Unique tool signatures
- Clear motivation and capability alignment

Very High (90-100%):
- Overwhelming technical evidence
- Perfect TTP match
- Unique operational signatures
- Corroborated by multiple sources
- Geopolitical context alignment
```

Attribution Challenges:
```
False Flags:
- Deliberate misdirection
- Code/tool reuse
- Language/timezone manipulation
- Infrastructure compromise

Attribution Difficulties:
- Proxy/VPN usage
- Tor/anonymization networks
- Compromised infrastructure
- Tool sharing among actors
- Ransomware-as-a-Service (RaaS)
```

---

## 8. Legal Frameworks

### 8.1 International Law

#### 8.1.1 Geneva Conventions (Cyber Context)

Principles:
```
Distinction:
- Must distinguish between military and civilian targets
- Cyber attacks on civilian infrastructure = war crime

Proportionality:
- Collateral damage must not be excessive
- Must weigh military advantage vs civilian harm

Necessity:
- Attack must be necessary for military objective
- No alternative less harmful methods

Humanity:
- Prohibit unnecessary suffering
- Respect for human dignity
```

#### 8.1.2 Tallinn Manual

Tallinn Manual 2.0 Key Rules:
```
Rule 1: Sovereignty
- State cyber operations violating another state's sovereignty are prohibited

Rule 10: Due Diligence
- States must not allow their territory for cyber operations harmful to other states

Rule 13: Countermeasures
- States may take proportionate countermeasures in response to cyber operations

Rule 68: Armed Attack
- Cyber operations causing death, injury, or significant destruction = armed attack
- Triggers right of self-defense (UN Charter Article 51)

Rule 71: Cyber Espionage
- Peacetime cyber espionage not prohibited by international law
- But may violate domestic law
```

#### 8.1.3 Budapest Convention

Convention on Cybercrime (2001):
```
Criminal Offenses:
1. Illegal access (unauthorized access)
2. Illegal interception (data interception)
3. Data interference (deletion, alteration)
4. System interference (DDoS, malware)
5. Misuse of devices (hacking tools)
6. Computer-related forgery
7. Computer-related fraud
8. Child pornography
9. Copyright infringement

Procedural Powers:
- Expedited preservation of data
- Production orders
- Search and seizure
- Real-time traffic data collection
- Interception of content data

International Cooperation:
- Mutual legal assistance
- Extradition
- 24/7 network of contact points
```

### 8.2 National Laws

#### 8.2.1 United States

Computer Fraud and Abuse Act (CFAA):
```
18 U.S.C. § 1030 Prohibits:
(a)(1) Unauthorized access to obtain classified information
(a)(2) Unauthorized access to obtain information from financial/government computers
(a)(3) Unauthorized access to non-public government computers
(a)(4) Fraud via unauthorized access
(a)(5) Damage via unauthorized access
(a)(6) Trafficking in passwords
(a)(7) Extortion involving threat to damage computer

Penalties:
- First offense: Up to 5 years
- Repeat offense: Up to 10 years
- Damage > $5,000: Felony charges
```

#### 8.2.2 European Union

GDPR (Data Protection):
```
Cybersecurity Obligations:
- Article 32: Security of processing
- Article 33: Breach notification (72 hours)
- Article 34: Communication to data subjects

NIS Directive (Network and Information Security):
- Critical infrastructure protection
- Incident reporting requirements
- Security risk management
```

### 8.3 Rules of Engagement (ROE)

Cyber ROE Considerations:
```
Authorization Levels:
- Strategic: National leadership
- Operational: Military command
- Tactical: Unit/team level


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
