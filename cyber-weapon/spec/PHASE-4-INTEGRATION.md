# WIA-DEF-004 PHASE 4 — Integration Specification

**Standard:** WIA-DEF-004
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

Engagement Criteria:
- Target identification (positive ID)
- Collateral damage assessment
- Legal review
- Proportionality assessment

Prohibited Actions:
- Attacks on civilian targets
- Attacks on protected sites (hospitals, cultural)
- Indiscriminate attacks
- Unnecessary destruction

Reporting Requirements:
- Pre-operation approval
- Real-time monitoring
- Post-operation assessment
- Incident documentation
```

---

## 9. Ethical Guidelines

### 9.1 Defensive Ethics

#### 9.1.1 Responsible Disclosure

Vulnerability Disclosure Process:
```
Step 1: Discovery (Day 0)
- Document vulnerability
- Assess severity
- Determine affected systems

Step 2: Vendor Notification (Day 1-7)
- Contact vendor security team
- Provide technical details
- Propose disclosure timeline

Step 3: Vendor Response (Day 7-30)
- Acknowledgment
- Validation
- Patch development timeline

Step 4: Public Disclosure (Day 90)
- Coordinate with vendor
- Publish advisory
- Provide mitigations

Exceptions:
- Critical vulnerabilities: Shorter timeline (30-45 days)
- Actively exploited: Immediate notification to affected parties
- Vendor unresponsive: Public disclosure after reasonable time
```

#### 9.1.2 Penetration Testing Ethics

Ethical Pentesting Principles:
```
1. Authorized Access Only:
   - Written permission required
   - Scope clearly defined
   - Rules of engagement documented

2. Minimize Harm:
   - Avoid service disruption
   - No data destruction
   - Respect for privacy

3. Confidentiality:
   - Protect client information
   - Secure storage of findings
   - NDAs and contracts

4. Professional Standards:
   - OSSTMM, PTES, OWASP
   - Certifications (OSCP, CEH, GPEN)
   - Continuous education

5. Reporting:
   - Comprehensive documentation
   - Actionable recommendations
   - Executive and technical reports
```

### 9.2 Research Ethics

#### 9.2.1 Security Research Guidelines

Ethical Boundaries:
```
Permitted:
✓ Vulnerability research on own systems
✓ Bug bounty program participation
✓ Responsible disclosure
✓ Defensive tool development
✓ Academic research with proper safeguards

Prohibited:
✗ Unauthorized access to systems
✗ Data theft or destruction
✗ Malware distribution
✗ Weaponization without authorization
✗ Exploitation for personal gain
```

Bug Bounty Best Practices:
```
Platform Selection:
- HackerOne, Bugcrowd, Synack
- Vendor-hosted programs

Scope Understanding:
- In-scope assets
- Out-of-scope restrictions
- Prohibited testing methods

Responsible Testing:
- Avoid excessive traffic
- Don't access user data
- Stop upon finding vulnerability
- Document steps carefully

Reward Expectations:
- Critical: $1,000 - $50,000+
- High: $500 - $10,000
- Medium: $100 - $1,000
- Low: Recognition / Swag
```

---

## 10. Defense Strategies

### 10.1 Defense in Depth

#### 10.1.1 Layered Security Model

Seven Layers:
```
Layer 1: Physical Security
- Data center access controls
- Hardware security modules (HSM)
- Secure disposal procedures

Layer 2: Network Security
- Firewalls (stateful, next-gen)
- IDS/IPS (Snort, Suricata)
- Network segmentation (VLANs)
- VPN (IPsec, WireGuard)

Layer 3: Endpoint Security
- Antivirus/EDR (CrowdStrike, SentinelOne)
- Host-based firewalls
- Application whitelisting
- Disk encryption (BitLocker, LUKS)

Layer 4: Application Security
- Secure coding practices (OWASP)
- Input validation
- Output encoding
- Security testing (SAST, DAST)

Layer 5: Data Security
- Encryption at rest (AES-256)
- Encryption in transit (TLS 1.3)
- Data classification
- DLP (Data Loss Prevention)

Layer 6: Identity & Access
- MFA (multi-factor authentication)
- SSO (single sign-on)
- PAM (privileged access management)
- Zero trust architecture

Layer 7: Policies & Procedures
- Security policies
- Incident response plans
- Business continuity plans
- Security awareness training
```

#### 10.1.2 Zero Trust Architecture

Principles:
```
Never Trust, Always Verify:
- Verify every access request
- Assume breach
- Least privilege access
- Micro-segmentation

Implementation:
1. Identity Verification:
   - Strong authentication (MFA)
   - Continuous verification
   - Device trust assessment

2. Device Security:
   - Endpoint compliance checking
   - Health attestation
   - Managed/unmanaged device policies

3. Network Segmentation:
   - Software-defined perimeters
   - Micro-segmentation
   - East-west traffic inspection

4. Data Protection:
   - Classification and labeling
   - Encryption everywhere
   - DLP controls

5. Monitoring & Analytics:
   - SIEM integration
   - UEBA (User and Entity Behavior Analytics)
   - Threat intelligence feeds
```

### 10.2 Incident Response

#### 10.2.1 NIST Incident Response Lifecycle

Four Phases:
```
Phase 1: Preparation
- Incident response team formation
- Tools and resources
- Communication plans
- Training and exercises

Phase 2: Detection & Analysis
- Event monitoring
- Alert triage
- Incident classification
- Initial analysis

Phase 3: Containment, Eradication & Recovery
- Short-term containment (isolate systems)
- Long-term containment (temporary fixes)
- Eradication (remove threat)
- Recovery (restore operations)

Phase 4: Post-Incident Activity
- Lessons learned meeting
- Incident documentation
- Process improvement
- Threat intelligence sharing
```

#### 10.2.2 Cyber Kill Chain

Lockheed Martin Model:
```
1. Reconnaissance
   Detection: Log analysis, threat intelligence
   Mitigation: Minimize public exposure

2. Weaponization
   Detection: Threat intelligence, malware analysis
   Mitigation: Antivirus, email filtering

3. Delivery
   Detection: Email filtering, network monitoring
   Mitigation: Email security, web filtering

4. Exploitation
   Detection: IDS/IPS, endpoint protection
   Mitigation: Patching, hardening

5. Installation
   Detection: EDR, file integrity monitoring
   Mitigation: Application whitelisting

6. Command & Control
   Detection: Network monitoring, DNS filtering
   Mitigation: Firewall rules, proxy filtering

7. Actions on Objectives
   Detection: DLP, behavioral analytics
   Mitigation: Rapid response, containment
```

### 10.3 Threat Intelligence

#### 10.3.1 Intelligence Lifecycle

Five Phases:
```
1. Planning & Direction:
   - Define intelligence requirements
   - Identify information sources
   - Prioritize collection

2. Collection:
   - Internal sources (logs, SIEM)
   - External sources (threat feeds, ISAC)
   - OSINT (open source intelligence)
   - Commercial feeds (Recorded Future, ThreatConnect)

3. Processing:
   - Normalization
   - Enrichment
   - Correlation
   - STIX/TAXII format

4. Analysis & Production:
   - Indicator analysis (IOCs)
   - TTP mapping (MITRE ATT&CK)
   - Threat actor profiling
   - Campaign tracking

5. Dissemination:
   - SIEM integration
   - Security tool feeds
   - Stakeholder reports
   - Information sharing (ISACs)
```

#### 10.3.2 Threat Hunting

Proactive Hunt Methodology:
```
Hypothesis-Driven:
1. Generate hypothesis (based on threat intelligence)
   "Attackers may use PowerShell for lateral movement"

2. Investigate:
   - Query logs for PowerShell execution
   - Analyze command-line arguments
   - Check for encoded commands

3. Uncover patterns:
   - Normal baseline vs anomalies
   - Time-based correlation
   - User/system profiling

4. Inform and enrich:
   - Update detection rules
   - Feed into analytics
   - Share intelligence

Tools:
- Elastic Stack (ELK)
- Splunk
- Microsoft Sentinel
- Velociraptor
- OSQuery
```

---

## 11. References

### 11.1 Standards & Frameworks

| Framework | Organization | Purpose |
|-----------|-------------|---------|
| MITRE ATT&CK | MITRE Corporation | Adversary tactics and techniques |
| NIST CSF | NIST | Cybersecurity framework |
| ISO 27001 | ISO | Information security management |
| CIS Controls | Center for Internet Security | Security best practices |
| OWASP Top 10 | OWASP | Web application security |
| SANS Top 25 | SANS Institute | Software security errors |

### 11.2 Legal Documents

1. **Geneva Conventions** (1949, Updated)
   - International humanitarian law
   - Application to cyber warfare

2. **Tallinn Manual 2.0** (2017)
   - International law applicable to cyber operations
   - NATO CCDCOE publication

3. **Budapest Convention** (2001)
   - Convention on Cybercrime
   - Council of Europe

4. **UN GGE Reports** (2013, 2015)
   - UN Group of Governmental Experts
   - Norms of state behavior in cyberspace

### 11.3 WIA Standards

Related Standards:
- WIA-DEF-001: Defense Infrastructure Standards
- WIA-DEF-002: Military Communication Protocols
- WIA-DEF-003: Electronic Warfare Systems
- WIA-SEC-001: Encryption and Security
- WIA-SEC-002: Authentication Systems
- WIA-OMNI-API: Universal Security API

### 11.4 Threat Intelligence Sources

Open Source:
- AlienVault OTX (Open Threat Exchange)
- MISP (Malware Information Sharing Platform)
- VirusTotal
- AbuseIPDB
- URLhaus

Commercial:
- Recorded Future
- Mandiant Threat Intelligence
- CrowdStrike Falcon Intelligence
- Palo Alto Networks Unit 42
- Microsoft Threat Intelligence

### 11.5 Key Publications

1. **"The Cuckoo's Egg"** - Clifford Stoll (1989)
   Early cyber espionage case study

2. **"Countdown to Zero Day"** - Kim Zetter (2014)
   Stuxnet analysis and attribution

3. **"Sandworm"** - Andy Greenberg (2019)
   Russian cyber warfare campaigns

4. **"This Is How They Tell Me the World Ends"** - Nicole Perlroth (2021)
   Zero-day market and cyber weapons proliferation

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-DEF-004 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**IMPORTANT LEGAL NOTICE**

This specification is provided for **defensive cybersecurity purposes only**. All information contained herein is intended to:

1. Educate security professionals
2. Improve defensive capabilities
3. Promote responsible security practices
4. Support compliance with international law

**Prohibited Uses:**
- Unauthorized access to computer systems
- Development or distribution of malware
- Cyber attacks or offensive operations
- Any violation of applicable laws

Users must comply with all applicable local, national, and international laws including but not limited to:
- Computer Fraud and Abuse Act (CFAA) - United States
- Computer Misuse Act - United Kingdom
- Cybercrime Act - Australia
- Budapest Convention on Cybercrime
- Geneva Conventions (as applicable to cyber operations)
- Tallinn Manual guidelines

The authors and WIA disclaim all liability for misuse of this information. This standard is published under the MIT License for lawful defensive purposes only.


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
