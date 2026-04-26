# WIA-DEF-005 PHASE 2 — API Interface Specification

**Standard:** WIA-DEF-005
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

```
IOC Matching Process:
1. Collect IOCs from threat feeds
2. Normalize IOC format (IP, domain, hash, URL)
3. Match against network/system logs
4. Calculate confidence score
5. Generate alert if confidence > threshold
```

### 3.3 Detection Metrics

Key Performance Indicators:

```
- Mean Time to Detect (MTTD): < 5 minutes
- Mean Time to Respond (MTTR): < 30 minutes
- False Positive Rate: < 5%
- Detection Coverage: > 95% of MITRE ATT&CK techniques
- Alert Volume: Optimized to < 100 actionable alerts/day
```

---

## 4. Incident Response

### 4.1 Incident Response Lifecycle

```
1. PREPARATION
   ├─ Develop IR plan and playbooks
   ├─ Train IR team
   ├─ Establish communication channels
   └─ Pre-configure tools and access

2. DETECTION & ANALYSIS
   ├─ Monitor security alerts
   ├─ Analyze indicators
   ├─ Determine incident scope
   └─ Classify incident severity

3. CONTAINMENT
   ├─ Short-term containment (isolate affected systems)
   ├─ Evidence preservation
   ├─ Long-term containment (apply patches, reconfigure)
   └─ Network segmentation

4. ERADICATION
   ├─ Remove malware and backdoors
   ├─ Close security gaps
   ├─ Reset compromised credentials
   └─ Verify complete removal

5. RECOVERY
   ├─ Restore systems from clean backups
   ├─ Rebuild compromised systems
   ├─ Enhanced monitoring
   └─ Gradual return to normal operations

6. POST-INCIDENT REVIEW
   ├─ Document lessons learned
   ├─ Update IR procedures
   ├─ Improve defenses
   └─ Compliance reporting
```

### 4.2 Incident Severity Matrix

```
┌──────────┬──────────┬────────────┬─────────────┬──────────────┐
│ Impact   │ Data Loss│ Downtime   │ Response SLA│ Escalation   │
├──────────┼──────────┼────────────┼─────────────┼──────────────┤
│ CRITICAL │ > 1GB    │ > 4 hours  │ 15 minutes  │ CISO + Exec  │
│ HIGH     │ > 100MB  │ > 1 hour   │ 1 hour      │ CISO         │
│ MEDIUM   │ > 10MB   │ > 30 min   │ 4 hours     │ SOC Manager  │
│ LOW      │ < 10MB   │ < 30 min   │ 24 hours    │ SOC Analyst  │
└──────────┴──────────┴────────────┴─────────────┴──────────────┘
```

### 4.3 Incident Response Playbooks

#### 4.3.1 Ransomware Response

```
PLAYBOOK: RANSOMWARE-001

1. IMMEDIATE ACTIONS (0-15 minutes)
   □ Isolate infected systems from network
   □ Identify ransomware variant
   □ Preserve forensic evidence
   □ Notify IR team and management

2. CONTAINMENT (15-60 minutes)
   □ Disable network shares
   □ Block C2 communications
   □ Identify patient zero
   □ Scan all endpoints for IOCs

3. ERADICATION (1-4 hours)
   □ Remove ransomware from systems
   □ Reset all credentials
   □ Apply security patches
   □ Update AV signatures

4. RECOVERY (4-24 hours)
   □ Verify backup integrity
   □ Restore from clean backups
   □ Test restored systems
   □ Gradual network reconnection

5. POST-INCIDENT (24-48 hours)
   □ Root cause analysis
   □ Update defenses
   □ Regulatory reporting
   □ Insurance claim filing
```

---

## 5. Network Hardening

### 5.1 Network Segmentation

```
Internet
    │
    ├─ DMZ (Demilitarized Zone)
    │   ├─ Web Servers (Public-facing)
    │   ├─ Mail Servers
    │   └─ DNS Servers
    │
    ├─ Production Network
    │   ├─ Application Servers
    │   ├─ Database Servers
    │   └─ API Gateways
    │
    ├─ Management Network
    │   ├─ Admin Workstations
    │   ├─ Jump Servers
    │   └─ Monitoring Systems
    │
    ├─ Internal Network
    │   ├─ Employee Workstations
    │   ├─ File Servers
    │   └─ Printers
    │
    └─ Critical Infrastructure (Air-Gapped)
        ├─ SCADA Systems
        ├─ Industrial Control Systems
        └─ Safety Systems
```

### 5.2 Firewall Configuration

#### 5.2.1 Default Deny Policy

```
Rule Priority: Explicit Deny > Explicit Allow > Implicit Deny

Default Rules:
1. DENY ALL incoming traffic
2. DENY ALL outgoing traffic
3. ALLOW specific approved traffic (whitelist)
4. LOG all denied connections
```

#### 5.2.2 Firewall Rule Example

```json
{
  "rule_id": "FW-001",
  "name": "Allow HTTPS from Internet to DMZ Web Servers",
  "source": "0.0.0.0/0",
  "destination": "10.1.1.0/24",
  "port": 443,
  "protocol": "TCP",
  "action": "ALLOW",
  "logging": true,
  "enabled": true
}
```

### 5.3 Intrusion Prevention

IPS configuration for critical systems:

```
Detection Modes:
  - Inline Mode: Block malicious traffic in real-time
  - Passive Mode: Alert only (for monitoring)

Signature Updates: Every 4 hours
Custom Signatures: Yes
SSL/TLS Inspection: Enabled for outbound traffic
Geo-blocking: Block traffic from high-risk countries
```

---


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
