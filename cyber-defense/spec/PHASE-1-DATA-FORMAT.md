# WIA-DEF-005 PHASE 1 — Data Format Specification

**Standard:** WIA-DEF-005
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-DEF-005: Cyber Defense Specification v1.0

> **Standard ID:** WIA-DEF-005
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense & Security Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Defense Architecture](#2-defense-architecture)
3. [Threat Detection](#3-threat-detection)
4. [Incident Response](#4-incident-response)
5. [Network Hardening](#5-network-hardening)
6. [SOC Operations](#6-soc-operations)
7. [Threat Intelligence](#7-threat-intelligence)
8. [SIEM Integration](#8-siem-integration)
9. [Critical Infrastructure Protection](#9-critical-infrastructure-protection)
10. [Implementation Guidelines](#10-implementation-guidelines)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive cyber defense framework for protecting critical infrastructure, enterprise networks, and sensitive systems from cyber threats. It provides standardized approaches for threat detection, incident response, and security operations.

### 1.2 Scope

The standard covers:
- Multi-layered defense architecture (Defense-in-Depth)
- Real-time threat detection and monitoring
- Structured incident response procedures
- Network hardening and security controls
- Security Operations Center (SOC) workflows
- Threat intelligence integration
- SIEM system requirements
- Critical infrastructure protection measures

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to protect critical infrastructure and essential services that society depends on, ensuring the security, resilience, and continuity of systems that benefit all of humanity.

### 1.4 Terminology

- **IOC (Indicator of Compromise)**: Evidence of a security breach or malicious activity
- **TTPs (Tactics, Techniques, and Procedures)**: Methods used by threat actors
- **SOC (Security Operations Center)**: Centralized unit for monitoring and responding to security incidents
- **SIEM (Security Information and Event Management)**: System for collecting and analyzing security logs
- **EDR (Endpoint Detection and Response)**: Endpoint security solution with advanced threat detection
- **Zero Trust**: Security model requiring strict verification for all users and devices
- **Defense-in-Depth**: Layered security approach with multiple defensive mechanisms

---

## 2. Defense Architecture

### 2.1 Seven-Layer Defense Model

The WIA-DEF-005 defense architecture implements a seven-layer security model:

```
┌──────────────────────────────────────────────────────┐
│ Layer 7: Application Security                       │
│   - Web Application Firewall (WAF)                  │
│   - API Gateway & Rate Limiting                     │
│   - Input Validation & Sanitization                 │
└──────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────┐
│ Layer 6: Data Security                              │
│   - Encryption (at rest & in transit)               │
│   - Data Loss Prevention (DLP)                      │
│   - Database Activity Monitoring (DAM)              │
└──────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────┐
│ Layer 5: Endpoint Security                          │
│   - Endpoint Detection & Response (EDR)             │
│   - Antivirus & Anti-malware                        │
│   - Host-based Firewall & IPS                       │
└──────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────┐
│ Layer 4: Network Security                           │
│   - Next-Gen Firewall (NGFW)                        │
│   - Intrusion Detection/Prevention (IDS/IPS)        │
│   - Network Segmentation & VLANs                    │
└──────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────┐
│ Layer 3: Identity & Access Management               │
│   - Multi-Factor Authentication (MFA)               │
│   - Role-Based Access Control (RBAC)                │
│   - Privileged Access Management (PAM)              │
└──────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────┐
│ Layer 2: Infrastructure Security                    │
│   - Server & OS Hardening                           │
│   - Patch Management                                │
│   - Configuration Management                        │
└──────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────┐
│ Layer 1: Physical Security                          │
│   - Facility Access Control                         │
│   - Hardware Security Modules (HSM)                 │
│   - Environmental Monitoring                        │
└──────────────────────────────────────────────────────┘
```

### 2.2 Defense Score Calculation

Each layer receives a security score (0-100), and the overall defense posture is calculated as:

```
Defense_Score = Σ(Layer_Score × Layer_Weight) / Σ(Layer_Weight)

Where Layer_Weight:
  Layer 7 (Application): 0.15
  Layer 6 (Data): 0.20
  Layer 5 (Endpoint): 0.15
  Layer 4 (Network): 0.15
  Layer 3 (Identity): 0.20
  Layer 2 (Infrastructure): 0.10
  Layer 1 (Physical): 0.05
```

### 2.3 Zero Trust Architecture

Implementation of Zero Trust principles:

```
1. Verify explicitly
   - Authenticate every access request
   - Use all available data points (identity, location, device, etc.)

2. Use least privilege access
   - Just-in-Time (JIT) access
   - Just-Enough-Access (JEA)
   - Risk-based adaptive policies

3. Assume breach
   - Minimize blast radius with segmentation
   - Verify end-to-end encryption
   - Use analytics for threat detection
```

---

## 3. Threat Detection

### 3.1 Threat Severity Classification

```
CRITICAL (Score: 90-100)
  - Active data exfiltration
  - Ransomware encryption in progress
  - Root/Admin compromise
  - Critical infrastructure disruption

HIGH (Score: 70-89)
  - Exploit attempts on critical systems
  - Malware detected on production servers
  - Privilege escalation attempts
  - Lateral movement detected

MEDIUM (Score: 40-69)
  - Suspicious network traffic patterns
  - Failed authentication attempts (brute force)
  - Policy violations
  - Outdated/vulnerable software detected

LOW (Score: 20-39)
  - Anomalous behavior (low confidence)
  - Minor policy violations
  - Reconnaissance scanning
  - Informational alerts

INFO (Score: 0-19)
  - Normal operations logging
  - Baseline behavior
  - Audit trail events
```

### 3.2 Detection Methods

#### 3.2.1 Signature-Based Detection

```
Detection Rule Format:
{
  "rule_id": "RULE-001",
  "name": "SQL Injection Attempt",
  "pattern": "/(union|select|insert|update|delete|drop).*from/i",
  "severity": "HIGH",
  "action": "BLOCK_AND_ALERT"
}
```

#### 3.2.2 Anomaly-Based Detection

Machine learning models for behavioral analysis:

```
Anomaly_Score = Σ(Feature_Deviation × Feature_Weight)

Features:
  - Network traffic volume (weight: 0.20)
  - Connection patterns (weight: 0.15)
  - Authentication patterns (weight: 0.25)
  - Data access patterns (weight: 0.25)
  - System resource usage (weight: 0.15)

Threshold: Anomaly_Score > 0.75 → Alert
```

#### 3.2.3 Threat Intelligence Matching


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
