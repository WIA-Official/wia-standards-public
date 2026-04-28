# WIA-COMP-016 — Phase 4: Integration

> Software-license canonical Phase 4: ecosystem integration (audit + OSPO + CI/CD + cross-walk).

# WIA-COMP-016: Software License Specification v1.0

> **Standard ID:** WIA-COMP-016
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Computing Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [License Types and Categories](#2-license-types-and-categories)
3. [SPDX Integration](#3-spdx-integration)
4. [License Compatibility](#4-license-compatibility)
5. [Commercial Licensing](#5-commercial-licensing)
6. [Dependency Management](#6-dependency-management)
7. [Compliance and Auditing](#7-compliance-and-auditing)
8. [License Templates](#8-license-templates)
9. [Copyright Management](#9-copyright-management)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [References](#11-references)

---


## 7. Compliance and Auditing

### 7.1 Compliance Requirements

**Attribution Requirements:**
```
For MIT/BSD/Apache:
1. Include copyright notice
2. Include license text
3. Maintain NOTICE file (Apache)

For GPL/LGPL:
1. Include copyright notice
2. Include license text
3. Provide source code or written offer
4. Document modifications
```

### 7.2 Compliance Checklist

```
License Compliance Checklist:
□ All dependencies identified
□ All licenses documented
□ License compatibility verified
□ Required notices included
□ NOTICE file created (if needed)
□ Source disclosure prepared (if GPL)
□ Attribution list generated
□ SBOM generated
□ Legal review completed
```

### 7.3 Audit Reports

```
Audit Report Structure:
1. Executive Summary
   - Total dependencies
   - License distribution
   - Risk level

2. Detailed Findings
   - Each dependency with license
   - Compatibility status
   - Required actions

3. Compliance Status
   - Compliant items
   - Non-compliant items
   - Recommendations

4. Action Items
   - License changes needed
   - Dependencies to replace
   - Documentation to add
```

---



## 10. Implementation Guidelines

### 10.1 Project Setup

**Step 1: Choose License**
```
Decision Tree:
1. Open source? → Yes
   a. Allow proprietary use? → Yes → MIT or Apache-2.0
   b. Require open source? → Yes → GPL-3.0
2. Proprietary? → Yes → Custom EULA
```

**Step 2: Add License Files**
```bash
project/
├── LICENSE          # Full license text
├── NOTICE          # Additional notices (Apache)
├── COPYRIGHT       # Copyright statement
└── README.md       # License badge
```

**Step 3: Add Headers**
```
Add license header to all source files:
- Use SPDX identifier: // SPDX-License-Identifier: MIT
- Or full header comment
```

### 10.2 Dependency Scanning

```bash
# Node.js
npm install -g license-checker
license-checker --json > licenses.json

# Python
pip install pip-licenses
pip-licenses --format=json --output-file=licenses.json

# Go
go-licenses csv ./... > licenses.csv
```

### 10.3 SBOM Generation

```typescript
// Generate SPDX SBOM
const sbom = generateSPDX({
  project: 'MyProject',
  version: '1.0.0',
  license: 'MIT',
  dependencies: scanDependencies('./package.json')
});

fs.writeFileSync('SBOM.spdx.json', JSON.stringify(sbom, null, 2));
```

---




---

## A.1 Compliance and auditing

The audit protocol takes a project's declared license, its SBOM, its source-tree state, and produces a signed compliance report. The report enumerates: declared outbound license, detected outbound license (from header scanning + LICENSE file parsing), discrepancies, transitive license set with conflict flags, copyleft trigger status given the declared deployment topology, and outstanding remediation items. Reports are valid for 90 days and re-issued on lockfile change.

## A.2 Open-source program office (OSPO) integration

Enterprise OSPO integrations exchange compliance reports over an authenticated channel. The integration covers project intake (new project obeys the policy from day one), exception requests (approver workflow with timeline), and continuous compliance (nightly re-scan plus alert on drift). The reference adapter targets the Linux Foundation's FOSSology, but the protocol is provider-neutral.

## A.3 CI / CD integration

GitHub Actions, GitLab CI, Bitbucket Pipelines, Jenkins, and Drone adapters live under `cli/integrations/`. The adapters run the SBOM submission → compatibility check → audit report sequence on every pull request and on every release tag. Failing checks block merge per the project's policy; advisory checks annotate the pull request without blocking.

## A.4 Cross-walk to international standards

| Concern                  | Standard                                                       |
|--------------------------|----------------------------------------------------------------|
| SBOM format              | ISO/IEC 5962:2021 (SPDX), ECMA-424 (CycloneDX-equivalent)      |
| Open-source identification | OpenChain ISO/IEC 5230:2020                                  |
| Vulnerability disclosure | ISO/IEC 29147:2018, ISO/IEC 30111:2019                         |
| Cryptographic licensing  | IETF RFC 5280 (X.509), RFC 8032 (Ed25519)                      |
| Copyright registry       | WIPO Copyright Treaty, US 17 U.S.C., EU Directive 2001/29/EC   |
| Commercial license terms | UCITA (US, where adopted), CISG (international sales)          |

The cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.5 Reference container, CLI, governance

The reference container at `wia/software-license-host:1.0.0` ships every Phase 2 endpoint with mock data and feeds the conformance suite. The companion CLI at `cli/software-license.sh` ships sample envelope generators for declarations, SBOM submissions, and audit reports. WIA Standards composition: WIA-INTENT for workload intent, WIA-OMNI-API for credential storage, WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for federation handshake.

## A.6 Reference list

- ISO/IEC 5962:2021 — SPDX 2.2.1
- ISO/IEC 5230:2020 — OpenChain compliance
- IETF RFC 8032 — Ed25519
- IETF RFC 5280 — X.509 PKI
- Apache Software Foundation — Apache 2.0 license text and patent grant
- Free Software Foundation — GPL family license texts
- Open Source Initiative — OSI-approved license list
- Linux Foundation — SPDX, CycloneDX, OpenChain, FOSSology

## A.7 Operational dashboards

Compliance dashboards expose per-project scoreboards: declared license vs. detected license, transitive license set, copyleft trigger status, vulnerability + license overlap, audit freshness, and outstanding remediation items. Dashboards are built on the live Phase 2 endpoints; tenants embed them via OAuth-protected iframes or build their own consumer.

## A.8 OSS-program metric set

Standard OSS-program metrics: percent of projects with conformant declarations (target: 100%), percent of dependencies with detected licenses (target: ≥99%), audit-report median age (target: ≤30 days), copyleft-trigger remediation SLA (target: ≤14 days), CLA / DCO coverage (target: 100%), reproducible-build pass rate (target: ≥95%). Metrics are emitted in OpenTelemetry format so observability stacks consume them without adapter.

## A.9 Tooling ecosystem

Reference adapters and consumers: FOSSology (open-source license scanner), ScanCode Toolkit (component-level license + copyright detection), CycloneDX CLI (SBOM generator), SPDX Tools (SPDX SBOM generator), Tern (container-image SBOM), Syft (multi-format SBOM generator), Grype (vulnerability matcher), Trivy (vulnerability + license scanner), OSV-Scanner (Open Source Vulnerabilities matcher), and Linux Foundation FOSSology workflow. Each adapter consumes the canonical SBOM envelope (Phase 1 §A.3) and emits the canonical analysis envelope (Phase 2 §A.3).

## A.10 Governance and policy templates

Policy templates at `https://wiastandards.com/software-license/policies/` cover: outbound-license selection (permissive vs. copyleft trade-offs), inbound-license allowlist / denylist (per project, per business unit), exception process (who approves, what evidence, what timeline), and the OSS-program reporting cadence (monthly / quarterly / annual). The templates are starting points; tenants customise per their legal-counsel guidance.

## A.11 Implementation runbook

A first deployment typically follows: (1) stand up the reference container; (2) generate an initial SBOM for the canonical project tree; (3) submit the SBOM via Phase 2 §A.3 and review the analysis; (4) declare the outbound license (Phase 2 §A.2); (5) wire CI / CD adapters per Phase 4 §A.3; (6) onboard the audit subscription per Phase 4 §A.1; (7) promote to production with the policy templates from §A.10 customised.

## A.12 Compliance sub-graph for SBOM consumers

Downstream consumers of WIA-licensed components subscribe to the compliance sub-graph for upstream relicensing events, vulnerability disclosures, and reproducibility-failure events. The sub-graph carries one row per `(component, version)` with a `last-updated` timestamp; consumers poll or webhook-subscribe per their preference. The sub-graph is signed by the WIA registry's Ed25519 key so downstream consumers can verify provenance without a separate trust anchor.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/software-license/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-software-license-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/software-license-host:1.0.0` ships every software-license envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/software-license.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Software-license deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
