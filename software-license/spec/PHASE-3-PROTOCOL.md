# WIA-COMP-016 — Phase 3: Protocol

> Software-license canonical Phase 3: protocols (compatibility + dependency resolution + change events).

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


## 4. License Compatibility

### 4.1 Compatibility Matrix

```
Compatibility Table:
(Outbound License → Inbound License)

            | MIT | Apache | BSD | LGPL | GPL | AGPL | Proprietary
------------|-----|--------|-----|------|-----|------|------------
MIT         |  ✓  |   ✓    |  ✓  |  ✓   |  ✓  |  ✓   |     ✓
Apache-2.0  |  ✓  |   ✓    |  ✓  |  ✓   |  ✓* |  ✓*  |     ✓
BSD         |  ✓  |   ✓    |  ✓  |  ✓   |  ✓  |  ✓   |     ✓
LGPL        |  ✓  |   ✓    |  ✓  |  ✓   |  ✓  |  ✓   |   ✓(link)
GPL         |  ✗  |   ✗    |  ✗  |  ✗   |  ✓  |  ✓   |     ✗
AGPL        |  ✗  |   ✗    |  ✗  |  ✗   |  ✓  |  ✓   |     ✗
Proprietary |  ✓  |   ✓    |  ✓  | ✓(link)| ✗ |  ✗   |   Custom

* GPL-3.0 only, not GPL-2.0
```

### 4.2 Compatibility Rules

**Rule 1: Permissive → Any**
```
Permissive licenses (MIT, BSD, Apache) can be included in any project,
including proprietary software.
```

**Rule 2: Weak Copyleft → Same or GPL**
```
LGPL code can be linked with proprietary, but modifications to LGPL code
must remain LGPL. Can be upgraded to GPL.
```

**Rule 3: Strong Copyleft → Same Only**
```
GPL code requires the entire combined work to be licensed under GPL.
Cannot be combined with proprietary code.
```

**Rule 4: Apache-2.0 ← → GPL**
```
Apache-2.0 is compatible with GPL-3.0 but NOT GPL-2.0 due to patent
clause differences.
```

### 4.3 Compatibility Checking Algorithm

```python
def check_compatibility(main_license, dep_license):
    """
    Check if dependency license is compatible with main license
    """
    # Permissive licenses are compatible with everything
    if dep_license in PERMISSIVE:
        return True

    # GPL/AGPL require matching license
    if main_license in STRONG_COPYLEFT:
        return dep_license in COMPATIBLE_WITH_GPL

    # Proprietary cannot use copyleft
    if main_license == "Proprietary":
        if dep_license in STRONG_COPYLEFT:
            return False
        if dep_license in WEAK_COPYLEFT:
            return True  # Can link
        return True

    # Default: check compatibility matrix
    return COMPATIBILITY_MATRIX[main_license][dep_license]
```

---



## 6. Dependency Management

### 6.1 Dependency Scanning

**Package.json Analysis (Node.js):**
```json
{
  "name": "my-project",
  "dependencies": {
    "express": "^4.18.0",      // MIT
    "mongoose": "^7.0.0",       // MIT
    "react": "^18.0.0"          // MIT
  },
  "devDependencies": {
    "webpack": "^5.75.0"        // MIT
  }
}
```

**Scanning Process:**
```
1. Parse package.json (or requirements.txt, go.mod, etc.)
2. Resolve all dependencies (including transitive)
3. Extract license information from each package
4. Build dependency tree with licenses
5. Check compatibility with main license
6. Generate report
```

### 6.2 License Conflict Detection

```
Conflict Example:
Main Project: MIT
Dependencies:
  ├─ lib-a (MIT) ✓
  ├─ lib-b (Apache-2.0) ✓
  └─ lib-c (GPL-3.0) ✗ CONFLICT!

Issue: MIT is permissive but GPL-3.0 is copyleft
Resolution: Either change main license to GPL or replace lib-c
```

### 6.3 Transitive Dependencies

```
Dependency Tree:
MyProject (Apache-2.0)
└─ PackageA (MIT)
   ├─ PackageB (BSD-3-Clause) ✓
   └─ PackageC (GPL-2.0) ✗ CONFLICT!

Even though PackageA is MIT, its dependency PackageC creates conflict.
Must analyze entire dependency tree.
```

---




---

## A.1 License-compatibility matrix

The compatibility protocol takes the inbound license set (from the SBOM) and the outbound license declaration and returns a compatibility verdict (compatible / conditionally-compatible / incompatible) plus the reasoning. The reference matrix is published at `https://wiastandards.com/software-license/compatibility-matrix.json` and is versioned. Conditionally-compatible verdicts carry a remediation block (use a different inbound version, dual-license outbound, ship as separate process, etc.).

## A.2 Copyleft trigger detection

Strong copyleft licenses (GPL, AGPL) are triggered by distribution; AGPL additionally by network use. The trigger detector inspects the deployment topology declared in Phase 4 (server, on-premise distribution, end-user binary, SaaS) and flags components whose copyleft trigger fires. The flag is non-blocking by default; tenants configure block-on-trigger policy per project.

## A.3 Dependency resolution protocol

Dependency resolution walks the project's lockfile (package-lock.json, yarn.lock, Cargo.lock, go.sum, requirements.txt with hashes, Pipfile.lock, etc.) and produces a resolved tree with one license per node. Transitive dependencies inherit their declared license; ambiguous nodes are tagged for human review. The protocol caches resolution results keyed by lockfile hash so re-checks are O(1) until the lockfile changes.

## A.4 Vulnerability and license overlap

Vulnerable dependencies (NVD CVE matches via OSV) are reported alongside their license; integrators commonly need both signals at once for upgrade decisions. The protocol layer joins both sources so the developer dashboard surfaces a single "this dependency needs attention because (a) license incompatibility, (b) CVE, or (c) both".

## A.5 License-change protocol

When an upstream dependency relicenses (e.g. MongoDB → SSPL, ElasticSearch → ELv2/SSPL, Redis → RSALv2/SSPL), the protocol fires a `license.changed` event for every project that consumed the prior version. Consumers subscribe via webhook and re-run the compatibility check against their outbound license.

## A.6 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for control-plane calls. Compatibility-check submissions are idempotent on the SBOM hash; duplicate submissions short-circuit to the cached result.

## A.7 Patent-grant tracking

Apache-2.0 §3 and similar patent-grant clauses are tracked separately from the license-text body so projects that consume Apache-2.0 dependencies can answer "do I have a patent grant covering this code?" The tracker carries the grant scope (claims practiced by the contribution), the granting entity, and the revocation conditions. Revocation events fire `patent.revoked` webhook deliveries to dependent projects.

## A.8 Contributor License Agreement (CLA / DCO)

CLA records track the agreement type (CLA-individual, CLA-corporate, DCO sign-off), the agreeing party, the agreement date, and the covered repositories. CLA enforcement integrates with GitHub, GitLab, and Bitbucket so pull requests from non-signing contributors are blocked at merge gate. DCO sign-off (Linux-style `Signed-off-by`) is verified on every commit.

## A.9 Reproducible build hash binding

Each component in the SBOM carries the source hash, the build configuration hash, and the resulting artefact hash. Reproducible-build verification re-runs the build from source and compares the artefact hash; mismatches surface as `reproducibility.failed` events. The verification result is itself signed by the verifying entity so downstream consumers can trust the chain.

## A.10 Provenance attestation (in-toto / SLSA)

Build provenance follows in-toto attestations, ideally at SLSA Level 3. The attestation carries the build platform identity, the source-of-truth commit, the build configuration, the materials (inputs) and products (outputs). The attestation is signed (Sigstore/Cosign) and bound to the SBOM through a shared subject digest. The compatibility-check protocol consumes the attestation to verify that the declared license matches the license the build actually consumed.

## A.11 License-relicensing migration playbook

When an upstream relicenses (MongoDB → SSPL, ElasticSearch → ELv2/SSPL, Redis → RSALv2/SSPL, HashiCorp BSL, MariaDB BSL), affected projects receive a `license.changed` event (Phase 3 §A.5). The migration playbook walks decision branches: (1) accept the new license; (2) pin the last permissively-licensed version; (3) fork the last permissively-licensed version; (4) migrate to a permissively-licensed alternative. The playbook captures the trade-offs and helps projects choose deliberately rather than by default.


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
