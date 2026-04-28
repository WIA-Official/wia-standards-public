# WIA-COMP-016 — Phase 1: Data Format

> Software-license canonical Phase 1: data shapes (SPDX + taxonomy + SBOM + templates).

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


## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive framework for software licensing, enabling developers to properly license their software, verify compliance, check compatibility, and manage complex multi-license scenarios.

### 1.2 Scope

The standard covers:
- Open-source and proprietary license types
- SPDX (Software Package Data Exchange) integration
- License compatibility checking
- Automated dependency scanning
- Compliance reporting and auditing
- License template management

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Proper software licensing promotes transparency, protects intellectual property rights, enables collaboration, and ensures fair use of software resources for the benefit of all.

### 1.4 Terminology

- **SPDX**: Software Package Data Exchange standard
- **Copyleft**: License requiring derivative works to use same license
- **Permissive**: License with minimal restrictions
- **Dual-Licensing**: Offering software under multiple licenses
- **SBOM**: Software Bill of Materials
- **Inbound License**: License of dependencies
- **Outbound License**: License of your software

---



## 2. License Types and Categories

### 2.1 License Classification

```
License Taxonomy:

1. Permissive Licenses
   - Minimal restrictions
   - Allows proprietary derivatives
   - Examples: MIT, Apache-2.0, BSD

2. Weak Copyleft
   - File/library-level copyleft
   - Linking with proprietary allowed
   - Examples: LGPL, MPL

3. Strong Copyleft
   - Full copyleft requirement
   - All derivatives must be same license
   - Examples: GPL, AGPL

4. Public Domain
   - No copyright claimed
   - No restrictions
   - Examples: CC0, Unlicense

5. Proprietary
   - All rights reserved
   - Custom terms
   - Examples: Commercial EULA
```

### 2.2 Common Open Source Licenses

**MIT License:**
```
Characteristics:
- Permissive
- Very short and simple
- Requires copyright notice
- No patent grant
- Compatible with everything
```

**Apache License 2.0:**
```
Characteristics:
- Permissive
- Explicit patent grant
- Requires attribution
- Includes trademark restrictions
- Industry standard for enterprise
```

**GNU GPL-3.0:**
```
Characteristics:
- Strong copyleft
- Requires source distribution
- Patent protection
- Anti-tivoization
- Incompatible with GPL-2.0 (without explicit permission)
```

**LGPL-3.0:**
```
Characteristics:
- Weak copyleft
- Allows linking with proprietary
- Source required for library modifications
- Used for libraries
```

### 2.3 License Properties

| License | Type | Attribution | Source Disclosure | Patent Grant | Commercial Use |
|---------|------|-------------|-------------------|--------------|----------------|
| MIT | Permissive | Required | No | No | Yes |
| Apache-2.0 | Permissive | Required | No | Yes | Yes |
| BSD-3-Clause | Permissive | Required | No | No | Yes |
| GPL-3.0 | Strong Copyleft | Required | Yes | Yes | Yes |
| LGPL-3.0 | Weak Copyleft | Required | Modifications | Yes | Yes |
| MPL-2.0 | Weak Copyleft | Required | Files | Yes | Yes |
| AGPL-3.0 | Network Copyleft | Required | Yes (network) | Yes | Yes |
| Proprietary | Commercial | N/A | No | Custom | Restricted |

---



## 3. SPDX Integration

### 3.1 SPDX Overview

The Software Package Data Exchange (SPDX) is an open standard for communicating software bill of material information.

**SPDX Document Structure:**
```json
{
  "spdxVersion": "SPDX-2.3",
  "dataLicense": "CC0-1.0",
  "SPDXID": "SPDXRef-DOCUMENT",
  "name": "ProjectName",
  "documentNamespace": "https://example.com/spdx/project-1.0.0",
  "creationInfo": {
    "created": "2025-12-27T00:00:00Z",
    "creators": ["Tool: WIA-COMP-016-1.0.0"]
  },
  "packages": [],
  "relationships": []
}
```

### 3.2 SPDX Identifiers

**Short Identifiers:**
```
Examples:
- MIT
- Apache-2.0
- GPL-3.0-or-later
- LGPL-2.1-only
- BSD-3-Clause
- Proprietary
```

**SPDX Expressions:**
```
Operators:
- AND: Multiple licenses apply (MIT AND Apache-2.0)
- OR: Choice of licenses (MIT OR Apache-2.0)
- WITH: License exception (GPL-2.0-or-later WITH Classpath-exception-2.0)

Examples:
- (MIT OR Apache-2.0)
- (GPL-2.0-only WITH GCC-exception-2.0)
- (LGPL-2.1-only AND MIT AND BSD-2-Clause)
```

### 3.3 Package Information

```json
{
  "SPDXID": "SPDXRef-Package-MyLib",
  "name": "MyLib",
  "versionInfo": "1.0.0",
  "downloadLocation": "https://github.com/example/mylib",
  "filesAnalyzed": true,
  "licenseConcluded": "MIT",
  "licenseDeclared": "MIT",
  "copyrightText": "Copyright 2025 SmileStory Inc.",
  "supplier": "Organization: SmileStory Inc.",
  "checksums": [
    {
      "algorithm": "SHA256",
      "checksumValue": "abc123..."
    }
  ]
}
```

### 3.4 Relationship Types

```
SPDX Relationships:
- DESCRIBES: Document describes package
- CONTAINS: Package contains file
- DEPENDENCY_OF: Package is dependency of another
- BUILD_DEPENDENCY_OF: Build-time dependency
- RUNTIME_DEPENDENCY_OF: Runtime dependency
- GENERATED_FROM: File generated from source
```

---



## 8. License Templates

### 8.1 MIT Template

```
MIT License

Copyright (c) {year} {copyright_holder}

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### 8.2 Apache-2.0 Header

```
Copyright {year} {copyright_holder}

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

### 8.3 GPL-3.0 Header

```
{program_name} - {description}
Copyright (C) {year}  {copyright_holder}

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

---




---

## A.1 SPDX expression canonical form

WIA-conformant license declarations use SPDX expressions in their canonical form: a single SPDX-License-Identifier value (e.g. `Apache-2.0`), a parenthesised disjunction (`(MIT OR Apache-2.0)`), or a parenthesised conjunction with optional WITH clauses (`(GPL-2.0-or-later WITH Classpath-exception-2.0)`). Parsers MUST follow the SPDX 3.x grammar and MUST reject unknown identifiers unless an explicit `LicenseRef-` prefix is used.

## A.2 License taxonomy

License taxonomy: permissive (MIT, Apache-2.0, BSD-3-Clause), weak copyleft (LGPL-3.0-or-later, MPL-2.0), strong copyleft (GPL-3.0-or-later, AGPL-3.0-or-later), public-domain-equivalent (CC0-1.0, Unlicense), and proprietary (commercial, internal-use-only, evaluation). Each taxonomy slot maps to a behavioural profile that downstream tooling consumes (modify-allowed, redistribute-allowed, sublicense-allowed, patent-grant, network-use-trigger, etc.).

## A.3 SBOM envelope (CycloneDX + SPDX)

The Software Bill of Materials carries a unique identifier, the producing tool, the document timestamp, and a list of components. Each component carries a name, version, supplier, license expression, copyright statement, hash set (SHA-1, SHA-256, SHA-512), and an optional pURL identifier. WIA-conformant SBOMs are emitted in CycloneDX 1.5 JSON or SPDX 2.3 / 3.0 tag-value or JSON-LD; consumers MUST accept both formats.

## A.4 License-template registry

A registry of pre-approved license templates lives at `https://wiastandards.com/software-license/templates/`. Each entry carries the SPDX identifier, the canonical text, the canonical text hash (SHA-256), the recommended copyright header, and a list of fields the project MUST fill (year, author, project name). Tooling fills the fields and re-checks the hash to detect tampering.

## A.5 Identifier and metadata fields

Every license declaration MUST carry: SPDX identifier, source-of-truth path (LICENSE file, SPDX tag in source headers, package metadata), declaring entity (legal name + contact), declaration timestamp, and an optional rationale field. The rationale is consumed by audit (Phase 4) when the declared license differs from the upstream-detected license.

## A.6 Copyright header policy

Source files MUST carry a copyright header that names the copyright holder, the year (or year range), and the SPDX-License-Identifier. The reference header generator at `cli/license-header.sh` emits a header per project policy; CI integrations verify that every committed file carries a conformant header before allowing merge. Generated files (vendored dependencies, build outputs) are excluded via a `.licenseignore` file in the repository root.

## A.7 Dual-licensing envelope

Projects that ship under multiple licenses (commercial + open-source dual-licensing, or AGPL-3.0 + commercial-exception dual-licensing) declare both licenses in a single SPDX expression: `(AGPL-3.0-or-later OR LicenseRef-Commercial-2025)`. The commercial-license terms are referenced via `LicenseRef-` and resolved against the commercial-license registry (Phase 2 §A.5).


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
