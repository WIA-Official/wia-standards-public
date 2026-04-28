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

## 5. Commercial Licensing

### 5.1 Dual Licensing

**Model:**
```
Dual License Structure:
1. Open Source License (e.g., AGPL-3.0)
   - Free for open source use
   - Requires source disclosure

2. Commercial License
   - Paid license
   - No source disclosure required
   - Additional support and features
```

**Example: MySQL Model**
```
Options:
- GPL: Free, must open source your application
- Commercial: Paid, can keep your application proprietary
```

### 5.2 License Tiers

```
Common Licensing Tiers:

1. Community Edition
   - Open source (MIT, Apache, GPL)
   - Basic features
   - Community support

2. Professional Edition
   - Commercial license
   - Advanced features
   - Email support
   - $99-999/year

3. Enterprise Edition
   - Commercial license
   - All features
   - SLA, phone support
   - Custom pricing
```

### 5.3 Proprietary License Terms

```
Typical EULA Components:
1. Grant of License
   - Scope of use
   - Number of users/devices
   - Territorial restrictions

2. Restrictions
   - No reverse engineering
   - No redistribution
   - No modification

3. Ownership
   - Vendor retains all rights
   - Customer has usage rights only

4. Warranty and Liability
   - Limited or no warranty
   - Liability disclaimers

5. Term and Termination
   - License duration
   - Termination conditions
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

## 9. Copyright Management

### 9.1 Copyright Ownership

```
Copyright Holder Types:
1. Individual: "Copyright 2025 John Doe"
2. Company: "Copyright 2025 SmileStory Inc."
3. Multiple: "Copyright 2025 John Doe and SmileStory Inc."
4. Range: "Copyright 2020-2025 SmileStory Inc."
```

### 9.2 Copyright Transfer

```
Copyright Assignment:
- Contributor License Agreement (CLA)
- Assigns copyright to project owner
- Required by some projects (Apache, Google)

Example CLA:
"I hereby assign copyright in my contributions to {Project},
 to be licensed under {License}."
```

### 9.3 Copyright Notice

```
Placement:
1. LICENSE file: Full license text
2. README: License badge and summary
3. Source files: Header comment
4. NOTICE: Additional attribution
5. About dialog: In application UI
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

## 11. References

### Standards Bodies
- **SPDX**: [spdx.org](https://spdx.org)
- **OSI**: Open Source Initiative
- **FSF**: Free Software Foundation
- **Apache Foundation**: apache.org

### Resources
1. "Choose an Open Source License" - choosealicense.com
2. "SPDX Specification" - spdx.github.io/spdx-spec
3. "Open Source License Compliance" - GitHub Guide
4. "GPL Compatibility" - gnu.org/licenses

### WIA Standards
- **WIA-INTENT**: Intent-based software configuration
- **WIA-OMNI-API**: Universal API gateway
- **WIA-BLOCKCHAIN**: Blockchain for license verification

---

**弘益人間 (Benefit All Humanity)**

*This specification is maintained by the WIA Computing Research Group and is continuously updated to reflect best practices in software licensing.*

*© 2025 SmileStory Inc. / WIA - MIT License*
