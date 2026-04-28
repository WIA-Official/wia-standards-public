# 📜 WIA-COMP-016: Software License Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-COMP-016
> **Version:** 1.0.0
> **Status:** Active
> **Category:** COMP / Computing & Software
> **Color:** Blue (#3B82F6)

---

## 🌟 Overview

The WIA-COMP-016 standard defines a comprehensive framework for software licensing, including license types, compliance verification, SPDX integration, commercial licensing models, and automated license compatibility checking.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to simplify software licensing, promote open-source adoption, ensure legal compliance, and foster transparent software development practices worldwide.

## 🎯 Key Features

- **SPDX Compliance**: Full support for SPDX license identifiers and expressions
- **License Compatibility**: Automated checking for license compatibility
- **Multi-License Support**: Manage projects with multiple licenses
- **Commercial Licensing**: Support for proprietary and dual-licensing models
- **Dependency Scanning**: Analyze dependency license compliance
- **License Templates**: Pre-built templates for common licenses
- **Compliance Reporting**: Generate compliance reports and BOMs
- **Copyright Management**: Track and manage copyright holders
- **License Exceptions**: Support for license exceptions and addendums
- **Audit Trail**: Complete history of license changes

## 📊 Core Concepts

### 1. License Categories

```
License Types:
- Permissive: MIT, Apache-2.0, BSD-3-Clause
- Weak Copyleft: LGPL, MPL
- Strong Copyleft: GPL-2.0, GPL-3.0, AGPL-3.0
- Public Domain: CC0, Unlicense
- Proprietary: Commercial, Closed-source
```

### 2. License Compatibility Matrix

```
Compatibility Rules:
- MIT → Apache-2.0 ✓
- MIT → GPL-3.0 ✓
- GPL-2.0 → GPL-3.0 ✗ (without explicit permission)
- LGPL → GPL ✓
- Apache-2.0 → GPL-2.0 ✗
```

### 3. SPDX Integration

| Component | Description |
|-----------|-------------|
| SPDX Identifier | Short license identifier (e.g., MIT, Apache-2.0) |
| SPDX Expression | Combined licenses (e.g., MIT OR Apache-2.0) |
| SPDX Document | Complete software bill of materials |
| License Text | Full license text with copyright |

## 🔧 Components

### TypeScript SDK

```typescript
import {
  validateLicense,
  checkLicenseCompatibility,
  generateSPDX,
  scanDependencies,
  createLicenseFromTemplate
} from '@wia/comp-016';

// Validate a license
const validation = validateLicense({
  identifier: 'MIT',
  copyrightHolder: 'SmileStory Inc.',
  copyrightYear: 2025
});

// Check compatibility
const compatibility = checkLicenseCompatibility({
  mainLicense: 'Apache-2.0',
  dependencies: [
    { name: 'lib-a', license: 'MIT' },
    { name: 'lib-b', license: 'GPL-3.0' }
  ]
});

console.log(compatibility.compatible, compatibility.conflicts);
```

### CLI Tool

```bash
# Validate a license
wia-comp-016 validate --license MIT --copyright "SmileStory Inc."

# Check compatibility
wia-comp-016 check-compat --main Apache-2.0 --deps MIT,BSD-3-Clause

# Generate SPDX document
wia-comp-016 generate-spdx --project MyProject --output SPDX.json

# Scan dependencies
wia-comp-016 scan-deps --package-json package.json
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-COMP-016-v1.0.md](./spec/WIA-COMP-016-v1.0.md) | Complete license specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-comp-016.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/software-license

# Run installation script
./install.sh

# Verify installation
wia-comp-016 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/comp-016

# Or yarn
yarn add @wia/comp-016
```

```typescript
import { LicenseSDK } from '@wia/comp-016';

const sdk = new LicenseSDK();

// Create license from template
const license = sdk.createLicenseFromTemplate({
  template: 'MIT',
  copyrightHolder: 'SmileStory Inc.',
  copyrightYear: 2025,
  projectName: 'MyProject'
});

console.log(license.text);

// Scan project dependencies
const scan = sdk.scanDependencies({
  packageManager: 'npm',
  projectPath: './my-project'
});

console.log(`Found ${scan.totalDependencies} dependencies`);
console.log(`Licenses: ${scan.uniqueLicenses.join(', ')}`);
```

## 🔬 Technical Specifications

### Supported Licenses

| Category | Licenses |
|----------|----------|
| Permissive | MIT, Apache-2.0, BSD-2-Clause, BSD-3-Clause, ISC |
| Weak Copyleft | LGPL-2.1, LGPL-3.0, MPL-2.0, EPL-2.0 |
| Strong Copyleft | GPL-2.0, GPL-3.0, AGPL-3.0 |
| Public Domain | CC0-1.0, Unlicense |
| Creative Commons | CC-BY-4.0, CC-BY-SA-4.0, CC-BY-NC-4.0 |

### SPDX Document Format

```json
{
  "spdxVersion": "SPDX-2.3",
  "dataLicense": "CC0-1.0",
  "SPDXID": "SPDXRef-DOCUMENT",
  "name": "MyProject",
  "documentNamespace": "https://example.com/spdx/myproject-1.0.0",
  "packages": [
    {
      "SPDXID": "SPDXRef-Package",
      "name": "MyProject",
      "licenseConcluded": "MIT",
      "copyrightText": "Copyright 2025 SmileStory Inc."
    }
  ]
}
```

### License Compatibility Rules

1. **Permissive → Any**: Permissive licenses can be combined with any license
2. **Weak Copyleft → Same/GPL**: LGPL can link with proprietary but must share modifications
3. **Strong Copyleft → Same Only**: GPL requires all combined work to be GPL
4. **Proprietary → Permissive**: Commercial software can use permissive licenses

## ⚠️ Compliance Considerations

1. **Attribution**: Always include license text and copyright notices
2. **Source Disclosure**: GPL requires source code distribution
3. **Patent Grants**: Apache-2.0 includes explicit patent grant
4. **Trademark**: Licenses don't grant trademark rights
5. **Liability**: Most open-source licenses disclaim liability

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based license selection
- **WIA-OMNI-API**: Universal license API
- **WIA-SOCIAL**: License sharing and discovery
- **WIA-BLOCKCHAIN**: License verification on blockchain
- **WIA-AI**: AI-powered license analysis

## 📖 Use Cases

1. **Open Source Projects**: Select and apply appropriate OSS licenses
2. **Commercial Software**: Dual-licensing and proprietary licensing
3. **Dependency Compliance**: Ensure all dependencies are compatible
4. **License Auditing**: Audit software for license compliance
5. **SBOM Generation**: Create software bills of materials
6. **Corporate Compliance**: Enterprise license management
7. **Academic Research**: Proper licensing for research code
8. **Government Software**: Public sector license requirements

## 🌍 Philosophy

**홍익인간 (弘益人間)** - Benefit All Humanity

This standard embodies the Korean philosophy of 弘益人間, aiming to benefit all humanity through innovation and standardization.

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **SPDX**: [spdx.org](https://spdx.org)

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
