# WIA-MED-028: Health Insurance Data Standards 📄

> **홍익인간 (弘益人間) - Benefit All Humanity** - Standardizing health insurance data exchange worldwide

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://wiastandards.com)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Standard](https://img.shields.io/badge/standard-WIA--MED--028-purple.svg)](https://wiastandards.com/standards/WIA-MED-028)

## Overview

WIA-MED-028 defines international standards for health insurance data exchange, covering:

- **Claims Data Formats** - CMS-1500, UB-04, 837 electronic transactions
- **EDI X12 Standards** - HIPAA-compliant transactions (837, 835, 270/271, 278)
- **Premium Calculation** - Actuarial models and risk adjustment
- **Eligibility Verification** - Real-time benefit checks
- **Prior Authorization** - Automated approval workflows
- **Explanation of Benefits** - Clear, patient-friendly EOB
- **Fraud Detection** - AI-powered anomaly detection
- **Cross-Payer Exchange** - Interoperable data standards

## Quick Start

### View Documentation

- **Main Page**: [index.html](index.html)
- **Korean Ebook**: [ebook/ko/](ebook/ko/) - 8 chapters, 3,381 lines, fully detailed
- **English Ebook**: [ebook/en/](ebook/en/) - 8 chapters
- **Specifications**: [spec/](spec/)
  - [WIA-MED-028-Overview.md](spec/WIA-MED-028-Overview.md)
  - [WIA-MED-028-API.md](spec/WIA-MED-028-API.md)

### Online Access

- **Website**: https://wiastandards.com/standards/WIA-MED-028
- **Certification**: https://cert.wiastandards.com
- **Ebook Store**: https://wiabook.com

## Features

### ✅ Claims Processing
- CMS-1500 and UB-04 standard forms
- 837P/I/D electronic submission
- Automated claim validation
- Real-time claim status tracking

### ✅ Eligibility & Benefits
- 270/271 real-time verification (< 5 sec)
- Copay, deductible, coinsurance details
- Prior authorization requirements
- Out-of-pocket maximum tracking

### ✅ Payment & Remittance
- 835 ERA (Electronic Remittance Advice)
- Auto-posting to practice management systems
- CARC/RARC adjustment codes
- ACH payment integration

### ✅ Fraud Detection
- AI anomaly detection (85-90% accuracy)
- Pattern recognition for phantom billing, upcoding
- Network analysis for organized fraud
- Real-time risk scoring

### ✅ Global Interoperability
- International code mapping (ICD-10-CM ↔ ICD-10-WHO ↔ KCD-8)
- Multi-currency support with real-time exchange rates
- Cross-border claims processing
- HL7 FHIR compatibility

## Architecture

```
health-insurance-data/
├── index.html                 # Main landing page
├── ebook/
│   ├── ko/                    # Korean ebook (FULL content)
│   │   ├── index.html
│   │   └── chapter-01.html ~ chapter-08.html (200+ lines each)
│   └── en/                    # English ebook
│       ├── index.html
│       └── chapter-01.html ~ chapter-08.html
├── spec/
│   ├── WIA-MED-028-Overview.md   # Technical overview
│   └── WIA-MED-028-API.md        # API specification
└── README.md
```

## Topics Covered

### Chapter 1: Introduction
- Global health insurance landscape
- Need for standardization
- WIA-MED-028 vision

### Chapter 2: Claims Data Formats
- CMS-1500 field-by-field guide
- UB-04 institutional claims
- 837 EDI structure
- Common claim errors

### Chapter 3: EDI X12 Standards
- Transaction set overview (837, 835, 270/271, 278)
- Segment structure and loops
- Implementation guides
- Real-world examples

### Chapter 4: Premium Calculation
- Actuarial formulas
- Risk adjustment (HCC models)
- Group vs individual rating
- Underwriting rules

### Chapter 5: Eligibility & Prior Auth
- Real-time verification workflows
- 270/271 transaction details
- Prior authorization process
- Medical necessity criteria

### Chapter 6: Explanation of Benefits
- Patient-friendly EOB design
- 835 ERA automation
- Adjustment codes (CARC/RARC)
- Appeals process

### Chapter 7: Fraud Detection
- Common fraud patterns
- AI/ML detection techniques
- Prepayment vs postpayment review
- ROI metrics

### Chapter 8: Cross-Payer Exchange
- Coordination of Benefits (COB)
- Health Information Exchange (HIE)
- International claims workflow
- Global code mapping

## Implementation

### API Integration

```javascript
const WIAHealth = require('@wia-health/sdk');

const client = new WIAHealth({
  clientId: process.env.WIA_CLIENT_ID,
  clientSecret: process.env.WIA_CLIENT_SECRET,
  environment: 'production'
});

// Check eligibility
const eligibility = await client.eligibility.check({
  memberId: 'ABC123456',
  dateOfBirth: '1980-05-15',
  serviceDate: '2025-01-26'
});

console.log(eligibility.benefits.copay.office); // 25.00
```

### EDI X12 Example

```
ISA*00*          *00*          *ZZ*SENDER123      *ZZ*RECEIVER456    *250126*1430*^*00501*000000001*0*P*:~
GS*HC*SENDER123*RECEIVER456*20250126*1430*1*X*005010X222A1~
ST*837*0001*005010X222A1~
BHT*0019*00*123456*20250126*1430*CH~
NM1*85*2*CITY MEDICAL CLINIC*****XX*1234567890~
CLM*CLM-001*150***11:B:1*Y*A*Y*Y~
HI*ABK:E119*ABF:I10~
SV1*HC:99213*150*UN*1***1~
SE*25*0001~
GE*1*1~
IEA*1*000000001~
```

## Certification

Get your organization certified:

1. **Review Standards**: Read ebook and specs
2. **Implement Features**: Build according to WIA-MED-028
3. **Self-Assessment**: Use checklist
4. **Apply**: https://cert.wiastandards.com
5. **Audit**: Independent verification
6. **Certification**: Bronze/Silver/Gold/Platinum

## Contributing

We welcome contributions!

1. Fork the repository
2. Create feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open Pull Request

## Resources

- **Official Website**: https://wiastandards.com
- **Documentation**: https://developer.wiastandards.com
- **GitHub**: https://github.com/WIA-Official/wia-standards
- **Certification**: https://cert.wiastandards.com
- **Ebook Store**: https://wiabook.com
- **Support**: standards@wiastandards.com

## License

MIT License - see [LICENSE](LICENSE) file

## Related Standards

- **WIA-MED-001** - Electronic Health Records
- **WIA-MED-005** - Medical IoT
- **WIA-MED-017** - Medical Data Privacy
- **WIA-MED-020** - Mental Health Monitoring

## Citation

```bibtex
@standard{wia-med-028,
  title={WIA-MED-028: Health Insurance Data Standards},
  author={WIA Standards Committee},
  organization={World Certification Industry Association},
  year={2025},
  version={1.0.0},
  url={https://wiastandards.com/standards/WIA-MED-028}
}
```

---

**홍익인간 (弘益人間) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA (World Certification Industry Association)

**Contact**: standards@wiastandards.com | **Website**: https://wiastandards.com
