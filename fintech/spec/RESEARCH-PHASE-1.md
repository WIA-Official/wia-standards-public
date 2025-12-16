# WIA Fintech Accessibility: Phase 1 Research

## Version Information
- **Document Version**: 1.0.0
- **Last Updated**: 2025-01-15
- **Status**: Phase 1 - Research Complete
- **Standard**: WIA-FIN-RES-001

---

## 1. Executive Summary

This research document analyzes the current state of financial technology accessibility, examining regulatory frameworks, industry standards, technological solutions, and accessibility gaps in banking and fintech services. The findings inform the WIA Fintech Accessibility Standard development.

### Key Findings

| Area | Current State | Gap |
|------|---------------|-----|
| Regulatory | Fragmented (ADA, EAA, EN 301 549) | No unified accessibility standard |
| ATM | 100,000+ talking ATMs in US | Inconsistent features across vendors |
| Mobile Banking | Major banks mostly accessible | Small fintechs lag behind |
| Payment Cards | Notched/Braille cards emerging | Not standardized |
| Screen Reader | Variable compatibility | Many apps fail basic tests |

---

## 2. Regulatory Landscape

### 2.1 United States

#### Americans with Disabilities Act (ADA)
- **Title III**: Banks as "places of public accommodation" must provide accessible services
- **Section 504**: Federal fund recipients must provide accessible programs
- **Section 508**: Government-contracted services must meet federal ICT standards
- **DOJ Guidance**: WCAG 2.1 Level AA compliance required by April 2026

#### Legal Landscape
- **3,500+ ADA lawsuits** in 2022 related to digital accessibility (14% increase YoY)
- **57%** of financial services professionals reported legal action related to accessibility
- Major settlements with banks over inaccessible websites and mobile apps

### 2.2 European Union

#### European Accessibility Act (EAA)
- **Implementation Deadline**: June 2025
- **Scope**: All financial products and services
- **Requirements**:
  - ATMs must meet specific technical standards
  - Websites and mobile apps must be accessible
  - Electronic documents and statements must be accessible
- **Penalties**: Up to 10% of organization's revenue for non-compliance

#### EN 301 549
- European standard for ICT accessibility
- Harmonized with WCAG 2.1
- Specific requirements for ATMs, kiosks, and digital services

### 2.3 Other Jurisdictions

| Region | Regulation | Status |
|--------|------------|--------|
| UK | Equality Act 2010, BS 8878 | Active enforcement |
| Canada | AODA, ACA | Provincial + Federal |
| Australia | DDA, WCAG adoption | Active |
| Korea | Disability Discrimination Act | Active |
| Japan | JIS X 8341 | Guidelines |

---

## 3. Market Statistics

### 3.1 Disability Demographics

```
Global Statistics:
├── 1.3 billion people with disabilities worldwide (16% of population)
├── 101 million in EU aged 16+ with some disability (27%)
├── 61 million in US with disabilities (26%)
└── Over 50% are aged 65+ with additional challenges
```

### 3.2 Financial Exclusion

| Metric | Value |
|--------|-------|
| Unbanked rate (disability households) | 3x higher than non-disabled |
| Online banking difficulty reported | 24% of disabled users |
| Would switch for better accessibility | 65% of users |
| Satisfied with current accessibility | Only 48% |

### 3.3 Economic Impact

- **$200+ billion** discretionary spending by people with disabilities in US
- **$13 trillion** global spending power of disabled consumers
- **$8 trillion** additional family/friend influence spending

---

## 4. Current Technology Landscape

### 4.1 ATM Accessibility

#### Talking ATMs
- **100,000+** accessible ATMs in US with audio guidance
- Standard **3.5mm headphone jack** for privacy
- Audio instructions for all transaction types

#### Tactile Features
| Feature | Description |
|---------|-------------|
| Keypad Braille | Raised dot on number 5 (center) |
| Confirm Button | Raised circle |
| Cancel Button | Raised cross/X |
| Screen Position | Consistent height standards |

#### Emerging Innovations
- **Smartphone Integration**: NFC/Bluetooth connection to ATM
- **QR Code Access**: Pre-configured transactions via mobile
- **Personal Device Control**: Use phone as ATM interface
- **Voice Banking**: Complete transactions via voice commands

### 4.2 Payment Cards

#### Accessible Card Features
| Bank | Features |
|------|----------|
| ING Belgium | First notched payment card (2021) |
| Barclays UK | Braille markings, large text |
| RBS/NatWest | High contrast, tactile elements |
| Wells Fargo | Multiple accessible card options |

#### Card Identification Standards
- **Notch System**: Edge cutout for orientation
- **Braille Embossing**: Account type indicator
- **Color Contrast**: High visibility design
- **Large Print**: Larger card numbers

### 4.3 Mobile Banking

#### Screen Reader Compatibility
- **Major Banks**: Generally good compatibility (Bank of America, Wells Fargo, Chase)
- **Fintech Apps**: Variable, often poor accessibility
- **Common Issues**:
  - Unlabeled buttons/images
  - Inaccessible forms
  - Poor error handling
  - Time-limited operations

#### Accessibility Features
| Feature | Description |
|---------|-------------|
| VoiceOver/TalkBack | Screen reader support |
| Dynamic Type | Adjustable text size |
| High Contrast | Dark mode, contrast options |
| Voice Commands | Siri/Google Assistant integration |
| Biometric Auth | Face ID, Touch ID, Voice recognition |

### 4.4 Online Banking

#### WCAG Compliance Status
- **None** of 30 major banks studied fully met WCAG 2.1 AA
- Common failures:
  - Form labels
  - Dashboard screen reader compatibility
  - Video captions
  - Keyboard navigation

#### Security vs Accessibility Conflicts
| Security Feature | Accessibility Issue |
|------------------|---------------------|
| CAPTCHA | Screen reader incompatible |
| Copy-paste disabled | Password manager blocked |
| 20-second OTP timeout | Insufficient time for disabled users |
| Image-based verification | Vision impaired exclusion |

---

## 5. Industry Leaders & Best Practices

### 5.1 Wells Fargo
- Screen reader compatible platforms
- Video banking with ASL interpreters
- Talking ATMs nationwide
- Accessible mobile app

### 5.2 Bank of America
- Comprehensive accessibility services
- Dedicated accessibility support line
- Screen magnification support
- TTY/TDD services

### 5.3 Barclays (UK)
- SignVideo service for Deaf customers
- High-visibility debit cards
- Accessibility-certified branches
- Accessible app with VoiceOver

### 5.4 Innovative Fintech Solutions

| Company | Innovation |
|---------|------------|
| HPS Worldwide | OTP solution for blind users with rule-based payments |
| Monzo | High-contrast card design, excellent app accessibility |
| Starling Bank | Best-in-class screen reader support |
| Revolut | Voice-enabled features |

---

## 6. Technology Standards & Protocols

### 6.1 WCAG 2.1 Level AA (Minimum for Banking)

#### Four Principles (POUR)
1. **Perceivable**: Information presentable in multiple ways
2. **Operable**: Interface operable via various methods
3. **Understandable**: Clear, predictable operation
4. **Robust**: Compatible with assistive technologies

#### Key Success Criteria for Banking
| Criterion | Description | Banking Application |
|-----------|-------------|---------------------|
| 1.1.1 | Non-text content alternatives | Account icons, graphs |
| 1.4.3 | Contrast ratio 4.5:1 | Account balances |
| 2.1.1 | Keyboard accessible | All transactions |
| 2.2.1 | Timing adjustable | OTP, session timeout |
| 3.3.1 | Error identification | Form validation |
| 4.1.2 | Name, Role, Value | Interactive elements |

### 6.2 ISO/IEC Standards

| Standard | Scope |
|----------|-------|
| ISO 9241-171 | Software accessibility guidance |
| ISO/IEC 40500 | WCAG 2.0 as ISO standard |
| ISO 21542 | Accessible built environment (ATMs, branches) |
| ISO 9999 | Assistive products classification |

### 6.3 Payment Industry Standards

| Standard | Organization | Scope |
|----------|--------------|-------|
| PCI DSS | PCI Council | Security (accessibility conflicts) |
| EMV | EMVCo | Chip card standards |
| ISO 8583 | ISO | Financial messaging |
| Open Banking | Various | API standards |

---

## 7. Identified Accessibility Gaps

### 7.1 Critical Gaps

```
1. No unified financial accessibility standard
   └── Fragmented regulations across jurisdictions

2. Security features create barriers
   └── CAPTCHA, timeouts, copy-paste restrictions

3. Small fintech companies lack resources
   └── Focus on features over accessibility

4. Inconsistent ATM accessibility
   └── No global standard for accessible ATMs

5. Payment card accessibility not standardized
   └── Voluntary implementations only
```

### 7.2 Technology Gaps

| Gap | Impact | WIA Solution Opportunity |
|-----|--------|--------------------------|
| No standard accessible profile format | Users repeat preferences | Unified profile schema |
| Inconsistent screen reader support | Variable experience | Standard API requirements |
| No WIA device integration | Isolated accessibility | Exoskeleton/Bionic Eye integration |
| Limited multi-modal feedback | Single-sense reliance | Haptic, audio, visual standards |

### 7.3 User Experience Gaps

- Account setup not accessible
- Transaction confirmation unclear
- Error messages not descriptive
- Help systems not accessible
- No consistent accessibility settings

---

## 8. WIA Integration Opportunities

### 8.1 Exoskeleton Integration
- Haptic feedback for transaction confirmations
- ATM location guidance
- Gesture-based banking commands

### 8.2 Bionic Eye Integration
- Real-time card/ATM recognition
- Transaction amount visualization
- Currency identification
- Receipt reading

### 8.3 Voice-Sign Integration
- Real-time customer service translation
- Document explanation in sign language
- Banking terminology glossary
- Emergency financial phrase library

### 8.4 Smart Wheelchair Integration
- Accessible ATM height adjustment
- Branch navigation assistance
- Optimal positioning for transactions

---

## 9. Recommendations

### 9.1 Data Format Standards
1. Define accessible financial profile schema
2. Standardize transaction accessibility metadata
3. Create accessible notification format
4. Define ATM/device capability descriptors

### 9.2 API Standards
1. Accessibility feature discovery API
2. User preference sync API
3. Multi-modal output API
4. WIA device integration API

### 9.3 Communication Protocols
1. Accessible real-time notifications
2. Secure accessible authentication
3. Multi-modal transaction confirmation
4. Emergency accessibility protocols

### 9.4 Ecosystem Integration
1. Cross-platform profile sync
2. WIA device mesh integration
3. Healthcare data linking (for medical payments)
4. Government benefits integration

---

## 10. Sources

### Regulatory & Legal
- [ADA Compliance for Banks: Essential 2025 Guide](https://accessibe.com/blog/knowledgebase/ada-compliance-for-banks)
- [Americans with Disabilities Act | ABA](https://www.aba.com/banking-topics/compliance/acts/americans-with-disabilities-act)
- [ADA Banking Requirements | Level Access](https://www.levelaccess.com/blog/ada-financial-institutions/)

### Industry Analysis
- [Digital Accessibility in Banking and Fintech](https://www.aubergine.co/insights/importance-of-digital-accessibility-in-banking-and-fintech)
- [Accessibility in Digital Banking | TestDevLab](https://www.testdevlab.com/blog/accessibility-in-digital-banking)
- [Financial Services Accessibility | TestParty](https://testparty.ai/blog/financial-services-accessibility)
- [Accessibility Testing in Fintech | HeadSpin](https://www.headspin.io/blog/accessibility-testing-in-fintech)

### User Experience
- [Accessible Banking for Visual Impairments | AFB](https://afb.org/blindness-and-low-vision/using-technology/online-shopping-and-banking-accessibility-people-visual-3)
- [Banks for Visually Impaired | Disabilitease](https://disabilitease.com/banks-banking-apps-visually-impaired/)
- [Accessible Banking UK | Choose](https://www.choose.co.uk/guide/accessible-banking-for-blind-partially-sighted.html)

### Technology & Innovation
- [Making Payments Accessible | HPS Worldwide](https://www.hps-worldwide.com/blog/making-payments-accessible-everyone)
- [How to Design Accessible Fintech | Softjourn](https://softjourn.com/insights/accessible-fintech)
- [Barrier-free Banking | Xperienz](https://medium.com/@xperienzRD/barrier-free-banking-from-branches-to-mobile-apps-accessible-for-all-c9b5b56a24b8)

---

## Document Information

- **Document ID**: WIA-FIN-RES-001
- **Classification**: Public Research
- **Maintainer**: WIA Standards Committee
- **License**: CC BY 4.0

弘益人間 (홍익인간) - Accessible Finance for All Humanity
