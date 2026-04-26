# WIA-EDU-012: Micro-Credential Standard 📜

**Version:** 2.0.0
**Status:** ✅ Complete
**Category:** Education
**Philosophy:** 홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

The WIA Micro-Credential Standard (WIA-EDU-012) defines a comprehensive framework for creating, issuing, managing, and verifying digital badges, skill certifications, and stackable credentials in the modern learning ecosystem.

Micro-credentials represent a transformative shift in how we recognize and validate learning achievements. Unlike traditional degrees that take years to complete, micro-credentials allow learners to demonstrate specific skills and competencies as they acquire them, creating a more granular, flexible, and responsive approach to lifelong learning.

## Key Features

- **📜 Digital Badges:** Visual representations of achievements with embedded verifiable metadata
- **🎯 Skill Certifications:** Validate specific technical and professional competencies
- **📚 Stackable Credentials:** Combine credentials to build toward larger qualifications
- **🔍 Cryptographic Verification:** Tamper-proof credential validation
- **🌐 Open Standards:** Based on Open Badges 3.0 with WIA extensions
- **🔒 Privacy Controls:** Granular sharing permissions and selective disclosure
- **🎓 Competency Mapping:** Integration with global competency frameworks
- **📊 Lifelong Learning Records:** Comprehensive portfolio of all achievements

## Quick Start

### Installation

```bash
npm install @wia/micro-credential
```

### Basic Usage

```typescript
import { MicroCredentialClient } from '@wia/micro-credential';

// Initialize client
const client = new MicroCredentialClient({
  apiKey: 'your-wia-api-key',
  environment: 'production'
});

// Issue a credential
const credential = await client.issue({
  recipientId: 'did:example:learner123',
  recipientEmail: 'learner@example.com',
  achievementId: 'https://wia.org/achievements/js-expert',
  competencies: [{
    framework: 'https://ec.europa.eu/esco',
    competencyId: 'S1.2.3',
    competencyName: 'JavaScript programming',
    proficiencyLevel: 'advanced'
  }],
  evidence: [{
    type: 'Portfolio',
    url: 'https://github.com/learner/portfolio',
    description: 'React application with test coverage'
  }]
});

// Verify a credential
const verification = await client.verify(credential.id);
console.log('Valid:', verification.valid);

// Export as CLR
const clr = await client.exportCLR('did:example:learner123');
```

## Architecture

### 4-Phase Standard

1. **Phase 1 (v1.0): Data Format & Structure**
   - Open Badges 3.0 base format
   - WIA metadata extensions
   - Competency mapping
   - Privacy controls

2. **Phase 2 (v1.1): API Interface**
   - RESTful credential management
   - Verification endpoints
   - Search and discovery
   - Webhook events

3. **Phase 3 (v1.2): Protocol & Synchronization**
   - Real-time sync
   - Distributed verification
   - Cross-platform portability
   - Selective disclosure

4. **Phase 4 (v2.0): WIA Ecosystem Integration**
   - WIA-INTENT integration
   - WIA-OMNI-API gateway
   - AI-powered recommendations
   - Global registry

## Directory Structure

```
micro-credential/
├── index.html              # Landing page
├── simulator/              # Interactive simulator
│   └── index.html
├── ebook/                  # Comprehensive guides
│   ├── en/                 # English version
│   │   ├── index.html
│   │   └── chapter-*.html  # 8 chapters
│   └── ko/                 # Korean version
│       ├── index.html
│       └── chapter-*.html  # 8 chapters
├── spec/                   # Technical specifications
│   ├── v1.0.md            # Phase 1: Data Format
│   ├── v1.1.md            # Phase 2: API Interface
│   ├── v1.2.md            # Phase 3: Protocol
│   └── v2.0.md            # Phase 4: WIA Integration
├── api/                    # SDK implementations
│   └── typescript/
│       ├── package.json
│       └── src/
│           ├── types.ts
│           └── index.ts
└── README.md              # This file
```

## Core Concepts

### Digital Badges

Digital badges are visual representations of achievements that contain embedded metadata about:
- What was achieved (competencies, skills, knowledge)
- Who earned it (recipient identity, verification data)
- Who issued it (organization, authority, accreditation)
- When it was earned (issue date, expiration if applicable)
- How it was earned (criteria, evidence, assessment methods)
- Where to verify it (verification URL, blockchain hash, public key)

### Competency-Based Education

The standard shifts focus from time-served (credit hours) to competency-demonstrated. Learners progress by proving mastery, not by completing seat time.

**Five Proficiency Levels:**
1. **Novice:** Basic familiarity with guidance and support
2. **Developing:** Routine competency with occasional assistance
3. **Proficient:** Independent competency in standard contexts
4. **Advanced:** Sophisticated competency in complex scenarios
5. **Expert:** Innovative competency creating new knowledge

### Stackable Credentials

Credentials can combine and accumulate toward larger qualifications:

- **Vertical Stacking:** Progressive levels (Fundamentals → Advanced → Expert)
- **Horizontal Stacking:** Related domains (Frontend + Backend + DevOps)
- **Cross-Domain Stacking:** Interdisciplinary expertise

## Use Cases

### For Learners

- Earn granular recognition for specific skills
- Build flexible, personalized learning pathways
- Demonstrate competencies to employers immediately
- Carry credentials across platforms and institutions
- Stack micro-credentials toward degrees and certificates

### For Educators

- Recognize learning achievements at multiple levels
- Provide frequent positive reinforcement
- Experiment with innovative pedagogical approaches
- Align assessment with measurable outcomes
- Support competency-based progression

### For Employers

- Understand exactly what skills candidates possess
- Quickly validate credential authenticity
- Design targeted upskilling pathways
- Reduce hiring risk through transparency
- Support evidence-based talent development

## Certification

Organizations issuing WIA-compliant micro-credentials can pursue official certification:

### Bronze Certification
- Open Badges 3.0 compliance
- All required WIA fields
- Functional verification
- Basic privacy controls

### Silver Certification
- Competency framework integration
- Evidence attachments
- Stackability metadata
- Advanced privacy controls

### Gold Certification
- Blockchain anchoring
- DID-based identity
- CLR export/import
- Quality assurance audit

## Privacy & Security

- **Granular Sharing:** Choose exactly what to share with whom
- **Selective Disclosure:** Zero-knowledge proofs for attribute sharing
- **Time-Limited Access:** Automatic expiration of shared data
- **Cryptographic Signatures:** Ed25519 or RSA-PSS
- **Revocation Support:** Multiple revocation mechanisms
- **GDPR Compliance:** Full data portability and consent management

## Integration

### Compatible Standards

- Open Badges 3.0 (1EdTech)
- W3C Verifiable Credentials
- IMS CLR (Comprehensive Learner Record)
- ESCO (European Skills/Competences/Occupations)
- O*NET (US Occupational Information)
- xAPI (Experience API)
- LTI Advantage

### Platforms

- Credly
- Badgr
- Accredible
- Canvas LMS
- Moodle
- Blackboard

## Resources

- **Website:** https://wiastandards.com/micro-credential
- **Simulator:** [Try Interactive Demo](simulator/index.html)
- **Ebook:** [Read Complete Guide](ebook/en/index.html)
- **Specifications:** [View Technical Specs](spec/v2.0.md)
- **Community:** https://community.wia.org
- **Support:** support@wiastandards.com
- **GitHub:** https://github.com/WIA-Official/wia-standards

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](https://github.com/WIA-Official/wia-standards/blob/main/CONTRIBUTING.md).

## License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/blob/main/LICENSE) for details.

## Philosophy

홍익인간 (弘益人間) - "Benefit All Humanity"

This principle guides our commitment to making micro-credentials accessible, equitable, and empowering for learners worldwide. By standardizing credential formats and verification mechanisms, we remove barriers to recognition and create pathways for lifelong learning across borders and contexts.

---

© 2025 WIA - World Certification Industry Association
MIT License

**Built with ❤️ for learners everywhere**
