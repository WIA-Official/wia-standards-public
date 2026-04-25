# WIA-CRYO-010 🔬 Cryopreservation Research Data Standard

**Version:** 2.0
**Status:** Official Release
**Philosophy:** 홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA-CRYO-010 is a comprehensive standard for documenting, sharing, and analyzing cryopreservation research data. This standard enables reproducible science, facilitates global collaboration, and supports regulatory compliance across basic research and clinical applications.

### Key Features

- **Standardized Data Formats** for temperature profiles, cryoprotectant composition, and viability metrics
- **Clinical Trial Extensions** with HIPAA/GDPR-compliant patient privacy protections
- **Multi-Center Collaboration** support with data harmonization and quality assurance
- **Regulatory Ready** formats for FDA, EMA, and other regulatory submissions
- **TypeScript SDK** for easy integration into existing systems
- **Interactive Simulator** for hands-on learning and testing
- **Comprehensive eBook** available in English and Korean

---

## Quick Start

### 1. View the Landing Page

Open `index.html` in your browser to explore the standard overview, 4-phase cards, and navigation.

### 2. Try the Simulator

Navigate to `simulator/index.html` to:
- Format experimental data
- Calculate viability metrics
- Build research protocols
- Test API integration
- Run compliance tests

### 3. Read the Documentation

**English eBook:** `ebook/en/index.html`
**Korean eBook:** `ebook/ko/index.html`

Covers:
- Introduction to cryopreservation research
- Experimental data formats
- Clinical trial data management
- Tissue viability metrics
- Revival success protocols
- Research collaboration standards
- Implementation guide
- Future directions & ethics

### 4. Review the Specification

See `spec/WIA-CRYO-010-v2.0.md` for the complete technical specification.

### 5. Use the TypeScript SDK

```bash
npm install @wia/cryo-010
```

```typescript
import { CryoResearchClient, ExperimentData } from '@wia/cryo-010';

const client = new CryoResearchClient({
  apiKey: process.env.WIA_API_KEY,
  endpoint: 'https://api.wia.org/cryo-010'
});

const experiment: ExperimentData = {
  standard: 'WIA-CRYO-010',
  version: '2.0',
  experiment: {
    id: 'CRYO-2025-001',
    type: 'cell',
    title: 'Mesenchymal Stem Cell Cryopreservation',
    date: new Date().toISOString(),
    researcher: {
      name: 'Dr. Jane Smith',
      orcid: '0000-0001-2345-6789',
      institution: 'Research Institute',
      email: 'jane@institute.org'
    }
  },
  sample: {
    type: 'human_mesenchymal_stem_cells',
    source: 'bone_marrow',
    quantity: 5000000,
    unit: 'cells'
  },
  protocol: {
    method: 'slow_freeze',
    temperature_profile: {
      unit: 'celsius',
      sampling_rate: 1,
      data_points: [
        { time: 0, temperature: 37, sample_temperature: 37 },
        { time: 60, temperature: -80, sample_temperature: -79 },
        { time: 120, temperature: -196, sample_temperature: -196 }
      ],
      statistics: {
        mean_cooling_rate: -1.02,
        temperature_uniformity: 0.95
      }
    },
    cryoprotectant: {
      components: [
        { name: 'DMSO', concentration: 10, unit: 'percent_v/v', grade: 'cell_culture_grade' },
        { name: 'FBS', concentration: 20, unit: 'percent_v/v', grade: 'research_grade' },
        { name: 'Culture_Medium', concentration: 70, unit: 'percent_v/v' }
      ],
      preparation_date: '2025-01-14'
    },
    equipment_settings: {
      model: 'CryoFreeze-3000',
      manufacturer: 'CryoTech Inc.',
      calibration_date: '2025-01-01'
    }
  },
  measurements: {
    pre_freeze_viability: 98.5,
    post_thaw_viability: 85.2
  },
  quality_control: {
    equipment_calibration: '2025-01-01'
  }
};

// Submit experiment
const result = await client.submitExperiment(experiment);
console.log('Experiment ID:', result.experiment_id);
console.log('Validation:', result.validation);
```

---

## Directory Structure

```
cryo-research/
├── index.html                 # Landing page
├── simulator/
│   └── index.html            # Interactive simulator (5 tabs)
├── ebook/
│   ├── en/                   # English eBook (9 files)
│   │   ├── index.html        # Table of contents
│   │   ├── chapter1.html     # Introduction
│   │   ├── chapter2.html     # Experimental data formats
│   │   ├── chapter3.html     # Clinical trial data
│   │   ├── chapter4.html     # Tissue viability metrics
│   │   ├── chapter5.html     # Revival success protocols
│   │   ├── chapter6.html     # Research collaboration
│   │   ├── chapter7.html     # Implementation guide
│   │   └── chapter8.html     # Future directions & ethics
│   └── ko/                   # Korean eBook (9 files)
│       └── ...               # Same structure as English
├── spec/                     # Specifications
│   ├── WIA-CRYO-010-v1.0.md # Initial release
│   ├── WIA-CRYO-010-v1.1.md # Enhanced statistics
│   ├── WIA-CRYO-010-v1.2.md # Clinical extensions
│   └── WIA-CRYO-010-v2.0.md # Current version
├── api/
│   └── typescript/           # TypeScript SDK
│       ├── package.json
│       └── src/
│           ├── types.ts      # Type definitions
│           └── index.ts      # SDK implementation
└── README.md                 # This file
```

---

## Core Concepts

### 1. Experiment ID Format

All experiments use the format: `CRYO-YYYY-NNN`

Examples:
- `CRYO-2025-001`
- `CRYO-2025-142`

### 2. Temperature Profile

High-resolution temperature tracking:
- Minimum 3 data points (recommended ≥ 60 for detailed profiles)
- Sample temperature vs. chamber temperature
- Statistical analysis (cooling rate, uniformity)

### 3. Cryoprotectant Composition

All components must sum to 100%:
- Individual concentrations with units
- Manufacturer and lot numbers (for reproducibility)
- Preparation and expiration dates

### 4. Viability Assessment

Multi-level success criteria:
- **Level 1:** Membrane integrity (trypan blue)
- **Level 2:** Metabolic activity (MTT/ATP)
- **Level 3:** Proliferative capacity
- **Level 4:** Functional recovery
- **Level 5:** Clinical efficacy

### 5. Quality Control

- Equipment calibration tracking
- Protocol deviation documentation
- Contamination screening
- Statistical validation

---

## Clinical Applications

For clinical use cases, additional requirements apply:

### Patient Privacy

- **NO** directly identifying information (PHI/PII)
- Use study-specific patient IDs
- Age reported in 5-year ranges
- Geographic data limited to region level

### Informed Consent

- Version tracking
- Consent date and language
- Special permissions (research use, data sharing)
- Withdrawal status

### Regulatory Compliance

- FDA 21 CFR Part 1271 (US)
- EU Directive 2004/23/EC (Europe)
- HFEA Code of Practice (UK)
- Local regulatory requirements

---

## Data Exchange

### Supported Formats

- **JSON** (primary, UTF-8)
- **XML** (regulatory submissions)
- **CSV** (statistical analysis)
- **HDF5** (large time-series data)

### API Endpoints

```
POST   /experiments          # Submit new experiment
GET    /experiments/{id}     # Retrieve specific experiment
GET    /experiments/search   # Search with filters
PUT    /experiments/{id}     # Update experiment
GET    /health              # API health status
```

### Authentication

OAuth 2.0 bearer tokens:

```typescript
const client = new CryoResearchClient({
  apiKey: 'your-api-key-here'
});
```

---

## Validation

All data must pass validation:

1. **Schema Validation:** Conform to JSON schema
2. **Range Checks:** Values within defined ranges
3. **Logic Checks:** Post-thaw ≤ pre-freeze viability
4. **Completeness:** All required fields present
5. **Format Checks:** Dates, ORCIDs, emails valid

### Using the Validator

```typescript
const validation = await client.validateExperiment(experiment);

if (!validation.valid) {
  console.error('Validation errors:', validation.errors);
}
```

---

## Quality Metrics

Target quality levels:

- **Completeness:** ≥ 98% of fields populated
- **Accuracy:** ≥ 99% passing validation
- **Timeliness:** Data entry within 24 hours

---

## Version History

| Version | Date | Key Changes |
|---------|------|-------------|
| **v2.0** | 2025-01-01 | AI/ML support, enhanced clinical requirements |
| v1.2 | 2024-06-15 | Clinical trial extensions, privacy protections |
| v1.1 | 2023-12-01 | Statistical requirements, equipment tracking |
| v1.0 | 2023-06-01 | Initial release |

---

## Resources

### Documentation

- **Landing Page:** `index.html`
- **Simulator:** `simulator/index.html`
- **English eBook:** `ebook/en/index.html`
- **Korean eBook:** `ebook/ko/index.html`
- **Specification:** `spec/WIA-CRYO-010-v2.0.md`

### SDK & Tools

- **TypeScript SDK:** `api/typescript/`
- **npm Package:** `@wia/cryo-010`

### Online

- **Website:** https://wia.org/standards/cryo-010
- **GitHub:** https://github.com/WIA-Official/wia-standards
- **Community Forum:** https://forum.wia.org/cryo-010
- **Support Email:** cryo-010@wia.org

---

## Contributing

We welcome contributions from the community!

### How to Contribute

1. **Report Issues:** Use GitHub Issues for bugs and feature requests
2. **Submit Pull Requests:** Follow our contribution guidelines
3. **Join Discussions:** Participate in the community forum
4. **Provide Feedback:** Share your implementation experiences

### Governance

WIA-CRYO-010 is maintained by the WIA Standards Committee with input from:
- Research scientists
- Clinical practitioners
- Regulatory experts
- Software developers
- Data scientists

---

## License

This standard and its documentation are licensed under **CC-BY-4.0** (Creative Commons Attribution 4.0 International).

You are free to:
- **Share** — copy and redistribute the material
- **Adapt** — remix, transform, and build upon the material

Under the following terms:
- **Attribution** — Give appropriate credit to WIA

---

## Citation

If you use WIA-CRYO-010 in your research, please cite:

```
WIA Standards Committee. (2025). WIA-CRYO-010: Cryopreservation Research Data Standard
(Version 2.0). World Certification Industry Association. https://wia.org/standards/cryo-010
```

BibTeX:
```bibtex
@techreport{wia2025cryo010,
  title={WIA-CRYO-010: Cryopreservation Research Data Standard},
  author={{WIA Standards Committee}},
  year={2025},
  institution={World Certification Industry Association},
  type={Standard},
  number={v2.0},
  url={https://wia.org/standards/cryo-010}
}
```

---

## Support

### Documentation

- Read the comprehensive eBook (English/Korean)
- Review the technical specification
- Try the interactive simulator

### Community

- **Forum:** https://forum.wia.org/cryo-010
- **Slack:** #wia-cryo-010
- **Email:** cryo-010@wia.org

### Training

- Monthly webinars (free registration)
- On-site workshops available
- Certification programs

---

## Acknowledgments

WIA-CRYO-010 was developed with input from researchers, clinicians, and institutions worldwide. We thank all contributors for their dedication to advancing cryopreservation science through standardization.

Special thanks to:
- The global cryobiology research community
- Clinical cryopreservation practitioners
- Regulatory agencies providing guidance
- Open science advocates

---

## Philosophy

**홍익인간 (弘益人間) - Benefit All Humanity**

By standardizing cryopreservation research data, we create a global commons of scientific knowledge that benefits all humanity. Every researcher who adopts WIA-CRYO-010 contributes to and benefits from this shared resource.

---

© 2025 SmileStory Inc. / WIA
홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity

This is a production-ready implementation of the WIA-CRYO-010 standard.
