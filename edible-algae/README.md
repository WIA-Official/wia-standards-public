# WIA-AGRI-033: Edible Algae Standard

## Standard Information

- **ID**: WIA-AGRI-033
- **Folder**: edible-algae
- **Emoji**: 🟢
- **EN Title**: Edible Algae
- **KO Title**: 식용 조류
- **Primary Color**: #84CC16 (Lime for AGRI)
- **Version**: 1.0.0
- **Status**: Complete

## Description

Global standard for edible algae cultivation, including spirulina, chlorella, seaweed farming, and microalgae nutrition for sustainable protein production.

## Directory Structure

```
edible-algae/
├── index.html                          # Landing page (EN/KO toggle, dark theme)
├── simulator/
│   └── index.html                      # 5-tab interactive simulator
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md         # Data format specification
│   ├── PHASE-2-API-INTERFACE.md       # API interface specification
│   ├── PHASE-3-PROTOCOL.md            # Communication protocol
│   └── PHASE-4-INTEGRATION.md         # Integration specification
└── ebook/
    ├── en/
    │   ├── index.html                  # English ebook index
    │   └── chapter-01.html to chapter-08.html (21KB - 35KB each)
    └── ko/
        ├── index.html                  # Korean ebook index
        └── chapter-01.html to chapter-08.html (35KB - 36KB each)
```

## Features

### Landing Page (index.html)
- Dark theme (#0f172a background)
- EN/KO language toggle with localStorage
- Responsive design
- 홍익인간 (弘益人間) footer
- Links to simulator, spec, and ebook
- target="_blank" for all external links

### Interactive Simulator (simulator/index.html)
- 5 comprehensive tabs:
  1. **Data Format** - Algae cultivation data generator
  2. **Algorithms** - Growth rate & nutrition calculator
  3. **Protocol** - Photobioreactor communication simulator
  4. **Integration** - Aquaculture systems integration tester
  5. **QR & VC** - Product certification QR code & verifiable credential generator
- Real-time data generation
- EN/KO language support
- Dark theme with #84CC16 primary color

### Specification Documents (spec/)
- **PHASE-1**: Comprehensive data format specification (54KB)
- **PHASE-2**: RESTful API and monitoring interfaces (12KB)
- **PHASE-3**: MQTT, sensor networks, control protocols (13KB)
- **PHASE-4**: SCADA, ERP, blockchain integration (17KB)

### Ebook (ebook/)
- **English**: Index + 8 chapters (21KB - 35KB each)
- **Korean**: Index + 8 chapters (35KB - 36KB each)
- All chapters exceed 15KB requirement
- Comprehensive coverage of:
  - Algae biology and species
  - Global food security
  - Standard implementation
  - Technical specifications
  - Integration guides
  - Case studies and best practices

## Key Topics Covered

1. **Algae Species**: Spirulina, Chlorella, Nannochloropsis, Kelp, Wakame, Nori
2. **Cultivation Systems**: Photobioreactors, Open Ponds, Sea Farms, Vertical Farms
3. **Nutrition**: Protein (60-70%), Omega-3, Vitamins, Minerals, Bioactive compounds
4. **Sustainability**: CO2 sequestration, Water efficiency, Land use optimization
5. **Quality Control**: Contamination screening, Heavy metals testing, Certifications
6. **Automation**: Real-time monitoring, Control loops, SCADA integration
7. **Traceability**: Blockchain, Supply chain, QR codes, Verifiable credentials

## Philosophy

**홍익인간 (弘益人間)** - Benefit All Humanity

Sustainable protein from the ocean for global food security.

---

© 2025 WIA Standards - MIT License
