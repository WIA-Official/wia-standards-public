# WIA-DIGITAL-MEMORIAL

Digital Memorial Standard - Creating Lasting Digital Remembrance

## Overview

WIA-DIGITAL-MEMORIAL defines protocols for creating, managing, and preserving digital memorials for deceased individuals. Unlike DIGITAL-ERASURE which removes digital presence, this standard helps create meaningful, lasting tributes that honor the memory of loved ones.

## Philosophy

**Hongik Ingan (홍익인간)**: The living honor the dead through remembrance. Digital memorials extend this tradition into the digital age, allowing memories to be preserved and shared across generations.

## Key Features

- Memorial website creation
- Interactive timeline of life
- Photo and video galleries
- Message and tribute collection
- Virtual visiting and condolence
- Legacy content preservation
- Anniversary and birthday reminders
- Cross-platform memorial syndication
- Privacy controls for family
- Long-term preservation guarantees

## Structure

```
digital-memorial/
├── README.md
├── spec/
│   └── DIGITAL-MEMORIAL-v1.0.md
├── simulator/
│   └── index.html
└── ebook/
    ├── en/
    │   ├── index.html
    │   └── chapter-01.html ... chapter-08.html
    └── ko/
        ├── index.html
        └── chapter-01.html ... chapter-08.html
```

## Related Standards

- WIA-DIGITAL-FUNERAL: Digital death planning
- WIA-DIGITAL-WILL: Digital testament
- WIA-DIGITAL-ERASURE: For those who choose deletion

## Memorial Types

- Public memorials - Open to everyone
- Family memorials - Family and friends only
- Private memorials - Invitation only
- Hybrid memorials - Public profile, private content

## Version

Current: 1.0.0

## License

MIT License - Anthropic

---

**홍익인간 (弘益人間)** - Benefit All Humanity 🌍

## Conformance Tiers

| Tier      | Scope                                                      | Audit cadence |
|-----------|------------------------------------------------------------|---------------|
| Surface   | data formats accepted; no formal audit                     | self-attested |
| Verified  | annual third-party audit against PHASE documents           | annual        |
| Anchored  | continuous evidence package per Annex G; SBOM signed       | continuous    |

Implementations declare their tier in the OpenAPI document via the
`x-wia-conformance-tier` extension field.

## Reference Implementations

- TypeScript SDK: `api/typescript/`
- Schemas: JSON Schema 2020-12 in `spec/PHASE-1-DATA-FORMAT.md`
- Conformance vectors: `tests/phase-vectors/`

## Open Governance

Issues and proposals are tracked at github.com/WIA-Official/wia-standards
issues with the `digital-memorial` label. The WIA Standards working group reviews open
issues at the start of every minor release cycle and publishes the
resulting decision log alongside the release notes.

弘益人間 (Hongik Ingan) — Benefit All Humanity
