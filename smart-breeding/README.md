# WIA Smart Breeding Standard

> Genomic selection, phenotyping, and breeding value estimation for livestock and crops

## Overview

The WIA Smart Breeding Data Format Standard defines a unified framework for genomic breeding programs, enabling genomic selection, phenotyping, breeding value estimation, and genetic diversity management across animal and plant breeding systems.

## Key Features

- **Genomic Selection**: Support for SNP arrays, whole genome sequencing, and genomic relationship matrices
- **Phenotype Management**: Standardized trait measurements and performance records
- **Breeding Values**: EBV (Estimated Breeding Value) and GEBV (Genomic EBV) calculation
- **Genetic Diversity**: Population metrics, inbreeding monitoring, and diversity conservation
- **Interoperability**: Compatible with VCF, PLINK, HapMap, and GFF3 formats

## Quick Start

### TypeScript SDK

```bash
npm install @wia/smart-breeding
```

```typescript
import { SmartBreedingClient } from '@wia/smart-breeding';

const client = new SmartBreedingClient({
  baseUrl: 'https://api.example.com/breeding',
  apiKey: 'your-api-key'
});

// Get individual breeding data
const individual = await client.getIndividual('BULL-2025-001');
console.log(individual.data);
```

## Documentation

- [Phase 1: Data Format](spec/PHASE-1-DATA-FORMAT.md)
- [Phase 2: API Interface](spec/PHASE-2-API-INTERFACE.md)
- [Phase 3: Protocol](spec/PHASE-3-PROTOCOL.md)
- [Phase 4: Integration](spec/PHASE-4-INTEGRATION.md)

## Use Cases

- **Dairy Cattle**: Genomic selection for milk production, fat percentage, protein content
- **Beef Cattle**: Breeding values for growth rate, carcass quality, marbling
- **Pigs**: Litter size, growth rate, feed conversion efficiency
- **Poultry**: Egg production, meat quality, disease resistance
- **Crops**: Yield potential, disease resistance, climate adaptation

## CLI

A reference CLI ships under `cli/smart-breeding.sh` with subcommands:

| Subcommand | Purpose |
|------------|---------|
| `validate` | Validate an individual breeding record JSON per Phase 1 |
| `gebv`     | Print a GEBV envelope skeleton for a given trait |
| `pedigree` | Print a pedigree envelope skeleton |
| `info`     | Show standard summary |

## Companion standards

* **VCF v4.3** — variant call format for genotype payloads
* **PLINK** — BED/BIM/FAM bundles for pedigree-genotype linkage
* **HapMap** — reference haplotype panel format
* **GFF3** — genomic feature annotation
* **WIA-OMNI-API** — credential storage for breeder identities
* **WIA-AIR-SHIELD** — transport hardening for cross-organisation data exchange

## License

MIT License - © 2025 WIA Standards Committee

**홍익인간 (弘益人間) - Benefit All Humanity**
