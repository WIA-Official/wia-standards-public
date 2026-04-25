# WIA-AGRI-029: Desert Agriculture Standard

> **Sustainable food production in arid and hyper-arid zones.**
> Aligned with FAO-56 (Penman-Monteith), UNEP aridity index, ISO 11277:2020 (soil texture), ISO 16586:2003 (soil water content).

## 1. Overview

The WIA-AGRI-029 Desert Agriculture Standard provides interoperable schemas, reference algorithms, and reporting metrics for cultivated systems operating under the FAO/UNEP definitions of *arid* (P/PET 0.05–0.20), *hyper-arid* (P/PET < 0.05), and *semi-arid* (P/PET 0.20–0.50) climate zones.

The standard targets three operational concerns of arid-zone cropping:

1. **Water budget transparency** — every farm reports irrigation, ET₀, and water-use efficiency (m³ per ton of produce) in a common schema.
2. **Soil salinity governance** — saturation paste extract electrical conductivity (ECe, dS/m) is recorded against the FAO soil-salinity classes (FAO Soils Bulletin 39).
3. **Drought-adaptation traceability** — cultivar selection, mulching, deficit irrigation, and shade structures are logged in a machine-readable record so audits can reproduce the management decisions.

## 2. Scope

The standard applies to:

- Open-field cultivation in arid, hyper-arid, semi-arid, and dry sub-humid zones (UNEP World Atlas of Desertification).
- Drip, sub-surface drip, micro-sprinkler, and pivot-irrigation systems where irrigation is the dominant water source.
- Greenhouse and shade-house operations situated in arid zones, where ET₀ informs ventilation and humidification controls.
- Public and private soil-monitoring stations reporting under ISO 11277:2020 (texture) and ISO 16586:2003 (water content).

Out of scope: rain-fed humid agriculture (P/PET ≥ 0.65), aquaculture, and animal husbandry without irrigated forage.

## 3. Key Capabilities

- **Reference ET₀ computation** following FAO-56 Eq. 6 (Penman-Monteith), with required inputs (T_min, T_max, RH_mean, u₂, R_s, elevation, latitude).
- **Aridity-index reporting** per UNEP P/PET classes.
- **Soil salinity classification** per FAO ECe bands (non-saline < 2 dS/m through very strongly saline ≥ 16 dS/m).
- **Crop water requirement (ETc)** with FAO-56 single-Kc and dual-Kc options.
- **Water-use efficiency (WUE)** in m³ irrigation water per ton harvested fresh weight.
- **Mulching and shade reporting** with surface-cover percentage and material class.
- **Renewable-energy integration** logging for solar-pump and wind-pump systems.

## 4. TypeScript SDK

### Installation

```bash
npm install @wia/desert-agriculture-sdk
```

### Usage

```typescript
import { createClient } from '@wia/desert-agriculture-sdk';

const client = createClient({
  baseURL: 'https://api.wia-agri.org',
  apiKey: process.env.WIA_API_KEY!
});

const farm = await client.registerFarm({
  name: 'Sahara Green Farm',
  location: {
    latitude: 30.5,
    longitude: -8.0,
    elevation: 450,
    region: 'Sahara',
    country: 'Morocco',
    timezone: 'Africa/Casablanca'
  },
  area: { value: 100, unit: 'hectares' },
  aridity_index: 0.12,
  irrigation_method: 'drip-subsurface',
  crops: ['date-palm', 'olive', 'pomegranate']
});

const et0 = await client.computeReferenceET({
  station_id: 'AGADIR-WX-01',
  date: '2026-04-26',
  T_min_C: 14.2,
  T_max_C: 28.7,
  RH_mean_pct: 36,
  u2_m_s: 2.1,
  R_s_MJ_m2_day: 24.4
});
console.log('ET₀ (mm/day):', et0.mm_per_day);

const salinity = await client.classifySalinity({
  ECe_dS_m: 5.4,
  reference: 'FAO-Soils-Bulletin-39'
});
console.log('Class:', salinity.class);  // "moderately-saline"
```

## 5. CLI Tool

```bash
chmod +x cli/desert-agriculture.sh
./cli/desert-agriculture.sh validate farm.json
./cli/desert-agriculture.sh et0 weather.json
./cli/desert-agriculture.sh aridity station.json
./cli/desert-agriculture.sh salinity soil.json
./cli/desert-agriculture.sh water farm.json
```

## 6. Reference Standards

| Reference | Role |
|-----------|------|
| FAO Irrigation and Drainage Paper 56 | ET₀ Penman-Monteith Eq. 6, Kc tables |
| FAO Soils Bulletin 39 (Ayers & Westcot) | ECe salinity bands |
| UNEP World Atlas of Desertification (1997) | Aridity index P/PET classes |
| ISO 11277:2020 | Soil quality — particle-size distribution |
| ISO 16586:2003 | Soil quality — water content / matric potential |
| ISO 19115-1:2014 | Geographic metadata for farm location |

All citations conform to the WIA Citation & Veracity Policy v1.0 (`docs/CITATION-POLICY.md` §2.1 ALLOW).

## 7. License

MIT License.

---

© 2025 WIA (World Certification Industry Association)
**弘益人間 (홍익인간)** — Benefit All Humanity 🌍
