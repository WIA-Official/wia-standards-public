# WIA-AGRI-034: Hydroponics Standard

> **Soil-less cultivation, nutrient-solution management, and controlled-environment agriculture (CEA).**
> Aligned with ISO 11148 (greenhouse environment), ISO 21500 series (project metadata), IEC 61131-3 (PLC programming languages), and W3C SOSA/SSN ontologies for sensor metadata.

## 1. Overview

The WIA-AGRI-034 Hydroponics Standard defines machine-readable schemas, reference algorithms, and reporting metrics for soil-less plant production systems operating under controlled environment conditions. The standard covers six common hydroponic system classes:

1. **Deep Water Culture (DWC)** — roots suspended in oxygenated nutrient solution.
2. **Nutrient Film Technique (NFT)** — thin film of recirculated solution flowing across roots.
3. **Ebb-and-Flow / Flood-and-Drain** — periodic flooding of the root zone.
4. **Drip systems** — substrate-based (rockwool, coco coir, perlite) with timed nutrient drip.
5. **Aeroponics** — roots periodically misted with nutrient solution.
6. **Vertical / Plant Factory** — stacked layers with artificial lighting and recirculation.

The standard targets four operational concerns:

- **Nutrient solution composition** — macronutrient (N, P, K, Ca, Mg, S) and micronutrient (Fe, Mn, B, Zn, Cu, Mo) targets in mg/L, with EC (electrical conductivity, mS/cm) and pH bounds per crop class.
- **Environmental control** — temperature, humidity, vapor pressure deficit (VPD), CO₂ enrichment, and PPFD (photosynthetic photon flux density, μmol·m⁻²·s⁻¹).
- **Water and nutrient circularity** — discharge volumes, recapture ratio, and nutrient solution renewal interval.
- **Light recipe traceability** — spectral composition (% red 600–700 nm, % blue 400–500 nm, % far-red, % UV) and photoperiod per growth stage.

## 2. Scope

The standard applies to:

- Commercial and research-scale hydroponic / aeroponic / aquaponic facilities reporting under WIA standards.
- Greenhouse climate-control systems implementing IEC 61131-3 PLC logic and BACnet/Modbus interfaces.
- Vertical farms and plant factories using LED light recipes.
- Sensor networks publishing telemetry under the W3C SOSA/SSN ontology.

Out of scope: open-field hydroponic adaptations operating outside controlled environments, animal-component aquaponics regulated under separate WIA aquaculture standards, and ornamental tissue culture.

## 3. Key Capabilities

- **System-class declaration** with schema-validated parameters (DWC oxygenation rate, NFT slope, aeroponic mist interval, etc.).
- **Nutrient solution recipe** in standard ion-balance format with EC/pH targets and tolerance bands.
- **Environmental setpoint logging** for day/night temperature, VPD, CO₂, and PPFD.
- **Light recipe metadata** with full spectral breakdown and photoperiod schedule.
- **Water-budget reporting** (input, recirculated, evapotranspired, discharged) in liters per kg fresh produce.
- **Pest and pathogen log** with non-pesticidal interventions (biocontrol agents, UV sterilization, ozonation).
- **Energy reporting** in kWh per kg produce, separated by lighting, HVAC, pumps, and controls.

## 4. TypeScript SDK

### Installation

```bash
npm install @wia/hydroponics-sdk
```

### Usage

```typescript
import { createClient } from '@wia/hydroponics-sdk';

const client = createClient({
  baseURL: 'https://api.wia-agri.org',
  apiKey: process.env.WIA_API_KEY!
});

// Register a hydroponic facility
const facility = await client.registerFacility({
  facility_id: 'NFT-LETTUCE-NL-01',
  system_class: 'NFT',
  location: { latitude: 52.0, longitude: 5.6, country: 'NL' },
  growing_area_m2: 1200,
  crops: ['Lactuca sativa cv. Salanova']
});

// Submit nutrient recipe
const recipe = await client.submitRecipe({
  facility_id: facility.facility_id,
  stage: 'vegetative',
  EC_mS_cm: 1.8,
  pH: 5.8,
  ions_mg_L: {
    NO3: 168, NH4: 14, P: 31, K: 195, Ca: 160, Mg: 24, S: 32,
    Fe: 0.84, Mn: 0.55, B: 0.27, Zn: 0.13, Cu: 0.05, Mo: 0.05
  }
});

// Light recipe (LED)
await client.submitLightRecipe({
  facility_id: facility.facility_id,
  stage: 'vegetative',
  PPFD_umol_m2_s: 220,
  photoperiod_h: 16,
  spectrum_pct: { red_600_700: 70, blue_400_500: 20, far_red: 5, green: 5 }
});
```

## 5. CLI Tool

```bash
chmod +x cli/hydroponics.sh
./cli/hydroponics.sh validate facility.json
./cli/hydroponics.sh ec-target lettuce
./cli/hydroponics.sh vpd 24 65        # T=24°C, RH=65% → VPD kPa
./cli/hydroponics.sh ppfd-to-dli 220 16
./cli/hydroponics.sh recipe-check recipe.json
```

## 6. Reference Standards

| Reference | Role |
|-----------|------|
| ISO 11148 series | Greenhouse and controlled-environment requirements |
| IEC 61131-3 | PLC programming languages for greenhouse automation |
| W3C SOSA/SSN | Sensor metadata vocabulary for environmental telemetry |
| ISO 19115-1:2014 | Geographic metadata for facility location |
| ISO 8601 | Date/time encoding for setpoint and harvest logs |
| RFC 8259 | JSON encoding of recipe and telemetry payloads |
| FAO Plant Production and Protection Paper 217 | Soilless culture reference |

All citations conform to the WIA Citation & Veracity Policy v1.0 (`docs/CITATION-POLICY.md` §2.1 ALLOW).

## 7. License

MIT License.

---

© 2025 WIA (World Certification Industry Association)
**弘益人間 (홍익인간)** — Benefit All Humanity 🌱
