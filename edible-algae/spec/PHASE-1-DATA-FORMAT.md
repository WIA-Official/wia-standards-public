# WIA Edible Algae Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - Agriculture)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Cultivation Batch Schema](#cultivation-batch-schema)
5. [Algae Species Classification](#algae-species-classification)
6. [Growth Parameters](#growth-parameters)
7. [Nutrient Composition](#nutrient-composition)
8. [Quality Metrics](#quality-metrics)
9. [Harvest Data Format](#harvest-data-format)
10. [Validation Rules](#validation-rules)
11. [Examples](#examples)
12. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Edible Algae Data Format Standard defines a unified framework for algae cultivation data, enabling standardized tracking of spirulina, chlorella, seaweed, and microalgae production for sustainable food and protein sources.

**Core Objectives**:
- Standardize algae cultivation batch tracking
- Enable photobioreactor and open pond monitoring
- Support nutritional profiling and quality assurance
- Facilitate harvest optimization and yield prediction
- Enable traceability from cultivation to consumer
- Support sustainable protein production metrics
- Promote food security and environmental sustainability

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Species Identity | Algae species, strain, genetic markers |
| Cultivation System | Photobioreactor, open pond, sea farm, vertical farm |
| Growth Monitoring | Cell density, biomass, growth rate, harvest cycles |
| Water Quality | Temperature, pH, salinity, dissolved oxygen, nutrients |
| Nutritional Profile | Protein, lipids, carbohydrates, vitamins, minerals |
| Bioactive Compounds | Chlorophyll, carotenoids, phycocyanin, omega-3 |
| Harvest Data | Yield, density, moisture content, processing method |
| Sustainability Metrics | CO2 sequestration, water use, energy efficiency |

### 1.3 Design Principles

1. **Traceability**: Full chain of custody from cultivation to product
2. **Nutrition Focus**: Comprehensive nutritional and bioactive data
3. **Sustainability**: Track environmental impact and carbon sequestration
4. **Quality Assurance**: Real-time monitoring and quality control
5. **Interoperability**: Compatible with aquaculture and food safety systems
6. **Scalability**: Support micro-cultivation to industrial production

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Edible Algae** | Algae species cultivated for human consumption |
| **Spirulina** | Blue-green algae (cyanobacteria) high in protein |
| **Chlorella** | Green microalgae rich in chlorophyll |
| **Photobioreactor (PBR)** | Closed system for controlled algae cultivation |
| **Open Pond** | Outdoor cultivation in raceway ponds |
| **Biomass** | Total algae cell mass (dry weight) |
| **Phycocyanin** | Blue pigment protein in spirulina |
| **Carotenoids** | Antioxidant pigments (beta-carotene, astaxanthin) |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `cultivation_id` | Unique batch identifier | `"CULT-2025-001"` |
| `species_code` | Algae species identifier | `"SPIRULINA_PLATENSIS"` |
| `biomass_density` | Concentration (g/L) | `4.5` |
| `ph_level` | Acidity/alkalinity | `7.8` |
| `protein_content` | Protein percentage | `65.2` |
| `harvest_date` | ISO 8601 date | `"2025-01-15T10:30:00Z"` |
| `nutrient_code` | Nutrient type | `"NITROGEN"`, `"PHOSPHORUS"` |

---

## Base Structure

### 3.1 Cultivation Batch Object

```json
{
  "cultivation_id": "CULT-2025-ALG-001",
  "facility_id": "FAC-OCEAN-FARMS-01",
  "created_at": "2025-01-01T08:00:00Z",
  "updated_at": "2025-01-15T16:30:00Z",
  "status": "ACTIVE",
  "species": {
    "scientific_name": "Arthrospira platensis",
    "common_name": "Spirulina",
    "strain_id": "SPR-HI-2023",
    "genetic_marker": "ITS-rRNA-NCBI-XY123456"
  },
  "cultivation_system": {
    "type": "PHOTOBIOREACTOR",
    "model": "TubularPBR-5000L",
    "volume_liters": 5000,
    "material": "Transparent acrylic",
    "automation_level": "FULLY_AUTOMATED"
  },
  "environmental_parameters": {
    "temperature": {
      "current": 25.5,
      "setpoint": 25.0,
      "min": 20.0,
      "max": 30.0,
      "unit": "celsius"
    },
    "ph": {
      "current": 7.8,
      "setpoint": 7.8,
      "min": 7.0,
      "max": 8.5,
      "unit": "pH"
    },
    "light_intensity": {
      "current": 400,
      "setpoint": 400,
      "photoperiod": "16:8",
      "unit": "umol/m2/s"
    },
    "dissolved_oxygen": {
      "current": 7.5,
      "min": 5.0,
      "unit": "mg/L"
    },
    "salinity": {
      "current": 0,
      "unit": "ppt",
      "note": "Freshwater for Spirulina"
    }
  },
  "nutrient_management": {
    "nitrogen_source": "UREA",
    "nitrogen_concentration": 150.0,
    "phosphorus_concentration": 25.0,
    "potassium_concentration": 35.0,
    "trace_elements": ["Fe", "Mn", "Zn", "Cu", "Mo", "B"],
    "co2_supply_rate": 2.5,
    "unit": "mg/L"
  },
  "biomass_tracking": {
    "initial_density": 0.5,
    "current_density": 3.8,
    "target_density": 4.5,
    "growth_rate": 0.32,
    "doubling_time_hours": 52.0,
    "unit": "g/L"
  },
  "harvest_schedule": {
    "start_date": "2025-01-01",
    "estimated_harvest": "2025-01-18",
    "harvest_method": "CONTINUOUS_FLOW",
    "harvest_percentage": 30
  }
}
```

### 3.2 Required Fields

**Minimum Required**:
- `cultivation_id`
- `facility_id`
- `created_at`
- `species.scientific_name`
- `cultivation_system.type`
- `cultivation_system.volume_liters`
- `environmental_parameters.temperature`
- `environmental_parameters.ph`
- `biomass_tracking.current_density`

---

## Cultivation Batch Schema

### 4.1 Batch Lifecycle States

| State | Description | Triggers |
|-------|-------------|----------|
| `PREPARATION` | System sterilization, nutrient loading | Manual |
| `INOCULATION` | Starter culture added | Operator action |
| `ACTIVE_GROWTH` | Exponential growth phase | Auto (density threshold) |
| `STATIONARY` | Growth plateau, harvest ready | Auto (target density) |
| `HARVESTING` | Biomass extraction in progress | Operator action |
| `COMPLETE` | Batch finished, yield recorded | System |
| `TERMINATED` | Batch failed or contaminated | Operator/System |

### 4.2 Growth Monitoring Schema

```json
{
  "growth_monitoring": {
    "measurements": [
      {
        "timestamp": "2025-01-15T10:00:00Z",
        "optical_density_OD680": 1.25,
        "biomass_concentration_g_per_L": 3.8,
        "cell_count_per_mL": 4.2e8,
        "chlorophyll_content_mg_per_L": 45.2,
        "ph": 7.82,
        "temperature_celsius": 25.3,
        "dissolved_oxygen_mg_per_L": 7.6,
        "turbidity_NTU": 250
      }
    ],
    "growth_curve_fit": {
      "model": "LOGISTIC",
      "r_squared": 0.98,
      "max_specific_growth_rate": 0.35,
      "lag_phase_hours": 12,
      "exponential_phase_hours": 120
    }
  }
}
```

---

## Algae Species Classification

### 5.1 Supported Species

| Species Code | Scientific Name | Common Name | Primary Use |
|--------------|-----------------|-------------|-------------|
| `SPIRULINA_PLATENSIS` | Arthrospira platensis | Spirulina | Protein, phycocyanin |
| `SPIRULINA_MAXIMA` | Arthrospira maxima | Spirulina | Nutritional supplement |
| `CHLORELLA_VULGARIS` | Chlorella vulgaris | Chlorella | Chlorophyll, detox |
| `CHLORELLA_PYRENOIDOSA` | Chlorella pyrenoidosa | Chlorella | Protein, CGF |
| `NANNOCHLOROPSIS` | Nannochloropsis oculata | Nano | Omega-3 EPA |
| `DUNALIELLA_SALINA` | Dunaliella salina | Dunaliella | Beta-carotene |
| `HAEMATOCOCCUS` | Haematococcus pluvialis | Blood algae | Astaxanthin |
| `KELP` | Saccharina japonica | Kombu | Iodine, minerals |
| `WAKAME` | Undaria pinnatifida | Wakame | Fucoxanthin |
| `NORI` | Porphyra spp. | Nori | Protein, vitamins |

### 5.2 Species-Specific Parameters

```json
{
  "species_parameters": {
    "SPIRULINA_PLATENSIS": {
      "optimal_temperature": 35,
      "optimal_ph": 9.5,
      "salinity_ppt": 0,
      "light_saturation": 400,
      "protein_content_range": "55-70%",
      "harvest_density_g_per_L": "4-8",
      "productivity_g_per_L_per_day": "0.3-0.5"
    },
    "CHLORELLA_VULGARIS": {
      "optimal_temperature": 30,
      "optimal_ph": 7.5,
      "salinity_ppt": 0,
      "light_saturation": 500,
      "protein_content_range": "50-60%",
      "harvest_density_g_per_L": "3-5",
      "productivity_g_per_L_per_day": "0.2-0.4"
    }
  }
}
```

---

## Growth Parameters

### 6.1 Kinetic Parameters

```json
{
  "kinetic_parameters": {
    "specific_growth_rate_per_day": 0.32,
    "doubling_time_hours": 52.0,
    "productivity_volumetric_g_per_L_per_day": 0.38,
    "productivity_areal_g_per_m2_per_day": 25.0,
    "light_conversion_efficiency_percent": 8.5,
    "co2_fixation_rate_g_per_L_per_day": 0.68,
    "biomass_yield_on_light_g_per_mol_photons": 1.2
  }
}
```

### 6.2 Environmental Response

```json
{
  "environmental_response": {
    "temperature_coefficient_Q10": 2.1,
    "light_saturation_point": 450,
    "light_inhibition_threshold": 1200,
    "co2_saturation_percent": 5.0,
    "optimal_nitrogen_mg_per_L": 150,
    "nitrogen_limitation_threshold": 50
  }
}
```

---

## Nutrient Composition

### 7.1 Macronutrient Profile

```json
{
  "macronutrients": {
    "protein_percent": 65.2,
    "carbohydrates_percent": 18.5,
    "lipids_percent": 8.3,
    "ash_percent": 6.0,
    "moisture_percent": 2.0,
    "fiber_percent": 3.5,
    "energy_kcal_per_100g": 380
  }
}
```

### 7.2 Micronutrient Profile

```json
{
  "micronutrients": {
    "vitamins": {
      "vitamin_B12_ug_per_100g": 250,
      "vitamin_A_IU_per_100g": 28000,
      "vitamin_K_ug_per_100g": 1200,
      "vitamin_E_mg_per_100g": 5.0,
      "thiamin_B1_mg_per_100g": 2.38,
      "riboflavin_B2_mg_per_100g": 3.67,
      "niacin_B3_mg_per_100g": 12.82
    },
    "minerals": {
      "iron_mg_per_100g": 28.5,
      "calcium_mg_per_100g": 120,
      "magnesium_mg_per_100g": 195,
      "zinc_mg_per_100g": 2.0,
      "selenium_ug_per_100g": 7.2,
      "iodine_ug_per_100g": 150
    }
  }
}
```

### 7.3 Bioactive Compounds

```json
{
  "bioactive_compounds": {
    "chlorophyll_a_mg_per_g": 12.5,
    "carotenoids": {
      "beta_carotene_mg_per_g": 1.7,
      "zeaxanthin_mg_per_g": 0.8,
      "astaxanthin_mg_per_g": 0.0
    },
    "phycocyanin_percent": 15.0,
    "omega_3_fatty_acids": {
      "total_omega3_percent": 1.2,
      "epa_mg_per_g": 4.5,
      "dha_mg_per_g": 2.0,
      "ala_mg_per_g": 8.0
    },
    "polysaccharides_percent": 4.5,
    "antioxidant_capacity_umol_TE_per_g": 280
  }
}
```

---

## Quality Metrics

### 8.1 Purity Assessment

```json
{
  "quality_control": {
    "purity_percent": 99.5,
    "contamination_screening": {
      "bacteria_cfu_per_g": 100,
      "yeast_cfu_per_g": 10,
      "mold_cfu_per_g": 10,
      "e_coli": "ABSENT",
      "salmonella": "ABSENT"
    },
    "heavy_metals_ppm": {
      "lead": 0.05,
      "cadmium": 0.02,
      "mercury": 0.01,
      "arsenic": 0.03
    },
    "pesticide_residue": "NONE_DETECTED",
    "allergen_status": "ALLERGEN_FREE",
    "gmo_status": "NON_GMO"
  }
}
```

### 8.2 Safety Certifications

```json
{
  "certifications": [
    {
      "standard": "ORGANIC",
      "certifier": "USDA Organic",
      "certificate_id": "ORG-2024-12345",
      "valid_until": "2026-01-01"
    },
    {
      "standard": "ISO_22000",
      "certifier": "SGS",
      "certificate_id": "ISO22000-2024-67890",
      "valid_until": "2027-01-01"
    },
    {
      "standard": "KOSHER",
      "certifier": "OK Kosher",
      "certificate_id": "K-2024-ALG-001",
      "valid_until": "2026-01-01"
    }
  ]
}
```

---

## Harvest Data Format

### 9.1 Harvest Record

```json
{
  "harvest_record": {
    "harvest_id": "HRV-2025-ALG-001",
    "cultivation_id": "CULT-2025-ALG-001",
    "harvest_date": "2025-01-18T14:00:00Z",
    "harvest_method": "CONTINUOUS_FLOW",
    "biomass_harvested": {
      "wet_weight_kg": 450,
      "dry_weight_kg": 22.5,
      "moisture_content_percent": 95,
      "cell_density_g_per_L": 4.8
    },
    "harvest_efficiency": {
      "recovery_rate_percent": 85,
      "cell_viability_percent": 92,
      "processing_time_hours": 3.5
    },
    "post_harvest_processing": {
      "dewatering_method": "CENTRIFUGATION",
      "drying_method": "SPRAY_DRYING",
      "drying_temperature_celsius": 55,
      "final_moisture_percent": 5.0,
      "powder_yield_kg": 21.4
    },
    "storage": {
      "storage_temperature_celsius": 4,
      "storage_humidity_percent": 30,
      "packaging": "Nitrogen_flushed_pouches",
      "shelf_life_months": 24
    }
  }
}
```

---

## Validation Rules

### 10.1 Field Validation

| Field | Rule | Error Message |
|-------|------|---------------|
| `cultivation_id` | Unique, format `CULT-YYYY-XXX-###` | Invalid cultivation ID |
| `temperature` | 15-40°C | Temperature out of safe range |
| `ph` | 6.0-10.0 | pH out of safe range |
| `biomass_density` | 0.1-10.0 g/L | Biomass density unrealistic |
| `protein_content` | 30-75% | Protein content out of range |
| `harvest_date` | ISO 8601, future allowed for estimates | Invalid date format |

### 10.2 Business Logic Validation

```javascript
// Harvest readiness check
if (biomass_density >= target_density && cultivation_days >= min_days) {
  status = "READY_FOR_HARVEST";
}

// Contamination alert
if (bacteria_cfu > 1000 || e_coli === "PRESENT") {
  alert = "CONTAMINATION_DETECTED";
  action = "TERMINATE_BATCH";
}

// Growth rate anomaly
if (specific_growth_rate < 0.1 || specific_growth_rate > 1.0) {
  alert = "ABNORMAL_GROWTH_RATE";
  action = "CHECK_SYSTEM";
}
```

---

## Examples

### 11.1 Spirulina Cultivation Batch

```json
{
  "cultivation_id": "CULT-2025-SPR-042",
  "facility_id": "FAC-HAWAII-ALGAE-01",
  "created_at": "2025-01-01T08:00:00Z",
  "status": "ACTIVE_GROWTH",
  "species": {
    "scientific_name": "Arthrospira platensis",
    "common_name": "Spirulina",
    "strain_id": "SPR-HI-2023"
  },
  "cultivation_system": {
    "type": "OPEN_POND",
    "model": "Raceway-50000L",
    "volume_liters": 50000
  },
  "environmental_parameters": {
    "temperature": {"current": 32.0, "unit": "celsius"},
    "ph": {"current": 9.2, "unit": "pH"},
    "light_intensity": {"current": 1200, "unit": "umol/m2/s"}
  },
  "biomass_tracking": {
    "current_density": 2.5,
    "target_density": 3.5,
    "growth_rate": 0.28,
    "unit": "g/L"
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

**弘益人間 (Hongik Ingan)** · Benefit All Humanity

© 2025 WIA Standards - MIT License
