# WIA-ENE-060: Wetland Conservation
## PHASE 1 - DATA FORMAT SPECIFICATION

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25

---

## Overview

This specification defines the standard data format for wetland conservation monitoring, including wetland characteristics, water quality metrics, biodiversity indicators, and ecosystem health data.

## Core Data Structure

### Wetland Identity

```json
{
  "wetland": {
    "id": "string (WL-XXXXXXXXX)",
    "name": "string",
    "type": "enum [marsh, swamp, bog, fen, coastal, riverine]",
    "ramsar_site": "boolean",
    "ramsar_id": "string (optional)",
    "location": {
      "latitude": "number",
      "longitude": "number",
      "country": "string (ISO 3166-1 alpha-2)",
      "region": "string",
      "elevation_m": "number"
    },
    "area_hectares": "number",
    "established_date": "string (ISO 8601)"
  }
}
```

### Water Quality Metrics

```json
{
  "water_quality": {
    "timestamp": "string (ISO 8601)",
    "ph": "number (0-14)",
    "dissolved_oxygen_mg_l": "number",
    "temperature_celsius": "number",
    "turbidity_ntu": "number",
    "conductivity_us_cm": "number",
    "nutrients": {
      "nitrate_mg_l": "number",
      "phosphate_mg_l": "number",
      "ammonia_mg_l": "number"
    },
    "pollutants": {
      "heavy_metals": {
        "lead_ug_l": "number",
        "mercury_ug_l": "number",
        "cadmium_ug_l": "number"
      },
      "pesticides_ug_l": "number",
      "microplastics_particles_l": "number"
    },
    "quality_index": "number (0-100)"
  }
}
```

### Biodiversity Data

```json
{
  "biodiversity": {
    "survey_date": "string (ISO 8601)",
    "surveyor_id": "string",
    "birds": {
      "species_count": "number",
      "individual_count": "number",
      "migratory_species": "number",
      "endangered_species": [
        {
          "scientific_name": "string",
          "common_name": "string",
          "iucn_status": "enum [CR, EN, VU, NT, LC]",
          "population_estimate": "number"
        }
      ],
      "breeding_pairs": "number"
    },
    "amphibians": {
      "species_count": "number",
      "individual_count": "number",
      "endangered_species": "array"
    },
    "fish": {
      "species_count": "number",
      "native_species_ratio": "number (0-1)",
      "invasive_species": "array"
    },
    "plants": {
      "species_count": "number",
      "endemic_species": "number",
      "invasive_species": "array"
    },
    "invertebrates": {
      "species_count": "number",
      "pollinator_species": "number"
    },
    "biodiversity_index": "number (0-100)"
  }
}
```

### Vegetation Metrics

```json
{
  "vegetation": {
    "assessment_date": "string (ISO 8601)",
    "coverage_percent": "number (0-100)",
    "native_species_ratio": "number (0-1)",
    "canopy_density": "number (0-1)",
    "biomass_tons_per_hectare": "number",
    "dominant_species": [
      {
        "scientific_name": "string",
        "common_name": "string",
        "coverage_percent": "number"
      }
    ],
    "invasive_species": [
      {
        "scientific_name": "string",
        "common_name": "string",
        "coverage_percent": "number",
        "management_status": "enum [controlled, monitoring, spreading, eradicated]"
      }
    ],
    "health_index": "number (0-100)"
  }
}
```

### Hydrology Data

```json
{
  "hydrology": {
    "measurement_date": "string (ISO 8601)",
    "water_level_m": "number",
    "seasonal_variation_m": "number",
    "water_source": "enum [rainfall, river, groundwater, tidal, mixed]",
    "inflow_rate_m3_day": "number",
    "outflow_rate_m3_day": "number",
    "water_retention_days": "number",
    "flood_frequency_per_year": "number",
    "drought_risk": "enum [low, moderate, high, severe]"
  }
}
```

### Ecosystem Services

```json
{
  "ecosystem_services": {
    "calculated_date": "string (ISO 8601)",
    "water_purification": {
      "capacity_liters_per_day": "number",
      "pollutant_removal_rate": "number (0-1)",
      "annual_value_usd": "number"
    },
    "flood_mitigation": {
      "storage_capacity_m3": "number",
      "protected_area_km2": "number",
      "annual_value_usd": "number"
    },
    "carbon_sequestration": {
      "rate_tons_per_year": "number",
      "total_stored_tons": "number",
      "annual_value_usd": "number"
    },
    "habitat_provision": {
      "species_supported": "number",
      "critical_habitat_species": "number",
      "annual_value_usd": "number"
    },
    "recreation": {
      "annual_visitors": "number",
      "economic_value_usd": "number"
    },
    "total_ecosystem_value_usd": "number"
  }
}
```

### Threats and Pressures

```json
{
  "threats": {
    "assessment_date": "string (ISO 8601)",
    "pollution": {
      "severity": "enum [none, low, moderate, high, critical]",
      "sources": "array of strings",
      "trend": "enum [improving, stable, worsening]"
    },
    "habitat_loss": {
      "severity": "enum [none, low, moderate, high, critical]",
      "causes": "array of strings",
      "area_lost_hectares_per_year": "number"
    },
    "invasive_species": {
      "severity": "enum [none, low, moderate, high, critical]",
      "species_count": "number",
      "management_active": "boolean"
    },
    "climate_change": {
      "temperature_increase_celsius": "number",
      "precipitation_change_percent": "number",
      "sea_level_rise_cm": "number (for coastal wetlands)"
    },
    "human_disturbance": {
      "severity": "enum [none, low, moderate, high, critical]",
      "activities": "array of strings"
    },
    "overall_threat_level": "enum [low, moderate, high, critical]"
  }
}
```

### Conservation Actions

```json
{
  "conservation": {
    "protection_status": "enum [none, partial, full, ramsar, unesco]",
    "management_plan": "boolean",
    "last_updated": "string (ISO 8601)",
    "active_projects": [
      {
        "id": "string",
        "name": "string",
        "type": "enum [restoration, monitoring, education, research, protection]",
        "start_date": "string (ISO 8601)",
        "end_date": "string (ISO 8601)",
        "budget_usd": "number",
        "lead_organization": "string"
      }
    ],
    "monitoring_frequency": "enum [daily, weekly, monthly, quarterly, annual]",
    "stakeholders": [
      {
        "name": "string",
        "type": "enum [government, ngo, community, research, private]",
        "role": "string"
      }
    ]
  }
}
```

## Complete Example

```json
{
  "wetland": {
    "id": "WL-OKAV-2025",
    "name": "Okavango Delta",
    "type": "riverine",
    "ramsar_site": true,
    "ramsar_id": "RS1234",
    "location": {
      "latitude": -19.2833,
      "longitude": 22.7333,
      "country": "BW",
      "region": "Northwest District",
      "elevation_m": 945
    },
    "area_hectares": 1500000,
    "established_date": "1996-12-04"
  },
  "water_quality": {
    "timestamp": "2025-12-25T10:00:00Z",
    "ph": 7.2,
    "dissolved_oxygen_mg_l": 8.5,
    "temperature_celsius": 24.5,
    "turbidity_ntu": 12,
    "conductivity_us_cm": 185,
    "nutrients": {
      "nitrate_mg_l": 0.8,
      "phosphate_mg_l": 0.15,
      "ammonia_mg_l": 0.05
    },
    "pollutants": {
      "heavy_metals": {
        "lead_ug_l": 2.1,
        "mercury_ug_l": 0.3,
        "cadmium_ug_l": 0.5
      },
      "pesticides_ug_l": 1.2,
      "microplastics_particles_l": 3
    },
    "quality_index": 88
  },
  "biodiversity": {
    "survey_date": "2025-12-20",
    "surveyor_id": "WIA-SURVEYOR-001",
    "birds": {
      "species_count": 444,
      "individual_count": 200000,
      "migratory_species": 165,
      "endangered_species": [
        {
          "scientific_name": "Grus carunculata",
          "common_name": "Wattled Crane",
          "iucn_status": "VU",
          "population_estimate": 250
        }
      ],
      "breeding_pairs": 8500
    },
    "amphibians": {
      "species_count": 33,
      "individual_count": 150000,
      "endangered_species": []
    },
    "fish": {
      "species_count": 71,
      "native_species_ratio": 0.97,
      "invasive_species": []
    },
    "plants": {
      "species_count": 1300,
      "endemic_species": 89,
      "invasive_species": []
    },
    "invertebrates": {
      "species_count": 2500,
      "pollinator_species": 450
    },
    "biodiversity_index": 94
  },
  "vegetation": {
    "assessment_date": "2025-12-15",
    "coverage_percent": 85,
    "native_species_ratio": 0.98,
    "canopy_density": 0.65,
    "biomass_tons_per_hectare": 125,
    "dominant_species": [
      {
        "scientific_name": "Cyperus papyrus",
        "common_name": "Papyrus",
        "coverage_percent": 35
      }
    ],
    "invasive_species": [],
    "health_index": 92
  },
  "hydrology": {
    "measurement_date": "2025-12-25",
    "water_level_m": 2.8,
    "seasonal_variation_m": 1.5,
    "water_source": "river",
    "inflow_rate_m3_day": 12000000,
    "outflow_rate_m3_day": 9500000,
    "water_retention_days": 180,
    "flood_frequency_per_year": 1,
    "drought_risk": "low"
  },
  "ecosystem_services": {
    "calculated_date": "2025-12-25",
    "water_purification": {
      "capacity_liters_per_day": 150000000,
      "pollutant_removal_rate": 0.82,
      "annual_value_usd": 45000000
    },
    "flood_mitigation": {
      "storage_capacity_m3": 18000000000,
      "protected_area_km2": 2500,
      "annual_value_usd": 125000000
    },
    "carbon_sequestration": {
      "rate_tons_per_year": 3750000,
      "total_stored_tons": 225000000,
      "annual_value_usd": 187500000
    },
    "habitat_provision": {
      "species_supported": 4500,
      "critical_habitat_species": 35,
      "annual_value_usd": 95000000
    },
    "recreation": {
      "annual_visitors": 85000,
      "economic_value_usd": 178000000
    },
    "total_ecosystem_value_usd": 630500000
  },
  "threats": {
    "assessment_date": "2025-12-01",
    "pollution": {
      "severity": "low",
      "sources": ["agricultural runoff"],
      "trend": "stable"
    },
    "habitat_loss": {
      "severity": "low",
      "causes": ["climate variability"],
      "area_lost_hectares_per_year": 50
    },
    "invasive_species": {
      "severity": "none",
      "species_count": 0,
      "management_active": true
    },
    "climate_change": {
      "temperature_increase_celsius": 1.2,
      "precipitation_change_percent": -5,
      "sea_level_rise_cm": 0
    },
    "human_disturbance": {
      "severity": "low",
      "activities": ["tourism", "fishing"]
    },
    "overall_threat_level": "low"
  },
  "conservation": {
    "protection_status": "unesco",
    "management_plan": true,
    "last_updated": "2024-06-15",
    "active_projects": [
      {
        "id": "PROJ-001",
        "name": "Delta Monitoring Network",
        "type": "monitoring",
        "start_date": "2020-01-01",
        "end_date": "2030-12-31",
        "budget_usd": 5000000,
        "lead_organization": "Okavango Research Institute"
      }
    ],
    "monitoring_frequency": "monthly",
    "stakeholders": [
      {
        "name": "Government of Botswana",
        "type": "government",
        "role": "Primary management authority"
      }
    ]
  }
}
```

## Validation Rules

1. **ID Format**: Wetland IDs must follow pattern `WL-[A-Z0-9]{4,12}`
2. **Date Format**: All dates must be ISO 8601 compliant
3. **Coordinates**: Latitude (-90 to 90), Longitude (-180 to 180)
4. **Percentages**: 0-100 range
5. **Ratios**: 0-1 range
6. **Indices**: 0-100 range
7. **Quality Index**: Calculated from water parameters, biodiversity, and vegetation health

## API Compatibility

This data format is designed for:
- RESTful API transmission (JSON)
- Time-series database storage
- GIS system integration
- Real-time monitoring dashboards
- Verifiable credentials generation

---

**Standard:** WIA-ENE-060
**Category:** Energy & Environment
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
