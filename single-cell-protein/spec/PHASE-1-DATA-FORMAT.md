# WIA-AGRI-034: Phase 1 - Data Format Specification

**Standard ID:** WIA-AGRI-034
**Title:** Single Cell Protein Production Data Format
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the standardized data format for Single Cell Protein (SCP) production systems, enabling interoperability between different fermentation platforms, quality control systems, and supply chain stakeholders.

### 1.1 Scope

This standard covers:
- Microbial strain documentation
- Fermentation process parameters
- Quality and safety metrics
- Production and efficiency data
- Environmental impact measurements

### 1.2 Objectives

- **Interoperability**: Enable seamless data exchange between SCP production systems
- **Traceability**: Maintain complete audit trails from strain to final product
- **Quality Assurance**: Standardize quality metrics and testing protocols
- **Optimization**: Facilitate AI/ML-driven process optimization
- **Sustainability**: Track environmental impact and resource efficiency

---

## 2. Core Data Structures

### 2.1 Batch Record

```json
{
  "batch_id": "string (required, unique)",
  "version": "1.0",
  "protocol": "WIA-AGRI-034",
  "timestamp": "ISO8601 datetime",
  "facility": {
    "id": "string",
    "name": "string",
    "location": {
      "country": "string",
      "region": "string",
      "coordinates": {
        "latitude": "number",
        "longitude": "number"
      }
    },
    "certifications": ["string"]
  },
  "strain": {
    "id": "string (required)",
    "taxonomy": {
      "domain": "Bacteria | Eukarya",
      "kingdom": "string",
      "phylum": "string",
      "class": "string",
      "order": "string",
      "family": "string",
      "genus": "string",
      "species": "string"
    },
    "strain_name": "string",
    "genetic_modifications": [
      {
        "type": "CRISPR | traditional | none",
        "target_gene": "string",
        "modification": "string",
        "purpose": "string"
      }
    ],
    "safety_classification": "GRAS | approved | experimental",
    "growth_characteristics": {
      "optimal_temperature_C": "number",
      "temperature_range_C": {
        "min": "number",
        "max": "number"
      },
      "optimal_pH": "number",
      "pH_range": {
        "min": "number",
        "max": "number"
      },
      "generation_time_hours": "number",
      "oxygen_requirement": "aerobic | anaerobic | facultative"
    }
  }
}
```

### 2.2 Fermentation Parameters

```json
{
  "fermentation": {
    "batch_id": "string (FK)",
    "start_time": "ISO8601 datetime",
    "end_time": "ISO8601 datetime",
    "duration_hours": "number",
    "reactor_type": "stirred_tank | airlift | bubble_column | fed_batch",
    "working_volume_L": "number",
    "substrate": {
      "primary_source": "methanol | methane | glucose | molasses | agricultural_waste | co2_h2",
      "concentration_g_L": "number",
      "feed_rate_g_h": "number (for fed-batch)",
      "total_consumed_kg": "number"
    },
    "process_parameters": {
      "temperature": {
        "setpoint_C": "number",
        "actual_C": "number",
        "variance_C": "number",
        "control_method": "PID | on_off | cascade"
      },
      "pH": {
        "setpoint": "number",
        "actual": "number",
        "control_agent_acid": "string",
        "control_agent_base": "string",
        "consumption_L": "number"
      },
      "dissolved_oxygen": {
        "setpoint_percent": "number",
        "actual_percent": "number",
        "air_flow_rate_vvm": "number",
        "oxygen_enrichment_percent": "number"
      },
      "agitation": {
        "speed_rpm": "number",
        "impeller_type": "rushton | pitched_blade | marine",
        "power_input_w_m3": "number"
      },
      "pressure": {
        "headspace_kPa": "number",
        "control_method": "string"
      }
    },
    "nutrient_feed": {
      "nitrogen_source": "ammonium_sulfate | ammonium_hydroxide | urea | nitrate",
      "nitrogen_g_L": "number",
      "phosphorus_source": "string",
      "phosphorus_mg_L": "number",
      "potassium_mg_L": "number",
      "magnesium_mg_L": "number",
      "trace_elements": [
        {
          "element": "Fe | Mn | Zn | Cu | Co | Mo | B",
          "concentration_mg_L": "number"
        }
      ],
      "vitamins": [
        {
          "name": "B1 | B12 | biotin | pantothenic_acid",
          "concentration_mg_L": "number"
        }
      ]
    }
  }
}
```

### 2.3 Real-time Sensor Data

```json
{
  "sensor_data": {
    "batch_id": "string (FK)",
    "timestamp": "ISO8601 datetime",
    "measurements": [
      {
        "sensor_id": "string",
        "parameter": "temperature | pH | DO | biomass | substrate | product",
        "value": "number",
        "unit": "string",
        "quality": "good | fair | poor",
        "calibration_date": "ISO8601 date"
      }
    ],
    "biomass": {
      "optical_density_600nm": "number",
      "dry_cell_weight_g_L": "number",
      "viable_cell_count_cfu_mL": "number"
    },
    "metabolites": {
      "substrate_remaining_g_L": "number",
      "protein_content_g_L": "number",
      "byproducts": [
        {
          "compound": "string",
          "concentration_g_L": "number"
        }
      ]
    }
  }
}
```

### 2.4 Quality Metrics

```json
{
  "quality_analysis": {
    "batch_id": "string (FK)",
    "analysis_date": "ISO8601 datetime",
    "laboratory": "string",
    "accreditation": "ISO17025 | other",
    "composition": {
      "protein_content": {
        "value_percent": "number (required)",
        "method": "Kjeldahl | Dumas | other",
        "nitrogen_to_protein_factor": "number"
      },
      "moisture_percent": "number",
      "ash_percent": "number",
      "fat_percent": "number",
      "fiber_percent": "number",
      "carbohydrate_percent": "number"
    },
    "amino_acid_profile": {
      "method": "HPLC | ion_exchange",
      "essential_amino_acids": {
        "histidine_g_100g": "number",
        "isoleucine_g_100g": "number",
        "leucine_g_100g": "number",
        "lysine_g_100g": "number",
        "methionine_g_100g": "number",
        "phenylalanine_g_100g": "number",
        "threonine_g_100g": "number",
        "tryptophan_g_100g": "number",
        "valine_g_100g": "number"
      },
      "amino_acid_score": "number (0-1)",
      "limiting_amino_acid": "string"
    },
    "digestibility": {
      "in_vitro_percent": "number",
      "in_vivo_percent": "number",
      "pdcaas": "number (Protein Digestibility Corrected Amino Acid Score)",
      "diaas": "number (Digestible Indispensable Amino Acid Score)"
    },
    "safety_testing": {
      "microbial_contamination": {
        "total_plate_count_cfu_g": "number",
        "yeast_mold_cfu_g": "number",
        "coliforms_cfu_g": "number",
        "salmonella": "detected | not_detected",
        "e_coli": "detected | not_detected"
      },
      "heavy_metals": {
        "lead_mg_kg": "number",
        "cadmium_mg_kg": "number",
        "mercury_mg_kg": "number",
        "arsenic_mg_kg": "number"
      },
      "mycotoxins": {
        "aflatoxin_b1_ug_kg": "number",
        "ochratoxin_a_ug_kg": "number",
        "deoxynivalenol_ug_kg": "number"
      },
      "allergens": ["string"],
      "nucleic_acid_content_percent": "number"
    },
    "nutritional_value": {
      "energy_kcal_100g": "number",
      "bioavailable_protein_g_100g": "number",
      "vitamins": [
        {
          "name": "string",
          "amount": "number",
          "unit": "mg | ug"
        }
      ],
      "minerals": [
        {
          "name": "string",
          "amount_mg_100g": "number"
        }
      ]
    },
    "overall_grade": "A+ | A | B | C | D | F",
    "certification_status": "organic | non_gmo | kosher | halal | vegan"
  }
}
```

### 2.5 Production Metrics

```json
{
  "production_metrics": {
    "batch_id": "string (FK)",
    "yield": {
      "biomass_yield_g_L": "number",
      "protein_yield_g_L": "number",
      "volumetric_productivity_g_L_h": "number",
      "specific_productivity_g_g_h": "number",
      "yield_coefficient_g_biomass_g_substrate": "number"
    },
    "efficiency": {
      "substrate_conversion_percent": "number",
      "carbon_conversion_percent": "number",
      "nitrogen_utilization_percent": "number"
    },
    "economics": {
      "substrate_cost_usd_kg": "number",
      "energy_cost_usd_kg": "number",
      "labor_cost_usd_kg": "number",
      "total_production_cost_usd_kg": "number",
      "estimated_market_price_usd_kg": "number",
      "profit_margin_percent": "number"
    },
    "environmental_impact": {
      "carbon_footprint_kg_co2eq_kg_protein": "number",
      "water_consumption_L_kg_protein": "number",
      "land_use_m2_kg_protein": "number",
      "energy_consumption_kwh_kg_protein": "number",
      "waste_generated_kg_kg_protein": "number",
      "circular_economy_score": "number (0-100)"
    }
  }
}
```

---

## 3. Data Validation Rules

### 3.1 Required Fields

- `batch_id`: Must be unique across all records
- `timestamp`: Must be valid ISO8601 format
- `strain.id`: Must reference valid strain database entry
- `quality_analysis.composition.protein_content.value_percent`: Must be present and >0

### 3.2 Value Ranges

```yaml
temperature_C: [15, 80]
pH: [2.0, 11.0]
dissolved_oxygen_percent: [0, 200]
protein_content_percent: [30, 95]
amino_acid_score: [0.0, 1.0]
```

### 3.3 Data Quality Indicators

- `timestamp_accuracy`: ±1 second
- `sensor_calibration_interval`: Maximum 30 days
- `quality_analysis_completion`: Within 48 hours of batch completion

---

## 4. File Formats

### 4.1 Supported Formats

- **JSON**: Primary format for API exchange
- **JSON-LD**: For semantic web integration
- **CSV**: For bulk data export/import
- **Parquet**: For big data analytics
- **Protocol Buffers**: For high-performance systems

### 4.2 Encoding

- Character encoding: UTF-8
- Date/time: ISO8601 with timezone
- Numbers: IEEE 754 double precision
- Coordinates: WGS84 decimal degrees

---

## 5. Metadata Standards

### 5.1 Dataset Metadata

```json
{
  "metadata": {
    "standard": "WIA-AGRI-034",
    "version": "1.0",
    "created": "ISO8601 datetime",
    "modified": "ISO8601 datetime",
    "creator": {
      "organization": "string",
      "contact": "string"
    },
    "license": "CC-BY-4.0 | proprietary | other",
    "language": "en | ko | other",
    "keywords": ["single cell protein", "fermentation", "alternative protein"],
    "related_standards": ["WIA-AGRI-001", "WIA-AGRI-002"]
  }
}
```

---

## 6. Compliance Requirements

### 6.1 Regulatory Compliance

- FDA GRAS notification requirements
- EU Novel Food Regulation (EU) 2015/2283
- Codex Alimentarius standards
- National food safety regulations

### 6.2 Data Privacy

- GDPR compliance for EU operations
- Data anonymization for research sharing
- Access control and audit logging

---

## 7. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-15 | Initial release |

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
