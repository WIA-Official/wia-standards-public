# WIA-AGRI-013: Pest Detection Standard
## Phase 1: Data Format Specification

### 1.1 Overview

Phase 1 establishes the foundational data formats and structures for pest detection systems. This phase defines how pest and disease information is represented, stored, and exchanged between detection systems, monitoring networks, and agricultural management platforms.

**Duration**: 3-6 months
**Key Outcome**: Standardized data schemas for pest detection information

### 1.2 Core Data Structures

#### 1.2.1 Pest Detection Schema

```json
{
  "detection_id": "string (UUID)",
  "timestamp": "ISO 8601 datetime",
  "detection_method": "enum [visual_inspection, image_ai, trap_count, sensor_array, lab_analysis]",
  "location": {
    "field_id": "string",
    "coordinates": {
      "latitude": "number (degrees)",
      "longitude": "number (degrees)",
      "elevation": "number (meters)"
    },
    "area_ha": "number",
    "region": "string"
  },
  "pest_identification": {
    "pest_type": "enum [insect, fungal, bacterial, viral, nematode, weed]",
    "species_id": "string (scientific name)",
    "common_name": {
      "en": "string",
      "ko": "string",
      "local": "string"
    },
    "life_stage": "enum [egg, larva, nymph, adult, spore, vegetative]",
    "confidence_score": "number (0-1)"
  },
  "severity_assessment": {
    "level": "enum [low, medium, high, critical]",
    "infestation_percent": "number (0-100)",
    "population_density": "number (per m² or per plant)",
    "damage_score": "number (0-10)",
    "economic_threshold_exceeded": "boolean"
  },
  "crop_information": {
    "crop_type": "string",
    "variety": "string",
    "growth_stage": "string",
    "planting_date": "ISO 8601 date"
  },
  "detection_metadata": {
    "inspector_id": "string",
    "equipment_used": "string",
    "images_analyzed": "number",
    "samples_collected": "number"
  }
}
```

#### 1.2.2 Disease Symptom Catalog

```json
{
  "symptom_id": "string (UUID)",
  "disease_id": "string",
  "disease_name": {
    "scientific": "string",
    "common_en": "string",
    "common_ko": "string"
  },
  "pathogen": {
    "type": "enum [fungal, bacterial, viral, phytoplasma]",
    "species": "string",
    "strain": "string"
  },
  "symptoms": [
    {
      "location": "enum [leaf, stem, root, fruit, flower]",
      "appearance": "string (description)",
      "color_pattern": "string",
      "texture": "string",
      "progression_stage": "enum [early, intermediate, advanced]"
    }
  ],
  "diagnostic_markers": {
    "visual_cues": ["array of strings"],
    "microscopic_features": ["array of strings"],
    "molecular_markers": ["array of gene sequences"]
  },
  "environmental_triggers": {
    "temperature_range_c": {"min": "number", "max": "number"},
    "humidity_range_percent": {"min": "number", "max": "number"},
    "favorable_conditions": "string"
  }
}
```

#### 1.2.3 Pest Population Data

```json
{
  "population_id": "string (UUID)",
  "location_id": "string",
  "pest_species": "string",
  "monitoring_period": {
    "start_date": "ISO 8601 date",
    "end_date": "ISO 8601 date"
  },
  "population_metrics": {
    "average_density": "number (per unit area)",
    "peak_density": "number",
    "growth_rate": "number (per day)",
    "generation_time_days": "number"
  },
  "trap_data": [
    {
      "trap_id": "string",
      "trap_type": "enum [pheromone, light, sticky, pitfall]",
      "deployment_date": "ISO 8601 date",
      "readings": [
        {
          "date": "ISO 8601 date",
          "count": "number",
          "species_breakdown": {"species_id": "count"}
        }
      ]
    }
  ],
  "spatial_distribution": {
    "pattern": "enum [random, clustered, edge_effect, uniform]",
    "hotspots": [
      {
        "coordinates": {"lat": "number", "lon": "number"},
        "density": "number",
        "radius_m": "number"
      }
    ]
  }
}
```

#### 1.2.4 Image Recognition Data

```json
{
  "image_id": "string (UUID)",
  "capture_timestamp": "ISO 8601 datetime",
  "image_metadata": {
    "resolution": "string (e.g., 4096x3072)",
    "format": "enum [jpg, png, tiff, raw]",
    "camera_model": "string",
    "focal_length_mm": "number"
  },
  "capture_conditions": {
    "lighting": "enum [natural, artificial, mixed]",
    "weather": "string",
    "time_of_day": "string"
  },
  "ai_analysis": {
    "model_version": "string",
    "model_type": "enum [cnn, yolo, rcnn, transformer]",
    "processing_time_ms": "number",
    "detections": [
      {
        "class": "string (pest/disease name)",
        "confidence": "number (0-1)",
        "bounding_box": {
          "x": "number",
          "y": "number",
          "width": "number",
          "height": "number"
        },
        "severity_estimate": "enum [low, medium, high]"
      }
    ],
    "total_detections": "number",
    "average_confidence": "number"
  },
  "ground_truth": {
    "verified": "boolean",
    "actual_pest": "string",
    "verification_method": "string",
    "verified_by": "string"
  }
}
```

### 1.3 Environmental Data

#### 1.3.1 Weather Correlation Schema

```json
{
  "weather_data_id": "string (UUID)",
  "location_id": "string",
  "observation_period": {
    "start": "ISO 8601 datetime",
    "end": "ISO 8601 datetime"
  },
  "meteorological_data": {
    "temperature": {
      "average_c": "number",
      "min_c": "number",
      "max_c": "number",
      "daily_readings": [{"timestamp": "ISO 8601", "value": "number"}]
    },
    "humidity": {
      "average_percent": "number",
      "min_percent": "number",
      "max_percent": "number",
      "daily_readings": [{"timestamp": "ISO 8601", "value": "number"}]
    },
    "precipitation": {
      "total_mm": "number",
      "events": [
        {
          "date": "ISO 8601 date",
          "amount_mm": "number",
          "duration_hours": "number"
        }
      ]
    },
    "leaf_wetness": {
      "hours_per_day": "number",
      "total_hours": "number"
    },
    "wind": {
      "average_speed_ms": "number",
      "max_gust_ms": "number",
      "direction_degrees": "number"
    },
    "solar_radiation": {
      "total_mj_m2": "number",
      "average_w_m2": "number"
    }
  },
  "pest_weather_correlation": {
    "favorable_index": "number (0-100)",
    "development_rate_multiplier": "number",
    "outbreak_risk": "enum [low, moderate, high, critical]"
  }
}
```

#### 1.3.2 Soil and Plant Health Data

```json
{
  "plant_health_id": "string (UUID)",
  "field_id": "string",
  "assessment_date": "ISO 8601 date",
  "plant_stress_indicators": {
    "chlorophyll_index": "number (0-1)",
    "ndvi": "number (-1 to 1)",
    "leaf_area_index": "number",
    "stress_level": "enum [none, mild, moderate, severe]"
  },
  "soil_conditions": {
    "moisture_percent": "number",
    "temperature_c": "number",
    "ph": "number",
    "organic_matter_percent": "number"
  },
  "pest_susceptibility": {
    "stress_induced_vulnerability": "boolean",
    "nutrient_deficiency_related": "boolean",
    "water_stress_factor": "number (0-1)"
  }
}
```

### 1.4 Treatment History

#### 1.4.1 Pesticide Application Record

```json
{
  "application_id": "string (UUID)",
  "field_id": "string",
  "application_date": "ISO 8601 datetime",
  "target_pest": "string",
  "pesticide": {
    "product_name": "string",
    "active_ingredient": "string",
    "concentration": "string",
    "registration_number": "string",
    "safety_class": "enum [I, II, III, IV]"
  },
  "application_details": {
    "method": "enum [spray, dust, granular, seed_treatment, soil_drench]",
    "rate_per_ha": "string",
    "total_volume_applied": "number (liters or kg)",
    "area_treated_ha": "number",
    "equipment_used": "string"
  },
  "conditions": {
    "temperature_c": "number",
    "wind_speed_ms": "number",
    "humidity_percent": "number",
    "time_of_day": "string"
  },
  "efficacy": {
    "pre_treatment_density": "number",
    "post_treatment_density": "number",
    "control_percentage": "number",
    "residual_period_days": "number"
  },
  "safety_compliance": {
    "phi_days": "number (pre-harvest interval)",
    "rei_hours": "number (restricted entry interval)",
    "applicator_id": "string",
    "ppe_used": ["array of equipment"]
  }
}
```

### 1.5 Data Validation Rules

#### 1.5.1 Required Fields
- All detection records MUST include: detection_id, timestamp, location, pest_identification
- Confidence scores MUST be between 0 and 1
- Coordinates MUST use WGS84 datum
- Timestamps MUST be in ISO 8601 format with timezone

#### 1.5.2 Data Quality Standards
- Image resolution MUST be minimum 1024x768 for AI analysis
- Population counts MUST include sampling methodology
- Weather data MUST include data source and accuracy rating
- All scientific names MUST follow current taxonomic nomenclature

#### 1.5.3 Data Retention
- Detection records: Minimum 5 years
- Image data: Minimum 3 years
- Treatment history: Minimum 10 years (regulatory requirement)
- Weather data: Minimum 3 years

### 1.6 Interoperability Requirements

#### 1.6.1 Export Formats
- JSON (primary format)
- CSV (for tabular data)
- GeoJSON (for spatial data)
- XML (for legacy system compatibility)

#### 1.6.2 API Payload Limits
- Single detection record: Maximum 5 MB
- Batch upload: Maximum 100 records or 50 MB
- Image upload: Maximum 20 MB per image
- Response pagination: 50 records per page

### 1.7 Korean Localization (한국어 현지화)

#### 1.7.1 Korean Pest Names
```json
{
  "pest_mapping": {
    "rice_planthopper": {
      "scientific": "Nilaparvata lugens",
      "en": "Rice Planthopper",
      "ko": "벼멸구"
    },
    "rice_stem_borer": {
      "scientific": "Chilo suppressalis",
      "en": "Rice Stem Borer",
      "ko": "이화명나방"
    },
    "powdery_mildew": {
      "scientific": "Blumeria graminis",
      "en": "Powdery Mildew",
      "ko": "흰가루병"
    },
    "anthracnose": {
      "scientific": "Colletotrichum spp.",
      "en": "Anthracnose",
      "ko": "탄저병"
    }
  }
}
```

#### 1.7.2 Korean Agricultural Terms
- 예찰 (Ye-chal): Pest monitoring/surveillance
- 방제 (Bang-je): Pest control
- 농약 (Nong-yak): Pesticide
- 병해충 (Byeong-hae-chung): Pests and diseases
- 종합적 병해충 관리 (IPM): Integrated Pest Management

### 1.8 Implementation Timeline

**Month 1-2**: Core data structure definition
**Month 3-4**: Validation rule implementation
**Month 5**: Localization and testing
**Month 6**: Documentation and reference implementation

### 1.9 Success Criteria

- ✓ 95%+ data validation pass rate
- ✓ Support for 50+ common agricultural pests
- ✓ Multilingual support (EN, KO, minimum)
- ✓ Interoperability with existing Korean RDA systems
- ✓ Processing time < 100ms for single record validation
