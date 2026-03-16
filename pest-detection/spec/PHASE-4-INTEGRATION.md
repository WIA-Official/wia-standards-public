# WIA-AGRI-013: Pest Detection Standard
## Phase 4: Integration Specification

### 4.1 Overview

Phase 4 defines the integration points with external systems including weather services, pesticide suppliers, extension services, export certification systems, and agricultural research networks. This phase enables comprehensive pest management through data interoperability and service coordination.

**Duration**: 6-8 months
**Key Outcome**: Fully integrated pest management ecosystem

### 4.2 Weather Service Integration

#### 4.2.1 Supported Weather APIs

**Primary Providers:**
- Korea Meteorological Administration (KMA / 기상청)
- OpenWeatherMap
- WeatherAPI
- NOAA Global Forecast System
- National Agrometeorological System

#### 4.2.2 Weather Data Schema

```json
{
  "weather_integration_id": "string (UUID)",
  "location": {
    "coordinates": {
      "latitude": "number",
      "longitude": "number"
    },
    "region": "string",
    "station_id": "string"
  },
  "current_conditions": {
    "timestamp": "ISO 8601 datetime",
    "temperature_c": "number",
    "humidity_percent": "number",
    "precipitation_mm_1h": "number",
    "wind_speed_ms": "number",
    "wind_direction_deg": "number",
    "pressure_hpa": "number",
    "solar_radiation_w_m2": "number",
    "leaf_wetness_hours": "number",
    "conditions": "string"
  },
  "forecast_7day": [
    {
      "date": "ISO 8601 date",
      "temperature_high_c": "number",
      "temperature_low_c": "number",
      "humidity_avg_percent": "number",
      "precipitation_probability_percent": "number",
      "precipitation_total_mm": "number",
      "wind_speed_avg_ms": "number",
      "conditions": "string"
    }
  ],
  "pest_risk_indices": {
    "pest_development_index": "number (0-100)",
    "disease_pressure_index": "number (0-100)",
    "favorable_days_ahead": "number",
    "risk_trend": "enum [decreasing, stable, increasing, critical]"
  },
  "historical_data": {
    "30_day_avg_temp_c": "number",
    "30_day_total_precip_mm": "number",
    "growing_degree_days": "number",
    "deviation_from_normal": "number"
  }
}
```

#### 4.2.3 KMA Integration (기상청 연동)

**Endpoint**: `https://api.kma.go.kr/pest-weather/v1`

**Authentication**: API key required

Request:
```http
GET /weather/forecast?location=35.8219,127.1489&days=7
Authorization: Bearer KMA_API_KEY
```

Response:
```json
{
  "location": {
    "region": "전라북도 김제시",
    "coordinates": {"lat": 35.8219, "lon": 127.1489}
  },
  "current": {
    "기온": 28.5,
    "습도": 75,
    "강수량_1시간": 0,
    "풍속": 2.3
  },
  "예보_7일": [
    {
      "날짜": "2025-06-16",
      "최고기온": 31,
      "최저기온": 22,
      "강수확률": 30,
      "예상강수량": 5
    }
  ],
  "병해충_위험도": {
    "벼멸구_발생위험": "높음",
    "흰가루병_발생위험": "보통",
    "전체위험지수": 72
  }
}
```

#### 4.2.4 Weather-Pest Correlation Models

**Rice Planthopper (벼멸구) Model:**
```python
def calculate_planthopper_risk(temp, humidity, rainfall, wind_speed):
    """
    Calculate rice planthopper outbreak risk based on weather

    Favorable conditions:
    - Temperature: 25-30°C (optimal: 28°C)
    - Humidity: >70%
    - Low rainfall (not heavy downpours)
    - Low wind speed (<3 m/s)
    """
    risk_score = 0

    # Temperature factor
    if 25 <= temp <= 30:
        risk_score += 30
        if temp == 28:
            risk_score += 10
    elif 20 <= temp <= 35:
        risk_score += 15

    # Humidity factor
    if humidity > 80:
        risk_score += 25
    elif humidity > 70:
        risk_score += 15

    # Rainfall factor (moderate is good, heavy is bad)
    if 0 < rainfall < 10:
        risk_score += 15
    elif rainfall > 50:
        risk_score -= 10

    # Wind factor (migration happens with moderate wind)
    if 1 < wind_speed < 5:
        risk_score += 15
    elif wind_speed > 10:
        risk_score -= 20

    return min(max(risk_score, 0), 100)
```

**Powdery Mildew (흰가루병) Model:**
```python
def calculate_powdery_mildew_risk(temp, humidity, leaf_wetness_hours):
    """
    Calculate powdery mildew risk

    Favorable conditions:
    - Temperature: 15-25°C
    - Humidity: >60% but not continuous wetness
    - Leaf wetness: 4-8 hours (too much is unfavorable)
    """
    risk_score = 0

    if 15 <= temp <= 25:
        risk_score += 35

    if 60 <= humidity <= 85:
        risk_score += 30

    if 4 <= leaf_wetness_hours <= 8:
        risk_score += 25
    elif leaf_wetness_hours > 12:
        risk_score -= 15

    return min(max(risk_score, 0), 100)
```

### 4.3 Pesticide Supplier Integration

#### 4.3.1 Supplier Database Schema

```json
{
  "supplier_id": "string (UUID)",
  "company_name": "string",
  "registration": {
    "business_number": "string",
    "pesticide_license": "string",
    "certifications": ["array of certification types"]
  },
  "contact": {
    "phone": "string",
    "email": "string",
    "website": "string",
    "address": "string"
  },
  "service_areas": [
    {
      "region": "string",
      "delivery_time_hours": "number",
      "minimum_order": "number (KRW)"
    }
  ],
  "product_catalog": [
    {
      "product_id": "string",
      "product_name": "string",
      "active_ingredient": "string",
      "concentration": "string",
      "registration_number": "string",
      "target_pests": ["array of pest IDs"],
      "approved_crops": ["array of crop types"],
      "application_rate": "string",
      "phi_days": "number",
      "rei_hours": "number",
      "price_per_unit": "number (KRW)",
      "unit_size": "string (e.g., 1L, 500g)",
      "in_stock": "boolean",
      "delivery_available": "boolean"
    }
  ]
}
```

#### 4.3.2 Pesticide Recommendation API

**POST /pesticides/recommend**

Request:
```json
{
  "pest_id": "Nilaparvata_lugens",
  "crop_type": "rice",
  "severity": "high",
  "area_ha": 10,
  "location": {
    "region": "Jeonbuk",
    "coordinates": {"lat": 35.8219, "lon": 127.1489}
  },
  "preferences": {
    "organic": false,
    "budget_per_ha": 50000,
    "delivery_urgency": "24hours"
  }
}
```

Response:
```json
{
  "recommendations": [
    {
      "rank": 1,
      "product": {
        "id": "prod-abc123",
        "name": "PlantHopper Guard 20SC",
        "active_ingredient": "Imidacloprid 20%",
        "registration": "농약-2021-0534",
        "manufacturer": "AgChem Korea"
      },
      "application": {
        "rate_per_ha": "1.0 L/ha",
        "total_volume_needed": "10 L",
        "water_volume": "100-150 L/ha",
        "number_of_applications": 1,
        "phi_days": 14
      },
      "cost": {
        "price_per_liter": 32000,
        "total_cost": 320000,
        "cost_per_ha": 32000
      },
      "availability": {
        "in_stock": true,
        "nearest_supplier": {
          "name": "Jeonbuk Agricultural Coop",
          "distance_km": 15,
          "delivery_time": "Same day",
          "phone": "063-545-1234"
        }
      },
      "efficacy": {
        "effectiveness_rating": 0.92,
        "control_duration_days": 14,
        "resistance_risk": "medium"
      },
      "safety": {
        "safety_class": "II (Moderately Hazardous)",
        "rei_hours": 12,
        "environmental_impact": "Moderate - toxic to aquatic organisms",
        "ppe_required": ["Gloves", "Mask", "Goggles"]
      }
    },
    {
      "rank": 2,
      "product": {
        "id": "prod-def456",
        "name": "Bio-Control Neem Plus",
        "active_ingredient": "Neem Oil 70%",
        "registration": "농약-2020-0891",
        "manufacturer": "Organic Solutions Korea"
      },
      "application": {
        "rate_per_ha": "2.5 L/ha",
        "total_volume_needed": "25 L",
        "water_volume": "100 L/ha",
        "number_of_applications": 2,
        "phi_days": 0
      },
      "cost": {
        "price_per_liter": 18000,
        "total_cost": 450000,
        "cost_per_ha": 45000
      },
      "availability": {
        "in_stock": true,
        "nearest_supplier": {
          "name": "EcoFarm Supply",
          "distance_km": 28,
          "delivery_time": "Next day",
          "phone": "063-547-5678"
        }
      },
      "efficacy": {
        "effectiveness_rating": 0.75,
        "control_duration_days": 7,
        "resistance_risk": "low"
      },
      "safety": {
        "safety_class": "IV (Slightly Hazardous)",
        "rei_hours": 4,
        "environmental_impact": "Low - safe for beneficial insects",
        "ppe_required": ["Gloves"]
      }
    }
  ],
  "ipm_context": {
    "economic_threshold": 20,
    "current_density": 45,
    "treatment_necessary": true,
    "urgency": "Apply within 48 hours",
    "resistance_management": "Rotate between Group 4A (Imidacloprid) and biopesticides"
  }
}
```

#### 4.3.3 Automated Ordering System

**POST /orders/create**

Request:
```json
{
  "supplier_id": "supplier-xyz789",
  "products": [
    {
      "product_id": "prod-abc123",
      "quantity": 10,
      "unit": "liter"
    }
  ],
  "delivery": {
    "address": "김제시 농협 123-45",
    "contact_phone": "010-1234-5678",
    "urgency": "standard",
    "delivery_instructions": "Leave at agricultural coop"
  },
  "payment": {
    "method": "card",
    "card_token": "encrypted_token"
  }
}
```

Response:
```json
{
  "order_id": "order-20250615-001",
  "status": "confirmed",
  "total_amount": 320000,
  "estimated_delivery": "2025-06-16T14:00:00+09:00",
  "tracking_number": "TRACK-ABC123",
  "invoice_url": "https://supplier.com/invoice/001"
}
```

### 4.4 Extension Service Integration

#### 4.4.1 RDA Integration (농촌진흥청 연동)

**System**: National Crop Pest Management System (병해충예찰정보시스템)

**Base URL**: `https://ncpms.rda.go.kr/api/v1`

**Data Exchange**:

1. **Submit Detection Report**
```json
POST /pest-reports

{
  "보고일자": "2025-06-15",
  "보고기관": "김제시 농업기술센터",
  "작물명": "벼",
  "병해충명": "벼멸구",
  "발생지역": {
    "시도": "전라북도",
    "시군구": "김제시",
    "읍면동": "백산면"
  },
  "발생면적_ha": 25.5,
  "발생정도": "다발생",
  "방제여부": "미실시",
  "비고": "AI 자동감지 시스템을 통한 조기 발견"
}
```

2. **Retrieve Regional Pest Data**
```json
GET /pest-data?region=전북&crop=벼&period=30days

Response:
{
  "지역": "전라북도",
  "조회기간": {
    "시작일": "2025-05-16",
    "종료일": "2025-06-15"
  },
  "병해충발생현황": [
    {
      "병해충명": "벼멸구",
      "발생지역수": 12,
      "총발생면적_ha": 847,
      "발생정도": "다발생",
      "전년대비": "+35%"
    }
  ],
  "방제권고사항": "즉시 약제 방제 실시 권장"
}
```

#### 4.4.2 Local Extension Office Coordination

**POST /extension/coordinate**

Request:
```json
{
  "alert_id": "WIA-PEST-2025-0615-001",
  "extension_office": "Jeonbuk Agricultural Technology Center",
  "coordination_request": {
    "type": "technical_support",
    "urgency": "high",
    "farmers_affected": 2840,
    "support_needed": [
      "Expert field assessment",
      "Treatment training session",
      "Equipment rental coordination",
      "Financial assistance information"
    ]
  }
}
```

Response:
```json
{
  "coordination_id": "coord-001",
  "status": "accepted",
  "assigned_experts": [
    {
      "name": "Dr. Kim Min-su (김민수 박사)",
      "specialty": "Rice Pest Management",
      "phone": "063-238-1234",
      "availability": "Available for field visit on 2025-06-16"
    }
  ],
  "scheduled_activities": [
    {
      "activity": "Farmer training session",
      "date": "2025-06-17",
      "location": "Gimje Agricultural Coop",
      "expected_participants": 150
    }
  ],
  "resources_allocated": {
    "spray_equipment": "10 units available for rental",
    "demonstration_plots": "3 plots for treatment comparison",
    "funding_support": "Disaster relief program applicable"
  }
}
```

### 4.5 Export Certification Integration

#### 4.5.1 Phytosanitary Certificate System

**POST /certification/phytosanitary**

Request:
```json
{
  "exporter": {
    "name": "Green Valley Organic Farm",
    "registration": "EXPORT-2024-001",
    "address": "Jeonbuk, Korea"
  },
  "product": {
    "crop": "organic_rice",
    "variety": "Koshihikari",
    "volume_kg": 50000,
    "harvest_date": "2025-09-15",
    "processing": "dried and polished"
  },
  "destination": {
    "country": "Japan",
    "port": "Fukuoka",
    "importer": "Tokyo Rice Trading Co."
  },
  "pest_status": {
    "pest_free": true,
    "monitoring_period": {
      "start": "2025-04-01",
      "end": "2025-09-15"
    },
    "detections": [],
    "treatments": [
      {
        "date": "2025-05-10",
        "pest": "Nilaparvata_lugens",
        "treatment": "Neem oil (organic)",
        "phi_days": 0,
        "residue_test": "ND (not detected)"
      }
    ]
  },
  "inspections": [
    {
      "date": "2025-09-20",
      "inspector": "NPQS Inspector Kim",
      "result": "No quarantine pests detected",
      "lab_test_id": "LAB-2025-0920-001"
    }
  ]
}
```

Response:
```json
{
  "certificate_id": "PHYTO-KR-2025-0920-001",
  "certificate_number": "PC-001234567",
  "issue_date": "2025-09-20",
  "valid_until": "2025-10-20",
  "status": "approved",
  "certification_body": "National Plant Quarantine Service (식물검역소)",
  "pest_declaration": {
    "free_from_quarantine_pests": true,
    "treatment_applied": true,
    "residue_compliant": true,
    "export_approved": true
  },
  "qr_code_url": "https://npqs.go.kr/verify/PHYTO-001234567",
  "blockchain_hash": "0x7f8a9b0c...",
  "digital_signature": "base64_encoded_signature"
}
```

#### 4.5.2 Pest-Free Certification for Domestic Markets

**POST /certification/pest-free**

Request:
```json
{
  "producer": "Green Valley Organic Farm",
  "crop": "organic_rice",
  "certification_type": "WIA-AGRI-013-PestFree",
  "production_data": {
    "area_ha": 45,
    "production_kg": 200000,
    "production_year": 2025
  },
  "monitoring_records": {
    "frequency": "weekly",
    "total_inspections": 24,
    "pest_detections": 2,
    "treatments_applied": 1,
    "pest_free_period_days": 120
  },
  "residue_testing": {
    "lab": "Korea Testing Laboratory",
    "test_date": "2025-09-18",
    "pesticide_residues": [
      {
        "compound": "Imidacloprid",
        "level_ppm": 0.003,
        "mrl_ppm": 0.1,
        "compliant": true
      }
    ],
    "heavy_metals": "All within limits",
    "mycotoxins": "Not detected"
  }
}
```

Response:
```json
{
  "certificate_id": "WIA-AGRI-013-PF-20250920-001",
  "certification_level": "Premium Pest-Free",
  "grade": "A+",
  "issue_date": "2025-09-20",
  "valid_until": "2026-03-20",
  "qr_code_data": {
    "verification_url": "https://wia.org/verify/pest-free/001",
    "producer": "Green Valley Organic Farm",
    "crop": "Organic Rice",
    "harvest_date": "2025-09-15",
    "pest_free": true,
    "organic": true,
    "residue_compliant": true
  },
  "verifiable_credential": {
    "@context": "https://www.w3.org/2018/credentials/v1",
    "type": ["VerifiableCredential", "PestFreeCertificate"],
    "issuer": "did:wia:agricultural-authority",
    "credentialSubject": {
      "id": "did:wia:producer:green-valley-organic",
      "pestFree": true,
      "certificationStandard": "WIA-AGRI-013"
    }
  }
}
```

### 4.6 Research Network Integration

#### 4.6.1 Data Sharing with Agricultural Research Institutions

**POST /research/share-data**

Share anonymized pest detection data with research institutions:

```json
{
  "data_submission_id": "research-2025-Q2",
  "submitting_organization": "Jeonbuk Agricultural Technology Center",
  "data_period": {
    "start": "2025-04-01",
    "end": "2025-06-30"
  },
  "data_summary": {
    "total_detections": 2456,
    "unique_pests": 15,
    "geographic_coverage_km2": 4500,
    "images_included": 12847
  },
  "research_consent": {
    "anonymized": true,
    "academic_use_only": true,
    "commercial_use": false
  },
  "data_format": "WIA-AGRI-013-v1.0-research-export",
  "download_url": "https://secure.wia.org/research-data/2025Q2.zip",
  "encryption": "AES-256",
  "access_key": "encrypted_access_key"
}
```

#### 4.6.2 AI Model Training Collaboration

**POST /ai/contribute-training-data**

Contribute verified detection data for AI model improvement:

```json
{
  "contribution_id": "ai-train-2025-06",
  "dataset": {
    "images": 5000,
    "verified_labels": 5000,
    "pest_species_covered": 25,
    "annotation_quality": 0.98
  },
  "licensing": {
    "license_type": "CC-BY-4.0",
    "attribution": "Jeonbuk Agricultural Technology Center",
    "commercial_use_allowed": true
  },
  "quality_metrics": {
    "ground_truth_verified": true,
    "expert_verification_rate": 1.0,
    "inter_annotator_agreement": 0.95
  }
}
```

### 4.7 Blockchain Integration for Traceability

#### 4.7.1 Pest Treatment Record on Blockchain

**POST /blockchain/record-treatment**

Request:
```json
{
  "field_id": "field-12345",
  "treatment_date": "2025-06-16",
  "pest_detected": "Nilaparvata_lugens",
  "treatment_applied": {
    "product": "Imidacloprid 20% SC",
    "batch_number": "BATCH-2025-001",
    "application_rate": "1.0 L/ha",
    "area_treated_ha": 10
  },
  "certifications": ["WIA-AGRI-013", "GAP", "Organic"]
}
```

Response:
```json
{
  "blockchain_record_id": "bc-rec-20250616-001",
  "transaction_hash": "0x7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2c",
  "block_number": 12847562,
  "timestamp": "2025-06-16T14:30:00Z",
  "network": "WIA-AgriChain",
  "immutable": true,
  "verification_url": "https://explorer.wia-agrichain.org/tx/0x7f8a9b..."
}
```

### 4.8 IoT Device Integration

#### 4.8.1 Smart Trap Integration

**POST /iot/smart-trap/register**

Register IoT pest monitoring trap:

```json
{
  "device_id": "trap-iot-001",
  "device_type": "smart_pheromone_trap",
  "location": {
    "field_id": "field-12345",
    "coordinates": {"lat": 35.8219, "lon": 127.1489},
    "installation_height_cm": 50
  },
  "target_pest": "Nilaparvata_lugens",
  "sensors": {
    "camera": {
      "resolution": "4K",
      "frame_rate": 1,
      "night_vision": true
    },
    "environmental": {
      "temperature": true,
      "humidity": true,
      "light_level": true
    }
  },
  "connectivity": {
    "protocol": "LoRaWAN",
    "update_frequency_minutes": 60,
    "battery_powered": true
  }
}
```

**Automatic Data Upload from IoT Trap:**

```json
POST /iot/smart-trap/data

{
  "device_id": "trap-iot-001",
  "timestamp": "2025-06-16T14:00:00Z",
  "pest_count": 47,
  "image_analysis": {
    "total_insects_detected": 52,
    "target_pest_count": 47,
    "non_target_count": 5,
    "ai_confidence": 0.94
  },
  "environmental": {
    "temperature_c": 28.5,
    "humidity_percent": 76,
    "light_level_lux": 45000
  },
  "device_status": {
    "battery_percent": 78,
    "signal_strength_dbm": -85,
    "pheromone_remaining_percent": 65
  }
}
```

### 4.9 Integration Testing and Certification

#### 4.9.1 Integration Compliance Testing

Third-party systems seeking WIA-AGRI-013 certification must pass:

1. **API Compatibility Test**: 100% endpoint compliance
2. **Data Format Validation**: All required fields present
3. **Authentication/Security**: OAuth 2.0 / JWT support
4. **Performance**: Response time < 2 seconds (95th percentile)
5. **Reliability**: 99.5% uptime over 30-day test period
6. **Interoperability**: Successful integration with 3+ reference systems

#### 4.9.2 Integration Certification Levels

- **Bronze**: Basic API integration (read-only)
- **Silver**: Full API integration (read-write)
- **Gold**: Real-time streaming + IoT integration
- **Platinum**: Full ecosystem integration (all systems connected)

### 4.10 Implementation Timeline

**Month 1-2**: Weather API integration
**Month 3-4**: Pesticide supplier system integration
**Month 5-6**: Extension service coordination
**Month 7**: Export certification systems
**Month 8**: Research network and blockchain integration

### 4.11 Success Criteria

- ✓ Weather data updated hourly from 2+ sources
- ✓ Pesticide recommendations delivered in < 5 seconds
- ✓ Extension office response within 4 hours (business hours)
- ✓ Phytosanitary certificate generation in < 24 hours
- ✓ 99.9% data accuracy in blockchain records
- ✓ IoT device data ingestion rate > 10,000 devices
