# WIA Livestock Tracking Data Format Standard
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
4. [Animal Identity Schema](#animal-identity-schema)
5. [RFID Tag Format](#rfid-tag-format)
6. [GPS Location Data](#gps-location-data)
7. [Health Records](#health-records)
8. [Movement Tracking](#movement-tracking)
9. [Blockchain Traceability](#blockchain-traceability)
10. [Validation Rules](#validation-rules)
11. [Examples](#examples)
12. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Livestock Tracking Data Format Standard defines a unified framework for animal identification, health monitoring, location tracking, and farm-to-table traceability across all livestock species.

**Core Objectives**:
- Standardize animal identification with RFID and GPS
- Enable real-time health monitoring and disease prevention
- Support farm-to-table traceability for food safety
- Facilitate blockchain-based proof of origin
- Ensure animal welfare compliance
- Enable cross-border livestock trade
- Support veterinary record integration

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Animal Identity | RFID tags, biometric data, unique identifiers |
| GPS Tracking | Real-time location, movement patterns, geofencing |
| Health Records | Medical history, vaccinations, treatments, growth |
| Breeding | Lineage, genetics, reproductive records |
| Feed Management | Diet, nutrition, feed conversion ratios |
| Slaughter Chain | Processing, meat grading, retail distribution |
| Traceability | Blockchain proofs, farm-to-table verification |
| Compliance | ISO 11784/11785, GS1, national regulations |

### 1.3 Design Principles

1. **Animal Welfare First**: Prioritize animal health and well-being
2. **Food Safety**: Enable complete traceability from farm to consumer
3. **Interoperability**: Compatible with existing livestock systems
4. **Privacy**: Protect farmer and commercial data
5. **Real-time**: Support live monitoring and alerts
6. **Scalability**: From small farms to industrial operations

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **RFID Tag** | Radio Frequency Identification device for animal ID |
| **EID** | Electronic Identification number (ISO 11784/11785) |
| **GPS Collar** | Wearable device with GPS/LoRaWAN tracking |
| **Herd Management** | Software system for livestock operations |
| **Traceability** | Ability to track animal history from birth to consumption |
| **Farm-to-Table** | Complete supply chain from producer to consumer |
| **LoRaWAN** | Long Range Wide Area Network for IoT devices |
| **GS1** | Global standards organization (barcodes, RFID) |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `animal_id` | Unique animal identifier | `"CATTLE-KR-2025-001234"` |
| `rfid_tag` | ISO 11784/11785 tag number | `"982-000123456789"` |
| `gps_coordinate` | WGS84 location | `{"lat": 37.5665, "lng": 126.9780}` |
| `species_code` | Animal species | `"CATTLE"`, `"PIG"`, `"SHEEP"` |
| `breed` | Animal breed | `"HOLSTEIN"`, `"HANWOO"`, `"ANGUS"` |
| `weight_kg` | Body weight in kilograms | `450.5` |
| `temperature_c` | Body temperature in Celsius | `38.5` |

### 2.3 Species Codes

| Code | Species | Common Types |
|------|---------|--------------|
| `CATTLE` | Bovine | Dairy cows, beef cattle (한우, 젖소) |
| `PIG` | Swine | Pork production (돼지) |
| `SHEEP` | Ovine | Lamb, wool production (양) |
| `GOAT` | Caprine | Meat, milk (염소) |
| `POULTRY` | Birds | Chicken, turkey, duck (닭, 오리) |
| `HORSE` | Equine | Racing, farming (말) |
| `FISH` | Aquaculture | Salmon, tuna, etc. (어류) |

---

## Base Structure

### 3.1 Root Object

```json
{
  "standard": "WIA-AGRI-009",
  "version": "1.0.0",
  "animal": { /* Animal object */ },
  "location_history": [ /* GPS tracking array */ ],
  "health_records": [ /* Medical records array */ ],
  "traceability": { /* Blockchain proof */ },
  "metadata": { /* Metadata object */ }
}
```

### 3.2 Animal Object

```json
{
  "animal_id": "CATTLE-KR-2025-001234",
  "rfid_tag": "982-000123456789",
  "species": "CATTLE",
  "breed": "HANWOO",
  "birth_date": "2023-03-15",
  "gender": "FEMALE",
  "current_weight_kg": 450.5,
  "farm": {
    "farm_id": "FARM-KR-001",
    "farm_name": "Green Valley Ranch",
    "owner": "Kim Farmer",
    "location": {
      "address": "123 Farm Road, Gangwon-do, Korea",
      "gps": { "lat": 37.566535, "lng": 126.977969 }
    }
  },
  "parent_ids": {
    "mother": "CATTLE-KR-2021-005678",
    "father": "CATTLE-KR-2020-003456"
  },
  "created_at": "2023-03-15T08:30:00Z",
  "updated_at": "2025-01-15T14:22:00Z"
}
```

**Field Requirements**:
- `animal_id` - REQUIRED - Unique identifier (country-year-sequence)
- `rfid_tag` - REQUIRED - ISO 11784/11785 compliant tag
- `species` - REQUIRED - Species code from Section 2.3
- `breed` - REQUIRED - Breed name
- `birth_date` - REQUIRED - Date of birth (YYYY-MM-DD)
- `gender` - REQUIRED - `MALE`, `FEMALE`, `CASTRATED`
- `current_weight_kg` - OPTIONAL - Latest weight measurement
- `farm` - REQUIRED - Farm ownership information
- `parent_ids` - OPTIONAL - Lineage tracking

---

## Animal Identity Schema

### 4.1 Complete Animal Profile

```json
{
  "animal_id": "CATTLE-KR-2025-001234",
  "rfid_tag": "982-000123456789",
  "species": "CATTLE",
  "breed": "HANWOO",
  "birth_date": "2023-03-15",
  "gender": "FEMALE",
  "color_markings": "Black with white face",
  "horn_status": "DEHORNED",
  "tattoo_id": "KR2025-1234",
  "microchip_id": "900123456789012",
  "dna_profile": {
    "sample_id": "DNA-KR-001234",
    "lab": "Korean Livestock Genomics Center",
    "collection_date": "2023-04-01",
    "genetic_markers": ["A1", "A2", "K-BB"]
  },
  "photos": [
    {
      "type": "FRONT_VIEW",
      "url": "https://cdn.farm.com/cattle/001234-front.jpg",
      "timestamp": "2025-01-15T10:00:00Z"
    },
    {
      "type": "RIGHT_SIDE",
      "url": "https://cdn.farm.com/cattle/001234-right.jpg",
      "timestamp": "2025-01-15T10:01:00Z"
    }
  ]
}
```

---

## RFID Tag Format

### 5.1 ISO 11784/11785 Standard

```json
{
  "rfid_tag": {
    "full_code": "982-000123456789",
    "country_code": "982",
    "manufacturer": "ALLFLEX",
    "tag_type": "FDX-B",
    "frequency_khz": 134.2,
    "implant_date": "2023-03-15",
    "implant_location": "LEFT_EAR",
    "battery_type": "PASSIVE",
    "expected_lifespan_years": 15
  }
}
```

**ISO 11784/11785 Breakdown**:
- Country Code: 3 digits (982 = Korea)
- Manufacturer Code: 3 digits
- Animal ID: 9 digits
- Check Digit: 1 digit (calculated)

### 5.2 EPC Gen2 (UHF RFID)

```json
{
  "epc_tag": {
    "epc_code": "E2001047100044000000001234",
    "protocol": "EPC_GEN2",
    "frequency_mhz": 915,
    "read_range_m": 10,
    "memory_bits": 512,
    "user_memory": {
      "farm_id": "FARM-KR-001",
      "batch_number": "2025-Q1-B01"
    }
  }
}
```

---

## GPS Location Data

### 6.1 Real-time Location

```json
{
  "location": {
    "lat": 37.566535,
    "lng": 126.977969,
    "altitude_m": 125.5,
    "accuracy_m": 2.5,
    "heading_deg": 180,
    "speed_kmh": 0.5,
    "timestamp": "2025-01-15T14:22:00Z",
    "device": {
      "device_id": "GPS-COLLAR-001",
      "battery_percent": 85,
      "signal_strength_dbm": -75
    }
  }
}
```

### 6.2 GPS Tracking Device

```json
{
  "gps_device": {
    "device_id": "GPS-COLLAR-001",
    "manufacturer": "DIGITANIMAL",
    "model": "LivestockTracker-2025",
    "firmware_version": "2.1.5",
    "network": {
      "type": "LORAWAN",
      "dev_eui": "0018B20000000001",
      "app_eui": "70B3D57ED0000001",
      "frequency_plan": "AS923-1",
      "data_rate": "SF7BW125"
    },
    "update_interval_min": 15,
    "geofence": {
      "enabled": true,
      "perimeter": [
        {"lat": 37.56, "lng": 126.97},
        {"lat": 37.57, "lng": 126.97},
        {"lat": 37.57, "lng": 126.98},
        {"lat": 37.56, "lng": 126.98}
      ],
      "alert_on_exit": true
    }
  }
}
```

---

## Health Records

### 7.1 Medical History

```json
{
  "health_records": [
    {
      "record_id": "HEALTH-001234-00001",
      "date": "2023-04-01",
      "type": "VACCINATION",
      "veterinarian": {
        "name": "Dr. Kim Veterinary",
        "license": "VET-KR-12345",
        "clinic": "Gangwon Animal Hospital"
      },
      "vaccination": {
        "vaccine_name": "Foot and Mouth Disease (FMD)",
        "manufacturer": "Boehringer Ingelheim",
        "batch_number": "FMD-2023-0401",
        "dose_ml": 2.0,
        "injection_site": "NECK_LEFT",
        "next_due_date": "2024-04-01"
      }
    },
    {
      "record_id": "HEALTH-001234-00002",
      "date": "2024-06-15",
      "type": "CHECKUP",
      "diagnosis": "Healthy",
      "weight_kg": 420.0,
      "temperature_c": 38.5,
      "heart_rate_bpm": 60,
      "body_condition_score": 3.5,
      "notes": "Animal in excellent health. No abnormalities detected."
    },
    {
      "record_id": "HEALTH-001234-00003",
      "date": "2024-11-10",
      "type": "TREATMENT",
      "condition": "Minor infection",
      "medication": {
        "name": "Penicillin G",
        "dosage": "10,000 IU/kg",
        "duration_days": 5,
        "withdrawal_period_days": 14
      },
      "outcome": "Full recovery"
    }
  ]
}
```

### 7.2 Growth Records

```json
{
  "growth_records": [
    {
      "date": "2023-03-15",
      "weight_kg": 50.0,
      "age_days": 0,
      "height_cm": 75
    },
    {
      "date": "2023-06-15",
      "weight_kg": 150.0,
      "age_days": 92,
      "height_cm": 95,
      "adg_kg": 1.09
    },
    {
      "date": "2024-01-15",
      "weight_kg": 350.0,
      "age_days": 306,
      "height_cm": 125,
      "adg_kg": 0.93
    },
    {
      "date": "2025-01-15",
      "weight_kg": 450.5,
      "age_days": 671,
      "height_cm": 140,
      "adg_kg": 0.28
    }
  ],
  "growth_metrics": {
    "birth_weight_kg": 50.0,
    "current_weight_kg": 450.5,
    "total_gain_kg": 400.5,
    "average_daily_gain_kg": 0.597,
    "feed_conversion_ratio": 6.5,
    "projected_market_weight_kg": 500.0,
    "projected_market_date": "2025-03-15"
  }
}
```

---

## Movement Tracking

### 8.1 Location History

```json
{
  "location_history": [
    {
      "timestamp": "2025-01-15T08:00:00Z",
      "lat": 37.566535,
      "lng": 126.977969,
      "activity": "GRAZING",
      "zone": "PASTURE_A"
    },
    {
      "timestamp": "2025-01-15T12:00:00Z",
      "lat": 37.567123,
      "lng": 126.978456,
      "activity": "DRINKING",
      "zone": "WATER_STATION_1"
    },
    {
      "timestamp": "2025-01-15T18:00:00Z",
      "lat": 37.566890,
      "lng": 126.977234,
      "activity": "RESTING",
      "zone": "BARN_SECTION_B"
    }
  ],
  "movement_analytics": {
    "total_distance_km": 2.5,
    "time_grazing_hours": 8.5,
    "time_resting_hours": 10.0,
    "time_moving_hours": 5.5,
    "zones_visited": ["PASTURE_A", "WATER_STATION_1", "BARN_SECTION_B"],
    "anomalies": []
  }
}
```

---

## Blockchain Traceability

### 9.1 Proof of Origin

```json
{
  "traceability": {
    "blockchain": {
      "network": "ETHEREUM",
      "contract_address": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
      "token_id": "CATTLE-KR-2025-001234",
      "hash": "0x1a2b3c4d5e6f7g8h9i0j1k2l3m4n5o6p7q8r9s0t",
      "timestamp": "2023-03-15T08:30:00Z"
    },
    "supply_chain": [
      {
        "stage": "BIRTH",
        "date": "2023-03-15",
        "location": "Green Valley Ranch",
        "hash": "0xabc123..."
      },
      {
        "stage": "VACCINATION",
        "date": "2023-04-01",
        "veterinarian": "Dr. Kim",
        "hash": "0xdef456..."
      },
      {
        "stage": "TRANSPORT",
        "date": "2025-01-20",
        "destination": "Seoul Slaughterhouse",
        "truck_id": "TRUCK-001",
        "hash": "0xghi789..."
      },
      {
        "stage": "SLAUGHTER",
        "date": "2025-01-21",
        "inspector": "Meat Inspector Lee",
        "carcass_weight_kg": 380,
        "meat_grade": "1++",
        "hash": "0xjkl012..."
      },
      {
        "stage": "RETAIL",
        "date": "2025-01-23",
        "store": "Lotte Mart Gangnam",
        "barcode": "8801234567890",
        "hash": "0xmno345..."
      }
    ]
  }
}
```

---

## Validation Rules

### 10.1 Animal ID Format

- Pattern: `{SPECIES}-{COUNTRY}-{YEAR}-{SEQUENCE}`
- Example: `CATTLE-KR-2025-001234`
- SPECIES: 3-10 uppercase letters
- COUNTRY: 2-letter ISO code
- YEAR: 4 digits
- SEQUENCE: 6+ digits with zero padding

### 10.2 RFID Tag Validation

- ISO 11784/11785: 15 digits (3+12)
- Country code must match ISO 3166-1 numeric
- Check digit validation required
- Format: `XXX-XXXXXXXXXXXX`

### 10.3 Weight Validation

```javascript
function validateWeight(weight_kg, age_days, species) {
  const limits = {
    CATTLE: { birth: [25, 60], adult: [400, 800] },
    PIG: { birth: [1, 2], adult: [80, 150] },
    SHEEP: { birth: [3, 6], adult: [40, 80] }
  };

  if (age_days < 7) {
    return weight_kg >= limits[species].birth[0] &&
           weight_kg <= limits[species].birth[1];
  } else {
    return weight_kg >= limits[species].birth[0] &&
           weight_kg <= limits[species].adult[1];
  }
}
```

---

## Examples

### 11.1 Complete Cattle Record

```json
{
  "standard": "WIA-AGRI-009",
  "version": "1.0.0",
  "animal": {
    "animal_id": "CATTLE-KR-2025-001234",
    "rfid_tag": "982-000123456789",
    "species": "CATTLE",
    "breed": "HANWOO",
    "birth_date": "2023-03-15",
    "gender": "FEMALE",
    "current_weight_kg": 450.5,
    "farm": {
      "farm_id": "FARM-KR-001",
      "farm_name": "Green Valley Ranch",
      "owner": "Kim Farmer",
      "certification": ["Organic", "Animal Welfare Approved"],
      "location": {
        "address": "123 Farm Road, Gangwon-do, Korea",
        "gps": { "lat": 37.566535, "lng": 126.977969 }
      }
    }
  },
  "location": {
    "lat": 37.566890,
    "lng": 126.977234,
    "timestamp": "2025-01-15T14:22:00Z",
    "device": {
      "device_id": "GPS-COLLAR-001",
      "battery_percent": 85
    }
  },
  "health_status": {
    "overall": "HEALTHY",
    "last_checkup": "2024-12-01",
    "temperature_c": 38.5,
    "vaccinations_current": true,
    "medications": []
  },
  "traceability": {
    "blockchain": {
      "network": "ETHEREUM",
      "hash": "0x1a2b3c4d5e6f7g8h9i0j1k2l3m4n5o6p7q8r9s0t"
    },
    "qr_code_url": "https://livestock.wia.com/verify/CATTLE-KR-2025-001234"
  },
  "metadata": {
    "created_at": "2023-03-15T08:30:00Z",
    "updated_at": "2025-01-15T14:22:00Z",
    "schema_version": "1.0.0"
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA-AGRI-009 Livestock Tracking Standard*
*© 2025 WIA - MIT License*
