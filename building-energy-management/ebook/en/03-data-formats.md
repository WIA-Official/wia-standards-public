# Chapter 3: Data Formats and Schemas (Phase 1)

## 3.1 Overview of WIA-BEMS Data Standards

### 3.1.1 Design Philosophy

Phase 1 of WIA-BEMS establishes the foundational data formats that enable interoperability across the building energy ecosystem. These standardized schemas serve as the common language for devices, systems, and applications from different vendors.

**Core Design Principles:**

```typescript
interface DataFormatPrinciples {
  universality: {
    description: "Schemas work for any building type and size";
    implementation: "Flexible structures with optional fields";
  };

  precision: {
    description: "Unambiguous semantics and units";
    implementation: "SI units primary, explicit unit fields";
  };

  quality: {
    description: "Built-in data quality indicators";
    implementation: "Quality objects with accuracy and confidence";
  };

  extensibility: {
    description: "Future-proof with extension mechanisms";
    implementation: "Metadata fields, custom properties";
  };

  performance: {
    description: "Efficient for real-time processing";
    implementation: "Compact structures, minimal nesting";
  };

  compatibility: {
    description: "Works with existing systems";
    implementation: "Haystack-compatible naming, BACnet mappable";
  };
}
```

### 3.1.2 Schema Organization

**Schema Categories:**

```
WIA-BEMS Schema Hierarchy:
├── Core Schemas
│   ├── building.json           # Building identification
│   ├── location.json           # Spatial hierarchy
│   ├── device.json             # Device registry
│   └── measurement.json        # Base measurement type
│
├── Energy Schemas
│   ├── energy-consumption.json # Energy usage records
│   ├── power-demand.json       # Real-time demand
│   ├── energy-generation.json  # On-site generation
│   └── energy-storage.json     # Battery/thermal storage
│
├── Environmental Schemas
│   ├── temperature.json        # Temperature measurements
│   ├── humidity.json           # Humidity measurements
│   ├── air-quality.json        # IAQ parameters
│   └── lighting.json           # Illumination levels
│
├── Occupancy Schemas
│   ├── occupancy.json          # Occupancy detection
│   ├── space-utilization.json  # Space usage metrics
│   └── comfort-feedback.json   # Occupant feedback
│
├── Equipment Schemas
│   ├── hvac-equipment.json     # HVAC systems
│   ├── lighting-equipment.json # Lighting systems
│   ├── electrical-equipment.json # Electrical distribution
│   └── renewables.json         # Solar, wind, etc.
│
└── Control Schemas
    ├── setpoint.json           # Control setpoints
    ├── schedule.json           # Operating schedules
    ├── command.json            # Control commands
    └── alarm.json              # Alerts and alarms
```

## 3.2 Core Schema Definitions

### 3.2.1 Building Schema

**Schema URI:** `https://wia.org/bems/v1/schemas/building.json`

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/bems/v1/schemas/building.json",
  "title": "WIA-BEMS Building",
  "description": "Building identification and metadata",
  "type": "object",
  "required": ["building_id", "name", "location", "type", "area"],

  "properties": {
    "building_id": {
      "type": "string",
      "pattern": "^BLDG-[A-Z0-9]{6,12}$",
      "description": "Unique building identifier"
    },

    "name": {
      "type": "string",
      "minLength": 1,
      "maxLength": 200,
      "description": "Human-readable building name"
    },

    "location": {
      "type": "object",
      "required": ["address", "coordinates"],
      "properties": {
        "address": {
          "type": "object",
          "required": ["country_code"],
          "properties": {
            "street": { "type": "string" },
            "city": { "type": "string" },
            "state_province": { "type": "string" },
            "postal_code": { "type": "string" },
            "country_code": {
              "type": "string",
              "pattern": "^[A-Z]{2}$",
              "description": "ISO 3166-1 alpha-2"
            }
          }
        },
        "coordinates": {
          "type": "object",
          "required": ["latitude", "longitude"],
          "properties": {
            "latitude": {
              "type": "number",
              "minimum": -90,
              "maximum": 90
            },
            "longitude": {
              "type": "number",
              "minimum": -180,
              "maximum": 180
            },
            "elevation_m": { "type": "number" }
          }
        },
        "timezone": {
          "type": "string",
          "description": "IANA timezone (e.g., America/New_York)"
        }
      }
    },

    "type": {
      "type": "string",
      "enum": [
        "office",
        "retail",
        "healthcare",
        "education",
        "hospitality",
        "industrial",
        "warehouse",
        "residential_multifamily",
        "data_center",
        "mixed_use",
        "other"
      ]
    },

    "area": {
      "type": "object",
      "required": ["gross_sqm"],
      "properties": {
        "gross_sqm": {
          "type": "number",
          "minimum": 0,
          "description": "Gross floor area in square meters"
        },
        "net_sqm": {
          "type": "number",
          "minimum": 0,
          "description": "Net usable floor area"
        },
        "conditioned_sqm": {
          "type": "number",
          "minimum": 0,
          "description": "Conditioned floor area"
        }
      }
    },

    "floors": {
      "type": "object",
      "properties": {
        "above_ground": { "type": "integer", "minimum": 1 },
        "below_ground": { "type": "integer", "minimum": 0 }
      }
    },

    "year_built": {
      "type": "integer",
      "minimum": 1800,
      "maximum": 2100
    },

    "year_renovated": {
      "type": "integer",
      "minimum": 1800,
      "maximum": 2100
    },

    "certifications": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "type": {
            "type": "string",
            "enum": ["LEED", "BREEAM", "WELL", "Energy_Star", "Green_Mark", "other"]
          },
          "level": { "type": "string" },
          "date_certified": { "type": "string", "format": "date" },
          "expiry_date": { "type": "string", "format": "date" }
        }
      }
    },

    "operating_hours": {
      "type": "object",
      "properties": {
        "typical_start": {
          "type": "string",
          "pattern": "^([01]?[0-9]|2[0-3]):[0-5][0-9]$"
        },
        "typical_end": {
          "type": "string",
          "pattern": "^([01]?[0-9]|2[0-3]):[0-5][0-9]$"
        },
        "days_per_week": {
          "type": "integer",
          "minimum": 1,
          "maximum": 7
        }
      }
    },

    "contacts": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "role": { "type": "string" },
          "name": { "type": "string" },
          "email": { "type": "string", "format": "email" },
          "phone": { "type": "string" }
        }
      }
    },

    "metadata": {
      "type": "object",
      "additionalProperties": true,
      "description": "Custom metadata fields"
    }
  }
}
```

### 3.2.2 Location Hierarchy Schema

**Schema URI:** `https://wia.org/bems/v1/schemas/location.json`

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/bems/v1/schemas/location.json",
  "title": "WIA-BEMS Location",
  "description": "Spatial hierarchy within a building",
  "type": "object",
  "required": ["location_id", "building_id", "type", "name"],

  "properties": {
    "location_id": {
      "type": "string",
      "pattern": "^LOC-[A-Z0-9]{8,16}$"
    },

    "building_id": {
      "type": "string",
      "pattern": "^BLDG-[A-Z0-9]{6,12}$"
    },

    "parent_location_id": {
      "type": "string",
      "pattern": "^LOC-[A-Z0-9]{8,16}$",
      "description": "Parent in spatial hierarchy (null for building level)"
    },

    "type": {
      "type": "string",
      "enum": [
        "building",
        "wing",
        "floor",
        "zone",
        "room",
        "space",
        "outdoor"
      ]
    },

    "name": {
      "type": "string",
      "minLength": 1
    },

    "floor_number": {
      "type": "integer",
      "description": "Floor number (0 = ground, negative for basement)"
    },

    "area_sqm": {
      "type": "number",
      "minimum": 0
    },

    "space_type": {
      "type": "string",
      "enum": [
        "office_open",
        "office_private",
        "meeting_room",
        "conference_center",
        "lobby",
        "corridor",
        "stairwell",
        "elevator",
        "restroom",
        "kitchen",
        "cafeteria",
        "server_room",
        "mechanical",
        "electrical",
        "storage",
        "parking",
        "other"
      ]
    },

    "occupancy_design": {
      "type": "integer",
      "minimum": 0,
      "description": "Design occupancy capacity"
    },

    "hvac_zone_id": {
      "type": "string",
      "description": "HVAC control zone identifier"
    },

    "lighting_zone_id": {
      "type": "string",
      "description": "Lighting control zone identifier"
    },

    "tags": {
      "type": "array",
      "items": { "type": "string" },
      "description": "Haystack-compatible tags"
    }
  }
}
```

## 3.3 Energy Measurement Schemas

### 3.3.1 Energy Consumption Schema

**Schema URI:** `https://wia.org/bems/v1/schemas/energy-consumption.json`

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/bems/v1/schemas/energy-consumption.json",
  "title": "WIA-BEMS Energy Consumption",
  "description": "Energy consumption measurement for a time period",
  "type": "object",
  "required": ["measurement_id", "timestamp_start", "timestamp_end", "location", "energy"],

  "properties": {
    "measurement_id": {
      "type": "string",
      "format": "uuid"
    },

    "timestamp_start": {
      "type": "string",
      "format": "date-time",
      "description": "Period start (ISO 8601 with timezone)"
    },

    "timestamp_end": {
      "type": "string",
      "format": "date-time",
      "description": "Period end (ISO 8601 with timezone)"
    },

    "location": {
      "type": "object",
      "required": ["building_id"],
      "properties": {
        "building_id": { "type": "string" },
        "floor_id": { "type": "string" },
        "zone_id": { "type": "string" },
        "meter_id": { "type": "string" }
      }
    },

    "energy": {
      "type": "object",
      "required": ["value", "unit", "type"],
      "properties": {
        "value": {
          "type": "number",
          "minimum": 0,
          "description": "Energy consumption value"
        },
        "unit": {
          "type": "string",
          "enum": ["kWh", "MWh", "GJ", "therm", "kBtu"],
          "default": "kWh"
        },
        "type": {
          "type": "string",
          "enum": [
            "electricity",
            "natural_gas",
            "district_heating",
            "district_cooling",
            "fuel_oil",
            "propane",
            "steam",
            "chilled_water",
            "hot_water"
          ]
        },
        "subtype": {
          "type": "string",
          "enum": [
            "total",
            "hvac",
            "lighting",
            "plug_loads",
            "process",
            "domestic_hot_water",
            "elevators",
            "data_center",
            "kitchen",
            "other"
          ]
        }
      }
    },

    "quality": {
      "type": "object",
      "properties": {
        "status": {
          "type": "string",
          "enum": ["actual", "estimated", "interpolated", "manual"],
          "default": "actual"
        },
        "accuracy_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Measurement accuracy (%)"
        },
        "validation_status": {
          "type": "string",
          "enum": ["validated", "pending", "failed", "not_validated"]
        },
        "validation_rules_passed": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },

    "cost": {
      "type": "object",
      "properties": {
        "energy_cost": {
          "type": "number",
          "minimum": 0
        },
        "demand_cost": {
          "type": "number",
          "minimum": 0
        },
        "total_cost": {
          "type": "number",
          "minimum": 0
        },
        "currency": {
          "type": "string",
          "pattern": "^[A-Z]{3}$",
          "description": "ISO 4217 currency code"
        },
        "rate_schedule": {
          "type": "string"
        }
      }
    },

    "carbon": {
      "type": "object",
      "properties": {
        "emissions_kg_co2e": {
          "type": "number",
          "minimum": 0
        },
        "emission_factor": {
          "type": "number",
          "description": "kg CO2e per unit energy"
        },
        "emission_factor_source": {
          "type": "string"
        }
      }
    },

    "metadata": {
      "type": "object",
      "additionalProperties": true
    }
  }
}
```

### 3.3.2 Power Demand Schema

**Schema URI:** `https://wia.org/bems/v1/schemas/power-demand.json`

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/bems/v1/schemas/power-demand.json",
  "title": "WIA-BEMS Power Demand",
  "description": "Instantaneous or interval power demand measurement",
  "type": "object",
  "required": ["measurement_id", "timestamp", "location", "power"],

  "properties": {
    "measurement_id": {
      "type": "string",
      "format": "uuid"
    },

    "timestamp": {
      "type": "string",
      "format": "date-time"
    },

    "interval_seconds": {
      "type": "integer",
      "minimum": 1,
      "description": "Measurement interval (e.g., 900 for 15-minute demand)"
    },

    "location": {
      "type": "object",
      "required": ["building_id", "meter_id"],
      "properties": {
        "building_id": { "type": "string" },
        "meter_id": { "type": "string" },
        "circuit_id": { "type": "string" }
      }
    },

    "power": {
      "type": "object",
      "required": ["real_kw"],
      "properties": {
        "real_kw": {
          "type": "number",
          "description": "Real power (kW)"
        },
        "reactive_kvar": {
          "type": "number",
          "description": "Reactive power (kVAR)"
        },
        "apparent_kva": {
          "type": "number",
          "minimum": 0,
          "description": "Apparent power (kVA)"
        },
        "power_factor": {
          "type": "number",
          "minimum": -1,
          "maximum": 1,
          "description": "Power factor"
        }
      }
    },

    "voltage": {
      "type": "object",
      "properties": {
        "phase_a_v": { "type": "number" },
        "phase_b_v": { "type": "number" },
        "phase_c_v": { "type": "number" },
        "average_v": { "type": "number" }
      }
    },

    "current": {
      "type": "object",
      "properties": {
        "phase_a_a": { "type": "number", "minimum": 0 },
        "phase_b_a": { "type": "number", "minimum": 0 },
        "phase_c_a": { "type": "number", "minimum": 0 },
        "neutral_a": { "type": "number", "minimum": 0 }
      }
    },

    "frequency_hz": {
      "type": "number",
      "minimum": 45,
      "maximum": 65
    },

    "quality": {
      "type": "object",
      "properties": {
        "status": {
          "type": "string",
          "enum": ["normal", "estimated", "stale", "error"]
        },
        "accuracy_class": {
          "type": "string",
          "enum": ["0.2", "0.5", "1.0", "2.0"]
        }
      }
    }
  }
}
```

## 3.4 Environmental Data Schemas

### 3.4.1 Temperature Schema

**Schema URI:** `https://wia.org/bems/v1/schemas/temperature.json`

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/bems/v1/schemas/temperature.json",
  "title": "WIA-BEMS Temperature",
  "type": "object",
  "required": ["measurement_id", "timestamp", "location", "temperature"],

  "properties": {
    "measurement_id": { "type": "string", "format": "uuid" },
    "timestamp": { "type": "string", "format": "date-time" },

    "location": {
      "type": "object",
      "required": ["location_id"],
      "properties": {
        "building_id": { "type": "string" },
        "location_id": { "type": "string" },
        "sensor_id": { "type": "string" }
      }
    },

    "temperature": {
      "type": "object",
      "required": ["value", "type"],
      "properties": {
        "value": { "type": "number" },
        "unit": {
          "type": "string",
          "enum": ["C", "F", "K"],
          "default": "C"
        },
        "type": {
          "type": "string",
          "enum": [
            "ambient_air",
            "supply_air",
            "return_air",
            "mixed_air",
            "outdoor_air",
            "exhaust_air",
            "surface",
            "radiant",
            "dewpoint",
            "wet_bulb",
            "chilled_water_supply",
            "chilled_water_return",
            "hot_water_supply",
            "hot_water_return",
            "condenser_water_supply",
            "condenser_water_return"
          ]
        }
      }
    },

    "quality": {
      "type": "object",
      "properties": {
        "accuracy_c": {
          "type": "number",
          "description": "Accuracy in ±°C"
        },
        "calibration_date": {
          "type": "string",
          "format": "date"
        },
        "status": {
          "type": "string",
          "enum": ["normal", "warning", "error", "offline"]
        }
      }
    }
  }
}
```

### 3.4.2 Air Quality Schema

**Schema URI:** `https://wia.org/bems/v1/schemas/air-quality.json`

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/bems/v1/schemas/air-quality.json",
  "title": "WIA-BEMS Air Quality",
  "type": "object",
  "required": ["measurement_id", "timestamp", "location"],

  "properties": {
    "measurement_id": { "type": "string", "format": "uuid" },
    "timestamp": { "type": "string", "format": "date-time" },

    "location": {
      "type": "object",
      "required": ["location_id"],
      "properties": {
        "building_id": { "type": "string" },
        "location_id": { "type": "string" },
        "sensor_id": { "type": "string" }
      }
    },

    "co2": {
      "type": "object",
      "properties": {
        "value_ppm": {
          "type": "number",
          "minimum": 0,
          "maximum": 10000
        },
        "status": {
          "type": "string",
          "enum": ["good", "moderate", "poor", "hazardous"]
        }
      }
    },

    "humidity": {
      "type": "object",
      "properties": {
        "relative_percent": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        },
        "absolute_g_m3": {
          "type": "number",
          "minimum": 0
        }
      }
    },

    "particulate_matter": {
      "type": "object",
      "properties": {
        "pm1_ug_m3": { "type": "number", "minimum": 0 },
        "pm2_5_ug_m3": { "type": "number", "minimum": 0 },
        "pm10_ug_m3": { "type": "number", "minimum": 0 }
      }
    },

    "tvoc": {
      "type": "object",
      "properties": {
        "value_ppb": {
          "type": "number",
          "minimum": 0
        },
        "value_ug_m3": {
          "type": "number",
          "minimum": 0
        }
      }
    },

    "formaldehyde": {
      "type": "object",
      "properties": {
        "value_ppb": { "type": "number", "minimum": 0 }
      }
    },

    "ozone": {
      "type": "object",
      "properties": {
        "value_ppb": { "type": "number", "minimum": 0 }
      }
    },

    "carbon_monoxide": {
      "type": "object",
      "properties": {
        "value_ppm": { "type": "number", "minimum": 0 }
      }
    },

    "nitrogen_dioxide": {
      "type": "object",
      "properties": {
        "value_ppb": { "type": "number", "minimum": 0 }
      }
    },

    "air_quality_index": {
      "type": "object",
      "properties": {
        "value": {
          "type": "integer",
          "minimum": 0,
          "maximum": 500
        },
        "category": {
          "type": "string",
          "enum": ["good", "moderate", "unhealthy_sensitive", "unhealthy", "very_unhealthy", "hazardous"]
        },
        "dominant_pollutant": { "type": "string" }
      }
    }
  }
}
```

## 3.5 Occupancy and Space Utilization Schemas

### 3.5.1 Occupancy Schema

**Schema URI:** `https://wia.org/bems/v1/schemas/occupancy.json`

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/bems/v1/schemas/occupancy.json",
  "title": "WIA-BEMS Occupancy",
  "type": "object",
  "required": ["measurement_id", "timestamp", "location", "occupancy"],

  "properties": {
    "measurement_id": { "type": "string", "format": "uuid" },
    "timestamp": { "type": "string", "format": "date-time" },

    "location": {
      "type": "object",
      "required": ["location_id"],
      "properties": {
        "building_id": { "type": "string" },
        "location_id": { "type": "string" },
        "zone_id": { "type": "string" }
      }
    },

    "occupancy": {
      "type": "object",
      "required": ["occupied"],
      "properties": {
        "occupied": {
          "type": "boolean",
          "description": "Binary occupied/vacant status"
        },
        "count": {
          "type": "integer",
          "minimum": 0,
          "description": "Occupant count (null if only binary)"
        },
        "count_entering": {
          "type": "integer",
          "minimum": 0,
          "description": "People entering since last reading"
        },
        "count_exiting": {
          "type": "integer",
          "minimum": 0,
          "description": "People exiting since last reading"
        },
        "density_sqm_per_person": {
          "type": "number",
          "minimum": 0
        }
      }
    },

    "detection_method": {
      "type": "string",
      "enum": [
        "pir",
        "ultrasonic",
        "microwave",
        "camera_vision",
        "thermal_camera",
        "wifi_probe",
        "bluetooth",
        "badge_access",
        "co2_derived",
        "schedule_based",
        "manual"
      ]
    },

    "confidence": {
      "type": "number",
      "minimum": 0,
      "maximum": 1,
      "description": "Statistical confidence in measurement"
    },

    "privacy_preserved": {
      "type": "boolean",
      "default": true,
      "description": "Whether individual identification is prevented"
    }
  }
}
```

## 3.6 Equipment Status Schemas

### 3.6.1 HVAC Equipment Schema

**Schema URI:** `https://wia.org/bems/v1/schemas/hvac-equipment.json`

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.org/bems/v1/schemas/hvac-equipment.json",
  "title": "WIA-BEMS HVAC Equipment",
  "type": "object",
  "required": ["equipment_id", "timestamp", "type", "status"],

  "properties": {
    "equipment_id": { "type": "string" },
    "timestamp": { "type": "string", "format": "date-time" },

    "location": {
      "type": "object",
      "properties": {
        "building_id": { "type": "string" },
        "location_id": { "type": "string" }
      }
    },

    "type": {
      "type": "string",
      "enum": [
        "ahu",
        "rtu",
        "fcu",
        "vav",
        "cav",
        "chiller",
        "boiler",
        "heat_pump",
        "cooling_tower",
        "pump",
        "fan",
        "humidifier",
        "dehumidifier",
        "heat_exchanger",
        "economizer",
        "erv",
        "hrv",
        "vrf",
        "split_system",
        "ptac"
      ]
    },

    "status": {
      "type": "object",
      "required": ["operational"],
      "properties": {
        "operational": {
          "type": "string",
          "enum": ["running", "stopped", "standby", "starting", "stopping", "alarm", "offline"]
        },
        "mode": {
          "type": "string",
          "enum": ["heating", "cooling", "ventilating", "economizer", "auto", "off"]
        },
        "enable": { "type": "boolean" },
        "alarm_active": { "type": "boolean" }
      }
    },

    "setpoints": {
      "type": "object",
      "properties": {
        "supply_air_temp_c": { "type": "number" },
        "return_air_temp_c": { "type": "number" },
        "discharge_air_temp_c": { "type": "number" },
        "static_pressure_pa": { "type": "number" },
        "supply_air_flow_m3s": { "type": "number" },
        "chilled_water_temp_c": { "type": "number" },
        "hot_water_temp_c": { "type": "number" }
      }
    },

    "feedback": {
      "type": "object",
      "properties": {
        "supply_air_temp_c": { "type": "number" },
        "return_air_temp_c": { "type": "number" },
        "mixed_air_temp_c": { "type": "number" },
        "outdoor_air_temp_c": { "type": "number" },
        "supply_air_humidity_pct": { "type": "number" },
        "static_pressure_pa": { "type": "number" },
        "supply_air_flow_m3s": { "type": "number" }
      }
    },

    "outputs": {
      "type": "object",
      "properties": {
        "cooling_valve_pct": { "type": "number", "minimum": 0, "maximum": 100 },
        "heating_valve_pct": { "type": "number", "minimum": 0, "maximum": 100 },
        "outdoor_air_damper_pct": { "type": "number", "minimum": 0, "maximum": 100 },
        "return_air_damper_pct": { "type": "number", "minimum": 0, "maximum": 100 },
        "exhaust_air_damper_pct": { "type": "number", "minimum": 0, "maximum": 100 },
        "supply_fan_speed_pct": { "type": "number", "minimum": 0, "maximum": 100 },
        "return_fan_speed_pct": { "type": "number", "minimum": 0, "maximum": 100 },
        "humidifier_output_pct": { "type": "number", "minimum": 0, "maximum": 100 }
      }
    },

    "power": {
      "type": "object",
      "properties": {
        "total_kw": { "type": "number", "minimum": 0 },
        "supply_fan_kw": { "type": "number", "minimum": 0 },
        "return_fan_kw": { "type": "number", "minimum": 0 },
        "compressor_kw": { "type": "number", "minimum": 0 }
      }
    },

    "maintenance": {
      "type": "object",
      "properties": {
        "runtime_hours": { "type": "number", "minimum": 0 },
        "filter_dp_pa": { "type": "number" },
        "filter_change_due": { "type": "string", "format": "date" },
        "last_maintenance": { "type": "string", "format": "date" },
        "next_scheduled_maintenance": { "type": "string", "format": "date" }
      }
    },

    "alarms": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "code": { "type": "string" },
          "description": { "type": "string" },
          "severity": { "type": "string", "enum": ["info", "warning", "critical"] },
          "timestamp": { "type": "string", "format": "date-time" },
          "acknowledged": { "type": "boolean" }
        }
      }
    }
  }
}
```

## 3.7 Validation and Quality Assurance

### 3.7.1 Validation Rules

WIA-BEMS includes comprehensive validation rules to ensure data quality:

**Validation Categories:**

```typescript
interface ValidationRules {
  schemaValidation: {
    description: "JSON Schema compliance";
    enforcement: "Mandatory";
    tools: ["AJV", "json-schema-validator", "custom"];
  };

  rangeValidation: {
    temperature_c: { min: -50, max: 100, typical: { min: -20, max: 50 } };
    humidity_pct: { min: 0, max: 100 };
    co2_ppm: { min: 300, max: 10000, outdoor_baseline: 420 };
    power_factor: { min: -1, max: 1 };
    energy_kwh: { min: 0 };
  };

  temporalValidation: {
    timestampFormat: "ISO 8601 required";
    futureAllowed: false;
    maxAge: "24 hours for real-time";
    sequence: "Monotonically increasing within stream";
  };

  relationalValidation: {
    buildingExists: "building_id must exist in registry";
    locationHierarchy: "parent_location_id must be valid";
    meterAssignment: "meter_id must be assigned to building";
  };

  physicalValidation: {
    energyBalance: "Input energy ≈ output + storage delta";
    temperatureLogic: "Supply air temp consistent with mode";
    flowMassBalance: "Flow in ≈ flow out for closed systems";
  };
}
```

### 3.7.2 Quality Scoring

**Data Quality Index Calculation:**

```python
class WIABEMSQualityScorer:
    """
    Calculate data quality score for WIA-BEMS measurements
    """

    def __init__(self):
        self.weights = {
            'completeness': 0.25,
            'timeliness': 0.20,
            'accuracy': 0.25,
            'consistency': 0.15,
            'validity': 0.15
        }

    def calculate_quality_score(self, measurement: dict) -> dict:
        scores = {}

        # Completeness: Required fields present
        scores['completeness'] = self._score_completeness(measurement)

        # Timeliness: How recent is the data
        scores['timeliness'] = self._score_timeliness(measurement)

        # Accuracy: Measurement precision
        scores['accuracy'] = self._score_accuracy(measurement)

        # Consistency: Agreement with related data
        scores['consistency'] = self._score_consistency(measurement)

        # Validity: Passes validation rules
        scores['validity'] = self._score_validity(measurement)

        # Weighted total
        total = sum(
            scores[dim] * self.weights[dim]
            for dim in scores
        )

        return {
            'total_score': round(total, 2),
            'dimensions': scores,
            'quality_level': self._quality_level(total)
        }

    def _quality_level(self, score: float) -> str:
        if score >= 0.9: return 'excellent'
        if score >= 0.75: return 'good'
        if score >= 0.6: return 'acceptable'
        if score >= 0.4: return 'poor'
        return 'unusable'
```

## 3.8 Implementation Examples

### 3.8.1 TypeScript SDK Usage

```typescript
import {
  WIABEMSClient,
  EnergyConsumption,
  TemperatureMeasurement,
  ValidationResult
} from 'wia-bems-sdk';

// Initialize client
const client = new WIABEMSClient({
  baseUrl: 'https://api.building.example.com',
  accessToken: process.env.BEMS_TOKEN
});

// Create energy consumption record
const energyRecord: EnergyConsumption = {
  measurement_id: crypto.randomUUID(),
  timestamp_start: '2025-01-15T00:00:00Z',
  timestamp_end: '2025-01-15T01:00:00Z',
  location: {
    building_id: 'BLDG-HQ001',
    meter_id: 'METER-E-301'
  },
  energy: {
    value: 245.5,
    unit: 'kWh',
    type: 'electricity',
    subtype: 'total'
  },
  quality: {
    status: 'actual',
    accuracy_percent: 99.5,
    validation_status: 'validated'
  }
};

// Validate before submission
const validation: ValidationResult = await client.validate(energyRecord);
if (!validation.valid) {
  console.error('Validation errors:', validation.errors);
  throw new Error('Invalid data');
}

// Submit measurement
await client.submitEnergyConsumption(energyRecord);

// Query historical data
const historicalData = await client.queryEnergy({
  building_id: 'BLDG-HQ001',
  start: '2025-01-01T00:00:00Z',
  end: '2025-01-31T23:59:59Z',
  interval: '1h',
  energy_type: 'electricity'
});

console.log(`Total energy: ${historicalData.summary.total_kwh} kWh`);
```

---

**Chapter Summary:**

This chapter covered the foundational data formats of WIA-BEMS Phase 1:

- JSON Schema-based definitions for all building energy data types
- Core schemas for buildings, locations, and devices
- Energy measurement schemas with cost and carbon tracking
- Environmental schemas for temperature, humidity, and air quality
- Occupancy and space utilization with privacy preservation
- Equipment status schemas for HVAC and other systems
- Comprehensive validation and quality assurance mechanisms

In the next chapter, we will explore the API Interface specification (Phase 2), covering RESTful endpoints, authentication, and real-time data streaming.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity
