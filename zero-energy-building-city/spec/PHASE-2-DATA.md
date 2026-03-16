# WIA-CITY-005: Zero Energy Building - PHASE 2 DATA FORMAT

**Version:** 1.0
**Status:** Active
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Overview

This document specifies the data formats, structures, and schemas for all energy-related data in Zero Energy Buildings. Standardized data formats ensure interoperability, enable analytics, and support certification processes.

---

## 2. Core Data Structures

### 2.1 Energy Production Data

#### 2.1.1 Solar PV Production

```json
{
  "type": "solar_pv_production",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "system": {
    "id": "PV-SYS-001",
    "capacity_kw": 100,
    "panel_count": 250,
    "panel_wattage": 400,
    "inverter": {
      "model": "SolarEdge SE100K",
      "efficiency": 0.985,
      "count": 2
    },
    "orientation": {
      "azimuth": 180,
      "tilt": 30
    }
  },
  "production": {
    "instant_power_kw": 85.5,
    "daily_kwh": 420.5,
    "monthly_kwh": 12650,
    "annual_kwh": 145000,
    "performance_ratio": 0.82
  },
  "environmental": {
    "irradiance_w_m2": 850,
    "temperature_c": 25,
    "cloud_cover": 0.15
  },
  "status": "operational",
  "efficiency": 0.85,
  "alerts": []
}
```

#### 2.1.2 Wind Production

```json
{
  "type": "wind_production",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "system": {
    "id": "WIND-SYS-001",
    "turbine_count": 2,
    "turbine_model": "Bergey Excel 10",
    "rated_power_kw": 50,
    "hub_height_m": 24,
    "rotor_diameter_m": 7
  },
  "production": {
    "instant_power_kw": 28.5,
    "daily_kwh": 180.3,
    "monthly_kwh": 5200,
    "annual_kwh": 65000
  },
  "environmental": {
    "wind_speed_ms": 8.5,
    "wind_direction_deg": 270,
    "air_density_kg_m3": 1.225,
    "temperature_c": 15
  },
  "status": "operational",
  "capacity_factor": 0.35
}
```

#### 2.1.3 Geothermal Production

```json
{
  "type": "geothermal_production",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "system": {
    "id": "GEO-SYS-001",
    "type": "ground_source_heat_pump",
    "capacity_kw": 30,
    "borehole_count": 6,
    "borehole_depth_m": 100,
    "ground_temperature_c": 15
  },
  "production": {
    "thermal_output_kw": 25.5,
    "cop": 4.2,
    "daily_kwh": 200,
    "monthly_kwh": 6000,
    "annual_kwh": 73000
  },
  "operational": {
    "inlet_temp_c": 8,
    "outlet_temp_c": 3,
    "flow_rate_lpm": 45
  },
  "status": "operational"
}
```

### 2.2 Energy Consumption Data

#### 2.2.1 HVAC Consumption

```json
{
  "type": "hvac_consumption",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "system": {
    "id": "HVAC-001",
    "type": "heat_pump",
    "zones": 12
  },
  "consumption": {
    "heating_kw": 15.5,
    "cooling_kw": 8.2,
    "ventilation_kw": 3.5,
    "total_instant_kw": 27.2,
    "daily_kwh": 300,
    "monthly_kwh": 9000,
    "annual_kwh": 95000
  },
  "performance": {
    "heating_cop": 4.0,
    "cooling_eer": 3.5,
    "ventilation_efficiency": 0.85
  },
  "indoor_conditions": {
    "temperature_c": 22,
    "humidity_percent": 45,
    "co2_ppm": 650
  },
  "outdoor_conditions": {
    "temperature_c": -5,
    "humidity_percent": 70
  }
}
```

#### 2.2.2 Lighting Consumption

```json
{
  "type": "lighting_consumption",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "consumption": {
    "interior_kw": 8.5,
    "exterior_kw": 2.1,
    "total_instant_kw": 10.6,
    "daily_kwh": 100,
    "monthly_kwh": 3000,
    "annual_kwh": 35000
  },
  "zones": [
    {
      "zone_id": "ZONE-1",
      "name": "Office Floor 1",
      "power_kw": 3.2,
      "light_level_lux": 500,
      "occupancy": true,
      "daylight_harvesting": true
    }
  ],
  "controls": {
    "occupancy_sensors": 45,
    "daylight_sensors": 20,
    "automated_schedule": true
  }
}
```

#### 2.2.3 Equipment and Appliances

```json
{
  "type": "equipment_consumption",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "consumption": {
    "elevators_kw": 5.5,
    "pumps_kw": 3.2,
    "appliances_kw": 12.5,
    "ev_charging_kw": 7.0,
    "other_kw": 2.8,
    "total_instant_kw": 31.0,
    "daily_kwh": 120,
    "monthly_kwh": 3600,
    "annual_kwh": 43800
  },
  "breakdown": {
    "elevators": {"count": 2, "power_kw": 5.5},
    "water_pumps": {"count": 4, "power_kw": 3.2},
    "refrigeration": {"count": 6, "power_kw": 4.5},
    "it_equipment": {"count": 150, "power_kw": 8.0}
  }
}
```

### 2.3 Energy Storage Data

#### 2.3.1 Battery ESS

```json
{
  "type": "battery_ess",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "system": {
    "id": "ESS-001",
    "technology": "lithium_ion",
    "manufacturer": "Tesla",
    "model": "Powerpack 2",
    "capacity_kwh": 500,
    "power_rating_kw": 100,
    "voltage_v": 800,
    "warranty_years": 10,
    "warranty_cycles": 5000
  },
  "state": {
    "soc_percent": 65.5,
    "soe_kwh": 327.5,
    "voltage_v": 795,
    "current_a": 45.5,
    "temperature_c": 28,
    "status": "charging"
  },
  "performance": {
    "charge_power_kw": 35.5,
    "discharge_power_kw": 0,
    "round_trip_efficiency": 0.92,
    "cycle_count": 1250,
    "health_percent": 97.5
  },
  "daily_stats": {
    "energy_charged_kwh": 150,
    "energy_discharged_kwh": 120,
    "cycles": 0.6
  }
}
```

### 2.4 Grid Interaction Data

```json
{
  "type": "grid_interaction",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "connection": {
    "utility": "Green City Electric",
    "account_id": "GCE-987654321",
    "tariff": "net_metering_residential",
    "connection_point": "METER-001"
  },
  "current": {
    "import_kw": 0,
    "export_kw": 15.5,
    "net_kw": 15.5,
    "power_factor": 0.98,
    "frequency_hz": 60.02,
    "voltage_v": 240
  },
  "daily": {
    "import_kwh": 25.5,
    "export_kwh": 85.2,
    "net_kwh": 59.7,
    "revenue_usd": 10.23
  },
  "monthly": {
    "import_kwh": 450,
    "export_kwh": 1850,
    "net_kwh": 1400,
    "revenue_usd": 168.00
  },
  "pricing": {
    "import_rate_usd_kwh": 0.15,
    "export_rate_usd_kwh": 0.12,
    "time_of_use": "off_peak"
  }
}
```

### 2.5 Carbon Emissions Data

```json
{
  "type": "carbon_emissions",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "emissions": {
    "grid_electricity_kg_co2": 12.5,
    "natural_gas_kg_co2": 0,
    "other_fuels_kg_co2": 0,
    "total_kg_co2": 12.5
  },
  "offsets": {
    "renewable_production_kg_co2": -45.5,
    "exported_energy_kg_co2": -8.5,
    "total_offset_kg_co2": -54.0
  },
  "net": {
    "daily_kg_co2": -41.5,
    "monthly_kg_co2": -1245,
    "annual_kg_co2": -15000,
    "status": "carbon_negative"
  },
  "intensity": {
    "grid_g_co2_kwh": 490,
    "building_g_co2_kwh": -85.5
  },
  "targets": {
    "annual_target_kg_co2": 0,
    "achievement_percent": 100
  }
}
```

---

## 3. Aggregated Data Formats

### 3.1 Daily Energy Summary

```json
{
  "type": "daily_energy_summary",
  "version": "1.0",
  "date": "2025-12-25",
  "building_id": "ZEB-2025-12345",
  "production": {
    "solar_kwh": 420.5,
    "wind_kwh": 180.3,
    "geothermal_kwh": 200.0,
    "total_kwh": 800.8
  },
  "consumption": {
    "hvac_kwh": 300.0,
    "lighting_kwh": 100.0,
    "equipment_kwh": 120.0,
    "total_kwh": 520.0
  },
  "storage": {
    "charged_kwh": 150.0,
    "discharged_kwh": 120.0,
    "net_kwh": -30.0
  },
  "grid": {
    "import_kwh": 25.5,
    "export_kwh": 85.2,
    "net_kwh": 59.7
  },
  "balance": {
    "production_kwh": 800.8,
    "consumption_kwh": 520.0,
    "surplus_kwh": 280.8,
    "balance_ratio": 1.54,
    "self_sufficiency_percent": 95.1
  },
  "carbon": {
    "emissions_kg_co2": 12.5,
    "offsets_kg_co2": -54.0,
    "net_kg_co2": -41.5
  },
  "weather": {
    "avg_temperature_c": 15,
    "max_temperature_c": 22,
    "min_temperature_c": 8,
    "avg_irradiance_w_m2": 600,
    "avg_wind_speed_ms": 6.5
  }
}
```

### 3.2 Monthly Performance Report

```json
{
  "type": "monthly_performance_report",
  "version": "1.0",
  "year": 2025,
  "month": 12,
  "building_id": "ZEB-2025-12345",
  "production": {
    "solar_kwh": 12650,
    "wind_kwh": 5200,
    "geothermal_kwh": 6000,
    "total_kwh": 23850
  },
  "consumption": {
    "hvac_kwh": 9000,
    "lighting_kwh": 3000,
    "equipment_kwh": 3600,
    "total_kwh": 15600
  },
  "grid": {
    "import_kwh": 450,
    "export_kwh": 1850,
    "net_kwh": 1400,
    "cost_usd": -100.50
  },
  "balance": {
    "balance_ratio": 1.53,
    "self_sufficiency_percent": 97.1,
    "zeb_grade": 1
  },
  "carbon": {
    "net_emissions_kg_co2": -1245,
    "carbon_neutral": true
  },
  "efficiency": {
    "solar_performance_ratio": 0.82,
    "hvac_cop": 3.8,
    "lighting_lpd_w_m2": 6.5
  },
  "cost_savings": {
    "utility_bill_usd": -100.50,
    "avoided_cost_usd": 2340.00,
    "total_savings_usd": 2440.50
  }
}
```

### 3.3 Annual Certification Data

```json
{
  "type": "annual_certification_data",
  "version": "1.0",
  "year": 2025,
  "building_id": "ZEB-2025-12345",
  "building_info": {
    "name": "Green Tower A",
    "address": "123 Eco Street, Green City, GC 12345",
    "type": "residential",
    "floor_area_m2": 5000,
    "occupancy": 250,
    "construction_year": 2024
  },
  "annual_production": {
    "solar_kwh": 145000,
    "wind_kwh": 65000,
    "geothermal_kwh": 73000,
    "total_kwh": 283000
  },
  "annual_consumption": {
    "hvac_kwh": 95000,
    "lighting_kwh": 35000,
    "equipment_kwh": 43800,
    "ev_charging_kwh": 12000,
    "total_kwh": 185800
  },
  "annual_grid": {
    "import_kwh": 5400,
    "export_kwh": 22200,
    "net_kwh": 16800,
    "revenue_usd": 2016.00
  },
  "performance_metrics": {
    "balance_ratio": 1.523,
    "self_sufficiency_percent": 97.1,
    "eui_kwh_m2_year": 37.16,
    "renewable_fraction": 1.523,
    "zeb_grade": 1,
    "grade_description": "Premium"
  },
  "carbon_metrics": {
    "total_emissions_kg_co2": 2646,
    "total_offsets_kg_co2": -17952,
    "net_emissions_kg_co2": -15306,
    "carbon_intensity_kg_co2_m2_year": -3.06,
    "carbon_neutral": true,
    "carbon_negative": true
  },
  "cost_metrics": {
    "utility_cost_usd": -2016.00,
    "avoided_cost_usd": 27870.00,
    "total_savings_usd": 29886.00,
    "payback_period_years": 8.5
  },
  "certification_status": {
    "certification_date": "2025-12-25",
    "expiration_date": "2026-12-25",
    "auditor": "WIA Certified Auditor #1234",
    "vc_issued": true,
    "qr_code": "https://verify.wia.official/zeb/ZEB-2025-12345"
  }
}
```

---

## 4. Real-Time Monitoring Data

### 4.1 Live Dashboard Feed

```json
{
  "type": "realtime_dashboard",
  "version": "1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "building_id": "ZEB-2025-12345",
  "update_interval_seconds": 5,
  "current_status": {
    "production_kw": 85.5,
    "consumption_kw": 52.3,
    "ess_power_kw": 15.2,
    "grid_power_kw": 18.0,
    "balance": "surplus"
  },
  "instantaneous": {
    "net_power_kw": 33.2,
    "self_sufficiency_percent": 100,
    "carbon_rate_kg_co2_hour": -2.5
  },
  "today_totals": {
    "production_kwh": 420.5,
    "consumption_kwh": 260.8,
    "balance_kwh": 159.7,
    "carbon_offset_kg_co2": -25.5
  },
  "system_health": {
    "solar": "optimal",
    "wind": "operational",
    "ess": "charging",
    "grid": "exporting",
    "overall": "excellent"
  },
  "alerts": []
}
```

---

## 5. Data Exchange Protocols

### 5.1 MQTT Topics

```
zeb/{building_id}/production/solar
zeb/{building_id}/production/wind
zeb/{building_id}/production/geothermal
zeb/{building_id}/consumption/hvac
zeb/{building_id}/consumption/lighting
zeb/{building_id}/consumption/equipment
zeb/{building_id}/storage/ess
zeb/{building_id}/grid/realtime
zeb/{building_id}/carbon/realtime
zeb/{building_id}/summary/daily
zeb/{building_id}/alerts
```

### 5.2 REST API Endpoints

```
GET  /api/v1/buildings/{building_id}/production/current
GET  /api/v1/buildings/{building_id}/consumption/current
GET  /api/v1/buildings/{building_id}/balance/daily/{date}
GET  /api/v1/buildings/{building_id}/balance/monthly/{year}/{month}
GET  /api/v1/buildings/{building_id}/balance/annual/{year}
GET  /api/v1/buildings/{building_id}/certificate
POST /api/v1/buildings/{building_id}/data/production
POST /api/v1/buildings/{building_id}/data/consumption
GET  /api/v1/buildings/{building_id}/carbon/current
GET  /api/v1/buildings/{building_id}/zeb-grade
```

### 5.3 WebSocket Streams

```javascript
// Real-time energy data stream
const ws = new WebSocket('wss://api.wia.official/v1/stream');

ws.send(JSON.stringify({
  action: 'subscribe',
  building_id: 'ZEB-2025-12345',
  streams: ['production', 'consumption', 'balance']
}));

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Real-time update:', data);
};
```

---

## 6. Data Validation Rules

### 6.1 Production Data

- **Solar Production**: 0 ≤ power ≤ rated capacity × 1.1
- **Wind Production**: 0 ≤ power ≤ rated capacity
- **Efficiency**: 0 ≤ efficiency ≤ 1.0
- **Timestamp**: Must be ISO 8601 format with timezone

### 6.2 Consumption Data

- **Power**: Must be non-negative
- **Total**: Sum of sub-categories must equal total
- **Efficiency**: COP ≥ 1.0, EER ≥ 1.0

### 6.3 ESS Data

- **State of Charge**: 0 ≤ SoC ≤ 100%
- **Power**: -max_discharge ≤ power ≤ max_charge
- **Health**: 0 ≤ health ≤ 100%

---

## 7. Data Retention Policy

| Data Type | Real-time | Daily | Monthly | Annual |
|-----------|-----------|-------|---------|--------|
| **Instantaneous** | 7 days | - | - | - |
| **15-minute intervals** | 30 days | 90 days | - | - |
| **Hourly** | 90 days | 1 year | - | - |
| **Daily summaries** | - | 2 years | 5 years | - |
| **Monthly reports** | - | - | 5 years | 10 years |
| **Annual certification** | - | - | - | Permanent |

---

## 8. Data Security

### 8.1 Access Control

- **Public**: Real-time dashboard (aggregated), certification status
- **Building Owner**: Full access to all data
- **Utility**: Grid interaction data
- **Auditor**: Read-only access for certification
- **WIA**: Anonymized data for research

### 8.2 Encryption

- **In Transit**: TLS 1.3 for all API calls
- **At Rest**: AES-256 encryption for stored data
- **Keys**: Managed through secure key management service

### 8.3 Privacy

- **Personal Data**: No occupant-level data collection
- **Aggregation**: Minimum 5-minute intervals for public data
- **Anonymization**: Remove PII before sharing

---

## 9. Example Implementation

### 9.1 TypeScript Interface

```typescript
interface EnergyProductionData {
  type: 'solar_pv_production' | 'wind_production' | 'geothermal_production';
  version: string;
  timestamp: string;
  building_id: string;
  system: {
    id: string;
    capacity_kw: number;
    // ... system-specific fields
  };
  production: {
    instant_power_kw: number;
    daily_kwh: number;
    monthly_kwh: number;
    annual_kwh: number;
  };
  status: 'operational' | 'degraded' | 'offline' | 'maintenance';
}

interface ZEBCertificationData {
  building_id: string;
  year: number;
  annual_production: {
    solar_kwh: number;
    wind_kwh: number;
    geothermal_kwh: number;
    total_kwh: number;
  };
  annual_consumption: {
    hvac_kwh: number;
    lighting_kwh: number;
    equipment_kwh: number;
    total_kwh: number;
  };
  performance_metrics: {
    balance_ratio: number;
    self_sufficiency_percent: number;
    eui_kwh_m2_year: number;
    zeb_grade: 1 | 2 | 3 | 4 | 5;
  };
}
```

---

**弘益人間 · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
WIA-CITY-005 v1.0 - Data Format Specification
https://wia.official/standards/CITY/005/data
