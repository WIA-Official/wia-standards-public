# WIA-ENE-051: Greenhouse Gas Monitoring Standard
## Phase 2: Data Format Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

---

## Table of Contents

1. [Overview](#overview)
2. [Data Schema](#data-schema)
3. [GHG Concentration Data](#ghg-concentration-data)
4. [Emission Inventory Data](#emission-inventory-data)
5. [Satellite Data Format](#satellite-data-format)
6. [Ground Station Data](#ground-station-data)
7. [Metadata Standards](#metadata-standards)
8. [Data Exchange Formats](#data-exchange-formats)
9. [Quality Flags](#quality-flags)
10. [Examples](#examples)

---

## Overview

### 1.1 Purpose

This specification defines standardized data formats for greenhouse gas monitoring, enabling interoperability between satellite systems, ground networks, national inventories, and reporting platforms.

**Key Features**:
- JSON/XML data exchange formats
- NetCDF for scientific data (satellite, model)
- CSV for tabular inventory data
- Quality assurance metadata
- Uncertainty quantification

### 1.2 Data Categories

| Category | Description | Format |
|----------|-------------|--------|
| **Concentration** | Atmospheric GHG levels (ppm, ppb) | JSON, NetCDF |
| **Emissions** | Source-based emissions (tons CO2e) | JSON, CSV |
| **Satellite** | Remote sensing retrievals | NetCDF, HDF5 |
| **Ground** | In-situ measurements | JSON, CSV |
| **Inventory** | National/sectoral totals | JSON, XML |

---

## Data Schema

### 2.1 Base GHG Data Object

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "GHGMeasurement",
  "id": "ghg:measurement:uuid",
  "timestamp": "2025-01-15T12:00:00Z",
  "location": {
    "type": "Point",
    "coordinates": [126.9780, 37.5665],
    "altitude": 50,
    "srid": "EPSG:4326"
  },
  "gas": "CO2",
  "concentration": {
    "value": 420.5,
    "unit": "ppm",
    "uncertainty": 0.5
  },
  "dataSource": {
    "platform": "SATELLITE",
    "instrument": "OCO-2",
    "level": "L2"
  },
  "quality": {
    "flag": "GOOD",
    "confidence": 0.95
  },
  "metadata": {
    "provider": "NASA",
    "license": "CC-BY-4.0"
  }
}
```

### 2.2 Required Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `@context` | URI | ✅ | JSON-LD context |
| `@type` | string | ✅ | Data type identifier |
| `id` | URI | ✅ | Unique identifier |
| `timestamp` | ISO 8601 | ✅ | Measurement time (UTC) |
| `location` | GeoJSON | ✅ | Geographic location |
| `gas` | enum | ✅ | GHG type (CO2, CH4, etc.) |
| `concentration` | object | ✅ | Measurement value |
| `dataSource` | object | ✅ | Source information |
| `quality` | object | ✅ | Quality indicators |

---

## GHG Concentration Data

### 3.1 Atmospheric Concentration Schema

```typescript
interface GHGConcentration {
  "@context": string;
  "@type": "GHGConcentration";
  id: string;
  timestamp: string; // ISO 8601
  location: GeoJSONPoint;
  gas: GHGType;
  concentration: Measurement;
  dataSource: DataSource;
  quality: QualityInfo;
  metadata?: Record<string, any>;
}

type GHGType = "CO2" | "CH4" | "N2O" | "HFC" | "PFC" | "SF6" | "NF3";

interface Measurement {
  value: number;
  unit: "ppm" | "ppb" | "ppt";
  uncertainty: number;
  uncertaintyType: "ABSOLUTE" | "RELATIVE";
}

interface GeoJSONPoint {
  type: "Point";
  coordinates: [number, number]; // [longitude, latitude]
  altitude?: number; // meters above sea level
  srid: string; // e.g., "EPSG:4326"
}
```

### 3.2 Example: CO2 Measurement

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "GHGConcentration",
  "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-15T12:30:00Z",
  "location": {
    "type": "Point",
    "coordinates": [126.9780, 37.5665],
    "altitude": 50,
    "srid": "EPSG:4326"
  },
  "gas": "CO2",
  "concentration": {
    "value": 420.5,
    "unit": "ppm",
    "uncertainty": 0.5,
    "uncertaintyType": "ABSOLUTE"
  },
  "dataSource": {
    "platform": "SATELLITE",
    "mission": "OCO-2",
    "instrument": "Spectrometer",
    "level": "L2",
    "version": "v10r"
  },
  "quality": {
    "flag": "GOOD",
    "confidence": 0.95,
    "cloudFraction": 0.05,
    "surfacePressure": 1013.25
  },
  "metadata": {
    "provider": "NASA JPL",
    "dataCenter": "GES DISC",
    "license": "CC-BY-4.0",
    "doi": "10.5067/ABC123"
  }
}
```

### 3.3 Example: Methane Measurement

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "GHGConcentration",
  "id": "urn:uuid:660e8400-e29b-41d4-a716-446655440001",
  "timestamp": "2025-01-15T14:00:00Z",
  "location": {
    "type": "Point",
    "coordinates": [-95.3698, 29.7604],
    "altitude": 10,
    "srid": "EPSG:4326"
  },
  "gas": "CH4",
  "concentration": {
    "value": 1950,
    "unit": "ppb",
    "uncertainty": 10,
    "uncertaintyType": "ABSOLUTE"
  },
  "dataSource": {
    "platform": "GROUND",
    "network": "NOAA GML",
    "station": "BRW",
    "instrument": "CRDS",
    "samplingFrequency": "1Hz"
  },
  "quality": {
    "flag": "GOOD",
    "confidence": 0.98,
    "calibrationDate": "2025-01-01"
  }
}
```

---

## Emission Inventory Data

### 4.1 Emission Data Schema

```typescript
interface EmissionInventory {
  "@context": string;
  "@type": "EmissionInventory";
  id: string;
  reportingEntity: string;
  inventoryYear: number;
  sector: SectorType;
  subsector?: string;
  gas: GHGType;
  emissions: EmissionValue;
  activityData?: ActivityData;
  emissionFactor?: EmissionFactor;
  methodology: Methodology;
  quality: InventoryQuality;
}

type SectorType = "ENERGY" | "INDUSTRIAL" | "AGRICULTURE" | "WASTE" | "LULUCF";

interface EmissionValue {
  value: number;
  unit: "tCO2e" | "GgCO2e" | "MtCO2e";
  uncertainty: number;
  uncertaintyType: "PERCENTAGE";
}

interface ActivityData {
  value: number;
  unit: string;
  description: string;
}

interface EmissionFactor {
  value: number;
  unit: string;
  source: "IPCC" | "NATIONAL" | "MEASURED";
  tier: 1 | 2 | 3;
}
```

### 4.2 Example: Energy Sector Emissions

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "EmissionInventory",
  "id": "urn:wia:inventory:KOR:2024:energy:001",
  "reportingEntity": "Republic of Korea",
  "inventoryYear": 2024,
  "sector": "ENERGY",
  "subsector": "Electricity and Heat Production",
  "gas": "CO2",
  "emissions": {
    "value": 245000000,
    "unit": "tCO2e",
    "uncertainty": 5,
    "uncertaintyType": "PERCENTAGE"
  },
  "activityData": {
    "value": 550000,
    "unit": "TJ",
    "description": "Coal consumption for electricity generation"
  },
  "emissionFactor": {
    "value": 94.6,
    "unit": "kgCO2/TJ",
    "source": "IPCC",
    "tier": 2
  },
  "methodology": {
    "approach": "IPCC 2006 Guidelines",
    "tier": 2,
    "calculationMethod": "ACTIVITY_DATA × EMISSION_FACTOR"
  },
  "quality": {
    "completeness": "100%",
    "consistency": "GOOD",
    "transparency": "FULL",
    "accuracy": "±5%"
  },
  "metadata": {
    "submissionDate": "2025-04-15",
    "verified": true,
    "verifier": "Third Party Auditor Inc."
  }
}
```

### 4.3 Example: Agriculture Sector

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "EmissionInventory",
  "id": "urn:wia:inventory:KOR:2024:agri:001",
  "reportingEntity": "Republic of Korea",
  "inventoryYear": 2024,
  "sector": "AGRICULTURE",
  "subsector": "Enteric Fermentation",
  "gas": "CH4",
  "emissions": {
    "value": 25000000,
    "unit": "tCO2e",
    "uncertainty": 20,
    "uncertaintyType": "PERCENTAGE"
  },
  "activityData": {
    "value": 3500000,
    "unit": "head",
    "description": "Cattle population"
  },
  "emissionFactor": {
    "value": 100,
    "unit": "kgCH4/head/year",
    "source": "IPCC",
    "tier": 1
  },
  "methodology": {
    "approach": "IPCC 2006 Guidelines",
    "tier": 1,
    "calculationMethod": "LIVESTOCK_POPULATION × EMISSION_FACTOR × GWP"
  },
  "quality": {
    "completeness": "95%",
    "consistency": "MODERATE",
    "transparency": "PARTIAL",
    "accuracy": "±20%"
  }
}
```

---

## Satellite Data Format

### 5.1 NetCDF Structure (OCO-2 L2)

```
netcdf oco2_L2_20250115 {
dimensions:
  sounding = UNLIMITED;
  vertex = 4;

variables:
  double time(sounding);
    time:long_name = "Time of sounding";
    time:units = "seconds since 1970-01-01 00:00:00 UTC";

  float latitude(sounding);
    latitude:long_name = "Center latitude";
    latitude:units = "degrees_north";
    latitude:valid_range = -90.0, 90.0;

  float longitude(sounding);
    longitude:long_name = "Center longitude";
    longitude:units = "degrees_east";
    longitude:valid_range = -180.0, 180.0;

  float xco2(sounding);
    xco2:long_name = "Column-averaged CO2";
    xco2:units = "ppm";
    xco2:uncertainty = "xco2_uncertainty";

  float xco2_uncertainty(sounding);
    xco2_uncertainty:long_name = "XCO2 uncertainty";
    xco2_uncertainty:units = "ppm";

  int xco2_quality_flag(sounding);
    xco2_quality_flag:long_name = "Quality flag";
    xco2_quality_flag:flag_values = 0, 1;
    xco2_quality_flag:flag_meanings = "good bad";

  float surface_pressure(sounding);
    surface_pressure:long_name = "Surface pressure";
    surface_pressure:units = "hPa";

// Global attributes
:title = "OCO-2 Level 2 XCO2 retrievals";
:institution = "NASA JPL";
:source = "OCO-2 satellite";
:version = "v10r";
:date_created = "2025-01-16T00:00:00Z";
}
```

### 5.2 JSON Conversion (Single Sounding)

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "SatelliteRetrieval",
  "mission": "OCO-2",
  "product": "L2_Lite",
  "version": "v10r",
  "sounding": {
    "id": "2025011512345678",
    "time": "2025-01-15T12:34:56Z",
    "location": {
      "type": "Polygon",
      "coordinates": [
        [
          [126.975, 37.565],
          [126.980, 37.565],
          [126.980, 37.570],
          [126.975, 37.570],
          [126.975, 37.565]
        ]
      ]
    },
    "xco2": {
      "value": 420.5,
      "unit": "ppm",
      "uncertainty": 0.5
    },
    "quality": {
      "flag": 0,
      "flagMeaning": "good",
      "cloudFraction": 0.05,
      "surfacePressure": 1013.25,
      "solarZenithAngle": 45.2
    }
  },
  "metadata": {
    "orbitNumber": 12345,
    "footprint": "nadir",
    "gainMode": "H"
  }
}
```

---

## Ground Station Data

### 6.1 NOAA Flask Sample Format

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "GroundMeasurement",
  "network": "NOAA GML",
  "station": {
    "code": "MLO",
    "name": "Mauna Loa Observatory",
    "location": {
      "type": "Point",
      "coordinates": [-155.5763, 19.5362],
      "altitude": 3397,
      "srid": "EPSG:4326"
    }
  },
  "sample": {
    "id": "MLO-2025-001-001",
    "collectionTime": "2025-01-15T12:00:00Z",
    "collectionMethod": "FLASK",
    "analysisDate": "2025-01-20T10:00:00Z"
  },
  "measurements": [
    {
      "gas": "CO2",
      "value": 420.5,
      "unit": "ppm",
      "uncertainty": 0.1,
      "instrument": "NDIR",
      "calibrationScale": "WMO-CO2-X2019"
    },
    {
      "gas": "CH4",
      "value": 1950,
      "unit": "ppb",
      "uncertainty": 2,
      "instrument": "GC-FID",
      "calibrationScale": "WMO-CH4-X2004A"
    }
  ],
  "quality": {
    "flag": "GOOD",
    "notes": ""
  }
}
```

### 6.2 TCCON Data Format

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "TCCONMeasurement",
  "site": {
    "code": "pa",
    "name": "Park Falls",
    "location": {
      "type": "Point",
      "coordinates": [-90.2732, 45.9459],
      "altitude": 442,
      "srid": "EPSG:4326"
    }
  },
  "timestamp": "2025-01-15T18:30:00Z",
  "solarZenithAngle": 65.2,
  "xco2": {
    "value": 420.8,
    "unit": "ppm",
    "uncertainty": 0.4,
    "retrievalAlgorithm": "GGG2020"
  },
  "xch4": {
    "value": 1.950,
    "unit": "ppm",
    "uncertainty": 0.01
  },
  "quality": {
    "flag": 0,
    "flagDescription": "good"
  }
}
```

---

## Metadata Standards

### 7.1 Provenance Information

```json
{
  "provenance": {
    "dataProvider": "NASA JPL",
    "dataCenter": "GES DISC",
    "processingLevel": "L2",
    "processingVersion": "v10r",
    "processingDate": "2025-01-16T00:00:00Z",
    "algorithmVersion": "ACOS v9",
    "inputData": [
      {
        "product": "OCO-2 L1B",
        "version": "v10r",
        "files": ["oco2_L1B_20250115_*.h5"]
      }
    ],
    "qualityControl": {
      "applied": true,
      "filters": ["cloud_filter", "aod_filter", "surface_type_filter"]
    }
  }
}
```

### 7.2 Licensing & Attribution

```json
{
  "license": {
    "type": "CC-BY-4.0",
    "url": "https://creativecommons.org/licenses/by/4.0/",
    "attribution": "NASA JPL OCO-2 Science Team"
  },
  "citation": {
    "doi": "10.5067/ABC123",
    "authors": "OCO-2 Science Team",
    "title": "OCO-2 Level 2 XCO2 Retrievals",
    "year": 2025,
    "publisher": "NASA GES DISC"
  }
}
```

---

## Data Exchange Formats

### 8.1 Supported Formats

| Format | Use Case | Advantages |
|--------|----------|------------|
| **JSON** | API, web services | Human-readable, flexible |
| **JSON-LD** | Linked data | Semantic web compatibility |
| **NetCDF** | Scientific data | Multidimensional arrays, metadata |
| **HDF5** | Satellite archives | High performance, compression |
| **CSV** | Inventory reports | Spreadsheet compatible |
| **XML** | UNFCCC submission | Standardized reporting format |

### 8.2 CSV Format (Inventory)

```csv
Country,Year,Sector,Subsector,Gas,Emissions_tCO2e,Uncertainty_%,Methodology
Republic of Korea,2024,Energy,Electricity,CO2,245000000,5,IPCC 2006
Republic of Korea,2024,Agriculture,Enteric,CH4,25000000,20,IPCC 2006
Republic of Korea,2024,Industry,Cement,CO2,35000000,8,IPCC 2006
```

### 8.3 XML Format (UNFCCC CRF)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CommonReportingFormat xmlns="http://unfccc.int/crf" version="2024">
  <Party>Republic of Korea</Party>
  <ReportingYear>2024</ReportingYear>
  <SubmissionDate>2025-04-15</SubmissionDate>

  <Summary>
    <TotalEmissions unit="MtCO2e">600</TotalEmissions>
    <TrendVs1990>-5%</TrendVs1990>
  </Summary>

  <SectoralEmissions>
    <Sector name="Energy">
      <Emissions unit="MtCO2e">450</Emissions>
      <Percentage>75%</Percentage>
    </Sector>
    <Sector name="Agriculture">
      <Emissions unit="MtCO2e">75</Emissions>
      <Percentage>12.5%</Percentage>
    </Sector>
  </SectoralEmissions>
</CommonReportingFormat>
```

---

## Quality Flags

### 9.1 Quality Flag System

| Flag | Value | Meaning | Action |
|------|-------|---------|--------|
| **GOOD** | 0 | High quality | Use for analysis |
| **MARGINAL** | 1 | Moderate quality | Use with caution |
| **BAD** | 2 | Poor quality | Exclude from analysis |
| **MISSING** | -999 | No data | Fill value |

### 9.2 Quality Criteria

**Satellite Data**:
```json
{
  "quality": {
    "flag": "GOOD",
    "criteria": {
      "cloudFraction": "<0.1",
      "solarZenithAngle": "<70°",
      "surfaceType": "land",
      "aod": "<0.3",
      "retrievalConvergence": "true"
    },
    "confidence": 0.95
  }
}
```

**Ground Data**:
```json
{
  "quality": {
    "flag": "GOOD",
    "criteria": {
      "calibrationStatus": "current",
      "instrumentStatus": "operational",
      "dataCompleteness": ">95%",
      "outlierTest": "passed"
    }
  }
}
```

---

## Examples

### 10.1 Complete Measurement Example

```json
{
  "@context": "https://wiastandards.com/context/ghg/v1",
  "@type": "GHGMeasurement",
  "id": "urn:wia:ghg:measurement:2025:001",
  "timestamp": "2025-01-15T12:00:00Z",
  "location": {
    "type": "Point",
    "coordinates": [126.9780, 37.5665],
    "altitude": 50,
    "srid": "EPSG:4326",
    "description": "Seoul, South Korea"
  },
  "gas": "CO2",
  "concentration": {
    "value": 420.5,
    "unit": "ppm",
    "uncertainty": 0.5,
    "uncertaintyType": "ABSOLUTE"
  },
  "dataSource": {
    "platform": "SATELLITE",
    "mission": "OCO-2",
    "instrument": "Spectrometer",
    "level": "L2",
    "version": "v10r",
    "soundingId": "2025011512000000"
  },
  "quality": {
    "flag": "GOOD",
    "flagValue": 0,
    "confidence": 0.95,
    "cloudFraction": 0.05,
    "surfacePressure": 1013.25,
    "solarZenithAngle": 45.2,
    "aod": 0.15
  },
  "provenance": {
    "provider": "NASA JPL",
    "processingDate": "2025-01-16T00:00:00Z",
    "processingVersion": "v10r",
    "license": "CC-BY-4.0",
    "doi": "10.5067/ABC123"
  },
  "metadata": {
    "orbitNumber": 12345,
    "footprintMode": "nadir",
    "gainMode": "H"
  }
}
```

---

**Document Status**: ✅ Phase 2 Complete
**Next Phase**: [PHASE-3-PROTOCOL.md](PHASE-3-PROTOCOL.md)
**Maintained by**: WIA Standards Committee
**弘益人間** · Benefit All Humanity

---

© 2025 WIA - World Certification Industry Association
Licensed under MIT License
