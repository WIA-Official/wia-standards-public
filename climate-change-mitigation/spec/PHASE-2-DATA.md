# WIA Climate Change Mitigation Data Format Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)
**Standard ID**: WIA-ENE-049

---

## Table of Contents

1. [Overview](#overview)
2. [Base Structure](#base-structure)
3. [GHG Emission Data](#ghg-emission-data)
4. [Mitigation Target Data](#mitigation-target-data)
5. [Policy & Measure Data](#policy--measure-data)
6. [Carbon Market Data](#carbon-market-data)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)

---

## Overview

### 1.1 Purpose

Phase 2 defines the standardized data formats for representing greenhouse gas emissions, mitigation targets, policies, and carbon market transactions in the WIA Climate Change Mitigation ecosystem.

**Key Design Principles**:
- Alignment with IPCC 2006 Guidelines (2019 Refinement)
- Compatible with UNFCCC reporting formats
- Support for Paris Agreement Enhanced Transparency Framework
- Interoperable with national GHG inventory systems
- Machine-readable for automated analysis

---

## Base Structure

### 2.1 Core Data Schema

```json
{
  "@context": "https://wiastandards.com/contexts/climate-mitigation/v1",
  "standardId": "WIA-ENE-049",
  "version": "1.0.0",
  "timestamp": "2025-01-15T12:00:00Z",
  "entity": {
    "id": "did:wia:country:kor",
    "name": "Republic of Korea",
    "type": "country | organization | project"
  },
  "reportingPeriod": {
    "startDate": "2024-01-01",
    "endDate": "2024-12-31",
    "frequency": "annual"
  },
  "emissionData": { ... },
  "mitigationTargets": { ... },
  "policies": { ... },
  "carbonMarkets": { ... }
}
```

### 2.2 Entity Types

| Type | Description | Examples |
|------|-------------|----------|
| **country** | National government | South Korea, Germany, Brazil |
| **region** | Subnational entity | California, Quebec, EU |
| **city** | Municipal government | Seoul, Copenhagen, Singapore |
| **organization** | Corporation or institution | Apple, Microsoft, university |
| **project** | Specific mitigation project | Solar farm, reforestation, energy efficiency |

---

## GHG Emission Data

### 3.1 Emission Inventory Schema

```json
{
  "emissionData": {
    "inventoryYear": 2024,
    "methodology": "IPCC 2006 Guidelines + 2019 Refinement",
    "coverageScope": {
      "gases": ["CO2", "CH4", "N2O", "HFCs", "PFCs", "SF6", "NF3"],
      "sectors": ["energy", "industrial_processes", "agriculture", "waste", "lulucf"],
      "geographicScope": "national"
    },
    "totalEmissions": {
      "value": 650.5,
      "unit": "MtCO2e",
      "gwp": "AR5",
      "excludingLULUCF": 650.5,
      "includingLULUCF": 630.2
    },
    "sectoralEmissions": [ ... ],
    "gasComposition": [ ... ],
    "uncertaintyAnalysis": { ... }
  }
}
```

### 3.2 Sectoral Emissions

**IPCC Sectors**:

```json
{
  "sectoralEmissions": [
    {
      "sector": "energy",
      "subsector": "electricity_heat",
      "emissions": {
        "CO2": 250.5,
        "CH4": 2.1,
        "N2O": 0.5,
        "totalCO2e": 253.1,
        "unit": "MtCO2e"
      },
      "activityData": {
        "coalConsumption": 50.2,
        "gasConsumption": 30.1,
        "oilConsumption": 10.5,
        "unit": "Mtoe"
      },
      "emissionFactors": {
        "coal": 95.3,
        "gas": 56.1,
        "oil": 73.3,
        "unit": "tCO2/TJ"
      }
    },
    {
      "sector": "transport",
      "subsector": "road_transport",
      "emissions": {
        "CO2": 90.5,
        "CH4": 0.5,
        "N2O": 1.2,
        "totalCO2e": 92.2,
        "unit": "MtCO2e"
      },
      "activityData": {
        "vehicleKilometers": 450.5,
        "unit": "billion km"
      }
    }
  ]
}
```

**Sector Codes** (IPCC):

| Code | Sector | Subsectors |
|------|--------|------------|
| **1** | Energy | 1.A Fuel combustion, 1.B Fugitive emissions |
| **2** | Industrial Processes | 2.A Mineral, 2.B Chemical, 2.C Metal, 2.D Non-energy products |
| **3** | Agriculture | 3.A Enteric fermentation, 3.B Manure, 3.C Rice, 3.D Soils |
| **4** | LULUCF | 4.A Forest land, 4.B Cropland, 4.C Grassland, 4.D Wetlands |
| **5** | Waste | 5.A Solid waste, 5.B Wastewater, 5.C Incineration |

### 3.3 Gas-Specific Data

```json
{
  "gasComposition": [
    {
      "gas": "CO2",
      "emissions": 580.5,
      "percentage": 89.2,
      "gwp": 1,
      "sources": ["fossil_fuel", "industrial_process", "land_use"],
      "unit": "MtCO2e"
    },
    {
      "gas": "CH4",
      "emissions": 45.2,
      "percentage": 6.9,
      "gwp": 28,
      "sources": ["agriculture", "waste", "energy"],
      "unit": "MtCO2e"
    },
    {
      "gas": "N2O",
      "emissions": 20.8,
      "percentage": 3.2,
      "gwp": 265,
      "sources": ["agriculture", "industry"],
      "unit": "MtCO2e"
    }
  ]
}
```

**Global Warming Potential** (GWP):

| Gas | Formula | AR5 GWP (100-year) | Lifetime |
|-----|---------|-------------------|----------|
| **CO2** | CO₂ | 1 | Variable |
| **CH4** | CH₄ | 28 | 12 years |
| **N2O** | N₂O | 265 | 121 years |
| **HFC-134a** | CH₂FCF₃ | 1,300 | 14 years |
| **SF6** | SF₆ | 23,500 | 3,200 years |

---

## Mitigation Target Data

### 4.1 NDC Target Schema

```json
{
  "mitigationTargets": {
    "ndc": {
      "version": "Enhanced NDC 2025",
      "submissionDate": "2025-01-15",
      "target": {
        "type": "absolute_reduction",
        "value": 40,
        "unit": "percent",
        "baseYear": 2018,
        "targetYear": 2030,
        "scope": {
          "gases": ["CO2", "CH4", "N2O", "HFCs", "PFCs", "SF6"],
          "sectors": ["all"],
          "coverage": 100,
          "unit": "percent"
        }
      },
      "targetEmissions": {
        "baseYear": 2018,
        "baseEmissions": 710.5,
        "targetYear": 2030,
        "targetEmissions": 426.3,
        "reductionAmount": 284.2,
        "unit": "MtCO2e"
      },
      "conditionalTarget": {
        "enabled": true,
        "value": 45,
        "conditions": "With international support"
      }
    }
  }
}
```

### 4.2 Target Types

| Type | Description | Example |
|------|-------------|---------|
| **absolute_reduction** | Reduce by X% from base year | 40% below 2018 by 2030 |
| **bau_reduction** | Reduce by X% from business-as-usual | 30% below BAU by 2030 |
| **intensity_reduction** | Reduce emissions per GDP/capita | 65% carbon intensity by 2030 |
| **absolute_limit** | Cap at X MtCO2e | Maximum 500 MtCO2e by 2030 |
| **peaking_year** | Peak by year X | Peak by 2025, carbon neutral by 2050 |

### 4.3 Net Zero Pledge

```json
{
  "netZeroTarget": {
    "targetYear": 2050,
    "scope": {
      "gases": ["all_ghg"],
      "sectors": ["all"],
      "coveragePercent": 100
    },
    "pathwayDescription": "Phase out coal by 2035, 100% renewable electricity by 2040, electrify transport and heating",
    "interimTargets": [
      { "year": 2030, "reductionPercent": 45 },
      { "year": 2040, "reductionPercent": 75 },
      { "year": 2050, "reductionPercent": 100 }
    ],
    "residualEmissions": {
      "sectors": ["aviation", "agriculture"],
      "amount": 10.5,
      "unit": "MtCO2e",
      "removalMethod": ["BECCS", "DACCS", "afforestation"]
    }
  }
}
```

---

## Policy & Measure Data

### 5.1 Mitigation Policy Schema

```json
{
  "policies": [
    {
      "policyId": "KOR-RPS-2025",
      "name": "Renewable Portfolio Standard",
      "type": "regulatory",
      "sector": "energy",
      "description": "Mandate 40% renewable electricity by 2030",
      "implementationDate": "2025-01-01",
      "targetDate": "2030-12-31",
      "status": "implemented",
      "instruments": [
        {
          "type": "regulatory_mandate",
          "details": "40% RPS for utilities"
        },
        {
          "type": "financial_incentive",
          "details": "Renewable energy certificates (RECs)"
        }
      ],
      "expectedImpact": {
        "emissionReduction": 50.2,
        "unit": "MtCO2e/year",
        "sector": "electricity",
        "methodology": "Ex-ante modeling"
      },
      "cobenefits": ["air_quality", "energy_security", "job_creation"]
    }
  ]
}
```

### 5.2 Policy Instruments

| Type | Description | Examples |
|------|-------------|----------|
| **carbon_pricing** | Price on emissions | ETS, carbon tax |
| **regulatory_mandate** | Required standards | RPS, fuel efficiency |
| **financial_incentive** | Subsidies, tax breaks | Solar subsidies, EV rebates |
| **information_program** | Awareness, labeling | Energy labels, campaigns |
| **voluntary_agreement** | Industry commitments | Net-zero pledges |
| **public_investment** | Government spending | Public transit, R&D |
| **phase_out** | Ban/restrict activity | Coal phase-out, ICE ban |

---

## Carbon Market Data

### 6.1 Carbon Credit Schema

```json
{
  "carbonMarkets": {
    "mechanism": "article_6.2",
    "credits": [
      {
        "creditId": "WIA-CC-2025-001",
        "projectId": "KOR-SOLAR-001",
        "projectName": "Jeju Island Offshore Wind Farm",
        "methodology": "CDM ACM0002",
        "vintageYear": 2024,
        "credits": {
          "issued": 500000,
          "retired": 0,
          "transferred": 0,
          "available": 500000,
          "unit": "tCO2e"
        },
        "verification": {
          "verifier": "TUV SUD",
          "verificationDate": "2025-01-10",
          "standard": "Gold Standard",
          "correspondingAdjustment": true
        },
        "sdgContributions": ["SDG7", "SDG13"]
      }
    ],
    "transactions": [
      {
        "transactionId": "TX-2025-001",
        "date": "2025-01-15",
        "seller": "did:wia:country:kor",
        "buyer": "did:wia:country:jpn",
        "credits": 100000,
        "price": 25.50,
        "currency": "USD",
        "unit": "tCO2e",
        "correspondingAdjustment": {
          "seller": -100000,
          "buyer": +100000
        }
      }
    ]
  }
}
```

### 6.2 Article 6 Mechanisms

**Article 6.2 - Cooperative Approaches**:
```json
{
  "article6.2": {
    "bilateralAgreement": {
      "parties": ["Republic of Korea", "Japan"],
      "agreementDate": "2024-06-01",
      "sectors": ["renewable_energy", "forest"],
      "correspondingAdjustments": "applied",
      "avoidDoubleCounting": true
    }
  }
}
```

**Article 6.4 - Sustainable Development Mechanism**:
```json
{
  "article6.4": {
    "projectType": "renewable_energy",
    "methodology": "SDM-RE-001",
    "registryId": "A6.4-2025-001",
    "hostCountry": "did:wia:country:kor",
    "sdgAlignment": ["SDG7", "SDG13"],
    "shareOfProceeds": {
      "adaptationFund": 0.05,
      "administrativeFees": 0.02
    }
  }
}
```

---

## Validation Rules

### 7.1 Data Quality Checks

**Emission Data**:
- Total emissions = sum of sectoral emissions (±0.1%)
- Total emissions = sum of gas emissions (±0.1%)
- All emissions >= 0
- Uncertainty bounds: 95% confidence interval
- Activity data matches national statistics

**Targets**:
- Target year > base year
- Reduction percent: 0-100%
- Unconditional + conditional targets consistent
- Coverage percent: 0-100%

**Completeness**:
- All IPCC sectors covered or explained
- All Kyoto gases covered or explained
- Time series: base year to reporting year

### 7.2 Consistency Checks

**Inter-annual Consistency**:
```javascript
// Emission changes should be explainable
if (Math.abs(emissions[year] - emissions[year-1]) > 0.1 * emissions[year-1]) {
  require(explanation);
}
```

**Cross-sector Consistency**:
```javascript
// Total energy consumption should match emission estimates
energyEmissions = energyConsumption * emissionFactor;
if (Math.abs(energyEmissions - reportedEmissions) > 0.05 * reportedEmissions) {
  flag_for_review();
}
```

---

## Examples

### 8.1 Complete Country Emission Report

```json
{
  "@context": "https://wiastandards.com/contexts/climate-mitigation/v1",
  "standardId": "WIA-ENE-049",
  "version": "1.0.0",
  "timestamp": "2025-01-15T12:00:00Z",
  "entity": {
    "id": "did:wia:country:kor",
    "name": "Republic of Korea",
    "type": "country",
    "population": 51780579,
    "gdp": 1823534,
    "gdpUnit": "million USD"
  },
  "reportingPeriod": {
    "inventoryYear": 2024,
    "submissionDate": "2025-01-15",
    "frequency": "annual"
  },
  "emissionData": {
    "methodology": "IPCC 2006 + 2019 Refinement",
    "totalEmissions": {
      "value": 650.5,
      "unit": "MtCO2e",
      "gwp": "AR5",
      "excludingLULUCF": 650.5,
      "includingLULUCF": 630.2
    },
    "perCapita": 12.57,
    "perGDP": 0.357,
    "sectoralEmissions": [
      {
        "sector": "energy",
        "emissions": 564.2,
        "percentage": 86.7
      },
      {
        "sector": "industrial_processes",
        "emissions": 52.1,
        "percentage": 8.0
      },
      {
        "sector": "agriculture",
        "emissions": 21.2,
        "percentage": 3.3
      },
      {
        "sector": "waste",
        "emissions": 13.0,
        "percentage": 2.0
      }
    ],
    "gasComposition": [
      {
        "gas": "CO2",
        "emissions": 580.5,
        "percentage": 89.2
      },
      {
        "gas": "CH4",
        "emissions": 45.2,
        "percentage": 6.9
      },
      {
        "gas": "N2O",
        "emissions": 20.8,
        "percentage": 3.2
      },
      {
        "gas": "F-gases",
        "emissions": 4.0,
        "percentage": 0.6
      }
    ]
  },
  "mitigationTargets": {
    "ndc": {
      "version": "Enhanced NDC 2025",
      "submissionDate": "2025-01-15",
      "target": {
        "type": "absolute_reduction",
        "value": 40,
        "baseYear": 2018,
        "targetYear": 2030,
        "baseEmissions": 710.5,
        "targetEmissions": 426.3,
        "unit": "MtCO2e"
      }
    },
    "netZeroTarget": {
      "targetYear": 2050,
      "scope": "all_ghg",
      "pathwayApproved": true
    }
  },
  "policies": [
    {
      "policyId": "KOR-COAL-PHASEOUT",
      "name": "Coal Power Plant Phase-out",
      "type": "phase_out",
      "sector": "energy",
      "targetDate": "2035-12-31",
      "expectedImpact": {
        "emissionReduction": 80.5,
        "unit": "MtCO2e/year"
      }
    }
  ]
}
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
