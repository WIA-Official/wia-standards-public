# WIA-AGRI-025: Insect Protein Standard
## PHASE 1: Data Format Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-025

---

## 1. Overview

This specification defines standardized JSON data formats for insect protein production, certification, and data exchange. It enables transparent tracking of nutrition, safety, sustainability, and traceability across the entire supply chain.

### 1.1 Purpose

- Standardize insect protein data representation
- Enable interoperability between farms, processors, and retailers
- Support regulatory compliance and certification
- Facilitate transparent communication of nutrition and safety data

### 1.2 Scope

This phase covers:
- Insect species identification
- Nutrition profile data structures
- Safety and quality testing results
- Sustainability metrics
- Production and processing metadata
- Certification information

---

## 2. Core Data Structures

### 2.1 Insect Product

The primary data structure representing an insect protein product.

```json
{
  "id": "string",
  "version": "1.0",
  "standard": "WIA-AGRI-025",
  "species": {
    "commonName": "string",
    "scientificName": "string",
    "taxonomy": {
      "order": "string",
      "family": "string",
      "genus": "string",
      "species": "string"
    }
  },
  "productForm": "whole|powder|protein-isolate|oil|paste|flour",
  "batchNumber": "string",
  "productionDate": "ISO 8601 datetime",
  "expiryDate": "ISO 8601 datetime",
  "farmInfo": {
    "farmId": "string",
    "farmName": "string",
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
  "nutrition": "NutritionProfile",
  "safety": "SafetyProfile",
  "sustainability": "SustainabilityMetrics",
  "processing": "ProcessingInfo",
  "certification": "CertificationInfo",
  "traceability": "TraceabilityInfo"
}
```

### 2.2 Nutrition Profile

Comprehensive nutritional composition data.

```json
{
  "macronutrients": {
    "protein": {
      "value": "number",
      "unit": "g/100g",
      "method": "Kjeldahl|Dumas|Bradford",
      "testDate": "ISO 8601 date"
    },
    "fat": {
      "value": "number",
      "unit": "g/100g",
      "saturated": "number",
      "monounsaturated": "number",
      "polyunsaturated": "number",
      "omega3": "number",
      "omega6": "number"
    },
    "carbohydrates": {
      "total": "number",
      "fiber": "number",
      "chitin": "number",
      "sugars": "number"
    },
    "moisture": {
      "value": "number",
      "unit": "g/100g"
    },
    "ash": {
      "value": "number",
      "unit": "g/100g"
    }
  },
  "aminoAcids": {
    "essential": {
      "histidine": "number",
      "isoleucine": "number",
      "leucine": "number",
      "lysine": "number",
      "methionine": "number",
      "phenylalanine": "number",
      "threonine": "number",
      "tryptophan": "number",
      "valine": "number"
    },
    "nonEssential": {
      "alanine": "number",
      "arginine": "number",
      "asparticAcid": "number",
      "cysteine": "number",
      "glutamicAcid": "number",
      "glycine": "number",
      "proline": "number",
      "serine": "number",
      "tyrosine": "number"
    },
    "unit": "g/100g protein"
  },
  "vitamins": {
    "A": { "value": "number", "unit": "μg" },
    "B1": { "value": "number", "unit": "mg" },
    "B2": { "value": "number", "unit": "mg" },
    "B3": { "value": "number", "unit": "mg" },
    "B5": { "value": "number", "unit": "mg" },
    "B6": { "value": "number", "unit": "mg" },
    "B9": { "value": "number", "unit": "μg" },
    "B12": { "value": "number", "unit": "μg" },
    "C": { "value": "number", "unit": "mg" },
    "D": { "value": "number", "unit": "μg" },
    "E": { "value": "number", "unit": "mg" },
    "K": { "value": "number", "unit": "μg" }
  },
  "minerals": {
    "calcium": { "value": "number", "unit": "mg" },
    "iron": { "value": "number", "unit": "mg" },
    "magnesium": { "value": "number", "unit": "mg" },
    "phosphorus": { "value": "number", "unit": "mg" },
    "potassium": { "value": "number", "unit": "mg" },
    "sodium": { "value": "number", "unit": "mg" },
    "zinc": { "value": "number", "unit": "mg" },
    "copper": { "value": "number", "unit": "mg" },
    "manganese": { "value": "number", "unit": "mg" },
    "selenium": { "value": "number", "unit": "μg" }
  },
  "energyContent": {
    "calories": { "value": "number", "unit": "kcal/100g" },
    "kilojoules": { "value": "number", "unit": "kJ/100g" }
  }
}
```

### 2.3 Safety Profile

Food safety and quality testing results.

```json
{
  "microbiological": {
    "totalPlateCount": {
      "value": "number",
      "unit": "CFU/g",
      "limit": "number",
      "passed": "boolean",
      "testDate": "ISO 8601 date"
    },
    "pathogenTests": {
      "salmonella": {
        "detected": "boolean",
        "method": "string",
        "testDate": "ISO 8601 date"
      },
      "listeria": {
        "detected": "boolean",
        "method": "string",
        "testDate": "ISO 8601 date"
      },
      "ecoli": {
        "value": "number",
        "unit": "CFU/g",
        "limit": "number",
        "passed": "boolean"
      }
    }
  },
  "heavyMetals": {
    "lead": {
      "value": "number",
      "unit": "mg/kg",
      "limit": "number",
      "passed": "boolean"
    },
    "cadmium": {
      "value": "number",
      "unit": "mg/kg",
      "limit": "number",
      "passed": "boolean"
    },
    "mercury": {
      "value": "number",
      "unit": "mg/kg",
      "limit": "number",
      "passed": "boolean"
    },
    "arsenic": {
      "value": "number",
      "unit": "mg/kg",
      "limit": "number",
      "passed": "boolean"
    }
  },
  "allergens": {
    "chitin": {
      "present": "boolean",
      "level": "low|medium|high",
      "warningRequired": "boolean"
    },
    "crossContamination": {
      "gluten": "boolean",
      "soy": "boolean",
      "dairy": "boolean",
      "nuts": "boolean"
    }
  },
  "pesticides": {
    "tested": "boolean",
    "residues": [
      {
        "substance": "string",
        "value": "number",
        "unit": "mg/kg",
        "limit": "number",
        "passed": "boolean"
      }
    ]
  },
  "antibiotics": {
    "tested": "boolean",
    "residues": []
  },
  "mycotoxins": {
    "tested": "boolean",
    "aflatoxin": {
      "value": "number",
      "unit": "μg/kg",
      "limit": "number",
      "passed": "boolean"
    }
  }
}
```

### 2.4 Sustainability Metrics

Environmental impact and sustainability data.

```json
{
  "carbonFootprint": {
    "total": {
      "value": "number",
      "unit": "kg CO2e/kg product"
    },
    "breakdown": {
      "farming": "number",
      "feed": "number",
      "processing": "number",
      "packaging": "number",
      "transport": "number"
    },
    "calculationMethod": "string",
    "verifiedBy": "string"
  },
  "waterUsage": {
    "total": {
      "value": "number",
      "unit": "L/kg product"
    },
    "farming": "number",
    "processing": "number"
  },
  "landUse": {
    "value": "number",
    "unit": "m²/kg/year",
    "comparison": {
      "beef": "number",
      "chicken": "number",
      "pork": "number"
    }
  },
  "feedConversionRatio": {
    "value": "number",
    "description": "kg feed / kg insect protein",
    "feedSource": "string"
  },
  "energyConsumption": {
    "total": {
      "value": "number",
      "unit": "MJ/kg product"
    },
    "renewable": "number",
    "nonRenewable": "number"
  },
  "wasteManagement": {
    "frass": {
      "quantity": "number",
      "unit": "kg/kg product",
      "utilization": "fertilizer|animal-feed|compost"
    },
    "exoskeletons": {
      "quantity": "number",
      "utilization": "chitin-extraction|compost"
    }
  },
  "biodiversity": {
    "impact": "positive|neutral|negative",
    "description": "string"
  }
}
```

### 2.5 Processing Information

Details about processing methods and conditions.

```json
{
  "harvestMethod": "manual|automated|semi-automated",
  "harvestDate": "ISO 8601 datetime",
  "insectAge": {
    "value": "number",
    "unit": "days"
  },
  "processingSteps": [
    {
      "step": "string",
      "method": "string",
      "temperature": {
        "value": "number",
        "unit": "celsius"
      },
      "duration": {
        "value": "number",
        "unit": "minutes|hours"
      },
      "timestamp": "ISO 8601 datetime"
    }
  ],
  "dryingMethod": "freeze-drying|oven-drying|sun-drying|spray-drying",
  "sterilization": {
    "method": "heat|radiation|chemical",
    "temperature": "number",
    "duration": "number",
    "verified": "boolean"
  },
  "packaging": {
    "type": "vacuum|modified-atmosphere|standard",
    "material": "string",
    "recyclable": "boolean",
    "packagingDate": "ISO 8601 date"
  },
  "storage": {
    "temperature": {
      "value": "number",
      "unit": "celsius"
    },
    "humidity": {
      "value": "number",
      "unit": "percent"
    },
    "conditions": "string"
  }
}
```

### 2.6 Certification Information

Certification and compliance data.

```json
{
  "certified": "boolean",
  "certificationId": "string",
  "standardVersion": "1.0",
  "certificationBody": {
    "name": "string",
    "accreditation": "string",
    "contactInfo": {
      "email": "string",
      "phone": "string",
      "website": "string"
    }
  },
  "issueDate": "ISO 8601 date",
  "expiryDate": "ISO 8601 date",
  "scope": "string",
  "additionalCertifications": [
    {
      "type": "organic|halal|kosher|HACCP|ISO22000",
      "certificationNumber": "string",
      "issuer": "string",
      "validUntil": "ISO 8601 date"
    }
  ],
  "complianceStatus": {
    "regulatoryCompliance": "boolean",
    "regulations": ["string"],
    "auditDate": "ISO 8601 date",
    "nextAuditDate": "ISO 8601 date"
  }
}
```

### 2.7 Traceability Information

Supply chain traceability data.

```json
{
  "traceabilityId": "string",
  "supplyChain": [
    {
      "stage": "farming|processing|packaging|distribution|retail",
      "operator": {
        "id": "string",
        "name": "string",
        "location": "string"
      },
      "timestamp": "ISO 8601 datetime",
      "batchNumber": "string",
      "certifications": ["string"]
    }
  ],
  "feedSource": {
    "type": "string",
    "origin": "string",
    "organic": "boolean",
    "traceabilityCode": "string"
  },
  "blockchain": {
    "enabled": "boolean",
    "network": "string",
    "transactionHash": "string",
    "smartContractAddress": "string"
  }
}
```

---

## 3. Supported Insect Species

### 3.1 Crickets (Acheta domesticus)

**Common Name:** House Cricket
**Protein Content:** 60-70% (dry weight)
**Key Characteristics:** High protein, complete amino acid profile

```json
{
  "species": {
    "commonName": "House Cricket",
    "scientificName": "Acheta domesticus",
    "taxonomy": {
      "order": "Orthoptera",
      "family": "Gryllidae",
      "genus": "Acheta",
      "species": "domesticus"
    }
  }
}
```

### 3.2 Mealworms (Tenebrio molitor)

**Common Name:** Yellow Mealworm
**Protein Content:** 45-55% (dry weight)
**Key Characteristics:** Rich in omega-3 and omega-6 fatty acids

```json
{
  "species": {
    "commonName": "Yellow Mealworm",
    "scientificName": "Tenebrio molitor",
    "taxonomy": {
      "order": "Coleoptera",
      "family": "Tenebrionidae",
      "genus": "Tenebrio",
      "species": "molitor"
    }
  }
}
```

### 3.3 Black Soldier Fly (Hermetia illucens)

**Common Name:** Black Soldier Fly
**Protein Content:** 40-45% (larval stage, dry weight)
**Key Characteristics:** Excellent for organic waste conversion

```json
{
  "species": {
    "commonName": "Black Soldier Fly",
    "scientificName": "Hermetia illucens",
    "taxonomy": {
      "order": "Diptera",
      "family": "Stratiomyidae",
      "genus": "Hermetia",
      "species": "illucens"
    }
  }
}
```

### 3.4 Grasshoppers (Locusta migratoria)

**Common Name:** Migratory Locust
**Protein Content:** 50-60% (dry weight)
**Key Characteristics:** Traditional protein source, high digestibility

```json
{
  "species": {
    "commonName": "Migratory Locust",
    "scientificName": "Locusta migratoria",
    "taxonomy": {
      "order": "Orthoptera",
      "family": "Acrididae",
      "genus": "Locusta",
      "species": "migratoria"
    }
  }
}
```

---

## 4. Product Forms

### 4.1 Whole Insects

Complete insects, typically dried and packaged.

**Use Cases:** Direct consumption, traditional cuisine, specialty markets

### 4.2 Powder

Ground insects in powder form.

**Use Cases:** Protein supplements, baking, smoothies, fortification

### 4.3 Protein Isolate

Extracted and purified protein (>80% protein content).

**Use Cases:** Sports nutrition, meal replacements, food fortification

### 4.4 Oil

Extracted insect oil rich in omega fatty acids.

**Use Cases:** Cooking oil, nutritional supplements, cosmetics

### 4.5 Paste

Ground insects in paste form with moisture content.

**Use Cases:** Food manufacturing, pet food, aquaculture feed

### 4.6 Flour

Fine powder suitable for baking applications.

**Use Cases:** Bread, pastries, pasta, snacks

---

## 5. Data Validation Rules

### 5.1 Required Fields

All insect product records MUST include:
- `id`
- `version`
- `standard`
- `species.scientificName`
- `productForm`
- `batchNumber`
- `productionDate`

### 5.2 Data Types

- Dates: ISO 8601 format (YYYY-MM-DD or full datetime)
- Numbers: Floating point for measurements
- Strings: UTF-8 encoding
- Booleans: true/false

### 5.3 Units

Standard units MUST be used:
- Weight: grams (g), kilograms (kg)
- Volume: liters (L), milliliters (mL)
- Temperature: Celsius (°C)
- Energy: kilocalories (kcal), kilojoules (kJ), megajoules (MJ)
- Concentration: mg/kg, μg/kg, CFU/g

### 5.4 Value Ranges

- Protein content: 0-100 g/100g
- Moisture content: 0-100 g/100g
- Temperature: -273.15 to 1000 °C
- pH: 0-14

---

## 6. Example: Complete Product Record

```json
{
  "id": "PRODUCT-2025-001-CRICKET",
  "version": "1.0",
  "standard": "WIA-AGRI-025",
  "species": {
    "commonName": "House Cricket",
    "scientificName": "Acheta domesticus",
    "taxonomy": {
      "order": "Orthoptera",
      "family": "Gryllidae",
      "genus": "Acheta",
      "species": "domesticus"
    }
  },
  "productForm": "powder",
  "batchNumber": "BATCH-2025-001",
  "productionDate": "2025-12-15T10:30:00Z",
  "expiryDate": "2026-12-15",
  "farmInfo": {
    "farmId": "FARM-KR-001",
    "farmName": "Seoul Cricket Farm",
    "location": {
      "country": "South Korea",
      "region": "Seoul",
      "coordinates": {
        "latitude": 37.5665,
        "longitude": 126.9780
      }
    },
    "certifications": ["WIA-AGRI-025", "ISO22000", "HACCP"]
  },
  "nutrition": {
    "macronutrients": {
      "protein": {
        "value": 65.5,
        "unit": "g/100g",
        "method": "Kjeldahl",
        "testDate": "2025-12-16"
      },
      "fat": {
        "value": 15.2,
        "unit": "g/100g",
        "saturated": 4.5,
        "monounsaturated": 3.8,
        "polyunsaturated": 6.9,
        "omega3": 1.2,
        "omega6": 5.7
      },
      "carbohydrates": {
        "total": 8.5,
        "fiber": 5.3,
        "chitin": 4.8,
        "sugars": 0.4
      },
      "moisture": {
        "value": 5.2,
        "unit": "g/100g"
      },
      "ash": {
        "value": 5.6,
        "unit": "g/100g"
      }
    },
    "energyContent": {
      "calories": {
        "value": 450,
        "unit": "kcal/100g"
      },
      "kilojoules": {
        "value": 1884,
        "unit": "kJ/100g"
      }
    }
  },
  "safety": {
    "microbiological": {
      "totalPlateCount": {
        "value": 8500,
        "unit": "CFU/g",
        "limit": 100000,
        "passed": true,
        "testDate": "2025-12-16"
      }
    },
    "heavyMetals": {
      "lead": {
        "value": 0.08,
        "unit": "mg/kg",
        "limit": 0.3,
        "passed": true
      }
    },
    "allergens": {
      "chitin": {
        "present": true,
        "level": "medium",
        "warningRequired": true
      }
    }
  },
  "sustainability": {
    "carbonFootprint": {
      "total": {
        "value": 0.5,
        "unit": "kg CO2e/kg product"
      }
    },
    "waterUsage": {
      "total": {
        "value": 1.2,
        "unit": "L/kg product"
      }
    },
    "feedConversionRatio": {
      "value": 1.7,
      "description": "kg feed / kg insect protein",
      "feedSource": "organic vegetable waste"
    }
  },
  "certification": {
    "certified": true,
    "certificationId": "CERT-WIA-AGRI-025-2025-001",
    "standardVersion": "1.0",
    "issueDate": "2025-12-20",
    "expiryDate": "2026-12-20"
  }
}
```

---

## 7. Conclusion

This data format specification provides a comprehensive framework for representing insect protein products in a standardized, machine-readable format. It enables transparent communication of nutrition, safety, sustainability, and traceability information across the entire supply chain.

**Next Phase:** [PHASE-2-API-INTERFACE.md](PHASE-2-API-INTERFACE.md)

---

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA
World Certification Industry Association
