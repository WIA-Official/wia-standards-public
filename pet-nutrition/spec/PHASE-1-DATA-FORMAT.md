# WIA-PET-009: Phase 1 - Data Format Specification

**Version:** 1.0  
**Status:** Final  
**Last Updated:** 2025-12-25  
**Category:** PET  
**Primary Color:** Amber #F59E0B

---

## Table of Contents

1. [Overview](#overview)
2. [Core Data Models](#core-data-models)
3. [JSON Schema Specifications](#json-schema-specifications)
4. [Data Types and Validation](#data-types-and-validation)
5. [Examples](#examples)

---

## Overview

Phase 1 of the WIA-PET-009 standard defines the foundational data structures for pet nutrition management. All implementations MUST support these core data models to achieve Level 1 certification.

### Design Principles

- **Interoperability:** JSON format ensures cross-platform compatibility
- **Extensibility:** Optional fields allow future enhancement without breaking changes
- **Validation:** JSON Schema provides automated validation
- **Human-Readable:** Clear field names and structure for developer accessibility

### Compliance Requirements

**Level 1 (Basic) Certification requires:**
- Support for all REQUIRED fields in core entities
- JSON Schema validation for all data
- Proper data type usage (ISO 8601 dates, standardized units)
- UTF-8 encoding

---

## Core Data Models

### 1. Pet Profile

The Pet Profile is the central entity linking all nutrition-related data.

**Required Fields:**
- `petId` (string): Unique identifier (UUID v4 recommended)
- `species` (enum): dog, cat, bird, rabbit, reptile, ferret, other
- `birthDate` (string): ISO 8601 date (YYYY-MM-DD)
- `sex` (enum): male, female, neutered_male, spayed_female
- `weight.current` (number): Current weight in kg
- `weight.unit` (string): Always "kg"

**Optional Fields:**
- `name` (string): Pet's name
- `breed` (string): Breed or mix description
- `microchipId` (string): ISO 11784/11785 microchip number
- `weight.ideal` (number): Target weight in kg
- `weight.history` (array): Historical weight measurements
- `activityLevel` (enum): sedentary, moderate, active, very_active
- `healthConditions` (array): Medical conditions
- `allergies` (array): Known allergens
- `medications` (array): Current medications

**JSON Schema:**

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "PetProfile",
  "type": "object",
  "required": ["petId", "species", "birthDate", "sex", "weight"],
  "properties": {
    "petId": {
      "type": "string",
      "pattern": "^PET-[0-9]{4}-[A-Z0-9]{4,20}$",
      "description": "Unique pet identifier"
    },
    "ownerId": {
      "type": "string",
      "pattern": "^OWNER-[0-9]{4}-[0-9]{6,12}$"
    },
    "name": {
      "type": "string",
      "minLength": 1,
      "maxLength": 100
    },
    "species": {
      "type": "string",
      "enum": ["dog", "cat", "bird", "rabbit", "reptile", "ferret", "other"]
    },
    "breed": {
      "type": "string",
      "maxLength": 200
    },
    "birthDate": {
      "type": "string",
      "format": "date",
      "description": "ISO 8601 date (YYYY-MM-DD)"
    },
    "sex": {
      "type": "string",
      "enum": ["male", "female", "neutered_male", "spayed_female"]
    },
    "microchipId": {
      "type": "string",
      "pattern": "^[0-9]{15}$",
      "description": "15-digit ISO microchip number"
    },
    "weight": {
      "type": "object",
      "required": ["current", "unit"],
      "properties": {
        "current": {
          "type": "number",
          "minimum": 0.1,
          "maximum": 200,
          "description": "Current weight in kg"
        },
        "ideal": {
          "type": "number",
          "minimum": 0.1,
          "maximum": 200
        },
        "unit": {
          "type": "string",
          "const": "kg"
        },
        "lastUpdated": {
          "type": "string",
          "format": "date-time"
        },
        "history": {
          "type": "array",
          "items": {
            "type": "object",
            "required": ["date", "weight"],
            "properties": {
              "date": {
                "type": "string",
                "format": "date-time"
              },
              "weight": {
                "type": "number",
                "minimum": 0.1
              },
              "bcs": {
                "type": "integer",
                "minimum": 1,
                "maximum": 9,
                "description": "Body Condition Score (9-point scale)"
              }
            }
          }
        }
      }
    },
    "activityLevel": {
      "type": "string",
      "enum": ["sedentary", "moderate", "active", "very_active"]
    },
    "healthConditions": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["condition"],
        "properties": {
          "condition": {
            "type": "string"
          },
          "diagnosedDate": {
            "type": "string",
            "format": "date"
          },
          "severity": {
            "type": "string",
            "enum": ["mild", "moderate", "severe"]
          },
          "medications": {
            "type": "array",
            "items": {
              "type": "string"
            }
          }
        }
      }
    },
    "allergies": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["allergen"],
        "properties": {
          "allergen": {
            "type": "string"
          },
          "type": {
            "type": "string",
            "enum": ["food_ingredient", "protein_source", "additive", "environmental"]
          },
          "severity": {
            "type": "string",
            "enum": ["mild", "moderate", "severe", "life_threatening"]
          },
          "symptoms": {
            "type": "array",
            "items": {
              "type": "string"
            }
          }
        }
      }
    }
  }
}
```

### 2. Nutritional Requirements

Calculated based on pet profile, this entity specifies daily nutrient needs.

**Required Fields:**
- `petId` (string): Link to pet profile
- `dailyCaloricNeeds` (number): Total kcal/day
- `macronutrients.protein.minimum` (number): Grams per day
- `macronutrients.fat.minimum` (number): Grams per day

**JSON Schema:**

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "NutritionalRequirements",
  "type": "object",
  "required": ["petId", "dailyCaloricNeeds", "macronutrients"],
  "properties": {
    "petId": {
      "type": "string"
    },
    "calculationMethod": {
      "type": "string",
      "enum": ["RER_based", "DER_based", "custom"],
      "description": "Method used to calculate requirements"
    },
    "dailyCaloricNeeds": {
      "type": "number",
      "minimum": 50,
      "maximum": 10000,
      "description": "Total daily energy requirement in kcal"
    },
    "macronutrients": {
      "type": "object",
      "required": ["protein", "fat"],
      "properties": {
        "protein": {
          "type": "object",
          "required": ["minimum"],
          "properties": {
            "minimum": {
              "type": "number",
              "minimum": 0,
              "description": "Minimum protein in grams/day"
            },
            "maximum": {
              "type": "number"
            },
            "percentage": {
              "type": "number",
              "minimum": 0,
              "maximum": 100,
              "description": "% of total calories from protein"
            }
          }
        },
        "fat": {
          "type": "object",
          "required": ["minimum"],
          "properties": {
            "minimum": {
              "type": "number",
              "minimum": 0
            },
            "maximum": {
              "type": "number"
            },
            "percentage": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            }
          }
        },
        "carbohydrates": {
          "type": "object",
          "properties": {
            "minimum": {
              "type": "number",
              "minimum": 0
            },
            "maximum": {
              "type": "number"
            },
            "percentage": {
              "type": "number",
              "minimum": 0,
              "maximum": 100
            }
          }
        }
      }
    },
    "micronutrients": {
      "type": "object",
      "properties": {
        "vitamins": {
          "type": "object",
          "additionalProperties": {
            "type": "object",
            "properties": {
              "amount": {
                "type": "number"
              },
              "unit": {
                "type": "string",
                "enum": ["IU", "mg", "mcg"]
              }
            }
          }
        },
        "minerals": {
          "type": "object",
          "additionalProperties": {
            "type": "object",
            "properties": {
              "amount": {
                "type": "number"
              },
              "unit": {
                "type": "string",
                "enum": ["mg", "g", "%"]
              }
            }
          }
        }
      }
    }
  }
}
```

### 3. Food Product

Product information for pet foods.

**Required Fields:**
- `productId` (string): Unique product identifier
- `name` (string): Product name
- `manufacturer` (string): Manufacturer name
- `productType` (enum): Product category
- `targetSpecies` (array): Intended species
- `guaranteedAnalysis.crudeProtein.minimum` (number)
- `guaranteedAnalysis.crudeFat.minimum` (number)
- `guaranteedAnalysis.crudeFiber.maximum` (number)
- `guaranteedAnalysis.moisture.maximum` (number)

### 4. Diet Plan

Prescribed feeding plan for a pet.

**Required Fields:**
- `planId` (string): Unique plan identifier
- `petId` (string): Pet this plan is for
- `startDate` (string): ISO 8601 date
- `goal` (enum): maintenance, weight_loss, weight_gain, therapeutic, growth
- `dailyMeals` (array): Meal schedule with portions

### 5. Feeding Log

Record of actual food consumption.

**Required Fields:**
- `logId` (string): Unique log identifier
- `petId` (string): Pet identifier
- `timestamp` (string): ISO 8601 datetime
- `foodsConsumed` (array): What and how much was eaten

---

## Data Types and Validation

### Standard Units

**Weight:** kilograms (kg) - all weight measurements MUST use kg
**Energy:** kilocalories (kcal) - for metabolizable energy
**Volume:** milliliters (mL) or liters (L)
**Temperature:** Celsius (°C)

### Date/Time Formats

**Date:** ISO 8601 (YYYY-MM-DD)  
Example: `"2025-12-25"`

**DateTime:** ISO 8601 with timezone (YYYY-MM-DDTHH:MM:SSZ)  
Example: `"2025-12-25T14:30:00Z"`

### Identifiers

**Pet ID:** `PET-YYYY-XXXX` where YYYY is year, XXXX is alphanumeric  
Example: `"PET-2025-BELLA-7821"`

**Owner ID:** `OWNER-YYYY-NNNNNN` where NNNNNN is 6-12 digits  
Example: `"OWNER-2025-567890"`

**Product ID:** `PROD-XXXX` format  
Example: `"PROD-SENIOR-FORMULA-001"`

---

## Examples

### Complete Pet Profile

```json
{
  "petId": "PET-2025-BELLA-7821",
  "ownerId": "OWNER-2025-123456",
  "name": "Bella",
  "species": "dog",
  "breed": "Golden Retriever",
  "birthDate": "2017-08-15",
  "sex": "female_spayed",
  "microchipId": "985112345678901",
  "weight": {
    "current": 35.2,
    "ideal": 30.0,
    "unit": "kg",
    "lastUpdated": "2025-12-15T10:30:00Z",
    "history": [
      {
        "date": "2025-11-15T09:00:00Z",
        "weight": 35.8,
        "bcs": 7
      },
      {
        "date": "2025-10-15T09:00:00Z",
        "weight": 36.2,
        "bcs": 7
      }
    ]
  },
  "activityLevel": "moderate",
  "healthConditions": [
    {
      "condition": "osteoarthritis",
      "diagnosedDate": "2024-06-10",
      "severity": "mild",
      "medications": ["carprofen"]
    }
  ],
  "allergies": [
    {
      "allergen": "chicken protein",
      "type": "food_ingredient",
      "severity": "moderate",
      "symptoms": ["itching", "digestive_upset"]
    }
  ]
}
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)  
Standard ID: WIA-PET-009 | Version: 1.0 | Phase: 1
