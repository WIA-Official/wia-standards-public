# WIA-PET-009: Pet Nutrition Standard 🍖

> **International Standard for Pet Nutrition Management**
> Standardized framework for pet dietary requirements, food labeling, allergy tracking, and personalized diet plans.

## Overview

The **WIA-PET-009 Pet Nutrition Standard** provides a comprehensive, interoperable framework for managing pet nutritional data across the entire ecosystem—from veterinary prescriptions to smart feeding devices. This standard enables:

- **Veterinarians**: Evidence-based dietary recommendations and therapeutic diet management
- **Pet Owners**: Personalized nutrition plans and allergy tracking
- **Pet Food Manufacturers**: Standardized labeling and nutritional transparency
- **Smart Device Makers**: Integration with automated feeding systems

---

## Table of Contents

1. [Features](#features)
2. [Use Cases](#use-cases)
3. [Data Models](#data-models)
4. [Getting Started](#getting-started)
5. [Resources](#resources)
6. [Compliance](#compliance)
7. [Contributing](#contributing)
8. [License](#license)

---

## Features

### 📊 Nutritional Requirements Database
- Species-specific nutritional profiles (dogs, cats, birds, reptiles, etc.)
- Breed-specific adjustments (large breeds, toy breeds, brachycephalic breeds)
- Life-stage requirements (puppy/kitten, adult, senior, pregnant/lactating)
- Activity-level modifications (sedentary, moderate, active, working)
- Medical condition adjustments (kidney disease, diabetes, allergies, etc.)

### 🏷️ Ingredient Analysis & Labeling
- Detailed ingredient composition with nutritional breakdown
- Guaranteed analysis (crude protein, fat, fiber, moisture)
- Micronutrient content (vitamins, minerals, amino acids)
- Caloric density and feeding guidelines
- Ingredient sourcing and quality certifications
- Allergen warnings and cross-contamination notices

### 🩺 Allergy & Sensitivity Tracking
- Individual allergy profiles per pet
- Severity ratings (mild, moderate, severe, life-threatening)
- Reaction documentation (symptoms, onset time, duration)
- Cross-reactivity warnings
- Elimination diet tracking
- Food challenge protocols

### 🍽️ Diet Planning & Recommendations
- Personalized meal planning based on:
  - Current health status
  - Weight goals (loss, maintenance, gain)
  - Activity level
  - Medical restrictions
  - Owner preferences (budget, availability)
- Recipe suggestions (homemade, commercial, hybrid)
- Portion calculations
- Transition schedules for diet changes

### ⚖️ Weight Management System
- Body Condition Score (BCS) tracking
- Weight trend analysis with charts
- Caloric intake monitoring
- Ideal weight range calculations
- Weight goal setting and progress tracking
- Metabolic rate estimation

### 🤖 Smart Feeding Integration
- Automated feeder compatibility
- Scheduled feeding with portion control
- Real-time feeding logs
- Adjustment alerts (health changes, activity changes)
- Multi-pet household management
- Remote monitoring and notifications

---

## Use Cases

### 1. Veterinary Prescription Diets
**Scenario**: A dog is diagnosed with chronic kidney disease.

**Workflow**:
1. Veterinarian creates a therapeutic diet prescription in WIA-PET-009 format
2. Prescription includes:
   - Reduced protein (0.3-0.5g/kg body weight)
   - Low phosphorus (<0.4% dry matter)
   - Omega-3 fatty acid supplementation
   - Increased B-vitamin content
3. Pet owner receives prescription via mobile app
4. Smart feeder automatically portions appropriate amounts
5. Weight and consumption tracked weekly
6. Veterinarian reviews progress remotely

### 2. Allergy Management
**Scenario**: A cat develops food allergies (itching, digestive issues).

**Workflow**:
1. Owner logs symptoms in WIA-PET-009 compatible app
2. Veterinarian recommends elimination diet
3. System suggests hypoallergenic foods (novel protein or hydrolyzed)
4. Owner tracks food trials and reactions
5. Allergen identified (e.g., chicken protein)
6. System filters all products containing chicken
7. Safe food recommendations provided

### 3. Weight Loss Program
**Scenario**: An overweight Labrador needs to lose 10 pounds.

**Workflow**:
1. Current weight: 95 lbs, Target weight: 85 lbs
2. System calculates:
   - Current caloric needs: ~1,800 kcal/day
   - Weight loss calories: ~1,400 kcal/day (22% reduction)
   - Timeline: 16-20 weeks (0.5-1 lb/week loss)
3. Personalized feeding plan generated
4. Smart feeder dispenses precise portions
5. Weekly weigh-ins tracked
6. Adjustments made based on progress

### 4. Multi-Pet Household
**Scenario**: Household with a diabetic cat and a healthy dog.

**Workflow**:
1. Each pet has individual nutritional profile
2. Cat requires:
   - Low-carbohydrate diet (<10% carbs)
   - High protein (>40%)
   - Timed feeding with insulin
3. Dog requires:
   - Standard adult maintenance diet
   - Free feeding acceptable
4. Smart feeders recognize individual pets (RFID/facial recognition)
5. Each receives appropriate food and portions
6. Consumption monitored separately

---

## Data Models

### Core Entities

```json
{
  "petProfile": {
    "petId": "string",
    "species": "enum [dog, cat, bird, rabbit, reptile, other]",
    "breed": "string",
    "birthDate": "ISO 8601 date",
    "sex": "enum [male, female, neutered_male, spayed_female]",
    "weight": {
      "current": "number (kg)",
      "ideal": "number (kg)",
      "history": [
        {
          "date": "ISO 8601 datetime",
          "weight": "number (kg)",
          "bcs": "integer (1-9)"
        }
      ]
    },
    "activityLevel": "enum [sedentary, moderate, active, very_active]",
    "healthConditions": ["string"],
    "allergies": ["string"],
    "medications": ["string"]
  },

  "nutritionalRequirements": {
    "dailyCaloricNeeds": "number (kcal)",
    "macronutrients": {
      "protein": {
        "minimum": "number (g/day)",
        "maximum": "number (g/day)",
        "percentage": "number (%)"
      },
      "fat": {
        "minimum": "number (g/day)",
        "maximum": "number (g/day)",
        "percentage": "number (%)"
      },
      "carbohydrates": {
        "minimum": "number (g/day)",
        "maximum": "number (g/day)",
        "percentage": "number (%)"
      }
    },
    "micronutrients": {
      "vitamins": {
        "vitaminA": "number (IU/day)",
        "vitaminD": "number (IU/day)",
        "vitaminE": "number (IU/day)",
        "thiamine": "number (mg/day)",
        "riboflavin": "number (mg/day)"
      },
      "minerals": {
        "calcium": "number (mg/day)",
        "phosphorus": "number (mg/day)",
        "magnesium": "number (mg/day)",
        "sodium": "number (mg/day)",
        "potassium": "number (mg/day)"
      }
    }
  },

  "foodProduct": {
    "productId": "string",
    "name": "string",
    "manufacturer": "string",
    "productType": "enum [dry_kibble, wet_food, raw, freeze_dried, dehydrated, treats]",
    "targetSpecies": ["string"],
    "lifeStage": ["enum [puppy, kitten, adult, senior, all_life_stages]"],
    "ingredients": [
      {
        "name": "string",
        "percentage": "number",
        "order": "integer"
      }
    ],
    "guaranteedAnalysis": {
      "crudeProtein": {
        "minimum": "number (%)"
      },
      "crudeFat": {
        "minimum": "number (%)"
      },
      "crudeFiber": {
        "maximum": "number (%)"
      },
      "moisture": {
        "maximum": "number (%)"
      }
    },
    "caloricContent": {
      "kcalPerKg": "number",
      "kcalPerCup": "number",
      "kcalPerCan": "number"
    },
    "allergenWarnings": ["string"],
    "certifications": ["string"]
  },

  "dietPlan": {
    "planId": "string",
    "petId": "string",
    "createdBy": "string (veterinarian/owner)",
    "startDate": "ISO 8601 date",
    "endDate": "ISO 8601 date (optional)",
    "goal": "enum [maintenance, weight_loss, weight_gain, therapeutic, growth]",
    "dailyMeals": [
      {
        "mealTime": "string (HH:MM)",
        "foods": [
          {
            "productId": "string",
            "portion": "number (grams or cups)",
            "calories": "number (kcal)"
          }
        ],
        "supplements": [
          {
            "name": "string",
            "dosage": "string"
          }
        ]
      }
    ],
    "totalDailyCalories": "number (kcal)",
    "notes": "string"
  },

  "feedingLog": {
    "logId": "string",
    "petId": "string",
    "timestamp": "ISO 8601 datetime",
    "mealType": "enum [breakfast, lunch, dinner, snack]",
    "foodsConsumed": [
      {
        "productId": "string",
        "amountOffered": "number (grams)",
        "amountConsumed": "number (grams)",
        "caloriesConsumed": "number (kcal)"
      }
    ],
    "appetite": "enum [normal, increased, decreased, refused]",
    "notes": "string"
  },

  "allergyProfile": {
    "petId": "string",
    "allergens": [
      {
        "allergen": "string",
        "type": "enum [ingredient, protein_source, additive, environmental]",
        "severity": "enum [mild, moderate, severe, life_threatening]",
        "symptoms": ["string"],
        "diagnosisDate": "ISO 8601 date",
        "diagnosisMethod": "enum [elimination_diet, blood_test, intradermal_test, observation]",
        "notes": "string"
      }
    ],
    "safeIngredients": ["string"],
    "unsafeIngredients": ["string"]
  }
}
```

---

## Getting Started

### For Pet Owners

1. **Create Pet Profile**: Input your pet's basic information (species, breed, age, weight)
2. **Health Assessment**: Add any medical conditions, allergies, or medications
3. **Set Goals**: Define dietary goals (maintenance, weight loss, etc.)
4. **Review Recommendations**: System generates personalized diet plan
5. **Track Progress**: Log meals, weight, and health observations

### For Veterinarians

1. **Import Client Data**: Integrate with existing practice management systems
2. **Prescribe Diets**: Create therapeutic diet plans using WIA-PET-009 format
3. **Monitor Compliance**: Review feeding logs and weight trends
4. **Adjust Plans**: Modify recommendations based on patient progress

### For Pet Food Manufacturers

1. **Product Registration**: Submit product data in WIA-PET-009 format
2. **Nutritional Analysis**: Provide detailed ingredient and nutrient breakdowns
3. **Compliance Verification**: Ensure labeling meets standard requirements
4. **API Integration**: Enable real-time product availability and updates

### For Device Manufacturers

1. **API Integration**: Connect smart feeders to WIA-PET-009 endpoints
2. **Portion Control**: Implement precise dispensing based on diet plans
3. **Consumption Tracking**: Log actual food consumed vs. offered
4. **Alerts & Notifications**: Notify owners of feeding events, low food, etc.

---

## Resources

### 📚 Documentation
- **[Interactive Simulator](simulator/index.html)**: Test nutrition calculations and diet planning
- **[English eBook](ebook/en/index.html)**: Comprehensive 8-chapter guide
- **[Korean eBook](ebook/ko/index.html)**: Full Korean translation
- **[Technical Specifications](spec/PHASE-1-DATA-FORMAT.md)**: Detailed API and data format specs

### 🔧 Technical Specs
1. **[PHASE 1: Data Format](spec/PHASE-1-DATA-FORMAT.md)** - JSON schemas and data structures
2. **[PHASE 2: API Specs](spec/PHASE-2-API-SPECS.md)** - RESTful API endpoints and authentication
3. **[PHASE 3: Implementation](spec/PHASE-3-IMPLEMENTATION.md)** - Best practices and code examples
4. **[PHASE 4: Integration](spec/PHASE-4-INTEGRATION.md)** - System integration and certification

### 🌐 Korean Documentation
- **[PHASE 1: 데이터 형식](spec/ko/PHASE-1-DATA-FORMAT.md)**
- **[PHASE 2: API 사양](spec/ko/PHASE-2-API-SPECS.md)**
- **[PHASE 3: 구현](spec/ko/PHASE-3-IMPLEMENTATION.md)**
- **[PHASE 4: 통합](spec/ko/PHASE-4-INTEGRATION.md)**

---

## Compliance

### Regulatory Alignment
This standard aligns with:
- **AAFCO** (Association of American Feed Control Officials) nutrient profiles
- **FEDIAF** (European Pet Food Industry Federation) guidelines
- **NRC** (National Research Council) nutrient requirements
- **FDA** pet food labeling regulations

### Certification Levels
- **Level 1: Basic Compliance** - Supports core data models and nutritional requirements
- **Level 2: Advanced Features** - Includes allergy tracking and diet planning
- **Level 3: Full Integration** - Complete API implementation with smart device support

---

## Contributing

We welcome contributions from:
- Veterinary nutritionists
- Pet food scientists
- Software developers
- Pet owners and advocates

### How to Contribute
1. Review existing standards and documentation
2. Submit improvement proposals via GitHub issues
3. Provide real-world use case examples
4. Test implementations and report feedback

---

## License

Copyright © 2025 WIA (World Certification Industry Association)

This standard is released under the **Creative Commons Attribution 4.0 International License (CC BY 4.0)**.

You are free to:
- **Share**: Copy and redistribute the material in any medium or format
- **Adapt**: Remix, transform, and build upon the material for any purpose

Under the following terms:
- **Attribution**: You must give appropriate credit to WIA

---

## Contact

- **Website**: https://wia.org/standards/pet-nutrition
- **Email**: pet-nutrition@wia.org
- **GitHub**: https://github.com/WIA-Official/wia-standards

---

<div align="center">

**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**

*WIA-PET-009: Pet Nutrition Standard*
*Version 1.0 | Category: PET | Color: Amber #F59E0B*

</div>
