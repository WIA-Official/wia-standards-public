# WIA-MED-022 Nutrition Tracking Standard
## Phase 1: Data Format Specification

### Version: 1.0.0
### Status: Complete
### Last Updated: 2025-01-15

---

## 1. Overview

This specification defines the standardized data formats for nutrition tracking systems compliant with WIA-MED-022.

---

## 2. Core Data Models

### 2.1 Food Item

```json
{
  "food_id": "string (UUID)",
  "name": "string",
  "name_translations": {
    "en": "string",
    "ko": "string"
  },
  "category": "string",
  "brand": "string (optional)",
  "barcode": "string (optional)",
  "serving_size": "number (grams)",
  "nutrition": {
    "calories": "number (kcal)",
    "protein": "number (g)",
    "carbohydrates": "number (g)",
    "fiber": "number (g)",
    "sugar": "number (g)",
    "fat_total": "number (g)",
    "fat_saturated": "number (g)",
    "fat_trans": "number (g)",
    "cholesterol": "number (mg)",
    "sodium": "number (mg)",
    "potassium": "number (mg)",
    "vitamins": {
      "vitamin_a": "number (μg)",
      "vitamin_c": "number (mg)",
      "vitamin_d": "number (μg)",
      "vitamin_e": "number (mg)",
      "vitamin_k": "number (μg)",
      "thiamin_b1": "number (mg)",
      "riboflavin_b2": "number (mg)",
      "niacin_b3": "number (mg)",
      "vitamin_b6": "number (mg)",
      "folate_b9": "number (μg)",
      "vitamin_b12": "number (μg)"
    },
    "minerals": {
      "calcium": "number (mg)",
      "iron": "number (mg)",
      "magnesium": "number (mg)",
      "phosphorus": "number (mg)",
      "zinc": "number (mg)",
      "copper": "number (mg)",
      "manganese": "number (mg)",
      "selenium": "number (μg)",
      "iodine": "number (μg)"
    }
  },
  "allergens": ["string"],
  "verified": "boolean",
  "data_source": "string",
  "created_at": "ISO8601 datetime",
  "updated_at": "ISO8601 datetime"
}
```

### 2.2 Meal Entry

```json
{
  "meal_id": "string (UUID)",
  "user_id": "string (UUID)",
  "datetime": "ISO8601 datetime",
  "meal_type": "enum (breakfast|lunch|dinner|snack)",
  "items": [
    {
      "food_id": "string (UUID)",
      "quantity": "number",
      "unit": "string (g|ml|oz|cup|tbsp|tsp)",
      "notes": "string (optional)"
    }
  ],
  "photo_url": "string (optional)",
  "location": "string (optional)",
  "notes": "string (optional)",
  "nutrition_summary": {
    "calories": "number",
    "protein": "number",
    "carbohydrates": "number",
    "fat": "number"
  },
  "created_at": "ISO8601 datetime",
  "updated_at": "ISO8601 datetime"
}
```

### 2.3 User Profile

```json
{
  "user_id": "string (UUID)",
  "email": "string",
  "personal_info": {
    "age": "number",
    "gender": "enum (male|female|other)",
    "height": "number (cm)",
    "weight": "number (kg)",
    "activity_level": "enum (sedentary|light|moderate|active|very_active)"
  },
  "goals": {
    "target_weight": "number (kg, optional)",
    "goal_type": "enum (lose|gain|maintain)",
    "daily_calories": "number",
    "macros": {
      "protein": "number (g)",
      "carbohydrates": "number (g)",
      "fat": "number (g)"
    }
  },
  "allergies": ["string"],
  "medical_conditions": ["string"],
  "medications": ["string"],
  "dietary_preferences": "enum (omnivore|vegetarian|vegan|keto|paleo|mediterranean)",
  "created_at": "ISO8601 datetime",
  "updated_at": "ISO8601 datetime"
}
```

### 2.4 Nutrition Goals

```json
{
  "goal_id": "string (UUID)",
  "user_id": "string (UUID)",
  "goal_type": "enum (weight_loss|muscle_gain|maintenance|health)",
  "target_metrics": {
    "weight": {
      "current": "number (kg)",
      "target": "number (kg)",
      "weekly_change": "number (kg)"
    },
    "daily_calories": "number",
    "macros": {
      "protein_g": "number",
      "protein_percent": "number",
      "carbs_g": "number",
      "carbs_percent": "number",
      "fat_g": "number",
      "fat_percent": "number"
    },
    "fiber_g": "number",
    "water_ml": "number"
  },
  "start_date": "ISO8601 date",
  "target_date": "ISO8601 date (optional)",
  "status": "enum (active|completed|abandoned)",
  "created_at": "ISO8601 datetime"
}
```

---

## 3. Enumerated Types

### 3.1 Meal Types
- `breakfast`: Morning meal
- `lunch`: Midday meal
- `dinner`: Evening meal
- `snack`: Between-meal snack

### 3.2 Activity Levels
- `sedentary`: Little to no exercise (BMR × 1.2)
- `light`: Light exercise 1-3 days/week (BMR × 1.375)
- `moderate`: Moderate exercise 3-5 days/week (BMR × 1.55)
- `active`: Hard exercise 6-7 days/week (BMR × 1.725)
- `very_active`: Very hard exercise & physical job (BMR × 1.9)

### 3.3 Goal Types
- `weight_loss`: Reduce body weight
- `muscle_gain`: Increase muscle mass
- `maintenance`: Maintain current weight
- `health`: General health improvement

---

## 4. Units of Measurement

### 4.1 Standard Units
- **Energy**: kcal (kilocalories)
- **Mass**: g (grams), mg (milligrams), μg (micrograms)
- **Volume**: mL (milliliters), L (liters)
- **Length**: cm (centimeters)

### 4.2 Conversion Factors
```
1 oz = 28.35 g
1 cup = 240 mL
1 tbsp = 15 mL
1 tsp = 5 mL
1 lb = 0.453592 kg
```

---

## 5. Validation Rules

### 5.1 Nutrition Data
- Calories: 0 ≤ value ≤ 9,000 kcal/100g
- Protein: 0 ≤ value ≤ 100 g/100g
- Carbohydrates: 0 ≤ value ≤ 100 g/100g
- Fat: 0 ≤ value ≤ 100 g/100g
- Macronutrient sum ≤ 100g/100g (for solid foods)

### 5.2 Energy Calculation
```
calories = (protein × 4) + (carbohydrates × 4) + (fat × 9)
Tolerance: ±5%
```

### 5.3 Date/Time
- All timestamps in ISO 8601 format: `YYYY-MM-DDTHH:MM:SSZ`
- UTC timezone for storage
- Local timezone for display

---

## 6. Data Integrity

### 6.1 Required Fields
- Food: `food_id`, `name`, `serving_size`, `calories`
- Meal: `meal_id`, `user_id`, `datetime`, `meal_type`, `items`
- User: `user_id`, `email`, `personal_info`

### 6.2 Referential Integrity
- `meal.items[].food_id` → `food.food_id`
- `meal.user_id` → `user.user_id`
- `goal.user_id` → `user.user_id`

---

## 7. Versioning

### 7.1 Semantic Versioning
Format: `MAJOR.MINOR.PATCH`
- MAJOR: Incompatible changes
- MINOR: Backwards-compatible functionality
- PATCH: Backwards-compatible bug fixes

### 7.2 Deprecation Policy
- Deprecated fields supported for 12 months
- Warning headers included in API responses
- Migration guide provided

---

© 2025 WIA Standards - Benefit All Humanity (弘益人間)
