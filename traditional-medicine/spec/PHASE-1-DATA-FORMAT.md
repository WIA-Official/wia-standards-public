# WIA-TRADITIONAL-MEDICINE: Phase 1 - Data Format

## 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-01

---

## 1. Overview

Phase 1 establishes standardized data formats for representing traditional medicine concepts in modern healthcare information systems. These formats enable interoperability between different traditional medicine systems while preserving their unique terminology and concepts.

### 1.1 Design Principles

- **Comprehensive**: Capture the full richness of traditional diagnostic information
- **Interoperable**: Compatible with HL7 FHIR and other healthcare standards
- **Multilingual**: Preserve original terminology alongside translations
- **Extensible**: Allow addition of new traditional medicine systems
- **Validated**: Include JSON Schema validation for data quality

---

## 2. Constitutional Profile Schema

The constitutional profile captures an individual's constitution assessment across multiple traditional medicine systems.

### 2.1 Schema Definition

```json
{
  "$schema": "https://wia.live/schemas/traditional-medicine/v1.0.0/constitutional-profile",
  "type": "object",
  "required": ["profile_id", "patient_id", "timestamp"],
  "properties": {
    "profile_id": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for this profile"
    },
    "patient_id": {
      "type": "string",
      "description": "Reference to patient record"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of assessment"
    },
    "practitioner_id": {
      "type": "string",
      "description": "Reference to practitioner record"
    },
    "tcm_constitution": {
      "$ref": "#/definitions/TCMConstitution"
    },
    "ayurveda_prakriti": {
      "$ref": "#/definitions/AyurvedaPrakriti"
    },
    "sasang_constitution": {
      "$ref": "#/definitions/SasangConstitution"
    },
    "genomic_correlates": {
      "$ref": "#/definitions/GenomicCorrelates"
    },
    "metabolomic_profile": {
      "$ref": "#/definitions/MetabolomicProfile"
    },
    "microbiome_profile": {
      "$ref": "#/definitions/MicrobiomeProfile"
    }
  }
}
```

### 2.2 TCM Constitution Types

| Type | Chinese | Pinyin | Characteristics |
|------|---------|--------|-----------------|
| qi_deficiency | 氣虛質 | Qì xū zhì | Fatigue, weak voice, susceptibility to colds |
| yang_deficiency | 陽虛質 | Yáng xū zhì | Cold intolerance, cold extremities |
| yin_deficiency | 陰虛質 | Yīn xū zhì | Heat sensations, dry skin, night sweats |
| phlegm_dampness | 痰濕質 | Tán shī zhì | Overweight, greasy skin, heavy sensation |
| damp_heat | 濕熱質 | Shī rè zhì | Oily skin, irritability |
| blood_stasis | 血瘀質 | Xuě yū zhì | Dark complexion, pain tendency |
| qi_stagnation | 氣郁質 | Qì yù zhì | Emotional sensitivity, sighing |
| special_diathesis | 特稟質 | Tè bǐng zhì | Allergic tendencies |
| balanced | 平和質 | Píng hé zhì | Optimal health state |

### 2.3 Sasang Constitution Types

| Type | Korean | Hanja | Organ Tendency | Prevalence |
|------|--------|-------|----------------|------------|
| taeyang | 태양인 | 太陽人 | Strong Lung, Weak Liver | ~0.1% |
| taeeum | 태음인 | 太陰人 | Strong Liver, Weak Lung | ~50% |
| soyang | 소양인 | 少陽人 | Strong Spleen, Weak Kidney | ~25% |
| soeum | 소음인 | 少陰人 | Strong Kidney, Weak Spleen | ~25% |

---

## 3. Traditional Diagnosis Schema

### 3.1 Four Examinations Structure

```json
{
  "four_examinations": {
    "inspection": {
      "complexion": "string",
      "spirit": "string",
      "tongue": {
        "body_color": "enum[pale|light_red|red|dark_red|purple]",
        "body_shape": "enum[thin|normal|swollen|teeth_marked]",
        "coating_color": "enum[white|yellow|gray|black]",
        "coating_thickness": "enum[thin|normal|thick|peeled]",
        "moisture": "enum[dry|normal|wet]",
        "image_url": "string",
        "ai_analysis": {
          "model": "string",
          "confidence": "number",
          "findings": "array[string]"
        }
      }
    },
    "auscultation_olfaction": {
      "voice": { "quality": "string", "volume": "string" },
      "breathing": { "pattern": "string" },
      "body_odor": "string"
    },
    "inquiry": {
      "chief_complaint": "string",
      "onset": "string",
      "ten_questions": {
        "cold_heat": "string",
        "perspiration": "string",
        "appetite_thirst": "string",
        "urination_defecation": "string",
        "sleep": "string",
        "pain": "string",
        "menstruation": "string",
        "emotions": "string"
      }
    },
    "palpation": {
      "pulse": {
        "rate": "number",
        "rhythm": "enum[regular|irregular]",
        "quality": "array[string]",
        "positions": {
          "left_cun": { "quality": "string" },
          "left_guan": { "quality": "string" },
          "left_chi": { "quality": "string" },
          "right_cun": { "quality": "string" },
          "right_guan": { "quality": "string" },
          "right_chi": { "quality": "string" }
        }
      }
    }
  }
}
```

### 3.2 Pulse Qualities (28 Classical Types)

| Category | Qualities |
|----------|-----------|
| Depth | Floating (浮), Deep (沉) |
| Speed | Slow (遲), Rapid (數) |
| Strength | Forceless (虛), Forceful (實) |
| Width | Thin (細), Big (大) |
| Length | Short (短), Long (長) |
| Rhythm | Intermittent (結代), Irregular (促) |
| Quality | Slippery (滑), Wiry (弦), Choppy (澀) |

---

## 4. Herbal Medicine Database Schema

### 4.1 Herb Entity Structure

```json
{
  "herbal_medicine": {
    "herb_id": "string",
    "names": {
      "scientific": "string",
      "chinese": "string",
      "pinyin": "string",
      "korean": "string",
      "japanese": "string",
      "sanskrit": "string",
      "common_en": "string"
    },
    "taxonomy": {
      "kingdom": "string",
      "family": "string",
      "genus": "string",
      "species": "string",
      "dna_barcode": "string"
    },
    "traditional_properties": {
      "taste": "array[enum]",
      "temperature": "enum[hot|warm|neutral|cool|cold]",
      "meridians": "array[string]",
      "actions": "array[string]",
      "indications": "array[string]",
      "contraindications": "array[string]"
    },
    "modern_pharmacology": {
      "active_compounds": "array[CompoundObject]",
      "network_pharmacology": {
        "targets": "array[string]",
        "pathways": "array[string]",
        "diseases": "array[string]"
      }
    },
    "safety": {
      "toxicity_class": "enum[non_toxic|low|moderate|high]",
      "max_daily_dose_g": "number",
      "pregnancy_category": "enum[A|B|C|D|X]",
      "drug_interactions": "array[InteractionObject]",
      "adverse_reactions": "array[string]"
    },
    "quality_standards": {
      "pharmacopoeia": "array[string]",
      "marker_compounds": "array[string]",
      "authentication_method": "array[string]"
    }
  }
}
```

### 4.2 Taste Classifications

| Taste | Chinese | Effect |
|-------|---------|--------|
| Sweet | 甘 | Tonifying, harmonizing |
| Bitter | 苦 | Clearing heat, drying dampness |
| Sour | 酸 | Astringent, consolidating |
| Pungent | 辛 | Dispersing, moving qi |
| Salty | 鹹 | Softening, descending |
| Bland | 淡 | Draining dampness |
| Astringent | 澀 | Consolidating, stopping leakage |

---

## 5. ICD-11 Traditional Medicine Codes

### 5.1 Pattern Codes (TM1:S)

| Code | Pattern (English) | Pattern (Chinese) |
|------|-------------------|-------------------|
| TM1:SA00 | Yin deficiency pattern | 陰虛證 |
| TM1:SB00 | Yang deficiency pattern | 陽虛證 |
| TM1:SC00 | Qi deficiency pattern | 氣虛證 |
| TM1:SD00 | Blood deficiency pattern | 血虛證 |
| TM1:SE00 | Qi stagnation pattern | 氣滯證 |
| TM1:SF00 | Blood stasis pattern | 血瘀證 |
| TM1:SG00 | Phlegm pattern | 痰證 |
| TM1:SH00 | Heat pattern | 熱證 |
| TM1:SI00 | Cold pattern | 寒證 |
| TM1:SJ00 | Dampness pattern | 濕證 |

---

## 6. Validation and Quality

### 6.1 Schema Validation

All data must pass JSON Schema validation before submission:

```bash
npm install @wia/tm-validator
wia-tm-validate constitution.json
```

### 6.2 Required Fields

| Data Type | Required Fields |
|-----------|-----------------|
| Constitutional Profile | profile_id, patient_id, timestamp |
| Traditional Diagnosis | diagnosis_id, patient_id, pattern_diagnosis |
| Herbal Medicine | herb_id, names.scientific, traditional_properties |

---

## References

- WHO ICD-11 Traditional Medicine Module
- ISO/TC 249 Traditional Chinese Medicine Standards
- Constitution in Chinese Medicine Questionnaire (CCMQ)
- Questionnaire for Sasang Constitution Classification II (QSCCII)

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
