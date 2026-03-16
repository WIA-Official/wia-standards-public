# WIA-TRADITIONAL-MEDICINE: Phase 3 - Protocol

## 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-01

---

## 1. Overview

Phase 3 defines standardized clinical protocols for traditional medicine practice, ensuring consistency, safety, and quality across different practitioners and settings.

---

## 2. Constitution Assessment Protocol

### 2.1 Pre-Assessment Requirements

| Requirement | Description |
|-------------|-------------|
| Patient Preparation | No food/drink 2 hours before (for tongue diagnosis) |
| Environment | Quiet room, natural or color-corrected lighting |
| Time Allocation | 30-45 minutes for comprehensive assessment |
| Documentation | Digital recording system ready |

### 2.2 Assessment Workflow

```
STEP 1: QUESTIONNAIRE ADMINISTRATION
├── Select appropriate questionnaire
│   ├── TCM: CCMQ (Constitution in Chinese Medicine Questionnaire)
│   ├── Ayurveda: Prakriti Assessment Questionnaire
│   └── Sasang: QSCCII (Questionnaire for Sasang Constitution Classification II)
├── Administer in patient's native language
├── Allow 15-20 minutes for completion
└── Calculate and record raw scores

STEP 2: PHYSICAL EXAMINATION
├── Body type assessment (frame, weight distribution)
├── Facial characteristics analysis
├── Voice quality assessment
├── Gait observation
└── Skin characteristics

STEP 3: AI-ASSISTED ANALYSIS (Optional)
├── Facial image capture (standardized protocol)
├── Voice recording (30-second sample)
├── Integration with questionnaire results
└── Confidence score generation

STEP 4: MULTI-OMICS INTEGRATION (When Available)
├── Pharmacogenomic markers (CYP450, etc.)
├── Metabolomic profile
├── Microbiome assessment
└── Constitutional biomarker panel

STEP 5: SYNTHESIS AND DOCUMENTATION
├── Integrate all data sources
├── Determine primary and secondary constitution types
├── Document confidence levels
└── Generate personalized recommendations
```

---

## 3. Four Examinations Protocol (四診)

### 3.1 Inspection Protocol (望診)

#### 3.1.1 Tongue Diagnosis Standards

**Image Capture Protocol:**

| Parameter | Requirement |
|-----------|-------------|
| Lighting | Natural daylight or 5500K color-corrected |
| Camera | Minimum 12MP, auto white-balance disabled |
| Distance | 15-20 cm from tongue |
| Duration | Capture within 15 seconds |
| Background | Neutral gray card included in frame |
| Patient Position | Seated, tongue extended naturally |
| Timing | Before eating, drinking, or brushing |

**Analysis Parameters:**

| Feature | Options | Significance |
|---------|---------|--------------|
| Body Color | Pale, Light Red, Red, Dark Red, Purple | Reflects Qi, Blood, Yin, Yang status |
| Body Shape | Thin, Normal, Swollen, Teeth-marked | Reflects body fluid and Qi status |
| Coating Color | White, Yellow, Gray, Black | Reflects pathogen and heat level |
| Coating Thickness | Thin, Normal, Thick, Peeled | Reflects pathogen depth |
| Moisture | Dry, Normal, Wet | Reflects body fluid status |

### 3.2 Auscultation Protocol (聞診)

**Voice Analysis:**
- Record 30-second speech sample
- Assess volume, clarity, strength
- Note breathing patterns during speech

**Parameters:**

| Quality | Indication |
|---------|------------|
| Weak, low volume | Qi deficiency |
| Loud, coarse | Heat, excess |
| Hoarse | Phlegm, blood stasis |
| Breathless | Lung Qi deficiency |

### 3.3 Inquiry Protocol (問診)

**Ten Questions Framework (十問):**

1. **Cold/Heat (寒熱):** Temperature preferences, fever patterns
2. **Perspiration (汗):** Spontaneous, night sweats, absence
3. **Head/Body (頭身):** Headache, body aches, dizziness
4. **Chest/Abdomen (胸腹):** Fullness, pain, palpitations
5. **Appetite/Thirst (食慾渴飲):** Hunger, taste, thirst
6. **Bowel/Urination (二便):** Frequency, quality, color
7. **Sleep (睡眠):** Quality, dreams, waking patterns
8. **Ears/Eyes (耳目):** Hearing, vision, discharges
9. **Previous Illness (既往病史):** Medical history
10. **Cause of Illness (發病原因):** Triggers, onset

### 3.4 Palpation Protocol (切診)

**Pulse Diagnosis Standards:**

| Step | Protocol |
|------|----------|
| Patient Preparation | Rest 5 minutes, arm at heart level |
| Position | Practitioner uses 3 fingers (index, middle, ring) |
| Location | Radial artery: Cun, Guan, Chi positions |
| Pressure | Three levels: superficial, middle, deep |
| Duration | Minimum 60 seconds per wrist |
| Documentation | Rate, rhythm, quality for each position |

**Pulse Sensor Protocol (Optional):**

| Parameter | Specification |
|-----------|---------------|
| Sensor Type | Piezoelectric or optical |
| Sampling Rate | ≥1000 Hz |
| Channels | Minimum 3 (one per position) |
| Duration | Minimum 60 seconds |
| Output | Time-domain waveform, frequency spectrum |

---

## 4. Treatment Protocol

### 4.1 Treatment Selection Framework

```
PATTERN DIAGNOSIS
      ↓
DETERMINE TREATMENT PRINCIPLE (治法)
      ↓
SELECT BASE FORMULA
      ↓
MODIFY FOR CONSTITUTION
      ↓
ADJUST FOR INDIVIDUAL FACTORS
      ↓
SAFETY SCREENING
      ↓
PRESCRIBE AND DOCUMENT
```

### 4.2 Dosage Guidelines

| Factor | Adjustment |
|--------|------------|
| Age (child) | 1/4 to 1/2 adult dose |
| Age (elderly) | 2/3 to 3/4 adult dose |
| Constitution (weak) | Start with lower dose |
| Condition (acute) | Higher dose, shorter duration |
| Condition (chronic) | Lower dose, longer duration |

### 4.3 Treatment Duration

| Condition Type | Typical Duration | Review Interval |
|----------------|------------------|-----------------|
| Acute | 3-7 days | Daily |
| Sub-acute | 2-4 weeks | Weekly |
| Chronic | 1-3 months | Bi-weekly |
| Constitutional | Ongoing | Monthly |

---

## 5. Safety Protocol

### 5.1 Pre-Prescription Safety Checks

- [ ] Confirm patient identity
- [ ] Review medical history
- [ ] Check current medications
- [ ] Verify allergies
- [ ] Assess pregnancy/lactation status
- [ ] Run drug-herb interaction check
- [ ] Verify herb quality certification

### 5.2 Contraindication Categories

| Category | Action |
|----------|--------|
| Absolute | Do not prescribe |
| Relative | Weigh risk/benefit, modify dose |
| Conditional | Monitor closely |

### 5.3 Adverse Event Response

```
SUSPECTED ADVERSE EVENT
      ↓
STOP TREATMENT IMMEDIATELY
      ↓
ASSESS PATIENT CONDITION
      ↓
PROVIDE SUPPORTIVE CARE
      ↓
DOCUMENT ALL DETAILS
      ↓
REPORT VIA PHARMACOVIGILANCE SYSTEM
      ↓
FOLLOW UP WITH PATIENT
```

---

## 6. Quality Control Protocol

### 6.1 Herb Quality Standards

| Test | Standard |
|------|----------|
| Identity | DNA barcode, macroscopic, microscopic |
| Heavy Metals | As <3ppm, Pb <10ppm, Hg <0.5ppm, Cd <1ppm |
| Pesticides | Per pharmacopoeia limits |
| Microbial | Total count <10^5 CFU/g |
| Active Content | Per pharmacopoeia specification |

### 6.2 Documentation Standards

All clinical encounters must document:
- Date, time, practitioner ID
- Patient chief complaint
- Four examinations findings
- Pattern diagnosis with ICD-11 TM code
- Treatment principle and prescription
- Patient instructions
- Follow-up plan

---

## 7. Telehealth Protocol

### 7.1 Remote Consultation Requirements

| Component | Requirement |
|-----------|-------------|
| Video Quality | Minimum 720p, stable connection |
| Lighting | Patient well-lit for visual assessment |
| Tongue Photo | Standardized protocol (see 3.1.1) |
| Pulse | Wearable sensor or patient self-report |
| Environment | Private, quiet space for both parties |

### 7.2 Limitations of Remote Assessment

| Examination | Remote Feasibility |
|-------------|-------------------|
| Inspection | High (with good video) |
| Tongue Diagnosis | Moderate (with proper photo) |
| Auscultation | Moderate (audio quality dependent) |
| Inquiry | High |
| Pulse Palpation | Low (sensor required) |
| Abdominal Palpation | Not possible |

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association
