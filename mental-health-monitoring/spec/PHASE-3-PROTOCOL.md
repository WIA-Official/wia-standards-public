# WIA-MENTAL-HEALTH: Phase 3 - Protocol Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the communication protocols, assessment procedures, and treatment guidelines for implementing the WIA-MENTAL-HEALTH standard. The core protocol principle is **evidence-based, neuroplasticity-centered care**.

## 2. Assessment Protocols

### 2.1 Standardized Assessment Battery

#### 2.1.1 Core Screening Protocol

```
┌─────────────────────────────────────────────────────────────┐
│  WIA-MENTAL-HEALTH Core Assessment Protocol                 │
├─────────────────────────────────────────────────────────────┤
│  STEP 1: Initial Screening (5-10 minutes)                   │
│  ├── PHQ-2 (Depression Quick Screen)                        │
│  ├── GAD-2 (Anxiety Quick Screen)                           │
│  └── PC-PTSD-5 (Trauma Quick Screen)                        │
│                                                             │
│  STEP 2: Full Assessment (if screens positive) (15-20 min)  │
│  ├── PHQ-9 (Depression Full)                                │
│  ├── GAD-7 (Anxiety Full)                                   │
│  ├── PCL-5 (PTSD Full - if trauma screen positive)          │
│  └── WHO-5 (Wellbeing)                                      │
│                                                             │
│  STEP 3: Risk Assessment (2-5 minutes)                      │
│  ├── Columbia Suicide Severity Rating Scale (C-SSRS)        │
│  └── Safety Planning (if indicated)                         │
│                                                             │
│  STEP 4: Functional Assessment (5-10 minutes)               │
│  ├── WHODAS 2.0 (Disability)                                │
│  └── Quality of Life measure                                │
└─────────────────────────────────────────────────────────────┘
```

#### 2.1.2 Assessment Frequency Protocol

| Clinical Status | Assessment Frequency | Instruments |
|-----------------|---------------------|-------------|
| Acute Crisis | Daily | PHQ-2, C-SSRS |
| Active Treatment | Weekly | PHQ-9, GAD-7 |
| Stable | Monthly | PHQ-9, GAD-7, WHO-5 |
| Maintenance | Quarterly | Full battery |
| Remission | Semi-annually | Full battery |

### 2.2 Digital Phenotyping Protocol

#### 2.2.1 Passive Data Collection

```json
{
  "collection_protocol": {
    "mobility": {
      "gps_sampling": "every_15_minutes",
      "accuracy": "100m_minimum",
      "battery_optimization": true
    },
    "communication": {
      "call_logs": "metadata_only",
      "message_logs": "frequency_only",
      "content": "never_collected"
    },
    "device_usage": {
      "screen_time": "aggregated_hourly",
      "app_categories": true,
      "specific_apps": false
    },
    "physiological": {
      "heart_rate": "continuous_when_worn",
      "sleep": "nightly",
      "activity": "continuous"
    }
  },
  "privacy_protocol": {
    "local_processing": true,
    "anonymization": "on_device",
    "transmission": "aggregated_daily",
    "raw_data_storage": "device_only"
  }
}
```

#### 2.2.2 Phenotype Risk Scores

```
Digital Phenotype Risk Algorithm:
┌───────────────────────────────────────────────────────────┐
│  INPUT FEATURES                      │  WEIGHT           │
├───────────────────────────────────────────────────────────┤
│  Home time increase (7-day delta)    │  0.20             │
│  Sleep regularity decrease           │  0.18             │
│  Social communication decrease       │  0.17             │
│  Physical activity decrease          │  0.15             │
│  Screen time increase (night)        │  0.12             │
│  Location entropy decrease           │  0.10             │
│  Voice features (if available)       │  0.08             │
├───────────────────────────────────────────────────────────┤
│  OUTPUT: Relapse Risk Score (0.0 - 1.0)                   │
│  Threshold: > 0.7 triggers clinical alert                 │
└───────────────────────────────────────────────────────────┘
```

## 3. Treatment Protocols

### 3.1 Evidence-Based Treatment Selection

#### 3.1.1 Stepped Care Model

```
┌─────────────────────────────────────────────────────────────┐
│  STEP 4: Specialist/Intensive Treatment                     │
│  - Inpatient care                                           │
│  - Intensive outpatient programs                            │
│  - Electroconvulsive therapy (ECT)                         │
│  - Psychedelic-assisted therapy (clinical trials)          │
├─────────────────────────────────────────────────────────────┤
│  STEP 3: High-Intensity Interventions                       │
│  - CBT, DBT, EMDR (individual therapy)                      │
│  - Medication + therapy combination                         │
│  - TMS/tDCS neuromodulation                                 │
├─────────────────────────────────────────────────────────────┤
│  STEP 2: Low-Intensity Interventions                        │
│  - Guided self-help (digital therapeutics)                  │
│  - Group therapy                                            │
│  - Peer support programs                                    │
├─────────────────────────────────────────────────────────────┤
│  STEP 1: Universal Prevention                               │
│  - Psychoeducation                                          │
│  - Lifestyle interventions (exercise, sleep, diet)          │
│  - Stress management apps                                   │
└─────────────────────────────────────────────────────────────┘
```

#### 3.1.2 Treatment Matching Protocol

```json
{
  "treatment_matching_algorithm": {
    "inputs": {
      "diagnosis": "ICD-11 code",
      "severity": "mild|moderate|severe",
      "comorbidities": ["list"],
      "previous_treatments": ["list with outcomes"],
      "patient_preferences": {
        "therapy_type": "individual|group|digital",
        "medication_openness": 0.0-1.0
      },
      "practical_factors": {
        "insurance": "coverage_type",
        "availability": "hours_per_week",
        "transportation": "accessibility_score"
      }
    },
    "matching_rules": [
      {
        "condition": "depression_moderate + no_prior_treatment",
        "first_line": ["CBT", "IPT", "behavioral_activation"],
        "evidence": "NICE_guidelines_2022"
      },
      {
        "condition": "depression_severe + failed_2_SSRIs",
        "options": ["augmentation_strategy", "TMS", "ketamine_referral"],
        "evidence": "APA_STAR*D_follow_up"
      },
      {
        "condition": "PTSD + trauma_focused_preference",
        "first_line": ["EMDR", "CPT", "prolonged_exposure"],
        "evidence": "VA_DoD_guidelines_2023"
      }
    ]
  }
}
```

### 3.2 Psychotherapy Protocol Standards

#### 3.2.1 CBT Protocol Structure

```
Standard CBT Protocol (12-16 sessions):
┌─────────────────────────────────────────────────────────────┐
│  PHASE 1: Assessment & Engagement (Sessions 1-2)            │
│  - Comprehensive assessment                                 │
│  - Psychoeducation about CBT model                         │
│  - Goal setting                                             │
│  - Mood monitoring introduction                             │
├─────────────────────────────────────────────────────────────┤
│  PHASE 2: Behavioral Activation (Sessions 3-5)              │
│  - Activity scheduling                                      │
│  - Pleasure and mastery activities                          │
│  - Behavioral experiments                                   │
├─────────────────────────────────────────────────────────────┤
│  PHASE 3: Cognitive Restructuring (Sessions 6-10)           │
│  - Identify automatic thoughts                              │
│  - Cognitive distortions education                          │
│  - Thought records                                          │
│  - Core beliefs work                                        │
├─────────────────────────────────────────────────────────────┤
│  PHASE 4: Skill Consolidation (Sessions 11-14)              │
│  - Advanced cognitive techniques                            │
│  - Problem-solving training                                 │
│  - Stress management                                        │
├─────────────────────────────────────────────────────────────┤
│  PHASE 5: Relapse Prevention (Sessions 15-16)               │
│  - Review progress                                          │
│  - Identify warning signs                                   │
│  - Develop maintenance plan                                 │
│  - Booster session scheduling                               │
└─────────────────────────────────────────────────────────────┘
```

#### 3.2.2 Session Documentation Protocol

```json
{
  "session_documentation": {
    "required_fields": {
      "session_number": "integer",
      "date_time": "ISO 8601",
      "duration_minutes": "integer",
      "format": "in_person|telehealth",
      "presenting_concerns": "string",
      "interventions_used": ["list"],
      "homework_review": "completed|partial|not_done",
      "new_homework": "string",
      "risk_assessment": "none|passive_SI|active_SI|plan",
      "next_session_plan": "string"
    },
    "optional_fields": {
      "validated_measure_scores": "object",
      "session_rating_scale": 0.0-1.0,
      "alliance_measure": 0.0-1.0
    },
    "encryption": "required_for_notes"
  }
}
```

### 3.3 Psychedelic-Assisted Therapy Protocol

#### 3.3.1 Safety Screening Protocol

```
PSYCHEDELIC THERAPY SCREENING CHECKLIST
┌─────────────────────────────────────────────────────────────┐
│  ABSOLUTE EXCLUSION CRITERIA                                │
├─────────────────────────────────────────────────────────────┤
│  □ Personal history of psychotic disorder                   │
│  □ First-degree relative with schizophrenia                │
│  □ Current manic episode                                    │
│  □ Severe cardiovascular disease (for MDMA)                 │
│  □ Pregnancy or breastfeeding                              │
│  □ Current use of MAOIs (for psilocybin)                   │
│  □ Uncontrolled hypertension                                │
├─────────────────────────────────────────────────────────────┤
│  RELATIVE CONTRAINDICATIONS (case-by-case)                  │
├─────────────────────────────────────────────────────────────┤
│  □ Bipolar II disorder                                      │
│  □ Severe personality disorders                             │
│  □ Active substance use disorder                            │
│  □ History of prolonged psychedelic adverse reactions       │
│  □ Current dissociative symptoms                            │
│  □ Unstable medical conditions                              │
├─────────────────────────────────────────────────────────────┤
│  REQUIRED ASSESSMENTS                                       │
├─────────────────────────────────────────────────────────────┤
│  □ Comprehensive psychiatric evaluation                     │
│  □ Medical history and physical exam                        │
│  □ ECG (for MDMA)                                           │
│  □ Urine drug screen                                        │
│  □ Blood pressure monitoring                                │
│  □ Informed consent process completed                       │
└─────────────────────────────────────────────────────────────┘
```

#### 3.3.2 Psilocybin Session Protocol

```json
{
  "psilocybin_session_protocol": {
    "preparation_phase": {
      "sessions": 2-3,
      "duration_each": "60-90 minutes",
      "objectives": [
        "Build therapeutic alliance",
        "Psychoeducation about psilocybin",
        "Set intentions",
        "Discuss expectations and concerns",
        "Practice grounding techniques"
      ]
    },
    "dosing_day": {
      "setting": {
        "room": "comfortable, home-like environment",
        "lighting": "soft, adjustable",
        "temperature": "comfortable (20-22°C)",
        "music": "curated playlist (Johns Hopkins, Imperial College)",
        "eye_shades": "available",
        "comfortable_furniture": "bed or reclining chair"
      },
      "timeline": {
        "T-60min": "Arrival, vital signs, final preparation",
        "T-0": "Dose administration (oral capsule)",
        "T+30-45min": "Onset of effects",
        "T+60-120min": "Peak experience",
        "T+4-6h": "Resolution phase",
        "T+6-8h": "Integration conversation, discharge planning"
      },
      "monitoring": {
        "vital_signs": "every 30 minutes during peak",
        "psychological_support": "continuous presence of trained facilitators",
        "documentation": "experience notes (with consent)"
      },
      "support_interventions": [
        "Grounding techniques",
        "Breathing exercises",
        "Reassurance ('Trust, let go, be open')",
        "Physical comfort (blankets, water)",
        "Music adjustments"
      ]
    },
    "integration_phase": {
      "sessions": 3-6,
      "timing": "Day 1, Week 1, Week 2, Week 4, Month 2, Month 3",
      "techniques": [
        "Narrative processing of experience",
        "Connecting insights to life",
        "Somatic integration",
        "Creative expression (art, journaling)",
        "Mindfulness practice continuation"
      ]
    }
  }
}
```

### 3.4 Digital Therapeutics Protocol

#### 3.4.1 Prescription Digital Therapeutic Workflow

```
PDT IMPLEMENTATION WORKFLOW
┌─────────────────────────────────────────────────────────────┐
│  1. ASSESSMENT                                              │
│  ├── Confirm diagnosis (ICD-11 code)                        │
│  ├── Verify PDT indication match                            │
│  └── Check contraindications                                │
├─────────────────────────────────────────────────────────────┤
│  2. PRESCRIPTION                                            │
│  ├── Select appropriate PDT                                 │
│  ├── Generate prescription code                             │
│  └── Set treatment duration (typically 90 days)             │
├─────────────────────────────────────────────────────────────┤
│  3. ONBOARDING                                              │
│  ├── Patient downloads app                                  │
│  ├── Enters prescription code                               │
│  ├── Completes baseline assessment                          │
│  └── Configures notification preferences                    │
├─────────────────────────────────────────────────────────────┤
│  4. ACTIVE TREATMENT                                        │
│  ├── Patient completes modules                              │
│  ├── Real-time engagement tracking                          │
│  ├── Provider dashboard monitoring                          │
│  └── Automated alerts for non-adherence                     │
├─────────────────────────────────────────────────────────────┤
│  5. OUTCOME ASSESSMENT                                      │
│  ├── Weekly symptom measures                                │
│  ├── Module completion tracking                             │
│  ├── Engagement analytics                                   │
│  └── Treatment response categorization                      │
├─────────────────────────────────────────────────────────────┤
│  6. CLINICAL REVIEW                                         │
│  ├── Monthly provider check-ins                             │
│  ├── Treatment adjustment if needed                         │
│  └── Transition planning (maintenance or step-up)           │
└─────────────────────────────────────────────────────────────┘
```

## 4. Crisis Protocol

### 4.1 Crisis Detection and Response

```
CRISIS RESPONSE PROTOCOL
┌─────────────────────────────────────────────────────────────┐
│  LEVEL 1: LOW RISK                                          │
│  - Passive suicidal ideation without intent                 │
│  - No plan, no access to means                              │
│  → Action: Safety plan review, increase monitoring          │
├─────────────────────────────────────────────────────────────┤
│  LEVEL 2: MODERATE RISK                                     │
│  - Suicidal ideation with some intent                       │
│  - Vague plan, limited access to means                      │
│  → Action: Same-day clinical contact, restrict means        │
├─────────────────────────────────────────────────────────────┤
│  LEVEL 3: HIGH RISK                                         │
│  - Active suicidal ideation with intent                     │
│  - Specific plan, access to means                           │
│  → Action: Immediate clinical intervention, possible ED     │
├─────────────────────────────────────────────────────────────┤
│  LEVEL 4: IMMINENT RISK                                     │
│  - Imminent danger to self                                  │
│  - Active psychosis with command hallucinations             │
│  → Action: Emergency services (911/988), stay on line       │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Safety Planning Protocol

```json
{
  "safety_plan": {
    "step_1_warning_signs": [
      "Patient-identified early warning signs",
      "Thoughts, images, moods, situations, behaviors"
    ],
    "step_2_coping_strategies": [
      "Internal coping strategies patient can do alone",
      "Distraction, relaxation, physical activity"
    ],
    "step_3_social_supports": [
      {
        "name": "Contact name",
        "relationship": "Friend/Family",
        "phone": "Contact number"
      }
    ],
    "step_4_professional_contacts": [
      {
        "provider": "Therapist name",
        "phone": "Office number",
        "crisis_line": "After-hours number"
      },
      {
        "resource": "988 Suicide & Crisis Lifeline",
        "phone": "988",
        "availability": "24/7"
      }
    ],
    "step_5_means_restriction": {
      "lethal_means_identified": ["list"],
      "restriction_plan": "Detailed plan to limit access"
    },
    "step_6_reasons_for_living": [
      "Patient-identified reasons to stay alive"
    ],
    "digital_access": true,
    "emergency_contact_notified": true,
    "last_reviewed": "ISO 8601 date"
  }
}
```

## 5. Interoperability Protocol

### 5.1 EHR Integration

```json
{
  "ehr_integration": {
    "standards_supported": ["HL7 FHIR R4", "CDA R2"],
    "resources": {
      "Patient": "demographics",
      "Condition": "diagnoses (ICD-11)",
      "Observation": "assessment scores",
      "CarePlan": "treatment protocols",
      "MedicationRequest": "prescriptions",
      "Appointment": "session scheduling",
      "QuestionnaireResponse": "assessment responses"
    },
    "data_exchange": {
      "real_time": ["vital_signs", "crisis_alerts"],
      "batch": ["session_notes", "assessments"],
      "on_demand": ["treatment_summaries"]
    }
  }
}
```

### 5.2 Wearable Device Protocol

```json
{
  "wearable_integration": {
    "supported_data": {
      "heart_rate": {
        "sampling": "continuous or 5-minute intervals",
        "format": "bpm with timestamp"
      },
      "hrv": {
        "metrics": ["rmssd", "sdnn", "lf_hf"],
        "calculation": "5-minute windows"
      },
      "sleep": {
        "stages": ["awake", "light", "deep", "rem"],
        "granularity": "30-second epochs"
      },
      "activity": {
        "steps": "hourly aggregates",
        "exercise": "detected sessions"
      }
    },
    "protocols": ["Apple HealthKit", "Google Fit", "Fitbit Web API", "Garmin Connect"],
    "sync_frequency": "daily minimum, real-time preferred"
  }
}
```

## 6. Data Security Protocol

### 6.1 Encryption Standards

| Data State | Encryption | Algorithm |
|------------|------------|-----------|
| At Rest | Required | AES-256-GCM |
| In Transit | Required | TLS 1.3 |
| In Processing | Required | Secure enclave where available |
| In Backup | Required | AES-256 + separate key |

### 6.2 Access Control Protocol

```json
{
  "access_control": {
    "authentication": {
      "method": "OAuth 2.0 + OpenID Connect",
      "mfa": "required for clinical access",
      "session_timeout": "30 minutes inactive"
    },
    "authorization": {
      "model": "Role-Based Access Control (RBAC)",
      "roles": [
        {
          "role": "patient",
          "access": ["own_data", "assessment_submission", "treatment_progress"]
        },
        {
          "role": "therapist",
          "access": ["assigned_patients", "session_notes", "treatment_planning"]
        },
        {
          "role": "psychiatrist",
          "access": ["assigned_patients", "prescribing", "neuroimaging"]
        },
        {
          "role": "researcher",
          "access": ["anonymized_aggregate_data"]
        }
      ]
    },
    "audit_logging": {
      "all_access": true,
      "retention": "7 years minimum",
      "tamper_proof": true
    }
  }
}
```

## 7. Quality Assurance Protocol

### 7.1 Treatment Fidelity Monitoring

```
FIDELITY MONITORING CHECKLIST
┌─────────────────────────────────────────────────────────────┐
│  □ Protocol adherence rating (0-100%)                       │
│  □ Competency assessment completed                          │
│  □ Supervision documentation current                        │
│  □ Patient outcome tracking active                          │
│  □ Adverse event reporting up to date                       │
│  □ Continuing education requirements met                    │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Outcome Measurement Protocol

| Measure | Frequency | Target |
|---------|-----------|--------|
| Symptom reduction | Weekly | ≥50% reduction at endpoint |
| Response rate | End of treatment | ≥50% of patients |
| Remission rate | End of treatment | ≥30% of patients |
| Dropout rate | Ongoing | ≤20% |
| Patient satisfaction | End of treatment | ≥4.0/5.0 |

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
