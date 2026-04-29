# WIA-CHRONIC-PAIN Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2026-01-04
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines integration patterns for connecting WIA-CHRONIC-PAIN with healthcare systems, pain clinics, research networks, and digital health platforms.

---

## 2. EHR Integration

### 2.1 FHIR R4 Resources

```json
{
  "resourceType": "Bundle",
  "type": "collection",
  "entry": [
    {
      "resource": {
        "resourceType": "Observation",
        "id": "pain-nrs-001",
        "status": "final",
        "category": [
          {
            "coding": [
              {
                "system": "http://terminology.hl7.org/CodeSystem/observation-category",
                "code": "survey"
              }
            ]
          }
        ],
        "code": {
          "coding": [
            {
              "system": "https://wia.live/chronic-pain/codes",
              "code": "WIA-CP-PAIN-001",
              "display": "Numeric Rating Scale Pain Score"
            },
            {
              "system": "http://loinc.org",
              "code": "72514-3",
              "display": "Pain severity - 0-10 verbal numeric rating"
            }
          ]
        },
        "subject": {
          "reference": "Patient/example"
        },
        "effectiveDateTime": "2026-01-04T14:00:00Z",
        "valueInteger": 7,
        "interpretation": [
          {
            "coding": [
              {
                "system": "http://terminology.hl7.org/CodeSystem/v3-ObservationInterpretation",
                "code": "H",
                "display": "High"
              }
            ]
          }
        ]
      }
    },
    {
      "resource": {
        "resourceType": "Observation",
        "id": "csi-score-001",
        "status": "final",
        "category": [
          {
            "coding": [
              {
                "system": "http://terminology.hl7.org/CodeSystem/observation-category",
                "code": "survey"
              }
            ]
          }
        ],
        "code": {
          "coding": [
            {
              "system": "https://wia.live/chronic-pain/codes",
              "code": "WIA-CP-CS-001",
              "display": "Central Sensitization Inventory Score"
            }
          ]
        },
        "subject": {
          "reference": "Patient/example"
        },
        "effectiveDateTime": "2026-01-04T14:00:00Z",
        "valueInteger": 62,
        "referenceRange": [
          {
            "high": { "value": 40 },
            "text": "Normal: <40"
          }
        ],
        "interpretation": [
          {
            "coding": [
              {
                "system": "http://terminology.hl7.org/CodeSystem/v3-ObservationInterpretation",
                "code": "H",
                "display": "High - Indicates central sensitization"
              }
            ]
          }
        ]
      }
    },
    {
      "resource": {
        "resourceType": "Condition",
        "id": "chronic-pain-001",
        "clinicalStatus": {
          "coding": [
            {
              "system": "http://terminology.hl7.org/CodeSystem/condition-clinical",
              "code": "active"
            }
          ]
        },
        "verificationStatus": {
          "coding": [
            {
              "system": "http://terminology.hl7.org/CodeSystem/condition-ver-status",
              "code": "confirmed"
            }
          ]
        },
        "category": [
          {
            "coding": [
              {
                "system": "http://terminology.hl7.org/CodeSystem/condition-category",
                "code": "problem-list-item"
              }
            ]
          }
        ],
        "code": {
          "coding": [
            {
              "system": "http://snomed.info/sct",
              "code": "82423001",
              "display": "Chronic pain syndrome"
            },
            {
              "system": "http://hl7.org/fhir/sid/icd-10-cm",
              "code": "G89.29",
              "display": "Other chronic pain"
            }
          ]
        },
        "subject": {
          "reference": "Patient/example"
        },
        "onsetDateTime": "2023-01-04",
        "note": [
          {
            "text": "Nociplastic pain with central sensitization. NRI: 45"
          }
        ]
      }
    },
    {
      "resource": {
        "resourceType": "CarePlan",
        "id": "neuromod-careplan-001",
        "status": "active",
        "intent": "plan",
        "title": "Neuroplasticity Reversal Program",
        "description": "Multimodal chronic pain treatment with focus on reversing maladaptive neuroplasticity",
        "subject": {
          "reference": "Patient/example"
        },
        "period": {
          "start": "2026-01-04",
          "end": "2026-04-04"
        },
        "activity": [
          {
            "detail": {
              "code": {
                "coding": [
                  {
                    "system": "https://wia.live/chronic-pain/procedures",
                    "code": "rTMS",
                    "display": "Repetitive Transcranial Magnetic Stimulation"
                  }
                ]
              },
              "status": "scheduled",
              "scheduledTiming": {
                "repeat": {
                  "frequency": 5,
                  "period": 1,
                  "periodUnit": "wk",
                  "count": 10
                }
              }
            }
          },
          {
            "detail": {
              "code": {
                "coding": [
                  {
                    "system": "http://snomed.info/sct",
                    "code": "228557008",
                    "display": "Cognitive behavioral therapy"
                  }
                ]
              },
              "status": "scheduled",
              "scheduledTiming": {
                "repeat": {
                  "frequency": 1,
                  "period": 1,
                  "periodUnit": "wk",
                  "count": 12
                }
              }
            }
          }
        ]
      }
    }
  ]
}
```

### 2.2 HL7 v2.x Message Mapping

```
MSH|^~\&|WIA_CHRONIC_PAIN|PAIN_CLINIC|EHR_SYSTEM|HOSPITAL|20260104140000||ORU^R01|MSG001|P|2.5.1
PID|1||PAT456789^^^WIA^MR||SMITH^JANE||19750315|F
OBR|1|ORD001|SPEC001|WIA-CP-ASSESS^Chronic Pain Assessment^WIA|||20260104140000
OBX|1|NM|WIA-CP-PAIN-001^NRS Pain Score^WIA||7|{score}|0-3|H|||F
OBX|2|NM|WIA-CP-CS-001^CSI Score^WIA||62|{score}|<40|H|||F
OBX|3|NM|WIA-CP-NRI^Neuroplasticity Reversal Index^WIA||45|{score}|>60||||F
OBX|4|ST|WIA-CP-TYPE^Pain Phenotype^WIA||nociplastic||||||F
OBX|5|NM|WIA-CP-PSY-001^Pain Catastrophizing^WIA||38|{score}|<20|H|||F
OBX|6|NM|WIA-CP-OPIOID^Current MME^WIA||0|mg/day|<50||||F
```

### 2.3 Integration Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        EHR INTEGRATION ARCHITECTURE                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌───────────────┐     ┌───────────────────┐     ┌──────────────────┐      │
│  │  Pain Clinic  │────▶│  WIA Integration  │────▶│ WIA-CHRONIC-PAIN │      │
│  │  EHR System   │◀────│     Gateway       │◀────│      API         │      │
│  └───────────────┘     └───────────────────┘     └──────────────────┘      │
│         │                       │                         │                │
│         │                       │                         │                │
│         ▼                       ▼                         ▼                │
│  ┌───────────────┐     ┌───────────────────┐     ┌──────────────────┐      │
│  │  Patient      │     │  Questionnaire    │     │  Treatment       │      │
│  │  Portal       │     │  Engine (PROs)    │     │  Recommendation  │      │
│  └───────────────┘     └───────────────────┘     └──────────────────┘      │
│                                                                             │
│  SUPPORTED EHR SYSTEMS:                                                     │
│  • Epic (FHIR R4, HL7v2) - Pain module integration                         │
│  • Cerner (FHIR R4, HL7v2)                                                 │
│  • Athenahealth (FHIR R4)                                                  │
│  • eClinicalWorks                                                          │
│  • Custom pain clinic EMRs                                                 │
│                                                                             │
│  PRO (Patient-Reported Outcomes) INTEGRATION:                              │
│  • PROMIS Pain Intensity                                                   │
│  • PROMIS Pain Interference                                                │
│  • PROMIS Physical Function                                                │
│  • BPI, CSI, PCS automated scoring                                        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Pain Clinic System Integration

### 3.1 Pain Management Platform Integration

```yaml
# Pain Clinic Platform Integration
integrations:
  - platform: "Pain Management Tracking Systems"
    capabilities:
      - medication_tracking
      - procedure_scheduling
      - outcome_monitoring
      - opioid_agreements

  - platform: "Neuromodulation Devices"
    devices:
      - name: "TMS Systems"
        vendors: ["MagVenture", "Nexstim", "BrainsWay"]
        data_exchange:
          - stimulation_parameters
          - session_logs
          - motor_threshold
          - treatment_response

      - name: "tDCS Devices"
        vendors: ["Soterix", "Neuroelectrics", "TCT Research"]
        data_exchange:
          - current_parameters
          - electrode_montage
          - session_duration
          - impedance_logs

      - name: "TENS/EMS"
        vendors: ["Various"]
        data_exchange:
          - usage_duration
          - program_settings
          - patient_reported_relief

  - platform: "Quantitative Sensory Testing"
    systems:
      - name: "Medoc QST"
        data_exchange:
          - thermal_thresholds
          - mechanical_thresholds
          - temporal_summation
          - cpm_results

      - name: "Algometry Systems"
        data_exchange:
          - pressure_pain_thresholds
          - mapping_data
```

### 3.2 Imaging Integration

```yaml
# Neuroimaging Integration for Pain
neuroimaging:
  modalities:
    - type: "structural_mri"
      analyses:
        - gray_matter_volume
        - cortical_thickness
        - white_matter_integrity
      regions_of_interest:
        - anterior_cingulate_cortex
        - insula
        - prefrontal_cortex
        - thalamus
        - periaqueductal_gray

    - type: "functional_mri"
      analyses:
        - resting_state_connectivity
        - default_mode_network
        - pain_matrix_activation
        - descending_modulation

    - type: "eeg"
      analyses:
        - pain_evoked_potentials
        - spectral_power
        - connectivity_metrics

  pacs_integration:
    - dicom_send_receive
    - radiomics_extraction
    - ai_analysis_results
    - longitudinal_comparison

  output_format: "WIA-CHRONIC-PAIN JSON + DICOM SR"
```

---

## 4. Digital Therapeutics Integration

### 4.1 Chronic Pain Apps

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    DIGITAL THERAPEUTICS ECOSYSTEM                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────┐       │
│  │                  WIA-CHRONIC-PAIN Patient App                    │       │
│  ├─────────────────────────────────────────────────────────────────┤       │
│  │                                                                  │       │
│  │  📊 Pain Tracking                                                │       │
│  │  ├── Daily NRS/VAS logging                                      │       │
│  │  ├── Location mapping (body diagram)                            │       │
│  │  ├── Activity correlation                                       │       │
│  │  └── Medication timing                                          │       │
│  │                                                                  │       │
│  │  🧠 CBT Modules                                                  │       │
│  │  ├── Pain neuroscience education                                │       │
│  │  ├── Cognitive restructuring exercises                          │       │
│  │  ├── Behavioral activation tracking                             │       │
│  │  └── Thought records                                            │       │
│  │                                                                  │       │
│  │  🧘 Mindfulness & Relaxation                                     │       │
│  │  ├── Guided meditations (pain-focused)                          │       │
│  │  ├── Progressive muscle relaxation                              │       │
│  │  ├── Body scan exercises                                        │       │
│  │  └── Breathing exercises                                        │       │
│  │                                                                  │       │
│  │  🏃 Exercise Programs                                            │       │
│  │  ├── Personalized routines                                      │       │
│  │  ├── Video demonstrations                                       │       │
│  │  ├── Progress tracking                                          │       │
│  │  └── Graded exposure protocols                                  │       │
│  │                                                                  │       │
│  │  📈 Progress Dashboard                                           │       │
│  │  ├── NRI trend over time                                        │       │
│  │  ├── Function improvement                                       │       │
│  │  ├── Treatment milestone tracking                               │       │
│  │  └── Goal progress                                              │       │
│  │                                                                  │       │
│  └─────────────────────────────────────────────────────────────────┘       │
│                                                                             │
│  THIRD-PARTY APP INTEGRATION:                                               │
│  • Curable (pain psychology)                                               │
│  • Pathways Pain Relief                                                    │
│  • Headspace/Calm (mindfulness)                                            │
│  • Physical therapy apps                                                   │
│                                                                             │
│  WEARABLE INTEGRATION:                                                      │
│  • Activity tracking (steps, exercise)                                     │
│  • Sleep monitoring                                                        │
│  • Heart rate variability                                                  │
│  • Stress indicators                                                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Telehealth Integration

```yaml
telehealth:
  platforms:
    - video_consultations
    - asynchronous_messaging
    - remote_monitoring

  chronic_pain_features:
    - virtual_physical_therapy
    - remote_cbt_sessions
    - group_pain_education
    - neuromodulation_supervision

  data_sharing:
    - real_time_pain_scores
    - exercise_adherence
    - medication_compliance
    - sleep_quality_data

  alerts:
    - pain_escalation
    - opioid_risk_indicators
    - suicidality_screening
    - treatment_non_adherence
```

---

## 5. Research & Clinical Trial Integration

### 5.1 CDISC Standards Mapping

```xml
<!-- CDISC CDASH Mapping for WIA-CHRONIC-PAIN -->
<cdisc:Domain name="QS" description="Questionnaires - Pain">
  <cdisc:Variable name="QSTESTCD" label="Pain Assessment Code">
    <cdisc:Mapping source="WIA-CP-PAIN-001" target="NRS"/>
    <cdisc:Mapping source="WIA-CP-PAIN-002" target="BPISEV"/>
    <cdisc:Mapping source="WIA-CP-PAIN-003" target="BPIINT"/>
    <cdisc:Mapping source="WIA-CP-CS-001" target="CSI"/>
    <cdisc:Mapping source="WIA-CP-PSY-001" target="PCS"/>
    <cdisc:Mapping source="WIA-CP-PSY-002" target="TSK"/>
  </cdisc:Variable>
</cdisc:Domain>

<cdisc:Domain name="NV" description="Nervous System Findings">
  <cdisc:Variable name="NVTESTCD" label="QST Results">
    <cdisc:Mapping source="QST_MDT" target="MECHDET"/>
    <cdisc:Mapping source="QST_PPT" target="PRESPPT"/>
    <cdisc:Mapping source="QST_TS" target="TEMPSUM"/>
    <cdisc:Mapping source="QST_CPM" target="CONDPM"/>
  </cdisc:Variable>
</cdisc:Domain>
```

### 5.2 Pain Research Networks

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    RESEARCH NETWORK INTEGRATION                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ACADEMIC PAIN CONSORTIA:                                                   │
│  ┌─────────────────────────────────────────────────────────────────┐       │
│  │ • NIH HEAL Initiative                                           │       │
│  │   - Back Pain Consortium (BACPAC)                               │       │
│  │   - Early Phase Pain Investigation Clinical Network (EPPIC-Net) │       │
│  │                                                                 │       │
│  │ • IMMPACT (pain outcomes)                                       │       │
│  │ • IASP research networks                                        │       │
│  │ • European Pain Federation (EFIC)                               │       │
│  └─────────────────────────────────────────────────────────────────┘       │
│                                                                             │
│  DATA SHARING:                                                              │
│  • FAIR principles (Findable, Accessible, Interoperable, Reusable)         │
│  • Anonymized QST profiles                                                  │
│  • Treatment response data                                                  │
│  • Neuroimaging sharing (BIDS format)                                      │
│                                                                             │
│  FEDERATED LEARNING:                                                        │
│  • Pain phenotype classifiers                                              │
│  • Treatment response prediction                                           │
│  • Chronification risk models                                              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. Opioid Monitoring Integration

### 6.1 PDMP Integration

```yaml
# Prescription Drug Monitoring Program Integration
pdmp_integration:
  supported_states: "All US states + territories"

  data_exchange:
    query:
      - patient_identifier
      - date_range
      - controlled_substance_only

    response:
      - prescription_history
      - mme_calculation
      - multiple_prescriber_alerts
      - overlapping_prescriptions

  alerts:
    - high_mme_warning (>90 MME)
    - multiple_prescribers
    - concurrent_benzodiazepine
    - early_refill_pattern

  compliance:
    - hipaa_compliant
    - state_specific_rules
    - audit_logging
```

### 6.2 CDC Opioid Guidelines Integration

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    OPIOID SAFETY INTEGRATION                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  CDC GUIDELINE DECISION SUPPORT:                                           │
│  ┌─────────────────────────────────────────────────────────────────┐       │
│  │                                                                 │       │
│  │  PRE-OPIOID CHECKLIST:                                          │       │
│  │  □ Non-opioid alternatives attempted                            │       │
│  │  □ Realistic goals established                                  │       │
│  │  □ Risks/benefits discussed                                     │       │
│  │  □ Treatment agreement signed                                   │       │
│  │  □ PDMP checked                                                 │       │
│  │  □ Urine drug screen obtained                                   │       │
│  │                                                                 │       │
│  │  ONGOING MONITORING:                                            │       │
│  │  □ MME threshold alerts (>50, >90)                              │       │
│  │  □ Concurrent benzodiazepine warning                            │       │
│  │  □ Functional improvement tracking                              │       │
│  │  □ Taper consideration prompts                                  │       │
│  │                                                                 │       │
│  └─────────────────────────────────────────────────────────────────┘       │
│                                                                             │
│  NALOXONE CO-PRESCRIBING:                                                  │
│  • Auto-prompt when MME >50                                                │
│  • Required documentation if declined                                      │
│  • Patient/caregiver education resources                                   │
│                                                                             │
│  OPIOID-INDUCED HYPERALGESIA DETECTION:                                    │
│  • Pain worsening despite dose increases                                   │
│  • Widespread pain development                                             │
│  • CSI score increase                                                      │
│  • Recommend opioid rotation or taper                                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 7. API Gateway Configuration

### 7.1 Security & Compliance

```yaml
# API Gateway Configuration
gateway:
  name: wia-chronic-pain-gateway
  version: "1.0.0"

  security:
    authentication:
      - type: oauth2
        provider: wia-auth
        scopes:
          - read:pain_profile
          - write:assessment
          - read:recommendations
          - admin:opioid_data
      - type: api_key
        header: X-WIA-API-Key

    authorization:
      rbac:
        roles:
          - patient: [read:own_profile, write:own_pain_scores]
          - clinician: [read:profile, write:assessment, read:recommendations]
          - pain_specialist: [all_clinical, opioid_management]
          - researcher: [read:anonymized]
          - admin: [all]

    encryption:
      in_transit: TLS 1.3
      at_rest: AES-256-GCM
      pii_fields: [patient_id, name, dob, address, opioid_history]
      substance_use_data: "42 CFR Part 2 compliant"

  compliance:
    - HIPAA
    - "42 CFR Part 2 (substance use data)"
    - State pain management regulations
    - DEA e-prescribing requirements
```

---

## 8. Deployment Options

### 8.1 Cloud Deployment

```yaml
# Kubernetes Deployment
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-chronic-pain-api
  namespace: wia-healthcare
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-chronic-pain
  template:
    metadata:
      labels:
        app: wia-chronic-pain
    spec:
      containers:
      - name: api
        image: wia/chronic-pain-api:1.0.0
        ports:
        - containerPort: 8080
        env:
        - name: PDMP_INTEGRATION
          value: "enabled"
        - name: NEUROMOD_DEVICES
          value: "enabled"
        resources:
          requests:
            memory: "1Gi"
            cpu: "500m"
          limits:
            memory: "4Gi"
            cpu: "2000m"
```

### 8.2 Pain Clinic On-Premise

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    PAIN CLINIC DEPLOYMENT                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  OPTION A: Cloud-Connected (Recommended)                                   │
│  ├── WIA API cloud-hosted                                                  │
│  ├── Local EHR integration gateway                                         │
│  ├── Neuromodulation device sync                                           │
│  └── Real-time PDMP queries                                                │
│                                                                             │
│  OPTION B: Hybrid                                                          │
│  ├── Core processing on-premise                                            │
│  ├── Cloud backup and analytics                                            │
│  ├── Federated learning participation                                      │
│  └── Air-gapped mode available                                             │
│                                                                             │
│  SYSTEM REQUIREMENTS:                                                       │
│  • 4 CPU cores (minimum)                                                   │
│  • 16 GB RAM                                                               │
│  • 200 GB SSD                                                              │
│  • Secure network connection                                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
