# WIA-AUTOIMMUNE Phase 3: Clinical Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2026-01-04
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines clinical protocols for implementing the Treg-Microbiome Axis approach to autoimmune disease management.

### 1.1 Core Protocol Philosophy

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    TREG-MICROBIOME AXIS PROTOCOL                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌───────────────┐     ┌──────────────────┐     ┌───────────────┐         │
│  │   ASSESS      │────▶│    INTERVENE     │────▶│   MONITOR     │         │
│  │               │     │                  │     │               │         │
│  │ • Treg Status │     │ • Restore Treg   │     │ • Track Treg  │         │
│  │ • Microbiome  │     │ • Fix Microbiome │     │ • Microbiome  │         │
│  │ • Activity    │     │ • Reduce Flare   │     │ • Symptoms    │         │
│  └───────────────┘     └──────────────────┘     └───────────────┘         │
│         ▲                                               │                  │
│         └───────────────────────────────────────────────┘                  │
│                         CONTINUOUS LOOP                                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Assessment Protocol

### 2.1 Initial Comprehensive Assessment

```
PROTOCOL: WIA-AI-ASSESS-001
Name: Initial Autoimmune Assessment
Duration: 2-3 weeks for complete workup

┌─────────────────────────────────────────────────────────────────────────────┐
│ STEP 1: Clinical History & Symptoms (Day 1)                                │
├─────────────────────────────────────────────────────────────────────────────┤
│ □ Disease onset and progression timeline                                   │
│ □ Flare frequency and triggers                                             │
│ □ Current medications and response                                         │
│ □ Family history of autoimmunity                                           │
│ □ Dietary habits and GI symptoms                                           │
│ □ Antibiotic history (affects microbiome)                                  │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ STEP 2: Treg Functional Panel (Day 1-7)                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│ REQUIRED:                                                                   │
│ □ CD4+CD25+FOXP3+ Treg count (flow cytometry)                              │
│ □ FOXP3 expression level (MFI)                                             │
│ □ In vitro suppression assay                                               │
│                                                                             │
│ RECOMMENDED:                                                                │
│ □ CD25 (IL-2Rα) expression                                                 │
│ □ CTLA-4 surface expression                                                │
│ □ Helios expression (thymic vs peripheral origin)                          │
│ □ GRAIL E3 ligase activity assay                                           │
│                                                                             │
│ ADVANCED:                                                                   │
│ □ Treg TCR repertoire analysis                                             │
│ □ Treg epigenetic stability (TSDR methylation)                             │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ STEP 3: Microbiome Analysis (Day 1-14)                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│ SAMPLE COLLECTION:                                                          │
│ □ Fasting morning stool sample                                             │
│ □ 3-day dietary log prior to collection                                    │
│ □ Document recent antibiotics (last 3 months)                              │
│                                                                             │
│ ANALYSIS PANEL:                                                             │
│ □ Shotgun metagenomics (preferred) or 16S rRNA                             │
│ □ SCFA quantification (butyrate, propionate, acetate)                      │
│ □ Diversity indices (Shannon, Simpson, Chao1)                              │
│ □ Dysbiosis scoring                                                         │
│                                                                             │
│ LEAKY GUT PANEL:                                                            │
│ □ Serum zonulin                                                             │
│ □ LPS (lipopolysaccharide)                                                  │
│ □ I-FABP                                                                    │
│ □ Lactulose/mannitol challenge (optional)                                  │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ STEP 4: Disease Activity Assessment (Day 1-7)                              │
├─────────────────────────────────────────────────────────────────────────────┤
│ DISEASE-SPECIFIC:                                                           │
│                                                                             │
│ RA:  □ DAS28-CRP  □ Joint count  □ HAQ-DI  □ RF/Anti-CCP                  │
│ SLE: □ SLEDAI-2K  □ SLICC/ACR  □ Anti-dsDNA  □ Complement                 │
│ MS:  □ EDSS  □ MRI lesion count  □ OCB status  □ Relapse hx              │
│ T1D: □ HbA1c  □ C-peptide  □ GAD65/IA-2 Ab  □ Time in range              │
│ IBD: □ CDAI/Mayo  □ Calprotectin  □ CRP  □ Endoscopy score               │
│ PSO: □ PASI  □ BSA  □ DLQI  □ PsA screening                               │
│                                                                             │
│ INFLAMMATORY PANEL (ALL):                                                   │
│ □ CRP, ESR                                                                  │
│ □ IL-6, IL-17, TNF-α                                                       │
│ □ IL-10, TGF-β                                                              │
│ □ Th17/Treg ratio calculation                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Integrated Scoring

```
PROTOCOL: WIA-AI-SCORE-001
Name: Treg-Microbiome Axis Score (TMAS)

┌─────────────────────────────────────────────────────────────────────────────┐
│                     TMAS CALCULATION                                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  TMAS = (Treg_Score × 0.4) + (Microbiome_Score × 0.4) + (Activity × 0.2)  │
│                                                                             │
│  Where:                                                                     │
│  • Treg_Score = (FOXP3% + Suppression% + Stability%) / 3                   │
│  • Microbiome_Score = 100 - Dysbiosis_Score                                 │
│  • Activity = 100 - Disease_Activity_Normalized                             │
│                                                                             │
│  Interpretation:                                                            │
│  80-100: Excellent tolerance, remission likely                              │
│  60-79:  Good tolerance, stable disease                                     │
│  40-59:  Impaired tolerance, active disease                                 │
│  20-39:  Poor tolerance, high flare risk                                    │
│  0-19:   Severely impaired, urgent intervention needed                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Treatment Protocols

### 3.1 Microbiome Restoration Protocol

```
PROTOCOL: WIA-AI-MB-RESTORE-001
Name: Microbiome Restoration for Immune Tolerance
Duration: 12-16 weeks

┌─────────────────────────────────────────────────────────────────────────────┐
│ PHASE A: Preparation (Weeks 1-2)                                           │
├─────────────────────────────────────────────────────────────────────────────┤
│ GOALS: Remove dysbiotic pressure, prepare gut environment                   │
│                                                                             │
│ □ Eliminate:                                                                │
│   • Artificial sweeteners (saccharin, sucralose)                           │
│   • Excessive processed foods                                               │
│   • Unnecessary medications (NSAIDs, PPIs if possible)                      │
│                                                                             │
│ □ Reduce:                                                                   │
│   • Simple sugars                                                           │
│   • Red meat (if excessive)                                                 │
│   • Alcohol                                                                 │
│                                                                             │
│ □ Continue: Current disease-modifying therapy                               │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ PHASE B: Seeding (Weeks 3-8)                                               │
├─────────────────────────────────────────────────────────────────────────────┤
│ GOALS: Introduce beneficial SCFA-producing bacteria                         │
│                                                                             │
│ □ Probiotics (evidence-based strains):                                      │
│   • Lactobacillus rhamnosus GG                                             │
│   • Bifidobacterium infantis 35624                                          │
│   • Faecalibacterium prausnitzii (if available)                            │
│   Dose: 10-50 billion CFU daily                                             │
│                                                                             │
│ □ Fermented Foods:                                                          │
│   • Kefir or yogurt (live cultures): 200-400g daily                        │
│   • Sauerkraut or kimchi: 50-100g daily                                    │
│   • Miso or tempeh: as tolerated                                            │
│                                                                             │
│ □ Consider FMT if:                                                          │
│   • Severe dysbiosis (score >70)                                            │
│   • Failed conventional probiotic therapy                                   │
│   • IBD or recurrent C. diff                                                │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ PHASE C: Feeding (Weeks 3-16, ongoing)                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│ GOALS: Sustain SCFA production through prebiotic fiber                      │
│                                                                             │
│ □ Prebiotic Fiber Targets:                                                  │
│   • Total fiber: 30-40g daily                                               │
│   • Resistant starch: 15-20g daily                                          │
│   • Inulin/FOS: 5-10g daily                                                 │
│                                                                             │
│ □ Butyrate-Boosting Foods:                                                  │
│   • Oats, barley, whole grains                                              │
│   • Legumes (lentils, chickpeas, beans)                                    │
│   • Green bananas, cooked and cooled potatoes                               │
│   • Onions, garlic, leeks, asparagus                                        │
│                                                                             │
│ □ Optional Supplements:                                                     │
│   • Butyrate supplements (sodium/calcium butyrate): 300-600mg TID          │
│   • Partially hydrolyzed guar gum: 5g daily                                 │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ MONITORING SCHEDULE                                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│ Week 0:  Baseline microbiome, SCFA, leaky gut markers                       │
│ Week 6:  Repeat SCFA levels, symptom assessment                             │
│ Week 12: Full microbiome retest, Treg function, disease activity           │
│ Week 16: Final assessment, long-term maintenance plan                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Low-Dose IL-2 Protocol

```
PROTOCOL: WIA-AI-IL2-001
Name: Low-Dose IL-2 for Treg Expansion
Indication: Documented Treg dysfunction with active autoimmune disease

┌─────────────────────────────────────────────────────────────────────────────┐
│ ELIGIBILITY CRITERIA                                                        │
├─────────────────────────────────────────────────────────────────────────────┤
│ INCLUSION:                                                                  │
│ □ Confirmed autoimmune disease (RA, SLE, T1D, etc.)                        │
│ □ Treg suppressive function <60%                                           │
│ □ Failed or inadequate response to 1+ conventional DMARDs                  │
│ □ No active infection                                                       │
│                                                                             │
│ EXCLUSION:                                                                  │
│ □ Active malignancy                                                         │
│ □ Pregnancy or breastfeeding                                                │
│ □ Severe cardiac disease                                                    │
│ □ History of capillary leak syndrome                                        │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ DOSING PROTOCOL                                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│ INDUCTION (Weeks 1-5):                                                      │
│ • Aldesleukin (IL-2) 1.0-1.5 MIU/m² subcutaneously                         │
│ • Daily for 5 days (Days 1-5)                                               │
│ • Then weekly (Days 15, 22, 29, 36)                                         │
│                                                                             │
│ MAINTENANCE (Week 6 onwards):                                               │
│ • 1.0-1.5 MIU/m² every 2 weeks                                              │
│ • Adjust based on Treg response                                             │
│                                                                             │
│ ALTERNATIVE (for SLE, per LUPIL-2 protocol):                                │
│ • 1.5 MIU/day for 5 days                                                    │
│ • 2-week rest                                                               │
│ • Repeat for 4 cycles                                                       │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ MONITORING                                                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│ WEEKLY during induction:                                                    │
│ □ CBC with differential                                                     │
│ □ LFTs, creatinine                                                          │
│ □ Treg count (CD4+CD25+FOXP3+)                                              │
│                                                                             │
│ MONTHLY during maintenance:                                                 │
│ □ Full Treg panel (count, FOXP3, suppression)                               │
│ □ Disease activity score                                                    │
│ □ Inflammatory markers                                                      │
│                                                                             │
│ TARGET RESPONSE:                                                            │
│ • Treg count increase >50% from baseline                                    │
│ • Improved suppressive function                                             │
│ • Reduced Th17/Treg ratio                                                   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.3 Integrated Treg-Microbiome Intervention

```
PROTOCOL: WIA-AI-INTEGRATED-001
Name: Combined Treg-Microbiome Axis Restoration
Indication: Patients with both Treg dysfunction AND dysbiosis

┌─────────────────────────────────────────────────────────────────────────────┐
│                     SYNERGISTIC APPROACH                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Week 1-2:   Preparation (diet modification, baseline testing)             │
│                                                                             │
│  Week 3-8:   Parallel intervention:                                         │
│              • Microbiome restoration (probiotics, prebiotics)              │
│              • Low-dose IL-2 if Treg <40% function                          │
│                                                                             │
│  Week 9-12:  Consolidation:                                                 │
│              • Continue microbiome support                                  │
│              • Adjust IL-2 based on response                                │
│              • Add butyrate supplementation                                 │
│                                                                             │
│  Week 13+:   Maintenance:                                                   │
│              • Dietary maintenance (fiber, fermented foods)                 │
│              • Monthly Treg monitoring                                      │
│              • Quarterly microbiome assessment                              │
│                                                                             │
│  EXPECTED SYNERGY:                                                          │
│  Butyrate from healthy microbiome → FOXP3 stabilization                    │
│  IL-2 → Treg expansion                                                      │
│  Combined → Superior tolerance restoration                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Flare Prediction & Prevention

### 4.1 Early Warning Protocol

```
PROTOCOL: WIA-AI-FLARE-WARN-001
Name: Treg-Microbiome Flare Early Warning System

┌─────────────────────────────────────────────────────────────────────────────┐
│ RISK STRATIFICATION                                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│ HIGH RISK (>70% flare probability in 30 days):                              │
│ • Treg suppressive function dropped >20% from baseline                      │
│ • Th17/Treg ratio >4.0                                                      │
│ • Dysbiosis score increased >15 points                                      │
│ • Butyrate levels <30 μmol/g                                                │
│ • Zonulin >100 ng/mL                                                        │
│                                                                             │
│ MODERATE RISK (40-70%):                                                     │
│ • Treg function 40-60%                                                      │
│ • Rising inflammatory cytokines                                             │
│ • Decreasing SCFA producers                                                 │
│                                                                             │
│ LOW RISK (<40%):                                                            │
│ • Stable Treg function >60%                                                 │
│ • Low inflammatory markers                                                  │
│ • Healthy microbiome diversity                                              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ PREVENTIVE ACTIONS BY RISK LEVEL                                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│ HIGH RISK:                                                                  │
│ □ Immediate clinical review                                                 │
│ □ Consider temporary increase in immunosuppression                          │
│ □ Intensive microbiome support (butyrate supplement)                        │
│ □ Daily symptom monitoring                                                  │
│ □ Stress reduction protocol                                                 │
│                                                                             │
│ MODERATE RISK:                                                              │
│ □ Weekly Treg/microbiome monitoring                                         │
│ □ Optimize prebiotic intake                                                 │
│ □ Consider probiotic boost                                                  │
│ □ Review medication adherence                                               │
│                                                                             │
│ LOW RISK:                                                                   │
│ □ Continue maintenance protocol                                             │
│ □ Monthly monitoring                                                        │
│ □ Dietary counseling reinforcement                                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Remission Protocol

### 5.1 Achieving Deep Remission

```
PROTOCOL: WIA-AI-REMISSION-001
Name: Treg-Microbiome Deep Remission Achievement

┌─────────────────────────────────────────────────────────────────────────────┐
│ REMISSION CRITERIA                                                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│ CLINICAL REMISSION:                                                         │
│ □ Disease activity score in remission range                                 │
│ □ No symptoms requiring treatment adjustment                                │
│ □ Maintained for >3 months                                                  │
│                                                                             │
│ BIOCHEMICAL REMISSION (add to clinical):                                    │
│ □ CRP <5 mg/L                                                               │
│ □ ESR normal for age                                                        │
│ □ IL-6 <7 pg/mL                                                             │
│                                                                             │
│ IMMUNOLOGICAL REMISSION (deep remission):                                   │
│ □ Treg suppressive function >70%                                            │
│ □ Th17/Treg ratio <2.0                                                      │
│ □ TMAS score >75                                                            │
│ □ Microbiome dysbiosis score <25                                            │
│ □ Normal SCFA production                                                    │
│                                                                             │
│ DRUG-FREE REMISSION:                                                        │
│ □ All above criteria                                                        │
│ □ Off all immunosuppressive therapy >6 months                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ MEDICATION TAPERING PROTOCOL                                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│ PREREQUISITES FOR TAPERING:                                                 │
│ □ Clinical remission >6 months                                              │
│ □ TMAS score >70 and stable                                                 │
│ □ No recent flares (>12 months)                                             │
│                                                                             │
│ TAPERING APPROACH:                                                          │
│ Step 1: Reduce corticosteroids first (if on)                                │
│ Step 2: Reduce conventional DMARDs by 25% every 3 months                    │
│ Step 3: Space biologic dosing before stopping                               │
│                                                                             │
│ CONTINUE THROUGHOUT:                                                        │
│ □ Microbiome maintenance diet                                               │
│ □ Monthly Treg monitoring during taper                                      │
│ □ Immediate re-escalation plan if Treg function drops                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. Quality Metrics

### 6.1 Protocol Performance Indicators

| Metric | Target | Measurement |
|--------|--------|-------------|
| Assessment completion rate | >90% | All required panels completed |
| Treg response rate | >60% | Patients with >30% Treg improvement |
| Microbiome improvement | >65% | Patients with dysbiosis reduction |
| Flare prediction accuracy | >75% | Sensitivity of early warning system |
| Remission achievement | >40% | Patients achieving immunological remission |
| Treatment adherence | >85% | Protocol compliance rate |

---

© 2026 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity
