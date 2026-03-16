# WIA-DEF-013: NBC Defense Specification v1.0

> **Standard ID:** WIA-DEF-013
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [CBRN Agent Classification](#2-cbrn-agent-classification)
3. [Detection and Identification](#3-detection-and-identification)
4. [Protection Measures](#4-protection-measures)
5. [Decontamination Protocols](#5-decontamination-protocols)
6. [Medical Countermeasures](#6-medical-countermeasures)
7. [Emergency Response Procedures](#7-emergency-response-procedures)
8. [Implementation Guidelines](#8-implementation-guidelines)
9. [Safety Protocols](#9-safety-protocols)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for NBC (Nuclear, Biological, Chemical) defense operations, also known as CBRN (Chemical, Biological, Radiological, Nuclear) defense. It provides protocols for detection, protection, decontamination, and medical response to CBRN threats.

### 1.2 Scope

The standard covers:
- Classification and characterization of CBRN agents
- Detection and identification systems
- Personal protective equipment (PPE) standards
- Decontamination procedures and equipment
- Medical countermeasures and treatment protocols
- Emergency response and incident command
- Training and certification requirements

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard prioritizes the protection of human life and the environment. CBRN defense is fundamentally a humanitarian mission, protecting populations from weapons of mass destruction and hazardous materials while ensuring rapid, effective response that minimizes casualties and suffering.

### 1.4 Terminology

- **CBRN**: Chemical, Biological, Radiological, Nuclear
- **NBC**: Nuclear, Biological, Chemical (traditional terminology)
- **WMD**: Weapons of Mass Destruction
- **PPE**: Personal Protective Equipment
- **MOPP**: Mission-Oriented Protective Posture
- **Decon**: Decontamination
- **TIC**: Toxic Industrial Chemical
- **TIM**: Toxic Industrial Material
- **LD₅₀**: Lethal Dose, 50% (median lethal dose)
- **LC₅₀**: Lethal Concentration, 50% (median lethal concentration)

---

## 2. CBRN Agent Classification

### 2.1 Chemical Agents

#### 2.1.1 Nerve Agents (G-Series, V-Series)

**G-Series (German agents)**:
```
GA (Tabun):      C₅H₁₁N₂O₂P
GB (Sarin):      C₄H₁₀FO₂P
GD (Soman):      C₇H₁₆FO₂P
GF (Cyclosarin): C₇H₁₄FO₂P
```

**V-Series (Venomous agents)**:
```
VE:    C₁₁H₂₆NO₂PS
VG:    C₁₀H₂₄NO₂PS
VM:    C₉H₂₂NO₂PS
VX:    C₁₁H₂₆NO₂PS (most toxic)
```

**Properties**:
- **Mechanism**: Acetylcholinesterase inhibition
- **LD₅₀ (dermal)**: 1-10 mg for VX
- **LC₅₀ (inhalation)**: 35-70 mg-min/m³ for Sarin
- **Symptoms**: SLUDGE (Salivation, Lacrimation, Urination, Defecation, GI upset, Emesis)
- **Onset**: 30 seconds - 30 minutes
- **Persistence**: VX (days-weeks), GB (minutes-hours)

#### 2.1.2 Blister Agents (Vesicants)

**Sulfur Mustard (H, HD)**:
```
H:  C₄H₈Cl₂S
HD: C₄H₈Cl₂S (distilled)
HT: Mixture with T (70% HD, 30% T)
```

**Nitrogen Mustard**:
```
HN-1: C₆H₁₃Cl₂N
HN-2: C₅H₁₁Cl₂N
HN-3: C₆H₁₂Cl₃N
```

**Lewisite (L)**:
```
L: C₂H₂AsCl₃
```

**Properties**:
- **Mechanism**: Alkylation of DNA, proteins
- **LD₅₀ (dermal)**: 100 mg/kg for mustard
- **Symptoms**: Skin blisters, eye damage, respiratory injury
- **Onset**: 2-24 hours (mustard), immediate (Lewisite)
- **Persistence**: Days to weeks

#### 2.1.3 Blood Agents

**Cyanide Compounds**:
```
AC (Hydrogen Cyanide):    HCN
CK (Cyanogen Chloride):   CNCl
SA (Arsine):              AsH₃
```

**Properties**:
- **Mechanism**: Cytochrome oxidase inhibition
- **LC₅₀**: 2,000-5,000 mg-min/m³ (HCN)
- **Symptoms**: Rapid breathing, seizures, cardiac arrest
- **Onset**: Seconds to minutes
- **Persistence**: Non-persistent (vapor)

#### 2.1.4 Choking Agents (Pulmonary Agents)

```
CG (Phosgene):         COCl₂
DP (Diphosgene):       C₂Cl₄O₂
CL (Chlorine):         Cl₂
PS (Chloropicrin):     CCl₃NO₂
```

**Properties**:
- **Mechanism**: Pulmonary edema
- **LC₅₀**: 1,600 mg-min/m³ (phosgene)
- **Symptoms**: Coughing, choking, pulmonary edema
- **Onset**: 2-24 hours (delayed)
- **Persistence**: Non-persistent

#### 2.1.5 Incapacitating Agents

```
BZ (Quinuclidinyl benzilate): C₂₁H₂₃NO₃
```

**Properties**:
- **Mechanism**: Anticholinergic
- **Symptoms**: Hallucinations, confusion, hyperthermia
- **Onset**: 30 minutes - 4 hours
- **Duration**: 2-4 days

### 2.2 Biological Agents

#### 2.2.1 Bacterial Agents

**Category A (Highest Priority)**:

| Agent | Disease | Transmission | Mortality | Incubation |
|-------|---------|--------------|-----------|------------|
| Bacillus anthracis | Anthrax | Spores, contact | 85-100% (untreated) | 1-6 days |
| Yersinia pestis | Plague | Aerosol, fleas | 100% pneumonic (untreated) | 1-6 days |
| Francisella tularensis | Tularemia | Aerosol, contact | 30-60% (untreated) | 3-5 days |
| Brucella spp. | Brucellosis | Aerosol, contact | <5% | 5-60 days |

**Category B (Second Priority)**:
- Burkholderia mallei (Glanders)
- Burkholderia pseudomallei (Melioidosis)
- Coxiella burnetii (Q fever)
- Rickettsia prowazekii (Typhus)

#### 2.2.2 Viral Agents

**Category A (Highest Priority)**:

| Agent | Disease | Transmission | Mortality | Incubation |
|-------|---------|--------------|-----------|------------|
| Variola major | Smallpox | Aerosol, contact | 30% | 7-17 days |
| Ebola virus | Ebola VHF | Contact, fluids | 50-90% | 2-21 days |
| Marburg virus | Marburg VHF | Contact, fluids | 23-90% | 2-21 days |
| Lassa virus | Lassa fever | Aerosol, contact | 1-15% | 6-21 days |

**Category B (Second Priority)**:
- Alphaviruses (VEE, EEE, WEE)
- Hantaviruses
- Nipah virus
- Influenza (pandemic strains)

#### 2.2.3 Toxins

| Toxin | Source | LD₅₀ (μg/kg) | Route | Onset |
|-------|--------|--------------|-------|-------|
| Botulinum toxin | Clostridium botulinum | 0.001 (inhalation) | Aerosol, oral | 12-72 hours |
| Ricin | Castor beans | 3-5 (inhalation) | Aerosol, oral | 4-8 hours |
| Staphylococcal enterotoxin B | Staphylococcus aureus | >27,000 | Aerosol, oral | 3-12 hours |
| T-2 mycotoxin | Fusarium fungi | 1,200 (oral) | Contact, aerosol | Minutes |
| Saxitoxin | Shellfish | 5 (oral) | Oral | 30 minutes |

### 2.3 Radiological and Nuclear Agents

#### 2.3.1 Radiation Types

**Alpha (α) Particles**:
- **Composition**: 2 protons + 2 neutrons (helium nucleus)
- **Penetration**: Stopped by paper, skin
- **Range**: 2-10 cm in air
- **Hazard**: Internal (ingestion, inhalation)
- **Examples**: Plutonium-239, Americium-241, Polonium-210

**Beta (β) Particles**:
- **Composition**: High-energy electrons
- **Penetration**: Stopped by plastic, aluminum
- **Range**: Up to 10 meters in air
- **Hazard**: Skin burns, internal
- **Examples**: Strontium-90, Cesium-137, Iodine-131

**Gamma (γ) Rays**:
- **Composition**: Electromagnetic radiation
- **Penetration**: Dense materials (lead, concrete)
- **Range**: Kilometers in air
- **Hazard**: Whole-body external exposure
- **Examples**: Cobalt-60, Cesium-137, Iridium-192

**Neutrons (n)**:
- **Composition**: Neutral particles
- **Penetration**: Water, concrete, polyethylene
- **Range**: Hundreds of meters
- **Hazard**: Whole-body, induced radioactivity
- **Source**: Nuclear reactors, weapons

#### 2.3.2 Radiological Dispersal Devices (RDD)

**Dirty Bomb Components**:
- **Radioactive Material**: Cobalt-60, Cesium-137, Strontium-90, Iridium-192
- **Explosive**: Conventional explosives (TNT, C-4)
- **Dispersal**: Blast, fire, aerosol

**Effects**:
- **Immediate**: Blast injury, trauma
- **Short-term**: Radiation exposure, contamination
- **Long-term**: Cancer risk, environmental contamination

#### 2.3.3 Nuclear Weapons Effects

**Blast Effects**:
- **Overpressure**: 5-20 psi (destroys buildings)
- **Winds**: 160-470 mph (projectiles, debris)
- **Radius**: 0.5-10 km (depending on yield)

**Thermal Effects**:
- **Flash**: 3,000-4,000°C at epicenter
- **Burns**: 3rd degree within 1-5 km
- **Fires**: Widespread ignition of flammables

**Radiation Effects**:
- **Prompt**: Gamma, neutron (within 1 minute)
- **Residual**: Fallout (radioactive debris)
- **Dose**: 450 rem (LD₅₀/60), 600 rem (LD₁₀₀)

**Electromagnetic Pulse (EMP)**:
- **E1**: Nanosecond pulse (electronics damage)
- **E2**: Microsecond pulse (similar to lightning)
- **E3**: Seconds-minutes (power grid disruption)

---

## 3. Detection and Identification

### 3.1 Chemical Agent Detection

#### 3.1.1 M8 Chemical Agent Detector Paper

**Specifications**:
- **Type**: Liquid contact detector
- **Agents**: G-series, V-series, H-series
- **Colors**:
  - Gold/Yellow: G-series nerve agents
  - Dark green: V-series nerve agents
  - Red/Pink: Blister agents (H)
- **Sensitivity**: 1-10 μg/cm²
- **False positives**: Petroleum products, insect repellent

#### 3.1.2 M9 Chemical Agent Detector Tape

**Specifications**:
- **Type**: Adhesive tape, liquid contact
- **Agents**: Nerve, blister agents
- **Color**: Red/pink (any agent)
- **Use**: Worn on equipment, vehicles
- **Limitations**: Does not identify specific agent

#### 3.1.3 Chemical Agent Monitor (CAM)

**Specifications**:
- **Type**: Ion mobility spectrometry (IMS)
- **Agents**: G, V, H nerve/blister agents
- **Modes**: Vapor, surface contact
- **Alarm**: Audio/visual, agent-specific
- **Sensitivity**: 0.1 mg/m³ (nerve), 1 mg/m³ (blister)
- **Response time**: 10-30 seconds

#### 3.1.4 Joint Chemical Agent Detector (JCAD)

**Specifications**:
- **Type**: IMS with GPS, datalink
- **Agents**: Nerve, blister, blood agents
- **Detection**: Continuous, real-time
- **Communication**: Wireless network, automatic alerts
- **Battery**: 8+ hours continuous operation

#### 3.1.5 Laboratory Methods

**Gas Chromatography-Mass Spectrometry (GC-MS)**:
- **Sensitivity**: Parts per billion (ppb)
- **Specificity**: Definitive identification
- **Time**: 15-60 minutes
- **Sample**: Liquid, vapor, wipe samples

**Liquid Chromatography-MS (LC-MS)**:
- **Use**: Non-volatile agents, toxins
- **Sensitivity**: Parts per trillion (ppt)
- **Time**: 20-90 minutes

### 3.2 Biological Agent Detection

#### 3.2.1 Biological Integrated Detection System (BIDS)

**Specifications**:
- **Sampling**: Continuous air sampling
- **Detection**: Immunoassay (antibody-based)
- **Agents**: Anthrax, plague, ricin, SEB
- **Time**: 15-30 minutes
- **Automation**: Hands-off operation

#### 3.2.2 Joint Biological Point Detection System (JBPDS)

**Specifications**:
- **Sampling**: Aerosol collector
- **Detection**: Immunoassay tickets
- **Agents**: 10+ bacterial, viral, toxin agents
- **Time**: 15-25 minutes
- **Mobility**: Vehicle-mounted

#### 3.2.3 Polymerase Chain Reaction (PCR)

**Real-Time PCR (qPCR)**:
- **Target**: Genetic material (DNA/RNA)
- **Sensitivity**: 10-100 copies
- **Specificity**: Near 100% (with proper primers)
- **Time**: 1-3 hours
- **Use**: Confirmatory identification

**Multiplex PCR**:
- **Capability**: Detect multiple agents simultaneously
- **Throughput**: High (96-384 samples)
- **Cost**: Moderate to high

#### 3.2.4 Mass Spectrometry

**MALDI-TOF MS (Matrix-Assisted Laser Desorption/Ionization)**:
- **Use**: Bacterial identification
- **Time**: 5-15 minutes
- **Database**: Spectral library matching
- **Accuracy**: >95% genus, >90% species

### 3.3 Radiological Detection

#### 3.3.1 Personal Radiation Detectors (PRD)

**Specifications**:
- **Type**: CsI scintillator or Geiger-Mueller
- **Detection**: Gamma, X-ray
- **Alarm**: Dose rate or total dose threshold
- **Range**: 0.01 μR/hr - 10 R/hr
- **Battery**: 200+ hours

#### 3.3.2 Radionuclide Identification Devices (RID)

**Specifications**:
- **Type**: High-purity germanium (HPGe) or NaI detector
- **Detection**: Gamma spectroscopy
- **Identification**: Specific isotopes (Cs-137, Co-60, etc.)
- **Time**: 30 seconds - 5 minutes
- **Database**: Spectral library (500+ isotopes)

#### 3.3.3 Portal Monitors

**Specifications**:
- **Use**: Screening vehicles, cargo, personnel
- **Detectors**: Plastic scintillators, NaI
- **Detection**: Gamma, neutron
- **Throughput**: 5-15 mph vehicle speed
- **Sensitivity**: 1-10 nCi (depending on isotope)

---

## 4. Protection Measures

### 4.1 Mission-Oriented Protective Posture (MOPP)

#### 4.1.1 MOPP Levels

**MOPP 0**:
- **Gear**: Available within 2 hours
- **Worn**: None
- **Use**: Low threat environment

**MOPP 1**:
- **Gear**: Protective suit worn, mask carried
- **Worn**: Overgarment, boots
- **Carried**: Mask, gloves, hood
- **Use**: Increased threat

**MOPP 2**:
- **Gear**: Protective suit worn, mask ready
- **Worn**: Overgarment, boots, mask carried
- **Use**: High threat, attack imminent

**MOPP 3**:
- **Gear**: Full protection, mask on
- **Worn**: All protective gear, mask donned
- **Hood**: Up, secured
- **Use**: Attack underway

**MOPP 4**:
- **Gear**: Maximum protection
- **Worn**: Fully sealed (suit, mask, gloves, boots)
- **Use**: Contaminated environment
- **Duration**: 2-8 hours (depending on conditions)

#### 4.1.2 Heat Stress Management

**Work/Rest Cycles**:
```
Temperature (°F)  | MOPP 4 Work | Rest  | Fluid Intake
------------------|-------------|-------|-------------
 < 75°F (24°C)    | 60 min      | 15 min| 1 qt/hr
75-85°F (24-29°C) | 40 min      | 20 min| 1.5 qt/hr
85-90°F (29-32°C) | 30 min      | 30 min| 2 qt/hr
 > 90°F (32°C)    | 20 min      | 40 min| 2.5 qt/hr
```

### 4.2 Personal Protective Equipment (PPE)

#### 4.2.1 Chemical Protective Suits

**Joint Service Lightweight Integrated Suit Technology (JSLIST)**:
- **Layers**: Outer (ripstop), carbon (adsorption), inner (comfort)
- **Protection**: Vapor, liquid chemical agents
- **Duration**: 24 hours vapor, 6 liquid splashes
- **Weight**: 3.5 lbs (suit), 9 lbs (complete ensemble)

**Joint Protective Aircrew Ensemble (JPACE)**:
- **Use**: Flight crew protection
- **Features**: Fire-resistant, allows mobility
- **Protection**: Chemical vapor, limited liquid

**Chemically Resistant Suit (Level A)**:
- **Type**: Fully encapsulated, gas-tight
- **Material**: Butyl rubber, Viton
- **Use**: Unknown agents, high concentration
- **SCBA**: Worn inside suit
- **Duration**: 20-30 minutes (SCBA limit)

#### 4.2.2 Respiratory Protection

**M50 Joint Service General Purpose Mask (JSGPM)**:
- **Type**: Full facepiece, dual filter
- **Protection**: CBRN agents, particulates
- **Filters**: C2A1 canister (both sides)
- **Duration**: 15 days continuous, 30 days intermittent
- **Features**: Voice amplifier, hydration port, optical inserts

**M53 CBRN Protective Mask**:
- **Use**: Armored vehicle crew
- **Features**: Low profile, integrates with vehicle systems
- **Protection**: Same as M50

**Powered Air-Purifying Respirator (PAPR)**:
- **Type**: Battery-powered, positive pressure
- **Protection**: HEPA filtration, biological agents
- **Comfort**: Reduced breathing resistance
- **Duration**: 4-8 hours (battery)

#### 4.2.3 Gloves and Footwear

**Chemical Protective Gloves**:
- **Butyl rubber**: 25-30 mil thickness
- **Neoprene**: Chemical resistance, dexterity
- **Duration**: 24 hours (vapor), 6 hours (liquid)

**Chemical Protective Overboots**:
- **Material**: Butyl rubber
- **Height**: Over-the-ankle
- **Sole**: Molded tread
- **Duration**: 24 hours

### 4.3 Collective Protection

#### 4.3.1 Collective Protection Shelter (CPS)

**Specifications**:
- **Filtration**: HEPA + carbon, 99.9999% efficiency
- **Pressure**: Positive (0.05-0.2 in H₂O)
- **Entry**: Air lock, decontamination vestibule
- **Capacity**: 10-1,000 personnel (depending on design)
- **Duration**: Indefinite (with filter replacement)

#### 4.3.2 Vehicle Collective Protection

**M93 Fox NBC Reconnaissance Vehicle**:
- **Protection**: Overpressure, filtration
- **Detection**: MM-1 mass spectrometer
- **Crew**: 4 (protected)

**M1 Abrams Tank (with NBCS)**:
- **System**: Nuclear, Biological, Chemical protection System
- **Filtration**: Air purification
- **Pressure**: Positive pressure

---

## 5. Decontamination Protocols

### 5.1 Decontamination Principles

#### 5.1.1 Decontamination Methods

**Physical Removal**:
- **Absorption**: Sponges, pads (M295 kit)
- **Brushing**: Mechanical removal
- **Evaporation**: Heat, air circulation
- **Washing**: Water, soap, detergent
- **Weathering**: Natural degradation

**Chemical Neutralization**:
- **Oxidation**: Bleach (hypochlorite), peroxide
- **Hydrolysis**: Caustic solution (NaOH)
- **Enzymatic**: Biological degradation

### 5.2 Personnel Decontamination

#### 5.2.1 Skin Decontamination Kit (M291)

**Specifications**:
- **Contents**: 6 packets (resin-filled pouches)
- **Mechanism**: Adsorption, physical removal
- **Agents**: Nerve, blister agents
- **Use**: Within 1 minute of exposure
- **Coverage**: Face, hands, neck (1 packet each)
- **Limitations**: Does not neutralize, only removes

#### 5.2.2 Reactive Skin Decontamination Lotion (RSDL)

**Specifications**:
- **Active ingredient**: Dekon 139 (proprietary)
- **Mechanism**: Chemical neutralization
- **Agents**: Nerve (G, V), blister (H, L), T-2 toxin
- **Application**: Sponge applicator, 2 minutes contact
- **Efficacy**: >99% removal/neutralization
- **Coverage**: 0.5 m² (one packet)

#### 5.2.3 Operational Decontamination

**Procedure**:
1. **Preparation**: Set up decon site (upwind, downhill from contamination)
2. **Stations**:
   - Station 1: Gear removal (contaminated → clean side)
   - Station 2: Washing (0.5% bleach solution)
   - Station 3: Rinse (clean water)
   - Station 4: Monitoring (check for residual contamination)
3. **Timing**: 5-10 minutes per person
4. **Waste**: Collect contaminated water, gear for disposal

#### 5.2.4 Thorough Decontamination

**Procedure**:
1. **Undress**: Remove all clothing, equipment
2. **Wash**: Soap and water, scrub all skin
3. **Rinse**: Clean water, multiple rinses
4. **Dry**: Towels (clean)
5. **Monitor**: Radiation survey, chemical detection
6. **Dress**: Clean uniform
7. **Duration**: 20-30 minutes per person

### 5.3 Equipment Decontamination

#### 5.3.1 Decontaminating Solution 2 (DS2)

**Composition**:
- 70% diethylenetriamine (DETA)
- 28% ethylene glycol monomethyl ether (EGME)
- 2% sodium hydroxide (NaOH)

**Properties**:
- **Agents**: Nerve (G, V), blister (H)
- **Mechanism**: Chemical breakdown
- **Application**: Spray, wipe, 30 minutes contact
- **Limitations**: Corrosive, flammable, toxic

#### 5.3.2 Super Tropical Bleach (STB)

**Composition**:
- Calcium hypochlorite: Ca(OCl)₂
- Dilution: 5% solution (slurry) or 0.5% (liquid)

**Properties**:
- **Agents**: Most chemical, biological agents
- **Mechanism**: Oxidation
- **Application**: Slurry on equipment, liquid on personnel
- **Contact time**: 30 minutes (chemical), 60 minutes (biological)
- **Limitations**: Corrosive to metals, bleaches fabrics

#### 5.3.3 High-Pressure Wash

**Specifications**:
- **Pressure**: 500-3,000 psi
- **Temperature**: Hot water (120-180°F) preferred
- **Detergent**: Military or commercial
- **Flow rate**: 2-10 gpm
- **Use**: Large vehicles, equipment

### 5.4 Area Decontamination

#### 5.4.1 Marking Contaminated Areas

**NATO Marking System**:
- **Color**: Yellow (chemical), blue (biological), white (radiological)
- **Sign**: Triangle with agent symbol
- **Information**: Type, date/time, unit

#### 5.4.2 Decontamination Methods

**Covering**:
- **Materials**: Plastic sheeting, tarps
- **Use**: Prevent spread, allow weathering
- **Duration**: Weeks to months

**Removal**:
- **Method**: Excavate top 6-12 inches of soil
- **Disposal**: Secure landfill, incineration
- **Coverage**: High-traffic areas, critical sites

**Washing**:
- **Solution**: Fire hose, DS2, bleach
- **Pressure**: High-pressure wash
- **Use**: Roads, buildings, hardstands

**Neutralization**:
- **Chemical**: Bleach slurry (5% STB)
- **Coverage**: 1 gallon per 100 ft²
- **Contact**: 6-24 hours
- **Removal**: Sweep, vacuum, wash

---

## 6. Medical Countermeasures

### 6.1 Chemical Agent Antidotes

#### 6.1.1 Nerve Agent Antidote Kit (NAAK)

**Mark I Kit (Atropine + 2-PAM)**:
- **Atropine**: 2 mg (antimuscarinic)
  - **Mechanism**: Blocks acetylcholine at muscarinic receptors
  - **Effects**: Reduces secretions, bronchodilation
  - **Administration**: IM auto-injector (thigh)
- **2-PAM Chloride**: 600 mg (pralidoxime)
  - **Mechanism**: Reactivates acetylcholinesterase
  - **Effects**: Restores nerve function
  - **Administration**: IM auto-injector (thigh)

**Dosing**:
- **Mild symptoms**: 1 Mark I kit
- **Severe symptoms**: 2-3 Mark I kits (5-10 min intervals)
- **Maximum**: 3 kits (without medical supervision)

#### 6.1.2 Anticonvulsant (CANA)

**Diazepam Auto-Injector**:
- **Dose**: 10 mg IM
- **Mechanism**: GABA agonist, stops seizures
- **Indication**: Severe nerve agent exposure, seizures
- **Timing**: After 2nd or 3rd Mark I kit

#### 6.1.3 Cyanide Antidotes

**Hydroxocobalamin (Cyanokit)**:
- **Dose**: 5 grams IV over 15 minutes
- **Mechanism**: Binds cyanide to form cyanocobalamin (vitamin B12)
- **Advantages**: Safe, no methemoglobinemia
- **Repeat dose**: 5 grams if needed

**Sodium Nitrite + Sodium Thiosulfate**:
- **Nitrite**: 300 mg IV (induces methemoglobinemia)
- **Thiosulfate**: 12.5 g IV (converts cyanide to thiocyanate)
- **Mechanism**: Two-step detoxification
- **Limitations**: Risk of hypotension, methemoglobinemia

#### 6.1.4 Lewisite Antidote

**British Anti-Lewisite (BAL, Dimercaprol)**:
- **Dose**: 3-5 mg/kg IM every 4 hours
- **Mechanism**: Chelates arsenic
- **Indication**: Lewisite exposure (within 2 hours)
- **Duration**: 2 days, then taper

### 6.2 Biological Agent Countermeasures

#### 6.2.1 Anthrax

**Prophylaxis**:
- **Ciprofloxacin**: 500 mg PO BID × 60 days
- **Doxycycline**: 100 mg PO BID × 60 days
- **Vaccine (AVA)**: 5-dose series (0, 1, 6, 12, 18 months)

**Treatment (Inhalational)**:
- **Ciprofloxacin**: 400 mg IV Q8H
- **Doxycycline**: 100 mg IV Q12H
- **Antitoxin**: Raxibacumab (monoclonal antibody)
- **Duration**: 60 days total (IV → oral)

#### 6.2.2 Plague

**Prophylaxis**:
- **Doxycycline**: 100 mg PO BID × 7 days
- **Ciprofloxacin**: 500 mg PO BID × 7 days

**Treatment (Pneumonic)**:
- **Streptomycin**: 1 g IM BID × 10 days
- **Gentamicin**: 5 mg/kg IV daily × 10 days
- **Alternatives**: Doxycycline, ciprofloxacin

#### 6.2.3 Smallpox

**Prophylaxis**:
- **Vaccinia vaccine**: Within 3-4 days of exposure
- **VIG (Vaccinia Immune Globulin)**: Complications of vaccination

**Treatment**:
- **Tecovirimat (TPOXX)**: 600 mg PO BID × 14 days
- **Cidofovir**: 5 mg/kg IV weekly (alternative)
- **Supportive care**: Isolation, fluid management

#### 6.2.4 Botulism

**Antitoxin**:
- **Heptavalent Botulism Antitoxin (HBAT)**: One vial IV
- **Timing**: As soon as possible (before symptoms worsen)
- **Mechanism**: Neutralizes circulating toxin
- **Note**: Does not reverse existing paralysis

**Supportive Care**:
- **Mechanical ventilation**: Respiratory failure
- **Nutrition**: TPN, feeding tube
- **Duration**: Weeks to months recovery

### 6.3 Radiological/Nuclear Countermeasures

#### 6.3.1 Potassium Iodide (KI)

**Dosing**:
- **Adults**: 130 mg (once daily)
- **Children 3-18 yrs**: 65 mg
- **Infants <3 yrs**: 32 mg

**Mechanism**: Saturates thyroid with stable iodine, prevents I-131 uptake

**Timing**: Before or immediately after exposure (within 3-4 hours)

**Duration**: Until risk of exposure passes (days to weeks)

#### 6.3.2 Prussian Blue (Radiogardase)

**Dosing**:
- **Adults**: 3 grams PO TID
- **Children 2-12 yrs**: 1 gram PO TID

**Mechanism**: Binds cesium-137, thallium in GI tract

**Duration**: 30 days (or until body burden reduced)

#### 6.3.3 DTPA (Diethylenetriaminepentaacetic Acid)

**Ca-DTPA (initial dose)**:
- **Dose**: 1 gram IV over 3-4 minutes
- **Use**: Plutonium, americium, curium

**Zn-DTPA (subsequent doses)**:
- **Dose**: 1 gram IV daily
- **Duration**: Until excretion reaches baseline

**Mechanism**: Chelates transuranic elements, enhances urinary excretion

#### 6.3.4 Filgrastim (Neupogen)

**Dosing**:
- **Dose**: 5 μg/kg SC daily
- **Indication**: Radiation-induced neutropenia (ARS)

**Mechanism**: Granulocyte colony-stimulating factor (G-CSF)

**Duration**: Until neutrophil recovery

---

## 7. Emergency Response Procedures

### 7.1 Incident Command System (ICS)

#### 7.1.1 ICS Structure

```
Incident Commander
├── Safety Officer
├── Public Information Officer
├── Liaison Officer
├── Operations Section
│   ├── Staging Area
│   ├── Decontamination
│   ├── Medical
│   └── Tactical Teams
├── Planning Section
│   ├── Resources Unit
│   ├── Situation Unit
│   └── Documentation
├── Logistics Section
│   ├── Supply Unit
│   ├── Communications
│   └── Medical
└── Finance/Admin Section
    ├── Cost Accounting
    └── Compensation/Claims
```

#### 7.1.2 Incident Response Phases

**Phase 1: Recognition and Notification (0-5 minutes)**:
- Detect CBRN incident
- Activate alarms, alerts
- Notify emergency services
- Initiate protective actions

**Phase 2: Initial Response (5-30 minutes)**:
- Establish incident command
- Secure perimeter (hot, warm, cold zones)
- Don PPE
- Rescue/evacuate casualties
- Begin decontamination

**Phase 3: Extended Response (30 minutes - 12 hours)**:
- Agent identification
- Medical treatment
- Mass decontamination
- Casualty transport
- Evidence collection

**Phase 4: Recovery (12 hours - weeks)**:
- Area decontamination
- Environmental monitoring
- Victim follow-up
- Investigation
- Restoration

### 7.2 Hot, Warm, Cold Zone Concept

#### 7.2.1 Hot Zone (Exclusion Zone)

**Definition**: Area of actual contamination

**Activities**:
- Rescue operations
- Evidence collection
- Initial decontamination

**PPE**: Level A or B (full protection)

**Access**: Restricted to essential personnel

#### 7.2.2 Warm Zone (Contamination Reduction Zone)

**Definition**: Buffer between hot and cold zones

**Activities**:
- Decontamination corridor
- Personnel decon
- Equipment decon

**PPE**: Level B or C

**Access**: Controlled, trained personnel only

#### 7.2.3 Cold Zone (Support Zone)

**Definition**: Safe area, no contamination

**Activities**:
- Incident command post
- Medical treatment
- Staging area
- Media area

**PPE**: Normal work attire or Level D

**Access**: General access (controlled entry)

### 7.3 Mass Casualty Decontamination

#### 7.3.1 Decontamination Corridor Layout

```
HOT ZONE → Triage → Disrobe → Wash → Rinse → Dress → Medical → COLD ZONE
```

**Throughput**: 100-150 people per hour (per lane)

**Lanes**: Ambulatory (walk-through), Non-ambulatory (stretcher)

**Staff**: 10-15 personnel per lane

#### 7.3.2 Decontamination Process

**Step 1: Triage**:
- Rapid assessment (SALT: Sort, Assess, Lifesaving, Treatment/Transport)
- Separate ambulatory from non-ambulatory

**Step 2: Disrobe**:
- Remove all clothing (80-90% decontamination)
- Place in bags (evidence, waste)
- Privacy considerations

**Step 3: Wash**:
- Low-pressure water (avoid aerosols)
- Soap/detergent
- Head to toe, 2-3 minutes

**Step 4: Rinse**:
- Clean water
- Remove soap, contaminants
- 2-3 minutes

**Step 5: Dress**:
- Provide clean clothing, blankets
- Maintain dignity, warmth

**Step 6: Medical**:
- Secondary triage
- Treatment (antidotes, supportive care)
- Transport to hospital

---

## 8. Implementation Guidelines

### 8.1 Required Components

Any WIA-DEF-013 compliant system must include:

1. **Detection Capability**: Real-time monitoring of CBRN threats
2. **Protection Equipment**: PPE for responders and population
3. **Decontamination Systems**: Personnel and equipment decon
4. **Medical Stockpiles**: Antidotes, vaccines, treatments
5. **Training Programs**: Responder training and exercises
6. **Communication Systems**: Alerts, notifications, coordination

### 8.2 API Interface

#### 8.2.1 Detect CBRN Agent

```typescript
interface CBRNDetectionRequest {
  sensorData: SensorReading[];
  location: GeoCoordinate;
  timestamp: Date;
}

interface CBRNDetectionResponse {
  agentType: AgentType;
  agentSubtype?: string;
  concentration: number;  // mg/m³ or Bq/m³
  confidence: number;     // 0-1
  threatLevel: ThreatLevel;
  recommendations: string[];
}
```

#### 8.2.2 Assess Threat Level

```typescript
interface ThreatAssessmentRequest {
  agentType: AgentType;
  concentration: number;
  weatherData: WeatherCondition;
  population: number;
  infrastructure: string[];
}

interface ThreatAssessmentResponse {
  level: ThreatLevel;           // 1-5
  description: string;
  affectedArea: number;         // km²
  estimatedCasualties: number;
  timeToImpact: number;         // seconds
  recommendedActions: Action[];
}
```

#### 8.2.3 Calculate Protection Requirements

```typescript
interface ProtectionRequest {
  agentType: AgentType;
  concentration: number;
  duration: number;  // seconds
  personnel: number;
}

interface ProtectionResponse {
  moppLevel: MOPPLevel;
  ppeRequired: PPEItem[];
  workRestCycle: WorkRestCycle;
  maxExposureTime: number;  // seconds
  respiratoryProtection: RespiratoryProtection;
}
```

### 8.3 Data Formats

#### 8.3.1 CBRN Incident Report

```json
{
  "incidentId": "CBRN-2025-001",
  "timestamp": "2025-01-15T14:30:00Z",
  "location": {
    "lat": 38.8977,
    "lon": -77.0365,
    "address": "1600 Pennsylvania Ave, Washington DC"
  },
  "agentType": "chemical",
  "agentSubtype": "nerve-agent-vx",
  "concentration": 0.08,
  "unit": "mg/m³",
  "threatLevel": 4,
  "affectedArea": 2.5,
  "casualties": {
    "estimated": 150,
    "confirmed": 42,
    "fatalities": 8
  },
  "response": {
    "moppLevel": 4,
    "deconStatus": "in-progress",
    "medicalCountermeasures": ["mark-i-kit", "cana"],
    "evacuationStatus": "complete"
  },
  "weatherConditions": {
    "temperature": 22,
    "windSpeed": 12,
    "windDirection": 180,
    "humidity": 65
  }
}
```

### 8.4 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| D001 | Sensor malfunction | Calibrate or replace sensor |
| D002 | Unknown agent | Request lab analysis |
| D003 | Concentration out of range | Use alternative detection method |
| P001 | Insufficient PPE | Request additional supplies |
| P002 | Heat stress warning | Implement work/rest cycle |
| M001 | Antidote shortage | Request resupply |
| M002 | Patient overload | Request additional medical support |

---

## 9. Safety Protocols

### 9.1 Personnel Safety Checklist

- [ ] PPE properly fitted and sealed
- [ ] Buddy system in place
- [ ] Communication established
- [ ] Decontamination corridor set up
- [ ] Medical support standing by
- [ ] Heat stress monitoring active
- [ ] Exposure time tracked
- [ ] Relief personnel available

### 9.2 Quality Assurance

**Detection Equipment**:
- Calibration: Daily (or per manufacturer)
- Testing: Weekly with simulant
- Maintenance: Per technical manual
- Replacement: Per lifecycle (3-5 years)

**PPE**:
- Inspection: Before each use
- Fit testing: Annual (respirators)
- Replacement: Per manufacturer or after exposure
- Storage: Climate-controlled, out of sunlight

**Decontamination**:
- Solution concentration: Test each batch
- Water quality: Potable or better
- Waste disposal: Per hazmat regulations
- Effectiveness: Monitor with detection equipment

### 9.3 Training Requirements

**First Responder Awareness (4 hours)**:
- Recognize CBRN threats
- Notify appropriate authorities
- Establish perimeter
- Self-protection

**First Responder Operations (24 hours)**:
- Don PPE (MOPP 4)
- Conduct decontamination
- Rescue casualties
- Evidence preservation

**Technician Level (80 hours)**:
- Operate detection equipment
- Conduct mass decontamination
- Administer medical countermeasures
- Incident command

**Specialist Level (160 hours)**:
- Advanced detection/identification
- Technical decontamination
- Agent-specific response
- Training instructor

### 9.4 Exercise and Drills

**Tabletop Exercise (TTX)**: Quarterly
- Scenario-based discussion
- Test plans and procedures
- No equipment deployment

**Functional Exercise (FE)**: Semi-annual
- Simulate operations
- Test communications, decision-making
- Minimal equipment deployment

**Full-Scale Exercise (FSE)**: Annual
- Realistic scenario
- All personnel, equipment
- Evaluate entire response capability

---

## 10. References

### 10.1 Military Publications

1. FM 3-11 "Multiservice Tactics, Techniques, and Procedures for CBRN Operations"
2. FM 4-02.7 "Multiservice Tactics, Techniques, and Procedures for Health Service Support in a CBRN Environment"
3. ATP 3-11.23 "Technical Chemical, Biological, Radiological, Nuclear, and Explosives Force Protection"
4. STANAG 2150 "NATO Standards for CBRN Defense Equipment"

### 10.2 Civilian Guidance

1. NFPA 1994 "Standard on Protective Ensembles for First Responders to CBRN Terrorism Incidents"
2. OSHA 1910.120 "Hazardous Waste Operations and Emergency Response (HAZWOPER)"
3. EPA 40 CFR Part 311 "Worker Protection"
4. CDC "Strategic National Stockpile" guidance

### 10.3 Agent Data

| Source | Description |
|--------|-------------|
| CDC Emergency Preparedness | Agent fact sheets, treatment guidelines |
| ATSDR ToxFAQs | Toxicological profiles |
| NIOSH Pocket Guide | Chemical hazard data |
| WHO Guidelines | International health guidance |

### 10.4 WIA Standards

- WIA-INTENT: Intent-based emergency commands
- WIA-OMNI-API: Universal sensor data API
- WIA-SOCIAL: Mass notification systems
- WIA-QUANTUM: Secure CBRN communications

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-DEF-013 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
