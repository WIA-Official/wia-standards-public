# WIA-NANOTECHNOLOGY: PHASE 3 - PROTOCOL SPECIFICATION

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2026-01-12
**Philosophy**: 弘益人間 (Benefit All Humanity)

## 1. Overview

This specification defines standardized protocols for nanomaterial synthesis, characterization procedures, safety protocols, and quality control in the WIA-NANOTECHNOLOGY ecosystem.

## 2. Synthesis Protocol Framework

### 2.1 Protocol Structure

Every synthesis protocol must include:

1. **Metadata**: Protocol ID, version, author, validation status
2. **Safety Information**: Hazards, PPE requirements, emergency procedures
3. **Materials List**: Precursors, solvents, substrates with specifications
4. **Equipment Requirements**: Instruments, tools, and facilities needed
5. **Procedure Steps**: Detailed step-by-step instructions
6. **Quality Control**: Testing and validation criteria
7. **Waste Disposal**: Environmental and safety considerations

### 2.2 Chemical Vapor Deposition (CVD) Protocol

```yaml
protocolId: "CVD-CNT-001"
version: "1.2"
name: "Single-Walled Carbon Nanotube Synthesis via CVD"
author: "WIA Nanotechnology Lab"
validatedBy: "ISO/TS 80004-13:2017"

safety:
  hazards:
    - "High temperature (1000°C)"
    - "Flammable gases (CH4, H2)"
    - "Carbon monoxide generation"
  ppe:
    - "Heat-resistant gloves"
    - "Safety goggles"
    - "Lab coat"
    - "Closed-toe shoes"
  emergency:
    - "Gas leak: Evacuate, shut off gas supply"
    - "Fire: Use CO2 extinguisher, do not use water"

materials:
  precursors:
    - name: "Methane (CH4)"
      purity: "99.99%"
      supplier: "Airgas"
      quantity: "100 sccm flow rate"
      casNumber: "74-82-8"
    - name: "Hydrogen (H2)"
      purity: "99.999%"
      supplier: "Airgas"
      quantity: "300 sccm flow rate"
      casNumber: "1333-74-0"
  catalysts:
    - name: "Iron nanoparticles"
      size: "1-2 nm"
      substrate: "Silicon wafer with Al2O3 layer"
  substrates:
    - name: "Silicon wafer"
      size: "4 inch diameter"
      pretreatment: "Clean with acetone, IPA, DI water"

equipment:
  - name: "Tube furnace"
    specifications: "Max temp 1200°C, 50mm tube diameter"
  - name: "Mass flow controllers"
    specifications: "0-500 sccm, ±1% accuracy"
  - name: "Vacuum pump"
    specifications: "Rotary vane, <1 mTorr base pressure"
  - name: "Gas detection system"
    specifications: "CH4, H2, CO detection"

procedure:
  preparation:
    - step: 1
      action: "Clean CVD chamber with acetone and IPA"
      duration: "15 min"
      verification: "Visual inspection"
    - step: 2
      action: "Deposit catalyst on substrate"
      method: "Spin coating or thermal evaporation"
      target: "1 nm Fe layer"
    - step: 3
      action: "Load substrate in chamber center"
      notes: "Ensure proper alignment"
    - step: 4
      action: "Seal chamber and evacuate"
      target: "< 1 mTorr"
      duration: "30 min"

  synthesis:
    - step: 5
      action: "Heat chamber under H2 flow"
      temperature: "25°C → 900°C"
      heatingRate: "20°C/min"
      gasFlow:
        H2: "300 sccm"
      pressure: "100 Torr"
      duration: "45 min"
    - step: 6
      action: "Catalyst annealing"
      temperature: "900°C"
      gasFlow:
        H2: "300 sccm"
      duration: "10 min"
      purpose: "Form catalyst nanoparticles"
    - step: 7
      action: "CNT growth"
      temperature: "900°C"
      gasFlow:
        CH4: "100 sccm"
        H2: "300 sccm"
      pressure: "100 Torr"
      duration: "15 min"
      monitoring: "Real-time Raman spectroscopy (optional)"
    - step: 8
      action: "Cool down under H2"
      temperature: "900°C → 25°C"
      coolingRate: "10°C/min"
      gasFlow:
        H2: "300 sccm"
      duration: "90 min"
    - step: 9
      action: "Vent chamber with N2"
      gasFlow:
        N2: "500 sccm"
      duration: "5 min"

  postProcessing:
    - step: 10
      action: "Remove sample from chamber"
      safety: "Allow to reach room temperature"
    - step: 11
      action: "Visual inspection"
      criteria: "Dark coating on substrate indicates CNT growth"

qualityControl:
  characterization:
    - technique: "Raman Spectroscopy"
      criteria:
        - "G-band at ~1580 cm⁻¹"
        - "D-band at ~1350 cm⁻¹"
        - "G/D ratio > 10 for high quality"
        - "RBM peaks indicate SWCNT diameter"
    - technique: "SEM"
      criteria:
        - "Dense, aligned CNT forest"
        - "Height: 10-50 μm"
        - "Diameter: 1-2 nm (from catalyst)"
    - technique: "TEM"
      criteria:
        - "Single-walled structure visible"
        - "Few defects"
        - "Uniform diameter distribution"
  acceptance:
    - "Raman G/D > 10"
    - "SEM shows uniform coverage > 95%"
    - "TEM confirms SWCNT structure"

wasteDisposal:
  gases:
    - "Vent exhaust gases through scrubber"
    - "Monitor CO levels in exhaust"
  solids:
    - "Failed samples: general lab waste"
    - "Catalyst-containing materials: metal waste"
  cleaning:
    - "Solvent waste: collect in designated container"

troubleshooting:
  - problem: "No CNT growth"
    causes:
      - "Catalyst poisoning"
      - "Incorrect temperature"
      - "Gas contamination"
    solutions:
      - "Use fresh catalyst"
      - "Calibrate temperature sensor"
      - "Check gas purity"
  - problem: "Amorphous carbon deposits"
    causes:
      - "Temperature too low"
      - "CH4 concentration too high"
    solutions:
      - "Increase temperature to 950°C"
      - "Reduce CH4 flow rate"
```

### 2.3 Sol-Gel Synthesis Protocol

```yaml
protocolId: "SOL-GEL-TiO2-001"
version: "1.0"
name: "TiO2 Nanoparticle Synthesis via Sol-Gel Method"

materials:
  precursors:
    - name: "Titanium isopropoxide (TTIP)"
      formula: "Ti(OCH(CH3)2)4"
      quantity: "10 mL"
      purity: "97%"
  solvents:
    - name: "Ethanol"
      quantity: "50 mL"
      purity: "99.9%"
    - name: "Deionized water"
      quantity: "5 mL"
  reagents:
    - name: "Acetic acid"
      quantity: "2 mL"
      purpose: "pH control and chelating agent"

procedure:
  - step: 1
    action: "Prepare Solution A"
    details: "Mix 10 mL TTIP with 25 mL ethanol"
    stirring: "Magnetic stirrer, 300 rpm"
    duration: "15 min"
    temperature: "25°C"
  - step: 2
    action: "Prepare Solution B"
    details: "Mix 5 mL water, 25 mL ethanol, 2 mL acetic acid"
    stirring: "Magnetic stirrer, 300 rpm"
    duration: "10 min"
  - step: 3
    action: "Hydrolysis"
    details: "Add Solution B dropwise to Solution A"
    rate: "1 drop per 5 seconds"
    stirring: "Vigorous, 500 rpm"
    observation: "White precipitate forms"
  - step: 4
    action: "Aging"
    duration: "24 hours"
    temperature: "25°C"
    conditions: "Covered beaker, dark place"
  - step: 5
    action: "Centrifugation"
    speed: "8000 rpm"
    duration: "10 min"
    repetitions: 3
    washing: "Ethanol between each centrifugation"
  - step: 6
    action: "Drying"
    method: "Oven"
    temperature: "80°C"
    duration: "12 hours"
  - step: 7
    action: "Calcination (optional)"
    temperature: "450°C"
    duration: "2 hours"
    atmosphere: "Air"
    purpose: "Crystallize to anatase phase"

expectedYield: "3-5 g TiO2 nanoparticles"
particleSize: "10-20 nm (before calcination)"

qualityControl:
  - technique: "XRD"
    criteria: "Anatase phase peaks at 25.3°, 37.8°, 48.0°"
  - technique: "BET"
    criteria: "Surface area: 50-100 m²/g"
  - technique: "DLS"
    criteria: "Hydrodynamic diameter: 50-100 nm in ethanol"
```

## 3. Characterization Protocol Framework

### 3.1 Transmission Electron Microscopy (TEM) Protocol

```yaml
protocolId: "CHAR-TEM-001"
version: "1.1"
name: "Standard TEM Characterization for Nanomaterials"

samplePreparation:
  - method: "Drop casting"
    steps:
      - "Disperse nanomaterial in ethanol (0.1 mg/mL)"
      - "Ultrasonicate for 10 minutes"
      - "Drop 5 μL on carbon-coated Cu grid"
      - "Allow to dry at room temperature"
      - "Store in desiccator"
  - method: "Focused Ion Beam (FIB)"
    use: "For bulk samples"
    steps:
      - "Coat sample with protective Pt layer"
      - "Mill thin lamella (~100 nm thick)"
      - "Transfer to TEM grid"

instrumentSetup:
  microscope: "TEM with 200 kV acceleration voltage"
  preparation:
    - "Ensure vacuum < 10⁻⁷ Torr"
    - "Calibrate magnification with standard"
    - "Align electron beam"
    - "Focus gun and condenser lenses"

imagingProcedure:
  lowMagnification:
    - magnification: "10,000x - 50,000x"
      purpose: "Survey, find regions of interest"
      mode: "Bright field"
  highMagnification:
    - magnification: "200,000x - 1,000,000x"
      purpose: "Atomic structure, lattice fringes"
      mode: "High-resolution TEM (HRTEM)"
  diffraction:
    - mode: "Selected Area Electron Diffraction (SAED)"
      purpose: "Crystal structure determination"
      aperture: "200 nm - 1 μm"

dataCollection:
  images:
    - quantity: "Minimum 10 representative images"
    - format: "TIFF, 16-bit"
    - metadata: "Magnification, voltage, dose, defocus"
  particleSizeAnalysis:
    - software: "ImageJ or Digital Micrograph"
    - method: "Measure >100 particles"
    - parameters: "Length, width, area, aspect ratio"
  latticeParameters:
    - method: "FFT of HRTEM images"
    - software: "Digital Micrograph or Gatan"

qualityChecklist:
  - "No contamination visible"
  - "Proper focus (sharp lattice fringes)"
  - "No beam damage"
  - "Consistent sample thickness"
  - "Proper magnification calibration"
```

### 3.2 X-Ray Diffraction (XRD) Protocol

```yaml
protocolId: "CHAR-XRD-001"
version: "1.0"
name: "Powder XRD for Crystalline Nanomaterials"

samplePreparation:
  powderSample:
    - "Dry sample completely in oven (80°C)"
    - "Grind to fine powder with mortar and pestle"
    - "Pack into sample holder"
    - "Smooth surface with glass slide"
    - "Ensure sample height matches reference plane"

instrumentSetup:
  xraySource: "Cu Kα (λ = 1.5406 Å)"
  voltage: "40 kV"
  current: "40 mA"
  detectorType: "Scintillation or solid-state"

scanParameters:
  2thetaRange: "10° - 80°"
  stepSize: "0.02°"
  scanSpeed: "2° per minute"
  totalTime: "~35 minutes"

dataAnalysis:
  peakIdentification:
    - database: "ICDD PDF-4+ or COD"
    - software: "Match!, X'Pert HighScore, or JADE"
    - method: "Compare d-spacing and intensity ratios"
  crystalliteSize:
    - method: "Scherrer equation"
    - formula: "D = Kλ / (β cos θ)"
    - where:
        D: "Crystallite size"
        K: "Shape factor (0.9 for spheres)"
        λ: "X-ray wavelength"
        β: "FWHM of peak (radians)"
        θ: "Bragg angle"
  latticeParameters:
    - method: "Rietveld refinement"
    - software: "GSAS-II, FullProf, or TOPAS"

reportingRequirements:
  - "Diffraction pattern plot with labeled peaks"
  - "Peak list with 2θ, d-spacing, intensity"
  - "Phase identification and percentage"
  - "Crystallite size for major phases"
  - "Lattice parameters with error bars"
```

## 4. Safety Protocol Framework

### 4.1 Laboratory Safety Protocol

```yaml
protocolId: "SAFE-LAB-001"
version: "2.0"
name: "General Nanomaterial Laboratory Safety"

beforeWork:
  training:
    - "Complete nanomaterial safety training"
    - "Read and understand SDSs for all materials"
    - "Review emergency procedures"
  ppe:
    required:
      - "Lab coat (disposable for nanomaterials)"
      - "Safety goggles or face shield"
      - "Nitrile gloves (double-gloving recommended)"
      - "Closed-toe shoes"
    optional:
      - "Respirator (N95 or P100 for powder handling)"
      - "Hearing protection (for ultrasonication)"

duringWork:
  goodPractices:
    - "Work in fume hood or glovebox when possible"
    - "Minimize generation of aerosols"
    - "Use wet methods to prevent dust"
    - "Never pipette by mouth"
    - "Clean up spills immediately"
  monitoring:
    - "Check air quality monitors"
    - "Use personal exposure monitoring"
    - "Log all nanomaterial use"

afterWork:
  decontamination:
    - "Clean work surfaces with wet wipes"
    - "Remove PPE carefully (inside-out)"
    - "Wash hands thoroughly"
    - "Shower if skin exposure suspected"
  wasteDisposal:
    - "Solid nanomaterial waste: sealed containers"
    - "Liquid waste: designated bottles by composition"
    - "Contaminated PPE: special nanomaterial waste"
    - "Follow institution waste management protocols"

emergencyProcedures:
  spill:
    small: "< 100 mL or 10 g"
    procedure:
      - "Alert others in area"
      - "Don appropriate PPE"
      - "Cover with damp paper towels"
      - "Carefully collect into sealed container"
      - "Clean area with wet wipes"
      - "Report to safety officer"
    large: "> 100 mL or 10 g"
    procedure:
      - "Evacuate area"
      - "Alert safety officer immediately"
      - "Do not attempt cleanup without training"
      - "Restrict access until professional cleanup"
  exposure:
    skin:
      - "Remove contaminated clothing"
      - "Wash with soap and water for 15 min"
      - "Seek medical attention"
      - "Report incident"
    eye:
      - "Flush with eyewash for 15 min"
      - "Seek medical attention immediately"
      - "Do not rub eyes"
    inhalation:
      - "Move to fresh air"
      - "If difficulty breathing, seek medical attention"
      - "Report incident"
  fire:
    - "Activate fire alarm"
    - "Evacuate according to fire plan"
    - "Use appropriate extinguisher if safe"
    - "Do not re-enter until cleared"
```

### 4.2 Nanomaterial-Specific Safety

```yaml
protocolId: "SAFE-NANO-001"
version: "1.0"
name: "Nanomaterial-Specific Safety Considerations"

carbonNanotubes:
  hazards:
    - "Potential respiratory hazard (fiber-like)"
    - "Possible skin irritant"
  controls:
    - "Always handle in fume hood"
    - "Use HEPA-filtered vacuum for cleanup"
    - "Wet methods for sample preparation"
  monitoring:
    - "Air sampling for CNT exposure"
    - "Medical surveillance for frequent users"

quantumDots:
  hazards:
    - "Heavy metal toxicity (Cd, Pb, Se)"
    - "Skin absorption"
  controls:
    - "Glovebox preferred"
    - "Heavy metal waste stream"
    - "Blood metal level monitoring"

metalNanoparticles:
  gold:
    hazards: "Low toxicity, inhalation concern"
  silver:
    hazards: "Potential cytotoxicity, argyria"
    controls: "Minimize skin contact"
  titaniumDioxide:
    hazards: "Possible carcinogen (IARC Group 2B)"
    controls: "Avoid aerosol generation"
```

## 5. Quality Control and Validation

### 5.1 Protocol Validation Framework

Every protocol must undergo:

1. **Literature Review**: Compare with published methods
2. **Risk Assessment**: Identify safety and quality risks
3. **Initial Testing**: 3 trial runs with full documentation
4. **Optimization**: Adjust parameters based on results
5. **Reproducibility**: 5 successful runs by different operators
6. **Documentation**: Complete protocol with all details
7. **Approval**: Review by safety and technical committees

### 5.2 Standard Operating Procedure (SOP) Format

All protocols must be converted to SOPs including:
- Scope and purpose
- Responsibilities
- Definitions and abbreviations
- Materials and equipment
- Detailed procedure
- Quality control
- Safety and environmental considerations
- References
- Revision history
- Approval signatures

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
