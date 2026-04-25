# Chapter 3: Advanced Photoresist Materials

## Enabling Nanoscale Pattern Transfer

---

## 🎯 **Learning Objectives**

By the end of this chapter, you will understand:

- Photoresist chemistry and mechanism of action
- EUV photoresist technology for sub-10nm nodes
- High-NA EUV requirements and challenges
- ArF and KrF excimer laser photoresists for mature nodes
- Critical performance metrics: resolution, sensitivity, LER
- Storage, handling, and shelf life management
- Major suppliers and market dynamics

---

## 1. What is Photoresist?

Photoresist is a light-sensitive material that undergoes chemical changes when exposed to specific wavelengths of electromagnetic radiation. In semiconductor manufacturing, photoresist acts as a temporary masking layer during lithography, enabling the transfer of circuit patterns onto silicon wafers with nanometer-scale precision.

### **Core Components**

**1. Polymer Resin (Base Material)**
- Provides structural integrity and film formation
- Determines etch resistance and mechanical properties
- Typical molecular weight: 5,000-50,000 Da
- Common polymers: Novolac (DUV), polyhydroxystyrene (ArF), acrylates (EUV)

**2. Photoactive Compound (PAC)**
- Absorbs light at specific wavelengths
- Initiates chemical reactions upon exposure
- Concentration: 5-30% by weight
- Examples: Diazonaphthoquinone (DNQ) for g-line/i-line, photoacid generators (PAG) for chemically amplified resists

**3. Solvent**
- Enables uniform coating via spin-on application
- Evaporates during soft-bake step
- Common solvents: Propylene glycol monomethyl ether acetate (PGMEA), cyclohexanone
- Composition: 60-85% of total formulation

**4. Additives**
- **Base quenchers**: Control acid diffusion in CAR
- **Surfactants**: Improve coating uniformity
- **Adhesion promoters**: Enhance substrate bonding
- **Dissolution inhibitors**: Fine-tune development rate
- Total additives: 1-5% by weight

### **Positive vs. Negative Photoresists**

**Positive Photoresists**:
- Exposed areas become MORE soluble in developer
- Enables finer line resolution
- Used for 95%+ of advanced logic/memory applications
- Mechanism: PAC destruction (DNQ) or acid-catalyzed deprotection (CAR)

**Negative Photoresists**:
- Exposed areas become LESS soluble (crosslink)
- Better adhesion and chemical resistance
- Used for thick coatings, packaging, MEMS
- Mechanism: Photo-initiated crosslinking

---

## 2. Evolution of Lithography Wavelengths

### **2.1 Historical Progression**

| Era | Wavelength | Resist Type | Min Feature | Application |
|-----|-----------|-------------|-------------|-------------|
| 1970s-1980s | 365 nm (i-line) | DNQ/Novolac | 0.8-1.5 µm | Early ICs |
| 1990s | 248 nm (KrF) | CAR | 0.25-0.5 µm | Mature nodes |
| 2000s | 193 nm (ArF) | CAR | 65-130 nm | DUV era |
| 2010s | 193 nm immersion | CAR + multiple patterning | 10-40 nm | Advanced DUV |
| 2020s+ | 13.5 nm (EUV) | Metal-oxide CAR | 3-8 nm | Cutting-edge logic |

### **2.2 Why Shorter Wavelengths?**

**Rayleigh's Resolution Equation**:
```
R = k₁ × λ / NA
```

Where:
- **R**: Minimum resolvable feature size
- **k₁**: Process-dependent factor (0.25-0.6)
- **λ**: Exposure wavelength
- **NA**: Numerical aperture of lens system

**To improve resolution**:
1. Reduce wavelength (λ) → New light sources and resist chemistry
2. Increase NA → Better lens design (limited by refractive index)
3. Reduce k₁ → Advanced resist processing, computational lithography

**Example calculation**:
- EUV (λ = 13.5 nm, NA = 0.33, k₁ = 0.5): R = 20 nm
- High-NA EUV (λ = 13.5 nm, NA = 0.55, k₁ = 0.5): R = 12 nm
- ArF immersion (λ = 193 nm, NA = 1.35, k₁ = 0.28): R = 40 nm

---

## 3. EUV Photoresist Technology

### **3.1 EUV Lithography Fundamentals**

**Light source**: Laser-produced plasma (LPP) generating 13.5 nm extreme ultraviolet light
- Tin (Sn) droplets bombarded with CO₂ laser
- Emits 13.5 nm photons (92 eV energy)
- Power at wafer: 250-350 W (current generation)

**Optical system**: All-reflective (no transmissive lenses possible at EUV)
- Multilayer mirrors (Mo/Si, 40-60 bilayers)
- Reflectivity: ~70% per mirror (6-10 mirrors in system)
- Overall transmission: 2-4%

**Vacuum environment**: 10⁻⁶ to 10⁻⁸ Pa
- Prevents EUV absorption by gases
- Requires contamination control (outgassing from resists)

### **3.2 EUV Resist Chemistry**

**First-Generation: Chemically Amplified Resists (CAR)**

**Mechanism**:
1. **Exposure**: EUV photons generate photoelectrons and secondary electrons
2. **Acid generation**: Electrons decompose photoacid generator (PAG)
3. **Post-exposure bake (PEB)**: Acid catalyzes deprotection of polymer
4. **Development**: Deprotected polymer dissolves in aqueous base developer (TMAH)

**Typical formulation**:
- Polymer: Protected polyhydroxystyrene or acrylate copolymer
- PAG: Onium salts (triarylsulfonium or diaryliodonium)
- Quencher: Amine or carboxylate base
- Solvent: PGMEA or PGME

**Challenges**:
- **Stochastic effects**: Random nature of photon/electron interactions
- **Line edge roughness (LER)**: Molecular-scale roughness at pattern edges
- **Pattern collapse**: Thin, high-aspect-ratio features fall over during development
- **Outgassing**: Volatile components contaminate scanner optics

**Second-Generation: Metal-Oxide Resists (MOR)**

**Composition**: Metal-containing molecules (Sn, Zr, Hf) in organic matrix

**Advantages**:
- Higher EUV absorption (5-10× vs. organic CAR)
- Reduced dose requirement (15-20 mJ/cm² vs. 30-50 mJ/cm²)
- Better LER performance
- Higher etch resistance

**Challenges**:
- Metal contamination concerns
- Limited solvent development options
- Compatibility with fab equipment
- Supply chain availability

**Third-Generation: Hybrid and Novel Approaches**

- **Photosensitized CAR**: Enhanced secondary electron generation
- **Dual-tone resists**: Can be developed as positive or negative
- **Resist-on-resist (RoR)**: Multi-layer approaches
- **Self-assembled materials**: Directed self-assembly for sub-20nm

### **3.3 EUV Resist Performance Metrics**

**Resolution-Line Edge Roughness-Sensitivity (RLS) Tradeoff**

This fundamental tradeoff limits simultaneous optimization:

**Resolution (R)**:
- Target: ≤8 nm half-pitch for High-NA EUV
- Current: 13-16 nm half-pitch demonstrated
- Limited by: Blur from electron scattering, molecular size

**Line Edge Roughness (LER)**:
- Target: <1.5 nm (3σ) for 5nm node
- Current: 2.0-2.5 nm (typical)
- Causes: Photon shot noise, acid diffusion, polymer molecular weight distribution
- Measurement: CD-SEM or CD-AFM

**Sensitivity (S)**:
- Target: 15-25 mJ/cm² for high-volume manufacturing
- Current EUV CAR: 25-40 mJ/cm²
- Current MOR: 15-25 mJ/cm²
- Impact: Scanner throughput (inversely proportional to dose)

**Mathematical relationship** (simplified):
```
LER ∝ 1 / √(Dose × Absorption)
Resolution ∝ √(Blur² + (LER/3)²)
```

### **3.4 High-NA EUV Requirements**

**ASML TWINSCAN EXE:5000 Series** (0.55 NA)

**Optical improvements**:
- Increased NA from 0.33 to 0.55 (67% improvement)
- Resolution: 8 nm half-pitch (single exposure)
- Anamorphic optics: 4× reduction in X, 8× in Y

**Resist challenges**:
- **Thinner films**: 25-30 nm (vs. 40-50 nm for 0.33 NA)
- **Better LER**: <1.2 nm (3σ) required
- **Lower defectivity**: <0.005 defects/cm²
- **Faster development**: Match 160+ wph scanner throughput

**Material requirements**:
- Higher absorbance for thin films
- Improved etch selectivity
- Reduced outgassing (<10⁻¹⁰ Torr·L/s)
- Metallic contamination <10⁹ atoms/cm²

---

## 4. ArF and KrF Photoresists

### **4.1 ArF (193 nm) Immersion Lithography**

**Applications**: 28nm-65nm nodes, mature products

**Resist chemistry**:
- **Polymer backbone**: Polymethacrylate derivatives
- **Protecting groups**: t-butyl, adamantyl ester
- **Transparency**: Aromatic rings avoided (high absorption at 193 nm)

**Typical ArF resist platform**:
```
Poly(methacrylate-co-norbornene lactone-co-hydroxystyrene)
+ Triarylsulfonium PAG
+ Amine quencher
+ Surfactant
```

**Immersion lithography enhancements**:
- Water as immersion fluid (n = 1.44 at 193 nm)
- Effective NA up to 1.35 (vs. 0.93 dry ArF)
- Resolution down to 38 nm single exposure
- Topcoat layer to protect resist from water

**Performance specifications**:
- Sensitivity: 25-35 mJ/cm²
- Resolution: 40-80 nm
- LER: 3-4 nm (3σ)
- Defectivity: <0.1 defects/cm²
- Etch selectivity: 2-3:1 (resist:BARC)

### **4.2 KrF (248 nm) Excimer Laser**

**Applications**: 130nm-250nm nodes, power devices, MEMS

**Resist chemistry**:
- **Polymer**: Poly(hydroxystyrene-co-alkyl acrylate)
- **PAG**: Diaryliodonium or triarylsulfonium salts
- **Mechanism**: Acid-catalyzed deprotection (CAR)

**Advantages over i-line**:
- Higher resolution capability
- Better etch resistance
- Process latitude

**Typical specifications**:
- Sensitivity: 15-30 mJ/cm²
- Resolution: 130-250 nm
- Thickness: 400-800 nm
- Contrast: >6.0
- Etch rate: <1.5× silicon dioxide

---

## 5. Critical Process Parameters

### **5.1 Coating Process**

**Spin coating**:
1. **Dispense**: 1-5 mL resist onto wafer center
2. **Spread**: Low speed (500 rpm) for 5-10 seconds
3. **Spin-off**: High speed (1500-3000 rpm) for 30-60 seconds
4. **Edge bead removal (EBR)**: Solvent rinse at wafer edge

**Film thickness control**:
```
Thickness ∝ 1 / √(Spin Speed)
```

- Typical range: 30-100 nm (EUV), 100-400 nm (ArF/KrF)
- Uniformity target: ±1% across wafer (1σ)
- Edge exclusion: 2-3 mm

### **5.2 Soft Bake (Post-Apply Bake)**

**Purpose**:
- Evaporate coating solvent (typically >99.5% removal)
- Densify resist film
- Improve adhesion to substrate

**Conditions**:
- Temperature: 90-130°C (material-dependent)
- Time: 60-90 seconds (hotplate) or 30-60 sec (rapid thermal processing)
- Uniformity: ±1°C across wafer

**Impact of under/over baking**:
- Under-baked: High outgassing, poor line profile, low sensitivity
- Over-baked: Increased sensitivity, reduced resolution, T-topping

### **5.3 Exposure**

**Dose control**:
- Precision: ±1% (3σ) for advanced nodes
- Range: 15-50 mJ/cm² depending on resist and wavelength
- Measurement: Dose-to-size calibration using test structures

**Focus control**:
- Precision: ±10 nm for EUV, ±25 nm for ArF immersion
- Correction: Scanner autofocus systems
- Impact: Resolution, depth of focus (DOF)

**Overlay accuracy**:
- Target: <2 nm (3σ) for 5nm node
- Measurement: Dedicated overlay targets
- Correction: Feed-forward/feed-back algorithms

### **5.4 Post-Exposure Bake (PEB)**

**Purpose** (for CAR only):
- Drive acid-catalyzed deprotection reaction
- Control acid diffusion length (impacts LER)

**Conditions**:
- Temperature: 90-130°C (typically 5-15°C above soft bake)
- Time: 60-90 seconds
- Uniformity: ±0.5°C critical for CD uniformity
- Delay: PEB should occur within 30 min of exposure (acid diffusion concern)

**Temperature sensitivity**:
```
ΔCD ≈ 3-5 nm per 1°C PEB temperature variation
```

### **5.5 Development**

**Developer chemistry**:
- **Positive resists**: Aqueous base (0.26N tetramethylammonium hydroxide - TMAH)
- **Negative resists**: Organic solvent (n-butyl acetate, etc.)

**Puddle development process**:
1. Dispense developer onto wafer
2. Puddle time: 30-60 seconds (no spinning)
3. Rinse with DI water
4. Spin dry at 2000-4000 rpm

**Developer time optimization**:
- Under-development: Residue in exposed areas (scum)
- Over-development: Line width loss, sidewall roughness
- Target: 10-20% over-development margin

### **5.6 Hard Bake (Post-Development Bake)**

**Purpose**:
- Remove residual developer and water
- Improve etch resistance
- Stabilize pattern for subsequent processing

**Conditions**:
- Temperature: 100-150°C
- Time: 60-90 seconds
- Environment: Nitrogen or vacuum (reduce oxidation)

---

## 6. Storage and Handling

### **6.1 Storage Requirements**

**Temperature control**:
- Storage: 4-8°C (refrigerated)
- Shelf life: 6 months (unopened), 3 months (opened)
- Warm-up before use: 2-4 hours to room temperature (prevents condensation)

**Light protection**:
- Store in amber bottles (blocks UV/visible light)
- Yellow room lighting (<10 lux, >500 nm wavelength)
- Avoid fluorescent lights (UV emission)

**Contamination prevention**:
- HEPA-filtered storage cabinets
- Sealed containers with dry nitrogen purge
- Dedicated chemical lines (no cross-contamination)

### **6.2 Filtration**

**Particle removal**:
- Point-of-use (POU) filters: 0.02-0.05 µm
- Recirculation filters: 0.1 µm
- Replace after 500-2000 liters (monitor pressure drop)

**Purpose**:
- Remove gel particles from polymer aggregation
- Eliminate foreign contaminants
- Prevent nozzle clogging and wafer defects

### **6.3 Quality Control Testing**

**Incoming inspection**:
- Batch certification review
- Particle count (<10 particles/mL >0.2 µm)
- Viscosity measurement (±3% of specification)
- Refractive index
- Contrast curve generation

**Periodic monitoring**:
- Sensitivity check every shift
- Linewidth control on monitor wafers
- Defect density trending

---

## 7. Major Photoresist Suppliers

### **7.1 Market Overview**

**Total addressable market**: $2.0-2.5 billion/year
- EUV resist: $300-400M (fastest growing segment)
- ArF immersion: $800-900M
- KrF: $400-500M
- i-line and others: $500-600M

**Market concentration**: Top 4 suppliers = 85%+ market share

### **7.2 Supplier Profiles**

#### **JSR Corporation (Japan)**
- **Market share**: 28-32%
- **Headquarters**: Tokyo
- **Revenue**: $650-750M (photoresist division)
- **Strengths**:
  - Leading EUV resist supplier (40%+ EUV market)
  - Co-developed resists with ASML and TSMC
  - Strong ArF immersion portfolio
- **Key products**:
  - EUV-series (metal-oxide and CAR platforms)
  - AR-series (ArF immersion)
  - KRF-series (KrF excimer)

#### **Tokyo Ohka Kogyo (TOK) (Japan)**
- **Market share**: 22-26%
- **Headquarters**: Kanagawa, Japan
- **Revenue**: $550-650M
- **Strengths**:
  - Broadest product portfolio
  - Strong in mature nodes (KrF, i-line)
  - Comprehensive ancillary materials (developers, strippers)
- **Key products**:
  - SEBP-series (EUV)
  - TARF-series (ArF)
  - THMR-series (KrF)

#### **Shin-Etsu Chemical (Japan)**
- **Market share**: 15-18%
- **Headquarters**: Tokyo
- **Revenue**: $400-500M
- **Strengths**:
  - Vertically integrated (makes both resists and wafers)
  - High-purity materials expertise
  - Cost-competitive
- **Key products**:
  - SEE-series (EUV)
  - SEPR-series (ArF immersion)

#### **DuPont (USA)**
- **Market share**: 12-15%
- **Headquarters**: Wilmington, Delaware
- **Revenue**: $350-450M
- **Strengths**:
  - Strong intellectual property portfolio
  - Advanced materials R&D capabilities
  - Close partnerships with Intel and Samsung
- **Key products**:
  - XP-series (EUV)
  - Ultimate-series (ArF immersion)

#### **Merck KGaA (Germany)**
- **Market share**: 5-8%
- **Headquarters**: Darmstadt, Germany
- **Revenue**: $150-200M
- **Strengths**:
  - European market leader
  - Strong in specialty and niche applications
  - Comprehensive materials portfolio beyond resists

---

## 8. Emerging Trends & Innovation

### **8.1 Next-Generation EUV Resists**

**Metal-oxide nanoparticle resists**:
- Discrete nanoparticles (2-5 nm diameter) vs. polymeric materials
- Ultra-high resolution potential (<5 nm)
- Challenges: Dispersion stability, adhesion

**High-absorption resists**:
- Target: >10 µm⁻¹ absorption at 13.5 nm
- Enables thinner films (<20 nm)
- Faster throughput (lower dose)

**Pattern transfer innovations**:
- Self-aligned multiple patterning (SAMP)
- Resist-on-resist (RoR) strategies
- Selective deposition for pattern transfer

### **8.2 AI and Machine Learning**

**Applications in resist optimization**:
- Predictive modeling of RLS tradeoffs
- Automated process window optimization
- Real-time defect classification
- Formulation design acceleration

**Example**: TSMC and IBM using ML to optimize EUV resist PEB conditions
- 30% reduction in optimization time
- 15% improvement in process window

### **8.3 Sustainable Photoresist Development**

**Green chemistry initiatives**:
- Bio-based solvents replacing petroleum-derived PGMEA
- Water-developable resists (reduce organic waste)
- Solvent recovery and recycling systems

**Regulatory drivers**:
- REACH regulations (EU)
- TSCA requirements (USA)
- VOC emission reduction mandates

---

## 9. Best Practices

### **9.1 Resist Management Program**

✅ **Inventory control**: FIFO (first-in-first-out), track lot genealogy
✅ **Temperature monitoring**: Continuous logging, alarms for excursions
✅ **Contamination prevention**: Dedicated lines, regular cleaning
✅ **Filter management**: Pressure drop monitoring, scheduled replacement
✅ **Waste segregation**: Proper disposal of expired materials

### **9.2 Process Optimization**

✅ **Design of experiments (DOE)**: Systematically optimize bake temps, PEB, development time
✅ **Statistical process control (SPC)**: Monitor CD, defectivity, sensitivity trends
✅ **Feed-forward/feed-back**: Use metrology data to adjust exposure/focus
✅ **Periodic qualification**: Revalidate process every 3-6 months

### **9.3 Troubleshooting Common Issues**

**High defectivity**:
- Check filter condition and particle count
- Inspect coating nozzle for clogs
- Verify EBR effectiveness
- Review soft bake temperature uniformity

**CD non-uniformity**:
- Verify hotplate temperature calibration
- Check PEB delay time consistency
- Confirm scanner dose/focus uniformity
- Review developer freshness and temperature

**Pattern collapse**:
- Reduce aspect ratio (thinner resist or wider features)
- Optimize rinse process (surfactant addition)
- Implement critical point drying
- Use pillar support structures

---

## 📊 **Case Study: ASML & JSR EUV Resist Co-Development**

**Challenge**: Enable 5nm node production with EUV lithography

**Approach**:
- Joint R&D program spanning 5 years
- Shared facilities at ASML campus (Veldhoven, Netherlands)
- Iterative resist-scanner optimization

**Innovations**:
1. Photosensitized CAR platform
   - 30% dose reduction (35 mJ/cm² → 25 mJ/cm²)
   - LER improvement from 3.2 nm to 2.5 nm

2. Advanced PAG design
   - Reduced acid diffusion length (10 nm → 6 nm)
   - Better line edge roughness

3. Outgassing mitigation
   - Novel polymer design with low volatile content
   - Enabled >95% scanner uptime

**Results**:
- Enabled TSMC 5nm mass production (2020)
- Scanner throughput: 170+ wph
- Manufacturing yield: >90% for logic products

---

## ✅ **Chapter Summary**

**Key Takeaways**:

1. Photoresists are complex formulations combining polymers, photoactive compounds, solvents, and additives

2. EUV resists face fundamental RLS (Resolution-LER-Sensitivity) tradeoffs requiring ongoing innovation

3. High-NA EUV will require thinner films (25-30 nm), better LER (<1.2 nm), and lower defectivity

4. ArF and KrF resists remain critical for mature nodes and represent majority of market volume

5. Process control (coating, bake, exposure, development) is critical for achieving target CD and defectivity

6. Proper storage (4-8°C, light-protected) and handling are essential for material stability and performance

7. Japanese suppliers (JSR, TOK, Shin-Etsu) dominate the market with 70%+ combined share

8. Future innovations focus on metal-oxide resists, AI optimization, and sustainable chemistry

---

## 🔍 **Review Questions**

1. Explain the mechanism of chemically amplified resists (CAR) and why they enable higher sensitivity
2. Calculate the theoretical resolution for High-NA EUV (λ=13.5nm, NA=0.55, k₁=0.4)
3. Why do EUV resists face more significant stochastic challenges than ArF resists?
4. What are the advantages and challenges of metal-oxide resists compared to traditional CAR?
5. Design a process flow for optimizing a new EUV resist for 3nm node production

---

*Next Chapter: Chapter 4 - Specialty Gases and Chemical Materials →*
