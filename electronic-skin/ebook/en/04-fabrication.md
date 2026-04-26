# Chapter 3: Fabrication Methods

## From Laboratory Prototypes to Manufacturing Scale

The journey from concept to product requires scalable, reproducible fabrication methods. While laboratory demonstrations of electronic skin often use expensive, slow techniques, commercial viability demands processes that can produce thousands of devices per day at acceptable cost. This chapter explores fabrication approaches across the spectrum from research to high-volume manufacturing.

## 3.1 Substrate Preparation

The foundation of e-skin fabrication begins with creating the base substrate with appropriate mechanical and surface properties.

### 3.1.1 Casting and Molding

**Basic PDMS Casting Process**:

1. **Mixing**: Combine base polymer and crosslinker (typical ratio 10:1 by weight)
2. **Degassing**: Vacuum chamber (20-30 minutes) to remove air bubbles
3. **Pouring**: Into mold or onto substrate
4. **Curing**: 60-80°C for 2-4 hours (or room temperature for 24-48 hours)
5. **Demolding**: Carefully peel or release from mold

**Critical Parameters**:
- **Crosslinker ratio**: Controls stiffness (more crosslinker = stiffer)
- **Curing time/temperature**: Affects crosslink density and mechanical properties
- **Degassing thoroughness**: Bubbles cause defects and weak points
- **Mold surface treatment**: Silanization prevents adhesion

**Microstructured Substrates**:

Creating surface textures enhances sensor performance:

**Master Fabrication**:
- Photolithography on silicon wafers
- Features: Pyramids, pillars, ridges (1-100 μm size)
- Materials: Photoresist (SU-8, AZ series) or etched silicon
- Resolution: Down to <1 μm with proper equipment

**Replica Molding**:
- Pour PDMS over master
- Cure and demold (surface features transferred)
- Master can be reused 50-100+ times
- Negative features become positive and vice versa

**Advanced Microstructures**:
- Interlocked microdomes: Enhanced normal and shear sensing
- Hierarchical structures: Multiple size scales (nano + micro)
- Bio-inspired patterns: Fingerprint ridges, lotus leaf textures

### 3.1.2 Spin Coating

Produces thin, uniform films:

**Process**:
1. Dispense polymer solution on substrate
2. Spin at 500-6000 RPM
3. Solvent evaporates, leaving thin film
4. Optional heat treatment for complete solvent removal

**Film Thickness Control**:
- Thickness ∝ 1/√(spin speed)
- Typical range: 0.1-10 μm
- Controlled by solution concentration and spin parameters

**Applications**:
- Thin encapsulation layers
- Dielectric layers for capacitive sensors
- Substrate for ultra-thin e-skin (<10 μm)

### 3.1.3 Extrusion and Calendaring

For high-volume production:

**Extrusion**:
- Thermoplastic elastomers (TPU, TPE) heated and forced through die
- Continuous process: Hundreds of meters per hour
- Thickness: 0.1-5 mm typically
- Can create multilayer structures

**Calendaring**:
- Polymer passed through rollers to create sheets
- Precise thickness control
- Surface texturing possible with patterned rollers
- High volume, low cost

## 3.2 Conductive Layer Deposition

Applying conductors to flexible substrates requires techniques compatible with polymer processing.

### 3.2.1 Solution Processing

**Spray Coating**:

Most versatile method for research and small production:

**Equipment**:
- Airbrush: Manual, good for small areas
- Automated spray coater: Computer-controlled X-Y stage
- Ultrasonic spray: Better uniformity, less waste

**Process**:
1. Prepare dispersion (AgNW, CNT, graphene in solvent)
2. Heat substrate to 40-80°C (accelerates drying)
3. Spray in sweeping motion or automated pattern
4. Multiple passes to build up desired sheet resistance
5. Dry and optional annealing

**Parameters**:
- Nozzle distance: 10-30 cm
- Air pressure: 10-30 PSI
- Flow rate: 0.1-1 mL/min
- Substrate temperature: 40-100°C

**Advantages**:
- Works on any substrate shape
- Minimal material waste
- Easy to create gradients or patterns
- Scalable to large areas

**Spin Coating for Conductive Films**:
- Similar to substrate spin coating
- Challenge: Maintaining uniform coverage on elastomers
- Often requires adhesion promoter
- Good for small-area devices

**Dip Coating**:
- Substrate immersed in solution, withdrawn at controlled rate
- Coating thickness controlled by withdrawal speed
- Uniform coating on complex shapes
- Used for CNT or graphene oxide films

### 3.2.2 Printing Technologies

**Inkjet Printing**:

Digital, additive patterning:

**Principles**:
- Piezoelectric or thermal droplet ejection
- Droplet volume: 1-100 pL
- Resolution: 20-100 μm
- Requires low-viscosity inks (<20 cP)

**Conductive Inks**:
- Silver nanoparticle inks (most common)
- PEDOT:PSS (lower conductivity but biocompatible)
- Graphene inks (emerging)
- Post-treatment: Sintering (thermal, photonic, or chemical)

**Advantages**:
- Fully digital (no masks)
- Material efficient
- Multi-material printing possible
- Good for prototyping

**Limitations**:
- Limited to thin films
- Nozzle clogging with nanoparticles
- Resolution limited for stretchable inks
- Slower than screen printing

**Screen Printing**:

High-throughput patterning:

**Process**:
1. Create screen with open mesh in desired pattern
2. Place screen on substrate
3. Force ink through mesh with squeegee
4. Lift screen and cure ink

**Parameters**:
- Mesh count: 50-400 threads per inch (higher = finer features)
- Ink viscosity: 1000-100,000 cP (paste-like)
- Squeegee pressure and speed
- Snap-off distance: Gap between screen and substrate

**Capabilities**:
- Feature size: 50-500 μm typically
- Film thickness: 1-100 μm
- Speed: >100 prints per hour
- Excellent for electrodes and interconnects

**Transfer Printing**:

Picking up and placing pre-fabricated structures:

**Micro-Transfer Printing**:
- Fabricate components on donor substrate
- Use elastomer stamp to pick up
- Transfer to receiving substrate
- Enables integration of conventional electronics on flexible substrates

**Applications**:
- Transferring silicon sensors to e-skin
- CVD graphene transfer
- Printed circuit integration

### 3.2.3 Physical Vapor Deposition (PVD)

For high-quality metal films:

**Thermal Evaporation**:
- Metal heated in vacuum until it evaporates
- Condenses on substrate
- Film thickness: 10-1000 nm
- Deposition rate: 0.1-10 nm/s

**Sputtering**:
- Plasma bombards metal target, ejects atoms
- Atoms deposit on substrate
- Better step coverage than evaporation
- Can deposit alloys and insulators

**Challenges with Flexible Substrates**:
- Rigid metal films crack when bent (>1-2% strain)
- Solutions:
  - Very thin films (<50 nm) more flexible
  - Serpentine patterns to accommodate strain
  - Deposit on pre-stretched substrate

## 3.3 Patterning Techniques

Creating spatially defined structures:

### 3.3.1 Photolithography

Standard microfabrication process:

**Process**:
1. Coat substrate with photoresist (spin coating)
2. Soft bake to remove solvent
3. Expose through photomask with UV light
4. Develop to remove exposed (positive resist) or unexposed (negative resist) areas
5. Pattern transfer by etching or lift-off
6. Remove remaining resist

**Resolution**:
- Contact lithography: ~1 μm
- Projection lithography: <0.5 μm
- Limited by light wavelength and diffraction

**Challenges on Flexible Substrates**:
- Substrate must be flat during exposure
- Some elastomers absorb photoresist solvents
- Thermal expansion mismatch

**Solutions**:
- Use rigid carrier during processing
- Select compatible resist/substrate combinations
- Pattern on rigid substrate, then transfer

### 3.3.2 Soft Lithography

Developed specifically for flexible materials:

**Microcontact Printing**:
- PDMS stamp inked with molecules
- Stamp pressed onto substrate, transfers ink pattern
- Ink can be self-assembled monolayers, nanoparticles, or catalysts
- Resolution: <100 nm possible

**Micromolding**:
- PDMS mold filled with prepolymer
- Cure polymer in mold
- Demold to reveal microstructured part

**Advantages**:
- Works on curved and flexible surfaces
- Low cost (single master makes many stamps)
- Ambient conditions (no vacuum or clean room)

### 3.3.3 Laser Patterning

Maskless digital patterning:

**Laser Cutting/Ablation**:
- CO₂ laser (10.6 μm wavelength): Cuts polymers cleanly
- UV laser (355 nm): Finer features, less thermal damage
- Resolution: 10-100 μm depending on system

**Applications**:
- Cutting e-skin to shape
- Ablating conductive films to create patterns
- Via drilling for interconnections

**Laser Sintering**:
- Selective sintering of nanoparticle inks
- Converts nanoparticle films to continuous conductive paths
- Fast (milliseconds), localized heating

## 3.4 Integration and Assembly

Combining components into functional devices:

### 3.4.1 Layer-by-Layer Assembly

Typical e-skin structure (bottom to top):
1. Substrate layer (PDMS, PU): Mechanical support
2. Bottom electrode (AgNW, PEDOT:PSS): Reference or ground
3. Sensing layer (dielectric, piezoresistive composite, etc.)
4. Top electrode (AgNW, PEDOT:PSS): Signal collection
5. Encapsulation layer: Protection and biocompatibility

**Bonding Methods**:

**Adhesive Bonding**:
- Thin layer of silicone adhesive or acrylic
- Apply, align layers, cure
- Challenge: Adhesive layer affects mechanical properties

**Oxygen Plasma Bonding**:
- Activate surfaces with O₂ plasma
- Brings surfaces into contact
- Covalent bonding occurs
- Permanent, strong bond for silicones

**Heat Bonding**:
- For thermoplastic elastomers
- Heat above glass transition temperature
- Press layers together
- Cool to form bond

### 3.4.2 Electronics Integration

Connecting e-skin to processing and wireless systems:

**Rigid-Flex Integration**:
- Flexible e-skin connected to rigid PCB
- Connector types:
  - Anisotropic conductive film (ACF): Conductive in Z-direction only
  - Zero insertion force (ZIF) connectors
  - Soldered connections to flexible tails

**Stretchable Electronics**:
- Island-bridge design:
  - Rigid islands contain conventional electronics
  - Serpentine interconnects between islands stretch
- Chip-on-flex:
  - Thin silicon chips bonded to flexible substrate
  - Can bend but not stretch significantly

**Wireless Modules**:
- BLE system-on-chip (e.g., Nordic nRF52)
- Antenna: Printed or etched on flexible substrate
- Battery: Thin Li-polymer or coin cells
- Power management IC

### 3.4.3 Encapsulation

Protecting devices from environment:

**Polymer Encapsulation**:
- Spray or spin coat thin polymer layer
- Materials: PDMS, PU, Parylene-C
- Thickness: 1-50 μm
- Provides mechanical and moisture protection

**Parylene-C Deposition**:
- Chemical vapor deposition process
- Conformal coating, penetrates into crevices
- Excellent moisture barrier
- USP Class VI biocompatible
- Thickness: 0.1-25 μm

**Multilayer Barriers**:
- Alternate organic and inorganic layers
- E.g., polymer/Al₂O₃/polymer/Al₂O₃/polymer
- Inorganic layers block moisture
- Organic layers accommodate flexibility
- Water vapor transmission rate <10⁻⁴ g/m²/day achievable

## 3.5 Scalable Manufacturing Approaches

Transitioning from lab to factory:

### 3.5.1 Roll-to-Roll Processing

Continuous manufacturing on flexible substrates:

**Process Flow**:
1. Unwind substrate from roll
2. Sequential processing stations:
   - Coating (slot-die, gravure, spray)
   - Patterning (screen printing, flexography)
   - Drying/curing (IR lamps, ovens)
   - Inspection (cameras, sensors)
3. Rewind onto roll or cut to sheets

**Capabilities**:
- Speed: 1-100 m/min depending on process
- Width: 0.3-2 m typical
- Can process kilometers of material per day

**Challenges**:
- Registration: Aligning layers within ±25 μm
- Tension control: Maintaining uniform substrate tension
- Defect management: Identifying and marking defects
- Process control: Ensuring consistency over long runs

**Example Process for E-Skin**:
1. PU film unwind
2. Slot-die coat PEDOT:PSS bottom electrode
3. IR dry
4. Screen print piezoresistive sensing layer
5. UV cure
6. Screen print PEDOT:PSS top electrodes
7. IR dry
8. Laminate protective layer
9. Die cut to individual sensors
10. Rewind or sheet

### 3.5.2 Injection Molding

For high-volume thermoplastic components:

**Process**:
1. Heat thermoplastic elastomer above melting point
2. Inject into mold under pressure
3. Cool to solidify
4. Eject part

**Advantages**:
- Very high production rate (thousands per hour)
- Excellent repeatability
- Complex 3D geometries
- Low per-unit cost at scale

**Multi-Material Molding**:
- Overmolding: Mold second material over first part
- Two-shot molding: Two materials in one mold
- Can create e-skin with integrated soft and hard regions

**Integration with E-Skin**:
- Pre-molded substrate with integrated features
- In-mold decoration: Placing conductive films in mold before injection
- Insert molding: Encapsulating sensors or electronics

### 3.5.3 3D Printing

Additive manufacturing for customization:

**Fused Deposition Modeling (FDM)**:
- Extrude molten thermoplastic layer-by-layer
- Materials: TPU (flexible), PLA, ABS (rigid)
- Resolution: 100-400 μm
- Can print multi-material structures

**Direct Ink Writing (DIW)**:
- Extrude viscoelastic inks through nozzle
- Inks: Silicones, hydrogels, conductive pastes
- Can print embedded sensors and conductors
- Resolution: 50-500 μm

**Stereolithography (SLA)**:
- UV light polymerizes liquid resin layer-by-layer
- Resolution: 25-100 μm (better than FDM)
- Flexible resins available
- Smoother surfaces than FDM

**Multi-Material Printing**:
- Print soft substrate and conductive traces in one process
- Enables complex integrated structures
- Example: 3D print prosthetic socket with integrated e-skin

**Advantages for E-Skin**:
- Customization: Patient-specific prosthetic interfaces
- Rapid prototyping: Design iteration in hours
- Complex geometries: Impossible with traditional methods

**Limitations**:
- Slower than molding (minutes to hours per part)
- Material properties may not match cast/molded parts
- Limited conductive material options

## 3.6 Quality Control and Testing

Ensuring device performance and reliability:

### 3.6.1 In-Process Monitoring

**Thickness Measurement**:
- Contact: Micrometer, profilometer
- Non-contact: Optical interferometry, ellipsometry
- Target: ±5% of specification

**Sheet Resistance**:
- Four-point probe measurement
- Maps conductivity uniformity
- Target: <10% variation across device

**Visual Inspection**:
- Optical microscopy for defects
- Automated optical inspection (AOI) for production
- Detect: Bubbles, particles, cracks, misalignment

### 3.6.2 Functional Testing

**Mechanical Testing**:
- Tensile testing: Measure Young's modulus, elongation at break
- Cyclic stretching: Validate >10,000 cycles per WIA-SEMI-016
- Bend testing: Minimum radius without failure

**Electrical Testing**:
- I-V curves: Check for shorts, opens, proper resistance
- Capacitance measurement: For capacitive sensors
- Signal-to-noise ratio: Minimum 40 dB per standard

**Sensing Performance**:
- Pressure response: Calibrated force application, measure output
- Sensitivity: ΔR/R or ΔC/C per kPa
- Linearity: R² > 0.95 over working range
- Hysteresis: <10% between loading and unloading

**Environmental Testing**:
- Temperature cycling: -20°C to +60°C
- Humidity: 30% to 90% RH
- Validate performance across conditions

### 3.6.3 Reliability Testing

**Accelerated Life Testing**:
- Continuous operation at elevated temperature
- Accelerates degradation mechanisms
- Arrhenius equation to extrapolate lifetime

**Mechanical Durability**:
- Cyclic stretching/bending to failure
- Abrasion resistance
- Tear propagation

**Biocompatibility Screening**:
- Extractables and leachables analysis
- Cytotoxicity testing on production samples
- Ensure batch-to-batch consistency

## 3.7 Cost Analysis

Understanding economics of e-skin production:

### Material Costs (per device, 100 cm² area):

**Research Prototype**:
- Substrate (PDMS): $2-5
- Conductor (AgNW): $3-10
- Electronics: $5-20
- **Total**: $10-35 per device

**Low-Volume Production (1,000 units/year)**:
- Substrate (medical silicone): $1-3
- Conductor (screen printed Ag): $0.50-2
- Electronics: $3-10
- Assembly labor: $5-15
- **Total**: $10-30 per device

**High-Volume Production (100,000+ units/year)**:
- Substrate (TPU, molded): $0.10-0.50
- Conductor (roll-to-roll printed): $0.05-0.20
- Electronics: $1-3 (bulk pricing)
- Automated assembly: $0.50-2
- **Total**: $2-6 per device

**Target for Consumer Products**: <$1 per device at million-unit scale

### Manufacturing Equipment Costs:

**Research Lab**:
- Spin coater: $5,000-15,000
- Spray coater: $2,000-10,000
- Curing oven: $1,000-5,000
- **Total**: ~$50,000 for basic setup

**Pilot Production**:
- Semi-automated screen printer: $20,000-50,000
- Laminator: $10,000-30,000
- Laser cutter: $15,000-50,000
- Testing equipment: $50,000-100,000
- **Total**: ~$300,000

**High-Volume Manufacturing**:
- Roll-to-roll line: $500,000-5,000,000
- Injection molding: $100,000-500,000 per mold
- Automated assembly: $200,000-1,000,000
- Quality control: $100,000-500,000
- **Total**: $5-20 million for complete facility

## 3.8 Best Practices

Lessons learned from e-skin fabrication:

### Material Handling:
- Store elastomers in cool, dry environment
- Nanomaterial dispersions: Sonicate before use
- Prevent contamination: Clean room or laminar flow hood

### Process Control:
- Document everything: Materials, parameters, results
- Maintain consistency: Calibrate equipment regularly
- Test samples from each batch

### Troubleshooting:
- Poor adhesion: Clean surfaces, use adhesion promoter
- Bubbles: Improve degassing, reduce deposition speed
- Non-uniform coating: Optimize solution concentration and deposition parameters
- High resistance: Improve conductor connectivity, increase loading

### Safety:
- Nanomaterials: Avoid inhalation, use fume hood
- Solvents: Proper ventilation, disposal
- UV exposure: Eye protection during photolithography

The path from laboratory curiosity to commercial product is long and requires expertise across materials, processes, equipment, and economics. But with careful planning and execution, electronic skin can be manufactured at scale and cost that makes it accessible to those who need it most.

---

**Next Chapter**: Sensing Technologies - Physics and Implementation of Multi-Modal Sensors
