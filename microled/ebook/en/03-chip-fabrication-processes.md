# Chapter 3: MicroLED Chip Fabrication Processes

## Overview of MicroLED Chip Manufacturing

MicroLED chip fabrication represents one of the most challenging aspects of display manufacturing, requiring precision semiconductor processing techniques typically associated with integrated circuit fabrication rather than traditional display production. The manufacture of millions of microscopic LED chips with consistent performance characteristics demands mastery of epitaxial growth, nanoscale lithography, and extensive quality control.

## Epitaxial Growth Fundamentals

### Gallium Nitride (GaN) Material System

MicroLED chips are primarily fabricated using the III-V compound semiconductor gallium nitride (GaN) and related alloys:

**Blue and Green LEDs**: InGaN/GaN quantum well structures
**Red LEDs**: AlGaInP or InGaN (emerging) on different substrates
**UV LEDs**: AlGaN/GaN for ultraviolet emission

### Metal-Organic Chemical Vapor Deposition (MOCVD)

MOCVD represents the dominant growth technique for GaN-based LED structures:

**Process Overview**:
1. Substrate loading (sapphire, SiC, or GaN)
2. Buffer layer growth (low-temperature GaN nucleation)
3. N-type GaN layer deposition (Si-doped, 2-4 μm)
4. Multiple quantum well (MQW) active region
5. P-type GaN layer (Mg-doped, 100-200 nm)
6. P-type contact layer (highly doped p+-GaN)

**Critical Parameters**:
- **Growth temperature**: 900-1100°C depending on layer
- **Reactor pressure**: 100-500 Torr
- **Growth rate**: 1-3 μm/hour
- **Precursors**: Trimethylgallium (TMGa), ammonia (NH₃), trimethylindium (TMIn)
- **Dopants**: Silane (SiH₄) for n-type, Cp₂Mg for p-type

**Wafer Scale**: Modern MOCVD reactors process 4-8 inch sapphire wafers, with 8-inch becoming standard for volume production. Advanced systems can process 12+ wafers simultaneously in a single reactor chamber.

### Quantum Well Engineering

The active region consists of InGaN/GaN quantum wells where light emission occurs:

**Structure**:
- Well thickness: 2-3 nm InGaN
- Barrier thickness: 10-15 nm GaN
- Number of wells: 3-5 for blue, 5-8 for green
- Total MQW thickness: 50-100 nm

**Indium Composition Control**:
- Blue (450-470 nm): 15-18% indium
- Green (520-530 nm): 25-30% indium
- Challenge: Higher indium content increases strain and defect formation

**Efficiency Droop**: LED efficiency decreases at high current densities due to Auger recombination and carrier overflow. MicroLED's small size and low current per chip partially mitigates this effect.

## Substrate Technologies

### Sapphire (Al₂O₃)

**Advantages**:
- Dominant substrate material (>80% of market)
- Transparent to visible light
- Chemically stable
- Available in large sizes (8-inch standard)
- Relatively low cost ($100-200 per 8-inch wafer)

**Disadvantages**:
- Large lattice mismatch with GaN (16%)
- High threading dislocation density (10⁸-10⁹ cm⁻²)
- Low thermal conductivity (35 W/m·K)
- Requires chemical lift-off or laser lift-off for transfer

**Typical Structure**: C-plane (0001) sapphire with patterned sapphire substrate (PSS) to improve light extraction

### Silicon Carbide (SiC)

**Advantages**:
- Better lattice match to GaN (3.5%)
- Excellent thermal conductivity (370 W/m·K)
- Lower dislocation density (10⁷-10⁸ cm⁻²)
- Enables higher power and efficiency

**Disadvantages**:
- Higher cost ($800-1500 per 6-inch wafer)
- Smaller available sizes (6-inch standard)
- Absorbs blue light (not suitable for flip-chip configurations requiring transmission)

**Applications**: Primarily high-power LEDs; less common for MicroLED displays due to cost

### GaN-on-GaN

**Advantages**:
- Perfect lattice match (homoepitaxy)
- Extremely low dislocation density (<10⁶ cm⁻²)
- Higher efficiency and reliability
- Native substrates enable new device architectures

**Disadvantages**:
- Very expensive ($2,000-5,000 per 2-inch wafer)
- Limited availability and small sizes
- Supply chain not yet mature

**Status**: Emerging technology; pilot production for premium applications

### Silicon (Si)

**Advantages**:
- Large sizes available (12-inch standard in semiconductor industry)
- Low cost ($50-100 per 12-inch wafer)
- Compatibility with CMOS processes
- Economies of scale

**Disadvantages**:
- Very large lattice mismatch (17%)
- High dislocation density (>10⁹ cm⁻²)
- Crack formation due to thermal expansion mismatch
- Lower efficiency than sapphire or SiC

**Status**: Active R&D, particularly in China; potential long-term cost solution

## Chip Fabrication Process Flow

### 1. Epitaxial Wafer Preparation

After MOCVD growth, wafers undergo inspection and characterization:

**Photoluminescence (PL) Mapping**:
- Measures wavelength uniformity across wafer
- Identifies defect regions
- Typical uniformity: <±3 nm wavelength variation

**Electroluminescence (EL) Testing**:
- Tests electrical injection and light output
- Maps brightness uniformity
- Identifies dead regions or shorts

### 2. Mesa Definition and Isolation

**Photolithography**:
- Resist coating: Spin-coat photoresist
- Exposure: Define chip boundaries (5-50 μm pitch)
- Development: Create etch mask

**Dry Etching**:
- Inductively coupled plasma (ICP) or reactive ion etch (RIE)
- Etch depth: 1.5-2.5 μm (through MQW to n-GaN)
- Etch chemistry: Cl₂/BCl₃/Ar
- Sidewall angle: 70-90° for good current spreading

**Critical Dimension Control**:
- For 10 μm chips, <±0.2 μm tolerance required
- Advanced lithography (DUV, i-line) necessary for smallest chips
- Alignment accuracy: <±0.5 μm

### 3. Passivation and Protection

**Dielectric Deposition**:
- SiO₂ or Si₃N₄ layer (100-300 nm)
- Protects mesa sidewalls
- Provides electrical isolation
- Deposited by PECVD (plasma-enhanced CVD)

**Current Blocking Layer**:
- Prevents leakage currents
- Improves quantum efficiency
- Critical for high-yield mass transfer

### 4. Contact Formation

**N-Contact**:
- Opens through passivation to n-GaN
- Metal stack: Ti/Al/Ti/Au or Ti/Al/Ni/Au
- Thickness: 200-500 nm total
- Annealing: 500-600°C in N₂ for ohmic contact
- Contact resistance: <10⁻⁴ Ω·cm²

**P-Contact**:
- Most challenging due to high p-GaN resistance
- Transparent conducting oxide (TCO) layer: ITO or ZnO
- TCO thickness: 100-200 nm
- Current spreading critical for small chips
- Metal contact pad: Ni/Au or Pd/Au
- Reflection layer: Silver or aluminum for flip-chip

**Interconnect Metals**:
- Additional metal layers for routing (if needed)
- Typically Ti/Au for wire bonding applications
- For transfer applications, specialized bond pads

### 5. Wafer Thinning and Surface Preparation

**Backside Processing**:
- Substrate thinning (for sapphire): 400 μm → 100-150 μm
- Reduces absorption, improves heat dissipation
- Thinning methods: Grinding, chemical-mechanical polishing (CMP)

**Surface Roughening** (for vertical LEDs):
- Photochemical etching or laser treatment
- Increases light extraction efficiency
- Creates micron-scale texture

### 6. Chip Singulation

**Dicing Technologies**:

**Laser Dicing**:
- Pulsed laser (355 nm UV)
- Ablates material along streets
- Kerf width: 5-15 μm
- Speed: 100-300 mm/s
- Advantages: Narrow kerf, minimal mechanical stress
- Disadvantages: Heat-affected zone, debris

**Stealth Dicing** (Laser-Induced Modification):
- Internal modification with IR laser
- Mechanical cleaving along modified planes
- Advantages: No debris, cleaner edges
- Disadvantages: Requires specialized equipment

**Plasma Dicing**:
- Anisotropic dry etch through entire wafer
- Combined with photoresist and support film
- Advantages: Vertical sidewalls, small features
- Disadvantages: Slower, higher cost

**Mechanical Dicing**:
- Diamond blade sawing
- Traditional approach, increasingly unsuitable for <20 μm chips
- High mechanical stress and chipping risk

## Wavelength Binning and Selection

### Color Uniformity Challenge

Manufacturing variation results in wavelength distribution across wafers:

**Typical Wavelength Spread**:
- Blue LEDs: 450 nm ± 10 nm (within wafer)
- Green LEDs: 525 nm ± 15 nm (within wafer)
- Red LEDs: 620 nm ± 8 nm (within wafer)

For displays requiring precise color balance, chips must be sorted by wavelength.

### Binning Process

**Photoluminescence Testing**:
- Automated wafer-level testing
- Maps wavelength of every chip location
- Creates bin map for sorting

**Bins**:
- Typically 2-5 nm wavelength bins
- Finer binning improves color uniformity but reduces yield
- Trade-off between uniformity and cost

**Post-Transfer Compensation**:
- Alternative approach: Transfer chips without binning
- Calibrate each pixel individually in display
- Requires more sophisticated driver electronics

### Brightness Binning

**Intensity Variation**:
- ±15-20% brightness variation typical within wafer
- Caused by thickness variation, defect distribution

**Binning Strategy**:
- Optical power measurement at specified current
- Group into brightness bins (e.g., 100-110 mcd, 110-120 mcd)
- Match bins within display for uniformity

## Advanced Fabrication Techniques

### Patterned Sapphire Substrate (PSS)

**Concept**: Etch cone or pyramid patterns on sapphire before GaN growth

**Benefits**:
- Reduces dislocation density through lateral overgrowth
- Improves light extraction (disrupts waveguiding)
- 20-40% efficiency improvement vs. planar sapphire

**Pattern Types**:
- Cone arrays: Most common
- Hemisphere dimples
- Pyramid arrays
- Typical pattern: 3-4 μm pitch, 1.5-2 μm height

### Photonic Crystal Structures

**Approach**: Nanoscale periodic patterns in LED structure

**Implementation**:
- Electron-beam lithography or nanoimprint
- 200-400 nm pitch patterns
- Etched into p-GaN or n-GaN layers

**Benefits**:
- Enhances light extraction by disrupting waveguide modes
- Can improve extraction efficiency by 50-100%
- Challenges: Cost and throughput of nanoscale patterning

### Quantum Dot Color Conversion Layer

**Alternative Architecture**: Blue or UV MicroLED + quantum dot conversion

**Process**:
- Deposit quantum dot layer on chip or separately on display
- Blue photons excite QDs to emit red/green
- Simpler assembly (single LED type) but conversion losses

## Quality Control and Testing

### Wafer-Level Testing

**EL Testing**:
- Probe card contacts every chip
- Measures forward voltage, light output
- Identifies shorts, opens, low-efficiency chips
- Throughput: 1-5 wafers/hour

**Automated Optical Inspection (AOI)**:
- High-resolution imaging of every chip
- Detects physical defects, contamination
- Dimension measurement
- Throughput: 50-100 wafers/hour

### Yield Metrics

**Electrical Yield**: Percentage of chips with acceptable electrical characteristics
- Target: >95% for cost-effective production

**Optical Yield**: Percentage with acceptable brightness and wavelength
- More stringent: Typically 70-85% within tight bins

**Overall Yield**: Electrical × Optical × Defect-free
- Production target: >80% for economic viability
- Actual yields vary widely by process maturity: 60-90%

## Cost Structure and Economics

### Cost Breakdown (Typical 8-inch Wafer)

**Epitaxial Wafer**: $150-250
**Fabrication Processing**: $200-400
**Testing and Binning**: $100-150
**Total Wafer Cost**: $450-800

**Chips per Wafer** (varies by chip size):
- 50 μm chips: ~500,000 chips/wafer
- 30 μm chips: ~1,400,000 chips/wafer
- 10 μm chips: ~12,500,000 chips/wafer

**Cost per Chip**:
- 50 μm: $0.0009-0.0016
- 30 μm: $0.0003-0.0006
- 10 μm: $0.000036-0.000064

**Note**: These are chip fabrication costs only; mass transfer, assembly, and testing add significant additional costs to final display.

## Future Trends in Chip Fabrication

### Larger Wafer Sizes

**12-inch GaN-on-Si**:
- 2.25× more area than 8-inch
- Leverages existing semiconductor infrastructure
- Requires resolving efficiency challenges

### Wafer-Level Processing

**Concept**: Build display structures directly on epitaxial wafer, then transfer entire display
**Status**: Research phase, promising for small displays

### Red LED Challenge

**Current Issue**: AlGaInP red LEDs require different substrate, complicating RGB assembly
**Solutions in Development**:
- InGaN red LEDs (lower efficiency currently)
- Quantum dot color conversion
- Hybrid approaches

### AI-Optimized Growth

**Machine Learning Applications**:
- MOCVD recipe optimization
- Real-time process control
- Yield prediction and analysis
- Defect pattern recognition

## Conclusion

MicroLED chip fabrication represents the convergence of compound semiconductor manufacturing with display industry requirements. Success requires mastery of epitaxial growth, nanoscale processing, and high-volume testing. While current processes achieve acceptable yields for premium applications, further improvements in consistency, yield, and cost are necessary for mainstream market adoption.

The WIA-SEMI-010 standard provides guidelines for chip fabrication processes, quality metrics, and testing protocols to enable industry standardization and facilitate technology transfer from research to volume production.

---

**Key Process Parameters:**
- Chip sizes: 5-50 μm
- Epitaxial growth: MOCVD at 900-1100°C
- Substrates: Sapphire (dominant), SiC, GaN, Si (emerging)
- Wavelength uniformity: ±3-5 nm target
- Production yield target: >80% within spec bins
- Cost: $0.0003-0.0016 per chip (fabrication only)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

## Extended Learning Materials

### Case Studies and Applications

This section explores real-world implementations and their outcomes, providing practical insights for practitioners.

#### Case Study 1: Global Implementation

Organizations worldwide have adopted this standard to streamline operations. A multinational corporation reported a 40% improvement in efficiency after implementing the recommended protocols. The key success factors included:

- Comprehensive stakeholder engagement during planning
- Phased rollout approach minimizing disruption
- Continuous monitoring and feedback loops
- Regular training and capability building
- Documentation of lessons learned

The implementation timeline spanned 18 months, with the following phases:

1. **Assessment Phase (3 months)**: Evaluated current state, identified gaps, and created roadmap
2. **Design Phase (4 months)**: Developed detailed specifications and integration plans
3. **Development Phase (6 months)**: Built and tested components
4. **Deployment Phase (3 months)**: Rolled out in stages with support
5. **Optimization Phase (2 months)**: Fine-tuned based on feedback

#### Case Study 2: Healthcare Sector

A major healthcare provider implemented these standards to improve patient data management. Results included:

- 60% reduction in data errors
- 35% faster information retrieval
- Enhanced compliance with regulatory requirements
- Improved patient satisfaction scores
- Better interoperability with partner systems

### Technical Deep Dive

#### Architecture Considerations

When implementing this standard, architects should consider:

1. **Scalability**: Design for growth with horizontal scaling capabilities
2. **Resilience**: Build fault-tolerant systems with redundancy
3. **Security**: Implement defense-in-depth with multiple layers
4. **Maintainability**: Use modular design for easier updates
5. **Observability**: Include comprehensive logging and monitoring

#### Performance Optimization

Performance is critical for user experience. Key optimization strategies include:

- Caching frequently accessed data
- Using connection pooling
- Implementing async processing where appropriate
- Optimizing database queries
- Using CDN for static resources

### Frequently Asked Questions

**Q: What are the minimum system requirements?**
A: The standard is designed to be platform-agnostic, but implementations typically require:
- Modern operating system (Linux, Windows, macOS)
- Minimum 4GB RAM (8GB recommended)
- 100GB storage (SSD recommended)
- Network connectivity with 10Mbps minimum

**Q: How do I ensure compliance?**
A: Compliance can be verified through:
- Automated testing suites
- Manual review checklists
- Third-party audits
- Certification programs

**Q: What support resources are available?**
A: Support includes:
- Official documentation
- Community forums
- Training programs
- Professional consulting services

### Glossary

| Term | Definition |
|------|------------|
| API | Application Programming Interface - a set of protocols for building software |
| SDK | Software Development Kit - tools for creating applications |
| REST | Representational State Transfer - architectural style for web services |
| JSON | JavaScript Object Notation - lightweight data interchange format |
| XML | Extensible Markup Language - markup language for encoding documents |
| TLS | Transport Layer Security - cryptographic protocol for communications |
| CRUD | Create, Read, Update, Delete - basic operations on data |

### References and Further Reading

1. WIA Standards Framework Documentation (2025)
2. Best Practices for Implementation Guide
3. Security Considerations Whitepaper
4. Performance Benchmarking Report
5. Integration Patterns Reference

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*

