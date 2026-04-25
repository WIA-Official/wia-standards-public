# Chapter 4: Fan-Out Wafer-Level Packaging

## 4.1 Introduction to Fan-Out Technology

Fan-Out Wafer-Level Packaging (FOWLP) represents a revolutionary departure from traditional packaging approaches, redistributing I/O connections beyond the die boundary at the wafer level rather than requiring a separate substrate. This elegant solution addresses several critical challenges in semiconductor packaging, including the need for finer-pitch connections, thinner packages, and improved electrical performance—all while potentially reducing costs through higher manufacturing efficiency.

The fundamental innovation of fan-out packaging lies in its ability to "fan out" the die's I/O connections beyond its physical boundary, creating a larger area for external connections while maintaining fine-pitch connections directly to the die. This is accomplished by embedding the die in a molding compound and creating redistribution layers (RDL) that route signals from the die's fine-pitch pads to larger, more coarse-pitch connections suitable for board-level assembly.

### Historical Development

The concept of wafer-level packaging dates back to the 1990s, but early implementations were limited to Fan-In Wafer-Level Packaging (FIWLP), where all I/O connections remained within the die footprint. While FIWLP offered advantages in package thickness and cost for low I/O count devices, it could not address the needs of modern high-performance processors with thousands of I/O connections.

The breakthrough came with the development of fan-out technologies that allowed I/O redistribution beyond the die boundary. Infineon pioneered the embedded Wafer-Level BGA (eWLB) process in the mid-2000s, demonstrating that fan-out packaging could be manufactured at scale. TSMC's subsequent development of Integrated Fan-Out (InFO) technology and its adoption by Apple for the iPhone's A-series processors validated fan-out as a viable solution for high-volume, high-performance applications.

### Key Advantages of Fan-Out Packaging

Fan-out packaging offers several compelling advantages over traditional packaging approaches:

**Thin Profile**: By eliminating the package substrate, fan-out packages achieve thickness of less than 1mm, critical for mobile and wearable applications where device thickness is a key design constraint.

**Improved Electrical Performance**: The shorter interconnect paths and elimination of wire bonds or substrate routing reduce parasitic inductance and resistance, improving signal integrity and enabling higher operating frequencies.

**Enhanced Thermal Performance**: Direct contact between the die backside and the board or heat sink provides an efficient thermal path, improving thermal dissipation compared to substrate-based packages where the die is thermally isolated by the substrate.

**Design Flexibility**: The RDL process allows flexible routing of signals, enabling optimization of power delivery networks, signal routing, and I/O placement without the constraints of traditional substrate manufacturing.

**Cost Reduction Potential**: For appropriate applications, fan-out packaging can reduce costs by eliminating the substrate and simplifying the assembly process. The wafer-level processing approach also benefits from economy of scale.

## 4.2 Fan-Out Manufacturing Process

The fan-out manufacturing process involves several unique steps that distinguish it from traditional packaging:

### Die Preparation and Placement

**Wafer Dicing**: Known good dies are singulated from processed wafers using either blade dicing or laser dicing. Die attach film (DAF) or other temporary adhesives may be applied to facilitate subsequent handling.

**Carrier Wafer Preparation**: A temporary carrier wafer (typically glass or silicon) is prepared with a release layer that will allow separation after molding and RDL formation. The carrier provides mechanical support during subsequent processing and must withstand thermal and chemical stresses while maintaining flatness.

**Die Placement**: Individual dies are placed face-down on the carrier wafer in a predetermined pattern using high-precision pick-and-place equipment. The placement pattern determines the final package size and must account for:
- Desired RDL routing area
- Keep-out zones for molding process
- Registration marks for subsequent lithography steps
- Optimal utilization of the reconstituted wafer area

Placement accuracy is critical, typically requiring sub-10 μm precision to ensure proper alignment for subsequent RDL lithography.

### Compression Molding and Grinding

**Compression Molding**: Epoxy molding compound is applied to encapsulate the dies, filling the spaces between dies and covering their backsides. The compression molding process must:
- Ensure complete void-free filling around die edges
- Minimize die shift during molding
- Control compound thickness uniformly across the wafer
- Avoid damaging delicate die structures

Modern molding compounds are formulated with controlled coefficient of thermal expansion (CTE), filler particle size distribution, and rheological properties to meet these requirements.

**Post-Mold Cure**: After molding, the reconstituted wafer undergoes thermal curing to fully cross-link the epoxy molding compound, developing its final mechanical and thermal properties.

**Grinding and Polishing**: The molded surface is ground and polished to:
- Achieve uniform thickness across the reconstituted wafer
- Expose the die bond pads for RDL connection
- Create a smooth, planar surface suitable for photolithography
- Remove any molding compound residue from the pad surfaces

The grinding process must be carefully controlled to avoid die damage while achieving the required surface planarity and roughness. Chemical-mechanical polishing (CMP) may be employed for final surface preparation.

### Redistribution Layer (RDL) Formation

The RDL process creates the fine-pitch routing that connects the die pads to larger external connections:

**Dielectric Deposition**: A polymer or inorganic dielectric layer is deposited on the ground surface to electrically isolate the subsequent metal routing. Common materials include:
- Polyimide (PI): Good electrical properties, high temperature resistance, low cost
- Polybenzoxazole (PBO): Better planarization, lower moisture absorption
- Silicon dioxide or nitride: Superior electrical performance but higher stress

The dielectric must provide adequate electrical isolation while maintaining good adhesion to both the molding compound and the metal routing layers.

**Photolithography and Via Opening**: Photolithography defines the via openings where the RDL will contact the die pads. The lithography process must achieve:
- Accurate alignment to the die pads (typically ±5 μm or better)
- Sharp via profiles to ensure reliable metal contact
- Minimal residue that could compromise electrical contact

After lithography, the dielectric is etched (wet or dry etch) to expose the underlying die pads.

**Metal Seed Layer Deposition**: A thin adhesion/barrier layer (typically titanium or titanium-tungsten) followed by a copper seed layer is deposited by physical vapor deposition (PVD). This seed layer must conformally coat the via sidewalls and field areas to enable subsequent electroplating.

**Photoresist Patterning**: A second photolithography step defines the RDL routing pattern. Thick photoresist (typically 10-30 μm) is used to enable adequate metal thickness for current carrying and resistance requirements.

**Copper Electroplating**: Copper is electroplated into the photoresist-defined trenches and vias. Plating parameters must be optimized to:
- Ensure complete via filling without voids
- Achieve uniform metal thickness across the wafer
- Minimize plating-induced stress
- Control copper grain structure for optimal electrical properties

**Photoresist Strip and Seed Layer Etch**: After plating, the photoresist is removed (typically using solvent-based or plasma ashing), and the exposed seed layer is etched away, leaving the plated copper RDL pattern.

**Passivation Layer**: A final passivation layer (typically polyimide or photosensitive polymer) is applied and patterned to protect the RDL while exposing bond pad areas for external connections.

For complex packages requiring multiple RDL layers, this process is repeated, building up a multi-layer metal interconnection structure similar to on-chip metallization but at larger dimensions.

### Under-Bump Metallurgy and Solder Ball Attachment

**UBM Formation**: Under-bump metallurgy (UBM) provides a solderable surface and acts as a diffusion barrier. Common UBM stacks include:
- Ti/Cu: Simple, low-cost option
- Ti/Ni/Cu or Ti/NiV/Cu: Better barrier properties
- TiW/Cu: Alternative with good adhesion

The UBM is typically formed through a combination of PVD and electroplating or electroless plating.

**Solder Ball Attachment**: Solder balls (typically lead-free alloys such as SAC305) are placed on the UBM pads using:
- Ball placement equipment for larger pitches
- Solder paste printing and reflow for finer pitches
- Electroplating for the finest pitches

After placement, the assembly undergoes reflow to form metallurgical bonds between the solder and UBM.

### Carrier Release and Singulation

**Carrier Debonding**: The reconstituted wafer is released from the temporary carrier by:
- Thermal release (heating to decompose the adhesive)
- Laser release (laser ablation through the carrier to decompose the adhesive)
- Mechanical release (for purely mechanical adhesion)

The debonding process must ensure complete separation without damaging the fragile packages.

**Singulation**: Individual packages are separated by saw dicing or laser cutting. The singulation process must:
- Achieve clean cuts without chipping or cracking
- Minimize die stress during cutting
- Control kerf width to maximize reconstituted wafer utilization

**Final Testing**: Completed packages undergo electrical testing, visual inspection, and marking before shipment.

## 4.3 Fan-Out Variants and Architectures

Several fan-out variants have been developed to address different application requirements:

### Fan-Out Wafer-Level Packaging (FOWLP)

The original fan-out approach processes devices at the wafer level, creating a "reconstituted wafer" of embedded dies:

**TSMC InFO**: TSMC's Integrated Fan-Out technology has been used in high-volume production for Apple's A-series processors and other applications. InFO features:
- Fine-pitch RDL capabilities (≥2 μm line/space)
- Support for multiple RDL layers (up to 5+)
- High assembly yields through process optimization
- Variants including InFO-MS (multiple system-in-package) and InFO-AiP (antenna-in-package)

**ASE Fan-Out Chip-on-Substrate (FOCoS)**: Combines fan-out technology with a supporting substrate, offering advantages for applications requiring very high I/O counts or enhanced thermal/mechanical performance.

### Fan-Out Panel-Level Packaging (FOPLP)

Panel-level processing uses large rectangular panels rather than round wafers, potentially improving manufacturing efficiency:

**Larger Processing Area**: Panels (typically 500mm x 600mm or larger) provide more area for die placement than 300mm wafers, potentially reducing cost through economy of scale.

**Better Area Utilization**: Rectangular panels achieve better utilization of reconstituted area compared to round wafers, particularly for rectangular packages.

**Equipment Challenges**: Panel-level processing requires new equipment designed for larger, non-circular substrates, including:
- Panel-compatible die placers
- Large-area molding presses
- Panel lithography and plating tools
- Specialized handling systems

**Samsung FOPLP**: Samsung has been a leader in panel-level fan-out development, though adoption has been slower than initially anticipated due to equipment maturity and yield challenges.

### System-in-Package (SiP) Fan-Out

Advanced fan-out implementations integrate multiple dies and passive components in a single package:

**Multi-Die Integration**: Multiple dies (processor, memory, RF, power management, etc.) are placed on the carrier and interconnected through the RDL, creating complete subsystems in a single package.

**Passive Integration**: Resistors, capacitors, and inductors can be integrated within the RDL layers or surface-mounted on the package, reducing board-level component count.

**Antenna Integration (AiP)**: For wireless applications, antennas can be formed within or on the RDL layers, creating compact modules for 5G, WiFi, or other wireless standards. TSMC's InFO-AiP technology exemplifies this approach for mmWave 5G applications.

## 4.4 Design Considerations for Fan-Out Packaging

Successful fan-out package design requires careful attention to several key factors:

### Mechanical Design and Warpage

**Warpage Sources**: Fan-out packages are susceptible to warpage due to:
- CTE mismatch between silicon die, molding compound, and RDL materials
- Thermal stress during molding and RDL processing
- Molding compound shrinkage during cure
- RDL metal stress

**Warpage Impacts**: Excessive warpage can cause:
- Lithography alignment errors during RDL formation
- Assembly challenges during board-level attachment
- Solder joint reliability issues
- Package cracking or delamination

**Mitigation Strategies**:
- Symmetric RDL design to balance stress
- Optimized molding compound formulation and cure profile
- Controlled metal density in RDL layers
- Package design rules that limit die size, RDL extent, and metal coverage
- Temporary carrier stiffening during processing
- Warpage compensation in lithography alignment

### Thermal Management

**Heat Dissipation Paths**: Heat generated in the die must be dissipated through:
- Die backside to board or heat sink (primary path)
- RDL and molding compound to board (secondary path)

**Thermal Enhancements**:
- Thinning the molding compound above die backside
- Thermal vias through the RDL to improve lateral heat spreading
- Heat spreaders or thermal caps attached to the package
- High thermal conductivity molding compounds

**Thermal Simulation**: Finite element thermal analysis is essential for optimizing package thermal performance and ensuring junction temperatures remain within specification across expected operating conditions.

### Electrical Design

**Power Delivery Network**: The RDL must provide adequate power delivery with minimal resistance and inductance:
- Wide power and ground traces in RDL
- Multiple RDL layers for power distribution if necessary
- Adequate decoupling capacitance (on-die, in-package, or on-board)
- Controlled impedance for high-speed signals

**Signal Integrity**: High-speed signals require careful RDL design:
- Controlled impedance transmission lines
- Crosstalk minimization through proper trace spacing and grounding
- Via optimization to minimize discontinuities
- Simulation of critical signal paths including package and board

**EMI/EMC Considerations**: Electromagnetic interference and compatibility must be addressed through:
- Adequate grounding and shielding
- Careful routing of sensitive signals
- Control of current return paths
- Package-level RF modeling for wireless applications

### Design for Manufacturing (DFM)

**RDL Design Rules**: Adherence to manufacturing design rules is critical for yield:
- Minimum line width and spacing
- Via size and keep-out zones
- Metal density requirements for plating uniformity
- Anchor features for adhesion and stress relief

**Die Placement Constraints**: Die placement must consider:
- Minimum spacing between dies for molding and dicing
- Registration mark placement for lithography alignment
- Exclusion zones near reconstituted wafer edges
- Optimal utilization of available area

**Process Variation**: Design must account for manufacturing tolerances:
- Die placement accuracy
- Lithography alignment and critical dimension control
- Molding compound thickness variation
- Metal thickness variation

## 4.5 Applications and Case Studies

Fan-out packaging has been successfully deployed across diverse applications:

### Mobile Application Processors

**Apple A-Series Processors**: Apple's adoption of TSMC InFO technology for iPhone application processors, beginning with the A10 in 2016, validated fan-out packaging for high-volume, high-performance mobile applications. The technology enables:
- Thin package profile critical for slim device design
- Excellent electrical performance supporting high clock speeds
- Efficient heat dissipation for sustained performance
- Integration of processor and DRAM in some implementations (InFO-PoP)

### RF and mmWave Applications

**5G mmWave Modules**: Fan-out AiP (Antenna-in-Package) technology integrates RF ICs and antenna arrays for 5G mmWave applications:
- Antenna elements formed in or on RDL layers
- Close integration of RF IC and antennas minimizes loss
- Compact form factor suitable for space-constrained devices
- Support for beam steering through phased array antennas

### Automotive Applications

**ADAS and Sensor Fusion**: Advanced driver assistance systems utilize fan-out SiP to integrate:
- Application processors
- Image processing accelerators
- Sensor interfaces
- Power management
- Memory

The compact, high-reliability packaging is well-suited to automotive requirements.

### IoT and Wearables

**Ultra-Thin Packages**: The minimal thickness of fan-out packaging makes it ideal for wearable devices and IoT sensors where size and weight are critical constraints.

## 4.6 Reliability and Qualification

Fan-out packages must meet stringent reliability requirements across diverse applications:

### Key Reliability Concerns

**Board-Level Reliability**: The absence of a substrate can make fan-out packages more susceptible to board-level thermomechanical stress:
- Thermal cycling (-40 to 125°C) induces stress due to CTE mismatch with board
- Drop test and mechanical shock create high strain rates
- Vibration induces cyclic loading

Mitigation approaches include optimized solder joint design, underfill application, and package design for mechanical robustness.

**Moisture Sensitivity**: RDL and molding compound materials can absorb moisture, which can vaporize during reflow, potentially causing delamination or "popcorning":
- Moisture sensitivity level (MSL) classification determines storage and handling requirements
- Low-moisture-absorption materials minimize sensitivity
- Baking before assembly can remove absorbed moisture

**Electromigration**: High current densities in fine-pitch RDL can lead to electromigration failures:
- Conservative design rules for current-carrying capacity
- RDL metal thickness adequate for expected currents
- Temperature-aware current derating

### Qualification Testing

Standard reliability qualification includes:

- Thermal cycling test (TCT): Typically -40 to 125°C, 1000+ cycles
- Highly accelerated stress test (HAST): 130°C, 85% RH, 96-168 hours
- Temperature/humidity/bias (THB): 85°C, 85% RH, 1000 hours
- High temperature storage life (HTSL): 150°C, 1000 hours
- Autoclave: 121°C, 100% RH, 2 atm, 96 hours
- Temperature cycle storage (TCS): -65 to 150°C
- Board-level reliability: Thermal cycling, drop test, vibration

## 4.7 Economic Considerations and Market Outlook

Fan-out packaging economics depend on several factors:

### Cost Structure

**Capital Equipment**: Fan-out manufacturing requires specialized equipment:
- Reconstitution tools (die placers, molding, grinding)
- RDL lithography and plating equipment
- Debonding and singulation tools

The capital intensity can be high, particularly for panel-level approaches.

**Manufacturing Costs**: Key cost elements include:
- Known good die cost
- Molding compound and RDL materials
- Processing costs for RDL formation
- Test and singulation
- Yield losses

**Break-Even Analysis**: Fan-out becomes cost-competitive with traditional packaging when:
- I/O count is moderate to high (>200 I/Os)
- Package thickness is critical
- Volume justifies tooling investment
- Substrate cost savings offset RDL costs

### Market Growth

The fan-out packaging market has experienced strong growth:
- Mobile processors remain the largest application segment
- RF and mmWave applications are growing rapidly with 5G deployment
- Automotive and industrial applications are expanding
- Emerging applications in AI edge computing and IoT

Industry analysts project continued strong growth, with fan-out packaging becoming a multi-billion dollar market by the mid-2020s.

## 4.8 Future Trends and Evolution

Several trends are shaping the future of fan-out packaging:

### High-Density Fan-Out (HDFO)

Next-generation fan-out technologies are pushing toward finer RDL pitches (≤2 μm line/space), enabling:
- Higher I/O density approaching 2.5D capabilities
- Integration of more dies and greater complexity
- Potential cost advantages over silicon interposer approaches

### Hybrid Fan-Out/2.5D

Combining local silicon bridges for highest-density connections with fan-out RDL for lower-density routing:
- Intel's EMIB (Embedded Multi-die Interconnect Bridge) exemplifies this approach
- Achieves performance approaching full 2.5D at potentially lower cost
- Enables flexible heterogeneous integration

### Multi-Layer RDL

Expanding to more RDL layers (6-10+) to support greater complexity:
- More sophisticated power delivery networks
- Higher routing density
- Integration of passive components

### Panel-Level Maturation

As panel-level equipment and processes mature:
- Improved economics through larger processing area
- Better utilization of processing capacity
- Potential to disrupt traditional substrate-based packaging

The continued evolution of fan-out packaging will ensure its role as a key enabling technology for next-generation electronic systems across a broad range of applications.
