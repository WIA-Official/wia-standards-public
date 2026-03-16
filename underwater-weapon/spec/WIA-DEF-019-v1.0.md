# WIA-DEF-019: Underwater Weapon Specification v1.0

> **Standard ID:** WIA-DEF-019
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Defense & Security Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Torpedo Systems](#2-torpedo-systems)
3. [Naval Mine Warfare](#3-naval-mine-warfare)
4. [Unmanned Underwater Vehicles](#4-unmanned-underwater-vehicles)
5. [Underwater Acoustics](#5-underwater-acoustics)
6. [Propulsion Systems](#6-propulsion-systems)
7. [Guidance & Control](#7-guidance--control)
8. [Sonar Systems](#8-sonar-systems)
9. [Countermeasures](#9-countermeasures)
10. [Mine Clearance Operations](#10-mine-clearance-operations)
11. [Maritime Security](#11-maritime-security)
12. [Safety & Ethical Guidelines](#12-safety--ethical-guidelines)
13. [References](#13-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for underwater weapons and systems, covering design, operation, and deployment standards with emphasis on maritime security and humanitarian mine clearance operations.

### 1.2 Scope

The standard covers:
- Torpedo design and operation (heavy, light, supercavitating)
- Naval mine warfare (deployment, detection, clearance)
- Unmanned underwater vehicles (AUVs, ROVs, gliders)
- Underwater acoustic propagation and detection
- Propulsion systems (electric, thermal, supercavitation)
- Guidance and control systems
- Sonar systems (active, passive, synthetic aperture)
- Countermeasures and defensive systems
- Humanitarian mine clearance operations

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard prioritizes the use of underwater weapon technology for defensive maritime security, protection of sea lanes, humanitarian mine clearance, and environmental protection while minimizing harm to civilian shipping and marine ecosystems.

### 1.4 Terminology

- **Torpedo**: Self-propelled underwater guided weapon
- **Naval Mine**: Underwater explosive device for area denial
- **UUV (Unmanned Underwater Vehicle)**: Autonomous or remotely operated underwater platform
- **AUV (Autonomous Underwater Vehicle)**: Pre-programmed autonomous system
- **ROV (Remotely Operated Vehicle)**: Tethered remotely controlled system
- **ASW (Anti-Submarine Warfare)**: Operations against hostile submarines
- **MCM (Mine Countermeasures)**: Mine detection, classification, and neutralization
- **SONAR (Sound Navigation and Ranging)**: Acoustic detection and ranging system
- **Cavitation**: Formation of vapor bubbles in liquid due to pressure reduction
- **Supercavitation**: Controlled cavitation for ultra-high speed underwater travel
- **Wake Homing**: Guidance method following target's turbulent wake
- **Wire-Guided**: Control via fiber-optic or copper wire
- **Convergence Zone**: Area where sound rays reconverge after refraction
- **SOFAR Channel**: Deep sound channel for long-range propagation
- **Knot**: Nautical mile per hour (1.852 km/h)

---

## 2. Torpedo Systems

### 2.1 Torpedo Classification

#### 2.1.1 Heavy Torpedoes (533mm, 650mm)

**Purpose**: Anti-ship and anti-submarine warfare from submarines and surface vessels

```
Specifications:
Diameter:     533mm (21 inches) or 650mm (25.5 inches)
Length:       5.5-9.0 meters
Weight:       1,500-3,000 kg
Range:        40-100+ km
Speed:        40-55 knots (conventional), 200+ knots (supercavitating)
Depth:        0-1,200 meters
Warhead:      200-500 kg HE or shaped charge
Propulsion:   Electric, thermal (Otto fuel II), or rocket
Guidance:     Wire-guided, wake homing, active/passive acoustic
Endurance:    20-45 minutes
```

**Key Features**:
- Dual-speed capability (quiet approach, high-speed attack)
- Multiple guidance modes with mode switching
- Advanced target discrimination
- Counter-countermeasure capability
- Stealth propulsion systems

#### 2.1.2 Light Torpedoes (324mm)

**Purpose**: Anti-submarine warfare from helicopters, aircraft, and light vessels

```
Specifications:
Diameter:     324mm (12.75 inches)
Length:       2.5-3.0 meters
Weight:       200-350 kg
Range:        10-20 km
Speed:        35-45 knots
Depth:        0-800 meters
Warhead:      45-75 kg HE
Propulsion:   Electric (battery) or solid fuel
Guidance:     Active/passive acoustic homing
Endurance:    5-15 minutes
```

**Key Features**:
- Air-droppable capability
- Shallow water operation
- Rapid acquisition and attack
- Compact storage and handling

#### 2.1.3 Supercavitating Torpedoes

**Purpose**: High-speed interception of fast-moving targets

```
Specifications:
Diameter:     533mm
Length:       7.0-9.0 meters
Weight:       2,700-3,000 kg
Range:        10-15 km
Speed:        200-230+ knots (370-425 km/h)
Depth:        0-400 meters
Warhead:      200-250 kg HE
Propulsion:   Solid rocket fuel
Guidance:     Inertial with command updates
Endurance:    2-4 minutes
```

**Key Features**:
- Nose cavitator creating gas bubble envelope
- Rocket propulsion with vectored thrust
- Minimal water contact (reduced drag)
- Limited range due to fuel consumption
- No acoustic homing (speed-induced noise)

### 2.2 Torpedo Propulsion

#### 2.2.1 Electric Propulsion

**Advantages**:
- Silent operation (minimal acoustic signature)
- No exhaust wake
- Precise speed control
- Variable speed capability

**Technologies**:
- Silver-zinc batteries (high energy density)
- Aluminum-silver oxide batteries
- Closed-cycle fuel cells
- Lithium-ion/polymer batteries (modern systems)

**Performance**:
```
Power Output:  50-200 kW
Endurance:     20-45 minutes
Top Speed:     45-55 knots
Range:         40-100 km
```

#### 2.2.2 Thermal Propulsion

**Technologies**:
- Otto Fuel II (monopropellant)
- Closed-cycle engines (Rankine, Stirling)
- Turbine-driven propellers

**Performance**:
```
Power Output:  200-400 kW
Endurance:     20-30 minutes
Top Speed:     50-65 knots
Range:         40-50 km
Exhaust:       Water vapor and CO2 (visible wake)
```

#### 2.2.3 Rocket Propulsion (Supercavitating)

**Technologies**:
- Solid fuel rocket motors
- Vectored thrust nozzles
- Nose cavitator (gas generator or mechanical)

**Performance**:
```
Thrust:        High (classified)
Endurance:     2-4 minutes
Top Speed:     200-230+ knots
Range:         10-15 km
Signature:     Extremely loud, visible wake
```

### 2.3 Torpedo Guidance Systems

#### 2.3.1 Wire-Guided

**Operation**:
- Fiber-optic or copper wire spool onboard
- Real-time command from launch platform
- Human operator or fire control system guidance
- Wire length: 20-50 km
- Seamless transition to autonomous homing

**Advantages**:
- Resistance to jamming
- Target re-engagement capability
- Mid-course correction
- Safety (human oversight)

#### 2.3.2 Acoustic Homing

**Passive Acoustic**:
- Listens to target noise (propeller, machinery)
- Silent operation (no active ping)
- Effective against noisy targets
- Vulnerable to quiet submarines

**Active Acoustic**:
- Emits sonar pulses
- Detects echoes from target
- Works against quiet targets
- Higher detection probability
- Reveals torpedo location

**Combined Active/Passive**:
- Passive search mode (long range, stealthy)
- Active attack mode (terminal homing, high accuracy)
- Optimized detection and engagement

#### 2.3.3 Wake Homing

**Operation**:
- Detects turbulent wake behind target
- Follows density gradients and bubble trails
- Effective against surface ships
- Difficult to decoy

**Advantages**:
- Cannot be jammed
- Works against quiet ships
- Long persistence of wake signature

#### 2.3.4 AI-Enhanced Guidance

**Capabilities**:
- Multi-sensor fusion (acoustic, magnetic, wake)
- Target classification and discrimination
- Countermeasure recognition and rejection
- Adaptive search patterns
- Swarm coordination (future)

### 2.4 Torpedo Warheads

#### 2.4.1 High Explosive (HE)

**Composition**: TNT, HBX, PBX explosives
**Weight**: 200-500 kg
**Effect**: Blast damage, hull fracture
**Fuzing**: Contact, proximity, or magnetic

#### 2.4.2 Shaped Charge

**Composition**: Explosive with metal liner
**Weight**: 200-300 kg
**Effect**: Penetrating jet (anti-armor)
**Application**: Submarines, hardened targets

#### 2.4.3 Dual-Purpose

**Design**: Combined blast and penetration
**Application**: Multi-mission capability
**Effectiveness**: Against varied target types

---

## 3. Naval Mine Warfare

### 3.1 Mine Classification

#### 3.1.1 Contact Mines

**Description**: Detonation on physical contact with ship's hull

```
Deployment:   Moored (floating) or bottom-sitting
Depth Range:  0-200 meters
Fuzing:       Hertz horns, pressure plates
Charge:       150-500 kg HE
Target:       Surface vessels and submarines
Lifespan:     1-5 years (battery/chemical degradation)
```

**Advantages**:
- Simple, reliable design
- Low cost
- Effective area denial

**Disadvantages**:
- No target discrimination
- Hazard to friendly and civilian vessels
- Weather/current drift (moored mines)

#### 3.1.2 Influence Mines

**Description**: Detonation triggered by target's physical signature

**Magnetic Influence**:
- Detects ship's magnetic field distortion
- Measures field strength and gradient
- Multi-pulse fuzing (countermeasure resistant)
- Effective against steel-hulled vessels

**Acoustic Influence**:
- Detects propeller and machinery noise
- Frequency analysis and pattern matching
- Can discriminate target types
- Resistant to simple decoys

**Pressure Influence**:
- Detects pressure wave from ship passing overhead
- Most effective in shallow water (<100m)
- Difficult to sweep (no magnetic/acoustic signature)
- Requires precise depth placement

**Combined Influence**:
- Multi-sensor fusion (magnetic + acoustic + pressure)
- AND/OR logic (e.g., magnetic AND acoustic)
- High target discrimination
- Maximum countermeasure resistance

```
Deployment:   Bottom-sitting (seabed)
Depth Range:  10-300 meters
Sensors:      Magnetic, acoustic, pressure (multi-sensor)
Charge:       150-1,000 kg HE
Fuzing:       Ship count, total ship count, time delay
Lifespan:     3-10 years (battery/power management)
```

#### 3.1.3 Rising Mines

**Description**: Bottom-tethered mine with ascending warhead

```
Operation:
1. Seabed canister detects target overhead
2. Warhead released, rises to optimal depth
3. Proximity fuze detonates near hull keel
4. Maximum damage from underwater explosion

Deployment:   Bottom canister with tethered mine
Depth Range:  50-500 meters
Rise Time:    5-15 seconds
Charge:       200-500 kg HE
Target:       Surface ships and submarines
```

**Advantages**:
- Effective in deep water
- Optimal detonation depth (under keel)
- Difficult to detect and sweep

#### 3.1.4 Intelligent Mines

**Description**: AI-enabled target classification and discrimination

```
Capabilities:
- Multi-sensor fusion (acoustic, magnetic, pressure, seismic)
- Neural network target classification
- Friend-or-foe discrimination
- Adaptive behavior (learning patterns)
- Network coordination (minefield awareness)
- Self-deactivation after mission period

Deployment:   Varied (moored, bottom, rising)
Depth Range:  10-500 meters
Sensors:      Multi-modal sensor array
Processing:   Embedded AI processor
Power:        Long-life battery (10-20 years)
```

**Advantages**:
- Minimal collateral damage
- Selective targeting (military only)
- Adaptive tactics
- Difficult to counter

### 3.2 Minefield Layout

#### 3.2.1 Barrier Minefield

**Purpose**: Deny access to specific area or sea lane

```
Pattern:      Linear, multiple rows
Density:      High (overlapping fields of effect)
Mine Types:   Mixed (contact + influence)
Depth:        Varied (surface to bottom)
Length:       Several kilometers
Width:        200-1,000 meters
```

#### 3.2.2 Defensive Minefield

**Purpose**: Protect harbors, naval bases, and critical infrastructure

```
Pattern:      Concentric rings, irregular
Density:      Very high (concentrated)
Mine Types:   Intelligent mines with IFF
Depth:        Multi-layer (surface, mid, bottom)
Coverage:     360-degree protection
```

#### 3.2.3 Offensive Minefield

**Purpose**: Blockade enemy ports and naval facilities

```
Pattern:      Covert placement, irregular
Density:      Medium to high
Mine Types:   Influence and intelligent
Depth:        Optimized for target vessels
Stealth:      Minimal acoustic/magnetic signature
```

### 3.3 Mine Deployment

#### 3.3.1 Surface Vessel

**Methods**:
- Stern ramps (gravity drop)
- Rail launchers
- Helicopter delivery to vessel

**Advantages**:
- Large payload capacity
- Precise placement
- Visual verification

**Disadvantages**:
- Visible deployment (detection risk)
- Slow speed
- Weather dependent

#### 3.3.2 Submarine

**Methods**:
- Torpedo tubes
- Mine chutes (dedicated systems)
- External canisters

**Advantages**:
- Covert deployment
- Access to denied areas
- Multi-mission capability

**Disadvantages**:
- Limited capacity
- Crew hazard (confined space)

#### 3.3.3 Aircraft

**Methods**:
- Parachute retardation
- Low-altitude drop
- Helicopter hover deployment

**Advantages**:
- Rapid deployment
- Large area coverage
- Standoff distance

**Disadvantages**:
- Lower accuracy
- Weather limitations
- Detection risk

#### 3.3.4 UUV (Unmanned Underwater Vehicle)

**Methods**:
- Autonomous pre-programmed routes
- Remote operation
- Swarm deployment

**Advantages**:
- Zero crew risk
- Covert operation
- Adaptive placement
- Long endurance

---

## 4. Unmanned Underwater Vehicles (UUVs)

### 4.1 UUV Classifications

#### 4.1.1 AUV (Autonomous Underwater Vehicle)

**Characteristics**:
- Pre-programmed mission execution
- No tether (free swimming)
- Onboard navigation and decision-making
- Mission duration: Hours to months

**Applications**:
- Oceanographic survey
- Mine reconnaissance
- Submarine tracking
- Cable inspection
- Seafloor mapping

**Systems**:
```
Navigation:   Inertial (INS), Doppler velocity log (DVL), GPS surface
Propulsion:   Electric thrusters, glider wings
Power:        Rechargeable batteries, fuel cells
Sensors:      Sonar, cameras, magnetometers, CTD
Depth:        100-6,000+ meters
Endurance:    6-120+ hours (gliders: weeks to months)
```

#### 4.1.2 ROV (Remotely Operated Vehicle)

**Characteristics**:
- Tethered to surface vessel or submarine
- Real-time operator control
- High-bandwidth data link
- Mission duration: Limited by tether and operator

**Applications**:
- Mine disposal
- Underwater construction
- Salvage operations
- Inspection and maintenance
- Forensic investigation

**Systems**:
```
Control:      Joystick, haptic feedback
Tether:       Fiber-optic and power (100-5,000m)
Propulsion:   Multi-directional thrusters
Manipulators: Robotic arms, cutters, grippers
Sensors:      HD cameras, sonar, lighting
Depth:        100-3,000 meters (work-class)
```

#### 4.1.3 Underwater Gliders

**Characteristics**:
- Buoyancy-driven propulsion
- Extremely long endurance
- Slow speed (0.5-1.5 knots)
- Minimal acoustic signature

**Applications**:
- Long-duration surveillance
- Oceanographic data collection
- Submarine detection networks
- Climate monitoring

**Systems**:
```
Propulsion:   Variable buoyancy (saw-tooth pattern)
Power:        Alkaline or lithium batteries
Navigation:   GPS surface fix, dead reckoning
Sensors:      CTD, acoustic arrays, fluorometers
Depth:        0-1,000 meters (dive cycles)
Endurance:    Weeks to months (>1,000 km range)
```

### 4.2 UUV Navigation

#### 4.2.1 Inertial Navigation System (INS)

**Technology**:
- Gyroscopes and accelerometers
- Dead reckoning from known start point
- Drift: 0.1-2% of distance traveled

**Advantages**:
- Self-contained, no external signals
- Works at any depth

**Disadvantages**:
- Accumulating error over time
- Requires periodic fixes

#### 4.2.2 Doppler Velocity Log (DVL)

**Technology**:
- Acoustic beams measure velocity over ground
- Integration for position update
- Reduces INS drift

**Performance**:
- Accuracy: 0.2-1% of distance traveled
- Requires seabed within 200-600m

#### 4.2.3 Acoustic Positioning

**Systems**:
- Long Baseline (LBL): Seabed transponder network
- Ultra-Short Baseline (USBL): Single transducer array
- Short Baseline (SBL): Hull-mounted array

**Accuracy**:
- LBL: ±1 meter (survey grade)
- USBL: ±1% of range
- SBL: ±5% of range

#### 4.2.4 Geophysical Navigation

**Methods**:
- Terrain-Referenced Navigation (TRN): Match sonar to seabed map
- Magnetic Anomaly Navigation: Follow magnetic field contours
- Gravity Gradiometry: High-precision gravity mapping

**Advantages**:
- Passive (no emissions)
- Accurate in mapped areas
- Continuous position updates

### 4.3 UUV Sensors

#### 4.3.1 Sonar Systems

**Side-Scan Sonar**:
- Imaging seabed features
- Mine detection
- Wreck identification
- Resolution: 1-10 cm

**Synthetic Aperture Sonar (SAS)**:
- High-resolution imaging
- Long-range detection
- Resolution: <1 cm
- Processing-intensive

**Forward-Looking Sonar**:
- Obstacle avoidance
- Collision prevention
- Navigation aid
- Range: 50-500 meters

#### 4.3.2 Magnetometers

**Purpose**: Detect ferrous objects (mines, submarines, wrecks)

**Types**:
- Scalar (total field strength)
- Vector (field direction and strength)

**Sensitivity**: 0.01-0.1 nanoTesla

#### 4.3.3 Optical Cameras

**Types**:
- Still cameras (high resolution)
- Video cameras (real-time)
- Low-light/intensified (dark water)

**Lighting**: LED arrays, strobes

**Challenges**: Turbidity, absorption, backscatter

---

## 5. Underwater Acoustics

### 5.1 Sound Propagation

#### 5.1.1 Sound Speed

**Equation**: c = 1449.2 + 4.6T - 0.055T² + 0.00029T³ + (1.34 - 0.010T)(S - 35) + 0.016D

Where:
- c = sound speed (m/s)
- T = temperature (°C)
- S = salinity (PSU)
- D = depth (meters)

**Typical Values**:
- Warm tropical surface: 1540 m/s
- Cold deep water: 1480 m/s
- Average: 1500 m/s

#### 5.1.2 Acoustic Propagation Modes

**Surface Duct**:
- Trapped between surface and thermocline
- Summer conditions (warm surface layer)
- Range: 10-50 km
- Frequency: >1 kHz

**Deep Sound Channel (SOFAR)**:
- Minimum sound speed at ~1000m depth
- Sound trapped and focused
- Range: 1,000+ km
- Frequency: 10-1000 Hz
- Used for: Long-range communication, whale calls

**Convergence Zones**:
- Sound refracts down then up
- Annular zones at 30-60 km intervals
- Effective in deep water (>1000m)
- Detection "hot spots"

**Bottom Bounce**:
- Sound reflects off seafloor
- Energy loss at each reflection
- Effective in shallow water
- Multi-path interference

**Shadow Zones**:
- Areas where sound does not reach
- Created by refraction
- "Acoustically blind" regions
- Hiding zones for submarines

### 5.2 Acoustic Absorption

**Frequency Dependence**:
```
100 Hz:    0.001 dB/km
1 kHz:     0.05 dB/km
10 kHz:    0.8 dB/km
100 kHz:   20 dB/km
```

**Implication**:
- Low frequency: Long range, low resolution
- High frequency: Short range, high resolution

### 5.3 Ambient Noise

**Sources**:
- Shipping traffic: 50-500 Hz (dominant in many areas)
- Wind and waves: 100-10,000 Hz
- Rain: 1-20 kHz (distinctive signature)
- Biological: Snapping shrimp, whales, fish (varied frequencies)
- Seismic activity: <100 Hz

**Sea State Noise Levels**:
```
Sea State 0 (calm):    40-50 dB
Sea State 3 (moderate): 60-70 dB
Sea State 6 (rough):   80-90 dB
```

### 5.4 Target Strength

**Definition**: Measure of acoustic reflectivity of target

**Typical Values**:
```
Large submarine:  20-30 dB
Small submarine:  10-20 dB
Surface ship:     15-35 dB (depends on aspect)
Mine (1m diameter): -10 to +5 dB
Fish school:      -5 to +15 dB
```

**Aspect Dependency**:
- Bow/stern: Lower target strength (streamlined)
- Beam aspect: Higher target strength (large cross-section)

---

## 6. Propulsion Systems

### 6.1 Conventional Propulsion

#### 6.1.1 Propeller-Driven

**Types**:
- Fixed-pitch propellers
- Variable-pitch propellers
- Ducted propellers (increased efficiency)
- Pumpjet (reduced cavitation, noise)

**Performance**:
```
Efficiency: 60-85%
Speed Range: 5-65 knots
Noise: Moderate to high (cavitation)
```

**Noise Sources**:
- Blade rate frequency (number of blades × RPM)
- Cavitation (bubble collapse)
- Tip vortex cavitation
- Propeller singing (resonance)

#### 6.1.2 Electric Thrusters

**Applications**: UUVs, torpedoes, AUVs

**Advantages**:
- Silent operation (no cavitation at low speed)
- Precise control
- Reversible thrust

**Technologies**:
- Brushless DC motors
- Direct-drive or geared
- Multi-directional (ROV maneuvering)

### 6.2 Advanced Propulsion

#### 6.2.1 Magnetohydrodynamic (MHD)

**Concept**: Electrically accelerate seawater using magnetic fields

**Advantages**:
- No moving parts (silent)
- No cavitation
- Theoretically stealthy

**Challenges**:
- Low efficiency (~5%)
- High power requirement
- Electrode erosion
- Large size and weight

**Status**: Experimental, limited practical application

#### 6.2.2 Supercavitation

**Concept**: Create vapor bubble envelope around vehicle

**Method**:
- Nose cavitator (disc or ventilated cavity)
- Gas injection (air, exhaust gas)
- High speed (>100 knots) reduces drag

**Advantages**:
- Ultra-high speed (200+ knots)
- Reduced drag (90% reduction)

**Challenges**:
- Control difficulty (minimal water contact)
- High power requirement
- Limited range (fuel consumption)
- Extremely loud (acoustic signature)
- Cannot use acoustic guidance (self-noise)

**Applications**:
- Interceptor torpedoes (e.g., Russian Shkval)
- Future projectiles

---

## 7. Guidance & Control

### 7.1 Guidance Modes

#### 7.1.1 Inertial Guidance

**Technology**:
- Gyroscopes (ring laser, fiber optic)
- Accelerometers (measure velocity changes)
- Pre-programmed path following

**Performance**:
- Drift: 1-5% of distance traveled
- Suitable for straight-line runs
- No external emissions

**Applications**:
- Cruise phase navigation
- Supercavitating torpedoes (only option)

#### 7.1.2 Wire-Guided

**Technology**:
- Fiber-optic or copper wire spool
- Real-time command link
- 20-50 km wire length

**Advantages**:
- Mid-course correction
- Target update
- Abort capability
- Jam-resistant

**Applications**:
- Heavy torpedoes
- Precision targeting

#### 7.1.3 Acoustic Homing

See Section 2.3.2 for details

#### 7.1.4 Terrain-Following

**Technology**:
- Bottom-looking sonar
- Maintain constant altitude over seabed
- Adaptive depth control

**Applications**:
- UUV seafloor mapping
- Mine detection
- Stealth transit (stay in acoustic shadow)

### 7.2 Control Systems

#### 7.2.1 Depth Control

**Methods**:
- Hydroplanes (pitch and dive angle)
- Ballast systems (buoyancy change)
- Thruster control (dynamic positioning)

**Sensors**:
- Pressure transducers (depth measurement)
- Inclinometers (pitch and roll)

#### 7.2.2 Heading Control

**Methods**:
- Rudders (directional control)
- Differential thrust (thruster-based)
- Vectored thrust (nozzle steering)

**Sensors**:
- Gyrocompasses
- Magnetic compasses
- Doppler velocity log (velocity vector)

#### 7.2.3 Speed Control

**Methods**:
- Propeller RPM modulation
- Thruster power adjustment
- Dual-speed settings (cruise/attack)

**Feedback**:
- Doppler log (velocity measurement)
- Propeller shaft RPM sensors

---

## 8. Sonar Systems

### 8.1 Active Sonar

#### 8.1.1 Operational Principles

**Operation**:
1. Transmit acoustic pulse (ping)
2. Listen for echoes from targets
3. Measure time delay (range)
4. Measure bearing (directional hydrophone array)
5. Analyze Doppler shift (target velocity)

**Performance Equation**:
```
Signal Excess (SE) = Source Level (SL) - Transmission Loss (TL) + Target Strength (TS) - Noise Level (NL) - Detection Threshold (DT)

SE > 0: Detection probable
SE < 0: Detection unlikely
```

**Typical Parameters**:
```
Source Level:      210-235 dB re 1 μPa @ 1m
Frequency:         1-10 kHz (hull-mounted), 10-100 kHz (high-res)
Pulse Length:      10-500 ms
Pulse Repetition:  Every 5-60 seconds
```

#### 8.1.2 Advantages

- Detects quiet targets
- Accurate range and bearing
- Target classification (echo characteristics)
- Works in high ambient noise

#### 8.1.3 Disadvantages

- Reveals sonar platform location
- Limited by reverberation (false echoes)
- Energy intensive
- Effective range: 5-50 km (varies greatly)

### 8.2 Passive Sonar

#### 8.2.1 Operational Principles

**Operation**:
- Listen for target-radiated noise
- Analyze frequency spectrum
- Directional hydrophone arrays (bearing)
- No active transmission (stealthy)

**Target Noise Sources**:
- Propeller cavitation (blade rate harmonics)
- Machinery vibrations (pumps, generators, gears)
- Hydrodynamic flow noise
- Hull popping (pressure changes)

**Performance**:
```
Detection Range: 10-100+ km (depends on target noise, sea conditions)
Frequency Range: 10 Hz - 10 kHz (broadband), narrowband analysis
Array Gain:      15-30 dB (directivity and noise reduction)
```

#### 8.2.2 Advantages

- Long detection range (quiet listening)
- Covert operation (no emissions)
- Continuous operation (no pulse cycle)
- Target classification (noise signature)

#### 8.2.3 Disadvantages

- Cannot detect quiet targets
- No range information (bearing only)
- Requires target to radiate noise
- Ineffective against stationary targets

### 8.3 Synthetic Aperture Sonar (SAS)

#### 8.3.1 Operational Principles

**Concept**:
- Platform motion creates virtual large array
- Signal processing synthesizes aperture
- High-resolution imaging

**Performance**:
```
Resolution:    <10 cm (constant across range)
Swath Width:   100-500 meters
Frequency:     100-300 kHz
Platform:      AUV, UUV, towed array
```

#### 8.3.2 Applications

- Mine detection and classification
- Seafloor mapping
- Wreck survey
- Archaeological imaging
- Cable/pipeline inspection

### 8.4 Towed Arrays

#### 8.4.1 Thin-Line Arrays

**Characteristics**:
- Long (100-1,000+ meters)
- Passive listening
- Low-frequency focus (10-500 Hz)
- Towed by submarines or surface vessels

**Advantages**:
- Large aperture (high directivity)
- Separation from tow platform noise
- Long detection range

**Challenges**:
- Flow noise (must tow slowly, <10 knots)
- Maneuvering limitations
- Vulnerable to damage

---

## 9. Countermeasures

### 9.1 Torpedo Countermeasures

#### 9.1.1 Acoustic Decoys

**Types**:
- Noisemakers: Simulate target noise signature
- Jammers: Emit masking noise
- Towed decoys: Seduction away from platform

**Deployment**:
- Ejected canisters (submarines, ships)
- Self-propelled mobile decoys (ROV-like)

**Effectiveness**:
- Against passive homing: High (if signature matches)
- Against active homing: Moderate (depends on sophistication)
- Against wake homing: Low to none

#### 9.1.2 Evasive Maneuvers

**Tactics**:
- High-speed sprint: Outrun torpedo (if faster)
- Sharp turns: Break wire guidance, complicate tracking
- Depth changes: Exploit thermoclines, bottom terrain
- Knuckle (tight turn): Create confusing acoustic return

**Effectiveness**:
- Depends on relative speed
- Modern torpedoes have counter-evasion tactics

#### 9.1.3 Hard-Kill

**Methods**:
- Anti-torpedo torpedoes (intercept and destroy)
- Rapid-fire decoys (kinetic disruption)

**Challenges**:
- Very short reaction time (<1 minute)
- High accuracy required
- Still experimental

### 9.2 Mine Countermeasures (MCM)

#### 9.2.1 Mine Hunting

**Approach**: Detect, classify, localize mines for disposal

**Platforms**:
- Dedicated mine hunters (surface ships with low magnetic signature)
- UUVs/ROVs (unmanned hunting)
- Airborne systems (helicopters with magnetic/acoustic sensors)

**Sensors**:
- High-resolution sonar (imaging)
- Magnetometers (ferrous detection)
- Electromagnetic sensors

**Process**:
1. Survey area with sonar/magnetic sensors
2. Identify mine-like contacts
3. Classify with imaging sonar or divers
4. Dispose of confirmed mines

#### 9.2.2 Mine Sweeping

**Approach**: Trigger or cut mines without hunting

**Mechanical Sweeping**:
- Wire sweeps (cut mooring cables of tethered mines)
- Towed by surface vessels or helicopters

**Influence Sweeping**:
- Magnetic: Generate strong magnetic field
- Acoustic: Emit ship-like noise
- Pressure: Create pressure waves
- Simulates ship passage, triggers mines

**Effectiveness**:
- Mechanical: Effective against moored contact mines
- Influence: Effective if sweep signature exceeds mine threshold
- Limited against intelligent mines (target discrimination)

#### 9.2.3 Mine Avoidance

**Approach**: Detect and avoid mined areas

**Methods**:
- Forward-looking sonar (detect ahead)
- Route planning (avoid known/suspected minefields)
- Intelligence (minefield charting)

**Applications**:
- Safe transit corridors
- Port approach routes

---

## 10. Mine Clearance Operations

### 10.1 Humanitarian Mine Clearance

#### 10.1.1 Post-Conflict Demining

**Objective**: Clear legacy naval minefields to restore safe navigation

**Challenges**:
- Unknown minefield locations (poor records)
- Aged mines (unstable, sensitive)
- Corroded/damaged (unpredictable)
- Mixed mine types

**Approach**:
1. Historical research (wartime records, charts)
2. Community interviews (local fishermen, witnesses)
3. Systematic survey (side-scan sonar, magnetometer)
4. UUV reconnaissance (autonomous search patterns)
5. Identification and classification
6. Disposal (in-situ neutralization or removal)

**Safety**:
- Standoff distances (UUV/ROV, minimize diver exposure)
- Controlled detonation (underwater demolition)
- Salvage (when safe, environmentally preferred)

#### 10.1.2 Fishing Ground Clearance

**Objective**: Restore access to fishing areas contaminated with mines

**Priority Areas**:
- Coastal waters
- Continental shelf
- Traditional fishing grounds

**Benefits**:
- Economic recovery
- Food security
- Livelihood restoration

**Stakeholders**:
- Local fishing communities
- Environmental organizations
- International demining NGOs
- Government maritime authorities

### 10.2 Critical Infrastructure Protection

#### 10.2.1 Submarine Cable Security

**Threats**:
- Mines placed near cables
- Terrorist sabotage
- State-sponsored disruption

**Countermeasures**:
- UUV patrol routes
- Persistent surveillance (gliders, seabed sensors)
- Rapid response teams (ROV cable repair)

**Importance**:
- >95% of international data traffic via submarine cables
- Critical for global economy and communications

#### 10.2.2 Port and Harbor Security

**Threats**:
- Mines in approach channels
- Terrorist attacks (limpet mines)
- Blockade by hostile forces

**Countermeasures**:
- Regular sonar sweeps
- UUV perimeter patrols
- Access control (diver detection sonar)
- ROV inspection of ship hulls

---

## 11. Maritime Security

### 11.1 Sea Lane Protection

#### 11.1.1 Chokepoint Security

**Critical Straits**:
- Strait of Hormuz (oil transport)
- Strait of Malacca (shipping route)
- Suez Canal (Europe-Asia trade)
- Bosphorus (Black Sea access)
- Panama Canal (inter-ocean transit)

**Threats**:
- Mine blockade
- Submarine interdiction
- Terrorist attacks

**Countermeasures**:
- Continuous MCM operations
- ASW patrols (UUV, submarines)
- Rapid mine clearance capability

#### 11.1.2 Freedom of Navigation

**International Law**: UNCLOS (UN Convention on the Law of the Sea)

**Rights**:
- Innocent passage through territorial waters
- Transit passage through straits
- Freedom of the high seas

**Enforcement**:
- Naval presence
- Mine-free corridors
- International cooperation

### 11.2 Anti-Piracy Operations

#### 11.2.1 Maritime Patrol

**Platforms**:
- UUV surveillance (persistent monitoring)
- Surface vessels (visible deterrence)
- Aircraft (rapid response)

**Sensors**:
- Sonar (detect underwater craft)
- AIS (Automatic Identification System, track ships)
- Radar (surface contacts)

#### 11.2.2 Port Security

**Measures**:
- Underwater surveillance sensors
- ROV hull inspections
- Diver detection sonar
- Access control barriers

---

## 12. Safety & Ethical Guidelines

### 12.1 International Law Compliance

#### 12.1.1 Mine Warfare

**Hague Convention VIII (1907)**:
- Automatic contact mines must become harmless after 1 hour adrift
- Anchored mines must become harmless when adrift
- Notification of minefield locations (when military necessity permits)

**Ottawa Convention (1997)**:
- Bans anti-personnel landmines (does not cover naval mines)

**Best Practices**:
- Declare minefield locations after hostilities
- Provide charts and records for clearance
- Prioritize civilian safety

#### 12.1.2 UNCLOS (Law of the Sea)

**Relevant Provisions**:
- Freedom of the high seas (Article 87)
- Right of innocent passage (Article 17-19)
- Archipelagic sea lanes passage (Article 53)

**Compliance**:
- No indiscriminate mining of international waters
- Respect territorial boundaries
- Minimize disruption to civilian shipping

### 12.2 Environmental Protection

#### 12.2.1 Marine Ecosystem Impact

**Concerns**:
- Underwater explosions (harm marine life)
- Sonar effects (marine mammal disruption)
- Chemical contamination (old mines leaking explosives)
- Physical debris (sunken munitions)

**Mitigation**:
- Marine mammal monitoring (sonar operations)
- Controlled detonations (minimize shock waves)
- Environmental surveys (pre- and post-operations)
- Salvage over detonation (when safe)

#### 12.2.2 Noise Pollution

**Impact**:
- Active sonar disrupts whale communication and navigation
- Torpedo testing disturbs marine habitats

**Measures**:
- Seasonal restrictions (breeding seasons)
- Geographic exclusion zones (critical habitats)
- Reduced power levels (when tactically feasible)

### 12.3 Ethical Considerations

#### 12.3.1 Civilian Protection

**Principles**:
- Distinction (military vs. civilian targets)
- Proportionality (minimize collateral damage)
- Precaution (take measures to protect civilians)

**Application**:
- Use intelligent mines with target discrimination
- Declare and chart minefields
- Provide safe corridors for civilian shipping
- Prioritize humanitarian demining

#### 12.3.2 Autonomous Weapons

**Concerns**:
- Intelligent mines with lethal decision-making
- UUVs with attack capability
- Accountability for autonomous actions

**Guidelines**:
- Human-in-the-loop for lethal decisions
- Clear rules of engagement
- Failsafe deactivation mechanisms
- International oversight and transparency

### 12.4 Operational Safety

#### 12.4.1 Personnel Safety

**Risks**:
- Diver exposure to mines (demining operations)
- ROV operators (electrical, mechanical hazards)
- Explosive ordnance disposal (EOD) teams

**Measures**:
- Maximize use of unmanned systems
- Comprehensive training and certification
- Safety protocols and equipment
- Medical support and emergency response

#### 12.4.2 Test Safety

**Requirements**:
- Designated test ranges (isolated from civilian traffic)
- Environmental impact assessments
- Range clearance (before and after)
- Unexploded ordnance (UXO) accounting

---

## 13. References

### 13.1 International Standards

- NATO STANAG 1364: NATO Mine Countermeasures Classification System
- IHO S-44: Standards for Hydrographic Surveys (mine clearance surveys)
- IMAS: International Mine Action Standards (adapted for naval mines)

### 13.2 Technical References

- Urick, R.J. (1983). *Principles of Underwater Sound* (3rd ed.). McGraw-Hill.
- Burdic, W.S. (1984). *Underwater Acoustic System Analysis*. Prentice Hall.
- Medwin, H., & Clay, C.S. (1998). *Fundamentals of Acoustical Oceanography*. Academic Press.
- Ainslie, M.A. (2010). *Principles of Sonar Performance Modeling*. Springer.

### 13.3 Military Publications

- U.S. Navy Publication NWP 3-15: Mine Warfare
- NATO ATP-6(E): Mine Warfare Doctrine
- JMEM (Joint Munitions Effectiveness Manual): Underwater munitions data

### 13.4 Legal Frameworks

- United Nations Convention on the Law of the Sea (UNCLOS, 1982)
- Hague Convention VIII (1907): Naval Mine Warfare
- Protocol on Explosive Remnants of War (Protocol V to CCW, 2003)

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-DEF-019: Underwater Weapon Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
