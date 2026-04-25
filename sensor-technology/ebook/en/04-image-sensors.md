# Chapter 4: CMOS Image Sensors (CIS)

## Introduction to Image Sensor Technology

CMOS image sensors have revolutionized digital imaging, enabling everything from smartphone cameras to autonomous vehicles. This chapter explores the architecture, performance metrics, and cutting-edge technologies from Sony, Samsung, and other leading manufacturers.

### Image Sensor Fundamentals

#### Photoelectric Effect

**Basic Principle:**
Light photons striking silicon create electron-hole pairs. The number of electrons is proportional to light intensity.

**Quantum Efficiency (QE):**
```
QE = (Number of electrons generated) / (Number of incident photons)

Typical values:
- Visible light (550 nm): QE = 60-90%
- NIR (850 nm): QE = 20-40%
- BSI sensors: QE up to 95%
```

**Spectral Response:**
- Silicon bandgap: 1.12 eV
- Maximum sensitivity: 800-900 nm
- Absorption depth varies with wavelength
  - Blue (450 nm): Absorbed in <1 μm
  - Red (650 nm): Absorbed in 2-3 μm
  - NIR (900 nm): Penetrates 10+ μm

#### Pixel Architecture

**3T (Three-Transistor) Pixel:**
```
Components:
- Photodiode (PD): Light detection
- Reset transistor: Reset PD to reference voltage
- Source follower: Buffer amplifier
- Row select: Enable pixel readout

Advantages: Simple, small pixel size
Disadvantages: High noise, rolling shutter only
```

**4T (Four-Transistor) Pixel:**
```
Components:
- Photodiode
- Transfer gate: Isolate PD from floating diffusion
- Reset, source follower, row select
- Floating diffusion (FD): Charge storage node

Advantages:
- Correlated Double Sampling (CDS) reduces noise
- Lower dark current
- Higher fill factor
- Global shutter possible

Pixel Size: 0.7-2.0 μm typical
Full Well Capacity: 2000-10000 electrons
Read Noise: 1-3 electrons RMS
```

**Advanced Pixel Designs:**

**Dual Conversion Gain (DCG):**
- Two readout modes: High conversion gain, low conversion gain
- High CG: Low light (low noise)
- Low CG: Bright scenes (high dynamic range)
- Combined: >100 dB dynamic range

**Example: Sony IMX586**
- 48MP Quad Bayer
- DCG pixel architecture
- HDR: 14.8 stops (89 dB)

**Stacked Pixel:**
- Photodiode layer on top
- Logic circuits beneath
- Connected by Through-Silicon Vias (TSV)
- Benefits: Higher fill factor, more on-chip processing

**Sony Exmor RS:**
- Stacked CMOS sensor
- Pixel layer: Photosensitive
- Logic layer: ADC, ISP, DRAM buffer
- Enables slow-motion 960fps video

#### Back-Side Illumination (BSI)

**Traditional Front-Side Illumination (FSI):**
- Light enters from front
- Passes through metal wiring, transistors
- Only 30-40% reaches photodiode

**Back-Side Illumination:**
- Wafer thinned from back
- Light enters directly to photodiode
- QE increases to 70-90%

**BSI Process:**
```
1. Fabricate pixel and transistors on front
2. Bond to support wafer
3. Grind silicon substrate from back (to 5-10 μm)
4. Form color filters and microlenses on backside
5. Release from support wafer
```

**Market Adoption:**
- Smartphones: 100% BSI since 2015
- Automotive: Transitioning to BSI for low-light performance
- Industrial: Mixed (cost vs. performance trade-off)

### Color Imaging

#### Bayer Filter

**Pattern:**
```
G R G R
B G B G
G R G R
B G B G

50% Green, 25% Red, 25% Blue
(Human eye most sensitive to green)
```

**Demosaicing:**
- Interpolate missing color values
- Algorithms: Bilinear, bicubic, adaptive edge-directed
- Artifacts: Moiré, zipper effects, false colors

**Color Accuracy:**
- Color correction matrix (CCM)
- White balance adjustment
- Gamma correction

#### Quad Bayer / Tetracell

**Structure:**
```
G1 G2 | G1 G2     2x2 blocks of same color
G3 G4 | G3 G4
------+------
B1 B2 | B1 B2
B3 B4 | B3 B4
```

**Pixel Binning Mode:**
- Combine 4 pixels → 1 output pixel
- 48MP sensor outputs 12MP
- 4x light sensitivity
- Ideal for low-light photography

**Full Resolution Mode:**
- Read all 48MP
- Remosaic algorithm
- Improved resolution vs. standard Bayer

**Examples:**
- Samsung ISOCELL GN2: 50MP (1.4 μm pixels), bins to 12.5MP
- Sony IMX766: 50MP (1.0 μm), bins to 12.5MP
- OmniVision OV64B: 64MP (0.7 μm), bins to 16MP

#### Other Color Filter Approaches

**RGBW (Red-Green-Blue-White):**
- White (clear) pixels for luminance
- Improved low-light sensitivity
- More complex demosaicing

**RGB-IR:**
- Infrared-pass pixels
- Simultaneous visible + IR capture
- Applications: Security cameras, gesture recognition

**Foveon X3:**
- Vertically stacked RGB photodiodes
- No color interpolation needed
- Challenge: Cross-talk between layers

### Advanced Image Sensor Technologies

#### Global Shutter

**Rolling Shutter Problem:**
- Rows exposed sequentially
- Moving objects appear distorted ("jello effect")
- Flash banding
- Unacceptable for fast motion (sports, automotive)

**Global Shutter Solution:**
- All pixels exposed simultaneously
- Each pixel stores charge in-pixel
- Readout after exposure ends

**In-Pixel Storage:**
- Floating diffusion capacitor
- Separate DRAM cell (Sony)
- Requires 2x pixel area or stacking

**Applications:**
- Industrial machine vision
- Automotive (100% global shutter by 2028)
- Drones and robotics
- AR/VR tracking

**Example: Sony IMX420**
- Global shutter
- 1.55 μm pixels
- 12.4MP resolution
- 119 fps max frame rate

#### High Dynamic Range (HDR)

**Challenge:**
Single exposure cannot capture bright sunlight (100,000 lux) and deep shadows (0.1 lux) simultaneously.

**HDR Techniques:**

**1. Multiple Exposure:**
- Capture 2-4 frames at different exposures
- Short exposure: Highlights
- Long exposure: Shadows
- Merge in ISP

**Frame Rate:**
- 3 exposures @ 30fps → effective 10fps
- Solution: Stagger exposures across rows (DOL-HDR)

**2. Dual Conversion Gain (DCG):**
- Single exposure, dual readout
- Low CG for highlights
- High CG for shadows
- Combine for >100 dB DR

**3. Split Pixel:**
- Large photodiode + small photodiode in same pixel
- Large PD: High full-well capacity (bright)
- Small PD: Low read noise (dark)

**4. Logarithmic Pixel:**
- Non-linear photodiode response
- Compresses wide dynamic range
- Challenge: Poor low-light SNR

**Automotive HDR Requirements:**
- Dynamic range: >120 dB (1,000,000:1)
- LED flicker mitigation (PWM headlights/taillights)
- Simultaneous tunnel exit and bright sky

**Example: Samsung ISOCELL Auto 4AC**
- 120 dB HDR
- LED flicker mitigation (LFM)
- Automotive ASIL-B safety rated

#### Phase Detection Autofocus (PDAF)

**Principle:**
- Dual photodiodes (left/right) in pixel
- Phase difference indicates focus direction
- Faster than contrast-detect autofocus

**Implementation:**

**Sparse PDAF:**
- 5-10% of pixels dedicated to PDAF
- Reduced resolution at PDAF pixels
- Example: Sony IMX586

**Dual Pixel PDAF:**
- 100% of pixels support PDAF
- Each pixel split into two photodiodes
- Full-resolution imaging + fast AF
- Example: Samsung ISOCELL GN2, Canon sensors

**PDAF Performance:**
- Focus speed: <0.1 seconds
- Focus accuracy: ±0.1% depth of field
- Works in low light (down to -3 EV)

#### Multi-Camera Systems

**Smartphone Camera Arrays (2025 Flagship):**

**Main Camera:**
- 50MP, 1/1.3" sensor size
- 1.2 μm pixels with quad-binning
- f/1.6-1.8 aperture
- OIS (optical image stabilization)

**Ultrawide:**
- 12-50MP, 1/2.5" sensor
- 120-130° field of view
- f/2.2 aperture
- Autofocus (some models)

**Telephoto:**
- 10-48MP periscope design
- 3-5x optical zoom
- f/2.5-3.5 aperture
- OIS

**Additional Sensors:**
- Macro: 2-5MP, close-focus
- ToF: Depth sensing
- Front: 10-32MP selfie

**Multi-Camera Challenges:**
- Color consistency across cameras
- Seamless zoom transitions
- Computational photography (HDR, night mode)
- Calibration (geometric, radiometric)

### Leading Manufacturers

#### Sony Image Sensors

**Market Position:**
- 52% market share (2024)
- Technology leader in BSI, stacked sensors
- Supplies: Apple, major Android OEMs, automotive

**Flagship Products:**

**IMX989 (1-inch sensor):**
- 50.3MP resolution
- 1.6 μm pixels
- Octa-PD PDAF (8-direction phase detection)
- 8K video @ 24fps
- Used in: Xiaomi 12S Ultra, Leica collaboration

**IMX766:**
- 50MP, 1/1.56" sensor
- DOL-HDR (staggered HDR)
- Dual pixel PDAF
- Popular mid-range sensor (OnePlus, OPPO, realme)

**IMX585 (8K Video):**
- 8.3MP (3840 x 2160 × 4)
- 2.9 μm large pixels
- Global shutter
- Cinema-grade color science

**Automotive: IMX490/IMX728:**
- 5.4MP, 1/2.3" sensor
- 120 dB HDR
- LED flicker mitigation
- ISO 26262 ASIL-B functional safety
- Temperature: -40°C to +105°C

**Technology Innovations:**

**Exmor RS (Stacked):**
- Pixel layer + logic layer via TSV
- Integrated DRAM buffer (1 Gb)
- Enables 960fps slow motion

**Pregius S (Industrial Global Shutter):**
- Polarized light imaging
- >80 dB SNR
- Machine vision applications

#### Samsung ISOCELL

**Market Position:**
- 28% market share
- Vertical integration: Galaxy smartphones
- Aggressive push to 200MP sensors

**Flagship Products:**

**ISOCELL HP2 (200MP):**
- 200MP resolution
- 0.6 μm pixels (ultra-small)
- 16-in-1 binning → 12.5MP (2.4 μm effective)
- 8K video @ 30fps
- Used in: Galaxy S23 Ultra

**ISOCELL GN2:**
- 50MP, 1/1.12" sensor (larger than most)
- 1.4 μm pixels
- Dual Pixel Pro PDAF
- Smart-ISO Pro (HDR)
- Staggered HDR video recording

**ISOCELL Vizion (ToF):**
- Indirect Time-of-Flight
- VGA resolution (640 × 480)
- Depth range: 0.2-5 meters
- Used for: AR, portrait mode, autofocus assist

**Technology Innovations:**

**Deep Trench Isolation (DTI):**
- Physical barriers between pixels
- Reduces crosstalk by 30%
- Improves color accuracy

**Tetracell (Quad Bayer):**
- 2×2 color filter binning
- Chameleon cells adapt to lighting

**Super QPD (Quad Phase Detection):**
- Each pixel split into 4 photodiodes
- Vertical + horizontal phase detection
- Improved AF accuracy

#### OmniVision Technologies

**Market Position:**
- 11% market share
- Focus: Automotive, medical, security
- CameraCubeChip integration

**Automotive Products:**

**OX03C (3MP Automotive):**
- 3MP, 1/2.7" sensor
- 120 dB HDR
- LED flicker mitigation
- ASIL-B safety rated
- Temperature: -40°C to +125°C

**OX08B (8MP Automotive):**
- 8MP, 1/2.5" sensor
- SuperClearPlus-2 HDR
- Cyber security features (secure boot)

**Medical Endoscopy:**

**OH0TA (1MP Global Shutter):**
- 1280 × 1024 resolution
- 3 μm pixels
- Global shutter
- Designed for ultra-small endoscopes

**Technology Innovations:**

**PureCel Plus-S:**
- Split conversion gain HDR
- >90 dB DR in single exposure

**Nyxel Technology:**
- NIR (near-infrared) optimized
- QE >60% @ 940 nm
- Security and ADAS applications

### Image Sensor Performance Metrics

#### Resolution and Pixel Size

**Resolution:**
- Measured in megapixels (MP)
- Formula: Width × Height pixels
- Example: 4000 × 3000 = 12MP

**Pixel Size vs. Performance:**

```
Large Pixels (>2 μm):
- Pros: High full-well capacity, low noise, better DR
- Cons: Larger sensors, higher cost
- Use: Professional cameras, medical imaging

Medium Pixels (1.0-2.0 μm):
- Pros: Balanced performance
- Cons: Moderate cost
- Use: Flagship smartphones, automotive

Small Pixels (<1.0 μm):
- Pros: High resolution, compact size
- Cons: Lower SNR, reduced DR
- Use: Multi-camera arrays, budget phones
```

**Pixel Binning:**
Combines adjacent pixels to simulate larger pixel
- 4-to-1 binning: 0.8 μm → 1.6 μm equivalent
- Improves: Sensitivity (4x), SNR (2x)
- Reduces: Resolution (1/4)

#### Sensitivity and Noise

**ISO Sensitivity:**
```
Sensor gain amplifies signal

ISO 100 (base ISO): No amplification
ISO 800: 8x amplification
ISO 6400: 64x amplification

Note: Amplification also increases noise
```

**Read Noise:**
- Noise introduced by readout circuitry
- Measured in electrons RMS
- Excellent: <1.5 e⁻ RMS
- Good: 1.5-3 e⁻ RMS
- Poor: >5 e⁻ RMS

**Dark Current Noise:**
- Thermally generated electrons
- Doubles every ~8°C temperature increase
- Mitigated by: Cooling, dark frame subtraction

**Photon Shot Noise:**
```
Noise = √N electrons (Poisson statistics)

Example:
10,000 photons → 100 electrons noise
SNR = 10,000 / 100 = 100:1 (40 dB)
```

**Signal-to-Noise Ratio (SNR):**
```
SNR (dB) = 20 × log₁₀(Signal / Noise)

Excellent: >50 dB
Good: 40-50 dB
Acceptable: 30-40 dB
```

#### Dynamic Range

**Definition:**
Ratio between maximum and minimum detectable signal.

```
DR (dB) = 20 × log₁₀(FWC / Read Noise)

Where:
FWC = Full-Well Capacity (electrons)
Read Noise = RMS read noise (electrons)

Example:
FWC = 10,000 e⁻
Read Noise = 2 e⁻
DR = 20 × log₁₀(5000) = 73.9 dB
```

**Dynamic Range Benchmarks:**
- Human eye: ~20 stops (120 dB)
- Film: 12-14 stops (72-84 dB)
- Consumer camera: 10-13 stops (60-78 dB)
- HDR sensor: 14-20 stops (84-120 dB)

**Automotive Requirements:**
- Minimum: 100 dB (tunnel to bright sky)
- Target: 120 dB for SAE Level 4+ autonomy

#### Frame Rate and Bandwidth

**Data Rate Calculation:**
```
Data rate = Width × Height × Bit depth × FPS

Example: 4K @ 60fps, 10-bit
= 3840 × 2160 × 10 × 60
= 4.98 Gb/s
```

**Interface Bandwidth:**
- MIPI CSI-2: 1-6 Gbps per lane (×4 lanes typical)
- SLVS-EC: Up to 8 Gbps per lane (automotive)
- GMSL: Gigabit Multimedia Serial Link (automotive)

**Frame Rate Trade-offs:**
- Higher FPS → Lower exposure time → Less light → More noise
- Solution: Larger pixels, BSI, HDR binning

**Slow Motion:**
- 240fps @ 1080p: Typical smartphone
- 960fps @ 720p: Sony Exmor RS (buffered)
- 1000fps @ 1080p: Industrial cameras

### Computational Photography

#### Multi-Frame Processing

**Night Mode:**
1. Capture 6-12 short exposures
2. Align frames (compensate hand shake)
3. Merge with noise reduction
4. Tone map for display

**Example: Google Night Sight**
- 15 frames in 3-6 seconds
- Motion metering (adjust count based on shake)
- Learning-based HDR+

**HDR Merge:**
- Bracket exposures (-2 EV, 0 EV, +2 EV)
- Align and ghost removal
- Tone mapping (compress DR for display)

**Super Resolution:**
- Capture burst of images
- Sub-pixel shifts (OIS or hand shake)
- Merge to increase resolution
- Example: Google Super Res Zoom (2x detail improvement)

#### AI-Enhanced Imaging

**Semantic Segmentation:**
- Identify: Sky, people, food, pets
- Apply scene-specific processing
- Sky: Enhance blue, increase contrast
- Skin: Smooth texture, optimize color

**Portrait Mode:**
- Dual camera or ToF depth sensing
- Depth map generation
- Bokeh simulation (blur background)
- Edge refinement with AI

**Low-Light Enhancement:**
- Denoise while preserving detail
- Learning-based vs. traditional NR
- Train on pairs (noisy, clean)

**Image Stacking:**
- Align and average multiple frames
- SNR improvement: √N frames
- 9 frames → 3x noise reduction (9.5 dB)

### Automotive Camera Requirements

#### Environmental Robustness

**Temperature Range:**
- Operating: -40°C to +105°C (junction)
- Storage: -40°C to +125°C
- Automotive-grade components throughout

**Humidity and Condensation:**
- 95% RH non-condensing
- Hydrophobic lens coatings
- Heated lens option for defrost

**Vibration and Shock:**
- Random vibration: 10 G RMS
- Shock: 50-100 G
- Ruggedized packaging, dampers

**EMI/EMC Compliance:**
- Electromagnetic immunity
- Low emissions (CISPR 25)
- Shielded housings

#### Functional Safety (ISO 26262)

**ASIL Ratings:**
- ASIL-A: Lowest safety requirement
- ASIL-B: Rear-view camera, surround view
- ASIL-C/D: ADAS, autonomous driving

**Safety Mechanisms:**
- Built-in self-test (BIST)
- Error correction codes (ECC) on memory
- Watchdog timers
- Redundant sensors (stereo cameras)

**Failure Metrics:**
- SPFM: Single Point Fault Metric
- LFM: Latent Fault Metric
- PMHF: Probabilistic Metric for Hardware Failures
- Target: <10 FIT (failures per billion hours)

#### Computer Vision Optimization

**Object Detection:**
- Neural network processing
- ResNet, YOLO, SSD architectures
- Detect: Vehicles, pedestrians, cyclists, signs

**Lane Detection:**
- Edge detection, Hough transform
- Polynomial curve fitting
- Learning-based semantic segmentation

**Depth Estimation:**
- Stereo vision (dual cameras)
- Structure from motion
- Monocular depth (learning-based)

**Sensor Fusion:**
- Camera + radar + LiDAR
- Kalman filtering
- Complementary strengths (weather robustness)

### Future Image Sensor Technologies

#### Quantum Dot Sensors

**Principle:**
- Nanocrystals tune absorption wavelength
- Placed on top of photodiodes
- Better color separation than Bayer

**Advantages:**
- Higher QE (>80% across visible)
- Improved color accuracy
- Potentially eliminate CFA (color filter array)

**Challenges:**
- Manufacturing stability
- Cost

#### Organic Photodiodes

**Concept:**
- Organic semiconductors
- Transparent to visible light
- Stack above conventional silicon pixels

**Benefit:**
- >100% fill factor
- No Bayer CFA (each pixel sees RGB)

**Status:**
- Panasonic and Fujifilm research
- Low QE currently (<30%)

#### Event-Based Vision (Neuromorphic)

**Dynamic Vision Sensor (DVS):**
- Asynchronous operation
- Pixels output events (change in brightness)
- No frames, only changes

**Advantages:**
- Ultra-low latency (<1 μs)
- High temporal resolution (1 MHz)
- Low power (sparse data)
- Natural HDR (>120 dB)

**Applications:**
- High-speed object tracking
- Robotics and drones
- Gesture recognition

**Companies:**
- Prophesee (Sony partnership)
- Samsung (Gen4 sensor)
- Intel (acquisition of prophesee technology)

#### Curved Image Sensors

**Motivation:**
- Optical lens naturally projects curved image
- Flat sensor requires field flattening elements
- Curved sensor matches focal surface

**Benefits:**
- Simpler lens design (fewer elements)
- Larger aperture possible
- Improved edge performance

**Challenges:**
- Fabrication (wafer curving)
- Higher cost
- Limited volume applications

**Status:**
- Sony curved sensor prototypes
- NASA Kepler space telescope (curved CCD)

---

**References:**
- Nakamura, J. (2017). *Image Sensors and Signal Processing for Digital Still Cameras*. CRC Press.
- Theuwissen, A. (2007). *CMOS Image Sensors*. IEEE Solid-State Circuits Magazine.
- Sony, Samsung, OmniVision Technical Datasheets
- International Image Sensor Society (IISS) Symposium Proceedings

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
