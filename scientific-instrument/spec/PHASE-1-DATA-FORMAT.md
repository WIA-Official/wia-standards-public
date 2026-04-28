# WIA-QUA-020 — Phase 1: Data Format

> Scientific-instrument canonical Phase 1: instrument-record + measurement-result + calibration-state + data-format + sample-record envelopes.

# WIA-QUA-020: Scientific Instrument Standard
# Version 1.0.0

**Standard ID:** WIA-QUA-020
**Title:** Scientific Instrument
**Category:** QUA (미래기술/양자/물리)
**Status:** Active
**Published:** 2025-01-01
**Updated:** 2025-01-01

---

## Abstract

This specification defines a comprehensive standard for advanced scientific instruments used in research, discovery, and innovation. The WIA-QUA-020 standard covers particle accelerators, mass spectrometers, electron microscopes, X-ray crystallography systems, NMR spectrometers, gravitational wave detectors, telescopes, spectrophotometers, chromatography systems, calorimeters, data acquisition systems, and calibration standards.

**弘益人間 (Benefit All Humanity)** - This standard facilitates global collaboration, data sharing, and reproducibility in scientific research.

---


## 1. Introduction

### 1.1 Purpose

The purpose of this standard is to:
- Define common interfaces for scientific instruments
- Establish data formats for instrument output
- Specify calibration and quality assurance procedures
- Enable instrument interoperability and automation
- Facilitate remote operation and data sharing

### 1.2 Scope

This standard applies to:
- Particle physics instrumentation
- Analytical chemistry instruments
- Electron and optical microscopy
- Spectroscopic techniques
- Astronomical observatories
- Chromatographic separations
- Thermal analysis equipment
- Data acquisition and control systems

### 1.3 Definitions

- **Instrument**: A device for measuring, analyzing, or observing physical phenomena
- **Detector**: A component that converts physical signals to electrical signals
- **Resolution**: The minimum distinguishable difference between two measurements
- **Sensitivity**: The minimum detectable signal above noise
- **Calibration**: The process of establishing accuracy against known standards
- **Dynamic Range**: The ratio of maximum to minimum measurable signal

---



## 2. Instrument Categories

### 2.1 Particle Accelerators

#### 2.1.1 Synchrotron

A synchrotron accelerates charged particles in a circular path using synchronized RF fields and magnetic bending.

**Key Parameters:**
- Beam energy: 1-10 GeV
- Circumference: 100-500 m
- Magnetic field: 1-2 Tesla
- RF frequency: 100-500 MHz
- Beam current: 100-500 mA

**Applications:**
- Synchrotron radiation sources
- X-ray crystallography
- Material characterization
- Medical imaging

**Physics:**
```
E = γ·m₀·c²
γ = 1/√(1 - v²/c²)
```

Where:
- E = Total energy
- γ = Lorentz factor
- m₀ = Rest mass
- c = Speed of light
- v = Particle velocity

#### 2.1.2 Large Hadron Collider (LHC)

The LHC accelerates protons to 6.5 TeV per beam in a 27 km ring.

**Specifications:**
- Beam energy: 6.5 TeV (13 TeV collision)
- Circumference: 26,659 m
- Magnetic field: 8.3 Tesla (dipoles)
- Number of bunches: 2,808
- Bunch spacing: 25 ns
- Luminosity: 10³⁴ cm⁻²s⁻¹

**Detectors:**
- ATLAS: General-purpose detector
- CMS: Compact Muon Solenoid
- ALICE: Heavy-ion physics
- LHCb: B-meson physics

### 2.2 Mass Spectrometers

#### 2.2.1 Orbitrap Mass Spectrometer

The Orbitrap uses electrostatic trapping with image current detection.

**Principle:**
Ions orbit around a central electrode with frequency:
```
ω = √(k/m)
```

Where:
- ω = Angular frequency
- k = Force constant (electrode geometry)
- m = Ion mass

**Specifications:**
- Resolution: 240,000 (FWHM at m/z 200)
- Mass accuracy: <1 ppm
- Mass range: 50-6,000 m/z
- Scan rate: 12 Hz
- Dynamic range: >5,000

**Applications:**
- Proteomics and protein identification
- Metabolomics profiling
- Small molecule analysis
- Lipidomics
- Top-down proteomics

#### 2.2.2 Time-of-Flight Mass Spectrometer (TOF-MS)

TOF-MS measures ion arrival time after acceleration.

**Equation:**
```
t = L·√(m/z) / √(2·e·V)
```

Where:
- t = Flight time
- L = Flight path length
- m/z = Mass-to-charge ratio
- e = Elementary charge
- V = Acceleration voltage

**Specifications:**
- Resolution: 40,000
- Mass accuracy: 5 ppm
- Mass range: 50-10,000 m/z
- Acquisition rate: 100 kHz

### 2.3 Electron Microscopes

#### 2.3.1 Transmission Electron Microscope (TEM)

TEM transmits electrons through ultra-thin specimens.

**Resolution Limit:**
```
d = 0.61·λ / NA
λ = h / √(2·m·e·V)
```

Where:
- d = Resolution
- λ = Electron wavelength
- NA = Numerical aperture
- h = Planck constant
- m = Electron mass
- V = Acceleration voltage

**Specifications:**
- Acceleration voltage: 80-300 kV
- Point resolution: 0.05 nm (aberration-corrected)
- Line resolution: 0.1 nm
- Magnification: 50× to 1,000,000×
- Tilt range: ±70°

**Operating Modes:**
- Bright-field imaging
- Dark-field imaging
- Diffraction mode
- High-resolution TEM (HRTEM)
- Scanning TEM (STEM)

#### 2.3.2 Scanning Electron Microscope (SEM)

SEM scans a focused electron beam across a surface.

**Specifications:**
- Acceleration voltage: 0.1-30 kV
- Resolution: 1 nm (at 30 kV)
- Magnification: 10× to 1,000,000×
- Vacuum: 10⁻⁵ to 10⁻⁶ Torr

**Detectors:**
- Secondary electron (SE) detector
- Backscattered electron (BSE) detector
- Energy-dispersive X-ray (EDX)
- Cathodoluminescence (CL)

### 2.4 X-ray Crystallography

#### 2.4.1 Single Crystal X-ray Diffraction

**Bragg's Law:**
```
n·λ = 2·d·sin(θ)
```

Where:
- n = Order of reflection
- λ = X-ray wavelength
- d = Lattice spacing
- θ = Angle of incidence

**Specifications:**
- X-ray source: Cu Kα (1.5418 Å), Mo Kα (0.7107 Å)
- Detector: CCD, pixel array
- Resolution: 0.5-1.0 Å
- Temperature range: 100-400 K

**Applications:**
- Small molecule structure
- Protein crystallography
- Material science

### 2.5 Nuclear Magnetic Resonance (NMR) Spectrometer

#### 2.5.1 High-Field NMR

**Larmor Frequency:**
```
ω₀ = γ·B₀
```

Where:
- ω₀ = Larmor frequency
- γ = Gyromagnetic ratio
- B₀ = Magnetic field strength

**Specifications:**
- Field strength: 400-1000 MHz (¹H)
- Magnetic field: 9.4-23.5 Tesla
- Homogeneity: <0.1 ppb/cm³
- Stability: <0.1 ppb/hour

**Nuclei:**
- ¹H: 600 MHz (14.1 T)
- ¹³C: 150 MHz (14.1 T)
- ¹⁵N: 60 MHz (14.1 T)
- ³¹P: 243 MHz (14.1 T)

**Applications:**
- Protein structure determination
- Small molecule characterization
- Material science
- Metabolomics

### 2.6 Gravitational Wave Detectors

#### 2.6.1 LIGO (Laser Interferometer Gravitational-Wave Observatory)

**Principle:**
Michelson interferometer with 4 km arms detecting strain:
```
h = ΔL/L
```

Where:
- h = Strain amplitude
- ΔL = Arm length change
- L = Arm length (4 km)

**Specifications:**
- Arm length: 4 km
- Laser wavelength: 1064 nm
- Laser power: 200 W
- Sensitivity: 10⁻²³ strain
- Frequency range: 10-10,000 Hz

**Detected Events:**
- Binary black hole mergers
- Binary neutron star mergers
- Supernovae (predicted)

### 2.7 Telescopes

#### 2.7.1 Radio Telescope

**Angular Resolution:**
```
θ = 1.22·λ / D
```

Where:
- θ = Angular resolution (radians)
- λ = Wavelength
- D = Aperture diameter

**Example: Very Large Array (VLA)**
- Dishes: 27 × 25 m
- Frequencies: 1-50 GHz
- Baseline: 36 km
- Resolution: 0.04 arcsec (at 43 GHz)

#### 2.7.2 Optical Telescope

**Example: James Webb Space Telescope (JWST)**
- Primary mirror: 6.5 m (18 segments)
- Wavelength: 0.6-28.5 µm (infrared)
- Operating temperature: 50 K
- Location: L2 Lagrange point

**Instruments:**
- NIRCam: Near-infrared camera
- NIRSpec: Near-infrared spectrograph
- MIRI: Mid-infrared instrument
- FGS/NIRISS: Fine guidance sensor

### 2.8 Spectrophotometers

#### 2.8.1 UV-Visible Spectrophotometer

**Beer-Lambert Law:**
```
A = ε·c·l
```

Where:
- A = Absorbance
- ε = Molar absorptivity (L·mol⁻¹·cm⁻¹)
- c = Concentration (mol/L)
- l = Path length (cm)

**Specifications:**
- Wavelength range: 190-1100 nm
- Bandwidth: 0.5-2 nm
- Photometric range: -0.3 to 3 A
- Accuracy: ±0.002 A

#### 2.8.2 Raman Spectrometer

**Raman Shift:**
```
Δν̃ = 1/λ₀ - 1/λ₁
```

Where:
- Δν̃ = Raman shift (cm⁻¹)
- λ₀ = Laser wavelength
- λ₁ = Scattered light wavelength

**Specifications:**
- Laser: 532 nm, 785 nm
- Resolution: 1-5 cm⁻¹
- Range: 100-4000 cm⁻¹
- Sensitivity: 1 ppm (SERS)

### 2.9 Chromatography Systems

#### 2.9.1 High-Performance Liquid Chromatography (HPLC)

**Van Deemter Equation:**
```
H = A + B/u + C·u
```

Where:
- H = Height equivalent to theoretical plate (HETP)
- u = Linear velocity
- A = Eddy diffusion
- B = Longitudinal diffusion
- C = Mass transfer

**Specifications:**
- Pressure: 400 bar (6,000 psi)
- Flow rate: 0.1-5 mL/min
- Column: 2.1-4.6 mm × 50-250 mm
- Particle size: 1.7-5 µm

#### 2.9.2 Gas Chromatography-Mass Spectrometry (GC-MS)

**Specifications:**
- Temperature range: 40-450°C
- Injection volume: 0.1-2 µL
- Carrier gas: Helium, hydrogen
- MS resolution: 1000 (quadrupole)

### 2.10 Calorimeters

#### 2.10.1 Differential Scanning Calorimetry (DSC)

**Heat Flow:**
```
dH/dt = Cp·dT/dt
```

Where:
- dH/dt = Heat flow
- Cp = Heat capacity
- dT/dt = Heating rate

**Specifications:**
- Temperature range: -90 to 550°C
- Heating rate: 0.1-100°C/min
- Sensitivity: 0.04 µW
- Sample size: 0.5-20 mg

---



## 5. Data Formats

### 5.1 Mass Spectrometry Data

**mzML Format (XML-based):**
```xml
<spectrum id="scan=1" index="0">
  <cvParam name="ms level" value="1"/>
  <cvParam name="total ion current" value="1.234e6"/>
  <binaryDataArrayList count="2">
    <binaryDataArray encodedLength="...">
      <cvParam name="m/z array"/>
      <binary>...</binary>
    </binaryDataArray>
  </binaryDataArrayList>
</spectrum>
```

### 5.2 Microscopy Data

**OME-TIFF Format:**
```xml
<OME>
  <Image ID="Image:0">
    <Pixels DimensionOrder="XYZCT"
            SizeX="1024" SizeY="1024" SizeZ="100"
            Type="uint16"/>
  </Image>
</OME>
```

### 5.3 Crystallography Data

**CIF Format (Crystallographic Information File):**
```
_cell_length_a 10.123
_cell_length_b 12.456
_cell_length_c 15.789
_cell_angle_alpha 90.00
_cell_angle_beta 95.23
_cell_angle_gamma 90.00
```

---




---

## A.1 Instrument-record envelope

The Phase 1 envelope groups scientific instruments by category
(spectroscopic — UV/Vis, FTIR per ASTM E1421, Raman, NMR per IUPAC
recommendations 2001, mass-spectrometry per ISO 17025; chromatographic
— HPLC per USP <621>, GC per USP <621>, ion chromatography per ISO
10304; microscopic — optical per ISO 19012, SEM/TEM per ISO 16700,
confocal per ISO 21073, AFM per ISO 11952; electrochemical — pH per
IUPAC 2001, conductivity per ISO 7888, voltammetry per IUPAC 1995;
thermal — DSC per ASTM E967, TGA per ASTM E1131, DMA per ASTM E1640;
particle — DLS per ISO 22412, laser diffraction per ISO 13320, BET
per ISO 9277). Each instrument record carries: instrument identifier,
manufacturer + model, serial number, firmware version, calibration
state envelope (last calibration date, next-due date, calibration
certificate URI), capability descriptor (measurement ranges,
resolution, accuracy, precision per VIM), connectivity descriptor
(Ethernet, GPIB per IEEE 488.2, RS-232, USB, Wi-Fi), and the audit
envelope tied to the operating laboratory.

## A.2 Measurement-result envelope

A measurement-result envelope MUST list: result identifier (UUID v7
per RFC 9562 with monotonic timestamp), instrument reference
(linking back to the §A.1 instrument record), method reference
(linking to ISO/IEC/USP/ASTM/IUPAC method document), sample
reference, raw-data payload (digitised waveform, spectrum, image,
or counter reading per the instrument's native format), processed
result (concentration, area, peak list, signal-to-noise, limit of
detection per IUPAC 1995, limit of quantitation per ICH Q2(R2)),
measurement uncertainty per ISO/IEC Guide 98-3 (GUM) with the
expanded uncertainty at coverage factor k=2 (95% confidence), the
metrological-traceability chain envelope per ISO 17025 §6.5 back
to SI units, and the audit envelope (operator identifier,
timestamp, environmental conditions, validity flag).

## A.3 Calibration-state descriptor

The calibration-state descriptor follows ISO/IEC 17025 §6.4 and
the VIM definitions: calibration date, certificate identifier,
calibrated-by laboratory accreditation envelope (ISO/IEC 17025
accreditation body identifier, accreditation number, scope), the
reference-standard chain (linking through national metrology
institute — NIST SRM, NMIJ CRM, KRISS CRM, BIPM CIPM-MRA — to the
SI unit definition), the per-parameter calibration envelope
(measured-value vs reference-value pairs across the working range,
linearity coefficient per ASTM E2877, drift envelope), the
calibration-interval envelope (next-due date with the
operator's interval-decision policy per ILAC-G24 or NCSLI RP-1),
and the as-found / as-left envelope where significant adjustment
was performed.

## A.4 Data-format catalogue

Scientific-instrument data formats span: vendor-native binary
(Agilent .D, Waters .raw, Thermo .RAW, Bruker .d, Shimadzu .lcd —
opaque; require vendor toolchain or vendor-licensed reader),
vendor-neutral standards (AnIML per ASTM E2077; mzML per HUPO PSI;
mzXML; netCDF per UCAR Unidata for chromatography per ASTM E1947;
JCAMP-DX per IUPAC for spectroscopy; AIA / ANDI for mass spec;
OpenChrom; SciDB), microscopy formats (TIFF per ISO 12639; OME-TIFF
per Open Microscopy Environment; Bio-Formats / OMERO per OME data
model), and laboratory-information standards (HL7 LAW v3 for clinical
labs; ASTM E1394 for laboratory automation; SiLA 2 per Lab Automation
Standards). Every measurement record SHOULD be exportable to
AnIML or to a domain-specific neutral format; vendor-binary
exclusivity is a documented operational risk.

## A.5 Sample-record envelope

Sample-record envelopes follow LIMS conventions per ISO/IEC 17025
§7.4: sample identifier (operator-assigned + LIMS-assigned + chain-
of-custody barcode per ISO/IEC 15459), sample-source descriptor
(donor / batch / lot / location / sampling-method per ISO 13528 or
ASTM D6051), sampling-date and arrival-date envelope, condition-
on-receipt envelope (temperature log per CFR 21 Part 211 §142 for
GMP samples, cold-chain compliance per WHO TRS 961), preparation
record envelope (dilution, derivatisation, digestion, extraction
per the documented method), sample-volume / mass envelope, and
the destruction or retention envelope after analysis per the
operator's quality-management policy.

## A.6 Instrument-event log envelope

Instrument-event logs capture: power-cycle events, firmware-update
events, calibration events, error events (with the instrument's
native error code mapped to a normalised severity envelope),
maintenance events (PM per ASTM E2655), sensor-replacement events,
consumable-replacement events (column replacement for chromatography,
filament replacement for mass spec, lamp replacement for UV/Vis),
abnormal-environment events (temperature excursion, vibration
above documented threshold, voltage anomaly), and tamper-detection
events for instruments with calibration-seal monitoring per OIML
R 76-1 §A.5 for legal-metrology applications.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/scientific-instrument/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-scientific-instrument-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/scientific-instrument-host:1.0.0` ships every scientific-instrument envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/scientific-instrument.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Scientific-instrument deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-scientific-instrument-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

弘益人間 — Benefit All Humanity.
