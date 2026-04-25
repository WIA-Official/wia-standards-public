# WIA-QUA-013: Dark Matter Detection Specification v1.0

> **Standard ID:** WIA-QUA-013
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Dark Matter Physics Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Dark Matter Candidates](#2-dark-matter-candidates)
3. [Direct Detection Methods](#3-direct-detection-methods)
4. [Indirect Detection Methods](#4-indirect-detection-methods)
5. [Collider Searches](#5-collider-searches)
6. [Astrophysical Observations](#6-astrophysical-observations)
7. [Background Reduction](#7-background-reduction)
8. [Signal Discrimination](#8-signal-discrimination)
9. [Statistical Analysis](#9-statistical-analysis)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive methods for detecting and studying dark matter, the mysterious substance that comprises approximately 27% of the universe's energy density but has never been directly observed.

### 1.2 Scope

The standard covers:
- WIMP (Weakly Interacting Massive Particle) searches
- Axion and axion-like particle detection
- Direct detection technologies
- Indirect detection via astrophysical signatures
- Collider production and detection
- Gravitational lensing observations
- Background reduction and signal discrimination

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Understanding dark matter is one of the most fundamental questions in physics. This standard aims to provide a unified framework for dark matter detection methods that will advance our understanding of the universe.

### 1.4 Terminology

- **WIMP**: Weakly Interacting Massive Particle
- **Axion**: Hypothetical pseudoscalar particle
- **Nuclear Recoil (NR)**: Recoil of atomic nucleus from WIMP collision
- **Electron Recoil (ER)**: Recoil of electron from background radiation
- **QBER**: Quantum Bit Error Rate (for detector performance)
- **m.w.e.**: Meters water equivalent (shielding depth)
- **DRU**: Differential Rate Unit (events/keV/kg/day)

---

## 2. Dark Matter Candidates

### 2.1 WIMPs (Weakly Interacting Massive Particles)

#### 2.1.1 Properties

**Mass Range**:
```
10 GeV ≤ mχ ≤ 10 TeV
```

**Interaction**: Weak nuclear force and gravity

**Relic Density**:
```
Ωχh² ≈ 0.12
```

Achieved when annihilation rate Γ ≈ Hubble rate H at freeze-out.

**Thermal Production**:
```
⟨σv⟩ ≈ 3 × 10⁻²⁶ cm³/s
```

For thermal relic to match observed dark matter density.

#### 2.1.2 Theoretical Candidates

**Neutralino** (Supersymmetry):
```
χ⁰ = N₁₁B̃ + N₁₂W̃⁰ + N₁₃H̃⁰ᵤ + N₁₄H̃⁰ᵈ
```

Lightest supersymmetric particle (LSP), stable if R-parity conserved.

**Kaluza-Klein Particles** (Extra Dimensions):
- Lightest KK particle (LKP)
- Mass: 500 GeV - 2 TeV
- KK-parity conserved

**Inert Higgs Doublet**:
- Additional Higgs doublet
- Mass: 50 GeV - 1 TeV
- Z₂ symmetry

### 2.2 Axions

#### 2.2.1 Properties

**Mass**:
```
mₐ = (0.6 eV) × (10⁷ GeV / fₐ)
```

Where fₐ is the Peccei-Quinn scale.

**Coupling to Photons**:
```
gaγγ = (α / 2πfₐ) × (E/N - 1.92)
```

Where E/N depends on axion model.

**Production**: Misalignment mechanism, thermal production

**Lifetime**: Effectively stable (τ >> age of universe)

#### 2.2.2 QCD Axion

**Strong CP Problem**: Solves θ̄ < 10⁻¹⁰

**Mass-Coupling Relation**:
```
gaγγ ∝ mₐ
```

**Allowed Range**:
- Mass: 1 μeV - 1 meV
- Coupling: 10⁻¹⁶ - 10⁻¹⁰ GeV⁻¹

### 2.3 Sterile Neutrinos

#### 2.3.1 Properties

**Mass Range**: keV - GeV

**Interaction**: Gravity + mixing with active neutrinos

**Mixing Angle**:
```
sin²(2θ) < 10⁻⁶
```

For dark matter stability.

**X-ray Signature**:
```
Eγ = mₛ / 2
```

From radiative decay νₛ → νₐ + γ.

### 2.4 Other Candidates

#### Primordial Black Holes
- Mass: 10¹⁵ - 10²⁶ g
- Formation: Early universe fluctuations
- Detection: Gravitational lensing, mergers

#### Self-Interacting Dark Matter
- Strong self-interactions
- σ/m ≈ 1 cm²/g
- Solves small-scale structure problems

---

## 3. Direct Detection Methods

### 3.1 WIMP-Nucleus Scattering

#### 3.1.1 Differential Event Rate

**Standard Halo Model**:
```
dR/dEᴿ = (ρ₀σ₀/2mχμ²) × ∫ vf(v)F²(Eᴿ) dv
```

Where:
- ρ₀ = 0.3 GeV/cm³ (local dark matter density)
- σ₀ = WIMP-nucleon cross section
- mχ = WIMP mass
- μ = reduced mass
- f(v) = velocity distribution
- F²(Eᴿ) = nuclear form factor
- Eᴿ = recoil energy

**Recoil Energy**:
```
Eᴿ = (μ²v²/mₙ) × (1 - cos θ)
```

**Maximum Recoil**:
```
Eᴿ,max = 2μ²v²/mₙ
```

#### 3.1.2 Velocity Distribution

**Standard Halo Model** (Maxwellian):
```
f(v) = (1/N) × exp(-v²/v₀²)
```

Where:
- v₀ = 220 km/s (circular velocity)
- vₑₛc = 544 km/s (escape velocity)
- vₑ = 232 km/s (Earth's velocity)

**Annual Modulation**:
```
v(t) = v₀ + vₑ cos(2π(t - t₀)/T)
```

Where T = 1 year, t₀ ≈ June 2.

**Modulation Amplitude**: ~3-7% for most WIMPs

#### 3.1.3 Nuclear Form Factor

**Helm Form Factor**:
```
F(q) = 3j₁(qR₁)/(qR₁) × exp(-q²s²/2)
```

Where:
- j₁ = spherical Bessel function
- R₁ = √(R² - 5s²)
- R = 1.2A^(1/3) fm
- s ≈ 1 fm (skin thickness)
- q = √(2mₙEᴿ) (momentum transfer)

**Spin-Independent**: Coherent enhancement ∝ A²

**Spin-Dependent**: No coherent enhancement

### 3.2 Noble Liquid Detectors

#### 3.2.1 Liquid Xenon

**Detection Principle**: Two-phase time projection chamber (TPC)

**Signal Generation**:
1. **S1 (Prompt Scintillation)**:
   - UV photons (178 nm)
   - Timing: ~10 ns
   - Energy: E₁ = Nγ × Wγ

2. **S2 (Ionization)**:
   - Electrons drift to gas phase
   - Proportional scintillation
   - Position reconstruction: (x, y) from PMT pattern
   - Timing: drift time → z position

**Discrimination**:
```
log₁₀(S2/S1) vs. S1
```

Nuclear recoils: Low S2/S1 (~0.1-0.3)
Electron recoils: High S2/S1 (~1.0-3.0)

**Energy Resolution**:
```
σE/E ≈ 5% at 1 MeV
```

**Threshold**: ~1 keV nuclear recoil

**Advantages**:
- Self-shielding (outer region vetoes)
- Excellent discrimination
- Scalable to multi-ton

**Challenges**:
- ⁸⁵Kr contamination (β: 687 keV)
- Radon emanation
- Electronic noise

#### 3.2.2 Liquid Argon

**Isotope**: ⁴⁰Ar (99.6% natural abundance)

**Scintillation**: 128 nm UV light

**Pulse Shape Discrimination**:
- Fast component: τf ≈ 7 ns
- Slow component: τs ≈ 1.6 μs
- Nuclear recoils: Higher singlet/triplet ratio

**Formula**:
```
fprompt = Sprompt / (Sprompt + Slate)
```

Nuclear recoils: fprompt ≈ 0.7
Electron recoils: fprompt ≈ 0.3

**Underground Argon**: Depleted ³⁹Ar (β: 565 keV, τ = 269 yr)

**Advantages**:
- No ⁸⁵Kr contamination
- Lower cost than xenon
- Good pulse shape discrimination

### 3.3 Cryogenic Bolometers

#### 3.3.1 Phonon Detection

**Operating Temperature**: 10-50 mK

**Heat Capacity**:
```
C(T) = βT³
```

Where β depends on material.

**Temperature Rise**:
```
ΔT = E / C(T)
```

For energy deposition E.

**Sensor Types**:
1. **NTD Germanium**: Neutron transmutation doped
2. **TES**: Transition edge sensors
3. **MMC**: Metallic magnetic calorimeters

**Energy Resolution**: <100 eV (world's best)

#### 3.3.2 Dual Readout

**Phonon + Ionization**:
```
Yield = Eionization / Ephonon
```

Nuclear recoils: Lower yield (~30%)
Electron recoils: Higher yield (~100%)

**SuperCDMS**:
- Si and Ge crystals
- Mass: ~1 kg per detector
- Threshold: ~50 eV

**CRESST**:
- CaWO₄ crystals
- Phonon + scintillation
- Light yield discrimination

### 3.4 Directional Detection

#### 3.4.1 Principle

WIMP flux direction changes due to Earth's motion:
```
v⃗(t) = v⃗sun + v⃗earth(t)
```

**Expected Direction**: Toward Cygnus constellation

**Daily Modulation**: Due to Earth's rotation

#### 3.4.2 Gas Time Projection Chambers

**Gas**: CF₄, CS₂, or Ar:SF₆

**Readout**: Charge or optical (camera)

**Track Length**:
```
L ≈ 1 mm for 10 keV F recoil in CF₄
```

**Directionality**: Head-tail discrimination

**DRIFT Experiment**:
- CS₂ gas (1 atm)
- Negative ion drift
- Multi-wire proportional chamber

**Challenges**:
- Low density → large volume needed
- Head-tail asymmetry
- Backgrounds

### 3.5 Annual Modulation

#### 3.5.1 DAMA/LIBRA

**Detector**: NaI(Tl) scintillators (250 kg)

**Signal**:
```
S(t) = S₀ + Sₘ cos(2π(t - t₀)/T)
```

**Observation**: ~9σ modulation (2-6 keV)

**Amplitude**: Sₘ/S₀ ≈ 0.03

**Phase**: t₀ ≈ June 2 (as expected)

**Controversy**: Other experiments see no signal

### 3.6 Threshold and Low-Mass WIMPs

#### 3.6.1 Sub-GeV Detection

**Challenge**: Low recoil energy

**Maximum Recoil** (mχ = 1 GeV, mₙ = 131 GeV Xe):
```
Eᴿ,max ≈ 3.5 keV
```

**Solutions**:
1. **Light Targets**: He, Si (lower A)
2. **Electron Recoils**: WIMP-electron scattering
3. **Semiconductors**: Electron-hole pairs
4. **Superconductors**: Cooper pair breaking

**SENSEI**: Skipper CCDs, ~1 eV threshold

**SuperCDMS HVeV**: High voltage, ~10 eV threshold

---

## 4. Indirect Detection Methods

### 4.1 Gamma Ray Searches

#### 4.1.1 WIMP Annihilation

**Flux**:
```
Φγ = (⟨σv⟩/8πmχ²) × ∫ ρ²(r) dV × dNγ/dE
```

Where:
- ⟨σv⟩ = velocity-averaged cross section
- ρ(r) = dark matter density profile
- dNγ/dE = photon spectrum per annihilation

**Channels**:
- χχ → bb̄ → γ (continuous)
- χχ → W⁺W⁻ → γ (continuous)
- χχ → γγ (line at Eγ = mχ)
- χχ → γZ (line at Eγ = mχ(1 - mZ²/4mχ²))

**J-factor** (astrophysical factor):
```
J = ∫ ρ²(r) dΩ dl
```

#### 4.1.2 Targets

**Galactic Center**:
- High J-factor (~10²⁴ GeV² cm⁻⁵)
- Large astrophysical backgrounds
- NFW profile: ρ(r) ∝ r⁻¹(r + rₛ)⁻²

**Dwarf Spheroidal Galaxies**:
- Lower J-factor (~10¹⁸-10¹⁹)
- Clean targets (low background)
- Examples: Segue 1, Ursa Major II

**Galaxy Clusters**:
- Moderate J-factor
- Extended sources
- Example: Fornax cluster

#### 4.1.3 Fermi-LAT

**Energy Range**: 20 MeV - 300 GeV

**Angular Resolution**: 0.6° at 1 GeV

**Effective Area**: 8000 cm² at 1 GeV

**Galactic Center Excess**:
- ~1-3 GeV photons
- Spherically symmetric
- Interpretation: WIMPs (30-40 GeV) or pulsars?

#### 4.1.4 Ground-Based Cherenkov

**HESS, VERITAS, MAGIC**:
- Energy: 100 GeV - 100 TeV
- Angular resolution: 0.1°
- Effective area: ~10⁵ m²

**Atmospheric Cherenkov**:
- γ-ray → e⁺e⁻ cascade
- Cherenkov light in atmosphere
- Timing: ~ns

### 4.2 Neutrino Searches

#### 4.2.1 Solar WIMP Capture

**Capture Rate**:
```
Cₒ = ∫ nχ(v) σχ,i v f(u) du
```

**Equilibrium**: Capture = Annihilation²/2

**Time to Equilibrium**:
```
τₑq = 1/√(CₒΓₐ)
```

For Sun: τₑq < 10¹⁰ yr (reached equilibrium)

**Annihilation Rate**:
```
Γₐ = √(CₒΓₐ) tanh²(t/τₑq)
```

#### 4.2.2 Neutrino Flux from Sun

**Energy Spectrum**:
```
dΦν/dEν ∝ Γₐ × dNν/dEν
```

**Channels**:
- χχ → bb̄ → ν (soft spectrum)
- χχ → W⁺W⁻ → ν (hard spectrum)
- χχ → τ⁺τ⁻ → ν (intermediate)

**Oscillations**: νμ/ντ easier to detect than νₑ

#### 4.2.3 IceCube

**Detector**: 1 km³ ice, 5160 PMTs

**Depth**: 1450-2450 m

**Energy Range**: GeV - PeV

**Angular Resolution**: ~1° for νμ tracks

**Signal**: Upward-going muons
```
ν + N → μ + X
```

**Background**: Atmospheric neutrinos

**Limit**: ⟨σv⟩ < 10⁻²³ cm³/s (TeV WIMPs)

### 4.3 Antimatter Searches

#### 4.3.1 Positron Excess

**AMS-02 Observation**: e⁺/(e⁺ + e⁻) increases with energy

**Energy Range**: 1-350 GeV

**Possible Explanations**:
1. WIMP annihilation (mχ ~1-3 TeV)
2. Pulsars (confirmed nearby pulsars)
3. Astrophysical sources

**Challenge**: Propagation uncertainties

#### 4.3.2 Antiproton Flux

**AMS-02**: Antiproton-to-proton ratio

**Energy**: 1-450 GeV

**Excess?**: Some tension at ~10-20 GeV

**Secondary Production**:
```
p + ISM → p̄ + X
```

**WIMP Signature**:
```
χχ → qq̄ → p̄
```

---

## 5. Collider Searches

### 5.1 LHC WIMP Production

#### 5.1.1 Missing Transverse Energy

**Signature**:
```
pp → χχ + X
```

WIMPs escape → missing energy

**Channels**:
1. **Monojet**: pp → χχ + j
2. **Mono-photon**: pp → χχ + γ
3. **Mono-Z**: pp → χχ + Z
4. **Mono-Higgs**: pp → χχ + h

**Missing ET**:
```
E̸T = -∑ pT,visible
```

#### 5.1.2 Effective Field Theory

**Contact Interaction**:
```
ℒeff = (1/Λ²) × (q̄Γq)(χ̄Γ'χ)
```

Where Λ is the mediator mass scale.

**Validity**: Λ >> √s (low-energy limit)

**Simplified Models**: Include explicit mediator

#### 5.1.3 ATLAS and CMS Results

**Luminosity**: 139 fb⁻¹ (Run 2)

**Limits**: Λ > 1-3 TeV (depending on operator)

**Monojet**: Most stringent for spin-independent

**Translation**: Collider → direct detection cross section

### 5.2 Supersymmetry Searches

#### 5.2.1 Neutralino Production

**Processes**:
- Squark pair: pp → q̃q̃ → qq + χχ
- Gluino pair: pp → g̃g̃ → qq̄qq̄ + χχ
- Chargino/neutralino: pp → χ̃⁺χ̃⁻ → W⁺W⁻ + χχ

**Signatures**:
- Jets + MET
- Leptons + MET
- Same-sign dileptons

**Exclusions**:
- Squarks: > 2 TeV
- Gluinos: > 2.3 TeV
- Winos: > 650 GeV

---

## 6. Astrophysical Observations

### 6.1 Gravitational Lensing

#### 6.1.1 Strong Lensing

**Einstein Radius**:
```
θE = √(4GM/c² × Dₗₛ/(DₗDₛ))
```

Where:
- M = lens mass
- Dₗ, Dₛ, Dₗₛ = angular diameter distances

**Arc Formation**: Background galaxy distorted into arc

**Multiple Images**: Typically 2-4 images

**Mass Reconstruction**: Invert lens equation

#### 6.1.2 Weak Lensing

**Shear**:
```
γ = (a - b)/(a + b) × e^(2iφ)
```

Where a, b are semi-major/minor axes.

**Convergence**:
```
κ = Σ / Σcrit
```

Where Σ is surface mass density.

**Power Spectrum**: P(ℓ) measures matter distribution

#### 6.1.3 Cluster Mass Maps

**Abell 1689**: Total mass ~2 × 10¹⁵ M☉

**Mass-to-Light Ratio**: M/L ≈ 300 M☉/L☉

**Dark Matter Fraction**: ~85%

**Bullet Cluster**: Offset between DM (lensing) and gas (X-ray)

### 6.2 Rotation Curves

#### 6.2.1 Galaxy Rotation

**Expected** (visible matter only):
```
v(r) ∝ r⁻¹/² for r > R
```

**Observed**: v(r) ≈ constant (flat)

**NFW Profile**:
```
ρ(r) = ρₛ / [(r/rₛ)(1 + r/rₛ)²]
```

**Circular Velocity**:
```
v²(r) = GM(r)/r
```

Where M(r) = ∫ 4πr²ρ(r) dr

#### 6.2.2 Halo Mass

**Milky Way**: Mhalo ≈ 10¹² M☉

**Virial Radius**: Rvir ≈ 200 kpc

**Concentration**: c = Rvir/rs ≈ 10-20

### 6.3 Cosmic Microwave Background

#### 6.3.1 Dark Matter Density

**Planck Results**:
```
Ωch² = 0.1200 ± 0.0012
```

**Dark Matter Fraction**:
```
Ωc ≈ 27% of total energy density
```

**Baryon-to-DM Ratio**: Ωb/Ωc ≈ 0.18

#### 6.3.2 Structure Formation

**Matter Power Spectrum**: P(k)

**Transfer Function**: Suppression below horizon at matter-radiation equality

**CDM vs. WDM**: Free-streaming scale

---

## 7. Background Reduction

### 7.1 Radioactive Backgrounds

#### 7.1.1 Material Selection

**Screening**:
- HPGe detectors for γ spectroscopy
- ICP-MS for trace elements
- NAA for activation analysis

**Radiopurity Goals**:
- U/Th: < 1 ppb
- ⁴⁰K: < 1 ppt
- ⁶⁰Co: < μBq/kg
- ²²⁶Ra: < μBq/kg

**Clean Materials**:
- Electroformed copper
- Ancient lead (Roman, pre-industrial)
- High-purity germanium
- Radiopure plastics

#### 7.1.2 Radon Control

**²²²Rn** (τ = 3.8 days):
- Emanates from materials
- Plates out on surfaces
- Progeny: ²¹⁴Pb, ²¹⁴Bi (β/γ emitters)

**Mitigation**:
- Nitrogen atmosphere
- Radon-free cleanroom
- Active radon removal
- Charcoal traps

### 7.2 Cosmogenic Activation

#### 7.2.1 Neutron-Induced

**Surface Exposure**:
- Cosmic ray spallation
- Thermal neutrons

**Activated Isotopes** (Ge):
- ⁶⁸Ge (τ = 271 d) → ⁶⁸Ga → ⁶⁸Zn
- ⁶⁰Co (τ = 5.27 yr)
- ³H (τ = 12.3 yr)

**Xenon**:
- ¹²⁷Xe (τ = 36.4 d)
- ¹²⁵I from ¹²⁶Xe (τ = 13.1 d)

**Mitigation**:
- Minimize surface time
- Shield during transport
- Underground storage
- Decay time

### 7.3 Depth Requirements

#### 7.3.1 Muon Flux Reduction

**Surface Muon Flux**: ~1 cm⁻²min⁻¹

**Vertical Intensity**:
```
I(h) = I₀ × exp(-h/Λ)
```

Where Λ ≈ 1500 m.w.e.

**Muon-Induced Neutrons**:
```
Yn ≈ 10⁻³ neutrons per muon
```

**Depth Requirements**:
- Ton-scale: > 3000 m.w.e.
- 10-ton: > 4000 m.w.e.
- 100-ton: > 5000 m.w.e.

### 7.4 Shielding

#### 7.4.1 Passive Shielding

**Water Tank**: Neutron moderator, muon veto

**Polyethylene**: Neutron moderator (high H content)

**Lead**: Gamma shield
- Ancient lead: Low ²¹⁰Pb
- Thickness: 10-20 cm

**Copper**: Inner shielding
- Electroformed or OFHC
- Thickness: 5-10 cm

#### 7.4.2 Active Veto

**Muon Veto**:
- Water Cherenkov
- Plastic scintillator
- Efficiency: > 99.5%

**Neutron Veto**:
- Gd-loaded water
- ³He tubes
- Boron-loaded plastic

---

## 8. Signal Discrimination

### 8.1 Pulse Shape Discrimination

#### 8.1.1 Liquid Argon

**Scintillation Time Profile**:
```
N(t) = Nf exp(-t/τf) + Ns exp(-t/τs)
```

Where:
- τf ≈ 7 ns (singlet)
- τs ≈ 1.6 μs (triplet)

**Prompt Fraction**:
```
fprompt = Nf / (Nf + Ns)
```

**Nuclear Recoils**: fprompt ≈ 0.7
**Electron Recoils**: fprompt ≈ 0.3

**Rejection Factor**: > 10⁸

#### 8.1.2 Liquid Xenon

**S2/S1 Ratio**:
- Nuclear: 0.1-0.3
- Electron: 1.0-3.0

**Log-Likelihood**:
```
ℒNR = Σ log[P(S1, S2 | NR)]
ℒER = Σ log[P(S1, S2 | ER)]
```

**Discrimination**: ℒNR - ℒER > threshold

### 8.2 Multiple Scatter Rejection

**Single Scatter**: WIMP candidate

**Multiple Scatter**: Background (Compton, neutron)

**Position Reconstruction**: Reject events with > 1 site

**Fiducial Volume**: Inner region only

### 8.3 Machine Learning

#### 8.3.1 Neural Networks

**Input Features**:
- S1, S2 (charge, light)
- Pulse shape parameters
- Position (x, y, z)
- Timing information

**Architecture**:
- Dense layers: 3-5 layers
- Activation: ReLU, sigmoid
- Output: P(NR) vs. P(ER)

**Training**: Labeled calibration data

**Performance**: Comparable or better than traditional

#### 8.3.2 Boosted Decision Trees

**XGBoost, LightGBM**:
- Fast training
- Handle missing data
- Feature importance

**Features**:
- Energy
- S2/S1
- Pulse widths
- Asymmetry

**Hyperparameters**:
- Trees: 100-1000
- Depth: 3-10
- Learning rate: 0.01-0.1

---

## 9. Statistical Analysis

### 9.1 Likelihood Analysis

#### 9.1.1 Profile Likelihood

**Likelihood**:
```
ℒ(μ, θ) = ∏ᵢ P(nᵢ | μsᵢ(θ) + bᵢ(θ))
```

Where:
- μ = signal strength
- θ = nuisance parameters
- sᵢ = expected signal
- bᵢ = expected background

**Profile Likelihood Ratio**:
```
λ(μ) = ℒ(μ, θ̂μ) / ℒ(μ̂, θ̂)
```

**Test Statistic**:
```
qμ = -2 ln λ(μ)
```

#### 9.1.2 p-value

**Significance**:
```
p = ∫qobs^∞ f(q | H₀) dq
```

**Discovery**: p < 2.87 × 10⁻⁷ (5σ)

**Evidence**: p < 0.0013 (3σ)

### 9.2 Limits

#### 9.2.1 Frequentist (CLₛ)

**CLₛ Method**:
```
CLₛ = CLₛ₊b / CLb
```

**90% CL Upper Limit**: CLₛ(σ₉₀) = 0.10

**Advantages**:
- Conservative
- Standard in particle physics

#### 9.2.2 Bayesian

**Posterior**:
```
P(σ | data) ∝ ℒ(data | σ) × π(σ)
```

**Prior**: Flat in log(σ) or σ

**90% Credible Interval**:
```
∫₀^σ₉₀ P(σ | data) dσ = 0.90
```

### 9.3 Optimum Interval Method

**Feldman-Cousins**: Unified approach

**Confidence Belt**: Neyman construction

**Ordering**: Likelihood ratio

---

## 10. Implementation Guidelines

### 10.1 Detector Design

**Energy Threshold**: As low as possible
- Phonon: < 100 eV
- Ionization: ~keV
- Scintillation: ~keV

**Fiducial Mass**: Maximize signal

**Background Budget**: < 1 event in ROI

**Radiopurity**: Screen all materials

### 10.2 Calibration

**Electron Recoil**:
- ⁵⁷Co (122 keV)
- ¹³³Ba (356 keV)
- ⁶⁰Co (1173, 1332 keV)
- ²²⁸Th (2615 keV)

**Nuclear Recoil**:
- ²⁴¹AmBe (neutron source)
- ²⁵²Cf (fission neutrons)
- DD generator (2.45 MeV n)
- DT generator (14.1 MeV n)

**Frequency**: Daily to monthly

### 10.3 Data Quality

**Cuts**:
- Detector performance
- Data acquisition
- Environmental (temperature, etc.)

**Blinding**: Hide signal region until analysis complete

**Validation**: Cross-checks, consistency

### 10.4 Systematics

**Sources**:
- Energy scale
- Background model
- Efficiency
- Astrophysical parameters

**Treatment**: Nuisance parameters in likelihood

---

## 11. References

### 11.1 Foundational Papers

1. Jungman, G., Kamionkowski, M., & Griest, K. (1996). "Supersymmetric dark matter." *Physics Reports*, 267(5-6), 195-373.

2. Lewin, J. D., & Smith, P. F. (1996). "Review of mathematics, numerical factors, and corrections for dark matter experiments based on elastic nuclear recoil." *Astroparticle Physics*, 6(1), 87-112.

3. Goodman, M. W., & Witten, E. (1985). "Detectability of certain dark-matter candidates." *Physical Review D*, 31(12), 3059.

4. Peccei, R. D., & Quinn, H. R. (1977). "CP conservation in the presence of pseudoparticles." *Physical Review Letters*, 38(25), 1440.

### 11.2 Direct Detection

5. Aprile, E., et al. (XENON Collaboration). (2020). "Excess electronic recoil events in XENON1T." *Physical Review D*, 102(7), 072004.

6. Agnese, R., et al. (SuperCDMS Collaboration). (2018). "Low-mass dark matter search with CDMSlite." *Physical Review D*, 97(2), 022002.

7. Angloher, G., et al. (CRESST Collaboration). (2019). "Results on MeV-scale dark matter from a gram-scale cryogenic calorimeter." *European Physical Journal C*, 79(12), 1014.

### 11.3 Indirect Detection

8. Ackermann, M., et al. (Fermi-LAT Collaboration). (2015). "Searching for dark matter annihilation from Milky Way dwarf spheroidal galaxies." *Physical Review Letters*, 115(23), 231301.

9. Aartsen, M. G., et al. (IceCube Collaboration). (2016). "Search for annihilating dark matter in the Sun with 3 years of IceCube data." *European Physical Journal C*, 77(3), 146.

10. Aguilar, M., et al. (AMS Collaboration). (2019). "Towards understanding the origin of cosmic-ray positrons." *Physical Review Letters*, 122(4), 041102.

### 11.4 Colliders

11. Aad, G., et al. (ATLAS Collaboration). (2021). "Search for new phenomena in events with an energetic jet and missing transverse momentum in pp collisions." *Physical Review D*, 103(11), 112006.

### 11.5 Astrophysics

12. 선행 연구. "A direct empirical proof of the existence of dark matter." *Astrophysical Journal Letters*, 648(2), L109.

13. Planck Collaboration. (2020). "Planck 2018 results. VI. Cosmological parameters." *Astronomy & Astrophysics*, 641, A6.

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
