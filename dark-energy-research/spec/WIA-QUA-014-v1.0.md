# WIA-QUA-014: Dark Energy Research Specification v1.0

> **Standard ID:** WIA-QUA-014
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Dark Energy Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Cosmological Constant (Lambda)](#2-cosmological-constant-lambda)
3. [Quintessence Models](#3-quintessence-models)
4. [Accelerating Universe Expansion](#4-accelerating-universe-expansion)
5. [Equation of State Parameter](#5-equation-of-state-parameter)
6. [Type Ia Supernovae Observations](#6-type-ia-supernovae-observations)
7. [Baryon Acoustic Oscillations](#7-baryon-acoustic-oscillations)
8. [CMB Measurements](#8-cmb-measurements)
9. [Hubble Constant Tension](#9-hubble-constant-tension)
10. [Dark Energy Surveys](#10-dark-energy-surveys)
11. [Modified Gravity Theories](#11-modified-gravity-theories)
12. [Vacuum Energy](#12-vacuum-energy)
13. [Future Cosmological Fate](#13-future-cosmological-fate)
14. [Implementation Guidelines](#14-implementation-guidelines)
15. [References](#15-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the framework for dark energy research, encompassing theoretical models, observational techniques, and computational methods for understanding the mysterious force driving the accelerating expansion of the universe.

### 1.2 Scope

The standard covers:
- Cosmological constant (Λ) and vacuum energy
- Dynamic dark energy models (quintessence, phantom energy)
- Observational probes (supernovae, BAO, CMB, weak lensing)
- Equation of state parameter w(z)
- Modified gravity alternatives to dark energy
- Cosmological simulations and data analysis

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to advance our understanding of dark energy and the ultimate fate of the universe, providing tools and methodologies that enable researchers worldwide to contribute to solving one of the greatest mysteries in physics.

### 1.4 Terminology

- **Dark Energy**: Unknown form of energy causing accelerated cosmic expansion
- **Cosmological Constant (Λ)**: Einstein's constant representing vacuum energy
- **Equation of State**: w = P/ρ, relating pressure to energy density
- **Redshift (z)**: Measure of cosmic expansion, (λ_observed - λ_emitted)/λ_emitted
- **Hubble Parameter**: H(z), expansion rate as function of redshift
- **Critical Density**: ρ_c, density for flat universe
- **Density Parameter**: Ω = ρ/ρ_c, fraction of critical density

---

## 2. Cosmological Constant (Lambda)

### 2.1 Einstein Field Equations with Λ

The Einstein field equations including the cosmological constant:

```
Gμν + Λgμν = (8πG/c⁴)Tμν
```

Where:
- Gμν = Ricci tensor - ½R×gμν (Einstein tensor)
- Λ = cosmological constant
- gμν = metric tensor
- Tμν = stress-energy tensor
- G = gravitational constant
- c = speed of light

### 2.2 Vacuum Energy Interpretation

The cosmological constant can be interpreted as vacuum energy density:

```
ρ_Λ = Λc²/(8πG)
```

**Current Observational Value**:
```
Λ ≈ 1.1 × 10⁻⁵² m⁻²
ρ_Λ ≈ 6 × 10⁻²⁷ kg/m³
Ω_Λ ≈ 0.685 (68.5% of total energy density)
```

### 2.3 Cosmological Constant Problem

**Quantum Field Theory Prediction**:
```
ρ_vacuum(QFT) ~ (Planck Energy)⁴ ~ 10¹¹³ J/m³
```

**Observed Value**:
```
ρ_Λ(observed) ~ 10⁻⁹ J/m³
```

**Discrepancy**: Factor of ~10¹²² - the worst prediction in physics!

### 2.4 Equation of State

For cosmological constant:
```
w_Λ = P_Λ/ρ_Λ = -1 (exactly)
```

This implies:
- Negative pressure (repulsive gravity)
- Constant energy density (does not dilute with expansion)
- Time-independent

---

## 3. Quintessence Models

### 3.1 Scalar Field Quintessence

Dynamic dark energy represented by scalar field φ(t):

**Lagrangian**:
```
L = ½∂μφ∂μφ - V(φ)
```

**Energy Density**:
```
ρ_φ = ½φ̇² + V(φ)
```

**Pressure**:
```
P_φ = ½φ̇² - V(φ)
```

**Equation of State**:
```
w_φ = (½φ̇² - V(φ))/(½φ̇² + V(φ))
```

### 3.2 Quintessence Potentials

#### 3.2.1 Power-Law Potential
```
V(φ) = V₀φ^(-α)
```

**Tracker Solution**: Equation of state tracks radiation/matter equation of state until recent times.

#### 3.2.2 Exponential Potential
```
V(φ) = V₀ exp(-λφ/M_pl)
```

Where M_pl = Planck mass ≈ 1.22 × 10¹⁹ GeV/c²

**Scaling Solution**: Energy density scales with background (radiation or matter).

#### 3.2.3 Pseudo-Nambu-Goldstone Boson
```
V(φ) = M⁴[1 + cos(φ/f)]
```

Where f is the symmetry-breaking scale.

### 3.3 Phantom Energy

Scalar field with negative kinetic energy:

**Equation of State**:
```
w < -1
```

**Consequences**:
- "Big Rip" scenario
- All structures torn apart in finite future
- Violates null energy condition

**Big Rip Time** (if w = constant < -1):
```
t_rip = t_0 + (2/3H₀|1+w|)
```

For w = -1.5: t_rip ~ 22 billion years from now

### 3.4 K-Essence

Non-canonical kinetic term:

**Lagrangian**:
```
L = K(φ,X)
```

Where X = -½∂μφ∂μφ

**Equation of State**:
```
w = K/(2X∂K/∂X - K)
```

---

## 4. Accelerating Universe Expansion

### 4.1 Friedmann Equations

For flat universe (k=0):

**First Friedmann Equation**:
```
H² = (ȧ/a)² = (8πG/3)ρ
```

**Second Friedmann Equation** (Acceleration Equation):
```
ä/a = -(4πG/3)(ρ + 3P)
```

Where:
- a(t) = scale factor
- H = Hubble parameter
- ρ = total energy density
- P = total pressure

### 4.2 Deceleration Parameter

```
q = -ä/(aH²) = -1 - Ḣ/H²
```

**Current Value**: q₀ ≈ -0.55

**Interpretation**:
- q > 0: Decelerating expansion
- q < 0: Accelerating expansion

### 4.3 Transition Redshift

Redshift at which universe transitioned from deceleration to acceleration:

For ΛCDM model:
```
z_acc = (2Ω_Λ/Ω_m)^(1/3) - 1 ≈ 0.67
```

**Cosmic Time**: t_acc ≈ 7.5 billion years after Big Bang

### 4.4 Scale Factor Evolution

For matter + dark energy:

```
a(t) ∝ sinh^(2/3)(3H₀√Ω_Λ t/2)
```

**Asymptotic Behavior**:
- Early times: a(t) ∝ t^(2/3) (matter-dominated)
- Late times: a(t) ∝ exp(H₀√Ω_Λ t) (dark energy-dominated)

---

## 5. Equation of State Parameter

### 5.1 Definition

```
w(z) = P(z)/ρ(z)
```

**Standard Values**:
- Radiation: w = 1/3
- Matter: w = 0
- Cosmological constant: w = -1
- Quintessence: -1 < w < -1/3
- Phantom energy: w < -1

### 5.2 Parameterizations

#### 5.2.1 Constant w
```
w(z) = w₀
```

Simplest extension of ΛCDM.

#### 5.2.2 CPL Parameterization (Chevallier-Polarski-Linder)
```
w(z) = w₀ + w_a z/(1+z)
```

**Parameters**:
- w₀ = present-day equation of state
- w_a = evolution parameter

**Current Constraints**:
- w₀ = -1.03 ± 0.03 (Planck + BAO + Pantheon)
- w_a = -0.03 ± 0.3

#### 5.2.3 Linear Parameterization
```
w(a) = w₀ + w_a(1-a)
```

Where a = 1/(1+z) is scale factor.

### 5.3 Energy Density Evolution

```
ρ_DE(z) = ρ_DE,0 exp[3∫₀^z (1+w(z'))dz'/(1+z')]
```

For constant w:
```
ρ_DE(z) = ρ_DE,0 (1+z)^(3(1+w))
```

---

## 6. Type Ia Supernovae Observations

### 6.1 Standard Candles

Type Ia supernovae (SNe Ia) are standardizable candles:

**Absolute Magnitude**: M ≈ -19.3 (after corrections)

**Phillips Relationship**:
```
M = M₀ + α×(Δm₁₅ - 1.1)
```

Where Δm₁₅ = magnitude decline in 15 days after peak.

### 6.2 Distance Modulus

```
μ = m - M = 5 log₁₀(d_L/10 pc)
```

Where:
- m = apparent magnitude
- M = absolute magnitude
- d_L = luminosity distance

### 6.3 Luminosity Distance

```
d_L(z) = (1+z) ∫₀^z dz'/H(z')
```

For flat ΛCDM:
```
d_L(z) = (c/H₀)(1+z) ∫₀^z dz'/√[Ω_m(1+z')³ + Ω_Λ]
```

### 6.4 Hubble Diagram

Plot of distance modulus vs redshift:

**Linear Regime** (z << 1):
```
μ ≈ 5 log₁₀(cz/H₀) + 25 + (1-q₀)z + ...
```

**Acceleration Discovery** (선행 연구, 선행 연구:
- High-z supernovae dimmer than expected
- Implies accelerating expansion
- Nobel Prize 2011

### 6.5 Pantheon+ Sample

Latest SNe Ia compilation:
- 1,701 light curves
- 1,550 supernovae
- Redshift range: 0.001 < z < 2.26
- Systematic uncertainties ~0.01 mag

---

## 7. Baryon Acoustic Oscillations

### 7.1 Sound Horizon

Comoving distance sound waves traveled before recombination:

```
r_s = ∫₀^(t_rec) c_s dt/a(t)
```

Where c_s = sound speed in baryon-photon plasma.

**Standard Ruler**: r_s ≈ 147 Mpc

### 7.2 Angular Diameter Distance

```
d_A(z) = ∫₀^z dz'/H(z')/(1+z)
```

**BAO Peak in Galaxy Correlation**:
```
θ_BAO(z) = r_s/d_A(z)
```

### 7.3 Spherically Averaged Distance

```
D_V(z) = [(1+z)²d_A²(z) cz/H(z)]^(1/3)
```

**Measurement**:
```
D_V(z)/r_s
```

### 7.4 Observational Data

**SDSS-III/BOSS**:
- z = 0.32: D_V/r_s = 8.25 ± 0.15
- z = 0.57: D_V/r_s = 13.77 ± 0.13

**eBOSS**:
- z = 0.698: D_M/r_s = 17.65 ± 0.30
- z = 1.48: D_M/r_s = 30.21 ± 0.79

### 7.5 Alcock-Paczynski Test

Anisotropic BAO measurements constrain:
- H(z) from line-of-sight clustering
- d_A(z) from transverse clustering

**Ratio**:
```
F_AP(z) = (1+z)d_A(z)H(z)/c
```

---

## 8. CMB Measurements

### 8.1 Cosmic Microwave Background

**Temperature**: T_CMB = 2.7255 K

**Blackbody Spectrum**: Perfect to 1 part in 10⁵

### 8.2 Angular Power Spectrum

Temperature fluctuations expanded in spherical harmonics:

```
ΔT(θ,φ)/T = Σ a_lm Y_lm(θ,φ)
```

**Power Spectrum**:
```
C_l = ⟨|a_lm|²⟩
```

### 8.3 Acoustic Peaks

**First Peak** (l ≈ 220):
- Determines spatial curvature
- Constrains Ω_total ≈ 1.000 ± 0.001

**Peak Spacing**:
- Determines baryon density Ω_b h²
- Determines matter density Ω_m h²

**ISW Effect** (Integrated Sachs-Wolfe):
- Large-scale power (l < 30)
- Sensitive to dark energy
- Enhanced by accelerating expansion

### 8.4 Sound Horizon at Recombination

```
r_s(z_*) = ∫_(z_*)^∞ c_s dz/H(z)
```

Where z_* ≈ 1090 (recombination redshift)

**Planck 2018**: r_s = 144.43 ± 0.26 Mpc

### 8.5 Angular Diameter Distance to Last Scattering

```
θ_* = r_s(z_*)/d_A(z_*)
```

**Constraint**:
```
θ_* = (1.04110 ± 0.00031)°
```

### 8.6 Planck Constraints

From Planck 2018 (TT,TE,EE+lowE):
- Ω_Λ = 0.6889 ± 0.0056
- Ω_m = 0.3111 ± 0.0056
- H₀ = 67.36 ± 0.54 km/s/Mpc
- σ₈ = 0.8111 ± 0.0060

---

## 9. Hubble Constant Tension

### 9.1 Early Universe Measurements

**Planck CMB** (2018):
```
H₀ = 67.36 ± 0.54 km/s/Mpc
```

**Assumption**: ΛCDM model extrapolated to z=0

### 9.2 Late Universe Measurements

**SH0ES Collaboration**:
```
H₀ = 73.04 ± 1.04 km/s/Mpc
```

**Methods**: Cepheid distance ladder + Type Ia supernovae

**Other Late-Time Probes**:
- H0LiCOW (lensing): H₀ = 73.3⁺¹·⁷₋₁.₈ km/s/Mpc
- Tip of Red Giant Branch: H₀ = 69.8 ± 1.9 km/s/Mpc

### 9.3 Tension Quantification

**Discrepancy**: ~5.0σ statistical significance

```
Δχ² ≈ 25 → ~5σ tension
```

### 9.4 Proposed Solutions

#### 9.4.1 Systematic Errors
- Cepheid calibration
- SNe Ia standardization
- CMB foregrounds

#### 9.4.2 New Physics
- Early dark energy
- Interacting dark energy
- Modified gravity
- Extra relativistic species
- Decaying dark matter

#### 9.4.3 Local Void
- Underdense region affecting local measurements
- Requires large void (~300 Mpc)
- Inconsistent with CMB

### 9.5 Impact on Dark Energy

Different H₀ values affect derived dark energy properties:

**Higher H₀**:
- Lower Ω_m
- Higher Ω_Λ or modified w(z)
- Affects age of universe

---

## 10. Dark Energy Surveys

### 10.1 Dark Energy Survey (DES)

**Coverage**: 5000 deg² of southern sky

**Observables**:
- Galaxy clustering
- Weak gravitational lensing
- Galaxy clusters
- Type Ia supernovae

**Key Results** (Y3):
- S₈ = σ₈√(Ω_m/0.3) = 0.776 ± 0.017
- Consistent with ΛCDM

### 10.2 DESI (Dark Energy Spectroscopic Instrument)

**Target**: 35 million galaxy redshifts

**Redshift Range**: 0 < z < 3.5

**Goals**:
- BAO measurements at multiple redshifts
- Redshift-space distortions
- w(z) constraints to ~1%

### 10.3 Euclid Space Telescope

**Launch**: 2023

**Observables**:
- Weak lensing over 15,000 deg²
- Galaxy clustering for 50 million galaxies
- BAO and growth rate

**Expected Precision**:
- w₀ to ~2%
- w_a to ~10%

### 10.4 LSST/Vera Rubin Observatory

**Survey**: Legacy Survey of Space and Time

**Coverage**: 18,000 deg² (full southern sky)

**Depth**: 10 billion galaxies

**Probes**:
- Weak lensing
- Large-scale structure
- Supernova time-domain astronomy
- Strong lensing time delays

### 10.5 Roman Space Telescope

**Formerly**: WFIRST

**Key Programs**:
- High-redshift SNe Ia survey
- Weak lensing survey
- BAO survey

**Expected**:
- 2,700 SNe Ia at z < 1.7
- σ(w₀) ~ 0.01, σ(w_a) ~ 0.1

---

## 11. Modified Gravity Theories

### 11.1 f(R) Gravity

**Action**:
```
S = ∫ d⁴x √(-g) f(R)/(16πG) + S_matter
```

Instead of Einstein-Hilbert f(R) = R

**Examples**:
- f(R) = R + αR² (Starobinsky)
- f(R) = R - μ⁴/R (MOND-like)

**Field Equations**:
```
f'(R)Rμν - ½f(R)gμν - ∇μ∇νf'(R) + gμν□f'(R) = 8πGTμν
```

### 11.2 Scalar-Tensor Theories

**Brans-Dicke Theory**:
```
S = ∫ d⁴x √(-g) [φR - ω/φ ∂μφ∂μφ]/(16πG)
```

Where φ is scalar field, ω is coupling parameter.

**Current Constraints**: ω > 40,000 (solar system tests)

### 11.3 DGP Model (Dvali-Gabadadze-Porrati)

**5-Dimensional Braneworld**:
- 4D brane embedded in 5D bulk
- Gravity leaks into bulk at large scales

**Modified Friedmann Equation**:
```
H² = 8πGρ/3 + H/r_c
```

Where r_c is crossover scale.

**Self-acceleration**: w_eff ≈ -1/3 at late times

### 11.4 Horndeski Theory

Most general scalar-tensor theory with second-order equations of motion:

**Lagrangian**:
```
L = Σᵢ₌₂⁵ Lᵢ(φ, ∂φ, gμν, ∂g)
```

**Includes**:
- Quintessence
- K-essence
- Covariant Galileon
- f(R) gravity

### 11.5 Tests of General Relativity

#### 11.5.1 Gravitational Wave Tests

**GW170817** (neutron star merger):
- Constrains speed of gravity: |c_g - c|/c < 10⁻¹⁵
- Rules out many modified gravity models

#### 11.5.2 Growth Rate

**Growth Factor**:
```
d(δρ_m/ρ_m)/dt = f(Ω_m)H δρ_m/ρ_m
```

**GR Prediction**: f ≈ Ω_m^0.55

**Parameterization**:
```
f(Ω_m) = Ω_m^γ
```

**Measurement**: γ = 0.55 ± 0.05 (consistent with GR)

---

## 12. Vacuum Energy

### 12.1 Quantum Field Theory Contribution

Each field mode contributes zero-point energy:

```
E_0 = ½ℏω
```

**Summing over all modes**:
```
ρ_vacuum = ∫ ½ℏω × (dk³/(2π)³)
```

**Divergence**: Integral diverges → need cutoff

### 12.2 Cutoff Scales

#### 12.2.1 Planck Scale Cutoff
```
k_max = M_Planck c/ℏ ≈ 10¹⁹ GeV
ρ_vacuum ≈ M_Planck⁴ ≈ 10¹¹³ J/m³
```

#### 12.2.2 Supersymmetry Breaking Scale
```
k_max ~ 1 TeV
ρ_vacuum ~ (1 TeV)⁴ ≈ 10⁴⁷ J/m³
```

Still 56 orders of magnitude too large!

### 12.3 Weinberg's No-Go Theorem

**Anthropic Bound**:
```
|ρ_Λ| < ρ_m(z_eq)
```

Where z_eq ~ 3400 (matter-radiation equality)

**Reasoning**: Larger |ρ_Λ| → no structure formation → no observers

### 12.4 Casimir Effect

**Observation**: Measurable vacuum energy between plates

**Force per Unit Area**:
```
F/A = -π²ℏc/(240d⁴)
```

Where d is plate separation.

**Interpretation**: Difference in zero-point energies, not absolute value

---

## 13. Future Cosmological Fate

### 13.1 ΛCDM Future

**Asymptotic Expansion**:
```
a(t) ∝ exp(H₀√Ω_Λ t)
```

**De Sitter Space**: Exponential expansion forever

**Observable Universe**: Shrinks in comoving coordinates
- Local Group bound (gravitationally)
- Other galaxies redshift beyond horizon
- CMB redshifts to undetectability

**Timeline**:
- 150 billion years: Distant galaxies exit horizon
- 1 trillion years: Local Group alone visible
- 10¹⁰⁰ years: Star formation ends (no gas)

### 13.2 Big Rip Scenario

If w < -1 (phantom energy):

**Rip Time**:
```
t_rip = t_0 + 2/(3H₀|1+w|)
```

**Example** (w = -1.5, H₀ = 70 km/s/Mpc):
```
t_rip ≈ 22 billion years from now
```

**Sequence**:
1. Galaxy clusters unbound (t - t_rip ~ 1 billion years)
2. Galaxies torn apart (t - t_rip ~ 60 million years)
3. Solar systems disrupted (t - t_rip ~ 3 months)
4. Stars explode (t - t_rip ~ 30 minutes)
5. Atoms ripped apart (t - t_rip ~ 10⁻¹⁹ s)

### 13.3 Big Crunch

If Ω_Λ = 0 and Ω_m > 1:
- Expansion stops
- Universe contracts
- Ends in singularity

**Not Favored** by current observations (Ω_total ≈ 1, Ω_Λ > 0)

### 13.4 Big Freeze

If w > -1 but expansion continues:
- Universe expands forever
- Temperature → 0
- Heat death

**ΛCDM is a Big Freeze scenario**

### 13.5 Vacuum Decay

Metastable vacuum could decay to true vacuum:

**Bubble Nucleation Rate**:
```
Γ/V ~ exp(-S_E/ℏ)
```

Where S_E is Euclidean action of bounce solution.

**Current Constraints**: Higgs vacuum appears metastable but lifetime >> age of universe

---

## 14. Implementation Guidelines

### 14.1 Data Analysis Requirements

**Computational Tools**:
- Boltzmann codes: CAMB, CLASS
- MCMC samplers: emcee, CosmoMC, Cobaya
- N-body simulations: Gadget, PKDGRAV
- Pipeline frameworks: Sacc, CCL

**Statistical Methods**:
- Bayesian parameter estimation
- Likelihood analysis
- Model selection (Bayes factors, AIC, BIC)
- Blinding protocols

### 14.2 Observational Data Formats

**Standard Catalogs**:
- FITS tables for astronomical data
- HDF5 for simulation outputs
- JSON for metadata and configurations

**Data Products**:
- Distance modulus vs redshift (SNe Ia)
- Angular power spectra C_l (CMB)
- Correlation functions ξ(r) (galaxies)
- Shear power spectra (weak lensing)

### 14.3 Theoretical Predictions

**Friedmann Solver**:
- Integrate H(z) to get distances
- Compute growth factor D(z)
- Calculate power spectrum P(k,z)

**Example Code Structure**:
```python
class DarkEnergyModel:
    def w(self, z):
        # Return equation of state at redshift z
        pass

    def H(self, z):
        # Hubble parameter
        return H0 * sqrt(Om*(1+z)**3 + Ode*self.rho_de(z)/rho_de0)

    def luminosity_distance(self, z):
        # Integrate to get d_L(z)
        pass
```

### 14.4 Model Comparison

**Information Criteria**:
```
AIC = -2 ln(L_max) + 2k
BIC = -2 ln(L_max) + k ln(n)
```

Where:
- L_max = maximum likelihood
- k = number of parameters
- n = number of data points

**Bayes Factor**:
```
B_12 = ∫ L(θ_1) π(θ_1) dθ_1 / ∫ L(θ_2) π(θ_2) dθ_2
```

**Interpretation**:
- |ln B_12| < 1: Inconclusive
- 1 < |ln B_12| < 2.5: Weak evidence
- 2.5 < |ln B_12| < 5: Moderate evidence
- |ln B_12| > 5: Strong evidence

### 14.5 Systematic Uncertainties

**Supernova Systematics**:
- Host galaxy mass correlation
- Color-luminosity relation
- Dust extinction models
- Malmquist bias

**Weak Lensing Systematics**:
- Shear calibration
- Photo-z uncertainties
- Intrinsic alignments
- Baryonic effects on P(k)

**Spectroscopic Systematics**:
- Fiber collisions
- Redshift-space distortions modeling
- Non-linear clustering

---

## 15. References

### 15.1 Foundational Papers

1. Einstein, A. (1917). "Kosmologische Betrachtungen zur allgemeinen Relativitätstheorie." *Sitzungsberichte der Königlich Preussischen Akademie der Wissenschaften*, 142-152.

2. 선행 연구. "Observational Evidence from Supernovae for an Accelerating Universe and a Cosmological Constant." *The Astronomical Journal*, 116(3), 1009.

3. 선행 연구. "Measurements of Ω and Λ from 42 High-Redshift Supernovae." *The Astrophysical Journal*, 517(2), 565.

4. Wetterich, C. (1988). "Cosmology and the fate of dilatation symmetry." *Nuclear Physics B*, 302(4), 668-696.

5. Ratra, B., & Peebles, P.J.E. (1988). "Cosmological consequences of a rolling homogeneous scalar field." *Physical Review D*, 37(12), 3406.

### 15.2 Dark Energy Probes

6. 선행 연구. "Detection of the Baryon Acoustic Peak in the Large-Scale Correlation Function of SDSS Luminous Red Galaxies." *The Astrophysical Journal*, 633(2), 560.

7. Planck Collaboration (2020). "Planck 2018 results. VI. Cosmological parameters." *Astronomy & Astrophysics*, 641, A6.

8. Abbott, T.M.C., et al. (DES Collaboration) (2022). "Dark Energy Survey Year 3 results: Cosmological constraints from galaxy clustering and weak lensing." *Physical Review D*, 105(2), 023520.

### 15.3 Hubble Tension

9. 선행 연구. "A Comprehensive Measurement of the Local Value of the Hubble Constant with 1 km s⁻¹ Mpc⁻¹ Uncertainty from the Hubble Space Telescope and the SH0ES Team." *The Astrophysical Journal Letters*, 934(1), L7.

10. Di 선행 연구. "In the realm of the Hubble tension—a review of solutions." *Classical and Quantum Gravity*, 38(15), 153001.

### 15.4 Modified Gravity

11. Dvali, G., Gabadadze, G., & Porrati, M. (2000). "4D gravity on a brane in 5D Minkowski space." *Physics Letters B*, 485(1-3), 208-214.

12. Abbott, B.P., et al. (LIGO/Virgo Collaboration) (2017). "GW170817: Observation of Gravitational Waves from a Binary Neutron Star Inspiral." *Physical Review Letters*, 119(16), 161101.

### 15.5 Reviews

13. Weinberg, S. (1989). "The cosmological constant problem." *Reviews of Modern Physics*, 61(1), 1.

14. Peebles, P.J.E., & Ratra, B. (2003). "The cosmological constant and dark energy." *Reviews of Modern Physics*, 75(2), 559.

15. 선행 연구. "Beyond the Cosmological Standard Model." *Physics Reports*, 568, 1-98.

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
