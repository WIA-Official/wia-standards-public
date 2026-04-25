# WIA-SEMI-018 Photoresist Specification v1.0

## Advanced Photoresist Quality Standards

**Document Number**: WIA-SEMI-018-SPEC-002
**Version**: 1.0
**Effective Date**: January 1, 2025

---

## 1. Scope

This specification defines quality requirements for photoresist materials used in semiconductor lithography, covering DUV (ArF, KrF) and EUV photoresists for technology nodes from 180nm to 3nm and beyond.

---

## 2. General Requirements

### 2.1 Resist Types Covered
- **EUV Photoresists** (13.5 nm wavelength): 3-8nm nodes
- **ArF Immersion Photoresists** (193nm): 28-65nm nodes
- **KrF Photoresists** (248nm): 130-250nm nodes

### 2.2 Purity Requirements
- Total impurities: <100 ppm
- Metallic contaminants: <10 ppb each (Na, K, Fe, Cu, Ni, Cr)
- Particles: <10 particles/mL (>0.2 µm)
- Gel content: <1 ppm

---

## 3. EUV Photoresist Specifications

### 3.1 Performance Parameters

| Parameter | Specification | Test Method |
|-----------|--------------|-------------|
| Resolution | ≤8 nm half-pitch (High-NA EUV) | SEM lithography test |
| Sensitivity | 15-25 mJ/cm² | Dose-to-size measurement |
| LER (3σ) | <1.5 nm | CD-SEM or AFM |
| Contrast | >5.0 | Contrast curve analysis |
| Defect Density | <0.01 defects/cm² | Post-develop inspection |
| Outgassing | <10⁻¹⁰ Torr·L/s | RGA measurement |

### 3.2 Physical Properties
- Viscosity: ±3% of specification (at 23°C)
- Refractive Index (@ 13.5nm): As specified (typically 0.92-0.98)
- Absorbance: >5 µm⁻¹ (metal-oxide resists) or >2 µm⁻¹ (CAR)
- Film thickness range: 25-50nm (coatable at 1500-3000 rpm)

### 3.3 Storage and Handling
- Storage temperature: 4-8°C
- Shelf life: 6 months (unopened), 3 months (opened)
- Container: HDPE or fluoropolymer bottle
- Filtration: 0.02 µm point-of-use filter required

---

## 4. ArF Photoresist Specifications

### 4.1 Performance Parameters

| Parameter | Specification | Test Method |
|-----------|--------------|-------------|
| Resolution | 40-80 nm | Lithography test |
| Sensitivity | 25-35 mJ/cm² | Dose-to-size |
| LER (3σ) | <3.0 nm | CD-SEM |
| Contrast | >5.0 | Contrast curve |
| Etch Selectivity | >2:1 (vs. BARC) | Etch rate measurement |

### 4.2 Storage Requirements
- Temperature: 4-8°C
- Shelf life: 6 months
- Light protection: Amber bottles, yellow room lighting

---

## 5. Quality Control

### 5.1 Incoming Inspection
- Visual inspection: 100% of containers
- Particle count: Per batch
- Viscosity: Per batch
- Sensitivity verification: Per batch (monitor wafer)

### 5.2 Certificate of Analysis
Required data:
- Batch number and manufacturing date
- Viscosity (cP at 23°C)
- Particle count
- Sensitivity range
- Recommended process window
- Expiration date

---

## 6. Acceptance Criteria

**Accept**: All parameters within specification
**Conditional Accept**: Minor viscosity deviation (<5%), documented
**Reject**: Particle count exceeded, sensitivity out of range, or expired

---

© 2025 WIA | 弘益人間
