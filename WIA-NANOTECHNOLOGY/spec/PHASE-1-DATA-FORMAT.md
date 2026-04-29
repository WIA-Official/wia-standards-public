# WIA-NANOTECHNOLOGY: PHASE 1 - DATA FORMAT SPECIFICATION

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2026-01-12
**Philosophy**: 弘益人間 (Benefit All Humanity)

## 1. Overview

This specification defines the standardized data formats for representing nanomaterials, characterization results, synthesis parameters, and experimental data in the WIA-NANOTECHNOLOGY ecosystem.

## 2. Nanomaterial Data Format

### 2.1 Material Definition

```json
{
  "materialId": "string (UUID)",
  "type": "CNT | graphene | quantum_dot | nanoparticle | nanocomposite | nanofilm",
  "name": "string",
  "composition": {
    "elements": [
      {
        "symbol": "string (periodic table)",
        "atomicNumber": "number",
        "percentage": "number (0-100)"
      }
    ],
    "formula": "string (chemical formula)",
    "molecularWeight": "number (g/mol)"
  },
  "dimensions": {
    "length": {
      "value": "number",
      "unit": "nm | μm",
      "range": {
        "min": "number",
        "max": "number"
      }
    },
    "width": "object (same structure as length)",
    "height": "object (same structure as length)",
    "diameter": "object (optional, for particles)",
    "aspectRatio": "number (optional)"
  },
  "structure": {
    "type": "crystalline | amorphous | polycrystalline",
    "latticeParameters": {
      "a": "number (Angstrom)",
      "b": "number (Angstrom)",
      "c": "number (Angstrom)",
      "alpha": "number (degrees)",
      "beta": "number (degrees)",
      "gamma": "number (degrees)"
    },
    "spaceGroup": "string (optional)"
  },
  "properties": {
    "physical": {
      "density": "number (g/cm³)",
      "surfaceArea": "number (m²/g)",
      "porosity": "number (0-1)",
      "crystallinity": "number (0-1)"
    },
    "mechanical": {
      "youngsModulus": "number (GPa)",
      "tensileStrength": "number (MPa)",
      "hardness": "number (GPa)"
    },
    "electrical": {
      "conductivity": "number (S/m)",
      "bandgap": "number (eV)",
      "mobility": "number (cm²/V·s)"
    },
    "optical": {
      "absorptionPeaks": [
        {
          "wavelength": "number (nm)",
          "intensity": "number"
        }
      ],
      "emissionPeaks": [
        {
          "wavelength": "number (nm)",
          "quantumYield": "number (0-1)"
        }
      ],
      "refractiveIndex": "number"
    },
    "magnetic": {
      "susceptibility": "number",
      "coercivity": "number (Oe)",
      "saturationMagnetization": "number (emu/g)"
    },
    "thermal": {
      "meltingPoint": "number (K)",
      "thermalConductivity": "number (W/m·K)",
      "specificHeat": "number (J/g·K)"
    }
  },
  "metadata": {
    "createdAt": "ISO 8601 timestamp",
    "updatedAt": "ISO 8601 timestamp",
    "source": "string (lab, supplier, database)",
    "doi": "string (optional)",
    "references": ["string (URLs or DOIs)"]
  }
}
```

### 2.2 Nanoparticle-Specific Format

```json
{
  "nanoparticleType": "gold | silver | titanium_dioxide | quantum_dot | other",
  "coreShell": {
    "hasShell": "boolean",
    "coreComposition": "string",
    "shellComposition": "string (optional)",
    "shellThickness": "number (nm, optional)"
  },
  "surfaceModification": {
    "ligands": [
      {
        "name": "string",
        "coverage": "number (molecules/nm²)",
        "chainLength": "number (optional)"
      }
    ],
    "functionalGroups": ["string"]
  },
  "dispersibility": {
    "solvent": "string",
    "concentration": "number (mg/mL)",
    "zetaPotential": "number (mV)",
    "hydrodynamicDiameter": "number (nm)"
  }
}
```

## 3. Characterization Results Format

### 3.1 General Characterization

```json
{
  "characterizationId": "string (UUID)",
  "materialId": "string (references material)",
  "technique": "TEM | SEM | AFM | XRD | XPS | Raman | FTIR | DLS | BET | other",
  "timestamp": "ISO 8601 timestamp",
  "operator": "string",
  "instrument": {
    "model": "string",
    "manufacturer": "string",
    "serialNumber": "string"
  },
  "conditions": {
    "temperature": "number (K)",
    "pressure": "number (Pa)",
    "atmosphere": "vacuum | air | N2 | Ar | other"
  },
  "results": "object (technique-specific, see below)"
}
```

### 3.2 TEM/SEM Results

```json
{
  "results": {
    "images": [
      {
        "url": "string (data URL or file path)",
        "magnification": "number",
        "accelerationVoltage": "number (kV)",
        "resolution": "number (nm)",
        "annotations": [
          {
            "type": "particle | defect | region",
            "coordinates": {"x": "number", "y": "number"},
            "measurement": {
              "value": "number",
              "unit": "nm"
            }
          }
        ]
      }
    ],
    "particleSizeDistribution": {
      "mean": "number (nm)",
      "median": "number (nm)",
      "standardDeviation": "number (nm)",
      "histogram": [
        {
          "binCenter": "number (nm)",
          "count": "number"
        }
      ]
    }
  }
}
```

### 3.3 XRD Results

```json
{
  "results": {
    "pattern": [
      {
        "twoTheta": "number (degrees)",
        "intensity": "number (counts)"
      }
    ],
    "peaks": [
      {
        "twoTheta": "number (degrees)",
        "dSpacing": "number (Angstrom)",
        "intensity": "number",
        "fwhm": "number (degrees)",
        "millerIndices": {
          "h": "number",
          "k": "number",
          "l": "number"
        }
      }
    ],
    "crystalliteSize": "number (nm)",
    "latticeParameters": {
      "a": "number (Angstrom)",
      "b": "number (Angstrom)",
      "c": "number (Angstrom)"
    },
    "phases": [
      {
        "name": "string",
        "percentage": "number (0-100)"
      }
    ]
  }
}
```

### 3.4 AFM Results

```json
{
  "results": {
    "scanSize": {
      "width": "number (μm)",
      "height": "number (μm)"
    },
    "resolution": {
      "x": "number (pixels)",
      "y": "number (pixels)"
    },
    "topography": {
      "imageUrl": "string",
      "dataMatrix": "array[array[number]] (height in nm)"
    },
    "roughnessAnalysis": {
      "rms": "number (nm)",
      "ra": "number (nm)",
      "peakToPeak": "number (nm)"
    },
    "features": [
      {
        "type": "particle | grain | defect",
        "height": "number (nm)",
        "lateralSize": "number (nm)"
      }
    ]
  }
}
```

## 4. Synthesis Parameters Format

### 4.1 Synthesis Record

```json
{
  "synthesisId": "string (UUID)",
  "materialId": "string (target material)",
  "method": "chemical_vapor_deposition | physical_vapor_deposition | sol_gel | hydrothermal | electrochemical | ball_milling | molecular_beam_epitaxy | lithography | self_assembly | other",
  "timestamp": "ISO 8601 timestamp",
  "operator": "string",
  "precursors": [
    {
      "name": "string",
      "formula": "string",
      "purity": "number (0-1)",
      "amount": {
        "value": "number",
        "unit": "g | mL | mol"
      },
      "supplier": "string"
    }
  ],
  "solvents": [
    {
      "name": "string",
      "volume": "number (mL)",
      "purity": "string"
    }
  ],
  "processSteps": [
    {
      "stepNumber": "number",
      "description": "string",
      "duration": "number (seconds)",
      "temperature": "number (K)",
      "pressure": "number (Pa)",
      "parameters": "object (method-specific)"
    }
  ],
  "yield": {
    "mass": "number (g)",
    "percentage": "number (0-100)"
  },
  "quality": {
    "uniformity": "number (0-1)",
    "purity": "number (0-1)",
    "defectDensity": "number (defects/nm²)"
  }
}
```

### 4.2 CVD-Specific Parameters

```json
{
  "parameters": {
    "gasFlow": [
      {
        "gas": "string",
        "flowRate": "number (sccm)"
      }
    ],
    "chamberPressure": "number (Torr)",
    "temperature": "number (K)",
    "rfPower": "number (W, optional)",
    "substrate": {
      "material": "string",
      "pretreatment": "string"
    },
    "growthTime": "number (seconds)",
    "coolingRate": "number (K/min)"
  }
}
```

## 5. Simulation Data Format

```json
{
  "simulationId": "string (UUID)",
  "type": "molecular_dynamics | density_functional_theory | finite_element | monte_carlo",
  "software": {
    "name": "string",
    "version": "string"
  },
  "system": {
    "materialId": "string (optional)",
    "numberOfAtoms": "number",
    "dimensions": {
      "x": "number (nm)",
      "y": "number (nm)",
      "z": "number (nm)"
    },
    "periodicBoundaryConditions": "boolean"
  },
  "parameters": {
    "timeStep": "number (fs)",
    "totalTime": "number (ps)",
    "temperature": "number (K)",
    "ensemble": "NVE | NVT | NPT",
    "potential": "string"
  },
  "results": {
    "energy": {
      "potential": "number (eV)",
      "kinetic": "number (eV)",
      "total": "number (eV)"
    },
    "trajectory": "string (file path or URL)",
    "properties": "object (simulation-specific)"
  }
}
```

## 6. Safety and Toxicology Data Format

```json
{
  "safetyId": "string (UUID)",
  "materialId": "string",
  "hazardClassification": {
    "ghs": ["string (GHS codes)"],
    "nfpa": {
      "health": "number (0-4)",
      "flammability": "number (0-4)",
      "reactivity": "number (0-4)",
      "special": "string (optional)"
    }
  },
  "exposureLimits": {
    "pel": "number (mg/m³)",
    "rel": "number (mg/m³)",
    "tlv": "number (mg/m³)"
  },
  "toxicology": {
    "ld50": {
      "value": "number (mg/kg)",
      "route": "oral | dermal | inhalation",
      "species": "string"
    },
    "cytotoxicity": [
      {
        "cellLine": "string",
        "ic50": "number (μg/mL)",
        "assay": "string"
      }
    ],
    "ecotoxicity": [
      {
        "organism": "string",
        "lc50": "number (mg/L)",
        "duration": "number (hours)"
      }
    ]
  },
  "handling": {
    "ppe": ["string (required PPE)"],
    "storageConditions": "string",
    "disposalMethod": "string"
  }
}
```

## 7. Data Exchange Format

All data exchange must use JSON format with UTF-8 encoding. File extensions:
- `.nanomat.json` - Material definitions
- `.nanochar.json` - Characterization results
- `.nanosyn.json` - Synthesis records
- `.nanosim.json` - Simulation data
- `.nanosafe.json` - Safety data

## 8. Versioning and Compatibility

- Format version follows semantic versioning (MAJOR.MINOR.PATCH)
- All records must include a `formatVersion` field
- Backward compatibility maintained within major versions

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
