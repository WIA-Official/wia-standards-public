# Chapter 4: Phase 1 - Data Format

## Standardizing Protein Dynamics Data

**弘益人間 (Benefit All Humanity)**

---

## 4.1 The Need for Standardization

### Current State: Fragmentation

Protein dynamics data currently exists in incompatible formats:

| Source | Format | Issues |
|--------|--------|--------|
| MD trajectories | XTC, DCD, TRR, NC | Binary, software-specific |
| NMR dynamics | Text, XML | No standard schema |
| Cryo-EM ensembles | Multi-model PDB | Limited metadata |
| Normal modes | Custom arrays | No standard representation |
| Flexibility metrics | CSV, JSON | Inconsistent fields |

This fragmentation creates barriers:
- Tools cannot interoperate
- Data cannot be compared across methods
- Databases cannot aggregate dynamics information
- Reproducibility is compromised

### The WIA Solution

A unified JSON schema that:
- Represents all types of dynamics data
- Is human-readable and machine-parseable
- Supports validation and quality checks
- Enables database integration
- Preserves method-specific details

---

## 4.2 Core Schema Structure

### Top-Level Organization

```json
{
  "$schema": "https://wia.live/schemas/protein-dynamics/v1.0.0",
  "protein_dynamics": {
    "protein_id": "UniProt_ID",
    "metadata": { ... },
    "static_structure": { ... },
    "conformational_ensemble": { ... },
    "dynamics_metrics": { ... },
    "allosteric_network": { ... },
    "functional_dynamics": { ... }
  }
}
```

### Metadata Block

```json
{
  "metadata": {
    "standard": "WIA-PROTEIN-DYNAMICS",
    "version": "1.0.0",
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z",
    "generator": {
      "software": "WIA-PD-Toolkit",
      "version": "1.2.0"
    },
    "source": {
      "type": "simulation",
      "method": "molecular_dynamics",
      "software": "GROMACS",
      "version": "2024.1"
    },
    "philosophy": "弘益人間"
  }
}
```

---

## 4.3 Static Structure Reference

### Linking to Experimental/Predicted Structures

```json
{
  "static_structure": {
    "pdb_ids": ["1ATP", "2GS2", "3POZ"],
    "alphafold_id": "AF-P00533-F1",
    "primary_reference": "1ATP",

    "alphafold_confidence": {
      "mean_plddt": 87.3,
      "plddt_per_residue": [92.1, 91.5, 88.2, ...],
      "pae_matrix_file": "pae_matrix.json",
      "high_confidence_regions": [
        {"start": 1, "end": 450, "mean_plddt": 91.2},
        {"start": 680, "end": 950, "mean_plddt": 89.8}
      ],
      "low_confidence_regions": [
        {"start": 451, "end": 520, "mean_plddt": 45.3, "interpretation": "likely_disordered"},
        {"start": 951, "end": 1000, "mean_plddt": 38.2, "interpretation": "flexible_tail"}
      ]
    },

    "experimental_data": {
      "resolution_angstrom": 2.1,
      "method": "X-ray",
      "r_factor": 0.185,
      "r_free": 0.221
    },

    "sequence": {
      "uniprot_id": "P00533",
      "length": 1210,
      "sequence": "MRPSGTAGAALLALLAALCPASRALEEKK...",
      "domains": [
        {"name": "Kinase", "start": 712, "end": 979},
        {"name": "C-terminal tail", "start": 980, "end": 1210}
      ]
    }
  }
}
```

---

## 4.4 Conformational Ensemble

### Complete Ensemble Representation

```json
{
  "conformational_ensemble": {
    "num_states": 5,
    "generation_method": {
      "type": "enhanced_sampling",
      "subtype": "metadynamics",
      "software": "GROMACS+PLUMED",
      "parameters": {
        "total_time_us": 10,
        "bias_factor": 15,
        "gaussian_height_kJ": 1.0
      }
    },

    "states": [
      {
        "state_id": "ground",
        "name": "Active kinase",
        "population": 0.65,
        "population_error": 0.03,
        "coordinates": {
          "pdb_file": "state_ground.pdb",
          "pdb_string": null,
          "format": "pdb"
        },
        "relative_energy": {
          "value": 0.0,
          "error": 0.0,
          "unit": "kcal/mol"
        },
        "rmsd_from_reference": {
          "reference": "1ATP",
          "value": 0.8,
          "unit": "angstrom",
          "atoms": "backbone"
        },
        "secondary_structure": {
          "helix_content": 0.35,
          "sheet_content": 0.20,
          "coil_content": 0.45
        },
        "key_features": [
          "DFG-in conformation",
          "αC-helix in",
          "Activation loop extended"
        ]
      },
      {
        "state_id": "excited_1",
        "name": "DFG-out inactive",
        "population": 0.25,
        "population_error": 0.02,
        "coordinates": {
          "pdb_file": "state_excited1.pdb"
        },
        "relative_energy": {
          "value": 0.8,
          "error": 0.2,
          "unit": "kcal/mol"
        },
        "rmsd_from_reference": {
          "reference": "1ATP",
          "value": 3.2,
          "unit": "angstrom",
          "atoms": "backbone"
        },
        "key_features": [
          "DFG-out conformation",
          "αC-helix rotated",
          "Type II inhibitor binding competent"
        ]
      }
    ],

    "free_energy_landscape": {
      "collective_variables": [
        {
          "name": "CV1",
          "type": "rmsd",
          "reference": "active_site_residues.pdb",
          "unit": "angstrom"
        },
        {
          "name": "CV2",
          "type": "distance",
          "atom1": "K745:CA",
          "atom2": "E762:CA",
          "unit": "angstrom"
        }
      ],
      "grid": {
        "cv1_min": 0.0,
        "cv1_max": 8.0,
        "cv1_bins": 80,
        "cv2_min": 5.0,
        "cv2_max": 25.0,
        "cv2_bins": 100
      },
      "surface_file": "fel_surface.npy",
      "minima": [
        {
          "id": "min_1",
          "cv_values": [0.5, 12.0],
          "free_energy": 0.0,
          "state": "ground"
        },
        {
          "id": "min_2",
          "cv_values": [3.5, 18.5],
          "free_energy": 0.8,
          "state": "excited_1"
        }
      ],
      "barriers": [
        {
          "from": "min_1",
          "to": "min_2",
          "transition_state_cv": [2.0, 15.0],
          "barrier_height": 4.2,
          "barrier_height_error": 0.5,
          "unit": "kcal/mol"
        }
      ]
    }
  }
}
```

---

## 4.5 Dynamics Metrics

### Multi-Timescale Characterization

```json
{
  "dynamics_metrics": {
    "timescales": {
      "ps_motions": {
        "description": "Bond vibrations, methyl rotations, loop fluctuations",
        "amplitude_angstrom": 0.5,
        "correlation_time_ps": 10,
        "methods": ["MD", "NMR_T1"]
      },
      "ns_motions": {
        "description": "Side chain rotations, loop movements, small domain motions",
        "amplitude_angstrom": 2.0,
        "correlation_time_ns": 5,
        "methods": ["MD", "NMR_relaxation"]
      },
      "us_ms_motions": {
        "description": "Large conformational changes, domain movements",
        "rate_per_second": 1000,
        "methods": ["NMR_CPMG", "FRET", "enhanced_sampling"]
      }
    },

    "flexibility": {
      "b_factors": {
        "source": "MD_derived",
        "values": [15.2, 14.8, 18.3, ...],
        "units": "angstrom_squared",
        "reference": "CA_atoms"
      },
      "rmsf": {
        "values": [0.8, 0.7, 1.2, ...],
        "units": "angstrom",
        "atoms": "CA",
        "time_window_ns": 100
      },
      "order_parameters_S2": {
        "method": "Lipari_Szabo",
        "values": [0.92, 0.90, 0.45, ...],
        "backbone_NH": true,
        "experimental_validation": {
          "source": "BMRB:12345",
          "correlation": 0.89
        }
      },
      "flexible_regions": [
        {
          "start": 145,
          "end": 165,
          "type": "activation_loop",
          "mean_rmsf": 3.2,
          "functional_role": "Regulates substrate access"
        },
        {
          "start": 980,
          "end": 1000,
          "type": "C_terminal_tail",
          "mean_rmsf": 5.5,
          "functional_role": "Autophosphorylation sites"
        }
      ]
    },

    "disorder": {
      "idr_regions": [
        {
          "start": 980,
          "end": 1050,
          "disorder_score": 0.85,
          "predictor": "IUPred2A",
          "function": "Regulatory tail",
          "binding_partners": ["Grb2", "Shc"],
          "phase_separation": false
        }
      ],
      "total_disorder_percent": 12.5,
      "disorder_to_order_transitions": [
        {
          "region": [980, 1000],
          "trigger": "phosphorylation",
          "partner": "Grb2_SH2"
        }
      ]
    },

    "dynamics_index": {
      "value": 0.72,
      "components": {
        "flexibility_score": 0.65,
        "timescale_diversity": 0.80,
        "allosteric_coupling": 0.75,
        "disorder_content": 0.68
      },
      "interpretation": "Moderately dynamic protein with significant regulatory flexibility"
    }
  }
}
```

---

## 4.6 Allosteric Network

### Communication Pathway Mapping

```json
{
  "allosteric_network": {
    "active_sites": [
      {
        "name": "ATP_binding",
        "residues": [695, 745, 762, 790, 791, 793, 800, 855]
      },
      {
        "name": "Substrate_binding",
        "residues": [837, 838, 844, 845]
      }
    ],

    "allosteric_sites": [
      {
        "name": "Juxtamembrane",
        "residues": [650, 653, 656, 660],
        "effect": "inhibitory",
        "mechanism": "Blocks αC-helix rotation"
      },
      {
        "name": "C_terminal_tail",
        "residues": [992, 998, 1016, 1045, 1068, 1086],
        "effect": "autoinhibitory",
        "mechanism": "Intramolecular inhibition until phosphorylation"
      }
    ],

    "communication_pathways": [
      {
        "id": "pathway_1",
        "from_site": "Juxtamembrane",
        "to_site": "ATP_binding",
        "from_residue": 656,
        "to_residue": 762,
        "pathway_residues": [656, 680, 695, 720, 745, 762],
        "correlation": 0.78,
        "mutual_information": 0.45,
        "mechanism": "Backbone hydrogen bond network",
        "response_time_ns": 50
      },
      {
        "id": "pathway_2",
        "from_site": "C_terminal_tail",
        "to_site": "Substrate_binding",
        "from_residue": 1068,
        "to_residue": 844,
        "pathway_residues": [1068, 950, 890, 870, 850, 844],
        "correlation": 0.65,
        "mechanism": "Electrostatic through solvent"
      }
    ],

    "community_structure": {
      "method": "Girvan-Newman",
      "num_communities": 4,
      "communities": [
        {
          "id": 1,
          "name": "N_lobe",
          "residues": [695, 700, 745, 762],
          "functional_role": "ATP binding and positioning"
        },
        {
          "id": 2,
          "name": "C_lobe",
          "residues": [800, 837, 855],
          "functional_role": "Catalysis and substrate binding"
        }
      ]
    },

    "cross_correlation_matrix": {
      "file": "correlation_matrix.npy",
      "format": "numpy",
      "size": [285, 285],
      "atoms": "CA",
      "threshold_for_edges": 0.5
    }
  }
}
```

---

## 4.7 Functional Dynamics

### Catalytic and Binding Dynamics

```json
{
  "functional_dynamics": {
    "catalytic_cycle": [
      {
        "step": "1_apo",
        "name": "Apo enzyme",
        "conformation": "open",
        "duration_us": 10,
        "rate_constant": 1e5,
        "activation_energy": 0.0,
        "key_motion": "Activation loop flexible"
      },
      {
        "step": "2_atp_bound",
        "name": "ATP binding",
        "conformation": "closed",
        "rate_constant": 1e7,
        "activation_energy": 2.5,
        "key_motion": "Cleft closure, Lys-Glu salt bridge forms"
      },
      {
        "step": "3_substrate_bound",
        "name": "Substrate binding",
        "conformation": "catalytic",
        "rate_constant": 5e6,
        "key_motion": "Activation loop orders"
      },
      {
        "step": "4_phosphotransfer",
        "name": "Chemical step",
        "conformation": "transition_state",
        "rate_constant": 500,
        "rate_limiting": true,
        "activation_energy": 15.0
      },
      {
        "step": "5_product_release",
        "name": "Product release",
        "conformation": "open",
        "rate_constant": 2e4,
        "key_motion": "Cleft opens, products diffuse"
      }
    ],

    "binding_dynamics": {
      "mechanism": "mixed",
      "conformational_selection_fraction": 0.7,
      "induced_fit_fraction": 0.3,
      "kon": {
        "value": 1.2e6,
        "unit": "M-1s-1",
        "atp": 5e6,
        "substrate": 1e6
      },
      "koff": {
        "value": 0.5,
        "unit": "s-1"
      },
      "binding_pathway": [
        {
          "step": 1,
          "description": "Initial encounter",
          "distance_angstrom": 15,
          "interactions": "electrostatic_steering"
        },
        {
          "step": 2,
          "description": "Conformational selection",
          "binding_competent_fraction": 0.3
        },
        {
          "step": 3,
          "description": "Induced fit",
          "rmsd_change": 1.5,
          "key_residues": [762, 790, 855]
        }
      ]
    }
  }
}
```

---

## 4.8 Drug Binding Dynamics Schema

### Complete Binding Characterization

```json
{
  "drug_binding_dynamics": {
    "complex_id": "EGFR_Erlotinib",
    "protein_id": "P00533",
    "ligand": {
      "name": "Erlotinib",
      "pubchem_cid": 176870,
      "chembl_id": "CHEMBL553",
      "smiles": "COc1cc2ncnc(Nc3cccc(c3)C#C)c2cc1OCCOC",
      "molecular_weight": 393.44
    },

    "binding_site": {
      "residues": [695, 719, 743, 745, 762, 790, 791, 793, 797, 855],
      "druggability_score": 0.92,
      "pocket_volume_angstrom3": 850,
      "hydrophobicity": 0.65,
      "hinge_region": [790, 791, 793],
      "selectivity_pocket": [762, 790]
    },

    "thermodynamics": {
      "delta_g": {
        "value": -10.2,
        "error": 0.3,
        "unit": "kcal/mol",
        "method": "FEP"
      },
      "delta_h": {
        "value": -12.5,
        "error": 0.5,
        "unit": "kcal/mol",
        "method": "ITC"
      },
      "t_delta_s": {
        "value": -2.3,
        "unit": "kcal/mol"
      },
      "kd": {
        "value": 0.5,
        "unit": "nM",
        "temperature_K": 298
      },
      "enthalpy_entropy_compensation": "enthalpy_driven"
    },

    "kinetics": {
      "kon": {
        "value": 2.5e6,
        "unit": "M-1s-1",
        "method": "SPR"
      },
      "koff": {
        "value": 1.3e-3,
        "unit": "s-1"
      },
      "residence_time": {
        "value": 770,
        "unit": "seconds"
      },
      "kinetic_selectivity_index": 0.85
    },

    "binding_pathway": {
      "mechanism": "conformational_selection",
      "binding_competent_population": 0.25,
      "intermediate_states": [
        {
          "name": "Encounter complex",
          "lifetime_ns": 50,
          "ligand_position": "Solvent exposed"
        },
        {
          "name": "Recognition complex",
          "lifetime_ns": 500,
          "key_interactions": ["K745-N1", "T790-N3"]
        }
      ],
      "rate_limiting_step": "Induced fit of αC-helix"
    },

    "conformational_changes": {
      "protein_rmsd_upon_binding": 1.8,
      "atoms": "backbone",
      "key_residue_movements": [
        {
          "residue": 766,
          "motion": "rotation",
          "magnitude_degrees": 15
        }
      ],
      "allosteric_effects": [
        {
          "affected_site": "dimerization_interface",
          "effect": "stabilization",
          "mechanism": "Rigidification of C-lobe"
        }
      ]
    }
  }
}
```

---

## 4.9 Validation and Quality Control

### Schema Validation

```python
import jsonschema
from jsonschema import validate

def validate_protein_dynamics(data, schema_version="1.0.0"):
    """
    Validate protein dynamics JSON against WIA schema.

    Returns:
    --------
    dict : Validation result with errors if any
    """
    # Load schema
    schema = load_wia_schema(schema_version)

    try:
        validate(instance=data, schema=schema)
        return {
            'valid': True,
            'errors': []
        }
    except jsonschema.ValidationError as e:
        return {
            'valid': False,
            'errors': [str(e)]
        }
```

### Quality Checks

```python
def quality_check_ensemble(ensemble_data):
    """
    Check quality of conformational ensemble data.
    """
    checks = []

    # Check populations sum to 1
    total_pop = sum(s['population'] for s in ensemble_data['states'])
    checks.append({
        'name': 'population_sum',
        'passed': abs(total_pop - 1.0) < 0.01,
        'value': total_pop
    })

    # Check energies are consistent with populations
    for state in ensemble_data['states']:
        expected_pop = calculate_boltzmann_population(
            state['relative_energy']['value'],
            ensemble_data['states']
        )
        checks.append({
            'name': f"boltzmann_consistency_{state['state_id']}",
            'passed': abs(expected_pop - state['population']) < 0.1,
            'expected': expected_pop,
            'actual': state['population']
        })

    return checks
```

---

## Summary

The WIA-PROTEIN-DYNAMICS data format provides:
- Unified representation for all dynamics data types
- Clear hierarchy from metadata to functional dynamics
- Rich annotation of conformational ensembles
- Complete characterization of binding dynamics
- Built-in validation and quality checks
- Interoperability across tools and databases

---

**Next Chapter:** [Phase 2: API Interface](./06-phase2-api-interface.md)

弘益人間 - Benefit All Humanity
