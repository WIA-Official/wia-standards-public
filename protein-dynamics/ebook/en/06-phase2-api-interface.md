# Chapter 5: Phase 2 - API Interface

## RESTful APIs for Protein Dynamics

**弘益人間 (Benefit All Humanity)**

---

## 5.1 API Design Principles

### RESTful Architecture

The WIA-PROTEIN-DYNAMICS API follows REST principles:

- **Resource-oriented**: URLs identify resources (proteins, ensembles, simulations)
- **Stateless**: Each request contains all needed information
- **Standard methods**: GET, POST, PUT, DELETE with defined semantics
- **JSON responses**: Consistent data format throughout
- **HATEOAS**: Responses include links to related resources

### Base URL

```
Production: https://api.wia.live/protein-dynamics/v1
Staging:    https://staging-api.wia.live/protein-dynamics/v1
```

### Authentication

```http
Authorization: Bearer <api_key>
X-WIA-Client-ID: <client_id>
```

API keys are obtained from the WIA Developer Portal.

---

## 5.2 Core Endpoints

### Protein Structure and Dynamics

#### Get Protein Dynamics Profile

```http
GET /api/v1/protein/{protein_id}/dynamics
```

**Path Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| protein_id | string | UniProt ID, PDB ID, or WIA ID |

**Query Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| include | array | all | Sections to include: ensemble, metrics, allosteric |
| format | string | json | Response format: json, pdb, mmcif |
| detail | string | standard | Detail level: minimal, standard, full |

**Response:**
```json
{
  "status": "success",
  "data": {
    "protein_id": "P00533",
    "name": "Epidermal growth factor receptor",
    "dynamics_profile": {
      "conformational_ensemble": { ... },
      "dynamics_metrics": { ... },
      "allosteric_network": { ... }
    },
    "links": {
      "self": "/api/v1/protein/P00533/dynamics",
      "structure": "/api/v1/protein/P00533/structure",
      "ensemble": "/api/v1/protein/P00533/ensemble"
    }
  },
  "metadata": {
    "generated_at": "2025-01-15T10:30:00Z",
    "cache_ttl": 3600
  }
}
```

#### Get Static Structure

```http
GET /api/v1/protein/{protein_id}/structure
```

**Query Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| source | string | alphafold | Structure source: pdb, alphafold, modeled |
| pdb_id | string | - | Specific PDB ID if source=pdb |
| format | string | json | Output: json, pdb, mmcif, mol2 |

**Response:**
```json
{
  "status": "success",
  "data": {
    "protein_id": "P00533",
    "source": "alphafold",
    "alphafold_id": "AF-P00533-F1",
    "coordinates": "https://alphafold.ebi.ac.uk/files/AF-P00533-F1-model_v4.pdb",
    "confidence": {
      "mean_plddt": 87.3,
      "plddt_per_residue": [...]
    },
    "sequence_length": 1210
  }
}
```

---

## 5.3 Conformational Ensemble Endpoints

### Get Ensemble

```http
GET /api/v1/protein/{protein_id}/ensemble
```

**Query Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| method | string | any | Generation method: md, alphaflow, boltzmann |
| max_states | int | 10 | Maximum states to return |
| min_population | float | 0.01 | Minimum population threshold |
| include_coordinates | bool | false | Include atomic coordinates |

**Response:**
```json
{
  "status": "success",
  "data": {
    "protein_id": "P00533",
    "ensemble": {
      "num_states": 5,
      "generation_method": "metadynamics",
      "states": [
        {
          "state_id": "ground",
          "population": 0.65,
          "relative_energy_kcal": 0.0,
          "pdb_url": "/api/v1/protein/P00533/ensemble/state/ground/pdb",
          "features": {
            "dfg_conformation": "in",
            "alpha_c_helix": "in"
          }
        }
      ]
    }
  }
}
```

### Get Individual State

```http
GET /api/v1/protein/{protein_id}/ensemble/state/{state_id}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "state_id": "ground",
    "population": 0.65,
    "coordinates": {
      "format": "pdb",
      "content": "ATOM      1  N   MET A   1..."
    },
    "metrics": {
      "rmsd_to_reference": 0.8,
      "radius_of_gyration": 22.5,
      "secondary_structure": {
        "helix": 0.35,
        "sheet": 0.20,
        "coil": 0.45
      }
    }
  }
}
```

### Get Free Energy Landscape

```http
GET /api/v1/protein/{protein_id}/ensemble/landscape
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| cv1 | string | First collective variable |
| cv2 | string | Second collective variable |
| bins | int | Number of bins per dimension |

**Response:**
```json
{
  "status": "success",
  "data": {
    "collective_variables": {
      "cv1": {"name": "rmsd", "unit": "angstrom"},
      "cv2": {"name": "rg", "unit": "angstrom"}
    },
    "surface": {
      "format": "2d_array",
      "shape": [50, 50],
      "values": [[0.0, 0.5, ...], ...],
      "unit": "kcal/mol"
    },
    "minima": [
      {"cv1": 0.5, "cv2": 22.0, "energy": 0.0},
      {"cv1": 3.5, "cv2": 24.5, "energy": 0.8}
    ]
  }
}
```

---

## 5.4 Dynamics Prediction Endpoints

### Predict Dynamics Profile

```http
POST /api/v1/protein/dynamics/predict
```

**Request Body:**
```json
{
  "sequence": "MRPSGTAGAALLALLAALCPASRALEEKK...",
  "options": {
    "ensemble_method": "alphaflow",
    "num_samples": 100,
    "include_flexibility": true,
    "include_disorder": true,
    "include_allosteric": true
  }
}
```

**Response:**
```json
{
  "status": "success",
  "job_id": "JOB-12345",
  "estimated_time_seconds": 60,
  "callback_url": "/api/v1/jobs/JOB-12345",
  "links": {
    "status": "/api/v1/jobs/JOB-12345/status",
    "result": "/api/v1/jobs/JOB-12345/result"
  }
}
```

### Get Job Status

```http
GET /api/v1/jobs/{job_id}/status
```

**Response:**
```json
{
  "status": "success",
  "job": {
    "job_id": "JOB-12345",
    "state": "running",
    "progress": 45,
    "current_step": "Generating ensemble",
    "steps_completed": ["Sequence validation", "Structure prediction"],
    "estimated_remaining_seconds": 35
  }
}
```

### Get Job Result

```http
GET /api/v1/jobs/{job_id}/result
```

**Response:**
```json
{
  "status": "success",
  "job_id": "JOB-12345",
  "completed_at": "2025-01-15T10:31:00Z",
  "result": {
    "protein_dynamics": { ... }
  }
}
```

---

## 5.5 Allosteric Network Endpoints

### Get Allosteric Network

```http
GET /api/v1/protein/{protein_id}/allosteric
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "protein_id": "P00533",
    "allosteric_network": {
      "active_sites": [...],
      "allosteric_sites": [...],
      "communication_pathways": [...]
    }
  }
}
```

### Find Allosteric Pathways

```http
POST /api/v1/protein/{protein_id}/allosteric/pathways
```

**Request Body:**
```json
{
  "source_residues": [55, 57, 58],
  "target_residues": [145, 147, 166],
  "method": "correlation",
  "threshold": 0.5
}
```

**Response:**
```json
{
  "status": "success",
  "pathways": [
    {
      "pathway_id": 1,
      "source": 55,
      "target": 145,
      "residues": [55, 80, 95, 120, 145],
      "strength": 0.78,
      "length": 5
    }
  ]
}
```

---

## 5.6 Drug Binding Endpoints

### Predict Binding Dynamics

```http
POST /api/v1/binding/predict
```

**Request Body:**
```json
{
  "protein_id": "P00533",
  "ligand": {
    "format": "smiles",
    "structure": "COc1cc2ncnc(Nc3cccc(c3)C#C)c2cc1OCCOC"
  },
  "options": {
    "predict_kinetics": true,
    "predict_pathway": true,
    "ensemble_docking": true
  }
}
```

**Response:**
```json
{
  "status": "success",
  "job_id": "BIND-67890",
  "estimated_time_seconds": 300
}
```

### Get Binding Prediction Result

```http
GET /api/v1/binding/predict/{job_id}/result
```

**Response:**
```json
{
  "status": "success",
  "binding_prediction": {
    "thermodynamics": {
      "delta_g_kcal": -10.2,
      "kd_nM": 0.5
    },
    "kinetics": {
      "kon_M_s": 2.5e6,
      "koff_s": 1.3e-3,
      "residence_time_s": 770
    },
    "mechanism": "conformational_selection",
    "binding_site_residues": [695, 745, 762, 790, 855],
    "pose": {
      "pdb_url": "/api/v1/binding/predict/BIND-67890/pose.pdb"
    }
  }
}
```

### Simulate Binding Pathway

```http
POST /api/v1/binding/simulate
```

**Request Body:**
```json
{
  "protein_id": "P00533",
  "ligand_smiles": "COc1cc2ncnc(Nc3cccc(c3)C#C)c2cc1OCCOC",
  "simulation_params": {
    "method": "steered_md",
    "duration_ns": 100,
    "starting_distance_angstrom": 20
  }
}
```

---

## 5.7 Simulation Endpoints

### Submit MD Simulation

```http
POST /api/v1/simulation/submit
```

**Request Body:**
```json
{
  "protein_id": "P00533",
  "protocol": {
    "engine": "gromacs",
    "force_field": "amber14sb",
    "water_model": "tip3p",
    "temperature_K": 300,
    "pressure_bar": 1.0,
    "duration_ns": 100,
    "timestep_fs": 2,
    "save_interval_ps": 10
  },
  "enhanced_sampling": {
    "method": "metadynamics",
    "collective_variables": [
      {"type": "rmsd", "reference": "active_site"}
    ],
    "gaussian_height_kJ": 1.0,
    "deposit_interval_ps": 1.0
  },
  "resources": {
    "gpus": 1,
    "priority": "normal"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "simulation_id": "SIM-98765",
  "estimated_time_hours": 48,
  "queue_position": 5,
  "links": {
    "status": "/api/v1/simulation/SIM-98765/status",
    "logs": "/api/v1/simulation/SIM-98765/logs",
    "trajectory": "/api/v1/simulation/SIM-98765/trajectory"
  }
}
```

### Get Simulation Results

```http
GET /api/v1/simulation/{simulation_id}/results
```

**Response:**
```json
{
  "status": "success",
  "simulation_id": "SIM-98765",
  "completed_at": "2025-01-17T10:30:00Z",
  "results": {
    "trajectory_url": "https://storage.wia.live/trajectories/SIM-98765.xtc",
    "analysis": {
      "rmsd": {...},
      "rmsf": {...},
      "secondary_structure": {...}
    },
    "ensemble": {
      "num_clusters": 5,
      "cluster_populations": [0.45, 0.25, 0.15, 0.10, 0.05]
    },
    "convergence": {
      "converged": true,
      "block_error": 0.02
    }
  }
}
```

---

## 5.8 Integration Endpoints

### Fetch from External Database

```http
GET /api/v1/integration/fetch
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| source | string | Database: pdb, alphafold, uniprot, chembl |
| id | string | External identifier |

**Response:**
```json
{
  "status": "success",
  "source": "pdb",
  "external_id": "1ATP",
  "data": {
    "structure": {...},
    "metadata": {...}
  },
  "wia_dynamics_available": true,
  "wia_id": "WIA-P00533"
}
```

### Sync with UniProt

```http
POST /api/v1/integration/sync/uniprot
```

**Request Body:**
```json
{
  "uniprot_ids": ["P00533", "P04626", "P00519"],
  "include_features": ["domains", "variants", "modifications"]
}
```

---

## 5.9 TypeScript SDK

### Installation

```bash
npm install @wia/protein-dynamics
```

### Usage Examples

```typescript
import { WIAProteinDynamics } from '@wia/protein-dynamics';

// Initialize client
const client = new WIAProteinDynamics({
  apiKey: process.env.WIA_API_KEY,
  clientId: 'my-application'
});

// Get dynamics profile
async function getDynamicsProfile(proteinId: string) {
  const response = await client.protein.getDynamics(proteinId, {
    include: ['ensemble', 'metrics', 'allosteric'],
    detail: 'full'
  });

  console.log(`Protein: ${response.data.name}`);
  console.log(`States: ${response.data.dynamics_profile.conformational_ensemble.num_states}`);

  return response.data;
}

// Predict binding
async function predictBinding(proteinId: string, ligandSmiles: string) {
  // Submit prediction job
  const job = await client.binding.predict({
    proteinId,
    ligand: { format: 'smiles', structure: ligandSmiles },
    options: {
      predictKinetics: true,
      ensembleDocking: true
    }
  });

  // Poll for results
  const result = await client.jobs.waitForCompletion(job.jobId, {
    pollInterval: 5000,
    timeout: 600000
  });

  console.log(`Binding affinity: ${result.thermodynamics.deltaG} kcal/mol`);
  console.log(`Residence time: ${result.kinetics.residenceTime} s`);

  return result;
}

// Run simulation
async function runSimulation(proteinId: string) {
  const simulation = await client.simulation.submit({
    proteinId,
    protocol: {
      engine: 'gromacs',
      forceField: 'amber14sb',
      durationNs: 100
    }
  });

  // Stream progress
  for await (const update of client.simulation.streamProgress(simulation.simulationId)) {
    console.log(`Progress: ${update.progress}%`);
  }

  return await client.simulation.getResults(simulation.simulationId);
}
```

---

## 5.10 Rate Limiting and Quotas

### Rate Limits

| Tier | Requests/min | Concurrent Jobs | Storage |
|------|--------------|-----------------|---------|
| Free | 60 | 1 | 1 GB |
| Basic | 300 | 5 | 10 GB |
| Pro | 1000 | 20 | 100 GB |
| Enterprise | Unlimited | Custom | Custom |

### Rate Limit Headers

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1705312260
```

### Error Handling

```json
{
  "status": "error",
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please retry after 30 seconds.",
    "retry_after": 30
  }
}
```

---

## Summary

The WIA-PROTEIN-DYNAMICS API provides:
- RESTful endpoints for all dynamics operations
- Asynchronous job processing for computations
- TypeScript SDK for easy integration
- Comprehensive rate limiting and error handling
- Integration with external databases

---

**Next Chapter:** [Phase 3: Protocols](./07-phase3-protocols.md)

弘益人間 - Benefit All Humanity
