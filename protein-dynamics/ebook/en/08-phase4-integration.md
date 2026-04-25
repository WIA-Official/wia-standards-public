# Chapter 7: Phase 4 - Integration

## Connecting with Databases and Pipelines

**弘益人間 (Benefit All Humanity)**

---

## 7.1 Integration Architecture

### The Integration Challenge

Protein dynamics data must connect with multiple ecosystems:

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA-PROTEIN-DYNAMICS                     │
│                      Integration Hub                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │   PDB   │  │AlphaFold│  │ UniProt │  │ ChEMBL  │       │
│  │Structure│  │   DB    │  │Function │  │ Bioact. │       │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘       │
│       │            │            │            │             │
│       └────────────┴─────┬──────┴────────────┘             │
│                          │                                  │
│                 ┌────────┴────────┐                        │
│                 │  WIA Dynamics   │                        │
│                 │    Standard     │                        │
│                 └────────┬────────┘                        │
│                          │                                  │
│       ┌──────────────────┼──────────────────┐              │
│       │                  │                  │              │
│  ┌────┴────┐       ┌─────┴─────┐      ┌────┴────┐         │
│  │  Drug   │       │  Protein  │      │Research │         │
│  │Discovery│       │Engineering│      │ Tools   │         │
│  └─────────┘       └───────────┘      └─────────┘         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Integration Principles

1. **Identifier Mapping**: Seamless translation between ID systems
2. **Data Enrichment**: Combine dynamics with function and disease
3. **Format Conversion**: Support all major file formats
4. **Real-time Sync**: Keep data current with source databases
5. **Provenance Tracking**: Maintain data lineage

---

## 7.2 PDB Integration

### Fetching Structures

```python
from wia_pd.integration import PDBClient

pdb = PDBClient()

# Fetch structure with metadata
structure = pdb.fetch("1ATP", include_metadata=True)

print(f"Title: {structure.metadata.title}")
print(f"Resolution: {structure.metadata.resolution} Å")
print(f"Method: {structure.metadata.experimental_method}")

# Get all structures for a protein
structures = pdb.search(
    uniprot_id="P00533",
    resolution_max=2.5,
    experimental_method="X-RAY"
)

for s in structures:
    print(f"  {s.pdb_id}: {s.resolution}Å, {s.ligands}")
```

### Extracting Dynamics Information

```python
# Extract B-factors as flexibility proxy
bfactors = pdb.get_bfactors("1ATP", chain="A", atoms="CA")

# Get multiple conformations if available
conformers = pdb.get_conformers("1ATP")
print(f"Alternate conformations: {len(conformers)}")

# Fetch NMR ensemble
nmr_ensemble = pdb.fetch_nmr_ensemble("1D3Z")
print(f"NMR models: {len(nmr_ensemble.models)}")

# Convert to WIA format
wia_dynamics = pdb.to_wia_format(
    pdb_id="1ATP",
    include_bfactors=True,
    include_conformers=True
)
```

### PDB API Endpoints

```http
GET /integration/pdb/{pdb_id}
GET /integration/pdb/search?uniprot_id={uniprot_id}
GET /integration/pdb/{pdb_id}/bfactors
GET /integration/pdb/{pdb_id}/conformers
POST /integration/pdb/to-wia-format
```

---

## 7.3 AlphaFold DB Integration

### Fetching Predictions

```python
from wia_pd.integration import AlphaFoldClient

af = AlphaFoldClient()

# Fetch prediction
prediction = af.fetch("P00533")

print(f"pLDDT mean: {prediction.plddt_mean:.1f}")
print(f"Confident regions: {len(prediction.high_confidence_regions)}")
print(f"Disordered regions: {len(prediction.low_confidence_regions)}")

# Get PAE matrix
pae = af.get_pae("P00533")
print(f"PAE shape: {pae.shape}")

# Identify domain boundaries from PAE
domains = af.identify_domains("P00533", pae_threshold=5.0)
for domain in domains:
    print(f"  Domain: {domain.start}-{domain.end}")
```

### Confidence to Dynamics Mapping

```python
def plddt_to_flexibility(plddt):
    """
    Convert AlphaFold pLDDT to flexibility estimate.

    High pLDDT (>90) = Confident, likely structured
    Medium pLDDT (70-90) = Moderate confidence
    Low pLDDT (<70) = Uncertain, likely flexible/disordered
    """
    flexibility = []
    for score in plddt:
        if score > 90:
            flex = 0.5  # Low flexibility
        elif score > 70:
            flex = 1.0 + (90 - score) * 0.05  # Moderate
        else:
            flex = 2.0 + (70 - score) * 0.1  # High flexibility
        flexibility.append(flex)
    return flexibility

# Use in WIA format
wia_dynamics = {
    "dynamics_metrics": {
        "flexibility": {
            "source": "alphafold_plddt_derived",
            "rmsf_estimated": plddt_to_flexibility(prediction.plddt)
        }
    }
}
```

---

## 7.4 UniProt Integration

### Functional Annotations

```python
from wia_pd.integration import UniProtClient

uniprot = UniProtClient()

# Fetch protein entry
entry = uniprot.fetch("P00533")

print(f"Name: {entry.name}")
print(f"Function: {entry.function}")
print(f"Subcellular location: {entry.subcellular_location}")

# Get domains
for domain in entry.domains:
    print(f"  {domain.name}: {domain.start}-{domain.end}")

# Get active sites
for site in entry.active_sites:
    print(f"  Active site: {site.position} - {site.description}")

# Get variants
for variant in entry.variants:
    print(f"  {variant.original}{variant.position}{variant.mutation}: {variant.consequence}")
```

### Mapping Features to Dynamics

```python
def map_uniprot_to_dynamics(uniprot_entry, dynamics_profile):
    """
    Annotate dynamics with functional features.
    """
    annotations = []

    # Map domains
    for domain in uniprot_entry.domains:
        mean_flex = calculate_mean_flexibility(
            dynamics_profile['flexibility']['rmsf'],
            domain.start, domain.end
        )
        annotations.append({
            'type': 'domain',
            'name': domain.name,
            'region': [domain.start, domain.end],
            'mean_flexibility': mean_flex,
            'functional_dynamics': infer_function_from_dynamics(mean_flex)
        })

    # Map active sites to allosteric network
    active_residues = [site.position for site in uniprot_entry.active_sites]
    allosteric_connections = find_connections(
        dynamics_profile['allosteric_network'],
        active_residues
    )

    # Map disease variants
    for variant in uniprot_entry.variants:
        if variant.disease:
            dynamics_at_position = get_dynamics_at_position(
                dynamics_profile, variant.position
            )
            annotations.append({
                'type': 'disease_variant',
                'variant': f"{variant.original}{variant.position}{variant.mutation}",
                'disease': variant.disease,
                'dynamics_context': dynamics_at_position
            })

    return annotations
```

---

## 7.5 ChEMBL/BindingDB Integration

### Drug and Bioactivity Data

```python
from wia_pd.integration import ChEMBLClient, BindingDBClient

chembl = ChEMBLClient()

# Get compounds targeting a protein
compounds = chembl.get_compounds_for_target("P00533")

print(f"Found {len(compounds)} compounds")

for compound in compounds[:10]:
    print(f"  {compound.chembl_id}: IC50={compound.ic50_nM} nM")

# Get detailed bioactivity
bioactivity = chembl.get_bioactivity("CHEMBL203")
print(f"Assays: {len(bioactivity.assays)}")

# Get kinetics data if available
kinetics = chembl.get_binding_kinetics("CHEMBL203", target="P00533")
if kinetics:
    print(f"  kon: {kinetics.kon}")
    print(f"  koff: {kinetics.koff}")
    print(f"  Residence time: {kinetics.residence_time}")
```

### Enriching with Dynamics

```python
def enrich_compound_with_dynamics(compound, protein_dynamics):
    """
    Add dynamics context to compound data.
    """
    # Get binding site dynamics
    binding_residues = compound.binding_site_residues
    site_flexibility = calculate_site_flexibility(
        protein_dynamics, binding_residues
    )

    # Check for cryptic binding site
    is_cryptic = check_cryptic_site(
        protein_dynamics['conformational_ensemble'],
        binding_residues
    )

    # Predict binding mechanism from dynamics
    if protein_dynamics['ensemble']['states'][0]['population'] > 0.8:
        predicted_mechanism = "induced_fit"
    else:
        predicted_mechanism = "conformational_selection"

    return {
        'compound_id': compound.chembl_id,
        'binding_affinity': compound.ic50_nM,
        'dynamics_context': {
            'site_flexibility': site_flexibility,
            'is_cryptic_site': is_cryptic,
            'predicted_mechanism': predicted_mechanism,
            'accessible_in_states': get_accessible_states(
                protein_dynamics, binding_residues
            )
        }
    }
```

---

## 7.6 Format Converters

### Supported Formats

| Format | Read | Write | Use Case |
|--------|------|-------|----------|
| WIA JSON | ✓ | ✓ | Standard exchange |
| PDB | ✓ | ✓ | Structures |
| mmCIF | ✓ | ✓ | Modern PDB format |
| XTC/DCD | ✓ | - | MD trajectories |
| GROMACS GRO | ✓ | ✓ | System topology |
| AMBER prmtop | ✓ | - | AMBER topology |
| MOL2/SDF | ✓ | ✓ | Ligands |
| NMR-STAR | ✓ | ✓ | NMR data |

### Conversion API

```python
from wia_pd.converters import convert

# MD trajectory to WIA ensemble
wia_data = convert(
    source="trajectory.xtc",
    source_topology="system.gro",
    target_format="wia",
    options={
        "clustering": True,
        "n_clusters": 5,
        "analysis": ["rmsf", "pca", "correlation"]
    }
)

# WIA to PDB multi-model
convert(
    source=wia_data,
    target="ensemble.pdb",
    target_format="pdb",
    options={
        "include_populations": True,  # As occupancy
        "include_bfactors": True
    }
)

# NMR data to WIA
wia_nmr = convert(
    source="nmr_data.str",
    source_format="nmr-star",
    target_format="wia",
    options={
        "extract_S2": True,
        "extract_rex": True
    }
)
```

---

## 7.7 Drug Discovery Pipeline Integration

### Schrodinger Integration

```python
from wia_pd.pipelines import SchrodingerPipeline

# Initialize with license
pipeline = SchrodingerPipeline(
    host="schrodinger-server",
    license_file="/path/to/license"
)

# Run Glide docking with dynamics
results = pipeline.dock_with_dynamics(
    protein_id="P00533",
    ligands="compounds.sdf",
    dynamics_profile=wia_dynamics,
    options={
        "use_ensemble_docking": True,
        "n_poses_per_state": 3,
        "sp_precision": True
    }
)

# Run Desmond MD
md_results = pipeline.run_desmond(
    complex_structure="docked_pose.mae",
    duration_ns=100,
    analysis=["mmgbsa", "interactions"]
)

# Convert to WIA format
wia_output = pipeline.to_wia_format(md_results)
```

### OpenEye Integration

```python
from wia_pd.pipelines import OpenEyePipeline

pipeline = OpenEyePipeline()

# Shape-based screening with dynamics
hits = pipeline.shape_screen(
    query="reference_ligand.sdf",
    database="compound_library.oeb",
    protein_dynamics=wia_dynamics,
    options={
        "consider_flexibility": True,
        "ensemble_shapes": True
    }
)
```

### KNIME Workflow Integration

```python
from wia_pd.pipelines import KNIMEIntegration

knime = KNIMEIntegration(
    workspace="/path/to/knime-workspace"
)

# Execute workflow with WIA data
results = knime.execute(
    workflow="ProteinDynamicsAnalysis",
    inputs={
        "dynamics_profile": wia_dynamics,
        "compounds": compounds_df
    }
)
```

---

## 7.8 Cloud Platform Integration

### AWS Integration

```python
from wia_pd.cloud import AWSProvider

aws = AWSProvider(
    region="us-east-1",
    credentials="~/.aws/credentials"
)

# Store dynamics data in S3
aws.store(
    data=wia_dynamics,
    bucket="my-dynamics-data",
    key="P00533/dynamics_v1.json"
)

# Run simulation on EC2/Batch
job = aws.run_simulation(
    protocol="WIA-PD-ENS-001",
    input_structure="s3://bucket/structure.pdb",
    instance_type="p3.2xlarge",  # GPU instance
    spot=True
)

# Query with Athena
results = aws.query("""
    SELECT protein_id, dynamics_index
    FROM dynamics_catalog
    WHERE dynamics_index > 0.7
""")
```

### Google Cloud Integration

```python
from wia_pd.cloud import GCPProvider

gcp = GCPProvider(project="my-project")

# Store in BigQuery
gcp.store_bigquery(
    data=dynamics_summary,
    dataset="protein_dynamics",
    table="profiles"
)

# Run on Cloud Life Sciences
job = gcp.run_pipeline(
    pipeline="dynamics-generation",
    inputs={"sequence": "MVLSPAD..."},
    machine_type="n1-standard-8-nvidia-tesla-v100"
)
```

---

## 7.9 Identifier Mapping Service

### Cross-Reference Resolution

```python
from wia_pd.integration import IdentifierMapper

mapper = IdentifierMapper()

# Map between ID systems
mappings = mapper.map(
    source_id="P00533",
    source_type="uniprot",
    target_types=["pdb", "alphafold", "ensembl", "refseq"]
)

print(mappings)
# {
#   'pdb': ['1ATP', '1M17', '2GS2', ...],
#   'alphafold': 'AF-P00533-F1',
#   'ensembl': 'ENSP00000275493',
#   'refseq': 'NP_005219.2'
# }

# Batch mapping
batch_mappings = mapper.map_batch(
    source_ids=["P00533", "P04626", "P00519"],
    source_type="uniprot",
    target_types=["pdb"]
)
```

### WIA ID System

```python
# WIA assigns stable identifiers
wia_id = mapper.get_wia_id("P00533")
print(f"WIA ID: {wia_id}")  # WIA-PROT-00533-v1

# Versioned references
versions = mapper.get_versions("WIA-PROT-00533")
for v in versions:
    print(f"  {v.version}: {v.created} - {v.changes}")
```

---

## 7.10 Data Synchronization

### Keeping Data Current

```python
from wia_pd.sync import DataSynchronizer

sync = DataSynchronizer()

# Check for updates
updates = sync.check_updates(
    proteins=["P00533", "P04626"],
    sources=["pdb", "uniprot", "alphafold"]
)

for update in updates:
    print(f"{update.protein}: {update.source} updated on {update.date}")

# Apply updates
sync.apply_updates(updates, conflict_resolution="merge")

# Schedule automatic sync
sync.schedule(
    frequency="daily",
    sources=["pdb", "uniprot"],
    notify_on_changes=True
)
```

### Change Tracking

```json
{
  "sync_log": {
    "protein_id": "P00533",
    "last_sync": "2025-01-15T00:00:00Z",
    "changes": [
      {
        "source": "pdb",
        "type": "new_structure",
        "pdb_id": "8XYZ",
        "date": "2025-01-14",
        "impact": "new_ligand_complex"
      },
      {
        "source": "uniprot",
        "type": "variant_update",
        "details": "New disease association for T790M"
      }
    ]
  }
}
```

---

## Summary

The WIA-PROTEIN-DYNAMICS integration layer provides:
- Seamless connection to PDB, AlphaFold, UniProt, ChEMBL
- Bidirectional format conversion
- Drug discovery pipeline integration
- Cloud platform support
- Identifier mapping across databases
- Automatic data synchronization

---

**Next Chapter:** [Drug Discovery Applications](./09-drug-discovery.md)

弘益人間 - Benefit All Humanity
