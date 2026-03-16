# WIA Nano Phase 4: Ecosystem Integration

## 1. Overview

Phase 4 defines the integration layer that connects WIA Nano data with external molecular visualization, simulation, and analysis tools. This enables seamless data exchange between WIA Nano systems and the broader scientific computing ecosystem.

### Goals

- Export WIA Nano molecular data to standard file formats
- Enable visualization in VMD, PyMOL, Chimera, and web viewers
- Provide input files for molecular dynamics simulations (LAMMPS, GROMACS)
- Support analysis workflows with standard tools

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Nano Data Layer                       │
│          (Molecule, Atom, Bond, Nanorobot, etc.)            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                     Output Manager                           │
│                   (OutputAdapter trait)                      │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────┬─────────┬─────────┬─────────┐
        ▼         ▼         ▼         ▼         ▼
   ┌────────┐┌────────┐┌────────┐┌────────┐┌────────┐
   │  PDB   ││  XYZ   ││ LAMMPS ││GROMACS ││  MOL2  │
   │Exporter││Exporter││Exporter││Exporter││Exporter│
   └────────┘└────────┘└────────┘└────────┘└────────┘
        │         │         │         │         │
        ▼         ▼         ▼         ▼         ▼
   [VMD/PyMOL] [Viewers] [LAMMPS]  [GROMACS] [ChemDraw]
```

---

## 2. Output Interface

### OutputAdapter Trait

```rust
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// Get the output format type
    fn format(&self) -> OutputFormat;

    /// Get adapter name
    fn name(&self) -> &str;

    /// Get file extension
    fn extension(&self) -> &str;

    /// Export molecule to string
    fn export_molecule(&self, molecule: &Molecule) -> Result<String, OutputError>;

    /// Export to file
    fn export_to_file(&self, molecule: &Molecule, path: &Path) -> Result<(), OutputError>;

    /// Check if format supports bonds
    fn supports_bonds(&self) -> bool;

    /// Check if format supports charges
    fn supports_charges(&self) -> bool;
}
```

### Output Formats

```rust
pub enum OutputFormat {
    /// PDB - Protein Data Bank
    Pdb,
    /// XYZ - Cartesian coordinates
    Xyz,
    /// MOL2 - Tripos SYBYL
    Mol2,
    /// CIF - Crystallographic Information
    Cif,
    /// LAMMPS - Molecular dynamics input
    LammpsData,
    /// GROMACS - MD topology
    GromacsTop,
    /// JSON - WIA native format
    Json,
}
```

---

## 3. PDB Exporter

### Format Specification

PDB files use fixed-column formatting (80 columns per line).

### ATOM Record Format

```
ATOM      1  C   MOL A   1      10.000  20.000  30.000  1.00  0.00           C
│         │  │   │   │   │      │       │       │       │     │             │
│         │  │   │   │   │      X       Y       Z       Occ   TempF         Element
│         │  │   │   │   Residue seq
│         │  │   │   Chain ID
│         │  │   Residue name
│         │  Atom name
│         Atom serial number
Record type
```

### Implementation

```rust
pub struct PdbExporter {
    pub title: String,
    pub author: String,
}

impl PdbExporter {
    /// Export molecule to PDB format
    /// Note: Coordinates are converted from nm to Angstroms (×10)
    pub fn export(&self, molecule: &Molecule) -> Result<String, OutputError>;
}
```

---

## 4. XYZ Exporter

### Format Specification

```
<number_of_atoms>
<comment>
<element> <x> <y> <z>
...
```

### Implementation

```rust
pub struct XyzExporter {
    pub comment: String,
}

impl XyzExporter {
    /// Export molecule to XYZ format
    /// Coordinates in Angstroms
    pub fn export(&self, molecule: &Molecule) -> Result<String, OutputError>;

    /// Export trajectory (multiple frames)
    pub fn export_trajectory(&self, frames: &[Molecule]) -> Result<String, OutputError>;
}
```

---

## 5. MOL2 Exporter

### Format Specification

```
@<TRIPOS>MOLECULE
molecule_name
num_atoms num_bonds num_subst num_feat num_sets
SMALL
NO_CHARGES

@<TRIPOS>ATOM
1 C1 10.0000 20.0000 30.0000 C.3 1 MOL 0.0000

@<TRIPOS>BOND
1 1 2 1
```

### Implementation

```rust
pub struct Mol2Exporter {
    pub mol_type: String,
    pub charge_type: String,
}

impl Mol2Exporter {
    /// Export molecule to MOL2 format
    /// Includes full connectivity and charge information
    pub fn export(&self, molecule: &Molecule) -> Result<String, OutputError>;
}
```

---

## 6. LAMMPS Data Exporter

### Format Specification

```
LAMMPS Description

N atoms
M bonds
K atom types

0.0 100.0 xlo xhi
0.0 100.0 ylo yhi
0.0 100.0 zlo zhi

Masses

1 12.0
2 14.0

Atoms

1 1 1 0.0 10.0 20.0 30.0

Bonds

1 1 1 2
```

### Implementation

```rust
pub struct LammpsDataExporter {
    pub description: String,
    pub box_size: (f64, f64, f64),
    pub atom_style: AtomStyle,
}

pub enum AtomStyle {
    Atomic,      // atom-ID atom-type x y z
    Molecular,   // atom-ID molecule-ID atom-type x y z
    Full,        // atom-ID molecule-ID atom-type q x y z
    Charge,      // atom-ID atom-type q x y z
}
```

---

## 7. GROMACS Topology Exporter

### Format Specification

```
; GROMACS Topology

#include "forcefield.itp"

[ moleculetype ]
; Name  nrexcl
MOL     3

[ atoms ]
;   nr  type  resnr residue  atom  cgnr  charge   mass
     1  opls_135    1    MOL    C1     1  0.0000  12.011

[ bonds ]
;  ai    aj  funct
    1     2      1
```

### Implementation

```rust
pub struct GromacsTopExporter {
    pub force_field: String,
    pub water_model: Option<String>,
}

impl GromacsTopExporter {
    /// Export molecule topology
    pub fn export_topology(&self, molecule: &Molecule) -> Result<String, OutputError>;

    /// Export coordinate file (.gro)
    pub fn export_coordinates(&self, molecule: &Molecule) -> Result<String, OutputError>;
}
```

---

## 8. Output Manager

### Unified Export Interface

```rust
pub struct OutputManager {
    adapters: HashMap<OutputFormat, Box<dyn OutputAdapter>>,
}

impl OutputManager {
    /// Create with default adapters
    pub fn new() -> Self;

    /// Register custom adapter
    pub fn register(&mut self, adapter: Box<dyn OutputAdapter>);

    /// Export to specific format
    pub fn export(&self, format: OutputFormat, molecule: &Molecule) -> Result<String, OutputError>;

    /// Export to file
    pub fn export_to_file(&self, format: OutputFormat, molecule: &Molecule, path: &Path) -> Result<(), OutputError>;

    /// Export to multiple formats
    pub fn export_all(&self, molecule: &Molecule, base_path: &Path) -> Vec<Result<PathBuf, OutputError>>;

    /// List available formats
    pub fn available_formats(&self) -> Vec<OutputFormat>;
}
```

---

## 9. Unit Conversions

### Position Units

| Source (WIA Nano) | Target | Conversion Factor |
|-------------------|--------|-------------------|
| nm (nanometer) | Å (Angstrom) | × 10.0 |
| pm (picometer) | Å (Angstrom) | × 0.01 |
| nm | nm | × 1.0 |

### Mass Units

| Source | Target | Conversion Factor |
|--------|--------|-------------------|
| Da (Dalton) | g/mol | × 1.0 |
| Da | kg | × 1.66054e-27 |

### Energy Units

| Source | Target | Conversion Factor |
|--------|--------|-------------------|
| kJ/mol | kcal/mol | × 0.239006 |
| eV | kJ/mol | × 96.485 |

---

## 10. Error Handling

```rust
#[derive(Debug, thiserror::Error)]
pub enum OutputError {
    #[error("Invalid molecule: {0}")]
    InvalidMolecule(String),

    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Format error: {0}")]
    FormatError(String),

    #[error("Missing required field: {0}")]
    MissingField(String),
}
```

---

## 11. Examples

### Export to PDB

```rust
use wia_nano::output::{PdbExporter, OutputAdapter};
use wia_nano::types::Molecule;

let molecule = Molecule::fullerene_c60();
let exporter = PdbExporter::new("C60 Fullerene", "WIA Nano");

let pdb_content = exporter.export_molecule(&molecule)?;
std::fs::write("fullerene.pdb", pdb_content)?;
```

### Export to Multiple Formats

```rust
use wia_nano::output::OutputManager;

let manager = OutputManager::new();
let molecule = Molecule::fullerene_c60();

// Export to all available formats
let results = manager.export_all(&molecule, Path::new("./output"));
```

### Export for LAMMPS Simulation

```rust
use wia_nano::output::{LammpsDataExporter, AtomStyle};

let exporter = LammpsDataExporter::new()
    .with_box_size(100.0, 100.0, 100.0)
    .with_atom_style(AtomStyle::Full);

let lammps_data = exporter.export_molecule(&molecule)?;
std::fs::write("system.data", lammps_data)?;
```

---

## 12. Visualization Integration

### VMD Loading

```tcl
# VMD script to load WIA Nano exported PDB
mol new fullerene.pdb type pdb
mol modstyle 0 0 CPK 1.0 0.3 12.0 12.0
```

### PyMOL Loading

```python
# PyMOL script
cmd.load("fullerene.pdb")
cmd.show("spheres")
cmd.color("green", "elem C")
```

### Web Visualization (NGL)

```javascript
// NGL Viewer integration
stage.loadFile("fullerene.pdb").then(function(o) {
    o.addRepresentation("ball+stick");
    stage.autoView();
});
```

---

## 13. Simulation Workflow

### LAMMPS Workflow

```
1. Export WIA Nano data → LAMMPS data file
2. Create LAMMPS input script
3. Run simulation
4. Import trajectory back to WIA Nano
```

### GROMACS Workflow

```
1. Export topology (.top) and coordinates (.gro)
2. Generate simulation box (editconf)
3. Add solvent (solvate)
4. Energy minimize (grompp + mdrun)
5. Run production MD
6. Analyze trajectory
```

---

## 14. Best Practices

### Data Validation

- Always validate molecule before export
- Check for missing required fields
- Verify coordinate ranges

### Unit Consistency

- Always convert to target format units
- Document unit conventions in output files
- Include conversion factors in comments

### Performance

- Use buffered I/O for large files
- Stream trajectory data for memory efficiency
- Cache atom type mappings

---

## 15. References

- [wwPDB File Format](https://www.wwpdb.org/documentation/file-format)
- [LAMMPS Documentation](https://docs.lammps.org/)
- [GROMACS Manual](https://manual.gromacs.org/)
- [VMD User Guide](https://www.ks.uiuc.edu/Research/vmd/current/ug/)

---

*WIA Nano Phase 4: Ecosystem Integration Specification v1.0.0*

**弘益人間** - Benefit All Humanity
