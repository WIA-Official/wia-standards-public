//! Output types and enumerations

use serde::{Deserialize, Serialize};

/// Output format types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputFormat {
    /// PDB - Protein Data Bank format
    Pdb,
    /// XYZ - Cartesian coordinate format
    Xyz,
    /// MOL2 - Tripos SYBYL format
    Mol2,
    /// CIF - Crystallographic Information File
    Cif,
    /// LAMMPS data file format
    LammpsData,
    /// GROMACS topology format
    GromacsTop,
    /// GROMACS coordinate format (.gro)
    GromacsGro,
    /// JSON - WIA native format
    Json,
}

impl OutputFormat {
    /// Get file extension for this format
    pub fn extension(&self) -> &'static str {
        match self {
            Self::Pdb => "pdb",
            Self::Xyz => "xyz",
            Self::Mol2 => "mol2",
            Self::Cif => "cif",
            Self::LammpsData => "data",
            Self::GromacsTop => "top",
            Self::GromacsGro => "gro",
            Self::Json => "json",
        }
    }

    /// Get MIME type for this format
    pub fn mime_type(&self) -> &'static str {
        match self {
            Self::Pdb => "chemical/x-pdb",
            Self::Xyz => "chemical/x-xyz",
            Self::Mol2 => "chemical/x-mol2",
            Self::Cif => "chemical/x-cif",
            Self::LammpsData => "text/plain",
            Self::GromacsTop => "text/plain",
            Self::GromacsGro => "text/plain",
            Self::Json => "application/json",
        }
    }

    /// Check if format supports bond information
    pub fn supports_bonds(&self) -> bool {
        matches!(
            self,
            Self::Pdb | Self::Mol2 | Self::LammpsData | Self::GromacsTop
        )
    }

    /// Check if format supports charges
    pub fn supports_charges(&self) -> bool {
        matches!(self, Self::Mol2 | Self::LammpsData | Self::GromacsTop)
    }
}

impl std::fmt::Display for OutputFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Pdb => write!(f, "PDB"),
            Self::Xyz => write!(f, "XYZ"),
            Self::Mol2 => write!(f, "MOL2"),
            Self::Cif => write!(f, "CIF"),
            Self::LammpsData => write!(f, "LAMMPS Data"),
            Self::GromacsTop => write!(f, "GROMACS Topology"),
            Self::GromacsGro => write!(f, "GROMACS Coordinates"),
            Self::Json => write!(f, "JSON"),
        }
    }
}

/// LAMMPS atom style
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LammpsAtomStyle {
    /// atom-ID atom-type x y z
    Atomic,
    /// atom-ID molecule-ID atom-type x y z
    Molecular,
    /// atom-ID molecule-ID atom-type q x y z
    Full,
    /// atom-ID atom-type q x y z
    Charge,
}

/// Unit conversion constants
pub mod units {
    /// Convert nanometers to Angstroms
    pub const NM_TO_ANGSTROM: f64 = 10.0;

    /// Convert picometers to Angstroms
    pub const PM_TO_ANGSTROM: f64 = 0.01;

    /// Convert kJ/mol to kcal/mol
    pub const KJ_TO_KCAL: f64 = 0.239006;

    /// Convert eV to kJ/mol
    pub const EV_TO_KJ_MOL: f64 = 96.485;
}
