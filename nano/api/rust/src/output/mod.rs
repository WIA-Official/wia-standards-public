//! Output Module - Ecosystem Integration
//!
//! Export WIA Nano data to external formats for visualization and simulation.
//!
//! ## Supported Formats
//!
//! - **PDB** - Protein Data Bank (visualization)
//! - **XYZ** - Cartesian coordinates (simple export)
//! - **MOL2** - Tripos SYBYL (connectivity + charges)
//! - **CIF** - Crystallographic Information File
//! - **LAMMPS** - Molecular dynamics simulation
//! - **GROMACS** - MD topology and coordinates
//!
//! ## Example
//!
//! ```rust,ignore
//! use wia_nano::output::{OutputManager, OutputFormat};
//! use wia_nano::types::Molecule;
//!
//! let molecule = Molecule::fullerene_c60();
//! let manager = OutputManager::new();
//!
//! let pdb = manager.export(OutputFormat::Pdb, &molecule)?;
//! std::fs::write("molecule.pdb", pdb)?;
//! ```

mod error;
mod types;
mod adapter;
mod manager;
mod pdb;
mod xyz;
mod mol2;
mod lammps;
mod gromacs;
mod cif;

pub use error::*;
pub use types::*;
pub use adapter::*;
pub use manager::*;
pub use pdb::*;
pub use xyz::*;
pub use mol2::*;
pub use lammps::*;
pub use gromacs::*;
pub use cif::*;
