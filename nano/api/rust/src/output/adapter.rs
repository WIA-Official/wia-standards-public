//! Output adapter trait

use crate::types::Molecule;
use super::{OutputError, OutputFormat, OutputResult};
use std::path::Path;

/// Trait for output format adapters
pub trait OutputAdapter: Send + Sync {
    /// Get the output format
    fn format(&self) -> OutputFormat;

    /// Get adapter name
    fn name(&self) -> &str;

    /// Get file extension
    fn extension(&self) -> &str {
        self.format().extension()
    }

    /// Export molecule to string
    fn export_molecule(&self, molecule: &Molecule) -> OutputResult<String>;

    /// Export molecule to file
    fn export_to_file(&self, molecule: &Molecule, path: &Path) -> OutputResult<()> {
        let content = self.export_molecule(molecule)?;
        std::fs::write(path, content)?;
        Ok(())
    }

    /// Check if format supports bonds
    fn supports_bonds(&self) -> bool {
        self.format().supports_bonds()
    }

    /// Check if format supports charges
    fn supports_charges(&self) -> bool {
        self.format().supports_charges()
    }

    /// Validate molecule before export
    fn validate(&self, molecule: &Molecule) -> OutputResult<()> {
        if molecule.atoms.is_empty() {
            return Err(OutputError::InvalidMolecule("Molecule has no atoms".into()));
        }
        Ok(())
    }
}

/// Trait for trajectory export (multiple frames)
pub trait TrajectoryExporter: OutputAdapter {
    /// Export trajectory (multiple frames)
    fn export_trajectory(&self, frames: &[Molecule]) -> OutputResult<String>;

    /// Export trajectory to file
    fn export_trajectory_to_file(&self, frames: &[Molecule], path: &Path) -> OutputResult<()> {
        let content = self.export_trajectory(frames)?;
        std::fs::write(path, content)?;
        Ok(())
    }
}

/// Helper to get element symbol (extended)
pub fn element_symbol(element: &crate::types::Element) -> &'static str {
    use crate::types::Element;
    match element {
        Element::H => "H", Element::He => "He",
        Element::Li => "Li", Element::Be => "Be", Element::B => "B",
        Element::C => "C", Element::N => "N", Element::O => "O",
        Element::F => "F", Element::Ne => "Ne",
        Element::Na => "Na", Element::Mg => "Mg", Element::Al => "Al",
        Element::Si => "Si", Element::P => "P", Element::S => "S",
        Element::Cl => "Cl", Element::Ar => "Ar",
        Element::K => "K", Element::Ca => "Ca",
        Element::Fe => "Fe", Element::Co => "Co", Element::Ni => "Ni",
        Element::Cu => "Cu", Element::Zn => "Zn",
        Element::Br => "Br", Element::Kr => "Kr",
        Element::Ag => "Ag", Element::Au => "Au", Element::Pt => "Pt",
        Element::Hg => "Hg", Element::Pb => "Pb",
        _ => "X",
    }
}

/// Helper to get atomic mass
pub fn element_mass(element: &crate::types::Element) -> f64 {
    use crate::types::Element;
    match element {
        Element::H => 1.008, Element::He => 4.003,
        Element::Li => 6.941, Element::Be => 9.012, Element::B => 10.81,
        Element::C => 12.011, Element::N => 14.007, Element::O => 15.999,
        Element::F => 18.998, Element::Ne => 20.180,
        Element::Na => 22.990, Element::Mg => 24.305, Element::Al => 26.982,
        Element::Si => 28.086, Element::P => 30.974, Element::S => 32.065,
        Element::Cl => 35.453, Element::Ar => 39.948,
        Element::K => 39.098, Element::Ca => 40.078,
        Element::Fe => 55.845, Element::Co => 58.933, Element::Ni => 58.693,
        Element::Cu => 63.546, Element::Zn => 65.38,
        Element::Br => 79.904, Element::Kr => 83.798,
        Element::Ag => 107.868, Element::Au => 196.967, Element::Pt => 195.084,
        Element::Hg => 200.592, Element::Pb => 207.2,
        _ => 12.0, // Default to carbon mass
    }
}
