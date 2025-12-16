//! XYZ format exporter

use crate::types::Molecule;
use super::{OutputAdapter, OutputFormat, OutputResult, TrajectoryExporter, units, element_symbol};

/// XYZ format adapter
pub struct XyzAdapter {
    /// Coordinate precision (decimal places)
    precision: usize,
    /// Use extended XYZ format with properties
    extended: bool,
}

impl Default for XyzAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl XyzAdapter {
    /// Create new XYZ adapter
    pub fn new() -> Self {
        Self {
            precision: 6,
            extended: false,
        }
    }

    /// Set coordinate precision
    pub fn with_precision(mut self, precision: usize) -> Self {
        self.precision = precision;
        self
    }

    /// Enable extended XYZ format
    pub fn with_extended(mut self, extended: bool) -> Self {
        self.extended = extended;
        self
    }

    /// Export single frame
    fn export_frame(&self, molecule: &Molecule) -> String {
        let mut output = String::new();

        // Number of atoms
        output.push_str(&format!("{}\n", molecule.atoms.len()));

        // Comment line
        let comment = if self.extended {
            let lattice = "Lattice=\"10.0 0.0 0.0 0.0 10.0 0.0 0.0 0.0 10.0\"";
            let props = "Properties=species:S:1:pos:R:3";
            format!("{} {}", lattice, props)
        } else {
            molecule.name.clone().unwrap_or_else(|| "WIA Nano Structure".to_string())
        };
        output.push_str(&format!("{}\n", comment));

        // Atom coordinates
        for atom in &molecule.atoms {
            let symbol = element_symbol(&atom.element);
            // Convert from nm to Angstroms
            let x = atom.position.x * units::NM_TO_ANGSTROM;
            let y = atom.position.y * units::NM_TO_ANGSTROM;
            let z = atom.position.z * units::NM_TO_ANGSTROM;

            output.push_str(&format!(
                "{:<2} {:>12.*} {:>12.*} {:>12.*}\n",
                symbol,
                self.precision, x,
                self.precision, y,
                self.precision, z
            ));
        }

        output
    }
}

impl OutputAdapter for XyzAdapter {
    fn format(&self) -> OutputFormat {
        OutputFormat::Xyz
    }

    fn name(&self) -> &str {
        "XYZ"
    }

    fn export_molecule(&self, molecule: &Molecule) -> OutputResult<String> {
        Ok(self.export_frame(molecule))
    }
}

impl TrajectoryExporter for XyzAdapter {
    fn export_trajectory(&self, frames: &[Molecule]) -> OutputResult<String> {
        let mut output = String::new();

        for frame in frames {
            output.push_str(&self.export_frame(frame));
        }

        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Atom, Element};

    #[test]
    fn test_xyz_export() {
        let molecule = Molecule {
            id: "water".to_string(),
            name: Some("Water".to_string()),
            formula: "H2O".to_string(),
            structure_type: None,
            atoms: vec![
                Atom::new(Element::O, 0.0, 0.0, 0.0),
                Atom::new(Element::H, 0.0957, 0.0, 0.0),
                Atom::new(Element::H, -0.024, 0.0927, 0.0),
            ],
            bonds: vec![],
            mass_da: None,
            charge: 0,
            smiles: None,
            inchi: None,
        };

        let adapter = XyzAdapter::new();
        let result = adapter.export_molecule(&molecule).unwrap();

        let lines: Vec<&str> = result.lines().collect();
        assert_eq!(lines[0], "3");
        assert!(lines[1].contains("Water"));
        assert!(lines[2].starts_with("O "));
    }
}
