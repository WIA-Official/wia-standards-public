//! LAMMPS data file format exporter

use std::collections::HashMap;

use crate::types::{Molecule, BondType};
use super::{OutputAdapter, OutputFormat, OutputResult, LammpsAtomStyle, units, element_symbol, element_mass};

/// LAMMPS data file adapter
pub struct LammpsDataAdapter {
    /// Atom style
    atom_style: LammpsAtomStyle,
    /// Box padding (Angstroms)
    box_padding: f64,
}

impl Default for LammpsDataAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl LammpsDataAdapter {
    /// Create new LAMMPS data adapter
    pub fn new() -> Self {
        Self {
            atom_style: LammpsAtomStyle::Full,
            box_padding: 5.0,
        }
    }

    /// Set atom style
    pub fn with_atom_style(mut self, style: LammpsAtomStyle) -> Self {
        self.atom_style = style;
        self
    }

    /// Set box padding
    pub fn with_box_padding(mut self, padding: f64) -> Self {
        self.box_padding = padding;
        self
    }

    /// Get element type ID (atomic number)
    fn element_type_id(element: &crate::types::Element) -> u32 {
        element.atomic_number() as u32
    }

    /// Get bond type ID
    fn bond_type_id(bond_type: &BondType) -> u32 {
        match bond_type {
            BondType::Single => 1,
            BondType::Double => 2,
            BondType::Triple => 3,
            BondType::Aromatic => 4,
            BondType::Hydrogen => 5,
            BondType::Ionic => 6,
            BondType::VanDerWaals => 7,
            BondType::Metallic => 8,
        }
    }

    /// Calculate bounding box
    fn calculate_bounds(molecule: &Molecule) -> ([f64; 3], [f64; 3]) {
        let mut min = [f64::MAX; 3];
        let mut max = [f64::MIN; 3];

        for atom in &molecule.atoms {
            let coords = [
                atom.position.x * units::NM_TO_ANGSTROM,
                atom.position.y * units::NM_TO_ANGSTROM,
                atom.position.z * units::NM_TO_ANGSTROM,
            ];
            for i in 0..3 {
                min[i] = min[i].min(coords[i]);
                max[i] = max[i].max(coords[i]);
            }
        }

        // Handle empty molecule
        if molecule.atoms.is_empty() {
            min = [0.0; 3];
            max = [10.0; 3];
        }

        (min, max)
    }
}

impl OutputAdapter for LammpsDataAdapter {
    fn format(&self) -> OutputFormat {
        OutputFormat::LammpsData
    }

    fn name(&self) -> &str {
        "LAMMPS Data"
    }

    fn export_molecule(&self, molecule: &Molecule) -> OutputResult<String> {
        let mut output = String::new();

        let title = molecule.name.as_deref().unwrap_or("WIA Nano Structure");
        output.push_str(&format!("# {} - LAMMPS data file\n\n", title));

        // Count unique atom types and bond types
        let mut atom_types: HashMap<u32, u32> = HashMap::new();
        let mut bond_types: HashMap<u32, u32> = HashMap::new();

        for atom in &molecule.atoms {
            let type_id = Self::element_type_id(&atom.element);
            let next_id = atom_types.len() as u32 + 1;
            atom_types.entry(type_id).or_insert(next_id);
        }

        for bond in &molecule.bonds {
            let type_id = Self::bond_type_id(&bond.bond_type);
            let next_id = bond_types.len() as u32 + 1;
            bond_types.entry(type_id).or_insert(next_id);
        }

        // Header section
        output.push_str(&format!("{} atoms\n", molecule.atoms.len()));
        output.push_str(&format!("{} bonds\n", molecule.bonds.len()));
        output.push_str("0 angles\n");
        output.push_str("0 dihedrals\n");
        output.push_str("0 impropers\n");
        output.push_str("\n");

        output.push_str(&format!("{} atom types\n", atom_types.len().max(1)));
        if !molecule.bonds.is_empty() {
            output.push_str(&format!("{} bond types\n", bond_types.len()));
        }
        output.push_str("\n");

        // Box bounds
        let (min, max) = Self::calculate_bounds(molecule);
        output.push_str(&format!(
            "{:.6} {:.6} xlo xhi\n",
            min[0] - self.box_padding,
            max[0] + self.box_padding
        ));
        output.push_str(&format!(
            "{:.6} {:.6} ylo yhi\n",
            min[1] - self.box_padding,
            max[1] + self.box_padding
        ));
        output.push_str(&format!(
            "{:.6} {:.6} zlo zhi\n",
            min[2] - self.box_padding,
            max[2] + self.box_padding
        ));
        output.push_str("\n");

        // Masses section
        output.push_str("Masses\n\n");
        let mut type_masses: Vec<(u32, u32, f64)> = Vec::new();
        for (atomic_num, type_id) in &atom_types {
            // Find element with this atomic number
            for atom in &molecule.atoms {
                if Self::element_type_id(&atom.element) == *atomic_num {
                    type_masses.push((*type_id, *atomic_num, element_mass(&atom.element)));
                    break;
                }
            }
        }
        type_masses.sort_by_key(|(id, _, _)| *id);
        for (type_id, _, mass) in &type_masses {
            output.push_str(&format!("{} {:.4}\n", type_id, mass));
        }
        output.push_str("\n");

        // Atoms section
        output.push_str("Atoms # ");
        match self.atom_style {
            LammpsAtomStyle::Atomic => output.push_str("atomic\n\n"),
            LammpsAtomStyle::Molecular => output.push_str("molecular\n\n"),
            LammpsAtomStyle::Full => output.push_str("full\n\n"),
            LammpsAtomStyle::Charge => output.push_str("charge\n\n"),
        }

        for (i, atom) in molecule.atoms.iter().enumerate() {
            let atom_id = i + 1;
            let type_id = atom_types[&Self::element_type_id(&atom.element)];
            let x = atom.position.x * units::NM_TO_ANGSTROM;
            let y = atom.position.y * units::NM_TO_ANGSTROM;
            let z = atom.position.z * units::NM_TO_ANGSTROM;
            let charge = atom.charge;

            match self.atom_style {
                LammpsAtomStyle::Atomic => {
                    output.push_str(&format!(
                        "{} {} {:.6} {:.6} {:.6}\n",
                        atom_id, type_id, x, y, z
                    ));
                }
                LammpsAtomStyle::Molecular => {
                    output.push_str(&format!(
                        "{} 1 {} {:.6} {:.6} {:.6}\n",
                        atom_id, type_id, x, y, z
                    ));
                }
                LammpsAtomStyle::Full => {
                    output.push_str(&format!(
                        "{} 1 {} {:.6} {:.6} {:.6} {:.6}\n",
                        atom_id, type_id, charge, x, y, z
                    ));
                }
                LammpsAtomStyle::Charge => {
                    output.push_str(&format!(
                        "{} {} {:.6} {:.6} {:.6} {:.6}\n",
                        atom_id, type_id, charge, x, y, z
                    ));
                }
            }
        }

        // Bonds section
        if !molecule.bonds.is_empty() {
            output.push_str("\nBonds\n\n");
            for (i, bond) in molecule.bonds.iter().enumerate() {
                let bond_id = i + 1;
                let type_id = bond_types[&Self::bond_type_id(&bond.bond_type)];
                output.push_str(&format!(
                    "{} {} {} {}\n",
                    bond_id,
                    type_id,
                    bond.atom1_idx + 1,
                    bond.atom2_idx + 1
                ));
            }
        }

        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Atom, Bond, Element};

    #[test]
    fn test_lammps_export() {
        let molecule = Molecule {
            id: "methane".to_string(),
            name: Some("Methane".to_string()),
            formula: "CH4".to_string(),
            structure_type: None,
            atoms: vec![
                Atom::new(Element::C, 0.0, 0.0, 0.0).with_charge(-0.4),
                Atom::new(Element::H, 0.109, 0.0, 0.0).with_charge(0.1),
            ],
            bonds: vec![Bond::single(0, 1)],
            mass_da: None,
            charge: 0,
            smiles: None,
            inchi: None,
        };

        let adapter = LammpsDataAdapter::new();
        let result = adapter.export_molecule(&molecule).unwrap();

        assert!(result.contains("atoms"));
        assert!(result.contains("bonds"));
        assert!(result.contains("Masses"));
        assert!(result.contains("Atoms"));
        assert!(result.contains("Bonds"));
    }
}
