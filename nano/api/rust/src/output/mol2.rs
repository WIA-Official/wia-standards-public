//! MOL2 (Tripos SYBYL) format exporter

use crate::types::{Molecule, BondType};
use super::{OutputAdapter, OutputFormat, OutputResult, units, element_symbol};

/// MOL2 format adapter
pub struct Mol2Adapter {
    /// Include charges in output
    include_charges: bool,
}

impl Default for Mol2Adapter {
    fn default() -> Self {
        Self::new()
    }
}

impl Mol2Adapter {
    /// Create new MOL2 adapter
    pub fn new() -> Self {
        Self {
            include_charges: true,
        }
    }

    /// Set whether to include charges
    pub fn with_charges(mut self, include: bool) -> Self {
        self.include_charges = include;
        self
    }

    /// Convert bond type to MOL2 bond type string
    fn bond_type_str(bond_type: &BondType) -> &'static str {
        match bond_type {
            BondType::Single => "1",
            BondType::Double => "2",
            BondType::Triple => "3",
            BondType::Aromatic => "ar",
            BondType::Hydrogen => "1",
            BondType::Ionic => "1",
            BondType::VanDerWaals => "nc",
            BondType::Metallic => "1",
        }
    }

    /// Get SYBYL atom type (simplified)
    fn sybyl_type(element: &crate::types::Element) -> String {
        format!("{}.3", element_symbol(element))
    }
}

impl OutputAdapter for Mol2Adapter {
    fn format(&self) -> OutputFormat {
        OutputFormat::Mol2
    }

    fn name(&self) -> &str {
        "MOL2"
    }

    fn export_molecule(&self, molecule: &Molecule) -> OutputResult<String> {
        let mut output = String::new();

        let mol_name = molecule.name.as_deref().unwrap_or("WIA_NANO");

        // MOLECULE record
        output.push_str("@<TRIPOS>MOLECULE\n");
        output.push_str(&format!("{}\n", mol_name));
        output.push_str(&format!(
            "{:>5}{:>6}{:>6}{:>6}{:>6}\n",
            molecule.atoms.len(),
            molecule.bonds.len(),
            0, // substructures
            0, // features
            0  // sets
        ));
        output.push_str("SMALL\n");
        if self.include_charges {
            output.push_str("USER_CHARGES\n");
        } else {
            output.push_str("NO_CHARGES\n");
        }
        output.push_str("\n");

        // ATOM record
        output.push_str("@<TRIPOS>ATOM\n");
        for (i, atom) in molecule.atoms.iter().enumerate() {
            let atom_id = i + 1;
            let sym = element_symbol(&atom.element);
            let atom_name = format!("{}{}", sym, atom_id);
            let sybyl_type = Self::sybyl_type(&atom.element);

            // Convert from nm to Angstroms
            let x = atom.position.x * units::NM_TO_ANGSTROM;
            let y = atom.position.y * units::NM_TO_ANGSTROM;
            let z = atom.position.z * units::NM_TO_ANGSTROM;

            output.push_str(&format!(
                "{:>7} {:<8} {:>10.4} {:>10.4} {:>10.4} {:<8} {:>4} {:<8} {:>10.4}\n",
                atom_id,
                atom_name,
                x, y, z,
                sybyl_type,
                1,      // substructure ID
                "MOL1", // substructure name
                atom.charge
            ));
        }

        // BOND record
        if !molecule.bonds.is_empty() {
            output.push_str("@<TRIPOS>BOND\n");
            for (i, bond) in molecule.bonds.iter().enumerate() {
                let bond_id = i + 1;
                let bond_type = Self::bond_type_str(&bond.bond_type);

                output.push_str(&format!(
                    "{:>6} {:>6} {:>6} {}\n",
                    bond_id,
                    bond.atom1_idx + 1,
                    bond.atom2_idx + 1,
                    bond_type
                ));
            }
        }

        // SUBSTRUCTURE record
        output.push_str("@<TRIPOS>SUBSTRUCTURE\n");
        output.push_str(&format!(
            "{:>6} {:<8} {:>6} {:<8} {} {:<8} {}\n",
            1, "MOL1", 1, "RESIDUE", 1, "A", "MOL1"
        ));

        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Atom, Bond, Element};

    #[test]
    fn test_mol2_export() {
        let molecule = Molecule {
            id: "ethane".to_string(),
            name: Some("Ethane".to_string()),
            formula: "C2H6".to_string(),
            structure_type: None,
            atoms: vec![
                Atom::new(Element::C, 0.0, 0.0, 0.0).with_charge(-0.3),
                Atom::new(Element::C, 0.154, 0.0, 0.0).with_charge(-0.3),
            ],
            bonds: vec![Bond::single(0, 1)],
            mass_da: None,
            charge: 0,
            smiles: None,
            inchi: None,
        };

        let adapter = Mol2Adapter::new();
        let result = adapter.export_molecule(&molecule).unwrap();

        assert!(result.contains("@<TRIPOS>MOLECULE"));
        assert!(result.contains("@<TRIPOS>ATOM"));
        assert!(result.contains("@<TRIPOS>BOND"));
        assert!(result.contains("Ethane"));
    }
}
