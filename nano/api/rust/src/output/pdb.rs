//! PDB (Protein Data Bank) format exporter

use crate::types::Molecule;
use super::{OutputAdapter, OutputFormat, OutputResult, TrajectoryExporter, units, element_symbol};

/// PDB format adapter
pub struct PdbAdapter {
    /// Include CONECT records for bonds
    include_conect: bool,
    /// Include TER records
    include_ter: bool,
}

impl Default for PdbAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl PdbAdapter {
    /// Create new PDB adapter
    pub fn new() -> Self {
        Self {
            include_conect: true,
            include_ter: true,
        }
    }

    /// Set whether to include CONECT records
    pub fn with_conect(mut self, include: bool) -> Self {
        self.include_conect = include;
        self
    }

    /// Set whether to include TER records
    pub fn with_ter(mut self, include: bool) -> Self {
        self.include_ter = include;
        self
    }

    /// Format element symbol for PDB (right-justified, 2 chars)
    fn format_element(element: &crate::types::Element) -> String {
        let sym = element_symbol(element);
        if sym.len() == 1 {
            format!(" {}", sym)
        } else {
            sym.to_string()
        }
    }

    /// Format atom name for PDB (4 characters)
    fn format_atom_name(element: &crate::types::Element, index: usize) -> String {
        let sym = element_symbol(element);
        let num = (index % 9999) + 1;
        if sym.len() == 1 {
            format!(" {}{:<2}", sym, num)
        } else {
            format!("{}{:<2}", sym, num)
        }
    }

    /// Export single frame/model
    fn export_frame(&self, molecule: &Molecule, model_num: Option<usize>) -> String {
        let mut output = String::new();

        // MODEL record for multi-frame
        if let Some(num) = model_num {
            output.push_str(&format!("MODEL     {:>4}\n", num));
        }

        // ATOM records
        for (i, atom) in molecule.atoms.iter().enumerate() {
            let serial = (i + 1) % 100000;
            let atom_name = Self::format_atom_name(&atom.element, i);
            let element = Self::format_element(&atom.element);

            // Convert from nm to Angstroms
            let x = atom.position.x * units::NM_TO_ANGSTROM;
            let y = atom.position.y * units::NM_TO_ANGSTROM;
            let z = atom.position.z * units::NM_TO_ANGSTROM;

            // PDB ATOM record format
            output.push_str(&format!(
                "ATOM  {:>5} {:4} {:3} {:1}{:>4}    {:>8.3}{:>8.3}{:>8.3}{:>6.2}{:>6.2}          {:>2}\n",
                serial,
                atom_name,
                "MOL",  // residue name
                "A",    // chain ID
                1,      // residue number
                x, y, z,
                1.00,   // occupancy
                0.00,   // temperature factor
                element
            ));
        }

        // TER record
        if self.include_ter && !molecule.atoms.is_empty() {
            let serial = molecule.atoms.len() + 1;
            output.push_str(&format!(
                "TER   {:>5}      {:3} {:1}{:>4}\n",
                serial, "MOL", "A", 1
            ));
        }

        // CONECT records for bonds
        if self.include_conect {
            // Group bonds by atom
            let mut connections: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
            for bond in &molecule.bonds {
                connections.entry(bond.atom1_idx).or_default().push(bond.atom2_idx);
                connections.entry(bond.atom2_idx).or_default().push(bond.atom1_idx);
            }

            for (atom_idx, bonded) in connections {
                if !bonded.is_empty() {
                    let serial = atom_idx + 1;
                    let mut line = format!("CONECT{:>5}", serial);
                    for bonded_idx in bonded.iter().take(4) {
                        line.push_str(&format!("{:>5}", bonded_idx + 1));
                    }
                    line.push('\n');
                    output.push_str(&line);
                }
            }
        }

        // ENDMDL for multi-frame
        if model_num.is_some() {
            output.push_str("ENDMDL\n");
        }

        output
    }
}

impl OutputAdapter for PdbAdapter {
    fn format(&self) -> OutputFormat {
        OutputFormat::Pdb
    }

    fn name(&self) -> &str {
        "PDB"
    }

    fn export_molecule(&self, molecule: &Molecule) -> OutputResult<String> {
        let mut output = String::new();

        // Header
        let title = molecule.name.as_deref().unwrap_or("WIA Nano Structure");
        output.push_str("HEADER    WIA NANO EXPORT\n");
        output.push_str(&format!("TITLE     {}\n", title));
        output.push_str(&format!("COMPND    {}\n", molecule.formula));

        // Crystal information
        output.push_str("CRYST1    1.000    1.000    1.000  90.00  90.00  90.00 P 1           1\n");

        // Export structure
        output.push_str(&self.export_frame(molecule, None));

        // End record
        output.push_str("END\n");

        Ok(output)
    }
}

impl TrajectoryExporter for PdbAdapter {
    fn export_trajectory(&self, frames: &[Molecule]) -> OutputResult<String> {
        let mut output = String::new();

        output.push_str("HEADER    WIA NANO TRAJECTORY\n");

        for (i, frame) in frames.iter().enumerate() {
            output.push_str(&self.export_frame(frame, Some(i + 1)));
        }

        output.push_str("END\n");

        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Atom, Element, Position3D};

    #[test]
    fn test_pdb_export() {
        let molecule = Molecule {
            id: "test".to_string(),
            name: Some("Test".to_string()),
            formula: "C2".to_string(),
            structure_type: None,
            atoms: vec![
                Atom::new(Element::C, 0.0, 0.0, 0.0),
                Atom::new(Element::C, 0.154, 0.0, 0.0),
            ],
            bonds: vec![],
            mass_da: None,
            charge: 0,
            smiles: None,
            inchi: None,
        };

        let adapter = PdbAdapter::new();
        let result = adapter.export_molecule(&molecule).unwrap();

        assert!(result.contains("HEADER"));
        assert!(result.contains("ATOM"));
        assert!(result.contains("END"));
    }
}
