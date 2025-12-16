//! GROMACS format exporters (.gro coordinates)

use crate::types::Molecule;
use super::{OutputAdapter, OutputFormat, OutputResult, TrajectoryExporter, element_symbol, element_mass};

/// GROMACS coordinate (.gro) format adapter
pub struct GromacsGroAdapter {
    /// Coordinate precision
    precision: usize,
    /// Include velocities
    include_velocities: bool,
}

impl Default for GromacsGroAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl GromacsGroAdapter {
    /// Create new GROMACS .gro adapter
    pub fn new() -> Self {
        Self {
            precision: 4,
            include_velocities: false,
        }
    }

    /// Set coordinate precision
    pub fn with_precision(mut self, precision: usize) -> Self {
        self.precision = precision;
        self
    }

    /// Set whether to include velocities
    pub fn with_velocities(mut self, include: bool) -> Self {
        self.include_velocities = include;
        self
    }

    /// Export single frame
    fn export_frame(&self, molecule: &Molecule, title: &str) -> String {
        let mut output = String::new();

        // Title line
        output.push_str(&format!("{}\n", title));

        // Number of atoms
        output.push_str(&format!("{:>5}\n", molecule.atoms.len()));

        // Atom lines
        for (i, atom) in molecule.atoms.iter().enumerate() {
            let atom_num = (i + 1) % 100000;
            let residue_num = 1;
            let residue_name = "MOL";
            let sym = element_symbol(&atom.element);
            let atom_name = format!("{}{}", sym, i + 1);

            // GROMACS uses nm directly
            let x = atom.position.x;
            let y = atom.position.y;
            let z = atom.position.z;

            if self.include_velocities {
                output.push_str(&format!(
                    "{:>5}{:<5}{:>5}{:>5}{:>8.*}{:>8.*}{:>8.*}{:>8.*}{:>8.*}{:>8.*}\n",
                    residue_num,
                    residue_name,
                    atom_name,
                    atom_num,
                    self.precision, x,
                    self.precision, y,
                    self.precision, z,
                    self.precision + 1, 0.0,
                    self.precision + 1, 0.0,
                    self.precision + 1, 0.0,
                ));
            } else {
                output.push_str(&format!(
                    "{:>5}{:<5}{:>5}{:>5}{:>8.*}{:>8.*}{:>8.*}\n",
                    residue_num,
                    residue_name,
                    atom_name,
                    atom_num,
                    self.precision, x,
                    self.precision, y,
                    self.precision, z,
                ));
            }
        }

        // Box vectors (placeholder - cubic box)
        let box_size = self.calculate_box_size(molecule);
        output.push_str(&format!(
            "{:>10.5}{:>10.5}{:>10.5}\n",
            box_size, box_size, box_size
        ));

        output
    }

    /// Calculate appropriate box size
    fn calculate_box_size(&self, molecule: &Molecule) -> f64 {
        let mut max_coord = 0.0_f64;

        for atom in &molecule.atoms {
            max_coord = max_coord.max(atom.position.x.abs());
            max_coord = max_coord.max(atom.position.y.abs());
            max_coord = max_coord.max(atom.position.z.abs());
        }

        // Add padding
        (max_coord + 0.5) * 2.0
    }
}

impl OutputAdapter for GromacsGroAdapter {
    fn format(&self) -> OutputFormat {
        OutputFormat::GromacsGro
    }

    fn name(&self) -> &str {
        "GROMACS GRO"
    }

    fn export_molecule(&self, molecule: &Molecule) -> OutputResult<String> {
        let title = molecule.name.as_deref().unwrap_or("WIA Nano Structure");
        Ok(self.export_frame(molecule, title))
    }
}

impl TrajectoryExporter for GromacsGroAdapter {
    fn export_trajectory(&self, frames: &[Molecule]) -> OutputResult<String> {
        let mut output = String::new();

        for (i, frame) in frames.iter().enumerate() {
            let title = format!(
                "{}, t={:.3}",
                frame.name.as_deref().unwrap_or("WIA Nano Trajectory"),
                i as f64 * 0.001
            );
            output.push_str(&self.export_frame(frame, &title));
        }

        Ok(output)
    }
}

/// GROMACS topology (.top) format adapter
pub struct GromacsTopAdapter;

impl Default for GromacsTopAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl GromacsTopAdapter {
    /// Create new GROMACS topology adapter
    pub fn new() -> Self {
        Self
    }
}

impl OutputAdapter for GromacsTopAdapter {
    fn format(&self) -> OutputFormat {
        OutputFormat::GromacsTop
    }

    fn name(&self) -> &str {
        "GROMACS TOP"
    }

    fn export_molecule(&self, molecule: &Molecule) -> OutputResult<String> {
        let mut output = String::new();

        let mol_name = molecule.name.as_deref().unwrap_or("MOL");

        // Header
        output.push_str("; GROMACS topology file generated by WIA Nano\n");
        output.push_str("; \n\n");

        // Defaults section
        output.push_str("[ defaults ]\n");
        output.push_str("; nbfunc comb-rule gen-pairs fudgeLJ fudgeQQ\n");
        output.push_str("1        2         yes       0.5     0.8333\n\n");

        // Atom types
        output.push_str("[ atomtypes ]\n");
        output.push_str("; name  bond_type  mass     charge  ptype  sigma      epsilon\n");

        let mut seen_elements: std::collections::HashSet<String> = std::collections::HashSet::new();
        for atom in &molecule.atoms {
            let sym = element_symbol(&atom.element);
            if !seen_elements.contains(sym) {
                seen_elements.insert(sym.to_string());
                let mass = element_mass(&atom.element);
                output.push_str(&format!(
                    "{:<6} {:<10} {:>8.4} {:>8.4}  A      {:>10.6} {:>10.6}\n",
                    sym, sym, mass, 0.0, 0.3, 0.1
                ));
            }
        }
        output.push_str("\n");

        // Molecule type
        output.push_str("[ moleculetype ]\n");
        output.push_str("; name  nrexcl\n");
        output.push_str(&format!("{:<6} 3\n\n", mol_name));

        // Atoms section
        output.push_str("[ atoms ]\n");
        output.push_str(";  nr  type  resnr residue atom  cgnr  charge   mass\n");
        for (i, atom) in molecule.atoms.iter().enumerate() {
            let atom_num = i + 1;
            let sym = element_symbol(&atom.element);
            let atom_name = format!("{}{}", sym, atom_num);
            let mass = element_mass(&atom.element);

            output.push_str(&format!(
                "{:>5} {:>6} {:>5} {:>6} {:>5} {:>5} {:>8.4} {:>10.4}\n",
                atom_num, sym, 1, "MOL", atom_name, atom_num, atom.charge, mass
            ));
        }
        output.push_str("\n");

        // Bonds section
        if !molecule.bonds.is_empty() {
            output.push_str("[ bonds ]\n");
            output.push_str(";  ai  aj  func  b0        kb\n");
            for bond in &molecule.bonds {
                output.push_str(&format!(
                    "{:>5} {:>5} {:>5}\n",
                    bond.atom1_idx + 1,
                    bond.atom2_idx + 1,
                    1
                ));
            }
            output.push_str("\n");
        }

        // System section
        output.push_str("[ system ]\n");
        output.push_str(&format!("{}\n\n", mol_name));

        // Molecules section
        output.push_str("[ molecules ]\n");
        output.push_str(&format!("{} 1\n", mol_name));

        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Atom, Element};

    #[test]
    fn test_gromacs_gro_export() {
        let molecule = Molecule {
            id: "test".to_string(),
            name: Some("Test".to_string()),
            formula: "C".to_string(),
            structure_type: None,
            atoms: vec![
                Atom::new(Element::C, 0.0, 0.0, 0.0),
            ],
            bonds: vec![],
            mass_da: None,
            charge: 0,
            smiles: None,
            inchi: None,
        };

        let adapter = GromacsGroAdapter::new();
        let result = adapter.export_molecule(&molecule).unwrap();

        let lines: Vec<&str> = result.lines().collect();
        assert_eq!(lines[0], "Test");
        assert_eq!(lines[1].trim(), "1");
    }

    #[test]
    fn test_gromacs_top_export() {
        let molecule = Molecule {
            id: "ethane".to_string(),
            name: Some("Ethane".to_string()),
            formula: "C2H6".to_string(),
            structure_type: None,
            atoms: vec![
                Atom::new(Element::C, 0.0, 0.0, 0.0),
            ],
            bonds: vec![],
            mass_da: None,
            charge: 0,
            smiles: None,
            inchi: None,
        };

        let adapter = GromacsTopAdapter::new();
        let result = adapter.export_molecule(&molecule).unwrap();

        assert!(result.contains("[ defaults ]"));
        assert!(result.contains("[ atomtypes ]"));
        assert!(result.contains("[ moleculetype ]"));
        assert!(result.contains("[ atoms ]"));
    }
}
