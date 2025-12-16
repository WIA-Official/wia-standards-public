//! CIF (Crystallographic Information File) format exporter

use crate::types::Molecule;
use super::{OutputAdapter, OutputFormat, OutputResult, units, element_symbol};

/// CIF format adapter
pub struct CifAdapter {
    /// Coordinate precision
    precision: usize,
}

impl Default for CifAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl CifAdapter {
    /// Create new CIF adapter
    pub fn new() -> Self {
        Self {
            precision: 5,
        }
    }

    /// Set coordinate precision
    pub fn with_precision(mut self, precision: usize) -> Self {
        self.precision = precision;
        self
    }

    /// Calculate bounding box for unit cell
    fn calculate_cell_params(molecule: &Molecule) -> (f64, f64, f64) {
        let mut max_x = 0.0_f64;
        let mut max_y = 0.0_f64;
        let mut max_z = 0.0_f64;

        for atom in &molecule.atoms {
            let x = atom.position.x.abs() * units::NM_TO_ANGSTROM;
            let y = atom.position.y.abs() * units::NM_TO_ANGSTROM;
            let z = atom.position.z.abs() * units::NM_TO_ANGSTROM;
            max_x = max_x.max(x);
            max_y = max_y.max(y);
            max_z = max_z.max(z);
        }

        // Add padding and ensure minimum size
        let a = (max_x * 2.0 + 5.0).max(10.0);
        let b = (max_y * 2.0 + 5.0).max(10.0);
        let c = (max_z * 2.0 + 5.0).max(10.0);

        (a, b, c)
    }
}

impl OutputAdapter for CifAdapter {
    fn format(&self) -> OutputFormat {
        OutputFormat::Cif
    }

    fn name(&self) -> &str {
        "CIF"
    }

    fn export_molecule(&self, molecule: &Molecule) -> OutputResult<String> {
        let mut output = String::new();

        let data_name = molecule.name
            .as_deref()
            .unwrap_or("WIA_NANO")
            .replace(' ', "_")
            .replace('-', "_");

        // Data block
        output.push_str(&format!("data_{}\n", data_name));
        output.push_str("\n");

        // Audit info
        output.push_str("_audit_creation_method         'WIA Nano Export'\n");
        output.push_str("\n");

        // Chemical info
        output.push_str(&format!("_chemical_formula_sum          '{}'\n", molecule.formula));
        if let Some(name) = &molecule.name {
            output.push_str(&format!("_chemical_name_common          '{}'\n", name));
        }
        output.push_str("\n");

        // Cell parameters
        let (a, b, c) = Self::calculate_cell_params(molecule);
        output.push_str(&format!("_cell_length_a                  {:.4}\n", a));
        output.push_str(&format!("_cell_length_b                  {:.4}\n", b));
        output.push_str(&format!("_cell_length_c                  {:.4}\n", c));
        output.push_str("_cell_angle_alpha               90.0000\n");
        output.push_str("_cell_angle_beta                90.0000\n");
        output.push_str("_cell_angle_gamma               90.0000\n");
        output.push_str("\n");

        // Symmetry
        output.push_str("_symmetry_space_group_name_H-M  'P 1'\n");
        output.push_str("_symmetry_Int_Tables_number     1\n");
        output.push_str("\n");

        // Symmetry operations
        output.push_str("loop_\n");
        output.push_str("_symmetry_equiv_pos_as_xyz\n");
        output.push_str("'x, y, z'\n");
        output.push_str("\n");

        // Atom sites
        output.push_str("loop_\n");
        output.push_str("_atom_site_label\n");
        output.push_str("_atom_site_type_symbol\n");
        output.push_str("_atom_site_fract_x\n");
        output.push_str("_atom_site_fract_y\n");
        output.push_str("_atom_site_fract_z\n");
        output.push_str("_atom_site_occupancy\n");

        for (i, atom) in molecule.atoms.iter().enumerate() {
            let sym = element_symbol(&atom.element);
            let label = format!("{}{}", sym, i + 1);

            // Convert to fractional coordinates
            let x_ang = atom.position.x * units::NM_TO_ANGSTROM;
            let y_ang = atom.position.y * units::NM_TO_ANGSTROM;
            let z_ang = atom.position.z * units::NM_TO_ANGSTROM;

            // Shift to positive and convert to fractional
            let frac_x = (x_ang + a / 2.0) / a;
            let frac_y = (y_ang + b / 2.0) / b;
            let frac_z = (z_ang + c / 2.0) / c;

            output.push_str(&format!(
                "{:<6} {:<4} {:>.*} {:>.*} {:>.*} 1.0000\n",
                label,
                sym,
                self.precision, frac_x,
                self.precision, frac_y,
                self.precision, frac_z
            ));
        }

        output.push_str("\n");

        // Bond information (if any)
        if !molecule.bonds.is_empty() {
            output.push_str("loop_\n");
            output.push_str("_geom_bond_atom_site_label_1\n");
            output.push_str("_geom_bond_atom_site_label_2\n");
            output.push_str("_geom_bond_distance\n");

            for bond in &molecule.bonds {
                let sym1 = element_symbol(&molecule.atoms[bond.atom1_idx].element);
                let sym2 = element_symbol(&molecule.atoms[bond.atom2_idx].element);
                let label1 = format!("{}{}", sym1, bond.atom1_idx + 1);
                let label2 = format!("{}{}", sym2, bond.atom2_idx + 1);

                // Calculate bond distance
                let pos1 = &molecule.atoms[bond.atom1_idx].position;
                let pos2 = &molecule.atoms[bond.atom2_idx].position;
                let dx = (pos2.x - pos1.x) * units::NM_TO_ANGSTROM;
                let dy = (pos2.y - pos1.y) * units::NM_TO_ANGSTROM;
                let dz = (pos2.z - pos1.z) * units::NM_TO_ANGSTROM;
                let distance = (dx * dx + dy * dy + dz * dz).sqrt();

                output.push_str(&format!(
                    "{:<6} {:<6} {:.4}\n",
                    label1, label2, distance
                ));
            }
        }

        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Atom, Element};

    #[test]
    fn test_cif_export() {
        let molecule = Molecule {
            id: "diamond".to_string(),
            name: Some("Diamond".to_string()),
            formula: "C".to_string(),
            structure_type: None,
            atoms: vec![
                Atom::new(Element::C, 0.0, 0.0, 0.0),
                Atom::new(Element::C, 0.154, 0.154, 0.154),
            ],
            bonds: vec![],
            mass_da: None,
            charge: 0,
            smiles: None,
            inchi: None,
        };

        let adapter = CifAdapter::new();
        let result = adapter.export_molecule(&molecule).unwrap();

        assert!(result.contains("data_Diamond"));
        assert!(result.contains("_cell_length_a"));
        assert!(result.contains("_atom_site_label"));
        assert!(result.contains("C1"));
        assert!(result.contains("C2"));
    }
}
