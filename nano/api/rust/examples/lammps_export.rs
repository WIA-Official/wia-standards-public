//! Example: Export molecule to LAMMPS data file format
//!
//! This example shows how to export molecular structures for use with
//! LAMMPS molecular dynamics simulation software.

use wia_nano::output::{LammpsDataAdapter, LammpsAtomStyle, OutputAdapter};
use wia_nano::types::{Molecule, Atom, Bond, Element};

fn main() {
    // Create a simple hydrocarbon chain (ethane)
    let ethane = Molecule {
        id: "ethane-001".to_string(),
        name: Some("Ethane".to_string()),
        formula: "C2H6".to_string(),
        structure_type: None,
        atoms: vec![
            // Carbon atoms
            Atom::new(Element::C, 0.0, 0.0, 0.0).with_charge(-0.18),
            Atom::new(Element::C, 0.154, 0.0, 0.0).with_charge(-0.18),
            // Hydrogen atoms
            Atom::new(Element::H, -0.035, 0.103, 0.0).with_charge(0.06),
            Atom::new(Element::H, -0.035, -0.051, 0.089).with_charge(0.06),
            Atom::new(Element::H, -0.035, -0.051, -0.089).with_charge(0.06),
            Atom::new(Element::H, 0.189, 0.103, 0.0).with_charge(0.06),
            Atom::new(Element::H, 0.189, -0.051, 0.089).with_charge(0.06),
            Atom::new(Element::H, 0.189, -0.051, -0.089).with_charge(0.06),
        ],
        bonds: vec![
            // C-C bond
            Bond::single(0, 1),
            // C-H bonds for first carbon
            Bond::single(0, 2),
            Bond::single(0, 3),
            Bond::single(0, 4),
            // C-H bonds for second carbon
            Bond::single(1, 5),
            Bond::single(1, 6),
            Bond::single(1, 7),
        ],
        mass_da: Some(30.07),
        charge: 0,
        smiles: Some("CC".to_string()),
        inchi: None,
    };

    // Create LAMMPS adapter with "full" atom style
    let adapter = LammpsDataAdapter::new()
        .with_atom_style(LammpsAtomStyle::Full)
        .with_box_padding(10.0);

    // Export to LAMMPS data format
    println!("=== LAMMPS Data File ===");
    let lammps_data = adapter.export_molecule(&ethane).unwrap();
    println!("{}", lammps_data);

    // You can also use the atomic style for simpler simulations
    println!("\n=== LAMMPS Atomic Style ===");
    let atomic_adapter = LammpsDataAdapter::new()
        .with_atom_style(LammpsAtomStyle::Atomic);
    let atomic_data = atomic_adapter.export_molecule(&ethane).unwrap();
    println!("{}", atomic_data);
}
