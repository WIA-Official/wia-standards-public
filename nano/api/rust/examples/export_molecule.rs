//! Example: Export molecule to various file formats
//!
//! This example demonstrates how to use the WIA Nano output module
//! to export molecular structures to common visualization and simulation formats.

use wia_nano::output::{OutputManager, OutputFormat};
use wia_nano::types::{Molecule, Atom, Bond, BondType, Element};

fn main() {
    // Create a simple water molecule
    let water = Molecule {
        id: "water-001".to_string(),
        name: Some("Water".to_string()),
        formula: "H2O".to_string(),
        structure_type: None,
        atoms: vec![
            Atom::new(Element::O, 0.0, 0.0, 0.0),
            Atom::new(Element::H, 0.0957, 0.0, 0.0),
            Atom::new(Element::H, -0.024, 0.0927, 0.0),
        ],
        bonds: vec![
            Bond::single(0, 1),
            Bond::single(0, 2),
        ],
        mass_da: Some(18.015),
        charge: 0,
        smiles: Some("O".to_string()),
        inchi: None,
    };

    // Create output manager
    let manager = OutputManager::new();

    // Export to XYZ format
    println!("=== XYZ Format ===");
    let xyz = manager.export(OutputFormat::Xyz, &water).unwrap();
    println!("{}", xyz);

    // Export to PDB format
    println!("=== PDB Format ===");
    let pdb = manager.export(OutputFormat::Pdb, &water).unwrap();
    println!("{}", pdb);

    // Export to MOL2 format
    println!("=== MOL2 Format ===");
    let mol2 = manager.export(OutputFormat::Mol2, &water).unwrap();
    println!("{}", mol2);

    // List available formats
    println!("=== Available Formats ===");
    for format in manager.available_formats() {
        println!("- {}", format);
    }
}
