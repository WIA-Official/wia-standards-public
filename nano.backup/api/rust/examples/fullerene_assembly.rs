//! Example: Fullerene (C60) Molecular Assembly
//!
//! This example demonstrates using the WIA Nano SDK to assemble
//! a C60 Buckminsterfullerene molecule using a molecular assembler.

use wia_nano::prelude::*;
use wia_nano::systems::StandardAssembler;

#[tokio::main]
async fn main() -> NanoResult<()> {
    println!("=== WIA Nano SDK: Fullerene Assembly Example ===\n");

    // Create a molecular assembler
    let mut assembler = StandardAssembler::builder("asm-001")
        .with_environment(Environment::vacuum())
        .with_precision(AssemblyPrecision::Atomic)
        .with_energy(100_000_000.0) // 100 pJ
        .build();

    // Initialize the assembler
    assembler.initialize().await?;
    println!("✓ Assembler initialized");

    // Use built-in C60 fullerene molecule
    let c60 = Molecule::fullerene_c60();
    println!("✓ Target molecule: {} ({} atoms)",
             c60.name.as_deref().unwrap_or("C60"),
             c60.atoms.len());

    // Start assembly
    let operation = assembler.start_assembly(&c60).await?;
    println!("✓ Assembly started: {}", operation.id);

    // Place atoms (simplified - just a few for demo)
    let radius = 0.355; // nm
    for i in 0..10 {
        // Calculate position on sphere
        let phi = std::f64::consts::PI * (3.0 - (5.0_f64).sqrt()) * (i as f64);
        let y = 1.0 - 2.0 * (i as f64 / 60.0);
        let r = (1.0 - y * y).sqrt();

        let pos = Position3D::new(
            radius * r * phi.cos(),
            radius * y,
            radius * r * phi.sin(),
        );

        assembler.place_atom(Element::C, pos).await?;
    }

    // Check progress
    if let Some(progress) = assembler.assembly_progress() {
        println!("\nAssembly Progress:");
        println!("  Atoms placed: {}/{}", progress.atoms_placed, progress.atoms_total);
        println!("  Completion: {:.1}%", progress.completion_percentage());
        println!("  Energy consumed: {:.2} fJ", progress.energy_consumed);
    }

    // Get assembler status
    let message = assembler.to_message()?;
    println!("\nAssembler Status Message:");
    println!("{}", serde_json::to_string_pretty(&message)?);

    // Shutdown
    assembler.shutdown().await?;
    println!("\n✓ Assembly complete!");

    Ok(())
}
