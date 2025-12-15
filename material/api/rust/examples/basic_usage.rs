//! Basic usage example for WIA Material SDK
//!
//! This example demonstrates the fundamental operations of the SDK.

use wia_material::prelude::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Material SDK - Basic Usage ===\n");

    // Create a client with default configuration
    let client = WiaMaterial::new();
    println!("Created WiaMaterial client (version {})", wia_material::VERSION);

    // Build a material using the builder pattern
    let material = MaterialBuilder::new()
        .material_type(MaterialType::Superconductor)
        .name("YBCO High-Tc Superconductor")
        .formula("YBa2Cu3O7-x")
        .classification(vec![
            "cuprate".to_string(),
            "high_tc".to_string(),
            "type_ii".to_string(),
        ])
        .structure(Structure {
            crystal_system: Some(CrystalSystem::Orthorhombic),
            space_group: Some("Pmmm".to_string()),
            lattice_parameters: Some(LatticeParameters {
                a_angstrom: Some(3.82),
                b_angstrom: Some(3.89),
                c_angstrom: Some(11.68),
                alpha_degree: Some(90.0),
                beta_degree: Some(90.0),
                gamma_degree: Some(90.0),
            }),
        })
        .electrical_properties(ElectricalProperties {
            resistivity_ohm_m: Some(0.0),
            ..Default::default()
        })
        .measurement(Measurement {
            temperature_k: Some(77.0),
            pressure_pa: Some(101325.0),
            method: Some("four_probe".to_string()),
            instrument: Some("PPMS DynaCool".to_string()),
            ..Default::default()
        })
        .confidence(0.98)
        .notes("High-quality single crystal sample from MIT Materials Lab")
        .build()?;

    println!("\nBuilt material: {}", material.identity.name);
    println!("  Material ID: {}", material.material_id);
    println!("  Formula: {}", material.identity.formula);
    println!("  Type: {}", material.material_type);

    // Store the material
    let stored = client.create_material(material).await?;
    println!("\nStored material successfully!");

    // Retrieve it back
    let retrieved = client.get_material(&stored.material_id).await?;
    println!("Retrieved: {}", retrieved.identity.name);

    // Convert to JSON
    let json = client.to_json(&retrieved)?;
    println!("\nJSON representation:");
    println!("{}", json);

    // Get statistics
    let stats = client.get_statistics().await;
    println!("\nStatistics:");
    println!("  Total materials: {}", stats.total_count);
    println!("  Superconductors: {}", stats.superconductor_count);

    println!("\n=== Basic Usage Complete ===");
    println!("\n弘益人間 🤟🦀");

    Ok(())
}
