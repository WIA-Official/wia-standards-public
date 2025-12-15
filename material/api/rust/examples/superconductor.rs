//! Superconductor-specific example for WIA Material SDK
//!
//! This example demonstrates working with superconductor materials.

use wia_material::prelude::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Material SDK - Superconductor Example ===\n");

    let client = WiaMaterial::new();

    // Create YBCO (High-Tc cuprate superconductor)
    println!("Creating YBCO superconductor...");
    let ybco_props = SuperconductorProperties {
        critical_temperature_k: 93.0,
        critical_pressure_pa: Some(101325.0),
        critical_current_density_a_m2: Some(1e10),
        critical_magnetic_field_t: Some(100.0),
        meissner_effect: Some(true),
        superconductor_type: Some(wia_material::SuperconductorType::TypeII),
        coherence_length_nm: Some(1.5),
        penetration_depth_nm: Some(150.0),
    };

    let ybco = client
        .create_superconductor("YBCO", "YBa2Cu3O7-x", ybco_props)
        .await?;

    println!("  Name: {}", ybco.identity.name);
    println!("  ID: {}", ybco.material_id);
    println!("  Tc: 93 K (-180°C)");
    println!("  Type: Type II superconductor");

    // Create MgB2 (Conventional BCS superconductor)
    println!("\nCreating MgB2 superconductor...");
    let mgb2_props = SuperconductorProperties {
        critical_temperature_k: 39.0,
        critical_pressure_pa: Some(101325.0),
        critical_current_density_a_m2: Some(1e9),
        critical_magnetic_field_t: Some(16.0),
        meissner_effect: Some(true),
        superconductor_type: Some(wia_material::SuperconductorType::TypeII),
        coherence_length_nm: Some(5.0),
        penetration_depth_nm: Some(140.0),
    };

    let mgb2 = client
        .create_superconductor("MgB2", "MgB2", mgb2_props)
        .await?;

    println!("  Name: {}", mgb2.identity.name);
    println!("  Tc: 39 K (-234°C)");

    // Create hypothetical room-temperature superconductor
    println!("\nCreating hypothetical room-temperature superconductor...");
    let rts_props = SuperconductorProperties {
        critical_temperature_k: 300.0, // Room temperature!
        critical_pressure_pa: Some(101325.0), // Ambient pressure
        critical_current_density_a_m2: Some(1e11),
        critical_magnetic_field_t: Some(200.0),
        meissner_effect: Some(true),
        superconductor_type: Some(wia_material::SuperconductorType::TypeII),
        coherence_length_nm: Some(0.5),
        penetration_depth_nm: Some(100.0),
    };

    let rts = client
        .create_superconductor("Hypothetical RTS", "Unknown", rts_props)
        .await?;

    println!("  Name: {}", rts.identity.name);
    println!("  Tc: 300 K (27°C) - Room temperature!");
    println!("  Note: This is hypothetical - no such material exists yet");

    // List all superconductors
    println!("\n--- All Superconductors ---");
    let superconductors = client
        .get_materials_by_type(MaterialType::Superconductor)
        .await;

    for (i, sc) in superconductors.iter().enumerate() {
        println!("{}. {} ({})", i + 1, sc.identity.name, sc.identity.formula);
    }

    // Statistics
    let stats = client.get_statistics().await;
    println!("\nTotal superconductors stored: {}", stats.superconductor_count);

    // Compare critical temperatures
    println!("\n--- Critical Temperature Comparison ---");
    println!("MgB2:  39 K  (-234°C)  - BCS conventional");
    println!("YBCO:  93 K  (-180°C)  - High-Tc cuprate");
    println!("RTS:   300 K (27°C)    - Hypothetical");

    println!("\n=== Superconductor Example Complete ===");
    println!("\n弘益人間 🤟🦀");

    Ok(())
}
