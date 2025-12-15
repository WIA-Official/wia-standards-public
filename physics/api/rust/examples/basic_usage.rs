//! Basic usage example for WIA Physics SDK
//!
//! This example demonstrates core features of the library.

use wia_physics::prelude::*;

fn main() -> PhysicsResult<()> {
    println!("=== WIA Physics SDK - Basic Usage ===\n");

    // =========================================================================
    // 1. Working with Measurements
    // =========================================================================
    println!("1. Working with Measurements");
    println!("{}", "-".repeat(40));

    // Create a measurement with uncertainty
    let higgs_mass = Measurement::new(125.25, 0.18, "GeV");
    println!(
        "Higgs boson mass: {:.2} ± {:.2} {}",
        higgs_mass.value, higgs_mass.uncertainty.total, higgs_mass.unit
    );

    // Create measurement with statistical and systematic uncertainties
    let top_mass = Measurement::with_uncertainty(172.69, 0.30, 0.50, "GeV");
    println!(
        "Top quark mass: {:.2} ± {:.2} (stat) ± {:.2} (syst) {}",
        top_mass.value,
        top_mass.uncertainty.statistical.unwrap(),
        top_mass.uncertainty.systematic.unwrap(),
        top_mass.unit
    );

    // Check if measurements are compatible
    let m1 = Measurement::new(100.0, 5.0, "GeV");
    let m2 = Measurement::new(108.0, 5.0, "GeV");
    println!(
        "Are measurements compatible? {}",
        if m1.is_compatible_with(&m2) { "Yes" } else { "No" }
    );
    println!();

    // =========================================================================
    // 2. Building Fusion Data
    // =========================================================================
    println!("2. Building Fusion Data");
    println!("{}", "-".repeat(40));

    let fusion_data = FusionDataBuilder::new()
        .experiment("ITER")
        .plasma_simple(150e6, "K", 1e20, "m^-3")
        .tokamak(5.3, 6.2, 2.0)
        .energy_balance(EnergyBalance {
            input_power: Some(Measurement::new(50.0, 1.0, "MW")),
            fusion_power: Some(Measurement::new(500.0, 10.0, "MW")),
            q_factor: Some(Measurement::new(10.0, 0.5, "")),
            neutron_power: None,
            alpha_heating_power: None,
            radiation_loss: None,
            stored_energy: None,
        })
        .quality(QualityFlag::Simulated)
        .build()?;

    println!("Experiment: {:?}", fusion_data.metadata.experiment);
    println!(
        "Plasma temperature: {:.2e} K",
        fusion_data.plasma.temperature.value
    );
    println!("Plasma density: {:.2e} m^-3", fusion_data.plasma.density.value);

    if let Some(ref config) = fusion_data.magnetic_configuration {
        println!("Confinement type: {:?}", config.confinement_type);
        if let Some(ref field) = config.toroidal_field {
            println!("Toroidal field: {:.1} T", field.value);
        }
    }

    if let Some(ref energy) = fusion_data.energy_balance {
        if let Some(ref q) = energy.q_factor {
            println!("Q-factor: {:.1}", q.value);
        }
    }
    println!();

    // =========================================================================
    // 3. Physics Calculations
    // =========================================================================
    println!("3. Physics Calculations");
    println!("{}", "-".repeat(40));

    // Fusion calculations
    let triple_product = FusionPhysics::triple_product(1e20, 15.0, 3.0);
    println!("Triple product: {:.2e} keV·s/m³", triple_product);
    println!(
        "Ignition possible? {}",
        if FusionPhysics::is_ignition_possible(triple_product) {
            "Yes!"
        } else {
            "Not yet"
        }
    );

    let q = FusionPhysics::q_factor(500.0, 50.0)?;
    println!("Q-factor (500 MW / 50 MW): {:.1}", q);

    // Particle physics calculations
    let significance = ParticlePhysics::significance(150.0, 25.0)?;
    println!(
        "Statistical significance: {:.1}σ ({})",
        significance,
        if ParticlePhysics::is_discovery(significance) {
            "Discovery!"
        } else {
            "Evidence"
        }
    );

    // Unit conversions
    let temp_ev = UnitConverter::kelvin_to_ev(150e6);
    println!("150 million K = {:.2e} eV", temp_ev);
    println!();

    // =========================================================================
    // 4. Particle Data
    // =========================================================================
    println!("4. Particle Data");
    println!("{}", "-".repeat(40));

    let muon = ParticleProperties {
        name: "muon".to_string(),
        pdg_id: 13,
        mass: Measurement::new(0.10566, 0.00001, "GeV"),
        charge: -1.0,
        spin: 0.5,
        parity: Some(-1),
        lifetime: Some(Measurement::new(2.197e-6, 0.001e-6, "s")),
        width: None,
        particle_type: Some(ParticleType::Lepton),
    };

    println!("Particle: {}", muon.name);
    println!("PDG ID: {}", muon.pdg_id);
    println!("Mass: {:.5} GeV", muon.mass.value);
    println!("Lifetime: {:.3e} s", muon.lifetime.as_ref().unwrap().value);
    println!();

    // =========================================================================
    // 5. Dark Matter Data
    // =========================================================================
    println!("5. Dark Matter Data");
    println!("{}", "-".repeat(40));

    let exclusion = ExclusionLimit {
        dm_candidate: DarkMatterCandidate::Wimp,
        dm_mass: Some(Measurement::new(100.0, 10.0, "GeV")),
        mass_range: None,
        cross_section_limit: Some(Measurement::new(1e-47, 1e-48, "cm^2")),
        coupling_limit: None,
        confidence_level: 0.90,
        interaction_type: InteractionType::SI,
        exposure: Some(Measurement::new(1000.0, 10.0, "kg·day")),
    };

    println!("DM candidate: {:?}", exclusion.dm_candidate);
    println!("Mass: {:.0} GeV", exclusion.dm_mass.as_ref().unwrap().value);
    println!(
        "Cross-section limit: {:.1e} cm²",
        exclusion.cross_section_limit.as_ref().unwrap().value
    );
    println!("Confidence level: {:.0}%", exclusion.confidence_level * 100.0);
    println!();

    // =========================================================================
    // 6. JSON Serialization
    // =========================================================================
    println!("6. JSON Serialization");
    println!("{}", "-".repeat(40));

    let json = serde_json::to_string_pretty(&fusion_data)?;
    println!("Fusion data JSON (first 500 chars):");
    println!("{}", &json[..json.len().min(500)]);
    println!("...");
    println!();

    // =========================================================================
    // 7. Quantum Gravity Calculations
    // =========================================================================
    println!("7. Quantum Gravity Calculations");
    println!("{}", "-".repeat(40));

    // Schwarzschild radius
    let earth_mass = 5.97e24; // kg
    let r_s = QuantumGravityPhysics::schwarzschild_radius(earth_mass);
    println!("Earth's Schwarzschild radius: {:.2e} m ({:.2} mm)", r_s, r_s * 1000.0);

    // Hawking temperature
    let solar_mass = 1.989e30; // kg
    let t_hawking = QuantumGravityPhysics::hawking_temperature(solar_mass);
    println!("Solar mass BH Hawking temp: {:.2e} K", t_hawking);

    // Planck mass
    let m_planck = QuantumGravityPhysics::planck_mass();
    println!("Planck mass: {:.3e} kg", m_planck);
    println!();

    println!("=== Example Complete! ===");
    println!();
    println!("弘益人間 - Benefit All Humanity 🤟");

    Ok(())
}
