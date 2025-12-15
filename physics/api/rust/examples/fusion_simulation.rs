//! Fusion simulation example
//!
//! This example demonstrates the fusion plasma simulator.

use wia_physics::prelude::*;

#[tokio::main]
async fn main() -> PhysicsResult<()> {
    println!("=== WIA Physics SDK - Fusion Simulation ===\n");

    // =========================================================================
    // Create ITER-like simulator
    // =========================================================================
    println!("Creating ITER-like fusion simulator...\n");

    let config = FusionSimConfig {
        initial_temperature: 15.0,  // keV
        initial_density: 1e20,      // m^-3
        heating_power: 50.0,        // MW
        magnetic_field: 5.3,        // T
        major_radius: 6.2,          // m
        minor_radius: 2.0,          // m
        dt: 0.001,                  // 1 ms time step
        noise_level: 0.02,          // 2% fluctuations
    };

    let mut simulator = FusionSimulator::new(config);

    // =========================================================================
    // Run simulation
    // =========================================================================
    println!("Running simulation for 1 second (1000 steps)...\n");
    println!("{:<10} {:>15} {:>15} {:>12} {:>12}",
             "Time (ms)", "Temp (MK)", "Density (m⁻³)", "τ_E (s)", "Q");
    println!("{}", "-".repeat(70));

    let steps = 1000;
    let print_interval = 100;

    for i in 0..=steps {
        if i % print_interval == 0 {
            let state = simulator.get_state()?;

            let temp_mk = state.plasma.temperature.value / 1e6; // to MK
            let density = state.plasma.density.value;
            let tau_e = state.plasma.confinement_time
                .as_ref()
                .map(|t| t.value)
                .unwrap_or(0.0);
            let q = state.energy_balance
                .as_ref()
                .and_then(|e| e.q_factor.as_ref())
                .map(|q| q.value)
                .unwrap_or(0.0);

            println!("{:<10} {:>15.1} {:>15.2e} {:>12.3} {:>12.2}",
                     (simulator.current_time() * 1000.0) as i64,
                     temp_mk,
                     density,
                     tau_e,
                     q);
        }

        if i < steps {
            simulator.step().await?;
        }
    }

    // =========================================================================
    // Final state analysis
    // =========================================================================
    println!("\n{}", "=".repeat(70));
    println!("Final State Analysis");
    println!("{}", "=".repeat(70));

    let final_state = simulator.get_state()?;

    println!("\nPlasma Parameters:");
    println!("  Temperature: {:.2e} K ({:.2} keV)",
             final_state.plasma.temperature.value,
             UnitConverter::kelvin_to_kev(final_state.plasma.temperature.value));
    println!("  Density: {:.2e} m⁻³", final_state.plasma.density.value);

    if let Some(tau) = &final_state.plasma.confinement_time {
        println!("  Confinement time: {:.3} s", tau.value);
    }

    if let Some(tp) = &final_state.plasma.triple_product {
        println!("  Triple product: {:.2e} keV·s/m³", tp.value);
        println!("  Ignition threshold: {:.2e} keV·s/m³", FusionPhysics::lawson_criterion_dt());
        println!("  Ignition possible: {}",
                 if FusionPhysics::is_ignition_possible(tp.value) { "YES!" } else { "Not yet" });
    }

    println!("\nMagnetic Configuration:");
    if let Some(config) = &final_state.magnetic_configuration {
        println!("  Type: {:?}", config.confinement_type);
        if let Some(b) = &config.toroidal_field {
            println!("  Toroidal field: {:.2} T", b.value);
        }
        if let Some(r) = &config.major_radius {
            println!("  Major radius: {:.2} m", r.value);
        }
        if let Some(a) = &config.minor_radius {
            println!("  Minor radius: {:.2} m", a.value);
        }
        if let Some(v) = &config.plasma_volume {
            println!("  Plasma volume: {:.1} m³", v.value);
        }
    }

    println!("\nEnergy Balance:");
    if let Some(energy) = &final_state.energy_balance {
        if let Some(p_in) = &energy.input_power {
            println!("  Input power: {:.1} MW", p_in.value);
        }
        if let Some(p_fus) = &energy.fusion_power {
            println!("  Fusion power: {:.1} MW", p_fus.value);
        }
        if let Some(q) = &energy.q_factor {
            println!("  Q-factor: {:.2}", q.value);
            if q.value >= 10.0 {
                println!("  🎉 Achieved Q ≥ 10 (ITER goal)!");
            }
        }
    }

    // =========================================================================
    // Export to JSON
    // =========================================================================
    println!("\nExporting final state to JSON...");
    let json = serde_json::to_string_pretty(&final_state)?;
    println!("JSON size: {} bytes", json.len());

    // Print first part of JSON
    println!("\nJSON preview (first 600 chars):");
    println!("{}", &json[..json.len().min(600)]);
    println!("...\n");

    // =========================================================================
    // Generate multiple data points
    // =========================================================================
    println!("Generating time series data...");

    simulator.reset();
    let mut time_series: Vec<FusionData> = Vec::new();

    for _ in 0..10 {
        for _ in 0..100 {
            simulator.step().await?;
        }
        time_series.push(simulator.get_state()?);
    }

    println!("Generated {} data points", time_series.len());

    // Show temperature evolution
    println!("\nTemperature evolution:");
    for (i, data) in time_series.iter().enumerate() {
        let temp_mk = data.plasma.temperature.value / 1e6;
        let bar_len = (temp_mk / 10.0) as usize;
        println!("  t={:>4}ms: {:>6.1} MK |{}",
                 (i + 1) * 100,
                 temp_mk,
                 "█".repeat(bar_len.min(50)));
    }

    println!("\n=== Simulation Complete! ===");
    println!("\n弘益人間 - Benefit All Humanity 🤟");

    Ok(())
}
