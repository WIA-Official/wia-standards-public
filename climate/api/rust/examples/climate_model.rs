//! Climate Model example using the simulator adapter
//!
//! This example demonstrates how to use the ClimateModelSimulator
//! to generate CMIP6-compatible climate projection data.

use wia_climate::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Climate Standard - Climate Model Simulator ===\n");

    // Create simulators for different SSP scenarios
    let scenarios = [
        (SspScenario::Ssp126, "Low emissions (Paris Agreement aligned)"),
        (SspScenario::Ssp245, "Middle of the road"),
        (SspScenario::Ssp585, "Fossil-fueled development (high emissions)"),
    ];

    println!("Comparing temperature projections for Seoul, Korea (2050)\n");
    println!("{:<20} {:<35} {:<15} {:<15}", "Scenario", "Description", "Temp (K)", "Anomaly (°C)");
    println!("{}", "-".repeat(85));

    for (scenario, description) in scenarios {
        let simulator = ClimateModelSimulator::new(
            Location::new(37.5665, 126.978), // Seoul, Korea
            Device::new("NCAR", "CESM2"),
        )
        .with_scenario(scenario)
        .with_variable("tas");

        let message = simulator.read().await?;

        if let ClimateData::ClimateModel(ref data) = message.data {
            println!("{:<20} {:<35} {:<15.2} {:<15.2}",
                format!("{:?}", scenario),
                description,
                data.value.data,
                data.value.anomaly.unwrap_or(0.0)
            );
        }
    }

    println!();

    // Detailed output for SSP2-4.5 scenario
    println!("=== Detailed SSP2-4.5 Projection ===\n");

    let simulator = ClimateModelSimulator::new(
        Location::new(37.5665, 126.978),
        Device::new("NCAR", "CESM2 Simulator"),
    )
    .with_scenario(SspScenario::Ssp245)
    .with_variable("tas");

    let message = simulator.read().await?;

    println!("Model Information:");
    if let ClimateData::ClimateModel(ref data) = message.data {
        println!("  Source ID: {}", data.model.source_id);
        println!("  Institution: {:?}", data.model.institution_id);
        println!("  Experiment: {}", data.model.experiment_id);
        println!("  Variant: {}", data.model.variant_label);
        println!();

        println!("Variable:");
        println!("  Name: {}", data.variable.name);
        println!("  Long Name: {:?}", data.variable.long_name);
        println!("  Units: {}", data.variable.units);
        println!();

        if let Some(ref grid) = data.grid {
            println!("Grid:");
            println!("  Resolution: {:?}°", grid.resolution_deg);
            println!("  Type: {:?}", grid.grid_type);
            println!();
        }

        println!("Time:");
        println!("  Period: {:?} to {:?}", data.time.start_date, data.time.end_date);
        println!("  Frequency: {:?}", data.time.frequency);
        println!();

        println!("Values:");
        println!("  Data: {:.2} K ({:.2}°C)",
            data.value.data,
            data.value.data - 273.15
        );
        println!("  Anomaly: {:+.2}°C", data.value.anomaly.unwrap_or(0.0));
        println!("  Climatology: {:.2} K", data.value.climatology.unwrap_or(0.0));
        println!("  5th percentile: {:?} K", data.value.percentile_5);
        println!("  95th percentile: {:?} K", data.value.percentile_95);
        println!();

        if let Some(ref scenario) = data.scenario {
            println!("Scenario:");
            println!("  SSP: {:?}", scenario.ssp);
            println!("  Forcing: {:?} W/m²", scenario.forcing_level_w_per_m2);
            println!("  Description: {:?}", scenario.description);
            println!();
        }

        if let Some(ref reference) = data.reference {
            println!("Reference:");
            println!("  URL: {:?}", reference.further_info_url);
            println!("  DOI: {:?}", reference.doi);
        }
    }

    // Export to JSON
    println!("\n--- JSON Export ---\n");
    let json = message.to_json_pretty()?;
    println!("{}", json);

    println!("\n弘益人間 - Benefit All Humanity 🌍");

    Ok(())
}
