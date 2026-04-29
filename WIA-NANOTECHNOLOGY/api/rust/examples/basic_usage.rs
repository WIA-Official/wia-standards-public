//! Basic usage example for WIA Nanotechnology SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_nanotechnology::*;
use chrono::Utc;
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<()> {
    println!("WIA Nanotechnology SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    // Create synthesis parameters
    let params = SynthesisParams {
        method: SynthesisMethod::ChemicalVaporDeposition,
        temperature_celsius: 800.0,
        pressure_kpa: 101.325,
        duration_minutes: 120.0,
        precursors: vec!["Methane".to_string(), "Hydrogen".to_string()],
    };

    // Validate parameters
    validators::validate_synthesis_params(&params)?;
    println!("✓ Synthesis parameters validated");

    // Create a carbon nanotube nanoparticle
    let nanoparticle = Nanoparticle {
        id: utils::generate_nanoparticle_id(),
        name: "CNT-001".to_string(),
        material: NanoMaterial::CarbonNanotube,
        size_nm: 10.0,
        shape: NanoShape::Tube,
        surface_area_m2: 0.000001,
        created_at: Utc::now(),
        properties: utils::create_default_properties(),
    };

    // Validate nanoparticle
    validators::validate_nanoparticle(&nanoparticle)?;
    println!("✓ Nanoparticle created: {}", utils::format_nanoparticle_info(&nanoparticle));

    // Calculate surface area to volume ratio
    let sa_v_ratio = utils::calculate_sa_to_v_ratio(nanoparticle.size_nm);
    println!("  SA/V ratio: {:.2}", sa_v_ratio);

    // Estimate synthesis time
    let estimated_time = utils::estimate_synthesis_time(&params.method);
    println!("  Estimated synthesis time: {:.0} minutes", estimated_time);

    // Calculate optimal temperature
    let optimal_temp = utils::calculate_optimal_temperature(&nanoparticle.material);
    println!("  Optimal synthesis temperature: {:.0}°C", optimal_temp);

    Ok(())
}
