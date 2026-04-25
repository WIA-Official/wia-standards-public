//! Basic Soil Sample Analysis Example for WIA-SOIL-MICROBIOME SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity
//!
//! This example demonstrates soil sample collection and microbiome analysis.

use wia_soil_microbiome_sdk::{
    types::{
        SoilSample, SoilType, Location, MicrobiomeAnalysis,
        DiversityIndex, CarbonMetrics,
    },
    SoilMicrobiomeClient, Environment,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("===========================================");
    println!("  WIA-SOIL-MICROBIOME SDK - Basic Example");
    println!("  弘益人間 (홍익인간) - Benefit All Humanity");
    println!("===========================================\n");

    // Example 1: Create a soil sample
    println!("🌱 Example 1: Creating Soil Sample");
    println!("----------------------------------");

    let sample = SoilSample {
        id: "SAMPLE-2025-001".to_string(),
        collection_date: chrono::Utc::now(),
        location: Location {
            latitude: 37.5665,
            longitude: 126.9780,
            altitude_m: Some(50.0),
            region: "Seoul, South Korea".to_string(),
            field_name: Some("Organic Farm A".to_string()),
        },
        depth_cm: 15.0,
        soil_type: SoilType::Loam,
        ph: Some(6.5),
        moisture_percent: Some(35.0),
        temperature_c: Some(18.0),
        organic_matter_percent: Some(4.2),
    };

    println!("Sample ID: {}", sample.id);
    println!("Location: {} ({:.4}, {:.4})",
        sample.location.region,
        sample.location.latitude,
        sample.location.longitude
    );
    println!("Depth: {} cm", sample.depth_cm);
    println!("Soil Type: {:?}", sample.soil_type);
    println!("pH: {:.1}", sample.ph.unwrap_or(0.0));
    println!("Organic Matter: {:.1}%", sample.organic_matter_percent.unwrap_or(0.0));

    // Example 2: Microbiome diversity analysis
    println!("\n🦠 Example 2: Microbiome Diversity");
    println!("----------------------------------");

    let diversity = DiversityIndex {
        shannon: 4.2,
        simpson: 0.92,
        chao1: 1850.0,
        observed_species: 1650,
        evenness: 0.85,
    };

    println!("Shannon Index: {:.2}", diversity.shannon);
    println!("Simpson Index: {:.2}", diversity.simpson);
    println!("Chao1 Richness: {:.0}", diversity.chao1);
    println!("Observed Species: {}", diversity.observed_species);
    println!("Evenness: {:.2}", diversity.evenness);

    // Example 3: Carbon metrics
    println!("\n🌍 Example 3: Carbon Metrics");
    println!("----------------------------");

    let carbon = CarbonMetrics {
        total_organic_carbon_percent: 2.8,
        soil_organic_matter_percent: 4.8,
        carbon_to_nitrogen_ratio: 12.5,
        microbial_biomass_carbon_mg_kg: 450.0,
        respiration_rate_mg_co2_kg_day: 28.5,
        carbon_sequestration_potential: Some(0.75),
    };

    println!("Total Organic Carbon: {:.1}%", carbon.total_organic_carbon_percent);
    println!("C:N Ratio: {:.1}", carbon.carbon_to_nitrogen_ratio);
    println!("Microbial Biomass C: {:.0} mg/kg", carbon.microbial_biomass_carbon_mg_kg);
    println!("Respiration Rate: {:.1} mg CO2/kg/day", carbon.respiration_rate_mg_co2_kg_day);
    println!("Sequestration Potential: {:.0}%", carbon.carbon_sequestration_potential.unwrap_or(0.0) * 100.0);

    // Example 4: API Client setup
    println!("\n🔗 Example 4: API Client");
    println!("------------------------");

    let api_key = std::env::var("WIA_SOIL_API_KEY")
        .unwrap_or_else(|_| "demo_key".to_string());

    let client = SoilMicrobiomeClient::builder()
        .api_key(&api_key)
        .environment(Environment::Sandbox)
        .build()?;

    println!("Client created successfully");
    println!("Environment: Sandbox");
    println!("Base URL: {}", client.base_url());

    println!("\n✅ Basic sample example completed!");
    println!("弘益人間 (홍익인간) - For sustainable agriculture 🌾");

    Ok(())
}
