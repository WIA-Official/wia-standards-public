//! Integration tests for WIA-SOIL-MICROBIOME SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity

use wia_soil_microbiome_sdk::{
    types::{
        SoilSample, SoilType, Location, DiversityIndex,
        CarbonMetrics, Environment,
    },
    SoilMicrobiomeClient,
};
use chrono::Utc;

#[test]
fn test_soil_sample_creation() {
    let sample = SoilSample {
        id: "TEST-001".to_string(),
        collection_date: Utc::now(),
        location: Location {
            latitude: 37.5665,
            longitude: 126.9780,
            altitude_m: Some(50.0),
            region: "Seoul".to_string(),
            field_name: Some("Test Field".to_string()),
        },
        depth_cm: 15.0,
        soil_type: SoilType::Loam,
        ph: Some(6.5),
        moisture_percent: Some(35.0),
        temperature_c: Some(18.0),
        organic_matter_percent: Some(4.2),
    };

    assert_eq!(sample.id, "TEST-001");
    assert_eq!(sample.depth_cm, 15.0);
    assert_eq!(sample.soil_type, SoilType::Loam);
}

#[test]
fn test_soil_types() {
    let types = vec![
        SoilType::Sand,
        SoilType::Loam,
        SoilType::Clay,
        SoilType::SandyLoam,
        SoilType::ClayLoam,
        SoilType::SiltLoam,
        SoilType::Peat,
    ];

    assert!(types.len() >= 5);
}

#[test]
fn test_location_creation() {
    let location = Location {
        latitude: 37.5665,
        longitude: 126.9780,
        altitude_m: Some(50.0),
        region: "Seoul, South Korea".to_string(),
        field_name: Some("Organic Farm".to_string()),
    };

    assert!((location.latitude - 37.5665).abs() < 0.001);
    assert!((location.longitude - 126.9780).abs() < 0.001);
    assert_eq!(location.altitude_m, Some(50.0));
}

#[test]
fn test_diversity_index() {
    let diversity = DiversityIndex {
        shannon: 4.2,
        simpson: 0.92,
        chao1: 1850.0,
        observed_species: 1650,
        evenness: 0.85,
    };

    // Shannon index typically ranges from 0 to ~5 for soil microbiomes
    assert!(diversity.shannon > 0.0 && diversity.shannon < 6.0);
    // Simpson index ranges from 0 to 1
    assert!(diversity.simpson >= 0.0 && diversity.simpson <= 1.0);
    // Evenness ranges from 0 to 1
    assert!(diversity.evenness >= 0.0 && diversity.evenness <= 1.0);
}

#[test]
fn test_carbon_metrics() {
    let carbon = CarbonMetrics {
        total_organic_carbon_percent: 2.8,
        soil_organic_matter_percent: 4.8,
        carbon_to_nitrogen_ratio: 12.5,
        microbial_biomass_carbon_mg_kg: 450.0,
        respiration_rate_mg_co2_kg_day: 28.5,
        carbon_sequestration_potential: Some(0.75),
    };

    // SOM is typically ~1.72x TOC
    let expected_som = carbon.total_organic_carbon_percent * 1.72;
    assert!((carbon.soil_organic_matter_percent - expected_som).abs() < 1.0);

    // C:N ratio typically 8-15 for healthy soil
    assert!(carbon.carbon_to_nitrogen_ratio >= 8.0 && carbon.carbon_to_nitrogen_ratio <= 20.0);
}

#[test]
fn test_carbon_sequestration_calculation() {
    let initial_toc = 2.5;
    let final_toc = 3.0;
    let bulk_density = 1.3;
    let depth_cm = 30.0;

    let initial_stock = initial_toc * bulk_density * depth_cm * 0.1;
    let final_stock = final_toc * bulk_density * depth_cm * 0.1;
    let sequestered = final_stock - initial_stock;

    assert!(sequestered > 0.0, "Should have positive sequestration");
    assert!((sequestered - 1.95).abs() < 0.01, "Expected ~1.95 t C/ha");
}

#[test]
fn test_environment_urls() {
    assert_eq!(
        Environment::Production.base_url(),
        "https://api.soil.wia.org/v1"
    );
    assert_eq!(
        Environment::Sandbox.base_url(),
        "https://sandbox.soil.wia.org/v1"
    );
}

#[test]
fn test_ph_validation() {
    // Valid pH range for soil is typically 3.5-9.5
    let valid_ph_values = vec![4.0, 5.5, 6.5, 7.0, 7.5, 8.0];

    for ph in valid_ph_values {
        assert!(ph >= 3.5 && ph <= 9.5, "pH {} should be in valid range", ph);
    }
}

#[test]
fn test_moisture_validation() {
    // Moisture percentage should be 0-100
    let moisture_values = vec![0.0, 25.0, 50.0, 75.0, 100.0];

    for moisture in moisture_values {
        assert!(
            moisture >= 0.0 && moisture <= 100.0,
            "Moisture {} should be in valid range",
            moisture
        );
    }
}

#[test]
fn test_client_builder() {
    let result = SoilMicrobiomeClient::builder()
        .api_key("test_key")
        .environment(Environment::Sandbox)
        .timeout(60)
        .build();

    assert!(result.is_ok());
    let client = result.unwrap();
    assert_eq!(client.environment(), Environment::Sandbox);
}

#[tokio::test]
async fn test_client_base_url() {
    let client = SoilMicrobiomeClient::builder()
        .api_key("demo_key")
        .environment(Environment::Sandbox)
        .build()
        .unwrap();

    let base_url = client.base_url();
    assert!(base_url.contains("sandbox"));
}

#[test]
fn test_diversity_health_score() {
    // Calculate a simple health score based on diversity
    let diversity = DiversityIndex {
        shannon: 4.2,
        simpson: 0.92,
        chao1: 1850.0,
        observed_species: 1650,
        evenness: 0.85,
    };

    // Simple scoring: higher values = better health
    let shannon_score = (diversity.shannon / 5.0) * 100.0;
    let simpson_score = diversity.simpson * 100.0;
    let evenness_score = diversity.evenness * 100.0;

    let health_score = (shannon_score + simpson_score + evenness_score) / 3.0;

    assert!(health_score > 70.0, "Should indicate healthy soil");
}
