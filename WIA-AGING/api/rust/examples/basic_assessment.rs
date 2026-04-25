//! Basic Assessment Example for WIA-AGING SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity
//!
//! This example demonstrates how to calculate biological age
//! using the WIA-AGING SDK.

use wia_aging_sdk::{
    types::{Biomarker, BiomarkerCategory, calculate_phenotypic_age},
    AgingClient, Environment,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("===========================================");
    println!("  WIA-AGING SDK - Basic Assessment Example");
    println!("  弘益人間 (홍익인간) - Benefit All Humanity");
    println!("===========================================\n");

    // Example 1: Calculate biological age locally
    println!("📊 Example 1: Local Biological Age Calculation");
    println!("----------------------------------------------");

    let chronological_age = 55.0;
    let biological_age = calculate_phenotypic_age(
        chronological_age,
        Some(4.3),   // Albumin (g/dL)
        Some(0.95),  // Creatinine (mg/dL)
        Some(98.0),  // Glucose (mg/dL)
        Some(1.2),   // CRP (mg/L)
        Some(26.0),  // Lymphocyte (%)
    );

    println!("Chronological Age: {:.1} years", chronological_age);
    println!("Biological Age: {:.1} years", biological_age.value);
    println!("Age Difference: {:+.1} years", biological_age.age_difference.unwrap_or(0.0));
    println!("Aging Rate: {:.2}x", biological_age.aging_rate.unwrap_or(1.0));
    println!("Confidence: {:.0}%", biological_age.confidence.unwrap_or(0.0) * 100.0);

    // Example 2: Create biomarkers
    println!("\n📈 Example 2: Creating Biomarkers");
    println!("---------------------------------");

    let biomarkers = vec![
        Biomarker::new("WIA-AGE-CRP", 1.2, "mg/L")
            .with_name("C-Reactive Protein")
            .with_category(BiomarkerCategory::Inflammatory),
        Biomarker::new("WIA-AGE-GLU", 98.0, "mg/dL")
            .with_name("Fasting Glucose")
            .with_category(BiomarkerCategory::Metabolic),
        Biomarker::new("WIA-AGE-ALB", 4.3, "g/dL")
            .with_name("Albumin")
            .with_category(BiomarkerCategory::OrganFunction),
    ];

    for biomarker in &biomarkers {
        println!(
            "  {} [{}]: {:.2} {}",
            biomarker.name.as_deref().unwrap_or("Unknown"),
            biomarker.code,
            biomarker.value,
            biomarker.unit
        );
    }

    // Example 3: API Client (requires API key)
    println!("\n🔗 Example 3: API Client Setup");
    println!("------------------------------");

    // Note: Replace with your actual API key
    let api_key = std::env::var("WIA_AGING_API_KEY").unwrap_or_else(|_| "demo_key".to_string());

    let client = AgingClient::builder()
        .api_key(&api_key)
        .environment(Environment::Sandbox)
        .build()?;

    println!("Client created for {} environment",
        if matches!(client.environment(), Environment::Sandbox) { "Sandbox" } else { "Production" }
    );
    println!("Base URL: {}", client.base_url());

    println!("\n✅ Assessment example completed!");
    println!("弘益人間 (홍익인간) - For the benefit of all humanity 🌍");

    Ok(())
}
