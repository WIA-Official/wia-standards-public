//! Carbon Tracking Example for WIA-SOIL-MICROBIOME SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity
//!
//! This example demonstrates soil carbon tracking and sequestration analysis.

use wia_soil_microbiome_sdk::{
    types::{CarbonMetrics, CarbonTrend, SequestrationReport},
    SoilMicrobiomeClient, Environment,
};
use chrono::{Utc, Duration};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("===========================================");
    println!("  WIA-SOIL-MICROBIOME - Carbon Tracking");
    println!("  弘益人間 (홍익인간) - Benefit All Humanity");
    println!("===========================================\n");

    // Example 1: Monthly carbon measurements
    println!("📊 Example 1: Monthly Carbon Tracking");
    println!("-------------------------------------");

    let monthly_data = vec![
        ("January", 2.5, 4.3),
        ("February", 2.6, 4.5),
        ("March", 2.7, 4.6),
        ("April", 2.8, 4.8),
        ("May", 2.9, 5.0),
        ("June", 3.0, 5.2),
    ];

    println!("Month        | TOC (%) | SOM (%) | Change");
    println!("-------------|---------|---------|-------");
    for (i, (month, toc, som)) in monthly_data.iter().enumerate() {
        let change = if i > 0 {
            format!("+{:.1}%", toc - monthly_data[i-1].1)
        } else {
            "  -  ".to_string()
        };
        println!("{:12} | {:6.1} | {:6.1} | {}", month, toc, som, change);
    }

    // Example 2: Carbon sequestration calculation
    println!("\n🌍 Example 2: Carbon Sequestration");
    println!("-----------------------------------");

    let initial_toc = 2.5;
    let final_toc = 3.0;
    let bulk_density = 1.3; // g/cm³
    let depth_cm = 30.0;

    // Calculate carbon sequestration (simplified)
    // SOC stock (t/ha) = TOC(%) × bulk_density × depth × 0.1
    let initial_stock = initial_toc * bulk_density * depth_cm * 0.1;
    let final_stock = final_toc * bulk_density * depth_cm * 0.1;
    let sequestered = final_stock - initial_stock;

    println!("Initial SOC Stock: {:.1} t C/ha", initial_stock);
    println!("Final SOC Stock: {:.1} t C/ha", final_stock);
    println!("Carbon Sequestered: {:.1} t C/ha", sequestered);
    println!("CO2 Equivalent: {:.1} t CO2/ha", sequestered * 3.67);

    // Example 3: Sequestration report generation
    println!("\n📝 Example 3: Sequestration Report");
    println!("-----------------------------------");

    let report = SequestrationReport {
        report_id: "SEQ-2025-001".to_string(),
        field_id: "FIELD-001".to_string(),
        start_date: Utc::now() - Duration::days(180),
        end_date: Utc::now(),
        initial_carbon_stock_t_ha: initial_stock,
        final_carbon_stock_t_ha: final_stock,
        net_sequestration_t_ha: sequestered,
        co2_equivalent_t_ha: sequestered * 3.67,
        practices: vec![
            "Cover cropping".to_string(),
            "No-till farming".to_string(),
            "Compost application".to_string(),
        ],
        verification_status: "Pending".to_string(),
    };

    println!("Report ID: {}", report.report_id);
    println!("Field ID: {}", report.field_id);
    println!("Period: 180 days");
    println!("Net Sequestration: {:.1} t C/ha", report.net_sequestration_t_ha);
    println!("CO2 Equivalent: {:.1} t CO2/ha", report.co2_equivalent_t_ha);
    println!("\nPractices Applied:");
    for practice in &report.practices {
        println!("  ✓ {}", practice);
    }

    // Example 4: Carbon credit estimation
    println!("\n💰 Example 4: Carbon Credit Potential");
    println!("--------------------------------------");

    let field_area_ha = 50.0;
    let total_sequestration = sequestered * field_area_ha;
    let carbon_price = 50.0; // USD per ton CO2

    println!("Field Area: {:.0} hectares", field_area_ha);
    println!("Total Sequestration: {:.1} t C", total_sequestration);
    println!("Total CO2 Equivalent: {:.1} t CO2", total_sequestration * 3.67);
    println!("Estimated Credit Value: ${:.0}", total_sequestration * 3.67 * carbon_price);

    println!("\n✅ Carbon tracking example completed!");
    println!("弘益人間 (홍익인간) - For climate action 🌍");

    Ok(())
}
