//! Basic usage example for WIA 3D Printing Construction SDK
//!
//! 弘益人間 (홍익인간) - Building homes that benefit all humanity

use wia_3d_printing_construction::{
    Client, BuildingDesign, MaterialType, StructureType,
    calculate_material_required, calculate_cost_estimate,
    estimate_print_time, PrinterConfig, PrinterStatus,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🏗️ WIA 3D Printing Construction SDK Example");
    println!("弘益人間 - Building the future with technology\n");

    // Initialize client
    let client = Client::new(
        "https://api.wia.global",
        std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()),
    )?;

    // Create a building design
    let mut design = BuildingDesign::new(
        "Modern Sustainable House",
        120.0, // 120 square meters
        MaterialType::ReinforcedConcrete,
    );
    design.height_m = 6.0; // 2 floors
    design.floors = 2;
    design.structure_type = StructureType::LoadBearing;

    println!("📐 Building Design:");
    println!("  Name: {}", design.name);
    println!("  Area: {} sqm", design.area_sqm);
    println!("  Height: {} m", design.height_m);
    println!("  Floors: {}", design.floors);
    println!("  Material: {:?}", design.material_type);
    println!();

    // Calculate estimates
    let material_required = calculate_material_required(&design, &design.material_type);
    let cost_estimate = calculate_cost_estimate(&design, &design.material_type);

    println!("📊 Estimates:");
    println!("  Material required: {:.1} kg", material_required);
    println!("  Cost estimate: ${:.2}", cost_estimate);
    println!();

    // Get printer configuration (mock for demo)
    let printer = PrinterConfig {
        printer_id: "PRINTER-001".to_string(),
        model: "WIA Construction Printer 5000".to_string(),
        max_build_area_sqm: 500.0,
        max_height_m: 12.0,
        nozzle_size_mm: 50.0,
        print_speed_mm_s: 100.0,
        material_capacity_kg: 10000.0,
        status: PrinterStatus::Idle,
    };

    let print_time = estimate_print_time(&design, &printer);
    println!("⏱️ Estimated print time: {} hours", print_time.num_hours());
    println!();

    // Submit print job (commented out for demo - requires real API)
    /*
    println!("🚀 Submitting print job...");
    let job = client.submit_print_job(&design).await?;
    println!("✅ Print job submitted successfully!");
    println!("  Job ID: {}", job.id);
    println!("  Status: {:?}", job.status);
    println!("  Estimated completion: {}", job.estimated_completion);
    */

    println!("💡 Demo completed! In production:");
    println!("  1. Submit design to WIA API");
    println!("  2. Monitor print progress in real-time");
    println!("  3. Receive quality check notifications");
    println!("  4. Get completion confirmation");
    println!();
    println!("弘益人間 - Technology that benefits all humanity 🌍");

    Ok(())
}
