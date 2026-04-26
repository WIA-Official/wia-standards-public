//! Utility functions for 3D printing construction
//!
//! 弘益人間 - Tools that benefit all builders

use crate::types::*;
use chrono::{DateTime, Duration, Utc};

/// Calculate estimated print time based on building volume
pub fn estimate_print_time(design: &BuildingDesign, printer: &PrinterConfig) -> Duration {
    // Simplified calculation: volume / print speed
    let volume_cubic_m = design.area_sqm * design.height_m;
    let volume_cubic_mm = volume_cubic_m * 1_000_000_000.0;

    // Estimate based on print speed and nozzle size
    let print_speed_cubic_mm_s = printer.print_speed_mm_s * printer.nozzle_size_mm * printer.nozzle_size_mm;
    let seconds = (volume_cubic_mm / print_speed_cubic_mm_s) as i64;

    Duration::seconds(seconds)
}

/// Calculate required material amount in kg
pub fn calculate_material_required(design: &BuildingDesign, material: &MaterialType) -> f64 {
    let volume_cubic_m = design.area_sqm * design.height_m;

    // Material density in kg/m³
    let density = match material {
        MaterialType::Concrete => 2400.0,
        MaterialType::ReinforcedConcrete => 2500.0,
        MaterialType::Polymer => 1200.0,
        MaterialType::Composite => 1500.0,
        MaterialType::RecycledMaterial => 2000.0,
        MaterialType::BioMaterial => 1000.0,
    };

    volume_cubic_m * density * 0.3 // 30% infill typical for construction
}

/// Calculate cost estimate in USD
pub fn calculate_cost_estimate(design: &BuildingDesign, material: &MaterialType) -> f64 {
    let material_kg = calculate_material_required(design, material);

    // Material cost per kg in USD
    let cost_per_kg = match material {
        MaterialType::Concrete => 0.10,
        MaterialType::ReinforcedConcrete => 0.15,
        MaterialType::Polymer => 2.50,
        MaterialType::Composite => 5.00,
        MaterialType::RecycledMaterial => 0.08,
        MaterialType::BioMaterial => 3.00,
    };

    let material_cost = material_kg * cost_per_kg;
    let labor_cost = design.area_sqm * 50.0; // $50 per sqm
    let overhead = (material_cost + labor_cost) * 0.20; // 20% overhead

    material_cost + labor_cost + overhead
}

/// Format print job status for display
pub fn format_status(job: &PrintJob) -> String {
    match job.status {
        PrintStatus::Queued => format!("⏳ Queued - Position in queue"),
        PrintStatus::Preparing => format!("🔧 Preparing - Setting up printer"),
        PrintStatus::Printing => format!("🏗️ Printing - {:.1}% complete", job.progress_percent),
        PrintStatus::Paused => format!("⏸️ Paused - {:.1}% complete", job.progress_percent),
        PrintStatus::QualityCheck => format!("🔍 Quality Check - Inspecting build"),
        PrintStatus::Completed => format!("✅ Completed - Ready for inspection"),
        PrintStatus::Failed => format!("❌ Failed - Check error logs"),
    }
}

/// Calculate energy consumption estimate in kWh
pub fn calculate_energy_consumption(design: &BuildingDesign, printer: &PrinterConfig) -> f64 {
    let print_time = estimate_print_time(design, printer);
    let hours = print_time.num_hours() as f64;

    // Average power consumption for 3D construction printers: 50 kW
    let power_kw = 50.0;

    hours * power_kw
}

/// Generate quality check checklist
pub fn generate_quality_checklist(design: &BuildingDesign) -> Vec<String> {
    vec![
        "Dimensional accuracy: ±5mm tolerance".to_string(),
        "Wall thickness: meets minimum requirements".to_string(),
        "Surface finish: smooth, no cracks".to_string(),
        format!("Structural integrity: supports {} floors", design.floors),
        "Material bonding: layers properly fused".to_string(),
        "Safety compliance: meets building codes".to_string(),
        "Environmental impact: within acceptable limits".to_string(),
    ]
}

/// Calculate carbon footprint in kg CO2
pub fn calculate_carbon_footprint(design: &BuildingDesign, material: &MaterialType) -> f64 {
    let material_kg = calculate_material_required(design, material);

    // CO2 emission per kg of material
    let co2_per_kg = match material {
        MaterialType::Concrete => 0.9,
        MaterialType::ReinforcedConcrete => 1.1,
        MaterialType::Polymer => 3.0,
        MaterialType::Composite => 2.0,
        MaterialType::RecycledMaterial => 0.3,
        MaterialType::BioMaterial => 0.1,
    };

    material_kg * co2_per_kg
}

/// Check if job requires safety inspection
pub fn requires_safety_inspection(design: &BuildingDesign) -> bool {
    design.floors > 1 || design.height_m > 5.0 || design.area_sqm > 200.0
}

/// Generate progress report
pub fn generate_progress_report(job: &PrintJob) -> String {
    let layers_completed = job.print_layers.iter()
        .filter(|l| l.status == LayerStatus::Completed)
        .count();

    let total_layers = job.print_layers.len();

    format!(
        "Print Job Progress Report\n\
         Status: {:?}\n\
         Progress: {:.1}%\n\
         Layers: {}/{}\n\
         Material Used: {:.1} kg\n\
         Quality Checks: {}",
        job.status,
        job.progress_percent,
        layers_completed,
        total_layers,
        job.material_used_kg,
        job.quality_checks.len()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_material_required() {
        let design = BuildingDesign::new("Test", 100.0, MaterialType::Concrete);
        let material = calculate_material_required(&design, &MaterialType::Concrete);
        assert!(material > 0.0);
    }

    #[test]
    fn test_calculate_cost_estimate() {
        let design = BuildingDesign::new("Test", 100.0, MaterialType::Concrete);
        let cost = calculate_cost_estimate(&design, &MaterialType::Concrete);
        assert!(cost > 0.0);
    }

    #[test]
    fn test_requires_safety_inspection() {
        let design = BuildingDesign::new("Test", 100.0, MaterialType::Concrete);
        assert!(!requires_safety_inspection(&design));
    }
}
