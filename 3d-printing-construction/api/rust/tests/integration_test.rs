//! Integration tests for 3D Printing Construction SDK
//!
//! 弘益人間 - Quality through testing

use wia_3d_printing_construction::*;

#[test]
fn test_building_design_creation() {
    let design = BuildingDesign::new("Test House", 100.0, MaterialType::Concrete);

    assert_eq!(design.name, "Test House");
    assert_eq!(design.area_sqm, 100.0);
    assert_eq!(design.material_type, MaterialType::Concrete);
    assert_eq!(design.floors, 1);
}

#[test]
fn test_material_calculation() {
    let design = BuildingDesign::new("Test", 100.0, MaterialType::Concrete);
    let material = calculate_material_required(&design, &MaterialType::Concrete);

    assert!(material > 0.0);
    assert!(material < 100000.0); // Sanity check
}

#[test]
fn test_cost_estimation() {
    let design = BuildingDesign::new("Test", 100.0, MaterialType::Concrete);
    let cost = calculate_cost_estimate(&design, &MaterialType::Concrete);

    assert!(cost > 0.0);
}

#[test]
fn test_design_validation() {
    let design = BuildingDesign::new("Valid House", 100.0, MaterialType::Concrete);
    assert!(validate_building_design(&design).is_ok());
}

#[test]
fn test_design_validation_invalid_area() {
    let mut design = BuildingDesign::new("Invalid", 100.0, MaterialType::Concrete);
    design.area_sqm = -10.0;
    assert!(validate_building_design(&design).is_err());
}

#[test]
fn test_design_validation_zero_floors() {
    let mut design = BuildingDesign::new("Invalid", 100.0, MaterialType::Concrete);
    design.floors = 0;
    assert!(validate_building_design(&design).is_err());
}

#[test]
fn test_material_compatibility() {
    let result = validate_material_compatibility(
        &MaterialType::ReinforcedConcrete,
        &StructureType::LoadBearing,
    );
    assert!(result.is_ok());
}

#[test]
fn test_printer_validation() {
    let config = PrinterConfig {
        printer_id: "TEST-001".to_string(),
        model: "Test Printer".to_string(),
        max_build_area_sqm: 500.0,
        max_height_m: 10.0,
        nozzle_size_mm: 50.0,
        print_speed_mm_s: 100.0,
        material_capacity_kg: 5000.0,
        status: PrinterStatus::Idle,
    };

    assert!(validate_printer_config(&config).is_ok());
}

#[test]
fn test_design_fits_printer() {
    let design = BuildingDesign::new("Small House", 100.0, MaterialType::Concrete);
    let printer = PrinterConfig {
        printer_id: "TEST-001".to_string(),
        model: "Test Printer".to_string(),
        max_build_area_sqm: 500.0,
        max_height_m: 10.0,
        nozzle_size_mm: 50.0,
        print_speed_mm_s: 100.0,
        material_capacity_kg: 5000.0,
        status: PrinterStatus::Idle,
    };

    assert!(validate_design_fits_printer(&design, &printer).is_ok());
}

#[test]
fn test_design_too_large_for_printer() {
    let design = BuildingDesign::new("Large Building", 1000.0, MaterialType::Concrete);
    let printer = PrinterConfig {
        printer_id: "TEST-001".to_string(),
        model: "Small Printer".to_string(),
        max_build_area_sqm: 100.0,
        max_height_m: 5.0,
        nozzle_size_mm: 30.0,
        print_speed_mm_s: 50.0,
        material_capacity_kg: 1000.0,
        status: PrinterStatus::Idle,
    };

    assert!(validate_design_fits_printer(&design, &printer).is_err());
}

#[test]
fn test_safety_inspection_required() {
    let mut design = BuildingDesign::new("Tall Building", 100.0, MaterialType::Concrete);
    design.floors = 3;

    assert!(requires_safety_inspection(&design));
}

#[test]
fn test_safety_inspection_not_required() {
    let design = BuildingDesign::new("Small House", 50.0, MaterialType::Concrete);

    assert!(!requires_safety_inspection(&design));
}

#[test]
fn test_carbon_footprint_calculation() {
    let design = BuildingDesign::new("Eco House", 100.0, MaterialType::BioMaterial);
    let footprint = calculate_carbon_footprint(&design, &MaterialType::BioMaterial);

    assert!(footprint > 0.0);

    // Bio material should have lower footprint than concrete
    let concrete_footprint = calculate_carbon_footprint(&design, &MaterialType::Concrete);
    assert!(footprint < concrete_footprint);
}

#[test]
fn test_energy_consumption_calculation() {
    let design = BuildingDesign::new("Test", 100.0, MaterialType::Concrete);
    let printer = PrinterConfig {
        printer_id: "TEST-001".to_string(),
        model: "Test Printer".to_string(),
        max_build_area_sqm: 500.0,
        max_height_m: 10.0,
        nozzle_size_mm: 50.0,
        print_speed_mm_s: 100.0,
        material_capacity_kg: 5000.0,
        status: PrinterStatus::Idle,
    };

    let energy = calculate_energy_consumption(&design, &printer);
    assert!(energy > 0.0);
}

#[test]
fn test_quality_checklist_generation() {
    let design = BuildingDesign::new("Test", 100.0, MaterialType::Concrete);
    let checklist = generate_quality_checklist(&design);

    assert!(!checklist.is_empty());
    assert!(checklist.len() >= 5);
}

#[test]
fn test_client_creation() {
    let client = Client::new("https://api.wia.global", "test-key");
    assert!(client.is_ok());
}

#[test]
fn test_client_empty_api_key() {
    let client = Client::new("https://api.wia.global", "");
    assert!(client.is_err());
}

#[test]
fn test_material_type_serialization() {
    let material = MaterialType::ReinforcedConcrete;
    let json = serde_json::to_string(&material).unwrap();
    assert_eq!(json, "\"reinforced_concrete\"");
}

#[test]
fn test_print_status_serialization() {
    let status = PrintStatus::Printing;
    let json = serde_json::to_string(&status).unwrap();
    assert_eq!(json, "\"printing\"");
}
