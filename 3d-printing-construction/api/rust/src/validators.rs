//! Validation functions for 3D printing construction data
//!
//! 弘益人間 - Safety through validation benefits all

use crate::error::{Error, Result};
use crate::types::*;

/// Validate building design specification
pub fn validate_building_design(design: &BuildingDesign) -> Result<()> {
    if design.name.is_empty() {
        return Err(Error::DesignValidationError("Building name cannot be empty".into()));
    }

    if design.area_sqm <= 0.0 {
        return Err(Error::DesignValidationError("Building area must be positive".into()));
    }

    if design.area_sqm > 10000.0 {
        return Err(Error::DesignValidationError("Building area exceeds maximum (10000 sqm)".into()));
    }

    if design.height_m <= 0.0 {
        return Err(Error::DesignValidationError("Building height must be positive".into()));
    }

    if design.height_m > 100.0 {
        return Err(Error::DesignValidationError("Building height exceeds maximum (100m)".into()));
    }

    if design.floors == 0 {
        return Err(Error::DesignValidationError("Building must have at least one floor".into()));
    }

    Ok(())
}

/// Validate material type compatibility
pub fn validate_material_compatibility(
    material: &MaterialType,
    structure: &StructureType,
) -> Result<()> {
    match (material, structure) {
        (MaterialType::Concrete, StructureType::Shell) => {
            Err(Error::MaterialError("Concrete not recommended for shell structures".into()))
        }
        (MaterialType::Polymer, StructureType::LoadBearing) if !cfg!(feature = "experimental") => {
            Err(Error::MaterialError("Polymer not approved for load-bearing structures".into()))
        }
        _ => Ok(()),
    }
}

/// Validate print job parameters
pub fn validate_print_job(job: &PrintJob) -> Result<()> {
    if job.progress_percent < 0.0 || job.progress_percent > 100.0 {
        return Err(Error::PrintJobError("Progress must be between 0 and 100".into()));
    }

    if job.material_used_kg < 0.0 {
        return Err(Error::MaterialError("Material used cannot be negative".into()));
    }

    Ok(())
}

/// Validate quality check measurements
pub fn validate_quality_check(check: &QualityCheck) -> Result<()> {
    if check.measurements.is_empty() {
        return Err(Error::QualityCheckError("Quality check must have measurements".into()));
    }

    for measurement in &check.measurements {
        if measurement.value < 0.0 {
            return Err(Error::QualityCheckError(
                format!("Invalid measurement value for {}", measurement.parameter)
            ));
        }

        if measurement.tolerance < 0.0 {
            return Err(Error::QualityCheckError(
                format!("Invalid tolerance for {}", measurement.parameter)
            ));
        }
    }

    Ok(())
}

/// Validate printer configuration
pub fn validate_printer_config(config: &PrinterConfig) -> Result<()> {
    if config.printer_id.is_empty() {
        return Err(Error::PrinterConfigError("Printer ID cannot be empty".into()));
    }

    if config.max_build_area_sqm <= 0.0 {
        return Err(Error::PrinterConfigError("Max build area must be positive".into()));
    }

    if config.max_height_m <= 0.0 {
        return Err(Error::PrinterConfigError("Max height must be positive".into()));
    }

    if config.nozzle_size_mm <= 0.0 {
        return Err(Error::PrinterConfigError("Nozzle size must be positive".into()));
    }

    if config.print_speed_mm_s <= 0.0 {
        return Err(Error::PrinterConfigError("Print speed must be positive".into()));
    }

    if config.material_capacity_kg <= 0.0 {
        return Err(Error::PrinterConfigError("Material capacity must be positive".into()));
    }

    Ok(())
}

/// Check if design fits in printer build area
pub fn validate_design_fits_printer(
    design: &BuildingDesign,
    printer: &PrinterConfig,
) -> Result<()> {
    if design.area_sqm > printer.max_build_area_sqm {
        return Err(Error::DesignValidationError(
            format!(
                "Design area ({} sqm) exceeds printer capacity ({} sqm)",
                design.area_sqm, printer.max_build_area_sqm
            )
        ));
    }

    if design.height_m > printer.max_height_m {
        return Err(Error::DesignValidationError(
            format!(
                "Design height ({} m) exceeds printer capacity ({} m)",
                design.height_m, printer.max_height_m
            )
        ));
    }

    Ok(())
}

/// Validate safety compliance
pub fn validate_safety_compliance(design: &BuildingDesign) -> Result<()> {
    // Check structural safety factors
    let min_wall_thickness_mm = match design.material_type {
        MaterialType::Concrete | MaterialType::ReinforcedConcrete => 150.0,
        MaterialType::Polymer => 200.0,
        MaterialType::Composite => 100.0,
        MaterialType::RecycledMaterial => 180.0,
        MaterialType::BioMaterial => 200.0,
    };

    // Additional safety checks would go here
    // This is a simplified example

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use uuid::Uuid;

    #[test]
    fn test_validate_building_design() {
        let design = BuildingDesign::new("Test House", 100.0, MaterialType::Concrete);
        assert!(validate_building_design(&design).is_ok());
    }

    #[test]
    fn test_validate_building_design_invalid_area() {
        let mut design = BuildingDesign::new("Test House", -10.0, MaterialType::Concrete);
        assert!(validate_building_design(&design).is_err());
    }

    #[test]
    fn test_validate_material_compatibility() {
        assert!(validate_material_compatibility(
            &MaterialType::ReinforcedConcrete,
            &StructureType::LoadBearing
        ).is_ok());
    }
}
