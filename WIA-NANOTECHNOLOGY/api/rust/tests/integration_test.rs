//! Integration tests for WIA Nanotechnology SDK

use wia_nanotechnology::*;
use chrono::Utc;

#[test]
fn test_synthesis_params_validation() {
    let valid_params = SynthesisParams {
        method: SynthesisMethod::ChemicalVaporDeposition,
        temperature_celsius: 800.0,
        pressure_kpa: 101.325,
        duration_minutes: 120.0,
        precursors: vec!["Methane".to_string()],
    };

    assert!(validators::validate_synthesis_params(&valid_params).is_ok());

    let invalid_params = SynthesisParams {
        method: SynthesisMethod::ChemicalVaporDeposition,
        temperature_celsius: -500.0,
        pressure_kpa: 101.325,
        duration_minutes: 120.0,
        precursors: vec!["Methane".to_string()],
    };

    assert!(validators::validate_synthesis_params(&invalid_params).is_err());
}

#[test]
fn test_nanoparticle_validation() {
    let valid_particle = Nanoparticle {
        id: utils::generate_nanoparticle_id(),
        name: "CNT-001".to_string(),
        material: NanoMaterial::CarbonNanotube,
        size_nm: 10.0,
        shape: NanoShape::Tube,
        surface_area_m2: 0.000001,
        created_at: Utc::now(),
        properties: utils::create_default_properties(),
    };

    assert!(validators::validate_nanoparticle(&valid_particle).is_ok());

    let invalid_particle = Nanoparticle {
        id: utils::generate_nanoparticle_id(),
        name: "".to_string(),
        material: NanoMaterial::CarbonNanotube,
        size_nm: 10.0,
        shape: NanoShape::Tube,
        surface_area_m2: 0.000001,
        created_at: Utc::now(),
        properties: utils::create_default_properties(),
    };

    assert!(validators::validate_nanoparticle(&invalid_particle).is_err());
}

#[test]
fn test_sa_to_v_ratio() {
    let ratio = utils::calculate_sa_to_v_ratio(10.0);
    assert_eq!(ratio, 0.6);
}

#[test]
fn test_synthesis_time_estimation() {
    let time = utils::estimate_synthesis_time(&SynthesisMethod::ChemicalVaporDeposition);
    assert_eq!(time, 120.0);
}

#[test]
fn test_optimal_temperature() {
    let temp = utils::calculate_optimal_temperature(&NanoMaterial::CarbonNanotube);
    assert_eq!(temp, 800.0);
}
