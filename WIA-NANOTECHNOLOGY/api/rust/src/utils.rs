//! Utility functions for nanotechnology

use crate::types::*;
use chrono::Utc;
use uuid::Uuid;

/// Calculate surface area to volume ratio
pub fn calculate_sa_to_v_ratio(size_nm: f64) -> f64 {
    // For spherical nanoparticles: SA/V = 6/diameter
    6.0 / size_nm
}

/// Estimate synthesis time based on method
pub fn estimate_synthesis_time(method: &SynthesisMethod) -> f64 {
    match method {
        SynthesisMethod::ChemicalVaporDeposition => 120.0,
        SynthesisMethod::SolGel => 180.0,
        SynthesisMethod::Hydrothermal => 240.0,
        SynthesisMethod::Precipitation => 60.0,
        SynthesisMethod::BallMilling => 360.0,
        SynthesisMethod::LaserAblation => 30.0,
    }
}

/// Create default nano properties
pub fn create_default_properties() -> NanoProperties {
    NanoProperties {
        electrical_conductivity: None,
        thermal_conductivity: None,
        optical_properties: None,
        magnetic_properties: None,
        toxicity_level: ToxicityLevel::Unknown,
    }
}

/// Generate nanoparticle ID
pub fn generate_nanoparticle_id() -> Uuid {
    Uuid::new_v4()
}

/// Calculate optimal synthesis temperature
pub fn calculate_optimal_temperature(material: &NanoMaterial) -> f64 {
    match material {
        NanoMaterial::CarbonNanotube => 800.0,
        NanoMaterial::Graphene => 1000.0,
        NanoMaterial::QuantumDot => 300.0,
        NanoMaterial::Fullerene => 600.0,
        NanoMaterial::MetalOxide => 500.0,
        NanoMaterial::GoldNanoparticle => 100.0,
        NanoMaterial::SilverNanoparticle => 80.0,
        NanoMaterial::Dendrimer => 25.0,
    }
}

/// Format nanoparticle info
pub fn format_nanoparticle_info(particle: &Nanoparticle) -> String {
    format!(
        "{} ({:?}) - Size: {:.2} nm, Shape: {:?}",
        particle.name, particle.material, particle.size_nm, particle.shape
    )
}
