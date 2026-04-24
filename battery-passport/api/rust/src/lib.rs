//! WIA Battery Passport API
//!
//! EU Battery Regulation 2027 compliant battery lifecycle tracking.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

pub mod types;
pub mod core;
pub mod server;

pub use types::*;
pub use core::*;

/// Library version
pub const VERSION: &str = "1.0.0";

/// EU Battery Regulation reference
pub const EU_REGULATION: &str = "EU 2023/1542";

/// Supported battery chemistries
pub const SUPPORTED_CHEMISTRIES: &[&str] = &[
    "lfp",      // Lithium Iron Phosphate
    "nmc",      // Nickel Manganese Cobalt
    "nca",      // Nickel Cobalt Aluminum
    "lco",      // Lithium Cobalt Oxide
    "lmo",      // Lithium Manganese Oxide
    "lto",      // Lithium Titanate
    "solid_state",
    "sodium_ion",
];

/// Carbon class thresholds (kg CO2e per kWh)
pub const CARBON_CLASS_THRESHOLDS: [(CarbonClass, f64); 5] = [
    (CarbonClass::A, 50.0),
    (CarbonClass::B, 65.0),
    (CarbonClass::C, 80.0),
    (CarbonClass::D, 95.0),
    (CarbonClass::E, f64::MAX),
];

/// EU recycled content targets (by 2031)
pub const EU_RECYCLED_TARGETS_2031: RecycledTargets = RecycledTargets {
    cobalt: 16.0,
    lithium: 6.0,
    nickel: 6.0,
    lead: 85.0,
};

/// EU material recovery targets (by 2031)
pub const EU_RECOVERY_TARGETS_2031: RecoveryTargets = RecoveryTargets {
    cobalt: 95.0,
    lithium: 80.0,
    nickel: 95.0,
    copper: 95.0,
};

/// Recycled content targets structure
pub struct RecycledTargets {
    pub cobalt: f64,
    pub lithium: f64,
    pub nickel: f64,
    pub lead: f64,
}

/// Material recovery targets structure
pub struct RecoveryTargets {
    pub cobalt: f64,
    pub lithium: f64,
    pub nickel: f64,
    pub copper: f64,
}

// ============================================================================
// Utility Functions
// ============================================================================

/// Validate QR code format (WIABAT:XXXXXXXXXXXXXXXX)
pub fn validate_qr_code(qr: &str) -> bool {
    if !qr.starts_with("WIABAT:") {
        return false;
    }
    let code = &qr[7..];
    code.len() == 16 && code.chars().all(|c| c.is_ascii_alphanumeric())
}

/// Generate QR code identifier
pub fn generate_qr_code() -> String {
    use sha2::{Sha256, Digest};
    let uuid = uuid::Uuid::now_v7();
    let mut hasher = Sha256::new();
    hasher.update(uuid.as_bytes());
    let hash = hasher.finalize();
    let code: String = hash.iter()
        .take(8)
        .map(|b| format!("{:02X}", b))
        .collect();
    format!("WIABAT:{}", code)
}

/// Determine carbon class from kg CO2e per kWh
pub fn determine_carbon_class(per_kwh_kg_co2e: f64) -> CarbonClass {
    for (class, threshold) in CARBON_CLASS_THRESHOLDS.iter() {
        if per_kwh_kg_co2e < *threshold {
            return *class;
        }
    }
    CarbonClass::E
}

/// Calculate SOH classification
pub fn classify_soh(soh_percent: f64) -> SohStatus {
    match soh_percent {
        s if s >= 90.0 => SohStatus::Excellent,
        s if s >= 80.0 => SohStatus::Good,
        s if s >= 70.0 => SohStatus::Fair,
        s if s >= 60.0 => SohStatus::Degraded,
        _ => SohStatus::Poor,
    }
}

/// SOH status classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SohStatus {
    Excellent,
    Good,
    Fair,
    Degraded,
    Poor,
}

impl SohStatus {
    pub fn recommendation(&self) -> &'static str {
        match self {
            SohStatus::Excellent => "Normal operation",
            SohStatus::Good => "Monitor closely",
            SohStatus::Fair => "Consider second-life assessment",
            SohStatus::Degraded => "Second-life application recommended",
            SohStatus::Poor => "Recycling recommended",
        }
    }
}

/// Calculate energy density (Wh/kg)
pub fn calculate_energy_density(capacity_wh: f64, weight_kg: f64) -> f64 {
    if weight_kg > 0.0 {
        capacity_wh / weight_kg
    } else {
        0.0
    }
}

/// Estimate cycles from energy throughput
pub fn estimate_cycles(
    total_energy_throughput_kwh: f64,
    capacity_kwh: f64,
) -> u32 {
    if capacity_kwh > 0.0 {
        (total_energy_throughput_kwh / capacity_kwh) as u32
    } else {
        0
    }
}

/// Check if country is in EU
pub fn is_eu_country(country_code: &str) -> bool {
    const EU_COUNTRIES: &[&str] = &[
        "AT", "BE", "BG", "HR", "CY", "CZ", "DK", "EE", "FI", "FR",
        "DE", "GR", "HU", "IE", "IT", "LV", "LT", "LU", "MT", "NL",
        "PL", "PT", "RO", "SK", "SI", "ES", "SE"
    ];
    EU_COUNTRIES.contains(&country_code.to_uppercase().as_str())
}

/// Assess country risk for responsible sourcing
pub fn assess_country_risk(country_code: &str) -> RiskLevel {
    // High-risk countries for conflict minerals
    const HIGH_RISK: &[&str] = &["CD", "RW", "UG", "SS", "CF"];
    const MEDIUM_RISK: &[&str] = &["CN", "ID", "PH", "MM", "ZM"];

    let code = country_code.to_uppercase();

    if HIGH_RISK.contains(&code.as_str()) {
        RiskLevel::High
    } else if MEDIUM_RISK.contains(&code.as_str()) {
        RiskLevel::Medium
    } else {
        RiskLevel::Low
    }
}

/// Risk level for supply chain
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum RiskLevel {
    Low,
    Medium,
    High,
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_qr_code() {
        assert!(validate_qr_code("WIABAT:ABC123DEF456GHI7"));
        assert!(!validate_qr_code("INVALID:ABC123"));
        assert!(!validate_qr_code("WIABAT:SHORT"));
    }

    #[test]
    fn test_determine_carbon_class() {
        assert_eq!(determine_carbon_class(45.0), CarbonClass::A);
        assert_eq!(determine_carbon_class(55.0), CarbonClass::B);
        assert_eq!(determine_carbon_class(75.0), CarbonClass::C);
        assert_eq!(determine_carbon_class(90.0), CarbonClass::D);
        assert_eq!(determine_carbon_class(100.0), CarbonClass::E);
    }

    #[test]
    fn test_classify_soh() {
        assert_eq!(classify_soh(95.0), SohStatus::Excellent);
        assert_eq!(classify_soh(85.0), SohStatus::Good);
        assert_eq!(classify_soh(75.0), SohStatus::Fair);
        assert_eq!(classify_soh(65.0), SohStatus::Degraded);
        assert_eq!(classify_soh(55.0), SohStatus::Poor);
    }

    #[test]
    fn test_is_eu_country() {
        assert!(is_eu_country("DE"));
        assert!(is_eu_country("FR"));
        assert!(!is_eu_country("US"));
        assert!(!is_eu_country("CN"));
    }

    #[test]
    fn test_assess_country_risk() {
        assert_eq!(assess_country_risk("CD"), RiskLevel::High);
        assert_eq!(assess_country_risk("CN"), RiskLevel::Medium);
        assert_eq!(assess_country_risk("AU"), RiskLevel::Low);
    }
}
