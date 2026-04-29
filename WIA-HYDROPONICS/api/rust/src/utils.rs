//! Utility functions for WIA-HYDROPONICS SDK
//!
//! 弘益人間 - Helpful tools for sustainable agriculture

use crate::types::*;
use chrono::{DateTime, Utc, Duration};
use uuid::Uuid;

/// Generate a new system ID
pub fn generate_system_id() -> Uuid {
    Uuid::new_v4()
}

/// Generate a new plant ID
pub fn generate_plant_id() -> Uuid {
    Uuid::new_v4()
}

/// Calculate optimal pH range for plant species
pub fn optimal_ph_range(species: &str) -> (f64, f64) {
    match species.to_lowercase().as_str() {
        "lettuce" | "spinach" | "kale" => (5.5, 6.5),
        "tomato" | "pepper" | "cucumber" => (5.5, 6.5),
        "strawberry" => (5.5, 6.0),
        "basil" => (5.5, 6.5),
        _ => (5.5, 6.5), // Default range
    }
}

/// Calculate optimal EC range for plant species
pub fn optimal_ec_range(species: &str) -> (f64, f64) {
    match species.to_lowercase().as_str() {
        "lettuce" => (0.8, 1.2),
        "tomato" => (2.0, 3.5),
        "cucumber" => (1.7, 2.5),
        "strawberry" => (1.0, 1.5),
        "basil" => (1.0, 1.6),
        _ => (1.5, 2.5), // Default range
    }
}

/// Calculate days until harvest
pub fn days_until_harvest(plant: &Plant) -> Option<i64> {
    plant.expected_harvest.map(|harvest_date| {
        (harvest_date - Utc::now()).num_days()
    })
}

/// Check if pH is within optimal range
pub fn is_ph_optimal(species: &str, current_ph: f64) -> bool {
    let (min, max) = optimal_ph_range(species);
    current_ph >= min && current_ph <= max
}

/// Check if EC is within optimal range
pub fn is_ec_optimal(species: &str, current_ec: f64) -> bool {
    let (min, max) = optimal_ec_range(species);
    current_ec >= min && current_ec <= max
}

/// Calculate average environment values
pub fn calculate_average_environment(data: &[EnvironmentData]) -> Option<EnvironmentData> {
    if data.is_empty() {
        return None;
    }

    let count = data.len() as f64;
    Some(EnvironmentData {
        system_id: data[0].system_id,
        timestamp: Utc::now(),
        temperature_celsius: data.iter().map(|d| d.temperature_celsius).sum::<f64>() / count,
        humidity_percent: data.iter().map(|d| d.humidity_percent).sum::<f64>() / count,
        light_intensity_lux: data.iter().map(|d| d.light_intensity_lux).sum::<f64>() / count,
        co2_ppm: None,
    })
}

/// Calculate average nutrient values
pub fn calculate_average_nutrients(data: &[NutrientData]) -> Option<NutrientData> {
    if data.is_empty() {
        return None;
    }

    let count = data.len() as f64;
    Some(NutrientData {
        system_id: data[0].system_id,
        timestamp: Utc::now(),
        ph_level: data.iter().map(|d| d.ph_level).sum::<f64>() / count,
        ec_level: data.iter().map(|d| d.ec_level).sum::<f64>() / count,
        temperature_celsius: data.iter().map(|d| d.temperature_celsius).sum::<f64>() / count,
        dissolved_oxygen: None,
    })
}

/// Filter plants by growth stage
pub fn filter_plants_by_stage(plants: &[Plant], stage: &GrowthStage) -> Vec<Plant> {
    plants.iter().filter(|p| &p.growth_stage == stage).cloned().collect()
}

/// Get plants ready for harvest
pub fn get_harvestable_plants(plants: &[Plant]) -> Vec<Plant> {
    plants.iter().filter(|p| {
        if let Some(harvest_date) = p.expected_harvest {
            harvest_date <= Utc::now() || p.growth_stage == GrowthStage::Harvest
        } else {
            false
        }
    }).cloned().collect()
}

/// Filter critical alerts
pub fn filter_critical_alerts(alerts: &[SystemAlert]) -> Vec<SystemAlert> {
    alerts.iter()
        .filter(|a| a.severity == AlertSeverity::Critical && a.resolved_at.is_none())
        .cloned()
        .collect()
}

/// Calculate system health score
pub fn calculate_system_health(
    plants: &[Plant],
    alerts: &[SystemAlert],
) -> f64 {
    if plants.is_empty() {
        return 100.0;
    }

    let avg_plant_health: f64 = plants.iter().map(|p| p.health_score).sum::<f64>() / plants.len() as f64;

    let critical_alerts = filter_critical_alerts(alerts).len() as f64;
    let alert_penalty = (critical_alerts * 10.0).min(30.0);

    (avg_plant_health - alert_penalty).max(0.0)
}

/// Format system status
pub fn format_system_status(system: &HydroponicSystem) -> String {
    format!(
        "{} - {:?} ({} plants, {:.1}L capacity)",
        system.name, system.status, system.plant_count, system.capacity_liters
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_optimal_ranges() {
        let (min_ph, max_ph) = optimal_ph_range("lettuce");
        assert_eq!((min_ph, max_ph), (5.5, 6.5));

        let (min_ec, max_ec) = optimal_ec_range("tomato");
        assert_eq!((min_ec, max_ec), (2.0, 3.5));
    }

    #[test]
    fn test_is_optimal() {
        assert!(is_ph_optimal("lettuce", 6.0));
        assert!(!is_ph_optimal("lettuce", 7.5));

        assert!(is_ec_optimal("lettuce", 1.0));
        assert!(!is_ec_optimal("lettuce", 3.0));
    }
}
