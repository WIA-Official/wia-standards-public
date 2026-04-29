//! Utility functions for galactic registry

use crate::types::*;

pub fn classify_kardashev(energy_consumption: f64) -> f64 {
    (energy_consumption.log10() - 6.0) / 10.0
}

pub fn estimate_population_from_kardashev(kardashev: f64) -> u64 {
    (10_f64.powf(9.0 + kardashev * 2.0)) as u64
}

pub fn calculate_distance_3d(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    let dz = a.2 - b.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn format_civilization_summary(civ: &Civilization) -> String {
    format!(
        "{} - {} (Kardashev {:.2})",
        civ.name, civ.species, civ.kardashev_scale
    )
}
