//! Utility functions for microplastic detection

use crate::types::*;
use chrono::Utc;
use uuid::Uuid;

/// Calculate average concentration from particles
pub fn calculate_average_concentration(particles: &[MicroplasticParticle]) -> f64 {
    if particles.is_empty() {
        return 0.0;
    }

    let sum: f64 = particles.iter().map(|p| p.concentration).sum();
    sum / particles.len() as f64
}

/// Find dominant plastic type
pub fn find_dominant_type(particles: &[MicroplasticParticle]) -> PlasticType {
    use std::collections::HashMap;

    let mut counts: HashMap<String, usize> = HashMap::new();

    for particle in particles {
        let type_name = format!("{:?}", particle.particle_type);
        *counts.entry(type_name).or_insert(0) += 1;
    }

    counts.iter()
        .max_by_key(|(_, count)| *count)
        .map(|(type_name, _)| {
            if type_name.contains("Polyethylene") {
                PlasticType::Polyethylene
            } else if type_name.contains("Polypropylene") {
                PlasticType::Polypropylene
            } else if type_name.contains("Polystyrene") {
                PlasticType::Polystyrene
            } else if type_name.contains("Pvc") {
                PlasticType::Pvc
            } else if type_name.contains("Pet") {
                PlasticType::Pet
            } else {
                PlasticType::Unknown
            }
        })
        .unwrap_or(PlasticType::Unknown)
}

/// Create analysis result from particles
pub fn create_analysis(particles: Vec<MicroplasticParticle>) -> AnalysisResult {
    AnalysisResult {
        total_particles: particles.len() as u64,
        average_concentration: calculate_average_concentration(&particles),
        dominant_type: find_dominant_type(&particles),
        analyzed_at: Utc::now(),
    }
}

/// Generate unique particle ID
pub fn generate_particle_id() -> Uuid {
    Uuid::new_v4()
}
