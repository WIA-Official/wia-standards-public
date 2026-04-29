//! Utility functions for art authentication

use crate::types::*;

pub fn calculate_authenticity_score(artwork: &Artwork) -> f64 {
    let mut score = 0.0;

    if artwork.verified {
        score += 50.0;
    }

    if !artwork.provenance.is_empty() {
        score += 30.0;
    }

    if !artwork.certificate_hash.is_empty() {
        score += 20.0;
    }

    score
}

pub fn format_provenance(records: &[ProvenanceRecord]) -> String {
    records.iter()
        .map(|r| format!("{} → {}", r.previous_owner, r.current_owner))
        .collect::<Vec<_>>()
        .join(" → ")
}
