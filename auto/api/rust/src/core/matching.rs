//! Accessibility matching algorithms

use crate::types::*;

/// Result of matching accessibility requirements to vehicle capabilities
#[derive(Debug, Clone)]
pub struct MatchResult {
    /// Overall match (all requirements met)
    pub is_match: bool,
    /// Match score (0-100)
    pub score: u8,
    /// Detailed match information
    pub details: AccessibilityMatch,
    /// Unmet requirements
    pub unmet: Vec<String>,
}

/// Match accessibility requirements against vehicle capabilities
pub fn match_vehicle(
    requirements: &AccessibilityRequirements,
    vehicle: &VehicleCapabilities,
) -> MatchResult {
    let mut score: u32 = 0;
    let mut max_score: u32 = 0;
    let mut unmet = Vec::new();

    // Check wheelchair accessibility
    let wheelchair_ok = if requirements.wheelchair_accessible {
        max_score += 30;
        let has_space = vehicle
            .accessibility_features
            .interior
            .as_ref()
            .map(|i| i.wheelchair_spaces > 0)
            .unwrap_or(false);

        if has_space {
            score += 30;
            true
        } else {
            unmet.push("Wheelchair space not available".to_string());
            false
        }
    } else {
        true
    };

    // Check ramp
    let ramp_ok = if requirements.ramp_required {
        max_score += 20;
        let has_ramp = vehicle
            .accessibility_features
            .entry
            .as_ref()
            .and_then(|e| e.ramp.as_ref())
            .map(|r| r.available)
            .unwrap_or(false);

        if has_ramp {
            score += 20;
            true
        } else {
            unmet.push("Ramp not available".to_string());
            false
        }
    } else {
        true
    };

    // Check lift
    let lift_ok = if requirements.lift_required {
        max_score += 20;
        let has_lift = vehicle
            .accessibility_features
            .entry
            .as_ref()
            .and_then(|e| e.lift.as_ref())
            .map(|l| l.available)
            .unwrap_or(false);

        if has_lift {
            score += 20;
            true
        } else {
            unmet.push("Lift not available".to_string());
            false
        }
    } else {
        true
    };

    // Check service animal
    let service_animal_ok = if requirements.service_animal_space {
        max_score += 10;
        let allows_animals = vehicle
            .capacity
            .as_ref()
            .map(|c| c.service_animals)
            .unwrap_or(false);

        if allows_animals {
            score += 10;
            true
        } else {
            unmet.push("Service animals not allowed".to_string());
            false
        }
    } else {
        true
    };

    // Check modalities
    let modalities_supported = check_modalities(requirements, vehicle);
    if !requirements.preferred_modalities.is_empty() {
        max_score += 20;
        let modality_score = (modalities_supported.len() as f32
            / requirements.preferred_modalities.len() as f32
            * 20.0) as u32;
        score += modality_score;

        if modalities_supported.len() < requirements.preferred_modalities.len() {
            unmet.push("Some preferred modalities not supported".to_string());
        }
    }

    // Calculate final score
    let final_score = if max_score > 0 {
        ((score as f32 / max_score as f32) * 100.0) as u8
    } else {
        100
    };

    let is_match = wheelchair_ok && ramp_ok && lift_ok && service_animal_ok;

    MatchResult {
        is_match,
        score: final_score,
        details: AccessibilityMatch {
            wheelchair_accessible: wheelchair_ok,
            ramp_available: ramp_ok,
            lift_available: lift_ok,
            service_animal_ok,
            modalities_supported,
            match_score: final_score,
        },
        unmet,
    }
}

/// Check which modalities are supported by the vehicle
fn check_modalities(
    requirements: &AccessibilityRequirements,
    vehicle: &VehicleCapabilities,
) -> Vec<InteractionModality> {
    let mut supported = Vec::new();

    let hmi = match &vehicle.accessibility_features.hmi {
        Some(h) => h,
        None => return supported,
    };

    for modality in &requirements.preferred_modalities {
        let is_supported = match modality {
            InteractionModality::VisualScreen => {
                hmi.screen.as_ref().map(|s| s.available).unwrap_or(false)
            }
            InteractionModality::VisualLed => true, // Assume LED is always available
            InteractionModality::AudioTts => hmi.audio.as_ref().map(|a| a.tts).unwrap_or(false),
            InteractionModality::AudioChime => {
                hmi.audio.as_ref().map(|a| a.chimes).unwrap_or(false)
            }
            InteractionModality::AudioSpeechRec => {
                hmi.audio.as_ref().map(|a| a.speech_recognition).unwrap_or(false)
            }
            InteractionModality::HapticVibration => {
                hmi.haptic.as_ref().map(|h| h.vibration).unwrap_or(false)
            }
            InteractionModality::HapticForce => {
                hmi.haptic.as_ref().map(|h| h.force_feedback).unwrap_or(false)
            }
            InteractionModality::Braille => {
                hmi.physical_controls
                    .as_ref()
                    .map(|p| p.braille_labels)
                    .unwrap_or(false)
            }
            InteractionModality::PhysicalButton => {
                hmi.physical_controls
                    .as_ref()
                    .map(|p| p.tactile_buttons)
                    .unwrap_or(false)
            }
        };

        if is_supported {
            supported.push(*modality);
        }
    }

    supported
}

/// Rank vehicles by accessibility match
pub fn rank_vehicles(
    requirements: &AccessibilityRequirements,
    vehicles: &[VehicleCapabilities],
) -> Vec<(VehicleCapabilities, MatchResult)> {
    let mut results: Vec<_> = vehicles
        .iter()
        .map(|v| (v.clone(), match_vehicle(requirements, v)))
        .collect();

    // Sort by: 1) is_match (matching first), 2) score (higher first)
    results.sort_by(|a, b| {
        match (a.1.is_match, b.1.is_match) {
            (true, false) => std::cmp::Ordering::Less,
            (false, true) => std::cmp::Ordering::Greater,
            _ => b.1.score.cmp(&a.1.score),
        }
    });

    results
}

/// Find best matching vehicle
pub fn find_best_match(
    requirements: &AccessibilityRequirements,
    vehicles: &[VehicleCapabilities],
) -> Option<(VehicleCapabilities, MatchResult)> {
    rank_vehicles(requirements, vehicles)
        .into_iter()
        .find(|(_, result)| result.is_match)
}

#[cfg(test)]
mod tests {
    use super::*;
    use uuid::Uuid;

    fn create_wheelchair_accessible_vehicle() -> VehicleCapabilities {
        VehicleCapabilities {
            vehicle_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            vehicle_info: VehicleInfo {
                make: "Test".to_string(),
                model: "AV".to_string(),
                year: Some(2024),
                sae_level: SAELevel::Level4,
                license_plate: None,
            },
            accessibility_features: AccessibilityFeatures {
                entry: Some(EntryFeatures {
                    ramp: Some(RampFeatures {
                        available: true,
                        ramp_type: Some(RampType::Deploy),
                        max_weight_kg: Some(300.0),
                        width_cm: Some(90.0),
                        auto_deploy: true,
                    }),
                    lift: None,
                    doors: Some(DoorFeatures {
                        door_type: Some(DoorType::Slide),
                        width_cm: Some(100.0),
                        auto_open: true,
                        low_step: true,
                        step_height_cm: Some(10.0),
                    }),
                }),
                interior: Some(InteriorFeatures {
                    wheelchair_spaces: 2,
                    wheelchair_securement: Some(SecurementFeatures {
                        available: true,
                        securement_type: Some(SecurementType::FullAuto),
                        max_width_cm: Some(80.0),
                        max_length_cm: Some(130.0),
                        max_weight_kg: Some(300.0),
                    }),
                    transfer_seat: true,
                    lowered_floor: true,
                    floor_height_cm: Some(20.0),
                    headroom_cm: Some(150.0),
                }),
                hmi: Some(HmiFeatures {
                    screen: Some(ScreenFeatures {
                        available: true,
                        size_inches: Some(12.0),
                        touch: true,
                        high_contrast: true,
                        adjustable_brightness: true,
                    }),
                    audio: Some(AudioFeatures {
                        tts: true,
                        speech_recognition: true,
                        languages: vec!["en".to_string(), "ko".to_string()],
                        volume_control: true,
                        chimes: true,
                    }),
                    haptic: Some(HapticFeatures {
                        vibration: true,
                        force_feedback: false,
                    }),
                    physical_controls: Some(PhysicalControlFeatures {
                        braille_labels: true,
                        tactile_buttons: true,
                        panic_button: true,
                        pull_over_button: true,
                    }),
                }),
                communication: Some(CommunicationFeatures {
                    live_support: true,
                    video_call: false,
                    text_chat: true,
                    sign_language_support: false,
                }),
            },
            capacity: Some(VehicleCapacity {
                total_passengers: 4,
                wheelchair_passengers: 2,
                service_animals: true,
            }),
        }
    }

    #[test]
    fn test_match_wheelchair_requirements() {
        let vehicle = create_wheelchair_accessible_vehicle();
        let requirements = AccessibilityRequirements {
            wheelchair_accessible: true,
            ramp_required: true,
            lift_required: false,
            service_animal_space: true,
            companion_space: 1,
            preferred_modalities: vec![
                InteractionModality::AudioTts,
                InteractionModality::HapticVibration,
            ],
        };

        let result = match_vehicle(&requirements, &vehicle);

        assert!(result.is_match);
        assert!(result.score > 90);
        assert!(result.details.wheelchair_accessible);
        assert!(result.details.ramp_available);
        assert!(result.details.service_animal_ok);
    }
}
