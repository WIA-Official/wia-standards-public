//! Basic usage example for WIA Auto API
//!
//! This example demonstrates:
//! - Creating passenger profiles with builder pattern
//! - Setting up HMI configurations for different accessibility needs
//! - Matching vehicles to accessibility requirements

use wia_auto::prelude::*;

#[tokio::main]
async fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Auto API - Basic Usage Example ===\n");

    // ========================================================================
    // 1. Create a Passenger Profile using Builder Pattern
    // ========================================================================
    println!("1. Creating passenger profile...");

    let profile = PassengerProfile::builder()
        .name("Kim Minji")
        .add_disability(DisabilityType::MobilityWheelchairPower)
        .mobility_aid_with_dimensions(
            MobilityAidType::PowerWheelchair,
            Dimensions {
                width_cm: 68.0,
                length_cm: 120.0,
                height_cm: 100.0,
                weight_kg: 120.0,
            },
            true,  // requires_ramp
            false, // requires_lift
            true,  // requires_securement
        )
        .preferred_language("ko")
        .high_contrast(true)
        .emergency_contact("Lee Bohoja", "010-1234-5678", Some("Caregiver"))
        .build()?;

    println!("   Profile created for: {:?}", profile.passenger.name);
    println!("   Profile ID: {}", profile.profile_id);
    if let Some(ref aid) = profile.mobility_aid {
        if let Some(ref dims) = aid.dimensions {
            println!("   Wheelchair dimensions: {}cm x {}cm", dims.width_cm, dims.length_cm);
        }
    }

    // ========================================================================
    // 2. Create HMI Configuration for Different Needs
    // ========================================================================
    println!("\n2. Creating HMI configurations...");

    // Preset for blind users
    let blind_config = HmiConfig::for_blind();
    println!("   Blind user config:");
    println!("     Visual enabled: {}", blind_config.visual.as_ref().map(|v| v.enabled).unwrap_or(false));
    println!("     Audio enabled: {}", blind_config.audio.as_ref().map(|a| a.enabled).unwrap_or(false));
    println!("     Haptic enabled: {}", blind_config.haptic.as_ref().map(|h| h.enabled).unwrap_or(false));

    // Preset for deaf users
    let deaf_config = HmiConfig::for_deaf();
    println!("   Deaf user config:");
    println!("     Visual enabled: {}", deaf_config.visual.as_ref().map(|v| v.enabled).unwrap_or(false));
    println!("     High contrast: {}", deaf_config.visual.as_ref().map(|v| matches!(v.contrast, ContrastMode::High)).unwrap_or(false));
    println!("     Audio enabled: {}", deaf_config.audio.as_ref().map(|a| a.enabled).unwrap_or(false));

    // Custom configuration using builder
    let custom_config = HmiConfig::builder()
        .visual(VisualConfig {
            enabled: true,
            brightness: 80,
            contrast: ContrastMode::High,
            text_size: TextSize::Large,
            color_scheme: ColorScheme::HighContrast,
            animations: false,
        })
        .audio(AudioConfig {
            enabled: true,
            volume: 70,
            tts_voice: None,
            tts_speed: TtsSpeed::Normal,
            chimes_enabled: true,
            speech_recognition: true,
            language: "ko".to_string(),
        })
        .haptic(HapticConfig {
            enabled: true,
            intensity: 60,
            patterns: None,
        })
        .build();

    println!("   Custom config created: {}", custom_config.config_id);

    // ========================================================================
    // 3. Build Accessibility Requirements
    // ========================================================================
    println!("\n3. Building accessibility requirements...");

    let requirements = AccessibilityRequirements::builder()
        .wheelchair_accessible()
        .requires_ramp()
        .service_animal_space()
        .companion_space(1)
        .add_modality(InteractionModality::AudioTts)
        .add_modality(InteractionModality::HapticVibration)
        .add_modality(InteractionModality::Braille)
        .build();

    println!("   Wheelchair required: {}", requirements.wheelchair_accessible);
    println!("   Ramp required: {}", requirements.ramp_required);
    println!("   Service animal space: {}", requirements.service_animal_space);
    println!("   Companion spaces: {}", requirements.companion_space);
    println!("   Preferred modalities: {:?}", requirements.preferred_modalities);

    // ========================================================================
    // 4. Create Test Vehicles and Match
    // ========================================================================
    println!("\n4. Matching vehicles to requirements...");

    let accessible_vehicle = SimulatorFleetManager::create_sample_wheelchair_av();
    let basic_vehicle = SimulatorFleetManager::create_sample_standard_av();

    let match_result = match_vehicle(&requirements, &accessible_vehicle);
    println!("\n   Accessible Vehicle Match:");
    println!("     Is match: {}", match_result.is_match);
    println!("     Score: {}/100", match_result.score);
    println!("     Wheelchair OK: {}", match_result.details.wheelchair_accessible);
    println!("     Ramp OK: {}", match_result.details.ramp_available);
    println!("     Service Animal OK: {}", match_result.details.service_animal_ok);
    if !match_result.unmet.is_empty() {
        println!("     Unmet: {:?}", match_result.unmet);
    }

    let basic_match = match_vehicle(&requirements, &basic_vehicle);
    println!("\n   Basic Vehicle Match:");
    println!("     Is match: {}", basic_match.is_match);
    println!("     Score: {}/100", basic_match.score);
    println!("     Unmet requirements: {:?}", basic_match.unmet);

    // ========================================================================
    // 5. Rank Multiple Vehicles
    // ========================================================================
    println!("\n5. Ranking vehicles...");

    let vehicles = vec![basic_vehicle, accessible_vehicle];
    let ranked = rank_vehicles(&requirements, &vehicles);

    for (i, (vehicle, result)) in ranked.iter().enumerate() {
        println!("   #{}: {} {} - Score: {}, Match: {}",
            i + 1,
            vehicle.vehicle_info.make,
            vehicle.vehicle_info.model,
            result.score,
            result.is_match
        );
    }

    // ========================================================================
    // 6. Find Best Match
    // ========================================================================
    println!("\n6. Finding best match...");

    let best = find_best_match(&requirements, &vehicles);
    match best {
        Some((vehicle, result)) => {
            println!("   Best match: {} {} (Score: {})",
                vehicle.vehicle_info.make,
                vehicle.vehicle_info.model,
                result.score
            );
        }
        None => {
            println!("   No matching vehicle found!");
        }
    }

    // ========================================================================
    // 7. Using the Simulator
    // ========================================================================
    println!("\n7. Using the simulator...");

    let fleet_sim = SimulatorFleetManager::new();
    let profile_sim = SimulatorProfileManager::new();

    // Register vehicles
    fleet_sim.add_vehicle(SimulatorFleetManager::create_sample_wheelchair_av()).await;
    fleet_sim.add_vehicle(SimulatorFleetManager::create_sample_standard_av()).await;
    println!("   Registered 2 vehicles");

    // Save profile
    profile_sim.save_profile(&profile).await?;
    println!("   Saved passenger profile");

    // Find a matching vehicle
    let location = GeoLocation {
        latitude: 37.5665,
        longitude: 126.9780,
        address: Some("Seoul City Hall".to_string()),
    };

    let found = fleet_sim.find_vehicle(&requirements, &location).await?;
    match found {
        Some(v) => println!("   Found matching vehicle: {} {}", v.vehicle_info.make, v.vehicle_info.model),
        None => println!("   No matching vehicle available"),
    }

    // ========================================================================
    // 8. Create Requirements from Profile
    // ========================================================================
    println!("\n8. Creating requirements from profile...");

    let profile_requirements = AccessibilityRequirements::from_profile(&profile);
    println!("   Wheelchair accessible: {}", profile_requirements.wheelchair_accessible);
    println!("   Ramp required: {}", profile_requirements.ramp_required);

    println!("\n=== Example Complete ===");
    Ok(())
}
