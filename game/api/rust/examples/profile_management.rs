//! Profile Management Example
//! 弘益人間 - Gaming for Everyone
//!
//! This example demonstrates advanced profile management.

use wia_game::{GameController, adapters::*, types::*};
use std::sync::Arc;

#[tokio::main]
async fn main() {
    println!("🎮 WIA Game - Profile Management Example\n");

    // ========================================================================
    // 1. Setup with storage adapter
    // ========================================================================
    println!("1. Setting up controller with storage...\n");

    let storage = Arc::new(SimulatorProfileStorage::new());
    let mut controller = GameController::new().with_storage(storage.clone());

    // ========================================================================
    // 2. Create and customize profile
    // ========================================================================
    println!("2. Creating custom profile...\n");

    let mut profile = controller.create_profile();

    // Customize for a player with low vision and motor difficulties
    profile.disability_context.primary_disabilities = vec![
        DisabilityType::LowVision,
        DisabilityType::FineMotorDifficulty,
    ];

    // Visual settings
    profile.visual_settings.magnification.enabled = true;
    profile.visual_settings.magnification.level = 1.5;
    profile.visual_settings.color_settings.high_contrast = true;
    profile.visual_settings.text_settings.size_multiplier = 1.3;
    profile.visual_settings.target_indicators.enemy_highlight = true;

    // Motor settings
    profile.motor_settings.aim_assist.enabled = true;
    profile.motor_settings.aim_assist.strength = AimAssistStrength::Strong;
    profile.motor_settings.button_behavior.hold_to_toggle = vec![
        "aim".to_string(),
        "sprint".to_string(),
    ];
    profile.motor_settings.stick_settings.left_dead_zone = 0.2;
    profile.motor_settings.stick_settings.right_dead_zone = 0.2;

    // Validate
    match GameController::validate_profile(&profile) {
        Ok(_) => println!("   ✅ Profile validated successfully"),
        Err(e) => println!("   ❌ Validation error: {}", e),
    }

    // Update in controller
    controller.update_profile(profile.clone()).unwrap();

    // ========================================================================
    // 3. Save to storage
    // ========================================================================
    println!("\n3. Saving profile to storage...\n");

    controller.save_profile(&profile).await.unwrap();
    println!("   Profile saved: {}", profile.profile_id);

    // Verify it exists
    let exists = storage.exists(profile.profile_id).await.unwrap();
    println!("   Storage verified: {}", exists);

    // ========================================================================
    // 4. Create preset from profile
    // ========================================================================
    println!("\n4. Creating preset from profile...\n");

    let preset = controller.create_preset_from_profile(
        "Low Vision + Motor".to_string(),
        &profile,
    );

    println!("   Created preset: {}", preset.name);
    println!("   Target disabilities: {:?}", preset.target_disability);
    println!("   Preset ID: {}", preset.preset_id);

    // ========================================================================
    // 5. List all profiles
    // ========================================================================
    println!("\n5. Listing all profiles...\n");

    let profiles = controller.list_profiles();
    for p in profiles {
        println!("   - {} ({})", p.profile_id,
            p.disability_context.primary_disabilities
                .iter()
                .map(|d| format!("{:?}", d))
                .collect::<Vec<_>>()
                .join(", "));
    }

    // ========================================================================
    // 6. Game compatibility check
    // ========================================================================
    println!("\n6. Game compatibility check...\n");

    // Register a sample game
    let game = GameMetadata {
        game_id: uuid::Uuid::new_v4(),
        title: "Sample FPS Game".to_string(),
        version: "2.0.0".to_string(),
        accessibility_version: Some("1.0.0".to_string()),
        platforms: vec![Platform::Pc, Platform::Xbox, Platform::Playstation],
        visual_features: VisualFeatures {
            colorblind_modes: vec![ColorblindMode::Deuteranopia],
            high_contrast_mode: true,
            customizable_hud: true,
            ..Default::default()
        },
        audio_features: AudioFeatures {
            subtitle_support: true,
            closed_captions: true,
            visual_sound_indicators: true,
            separate_volume_controls: vec![
                "master".to_string(),
                "music".to_string(),
                "sfx".to_string(),
            ],
            ..Default::default()
        },
        motor_features: MotorFeatures {
            full_button_remapping: true,
            aim_assist_levels: vec![
                AimAssistStrength::Low,
                AimAssistStrength::Medium,
                AimAssistStrength::High,
            ],
            hold_to_toggle_options: true,
            adjustable_dead_zones: true,
            ..Default::default()
        },
        cognitive_features: CognitiveFeatures {
            multiple_difficulty_levels: true,
            objective_markers: true,
            ..Default::default()
        },
        input_support: InputSupport::default(),
        accessibility_rating: None,
    };

    let game_id = controller.register_game(game);

    // Check compatibility
    let report = controller.get_compatibility(&profile, game_id).unwrap();

    println!("   Game: Sample FPS Game");
    println!("   Compatibility Score: {}%", report.score);
    println!("   Supported Features: {:?}", report.supported_features);

    if !report.partial_features.is_empty() {
        println!("   Partial Support: {:?}", report.partial_features);
    }

    if !report.missing_features.is_empty() {
        println!("   Missing Features: {:?}", report.missing_features);
        println!("   Recommendations:");
        for rec in &report.recommendations {
            println!("     - {}", rec);
        }
    }

    // ========================================================================
    // 7. Input device simulation
    // ========================================================================
    println!("\n7. Input device check...\n");

    let input_adapter = SimulatorInputAdapter::new();
    let devices = input_adapter.get_devices().await.unwrap();

    println!("   Connected devices:");
    for device in &devices {
        println!("   - {} ({:?})", device.name, device.device_type);
        println!("     Haptics: {}, Gyro: {}", device.has_haptics, device.has_gyro);
        if let Some(battery) = device.battery_level {
            println!("     Battery: {}%", battery);
        }
    }

    // Test haptics
    println!("\n   Testing haptic feedback...");
    input_adapter.test_haptics("sim-controller-1", 0.5).await.unwrap();

    println!("\n🤟 弘益人間 - Gaming for Everyone\n");
}
