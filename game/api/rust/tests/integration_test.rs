//! WIA Game Integration Tests
//! 弘益人間 - Gaming for Everyone

use wia_game::*;
use std::sync::Arc;

#[test]
fn test_full_profile_lifecycle() {
    let mut controller = GameController::new();

    // Create profile for blind user
    let profile = controller.create_profile_for_disability(DisabilityType::Blind);
    let profile_id = profile.profile_id;

    // Verify blind-friendly settings
    assert!(profile.visual_settings.screen_reader.enabled);
    assert_eq!(profile.visual_settings.screen_reader.verbosity, Verbosity::High);
    assert!(profile.motor_settings.aim_assist.auto_aim);
    assert_eq!(profile.motor_settings.aim_assist.strength, AimAssistStrength::Maximum);

    // Export profile
    let json = controller.export_profile(profile_id).unwrap();
    assert!(json.contains("screen_reader"));

    // Delete original
    controller.delete_profile(profile_id).unwrap();

    // Import profile
    let imported = controller.import_profile(&json).unwrap();
    assert_ne!(imported.profile_id, profile_id); // New ID
    assert!(imported.visual_settings.screen_reader.enabled);
}

#[test]
fn test_preset_system() {
    let controller = GameController::new();

    // Get system presets
    let system_presets = controller.get_system_presets();
    assert!(system_presets.len() >= 5);

    // Find presets by disability
    let deaf_presets = controller.find_presets_for_disability(DisabilityType::Deaf);
    assert!(!deaf_presets.is_empty());

    for preset in &deaf_presets {
        assert!(preset.target_disability.contains(&DisabilityType::Deaf)
            || preset.target_disability.contains(&DisabilityType::HardOfHearing));
    }
}

#[test]
fn test_game_compatibility() {
    let mut controller = GameController::new();

    // Register a game with accessibility features
    let game = GameMetadata {
        game_id: uuid::Uuid::new_v4(),
        title: "Accessible Game".to_string(),
        version: "1.0.0".to_string(),
        accessibility_version: Some("1.0.0".to_string()),
        platforms: vec![Platform::Pc, Platform::Xbox],
        visual_features: VisualFeatures {
            screen_reader_support: true,
            colorblind_modes: vec![ColorblindMode::Deuteranopia, ColorblindMode::Protanopia],
            high_contrast_mode: true,
            ..Default::default()
        },
        audio_features: AudioFeatures {
            subtitle_support: true,
            closed_captions: true,
            visual_sound_indicators: true,
            ..Default::default()
        },
        motor_features: MotorFeatures {
            full_button_remapping: true,
            aim_assist_levels: vec![AimAssistStrength::Medium, AimAssistStrength::High],
            one_handed_modes: vec![Hand::Right],
            adaptive_controller_support: true,
            ..Default::default()
        },
        cognitive_features: CognitiveFeatures {
            multiple_difficulty_levels: true,
            hint_system: true,
            ..Default::default()
        },
        input_support: InputSupport::default(),
        accessibility_rating: None,
    };

    let game_id = controller.register_game(game);

    // Create profile for deaf user
    let profile = controller.create_profile_for_disability(DisabilityType::Deaf);

    // Check compatibility
    let report = controller.get_compatibility(&profile, game_id).unwrap();
    assert!(report.score > 50);
    assert!(!report.supported_features.is_empty());
}

#[test]
fn test_controller_config() {
    use std::collections::HashMap;

    let mut button_mapping = HashMap::new();
    button_mapping.insert(
        "a".to_string(),
        ButtonAction {
            action: "jump".to_string(),
            modifiers: vec![],
            behavior: ButtonBehavior::Press,
            sensitivity: 1.0,
            enabled: true,
        },
    );
    button_mapping.insert(
        "b".to_string(),
        ButtonAction {
            action: "crouch".to_string(),
            modifiers: vec![],
            behavior: ButtonBehavior::Toggle,
            sensitivity: 1.0,
            enabled: true,
        },
    );

    let config = ControllerConfig {
        config_id: Some(uuid::Uuid::new_v4()),
        name: Some("My Config".to_string()),
        controller_type: InputDeviceType::XboxAdaptiveController,
        button_mapping,
        combo_mapping: vec![
            ComboAction {
                buttons: vec!["lb".to_string(), "rb".to_string()],
                action: "ultimate".to_string(),
                simultaneous: true,
                timeout_ms: 500,
            },
        ],
        macro_mapping: vec![],
    };

    // Validate config
    let result = GameController::validate_controller_config(&config);
    assert!(result.is_ok());
}

#[test]
fn test_validation() {
    // Valid profile
    let profile = PlayerProfile::default();
    assert!(GameController::validate_profile(&profile).is_ok());

    // Invalid settings
    let mut invalid_profile = PlayerProfile::default();
    invalid_profile.visual_settings.screen_reader.speed = 10.0; // Out of range
    assert!(GameController::validate_profile(&invalid_profile).is_err());
}

#[tokio::test]
async fn test_storage_adapter() {
    let storage = Arc::new(SimulatorProfileStorage::new());
    let controller = GameController::new().with_storage(storage.clone());

    let profile = PlayerProfile::default();

    // Save profile
    controller.save_profile(&profile).await.unwrap();

    // Verify saved
    assert!(storage.exists(profile.profile_id).await.unwrap());

    // Load profile
    let loaded = storage.load_profile(profile.profile_id).await.unwrap();
    assert_eq!(loaded.profile_id, profile.profile_id);
}

#[tokio::test]
async fn test_input_adapter() {
    let adapter = SimulatorInputAdapter::new();

    // Get devices
    let devices = adapter.get_devices().await.unwrap();
    assert!(!devices.is_empty());

    let standard = devices.iter().find(|d| d.device_type == InputDeviceType::StandardController);
    assert!(standard.is_some());

    let adaptive = devices.iter().find(|d| d.device_type == InputDeviceType::XboxAdaptiveController);
    assert!(adaptive.is_some());

    // Test haptics
    let result = adapter.test_haptics("sim-controller-1", 0.7).await;
    assert!(result.is_ok());
}

#[test]
fn test_disability_profiles() {
    let mut controller = GameController::new();

    // Test all disability types
    let disabilities = vec![
        DisabilityType::Blind,
        DisabilityType::LowVision,
        DisabilityType::Colorblind,
        DisabilityType::Deaf,
        DisabilityType::HardOfHearing,
        DisabilityType::LimitedMobility,
        DisabilityType::OneHanded,
        DisabilityType::Cognitive,
        DisabilityType::Dyslexia,
        DisabilityType::Epilepsy,
    ];

    for disability in disabilities {
        let profile = controller.create_profile_for_disability(disability);
        assert!(profile.disability_context.primary_disabilities.contains(&disability));
    }
}

#[test]
fn test_preset_creation_from_profile() {
    let mut controller = GameController::new();

    let profile = controller.create_profile_for_disability(DisabilityType::LowVision);

    let preset = controller.create_preset_from_profile(
        "My Low Vision Setup".to_string(),
        &profile,
    );

    assert_eq!(preset.name, "My Low Vision Setup");
    assert_eq!(preset.preset_type, PresetType::User);
    assert!(preset.settings_override.visual_settings.is_some());
}

#[test]
fn test_serialization() {
    let profile = PlayerProfile::default();

    // Serialize
    let json = serde_json::to_string_pretty(&profile).unwrap();

    // Deserialize
    let deserialized: PlayerProfile = serde_json::from_str(&json).unwrap();

    assert_eq!(profile.profile_id, deserialized.profile_id);
    assert_eq!(profile.version, deserialized.version);
}
