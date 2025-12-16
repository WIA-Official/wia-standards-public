//! WIA Smart Home Basic Usage Example
//! 弘益人間 - Benefit All Humanity
//!
//! This example demonstrates basic usage of the WIA Smart Home API
//! with accessibility features.

use std::collections::HashMap;
use std::sync::Arc;
use wia_smarthome::{
    adapters::{SimulatorDeviceAdapter, SimulatorNotificationService},
    core::DeviceAdapter,
    types::*,
    SmartHomeController,
};

#[tokio::main]
async fn main() -> wia_smarthome::Result<()> {
    println!("===========================================");
    println!("  WIA Smart Home API - Basic Usage Example");
    println!("  弘益人間 - Benefit All Humanity");
    println!("===========================================\n");

    // Create adapters
    let device_adapter = Arc::new(SimulatorDeviceAdapter::new().with_delay(50));
    let notification_service = Arc::new(SimulatorNotificationService::new().with_print_output(true));

    // Create controller
    let controller = SmartHomeController::new()
        .with_device_adapter(device_adapter.clone())
        .with_notification_service(notification_service.clone());

    // =========================================================================
    // Step 1: Create User Profile with Accessibility Requirements
    // =========================================================================
    println!("📋 Creating user profile with accessibility settings...\n");

    let mut profile = controller.create_profile().await?;

    // Set personal info
    profile.personal_info.name = Some("김민수".to_string());
    profile.personal_info.preferred_language = "ko-KR".to_string();
    profile.personal_info.timezone = "Asia/Seoul".to_string();

    // Set accessibility requirements (low vision user)
    profile
        .accessibility_requirements
        .primary_disabilities
        .push(DisabilityType::VisualLowVision);
    profile.accessibility_requirements.wcag_level = WcagLevel::AA;
    profile.accessibility_requirements.specific_needs.visual = VisualNeeds {
        screen_reader_required: false,
        magnification_required: true,
        high_contrast_required: true,
        audio_descriptions_required: true,
    };

    // Set interaction preferences
    profile.interaction_preferences.preferred_input_modalities =
        vec![InputModality::Voice, InputModality::Touch];
    profile.interaction_preferences.preferred_output_modalities =
        vec![OutputModality::AudioTts, OutputModality::VisualScreen];

    // Voice settings
    profile.interaction_preferences.voice_settings.wake_word = Some("안녕 집".to_string());
    profile.interaction_preferences.voice_settings.speech_rate = 0.9;

    // Visual settings
    profile.interaction_preferences.visual_settings.high_contrast = true;
    profile.interaction_preferences.visual_settings.large_text = true;
    profile.interaction_preferences.visual_settings.text_scale = 1.5;

    controller.update_profile(profile.clone()).await?;
    println!("✅ Created profile for: {}\n", profile.personal_info.name.as_ref().unwrap());

    // =========================================================================
    // Step 2: Create Home
    // =========================================================================
    println!("🏠 Creating smart home...\n");

    let home = controller
        .create_home("민수네 집".to_string(), profile.profile_id)
        .await?;
    println!("✅ Created home: {}\n", home.name);

    // =========================================================================
    // Step 3: Create Zones
    // =========================================================================
    println!("🏷️ Creating zones...\n");

    let living_room = controller
        .create_zone(home.home_id, "거실".to_string(), ZoneType::LivingRoom)
        .await?;
    let bedroom = controller
        .create_zone(home.home_id, "침실".to_string(), ZoneType::Bedroom)
        .await?;
    let kitchen = controller
        .create_zone(home.home_id, "주방".to_string(), ZoneType::Kitchen)
        .await?;

    println!("✅ Created zones: 거실, 침실, 주방\n");

    // =========================================================================
    // Step 4: Register Devices with Accessibility Features
    // =========================================================================
    println!("💡 Registering devices with accessibility features...\n");

    // Living room light
    let mut light = controller
        .register_device(
            DeviceType::LightDimmer,
            "거실 조명".to_string(),
            home.home_id,
            Some(living_room.zone_id),
        )
        .await?;

    // Add accessibility features
    light.accessibility_features.supported_inputs =
        vec![InputModality::Voice, InputModality::Touch, InputModality::Switch];
    light.accessibility_features.supported_outputs =
        vec![OutputModality::AudioTts, OutputModality::VisualLed];

    // Add voice commands
    light.accessibility_features.voice_commands = vec![
        VoiceCommand {
            command: "거실 불 켜".to_string(),
            aliases: vec!["거실 조명 켜".to_string(), "불 켜줘".to_string()],
            action: "on".to_string(),
            parameters: HashMap::new(),
            confirmation_phrase: Some("거실 조명을 켰습니다".to_string()),
        },
        VoiceCommand {
            command: "거실 불 꺼".to_string(),
            aliases: vec!["거실 조명 꺼".to_string(), "불 꺼줘".to_string()],
            action: "off".to_string(),
            parameters: HashMap::new(),
            confirmation_phrase: Some("거실 조명을 껐습니다".to_string()),
        },
        VoiceCommand {
            command: "밝기 올려".to_string(),
            aliases: vec!["더 밝게".to_string()],
            action: "set_brightness".to_string(),
            parameters: {
                let mut p = HashMap::new();
                p.insert("level".to_string(), serde_json::json!(80));
                p
            },
            confirmation_phrase: Some("밝기를 올렸습니다".to_string()),
        },
    ];

    // Audio feedback
    light.accessibility_features.audio_feedback = AudioFeedback {
        enabled: true,
        volume: 70,
        tts_voice: Some("ko-KR-Wavenet-A".to_string()),
        tones: {
            let mut t = HashMap::new();
            t.insert("on".to_string(), "tone_success".to_string());
            t.insert("off".to_string(), "tone_soft".to_string());
            t
        },
    };

    controller.update_device(light.clone()).await?;

    // Door lock
    let mut lock = controller
        .register_device(
            DeviceType::Lock,
            "현관 도어락".to_string(),
            home.home_id,
            None,
        )
        .await?;

    lock.accessibility_features.voice_commands = vec![
        VoiceCommand {
            command: "문 잠가".to_string(),
            aliases: vec!["잠금".to_string(), "도어락 잠가".to_string()],
            action: "lock".to_string(),
            parameters: HashMap::new(),
            confirmation_phrase: Some("현관문을 잠갔습니다".to_string()),
        },
        VoiceCommand {
            command: "문 열어".to_string(),
            aliases: vec!["잠금 해제".to_string()],
            action: "unlock".to_string(),
            parameters: HashMap::new(),
            confirmation_phrase: Some("현관문 잠금을 해제했습니다".to_string()),
        },
    ];

    controller.update_device(lock.clone()).await?;

    // Thermostat
    let thermostat = controller
        .register_device(
            DeviceType::Thermostat,
            "에어컨".to_string(),
            home.home_id,
            Some(living_room.zone_id),
        )
        .await?;

    // Add devices to simulator
    device_adapter.add_device(light.device_id).await;
    device_adapter.add_device(lock.device_id).await;
    device_adapter.add_device(thermostat.device_id).await;

    println!("✅ Registered devices: 거실 조명, 현관 도어락, 에어컨\n");

    // =========================================================================
    // Step 5: Create Automation
    // =========================================================================
    println!("⚡ Creating automation...\n");

    let trigger = Trigger {
        trigger_type: TriggerType::VoiceCommand,
        config: {
            let mut c = HashMap::new();
            c.insert("command".to_string(), serde_json::json!("좋은 아침"));
            c
        },
    };

    let actions = vec![
        Action {
            action_type: ActionType::DeviceControl,
            device_id: Some(light.device_id),
            command: Some("on".to_string()),
            parameters: HashMap::new(),
            delay_ms: None,
        },
        Action {
            action_type: ActionType::VoiceAnnouncement,
            device_id: None,
            command: None,
            parameters: {
                let mut p = HashMap::new();
                p.insert(
                    "text".to_string(),
                    serde_json::json!("좋은 아침입니다, 민수님! 오늘 날씨는 맑습니다."),
                );
                p
            },
            delay_ms: Some(500),
        },
    ];

    let mut automation = controller
        .create_automation("아침 인사".to_string(), home.home_id, trigger, actions)
        .await?;

    automation.accessibility_settings.announce_activation = true;
    automation.accessibility_settings.announcement_text = Some("아침 루틴을 시작합니다".to_string());
    automation.zone_ids.push(living_room.zone_id);
    automation.schedule.active_days = vec![
        DayOfWeek::Mon,
        DayOfWeek::Tue,
        DayOfWeek::Wed,
        DayOfWeek::Thu,
        DayOfWeek::Fri,
    ];
    automation.schedule.active_start_time = Some("06:00".to_string());
    automation.schedule.active_end_time = Some("10:00".to_string());

    controller.update_automation(automation.clone()).await?;
    println!("✅ Created automation: {}\n", automation.name);

    // =========================================================================
    // Step 6: Execute Commands with Accessibility Feedback
    // =========================================================================
    println!("🎤 Executing voice commands...\n");

    // Execute voice command
    println!("  User: \"거실 불 켜\"");
    controller
        .execute_voice_command(light.device_id, "거실 불 켜", &profile)
        .await?;

    // Check device state
    let state = device_adapter.get_state(light.device_id).await?;
    println!("  Light state: {:?}\n", state);

    // Execute another command
    println!("  User: \"문 잠가\"");
    controller
        .execute_voice_command(lock.device_id, "문 잠가", &profile)
        .await?;

    let state = device_adapter.get_state(lock.device_id).await?;
    println!("  Lock state: {:?}\n", state);

    // =========================================================================
    // Step 7: Execute Automation
    // =========================================================================
    println!("🤖 Executing automation...\n");
    controller
        .execute_automation(automation.automation_id, Some(&profile))
        .await?;

    // =========================================================================
    // Summary
    // =========================================================================
    println!("\n===========================================");
    println!("  Summary");
    println!("===========================================");

    let homes = controller.list_homes().await;
    let zones = controller.list_zones(home.home_id).await;
    let devices = controller.list_devices(home.home_id).await;
    let automations = controller.list_automations(home.home_id).await;

    println!("  Homes: {}", homes.len());
    println!("  Zones: {}", zones.len());
    println!("  Devices: {}", devices.len());
    println!("  Automations: {}", automations.len());

    // Show accessibility summary
    println!("\n  Accessibility Settings:");
    println!(
        "    - Preferred Input: {:?}",
        profile.interaction_preferences.preferred_input_modalities
    );
    println!(
        "    - Preferred Output: {:?}",
        profile.interaction_preferences.preferred_output_modalities
    );
    println!(
        "    - Speech Rate: {}",
        profile.interaction_preferences.voice_settings.speech_rate
    );
    println!(
        "    - High Contrast: {}",
        profile.interaction_preferences.visual_settings.high_contrast
    );

    println!("\n✅ Example completed successfully!");
    println!("弘益人間 - Benefit All Humanity 🤟\n");

    Ok(())
}
