//! WIA Smart Home Integration Tests
//! 弘益人間 - Benefit All Humanity

use std::collections::HashMap;
use std::sync::Arc;
use wia_smarthome::{
    adapters::{SimulatorDeviceAdapter, SimulatorNotificationService},
    core::DeviceAdapter,
    types::*,
    SmartHomeController,
};

/// Helper to create a controller with simulator adapters
async fn create_test_controller() -> (
    SmartHomeController,
    Arc<SimulatorDeviceAdapter>,
    Arc<SimulatorNotificationService>,
) {
    let device_adapter = Arc::new(SimulatorDeviceAdapter::new().with_delay(0));
    let notification_service = Arc::new(SimulatorNotificationService::new());

    let controller = SmartHomeController::new()
        .with_device_adapter(device_adapter.clone())
        .with_notification_service(notification_service.clone());

    (controller, device_adapter, notification_service)
}

#[tokio::test]
async fn test_full_home_setup() {
    let (controller, device_adapter, _notification_service) = create_test_controller().await;

    // Create user profile with accessibility requirements
    let mut profile = controller.create_profile().await.unwrap();
    profile.personal_info.name = Some("김철수".to_string());
    profile.personal_info.preferred_language = "ko-KR".to_string();
    profile
        .accessibility_requirements
        .primary_disabilities
        .push(DisabilityType::VisualLowVision);
    profile
        .interaction_preferences
        .preferred_input_modalities
        .push(InputModality::Voice);
    profile
        .interaction_preferences
        .preferred_output_modalities
        .push(OutputModality::AudioTts);
    controller.update_profile(profile.clone()).await.unwrap();

    // Create home
    let home = controller
        .create_home("우리 집".to_string(), profile.profile_id)
        .await
        .unwrap();
    assert_eq!(home.name, "우리 집");

    // Create zones
    let living_room = controller
        .create_zone(home.home_id, "거실".to_string(), ZoneType::LivingRoom)
        .await
        .unwrap();
    let bedroom = controller
        .create_zone(home.home_id, "침실".to_string(), ZoneType::Bedroom)
        .await
        .unwrap();

    // Register devices
    let light = controller
        .register_device(
            DeviceType::Light,
            "거실 조명".to_string(),
            home.home_id,
            Some(living_room.zone_id),
        )
        .await
        .unwrap();

    let thermostat = controller
        .register_device(
            DeviceType::Thermostat,
            "에어컨".to_string(),
            home.home_id,
            Some(living_room.zone_id),
        )
        .await
        .unwrap();

    let lock = controller
        .register_device(
            DeviceType::Lock,
            "현관 도어락".to_string(),
            home.home_id,
            None,
        )
        .await
        .unwrap();

    // Add devices to simulator
    device_adapter.add_device(light.device_id).await;
    device_adapter.add_device(thermostat.device_id).await;
    device_adapter.add_device(lock.device_id).await;

    // Verify home structure
    let zones = controller.list_zones(home.home_id).await;
    assert_eq!(zones.len(), 2);

    let devices = controller.list_devices(home.home_id).await;
    assert_eq!(devices.len(), 3);

    let living_room_devices = controller.list_devices_in_zone(living_room.zone_id).await;
    assert_eq!(living_room_devices.len(), 2);

    let bedroom_devices = controller.list_devices_in_zone(bedroom.zone_id).await;
    assert_eq!(bedroom_devices.len(), 0);
}

#[tokio::test]
async fn test_device_control_with_accessibility() {
    let (controller, device_adapter, notification_service) = create_test_controller().await;

    // Create profile with accessibility preferences
    let mut profile = controller.create_profile().await.unwrap();
    profile
        .interaction_preferences
        .preferred_output_modalities
        .push(OutputModality::AudioTts);
    profile
        .interaction_preferences
        .preferred_output_modalities
        .push(OutputModality::Haptic);
    controller.update_profile(profile.clone()).await.unwrap();

    // Create home and device
    let home = controller
        .create_home("Test Home".to_string(), profile.profile_id)
        .await
        .unwrap();

    let mut device = controller
        .register_device(DeviceType::Light, "Smart Light".to_string(), home.home_id, None)
        .await
        .unwrap();

    // Add voice commands with confirmation phrases
    device.accessibility_features.voice_commands.push(VoiceCommand {
        command: "불 켜".to_string(),
        aliases: vec!["조명 켜".to_string(), "라이트 온".to_string()],
        action: "on".to_string(),
        parameters: HashMap::new(),
        confirmation_phrase: Some("조명을 켰습니다".to_string()),
    });
    device.accessibility_features.voice_commands.push(VoiceCommand {
        command: "불 꺼".to_string(),
        aliases: vec!["조명 꺼".to_string(), "라이트 오프".to_string()],
        action: "off".to_string(),
        parameters: HashMap::new(),
        confirmation_phrase: Some("조명을 껐습니다".to_string()),
    });
    // Set device to online and update
    device.status = DeviceStatus::Online;
    controller.update_device(device.clone()).await.unwrap();

    // Add device to simulator
    device_adapter.add_device(device.device_id).await;

    // Control device with accessibility feedback
    controller
        .control_device(
            device.device_id,
            "on",
            HashMap::new(),
            Some(&profile),
        )
        .await
        .unwrap();

    // Verify device state
    let state = device_adapter.get_state(device.device_id).await.unwrap();
    assert_eq!(state.get("power"), Some(&serde_json::json!(true)));

    // Verify accessibility feedback was sent
    let announcements = notification_service.get_announcements().await;
    assert!(!announcements.is_empty());
}

#[tokio::test]
async fn test_automation_with_accessibility() {
    let (controller, device_adapter, notification_service) = create_test_controller().await;

    // Create profile
    let profile = controller.create_profile().await.unwrap();

    // Create home
    let home = controller
        .create_home("Test Home".to_string(), profile.profile_id)
        .await
        .unwrap();

    // Create zone
    let zone = controller
        .create_zone(home.home_id, "Living Room".to_string(), ZoneType::LivingRoom)
        .await
        .unwrap();

    // Create device
    let mut device = controller
        .register_device(
            DeviceType::Light,
            "Living Room Light".to_string(),
            home.home_id,
            Some(zone.zone_id),
        )
        .await
        .unwrap();
    device.status = DeviceStatus::Online;
    controller.update_device(device.clone()).await.unwrap();
    device_adapter.add_device(device.device_id).await;

    // Create automation
    let trigger = Trigger {
        trigger_type: TriggerType::Manual,
        config: HashMap::new(),
    };

    let mut params = HashMap::new();
    params.insert("text".to_string(), serde_json::json!("좋은 아침입니다!"));

    let actions = vec![
        Action {
            action_type: ActionType::DeviceControl,
            device_id: Some(device.device_id),
            command: Some("on".to_string()),
            parameters: HashMap::new(),
            delay_ms: None,
        },
        Action {
            action_type: ActionType::VoiceAnnouncement,
            device_id: None,
            command: None,
            parameters: params,
            delay_ms: None,
        },
    ];

    let mut automation = controller
        .create_automation(
            "아침 루틴".to_string(),
            home.home_id,
            trigger,
            actions,
        )
        .await
        .unwrap();

    // Configure accessibility settings
    automation.accessibility_settings.announce_activation = true;
    automation.accessibility_settings.announcement_text =
        Some("아침 루틴을 시작합니다".to_string());
    automation.zone_ids.push(zone.zone_id);
    controller.update_automation(automation.clone()).await.unwrap();

    // Execute automation
    controller
        .execute_automation(automation.automation_id, Some(&profile))
        .await
        .unwrap();

    // Verify device was turned on
    let state = device_adapter.get_state(device.device_id).await.unwrap();
    assert_eq!(state.get("power"), Some(&serde_json::json!(true)));

    // Verify announcements were made
    let announcements = notification_service.get_announcements().await;
    assert!(announcements.len() >= 2); // Activation + voice announcement
}

#[tokio::test]
async fn test_notification_adaptation() {
    let (controller, _, notification_service) = create_test_controller().await;

    // Create profile with specific accessibility needs
    let mut profile = controller.create_profile().await.unwrap();
    profile.personal_info.preferred_language = "ko-KR".to_string();
    profile
        .interaction_preferences
        .preferred_output_modalities = vec![OutputModality::AudioTts, OutputModality::VisualScreen];
    profile.interaction_preferences.voice_settings.speech_rate = 0.8;
    profile.interaction_preferences.voice_settings.voice_id = Some("ko-female-1".to_string());
    controller.update_profile(profile.clone()).await.unwrap();

    // Create home
    let home = controller
        .create_home("Test Home".to_string(), profile.profile_id)
        .await
        .unwrap();

    // Create notification
    let delivery = NotificationDelivery {
        modalities: vec![
            OutputModality::AudioTts,
            OutputModality::VisualScreen,
            OutputModality::Haptic, // This should be filtered out
        ],
        audio: Some(AudioDelivery {
            tts_text: Some("문이 열렸습니다".to_string()),
            tts_language: "ko-KR".to_string(),
            ..Default::default()
        }),
        visual: Some(VisualDelivery {
            title: Some("보안 알림".to_string()),
            body: Some("현관문이 열렸습니다".to_string()),
            ..Default::default()
        }),
        haptic: None,
    };

    let notification = Notification::new(
        uuid::Uuid::new_v4(),
        NotificationType::Alert,
        NotificationPriority::High,
        "현관문이 열렸습니다".to_string(),
        delivery,
    );

    // Send notification (will be adapted)
    controller
        .send_notification(notification, &[profile.profile_id])
        .await
        .unwrap();

    // Verify notification was sent
    let notifications = notification_service.get_notifications().await;
    assert_eq!(notifications.len(), 1);

    // Verify haptic was filtered out (user doesn't prefer it)
    let sent = &notifications[0].notification;
    assert!(!sent.delivery.modalities.contains(&OutputModality::Haptic));
}

#[tokio::test]
async fn test_voice_command_execution() {
    let (controller, device_adapter, _) = create_test_controller().await;

    // Create profile
    let profile = controller.create_profile().await.unwrap();

    // Create home and device
    let home = controller
        .create_home("Test Home".to_string(), profile.profile_id)
        .await
        .unwrap();

    let mut device = controller
        .register_device(DeviceType::Lock, "Door Lock".to_string(), home.home_id, None)
        .await
        .unwrap();

    // Add voice commands
    device.accessibility_features.voice_commands = vec![
        VoiceCommand {
            command: "문 잠가".to_string(),
            aliases: vec!["잠금".to_string(), "락".to_string()],
            action: "lock".to_string(),
            parameters: HashMap::new(),
            confirmation_phrase: Some("문을 잠갔습니다".to_string()),
        },
        VoiceCommand {
            command: "문 열어".to_string(),
            aliases: vec!["잠금 해제".to_string(), "언락".to_string()],
            action: "unlock".to_string(),
            parameters: HashMap::new(),
            confirmation_phrase: Some("문을 열었습니다".to_string()),
        },
    ];
    device.status = DeviceStatus::Online;
    controller.update_device(device.clone()).await.unwrap();

    // Add to simulator
    device_adapter.add_device(device.device_id).await;

    // Execute voice command using primary command
    controller
        .execute_voice_command(device.device_id, "문 잠가", &profile)
        .await
        .unwrap();

    let state = device_adapter.get_state(device.device_id).await.unwrap();
    assert_eq!(state.get("locked"), Some(&serde_json::json!(true)));

    // Execute voice command using alias
    controller
        .execute_voice_command(device.device_id, "언락", &profile)
        .await
        .unwrap();

    let state = device_adapter.get_state(device.device_id).await.unwrap();
    assert_eq!(state.get("locked"), Some(&serde_json::json!(false)));
}

#[tokio::test]
async fn test_accessibility_event_creation() {
    let source = EventSource {
        source_type: EventSourceType::Device,
        id: uuid::Uuid::new_v4(),
        name: Some("Smart Light".to_string()),
    };

    let mut event = AccessibilityEvent::new(EventType::VoiceCommandExecuted, source);
    event.severity = EventSeverity::Info;
    event.accessibility_context.modalities_used = vec![InputModality::Voice];
    event.accessibility_context.response_time_ms = Some(150);
    event
        .data
        .insert("command".to_string(), serde_json::json!("불 켜"));

    assert_eq!(event.event_type, EventType::VoiceCommandExecuted);
    assert_eq!(event.severity, EventSeverity::Info);
    assert_eq!(
        event.accessibility_context.modalities_used,
        vec![InputModality::Voice]
    );
}

#[tokio::test]
async fn test_error_handling() {
    let (controller, _, _) = create_test_controller().await;

    // Try to get non-existent home
    let result = controller.get_home(uuid::Uuid::new_v4()).await;
    assert!(result.is_err());

    // Try to create zone in non-existent home
    let result = controller
        .create_zone(
            uuid::Uuid::new_v4(),
            "Test Zone".to_string(),
            ZoneType::LivingRoom,
        )
        .await;
    assert!(result.is_err());
}
