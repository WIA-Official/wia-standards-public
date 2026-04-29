//! Integration tests for WIA-HOME SDK
//!
//! 弘益人間 - Ensuring quality for humanity

use wia_home::*;
use uuid::Uuid;
use chrono::Utc;

#[test]
fn test_device_type_serialization() {
    let device_type = DeviceType::Light;
    let json = serde_json::to_string(&device_type).unwrap();
    assert_eq!(json, "\"light\"");

    let custom = DeviceType::Custom("smart_fan".to_string());
    let json = serde_json::to_string(&custom).unwrap();
    assert!(json.contains("smart_fan"));
}

#[test]
fn test_device_status() {
    let status = DeviceStatus::Online;
    assert_eq!(status, DeviceStatus::Online);
    assert_ne!(status, DeviceStatus::Offline);
}

#[test]
fn test_device_creation() {
    let device = Device {
        id: Uuid::new_v4(),
        name: "Test Light".to_string(),
        device_type: DeviceType::Light,
        status: DeviceStatus::Online,
        room: Some("Living Room".to_string()),
        capabilities: vec!["on_off".to_string(), "brightness".to_string()],
        metadata: serde_json::json!({}),
        created_at: Utc::now(),
        updated_at: Utc::now(),
    };

    assert_eq!(device.name, "Test Light");
    assert_eq!(device.device_type, DeviceType::Light);
    assert!(device.room.is_some());
}

#[test]
fn test_room_creation() {
    let room = Room {
        id: Uuid::new_v4(),
        name: "Kitchen".to_string(),
        floor: Some(1),
        devices: vec![Uuid::new_v4(), Uuid::new_v4()],
        metadata: serde_json::json!({"size": "large"}),
    };

    assert_eq!(room.name, "Kitchen");
    assert_eq!(room.devices.len(), 2);
    assert_eq!(room.floor, Some(1));
}

#[test]
fn test_scene_creation() {
    let scene = Scene {
        id: Uuid::new_v4(),
        name: "Movie Time".to_string(),
        description: Some("Dim lights and close blinds".to_string()),
        actions: vec![
            SceneAction {
                device_id: Uuid::new_v4(),
                action: "dim".to_string(),
                parameters: serde_json::json!({"level": 20}),
                delay_seconds: None,
            },
        ],
        triggers: vec![
            SceneTrigger {
                trigger_type: TriggerType::Manual,
                condition: serde_json::json!({}),
            },
        ],
        enabled: true,
        created_at: Utc::now(),
    };

    assert_eq!(scene.name, "Movie Time");
    assert_eq!(scene.actions.len(), 1);
    assert!(scene.enabled);
}

#[test]
fn test_energy_data() {
    let energy = EnergyData {
        device_id: Uuid::new_v4(),
        timestamp: Utc::now(),
        consumption_kwh: 2.5,
        cost: Some(0.30),
        currency: Some("USD".to_string()),
    };

    assert_eq!(energy.consumption_kwh, 2.5);
    assert_eq!(energy.cost, Some(0.30));
}

#[test]
fn test_home_config() {
    let config = HomeConfig {
        home_id: Uuid::new_v4(),
        name: "My Smart Home".to_string(),
        address: Some("123 Main St".to_string()),
        timezone: "America/New_York".to_string(),
        rooms: vec![],
        metadata: serde_json::json!({}),
    };

    assert_eq!(config.name, "My Smart Home");
    assert_eq!(config.timezone, "America/New_York");
}

#[test]
fn test_api_response() {
    let response: ApiResponse<String> = ApiResponse {
        success: true,
        data: Some("test".to_string()),
        error: None,
        timestamp: Utc::now(),
    };

    assert!(response.success);
    assert!(response.data.is_some());
    assert!(response.error.is_none());
}

#[test]
fn test_error_types() {
    let error = HomeError::DeviceNotFound("device-123".to_string());
    assert_eq!(error.error_code(), "DEVICE_NOT_FOUND");
    assert!(!error.is_retryable());

    let timeout_error = HomeError::Timeout;
    assert!(timeout_error.is_retryable());
    assert_eq!(timeout_error.error_code(), "TIMEOUT");
}

#[test]
fn test_validators() {
    // Valid cases
    assert!(validators::validate_url("https://api.wia-home.org").is_ok());
    assert!(validators::validate_api_key("1234567890abcdef").is_ok());
    assert!(validators::validate_device_name("Kitchen Light").is_ok());
    assert!(validators::validate_energy_value(100.0).is_ok());

    // Invalid cases
    assert!(validators::validate_url("").is_err());
    assert!(validators::validate_api_key("short").is_err());
    assert!(validators::validate_device_name("").is_err());
    assert!(validators::validate_energy_value(-1.0).is_err());
}

#[test]
fn test_utils() {
    let device = Device {
        id: Uuid::new_v4(),
        name: "Test Device".to_string(),
        device_type: DeviceType::Light,
        status: DeviceStatus::Online,
        room: Some("Bedroom".to_string()),
        capabilities: vec![],
        metadata: serde_json::json!({}),
        created_at: Utc::now(),
        updated_at: Utc::now(),
    };

    let formatted = utils::format_device_name(&device);
    assert!(formatted.contains("Test Device"));
    assert!(formatted.contains("Bedroom"));

    assert!(utils::is_device_online(&device));

    assert_eq!(utils::format_energy(0.5), "500.00 Wh");
    assert_eq!(utils::parse_device_type("light"), DeviceType::Light);
}

#[test]
fn test_filter_devices() {
    let devices = vec![
        Device {
            id: Uuid::new_v4(),
            name: "Light 1".to_string(),
            device_type: DeviceType::Light,
            status: DeviceStatus::Online,
            room: Some("Living Room".to_string()),
            capabilities: vec![],
            metadata: serde_json::json!({}),
            created_at: Utc::now(),
            updated_at: Utc::now(),
        },
        Device {
            id: Uuid::new_v4(),
            name: "Camera 1".to_string(),
            device_type: DeviceType::Camera,
            status: DeviceStatus::Offline,
            room: Some("Living Room".to_string()),
            capabilities: vec![],
            metadata: serde_json::json!({}),
            created_at: Utc::now(),
            updated_at: Utc::now(),
        },
    ];

    let lights = utils::filter_devices_by_type(&devices, &DeviceType::Light);
    assert_eq!(lights.len(), 1);

    let living_room = utils::filter_devices_by_room(&devices, "Living Room");
    assert_eq!(living_room.len(), 2);

    let offline = utils::get_offline_devices(&devices);
    assert_eq!(offline.len(), 1);
}
