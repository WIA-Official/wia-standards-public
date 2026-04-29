//! Integration tests for 3D Touch SDK
//!
//! 弘益人間 - Quality through testing

use wia_3d_touch::*;
use uuid::Uuid;

#[test]
fn test_touch_event_creation() {
    let event = TouchEvent::new(100.0, 200.0, 0.5);
    assert_eq!(event.x, 100.0);
    assert_eq!(event.y, 200.0);
    assert_eq!(event.pressure, 0.5);
    assert_eq!(event.z, 0.0);
}

#[test]
fn test_haptic_pattern_click() {
    let pattern = HapticPattern::click();
    assert_eq!(pattern.name, "click");
    assert_eq!(pattern.duration_ms, 10);
    assert_eq!(pattern.repetitions, 1);
}

#[test]
fn test_haptic_pattern_notification() {
    let pattern = HapticPattern::notification();
    assert_eq!(pattern.name, "notification");
    assert_eq!(pattern.repetitions, 2);
}

#[test]
fn test_validate_touch_event() {
    let event = TouchEvent::new(100.0, 200.0, 0.5);
    assert!(validate_touch_event(&event).is_ok());
}

#[test]
fn test_validate_touch_event_invalid_pressure() {
    let mut event = TouchEvent::new(100.0, 200.0, 1.5);
    assert!(validate_touch_event(&event).is_err());
}

#[test]
fn test_validate_haptic_pattern() {
    let pattern = HapticPattern::click();
    assert!(validate_haptic_pattern(&pattern).is_ok());
}

#[test]
fn test_validate_haptic_pattern_invalid_intensity() {
    let mut pattern = HapticPattern::click();
    pattern.intensity = 1.5;
    assert!(validate_haptic_pattern(&pattern).is_err());
}

#[test]
fn test_calculate_distance() {
    let p1 = TouchPoint {
        x: 0.0,
        y: 0.0,
        pressure: 0.5,
        timestamp: chrono::Utc::now(),
    };
    let p2 = TouchPoint {
        x: 3.0,
        y: 4.0,
        pressure: 0.5,
        timestamp: chrono::Utc::now(),
    };

    let distance = calculate_distance(&p1, &p2);
    assert!((distance - 5.0).abs() < 0.001);
}

#[test]
fn test_detect_swipe_direction_right() {
    let events = vec![
        TouchEvent::new(0.0, 100.0, 0.5),
        TouchEvent::new(50.0, 100.0, 0.5),
        TouchEvent::new(100.0, 100.0, 0.5),
    ];

    let direction = detect_swipe_direction(&events);
    assert_eq!(direction, Some("right".to_string()));
}

#[test]
fn test_detect_swipe_direction_up() {
    let events = vec![
        TouchEvent::new(100.0, 100.0, 0.5),
        TouchEvent::new(100.0, 50.0, 0.5),
        TouchEvent::new(100.0, 0.0, 0.5),
    ];

    let direction = detect_swipe_direction(&events);
    assert_eq!(direction, Some("up".to_string()));
}

#[test]
fn test_is_tap() {
    let events = vec![
        TouchEvent::new(100.0, 100.0, 0.5),
        TouchEvent::new(101.0, 101.0, 0.6),
    ];

    assert!(is_tap(&events, 200, 5.0));
}

#[test]
fn test_is_not_tap_too_much_movement() {
    let events = vec![
        TouchEvent::new(100.0, 100.0, 0.5),
        TouchEvent::new(200.0, 200.0, 0.5),
    ];

    assert!(!is_tap(&events, 200, 5.0));
}

#[test]
fn test_calculate_average_pressure() {
    let events = vec![
        TouchEvent::new(0.0, 0.0, 0.3),
        TouchEvent::new(0.0, 0.0, 0.5),
        TouchEvent::new(0.0, 0.0, 0.7),
    ];

    let avg = calculate_average_pressure(&events);
    assert!((avg - 0.5).abs() < 0.001);
}

#[test]
fn test_normalize_coordinates() {
    let event = TouchEvent::new(50.0, 100.0, 0.5);
    let surface = TouchSurface {
        id: Uuid::new_v4(),
        name: "Test Surface".to_string(),
        width_mm: 100.0,
        height_mm: 200.0,
        resolution_dpi: 300,
        max_pressure: 1.0,
        multi_touch_points: 10,
        haptic_enabled: true,
        force_sensing: true,
    };

    let (norm_x, norm_y) = normalize_coordinates(&event, &surface);
    assert!((norm_x - 0.5).abs() < 0.001);
    assert!((norm_y - 0.5).abs() < 0.001);
}

#[test]
fn test_create_haptic_pattern() {
    let pattern = create_haptic_pattern("button_click");
    assert_eq!(pattern.name, "button_click");

    let pattern = create_haptic_pattern("success");
    assert_eq!(pattern.name, "success");

    let pattern = create_haptic_pattern("error");
    assert_eq!(pattern.name, "error");
}

#[test]
fn test_filter_noise() {
    let events = vec![
        TouchEvent::new(0.0, 0.0, 0.1),
        TouchEvent::new(1.0, 1.0, 0.5),
        TouchEvent::new(2.0, 2.0, 0.2),
        TouchEvent::new(3.0, 3.0, 0.8),
    ];

    let filtered = filter_noise(&events, 0.3);
    assert_eq!(filtered.len(), 2);
}

#[test]
fn test_client_creation() {
    let client = Client::new("https://api.wia.global", "test-key");
    assert!(client.is_ok());
}

#[test]
fn test_client_empty_api_key() {
    let client = Client::new("https://api.wia.global", "");
    assert!(client.is_err());
}

#[test]
fn test_validate_touch_surface() {
    let surface = TouchSurface {
        id: Uuid::new_v4(),
        name: "Test Surface".to_string(),
        width_mm: 100.0,
        height_mm: 200.0,
        resolution_dpi: 300,
        max_pressure: 1.0,
        multi_touch_points: 10,
        haptic_enabled: true,
        force_sensing: true,
    };

    assert!(validate_touch_surface(&surface).is_ok());
}

#[test]
fn test_validate_device_capabilities() {
    let caps = DeviceCapabilities {
        device_id: "DEV-001".to_string(),
        supports_3d: true,
        supports_haptic: true,
        supports_force: true,
        max_touch_points: 10,
        pressure_levels: 256,
        haptic_actuators: 2,
    };

    assert!(validate_device_capabilities(&caps).is_ok());
}
