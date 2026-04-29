//! Utility functions for 3D touch operations
//!
//! 弘益人間 - Tools that benefit all users

use crate::types::*;
use chrono::{DateTime, Duration, Utc};

/// Calculate distance between two touch points
pub fn calculate_distance(p1: &TouchPoint, p2: &TouchPoint) -> f64 {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    (dx * dx + dy * dy).sqrt()
}

/// Calculate velocity from touch points
pub fn calculate_velocity(points: &[TouchPoint]) -> Option<Velocity> {
    if points.len() < 2 {
        return None;
    }

    let first = &points[0];
    let last = &points[points.len() - 1];

    let dx = last.x - first.x;
    let dy = last.y - first.y;

    let time_diff = last.timestamp.signed_duration_since(first.timestamp);
    let seconds = time_diff.num_milliseconds() as f64 / 1000.0;

    if seconds == 0.0 {
        return None;
    }

    let vx = dx / seconds;
    let vy = dy / seconds;
    let magnitude = (vx * vx + vy * vy).sqrt();

    Some(Velocity { vx, vy, magnitude })
}

/// Detect swipe direction from touch events
pub fn detect_swipe_direction(events: &[TouchEvent]) -> Option<String> {
    if events.len() < 2 {
        return None;
    }

    let first = &events[0];
    let last = &events[events.len() - 1];

    let dx = last.x - first.x;
    let dy = last.y - first.y;

    let distance = (dx * dx + dy * dy).sqrt();
    if distance < 50.0 {
        return None; // Too small to be a swipe
    }

    if dx.abs() > dy.abs() {
        if dx > 0.0 {
            Some("right".to_string())
        } else {
            Some("left".to_string())
        }
    } else {
        if dy > 0.0 {
            Some("down".to_string())
        } else {
            Some("up".to_string())
        }
    }
}

/// Check if touch is a tap (short duration, minimal movement)
pub fn is_tap(events: &[TouchEvent], max_duration_ms: u64, max_movement_mm: f64) -> bool {
    if events.len() < 2 {
        return false;
    }

    let first = &events[0];
    let last = &events[events.len() - 1];

    let duration = last.timestamp.signed_duration_since(first.timestamp);
    if duration.num_milliseconds() as u64 > max_duration_ms {
        return false;
    }

    let dx = last.x - first.x;
    let dy = last.y - first.y;
    let distance = (dx * dx + dy * dy).sqrt();

    distance <= max_movement_mm
}

/// Calculate average pressure from touch events
pub fn calculate_average_pressure(events: &[TouchEvent]) -> f64 {
    if events.is_empty() {
        return 0.0;
    }

    let sum: f64 = events.iter().map(|e| e.pressure).sum();
    sum / events.len() as f64
}

/// Normalize touch coordinates to 0-1 range
pub fn normalize_coordinates(event: &TouchEvent, surface: &TouchSurface) -> (f64, f64) {
    let norm_x = event.x / surface.width_mm;
    let norm_y = event.y / surface.height_mm;
    (norm_x, norm_y)
}

/// Convert normalized coordinates to surface coordinates
pub fn denormalize_coordinates(norm_x: f64, norm_y: f64, surface: &TouchSurface) -> (f64, f64) {
    let x = norm_x * surface.width_mm;
    let y = norm_y * surface.height_mm;
    (x, y)
}

/// Calculate touch intensity (combination of pressure and area)
pub fn calculate_touch_intensity(pressure: f64, area_sqmm: f64) -> f64 {
    pressure * area_sqmm.sqrt()
}

/// Generate haptic pattern for specific sensation
pub fn create_haptic_pattern(sensation: &str) -> HapticPattern {
    match sensation {
        "button_click" => HapticPattern {
            id: uuid::Uuid::new_v4(),
            name: "button_click".to_string(),
            intensity: 0.4,
            duration_ms: 10,
            frequency_hz: 250.0,
            waveform: WaveformType::Sharp,
            repetitions: 1,
        },
        "success" => HapticPattern {
            id: uuid::Uuid::new_v4(),
            name: "success".to_string(),
            intensity: 0.6,
            duration_ms: 50,
            frequency_hz: 200.0,
            waveform: WaveformType::Smooth,
            repetitions: 1,
        },
        "error" => HapticPattern {
            id: uuid::Uuid::new_v4(),
            name: "error".to_string(),
            intensity: 0.8,
            duration_ms: 30,
            frequency_hz: 150.0,
            waveform: WaveformType::Sharp,
            repetitions: 3,
        },
        "notification" => HapticPattern::notification(),
        _ => HapticPattern::click(),
    }
}

/// Check if two touch events are from the same touch session
pub fn is_same_touch_session(e1: &TouchEvent, e2: &TouchEvent, max_gap_ms: i64) -> bool {
    let duration = e2.timestamp.signed_duration_since(e1.timestamp);
    duration.num_milliseconds().abs() <= max_gap_ms
}

/// Filter out noise from touch events
pub fn filter_noise(events: &[TouchEvent], pressure_threshold: f64) -> Vec<TouchEvent> {
    events.iter()
        .filter(|e| e.pressure >= pressure_threshold)
        .cloned()
        .collect()
}

/// Smooth touch coordinates using moving average
pub fn smooth_coordinates(events: &[TouchEvent], window_size: usize) -> Vec<TouchEvent> {
    if events.len() <= window_size {
        return events.to_vec();
    }

    let mut smoothed = Vec::new();

    for i in 0..events.len() {
        let start = i.saturating_sub(window_size / 2);
        let end = (i + window_size / 2 + 1).min(events.len());
        let window = &events[start..end];

        let avg_x: f64 = window.iter().map(|e| e.x).sum::<f64>() / window.len() as f64;
        let avg_y: f64 = window.iter().map(|e| e.y).sum::<f64>() / window.len() as f64;

        let mut smoothed_event = events[i].clone();
        smoothed_event.x = avg_x;
        smoothed_event.y = avg_y;
        smoothed.push(smoothed_event);
    }

    smoothed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_distance() {
        let p1 = TouchPoint {
            x: 0.0,
            y: 0.0,
            pressure: 0.5,
            timestamp: Utc::now(),
        };
        let p2 = TouchPoint {
            x: 3.0,
            y: 4.0,
            pressure: 0.5,
            timestamp: Utc::now(),
        };

        let distance = calculate_distance(&p1, &p2);
        assert!((distance - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_create_haptic_pattern() {
        let pattern = create_haptic_pattern("button_click");
        assert_eq!(pattern.name, "button_click");
        assert_eq!(pattern.duration_ms, 10);
    }

    #[test]
    fn test_calculate_average_pressure() {
        let events = vec![
            TouchEvent::new(0.0, 0.0, 0.5),
            TouchEvent::new(1.0, 1.0, 0.7),
            TouchEvent::new(2.0, 2.0, 0.6),
        ];

        let avg = calculate_average_pressure(&events);
        assert!((avg - 0.6).abs() < 0.001);
    }
}
