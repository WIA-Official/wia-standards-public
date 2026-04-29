//! Validation functions for 3D touch data
//!
//! 弘益人間 - Accurate validation benefits all

use crate::error::{Error, Result};
use crate::types::*;

/// Validate touch event data
pub fn validate_touch_event(event: &TouchEvent) -> Result<()> {
    if event.pressure < 0.0 || event.pressure > 1.0 {
        return Err(Error::TouchEventError("Pressure must be between 0.0 and 1.0".into()));
    }

    if event.z < 0.0 {
        return Err(Error::TouchEventError("Z coordinate cannot be negative".into()));
    }

    Ok(())
}

/// Validate haptic pattern
pub fn validate_haptic_pattern(pattern: &HapticPattern) -> Result<()> {
    if pattern.name.is_empty() {
        return Err(Error::HapticError("Pattern name cannot be empty".into()));
    }

    if pattern.intensity < 0.0 || pattern.intensity > 1.0 {
        return Err(Error::HapticError("Intensity must be between 0.0 and 1.0".into()));
    }

    if pattern.duration_ms == 0 {
        return Err(Error::HapticError("Duration must be greater than 0".into()));
    }

    if pattern.duration_ms > 5000 {
        return Err(Error::HapticError("Duration cannot exceed 5000ms".into()));
    }

    if pattern.frequency_hz < 10.0 || pattern.frequency_hz > 1000.0 {
        return Err(Error::HapticError("Frequency must be between 10 and 1000 Hz".into()));
    }

    if pattern.repetitions > 100 {
        return Err(Error::HapticError("Repetitions cannot exceed 100".into()));
    }

    Ok(())
}

/// Validate touch surface configuration
pub fn validate_touch_surface(surface: &TouchSurface) -> Result<()> {
    if surface.name.is_empty() {
        return Err(Error::InvalidInput("Surface name cannot be empty".into()));
    }

    if surface.width_mm <= 0.0 || surface.height_mm <= 0.0 {
        return Err(Error::InvalidInput("Surface dimensions must be positive".into()));
    }

    if surface.resolution_dpi < 10 || surface.resolution_dpi > 10000 {
        return Err(Error::InvalidInput("Resolution must be between 10 and 10000 DPI".into()));
    }

    if surface.max_pressure <= 0.0 {
        return Err(Error::InvalidInput("Max pressure must be positive".into()));
    }

    if surface.multi_touch_points == 0 {
        return Err(Error::InvalidInput("Must support at least 1 touch point".into()));
    }

    Ok(())
}

/// Validate calibration data
pub fn validate_calibration_data(data: &CalibrationData) -> Result<()> {
    if data.pressure_min < 0.0 || data.pressure_max <= data.pressure_min {
        return Err(Error::CalibrationError("Invalid pressure range".into()));
    }

    if data.position_accuracy_mm < 0.0 {
        return Err(Error::CalibrationError("Position accuracy cannot be negative".into()));
    }

    if data.force_accuracy_n < 0.0 {
        return Err(Error::CalibrationError("Force accuracy cannot be negative".into()));
    }

    if data.touch_points.is_empty() {
        return Err(Error::CalibrationError("Must have at least one calibration point".into()));
    }

    for point in &data.touch_points {
        if point.deviation.abs() > 10.0 {
            return Err(Error::CalibrationError(
                format!("Calibration point deviation too large: {}", point.deviation)
            ));
        }
    }

    Ok(())
}

/// Validate gesture result
pub fn validate_gesture_result(result: &GestureResult) -> Result<()> {
    if result.confidence < 0.0 || result.confidence > 1.0 {
        return Err(Error::GestureError("Confidence must be between 0.0 and 1.0".into()));
    }

    if result.end_time < result.start_time {
        return Err(Error::GestureError("End time must be after start time".into()));
    }

    if result.touch_points.is_empty() {
        return Err(Error::GestureError("Gesture must have at least one touch point".into()));
    }

    Ok(())
}

/// Validate touch coordinates are within surface bounds
pub fn validate_touch_in_bounds(
    event: &TouchEvent,
    surface: &TouchSurface,
) -> Result<()> {
    if event.x < 0.0 || event.x > surface.width_mm {
        return Err(Error::TouchEventError(
            format!("X coordinate {} out of bounds (0-{})", event.x, surface.width_mm)
        ));
    }

    if event.y < 0.0 || event.y > surface.height_mm {
        return Err(Error::TouchEventError(
            format!("Y coordinate {} out of bounds (0-{})", event.y, surface.height_mm)
        ));
    }

    Ok(())
}

/// Validate device capabilities
pub fn validate_device_capabilities(caps: &DeviceCapabilities) -> Result<()> {
    if caps.device_id.is_empty() {
        return Err(Error::DeviceError("Device ID cannot be empty".into()));
    }

    if caps.max_touch_points == 0 {
        return Err(Error::DeviceError("Device must support at least 1 touch point".into()));
    }

    if caps.pressure_levels == 0 {
        return Err(Error::DeviceError("Device must support at least 1 pressure level".into()));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use uuid::Uuid;

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
}
