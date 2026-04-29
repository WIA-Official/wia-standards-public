# WIA 3D Touch - Rust SDK

弘益人間 (홍익인간) - Touch technology that connects all humanity

## Overview

The WIA 3D Touch Standard provides a comprehensive framework for 3D touch interfaces and haptic feedback systems. This Rust SDK enables developers to integrate advanced touch and haptic capabilities into their applications with precision and accessibility.

## Features

- **3D Touch Detection**: Multi-dimensional touch tracking (X, Y, Z, pressure)
- **Haptic Feedback**: Rich haptic patterns and waveforms
- **Gesture Recognition**: Tap, swipe, pinch, rotate, and custom gestures
- **Force Sensing**: Pressure-sensitive touch input
- **Surface Calibration**: Automatic touch surface calibration
- **Multi-Touch**: Support for multiple simultaneous touch points
- **Device Capabilities**: Query and adapt to device capabilities

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-3d-touch = "1.0.0"
```

## Quick Start

```rust
use wia_3d_touch::{Client, TouchEvent, HapticPattern};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize client
    let client = Client::new("https://api.wia.global", "your-api-key")?;

    // Process touch event
    let touch = TouchEvent::new(150.0, 250.0, 0.6);
    let processed = client.process_touch_event(&touch).await?;

    // Trigger haptic feedback
    let haptic = HapticPattern::click();
    client.trigger_haptic(&haptic).await?;

    Ok(())
}
```

## Touch Events

### Creating Touch Events

```rust
use wia_3d_touch::TouchEvent;

// Simple 2D touch
let touch = TouchEvent::new(100.0, 200.0, 0.5);

// Full 3D touch with all properties
let mut touch = TouchEvent::new(100.0, 200.0, 0.8);
touch.z = 5.0; // Height above surface
touch.touch_type = TouchType::Force;
```

### Processing Touch Events

```rust
// Validate touch event
validate_touch_event(&touch)?;

// Process through API
let processed = client.process_touch_event(&touch).await?;
```

## Haptic Feedback

### Built-in Patterns

```rust
use wia_3d_touch::HapticPattern;

// Simple click
let click = HapticPattern::click();

// Notification
let notification = HapticPattern::notification();

// Custom patterns
let success = create_haptic_pattern("success");
let error = create_haptic_pattern("error");
let button_click = create_haptic_pattern("button_click");
```

### Custom Haptic Patterns

```rust
use wia_3d_touch::{HapticPattern, WaveformType};
use uuid::Uuid;

let custom = HapticPattern {
    id: Uuid::new_v4(),
    name: "custom_vibration".to_string(),
    intensity: 0.7,
    duration_ms: 150,
    frequency_hz: 180.0,
    waveform: WaveformType::Smooth,
    repetitions: 3,
};

client.trigger_haptic(&custom).await?;
```

## Gesture Recognition

### Swipe Detection

```rust
use wia_3d_touch::{TouchEvent, detect_swipe_direction};

let swipe_events = vec![
    TouchEvent::new(100.0, 200.0, 0.5),
    TouchEvent::new(150.0, 200.0, 0.6),
    TouchEvent::new(200.0, 200.0, 0.5),
];

if let Some(direction) = detect_swipe_direction(&swipe_events) {
    println!("Swiped {}", direction); // "left", "right", "up", or "down"
}
```

### Tap Detection

```rust
use wia_3d_touch::is_tap;

let tap_events = vec![
    TouchEvent::new(100.0, 100.0, 0.5),
    TouchEvent::new(101.0, 101.0, 0.6),
];

if is_tap(&tap_events, 200, 5.0) {
    println!("Tap detected!");
}
```

### Advanced Gesture Recognition

```rust
let gesture_result = client.recognize_gesture(&touch_events).await?;

println!("Gesture: {:?}", gesture_result.gesture);
println!("Confidence: {}", gesture_result.confidence);
if let Some(velocity) = gesture_result.velocity {
    println!("Velocity: {} mm/s", velocity.magnitude);
}
```

## Touch Surface Management

### Surface Configuration

```rust
use wia_3d_touch::TouchSurface;

let surface = TouchSurface {
    id: Uuid::new_v4(),
    name: "Main Display".to_string(),
    width_mm: 200.0,
    height_mm: 300.0,
    resolution_dpi: 300,
    max_pressure: 1.0,
    multi_touch_points: 10,
    haptic_enabled: true,
    force_sensing: true,
};

validate_touch_surface(&surface)?;
```

### Calibration

```rust
// Calibrate touch surface
let calibration = client.calibrate_surface(surface.id).await?;

println!("Pressure range: {} - {}",
    calibration.pressure_min,
    calibration.pressure_max
);
println!("Position accuracy: {} mm", calibration.position_accuracy_mm);
```

### Coordinate Normalization

```rust
// Normalize to 0-1 range
let (norm_x, norm_y) = normalize_coordinates(&touch, &surface);

// Convert back to surface coordinates
let (x, y) = denormalize_coordinates(norm_x, norm_y, &surface);
```

## Utility Functions

### Distance Calculation

```rust
let distance = calculate_distance(&point1, &point2);
println!("Distance: {} mm", distance);
```

### Velocity Calculation

```rust
if let Some(velocity) = calculate_velocity(&touch_points) {
    println!("Speed: {} mm/s", velocity.magnitude);
    println!("Direction: ({}, {})", velocity.vx, velocity.vy);
}
```

### Pressure Analysis

```rust
let avg_pressure = calculate_average_pressure(&touch_events);
println!("Average pressure: {}", avg_pressure);
```

### Noise Filtering

```rust
// Filter out touches below threshold
let filtered = filter_noise(&touch_events, 0.3);

// Smooth coordinates using moving average
let smoothed = smooth_coordinates(&touch_events, 5);
```

## Device Capabilities

```rust
let capabilities = client.get_device_capabilities("device-001").await?;

println!("3D Touch: {}", capabilities.supports_3d);
println!("Haptic: {}", capabilities.supports_haptic);
println!("Force Sensing: {}", capabilities.supports_force);
println!("Max Touch Points: {}", capabilities.max_touch_points);
println!("Pressure Levels: {}", capabilities.pressure_levels);
```

## Error Handling

```rust
use wia_3d_touch::{Error, Result};

match client.process_touch_event(&touch).await {
    Ok(processed) => println!("Success: {}", processed.id),
    Err(Error::TouchEventError(msg)) => {
        eprintln!("Touch error: {}", msg);
    }
    Err(Error::HapticError(msg)) => {
        eprintln!("Haptic error: {}", msg);
    }
    Err(e) if e.is_retryable() => {
        eprintln!("Retryable error: {}", e);
        // Implement retry logic
    }
    Err(e) => eprintln!("Error: {}", e),
}
```

## Examples

Run the basic example:

```bash
cargo run --example basic_usage
```

## Testing

Run the test suite:

```bash
cargo test
```

Run tests with output:

```bash
cargo test -- --nocapture
```

## API Documentation

Generate and open API documentation:

```bash
cargo doc --open
```

## Use Cases

### Gaming

- Pressure-sensitive game controls
- Immersive haptic feedback
- Complex gesture recognition

### Accessibility

- Touch assistance for users with disabilities
- Haptic navigation for visually impaired
- Customizable touch sensitivity

### Design & Art

- Pressure-sensitive drawing applications
- 3D sculpting interfaces
- Multi-touch creative tools

### Medical

- Haptic feedback for surgical training
- Touch-based diagnostic tools
- Rehabilitation interfaces

## Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

We believe touch is a universal language that connects all people. Our 3D touch technology aims to:

- Make digital interfaces more accessible
- Provide natural, intuitive interactions
- Support users with different abilities
- Enable new forms of creative expression
- Connect people through haptic communication

## Contributing

Contributions are welcome! Please read our contributing guidelines and submit pull requests to our repository.

## License

MIT License - see LICENSE file for details

## Support

- Documentation: https://docs.wia.global/standards/3d-touch
- API Reference: https://api.wia.global/docs
- Issues: https://github.com/WIA-Official/wia-standards/issues
- Email: standards@wia.global

## Related Standards

- WIA-HAPTIC: Haptic feedback standards
- WIA-GESTURE: Gesture recognition standards
- WIA-ACCESSIBILITY: Accessibility standards

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
