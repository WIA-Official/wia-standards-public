//! Basic usage example for WIA 3D Touch SDK
//!
//! 弘益人間 (홍익인간) - Touch that connects all humanity

use wia_3d_touch::{
    Client, TouchEvent, HapticPattern, GestureType,
    create_haptic_pattern, detect_swipe_direction, is_tap,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("👆 WIA 3D Touch SDK Example");
    println!("弘益人間 - Technology that touches all\n");

    // Initialize client
    let client = Client::new(
        "https://api.wia.global",
        std::env::var("WIA_API_KEY").unwrap_or_else(|_| "demo-key".to_string()),
    )?;

    // Example 1: Simple touch event
    println!("📍 Example 1: Processing touch event");
    let touch = TouchEvent::new(150.0, 250.0, 0.6);
    println!("  Touch at ({}, {}) with pressure {}", touch.x, touch.y, touch.pressure);

    // Example 2: Haptic feedback patterns
    println!("\n🔊 Example 2: Haptic feedback");
    let click_pattern = HapticPattern::click();
    println!("  Click: {} intensity, {} Hz", click_pattern.intensity, click_pattern.frequency_hz);

    let notification_pattern = HapticPattern::notification();
    println!("  Notification: {} intensity, {} ms",
        notification_pattern.intensity, notification_pattern.duration_ms);

    let custom_pattern = create_haptic_pattern("success");
    println!("  Success: {} intensity, {} repetitions",
        custom_pattern.intensity, custom_pattern.repetitions);

    // Example 3: Gesture detection
    println!("\n✋ Example 3: Gesture detection");
    let swipe_events = vec![
        TouchEvent::new(100.0, 200.0, 0.5),
        TouchEvent::new(150.0, 200.0, 0.6),
        TouchEvent::new(200.0, 200.0, 0.5),
        TouchEvent::new(250.0, 200.0, 0.4),
    ];

    if let Some(direction) = detect_swipe_direction(&swipe_events) {
        println!("  Detected swipe: {}", direction);
    }

    let tap_events = vec![
        TouchEvent::new(100.0, 100.0, 0.5),
        TouchEvent::new(101.0, 101.0, 0.6),
    ];

    if is_tap(&tap_events, 200, 5.0) {
        println!("  Detected: Tap gesture");
    }

    // Example 4: API interaction (commented for demo)
    /*
    println!("\n🌐 Example 4: API interaction");
    let processed = client.process_touch_event(&touch).await?;
    println!("  Event processed: {}", processed.id);

    client.trigger_haptic(&click_pattern).await?;
    println!("  Haptic feedback triggered");
    */

    println!("\n💡 Demo completed! In production:");
    println!("  1. Process real-time touch events");
    println!("  2. Trigger haptic feedback for user actions");
    println!("  3. Recognize complex gestures");
    println!("  4. Calibrate touch surfaces");
    println!("  5. Monitor device capabilities");
    println!();
    println!("弘益人間 - Touch technology for everyone 🌍");

    Ok(())
}
