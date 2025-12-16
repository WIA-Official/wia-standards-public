//! Basic Usage Example for WIA Fintech API
//!
//! Run with: cargo run --example basic_usage

use wia_fintech::prelude::*;
use wia_fintech::{
    NotificationBuilder, SimulatorAdapter, UserProfileBuilder,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Fintech API - Basic Usage Example ===\n");

    // Create a user profile
    println!("Creating user profile...");
    let profile = UserProfileBuilder::new("demo-user-001")
        .preferred_name("Kim Min-jun")
        .preferred_language("ko-KR")
        .region("KR")
        .visual_level(VisualLevel::LowVision)
        .font_size(FontSize::XLarge)
        .high_contrast(true)
        .auth_method(AuthenticationMethod::BiometricFingerprint)
        .wia_device("eye-001", WIADeviceType::BionicEye)
        .build()?;

    println!("Created profile: {}", profile.profile_id);
    println!("Language: {}", profile.personal_info.preferred_language);
    println!();

    // Create adapter with sample data
    let adapter = SimulatorAdapter::with_sample_data().await;

    // Find ATMs nearby
    println!("Finding ATMs near Seoul...");
    let atms = adapter.find_atms_nearby(37.5665, 126.9780, 50.0).await?;
    println!("Found {} ATMs\n", atms.len());

    for atm in &atms {
        println!("ATM: {}", atm.atm_id);
        println!("  Location: {}, {}", atm.location.city, atm.location.country);
        println!(
            "  Audio Guidance: {}, Wheelchair: {}",
            atm.audio_accessibility.audio_guidance,
            atm.physical_accessibility.wheelchair_accessible
        );
        println!();
    }

    // Create a notification
    println!("Building notification...");
    let notification = NotificationBuilder::new()
        .notification_type(NotificationType::Transaction)
        .priority(NotificationPriority::Normal)
        .category(NotificationCategory::Financial)
        .title("Transfer Complete")
        .body("Your transfer of ₩500,000 to Park has been completed.")
        .simple_language("You sent 500,000 won to Park!")
        .voice_script("Your transfer completed successfully.")
        .exoskeleton_enabled(true)
        .exoskeleton_pattern("gentle_pulse")
        .bionic_eye_enabled(true)
        .bionic_eye_mode(DisplayMode::Overlay)
        .build()?;

    println!("Notification created:");
    println!("  Type: {:?}", notification.notification_type);
    println!("  Title: {}", notification.content.title);
    println!();

    println!("=== Example Complete ===");
    Ok(())
}
