//! Basic usage example for WIA Medical Device Accessibility
//!
//! This example demonstrates:
//! - Creating and managing device profiles
//! - Creating user accessibility profiles
//! - Checking device-user compatibility
//! - Calculating accessibility scores

use wia_medical::prelude::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Medical Device Accessibility ===\n");
    println!("Version: {}", wia_medical::VERSION);
    println!("Standard: {}\n", wia_medical::STANDARD_VERSION);

    // Create a simulator with sample data
    let adapter = SimulatorAdapter::with_sample_data().await;

    // List available device profiles
    println!("--- Available Device Profiles ---");
    let device_ids = adapter.list_device_profiles().await?;
    for id in &device_ids {
        let profile = adapter.get_device_profile(id).await?;
        println!(
            "  - {} ({} {})",
            profile.device.device_name,
            profile.device.manufacturer,
            profile.device.model
        );
    }
    println!();

    // List available user profiles
    println!("--- Available User Profiles ---");
    let user_ids = adapter.list_user_profiles().await?;
    for id in &user_ids {
        let profile = adapter.get_user_profile(id).await?;
        let needs = describe_user_needs(&profile);
        println!("  - {} ({})", id, needs);
    }
    println!();

    // Get CGM device profile
    let cgm = adapter.get_device_profile("cgm_dexcom_g7").await?;
    println!("--- Device: {} ---", cgm.device.device_name);
    println!("  Manufacturer: {}", cgm.device.manufacturer);
    println!("  Model: {}", cgm.device.model);
    println!("  Type: {:?}", cgm.device.device_type);
    println!("  Category: {:?}", cgm.device.device_category);
    println!("  Use Environment: {:?}", cgm.device.use_environment);

    // Show accessibility score
    if let Some(ref score) = cgm.accessibility.accessibility_score {
        println!("\n  Accessibility Scores:");
        println!("    Overall: {:.1}%", score.overall);
        println!("    Visual: {:.1}%", score.visual);
        println!("    Auditory: {:.1}%", score.auditory);
        println!("    Motor: {:.1}%", score.motor);
        println!("    Cognitive: {:.1}%", score.cognitive);
    }

    // Show WIA certification
    if let Some(ref cert) = cgm.compliance.wia_certification {
        println!("\n  WIA Certification:");
        println!("    Level: {:?}", cert.level);
        println!("    Certificate: {}", cert.certificate_id);
        println!("    Valid Until: {}", cert.valid_until);
    }
    println!();

    // Check compatibility with blind user
    let blind_user = adapter.get_user_profile("user_blind_001").await?;
    println!("--- Compatibility Check ---");
    println!("Device: {}", cgm.device.device_name);
    println!("User: user_blind_001 (Totally Blind)");

    let result = ProfileMatcher::is_compatible(&cgm, &blind_user);
    println!("\nResult:");
    println!("  Compatible: {}", if result.compatible { "Yes" } else { "No" });
    println!("  Score: {:.1}%", result.score);

    if !result.issues.is_empty() {
        println!("  Issues:");
        for issue in &result.issues {
            println!(
                "    [{:?}] {:?}: {}",
                issue.severity, issue.category, issue.description
            );
            println!("      Recommendation: {}", issue.recommendation);
        }
    }
    println!();

    // Check compatibility with deaf user
    let deaf_user = adapter.get_user_profile("user_deaf_001").await?;
    println!("--- Compatibility Check ---");
    println!("Device: {}", cgm.device.device_name);
    println!("User: user_deaf_001 (Deaf)");

    let result = ProfileMatcher::is_compatible(&cgm, &deaf_user);
    println!("\nResult:");
    println!("  Compatible: {}", if result.compatible { "Yes" } else { "No" });
    println!("  Score: {:.1}%", result.score);

    if !result.issues.is_empty() {
        println!("  Issues:");
        for issue in &result.issues {
            println!(
                "    [{:?}] {:?}: {}",
                issue.severity, issue.category, issue.description
            );
        }
    }
    println!();

    // Build a new device profile
    println!("--- Building Custom Device Profile ---");
    let custom_device = DeviceProfileBuilder::new()
        .manufacturer("WIA Medical")
        .model("AccessibleBGM-100")
        .device_name("WIA Accessible Blood Glucose Monitor")
        .device_type(MedicalDeviceType::Monitoring)
        .device_category(DeviceCategory::BloodGlucoseMonitor)
        .with_voice_output(vec!["en".to_string(), "ko".to_string(), "ja".to_string()])
        .with_haptic_feedback()
        .build()?;

    println!("Created profile: {}", custom_device.profile_id);
    println!("  Device: {}", custom_device.device.device_name);
    if let Some(ref score) = custom_device.accessibility.accessibility_score {
        println!("  Accessibility Score: {:.1}%", score.overall);
    }
    println!();

    // Build a user profile
    println!("--- Building Custom User Profile ---");
    let custom_user = UserProfileBuilder::new()
        .visual_needs(VisualLevel::LowVision)
        .color_blind(ColorBlindMode::Deuteranopia)
        .primary_input(InputMethod::Voice)
        .visual_preferences(TextSize::ExtraLarge, true, true)
        .build();

    println!("Created user profile: {}", custom_user.user_id);
    println!("  Primary Input: {:?}", custom_user.input_preferences.primary_input);
    println!();

    // Check compatibility with custom profiles
    println!("--- Custom Profile Compatibility ---");
    let result = ProfileMatcher::is_compatible(&custom_device, &custom_user);
    println!("Compatible: {}", if result.compatible { "Yes" } else { "No" });
    println!("Score: {:.1}%", result.score);

    println!("\n=== Demo Complete ===");
    println!("弘益人間 - Benefit All Humanity");

    Ok(())
}

fn describe_user_needs(profile: &UserMedicalAccessibilityProfile) -> String {
    let mut needs = Vec::new();

    if let Some(ref sensory) = profile.accessibility_needs.sensory {
        if let Some(ref visual) = sensory.visual {
            match visual.level {
                VisualLevel::TotallyBlind => needs.push("Blind"),
                VisualLevel::LegallyBlind => needs.push("Legally Blind"),
                VisualLevel::LowVision => needs.push("Low Vision"),
                VisualLevel::None => {}
            }
        }
        if let Some(ref auditory) = sensory.auditory {
            match auditory.level {
                AuditoryLevel::Deaf => needs.push("Deaf"),
                AuditoryLevel::HardOfHearing => needs.push("Hard of Hearing"),
                AuditoryLevel::None => {}
            }
        }
    }

    if let Some(ref motor) = profile.accessibility_needs.motor {
        if let Some(ref upper_limb) = motor.upper_limb {
            if upper_limb.level != ImpairmentLevel::None {
                needs.push("Motor Impairment");
            }
        }
    }

    if needs.is_empty() {
        "No specific needs".to_string()
    } else {
        needs.join(", ")
    }
}
