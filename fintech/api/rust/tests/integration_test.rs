//! Integration tests for WIA Fintech API

use wia_fintech::prelude::*;
use wia_fintech::{
    AccessibilityScoreCalculator, CompatibilityChecker,
    NotificationBuilder, UserProfileBuilder, SimulatorAdapter,
};

// ============================================================================
// Profile Management Tests
// ============================================================================

#[tokio::test]
async fn test_create_and_retrieve_profile() {
    let adapter = SimulatorAdapter::new();

    let profile = UserProfileBuilder::new("integration-test-001")
        .preferred_language("ko-KR")
        .region("KR")
        .visual_level(VisualLevel::LowVision)
        .font_size(FontSize::XLarge)
        .high_contrast(true)
        .auth_method(AuthenticationMethod::BiometricFingerprint)
        .build()
        .unwrap();

    // Create profile
    adapter.create_profile(profile.clone()).await.unwrap();

    // Retrieve and verify
    let retrieved = adapter.get_profile("integration-test-001").await.unwrap();
    assert_eq!(retrieved.profile_id, "integration-test-001");
    assert_eq!(retrieved.personal_info.preferred_language, "ko-KR");
}

#[tokio::test]
async fn test_sample_data_profiles() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    let profiles = adapter.list_profiles().await.unwrap();
    assert!(!profiles.is_empty());

    // Check blind user
    let blind_user = adapter.get_profile("user_blind_001").await.unwrap();
    assert!(blind_user.accessibility_needs.sensory.visual.is_some());
}

// ============================================================================
// ATM Management Tests
// ============================================================================

#[tokio::test]
async fn test_find_atms_nearby() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    // Search near San Francisco (sample data location)
    let atms = adapter.find_atms_nearby(37.7749, -122.4194, 50.0).await.unwrap();
    assert!(!atms.is_empty());
}

#[tokio::test]
async fn test_atm_accessibility_scoring() {
    let adapter = SimulatorAdapter::with_sample_data().await;
    let calculator = AccessibilityScoreCalculator::new();

    // Gold ATM should have high scores
    let gold_atm = adapter.get_atm("atm_gold_001").await.unwrap();
    let gold_score = calculator.calculate_atm_score(&gold_atm);
    assert!(gold_score.overall >= 50.0);

    // Non-accessible ATM should have lower scores
    let basic_atm = adapter.get_atm("atm_basic_001").await.unwrap();
    let basic_score = calculator.calculate_atm_score(&basic_atm);
    assert!(basic_score.overall < gold_score.overall);
}

// ============================================================================
// Compatibility Checker Tests
// ============================================================================

#[tokio::test]
async fn test_blind_user_compatibility() {
    let adapter = SimulatorAdapter::with_sample_data().await;
    let checker = CompatibilityChecker::new();

    let blind_user = adapter.get_profile("user_blind_001").await.unwrap();
    let gold_atm = adapter.get_atm("atm_gold_001").await.unwrap();

    let result = checker.check_compatibility(&blind_user, &gold_atm);
    assert!(result.compatibility_score >= 0.0);
}

// ============================================================================
// Notification Builder Tests
// ============================================================================

#[test]
fn test_transaction_notification() {
    let notification = NotificationBuilder::new()
        .notification_type(NotificationType::Transaction)
        .priority(NotificationPriority::Normal)
        .category(NotificationCategory::Financial)
        .title("Transfer Complete")
        .body("Your transfer of ₩100,000 to Kim has been completed.")
        .simple_language("You sent 100,000 won to Kim. Done!")
        .voice_script("Your transfer completed successfully.")
        .build()
        .unwrap();

    assert_eq!(notification.notification_type, NotificationType::Transaction);
    assert_eq!(notification.content.title, "Transfer Complete");
}

#[test]
fn test_fraud_alert_notification() {
    let notification = NotificationBuilder::new()
        .notification_type(NotificationType::FraudAlert)
        .priority(NotificationPriority::Emergency)
        .category(NotificationCategory::Security)
        .title("Suspicious Activity Detected")
        .body("Unusual login attempt from unknown device.")
        .simple_language("Someone tried to log in. Check now!")
        .voice_script("Alert! Suspicious activity detected.")
        .action_required(true)
        .action_type(ActionType::Confirm)
        .action_label("Review Activity")
        .build()
        .unwrap();

    assert_eq!(notification.priority, NotificationPriority::Emergency);
    assert!(notification.action.as_ref().unwrap().required);
}

#[test]
fn test_notification_wia_delivery() {
    let notification = NotificationBuilder::new()
        .notification_type(NotificationType::BalanceAlert)
        .priority(NotificationPriority::High)
        .title("Low Balance Warning")
        .body("Your account balance is below ₩50,000.")
        .exoskeleton_enabled(true)
        .exoskeleton_pattern("double_pulse")
        .exoskeleton_intensity(70.0)
        .bionic_eye_enabled(true)
        .bionic_eye_mode(DisplayMode::Overlay)
        .build()
        .unwrap();

    let wia = notification.wia_delivery.as_ref().unwrap();
    assert!(wia.exoskeleton.as_ref().unwrap().enabled);
    assert!(wia.bionic_eye.as_ref().unwrap().enabled);
}

// ============================================================================
// User Profile Builder Tests
// ============================================================================

#[test]
fn test_comprehensive_profile_building() {
    let profile = UserProfileBuilder::new("comprehensive-001")
        .wia_id("wia-global-12345")
        .preferred_name("김민수")
        .preferred_language("ko-KR")
        .region("KR")
        .timezone("Asia/Seoul")
        .visual_level(VisualLevel::LegallyBlind)
        .font_size(FontSize::XxLarge)
        .high_contrast(true)
        .dark_mode(true)
        .uses_screen_reader(true)
        .screen_reader_type(ScreenReaderType::Talkback)
        .uses_braille_display(true)
        .auth_method(AuthenticationMethod::BiometricVoice)
        .auth_method(AuthenticationMethod::Pin)
        .wia_device("exo-001", WIADeviceType::Exoskeleton)
        .build()
        .unwrap();

    assert_eq!(profile.profile_id, "comprehensive-001");
    assert_eq!(profile.wia_id, Some("wia-global-12345".to_string()));
    assert!(profile.accessibility_needs.sensory.visual.is_some());
    assert_eq!(profile.financial_preferences.preferred_auth_method.len(), 2);
}

#[test]
fn test_profile_validation_fails_without_required_fields() {
    // Missing language
    let result = UserProfileBuilder::new("test-001")
        .region("US")
        .build();

    assert!(result.is_err());

    // Missing region
    let result = UserProfileBuilder::new("test-001")
        .preferred_language("en-US")
        .build();

    assert!(result.is_err());
}
