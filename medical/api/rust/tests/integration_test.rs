//! Integration tests for WIA Medical Device Accessibility

use wia_medical::prelude::*;

#[tokio::test]
async fn test_simulator_with_sample_data() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    // Test listing profiles
    let device_profiles = adapter.list_device_profiles().await.unwrap();
    assert!(!device_profiles.is_empty());

    let user_profiles = adapter.list_user_profiles().await.unwrap();
    assert!(!user_profiles.is_empty());
}

#[tokio::test]
async fn test_get_cgm_profile() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    let profile = adapter.get_device_profile("cgm_dexcom_g7").await.unwrap();
    assert_eq!(profile.device.manufacturer, "Dexcom");
    assert_eq!(profile.device.model, "G7");
    assert_eq!(profile.device.device_category, DeviceCategory::Cgm);
}

#[tokio::test]
async fn test_get_user_profile() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    let profile = adapter.get_user_profile("user_blind_001").await.unwrap();

    // Verify visual needs
    let sensory = profile.accessibility_needs.sensory.as_ref().unwrap();
    let visual = sensory.visual.as_ref().unwrap();
    assert_eq!(visual.level, VisualLevel::TotallyBlind);
}

#[tokio::test]
async fn test_compatibility_blind_user_with_cgm() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    let device = adapter.get_device_profile("cgm_dexcom_g7").await.unwrap();
    let user = adapter.get_user_profile("user_blind_001").await.unwrap();

    let result = ProfileMatcher::is_compatible(&device, &user);

    // CGM should be compatible with blind user (has voice output)
    assert!(result.compatible);
    assert!(result.score > 50.0);
}

#[tokio::test]
async fn test_compatibility_deaf_user_with_bp_monitor() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    let device = adapter.get_device_profile("bp_talking_monitor").await.unwrap();
    let user = adapter.get_user_profile("user_deaf_001").await.unwrap();

    let result = ProfileMatcher::is_compatible(&device, &user);

    // BP monitor should be compatible with deaf user (has visual/haptic alerts)
    assert!(result.compatible);
}

#[tokio::test]
async fn test_save_and_retrieve_profile() {
    let adapter = SimulatorAdapter::new();

    let profile = DeviceProfileBuilder::new()
        .manufacturer("Test Manufacturer")
        .model("Test Model")
        .device_name("Test Device")
        .device_type(MedicalDeviceType::Monitoring)
        .device_category(DeviceCategory::BloodGlucoseMonitor)
        .build()
        .unwrap();

    // Save profile
    adapter.save_device_profile(&profile).await.unwrap();

    // Retrieve profile
    let retrieved = adapter.get_device_profile(&profile.profile_id).await.unwrap();
    assert_eq!(retrieved.device.manufacturer, "Test Manufacturer");
    assert_eq!(retrieved.device.model, "Test Model");
}

#[tokio::test]
async fn test_delete_profile() {
    let adapter = SimulatorAdapter::new();

    let profile = DeviceProfileBuilder::new()
        .manufacturer("To Delete")
        .model("Delete Me")
        .device_name("Deletable Device")
        .device_type(MedicalDeviceType::Diagnostic)
        .device_category(DeviceCategory::Other)
        .build()
        .unwrap();

    let profile_id = profile.profile_id.clone();

    // Save and delete
    adapter.save_device_profile(&profile).await.unwrap();
    adapter.delete_device_profile(&profile_id).await.unwrap();

    // Should not be found
    let result = adapter.get_device_profile(&profile_id).await;
    assert!(result.is_err());
}

#[tokio::test]
async fn test_accessibility_score_calculation() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    let profile = adapter.get_device_profile("cgm_dexcom_g7").await.unwrap();

    let score = profile.accessibility.accessibility_score.unwrap();
    assert!(score.overall > 0.0);
    assert!(score.visual > 0.0);
    assert!(score.auditory > 0.0);
    assert!(score.motor > 0.0);
    assert!(score.cognitive > 0.0);
}

#[tokio::test]
async fn test_wia_integration() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    let profile = adapter.get_device_profile("cgm_dexcom_g7").await.unwrap();

    let wia = profile.wia_integration.unwrap();
    assert!(wia.supported_protocols.exoskeleton);
    assert!(wia.supported_protocols.voice_sign);
    assert!(wia.supported_protocols.haptic);

    // Check exoskeleton integration
    let exo = wia.exoskeleton.unwrap();
    assert!(!exo.haptic_feedback_mapping.is_empty());

    // Check voice-sign integration
    let voice_sign = wia.voice_sign.unwrap();
    assert!(voice_sign.medical_terminology_support);
}

#[tokio::test]
async fn test_profile_builder_validation() {
    // Should fail without manufacturer
    let result = DeviceProfileBuilder::new()
        .model("Test Model")
        .device_name("Test Device")
        .build();

    assert!(result.is_err());

    // Should fail without model
    let result = DeviceProfileBuilder::new()
        .manufacturer("Test Manufacturer")
        .device_name("Test Device")
        .build();

    assert!(result.is_err());
}

#[tokio::test]
async fn test_user_profile_builder() {
    let profile = UserProfileBuilder::new()
        .visual_needs(VisualLevel::LowVision)
        .color_blind(ColorBlindMode::Deuteranopia)
        .auditory_needs(AuditoryLevel::HardOfHearing)
        .motor_needs(ImpairmentLevel::Mild)
        .primary_input(InputMethod::Voice)
        .visual_preferences(TextSize::Large, true, false)
        .build();

    let sensory = profile.accessibility_needs.sensory.unwrap();
    let visual = sensory.visual.unwrap();
    assert_eq!(visual.level, VisualLevel::LowVision);
    assert_eq!(visual.color_blind, Some(ColorBlindMode::Deuteranopia));

    let auditory = sensory.auditory.unwrap();
    assert_eq!(auditory.level, AuditoryLevel::HardOfHearing);

    assert_eq!(profile.input_preferences.primary_input, InputMethod::Voice);
}

#[tokio::test]
async fn test_compliance_information() {
    let adapter = SimulatorAdapter::with_sample_data().await;

    let profile = adapter.get_device_profile("cgm_dexcom_g7").await.unwrap();

    // FDA compliance
    let fda = profile.compliance.fda.unwrap();
    assert!(fda.registered);
    assert!(fda.human_factors_validated);

    // WIA certification
    let wia_cert = profile.compliance.wia_certification.unwrap();
    assert_eq!(wia_cert.level, CertificationLevel::Gold);

    // International standards
    let intl = profile.compliance.international.unwrap();
    assert!(intl.iec_62366);
    assert!(intl.iso_14971);
}

#[test]
fn test_serialization() {
    let profile = MedicalDeviceAccessibilityProfile::default();

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&profile).unwrap();
    assert!(!json.is_empty());

    // Deserialize back
    let deserialized: MedicalDeviceAccessibilityProfile = serde_json::from_str(&json).unwrap();
    assert_eq!(deserialized.profile_id, profile.profile_id);
}

#[test]
fn test_user_profile_serialization() {
    let profile = UserMedicalAccessibilityProfile::default();

    let json = serde_json::to_string_pretty(&profile).unwrap();
    let deserialized: UserMedicalAccessibilityProfile = serde_json::from_str(&json).unwrap();

    assert_eq!(deserialized.user_id, profile.user_id);
}
