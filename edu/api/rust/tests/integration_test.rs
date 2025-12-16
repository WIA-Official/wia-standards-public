//! WIA Education Integration Tests
//! 弘益人間 - Education for Everyone

use wia_edu::*;
use uuid::Uuid;

#[test]
fn test_full_learner_workflow() {
    let mut controller = EduController::new();

    // Create profiles for different disabilities
    let blind_profile = controller.create_profile_for_disability(DisabilityType::Blind);
    let deaf_profile = controller.create_profile_for_disability(DisabilityType::Deaf);
    let dyslexic_profile = controller.create_profile_for_disability(DisabilityType::Dyslexia);

    // Verify appropriate settings
    assert!(blind_profile.display_preferences.screen_reader.enabled);
    assert!(deaf_profile.content_preferences.captions.required);
    assert_eq!(
        dyslexic_profile.display_preferences.text_settings.font_family,
        Some(FontFamily::Opendyslexic)
    );

    // Export and import
    let json = controller.export_profile(blind_profile.profile_id).unwrap();
    let imported = controller.import_profile(&json).unwrap();

    assert!(imported.display_preferences.screen_reader.enabled);
    assert_ne!(imported.profile_id, blind_profile.profile_id); // New ID
}

#[test]
fn test_full_course_workflow() {
    let mut controller = EduController::new();

    // Create course
    let course = controller.create_course("Introduction to Accessibility".to_string());
    let course_id = course.course_id;

    // Set accessibility statement
    controller.set_accessibility_statement(
        course_id,
        AccessibilityStatement {
            commitment: Some("We are committed to making this course accessible.".to_string()),
            wcag_conformance: Some(WCAGLevel::AA),
            conformance_date: Some("2025-01-01".to_string()),
            known_issues: Vec::new(),
            contact: Some(ContactInfo {
                name: Some("Accessibility Office".to_string()),
                email: Some("accessibility@example.edu".to_string()),
                phone: None,
            }),
            accommodation_request_process: Some(
                "Contact the accessibility office before the semester starts.".to_string(),
            ),
        },
    ).unwrap();

    // Add modules
    let module1 = controller.add_module(course_id, "Introduction".to_string()).unwrap();
    let _module2 = controller.add_module(course_id, "Visual Accessibility".to_string()).unwrap();

    // Add content to module
    let video_content = ContentItem {
        content_id: Uuid::new_v4(),
        content_type: ContentType::Video,
        title: "What is Accessibility?".to_string(),
        description: Some("Introduction video".to_string()),
        sequence: Some(1),
        required: true,
        duration_minutes: Some(15),
        url: Some("https://example.com/video.mp4".to_string()),
        accessibility_metadata: ContentAccessibilityMetadata {
            has_captions: true,
            has_transcript: true,
            has_audio_description: true,
            screen_reader_compatible: true,
            keyboard_accessible: true,
            color_independent: true,
            no_flashing: true,
            ..Default::default()
        },
        alternatives: vec![
            ContentAlternative {
                alternative_type: AlternativeType::Transcript,
                url: Some("https://example.com/transcript.txt".to_string()),
                language: Some("en".to_string()),
            },
            ContentAlternative {
                alternative_type: AlternativeType::Captions,
                url: Some("https://example.com/captions.vtt".to_string()),
                language: Some("en".to_string()),
            },
        ],
    };

    controller.add_content_to_module(course_id, module1.module_id, video_content).unwrap();

    // Verify
    let updated_course = controller.get_course(course_id).unwrap();
    assert_eq!(updated_course.modules.len(), 2);
    assert_eq!(updated_course.modules[0].content_items.len(), 1);
    assert_eq!(
        updated_course.accessibility_statement.wcag_conformance,
        Some(WCAGLevel::AA)
    );
}

#[test]
fn test_assessment_accommodations() {
    let mut controller = EduController::new();

    // Create assessment
    let assessment = controller.create_assessment(
        "Final Exam".to_string(),
        AssessmentType::Exam,
    );
    let assessment_id = assessment.assessment_id;

    // Set time limit
    controller.set_assessment_time_limit(assessment_id, 120).unwrap();

    // Create learner with extended time
    let mut profile = controller.create_profile_for_disability(DisabilityType::Blind);
    profile.assessment_accommodations.timing.extended_time = true;
    profile.assessment_accommodations.timing.time_multiplier = 1.5;
    controller.update_profile(profile.clone()).unwrap();

    // Get effective time
    let effective_time = controller
        .get_effective_time_limit(assessment_id, &profile)
        .unwrap();

    assert_eq!(effective_time, Some(180)); // 120 * 1.5

    // Test accommodation support
    assert!(controller
        .assessment_supports_accommodation(assessment_id, "extended_time")
        .unwrap());
    assert!(controller
        .assessment_supports_accommodation(assessment_id, "screen_reader")
        .unwrap());
}

#[test]
fn test_accessibility_matching() {
    let mut controller = EduController::new();

    // Create blind learner profile
    let profile = controller.create_profile_for_disability(DisabilityType::Blind);

    // Create accessible content
    let accessible_content = ContentItem {
        content_id: Uuid::new_v4(),
        content_type: ContentType::Video,
        title: "Accessible Video".to_string(),
        description: None,
        sequence: Some(1),
        required: true,
        duration_minutes: Some(10),
        url: None,
        accessibility_metadata: ContentAccessibilityMetadata {
            has_captions: true,
            has_transcript: true,
            has_audio_description: true,
            alt_text_provided: true,
            keyboard_accessible: true,
            screen_reader_compatible: true,
            color_independent: true,
            no_flashing: true,
            ..Default::default()
        },
        alternatives: vec![],
    };

    // Create inaccessible content
    let inaccessible_content = ContentItem {
        content_id: Uuid::new_v4(),
        content_type: ContentType::Video,
        title: "Inaccessible Video".to_string(),
        description: None,
        sequence: Some(1),
        required: true,
        duration_minutes: Some(10),
        url: None,
        accessibility_metadata: ContentAccessibilityMetadata::default(),
        alternatives: vec![],
    };

    // Check matching
    let accessible_result = EduController::check_content_accessibility(&profile, &accessible_content);
    let inaccessible_result = EduController::check_content_accessibility(&profile, &inaccessible_content);

    assert!(accessible_result.compatible);
    assert!(!inaccessible_result.compatible);
    assert!(!inaccessible_result.issues.is_empty());
}

#[test]
fn test_recommended_accommodations() {
    let mut controller = EduController::new();

    // Create profile with multiple disabilities
    let mut profile = controller.create_profile();
    profile.disability_profile.disclosed = true;
    profile.disability_profile.disability_types = vec![
        DisabilityType::Blind,
        DisabilityType::Adhd,
    ];
    profile.display_preferences.screen_reader.enabled = true;
    profile.display_preferences.magnification.enabled = true;

    let recommendations = EduController::get_recommended_accommodations(&profile);

    // Should have recommendations for both disabilities
    assert!(recommendations.iter().any(|r| r.accommodation_type == "screen_reader"));
    assert!(recommendations.iter().any(|r| r.accommodation_type == "extended_time"));
    assert!(recommendations.iter().any(|r| r.accommodation_type == "reduced_distractions"));
}

#[test]
fn test_wcag_conformance_check() {
    // Create content missing captions
    let content = ContentItem {
        content_id: Uuid::new_v4(),
        content_type: ContentType::Video,
        title: "Uncaptioned Video".to_string(),
        description: None,
        sequence: Some(1),
        required: true,
        duration_minutes: Some(10),
        url: None,
        accessibility_metadata: ContentAccessibilityMetadata {
            has_captions: false,
            has_transcript: false,
            has_audio_description: false,
            screen_reader_compatible: false,
            ..Default::default()
        },
        alternatives: vec![],
    };

    let issues = EduController::check_wcag_conformance(&content);

    // Should have issues for missing captions (1.2.2) and audio description (1.2.3)
    assert!(issues.iter().any(|i| i.criterion == "1.2.2"));
    assert!(issues.iter().any(|i| i.criterion == "1.2.3"));
    assert!(issues.iter().any(|i| i.criterion == "4.1.2")); // Screen reader compatibility
}

#[test]
fn test_simulator() {
    let simulator = LearnerSimulator::new();

    let blind = simulator.create_blind_learner();
    assert!(blind.display_preferences.screen_reader.enabled);

    let deaf = simulator.create_deaf_learner();
    assert!(deaf.content_preferences.captions.required);

    let dyslexic = simulator.create_dyslexic_learner();
    assert_eq!(
        dyslexic.display_preferences.text_settings.font_family,
        Some(FontFamily::Opendyslexic)
    );

    let adhd = simulator.create_adhd_learner();
    assert!(adhd.display_preferences.reduce_motion);

    let motor = simulator.create_motor_impaired_learner();
    assert!(motor.control_preferences.click_settings.large_targets);

    let korean_deaf = simulator.create_korean_deaf_learner();
    assert_eq!(
        korean_deaf.content_preferences.sign_language.language,
        Some(SignLanguageType::Ksl)
    );
}

#[tokio::test]
async fn test_memory_storage() {
    let storage = std::sync::Arc::new(MemoryStorage::new());
    let mut controller = EduController::new().with_storage(storage);

    // Create and save profile
    let profile = controller.create_profile_for_disability(DisabilityType::Deaf);
    controller.save_profile(&profile).await.unwrap();

    // Load from storage
    let loaded = controller.load_profile(profile.profile_id).await.unwrap();
    assert_eq!(loaded.profile_id, profile.profile_id);
    assert!(loaded.content_preferences.captions.required);
}

#[test]
fn test_validation() {
    let mut controller = EduController::new();
    let mut profile = controller.create_profile();

    // Valid profile should pass
    assert!(EduController::validate_profile(&profile).is_ok());

    // Invalid magnification should fail
    profile.display_preferences.magnification.level = 15.0;
    assert!(EduController::validate_profile(&profile).is_err());
}
