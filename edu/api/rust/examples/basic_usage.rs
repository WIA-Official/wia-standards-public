//! Basic Usage Example
//! 弘益人間 - Education for Everyone

use wia_edu::{EduController, types::*};

fn main() {
    // Create a new education controller
    let mut controller = EduController::new();

    println!("=== WIA Education API Basic Usage ===\n");

    // ========================================================================
    // Creating Learner Profiles
    // ========================================================================
    println!("1. Creating Learner Profiles");
    println!("{}", "-".repeat(40));

    // Create a profile for a blind learner
    let blind_profile = controller.create_profile_for_disability(DisabilityType::Blind);
    println!("Created blind learner profile: {}", blind_profile.profile_id);
    println!("  - Screen reader enabled: {}", blind_profile.display_preferences.screen_reader.enabled);
    println!("  - Audio description required: {}", blind_profile.content_preferences.audio_description.required);
    println!("  - Keyboard only: {}", blind_profile.control_preferences.input_method.keyboard_only);

    // Create a profile for a deaf learner
    let deaf_profile = controller.create_profile_for_disability(DisabilityType::Deaf);
    println!("\nCreated deaf learner profile: {}", deaf_profile.profile_id);
    println!("  - Captions required: {}", deaf_profile.content_preferences.captions.required);
    println!("  - Sign language preferred: {}", deaf_profile.content_preferences.sign_language.preferred);

    // Create a profile for a dyslexic learner
    let dyslexic_profile = controller.create_profile_for_disability(DisabilityType::Dyslexia);
    println!("\nCreated dyslexic learner profile: {}", dyslexic_profile.profile_id);
    println!("  - Font family: {:?}", dyslexic_profile.display_preferences.text_settings.font_family);
    println!("  - Text-to-speech enabled: {}", dyslexic_profile.content_preferences.text_to_speech.enabled);
    println!("  - Extended time: {}", dyslexic_profile.assessment_accommodations.timing.extended_time);

    // ========================================================================
    // Creating a Course
    // ========================================================================
    println!("\n2. Creating an Accessible Course");
    println!("{}", "-".repeat(40));

    let course = controller.create_course("Introduction to Web Accessibility".to_string());
    let course_id = course.course_id;
    println!("Created course: {} ({})", course.title, course_id);

    // Set accessibility statement
    controller.set_accessibility_statement(
        course_id,
        AccessibilityStatement {
            commitment: Some("We are committed to providing an inclusive learning experience.".to_string()),
            wcag_conformance: Some(WCAGLevel::AA),
            conformance_date: Some("2025-01-01".to_string()),
            known_issues: Vec::new(),
            contact: Some(ContactInfo {
                name: Some("Accessibility Office".to_string()),
                email: Some("accessibility@university.edu".to_string()),
                phone: None,
            }),
            accommodation_request_process: Some(
                "Please contact the Accessibility Office at least 2 weeks before the course starts.".to_string(),
            ),
        },
    ).unwrap();
    println!("Set accessibility statement with WCAG AA conformance");

    // Add modules
    let module1 = controller.add_module(course_id, "Week 1: Introduction to WCAG".to_string()).unwrap();
    let module2 = controller.add_module(course_id, "Week 2: Perceivable Content".to_string()).unwrap();
    println!("Added modules: {}, {}", module1.title, module2.title);

    // Add accessible content
    let video = ContentItem {
        content_id: uuid::Uuid::new_v4(),
        content_type: ContentType::Video,
        title: "What is Web Accessibility?".to_string(),
        description: Some("An introduction to web accessibility concepts".to_string()),
        sequence: Some(1),
        required: true,
        duration_minutes: Some(15),
        url: Some("https://example.com/intro-video.mp4".to_string()),
        accessibility_metadata: ContentAccessibilityMetadata {
            has_captions: true,
            has_transcript: true,
            has_audio_description: true,
            screen_reader_compatible: true,
            keyboard_accessible: true,
            color_independent: true,
            no_flashing: true,
            accessibility_features: vec![
                AccessibilityFeature::Captions,
                AccessibilityFeature::Transcript,
                AccessibilityFeature::AudioDescription,
            ],
            ..Default::default()
        },
        alternatives: vec![
            ContentAlternative {
                alternative_type: AlternativeType::Transcript,
                url: Some("https://example.com/intro-transcript.txt".to_string()),
                language: Some("en".to_string()),
            },
        ],
    };

    controller.add_content_to_module(course_id, module1.module_id, video).unwrap();
    println!("Added accessible video content with captions and audio description");

    // ========================================================================
    // Checking Content Accessibility
    // ========================================================================
    println!("\n3. Checking Content Accessibility");
    println!("{}", "-".repeat(40));

    let updated_course = controller.get_course(course_id).unwrap();
    let content = &updated_course.modules[0].content_items[0];

    // Check if content meets blind learner's needs
    let match_result = EduController::check_content_accessibility(&blind_profile, content);
    println!("Content accessible for blind learner: {}", match_result.compatible);
    if !match_result.issues.is_empty() {
        println!("  Issues found:");
        for issue in &match_result.issues {
            println!("    - {} ({:?}): {}", issue.category, issue.severity, issue.description);
        }
    }

    // ========================================================================
    // Getting Recommended Accommodations
    // ========================================================================
    println!("\n4. Recommended Accommodations");
    println!("{}", "-".repeat(40));

    let recommendations = EduController::get_recommended_accommodations(&blind_profile);
    println!("Accommodations recommended for blind learner:");
    for rec in &recommendations {
        println!(
            "  - {} ({:?}): {}",
            rec.accommodation_type, rec.priority, rec.description
        );
        if let Some(value) = &rec.suggested_value {
            println!("    Suggested: {}", value);
        }
    }

    // ========================================================================
    // Creating an Assessment
    // ========================================================================
    println!("\n5. Creating an Accessible Assessment");
    println!("{}", "-".repeat(40));

    let assessment = controller.create_assessment(
        "Module 1 Quiz".to_string(),
        AssessmentType::Quiz,
    );
    let assessment_id = assessment.assessment_id;
    println!("Created assessment: {} ({})", assessment.title, assessment_id);

    // Set time limit
    controller.set_assessment_time_limit(assessment_id, 30).unwrap();
    println!("Set time limit: 30 minutes");

    // Get effective time for learner with extended time
    let effective_time = controller
        .get_effective_time_limit(assessment_id, &blind_profile)
        .unwrap();
    println!(
        "Effective time for blind learner (1.5x): {:?} minutes",
        effective_time
    );

    // ========================================================================
    // Export and Import
    // ========================================================================
    println!("\n6. Export and Import");
    println!("{}", "-".repeat(40));

    let json = controller.export_profile(blind_profile.profile_id).unwrap();
    println!("Exported profile to JSON ({} bytes)", json.len());

    let imported = controller.import_profile(&json).unwrap();
    println!("Imported profile with new ID: {}", imported.profile_id);
    println!("  - Screen reader still enabled: {}", imported.display_preferences.screen_reader.enabled);

    println!("\n=== Done ===");
}
