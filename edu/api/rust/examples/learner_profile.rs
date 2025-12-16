//! Learner Profile Example
//! Demonstrates creating and customizing learner accessibility profiles
//! 弘益人間 - Education for Everyone

use wia_edu::{EduController, types::*, adapters::LearnerSimulator};

fn main() {
    println!("=== WIA Education - Learner Profile Examples ===\n");

    let mut controller = EduController::new();
    let simulator = LearnerSimulator::new();

    // ========================================================================
    // 1. Profile for Different Disabilities
    // ========================================================================
    println!("1. Profiles for Different Disabilities");
    println!("{}", "=".repeat(50));

    // Blind learner
    println!("\n[Blind Learner]");
    let blind = simulator.create_blind_learner();
    print_profile_summary(&blind);

    // Deaf learner
    println!("\n[Deaf Learner]");
    let deaf = simulator.create_deaf_learner();
    print_profile_summary(&deaf);

    // Dyslexic learner
    println!("\n[Dyslexic Learner]");
    let dyslexic = simulator.create_dyslexic_learner();
    print_profile_summary(&dyslexic);

    // ADHD learner
    println!("\n[ADHD Learner]");
    let adhd = simulator.create_adhd_learner();
    print_profile_summary(&adhd);

    // Motor impaired learner
    println!("\n[Motor Impaired Learner]");
    let motor = simulator.create_motor_impaired_learner();
    print_profile_summary(&motor);

    // Korean deaf learner
    println!("\n[Korean Deaf Learner - 한국어 청각장애 학습자]");
    let korean_deaf = simulator.create_korean_deaf_learner();
    print_profile_summary(&korean_deaf);

    // ========================================================================
    // 2. Custom Profile Creation
    // ========================================================================
    println!("\n\n2. Custom Profile Creation");
    println!("{}", "=".repeat(50));

    let mut custom = controller.create_profile();

    // Set learner info
    custom.learner_info = LearnerInfo {
        display_name: Some("Alex".to_string()),
        preferred_language: Some("en".to_string()),
        secondary_languages: vec!["es".to_string()],
        age_group: Some(AgeGroup::Adult),
        education_level: Some(EducationLevel::Undergraduate),
    };

    // Disclose disabilities
    custom.disability_profile = DisabilityProfile {
        disclosed: true,
        disability_types: vec![DisabilityType::LowVision, DisabilityType::Adhd],
        notes: Some("Uses magnification and needs frequent breaks".to_string()),
    };

    // Display preferences
    custom.display_preferences = DisplayPreferences {
        magnification: MagnificationSettings {
            enabled: true,
            level: 2.5,
            follow_focus: true,
        },
        text_settings: TextSettings {
            font_size: FontSize::XLarge,
            font_family: Some(FontFamily::SansSerif),
            line_spacing: 1.8,
            letter_spacing: Spacing::Wide,
            ..Default::default()
        },
        color_settings: ColorSettings {
            high_contrast: true,
            contrast_mode: Some(ContrastMode::HighContrastDark),
            ..Default::default()
        },
        reduce_motion: true,
        ..Default::default()
    };

    // Control preferences
    custom.control_preferences = ControlPreferences {
        timing: TimingPreferences {
            extended_time: true,
            time_multiplier: 1.5,
            disable_auto_advance: true,
            pause_on_inactivity: true,
        },
        ..Default::default()
    };

    // Content preferences
    custom.content_preferences = ContentPreferences {
        simplification: SimplificationSettings {
            chunked_content: true,
            definitions: true,
            ..Default::default()
        },
        ..Default::default()
    };

    // Learning style (UDL)
    custom.learning_style = LearningStyle {
        engagement: EngagementPreferences {
            interest_triggers: vec![
                InterestTrigger::Choice,
                InterestTrigger::Gamification,
            ],
            collaboration_preference: Some(CollaborationPreference::SmallGroups),
            feedback_style: Some(FeedbackStyle::Immediate),
            self_regulation_support: true,
        },
        representation: RepresentationPreferences {
            preferred_modalities: vec![
                LearningModality::Visual,
                LearningModality::Interactive,
            ],
            background_knowledge_support: true,
            pattern_highlighting: true,
        },
        action_expression: ActionExpressionPreferences {
            expression_preferences: vec![
                ExpressionMethod::Multimedia,
                ExpressionMethod::Visual,
            ],
            planning_support: true,
            progress_tracking: Some(ProgressTrackingStyle::Visual),
        },
    };

    // Assessment accommodations
    custom.assessment_accommodations = AssessmentAccommodations {
        timing: AssessmentTimingAccommodations {
            extended_time: true,
            time_multiplier: 1.5,
            breaks_allowed: true,
            break_frequency_minutes: Some(30),
            ..Default::default()
        },
        environment: EnvironmentAccommodations {
            reduced_distractions: true,
            ..Default::default()
        },
        ..Default::default()
    };

    println!("\n[Custom Profile for Low Vision + ADHD Learner]");
    print_profile_summary(&custom);

    // Validate the profile
    match EduController::validate_profile(&custom) {
        Ok(_) => println!("\n✓ Profile is valid"),
        Err(e) => println!("\n✗ Profile validation failed: {}", e),
    }

    // Save to controller
    controller.update_profile(custom.clone()).unwrap();
    println!("✓ Profile saved to controller");

    // ========================================================================
    // 3. Recommended Accommodations
    // ========================================================================
    println!("\n\n3. Recommended Accommodations");
    println!("{}", "=".repeat(50));

    let recommendations = EduController::get_recommended_accommodations(&custom);
    println!("\nRecommendations for custom profile:");
    for rec in recommendations {
        let priority = match rec.priority {
            wia_edu::AccommodationPriority::Required => "REQUIRED",
            wia_edu::AccommodationPriority::High => "HIGH",
            wia_edu::AccommodationPriority::Medium => "MEDIUM",
            wia_edu::AccommodationPriority::Low => "LOW",
        };
        println!("\n  [{}] {}", priority, rec.accommodation_type);
        println!("    {}", rec.description);
        if let Some(value) = rec.suggested_value {
            println!("    Suggested: {}", value);
        }
    }

    // ========================================================================
    // 4. Export Profile
    // ========================================================================
    println!("\n\n4. Export Profile to JSON");
    println!("{}", "=".repeat(50));

    let json = controller.export_profile(custom.profile_id).unwrap();
    println!("\nProfile JSON (truncated):");
    println!("{}", &json[..json.len().min(500)]);
    if json.len() > 500 {
        println!("... ({} more bytes)", json.len() - 500);
    }

    println!("\n=== Done ===");
}

fn print_profile_summary(profile: &LearnerProfile) {
    println!("  ID: {}", profile.profile_id);

    if let Some(name) = &profile.learner_info.display_name {
        println!("  Name: {}", name);
    }

    if let Some(lang) = &profile.learner_info.preferred_language {
        println!("  Language: {}", lang);
    }

    if !profile.disability_profile.disability_types.is_empty() {
        let types: Vec<String> = profile
            .disability_profile
            .disability_types
            .iter()
            .map(|d| format!("{:?}", d))
            .collect();
        println!("  Disabilities: {}", types.join(", "));
    }

    // Display settings
    let mut display_features = Vec::new();
    if profile.display_preferences.screen_reader.enabled {
        display_features.push("Screen Reader");
    }
    if profile.display_preferences.magnification.enabled {
        display_features.push(format!(
            "Magnification ({}x)",
            profile.display_preferences.magnification.level
        ).leak());
    }
    if profile.display_preferences.color_settings.high_contrast {
        display_features.push("High Contrast");
    }
    if profile.display_preferences.reduce_motion {
        display_features.push("Reduced Motion");
    }
    if !display_features.is_empty() {
        println!("  Display: {}", display_features.join(", "));
    }

    // Content preferences
    let mut content_features = Vec::new();
    if profile.content_preferences.captions.required {
        content_features.push("Captions");
    }
    if profile.content_preferences.audio_description.required {
        content_features.push("Audio Description");
    }
    if profile.content_preferences.transcripts.required {
        content_features.push("Transcripts");
    }
    if profile.content_preferences.sign_language.preferred {
        let lang = profile
            .content_preferences
            .sign_language
            .language
            .map(|l| format!("{:?}", l))
            .unwrap_or_else(|| "Sign Language".to_string());
        content_features.push(lang.leak());
    }
    if profile.content_preferences.text_to_speech.enabled {
        content_features.push("Text-to-Speech");
    }
    if !content_features.is_empty() {
        println!("  Content Needs: {}", content_features.join(", "));
    }

    // Assessment accommodations
    let mut assess_features = Vec::new();
    if profile.assessment_accommodations.timing.extended_time {
        assess_features.push(format!(
            "Extended Time ({}x)",
            profile.assessment_accommodations.timing.time_multiplier
        ));
    }
    if profile.assessment_accommodations.timing.breaks_allowed {
        assess_features.push("Breaks Allowed".to_string());
    }
    if profile.assessment_accommodations.assistance.reader {
        assess_features.push("Reader".to_string());
    }
    if profile.assessment_accommodations.assistance.spell_check {
        assess_features.push("Spell Check".to_string());
    }
    if !assess_features.is_empty() {
        println!("  Assessment: {}", assess_features.join(", "));
    }
}
