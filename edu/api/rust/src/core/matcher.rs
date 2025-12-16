//! Accessibility Matcher
//! Matches learner needs with content/course accessibility features
//! 弘益人間 - Education for Everyone

use crate::types::*;

/// Matches learner accessibility needs with available content
pub struct AccessibilityMatcher;

impl AccessibilityMatcher {
    /// Check if content meets learner's accessibility needs
    pub fn content_meets_needs(profile: &LearnerProfile, content: &ContentItem) -> MatchResult {
        let mut issues = Vec::new();
        let mut adaptations_needed = Vec::new();

        let meta = &content.accessibility_metadata;

        // Check for blind/screen reader users
        if profile.display_preferences.screen_reader.enabled {
            if !meta.screen_reader_compatible {
                issues.push(MatchIssue {
                    category: "screen_reader".to_string(),
                    description: "Content may not be fully screen reader compatible".to_string(),
                    severity: MatchSeverity::High,
                });
            }

            if content.content_type == ContentType::Image && !meta.alt_text_provided {
                issues.push(MatchIssue {
                    category: "alt_text".to_string(),
                    description: "Image is missing alternative text".to_string(),
                    severity: MatchSeverity::High,
                });
            }

            if content.content_type == ContentType::Video && !meta.has_audio_description {
                adaptations_needed.push(AdaptationNeeded {
                    adaptation_type: "audio_description".to_string(),
                    description: "Audio description needed for video content".to_string(),
                    alternatives: content
                        .alternatives
                        .iter()
                        .filter(|a| a.alternative_type == AlternativeType::AudioDescription)
                        .map(|a| a.url.clone().unwrap_or_default())
                        .collect(),
                });
            }
        }

        // Check for deaf/hard of hearing users
        if profile.content_preferences.captions.required {
            if content.content_type == ContentType::Video && !meta.has_captions {
                issues.push(MatchIssue {
                    category: "captions".to_string(),
                    description: "Video content requires captions".to_string(),
                    severity: MatchSeverity::High,
                });

                // Check if alternative exists
                let has_caption_alt = content
                    .alternatives
                    .iter()
                    .any(|a| a.alternative_type == AlternativeType::Captions);

                if has_caption_alt {
                    adaptations_needed.push(AdaptationNeeded {
                        adaptation_type: "captions".to_string(),
                        description: "Use captioned version".to_string(),
                        alternatives: content
                            .alternatives
                            .iter()
                            .filter(|a| a.alternative_type == AlternativeType::Captions)
                            .map(|a| a.url.clone().unwrap_or_default())
                            .collect(),
                    });
                }
            }
        }

        if profile.content_preferences.transcripts.required {
            if matches!(
                content.content_type,
                ContentType::Video | ContentType::Audio
            ) && !meta.has_transcript
            {
                let has_transcript = content
                    .alternatives
                    .iter()
                    .any(|a| a.alternative_type == AlternativeType::Transcript);

                if has_transcript {
                    adaptations_needed.push(AdaptationNeeded {
                        adaptation_type: "transcript".to_string(),
                        description: "Transcript available".to_string(),
                        alternatives: content
                            .alternatives
                            .iter()
                            .filter(|a| a.alternative_type == AlternativeType::Transcript)
                            .map(|a| a.url.clone().unwrap_or_default())
                            .collect(),
                    });
                } else {
                    issues.push(MatchIssue {
                        category: "transcript".to_string(),
                        description: "Audio/video content requires transcript".to_string(),
                        severity: MatchSeverity::Medium,
                    });
                }
            }
        }

        // Check for keyboard-only users
        if profile.control_preferences.input_method.keyboard_only {
            if content.content_type == ContentType::Interactive && !meta.keyboard_accessible {
                issues.push(MatchIssue {
                    category: "keyboard".to_string(),
                    description: "Interactive content is not keyboard accessible".to_string(),
                    severity: MatchSeverity::High,
                });
            }
        }

        // Check for motion sensitivity
        if profile.display_preferences.reduce_motion {
            if meta.hazards.contains(&ContentHazard::Motion) {
                issues.push(MatchIssue {
                    category: "motion".to_string(),
                    description: "Content contains motion that may cause discomfort".to_string(),
                    severity: MatchSeverity::Medium,
                });
            }
        }

        // Check for photosensitivity
        if meta.hazards.contains(&ContentHazard::Flashing) {
            issues.push(MatchIssue {
                category: "flashing".to_string(),
                description: "Content contains flashing that may cause seizures".to_string(),
                severity: MatchSeverity::Critical,
            });
        }

        // Check for simplified language needs
        if profile.content_preferences.simplification.reading_level != ReadingLevel::Standard {
            if let Some(level) = meta.reading_level {
                if level == ContentReadingLevel::Advanced
                    && profile.content_preferences.simplification.reading_level
                        == ReadingLevel::EasyRead
                {
                    let has_simplified = content
                        .alternatives
                        .iter()
                        .any(|a| a.alternative_type == AlternativeType::Simplified);

                    if has_simplified {
                        adaptations_needed.push(AdaptationNeeded {
                            adaptation_type: "simplified".to_string(),
                            description: "Simplified version available".to_string(),
                            alternatives: content
                                .alternatives
                                .iter()
                                .filter(|a| a.alternative_type == AlternativeType::Simplified)
                                .map(|a| a.url.clone().unwrap_or_default())
                                .collect(),
                        });
                    } else {
                        issues.push(MatchIssue {
                            category: "reading_level".to_string(),
                            description: "Content reading level may be too advanced".to_string(),
                            severity: MatchSeverity::Low,
                        });
                    }
                }
            }
        }

        let compatible = !issues.iter().any(|i| {
            matches!(i.severity, MatchSeverity::Critical | MatchSeverity::High)
        });

        MatchResult {
            compatible,
            issues,
            adaptations_needed,
        }
    }

    /// Check if course meets learner's accessibility needs
    pub fn course_meets_needs(profile: &LearnerProfile, course: &Course) -> CourseMatchResult {
        let mut module_results = Vec::new();
        let mut overall_issues = Vec::new();

        // Check course-level features
        let needs_captions = profile.content_preferences.captions.required;
        let needs_screen_reader = profile.display_preferences.screen_reader.enabled;

        if needs_captions
            && !course
                .accessibility_features
                .features
                .contains(&CourseFeature::CaptionsAllVideos)
        {
            overall_issues.push(MatchIssue {
                category: "captions".to_string(),
                description: "Course may not have captions for all videos".to_string(),
                severity: MatchSeverity::Medium,
            });
        }

        if needs_screen_reader
            && !course
                .accessibility_features
                .features
                .contains(&CourseFeature::ScreenReaderSupport)
        {
            overall_issues.push(MatchIssue {
                category: "screen_reader".to_string(),
                description: "Course may not fully support screen readers".to_string(),
                severity: MatchSeverity::High,
            });
        }

        // Check WCAG conformance
        if let Some(level) = course.accessibility_statement.wcag_conformance {
            if level == WCAGLevel::None {
                overall_issues.push(MatchIssue {
                    category: "wcag".to_string(),
                    description: "Course has not been evaluated for WCAG conformance".to_string(),
                    severity: MatchSeverity::Medium,
                });
            }
        }

        // Check each module's content
        for module in &course.modules {
            let mut content_results = Vec::new();

            for content in &module.content_items {
                let result = Self::content_meets_needs(profile, content);
                content_results.push((content.content_id, result));
            }

            module_results.push(ModuleMatchResult {
                module_id: module.module_id,
                content_results,
            });
        }

        // Calculate overall compatibility
        let has_critical_issues = overall_issues
            .iter()
            .any(|i| matches!(i.severity, MatchSeverity::Critical));

        let content_compatible = module_results
            .iter()
            .flat_map(|m| &m.content_results)
            .all(|(_, r)| r.compatible);

        CourseMatchResult {
            compatible: !has_critical_issues && content_compatible,
            course_issues: overall_issues,
            module_results,
        }
    }

    /// Get recommended accommodations for an assessment
    pub fn get_recommended_accommodations(
        profile: &LearnerProfile,
    ) -> Vec<RecommendedAccommodation> {
        let mut recommendations = Vec::new();

        // Time accommodations
        if profile.disability_profile.disability_types.iter().any(|d| {
            matches!(
                d,
                DisabilityType::Blind
                    | DisabilityType::LowVision
                    | DisabilityType::Dyslexia
                    | DisabilityType::Adhd
                    | DisabilityType::MotorImpairment
                    | DisabilityType::Cognitive
            )
        }) {
            recommendations.push(RecommendedAccommodation {
                accommodation_type: "extended_time".to_string(),
                description: "Extended time for assessments".to_string(),
                suggested_value: Some("1.5x".to_string()),
                priority: AccommodationPriority::High,
            });
        }

        // Presentation accommodations
        if profile.display_preferences.screen_reader.enabled {
            recommendations.push(RecommendedAccommodation {
                accommodation_type: "screen_reader".to_string(),
                description: "Screen reader compatible assessment format".to_string(),
                suggested_value: None,
                priority: AccommodationPriority::Required,
            });
        }

        if profile.display_preferences.magnification.enabled {
            recommendations.push(RecommendedAccommodation {
                accommodation_type: "large_print".to_string(),
                description: "Large print or zoomed display".to_string(),
                suggested_value: Some(format!(
                    "{}x zoom",
                    profile.display_preferences.magnification.level
                )),
                priority: AccommodationPriority::High,
            });
        }

        if profile.display_preferences.color_settings.high_contrast {
            recommendations.push(RecommendedAccommodation {
                accommodation_type: "high_contrast".to_string(),
                description: "High contrast display mode".to_string(),
                suggested_value: None,
                priority: AccommodationPriority::Medium,
            });
        }

        // Response accommodations
        if profile.content_preferences.text_to_speech.enabled {
            recommendations.push(RecommendedAccommodation {
                accommodation_type: "text_to_speech".to_string(),
                description: "Text-to-speech for reading questions".to_string(),
                suggested_value: None,
                priority: AccommodationPriority::High,
            });
        }

        if profile.assessment_accommodations.assistance.spell_check {
            recommendations.push(RecommendedAccommodation {
                accommodation_type: "spell_check".to_string(),
                description: "Spell check enabled for written responses".to_string(),
                suggested_value: None,
                priority: AccommodationPriority::Medium,
            });
        }

        if profile.assessment_accommodations.assistance.calculator {
            recommendations.push(RecommendedAccommodation {
                accommodation_type: "calculator".to_string(),
                description: "Calculator access".to_string(),
                suggested_value: None,
                priority: AccommodationPriority::Medium,
            });
        }

        // Environment accommodations
        if profile.disability_profile.disability_types.iter().any(|d| {
            matches!(
                d,
                DisabilityType::Adhd | DisabilityType::Anxiety | DisabilityType::Autism
            )
        }) {
            recommendations.push(RecommendedAccommodation {
                accommodation_type: "reduced_distractions".to_string(),
                description: "Reduced distraction environment".to_string(),
                suggested_value: None,
                priority: AccommodationPriority::High,
            });

            recommendations.push(RecommendedAccommodation {
                accommodation_type: "breaks".to_string(),
                description: "Scheduled breaks during assessment".to_string(),
                suggested_value: Some("Every 30 minutes".to_string()),
                priority: AccommodationPriority::Medium,
            });
        }

        recommendations
    }
}

/// Result of matching learner needs with content
#[derive(Debug, Clone)]
pub struct MatchResult {
    /// Whether the content is compatible with learner needs
    pub compatible: bool,
    /// Issues found
    pub issues: Vec<MatchIssue>,
    /// Adaptations that could help
    pub adaptations_needed: Vec<AdaptationNeeded>,
}

/// Issue found during matching
#[derive(Debug, Clone)]
pub struct MatchIssue {
    pub category: String,
    pub description: String,
    pub severity: MatchSeverity,
}

/// Severity of matching issue
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatchSeverity {
    Low,
    Medium,
    High,
    Critical,
}

/// Adaptation that could help with accessibility
#[derive(Debug, Clone)]
pub struct AdaptationNeeded {
    pub adaptation_type: String,
    pub description: String,
    pub alternatives: Vec<String>,
}

/// Result of matching course with learner needs
#[derive(Debug, Clone)]
pub struct CourseMatchResult {
    pub compatible: bool,
    pub course_issues: Vec<MatchIssue>,
    pub module_results: Vec<ModuleMatchResult>,
}

/// Module-level match results
#[derive(Debug, Clone)]
pub struct ModuleMatchResult {
    pub module_id: ModuleId,
    pub content_results: Vec<(ContentId, MatchResult)>,
}

/// Recommended accommodation
#[derive(Debug, Clone)]
pub struct RecommendedAccommodation {
    pub accommodation_type: String,
    pub description: String,
    pub suggested_value: Option<String>,
    pub priority: AccommodationPriority,
}

/// Priority of accommodation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccommodationPriority {
    Low,
    Medium,
    High,
    Required,
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use uuid::Uuid;

    fn create_blind_profile() -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo::default(),
            disability_profile: DisabilityProfile {
                disclosed: true,
                disability_types: vec![DisabilityType::Blind],
                notes: None,
            },
            display_preferences: DisplayPreferences {
                screen_reader: ScreenReaderSettings {
                    enabled: true,
                    preferred_reader: Some(ScreenReaderType::Nvda),
                    speech_rate: 1.5,
                    verbosity: Some(Verbosity::Verbose),
                },
                ..Default::default()
            },
            control_preferences: ControlPreferences {
                input_method: InputMethodSettings {
                    keyboard_only: true,
                    ..Default::default()
                },
                ..Default::default()
            },
            content_preferences: ContentPreferences {
                audio_description: AudioDescriptionSettings {
                    required: true,
                    extended: false,
                },
                transcripts: TranscriptSettings {
                    required: true,
                    interactive: true,
                },
                ..Default::default()
            },
            learning_style: LearningStyle::default(),
            assessment_accommodations: AssessmentAccommodations::default(),
            assistive_technology: AssistiveTechnology::default(),
            wia_integrations: WIAIntegrations::default(),
        }
    }

    fn create_accessible_video_content() -> ContentItem {
        ContentItem {
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
                screen_reader_compatible: true,
                keyboard_accessible: true,
                ..Default::default()
            },
            alternatives: vec![
                ContentAlternative {
                    alternative_type: AlternativeType::Transcript,
                    url: Some("https://example.com/transcript.txt".to_string()),
                    language: Some("en".to_string()),
                },
                ContentAlternative {
                    alternative_type: AlternativeType::AudioDescription,
                    url: Some("https://example.com/audio-desc.mp3".to_string()),
                    language: Some("en".to_string()),
                },
            ],
        }
    }

    #[test]
    fn test_accessible_content_matches() {
        let profile = create_blind_profile();
        let content = create_accessible_video_content();

        let result = AccessibilityMatcher::content_meets_needs(&profile, &content);
        assert!(result.compatible);
    }

    #[test]
    fn test_inaccessible_content_fails() {
        let profile = create_blind_profile();
        let content = ContentItem {
            content_id: Uuid::new_v4(),
            content_type: ContentType::Video,
            title: "Inaccessible Video".to_string(),
            description: None,
            sequence: Some(1),
            required: true,
            duration_minutes: Some(10),
            url: None,
            accessibility_metadata: ContentAccessibilityMetadata::default(),
            alternatives: Vec::new(),
        };

        let result = AccessibilityMatcher::content_meets_needs(&profile, &content);
        assert!(!result.compatible);
        assert!(!result.issues.is_empty());
    }

    #[test]
    fn test_recommended_accommodations() {
        let profile = create_blind_profile();
        let recommendations = AccessibilityMatcher::get_recommended_accommodations(&profile);

        assert!(recommendations
            .iter()
            .any(|r| r.accommodation_type == "screen_reader"));
        assert!(recommendations
            .iter()
            .any(|r| r.accommodation_type == "extended_time"));
    }
}
