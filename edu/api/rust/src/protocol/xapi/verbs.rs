//! xAPI Accessibility Verbs
//! 弘益人間 - Custom verbs for accessibility learning activities

use super::statements::XapiVerb;
use std::collections::HashMap;

/// WIA Accessibility Verb definitions
pub struct AccessibilityVerb;

impl AccessibilityVerb {
    /// WIA accessibility verb namespace
    pub const NAMESPACE: &'static str = "https://wia.live/xapi/verbs";

    // ========================================================================
    // Standard ADL Verbs
    // ========================================================================

    /// Completed an activity
    pub fn completed() -> XapiVerb {
        XapiVerb {
            id: "http://adlnet.gov/expapi/verbs/completed".to_string(),
            display: Some(create_display(&[
                ("en-US", "completed"),
                ("ko-KR", "완료함"),
            ])),
        }
    }

    /// Attempted an activity
    pub fn attempted() -> XapiVerb {
        XapiVerb {
            id: "http://adlnet.gov/expapi/verbs/attempted".to_string(),
            display: Some(create_display(&[
                ("en-US", "attempted"),
                ("ko-KR", "시도함"),
            ])),
        }
    }

    /// Passed an activity
    pub fn passed() -> XapiVerb {
        XapiVerb {
            id: "http://adlnet.gov/expapi/verbs/passed".to_string(),
            display: Some(create_display(&[
                ("en-US", "passed"),
                ("ko-KR", "통과함"),
            ])),
        }
    }

    /// Failed an activity
    pub fn failed() -> XapiVerb {
        XapiVerb {
            id: "http://adlnet.gov/expapi/verbs/failed".to_string(),
            display: Some(create_display(&[
                ("en-US", "failed"),
                ("ko-KR", "실패함"),
            ])),
        }
    }

    /// Experienced (interacted with) content
    pub fn experienced() -> XapiVerb {
        XapiVerb {
            id: "http://adlnet.gov/expapi/verbs/experienced".to_string(),
            display: Some(create_display(&[
                ("en-US", "experienced"),
                ("ko-KR", "경험함"),
            ])),
        }
    }

    /// Launched an activity
    pub fn launched() -> XapiVerb {
        XapiVerb {
            id: "http://adlnet.gov/expapi/verbs/launched".to_string(),
            display: Some(create_display(&[
                ("en-US", "launched"),
                ("ko-KR", "시작함"),
            ])),
        }
    }

    // ========================================================================
    // WIA Accessibility Verbs
    // ========================================================================

    /// Enabled an accessibility feature
    pub fn enabled_accessibility() -> XapiVerb {
        XapiVerb {
            id: format!("{}/enabled-accessibility", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "enabled accessibility feature"),
                ("ko-KR", "접근성 기능 활성화"),
            ])),
        }
    }

    /// Disabled an accessibility feature
    pub fn disabled_accessibility() -> XapiVerb {
        XapiVerb {
            id: format!("{}/disabled-accessibility", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "disabled accessibility feature"),
                ("ko-KR", "접근성 기능 비활성화"),
            ])),
        }
    }

    /// Requested an accommodation
    pub fn requested_accommodation() -> XapiVerb {
        XapiVerb {
            id: format!("{}/requested-accommodation", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "requested accommodation"),
                ("ko-KR", "편의 제공 요청함"),
            ])),
        }
    }

    /// Accommodation was applied
    pub fn applied_accommodation() -> XapiVerb {
        XapiVerb {
            id: format!("{}/applied-accommodation", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "applied accommodation"),
                ("ko-KR", "편의 제공 적용됨"),
            ])),
        }
    }

    /// Adjusted display settings
    pub fn adjusted_display() -> XapiVerb {
        XapiVerb {
            id: format!("{}/adjusted-display", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "adjusted display settings"),
                ("ko-KR", "화면 설정 조정함"),
            ])),
        }
    }

    /// Used assistive technology
    pub fn used_assistive_tech() -> XapiVerb {
        XapiVerb {
            id: format!("{}/used-assistive-technology", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "used assistive technology"),
                ("ko-KR", "보조 기술 사용함"),
            ])),
        }
    }

    /// Accessed alternative content
    pub fn accessed_alternative() -> XapiVerb {
        XapiVerb {
            id: format!("{}/accessed-alternative", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "accessed alternative content"),
                ("ko-KR", "대체 콘텐츠 접근함"),
            ])),
        }
    }

    /// Enabled captions
    pub fn enabled_captions() -> XapiVerb {
        XapiVerb {
            id: format!("{}/enabled-captions", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "enabled captions"),
                ("ko-KR", "자막 활성화"),
            ])),
        }
    }

    /// Enabled audio description
    pub fn enabled_audio_description() -> XapiVerb {
        XapiVerb {
            id: format!("{}/enabled-audio-description", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "enabled audio description"),
                ("ko-KR", "화면 해설 활성화"),
            ])),
        }
    }

    /// Enabled sign language
    pub fn enabled_sign_language() -> XapiVerb {
        XapiVerb {
            id: format!("{}/enabled-sign-language", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "enabled sign language"),
                ("ko-KR", "수어 통역 활성화"),
            ])),
        }
    }

    /// Used extended time
    pub fn used_extended_time() -> XapiVerb {
        XapiVerb {
            id: format!("{}/used-extended-time", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "used extended time"),
                ("ko-KR", "연장 시간 사용함"),
            ])),
        }
    }

    /// Took a break
    pub fn took_break() -> XapiVerb {
        XapiVerb {
            id: format!("{}/took-break", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "took a break"),
                ("ko-KR", "휴식 취함"),
            ])),
        }
    }

    /// Updated accessibility profile
    pub fn updated_profile() -> XapiVerb {
        XapiVerb {
            id: format!("{}/updated-profile", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "updated accessibility profile"),
                ("ko-KR", "접근성 프로필 업데이트함"),
            ])),
        }
    }

    /// Synced profile
    pub fn synced_profile() -> XapiVerb {
        XapiVerb {
            id: format!("{}/synced-profile", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "synced accessibility profile"),
                ("ko-KR", "접근성 프로필 동기화함"),
            ])),
        }
    }

    /// Reported accessibility issue
    pub fn reported_issue() -> XapiVerb {
        XapiVerb {
            id: format!("{}/reported-accessibility-issue", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "reported accessibility issue"),
                ("ko-KR", "접근성 문제 신고함"),
            ])),
        }
    }

    /// Accessibility barrier encountered
    pub fn encountered_barrier() -> XapiVerb {
        XapiVerb {
            id: format!("{}/encountered-barrier", Self::NAMESPACE),
            display: Some(create_display(&[
                ("en-US", "encountered accessibility barrier"),
                ("ko-KR", "접근성 장벽 만남"),
            ])),
        }
    }
}

/// Helper to create display map
fn create_display(entries: &[(&str, &str)]) -> HashMap<String, String> {
    entries
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

/// WIA Accessibility Activity Types
pub struct AccessibilityActivityType;

impl AccessibilityActivityType {
    /// WIA activity type namespace
    pub const NAMESPACE: &'static str = "https://wia.live/xapi/activities";

    /// Accessibility profile activity
    pub fn accessibility_profile() -> String {
        format!("{}/accessibility-profile", Self::NAMESPACE)
    }

    /// Accessibility setting activity
    pub fn accessibility_setting() -> String {
        format!("{}/accessibility-setting", Self::NAMESPACE)
    }

    /// Accommodation activity
    pub fn accommodation() -> String {
        format!("{}/accommodation", Self::NAMESPACE)
    }

    /// Alternative content activity
    pub fn alternative_content() -> String {
        format!("{}/alternative-content", Self::NAMESPACE)
    }

    /// Assistive technology activity
    pub fn assistive_technology() -> String {
        format!("{}/assistive-technology", Self::NAMESPACE)
    }

    /// Caption activity
    pub fn caption() -> String {
        format!("{}/caption", Self::NAMESPACE)
    }

    /// Audio description activity
    pub fn audio_description() -> String {
        format!("{}/audio-description", Self::NAMESPACE)
    }

    /// Sign language activity
    pub fn sign_language() -> String {
        format!("{}/sign-language", Self::NAMESPACE)
    }

    /// Assessment with accommodations
    pub fn accessible_assessment() -> String {
        format!("{}/accessible-assessment", Self::NAMESPACE)
    }

    /// Accessible course
    pub fn accessible_course() -> String {
        format!("{}/accessible-course", Self::NAMESPACE)
    }

    /// Accessible content
    pub fn accessible_content() -> String {
        format!("{}/accessible-content", Self::NAMESPACE)
    }
}

/// WIA Accessibility Extension Keys
pub struct AccessibilityExtension;

impl AccessibilityExtension {
    /// Extension namespace
    pub const NAMESPACE: &'static str = "https://wia.live/xapi/extensions";

    /// Accessibility features used
    pub fn features_used() -> String {
        format!("{}/features-used", Self::NAMESPACE)
    }

    /// Accommodation type
    pub fn accommodation_type() -> String {
        format!("{}/accommodation-type", Self::NAMESPACE)
    }

    /// Time multiplier used
    pub fn time_multiplier() -> String {
        format!("{}/time-multiplier", Self::NAMESPACE)
    }

    /// Assistive technology type
    pub fn assistive_tech_type() -> String {
        format!("{}/assistive-technology-type", Self::NAMESPACE)
    }

    /// Disability type (if disclosed)
    pub fn disability_type() -> String {
        format!("{}/disability-type", Self::NAMESPACE)
    }

    /// WCAG conformance level
    pub fn wcag_level() -> String {
        format!("{}/wcag-level", Self::NAMESPACE)
    }

    /// Profile version
    pub fn profile_version() -> String {
        format!("{}/profile-version", Self::NAMESPACE)
    }

    /// Barrier type encountered
    pub fn barrier_type() -> String {
        format!("{}/barrier-type", Self::NAMESPACE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_standard_verbs() {
        let completed = AccessibilityVerb::completed();
        assert_eq!(completed.id, "http://adlnet.gov/expapi/verbs/completed");
        assert!(completed.display.is_some());
    }

    #[test]
    fn test_accessibility_verbs() {
        let enabled = AccessibilityVerb::enabled_accessibility();
        assert!(enabled.id.starts_with("https://wia.live/xapi/verbs/"));

        let captions = AccessibilityVerb::enabled_captions();
        assert!(captions.display.as_ref().unwrap().contains_key("ko-KR"));
    }

    #[test]
    fn test_activity_types() {
        let profile = AccessibilityActivityType::accessibility_profile();
        assert!(profile.starts_with("https://wia.live/xapi/activities/"));
    }

    #[test]
    fn test_extensions() {
        let features = AccessibilityExtension::features_used();
        assert!(features.starts_with("https://wia.live/xapi/extensions/"));
    }
}
