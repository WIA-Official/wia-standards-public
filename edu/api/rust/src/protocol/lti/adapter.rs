//! LTI 1.3 Platform/Tool Adapter
//! 弘益人間 - Bridge between LMS and accessibility tools

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use std::collections::HashMap;

use crate::error::{EduError, Result};
use crate::types::LearnerProfile;
use super::messages::{
    LtiLaunchRequest, LtiDeepLinkingResponse, LtiResourceLink,
    LtiRole, LtiContext, ContentItem, ContentItemType,
    LtiMessage, LtiMessageType, DeepLinkingSettings,
};
use super::claims::AccessibilityClaims;

/// LTI Platform configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlatformConfig {
    /// Platform issuer URL
    pub issuer: String,
    /// Client ID for this tool
    pub client_id: String,
    /// Deployment ID
    pub deployment_id: String,
    /// JWKS URL for key verification
    pub jwks_url: String,
    /// Authorization endpoint
    pub auth_endpoint: String,
    /// Token endpoint
    pub token_endpoint: String,
}

/// LTI Tool configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolConfig {
    /// Tool's client ID
    pub client_id: String,
    /// Tool's launch URL
    pub launch_url: String,
    /// Deep linking URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deep_link_url: Option<String>,
    /// Tool's JWKS URL
    pub jwks_url: String,
    /// Tool name
    pub name: String,
    /// Tool description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// LTI Adapter for handling LTI 1.3 communications
pub struct LtiAdapter {
    /// Platform configuration
    platform: PlatformConfig,
    /// Tool configuration
    tool: ToolConfig,
    /// State storage for OIDC flow
    state_storage: HashMap<String, StateData>,
    /// Nonce storage for replay protection
    nonce_storage: HashMap<String, i64>,
}

/// State data for OIDC flow
#[derive(Debug, Clone)]
struct StateData {
    /// Original state value
    state: String,
    /// Nonce for this request
    nonce: String,
    /// Timestamp
    timestamp: i64,
    /// Target link URI
    target_link_uri: String,
}

impl LtiAdapter {
    /// Create a new LTI adapter
    pub fn new(platform: PlatformConfig, tool: ToolConfig) -> Self {
        Self {
            platform,
            tool,
            state_storage: HashMap::new(),
            nonce_storage: HashMap::new(),
        }
    }

    /// Generate OIDC login initiation URL
    pub fn generate_login_url(&mut self, target_link_uri: &str, login_hint: &str) -> String {
        let state = Uuid::new_v4().to_string();
        let nonce = Uuid::new_v4().to_string();

        // Store state for verification
        self.state_storage.insert(state.clone(), StateData {
            state: state.clone(),
            nonce: nonce.clone(),
            timestamp: chrono::Utc::now().timestamp(),
            target_link_uri: target_link_uri.to_string(),
        });

        format!(
            "{}?scope=openid&response_type=id_token&client_id={}&redirect_uri={}&login_hint={}&state={}&nonce={}&response_mode=form_post&lti_message_hint={}",
            self.platform.auth_endpoint,
            self.tool.client_id,
            urlencoding::encode(&self.tool.launch_url),
            urlencoding::encode(login_hint),
            state,
            nonce,
            urlencoding::encode(target_link_uri)
        )
    }

    /// Validate state from OIDC callback
    pub fn validate_state(&mut self, state: &str) -> Result<()> {
        if let Some(state_data) = self.state_storage.remove(state) {
            // Check if state is not too old (5 minutes)
            let now = chrono::Utc::now().timestamp();
            if now - state_data.timestamp > 300 {
                return Err(EduError::ValidationError("State expired".to_string()));
            }
            Ok(())
        } else {
            Err(EduError::ValidationError("Invalid state".to_string()))
        }
    }

    /// Validate nonce for replay protection
    pub fn validate_nonce(&mut self, nonce: &str) -> Result<()> {
        let now = chrono::Utc::now().timestamp();

        // Clean up old nonces (older than 1 hour)
        self.nonce_storage.retain(|_, ts| now - *ts < 3600);

        // Check if nonce was already used
        if self.nonce_storage.contains_key(nonce) {
            return Err(EduError::ValidationError("Nonce already used".to_string()));
        }

        // Store nonce
        self.nonce_storage.insert(nonce.to_string(), now);
        Ok(())
    }

    /// Create a launch request with accessibility claims
    pub fn create_launch_request(
        &self,
        user_id: &str,
        resource_link: LtiResourceLink,
        roles: Vec<LtiRole>,
        context: Option<LtiContext>,
        profile: Option<&LearnerProfile>,
    ) -> LtiLaunchRequest {
        let mut request = LtiLaunchRequest::new(
            self.platform.deployment_id.clone(),
            self.tool.launch_url.clone(),
            user_id.to_string(),
            self.platform.issuer.clone(),
            self.tool.client_id.clone(),
            resource_link,
            roles,
        );

        request.context = context;

        // Add accessibility claims if profile is provided
        if let Some(profile) = profile {
            let accessibility_claims = AccessibilityClaims::from_profile(
                profile.disability_profile.disability_types.clone(),
                &profile.display_preferences,
                &profile.control_preferences,
                &profile.content_preferences,
                &profile.assessment_accommodations,
                &profile.learning_style,
            );

            // Add as custom parameter (serialized JSON)
            if let Ok(json) = serde_json::to_string(&accessibility_claims) {
                request.custom.insert(
                    "wia_accessibility".to_string(),
                    json,
                );
            }
        }

        request
    }

    /// Extract accessibility claims from launch request
    pub fn extract_accessibility_claims(request: &LtiLaunchRequest) -> Option<AccessibilityClaims> {
        request.custom.get("wia_accessibility")
            .and_then(|json| serde_json::from_str(json).ok())
    }

    /// Create a deep linking response with accessible content
    pub fn create_deep_linking_response(
        &self,
        settings: &DeepLinkingSettings,
        items: Vec<AccessibleContentItem>,
    ) -> LtiDeepLinkingResponse {
        let content_items: Vec<ContentItem> = items
            .into_iter()
            .map(|item| item.into_content_item())
            .collect();

        LtiDeepLinkingResponse {
            message: LtiMessage::new(
                LtiMessageType::LtiDeepLinkingResponse,
                self.platform.deployment_id.clone(),
                settings.deep_link_return_url.clone(),
                "".to_string(), // No subject for deep linking response
                self.tool.client_id.clone(),
                self.platform.issuer.clone(),
            ),
            content_items,
            data: settings.data.clone(),
            msg: Some("Content items selected successfully".to_string()),
            log: None,
            error_msg: None,
            error_log: None,
        }
    }

    /// Get platform configuration
    pub fn platform(&self) -> &PlatformConfig {
        &self.platform
    }

    /// Get tool configuration
    pub fn tool(&self) -> &ToolConfig {
        &self.tool
    }
}

/// Accessible content item for deep linking
#[derive(Debug, Clone)]
pub struct AccessibleContentItem {
    /// Content type
    pub content_type: ContentItemType,
    /// Content title
    pub title: String,
    /// Content description
    pub description: Option<String>,
    /// Content URL
    pub url: String,
    /// Accessibility features available
    pub accessibility_features: AccessibilityFeatures,
}

/// Accessibility features for content
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AccessibilityFeatures {
    /// Has captions
    pub has_captions: bool,
    /// Has audio description
    pub has_audio_description: bool,
    /// Has transcript
    pub has_transcript: bool,
    /// Has sign language
    pub has_sign_language: bool,
    /// WCAG conformance level
    pub wcag_level: Option<String>,
    /// Screen reader compatible
    pub screen_reader_compatible: bool,
    /// Keyboard accessible
    pub keyboard_accessible: bool,
}

impl AccessibleContentItem {
    /// Convert to LTI content item
    pub fn into_content_item(self) -> ContentItem {
        let mut custom = HashMap::new();

        // Add accessibility features as custom parameters
        if let Ok(json) = serde_json::to_string(&self.accessibility_features) {
            custom.insert("wia_accessibility_features".to_string(), json);
        }

        ContentItem {
            content_type: self.content_type,
            title: Some(self.title),
            text: self.description,
            url: Some(self.url),
            icon: None,
            thumbnail: None,
            custom,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::ProfileManager;
    use crate::types::DisabilityType;

    fn create_test_adapter() -> LtiAdapter {
        let platform = PlatformConfig {
            issuer: "https://lms.example.com".to_string(),
            client_id: "tool-123".to_string(),
            deployment_id: "deploy-1".to_string(),
            jwks_url: "https://lms.example.com/.well-known/jwks.json".to_string(),
            auth_endpoint: "https://lms.example.com/auth".to_string(),
            token_endpoint: "https://lms.example.com/token".to_string(),
        };

        let tool = ToolConfig {
            client_id: "tool-123".to_string(),
            launch_url: "https://tool.example.com/launch".to_string(),
            deep_link_url: Some("https://tool.example.com/deep-link".to_string()),
            jwks_url: "https://tool.example.com/.well-known/jwks.json".to_string(),
            name: "WIA Accessible Learning Tool".to_string(),
            description: Some("Accessibility-first learning tool".to_string()),
        };

        LtiAdapter::new(platform, tool)
    }

    #[test]
    fn test_create_adapter() {
        let adapter = create_test_adapter();
        assert_eq!(adapter.platform().issuer, "https://lms.example.com");
        assert_eq!(adapter.tool().name, "WIA Accessible Learning Tool");
    }

    #[test]
    fn test_generate_login_url() {
        let mut adapter = create_test_adapter();
        let url = adapter.generate_login_url(
            "https://tool.example.com/course/123",
            "user-456",
        );

        assert!(url.starts_with("https://lms.example.com/auth?"));
        assert!(url.contains("scope=openid"));
        assert!(url.contains("response_type=id_token"));
    }

    #[test]
    fn test_state_validation() {
        let mut adapter = create_test_adapter();

        // Generate URL (creates state)
        let url = adapter.generate_login_url("https://tool.example.com", "user-1");

        // Extract state from URL
        let state = url.split("state=").nth(1).unwrap().split('&').next().unwrap();

        // Validate state
        assert!(adapter.validate_state(state).is_ok());

        // Second validation should fail (state consumed)
        assert!(adapter.validate_state(state).is_err());
    }

    #[test]
    fn test_nonce_validation() {
        let mut adapter = create_test_adapter();

        let nonce = "unique-nonce-123";

        // First validation should succeed
        assert!(adapter.validate_nonce(nonce).is_ok());

        // Second validation should fail (replay)
        assert!(adapter.validate_nonce(nonce).is_err());
    }

    #[test]
    fn test_create_launch_request() {
        let adapter = create_test_adapter();

        let resource_link = LtiResourceLink {
            id: "link-1".to_string(),
            title: Some("Accessible Course".to_string()),
            description: None,
        };

        let request = adapter.create_launch_request(
            "user-123",
            resource_link,
            vec![LtiRole::Learner],
            None,
            None,
        );

        assert_eq!(request.message.iss, "https://lms.example.com");
        assert!(request.is_learner());
    }

    #[test]
    fn test_extract_accessibility_claims() {
        let adapter = create_test_adapter();

        let mut manager = ProfileManager::new();
        let mut profile = manager.create_profile_for_disability(DisabilityType::Blind);
        profile.content_preferences.captions.required = true;

        let resource_link = LtiResourceLink {
            id: "link-1".to_string(),
            title: None,
            description: None,
        };

        let request = adapter.create_launch_request(
            "user-123",
            resource_link,
            vec![LtiRole::Learner],
            None,
            Some(&profile),
        );

        let claims = LtiAdapter::extract_accessibility_claims(&request);
        assert!(claims.is_some());

        let claims = claims.unwrap();
        assert!(claims.needs_screen_reader());
        assert!(claims.needs_captions());
    }
}
