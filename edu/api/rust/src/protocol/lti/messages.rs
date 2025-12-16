//! LTI 1.3 Message Types
//! 弘益人間 - Standard LTI message definitions

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::Utc;

/// LTI Message Types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum LtiMessageType {
    /// Resource link launch request
    LtiResourceLinkRequest,
    /// Deep linking request
    LtiDeepLinkingRequest,
    /// Deep linking response
    LtiDeepLinkingResponse,
    /// Submission review request
    LtiSubmissionReviewRequest,
}

/// LTI User Roles
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LtiRole {
    /// Learner/Student role
    Learner,
    /// Instructor/Teacher role
    Instructor,
    /// Administrator role
    Administrator,
    /// Content Developer role
    ContentDeveloper,
    /// Mentor/Advisor role
    Mentor,
    /// Teaching Assistant
    TeachingAssistant,
}

impl LtiRole {
    /// Get the IMS LTI role URI
    pub fn as_uri(&self) -> &'static str {
        match self {
            LtiRole::Learner => "http://purl.imsglobal.org/vocab/lis/v2/membership#Learner",
            LtiRole::Instructor => "http://purl.imsglobal.org/vocab/lis/v2/membership#Instructor",
            LtiRole::Administrator => "http://purl.imsglobal.org/vocab/lis/v2/membership#Administrator",
            LtiRole::ContentDeveloper => "http://purl.imsglobal.org/vocab/lis/v2/membership#ContentDeveloper",
            LtiRole::Mentor => "http://purl.imsglobal.org/vocab/lis/v2/membership#Mentor",
            LtiRole::TeachingAssistant => "http://purl.imsglobal.org/vocab/lis/v2/membership#Instructor#TeachingAssistant",
        }
    }
}

/// LTI Context (Course/Section)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LtiContext {
    /// Context identifier
    pub id: String,
    /// Context label (short name)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
    /// Context title (full name)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    /// Context types
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub r#type: Vec<String>,
}

/// LTI Resource Link
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LtiResourceLink {
    /// Resource link ID
    pub id: String,
    /// Resource link title
    #[serde(skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    /// Resource link description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// Base LTI Message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LtiMessage {
    /// Message type
    #[serde(rename = "https://purl.imsglobal.org/spec/lti/claim/message_type")]
    pub message_type: LtiMessageType,
    /// LTI version (always "1.3.0")
    #[serde(rename = "https://purl.imsglobal.org/spec/lti/claim/version")]
    pub version: String,
    /// Deployment ID
    #[serde(rename = "https://purl.imsglobal.org/spec/lti/claim/deployment_id")]
    pub deployment_id: String,
    /// Target link URI
    #[serde(rename = "https://purl.imsglobal.org/spec/lti/claim/target_link_uri")]
    pub target_link_uri: String,
    /// User subject (unique identifier)
    pub sub: String,
    /// Issued at timestamp
    pub iat: i64,
    /// Expiration timestamp
    pub exp: i64,
    /// Issuer (Platform)
    pub iss: String,
    /// Audience (Tool)
    pub aud: String,
    /// Nonce for replay protection
    pub nonce: String,
}

impl LtiMessage {
    /// Create a new LTI message
    pub fn new(
        message_type: LtiMessageType,
        deployment_id: String,
        target_link_uri: String,
        subject: String,
        issuer: String,
        audience: String,
    ) -> Self {
        let now = Utc::now().timestamp();
        Self {
            message_type,
            version: "1.3.0".to_string(),
            deployment_id,
            target_link_uri,
            sub: subject,
            iat: now,
            exp: now + 3600, // 1 hour validity
            iss: issuer,
            aud: audience,
            nonce: Uuid::new_v4().to_string(),
        }
    }

    /// Check if the message is expired
    pub fn is_expired(&self) -> bool {
        Utc::now().timestamp() > self.exp
    }
}

/// LTI Resource Link Launch Request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LtiLaunchRequest {
    /// Base message fields
    #[serde(flatten)]
    pub message: LtiMessage,
    /// Resource link information
    #[serde(rename = "https://purl.imsglobal.org/spec/lti/claim/resource_link")]
    pub resource_link: LtiResourceLink,
    /// User roles
    #[serde(rename = "https://purl.imsglobal.org/spec/lti/claim/roles")]
    pub roles: Vec<String>,
    /// Context (course/section)
    #[serde(rename = "https://purl.imsglobal.org/spec/lti/claim/context")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<LtiContext>,
    /// User's given name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub given_name: Option<String>,
    /// User's family name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub family_name: Option<String>,
    /// User's name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// User's email
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email: Option<String>,
    /// User's locale
    #[serde(skip_serializing_if = "Option::is_none")]
    pub locale: Option<String>,
    /// Custom parameters
    #[serde(rename = "https://purl.imsglobal.org/spec/lti/claim/custom")]
    #[serde(default, skip_serializing_if = "std::collections::HashMap::is_empty")]
    pub custom: std::collections::HashMap<String, String>,
}

impl LtiLaunchRequest {
    /// Create a new launch request
    pub fn new(
        deployment_id: String,
        target_link_uri: String,
        subject: String,
        issuer: String,
        audience: String,
        resource_link: LtiResourceLink,
        roles: Vec<LtiRole>,
    ) -> Self {
        Self {
            message: LtiMessage::new(
                LtiMessageType::LtiResourceLinkRequest,
                deployment_id,
                target_link_uri,
                subject,
                issuer,
                audience,
            ),
            resource_link,
            roles: roles.iter().map(|r| r.as_uri().to_string()).collect(),
            context: None,
            given_name: None,
            family_name: None,
            name: None,
            email: None,
            locale: None,
            custom: std::collections::HashMap::new(),
        }
    }

    /// Check if user has a specific role
    pub fn has_role(&self, role: &LtiRole) -> bool {
        let uri = role.as_uri();
        self.roles.iter().any(|r| r == uri)
    }

    /// Check if user is a learner
    pub fn is_learner(&self) -> bool {
        self.has_role(&LtiRole::Learner)
    }

    /// Check if user is an instructor
    pub fn is_instructor(&self) -> bool {
        self.has_role(&LtiRole::Instructor)
    }
}

/// LTI Deep Linking Request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LtiDeepLinkingRequest {
    /// Base message fields
    #[serde(flatten)]
    pub message: LtiMessage,
    /// Deep linking settings
    #[serde(rename = "https://purl.imsglobal.org/spec/lti-dl/claim/deep_linking_settings")]
    pub deep_linking_settings: DeepLinkingSettings,
}

/// Deep Linking Settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeepLinkingSettings {
    /// Return URL for deep linking response
    pub deep_link_return_url: String,
    /// Accepted content types
    pub accept_types: Vec<String>,
    /// Accepted presentation types
    #[serde(default)]
    pub accept_presentation_document_targets: Vec<String>,
    /// Whether multiple items can be selected
    #[serde(default)]
    pub accept_multiple: bool,
    /// Whether line items can be created
    #[serde(default)]
    pub accept_lineitem: bool,
    /// Auto-create line item
    #[serde(default)]
    pub auto_create: bool,
    /// Custom data to return
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<String>,
}

/// LTI Deep Linking Response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LtiDeepLinkingResponse {
    /// Base message fields
    #[serde(flatten)]
    pub message: LtiMessage,
    /// Content items being returned
    #[serde(rename = "https://purl.imsglobal.org/spec/lti-dl/claim/content_items")]
    pub content_items: Vec<ContentItem>,
    /// Data from deep linking settings
    #[serde(rename = "https://purl.imsglobal.org/spec/lti-dl/claim/data")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<String>,
    /// Message for platform
    #[serde(rename = "https://purl.imsglobal.org/spec/lti-dl/claim/msg")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub msg: Option<String>,
    /// Log message
    #[serde(rename = "https://purl.imsglobal.org/spec/lti-dl/claim/log")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub log: Option<String>,
    /// Error message
    #[serde(rename = "https://purl.imsglobal.org/spec/lti-dl/claim/errormsg")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_msg: Option<String>,
    /// Error log
    #[serde(rename = "https://purl.imsglobal.org/spec/lti-dl/claim/errorlog")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_log: Option<String>,
}

/// Content Item for Deep Linking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentItem {
    /// Content type
    #[serde(rename = "type")]
    pub content_type: ContentItemType,
    /// Content title
    #[serde(skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    /// Content text/description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
    /// URL for the content
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    /// Icon URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub icon: Option<ContentIcon>,
    /// Thumbnail URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub thumbnail: Option<ContentIcon>,
    /// Custom parameters
    #[serde(default, skip_serializing_if = "std::collections::HashMap::is_empty")]
    pub custom: std::collections::HashMap<String, String>,
}

/// Content Item Type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContentItemType {
    #[serde(rename = "ltiResourceLink")]
    LtiResourceLink,
    #[serde(rename = "link")]
    Link,
    #[serde(rename = "file")]
    File,
    #[serde(rename = "html")]
    Html,
    #[serde(rename = "image")]
    Image,
}

/// Content Icon/Thumbnail
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentIcon {
    /// Icon URL
    pub url: String,
    /// Icon width
    #[serde(skip_serializing_if = "Option::is_none")]
    pub width: Option<u32>,
    /// Icon height
    #[serde(skip_serializing_if = "Option::is_none")]
    pub height: Option<u32>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_launch_request() {
        let resource_link = LtiResourceLink {
            id: "link-123".to_string(),
            title: Some("Accessible Math Course".to_string()),
            description: None,
        };

        let request = LtiLaunchRequest::new(
            "deploy-1".to_string(),
            "https://tool.example.com/launch".to_string(),
            "user-456".to_string(),
            "https://platform.example.com".to_string(),
            "tool-client-id".to_string(),
            resource_link,
            vec![LtiRole::Learner],
        );

        assert_eq!(request.message.version, "1.3.0");
        assert!(request.is_learner());
        assert!(!request.is_instructor());
        assert!(!request.message.is_expired());
    }

    #[test]
    fn test_lti_roles() {
        assert_eq!(
            LtiRole::Learner.as_uri(),
            "http://purl.imsglobal.org/vocab/lis/v2/membership#Learner"
        );
        assert_eq!(
            LtiRole::Instructor.as_uri(),
            "http://purl.imsglobal.org/vocab/lis/v2/membership#Instructor"
        );
    }

    #[test]
    fn test_message_expiration() {
        let mut message = LtiMessage::new(
            LtiMessageType::LtiResourceLinkRequest,
            "deploy-1".to_string(),
            "https://tool.example.com".to_string(),
            "user-1".to_string(),
            "https://platform.example.com".to_string(),
            "tool-id".to_string(),
        );

        assert!(!message.is_expired());

        // Set expiration to past
        message.exp = Utc::now().timestamp() - 100;
        assert!(message.is_expired());
    }
}
