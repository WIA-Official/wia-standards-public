//! Security and privacy framework

use serde::{Deserialize, Serialize};
use crate::types::Timestamp;

/// Data classification levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataClassification {
    /// Internal data (device status, logs)
    Internal,
    /// Private data (conversations, emotions)
    Private,
    /// Sensitive data (location, daily patterns)
    Sensitive,
    /// Critical data (medical info, vitals)
    Critical,
}

impl DataClassification {
    /// Get retention period in days
    pub fn retention_days(&self) -> u32 {
        match self {
            DataClassification::Internal => 90,
            DataClassification::Private => 30,
            DataClassification::Sensitive => 365,
            DataClassification::Critical => 365 * 7, // 7 years for medical
        }
    }

    /// Check if encryption is required
    pub fn requires_encryption(&self) -> bool {
        matches!(
            self,
            DataClassification::Private
                | DataClassification::Sensitive
                | DataClassification::Critical
        )
    }
}

/// Consent management
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsentRecord {
    /// Recipient ID
    pub recipient_id: String,
    /// Guardian ID (if consent given by guardian)
    pub guardian_id: Option<String>,
    /// Who granted consent
    pub granted_by: ConsentGrantor,
    /// Data collection consents
    pub data_collection: DataCollectionConsent,
    /// Data sharing consents
    pub data_sharing: DataSharingConsent,
    /// Automated action consents
    pub automated_actions: AutomatedActionConsent,
    /// Valid from date
    pub valid_from: String,
    /// Valid until date
    pub valid_until: String,
    /// Review required date
    pub review_required: String,
    /// Last updated
    pub updated_at: Timestamp,
}

impl ConsentRecord {
    /// Create a new consent record
    pub fn new(recipient_id: &str) -> Self {
        let now = chrono::Utc::now();
        Self {
            recipient_id: recipient_id.to_string(),
            guardian_id: None,
            granted_by: ConsentGrantor::Recipient,
            data_collection: DataCollectionConsent::default(),
            data_sharing: DataSharingConsent::default(),
            automated_actions: AutomatedActionConsent::default(),
            valid_from: now.format("%Y-%m-%d").to_string(),
            valid_until: (now + chrono::Duration::days(365)).format("%Y-%m-%d").to_string(),
            review_required: (now + chrono::Duration::days(180)).format("%Y-%m-%d").to_string(),
            updated_at: Timestamp::now(),
        }
    }

    /// Set guardian as consent grantor
    pub fn by_guardian(mut self, guardian_id: &str) -> Self {
        self.guardian_id = Some(guardian_id.to_string());
        self.granted_by = ConsentGrantor::Guardian;
        self
    }

    /// Enable all essential consents for care
    pub fn enable_essential_care(mut self) -> Self {
        self.data_collection.vital_signs = true;
        self.data_collection.location_tracking = true;
        self.data_collection.emotion_analysis = true;
        self.data_sharing.family = true;
        self.data_sharing.care_team = true;
        self.data_sharing.emergency_services = true;
        self.automated_actions.emergency_call = true;
        self.automated_actions.family_notification = true;
        self.automated_actions.medication_reminder = true;
        self
    }

    /// Check if consent is currently valid
    pub fn is_valid(&self) -> bool {
        let today = chrono::Utc::now().format("%Y-%m-%d").to_string();
        today >= self.valid_from && today <= self.valid_until
    }

    /// Check if review is needed
    pub fn needs_review(&self) -> bool {
        let today = chrono::Utc::now().format("%Y-%m-%d").to_string();
        today >= self.review_required
    }

    /// Check if a specific data type can be collected
    pub fn can_collect(&self, data_type: &str) -> bool {
        if !self.is_valid() {
            return false;
        }

        match data_type {
            "vital_signs" => self.data_collection.vital_signs,
            "location" => self.data_collection.location_tracking,
            "emotion" => self.data_collection.emotion_analysis,
            "conversation" => self.data_collection.conversation_recording,
            _ => false,
        }
    }

    /// Check if data can be shared with a specific party
    pub fn can_share_with(&self, party: &str) -> bool {
        if !self.is_valid() {
            return false;
        }

        match party {
            "family" => self.data_sharing.family,
            "care_team" => self.data_sharing.care_team,
            "emergency" => self.data_sharing.emergency_services,
            "research" => self.data_sharing.research_anonymized,
            _ => false,
        }
    }
}

/// Who granted consent
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConsentGrantor {
    Recipient,
    Guardian,
    LegalRepresentative,
}

/// Data collection consent settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DataCollectionConsent {
    pub vital_signs: bool,
    pub location_tracking: bool,
    pub emotion_analysis: bool,
    pub conversation_recording: bool,
}

/// Data sharing consent settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DataSharingConsent {
    pub family: bool,
    pub care_team: bool,
    pub emergency_services: bool,
    pub research_anonymized: bool,
}

/// Automated action consent settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AutomatedActionConsent {
    pub emergency_call: bool,
    pub family_notification: bool,
    pub medication_reminder: bool,
}

/// Device authentication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceAuth {
    /// Device ID
    pub device_id: String,
    /// X.509 certificate
    pub certificate: Option<String>,
    /// TPM attestation
    pub attestation: Option<String>,
    /// Registration info
    pub registration: DeviceRegistration,
}

impl DeviceAuth {
    /// Create new device authentication
    pub fn new(device_id: &str, owner_id: &str, recipient_id: &str) -> Self {
        Self {
            device_id: device_id.to_string(),
            certificate: None,
            attestation: None,
            registration: DeviceRegistration {
                owner_id: owner_id.to_string(),
                recipient_id: recipient_id.to_string(),
                installation_date: chrono::Utc::now().format("%Y-%m-%d").to_string(),
                verified: false,
            },
        }
    }

    /// Set device certificate
    pub fn with_certificate(mut self, cert: &str) -> Self {
        self.certificate = Some(cert.to_string());
        self
    }

    /// Mark as verified
    pub fn verify(&mut self) {
        self.registration.verified = true;
    }
}

/// Device registration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceRegistration {
    pub owner_id: String,
    pub recipient_id: String,
    pub installation_date: String,
    pub verified: bool,
}

/// Family app authentication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FamilyAuth {
    /// Authentication method
    pub method: AuthMethod,
    /// MFA configuration
    pub mfa: MfaConfig,
    /// User role
    pub role: FamilyRole,
    /// Permissions
    pub permissions: Vec<Permission>,
}

impl FamilyAuth {
    /// Create authentication for primary guardian
    pub fn primary_guardian() -> Self {
        Self {
            method: AuthMethod::OAuth2,
            mfa: MfaConfig {
                required: true,
                methods: vec![MfaMethod::AuthenticatorApp, MfaMethod::Sms],
            },
            role: FamilyRole::PrimaryGuardian,
            permissions: vec![
                Permission::FullAccess,
                Permission::EmergencyContact,
                Permission::ManageConsent,
            ],
        }
    }

    /// Create authentication for family member
    pub fn family_member() -> Self {
        Self {
            method: AuthMethod::OAuth2,
            mfa: MfaConfig {
                required: false,
                methods: vec![MfaMethod::Sms],
            },
            role: FamilyRole::FamilyMember,
            permissions: vec![
                Permission::View,
                Permission::VideoCall,
                Permission::ReceiveAlerts,
            ],
        }
    }

    /// Create authentication for care worker
    pub fn care_worker() -> Self {
        Self {
            method: AuthMethod::OAuth2,
            mfa: MfaConfig {
                required: true,
                methods: vec![MfaMethod::AuthenticatorApp],
            },
            role: FamilyRole::CareWorker,
            permissions: vec![Permission::ViewHealth, Permission::DailyReport],
        }
    }

    /// Check if user has permission
    pub fn has_permission(&self, permission: Permission) -> bool {
        self.permissions.contains(&Permission::FullAccess)
            || self.permissions.contains(&permission)
    }
}

/// Authentication method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthMethod {
    OAuth2,
    Saml,
    ApiKey,
}

/// MFA configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MfaConfig {
    pub required: bool,
    pub methods: Vec<MfaMethod>,
}

/// MFA methods
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MfaMethod {
    AuthenticatorApp,
    Sms,
    Email,
    BiometricDevice,
}

/// Family member roles
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FamilyRole {
    PrimaryGuardian,
    FamilyMember,
    CareWorker,
}

/// Permissions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Permission {
    FullAccess,
    View,
    ViewHealth,
    VideoCall,
    ReceiveAlerts,
    EmergencyContact,
    ManageConsent,
    DailyReport,
}

/// Audit log entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditLogEntry {
    /// Log ID
    pub log_id: String,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Actor (who performed the action)
    pub actor: AuditActor,
    /// Action performed
    pub action: AuditAction,
    /// Resource accessed
    pub resource: String,
    /// Outcome
    pub outcome: AuditOutcome,
    /// Additional details
    pub details: Option<String>,
}

impl AuditLogEntry {
    /// Create a new audit log entry
    pub fn new(actor: AuditActor, action: AuditAction, resource: &str) -> Self {
        Self {
            log_id: format!("audit-{}", uuid::Uuid::new_v4()),
            timestamp: Timestamp::now(),
            actor,
            action,
            resource: resource.to_string(),
            outcome: AuditOutcome::Success,
            details: None,
        }
    }

    /// Mark as failed
    pub fn failed(mut self, reason: &str) -> Self {
        self.outcome = AuditOutcome::Failure;
        self.details = Some(reason.to_string());
        self
    }
}

/// Audit actor
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditActor {
    pub actor_type: ActorType,
    pub actor_id: String,
    pub ip_address: Option<String>,
}

/// Actor types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActorType {
    Device,
    FamilyMember,
    CareTeam,
    System,
    Admin,
}

/// Audit actions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuditAction {
    DataAccess,
    DataModify,
    DataDelete,
    DataExport,
    ConsentUpdate,
    EmergencyAccess,
    Login,
    Logout,
    ConfigChange,
}

/// Audit outcome
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuditOutcome {
    Success,
    Failure,
    Denied,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_consent_record() {
        let consent = ConsentRecord::new("recipient-001")
            .by_guardian("family-001")
            .enable_essential_care();

        assert!(consent.is_valid());
        assert!(consent.can_collect("vital_signs"));
        assert!(consent.can_share_with("family"));
        assert!(!consent.can_collect("conversation")); // Not enabled by default
    }

    #[test]
    fn test_data_classification() {
        assert!(DataClassification::Critical > DataClassification::Sensitive);
        assert!(DataClassification::Critical.requires_encryption());
        assert!(!DataClassification::Internal.requires_encryption());
    }

    #[test]
    fn test_family_auth_permissions() {
        let guardian = FamilyAuth::primary_guardian();
        assert!(guardian.has_permission(Permission::FullAccess));
        assert!(guardian.has_permission(Permission::ViewHealth));

        let member = FamilyAuth::family_member();
        assert!(member.has_permission(Permission::VideoCall));
        assert!(!member.has_permission(Permission::ManageConsent));
    }

    #[test]
    fn test_device_auth() {
        let mut auth = DeviceAuth::new("carebot-001", "owner-001", "recipient-001");
        assert!(!auth.registration.verified);

        auth.verify();
        assert!(auth.registration.verified);
    }
}
