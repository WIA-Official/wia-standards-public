//! Care recipient profile and management

use serde::{Deserialize, Serialize};
use chrono::NaiveDate;
use crate::types::*;

/// Care recipient profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CareRecipient {
    /// Unique recipient identifier
    pub id: String,
    /// Profile information
    pub profile: RecipientProfile,
    /// Health information
    pub health: HealthInfo,
    /// Medications
    pub medications: Vec<Medication>,
    /// Preferences
    pub preferences: RecipientPreferences,
    /// Emergency contacts
    pub emergency_contacts: Vec<EmergencyContact>,
    /// Care team members
    pub care_team: Vec<CareTeamMember>,
}

impl CareRecipient {
    /// Create a new care recipient
    pub fn new(id: &str, name: &str) -> Self {
        Self {
            id: id.to_string(),
            profile: RecipientProfile::new(name),
            health: HealthInfo::default(),
            medications: Vec::new(),
            preferences: RecipientPreferences::default(),
            emergency_contacts: Vec::new(),
            care_team: Vec::new(),
        }
    }

    /// Set preferred name (nickname)
    pub fn with_preferred_name(mut self, name: &str) -> Self {
        self.profile.preferred_name = Some(name.to_string());
        self
    }

    /// Set cognitive level
    pub fn with_cognitive_level(mut self, level: CognitiveLevel) -> Self {
        self.health.cognitive_level = Some(level);
        self
    }

    /// Set mobility level
    pub fn with_mobility_level(mut self, level: MobilityLevel) -> Self {
        self.health.mobility_level = Some(level);
        self
    }

    /// Add emergency contact
    pub fn add_emergency_contact(&mut self, contact: EmergencyContact) {
        self.emergency_contacts.push(contact);
        self.emergency_contacts.sort_by_key(|c| c.priority);
    }

    /// Add medication
    pub fn add_medication(&mut self, medication: Medication) {
        self.medications.push(medication);
    }

    /// Get primary emergency contact
    pub fn primary_emergency_contact(&self) -> Option<&EmergencyContact> {
        self.emergency_contacts.first()
    }

    /// Check if recipient has cognitive impairment
    pub fn has_cognitive_impairment(&self) -> bool {
        matches!(
            self.health.cognitive_level,
            Some(CognitiveLevel::MildImpairment)
                | Some(CognitiveLevel::ModerateImpairment)
                | Some(CognitiveLevel::SevereImpairment)
        )
    }
}

/// Recipient profile information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecipientProfile {
    /// Full name
    pub name: String,
    /// Preferred name (nickname)
    pub preferred_name: Option<String>,
    /// Birth date
    pub birth_date: Option<NaiveDate>,
    /// Gender
    pub gender: Option<Gender>,
    /// Blood type
    pub blood_type: Option<String>,
    /// Photo URL
    pub photo_url: Option<String>,
}

impl RecipientProfile {
    /// Create new profile with name
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            preferred_name: None,
            birth_date: None,
            gender: None,
            blood_type: None,
            photo_url: None,
        }
    }

    /// Get display name (preferred name if set, otherwise full name)
    pub fn display_name(&self) -> &str {
        self.preferred_name.as_deref().unwrap_or(&self.name)
    }
}

/// Health information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HealthInfo {
    /// Medical conditions
    pub conditions: Vec<String>,
    /// Allergies
    pub allergies: Vec<String>,
    /// Mobility level
    pub mobility_level: Option<MobilityLevel>,
    /// Cognitive level
    pub cognitive_level: Option<CognitiveLevel>,
    /// Hearing level
    pub hearing_level: Option<HearingLevel>,
    /// Vision level
    pub vision_level: Option<VisionLevel>,
}

/// Medication information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Medication {
    /// Medication name
    pub name: String,
    /// Dosage (e.g., "10mg")
    pub dosage: String,
    /// Frequency (e.g., "twice daily")
    pub frequency: String,
    /// Specific times
    pub times: Vec<String>,
    /// Take with food
    pub with_food: bool,
    /// Additional notes
    pub notes: Option<String>,
}

impl Medication {
    /// Create new medication
    pub fn new(name: &str, dosage: &str, frequency: &str) -> Self {
        Self {
            name: name.to_string(),
            dosage: dosage.to_string(),
            frequency: frequency.to_string(),
            times: Vec::new(),
            with_food: false,
            notes: None,
        }
    }

    /// Add scheduled time
    pub fn with_time(mut self, time: &str) -> Self {
        self.times.push(time.to_string());
        self
    }

    /// Set take with food
    pub fn with_food(mut self) -> Self {
        self.with_food = true;
        self
    }
}

/// Emergency contact
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyContact {
    /// Contact name
    pub name: String,
    /// Relationship to recipient
    pub relationship: String,
    /// Phone number
    pub phone: String,
    /// Priority (1 = highest)
    pub priority: u8,
    /// Events to notify on
    pub notify_on: Vec<String>,
}

impl EmergencyContact {
    /// Create new emergency contact
    pub fn new(name: &str, relationship: &str, phone: &str, priority: u8) -> Self {
        Self {
            name: name.to_string(),
            relationship: relationship.to_string(),
            phone: phone.to_string(),
            priority,
            notify_on: vec![
                "fall".to_string(),
                "emergency".to_string(),
                "sos".to_string(),
            ],
        }
    }
}

/// Care team member
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CareTeamMember {
    /// Name
    pub name: String,
    /// Role (e.g., "주치의", "간병인")
    pub role: String,
    /// Hospital/Organization
    pub hospital: Option<String>,
    /// Phone number
    pub phone: Option<String>,
    /// Specialty
    pub specialty: Option<String>,
}

/// Recipient preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecipientPreferences {
    /// Wake time
    pub wake_time: Option<String>,
    /// Sleep time
    pub sleep_time: Option<String>,
    /// Preferred language
    pub preferred_language: String,
    /// Voice speed preference
    pub voice_speed: VoiceSpeed,
    /// Voice volume preference
    pub voice_volume: VoiceVolume,
    /// Preferred conversation topics
    pub preferred_topics: Vec<String>,
    /// Favorite music genres
    pub music_genres: Vec<String>,
    /// Conversation style
    pub conversation_style: Option<String>,
}

impl Default for RecipientPreferences {
    fn default() -> Self {
        Self {
            wake_time: Some("07:00".to_string()),
            sleep_time: Some("22:00".to_string()),
            preferred_language: "ko-KR".to_string(),
            voice_speed: VoiceSpeed::Normal,
            voice_volume: VoiceVolume::Normal,
            preferred_topics: Vec::new(),
            music_genres: Vec::new(),
            conversation_style: None,
        }
    }
}
