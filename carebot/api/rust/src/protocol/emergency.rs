//! Emergency protocol implementation

use serde::{Deserialize, Serialize};
use crate::types::Timestamp;

/// Emergency dispatch request for 119 integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyDispatch {
    /// Call ID
    pub call_id: String,
    /// Source system
    pub source: String,
    /// Incident information
    pub incident: IncidentInfo,
    /// Location information
    pub location: EmergencyLocation,
    /// Patient information
    pub patient: PatientInfo,
    /// Contact information
    pub contacts: Vec<EmergencyContact>,
    /// Timestamp
    pub timestamp: Timestamp,
}

impl EmergencyDispatch {
    /// Create a new emergency dispatch request
    pub fn new(
        incident_type: EmergencyType,
        recipient_id: &str,
        recipient_name: &str,
        age: u8,
        address: &str,
    ) -> Self {
        let call_id = format!(
            "119-{}-{}",
            chrono::Utc::now().format("%Y%m%d-%H%M%S"),
            uuid::Uuid::new_v4().to_string().split('-').next().unwrap()
        );

        Self {
            call_id,
            source: "WIA-CareBot".to_string(),
            incident: IncidentInfo {
                incident_type,
                subtype: None,
                priority: 1,
                description: None,
            },
            location: EmergencyLocation {
                address: address.to_string(),
                coordinates: None,
                access_notes: None,
            },
            patient: PatientInfo {
                id: recipient_id.to_string(),
                name: recipient_name.to_string(),
                age,
                gender: None,
                conditions: Vec::new(),
                medications: Vec::new(),
                allergies: Vec::new(),
                vitals_at_event: None,
            },
            contacts: Vec::new(),
            timestamp: Timestamp::now(),
        }
    }

    /// Create dispatch for fall detection
    pub fn fall_detected(
        recipient_id: &str,
        recipient_name: &str,
        age: u8,
        address: &str,
        room: &str,
    ) -> Self {
        let mut dispatch = Self::new(
            EmergencyType::MedicalEmergency,
            recipient_id,
            recipient_name,
            age,
            address,
        );
        dispatch.incident.subtype = Some("fall_elderly".to_string());
        dispatch.incident.description = Some(format!("{}에서 낙상 감지", room));
        dispatch
    }

    /// Create dispatch for SOS button
    pub fn sos_button(
        recipient_id: &str,
        recipient_name: &str,
        age: u8,
        address: &str,
    ) -> Self {
        let mut dispatch = Self::new(
            EmergencyType::MedicalEmergency,
            recipient_id,
            recipient_name,
            age,
            address,
        );
        dispatch.incident.subtype = Some("sos_activation".to_string());
        dispatch.incident.description = Some("SOS 버튼 활성화".to_string());
        dispatch
    }

    /// Create dispatch for vital sign emergency
    pub fn vital_emergency(
        recipient_id: &str,
        recipient_name: &str,
        age: u8,
        address: &str,
        vital_type: &str,
        value: &str,
    ) -> Self {
        let mut dispatch = Self::new(
            EmergencyType::MedicalEmergency,
            recipient_id,
            recipient_name,
            age,
            address,
        );
        dispatch.incident.subtype = Some("vital_sign_critical".to_string());
        dispatch.incident.description = Some(format!("{}: {}", vital_type, value));
        dispatch
    }

    /// Add patient medical conditions
    pub fn with_conditions(mut self, conditions: Vec<String>) -> Self {
        self.patient.conditions = conditions;
        self
    }

    /// Add patient medications
    pub fn with_medications(mut self, medications: Vec<String>) -> Self {
        self.patient.medications = medications;
        self
    }

    /// Add patient allergies
    pub fn with_allergies(mut self, allergies: Vec<String>) -> Self {
        self.patient.allergies = allergies;
        self
    }

    /// Add location coordinates
    pub fn with_coordinates(mut self, lat: f64, lng: f64) -> Self {
        self.location.coordinates = Some(Coordinates { lat, lng });
        self
    }

    /// Add access notes
    pub fn with_access_notes(mut self, notes: &str) -> Self {
        self.location.access_notes = Some(notes.to_string());
        self
    }

    /// Add emergency contact
    pub fn add_contact(&mut self, name: &str, relationship: &str, phone: &str) {
        self.contacts.push(EmergencyContact {
            name: name.to_string(),
            relationship: relationship.to_string(),
            phone: phone.to_string(),
        });
    }

    /// Set current vitals
    pub fn with_vitals(mut self, heart_rate: u16, blood_pressure: &str) -> Self {
        self.patient.vitals_at_event = Some(VitalsAtEvent {
            heart_rate: Some(heart_rate),
            blood_pressure: Some(blood_pressure.to_string()),
            spo2: None,
            temperature: None,
        });
        self
    }

    /// Convert to CAD-XML format for 119 integration
    pub fn to_cad_xml(&self) -> String {
        format!(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<CAD_Dispatch>
  <CallID>{}</CallID>
  <Source>{}</Source>
  <Incident>
    <Type>{:?}</Type>
    <Subtype>{}</Subtype>
    <Priority>{}</Priority>
    <Description>{}</Description>
  </Incident>
  <Location>
    <Address>{}</Address>
    <AccessNotes>{}</AccessNotes>
  </Location>
  <Patient>
    <Name>{}</Name>
    <Age>{}</Age>
    <Conditions>{}</Conditions>
    <Medications>{}</Medications>
    <Allergies>{}</Allergies>
  </Patient>
  <Timestamp>{}</Timestamp>
</CAD_Dispatch>"#,
            self.call_id,
            self.source,
            self.incident.incident_type,
            self.incident.subtype.as_deref().unwrap_or(""),
            self.incident.priority,
            self.incident.description.as_deref().unwrap_or(""),
            self.location.address,
            self.location.access_notes.as_deref().unwrap_or(""),
            self.patient.name,
            self.patient.age,
            self.patient.conditions.join(", "),
            self.patient.medications.join(", "),
            self.patient.allergies.join(", "),
            chrono::Utc::now().to_rfc3339()
        )
    }
}

/// Emergency type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EmergencyType {
    MedicalEmergency,
    FireEmergency,
    SecurityEmergency,
    NaturalDisaster,
}

/// Incident information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IncidentInfo {
    /// Type of incident
    #[serde(rename = "type")]
    pub incident_type: EmergencyType,
    /// Subtype for more specific classification
    pub subtype: Option<String>,
    /// Priority (1 = highest)
    pub priority: u8,
    /// Description
    pub description: Option<String>,
}

/// Emergency location
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyLocation {
    /// Full address
    pub address: String,
    /// GPS coordinates
    pub coordinates: Option<Coordinates>,
    /// Access notes (door code, floor, etc.)
    pub access_notes: Option<String>,
}

/// GPS coordinates
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Coordinates {
    pub lat: f64,
    pub lng: f64,
}

/// Patient information for emergency dispatch
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PatientInfo {
    /// Patient/Recipient ID
    pub id: String,
    /// Name
    pub name: String,
    /// Age
    pub age: u8,
    /// Gender
    pub gender: Option<String>,
    /// Medical conditions
    pub conditions: Vec<String>,
    /// Current medications
    pub medications: Vec<String>,
    /// Known allergies
    pub allergies: Vec<String>,
    /// Vitals at time of event
    pub vitals_at_event: Option<VitalsAtEvent>,
}

/// Vitals at time of emergency
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VitalsAtEvent {
    pub heart_rate: Option<u16>,
    pub blood_pressure: Option<String>,
    pub spo2: Option<u8>,
    pub temperature: Option<f32>,
}

/// Emergency contact
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyContact {
    pub name: String,
    pub relationship: String,
    pub phone: String,
}

/// Emergency response protocol handler
#[derive(Debug, Clone)]
pub struct EmergencyProtocol {
    /// Escalation levels
    escalation_levels: Vec<EscalationLevel>,
    /// Current escalation level
    current_level: usize,
    /// Response timeout in seconds
    response_timeout: u32,
}

impl Default for EmergencyProtocol {
    fn default() -> Self {
        Self {
            escalation_levels: vec![
                EscalationLevel {
                    level: 1,
                    name: "Voice Prompt".to_string(),
                    timeout_seconds: 30,
                    actions: vec![EscalationAction::VoicePrompt],
                },
                EscalationLevel {
                    level: 2,
                    name: "Family Notification".to_string(),
                    timeout_seconds: 60,
                    actions: vec![EscalationAction::FamilyPush, EscalationAction::FamilySms],
                },
                EscalationLevel {
                    level: 3,
                    name: "Care Team Alert".to_string(),
                    timeout_seconds: 120,
                    actions: vec![EscalationAction::CareTeamCall],
                },
                EscalationLevel {
                    level: 4,
                    name: "Emergency Services".to_string(),
                    timeout_seconds: 0,
                    actions: vec![EscalationAction::Call119],
                },
            ],
            current_level: 0,
            response_timeout: 30,
        }
    }
}

impl EmergencyProtocol {
    /// Create a new emergency protocol
    pub fn new() -> Self {
        Self::default()
    }

    /// Get the current escalation level
    pub fn current_level(&self) -> &EscalationLevel {
        &self.escalation_levels[self.current_level]
    }

    /// Escalate to next level
    pub fn escalate(&mut self) -> Option<&EscalationLevel> {
        if self.current_level < self.escalation_levels.len() - 1 {
            self.current_level += 1;
            Some(&self.escalation_levels[self.current_level])
        } else {
            None
        }
    }

    /// Reset to first level
    pub fn reset(&mut self) {
        self.current_level = 0;
    }

    /// Skip to emergency services (highest level)
    pub fn immediate_emergency(&mut self) -> &EscalationLevel {
        self.current_level = self.escalation_levels.len() - 1;
        &self.escalation_levels[self.current_level]
    }

    /// Check if at highest escalation level
    pub fn is_max_escalation(&self) -> bool {
        self.current_level == self.escalation_levels.len() - 1
    }
}

/// Escalation level definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EscalationLevel {
    pub level: u8,
    pub name: String,
    pub timeout_seconds: u32,
    pub actions: Vec<EscalationAction>,
}

/// Escalation actions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EscalationAction {
    VoicePrompt,
    FamilyPush,
    FamilySms,
    FamilyCall,
    CareTeamCall,
    Call119,
    VideoCall,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_emergency_dispatch_creation() {
        let dispatch = EmergencyDispatch::fall_detected(
            "recipient-001",
            "김영희",
            78,
            "서울시 강남구 역삼동 123-45",
            "bathroom",
        );

        assert!(dispatch.call_id.starts_with("119-"));
        assert_eq!(dispatch.incident.incident_type, EmergencyType::MedicalEmergency);
        assert_eq!(dispatch.patient.name, "김영희");
    }

    #[test]
    fn test_emergency_protocol_escalation() {
        let mut protocol = EmergencyProtocol::new();

        assert_eq!(protocol.current_level().level, 1);

        protocol.escalate();
        assert_eq!(protocol.current_level().level, 2);

        protocol.immediate_emergency();
        assert!(protocol.is_max_escalation());
    }

    #[test]
    fn test_cad_xml_generation() {
        let dispatch = EmergencyDispatch::fall_detected(
            "recipient-001",
            "김영희",
            78,
            "서울시 강남구",
            "bathroom",
        )
        .with_conditions(vec!["고혈압".to_string()])
        .with_medications(vec!["혈압약".to_string()]);

        let xml = dispatch.to_cad_xml();

        assert!(xml.contains("<Name>김영희</Name>"));
        assert!(xml.contains("<Age>78</Age>"));
        assert!(xml.contains("고혈압"));
    }
}
