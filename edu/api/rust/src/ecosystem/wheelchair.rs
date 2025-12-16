//! Smart Wheelchair Integration
//! 弘益人間 - Enable learning during mobility with smart wheelchair

use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::error::{EduError, Result};

/// Wheelchair Education trait for learning during mobility
pub trait WheelchairEducation: Send + Sync {
    /// Get current location context
    fn get_location_context(&self) -> Option<LocationContext>;

    /// Set learning mode based on movement state
    fn set_learning_mode(&mut self, mode: LearningMode) -> Result<()>;

    /// Get recommended content format for current state
    fn recommended_content_format(&self) -> ContentFormat;

    /// Check if safe to display visual content
    fn is_safe_for_visual(&self) -> bool;

    /// Get audio learning settings
    fn audio_learning_settings(&self) -> AudioSettings;

    /// Report position change for context-aware learning
    fn on_position_change(&mut self, position: Position);

    /// Check if wheelchair is connected
    fn is_connected(&self) -> bool;
}

/// Location context for learning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocationContext {
    /// Current position
    pub position: Position,
    /// Location type
    pub location_type: LocationType,
    /// Location name (if known)
    pub location_name: Option<String>,
    /// Noise level (0.0-1.0)
    pub noise_level: f32,
    /// Lighting level (0.0-1.0)
    pub lighting_level: f32,
    /// Movement state
    pub movement_state: MovementState,
    /// Nearby learning resources
    pub nearby_resources: Vec<NearbyResource>,
}

/// Position coordinates
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Position {
    /// X coordinate (meters from origin)
    pub x: f32,
    /// Y coordinate (meters from origin)
    pub y: f32,
    /// Floor/level
    pub floor: i32,
    /// Heading (degrees, 0-360)
    pub heading: f32,
}

/// Location type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LocationType {
    /// Classroom
    Classroom,
    /// Library
    Library,
    /// Study room
    StudyRoom,
    /// Hallway/corridor
    Hallway,
    /// Outdoors
    Outdoors,
    /// Home
    Home,
    /// Laboratory
    Laboratory,
    /// Cafeteria
    Cafeteria,
    /// Unknown
    Unknown,
}

/// Movement state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MovementState {
    /// Stationary
    Stationary,
    /// Moving slowly
    MovingSlow,
    /// Moving at normal speed
    MovingNormal,
    /// Moving fast
    MovingFast,
    /// Turning
    Turning,
    /// Stopped at destination
    AtDestination,
}

/// Learning mode based on mobility state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LearningMode {
    /// Full interactive learning (stationary)
    FullInteractive,
    /// Audio-only learning (moving)
    AudioOnly,
    /// Passive learning (listening/ambient)
    Passive,
    /// Review mode (quick summaries)
    Review,
    /// Paused (safety priority)
    Paused,
}

/// Content format recommendation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContentFormat {
    /// Full multimedia
    FullMultimedia,
    /// Audio with simple visuals
    AudioWithVisuals,
    /// Audio only
    AudioOnly,
    /// Text with TTS
    TextWithTTS,
    /// No content (safety)
    None,
}

/// Audio settings for learning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioSettings {
    /// Volume level (0.0-1.0)
    pub volume: f32,
    /// Speech rate (0.5-2.0)
    pub speech_rate: f32,
    /// Use bone conduction (for ambient awareness)
    pub bone_conduction: bool,
    /// Audio description enabled
    pub audio_description: bool,
    /// Background audio ducking
    pub background_ducking: bool,
}

impl Default for AudioSettings {
    fn default() -> Self {
        Self {
            volume: 0.7,
            speech_rate: 1.0,
            bone_conduction: false,
            audio_description: true,
            background_ducking: true,
        }
    }
}

/// Nearby learning resource
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NearbyResource {
    /// Resource ID
    pub id: String,
    /// Resource name
    pub name: String,
    /// Resource type
    pub resource_type: ResourceType,
    /// Distance in meters
    pub distance_m: f32,
    /// Direction (degrees)
    pub direction: f32,
}

/// Type of nearby resource
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResourceType {
    /// Interactive display/kiosk
    InteractiveDisplay,
    /// AR marker
    ARMarker,
    /// Beacon with learning content
    LearningBeacon,
    /// Study space
    StudySpace,
    /// Group learning area
    GroupArea,
}

/// Wheelchair Adapter implementation
pub struct WheelchairAdapter {
    /// Current position
    current_position: Option<Position>,
    /// Current movement state
    movement_state: MovementState,
    /// Current learning mode
    learning_mode: LearningMode,
    /// Location database (simplified)
    known_locations: Vec<KnownLocation>,
    /// Audio settings
    audio_settings: AudioSettings,
    /// Connection status
    connected: bool,
}

/// Known location in the system
#[derive(Debug, Clone)]
struct KnownLocation {
    name: String,
    location_type: LocationType,
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32,
    floor: i32,
}

impl WheelchairAdapter {
    /// Create a new wheelchair adapter
    pub fn new() -> Self {
        let mut adapter = Self {
            current_position: None,
            movement_state: MovementState::Stationary,
            learning_mode: LearningMode::FullInteractive,
            known_locations: Vec::new(),
            audio_settings: AudioSettings::default(),
            connected: true,
        };
        adapter.load_default_locations();
        adapter
    }

    /// Load default location database
    fn load_default_locations(&mut self) {
        self.known_locations = vec![
            KnownLocation {
                name: "Main Classroom".to_string(),
                location_type: LocationType::Classroom,
                x_min: 0.0,
                x_max: 10.0,
                y_min: 0.0,
                y_max: 8.0,
                floor: 1,
            },
            KnownLocation {
                name: "Library".to_string(),
                location_type: LocationType::Library,
                x_min: 15.0,
                x_max: 30.0,
                y_min: 0.0,
                y_max: 20.0,
                floor: 1,
            },
            KnownLocation {
                name: "Study Room A".to_string(),
                location_type: LocationType::StudyRoom,
                x_min: 12.0,
                x_max: 15.0,
                y_min: 5.0,
                y_max: 8.0,
                floor: 1,
            },
        ];
    }

    /// Find location type for position
    fn find_location_type(&self, position: &Position) -> (LocationType, Option<String>) {
        for loc in &self.known_locations {
            if position.floor == loc.floor
                && position.x >= loc.x_min
                && position.x <= loc.x_max
                && position.y >= loc.y_min
                && position.y <= loc.y_max
            {
                return (loc.location_type, Some(loc.name.clone()));
            }
        }
        (LocationType::Unknown, None)
    }

    /// Set connection status
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }

    /// Set movement state
    pub fn set_movement_state(&mut self, state: MovementState) {
        self.movement_state = state;
        // Auto-adjust learning mode based on movement
        self.learning_mode = match state {
            MovementState::Stationary | MovementState::AtDestination => LearningMode::FullInteractive,
            MovementState::MovingSlow => LearningMode::AudioOnly,
            MovementState::MovingNormal | MovementState::Turning => LearningMode::Passive,
            MovementState::MovingFast => LearningMode::Paused,
        };
    }

    /// Update audio settings
    pub fn update_audio_settings(&mut self, settings: AudioSettings) {
        self.audio_settings = settings;
    }
}

impl Default for WheelchairAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl WheelchairEducation for WheelchairAdapter {
    fn get_location_context(&self) -> Option<LocationContext> {
        let position = self.current_position?;
        let (location_type, location_name) = self.find_location_type(&position);

        // Estimate environmental factors based on location
        let (noise, lighting) = match location_type {
            LocationType::Library => (0.2, 0.7),
            LocationType::Classroom => (0.4, 0.8),
            LocationType::StudyRoom => (0.3, 0.8),
            LocationType::Hallway => (0.6, 0.6),
            LocationType::Cafeteria => (0.8, 0.7),
            LocationType::Outdoors => (0.5, 0.9),
            LocationType::Home => (0.3, 0.7),
            LocationType::Laboratory => (0.4, 0.9),
            LocationType::Unknown => (0.5, 0.5),
        };

        Some(LocationContext {
            position,
            location_type,
            location_name,
            noise_level: noise,
            lighting_level: lighting,
            movement_state: self.movement_state,
            nearby_resources: vec![], // Would be populated from beacon/AR system
        })
    }

    fn set_learning_mode(&mut self, mode: LearningMode) -> Result<()> {
        // Safety check: don't allow full interactive while moving fast
        if mode == LearningMode::FullInteractive && self.movement_state == MovementState::MovingFast {
            return Err(EduError::ValidationError(
                "Cannot enable full interactive mode while moving fast".to_string()
            ));
        }
        self.learning_mode = mode;
        Ok(())
    }

    fn recommended_content_format(&self) -> ContentFormat {
        match self.learning_mode {
            LearningMode::FullInteractive => ContentFormat::FullMultimedia,
            LearningMode::AudioOnly => ContentFormat::AudioOnly,
            LearningMode::Passive => ContentFormat::AudioOnly,
            LearningMode::Review => ContentFormat::TextWithTTS,
            LearningMode::Paused => ContentFormat::None,
        }
    }

    fn is_safe_for_visual(&self) -> bool {
        matches!(
            self.movement_state,
            MovementState::Stationary | MovementState::AtDestination
        )
    }

    fn audio_learning_settings(&self) -> AudioSettings {
        let mut settings = self.audio_settings.clone();

        // Adjust based on movement state
        match self.movement_state {
            MovementState::MovingNormal | MovementState::MovingFast => {
                settings.bone_conduction = true; // Keep ears open for safety
                settings.background_ducking = true;
            }
            MovementState::Stationary | MovementState::AtDestination => {
                settings.bone_conduction = false;
            }
            _ => {}
        }

        settings
    }

    fn on_position_change(&mut self, position: Position) {
        self.current_position = Some(position);
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_wheelchair_adapter() {
        let adapter = WheelchairAdapter::new();
        assert!(adapter.is_connected());
        assert!(adapter.get_location_context().is_none());
    }

    #[test]
    fn test_position_change() {
        let mut adapter = WheelchairAdapter::new();

        adapter.on_position_change(Position {
            x: 5.0,
            y: 4.0,
            floor: 1,
            heading: 90.0,
        });

        let context = adapter.get_location_context();
        assert!(context.is_some());
        assert_eq!(context.unwrap().location_type, LocationType::Classroom);
    }

    #[test]
    fn test_movement_state_affects_mode() {
        let mut adapter = WheelchairAdapter::new();

        adapter.set_movement_state(MovementState::MovingFast);
        assert!(!adapter.is_safe_for_visual());
        assert_eq!(adapter.recommended_content_format(), ContentFormat::None);

        adapter.set_movement_state(MovementState::Stationary);
        assert!(adapter.is_safe_for_visual());
        assert_eq!(adapter.recommended_content_format(), ContentFormat::FullMultimedia);
    }

    #[test]
    fn test_learning_mode_safety() {
        let mut adapter = WheelchairAdapter::new();
        adapter.set_movement_state(MovementState::MovingFast);

        let result = adapter.set_learning_mode(LearningMode::FullInteractive);
        assert!(result.is_err());
    }

    #[test]
    fn test_audio_settings() {
        let mut adapter = WheelchairAdapter::new();

        adapter.set_movement_state(MovementState::MovingNormal);
        let settings = adapter.audio_learning_settings();
        assert!(settings.bone_conduction);

        adapter.set_movement_state(MovementState::Stationary);
        let settings = adapter.audio_learning_settings();
        assert!(!settings.bone_conduction);
    }

    #[test]
    fn test_location_detection() {
        let mut adapter = WheelchairAdapter::new();

        // Library location
        adapter.on_position_change(Position {
            x: 20.0,
            y: 10.0,
            floor: 1,
            heading: 0.0,
        });

        let context = adapter.get_location_context().unwrap();
        assert_eq!(context.location_type, LocationType::Library);
        assert!(context.noise_level < 0.5); // Library is quiet
    }
}
