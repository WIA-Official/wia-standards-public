//! Gestural representation (제스처)

use serde::{Deserialize, Serialize};

/// Gestural representation
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GesturalRep {
    /// Supported gestures
    pub gestures: Vec<Gesture>,

    /// Voice commands
    pub voice_commands: Vec<VoiceCommand>,

    /// Interaction hints
    pub interaction_hints: Vec<InteractionHint>,
}

/// Gesture types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Gesture {
    Tap,
    DoubleTap,
    LongPress,
    Swipe(SwipeDirection),
    Pinch,
    Rotate,
}

/// Swipe directions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SwipeDirection {
    Up,
    Down,
    Left,
    Right,
}

/// Voice command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCommand {
    pub phrase: String,
    pub action: String,
}

impl VoiceCommand {
    pub fn new(phrase: impl Into<String>) -> Self {
        Self {
            phrase: phrase.into(),
            action: String::new(),
        }
    }

    pub fn with_action(mut self, action: impl Into<String>) -> Self {
        self.action = action.into();
        self
    }
}

/// Interaction hint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InteractionHint {
    pub hint_type: InteractionHintType,
    pub description: String,
}

/// Interaction hint types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InteractionHintType {
    Tooltip,
    Announcement,
    Guidance,
}

impl GesturalRep {
    /// Create a gestural representation with a gesture
    pub fn with_gesture(gesture: Gesture) -> Self {
        Self {
            gestures: vec![gesture],
            ..Default::default()
        }
    }

    /// Add a gesture
    pub fn add_gesture(mut self, gesture: Gesture) -> Self {
        self.gestures.push(gesture);
        self
    }

    /// Add a voice command
    pub fn add_voice_command(mut self, command: VoiceCommand) -> Self {
        self.voice_commands.push(command);
        self
    }

    /// Add an interaction hint
    pub fn add_interaction_hint(mut self, hint: InteractionHint) -> Self {
        self.interaction_hints.push(hint);
        self
    }
}
