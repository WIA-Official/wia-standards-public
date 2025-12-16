//! Macro System
//! 弘益人間 - Gaming for Everyone
//!
//! Macro recording and playback for accessibility.

use super::event::{Axis, Button, InputEvent, Point2D};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

/// Macro identifier
pub type MacroId = Uuid;

/// Macro definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Macro {
    pub id: MacroId,
    pub name: String,
    pub description: Option<String>,
    pub trigger: MacroTrigger,
    pub actions: Vec<MacroAction>,
    pub repeat_mode: RepeatMode,
    pub enabled: bool,
}

impl Default for Macro {
    fn default() -> Self {
        Self {
            id: Uuid::new_v4(),
            name: "New Macro".to_string(),
            description: None,
            trigger: MacroTrigger::Button { button: Button::Extra1 },
            actions: Vec::new(),
            repeat_mode: RepeatMode::Once,
            enabled: true,
        }
    }
}

/// Macro trigger types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MacroTrigger {
    /// Single button press
    Button { button: Button },
    /// Button combination (chord)
    ButtonCombo { buttons: Vec<Button>, simultaneous: bool },
    /// Voice command
    VoiceCommand { phrase: String },
    /// Gaze at region
    GazeRegion { x: f32, y: f32, width: f32, height: f32, dwell_ms: u32 },
    /// Timer/interval
    Timer { interval_ms: u32 },
    /// On game event (custom)
    GameEvent { event_name: String },
}

/// Macro action types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MacroAction {
    /// Press a button (down and up)
    PressButton { button: Button },
    /// Hold button down
    HoldButton { button: Button },
    /// Release button
    ReleaseButton { button: Button },
    /// Hold button for duration
    HoldButtonTimed { button: Button, duration_ms: u32 },
    /// Move axis to value
    MoveAxis { axis: Axis, value: f32 },
    /// Move axis over time (smooth)
    MoveAxisSmooth { axis: Axis, start: f32, end: f32, duration_ms: u32 },
    /// Wait/delay
    Delay { ms: u32 },
    /// Wait for button release
    WaitForRelease { button: Button },
    /// Play sound
    PlaySound { sound_id: String },
    /// Show notification
    ShowNotification { message: String },
    /// Run another macro
    RunMacro { macro_id: MacroId },
    /// Conditional action
    Conditional { condition: MacroCondition, then_actions: Vec<MacroAction>, else_actions: Vec<MacroAction> },
}

/// Macro condition for conditional actions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MacroCondition {
    ButtonHeld { button: Button },
    AxisInRange { axis: Axis, min: f32, max: f32 },
    GazeInRegion { x: f32, y: f32, width: f32, height: f32 },
    Random { probability: f32 },
}

/// Repeat mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RepeatMode {
    /// Execute once
    Once,
    /// Repeat while trigger held
    WhileHeld,
    /// Toggle on/off
    Toggle,
    /// Repeat N times
    Times(u32),
    /// Repeat infinitely until cancelled
    Infinite,
}

/// Macro engine for managing and executing macros
#[derive(Debug)]
pub struct MacroEngine {
    macros: HashMap<MacroId, Macro>,
    active_macros: HashMap<MacroId, MacroState>,
    button_states: HashMap<Button, bool>,
}

/// State of a running macro
#[derive(Debug, Clone)]
struct MacroState {
    current_action: usize,
    started_at: u64,
    repeat_count: u32,
    is_toggled_on: bool,
}

impl MacroEngine {
    /// Create a new macro engine
    pub fn new() -> Self {
        Self {
            macros: HashMap::new(),
            active_macros: HashMap::new(),
            button_states: HashMap::new(),
        }
    }

    /// Add a macro
    pub fn add_macro(&mut self, macro_def: Macro) {
        self.macros.insert(macro_def.id, macro_def);
    }

    /// Remove a macro
    pub fn remove_macro(&mut self, id: MacroId) -> Option<Macro> {
        self.active_macros.remove(&id);
        self.macros.remove(&id)
    }

    /// Get a macro
    pub fn get_macro(&self, id: MacroId) -> Option<&Macro> {
        self.macros.get(&id)
    }

    /// List all macros
    pub fn list_macros(&self) -> Vec<&Macro> {
        self.macros.values().collect()
    }

    /// Enable a macro
    pub fn enable_macro(&mut self, id: MacroId) {
        if let Some(m) = self.macros.get_mut(&id) {
            m.enabled = true;
        }
    }

    /// Disable a macro
    pub fn disable_macro(&mut self, id: MacroId) {
        if let Some(m) = self.macros.get_mut(&id) {
            m.enabled = false;
        }
        self.active_macros.remove(&id);
    }

    /// Check if an event triggers any macro
    pub fn check_trigger(&self, event: &InputEvent) -> Option<InputEvent> {
        for macro_def in self.macros.values() {
            if !macro_def.enabled {
                continue;
            }

            if self.matches_trigger(&macro_def.trigger, event) {
                // In a real implementation, this would start executing the macro
                // For now, we just return None
                return None;
            }
        }
        None
    }

    /// Check if event matches trigger
    fn matches_trigger(&self, trigger: &MacroTrigger, event: &InputEvent) -> bool {
        match trigger {
            MacroTrigger::Button { button } => {
                if let InputEvent::ButtonPressed { button: pressed, .. } = event {
                    pressed == button
                } else {
                    false
                }
            }
            MacroTrigger::ButtonCombo { buttons, simultaneous } => {
                if *simultaneous {
                    // Check if all buttons are currently held
                    buttons.iter().all(|b| *self.button_states.get(b).unwrap_or(&false))
                } else {
                    // Sequential combo - would need more state tracking
                    false
                }
            }
            MacroTrigger::VoiceCommand { phrase } => {
                if let InputEvent::VoiceCommand { command, .. } = event {
                    command.to_lowercase().contains(&phrase.to_lowercase())
                } else {
                    false
                }
            }
            MacroTrigger::GazeRegion { x, y, width, height, .. } => {
                if let InputEvent::GazePoint { x: gx, y: gy, .. } = event {
                    *gx >= *x && *gx <= *x + *width && *gy >= *y && *gy <= *y + *height
                } else {
                    false
                }
            }
            MacroTrigger::Timer { .. } | MacroTrigger::GameEvent { .. } => false,
        }
    }

    /// Update button state
    pub fn update_button_state(&mut self, button: Button, pressed: bool) {
        self.button_states.insert(button, pressed);
    }

    /// Create a rapid fire macro
    pub fn create_rapid_fire(button: Button, rate_ms: u32) -> Macro {
        Macro {
            id: Uuid::new_v4(),
            name: format!("Rapid Fire {:?}", button),
            description: Some(format!("Rapid fire {} every {}ms", format!("{:?}", button), rate_ms)),
            trigger: MacroTrigger::Button { button },
            actions: vec![
                MacroAction::PressButton { button },
                MacroAction::Delay { ms: rate_ms / 2 },
            ],
            repeat_mode: RepeatMode::WhileHeld,
            enabled: true,
        }
    }

    /// Create a turbo toggle macro
    pub fn create_turbo_toggle(button: Button, rate_ms: u32) -> Macro {
        Macro {
            id: Uuid::new_v4(),
            name: format!("Turbo {:?}", button),
            description: Some(format!("Toggle turbo for {:?}", button)),
            trigger: MacroTrigger::Button { button: Button::Extra1 },
            actions: vec![
                MacroAction::PressButton { button },
                MacroAction::Delay { ms: rate_ms },
            ],
            repeat_mode: RepeatMode::Toggle,
            enabled: true,
        }
    }

    /// Create a combo sequence macro
    pub fn create_combo(name: &str, trigger: Button, combo: Vec<(Button, u32)>) -> Macro {
        let mut actions = Vec::new();

        for (button, delay) in combo {
            actions.push(MacroAction::PressButton { button });
            if delay > 0 {
                actions.push(MacroAction::Delay { ms: delay });
            }
        }

        Macro {
            id: Uuid::new_v4(),
            name: name.to_string(),
            description: Some(format!("Combo sequence: {}", name)),
            trigger: MacroTrigger::Button { button: trigger },
            actions,
            repeat_mode: RepeatMode::Once,
            enabled: true,
        }
    }
}

impl Default for MacroEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_macro() {
        let macro_def = Macro {
            id: Uuid::new_v4(),
            name: "Test Macro".to_string(),
            description: Some("A test macro".to_string()),
            trigger: MacroTrigger::Button { button: Button::A },
            actions: vec![
                MacroAction::PressButton { button: Button::X },
                MacroAction::Delay { ms: 100 },
                MacroAction::PressButton { button: Button::Y },
            ],
            repeat_mode: RepeatMode::Once,
            enabled: true,
        };

        assert_eq!(macro_def.name, "Test Macro");
        assert_eq!(macro_def.actions.len(), 3);
    }

    #[test]
    fn test_macro_engine() {
        let mut engine = MacroEngine::new();

        let macro_def = Macro::default();
        let id = macro_def.id;

        engine.add_macro(macro_def);
        assert!(engine.get_macro(id).is_some());

        engine.disable_macro(id);
        assert!(!engine.get_macro(id).unwrap().enabled);

        engine.remove_macro(id);
        assert!(engine.get_macro(id).is_none());
    }

    #[test]
    fn test_rapid_fire_macro() {
        let rapid_fire = MacroEngine::create_rapid_fire(Button::A, 50);
        assert_eq!(rapid_fire.repeat_mode, RepeatMode::WhileHeld);
        assert_eq!(rapid_fire.actions.len(), 2);
    }

    #[test]
    fn test_combo_macro() {
        let combo = MacroEngine::create_combo(
            "Hadouken",
            Button::Extra1,
            vec![
                (Button::DPadDown, 50),
                (Button::DPadRight, 50),
                (Button::X, 0),
            ],
        );

        assert_eq!(combo.name, "Hadouken");
        assert!(combo.actions.len() >= 3);
    }
}
