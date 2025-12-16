//! WIA Game Ecosystem Integration
//!
//! Phase 4 integration with WIA assistive technology ecosystem and external gaming platforms.

pub mod aac;
pub mod analytics;
pub mod bci;
pub mod cloud;
pub mod eye_gaze;
pub mod haptic;
pub mod input_processor;
pub mod platforms;
pub mod smart_home;
pub mod wheelchair;

pub use aac::*;
pub use analytics::*;
pub use bci::*;
pub use cloud::*;
pub use eye_gaze::*;
pub use haptic::*;
pub use input_processor::*;
pub use smart_home::*;
pub use wheelchair::*;

use crate::error::GameError;
use crate::types::PlayerProfile;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

/// Central hub for all ecosystem integrations
pub struct EcosystemHub {
    /// Eye gaze adapter for aiming and camera control
    pub eye_gaze: Option<EyeGazeAdapter>,
    /// Brain-computer interface adapter
    pub bci: Option<BCIAdapter>,
    /// Augmentative and alternative communication adapter
    pub aac: Option<AACAdapter>,
    /// Smart wheelchair integration
    pub wheelchair: Option<WheelchairAdapter>,
    /// Haptic feedback system
    pub haptic: Option<HapticAdapter>,
    /// Smart home environmental effects
    pub smart_home: Option<SmartHomeAdapter>,

    /// External platform integrations
    pub platforms: PlatformIntegrations,

    /// Cloud sync manager
    pub cloud_sync: CloudSyncManager,

    /// Unified input processor
    pub input_processor: UnifiedInputProcessor,

    /// Analytics collector
    pub analytics: AnalyticsCollector,

    /// Active player profile
    active_profile: Arc<RwLock<Option<PlayerProfile>>>,
}

/// External gaming platform integrations
pub struct PlatformIntegrations {
    pub xbox: Option<platforms::XboxIntegration>,
    pub playstation: Option<platforms::PlayStationIntegration>,
    pub steam: Option<platforms::SteamIntegration>,
    pub cloud_gaming: Option<platforms::CloudGamingSettings>,
}

impl EcosystemHub {
    /// Create a new ecosystem hub with default settings
    pub fn new() -> Self {
        Self {
            eye_gaze: None,
            bci: None,
            aac: None,
            wheelchair: None,
            haptic: None,
            smart_home: None,
            platforms: PlatformIntegrations::new(),
            cloud_sync: CloudSyncManager::new("https://api.wia.live/game".to_string()),
            input_processor: UnifiedInputProcessor::new(),
            analytics: AnalyticsCollector::new(),
            active_profile: Arc::new(RwLock::new(None)),
        }
    }

    /// Create hub with all WIA adapters enabled (simulated)
    pub fn with_all_adapters() -> Self {
        Self {
            eye_gaze: Some(EyeGazeAdapter::new()),
            bci: Some(BCIAdapter::new()),
            aac: Some(AACAdapter::new()),
            wheelchair: Some(WheelchairAdapter::new()),
            haptic: Some(HapticAdapter::new()),
            smart_home: Some(SmartHomeAdapter::new()),
            platforms: PlatformIntegrations::with_all(),
            cloud_sync: CloudSyncManager::new("https://api.wia.live/game".to_string()),
            input_processor: UnifiedInputProcessor::new(),
            analytics: AnalyticsCollector::new(),
            active_profile: Arc::new(RwLock::new(None)),
        }
    }

    /// Set the active player profile
    pub async fn set_profile(&self, profile: PlayerProfile) {
        let mut active = self.active_profile.write().await;
        *active = Some(profile);
    }

    /// Get the active player profile
    pub async fn get_profile(&self) -> Option<PlayerProfile> {
        self.active_profile.read().await.clone()
    }

    /// Enable eye gaze adapter
    pub fn enable_eye_gaze(&mut self, config: EyeGazeConfig) {
        let mut adapter = EyeGazeAdapter::new();
        adapter.configure(config);
        self.eye_gaze = Some(adapter);
    }

    /// Enable BCI adapter
    pub fn enable_bci(&mut self, config: BCIConfig) {
        let mut adapter = BCIAdapter::new();
        adapter.configure(config);
        self.bci = Some(adapter);
    }

    /// Enable AAC adapter
    pub fn enable_aac(&mut self, config: AACConfig) {
        let mut adapter = AACAdapter::new();
        adapter.configure(config);
        self.aac = Some(adapter);
    }

    /// Enable wheelchair adapter
    pub fn enable_wheelchair(&mut self, config: WheelchairConfig) {
        let mut adapter = WheelchairAdapter::new();
        adapter.configure(config);
        self.wheelchair = Some(adapter);
    }

    /// Enable haptic adapter
    pub fn enable_haptic(&mut self, config: HapticConfig) {
        let mut adapter = HapticAdapter::new();
        adapter.configure(config);
        self.haptic = Some(adapter);
    }

    /// Enable smart home adapter
    pub fn enable_smart_home(&mut self, config: SmartHomeConfig) {
        let mut adapter = SmartHomeAdapter::new();
        adapter.configure(config);
        self.smart_home = Some(adapter);
    }

    /// Process input from all enabled sources
    pub async fn process_inputs(&mut self) -> Vec<GameAction> {
        let mut actions = Vec::new();

        // Collect inputs from all sources
        if let Some(ref mut eye_gaze) = self.eye_gaze {
            if let Some(input) = eye_gaze.poll() {
                self.input_processor.add_input(InputEvent::EyeGaze(input));
            }
        }

        if let Some(ref mut bci) = self.bci {
            if let Some(input) = bci.poll() {
                self.input_processor.add_input(InputEvent::BCI(input));
            }
        }

        if let Some(ref mut aac) = self.aac {
            if let Some(input) = aac.poll() {
                self.input_processor.add_input(InputEvent::AAC(input));
            }
        }

        if let Some(ref mut wheelchair) = self.wheelchair {
            if let Some(input) = wheelchair.poll() {
                self.input_processor.add_input(InputEvent::Wheelchair(input));
            }
        }

        // Process fused inputs
        actions.extend(self.input_processor.process());

        // Record analytics
        for action in &actions {
            self.analytics.record_action(action.clone());
        }

        actions
    }

    /// Trigger haptic feedback for a game event
    pub fn trigger_haptic(&mut self, event: GameEvent) {
        if let Some(ref mut haptic) = self.haptic {
            haptic.trigger(event);
        }
    }

    /// Sync environment with game state
    pub fn sync_environment(&mut self, state: &GameState) {
        if let Some(ref mut smart_home) = self.smart_home {
            smart_home.sync_with_game(state);
        }
    }

    /// Sync profile to external platform
    pub async fn sync_to_platform(
        &self,
        platform: Platform,
    ) -> Result<(), GameError> {
        let profile = self.get_profile().await
            .ok_or_else(|| GameError::InvalidConfiguration("No active profile".to_string()))?;

        match platform {
            Platform::Xbox => {
                if let Some(ref xbox) = self.platforms.xbox {
                    xbox.sync_profile(&profile).await
                } else {
                    Err(GameError::InvalidConfiguration("Xbox not enabled".to_string()))
                }
            }
            Platform::PlayStation => {
                if let Some(ref ps) = self.platforms.playstation {
                    ps.sync_profile(&profile).await
                } else {
                    Err(GameError::InvalidConfiguration("PlayStation not enabled".to_string()))
                }
            }
            Platform::Steam => {
                if let Some(ref steam) = self.platforms.steam {
                    steam.sync_profile(&profile).await
                } else {
                    Err(GameError::InvalidConfiguration("Steam not enabled".to_string()))
                }
            }
        }
    }

    /// Get analytics summary
    pub fn get_analytics(&self) -> AnalyticsSummary {
        self.analytics.get_summary()
    }
}

impl Default for EcosystemHub {
    fn default() -> Self {
        Self::new()
    }
}

impl PlatformIntegrations {
    pub fn new() -> Self {
        Self {
            xbox: None,
            playstation: None,
            steam: None,
            cloud_gaming: None,
        }
    }

    pub fn with_all() -> Self {
        Self {
            xbox: Some(platforms::XboxIntegration::new()),
            playstation: Some(platforms::PlayStationIntegration::new()),
            steam: Some(platforms::SteamIntegration::new()),
            cloud_gaming: Some(platforms::CloudGamingSettings::default()),
        }
    }
}

impl Default for PlatformIntegrations {
    fn default() -> Self {
        Self::new()
    }
}

/// Target gaming platform
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Platform {
    Xbox,
    PlayStation,
    Steam,
}

/// Game action produced by input processing
#[derive(Debug, Clone, PartialEq)]
pub enum GameAction {
    /// Movement in a direction
    Move { direction: (f32, f32), speed: f32 },
    /// Camera rotation
    Camera { delta: (f32, f32) },
    /// Aim at a target
    Aim { target: AimTarget, magnetism: f32 },
    /// Primary attack/fire
    Attack,
    /// Secondary attack
    SecondaryAttack,
    /// Use/interact
    Interact,
    /// Jump
    Jump,
    /// Crouch/duck
    Crouch,
    /// Sprint toggle
    Sprint,
    /// Reload weapon
    Reload,
    /// Switch weapon
    SwitchWeapon,
    /// Use ability
    UseAbility { slot: u8 },
    /// Use item
    UseItem { slot: u8 },
    /// Open menu
    OpenMenu { menu: MenuType },
    /// Confirm selection
    Confirm,
    /// Cancel/back
    Cancel,
    /// Pause game
    Pause,
    /// Quick save
    QuickSave,
    /// Lock onto target
    LockTarget { entity_id: Option<Uuid> },
    /// Execute macro
    ExecuteMacro { macro_id: Uuid },
    /// Custom action
    Custom { name: String, value: f32 },
}

/// Aim target types
#[derive(Debug, Clone, PartialEq)]
pub enum AimTarget {
    Point { x: f32, y: f32 },
    Entity { id: Uuid },
    Direction { angle: f32 },
}

/// Menu types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuType {
    Pause,
    Inventory,
    Map,
    Settings,
    Abilities,
    Quests,
}

/// Current game state for environment sync
#[derive(Debug, Clone)]
pub struct GameState {
    pub time_of_day: TimeOfDay,
    pub weather: Weather,
    pub combat_active: bool,
    pub stealth_mode: bool,
    pub player_health_percent: f32,
    pub is_paused: bool,
    pub current_location: Option<String>,
}

/// Time of day in game
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeOfDay {
    Dawn,
    Day,
    Dusk,
    Night,
}

/// Weather conditions in game
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Weather {
    Clear,
    Cloudy,
    Rain,
    Storm,
    Snow,
    Fog,
    Fire,
    Sandstorm,
}

/// Game events for haptic feedback
#[derive(Debug, Clone)]
pub enum GameEvent {
    DamageTaken { amount: f32, direction: Option<f32> },
    HealthLow { percent: f32 },
    EnemyNearby { distance: f32, direction: f32 },
    WeaponFire { weapon_type: String },
    Explosion { distance: f32, direction: Option<f32> },
    Footsteps { direction: f32, distance: f32 },
    UISelection,
    Achievement { name: String },
    Victory,
    Defeat,
    Collision { intensity: f32 },
    EnvironmentEffect { effect_type: String },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ecosystem_hub_creation() {
        let hub = EcosystemHub::new();
        assert!(hub.eye_gaze.is_none());
        assert!(hub.bci.is_none());
    }

    #[test]
    fn test_ecosystem_hub_with_adapters() {
        let hub = EcosystemHub::with_all_adapters();
        assert!(hub.eye_gaze.is_some());
        assert!(hub.bci.is_some());
        assert!(hub.aac.is_some());
        assert!(hub.wheelchair.is_some());
        assert!(hub.haptic.is_some());
        assert!(hub.smart_home.is_some());
    }

    #[tokio::test]
    async fn test_set_profile() {
        let hub = EcosystemHub::new();
        let profile = PlayerProfile::default();
        hub.set_profile(profile.clone()).await;

        let retrieved = hub.get_profile().await;
        assert!(retrieved.is_some());
    }

    #[test]
    fn test_platform_integrations() {
        let platforms = PlatformIntegrations::with_all();
        assert!(platforms.xbox.is_some());
        assert!(platforms.playstation.is_some());
        assert!(platforms.steam.is_some());
    }
}
