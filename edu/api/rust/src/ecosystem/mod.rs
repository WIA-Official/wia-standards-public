//! WIA Ecosystem Integration
//! 弘益人間 - Integrate education with WIA assistive technologies

pub mod aac;
pub mod bci;
pub mod eye_gaze;
pub mod wheelchair;
pub mod haptic;
pub mod external;

// Re-exports
pub use aac::{AACEducation, AACAdapter, AACSymbol, AACMessage};
pub use bci::{BCIEducation, BCIAdapter, BCIIntent, BCISelection, NavigationIntent};
pub use eye_gaze::{EyeGazeEducation, EyeGazeAdapter, GazePoint, ReadingAnalysis};
pub use wheelchair::{WheelchairEducation, WheelchairAdapter, LocationContext};
pub use haptic::{HapticEducation, HapticAdapter, HapticPattern, BrailleOutput};
pub use external::{
    LMSAdapter, LMSPlatform, LMSConfig,
    CanvasAdapter, MoodleAdapter, BlackboardAdapter, GoogleClassroomAdapter,
};

use crate::types::LearnerProfile;
use crate::error::Result;

/// Ecosystem integration manager
/// Coordinates between WIA assistive devices and educational platforms
pub struct EcosystemManager {
    /// AAC adapter
    aac: Option<Box<dyn AACEducation>>,
    /// BCI adapter
    bci: Option<Box<dyn BCIEducation>>,
    /// Eye gaze adapter
    eye_gaze: Option<Box<dyn EyeGazeEducation>>,
    /// Wheelchair adapter
    wheelchair: Option<Box<dyn WheelchairEducation>>,
    /// Haptic adapter
    haptic: Option<Box<dyn HapticEducation>>,
    /// LMS adapters
    lms_adapters: Vec<Box<dyn LMSAdapter>>,
    /// Current learner profile
    profile: Option<LearnerProfile>,
}

impl EcosystemManager {
    /// Create a new ecosystem manager
    pub fn new() -> Self {
        Self {
            aac: None,
            bci: None,
            eye_gaze: None,
            wheelchair: None,
            haptic: None,
            lms_adapters: Vec::new(),
            profile: None,
        }
    }

    /// Set the current learner profile
    pub fn set_profile(&mut self, profile: LearnerProfile) {
        self.profile = Some(profile);
    }

    /// Get the current learner profile
    pub fn profile(&self) -> Option<&LearnerProfile> {
        self.profile.as_ref()
    }

    /// Register AAC adapter
    pub fn register_aac(&mut self, adapter: Box<dyn AACEducation>) {
        self.aac = Some(adapter);
    }

    /// Register BCI adapter
    pub fn register_bci(&mut self, adapter: Box<dyn BCIEducation>) {
        self.bci = Some(adapter);
    }

    /// Register Eye Gaze adapter
    pub fn register_eye_gaze(&mut self, adapter: Box<dyn EyeGazeEducation>) {
        self.eye_gaze = Some(adapter);
    }

    /// Register Wheelchair adapter
    pub fn register_wheelchair(&mut self, adapter: Box<dyn WheelchairEducation>) {
        self.wheelchair = Some(adapter);
    }

    /// Register Haptic adapter
    pub fn register_haptic(&mut self, adapter: Box<dyn HapticEducation>) {
        self.haptic = Some(adapter);
    }

    /// Register LMS adapter
    pub fn register_lms(&mut self, adapter: Box<dyn LMSAdapter>) {
        self.lms_adapters.push(adapter);
    }

    /// Get AAC adapter
    pub fn aac(&self) -> Option<&dyn AACEducation> {
        self.aac.as_deref()
    }

    /// Get BCI adapter
    pub fn bci(&self) -> Option<&dyn BCIEducation> {
        self.bci.as_deref()
    }

    /// Get Eye Gaze adapter
    pub fn eye_gaze(&self) -> Option<&dyn EyeGazeEducation> {
        self.eye_gaze.as_deref()
    }

    /// Get Wheelchair adapter
    pub fn wheelchair(&self) -> Option<&dyn WheelchairEducation> {
        self.wheelchair.as_deref()
    }

    /// Get Haptic adapter
    pub fn haptic(&self) -> Option<&dyn HapticEducation> {
        self.haptic.as_deref()
    }

    /// Get LMS adapters
    pub fn lms_adapters(&self) -> &[Box<dyn LMSAdapter>] {
        &self.lms_adapters
    }

    /// Sync profile to all registered LMS platforms
    pub async fn sync_profile_to_all_lms(&self) -> Result<Vec<SyncResult>> {
        let mut results = Vec::new();

        if let Some(profile) = &self.profile {
            for adapter in &self.lms_adapters {
                let result = adapter.sync_profile(profile).await;
                results.push(SyncResult {
                    platform: adapter.platform(),
                    success: result.is_ok(),
                    error: result.err().map(|e| e.to_string()),
                });
            }
        }

        Ok(results)
    }

    /// Check which assistive devices are available
    pub fn available_devices(&self) -> AvailableDevices {
        AvailableDevices {
            aac: self.aac.is_some(),
            bci: self.bci.is_some(),
            eye_gaze: self.eye_gaze.is_some(),
            wheelchair: self.wheelchair.is_some(),
            haptic: self.haptic.is_some(),
        }
    }
}

impl Default for EcosystemManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of syncing to an LMS platform
#[derive(Debug, Clone)]
pub struct SyncResult {
    /// Platform name
    pub platform: LMSPlatform,
    /// Whether sync was successful
    pub success: bool,
    /// Error message if failed
    pub error: Option<String>,
}

/// Available assistive devices
#[derive(Debug, Clone, Default)]
pub struct AvailableDevices {
    /// AAC device available
    pub aac: bool,
    /// BCI device available
    pub bci: bool,
    /// Eye gaze device available
    pub eye_gaze: bool,
    /// Wheelchair device available
    pub wheelchair: bool,
    /// Haptic device available
    pub haptic: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_ecosystem_manager() {
        let manager = EcosystemManager::new();
        assert!(manager.profile().is_none());
        assert!(manager.aac().is_none());
        assert!(manager.bci().is_none());
    }

    #[test]
    fn test_available_devices() {
        let manager = EcosystemManager::new();
        let devices = manager.available_devices();
        assert!(!devices.aac);
        assert!(!devices.bci);
        assert!(!devices.eye_gaze);
    }
}
