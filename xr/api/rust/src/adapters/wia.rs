//! WIA Integration Adapters
//!
//! Adapters for integrating with other WIA accessibility systems.

use async_trait::async_trait;
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock};

use crate::error::{WIAIntegrationError, WIAResult, Result, XRAccessibilityError};
use crate::types::*;

/// WIA integration hub for coordinating multiple WIA systems
pub struct WIAIntegrationHub {
    exoskeleton: Option<Arc<dyn ExoskeletonAdapter>>,
    bionic_eye: Option<Arc<dyn BionicEyeAdapter>>,
    voice_sign: Option<Arc<dyn VoiceSignAdapter>>,
    event_sender: Option<mpsc::Sender<WIAEvent>>,
    sync_state: Arc<RwLock<WIASyncState>>,
}

/// WIA system synchronization state
#[derive(Debug, Clone, Default)]
pub struct WIASyncState {
    pub exoskeleton_connected: bool,
    pub bionic_eye_connected: bool,
    pub voice_sign_connected: bool,
    pub last_sync: Option<chrono::DateTime<chrono::Utc>>,
}

/// Events from WIA integrations
#[derive(Debug, Clone)]
pub enum WIAEvent {
    ExoskeletonConnected { device_id: String },
    ExoskeletonDisconnected { device_id: String },
    BionicEyeConnected { device_id: String },
    BionicEyeDisconnected { device_id: String },
    VoiceSignConnected,
    VoiceSignDisconnected,
    SyncCompleted,
    SyncFailed { reason: String },
}

impl WIAIntegrationHub {
    /// Create a new integration hub
    pub fn new() -> Self {
        Self {
            exoskeleton: None,
            bionic_eye: None,
            voice_sign: None,
            event_sender: None,
            sync_state: Arc::new(RwLock::new(WIASyncState::default())),
        }
    }

    /// Create with event channel
    pub fn with_events() -> (Self, mpsc::Receiver<WIAEvent>) {
        let (tx, rx) = mpsc::channel(100);
        let mut hub = Self::new();
        hub.event_sender = Some(tx);
        (hub, rx)
    }

    /// Register exoskeleton integration
    pub async fn register_exoskeleton(&mut self, adapter: Arc<dyn ExoskeletonAdapter>) {
        self.exoskeleton = Some(adapter);
        self.sync_state.write().await.exoskeleton_connected = true;
    }

    /// Register bionic eye integration
    pub async fn register_bionic_eye(&mut self, adapter: Arc<dyn BionicEyeAdapter>) {
        self.bionic_eye = Some(adapter);
        self.sync_state.write().await.bionic_eye_connected = true;
    }

    /// Register voice-sign integration
    pub async fn register_voice_sign(&mut self, adapter: Arc<dyn VoiceSignAdapter>) {
        self.voice_sign = Some(adapter);
        self.sync_state.write().await.voice_sign_connected = true;
    }

    /// Get sync state
    pub async fn get_sync_state(&self) -> WIASyncState {
        self.sync_state.read().await.clone()
    }

    /// Sync profile across all connected systems
    pub async fn sync_profile(&self, profile: &XRAccessibilityProfile) -> Result<()> {
        let mut errors = Vec::new();

        if let Some(ref exo) = self.exoskeleton {
            if let Some(ref integration) = profile.wia_integrations {
                if let Some(ref exo_settings) = integration.exoskeleton {
                    if let Err(e) = exo.sync_profile(exo_settings).await {
                        errors.push(format!("Exoskeleton: {}", e));
                    }
                }
            }
        }

        if let Some(ref eye) = self.bionic_eye {
            if let Some(ref integration) = profile.wia_integrations {
                if let Some(ref eye_settings) = integration.bionic_eye {
                    if let Err(e) = eye.sync_profile(eye_settings).await {
                        errors.push(format!("Bionic Eye: {}", e));
                    }
                }
            }
        }

        if let Some(ref vs) = self.voice_sign {
            if let Some(ref integration) = profile.wia_integrations {
                if let Some(ref vs_settings) = integration.voice_sign {
                    if let Err(e) = vs.sync_profile(vs_settings).await {
                        errors.push(format!("Voice-Sign: {}", e));
                    }
                }
            }
        }

        let mut state = self.sync_state.write().await;
        state.last_sync = Some(chrono::Utc::now());

        if errors.is_empty() {
            self.send_event(WIAEvent::SyncCompleted).await;
            Ok(())
        } else {
            let reason = errors.join("; ");
            self.send_event(WIAEvent::SyncFailed { reason: reason.clone() }).await;
            Err(XRAccessibilityError::WIAIntegration(
                WIAIntegrationError::SyncFailed(reason)
            ))
        }
    }

    /// Send haptic command to exoskeleton
    pub async fn send_haptic(&self, command: HapticCommand) -> Result<()> {
        if let Some(ref exo) = self.exoskeleton {
            exo.send_haptic(command).await.map_err(|e| {
                XRAccessibilityError::WIAIntegration(WIAIntegrationError::Exoskeleton(e.to_string()))
            })
        } else {
            Err(XRAccessibilityError::WIAIntegration(
                WIAIntegrationError::ServiceUnavailable("Exoskeleton not connected".into())
            ))
        }
    }

    /// Request sign language translation
    pub async fn translate_to_sign(&self, text: &str) -> Result<Vec<String>> {
        if let Some(ref vs) = self.voice_sign {
            vs.translate_to_sign(text).await.map_err(|e| {
                XRAccessibilityError::WIAIntegration(WIAIntegrationError::VoiceSign(e.to_string()))
            })
        } else {
            Err(XRAccessibilityError::WIAIntegration(
                WIAIntegrationError::ServiceUnavailable("Voice-Sign not connected".into())
            ))
        }
    }

    async fn send_event(&self, event: WIAEvent) {
        if let Some(ref sender) = self.event_sender {
            let _ = sender.send(event).await;
        }
    }
}

impl Default for WIAIntegrationHub {
    fn default() -> Self {
        Self::new()
    }
}

/// Exoskeleton integration trait
#[async_trait]
pub trait ExoskeletonAdapter: Send + Sync {
    /// Connect to exoskeleton
    async fn connect(&mut self, device_id: &str) -> WIAResult<()>;

    /// Disconnect
    async fn disconnect(&mut self) -> WIAResult<()>;

    /// Sync profile settings
    async fn sync_profile(&self, settings: &ExoskeletonIntegration) -> WIAResult<()>;

    /// Send haptic feedback command
    async fn send_haptic(&self, command: HapticCommand) -> WIAResult<()>;

    /// Check if connected
    fn is_connected(&self) -> bool;
}

/// Haptic command for exoskeleton
#[derive(Debug, Clone)]
pub struct HapticCommand {
    pub target: HapticTarget,
    pub intensity: f32,
    pub duration_ms: u64,
}

#[derive(Debug, Clone, Copy)]
pub enum HapticTarget {
    LeftHand,
    RightHand,
    LeftArm,
    RightArm,
    Back,
    Chest,
    Full,
}

/// Bionic eye integration trait
#[async_trait]
pub trait BionicEyeAdapter: Send + Sync {
    /// Connect to bionic eye
    async fn connect(&mut self, device_id: &str) -> WIAResult<()>;

    /// Disconnect
    async fn disconnect(&mut self) -> WIAResult<()>;

    /// Sync profile settings
    async fn sync_profile(&self, settings: &BionicEyeIntegration) -> WIAResult<()>;

    /// Check if connected
    fn is_connected(&self) -> bool;
}

/// Voice-Sign integration trait
#[async_trait]
pub trait VoiceSignAdapter: Send + Sync {
    /// Connect to voice-sign service
    async fn connect(&mut self) -> WIAResult<()>;

    /// Disconnect
    async fn disconnect(&mut self) -> WIAResult<()>;

    /// Sync profile settings
    async fn sync_profile(&self, settings: &VoiceSignIntegration) -> WIAResult<()>;

    /// Translate text to sign language
    async fn translate_to_sign(&self, text: &str) -> WIAResult<Vec<String>>;

    /// Translate sign to text
    async fn translate_from_sign(&self, signs: &[String]) -> WIAResult<String>;

    /// Check if connected
    fn is_connected(&self) -> bool;
}

/// Mock exoskeleton adapter
pub struct MockExoskeletonAdapter {
    connected: bool,
}

impl MockExoskeletonAdapter {
    pub fn new() -> Self {
        Self { connected: false }
    }
}

impl Default for MockExoskeletonAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl ExoskeletonAdapter for MockExoskeletonAdapter {
    async fn connect(&mut self, _device_id: &str) -> WIAResult<()> {
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> WIAResult<()> {
        self.connected = false;
        Ok(())
    }

    async fn sync_profile(&self, _settings: &ExoskeletonIntegration) -> WIAResult<()> {
        if !self.connected {
            return Err(WIAIntegrationError::ServiceUnavailable("Not connected".into()));
        }
        Ok(())
    }

    async fn send_haptic(&self, _command: HapticCommand) -> WIAResult<()> {
        if !self.connected {
            return Err(WIAIntegrationError::ServiceUnavailable("Not connected".into()));
        }
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}

/// Mock bionic eye adapter
pub struct MockBionicEyeAdapter {
    connected: bool,
}

impl MockBionicEyeAdapter {
    pub fn new() -> Self {
        Self { connected: false }
    }
}

impl Default for MockBionicEyeAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl BionicEyeAdapter for MockBionicEyeAdapter {
    async fn connect(&mut self, _device_id: &str) -> WIAResult<()> {
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> WIAResult<()> {
        self.connected = false;
        Ok(())
    }

    async fn sync_profile(&self, _settings: &BionicEyeIntegration) -> WIAResult<()> {
        if !self.connected {
            return Err(WIAIntegrationError::ServiceUnavailable("Not connected".into()));
        }
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}

/// Mock voice-sign adapter
pub struct MockVoiceSignAdapter {
    connected: bool,
}

impl MockVoiceSignAdapter {
    pub fn new() -> Self {
        Self { connected: false }
    }
}

impl Default for MockVoiceSignAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl VoiceSignAdapter for MockVoiceSignAdapter {
    async fn connect(&mut self) -> WIAResult<()> {
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> WIAResult<()> {
        self.connected = false;
        Ok(())
    }

    async fn sync_profile(&self, _settings: &VoiceSignIntegration) -> WIAResult<()> {
        if !self.connected {
            return Err(WIAIntegrationError::ServiceUnavailable("Not connected".into()));
        }
        Ok(())
    }

    async fn translate_to_sign(&self, text: &str) -> WIAResult<Vec<String>> {
        if !self.connected {
            return Err(WIAIntegrationError::ServiceUnavailable("Not connected".into()));
        }
        Ok(text.split_whitespace().map(|s| s.to_uppercase()).collect())
    }

    async fn translate_from_sign(&self, signs: &[String]) -> WIAResult<String> {
        if !self.connected {
            return Err(WIAIntegrationError::ServiceUnavailable("Not connected".into()));
        }
        Ok(signs.join(" ").to_lowercase())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_integration_hub() {
        let (hub, _rx) = WIAIntegrationHub::with_events();

        let state = hub.get_sync_state().await;
        assert!(!state.exoskeleton_connected);
    }

    #[tokio::test]
    async fn test_mock_exoskeleton() {
        let mut exo = MockExoskeletonAdapter::new();

        assert!(!exo.is_connected());

        exo.connect("test-device").await.unwrap();
        assert!(exo.is_connected());
    }

    #[tokio::test]
    async fn test_mock_voice_sign() {
        let mut vs = MockVoiceSignAdapter::new();

        vs.connect().await.unwrap();

        let signs = vs.translate_to_sign("hello world").await.unwrap();
        assert_eq!(signs, vec!["HELLO", "WORLD"]);

        let text = vs.translate_from_sign(&signs).await.unwrap();
        assert_eq!(text, "hello world");
    }
}
