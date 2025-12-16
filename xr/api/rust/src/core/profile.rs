//! Profile Management
//!
//! Load, save, and manage XR accessibility profiles.

use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

use crate::error::{ProfileError, ProfileResult, Result, XRAccessibilityError};
use crate::types::*;

/// Profile storage trait for persistence
#[async_trait]
pub trait ProfileStorage: Send + Sync {
    /// Load a profile by ID
    async fn load(&self, profile_id: &str) -> ProfileResult<XRAccessibilityProfile>;

    /// Save a profile
    async fn save(&self, profile: &XRAccessibilityProfile) -> ProfileResult<()>;

    /// Delete a profile
    async fn delete(&self, profile_id: &str) -> ProfileResult<()>;

    /// List all profile IDs
    async fn list(&self) -> ProfileResult<Vec<String>>;

    /// Check if profile exists
    async fn exists(&self, profile_id: &str) -> ProfileResult<bool>;
}

/// In-memory profile storage for testing
pub struct InMemoryProfileStorage {
    profiles: RwLock<HashMap<String, XRAccessibilityProfile>>,
}

impl InMemoryProfileStorage {
    pub fn new() -> Self {
        Self {
            profiles: RwLock::new(HashMap::new()),
        }
    }
}

impl Default for InMemoryProfileStorage {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl ProfileStorage for InMemoryProfileStorage {
    async fn load(&self, profile_id: &str) -> ProfileResult<XRAccessibilityProfile> {
        let profiles = self.profiles.read().await;
        profiles
            .get(profile_id)
            .cloned()
            .ok_or_else(|| ProfileError::NotFound(profile_id.to_string()))
    }

    async fn save(&self, profile: &XRAccessibilityProfile) -> ProfileResult<()> {
        let mut profiles = self.profiles.write().await;
        profiles.insert(profile.profile_id.clone(), profile.clone());
        Ok(())
    }

    async fn delete(&self, profile_id: &str) -> ProfileResult<()> {
        let mut profiles = self.profiles.write().await;
        if profiles.remove(profile_id).is_some() {
            Ok(())
        } else {
            Err(ProfileError::NotFound(profile_id.to_string()))
        }
    }

    async fn list(&self) -> ProfileResult<Vec<String>> {
        let profiles = self.profiles.read().await;
        Ok(profiles.keys().cloned().collect())
    }

    async fn exists(&self, profile_id: &str) -> ProfileResult<bool> {
        let profiles = self.profiles.read().await;
        Ok(profiles.contains_key(profile_id))
    }
}

/// Profile manager for handling profile operations
pub struct ProfileManager {
    storage: Arc<dyn ProfileStorage>,
    cache: RwLock<HashMap<String, XRAccessibilityProfile>>,
    cache_enabled: bool,
}

impl ProfileManager {
    /// Create a new profile manager with the given storage
    pub fn new(storage: Arc<dyn ProfileStorage>) -> Self {
        Self {
            storage,
            cache: RwLock::new(HashMap::new()),
            cache_enabled: true,
        }
    }

    /// Create a new profile manager with in-memory storage
    pub fn in_memory() -> Self {
        Self::new(Arc::new(InMemoryProfileStorage::new()))
    }

    /// Disable caching
    pub fn disable_cache(&mut self) {
        self.cache_enabled = false;
    }

    /// Get a profile by ID
    pub async fn get(&self, profile_id: &str) -> Result<XRAccessibilityProfile> {
        if self.cache_enabled {
            let cache = self.cache.read().await;
            if let Some(profile) = cache.get(profile_id) {
                return Ok(profile.clone());
            }
        }

        let profile = self.storage.load(profile_id).await?;

        if self.cache_enabled {
            let mut cache = self.cache.write().await;
            cache.insert(profile_id.to_string(), profile.clone());
        }

        Ok(profile)
    }

    /// Create a new profile
    pub async fn create(&self, profile: XRAccessibilityProfile) -> Result<()> {
        if self.storage.exists(&profile.profile_id).await? {
            return Err(XRAccessibilityError::Profile(
                ProfileError::AlreadyExists(profile.profile_id.clone())
            ));
        }

        self.validate(&profile)?;
        self.storage.save(&profile).await?;

        if self.cache_enabled {
            let mut cache = self.cache.write().await;
            cache.insert(profile.profile_id.clone(), profile);
        }

        Ok(())
    }

    /// Update an existing profile
    pub async fn update(&self, profile: XRAccessibilityProfile) -> Result<()> {
        if !self.storage.exists(&profile.profile_id).await? {
            return Err(XRAccessibilityError::Profile(
                ProfileError::NotFound(profile.profile_id.clone())
            ));
        }

        self.validate(&profile)?;
        self.storage.save(&profile).await?;

        if self.cache_enabled {
            let mut cache = self.cache.write().await;
            cache.insert(profile.profile_id.clone(), profile);
        }

        Ok(())
    }

    /// Delete a profile
    pub async fn delete(&self, profile_id: &str) -> Result<()> {
        self.storage.delete(profile_id).await?;

        if self.cache_enabled {
            let mut cache = self.cache.write().await;
            cache.remove(profile_id);
        }

        Ok(())
    }

    /// List all profile IDs
    pub async fn list(&self) -> Result<Vec<String>> {
        Ok(self.storage.list().await?)
    }

    /// Export a profile to JSON
    pub async fn export_json(&self, profile_id: &str) -> Result<String> {
        let profile = self.get(profile_id).await?;
        let json = serde_json::to_string_pretty(&profile)?;
        Ok(json)
    }

    /// Import a profile from JSON
    pub async fn import_json(&self, json: &str) -> Result<XRAccessibilityProfile> {
        let profile: XRAccessibilityProfile = serde_json::from_str(json)
            .map_err(|e| ProfileError::ImportFailed(e.to_string()))?;

        self.create(profile.clone()).await?;
        Ok(profile)
    }

    /// Clone a profile with a new ID
    pub async fn clone_profile(
        &self,
        source_id: &str,
        new_id: String,
    ) -> Result<XRAccessibilityProfile> {
        let mut profile = self.get(source_id).await?;
        profile.profile_id = new_id;
        profile.created_at = chrono::Utc::now();
        profile.updated_at = chrono::Utc::now();

        self.create(profile.clone()).await?;
        Ok(profile)
    }

    fn validate(&self, profile: &XRAccessibilityProfile) -> Result<()> {
        if profile.profile_id.is_empty() {
            return Err(XRAccessibilityError::Validation(
                "Profile ID cannot be empty".into()
            ));
        }

        let version_parts: Vec<&str> = profile.version.split('.').collect();
        if version_parts.len() != 3 {
            return Err(XRAccessibilityError::Validation(
                "Invalid version format".into()
            ));
        }

        Ok(())
    }

    /// Clear the cache
    pub async fn clear_cache(&self) {
        let mut cache = self.cache.write().await;
        cache.clear();
    }
}

/// Profile builder for creating profiles
pub struct ProfileBuilder {
    profile_id: String,
    user_hash: Option<String>,
    disabilities: DisabilityProfile,
}

impl ProfileBuilder {
    /// Create a new profile builder
    pub fn new(profile_id: impl Into<String>) -> Self {
        Self {
            profile_id: profile_id.into(),
            user_hash: None,
            disabilities: DisabilityProfile::default(),
        }
    }

    /// Set the user hash
    pub fn user_id(mut self, user_id: impl Into<String>) -> Self {
        self.user_hash = Some(user_id.into());
        self
    }

    /// Set visual disability
    pub fn visual_disability(mut self, visual: VisualDisability) -> Self {
        self.disabilities.visual = Some(visual);
        self
    }

    /// Set auditory disability
    pub fn auditory_disability(mut self, auditory: AuditoryDisability) -> Self {
        self.disabilities.auditory = Some(auditory);
        self
    }

    /// Set motor disability
    pub fn motor_disability(mut self, motor: MotorDisability) -> Self {
        self.disabilities.motor = Some(motor);
        self
    }

    /// Set cognitive disability
    pub fn cognitive_disability(mut self, cognitive: CognitiveDisability) -> Self {
        self.disabilities.cognitive = Some(cognitive);
        self
    }

    /// Set speech disability
    pub fn speech_disability(mut self, speech: SpeechDisability) -> Self {
        self.disabilities.speech = Some(speech);
        self
    }

    /// Build the profile
    pub fn build(self) -> XRAccessibilityProfile {
        let mut profile = XRAccessibilityProfile::default();
        profile.profile_id = self.profile_id;
        profile.user_hash = self.user_hash;
        profile.disabilities = self.disabilities;
        profile
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_profile_manager_create() {
        let manager = ProfileManager::in_memory();
        let profile = ProfileBuilder::new("test-001")
            .user_id("user-001")
            .build();

        let result = manager.create(profile).await;
        assert!(result.is_ok());

        let loaded = manager.get("test-001").await;
        assert!(loaded.is_ok());
    }

    #[tokio::test]
    async fn test_profile_not_found() {
        let manager = ProfileManager::in_memory();
        let result = manager.get("nonexistent").await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_profile_duplicate() {
        let manager = ProfileManager::in_memory();
        let profile = ProfileBuilder::new("test-001").build();

        manager.create(profile.clone()).await.unwrap();
        let result = manager.create(profile).await;
        assert!(result.is_err());
    }
}
