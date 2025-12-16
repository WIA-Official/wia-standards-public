//! Storage Adapters
//! 弘益人間 - Education for Everyone

use async_trait::async_trait;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, RwLock};
use tokio::fs;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::error::{EduError, Result};
use crate::types::*;

/// Trait for profile storage backends
#[async_trait]
pub trait ProfileStorage: Send + Sync + std::fmt::Debug {
    /// Save a learner profile
    async fn save_profile(&self, profile: &LearnerProfile) -> Result<()>;

    /// Load a learner profile
    async fn load_profile(&self, profile_id: ProfileId) -> Result<LearnerProfile>;

    /// Delete a learner profile
    async fn delete_profile(&self, profile_id: ProfileId) -> Result<()>;

    /// List all profile IDs
    async fn list_profile_ids(&self) -> Result<Vec<ProfileId>>;

    /// Check if a profile exists
    async fn profile_exists(&self, profile_id: ProfileId) -> Result<bool>;
}

/// File-based storage adapter
#[derive(Debug)]
pub struct FileStorage {
    base_path: PathBuf,
}

impl FileStorage {
    /// Create a new file storage adapter
    pub fn new(base_path: PathBuf) -> Self {
        Self { base_path }
    }

    /// Get the path for a profile
    fn profile_path(&self, profile_id: ProfileId) -> PathBuf {
        self.base_path.join(format!("{}.json", profile_id))
    }

    /// Ensure the base directory exists
    async fn ensure_dir(&self) -> Result<()> {
        if !self.base_path.exists() {
            fs::create_dir_all(&self.base_path).await?;
        }
        Ok(())
    }
}

#[async_trait]
impl ProfileStorage for FileStorage {
    async fn save_profile(&self, profile: &LearnerProfile) -> Result<()> {
        self.ensure_dir().await?;

        let path = self.profile_path(profile.profile_id);
        let json = serde_json::to_string_pretty(profile)?;

        let mut file = fs::File::create(&path).await?;
        file.write_all(json.as_bytes()).await?;

        Ok(())
    }

    async fn load_profile(&self, profile_id: ProfileId) -> Result<LearnerProfile> {
        let path = self.profile_path(profile_id);

        if !path.exists() {
            return Err(EduError::ProfileNotFound(profile_id));
        }

        let mut file = fs::File::open(&path).await?;
        let mut contents = String::new();
        file.read_to_string(&mut contents).await?;

        let profile: LearnerProfile = serde_json::from_str(&contents)?;
        Ok(profile)
    }

    async fn delete_profile(&self, profile_id: ProfileId) -> Result<()> {
        let path = self.profile_path(profile_id);

        if !path.exists() {
            return Err(EduError::ProfileNotFound(profile_id));
        }

        fs::remove_file(&path).await?;
        Ok(())
    }

    async fn list_profile_ids(&self) -> Result<Vec<ProfileId>> {
        self.ensure_dir().await?;

        let mut ids = Vec::new();
        let mut entries = fs::read_dir(&self.base_path).await?;

        while let Some(entry) = entries.next_entry().await? {
            let path = entry.path();
            if path.extension().map(|e| e == "json").unwrap_or(false) {
                if let Some(stem) = path.file_stem() {
                    if let Ok(id) = stem.to_string_lossy().parse() {
                        ids.push(id);
                    }
                }
            }
        }

        Ok(ids)
    }

    async fn profile_exists(&self, profile_id: ProfileId) -> Result<bool> {
        let path = self.profile_path(profile_id);
        Ok(path.exists())
    }
}

/// In-memory storage adapter (for testing)
#[derive(Debug, Default)]
pub struct MemoryStorage {
    profiles: Arc<RwLock<HashMap<ProfileId, LearnerProfile>>>,
}

impl MemoryStorage {
    /// Create a new in-memory storage
    pub fn new() -> Self {
        Self {
            profiles: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

#[async_trait]
impl ProfileStorage for MemoryStorage {
    async fn save_profile(&self, profile: &LearnerProfile) -> Result<()> {
        let mut profiles = self.profiles.write().unwrap();
        profiles.insert(profile.profile_id, profile.clone());
        Ok(())
    }

    async fn load_profile(&self, profile_id: ProfileId) -> Result<LearnerProfile> {
        let profiles = self.profiles.read().unwrap();
        profiles
            .get(&profile_id)
            .cloned()
            .ok_or(EduError::ProfileNotFound(profile_id))
    }

    async fn delete_profile(&self, profile_id: ProfileId) -> Result<()> {
        let mut profiles = self.profiles.write().unwrap();
        profiles
            .remove(&profile_id)
            .ok_or(EduError::ProfileNotFound(profile_id))?;
        Ok(())
    }

    async fn list_profile_ids(&self) -> Result<Vec<ProfileId>> {
        let profiles = self.profiles.read().unwrap();
        Ok(profiles.keys().copied().collect())
    }

    async fn profile_exists(&self, profile_id: ProfileId) -> Result<bool> {
        let profiles = self.profiles.read().unwrap();
        Ok(profiles.contains_key(&profile_id))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use uuid::Uuid;

    fn create_test_profile() -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo::default(),
            disability_profile: DisabilityProfile::default(),
            display_preferences: DisplayPreferences::default(),
            control_preferences: ControlPreferences::default(),
            content_preferences: ContentPreferences::default(),
            learning_style: LearningStyle::default(),
            assessment_accommodations: AssessmentAccommodations::default(),
            assistive_technology: AssistiveTechnology::default(),
            wia_integrations: WIAIntegrations::default(),
        }
    }

    #[tokio::test]
    async fn test_memory_storage() {
        let storage = MemoryStorage::new();
        let profile = create_test_profile();
        let id = profile.profile_id;

        // Save
        storage.save_profile(&profile).await.unwrap();

        // Check exists
        assert!(storage.profile_exists(id).await.unwrap());

        // Load
        let loaded = storage.load_profile(id).await.unwrap();
        assert_eq!(loaded.profile_id, id);

        // List
        let ids = storage.list_profile_ids().await.unwrap();
        assert!(ids.contains(&id));

        // Delete
        storage.delete_profile(id).await.unwrap();
        assert!(!storage.profile_exists(id).await.unwrap());
    }
}
