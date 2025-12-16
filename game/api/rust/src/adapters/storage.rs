//! File-based Profile Storage
//! 弘益人間 - Gaming for Everyone

use super::ProfileStorage;
use crate::error::{GameError, Result};
use crate::types::*;
use async_trait::async_trait;
use std::path::PathBuf;

/// File-based profile storage
#[derive(Debug, Clone)]
pub struct FileProfileStorage {
    base_path: PathBuf,
}

impl FileProfileStorage {
    /// Create a new file storage with the given base path
    pub fn new(base_path: PathBuf) -> Self {
        Self { base_path }
    }

    /// Get the file path for a profile
    fn profile_path(&self, profile_id: ProfileId) -> PathBuf {
        self.base_path.join(format!("{}.wia.json", profile_id))
    }

    /// Ensure the base directory exists
    async fn ensure_dir(&self) -> Result<()> {
        tokio::fs::create_dir_all(&self.base_path)
            .await
            .map_err(|e| GameError::IoError(e))
    }
}

#[async_trait]
impl ProfileStorage for FileProfileStorage {
    async fn save_profile(&self, profile: &PlayerProfile) -> Result<()> {
        self.ensure_dir().await?;

        let path = self.profile_path(profile.profile_id);
        let json = serde_json::to_string_pretty(profile)?;

        tokio::fs::write(&path, json)
            .await
            .map_err(|e| GameError::IoError(e))
    }

    async fn load_profile(&self, profile_id: ProfileId) -> Result<PlayerProfile> {
        let path = self.profile_path(profile_id);

        let json = tokio::fs::read_to_string(&path)
            .await
            .map_err(|_| GameError::ProfileNotFound(profile_id))?;

        serde_json::from_str(&json).map_err(|e| GameError::SerializationError(e))
    }

    async fn delete_profile(&self, profile_id: ProfileId) -> Result<()> {
        let path = self.profile_path(profile_id);

        tokio::fs::remove_file(&path)
            .await
            .map_err(|_| GameError::ProfileNotFound(profile_id))
    }

    async fn list_profiles(&self) -> Result<Vec<ProfileId>> {
        self.ensure_dir().await?;

        let mut profiles = Vec::new();
        let mut entries = tokio::fs::read_dir(&self.base_path)
            .await
            .map_err(|e| GameError::IoError(e))?;

        while let Some(entry) = entries.next_entry().await.map_err(|e| GameError::IoError(e))? {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                if name.ends_with(".wia.json") {
                    let id_str = name.trim_end_matches(".wia.json");
                    if let Ok(id) = uuid::Uuid::parse_str(id_str) {
                        profiles.push(id);
                    }
                }
            }
        }

        Ok(profiles)
    }

    async fn exists(&self, profile_id: ProfileId) -> Result<bool> {
        let path = self.profile_path(profile_id);
        Ok(path.exists())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::env::temp_dir;

    #[tokio::test]
    async fn test_file_storage() {
        let temp_path = temp_dir().join("wia_game_test");
        let storage = FileProfileStorage::new(temp_path.clone());

        let profile = PlayerProfile::default();
        let id = profile.profile_id;

        // Save
        storage.save_profile(&profile).await.unwrap();
        assert!(storage.exists(id).await.unwrap());

        // Load
        let loaded = storage.load_profile(id).await.unwrap();
        assert_eq!(loaded.profile_id, id);

        // List
        let profiles = storage.list_profiles().await.unwrap();
        assert!(profiles.contains(&id));

        // Delete
        storage.delete_profile(id).await.unwrap();
        assert!(!storage.exists(id).await.unwrap());

        // Cleanup
        let _ = std::fs::remove_dir_all(temp_path);
    }
}
