//! Profile Synchronization Manager
//! 弘益人間 - Sync learner profiles across educational platforms

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};
use std::collections::HashMap;

use crate::error::Result;
use crate::types::LearnerProfile;
use super::conflict::{SyncConflict, ConflictResolution, ConflictResolver};

/// Sync direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SyncDirection {
    /// Push local changes to remote
    Push,
    /// Pull remote changes to local
    Pull,
    /// Bidirectional sync
    Bidirectional,
}

/// Sync status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SyncStatus {
    /// Sync pending
    Pending,
    /// Sync in progress
    InProgress,
    /// Sync completed successfully
    Completed,
    /// Sync failed
    Failed,
    /// Sync has conflicts
    Conflict,
}

/// Profile version information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileVersion {
    /// Profile ID
    pub profile_id: Uuid,
    /// Version number (monotonically increasing)
    pub version: u64,
    /// Last modified timestamp
    pub last_modified: DateTime<Utc>,
    /// Hash of profile content
    pub content_hash: String,
    /// Platform that made the change
    pub modified_by: String,
}

impl ProfileVersion {
    /// Create a new version
    pub fn new(profile_id: Uuid, version: u64, platform: &str, profile: &LearnerProfile) -> Self {
        Self {
            profile_id,
            version,
            last_modified: Utc::now(),
            content_hash: Self::compute_hash(profile),
            modified_by: platform.to_string(),
        }
    }

    /// Compute content hash for profile
    pub fn compute_hash(profile: &LearnerProfile) -> String {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let json = serde_json::to_string(profile).unwrap_or_default();
        let mut hasher = DefaultHasher::new();
        json.hash(&mut hasher);
        format!("{:016x}", hasher.finish())
    }

    /// Check if this version is newer than another
    pub fn is_newer_than(&self, other: &ProfileVersion) -> bool {
        self.version > other.version
    }
}

/// Sync configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyncConfig {
    /// Local platform identifier
    pub local_platform: String,
    /// Remote platform identifiers
    pub remote_platforms: Vec<String>,
    /// Sync direction
    pub direction: SyncDirection,
    /// Auto-resolve conflicts
    pub auto_resolve: bool,
    /// Conflict resolution strategy
    pub resolution_strategy: ConflictResolutionStrategy,
    /// Fields to sync (empty = all fields)
    pub sync_fields: Vec<String>,
    /// Fields to exclude from sync
    pub exclude_fields: Vec<String>,
}

impl Default for SyncConfig {
    fn default() -> Self {
        Self {
            local_platform: "local".to_string(),
            remote_platforms: vec![],
            direction: SyncDirection::Bidirectional,
            auto_resolve: true,
            resolution_strategy: ConflictResolutionStrategy::LastWriteWins,
            sync_fields: vec![],
            exclude_fields: vec![],
        }
    }
}

/// Conflict resolution strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConflictResolutionStrategy {
    /// Most recent change wins
    LastWriteWins,
    /// Local changes take priority
    LocalWins,
    /// Remote changes take priority
    RemoteWins,
    /// Merge changes (field by field)
    Merge,
    /// Always ask user
    Manual,
}

/// Sync result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyncResult {
    /// Sync ID
    pub sync_id: Uuid,
    /// Profile ID
    pub profile_id: Uuid,
    /// Sync status
    pub status: SyncStatus,
    /// Local version after sync
    pub local_version: Option<ProfileVersion>,
    /// Remote versions after sync
    pub remote_versions: HashMap<String, ProfileVersion>,
    /// Conflicts encountered
    pub conflicts: Vec<SyncConflict>,
    /// Resolved conflicts
    pub resolved: Vec<ConflictResolution>,
    /// Sync timestamp
    pub timestamp: DateTime<Utc>,
    /// Error message if failed
    pub error: Option<String>,
}

impl SyncResult {
    /// Create a successful sync result
    pub fn success(profile_id: Uuid, local_version: ProfileVersion) -> Self {
        Self {
            sync_id: Uuid::new_v4(),
            profile_id,
            status: SyncStatus::Completed,
            local_version: Some(local_version),
            remote_versions: HashMap::new(),
            conflicts: vec![],
            resolved: vec![],
            timestamp: Utc::now(),
            error: None,
        }
    }

    /// Create a failed sync result
    pub fn failed(profile_id: Uuid, error: &str) -> Self {
        Self {
            sync_id: Uuid::new_v4(),
            profile_id,
            status: SyncStatus::Failed,
            local_version: None,
            remote_versions: HashMap::new(),
            conflicts: vec![],
            resolved: vec![],
            timestamp: Utc::now(),
            error: Some(error.to_string()),
        }
    }

    /// Create a conflict result
    pub fn conflict(profile_id: Uuid, conflicts: Vec<SyncConflict>) -> Self {
        Self {
            sync_id: Uuid::new_v4(),
            profile_id,
            status: SyncStatus::Conflict,
            local_version: None,
            remote_versions: HashMap::new(),
            conflicts,
            resolved: vec![],
            timestamp: Utc::now(),
            error: None,
        }
    }

    /// Check if sync was successful
    pub fn is_success(&self) -> bool {
        self.status == SyncStatus::Completed
    }

    /// Check if sync has conflicts
    pub fn has_conflicts(&self) -> bool {
        self.status == SyncStatus::Conflict || !self.conflicts.is_empty()
    }
}

/// Profile Sync Manager
pub struct ProfileSyncManager {
    /// Sync configuration
    config: SyncConfig,
    /// Local profile versions
    local_versions: HashMap<Uuid, ProfileVersion>,
    /// Remote profile versions by platform
    remote_versions: HashMap<String, HashMap<Uuid, ProfileVersion>>,
    /// Conflict resolver
    conflict_resolver: ConflictResolver,
    /// Sync history
    sync_history: Vec<SyncResult>,
}

impl ProfileSyncManager {
    /// Create a new sync manager
    pub fn new(config: SyncConfig) -> Self {
        Self {
            config,
            local_versions: HashMap::new(),
            remote_versions: HashMap::new(),
            conflict_resolver: ConflictResolver::new(),
            sync_history: vec![],
        }
    }

    /// Register a local profile for syncing
    pub fn register_profile(&mut self, profile: &LearnerProfile) -> ProfileVersion {
        let version = ProfileVersion::new(
            profile.profile_id,
            1,
            &self.config.local_platform,
            profile,
        );
        self.local_versions.insert(profile.profile_id, version.clone());
        version
    }

    /// Update local profile version
    pub fn update_local_version(&mut self, profile: &LearnerProfile) -> ProfileVersion {
        let current_version = self.local_versions
            .get(&profile.profile_id)
            .map(|v| v.version)
            .unwrap_or(0);

        let new_version = ProfileVersion::new(
            profile.profile_id,
            current_version + 1,
            &self.config.local_platform,
            profile,
        );
        self.local_versions.insert(profile.profile_id, new_version.clone());
        new_version
    }

    /// Register remote profile version
    pub fn register_remote_version(&mut self, platform: &str, version: ProfileVersion) {
        let platform_versions = self.remote_versions
            .entry(platform.to_string())
            .or_insert_with(HashMap::new);
        platform_versions.insert(version.profile_id, version);
    }

    /// Get local version for a profile
    pub fn get_local_version(&self, profile_id: &Uuid) -> Option<&ProfileVersion> {
        self.local_versions.get(profile_id)
    }

    /// Get remote version for a profile on a platform
    pub fn get_remote_version(&self, platform: &str, profile_id: &Uuid) -> Option<&ProfileVersion> {
        self.remote_versions
            .get(platform)
            .and_then(|versions| versions.get(profile_id))
    }

    /// Check if profile needs sync
    pub fn needs_sync(&self, profile_id: &Uuid) -> bool {
        let local = self.local_versions.get(profile_id);

        for (_platform, versions) in &self.remote_versions {
            if let Some(remote) = versions.get(profile_id) {
                match local {
                    Some(l) if l.content_hash != remote.content_hash => return true,
                    None => return true,
                    _ => {}
                }
            }
        }

        false
    }

    /// Prepare sync by comparing versions
    pub fn prepare_sync(
        &self,
        profile_id: &Uuid,
        local_profile: &LearnerProfile,
        remote_profile: &LearnerProfile,
        remote_platform: &str,
    ) -> Result<SyncPreparation> {
        let local_version = self.local_versions.get(profile_id);
        let remote_version = self.remote_versions
            .get(remote_platform)
            .and_then(|v| v.get(profile_id));

        // Check for conflicts
        let local_hash = ProfileVersion::compute_hash(local_profile);
        let remote_hash = ProfileVersion::compute_hash(remote_profile);

        if local_hash == remote_hash {
            return Ok(SyncPreparation::NoChanges);
        }

        // Determine if there's a conflict
        match (local_version, remote_version) {
            (Some(l), Some(r)) => {
                if l.version == r.version && l.content_hash != r.content_hash {
                    // Same version but different content = conflict
                    let conflicts = self.detect_conflicts(local_profile, remote_profile);
                    if conflicts.is_empty() {
                        Ok(SyncPreparation::ReadyToSync {
                            direction: self.config.direction,
                        })
                    } else {
                        Ok(SyncPreparation::HasConflicts { conflicts })
                    }
                } else if l.version > r.version {
                    Ok(SyncPreparation::ReadyToSync {
                        direction: SyncDirection::Push,
                    })
                } else {
                    Ok(SyncPreparation::ReadyToSync {
                        direction: SyncDirection::Pull,
                    })
                }
            }
            (Some(_), None) => Ok(SyncPreparation::ReadyToSync {
                direction: SyncDirection::Push,
            }),
            (None, Some(_)) => Ok(SyncPreparation::ReadyToSync {
                direction: SyncDirection::Pull,
            }),
            (None, None) => Ok(SyncPreparation::NoChanges),
        }
    }

    /// Detect conflicts between profiles
    fn detect_conflicts(
        &self,
        local: &LearnerProfile,
        remote: &LearnerProfile,
    ) -> Vec<SyncConflict> {
        let mut conflicts = vec![];

        // Compare display preferences
        if local.display_preferences.screen_reader.enabled
            != remote.display_preferences.screen_reader.enabled
        {
            conflicts.push(SyncConflict::new(
                "display_preferences.screen_reader.enabled",
                serde_json::to_value(local.display_preferences.screen_reader.enabled).ok(),
                serde_json::to_value(remote.display_preferences.screen_reader.enabled).ok(),
            ));
        }

        // Compare content preferences
        if local.content_preferences.captions.required
            != remote.content_preferences.captions.required
        {
            conflicts.push(SyncConflict::new(
                "content_preferences.captions.required",
                serde_json::to_value(local.content_preferences.captions.required).ok(),
                serde_json::to_value(remote.content_preferences.captions.required).ok(),
            ));
        }

        // Compare assessment accommodations
        if local.assessment_accommodations.timing.time_multiplier
            != remote.assessment_accommodations.timing.time_multiplier
        {
            conflicts.push(SyncConflict::new(
                "assessment_accommodations.timing.time_multiplier",
                serde_json::to_value(local.assessment_accommodations.timing.time_multiplier).ok(),
                serde_json::to_value(remote.assessment_accommodations.timing.time_multiplier).ok(),
            ));
        }

        conflicts
    }

    /// Execute sync with auto-resolution
    pub fn sync(
        &mut self,
        profile_id: Uuid,
        local_profile: &mut LearnerProfile,
        remote_profile: &LearnerProfile,
        remote_platform: &str,
    ) -> SyncResult {
        // Prepare sync
        let preparation = match self.prepare_sync(
            &profile_id,
            local_profile,
            remote_profile,
            remote_platform,
        ) {
            Ok(p) => p,
            Err(e) => return SyncResult::failed(profile_id, &e.to_string()),
        };

        match preparation {
            SyncPreparation::NoChanges => {
                let version = self.get_local_version(&profile_id)
                    .cloned()
                    .unwrap_or_else(|| self.register_profile(local_profile));
                SyncResult::success(profile_id, version)
            }
            SyncPreparation::ReadyToSync { direction } => {
                match direction {
                    SyncDirection::Pull => {
                        // Apply remote changes to local
                        *local_profile = remote_profile.clone();
                    }
                    SyncDirection::Push => {
                        // Local changes will be pushed (no local modification needed)
                    }
                    SyncDirection::Bidirectional => {
                        // Merge (simplified: take newer)
                        let local_v = self.get_local_version(&profile_id);
                        let remote_v = self.get_remote_version(remote_platform, &profile_id);

                        match (local_v, remote_v) {
                            (Some(l), Some(r)) if r.last_modified > l.last_modified => {
                                *local_profile = remote_profile.clone();
                            }
                            _ => {}
                        }
                    }
                }

                let version = self.update_local_version(local_profile);
                SyncResult::success(profile_id, version)
            }
            SyncPreparation::HasConflicts { conflicts } => {
                if self.config.auto_resolve {
                    // Auto-resolve conflicts
                    let resolved = self.conflict_resolver.resolve_all(
                        &conflicts,
                        self.config.resolution_strategy,
                    );

                    // Apply resolutions
                    for resolution in &resolved {
                        self.apply_resolution(local_profile, resolution);
                    }

                    let version = self.update_local_version(local_profile);
                    let mut result = SyncResult::success(profile_id, version);
                    result.resolved = resolved;
                    result
                } else {
                    SyncResult::conflict(profile_id, conflicts)
                }
            }
        }
    }

    /// Apply a conflict resolution to a profile
    fn apply_resolution(&self, profile: &mut LearnerProfile, resolution: &ConflictResolution) {
        let value = match resolution.chosen_value.as_ref() {
            Some(v) => v,
            None => return,
        };

        // Apply based on field path
        match resolution.field_path.as_str() {
            "display_preferences.screen_reader.enabled" => {
                if let Some(enabled) = value.as_bool() {
                    profile.display_preferences.screen_reader.enabled = enabled;
                }
            }
            "content_preferences.captions.required" => {
                if let Some(required) = value.as_bool() {
                    profile.content_preferences.captions.required = required;
                }
            }
            "assessment_accommodations.timing.time_multiplier" => {
                if let Some(multiplier) = value.as_f64() {
                    profile.assessment_accommodations.timing.time_multiplier = multiplier as f32;
                }
            }
            _ => {}
        }
    }

    /// Get sync history
    pub fn get_history(&self) -> &[SyncResult] {
        &self.sync_history
    }

    /// Get configuration
    pub fn config(&self) -> &SyncConfig {
        &self.config
    }
}

/// Sync preparation result
#[derive(Debug, Clone)]
pub enum SyncPreparation {
    /// No changes to sync
    NoChanges,
    /// Ready to sync in specified direction
    ReadyToSync { direction: SyncDirection },
    /// Has conflicts that need resolution
    HasConflicts { conflicts: Vec<SyncConflict> },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::ProfileManager;
    use crate::types::DisabilityType;

    fn create_test_profile() -> LearnerProfile {
        let mut manager = ProfileManager::new();
        let mut profile = manager.create_profile_for_disability(DisabilityType::Blind);
        profile.profile_id = Uuid::new_v4();
        profile
    }

    #[test]
    fn test_register_profile() {
        let config = SyncConfig::default();
        let mut manager = ProfileSyncManager::new(config);

        let profile = create_test_profile();
        let version = manager.register_profile(&profile);

        assert_eq!(version.version, 1);
        assert_eq!(version.profile_id, profile.profile_id);
    }

    #[test]
    fn test_update_version() {
        let config = SyncConfig::default();
        let mut manager = ProfileSyncManager::new(config);

        let mut profile = create_test_profile();
        manager.register_profile(&profile);

        profile.display_preferences.magnification.enabled = true;
        let version = manager.update_local_version(&profile);

        assert_eq!(version.version, 2);
    }

    #[test]
    fn test_needs_sync() {
        let config = SyncConfig::default();
        let mut manager = ProfileSyncManager::new(config);

        let profile = create_test_profile();
        let version = manager.register_profile(&profile);

        // Create different remote version
        let mut remote_version = version.clone();
        remote_version.content_hash = "different_hash".to_string();

        manager.register_remote_version("remote-lms", remote_version);

        assert!(manager.needs_sync(&profile.profile_id));
    }

    #[test]
    fn test_sync_no_changes() {
        let config = SyncConfig::default();
        let mut manager = ProfileSyncManager::new(config);

        let mut profile = create_test_profile();
        let remote = profile.clone();

        let result = manager.sync(
            profile.profile_id,
            &mut profile,
            &remote,
            "remote-lms",
        );

        assert!(result.is_success());
        assert!(!result.has_conflicts());
    }

    #[test]
    fn test_detect_conflicts() {
        let config = SyncConfig::default();
        let manager = ProfileSyncManager::new(config);

        let mut local = create_test_profile();
        local.display_preferences.screen_reader.enabled = true;

        let mut remote = local.clone();
        remote.display_preferences.screen_reader.enabled = false;

        let conflicts = manager.detect_conflicts(&local, &remote);
        assert!(!conflicts.is_empty());
    }
}
