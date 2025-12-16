//! Sync Conflict Resolution
//! 弘益人間 - Resolve profile sync conflicts

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

use super::profile_sync::ConflictResolutionStrategy;

/// Sync conflict between local and remote profiles
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyncConflict {
    /// Conflict ID
    pub conflict_id: Uuid,
    /// Field path with conflict
    pub field_path: String,
    /// Local value
    pub local_value: Option<serde_json::Value>,
    /// Remote value
    pub remote_value: Option<serde_json::Value>,
    /// Conflict detected timestamp
    pub detected_at: DateTime<Utc>,
    /// Conflict description
    pub description: Option<String>,
}

impl SyncConflict {
    /// Create a new conflict
    pub fn new(
        field_path: &str,
        local_value: Option<serde_json::Value>,
        remote_value: Option<serde_json::Value>,
    ) -> Self {
        Self {
            conflict_id: Uuid::new_v4(),
            field_path: field_path.to_string(),
            local_value,
            remote_value,
            detected_at: Utc::now(),
            description: None,
        }
    }

    /// Add description
    pub fn with_description(mut self, description: &str) -> Self {
        self.description = Some(description.to_string());
        self
    }

    /// Get human-readable conflict description
    pub fn describe(&self) -> String {
        let local = self.local_value
            .as_ref()
            .map(|v| v.to_string())
            .unwrap_or_else(|| "none".to_string());
        let remote = self.remote_value
            .as_ref()
            .map(|v| v.to_string())
            .unwrap_or_else(|| "none".to_string());

        format!(
            "Conflict in '{}': local={}, remote={}",
            self.field_path, local, remote
        )
    }
}

/// Resolution for a sync conflict
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConflictResolution {
    /// Resolution ID
    pub resolution_id: Uuid,
    /// Original conflict ID
    pub conflict_id: Uuid,
    /// Field path
    pub field_path: String,
    /// Chosen value
    pub chosen_value: Option<serde_json::Value>,
    /// Resolution source (local, remote, merged, custom)
    pub source: ResolutionSource,
    /// Resolution timestamp
    pub resolved_at: DateTime<Utc>,
    /// Resolution reason
    pub reason: Option<String>,
}

/// Source of the resolution
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResolutionSource {
    /// Local value was chosen
    Local,
    /// Remote value was chosen
    Remote,
    /// Values were merged
    Merged,
    /// Custom value was provided
    Custom,
    /// User selected manually
    Manual,
}

impl ConflictResolution {
    /// Create resolution choosing local value
    pub fn choose_local(conflict: &SyncConflict) -> Self {
        Self {
            resolution_id: Uuid::new_v4(),
            conflict_id: conflict.conflict_id,
            field_path: conflict.field_path.clone(),
            chosen_value: conflict.local_value.clone(),
            source: ResolutionSource::Local,
            resolved_at: Utc::now(),
            reason: Some("Local value chosen".to_string()),
        }
    }

    /// Create resolution choosing remote value
    pub fn choose_remote(conflict: &SyncConflict) -> Self {
        Self {
            resolution_id: Uuid::new_v4(),
            conflict_id: conflict.conflict_id,
            field_path: conflict.field_path.clone(),
            chosen_value: conflict.remote_value.clone(),
            source: ResolutionSource::Remote,
            resolved_at: Utc::now(),
            reason: Some("Remote value chosen".to_string()),
        }
    }

    /// Create resolution with custom value
    pub fn custom(conflict: &SyncConflict, value: serde_json::Value, reason: &str) -> Self {
        Self {
            resolution_id: Uuid::new_v4(),
            conflict_id: conflict.conflict_id,
            field_path: conflict.field_path.clone(),
            chosen_value: Some(value),
            source: ResolutionSource::Custom,
            resolved_at: Utc::now(),
            reason: Some(reason.to_string()),
        }
    }

    /// Create resolution by merging
    pub fn merged(conflict: &SyncConflict, merged_value: serde_json::Value) -> Self {
        Self {
            resolution_id: Uuid::new_v4(),
            conflict_id: conflict.conflict_id,
            field_path: conflict.field_path.clone(),
            chosen_value: Some(merged_value),
            source: ResolutionSource::Merged,
            resolved_at: Utc::now(),
            reason: Some("Values merged".to_string()),
        }
    }
}

/// Conflict Resolver
pub struct ConflictResolver {
    /// Default strategy
    default_strategy: ConflictResolutionStrategy,
    /// Field-specific strategies
    field_strategies: std::collections::HashMap<String, ConflictResolutionStrategy>,
}

impl ConflictResolver {
    /// Create a new conflict resolver
    pub fn new() -> Self {
        Self {
            default_strategy: ConflictResolutionStrategy::LastWriteWins,
            field_strategies: std::collections::HashMap::new(),
        }
    }

    /// Set default strategy
    pub fn with_default_strategy(mut self, strategy: ConflictResolutionStrategy) -> Self {
        self.default_strategy = strategy;
        self
    }

    /// Set strategy for specific field
    pub fn with_field_strategy(mut self, field: &str, strategy: ConflictResolutionStrategy) -> Self {
        self.field_strategies.insert(field.to_string(), strategy);
        self
    }

    /// Resolve a single conflict
    pub fn resolve(
        &self,
        conflict: &SyncConflict,
        strategy: ConflictResolutionStrategy,
    ) -> ConflictResolution {
        match strategy {
            ConflictResolutionStrategy::LastWriteWins => {
                // For now, assume remote is more recent if different
                // In practice, we'd compare timestamps
                ConflictResolution::choose_remote(conflict)
            }
            ConflictResolutionStrategy::LocalWins => {
                ConflictResolution::choose_local(conflict)
            }
            ConflictResolutionStrategy::RemoteWins => {
                ConflictResolution::choose_remote(conflict)
            }
            ConflictResolutionStrategy::Merge => {
                self.merge_values(conflict)
            }
            ConflictResolutionStrategy::Manual => {
                // Cannot auto-resolve manual conflicts
                // Return local as placeholder
                let mut resolution = ConflictResolution::choose_local(conflict);
                resolution.source = ResolutionSource::Manual;
                resolution.reason = Some("Manual resolution required".to_string());
                resolution
            }
        }
    }

    /// Resolve all conflicts
    pub fn resolve_all(
        &self,
        conflicts: &[SyncConflict],
        strategy: ConflictResolutionStrategy,
    ) -> Vec<ConflictResolution> {
        conflicts
            .iter()
            .map(|c| {
                // Check for field-specific strategy
                let field_strategy = self.field_strategies
                    .get(&c.field_path)
                    .copied()
                    .unwrap_or(strategy);
                self.resolve(c, field_strategy)
            })
            .collect()
    }

    /// Merge values (field-type aware)
    fn merge_values(&self, conflict: &SyncConflict) -> ConflictResolution {
        // Default merge strategy based on field type
        match (&conflict.local_value, &conflict.remote_value) {
            (Some(local), Some(remote)) => {
                // For booleans: OR operation (enable if either enables)
                if let (Some(l), Some(r)) = (local.as_bool(), remote.as_bool()) {
                    return ConflictResolution::merged(
                        conflict,
                        serde_json::Value::Bool(l || r),
                    );
                }

                // For numbers: take max (more accommodating)
                if let (Some(l), Some(r)) = (local.as_f64(), remote.as_f64()) {
                    return ConflictResolution::merged(
                        conflict,
                        serde_json::json!(l.max(r)),
                    );
                }

                // For arrays: union
                if let (Some(l), Some(r)) = (local.as_array(), remote.as_array()) {
                    let mut merged: Vec<serde_json::Value> = l.clone();
                    for item in r {
                        if !merged.contains(item) {
                            merged.push(item.clone());
                        }
                    }
                    return ConflictResolution::merged(
                        conflict,
                        serde_json::Value::Array(merged),
                    );
                }

                // For strings: prefer non-empty, or remote
                if local.as_str().map(|s| s.is_empty()).unwrap_or(true) {
                    return ConflictResolution::choose_remote(conflict);
                }

                // Default: choose remote
                ConflictResolution::choose_remote(conflict)
            }
            (Some(_), None) => ConflictResolution::choose_local(conflict),
            (None, Some(_)) => ConflictResolution::choose_remote(conflict),
            (None, None) => ConflictResolution {
                resolution_id: Uuid::new_v4(),
                conflict_id: conflict.conflict_id,
                field_path: conflict.field_path.clone(),
                chosen_value: None,
                source: ResolutionSource::Merged,
                resolved_at: Utc::now(),
                reason: Some("Both values are null".to_string()),
            },
        }
    }

    /// Suggest resolution based on accessibility principles
    pub fn suggest_accessible_resolution(&self, conflict: &SyncConflict) -> ConflictResolution {
        // For accessibility features, prefer the more accommodating option
        match (&conflict.local_value, &conflict.remote_value) {
            (Some(local), Some(remote)) => {
                // For booleans: enable feature if either enables it
                if let (Some(l), Some(r)) = (local.as_bool(), remote.as_bool()) {
                    let value = l || r;
                    return ConflictResolution::custom(
                        conflict,
                        serde_json::Value::Bool(value),
                        "Accessibility feature enabled (more accommodating option)",
                    );
                }

                // For time multipliers: use higher value
                if conflict.field_path.contains("time_multiplier") {
                    if let (Some(l), Some(r)) = (local.as_f64(), remote.as_f64()) {
                        let value = l.max(r);
                        return ConflictResolution::custom(
                            conflict,
                            serde_json::json!(value),
                            "Higher time multiplier selected (more accommodating)",
                        );
                    }
                }

                // For magnification level: use higher value
                if conflict.field_path.contains("magnification") && conflict.field_path.contains("level") {
                    if let (Some(l), Some(r)) = (local.as_f64(), remote.as_f64()) {
                        let value = l.max(r);
                        return ConflictResolution::custom(
                            conflict,
                            serde_json::json!(value),
                            "Higher magnification level selected (more accommodating)",
                        );
                    }
                }

                // Default: merge
                self.merge_values(conflict)
            }
            (Some(_), None) => ConflictResolution::choose_local(conflict),
            (None, Some(_)) => ConflictResolution::choose_remote(conflict),
            (None, None) => ConflictResolution {
                resolution_id: Uuid::new_v4(),
                conflict_id: conflict.conflict_id,
                field_path: conflict.field_path.clone(),
                chosen_value: None,
                source: ResolutionSource::Merged,
                resolved_at: Utc::now(),
                reason: Some("Both values are null".to_string()),
            },
        }
    }
}

impl Default for ConflictResolver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_conflict() {
        let conflict = SyncConflict::new(
            "display_preferences.screen_reader.enabled",
            Some(serde_json::json!(true)),
            Some(serde_json::json!(false)),
        );

        assert_eq!(conflict.field_path, "display_preferences.screen_reader.enabled");
        assert!(conflict.local_value.is_some());
        assert!(conflict.remote_value.is_some());
    }

    #[test]
    fn test_conflict_description() {
        let conflict = SyncConflict::new(
            "test_field",
            Some(serde_json::json!(1)),
            Some(serde_json::json!(2)),
        );

        let desc = conflict.describe();
        assert!(desc.contains("test_field"));
        assert!(desc.contains("1"));
        assert!(desc.contains("2"));
    }

    #[test]
    fn test_resolve_local_wins() {
        let resolver = ConflictResolver::new();
        let conflict = SyncConflict::new(
            "test",
            Some(serde_json::json!("local")),
            Some(serde_json::json!("remote")),
        );

        let resolution = resolver.resolve(&conflict, ConflictResolutionStrategy::LocalWins);
        assert_eq!(resolution.source, ResolutionSource::Local);
        assert_eq!(resolution.chosen_value, Some(serde_json::json!("local")));
    }

    #[test]
    fn test_resolve_remote_wins() {
        let resolver = ConflictResolver::new();
        let conflict = SyncConflict::new(
            "test",
            Some(serde_json::json!("local")),
            Some(serde_json::json!("remote")),
        );

        let resolution = resolver.resolve(&conflict, ConflictResolutionStrategy::RemoteWins);
        assert_eq!(resolution.source, ResolutionSource::Remote);
        assert_eq!(resolution.chosen_value, Some(serde_json::json!("remote")));
    }

    #[test]
    fn test_merge_booleans() {
        let resolver = ConflictResolver::new();
        let conflict = SyncConflict::new(
            "enabled",
            Some(serde_json::json!(true)),
            Some(serde_json::json!(false)),
        );

        let resolution = resolver.resolve(&conflict, ConflictResolutionStrategy::Merge);
        assert_eq!(resolution.source, ResolutionSource::Merged);
        assert_eq!(resolution.chosen_value, Some(serde_json::json!(true)));
    }

    #[test]
    fn test_merge_numbers() {
        let resolver = ConflictResolver::new();
        let conflict = SyncConflict::new(
            "time_multiplier",
            Some(serde_json::json!(1.5)),
            Some(serde_json::json!(2.0)),
        );

        let resolution = resolver.resolve(&conflict, ConflictResolutionStrategy::Merge);
        assert_eq!(resolution.source, ResolutionSource::Merged);
        assert_eq!(resolution.chosen_value, Some(serde_json::json!(2.0)));
    }

    #[test]
    fn test_accessible_resolution() {
        let resolver = ConflictResolver::new();
        let conflict = SyncConflict::new(
            "screen_reader.enabled",
            Some(serde_json::json!(false)),
            Some(serde_json::json!(true)),
        );

        let resolution = resolver.suggest_accessible_resolution(&conflict);
        // Should prefer enabling the feature
        assert_eq!(resolution.chosen_value, Some(serde_json::json!(true)));
    }

    #[test]
    fn test_field_specific_strategy() {
        let resolver = ConflictResolver::new()
            .with_field_strategy("important_field", ConflictResolutionStrategy::LocalWins);

        let conflict = SyncConflict::new(
            "important_field",
            Some(serde_json::json!("local")),
            Some(serde_json::json!("remote")),
        );

        let resolutions = resolver.resolve_all(&[conflict], ConflictResolutionStrategy::RemoteWins);
        assert_eq!(resolutions.len(), 1);
        assert_eq!(resolutions[0].source, ResolutionSource::Local);
    }
}
