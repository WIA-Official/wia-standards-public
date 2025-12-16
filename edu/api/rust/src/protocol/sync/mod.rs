//! Profile Synchronization Protocol Module
//! 弘益人間 - Sync accessibility profiles across platforms

mod profile_sync;
mod conflict;

pub use profile_sync::{
    ProfileSyncManager, SyncResult, SyncDirection, ProfileVersion,
    SyncConfig, SyncStatus,
};
pub use conflict::{SyncConflict, ConflictResolution, ConflictResolver};
