//! Document-level structures

use super::node::ContentNode;
use serde::{Deserialize, Serialize};

/// A complete PubScript document
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PubScriptDocument {
    /// Document metadata
    pub metadata: Metadata,

    /// Content node tree
    pub content: Vec<ContentNode>,

    /// Optional timeline for synchronization
    pub timeline: Option<Timeline>,
}

/// Document metadata
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Metadata {
    pub title: Option<String>,
    pub author: Option<String>,
    pub language: Option<String>,

    /// IR version (e.g., "3.0")
    pub version: String,

    pub created_at: Option<String>,
    pub updated_at: Option<String>,
}

/// Timeline for time-based synchronization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timeline {
    /// Duration in seconds
    pub duration: f64,

    /// Timeline events
    pub events: Vec<TimelineEvent>,
}

/// A single timeline event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimelineEvent {
    /// Timestamp in seconds
    pub timestamp: f64,

    /// Target node ID
    pub node_id: String,

    /// Event type
    pub event_type: TimelineEventType,
}

/// Types of timeline events
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TimelineEventType {
    Show,
    Hide,
    Play,
    Pause,
    Sync,
}
