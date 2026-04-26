//! Intermediate Representation (IR) v3.0
//!
//! The core data structures that represent multi-sensory content.

pub mod document;
pub mod metadata;
pub mod node;
pub mod representations;

pub use document::{Metadata, PubScriptDocument, Timeline, TimelineEvent, TimelineEventType};
pub use metadata::NodeMetadata;
pub use node::ContentNode;
pub use representations::Representations;
