//! Content node structure

use super::metadata::NodeMetadata;
use super::representations::Representations;
use serde::{Deserialize, Serialize};

/// A content node in the document tree
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentNode {
    /// Unique identifier
    pub id: String,

    /// Five equal representations
    /// ALL ARE EQUAL - NO DEFAULT!
    pub representations: Representations,

    /// Child nodes
    pub children: Vec<ContentNode>,

    /// Node metadata
    pub metadata: NodeMetadata,
}

impl ContentNode {
    /// Create a new content node
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            representations: Representations::default(),
            children: Vec::new(),
            metadata: NodeMetadata::default(),
        }
    }

    /// Add a child node
    pub fn add_child(mut self, child: ContentNode) -> Self {
        self.children.push(child);
        self
    }

    /// Set representations
    pub fn with_representations(mut self, representations: Representations) -> Self {
        self.representations = representations;
        self
    }

    /// Set metadata
    pub fn with_metadata(mut self, metadata: NodeMetadata) -> Self {
        self.metadata = metadata;
        self
    }
}
