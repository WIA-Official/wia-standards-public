//! Node metadata structures

use serde::{Deserialize, Serialize};

/// Metadata for a content node
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NodeMetadata {
    /// Semantic type of the node
    pub semantic_type: Option<SemanticType>,

    /// Language code (e.g., "en", "ko")
    pub language: Option<String>,

    /// Accessibility label
    pub aria_label: Option<String>,

    /// Custom attributes
    pub custom: std::collections::HashMap<String, String>,
}

/// Semantic types for content nodes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SemanticType {
    Heading { level: u8 },
    Paragraph,
    List,
    ListItem,
    Image,
    Link,
    Code,
    Quote,
    Table,
    Custom(String),
}
