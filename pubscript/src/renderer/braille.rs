//! Braille Renderer
//!
//! **Philosophy: Braille is a FIRST-CLASS output format**
//! **Braille is NOT an "accessibility feature" - it is EQUAL to text**
//!
//! Renders WIA PubScript IR to Braille Ready Format (BRF) and plain braille text.

use super::{RenderError, Renderer};
use crate::ir::*;
use crate::SemanticType;

/// Braille renderer
pub struct BrailleRenderer {
    /// Output format
    format: BrailleFormat,
}

/// Braille output format
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BrailleFormat {
    /// Plain braille text
    PlainText,

    /// BRF (Braille Ready Format)
    Brf,
}

impl Default for BrailleRenderer {
    fn default() -> Self {
        Self::new()
    }
}

impl BrailleRenderer {
    /// Create a new braille renderer
    pub fn new() -> Self {
        Self {
            format: BrailleFormat::PlainText,
        }
    }

    /// Set output format
    pub fn with_format(mut self, format: BrailleFormat) -> Self {
        self.format = format;
        self
    }

    /// Render a content node to braille
    fn render_node(&self, node: &ContentNode, depth: usize) -> String {
        let mut output = String::new();

        // Get braille from tactile representation
        if let Some(tactile) = &node.representations.tactile {
            if let Some(braille) = &tactile.braille {
                // Add indentation for nested nodes
                if depth > 0 {
                    output.push_str(&"  ".repeat(depth));
                }

                output.push_str(braille);

                // Add semantic markers
                if let Some(semantic_type) = &node.metadata.semantic_type {
                    match semantic_type {
                        SemanticType::Heading { level } => {
                            // Add heading marker
                            output.push_str(&format!(" [H{}]", level));
                        }
                        _ => {}
                    }
                }

                output.push('\n');
            }
        }

        // Render children
        for child in &node.children {
            output.push_str(&self.render_node(child, depth + 1));
        }

        output
    }
}

impl Renderer for BrailleRenderer {
    fn render(&self, doc: &PubScriptDocument) -> Result<String, RenderError> {
        let mut output = String::new();

        // Add document metadata as braille comment
        if let Some(title) = &doc.metadata.title {
            output.push_str(&format!("; {}\n", title));
            output.push_str(&format!("; Version: {}\n", doc.metadata.version));
            output.push('\n');
        }

        // Render all content nodes
        for node in &doc.content {
            output.push_str(&self.render_node(node, 0));
        }

        // Add BRF footer if needed
        if self.format == BrailleFormat::Brf {
            output.push_str("\n; End of document\n");
        }

        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TactileRep;

    #[test]
    fn test_render_simple_braille() {
        let doc = PubScriptDocument {
            metadata: Metadata {
                title: Some("Test Document".to_string()),
                version: "3.0".to_string(),
                ..Default::default()
            },
            content: vec![ContentNode::new("node-1").with_representations(
                Representations::new().with_tactile(TactileRep::braille("⠓⠑⠇⠇⠕")),
            )],
            timeline: None,
        };

        let renderer = BrailleRenderer::new();
        let output = renderer.render(&doc).unwrap();

        assert!(output.contains("⠓⠑⠇⠇⠕"));
    }

    #[test]
    fn test_render_with_heading() {
        let doc = PubScriptDocument {
            metadata: Metadata {
                title: Some("Test".to_string()),
                version: "3.0".to_string(),
                ..Default::default()
            },
            content: vec![ContentNode::new("node-1")
                .with_representations(
                    Representations::new().with_tactile(TactileRep::braille("⠓⠑⠁⠙⠊⠝⠛")),
                )
                .with_metadata(NodeMetadata {
                    semantic_type: Some(SemanticType::Heading { level: 1 }),
                    ..Default::default()
                })],
            timeline: None,
        };

        let renderer = BrailleRenderer::new();
        let output = renderer.render(&doc).unwrap();

        assert!(output.contains("⠓⠑⠁⠙⠊⠝⠛"));
        assert!(output.contains("[H1]"));
    }
}
