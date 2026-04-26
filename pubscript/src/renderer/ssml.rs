//! SSML Renderer
//!
//! **Philosophy: Text-to-Speech is a FIRST-CLASS output format**
//! **TTS is NOT an "accessibility feature" - it is EQUAL to text**
//!
//! Renders WIA PubScript IR to SSML (Speech Synthesis Markup Language).

use super::{RenderError, Renderer};
use crate::ir::*;
use crate::SemanticType;

/// SSML renderer
pub struct SsmlRenderer {
    /// SSML version
    version: SsmlVersion,
}

/// SSML version
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SsmlVersion {
    /// SSML 1.0
    V1_0,

    /// SSML 1.1
    V1_1,
}

impl Default for SsmlRenderer {
    fn default() -> Self {
        Self::new()
    }
}

impl SsmlRenderer {
    /// Create a new SSML renderer
    pub fn new() -> Self {
        Self {
            version: SsmlVersion::V1_1,
        }
    }

    /// Set SSML version
    pub fn with_version(mut self, version: SsmlVersion) -> Self {
        self.version = version;
        self
    }

    /// Render a content node to SSML
    fn render_node(&self, node: &ContentNode) -> String {
        let mut output = String::new();

        // Get audio from auditory representation
        if let Some(auditory) = &node.representations.auditory {
            // Check if custom SSML is provided
            if let Some(ssml) = &auditory.ssml {
                output.push_str(ssml);
            } else if let Some(text) = &auditory.speech_text {
                // Generate SSML based on semantic type
                if let Some(semantic_type) = &node.metadata.semantic_type {
                    match semantic_type {
                        SemanticType::Heading { level } => {
                            // Emphasize headings
                            let emphasis_level = match level {
                                1 => "strong",
                                2 => "moderate",
                                _ => "reduced",
                            };
                            output.push_str(&format!(
                                "<emphasis level=\"{}\">{}</emphasis><break time=\"500ms\"/>",
                                emphasis_level, text
                            ));
                        }
                        SemanticType::Paragraph => {
                            output.push_str(text);
                            output.push_str("<break time=\"300ms\"/>");
                        }
                        _ => {
                            output.push_str(text);
                        }
                    }
                } else {
                    output.push_str(text);
                }
            }

            // Add sound effects
            for effect in &auditory.sound_effects {
                output.push_str(&format!(
                    "<audio src=\"{:?}\"/>",
                    effect.effect_type
                ));
            }
        }

        // Render children
        for child in &node.children {
            output.push_str(&self.render_node(child));
        }

        output
    }
}

impl Renderer for SsmlRenderer {
    fn render(&self, doc: &PubScriptDocument) -> Result<String, RenderError> {
        let mut output = String::new();

        // SSML header
        output.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        output.push_str("<speak version=\"1.1\" xmlns=\"http://www.w3.org/2001/10/synthesis\" ");
        output.push_str("xml:lang=\"");

        // Language from metadata
        if let Some(lang) = &doc.metadata.language {
            output.push_str(lang);
        } else {
            output.push_str("en-US");
        }

        output.push_str("\">\n");

        // Document title as prosody
        if let Some(title) = &doc.metadata.title {
            output.push_str(&format!(
                "  <prosody rate=\"slow\" volume=\"loud\">{}</prosody>\n",
                title
            ));
            output.push_str("  <break time=\"1s\"/>\n");
        }

        // Render all content nodes
        for node in &doc.content {
            output.push_str("  ");
            output.push_str(&self.render_node(node));
            output.push('\n');
        }

        // SSML footer
        output.push_str("</speak>\n");

        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AuditoryRep;

    #[test]
    fn test_render_simple_ssml() {
        let doc = PubScriptDocument {
            metadata: Metadata {
                title: Some("Test Document".to_string()),
                version: "3.0".to_string(),
                language: Some("en-US".to_string()),
                ..Default::default()
            },
            content: vec![ContentNode::new("node-1").with_representations(
                Representations::new().with_auditory(AuditoryRep::speech("Hello, World!")),
            )],
            timeline: None,
        };

        let renderer = SsmlRenderer::new();
        let output = renderer.render(&doc).unwrap();

        assert!(output.contains("<?xml"));
        assert!(output.contains("<speak"));
        assert!(output.contains("Hello, World!"));
        assert!(output.contains("</speak>"));
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
                    Representations::new()
                        .with_auditory(AuditoryRep::speech("Heading Text")),
                )
                .with_metadata(NodeMetadata {
                    semantic_type: Some(SemanticType::Heading { level: 1 }),
                    ..Default::default()
                })],
            timeline: None,
        };

        let renderer = SsmlRenderer::new();
        let output = renderer.render(&doc).unwrap();

        assert!(output.contains("<emphasis"));
        assert!(output.contains("Heading Text"));
    }
}
