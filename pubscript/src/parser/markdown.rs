//! Markdown to IR parser
//!
//! Converts Markdown text to WIA PubScript IR with all five equal representations.

use super::{Parser, ParserError};
use crate::ir::*;
use crate::{
    AuditoryRep, FontWeight, Gesture, GesturalRep, HapticPattern, Representations, SemanticType,
    SpatialRep, TactileRep, Vec3, VisualRep, VisualStyle, VoiceCommand,
};
use pulldown_cmark::{Event, Options, Parser as CmarkParser, Tag, TagEnd};

/// Markdown parser
pub struct MarkdownParser {
    /// Generate all five representations
    generate_all_representations: bool,
}

impl Default for MarkdownParser {
    fn default() -> Self {
        Self::new()
    }
}

impl MarkdownParser {
    /// Create a new Markdown parser
    pub fn new() -> Self {
        Self {
            generate_all_representations: true,
        }
    }

    /// Convert Markdown text to Braille
    /// This is a placeholder - will be replaced with proper implementation
    fn text_to_braille(&self, text: &str) -> String {
        // Import from braille module (to be implemented)
        crate::braille::text_to_braille(text)
    }

    /// Create representations for a text node
    fn create_representations(&self, text: &str) -> Representations {
        if !self.generate_all_representations {
            return Representations::new().with_visual(VisualRep::text(text));
        }

        // ALL FIVE ARE EQUAL!
        Representations::new()
            .with_visual(VisualRep::text(text))
            .with_auditory(AuditoryRep::speech(text))
            .with_tactile(TactileRep::braille(self.text_to_braille(text)))
            .with_spatial(SpatialRep::at_position(Vec3::zero()))
            .with_gestural(GesturalRep::with_gesture(Gesture::Tap))
    }

    /// Create representations for a heading
    fn create_heading_representations(&self, text: &str, level: u8) -> Representations {
        if !self.generate_all_representations {
            return Representations::new().with_visual(
                VisualRep::text(text).with_style(VisualStyle {
                    font_size: Some(32.0 - (level as f32 * 4.0)),
                    font_weight: Some(FontWeight::Bold),
                    ..Default::default()
                }),
            );
        }

        // ALL FIVE ARE EQUAL!
        Representations::new()
            .with_visual(
                VisualRep::text(text).with_style(VisualStyle {
                    font_size: Some(32.0 - (level as f32 * 4.0)),
                    font_weight: Some(FontWeight::Bold),
                    ..Default::default()
                }),
            )
            .with_auditory(
                AuditoryRep::speech(text).with_ssml(format!(
                    r#"<speak><emphasis level="strong">{}</emphasis></speak>"#,
                    text
                )),
            )
            .with_tactile(
                TactileRep::braille(self.text_to_braille(text))
                    .with_haptic_pattern(HapticPattern::Heading),
            )
            .with_spatial(SpatialRep::at_position(Vec3::new(
                0.0,
                (6 - level) as f32 * 2.0,
                0.0,
            )))
            .with_gestural(
                GesturalRep::with_gesture(Gesture::Tap).add_voice_command(
                    VoiceCommand::new("read heading").with_action("read"),
                ),
            )
    }
}

impl Parser for MarkdownParser {
    fn parse(&self, input: &str) -> Result<PubScriptDocument, ParserError> {
        let mut options = Options::empty();
        options.insert(Options::ENABLE_STRIKETHROUGH);
        options.insert(Options::ENABLE_TABLES);
        options.insert(Options::ENABLE_HEADING_ATTRIBUTES);

        let parser = CmarkParser::new_ext(input, options);

        let mut content_nodes = Vec::new();
        let mut current_text = String::new();
        let mut node_counter = 0;
        let mut heading_level = None;

        for event in parser {
            match event {
                Event::Start(Tag::Heading { level, .. }) => {
                    heading_level = Some(level as u8);
                    current_text.clear();
                }
                Event::End(TagEnd::Heading(_)) => {
                    if let Some(level) = heading_level.take() {
                        node_counter += 1;
                        let node = ContentNode::new(format!("node-{}", node_counter))
                            .with_representations(
                                self.create_heading_representations(&current_text, level),
                            )
                            .with_metadata(NodeMetadata {
                                semantic_type: Some(SemanticType::Heading { level }),
                                ..Default::default()
                            });
                        content_nodes.push(node);
                        current_text.clear();
                    }
                }
                Event::Start(Tag::Paragraph) => {
                    current_text.clear();
                }
                Event::End(TagEnd::Paragraph) => {
                    if !current_text.is_empty() {
                        node_counter += 1;
                        let node = ContentNode::new(format!("node-{}", node_counter))
                            .with_representations(self.create_representations(&current_text))
                            .with_metadata(NodeMetadata {
                                semantic_type: Some(SemanticType::Paragraph),
                                ..Default::default()
                            });
                        content_nodes.push(node);
                        current_text.clear();
                    }
                }
                Event::Text(text) => {
                    current_text.push_str(&text);
                }
                Event::Code(code) => {
                    current_text.push_str(&code);
                }
                Event::SoftBreak | Event::HardBreak => {
                    current_text.push(' ');
                }
                _ => {}
            }
        }

        Ok(PubScriptDocument {
            metadata: Metadata {
                title: Some("Markdown Document".to_string()),
                version: "3.0".to_string(),
                ..Default::default()
            },
            content: content_nodes,
            timeline: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_paragraph() {
        let parser = MarkdownParser::new();
        let doc = parser.parse("Hello, World!").unwrap();

        assert_eq!(doc.content.len(), 1);
        assert!(doc.content[0].representations.visual.is_some());
        assert!(doc.content[0].representations.auditory.is_some());
        assert!(doc.content[0].representations.tactile.is_some());
    }

    #[test]
    fn test_parse_heading() {
        let parser = MarkdownParser::new();
        let doc = parser.parse("# Heading 1\n\nParagraph").unwrap();

        assert_eq!(doc.content.len(), 2);

        // Check heading
        if let Some(SemanticType::Heading { level }) = &doc.content[0].metadata.semantic_type {
            assert_eq!(*level, 1);
        } else {
            panic!("Expected heading");
        }

        // Check paragraph
        if let Some(SemanticType::Paragraph) = &doc.content[1].metadata.semantic_type {
            // OK
        } else {
            panic!("Expected paragraph");
        }
    }
}
