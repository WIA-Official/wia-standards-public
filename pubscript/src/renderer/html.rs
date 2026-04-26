//! HTML Renderer
//!
//! **Philosophy: HTML is a FIRST-CLASS output format**
//! **Semantic HTML + ARIA is EQUAL to all other representations**
//!
//! Renders WIA PubScript IR to accessible HTML with full ARIA support.

use super::{RenderError, Renderer};
use crate::ir::*;
use crate::SemanticType;

/// HTML renderer
pub struct HtmlRenderer {
    /// Include ARIA attributes
    include_aria: bool,

    /// Include CSS classes
    include_classes: bool,
}

impl Default for HtmlRenderer {
    fn default() -> Self {
        Self::new()
    }
}

impl HtmlRenderer {
    /// Create a new HTML renderer
    pub fn new() -> Self {
        Self {
            include_aria: true,
            include_classes: true,
        }
    }

    /// Set whether to include ARIA attributes
    pub fn with_aria(mut self, include: bool) -> Self {
        self.include_aria = include;
        self
    }

    /// Set whether to include CSS classes
    pub fn with_classes(mut self, include: bool) -> Self {
        self.include_classes = include;
        self
    }

    /// Render a content node to HTML
    fn render_node(&self, node: &ContentNode, depth: usize) -> String {
        let mut output = String::new();

        // Get visual representation
        if let Some(visual) = &node.representations.visual {
            // Determine HTML tag based on semantic type
            let (open_tag, close_tag) = match &node.metadata.semantic_type {
                Some(SemanticType::Heading { level }) => {
                    let tag = format!("h{}", level);
                    let mut open = format!("<{}", tag);

                    // Add ARIA attributes
                    if self.include_aria {
                        open.push_str(&format!(" role=\"heading\" aria-level=\"{}\"", level));
                    }

                    // Add CSS classes
                    if self.include_classes {
                        open.push_str(&format!(" class=\"pubscript-heading heading-{}\"", level));
                    }

                    open.push('>');
                    (open, format!("</{}>", tag))
                }
                Some(SemanticType::Paragraph) => {
                    let mut open = String::from("<p");

                    if self.include_classes {
                        open.push_str(" class=\"pubscript-paragraph\"");
                    }

                    open.push('>');
                    ("<p>".to_string(), "</p>".to_string())
                }
                Some(SemanticType::Link) => {
                    let mut open = String::from("<a");

                    if self.include_aria {
                        open.push_str(" role=\"link\"");
                    }

                    open.push('>');
                    (open, "</a>".to_string())
                }
                Some(SemanticType::Code) => {
                    ("<code>".to_string(), "</code>".to_string())
                }
                _ => ("<div>".to_string(), "</div>".to_string()),
            };

            // Add indentation
            if depth > 0 {
                output.push_str(&"  ".repeat(depth));
            }

            output.push_str(&open_tag);

            // Add text content
            if let Some(text) = &visual.text {
                output.push_str(&html_escape(text));
            }

            // Render children inline if they exist
            if !node.children.is_empty() {
                output.push('\n');
                for child in &node.children {
                    output.push_str(&self.render_node(child, depth + 1));
                }
                if depth > 0 {
                    output.push_str(&"  ".repeat(depth));
                }
            }

            output.push_str(&close_tag);
            output.push('\n');
        }

        output
    }
}

impl Renderer for HtmlRenderer {
    fn render(&self, doc: &PubScriptDocument) -> Result<String, RenderError> {
        let mut output = String::new();

        // HTML5 header
        output.push_str("<!DOCTYPE html>\n");
        output.push_str("<html");

        // Language
        if let Some(lang) = &doc.metadata.language {
            output.push_str(&format!(" lang=\"{}\"", lang));
        } else {
            output.push_str(" lang=\"en\"");
        }

        output.push_str(">\n");
        output.push_str("<head>\n");
        output.push_str("  <meta charset=\"UTF-8\">\n");
        output.push_str("  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n");

        // Title
        if let Some(title) = &doc.metadata.title {
            output.push_str(&format!("  <title>{}</title>\n", html_escape(title)));
        }

        // Meta generator
        output.push_str(&format!(
            "  <meta name=\"generator\" content=\"WIA PubScript v{}\">\n",
            doc.metadata.version
        ));

        // Basic CSS for accessibility
        output.push_str("  <style>\n");
        output.push_str("    body {\n");
        output.push_str("      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;\n");
        output.push_str("      line-height: 1.6;\n");
        output.push_str("      max-width: 800px;\n");
        output.push_str("      margin: 0 auto;\n");
        output.push_str("      padding: 2rem;\n");
        output.push_str("    }\n");
        output.push_str("    .pubscript-heading { margin-top: 1.5em; }\n");
        output.push_str("    .pubscript-paragraph { margin: 1em 0; }\n");
        output.push_str("  </style>\n");
        output.push_str("</head>\n");
        output.push_str("<body>\n");

        // Main content with ARIA landmark
        if self.include_aria {
            output.push_str("  <main role=\"main\" aria-label=\"Document content\">\n");
        } else {
            output.push_str("  <main>\n");
        }

        // Render all content nodes
        for node in &doc.content {
            output.push_str("    ");
            output.push_str(&self.render_node(node, 2));
        }

        output.push_str("  </main>\n");
        output.push_str("</body>\n");
        output.push_str("</html>\n");

        Ok(output)
    }
}

/// Escape HTML special characters
fn html_escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&#39;")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Metadata, NodeMetadata, PubScriptDocument, Representations, VisualRep};

    #[test]
    fn test_render_simple_html() {
        let doc = PubScriptDocument {
            metadata: Metadata {
                title: Some("Test Document".to_string()),
                version: "3.0".to_string(),
                language: Some("en".to_string()),
                ..Default::default()
            },
            content: vec![ContentNode::new("node-1").with_representations(
                Representations::new().with_visual(VisualRep::text("Hello, World!")),
            )],
            timeline: None,
        };

        let renderer = HtmlRenderer::new();
        let output = renderer.render(&doc).unwrap();

        assert!(output.contains("<!DOCTYPE html>"));
        assert!(output.contains("<html lang=\"en\">"));
        assert!(output.contains("<title>Test Document</title>"));
        assert!(output.contains("Hello, World!"));
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
                    Representations::new().with_visual(VisualRep::text("Heading Text")),
                )
                .with_metadata(NodeMetadata {
                    semantic_type: Some(SemanticType::Heading { level: 1 }),
                    ..Default::default()
                })],
            timeline: None,
        };

        let renderer = HtmlRenderer::new();
        let output = renderer.render(&doc).unwrap();

        assert!(output.contains("<h1"));
        assert!(output.contains("role=\"heading\""));
        assert!(output.contains("aria-level=\"1\""));
        assert!(output.contains("Heading Text"));
    }

    #[test]
    fn test_html_escape() {
        let escaped = html_escape("<script>alert('xss')</script>");
        assert_eq!(
            escaped,
            "&lt;script&gt;alert(&#39;xss&#39;)&lt;/script&gt;"
        );
    }
}
