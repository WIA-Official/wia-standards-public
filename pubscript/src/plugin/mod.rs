//! Plugin system for extensible parsers and renderers
//!
//! Enables custom parsers and renderers to be registered and used
//! without modifying the core WIA PubScript code.
//!
//! ## Philosophy
//!
//! - **Extensibility**: Anyone can add support for new formats
//! - **No Core Changes**: Plugins work without modifying core code
//! - **Equal Status**: Built-in and plugin formats are equal
//!
//! ## Example
//!
//! ```rust
//! use wia_pubscript::plugin::{PluginRegistry, ParserPlugin, RendererPlugin};
//! use wia_pubscript::ir::PubScriptDocument;
//!
//! // Create registry
//! let mut registry = PluginRegistry::new();
//!
//! // Register custom parser
//! // registry.register_parser("docx", Box::new(DocxParser::new()));
//!
//! // Register custom renderer
//! // registry.register_renderer("epub", Box::new(EpubRenderer::new()));
//!
//! // Use registered plugins
//! // let doc = registry.parse("docx", input)?;
//! // let output = registry.render("epub", &doc)?;
//! ```

use crate::ir::PubScriptDocument;
use std::collections::HashMap;
use std::error::Error as StdError;
use std::fmt;

/// Error type for plugin operations
#[derive(Debug, Clone)]
pub enum PluginError {
    /// Plugin not found
    NotFound(String),
    /// Parse error
    ParseError(String),
    /// Render error
    RenderError(String),
    /// Invalid plugin
    InvalidPlugin(String),
}

impl fmt::Display for PluginError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PluginError::NotFound(name) => write!(f, "Plugin not found: {}", name),
            PluginError::ParseError(msg) => write!(f, "Parse error: {}", msg),
            PluginError::RenderError(msg) => write!(f, "Render error: {}", msg),
            PluginError::InvalidPlugin(msg) => write!(f, "Invalid plugin: {}", msg),
        }
    }
}

impl StdError for PluginError {}

/// Parser plugin trait
///
/// Implement this trait to create custom parsers for new input formats.
pub trait ParserPlugin: Send + Sync {
    /// Get plugin name (e.g., "docx", "latex", "org-mode")
    fn name(&self) -> &str;

    /// Get supported file extensions
    fn extensions(&self) -> Vec<&str>;

    /// Parse input into IR document
    fn parse(&self, input: &str) -> Result<PubScriptDocument, PluginError>;

    /// Get plugin metadata
    fn metadata(&self) -> PluginMetadata {
        PluginMetadata {
            name: self.name().to_string(),
            version: "1.0.0".to_string(),
            description: format!("{} parser plugin", self.name()),
            author: "Unknown".to_string(),
        }
    }
}

/// Renderer plugin trait
///
/// Implement this trait to create custom renderers for new output formats.
pub trait RendererPlugin: Send + Sync {
    /// Get plugin name (e.g., "epub", "pdf", "docx")
    fn name(&self) -> &str;

    /// Get supported file extensions
    fn extensions(&self) -> Vec<&str>;

    /// Render IR document to output format
    fn render(&self, doc: &PubScriptDocument) -> Result<String, PluginError>;

    /// Get plugin metadata
    fn metadata(&self) -> PluginMetadata {
        PluginMetadata {
            name: self.name().to_string(),
            version: "1.0.0".to_string(),
            description: format!("{} renderer plugin", self.name()),
            author: "Unknown".to_string(),
        }
    }
}

/// Plugin metadata
#[derive(Debug, Clone)]
pub struct PluginMetadata {
    /// Plugin name
    pub name: String,
    /// Plugin version
    pub version: String,
    /// Plugin description
    pub description: String,
    /// Plugin author
    pub author: String,
}

/// Plugin registry
///
/// Manages registered parsers and renderers.
pub struct PluginRegistry {
    /// Registered parser plugins
    parsers: HashMap<String, Box<dyn ParserPlugin>>,
    /// Registered renderer plugins
    renderers: HashMap<String, Box<dyn RendererPlugin>>,
}

impl Default for PluginRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl PluginRegistry {
    /// Create a new plugin registry
    pub fn new() -> Self {
        Self {
            parsers: HashMap::new(),
            renderers: HashMap::new(),
        }
    }

    /// Register a parser plugin
    pub fn register_parser(&mut self, plugin: Box<dyn ParserPlugin>) {
        let name = plugin.name().to_string();
        self.parsers.insert(name, plugin);
    }

    /// Register a renderer plugin
    pub fn register_renderer(&mut self, plugin: Box<dyn RendererPlugin>) {
        let name = plugin.name().to_string();
        self.renderers.insert(name, plugin);
    }

    /// Get parser by name
    pub fn get_parser(&self, name: &str) -> Option<&dyn ParserPlugin> {
        self.parsers.get(name).map(|p| p.as_ref())
    }

    /// Get renderer by name
    pub fn get_renderer(&self, name: &str) -> Option<&dyn RendererPlugin> {
        self.renderers.get(name).map(|r| r.as_ref())
    }

    /// Parse input using named parser
    pub fn parse(&self, parser_name: &str, input: &str) -> Result<PubScriptDocument, PluginError> {
        let parser = self
            .get_parser(parser_name)
            .ok_or_else(|| PluginError::NotFound(parser_name.to_string()))?;

        parser.parse(input)
    }

    /// Render document using named renderer
    pub fn render(&self, renderer_name: &str, doc: &PubScriptDocument) -> Result<String, PluginError> {
        let renderer = self
            .get_renderer(renderer_name)
            .ok_or_else(|| PluginError::NotFound(renderer_name.to_string()))?;

        renderer.render(doc)
    }

    /// List all registered parser names
    pub fn list_parsers(&self) -> Vec<String> {
        self.parsers.keys().cloned().collect()
    }

    /// List all registered renderer names
    pub fn list_renderers(&self) -> Vec<String> {
        self.renderers.keys().cloned().collect()
    }

    /// Get parser metadata
    pub fn get_parser_metadata(&self, name: &str) -> Option<PluginMetadata> {
        self.get_parser(name).map(|p| p.metadata())
    }

    /// Get renderer metadata
    pub fn get_renderer_metadata(&self, name: &str) -> Option<PluginMetadata> {
        self.get_renderer(name).map(|r| r.metadata())
    }

    /// Get all plugin metadata
    pub fn list_all_plugins(&self) -> Vec<(String, PluginMetadata)> {
        let mut plugins = Vec::new();

        for name in self.list_parsers() {
            if let Some(meta) = self.get_parser_metadata(&name) {
                plugins.push((format!("parser:{}", name), meta));
            }
        }

        for name in self.list_renderers() {
            if let Some(meta) = self.get_renderer_metadata(&name) {
                plugins.push((format!("renderer:{}", name), meta));
            }
        }

        plugins
    }
}

// Example built-in plugins (demonstrating the system)

/// Example JSON parser plugin
pub struct JsonParserPlugin;

impl ParserPlugin for JsonParserPlugin {
    fn name(&self) -> &str {
        "json"
    }

    fn extensions(&self) -> Vec<&str> {
        vec!["json"]
    }

    fn parse(&self, input: &str) -> Result<PubScriptDocument, PluginError> {
        // Parse JSON and convert to IR
        serde_json::from_str(input)
            .map_err(|e| PluginError::ParseError(format!("JSON parse error: {}", e)))
    }

    fn metadata(&self) -> PluginMetadata {
        PluginMetadata {
            name: "json".to_string(),
            version: "1.0.0".to_string(),
            description: "JSON IR parser (built-in)".to_string(),
            author: "WIA PubScript Team".to_string(),
        }
    }
}

/// Example JSON renderer plugin
pub struct JsonRendererPlugin;

impl RendererPlugin for JsonRendererPlugin {
    fn name(&self) -> &str {
        "json"
    }

    fn extensions(&self) -> Vec<&str> {
        vec!["json"]
    }

    fn render(&self, doc: &PubScriptDocument) -> Result<String, PluginError> {
        // Render IR to JSON
        serde_json::to_string_pretty(doc)
            .map_err(|e| PluginError::RenderError(format!("JSON render error: {}", e)))
    }

    fn metadata(&self) -> PluginMetadata {
        PluginMetadata {
            name: "json".to_string(),
            version: "1.0.0".to_string(),
            description: "JSON IR renderer (built-in)".to_string(),
            author: "WIA PubScript Team".to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plugin_registry_create() {
        let registry = PluginRegistry::new();
        assert_eq!(registry.list_parsers().len(), 0);
        assert_eq!(registry.list_renderers().len(), 0);
    }

    #[test]
    fn test_register_parser() {
        let mut registry = PluginRegistry::new();
        registry.register_parser(Box::new(JsonParserPlugin));

        assert_eq!(registry.list_parsers().len(), 1);
        assert!(registry.get_parser("json").is_some());
    }

    #[test]
    fn test_register_renderer() {
        let mut registry = PluginRegistry::new();
        registry.register_renderer(Box::new(JsonRendererPlugin));

        assert_eq!(registry.list_renderers().len(), 1);
        assert!(registry.get_renderer("json").is_some());
    }

    #[test]
    fn test_plugin_metadata() {
        let parser = JsonParserPlugin;
        let meta = parser.metadata();

        assert_eq!(meta.name, "json");
        assert_eq!(meta.version, "1.0.0");
        assert!(meta.description.contains("JSON"));
    }

    #[test]
    fn test_list_all_plugins() {
        let mut registry = PluginRegistry::new();
        registry.register_parser(Box::new(JsonParserPlugin));
        registry.register_renderer(Box::new(JsonRendererPlugin));

        let plugins = registry.list_all_plugins();
        assert_eq!(plugins.len(), 2);
    }
}
