//! WebAssembly API
//!
//! **Philosophy: WASM brings multi-sensory publishing to the browser!**
//!
//! This module provides JavaScript bindings for WIA PubScript,
//! allowing web applications to generate Braille, SSML, and HTML
//! directly in the browser.

use wasm_bindgen::prelude::*;

use crate::braille::text_to_braille;
use crate::parser::{markdown::MarkdownParser, Parser};
use crate::renderer::{braille::BrailleRenderer, html::HtmlRenderer, ssml::SsmlRenderer, Renderer};

/// Initialize WASM module
#[wasm_bindgen(start)]
pub fn init() {
    // Set panic hook for better error messages
    #[cfg(feature = "console_error_panic_hook")]
    console_error_panic_hook::set_once();
}

/// Convert text to Braille
///
/// # Example (JavaScript)
/// ```js
/// import { textToBraille } from 'wia_pubscript';
/// const braille = textToBraille("안녕하세요");
/// console.log(braille); // "⠅⠣⠉⠉⠱⠎⠅⠟⠣⠎⠝⠅⠍"
/// ```
#[wasm_bindgen(js_name = textToBraille)]
pub fn text_to_braille_wasm(text: &str) -> String {
    text_to_braille(text)
}

/// Convert Markdown to IR (JSON)
///
/// # Example (JavaScript)
/// ```js
/// import { markdownToIr } from 'wia_pubscript';
/// const ir = markdownToIr("# Hello\n\nWorld!");
/// const doc = JSON.parse(ir);
/// ```
#[wasm_bindgen(js_name = markdownToIr)]
pub fn markdown_to_ir_wasm(markdown: &str) -> Result<String, JsValue> {
    let parser = MarkdownParser::new();
    let doc = parser
        .parse(markdown)
        .map_err(|e| JsValue::from_str(&format!("Parse error: {}", e)))?;

    serde_json::to_string_pretty(&doc)
        .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
}

/// Convert Markdown to Braille
///
/// # Example (JavaScript)
/// ```js
/// import { markdownToBraille } from 'wia_pubscript';
/// const braille = markdownToBraille("# 안녕하세요\n\n점자 변환!");
/// ```
#[wasm_bindgen(js_name = markdownToBraille)]
pub fn markdown_to_braille_wasm(markdown: &str) -> Result<String, JsValue> {
    let parser = MarkdownParser::new();
    let doc = parser
        .parse(markdown)
        .map_err(|e| JsValue::from_str(&format!("Parse error: {}", e)))?;

    let renderer = BrailleRenderer::new();
    renderer
        .render(&doc)
        .map_err(|e| JsValue::from_str(&format!("Render error: {}", e)))
}

/// Convert Markdown to HTML with ARIA
///
/// # Example (JavaScript)
/// ```js
/// import { markdownToHtml } from 'wia_pubscript';
/// const html = markdownToHtml("# Hello\n\nAccessible HTML!");
/// ```
#[wasm_bindgen(js_name = markdownToHtml)]
pub fn markdown_to_html_wasm(markdown: &str) -> Result<String, JsValue> {
    let parser = MarkdownParser::new();
    let doc = parser
        .parse(markdown)
        .map_err(|e| JsValue::from_str(&format!("Parse error: {}", e)))?;

    let renderer = HtmlRenderer::new();
    renderer
        .render(&doc)
        .map_err(|e| JsValue::from_str(&format!("Render error: {}", e)))
}

/// Convert Markdown to SSML (for TTS)
///
/// # Example (JavaScript)
/// ```js
/// import { markdownToSsml } from 'wia_pubscript';
/// const ssml = markdownToSsml("# Welcome\n\nText-to-speech ready!");
/// ```
#[wasm_bindgen(js_name = markdownToSsml)]
pub fn markdown_to_ssml_wasm(markdown: &str) -> Result<String, JsValue> {
    let parser = MarkdownParser::new();
    let doc = parser
        .parse(markdown)
        .map_err(|e| JsValue::from_str(&format!("Parse error: {}", e)))?;

    let renderer = SsmlRenderer::new();
    renderer
        .render(&doc)
        .map_err(|e| JsValue::from_str(&format!("Render error: {}", e)))
}

/// Convert Markdown to all formats at once
///
/// Returns a JSON object with `{ braille, html, ssml }`
///
/// # Example (JavaScript)
/// ```js
/// import { markdownToAll } from 'wia_pubscript';
/// const result = JSON.parse(markdownToAll("# Hello"));
/// console.log(result.braille);
/// console.log(result.html);
/// console.log(result.ssml);
/// ```
#[wasm_bindgen(js_name = markdownToAll)]
pub fn markdown_to_all_wasm(markdown: &str) -> Result<String, JsValue> {
    let parser = MarkdownParser::new();
    let doc = parser
        .parse(markdown)
        .map_err(|e| JsValue::from_str(&format!("Parse error: {}", e)))?;

    let braille_renderer = BrailleRenderer::new();
    let braille = braille_renderer
        .render(&doc)
        .map_err(|e| JsValue::from_str(&format!("Braille render error: {}", e)))?;

    let html_renderer = HtmlRenderer::new();
    let html = html_renderer
        .render(&doc)
        .map_err(|e| JsValue::from_str(&format!("HTML render error: {}", e)))?;

    let ssml_renderer = SsmlRenderer::new();
    let ssml = ssml_renderer
        .render(&doc)
        .map_err(|e| JsValue::from_str(&format!("SSML render error: {}", e)))?;

    let result = serde_json::json!({
        "braille": braille,
        "html": html,
        "ssml": ssml,
    });

    serde_json::to_string(&result)
        .map_err(|e| JsValue::from_str(&format!("Serialization error: {}", e)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wasm_text_to_braille() {
        let result = text_to_braille_wasm("hello");
        assert_eq!(result, "⠓⠑⠇⠇⠕");
    }

    #[test]
    fn test_wasm_korean_braille() {
        let result = text_to_braille_wasm("안녕");
        assert!(!result.is_empty());
    }
}
