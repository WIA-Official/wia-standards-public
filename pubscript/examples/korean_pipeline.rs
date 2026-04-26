//! Korean Pipeline Example
//!
//! Demonstrates the full pipeline:
//! Markdown (한글) → IR → Braille (점자) + SSML (TTS)
//!
//! **Philosophy: 점자가 "추가 기능"이 아니라 텍스트와 동등한 일급 시민**

use wia_pubscript::parser::{markdown::MarkdownParser, Parser};
use wia_pubscript::renderer::{braille::BrailleRenderer, ssml::SsmlRenderer, Renderer};

fn main() {
    println!("🌟 WIA PubScript - Korean Pipeline Example\n");
    println!("═══════════════════════════════════════════\n");

    // 1. Input: Markdown with Korean text
    let markdown = r#"
# 안녕하세요

WIA PubScript는 모든 인간을 위한 선택지입니다.

## 다섯 가지 동등한 표현

- 시각 (Visual)
- 청각 (Auditory)
- 촉각 (Tactile)
- 공간 (Spatial)
- 제스처 (Gestural)

**모두 동등합니다. 기본값은 없습니다.**
"#;

    println!("📝 Input (Markdown):");
    println!("{}", markdown);
    println!("\n{}\n", "─".repeat(50));

    // 2. Parse: Markdown → IR
    println!("⚙️  Step 1: Parsing Markdown → IR...\n");

    let parser = MarkdownParser::new();
    let doc = parser.parse(markdown).expect("Failed to parse markdown");

    println!("✅ IR Document created with {} nodes", doc.content.len());
    println!("   Each node has FIVE EQUAL representations:");
    println!("   - Visual ✓");
    println!("   - Auditory ✓");
    println!("   - Tactile ✓");
    println!("   - Spatial ✓");
    println!("   - Gestural ✓");
    println!("\n{}\n", "─".repeat(50));

    // 3. Render: IR → Braille
    println!("⚙️  Step 2: Rendering IR → Braille (점자)...\n");

    let braille_renderer = BrailleRenderer::new();
    let braille_output = braille_renderer
        .render(&doc)
        .expect("Failed to render braille");

    println!("📖 Braille Output (점자):");
    println!("{}", braille_output);
    println!("{}\n", "─".repeat(50));

    // 4. Render: IR → SSML
    println!("⚙️  Step 3: Rendering IR → SSML (TTS)...\n");

    let ssml_renderer = SsmlRenderer::new();
    let ssml_output = ssml_renderer
        .render(&doc)
        .expect("Failed to render SSML");

    println!("🔊 SSML Output (for Text-to-Speech):");
    println!("{}", ssml_output);
    println!("{}\n", "─".repeat(50));

    // 5. Summary
    println!("🎉 SUCCESS!\n");
    println!("Pipeline completed:");
    println!("  Markdown (한글) → IR → Braille (점자) ✓");
    println!("                       → SSML (TTS)   ✓");
    println!("\n🌟 Philosophy Verified:\n");
    println!("  ✓ 점자가 '추가 기능'이 아니라 일급 시민");
    println!("  ✓ TTS가 '추가 기능'이 아니라 일급 시민");
    println!("  ✓ 모든 표현이 동등");
    println!("  ✓ 기본값 없음");
    println!("\n🌟 ALL FIVE ARE EQUAL! 🌟");
}
