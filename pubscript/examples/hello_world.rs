//! Hello World example for WIA PubScript
//!
//! Demonstrates creating a simple document with all five equal representations.

use wia_pubscript::*;

fn main() {
    println!("🌟 WIA PubScript - Hello World Example\n");

    // Create a simple "Hello, World!" node with all five equal representations
    let hello_node = ContentNode::new("hello-1")
        .with_representations(
            Representations::new()
                // Visual: Text display
                .with_visual(VisualRep::text("Hello, World!").with_style(VisualStyle {
                    font_size: Some(24.0),
                    font_weight: Some(FontWeight::Bold),
                    color: Some(Color::rgb(0, 120, 212)), // Blue
                    ..Default::default()
                }))
                // Auditory: Speech synthesis
                .with_auditory(AuditoryRep::speech("Hello, World!"))
                // Tactile: Braille
                .with_tactile(TactileRep::braille("⠠⠓⠑⠇⠇⠕⠂ ⠠⠺⠕⠗⠇⠙⠖"))
                // Spatial: 3D position
                .with_spatial(SpatialRep::at_position(Vec3::new(0.0, 0.0, 0.0)))
                // Gestural: Tap to interact
                .with_gestural(GesturalRep::with_gesture(Gesture::Tap)),
        )
        .with_metadata(NodeMetadata {
            semantic_type: Some(SemanticType::Paragraph),
            language: Some("en".to_string()),
            ..Default::default()
        });

    // Create a complete document
    let doc = PubScriptDocument {
        metadata: Metadata {
            title: Some("Hello World".to_string()),
            author: Some("WIA Team".to_string()),
            language: Some("en".to_string()),
            version: "3.0".to_string(),
            ..Default::default()
        },
        content: vec![hello_node],
        timeline: None,
    };

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&doc).expect("Failed to serialize");

    println!("Generated IR v3.0 JSON:\n");
    println!("{}", json);

    println!("\n✅ Success! All five representations are equal:");
    println!("   • Visual: ✓");
    println!("   • Auditory: ✓");
    println!("   • Tactile: ✓");
    println!("   • Spatial: ✓");
    println!("   • Gestural: ✓");
    println!("\n🌟 NO DEFAULT EXISTS - ALL ARE EQUAL! 🌟");
}
