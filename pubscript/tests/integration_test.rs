//! Integration tests for WIA PubScript

use wia_pubscript::*;

#[test]
fn test_simple_text_to_ir() {
    let text = "Hello, World!";

    // Create a document with all five equal representations
    let doc = PubScriptDocument {
        metadata: Metadata {
            title: Some("Simple Document".to_string()),
            version: "3.0".to_string(),
            ..Default::default()
        },
        content: vec![ContentNode {
            id: "node-1".to_string(),
            representations: Representations::new()
                .with_visual(VisualRep::text(text))
                .with_auditory(AuditoryRep::speech(text))
                .with_tactile(TactileRep::braille("⠠⠓⠑⠇⠇⠕⠂ ⠠⠺⠕⠗⠇⠙⠖"))
                .with_spatial(SpatialRep::at_position(Vec3::new(0.0, 0.0, 0.0)))
                .with_gestural(GesturalRep::with_gesture(Gesture::Tap)),
            children: vec![],
            metadata: NodeMetadata::default(),
        }],
        timeline: None,
    };

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&doc).unwrap();
    println!("\n{}", json);

    // Verify JSON is valid
    assert!(json.contains("Hello, World!"));
    assert!(json.contains("⠠⠓⠑⠇⠇⠕⠂ ⠠⠺⠕⠗⠇⠙⠖"));

    // Deserialize back
    let parsed: PubScriptDocument = serde_json::from_str(&json).unwrap();
    assert_eq!(parsed.metadata.version, "3.0");
    assert_eq!(parsed.content.len(), 1);

    // Verify all five representations exist
    let node = &parsed.content[0];
    assert!(node.representations.visual.is_some());
    assert!(node.representations.auditory.is_some());
    assert!(node.representations.tactile.is_some());
    assert!(node.representations.spatial.is_some());
    assert!(node.representations.gestural.is_some());
}

#[test]
fn test_heading_with_all_representations() {
    // Create a heading node with all five equal representations
    let heading = ContentNode::new("heading-1")
        .with_representations(
            Representations::new()
                .with_visual(
                    VisualRep::text("Chapter 1: Introduction").with_style(VisualStyle {
                        font_size: Some(32.0),
                        font_weight: Some(FontWeight::Bold),
                        ..Default::default()
                    }),
                )
                .with_auditory(
                    AuditoryRep::speech("Chapter 1").with_ssml(
                        r#"<speak>
                            <emphasis level="strong">Chapter 1</emphasis>
                            <break time="500ms"/>
                            Introduction
                        </speak>"#,
                    ),
                )
                .with_tactile(
                    TactileRep::braille("⠠⠉⠓⠁⠏⠞⠑⠗ ⠼⠁⠒ ⠠⠊⠝⠞⠗⠕⠙⠥⠉⠞⠊⠕⠝")
                        .with_haptic_pattern(HapticPattern::Heading),
                )
                .with_spatial(
                    SpatialRep::at_position(Vec3::new(0.0, 10.0, 0.0))
                        .with_scale(Vec3::new(2.0, 2.0, 1.0)),
                )
                .with_gestural(
                    GesturalRep::with_gesture(Gesture::Tap).add_voice_command(
                        VoiceCommand::new("read heading").with_action("read"),
                    ),
                ),
        )
        .with_metadata(NodeMetadata {
            semantic_type: Some(SemanticType::Heading { level: 1 }),
            ..Default::default()
        });

    // Serialize and check
    let json = serde_json::to_string_pretty(&heading).unwrap();
    println!("\n{}", json);

    // All five representations should be present
    assert!(json.contains("visual"));
    assert!(json.contains("auditory"));
    assert!(json.contains("tactile"));
    assert!(json.contains("spatial"));
    assert!(json.contains("gestural"));

    // Deserialize back
    let parsed: ContentNode = serde_json::from_str(&json).unwrap();
    assert_eq!(parsed.id, "heading-1");

    // Verify semantic type
    match parsed.metadata.semantic_type {
        Some(SemanticType::Heading { level }) => assert_eq!(level, 1),
        _ => panic!("Expected Heading semantic type"),
    }
}

#[test]
fn test_no_default_representation() {
    // Create a node with NO representations
    let empty_node = ContentNode::new("empty-node");

    // All should be None
    assert!(empty_node.representations.visual.is_none());
    assert!(empty_node.representations.auditory.is_none());
    assert!(empty_node.representations.tactile.is_none());
    assert!(empty_node.representations.spatial.is_none());
    assert!(empty_node.representations.gestural.is_none());

    // Serialize
    let json = serde_json::to_string_pretty(&empty_node).unwrap();
    println!("\n{}", json);

    // NO DEFAULT EXISTS!
    // This is a core principle
}

#[test]
fn test_document_tree() {
    // Create a document with nested nodes
    let doc = PubScriptDocument {
        metadata: Metadata {
            title: Some("Document Tree Test".to_string()),
            version: "3.0".to_string(),
            ..Default::default()
        },
        content: vec![
            ContentNode::new("root-1")
                .with_representations(Representations::new().with_visual(VisualRep::text("Root")))
                .add_child(
                    ContentNode::new("child-1-1").with_representations(
                        Representations::new().with_visual(VisualRep::text("Child 1")),
                    ),
                )
                .add_child(
                    ContentNode::new("child-1-2").with_representations(
                        Representations::new().with_visual(VisualRep::text("Child 2")),
                    ),
                ),
        ],
        timeline: None,
    };

    // Serialize
    let json = serde_json::to_string_pretty(&doc).unwrap();
    println!("\n{}", json);

    // Deserialize
    let parsed: PubScriptDocument = serde_json::from_str(&json).unwrap();
    assert_eq!(parsed.content.len(), 1);
    assert_eq!(parsed.content[0].children.len(), 2);
}
