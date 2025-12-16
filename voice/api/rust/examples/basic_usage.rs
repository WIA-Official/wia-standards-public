//! Basic usage example for WIA Voice-Sign API
//!
//! This example demonstrates how to use the Voice-Sign API to translate
//! text input to sign language representation.
//!
//! Run with: cargo run --example basic_usage

use wia_voice_sign::*;

#[tokio::main]
async fn main() {
    println!("WIA Voice-Sign API - Basic Usage Example");
    println!("=========================================\n");

    // Create a translation pipeline with ASL as target language
    let pipeline = create_pipeline(SignLanguageCode::Asl);

    // Example 1: Simple text translation
    println!("Example 1: Simple text to ASL translation");
    println!("-----------------------------------------");

    let request = TranslationRequest {
        request_id: "example-001".to_string(),
        audio: None,
        text: Some(TextInput {
            text: "Hello, how are you?".to_string(),
            language: "en".to_string(),
        }),
        target_language: SignLanguageCode::Asl,
        output: OutputPreferences {
            include_transcript: true,
            include_gloss: true,
            include_notation: true,
            include_pose: false,
            include_render: false,
            render_settings: None,
        },
        options: None,
    };

    let response = pipeline.process(request).await;

    println!("Status: {:?}", response.status);
    println!(
        "Processing time: {:.2}ms",
        response.processing.total_time_ms
    );

    if let Some(gloss) = &response.gloss {
        println!("\nGloss sequence ({}):", gloss.sign_language);
        for g in &gloss.glosses {
            println!(
                "  - {} ({:?}) [{:.0}ms - {:.0}ms]",
                g.gloss, g.sign_type, g.start_ms, g.end_ms
            );
            if let Some(hamnosys) = &g.hamnosys {
                println!("    HamNoSys: {}", hamnosys);
            }
        }
    }

    // Example 2: Korean to KSL translation
    println!("\n\nExample 2: Korean to KSL translation");
    println!("-------------------------------------");

    let ksl_pipeline = create_pipeline(SignLanguageCode::Ksl);

    let ksl_request = TranslationRequest {
        request_id: "example-002".to_string(),
        audio: None,
        text: Some(TextInput {
            text: "안녕하세요".to_string(),
            language: "ko".to_string(),
        }),
        target_language: SignLanguageCode::Ksl,
        output: OutputPreferences {
            include_transcript: false,
            include_gloss: true,
            include_notation: false,
            include_pose: false,
            include_render: false,
            render_settings: None,
        },
        options: None,
    };

    let ksl_response = ksl_pipeline.process(ksl_request).await;

    println!("Status: {:?}", ksl_response.status);

    if let Some(gloss) = &ksl_response.gloss {
        println!("\nKSL Gloss sequence:");
        for g in &gloss.glosses {
            println!("  - {} ({:?})", g.gloss, g.sign_type);
        }
    }

    // Example 3: Get pose animation data
    println!("\n\nExample 3: Generate pose animation");
    println!("-----------------------------------");

    let pose_request = TranslationRequest {
        request_id: "example-003".to_string(),
        audio: None,
        text: Some(TextInput {
            text: "Thank you".to_string(),
            language: "en".to_string(),
        }),
        target_language: SignLanguageCode::Asl,
        output: OutputPreferences {
            include_transcript: false,
            include_gloss: true,
            include_notation: false,
            include_pose: true,
            include_render: false,
            render_settings: None,
        },
        options: None,
    };

    let pose_response = pipeline.process(pose_request).await;

    if let Some(pose) = &pose_response.pose {
        println!("Pose sequence generated:");
        println!("  - Frame rate: {} FPS", pose.frame_rate);
        println!("  - Total frames: {}", pose.total_frames);
        println!("  - Duration: {:.2}ms", pose.duration_ms);
        println!("  - Skeleton standard: {:?}", pose.skeleton.standard);
        println!("  - Joint count: {}", pose.skeleton.joints.len());

        if let Some(first_frame) = pose.frames.first() {
            println!("\nFirst frame data:");
            println!("  - Frame index: {}", first_frame.frame_index);
            println!("  - Joint positions: {}", first_frame.joints.len());
            if first_frame.left_hand.is_some() {
                println!("  - Left hand: 21 landmarks");
            }
            if first_frame.right_hand.is_some() {
                println!("  - Right hand: 21 landmarks");
            }
        }
    }

    // Example 4: Using the gloss database directly
    println!("\n\nExample 4: Direct gloss database lookup");
    println!("----------------------------------------");

    let db = GlossDatabase::new(SignLanguageCode::Asl);

    let words = ["hello", "thank", "please", "help"];
    for word in words {
        let entries = db.lookup_word(word);
        if let Some(entry) = entries.first() {
            println!(
                "  '{}' -> {} ({:?})",
                word, entry.gloss, entry.sign_type
            );
        }
    }

    println!("\n\nAll examples completed successfully!");
}
