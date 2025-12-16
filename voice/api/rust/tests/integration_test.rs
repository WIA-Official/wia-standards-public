//! Integration tests for WIA Voice-Sign API

use chrono::Utc;
use wia_voice_sign::*;
use wia_voice_sign::adapters::*;

/// Test the complete translation pipeline with text input
#[tokio::test]
async fn test_text_to_sign_pipeline() {
    let pipeline = create_simulator_pipeline(SignLanguageCode::Asl);

    let request = TranslationRequest {
        request_id: "integration-test-001".to_string(),
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
            include_pose: true,
            include_render: false,
            render_settings: None,
        },
        options: None,
    };

    let response = pipeline.process(request).await;

    // Verify success
    assert_eq!(response.status, ResponseStatus::Success);
    assert!(response.error.is_none());

    // Verify gloss output
    assert!(response.gloss.is_some());
    let gloss = response.gloss.unwrap();
    assert!(!gloss.glosses.is_empty());
    assert_eq!(gloss.sign_language, SignLanguageCode::Asl);

    // Verify pose output
    assert!(response.pose.is_some());
    let pose = response.pose.unwrap();
    assert!(pose.total_frames > 0);
    assert!(!pose.frames.is_empty());

    // Verify processing times
    assert!(response.processing.total_time_ms > 0.0);
}

/// Test the pipeline with audio input
#[tokio::test]
async fn test_audio_to_sign_pipeline() {
    let pipeline = create_simulator_pipeline(SignLanguageCode::Asl);

    let request = TranslationRequest {
        request_id: "integration-test-002".to_string(),
        audio: Some(AudioInput {
            audio_id: "test-audio".to_string(),
            timestamp: Utc::now(),
            format: AudioFormat::Wav,
            sample_rate: 16000,
            channels: 1,
            bit_depth: 16,
            duration_ms: 2000.0,
            source: None,
            language_hint: Some("en".to_string()),
            data: Some(vec![0u8; 64000]),
            data_url: None,
        }),
        text: None,
        target_language: SignLanguageCode::Asl,
        output: OutputPreferences {
            include_transcript: true,
            include_gloss: true,
            include_notation: false,
            include_pose: false,
            include_render: false,
            render_settings: None,
        },
        options: None,
    };

    let response = pipeline.process(request).await;

    assert_eq!(response.status, ResponseStatus::Success);
    assert!(response.transcript.is_some());
    assert!(response.gloss.is_some());
}

/// Test Korean Sign Language (KSL) translation
#[tokio::test]
async fn test_korean_sign_language() {
    let pipeline = create_simulator_pipeline(SignLanguageCode::Ksl);

    let request = TranslationRequest {
        request_id: "ksl-test-001".to_string(),
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

    let response = pipeline.process(request).await;

    assert_eq!(response.status, ResponseStatus::Success);
    assert!(response.gloss.is_some());

    let gloss = response.gloss.unwrap();
    assert_eq!(gloss.sign_language, SignLanguageCode::Ksl);
}

/// Test request validation
#[test]
fn test_request_validation() {
    // Valid request
    let valid_request = TranslationRequest {
        request_id: "valid".to_string(),
        audio: None,
        text: Some(TextInput {
            text: "Hello".to_string(),
            language: "en".to_string(),
        }),
        target_language: SignLanguageCode::Asl,
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

    assert!(validate_request(&valid_request).is_ok());

    // Invalid request: no input
    let no_input_request = TranslationRequest {
        request_id: "invalid".to_string(),
        audio: None,
        text: None,
        target_language: SignLanguageCode::Asl,
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

    assert!(validate_request(&no_input_request).is_err());

    // Invalid request: both audio and text
    let both_input_request = TranslationRequest {
        request_id: "invalid".to_string(),
        audio: Some(AudioInput {
            audio_id: "test".to_string(),
            timestamp: Utc::now(),
            format: AudioFormat::Wav,
            sample_rate: 16000,
            channels: 1,
            bit_depth: 16,
            duration_ms: 1000.0,
            source: None,
            language_hint: None,
            data: Some(vec![0]),
            data_url: None,
        }),
        text: Some(TextInput {
            text: "Hello".to_string(),
            language: "en".to_string(),
        }),
        target_language: SignLanguageCode::Asl,
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

    assert!(validate_request(&both_input_request).is_err());
}

/// Test gloss database lookups
#[test]
fn test_gloss_database() {
    let db = GlossDatabase::new(SignLanguageCode::Asl);

    // Test direct lookup
    let hello = db.get("HELLO");
    assert!(hello.is_some());
    assert_eq!(hello.unwrap().sign_type, SignType::Lexical);

    // Test word lookup
    let glosses = db.lookup_word("hello");
    assert!(!glosses.is_empty());
    assert_eq!(glosses[0].gloss, "HELLO");

    // Test has_sign
    assert!(db.has_sign("hello"));
    assert!(db.has_sign("thank"));
    assert!(!db.has_sign("supercalifragilisticexpialidocious"));
}

/// Test audio processor
#[test]
fn test_audio_processor() {
    let processor = AudioProcessor::new();

    // Valid audio
    let valid_audio = AudioInput {
        audio_id: "test".to_string(),
        timestamp: Utc::now(),
        format: AudioFormat::Wav,
        sample_rate: 16000,
        channels: 1,
        bit_depth: 16,
        duration_ms: 1000.0,
        source: None,
        language_hint: None,
        data: Some(vec![0u8; 32000]),
        data_url: None,
    };

    assert!(processor.validate(&valid_audio).is_ok());

    // Invalid sample rate
    let invalid_rate = AudioInput {
        sample_rate: 12345,
        ..valid_audio.clone()
    };
    assert!(processor.validate(&invalid_rate).is_err());

    // Duration too long
    let too_long = AudioInput {
        duration_ms: 4000000.0,
        ..valid_audio.clone()
    };
    assert!(processor.validate(&too_long).is_err());
}

/// Test rendering with avatar
#[tokio::test]
async fn test_rendering_pipeline() {
    let pipeline = create_simulator_pipeline(SignLanguageCode::Asl);

    let request = TranslationRequest {
        request_id: "render-test-001".to_string(),
        audio: None,
        text: Some(TextInput {
            text: "Hello".to_string(),
            language: "en".to_string(),
        }),
        target_language: SignLanguageCode::Asl,
        output: OutputPreferences {
            include_transcript: false,
            include_gloss: true,
            include_notation: false,
            include_pose: true,
            include_render: true,
            render_settings: Some(RenderSettings {
                format: OutputType::Video,
                resolution: Resolution {
                    width: 1920,
                    height: 1080,
                },
                frame_rate: 30,
                avatar_id: Some("avatar-001".to_string()),
            }),
        },
        options: None,
    };

    let response = pipeline.process(request).await;

    assert_eq!(response.status, ResponseStatus::Success);
    assert!(response.render.is_some());

    let render = response.render.unwrap();
    assert_eq!(render.resolution.width, 1920);
    assert_eq!(render.resolution.height, 1080);
    assert!(render.data_url.is_some());
}

/// Test serialization round-trip
#[test]
fn test_serialization() {
    let gloss = SignGloss {
        gloss: "HELLO".to_string(),
        start_ms: 0.0,
        end_ms: 500.0,
        duration_ms: 500.0,
        sign_type: SignType::Lexical,
        hamnosys: Some("hamfinger2".to_string()),
        sigml: None,
        signwriting: None,
        modifiers: None,
        spatial_ref: Some(SpatialReference {
            x: 0.0,
            y: 0.5,
            z: 0.2,
        }),
        confidence: 0.95,
    };

    let json = serde_json::to_string(&gloss).unwrap();
    let parsed: SignGloss = serde_json::from_str(&json).unwrap();

    assert_eq!(parsed.gloss, "HELLO");
    assert_eq!(parsed.sign_type, SignType::Lexical);
    assert!(parsed.spatial_ref.is_some());
}

/// Test error codes
#[test]
fn test_error_codes() {
    use wia_voice_sign::error::*;

    let audio_err = VoiceSignError::Audio(AudioError::EmptyAudio);
    assert_eq!(audio_err.code(), "AUDIO_EMPTY");

    let trans_err = VoiceSignError::Translation(TranslationError::UnsupportedSignLanguage(
        "XYZ".to_string(),
    ));
    assert_eq!(trans_err.code(), "TRANSLATION_UNSUPPORTED_LANGUAGE");

    let val_err = VoiceSignError::Validation(ValidationError::NoInput);
    assert_eq!(val_err.code(), "VALIDATION_NO_INPUT");
}
