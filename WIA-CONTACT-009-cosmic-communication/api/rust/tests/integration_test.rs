//! Integration tests for WIA-CONTACT-009 Cosmic Communication SDK

use wia_contact_009::*;
use chrono::Utc;
use uuid::Uuid;

#[test]
fn test_signal_type_variants() {
    let types = vec![
        SignalType::Radio,
        SignalType::Optical,
        SignalType::Neutrino,
        SignalType::Gravitational,
        SignalType::Unknown,
    ];

    for signal_type in types {
        let json = serde_json::to_string(&signal_type).unwrap();
        let deserialized: SignalType = serde_json::from_str(&json).unwrap();
        assert_eq!(signal_type, deserialized);
    }
}

#[test]
fn test_cosmic_coordinate_validation() {
    use wia_contact_009::validators::validate_coordinate;

    // Valid coordinate
    let valid_coord = CosmicCoordinate {
        right_ascension: 12.5,
        declination: 45.0,
        distance: 100.0,
        galactic_longitude: Some(180.0),
        galactic_latitude: Some(-30.0),
    };
    assert!(validate_coordinate(&valid_coord).is_ok());

    // Invalid right ascension
    let invalid_coord = CosmicCoordinate {
        right_ascension: 25.0, // > 24
        declination: 45.0,
        distance: 100.0,
        galactic_longitude: None,
        galactic_latitude: None,
    };
    assert!(validate_coordinate(&invalid_coord).is_err());

    // Invalid declination
    let invalid_coord = CosmicCoordinate {
        right_ascension: 12.0,
        declination: 100.0, // > 90
        distance: 100.0,
        galactic_longitude: None,
        galactic_latitude: None,
    };
    assert!(validate_coordinate(&invalid_coord).is_err());
}

#[test]
fn test_universal_message_creation() {
    let message = UniversalMessage {
        message_id: Uuid::new_v4().to_string(),
        message_type: MessageType::Greeting,
        content: "Test message".to_string(),
        encoding: EncodingFormat::Binary,
        protocol_version: "1.0".to_string(),
        created_at: Utc::now(),
    };

    assert!(!message.message_id.is_empty());
    assert_eq!(message.message_type, MessageType::Greeting);
    assert_eq!(message.content, "Test message");
}

#[test]
fn test_message_validation() {
    use wia_contact_009::validators::validate_message;

    let valid_message = UniversalMessage {
        message_id: "msg-001".to_string(),
        message_type: MessageType::Science,
        content: "Scientific data".to_string(),
        encoding: EncodingFormat::Mathematical,
        protocol_version: "1.0".to_string(),
        created_at: Utc::now(),
    };
    assert!(validate_message(&valid_message).is_ok());

    let invalid_message = UniversalMessage {
        message_id: "".to_string(), // Empty
        message_type: MessageType::Query,
        content: "Query".to_string(),
        encoding: EncodingFormat::Prime,
        protocol_version: "1.0".to_string(),
        created_at: Utc::now(),
    };
    assert!(validate_message(&invalid_message).is_err());
}

#[test]
fn test_frequency_wavelength_conversion() {
    use wia_contact_009::utils::{frequency_to_wavelength, wavelength_to_frequency};

    let freq = 1420e6; // Hydrogen line
    let wavelength = frequency_to_wavelength(freq);
    let freq2 = wavelength_to_frequency(wavelength);

    assert!((freq - freq2).abs() < 1.0); // Within 1 Hz
}

#[test]
fn test_snr_calculation() {
    use wia_contact_009::utils::calculate_snr;

    let snr = calculate_snr(100.0, 10.0);
    assert!((snr - 10.0).abs() < 0.01);

    let snr = calculate_snr(1000.0, 100.0);
    assert!((snr - 10.0).abs() < 0.01);
}

#[test]
fn test_distance_conversions() {
    use wia_contact_009::utils::{light_years_to_parsecs, parsecs_to_light_years};

    let ly = 100.0;
    let pc = light_years_to_parsecs(ly);
    let ly2 = parsecs_to_light_years(pc);

    assert!((ly - ly2).abs() < 0.01);
}

#[test]
fn test_drake_equation() {
    use wia_contact_009::utils::drake_equation;

    // Optimistic estimate
    let n = drake_equation(
        1.0,   // star formation rate
        1.0,   // fraction with planets
        1.0,   // planets per system
        1.0,   // fraction with life
        1.0,   // fraction intelligent
        1.0,   // fraction communicating
        1000.0, // civilization lifetime
    );

    assert_eq!(n, 1000.0);

    // Pessimistic estimate
    let n = drake_equation(
        1.0,
        0.5,
        0.5,
        0.01,
        0.01,
        0.01,
        100.0,
    );

    assert!(n > 0.0 && n < 1.0);
}

#[test]
fn test_coordinate_formatting() {
    use wia_contact_009::utils::format_coordinate;

    let coord = CosmicCoordinate {
        right_ascension: 12.5,
        declination: 45.0,
        distance: 100.0,
        galactic_longitude: Some(180.0),
        galactic_latitude: Some(-30.0),
    };

    let formatted = format_coordinate(&coord);
    assert!(formatted.contains("12.50"));
    assert!(formatted.contains("45.00"));
    assert!(formatted.contains("100.00"));
}

#[test]
fn test_generate_greeting() {
    use wia_contact_009::utils::generate_greeting;

    let greeting = generate_greeting();
    assert_eq!(greeting.message_type, MessageType::Greeting);
    assert!(!greeting.content.is_empty());
    assert!(greeting.content.contains("홍익인간"));
    assert_eq!(greeting.encoding, EncodingFormat::Binary);
}

#[test]
fn test_client_creation() {
    let client = CosmicClient::new("test-key".to_string());
    assert_eq!(client.clone().api_key, "test-key");

    let client = CosmicClient::with_base_url(
        "test-key".to_string(),
        "https://custom.api".to_string()
    );
    assert_eq!(client.base_url, "https://custom.api");
}

#[test]
fn test_analysis_result_validation() {
    use wia_contact_009::validators::validate_analysis;

    let valid_analysis = AnalysisResult {
        analysis_id: "analysis-001".to_string(),
        signal_id: "signal-001".to_string(),
        artificial_probability: 0.85,
        patterns: vec!["prime_sequence".to_string()],
        recommendations: vec!["Further observation".to_string()],
        analyzed_at: Utc::now(),
    };
    assert!(validate_analysis(&valid_analysis).is_ok());

    let invalid_analysis = AnalysisResult {
        analysis_id: "".to_string(), // Empty
        signal_id: "signal-001".to_string(),
        artificial_probability: 0.85,
        patterns: vec![],
        recommendations: vec![],
        analyzed_at: Utc::now(),
    };
    assert!(validate_analysis(&invalid_analysis).is_err());
}
