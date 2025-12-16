//! Integration tests for WIA Auto API

use wia_auto::prelude::*;
use uuid::Uuid;
use chrono::Utc;

// ============================================================================
// Simulator Fleet Tests
// ============================================================================

#[tokio::test]
async fn test_simulator_vehicle_registration() {
    let fleet = SimulatorFleetManager::new();

    let vehicle = SimulatorFleetManager::create_sample_wheelchair_av();
    let vehicle_id = vehicle.vehicle_id;

    fleet.add_vehicle(vehicle.clone()).await;

    let retrieved = fleet.get_vehicle(vehicle_id).await.unwrap();
    assert!(retrieved.is_some());
    assert_eq!(retrieved.unwrap().vehicle_id, vehicle_id);
}

#[tokio::test]
async fn test_simulator_find_vehicle_by_requirements() {
    let fleet = SimulatorFleetManager::new();

    // Register a wheelchair-accessible vehicle
    let wheelchair_av = SimulatorFleetManager::create_sample_wheelchair_av();
    fleet.add_vehicle(wheelchair_av).await;

    // Register a non-accessible vehicle
    let standard_av = SimulatorFleetManager::create_sample_standard_av();
    fleet.add_vehicle(standard_av).await;

    // Search for wheelchair-accessible vehicle
    let requirements = AccessibilityRequirements::builder()
        .wheelchair_accessible()
        .requires_ramp()
        .build();

    let location = GeoLocation {
        latitude: 37.5665,
        longitude: 126.9780,
        address: None,
    };

    let found = fleet.find_vehicle(&requirements, &location).await.unwrap();
    assert!(found.is_some());

    let found_vehicle = found.unwrap();
    assert!(found_vehicle.accessibility_features.interior
        .as_ref()
        .map(|i| i.wheelchair_spaces > 0)
        .unwrap_or(false));
}

#[tokio::test]
async fn test_simulator_profile_crud() {
    let profiles = SimulatorProfileManager::new();

    let profile = PassengerProfile::builder()
        .name("Test User")
        .add_disability(DisabilityType::MobilityWheelchairPower)
        .mobility_aid(MobilityAidType::PowerWheelchair, true, true)
        .build()
        .unwrap();

    let profile_id = profile.profile_id;

    // Create
    profiles.save_profile(&profile).await.unwrap();

    // Read
    let retrieved = profiles.get_profile(profile_id).await.unwrap();
    assert!(retrieved.is_some());
    assert_eq!(retrieved.unwrap().profile_id, profile_id);

    // Delete
    profiles.delete_profile(profile_id).await.unwrap();
    let retrieved = profiles.get_profile(profile_id).await.unwrap();
    assert!(retrieved.is_none());
}

#[tokio::test]
async fn test_simulator_trip_flow() {
    let fleet = SimulatorFleetManager::new();
    fleet.add_vehicle(SimulatorFleetManager::create_sample_wheelchair_av()).await;

    let trips = SimulatorTripManager::new(fleet);

    let request = TripRequest {
        request_id: Uuid::new_v4(),
        version: "1.0.0".to_string(),
        timestamp: Utc::now(),
        passenger_profile_id: Some(Uuid::new_v4()),
        trip: TripDetails {
            pickup: TripLocation {
                location: GeoLocation {
                    latitude: 37.5665,
                    longitude: 126.9780,
                    address: Some("Seoul City Hall".to_string()),
                },
                notes: None,
                pickup_side: Some(PickupSide::SameSide),
                curb_to_curb: true,
            },
            dropoff: TripLocation {
                location: GeoLocation {
                    latitude: 37.5512,
                    longitude: 126.9882,
                    address: Some("Namsan Tower".to_string()),
                },
                notes: None,
                pickup_side: None,
                curb_to_curb: false,
            },
            scheduled_time: None,
            flexibility_minutes: Some(15),
        },
        accessibility_requirements: Some(AccessibilityRequirements::builder()
            .wheelchair_accessible()
            .requires_ramp()
            .build()),
        preferences: None,
    };

    // Request trip
    let response = trips.request_trip(&request).await.unwrap();
    assert_eq!(response.status, TripStatus::Confirmed);
    assert!(response.vehicle.is_some());

    // Get trip
    let retrieved = trips.get_trip(request.request_id).await.unwrap();
    assert!(retrieved.is_some());

    // Cancel trip
    trips.cancel_trip(request.request_id).await.unwrap();
    let cancelled = trips.get_trip(request.request_id).await.unwrap();
    assert!(cancelled.is_none());
}

// ============================================================================
// Matching Algorithm Tests
// ============================================================================

#[test]
fn test_match_wheelchair_requirements() {
    let vehicle = SimulatorFleetManager::create_sample_wheelchair_av();
    let requirements = AccessibilityRequirements::builder()
        .wheelchair_accessible()
        .requires_ramp()
        .service_animal_space()
        .add_modality(InteractionModality::AudioTts)
        .add_modality(InteractionModality::HapticVibration)
        .build();

    let result = match_vehicle(&requirements, &vehicle);

    assert!(result.is_match);
    assert!(result.score > 90);
    assert!(result.details.wheelchair_accessible);
    assert!(result.details.ramp_available);
    assert!(result.details.service_animal_ok);
    assert!(result.unmet.is_empty());
}

#[test]
fn test_match_unmet_requirements() {
    let vehicle = SimulatorFleetManager::create_sample_standard_av();
    let requirements = AccessibilityRequirements::builder()
        .wheelchair_accessible()
        .requires_ramp()
        .build();

    let result = match_vehicle(&requirements, &vehicle);

    assert!(!result.is_match);
    assert!(!result.details.wheelchair_accessible);
    assert!(!result.details.ramp_available);
    assert!(!result.unmet.is_empty());
}

#[test]
fn test_rank_vehicles() {
    let wheelchair_av = SimulatorFleetManager::create_sample_wheelchair_av();
    let standard_av = SimulatorFleetManager::create_sample_standard_av();

    let vehicles = vec![standard_av, wheelchair_av];

    let requirements = AccessibilityRequirements::builder()
        .wheelchair_accessible()
        .requires_ramp()
        .build();

    let ranked = rank_vehicles(&requirements, &vehicles);

    assert_eq!(ranked.len(), 2);
    // Wheelchair-accessible vehicle should be first
    assert!(ranked[0].1.is_match);
    assert!(!ranked[1].1.is_match);
}

#[test]
fn test_find_best_match() {
    let wheelchair_av = SimulatorFleetManager::create_sample_wheelchair_av();
    let standard_av = SimulatorFleetManager::create_sample_standard_av();

    let vehicles = vec![standard_av.clone(), wheelchair_av];

    let requirements = AccessibilityRequirements::builder()
        .wheelchair_accessible()
        .requires_ramp()
        .build();

    let best = find_best_match(&requirements, &vehicles);
    assert!(best.is_some());
    assert!(best.unwrap().1.is_match);

    // No match when all vehicles are standard
    let only_standard = vec![standard_av];
    let best = find_best_match(&requirements, &only_standard);
    assert!(best.is_none());
}

// ============================================================================
// Builder Tests
// ============================================================================

#[test]
fn test_passenger_profile_builder() {
    let profile = PassengerProfile::builder()
        .name("Test User")
        .add_disability(DisabilityType::MobilityWheelchairPower)
        .mobility_aid(MobilityAidType::PowerWheelchair, true, true)
        .preferred_language("ko")
        .high_contrast(true)
        .build()
        .unwrap();

    assert_eq!(profile.passenger.name, Some("Test User".to_string()));
    assert_eq!(profile.passenger.disabilities.len(), 1);
    assert!(profile.mobility_aid.is_some());
    assert!(profile.communication.is_some());
    assert!(profile.preferences.is_some());
}

#[test]
fn test_profile_builder_requires_disability() {
    let result = PassengerProfile::builder()
        .name("No Disability")
        .build();

    assert!(result.is_err());
}

#[test]
fn test_hmi_config_presets() {
    let blind_config = HmiConfig::for_blind();
    assert!(blind_config.visual.as_ref().map(|v| !v.enabled).unwrap_or(true));
    assert!(blind_config.audio.as_ref().map(|a| a.enabled).unwrap_or(false));
    assert!(blind_config.haptic.as_ref().map(|h| h.enabled).unwrap_or(false));

    let deaf_config = HmiConfig::for_deaf();
    assert!(deaf_config.visual.as_ref().map(|v| v.enabled).unwrap_or(false));
    assert!(deaf_config.audio.as_ref().map(|a| !a.enabled).unwrap_or(true));
    assert!(deaf_config.haptic.as_ref().map(|h| h.enabled).unwrap_or(false));
}

#[test]
fn test_accessibility_requirements_builder() {
    let requirements = AccessibilityRequirements::builder()
        .wheelchair_accessible()
        .requires_ramp()
        .service_animal_space()
        .companion_space(1)
        .add_modality(InteractionModality::AudioTts)
        .add_modality(InteractionModality::HapticVibration)
        .build();

    assert!(requirements.wheelchair_accessible);
    assert!(requirements.ramp_required);
    assert!(requirements.service_animal_space);
    assert_eq!(requirements.companion_space, 1);
    assert_eq!(requirements.preferred_modalities.len(), 2);
}

#[test]
fn test_requirements_from_profile() {
    let profile = PassengerProfile::builder()
        .name("Wheelchair User")
        .add_disability(DisabilityType::MobilityWheelchairPower)
        .mobility_aid(MobilityAidType::PowerWheelchair, true, true)
        .service_animal("dog", AnimalSize::Large)
        .preferred_modalities(vec![InteractionModality::AudioTts])
        .build()
        .unwrap();

    let requirements = AccessibilityRequirements::from_profile(&profile);

    assert!(requirements.wheelchair_accessible);
    assert!(requirements.ramp_required);
    assert!(requirements.service_animal_space);
}

// ============================================================================
// Type Serialization Tests
// ============================================================================

#[test]
fn test_profile_serialization() {
    let profile = PassengerProfile::builder()
        .name("Test")
        .add_disability(DisabilityType::VisualBlind)
        .build()
        .unwrap();

    let json = serde_json::to_string(&profile).unwrap();
    let deserialized: PassengerProfile = serde_json::from_str(&json).unwrap();

    assert_eq!(profile.profile_id, deserialized.profile_id);
    assert_eq!(profile.passenger.name, deserialized.passenger.name);
}

#[test]
fn test_vehicle_serialization() {
    let vehicle = SimulatorFleetManager::create_sample_wheelchair_av();

    let json = serde_json::to_string(&vehicle).unwrap();
    let deserialized: VehicleCapabilities = serde_json::from_str(&json).unwrap();

    assert_eq!(vehicle.vehicle_id, deserialized.vehicle_id);
}

#[test]
fn test_trip_request_serialization() {
    let request = TripRequest {
        request_id: Uuid::new_v4(),
        version: "1.0.0".to_string(),
        timestamp: Utc::now(),
        passenger_profile_id: None,
        trip: TripDetails {
            pickup: TripLocation {
                location: GeoLocation {
                    latitude: 37.5665,
                    longitude: 126.9780,
                    address: None,
                },
                notes: None,
                pickup_side: None,
                curb_to_curb: false,
            },
            dropoff: TripLocation {
                location: GeoLocation {
                    latitude: 37.5512,
                    longitude: 126.9882,
                    address: None,
                },
                notes: None,
                pickup_side: None,
                curb_to_curb: false,
            },
            scheduled_time: None,
            flexibility_minutes: None,
        },
        accessibility_requirements: None,
        preferences: None,
    };

    let json = serde_json::to_string(&request).unwrap();
    let deserialized: TripRequest = serde_json::from_str(&json).unwrap();

    assert_eq!(request.request_id, deserialized.request_id);
}

// ============================================================================
// Simulator Client Tests
// ============================================================================

#[tokio::test]
async fn test_create_simulator_client() {
    let client = create_simulator_client();

    // Add vehicle through fleet manager
    client.fleet().add_vehicle(SimulatorFleetManager::create_sample_wheelchair_av()).await;

    // Verify vehicle was added
    let vehicles = client.fleet().list_vehicles().await.unwrap();
    assert_eq!(vehicles.len(), 1);
}

#[tokio::test]
async fn test_hmi_manager() {
    let hmi = SimulatorHmiManager::new();
    let vehicle_id = Uuid::new_v4();

    let config = HmiConfig::for_blind();
    hmi.apply_config(vehicle_id, &config).await.unwrap();

    let retrieved = hmi.get_config(vehicle_id).await.unwrap();
    assert!(retrieved.is_some());
    assert_eq!(retrieved.unwrap().config_id, config.config_id);

    hmi.reset_config(vehicle_id).await.unwrap();
    let after_reset = hmi.get_config(vehicle_id).await.unwrap();
    assert!(after_reset.is_none());
}

#[tokio::test]
async fn test_securement_monitor() {
    let securement = SimulatorSecurementMonitor::new();
    let vehicle_id = Uuid::new_v4();

    // Start securing
    securement.start_securing(vehicle_id).await.unwrap();

    // Check status
    let status = securement.get_status(vehicle_id).await.unwrap();
    assert_eq!(status.overall_status, OverallSecurementStatus::Securing);
    assert!(status.wheelchair_detected);

    // Release
    securement.release(vehicle_id).await.unwrap();

    // Should fail now
    let result = securement.get_status(vehicle_id).await;
    assert!(result.is_err());
}

#[tokio::test]
async fn test_emergency_handler() {
    let emergency = SimulatorEmergencyHandler::new();
    let vehicle_id = Uuid::new_v4();
    let location = GeoLocation {
        latitude: 37.5665,
        longitude: 126.9780,
        address: Some("Test Location".to_string()),
    };

    // Trigger panic button
    let event = emergency.panic_button(vehicle_id, location).await.unwrap();

    assert_eq!(event.event_type, EmergencyEventType::PanicButton);
    assert_eq!(event.severity, EmergencySeverity::High);
    assert!(event.response.as_ref().map(|r| r.auto_pulled_over).unwrap_or(false));

    // Verify event was recorded
    let events = emergency.get_events().await;
    assert_eq!(events.len(), 1);
}
