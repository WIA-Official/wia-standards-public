//! Trip request example for WIA Auto API
//!
//! This example demonstrates:
//! - Creating and managing trip requests
//! - Using the simulator for trip lifecycle
//! - Handling emergency events
//! - Monitoring securement status

use wia_auto::prelude::*;
use uuid::Uuid;
use chrono::Utc;

#[tokio::main]
async fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Auto API - Trip Request Example ===\n");

    // Initialize simulators
    let fleet = SimulatorFleetManager::new();
    let profiles = SimulatorProfileManager::new();
    let securement = SimulatorSecurementMonitor::new();
    let emergency = SimulatorEmergencyHandler::new();

    // ========================================================================
    // 1. Setup: Create Profile and Register Vehicles
    // ========================================================================
    println!("1. Setting up profile and vehicles...");

    // Create passenger profile
    let profile = PassengerProfile::builder()
        .name("Lee Seunggaek")
        .add_disability(DisabilityType::MobilityWheelchairPower)
        .mobility_aid_with_dimensions(
            MobilityAidType::PowerWheelchair,
            Dimensions {
                width_cm: 70.0,
                length_cm: 125.0,
                height_cm: 105.0,
                weight_kg: 150.0,
            },
            true,  // requires_ramp
            false, // requires_lift
            true,  // requires_securement
        )
        .preferred_language("ko")
        .emergency_contact("Lee Bohoja", "010-9876-5432", Some("Spouse"))
        .build()?;

    let profile_id = profile.profile_id;
    profiles.save_profile(&profile).await?;
    println!("   Created profile: {}", profile_id);

    // Register accessible vehicle
    let vehicle = SimulatorFleetManager::create_sample_wheelchair_av();
    let vehicle_id = vehicle.vehicle_id;
    fleet.add_vehicle(vehicle).await;
    println!("   Registered vehicle: {}", vehicle_id);

    // Create trip manager with fleet
    let trips = SimulatorTripManager::new(fleet.clone());

    // ========================================================================
    // 2. Create Trip Request
    // ========================================================================
    println!("\n2. Creating trip request...");

    let trip_request = TripRequest {
        request_id: Uuid::new_v4(),
        version: "1.0.0".to_string(),
        timestamp: Utc::now(),
        passenger_profile_id: Some(profile_id),
        trip: TripDetails {
            pickup: TripLocation {
                location: GeoLocation {
                    latitude: 37.5665,
                    longitude: 126.9780,
                    address: Some("Seoul City Hall, 110 Sejong-daero, Jung-gu".to_string()),
                },
                notes: Some("Wheelchair boarding area at main entrance".to_string()),
                pickup_side: Some(PickupSide::SameSide),
                curb_to_curb: true,
            },
            dropoff: TripLocation {
                location: GeoLocation {
                    latitude: 37.5796,
                    longitude: 126.9770,
                    address: Some("Gyeongbokgung Palace, 161 Sajik-ro, Jongno-gu".to_string()),
                },
                notes: Some("Gwanghwamun Gate entrance".to_string()),
                pickup_side: None,
                curb_to_curb: true,
            },
            scheduled_time: Some(Utc::now() + chrono::Duration::minutes(30)),
            flexibility_minutes: Some(10),
        },
        accessibility_requirements: Some(AccessibilityRequirements::builder()
            .wheelchair_accessible()
            .requires_ramp()
            .add_modality(InteractionModality::AudioTts)
            .add_modality(InteractionModality::HapticVibration)
            .build()),
        preferences: Some(TripPreferences {
            minimize_walking: true,
            audio_guidance: true,
            visual_guidance: false,
            haptic_feedback: true,
            quiet_ride: true,
        }),
    };

    println!("   Request ID: {}", trip_request.request_id);
    println!("   From: {:?}", trip_request.trip.pickup.location.address);
    println!("   To: {:?}", trip_request.trip.dropoff.location.address);

    // ========================================================================
    // 3. Submit Trip Request
    // ========================================================================
    println!("\n3. Submitting trip request...");

    let response = trips.request_trip(&trip_request).await?;

    println!("   Response ID: {}", response.response_id);
    println!("   Status: {:?}", response.status);

    if let Some(ref vehicle_assignment) = response.vehicle {
        println!("   Vehicle assigned: {}", vehicle_assignment.vehicle_id);
        println!("   ETA: {:.1} minutes", vehicle_assignment.eta_minutes);
        println!("   Distance: {:.1} km", vehicle_assignment.distance_km);
    }

    if let Some(ref access_match) = response.accessibility_match {
        println!("   Accessibility score: {}", access_match.match_score);
    }

    if let Some(ref wayfinding) = response.wayfinding {
        println!("   Wayfinding: {:?}", wayfinding.pickup_instructions);
    }

    // ========================================================================
    // 4. Monitor Trip Status
    // ========================================================================
    println!("\n4. Monitoring trip status...");

    let retrieved = trips.get_trip(trip_request.request_id).await?;
    if let Some(trip) = retrieved {
        println!("   Retrieved trip status: {:?}", trip.status);
    }

    // ========================================================================
    // 5. Securement Status Monitoring
    // ========================================================================
    println!("\n5. Monitoring securement status...");

    // Start securing wheelchair
    securement.start_securing(vehicle_id).await?;

    let status = securement.get_status(vehicle_id).await?;
    println!("   Wheelchair detected: {}", status.wheelchair_detected);
    println!("   Overall status: {:?}", status.overall_status);
    println!("   Securement points: {}", status.securement_points.len());
    for point in &status.securement_points {
        println!("     - {} ({:?}): {:?}", point.point_id, point.position, point.status);
    }
    println!("   Ready to move: {}", status.ready_to_move);

    // Release securement at destination
    securement.release(vehicle_id).await?;
    println!("   Securement released");

    // ========================================================================
    // 6. Emergency Event Handling
    // ========================================================================
    println!("\n6. Demonstrating emergency handling...");

    let panic_location = GeoLocation {
        latitude: 37.5730,
        longitude: 126.9775,
        address: Some("Sejong-ro, Jongno-gu, Seoul".to_string()),
    };

    // Trigger panic button
    let event = emergency.panic_button(vehicle_id, panic_location).await?;

    println!("   Emergency reported: {:?}", event.event_type);
    println!("   Severity: {:?}", event.severity);
    println!("   Location: {:?}", event.location.address);

    if let Some(ref response) = event.response {
        println!("   Auto pulled over: {}", response.auto_pulled_over);
        println!("   Support contacted: {}", response.support_contacted);
        if let Some(eta) = response.eta_support_minutes {
            println!("   Support ETA: {:.0} minutes", eta);
        }
    }

    // Get all emergency events
    let events = emergency.get_events().await;
    println!("   Total emergency events recorded: {}", events.len());

    // ========================================================================
    // 7. HMI Configuration
    // ========================================================================
    println!("\n7. Configuring HMI...");

    let hmi = SimulatorHmiManager::new();

    // Apply blind-friendly config
    let blind_config = HmiConfig::for_blind();
    hmi.apply_config(vehicle_id, &blind_config).await?;
    println!("   Applied blind-friendly HMI config");

    let applied = hmi.get_config(vehicle_id).await?;
    if let Some(config) = applied {
        println!("   Config ID: {}", config.config_id);
        println!("   Visual enabled: {}", config.visual.as_ref().map(|v| v.enabled).unwrap_or(false));
        println!("   Audio enabled: {}", config.audio.as_ref().map(|a| a.enabled).unwrap_or(false));
    }

    // Reset config at end of trip
    hmi.reset_config(vehicle_id).await?;
    println!("   HMI config reset to default");

    // ========================================================================
    // 8. Trip Cancellation
    // ========================================================================
    println!("\n8. Demonstrating trip cancellation...");

    // Create another trip
    let cancel_request = TripRequest {
        request_id: Uuid::new_v4(),
        version: "1.0.0".to_string(),
        timestamp: Utc::now(),
        passenger_profile_id: Some(profile_id),
        trip: TripDetails {
            pickup: TripLocation {
                location: GeoLocation {
                    latitude: 37.5665,
                    longitude: 126.9780,
                    address: Some("Test pickup".to_string()),
                },
                notes: None,
                pickup_side: None,
                curb_to_curb: false,
            },
            dropoff: TripLocation {
                location: GeoLocation {
                    latitude: 37.5796,
                    longitude: 126.9770,
                    address: Some("Test dropoff".to_string()),
                },
                notes: None,
                pickup_side: None,
                curb_to_curb: false,
            },
            scheduled_time: None,
            flexibility_minutes: None,
        },
        accessibility_requirements: Some(AccessibilityRequirements::builder()
            .wheelchair_accessible()
            .build()),
        preferences: None,
    };

    let cancel_response = trips.request_trip(&cancel_request).await?;
    println!("   Created trip: {:?}", cancel_response.status);

    trips.cancel_trip(cancel_request.request_id).await?;
    println!("   Trip cancelled");

    let cancelled = trips.get_trip(cancel_request.request_id).await?;
    println!("   Trip after cancellation: {:?}", cancelled.map(|_| "found").unwrap_or("not found"));

    // ========================================================================
    // 9. Full Client Usage
    // ========================================================================
    println!("\n9. Using full simulator client...");

    let client = create_simulator_client();

    // Add vehicle to client's fleet
    client.fleet().add_vehicle(SimulatorFleetManager::create_sample_wheelchair_av()).await;

    // List vehicles
    let vehicles = client.fleet().list_vehicles().await?;
    println!("   Vehicles in fleet: {}", vehicles.len());

    // Save profile via client
    client.profiles().save_profile(&profile).await?;

    // List profiles
    let all_profiles = client.profiles().list_profiles().await?;
    println!("   Profiles stored: {}", all_profiles.len());

    println!("\n=== Trip Request Example Complete ===");
    Ok(())
}
