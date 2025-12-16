//! Core autonomous vehicle accessibility functionality

use async_trait::async_trait;
use chrono::Utc;
use uuid::Uuid;

use crate::error::Result;
use crate::types::*;

/// Trait for fleet management operations
#[async_trait]
pub trait FleetManager: Send + Sync {
    /// Find an accessible vehicle matching the requirements
    async fn find_vehicle(
        &self,
        requirements: &AccessibilityRequirements,
        location: &GeoLocation,
    ) -> Result<Option<VehicleCapabilities>>;

    /// Get vehicle by ID
    async fn get_vehicle(&self, vehicle_id: Uuid) -> Result<Option<VehicleCapabilities>>;

    /// Get all available vehicles
    async fn list_vehicles(&self) -> Result<Vec<VehicleCapabilities>>;

    /// Dispatch vehicle to pickup location
    async fn dispatch_vehicle(
        &self,
        vehicle_id: Uuid,
        pickup: &GeoLocation,
        dropoff: &GeoLocation,
    ) -> Result<VehicleAssignment>;
}

/// Trait for passenger profile management
#[async_trait]
pub trait ProfileManager: Send + Sync {
    /// Save a passenger profile
    async fn save_profile(&self, profile: &PassengerProfile) -> Result<()>;

    /// Get a passenger profile by ID
    async fn get_profile(&self, profile_id: Uuid) -> Result<Option<PassengerProfile>>;

    /// Delete a passenger profile
    async fn delete_profile(&self, profile_id: Uuid) -> Result<()>;

    /// List all profiles
    async fn list_profiles(&self) -> Result<Vec<PassengerProfile>>;
}

/// Trait for trip management
#[async_trait]
pub trait TripManager: Send + Sync {
    /// Request a new trip
    async fn request_trip(&self, request: &TripRequest) -> Result<TripResponse>;

    /// Get trip status
    async fn get_trip(&self, request_id: Uuid) -> Result<Option<TripResponse>>;

    /// Cancel a trip
    async fn cancel_trip(&self, request_id: Uuid) -> Result<()>;
}

/// Trait for HMI management
#[async_trait]
pub trait HmiManager: Send + Sync {
    /// Apply HMI configuration to a vehicle
    async fn apply_config(&self, vehicle_id: Uuid, config: &HmiConfig) -> Result<()>;

    /// Get current HMI configuration
    async fn get_config(&self, vehicle_id: Uuid) -> Result<Option<HmiConfig>>;

    /// Reset HMI to defaults
    async fn reset_config(&self, vehicle_id: Uuid) -> Result<()>;
}

/// Trait for securement monitoring
#[async_trait]
pub trait SecurementMonitor: Send + Sync {
    /// Get current securement status
    async fn get_status(&self, vehicle_id: Uuid) -> Result<SecurementStatus>;

    /// Start securing wheelchair
    async fn start_securing(&self, vehicle_id: Uuid) -> Result<()>;

    /// Release securement
    async fn release(&self, vehicle_id: Uuid) -> Result<()>;

    /// Check if ready to move
    async fn is_ready_to_move(&self, vehicle_id: Uuid) -> Result<bool>;
}

/// Trait for emergency handling
#[async_trait]
pub trait EmergencyHandler: Send + Sync {
    /// Report an emergency event
    async fn report_emergency(&self, event: &EmergencyEvent) -> Result<()>;

    /// Trigger panic button
    async fn panic_button(&self, vehicle_id: Uuid, location: GeoLocation) -> Result<EmergencyEvent>;

    /// Request pull over
    async fn pull_over(&self, vehicle_id: Uuid) -> Result<()>;

    /// Contact support
    async fn contact_support(&self, vehicle_id: Uuid) -> Result<()>;
}

/// Main WIA Auto client
pub struct WiaAutoClient<F, P, T, H, S, E>
where
    F: FleetManager,
    P: ProfileManager,
    T: TripManager,
    H: HmiManager,
    S: SecurementMonitor,
    E: EmergencyHandler,
{
    fleet: F,
    profiles: P,
    trips: T,
    hmi: H,
    securement: S,
    emergency: E,
}

impl<F, P, T, H, S, E> WiaAutoClient<F, P, T, H, S, E>
where
    F: FleetManager,
    P: ProfileManager,
    T: TripManager,
    H: HmiManager,
    S: SecurementMonitor,
    E: EmergencyHandler,
{
    /// Create a new WIA Auto client
    pub fn new(
        fleet: F,
        profiles: P,
        trips: T,
        hmi: H,
        securement: S,
        emergency: E,
    ) -> Self {
        Self {
            fleet,
            profiles,
            trips,
            hmi,
            securement,
            emergency,
        }
    }

    /// Get the fleet manager
    pub fn fleet(&self) -> &F {
        &self.fleet
    }

    /// Get the profile manager
    pub fn profiles(&self) -> &P {
        &self.profiles
    }

    /// Get the trip manager
    pub fn trips(&self) -> &T {
        &self.trips
    }

    /// Get the HMI manager
    pub fn hmi(&self) -> &H {
        &self.hmi
    }

    /// Get the securement monitor
    pub fn securement(&self) -> &S {
        &self.securement
    }

    /// Get the emergency handler
    pub fn emergency(&self) -> &E {
        &self.emergency
    }

    /// Request an accessible trip
    pub async fn request_accessible_trip(
        &self,
        profile_id: Option<Uuid>,
        pickup: GeoLocation,
        dropoff: GeoLocation,
        requirements: AccessibilityRequirements,
    ) -> Result<TripResponse> {
        let request = TripRequest {
            request_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            timestamp: Utc::now(),
            passenger_profile_id: profile_id,
            trip: TripDetails {
                pickup: TripLocation {
                    location: pickup,
                    notes: None,
                    pickup_side: Some(PickupSide::SameSide),
                    curb_to_curb: true,
                },
                dropoff: TripLocation {
                    location: dropoff,
                    notes: None,
                    pickup_side: None,
                    curb_to_curb: false,
                },
                scheduled_time: None,
                flexibility_minutes: None,
            },
            accessibility_requirements: Some(requirements),
            preferences: Some(TripPreferences {
                minimize_walking: true,
                audio_guidance: true,
                visual_guidance: true,
                haptic_feedback: true,
                quiet_ride: false,
            }),
        };

        self.trips.request_trip(&request).await
    }

    /// Handle emergency situation
    pub async fn handle_emergency(
        &self,
        vehicle_id: Uuid,
        event_type: EmergencyEventType,
        severity: EmergencySeverity,
        location: GeoLocation,
    ) -> Result<EmergencyEvent> {
        let event = EmergencyEvent {
            event_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            timestamp: Utc::now(),
            vehicle_id: Some(vehicle_id),
            trip_id: None,
            event_type,
            severity,
            location,
            passenger: None,
            vehicle_status: None,
            response: Some(EmergencyResponse {
                auto_pulled_over: true,
                support_contacted: true,
                emergency_services_called: severity == EmergencySeverity::Critical,
                eta_support_minutes: Some(5.0),
            }),
        };

        self.emergency.report_emergency(&event).await?;
        Ok(event)
    }
}

/// Create a message envelope
pub fn create_message<T: serde::Serialize>(
    source: MessageEndpoint,
    destination: MessageEndpoint,
    message_type: MessageType,
    payload: &T,
    priority: MessagePriority,
) -> Result<MessageEnvelope> {
    let payload_value = serde_json::to_value(payload)?;

    Ok(MessageEnvelope {
        wia_auto: WiaAutoMessage {
            version: "1.0.0".to_string(),
            message_id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source,
            destination,
            message_type,
            correlation_id: None,
            priority,
            payload: payload_value,
        },
    })
}

/// Parse a message envelope payload
pub fn parse_message_payload<T: serde::de::DeserializeOwned>(
    envelope: &MessageEnvelope,
) -> Result<T> {
    let payload: T = serde_json::from_value(envelope.wia_auto.payload.clone())?;
    Ok(payload)
}
