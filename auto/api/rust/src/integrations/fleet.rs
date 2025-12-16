//! Fleet/MaaS Integration
//!
//! Provides integration with fleet management systems,
//! GTFS-RT, and multi-modal trip planning.

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::error::Result;
use crate::types::*;

/// Reservation request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelchairReservation {
    /// Trip ID
    pub trip_id: Uuid,
    /// Passenger profile ID
    pub passenger_profile_id: Uuid,
    /// Wheelchair type
    pub wheelchair_type: WheelchairReservationType,
    /// Wheelchair dimensions
    pub dimensions: Option<ReservationDimensions>,
    /// Estimated weight (kg)
    pub weight_kg: Option<f32>,
    /// Companion traveling
    pub companion: bool,
    /// Assistance needed
    pub assistance_needed: bool,
}

/// Wheelchair type for reservation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WheelchairReservationType {
    Manual,
    Power,
    Scooter,
}

/// Dimensions for reservation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReservationDimensions {
    pub width_cm: f32,
    pub length_cm: f32,
}

/// Reservation response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReservationResponse {
    /// Reservation ID
    pub reservation_id: Uuid,
    /// Confirmed
    pub confirmed: bool,
    /// Assigned vehicle ID
    pub vehicle_id: Option<Uuid>,
    /// Boarding point
    pub boarding_point: Option<GeoLocation>,
    /// Boarding time
    pub boarding_time: Option<chrono::DateTime<chrono::Utc>>,
    /// Assigned wheelchair space (1 or 2)
    pub space_assigned: Option<u8>,
    /// Special instructions
    pub special_instructions: Option<String>,
}

/// Multi-modal trip plan
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiModalTripPlan {
    /// Journey overview
    pub journey: JourneyOverview,
    /// Trip legs
    pub legs: Vec<TripLeg>,
    /// Accessibility summary
    pub accessibility_summary: AccessibilitySummary,
}

/// Journey overview
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JourneyOverview {
    /// Origin
    pub origin: GeoLocation,
    /// Destination
    pub destination: GeoLocation,
    /// Total duration in minutes
    pub total_duration_minutes: u32,
    /// Total transfers
    pub total_transfers: u32,
    /// Total cost
    pub total_cost: Option<TripCost>,
}

/// Trip leg (single mode segment)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TripLeg {
    /// Leg ID
    pub leg_id: Uuid,
    /// Mode of transport
    pub mode: TransportMode,
    /// From location
    pub from: GeoLocation,
    /// To location
    pub to: GeoLocation,
    /// Duration in minutes
    pub duration_minutes: u32,
    /// Distance in km
    pub distance_km: Option<f32>,
    /// Mode-specific details
    pub details: LegDetails,
}

/// Transport mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransportMode {
    Walk,
    Wheelchair,
    AutonomousVehicle,
    Bus,
    Subway,
    Train,
    Tram,
    Ferry,
    Taxi,
}

/// Leg details
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum LegDetails {
    /// Walking/wheelchair segment
    #[serde(rename = "walk")]
    Walk(WalkDetails),
    /// Vehicle segment
    #[serde(rename = "vehicle")]
    Vehicle(VehicleLegDetails),
    /// Transit segment
    #[serde(rename = "transit")]
    Transit(TransitDetails),
}

/// Walk segment details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WalkDetails {
    /// Surface type
    pub surface_type: String,
    /// Maximum slope degrees
    pub max_slope_degrees: f32,
    /// Step-free route
    pub step_free: bool,
    /// Obstacles
    pub obstacles: Vec<RouteObstacle>,
}

/// Route obstacle
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteObstacle {
    /// Obstacle type
    pub obstacle_type: String,
    /// Location
    pub location: GeoLocation,
    /// Severity
    pub severity: ObstacleSeverity,
}

/// Obstacle severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObstacleSeverity {
    Low,
    Medium,
    High,
    Impassable,
}

/// Vehicle leg details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleLegDetails {
    /// Vehicle ID
    pub vehicle_id: Uuid,
    /// Accessibility features
    pub accessibility: VehicleAccessibility,
    /// Boarding time estimate
    pub boarding_time_minutes: u32,
    /// Securement type
    pub securement_type: SecurementType,
}

/// Vehicle accessibility info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleAccessibility {
    pub wheelchair_accessible: bool,
    pub ramp_available: bool,
    pub lift_available: bool,
    pub wheelchair_spaces: u8,
}

/// Transit segment details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransitDetails {
    /// Route ID
    pub route_id: String,
    /// Route name
    pub route_name: String,
    /// Departure stop
    pub departure_stop: TransitStop,
    /// Arrival stop
    pub arrival_stop: TransitStop,
    /// Number of stops
    pub stop_count: u32,
    /// Accessibility
    pub accessibility: TransitAccessibility,
}

/// Transit stop
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransitStop {
    /// Stop ID
    pub stop_id: String,
    /// Stop name
    pub stop_name: String,
    /// Location
    pub location: GeoLocation,
    /// Scheduled time
    pub scheduled_time: chrono::DateTime<chrono::Utc>,
    /// Platform/bay number
    pub platform: Option<String>,
}

/// Transit accessibility
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransitAccessibility {
    pub wheelchair_accessible: bool,
    pub level_boarding: bool,
    pub audio_announcements: bool,
    pub visual_displays: bool,
    pub tactile_guidance: bool,
}

/// Trip cost
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TripCost {
    /// Total amount
    pub amount: f64,
    /// Currency code
    pub currency: String,
    /// Breakdown by leg
    pub breakdown: Vec<CostBreakdown>,
    /// Discounts applied
    pub discounts: Vec<Discount>,
}

/// Cost breakdown
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CostBreakdown {
    pub leg_id: Uuid,
    pub amount: f64,
    pub description: String,
}

/// Discount
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Discount {
    pub discount_type: String,
    pub amount: f64,
    pub description: String,
}

/// Accessibility summary
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilitySummary {
    /// All segments accessible
    pub all_segments_accessible: bool,
    /// Wheelchair accessible throughout
    pub wheelchair_accessible: bool,
    /// Step-free throughout
    pub step_free: bool,
    /// Audio guidance available
    pub audio_guidance_available: bool,
    /// Issues found
    pub issues: Vec<AccessibilityIssue>,
}

/// Accessibility issue
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityIssue {
    /// Issue type
    pub issue_type: String,
    /// Location
    pub location: Option<GeoLocation>,
    /// Description
    pub description: String,
    /// Severity
    pub severity: ObstacleSeverity,
    /// Affected leg ID
    pub leg_id: Option<Uuid>,
}

/// GTFS-RT accessibility extension
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GtfsAccessibilityStatus {
    /// Vehicle ID
    pub vehicle_id: String,
    /// Wheelchair spaces available
    pub wheelchair_spaces_available: u8,
    /// Wheelchair spaces total
    pub wheelchair_spaces_total: u8,
    /// Entry type
    pub entry_type: EntryType,
    /// Ramp deployed
    pub ramp_deployed: bool,
    /// Lift operational
    pub lift_operational: bool,
}

/// Fleet integration handler
#[async_trait]
pub trait FleetIntegration: Send + Sync {
    /// Search for accessible vehicles
    async fn find_accessible_vehicles(
        &self,
        location: &GeoLocation,
        requirements: &AccessibilityRequirements,
    ) -> Result<Vec<AccessibleVehicle>>;

    /// Reserve wheelchair space
    async fn reserve_wheelchair_space(
        &self,
        reservation: &WheelchairReservation,
    ) -> Result<ReservationResponse>;

    /// Cancel reservation
    async fn cancel_reservation(&self, reservation_id: Uuid) -> Result<()>;

    /// Plan multi-modal trip
    async fn plan_trip(
        &self,
        origin: &GeoLocation,
        destination: &GeoLocation,
        requirements: &AccessibilityRequirements,
    ) -> Result<MultiModalTripPlan>;

    /// Get GTFS-RT accessibility status
    async fn gtfs_accessibility_status(
        &self,
        vehicle_id: &str,
    ) -> Result<Option<GtfsAccessibilityStatus>>;
}

/// Accessible vehicle info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibleVehicle {
    /// Vehicle ID
    pub vehicle_id: Uuid,
    /// Location
    pub location: GeoLocation,
    /// Distance in km
    pub distance_km: f32,
    /// ETA in minutes
    pub eta_minutes: u32,
    /// Accessibility features
    pub accessibility: VehicleAccessibility,
    /// Current status
    pub status: VehicleStatus,
}

/// Vehicle status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VehicleStatus {
    Available,
    EnRoute,
    Occupied,
    Maintenance,
    Offline,
}

/// Mock fleet integration for testing
pub struct MockFleetIntegration {
    vehicles: std::sync::Arc<tokio::sync::RwLock<Vec<AccessibleVehicle>>>,
    reservations: std::sync::Arc<tokio::sync::RwLock<Vec<ReservationResponse>>>,
}

impl MockFleetIntegration {
    /// Create new mock integration
    pub fn new() -> Self {
        Self {
            vehicles: std::sync::Arc::new(tokio::sync::RwLock::new(Vec::new())),
            reservations: std::sync::Arc::new(tokio::sync::RwLock::new(Vec::new())),
        }
    }

    /// Add mock vehicle
    pub async fn add_vehicle(&self, vehicle: AccessibleVehicle) {
        self.vehicles.write().await.push(vehicle);
    }
}

impl Default for MockFleetIntegration {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl FleetIntegration for MockFleetIntegration {
    async fn find_accessible_vehicles(
        &self,
        _location: &GeoLocation,
        _requirements: &AccessibilityRequirements,
    ) -> Result<Vec<AccessibleVehicle>> {
        Ok(self.vehicles.read().await.clone())
    }

    async fn reserve_wheelchair_space(
        &self,
        _reservation: &WheelchairReservation,
    ) -> Result<ReservationResponse> {
        let response = ReservationResponse {
            reservation_id: Uuid::new_v4(),
            confirmed: true,
            vehicle_id: Some(Uuid::new_v4()),
            boarding_point: None,
            boarding_time: Some(chrono::Utc::now() + chrono::Duration::minutes(10)),
            space_assigned: Some(1),
            special_instructions: Some("램프 입구에서 대기해 주세요.".to_string()),
        };

        self.reservations.write().await.push(response.clone());
        Ok(response)
    }

    async fn cancel_reservation(&self, reservation_id: Uuid) -> Result<()> {
        self.reservations.write().await.retain(|r| r.reservation_id != reservation_id);
        Ok(())
    }

    async fn plan_trip(
        &self,
        origin: &GeoLocation,
        destination: &GeoLocation,
        _requirements: &AccessibilityRequirements,
    ) -> Result<MultiModalTripPlan> {
        let leg_id = Uuid::new_v4();

        Ok(MultiModalTripPlan {
            journey: JourneyOverview {
                origin: origin.clone(),
                destination: destination.clone(),
                total_duration_minutes: 25,
                total_transfers: 0,
                total_cost: Some(TripCost {
                    amount: 3500.0,
                    currency: "KRW".to_string(),
                    breakdown: vec![],
                    discounts: vec![],
                }),
            },
            legs: vec![TripLeg {
                leg_id,
                mode: TransportMode::AutonomousVehicle,
                from: origin.clone(),
                to: destination.clone(),
                duration_minutes: 25,
                distance_km: Some(8.5),
                details: LegDetails::Vehicle(VehicleLegDetails {
                    vehicle_id: Uuid::new_v4(),
                    accessibility: VehicleAccessibility {
                        wheelchair_accessible: true,
                        ramp_available: true,
                        lift_available: false,
                        wheelchair_spaces: 2,
                    },
                    boarding_time_minutes: 3,
                    securement_type: SecurementType::FullAuto,
                }),
            }],
            accessibility_summary: AccessibilitySummary {
                all_segments_accessible: true,
                wheelchair_accessible: true,
                step_free: true,
                audio_guidance_available: true,
                issues: vec![],
            },
        })
    }

    async fn gtfs_accessibility_status(
        &self,
        vehicle_id: &str,
    ) -> Result<Option<GtfsAccessibilityStatus>> {
        Ok(Some(GtfsAccessibilityStatus {
            vehicle_id: vehicle_id.to_string(),
            wheelchair_spaces_available: 2,
            wheelchair_spaces_total: 2,
            entry_type: EntryType::Ramp,
            ramp_deployed: false,
            lift_operational: true,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reservation_response() {
        let response = ReservationResponse {
            reservation_id: Uuid::new_v4(),
            confirmed: true,
            vehicle_id: Some(Uuid::new_v4()),
            boarding_point: None,
            boarding_time: None,
            space_assigned: Some(1),
            special_instructions: None,
        };

        assert!(response.confirmed);
        assert_eq!(response.space_assigned, Some(1));
    }

    #[tokio::test]
    async fn test_mock_fleet_integration() {
        let integration = MockFleetIntegration::new();

        let reservation = WheelchairReservation {
            trip_id: Uuid::new_v4(),
            passenger_profile_id: Uuid::new_v4(),
            wheelchair_type: WheelchairReservationType::Power,
            dimensions: None,
            weight_kg: None,
            companion: false,
            assistance_needed: true,
        };

        let response = integration.reserve_wheelchair_space(&reservation).await.unwrap();
        assert!(response.confirmed);
        assert!(response.vehicle_id.is_some());
    }

    #[tokio::test]
    async fn test_trip_planning() {
        let integration = MockFleetIntegration::new();

        let origin = GeoLocation {
            latitude: 37.5665,
            longitude: 126.9780,
            address: Some("서울역".to_string()),
        };

        let destination = GeoLocation {
            latitude: 37.4979,
            longitude: 127.0276,
            address: Some("강남역".to_string()),
        };

        let requirements = AccessibilityRequirements::default();

        let plan = integration.plan_trip(&origin, &destination, &requirements).await.unwrap();
        assert!(plan.accessibility_summary.wheelchair_accessible);
        assert_eq!(plan.legs.len(), 1);
    }
}
