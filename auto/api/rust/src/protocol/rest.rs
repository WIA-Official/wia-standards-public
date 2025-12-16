//! REST API client for WIA Auto

use async_trait::async_trait;
use chrono::Utc;
use serde::Serialize;
use uuid::Uuid;

use crate::error::Result;
use crate::types::*;
use super::ClientConfig;

/// REST API client trait
#[async_trait]
pub trait RestClient: Send + Sync {
    /// Create a passenger profile
    async fn create_profile(&self, profile: &PassengerProfile) -> Result<CreateProfileResponse>;

    /// Get a passenger profile
    async fn get_profile(&self, profile_id: Uuid) -> Result<Option<PassengerProfile>>;

    /// Update a passenger profile
    async fn update_profile(&self, profile: &PassengerProfile) -> Result<PassengerProfile>;

    /// Delete a passenger profile
    async fn delete_profile(&self, profile_id: Uuid) -> Result<()>;

    /// Request a trip
    async fn request_trip(&self, request: &TripRequest) -> Result<TripResponse>;

    /// Get trip status
    async fn get_trip(&self, request_id: Uuid) -> Result<Option<TripResponse>>;

    /// Cancel a trip
    async fn cancel_trip(&self, request_id: Uuid) -> Result<()>;

    /// Find available vehicles
    async fn find_vehicles(&self, query: &FindVehiclesQuery) -> Result<Vec<VehicleMatch>>;

    /// Get vehicle details
    async fn get_vehicle(&self, vehicle_id: Uuid) -> Result<Option<VehicleCapabilities>>;

    /// Report emergency
    async fn report_emergency(&self, event: &EmergencyReport) -> Result<EmergencyReportResponse>;
}

/// Response from create profile
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct CreateProfileResponse {
    pub profile_id: Uuid,
    pub version: String,
    pub created_at: chrono::DateTime<Utc>,
}

/// Query parameters for finding vehicles
#[derive(Debug, Clone, Default)]
pub struct FindVehiclesQuery {
    pub latitude: f64,
    pub longitude: f64,
    pub radius_km: Option<f64>,
    pub wheelchair: Option<bool>,
    pub ramp: Option<bool>,
    pub lift: Option<bool>,
    pub service_animal: Option<bool>,
}

/// Vehicle match result
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct VehicleMatch {
    pub vehicle_id: Uuid,
    pub vehicle_info: VehicleInfo,
    pub location: GeoLocation,
    pub distance_km: f64,
    pub eta_minutes: f64,
    pub match_score: u8,
    pub accessibility: AccessibilityMatch,
}

/// Emergency report request
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct EmergencyReport {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vehicle_id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trip_id: Option<Uuid>,
    pub event_type: EmergencyEventType,
    pub severity: EmergencySeverity,
    pub location: GeoLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub passenger: Option<EmergencyPassengerInfo>,
}

/// Emergency report response
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct EmergencyReportResponse {
    pub event_id: Uuid,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub response: Option<EmergencyResponse>,
}

/// Mock REST client for testing
#[derive(Debug, Clone)]
pub struct MockRestClient {
    config: ClientConfig,
    profiles: std::sync::Arc<tokio::sync::RwLock<std::collections::HashMap<Uuid, PassengerProfile>>>,
    trips: std::sync::Arc<tokio::sync::RwLock<std::collections::HashMap<Uuid, TripResponse>>>,
    vehicles: std::sync::Arc<tokio::sync::RwLock<std::collections::HashMap<Uuid, VehicleCapabilities>>>,
}

impl MockRestClient {
    /// Create a new mock REST client
    pub fn new(config: ClientConfig) -> Self {
        Self {
            config,
            profiles: std::sync::Arc::new(tokio::sync::RwLock::new(std::collections::HashMap::new())),
            trips: std::sync::Arc::new(tokio::sync::RwLock::new(std::collections::HashMap::new())),
            vehicles: std::sync::Arc::new(tokio::sync::RwLock::new(std::collections::HashMap::new())),
        }
    }

    /// Add a vehicle to the mock fleet
    pub async fn add_vehicle(&self, vehicle: VehicleCapabilities) {
        self.vehicles.write().await.insert(vehicle.vehicle_id, vehicle);
    }

    /// Get configuration
    pub fn config(&self) -> &ClientConfig {
        &self.config
    }
}

impl Default for MockRestClient {
    fn default() -> Self {
        Self::new(ClientConfig::mock())
    }
}

#[async_trait]
impl RestClient for MockRestClient {
    async fn create_profile(&self, profile: &PassengerProfile) -> Result<CreateProfileResponse> {
        self.profiles.write().await.insert(profile.profile_id, profile.clone());
        Ok(CreateProfileResponse {
            profile_id: profile.profile_id,
            version: profile.version.clone(),
            created_at: Utc::now(),
        })
    }

    async fn get_profile(&self, profile_id: Uuid) -> Result<Option<PassengerProfile>> {
        Ok(self.profiles.read().await.get(&profile_id).cloned())
    }

    async fn update_profile(&self, profile: &PassengerProfile) -> Result<PassengerProfile> {
        self.profiles.write().await.insert(profile.profile_id, profile.clone());
        Ok(profile.clone())
    }

    async fn delete_profile(&self, profile_id: Uuid) -> Result<()> {
        self.profiles.write().await.remove(&profile_id);
        Ok(())
    }

    async fn request_trip(&self, request: &TripRequest) -> Result<TripResponse> {
        // Find a matching vehicle
        let vehicles = self.vehicles.read().await;
        let requirements = request.accessibility_requirements.clone().unwrap_or_default();

        let matching_vehicle = vehicles.values().find(|v| {
            if requirements.wheelchair_accessible {
                v.accessibility_features.interior
                    .as_ref()
                    .map(|i| i.wheelchair_spaces > 0)
                    .unwrap_or(false)
            } else {
                true
            }
        });

        let (status, vehicle, accessibility_match) = match matching_vehicle {
            Some(v) => (
                TripStatus::Confirmed,
                Some(VehicleAssignment {
                    vehicle_id: v.vehicle_id,
                    eta_minutes: 5.0,
                    distance_km: 2.0,
                }),
                Some(AccessibilityMatch {
                    wheelchair_accessible: v.accessibility_features.interior
                        .as_ref()
                        .map(|i| i.wheelchair_spaces > 0)
                        .unwrap_or(false),
                    ramp_available: v.accessibility_features.entry
                        .as_ref()
                        .and_then(|e| e.ramp.as_ref())
                        .map(|r| r.available)
                        .unwrap_or(false),
                    lift_available: v.accessibility_features.entry
                        .as_ref()
                        .and_then(|e| e.lift.as_ref())
                        .map(|l| l.available)
                        .unwrap_or(false),
                    service_animal_ok: v.capacity
                        .as_ref()
                        .map(|c| c.service_animals)
                        .unwrap_or(false),
                    modalities_supported: vec![],
                    match_score: 95,
                }),
            ),
            None => (TripStatus::NoVehicle, None, None),
        };

        let response = TripResponse {
            response_id: Uuid::new_v4(),
            request_id: request.request_id,
            version: "1.0.0".to_string(),
            timestamp: Utc::now(),
            status,
            vehicle,
            accessibility_match,
            wayfinding: Some(WayfindingInfo {
                pickup_instructions: Some("Vehicle will arrive at the curb.".to_string()),
                audio_guidance_available: true,
                find_vehicle_features: vec![
                    FindVehicleFeature::Horn,
                    FindVehicleFeature::Lights,
                ],
                braille_identifier: Some("WIA-AUTO".to_string()),
            }),
        };

        self.trips.write().await.insert(request.request_id, response.clone());
        Ok(response)
    }

    async fn get_trip(&self, request_id: Uuid) -> Result<Option<TripResponse>> {
        Ok(self.trips.read().await.get(&request_id).cloned())
    }

    async fn cancel_trip(&self, request_id: Uuid) -> Result<()> {
        self.trips.write().await.remove(&request_id);
        Ok(())
    }

    async fn find_vehicles(&self, query: &FindVehiclesQuery) -> Result<Vec<VehicleMatch>> {
        let vehicles = self.vehicles.read().await;
        let matches: Vec<VehicleMatch> = vehicles.values()
            .filter(|v| {
                // Filter by accessibility requirements
                if query.wheelchair.unwrap_or(false) {
                    if !v.accessibility_features.interior
                        .as_ref()
                        .map(|i| i.wheelchair_spaces > 0)
                        .unwrap_or(false) {
                        return false;
                    }
                }
                if query.ramp.unwrap_or(false) {
                    if !v.accessibility_features.entry
                        .as_ref()
                        .and_then(|e| e.ramp.as_ref())
                        .map(|r| r.available)
                        .unwrap_or(false) {
                        return false;
                    }
                }
                true
            })
            .map(|v| VehicleMatch {
                vehicle_id: v.vehicle_id,
                vehicle_info: v.vehicle_info.clone(),
                location: GeoLocation {
                    latitude: query.latitude + 0.001, // Simulated nearby
                    longitude: query.longitude + 0.001,
                    address: None,
                },
                distance_km: 2.0,
                eta_minutes: 5.0,
                match_score: 95,
                accessibility: AccessibilityMatch {
                    wheelchair_accessible: v.accessibility_features.interior
                        .as_ref()
                        .map(|i| i.wheelchair_spaces > 0)
                        .unwrap_or(false),
                    ramp_available: v.accessibility_features.entry
                        .as_ref()
                        .and_then(|e| e.ramp.as_ref())
                        .map(|r| r.available)
                        .unwrap_or(false),
                    lift_available: v.accessibility_features.entry
                        .as_ref()
                        .and_then(|e| e.lift.as_ref())
                        .map(|l| l.available)
                        .unwrap_or(false),
                    service_animal_ok: v.capacity
                        .as_ref()
                        .map(|c| c.service_animals)
                        .unwrap_or(false),
                    modalities_supported: vec![],
                    match_score: 95,
                },
            })
            .collect();

        Ok(matches)
    }

    async fn get_vehicle(&self, vehicle_id: Uuid) -> Result<Option<VehicleCapabilities>> {
        Ok(self.vehicles.read().await.get(&vehicle_id).cloned())
    }

    async fn report_emergency(&self, event: &EmergencyReport) -> Result<EmergencyReportResponse> {
        Ok(EmergencyReportResponse {
            event_id: Uuid::new_v4(),
            response: Some(EmergencyResponse {
                auto_pulled_over: true,
                support_contacted: true,
                emergency_services_called: event.severity == EmergencySeverity::Critical,
                eta_support_minutes: Some(3.0),
            }),
        })
    }
}

/// HTTP method enumeration
#[derive(Debug, Clone, Copy)]
pub enum HttpMethod {
    Get,
    Post,
    Put,
    Delete,
}

/// Request builder for REST API calls
#[derive(Debug)]
pub struct RequestBuilder {
    method: HttpMethod,
    path: String,
    query: Vec<(String, String)>,
    body: Option<serde_json::Value>,
}

impl RequestBuilder {
    /// Create a new GET request
    pub fn get(path: impl Into<String>) -> Self {
        Self {
            method: HttpMethod::Get,
            path: path.into(),
            query: vec![],
            body: None,
        }
    }

    /// Create a new POST request
    pub fn post(path: impl Into<String>) -> Self {
        Self {
            method: HttpMethod::Post,
            path: path.into(),
            query: vec![],
            body: None,
        }
    }

    /// Create a new PUT request
    pub fn put(path: impl Into<String>) -> Self {
        Self {
            method: HttpMethod::Put,
            path: path.into(),
            query: vec![],
            body: None,
        }
    }

    /// Create a new DELETE request
    pub fn delete(path: impl Into<String>) -> Self {
        Self {
            method: HttpMethod::Delete,
            path: path.into(),
            query: vec![],
            body: None,
        }
    }

    /// Add a query parameter
    pub fn query(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.query.push((key.into(), value.into()));
        self
    }

    /// Set the request body
    pub fn body<T: Serialize>(mut self, body: &T) -> Self {
        self.body = serde_json::to_value(body).ok();
        self
    }

    /// Get the HTTP method
    pub fn method(&self) -> HttpMethod {
        self.method
    }

    /// Get the path
    pub fn path(&self) -> &str {
        &self.path
    }

    /// Get query parameters
    pub fn query_params(&self) -> &[(String, String)] {
        &self.query
    }

    /// Get the body
    pub fn body_json(&self) -> Option<&serde_json::Value> {
        self.body.as_ref()
    }
}
