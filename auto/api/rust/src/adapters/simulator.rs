//! Simulator adapter for testing and development

use async_trait::async_trait;
use chrono::Utc;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

use crate::core::*;
use crate::error::{Error, Result};
use crate::types::*;

/// In-memory simulator for fleet management
#[derive(Debug, Clone)]
pub struct SimulatorFleetManager {
    vehicles: Arc<RwLock<HashMap<Uuid, VehicleCapabilities>>>,
}

impl SimulatorFleetManager {
    /// Create a new simulator fleet manager
    pub fn new() -> Self {
        Self {
            vehicles: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Add a vehicle to the fleet
    pub async fn add_vehicle(&self, vehicle: VehicleCapabilities) {
        self.vehicles.write().await.insert(vehicle.vehicle_id, vehicle);
    }

    /// Remove a vehicle from the fleet
    pub async fn remove_vehicle(&self, vehicle_id: Uuid) {
        self.vehicles.write().await.remove(&vehicle_id);
    }

    /// Create a sample wheelchair-accessible vehicle
    pub fn create_sample_wheelchair_av() -> VehicleCapabilities {
        VehicleCapabilities {
            vehicle_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            vehicle_info: VehicleInfo {
                make: "WIA".to_string(),
                model: "AccessAV Pro".to_string(),
                year: Some(2025),
                sae_level: SAELevel::Level4,
                license_plate: Some("WIA-001".to_string()),
            },
            accessibility_features: AccessibilityFeatures {
                entry: Some(EntryFeatures {
                    ramp: Some(RampFeatures {
                        available: true,
                        ramp_type: Some(RampType::Deploy),
                        max_weight_kg: Some(400.0),
                        width_cm: Some(100.0),
                        auto_deploy: true,
                    }),
                    lift: Some(LiftFeatures {
                        available: true,
                        lift_type: Some(LiftType::Platform),
                        max_weight_kg: Some(400.0),
                        width_cm: Some(90.0),
                    }),
                    doors: Some(DoorFeatures {
                        door_type: Some(DoorType::Slide),
                        width_cm: Some(120.0),
                        auto_open: true,
                        low_step: true,
                        step_height_cm: Some(5.0),
                    }),
                }),
                interior: Some(InteriorFeatures {
                    wheelchair_spaces: 2,
                    wheelchair_securement: Some(SecurementFeatures {
                        available: true,
                        securement_type: Some(SecurementType::FullAuto),
                        max_width_cm: Some(85.0),
                        max_length_cm: Some(140.0),
                        max_weight_kg: Some(400.0),
                    }),
                    transfer_seat: true,
                    lowered_floor: true,
                    floor_height_cm: Some(15.0),
                    headroom_cm: Some(160.0),
                }),
                hmi: Some(HmiFeatures {
                    screen: Some(ScreenFeatures {
                        available: true,
                        size_inches: Some(15.0),
                        touch: true,
                        high_contrast: true,
                        adjustable_brightness: true,
                    }),
                    audio: Some(AudioFeatures {
                        tts: true,
                        speech_recognition: true,
                        languages: vec![
                            "en".to_string(),
                            "ko".to_string(),
                            "es".to_string(),
                            "zh".to_string(),
                        ],
                        volume_control: true,
                        chimes: true,
                    }),
                    haptic: Some(HapticFeatures {
                        vibration: true,
                        force_feedback: true,
                    }),
                    physical_controls: Some(PhysicalControlFeatures {
                        braille_labels: true,
                        tactile_buttons: true,
                        panic_button: true,
                        pull_over_button: true,
                    }),
                }),
                communication: Some(CommunicationFeatures {
                    live_support: true,
                    video_call: true,
                    text_chat: true,
                    sign_language_support: true,
                }),
            },
            capacity: Some(VehicleCapacity {
                total_passengers: 6,
                wheelchair_passengers: 2,
                service_animals: true,
            }),
        }
    }

    /// Create a sample standard AV (non-wheelchair accessible)
    pub fn create_sample_standard_av() -> VehicleCapabilities {
        VehicleCapabilities {
            vehicle_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            vehicle_info: VehicleInfo {
                make: "WIA".to_string(),
                model: "AccessAV Lite".to_string(),
                year: Some(2025),
                sae_level: SAELevel::Level4,
                license_plate: Some("WIA-002".to_string()),
            },
            accessibility_features: AccessibilityFeatures {
                entry: Some(EntryFeatures {
                    ramp: None,
                    lift: None,
                    doors: Some(DoorFeatures {
                        door_type: Some(DoorType::Swing),
                        width_cm: Some(90.0),
                        auto_open: true,
                        low_step: true,
                        step_height_cm: Some(20.0),
                    }),
                }),
                interior: Some(InteriorFeatures {
                    wheelchair_spaces: 0,
                    wheelchair_securement: None,
                    transfer_seat: false,
                    lowered_floor: false,
                    floor_height_cm: Some(40.0),
                    headroom_cm: Some(140.0),
                }),
                hmi: Some(HmiFeatures {
                    screen: Some(ScreenFeatures {
                        available: true,
                        size_inches: Some(10.0),
                        touch: true,
                        high_contrast: true,
                        adjustable_brightness: true,
                    }),
                    audio: Some(AudioFeatures {
                        tts: true,
                        speech_recognition: true,
                        languages: vec!["en".to_string(), "ko".to_string()],
                        volume_control: true,
                        chimes: true,
                    }),
                    haptic: Some(HapticFeatures {
                        vibration: true,
                        force_feedback: false,
                    }),
                    physical_controls: Some(PhysicalControlFeatures {
                        braille_labels: true,
                        tactile_buttons: true,
                        panic_button: true,
                        pull_over_button: true,
                    }),
                }),
                communication: Some(CommunicationFeatures {
                    live_support: true,
                    video_call: false,
                    text_chat: true,
                    sign_language_support: false,
                }),
            },
            capacity: Some(VehicleCapacity {
                total_passengers: 4,
                wheelchair_passengers: 0,
                service_animals: true,
            }),
        }
    }
}

impl Default for SimulatorFleetManager {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl FleetManager for SimulatorFleetManager {
    async fn find_vehicle(
        &self,
        requirements: &AccessibilityRequirements,
        _location: &GeoLocation,
    ) -> Result<Option<VehicleCapabilities>> {
        let vehicles = self.vehicles.read().await;
        let vehicle_list: Vec<_> = vehicles.values().cloned().collect();

        Ok(find_best_match(requirements, &vehicle_list).map(|(v, _)| v))
    }

    async fn get_vehicle(&self, vehicle_id: Uuid) -> Result<Option<VehicleCapabilities>> {
        Ok(self.vehicles.read().await.get(&vehicle_id).cloned())
    }

    async fn list_vehicles(&self) -> Result<Vec<VehicleCapabilities>> {
        Ok(self.vehicles.read().await.values().cloned().collect())
    }

    async fn dispatch_vehicle(
        &self,
        vehicle_id: Uuid,
        _pickup: &GeoLocation,
        _dropoff: &GeoLocation,
    ) -> Result<VehicleAssignment> {
        // Verify vehicle exists
        if !self.vehicles.read().await.contains_key(&vehicle_id) {
            return Err(Error::vehicle_not_found(vehicle_id.to_string()));
        }

        // Simulate dispatch
        Ok(VehicleAssignment {
            vehicle_id,
            eta_minutes: 5.0, // Simulated ETA
            distance_km: 2.0, // Simulated distance
        })
    }
}

/// In-memory simulator for profile management
#[derive(Debug, Clone)]
pub struct SimulatorProfileManager {
    profiles: Arc<RwLock<HashMap<Uuid, PassengerProfile>>>,
}

impl SimulatorProfileManager {
    pub fn new() -> Self {
        Self {
            profiles: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

impl Default for SimulatorProfileManager {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl ProfileManager for SimulatorProfileManager {
    async fn save_profile(&self, profile: &PassengerProfile) -> Result<()> {
        self.profiles
            .write()
            .await
            .insert(profile.profile_id, profile.clone());
        Ok(())
    }

    async fn get_profile(&self, profile_id: Uuid) -> Result<Option<PassengerProfile>> {
        Ok(self.profiles.read().await.get(&profile_id).cloned())
    }

    async fn delete_profile(&self, profile_id: Uuid) -> Result<()> {
        self.profiles.write().await.remove(&profile_id);
        Ok(())
    }

    async fn list_profiles(&self) -> Result<Vec<PassengerProfile>> {
        Ok(self.profiles.read().await.values().cloned().collect())
    }
}

/// In-memory simulator for trip management
#[derive(Debug, Clone)]
pub struct SimulatorTripManager {
    fleet: SimulatorFleetManager,
    trips: Arc<RwLock<HashMap<Uuid, TripResponse>>>,
}

impl SimulatorTripManager {
    pub fn new(fleet: SimulatorFleetManager) -> Self {
        Self {
            fleet,
            trips: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

#[async_trait]
impl TripManager for SimulatorTripManager {
    async fn request_trip(&self, request: &TripRequest) -> Result<TripResponse> {
        let requirements = request
            .accessibility_requirements
            .clone()
            .unwrap_or_default();

        // Find a matching vehicle
        let vehicle = self
            .fleet
            .find_vehicle(&requirements, &request.trip.pickup.location)
            .await?;

        let (status, vehicle_assignment, accessibility_match) = match vehicle {
            Some(v) => {
                let assignment = self
                    .fleet
                    .dispatch_vehicle(
                        v.vehicle_id,
                        &request.trip.pickup.location,
                        &request.trip.dropoff.location,
                    )
                    .await?;

                let match_result = match_vehicle(&requirements, &v);

                (
                    TripStatus::Confirmed,
                    Some(assignment),
                    Some(match_result.details),
                )
            }
            None => (TripStatus::NoVehicle, None, None),
        };

        let response = TripResponse {
            response_id: Uuid::new_v4(),
            request_id: request.request_id,
            version: "1.0.0".to_string(),
            timestamp: Utc::now(),
            status,
            vehicle: vehicle_assignment,
            accessibility_match,
            wayfinding: Some(WayfindingInfo {
                pickup_instructions: Some("The vehicle will arrive at the curb.".to_string()),
                audio_guidance_available: true,
                find_vehicle_features: vec![
                    FindVehicleFeature::Horn,
                    FindVehicleFeature::Lights,
                    FindVehicleFeature::Melody,
                ],
                braille_identifier: Some("WIA-AV".to_string()),
            }),
        };

        self.trips
            .write()
            .await
            .insert(request.request_id, response.clone());

        Ok(response)
    }

    async fn get_trip(&self, request_id: Uuid) -> Result<Option<TripResponse>> {
        Ok(self.trips.read().await.get(&request_id).cloned())
    }

    async fn cancel_trip(&self, request_id: Uuid) -> Result<()> {
        self.trips.write().await.remove(&request_id);
        Ok(())
    }
}

/// Simulator HMI manager
#[derive(Debug, Clone)]
pub struct SimulatorHmiManager {
    configs: Arc<RwLock<HashMap<Uuid, HmiConfig>>>,
}

impl SimulatorHmiManager {
    pub fn new() -> Self {
        Self {
            configs: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

impl Default for SimulatorHmiManager {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl HmiManager for SimulatorHmiManager {
    async fn apply_config(&self, vehicle_id: Uuid, config: &HmiConfig) -> Result<()> {
        self.configs.write().await.insert(vehicle_id, config.clone());
        Ok(())
    }

    async fn get_config(&self, vehicle_id: Uuid) -> Result<Option<HmiConfig>> {
        Ok(self.configs.read().await.get(&vehicle_id).cloned())
    }

    async fn reset_config(&self, vehicle_id: Uuid) -> Result<()> {
        self.configs.write().await.remove(&vehicle_id);
        Ok(())
    }
}

/// Simulator securement monitor
#[derive(Debug, Clone)]
pub struct SimulatorSecurementMonitor {
    status: Arc<RwLock<HashMap<Uuid, SecurementStatus>>>,
}

impl SimulatorSecurementMonitor {
    pub fn new() -> Self {
        Self {
            status: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

impl Default for SimulatorSecurementMonitor {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl SecurementMonitor for SimulatorSecurementMonitor {
    async fn get_status(&self, vehicle_id: Uuid) -> Result<SecurementStatus> {
        self.status
            .read()
            .await
            .get(&vehicle_id)
            .cloned()
            .ok_or_else(|| Error::vehicle_not_found(vehicle_id.to_string()))
    }

    async fn start_securing(&self, vehicle_id: Uuid) -> Result<()> {
        let status = SecurementStatus {
            status_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            timestamp: Utc::now(),
            vehicle_id: Some(vehicle_id),
            securement_points: vec![
                SecurementPoint {
                    point_id: "FL".to_string(),
                    position: SecurementPosition::FrontLeft,
                    status: SecurementPointStatus::Engaging,
                    force_newtons: Some(0.0),
                    locked: false,
                },
                SecurementPoint {
                    point_id: "FR".to_string(),
                    position: SecurementPosition::FrontRight,
                    status: SecurementPointStatus::Engaging,
                    force_newtons: Some(0.0),
                    locked: false,
                },
            ],
            overall_status: OverallSecurementStatus::Securing,
            wheelchair_detected: true,
            wheelchair_type: Some(WheelchairType::Power),
            safety_check_passed: false,
            ready_to_move: false,
            error_message: None,
        };

        self.status.write().await.insert(vehicle_id, status);
        Ok(())
    }

    async fn release(&self, vehicle_id: Uuid) -> Result<()> {
        self.status.write().await.remove(&vehicle_id);
        Ok(())
    }

    async fn is_ready_to_move(&self, vehicle_id: Uuid) -> Result<bool> {
        Ok(self
            .status
            .read()
            .await
            .get(&vehicle_id)
            .map(|s| s.ready_to_move)
            .unwrap_or(false))
    }
}

/// Simulator emergency handler
#[derive(Debug, Clone)]
pub struct SimulatorEmergencyHandler {
    events: Arc<RwLock<Vec<EmergencyEvent>>>,
}

impl SimulatorEmergencyHandler {
    pub fn new() -> Self {
        Self {
            events: Arc::new(RwLock::new(Vec::new())),
        }
    }

    pub async fn get_events(&self) -> Vec<EmergencyEvent> {
        self.events.read().await.clone()
    }
}

impl Default for SimulatorEmergencyHandler {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl EmergencyHandler for SimulatorEmergencyHandler {
    async fn report_emergency(&self, event: &EmergencyEvent) -> Result<()> {
        self.events.write().await.push(event.clone());
        Ok(())
    }

    async fn panic_button(&self, vehicle_id: Uuid, location: GeoLocation) -> Result<EmergencyEvent> {
        let event = EmergencyEvent {
            event_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            timestamp: Utc::now(),
            vehicle_id: Some(vehicle_id),
            trip_id: None,
            event_type: EmergencyEventType::PanicButton,
            severity: EmergencySeverity::High,
            location,
            passenger: None,
            vehicle_status: Some(EmergencyVehicleStatus {
                speed_kmh: Some(0.0),
                is_moving: false,
                doors_locked: false,
                securement_status: Some(OverallSecurementStatus::Secured),
            }),
            response: Some(EmergencyResponse {
                auto_pulled_over: true,
                support_contacted: true,
                emergency_services_called: false,
                eta_support_minutes: Some(3.0),
            }),
        };

        self.report_emergency(&event).await?;
        Ok(event)
    }

    async fn pull_over(&self, _vehicle_id: Uuid) -> Result<()> {
        // Simulate pull over
        Ok(())
    }

    async fn contact_support(&self, _vehicle_id: Uuid) -> Result<()> {
        // Simulate contact support
        Ok(())
    }
}

/// Create a complete simulator client
pub fn create_simulator_client() -> WiaAutoClient<
    SimulatorFleetManager,
    SimulatorProfileManager,
    SimulatorTripManager,
    SimulatorHmiManager,
    SimulatorSecurementMonitor,
    SimulatorEmergencyHandler,
> {
    let fleet = SimulatorFleetManager::new();
    let profiles = SimulatorProfileManager::new();
    let trips = SimulatorTripManager::new(fleet.clone());
    let hmi = SimulatorHmiManager::new();
    let securement = SimulatorSecurementMonitor::new();
    let emergency = SimulatorEmergencyHandler::new();

    WiaAutoClient::new(fleet, profiles, trips, hmi, securement, emergency)
}

impl Default for AccessibilityRequirements {
    fn default() -> Self {
        Self {
            wheelchair_accessible: false,
            ramp_required: false,
            lift_required: false,
            service_animal_space: false,
            companion_space: 0,
            preferred_modalities: vec![],
        }
    }
}
