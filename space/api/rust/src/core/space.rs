//! Space technology specifications
//!
//! This module contains the main specification types for each technology category.

use serde::{Deserialize, Serialize};
use crate::types::*;
use crate::error::{SpaceError, SpaceResult};

// ============================================================================
// Dyson Sphere Specification
// ============================================================================

/// Dyson sphere specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DysonSphereSpec {
    /// Schema URL
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    /// Technology ID
    pub technology_id: String,

    /// Category (always dyson_sphere)
    pub category: TechnologyCategory,

    /// Star parameters
    pub star_parameters: StarParameters,

    /// Structure type
    pub structure_type: DysonStructureType,

    /// Structure parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub structure_parameters: Option<DysonStructureParameters>,

    /// Energy harvesting
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_harvesting: Option<EnergyHarvesting>,
}

/// Dyson structure parameters
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DysonStructureParameters {
    /// Orbital radius in AU
    pub orbital_radius_au: f64,
    /// Number of collector units
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_collectors: Option<u64>,
    /// Area per collector in km²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub collector_area_km2: Option<f64>,
    /// Total collection area in km²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_collection_area_km2: Option<f64>,
    /// Fraction of sphere covered (0-1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coverage_fraction: Option<f64>,
    /// Total structure mass in kg
    #[serde(skip_serializing_if = "Option::is_none")]
    pub material_mass_kg: Option<f64>,
}

/// Energy harvesting configuration
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EnergyHarvesting {
    /// Efficiency percent (0-100)
    pub efficiency_percent: f64,
    /// Total power in watts
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_power_watts: Option<f64>,
    /// Collection method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub collection_method: Option<EnergyCollectionMethod>,
    /// Transmission method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transmission_method: Option<EnergyTransmissionMethod>,
}

impl DysonSphereSpec {
    /// Create a new Dyson sphere specification
    pub fn new(id: impl Into<String>, star: StarParameters, structure_type: DysonStructureType) -> Self {
        Self {
            schema: Some(TechnologyCategory::DysonSphere.schema_url().to_string()),
            technology_id: id.into(),
            category: TechnologyCategory::DysonSphere,
            star_parameters: star,
            structure_type,
            structure_parameters: None,
            energy_harvesting: None,
        }
    }

    /// Create a Sol-based Dyson swarm
    pub fn sol_swarm(id: impl Into<String>) -> Self {
        Self::new(id, StarParameters::sol(), DysonStructureType::DysonSwarm)
    }

    /// Set structure parameters
    pub fn with_structure(mut self, params: DysonStructureParameters) -> Self {
        self.structure_parameters = Some(params);
        self
    }

    /// Set energy harvesting
    pub fn with_energy(mut self, harvesting: EnergyHarvesting) -> Self {
        self.energy_harvesting = Some(harvesting);
        self
    }

    /// Calculate total power based on star luminosity and coverage
    pub fn calculate_power(&self) -> Option<f64> {
        let coverage = self.structure_parameters.as_ref()?.coverage_fraction?;
        let efficiency = self.energy_harvesting.as_ref()?.efficiency_percent / 100.0;
        let solar_luminosity_watts = 3.828e26;
        let total_luminosity = self.star_parameters.luminosity_solar * solar_luminosity_watts;
        Some(total_luminosity * coverage * efficiency)
    }
}

// ============================================================================
// Mars Terraforming Specification
// ============================================================================

/// Mars terraforming specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MarsTerraformingSpec {
    /// Schema URL
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    /// Technology ID
    pub technology_id: String,

    /// Category
    pub category: TechnologyCategory,

    /// Target body
    pub target_body: TargetBody,

    /// Current conditions
    pub current_conditions: PlanetaryConditions,

    /// Target conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_conditions: Option<PlanetaryConditions>,

    /// Intervention methods
    #[serde(skip_serializing_if = "Option::is_none")]
    pub intervention_methods: Option<Vec<InterventionMethod>>,

    /// Timeline
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeline: Option<TerraformingTimeline>,
}

/// Target body for terraforming
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TargetBody {
    pub name: String,
    #[serde(rename = "type")]
    pub body_type: CelestialBodyType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub orbital_radius_au: Option<f64>,
}

/// Planetary conditions
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanetaryConditions {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub surface_pressure_mbar: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mean_temperature_celsius: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub atmosphere_composition: Option<AtmosphereComposition>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub surface_gravity_g: Option<f64>,
}

/// Terraforming intervention method
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InterventionMethod {
    pub method: TerraformingMethod,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature_effect_kelvin: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub implementation_time_years: Option<f64>,
}

/// Terraforming timeline
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TerraformingTimeline {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phase_1_warming_years: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phase_2_atmosphere_years: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phase_3_biosphere_years: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_estimate_years: Option<f64>,
}

impl MarsTerraformingSpec {
    /// Create Mars terraforming specification
    pub fn mars(id: impl Into<String>) -> Self {
        Self {
            schema: Some(TechnologyCategory::MarsTerraforming.schema_url().to_string()),
            technology_id: id.into(),
            category: TechnologyCategory::MarsTerraforming,
            target_body: TargetBody {
                name: "Mars".to_string(),
                body_type: CelestialBodyType::Planet,
                orbital_radius_au: Some(1.524),
            },
            current_conditions: PlanetaryConditions {
                surface_pressure_mbar: Some(6.1),
                mean_temperature_celsius: Some(-63.0),
                atmosphere_composition: Some(AtmosphereComposition::mars_current()),
                surface_gravity_g: Some(0.38),
            },
            target_conditions: None,
            intervention_methods: None,
            timeline: None,
        }
    }

    /// Add intervention method
    pub fn add_method(mut self, method: InterventionMethod) -> Self {
        self.intervention_methods
            .get_or_insert_with(Vec::new)
            .push(method);
        self
    }

    /// Set target conditions
    pub fn with_target(mut self, conditions: PlanetaryConditions) -> Self {
        self.target_conditions = Some(conditions);
        self
    }
}

// ============================================================================
// Warp Drive Specification
// ============================================================================

/// Warp drive specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WarpDriveSpec {
    /// Schema URL
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    /// Technology ID
    pub technology_id: String,

    /// Category
    pub category: TechnologyCategory,

    /// Drive type
    pub drive_type: WarpDriveType,

    /// Theoretical basis
    pub theoretical_basis: TheoreticalBasis,

    /// Bubble parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bubble_parameters: Option<BubbleParameters>,

    /// Performance
    #[serde(skip_serializing_if = "Option::is_none")]
    pub performance: Option<WarpPerformance>,
}

/// Theoretical basis for warp drive
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TheoreticalBasis {
    pub metric: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requires_exotic_matter: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_conditions_satisfied: Option<Vec<String>>,
}

/// Warp bubble parameters
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BubbleParameters {
    pub radius_meters: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wall_thickness_meters: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub shape: Option<BubbleShape>,
}

/// Warp drive performance
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WarpPerformance {
    /// Maximum velocity as fraction of c
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_velocity_c: Option<f64>,
    /// Cruise velocity as fraction of c
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cruise_velocity_c: Option<f64>,
    /// Energy requirement in joules
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_requirement_joules: Option<f64>,
}

impl WarpDriveSpec {
    /// Create a new warp drive specification
    pub fn new(id: impl Into<String>, drive_type: WarpDriveType) -> Self {
        Self {
            schema: Some(TechnologyCategory::WarpDrive.schema_url().to_string()),
            technology_id: id.into(),
            category: TechnologyCategory::WarpDrive,
            drive_type,
            theoretical_basis: TheoreticalBasis {
                metric: "alcubierre".to_string(),
                requires_exotic_matter: None,
                energy_conditions_satisfied: None,
            },
            bubble_parameters: None,
            performance: None,
        }
    }

    /// Create subluminal Alcubierre drive (2024 research)
    pub fn subluminal(id: impl Into<String>) -> Self {
        Self::new(id, WarpDriveType::AlcubierreSubluminal)
            .with_theoretical_basis(TheoreticalBasis {
                metric: "alcubierre".to_string(),
                requires_exotic_matter: Some(false),
                energy_conditions_satisfied: Some(vec![
                    "null".to_string(),
                    "weak".to_string(),
                    "strong".to_string(),
                    "dominant".to_string(),
                ]),
            })
    }

    /// Set theoretical basis
    pub fn with_theoretical_basis(mut self, basis: TheoreticalBasis) -> Self {
        self.theoretical_basis = basis;
        self
    }

    /// Set bubble parameters
    pub fn with_bubble(mut self, params: BubbleParameters) -> Self {
        self.bubble_parameters = Some(params);
        self
    }

    /// Set performance
    pub fn with_performance(mut self, perf: WarpPerformance) -> Self {
        self.performance = Some(perf);
        self
    }

    /// Validate velocity is below speed of light
    pub fn validate(&self) -> SpaceResult<()> {
        if let Some(ref perf) = self.performance {
            if let Some(v) = perf.max_velocity_c {
                if v > 1.0 && self.drive_type == WarpDriveType::AlcubierreSubluminal {
                    return Err(SpaceError::physics(
                        "Subluminal drive cannot exceed speed of light",
                    ));
                }
            }
        }
        Ok(())
    }
}

// ============================================================================
// Space Elevator Specification
// ============================================================================

/// Space elevator specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpaceElevatorSpec {
    /// Schema URL
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    /// Technology ID
    pub technology_id: String,

    /// Category
    pub category: TechnologyCategory,

    /// Location
    pub location: ElevatorLocation,

    /// Tether specification
    pub tether: TetherSpec,

    /// Counterweight
    #[serde(skip_serializing_if = "Option::is_none")]
    pub counterweight: Option<Counterweight>,

    /// Climber
    #[serde(skip_serializing_if = "Option::is_none")]
    pub climber: Option<ClimberSpec>,
}

/// Elevator anchor location
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ElevatorLocation {
    pub anchor_latitude: f64,
    pub anchor_longitude: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anchor_altitude_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anchor_description: Option<String>,
}

/// Tether specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TetherSpec {
    pub material: TetherMaterial,
    pub total_length_km: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub taper_ratio: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tensile_strength_gpa: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_mass_kg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety_factor: Option<f64>,
}

/// Counterweight specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Counterweight {
    #[serde(rename = "type")]
    pub weight_type: CounterweightType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass_kg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub altitude_km: Option<f64>,
}

/// Climber specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ClimberSpec {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload_capacity_kg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ascent_velocity_m_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub power_source: Option<ClimberPowerSource>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trip_time_hours: Option<f64>,
}

impl SpaceElevatorSpec {
    /// Create a new space elevator specification
    pub fn new(
        id: impl Into<String>,
        latitude: f64,
        longitude: f64,
        material: TetherMaterial,
        length_km: f64,
    ) -> Self {
        Self {
            schema: Some(TechnologyCategory::SpaceElevator.schema_url().to_string()),
            technology_id: id.into(),
            category: TechnologyCategory::SpaceElevator,
            location: ElevatorLocation {
                anchor_latitude: latitude,
                anchor_longitude: longitude,
                anchor_altitude_m: Some(0.0),
                anchor_description: None,
            },
            tether: TetherSpec {
                material,
                total_length_km: length_km,
                taper_ratio: None,
                tensile_strength_gpa: Some(material.tensile_strength_gpa()),
                total_mass_kg: None,
                safety_factor: Some(2.0),
            },
            counterweight: None,
            climber: None,
        }
    }

    /// Create Earth equatorial elevator
    pub fn earth_equatorial(id: impl Into<String>) -> Self {
        Self::new(id, 0.0, -80.0, TetherMaterial::SingleCrystalGraphene, 100000.0)
            .with_counterweight(Counterweight {
                weight_type: CounterweightType::CapturedAsteroid,
                mass_kg: Some(1e12),
                altitude_km: Some(100000.0),
            })
            .with_climber(ClimberSpec {
                payload_capacity_kg: Some(20000.0),
                ascent_velocity_m_s: Some(200.0),
                power_source: Some(ClimberPowerSource::LaserBeamed),
                trip_time_hours: Some(139.0),
            })
    }

    /// Set counterweight
    pub fn with_counterweight(mut self, cw: Counterweight) -> Self {
        self.counterweight = Some(cw);
        self
    }

    /// Set climber
    pub fn with_climber(mut self, climber: ClimberSpec) -> Self {
        self.climber = Some(climber);
        self
    }

    /// Calculate trip time to GEO
    pub fn calculate_geo_trip_time(&self) -> Option<f64> {
        let velocity = self.climber.as_ref()?.ascent_velocity_m_s?;
        let geo_altitude = 35786.0 * 1000.0; // meters
        Some(geo_altitude / velocity / 3600.0) // hours
    }
}

// ============================================================================
// Asteroid Mining Specification
// ============================================================================

/// Asteroid mining specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AsteroidMiningSpec {
    /// Schema URL
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    /// Technology ID
    pub technology_id: String,

    /// Category
    pub category: TechnologyCategory,

    /// Target asteroid
    pub target_asteroid: TargetAsteroid,

    /// Resource assessment
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resource_assessment: Option<ResourceAssessment>,

    /// Mission parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mission_parameters: Option<MiningMissionParameters>,

    /// Economic analysis
    #[serde(skip_serializing_if = "Option::is_none")]
    pub economic_analysis: Option<EconomicAnalysis>,
}

/// Target asteroid
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TargetAsteroid {
    pub name: String,
    #[serde(rename = "type")]
    pub asteroid_type: AsteroidType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub designation: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub diameter_km: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass_kg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub orbital_parameters: Option<OrbitalParameters>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<AsteroidLocation>,
}

impl TargetAsteroid {
    pub fn new(name: impl Into<String>, asteroid_type: AsteroidType) -> Self {
        Self {
            name: name.into(),
            asteroid_type,
            designation: None,
            diameter_km: None,
            mass_kg: None,
            orbital_parameters: None,
            location: None,
        }
    }

    pub fn with_diameter_km(mut self, d: f64) -> Self {
        self.diameter_km = Some(d);
        self
    }

    pub fn with_mass_kg(mut self, m: f64) -> Self {
        self.mass_kg = Some(m);
        self
    }

    pub fn with_location(mut self, loc: AsteroidLocation) -> Self {
        self.location = Some(loc);
        self
    }

    pub fn with_orbit(mut self, orbit: OrbitalParameters) -> Self {
        self.orbital_parameters = Some(orbit);
        self
    }
}

/// Resource assessment
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResourceAssessment {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assessment_date: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence_level: Option<f64>,
    pub resources: Vec<Resource>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_estimated_value_usd: Option<f64>,
}

/// Mining mission parameters
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MiningMissionParameters {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub launch_window: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delta_v_km_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub travel_time_days: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stay_time_days: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extraction_method: Option<ExtractionMethod>,
}

/// Economic analysis
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EconomicAnalysis {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mission_cost_usd: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expected_return_usd: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub roi_percent: Option<f64>,
}

impl AsteroidMiningSpec {
    /// Create a new asteroid mining specification
    pub fn new(id: impl Into<String>, asteroid: TargetAsteroid) -> Self {
        Self {
            schema: Some(TechnologyCategory::AsteroidMining.schema_url().to_string()),
            technology_id: id.into(),
            category: TechnologyCategory::AsteroidMining,
            target_asteroid: asteroid,
            resource_assessment: None,
            mission_parameters: None,
            economic_analysis: None,
        }
    }

    /// Create 16 Psyche mining mission
    pub fn psyche(id: impl Into<String>) -> Self {
        let asteroid = TargetAsteroid::new("16 Psyche", AsteroidType::MType)
            .with_diameter_km(226.0)
            .with_mass_kg(2.72e19)
            .with_location(AsteroidLocation::MainBelt)
            .with_orbit(OrbitalParameters::new(2.923, 0.134, 3.095));

        Self::new(id, asteroid).with_resources(ResourceAssessment {
            assessment_date: None,
            confidence_level: Some(0.7),
            resources: vec![
                Resource::new("Fe", 0.85).with_name("Iron"),
                Resource::new("Ni", 0.10).with_name("Nickel"),
                Resource::new("Au", 0.0001).with_name("Gold"),
                Resource::new("Pt", 0.00005).with_name("Platinum"),
            ],
            total_estimated_value_usd: Some(1e17),
        })
    }

    /// Set resource assessment
    pub fn with_resources(mut self, assessment: ResourceAssessment) -> Self {
        self.resource_assessment = Some(assessment);
        self
    }

    /// Set mission parameters
    pub fn with_mission(mut self, params: MiningMissionParameters) -> Self {
        self.mission_parameters = Some(params);
        self
    }

    /// Calculate ROI
    pub fn calculate_roi(&self) -> Option<f64> {
        let cost = self.economic_analysis.as_ref()?.mission_cost_usd?;
        let return_val = self.economic_analysis.as_ref()?.expected_return_usd?;
        Some((return_val - cost) / cost * 100.0)
    }
}

// ============================================================================
// Interstellar Travel Specification
// ============================================================================

/// Interstellar travel specification
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InterstellarTravelSpec {
    /// Schema URL
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    /// Technology ID
    pub technology_id: String,

    /// Category
    pub category: TechnologyCategory,

    /// Mission info
    pub mission: InterstellarMission,

    /// Propulsion
    pub propulsion: InterstellarPropulsion,

    /// Spacecraft
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spacecraft: Option<InterstellarSpacecraft>,

    /// Trajectory
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trajectory: Option<InterstellarTrajectory>,
}

/// Interstellar mission info
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InterstellarMission {
    pub name: String,
    #[serde(rename = "type")]
    pub mission_type: MissionType,
    pub target_system: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_body: Option<String>,
    pub distance_ly: f64,
}

/// Interstellar propulsion
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InterstellarPropulsion {
    #[serde(rename = "type")]
    pub propulsion_type: PropulsionType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sail_area_m2: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sail_mass_kg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub laser_array_power_gw: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub acceleration_g: Option<f64>,
}

/// Interstellar spacecraft
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InterstellarSpacecraft {
    #[serde(rename = "type")]
    pub spacecraft_type: SpacecraftType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_mass_kg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload_mass_kg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instruments: Option<Vec<String>>,
}

/// Interstellar trajectory
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InterstellarTrajectory {
    /// Cruise velocity as fraction of c
    pub cruise_velocity_c: f64,
    /// Travel time in years
    pub travel_time_years: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub launch_date: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arrival_date: Option<String>,
}

impl InterstellarTravelSpec {
    /// Create a new interstellar travel specification
    pub fn new(id: impl Into<String>, mission: InterstellarMission, propulsion: InterstellarPropulsion) -> Self {
        Self {
            schema: Some(TechnologyCategory::InterstellarTravel.schema_url().to_string()),
            technology_id: id.into(),
            category: TechnologyCategory::InterstellarTravel,
            mission,
            propulsion,
            spacecraft: None,
            trajectory: None,
        }
    }

    /// Create Alpha Centauri lightsail mission (Breakthrough Starshot style)
    pub fn alpha_centauri_starshot(id: impl Into<String>) -> Self {
        let mission = InterstellarMission {
            name: "Alpha Centauri Probe".to_string(),
            mission_type: MissionType::Flyby,
            target_system: "Alpha Centauri".to_string(),
            target_body: Some("Proxima Centauri b".to_string()),
            distance_ly: 4.246,
        };

        let propulsion = InterstellarPropulsion {
            propulsion_type: PropulsionType::LaserLightsail,
            sail_area_m2: Some(16.0),
            sail_mass_kg: Some(0.001),
            laser_array_power_gw: Some(100.0),
            acceleration_g: Some(30000.0),
        };

        Self::new(id, mission, propulsion)
            .with_spacecraft(InterstellarSpacecraft {
                spacecraft_type: SpacecraftType::Starchip,
                total_mass_kg: Some(0.005),
                payload_mass_kg: Some(0.004),
                instruments: Some(vec![
                    "camera_4k".to_string(),
                    "spectrometer".to_string(),
                    "magnetometer".to_string(),
                ]),
            })
            .with_trajectory(InterstellarTrajectory {
                cruise_velocity_c: 0.20,
                travel_time_years: 21.2,
                launch_date: None,
                arrival_date: None,
            })
    }

    /// Set spacecraft
    pub fn with_spacecraft(mut self, sc: InterstellarSpacecraft) -> Self {
        self.spacecraft = Some(sc);
        self
    }

    /// Set trajectory
    pub fn with_trajectory(mut self, traj: InterstellarTrajectory) -> Self {
        self.trajectory = Some(traj);
        self
    }

    /// Calculate travel time from distance and velocity
    pub fn calculate_travel_time(&self) -> Option<f64> {
        let velocity_c = self.trajectory.as_ref()?.cruise_velocity_c;
        if velocity_c <= 0.0 {
            return None;
        }
        Some(self.mission.distance_ly / velocity_c)
    }

    /// Validate velocity constraints
    pub fn validate(&self) -> SpaceResult<()> {
        if let Some(ref traj) = self.trajectory {
            if traj.cruise_velocity_c > 1.0 {
                return Err(SpaceError::physics("Velocity cannot exceed speed of light"));
            }
            if traj.cruise_velocity_c <= 0.0 {
                return Err(SpaceError::validation("Velocity must be positive"));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dyson_sphere() {
        let dyson = DysonSphereSpec::sol_swarm("dyson-001");
        assert_eq!(dyson.category, TechnologyCategory::DysonSphere);
        assert_eq!(dyson.star_parameters.name, "Sol");
    }

    #[test]
    fn test_asteroid_mining() {
        let mining = AsteroidMiningSpec::psyche("mining-psyche");
        assert_eq!(mining.target_asteroid.name, "16 Psyche");
        assert!(mining.resource_assessment.is_some());
    }

    #[test]
    fn test_interstellar_starshot() {
        let starshot = InterstellarTravelSpec::alpha_centauri_starshot("starshot-001");
        assert_eq!(starshot.mission.distance_ly, 4.246);
        assert!(starshot.validate().is_ok());
    }

    #[test]
    fn test_space_elevator() {
        let elevator = SpaceElevatorSpec::earth_equatorial("elevator-001");
        let trip_time = elevator.calculate_geo_trip_time();
        assert!(trip_time.is_some());
        assert!(trip_time.unwrap() > 40.0); // Should be around 50 hours
    }

    #[test]
    fn test_serialization() {
        let spec = AsteroidMiningSpec::psyche("test");
        let json = serde_json::to_string_pretty(&spec).unwrap();
        assert!(json.contains("asteroid_mining"));
        assert!(json.contains("16 Psyche"));
    }
}
