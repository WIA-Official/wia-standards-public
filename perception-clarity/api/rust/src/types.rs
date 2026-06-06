//! Phase 1 data model — enums, the PCI newtype, axes/weights, and the report schema.
//!
//! Every type mirrors `PHASE-1-DATA-FORMAT.md` 1:1 and serializes to the exact wire
//! format defined there (snake_case enum values, camelCase report fields).

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

// ============================================================================
// Enums — Phase 1 §2
// ============================================================================

/// Sensor class — Phase 1 §2.1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorClass {
    /// Visible-light camera — `"rgb_camera"`
    RgbCamera,
    /// Infrared / thermal — `"ir_thermal"`
    IrThermal,
    /// LiDAR transmission window — `"lidar_window"`
    LidarWindow,
    /// Radar radome — `"radar_radome"`
    RadarRadome,
    /// Ultrasonic transducer face — `"ultrasonic"`
    Ultrasonic,
}

/// Clarity state — Phase 1 §2.2.
///
/// `Ord` is derived to preserve the invariant ordering CLEAR > DEGRADED >
/// OBSTRUCTED > BLIND at the type level. Variants are declared worst-first so
/// that a larger value means clearer; `min()` over a set therefore yields the
/// worst state (handy for safety gating).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClarityState {
    /// PCI 0–29 — sensor disabled, enter safe-state — `"blind"`
    Blind,
    /// PCI 30–59 — safe action (slow down, reroute, clean) — `"obstructed"`
    Obstructed,
    /// PCI 60–89 — monitor, trigger cleaning — `"degraded"`
    Degraded,
    /// PCI 90–100 — nominal — `"clear"`
    Clear,
}

/// Contaminant type — Phase 1 §2.3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContaminantType {
    /// Rain film — `"rain_film"`
    RainFilm,
    /// Mud / dust — `"mud_dust"`
    MudDust,
    /// Salt spray — `"salt_spray"`
    SaltSpray,
    /// Insect strike — `"insect_strike"`
    InsectStrike,
    /// Frost / ice — `"frost_ice"`
    FrostIce,
    /// Condensation / fogging — `"condensation"`
    Condensation,
    /// Sun glare / saturation — `"sun_glare"`
    SunGlare,
    /// Scratch / abrasion — `"scratch_abrasion"`
    ScratchAbrasion,
    /// Unclassified — `"other"` (a `note` is recommended)
    Other,
}

/// Agent type — Phase 1 §2.4.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AgentType {
    Vehicle,
    Robot,
    Drone,
    Amr,
    Other,
}

/// Conformance level — Phase 1 §2.5. Wire values use hyphens (`L-A`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ConformanceLevel {
    /// PCI computation + state enum exposure
    #[serde(rename = "L-A")]
    La,
    /// L-A + standard report emission
    #[serde(rename = "L-B")]
    Lb,
    /// L-B + dwell-time SLA + safe-action trigger integration
    #[serde(rename = "L-C")]
    Lc,
}

// ============================================================================
// PciIndex — Phase 1 §3
// ============================================================================

/// Perception Clarity Index — Phase 1 §3. A 0–100 integer; higher is clearer.
///
/// Wrapped in a newtype so that construction always clamps to `[0, 100]` and the
/// PCI → state mapping lives in exactly one place.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct PciIndex(u8);

impl PciIndex {
    /// Construct from a raw integer, clamping to `[0, 100]`. All computed PCI
    /// values pass through this constructor.
    pub fn new(value: i32) -> Self {
        PciIndex(value.clamp(0, 100) as u8)
    }

    /// The underlying 0–100 value.
    pub fn value(self) -> u8 {
        self.0
    }

    /// Phase 1 §3.3 default band mapping.
    pub fn state(self) -> ClarityState {
        match self.0 {
            90..=100 => ClarityState::Clear,
            60..=89 => ClarityState::Degraded,
            30..=59 => ClarityState::Obstructed,
            _ => ClarityState::Blind, // 0–29
        }
    }
}

// ============================================================================
// Axes & weights — Phase 1 §3.1 / §3.2
// ============================================================================

/// The three PCI damage axes — each in `[0.0, 1.0]` (Phase 1 §3.1).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ClarityAxes {
    /// Occluded fraction of the effective field of view (OCC).
    pub occlusion: f64,
    /// `(reference_range - current_effective_range) / reference_range` (DIST).
    #[serde(rename = "distanceDegradation")]
    pub distance_degradation: f64,
    /// `1 - (current_MTF50 / reference_MTF50)` (MTF).
    #[serde(rename = "mtfReduction")]
    pub mtf_reduction: f64,
}

impl ClarityAxes {
    /// Construct an axes triple. Values are stored as-is; use
    /// [`ClarityAxes::is_valid`] to confirm each is within `[0.0, 1.0]`.
    pub fn new(occlusion: f64, distance_degradation: f64, mtf_reduction: f64) -> Self {
        Self {
            occlusion,
            distance_degradation,
            mtf_reduction,
        }
    }

    /// Pristine sensor — every axis at zero damage.
    pub fn clean() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// True when every axis lies within `[0.0, 1.0]` (Phase 1 §6.2).
    pub fn is_valid(&self) -> bool {
        let in_range = |v: f64| (0.0..=1.0).contains(&v);
        in_range(self.occlusion) && in_range(self.distance_degradation) && in_range(self.mtf_reduction)
    }
}

/// Sensor-class weight set — must sum to 1.00 (±0.001), Phase 1 §3.2.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PciWeights {
    pub occlusion: f64,
    pub distance: f64,
    pub mtf: f64,
}

impl PciWeights {
    /// Default weights for a sensor class — Phase 1 §3.2 table.
    pub fn default_for(class: SensorClass) -> Self {
        match class {
            SensorClass::RgbCamera => PciWeights { occlusion: 0.40, distance: 0.25, mtf: 0.35 },
            SensorClass::IrThermal => PciWeights { occlusion: 0.40, distance: 0.35, mtf: 0.25 },
            SensorClass::LidarWindow => PciWeights { occlusion: 0.45, distance: 0.45, mtf: 0.10 },
            SensorClass::RadarRadome => PciWeights { occlusion: 0.30, distance: 0.65, mtf: 0.05 },
            SensorClass::Ultrasonic => PciWeights { occlusion: 0.50, distance: 0.50, mtf: 0.00 },
        }
    }

    /// The sum of the three weights.
    pub fn sum(&self) -> f64 {
        self.occlusion + self.distance + self.mtf
    }

    /// True when the weights sum to 1.00 (±0.001) — Phase 1 §6.2.
    pub fn is_valid(&self) -> bool {
        (self.sum() - 1.0).abs() <= 0.001
    }
}

// ============================================================================
// Contaminant — Phase 1 §4.1
// ============================================================================

/// A contaminant entry — `type` and `coverage` are required (Phase 1 §4.1).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Contaminant {
    /// Contaminant classification (serialized as `"type"`).
    #[serde(rename = "type")]
    pub kind: ContaminantType,
    /// Covered fraction — `[0.0, 1.0]`.
    pub coverage: f64,
    /// Free-text note (recommended when `kind == Other`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

impl Contaminant {
    /// Create a contaminant entry.
    pub fn new(kind: ContaminantType, coverage: f64) -> Self {
        Self { kind, coverage, note: None }
    }

    /// Attach a descriptive note.
    pub fn with_note(mut self, note: impl Into<String>) -> Self {
        self.note = Some(note.into());
        self
    }
}

// ============================================================================
// Sensor — Phase 1 §4.1
// ============================================================================

/// A single sensor's clarity snapshot — Phase 1 §4.1.
///
/// Required: `sensorId`, `sensorClass`, `pci`, `state`, `dwellSeconds`,
/// `confidence`. `axes` and `pciWeights` are optional audit fields.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Sensor {
    #[serde(rename = "sensorId")]
    pub sensor_id: String,
    #[serde(rename = "sensorClass")]
    pub sensor_class: SensorClass,
    pub pci: PciIndex,
    pub state: ClarityState,

    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub contaminants: Vec<Contaminant>,

    /// Audit / transparency field (Phase 1 §4.1).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub axes: Option<ClarityAxes>,
    #[serde(rename = "pciWeights", skip_serializing_if = "Option::is_none")]
    pub pci_weights: Option<PciWeights>,

    #[serde(rename = "lastCleanedAt", skip_serializing_if = "Option::is_none")]
    pub last_cleaned_at: Option<DateTime<Utc>>,

    /// Seconds the sensor has continuously been in a non-CLEAR state. 0 if CLEAR.
    #[serde(rename = "dwellSeconds")]
    pub dwell_seconds: f64,

    /// Confidence in the PCI computation itself — `[0.0, 1.0]`.
    pub confidence: f64,
}

// ============================================================================
// SensorClarityReport — Phase 1 §4.1
// ============================================================================

/// Report header — Phase 1 §4.1.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    /// Payload schema version, e.g. `"1.0.0"`.
    pub version: String,
    #[serde(rename = "messageId")]
    pub message_id: String,
    pub timestamp: DateTime<Utc>,
    #[serde(rename = "agentId")]
    pub agent_id: String,
    #[serde(rename = "agentType")]
    pub agent_type: AgentType,
    #[serde(rename = "conformanceLevel", skip_serializing_if = "Option::is_none")]
    pub conformance_level: Option<ConformanceLevel>,
}

/// A point-in-time snapshot of every sensor's clarity for one agent — Phase 1 §4.1.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorClarityReport {
    pub header: Header,
    /// `minItems: 1`.
    pub sensors: Vec<Sensor>,
}

impl SensorClarityReport {
    /// The worst (lowest-ordered) state across all sensors. Empty → `Clear`.
    pub fn worst_state(&self) -> ClarityState {
        self.sensors
            .iter()
            .map(|s| s.state)
            .min()
            .unwrap_or(ClarityState::Clear)
    }

    /// Find a sensor by id.
    pub fn sensor(&self, sensor_id: &str) -> Option<&Sensor> {
        self.sensors.iter().find(|s| s.sensor_id == sensor_id)
    }
}
