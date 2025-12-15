//! Electrode array management for WIA Bionic Eye

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

/// Electrode array configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectrodeArray {
    pub array_id: String,
    pub implant_type: ImplantType,
    pub manufacturer: String,
    pub model: String,
    pub serial_number: String,
    pub physical: PhysicalSpec,
    pub electrodes: Vec<ElectrodeInfo>,
    pub placement: PlacementInfo,
    pub safety_limits: SafetyLimits,
}

/// Implant type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImplantType {
    Epiretinal,
    Subretinal,
    Suprachoroidal,
    OpticNerve,
    Cortical,
}

/// Physical specifications
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicalSpec {
    pub total_electrodes: u32,
    pub active_electrodes: u32,
    pub rows: u32,
    pub columns: u32,
    pub electrode_diameter_um: f32,
    pub electrode_spacing_um: f32,
    pub material: ElectrodeMaterial,
}

/// Electrode material
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ElectrodeMaterial {
    Platinum,
    PlatinumIridium,
    IridiumOxide,
    TitaniumNitride,
    Pedot,
    CarbonNanotube,
}

/// Individual electrode information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectrodeInfo {
    pub index: u32,
    pub grid_position: GridPosition,
    pub functional: bool,
    pub enabled: bool,
    pub impedance_kohm: f32,
    pub threshold_current_ua: f32,
    pub max_safe_current_ua: f32,
}

/// Grid position
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GridPosition {
    pub row: u32,
    pub col: u32,
}

/// Placement information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlacementInfo {
    pub eye: Eye,
    pub implant_date: DateTime<Utc>,
    pub surgeon: String,
    pub hospital: String,
}

/// Eye side
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Eye {
    Left,
    Right,
}

/// Safety limits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyLimits {
    /// Maximum charge density in μC/cm²
    pub max_charge_density: f32,
    /// Maximum charge per phase in μC
    pub max_charge_per_phase: f32,
    /// Maximum current per electrode in μA
    pub max_current_per_electrode: f32,
    /// Maximum total current in mA
    pub max_total_current: f32,
    /// Maximum stimulation frequency in Hz
    pub max_frequency: f32,
    /// Maximum duty cycle in percent
    pub max_duty_cycle: f32,
}

impl ElectrodeArray {
    /// Create a new electrode array
    pub fn new(
        array_id: String,
        implant_type: ImplantType,
        rows: u32,
        columns: u32,
    ) -> Self {
        let total = rows * columns;
        let mut electrodes = Vec::with_capacity(total as usize);

        for i in 0..total {
            electrodes.push(ElectrodeInfo {
                index: i,
                grid_position: GridPosition {
                    row: i / columns,
                    col: i % columns,
                },
                functional: true,
                enabled: true,
                impedance_kohm: 10.0,
                threshold_current_ua: 100.0,
                max_safe_current_ua: 500.0,
            });
        }

        Self {
            array_id,
            implant_type,
            manufacturer: String::new(),
            model: String::new(),
            serial_number: String::new(),
            physical: PhysicalSpec {
                total_electrodes: total,
                active_electrodes: total,
                rows,
                columns,
                electrode_diameter_um: 200.0,
                electrode_spacing_um: 575.0,
                material: ElectrodeMaterial::Platinum,
            },
            electrodes,
            placement: PlacementInfo {
                eye: Eye::Right,
                implant_date: Utc::now(),
                surgeon: String::new(),
                hospital: String::new(),
            },
            safety_limits: SafetyLimits::default(),
        }
    }

    /// Get functional electrode count
    pub fn functional_count(&self) -> usize {
        self.electrodes.iter().filter(|e| e.functional).count()
    }

    /// Get enabled electrode count
    pub fn enabled_count(&self) -> usize {
        self.electrodes.iter().filter(|e| e.enabled).count()
    }

    /// Check if an electrode is within safety limits
    pub fn check_electrode_safety(&self, index: u32, current_ua: f32) -> bool {
        if let Some(electrode) = self.electrodes.get(index as usize) {
            current_ua <= electrode.max_safe_current_ua
                && current_ua <= self.safety_limits.max_current_per_electrode
        } else {
            false
        }
    }
}

impl Default for SafetyLimits {
    fn default() -> Self {
        Self {
            max_charge_density: 35.0,
            max_charge_per_phase: 0.5,
            max_current_per_electrode: 1000.0,
            max_total_current: 10.0,
            max_frequency: 300.0,
            max_duty_cycle: 50.0,
        }
    }
}
