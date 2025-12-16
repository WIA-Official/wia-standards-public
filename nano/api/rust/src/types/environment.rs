//! Environment condition types for nano systems

use serde::{Deserialize, Serialize};

/// Medium type enumeration
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Medium {
    Vacuum,
    Air,
    Water,
    Saline,
    Blood,
    Cytoplasm,
    Custom(String),
}

impl Default for Medium {
    fn default() -> Self {
        Medium::Water
    }
}

/// Environment conditions for nano systems
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Environment {
    /// Temperature in Kelvin
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature_k: Option<f64>,

    /// Pressure in Pascals
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pressure_pa: Option<f64>,

    /// Medium type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub medium: Option<Medium>,

    /// pH value (0-14)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ph: Option<f64>,

    /// Ionic strength in mol/L
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ionic_strength_m: Option<f64>,

    /// Viscosity in Pa·s
    #[serde(skip_serializing_if = "Option::is_none")]
    pub viscosity_pa_s: Option<f64>,
}

impl Environment {
    /// Create a new empty environment
    pub fn new() -> Self {
        Self::default()
    }

    /// Standard room temperature and pressure
    pub fn standard() -> Self {
        Self {
            temperature_k: Some(298.15), // 25°C
            pressure_pa: Some(101325.0), // 1 atm
            medium: Some(Medium::Air),
            ph: None,
            ionic_strength_m: None,
            viscosity_pa_s: None,
        }
    }

    /// Physiological conditions (blood)
    pub fn physiological() -> Self {
        Self {
            temperature_k: Some(310.15), // 37°C
            pressure_pa: Some(101325.0), // 1 atm
            medium: Some(Medium::Blood),
            ph: Some(7.4),
            ionic_strength_m: Some(0.15),
            viscosity_pa_s: Some(0.004), // Blood viscosity
        }
    }

    /// Intracellular conditions
    pub fn intracellular() -> Self {
        Self {
            temperature_k: Some(310.15), // 37°C
            pressure_pa: Some(101325.0),
            medium: Some(Medium::Cytoplasm),
            ph: Some(7.2),
            ionic_strength_m: Some(0.15),
            viscosity_pa_s: Some(0.002),
        }
    }

    /// Vacuum conditions
    pub fn vacuum() -> Self {
        Self {
            temperature_k: Some(300.0),
            pressure_pa: Some(1e-6), // Near vacuum
            medium: Some(Medium::Vacuum),
            ph: None,
            ionic_strength_m: None,
            viscosity_pa_s: None,
        }
    }

    /// Aqueous solution
    pub fn aqueous(ph: f64) -> Self {
        Self {
            temperature_k: Some(298.15),
            pressure_pa: Some(101325.0),
            medium: Some(Medium::Water),
            ph: Some(ph),
            ionic_strength_m: Some(0.01),
            viscosity_pa_s: Some(0.001),
        }
    }

    /// Builder methods
    pub fn with_temperature(mut self, temp_k: f64) -> Self {
        self.temperature_k = Some(temp_k);
        self
    }

    pub fn with_pressure(mut self, pressure_pa: f64) -> Self {
        self.pressure_pa = Some(pressure_pa);
        self
    }

    pub fn with_medium(mut self, medium: Medium) -> Self {
        self.medium = Some(medium);
        self
    }

    pub fn with_ph(mut self, ph: f64) -> Self {
        self.ph = Some(ph);
        self
    }
}

impl Default for Environment {
    fn default() -> Self {
        Self {
            temperature_k: None,
            pressure_pa: None,
            medium: None,
            ph: None,
            ionic_strength_m: None,
            viscosity_pa_s: None,
        }
    }
}
