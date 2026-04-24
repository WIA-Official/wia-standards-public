//! I/O adapters for reading and writing WIA Space data
//!
//! Provides utilities for serialization, file I/O, and format conversion.

use std::path::Path;
use crate::error::{SpaceError, SpaceResult};
use crate::types::TechnologyCategory;
use crate::core::*;
use serde::{de::DeserializeOwned, Serialize};

/// Read a WIA Space specification from a JSON file
pub fn read_json<T: DeserializeOwned>(path: impl AsRef<Path>) -> SpaceResult<T> {
    let content = std::fs::read_to_string(path)?;
    let data = serde_json::from_str(&content)?;
    Ok(data)
}

/// Write a WIA Space specification to a JSON file
pub fn write_json<T: Serialize>(path: impl AsRef<Path>, data: &T) -> SpaceResult<()> {
    let content = serde_json::to_string_pretty(data)?;
    std::fs::write(path, content)?;
    Ok(())
}

/// Read a specification from a JSON string
pub fn from_json_str<T: DeserializeOwned>(json: &str) -> SpaceResult<T> {
    let data = serde_json::from_str(json)?;
    Ok(data)
}

/// Convert a specification to a JSON string
pub fn to_json_str<T: Serialize>(data: &T) -> SpaceResult<String> {
    let json = serde_json::to_string_pretty(data)?;
    Ok(json)
}

/// Specification type detector
pub struct SpecType;

impl SpecType {
    /// Detect specification type from JSON
    pub fn detect(json: &str) -> SpaceResult<TechnologyCategory> {
        let value: serde_json::Value = serde_json::from_str(json)?;

        if let Some(category) = value.get("category").and_then(|v| v.as_str()) {
            match category {
                "dyson_sphere" => Ok(TechnologyCategory::DysonSphere),
                "mars_terraforming" => Ok(TechnologyCategory::MarsTerraforming),
                "warp_drive" => Ok(TechnologyCategory::WarpDrive),
                "space_elevator" => Ok(TechnologyCategory::SpaceElevator),
                "asteroid_mining" => Ok(TechnologyCategory::AsteroidMining),
                "interstellar_travel" => Ok(TechnologyCategory::InterstellarTravel),
                _ => Err(SpaceError::InvalidCategory(category.to_string())),
            }
        } else {
            Err(SpaceError::MissingField("category".to_string()))
        }
    }

    /// Load specification dynamically based on category
    pub fn load_any(json: &str) -> SpaceResult<Box<dyn std::any::Any>> {
        let category = Self::detect(json)?;

        match category {
            TechnologyCategory::DysonSphere => {
                let spec: DysonSphereSpec = serde_json::from_str(json)?;
                Ok(Box::new(spec))
            }
            TechnologyCategory::MarsTerraforming => {
                let spec: MarsTerraformingSpec = serde_json::from_str(json)?;
                Ok(Box::new(spec))
            }
            TechnologyCategory::WarpDrive => {
                let spec: WarpDriveSpec = serde_json::from_str(json)?;
                Ok(Box::new(spec))
            }
            TechnologyCategory::SpaceElevator => {
                let spec: SpaceElevatorSpec = serde_json::from_str(json)?;
                Ok(Box::new(spec))
            }
            TechnologyCategory::AsteroidMining => {
                let spec: AsteroidMiningSpec = serde_json::from_str(json)?;
                Ok(Box::new(spec))
            }
            TechnologyCategory::InterstellarTravel => {
                let spec: InterstellarTravelSpec = serde_json::from_str(json)?;
                Ok(Box::new(spec))
            }
        }
    }
}

/// Schema validator (placeholder for JSON Schema validation)
pub struct Validator;

impl Validator {
    /// Validate JSON against WIA Space schema (basic validation)
    pub fn validate_basic(json: &str) -> SpaceResult<()> {
        let value: serde_json::Value = serde_json::from_str(json)?;

        // Check for required fields
        if !value.get("technology_id").is_some() {
            return Err(SpaceError::missing("technology_id"));
        }

        if !value.get("category").is_some() {
            return Err(SpaceError::missing("category"));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_category() {
        let json = r#"{"technology_id": "test", "category": "asteroid_mining"}"#;
        let category = SpecType::detect(json).unwrap();
        assert_eq!(category, TechnologyCategory::AsteroidMining);
    }

    #[test]
    fn test_serialization_roundtrip() {
        let spec = AsteroidMiningSpec::psyche("test-001");
        let json = to_json_str(&spec).unwrap();
        let restored: AsteroidMiningSpec = from_json_str(&json).unwrap();
        assert_eq!(spec.technology_id, restored.technology_id);
    }

    #[test]
    fn test_validation() {
        let valid = r#"{"technology_id": "test", "category": "warp_drive"}"#;
        assert!(Validator::validate_basic(valid).is_ok());

        let invalid = r#"{"category": "warp_drive"}"#;
        assert!(Validator::validate_basic(invalid).is_err());
    }
}
