//! Utility functions for WIA-IND-001

use crate::types::*;
use uuid::Uuid;

pub fn generate_id() -> Uuid {
    Uuid::new_v4()
}

pub fn calculate_size_from_measurements(measurements: &BodyMeasurements, category: &ProductCategory) -> String {
    match category {
        ProductCategory::Tops => {
            if let Some(chest) = measurements.chest_cm {
                if chest < 86.0 { "XS".to_string() }
                else if chest < 91.0 { "S".to_string() }
                else if chest < 97.0 { "M".to_string() }
                else if chest < 102.0 { "L".to_string() }
                else { "XL".to_string() }
            } else {
                "M".to_string()
            }
        }
        _ => "M".to_string(),
    }
}

pub fn format_price(price: f64, currency: &str) -> String {
    format!("{:.2} {}", price, currency)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_size_calculation() {
        let measurements = BodyMeasurements {
            height_cm: 170.0,
            weight_kg: 70.0,
            chest_cm: Some(90.0),
            waist_cm: None,
            hips_cm: None,
        };
        let size = calculate_size_from_measurements(&measurements, &ProductCategory::Tops);
        assert_eq!(size, "S");
    }
}
