//! Integration tests for WIA-IND-001

use wia_fashion_tech::*;
use uuid::Uuid;
use chrono::Utc;

#[test]
fn test_product_category() {
    let category = ProductCategory::Tops;
    assert_eq!(category, ProductCategory::Tops);
}

#[test]
fn test_body_measurements() {
    let measurements = BodyMeasurements {
        height_cm: 170.0,
        weight_kg: 65.0,
        chest_cm: Some(90.0),
        waist_cm: Some(75.0),
        hips_cm: Some(95.0),
    };
    assert_eq!(measurements.height_cm, 170.0);
}

#[test]
fn test_validators() {
    assert!(validators::validate_url("https://api.example.com").is_ok());
    assert!(validators::validate_url("").is_err());

    assert!(validators::validate_api_key("1234567890abcdef").is_ok());
    assert!(validators::validate_api_key("short").is_err());
}

#[test]
fn test_utils() {
    let measurements = BodyMeasurements {
        height_cm: 170.0,
        weight_kg: 70.0,
        chest_cm: Some(90.0),
        waist_cm: None,
        hips_cm: None,
    };
    let size = utils::calculate_size_from_measurements(&measurements, &ProductCategory::Tops);
    assert_eq!(size, "S");

    let price = utils::format_price(99.99, "USD");
    assert!(price.contains("99.99"));
}
