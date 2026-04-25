//! Type definitions for WIA-IND-001 Fashion Technology

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Product {
    pub id: Uuid,
    pub name: String,
    pub brand: String,
    pub category: ProductCategory,
    pub sizes: Vec<String>,
    pub colors: Vec<String>,
    pub price: f64,
    pub currency: String,
    pub metadata: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ProductCategory {
    Tops,
    Bottoms,
    Dresses,
    Outerwear,
    Footwear,
    Accessories,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SizeRecommendation {
    pub user_id: Uuid,
    pub product_id: Uuid,
    pub recommended_size: String,
    pub confidence: f64,
    pub measurements: BodyMeasurements,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyMeasurements {
    pub height_cm: f64,
    pub weight_kg: f64,
    pub chest_cm: Option<f64>,
    pub waist_cm: Option<f64>,
    pub hips_cm: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StyleProfile {
    pub user_id: Uuid,
    pub preferences: Vec<String>,
    pub favorite_colors: Vec<String>,
    pub favorite_brands: Vec<String>,
    pub style_tags: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualTryOn {
    pub id: Uuid,
    pub user_id: Uuid,
    pub product_id: Uuid,
    pub image_url: String,
    pub created_at: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiResponse<T> {
    pub success: bool,
    pub data: Option<T>,
    pub error: Option<String>,
    pub timestamp: DateTime<Utc>,
}
