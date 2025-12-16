//! AI/ML dataset generation adapters
//!
//! This module provides exporters for machine learning:
//!
//! - **Dataset**: Training data export for TensorFlow/PyTorch

pub mod dataset;

pub use dataset::{DatasetExporter, TrainingDatapoint, DatasetMetadata};
