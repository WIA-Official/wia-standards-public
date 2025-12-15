//! Built-in output adapter implementations
//!
//! This module provides ready-to-use adapters for common output destinations:
//!
//! - [`ConsoleAdapter`] - Prints messages to stdout (useful for testing)
//! - [`WebhookAdapter`] - Sends notifications via HTTP webhooks
//! - [`InfluxDBAdapter`] - Writes to InfluxDB time-series database

mod console;
mod webhook;
mod influxdb;

pub use console::ConsoleAdapter;
pub use webhook::{WebhookAdapter, WebhookConfig, WebhookPayload};
pub use influxdb::{InfluxDBAdapter, InfluxDBConfig};
