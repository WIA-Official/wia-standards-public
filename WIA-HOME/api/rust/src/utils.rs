//! Utility functions for WIA-HOME SDK
//!
//! 弘益人間 - Helpful tools for smart home development

use crate::types::*;
use chrono::{DateTime, Utc};
use uuid::Uuid;

/// Generate a new device ID
pub fn generate_device_id() -> Uuid {
    Uuid::new_v4()
}

/// Generate a new scene ID
pub fn generate_scene_id() -> Uuid {
    Uuid::new_v4()
}

/// Get current timestamp
pub fn current_timestamp() -> DateTime<Utc> {
    Utc::now()
}

/// Format device name for display
pub fn format_device_name(device: &Device) -> String {
    if let Some(room) = &device.room {
        format!("{} ({})", device.name, room)
    } else {
        device.name.clone()
    }
}

/// Calculate total energy consumption
pub fn calculate_total_energy(data: &[EnergyData]) -> f64 {
    data.iter().map(|d| d.consumption_kwh).sum()
}

/// Calculate average energy consumption
pub fn calculate_average_energy(data: &[EnergyData]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    calculate_total_energy(data) / data.len() as f64
}

/// Filter devices by type
pub fn filter_devices_by_type(devices: &[Device], device_type: &DeviceType) -> Vec<Device> {
    devices
        .iter()
        .filter(|d| &d.device_type == device_type)
        .cloned()
        .collect()
}

/// Filter devices by room
pub fn filter_devices_by_room(devices: &[Device], room: &str) -> Vec<Device> {
    devices
        .iter()
        .filter(|d| d.room.as_deref() == Some(room))
        .cloned()
        .collect()
}

/// Filter devices by status
pub fn filter_devices_by_status(devices: &[Device], status: &DeviceStatus) -> Vec<Device> {
    devices
        .iter()
        .filter(|d| &d.status == status)
        .cloned()
        .collect()
}

/// Count devices by type
pub fn count_devices_by_type(devices: &[Device]) -> std::collections::HashMap<String, usize> {
    let mut counts = std::collections::HashMap::new();
    for device in devices {
        let type_name = match &device.device_type {
            DeviceType::Custom(name) => name.clone(),
            other => format!("{:?}", other).to_lowercase(),
        };
        *counts.entry(type_name).or_insert(0) += 1;
    }
    counts
}

/// Check if device is online
pub fn is_device_online(device: &Device) -> bool {
    device.status == DeviceStatus::Online
}

/// Get offline devices
pub fn get_offline_devices(devices: &[Device]) -> Vec<Device> {
    filter_devices_by_status(devices, &DeviceStatus::Offline)
}

/// Format energy consumption with unit
pub fn format_energy(kwh: f64) -> String {
    if kwh < 1.0 {
        format!("{:.2} Wh", kwh * 1000.0)
    } else if kwh < 1000.0 {
        format!("{:.2} kWh", kwh)
    } else {
        format!("{:.2} MWh", kwh / 1000.0)
    }
}

/// Parse device type from string
pub fn parse_device_type(s: &str) -> DeviceType {
    match s.to_lowercase().as_str() {
        "light" => DeviceType::Light,
        "thermostat" => DeviceType::Thermostat,
        "lock" => DeviceType::Lock,
        "camera" => DeviceType::Camera,
        "sensor" => DeviceType::Sensor,
        "switch" => DeviceType::Switch,
        "outlet" => DeviceType::Outlet,
        "speaker" => DeviceType::Speaker,
        "display" => DeviceType::Display,
        "appliance" => DeviceType::Appliance,
        custom => DeviceType::Custom(custom.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_ids() {
        let id1 = generate_device_id();
        let id2 = generate_device_id();
        assert_ne!(id1, id2);
    }

    #[test]
    fn test_format_energy() {
        assert_eq!(format_energy(0.5), "500.00 Wh");
        assert_eq!(format_energy(1.5), "1.50 kWh");
        assert_eq!(format_energy(1500.0), "1.50 MWh");
    }

    #[test]
    fn test_parse_device_type() {
        assert_eq!(parse_device_type("light"), DeviceType::Light);
        assert_eq!(parse_device_type("CAMERA"), DeviceType::Camera);
        match parse_device_type("custom_device") {
            DeviceType::Custom(name) => assert_eq!(name, "custom_device"),
            _ => panic!("Expected Custom device type"),
        }
    }

    #[test]
    fn test_calculate_energy() {
        let data = vec![
            EnergyData {
                device_id: Uuid::new_v4(),
                timestamp: Utc::now(),
                consumption_kwh: 1.5,
                cost: None,
                currency: None,
            },
            EnergyData {
                device_id: Uuid::new_v4(),
                timestamp: Utc::now(),
                consumption_kwh: 2.5,
                cost: None,
                currency: None,
            },
        ];

        assert_eq!(calculate_total_energy(&data), 4.0);
        assert_eq!(calculate_average_energy(&data), 2.0);
    }
}
