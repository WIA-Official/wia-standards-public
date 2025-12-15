//! EPICS Integration Adapter
//!
//! Provides integration with EPICS (Experimental Physics and Industrial Control System)
//! for accelerator and fusion facility control systems.

use super::traits::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// EPICS protocol type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EPICSProtocol {
    /// Channel Access (traditional EPICS, port 5064)
    ChannelAccess,
    /// pvAccess (EPICS7, port 5075)
    PvAccess,
}

impl Default for EPICSProtocol {
    fn default() -> Self {
        Self::ChannelAccess
    }
}

/// Alarm severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AlarmSeverity {
    NoAlarm,
    Minor,
    Major,
    Invalid,
}

impl Default for AlarmSeverity {
    fn default() -> Self {
        Self::NoAlarm
    }
}

/// Alarm status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AlarmStatus {
    NoAlarm,
    Read,
    Write,
    HiHi,
    High,
    LoLo,
    Low,
    State,
    Cos,
    Comm,
    Timeout,
    HwLimit,
    Calc,
    Scan,
    Link,
    Soft,
    BadSub,
    Udf,
    Disable,
    Simm,
    ReadAccess,
    WriteAccess,
}

impl Default for AlarmStatus {
    fn default() -> Self {
        Self::NoAlarm
    }
}

/// Process Variable data types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum PVData {
    Double(f64),
    DoubleArray(Vec<f64>),
    Int(i32),
    IntArray(Vec<i32>),
    String(String),
    Enum(i16),
}

/// Process Variable limits
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PVLimits {
    pub low_alarm: f64,
    pub low_warn: f64,
    pub high_warn: f64,
    pub high_alarm: f64,
}

/// Process Variable value with metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PVValue {
    /// Value
    pub value: PVData,

    /// Timestamp (nanoseconds since epoch)
    pub timestamp: i64,

    /// Alarm severity
    pub severity: AlarmSeverity,

    /// Alarm status
    pub status: AlarmStatus,

    /// Engineering units
    pub units: Option<String>,

    /// Display limits
    pub limits: Option<PVLimits>,
}

impl PVValue {
    /// Create new PV value with double
    pub fn double(value: f64) -> Self {
        Self {
            value: PVData::Double(value),
            timestamp: chrono::Utc::now().timestamp_nanos_opt().unwrap_or(0),
            severity: AlarmSeverity::NoAlarm,
            status: AlarmStatus::NoAlarm,
            units: None,
            limits: None,
        }
    }

    /// Create with units
    pub fn with_units(mut self, units: &str) -> Self {
        self.units = Some(units.to_string());
        self
    }

    /// Create with limits
    pub fn with_limits(mut self, limits: PVLimits) -> Self {
        self.limits = Some(limits);
        self
    }

    /// Set alarm
    pub fn with_alarm(mut self, severity: AlarmSeverity, status: AlarmStatus) -> Self {
        self.severity = severity;
        self.status = status;
        self
    }
}

/// Monitor handle for subscription
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MonitorHandle(pub u64);

/// PV Monitor callback
pub type PVCallback = Box<dyn Fn(&PVValue) + Send + Sync>;

/// EPICS Integration Adapter
pub struct EPICSAdapter {
    state: IntegrationState,
    endpoint: String,
    protocol: EPICSProtocol,
    timeout_ms: u64,
    // Mock PV storage
    mock_pvs: HashMap<String, PVValue>,
    next_monitor_id: u64,
}

impl EPICSAdapter {
    /// Create new EPICS adapter
    pub fn new() -> Self {
        Self {
            state: IntegrationState::Disconnected,
            endpoint: "localhost:5064".to_string(),
            protocol: EPICSProtocol::ChannelAccess,
            timeout_ms: 5000,
            mock_pvs: HashMap::new(),
            next_monitor_id: 1,
        }
    }

    /// Create with endpoint
    pub fn with_endpoint(mut self, endpoint: &str) -> Self {
        self.endpoint = endpoint.to_string();
        self
    }

    /// Create with protocol
    pub fn with_protocol(mut self, protocol: EPICSProtocol) -> Self {
        self.protocol = protocol;
        self
    }

    /// Create with timeout
    pub fn with_timeout(mut self, timeout_ms: u64) -> Self {
        self.timeout_ms = timeout_ms;
        self
    }

    /// Get process variable value
    pub fn get_pv(&self, pv_name: &str) -> Result<PVValue, IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }

        self.mock_pvs
            .get(pv_name)
            .cloned()
            .ok_or_else(|| IntegrationError::IoError(format!("PV not found: {}", pv_name)))
    }

    /// Set process variable value
    pub fn put_pv(&mut self, pv_name: &str, value: PVValue) -> Result<(), IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }

        self.mock_pvs.insert(pv_name.to_string(), value);
        Ok(())
    }

    /// Start monitoring a PV (returns handle)
    pub fn monitor_pv(&mut self, pv_name: &str, _callback: PVCallback) -> Result<MonitorHandle, IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }

        // Check PV exists or create placeholder
        if !self.mock_pvs.contains_key(pv_name) {
            self.mock_pvs.insert(pv_name.to_string(), PVValue::double(0.0));
        }

        let handle = MonitorHandle(self.next_monitor_id);
        self.next_monitor_id += 1;
        Ok(handle)
    }

    /// Stop monitoring
    pub fn stop_monitor(&mut self, _handle: MonitorHandle) -> Result<(), IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }
        // In real implementation, would cancel subscription
        Ok(())
    }

    /// List PVs matching pattern
    pub fn list_pvs(&self, pattern: &str) -> Result<Vec<String>, IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }

        let pvs: Vec<String> = self.mock_pvs
            .keys()
            .filter(|k| {
                if pattern.contains('*') {
                    let prefix = pattern.trim_end_matches('*');
                    k.starts_with(prefix)
                } else {
                    k.contains(pattern)
                }
            })
            .cloned()
            .collect();

        Ok(pvs)
    }

    /// Get protocol
    pub fn protocol(&self) -> EPICSProtocol {
        self.protocol
    }

    /// Get endpoint
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    /// Convert fusion data to EPICS PVs
    pub fn fusion_to_pvs(data: &serde_json::Value, prefix: &str) -> Vec<(String, PVValue)> {
        let mut pvs = Vec::new();

        // Plasma data
        if let Some(plasma) = data.get("plasma") {
            if let Some(temp) = plasma.get("temperature") {
                if let Some(value) = temp.get("value").and_then(|v| v.as_f64()) {
                    let unit = temp.get("unit").and_then(|u| u.as_str()).unwrap_or("K");
                    pvs.push((
                        format!("{}:PLASMA:TEMP:CORE", prefix),
                        PVValue::double(value).with_units(unit),
                    ));
                }
            }

            if let Some(density) = plasma.get("density") {
                if let Some(value) = density.get("value").and_then(|v| v.as_f64()) {
                    let unit = density.get("unit").and_then(|u| u.as_str()).unwrap_or("m^-3");
                    pvs.push((
                        format!("{}:PLASMA:DENSITY:AVG", prefix),
                        PVValue::double(value).with_units(unit),
                    ));
                }
            }

            if let Some(confinement) = plasma.get("confinement_time") {
                if let Some(value) = confinement.get("value").and_then(|v| v.as_f64()) {
                    pvs.push((
                        format!("{}:PLASMA:CONFINEMENT", prefix),
                        PVValue::double(value).with_units("s"),
                    ));
                }
            }
        }

        // Magnetics data
        if let Some(magnetics) = data.get("magnetics") {
            if let Some(tf) = magnetics.get("toroidal_field") {
                if let Some(value) = tf.get("value").and_then(|v| v.as_f64()) {
                    pvs.push((
                        format!("{}:MAGNETS:TF:FIELD", prefix),
                        PVValue::double(value).with_units("T"),
                    ));
                }
            }

            if let Some(current) = magnetics.get("plasma_current") {
                if let Some(value) = current.get("value").and_then(|v| v.as_f64()) {
                    pvs.push((
                        format!("{}:MAGNETS:IP:CURRENT", prefix),
                        PVValue::double(value).with_units("MA"),
                    ));
                }
            }
        }

        // Energy data
        if let Some(energy) = data.get("energy") {
            if let Some(q) = energy.get("q_factor").and_then(|v| v.as_f64()) {
                pvs.push((
                    format!("{}:ENERGY:QFACTOR", prefix),
                    PVValue::double(q),
                ));
            }

            if let Some(heating) = energy.get("heating_power") {
                if let Some(value) = heating.get("value").and_then(|v| v.as_f64()) {
                    pvs.push((
                        format!("{}:HEATING:TOTAL", prefix),
                        PVValue::double(value).with_units("MW"),
                    ));
                }
            }
        }

        pvs
    }

    /// Export fusion data to EPICS
    pub fn export_fusion(&mut self, data: &serde_json::Value, prefix: &str) -> Result<u64, IntegrationError> {
        let pvs = Self::fusion_to_pvs(data, prefix);
        let count = pvs.len() as u64;

        for (name, value) in pvs {
            self.put_pv(&name, value)?;
        }

        Ok(count)
    }
}

impl Default for EPICSAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl IntegrationAdapter for EPICSAdapter {
    fn integration_type(&self) -> IntegrationType {
        IntegrationType::Epics
    }

    fn name(&self) -> &str {
        match self.protocol {
            EPICSProtocol::ChannelAccess => "EPICS Channel Access",
            EPICSProtocol::PvAccess => "EPICS pvAccess",
        }
    }

    fn state(&self) -> IntegrationState {
        self.state
    }

    fn initialize(&mut self, options: IntegrationOptions) -> Result<(), IntegrationError> {
        if let Some(endpoint) = options.endpoint {
            self.endpoint = endpoint;
        }

        if let Some(timeout) = options.timeout_ms {
            self.timeout_ms = timeout;
        }

        if let Some(proto) = options.extra.get("protocol").and_then(|v| v.as_str()) {
            self.protocol = match proto {
                "pva" | "pvaccess" => EPICSProtocol::PvAccess,
                _ => EPICSProtocol::ChannelAccess,
            };
        }

        Ok(())
    }

    fn connect(&mut self) -> Result<(), IntegrationError> {
        self.state = IntegrationState::Connecting;
        // In real implementation, would establish CA/PVA connection
        self.state = IntegrationState::Connected;
        Ok(())
    }

    fn disconnect(&mut self) -> Result<(), IntegrationError> {
        self.state = IntegrationState::Disconnected;
        self.mock_pvs.clear();
        Ok(())
    }

    fn export(&self, data: &PhysicsData) -> Result<ExportResult, IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }

        let prefix = data.metadata.experiment.as_deref().unwrap_or("WIA");
        let pvs = Self::fusion_to_pvs(&data.payload, prefix);

        Ok(ExportResult {
            success: true,
            records_exported: pvs.len() as u64,
            destination: self.endpoint.clone(),
            timestamp: chrono::Utc::now().timestamp_millis(),
            details: Some(format!("Exported {} PVs to EPICS", pvs.len())),
        })
    }

    fn import(&self, pv_name: &str) -> Result<PhysicsData, IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }

        let pv_value = self.get_pv(pv_name)?;

        Ok(PhysicsData {
            data_type: PhysicsDataType::Fusion,
            metadata: DataMetadata {
                id: format!("epics-{}", chrono::Utc::now().timestamp()),
                experiment: Some("EPICS Import".to_string()),
                created: Some(chrono::Utc::now().to_rfc3339()),
                ..Default::default()
            },
            payload: serde_json::json!({
                "pv_name": pv_name,
                "value": match pv_value.value {
                    PVData::Double(v) => serde_json::json!(v),
                    PVData::DoubleArray(v) => serde_json::json!(v),
                    PVData::Int(v) => serde_json::json!(v),
                    PVData::IntArray(v) => serde_json::json!(v),
                    PVData::String(v) => serde_json::json!(v),
                    PVData::Enum(v) => serde_json::json!(v),
                },
                "units": pv_value.units,
                "timestamp": pv_value.timestamp
            }),
        })
    }

    fn is_available(&self) -> bool {
        self.state == IntegrationState::Connected
    }

    fn dispose(&mut self) -> Result<(), IntegrationError> {
        self.disconnect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_epics_adapter_creation() {
        let adapter = EPICSAdapter::new();
        assert_eq!(adapter.state(), IntegrationState::Disconnected);
        assert_eq!(adapter.protocol(), EPICSProtocol::ChannelAccess);
    }

    #[test]
    fn test_pv_operations() {
        let mut adapter = EPICSAdapter::new();
        adapter.connect().unwrap();

        let value = PVValue::double(150e6)
            .with_units("K")
            .with_limits(PVLimits {
                low_alarm: 0.0,
                low_warn: 50e6,
                high_warn: 200e6,
                high_alarm: 250e6,
            });

        adapter.put_pv("ITER:PLASMA:TEMP:CORE", value).unwrap();

        let retrieved = adapter.get_pv("ITER:PLASMA:TEMP:CORE").unwrap();
        match retrieved.value {
            PVData::Double(v) => assert!((v - 150e6).abs() < 1.0),
            _ => panic!("Expected double value"),
        }
        assert_eq!(retrieved.units, Some("K".to_string()));
    }

    #[test]
    fn test_pv_list() {
        let mut adapter = EPICSAdapter::new();
        adapter.connect().unwrap();

        adapter.put_pv("ITER:PLASMA:TEMP:CORE", PVValue::double(150e6)).unwrap();
        adapter.put_pv("ITER:PLASMA:DENSITY:AVG", PVValue::double(1e20)).unwrap();
        adapter.put_pv("ITER:MAGNETS:TF:FIELD", PVValue::double(5.3)).unwrap();

        let plasma_pvs = adapter.list_pvs("ITER:PLASMA:*").unwrap();
        assert_eq!(plasma_pvs.len(), 2);

        let all_pvs = adapter.list_pvs("ITER").unwrap();
        assert_eq!(all_pvs.len(), 3);
    }

    #[test]
    fn test_fusion_to_pvs() {
        let data = serde_json::json!({
            "plasma": {
                "temperature": {"value": 150e6, "unit": "K"},
                "density": {"value": 1e20, "unit": "m^-3"}
            },
            "magnetics": {
                "toroidal_field": {"value": 5.3, "unit": "T"}
            },
            "energy": {
                "q_factor": 10.0
            }
        });

        let pvs = EPICSAdapter::fusion_to_pvs(&data, "ITER");
        assert_eq!(pvs.len(), 4);

        let pv_names: Vec<&str> = pvs.iter().map(|(n, _)| n.as_str()).collect();
        assert!(pv_names.contains(&"ITER:PLASMA:TEMP:CORE"));
        assert!(pv_names.contains(&"ITER:PLASMA:DENSITY:AVG"));
        assert!(pv_names.contains(&"ITER:MAGNETS:TF:FIELD"));
        assert!(pv_names.contains(&"ITER:ENERGY:QFACTOR"));
    }

    #[test]
    fn test_alarm_states() {
        let value = PVValue::double(300e6)
            .with_alarm(AlarmSeverity::Major, AlarmStatus::HiHi);

        assert_eq!(value.severity, AlarmSeverity::Major);
        assert_eq!(value.status, AlarmStatus::HiHi);
    }
}
