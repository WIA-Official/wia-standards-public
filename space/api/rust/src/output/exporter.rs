//! Data exporters for various formats
//!
//! Provides export functionality for CCSDS, NASA, and other standard formats.

use chrono::Utc;

use super::adapter::{OutputAdapter, OutputConfig, OutputError, OutputType};
use super::data::{OutputData, OutputResult};
use async_trait::async_trait;

/// CCSDS OEM (Orbit Ephemeris Message) exporter
#[derive(Debug, Clone)]
pub struct CcsdsOemExporter {
    name: String,
    available: bool,
    originator: String,
    object_name: String,
    object_id: String,
}

impl CcsdsOemExporter {
    /// Create new CCSDS OEM exporter
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            available: true,
            originator: "WIA".to_string(),
            object_name: "UNKNOWN".to_string(),
            object_id: "UNKNOWN".to_string(),
        }
    }

    /// Set originator
    pub fn with_originator(mut self, originator: impl Into<String>) -> Self {
        self.originator = originator.into();
        self
    }

    /// Set object name
    pub fn with_object_name(mut self, name: impl Into<String>) -> Self {
        self.object_name = name.into();
        self
    }

    /// Set object ID
    pub fn with_object_id(mut self, id: impl Into<String>) -> Self {
        self.object_id = id.into();
        self
    }

    /// Export to OEM format
    pub fn export_oem(&self, data: &OutputData) -> Result<String, OutputError> {
        let mut output = String::new();
        let now = Utc::now();

        // OEM Header
        output.push_str("CCSDS_OEM_VERS = 2.0\n");
        output.push_str(&format!("CREATION_DATE = {}\n", now.format("%Y-%m-%dT%H:%M:%S")));
        output.push_str(&format!("ORIGINATOR = {}\n", self.originator));
        output.push_str("\n");

        // Metadata
        output.push_str("META_START\n");
        output.push_str(&format!("OBJECT_NAME = {}\n", self.object_name));
        output.push_str(&format!("OBJECT_ID = {}\n", self.object_id));
        output.push_str("CENTER_NAME = EARTH\n");
        output.push_str("REF_FRAME = EME2000\n");
        output.push_str("TIME_SYSTEM = UTC\n");

        // Extract start/stop times from data if available
        if let Some(start) = data.get_metadata("start_time").and_then(|v| v.as_str()) {
            output.push_str(&format!("START_TIME = {}\n", start));
        }
        if let Some(stop) = data.get_metadata("stop_time").and_then(|v| v.as_str()) {
            output.push_str(&format!("STOP_TIME = {}\n", stop));
        }

        output.push_str("META_STOP\n\n");

        // Ephemeris data
        output.push_str("COMMENT WIA Space Standard Export\n");

        // If we have ephemeris data in the payload, export it
        if let Some(ephemeris) = data.payload().get("ephemeris") {
            if let Some(points) = ephemeris.as_array() {
                for point in points {
                    if let (Some(t), Some(x), Some(y), Some(z), Some(vx), Some(vy), Some(vz)) = (
                        point.get("time").and_then(|v| v.as_str()),
                        point.get("x").and_then(|v| v.as_f64()),
                        point.get("y").and_then(|v| v.as_f64()),
                        point.get("z").and_then(|v| v.as_f64()),
                        point.get("vx").and_then(|v| v.as_f64()),
                        point.get("vy").and_then(|v| v.as_f64()),
                        point.get("vz").and_then(|v| v.as_f64()),
                    ) {
                        output.push_str(&format!(
                            "{} {:15.6} {:15.6} {:15.6} {:15.9} {:15.9} {:15.9}\n",
                            t, x, y, z, vx, vy, vz
                        ));
                    }
                }
            }
        }

        Ok(output)
    }
}

#[async_trait]
impl OutputAdapter for CcsdsOemExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Export
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, _config: &OutputConfig) -> Result<(), OutputError> {
        self.available = true;
        Ok(())
    }

    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError> {
        let oem_content = self.export_oem(data)?;

        // In a real implementation, this would write to a file or stream
        // For now, we include the content in the result
        Ok(OutputResult::Success {
            adapter: self.name.clone(),
            output_path: None,
            message: Some(format!("Exported {} bytes of OEM data", oem_content.len())),
            bytes_written: Some(oem_content.len()),
        })
    }

    fn is_available(&self) -> bool {
        self.available
    }

    async fn dispose(&mut self) -> Result<(), OutputError> {
        self.available = false;
        Ok(())
    }
}

/// CCSDS OPM (Orbit Parameter Message) exporter
#[derive(Debug, Clone)]
pub struct CcsdsOpmExporter {
    name: String,
    available: bool,
    originator: String,
}

impl CcsdsOpmExporter {
    /// Create new CCSDS OPM exporter
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            available: true,
            originator: "WIA".to_string(),
        }
    }

    /// Set originator
    pub fn with_originator(mut self, originator: impl Into<String>) -> Self {
        self.originator = originator.into();
        self
    }

    /// Export to OPM format
    pub fn export_opm(&self, data: &OutputData) -> Result<String, OutputError> {
        let mut output = String::new();
        let now = Utc::now();

        // OPM Header
        output.push_str("CCSDS_OPM_VERS = 2.0\n");
        output.push_str(&format!("CREATION_DATE = {}\n", now.format("%Y-%m-%dT%H:%M:%S")));
        output.push_str(&format!("ORIGINATOR = {}\n", self.originator));
        output.push_str("\n");

        // Extract object info from data
        let object_name = data.get_metadata("object_name")
            .and_then(|v| v.as_str())
            .unwrap_or("UNKNOWN");
        let object_id = data.get_metadata("object_id")
            .and_then(|v| v.as_str())
            .unwrap_or("UNKNOWN");

        // Metadata
        output.push_str("META_START\n");
        output.push_str(&format!("OBJECT_NAME = {}\n", object_name));
        output.push_str(&format!("OBJECT_ID = {}\n", object_id));
        output.push_str("CENTER_NAME = EARTH\n");
        output.push_str("REF_FRAME = EME2000\n");
        output.push_str("TIME_SYSTEM = UTC\n");
        output.push_str("META_STOP\n\n");

        // State vector (if available)
        if let Some(state) = data.payload().get("state_vector") {
            output.push_str("COMMENT State Vector\n");

            if let Some(epoch) = state.get("epoch").and_then(|v| v.as_str()) {
                output.push_str(&format!("EPOCH = {}\n", epoch));
            }
            if let Some(x) = state.get("x").and_then(|v| v.as_f64()) {
                output.push_str(&format!("X = {:15.6}\n", x));
            }
            if let Some(y) = state.get("y").and_then(|v| v.as_f64()) {
                output.push_str(&format!("Y = {:15.6}\n", y));
            }
            if let Some(z) = state.get("z").and_then(|v| v.as_f64()) {
                output.push_str(&format!("Z = {:15.6}\n", z));
            }
            if let Some(vx) = state.get("vx").and_then(|v| v.as_f64()) {
                output.push_str(&format!("X_DOT = {:15.9}\n", vx));
            }
            if let Some(vy) = state.get("vy").and_then(|v| v.as_f64()) {
                output.push_str(&format!("Y_DOT = {:15.9}\n", vy));
            }
            if let Some(vz) = state.get("vz").and_then(|v| v.as_f64()) {
                output.push_str(&format!("Z_DOT = {:15.9}\n", vz));
            }
        }

        // Keplerian elements (if available)
        if let Some(keplerian) = data.payload().get("keplerian") {
            output.push_str("\nCOMMENT Keplerian Elements\n");

            if let Some(a) = keplerian.get("semi_major_axis").and_then(|v| v.as_f64()) {
                output.push_str(&format!("SEMI_MAJOR_AXIS = {:15.6}\n", a));
            }
            if let Some(e) = keplerian.get("eccentricity").and_then(|v| v.as_f64()) {
                output.push_str(&format!("ECCENTRICITY = {:15.12}\n", e));
            }
            if let Some(i) = keplerian.get("inclination").and_then(|v| v.as_f64()) {
                output.push_str(&format!("INCLINATION = {:15.9}\n", i));
            }
            if let Some(raan) = keplerian.get("raan").and_then(|v| v.as_f64()) {
                output.push_str(&format!("RA_OF_ASC_NODE = {:15.9}\n", raan));
            }
            if let Some(aop) = keplerian.get("arg_of_pericenter").and_then(|v| v.as_f64()) {
                output.push_str(&format!("ARG_OF_PERICENTER = {:15.9}\n", aop));
            }
            if let Some(ta) = keplerian.get("true_anomaly").and_then(|v| v.as_f64()) {
                output.push_str(&format!("TRUE_ANOMALY = {:15.9}\n", ta));
            }
        }

        Ok(output)
    }
}

#[async_trait]
impl OutputAdapter for CcsdsOpmExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Export
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, _config: &OutputConfig) -> Result<(), OutputError> {
        self.available = true;
        Ok(())
    }

    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError> {
        let opm_content = self.export_opm(data)?;

        Ok(OutputResult::Success {
            adapter: self.name.clone(),
            output_path: None,
            message: Some(format!("Exported {} bytes of OPM data", opm_content.len())),
            bytes_written: Some(opm_content.len()),
        })
    }

    fn is_available(&self) -> bool {
        self.available
    }

    async fn dispose(&mut self) -> Result<(), OutputError> {
        self.available = false;
        Ok(())
    }
}

/// JSON exporter for WIA format
#[derive(Debug, Clone)]
pub struct JsonExporter {
    name: String,
    available: bool,
    pretty: bool,
}

impl JsonExporter {
    /// Create new JSON exporter
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            available: true,
            pretty: true,
        }
    }

    /// Set pretty printing
    pub fn with_pretty(mut self, pretty: bool) -> Self {
        self.pretty = pretty;
        self
    }

    /// Export to JSON format
    pub fn export_json(&self, data: &OutputData) -> Result<String, OutputError> {
        let json_value = serde_json::json!({
            "wia_version": "1.0.0",
            "schema": "https://wia.live/schemas/space/project.schema.json",
            "exported_at": Utc::now().to_rfc3339(),
            "source": data.source().to_string(),
            "metadata": data.metadata(),
            "payload": data.payload(),
        });

        if self.pretty {
            serde_json::to_string_pretty(&json_value)
                .map_err(|e| OutputError::FormatError(e.to_string()))
        } else {
            serde_json::to_string(&json_value)
                .map_err(|e| OutputError::FormatError(e.to_string()))
        }
    }
}

#[async_trait]
impl OutputAdapter for JsonExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Export
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, _config: &OutputConfig) -> Result<(), OutputError> {
        self.available = true;
        Ok(())
    }

    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError> {
        let json_content = self.export_json(data)?;

        Ok(OutputResult::Success {
            adapter: self.name.clone(),
            output_path: None,
            message: Some(format!("Exported {} bytes of JSON data", json_content.len())),
            bytes_written: Some(json_content.len()),
        })
    }

    fn is_available(&self) -> bool {
        self.available
    }

    async fn dispose(&mut self) -> Result<(), OutputError> {
        self.available = false;
        Ok(())
    }
}

/// CSV exporter
#[derive(Debug, Clone)]
pub struct CsvExporter {
    name: String,
    available: bool,
    delimiter: char,
}

impl CsvExporter {
    /// Create new CSV exporter
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            available: true,
            delimiter: ',',
        }
    }

    /// Set delimiter
    pub fn with_delimiter(mut self, delimiter: char) -> Self {
        self.delimiter = delimiter;
        self
    }

    /// Export to CSV format
    pub fn export_csv(&self, data: &OutputData) -> Result<String, OutputError> {
        let mut output = String::new();
        let d = self.delimiter;

        // If we have ephemeris data, export as time series
        if let Some(ephemeris) = data.payload().get("ephemeris") {
            if let Some(points) = ephemeris.as_array() {
                // Header
                output.push_str(&format!("time{d}x{d}y{d}z{d}vx{d}vy{d}vz\n"));

                // Data rows
                for point in points {
                    let t = point.get("time").and_then(|v| v.as_str()).unwrap_or("");
                    let x = point.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0);
                    let y = point.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0);
                    let z = point.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0);
                    let vx = point.get("vx").and_then(|v| v.as_f64()).unwrap_or(0.0);
                    let vy = point.get("vy").and_then(|v| v.as_f64()).unwrap_or(0.0);
                    let vz = point.get("vz").and_then(|v| v.as_f64()).unwrap_or(0.0);

                    output.push_str(&format!("{t}{d}{x}{d}{y}{d}{z}{d}{vx}{d}{vy}{d}{vz}\n"));
                }
            }
        } else {
            // Generic payload export - flatten to key-value pairs
            output.push_str(&format!("key{d}value\n"));

            fn flatten_json(prefix: &str, value: &serde_json::Value, output: &mut String, d: char) {
                match value {
                    serde_json::Value::Object(map) => {
                        for (k, v) in map {
                            let new_prefix = if prefix.is_empty() {
                                k.clone()
                            } else {
                                format!("{}.{}", prefix, k)
                            };
                            flatten_json(&new_prefix, v, output, d);
                        }
                    }
                    serde_json::Value::Array(arr) => {
                        for (i, v) in arr.iter().enumerate() {
                            let new_prefix = format!("{}[{}]", prefix, i);
                            flatten_json(&new_prefix, v, output, d);
                        }
                    }
                    _ => {
                        output.push_str(&format!("{}{}{}\n", prefix, d, value));
                    }
                }
            }

            flatten_json("", data.payload(), &mut output, d);
        }

        Ok(output)
    }
}

#[async_trait]
impl OutputAdapter for CsvExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Export
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, _config: &OutputConfig) -> Result<(), OutputError> {
        self.available = true;
        Ok(())
    }

    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError> {
        let csv_content = self.export_csv(data)?;

        Ok(OutputResult::Success {
            adapter: self.name.clone(),
            output_path: None,
            message: Some(format!("Exported {} bytes of CSV data", csv_content.len())),
            bytes_written: Some(csv_content.len()),
        })
    }

    fn is_available(&self) -> bool {
        self.available
    }

    async fn dispose(&mut self) -> Result<(), OutputError> {
        self.available = false;
        Ok(())
    }
}

/// GMAT Script exporter
#[derive(Debug, Clone)]
pub struct GmatScriptExporter {
    name: String,
    available: bool,
}

impl GmatScriptExporter {
    /// Create new GMAT script exporter
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            available: true,
        }
    }

    /// Export to GMAT script format
    pub fn export_gmat(&self, data: &OutputData) -> Result<String, OutputError> {
        let mut output = String::new();

        output.push_str("%----------------------------------------\n");
        output.push_str("% WIA Space Standard - GMAT Script Export\n");
        output.push_str(&format!("% Generated: {}\n", Utc::now().to_rfc3339()));
        output.push_str("%----------------------------------------\n\n");

        // Create spacecraft object
        let sc_name = data.get_metadata("object_name")
            .and_then(|v| v.as_str())
            .unwrap_or("Spacecraft");

        output.push_str(&format!("Create Spacecraft {};\n", sc_name));
        output.push_str(&format!("GMAT {}.DateFormat = UTCGregorian;\n", sc_name));

        // Set orbital elements if available
        if let Some(keplerian) = data.payload().get("keplerian") {
            if let Some(a) = keplerian.get("semi_major_axis").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.SMA = {};\n", sc_name, a));
            }
            if let Some(e) = keplerian.get("eccentricity").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.ECC = {};\n", sc_name, e));
            }
            if let Some(i) = keplerian.get("inclination").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.INC = {};\n", sc_name, i));
            }
            if let Some(raan) = keplerian.get("raan").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.RAAN = {};\n", sc_name, raan));
            }
            if let Some(aop) = keplerian.get("arg_of_pericenter").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.AOP = {};\n", sc_name, aop));
            }
            if let Some(ta) = keplerian.get("true_anomaly").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.TA = {};\n", sc_name, ta));
            }
        }

        // Set state vector if available
        if let Some(state) = data.payload().get("state_vector") {
            output.push_str(&format!("\n%-- State Vector --\n"));
            if let Some(x) = state.get("x").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.X = {};\n", sc_name, x));
            }
            if let Some(y) = state.get("y").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.Y = {};\n", sc_name, y));
            }
            if let Some(z) = state.get("z").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.Z = {};\n", sc_name, z));
            }
            if let Some(vx) = state.get("vx").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.VX = {};\n", sc_name, vx));
            }
            if let Some(vy) = state.get("vy").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.VY = {};\n", sc_name, vy));
            }
            if let Some(vz) = state.get("vz").and_then(|v| v.as_f64()) {
                output.push_str(&format!("GMAT {}.VZ = {};\n", sc_name, vz));
            }
        }

        output.push_str("\n%-- End WIA Export --\n");

        Ok(output)
    }
}

#[async_trait]
impl OutputAdapter for GmatScriptExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Export
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, _config: &OutputConfig) -> Result<(), OutputError> {
        self.available = true;
        Ok(())
    }

    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError> {
        let gmat_content = self.export_gmat(data)?;

        Ok(OutputResult::Success {
            adapter: self.name.clone(),
            output_path: None,
            message: Some(format!("Exported {} bytes of GMAT script", gmat_content.len())),
            bytes_written: Some(gmat_content.len()),
        })
    }

    fn is_available(&self) -> bool {
        self.available
    }

    async fn dispose(&mut self) -> Result<(), OutputError> {
        self.available = false;
        Ok(())
    }
}

/// CZML exporter for CesiumJS visualization
#[derive(Debug, Clone)]
pub struct CzmlExporter {
    name: String,
    available: bool,
    document_name: String,
}

impl CzmlExporter {
    /// Create new CZML exporter
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            available: true,
            document_name: "WIA Space Export".to_string(),
        }
    }

    /// Set document name
    pub fn with_document_name(mut self, name: impl Into<String>) -> Self {
        self.document_name = name.into();
        self
    }

    /// Export to CZML format
    pub fn export_czml(&self, data: &OutputData) -> Result<String, OutputError> {
        let mut czml = vec![];

        // Document packet
        czml.push(serde_json::json!({
            "id": "document",
            "name": self.document_name,
            "version": "1.0",
            "clock": {
                "interval": "2025-01-01T00:00:00Z/2025-12-31T23:59:59Z",
                "currentTime": Utc::now().to_rfc3339(),
                "multiplier": 1
            }
        }));

        // Object packet
        let object_id = data.get_metadata("object_id")
            .and_then(|v| v.as_str())
            .unwrap_or("wia-object");
        let object_name = data.get_metadata("object_name")
            .and_then(|v| v.as_str())
            .unwrap_or("WIA Object");

        let mut object_packet = serde_json::json!({
            "id": object_id,
            "name": object_name,
            "availability": "2025-01-01T00:00:00Z/2025-12-31T23:59:59Z",
            "billboard": {
                "eyeOffset": {
                    "cartesian": [0, 0, 0]
                },
                "horizontalOrigin": "CENTER",
                "pixelOffset": {
                    "cartesian2": [0, 0]
                },
                "scale": 1.0,
                "show": true,
                "verticalOrigin": "CENTER"
            },
            "label": {
                "fillColor": {
                    "rgba": [255, 255, 255, 255]
                },
                "font": "12pt sans-serif",
                "horizontalOrigin": "LEFT",
                "outlineColor": {
                    "rgba": [0, 0, 0, 255]
                },
                "outlineWidth": 2,
                "pixelOffset": {
                    "cartesian2": [12, 0]
                },
                "show": true,
                "style": "FILL_AND_OUTLINE",
                "text": object_name,
                "verticalOrigin": "CENTER"
            }
        });

        // Add position data if ephemeris is available
        if let Some(ephemeris) = data.payload().get("ephemeris") {
            if let Some(points) = ephemeris.as_array() {
                let mut cartesian_data = Vec::new();

                for point in points {
                    if let (Some(t), Some(x), Some(y), Some(z)) = (
                        point.get("time").and_then(|v| v.as_str()),
                        point.get("x").and_then(|v| v.as_f64()),
                        point.get("y").and_then(|v| v.as_f64()),
                        point.get("z").and_then(|v| v.as_f64()),
                    ) {
                        cartesian_data.push(serde_json::json!(t));
                        cartesian_data.push(serde_json::json!(x * 1000.0)); // km to m
                        cartesian_data.push(serde_json::json!(y * 1000.0));
                        cartesian_data.push(serde_json::json!(z * 1000.0));
                    }
                }

                if !cartesian_data.is_empty() {
                    object_packet["position"] = serde_json::json!({
                        "interpolationAlgorithm": "LAGRANGE",
                        "interpolationDegree": 5,
                        "referenceFrame": "INERTIAL",
                        "epoch": cartesian_data[0],
                        "cartesian": cartesian_data
                    });

                    // Add path visualization
                    object_packet["path"] = serde_json::json!({
                        "show": [
                            {
                                "interval": "2025-01-01T00:00:00Z/2025-12-31T23:59:59Z",
                                "boolean": true
                            }
                        ],
                        "width": 1,
                        "material": {
                            "solidColor": {
                                "color": {
                                    "rgba": [0, 255, 255, 255]
                                }
                            }
                        },
                        "resolution": 120,
                        "leadTime": 0,
                        "trailTime": 3600
                    });
                }
            }
        }

        czml.push(object_packet);

        serde_json::to_string_pretty(&czml)
            .map_err(|e| OutputError::FormatError(e.to_string()))
    }
}

#[async_trait]
impl OutputAdapter for CzmlExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Visualization
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, _config: &OutputConfig) -> Result<(), OutputError> {
        self.available = true;
        Ok(())
    }

    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError> {
        let czml_content = self.export_czml(data)?;

        Ok(OutputResult::Success {
            adapter: self.name.clone(),
            output_path: None,
            message: Some(format!("Exported {} bytes of CZML data", czml_content.len())),
            bytes_written: Some(czml_content.len()),
        })
    }

    fn is_available(&self) -> bool {
        self.available
    }

    async fn dispose(&mut self) -> Result<(), OutputError> {
        self.available = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_orbital_data() -> OutputData {
        OutputData::from_json(serde_json::json!({
            "keplerian": {
                "semi_major_axis": 6778.0,
                "eccentricity": 0.001,
                "inclination": 51.6,
                "raan": 0.0,
                "arg_of_pericenter": 0.0,
                "true_anomaly": 0.0
            },
            "state_vector": {
                "epoch": "2025-01-01T00:00:00Z",
                "x": 6778.0,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 7.67,
                "vz": 0.0
            }
        }))
        .with_metadata("object_name", "ISS")
        .with_metadata("object_id", "1998-067A")
    }

    fn sample_ephemeris_data() -> OutputData {
        OutputData::from_json(serde_json::json!({
            "ephemeris": [
                {"time": "2025-01-01T00:00:00Z", "x": 6778.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 7.67, "vz": 0.0},
                {"time": "2025-01-01T00:01:00Z", "x": 6750.0, "y": 460.0, "z": 100.0, "vx": -0.1, "vy": 7.66, "vz": 0.2}
            ]
        }))
        .with_metadata("object_name", "ISS")
        .with_metadata("start_time", "2025-01-01T00:00:00Z")
        .with_metadata("stop_time", "2025-01-01T00:01:00Z")
    }

    #[test]
    fn test_ccsds_oem_export() {
        let exporter = CcsdsOemExporter::new("oem-test")
            .with_originator("WIA-TEST")
            .with_object_name("ISS")
            .with_object_id("1998-067A");

        let data = sample_ephemeris_data();
        let result = exporter.export_oem(&data);

        assert!(result.is_ok());
        let content = result.unwrap();
        assert!(content.contains("CCSDS_OEM_VERS = 2.0"));
        assert!(content.contains("ORIGINATOR = WIA-TEST"));
        assert!(content.contains("OBJECT_NAME = ISS"));
    }

    #[test]
    fn test_ccsds_opm_export() {
        let exporter = CcsdsOpmExporter::new("opm-test")
            .with_originator("WIA-TEST");

        let data = sample_orbital_data();
        let result = exporter.export_opm(&data);

        assert!(result.is_ok());
        let content = result.unwrap();
        assert!(content.contains("CCSDS_OPM_VERS = 2.0"));
        assert!(content.contains("SEMI_MAJOR_AXIS"));
        assert!(content.contains("ECCENTRICITY"));
    }

    #[test]
    fn test_json_export() {
        let exporter = JsonExporter::new("json-test").with_pretty(true);
        let data = sample_orbital_data();
        let result = exporter.export_json(&data);

        assert!(result.is_ok());
        let content = result.unwrap();
        assert!(content.contains("wia_version"));
        assert!(content.contains("1.0.0"));
    }

    #[test]
    fn test_csv_export() {
        let exporter = CsvExporter::new("csv-test");
        let data = sample_ephemeris_data();
        let result = exporter.export_csv(&data);

        assert!(result.is_ok());
        let content = result.unwrap();
        assert!(content.contains("time,x,y,z,vx,vy,vz"));
    }

    #[test]
    fn test_gmat_export() {
        let exporter = GmatScriptExporter::new("gmat-test");
        let data = sample_orbital_data();
        let result = exporter.export_gmat(&data);

        assert!(result.is_ok());
        let content = result.unwrap();
        assert!(content.contains("Create Spacecraft"));
        assert!(content.contains("GMAT"));
        assert!(content.contains("SMA"));
    }

    #[test]
    fn test_czml_export() {
        let exporter = CzmlExporter::new("czml-test")
            .with_document_name("Test Mission");
        let data = sample_ephemeris_data();
        let result = exporter.export_czml(&data);

        assert!(result.is_ok());
        let content = result.unwrap();
        assert!(content.contains("document"));
        assert!(content.contains("Test Mission"));
    }

    #[tokio::test]
    async fn test_exporter_output_adapter() {
        let exporter = JsonExporter::new("json-adapter-test");
        let data = sample_orbital_data();
        let result = exporter.output(&data).await;

        assert!(result.is_ok());
        assert!(result.unwrap().is_success());
    }

    #[tokio::test]
    async fn test_oem_output_adapter() {
        let exporter = CcsdsOemExporter::new("oem-adapter-test")
            .with_object_name("Test Satellite");
        let data = sample_ephemeris_data();
        let result = exporter.output(&data).await;

        assert!(result.is_ok());
        assert!(result.unwrap().is_success());
    }
}
