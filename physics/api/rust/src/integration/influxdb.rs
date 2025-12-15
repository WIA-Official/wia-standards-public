//! InfluxDB Integration Adapter
//!
//! Provides time-series database integration for real-time physics data monitoring.

use super::traits::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Field value for InfluxDB
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum FieldValue {
    Float(f64),
    Int(i64),
    Bool(bool),
    String(String),
}

/// Single data point for InfluxDB
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPoint {
    /// Measurement name
    pub measurement: String,

    /// Tags (indexed strings)
    pub tags: HashMap<String, String>,

    /// Fields (values)
    pub fields: HashMap<String, FieldValue>,

    /// Timestamp (nanoseconds since epoch)
    pub timestamp: i64,
}

impl DataPoint {
    /// Create new data point
    pub fn new(measurement: &str) -> Self {
        Self {
            measurement: measurement.to_string(),
            tags: HashMap::new(),
            fields: HashMap::new(),
            timestamp: chrono::Utc::now().timestamp_nanos_opt().unwrap_or(0),
        }
    }

    /// Add tag
    pub fn tag(mut self, key: &str, value: &str) -> Self {
        self.tags.insert(key.to_string(), value.to_string());
        self
    }

    /// Add float field
    pub fn field_float(mut self, key: &str, value: f64) -> Self {
        self.fields.insert(key.to_string(), FieldValue::Float(value));
        self
    }

    /// Add integer field
    pub fn field_int(mut self, key: &str, value: i64) -> Self {
        self.fields.insert(key.to_string(), FieldValue::Int(value));
        self
    }

    /// Add boolean field
    pub fn field_bool(mut self, key: &str, value: bool) -> Self {
        self.fields.insert(key.to_string(), FieldValue::Bool(value));
        self
    }

    /// Add string field
    pub fn field_string(mut self, key: &str, value: &str) -> Self {
        self.fields.insert(key.to_string(), FieldValue::String(value.to_string()));
        self
    }

    /// Set timestamp
    pub fn at(mut self, timestamp: i64) -> Self {
        self.timestamp = timestamp;
        self
    }

    /// Convert to InfluxDB line protocol
    pub fn to_line_protocol(&self) -> String {
        let mut line = self.measurement.clone();

        // Add tags
        if !self.tags.is_empty() {
            let tags: Vec<String> = self.tags
                .iter()
                .map(|(k, v)| format!("{}={}", k, v))
                .collect();
            line.push(',');
            line.push_str(&tags.join(","));
        }

        // Add fields
        if !self.fields.is_empty() {
            let fields: Vec<String> = self.fields
                .iter()
                .map(|(k, v)| {
                    match v {
                        FieldValue::Float(f) => format!("{}={}", k, f),
                        FieldValue::Int(i) => format!("{}={}i", k, i),
                        FieldValue::Bool(b) => format!("{}={}", k, b),
                        FieldValue::String(s) => format!("{}=\"{}\"", k, s),
                    }
                })
                .collect();
            line.push(' ');
            line.push_str(&fields.join(","));
        }

        // Add timestamp
        line.push(' ');
        line.push_str(&self.timestamp.to_string());

        line
    }
}

/// Time range for queries
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeRange {
    pub start: i64,
    pub end: i64,
}

/// Aggregation type
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Aggregation {
    Mean,
    Sum,
    Min,
    Max,
    Count,
    First,
    Last,
}

/// Query for time-series data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TimeSeriesQuery {
    /// Measurement name
    pub measurement: String,

    /// Fields to select
    pub fields: Vec<String>,

    /// Time range
    pub time_range: Option<TimeRange>,

    /// Tag filters
    pub filters: HashMap<String, String>,

    /// Aggregation function
    pub aggregation: Option<Aggregation>,

    /// Group by time interval (e.g., "1m", "5s")
    pub group_by: Option<String>,

    /// Result limit
    pub limit: Option<u64>,
}

impl TimeSeriesQuery {
    /// Create new query
    pub fn new(measurement: &str) -> Self {
        Self {
            measurement: measurement.to_string(),
            ..Default::default()
        }
    }

    /// Select fields
    pub fn select(mut self, fields: Vec<&str>) -> Self {
        self.fields = fields.into_iter().map(String::from).collect();
        self
    }

    /// Set time range
    pub fn range(mut self, start: i64, end: i64) -> Self {
        self.time_range = Some(TimeRange { start, end });
        self
    }

    /// Add filter
    pub fn filter(mut self, key: &str, value: &str) -> Self {
        self.filters.insert(key.to_string(), value.to_string());
        self
    }

    /// Set aggregation
    pub fn aggregate(mut self, agg: Aggregation) -> Self {
        self.aggregation = Some(agg);
        self
    }

    /// Set group by interval
    pub fn group_by_time(mut self, interval: &str) -> Self {
        self.group_by = Some(interval.to_string());
        self
    }

    /// Set limit
    pub fn limit(mut self, limit: u64) -> Self {
        self.limit = Some(limit);
        self
    }

    /// Convert to InfluxQL
    pub fn to_influxql(&self) -> String {
        let fields_str = if self.fields.is_empty() {
            "*".to_string()
        } else {
            match self.aggregation {
                Some(agg) => {
                    let agg_fn = match agg {
                        Aggregation::Mean => "MEAN",
                        Aggregation::Sum => "SUM",
                        Aggregation::Min => "MIN",
                        Aggregation::Max => "MAX",
                        Aggregation::Count => "COUNT",
                        Aggregation::First => "FIRST",
                        Aggregation::Last => "LAST",
                    };
                    self.fields.iter().map(|f| format!("{}({})", agg_fn, f)).collect::<Vec<_>>().join(", ")
                }
                None => self.fields.join(", ")
            }
        };

        let mut query = format!("SELECT {} FROM {}", fields_str, self.measurement);

        // Add WHERE clause
        let mut conditions = Vec::new();

        if let Some(ref range) = self.time_range {
            conditions.push(format!("time >= {}ns AND time <= {}ns", range.start, range.end));
        }

        for (key, value) in &self.filters {
            conditions.push(format!("{} = '{}'", key, value));
        }

        if !conditions.is_empty() {
            query.push_str(" WHERE ");
            query.push_str(&conditions.join(" AND "));
        }

        // Add GROUP BY
        if let Some(ref group_by) = self.group_by {
            query.push_str(&format!(" GROUP BY time({})", group_by));
        }

        // Add LIMIT
        if let Some(limit) = self.limit {
            query.push_str(&format!(" LIMIT {}", limit));
        }

        query
    }
}

/// Query result
#[derive(Debug, Clone)]
pub struct QueryResult {
    pub series: Vec<Series>,
}

/// Series in query result
#[derive(Debug, Clone)]
pub struct Series {
    pub name: String,
    pub tags: HashMap<String, String>,
    pub columns: Vec<String>,
    pub values: Vec<Vec<serde_json::Value>>,
}

/// InfluxDB Integration Adapter
pub struct InfluxDBAdapter {
    state: IntegrationState,
    endpoint: String,
    database: String,
    token: Option<String>,
    org: Option<String>,
    // Mock storage
    mock_points: Vec<DataPoint>,
}

impl InfluxDBAdapter {
    /// Create new InfluxDB adapter
    pub fn new(endpoint: &str, database: &str) -> Self {
        Self {
            state: IntegrationState::Disconnected,
            endpoint: endpoint.to_string(),
            database: database.to_string(),
            token: None,
            org: None,
            mock_points: Vec::new(),
        }
    }

    /// Set authentication token
    pub fn with_token(mut self, token: &str) -> Self {
        self.token = Some(token.to_string());
        self
    }

    /// Set organization
    pub fn with_org(mut self, org: &str) -> Self {
        self.org = Some(org.to_string());
        self
    }

    /// Write single point
    pub fn write_point(&mut self, point: DataPoint) -> Result<(), IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }
        self.mock_points.push(point);
        Ok(())
    }

    /// Write batch of points
    pub fn write_batch(&mut self, points: Vec<DataPoint>) -> Result<(), IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }
        self.mock_points.extend(points);
        Ok(())
    }

    /// Query data
    pub fn query(&self, query: &TimeSeriesQuery) -> Result<QueryResult, IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }

        // Mock query - filter points by measurement
        let filtered: Vec<&DataPoint> = self.mock_points
            .iter()
            .filter(|p| p.measurement == query.measurement)
            .collect();

        let columns = if query.fields.is_empty() {
            vec!["time".to_string()]
        } else {
            let mut cols = vec!["time".to_string()];
            cols.extend(query.fields.clone());
            cols
        };

        let values: Vec<Vec<serde_json::Value>> = filtered
            .iter()
            .map(|p| {
                let mut row = vec![serde_json::json!(p.timestamp)];
                for field in &query.fields {
                    if let Some(val) = p.fields.get(field) {
                        row.push(match val {
                            FieldValue::Float(f) => serde_json::json!(f),
                            FieldValue::Int(i) => serde_json::json!(i),
                            FieldValue::Bool(b) => serde_json::json!(b),
                            FieldValue::String(s) => serde_json::json!(s),
                        });
                    }
                }
                row
            })
            .collect();

        Ok(QueryResult {
            series: vec![Series {
                name: query.measurement.clone(),
                tags: HashMap::new(),
                columns,
                values,
            }],
        })
    }

    /// Convert fusion data to InfluxDB points
    pub fn fusion_to_points(data: &serde_json::Value, tags: HashMap<String, String>) -> Vec<DataPoint> {
        let mut points = Vec::new();
        let timestamp = chrono::Utc::now().timestamp_nanos_opt().unwrap_or(0);

        // Plasma data
        if let Some(plasma) = data.get("plasma") {
            let mut point = DataPoint::new("wia_fusion_plasma").at(timestamp);

            for (key, value) in tags.iter() {
                point = point.tag(key, value);
            }

            if let Some(temp) = plasma.get("temperature").and_then(|t| t.get("value")).and_then(|v| v.as_f64()) {
                point = point.field_float("temperature", temp);
            }

            if let Some(density) = plasma.get("density").and_then(|d| d.get("value")).and_then(|v| v.as_f64()) {
                point = point.field_float("density", density);
            }

            if let Some(confinement) = plasma.get("confinement_time").and_then(|c| c.get("value")).and_then(|v| v.as_f64()) {
                point = point.field_float("confinement_time", confinement);
            }

            if !point.fields.is_empty() {
                points.push(point);
            }
        }

        // Magnetics data
        if let Some(magnetics) = data.get("magnetics") {
            let mut point = DataPoint::new("wia_fusion_magnetics").at(timestamp);

            for (key, value) in tags.iter() {
                point = point.tag(key, value);
            }

            if let Some(tf) = magnetics.get("toroidal_field").and_then(|t| t.get("value")).and_then(|v| v.as_f64()) {
                point = point.field_float("toroidal_field", tf);
            }

            if let Some(current) = magnetics.get("plasma_current").and_then(|c| c.get("value")).and_then(|v| v.as_f64()) {
                point = point.field_float("plasma_current", current);
            }

            if !point.fields.is_empty() {
                points.push(point);
            }
        }

        // Energy data
        if let Some(energy) = data.get("energy") {
            let mut point = DataPoint::new("wia_fusion_energy").at(timestamp);

            for (key, value) in tags.iter() {
                point = point.tag(key, value);
            }

            if let Some(q) = energy.get("q_factor").and_then(|v| v.as_f64()) {
                point = point.field_float("q_factor", q);
            }

            if let Some(heating) = energy.get("heating_power").and_then(|h| h.get("value")).and_then(|v| v.as_f64()) {
                point = point.field_float("heating_power", heating);
            }

            if !point.fields.is_empty() {
                points.push(point);
            }
        }

        points
    }

    /// Get endpoint
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    /// Get database
    pub fn database(&self) -> &str {
        &self.database
    }

    /// Get stored points count (for testing)
    pub fn points_count(&self) -> usize {
        self.mock_points.len()
    }
}

impl IntegrationAdapter for InfluxDBAdapter {
    fn integration_type(&self) -> IntegrationType {
        IntegrationType::InfluxDb
    }

    fn name(&self) -> &str {
        "InfluxDB Time-Series"
    }

    fn state(&self) -> IntegrationState {
        self.state
    }

    fn initialize(&mut self, options: IntegrationOptions) -> Result<(), IntegrationError> {
        if let Some(endpoint) = options.endpoint {
            self.endpoint = endpoint;
        }

        if let Some(ref creds) = options.credentials {
            if let Some(ref token) = creds.token {
                self.token = Some(token.clone());
            }
        }

        if let Some(org) = options.extra.get("org").and_then(|v| v.as_str()) {
            self.org = Some(org.to_string());
        }

        if let Some(db) = options.extra.get("database").and_then(|v| v.as_str()) {
            self.database = db.to_string();
        }

        Ok(())
    }

    fn connect(&mut self) -> Result<(), IntegrationError> {
        // In real implementation, would establish HTTP connection
        self.state = IntegrationState::Connected;
        Ok(())
    }

    fn disconnect(&mut self) -> Result<(), IntegrationError> {
        self.state = IntegrationState::Disconnected;
        Ok(())
    }

    fn export(&self, data: &PhysicsData) -> Result<ExportResult, IntegrationError> {
        if self.state != IntegrationState::Connected {
            return Err(IntegrationError::NotConnected);
        }

        let mut tags = HashMap::new();
        if let Some(ref exp) = data.metadata.experiment {
            tags.insert("experiment".to_string(), exp.clone());
        }

        let points = Self::fusion_to_points(&data.payload, tags);
        let records = points.len() as u64;

        Ok(ExportResult {
            success: true,
            records_exported: records,
            destination: format!("{}:{}", self.endpoint, self.database),
            timestamp: chrono::Utc::now().timestamp_millis(),
            details: Some(format!("Wrote {} points to InfluxDB", records)),
        })
    }

    fn import(&self, _source: &str) -> Result<PhysicsData, IntegrationError> {
        Err(IntegrationError::NotSupported("InfluxDB import not implemented".to_string()))
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
    fn test_data_point_creation() {
        let point = DataPoint::new("test_measurement")
            .tag("experiment", "ITER")
            .tag("location", "core")
            .field_float("temperature", 150e6)
            .field_int("shot_number", 12345);

        assert_eq!(point.measurement, "test_measurement");
        assert_eq!(point.tags.get("experiment"), Some(&"ITER".to_string()));
        assert!(matches!(point.fields.get("temperature"), Some(FieldValue::Float(t)) if (*t - 150e6).abs() < 1.0));
    }

    #[test]
    fn test_line_protocol() {
        let point = DataPoint::new("plasma")
            .tag("experiment", "ITER")
            .field_float("temperature", 150e6)
            .at(1234567890000000000);

        let line = point.to_line_protocol();
        assert!(line.contains("plasma"));
        assert!(line.contains("experiment=ITER"));
        assert!(line.contains("temperature="));
        assert!(line.contains("1234567890000000000"));
    }

    #[test]
    fn test_query_builder() {
        let query = TimeSeriesQuery::new("wia_fusion_plasma")
            .select(vec!["temperature", "density"])
            .filter("experiment", "ITER")
            .aggregate(Aggregation::Mean)
            .group_by_time("1m")
            .limit(100);

        let sql = query.to_influxql();
        assert!(sql.contains("SELECT MEAN(temperature), MEAN(density)"));
        assert!(sql.contains("FROM wia_fusion_plasma"));
        assert!(sql.contains("experiment = 'ITER'"));
        assert!(sql.contains("GROUP BY time(1m)"));
        assert!(sql.contains("LIMIT 100"));
    }

    #[test]
    fn test_influxdb_adapter() {
        let mut adapter = InfluxDBAdapter::new("http://localhost:8086", "wia_physics");
        adapter.connect().unwrap();

        let point = DataPoint::new("test")
            .field_float("value", 42.0);
        adapter.write_point(point).unwrap();

        assert_eq!(adapter.points_count(), 1);
    }

    #[test]
    fn test_fusion_to_points() {
        let data = serde_json::json!({
            "plasma": {
                "temperature": {"value": 150e6},
                "density": {"value": 1e20}
            },
            "energy": {
                "q_factor": 10.0
            }
        });

        let mut tags = HashMap::new();
        tags.insert("experiment".to_string(), "ITER".to_string());

        let points = InfluxDBAdapter::fusion_to_points(&data, tags);
        assert_eq!(points.len(), 2); // plasma + energy
    }
}
