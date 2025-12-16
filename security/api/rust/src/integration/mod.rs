//! WIA Security Ecosystem Integration
//!
//! Provides integrations with external security tools, SIEM systems,
//! cloud security platforms, and CI/CD pipelines.
//!
//! ## Features
//!
//! ### Importers
//! - NVD/CVE vulnerability database
//! - Nessus XML scan reports
//! - OpenVAS XML scan reports
//! - STIX/TAXII threat intelligence
//!
//! ### Exporters
//! - Splunk HEC (HTTP Event Collector)
//! - Elasticsearch/OpenSearch
//! - PDF/HTML report generation
//! - Grafana dashboard JSON
//!
//! ### Cloud Integrations
//! - AWS Security Hub
//! - Azure Defender
//! - GCP Security Command Center
//!
//! ### CI/CD
//! - GitHub Actions integration
//! - GitLab CI integration

pub mod importers;
pub mod exporters;
pub mod cloud;
pub mod cicd;

pub use importers::*;
pub use exporters::*;
pub use cloud::*;
pub use cicd::*;

/// Integration version
pub const INTEGRATION_VERSION: &str = "1.0.0";
