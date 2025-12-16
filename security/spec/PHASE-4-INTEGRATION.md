# WIA Security Phase 4: Ecosystem Integration Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2024-12-16

## Overview

Phase 4 defines the ecosystem integration layer for WIA Security, enabling seamless data exchange with external security tools, SIEM platforms, cloud security services, and CI/CD pipelines.

## Table of Contents

1. [Data Importers](#1-data-importers)
2. [Data Exporters](#2-data-exporters)
3. [Cloud Integrations](#3-cloud-integrations)
4. [CI/CD Integrations](#4-cicd-integrations)
5. [Data Format Specifications](#5-data-format-specifications)

---

## 1. Data Importers

### 1.1 NVD/CVE Importer

Import vulnerability data from NIST National Vulnerability Database.

#### Configuration

```rust
pub struct NvdConfig {
    pub api_key: Option<String>,          // NVD API key (optional, recommended)
    pub base_url: String,                  // API endpoint
    pub results_per_page: u32,             // Pagination size (max: 2000)
    pub rate_limit_delay_ms: u64,          // Rate limiting delay
    pub cache_ttl_hours: u32,              // Cache duration
}
```

#### API Endpoints

| Endpoint | Description |
|----------|-------------|
| `/cves/2.0` | Query CVE records |
| `/cvehistory/2.0` | CVE change history |
| `/cpematch/2.0` | CPE match strings |

#### Output Format

```json
{
  "vulnerabilities": [
    {
      "id": "CVE-2024-1234",
      "description": "Vulnerability description",
      "cvss": {
        "version": "3.1",
        "score": 7.5,
        "vector": "CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:N/A:N",
        "severity": "high"
      },
      "references": [],
      "affected_products": []
    }
  ]
}
```

### 1.2 Nessus Importer

Parse Tenable Nessus XML scan reports (.nessus files).

#### Supported Elements

- `NessusClientData_v2` root element
- `ReportHost` with host properties
- `ReportItem` findings with plugin details
- CVSS v2/v3 scores and vectors

#### Severity Mapping

| Nessus Severity | WIA Severity |
|-----------------|--------------|
| 4 | Critical |
| 3 | High |
| 2 | Medium |
| 1 | Low |
| 0 | Info |

### 1.3 OpenVAS Importer

Parse OpenVAS/GVM XML scan reports.

#### Supported Elements

- `report` root element
- `results/result` findings
- `nvt` (Network Vulnerability Test) details
- QoD (Quality of Detection) values

#### Threat Level Mapping

| OpenVAS Threat | WIA Severity |
|----------------|--------------|
| High | Critical |
| Medium | High |
| Low | Medium |
| Log/Debug | Info |

### 1.4 STIX/TAXII Importer

Import threat intelligence from STIX 2.1 feeds via TAXII 2.1 protocol.

#### TAXII Client Configuration

```rust
pub struct TaxiiClientConfig {
    pub server_url: String,
    pub api_root: String,
    pub collection_id: Option<String>,
    pub auth: TaxiiAuth,
    pub timeout_secs: u64,
}

pub enum TaxiiAuth {
    None,
    Basic { username: String, password: String },
    ApiKey { header: String, key: String },
    Certificate { cert_path: String, key_path: String },
}
```

#### Supported STIX Objects

- `indicator` - IOCs
- `threat-actor` - Threat actor profiles
- `attack-pattern` - TTPs (MITRE ATT&CK)
- `malware` - Malware information
- `vulnerability` - CVE data
- `relationship` - Object relationships

---

## 2. Data Exporters

### 2.1 Splunk HEC Exporter

Export security events to Splunk via HTTP Event Collector.

#### Configuration

```rust
pub struct SplunkConfig {
    pub endpoint: String,           // HEC endpoint URL
    pub token: String,              // HEC token
    pub index: Option<String>,      // Target index
    pub source: Option<String>,     // Event source
    pub sourcetype: Option<String>, // Event sourcetype
    pub verify_ssl: bool,
    pub batch_size: usize,
}
```

#### Event Format

```json
{
  "time": 1702684800.0,
  "index": "wia_security",
  "source": "wia-security-api",
  "sourcetype": "wia:security:event",
  "event": {
    "category": "security",
    "event_type": "vulnerability",
    "severity": "high",
    "message": "Finding description"
  }
}
```

### 2.2 Elasticsearch Exporter

Export to Elasticsearch using Elastic Common Schema (ECS) format.

#### Configuration

```rust
pub struct ElasticsearchConfig {
    pub hosts: Vec<String>,
    pub index_prefix: String,
    pub auth: Option<ElasticsearchAuth>,
    pub use_ssl: bool,
    pub bulk_size: usize,
}
```

#### ECS Mapping

| WIA Field | ECS Field |
|-----------|-----------|
| finding.id | event.id |
| finding.severity | event.severity |
| finding.cvss_score | vulnerability.score.base |
| host | host.hostname |
| port | destination.port |

### 2.3 PDF Report Generator

Generate professional PDF security assessment reports.

#### Report Sections

1. **Executive Summary** - Risk overview with key metrics
2. **Severity Distribution** - Visual charts
3. **Hosts Summary** - Finding counts per host
4. **Detailed Findings** - Full vulnerability details
5. **Recommendations** - Remediation guidance

#### Configuration

```rust
pub struct PdfConfig {
    pub title: String,
    pub organization: String,
    pub author: Option<String>,
    pub include_executive_summary: bool,
    pub include_detailed_findings: bool,
    pub include_charts: bool,
    pub page_size: PageSize,
    pub color_scheme: ColorScheme,
}
```

### 2.4 Grafana Dashboard Generator

Generate Grafana dashboard JSON for security metrics visualization.

#### Dashboard Panels

| Panel | Type | Description |
|-------|------|-------------|
| Total Findings | Stat | Count of all findings |
| Critical Count | Stat | Critical vulnerabilities |
| Severity Distribution | Pie | Donut chart by severity |
| Findings Over Time | Time Series | Trend analysis |
| Top Vulnerabilities | Table | Most common findings |
| Affected Hosts | Table | Hosts by finding count |

---

## 3. Cloud Integrations

### 3.1 AWS Security Hub

Export findings using AWS Security Finding Format (ASFF).

#### Configuration

```rust
pub struct AwsSecurityHubConfig {
    pub region: String,
    pub account_id: String,
    pub product_arn: Option<String>,
    pub batch_import: bool,
    pub batch_size: usize,
}
```

#### ASFF Mapping

| WIA Field | ASFF Field |
|-----------|------------|
| finding.id | Id |
| finding.title | Title |
| finding.severity | Severity.Label |
| cvss_score | Severity.Normalized |
| description | Description |
| solution | Remediation.Recommendation.Text |

#### Severity Normalization

| WIA Severity | ASFF Label | Normalized (0-100) |
|--------------|------------|-------------------|
| Critical | CRITICAL | 90 |
| High | HIGH | 70 |
| Medium | MEDIUM | 50 |
| Low | LOW | 30 |
| Info | INFORMATIONAL | 10 |

### 3.2 Azure Defender (Microsoft Defender for Cloud)

Integration with Azure security services.

#### Configuration

```rust
pub struct AzureDefenderConfig {
    pub subscription_id: String,
    pub resource_group: String,
    pub workspace_id: String,      // Log Analytics
    pub tenant_id: String,
    pub client_id: String,
    pub environment: AzureEnvironment,
}
```

#### Export Options

1. **Security Alerts** - Native Azure Security Alert format
2. **Log Analytics** - Custom log entries for Sentinel queries

### 3.3 GCP Security Command Center

Export findings to Google Cloud Security Command Center.

#### Configuration

```rust
pub struct GcpSccConfig {
    pub project_id: String,
    pub organization_id: String,
    pub source_id: Option<String>,
    pub use_adc: bool,             // Application Default Credentials
}
```

#### SCC Finding Categories

- `VULNERABILITY` - Security vulnerabilities
- `MISCONFIGURATION` - Configuration issues
- `THREAT` - Active threats
- `OBSERVATION` - Security observations

---

## 4. CI/CD Integrations

### 4.1 GitHub Actions

#### Workflow Features

- Security scanning on push/PR
- SARIF upload to GitHub Security tab
- Caching for faster scans
- Configurable failure thresholds
- SBOM generation

#### Sample Workflow

```yaml
name: WIA Security Scan
on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  security-scan:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Run WIA Security Scan
        run: |
          wia-security scan \
            --fail-on-critical=true \
            --output-format=sarif \
            --output=results.sarif
      - name: Upload SARIF
        uses: github/codeql-action/upload-sarif@v3
        with:
          sarif_file: results.sarif
```

### 4.2 GitLab CI

#### Pipeline Features

- Integration with GitLab Security Dashboard
- SAST report format compatibility
- Compliance framework templates
- Auto DevOps override support

#### Sample Pipeline

```yaml
include:
  - template: Security/SAST.gitlab-ci.yml

wia-security-scan:
  stage: security
  script:
    - wia-security scan --output-format=gitlab-sast --output=gl-sast-report.json
  artifacts:
    reports:
      sast: gl-sast-report.json
```

### 4.3 Security Gate Configuration

```rust
pub struct SecurityScanConfig {
    pub fail_on_critical: bool,
    pub fail_on_high: bool,
    pub max_critical: u32,
    pub max_high: u32,
    pub timeout_minutes: u32,
    pub enable_sbom: bool,
}
```

---

## 5. Data Format Specifications

### 5.1 WIA Scan Result

```json
{
  "scan_id": "uuid",
  "scan_name": "Security Scan",
  "scan_time": "2024-12-16T00:00:00Z",
  "targets": [
    {
      "host": "192.168.1.100",
      "ip": "192.168.1.100",
      "os": "Linux",
      "findings": []
    }
  ],
  "summary": {
    "total_hosts": 1,
    "total_findings": 10,
    "critical": 1,
    "high": 3,
    "medium": 4,
    "low": 2,
    "info": 0
  }
}
```

### 5.2 WIA Finding

```json
{
  "id": "VULN-001",
  "title": "Vulnerability Title",
  "description": "Detailed description",
  "severity": "high",
  "cvss_score": 7.5,
  "cvss_vector": "CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:N/A:N",
  "port": 443,
  "protocol": "tcp",
  "service": "https",
  "cve": ["CVE-2024-1234"],
  "cwe": ["CWE-79"],
  "solution": "Remediation steps",
  "references": ["https://example.com"],
  "exploit_available": false,
  "patch_available": true
}
```

### 5.3 SARIF Output

WIA Security generates SARIF 2.1.0 format for code scanning integrations:

```json
{
  "$schema": "https://raw.githubusercontent.com/oasis-tcs/sarif-spec/master/Schemata/sarif-schema-2.1.0.json",
  "version": "2.1.0",
  "runs": [
    {
      "tool": {
        "driver": {
          "name": "WIA Security Scanner",
          "version": "1.0.0",
          "rules": []
        }
      },
      "results": []
    }
  ]
}
```

---

## Implementation Status

| Component | Status | Module |
|-----------|--------|--------|
| NVD Importer | ✅ Complete | `integration::importers::nvd` |
| Nessus Importer | ✅ Complete | `integration::importers::nessus` |
| OpenVAS Importer | ✅ Complete | `integration::importers::openvas` |
| STIX/TAXII Importer | ✅ Complete | `integration::importers::stix` |
| Splunk Exporter | ✅ Complete | `integration::exporters::splunk` |
| Elasticsearch Exporter | ✅ Complete | `integration::exporters::elasticsearch` |
| PDF Generator | ✅ Complete | `integration::exporters::pdf` |
| Grafana Generator | ✅ Complete | `integration::exporters::grafana` |
| AWS Security Hub | ✅ Complete | `integration::cloud::aws` |
| Azure Defender | ✅ Complete | `integration::cloud::azure` |
| GCP SCC | ✅ Complete | `integration::cloud::gcp` |
| GitHub Actions | ✅ Complete | `integration::cicd::github` |
| GitLab CI | ✅ Complete | `integration::cicd::gitlab` |

---

## References

- [NVD API Documentation](https://nvd.nist.gov/developers)
- [STIX 2.1 Specification](https://oasis-open.github.io/cti-documentation/)
- [TAXII 2.1 Specification](https://docs.oasis-open.org/cti/taxii/v2.1/taxii-v2.1.html)
- [AWS ASFF Format](https://docs.aws.amazon.com/securityhub/latest/userguide/securityhub-findings-format.html)
- [Elastic Common Schema](https://www.elastic.co/guide/en/ecs/current/index.html)
- [SARIF Specification](https://docs.oasis-open.org/sarif/sarif/v2.1.0/sarif-v2.1.0.html)
- [GitLab SAST Report Format](https://docs.gitlab.com/ee/development/integrations/secure.html)
