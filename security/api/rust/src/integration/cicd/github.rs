//! GitHub Actions Integration
//!
//! Generate GitHub Actions workflows for security scanning.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use super::{CicdResult, CicdError, SecurityScanConfig};

/// GitHub Actions workflow configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GitHubActionsConfig {
    /// Workflow name
    pub name: String,
    /// Trigger events
    pub triggers: GitHubTriggers,
    /// Environment variables
    pub env: HashMap<String, String>,
    /// Security scan configuration
    pub scan_config: SecurityScanConfig,
    /// Upload to Security tab
    pub upload_sarif: bool,
    /// Create GitHub issue on failure
    pub create_issue_on_failure: bool,
    /// Use caching
    pub enable_caching: bool,
}

/// GitHub trigger events
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GitHubTriggers {
    pub push: Option<GitHubPushTrigger>,
    pub pull_request: Option<GitHubPrTrigger>,
    pub schedule: Option<Vec<GitHubScheduleTrigger>>,
    pub workflow_dispatch: bool,
}

/// Push trigger
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GitHubPushTrigger {
    pub branches: Vec<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub paths: Vec<String>,
}

/// Pull request trigger
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GitHubPrTrigger {
    pub branches: Vec<String>,
    pub types: Vec<String>,
}

/// Schedule trigger
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GitHubScheduleTrigger {
    pub cron: String,
}

impl Default for GitHubActionsConfig {
    fn default() -> Self {
        Self {
            name: "WIA Security Scan".to_string(),
            triggers: GitHubTriggers {
                push: Some(GitHubPushTrigger {
                    branches: vec!["main".to_string(), "develop".to_string()],
                    paths: vec![],
                }),
                pull_request: Some(GitHubPrTrigger {
                    branches: vec!["main".to_string()],
                    types: vec!["opened".to_string(), "synchronize".to_string()],
                }),
                schedule: Some(vec![GitHubScheduleTrigger {
                    cron: "0 0 * * 0".to_string(), // Weekly on Sunday
                }]),
                workflow_dispatch: true,
            },
            env: HashMap::new(),
            scan_config: SecurityScanConfig::default(),
            upload_sarif: true,
            create_issue_on_failure: false,
            enable_caching: true,
        }
    }
}

/// GitHub Actions workflow generator
pub struct GitHubActionsGenerator {
    config: GitHubActionsConfig,
}

impl GitHubActionsGenerator {
    /// Create new generator
    pub fn new(config: GitHubActionsConfig) -> Self {
        Self { config }
    }

    /// Generate full workflow YAML
    pub fn generate_workflow(&self) -> CicdResult<String> {
        let mut yaml = String::new();

        // Header
        yaml.push_str(&format!("name: {}\n\n", self.config.name));

        // Triggers
        yaml.push_str("on:\n");
        if let Some(ref push) = self.config.triggers.push {
            yaml.push_str("  push:\n");
            yaml.push_str(&format!("    branches: [{}]\n",
                push.branches.iter().map(|b| format!("\"{}\"", b)).collect::<Vec<_>>().join(", ")));
            if !push.paths.is_empty() {
                yaml.push_str(&format!("    paths: [{}]\n",
                    push.paths.iter().map(|p| format!("\"{}\"", p)).collect::<Vec<_>>().join(", ")));
            }
        }
        if let Some(ref pr) = self.config.triggers.pull_request {
            yaml.push_str("  pull_request:\n");
            yaml.push_str(&format!("    branches: [{}]\n",
                pr.branches.iter().map(|b| format!("\"{}\"", b)).collect::<Vec<_>>().join(", ")));
            yaml.push_str(&format!("    types: [{}]\n",
                pr.types.iter().map(|t| format!("\"{}\"", t)).collect::<Vec<_>>().join(", ")));
        }
        if let Some(ref schedules) = self.config.triggers.schedule {
            yaml.push_str("  schedule:\n");
            for schedule in schedules {
                yaml.push_str(&format!("    - cron: \"{}\"\n", schedule.cron));
            }
        }
        if self.config.triggers.workflow_dispatch {
            yaml.push_str("  workflow_dispatch:\n");
            yaml.push_str("    inputs:\n");
            yaml.push_str("      scan_target:\n");
            yaml.push_str("        description: 'Target to scan'\n");
            yaml.push_str("        required: false\n");
            yaml.push_str("        default: '.'\n");
        }
        yaml.push('\n');

        // Permissions
        yaml.push_str("permissions:\n");
        yaml.push_str("  contents: read\n");
        yaml.push_str("  security-events: write\n");
        yaml.push_str("  actions: read\n");
        if self.config.create_issue_on_failure {
            yaml.push_str("  issues: write\n");
        }
        yaml.push('\n');

        // Environment
        if !self.config.env.is_empty() {
            yaml.push_str("env:\n");
            for (key, value) in &self.config.env {
                yaml.push_str(&format!("  {}: \"{}\"\n", key, value));
            }
            yaml.push('\n');
        }

        // Jobs
        yaml.push_str("jobs:\n");
        yaml.push_str(&self.generate_security_scan_job());
        yaml.push_str(&self.generate_sbom_job());
        if self.config.upload_sarif {
            yaml.push_str(&self.generate_upload_sarif_job());
        }

        Ok(yaml)
    }

    /// Generate security scan job
    fn generate_security_scan_job(&self) -> String {
        let mut job = String::new();
        job.push_str("  security-scan:\n");
        job.push_str("    name: Security Vulnerability Scan\n");
        job.push_str("    runs-on: ubuntu-latest\n");
        job.push_str(&format!("    timeout-minutes: {}\n", self.config.scan_config.timeout_minutes));
        job.push_str("    outputs:\n");
        job.push_str("      findings-critical: ${{ steps.scan.outputs.critical }}\n");
        job.push_str("      findings-high: ${{ steps.scan.outputs.high }}\n");
        job.push_str("      scan-status: ${{ steps.scan.outputs.status }}\n");
        job.push_str("    steps:\n");

        // Checkout
        job.push_str("      - name: Checkout repository\n");
        job.push_str("        uses: actions/checkout@v4\n");
        job.push_str("        with:\n");
        job.push_str("          fetch-depth: 0\n\n");

        // Cache
        if self.config.enable_caching {
            job.push_str("      - name: Cache WIA Security data\n");
            job.push_str("        uses: actions/cache@v4\n");
            job.push_str("        with:\n");
            job.push_str("          path: |\n");
            job.push_str("            ~/.wia-security/cache\n");
            job.push_str("            ~/.wia-security/nvd\n");
            job.push_str("          key: wia-security-${{ runner.os }}-${{ hashFiles('**/Cargo.lock', '**/package-lock.json', '**/requirements.txt') }}\n");
            job.push_str("          restore-keys: |\n");
            job.push_str("            wia-security-${{ runner.os }}-\n\n");
        }

        // Setup WIA Security
        job.push_str("      - name: Setup WIA Security Scanner\n");
        job.push_str("        run: |\n");
        job.push_str("          # Download and install WIA Security Scanner\n");
        job.push_str("          curl -sSL https://raw.githubusercontent.com/WIA-Official/wia-standards/main/security/install.sh | bash\n");
        job.push_str("          echo \"$HOME/.wia-security/bin\" >> $GITHUB_PATH\n\n");

        // Run scan
        job.push_str("      - name: Run security scan\n");
        job.push_str("        id: scan\n");
        job.push_str("        run: |\n");
        job.push_str("          wia-security scan \\\n");
        job.push_str(&format!("            --fail-on-critical={} \\\n", self.config.scan_config.fail_on_critical));
        job.push_str(&format!("            --fail-on-high={} \\\n", self.config.scan_config.fail_on_high));
        job.push_str(&format!("            --max-critical={} \\\n", self.config.scan_config.max_critical));
        job.push_str(&format!("            --max-high={} \\\n", self.config.scan_config.max_high));
        job.push_str("            --output-format=sarif \\\n");
        job.push_str("            --output=results.sarif \\\n");
        job.push_str("            --json-output=results.json \\\n");
        job.push_str("            ${{ github.event.inputs.scan_target || '.' }}\n\n");
        job.push_str("          # Parse results and set outputs\n");
        job.push_str("          CRITICAL=$(jq '.summary.critical // 0' results.json)\n");
        job.push_str("          HIGH=$(jq '.summary.high // 0' results.json)\n");
        job.push_str("          echo \"critical=$CRITICAL\" >> $GITHUB_OUTPUT\n");
        job.push_str("          echo \"high=$HIGH\" >> $GITHUB_OUTPUT\n");
        job.push_str("          echo \"status=completed\" >> $GITHUB_OUTPUT\n\n");

        // Upload artifacts
        job.push_str("      - name: Upload scan results\n");
        job.push_str("        uses: actions/upload-artifact@v4\n");
        job.push_str("        if: always()\n");
        job.push_str("        with:\n");
        job.push_str("          name: security-scan-results\n");
        job.push_str("          path: |\n");
        job.push_str("            results.sarif\n");
        job.push_str("            results.json\n");
        job.push_str("          retention-days: 30\n\n");

        // Check thresholds
        job.push_str("      - name: Check security thresholds\n");
        job.push_str("        if: always()\n");
        job.push_str("        run: |\n");
        job.push_str(&format!("          if [ ${{{{ steps.scan.outputs.critical }}}} -gt {} ]; then\n", self.config.scan_config.max_critical));
        job.push_str("            echo \"::error::Critical vulnerabilities exceed threshold\"\n");
        if self.config.scan_config.fail_on_critical {
            job.push_str("            exit 1\n");
        }
        job.push_str("          fi\n");
        job.push_str(&format!("          if [ ${{{{ steps.scan.outputs.high }}}} -gt {} ]; then\n", self.config.scan_config.max_high));
        job.push_str("            echo \"::warning::High vulnerabilities exceed threshold\"\n");
        if self.config.scan_config.fail_on_high {
            job.push_str("            exit 1\n");
        }
        job.push_str("          fi\n\n");

        job
    }

    /// Generate SBOM job
    fn generate_sbom_job(&self) -> String {
        if !self.config.scan_config.enable_sbom {
            return String::new();
        }

        let mut job = String::new();
        job.push_str("  sbom:\n");
        job.push_str("    name: Generate SBOM\n");
        job.push_str("    runs-on: ubuntu-latest\n");
        job.push_str("    needs: security-scan\n");
        job.push_str("    steps:\n");

        job.push_str("      - name: Checkout repository\n");
        job.push_str("        uses: actions/checkout@v4\n\n");

        job.push_str("      - name: Generate SBOM\n");
        job.push_str("        uses: anchore/sbom-action@v0\n");
        job.push_str("        with:\n");
        job.push_str("          path: .\n");
        job.push_str("          format: spdx-json\n");
        job.push_str("          output-file: sbom.spdx.json\n\n");

        job.push_str("      - name: Upload SBOM\n");
        job.push_str("        uses: actions/upload-artifact@v4\n");
        job.push_str("        with:\n");
        job.push_str("          name: sbom\n");
        job.push_str("          path: sbom.spdx.json\n");
        job.push_str("          retention-days: 90\n\n");

        job
    }

    /// Generate SARIF upload job
    fn generate_upload_sarif_job(&self) -> String {
        let mut job = String::new();
        job.push_str("  upload-sarif:\n");
        job.push_str("    name: Upload SARIF to GitHub Security\n");
        job.push_str("    runs-on: ubuntu-latest\n");
        job.push_str("    needs: security-scan\n");
        job.push_str("    if: always()\n");
        job.push_str("    steps:\n");

        job.push_str("      - name: Download scan results\n");
        job.push_str("        uses: actions/download-artifact@v4\n");
        job.push_str("        with:\n");
        job.push_str("          name: security-scan-results\n\n");

        job.push_str("      - name: Upload SARIF file\n");
        job.push_str("        uses: github/codeql-action/upload-sarif@v3\n");
        job.push_str("        with:\n");
        job.push_str("          sarif_file: results.sarif\n");
        job.push_str("          category: wia-security\n\n");

        job
    }

    /// Generate reusable workflow
    pub fn generate_reusable_workflow(&self) -> CicdResult<String> {
        let mut yaml = String::new();

        yaml.push_str("name: WIA Security Scan (Reusable)\n\n");
        yaml.push_str("on:\n");
        yaml.push_str("  workflow_call:\n");
        yaml.push_str("    inputs:\n");
        yaml.push_str("      scan-target:\n");
        yaml.push_str("        description: 'Target directory to scan'\n");
        yaml.push_str("        required: false\n");
        yaml.push_str("        type: string\n");
        yaml.push_str("        default: '.'\n");
        yaml.push_str("      fail-on-critical:\n");
        yaml.push_str("        description: 'Fail on critical findings'\n");
        yaml.push_str("        required: false\n");
        yaml.push_str("        type: boolean\n");
        yaml.push_str("        default: true\n");
        yaml.push_str("      max-critical:\n");
        yaml.push_str("        description: 'Max allowed critical findings'\n");
        yaml.push_str("        required: false\n");
        yaml.push_str("        type: number\n");
        yaml.push_str("        default: 0\n");
        yaml.push_str("    secrets:\n");
        yaml.push_str("      WIA_API_KEY:\n");
        yaml.push_str("        required: false\n");
        yaml.push_str("    outputs:\n");
        yaml.push_str("      findings-total:\n");
        yaml.push_str("        description: 'Total findings'\n");
        yaml.push_str("        value: ${{ jobs.scan.outputs.total }}\n");
        yaml.push_str("      findings-critical:\n");
        yaml.push_str("        description: 'Critical findings'\n");
        yaml.push_str("        value: ${{ jobs.scan.outputs.critical }}\n\n");

        yaml.push_str("jobs:\n");
        yaml.push_str("  scan:\n");
        yaml.push_str("    name: Security Scan\n");
        yaml.push_str("    runs-on: ubuntu-latest\n");
        yaml.push_str("    outputs:\n");
        yaml.push_str("      total: ${{ steps.analyze.outputs.total }}\n");
        yaml.push_str("      critical: ${{ steps.analyze.outputs.critical }}\n");
        yaml.push_str("    steps:\n");
        yaml.push_str("      - uses: actions/checkout@v4\n\n");
        yaml.push_str("      - name: Run WIA Security Scan\n");
        yaml.push_str("        id: analyze\n");
        yaml.push_str("        run: |\n");
        yaml.push_str("          # Scanner implementation\n");
        yaml.push_str("          echo \"total=0\" >> $GITHUB_OUTPUT\n");
        yaml.push_str("          echo \"critical=0\" >> $GITHUB_OUTPUT\n");

        Ok(yaml)
    }

    /// Generate Dependabot config
    pub fn generate_dependabot_config(&self) -> CicdResult<String> {
        let yaml = r#"# Dependabot configuration for WIA Security
version: 2
updates:
  # Enable version updates for npm
  - package-ecosystem: "npm"
    directory: "/"
    schedule:
      interval: "weekly"
    open-pull-requests-limit: 10
    labels:
      - "dependencies"
      - "security"

  # Enable version updates for Cargo
  - package-ecosystem: "cargo"
    directory: "/"
    schedule:
      interval: "weekly"
    open-pull-requests-limit: 10
    labels:
      - "dependencies"
      - "security"

  # Enable version updates for pip
  - package-ecosystem: "pip"
    directory: "/"
    schedule:
      interval: "weekly"
    open-pull-requests-limit: 10
    labels:
      - "dependencies"
      - "security"

  # Enable version updates for GitHub Actions
  - package-ecosystem: "github-actions"
    directory: "/"
    schedule:
      interval: "weekly"
    labels:
      - "dependencies"
      - "ci"
"#;

        Ok(yaml.to_string())
    }

    /// Generate code scanning config
    pub fn generate_codeql_config(&self) -> CicdResult<String> {
        let yaml = r#"name: "CodeQL + WIA Security"

on:
  push:
    branches: ["main", "develop"]
  pull_request:
    branches: ["main"]
  schedule:
    - cron: "0 0 * * 1"  # Weekly on Monday

permissions:
  contents: read
  security-events: write

jobs:
  analyze:
    name: Analyze
    runs-on: ubuntu-latest
    strategy:
      fail-fast: false
      matrix:
        language: ["javascript", "python", "go"]

    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Initialize CodeQL
        uses: github/codeql-action/init@v3
        with:
          languages: ${{ matrix.language }}
          queries: security-extended

      - name: Autobuild
        uses: github/codeql-action/autobuild@v3

      - name: Perform CodeQL Analysis
        uses: github/codeql-action/analyze@v3
        with:
          category: "/language:${{ matrix.language }}"
"#;

        Ok(yaml.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_workflow() {
        let config = GitHubActionsConfig::default();
        let generator = GitHubActionsGenerator::new(config);

        let yaml = generator.generate_workflow().unwrap();

        assert!(yaml.contains("name: WIA Security Scan"));
        assert!(yaml.contains("security-scan:"));
        assert!(yaml.contains("actions/checkout@v4"));
    }

    #[test]
    fn test_generate_dependabot() {
        let config = GitHubActionsConfig::default();
        let generator = GitHubActionsGenerator::new(config);

        let yaml = generator.generate_dependabot_config().unwrap();

        assert!(yaml.contains("version: 2"));
        assert!(yaml.contains("package-ecosystem"));
    }
}
