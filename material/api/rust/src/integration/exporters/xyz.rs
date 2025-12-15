//! XYZ format exporter
//!
//! Exports material data to XYZ format for molecular visualization.

use chrono::Utc;

use super::traits::{ExportFormat, ExportOptions, Exporter, ImportOptions};
use crate::integration::error::{IntegrationError, IntegrationResult};
use crate::types::{Identity, MaterialData, MaterialType, Properties, Timestamp};

/// XYZ format exporter
pub struct XyzExporter;

impl XyzExporter {
    /// Create a new XYZ exporter
    pub fn new() -> Self {
        Self
    }

    /// Generate XYZ content from MaterialData
    fn generate_xyz(
        &self,
        material: &MaterialData,
        options: &ExportOptions,
    ) -> IntegrationResult<String> {
        let precision = options.precision;
        let mut xyz = String::new();

        // Extract elements from formula for placeholder positions
        let elements = extract_elements_from_formula(&material.identity.formula);

        // Line 1: Number of atoms (placeholder count based on formula)
        xyz.push_str(&format!("{}", elements.len()));
        xyz.push('\n');

        // Line 2: Comment line
        if options.include_metadata {
            xyz.push_str(&format!(
                "{} ({}) - {}",
                material.identity.name, material.identity.formula, material.material_id
            ));
        } else {
            xyz.push_str(&material.identity.formula);
        }
        xyz.push('\n');

        // Lines 3+: Placeholder atom coordinates (actual positions not available)
        // Generate simple placeholder positions
        for (i, element) in elements.iter().enumerate() {
            let x = i as f64 * 2.0;
            xyz.push_str(&format!(
                "{:<2} {:>15.*} {:>15.*} {:>15.*}",
                element, precision, x, precision, 0.0, precision, 0.0
            ));
            xyz.push('\n');
        }

        if elements.is_empty() {
            xyz.push_str("# No atomic positions available");
            xyz.push('\n');
        }

        Ok(xyz)
    }

    /// Parse XYZ content into MaterialData
    fn parse_xyz(&self, content: &str, options: &ImportOptions) -> IntegrationResult<MaterialData> {
        let lines: Vec<&str> = content.lines().collect();

        if lines.len() < 2 {
            return Err(IntegrationError::ParseError(
                "XYZ: insufficient lines".to_string(),
            ));
        }

        // Line 1: Number of atoms
        let _num_atoms: usize = lines[0]
            .trim()
            .parse()
            .map_err(|_| IntegrationError::ParseError("XYZ: invalid atom count".to_string()))?;

        // Line 2: Comment
        let comment = lines[1].trim();

        // Parse comment for name/formula
        let (name, formula) = parse_comment(comment);

        // Lines 3+: Atom coordinates - extract elements for formula
        let mut elements: Vec<String> = Vec::new();

        for line in lines.iter().skip(2) {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 4 {
                let element = parts[0].to_string();
                if !elements.contains(&element) {
                    elements.push(element);
                }
            }
        }

        // Build formula from elements if not available
        let formula = if formula.is_empty() {
            elements.join("")
        } else {
            formula
        };

        let now = Utc::now();
        let material_id = options
            .material_id
            .clone()
            .unwrap_or_else(|| format!("wia-mat-{:08x}", generate_id() as u32));

        Ok(MaterialData {
            schema: Some("https://wia.live/material/v1/schema.json".to_string()),
            version: "1.0.0".to_string(),
            material_type: options
                .default_material_type
                .clone()
                .unwrap_or(MaterialType::Custom),
            material_id,
            timestamp: Timestamp {
                created: now,
                modified: None,
            },
            identity: Identity {
                name: if name.is_empty() {
                    formula.clone()
                } else {
                    name
                },
                formula,
                classification: Some(vec!["xyz_import".to_string()]),
            },
            structure: None, // XYZ doesn't provide crystal structure info
            properties: Properties::default(),
            measurement: None,
            provenance: None,
            external_references: None,
            meta: None,
        })
    }
}

impl Default for XyzExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl Exporter for XyzExporter {
    fn format(&self) -> ExportFormat {
        ExportFormat::Xyz
    }

    fn extensions(&self) -> &'static [&'static str] {
        &["xyz"]
    }

    fn export(&self, material: &MaterialData, options: &ExportOptions) -> IntegrationResult<String> {
        self.generate_xyz(material, options)
    }

    fn import(&self, content: &str, options: &ImportOptions) -> IntegrationResult<MaterialData> {
        self.parse_xyz(content, options)
    }
}

// Helper functions

fn extract_elements_from_formula(formula: &str) -> Vec<String> {
    let mut elements = Vec::new();
    let mut current = String::new();

    for c in formula.chars() {
        if c.is_uppercase() {
            if !current.is_empty() {
                let elem: String = current.chars().take_while(|c| c.is_alphabetic()).collect();
                if !elem.is_empty() && !elements.contains(&elem) {
                    elements.push(elem);
                }
            }
            current = c.to_string();
        } else if c.is_lowercase() {
            current.push(c);
        }
    }

    if !current.is_empty() {
        let elem: String = current.chars().take_while(|c| c.is_alphabetic()).collect();
        if !elem.is_empty() && !elements.contains(&elem) {
            elements.push(elem);
        }
    }

    if elements.is_empty() {
        elements.push("X".to_string());
    }

    elements
}

fn parse_comment(comment: &str) -> (String, String) {
    // Try to extract name and formula from comment
    // Common formats: "Name (Formula)", "Formula - ID", "Name"

    if comment.contains('(') && comment.contains(')') {
        let parts: Vec<&str> = comment.splitn(2, '(').collect();
        if parts.len() == 2 {
            let name = parts[0].trim().to_string();
            let formula = parts[1]
                .trim_end_matches(')')
                .split(')')
                .next()
                .unwrap_or("")
                .trim()
                .to_string();
            return (name, formula);
        }
    }

    if comment.contains(" - ") {
        let parts: Vec<&str> = comment.splitn(2, " - ").collect();
        if parts.len() == 2 {
            return (parts[0].trim().to_string(), String::new());
        }
    }

    (comment.to_string(), String::new())
}

fn generate_id() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_material() -> MaterialData {
        let now = Utc::now();
        MaterialData {
            schema: Some("https://wia.live/material/v1/schema.json".to_string()),
            version: "1.0.0".to_string(),
            material_type: MaterialType::Custom,
            material_id: "wia-mat-test1234".to_string(),
            timestamp: Timestamp {
                created: now,
                modified: None,
            },
            identity: Identity {
                name: "Water".to_string(),
                formula: "H2O".to_string(),
                classification: None,
            },
            structure: None,
            properties: Properties::default(),
            measurement: None,
            provenance: None,
            external_references: None,
            meta: None,
        }
    }

    #[test]
    fn test_xyz_export() {
        let exporter = XyzExporter::new();
        let material = sample_material();

        let xyz = exporter.export(&material, &ExportOptions::default()).unwrap();

        assert!(xyz.contains("Water"));
        assert!(xyz.contains("H2O"));
    }

    #[test]
    fn test_xyz_import() {
        let xyz_content = r#"3
Water molecule
O   0.000000   0.000000   0.000000
H   0.757000   0.586000   0.000000
H  -0.757000   0.586000   0.000000
"#;

        let exporter = XyzExporter::new();
        let material = exporter
            .import(xyz_content, &ImportOptions::default())
            .unwrap();

        assert_eq!(material.identity.name, "Water molecule");
    }

    #[test]
    fn test_xyz_roundtrip() {
        let exporter = XyzExporter::new();
        let original = sample_material();

        let xyz = exporter.export(&original, &ExportOptions::default()).unwrap();
        let imported = exporter.import(&xyz, &ImportOptions::default()).unwrap();

        assert!(imported.identity.name.contains("Water"));
    }

    #[test]
    fn test_parse_comment() {
        let (name, formula) = parse_comment("Water (H2O) - abc123");
        assert_eq!(name, "Water");
        assert_eq!(formula, "H2O");

        let (name2, _) = parse_comment("Simple Name - ID");
        assert_eq!(name2, "Simple Name");
    }

    #[test]
    fn test_extract_elements() {
        assert_eq!(
            extract_elements_from_formula("YBa2Cu3O7"),
            vec!["Y", "Ba", "Cu", "O"]
        );
        assert_eq!(extract_elements_from_formula("Fe2O3"), vec!["Fe", "O"]);
    }
}
