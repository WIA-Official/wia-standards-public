//! CIF (Crystallographic Information File) exporter
//!
//! Exports material data to CIF format for crystallography tools.

use chrono::Utc;

use super::traits::{ExportFormat, ExportOptions, Exporter, ImportOptions};
use crate::integration::error::{IntegrationError, IntegrationResult};
use crate::types::{
    CrystalSystem, Identity, LatticeParameters, MaterialData, MaterialType, Properties,
    Structure, Timestamp,
};

/// CIF format exporter
pub struct CifExporter;

impl CifExporter {
    /// Create a new CIF exporter
    pub fn new() -> Self {
        Self
    }

    /// Generate CIF content from MaterialData
    fn generate_cif(&self, material: &MaterialData, options: &ExportOptions) -> String {
        let precision = options.precision;
        let mut cif = String::new();

        // Data block header
        let data_name = sanitize_cif_name(&material.material_id);
        cif.push_str(&format!("data_{}\n", data_name));
        cif.push('\n');

        // Metadata as comments
        if options.include_metadata {
            cif.push_str(&format!(
                "# WIA Material: {}\n",
                material.identity.name
            ));
            cif.push_str(&format!(
                "# Generated: {}\n",
                Utc::now().format("%Y-%m-%d %H:%M:%S UTC")
            ));
            cif.push_str("#\n\n");
        }

        // Chemical formula
        cif.push_str(&format!(
            "_chemical_formula_sum '{}'\n",
            material.identity.formula
        ));

        // Chemical name
        cif.push_str(&format!(
            "_chemical_name_common '{}'\n",
            material.identity.name
        ));
        cif.push('\n');

        // Cell parameters
        if let Some(ref structure) = material.structure {
            if let Some(ref params) = structure.lattice_parameters {
                if let Some(a) = params.a_angstrom {
                    cif.push_str(&format!("_cell_length_a {:.*}\n", precision, a));
                }
                if let Some(b) = params.b_angstrom {
                    cif.push_str(&format!("_cell_length_b {:.*}\n", precision, b));
                }
                if let Some(c) = params.c_angstrom {
                    cif.push_str(&format!("_cell_length_c {:.*}\n", precision, c));
                }
                if let Some(alpha) = params.alpha_degree {
                    cif.push_str(&format!("_cell_angle_alpha {:.*}\n", 2, alpha));
                }
                if let Some(beta) = params.beta_degree {
                    cif.push_str(&format!("_cell_angle_beta {:.*}\n", 2, beta));
                }
                if let Some(gamma) = params.gamma_degree {
                    cif.push_str(&format!("_cell_angle_gamma {:.*}\n", 2, gamma));
                }
                cif.push('\n');
            }

            // Space group
            if options.include_symmetry {
                if let Some(ref sg) = structure.space_group {
                    cif.push_str(&format!("_symmetry_space_group_name_H-M '{}'\n", sg));
                }
                if let Some(ref cs) = structure.crystal_system {
                    cif.push_str(&format!(
                        "_symmetry_cell_setting {}\n",
                        crystal_system_to_string(cs)
                    ));
                }
                cif.push('\n');
            }
        }

        // Add external references as comments
        if options.include_metadata {
            if let Some(ref ext_refs) = material.external_references {
                if let Some(ref mp_id) = ext_refs.materials_project_id {
                    cif.push_str(&format!("# Materials Project ID: {}\n", mp_id));
                }
            }
        }

        cif
    }

    /// Parse CIF content into MaterialData
    fn parse_cif(&self, content: &str, options: &ImportOptions) -> IntegrationResult<MaterialData> {
        let mut formula = String::new();
        let mut name = String::new();
        let mut space_group = None;
        let mut crystal_system = None;

        let mut a: Option<f64> = None;
        let mut b: Option<f64> = None;
        let mut c: Option<f64> = None;
        let mut alpha: Option<f64> = None;
        let mut beta: Option<f64> = None;
        let mut gamma: Option<f64> = None;

        // Simple line-by-line parser
        for line in content.lines() {
            let line = line.trim();

            if line.starts_with("_chemical_formula_sum") {
                formula = extract_quoted_value(line).unwrap_or_default();
            } else if line.starts_with("_chemical_name_common") {
                name = extract_quoted_value(line).unwrap_or_default();
            } else if line.starts_with("_symmetry_space_group_name_H-M") {
                space_group = extract_quoted_value(line);
            } else if line.starts_with("_symmetry_cell_setting") {
                crystal_system = parse_crystal_system(line);
            } else if line.starts_with("_cell_length_a") {
                a = extract_float_value(line);
            } else if line.starts_with("_cell_length_b") {
                b = extract_float_value(line);
            } else if line.starts_with("_cell_length_c") {
                c = extract_float_value(line);
            } else if line.starts_with("_cell_angle_alpha") {
                alpha = extract_float_value(line);
            } else if line.starts_with("_cell_angle_beta") {
                beta = extract_float_value(line);
            } else if line.starts_with("_cell_angle_gamma") {
                gamma = extract_float_value(line);
            }
        }

        if formula.is_empty() {
            return Err(IntegrationError::ParseError(
                "CIF: missing _chemical_formula_sum".to_string(),
            ));
        }

        if name.is_empty() {
            name = formula.clone();
        }

        let lattice_parameters = if a.is_some() || b.is_some() || c.is_some() {
            Some(LatticeParameters {
                a_angstrom: a,
                b_angstrom: b,
                c_angstrom: c,
                alpha_degree: alpha,
                beta_degree: beta,
                gamma_degree: gamma,
            })
        } else {
            None
        };

        let structure = if lattice_parameters.is_some() || space_group.is_some() {
            Some(Structure {
                crystal_system,
                space_group,
                lattice_parameters,
            })
        } else {
            None
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
                name,
                formula,
                classification: Some(vec!["cif_import".to_string()]),
            },
            structure,
            properties: Properties::default(),
            measurement: None,
            provenance: None,
            external_references: None,
            meta: None,
        })
    }
}

impl Default for CifExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl Exporter for CifExporter {
    fn format(&self) -> ExportFormat {
        ExportFormat::Cif
    }

    fn extensions(&self) -> &'static [&'static str] {
        &["cif"]
    }

    fn export(&self, material: &MaterialData, options: &ExportOptions) -> IntegrationResult<String> {
        Ok(self.generate_cif(material, options))
    }

    fn import(&self, content: &str, options: &ImportOptions) -> IntegrationResult<MaterialData> {
        self.parse_cif(content, options)
    }
}

// Helper functions

fn sanitize_cif_name(name: &str) -> String {
    name.chars()
        .map(|c| if c.is_alphanumeric() || c == '_' { c } else { '_' })
        .collect()
}

fn crystal_system_to_string(cs: &CrystalSystem) -> &'static str {
    match cs {
        CrystalSystem::Triclinic => "triclinic",
        CrystalSystem::Monoclinic => "monoclinic",
        CrystalSystem::Orthorhombic => "orthorhombic",
        CrystalSystem::Tetragonal => "tetragonal",
        CrystalSystem::Trigonal => "trigonal",
        CrystalSystem::Rhombohedral => "rhombohedral",
        CrystalSystem::Hexagonal => "hexagonal",
        CrystalSystem::Cubic => "cubic",
        CrystalSystem::Amorphous => "amorphous",
    }
}

fn extract_quoted_value(line: &str) -> Option<String> {
    if let Some(start) = line.find('\'') {
        if let Some(end) = line[start + 1..].find('\'') {
            return Some(line[start + 1..start + 1 + end].to_string());
        }
    }
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() >= 2 {
        Some(parts[1..].join(" "))
    } else {
        None
    }
}

fn extract_float_value(line: &str) -> Option<f64> {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() >= 2 {
        let value_str = parts[1].split('(').next().unwrap_or(parts[1]);
        value_str.parse().ok()
    } else {
        None
    }
}

fn parse_crystal_system(line: &str) -> Option<CrystalSystem> {
    let lower = line.to_lowercase();
    if lower.contains("cubic") {
        Some(CrystalSystem::Cubic)
    } else if lower.contains("tetragonal") {
        Some(CrystalSystem::Tetragonal)
    } else if lower.contains("orthorhombic") {
        Some(CrystalSystem::Orthorhombic)
    } else if lower.contains("hexagonal") {
        Some(CrystalSystem::Hexagonal)
    } else if lower.contains("trigonal") {
        Some(CrystalSystem::Trigonal)
    } else if lower.contains("rhombohedral") {
        Some(CrystalSystem::Rhombohedral)
    } else if lower.contains("monoclinic") {
        Some(CrystalSystem::Monoclinic)
    } else if lower.contains("triclinic") {
        Some(CrystalSystem::Triclinic)
    } else if lower.contains("amorphous") {
        Some(CrystalSystem::Amorphous)
    } else {
        None
    }
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
            material_type: MaterialType::Superconductor,
            material_id: "wia-mat-test1234".to_string(),
            timestamp: Timestamp {
                created: now,
                modified: None,
            },
            identity: Identity {
                name: "YBCO".to_string(),
                formula: "YBa2Cu3O7".to_string(),
                classification: None,
            },
            structure: Some(Structure {
                crystal_system: Some(CrystalSystem::Orthorhombic),
                space_group: Some("Pmmm".to_string()),
                lattice_parameters: Some(LatticeParameters {
                    a_angstrom: Some(3.82),
                    b_angstrom: Some(3.89),
                    c_angstrom: Some(11.68),
                    alpha_degree: Some(90.0),
                    beta_degree: Some(90.0),
                    gamma_degree: Some(90.0),
                }),
            }),
            properties: Properties::default(),
            measurement: None,
            provenance: None,
            external_references: None,
            meta: None,
        }
    }

    #[test]
    fn test_cif_export() {
        let exporter = CifExporter::new();
        let material = sample_material();

        let cif = exporter.export(&material, &ExportOptions::default()).unwrap();

        assert!(cif.contains("data_wia_mat_test1234"));
        assert!(cif.contains("_chemical_formula_sum 'YBa2Cu3O7'"));
        assert!(cif.contains("_cell_length_a"));
        assert!(cif.contains("_symmetry_space_group_name_H-M 'Pmmm'"));
    }

    #[test]
    fn test_cif_import() {
        let cif_content = r#"
data_test
_chemical_formula_sum 'Fe2O3'
_chemical_name_common 'Hematite'
_cell_length_a 5.035
_cell_length_b 5.035
_cell_length_c 13.75
_cell_angle_alpha 90.0
_cell_angle_beta 90.0
_cell_angle_gamma 120.0
_symmetry_space_group_name_H-M 'R-3c'
_symmetry_cell_setting trigonal
"#;

        let exporter = CifExporter::new();
        let material = exporter.import(cif_content, &ImportOptions::default()).unwrap();

        assert_eq!(material.identity.formula, "Fe2O3");
        assert_eq!(material.identity.name, "Hematite");
        assert!(material.structure.is_some());

        let structure = material.structure.unwrap();
        assert_eq!(structure.space_group, Some("R-3c".to_string()));
        assert_eq!(structure.crystal_system, Some(CrystalSystem::Trigonal));
    }

    #[test]
    fn test_sanitize_cif_name() {
        assert_eq!(sanitize_cif_name("test-123"), "test_123");
        assert_eq!(sanitize_cif_name("wia-mat-abc"), "wia_mat_abc");
    }
}
