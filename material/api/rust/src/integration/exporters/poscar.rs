//! VASP POSCAR format exporter
//!
//! Exports material data to POSCAR format for VASP simulations.

use chrono::Utc;

use super::traits::{ExportFormat, ExportOptions, Exporter, ImportOptions};
use crate::integration::error::{IntegrationError, IntegrationResult};
use crate::types::{
    Identity, LatticeParameters, MaterialData, MaterialType, Properties, Structure, Timestamp,
};

/// VASP POSCAR format exporter
pub struct PoscarExporter;

impl PoscarExporter {
    /// Create a new POSCAR exporter
    pub fn new() -> Self {
        Self
    }

    /// Generate POSCAR content from MaterialData
    fn generate_poscar(
        &self,
        material: &MaterialData,
        options: &ExportOptions,
    ) -> IntegrationResult<String> {
        let structure = material
            .structure
            .as_ref()
            .ok_or(IntegrationError::MissingStructure)?;

        let params = structure
            .lattice_parameters
            .as_ref()
            .ok_or_else(|| IntegrationError::ExportError("Missing lattice parameters".to_string()))?;

        let precision = options.precision;
        let mut poscar = String::new();

        // Line 1: Comment
        poscar.push_str(&format!(
            "{} - {}\n",
            material.identity.name, material.material_id
        ));

        // Line 2: Universal scaling factor
        poscar.push_str("1.0\n");

        // Lines 3-5: Lattice vectors
        let a = params.a_angstrom.unwrap_or(1.0);
        let b = params.b_angstrom.unwrap_or(1.0);
        let c = params.c_angstrom.unwrap_or(1.0);
        let alpha = params.alpha_degree.unwrap_or(90.0).to_radians();
        let beta = params.beta_degree.unwrap_or(90.0).to_radians();
        let gamma = params.gamma_degree.unwrap_or(90.0).to_radians();

        // Calculate lattice vectors from parameters
        let (v1, v2, v3) = lattice_vectors_from_params(a, b, c, alpha, beta, gamma);

        poscar.push_str(&format!(
            "  {:>15.*}  {:>15.*}  {:>15.*}\n",
            precision, v1.0, precision, v1.1, precision, v1.2
        ));
        poscar.push_str(&format!(
            "  {:>15.*}  {:>15.*}  {:>15.*}\n",
            precision, v2.0, precision, v2.1, precision, v2.2
        ));
        poscar.push_str(&format!(
            "  {:>15.*}  {:>15.*}  {:>15.*}\n",
            precision, v3.0, precision, v3.1, precision, v3.2
        ));

        // Line 6: Element symbols - extract from formula
        let elements = extract_elements_from_formula(&material.identity.formula);
        poscar.push_str(&format!("  {}\n", elements.join("  ")));

        // Line 7: Number of atoms per element (placeholder)
        let counts: Vec<usize> = elements.iter().map(|_| 1).collect();
        poscar.push_str(&format!(
            "  {}\n",
            counts
                .iter()
                .map(|c| c.to_string())
                .collect::<Vec<_>>()
                .join("  ")
        ));

        // Line 8: Coordinate type
        if options.use_fractional {
            poscar.push_str("Direct\n");
        } else {
            poscar.push_str("Cartesian\n");
        }

        // Add placeholder positions
        poscar.push_str("  0.0  0.0  0.0  # Placeholder - structure details not available\n");

        Ok(poscar)
    }

    /// Parse POSCAR content into MaterialData
    fn parse_poscar(
        &self,
        content: &str,
        options: &ImportOptions,
    ) -> IntegrationResult<MaterialData> {
        let lines: Vec<&str> = content.lines().collect();

        if lines.len() < 8 {
            return Err(IntegrationError::ParseError(
                "POSCAR: insufficient lines".to_string(),
            ));
        }

        // Line 1: Comment
        let comment = lines[0].trim();

        // Line 2: Scaling factor
        let scale: f64 = lines[1]
            .trim()
            .parse()
            .map_err(|_| IntegrationError::ParseError("POSCAR: invalid scaling factor".to_string()))?;

        // Lines 3-5: Lattice vectors
        let v1 = parse_vector(lines[2])?;
        let v2 = parse_vector(lines[3])?;
        let v3 = parse_vector(lines[4])?;

        // Apply scaling
        let v1 = (v1.0 * scale, v1.1 * scale, v1.2 * scale);
        let v2 = (v2.0 * scale, v2.1 * scale, v2.2 * scale);
        let v3 = (v3.0 * scale, v3.1 * scale, v3.2 * scale);

        // Calculate lattice parameters from vectors
        let (a, b, c, alpha, beta, gamma) = lattice_params_from_vectors(&v1, &v2, &v3);

        // Line 6: Elements (optional in VASP 4, required in VASP 5+)
        let line6 = lines[5].trim();
        let first_token = line6.split_whitespace().next().unwrap_or("");

        let elements: Vec<String> = if first_token.parse::<usize>().is_ok() {
            vec!["X".to_string()]
        } else {
            line6.split_whitespace().map(|s| s.to_string()).collect()
        };

        // Build formula from elements
        let formula = elements.join("");

        let lattice_parameters = LatticeParameters {
            a_angstrom: Some(a),
            b_angstrom: Some(b),
            c_angstrom: Some(c),
            alpha_degree: Some(alpha),
            beta_degree: Some(beta),
            gamma_degree: Some(gamma),
        };

        let structure = Structure {
            crystal_system: None,
            space_group: None,
            lattice_parameters: Some(lattice_parameters),
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
                name: comment.to_string(),
                formula,
                classification: Some(vec!["poscar_import".to_string()]),
            },
            structure: Some(structure),
            properties: Properties::default(),
            measurement: None,
            provenance: None,
            external_references: None,
            meta: None,
        })
    }
}

impl Default for PoscarExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl Exporter for PoscarExporter {
    fn format(&self) -> ExportFormat {
        ExportFormat::Poscar
    }

    fn extensions(&self) -> &'static [&'static str] {
        &["poscar", "vasp", "contcar"]
    }

    fn export(&self, material: &MaterialData, options: &ExportOptions) -> IntegrationResult<String> {
        self.generate_poscar(material, options)
    }

    fn import(&self, content: &str, options: &ImportOptions) -> IntegrationResult<MaterialData> {
        self.parse_poscar(content, options)
    }
}

// Helper functions

fn lattice_vectors_from_params(
    a: f64,
    b: f64,
    c: f64,
    alpha: f64,
    beta: f64,
    gamma: f64,
) -> ((f64, f64, f64), (f64, f64, f64), (f64, f64, f64)) {
    let cos_alpha = alpha.cos();
    let cos_beta = beta.cos();
    let cos_gamma = gamma.cos();
    let sin_gamma = gamma.sin();

    let v1 = (a, 0.0, 0.0);
    let v2 = (b * cos_gamma, b * sin_gamma, 0.0);

    let c_x = c * cos_beta;
    let c_y = c * (cos_alpha - cos_beta * cos_gamma) / sin_gamma;
    let c_z = (c * c - c_x * c_x - c_y * c_y).sqrt();
    let v3 = (c_x, c_y, c_z);

    (v1, v2, v3)
}

fn lattice_params_from_vectors(
    v1: &(f64, f64, f64),
    v2: &(f64, f64, f64),
    v3: &(f64, f64, f64),
) -> (f64, f64, f64, f64, f64, f64) {
    let a = vector_length(v1);
    let b = vector_length(v2);
    let c = vector_length(v3);

    let alpha = angle_between(v2, v3).to_degrees();
    let beta = angle_between(v1, v3).to_degrees();
    let gamma = angle_between(v1, v2).to_degrees();

    (a, b, c, alpha, beta, gamma)
}

fn vector_length(v: &(f64, f64, f64)) -> f64 {
    (v.0 * v.0 + v.1 * v.1 + v.2 * v.2).sqrt()
}

fn dot_product(a: &(f64, f64, f64), b: &(f64, f64, f64)) -> f64 {
    a.0 * b.0 + a.1 * b.1 + a.2 * b.2
}

fn angle_between(a: &(f64, f64, f64), b: &(f64, f64, f64)) -> f64 {
    let cos_angle = dot_product(a, b) / (vector_length(a) * vector_length(b));
    cos_angle.clamp(-1.0, 1.0).acos()
}

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

fn parse_vector(line: &str) -> IntegrationResult<(f64, f64, f64)> {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() < 3 {
        return Err(IntegrationError::ParseError(
            "POSCAR: invalid vector".to_string(),
        ));
    }

    let x = parts[0]
        .parse()
        .map_err(|_| IntegrationError::ParseError("POSCAR: invalid x".to_string()))?;
    let y = parts[1]
        .parse()
        .map_err(|_| IntegrationError::ParseError("POSCAR: invalid y".to_string()))?;
    let z = parts[2]
        .parse()
        .map_err(|_| IntegrationError::ParseError("POSCAR: invalid z".to_string()))?;

    Ok((x, y, z))
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
                name: "Iron".to_string(),
                formula: "Fe".to_string(),
                classification: None,
            },
            structure: Some(Structure {
                crystal_system: None,
                space_group: None,
                lattice_parameters: Some(LatticeParameters {
                    a_angstrom: Some(2.87),
                    b_angstrom: Some(2.87),
                    c_angstrom: Some(2.87),
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
    fn test_poscar_export() {
        let exporter = PoscarExporter::new();
        let material = sample_material();

        let poscar = exporter.export(&material, &ExportOptions::default()).unwrap();

        assert!(poscar.contains("Iron"));
        assert!(poscar.contains("1.0"));
        assert!(poscar.contains("Fe"));
    }

    #[test]
    fn test_poscar_import() {
        let poscar_content = r#"Iron BCC
1.0
  2.87000000  0.00000000  0.00000000
  0.00000000  2.87000000  0.00000000
  0.00000000  0.00000000  2.87000000
  Fe
  2
Direct
  0.00000000  0.00000000  0.00000000
  0.50000000  0.50000000  0.50000000
"#;

        let exporter = PoscarExporter::new();
        let material = exporter.import(poscar_content, &ImportOptions::default()).unwrap();

        assert!(material.identity.formula.contains("Fe"));
        assert!(material.structure.is_some());
    }

    #[test]
    fn test_extract_elements_from_formula() {
        assert_eq!(
            extract_elements_from_formula("YBa2Cu3O7"),
            vec!["Y", "Ba", "Cu", "O"]
        );
        assert_eq!(extract_elements_from_formula("Fe2O3"), vec!["Fe", "O"]);
    }
}
