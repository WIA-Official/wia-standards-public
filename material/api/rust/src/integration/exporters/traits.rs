//! Exporter traits and types

use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::integration::error::IntegrationResult;
use crate::types::{MaterialData, MaterialType};

/// Supported export formats
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ExportFormat {
    /// Crystallographic Information File
    Cif,
    /// VASP POSCAR
    Poscar,
    /// XYZ coordinates
    Xyz,
    /// Protein Data Bank
    Pdb,
    /// WIA Material JSON
    WiaJson,
}

impl ExportFormat {
    /// Get file extensions for this format
    pub fn extensions(&self) -> &[&str] {
        match self {
            ExportFormat::Cif => &["cif"],
            ExportFormat::Poscar => &["poscar", "vasp", "contcar"],
            ExportFormat::Xyz => &["xyz"],
            ExportFormat::Pdb => &["pdb"],
            ExportFormat::WiaJson => &["json", "wia.json"],
        }
    }

    /// Get primary extension
    pub fn primary_extension(&self) -> &str {
        match self {
            ExportFormat::Cif => "cif",
            ExportFormat::Poscar => "poscar",
            ExportFormat::Xyz => "xyz",
            ExportFormat::Pdb => "pdb",
            ExportFormat::WiaJson => "json",
        }
    }

    /// Get format from file extension
    pub fn from_extension(ext: &str) -> Option<Self> {
        let ext_lower = ext.to_lowercase();
        match ext_lower.as_str() {
            "cif" => Some(ExportFormat::Cif),
            "poscar" | "vasp" | "contcar" => Some(ExportFormat::Poscar),
            "xyz" => Some(ExportFormat::Xyz),
            "pdb" => Some(ExportFormat::Pdb),
            "json" => Some(ExportFormat::WiaJson),
            _ => None,
        }
    }

    /// Get format from file path
    pub fn from_path(path: &Path) -> Option<Self> {
        // Check for POSCAR/CONTCAR without extension
        if let Some(filename) = path.file_name().and_then(|n| n.to_str()) {
            let filename_upper = filename.to_uppercase();
            if filename_upper == "POSCAR" || filename_upper == "CONTCAR" {
                return Some(ExportFormat::Poscar);
            }
        }

        // Check by extension
        path.extension()
            .and_then(|e| e.to_str())
            .and_then(Self::from_extension)
    }
}

impl std::fmt::Display for ExportFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExportFormat::Cif => write!(f, "CIF"),
            ExportFormat::Poscar => write!(f, "POSCAR"),
            ExportFormat::Xyz => write!(f, "XYZ"),
            ExportFormat::Pdb => write!(f, "PDB"),
            ExportFormat::WiaJson => write!(f, "WIA JSON"),
        }
    }
}

/// Export options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportOptions {
    /// Include comments/metadata in output
    pub include_metadata: bool,

    /// Precision for floating point numbers
    pub precision: usize,

    /// Use fractional (direct) coordinates instead of Cartesian
    pub use_fractional: bool,

    /// Supercell dimensions for visualization (a, b, c)
    pub supercell: Option<(usize, usize, usize)>,

    /// Include symmetry information
    pub include_symmetry: bool,

    /// Pretty print JSON output
    pub pretty_print: bool,
}

impl Default for ExportOptions {
    fn default() -> Self {
        Self {
            include_metadata: true,
            precision: 6,
            use_fractional: true,
            supercell: None,
            include_symmetry: true,
            pretty_print: true,
        }
    }
}

impl ExportOptions {
    /// Create options for minimal output
    pub fn minimal() -> Self {
        Self {
            include_metadata: false,
            precision: 4,
            use_fractional: true,
            supercell: None,
            include_symmetry: false,
            pretty_print: false,
        }
    }

    /// Create options with custom precision
    pub fn with_precision(mut self, precision: usize) -> Self {
        self.precision = precision;
        self
    }

    /// Set supercell dimensions
    pub fn with_supercell(mut self, a: usize, b: usize, c: usize) -> Self {
        self.supercell = Some((a, b, c));
        self
    }

    /// Use Cartesian coordinates
    pub fn cartesian(mut self) -> Self {
        self.use_fractional = false;
        self
    }
}

/// Import options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImportOptions {
    /// Default material type if not detectable
    pub default_material_type: Option<MaterialType>,

    /// Override material ID (generate new if None)
    pub material_id: Option<String>,

    /// Tolerance for symmetry detection
    pub symmetry_tolerance: Option<f64>,

    /// Source attribution
    pub source: Option<String>,

    /// Validate imported data
    pub validate: bool,
}

impl Default for ImportOptions {
    fn default() -> Self {
        Self {
            default_material_type: Some(MaterialType::Custom),
            material_id: None,
            symmetry_tolerance: Some(0.01),
            source: None,
            validate: true,
        }
    }
}

impl ImportOptions {
    /// Set default material type
    pub fn with_type(mut self, material_type: MaterialType) -> Self {
        self.default_material_type = Some(material_type);
        self
    }

    /// Set material ID
    pub fn with_id(mut self, id: impl Into<String>) -> Self {
        self.material_id = Some(id.into());
        self
    }

    /// Set source
    pub fn with_source(mut self, source: impl Into<String>) -> Self {
        self.source = Some(source.into());
        self
    }

    /// Skip validation
    pub fn skip_validation(mut self) -> Self {
        self.validate = false;
        self
    }
}

/// Trait for format exporters/importers
pub trait Exporter: Send + Sync {
    /// Get supported format
    fn format(&self) -> ExportFormat;

    /// Get file extensions (must be implemented by each exporter)
    fn extensions(&self) -> &'static [&'static str];

    /// Export material to string
    fn export(&self, material: &MaterialData, options: &ExportOptions) -> IntegrationResult<String>;

    /// Export material to file
    fn export_to_file(
        &self,
        material: &MaterialData,
        path: &Path,
        options: &ExportOptions,
    ) -> IntegrationResult<()> {
        let content = self.export(material, options)?;
        std::fs::write(path, content)?;
        Ok(())
    }

    /// Import material from string
    fn import(&self, content: &str, options: &ImportOptions) -> IntegrationResult<MaterialData>;

    /// Import material from file
    fn import_from_file(
        &self,
        path: &Path,
        options: &ImportOptions,
    ) -> IntegrationResult<MaterialData> {
        let content = std::fs::read_to_string(path)?;
        self.import(&content, options)
    }

    /// Check if format is supported for the given path
    fn supports_path(&self, path: &Path) -> bool {
        ExportFormat::from_path(path)
            .map(|f| f == self.format())
            .unwrap_or(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_extensions() {
        assert_eq!(ExportFormat::Cif.extensions(), &["cif"]);
        assert!(ExportFormat::Poscar.extensions().contains(&"poscar"));
    }

    #[test]
    fn test_format_from_extension() {
        assert_eq!(ExportFormat::from_extension("cif"), Some(ExportFormat::Cif));
        assert_eq!(
            ExportFormat::from_extension("POSCAR"),
            Some(ExportFormat::Poscar)
        );
        assert_eq!(ExportFormat::from_extension("xyz"), Some(ExportFormat::Xyz));
        assert_eq!(ExportFormat::from_extension("unknown"), None);
    }

    #[test]
    fn test_format_from_path() {
        use std::path::PathBuf;

        assert_eq!(
            ExportFormat::from_path(&PathBuf::from("test.cif")),
            Some(ExportFormat::Cif)
        );
        assert_eq!(
            ExportFormat::from_path(&PathBuf::from("POSCAR")),
            Some(ExportFormat::Poscar)
        );
        assert_eq!(
            ExportFormat::from_path(&PathBuf::from("structure.xyz")),
            Some(ExportFormat::Xyz)
        );
    }

    #[test]
    fn test_export_options_default() {
        let opts = ExportOptions::default();
        assert!(opts.include_metadata);
        assert_eq!(opts.precision, 6);
        assert!(opts.use_fractional);
    }

    #[test]
    fn test_export_options_builder() {
        let opts = ExportOptions::default()
            .with_precision(8)
            .with_supercell(2, 2, 2)
            .cartesian();

        assert_eq!(opts.precision, 8);
        assert_eq!(opts.supercell, Some((2, 2, 2)));
        assert!(!opts.use_fractional);
    }
}
