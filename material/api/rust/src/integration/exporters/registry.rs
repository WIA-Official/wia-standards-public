//! Exporter registry for managing format exporters

use std::collections::HashMap;
use std::path::Path;

use super::cif::CifExporter;
use super::poscar::PoscarExporter;
use super::traits::{ExportFormat, ExportOptions, Exporter, ImportOptions};
use super::xyz::XyzExporter;
use crate::integration::error::{IntegrationError, IntegrationResult};
use crate::types::MaterialData;

/// Registry for managing format exporters
pub struct ExporterRegistry {
    exporters: HashMap<ExportFormat, Box<dyn Exporter>>,
}

impl ExporterRegistry {
    /// Create a new empty registry
    pub fn new() -> Self {
        Self {
            exporters: HashMap::new(),
        }
    }

    /// Create a registry with default exporters
    pub fn with_defaults() -> Self {
        let mut registry = Self::new();

        registry.register(Box::new(CifExporter::new()));
        registry.register(Box::new(PoscarExporter::new()));
        registry.register(Box::new(XyzExporter::new()));

        registry
    }

    /// Register an exporter
    pub fn register(&mut self, exporter: Box<dyn Exporter>) {
        self.exporters.insert(exporter.format(), exporter);
    }

    /// Unregister an exporter by format
    pub fn unregister(&mut self, format: ExportFormat) -> Option<Box<dyn Exporter>> {
        self.exporters.remove(&format)
    }

    /// Get an exporter by format
    pub fn get(&self, format: ExportFormat) -> Option<&dyn Exporter> {
        self.exporters.get(&format).map(|e| e.as_ref())
    }

    /// Get an exporter by file extension
    pub fn get_by_extension(&self, ext: &str) -> Option<&dyn Exporter> {
        let format = ExportFormat::from_extension(ext)?;
        self.get(format)
    }

    /// Get an exporter by file path
    pub fn get_by_path(&self, path: &Path) -> Option<&dyn Exporter> {
        let format = ExportFormat::from_path(path)?;
        self.get(format)
    }

    /// List all registered formats
    pub fn list(&self) -> Vec<ExportFormat> {
        self.exporters.keys().cloned().collect()
    }

    /// Check if a format is supported
    pub fn supports(&self, format: ExportFormat) -> bool {
        self.exporters.contains_key(&format)
    }

    /// Check if a path is supported
    pub fn supports_path(&self, path: &Path) -> bool {
        ExportFormat::from_path(path)
            .map(|f| self.supports(f))
            .unwrap_or(false)
    }

    /// Export material to specified format
    pub fn export(
        &self,
        format: ExportFormat,
        material: &MaterialData,
        options: &ExportOptions,
    ) -> IntegrationResult<String> {
        let exporter = self
            .exporters
            .get(&format)
            .ok_or_else(|| IntegrationError::UnsupportedFormat(format.to_string()))?;

        exporter.export(material, options)
    }

    /// Export material to file
    pub fn export_to_file(
        &self,
        material: &MaterialData,
        path: &Path,
        options: &ExportOptions,
    ) -> IntegrationResult<()> {
        let exporter = self
            .get_by_path(path)
            .ok_or_else(|| IntegrationError::UnsupportedFormat(path.display().to_string()))?;

        exporter.export_to_file(material, path, options)
    }

    /// Import material from string with specified format
    pub fn import(
        &self,
        format: ExportFormat,
        content: &str,
        options: &ImportOptions,
    ) -> IntegrationResult<MaterialData> {
        let exporter = self
            .exporters
            .get(&format)
            .ok_or_else(|| IntegrationError::UnsupportedFormat(format.to_string()))?;

        exporter.import(content, options)
    }

    /// Import material from file (auto-detect format)
    pub fn import_file(&self, path: &Path, options: &ImportOptions) -> IntegrationResult<MaterialData> {
        let exporter = self
            .get_by_path(path)
            .ok_or_else(|| IntegrationError::UnsupportedFormat(path.display().to_string()))?;

        exporter.import_from_file(path, options)
    }

    /// Get number of registered exporters
    pub fn count(&self) -> usize {
        self.exporters.len()
    }
}

impl Default for ExporterRegistry {
    fn default() -> Self {
        Self::with_defaults()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_registry_with_defaults() {
        let registry = ExporterRegistry::with_defaults();

        assert!(registry.supports(ExportFormat::Cif));
        assert!(registry.supports(ExportFormat::Poscar));
        assert!(registry.supports(ExportFormat::Xyz));
        assert!(!registry.supports(ExportFormat::Pdb)); // Not implemented yet
    }

    #[test]
    fn test_registry_get_by_extension() {
        let registry = ExporterRegistry::with_defaults();

        assert!(registry.get_by_extension("cif").is_some());
        assert!(registry.get_by_extension("poscar").is_some());
        assert!(registry.get_by_extension("xyz").is_some());
        assert!(registry.get_by_extension("unknown").is_none());
    }

    #[test]
    fn test_registry_get_by_path() {
        let registry = ExporterRegistry::with_defaults();

        assert!(registry.get_by_path(Path::new("test.cif")).is_some());
        assert!(registry.get_by_path(Path::new("POSCAR")).is_some());
        assert!(registry.get_by_path(Path::new("structure.xyz")).is_some());
    }

    #[test]
    fn test_registry_list() {
        let registry = ExporterRegistry::with_defaults();
        let formats = registry.list();

        assert!(formats.contains(&ExportFormat::Cif));
        assert!(formats.contains(&ExportFormat::Poscar));
        assert!(formats.contains(&ExportFormat::Xyz));
    }

    #[test]
    fn test_registry_register_unregister() {
        let mut registry = ExporterRegistry::new();
        assert_eq!(registry.count(), 0);

        registry.register(Box::new(CifExporter::new()));
        assert_eq!(registry.count(), 1);
        assert!(registry.supports(ExportFormat::Cif));

        let removed = registry.unregister(ExportFormat::Cif);
        assert!(removed.is_some());
        assert_eq!(registry.count(), 0);
        assert!(!registry.supports(ExportFormat::Cif));
    }
}
