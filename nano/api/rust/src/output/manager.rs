//! Output manager for unified export interface

use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;

use crate::types::Molecule;
use super::{
    OutputAdapter, OutputError, OutputFormat, OutputResult,
    PdbAdapter, XyzAdapter, Mol2Adapter, CifAdapter,
    LammpsDataAdapter, GromacsGroAdapter,
};

/// Output manager for unified molecule export
pub struct OutputManager {
    adapters: HashMap<OutputFormat, Arc<dyn OutputAdapter>>,
}

impl Default for OutputManager {
    fn default() -> Self {
        Self::new()
    }
}

impl OutputManager {
    /// Create a new output manager with all default adapters
    pub fn new() -> Self {
        let mut manager = Self {
            adapters: HashMap::new(),
        };

        // Register default adapters
        manager.register(Arc::new(PdbAdapter::new()));
        manager.register(Arc::new(XyzAdapter::new()));
        manager.register(Arc::new(Mol2Adapter::new()));
        manager.register(Arc::new(CifAdapter::new()));
        manager.register(Arc::new(LammpsDataAdapter::new()));
        manager.register(Arc::new(GromacsGroAdapter::new()));

        manager
    }

    /// Register a custom adapter
    pub fn register(&mut self, adapter: Arc<dyn OutputAdapter>) {
        self.adapters.insert(adapter.format(), adapter);
    }

    /// Get adapter for a specific format
    pub fn get_adapter(&self, format: OutputFormat) -> Option<Arc<dyn OutputAdapter>> {
        self.adapters.get(&format).cloned()
    }

    /// Export molecule to specified format
    pub fn export(&self, format: OutputFormat, molecule: &Molecule) -> OutputResult<String> {
        let adapter = self.adapters.get(&format)
            .ok_or_else(|| OutputError::AdapterNotFound(format.to_string()))?;

        adapter.validate(molecule)?;
        adapter.export_molecule(molecule)
    }

    /// Export molecule to file
    pub fn export_to_file(
        &self,
        format: OutputFormat,
        molecule: &Molecule,
        path: &Path,
    ) -> OutputResult<()> {
        let adapter = self.adapters.get(&format)
            .ok_or_else(|| OutputError::AdapterNotFound(format.to_string()))?;

        adapter.validate(molecule)?;
        adapter.export_to_file(molecule, path)
    }

    /// List all registered formats
    pub fn available_formats(&self) -> Vec<OutputFormat> {
        self.adapters.keys().copied().collect()
    }

    /// Check if format is supported
    pub fn supports_format(&self, format: OutputFormat) -> bool {
        self.adapters.contains_key(&format)
    }

    /// Get recommended format for file extension
    pub fn format_for_extension(extension: &str) -> Option<OutputFormat> {
        match extension.to_lowercase().as_str() {
            "pdb" => Some(OutputFormat::Pdb),
            "xyz" => Some(OutputFormat::Xyz),
            "mol2" => Some(OutputFormat::Mol2),
            "cif" => Some(OutputFormat::Cif),
            "data" | "lammps" => Some(OutputFormat::LammpsData),
            "top" => Some(OutputFormat::GromacsTop),
            "gro" => Some(OutputFormat::GromacsGro),
            "json" => Some(OutputFormat::Json),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manager_creation() {
        let manager = OutputManager::new();
        assert!(manager.supports_format(OutputFormat::Pdb));
        assert!(manager.supports_format(OutputFormat::Xyz));
        assert!(manager.supports_format(OutputFormat::Mol2));
    }

    #[test]
    fn test_format_for_extension() {
        assert_eq!(OutputManager::format_for_extension("pdb"), Some(OutputFormat::Pdb));
        assert_eq!(OutputManager::format_for_extension("xyz"), Some(OutputFormat::Xyz));
        assert_eq!(OutputManager::format_for_extension("unknown"), None);
    }
}
