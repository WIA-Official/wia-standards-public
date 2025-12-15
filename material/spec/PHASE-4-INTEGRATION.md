# WIA Material Integration Specification v1.0.0
# Phase 4: Ecosystem Integration Standard

---

**Version**: 1.0.0
**Date**: 2025-12-14
**Status**: Draft
**Authors**: Claude Code (Opus 4.5)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Integration Architecture](#2-integration-architecture)
3. [Data Providers](#3-data-providers)
4. [Exporters](#4-exporters)
5. [OPTIMADE Compatibility](#5-optimade-compatibility)
6. [Materials Project Integration](#6-materials-project-integration)
7. [File Format Converters](#7-file-format-converters)
8. [Integration Manager](#8-integration-manager)
9. [Error Handling](#9-error-handling)
10. [Examples](#10-examples)

---

## 1. Introduction

### 1.1 Purpose

Phase 4 defines the ecosystem integration layer for WIA Material Standard. This specification enables:

- Data exchange with external material databases
- Import/export to standard crystallographic formats
- Interoperability with existing materials science tools
- Unified interface for heterogeneous data sources

### 1.2 Scope

```
Integration Layer Scope:

External Data Sources          File Formats             WIA Ecosystem
├─ Materials Project    ←→    ├─ CIF           ←→     ├─ WIA Quantum
├─ OPTIMADE Providers   ←→    ├─ POSCAR        ←→     ├─ WIA Standard Hub
├─ NOMAD               ←→     ├─ XYZ           ←→     └─ Future Standards
└─ AFLOW               ←→     └─ PDB           ←→
```

### 1.3 Relationship to Previous Phases

| Phase | Role | Phase 4 Integration |
|-------|------|---------------------|
| Phase 1 | Data Format | Core MaterialData structure |
| Phase 2 | Rust SDK | Integration module home |
| Phase 3 | Protocol | External API communication |
| **Phase 4** | **Integration** | **Ecosystem connectivity** |

---

## 2. Integration Architecture

### 2.1 Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                 Application Layer                            │
│              (User Applications, CLI, GUI)                   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   WIA Material SDK                           │
│                    (Phase 1-3)                               │
│  ┌─────────┐  ┌─────────┐  ┌──────────┐  ┌──────────────┐   │
│  │  Core   │  │  Types  │  │ Protocol │  │  Transport   │   │
│  └─────────┘  └─────────┘  └──────────┘  └──────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  Integration Layer                           │
│                     (Phase 4)                                │
│                                                              │
│  ┌──────────────────┐  ┌──────────────────┐                 │
│  │   Providers      │  │    Exporters     │                 │
│  │                  │  │                  │                 │
│  │ ┌──────────────┐ │  │ ┌──────────────┐ │                 │
│  │ │  OPTIMADE    │ │  │ │    CIF       │ │                 │
│  │ └──────────────┘ │  │ └──────────────┘ │                 │
│  │ ┌──────────────┐ │  │ ┌──────────────┐ │                 │
│  │ │  Materials   │ │  │ │   POSCAR     │ │                 │
│  │ │   Project    │ │  │ └──────────────┘ │                 │
│  │ └──────────────┘ │  │ ┌──────────────┐ │                 │
│  │ ┌──────────────┐ │  │ │    XYZ       │ │                 │
│  │ │   Mock       │ │  │ └──────────────┘ │                 │
│  │ └──────────────┘ │  │                  │                 │
│  └──────────────────┘  └──────────────────┘                 │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              IntegrationManager                       │   │
│  │    (Unified interface for all integration features)  │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
         ┌────────────────────┴────────────────────┐
         │                                         │
    External APIs                             File System
    ├─ api.materialsproject.org              ├─ *.cif
    ├─ optimade.org providers                ├─ POSCAR
    └─ nomad-lab.eu                          └─ *.xyz
```

### 2.2 Module Structure

```
src/integration/
├── mod.rs                    # Module root
├── manager.rs                # IntegrationManager
├── providers/
│   ├── mod.rs
│   ├── traits.rs             # DataProvider trait
│   ├── optimade.rs           # OPTIMADE client
│   ├── materials_project.rs  # Materials Project client
│   └── mock.rs               # Mock provider for testing
├── exporters/
│   ├── mod.rs
│   ├── traits.rs             # Exporter trait
│   ├── cif.rs                # CIF format
│   ├── poscar.rs             # VASP POSCAR format
│   └── xyz.rs                # XYZ format
└── converters/
    ├── mod.rs
    └── structure.rs          # Structure conversion utilities
```

---

## 3. Data Providers

### 3.1 DataProvider Trait

```rust
use async_trait::async_trait;

/// Data provider configuration
#[derive(Debug, Clone)]
pub struct ProviderConfig {
    /// Provider base URL
    pub base_url: String,

    /// API key (if required)
    pub api_key: Option<String>,

    /// Request timeout in seconds
    pub timeout_secs: u64,

    /// Maximum results per query
    pub max_results: usize,
}

/// Query parameters for searching materials
#[derive(Debug, Clone, Default)]
pub struct ProviderQuery {
    /// Filter by material type
    pub material_type: Option<MaterialType>,

    /// Filter by elements (e.g., ["Fe", "O"])
    pub elements: Option<Vec<String>>,

    /// Filter by formula
    pub formula: Option<String>,

    /// Filter by band gap range
    pub band_gap_range: Option<(f64, f64)>,

    /// Maximum results
    pub limit: Option<usize>,

    /// Pagination offset
    pub offset: Option<usize>,
}

/// Provider status information
#[derive(Debug, Clone)]
pub struct ProviderStatus {
    pub name: String,
    pub connected: bool,
    pub version: String,
    pub available_count: Option<usize>,
}

/// Trait for external data providers
#[async_trait]
pub trait DataProvider: Send + Sync {
    /// Provider name
    fn name(&self) -> &str;

    /// Connect to the provider
    async fn connect(&mut self, config: ProviderConfig) -> IntegrationResult<()>;

    /// Disconnect from the provider
    async fn disconnect(&mut self) -> IntegrationResult<()>;

    /// Check connection status
    fn is_connected(&self) -> bool;

    /// Get provider status
    async fn status(&self) -> IntegrationResult<ProviderStatus>;

    /// Search for materials
    async fn search(&self, query: &ProviderQuery) -> IntegrationResult<Vec<MaterialData>>;

    /// Get material by external ID
    async fn get_by_id(&self, external_id: &str) -> IntegrationResult<MaterialData>;

    /// Get material by WIA material ID
    async fn get_by_wia_id(&self, wia_id: &str) -> IntegrationResult<Option<MaterialData>>;
}
```

### 3.2 Provider Registry

```rust
/// Registry for managing multiple data providers
pub struct ProviderRegistry {
    providers: HashMap<String, Box<dyn DataProvider>>,
}

impl ProviderRegistry {
    /// Create a new registry
    pub fn new() -> Self;

    /// Register a provider
    pub fn register(&mut self, provider: Box<dyn DataProvider>);

    /// Unregister a provider
    pub fn unregister(&mut self, name: &str) -> Option<Box<dyn DataProvider>>;

    /// Get a provider by name
    pub fn get(&self, name: &str) -> Option<&dyn DataProvider>;

    /// Get mutable reference to provider
    pub fn get_mut(&mut self, name: &str) -> Option<&mut Box<dyn DataProvider>>;

    /// List all registered providers
    pub fn list(&self) -> Vec<&str>;

    /// Search across all connected providers
    pub async fn search_all(&self, query: &ProviderQuery) -> IntegrationResult<Vec<(String, MaterialData)>>;
}
```

---

## 4. Exporters

### 4.1 Exporter Trait

```rust
/// Supported export formats
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

/// Export options
#[derive(Debug, Clone, Default)]
pub struct ExportOptions {
    /// Include comments/metadata
    pub include_metadata: bool,

    /// Precision for floating point numbers
    pub precision: Option<usize>,

    /// Use fractional (direct) or Cartesian coordinates
    pub use_fractional: bool,

    /// Supercell dimensions (for visualization)
    pub supercell: Option<(usize, usize, usize)>,
}

/// Import options
#[derive(Debug, Clone, Default)]
pub struct ImportOptions {
    /// Default material type if not detectable
    pub default_material_type: Option<MaterialType>,

    /// Override material ID
    pub material_id: Option<String>,

    /// Tolerance for symmetry detection
    pub symmetry_tolerance: Option<f64>,
}

/// Trait for format exporters/importers
pub trait Exporter: Send + Sync {
    /// Get supported format
    fn format(&self) -> ExportFormat;

    /// Get file extensions
    fn extensions(&self) -> &[&str];

    /// Export material to string
    fn export(&self, material: &MaterialData, options: &ExportOptions) -> IntegrationResult<String>;

    /// Export material to file
    fn export_to_file(
        &self,
        material: &MaterialData,
        path: &Path,
        options: &ExportOptions,
    ) -> IntegrationResult<()>;

    /// Import material from string
    fn import(&self, content: &str, options: &ImportOptions) -> IntegrationResult<MaterialData>;

    /// Import material from file
    fn import_from_file(
        &self,
        path: &Path,
        options: &ImportOptions,
    ) -> IntegrationResult<MaterialData>;

    /// Check if format is supported for the given path
    fn supports_path(&self, path: &Path) -> bool {
        if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
            self.extensions().contains(&ext.to_lowercase().as_str())
        } else {
            false
        }
    }
}
```

### 4.2 Exporter Registry

```rust
/// Registry for managing format exporters
pub struct ExporterRegistry {
    exporters: HashMap<ExportFormat, Box<dyn Exporter>>,
}

impl ExporterRegistry {
    /// Create registry with default exporters
    pub fn with_defaults() -> Self;

    /// Register an exporter
    pub fn register(&mut self, exporter: Box<dyn Exporter>);

    /// Get exporter by format
    pub fn get(&self, format: ExportFormat) -> Option<&dyn Exporter>;

    /// Get exporter by file extension
    pub fn get_by_extension(&self, ext: &str) -> Option<&dyn Exporter>;

    /// Export material to specified format
    pub fn export(
        &self,
        format: ExportFormat,
        material: &MaterialData,
        options: &ExportOptions,
    ) -> IntegrationResult<String>;

    /// Import material from file (auto-detect format)
    pub fn import_file(
        &self,
        path: &Path,
        options: &ImportOptions,
    ) -> IntegrationResult<MaterialData>;
}
```

---

## 5. OPTIMADE Compatibility

### 5.1 OPTIMADE Client

```rust
/// OPTIMADE client implementation
pub struct OptimadeProvider {
    config: ProviderConfig,
    connected: bool,
    base_url: String,
}

impl OptimadeProvider {
    /// Create a new OPTIMADE client
    pub fn new() -> Self;

    /// Create with specific provider
    pub fn with_provider(provider_url: &str) -> Self;

    /// Convert OPTIMADE structure to WIA MaterialData
    fn convert_structure(&self, optimade: &OptimadeStructure) -> IntegrationResult<MaterialData>;

    /// Convert WIA filter to OPTIMADE filter string
    fn build_filter(&self, query: &ProviderQuery) -> String;
}

#[async_trait]
impl DataProvider for OptimadeProvider {
    fn name(&self) -> &str {
        "optimade"
    }

    async fn search(&self, query: &ProviderQuery) -> IntegrationResult<Vec<MaterialData>> {
        let filter = self.build_filter(query);
        let url = format!(
            "{}/v1/structures?filter={}",
            self.base_url,
            urlencoding::encode(&filter)
        );

        // HTTP request via transport layer
        let response = self.fetch(&url).await?;

        // Convert OPTIMADE response to WIA format
        let structures: Vec<MaterialData> = response
            .data
            .iter()
            .map(|s| self.convert_structure(s))
            .collect::<Result<Vec<_>, _>>()?;

        Ok(structures)
    }

    // ... other trait methods
}
```

### 5.2 OPTIMADE Filter Translation

```rust
/// Convert WIA query to OPTIMADE filter syntax
fn build_optimade_filter(query: &ProviderQuery) -> String {
    let mut filters = Vec::new();

    // Elements filter
    if let Some(ref elements) = query.elements {
        let elements_str = elements
            .iter()
            .map(|e| format!("\"{}\"", e))
            .collect::<Vec<_>>()
            .join(", ");
        filters.push(format!("elements HAS ALL {}", elements_str));
    }

    // Formula filter
    if let Some(ref formula) = query.formula {
        filters.push(format!("chemical_formula_reduced = \"{}\"", formula));
    }

    // Band gap filter
    if let Some((min, max)) = query.band_gap_range {
        filters.push(format!("band_gap >= {} AND band_gap <= {}", min, max));
    }

    filters.join(" AND ")
}
```

### 5.3 Structure Mapping

| OPTIMADE Field | WIA Material Field |
|---------------|-------------------|
| `id` | `external_references.optimade_id` |
| `chemical_formula_reduced` | `identity.formula` |
| `elements` | Derived from `identity.formula` |
| `nelements` | Calculated |
| `lattice_vectors` | `structure.lattice_parameters` |
| `cartesian_site_positions` | `structure.atom_positions` |
| `species_at_sites` | `structure.composition` |

---

## 6. Materials Project Integration

### 6.1 Materials Project Client

```rust
/// Materials Project client
pub struct MaterialsProjectProvider {
    config: ProviderConfig,
    connected: bool,
    api_key: Option<String>,
}

impl MaterialsProjectProvider {
    /// Create a new Materials Project client
    pub fn new() -> Self;

    /// Create with API key
    pub fn with_api_key(api_key: &str) -> Self;

    /// Convert MP structure to WIA MaterialData
    fn convert_material(&self, mp_data: &MpMaterial) -> IntegrationResult<MaterialData>;
}

#[async_trait]
impl DataProvider for MaterialsProjectProvider {
    fn name(&self) -> &str {
        "materials_project"
    }

    async fn get_by_id(&self, mp_id: &str) -> IntegrationResult<MaterialData> {
        let url = format!(
            "https://api.materialsproject.org/materials/{}",
            mp_id
        );

        let response = self.fetch_with_auth(&url).await?;
        self.convert_material(&response)
    }

    async fn search(&self, query: &ProviderQuery) -> IntegrationResult<Vec<MaterialData>> {
        let mut params = Vec::new();

        if let Some(ref elements) = query.elements {
            params.push(format!("elements={}", elements.join(",")));
        }

        if let Some(ref formula) = query.formula {
            params.push(format!("formula={}", formula));
        }

        let url = format!(
            "https://api.materialsproject.org/materials/summary?{}",
            params.join("&")
        );

        let response = self.fetch_with_auth(&url).await?;
        response
            .iter()
            .map(|m| self.convert_material(m))
            .collect()
    }

    // ... other trait methods
}
```

### 6.2 MP Field Mapping

| Materials Project Field | WIA Material Field |
|------------------------|-------------------|
| `material_id` | `external_references.materials_project_id` |
| `formula_pretty` | `identity.formula` |
| `structure` | `structure` |
| `band_gap` | `properties.electrical.band_gap_ev` |
| `formation_energy_per_atom` | `properties.thermal.formation_energy_ev` |
| `density` | `properties.mechanical.density_kg_m3` |
| `symmetry.crystal_system` | `structure.crystal_system` |
| `symmetry.symbol` | `structure.space_group` |

---

## 7. File Format Converters

### 7.1 CIF Exporter

```rust
/// CIF format exporter
pub struct CifExporter;

impl CifExporter {
    /// Generate CIF content from MaterialData
    fn generate_cif(&self, material: &MaterialData, options: &ExportOptions) -> String {
        let mut cif = String::new();

        // Data block
        cif.push_str(&format!("data_{}\n", material.material_id));

        // Chemical formula
        cif.push_str(&format!(
            "_chemical_formula_sum '{}'\n",
            material.identity.formula
        ));

        // Cell parameters
        if let Some(ref structure) = material.structure {
            if let Some(ref params) = structure.lattice_parameters {
                if let Some(a) = params.a_angstrom {
                    cif.push_str(&format!("_cell_length_a {:.4}\n", a));
                }
                if let Some(b) = params.b_angstrom {
                    cif.push_str(&format!("_cell_length_b {:.4}\n", b));
                }
                if let Some(c) = params.c_angstrom {
                    cif.push_str(&format!("_cell_length_c {:.4}\n", c));
                }
                if let Some(alpha) = params.alpha_degree {
                    cif.push_str(&format!("_cell_angle_alpha {:.2}\n", alpha));
                }
                if let Some(beta) = params.beta_degree {
                    cif.push_str(&format!("_cell_angle_beta {:.2}\n", beta));
                }
                if let Some(gamma) = params.gamma_degree {
                    cif.push_str(&format!("_cell_angle_gamma {:.2}\n", gamma));
                }
            }

            // Space group
            if let Some(ref sg) = structure.space_group {
                cif.push_str(&format!("_symmetry_space_group_name_H-M '{}'\n", sg));
            }
        }

        cif
    }
}

impl Exporter for CifExporter {
    fn format(&self) -> ExportFormat {
        ExportFormat::Cif
    }

    fn extensions(&self) -> &[&str] {
        &["cif"]
    }

    fn export(&self, material: &MaterialData, options: &ExportOptions) -> IntegrationResult<String> {
        Ok(self.generate_cif(material, options))
    }

    fn import(&self, content: &str, options: &ImportOptions) -> IntegrationResult<MaterialData> {
        // Parse CIF content
        let parsed = parse_cif(content)?;
        self.convert_to_material(&parsed, options)
    }
}
```

### 7.2 POSCAR Exporter

```rust
/// VASP POSCAR format exporter
pub struct PoscarExporter;

impl PoscarExporter {
    /// Generate POSCAR content
    fn generate_poscar(&self, material: &MaterialData, options: &ExportOptions) -> IntegrationResult<String> {
        let mut poscar = String::new();

        // Comment line
        poscar.push_str(&format!("{} - {}\n", material.identity.name, material.material_id));

        // Scaling factor
        poscar.push_str("1.0\n");

        // Lattice vectors
        if let Some(ref structure) = material.structure {
            if let Some(ref params) = structure.lattice_parameters {
                let a = params.a_angstrom.unwrap_or(1.0);
                let b = params.b_angstrom.unwrap_or(1.0);
                let c = params.c_angstrom.unwrap_or(1.0);

                // Simplified: assumes orthorhombic cell
                poscar.push_str(&format!("  {:12.8}  {:12.8}  {:12.8}\n", a, 0.0, 0.0));
                poscar.push_str(&format!("  {:12.8}  {:12.8}  {:12.8}\n", 0.0, b, 0.0));
                poscar.push_str(&format!("  {:12.8}  {:12.8}  {:12.8}\n", 0.0, 0.0, c));
            }
        }

        // Element symbols and counts
        // ...

        Ok(poscar)
    }
}

impl Exporter for PoscarExporter {
    fn format(&self) -> ExportFormat {
        ExportFormat::Poscar
    }

    fn extensions(&self) -> &[&str] {
        &["poscar", "vasp", "contcar"]
    }

    // ... implementation
}
```

### 7.3 XYZ Exporter

```rust
/// XYZ format exporter
pub struct XyzExporter;

impl Exporter for XyzExporter {
    fn format(&self) -> ExportFormat {
        ExportFormat::Xyz
    }

    fn extensions(&self) -> &[&str] {
        &["xyz"]
    }

    fn export(&self, material: &MaterialData, options: &ExportOptions) -> IntegrationResult<String> {
        let mut xyz = String::new();

        // Atom count (placeholder)
        xyz.push_str("0\n");

        // Comment line
        xyz.push_str(&format!("{} - {}\n", material.identity.name, material.identity.formula));

        // Atom coordinates would go here
        // Format: Element  X  Y  Z

        Ok(xyz)
    }

    // ... import implementation
}
```

---

## 8. Integration Manager

### 8.1 IntegrationManager

```rust
/// Unified integration manager
pub struct IntegrationManager {
    providers: ProviderRegistry,
    exporters: ExporterRegistry,
}

impl IntegrationManager {
    /// Create a new integration manager with default components
    pub fn new() -> Self {
        let mut manager = Self {
            providers: ProviderRegistry::new(),
            exporters: ExporterRegistry::with_defaults(),
        };

        // Register default providers
        manager.providers.register(Box::new(OptimadeProvider::new()));
        manager.providers.register(Box::new(MockProvider::new()));

        manager
    }

    /// Create with Materials Project API key
    pub fn with_materials_project(api_key: &str) -> Self {
        let mut manager = Self::new();
        manager.providers.register(Box::new(
            MaterialsProjectProvider::with_api_key(api_key)
        ));
        manager
    }

    // Provider operations

    /// Connect to a provider
    pub async fn connect_provider(
        &mut self,
        name: &str,
        config: ProviderConfig,
    ) -> IntegrationResult<()> {
        if let Some(provider) = self.providers.get_mut(name) {
            provider.connect(config).await
        } else {
            Err(IntegrationError::ProviderNotFound(name.to_string()))
        }
    }

    /// Search across all connected providers
    pub async fn search(&self, query: &ProviderQuery) -> IntegrationResult<Vec<MaterialData>> {
        self.providers.search_all(query).await
            .map(|results| results.into_iter().map(|(_, m)| m).collect())
    }

    /// Get material by external ID from specific provider
    pub async fn fetch(
        &self,
        provider: &str,
        external_id: &str,
    ) -> IntegrationResult<MaterialData> {
        if let Some(p) = self.providers.get(provider) {
            p.get_by_id(external_id).await
        } else {
            Err(IntegrationError::ProviderNotFound(provider.to_string()))
        }
    }

    // Exporter operations

    /// Export material to format
    pub fn export(
        &self,
        format: ExportFormat,
        material: &MaterialData,
    ) -> IntegrationResult<String> {
        self.exporters.export(format, material, &ExportOptions::default())
    }

    /// Export material to file
    pub fn export_to_file(
        &self,
        material: &MaterialData,
        path: &Path,
    ) -> IntegrationResult<()> {
        let ext = path.extension()
            .and_then(|e| e.to_str())
            .ok_or_else(|| IntegrationError::UnsupportedFormat("unknown".to_string()))?;

        if let Some(exporter) = self.exporters.get_by_extension(ext) {
            exporter.export_to_file(material, path, &ExportOptions::default())
        } else {
            Err(IntegrationError::UnsupportedFormat(ext.to_string()))
        }
    }

    /// Import material from file
    pub fn import_file(&self, path: &Path) -> IntegrationResult<MaterialData> {
        self.exporters.import_file(path, &ImportOptions::default())
    }

    /// Convert between formats
    pub fn convert(
        &self,
        input_path: &Path,
        output_path: &Path,
    ) -> IntegrationResult<()> {
        let material = self.import_file(input_path)?;
        self.export_to_file(&material, output_path)
    }
}
```

### 8.2 Builder Pattern

```rust
/// Builder for IntegrationManager
pub struct IntegrationManagerBuilder {
    mp_api_key: Option<String>,
    optimade_providers: Vec<String>,
    custom_exporters: Vec<Box<dyn Exporter>>,
}

impl IntegrationManagerBuilder {
    pub fn new() -> Self {
        Self {
            mp_api_key: None,
            optimade_providers: Vec::new(),
            custom_exporters: Vec::new(),
        }
    }

    /// Add Materials Project with API key
    pub fn with_materials_project(mut self, api_key: &str) -> Self {
        self.mp_api_key = Some(api_key.to_string());
        self
    }

    /// Add OPTIMADE provider
    pub fn with_optimade_provider(mut self, url: &str) -> Self {
        self.optimade_providers.push(url.to_string());
        self
    }

    /// Add custom exporter
    pub fn with_exporter(mut self, exporter: Box<dyn Exporter>) -> Self {
        self.custom_exporters.push(exporter);
        self
    }

    /// Build the manager
    pub fn build(self) -> IntegrationManager {
        let mut manager = IntegrationManager::new();

        if let Some(api_key) = self.mp_api_key {
            manager.providers.register(Box::new(
                MaterialsProjectProvider::with_api_key(&api_key)
            ));
        }

        for url in self.optimade_providers {
            manager.providers.register(Box::new(
                OptimadeProvider::with_provider(&url)
            ));
        }

        for exporter in self.custom_exporters {
            manager.exporters.register(exporter);
        }

        manager
    }
}
```

---

## 9. Error Handling

### 9.1 Error Types

```rust
/// Integration error types
#[derive(Debug, thiserror::Error)]
pub enum IntegrationError {
    #[error("Provider not found: {0}")]
    ProviderNotFound(String),

    #[error("Provider not connected: {0}")]
    ProviderNotConnected(String),

    #[error("External ID not found: {0}")]
    ExternalIdNotFound(String),

    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    #[error("Parse error: {0}")]
    ParseError(String),

    #[error("Export error: {0}")]
    ExportError(String),

    #[error("Conversion error: {0}")]
    ConversionError(String),

    #[error("Network error: {0}")]
    NetworkError(String),

    #[error("Authentication error: {0}")]
    AuthError(String),

    #[error("Rate limited")]
    RateLimited,

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Material error: {0}")]
    MaterialError(#[from] crate::error::MaterialError),
}

pub type IntegrationResult<T> = Result<T, IntegrationError>;
```

---

## 10. Examples

### 10.1 Basic Integration Usage

```rust
use wia_material::integration::{
    IntegrationManager, ProviderConfig, ProviderQuery, ExportFormat,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create integration manager
    let mut manager = IntegrationManager::new();

    // Connect to OPTIMADE provider
    manager.connect_provider("optimade", ProviderConfig {
        base_url: "https://providers.optimade.org".to_string(),
        api_key: None,
        timeout_secs: 30,
        max_results: 100,
    }).await?;

    // Search for materials
    let query = ProviderQuery {
        elements: Some(vec!["Fe".to_string(), "O".to_string()]),
        limit: Some(10),
        ..Default::default()
    };

    let materials = manager.search(&query).await?;

    println!("Found {} materials", materials.len());

    // Export first result to CIF
    if let Some(material) = materials.first() {
        let cif = manager.export(ExportFormat::Cif, material)?;
        println!("CIF output:\n{}", cif);
    }

    Ok(())
}
```

### 10.2 Materials Project Integration

```rust
use wia_material::integration::{
    IntegrationManagerBuilder, ProviderConfig,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create manager with Materials Project
    let mut manager = IntegrationManagerBuilder::new()
        .with_materials_project("YOUR_MP_API_KEY")
        .build();

    // Connect
    manager.connect_provider("materials_project", ProviderConfig {
        base_url: "https://api.materialsproject.org".to_string(),
        api_key: Some("YOUR_MP_API_KEY".to_string()),
        timeout_secs: 30,
        max_results: 100,
    }).await?;

    // Fetch specific material
    let material = manager.fetch("materials_project", "mp-13").await?;

    println!("Fetched: {} ({})", material.identity.name, material.identity.formula);

    // Export to POSCAR
    manager.export_to_file(&material, Path::new("output.poscar"))?;

    Ok(())
}
```

### 10.3 File Format Conversion

```rust
use wia_material::integration::IntegrationManager;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manager = IntegrationManager::new();

    // Import CIF, export as POSCAR
    let material = manager.import_file(Path::new("input.cif"))?;
    manager.export_to_file(&material, Path::new("output.poscar"))?;

    // Or convert directly
    manager.convert(
        Path::new("structure.cif"),
        Path::new("POSCAR"),
    )?;

    Ok(())
}
```

### 10.4 Multi-Provider Search

```rust
use wia_material::integration::{
    IntegrationManagerBuilder, ProviderConfig, ProviderQuery,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut manager = IntegrationManagerBuilder::new()
        .with_materials_project("YOUR_MP_API_KEY")
        .with_optimade_provider("https://optimade.odbx.science")
        .with_optimade_provider("https://optimade.materialsproject.org")
        .build();

    // Connect to all providers
    // ...

    // Search across all providers
    let query = ProviderQuery {
        formula: Some("Fe2O3".to_string()),
        ..Default::default()
    };

    let results = manager.search(&query).await?;

    for material in results {
        println!("Found: {} from {}",
            material.identity.name,
            material.external_references.map(|r| r.materials_project_id.or(r.optimade_id)).flatten().unwrap_or("unknown".to_string())
        );
    }

    Ok(())
}
```

---

## Appendix A: Supported OPTIMADE Providers

| Provider | URL | Description |
|----------|-----|-------------|
| Materials Project | optimade.materialsproject.org | US DOE database |
| AFLOW | aflow.org/API/optimade | Duke University |
| OQMD | oqmd.org/optimade | Open Quantum Materials Database |
| NOMAD | nomad-lab.eu/prod/v1/api/optimade | European repository |
| COD | crystallography.net/cod/optimade | Open crystallography |

---

## Appendix B: Format Comparison

| Feature | CIF | POSCAR | XYZ | WIA JSON |
|---------|-----|--------|-----|----------|
| Lattice | Yes | Yes | No | Yes |
| Symmetry | Yes | No | No | Yes |
| Properties | Limited | No | No | Full |
| Metadata | Yes | Comment | Comment | Full |
| Binary data | No | No | No | No |
| Human readable | Yes | Yes | Yes | Yes |

---

<div align="center">

**WIA Material Integration Specification v1.0.0**

---

弘益人間 🤟

</div>
