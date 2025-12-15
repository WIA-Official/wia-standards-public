//! # WIA Biotech - Rust API
//!
//! WIA Biotechnology Standard implementation in Rust.
//!
//! This crate provides types, utilities, and adapters for working with
//! biotechnology data including gene sequences, CRISPR experiments,
//! protein structures, and synthetic biology parts.
//!
//! ## Features
//!
//! - **Sequence Analysis**: DNA/RNA validation, GC content, reverse complement
//! - **CRISPR Design**: PAM site finding, guide RNA design, experiment management
//! - **Protein Structures**: AlphaFold confidence metrics, domain analysis
//! - **Synthetic Biology**: BioBrick parts, assembly design
//!
//! ## Quick Start
//!
//! ```rust
//! use wia_bio::prelude::*;
//!
//! // Create a DNA sequence
//! let sequence = create_sequence(
//!     "My Gene",
//!     "ATCGATCGATCGATCG",
//!     SequenceType::Dna,
//! ).unwrap();
//!
//! // Calculate GC content
//! let gc = calculate_gc_content("ATCGATCG");
//! assert!((gc - 0.5).abs() < 0.001);
//!
//! // Find PAM sites for CRISPR
//! let pam_sites = find_pam_sites_ngg("ATCGATCGNGGCGATCG");
//! ```
//!
//! ## Modules
//!
//! - [`types`]: Data type definitions (Sequence, CrisprExperiment, ProteinStructure, etc.)
//! - [`core`]: Core business logic for biotech operations
//! - [`adapters`]: Data source adapters (simulator, file I/O)
//! - [`error`]: Error types and result aliases
//!
//! ## Compliance
//!
//! This implementation follows the WIA Biotech Data Format Specification v1.0.0
//! and is compatible with:
//! - FASTA/FASTQ sequence formats
//! - PDB/mmCIF structure formats
//! - SBOL synthetic biology standard
//! - BioBrick assembly standard

#![warn(missing_docs)]
#![warn(rustdoc::missing_doc_code_examples)]

pub mod adapters;
pub mod core;
pub mod ecosystem;
pub mod error;
pub mod protocol;
pub mod transport;
pub mod types;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::core::*;
    pub use crate::error::{BioError, BioResult};
    pub use crate::types::*;
    pub use crate::adapters::{BioProvider, SimulatorAdapter};
    pub use crate::protocol::{
        BioMessage, MessageType, MessagePayload, MessageBuilder,
        ProtocolHandler, ProtocolCapability, ConnectionStatus,
    };
    pub use crate::transport::{
        ITransport, TransportConfig, TransportState, TransportError,
        TransportFactory, TransportType,
    };
    pub use crate::ecosystem::{
        IEcosystemAdapter, AdapterType, AdapterConfig, AdapterError,
        EcosystemManager, HealthStatus, ExportResult,
    };
}

// Re-export commonly used items at crate root
pub use crate::core::*;
pub use crate::error::{BioError, BioResult};
pub use crate::types::*;

/// WIA Biotech version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA specification version
pub const WIA_SPEC_VERSION: &str = "1.0.0";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
        assert_eq!(WIA_SPEC_VERSION, "1.0.0");
    }

    #[test]
    fn test_prelude_imports() {
        use crate::prelude::*;

        // Test that types are available
        let _seq_type = SequenceType::Dna;
        let _crispr_type = CrisprSystemType::CrisprCas9;
        let _part_type = PartType::Promoter;
    }

    #[test]
    fn test_sequence_workflow() {
        // Create sequence
        let seq = create_sequence("Test Gene", "ATCGATCGATCG", SequenceType::Dna).unwrap();
        assert!(seq.sequence_id.starts_with("seq-"));
        assert_eq!(seq.length_bp, 12);

        // Validate
        assert!(validate_dna_sequence("ATCGATCG").is_ok());
        assert!(validate_dna_sequence("ATCGX").is_err());

        // GC content
        let gc = calculate_gc_content("ATCGATCG");
        assert!((gc - 0.5).abs() < 0.001);

        // Reverse complement
        assert_eq!(reverse_complement("ATCG"), "CGAT");

        // Transcription
        assert_eq!(transcribe_dna_to_rna("ATCG"), "AUCG");
    }

    #[test]
    fn test_crispr_workflow() {
        // Find PAM sites
        let sites = find_pam_sites_ngg("ATCGATCGATCGNGGCGATCGATCG");
        assert!(!sites.is_empty());

        // Create experiment
        let grna = GuideRna {
            grna_id: "grna-001".to_string(),
            name: Some("Test gRNA".to_string()),
            sequence: "AUCGAUCGAUCGAUCGAUCG".to_string(),
            target_start: Some(1),
            target_end: Some(20),
            strand: Some(Strand::Plus),
            pam: Some("NGG".to_string()),
            off_target_score: Some(0.95),
            on_target_score: Some(0.88),
        };

        let exp = create_crispr_experiment(
            "Test Knockout",
            "ABC1",
            CrisprSystemType::CrisprCas9,
            vec![grna],
        )
        .unwrap();

        assert!(exp.experiment_id.starts_with("crispr-exp-"));
        assert_eq!(exp.target.gene_symbol, "ABC1");
    }

    #[test]
    fn test_structure_workflow() {
        // Create structure
        let structure = create_protein_structure(
            "Test Protein",
            Some("P12345"),
            PredictionMethod::Alphafold,
            500,
        );

        assert!(structure.structure_id.starts_with("struct-"));
        assert_eq!(structure.protein.name, "Test Protein");

        // pLDDT interpretation
        assert!(interpret_plddt(95.0).contains("Very high"));
        assert!(interpret_plddt(40.0).contains("Very low"));

        // Mean pLDDT calculation
        let scores = vec![90.0, 85.0, 80.0, 75.0, 70.0];
        assert!((calculate_mean_plddt(&scores) - 80.0).abs() < 0.001);
    }

    #[test]
    fn test_synbio_workflow() {
        // Create part
        let part = create_biopart(
            "Strong Promoter",
            PartType::Promoter,
            "TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC",
        )
        .unwrap();

        assert!(part.part_id.starts_with("part-"));
        assert_eq!(part.part_info.part_type, PartType::Promoter);

        // BioBrick scars
        let scarred = add_biobrick_scars("ATCGATCG");
        assert!(scarred.contains("GAATTCGCGGCCGCTTCTAGAG")); // Prefix
        assert!(scarred.contains("TACTAGTAGCGGCCGCTGCAG")); // Suffix

        // Create assembly
        let assembly = create_assembly(
            "GFP Expression",
            AssemblyStandard::Biobrick,
            vec![
                ("part-001".to_string(), PartType::Promoter),
                ("part-002".to_string(), PartType::Rbs),
                ("part-003".to_string(), PartType::Cds),
            ],
        );

        assert!(assembly.assembly_id.starts_with("asm-"));
        assert_eq!(assembly.parts.len(), 3);
    }

    #[test]
    fn test_project_workflow() {
        let project = create_project("Gene Therapy Study", ProjectCategory::GeneTherapy);

        assert!(project.project_id.starts_with("proj-"));
        assert_eq!(project.project_info.name, "Gene Therapy Study");
        assert_eq!(project.project_info.status, Some(ProjectStatus::Active));
    }
}
