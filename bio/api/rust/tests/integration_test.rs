//! Integration tests for WIA Biotech Rust API

use wia_bio::prelude::*;
use wia_bio::adapters::SimulatorAdapter;

#[tokio::test]
async fn test_full_crispr_workflow() {
    // 1. Create simulator adapter with sample data
    let adapter = SimulatorAdapter::with_sample_data().unwrap();

    // 2. Create a target sequence
    let target_sequence = create_sequence(
        "Target Gene BRCA1",
        "ATCGATCGATCGATCGNGGCGATCGATCGATCGNGGCGATCGATCGATCG",
        SequenceType::Dna,
    )
    .unwrap();

    // Store the sequence
    let seq_id = adapter.store_sequence(target_sequence.clone()).await.unwrap();
    assert!(seq_id.starts_with("seq-"));

    // 3. Find PAM sites in the target
    let pam_sites = find_pam_sites_ngg(
        "ATCGATCGATCGATCGNGGCGATCGATCGATCGNGGCGATCGATCGATCG"
    );
    assert!(!pam_sites.is_empty(), "Should find PAM sites");

    // 4. Design guide RNAs
    let grna = GuideRna {
        grna_id: "grna-test-001".to_string(),
        name: Some("BRCA1 gRNA 1".to_string()),
        sequence: "AUCGAUCGAUCGAUCGAUCG".to_string(),
        target_start: Some(1),
        target_end: Some(20),
        strand: Some(Strand::Plus),
        pam: Some("NGG".to_string()),
        off_target_score: Some(0.92),
        on_target_score: Some(0.85),
    };

    // 5. Create CRISPR experiment
    let experiment = create_crispr_experiment(
        "BRCA1 Knockout Study",
        "BRCA1",
        CrisprSystemType::CrisprCas9,
        vec![grna],
    )
    .unwrap();

    // Store the experiment
    let exp_id = adapter.store_experiment(experiment.clone()).await.unwrap();
    assert!(exp_id.starts_with("crispr-exp-"));

    // 6. Simulate editing results
    let results = adapter.simulate_editing_results(&exp_id).unwrap();
    assert!(results.editing_efficiency.is_some());

    let efficiency = results.editing_efficiency.unwrap();
    assert!(efficiency.overall_efficiency.unwrap() > 0.0);
    assert!(efficiency.overall_efficiency.unwrap() <= 1.0);

    // 7. Verify off-target analysis
    assert!(results.off_target_analysis.is_some());
    let off_target = results.off_target_analysis.unwrap();
    assert!(off_target.sites_detected >= 0);
}

#[tokio::test]
async fn test_full_protein_structure_workflow() {
    let adapter = SimulatorAdapter::new();

    // 1. Simulate AlphaFold prediction
    let (structure, confidence) = adapter.simulate_alphafold_prediction("Human p53", 393);

    // 2. Verify structure
    assert!(structure.structure_id.starts_with("struct-"));
    assert_eq!(structure.protein.name, "Human p53");
    assert_eq!(structure.protein.length_aa, Some(393));

    // 3. Verify quality metrics
    let quality = structure.quality.clone().unwrap();
    assert!(quality.mean_plddt.is_some());
    let mean_plddt = quality.mean_plddt.unwrap();
    assert!(mean_plddt >= 0.0 && mean_plddt <= 100.0);

    // 4. Verify confidence data
    let plddt = confidence.plddt.unwrap();
    assert_eq!(plddt.values.len(), 393);

    // 5. Check interpretation
    assert!(confidence.interpretation.is_some());

    // 6. Store structure
    let struct_id = adapter.store_structure(structure).await.unwrap();
    assert!(struct_id.starts_with("struct-"));

    // 7. Retrieve structure
    let retrieved = adapter.get_structure(&struct_id).await.unwrap();
    assert_eq!(retrieved.protein.name, "Human p53");
}

#[tokio::test]
async fn test_full_synbio_workflow() {
    let adapter = SimulatorAdapter::new();

    // 1. Create promoter part
    let promoter = create_biopart(
        "J23100",
        PartType::Promoter,
        "TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC",
    )
    .unwrap();
    let prom_id = adapter.store_part(promoter).await.unwrap();

    // 2. Create RBS part
    let rbs = create_biopart(
        "B0034",
        PartType::Rbs,
        "AAAGAGGAGAAA",
    )
    .unwrap();
    let rbs_id = adapter.store_part(rbs).await.unwrap();

    // 3. Create GFP coding sequence
    let gfp = create_biopart(
        "E0040",
        PartType::Cds,
        "ATGCGTAAAGGAGAAGAACTTTTCACTGGAGTTGTC", // Truncated for test
    )
    .unwrap();
    let gfp_id = adapter.store_part(gfp).await.unwrap();

    // 4. Create terminator
    let terminator = create_biopart(
        "B0015",
        PartType::Terminator,
        "CCAGGCATCAAATAAAACGAAAGGCTCAGTCGAAAGACTGGGCC",
    )
    .unwrap();
    let term_id = adapter.store_part(terminator).await.unwrap();

    // 5. Create assembly
    let assembly = create_assembly(
        "GFP Expression Device",
        AssemblyStandard::Biobrick,
        vec![
            (prom_id.clone(), PartType::Promoter),
            (rbs_id.clone(), PartType::Rbs),
            (gfp_id.clone(), PartType::Cds),
            (term_id.clone(), PartType::Terminator),
        ],
    );

    assert_eq!(assembly.parts.len(), 4);
    assert_eq!(assembly.parts[0].position, 1);
    assert_eq!(assembly.parts[3].position, 4);

    // 6. Verify parts can be retrieved
    let parts = adapter.list_parts().await.unwrap();
    assert_eq!(parts.len(), 4);
}

#[tokio::test]
async fn test_sequence_operations() {
    // GC content
    assert!((calculate_gc_content("ATCG") - 0.5).abs() < 0.001);
    assert!((calculate_gc_content("GCGC") - 1.0).abs() < 0.001);
    assert!((calculate_gc_content("ATAT") - 0.0).abs() < 0.001);

    // DNA validation
    assert!(validate_dna_sequence("ATCGATCGATCG").is_ok());
    assert!(validate_dna_sequence("ATCGN").is_err()); // N not allowed
    assert!(validate_dna_sequence("ATCGU").is_err()); // U not allowed in DNA

    // RNA validation
    assert!(validate_rna_sequence("AUCGAUCG").is_ok());
    assert!(validate_rna_sequence("ATCG").is_err()); // T not allowed in RNA

    // Reverse complement
    assert_eq!(reverse_complement("ATCG"), "CGAT");
    assert_eq!(reverse_complement("AAAA"), "TTTT");
    assert_eq!(reverse_complement("GCGC"), "GCGC");

    // Transcription
    assert_eq!(transcribe_dna_to_rna("ATCGATCG"), "AUCGAUCG");
}

#[tokio::test]
async fn test_plddt_interpretation() {
    // Very high confidence
    let interpretation = interpret_plddt(95.0);
    assert!(interpretation.contains("Very high"));

    // High confidence
    let interpretation = interpret_plddt(80.0);
    assert!(interpretation.contains("High"));

    // Low confidence
    let interpretation = interpret_plddt(60.0);
    assert!(interpretation.contains("Low"));

    // Very low confidence
    let interpretation = interpret_plddt(30.0);
    assert!(interpretation.contains("Very low"));
}

#[tokio::test]
async fn test_confidence_region_identification() {
    let scores = vec![
        90.0, 92.0, 88.0, 85.0, // High confidence region
        45.0, 42.0, 48.0,       // Low confidence region
        75.0, 78.0, 80.0,       // Medium-high confidence
    ];

    let interpretation = identify_confidence_regions(&scores, 70.0, 50.0);

    // Should have high confidence regions
    assert!(interpretation.high_confidence_regions.is_some());
    let high_regions = interpretation.high_confidence_regions.unwrap();
    assert!(!high_regions.is_empty());

    // Should have low confidence regions
    assert!(interpretation.low_confidence_regions.is_some());
    let low_regions = interpretation.low_confidence_regions.unwrap();
    assert!(!low_regions.is_empty());

    // Verify low region is marked as likely disordered
    assert_eq!(low_regions[0].likely_disordered, Some(true));
}

#[tokio::test]
async fn test_editing_efficiency_calculation() {
    // Knockout (NHEJ only)
    let efficiency = calculate_editing_efficiency(0.75, None);
    assert!((efficiency.overall_efficiency.unwrap() - 0.75).abs() < 0.001);
    assert!((efficiency.wild_type_frequency.unwrap() - 0.25).abs() < 0.001);

    // Knock-in (NHEJ + HDR)
    let efficiency = calculate_editing_efficiency(0.50, Some(0.20));
    assert!((efficiency.overall_efficiency.unwrap() - 0.70).abs() < 0.001);
    assert!((efficiency.wild_type_frequency.unwrap() - 0.30).abs() < 0.001);
}

#[tokio::test]
async fn test_project_creation() {
    let project = create_project("CRISPR Therapy Development", ProjectCategory::GeneTherapy);

    assert!(project.project_id.starts_with("proj-"));
    assert_eq!(project.project_info.name, "CRISPR Therapy Development");
    assert_eq!(project.project_info.category, Some(ProjectCategory::GeneTherapy));
    assert_eq!(project.project_info.status, Some(ProjectStatus::Active));
    assert!(project.created_at.is_some());
}

#[tokio::test]
async fn test_json_serialization() {
    // Sequence
    let sequence = create_sequence("Test Gene", "ATCGATCG", SequenceType::Dna).unwrap();
    let json = serde_json::to_string_pretty(&sequence).unwrap();
    assert!(json.contains("sequence_id"));
    assert!(json.contains("dna"));

    // Parse back
    let parsed: Sequence = serde_json::from_str(&json).unwrap();
    assert_eq!(parsed.sequence_id, sequence.sequence_id);

    // CRISPR Experiment
    let grna = GuideRna {
        grna_id: "grna-001".to_string(),
        name: None,
        sequence: "AUCGAUCGAUCGAUCGAUCG".to_string(),
        target_start: None,
        target_end: None,
        strand: None,
        pam: None,
        off_target_score: None,
        on_target_score: None,
    };
    let experiment = create_crispr_experiment(
        "Test",
        "ABC1",
        CrisprSystemType::CrisprCas9,
        vec![grna],
    )
    .unwrap();

    let json = serde_json::to_string_pretty(&experiment).unwrap();
    assert!(json.contains("crispr_cas9"));
    assert!(json.contains("ABC1"));
}

#[tokio::test]
async fn test_adapter_error_handling() {
    let adapter = SimulatorAdapter::new();

    // Try to get non-existent sequence
    let result = adapter.get_sequence("non-existent-id").await;
    assert!(result.is_err());

    // Try to get non-existent experiment
    let result = adapter.get_experiment("non-existent-id").await;
    assert!(result.is_err());

    // Try to simulate results for non-existent experiment
    let result = adapter.simulate_editing_results("non-existent-id");
    assert!(result.is_err());
}
