//! CRISPR Experiment Example for WIA Biotech Rust API
//!
//! This example demonstrates a complete CRISPR gene editing workflow
//! using the WIA Biotech library.

use wia_bio::prelude::*;
use wia_bio::adapters::SimulatorAdapter;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Biotech - CRISPR Experiment Workflow ===\n");

    // ========================================
    // Step 1: Initialize the data provider
    // ========================================
    println!("Step 1: Initializing simulator adapter...");
    let adapter = SimulatorAdapter::new();
    println!("  Adapter ready\n");

    // ========================================
    // Step 2: Define the target gene sequence
    // ========================================
    println!("Step 2: Creating target gene sequence...");

    // This is a simplified example sequence with PAM sites
    let target_gene_sequence = concat!(
        "ATGCGATCGATCGATCGATCGATCGATCGATCGATCG",  // Start codon region
        "NGGCGATCGATCGATCGATCGATCGATCGATCGATCG",  // PAM site 1
        "CGATCGATCGATCGATCGATCGATCGATCGATCGATCG",
        "NGGCGATCGATCGATCGATCGATCGATCGATCGATCG",  // PAM site 2
        "CGATCGATCGATCGATCGATCGATCGATCGATCGATCG",
    );

    // Create and store the sequence
    let sequence = create_sequence(
        "BRCA1 Exon 10",
        target_gene_sequence,
        SequenceType::Dna,
    )?;

    let seq_id = adapter.store_sequence(sequence.clone()).await?;
    println!("  Sequence ID: {}", seq_id);
    println!("  Length: {} bp", sequence.length_bp);
    println!("  GC Content: {:.1}%\n", sequence.gc_content.unwrap() * 100.0);

    // ========================================
    // Step 3: Find PAM sites for SpCas9
    // ========================================
    println!("Step 3: Scanning for PAM sites (NGG)...");

    let pam_sites = find_pam_sites_ngg(target_gene_sequence);
    println!("  Found {} potential PAM sites:", pam_sites.len());

    for (i, (position, strand)) in pam_sites.iter().enumerate().take(5) {
        let strand_str = match strand {
            Strand::Plus => "forward (+)",
            Strand::Minus => "reverse (-)",
        };
        println!("    Site {}: position {}, {}", i + 1, position, strand_str);
    }
    if pam_sites.len() > 5 {
        println!("    ... and {} more sites", pam_sites.len() - 5);
    }
    println!();

    // ========================================
    // Step 4: Design guide RNAs
    // ========================================
    println!("Step 4: Designing guide RNAs...");

    let guide_rnas = vec![
        GuideRna {
            grna_id: "grna-brca1-001".to_string(),
            name: Some("BRCA1-gRNA-1".to_string()),
            sequence: "AUCGAUCGAUCGAUCGAUCG".to_string(),
            target_start: Some(38),
            target_end: Some(58),
            strand: Some(Strand::Plus),
            pam: Some("NGG".to_string()),
            off_target_score: Some(0.92),
            on_target_score: Some(0.87),
        },
        GuideRna {
            grna_id: "grna-brca1-002".to_string(),
            name: Some("BRCA1-gRNA-2".to_string()),
            sequence: "CGAUCGAUCGAUCGAUCGAU".to_string(),
            target_start: Some(118),
            target_end: Some(138),
            strand: Some(Strand::Plus),
            pam: Some("NGG".to_string()),
            off_target_score: Some(0.95),
            on_target_score: Some(0.82),
        },
    ];

    for grna in &guide_rnas {
        println!("  {} ({})", grna.grna_id, grna.name.as_ref().unwrap());
        println!("    Sequence: {}", grna.sequence);
        println!("    Position: {}-{}",
            grna.target_start.unwrap(),
            grna.target_end.unwrap()
        );
        println!("    On-target score: {:.2}", grna.on_target_score.unwrap());
        println!("    Off-target score: {:.2}", grna.off_target_score.unwrap());
    }
    println!();

    // ========================================
    // Step 5: Create CRISPR experiment
    // ========================================
    println!("Step 5: Creating CRISPR experiment...");

    let mut experiment = create_crispr_experiment(
        "BRCA1 Knockout Study - HEK293T",
        "BRCA1",
        CrisprSystemType::CrisprCas9,
        guide_rnas,
    )?;

    // Add additional experiment details
    experiment.target.organism = Some("Homo sapiens".to_string());
    experiment.target.cell_line = Some("HEK293T".to_string());
    experiment.target.sequence_ref = Some(seq_id.clone());
    experiment.editing_system.cas_variant = Some("SpCas9".to_string());
    experiment.editing_system.delivery_method = Some(DeliveryMethod::Lipofection);
    experiment.editing_type = Some(EditingType::Knockout);
    experiment.protocol = Some(CrisprProtocol {
        cas9_concentration_nm: Some(100.0),
        grna_concentration_nm: Some(200.0),
        transfection_reagent: Some("Lipofectamine 3000".to_string()),
        incubation_hours: Some(48.0),
        selection_method: Some("Puromycin".to_string()),
    });

    // Store the experiment
    let exp_id = adapter.store_experiment(experiment.clone()).await?;
    println!("  Experiment ID: {}", exp_id);
    println!("  Target gene: {}", experiment.target.gene_symbol);
    println!("  Cell line: {}", experiment.target.cell_line.as_ref().unwrap());
    println!("  CRISPR system: {:?}", experiment.editing_system.system_type);
    println!("  Delivery: {:?}", experiment.editing_system.delivery_method.as_ref().unwrap());
    println!("  Guide RNAs: {}", experiment.guide_rnas.len());
    println!();

    // ========================================
    // Step 6: Simulate editing results
    // ========================================
    println!("Step 6: Simulating editing results...");

    let results = adapter.simulate_editing_results(&exp_id)?;

    println!("  Results ID: {}", results.results_id);
    println!("  Analysis method: {:?}", results.analysis_method.as_ref().unwrap());

    if let Some(efficiency) = &results.editing_efficiency {
        println!("\n  Editing Efficiency:");
        println!("    Overall: {:.1}%", efficiency.overall_efficiency.unwrap() * 100.0);
        println!("    Indel frequency: {:.1}%", efficiency.indel_frequency.unwrap() * 100.0);
        println!("    Wild-type: {:.1}%", efficiency.wild_type_frequency.unwrap() * 100.0);
    }

    if let Some(indels) = &results.indel_profile {
        println!("\n  Indel Profile:");
        for indel in indels {
            let size_str = if indel.size_bp > 0 {
                format!("+{}", indel.size_bp)
            } else {
                indel.size_bp.to_string()
            };
            println!(
                "    {:?}: {} bp ({:.1}%)",
                indel.indel_type,
                size_str,
                indel.frequency * 100.0
            );
        }
    }

    if let Some(off_target) = &results.off_target_analysis {
        println!("\n  Off-target Analysis:");
        println!("    Method: {:?}", off_target.method.as_ref().unwrap());
        println!("    Sites detected: {}", off_target.sites_detected);

        if let Some(sites) = &off_target.sites {
            for site in sites.iter().take(3) {
                println!(
                    "      {}:{} - {} mismatches, {:.3}% frequency",
                    site.chromosome.as_ref().unwrap_or(&"?".to_string()),
                    site.position.unwrap_or(0),
                    site.mismatches.unwrap_or(0),
                    site.frequency.unwrap_or(0.0) * 100.0
                );
            }
        }
    }

    if let Some(quality) = &results.quality_metrics {
        println!("\n  Quality Metrics:");
        println!("    Sequencing depth: {}x", quality.sequencing_depth.unwrap());
        println!("    Mapping rate: {:.1}%", quality.mapping_rate.unwrap() * 100.0);
        println!("    Confidence: {:.1}%", quality.confidence_score.unwrap() * 100.0);
    }

    // ========================================
    // Step 7: Export to JSON
    // ========================================
    println!("\n\nStep 7: Exporting experiment to JSON...");

    let json = serde_json::to_string_pretty(&experiment)?;
    let json_lines: Vec<&str> = json.lines().collect();

    println!("  JSON output ({} lines):", json_lines.len());
    println!("  ---");
    for line in json_lines.iter().take(20) {
        println!("  {}", line);
    }
    if json_lines.len() > 20 {
        println!("  ... ({} more lines)", json_lines.len() - 20);
    }
    println!("  ---");

    // ========================================
    // Summary
    // ========================================
    println!("\n=== Workflow Complete ===");
    println!("Created:");
    println!("  - 1 target sequence ({})", seq_id);
    println!("  - 1 CRISPR experiment ({})", exp_id);
    println!("  - 2 guide RNAs");
    println!("  - Simulated editing results with {:.1}% efficiency",
        results.editing_efficiency.unwrap().overall_efficiency.unwrap() * 100.0
    );

    Ok(())
}
