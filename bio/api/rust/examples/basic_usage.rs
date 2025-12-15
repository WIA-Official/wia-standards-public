//! Basic Usage Example for WIA Biotech Rust API
//!
//! This example demonstrates the fundamental operations available
//! in the WIA Biotech library.

use wia_bio::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Biotech Rust API - Basic Usage ===\n");

    // ========================================
    // 1. Sequence Operations
    // ========================================
    println!("1. Sequence Operations");
    println!("-----------------------");

    // Create a DNA sequence
    let dna_sequence = "ATCGATCGATCGATCGATCG";
    println!("DNA Sequence: {}", dna_sequence);

    // Validate the sequence
    match validate_dna_sequence(dna_sequence) {
        Ok(()) => println!("  Validation: Valid DNA sequence"),
        Err(e) => println!("  Validation Error: {}", e),
    }

    // Calculate GC content
    let gc_content = calculate_gc_content(dna_sequence);
    println!("  GC Content: {:.1}%", gc_content * 100.0);

    // Get reverse complement
    let rev_comp = reverse_complement(dna_sequence);
    println!("  Reverse Complement: {}", rev_comp);

    // Transcribe to RNA
    let rna = transcribe_dna_to_rna(dna_sequence);
    println!("  Transcribed RNA: {}", rna);

    // Create a formal Sequence object
    let sequence = create_sequence("Sample Gene", dna_sequence, SequenceType::Dna)?;
    println!("  Created Sequence ID: {}", sequence.sequence_id);
    println!("  Length: {} bp\n", sequence.length_bp);

    // ========================================
    // 2. CRISPR Operations
    // ========================================
    println!("2. CRISPR Operations");
    println!("--------------------");

    // Target sequence with PAM sites
    let target = "ATCGATCGATCGATCGNGGCGATCGATCGATCGNGGCGATCG";
    println!("Target Sequence: {}", target);

    // Find PAM sites (NGG for SpCas9)
    let pam_sites = find_pam_sites_ngg(target);
    println!("  Found {} PAM sites:", pam_sites.len());
    for (pos, strand) in &pam_sites {
        let strand_str = match strand {
            Strand::Plus => "+",
            Strand::Minus => "-",
        };
        println!("    Position: {}, Strand: {}", pos, strand_str);
    }

    // Create a guide RNA
    let grna = GuideRna {
        grna_id: "grna-001".to_string(),
        name: Some("Target gRNA".to_string()),
        sequence: "AUCGAUCGAUCGAUCGAUCG".to_string(),
        target_start: Some(1),
        target_end: Some(20),
        strand: Some(Strand::Plus),
        pam: Some("NGG".to_string()),
        off_target_score: Some(0.95),
        on_target_score: Some(0.88),
    };

    // Create CRISPR experiment
    let experiment = create_crispr_experiment(
        "Gene Knockout Study",
        "ABC1",
        CrisprSystemType::CrisprCas9,
        vec![grna],
    )?;
    println!("\n  Created Experiment ID: {}", experiment.experiment_id);
    println!("  Target Gene: {}", experiment.target.gene_symbol);
    println!("  System: {:?}", experiment.editing_system.system_type);

    // Calculate editing efficiency
    let efficiency = calculate_editing_efficiency(0.75, Some(0.10));
    println!("  Editing Efficiency:");
    println!(
        "    Overall: {:.1}%",
        efficiency.overall_efficiency.unwrap() * 100.0
    );
    println!(
        "    Indel: {:.1}%",
        efficiency.indel_frequency.unwrap() * 100.0
    );
    println!(
        "    HDR: {:.1}%",
        efficiency.hdr_frequency.unwrap() * 100.0
    );
    println!(
        "    Wild-type: {:.1}%\n",
        efficiency.wild_type_frequency.unwrap() * 100.0
    );

    // ========================================
    // 3. Protein Structure Operations
    // ========================================
    println!("3. Protein Structure Operations");
    println!("-------------------------------");

    // Create a protein structure record
    let structure = create_protein_structure(
        "Human p53",
        Some("P04637"),
        PredictionMethod::Alphafold,
        393,
    );
    println!("Created Structure ID: {}", structure.structure_id);
    println!("  Protein: {}", structure.protein.name);
    println!("  Length: {} aa", structure.protein.length_aa.unwrap());
    println!("  Method: {:?}", structure.prediction.method);

    // pLDDT interpretation
    let sample_scores = [95.0, 85.0, 60.0, 35.0];
    println!("\n  pLDDT Interpretation:");
    for score in sample_scores {
        let interpretation = interpret_plddt(score);
        println!("    Score {:.0}: {}", score, interpretation);
    }

    // Calculate mean pLDDT
    let plddt_values = vec![90.0, 88.0, 92.0, 85.0, 87.0];
    let mean = calculate_mean_plddt(&plddt_values);
    println!("\n  Mean pLDDT: {:.1}\n", mean);

    // ========================================
    // 4. Synthetic Biology Operations
    // ========================================
    println!("4. Synthetic Biology Operations");
    println!("-------------------------------");

    // Create a BioBrick part
    let promoter = create_biopart(
        "J23100 Constitutive Promoter",
        PartType::Promoter,
        "TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC",
    )?;
    println!("Created Part ID: {}", promoter.part_id);
    println!("  Name: {}", promoter.part_info.name.unwrap());
    println!("  Type: {:?}", promoter.part_info.part_type);
    println!(
        "  Length: {} bp",
        promoter.sequence.length_bp.unwrap()
    );

    // Add BioBrick scars
    let scarred = add_biobrick_scars("ATCGATCG");
    println!("\n  BioBrick Scarred Sequence:");
    println!("    Original: ATCGATCG");
    println!("    With scars: {} bp", scarred.len());

    // Create an assembly design
    let assembly = create_assembly(
        "GFP Expression Cassette",
        AssemblyStandard::Biobrick,
        vec![
            ("part-prom".to_string(), PartType::Promoter),
            ("part-rbs".to_string(), PartType::Rbs),
            ("part-gfp".to_string(), PartType::Cds),
            ("part-term".to_string(), PartType::Terminator),
        ],
    );
    println!("\n  Created Assembly ID: {}", assembly.assembly_id);
    println!("  Parts in assembly: {}", assembly.parts.len());
    for part in &assembly.parts {
        println!(
            "    Position {}: {:?}",
            part.position,
            part.part_type.as_ref().unwrap()
        );
    }

    // ========================================
    // 5. Project Management
    // ========================================
    println!("\n5. Project Management");
    println!("---------------------");

    let project = create_project("CRISPR Gene Therapy Development", ProjectCategory::GeneTherapy);
    println!("Created Project ID: {}", project.project_id);
    println!("  Name: {}", project.project_info.name);
    println!("  Category: {:?}", project.project_info.category.unwrap());
    println!("  Status: {:?}", project.project_info.status.unwrap());
    println!("  WIA Version: {}", project.wia_version);

    // ========================================
    // 6. JSON Serialization
    // ========================================
    println!("\n6. JSON Serialization");
    println!("---------------------");

    let json = serde_json::to_string_pretty(&sequence)?;
    println!("Sequence JSON (truncated):");
    let lines: Vec<&str> = json.lines().take(10).collect();
    for line in lines {
        println!("  {}", line);
    }
    println!("  ...\n");

    println!("=== Demo Complete ===");
    Ok(())
}
