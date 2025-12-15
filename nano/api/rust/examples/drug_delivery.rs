//! Example: Targeted Drug Delivery
//!
//! This example demonstrates a nanomedicine drug delivery system
//! with targeted release at tumor sites.

use wia_nano::prelude::*;
use wia_nano::systems::StandardDrugDelivery;

#[tokio::main]
async fn main() -> NanoResult<()> {
    println!("=== WIA Nano SDK: Drug Delivery Example ===\n");

    // Create a liposomal drug carrier
    let mut carrier = StandardDrugDelivery::builder("lipo-001", CarrierType::Liposome)
        .with_size(100.0) // 100 nm diameter
        .with_environment(Environment::physiological())
        .with_targeting(TargetingConfig {
            targeting_type: TargetingType::LigandReceptor,
            target_markers: vec![
                "EGFR".to_string(),    // Epidermal growth factor receptor
                "HER2".to_string(),    // Human epidermal growth factor receptor 2
            ],
            target_tissue: Some("breast_tumor".to_string()),
            binding_affinity: Some(0.5), // Kd = 0.5 nM
            specificity: 0.95,
        })
        .with_payload(Payload {
            drug_name: "Doxorubicin".to_string(),
            drug_class: DrugClass::SmallMolecule,
            amount_ng: 50.0,     // 50 ng per carrier
            loading_efficiency: 0.85,
            released_amount_ng: 0.0,
        })
        .build();

    // Add pH-sensitive release trigger
    carrier.add_trigger(ReleaseTrigger {
        trigger_type: TriggerType::Ph,
        threshold: 6.5, // Acidic tumor microenvironment
        unit: "pH".to_string(),
        is_active: true,
    });

    // Initialize
    carrier.initialize().await?;

    println!("✓ Drug carrier initialized");
    println!("  Carrier: {:?}", carrier.carrier_type());
    println!("  Size: {} nm", carrier.size_nm());
    println!("\nPayload:");
    let payload = carrier.payload();
    println!("  Drug: {}", payload.drug_name);
    println!("  Amount: {} ng", payload.amount_ng);
    println!("  Loading efficiency: {:.0}%", payload.loading_efficiency * 100.0);

    println!("\nTargeting:");
    let targeting = carrier.targeting();
    println!("  Type: {:?}", targeting.targeting_type);
    println!("  Markers: {:?}", targeting.target_markers);
    println!("  Specificity: {:.0}%", targeting.specificity * 100.0);

    println!("\nRelease Triggers:");
    for trigger in carrier.release_triggers() {
        println!("  {:?}: {} {} (active: {})",
                 trigger.trigger_type,
                 trigger.threshold,
                 trigger.unit,
                 trigger.is_active);
    }

    // Simulate circulation
    println!("\n--- Simulating Circulation ---\n");

    // Normal blood pH
    println!("Phase 1: Normal circulation (pH 7.4)");
    carrier.set_environment(Environment::physiological());
    let status = carrier.release_status();
    println!("  Release state: {:?}", status.state);
    println!("  Released: {:.1}%", status.cumulative_released);

    // Track position
    for i in 1..=3 {
        let tracking = carrier.update_tracking().await?;
        println!("  Position update {}: {:?} (confidence: {:.0}%)",
                 i,
                 tracking.position.map(|p| format!("({:.0}, {:.0}, {:.0})",
                                                    p.x, p.y, p.z)),
                 tracking.confidence * 100.0);
    }

    // Reaching tumor microenvironment
    println!("\nPhase 2: Reaching tumor site (pH 6.2)");
    carrier.set_environment(
        Environment::physiological()
            .with_ph(6.2) // Acidic tumor environment
    );

    let status = carrier.release_status();
    println!("  Release triggered! State: {:?}", status.state);

    // Trigger release
    println!("\nPhase 3: Drug Release");
    let release_result = carrier.trigger_release().await?;
    println!("  ✓ Release successful");
    println!("    Trigger: {:?}", release_result.trigger_used);
    println!("    Amount released: {:.2} ng", release_result.amount_released_ng);
    println!("    Duration: {} ms", release_result.duration_ms);

    // Final status
    let final_status = carrier.release_status();
    println!("\nFinal Release Status:");
    println!("  State: {:?}", final_status.state);
    println!("  Cumulative released: {:.1}%", final_status.cumulative_released);

    let remaining = carrier.payload().remaining_amount();
    println!("  Remaining payload: {:.2} ng", remaining);

    // Safety monitoring
    println!("\n--- Safety Assessment ---");
    let biocompat = carrier.biocompatibility();
    println!("Biocompatibility:");
    println!("  Compatible: {}", biocompat.is_biocompatible);
    println!("  Cytotoxicity: {:.1}%", biocompat.cytotoxicity_level * 100.0);

    let toxicity = carrier.toxicity_assessment();
    println!("\nToxicity:");
    println!("  Acute: {:.1}%", toxicity.acute_toxicity * 100.0);
    println!("  Genotoxic: {}", toxicity.genotoxicity);

    let immune = carrier.immune_response();
    println!("\nImmune Response:");
    println!("  Immunogenic: {}", immune.immunogenic);
    println!("  Inflammation: {:.1}%", immune.inflammation_level * 100.0);

    // Pharmacokinetics
    let pk = carrier.pharmacokinetics();
    println!("\nPharmacokinetics:");
    println!("  Half-life: {:.1} hours", pk.half_life_hours);
    println!("  Bioavailability: {:.0}%", pk.bioavailability * 100.0);
    println!("  Peak concentration time: {:.1} hours",
             pk.peak_concentration_time_hours.unwrap_or(0.0));

    // Final message
    let message = carrier.to_message()?;
    println!("\nCarrier Status Message:");
    println!("{}", serde_json::to_string_pretty(&message)?);

    println!("\n✓ Drug delivery simulation complete!");

    Ok(())
}
