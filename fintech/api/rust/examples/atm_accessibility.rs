//! ATM Accessibility Example for WIA Fintech API
//!
//! Run with: cargo run --example atm_accessibility

use wia_fintech::prelude::*;
use wia_fintech::{
    AccessibilityScoreCalculator, CompatibilityChecker,
    SimulatorAdapter,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA Fintech - ATM Accessibility Example ===\n");

    let adapter = SimulatorAdapter::with_sample_data().await;
    let calculator = AccessibilityScoreCalculator::new();
    let checker = CompatibilityChecker::new();

    // Get blind user profile
    println!("Loading blind user profile...");
    let blind_user = adapter.get_profile("user_blind_001").await?;
    println!("User: {}", blind_user.profile_id);
    if let Some(visual) = &blind_user.accessibility_needs.sensory.visual {
        println!("Visual Level: {:?}", visual.level);
    }
    println!();

    // List all ATMs and their scores
    println!("ATM Accessibility Scores:");
    println!("{}", "-".repeat(60));

    let atms = adapter.list_atms().await?;
    for atm in &atms {
        let score = calculator.calculate_atm_score(atm);
        let compat = checker.check_compatibility(&blind_user, atm);

        let status = if compat.is_compatible { "✓" } else { "✗" };

        println!(
            "{} {} - Score: {:.1}, Compatible: {} ({:.0}%)",
            status,
            atm.atm_id,
            score.overall,
            compat.is_compatible,
            compat.compatibility_score
        );

        if !compat.issues.is_empty() {
            for issue in &compat.issues {
                println!("    - {}: {}", issue.category, issue.description);
            }
        }
    }

    println!();
    println!("=== Example Complete ===");
    Ok(())
}
