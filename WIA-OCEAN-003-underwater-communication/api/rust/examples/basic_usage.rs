//! Basic usage example for WIA Ocean Underwater Communication SDK
//!
//! 弘益人間 (Benefit All Humanity)

use wia_ocean_003_underwater_communication::*;
use chrono::Utc;

#[tokio::main]
async fn main() -> Result<()> {
    println!("WIA Ocean Underwater Communication SDK v{}", VERSION);
    println!("弘益人間 - Benefit All Humanity\n");

    let message = AcousticMessage {
        id: utils::generate_message_id(),
        sender_id: "AUV-001".to_string(),
        receiver_id: "BASE-STATION".to_string(),
        payload: vec![1, 2, 3, 4, 5],
        frequency_khz: 25.0,
        transmission_power_db: 180.0,
        timestamp: Utc::now(),
    };

    validators::validate_message(&message)?;
    println!("✓ Message validated");

    let delay = utils::calculate_propagation_delay(1000.0);
    println!("  Propagation delay for 1km: {:.3} seconds", delay);

    Ok(())
}
