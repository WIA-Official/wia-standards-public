//! WIA Battery Passport API Server
//!
//! EU Battery Regulation 2027 compliant battery lifecycle tracking.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity
//!
//! # Usage
//! ```bash
//! # Run with defaults (0.0.0.0:8080)
//! cargo run
//!
//! # Custom host and port
//! WIA_HOST=127.0.0.1 WIA_PORT=3000 cargo run
//! ```

use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Initialize logging
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer())
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "wia_battery_passport=info,tower_http=debug".into()),
        )
        .init();

    // Configuration from environment
    let host = std::env::var("WIA_HOST").unwrap_or_else(|_| "0.0.0.0".to_string());
    let port: u16 = std::env::var("WIA_PORT")
        .ok()
        .and_then(|p| p.parse().ok())
        .unwrap_or(8080);

    println!(r#"
╔══════════════════════════════════════════════════════════════════╗
║     WIA Battery Passport API Server                              ║
║     EU Battery Regulation 2027 Compliant                         ║
║     홍익인간 (弘益人間) - Benefit All Humanity                   ║
╠══════════════════════════════════════════════════════════════════╣
║  Version: {}                                                ║
║  Standard: WIA-BATTERY-PASSPORT                                  ║
║  EU Regulation: {}                                       ║
╚══════════════════════════════════════════════════════════════════╝
"#, wia_battery_passport::VERSION, wia_battery_passport::EU_REGULATION);

    // Start server
    wia_battery_passport::server::start_server(&host, port).await
}
