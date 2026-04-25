//! WIA Refugee Credential API Server
//!
//! Dignified credential verification for displaced persons.
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
                .unwrap_or_else(|_| "wia_refugee_credential=info,tower_http=debug".into()),
        )
        .init();

    // Configuration from environment
    let host = std::env::var("WIA_HOST").unwrap_or_else(|_| "0.0.0.0".to_string());
    let port: u16 = std::env::var("WIA_PORT")
        .ok()
        .and_then(|p| p.parse().ok())
        .unwrap_or(8080);

    println!(
        r#"
╔══════════════════════════════════════════════════════════════════╗
║     WIA Refugee Credential API Server                            ║
║     Dignified Verification for Displaced Persons                 ║
║     홍익인간 (弘益人間) - Benefit All Humanity                   ║
╠══════════════════════════════════════════════════════════════════╣
║  Version: {}                                                ║
║  Standard: WIA-REFUGEE-CREDENTIAL                                ║
║  Philosophy: 국가가 무너져도, 사람의 가치는 무너지지 않습니다   ║
╠══════════════════════════════════════════════════════════════════╣
║  Verification Levels:                                            ║
║    Level 1: Self-Declaration (30% confidence)                    ║
║    Level 2: Peer Verified (60% confidence)                       ║
║    Level 3: Assessment Verified (80% confidence)                 ║
║    Level 4: Document Verified (95% confidence)                   ║
╠══════════════════════════════════════════════════════════════════╣
║  Supported Languages:                                            ║
║    ar (العربية), uk (Українська), fa (فارسی), ps (پښتو),        ║
║    ti (ትግርኛ), fr, en, es, de, tr, ko                            ║
╚══════════════════════════════════════════════════════════════════╝
"#,
        wia_refugee_credential::VERSION
    );

    // Start server
    wia_refugee_credential::server::start_server(&host, port).await
}
