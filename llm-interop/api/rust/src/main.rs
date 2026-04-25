//! WIA-LLM-INTEROP API Server
//!
//! AI들의 연동 표준 - 언어, 국경, 회사의 벽을 넘어 모든 AI가 협업합니다.
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
                .unwrap_or_else(|_| "wia_llm_interop=info,tower_http=debug".into()),
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
║            WIA-LLM-INTEROP API Server                            ║
║     AI들의 연동 표준 - All AIs Collaborate as One                ║
║     홍익인간 (弘益人間) - Benefit All Humanity                   ║
╠══════════════════════════════════════════════════════════════════╣
║  Version: {}                                                ║
║  WIA Version: {}                                                 ║
║  Standard: WIA-LLM-INTEROP                                       ║
╠══════════════════════════════════════════════════════════════════╣
║  Components:                                                     ║
║    📋 WIA-LLM-CAPABILITY  - AI 능력 선언                         ║
║    📨 WIA-LLM-MESSAGE     - AI간 메시지 형식                     ║
║    🤝 WIA-LLM-FEDERATION  - AI 연합 프로토콜                     ║
╠══════════════════════════════════════════════════════════════════╣
║  Capability Levels:                                              ║
║    Level 1: Basic (Simple Q&A)                                   ║
║    Level 2: Reasoning (Multi-step analysis)                      ║
║    Level 3: Specialist (Domain expert)                           ║
║    Level 4: Advanced (Tool use, multi-modal, agentic)            ║
╠══════════════════════════════════════════════════════════════════╣
║  Endpoints:                                                      ║
║    REST:      /wia/llm-interop/v1/                               ║
║    WebSocket: /wia/llm-interop/v1/ws                             ║
║    Discovery: /.well-known/wia-llm-interop                       ║
╚══════════════════════════════════════════════════════════════════╝
"#,
        wia_llm_interop::VERSION,
        wia_llm_interop::WIA_VERSION
    );

    // Start server
    wia_llm_interop::server::start_server(&host, port).await
}
