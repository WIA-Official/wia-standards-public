# WIA-LEGAL-006-ip-digital Rust SDK

> 弘益人間 (홍익인간) - Benefit All Humanity

Official Rust SDK for WIA-LEGAL-006-ip-digital - Digital Intellectual Property

## Installation

```toml
[dependencies]
wia-legal-006-ip-digital = "1.0.0"
tokio = { version = "1.0", features = ["full"] }
```

## Quick Start

```rust
use wia_legal_006_ip_digital::Client;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = Client::new("https://api.wia.org", "your-api-key")?;
    let resources = client.list_resources().await?;
    println!("Found {} resources", resources.len());
    Ok(())
}
```

## License

MIT License - © 2025 SmileStory Inc. / WIA
