# WIA-LEGAL-007-digital-forensics Rust SDK

> 弘益人間 (홍익인간) - Benefit All Humanity

Official Rust SDK for WIA-LEGAL-007-digital-forensics - Digital Forensics

## Installation

```toml
[dependencies]
wia-legal-007-digital-forensics = "1.0.0"
tokio = { version = "1.0", features = ["full"] }
```

## Quick Start

```rust
use wia_legal_007_digital_forensics::Client;

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
