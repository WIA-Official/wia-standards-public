# WIA-LANG-002-indigenous-script Rust SDK

> 弘益人間 (홍익인간) - Benefit All Humanity

Official Rust SDK for WIA-LANG-002-indigenous-script - Indigenous Script Digitization

## Installation

```toml
[dependencies]
wia-lang-002-indigenous-script = "1.0.0"
tokio = { version = "1.0", features = ["full"] }
```

## Quick Start

```rust
use wia_lang_002_indigenous_script::Client;

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
