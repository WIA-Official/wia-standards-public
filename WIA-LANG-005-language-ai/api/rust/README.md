# WIA-LANG-005-language-ai Rust SDK

> 弘益人間 (홍익인간) - Benefit All Humanity

Official Rust SDK for WIA-LANG-005-language-ai - Language AI Technology

## Installation

```toml
[dependencies]
wia-lang-005-language-ai = "1.0.0"
tokio = { version = "1.0", features = ["full"] }
```

## Quick Start

```rust
use wia_lang_005_language_ai::Client;

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
