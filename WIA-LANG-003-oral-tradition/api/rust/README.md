# WIA-LANG-003-oral-tradition Rust SDK

> 弘益人間 (홍익인간) - Benefit All Humanity

Official Rust SDK for WIA-LANG-003-oral-tradition - Oral Tradition Documentation

## Installation

```toml
[dependencies]
wia-lang-003-oral-tradition = "1.0.0"
tokio = { version = "1.0", features = ["full"] }
```

## Quick Start

```rust
use wia_lang_003_oral_tradition::Client;

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
