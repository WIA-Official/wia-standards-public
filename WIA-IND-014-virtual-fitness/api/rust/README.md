# WIA-IND-014-virtual-fitness Rust SDK

> 弘益人間 (홍익인간) - Benefit All Humanity

Official Rust SDK for WIA-IND-014-virtual-fitness - Virtual Fitness Platform

## Features

- Type-safe API client
- Async/await support
- Comprehensive error handling
- Full test coverage

## Installation

```toml
[dependencies]
wia-ind-014-virtual-fitness = "1.0.0"
tokio = { version = "1.0", features = ["full"] }
```

## Quick Start

```rust
use wia_ind_014_virtual_fitness::Client;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = Client::new("https://api.wia.org", "your-api-key")?;
    let resources = client.list_resources().await?;
    println!("Found {} resources", resources.len());
    Ok(())
}
```

## Philosophy

弘益人間 (홍익인간) - Benefit All Humanity

## License

MIT License

---

**© 2025 SmileStory Inc. / WIA**
