# WIA Non-Human Intelligence - Rust SDK

弘益人間 (홍익인간) - Benefit All Humanity

## Overview

SDK for Non-Human Intelligence standard.

## Installation

```toml
[dependencies]
wia-non-human-intelligence = "1.0.0"
```

## Quick Start

```rust
use wia_non_human_intelligence::Client;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = Client::new("https://api.wia.global", "your-api-key")?;
    Ok(())
}
```

## Philosophy

弘益人間 - Technology that benefits all of humanity.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
