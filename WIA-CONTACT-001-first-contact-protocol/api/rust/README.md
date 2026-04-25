# WIA First Contact Protocol - Rust SDK

弘益人間 (홍익인간) - Benefit All Humanity

## Overview

SDK for First Contact Protocol standard.

## Installation

```toml
[dependencies]
wia-first-contact = "1.0.0"
```

## Quick Start

```rust
use wia_first_contact::Client;

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
