# WIA Interstellar Message - Rust SDK

弘益人間 (홍익인간) - Benefit All Humanity

## Overview

SDK for Interstellar Message standard.

## Installation

```toml
[dependencies]
wia-interstellar-message = "1.0.0"
```

## Quick Start

```rust
use wia_interstellar_message::Client;

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
