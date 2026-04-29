# WIA Child Digital Rights - Rust SDK

弘益人間 (홍익인간) - Benefit All Humanity

## Overview

SDK for Child Digital Rights standard.

## Installation

```toml
[dependencies]
wia-child-digital-rights = "1.0.0"
```

## Quick Start

```rust
use wia_child_digital_rights::Client;

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
