# WIA Parental Control - Rust SDK

弘益人間 (홍익인간) - Protecting all children

## Overview

SDK for parental control systems, content filtering, and screen time management.

## Installation

```toml
[dependencies]
wia-child-parental-control = "1.0.0"
```

## Quick Start

```rust
use wia_child_parental_control::{Client, ParentalControl};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = Client::new("https://api.wia.global", "your-api-key")?;
    let controls = client.get_controls(child_id).await?;
    println!("Screen time limit: {} minutes", controls.screen_time_limit_minutes);
    Ok(())
}
```

## Features

- Screen time management
- Content filtering
- Activity monitoring
- Age-appropriate controls
- Real-time alerts

## Philosophy

弘益人間 - Protecting children online benefits all of humanity.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
