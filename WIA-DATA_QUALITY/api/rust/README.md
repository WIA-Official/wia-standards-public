# WIA-DATA_QUALITY Rust SDK

> 데이터 품질 관리 표준
>
> **철학**: 홍익인간 (弘益人間) - "널리 인간을 이롭게 하라"

Official Rust SDK for the WIA-DATA_QUALITY Standard.

## Features

- 📊 **Quality Assessment**: Comprehensive data quality analysis
- ✅ **Multi-dimensional**: Completeness, accuracy, consistency, timeliness, validity
- 🔍 **Issue Detection**: Identify and categorize data quality issues
- 📈 **Metrics**: Detailed quality metrics and scoring
- 🛡️ **Type Safety**: Fully typed with Rust's strong type system

## Installation

```toml
[dependencies]
wia-data-quality = "1.0.0"
```

## Quick Start

```rust
use wia_data_quality::DataQualityClient;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let client = DataQualityClient::new("your-api-key".to_string());
    let report = client.assess_quality("dataset-id").await?;
    println!("Overall score: {:.2}", report.overall_score);
    Ok(())
}
```

## License

MIT License

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
