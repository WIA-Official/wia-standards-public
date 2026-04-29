# WIA-CHILD-002-age-verification Rust SDK

> **弘益人間 (Benefit All Humanity)**

Official Rust SDK for the WIA-CHILD-002-age-verification Standard.

## Philosophy

Creating a safer digital environment for children worldwide

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
wia-child-002-age-verification-sdk = "1.0"
tokio = { version = "1", features = ["full"] }
serde_json = "1.0"
```

## Quick Start

```rust
use wia_child_002_age_verification_sdk::{Child002AgeVerificationClient, Config};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize client
    let client = Child002AgeVerificationClient::new(Config {
        api_key: "your-api-key".to_string(),
        ..Default::default()
    });

    // Validate data
    let data = serde_json::json!({
        "field": "value"
    });

    let result = client.validate(&data).await?;
    println!("Valid: {}", result.valid);

    Ok(())
}
```

## Features

- ✅ **Type Safety** - Strong typing for all API interactions
- ✅ **Async/Await** - Built on Tokio for high performance
- ✅ **Data Validation** - Validate data against WIA-CHILD-002-age-verification standards
- ✅ **Error Handling** - Comprehensive error types
- ✅ **Documentation** - Fully documented API

## API Reference

### Client Initialization

```rust
use wia_child_002_age_verification_sdk::{Child002AgeVerificationClient, Config};

let client = Child002AgeVerificationClient::new(Config {
    api_key: "your-api-key".to_string(),
    base_url: None, // Uses default
    timeout_secs: 30,
});
```

### Operations

#### Validate Data

```rust
let validation = client.validate(&data).await?;
if !validation.valid {
    for error in validation.errors {
        println!("Error in {}: {}", error.field, error.message);
    }
}
```

#### Create Data

```rust
use wia_child_002_age_verification_sdk::CreateChild002AgeVerificationRequest;

let request = CreateChild002AgeVerificationRequest {
    content: serde_json::json!({"key": "value"}),
    metadata: None,
};

let created = client.create(request).await?;
println!("Created ID: {}", created.id);
```

#### Get Data

```rust
let data = client.get("data-id").await?;
println!("Content: {}", data.content);
```

#### List Data

```rust
use wia_child_002_age_verification_sdk::ListParams;

let params = ListParams {
    page: Some(0),
    limit: Some(10),
    ..Default::default()
};

let list = client.list(Some(params)).await?;
println!("Total: {}", list.total);
```

#### Delete Data

```rust
client.delete("data-id").await?;
```

## Examples

Run the basic usage example:

```bash
cargo run --example basic_usage
```

## Testing

Run tests:

```bash
cargo test
```

## Error Handling

The SDK uses a comprehensive `Error` enum:

```rust
use wia_child_002_age_verification_sdk::Error;

match client.get("id").await {
    Ok(data) => println!("Success: {:?}", data),
    Err(Error::NotFound(_)) => println!("Not found"),
    Err(Error::Authentication(_)) => println!("Auth failed"),
    Err(Error::RateLimited { retry_after }) => {
        println!("Rate limited, retry after {:?}", retry_after)
    }
    Err(e) => println!("Other error: {}", e),
}
```

## Domain

**child_safety**

## Standard Version

**1.0.0**

## License

MIT License - See [LICENSE](https://github.com/WIA-Official/wia-standards/blob/main/LICENSE)

## Support

- Documentation: https://docs.wiastandards.com/WIA-CHILD-002-age-verification
- Issues: https://github.com/WIA-Official/wia-standards/issues
- Website: https://wiastandards.com

---

**© 2025 WIA - World Certification Industry Association**

弘益人間 (Benefit All Humanity)
