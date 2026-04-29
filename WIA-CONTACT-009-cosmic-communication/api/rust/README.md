# WIA-CONTACT-009 Rust SDK

> 우주 통신 및 외계 접촉 프로토콜 표준
>
> **철학**: 홍익인간 (弘益人間) - "널리 인간을 이롭게 하라"

Official Rust SDK for the WIA-CONTACT-009 Cosmic Communication Standard.

## Features

- 📡 **Signal Reception**: Receive and process cosmic signals from various sources
- 🌌 **Universal Messaging**: Encode and decode messages for interstellar communication
- 🔬 **Signal Analysis**: Analyze signals for artificial origins and patterns
- 📊 **Coordinate Systems**: Support for equatorial and galactic coordinate systems
- 🛡️ **Type Safety**: Fully typed with Rust's strong type system
- ⚡ **Async/Await**: Built on Tokio for high-performance async operations

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-contact-009 = "1.0.0"
```

## Quick Start

```rust
use wia_contact_009::{CosmicClient, MessageType, EncodingFormat, UniversalMessage};
use chrono::Utc;
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize client
    let client = CosmicClient::new("your-api-key".to_string());

    // Receive cosmic signal
    let signal = client.receive_signal().await?;
    println!("Received signal: {:?}", signal.signal_type);

    // Analyze signal
    let analysis = client.analyze_signal(&signal.signal_id).await?;
    println!("Artificial probability: {:.2}%",
        analysis.artificial_probability * 100.0);

    // Send universal message
    let message = UniversalMessage {
        message_id: Uuid::new_v4().to_string(),
        message_type: MessageType::Greeting,
        content: "Hello Universe! 홍익인간".to_string(),
        encoding: EncodingFormat::Binary,
        protocol_version: "1.0".to_string(),
        created_at: Utc::now(),
    };

    let message_id = client.send_message(&message).await?;
    println!("Message sent: {}", message_id);

    Ok(())
}
```

## Core Types

### SignalType
```rust
pub enum SignalType {
    Radio,        // Radio frequency signals
    Optical,      // Optical/laser signals
    Neutrino,     // Neutrino-based signals
    Gravitational, // Gravitational wave patterns
    Unknown,      // Unknown signal type
}
```

### CosmicSignal
```rust
pub struct CosmicSignal {
    pub signal_id: String,
    pub signal_type: SignalType,
    pub source: CosmicCoordinate,
    pub frequency: f64,
    pub strength: f64,
    pub detected_at: DateTime<Utc>,
    pub raw_data: String,
    pub decoded_message: Option<String>,
    pub confidence: f64,
    pub metadata: SignalMetadata,
}
```

### UniversalMessage
```rust
pub struct UniversalMessage {
    pub message_id: String,
    pub message_type: MessageType,
    pub content: String,
    pub encoding: EncodingFormat,
    pub protocol_version: String,
    pub created_at: DateTime<Utc>,
}
```

## API Methods

### Signal Reception
```rust
// Receive latest signal
let signal = client.receive_signal().await?;

// List recent signals
let signals = client.list_signals(10).await?;
```

### Message Sending
```rust
let message_id = client.send_message(&message).await?;
```

### Signal Analysis
```rust
let analysis = client.analyze_signal(&signal_id).await?;
println!("Artificial probability: {:.2}%",
    analysis.artificial_probability * 100.0);
```

## Utility Functions

```rust
use wia_contact_009::utils::*;

// Convert frequency to wavelength
let wavelength = frequency_to_wavelength(1420e6); // Hydrogen line

// Calculate SNR
let snr = calculate_snr(signal_strength, noise_level);

// Convert coordinates
let (l, b) = equatorial_to_galactic(ra, dec);

// Distance conversions
let parsecs = light_years_to_parsecs(100.0);

// Drake equation
let n = drake_equation(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1000.0);

// Generate greeting
let greeting = generate_greeting();
```

## Validation

```rust
use wia_contact_009::validators::*;

// Validate signal
validate_signal(&signal)?;

// Validate coordinate
validate_coordinate(&coordinate)?;

// Validate message
validate_message(&message)?;

// Validate analysis
validate_analysis(&analysis)?;
```

## Examples

Run the basic usage example:

```bash
cargo run --example basic_usage
```

Set your API key:

```bash
export WIA_API_KEY=your-api-key
cargo run --example basic_usage
```

## Testing

Run the test suite:

```bash
cargo test
```

Run with output:

```bash
cargo test -- --nocapture
```

## Error Handling

```rust
use wia_contact_009::Error;

match client.receive_signal().await {
    Ok(signal) => println!("Success: {:?}", signal),
    Err(Error::NetworkError(e)) => eprintln!("Network error: {}", e),
    Err(Error::ValidationError(e)) => eprintln!("Validation error: {}", e),
    Err(e) => eprintln!("Other error: {}", e),
}
```

## Philosophy

**홍익인간 (弘益人間)** - "Benefit All Humanity"

This SDK embodies the principle of benefiting all humanity through:
- Open communication with potential cosmic civilizations
- Peaceful and scientific approach to extraterrestrial contact
- Sharing knowledge for the advancement of all intelligent life
- Preserving and respecting cosmic diversity

## License

MIT License - See LICENSE file for details

## Links

- [WIA Standard](https://github.com/WIA-Official/wia-standards)
- [Documentation](https://docs.wia.org/standards/contact-009)
- [API Reference](https://api.wia.org/docs/contact-009)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
