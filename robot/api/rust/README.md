# WIA Robot SDK (Rust)

A comprehensive Rust library for the WIA Robot Accessibility Standard, providing unified APIs for controlling assistive robots including exoskeletons, prosthetics, rehabilitation robots, care robots, surgical assistants, and mobility aids.

## Features

- **Type-safe robot control**: Strongly typed interfaces for each robot type
- **Safety first**: Built-in safety validation and emergency stop (E-Stop)
- **Real-time ready**: Designed for low-latency control loops
- **ROS2 compatible**: Data structures compatible with ROS/ROS2
- **WIA Robot Protocol (WRP)**: Standard communication protocol
- **Ecosystem Integration**: Export to RViz, Gazebo, FHIR, TensorFlow, and more

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-robot = "1.0"
```

## Quick Start

```rust
use wia_robot::prelude::*;

// Create an exoskeleton
let mut exo = ExoskeletonSpec::new_lower_body();
exo.assist_level = 0.75;

// Calculate walking speed
exo.gait.cadence_steps_min = 60.0;
exo.gait.stride_length_cm = 65.0;
let speed = exo.walking_speed_m_s();
println!("Walking speed: {:.2} m/s", speed);

// Check for fall risk
if exo.detect_fall_risk() {
    println!("Warning: Fall risk detected!");
}
```

## Robot Types

### Exoskeleton
```rust
use wia_robot::adapters::exoskeleton::*;

let exo = ExoskeletonSpec::new_lower_body();
```

### Prosthetics
```rust
use wia_robot::adapters::prosthetics::*;
use wia_robot::types::Side;

let hand = ProstheticSpec::new_hand(Side::Right);
```

### Rehabilitation Robot
```rust
use wia_robot::adapters::rehabilitation::*;

let rehab = RehabilitationSpec::new_upper_limb();
```

### Care Robot
```rust
use wia_robot::adapters::care::*;

let care = CareRobotSpec::new_elderly_companion();
```

### Surgical Robot
```rust
use wia_robot::adapters::surgical::*;

let surgical = SurgicalSpec::new_minimally_invasive();
```

### Mobility Aid
```rust
use wia_robot::adapters::mobility::*;

let wheelchair = MobilityAidSpec::new_wheelchair();
```

## Safety System

All robot operations should check safety status:

```rust
use wia_robot::safety::*;

let mut safety = SafetyStatus::new_safe();

// Validate before operation
if let Err(e) = safety.validate() {
    println!("Safety error: {}", e);
}

// Trigger emergency stop if needed
safety.trigger_estop(EStopSource::UserButton);
```

## WIA Robot Protocol (WRP)

Communication protocol for robot messaging:

```rust
use wia_robot::protocol::*;

// Create a telemetry message
let msg = MessageBuilder::telemetry()
    .from_device("exo-001", "exoskeleton")
    .to_device("server-001", "server")
    .payload(serde_json::json!({"status": "ok"}))
    .build();

// Create emergency stop message
let estop = MessageBuilder::emergency_stop()
    .from_device("exo-001", "exoskeleton")
    .build();
```

## Output Integration

Export robot data to external systems:

```rust
use wia_robot::output::*;

// Create output manager
let mut manager = OutputManager::new();

// Register exporters
manager.register("json", Box::new(JsonExporter::new("./data")));
manager.register("csv", Box::new(CsvExporter::new("./data")));
manager.register("fhir", Box::new(FhirExporter::new()));
manager.register("rviz", Box::new(RVizMarkerExporter::new()));

// Create output data
let data = OutputData::new("exo-001", "exoskeleton")
    .with_data(serde_json::json!({"status": "active"}));

// Export to specific adapter
let result = manager.output_to("json", &data).unwrap();
```

### Available Adapters

| Adapter | Type | Description |
|---------|------|-------------|
| `JsonExporter` | Export | JSON file export |
| `CsvExporter` | Export | CSV file export |
| `UrdfGenerator` | Export | URDF robot description |
| `RVizMarkerExporter` | Visualization | RViz2 markers |
| `GazeboSdfExporter` | Visualization | Gazebo SDF format |
| `FhirExporter` | Medical | HL7 FHIR resources |
| `DatasetExporter` | AI/ML | Training datasets |
| `DashboardAdapter` | Dashboard | WebSocket streaming |

## Module Structure

```
wia-robot/
├── adapters/        # Robot type adapters
│   ├── exoskeleton
│   ├── prosthetics
│   ├── rehabilitation
│   ├── care
│   ├── surgical
│   └── mobility
├── core/            # Control systems
│   ├── device       # Device management
│   └── control      # PID, trajectory
├── protocol/        # WRP communication
│   ├── message      # Message types
│   ├── builder      # Message builder
│   └── handler      # Protocol handler
├── transport/       # Transport layer
│   ├── base         # Transport trait
│   └── mock         # Mock transport
├── output/          # Output integration
│   ├── visualization  # RViz, Gazebo
│   ├── medical        # FHIR
│   ├── aiml           # ML datasets
│   ├── dashboard      # WebSocket
│   └── export         # JSON, CSV
├── safety/          # Safety system
├── types/           # Common types
└── error/           # Error handling
```

## WIA Standard Phases

| Phase | Status | Description |
|-------|--------|-------------|
| Phase 1 | ✅ Complete | Data Format Standards |
| Phase 2 | ✅ Complete | Rust API Implementation |
| Phase 3 | ✅ Complete | Communication Protocol |
| Phase 4 | ✅ Complete | Ecosystem Integration |

## License

MIT OR Apache-2.0

## Links

- [WIA Standards Repository](https://github.com/WIA-Official/wia-standards)
- [Documentation](https://docs.rs/wia-robot)

---

**弘益人間 - Benefit All Humanity**
