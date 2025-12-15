//! WIA Exoskeleton API
//!
//! This crate provides the WIA standard API for rehabilitation exoskeletons,
//! including control interfaces, intent detection, and safety systems.
//!
//! # Modules
//!
//! - [`control`]: Exoskeleton control interfaces (position, velocity, torque, impedance)
//! - [`intent`]: User intent detection with sensor fusion (EMG, GRF, IMU)
//! - [`safety`]: Safety systems (E-Stop, joint limits, overload protection)
//!
//! # Example
//!
//! ```rust
//! use wia_exoskeleton::{
//!     control::{ExoController, JointType, Side, ImpedanceParams, AssistanceMode},
//!     intent::{IntentDetector, IntentDetectorConfig, UserIntent},
//!     safety::{SafetySystem, EStopTriggerType, EStopSource},
//! };
//!
//! // Create safety system
//! let mut safety = SafetySystem::new();
//!
//! // Create controller with 50% initial assistance
//! let mut controller = ExoController::new(50.0);
//!
//! // Set assistance mode
//! controller.set_assistance_mode(AssistanceMode::ActiveAssist);
//!
//! // Check safety before operation
//! if safety.get_status().safe_to_operate {
//!     // Apply impedance control to right knee
//!     let response = controller.impedance_control(
//!         JointType::Knee,
//!         Side::Right,
//!         ImpedanceParams {
//!             stiffness: 30.0,
//!             damping: 2.0,
//!             equilibrium_angle: 20.0,
//!             inertia: None,
//!         },
//!     );
//! }
//!
//! // Create intent detector
//! let config = IntentDetectorConfig::default();
//! let mut detector = IntentDetector::new(config);
//!
//! // Detect intent (after processing sensor data)
//! let detection = detector.detect_intent();
//! if detection.intent != UserIntent::Idle {
//!     println!("Detected intent: {:?}", detection.intent);
//! }
//!
//! // Periodic safety heartbeat
//! safety.heartbeat();
//! ```

pub mod control;
pub mod intent;
pub mod safety;

// Re-exports for convenience
pub use control::{
    AssistanceMode, ControlMode, ControlResponse, ExoController, ImpedanceParams, JointType,
    PositionControlParams, ResponseStatus, Side, TorqueControlParams, VelocityControlParams,
};

pub use intent::{
    IntentDetection, IntentDetector, IntentDetectorConfig, IntentSource, IntentState, UserIntent,
};

pub use safety::{
    SafetySystem, SafetyStatus,
    EmergencyStopSystem, EStopConfig, EStopEvent, EStopTriggerType, EStopSource, EStopStatus,
    JointLimitsSystem, JointLimits, LimitZone, EnforcementResult,
    OverloadDetector, OverloadConfig, OverloadType, OverloadLevel, OverloadStatus,
};

/// Crate version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
