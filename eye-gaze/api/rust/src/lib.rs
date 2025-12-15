//! WIA Eye Gaze Interoperability Protocol SDK
//!
//! Standard interface for eye tracking devices and gaze-aware applications.
//!
//! # Example
//!
//! ```rust,no_run
//! use wia_eye_gaze::{EyeTracker, MockAdapter, DwellController};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Create tracker with mock adapter
//!     let adapter = MockAdapter::new(60);
//!     let mut tracker = EyeTracker::new(adapter);
//!
//!     // Connect and calibrate
//!     tracker.connect().await?;
//!     tracker.start_calibration(None).await?;
//!
//!     // Subscribe to gaze data
//!     tracker.subscribe(|gaze| {
//!         println!("Gaze: ({:.2}, {:.2})", gaze.x, gaze.y);
//!     });
//!
//!     tracker.start_tracking();
//!     Ok(())
//! }
//! ```
//!
//! 弘益人間 (홍익인간) - 널리 인간을 이롭게

pub mod types;
pub mod tracker;
pub mod adapters;
pub mod dwell;
pub mod app;

// Phase 3: Real-time Communication
pub mod binary;
#[cfg(feature = "server")]
pub mod server;
pub mod ipc;
pub mod discovery;

pub use types::*;
pub use tracker::{EyeTracker, EyeTrackerAdapter};
pub use adapters::MockAdapter;
pub use dwell::DwellController;
pub use app::GazeAwareApp;

// Phase 3 exports
pub use binary::{GazePointBinary, GazeEventBinary, encode_batch, decode_batch};
#[cfg(feature = "server")]
pub use server::{GazeServer, ServerConfig};
pub use ipc::{IpcClient, IpcMessage};
pub use discovery::{ServiceDiscovery, EyeTrackerService};

/// SDK Version
pub const VERSION: &str = "1.0.0-alpha.1";

/// Protocol Version
pub const PROTOCOL_VERSION: &str = "1.0.0";
