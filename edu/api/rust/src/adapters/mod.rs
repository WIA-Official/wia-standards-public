//! WIA Education Adapters Module
//! Storage and external service adapters
//! 弘益人間 - Education for Everyone

mod storage;
mod simulator;

pub use storage::{ProfileStorage, FileStorage, MemoryStorage};
pub use simulator::LearnerSimulator;
