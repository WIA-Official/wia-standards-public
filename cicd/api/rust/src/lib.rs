//! wia-cicd — reference Rust SDK for the WIA-CICD open standard.
//!
//! 弘益人間 — Benefit All Humanity · MIT License
//!
//! The simulator at `/cicd/simulator/index.html` is the source of truth for
//! ENUMs and thresholds; this crate mirrors them as Rust constants.

#![forbid(unsafe_code)]
#![deny(missing_docs)]

pub mod pipeline;
pub mod sbom;
pub mod dora;

pub use pipeline::{Pipeline, Node, NodeType, Trigger, SecurityGate, PolicyMode, DeployStrategy, validate, topo_sort, ValidationError};
pub use sbom::{SbomFormat, SlsaLevel, SbomComponent, SbomMeta, emit_cyclonedx, emit_spdx_tag_value, meets_slsa_target};
pub use dora::{DoraMetrics, DoraVerdict, evaluate_dora, compose_cache_hit, error_budget_burn, DORA_ELITE_LEAD_TIME_HR, DORA_ELITE_FAILURE_RATE, REWORK_RATE_THRESHOLD};
