//! # WIA PubScript
//!
//! Multi-sensory publishing engine with equal representations.
//!
//! ## Philosophy
//!
//! This is NOT "technology for disabled people" - it's **"choices for all humans"**.
//!
//! ### Five Equal Representations
//!
//! - **Visual**: Text, images, layout, colors, typography
//! - **Auditory**: Speech synthesis, music, sound effects, spatial audio
//! - **Tactile**: Braille, haptic feedback, textures
//! - **Spatial**: 3D positioning, AR/VR, navigation
//! - **Gestural**: Swipe, tap, voice commands, eye tracking
//!
//! **ALL FIVE ARE EQUAL. NO DEFAULT EXISTS.**

pub mod braille;
pub mod ir;
pub mod parser;
pub mod renderer;
pub mod timeline;
pub mod plugin;
pub mod gesture;

#[cfg(feature = "wasm")]
pub mod wasm;

#[cfg(feature = "python")]
pub mod python;

pub use ir::{
    document::{Metadata, PubScriptDocument, Timeline, TimelineEvent, TimelineEventType},
    metadata::{NodeMetadata, SemanticType},
    node::ContentNode,
    representations::{
        auditory::{AuditoryRep, SoundEffect, SoundEffectType, SpatialAudio, Vec3},
        gestural::{
            Gesture, GesturalRep, InteractionHint, InteractionHintType, SwipeDirection,
            VoiceCommand,
        },
        spatial::{BoundingBox, Quat, SpatialRelation, SpatialRelationType, SpatialRep},
        tactile::{HapticEvent, HapticPattern, TactileRep, Texture, TextureType},
        visual::{Color, FontWeight, ImageData, Layout, VisualRep, VisualStyle},
        Representations,
    },
};
