//! Five equal representations
//!
//! Visual, Auditory, Tactile, Spatial, Gestural
//!
//! **ALL FIVE ARE EQUAL. NO DEFAULT EXISTS.**

pub mod auditory;
pub mod gestural;
pub mod spatial;
pub mod tactile;
pub mod visual;

pub use auditory::AuditoryRep;
pub use gestural::GesturalRep;
pub use spatial::SpatialRep;
pub use tactile::TactileRep;
pub use visual::VisualRep;

use serde::{Deserialize, Serialize};

/// Five equal representations
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Representations {
    /// Visual representation (시각)
    pub visual: Option<VisualRep>,

    /// Auditory representation (청각)
    pub auditory: Option<AuditoryRep>,

    /// Tactile representation (촉각)
    pub tactile: Option<TactileRep>,

    /// Spatial representation (공간)
    pub spatial: Option<SpatialRep>,

    /// Gestural representation (제스처)
    pub gestural: Option<GesturalRep>,
}

impl Representations {
    /// Create a new empty representations set
    pub fn new() -> Self {
        Self::default()
    }

    /// Set visual representation
    pub fn with_visual(mut self, visual: VisualRep) -> Self {
        self.visual = Some(visual);
        self
    }

    /// Set auditory representation
    pub fn with_auditory(mut self, auditory: AuditoryRep) -> Self {
        self.auditory = Some(auditory);
        self
    }

    /// Set tactile representation
    pub fn with_tactile(mut self, tactile: TactileRep) -> Self {
        self.tactile = Some(tactile);
        self
    }

    /// Set spatial representation
    pub fn with_spatial(mut self, spatial: SpatialRep) -> Self {
        self.spatial = Some(spatial);
        self
    }

    /// Set gestural representation
    pub fn with_gestural(mut self, gestural: GesturalRep) -> Self {
        self.gestural = Some(gestural);
        self
    }
}
