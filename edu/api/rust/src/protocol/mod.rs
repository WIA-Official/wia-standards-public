//! Communication Protocol Module
//! 弘益人間 - Education for Everyone
//!
//! Implements LTI 1.3 and xAPI protocols for educational accessibility
//! data exchange between learning management systems and tools.

pub mod lti;
pub mod xapi;
pub mod sync;
pub mod events;

// Re-export main types
pub use lti::{
    LtiAdapter, LtiMessage, LtiMessageType, LtiLaunchRequest,
    LtiDeepLinkingRequest, LtiDeepLinkingResponse, LtiResourceLink,
    AccessibilityClaims, LtiRole, LtiContext,
};

pub use xapi::{
    XapiStatement, XapiActor, XapiVerb, XapiObject, XapiResult,
    XapiContext, StatementGenerator, AccessibilityVerb,
};

pub use sync::{
    ProfileSyncManager, SyncResult, SyncConflict, ConflictResolution,
    ProfileVersion, SyncDirection,
};

pub use events::{
    AccessibilityEvent, EventType, EventStream, AdaptationRequest,
    AdaptationResponse, EventHandler,
};
